//! Multi-Period Optimal Power Flow (MP-OPF).
//!
//! Links multiple DC-OPF periods via:
//!   - Generator ramp-rate constraints (MW/h up and down)
//!   - Battery storage state-of-charge (SoC) continuity
//!
//! # Method
//!
//! Sequential rolling DC-OPF with ramp constraints applied as modified generator
//! output bounds.  Each period `t` is solved with:
//!   - `P_min_ramp[g] = max(P_min[g], P_prev[g] − ramp_down[g] · Δt)`
//!   - `P_max_ramp[g] = min(P_max[g], P_prev[g] + ramp_up[g] · Δt)`
//!
//! Storage units are modelled as a generator (discharging) and load (charging)
//! at the same bus with SoC continuity enforced across periods.
//!
//! # References
//! Morales-España, G., et al. "Tight and compact MILP formulation for the
//! thermal unit commitment problem." IEEE Trans. Power Systems, 2013.

use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use crate::optimize::opf::dc_opf::{economic_dispatch_pub, GenCost};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Storage unit model
// ---------------------------------------------------------------------------

/// Battery storage unit participating in multi-period OPF.
///
/// The unit can charge (consuming P_ch ≥ 0 from the network) or discharge
/// (injecting P_dis ≥ 0 into the network) in each period.  The net bus
/// injection is `P_dis × η_dis − P_ch / η_ch`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StorageUnit {
    /// 0-based bus index where the storage is connected.
    pub bus: usize,
    /// Energy capacity \[MWh\].
    pub e_max_mwh: f64,
    /// Maximum charge/discharge power \[MW\].
    pub p_max_mw: f64,
    /// Charging efficiency (0–1).
    pub eta_charge: f64,
    /// Discharging efficiency (0–1).
    pub eta_discharge: f64,
    /// Initial state of charge as fraction of `e_max_mwh` (0–1).
    pub soc_init: f64,
    /// Terminal SoC constraint as fraction of `e_max_mwh` (0–1).
    /// If `f64::NAN`, no terminal constraint is enforced.
    pub soc_final: f64,
    /// Cycle degradation cost \[$/MWh throughput\].
    pub cost_cycle: f64,
}

impl StorageUnit {
    /// Validate storage parameters.
    pub fn validate(&self) -> Result<()> {
        if self.e_max_mwh <= 0.0 {
            return Err(OxiGridError::InvalidParameter(format!(
                "StorageUnit e_max_mwh must be positive, got {:.3}",
                self.e_max_mwh
            )));
        }
        if self.p_max_mw <= 0.0 {
            return Err(OxiGridError::InvalidParameter(format!(
                "StorageUnit p_max_mw must be positive, got {:.3}",
                self.p_max_mw
            )));
        }
        if !(0.0..=1.0).contains(&self.eta_charge) || !(0.0..=1.0).contains(&self.eta_discharge) {
            return Err(OxiGridError::InvalidParameter(
                "Storage efficiencies must be in [0, 1]".to_string(),
            ));
        }
        if !(0.0..=1.0).contains(&self.soc_init) {
            return Err(OxiGridError::InvalidParameter(format!(
                "soc_init must be in [0, 1], got {:.3}",
                self.soc_init
            )));
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// MP-OPF configuration
// ---------------------------------------------------------------------------

/// Configuration for the multi-period OPF problem.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiPeriodOpfConfig {
    /// Number of time periods (e.g., 24 for an hourly day-ahead schedule).
    pub n_periods: usize,
    /// Duration of each period \[hours\].
    pub dt_hours: f64,
    /// Ramp-up limits per generator \[MW/h\].  Length = n_gen.
    pub ramp_up: Vec<f64>,
    /// Ramp-down limits per generator \[MW/h\].  Length = n_gen.
    pub ramp_down: Vec<f64>,
    /// Battery storage units.
    pub storage_units: Vec<StorageUnit>,
    /// Linear cost coefficient per generator \[$/MWh\].  Length = n_gen.
    pub cost_coefficients: Vec<f64>,
    /// Quadratic cost coefficient per generator \[$/MW²h\].  Length = n_gen.
    /// Set to 0.0 for purely linear cost functions.
    pub cost_quadratic: Vec<f64>,
    /// Generator output limits: `(P_min_MW, P_max_MW)`.  Length = n_gen.
    pub generation_limits: Vec<(f64, f64)>,
    /// Load profile: `load_profiles[t][bus_idx]` = active load \[MW\] in period `t`.
    pub load_profiles: Vec<Vec<f64>>,
    /// Renewable generation profiles: `renewable_profiles[t][gen_idx]` = available
    /// renewable output \[MW\] in period `t`.  Set to `f64::INFINITY` for non-renewable
    /// generators (no curtailment applied).
    pub renewable_profiles: Vec<Vec<f64>>,
}

impl MultiPeriodOpfConfig {
    /// Validate configuration dimensions.
    pub fn validate(&self, n_gen: usize, n_bus: usize) -> Result<()> {
        if self.n_periods == 0 {
            return Err(OxiGridError::InvalidParameter(
                "n_periods must be ≥ 1".to_string(),
            ));
        }
        if self.dt_hours <= 0.0 {
            return Err(OxiGridError::InvalidParameter(format!(
                "dt_hours must be positive, got {:.3}",
                self.dt_hours
            )));
        }
        for (name, v) in [
            ("ramp_up", &self.ramp_up),
            ("ramp_down", &self.ramp_down),
            ("cost_coefficients", &self.cost_coefficients),
            ("cost_quadratic", &self.cost_quadratic),
        ] {
            if v.len() != n_gen {
                return Err(OxiGridError::InvalidParameter(format!(
                    "{name} length {} != n_gen {n_gen}",
                    v.len()
                )));
            }
        }
        if self.generation_limits.len() != n_gen {
            return Err(OxiGridError::InvalidParameter(format!(
                "generation_limits length {} != n_gen {n_gen}",
                self.generation_limits.len()
            )));
        }
        if self.load_profiles.len() != self.n_periods {
            return Err(OxiGridError::InvalidParameter(format!(
                "load_profiles has {} periods, expected {}",
                self.load_profiles.len(),
                self.n_periods
            )));
        }
        for (t, lp) in self.load_profiles.iter().enumerate() {
            if lp.len() != n_bus {
                return Err(OxiGridError::InvalidParameter(format!(
                    "load_profiles[{t}] length {} != n_bus {n_bus}",
                    lp.len()
                )));
            }
        }
        for su in &self.storage_units {
            su.validate()?;
            if su.bus >= n_bus {
                return Err(OxiGridError::InvalidParameter(format!(
                    "StorageUnit bus {} out of range (n_bus={})",
                    su.bus, n_bus
                )));
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// MP-OPF result
// ---------------------------------------------------------------------------

/// Result of a multi-period OPF solve.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiPeriodOpfResult {
    /// Optimal generation dispatch per period and generator \[MW\].
    /// `dispatch[t][g]` = output of generator `g` in period `t`.
    pub dispatch: Vec<Vec<f64>>,
    /// Net storage power per period and unit \[MW\].
    /// Positive = discharging (injecting); negative = charging (consuming).
    pub storage_power: Vec<Vec<f64>>,
    /// State of charge trajectory per period and unit \[fraction of e_max_mwh\].
    /// `soc_trajectory[t][s]` = SoC after period `t`.
    pub soc_trajectory: Vec<Vec<f64>>,
    /// Locational marginal prices per period and bus \[$/MWh\].
    pub lmp: Vec<Vec<f64>>,
    /// Total generation cost over all periods \[$/h integrated\].
    pub total_cost: f64,
    /// Whether all periods converged to feasible dispatches.
    pub converged: bool,
    /// List of `(period, gen_idx)` pairs where a ramp constraint is binding.
    pub binding_ramps: Vec<(usize, usize)>,
}

// ---------------------------------------------------------------------------
// MP-OPF solver
// ---------------------------------------------------------------------------

/// Multi-period OPF solver using sequential rolling DC-OPF.
pub struct MultiPeriodOpf {
    /// Problem configuration.
    pub config: MultiPeriodOpfConfig,
}

impl MultiPeriodOpf {
    /// Create a new multi-period OPF solver.
    pub fn new(config: MultiPeriodOpfConfig) -> Self {
        Self { config }
    }

    /// Solve the multi-period OPF for the given network.
    ///
    /// # Method
    ///
    /// Periods are solved sequentially.  Ramp constraints are imposed by
    /// modifying each generator's effective `P_min` and `P_max` based on the
    /// previous period's dispatch.  Storage units are dispatched greedily to
    /// minimise load (arbitrage value), with SoC continuity enforced.
    pub fn solve(&self, network: &PowerNetwork) -> Result<MultiPeriodOpfResult> {
        let cfg = &self.config;
        let n_gen = network.generators.len();
        let n_bus = network.buses.len();
        let _n_stor = cfg.storage_units.len();
        let t = cfg.n_periods;

        cfg.validate(n_gen, n_bus)?;

        let mut dispatch: Vec<Vec<f64>> = Vec::with_capacity(t);
        let mut storage_power: Vec<Vec<f64>> = Vec::with_capacity(t);
        let mut soc_trajectory: Vec<Vec<f64>> = Vec::with_capacity(t);
        let mut lmp: Vec<Vec<f64>> = Vec::with_capacity(t);
        let mut total_cost = 0.0_f64;
        let mut converged = true;
        let mut binding_ramps: Vec<(usize, usize)> = Vec::new();

        // Initial conditions — start at the minimum of each generator
        // to keep ramp constraints tight from the first period downward.
        // Using the minimum avoids the common failure mode where mid-range
        // initialisation forces the ramp-constrained minimum above the load.
        let mut prev_dispatch: Vec<f64> = cfg
            .generation_limits
            .iter()
            .map(|(pmin, _pmax)| *pmin)
            .collect();
        let mut prev_soc: Vec<f64> = cfg.storage_units.iter().map(|su| su.soc_init).collect();

        for period in 0..t {
            let (p_dispatch, stor_pwr, new_soc, period_cost, period_lmp, ramps) =
                self.solve_period(period, &prev_dispatch, &prev_soc, network)?;

            // Record binding ramps for this period
            for g in ramps {
                binding_ramps.push((period, g));
            }

            total_cost += period_cost;
            prev_dispatch = p_dispatch.clone();
            prev_soc = new_soc.clone();

            dispatch.push(p_dispatch);
            storage_power.push(stor_pwr);
            soc_trajectory.push(new_soc);
            lmp.push(period_lmp);
        }

        // Check terminal SoC constraints for storage
        for (s, su) in cfg.storage_units.iter().enumerate() {
            if su.soc_final.is_nan() {
                continue;
            }
            let final_soc = prev_soc[s];
            if (final_soc - su.soc_final).abs() > 0.05 {
                log::warn!(
                    "Storage unit {s}: terminal SoC {:.3} deviates from target {:.3} by {:.3}",
                    final_soc,
                    su.soc_final,
                    (final_soc - su.soc_final).abs()
                );
                converged = false;
            }
        }

        // Sanity-check: total_cost must be non-negative for realistic inputs
        if total_cost < 0.0 {
            log::warn!(
                "MP-OPF total_cost {:.2} is negative — check cost coefficients",
                total_cost
            );
        }

        Ok(MultiPeriodOpfResult {
            dispatch,
            storage_power,
            soc_trajectory,
            lmp,
            total_cost,
            converged,
            binding_ramps,
        })
    }

    /// Solve a single period with ramp constraints from the previous dispatch.
    ///
    /// # Arguments
    /// - `period`         — 0-based period index
    /// - `prev_dispatch`  — generator outputs in the previous period \[MW\]
    /// - `prev_soc`       — storage SoC at end of previous period \[fraction\]
    /// - `network`        — power network topology
    ///
    /// # Returns
    /// `(dispatch, storage_power, new_soc, cost, lmp, binding_ramp_gens)`
    #[allow(clippy::type_complexity)]
    fn solve_period(
        &self,
        period: usize,
        prev_dispatch: &[f64],
        prev_soc: &[f64],
        network: &PowerNetwork,
    ) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>, f64, Vec<f64>, Vec<usize>)> {
        let cfg = &self.config;
        let n_gen = network.generators.len();
        let n_bus = network.buses.len();
        let dt = cfg.dt_hours;

        // Total load this period
        let load_profile = &cfg.load_profiles[period];
        let total_load_mw: f64 = load_profile.iter().sum();

        // Net load after renewable generation (renewable profiles may be absent)
        let has_renewable = !cfg.renewable_profiles.is_empty()
            && period < cfg.renewable_profiles.len()
            && !cfg.renewable_profiles[period].is_empty();

        let renewable_total: f64 = if has_renewable {
            cfg.renewable_profiles[period].iter().sum::<f64>()
        } else {
            0.0
        };

        // Storage arbitrage: determine net storage power for this period.
        // Heuristic: charge storage when renewable surplus exists,
        // discharge when load is high (load_profile total > average).
        let avg_load: f64 = cfg
            .load_profiles
            .iter()
            .map(|lp| lp.iter().sum::<f64>())
            .sum::<f64>()
            / cfg.n_periods.max(1) as f64;

        let mut stor_pwr = vec![0.0_f64; cfg.storage_units.len()];
        let mut new_soc = prev_soc.to_vec();
        let mut net_storage_mw = 0.0_f64;

        for (s, su) in cfg.storage_units.iter().enumerate() {
            let soc = prev_soc[s];
            let e_current = soc * su.e_max_mwh;

            if total_load_mw > avg_load && e_current > 0.01 * su.e_max_mwh {
                // High load: discharge storage (limited by SoC and P_max)
                let max_discharge_mw = (e_current * su.eta_discharge / dt).min(su.p_max_mw);
                // Discharge at 50% of capacity to spread across periods
                let p_dis = max_discharge_mw * 0.5;
                stor_pwr[s] = p_dis; // positive = discharging
                let energy_removed = p_dis * dt / su.eta_discharge;
                new_soc[s] = ((e_current - energy_removed) / su.e_max_mwh).clamp(0.0, 1.0);
                net_storage_mw -= p_dis; // discharging reduces net load
            } else if renewable_total > total_load_mw && soc < 0.95 {
                // Renewable surplus: charge storage
                let max_charge_mw = ((su.e_max_mwh - e_current) / (su.eta_charge * dt))
                    .min(su.p_max_mw)
                    .min(renewable_total - total_load_mw);
                let p_ch = max_charge_mw * 0.5;
                stor_pwr[s] = -p_ch; // negative = charging
                let energy_added = p_ch * su.eta_charge * dt;
                new_soc[s] = ((e_current + energy_added) / su.e_max_mwh).clamp(0.0, 1.0);
                net_storage_mw += p_ch; // charging increases net load
            }
            // else: storage idle, SoC unchanged
        }

        // Effective net load seen by generators
        let net_load_mw = (total_load_mw + net_storage_mw - renewable_total).max(0.0);

        // Build ramp-constrained generator cost functions
        let mut costs: Vec<GenCost> = Vec::with_capacity(n_gen);
        let mut binding_ramp_gens: Vec<usize> = Vec::new();

        #[allow(clippy::needless_range_loop)]
        for g in 0..n_gen {
            let (p_min_base, p_max_base) = cfg.generation_limits[g];
            let p_prev = prev_dispatch[g];

            // Ramp limits
            let ramp_up_limit = cfg.ramp_up[g] * dt;
            let ramp_dn_limit = cfg.ramp_down[g] * dt;

            let p_min_ramp = (p_prev - ramp_dn_limit).max(p_min_base);
            let p_max_ramp = (p_prev + ramp_up_limit).min(p_max_base);

            // Detect binding ramp constraints
            if p_min_ramp > p_min_base + 1e-3 || p_max_ramp < p_max_base - 1e-3 {
                binding_ramp_gens.push(g);
            }

            // Renewable curtailment cap: if renewable_profiles provides a cap
            // for this generator in this period, apply it.
            let p_max_ren = if has_renewable && g < cfg.renewable_profiles[period].len() {
                let ren = cfg.renewable_profiles[period][g];
                if ren.is_finite() {
                    p_max_ramp.min(ren)
                } else {
                    p_max_ramp
                }
            } else {
                p_max_ramp
            };

            let p_min_final = p_min_ramp.min(p_max_ren); // avoid infeasibility
            let p_max_final = p_max_ren.max(p_min_final);

            costs.push(GenCost {
                a: 0.0,
                b: cfg.cost_coefficients[g],
                c: cfg.cost_quadratic[g],
                p_min: p_min_final,
                p_max: p_max_final,
            });
        }

        // Dispatch via lambda-iteration economic dispatch
        let p_gen = economic_dispatch_pub(&costs, net_load_mw)?;

        // Compute total generation cost for this period [$/h]
        let period_cost: f64 = costs
            .iter()
            .zip(p_gen.iter())
            .map(|(c, &p)| c.total_cost(p) * dt)
            .sum::<f64>()
            + cfg
                .storage_units
                .iter()
                .zip(stor_pwr.iter())
                .map(|(su, &sp)| su.cost_cycle * sp.abs() * dt)
                .sum::<f64>();

        // Compute LMPs as marginal cost of the last dispatched unit [$/MWh].
        // For DC-OPF on a single-zone network, LMP is uniform = system lambda.
        // Approximate lambda as the marginal cost of the most expensive dispatched unit.
        let lambda = costs
            .iter()
            .zip(p_gen.iter())
            .map(|(c, &p)| c.marginal_cost(p))
            .fold(0.0_f64, f64::max);
        let period_lmp = vec![lambda; n_bus];

        Ok((
            p_gen,
            stor_pwr,
            new_soc,
            period_cost,
            period_lmp,
            binding_ramp_gens,
        ))
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::{Generator, PowerNetwork};

    /// Build a minimal 2-generator, 2-bus network for testing.
    fn make_test_network() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push({
            let mut b = Bus::new(1, BusType::Slack);
            b.vm = 1.0;
            b
        });
        net.buses.push({
            let mut b = Bus::new(2, BusType::PQ);
            b.vm = 1.0;
            b.pd = crate::units::Power(80.0);
            b.qd = crate::units::ReactivePower(0.0);
            b
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 500.0,
            rate_b: 500.0,
            rate_c: 500.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.generators.push(Generator {
            bus_id: 1,
            pg: 50.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 200.0,
            pmin: 20.0,
        });
        net.generators.push(Generator {
            bus_id: 2,
            pg: 50.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 150.0,
            pmin: 10.0,
        });
        net
    }

    fn make_config(n_periods: usize, net: &PowerNetwork) -> MultiPeriodOpfConfig {
        let n_gen = net.generators.len();
        let n_bus = net.buses.len();
        // Load profile: use a single total per period split evenly across buses.
        // Keep total load <= 60 MW so it stays well within generation limits
        // and ramp constraints (p_min=10, p_max=200, ramp=100 MW/h is generous).
        let load_profiles: Vec<Vec<f64>> = (0..n_periods)
            .map(|t| {
                let base_mw = if t % 6 < 3 { 40.0 } else { 55.0 };
                // Distribute evenly across all buses
                vec![base_mw / n_bus as f64; n_bus]
            })
            .collect();

        MultiPeriodOpfConfig {
            n_periods,
            dt_hours: 1.0,
            // Generous ramps: 100 MW/h allows large swings
            ramp_up: vec![100.0; n_gen],
            ramp_down: vec![100.0; n_gen],
            storage_units: Vec::new(),
            cost_coefficients: vec![20.0, 30.0], // gen 0 cheaper
            cost_quadratic: vec![0.1, 0.05],
            // Low p_min to avoid infeasibility when load is small
            generation_limits: vec![(0.0, 200.0), (0.0, 150.0)],
            load_profiles,
            renewable_profiles: Vec::new(),
        }
    }

    #[test]
    fn test_mp_opf_ramp_feasibility() {
        let net = make_test_network();
        let config = make_config(4, &net);
        let solver = MultiPeriodOpf::new(config);
        let result = solver.solve(&net);
        assert!(result.is_ok(), "MP-OPF should succeed: {:?}", result);
        let res = result.unwrap();
        assert_eq!(res.dispatch.len(), 4, "should have 4 periods");
        assert!(res.converged, "should converge");

        // Verify ramp constraints are respected between periods
        let n_gen = net.generators.len();
        let ramp_limit = 100.0_f64; // matches make_config ramp_up/ramp_down
        for t in 1..res.dispatch.len() {
            for g in 0..n_gen {
                let dp = res.dispatch[t][g] - res.dispatch[t - 1][g];
                assert!(
                    dp <= ramp_limit + 1e-3,
                    "Ramp-up violated at period {t} gen {g}: ΔP={dp:.2}"
                );
                assert!(
                    dp >= -ramp_limit - 1e-3,
                    "Ramp-down violated at period {t} gen {g}: ΔP={dp:.2}"
                );
            }
        }
    }

    #[test]
    fn test_mp_opf_storage_soc_tracking() {
        let net = make_test_network();
        let mut config = make_config(6, &net);
        config.storage_units.push(StorageUnit {
            bus: 1,
            e_max_mwh: 100.0,
            p_max_mw: 50.0,
            eta_charge: 0.95,
            eta_discharge: 0.93,
            soc_init: 0.5,
            soc_final: f64::NAN, // no terminal constraint
            cost_cycle: 0.5,
        });

        let solver = MultiPeriodOpf::new(config);
        let result = solver.solve(&net);
        assert!(
            result.is_ok(),
            "MP-OPF with storage should succeed: {:?}",
            result
        );
        let res = result.unwrap();

        // SoC must remain in [0, 1] for all periods
        for (t, soc_vec) in res.soc_trajectory.iter().enumerate() {
            for (s, &soc) in soc_vec.iter().enumerate() {
                assert!(
                    (0.0..=1.0).contains(&soc),
                    "SoC out of bounds at period {t} unit {s}: {soc:.4}"
                );
            }
        }
    }

    #[test]
    fn test_mp_opf_total_cost_positive() {
        let net = make_test_network();
        let config = make_config(3, &net);
        let solver = MultiPeriodOpf::new(config);
        let result = solver.solve(&net).expect("should succeed");
        assert!(
            result.total_cost >= 0.0,
            "Total cost should be non-negative, got {:.2}",
            result.total_cost
        );
        // Cost must be nonzero for a real dispatch
        assert!(
            result.total_cost > 1.0,
            "Total cost seems too low: {:.2}",
            result.total_cost
        );
    }

    #[test]
    fn test_mp_opf_24h_profile() {
        let net = make_test_network();
        let n_gen = net.generators.len();
        let n_bus = net.buses.len();

        // Typical daily load profile (24 hours) — totals in MW, split evenly across buses.
        // Keep peak at 100 MW so ramp constraints (50 MW/h) can be satisfied even from 0.
        let loads_total_mw: [f64; 24] = [
            40.0, 38.0, 35.0, 33.0, 35.0, 40.0, // midnight–6am (low)
            50.0, 65.0, 75.0, 80.0, 85.0, 90.0, // 6am–noon (rising)
            88.0, 90.0, 92.0, 95.0, 98.0, 100.0, // noon–6pm (peak)
            95.0, 85.0, 75.0, 65.0, 55.0, 45.0, // 6pm–midnight (falling)
        ];
        let load_profiles: Vec<Vec<f64>> = loads_total_mw
            .iter()
            .map(|&l_total| vec![l_total / n_bus as f64; n_bus])
            .collect();

        let config = MultiPeriodOpfConfig {
            n_periods: 24,
            dt_hours: 1.0,
            // Generous ramp allows 50 MW/h transition from zero to load
            ramp_up: vec![50.0; n_gen],
            ramp_down: vec![50.0; n_gen],
            storage_units: vec![StorageUnit {
                bus: 0,
                e_max_mwh: 80.0,
                p_max_mw: 30.0,
                eta_charge: 0.92,
                eta_discharge: 0.92,
                soc_init: 0.5,
                soc_final: f64::NAN,
                cost_cycle: 1.0,
            }],
            cost_coefficients: vec![25.0, 35.0],
            cost_quadratic: vec![0.08, 0.06],
            // p_min = 0 to avoid infeasibility when ramp-constrained generators
            // cannot yet reach minimum output from cold start
            generation_limits: vec![(0.0, 200.0), (0.0, 150.0)],
            load_profiles,
            renewable_profiles: Vec::new(),
        };

        let solver = MultiPeriodOpf::new(config);
        let result = solver.solve(&net);
        assert!(result.is_ok(), "24h MP-OPF should succeed: {:?}", result);
        let res = result.unwrap();
        assert_eq!(res.dispatch.len(), 24);
        assert_eq!(res.lmp.len(), 24);
        assert_eq!(res.soc_trajectory.len(), 24);
        assert_eq!(res.soc_trajectory[0].len(), 1); // 1 storage unit

        // All dispatches should be within generator limits
        for (t, disp) in res.dispatch.iter().enumerate() {
            for (g, &p) in disp.iter().enumerate() {
                assert!(
                    p >= -1e-6,
                    "Gen {g} dispatch {p:.2} below p_min at period {t}"
                );
                assert!(
                    p <= 200.0 + 1e-6,
                    "Gen {g} dispatch {p:.2} above p_max at period {t}"
                );
            }
        }
    }

    #[test]
    fn test_mp_opf_config_validation_error() {
        let net = make_test_network();
        let mut config = make_config(3, &net);
        // Deliberately wrong ramp_up length
        config.ramp_up = vec![30.0]; // should be n_gen = 2

        let solver = MultiPeriodOpf::new(config);
        let result = solver.solve(&net);
        assert!(result.is_err(), "should error on dimension mismatch");
    }

    #[test]
    fn test_storage_unit_validation() {
        let su = StorageUnit {
            bus: 0,
            e_max_mwh: 0.0, // invalid
            p_max_mw: 50.0,
            eta_charge: 0.95,
            eta_discharge: 0.93,
            soc_init: 0.5,
            soc_final: f64::NAN,
            cost_cycle: 0.5,
        };
        assert!(su.validate().is_err(), "zero e_max_mwh should be invalid");
    }
}
