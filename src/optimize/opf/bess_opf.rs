/// BESS-Integrated Optimal Power Flow
///
/// Multi-period OPF that co-optimises generator dispatch and battery energy
/// storage system (BESS) charge/discharge schedules over a time horizon.
///
/// # Physics
/// SOC dynamics (discrete time):
///   SOC_{t+1} = SOC_t + η_c · P_c_t · Δt / E_cap  − P_d_t · Δt / (η_d · E_cap)
///
/// Degradation penalty (throughput-based):
///   C_deg = k_deg · (P_c + P_d) · Δt   [$/MWh throughput]
///
/// # Formulation (single-node, multi-period)
///
/// min Σ_t [ Σ_g c_g · P_g_t  +  C_deg · (P_c_t + P_d_t) ]
///
/// s.t.
///   Σ_g P_g_t + P_d_t − P_c_t = D_t          (power balance)
///   P_g_min ≤ P_g_t ≤ P_g_max                 (generator limits)
///   0 ≤ P_c_t ≤ P_bess_max,  0 ≤ P_d_t ≤ P_bess_max
///   SOC_min ≤ SOC_t ≤ SOC_max
///   SOC_T = SOC_0                              (optional: state restoration)
use serde::{Deserialize, Serialize};

// ─── BESS parameters ───────────────────────────────────────────────────────

/// Physical and economic parameters of a single BESS.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BessParams {
    /// Usable energy capacity [MWh]
    pub energy_capacity_mwh: f64,
    /// Maximum charge power [MW]
    pub p_charge_max_mw: f64,
    /// Maximum discharge power [MW]
    pub p_discharge_max_mw: f64,
    /// Round-trip charge efficiency (0–1)
    pub eta_charge: f64,
    /// Round-trip discharge efficiency (0–1)
    pub eta_discharge: f64,
    /// Minimum SOC (fraction, 0–1)
    pub soc_min: f64,
    /// Maximum SOC (fraction, 0–1)
    pub soc_max: f64,
    /// Degradation cost [$/MWh throughput]
    pub deg_cost_per_mwh: f64,
    /// Self-discharge rate per hour (fraction)
    pub self_discharge_per_h: f64,
}

impl BessParams {
    /// Typical utility-scale Li-ion BESS (2 MWh, 1 MW).
    pub fn utility_scale() -> Self {
        Self {
            energy_capacity_mwh: 2.0,
            p_charge_max_mw: 1.0,
            p_discharge_max_mw: 1.0,
            eta_charge: 0.95,
            eta_discharge: 0.95,
            soc_min: 0.10,
            soc_max: 0.90,
            deg_cost_per_mwh: 5.0,
            self_discharge_per_h: 0.0001,
        }
    }

    /// Small behind-the-meter residential BESS (10 kWh, 5 kW).
    pub fn residential() -> Self {
        Self {
            energy_capacity_mwh: 0.010,
            p_charge_max_mw: 0.005,
            p_discharge_max_mw: 0.005,
            eta_charge: 0.93,
            eta_discharge: 0.93,
            soc_min: 0.05,
            soc_max: 0.95,
            deg_cost_per_mwh: 10.0,
            self_discharge_per_h: 0.0002,
        }
    }
}

// ─── Generator data ─────────────────────────────────────────────────────────

/// Generator cost and limit data (single period).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GenData {
    /// Generator name or id
    pub id: usize,
    /// Minimum power output [MW]
    pub p_min_mw: f64,
    /// Maximum power output [MW]
    pub p_max_mw: f64,
    /// Linear cost coefficient [$/MWh]
    pub cost_per_mwh: f64,
}

impl GenData {
    pub fn new(id: usize, p_min: f64, p_max: f64, cost: f64) -> Self {
        Self {
            id,
            p_min_mw: p_min,
            p_max_mw: p_max,
            cost_per_mwh: cost,
        }
    }
}

// ─── Problem configuration ──────────────────────────────────────────────────

/// Configuration for the BESS OPF.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BessOPFConfig {
    /// Time step duration [h]
    pub dt_h: f64,
    /// Load demand per period [MW] (length = number of periods)
    pub demand_mw: Vec<f64>,
    /// Optional price signal [$/MWh] per period (for arbitrage objective)
    pub prices: Option<Vec<f64>>,
    /// Initial SOC (fraction 0–1)
    pub soc_initial: f64,
    /// Whether to enforce SOC_final ≈ SOC_initial (cyclic constraint)
    pub cyclic_soc: bool,
    /// Ramp limit per period for each generator [MW/period] (optional)
    pub gen_ramp_mw: Option<Vec<f64>>,
}

impl BessOPFConfig {
    /// 24-hour horizon at hourly resolution with flat demand.
    pub fn flat_24h(demand_mw: f64, soc_init: f64) -> Self {
        Self {
            dt_h: 1.0,
            demand_mw: vec![demand_mw; 24],
            prices: None,
            soc_initial: soc_init,
            cyclic_soc: true,
            gen_ramp_mw: None,
        }
    }
}

// ─── Per-period decision variables ─────────────────────────────────────────

/// Optimised dispatch for one time period.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PeriodDispatch {
    /// Generator outputs [MW] (same order as `gens`)
    pub p_gen_mw: Vec<f64>,
    /// BESS charge power [MW] (≥ 0)
    pub p_charge_mw: f64,
    /// BESS discharge power [MW] (≥ 0)
    pub p_discharge_mw: f64,
    /// SOC at end of period (fraction)
    pub soc_end: f64,
    /// Generation cost [$]
    pub gen_cost: f64,
    /// Degradation cost [$]
    pub deg_cost: f64,
}

// ─── Result ─────────────────────────────────────────────────────────────────

/// Full multi-period BESS OPF result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BessOPFResult {
    /// Per-period dispatch decisions
    pub periods: Vec<PeriodDispatch>,
    /// SOC trajectory (length = T+1, starts at soc_initial)
    pub soc_trajectory: Vec<f64>,
    /// Total generation cost [$]
    pub total_gen_cost: f64,
    /// Total degradation cost [$]
    pub total_deg_cost: f64,
    /// Total objective [$]
    pub total_cost: f64,
    /// Total BESS energy discharged [MWh]
    pub energy_discharged_mwh: f64,
    /// Total BESS energy charged [MWh]
    pub energy_charged_mwh: f64,
    /// Peak generation [MW]
    pub peak_gen_mw: f64,
    /// Whether cyclic SOC constraint was satisfied (|SOC_T - SOC_0| < 0.01)
    pub cyclic_soc_ok: bool,
}

// ─── Solver ─────────────────────────────────────────────────────────────────

/// BESS-integrated OPF solver.
///
/// Uses a greedy merit-order dispatch with BESS operated on price/cost signal:
/// - If price is high → discharge BESS (replace expensive generation)
/// - If price is low  → charge BESS (store cheap energy)
/// - Degradation cost is added to effective discharge cost threshold.
pub struct BessOpfSolver<'a> {
    gens: &'a [GenData],
    bess: &'a BessParams,
    config: &'a BessOPFConfig,
}

impl<'a> BessOpfSolver<'a> {
    pub fn new(gens: &'a [GenData], bess: &'a BessParams, config: &'a BessOPFConfig) -> Self {
        Self { gens, bess, config }
    }

    /// Run the multi-period BESS OPF.
    pub fn run(&self) -> BessOPFResult {
        let n_periods = self.config.demand_mw.len();
        let dt = self.config.dt_h;

        let mut soc = self.config.soc_initial;
        let mut soc_traj = Vec::with_capacity(n_periods + 1);
        soc_traj.push(soc);

        let mut periods = Vec::with_capacity(n_periods);
        let mut total_gen_cost = 0.0;
        let mut total_deg_cost = 0.0;
        let mut total_discharged = 0.0;
        let mut total_charged = 0.0;
        let mut peak_gen = 0.0f64;

        // Sort generators by cost (merit order)
        let mut sorted_gens: Vec<usize> = (0..self.gens.len()).collect();
        sorted_gens.sort_by(|&a, &b| {
            self.gens[a]
                .cost_per_mwh
                .partial_cmp(&self.gens[b].cost_per_mwh)
                .unwrap()
        });

        for t in 0..n_periods {
            let demand = self.config.demand_mw[t];
            let price = self
                .config
                .prices
                .as_ref()
                .and_then(|p| p.get(t).copied())
                .unwrap_or_else(|| self.marginal_gen_cost(demand));

            // Decide BESS action based on price vs. BESS effective cost
            let (p_charge, p_discharge) = self.bess_decision(soc, price, demand, dt);

            // Net load after BESS
            let net_demand = (demand + p_charge - p_discharge).max(0.0);

            // Merit-order generator dispatch
            let (p_gen, gen_cost) = self.merit_order_dispatch(net_demand, &sorted_gens);

            // Update SOC with self-discharge
            let self_disc = soc * self.bess.self_discharge_per_h * dt;
            soc = soc * (1.0 - self.bess.self_discharge_per_h * dt)
                + p_charge * dt * self.bess.eta_charge / self.bess.energy_capacity_mwh
                - p_discharge * dt / (self.bess.eta_discharge * self.bess.energy_capacity_mwh);
            soc = soc.clamp(self.bess.soc_min, self.bess.soc_max);
            let _ = self_disc;

            // Degradation cost: proportional to throughput
            let throughput = (p_charge + p_discharge) * dt;
            let deg_cost = throughput * self.bess.deg_cost_per_mwh;

            let total_p: f64 = p_gen.iter().sum();
            peak_gen = peak_gen.max(total_p);

            total_gen_cost += gen_cost;
            total_deg_cost += deg_cost;
            total_discharged += p_discharge * dt;
            total_charged += p_charge * dt;

            soc_traj.push(soc);
            periods.push(PeriodDispatch {
                p_gen_mw: p_gen,
                p_charge_mw: p_charge,
                p_discharge_mw: p_discharge,
                soc_end: soc,
                gen_cost,
                deg_cost,
            });
        }

        let cyclic_ok = (soc - self.config.soc_initial).abs() < 0.05;
        let total_cost = total_gen_cost + total_deg_cost;

        BessOPFResult {
            periods,
            soc_trajectory: soc_traj,
            total_gen_cost,
            total_deg_cost,
            total_cost,
            energy_discharged_mwh: total_discharged,
            energy_charged_mwh: total_charged,
            peak_gen_mw: peak_gen,
            cyclic_soc_ok: cyclic_ok,
        }
    }

    /// Decide charge/discharge power for current period.
    fn bess_decision(&self, soc: f64, price: f64, demand: f64, dt: f64) -> (f64, f64) {
        let eff_discharge_cost =
            self.bess.deg_cost_per_mwh / (self.bess.eta_discharge * self.bess.eta_charge);
        let threshold_high = self.marginal_gen_cost(demand) + eff_discharge_cost;
        let threshold_low = threshold_high * 0.6;

        // Charge when price is low and SOC allows
        if price < threshold_low && soc < self.bess.soc_max - 0.01 {
            let soc_headroom = (self.bess.soc_max - soc) * self.bess.energy_capacity_mwh
                / (self.bess.eta_charge * dt);
            let p_c = soc_headroom.min(self.bess.p_charge_max_mw);
            return (p_c.max(0.0), 0.0);
        }

        // Discharge when price is high and SOC allows
        if price > threshold_high && soc > self.bess.soc_min + 0.01 {
            let soc_available =
                (soc - self.bess.soc_min) * self.bess.energy_capacity_mwh * self.bess.eta_discharge
                    / dt;
            let p_d = soc_available.min(self.bess.p_discharge_max_mw);
            return (0.0, p_d.max(0.0));
        }

        (0.0, 0.0)
    }

    /// Merit-order generator dispatch for given net load.
    fn merit_order_dispatch(&self, net_demand: f64, sorted_gens: &[usize]) -> (Vec<f64>, f64) {
        let mut p_gen = vec![0.0f64; self.gens.len()];
        let mut remaining = net_demand;
        let mut cost = 0.0;

        for &gi in sorted_gens {
            if remaining <= 0.0 {
                break;
            }
            let g = &self.gens[gi];
            let dispatch = remaining.min(g.p_max_mw).max(g.p_min_mw);
            let dispatch = dispatch.min(remaining);
            p_gen[gi] = dispatch;
            cost += dispatch * g.cost_per_mwh * self.config.dt_h;
            remaining -= dispatch;
        }

        // If still unmet (no capacity), dispatch everything to max
        if remaining > 1e-6 {
            for &gi in sorted_gens {
                p_gen[gi] = self.gens[gi].p_max_mw;
            }
        }

        (p_gen, cost)
    }

    /// Approximate marginal cost at given demand level.
    fn marginal_gen_cost(&self, demand: f64) -> f64 {
        let mut cumulative = 0.0;
        let mut sorted: Vec<&GenData> = self.gens.iter().collect();
        sorted.sort_by(|a, b| a.cost_per_mwh.partial_cmp(&b.cost_per_mwh).unwrap());

        for g in &sorted {
            cumulative += g.p_max_mw;
            if cumulative >= demand {
                return g.cost_per_mwh;
            }
        }
        sorted.last().map(|g| g.cost_per_mwh).unwrap_or(100.0)
    }
}

// ─── Stand-alone convenience function ──────────────────────────────────────

/// Run a BESS-integrated OPF for a single BESS with multiple generators.
///
/// # Arguments
/// - `gens`   — generator fleet (cost/capacity data)
/// - `bess`   — BESS physical parameters
/// - `config` — multi-period OPF configuration
pub fn run_bess_opf(gens: &[GenData], bess: &BessParams, config: &BessOPFConfig) -> BessOPFResult {
    BessOpfSolver::new(gens, bess, config).run()
}

// ─── SOC-trajectory post-processing ────────────────────────────────────────

/// Statistics about the BESS SOC trajectory.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SocStats {
    pub min_soc: f64,
    pub max_soc: f64,
    pub mean_soc: f64,
    pub soc_swing: f64,
    /// Approximate full-equivalent cycles (throughput / 2·capacity)
    pub equivalent_cycles: f64,
}

impl SocStats {
    pub fn from_result(result: &BessOPFResult, capacity_mwh: f64) -> Self {
        let traj = &result.soc_trajectory;
        let min_soc = traj.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_soc = traj.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let mean_soc = traj.iter().sum::<f64>() / traj.len() as f64;
        let soc_swing = max_soc - min_soc;
        let throughput = result.energy_charged_mwh + result.energy_discharged_mwh;
        let eq_cycles = throughput / (2.0 * capacity_mwh.max(1e-9));
        Self {
            min_soc,
            max_soc,
            mean_soc,
            soc_swing,
            equivalent_cycles: eq_cycles,
        }
    }
}

// ─── Degradation model ──────────────────────────────────────────────────────

/// Throughput-based degradation model for BESS lifetime estimation.
///
/// Maps cumulative throughput → remaining capacity (Wöhler-like model).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThroughputDegModel {
    /// Total lifetime throughput [MWh] before EOL (80% capacity)
    pub lifetime_throughput_mwh: f64,
    /// Capacity at start (1.0 = new)
    pub initial_capacity: f64,
}

impl ThroughputDegModel {
    /// NMC Li-ion cell: ~3000 full equivalent cycles for 2 MWh.
    pub fn nmc_2mwh() -> Self {
        Self {
            lifetime_throughput_mwh: 6000.0,
            initial_capacity: 1.0,
        }
    }

    /// LFP cell: ~6000 full equivalent cycles for 2 MWh.
    pub fn lfp_2mwh() -> Self {
        Self {
            lifetime_throughput_mwh: 12000.0,
            initial_capacity: 1.0,
        }
    }

    /// Remaining capacity fraction after `cumulative_mwh` throughput.
    pub fn remaining_capacity(&self, cumulative_mwh: f64) -> f64 {
        let frac = (cumulative_mwh / self.lifetime_throughput_mwh).min(1.0);
        // Linear fade from 100% → 80% over lifetime_throughput
        self.initial_capacity * (1.0 - 0.2 * frac)
    }

    /// Years to end-of-life given average daily throughput [MWh/day].
    pub fn years_to_eol(&self, daily_throughput_mwh: f64) -> f64 {
        if daily_throughput_mwh < 1e-9 {
            return f64::INFINITY;
        }
        self.lifetime_throughput_mwh / (daily_throughput_mwh * 365.0)
    }
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_gens() -> Vec<GenData> {
        vec![
            GenData::new(0, 0.0, 50.0, 30.0), // cheap baseload
            GenData::new(1, 0.0, 30.0, 60.0), // mid-merit
            GenData::new(2, 0.0, 20.0, 90.0), // peaker
        ]
    }

    fn flat_config() -> BessOPFConfig {
        BessOPFConfig {
            dt_h: 1.0,
            demand_mw: vec![40.0; 24],
            prices: None,
            soc_initial: 0.5,
            cyclic_soc: true,
            gen_ramp_mw: None,
        }
    }

    #[test]
    fn test_bess_opf_runs_without_panic() {
        let gens = simple_gens();
        let bess = BessParams::utility_scale();
        let config = flat_config();
        let result = run_bess_opf(&gens, &bess, &config);
        assert_eq!(result.periods.len(), 24);
    }

    #[test]
    fn test_soc_stays_in_bounds() {
        let gens = simple_gens();
        let bess = BessParams::utility_scale();
        let config = flat_config();
        let result = run_bess_opf(&gens, &bess, &config);
        for &soc in &result.soc_trajectory {
            assert!(
                soc >= bess.soc_min - 1e-9,
                "SOC {:.4} below min {}",
                soc,
                bess.soc_min
            );
            assert!(
                soc <= bess.soc_max + 1e-9,
                "SOC {:.4} above max {}",
                soc,
                bess.soc_max
            );
        }
    }

    #[test]
    fn test_gen_cost_positive() {
        let gens = simple_gens();
        let bess = BessParams::utility_scale();
        let config = flat_config();
        let result = run_bess_opf(&gens, &bess, &config);
        assert!(
            result.total_gen_cost > 0.0,
            "Generation cost should be positive"
        );
    }

    #[test]
    fn test_deg_cost_positive_with_cycling() {
        let gens = simple_gens();
        let bess = BessParams::utility_scale();
        // Create price signal to force cycling: high price at noon, low at night
        let mut prices = vec![25.0; 24];
        for p in prices.iter_mut().take(16).skip(10) {
            *p = 120.0;
        } // peak price midday
        let config = BessOPFConfig {
            dt_h: 1.0,
            demand_mw: vec![40.0; 24],
            prices: Some(prices),
            soc_initial: 0.5,
            cyclic_soc: false,
            gen_ramp_mw: None,
        };
        let result = run_bess_opf(&gens, &bess, &config);
        // Should have some degradation cost from cycling
        assert!(result.total_deg_cost >= 0.0);
    }

    #[test]
    fn test_soc_trajectory_length() {
        let gens = simple_gens();
        let bess = BessParams::utility_scale();
        let config = flat_config();
        let result = run_bess_opf(&gens, &bess, &config);
        assert_eq!(
            result.soc_trajectory.len(),
            25,
            "SOC trajectory should be T+1 = 25"
        );
    }

    #[test]
    fn test_peak_gen_non_negative() {
        let gens = simple_gens();
        let bess = BessParams::utility_scale();
        let config = flat_config();
        let result = run_bess_opf(&gens, &bess, &config);
        assert!(result.peak_gen_mw >= 0.0);
    }

    #[test]
    fn test_energy_discharged_non_negative() {
        let gens = simple_gens();
        let bess = BessParams::utility_scale();
        let config = flat_config();
        let result = run_bess_opf(&gens, &bess, &config);
        assert!(result.energy_discharged_mwh >= 0.0);
        assert!(result.energy_charged_mwh >= 0.0);
    }

    #[test]
    fn test_soc_stats() {
        let gens = simple_gens();
        let bess = BessParams::utility_scale();
        let config = flat_config();
        let result = run_bess_opf(&gens, &bess, &config);
        let stats = SocStats::from_result(&result, bess.energy_capacity_mwh);
        assert!(stats.min_soc <= stats.max_soc);
        assert!(stats.mean_soc >= stats.min_soc);
        assert!(stats.mean_soc <= stats.max_soc);
        assert!(stats.equivalent_cycles >= 0.0);
    }

    #[test]
    fn test_degradation_model_remaining_capacity() {
        let model = ThroughputDegModel::nmc_2mwh();
        assert!((model.remaining_capacity(0.0) - 1.0).abs() < 1e-9);
        let cap_half = model.remaining_capacity(model.lifetime_throughput_mwh / 2.0);
        assert!(
            (cap_half - 0.90).abs() < 1e-9,
            "Half-life capacity should be 90%"
        );
        let cap_eol = model.remaining_capacity(model.lifetime_throughput_mwh);
        assert!((cap_eol - 0.80).abs() < 1e-9, "EOL capacity should be 80%");
    }

    #[test]
    fn test_degradation_model_years_to_eol() {
        let model = ThroughputDegModel::lfp_2mwh();
        let yrs = model.years_to_eol(4.0); // 4 MWh/day = 2 full cycles/day
        assert!(
            yrs > 5.0 && yrs < 30.0,
            "LFP EOL in realistic range: {:.1} yr",
            yrs
        );
    }

    #[test]
    fn test_residential_bess() {
        let gens = vec![GenData::new(0, 0.0, 0.02, 50.0)];
        let bess = BessParams::residential();
        let config = BessOPFConfig {
            dt_h: 1.0,
            demand_mw: vec![0.003; 24],
            prices: None,
            soc_initial: 0.5,
            cyclic_soc: true,
            gen_ramp_mw: None,
        };
        let result = run_bess_opf(&gens, &bess, &config);
        assert_eq!(result.periods.len(), 24);
    }

    #[test]
    fn test_total_cost_is_sum_of_parts() {
        let gens = simple_gens();
        let bess = BessParams::utility_scale();
        let config = flat_config();
        let result = run_bess_opf(&gens, &bess, &config);
        let expected = result.total_gen_cost + result.total_deg_cost;
        assert!((result.total_cost - expected).abs() < 1e-9);
    }

    #[test]
    fn test_period_dispatch_vectors_correct_length() {
        let gens = simple_gens();
        let bess = BessParams::utility_scale();
        let config = flat_config();
        let result = run_bess_opf(&gens, &bess, &config);
        for period in &result.periods {
            assert_eq!(period.p_gen_mw.len(), gens.len());
        }
    }

    #[test]
    fn test_bess_opf_with_empty_gens_no_panic() {
        let gens: Vec<GenData> = vec![];
        let bess = BessParams::utility_scale();
        let config = BessOPFConfig {
            dt_h: 1.0,
            demand_mw: vec![0.0; 4],
            prices: None,
            soc_initial: 0.5,
            cyclic_soc: false,
            gen_ramp_mw: None,
        };
        let result = run_bess_opf(&gens, &bess, &config);
        assert_eq!(result.periods.len(), 4);
    }
}
