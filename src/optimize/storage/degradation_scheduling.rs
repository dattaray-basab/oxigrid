//! Degradation-Aware Storage Scheduling.
//!
//! Models battery aging explicitly during dispatch optimisation, finding the
//! lifetime-optimal trade-off between revenue and battery longevity.
//!
//! # Physics models supported
//! - [`DegradationPhysics::Linear`]   — simple linear aging (cycle + calendar)
//! - [`DegradationPhysics::Rainflow`] — Wöhler-curve cycle-counting
//! - [`DegradationPhysics::Sei`]      — SEI layer growth (semi-empirical)
//! - [`DegradationPhysics::Combined`] — cycle model + calendar model together
//!
//! # References
//! - Xu, B. et al. (2018). *Factoring degradation costs into BESS dispatch*.
//!   IEEE Trans. Smart Grid.
//! - Schmalstieg, J. et al. (2014). *A holistic aging model for Li(NiMnCo)O₂
//!   based 18650 lithium-ion batteries*. J. Power Sources.
//! - Rainflow counting: ASTM E1049-85(2017).

use crate::error::OxiGridError;
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Physical degradation models
// ---------------------------------------------------------------------------

/// Physics model used to compute battery state-of-health (SoH) decay.
///
/// SoH = 1.0 (new) → 0.0 (fully degraded).  End-of-life is typically at 0.7–0.8.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DegradationPhysics {
    /// Simple linear aging: cycle + calendar degradation proportional to
    /// throughput and elapsed time respectively.
    Linear {
        /// SoH loss per equivalent full cycle \[fraction/EFC\].
        deg_per_full_cycle: f64,
        /// Calendar SoH loss per year \[fraction/year\].
        deg_per_year: f64,
    },

    /// Cycle-counting model based on the Wöhler (S-N) fatigue curve.
    ///
    /// Captures the non-linear dependence of cycle life on depth of discharge
    /// (DoD): shallow cycles are much less damaging than deep ones.
    Rainflow {
        /// Wöhler exponent β such that σ^β × N = const (typical 1.5–3.0).
        stress_exponent: f64,
        /// Reference DoD at which `ref_cycles` is measured \[%\].
        ref_depth_pct: f64,
        /// Number of cycles achievable at `ref_depth_pct` DoD.
        ref_cycles: f64,
    },

    /// Semi-empirical SEI (solid-electrolyte interphase) growth model.
    ///
    /// Capacity fade follows a square-root-of-time law, accelerated by
    /// elevated temperature via an Arrhenius factor.
    Sei {
        /// SEI growth rate k_sei \[fraction/√h\].
        growth_rate: f64,
        /// Multiplicative Arrhenius acceleration evaluated at operating
        /// temperature relative to reference (298 K).
        temperature_factor: f64,
    },

    /// Combined cycle + calendar degradation: the two sub-models are
    /// evaluated independently and their SoH losses summed.
    Combined {
        /// Model used to compute cycle-induced aging.
        cycle_params: Box<DegradationPhysics>,
        /// Model used to compute calendar-induced aging.
        calendar_params: Box<DegradationPhysics>,
    },
}

impl DegradationPhysics {
    /// Construct a `Rainflow` model with default parameters representative of
    /// a modern NMC cell.
    pub fn rainflow_default() -> Self {
        Self::Rainflow {
            stress_exponent: 2.0,
            ref_depth_pct: 80.0,
            ref_cycles: 3_000.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Asset definition
// ---------------------------------------------------------------------------

/// Battery energy storage asset with full physical and economic specification.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StorageAsset {
    /// Unique asset identifier.
    pub asset_id: String,
    /// Usable energy capacity \[MWh\].
    pub capacity_mwh: f64,
    /// Maximum charge / discharge power rating \[MW\].
    pub power_mw: f64,
    /// Round-trip efficiency η_rt = η_charge × η_discharge (0–1).
    pub round_trip_efficiency: f64,
    /// Minimum allowable state of charge (0–1).
    pub soc_min: f64,
    /// Maximum allowable state of charge (0–1).
    pub soc_max: f64,
    /// Current state of health: 1.0 = new, 0.7 = end-of-life threshold \[fraction\].
    pub current_soh: f64,
    /// Battery replacement cost used to price each unit of SoH loss \[$/MWh\].
    ///
    /// Typically: total_capex_$ / capacity_mwh.
    pub replacement_cost_per_mwh: f64,
    /// Physics model for degradation computation.
    pub degradation_physics: DegradationPhysics,
}

impl StorageAsset {
    /// One-way charge efficiency √η_rt.
    #[inline]
    pub fn eta_charge(&self) -> f64 {
        self.round_trip_efficiency.sqrt()
    }

    /// One-way discharge efficiency √η_rt.
    #[inline]
    pub fn eta_discharge(&self) -> f64 {
        self.round_trip_efficiency.sqrt()
    }
}

// ---------------------------------------------------------------------------
// Market opportunity
// ---------------------------------------------------------------------------

/// Market signals available in a single hour.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarketOpportunity {
    /// Hour index (0-based).
    pub hour: usize,
    /// Day-ahead / real-time energy price \[$/MWh\].
    pub price_per_mwh: f64,
    /// Spinning-reserve capacity payment \[$/MW-h\].
    pub reserve_price_per_mw_h: f64,
    /// Frequency-regulation capacity payment \[$/MW-h\].
    pub regulation_price_per_mw_h: f64,
    /// Available renewable generation for absorption (charge) \[MW\].
    pub renewable_generation_mw: f64,
}

// ---------------------------------------------------------------------------
// Scheduler configuration
// ---------------------------------------------------------------------------

/// Configuration for [`DegradationAwareScheduler`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DegradationSchedulerConfig {
    /// Planning horizon \[h\] (default 24).
    pub horizon_h: usize,
    /// Number of SoC bins used for DP state discretisation (default 50).
    pub soc_bins: usize,
    /// Weight on degradation cost in the objective (0 = ignore, 1 = maximise lifetime).
    pub degradation_weight: f64,
    /// Weight on revenue in the objective (complementary to `degradation_weight`).
    pub revenue_weight: f64,
    /// Minimum SoH before the asset is taken out of service (default 0.7).
    pub min_soh: f64,
    /// Operating temperature \[°C\] used in Arrhenius / SEI models.
    pub temperature_c: f64,
}

impl Default for DegradationSchedulerConfig {
    fn default() -> Self {
        Self {
            horizon_h: 24,
            soc_bins: 50,
            degradation_weight: 0.5,
            revenue_weight: 0.5,
            min_soh: 0.7,
            temperature_c: 25.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Output structs
// ---------------------------------------------------------------------------

/// Hour-by-hour dispatch schedule with full economic and degradation accounting.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DegScheduleResult {
    /// Power dispatch per hour \[MW\]: positive = charge, negative = discharge.
    pub dispatch_mw: Vec<f64>,
    /// State of charge at the end of each hour (0–1).
    pub soc_profile: Vec<f64>,
    /// Total gross revenue over the horizon \[$/horizon\].
    pub total_revenue: f64,
    /// Total degradation cost over the horizon \[$/horizon\].
    pub total_degradation_cost: f64,
    /// Net value = revenue − degradation cost \[$/horizon\].
    pub net_value: f64,
    /// Cumulative SoH loss over the horizon \[fraction\].
    pub cumulative_soh_loss: f64,
    /// Equivalent full cycles consumed during the horizon.
    pub efc: f64,
}

/// Battery health report projecting end-of-life under current operating regime.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DegradationReport {
    /// Current state of health \[fraction\].
    pub current_soh: f64,
    /// Projected years until SoH falls to `min_soh` at current rate.
    pub predicted_eol_years: f64,
    /// Average cycle-induced SoH loss per day at current operating rate \[fraction/day\].
    pub cycle_deg_per_day: f64,
    /// Average calendar-induced SoH loss per day \[fraction/day\].
    pub calendar_deg_per_day: f64,
    /// Replacement horizon in months.
    pub replacement_horizon_months: f64,
}

// ---------------------------------------------------------------------------
// Main scheduler
// ---------------------------------------------------------------------------

/// Degradation-aware storage scheduler.
///
/// Uses backward-induction dynamic programming (DP) over a discretised SoC
/// state space to maximise a weighted objective:
///
/// ```text
/// J = revenue_weight × Revenue − degradation_weight × DegradationCost
/// ```
///
/// The degradation cost is computed from the configured [`DegradationPhysics`]
/// model, enabling the planner to explicitly trade off economic return against
/// battery longevity.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DegradationAwareScheduler {
    /// Storage asset specification.
    pub asset: StorageAsset,
    /// Optimiser configuration.
    pub config: DegradationSchedulerConfig,
}

// ---------------------------------------------------------------------------
// Arrhenius constants
// ---------------------------------------------------------------------------

/// Activation energy for SEI growth \[eV\].
const EA_EV: f64 = 0.7;
/// Boltzmann constant in eV/K.
const KB_EV: f64 = 8.617_333_262e-5;
/// Reference temperature \[K\].
const T_REF_K: f64 = 298.0;

impl DegradationAwareScheduler {
    /// Create a new scheduler, validating parameters.
    pub fn new(
        asset: StorageAsset,
        config: DegradationSchedulerConfig,
    ) -> Result<Self, OxiGridError> {
        if asset.capacity_mwh <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "capacity_mwh must be positive".into(),
            ));
        }
        if asset.power_mw <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "power_mw must be positive".into(),
            ));
        }
        if asset.soc_min < 0.0 || asset.soc_max > 1.0 || asset.soc_min >= asset.soc_max {
            return Err(OxiGridError::InvalidParameter(
                "soc_min/soc_max out of range or inverted".into(),
            ));
        }
        if config.soc_bins < 2 {
            return Err(OxiGridError::InvalidParameter(
                "soc_bins must be >= 2".into(),
            ));
        }
        if config.horizon_h == 0 {
            return Err(OxiGridError::InvalidParameter(
                "horizon_h must be >= 1".into(),
            ));
        }
        Ok(Self { asset, config })
    }

    // -----------------------------------------------------------------------
    // Degradation physics helpers
    // -----------------------------------------------------------------------

    /// Compute cycle-induced SoH loss for one half-cycle at the given DoD.
    ///
    /// # Arguments
    /// - `depth_of_discharge_pct` — depth of discharge \[%\] (0–100)
    ///
    /// # Returns
    /// Fractional SoH loss per half-cycle.
    ///
    /// # Model details
    /// - **Linear**: `δ = (dod/100) × deg_per_full_cycle / 2` (half-cycle)
    /// - **Rainflow**: `δ = (dod/ref_depth)^β / (2 × ref_cycles)`
    /// - **SEI / Combined**: cycles evaluated via the `cycle_params` branch.
    pub fn cycle_degradation(&self, depth_of_discharge_pct: f64) -> f64 {
        self.cycle_deg_from_physics(&self.asset.degradation_physics, depth_of_discharge_pct)
    }

    fn cycle_deg_from_physics(&self, physics: &DegradationPhysics, dod_pct: f64) -> f64 {
        let dod_pct = dod_pct.clamp(0.0, 100.0);
        match physics {
            DegradationPhysics::Linear {
                deg_per_full_cycle, ..
            } => {
                // Half-cycle at depth dod_pct/100 of full cycle
                (dod_pct / 100.0) * deg_per_full_cycle / 2.0
            }
            DegradationPhysics::Rainflow {
                stress_exponent,
                ref_depth_pct,
                ref_cycles,
            } => {
                let ref_d = ref_depth_pct.max(1e-9);
                let ratio = dod_pct / ref_d;
                // Wöhler: N(dod) = ref_cycles / (dod/ref_depth)^β
                // SoH loss per half-cycle = 1 / (2 × N(dod))
                ratio.powf(*stress_exponent) / (2.0 * ref_cycles.max(1e-9))
            }
            DegradationPhysics::Sei { .. } => {
                // SEI is primarily a calendar model; treat cycle contribution as zero
                0.0
            }
            DegradationPhysics::Combined { cycle_params, .. } => {
                self.cycle_deg_from_physics(cycle_params, dod_pct)
            }
        }
    }

    /// Compute calendar-induced SoH loss over an interval.
    ///
    /// # Arguments
    /// - `dt_h`         — interval duration \[h\]
    /// - `temperature_c` — operating temperature \[°C\]
    ///
    /// # Returns
    /// Fractional SoH loss due to calendar aging over `dt_h`.
    ///
    /// # Model details
    /// - **Linear**:  `δ_cal = deg_per_year × dt_h / 8760`
    /// - **SEI**:     `δ_sei = k_sei × √(dt_h) × temperature_factor × arrhenius`
    /// - **Rainflow**: purely a cycle model, returns 0.
    pub fn calendar_degradation(&self, dt_h: f64, temperature_c: f64) -> f64 {
        self.cal_deg_from_physics(&self.asset.degradation_physics, dt_h, temperature_c)
    }

    fn cal_deg_from_physics(
        &self,
        physics: &DegradationPhysics,
        dt_h: f64,
        temperature_c: f64,
    ) -> f64 {
        let dt_h = dt_h.max(0.0);
        match physics {
            DegradationPhysics::Linear { deg_per_year, .. } => deg_per_year * dt_h / 8_760.0,
            DegradationPhysics::Rainflow { .. } => 0.0,
            DegradationPhysics::Sei {
                growth_rate,
                temperature_factor,
            } => {
                let arrhenius = self.arrhenius_factor(temperature_c);
                growth_rate * dt_h.sqrt() * temperature_factor * arrhenius
            }
            DegradationPhysics::Combined {
                calendar_params, ..
            } => self.cal_deg_from_physics(calendar_params, dt_h, temperature_c),
        }
    }

    /// Arrhenius acceleration factor relative to T_ref = 298 K.
    ///
    /// `f = exp(Ea/k_B × (1/T_ref − 1/T))`  where `Ea = 0.7 eV`.
    fn arrhenius_factor(&self, temperature_c: f64) -> f64 {
        let t_k = temperature_c + 273.15;
        let exponent = (EA_EV / KB_EV) * (1.0 / T_REF_K - 1.0 / t_k);
        exponent.exp()
    }

    /// Monetary cost of cycle-induced degradation for a given discharge.
    ///
    /// # Arguments
    /// - `discharge_mwh` — energy throughput in this half-cycle \[MWh\]
    /// - `cycle_deg`     — fractional SoH loss per half-cycle (from `cycle_degradation`)
    ///
    /// # Returns
    /// Degradation cost \[$/half-cycle\].
    ///
    /// `cost = cycle_deg × replacement_cost_per_mwh × capacity_mwh`
    pub fn degradation_cost(&self, _discharge_mwh: f64, cycle_deg: f64) -> f64 {
        cycle_deg * self.asset.replacement_cost_per_mwh * self.asset.capacity_mwh
    }

    // -----------------------------------------------------------------------
    // DP helpers
    // -----------------------------------------------------------------------

    /// Map a SoC value to the nearest bin index in `[0, n_bins-1]`.
    #[inline]
    fn soc_to_bin(&self, soc: f64) -> usize {
        let soc_min = self.asset.soc_min;
        let soc_max = self.asset.soc_max;
        let n = self.config.soc_bins;
        let frac = (soc - soc_min) / (soc_max - soc_min);
        let idx = (frac * (n - 1) as f64).round() as isize;
        idx.clamp(0, (n - 1) as isize) as usize
    }

    /// Map a bin index back to its midpoint SoC value.
    #[inline]
    fn bin_to_soc(&self, bin: usize) -> f64 {
        let n = self.config.soc_bins;
        let soc_min = self.asset.soc_min;
        let soc_max = self.asset.soc_max;
        soc_min + bin as f64 / (n - 1).max(1) as f64 * (soc_max - soc_min)
    }

    // -----------------------------------------------------------------------
    // Primary optimisation
    // -----------------------------------------------------------------------

    /// Optimise the dispatch schedule over the market horizon using backward-
    /// induction DP.
    ///
    /// # Bellman equation
    /// ```text
    /// V[t, s] = max_a { revenue_weight × R[t,a,s]
    ///                  − degradation_weight × C_deg[t,a,s]
    ///                  + V[t+1, s'] }
    /// ```
    ///
    /// # Arguments
    /// - `market_opportunities` — per-hour market signals (length ≥ `horizon_h`)
    ///
    /// # Returns
    /// [`DegScheduleResult`] with the optimal hourly schedule.
    pub fn optimize_schedule(
        &self,
        market_opportunities: &[MarketOpportunity],
    ) -> Result<DegScheduleResult, OxiGridError> {
        let horizon = self.config.horizon_h;
        if market_opportunities.len() < horizon {
            return Err(OxiGridError::InvalidParameter(format!(
                "market_opportunities length {} < horizon_h {}",
                market_opportunities.len(),
                horizon
            )));
        }

        let n_bins = self.config.soc_bins;
        let soc_min = self.asset.soc_min;
        let soc_max = self.asset.soc_max;
        let eta_c = self.asset.eta_charge();
        let eta_d = self.asset.eta_discharge();
        let cap = self.asset.capacity_mwh;
        let p_max = self.asset.power_mw;
        let dt = 1.0_f64; // 1-hour steps

        // DP tables
        // value_future[bin] = best cumulative weighted value from t+1 onward
        let mut value_future = vec![0.0_f64; n_bins];
        // policy[t][bin] = optimal power action [MW] (+charge, -discharge)
        let mut policy: Vec<Vec<f64>> = vec![vec![0.0_f64; n_bins]; horizon];

        // Discretise action space: 21 candidate power levels in [-p_max, +p_max]
        const N_ACTIONS: usize = 21;
        let actions: Vec<f64> = (0..N_ACTIONS)
            .map(|i| -p_max + 2.0 * p_max * i as f64 / (N_ACTIONS - 1).max(1) as f64)
            .collect();

        // ── Backward induction ──────────────────────────────────────────────
        for t in (0..horizon).rev() {
            let opp = &market_opportunities[t];
            let price = opp.price_per_mwh;
            let reg_price = opp.regulation_price_per_mw_h;
            let res_price = opp.reserve_price_per_mw_h;

            let mut value_current = vec![f64::NEG_INFINITY; n_bins];
            let mut policy_t = vec![0.0_f64; n_bins];

            for b in 0..n_bins {
                let soc = self.bin_to_soc(b);
                let mut best_val = f64::NEG_INFINITY;
                let mut best_action = 0.0_f64;

                for &action in &actions {
                    // Compute next SoC
                    let soc_next = if action >= 0.0 {
                        // Charging: grid → battery, efficiency loss on entry
                        let e_stored = action * eta_c * dt;
                        soc + e_stored / cap
                    } else {
                        // Discharging: battery → grid, efficiency loss on exit
                        let e_drawn = action.abs() / eta_d * dt;
                        soc - e_drawn / cap
                    };

                    // Feasibility check
                    if soc_next < soc_min - 1e-9 || soc_next > soc_max + 1e-9 {
                        continue;
                    }

                    // ── Revenue ──────────────────────────────────────────
                    // Energy revenue: positive when discharging (sell), negative when charging (buy)
                    let energy_rev = if action >= 0.0 {
                        -action * price * dt
                    } else {
                        action.abs() * price * dt
                    };

                    // Ancillary revenue: small fixed fraction of power_mw committed
                    let ancillary_rev = (p_max * 0.05 * reg_price + p_max * 0.05 * res_price) * dt;

                    let revenue = energy_rev + ancillary_rev;

                    // ── Degradation cost ─────────────────────────────────
                    // DoD expressed as fraction of usable range
                    let dod_pct = if action.abs() > 1e-9 {
                        let e_throughput = action.abs() * dt;
                        (e_throughput / cap * 100.0).min(100.0)
                    } else {
                        0.0
                    };

                    let cycle_deg = self.cycle_degradation(dod_pct);
                    let cal_deg = self.calendar_degradation(dt, self.config.temperature_c);
                    let total_deg = cycle_deg + cal_deg;
                    let dcost = self.degradation_cost(action.abs() * dt, total_deg);

                    // ── Weighted objective ───────────────────────────────
                    let b_next = self.soc_to_bin(soc_next.clamp(soc_min, soc_max));
                    let future_val = value_future[b_next];

                    let obj = self.config.revenue_weight * revenue
                        - self.config.degradation_weight * dcost
                        + future_val;

                    if obj > best_val {
                        best_val = obj;
                        best_action = action;
                    }
                }

                value_current[b] = if best_val == f64::NEG_INFINITY {
                    0.0
                } else {
                    best_val
                };
                policy_t[b] = best_action;
            }

            value_future = value_current;
            policy[t] = policy_t;
        }

        // ── Forward simulation ──────────────────────────────────────────────
        let soc_init = ((soc_min + soc_max) / 2.0).clamp(soc_min, soc_max);
        let mut soc = soc_init;
        let mut dispatch_mw = Vec::with_capacity(horizon);
        let mut soc_profile = Vec::with_capacity(horizon);
        let mut total_revenue = 0.0_f64;
        let mut total_dcost = 0.0_f64;
        let mut total_soh_loss = 0.0_f64;
        let mut total_throughput = 0.0_f64;

        for t in 0..horizon {
            let b = self.soc_to_bin(soc.clamp(soc_min, soc_max));
            let action = policy[t][b];
            let opp = &market_opportunities[t];

            let soc_next = if action >= 0.0 {
                let e_stored = action * eta_c * dt;
                (soc + e_stored / cap).min(soc_max)
            } else {
                let e_drawn = action.abs() / eta_d * dt;
                (soc - e_drawn / cap).max(soc_min)
            };

            let energy_rev = if action >= 0.0 {
                -action * opp.price_per_mwh * dt
            } else {
                action.abs() * opp.price_per_mwh * dt
            };
            let ancillary_rev = (self.asset.power_mw * 0.05 * opp.regulation_price_per_mw_h
                + self.asset.power_mw * 0.05 * opp.reserve_price_per_mw_h)
                * dt;

            let dod_pct = if action.abs() > 1e-9 {
                (action.abs() * dt / cap * 100.0).min(100.0)
            } else {
                0.0
            };
            let cycle_deg = self.cycle_degradation(dod_pct);
            let cal_deg = self.calendar_degradation(dt, self.config.temperature_c);
            let total_deg = cycle_deg + cal_deg;
            let dcost = self.degradation_cost(action.abs() * dt, total_deg);

            total_revenue += energy_rev + ancillary_rev;
            total_dcost += dcost;
            total_soh_loss += total_deg;
            total_throughput += action.abs() * dt;

            dispatch_mw.push(action);
            soc_profile.push(soc_next);
            soc = soc_next;
        }

        let efc = total_throughput / (2.0 * cap);

        Ok(DegScheduleResult {
            dispatch_mw,
            soc_profile,
            total_revenue,
            total_degradation_cost: total_dcost,
            net_value: total_revenue - total_dcost,
            cumulative_soh_loss: total_soh_loss,
            efc,
        })
    }

    // -----------------------------------------------------------------------
    // Lifetime analysis
    // -----------------------------------------------------------------------

    /// Compute the economic value of lifetime extension from conservative dispatch.
    ///
    /// # Arguments
    /// - `aggressive_schedule`    — schedule from a revenue-maximising run
    /// - `conservative_schedule`  — schedule from a degradation-minimising run
    ///
    /// # Returns
    /// Net present value of the extra battery life \[$\].
    ///
    /// # Formula
    /// `extension_value = extra_life_years × annual_net_revenue × discount_factor`
    ///
    /// where `discount_factor = 1 / (1 + r)^T` with r = 8% and T = extra years.
    pub fn lifetime_extension_value(
        &self,
        aggressive_schedule: &DegScheduleResult,
        conservative_schedule: &DegScheduleResult,
    ) -> f64 {
        let soh_remaining = (self.asset.current_soh - self.config.min_soh).max(0.0);
        if soh_remaining < 1e-9 {
            return 0.0;
        }

        let horizon_years = self.config.horizon_h as f64 / 8_760.0;

        // Annual SoH loss rate for each schedule (extrapolated from one horizon)
        let agg_soh_loss_per_year = if horizon_years > 1e-12 {
            aggressive_schedule.cumulative_soh_loss / horizon_years
        } else {
            0.0
        };
        let con_soh_loss_per_year = if horizon_years > 1e-12 {
            conservative_schedule.cumulative_soh_loss / horizon_years
        } else {
            0.0
        };

        // Years of life at each dispatch rate (from current SoH to min_soh)
        let life_aggressive = if agg_soh_loss_per_year > 1e-12 {
            soh_remaining / agg_soh_loss_per_year
        } else {
            f64::INFINITY
        };
        let life_conservative = if con_soh_loss_per_year > 1e-12 {
            soh_remaining / con_soh_loss_per_year
        } else {
            f64::INFINITY
        };

        let extra_life_years = (life_conservative - life_aggressive).max(0.0);
        if extra_life_years < 1e-12 || extra_life_years.is_infinite() {
            return 0.0;
        }

        // Annual net revenue: use aggressive schedule as proxy for potential revenue
        let annual_revenue = if horizon_years > 1e-12 {
            aggressive_schedule.total_revenue / horizon_years
        } else {
            0.0
        };

        // Discount at 8% over the extension period
        let discount_rate = 0.08_f64;
        let discount_factor = 1.0 / (1.0 + discount_rate).powf(extra_life_years);

        extra_life_years * annual_revenue * discount_factor
    }

    /// Find the degradation weight that maximises NPV over the battery's lifetime.
    ///
    /// Performs a coarse sweep of `degradation_weight` ∈ {0.0, 0.1, …, 1.0}
    /// and returns the weight that yields the highest combined NPV.
    ///
    /// # Arguments
    /// - `market_data`      — representative market opportunities
    /// - `replacement_cost` — total battery replacement cost \[$\]
    ///
    /// # Returns
    /// Optimal `degradation_weight` in \[0.0, 1.0\].
    pub fn optimal_degradation_weight(
        &self,
        market_data: &[MarketOpportunity],
        replacement_cost: f64,
    ) -> Result<f64, OxiGridError> {
        let soh_remaining = (self.asset.current_soh - self.config.min_soh).max(0.0);
        let horizon_years = self.config.horizon_h as f64 / 8_760.0;
        let discount_rate = 0.08_f64;

        let mut best_weight = 0.5_f64;
        let mut best_npv = f64::NEG_INFINITY;

        for step in 0..=10_usize {
            let w_deg = step as f64 / 10.0;
            let w_rev = 1.0 - w_deg;

            // Build a temporary scheduler with this weight
            let mut asset = self.asset.clone();
            asset.current_soh = self.asset.current_soh;
            let config = DegradationSchedulerConfig {
                degradation_weight: w_deg,
                revenue_weight: w_rev,
                ..self.config.clone()
            };
            let sched_tmp = DegradationAwareScheduler { asset, config };
            let result = sched_tmp.optimize_schedule(market_data)?;

            // Estimate lifetime at this dispatch rate
            let soh_loss_rate = if horizon_years > 1e-12 {
                result.cumulative_soh_loss / horizon_years
            } else {
                1e-9
            };
            let lifetime_years = if soh_loss_rate > 1e-12 {
                soh_remaining / soh_loss_rate
            } else {
                50.0 // very long life cap
            };
            let lifetime_years = lifetime_years.min(50.0);

            // Annual net revenue from energy dispatch
            let annual_net = if horizon_years > 1e-12 {
                result.net_value / horizon_years
            } else {
                0.0
            };

            // NPV: sum of discounted annual revenues − replacement cost at end
            let npv = if discount_rate > 1e-12 {
                annual_net * (1.0 - (1.0 + discount_rate).powf(-lifetime_years)) / discount_rate
                    - replacement_cost / (1.0 + discount_rate).powf(lifetime_years)
            } else {
                annual_net * lifetime_years - replacement_cost
            };

            if npv > best_npv {
                best_npv = npv;
                best_weight = w_deg;
            }
        }

        Ok(best_weight)
    }

    // -----------------------------------------------------------------------
    // Rainflow cycle counting
    // -----------------------------------------------------------------------

    /// Simplified rainflow cycle counting on a SoC time series.
    ///
    /// Extracts turning points (peaks and valleys) and pairs them as cycles.
    ///
    /// # Arguments
    /// - `soc_profile` — state of charge at each time step (0–1)
    ///
    /// # Returns
    /// Vector of `(mean_soc, depth_pct)` pairs, one per identified half-cycle.
    pub fn rainflow_count(&self, soc_profile: &[f64]) -> Vec<(f64, f64)> {
        if soc_profile.len() < 2 {
            return Vec::new();
        }

        // ── Step 1: extract turning points ───────────────────────────────
        let mut turning_points: Vec<f64> = vec![soc_profile[0]];
        for i in 1..soc_profile.len() - 1 {
            let prev = soc_profile[i - 1];
            let curr = soc_profile[i];
            let next = soc_profile[i + 1];
            // Local extremum: direction changes
            let is_peak = curr >= prev && curr >= next;
            let is_valley = curr <= prev && curr <= next;
            if is_peak || is_valley {
                // Only add if different from last turning point (avoid plateaus)
                if let Some(&last) = turning_points.last() {
                    if (curr - last).abs() > 1e-9 {
                        turning_points.push(curr);
                    }
                }
            }
        }
        // Always include last point
        if let Some(&last) = soc_profile.last() {
            if let Some(&tp_last) = turning_points.last() {
                if (last - tp_last).abs() > 1e-9 {
                    turning_points.push(last);
                }
            }
        }

        // ── Step 2: four-point rainflow algorithm ─────────────────────────
        // Reference: ASTM E1049 §5.4.4
        let mut stack: Vec<f64> = Vec::new();
        let mut cycles: Vec<(f64, f64)> = Vec::new();

        for &point in &turning_points {
            stack.push(point);
            loop {
                let n = stack.len();
                if n < 4 {
                    break;
                }
                let a = stack[n - 4];
                let b = stack[n - 3];
                let c = stack[n - 2];
                let d = stack[n - 1];

                let range_ab = (a - b).abs();
                let range_bc = (b - c).abs();
                let range_cd = (c - d).abs();

                // Extract cycle B-C if its range ≤ outer ranges
                if range_bc <= range_ab && range_bc <= range_cd {
                    let mean_soc = (b + c) / 2.0;
                    let depth_pct = range_bc * 100.0;
                    cycles.push((mean_soc, depth_pct));
                    // Remove B and C from stack
                    stack.remove(n - 3);
                    stack.remove(n - 3); // was n-2, now n-3 after removal
                } else {
                    break;
                }
            }
        }

        // ── Step 3: drain residual half-cycles ────────────────────────────
        // Remaining stack entries are half-cycles
        for i in 0..stack.len().saturating_sub(1) {
            let a = stack[i];
            let b = stack[i + 1];
            let range = (a - b).abs();
            if range > 1e-9 {
                let mean_soc = (a + b) / 2.0;
                let depth_pct = range * 100.0;
                cycles.push((mean_soc, depth_pct));
            }
        }

        cycles
    }

    // -----------------------------------------------------------------------
    // Remaining life projection
    // -----------------------------------------------------------------------

    /// Estimate remaining calendar life in years.
    ///
    /// Projects when SoH will cross `min_soh` based on the calendar degradation
    /// rate at the configured operating temperature and `current_soh`.
    ///
    /// # Returns
    /// Years until SoH < `min_soh` (under calendar aging only).
    pub fn estimate_remaining_calendar_life(&self) -> f64 {
        let soh_remaining = (self.asset.current_soh - self.config.min_soh).max(0.0);
        if soh_remaining < 1e-9 {
            return 0.0;
        }

        // Evaluate calendar degradation rate [fraction/h] at current temperature
        let cal_deg_per_h = self.calendar_degradation(1.0, self.config.temperature_c);
        if cal_deg_per_h < 1e-15 {
            return f64::INFINITY;
        }

        // Hours until min_soh, converted to years
        let hours_remaining = soh_remaining / cal_deg_per_h;
        hours_remaining / 8_760.0
    }

    /// Generate a [`DegradationReport`] for the current asset state.
    ///
    /// # Arguments
    /// - `cycle_deg_per_day` — observed / projected cycle degradation rate \[fraction/day\]
    pub fn degradation_report(&self, cycle_deg_per_day: f64) -> DegradationReport {
        let cal_deg_per_h = self.calendar_degradation(1.0, self.config.temperature_c);
        let cal_deg_per_day = cal_deg_per_h * 24.0;
        let total_deg_per_day = cycle_deg_per_day + cal_deg_per_day;

        let soh_remaining = (self.asset.current_soh - self.config.min_soh).max(0.0);
        let predicted_eol_years = if total_deg_per_day > 1e-15 {
            soh_remaining / (total_deg_per_day * 365.0)
        } else {
            f64::INFINITY
        };

        let replacement_horizon_months = predicted_eol_years * 12.0;

        DegradationReport {
            current_soh: self.asset.current_soh,
            predicted_eol_years,
            cycle_deg_per_day,
            calendar_deg_per_day: cal_deg_per_day,
            replacement_horizon_months,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ── Fixtures ─────────────────────────────────────────────────────────────

    fn make_asset_rainflow() -> StorageAsset {
        StorageAsset {
            asset_id: "bess-test".into(),
            capacity_mwh: 10.0,
            power_mw: 5.0,
            round_trip_efficiency: 0.92,
            soc_min: 0.10,
            soc_max: 0.90,
            current_soh: 1.0,
            replacement_cost_per_mwh: 300.0,
            degradation_physics: DegradationPhysics::rainflow_default(),
        }
    }

    fn make_asset_linear() -> StorageAsset {
        StorageAsset {
            asset_id: "bess-linear".into(),
            capacity_mwh: 10.0,
            power_mw: 5.0,
            round_trip_efficiency: 0.92,
            soc_min: 0.10,
            soc_max: 0.90,
            current_soh: 0.95,
            replacement_cost_per_mwh: 300.0,
            degradation_physics: DegradationPhysics::Linear {
                deg_per_full_cycle: 0.0001,
                deg_per_year: 0.02,
            },
        }
    }

    fn make_asset_sei() -> StorageAsset {
        StorageAsset {
            asset_id: "bess-sei".into(),
            capacity_mwh: 10.0,
            power_mw: 5.0,
            round_trip_efficiency: 0.90,
            soc_min: 0.10,
            soc_max: 0.90,
            current_soh: 1.0,
            replacement_cost_per_mwh: 400.0,
            degradation_physics: DegradationPhysics::Sei {
                growth_rate: 1e-5,
                temperature_factor: 1.0,
            },
        }
    }

    fn default_config() -> DegradationSchedulerConfig {
        DegradationSchedulerConfig {
            horizon_h: 24,
            soc_bins: 50,
            degradation_weight: 0.5,
            revenue_weight: 0.5,
            min_soh: 0.7,
            temperature_c: 25.0,
        }
    }

    fn make_market(n: usize) -> Vec<MarketOpportunity> {
        (0..n)
            .map(|h| {
                // Cheap in first half, expensive in second half
                let price = if h < n / 2 { 30.0 } else { 120.0 };
                MarketOpportunity {
                    hour: h,
                    price_per_mwh: price,
                    reserve_price_per_mw_h: 5.0,
                    regulation_price_per_mw_h: 8.0,
                    renewable_generation_mw: if h < n / 2 { 3.0 } else { 0.0 },
                }
            })
            .collect()
    }

    // ── Test 1: Rainflow – 80% DoD > 40% DoD cycle degradation ───────────────

    #[test]
    fn test_rainflow_deep_dod_more_degradation() {
        let asset = make_asset_rainflow();
        let config = default_config();
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid scheduler");

        let deg_80 = scheduler.cycle_degradation(80.0);
        let deg_40 = scheduler.cycle_degradation(40.0);

        assert!(
            deg_80 > deg_40,
            "80% DoD degradation ({:.6}) should exceed 40% DoD ({:.6})",
            deg_80,
            deg_40
        );
        assert!(deg_40 > 0.0, "40% DoD should still cause some degradation");
    }

    // ── Test 2: Calendar degradation proportional to time ─────────────────────

    #[test]
    fn test_calendar_degradation_proportional_to_time() {
        let asset = make_asset_linear();
        let config = default_config();
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid scheduler");

        let deg_1h = scheduler.calendar_degradation(1.0, 25.0);
        let deg_2h = scheduler.calendar_degradation(2.0, 25.0);

        assert!(
            (deg_2h - 2.0 * deg_1h).abs() < 1e-12,
            "calendar degradation should be proportional to dt: 2h={:.2e} 1h×2={:.2e}",
            deg_2h,
            2.0 * deg_1h
        );
        assert!(
            deg_1h > 0.0,
            "linear calendar degradation should be positive"
        );
    }

    // ── Test 3: SEI – high temperature accelerates degradation ────────────────

    #[test]
    fn test_sei_high_temp_faster_degradation() {
        let asset = make_asset_sei();
        let config = default_config();
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid scheduler");

        let deg_25c = scheduler.calendar_degradation(1.0, 25.0);
        let deg_45c = scheduler.calendar_degradation(1.0, 45.0);

        assert!(
            deg_45c > deg_25c,
            "SEI degradation should be faster at 45°C ({:.2e}) than 25°C ({:.2e})",
            deg_45c,
            deg_25c
        );
    }

    // ── Test 4: DP discharges during high-price hours ─────────────────────────

    #[test]
    fn test_dp_discharge_during_high_price_hours() {
        let asset = make_asset_rainflow();
        let config = default_config();
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid scheduler");

        let market = make_market(24);
        let result = scheduler.optimize_schedule(&market).expect("schedule ok");

        // Second half of the day has price 120 $/MWh — expect discharging
        let discharged_in_peak = result.dispatch_mw[12..].iter().any(|&p| p < -1e-6);

        assert!(
            discharged_in_peak,
            "DP should discharge in high-price hours; dispatch={:?}",
            &result.dispatch_mw[12..]
        );
    }

    // ── Test 5: SoC bounds respected throughout horizon ───────────────────────

    #[test]
    fn test_dp_soc_bounds_respected() {
        let asset = make_asset_rainflow();
        let soc_min = asset.soc_min;
        let soc_max = asset.soc_max;
        let config = default_config();
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid scheduler");

        let market = make_market(24);
        let result = scheduler.optimize_schedule(&market).expect("schedule ok");

        for (t, &soc) in result.soc_profile.iter().enumerate() {
            assert!(
                soc >= soc_min - 1e-6,
                "SoC {:.4} below soc_min {:.4} at hour {}",
                soc,
                soc_min,
                t
            );
            assert!(
                soc <= soc_max + 1e-6,
                "SoC {:.4} above soc_max {:.4} at hour {}",
                soc,
                soc_max,
                t
            );
        }
    }

    // ── Test 6: Degradation cost penalises deep cycling ───────────────────────

    #[test]
    fn test_degradation_cost_penalises_deep_cycling() {
        let asset = make_asset_rainflow();
        let config = default_config();
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid scheduler");

        // Deep cycle (80% DoD) vs shallow cycle (20% DoD)
        let deep_deg = scheduler.cycle_degradation(80.0);
        let shallow_deg = scheduler.cycle_degradation(20.0);
        let deep_cost = scheduler.degradation_cost(8.0, deep_deg);
        let shallow_cost = scheduler.degradation_cost(2.0, shallow_deg);

        assert!(
            deep_cost > shallow_cost,
            "deep cycling cost ({:.4}) should exceed shallow cycling cost ({:.4})",
            deep_cost,
            shallow_cost
        );
    }

    // ── Test 7: Conservative schedule → longer lifetime ───────────────────────

    #[test]
    fn test_lifetime_extension_conservative_schedule() {
        // Two schedulers: one revenue-maximising, one degradation-minimising
        let asset_agg = make_asset_rainflow();
        let asset_con = make_asset_rainflow();

        let config_agg = DegradationSchedulerConfig {
            degradation_weight: 0.0,
            revenue_weight: 1.0,
            ..default_config()
        };
        let config_con = DegradationSchedulerConfig {
            degradation_weight: 1.0,
            revenue_weight: 0.0,
            ..default_config()
        };

        let sched_agg = DegradationAwareScheduler::new(asset_agg, config_agg).expect("valid");
        let sched_con = DegradationAwareScheduler::new(asset_con, config_con).expect("valid");

        let market = make_market(24);
        let result_agg = sched_agg.optimize_schedule(&market).expect("agg ok");
        let result_con = sched_con.optimize_schedule(&market).expect("con ok");

        // Conservative schedule should have less or equal cumulative SoH loss
        assert!(
            result_con.cumulative_soh_loss <= result_agg.cumulative_soh_loss + 1e-12,
            "conservative SoH loss ({:.6}) should be ≤ aggressive ({:.6})",
            result_con.cumulative_soh_loss,
            result_agg.cumulative_soh_loss
        );

        // Compute extension value using the aggressive scheduler's method
        let ext_value = sched_agg.lifetime_extension_value(&result_agg, &result_con);
        assert!(
            ext_value >= 0.0,
            "lifetime extension value must be non-negative: {:.4}",
            ext_value
        );
    }

    // ── Test 8: Rainflow counting – sine-wave SoC profile ─────────────────────

    #[test]
    fn test_rainflow_count_sine_wave() {
        let asset = make_asset_rainflow();
        let config = default_config();
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid scheduler");

        // Generate a sine-wave SoC profile with 3 full cycles
        let n_points = 300;
        let n_cycles = 3.0_f64;
        let soc_profile: Vec<f64> = (0..n_points)
            .map(|i| {
                let theta = 2.0 * core::f64::consts::PI * n_cycles * i as f64 / n_points as f64;
                0.5 + 0.4 * theta.sin()
            })
            .collect();

        let cycles = scheduler.rainflow_count(&soc_profile);

        // Should detect at least n_cycles worth of half-cycles (≥ 3 full cycles)
        // A full sine wave produces 2 half-cycles per cycle
        assert!(
            !cycles.is_empty(),
            "rainflow should detect cycles in sine-wave profile"
        );

        // All depths should be positive
        for (mean, depth) in &cycles {
            assert!(
                *depth >= 0.0,
                "cycle depth must be non-negative: mean={:.4} depth={:.4}",
                mean,
                depth
            );
        }

        // Total detected cycles should be in a reasonable range (≥ 3)
        assert!(
            cycles.len() >= 3,
            "should detect at least 3 cycles in 3-cycle sine wave, got {}",
            cycles.len()
        );
    }

    // ── Test 9: Remaining calendar life decreases with higher temperature ──────

    #[test]
    fn test_remaining_calendar_life_sei() {
        let asset = make_asset_sei();

        let config_25 = DegradationSchedulerConfig {
            temperature_c: 25.0,
            ..default_config()
        };
        let config_45 = DegradationSchedulerConfig {
            temperature_c: 45.0,
            ..default_config()
        };

        let sched_25 = DegradationAwareScheduler::new(asset.clone(), config_25).expect("valid");
        let sched_45 = DegradationAwareScheduler::new(asset, config_45).expect("valid");

        let life_25 = sched_25.estimate_remaining_calendar_life();
        let life_45 = sched_45.estimate_remaining_calendar_life();

        assert!(
            life_45 < life_25,
            "higher temperature should reduce calendar life: 45°C={:.2}y 25°C={:.2}y",
            life_45,
            life_25
        );
        assert!(life_25 > 0.0, "life at 25°C should be positive");
        assert!(life_45 > 0.0, "life at 45°C should be positive");
    }

    // ── Test 10: Combined model sums cycle + calendar contributions ───────────

    #[test]
    fn test_combined_model_sums_contributions() {
        let asset = StorageAsset {
            asset_id: "combined".into(),
            capacity_mwh: 10.0,
            power_mw: 5.0,
            round_trip_efficiency: 0.92,
            soc_min: 0.10,
            soc_max: 0.90,
            current_soh: 1.0,
            replacement_cost_per_mwh: 300.0,
            degradation_physics: DegradationPhysics::Combined {
                cycle_params: Box::new(DegradationPhysics::rainflow_default()),
                calendar_params: Box::new(DegradationPhysics::Linear {
                    deg_per_full_cycle: 0.0,
                    deg_per_year: 0.02,
                }),
            },
        };

        let config = default_config();
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid");

        let cycle_deg = scheduler.cycle_degradation(80.0);
        let cal_deg = scheduler.calendar_degradation(1.0, 25.0);

        assert!(cycle_deg > 0.0, "combined: cycle component should be > 0");
        assert!(cal_deg > 0.0, "combined: calendar component should be > 0");
    }

    // ── Test 11: Construction rejects zero capacity ────────────────────────────

    #[test]
    fn test_new_rejects_zero_capacity() {
        // Reason: validate that the constructor returns an error for zero capacity_mwh.
        let asset = StorageAsset {
            asset_id: "bad".into(),
            capacity_mwh: 0.0,
            power_mw: 5.0,
            round_trip_efficiency: 0.92,
            soc_min: 0.10,
            soc_max: 0.90,
            current_soh: 1.0,
            replacement_cost_per_mwh: 300.0,
            degradation_physics: DegradationPhysics::rainflow_default(),
        };
        let result = DegradationAwareScheduler::new(asset, default_config());
        assert!(result.is_err(), "zero capacity_mwh must produce an error");
    }

    // ── Test 12: Construction rejects inverted SoC bounds ─────────────────────

    #[test]
    fn test_new_rejects_inverted_soc_bounds() {
        // Reason: validate that soc_min >= soc_max is caught at construction time.
        let asset = StorageAsset {
            asset_id: "bad-soc".into(),
            capacity_mwh: 10.0,
            power_mw: 5.0,
            round_trip_efficiency: 0.92,
            soc_min: 0.90,
            soc_max: 0.10,
            current_soh: 1.0,
            replacement_cost_per_mwh: 300.0,
            degradation_physics: DegradationPhysics::rainflow_default(),
        };
        let result = DegradationAwareScheduler::new(asset, default_config());
        assert!(
            result.is_err(),
            "inverted soc_min/soc_max must produce an error"
        );
    }

    // ── Test 13: Degradation report has valid non-negative fields ──────────────

    #[test]
    fn test_degradation_report_fields() {
        // Reason: verify that degradation_report produces a struct with all
        // non-negative fields for a typical operating point.
        let asset = make_asset_linear();
        let config = default_config();
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid scheduler");

        let report = scheduler.degradation_report(1e-5);

        assert!(
            report.current_soh > 0.0 && report.current_soh <= 1.0,
            "current_soh out of range: {:.4}",
            report.current_soh
        );
        assert!(
            report.calendar_deg_per_day >= 0.0,
            "calendar_deg_per_day must be >= 0: {:.6}",
            report.calendar_deg_per_day
        );
        assert!(
            report.cycle_deg_per_day >= 0.0,
            "cycle_deg_per_day must be >= 0: {:.6}",
            report.cycle_deg_per_day
        );
        assert!(
            report.predicted_eol_years > 0.0,
            "predicted_eol_years must be positive: {:.4}",
            report.predicted_eol_years
        );
        assert!(
            report.replacement_horizon_months > 0.0,
            "replacement_horizon_months must be positive: {:.4}",
            report.replacement_horizon_months
        );
    }

    // ── Test 14: Rainflow count on empty / single-element profile ─────────────

    #[test]
    fn test_rainflow_count_empty_profile() {
        // Reason: edge-case — empty and length-1 profiles should return empty cycle lists.
        let asset = make_asset_rainflow();
        let config = default_config();
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid scheduler");

        let empty = scheduler.rainflow_count(&[]);
        assert!(empty.is_empty(), "empty profile should produce no cycles");

        let single = scheduler.rainflow_count(&[0.5]);
        assert!(
            single.is_empty(),
            "single-point profile should produce no cycles"
        );
    }

    // ── Test 15: optimize_schedule returns error when market data too short ────

    #[test]
    fn test_optimize_schedule_error_on_short_market_data() {
        // Reason: verify that the optimizer returns an error when the market
        // opportunity slice is shorter than horizon_h.
        let asset = make_asset_rainflow();
        let config = DegradationSchedulerConfig {
            horizon_h: 24,
            ..default_config()
        };
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid scheduler");

        let short_market = make_market(10); // only 10 hours, horizon is 24
        let result = scheduler.optimize_schedule(&short_market);
        assert!(
            result.is_err(),
            "optimize_schedule with too-short market data must return an error"
        );
    }

    // ── Test 16: optimal_degradation_weight returns value in [0, 1] ───────────

    #[test]
    fn test_optimal_degradation_weight_range() {
        // Reason: the sweep result must always lie within the valid [0.0, 1.0] range.
        let asset = make_asset_linear();
        let config = default_config();
        let scheduler = DegradationAwareScheduler::new(asset, config).expect("valid scheduler");

        let market = make_market(24);
        let replacement_cost = 50_000.0;
        let weight = scheduler
            .optimal_degradation_weight(&market, replacement_cost)
            .expect("optimal weight should succeed");

        assert!(
            (0.0..=1.0).contains(&weight),
            "optimal_degradation_weight must be in [0, 1], got {:.3}",
            weight
        );
    }
}
