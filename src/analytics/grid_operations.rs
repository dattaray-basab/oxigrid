//! Grid Operations Analytics — KPIs, generator performance, congestion, renewables, and demand.
//!
//! Provides six major analytic structs for operational intelligence of power systems:
//!
//! | Struct | Purpose |
//! |---|---|
//! | [`OperationalKpis`] | System-wide KPIs over a period |
//! | [`GeneratorPerformance`] | Per-unit generation statistics |
//! | [`CongestionAnalyzer`] | Transmission branch loading and LMP spread |
//! | [`RenewableMetrics`] | Renewable penetration, curtailment, variability |
//! | [`DemandAnalytics`] | Load profiling, OLS temperature sensitivity |
//! | [`OperationsReport`] | Composite report with auto-generated alerts |

// ─── 1. Operational KPIs ─────────────────────────────────────────────────────

/// Aggregate operational key performance indicators for a power system.
///
/// All energy quantities are in \[MWh\], power in \[MW\], cost in currency units,
/// and emissions in \[tonnes CO₂\].
#[derive(Debug, Clone)]
pub struct OperationalKpis {
    /// Length of the reporting period \[h\].
    pub period_hours: f64,
    /// Total electrical energy produced \[MWh\].
    pub total_energy_generated_mwh: f64,
    /// Total electrical energy reaching customers \[MWh\] (generated minus losses).
    pub total_energy_delivered_mwh: f64,
    /// Observed peak system load \[MW\].
    pub peak_load_mw: f64,
    /// Time-average system load \[MW\].
    pub average_load_mw: f64,
    /// Thermal fuel consumed \[MMBTU\].
    pub fuel_consumed_mmbtu: f64,
    /// Carbon dioxide emitted \[tonnes\].
    pub co2_emitted_tonnes: f64,
    /// Cumulative customer-weighted outage duration \[h\].
    pub outage_hours: f64,
    /// Number of customers affected by outages.
    pub outage_customers: usize,
    /// Operations and maintenance expenditure \[currency\].
    pub maintenance_cost: f64,
    /// Fuel procurement cost \[currency\].
    pub fuel_cost: f64,
}

impl OperationalKpis {
    /// System efficiency \[%\] = `total_energy_delivered_mwh / total_energy_generated_mwh * 100`.
    ///
    /// Returns `0.0` when no energy was generated.
    pub fn system_efficiency_pct(&self) -> f64 {
        if self.total_energy_generated_mwh == 0.0 {
            return 0.0;
        }
        self.total_energy_delivered_mwh / self.total_energy_generated_mwh * 100.0
    }

    /// Load factor \[%\] = `average_load_mw / peak_load_mw * 100`.
    ///
    /// Returns `0.0` when `peak_load_mw` is zero.
    pub fn load_factor_pct(&self) -> f64 {
        if self.peak_load_mw == 0.0 {
            return 0.0;
        }
        self.average_load_mw / self.peak_load_mw * 100.0
    }

    /// Station heat rate \[BTU/kWh\] = `fuel_consumed_mmbtu * 1e6 / (generated_mwh * 1e3)`.
    ///
    /// Returns `0.0` when no energy was generated.
    pub fn heat_rate_btu_per_kwh(&self) -> f64 {
        let generated_kwh = self.total_energy_generated_mwh * 1_000.0;
        if generated_kwh == 0.0 {
            return 0.0;
        }
        self.fuel_consumed_mmbtu * 1_000_000.0 / generated_kwh
    }

    /// Emission intensity \[kg CO₂/MWh\] = `co2_emitted_tonnes * 1000 / delivered_mwh`.
    ///
    /// Returns `0.0` when no energy was delivered.
    pub fn emissions_intensity_kg_per_mwh(&self) -> f64 {
        if self.total_energy_delivered_mwh == 0.0 {
            return 0.0;
        }
        self.co2_emitted_tonnes * 1_000.0 / self.total_energy_delivered_mwh
    }

    /// SAIDI \[minutes\] per IEEE 1366: `outage_hours * 60 * outage_customers / total_customers`.
    ///
    /// Returns `0.0` when `total_customers` is zero.
    pub fn saidi_minutes(&self, total_customers: usize) -> f64 {
        if total_customers == 0 {
            return 0.0;
        }
        self.outage_hours * 60.0 * self.outage_customers as f64 / total_customers as f64
    }

    /// All-in cost per \[MWh\] delivered = `(maintenance_cost + fuel_cost) / delivered_mwh`.
    ///
    /// Returns `0.0` when no energy was delivered.
    pub fn cost_per_mwh_delivered(&self) -> f64 {
        if self.total_energy_delivered_mwh == 0.0 {
            return 0.0;
        }
        (self.maintenance_cost + self.fuel_cost) / self.total_energy_delivered_mwh
    }

    /// Availability factor \[%\] = `(period_hours − outage_hours) / period_hours * 100`.
    ///
    /// Returns `0.0` when `period_hours` is zero.
    pub fn availability_factor_pct(&self) -> f64 {
        if self.period_hours == 0.0 {
            return 0.0;
        }
        (self.period_hours - self.outage_hours).max(0.0) / self.period_hours * 100.0
    }
}

// ─── 2. Generator Fuel Type ───────────────────────────────────────────────────

/// Fuel or energy source classification for a generating unit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GeneratorFuelType {
    /// Combined-cycle or simple-cycle gas turbine.
    NaturalGas,
    /// Steam-cycle coal plant.
    Coal,
    /// Residual or distillate oil combustion.
    Oil,
    /// Nuclear fission (light-water or heavy-water reactor).
    Nuclear,
    /// Run-of-river or reservoir hydropower.
    Hydro,
    /// Onshore or offshore wind turbine.
    Wind,
    /// Photovoltaic or concentrating solar.
    Solar,
    /// Dedicated biomass combustion or co-firing.
    Biomass,
}

// ─── 3. Generator Performance ─────────────────────────────────────────────────

/// Hourly performance record for a single generating unit.
///
/// All vectors are indexed by hour; they must be of equal length for methods to be meaningful.
#[derive(Debug, Clone)]
pub struct GeneratorPerformance {
    /// Unique numeric identifier for the unit.
    pub unit_id: usize,
    /// Human-readable unit name.
    pub name: String,
    /// Fuel or energy source.
    pub fuel_type: GeneratorFuelType,
    /// Nameplate capacity \[MW\].
    pub rated_mw: f64,
    /// Hourly net energy output \[MWh\].
    pub generation_mwh: Vec<f64>,
    /// Hourly thermal fuel consumption \[MMBTU\].
    pub fuel_consumed: Vec<f64>,
    /// `true` in every hour the unit executed a start-up.
    pub starts: Vec<bool>,
    /// `true` in every hour the unit was unavailable due to a forced outage.
    pub outage_flags: Vec<bool>,
}

impl GeneratorPerformance {
    /// Capacity factor \[%\] = `Σ generation_mwh / (rated_mw × n_hours) × 100`.
    ///
    /// Returns `0.0` when `rated_mw` is zero or the generation vector is empty.
    pub fn capacity_factor_pct(&self) -> f64 {
        let n = self.generation_mwh.len();
        if n == 0 || self.rated_mw == 0.0 {
            return 0.0;
        }
        let total: f64 = self.generation_mwh.iter().sum();
        total / (self.rated_mw * n as f64) * 100.0
    }

    /// Equivalent forced outage rate (EFOR) \[%\].
    ///
    /// `EFOR = forced_outage_hours / (forced_outage_hours + in_service_hours) × 100`
    ///
    /// Returns `0.0` when the denominator is zero.
    pub fn equivalent_forced_outage_rate_pct(&self) -> f64 {
        let forced: f64 = self.outage_flags.iter().filter(|&&f| f).count() as f64;
        let in_service: f64 = self.outage_flags.iter().filter(|&&f| !f).count() as f64;
        let denom = forced + in_service;
        if denom == 0.0 {
            return 0.0;
        }
        forced / denom * 100.0
    }

    /// Number of start events recorded in `starts`.
    pub fn start_count(&self) -> usize {
        self.starts.iter().filter(|&&s| s).count()
    }

    /// Fleet-average heat rate \[BTU/kWh\].
    ///
    /// Computed as `Σ fuel_consumed_mmbtu × 1e6 / (Σ generation_mwh × 1e3)`.
    /// Returns `0.0` when total generation is zero.
    pub fn average_heat_rate(&self) -> f64 {
        let total_gen_kwh: f64 = self.generation_mwh.iter().sum::<f64>() * 1_000.0;
        if total_gen_kwh == 0.0 {
            return 0.0;
        }
        let total_fuel_btu: f64 = self.fuel_consumed.iter().sum::<f64>() * 1_000_000.0;
        total_fuel_btu / total_gen_kwh
    }

    /// Count of ramp cycles: consecutive-hour load changes exceeding 10 \[%\] of `rated_mw`.
    ///
    /// A cycle is any hour-to-hour transition where `|gen[h] − gen[h-1]| > 0.10 × rated_mw`.
    pub fn ramp_cycles(&self) -> usize {
        if self.generation_mwh.len() < 2 || self.rated_mw == 0.0 {
            return 0;
        }
        let threshold = 0.10 * self.rated_mw;
        self.generation_mwh
            .windows(2)
            .filter(|w| (w[1] - w[0]).abs() > threshold)
            .count()
    }

    /// Equivalent operating hours (EOH) using the GE approximation.
    ///
    /// `EOH = run_hours + 5 × hot_starts + 10 × cold_starts`
    ///
    /// The first start event is classified as a hot start; all subsequent ones as cold starts.
    pub fn equivalent_operating_hours(&self) -> f64 {
        let run_hours: f64 = self.outage_flags.iter().filter(|&&f| !f).count() as f64;
        let total_starts = self.start_count();
        let hot_starts = if total_starts > 0 { 1 } else { 0 };
        let cold_starts = total_starts.saturating_sub(hot_starts);
        run_hours + 5.0 * hot_starts as f64 + 10.0 * cold_starts as f64
    }
}

// ─── 4. Congestion Analyzer ───────────────────────────────────────────────────

/// Transmission congestion analytics over a multi-hour study period.
///
/// `branch_flows_mw[hour][branch]` — signed branch MW flow.
/// `lmp_prices[hour][bus]` — locational marginal price \[$/MWh\].
#[derive(Debug, Clone)]
pub struct CongestionAnalyzer {
    /// Thermal rating of each branch \[MW\].
    pub branch_ratings_mw: Vec<f64>,
    /// Hourly branch power flows \[MW\]; outer index = hour, inner = branch.
    pub branch_flows_mw: Vec<Vec<f64>>,
    /// Human-readable label for each branch.
    pub branch_names: Vec<String>,
    /// Hourly LMP by bus \[$/MWh\]; outer index = hour, inner = bus.
    pub lmp_prices: Vec<Vec<f64>>,
}

impl CongestionAnalyzer {
    /// Number of hours each branch is loaded above 90 \[%\] of its rating.
    pub fn congested_hours_per_branch(&self) -> Vec<usize> {
        let n_branches = self.branch_ratings_mw.len();
        let mut counts = vec![0usize; n_branches];
        for hour_flows in &self.branch_flows_mw {
            for (b, &flow) in hour_flows.iter().enumerate() {
                if b >= n_branches {
                    break;
                }
                let rating = self.branch_ratings_mw[b];
                if rating > 0.0 && flow.abs() / rating > 0.90 {
                    counts[b] += 1;
                }
            }
        }
        counts
    }

    /// Branch loading \[%\] = `|flow| / rating × 100` for the given branch and hour.
    ///
    /// Returns `0.0` when indices are out of range or the rating is zero.
    pub fn loading_pct(&self, branch: usize, hour: usize) -> f64 {
        let rating = match self.branch_ratings_mw.get(branch) {
            Some(&r) if r > 0.0 => r,
            _ => return 0.0,
        };
        let flow = match self.branch_flows_mw.get(hour).and_then(|h| h.get(branch)) {
            Some(&f) => f,
            None => return 0.0,
        };
        flow.abs() / rating * 100.0
    }

    /// Branch with the highest time-average loading \[%\].
    ///
    /// Returns `None` when there are no branches or no flow data.
    pub fn most_congested_branch(&self) -> Option<(usize, f64)> {
        let n_branches = self.branch_ratings_mw.len();
        if n_branches == 0 || self.branch_flows_mw.is_empty() {
            return None;
        }
        let n_hours = self.branch_flows_mw.len() as f64;
        let avg_loading: Vec<f64> = (0..n_branches)
            .map(|b| {
                let sum: f64 = self
                    .branch_flows_mw
                    .iter()
                    .map(|h| {
                        let flow = h.get(b).copied().unwrap_or(0.0);
                        let rating = self.branch_ratings_mw[b];
                        if rating > 0.0 {
                            flow.abs() / rating * 100.0
                        } else {
                            0.0
                        }
                    })
                    .sum();
                sum / n_hours
            })
            .collect();

        avg_loading
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(idx, &val)| (idx, val))
    }

    /// Congestion cost per hour \[currency/h\].
    ///
    /// For each hour: `Σ_branch max(0, |flow| − rating) × avg_lmp`.
    ///
    /// The `ptdf` parameter is accepted for API compatibility but the simplified
    /// formulation uses only the average LMP across all buses as the shadow price.
    pub fn congestion_cost_per_hour(&self, _ptdf: &[Vec<f64>]) -> Vec<f64> {
        self.branch_flows_mw
            .iter()
            .zip(self.lmp_prices.iter())
            .map(|(flows, lmps)| {
                let avg_lmp = if lmps.is_empty() {
                    0.0
                } else {
                    lmps.iter().sum::<f64>() / lmps.len() as f64
                };
                flows
                    .iter()
                    .enumerate()
                    .map(|(b, &flow)| {
                        let rating = self.branch_ratings_mw.get(b).copied().unwrap_or(0.0);
                        (flow.abs() - rating).max(0.0) * avg_lmp
                    })
                    .sum()
            })
            .collect()
    }

    /// LMP spread \[$/MWh\] per hour = `max(lmp) − min(lmp)`.
    ///
    /// Returns `0.0` for hours with no bus data.
    pub fn lmp_spread_mwh(&self) -> Vec<f64> {
        self.lmp_prices
            .iter()
            .map(|lmps| {
                if lmps.is_empty() {
                    return 0.0;
                }
                let max = lmps.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                let min = lmps.iter().cloned().fold(f64::INFINITY, f64::min);
                max - min
            })
            .collect()
    }

    /// Net interface flow \[MW\] = sum of flows on the specified branches at the given hour.
    ///
    /// Returns `0.0` for out-of-range hours.
    pub fn interface_flow(&self, branch_indices: &[usize], hour: usize) -> f64 {
        let hour_flows = match self.branch_flows_mw.get(hour) {
            Some(f) => f,
            None => return 0.0,
        };
        branch_indices
            .iter()
            .map(|&b| hour_flows.get(b).copied().unwrap_or(0.0))
            .sum()
    }
}

// ─── 5. Renewable Metrics ─────────────────────────────────────────────────────

/// Renewable energy integration metrics over a study horizon.
///
/// All energy vectors are indexed by hour in \[MWh\].
#[derive(Debug, Clone)]
pub struct RenewableMetrics {
    /// Hourly total system generation \[MWh\].
    pub total_generation_mwh: Vec<f64>,
    /// Hourly renewable generation dispatched \[MWh\].
    pub renewable_generation_mwh: Vec<f64>,
    /// Hourly renewable energy curtailed \[MWh\].
    pub curtailed_mwh: Vec<f64>,
    /// Installed renewable nameplate capacity \[MW\].
    pub re_capacity_mw: f64,
    /// Hourly energy charged into storage \[MWh\].
    pub storage_charge_mwh: Vec<f64>,
    /// Hourly energy discharged from storage \[MWh\].
    pub storage_discharge_mwh: Vec<f64>,
}

impl RenewableMetrics {
    /// Renewable penetration \[%\] per hour = `renewable / total × 100`.
    ///
    /// Returns `0.0` for hours with zero total generation.
    pub fn penetration_pct_per_hour(&self) -> Vec<f64> {
        self.renewable_generation_mwh
            .iter()
            .zip(self.total_generation_mwh.iter())
            .map(|(&re, &tot)| {
                if tot == 0.0 {
                    0.0
                } else {
                    (re / tot * 100.0).min(100.0)
                }
            })
            .collect()
    }

    /// Annual (period-average) renewable penetration \[%\].
    pub fn annual_penetration_pct(&self) -> f64 {
        let total_re: f64 = self.renewable_generation_mwh.iter().sum();
        let total_gen: f64 = self.total_generation_mwh.iter().sum();
        if total_gen == 0.0 {
            return 0.0;
        }
        (total_re / total_gen * 100.0).min(100.0)
    }

    /// Curtailment rate \[%\] = `curtailed / (curtailed + renewable_generation) × 100`.
    ///
    /// Returns `0.0` when the sum is zero.
    pub fn curtailment_rate_pct(&self) -> f64 {
        let curtailed: f64 = self.curtailed_mwh.iter().sum();
        let dispatched: f64 = self.renewable_generation_mwh.iter().sum();
        let denom = curtailed + dispatched;
        if denom == 0.0 {
            return 0.0;
        }
        curtailed / denom * 100.0
    }

    /// Renewable capacity factor \[%\] = `Σ renewable_generation / (re_capacity × n_hours) × 100`.
    ///
    /// Returns `0.0` when capacity or hours are zero.
    pub fn capacity_factor_pct(&self) -> f64 {
        let n = self.renewable_generation_mwh.len();
        if n == 0 || self.re_capacity_mw == 0.0 {
            return 0.0;
        }
        let total_re: f64 = self.renewable_generation_mwh.iter().sum();
        (total_re / (self.re_capacity_mw * n as f64) * 100.0).min(100.0)
    }

    /// Storage utilisation \[%\] = `Σ discharge / (re_capacity × n_hours) × 100`.
    ///
    /// Returns `0.0` when capacity or hours are zero.
    pub fn storage_utilization_pct(&self) -> f64 {
        let n = self.storage_discharge_mwh.len();
        if n == 0 || self.re_capacity_mw == 0.0 {
            return 0.0;
        }
        let total_discharge: f64 = self.storage_discharge_mwh.iter().sum();
        total_discharge / (self.re_capacity_mw * n as f64) * 100.0
    }

    /// Number of hours where renewable penetration exceeds `threshold_pct` \[%\].
    pub fn hours_above_penetration(&self, threshold_pct: f64) -> usize {
        self.penetration_pct_per_hour()
            .iter()
            .filter(|&&p| p > threshold_pct)
            .count()
    }

    /// Variability index = `std(renewable_generation) / mean(total_generation)`.
    ///
    /// Returns `0.0` when the series are empty or the mean total generation is zero.
    pub fn variability_index(&self) -> f64 {
        let n = self.renewable_generation_mwh.len();
        if n == 0 {
            return 0.0;
        }
        let mean_re = self.renewable_generation_mwh.iter().sum::<f64>() / n as f64;
        let variance: f64 = self
            .renewable_generation_mwh
            .iter()
            .map(|&x| (x - mean_re).powi(2))
            .sum::<f64>()
            / n as f64;
        let std_re = variance.sqrt();

        let mean_tot = self.total_generation_mwh.iter().sum::<f64>()
            / self.total_generation_mwh.len().max(1) as f64;
        if mean_tot == 0.0 {
            return 0.0;
        }
        std_re / mean_tot
    }
}

// ─── 6. Day Type ─────────────────────────────────────────────────────────────

/// Demand calendar classification for each hour of the load profile.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DayType {
    /// Monday–Friday (non-holiday).
    Weekday,
    /// Saturday.
    Saturday,
    /// Sunday.
    Sunday,
    /// Designated public holiday.
    Holiday,
}

// ─── 7. Demand Analytics ──────────────────────────────────────────────────────

/// Demand analytics derived from an hourly load profile.
///
/// Typically covers a full year (8 760 hours) but works for any horizon.
/// `load_profile_mw`, `temperature_c`, and `day_type` should all have the same length.
#[derive(Debug, Clone)]
pub struct DemandAnalytics {
    /// Hourly gross system load \[MW\].
    pub load_profile_mw: Vec<f64>,
    /// Hourly ambient temperature \[°C\] (co-indexed with load).
    pub temperature_c: Vec<f64>,
    /// Calendar classification for each hour.
    pub day_type: Vec<DayType>,
}

impl DemandAnalytics {
    /// Maximum load in the profile \[MW\].
    ///
    /// Returns `0.0` for an empty profile.
    pub fn peak_mw(&self) -> f64 {
        self.load_profile_mw
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
            .max(0.0)
    }

    /// Minimum load in the profile \[MW\].
    ///
    /// Returns `0.0` for an empty profile.
    pub fn valley_mw(&self) -> f64 {
        if self.load_profile_mw.is_empty() {
            return 0.0;
        }
        self.load_profile_mw
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min)
            .max(0.0)
    }

    /// Peak-to-valley ratio = `peak / valley`.
    ///
    /// Returns `0.0` when the valley is zero or the profile is empty.
    pub fn peak_to_valley_ratio(&self) -> f64 {
        let valley = self.valley_mw();
        if valley == 0.0 {
            return 0.0;
        }
        self.peak_mw() / valley
    }

    /// Total energy consumption over the period \[GWh\] = `Σ load_mw / 1000`.
    pub fn annual_energy_gwh(&self) -> f64 {
        self.load_profile_mw.iter().sum::<f64>() / 1_000.0
    }

    /// Load duration curve: pairs of `(rank_hours, MW)` sorted by descending MW.
    ///
    /// Rank 1 corresponds to the highest-load hour; rank N to the lowest.
    pub fn load_duration_curve(&self) -> Vec<(f64, f64)> {
        let mut sorted = self.load_profile_mw.clone();
        sorted.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));
        sorted
            .into_iter()
            .enumerate()
            .map(|(i, mw)| ((i + 1) as f64, mw))
            .collect()
    }

    /// OLS regression slope of load on temperature \[MW/°C\].
    ///
    /// Fits `P = a + b × T` by ordinary least squares; returns `b`.
    /// Returns `0.0` when the profile is empty, lengths differ, or the
    /// temperature variance is zero.
    pub fn temperature_sensitivity_mw_per_c(&self) -> f64 {
        let n = self.load_profile_mw.len();
        if n == 0 || self.temperature_c.len() != n {
            return 0.0;
        }
        let n_f = n as f64;
        let sum_x: f64 = self.temperature_c.iter().sum();
        let sum_y: f64 = self.load_profile_mw.iter().sum();
        let sum_xy: f64 = self
            .temperature_c
            .iter()
            .zip(self.load_profile_mw.iter())
            .map(|(x, y)| x * y)
            .sum();
        let sum_xx: f64 = self.temperature_c.iter().map(|x| x * x).sum();
        let denom = n_f * sum_xx - sum_x * sum_x;
        if denom == 0.0 {
            return 0.0;
        }
        (n_f * sum_xy - sum_x * sum_y) / denom
    }

    /// Index of the hour with the highest load.
    ///
    /// Returns `0` for an empty profile.
    pub fn peak_hour_of_year(&self) -> usize {
        self.load_profile_mw
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0)
    }

    /// Ratio of mean weekday load to mean weekend (Saturday + Sunday + Holiday) load.
    ///
    /// Returns `0.0` when there are no weekday or no weekend hours.
    pub fn weekday_vs_weekend_ratio(&self) -> f64 {
        let mut weekday_sum = 0.0_f64;
        let mut weekday_count = 0usize;
        let mut weekend_sum = 0.0_f64;
        let mut weekend_count = 0usize;

        for (load, day) in self.load_profile_mw.iter().zip(self.day_type.iter()) {
            match day {
                DayType::Weekday => {
                    weekday_sum += load;
                    weekday_count += 1;
                }
                DayType::Saturday | DayType::Sunday | DayType::Holiday => {
                    weekend_sum += load;
                    weekend_count += 1;
                }
            }
        }

        if weekday_count == 0 || weekend_count == 0 {
            return 0.0;
        }
        let weekday_avg = weekday_sum / weekday_count as f64;
        let weekend_avg = weekend_sum / weekend_count as f64;
        if weekend_avg == 0.0 {
            return 0.0;
        }
        weekday_avg / weekend_avg
    }

    /// Theoretical demand flexibility potential \[%\] = `(peak − average) / peak × 100`.
    ///
    /// Represents the fraction of peak demand that could theoretically be shifted.
    /// Returns `0.0` when peak is zero or the profile is empty.
    pub fn demand_flexibility_potential_pct(&self) -> f64 {
        let n = self.load_profile_mw.len();
        if n == 0 {
            return 0.0;
        }
        let peak = self.peak_mw();
        if peak == 0.0 {
            return 0.0;
        }
        let avg = self.load_profile_mw.iter().sum::<f64>() / n as f64;
        (peak - avg) / peak * 100.0
    }
}

// ─── 8. Operations Report ─────────────────────────────────────────────────────

/// Compact renewable energy summary for inclusion in an operations report.
#[derive(Debug, Clone)]
pub struct RenewableSummary {
    /// Aggregate renewable penetration \[%\] over the reporting period.
    pub penetration_pct: f64,
    /// Curtailment rate \[%\] over the reporting period.
    pub curtailment_pct: f64,
    /// Renewable capacity factor \[%\] over the reporting period.
    pub capacity_factor_pct: f64,
}

/// Severity classification for an operations alert.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AlertSeverity {
    /// Informational notice; no action required.
    Info,
    /// Performance degradation warrants investigation.
    Warning,
    /// Immediate action required.
    Critical,
}

/// A single triggered alert with its context values.
#[derive(Debug, Clone)]
pub struct OperationsAlert {
    /// Alert severity level.
    pub severity: AlertSeverity,
    /// Human-readable alert message.
    pub message: String,
    /// Observed metric value that triggered the alert.
    pub value: f64,
    /// Threshold that was exceeded or not met.
    pub threshold: f64,
}

/// Composite operations report combining KPIs, congestion, renewables, and alerts.
#[derive(Debug, Clone)]
pub struct OperationsReport {
    /// Label identifying the reporting period (e.g. "2025-Q1").
    pub period_label: String,
    /// Aggregate system KPIs for the period.
    pub kpis: OperationalKpis,
    /// Top congested branches: `(branch_name, avg_loading_pct)`.
    pub top_congested_branches: Vec<(String, f64)>,
    /// High-level renewable summary.
    pub renewable_summary: RenewableSummary,
    /// Auto-generated alerts triggered by threshold breaches.
    pub alerts: Vec<OperationsAlert>,
}

impl OperationsReport {
    /// Build a report from pre-computed KPIs, a congestion analyser, and renewable metrics.
    ///
    /// Alert thresholds applied:
    /// - Efficiency \[%\] < 85 → Warning; < 70 → Critical
    /// - Curtailment \[%\] > 10 → Warning; > 25 → Critical
    /// - Availability \[%\] < 95 → Warning
    pub fn generate(
        kpis: OperationalKpis,
        congestion: &CongestionAnalyzer,
        renewables: &RenewableMetrics,
    ) -> Self {
        // ── Congested branches ────────────────────────────────────────────────
        let top_congested_branches = match congestion.most_congested_branch() {
            Some((idx, loading)) => {
                let name = congestion
                    .branch_names
                    .get(idx)
                    .cloned()
                    .unwrap_or_else(|| format!("branch_{idx}"));
                vec![(name, loading)]
            }
            None => vec![],
        };

        // ── Renewable summary ─────────────────────────────────────────────────
        let renewable_summary = RenewableSummary {
            penetration_pct: renewables.annual_penetration_pct(),
            curtailment_pct: renewables.curtailment_rate_pct(),
            capacity_factor_pct: renewables.capacity_factor_pct(),
        };

        // ── Alerts ────────────────────────────────────────────────────────────
        let mut alerts: Vec<OperationsAlert> = Vec::new();

        let efficiency = kpis.system_efficiency_pct();
        if efficiency < 70.0 {
            alerts.push(OperationsAlert {
                severity: AlertSeverity::Critical,
                message: "System efficiency critically low".to_string(),
                value: efficiency,
                threshold: 70.0,
            });
        } else if efficiency < 85.0 {
            alerts.push(OperationsAlert {
                severity: AlertSeverity::Warning,
                message: "System efficiency below threshold".to_string(),
                value: efficiency,
                threshold: 85.0,
            });
        }

        let curtailment = renewables.curtailment_rate_pct();
        if curtailment > 25.0 {
            alerts.push(OperationsAlert {
                severity: AlertSeverity::Critical,
                message: "Very high renewable curtailment".to_string(),
                value: curtailment,
                threshold: 25.0,
            });
        } else if curtailment > 10.0 {
            alerts.push(OperationsAlert {
                severity: AlertSeverity::Warning,
                message: "High renewable curtailment".to_string(),
                value: curtailment,
                threshold: 10.0,
            });
        }

        let availability = kpis.availability_factor_pct();
        if availability < 95.0 {
            alerts.push(OperationsAlert {
                severity: AlertSeverity::Warning,
                message: "Availability factor below 95%".to_string(),
                value: availability,
                threshold: 95.0,
            });
        }

        Self {
            period_label: "Auto-generated report".to_string(),
            kpis,
            top_congested_branches,
            renewable_summary,
            alerts,
        }
    }

    /// Returns `true` when at least one `Critical` alert is present.
    pub fn has_critical_alerts(&self) -> bool {
        self.alerts
            .iter()
            .any(|a| a.severity == AlertSeverity::Critical)
    }

    /// Plain-text executive summary of the report.
    pub fn summary_text(&self) -> String {
        let mut lines: Vec<String> = Vec::new();
        lines.push(format!("=== Operations Report: {} ===", self.period_label));
        lines.push(format!(
            "System Efficiency: {:.1}%",
            self.kpis.system_efficiency_pct()
        ));
        lines.push(format!("Load Factor: {:.1}%", self.kpis.load_factor_pct()));
        lines.push(format!(
            "Availability: {:.2}%",
            self.kpis.availability_factor_pct()
        ));
        lines.push(format!(
            "RE Penetration: {:.1}%  Curtailment: {:.1}%",
            self.renewable_summary.penetration_pct, self.renewable_summary.curtailment_pct
        ));
        if self.top_congested_branches.is_empty() {
            lines.push("No congested branches detected.".to_string());
        } else {
            for (name, load) in &self.top_congested_branches {
                lines.push(format!("Most Congested Branch: {name} @ {load:.1}%"));
            }
        }
        if self.alerts.is_empty() {
            lines.push("No alerts.".to_string());
        } else {
            for alert in &self.alerts {
                let sev = match alert.severity {
                    AlertSeverity::Info => "INFO",
                    AlertSeverity::Warning => "WARN",
                    AlertSeverity::Critical => "CRIT",
                };
                lines.push(format!(
                    "[{}] {} (value={:.2}, threshold={:.2})",
                    sev, alert.message, alert.value, alert.threshold
                ));
            }
        }
        lines.join("\n")
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn base_kpis() -> OperationalKpis {
        OperationalKpis {
            period_hours: 8760.0,
            total_energy_generated_mwh: 1_000_000.0,
            total_energy_delivered_mwh: 950_000.0,
            peak_load_mw: 200.0,
            average_load_mw: 100.0,
            fuel_consumed_mmbtu: 8_000_000.0,
            co2_emitted_tonnes: 400_000.0,
            outage_hours: 876.0,
            outage_customers: 1_000,
            maintenance_cost: 500_000.0,
            fuel_cost: 4_000_000.0,
        }
    }

    // ── OperationalKpis ───────────────────────────────────────────────────────

    #[test]
    fn test_kpis_load_factor_le_100() {
        let k = base_kpis();
        let lf = k.load_factor_pct();
        assert!(lf <= 100.0, "load_factor must be ≤ 100%, got {lf}");
        assert!((lf - 50.0).abs() < 1e-9, "expected 50.0, got {lf}");
    }

    #[test]
    fn test_kpis_system_efficiency_lt_100() {
        let k = base_kpis();
        let eff = k.system_efficiency_pct();
        assert!(eff < 100.0, "efficiency should be < 100%, got {eff}");
        assert!((eff - 95.0).abs() < 1e-9, "expected 95.0, got {eff}");
    }

    #[test]
    fn test_kpis_saidi_calculation() {
        // outage_hours=2, outage_customers=500, total=10_000 → 2*60*500/10000 = 6.0
        let k = OperationalKpis {
            period_hours: 8760.0,
            total_energy_generated_mwh: 1_000.0,
            total_energy_delivered_mwh: 950.0,
            peak_load_mw: 200.0,
            average_load_mw: 100.0,
            fuel_consumed_mmbtu: 8_000.0,
            co2_emitted_tonnes: 400.0,
            outage_hours: 2.0,
            outage_customers: 500,
            maintenance_cost: 0.0,
            fuel_cost: 0.0,
        };
        let saidi = k.saidi_minutes(10_000);
        assert!(
            (saidi - 6.0).abs() < 1e-9,
            "expected SAIDI=6.0, got {saidi}"
        );
    }

    #[test]
    fn test_kpis_availability() {
        // period=8760, outage=876 → (8760-876)/8760*100 = 90.0
        let k = base_kpis();
        let avail = k.availability_factor_pct();
        assert!((avail - 90.0).abs() < 1e-6, "expected 90.0%, got {avail}");
    }

    #[test]
    fn test_kpis_zero_denominators() {
        let k = OperationalKpis {
            period_hours: 0.0,
            total_energy_generated_mwh: 0.0,
            total_energy_delivered_mwh: 0.0,
            peak_load_mw: 0.0,
            average_load_mw: 0.0,
            fuel_consumed_mmbtu: 0.0,
            co2_emitted_tonnes: 0.0,
            outage_hours: 0.0,
            outage_customers: 0,
            maintenance_cost: 0.0,
            fuel_cost: 0.0,
        };
        assert_eq!(k.system_efficiency_pct(), 0.0);
        assert_eq!(k.load_factor_pct(), 0.0);
        assert_eq!(k.heat_rate_btu_per_kwh(), 0.0);
        assert_eq!(k.emissions_intensity_kg_per_mwh(), 0.0);
        assert_eq!(k.saidi_minutes(0), 0.0);
        assert_eq!(k.cost_per_mwh_delivered(), 0.0);
        assert_eq!(k.availability_factor_pct(), 0.0);
    }

    // ── GeneratorPerformance ──────────────────────────────────────────────────

    #[test]
    fn test_generator_capacity_factor_le_100() {
        let g = GeneratorPerformance {
            unit_id: 1,
            name: "Gas1".to_string(),
            fuel_type: GeneratorFuelType::NaturalGas,
            rated_mw: 100.0,
            generation_mwh: vec![80.0; 24],
            fuel_consumed: vec![800.0; 24],
            starts: vec![false; 24],
            outage_flags: vec![false; 24],
        };
        let cf = g.capacity_factor_pct();
        assert!(cf <= 100.0, "capacity factor must be ≤ 100%, got {cf}");
        assert!((cf - 80.0).abs() < 1e-9, "expected 80.0%, got {cf}");
    }

    #[test]
    fn test_generator_start_count() {
        let mut starts = vec![false; 10];
        starts[0] = true;
        starts[5] = true;
        let g = GeneratorPerformance {
            unit_id: 2,
            name: "Coal1".to_string(),
            fuel_type: GeneratorFuelType::Coal,
            rated_mw: 500.0,
            generation_mwh: vec![400.0; 10],
            fuel_consumed: vec![4_000.0; 10],
            starts,
            outage_flags: vec![false; 10],
        };
        assert_eq!(g.start_count(), 2);
    }

    #[test]
    fn test_generator_ramp_cycles() {
        // rated=100 MW, threshold=10 MW
        // gen=[0,60,0,60] → changes: |60-0|=60>10, |0-60|=60>10, |60-0|=60>10 → 3 cycles
        let g = GeneratorPerformance {
            unit_id: 3,
            name: "Wind1".to_string(),
            fuel_type: GeneratorFuelType::Wind,
            rated_mw: 100.0,
            generation_mwh: vec![0.0, 60.0, 0.0, 60.0],
            fuel_consumed: vec![0.0; 4],
            starts: vec![false; 4],
            outage_flags: vec![false; 4],
        };
        assert_eq!(g.ramp_cycles(), 3);
    }

    #[test]
    fn test_generator_efor() {
        let outage_flags = vec![
            true, true, false, false, false, false, false, false, false, false,
        ];
        let g = GeneratorPerformance {
            unit_id: 4,
            name: "Nuclear1".to_string(),
            fuel_type: GeneratorFuelType::Nuclear,
            rated_mw: 1_000.0,
            generation_mwh: vec![
                0.0, 0.0, 900.0, 900.0, 900.0, 900.0, 900.0, 900.0, 900.0, 900.0,
            ],
            fuel_consumed: vec![0.0; 10],
            starts: vec![false; 10],
            outage_flags,
        };
        // forced=2, in_service=8 → EFOR = 2/(2+8)*100 = 20%
        let efor = g.equivalent_forced_outage_rate_pct();
        assert!(
            (efor - 20.0).abs() < 1e-9,
            "expected EFOR=20.0%, got {efor}"
        );
    }

    // ── CongestionAnalyzer ────────────────────────────────────────────────────

    #[test]
    fn test_congestion_loading_pct() {
        let analyzer = CongestionAnalyzer {
            branch_ratings_mw: vec![100.0, 200.0],
            branch_flows_mw: vec![vec![90.0, 150.0]],
            branch_names: vec!["L1".to_string(), "L2".to_string()],
            lmp_prices: vec![vec![30.0, 35.0]],
        };
        let pct = analyzer.loading_pct(0, 0);
        assert!((pct - 90.0).abs() < 1e-9, "expected 90.0%, got {pct}");
        let pct2 = analyzer.loading_pct(1, 0);
        assert!((pct2 - 75.0).abs() < 1e-9, "expected 75.0%, got {pct2}");
    }

    #[test]
    fn test_congestion_most_congested() {
        // branch 0: 50% avg, branch 1: 90% avg → branch 1 is most congested
        let analyzer = CongestionAnalyzer {
            branch_ratings_mw: vec![100.0, 100.0],
            branch_flows_mw: vec![vec![50.0, 90.0], vec![50.0, 90.0]],
            branch_names: vec!["L1".to_string(), "L2".to_string()],
            lmp_prices: vec![vec![30.0], vec![32.0]],
        };
        let mc = analyzer.most_congested_branch();
        assert!(mc.is_some());
        let (idx, avg) = mc.unwrap();
        assert_eq!(idx, 1, "expected branch 1 to be most congested");
        assert!(
            (avg - 90.0).abs() < 1e-6,
            "expected avg loading 90.0%, got {avg}"
        );
    }

    #[test]
    fn test_congestion_interface_flow() {
        let analyzer = CongestionAnalyzer {
            branch_ratings_mw: vec![100.0, 100.0, 100.0],
            branch_flows_mw: vec![vec![40.0, 60.0, 20.0]],
            branch_names: vec!["A".to_string(), "B".to_string(), "C".to_string()],
            lmp_prices: vec![vec![30.0]],
        };
        let iflow = analyzer.interface_flow(&[0, 2], 0);
        assert!(
            (iflow - 60.0).abs() < 1e-9,
            "expected interface flow 60.0, got {iflow}"
        );
    }

    // ── RenewableMetrics ──────────────────────────────────────────────────────

    #[test]
    fn test_renewable_penetration_le_100() {
        let rm = RenewableMetrics {
            total_generation_mwh: vec![100.0],
            renewable_generation_mwh: vec![50.0],
            curtailed_mwh: vec![0.0],
            re_capacity_mw: 60.0,
            storage_charge_mwh: vec![0.0],
            storage_discharge_mwh: vec![0.0],
        };
        let pen = rm.penetration_pct_per_hour();
        assert!(!pen.is_empty());
        assert!(
            pen[0] <= 100.0,
            "penetration must be ≤ 100%, got {}",
            pen[0]
        );
        assert!(
            (pen[0] - 50.0).abs() < 1e-9,
            "expected 50.0%, got {}",
            pen[0]
        );
    }

    #[test]
    fn test_renewable_curtailment_rate() {
        // curtailed=10, dispatched=90 → 10/(10+90)*100 = 10%
        let rm = RenewableMetrics {
            total_generation_mwh: vec![100.0],
            renewable_generation_mwh: vec![90.0],
            curtailed_mwh: vec![10.0],
            re_capacity_mw: 110.0,
            storage_charge_mwh: vec![0.0],
            storage_discharge_mwh: vec![0.0],
        };
        let cr = rm.curtailment_rate_pct();
        assert!(
            (cr - 10.0).abs() < 1e-9,
            "expected curtailment 10.0%, got {cr}"
        );
    }

    // ── DemandAnalytics ───────────────────────────────────────────────────────

    #[test]
    fn test_demand_peak_ge_valley() {
        let da = DemandAnalytics {
            load_profile_mw: vec![10.0, 100.0, 50.0],
            temperature_c: vec![15.0, 30.0, 22.0],
            day_type: vec![DayType::Weekday, DayType::Weekday, DayType::Saturday],
        };
        assert!(da.peak_mw() >= da.valley_mw(), "peak must be ≥ valley");
        assert!((da.peak_mw() - 100.0).abs() < 1e-9);
        assert!((da.valley_mw() - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_demand_annual_energy() {
        // 8760 hours × 1000 MW = 8760 GWh
        let da = DemandAnalytics {
            load_profile_mw: vec![1_000.0; 8_760],
            temperature_c: vec![20.0; 8_760],
            day_type: vec![DayType::Weekday; 8_760],
        };
        let energy = da.annual_energy_gwh();
        assert!(
            (energy - 8_760.0).abs() < 1e-6,
            "expected 8760.0 GWh, got {energy}"
        );
    }

    #[test]
    fn test_demand_temperature_sensitivity_ols() {
        // T=[0,1,2], P=[10,12,14] → slope should be exactly 2.0
        let da = DemandAnalytics {
            load_profile_mw: vec![10.0, 12.0, 14.0],
            temperature_c: vec![0.0, 1.0, 2.0],
            day_type: vec![DayType::Weekday; 3],
        };
        let slope = da.temperature_sensitivity_mw_per_c();
        assert!(
            (slope - 2.0).abs() < 1e-6,
            "expected OLS slope=2.0, got {slope}"
        );
    }

    #[test]
    fn test_demand_flexibility_potential() {
        // peak=100, average=(100+50+150)/3=100 → potential=0%?
        // use asymmetric: [50,50,200] avg=100, peak=200 → (200-100)/200*100=50%
        let da = DemandAnalytics {
            load_profile_mw: vec![50.0, 50.0, 200.0],
            temperature_c: vec![20.0; 3],
            day_type: vec![DayType::Weekday; 3],
        };
        let flex = da.demand_flexibility_potential_pct();
        assert!(
            (flex - 50.0).abs() < 1e-6,
            "expected flexibility 50.0%, got {flex}"
        );
    }

    // ── OperationsReport ──────────────────────────────────────────────────────

    #[test]
    fn test_operations_report_has_critical_alerts() {
        // efficiency < 70% → Critical
        let kpis = OperationalKpis {
            period_hours: 100.0,
            total_energy_generated_mwh: 1_000.0,
            total_energy_delivered_mwh: 500.0, // 50% efficiency → Critical
            peak_load_mw: 20.0,
            average_load_mw: 10.0,
            fuel_consumed_mmbtu: 10_000.0,
            co2_emitted_tonnes: 500.0,
            outage_hours: 0.0,
            outage_customers: 0,
            maintenance_cost: 0.0,
            fuel_cost: 0.0,
        };
        let congestion = CongestionAnalyzer {
            branch_ratings_mw: vec![],
            branch_flows_mw: vec![],
            branch_names: vec![],
            lmp_prices: vec![],
        };
        let renewables = RenewableMetrics {
            total_generation_mwh: vec![1_000.0],
            renewable_generation_mwh: vec![200.0],
            curtailed_mwh: vec![0.0],
            re_capacity_mw: 300.0,
            storage_charge_mwh: vec![0.0],
            storage_discharge_mwh: vec![0.0],
        };
        let report = OperationsReport::generate(kpis, &congestion, &renewables);
        assert!(
            report.has_critical_alerts(),
            "expected at least one critical alert for 50% efficiency"
        );
    }

    #[test]
    fn test_operations_report_no_critical_alerts() {
        // High-performing system — no alerts expected
        let kpis = OperationalKpis {
            period_hours: 8760.0,
            total_energy_generated_mwh: 1_000_000.0,
            total_energy_delivered_mwh: 980_000.0, // 98% efficiency
            peak_load_mw: 200.0,
            average_load_mw: 115.0,
            fuel_consumed_mmbtu: 8_000_000.0,
            co2_emitted_tonnes: 400_000.0,
            outage_hours: 0.0,
            outage_customers: 0,
            maintenance_cost: 500_000.0,
            fuel_cost: 4_000_000.0,
        };
        let congestion = CongestionAnalyzer {
            branch_ratings_mw: vec![100.0],
            branch_flows_mw: vec![vec![50.0]],
            branch_names: vec!["L1".to_string()],
            lmp_prices: vec![vec![30.0]],
        };
        let renewables = RenewableMetrics {
            total_generation_mwh: vec![1_000.0; 100],
            renewable_generation_mwh: vec![200.0; 100],
            curtailed_mwh: vec![0.0; 100],
            re_capacity_mw: 250.0,
            storage_charge_mwh: vec![0.0; 100],
            storage_discharge_mwh: vec![0.0; 100],
        };
        let report = OperationsReport::generate(kpis, &congestion, &renewables);
        assert!(
            !report.has_critical_alerts(),
            "high-performing system should have no critical alerts"
        );
    }

    // ── OperationalKpis: non-zero derived metrics ─────────────────────────────

    #[test]
    fn test_kpis_heat_rate_and_emissions_and_cost() {
        // Reason: exercises heat_rate_btu_per_kwh, emissions_intensity_kg_per_mwh,
        // and cost_per_mwh_delivered on non-zero inputs — all three are uncovered.
        //
        // heat_rate = 2_000_000 MMBTU * 1e6 / (1_000_000 MWh * 1e3)
        //           = 2e12 / 1e9 = 2000 BTU/kWh
        // emissions = 400_000 t * 1000 / 950_000 MWh ≈ 421.053 kg/MWh
        // cost      = (500_000 + 4_000_000) / 950_000 ≈ 4.7368 $/MWh
        let k = base_kpis(); // generated=1e6 MWh, delivered=950k MWh, fuel=8e6 MMBTU
                             // Override fuel to 2e6 for a rounder heat-rate:
        let k2 = OperationalKpis {
            fuel_consumed_mmbtu: 2_000_000.0,
            ..k.clone()
        };
        approx::assert_relative_eq!(k2.heat_rate_btu_per_kwh(), 2000.0, epsilon = 1e-6);

        let ei = k.emissions_intensity_kg_per_mwh();
        approx::assert_relative_eq!(ei, 400_000.0 * 1000.0 / 950_000.0, epsilon = 1e-6);

        let cost = k.cost_per_mwh_delivered();
        approx::assert_relative_eq!(cost, (500_000.0 + 4_000_000.0) / 950_000.0, epsilon = 1e-6);
    }

    // ── GeneratorPerformance: heat rate and EOH ───────────────────────────────

    #[test]
    fn test_generator_average_heat_rate() {
        // Reason: average_heat_rate is not exercised by any existing test.
        // 24 h × 80 MWh = 1920 MWh total; 24 h × 800 MMBTU = 19200 MMBTU total
        // heat_rate = 19200 * 1e6 BTU / (1920 * 1e3 kWh) = 10000 BTU/kWh
        let g = GeneratorPerformance {
            unit_id: 5,
            name: "Gas2".to_string(),
            fuel_type: GeneratorFuelType::NaturalGas,
            rated_mw: 100.0,
            generation_mwh: vec![80.0; 24],
            fuel_consumed: vec![800.0; 24],
            starts: vec![false; 24],
            outage_flags: vec![false; 24],
        };
        approx::assert_relative_eq!(g.average_heat_rate(), 10_000.0, epsilon = 1e-6);
    }

    #[test]
    fn test_generator_equivalent_operating_hours() {
        // Reason: equivalent_operating_hours is uncovered.
        // 3 starts: hot_starts=1, cold_starts=2
        // run_hours = 7 (outage_flags has 3 trues out of 10)
        // EOH = 7 + 5*1 + 10*2 = 7 + 5 + 20 = 32
        let mut starts = vec![false; 10];
        starts[0] = true;
        starts[3] = true;
        starts[7] = true;
        let mut outage_flags = vec![false; 10];
        outage_flags[1] = true;
        outage_flags[5] = true;
        outage_flags[9] = true;
        let g = GeneratorPerformance {
            unit_id: 6,
            name: "Hydro1".to_string(),
            fuel_type: GeneratorFuelType::Hydro,
            rated_mw: 200.0,
            generation_mwh: vec![150.0; 10],
            fuel_consumed: vec![0.0; 10],
            starts,
            outage_flags,
        };
        approx::assert_relative_eq!(g.equivalent_operating_hours(), 32.0, epsilon = 1e-9);
    }

    // ── CongestionAnalyzer: congested hours and overload cost ─────────────────

    #[test]
    fn test_congestion_congested_hours_and_cost() {
        // Reason: congested_hours_per_branch and congestion_cost_per_hour are uncovered.
        // hour 0: branch0 flow=95 MW on rating=100 → 95% > 90% → congested
        // hour 1: branch0 flow=80 MW on rating=100 → 80% ≤ 90% → not congested
        // congestion cost hour 0: overload = max(0, 95-100)=0 → cost=0
        // hour 2: branch0 flow=120 MW on rating=100 → overload=20 MW, avg_lmp=40 → cost=800
        let ca = CongestionAnalyzer {
            branch_ratings_mw: vec![100.0],
            branch_flows_mw: vec![vec![95.0], vec![80.0], vec![120.0]],
            branch_names: vec!["TX1".to_string()],
            lmp_prices: vec![vec![30.0], vec![35.0], vec![40.0]],
        };
        let ch = ca.congested_hours_per_branch();
        assert_eq!(ch.len(), 1);
        // hour-0: 95/100 = 95% > 90% → congested
        // hour-1: 80/100 = 80% ≤ 90% → not congested
        // hour-2: 120/100 = 120% > 90% → congested
        assert_eq!(
            ch[0], 2,
            "hours 0 and 2 should be congested (95% and 120% loading)"
        );

        let costs = ca.congestion_cost_per_hour(&[]);
        assert_eq!(costs.len(), 3);
        approx::assert_relative_eq!(costs[0], 0.0, epsilon = 1e-9); // not overloaded
        approx::assert_relative_eq!(costs[1], 0.0, epsilon = 1e-9);
        approx::assert_relative_eq!(costs[2], 20.0 * 40.0, epsilon = 1e-6); // overload=20, lmp=40
    }

    #[test]
    fn test_congestion_lmp_spread() {
        // Reason: lmp_spread_mwh is uncovered.
        // hour 0: buses [20, 50, 35] → spread = 30
        // hour 1: buses [45, 45]     → spread = 0
        let ca = CongestionAnalyzer {
            branch_ratings_mw: vec![100.0],
            branch_flows_mw: vec![vec![50.0], vec![50.0]],
            branch_names: vec!["L1".to_string()],
            lmp_prices: vec![vec![20.0, 50.0, 35.0], vec![45.0, 45.0]],
        };
        let spread = ca.lmp_spread_mwh();
        assert_eq!(spread.len(), 2);
        approx::assert_relative_eq!(spread[0], 30.0, epsilon = 1e-9);
        approx::assert_relative_eq!(spread[1], 0.0, epsilon = 1e-9);
    }

    // ── RenewableMetrics: capacity factor, storage util, variability ──────────

    #[test]
    fn test_renewable_capacity_factor_and_storage_util() {
        // Reason: capacity_factor_pct and storage_utilization_pct are uncovered.
        // 4 hours, re_capacity=100 MW, re_gen=[60,60,60,60] → CF=60%
        // storage_discharge=[10,10,10,10] → util = 40/(100*4)*100 = 10%
        let rm = RenewableMetrics {
            total_generation_mwh: vec![100.0; 4],
            renewable_generation_mwh: vec![60.0; 4],
            curtailed_mwh: vec![0.0; 4],
            re_capacity_mw: 100.0,
            storage_charge_mwh: vec![5.0; 4],
            storage_discharge_mwh: vec![10.0; 4],
        };
        approx::assert_relative_eq!(rm.capacity_factor_pct(), 60.0, epsilon = 1e-9);
        approx::assert_relative_eq!(rm.storage_utilization_pct(), 10.0, epsilon = 1e-9);
    }

    #[test]
    fn test_renewable_hours_above_penetration_and_variability() {
        // Reason: hours_above_penetration and variability_index are uncovered.
        // 4 hours: penetrations = [0%, 50%, 75%, 100%]
        // hours above 60% = 2  (hours at 75% and 100%)
        // re_gen=[0, 50, 75, 100], total=[100,100,100,100]
        // mean_re=56.25, std_re=sqrt(((56.25^2 + 6.25^2 + 18.75^2 + 43.75^2)/4))
        // Just verify it is > 0 and ≤ 1 for a sensible series.
        let rm = RenewableMetrics {
            total_generation_mwh: vec![100.0; 4],
            renewable_generation_mwh: vec![0.0, 50.0, 75.0, 100.0],
            curtailed_mwh: vec![0.0; 4],
            re_capacity_mw: 100.0,
            storage_charge_mwh: vec![0.0; 4],
            storage_discharge_mwh: vec![0.0; 4],
        };
        assert_eq!(rm.hours_above_penetration(60.0), 2);
        let vi = rm.variability_index();
        assert!(
            vi > 0.0,
            "variability index must be positive for non-flat series"
        );
        assert!(
            vi <= 1.0,
            "variability index should be ≤ 1.0 for this series"
        );
    }

    // ── DemandAnalytics: load duration, peak hour, weekday ratio ─────────────

    #[test]
    fn test_demand_load_duration_curve_and_peak_hour() {
        // Reason: load_duration_curve and peak_hour_of_year are uncovered.
        // Profile: [30, 10, 70, 50] — peak is at index 2
        let da = DemandAnalytics {
            load_profile_mw: vec![30.0, 10.0, 70.0, 50.0],
            temperature_c: vec![20.0; 4],
            day_type: vec![DayType::Weekday; 4],
        };
        let ldc = da.load_duration_curve();
        assert_eq!(ldc.len(), 4, "LDC should have one entry per hour");
        // LDC is descending: first entry is (1, 70.0)
        approx::assert_relative_eq!(ldc[0].1, 70.0, epsilon = 1e-9);
        approx::assert_relative_eq!(ldc[1].1, 50.0, epsilon = 1e-9);
        assert_eq!(da.peak_hour_of_year(), 2, "peak hour index should be 2");
    }

    #[test]
    fn test_demand_weekday_vs_weekend_ratio() {
        // Reason: weekday_vs_weekend_ratio is uncovered.
        // 3 weekday hours avg=100, 2 weekend hours avg=60 → ratio = 100/60 ≈ 1.6667
        let da = DemandAnalytics {
            load_profile_mw: vec![90.0, 100.0, 110.0, 60.0, 60.0],
            temperature_c: vec![20.0; 5],
            day_type: vec![
                DayType::Weekday,
                DayType::Weekday,
                DayType::Weekday,
                DayType::Saturday,
                DayType::Sunday,
            ],
        };
        approx::assert_relative_eq!(da.weekday_vs_weekend_ratio(), 100.0 / 60.0, epsilon = 1e-9);
    }

    #[test]
    fn test_demand_temperature_sensitivity_edge_cases() {
        // Reason: the zero-variance and mismatched-length error paths of
        // temperature_sensitivity_mw_per_c are uncovered.
        let da_const_temp = DemandAnalytics {
            load_profile_mw: vec![100.0, 120.0, 110.0],
            temperature_c: vec![25.0, 25.0, 25.0], // constant T → zero variance
            day_type: vec![DayType::Weekday; 3],
        };
        assert_eq!(
            da_const_temp.temperature_sensitivity_mw_per_c(),
            0.0,
            "constant temperature should yield slope 0.0"
        );

        let da_mismatch = DemandAnalytics {
            load_profile_mw: vec![100.0, 120.0],
            temperature_c: vec![20.0], // length mismatch
            day_type: vec![DayType::Weekday; 2],
        };
        assert_eq!(
            da_mismatch.temperature_sensitivity_mw_per_c(),
            0.0,
            "mismatched lengths should yield slope 0.0"
        );
    }

    // ── OperationsReport: curtailment warning band and summary_text ───────────

    #[test]
    fn test_operations_report_curtailment_warning_and_summary_text() {
        // Reason: the curtailment Warning band (10 < curtailment ≤ 25) and
        // summary_text are uncovered.
        // Set curtailment = 15%: curtailed=15, dispatched=85 → 15/100 = 15%
        let kpis = OperationalKpis {
            period_hours: 100.0,
            total_energy_generated_mwh: 10_000.0,
            total_energy_delivered_mwh: 9_500.0, // 95% eff → no efficiency alert
            peak_load_mw: 100.0,
            average_load_mw: 95.0,
            fuel_consumed_mmbtu: 50_000.0,
            co2_emitted_tonnes: 2_000.0,
            outage_hours: 0.0,
            outage_customers: 0,
            maintenance_cost: 0.0,
            fuel_cost: 0.0,
        };
        let congestion = CongestionAnalyzer {
            branch_ratings_mw: vec![],
            branch_flows_mw: vec![],
            branch_names: vec![],
            lmp_prices: vec![],
        };
        let renewables = RenewableMetrics {
            total_generation_mwh: vec![100.0],
            renewable_generation_mwh: vec![85.0],
            curtailed_mwh: vec![15.0], // 15% curtailment → Warning
            re_capacity_mw: 100.0,
            storage_charge_mwh: vec![0.0],
            storage_discharge_mwh: vec![0.0],
        };
        let report = OperationsReport::generate(kpis, &congestion, &renewables);
        // Should have a Warning but no Critical alert
        assert!(
            !report.has_critical_alerts(),
            "15% curtailment is warning, not critical"
        );
        let has_curtailment_warn = report
            .alerts
            .iter()
            .any(|a| a.severity == AlertSeverity::Warning && a.message.contains("curtailment"));
        assert!(has_curtailment_warn, "expected a curtailment Warning alert");

        // Verify summary_text runs without panicking and contains key fields
        let text = report.summary_text();
        assert!(
            text.contains("Operations Report"),
            "summary should contain header"
        );
        assert!(
            text.contains("RE Penetration"),
            "summary should mention RE penetration"
        );
    }
}
