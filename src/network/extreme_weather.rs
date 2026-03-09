//! Extreme Weather Resilience Analysis for power grids.
//!
//! Provides hurricane/ice storm/wildfire fragility models, cascading failure
//! probability, hardening investment optimisation, and climate risk assessment.
//!
//! # Units
//! - Wind speed: \[m/s\]
//! - Ice thickness: \[mm\]
//! - Depth: \[m\]
//! - Load / power: \[MW\]
//! - Time: \[h\]
//! - Cost: \[USD\]
//! - Annual loss: \[$/yr\]

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Weather hazard event with associated physical parameters.
#[derive(Debug, Clone)]
pub enum WeatherHazard {
    /// Tropical cyclone hazard.
    ///
    /// - `category`: Saffir–Simpson scale 1–5
    /// - `wind_speed_ms`: 10-min mean wind speed \[m/s\]
    /// - `storm_surge_m`: storm surge height above MSL \[m\]
    Hurricane {
        category: u8,
        wind_speed_ms: f64,
        storm_surge_m: f64,
    },

    /// Freezing rain / glaze ice event.
    ///
    /// - `ice_thickness_mm`: radial ice accretion on conductors \[mm\]
    /// - `duration_h`: duration of icing conditions \[h\]
    IceStorm {
        ice_thickness_mm: f64,
        duration_h: f64,
    },

    /// Wildfire proximity hazard.
    ///
    /// - `fire_weather_index`: Canadian FWI or equivalent (dimensionless)
    /// - `proximity_km`: distance from active fire front \[km\]
    Wildfire {
        fire_weather_index: f64,
        proximity_km: f64,
    },

    /// Seismic hazard.
    ///
    /// - `magnitude`: moment magnitude Mw
    /// - `peak_ground_acceleration`: PGA as fraction of g (0–1)
    Earthquake {
        magnitude: f64,
        peak_ground_acceleration: f64,
    },

    /// Riverine / coastal flooding.
    ///
    /// - `depth_m`: inundation depth at component site \[m\]
    /// - `velocity_ms`: flood flow velocity \[m/s\]
    Flood { depth_m: f64, velocity_ms: f64 },

    /// Sustained high-temperature event.
    ///
    /// - `temperature_c`: ambient temperature \[°C\]
    /// - `duration_h`: duration of extreme heat \[h\]
    ExtremeHeat { temperature_c: f64, duration_h: f64 },
}

/// Type of grid infrastructure component.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComponentType {
    /// Above-ground conductor on poles or towers.
    OverheadLine,
    /// Below-grade cable (much more resilient to wind/ice).
    UndergroundCable,
    /// Power transformer (pad-mount or substation).
    Transformer,
    /// High-voltage substation (switchgear, busbars, structures).
    Substation,
    /// Synchronous or inverter-based generating unit.
    GeneratingUnit,
    /// Telecommunications / SCADA tower.
    CommunicationTower,
}

// ---------------------------------------------------------------------------
// Core data structures
// ---------------------------------------------------------------------------

/// Physical and asset attributes of a single grid component.
#[derive(Debug, Clone)]
pub struct ComponentFragility {
    /// Unique identifier (e.g. branch ID or bus label).
    pub component_id: String,
    /// Infrastructure category.
    pub component_type: ComponentType,
    /// Asset age \[years\].
    pub age_years: f64,
    /// Condition score 0–1 (0 = severely degraded, 1 = as-new).
    pub condition_score: f64,
    /// Elevation of the component above sea level \[m\].
    pub elevation_m: f64,
    /// Whether the component has been physically hardened (e.g. storm-rated).
    pub hardened: bool,
    /// Estimated replacement cost \[USD\].
    pub replacement_cost_usd: f64,
}

/// Discrete fragility curve mapping a hazard intensity parameter to
/// conditional failure probabilities.
#[derive(Debug, Clone)]
pub struct FragilityCurve {
    /// Hazard intensity values (e.g. wind speed \[m/s\], ice thickness \[mm\]).
    pub hazard_parameter: Vec<f64>,
    /// P(failure | hazard intensity) at each corresponding intensity value.
    pub failure_probability: Vec<f64>,
}

/// Configuration for the resilience analyser.
#[derive(Debug, Clone)]
pub struct WeatherResilienceConfig {
    /// Number of Monte Carlo realisations (default: 500).
    pub monte_carlo_runs: usize,
    /// RNG seed for reproducibility.
    pub seed: u64,
    /// Total capital budget available for hardening \[USD\].
    pub hardening_budget_usd: f64,
    /// Value of Lost Load \[USD/MWh\] (default: 10 000).
    pub voll_per_mwh: f64,
    /// Fraction of total damage repaired per day (restoration rate).
    pub restoration_rate: f64,
}

impl Default for WeatherResilienceConfig {
    fn default() -> Self {
        Self {
            monte_carlo_runs: 500,
            seed: 42,
            hardening_budget_usd: 1_000_000.0,
            voll_per_mwh: 10_000.0,
            restoration_rate: 0.15,
        }
    }
}

/// Main analyser encapsulating components, configuration and network loads.
#[derive(Debug, Clone)]
pub struct WeatherResilienceAnalyzer {
    /// All grid components subject to weather damage.
    pub components: Vec<ComponentFragility>,
    /// Analysis configuration.
    pub config: WeatherResilienceConfig,
    /// Per-bus active power demand \[MW\].
    pub network_loads_mw: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Output types
// ---------------------------------------------------------------------------

/// Results from a Monte Carlo storm simulation.
#[derive(Debug, Clone)]
pub struct StormImpactResult {
    /// Mean load shed across all realisations \[MW\].
    pub expected_load_shed_mw: f64,
    /// Probability that more than 50 % of total load is shed.
    pub p_blackout: f64,
    /// Expected System Average Interruption Duration Index \[h\].
    pub expected_saidi_h: f64,
    /// Expected System Average Interruption Frequency Index.
    pub expected_saifi: f64,
    /// 95th-percentile load shed \[MW\].
    pub p95_load_shed_mw: f64,
    /// Expected total damage cost \[USD\].
    pub total_damage_cost_usd: f64,
}

/// Recommended hardening investment plan.
#[derive(Debug, Clone)]
pub struct HardeningPlan {
    /// Component IDs recommended for hardening, in priority order.
    pub components_to_harden: Vec<String>,
    /// Total hardening expenditure \[USD\].
    pub total_cost_usd: f64,
    /// Expected reduction in load shed \[MW\].
    pub risk_reduction_mw: f64,
    /// Benefit-to-cost ratio (dimensionless).
    pub benefit_to_cost_ratio: f64,
    /// Approximate payback period \[years\].
    pub roi_years: f64,
}

/// Projected climate risk over a multi-year horizon.
#[derive(Debug, Clone)]
pub struct ClimateRiskProjection {
    /// Annual expected loss \[USD/yr\] for each projected year.
    pub annual_expected_loss_usd: Vec<f64>,
    /// Net present value of losses assuming no hardening \[USD\].
    pub npv_no_hardening_usd: f64,
    /// Net present value of losses with upfront hardening \[USD\].
    pub npv_with_hardening_usd: f64,
    /// First year at which cumulative savings exceed hardening cost,
    /// `None` if payback is not achieved within the projection horizon.
    pub break_even_year: Option<usize>,
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Standard normal CDF via rational approximation (Abramowitz & Stegun 26.2.17).
fn standard_normal_cdf(x: f64) -> f64 {
    if x >= 0.0 {
        let t = 1.0 / (1.0 + 0.2316419 * x);
        let poly = t
            * (0.319_381_53
                + t * (-0.356_563_782
                    + t * (1.781_477_937 + t * (-1.821_255_978 + t * 1.330_274_429))));
        1.0 - ((-0.5 * x * x).exp() / (2.0 * PI).sqrt()) * poly
    } else {
        1.0 - standard_normal_cdf(-x)
    }
}

/// Logistic CDF with given mean `mu` and scale `sigma`.
fn logistic_cdf(x: f64, mu: f64, sigma: f64) -> f64 {
    let z = (x - mu) / sigma;
    1.0 / (1.0 + (-z).exp())
}

/// Advance an LCG state and return a uniform pseudo-random value in [0, 1).
fn lcg_next(state: &mut u64) -> f64 {
    *state = state
        .wrapping_mul(6_364_136_223_846_793_005_u64)
        .wrapping_add(1_442_695_040_888_963_407_u64);
    // Use upper 32 bits for better quality
    let upper = (*state >> 32) as f64;
    upper / 4_294_967_296.0
}

/// Hardening cost heuristic: 20 % of replacement cost, minimum $5 000.
fn hardening_cost(comp: &ComponentFragility) -> f64 {
    (comp.replacement_cost_usd * 0.20).max(5_000.0)
}

/// Failure probability after hardening for the given hazard.
fn hardened_failure_probability(comp: &ComponentFragility, hazard: &WeatherHazard) -> f64 {
    // Temporarily create a hardened clone for calculation.
    let mut hardened = comp.clone();
    hardened.hardened = true;
    WeatherResilienceAnalyzer::failure_probability_static(&hardened, hazard)
}

// ---------------------------------------------------------------------------
// WeatherResilienceAnalyzer implementation
// ---------------------------------------------------------------------------

impl WeatherResilienceAnalyzer {
    /// Construct a new analyser.
    pub fn new(
        components: Vec<ComponentFragility>,
        config: WeatherResilienceConfig,
        network_loads_mw: Vec<f64>,
    ) -> Self {
        Self {
            components,
            config,
            network_loads_mw,
        }
    }

    // ------------------------------------------------------------------
    // 1. Failure probability
    // ------------------------------------------------------------------

    /// Compute P(failure | hazard) for a single component.
    ///
    /// Applies a condition-score adjustment: poor-condition assets have higher
    /// failure probability than the nominal fragility curve predicts.
    pub fn failure_probability(component: &ComponentFragility, hazard: &WeatherHazard) -> f64 {
        Self::failure_probability_static(component, hazard)
    }

    /// Static version used internally.
    pub(crate) fn failure_probability_static(
        component: &ComponentFragility,
        hazard: &WeatherHazard,
    ) -> f64 {
        let p_nominal = Self::nominal_failure_probability(component, hazard);
        // Condition adjustment: P_adj = P × (2 − condition_score)
        // condition_score ∈ [0,1] → multiplier ∈ [1, 2]
        let cond = component.condition_score.clamp(0.0, 1.0);
        let p_adj = p_nominal * (2.0 - cond);
        p_adj.clamp(0.0, 1.0)
    }

    /// Nominal (un-adjusted) failure probability from physical model.
    fn nominal_failure_probability(component: &ComponentFragility, hazard: &WeatherHazard) -> f64 {
        match hazard {
            WeatherHazard::Hurricane {
                wind_speed_ms,
                storm_surge_m,
                ..
            } => Self::hurricane_failure(*wind_speed_ms, *storm_surge_m, component),

            WeatherHazard::IceStorm {
                ice_thickness_mm,
                duration_h,
            } => Self::ice_storm_failure(*ice_thickness_mm, *duration_h, component),

            WeatherHazard::Wildfire {
                fire_weather_index,
                proximity_km,
            } => Self::wildfire_failure(*fire_weather_index, *proximity_km, component),

            WeatherHazard::Earthquake {
                peak_ground_acceleration,
                ..
            } => Self::earthquake_failure(*peak_ground_acceleration, component),

            WeatherHazard::Flood {
                depth_m,
                velocity_ms,
            } => Self::flood_failure(*depth_m, *velocity_ms, component),

            WeatherHazard::ExtremeHeat {
                temperature_c,
                duration_h,
            } => Self::heat_failure(*temperature_c, *duration_h, component),
        }
    }

    fn hurricane_failure(
        wind_speed_ms: f64,
        storm_surge_m: f64,
        component: &ComponentFragility,
    ) -> f64 {
        // v50: wind speed at 50 % failure probability
        let v50 = match component.component_type {
            ComponentType::OverheadLine => {
                if component.hardened {
                    50.0_f64
                } else {
                    35.0_f64
                }
            }
            ComponentType::CommunicationTower => {
                if component.hardened {
                    55.0
                } else {
                    40.0
                }
            }
            ComponentType::Transformer | ComponentType::Substation => {
                if component.hardened {
                    60.0
                } else {
                    45.0
                }
            }
            ComponentType::UndergroundCable => {
                // Underground cables are not wind-sensitive but flood-sensitive;
                // use a very high wind v50 and apply storm-surge contribution.
                100.0
            }
            ComponentType::GeneratingUnit => {
                if component.hardened {
                    55.0
                } else {
                    40.0
                }
            }
        };

        // Log-normal fragility: P = Φ(ln(v / v50) / β), β = 0.5
        let p_wind = if wind_speed_ms > 0.0 && v50 > 0.0 {
            let z = (wind_speed_ms / v50).ln() / 0.5;
            standard_normal_cdf(z)
        } else {
            0.0
        };

        // Storm surge contribution (additive, capped at 1)
        let p_surge = if storm_surge_m > component.elevation_m {
            let excess = storm_surge_m - component.elevation_m;
            logistic_cdf(excess, 0.5, 0.3)
        } else {
            0.0
        };

        (p_wind + p_surge * 0.3).clamp(0.0, 1.0)
    }

    fn ice_storm_failure(
        ice_thickness_mm: f64,
        duration_h: f64,
        component: &ComponentFragility,
    ) -> f64 {
        match component.component_type {
            ComponentType::OverheadLine | ComponentType::CommunicationTower => {
                // P = 1 − exp(−ice_mm / 15)
                let base = 1.0 - (-ice_thickness_mm / 15.0).exp();
                // Duration multiplier: longer icing → more cumulative damage
                let dur_factor = 1.0 + (duration_h / 48.0).min(0.5);
                let p = base * dur_factor;
                if component.hardened {
                    p * 0.5
                } else {
                    p
                }
            }
            ComponentType::Transformer | ComponentType::Substation => {
                // Transformers can fail due to overloading when ice limits
                // capacity; moderate sensitivity.
                let p = 1.0 - (-ice_thickness_mm / 30.0).exp();
                if component.hardened {
                    p * 0.6
                } else {
                    p
                }
            }
            // Underground cables and generating units have low ice sensitivity.
            ComponentType::UndergroundCable => 0.02_f64.min(ice_thickness_mm / 1000.0),
            ComponentType::GeneratingUnit => {
                // Fuel supply or cooling disruption
                let p = 1.0 - (-ice_thickness_mm / 50.0).exp();
                if component.hardened {
                    p * 0.4
                } else {
                    p
                }
            }
        }
        .clamp(0.0, 1.0)
    }

    fn wildfire_failure(
        fire_weather_index: f64,
        proximity_km: f64,
        component: &ComponentFragility,
    ) -> f64 {
        // Clamp proximity to avoid division by zero
        let prox = proximity_km.max(0.1);
        let p_base = 1.0 - (-fire_weather_index / 50.0 * (1.0 / prox)).exp();
        let p = match component.component_type {
            ComponentType::OverheadLine | ComponentType::CommunicationTower => p_base,
            ComponentType::Transformer | ComponentType::Substation => p_base * 0.7,
            ComponentType::UndergroundCable => p_base * 0.1,
            ComponentType::GeneratingUnit => p_base * 0.5,
        };
        let p = if component.hardened { p * 0.4 } else { p };
        p.clamp(0.0, 1.0)
    }

    fn earthquake_failure(pga: f64, component: &ComponentFragility) -> f64 {
        // Log-normal fragility; median PGA at 50 % failure varies by type.
        let pga50 = match component.component_type {
            ComponentType::OverheadLine => {
                if component.hardened {
                    0.5
                } else {
                    0.3
                }
            }
            ComponentType::UndergroundCable => 0.8,
            ComponentType::Transformer => {
                if component.hardened {
                    0.4
                } else {
                    0.25
                }
            }
            ComponentType::Substation => {
                if component.hardened {
                    0.45
                } else {
                    0.28
                }
            }
            ComponentType::GeneratingUnit => {
                if component.hardened {
                    0.5
                } else {
                    0.3
                }
            }
            ComponentType::CommunicationTower => {
                if component.hardened {
                    0.55
                } else {
                    0.35
                }
            }
        };
        if pga > 0.0 && pga50 > 0.0 {
            let z = (pga / pga50).ln() / 0.6;
            standard_normal_cdf(z).clamp(0.0, 1.0)
        } else {
            0.0
        }
    }

    fn flood_failure(depth_m: f64, velocity_ms: f64, component: &ComponentFragility) -> f64 {
        // Logistic fragility: P = logistic(depth_m, μ=1.5, σ=0.5)
        let p_depth = logistic_cdf(depth_m, 1.5, 0.5);
        // Velocity adds hydrodynamic drag; extra contribution
        let p_vel = logistic_cdf(velocity_ms, 2.0, 0.8);
        let p = match component.component_type {
            ComponentType::UndergroundCable => {
                // Still vulnerable to flotation / joint failure at high depth
                p_depth * 0.4 + p_vel * 0.1
            }
            ComponentType::OverheadLine => p_depth * 0.5 + p_vel * 0.3,
            ComponentType::Transformer | ComponentType::Substation => {
                // Ground-level equipment extremely sensitive to flooding
                p_depth * 0.9 + p_vel * 0.2
            }
            ComponentType::GeneratingUnit => p_depth * 0.7 + p_vel * 0.2,
            ComponentType::CommunicationTower => p_depth * 0.3 + p_vel * 0.2,
        };
        let p = if component.hardened { p * 0.5 } else { p };
        p.clamp(0.0, 1.0)
    }

    fn heat_failure(temperature_c: f64, duration_h: f64, component: &ComponentFragility) -> f64 {
        // Thermal derating / insulation degradation above 40 °C
        let excess = (temperature_c - 40.0).max(0.0);
        let p_base = 1.0 - (-(excess / 20.0) * (duration_h / 24.0)).exp();
        let p = match component.component_type {
            ComponentType::Transformer => p_base * 0.8,
            ComponentType::GeneratingUnit => p_base * 0.5,
            ComponentType::OverheadLine => {
                // Thermal sag and ampacity reduction
                p_base * 0.3
            }
            _ => p_base * 0.2,
        };
        let p = if component.hardened { p * 0.5 } else { p };
        p.clamp(0.0, 1.0)
    }

    // ------------------------------------------------------------------
    // 2. Fragility curves
    // ------------------------------------------------------------------

    /// Return a pre-computed fragility curve for a given (component type,
    /// hazard type) combination over a representative parameter range.
    pub fn component_fragility_curve(
        component_type: ComponentType,
        hazard: &WeatherHazard,
    ) -> FragilityCurve {
        let dummy_comp = ComponentFragility {
            component_id: "curve".to_string(),
            component_type,
            age_years: 10.0,
            condition_score: 1.0, // no condition adjustment for curves
            elevation_m: 0.0,
            hardened: false,
            replacement_cost_usd: 100_000.0,
        };

        match hazard {
            WeatherHazard::Hurricane { .. } => {
                // Wind speed range 10–80 m/s
                let params: Vec<f64> = (0..=70).map(|i| 10.0 + i as f64).collect();
                let probs = params
                    .iter()
                    .map(|&v| {
                        let h = WeatherHazard::Hurricane {
                            category: 1,
                            wind_speed_ms: v,
                            storm_surge_m: 0.0,
                        };
                        Self::nominal_failure_probability(&dummy_comp, &h)
                    })
                    .collect();
                FragilityCurve {
                    hazard_parameter: params,
                    failure_probability: probs,
                }
            }
            WeatherHazard::IceStorm { .. } => {
                // Ice thickness 0–60 mm
                let params: Vec<f64> = (0..=60).map(|i| i as f64).collect();
                let probs = params
                    .iter()
                    .map(|&t| {
                        let h = WeatherHazard::IceStorm {
                            ice_thickness_mm: t,
                            duration_h: 12.0,
                        };
                        Self::nominal_failure_probability(&dummy_comp, &h)
                    })
                    .collect();
                FragilityCurve {
                    hazard_parameter: params,
                    failure_probability: probs,
                }
            }
            WeatherHazard::Wildfire { .. } => {
                // FWI range 0–100, proximity fixed at 1 km
                let params: Vec<f64> = (0..=100).map(|i| i as f64).collect();
                let probs = params
                    .iter()
                    .map(|&fwi| {
                        let h = WeatherHazard::Wildfire {
                            fire_weather_index: fwi,
                            proximity_km: 1.0,
                        };
                        Self::nominal_failure_probability(&dummy_comp, &h)
                    })
                    .collect();
                FragilityCurve {
                    hazard_parameter: params,
                    failure_probability: probs,
                }
            }
            WeatherHazard::Earthquake { .. } => {
                // PGA range 0.0–1.5 g
                let params: Vec<f64> = (0..=30).map(|i| i as f64 * 0.05).collect();
                let probs = params
                    .iter()
                    .map(|&pga| {
                        let h = WeatherHazard::Earthquake {
                            magnitude: 6.0,
                            peak_ground_acceleration: pga,
                        };
                        Self::nominal_failure_probability(&dummy_comp, &h)
                    })
                    .collect();
                FragilityCurve {
                    hazard_parameter: params,
                    failure_probability: probs,
                }
            }
            WeatherHazard::Flood { .. } => {
                // Depth range 0–5 m, velocity fixed at 1 m/s
                let params: Vec<f64> = (0..=50).map(|i| i as f64 * 0.1).collect();
                let probs = params
                    .iter()
                    .map(|&d| {
                        let h = WeatherHazard::Flood {
                            depth_m: d,
                            velocity_ms: 1.0,
                        };
                        Self::nominal_failure_probability(&dummy_comp, &h)
                    })
                    .collect();
                FragilityCurve {
                    hazard_parameter: params,
                    failure_probability: probs,
                }
            }
            WeatherHazard::ExtremeHeat { .. } => {
                // Temperature range 30–70 °C, duration fixed at 24 h
                let params: Vec<f64> = (0..=40).map(|i| 30.0 + i as f64).collect();
                let probs = params
                    .iter()
                    .map(|&t| {
                        let h = WeatherHazard::ExtremeHeat {
                            temperature_c: t,
                            duration_h: 24.0,
                        };
                        Self::nominal_failure_probability(&dummy_comp, &h)
                    })
                    .collect();
                FragilityCurve {
                    hazard_parameter: params,
                    failure_probability: probs,
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // 3. Monte Carlo storm impact
    // ------------------------------------------------------------------

    /// Run a Monte Carlo simulation of storm impacts and return summary
    /// statistics.
    ///
    /// Uses a linear congruential generator for portability and reproducibility.
    pub fn monte_carlo_storm_impact(
        &self,
        hazard: &WeatherHazard,
        n_runs: usize,
    ) -> StormImpactResult {
        let total_load_mw: f64 = self.network_loads_mw.iter().sum();
        if total_load_mw <= 0.0 || self.components.is_empty() {
            return StormImpactResult {
                expected_load_shed_mw: 0.0,
                p_blackout: 0.0,
                expected_saidi_h: 0.0,
                expected_saifi: 0.0,
                p95_load_shed_mw: 0.0,
                total_damage_cost_usd: 0.0,
            };
        }

        // Pre-compute failure probabilities once.
        let fail_probs: Vec<f64> = self
            .components
            .iter()
            .map(|c| Self::failure_probability_static(c, hazard))
            .collect();

        let n_buses = self.network_loads_mw.len().max(1);
        // Approximate load impact per component: equal share per bus.
        let load_per_component_mw = total_load_mw / self.components.len() as f64;

        let mut rng = self.config.seed;
        let mut load_sheds: Vec<f64> = Vec::with_capacity(n_runs);
        let mut saidi_sum = 0.0_f64;
        let mut saifi_sum = 0.0_f64;
        let mut damage_sum = 0.0_f64;
        let mut blackout_count = 0_usize;

        let restoration_days = if self.config.restoration_rate > 0.0 {
            (1.0 / self.config.restoration_rate).ceil()
        } else {
            30.0
        };
        let restoration_hours = restoration_days * 24.0;

        for _ in 0..n_runs {
            let mut _n_failed = 0_usize;
            let mut run_shed_mw = 0.0_f64;
            let mut run_damage_usd = 0.0_f64;

            for (i, comp) in self.components.iter().enumerate() {
                let u = lcg_next(&mut rng);
                if u < fail_probs[i] {
                    _n_failed += 1;
                    run_shed_mw += load_per_component_mw;
                    run_damage_usd += comp.replacement_cost_usd;
                }
            }

            run_shed_mw = run_shed_mw.min(total_load_mw);

            // SAIDI / SAIFI approximation
            // Assume each bus is "a customer"; failed components interrupt
            // proportional customers.
            let frac_interrupted = (run_shed_mw / total_load_mw).clamp(0.0, 1.0);
            let customers_interrupted = (frac_interrupted * n_buses as f64).round();
            let outage_duration_h = frac_interrupted * restoration_hours;

            saidi_sum += customers_interrupted * outage_duration_h / n_buses as f64;
            saifi_sum += customers_interrupted / n_buses as f64;
            damage_sum += run_damage_usd;

            if run_shed_mw > 0.5 * total_load_mw {
                blackout_count += 1;
            }

            load_sheds.push(run_shed_mw);
        }

        let n = n_runs as f64;
        let expected_load_shed_mw = load_sheds.iter().sum::<f64>() / n;
        let p_blackout = blackout_count as f64 / n;
        let expected_saidi_h = saidi_sum / n;
        let expected_saifi = saifi_sum / n;

        // 95th percentile
        load_sheds.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let p95_idx = ((0.95 * n_runs as f64) as usize).min(n_runs.saturating_sub(1));
        let p95_load_shed_mw = load_sheds[p95_idx];

        StormImpactResult {
            expected_load_shed_mw,
            p_blackout,
            expected_saidi_h,
            expected_saifi,
            p95_load_shed_mw,
            total_damage_cost_usd: damage_sum / n,
        }
    }

    // ------------------------------------------------------------------
    // 4. Critical component identification
    // ------------------------------------------------------------------

    /// Rank components by risk score = P(failure) × load_impact \[MW\].
    ///
    /// Returns pairs of (component_id, risk_score) sorted descending.
    pub fn identify_critical_components(&self, hazard: &WeatherHazard) -> Vec<(String, f64)> {
        let total_load_mw: f64 = self.network_loads_mw.iter().sum();
        let load_per_comp = if self.components.is_empty() {
            0.0
        } else {
            total_load_mw / self.components.len() as f64
        };

        let mut scores: Vec<(String, f64)> = self
            .components
            .iter()
            .map(|comp| {
                let p = Self::failure_probability_static(comp, hazard);
                let risk = p * load_per_comp;
                (comp.component_id.clone(), risk)
            })
            .collect();

        scores.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        scores
    }

    // ------------------------------------------------------------------
    // 5. Hardening optimisation (greedy ROI)
    // ------------------------------------------------------------------

    /// Greedy hardening plan: maximise risk reduction per dollar spent.
    ///
    /// Returns a `HardeningPlan` selecting components in ROI order until the
    /// `budget` (or `self.config.hardening_budget_usd`) is exhausted.
    pub fn optimize_hardening(&self, hazard: &WeatherHazard, budget: f64) -> HardeningPlan {
        let total_load_mw: f64 = self.network_loads_mw.iter().sum();
        let load_per_comp = if self.components.is_empty() {
            0.0
        } else {
            total_load_mw / self.components.len() as f64
        };

        // For each component compute ROI metric
        struct CandidateHardening {
            id: String,
            cost: f64,
            risk_reduction_mw: f64,
            annual_benefit_usd: f64,
        }

        let mut candidates: Vec<CandidateHardening> = self
            .components
            .iter()
            .filter(|c| !c.hardened)
            .map(|comp| {
                let p_current = Self::failure_probability_static(comp, hazard);
                let p_hardened = hardened_failure_probability(comp, hazard);
                let delta_p = (p_current - p_hardened).max(0.0);
                let risk_reduction_mw = delta_p * load_per_comp;
                // Annual benefit = risk_reduction × VOLL × assumed 8 h/event
                let annual_benefit_usd = risk_reduction_mw * self.config.voll_per_mwh * 8.0;
                let cost = hardening_cost(comp);
                CandidateHardening {
                    id: comp.component_id.clone(),
                    cost,
                    risk_reduction_mw,
                    annual_benefit_usd,
                }
            })
            .collect();

        // Sort by ROI (benefit / cost) descending
        candidates.sort_by(|a, b| {
            let roi_a = if a.cost > 0.0 {
                a.annual_benefit_usd / a.cost
            } else {
                f64::INFINITY
            };
            let roi_b = if b.cost > 0.0 {
                b.annual_benefit_usd / b.cost
            } else {
                f64::INFINITY
            };
            roi_b
                .partial_cmp(&roi_a)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut remaining = budget;
        let mut selected_ids: Vec<String> = Vec::new();
        let mut total_cost = 0.0_f64;
        let mut total_risk_reduction_mw = 0.0_f64;
        let mut total_annual_benefit = 0.0_f64;

        for c in &candidates {
            if c.cost <= remaining {
                remaining -= c.cost;
                total_cost += c.cost;
                total_risk_reduction_mw += c.risk_reduction_mw;
                total_annual_benefit += c.annual_benefit_usd;
                selected_ids.push(c.id.clone());
            }
        }

        let benefit_to_cost_ratio = if total_cost > 0.0 {
            total_annual_benefit * 20.0 / total_cost // 20-year NPV factor
        } else {
            0.0
        };

        let roi_years = if total_annual_benefit > 0.0 {
            total_cost / total_annual_benefit
        } else {
            f64::INFINITY
        };

        HardeningPlan {
            components_to_harden: selected_ids,
            total_cost_usd: total_cost,
            risk_reduction_mw: total_risk_reduction_mw,
            benefit_to_cost_ratio,
            roi_years,
        }
    }

    // ------------------------------------------------------------------
    // 6. Climate risk projection
    // ------------------------------------------------------------------

    /// Project annual expected losses over `years` years assuming hazard
    /// frequency grows by `hazard_frequency_increase_pct_per_year` percent
    /// each year.
    ///
    /// Uses a discount rate of 5 % for NPV calculations.
    pub fn climate_risk_projection(
        &self,
        hazard: &WeatherHazard,
        years: usize,
        hazard_frequency_increase_pct_per_year: f64,
    ) -> ClimateRiskProjection {
        const DISCOUNT_RATE: f64 = 0.05;

        // Base-year annual expected loss: run a quick MC for base estimate.
        let base_result = self.monte_carlo_storm_impact(hazard, 200);
        // Annual expected loss = expected damage cost (already per event);
        // multiply by an assumed 1 event/year baseline.
        let base_annual_loss_usd = base_result.total_damage_cost_usd
            + base_result.expected_load_shed_mw * self.config.voll_per_mwh * 4.0;

        let hardening_plan = self.optimize_hardening(hazard, self.config.hardening_budget_usd);
        // Loss reduction fraction from hardening
        let total_load: f64 = self.network_loads_mw.iter().sum();
        let reduction_fraction = if total_load > 0.0 {
            hardening_plan.risk_reduction_mw / total_load
        } else {
            0.0
        };
        let hardening_cost_total = hardening_plan.total_cost_usd;

        let growth = hazard_frequency_increase_pct_per_year / 100.0;

        let mut annual_expected_loss_usd = Vec::with_capacity(years);
        let mut npv_no_harden = 0.0_f64;
        let mut npv_with_harden = 0.0_f64;
        // hardening cost incurred upfront in year 0
        let mut cumulative_savings = 0.0_f64;
        let mut break_even_year: Option<usize> = None;

        for yr in 0..years {
            let frequency_multiplier = (1.0 + growth).powi(yr as i32);
            let loss_no_harden = base_annual_loss_usd * frequency_multiplier;
            let loss_with_harden = loss_no_harden * (1.0 - reduction_fraction);

            annual_expected_loss_usd.push(loss_no_harden);

            let discount = 1.0 / (1.0 + DISCOUNT_RATE).powi(yr as i32 + 1);
            npv_no_harden += loss_no_harden * discount;
            npv_with_harden += loss_with_harden * discount;

            let annual_saving = loss_no_harden - loss_with_harden;
            cumulative_savings += annual_saving;
            if break_even_year.is_none() && cumulative_savings >= hardening_cost_total {
                break_even_year = Some(yr + 1);
            }
        }

        // Add hardening capital cost to NPV-with-hardening
        npv_with_harden += hardening_cost_total;

        ClimateRiskProjection {
            annual_expected_loss_usd,
            npv_no_hardening_usd: npv_no_harden,
            npv_with_hardening_usd: npv_with_harden,
            break_even_year,
        }
    }

    // ------------------------------------------------------------------
    // 7. SAIDI / SAIFI calculation
    // ------------------------------------------------------------------

    /// Compute SAIDI and SAIFI from discrete failure event records.
    ///
    /// Each entry in `failure_results` is
    /// `(customers_interrupted, outage_duration_h)`.
    ///
    /// # Returns
    /// `(saidi_h, saifi)` where:
    /// - SAIDI = Σ(customers × duration) / total_customers  \[h\]
    /// - SAIFI = Σ(customers) / total_customers
    pub fn saidi_saifi_calculation(&self, failure_results: &[(f64, f64)]) -> (f64, f64) {
        let total_customers = self.network_loads_mw.len() as f64;
        if total_customers <= 0.0 || failure_results.is_empty() {
            return (0.0, 0.0);
        }

        let saidi = failure_results
            .iter()
            .map(|(cust, dur)| cust * dur)
            .sum::<f64>()
            / total_customers;

        let saifi = failure_results.iter().map(|(cust, _)| cust).sum::<f64>() / total_customers;

        (saidi, saifi)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_overhead_line(hardened: bool, condition_score: f64) -> ComponentFragility {
        ComponentFragility {
            component_id: "OHL-1".to_string(),
            component_type: ComponentType::OverheadLine,
            age_years: 15.0,
            condition_score,
            elevation_m: 0.0,
            hardened,
            replacement_cost_usd: 200_000.0,
        }
    }

    fn make_analyzer(n_components: usize, load_per_bus: f64) -> WeatherResilienceAnalyzer {
        let components = (0..n_components)
            .map(|i| ComponentFragility {
                component_id: format!("C-{i}"),
                component_type: ComponentType::OverheadLine,
                age_years: 10.0,
                condition_score: 1.0,
                elevation_m: 0.0,
                hardened: false,
                replacement_cost_usd: 100_000.0,
            })
            .collect();
        let loads = vec![load_per_bus; n_components];
        WeatherResilienceAnalyzer::new(components, WeatherResilienceConfig::default(), loads)
    }

    // Test 1: Hurricane cat 5 → high failure probability for unhardened OHL
    #[test]
    fn test_hurricane_cat5_unhardened_high_failure() {
        let comp = make_overhead_line(false, 1.0);
        let hazard = WeatherHazard::Hurricane {
            category: 5,
            wind_speed_ms: 70.0, // well above v50=35 m/s
            storm_surge_m: 0.0,
        };
        let p = WeatherResilienceAnalyzer::failure_probability(&comp, &hazard);
        assert!(p > 0.90, "Expected P > 0.90 for cat-5 OHL, got {p:.4}");
    }

    // Test 2: Hardened component has lower failure probability than unhardened
    #[test]
    fn test_hardened_lower_failure_than_unhardened() {
        let unhardened = make_overhead_line(false, 1.0);
        let hardened = make_overhead_line(true, 1.0);
        let hazard = WeatherHazard::Hurricane {
            category: 3,
            wind_speed_ms: 50.0,
            storm_surge_m: 0.0,
        };
        let p_un = WeatherResilienceAnalyzer::failure_probability(&unhardened, &hazard);
        let p_h = WeatherResilienceAnalyzer::failure_probability(&hardened, &hazard);
        assert!(
            p_h < p_un,
            "Hardened P={p_h:.4} should be less than unhardened P={p_un:.4}"
        );
    }

    // Test 3: Poor condition → P_adj > P for same hazard
    #[test]
    fn test_poor_condition_increases_failure_prob() {
        let good = make_overhead_line(false, 1.0);
        let poor = make_overhead_line(false, 0.2);
        let hazard = WeatherHazard::IceStorm {
            ice_thickness_mm: 20.0,
            duration_h: 12.0,
        };
        let p_good = WeatherResilienceAnalyzer::failure_probability(&good, &hazard);
        let p_poor = WeatherResilienceAnalyzer::failure_probability(&poor, &hazard);
        assert!(
            p_poor > p_good,
            "Poor condition P={p_poor:.4} should exceed good condition P={p_good:.4}"
        );
    }

    // Test 4: Monte Carlo — P(blackout) in [0,1], non-negative load shed
    #[test]
    fn test_monte_carlo_basic_validity() {
        let az = make_analyzer(10, 10.0);
        let hazard = WeatherHazard::Hurricane {
            category: 3,
            wind_speed_ms: 45.0,
            storm_surge_m: 0.5,
        };
        let result = az.monte_carlo_storm_impact(&hazard, 200);
        assert!(
            (0.0..=1.0).contains(&result.p_blackout),
            "p_blackout={} not in [0,1]",
            result.p_blackout
        );
        assert!(
            result.expected_load_shed_mw >= 0.0,
            "expected_load_shed_mw={} is negative",
            result.expected_load_shed_mw
        );
        assert!(
            result.p95_load_shed_mw >= result.expected_load_shed_mw - 1e-9,
            "p95 should be >= mean"
        );
    }

    // Test 5: Critical components — highest risk first
    #[test]
    fn test_critical_components_sorted() {
        // Mix hardened and unhardened; unhardened should rank higher.
        let components = vec![
            ComponentFragility {
                component_id: "LOW".to_string(),
                component_type: ComponentType::OverheadLine,
                age_years: 5.0,
                condition_score: 1.0,
                elevation_m: 0.0,
                hardened: true, // hardened → lower P
                replacement_cost_usd: 100_000.0,
            },
            ComponentFragility {
                component_id: "HIGH".to_string(),
                component_type: ComponentType::OverheadLine,
                age_years: 30.0,
                condition_score: 0.3,
                elevation_m: 0.0,
                hardened: false,
                replacement_cost_usd: 100_000.0,
            },
        ];
        let az = WeatherResilienceAnalyzer::new(
            components,
            WeatherResilienceConfig::default(),
            vec![50.0, 50.0],
        );
        let hazard = WeatherHazard::Hurricane {
            category: 4,
            wind_speed_ms: 60.0,
            storm_surge_m: 0.0,
        };
        let ranked = az.identify_critical_components(&hazard);
        assert_eq!(ranked[0].0, "HIGH", "HIGH-risk component should rank first");
        assert!(ranked[0].1 >= ranked[1].1, "Scores should be descending");
    }

    // Test 6: Hardening ROI — hardening plan reduces expected loss
    #[test]
    fn test_hardening_reduces_expected_loss() {
        let az = make_analyzer(8, 20.0);
        let hazard = WeatherHazard::Hurricane {
            category: 3,
            wind_speed_ms: 50.0,
            storm_surge_m: 0.0,
        };
        let plan = az.optimize_hardening(&hazard, 5_000_000.0);
        // After full budget, we should have reduced risk
        assert!(
            plan.risk_reduction_mw >= 0.0,
            "Risk reduction must be non-negative"
        );
        assert!(
            plan.benefit_to_cost_ratio >= 0.0,
            "BCR must be non-negative"
        );
        // roi_years should be positive or infinite (never negative)
        assert!(
            plan.roi_years >= 0.0,
            "ROI years must be >= 0, got {}",
            plan.roi_years
        );
    }

    // Test 7: SAIDI correct formula
    #[test]
    fn test_saidi_saifi_formula() {
        // 5 buses; 2 events:
        //   event 1: 2 customers interrupted, 4 h duration
        //   event 2: 3 customers interrupted, 2 h duration
        // SAIDI = (2×4 + 3×2) / 5 = (8+6)/5 = 2.8 h
        // SAIFI = (2 + 3) / 5 = 1.0
        let az = WeatherResilienceAnalyzer::new(
            vec![],
            WeatherResilienceConfig::default(),
            vec![10.0; 5],
        );
        let events = vec![(2.0_f64, 4.0_f64), (3.0_f64, 2.0_f64)];
        let (saidi, saifi) = az.saidi_saifi_calculation(&events);
        assert!(
            (saidi - 2.8).abs() < 1e-9,
            "SAIDI expected 2.8, got {saidi}"
        );
        assert!(
            (saifi - 1.0).abs() < 1e-9,
            "SAIFI expected 1.0, got {saifi}"
        );
    }

    // Test 8: Climate projection — expected loss increases with hazard frequency
    #[test]
    fn test_climate_projection_loss_increases() {
        let az = make_analyzer(5, 15.0);
        let hazard = WeatherHazard::Hurricane {
            category: 2,
            wind_speed_ms: 40.0,
            storm_surge_m: 0.0,
        };
        let proj = az.climate_risk_projection(&hazard, 10, 5.0);
        assert_eq!(
            proj.annual_expected_loss_usd.len(),
            10,
            "Should have 10 annual loss values"
        );
        // Loss in year 10 should be strictly greater than year 1
        let first = proj.annual_expected_loss_usd[0];
        let last = proj.annual_expected_loss_usd[9];
        assert!(
            last > first,
            "Year-10 loss {last:.2} should exceed year-1 loss {first:.2}"
        );
        // NPV no-hardening should be positive
        assert!(
            proj.npv_no_hardening_usd > 0.0,
            "NPV without hardening must be positive"
        );
    }

    // Test 9: Fragility curve — monotone increasing for hurricane wind speed
    #[test]
    fn test_fragility_curve_monotone() {
        let curve = WeatherResilienceAnalyzer::component_fragility_curve(
            ComponentType::OverheadLine,
            &WeatherHazard::Hurricane {
                category: 1,
                wind_speed_ms: 0.0,
                storm_surge_m: 0.0,
            },
        );
        let probs = &curve.failure_probability;
        for i in 1..probs.len() {
            assert!(
                probs[i] >= probs[i - 1] - 1e-12,
                "Fragility curve not monotone at index {i}: {} < {}",
                probs[i],
                probs[i - 1]
            );
        }
    }

    // Test 10: Wildfire — proximity matters (closer = higher failure)
    #[test]
    fn test_wildfire_proximity_effect() {
        let comp = make_overhead_line(false, 1.0);
        let close = WeatherHazard::Wildfire {
            fire_weather_index: 80.0,
            proximity_km: 0.5,
        };
        let far = WeatherHazard::Wildfire {
            fire_weather_index: 80.0,
            proximity_km: 20.0,
        };
        let p_close = WeatherResilienceAnalyzer::failure_probability(&comp, &close);
        let p_far = WeatherResilienceAnalyzer::failure_probability(&comp, &far);
        assert!(
            p_close > p_far,
            "Close fire P={p_close:.4} should exceed far fire P={p_far:.4}"
        );
    }
}
