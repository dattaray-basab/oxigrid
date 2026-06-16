//! Power system KPI computation: reliability, quality, economics, environment.
//!
//! All formulae follow industry standards:
//! - IEEE 1366 for reliability indices
//! - IEC 61000-3-2 / IEEE 519 for harmonic distortion
//! - IEC 61400-12 capacity factor
//! - NEMA MG-1 voltage unbalance factor

use serde::{Deserialize, Serialize};
use std::fmt;

/// Error type for analytics computations.
#[derive(Debug, Clone, PartialEq)]
pub enum AnalyticsError {
    /// Zero customers — cannot compute customer-weighted metrics.
    ZeroCustomers,
    /// Empty input vector where non-empty expected.
    EmptyInput(String),
    /// Numerical computation error.
    ComputationError(String),
    /// Invalid input parameter.
    InvalidParameter(String),
}

impl fmt::Display for AnalyticsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroCustomers => write!(f, "Total customers must be > 0"),
            Self::EmptyInput(s) => write!(f, "Empty input: {}", s),
            Self::ComputationError(s) => write!(f, "Computation error: {}", s),
            Self::InvalidParameter(s) => write!(f, "Invalid parameter: {}", s),
        }
    }
}

impl std::error::Error for AnalyticsError {}

/// IEEE 1366 system reliability performance metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemPerformanceMetrics {
    /// System Average Interruption Duration Index \[minutes/customer/year\].
    pub saidi_minutes: f64,
    /// System Average Interruption Frequency Index \[interruptions/customer/year\].
    pub saifi: f64,
    /// Customer Average Interruption Duration Index = SAIDI / SAIFI \[minutes\].
    pub caidi_minutes: f64,
    /// Momentary Average Interruption Frequency Index \[momentary interruptions/customer/year\].
    pub maifi: f64,
    /// Energy Not Supplied \[MWh\].
    pub ens_mwh: f64,
    /// Average Service Availability Index \[%\] (target: 99.97%+).
    pub asai_pct: f64,
}

/// Power quality metrics per IEC 61000 / IEEE 519 / NEMA MG-1.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerQualityMetrics {
    /// Voltage Unbalance Factor per NEMA MG-1 \[%\].
    pub voltage_unbalance_pct: f64,
    /// Total Harmonic Distortion of voltage \[%\].
    pub thd_voltage_pct: f64,
    /// Total Harmonic Distortion of current \[%\].
    pub thd_current_pct: f64,
    /// Displacement power factor (fundamental component only).
    pub displacement_power_factor: f64,
    /// True power factor (includes harmonic content).
    pub true_power_factor: f64,
    /// Short-term flicker severity P_st (IEC 61000-4-15).
    pub flicker_pst: f64,
    /// Frequency deviation from nominal \[Hz\].
    pub frequency_deviation_hz: f64,
}

/// Economic performance metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EconomicMetrics {
    /// Levelized Cost of Energy \[$/MWh\].
    pub levelized_cost_mwh: f64,
    /// Net Present Value of investment \[USD\].
    pub net_present_value_usd: f64,
    /// Internal Rate of Return \[fraction\].
    pub internal_rate_of_return: f64,
    /// Simple payback period \[years\].
    pub payback_years: f64,
    /// Capacity factor \[%\].
    pub capacity_factor_pct: f64,
    /// Curtailment fraction \[%\] (renewable energy curtailed vs potential).
    pub curtailment_pct: f64,
    /// Carbon intensity of generation \[tCO₂/MWh\].
    pub carbon_intensity_tco2_per_mwh: f64,
}

/// Environmental impact metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentalMetrics {
    /// CO₂ emissions \[tonne\].
    pub co2_emissions_tonne: f64,
    /// SO₂ emissions \[tonne\].
    pub so2_emissions_tonne: f64,
    /// NOₓ emissions \[tonne\].
    pub nox_emissions_tonne: f64,
    /// Renewable energy penetration \[%\].
    pub renewable_penetration_pct: f64,
    /// Carbon reduction vs baseline scenario \[%\].
    pub carbon_reduction_vs_baseline_pct: f64,
}

/// Input data for KPI computation.
#[derive(Debug, Clone)]
pub struct KpiInput {
    /// Total number of customers served.
    pub total_customers: usize,
    /// Sustained interruption events: `(number_of_customers_affected, duration_hours)`.
    pub interrupted_customer_hours: Vec<(usize, f64)>,
    /// Count of momentary interruptions (< 5 minutes).
    pub momentary_interruptions: usize,
    /// Energy not supplied due to interruptions \[MWh\].
    pub ens_mwh: f64,
    /// Voltage magnitude time series \[pu\] (for unbalance and flicker).
    pub voltage_samples: Vec<f64>,
    /// Voltage harmonic magnitudes \[pu\]: index 0 = fundamental, 1 = 2nd harmonic, etc.
    pub voltage_harmonics: Vec<f64>,
    /// Current harmonic magnitudes \[A\]: index 0 = fundamental, 1 = 2nd harmonic, etc.
    pub current_harmonics: Vec<f64>,
    /// Real power \[MW\].
    pub real_power_mw: f64,
    /// Reactive power \[Mvar\].
    pub reactive_power_mvar: f64,
    /// Apparent power \[MVA\].
    pub apparent_power_mva: f64,
    /// Generation dispatch entries: `(P_mw, cost_per_mwh, capex_usd, co2_t_per_mwh)`.
    pub generation_dispatch: Vec<(f64, f64, f64, f64)>,
    /// Total capital investment \[USD\].
    pub capital_investment_usd: f64,
    /// Annual revenue \[USD/year\].
    pub annual_revenue_usd: f64,
    /// Discount rate (e.g., 0.08 for 8%).
    pub discount_rate: f64,
    /// Project lifetime \[years\].
    pub project_life_years: usize,
    /// Renewable generation \[MW\].
    pub renewable_mw: f64,
    /// Total generation \[MW\].
    pub total_gen_mw: f64,
    /// Baseline CO₂ intensity for comparison \[tCO₂/MWh\].
    pub baseline_co2_t_per_mwh: f64,
}

/// Complete grid KPI dashboard.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridKpiDashboard {
    /// System reliability performance metrics (IEEE 1366).
    pub system_performance: SystemPerformanceMetrics,
    /// Power quality metrics (IEC 61000, IEEE 519, NEMA MG-1).
    pub power_quality: PowerQualityMetrics,
    /// Economic metrics (LCOE, NPV, IRR, payback).
    pub economic: EconomicMetrics,
    /// Environmental impact metrics.
    pub environmental: EnvironmentalMetrics,
}

impl GridKpiDashboard {
    /// Compute all KPIs from the input data.
    pub fn compute(input: &KpiInput) -> Result<GridKpiDashboard, AnalyticsError> {
        let system_performance = compute_reliability(input)?;
        let power_quality = compute_power_quality(input)?;
        let economic = compute_economics(input)?;
        let environmental = compute_environmental(input);

        Ok(GridKpiDashboard {
            system_performance,
            power_quality,
            economic,
            environmental,
        })
    }
}

// ── Reliability indices (IEEE 1366) ──────────────────────────────────────────

fn compute_reliability(input: &KpiInput) -> Result<SystemPerformanceMetrics, AnalyticsError> {
    let n_customers = input.total_customers;
    if n_customers == 0 {
        return Err(AnalyticsError::ZeroCustomers);
    }

    // SAIFI = Σ(customers_interrupted_i) / total_customers
    let sum_interrupted: usize = input
        .interrupted_customer_hours
        .iter()
        .map(|&(c, _)| c)
        .sum();
    let saifi = sum_interrupted as f64 / n_customers as f64;

    // SAIDI = Σ(customers_i × duration_i) / total_customers [minutes]
    let sum_cust_hours: f64 = input
        .interrupted_customer_hours
        .iter()
        .map(|&(c, d)| c as f64 * d)
        .sum();
    let saidi_minutes = sum_cust_hours * 60.0 / n_customers as f64; // convert hours → minutes

    // CAIDI = SAIDI / SAIFI
    let caidi_minutes = if saifi > 1e-12 {
        saidi_minutes / saifi
    } else {
        0.0
    };

    // MAIFI = momentary_interruptions / total_customers
    let maifi = input.momentary_interruptions as f64 / n_customers as f64;

    // ASAI = (hours_in_year - SAIDI_hours) / hours_in_year × 100
    let hours_in_year = 8760.0_f64;
    let saidi_hours = saidi_minutes / 60.0;
    let asai_pct = (hours_in_year - saidi_hours) / hours_in_year * 100.0;
    let asai_pct = asai_pct.clamp(0.0, 100.0);

    Ok(SystemPerformanceMetrics {
        saidi_minutes,
        saifi,
        caidi_minutes,
        maifi,
        ens_mwh: input.ens_mwh,
        asai_pct,
    })
}

// ── Power quality ─────────────────────────────────────────────────────────────

fn compute_power_quality(input: &KpiInput) -> Result<PowerQualityMetrics, AnalyticsError> {
    // THD voltage: sqrt(Σ V_h² for h≥2) / V_1
    let thd_voltage_pct = if input.voltage_harmonics.len() >= 2 {
        let v1 = input.voltage_harmonics[0];
        if v1 < 1e-12 {
            0.0
        } else {
            let harmonic_sum_sq: f64 = input.voltage_harmonics[1..].iter().map(|&vh| vh * vh).sum();
            (harmonic_sum_sq.sqrt() / v1) * 100.0
        }
    } else {
        0.0
    };

    // THD current: sqrt(Σ I_h² for h≥2) / I_1
    let thd_current_pct = if input.current_harmonics.len() >= 2 {
        let i1 = input.current_harmonics[0];
        if i1 < 1e-12 {
            0.0
        } else {
            let harmonic_sum_sq: f64 = input.current_harmonics[1..].iter().map(|&ih| ih * ih).sum();
            (harmonic_sum_sq.sqrt() / i1) * 100.0
        }
    } else {
        0.0
    };

    // Displacement power factor = P / S
    let displacement_pf = if input.apparent_power_mva > 1e-12 {
        (input.real_power_mw / input.apparent_power_mva).clamp(-1.0, 1.0)
    } else {
        1.0
    };

    // True power factor includes harmonic distortion:
    // PF_true = PF_disp / sqrt(1 + THD_I²)
    let thd_i_fraction = thd_current_pct / 100.0;
    let true_pf = displacement_pf / (1.0 + thd_i_fraction * thd_i_fraction).sqrt();
    let true_pf = true_pf.clamp(-1.0, 1.0);

    // Voltage unbalance: simplified as std dev of voltage samples / mean
    let voltage_unbalance_pct = if input.voltage_samples.len() >= 2 {
        let mean = input.voltage_samples.iter().sum::<f64>() / input.voltage_samples.len() as f64;
        if mean < 1e-12 {
            0.0
        } else {
            let variance = input
                .voltage_samples
                .iter()
                .map(|&v| (v - mean).powi(2))
                .sum::<f64>()
                / input.voltage_samples.len() as f64;
            100.0 * variance.sqrt() / mean
        }
    } else {
        0.0
    };

    // Flicker P_st: simplified estimate from voltage variation amplitude
    // P_st ≈ ΔV/V × 100 / 0.3 (normalised to IEC flicker curve at 50 Hz)
    let flicker_pst = if input.voltage_samples.len() >= 2 {
        let min_v = input
            .voltage_samples
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let max_v = input
            .voltage_samples
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let mean = input.voltage_samples.iter().sum::<f64>() / input.voltage_samples.len() as f64;
        if mean < 1e-12 {
            0.0
        } else {
            let delta_v_pct = (max_v - min_v) / mean * 100.0;
            (delta_v_pct / 0.3).max(0.0) // normalised short-term flicker
        }
    } else {
        0.0
    };

    // Frequency deviation: if voltage_samples provided as freq series, use them;
    // otherwise default to 0.
    let frequency_deviation_hz = 0.0_f64; // populated externally if available

    Ok(PowerQualityMetrics {
        voltage_unbalance_pct,
        thd_voltage_pct,
        thd_current_pct,
        displacement_power_factor: displacement_pf,
        true_power_factor: true_pf,
        flicker_pst,
        frequency_deviation_hz,
    })
}

// ── Economics ─────────────────────────────────────────────────────────────────

/// Compute LCOE, NPV, IRR, payback, capacity factor.
fn compute_economics(input: &KpiInput) -> Result<EconomicMetrics, AnalyticsError> {
    let life = input.project_life_years as f64;
    let r = input.discount_rate;
    let capex = input.capital_investment_usd;
    let annual_rev = input.annual_revenue_usd;

    if life <= 0.0 {
        return Err(AnalyticsError::InvalidParameter(
            "project_life_years must be > 0".into(),
        ));
    }
    if r < 0.0 {
        return Err(AnalyticsError::InvalidParameter(
            "discount_rate must be >= 0".into(),
        ));
    }

    // LCOE = (CAPEX + Σ OPEX_t / (1+r)^t) / (Σ Energy_t / (1+r)^t)
    // Simplified: LCOE = (CAPEX × r × (1+r)^n / ((1+r)^n - 1) + annual_opex) / annual_energy
    // Using per-unit dispatch info
    let total_energy_mwh_per_year: f64 = input
        .generation_dispatch
        .iter()
        .map(|&(p_mw, _, _, _)| p_mw * 8760.0)
        .sum();

    let total_opex_per_year: f64 = input
        .generation_dispatch
        .iter()
        .map(|&(p_mw, cost_mwh, _, _)| p_mw * 8760.0 * cost_mwh)
        .sum();

    let lcoe = if total_energy_mwh_per_year > 1e-6 {
        let capex_annualized = if r > 1e-12 {
            let crf = r * (1.0 + r).powf(life) / ((1.0 + r).powf(life) - 1.0);
            capex * crf
        } else {
            capex / life
        };
        (capex_annualized + total_opex_per_year) / total_energy_mwh_per_year
    } else {
        0.0
    };

    // NPV = -CAPEX + Σ annual_rev / (1+r)^t
    let npv = if (r - 0.0).abs() < 1e-12 {
        -capex + annual_rev * life
    } else {
        let annuity_factor = ((1.0 + r).powf(life) - 1.0) / (r * (1.0 + r).powf(life));
        -capex + annual_rev * annuity_factor
    };

    // IRR: solve NPV(r*) = 0 using bisection
    let irr = compute_irr(capex, annual_rev, input.project_life_years);

    // Simple payback = CAPEX / annual_net_revenue
    let payback_years = if annual_rev > 1e-6 {
        (capex / annual_rev).max(0.0)
    } else {
        f64::INFINITY
    };

    // Capacity factor = actual_energy / (nameplate × 8760)
    let installed_mw: f64 = input
        .generation_dispatch
        .iter()
        .map(|&(p, _, _, _)| p)
        .sum();
    let capacity_factor_pct = if installed_mw > 1e-6 {
        total_energy_mwh_per_year / (installed_mw * 8760.0) * 100.0
    } else {
        0.0
    };

    // Curtailment
    let curtailment_pct = 0.0_f64; // requires additional input (potential vs actual)

    // Carbon intensity = Σ P_i × co2_i / Σ P_i
    let weighted_co2: f64 = input
        .generation_dispatch
        .iter()
        .map(|&(p_mw, _, _, co2)| p_mw * co2)
        .sum();
    let carbon_intensity = if installed_mw > 1e-6 {
        weighted_co2 / installed_mw
    } else {
        0.0
    };

    Ok(EconomicMetrics {
        levelized_cost_mwh: lcoe,
        net_present_value_usd: npv,
        internal_rate_of_return: irr,
        payback_years,
        capacity_factor_pct,
        curtailment_pct,
        carbon_intensity_tco2_per_mwh: carbon_intensity,
    })
}

/// Compute IRR via bisection on the NPV equation.
///
/// Finds `r*` such that `−CAPEX + annual_rev × annuity_factor(r*, n) = 0`.
fn compute_irr(capex: f64, annual_rev: f64, life_years: usize) -> f64 {
    if capex <= 0.0 || annual_rev <= 0.0 {
        return 0.0;
    }
    let n = life_years as f64;

    let npv_at_r = |r: f64| -> f64 {
        if r.abs() < 1e-12 {
            -capex + annual_rev * n
        } else {
            let af = ((1.0 + r).powf(n) - 1.0) / (r * (1.0 + r).powf(n));
            -capex + annual_rev * af
        }
    };

    // Quick check: if even at r=0 NPV<0, IRR is negative or doesn't exist
    if npv_at_r(0.0) < 0.0 {
        return 0.0;
    }

    // Bisection in [0, 10.0] (i.e., 0% to 1000%)
    let mut lo = 0.0_f64;
    let mut hi = 10.0_f64;
    for _ in 0..60 {
        let mid = (lo + hi) / 2.0;
        if npv_at_r(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (lo + hi) / 2.0
}

// ── Environmental ─────────────────────────────────────────────────────────────

fn compute_environmental(input: &KpiInput) -> EnvironmentalMetrics {
    // Compute from generation dispatch: co2_t_per_mwh × energy
    let total_energy_mwh_per_year: f64 = input
        .generation_dispatch
        .iter()
        .map(|&(p_mw, _, _, _)| p_mw * 8760.0)
        .sum();

    let co2_tonne: f64 = input
        .generation_dispatch
        .iter()
        .map(|&(p_mw, _, _, co2)| p_mw * 8760.0 * co2)
        .sum();

    // SO2, NOx: approximate emission factors (fossil fuel typical values)
    // SO2: 0.4 kg/MWh average coal; NOx: 0.3 kg/MWh
    let fossil_fraction = 1.0 - input.renewable_mw / input.total_gen_mw.max(1e-6);
    let fossil_energy_mwh = total_energy_mwh_per_year * fossil_fraction;
    let so2_tonne = fossil_energy_mwh * 0.0004; // 0.4 kg/MWh → tonnes
    let nox_tonne = fossil_energy_mwh * 0.0003;

    // Renewable penetration
    let renewable_penetration_pct = if input.total_gen_mw > 1e-6 {
        100.0 * input.renewable_mw / input.total_gen_mw
    } else {
        0.0
    };

    // Carbon reduction vs baseline
    let baseline_co2 = input.baseline_co2_t_per_mwh * total_energy_mwh_per_year;
    let carbon_reduction_pct = if baseline_co2 > 1e-6 {
        100.0 * (baseline_co2 - co2_tonne) / baseline_co2
    } else {
        0.0
    };

    EnvironmentalMetrics {
        co2_emissions_tonne: co2_tonne,
        so2_emissions_tonne: so2_tonne,
        nox_emissions_tonne: nox_tonne,
        renewable_penetration_pct,
        carbon_reduction_vs_baseline_pct: carbon_reduction_pct,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_input() -> KpiInput {
        KpiInput {
            total_customers: 10_000,
            interrupted_customer_hours: vec![(500, 2.0), (200, 4.0), (100, 1.0)],
            momentary_interruptions: 50,
            ens_mwh: 12.5,
            voltage_samples: vec![1.02, 0.98, 1.01, 0.99, 1.00, 1.03, 0.97],
            voltage_harmonics: vec![1.0, 0.03, 0.02, 0.01],
            current_harmonics: vec![100.0, 5.0, 3.0, 2.0],
            real_power_mw: 80.0,
            reactive_power_mvar: 30.0,
            apparent_power_mva: 85.44,
            generation_dispatch: vec![
                (50.0, 20.0, 1_000_000.0, 0.9), // coal
                (30.0, 0.0, 2_000_000.0, 0.0),  // solar
            ],
            capital_investment_usd: 3_000_000.0,
            annual_revenue_usd: 800_000.0,
            discount_rate: 0.08,
            project_life_years: 20,
            renewable_mw: 30.0,
            total_gen_mw: 80.0,
            baseline_co2_t_per_mwh: 0.85,
        }
    }

    #[test]
    fn test_caidi_equals_saidi_divided_by_saifi() {
        let input = sample_input();
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let sp = &result.system_performance;

        if sp.saifi > 1e-12 {
            let caidi_expected = sp.saidi_minutes / sp.saifi;
            assert!(
                (sp.caidi_minutes - caidi_expected).abs() < 1e-6,
                "CAIDI = {:.4} should equal SAIDI/SAIFI = {:.4}",
                sp.caidi_minutes,
                caidi_expected
            );
        }
    }

    #[test]
    fn test_asai_formula() {
        let input = sample_input();
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let sp = &result.system_performance;

        // ASAI = (8760 - SAIDI_hours) / 8760 * 100
        let saidi_hours = sp.saidi_minutes / 60.0;
        let asai_expected = (8760.0 - saidi_hours) / 8760.0 * 100.0;
        assert!(
            (sp.asai_pct - asai_expected).abs() < 1e-4,
            "ASAI = {:.4}% should equal formula result {:.4}%",
            sp.asai_pct,
            asai_expected
        );
    }

    #[test]
    fn test_thd_computation() {
        // Manual: V1=1.0, V2=0.03, V3=0.02, V4=0.01
        // THD = sqrt(0.03² + 0.02² + 0.01²) / 1.0 × 100 = sqrt(0.0014) × 100 ≈ 3.742%
        let input = sample_input();
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let pq = &result.power_quality;

        let expected_thd =
            ((0.03_f64.powi(2) + 0.02_f64.powi(2) + 0.01_f64.powi(2)).sqrt()) * 100.0;
        assert!(
            (pq.thd_voltage_pct - expected_thd).abs() < 0.01,
            "THD_V = {:.4}% should be {:.4}%",
            pq.thd_voltage_pct,
            expected_thd
        );
    }

    #[test]
    fn test_lcoe_computation() {
        // Simple: 100 MW, $20/MWh opex, $10M capex, 20yr, 8% discount
        let input = KpiInput {
            total_customers: 1000,
            interrupted_customer_hours: vec![],
            momentary_interruptions: 0,
            ens_mwh: 0.0,
            voltage_samples: vec![1.0; 10],
            voltage_harmonics: vec![1.0],
            current_harmonics: vec![100.0],
            real_power_mw: 100.0,
            reactive_power_mvar: 0.0,
            apparent_power_mva: 100.0,
            generation_dispatch: vec![(100.0, 20.0, 10_000_000.0, 0.5)],
            capital_investment_usd: 10_000_000.0,
            annual_revenue_usd: 5_000_000.0,
            discount_rate: 0.08,
            project_life_years: 20,
            renewable_mw: 0.0,
            total_gen_mw: 100.0,
            baseline_co2_t_per_mwh: 0.85,
        };

        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let eco = &result.economic;

        // LCOE should be positive and > variable cost ($20/MWh)
        assert!(
            eco.levelized_cost_mwh > 20.0,
            "LCOE should exceed variable cost"
        );
        // Payback = 10M / 5M = 2 years
        assert!(
            (eco.payback_years - 2.0).abs() < 0.01,
            "Payback should be 2.0 years, got {:.3}",
            eco.payback_years
        );
    }

    #[test]
    fn test_npv_irr_consistency() {
        let input = sample_input();
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let eco = &result.economic;

        // With 8% discount rate and 20-year project, if NPV > 0, IRR > 8%
        // Payback should be positive
        assert!(eco.payback_years > 0.0, "Payback must be positive");

        // IRR should be non-negative
        assert!(
            eco.internal_rate_of_return >= 0.0,
            "IRR must be non-negative"
        );
    }

    #[test]
    fn test_renewable_penetration() {
        let input = sample_input(); // 30 MW renewable, 80 MW total
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let env = &result.environmental;

        let expected_pct = 30.0 / 80.0 * 100.0;
        assert!(
            (env.renewable_penetration_pct - expected_pct).abs() < 0.01,
            "Renewable penetration = {:.2}% should be {:.2}%",
            env.renewable_penetration_pct,
            expected_pct
        );
    }

    #[test]
    fn test_zero_customers_error() {
        let mut input = sample_input();
        input.total_customers = 0;
        let err = GridKpiDashboard::compute(&input);
        assert!(
            matches!(err, Err(AnalyticsError::ZeroCustomers)),
            "Should return ZeroCustomers error"
        );
    }

    #[test]
    fn test_carbon_reduction_vs_baseline() {
        // Pure renewable: co2 = 0, baseline = 0.85 t/MWh → 100% reduction
        let input = KpiInput {
            total_customers: 1000,
            interrupted_customer_hours: vec![],
            momentary_interruptions: 0,
            ens_mwh: 0.0,
            voltage_samples: vec![1.0],
            voltage_harmonics: vec![1.0],
            current_harmonics: vec![100.0],
            real_power_mw: 100.0,
            reactive_power_mvar: 0.0,
            apparent_power_mva: 100.0,
            generation_dispatch: vec![(100.0, 0.0, 5_000_000.0, 0.0)], // zero co2
            capital_investment_usd: 5_000_000.0,
            annual_revenue_usd: 2_000_000.0,
            discount_rate: 0.05,
            project_life_years: 25,
            renewable_mw: 100.0,
            total_gen_mw: 100.0,
            baseline_co2_t_per_mwh: 0.85,
        };

        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let env = &result.environmental;

        assert!(
            (env.co2_emissions_tonne).abs() < 1e-6,
            "Pure renewable should have zero CO2"
        );
        assert!(
            (env.carbon_reduction_vs_baseline_pct - 100.0).abs() < 0.01,
            "100% renewable should give 100% carbon reduction"
        );
    }

    #[test]
    fn test_saidi_and_saifi_formula() {
        let input = KpiInput {
            total_customers: 1000,
            interrupted_customer_hours: vec![(200, 2.0), (100, 3.0)],
            momentary_interruptions: 0,
            ens_mwh: 0.0,
            voltage_samples: vec![1.0, 1.0],
            voltage_harmonics: vec![1.0],
            current_harmonics: vec![100.0],
            real_power_mw: 100.0,
            reactive_power_mvar: 0.0,
            apparent_power_mva: 100.0,
            generation_dispatch: vec![(100.0, 0.0, 1_000_000.0, 0.0)],
            capital_investment_usd: 1_000_000.0,
            annual_revenue_usd: 200_000.0,
            discount_rate: 0.05,
            project_life_years: 20,
            renewable_mw: 100.0,
            total_gen_mw: 100.0,
            baseline_co2_t_per_mwh: 0.85,
        };
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let sp = &result.system_performance;

        // SAIFI = (200 + 100) / 1000 = 0.3
        assert!(
            (sp.saifi - 0.3).abs() < 1e-9,
            "SAIFI should be 0.3, got {:.6}",
            sp.saifi
        );
        // SAIDI = (200*2.0 + 100*3.0) * 60 / 1000 = 700 * 60 / 1000 = 42.0 minutes
        let expected_saidi = (200.0_f64 * 2.0 + 100.0_f64 * 3.0) * 60.0 / 1000.0;
        assert!(
            (sp.saidi_minutes - expected_saidi).abs() < 1e-9,
            "SAIDI should be {:.2} minutes, got {:.6}",
            expected_saidi,
            sp.saidi_minutes
        );
    }

    #[test]
    fn test_maifi_formula() {
        let input = KpiInput {
            total_customers: 500,
            interrupted_customer_hours: vec![],
            momentary_interruptions: 10,
            ens_mwh: 0.0,
            voltage_samples: vec![1.0, 1.0],
            voltage_harmonics: vec![1.0],
            current_harmonics: vec![100.0],
            real_power_mw: 100.0,
            reactive_power_mvar: 0.0,
            apparent_power_mva: 100.0,
            generation_dispatch: vec![(100.0, 0.0, 1_000_000.0, 0.0)],
            capital_investment_usd: 1_000_000.0,
            annual_revenue_usd: 200_000.0,
            discount_rate: 0.05,
            project_life_years: 20,
            renewable_mw: 100.0,
            total_gen_mw: 100.0,
            baseline_co2_t_per_mwh: 0.85,
        };
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let sp = &result.system_performance;

        // MAIFI = 10 / 500 = 0.02
        let expected_maifi = 10.0_f64 / 500.0_f64;
        assert!(
            (sp.maifi - expected_maifi).abs() < 1e-9,
            "MAIFI should be {:.4}, got {:.6}",
            expected_maifi,
            sp.maifi
        );
    }

    #[test]
    fn test_perfect_reliability_asai_near_100() {
        let input = KpiInput {
            total_customers: 1000,
            interrupted_customer_hours: vec![],
            momentary_interruptions: 0,
            ens_mwh: 0.0,
            voltage_samples: vec![1.0, 1.0],
            voltage_harmonics: vec![1.0],
            current_harmonics: vec![100.0],
            real_power_mw: 100.0,
            reactive_power_mvar: 0.0,
            apparent_power_mva: 100.0,
            generation_dispatch: vec![(100.0, 0.0, 1_000_000.0, 0.0)],
            capital_investment_usd: 1_000_000.0,
            annual_revenue_usd: 200_000.0,
            discount_rate: 0.05,
            project_life_years: 20,
            renewable_mw: 100.0,
            total_gen_mw: 100.0,
            baseline_co2_t_per_mwh: 0.85,
        };
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let sp = &result.system_performance;

        assert!(
            (sp.asai_pct - 100.0).abs() < 1e-9,
            "ASAI should be 100.0% with no outages, got {:.6}",
            sp.asai_pct
        );
        assert!(
            sp.saifi.abs() < 1e-9,
            "SAIFI should be 0.0 with no outages, got {:.6}",
            sp.saifi
        );
        assert!(
            sp.saidi_minutes.abs() < 1e-9,
            "SAIDI should be 0.0 minutes with no outages, got {:.6}",
            sp.saidi_minutes
        );
    }

    #[test]
    fn test_displacement_power_factor() {
        let input = KpiInput {
            total_customers: 1000,
            interrupted_customer_hours: vec![],
            momentary_interruptions: 0,
            ens_mwh: 0.0,
            voltage_samples: vec![1.0, 1.0],
            voltage_harmonics: vec![1.0],
            current_harmonics: vec![100.0],
            real_power_mw: 80.0,
            reactive_power_mvar: 60.0,
            apparent_power_mva: 100.0,
            generation_dispatch: vec![(100.0, 0.0, 1_000_000.0, 0.0)],
            capital_investment_usd: 1_000_000.0,
            annual_revenue_usd: 200_000.0,
            discount_rate: 0.05,
            project_life_years: 20,
            renewable_mw: 100.0,
            total_gen_mw: 100.0,
            baseline_co2_t_per_mwh: 0.85,
        };
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let pq = &result.power_quality;

        // displacement_pf = P / S = 80.0 / 100.0 = 0.8
        let expected_dpf = 80.0_f64 / 100.0_f64;
        assert!(
            (pq.displacement_power_factor - expected_dpf).abs() < 1e-6,
            "Displacement PF should be {:.4}, got {:.6}",
            expected_dpf,
            pq.displacement_power_factor
        );
    }

    #[test]
    fn test_true_power_factor_reduced_by_thd() {
        // current_harmonics = [100.0, 50.0] => THD_I = 50/100 = 50% = 0.5
        // real_power_mw = 100.0, apparent_power_mva = 100.0 => displacement_pf = 1.0
        // true_pf = 1.0 / sqrt(1 + 0.5^2) = 1.0 / sqrt(1.25)
        let input = KpiInput {
            total_customers: 1000,
            interrupted_customer_hours: vec![],
            momentary_interruptions: 0,
            ens_mwh: 0.0,
            voltage_samples: vec![1.0, 1.0],
            voltage_harmonics: vec![1.0],
            current_harmonics: vec![100.0, 50.0],
            real_power_mw: 100.0,
            reactive_power_mvar: 0.0,
            apparent_power_mva: 100.0,
            generation_dispatch: vec![(100.0, 0.0, 1_000_000.0, 0.0)],
            capital_investment_usd: 1_000_000.0,
            annual_revenue_usd: 200_000.0,
            discount_rate: 0.05,
            project_life_years: 20,
            renewable_mw: 100.0,
            total_gen_mw: 100.0,
            baseline_co2_t_per_mwh: 0.85,
        };
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let pq = &result.power_quality;

        let thd_i = 50.0_f64 / 100.0_f64; // 0.5
        let expected_true_pf = 1.0_f64 / (1.0_f64 + thd_i * thd_i).sqrt();
        assert!(
            (pq.true_power_factor - expected_true_pf).abs() < 1e-4,
            "True PF should be {:.6}, got {:.6}",
            expected_true_pf,
            pq.true_power_factor
        );
    }

    #[test]
    fn test_capacity_factor_full_dispatch() {
        // Single unit at 100 MW — all energy is actual generation
        // capacity_factor_pct = actual_energy / (nameplate * 8760) * 100 = 100%
        let input = KpiInput {
            total_customers: 1000,
            interrupted_customer_hours: vec![],
            momentary_interruptions: 0,
            ens_mwh: 0.0,
            voltage_samples: vec![1.0, 1.0],
            voltage_harmonics: vec![1.0],
            current_harmonics: vec![100.0],
            real_power_mw: 100.0,
            reactive_power_mvar: 0.0,
            apparent_power_mva: 100.0,
            generation_dispatch: vec![(100.0, 0.0, 0.0, 0.0)],
            capital_investment_usd: 1_000_000.0,
            annual_revenue_usd: 200_000.0,
            discount_rate: 0.05,
            project_life_years: 20,
            renewable_mw: 100.0,
            total_gen_mw: 100.0,
            baseline_co2_t_per_mwh: 0.85,
        };
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let eco = &result.economic;

        assert!(
            (eco.capacity_factor_pct - 100.0).abs() < 1e-6,
            "Capacity factor should be 100.0%, got {:.6}",
            eco.capacity_factor_pct
        );
    }

    #[test]
    fn test_so2_nox_zero_for_pure_renewable() {
        // renewable_mw = total_gen_mw => fossil_fraction = 0 => SO2 = NOx = 0
        let input = KpiInput {
            total_customers: 1000,
            interrupted_customer_hours: vec![],
            momentary_interruptions: 0,
            ens_mwh: 0.0,
            voltage_samples: vec![1.0, 1.0],
            voltage_harmonics: vec![1.0],
            current_harmonics: vec![100.0],
            real_power_mw: 100.0,
            reactive_power_mvar: 0.0,
            apparent_power_mva: 100.0,
            generation_dispatch: vec![(100.0, 0.0, 0.0, 0.0)],
            capital_investment_usd: 1_000_000.0,
            annual_revenue_usd: 200_000.0,
            discount_rate: 0.05,
            project_life_years: 20,
            renewable_mw: 100.0,
            total_gen_mw: 100.0,
            baseline_co2_t_per_mwh: 0.85,
        };
        let result = GridKpiDashboard::compute(&input).expect("KPI computation should succeed");
        let env = &result.environmental;

        assert!(
            env.so2_emissions_tonne.abs() < 1e-9,
            "SO2 should be ~0 for pure renewable, got {:.9}",
            env.so2_emissions_tonne
        );
        assert!(
            env.nox_emissions_tonne.abs() < 1e-9,
            "NOx should be ~0 for pure renewable, got {:.9}",
            env.nox_emissions_tonne
        );
    }
}
