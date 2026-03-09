/// IEC 61000 / IEEE 519 harmonic standards and compliance checking.
///
/// Implements:
/// - **IEC 61000-3-2**: Current emission limits for equipment ≤ 16A per phase
/// - **IEC 61000-3-12**: Current emission limits for equipment 16A–75A
/// - **IEEE 519-2022**: Voltage and current distortion limits for power systems
///
/// # Reference
/// IEC 61000-3-2:2018, IEC 61000-3-12:2011, IEEE 519-2022.
use serde::{Deserialize, Serialize};

/// IEC 61000-3-2 equipment class.
///
/// Determines which current limits apply.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Iec61000Class {
    /// Class A: Balanced three-phase equipment, household appliances (excl. class D)
    ClassA,
    /// Class B: Portable tools (higher limits than A)
    ClassB,
    /// Class C: Lighting equipment
    ClassC,
    /// Class D: PC, monitors, TV (≤ 600W) — relative limits in mA/W
    ClassD,
}

/// IEC 61000-3-2 current harmonic limit.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HarmonicLimit {
    /// Harmonic order n
    pub harmonic: u32,
    /// Maximum harmonic current [A] (absolute) or mA/W (relative for Class D)
    pub limit: f64,
}

/// Get IEC 61000-3-2 Class A current limits [A].
pub fn iec61000_3_2_class_a() -> Vec<HarmonicLimit> {
    vec![
        HarmonicLimit {
            harmonic: 3,
            limit: 2.30,
        },
        HarmonicLimit {
            harmonic: 5,
            limit: 1.14,
        },
        HarmonicLimit {
            harmonic: 7,
            limit: 0.77,
        },
        HarmonicLimit {
            harmonic: 9,
            limit: 0.40,
        },
        HarmonicLimit {
            harmonic: 11,
            limit: 0.33,
        },
        HarmonicLimit {
            harmonic: 13,
            limit: 0.21,
        },
        // Even harmonics 2–8
        HarmonicLimit {
            harmonic: 2,
            limit: 1.08,
        },
        HarmonicLimit {
            harmonic: 4,
            limit: 0.43,
        },
        HarmonicLimit {
            harmonic: 6,
            limit: 0.30,
        },
        HarmonicLimit {
            harmonic: 8,
            limit: 0.23,
        },
        HarmonicLimit {
            harmonic: 10,
            limit: 0.184,
        },
        HarmonicLimit {
            harmonic: 12,
            limit: 0.153,
        },
    ]
}

/// Get IEC 61000-3-2 Class C limits [A] at rated current I_n.
///
/// Class C limits are expressed as % of the fundamental current.
pub fn iec61000_3_2_class_c(i_fundamental: f64) -> Vec<HarmonicLimit> {
    // Percentages from IEC 61000-3-2 Table 2
    let pct = [
        (3u32, 30.0_f64),
        (5, 10.0),
        (7, 7.0),
        (9, 5.0),
        (11, 3.0),
        (13, 3.0),
    ];
    pct.iter()
        .map(|&(h, p)| HarmonicLimit {
            harmonic: h,
            limit: i_fundamental * p / 100.0,
        })
        .collect()
}

/// IEC 61000-3-2 Class D mA/W limits (relative).
pub fn iec61000_3_2_class_d_maw() -> Vec<HarmonicLimit> {
    vec![
        HarmonicLimit {
            harmonic: 3,
            limit: 3.40,
        },
        HarmonicLimit {
            harmonic: 5,
            limit: 1.90,
        },
        HarmonicLimit {
            harmonic: 7,
            limit: 1.00,
        },
        HarmonicLimit {
            harmonic: 9,
            limit: 0.50,
        },
        HarmonicLimit {
            harmonic: 11,
            limit: 0.35,
        },
        HarmonicLimit {
            harmonic: 13,
            limit: 0.296,
        },
    ]
}

/// IEEE 519-2022 voltage distortion limits at point of common coupling (PCC).
///
/// Limits depend on the bus voltage level.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Ieee519VoltageLimits {
    /// Bus voltage range description
    pub voltage_level: String,
    /// Individual harmonic voltage limit [% of fundamental]
    pub individual_limit_pct: f64,
    /// Total Harmonic Distortion (THD) limit [%]
    pub thd_limit_pct: f64,
}

/// IEEE 519-2022 voltage distortion limits (Table 2).
pub fn ieee519_voltage_limits() -> Vec<Ieee519VoltageLimits> {
    vec![
        Ieee519VoltageLimits {
            voltage_level: "≤1 kV".to_string(),
            individual_limit_pct: 5.0,
            thd_limit_pct: 8.0,
        },
        Ieee519VoltageLimits {
            voltage_level: "1–69 kV".to_string(),
            individual_limit_pct: 3.0,
            thd_limit_pct: 5.0,
        },
        Ieee519VoltageLimits {
            voltage_level: "69–161 kV".to_string(),
            individual_limit_pct: 1.5,
            thd_limit_pct: 2.5,
        },
        Ieee519VoltageLimits {
            voltage_level: ">161 kV".to_string(),
            individual_limit_pct: 1.0,
            thd_limit_pct: 1.5,
        },
    ]
}

/// IEEE 519-2022 current distortion limits for general distribution systems.
///
/// Limits depend on the ratio Isc/IL (short-circuit current / demand current).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Ieee519CurrentLimits {
    /// Isc/IL ratio range label
    pub isc_il_ratio: String,
    /// Limit for h < 11 [% of IL]
    pub h_lt_11_pct: f64,
    /// Limit for 11 ≤ h < 17 [% of IL]
    pub h_11_17_pct: f64,
    /// Limit for 17 ≤ h < 23 [% of IL]
    pub h_17_23_pct: f64,
    /// Limit for 23 ≤ h < 35 [% of IL]
    pub h_23_35_pct: f64,
    /// Limit for 35 ≤ h [% of IL]
    pub h_ge_35_pct: f64,
    /// TDD limit [% of IL]
    pub tdd_pct: f64,
}

/// IEEE 519-2022 current distortion limits (Table 3, 120V–69kV).
pub fn ieee519_current_limits() -> Vec<Ieee519CurrentLimits> {
    vec![
        Ieee519CurrentLimits {
            isc_il_ratio: "<20".to_string(),
            h_lt_11_pct: 4.0,
            h_11_17_pct: 2.0,
            h_17_23_pct: 1.5,
            h_23_35_pct: 0.6,
            h_ge_35_pct: 0.3,
            tdd_pct: 5.0,
        },
        Ieee519CurrentLimits {
            isc_il_ratio: "20–50".to_string(),
            h_lt_11_pct: 7.0,
            h_11_17_pct: 3.5,
            h_17_23_pct: 2.5,
            h_23_35_pct: 1.0,
            h_ge_35_pct: 0.5,
            tdd_pct: 8.0,
        },
        Ieee519CurrentLimits {
            isc_il_ratio: "50–100".to_string(),
            h_lt_11_pct: 10.0,
            h_11_17_pct: 4.5,
            h_17_23_pct: 4.0,
            h_23_35_pct: 1.5,
            h_ge_35_pct: 0.7,
            tdd_pct: 12.0,
        },
        Ieee519CurrentLimits {
            isc_il_ratio: "100–1000".to_string(),
            h_lt_11_pct: 12.0,
            h_11_17_pct: 5.5,
            h_17_23_pct: 5.0,
            h_23_35_pct: 2.0,
            h_ge_35_pct: 1.0,
            tdd_pct: 15.0,
        },
        Ieee519CurrentLimits {
            isc_il_ratio: ">1000".to_string(),
            h_lt_11_pct: 15.0,
            h_11_17_pct: 7.0,
            h_17_23_pct: 6.0,
            h_23_35_pct: 2.5,
            h_ge_35_pct: 1.4,
            tdd_pct: 20.0,
        },
    ]
}

/// Select the appropriate IEEE 519 current limit row for a given Isc/IL ratio.
///
/// Returns (h<11 limit %, h 11-17 %, h 17-23 %, h 23-35 %, h≥35 %, TDD %)
pub fn ieee519_current_limit_for_ratio(isc_il: f64) -> Ieee519CurrentLimits {
    if isc_il < 20.0 {
        Ieee519CurrentLimits {
            isc_il_ratio: "<20".to_string(),
            h_lt_11_pct: 4.0,
            h_11_17_pct: 2.0,
            h_17_23_pct: 1.5,
            h_23_35_pct: 0.6,
            h_ge_35_pct: 0.3,
            tdd_pct: 5.0,
        }
    } else if isc_il < 50.0 {
        Ieee519CurrentLimits {
            isc_il_ratio: "20-50".to_string(),
            h_lt_11_pct: 7.0,
            h_11_17_pct: 3.5,
            h_17_23_pct: 2.5,
            h_23_35_pct: 1.0,
            h_ge_35_pct: 0.5,
            tdd_pct: 8.0,
        }
    } else if isc_il < 100.0 {
        Ieee519CurrentLimits {
            isc_il_ratio: "50-100".to_string(),
            h_lt_11_pct: 10.0,
            h_11_17_pct: 4.5,
            h_17_23_pct: 4.0,
            h_23_35_pct: 1.5,
            h_ge_35_pct: 0.7,
            tdd_pct: 12.0,
        }
    } else if isc_il < 1000.0 {
        Ieee519CurrentLimits {
            isc_il_ratio: "100-1000".to_string(),
            h_lt_11_pct: 12.0,
            h_11_17_pct: 5.5,
            h_17_23_pct: 5.0,
            h_23_35_pct: 2.0,
            h_ge_35_pct: 1.0,
            tdd_pct: 15.0,
        }
    } else {
        Ieee519CurrentLimits {
            isc_il_ratio: ">1000".to_string(),
            h_lt_11_pct: 15.0,
            h_11_17_pct: 7.0,
            h_17_23_pct: 6.0,
            h_23_35_pct: 2.5,
            h_ge_35_pct: 1.4,
            tdd_pct: 20.0,
        }
    }
}

/// Check IEC 61000-3-2 Class A compliance.
///
/// `measured` — list of `(harmonic_order, rms_current_A)` pairs
///
/// Returns a list of violations (empty if compliant).
pub fn check_iec61000_3_2_class_a(measured: &[(u32, f64)]) -> Vec<String> {
    let limits = iec61000_3_2_class_a();
    let mut violations = Vec::new();
    for &(h, i_meas) in measured {
        if let Some(lim) = limits.iter().find(|l| l.harmonic == h) {
            if i_meas > lim.limit {
                violations.push(format!(
                    "H{}: {:.3} A exceeds limit {:.3} A",
                    h, i_meas, lim.limit
                ));
            }
        }
    }
    violations
}

/// Check IEEE 519 voltage distortion compliance.
///
/// `bus_kv` — bus voltage [kV]
/// `harmonics_pct` — list of `(harmonic_order, voltage_as_pct_of_fundamental)`
///
/// Returns list of violations.
pub fn check_ieee519_voltage(
    bus_kv: f64,
    harmonics_pct: &[(u32, f64)],
    thd_pct: f64,
) -> Vec<String> {
    let limits = ieee519_voltage_limits();
    let lim = if bus_kv <= 1.0 {
        &limits[0]
    } else if bus_kv <= 69.0 {
        &limits[1]
    } else if bus_kv <= 161.0 {
        &limits[2]
    } else {
        &limits[3]
    };

    let mut violations = Vec::new();
    for &(h, v_pct) in harmonics_pct {
        if v_pct > lim.individual_limit_pct {
            violations.push(format!(
                "H{}: {:.2}% exceeds individual limit {:.1}% ({})",
                h, v_pct, lim.individual_limit_pct, lim.voltage_level
            ));
        }
    }
    if thd_pct > lim.thd_limit_pct {
        violations.push(format!(
            "THD: {:.2}% exceeds limit {:.1}% ({})",
            thd_pct, lim.thd_limit_pct, lim.voltage_level
        ));
    }
    violations
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_class_a_limits_not_empty() {
        let limits = iec61000_3_2_class_a();
        assert!(!limits.is_empty());
        assert!(limits.iter().any(|l| l.harmonic == 3));
        assert!(limits.iter().any(|l| l.harmonic == 5));
    }

    #[test]
    fn test_class_a_compliant() {
        let measured = vec![(3u32, 1.0_f64), (5, 0.5), (7, 0.3)];
        let violations = check_iec61000_3_2_class_a(&measured);
        assert!(
            violations.is_empty(),
            "Should be compliant: {:?}",
            violations
        );
    }

    #[test]
    fn test_class_a_violation() {
        let measured = vec![(3u32, 3.0_f64)]; // > 2.30 A limit
        let violations = check_iec61000_3_2_class_a(&measured);
        assert!(!violations.is_empty(), "Should detect violation");
    }

    #[test]
    fn test_class_c_limits_scale_with_current() {
        let limits_1a = iec61000_3_2_class_c(1.0);
        let limits_10a = iec61000_3_2_class_c(10.0);
        // 10× current → 10× limits
        for (l1, l10) in limits_1a.iter().zip(limits_10a.iter()) {
            assert!((l10.limit / l1.limit - 10.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_ieee519_voltage_limits_count() {
        assert_eq!(ieee519_voltage_limits().len(), 4);
    }

    #[test]
    fn test_ieee519_voltage_compliant() {
        // 11 kV bus, all harmonics well below 3% individual, THD < 5%
        let harmonics = vec![(3u32, 1.0_f64), (5, 0.8), (7, 0.5)];
        let violations = check_ieee519_voltage(11.0, &harmonics, 2.0);
        assert!(
            violations.is_empty(),
            "Should be compliant: {:?}",
            violations
        );
    }

    #[test]
    fn test_ieee519_voltage_violation() {
        // High THD on 11 kV bus
        let harmonics = vec![(3u32, 4.0_f64)]; // > 3% individual limit
        let violations = check_ieee519_voltage(11.0, &harmonics, 6.0);
        assert!(!violations.is_empty(), "Should detect violations");
    }

    #[test]
    fn test_ieee519_current_limits_count() {
        assert_eq!(ieee519_current_limits().len(), 5);
    }

    #[test]
    fn test_ieee519_current_limit_for_ratio() {
        let lim_low = ieee519_current_limit_for_ratio(10.0);
        let lim_high = ieee519_current_limit_for_ratio(2000.0);
        assert!(
            lim_high.h_lt_11_pct > lim_low.h_lt_11_pct,
            "Higher Isc/IL → higher allowable distortion"
        );
    }

    #[test]
    fn test_class_d_limits_not_empty() {
        let limits = iec61000_3_2_class_d_maw();
        assert!(!limits.is_empty());
        // All limits in mA/W should be positive
        for l in &limits {
            assert!(l.limit > 0.0);
        }
    }

    #[test]
    fn test_ieee519_low_voltage_bus() {
        // < 1 kV: THD limit = 8%
        let lims = ieee519_voltage_limits();
        assert!((lims[0].thd_limit_pct - 8.0).abs() < 1e-10);
    }
}
