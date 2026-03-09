#![cfg(feature = "harmonics")]
use oxigrid::harmonics::analysis::{analyse, synthetic_waveform};
use proptest::prelude::*;

const FS: f64 = 6000.0; // sample rate [Hz]
const F0: f64 = 60.0; // fundamental [Hz]
const N: usize = 6000; // one-second window

proptest! {
    /// THD of a pure sine wave is near zero.
    #[test]
    fn prop_pure_sine_thd_near_zero(amp in 0.1_f64..10.0_f64) {
        let samples = synthetic_waveform(F0, FS, N, &[(1, amp, 0.0)]);
        let spec = analyse(&samples, FS, F0, 10, None);
        prop_assert!(spec.thd_pct < 1.5,
            "THD={:.3}% for pure sine amp={}", spec.thd_pct, amp);
    }

    /// Adding a harmonic component strictly increases THD.
    #[test]
    fn prop_added_harmonic_increases_thd(
        h_amp in 0.05_f64..0.5_f64,
        h_order in 2_u32..=9_u32,
    ) {
        let pure = synthetic_waveform(F0, FS, N, &[(1, 1.0, 0.0)]);
        let distorted = synthetic_waveform(F0, FS, N, &[(1, 1.0, 0.0), (h_order, h_amp, 0.0)]);
        let spec_pure = analyse(&pure, FS, F0, 10, None);
        let spec_dist = analyse(&distorted, FS, F0, 10, None);
        prop_assert!(spec_dist.thd_pct > spec_pure.thd_pct,
            "THD with harmonic {:.3}% ≤ pure {:.3}%",
            spec_dist.thd_pct, spec_pure.thd_pct);
    }

    /// THD is always non-negative.
    #[test]
    fn prop_thd_non_negative(
        amp1 in 0.5_f64..2.0_f64,
        amp3 in 0.0_f64..0.5_f64,
        amp5 in 0.0_f64..0.3_f64,
    ) {
        let samples = synthetic_waveform(F0, FS, N,
            &[(1, amp1, 0.0), (3, amp3, 0.0), (5, amp5, 0.0)]);
        let spec = analyse(&samples, FS, F0, 10, None);
        prop_assert!(spec.thd_pct >= 0.0, "THD={}", spec.thd_pct);
    }

    /// IHD% for each harmonic equals magnitude/fundamental × 100.
    #[test]
    fn prop_ihd_matches_ratio(
        fund_amp in 1.0_f64..3.0_f64,
        h3_amp in 0.05_f64..0.5_f64,
    ) {
        let samples = synthetic_waveform(F0, FS, N,
            &[(1, fund_amp, 0.0), (3, h3_amp, 0.0)]);
        let spec = analyse(&samples, FS, F0, 5, None);
        let h3 = spec.harmonics.iter().find(|h| h.order == 3).unwrap();
        let expected_ihd = h3.magnitude / spec.fundamental * 100.0;
        prop_assert!((h3.ihd_pct - expected_ihd).abs() < 1e-6,
            "IHD={:.4}% expected={:.4}%", h3.ihd_pct, expected_ihd);
    }

    /// THD scales consistently: doubling all amplitudes keeps THD unchanged.
    #[test]
    fn prop_thd_amplitude_invariant(
        amp_scale in 0.5_f64..5.0_f64,
        h3_ratio in 0.05_f64..0.3_f64,
    ) {
        let s1 = synthetic_waveform(F0, FS, N,
            &[(1, 1.0, 0.0), (3, h3_ratio, 0.0)]);
        let s2 = synthetic_waveform(F0, FS, N,
            &[(1, amp_scale, 0.0), (3, amp_scale * h3_ratio, 0.0)]);
        let spec1 = analyse(&s1, FS, F0, 5, None);
        let spec2 = analyse(&s2, FS, F0, 5, None);
        prop_assert!((spec1.thd_pct - spec2.thd_pct).abs() < 0.5,
            "THD1={:.3}% THD2={:.3}%", spec1.thd_pct, spec2.thd_pct);
    }

    /// IEEE 519 compliance: low harmonic content always complies at medium voltage.
    #[test]
    fn prop_low_thd_ieee519_compliant(amp in 0.5_f64..3.0_f64) {
        // 1% harmonic → THD ≈ 1% << 5% limit for 1–69 kV
        let samples = synthetic_waveform(F0, FS, N, &[(1, amp, 0.0), (3, amp * 0.01, 0.0)]);
        let spec = analyse(&samples, FS, F0, 10, None);
        prop_assert!(spec.ieee519_voltage_compliant(13.8),
            "THD={:.3}% should be ≤ 5%", spec.thd_pct);
    }

    /// Harmonic spectrum has exactly (max_order - 1) components.
    #[test]
    fn prop_spectrum_component_count(max_order in 2_u32..=15_u32) {
        let samples = synthetic_waveform(F0, FS, N, &[(1, 1.0, 0.0)]);
        let spec = analyse(&samples, FS, F0, max_order, None);
        prop_assert_eq!(spec.harmonics.len(), (max_order - 1) as usize);
    }
}
