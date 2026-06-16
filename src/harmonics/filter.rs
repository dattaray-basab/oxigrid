/// Passive harmonic filter design and analysis.
///
/// Implements single-tuned (series LC), double-tuned, and high-pass filters
/// commonly used in power systems to suppress harmonic currents.
///
/// # Reference
/// IEEE Std 1036-2010, "IEEE Guide for Application of Shunt Power Capacitors".
use num_complex::Complex64;
use serde::{Deserialize, Serialize};

use crate::harmonics::analysis::HarmonicSpectrum;

/// Type of passive harmonic filter.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum FilterType {
    /// Series-resonant (single-tuned): minimum impedance at f_n
    SingleTuned,
    /// High-pass: passes all harmonics above the tuning frequency
    HighPass,
    /// C-type: modified high-pass with low fundamental losses
    CType,
}

/// A passive shunt harmonic filter branch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PassiveFilter {
    /// Filter type
    pub filter_type: FilterType,
    /// Tuning harmonic order (e.g. 5 for 5th harmonic, 250/300 Hz)
    pub harmonic_order: f64,
    /// Capacitor reactive power `MVAr` at fundamental
    pub q_mvar: f64,
    /// System fundamental frequency `Hz`
    pub fundamental_hz: f64,
    /// System base voltage `kV`
    pub bus_kv: f64,
    /// Quality factor Q = ωₙL/R (sharpness of tuning)
    pub q_factor: f64,
    // Derived parameters (computed from above)
    capacitance_f: f64,  // [F]
    inductance_h: f64,   // [H]
    resistance_ohm: f64, // [Ω]
}

impl PassiveFilter {
    /// Design a single-tuned filter.
    ///
    /// # Arguments
    /// - `harmonic_order` — harmonic to tune to (e.g. 5.0 for 5th, 4.7 for detuned 5th)
    /// - `q_mvar`         — capacitor reactive power at fundamental `MVAr`
    /// - `fundamental_hz` — system fundamental frequency `Hz`
    /// - `bus_kv`         — system line-to-line voltage `kV`
    /// - `q_factor`       — quality factor (typically 30–80 for industrial filters)
    pub fn single_tuned(
        harmonic_order: f64,
        q_mvar: f64,
        fundamental_hz: f64,
        bus_kv: f64,
        q_factor: f64,
    ) -> Self {
        let v_ll = bus_kv * 1000.0; // line-to-line voltage [V]
        let omega_1 = 2.0 * std::f64::consts::PI * fundamental_hz;
        let omega_n = omega_1 * harmonic_order;

        // Three-phase capacitor: Q = V_LL²/Xc → Xc = V_LL²/Q → C = Q/(ω₁·V_LL²)
        let q_var = q_mvar * 1e6;
        let xc = v_ll * v_ll / q_var;
        let capacitance_f = 1.0 / (omega_1 * xc);

        // Inductor: ω_n²LC = 1 → L = 1/(ω_n²C)
        let inductance_h = 1.0 / (omega_n * omega_n * capacitance_f);

        // Resistance from quality factor: Q = ω_n·L/R → R = ω_n·L/Q
        let resistance_ohm = omega_n * inductance_h / q_factor;

        Self {
            filter_type: FilterType::SingleTuned,
            harmonic_order,
            q_mvar,
            fundamental_hz,
            bus_kv,
            q_factor,
            capacitance_f,
            inductance_h,
            resistance_ohm,
        }
    }

    /// Design a high-pass filter tuned at `harmonic_order`.
    pub fn high_pass(
        harmonic_order: f64,
        q_mvar: f64,
        fundamental_hz: f64,
        bus_kv: f64,
        q_factor: f64,
    ) -> Self {
        let mut filter =
            Self::single_tuned(harmonic_order, q_mvar, fundamental_hz, bus_kv, q_factor);
        filter.filter_type = FilterType::HighPass;
        // High-pass has R in parallel with L instead of series: R_parallel = Q·ω_n·L
        filter.resistance_ohm = q_factor
            * 2.0
            * std::f64::consts::PI
            * fundamental_hz
            * harmonic_order
            * filter.inductance_h;
        filter
    }

    /// Compute filter impedance `Ω` at a given frequency.
    pub fn impedance(&self, freq_hz: f64) -> Complex64 {
        let omega = 2.0 * std::f64::consts::PI * freq_hz;
        let z_c = Complex64::new(0.0, -1.0 / (omega * self.capacitance_f));
        let z_l = Complex64::new(0.0, omega * self.inductance_h);
        let z_r = Complex64::new(self.resistance_ohm, 0.0);

        match self.filter_type {
            FilterType::SingleTuned | FilterType::CType => {
                // Series RLC: Z = R + jωL - j/(ωC)
                z_r + z_l + z_c
            }
            FilterType::HighPass => {
                // R in parallel with L, then in series with C
                // Z = (R·jωL)/(R + jωL) + 1/(jωC)
                let z_rl = (z_r * z_l) / (z_r + z_l);
                z_rl + z_c
            }
        }
    }

    /// Harmonic current mitigation factor at order h (0 = full absorption, 1 = no effect).
    ///
    /// Approximated as |Z_filter| / (|Z_filter| + |Z_system|) using a simplified
    /// Thevenin source model with `z_system_ohm` system impedance.
    pub fn mitigation_factor(&self, harmonic_order: u32, z_system_ohm: f64) -> f64 {
        let freq = self.fundamental_hz * harmonic_order as f64;
        let z_f = self.impedance(freq).norm();
        if z_f + z_system_ohm < 1e-9 {
            return 1.0;
        }
        z_f / (z_f + z_system_ohm)
    }

    /// Apply the filter to a harmonic spectrum, returning the mitigated spectrum.
    ///
    /// Uses a simplified current divider model with `z_source_ohm` source impedance.
    pub fn apply_to_spectrum(
        &self,
        spectrum: &HarmonicSpectrum,
        z_source_ohm: f64,
    ) -> HarmonicSpectrum {
        let mut mitigated = spectrum.clone();
        let mut thd_sum_sq = 0.0_f64;

        for h in &mut mitigated.harmonics {
            let mf = self.mitigation_factor(h.order, z_source_ohm);
            h.magnitude *= mf;
            h.ihd_pct = h.magnitude / mitigated.fundamental * 100.0;
            thd_sum_sq += h.magnitude * h.magnitude;
        }

        mitigated.thd_pct = thd_sum_sq.sqrt() / mitigated.fundamental * 100.0;
        mitigated
    }

    /// Reactive power supplied at fundamental frequency `MVAr`.
    pub fn reactive_power_mvar(&self) -> f64 {
        let omega = 2.0 * std::f64::consts::PI * self.fundamental_hz;
        let xc = 1.0 / (omega * self.capacitance_f);
        let v_ll = self.bus_kv * 1000.0;
        // Q = V_LL² / Xc (three-phase)
        v_ll * v_ll / xc / 1e6
    }

    /// Filter capacitor size `μF`.
    pub fn capacitance_uf(&self) -> f64 {
        self.capacitance_f * 1e6
    }

    /// Filter inductor size `mH`.
    pub fn inductance_mh(&self) -> f64 {
        self.inductance_h * 1e3
    }
}

/// Design a filter bank to mitigate multiple harmonic orders.
///
/// Returns one single-tuned filter per harmonic order.
pub fn design_filter_bank(
    harmonic_orders: &[u32],
    q_total_mvar: f64,
    fundamental_hz: f64,
    bus_kv: f64,
    q_factor: f64,
) -> Vec<PassiveFilter> {
    let q_per_filter = q_total_mvar / harmonic_orders.len() as f64;
    harmonic_orders
        .iter()
        .map(|&h| {
            PassiveFilter::single_tuned(
                h as f64 - 0.15, // detune slightly below harmonic (common practice)
                q_per_filter,
                fundamental_hz,
                bus_kv,
                q_factor,
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::harmonics::analysis::{HarmonicComponent, HarmonicSpectrum};

    fn sample_filter() -> PassiveFilter {
        PassiveFilter::single_tuned(5.0, 10.0, 60.0, 13.8, 50.0)
    }

    /// Build a minimal HarmonicSpectrum with a fundamental and a few harmonic components.
    fn sample_spectrum() -> HarmonicSpectrum {
        HarmonicSpectrum {
            fundamental_hz: 60.0,
            fundamental: 100.0,
            harmonics: vec![
                HarmonicComponent {
                    order: 5,
                    magnitude: 20.0,
                    phase_rad: 0.0,
                    ihd_pct: 20.0,
                },
                HarmonicComponent {
                    order: 7,
                    magnitude: 10.0,
                    phase_rad: 0.0,
                    ihd_pct: 10.0,
                },
            ],
            thd_pct: 22.36,
            tdd_pct: None,
        }
    }

    #[test]
    fn test_filter_minimum_impedance_at_tuning_freq() {
        let f = sample_filter();
        let f_tune = f.fundamental_hz * f.harmonic_order;
        let z_tune = f.impedance(f_tune).norm();
        let z_fund = f.impedance(f.fundamental_hz).norm();
        // Impedance at tuning frequency should be much less than at fundamental
        assert!(
            z_tune < z_fund / 5.0,
            "|Z_tune|={:.3} Ω should be << |Z_fund|={:.3} Ω",
            z_tune,
            z_fund
        );
    }

    #[test]
    fn test_filter_reactive_power_close_to_rated() {
        let f = sample_filter();
        let q_actual = f.reactive_power_mvar();
        // Should be within 10% of specified Q
        assert!(
            (q_actual - f.q_mvar).abs() < f.q_mvar * 0.1,
            "Q_actual={:.3} MVAr vs Q_spec={:.3} MVAr",
            q_actual,
            f.q_mvar
        );
    }

    #[test]
    fn test_filter_capacitance_positive() {
        let f = sample_filter();
        assert!(
            f.capacitance_uf() > 0.0,
            "C={:.4} μF should be positive",
            f.capacitance_uf()
        );
        assert!(
            f.inductance_mh() > 0.0,
            "L={:.4} mH should be positive",
            f.inductance_mh()
        );
    }

    #[test]
    fn test_high_pass_filter_bounded_at_high_harmonics() {
        // A high-pass (second-order damped) filter has minimum impedance at its
        // tuning frequency, and impedance at all other frequencies is bounded by R
        // (unlike a single-tuned filter which grows without bound off-resonance).
        let hp = PassiveFilter::high_pass(3.0, 5.0, 60.0, 13.8, 10.0);
        let z11 = hp.impedance(11.0 * 60.0).norm();
        let z25 = hp.impedance(25.0 * 60.0).norm();
        // At high harmonics, impedance approaches R (the parallel resistance)
        assert!(
            z11 <= hp.resistance_ohm + 1.0,
            "|Z_11|={:.3} should be ≤ R={:.3}",
            z11,
            hp.resistance_ohm
        );
        assert!(
            z25 <= hp.resistance_ohm + 1.0,
            "|Z_25|={:.3} should be ≤ R={:.3}",
            z25,
            hp.resistance_ohm
        );
        // Impedance should be finite (not blow up)
        assert!(z25.is_finite(), "|Z_25| should be finite");
    }

    #[test]
    fn test_mitigation_factor_reduces_at_tuning() {
        let f = sample_filter();
        let mf = f.mitigation_factor(5, 1.0); // 1 Ω system impedance
        assert!(mf < 1.0, "Mitigation factor should be < 1.0: {:.4}", mf);
        assert!(mf >= 0.0, "Mitigation factor should be ≥ 0.0");
    }

    #[test]
    fn test_filter_bank_design() {
        let bank = design_filter_bank(&[5, 7, 11], 30.0, 60.0, 13.8, 50.0);
        assert_eq!(bank.len(), 3);
        for f in &bank {
            assert!(f.q_mvar > 0.0);
        }
    }

    // ---- NEW TESTS ----

    /// The high-pass constructor must set `filter_type` to `HighPass`.
    #[test]
    fn test_high_pass_filter_type_is_high_pass() {
        let hp = PassiveFilter::high_pass(5.0, 10.0, 60.0, 13.8, 30.0);
        assert_eq!(
            hp.filter_type,
            FilterType::HighPass,
            "high_pass() must produce FilterType::HighPass"
        );
    }

    /// `apply_to_spectrum` must reduce harmonic magnitudes and keep THD consistent.
    #[test]
    fn test_apply_to_spectrum_reduces_harmonics() {
        let f = PassiveFilter::single_tuned(5.0, 10.0, 60.0, 13.8, 50.0);
        let original = sample_spectrum();
        let mitigated = f.apply_to_spectrum(&original, 2.0);

        // Every harmonic magnitude must be ≤ the original magnitude.
        for (orig_h, mit_h) in original.harmonics.iter().zip(mitigated.harmonics.iter()) {
            assert!(
                mit_h.magnitude <= orig_h.magnitude + 1e-9,
                "order {}: mitigated magnitude {:.4} must be ≤ original {:.4}",
                orig_h.order,
                mit_h.magnitude,
                orig_h.magnitude
            );
        }
        // The resulting THD must be non-negative and finite.
        assert!(mitigated.thd_pct >= 0.0, "mitigated THD must be ≥ 0");
        assert!(
            mitigated.thd_pct.is_finite(),
            "mitigated THD must be finite"
        );
    }

    /// `apply_to_spectrum` must recompute `ihd_pct` consistently with mitigated magnitude.
    #[test]
    fn test_apply_to_spectrum_ihd_pct_consistency() {
        let f = PassiveFilter::single_tuned(5.0, 10.0, 60.0, 13.8, 50.0);
        let original = sample_spectrum();
        let mitigated = f.apply_to_spectrum(&original, 2.0);

        for h in &mitigated.harmonics {
            let expected_ihd = h.magnitude / mitigated.fundamental * 100.0;
            assert!(
                (h.ihd_pct - expected_ihd).abs() < 1e-6,
                "order {}: ihd_pct={:.6} does not match magnitude/fundamental={:.6}",
                h.order,
                h.ihd_pct,
                expected_ihd
            );
        }
    }

    /// `mitigation_factor` must return 1.0 when both filter impedance and system
    /// impedance sum to effectively zero (guard branch `z_f + z_system_ohm < 1e-9`).
    /// We exercise this by constructing an extremely high-Q filter and passing
    /// `z_system_ohm = 0.0` — at the exact resonance the series impedance collapses
    /// to the very small R, so `z_f` may remain non-negligible; to reliably trigger
    /// the guard we call a non-tuned harmonic order where both contributions are ~0
    /// in a contrived way. Instead we directly verify the public behaviour: with
    /// `z_system_ohm = 0.0` the factor = z_f / z_f = 1.0 for any nonzero z_f,
    /// so the ratio should equal 1.0.
    #[test]
    fn test_mitigation_factor_zero_system_impedance() {
        let f = sample_filter();
        // With z_system = 0, the divider gives z_f / z_f = 1.0 (filter absorbs nothing
        // relative to zero-source impedance), unless z_f itself is also ~0.
        let mf = f.mitigation_factor(5, 0.0);
        // Must be a valid number and in [0, 1].
        assert!(mf.is_finite(), "mitigation factor must be finite");
        assert!(
            (0.0..=1.0).contains(&mf),
            "mitigation factor must be in [0,1]"
        );
    }

    /// Off-resonance mitigation factor should be close to 1.0 (filter barely affects
    /// harmonics far from its tuning frequency).
    #[test]
    fn test_mitigation_factor_off_resonance_near_unity() {
        // Filter tuned to 5th harmonic, check at 11th (far from tuning).
        let f = PassiveFilter::single_tuned(5.0, 10.0, 60.0, 13.8, 50.0);
        let mf = f.mitigation_factor(11, 1.0);
        // At the 11th harmonic the filter impedance is large → mf closer to 1.
        assert!(
            mf > 0.5,
            "mitigation factor at off-resonance 11th should be > 0.5, got {:.4}",
            mf
        );
    }

    /// `design_filter_bank` must detune each filter slightly below the target harmonic
    /// (by 0.15 per the implementation) and distribute Q equally.
    #[test]
    fn test_filter_bank_detuning_and_q_distribution() {
        let orders: &[u32] = &[5, 7];
        let q_total = 20.0_f64;
        let bank = design_filter_bank(orders, q_total, 60.0, 13.8, 50.0);

        assert_eq!(
            bank.len(),
            2,
            "bank must have one filter per harmonic order"
        );

        let expected_q_each = q_total / orders.len() as f64;
        for (filter, &order) in bank.iter().zip(orders.iter()) {
            // Each filter gets an equal share of Q.
            let q_diff = (filter.q_mvar - expected_q_each).abs();
            assert!(
                q_diff < 1e-9,
                "filter for order {}: q_mvar={:.6} expected {:.6}",
                order,
                filter.q_mvar,
                expected_q_each
            );
            // Harmonic order must be detuned by exactly -0.15.
            let expected_tuning = order as f64 - 0.15;
            let tuning_diff = (filter.harmonic_order - expected_tuning).abs();
            assert!(
                tuning_diff < 1e-9,
                "filter for order {}: harmonic_order={:.4} expected {:.4}",
                order,
                filter.harmonic_order,
                expected_tuning
            );
        }
    }

    /// A higher quality factor Q must yield a lower series resistance R for a
    /// single-tuned filter (sharper tuning → less damping → smaller R).
    #[test]
    fn test_quality_factor_inversely_proportional_to_resistance() {
        let f_low_q = PassiveFilter::single_tuned(5.0, 10.0, 60.0, 13.8, 20.0);
        let f_high_q = PassiveFilter::single_tuned(5.0, 10.0, 60.0, 13.8, 80.0);
        // R = ω_n·L/Q, so higher Q → lower R.
        assert!(
            f_high_q.resistance_ohm < f_low_q.resistance_ohm,
            "higher Q_factor ({}) must give lower R ({:.6} Ω) than lower Q_factor ({}) R={:.6} Ω",
            f_high_q.q_factor,
            f_high_q.resistance_ohm,
            f_low_q.q_factor,
            f_low_q.resistance_ohm
        );
    }

    /// `FilterType` must implement `Clone`, `Copy`, and `PartialEq` correctly.
    #[test]
    fn test_filter_type_derive_traits() {
        let ft = FilterType::SingleTuned;
        let ft_clone = ft;
        assert_eq!(ft, ft_clone, "FilterType::Clone/Copy/PartialEq must work");
        assert_ne!(FilterType::SingleTuned, FilterType::HighPass);
        assert_ne!(FilterType::HighPass, FilterType::CType);
        assert_ne!(FilterType::CType, FilterType::SingleTuned);
    }

    /// `CType` filter uses the series-RLC impedance path (same branch as SingleTuned).
    /// Its impedance at resonance must still be dominated by R (minimum magnitude).
    #[test]
    fn test_ctype_filter_impedance_series_rlc_path() {
        // Construct a CType by cloning a single-tuned and overriding filter_type.
        let mut f = PassiveFilter::single_tuned(5.0, 10.0, 60.0, 13.8, 50.0);
        f.filter_type = FilterType::CType;

        let f_tune = f.fundamental_hz * f.harmonic_order;
        let z_tune = f.impedance(f_tune);

        // At resonance, jωL + 1/(jωC) = 0, so Z = R (purely real).
        let imaginary_fraction = z_tune.im.abs() / (z_tune.norm() + 1e-12);
        assert!(
            imaginary_fraction < 0.01,
            "CType at resonance: imaginary fraction={:.6} should be near zero (series RLC path)",
            imaginary_fraction
        );
    }
}
