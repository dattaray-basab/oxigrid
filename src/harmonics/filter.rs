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

    fn sample_filter() -> PassiveFilter {
        PassiveFilter::single_tuned(5.0, 10.0, 60.0, 13.8, 50.0)
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
}
