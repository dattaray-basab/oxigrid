//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

#[cfg(test)]
mod tests {
    use super::super::*;
    use std::f64::consts::PI;
    fn modal_07hz() -> GeneratorModal {
        GeneratorModal {
            gen_id: 1,
            mode_freq_hz: 0.7,
            damping_ratio: 0.02,
            residue_magnitude: 0.5,
            residue_angle_deg: 60.0,
            inertia_h: 5.0,
        }
    }
    fn config_pss1a() -> PssTuningConfig {
        PssTuningConfig {
            target_damping: 0.05,
            target_gain_db: 20.0,
            freq_min_hz: 0.01,
            freq_max_hz: 10.0,
            n_freq_points: 50,
            pss_type: PssType::Pss1A,
        }
    }
    fn default_pss1a() -> PssModel {
        PssModel::Pss1A {
            input: PssInput::RotorSpeed,
            tw: 10.0,
            lead_lag_1: (0.30, 0.05),
            lead_lag_2: (0.30, 0.05),
            k_s: 5.0,
            v_st_min: -0.1,
            v_st_max: 0.1,
        }
    }
    #[test]
    fn test_pss1a_lead_lag_t1_gt_t2() {
        let (t1, t2) = PssDesigner::lead_lag_constants(45.0, 0.7);
        assert!(t1 > t2, "Lead network: T1={t1:.4} must exceed T2={t2:.4}");
    }
    #[test]
    fn test_pss1a_phase_compensation_formula() {
        let modal = modal_07hz();
        let phi_c = 180.0 - modal.residue_angle_deg;
        assert!((phi_c - 120.0).abs() < 1e-9);
    }
    #[test]
    fn test_washout_dc_gain_zero() {
        let tf = TransferFunction::washout(10.0);
        let (mag_db, _) = tf.evaluate_at_freq(0.001);
        let mag_lin = 10.0_f64.powf(mag_db / 20.0);
        assert!(
            mag_lin < 0.1,
            "Washout should block near-DC: mag={mag_lin:.4}"
        );
    }
    #[test]
    fn test_washout_high_freq_near_unity() {
        let tf = TransferFunction::washout(10.0);
        let (mag_db, _) = tf.evaluate_at_freq(100.0);
        let mag_lin = 10.0_f64.powf(mag_db / 20.0);
        assert!(
            mag_lin > 0.9,
            "Washout should pass high freq: mag={mag_lin:.4}"
        );
    }
    #[test]
    fn test_lead_lag_provides_phase_lead() {
        let (t1, t2) = PssDesigner::lead_lag_constants(60.0, 0.7);
        let tf = TransferFunction::lead_lag(t1, t2);
        let (_, phase) = tf.evaluate_at_freq(0.7);
        assert!(
            phase > 0.0,
            "Lead network must give positive phase: {phase:.2}°"
        );
    }
    #[test]
    fn test_gain_margin_positive() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).expect("design should succeed");
        assert!(
            result.gain_margin_db > 0.0,
            "Gain margin must be positive: {:.2} dB",
            result.gain_margin_db
        );
    }
    #[test]
    fn test_phase_margin_positive() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).expect("design should succeed");
        assert!(
            result.phase_margin_deg > 0.0,
            "Phase margin must be positive: {:.2}°",
            result.phase_margin_deg
        );
    }
    #[test]
    fn test_freq_response_at_mode_frequency() {
        let tf = TransferFunction::lead_lag(0.3, 0.05).series(&TransferFunction::gain(5.0));
        let (mag_db, _) = tf.evaluate_at_freq(0.7);
        assert!(mag_db.is_finite(), "Frequency response must be finite");
    }
    #[test]
    fn test_high_freq_rolloff() {
        let tf = TransferFunction::washout(10.0).series(&TransferFunction::lead_lag(0.3, 0.05));
        let (mag_lo, _) = tf.evaluate_at_freq(1.0);
        let (mag_hi, _) = tf.evaluate_at_freq(1000.0);
        assert!(mag_lo.is_finite() && mag_hi.is_finite());
    }
    #[test]
    fn test_pss1a_tf_numerator_denominator_degree() {
        let pss = default_pss1a();
        let tf = PssDesigner::compute_transfer_function(&pss);
        assert!(!tf.numerator.is_empty());
        assert!(!tf.denominator.is_empty());
        assert_eq!(tf.numerator.len(), tf.denominator.len());
    }
    #[test]
    fn test_pss2b_model_construction() {
        let mut designer = PssDesigner::new(PssTuningConfig {
            pss_type: PssType::Pss2B,
            ..config_pss1a()
        });
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).expect("PSS2B design should succeed");
        match &result.pss_model {
            PssModel::Pss2B { k_s1, k_s2, .. } => {
                assert!(*k_s1 > 0.0);
                assert!(*k_s2 > 0.0);
            }
            _ => panic!("Expected Pss2B"),
        }
    }
    #[test]
    fn test_pss4b_model_construction() {
        let mut designer = PssDesigner::new(PssTuningConfig {
            pss_type: PssType::Pss4B,
            ..config_pss1a()
        });
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).expect("PSS4B design should succeed");
        match &result.pss_model {
            PssModel::Pss4B {
                low_band,
                inter_band,
                high_band,
                ..
            } => {
                assert!(low_band.k_l > 0.0);
                assert!(inter_band.k_l > 0.0);
                assert!(high_band.k_l > 0.0);
            }
            _ => panic!("Expected Pss4B"),
        }
    }
    #[test]
    fn test_pss4b_band_separation() {
        let mut designer = PssDesigner::new(PssTuningConfig {
            pss_type: PssType::Pss4B,
            ..config_pss1a()
        });
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).unwrap();
        match &result.pss_model {
            PssModel::Pss4B {
                low_band,
                high_band,
                ..
            } => {
                assert!(
                    low_band.t_l1 > high_band.t_l1,
                    "Low-band T should exceed high-band T: {} vs {}",
                    low_band.t_l1,
                    high_band.t_l1
                );
            }
            _ => panic!("Expected Pss4B"),
        }
    }
    #[test]
    fn test_simulate_pss_step_bounded() {
        let pss = default_pss1a();
        let mut state = PssState::zero(3);
        for _ in 0..100 {
            let out = PssDesigner::simulate_pss_step(&pss, &mut state, 1.0, 0.01);
            assert!(
                (-0.1 - 1e-10..=0.1 + 1e-10).contains(&out),
                "Output out of bounds: {out}"
            );
        }
    }
    #[test]
    fn test_simulate_pss_step_steady_state_zero() {
        let pss = default_pss1a();
        let mut state = PssState::zero(3);
        let mut last_out = 1.0_f64;
        for _ in 0..5000 {
            last_out = PssDesigner::simulate_pss_step(&pss, &mut state, 0.001, 0.01);
        }
        assert!(
            last_out.abs() < 0.01,
            "Steady-state output should decay: {last_out:.6}"
        );
    }
    #[test]
    fn test_tf_evaluate_at_freq_dc_gain() {
        let tf = TransferFunction::lead_lag(0.3, 0.05);
        let (mag_db, _) = tf.evaluate_at_freq(0.0001);
        let mag = 10.0_f64.powf(mag_db / 20.0);
        assert!(
            (mag - 1.0).abs() < 0.01,
            "DC gain should be ≈1, got {mag:.4}"
        );
    }
    #[test]
    fn test_compute_gain_phase_margins() {
        let fr: Vec<FreqResponsePoint> = vec![
            FreqResponsePoint {
                freq_hz: 1.0,
                magnitude_db: 10.0,
                phase_deg: -150.0,
            },
            FreqResponsePoint {
                freq_hz: 2.0,
                magnitude_db: 0.0,
                phase_deg: -160.0,
            },
            FreqResponsePoint {
                freq_hz: 4.0,
                magnitude_db: -10.0,
                phase_deg: -180.0,
            },
            FreqResponsePoint {
                freq_hz: 8.0,
                magnitude_db: -20.0,
                phase_deg: -200.0,
            },
        ];
        let (gm, pm) = PssDesigner::compute_gain_phase_margins(&fr);
        assert!(gm.is_finite());
        assert!(pm.is_finite());
    }
    #[test]
    fn test_design_converged_flag() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).unwrap();
        assert!(
            result.design_converged,
            "Design should converge for reasonable input"
        );
    }
    #[test]
    fn test_expected_damping_improvement_positive() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).unwrap();
        assert!(
            result.expected_damping_improvement > 0.0,
            "Damping improvement must be positive: {:.4}",
            result.expected_damping_improvement
        );
    }
    #[test]
    fn test_pss_input_rotor_speed_variant() {
        match PssInput::RotorSpeed {
            PssInput::RotorSpeed => {}
            _ => panic!("Wrong variant"),
        }
    }
    #[test]
    fn test_pss_input_electrical_power_variant() {
        match PssInput::ElectricalPower {
            PssInput::ElectricalPower => {}
            _ => panic!("Wrong variant"),
        }
    }
    #[test]
    fn test_freq_response_n_points() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).unwrap();
        assert_eq!(
            result.freq_response.len(),
            50,
            "Frequency response should have 50 points"
        );
    }
    #[test]
    fn test_phase_compensation_for_45_deg_residue() {
        let phi_c = 180.0 - 45.0_f64;
        assert!(
            (phi_c - 135.0).abs() < 1e-9,
            "Expected φ_c=135°, got {phi_c}"
        );
    }
    #[test]
    fn test_lead_time_constant_t1_gt_t2_for_lead() {
        let (t1, t2) = PssDesigner::lead_lag_constants(30.0, 1.0);
        assert!(
            t1 > t2,
            "T1={t1:.4} must exceed T2={t2:.4} for lead network"
        );
    }
    #[test]
    fn test_multiple_generators_independent_designs() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        designer.add_generator_modal(GeneratorModal {
            gen_id: 2,
            mode_freq_hz: 1.2,
            damping_ratio: 0.03,
            residue_magnitude: 0.3,
            residue_angle_deg: 45.0,
            inertia_h: 4.0,
        });
        let r1 = designer.design_pss(1).unwrap();
        let r2 = designer.design_pss(2).unwrap();
        assert_eq!(r1.generator_id, 1);
        assert_eq!(r2.generator_id, 2);
        match (&r1.pss_model, &r2.pss_model) {
            (
                PssModel::Pss1A {
                    lead_lag_1: ll1, ..
                },
                PssModel::Pss1A {
                    lead_lag_1: ll2, ..
                },
            ) => {
                assert!(
                    (ll1.0 - ll2.0).abs() > 1e-9,
                    "Different generators must produce different PSS parameters"
                );
            }
            _ => panic!("Expected Pss1A for both"),
        }
    }
    #[test]
    fn test_band_params_vmax_gt_vmin() {
        let band = PssBandParams {
            k_l: 5.0,
            t_l1: 0.3,
            t_l2: 0.05,
            t_l3: 0.3,
            t_l4: 0.05,
            v_lmax: 0.05,
            v_lmin: -0.05,
        };
        assert!(band.v_lmax > band.v_lmin);
    }
    #[test]
    fn test_tf_multiply_cascades_correctly() {
        let g1 = TransferFunction::gain(2.0);
        let g2 = TransferFunction::gain(3.0);
        let g = g1.multiply(&g2);
        let (mag_db, _) = g.evaluate_at_freq(0.0001);
        let mag = 10.0_f64.powf(mag_db / 20.0);
        assert!((mag - 6.0).abs() < 0.01, "2×3 = 6, got {mag:.4}");
    }
    #[test]
    fn test_pss1a_design_for_07hz_mode_lead() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).unwrap();
        match &result.pss_model {
            PssModel::Pss1A {
                lead_lag_1,
                lead_lag_2,
                ..
            } => {
                assert!(lead_lag_1.0 > lead_lag_1.1, "T1 > T2 for lead");
                assert!(lead_lag_2.0 > lead_lag_2.1, "T3 > T4 for lead");
            }
            _ => panic!("Expected Pss1A"),
        }
    }
    #[test]
    fn test_missing_generator_returns_error() {
        let designer = PssDesigner::new(config_pss1a());
        let result = designer.design_pss(99);
        assert!(result.is_err(), "Should return error for missing generator");
    }
    #[test]
    fn test_eigenvalue_stable() {
        let eig = Eigenvalue::new(-0.5, std::f64::consts::TAU);
        assert!(eig.is_stable());
        assert!(eig.damping_ratio > 0.0);
    }
    #[test]
    fn test_eigenvalue_poorly_damped() {
        let eig = Eigenvalue::new(-0.01, std::f64::consts::TAU);
        assert!(eig.is_poorly_damped());
    }
    #[test]
    fn test_eigenvalue_zero_real_zero_damping() {
        let omega = 2.0 * PI;
        let eig = Eigenvalue::new(0.0, omega);
        approx::assert_relative_eq!(eig.damping_ratio, 0.0, epsilon = 1e-10);
        approx::assert_relative_eq!(eig.frequency_hz, 1.0, epsilon = 1e-10);
        assert!(!eig.is_stable(), "Re=0 is not strictly stable");
    }
    #[test]
    fn test_eigenvalue_real_only_zero_frequency() {
        let eig = Eigenvalue::new(-2.0, 0.0);
        approx::assert_relative_eq!(eig.frequency_hz, 0.0, epsilon = 1e-10);
        assert!(eig.is_stable());
    }
    fn hp_generator() -> PssGeneratorModel {
        PssGeneratorModel {
            machine_id: 1,
            rated_mva: 100.0,
            h_inertia_s: 5.0,
            d_damping: 2.0,
            xd_transient: 0.3,
            td0_transient_s: 5.0,
            exciter_gain_ka: 50.0,
            exciter_time_ta_s: 0.05,
            k1: 1.5,
            k2: 1.2,
            k3: 0.4,
            k4: 1.0,
            k5: 0.1,
            k6: 0.4,
        }
    }
    fn hp_spec() -> PssDesignSpec {
        PssDesignSpec {
            target_mode_frequency_hz: 0.7,
            target_damping_ratio: 0.05,
            phase_compensation_deg: 60.0,
            max_gain: 20.0,
            washout_freq_hz: 0.1,
        }
    }
    #[test]
    fn test_hp_designer_open_loop_eigenvalues_count() {
        let designer = HpPssDesigner::new(hp_generator(), hp_spec());
        let eigs = designer.compute_open_loop_eigenvalues();
        assert_eq!(eigs.len(), 4, "4×4 system must have 4 eigenvalues");
    }
    #[test]
    fn test_hp_designer_phase_compensation_t1_ge_t2() {
        let designer = HpPssDesigner::new(hp_generator(), hp_spec());
        let (t1, t2, t3, t4) = designer.design_phase_compensation();
        assert!(
            t1 >= t2,
            "T1={t1:.4} must be >= T2={t2:.4} for lead network"
        );
        assert!(
            t3 >= t4,
            "T3={t3:.4} must be >= T4={t4:.4} for lead network"
        );
    }
    #[test]
    fn test_hp_designer_select_gain_bounded() {
        let spec = hp_spec();
        let max_g = spec.max_gain;
        let designer = HpPssDesigner::new(hp_generator(), spec);
        let (t1, t2, t3, t4) = designer.design_phase_compensation();
        let ks = designer.select_gain(t1, t2, t3, t4);
        assert!(
            ks <= max_g + 1e-9,
            "Selected gain {ks:.4} must be <= max_gain {max_g}"
        );
        assert!(ks > 0.0, "Selected gain must be positive");
    }
    #[test]
    fn test_hp_designer_design_pss1a_ok() {
        let designer = HpPssDesigner::new(hp_generator(), hp_spec());
        let result = designer.design_pss1a();
        assert!(result.is_ok(), "HP design_pss1a should succeed");
        let r = result.expect("design succeeded");
        assert!(
            !r.design_notes.is_empty(),
            "Design notes should be populated"
        );
        assert!(
            r.achieved_damping.is_finite(),
            "Achieved damping must be finite"
        );
    }
    #[test]
    fn test_simulate_pss2b_step_bounded() {
        let pss2b = PssModel::Pss2B {
            k_s1: 5.0,
            k_s2: 2.5,
            t_w1: 10.0,
            t_w2: 10.0,
            t_w3: 10.0,
            t_w4: 10.0,
            t1: 0.3,
            t2: 0.05,
            t3: 0.3,
            t4: 0.05,
            t10: 0.2,
            t11: 0.05,
            v_st_min: -0.1,
            v_st_max: 0.1,
        };
        let mut state = PssState::zero(3);
        for _ in 0..100 {
            let out = PssDesigner::simulate_pss_step(&pss2b, &mut state, 1.0, 0.01);
            assert!(
                (-0.1 - 1e-10..=0.1 + 1e-10).contains(&out),
                "PSS2B output out of limits: {out}"
            );
        }
    }
    #[test]
    fn test_simulate_pss4b_step_bounded() {
        let pss4b = PssModel::Pss4B {
            low_band: PssBandParams {
                k_l: 3.0,
                t_l1: 1.0,
                t_l2: 0.2,
                t_l3: 1.0,
                t_l4: 0.2,
                v_lmax: 0.05,
                v_lmin: -0.05,
            },
            inter_band: PssBandParams {
                k_l: 4.0,
                t_l1: 0.3,
                t_l2: 0.05,
                t_l3: 0.3,
                t_l4: 0.05,
                v_lmax: 0.05,
                v_lmin: -0.05,
            },
            high_band: PssBandParams {
                k_l: 2.0,
                t_l1: 0.08,
                t_l2: 0.02,
                t_l3: 0.08,
                t_l4: 0.02,
                v_lmax: 0.03,
                v_lmin: -0.03,
            },
            v_st_min: -0.1,
            v_st_max: 0.1,
        };
        let mut state = PssState::zero(3);
        for _ in 0..100 {
            let out = PssDesigner::simulate_pss_step(&pss4b, &mut state, 0.5, 0.01);
            assert!(
                (-0.1 - 1e-10..=0.1 + 1e-10).contains(&out),
                "PSS4B output out of limits: {out}"
            );
        }
    }
    #[test]
    fn test_bode_plot_monotonic_and_length() {
        let tf = TransferFunction::lead_lag(0.3, 0.05);
        let n = 20usize;
        let pts = tf.bode_plot(0.1, 10.0, n);
        assert_eq!(pts.len(), n, "bode_plot should return {n} points");
        for w in pts.windows(2) {
            assert!(
                w[1].0 > w[0].0,
                "Frequencies must be monotonically increasing: {} <= {}",
                w[1].0,
                w[0].0
            );
        }
    }
    #[test]
    fn test_pss_state_zero_initial_fields() {
        let n = 5usize;
        let s = PssState::zero(n);
        assert_eq!(s.states.len(), n);
        assert_eq!(s.output, 0.0);
        assert_eq!(s.time_s, 0.0);
        for &v in &s.states {
            approx::assert_relative_eq!(v, 0.0, epsilon = 1e-15);
        }
    }
    #[test]
    fn test_pss_tuning_config_default_values() {
        let cfg = PssTuningConfig::default();
        approx::assert_relative_eq!(cfg.target_damping, 0.05, epsilon = 1e-10);
        approx::assert_relative_eq!(cfg.target_gain_db, 20.0, epsilon = 1e-10);
        assert_eq!(cfg.pss_type, PssType::Pss1A);
        assert_eq!(cfg.n_freq_points, 100);
    }
}
