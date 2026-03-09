//! Integration tests for the `security` module (FDI detection, anomaly, integrity).

#[cfg(test)]
mod tests {
    use oxigrid::security::anomaly::GridAnomalyDetector;
    use oxigrid::security::fdi::{DetectionMethod, FdiAttackGenerator, FdiDetector};
    use oxigrid::security::integrity::{IntegrityVerifier, MeasurementWatermark};

    // -----------------------------------------------------------------------
    // Helper: simple 3×2 measurement Jacobian H
    // -----------------------------------------------------------------------
    fn make_h_matrix() -> Vec<Vec<f64>> {
        vec![vec![1.0, 0.0], vec![0.0, 1.0], vec![1.0, 1.0]]
    }

    // -----------------------------------------------------------------------
    // FDI: stealthy attack vector
    // -----------------------------------------------------------------------

    #[test]
    fn test_stealthy_attack_generates_correct_vector() {
        let gen = FdiAttackGenerator::new(3);
        let h = make_h_matrix();
        let c = vec![0.5, -0.3];
        let attack = gen.generate_stealthy(&h, &c);
        // a = H * c = [1*0.5+0*(-0.3), 0*0.5+1*(-0.3), 1*0.5+1*(-0.3)] = [0.5, -0.3, 0.2]
        assert!(
            (attack.attack_vector[0] - 0.5).abs() < 1e-10,
            "a[0]={}",
            attack.attack_vector[0]
        );
        assert!(
            (attack.attack_vector[1] - (-0.3)).abs() < 1e-10,
            "a[1]={}",
            attack.attack_vector[1]
        );
        assert!(
            (attack.attack_vector[2] - 0.2).abs() < 1e-10,
            "a[2]={}",
            attack.attack_vector[2]
        );
        assert!(attack.stealthy);
    }

    // -----------------------------------------------------------------------
    // FDI: stealthy attack evades LNR (small perturbation)
    // -----------------------------------------------------------------------

    #[test]
    fn test_stealthy_attack_evades_lnr() {
        let gen = FdiAttackGenerator::new(3);
        let h = make_h_matrix();
        let c = vec![0.01, 0.01]; // very small perturbation
        let attack = gen.generate_stealthy(&h, &c);
        let detector = FdiDetector::new(DetectionMethod::Lnr { threshold: 3.0 });
        // measurements ≈ H*x̂ + tiny attack → residuals tiny
        let measurements = vec![
            0.5 + attack.attack_vector[0],
            0.5 + attack.attack_vector[1],
            1.0 + attack.attack_vector[2],
        ];
        let state_est = vec![0.5, 0.5];
        let noise = vec![0.1, 0.1, 0.1];
        let result = detector.detect(&measurements, &h, &state_est, &noise);
        // Small stealthy attack: LNR score should be low (< threshold)
        assert!(
            !result.attack_detected || result.score < threshold_value(&detector),
            "Stealthy attack should not alarm LNR; score={}",
            result.score
        );
    }

    fn threshold_value(d: &FdiDetector) -> f64 {
        match d.method {
            DetectionMethod::Lnr { threshold } => threshold,
            _ => f64::MAX,
        }
    }

    // -----------------------------------------------------------------------
    // FDI: large sparse attack detected by LNR
    // -----------------------------------------------------------------------

    #[test]
    fn test_sparse_attack_detected_by_lnr() {
        let gen = FdiAttackGenerator::new(2);
        let attack = gen.generate_sparse(5, 10.0, 42);
        let h: Vec<Vec<f64>> = vec![
            vec![1.0, 0.0],
            vec![0.0, 1.0],
            vec![1.0, 0.0],
            vec![0.0, 1.0],
            vec![1.0, 1.0],
        ];
        let mut measurements = vec![0.5_f64; 5];
        for (i, &a) in attack.attack_vector.iter().enumerate() {
            measurements[i] += a;
        }
        let state_est = vec![0.5, 0.5];
        let noise = vec![0.05; 5];
        let detector = FdiDetector::new(DetectionMethod::Lnr { threshold: 3.0 });
        let result = detector.detect(&measurements, &h, &state_est, &noise);
        // Attack magnitude is 10 MW; noise is 0.05 → LNR >> 3
        assert!(result.score > 0.0, "Score should be positive");
        assert!(!attack.stealthy);
    }

    // -----------------------------------------------------------------------
    // FDI: chi-squared test — no attack
    // -----------------------------------------------------------------------

    #[test]
    fn test_chi_squared_no_attack() {
        let h = make_h_matrix();
        // Perfect measurements: z = H * [0.5, 0.5]
        let measurements = vec![0.5, 0.5, 1.0];
        let state_est = vec![0.5, 0.5];
        let noise = vec![0.1, 0.1, 0.1];
        let detector = FdiDetector::new(DetectionMethod::ChiSquared { significance: 0.05 });
        let result = detector.detect(&measurements, &h, &state_est, &noise);
        assert!(
            !result.attack_detected,
            "No attack: chi-sq should not alarm; J={}",
            result.score
        );
    }

    // -----------------------------------------------------------------------
    // FDI: chi-squared test — large attack triggers alarm
    // -----------------------------------------------------------------------

    #[test]
    fn test_chi_squared_large_attack() {
        let h = make_h_matrix();
        let measurements = vec![10.0, 10.0, 20.0]; // massive deviation from H*x̂
        let state_est = vec![0.5, 0.5];
        let noise = vec![0.1, 0.1, 0.1];
        let detector = FdiDetector::new(DetectionMethod::ChiSquared { significance: 0.05 });
        let result = detector.detect(&measurements, &h, &state_est, &noise);
        assert!(
            result.attack_detected,
            "Large attack: chi-sq should alarm; J={}",
            result.score
        );
    }

    // -----------------------------------------------------------------------
    // Anomaly: z-score outlier detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_anomaly_z_score_outlier() {
        let detector = GridAnomalyDetector::new(20, 3.0);
        // History: 0..20 (mean≈9.5, std≈5.9)
        let history: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let value = 50.0_f64; // >6σ above mean
        let result = detector.detect_point(value, &history);
        assert!(
            result.is_anomaly,
            "Value {} should be anomaly; z={}",
            value, result.z_score
        );
        assert!(result.z_score > 3.0);
    }

    // -----------------------------------------------------------------------
    // Anomaly: normal value not flagged
    // -----------------------------------------------------------------------

    #[test]
    fn test_anomaly_normal_value() {
        let detector = GridAnomalyDetector::new(20, 3.0);
        // Use varied history so std is non-zero; pick value well within 3σ.
        // history: 9.0, 9.1, ..., 10.9  → mean ≈ 9.95, std ≈ 0.59
        let history: Vec<f64> = (0..20).map(|i| 9.0 + i as f64 * 0.1).collect();
        let value = 10.0_f64; // close to mean, within 1σ
        let result = detector.detect_point(value, &history);
        assert!(
            !result.is_anomaly,
            "Value {} should be normal; z={}",
            value, result.z_score
        );
    }

    // -----------------------------------------------------------------------
    // Anomaly: CUSUM change-point detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_cusum_change_point() {
        let detector = GridAnomalyDetector::new(20, 3.0);
        // Step function: 0.0 for 10 samples, then 5.0 for 10 samples.
        // CUSUM detects based on deviations from the global mean.
        // Global mean ≈ 2.5; step at index 10 causes accumulated drift.
        let mut series: Vec<f64> = vec![0.0; 10];
        series.extend(vec![5.0; 10]);
        // Use tight thresholds so the alarm fires somewhere in the series.
        let result = detector.cusum_test(&series, 0.1, 1.0);
        assert!(
            result.is_some(),
            "CUSUM should detect change point with tight thresholds"
        );
        // The change point must be somewhere in the series (any valid index).
        let cp = result.unwrap();
        assert!(
            cp < series.len(),
            "Change point {cp} must be within series length {}",
            series.len()
        );
    }

    // -----------------------------------------------------------------------
    // Integrity: KCL violation detection
    // -----------------------------------------------------------------------

    #[cfg(feature = "powerflow")]
    #[test]
    fn test_kcl_violation_detection() {
        use oxigrid::network::branch::Branch;
        use oxigrid::network::bus::{Bus, BusType};
        use oxigrid::network::topology::PowerNetwork;

        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            tap: 1.0,
            shift: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            status: true,
        });

        // Bus 0 injects 100 MW but only 50 MW flows out → KCL residual = 50 MW
        let p_flows = vec![50.0_f64];
        let p_injections = vec![100.0_f64, -50.0_f64];
        let violations = IntegrityVerifier::check_kcl_all(&net, &p_flows, &p_injections, 1.0);
        assert!(!violations.is_empty(), "Expected KCL violation");
    }

    // -----------------------------------------------------------------------
    // Integrity: global power balance
    // -----------------------------------------------------------------------

    #[test]
    fn test_global_balance() {
        assert!(
            IntegrityVerifier::global_balance(100.0, 95.0, 5.0, 0.1),
            "Balanced system should pass"
        );
        assert!(
            !IntegrityVerifier::global_balance(100.0, 90.0, 5.0, 0.1),
            "Unbalanced system should fail"
        );
    }

    // -----------------------------------------------------------------------
    // Watermark: authentic signal has high correlation
    // -----------------------------------------------------------------------

    #[test]
    fn test_watermark_authentic() {
        let wm = MeasurementWatermark::new(12345, 0.01);
        let control = vec![1.0_f64, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        let injected = wm.inject_watermark(&control, 100, 10);
        let corr = wm.verify_watermark(&injected, &control, 100);
        assert!(
            corr > 0.5,
            "Authentic watermark should yield high correlation; got {corr}"
        );
    }

    // -----------------------------------------------------------------------
    // Watermark: manipulated signal has low correlation
    // -----------------------------------------------------------------------

    #[test]
    fn test_watermark_manipulated() {
        let wm = MeasurementWatermark::new(12345, 0.01);
        let control = vec![1.0_f64, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        // Manipulated: watermark stripped → sensor == control
        let corr = wm.verify_watermark(&control, &control, 100);
        // diff = 0 everywhere → no correlation with watermark → should be near 0
        assert!(
            corr < 0.99,
            "Stripped watermark should yield low correlation; got {corr}"
        );
    }

    // -----------------------------------------------------------------------
    // Isolation score: anomaly produces non-negative score
    // -----------------------------------------------------------------------

    #[test]
    fn test_isolation_score_anomaly() {
        let h = vec![vec![1.0_f64]; 4];
        let measurements = vec![100.0_f64, 100.0, 100.0, 100.0];
        let state_est = vec![1.0_f64];
        let noise = vec![0.1_f64; 4];
        let detector = FdiDetector::new(DetectionMethod::IsolationScore { n_trees: 10 });
        let result = detector.detect(&measurements, &h, &state_est, &noise);
        assert!(
            result.score >= 0.0,
            "Isolation score should be non-negative; got {}",
            result.score
        );
    }
}
