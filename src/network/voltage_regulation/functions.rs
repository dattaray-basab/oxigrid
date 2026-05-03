//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

#[cfg(test)]
mod tests {
    use super::super::*;
    fn make_oltc() -> OltcController {
        OltcController::new(1, 10, -16, 16)
    }
    fn make_cap() -> CapacitorBank {
        CapacitorBank::new(1, 10, 4, 150.0)
    }
    #[test]
    fn test_oltc_voltage_ratio_positive_tap() {
        let mut oltc = make_oltc();
        oltc.current_tap = 1;
        assert!(oltc.voltage_ratio() > 1.0, "positive tap must raise ratio");
    }
    #[test]
    fn test_oltc_voltage_ratio_negative_tap() {
        let mut oltc = make_oltc();
        oltc.current_tap = -1;
        assert!(oltc.voltage_ratio() < 1.0, "negative tap must lower ratio");
    }
    #[test]
    fn test_oltc_action_low_voltage() {
        let oltc = make_oltc();
        assert_eq!(oltc.compute_action(0.93), Some(1));
    }
    #[test]
    fn test_oltc_action_high_voltage() {
        let oltc = make_oltc();
        assert_eq!(oltc.compute_action(1.06), Some(-1));
    }
    #[test]
    fn test_oltc_action_within_deadband() {
        let oltc = make_oltc();
        assert_eq!(oltc.compute_action(1.005), None);
    }
    #[test]
    fn test_oltc_tap_limits() {
        let mut oltc = make_oltc();
        oltc.current_tap = 16;
        assert!(
            oltc.apply_tap(1).is_err(),
            "tapping beyond max_tap must fail"
        );
    }
    #[test]
    fn test_oltc_daily_limit() {
        let mut oltc = make_oltc();
        oltc.daily_operations = 20;
        assert!(
            oltc.apply_tap(1).is_err(),
            "exceeding daily limit must fail"
        );
    }
    #[test]
    fn test_oltc_apply_tap_success() {
        let mut oltc = make_oltc();
        let ratio = oltc.apply_tap(1).expect("first tap should succeed");
        assert!((ratio - 1.00625).abs() < 1e-9);
        assert_eq!(oltc.current_tap, 1);
        assert_eq!(oltc.daily_operations, 1);
    }
    #[test]
    fn test_oltc_reset_daily_counter() {
        let mut oltc = make_oltc();
        oltc.daily_operations = 15;
        oltc.reset_daily_counter();
        assert_eq!(oltc.daily_operations, 0);
    }
    #[test]
    fn test_capacitor_bank_total_kvar() {
        let mut cap = make_cap();
        cap.active_steps = 2;
        assert!((cap.total_kvar() - 300.0).abs() < 1e-9);
    }
    #[test]
    fn test_capacitor_switch_in_low_v() {
        let cap = make_cap();
        assert_eq!(cap.compute_switching_action(0.95), 1);
    }
    #[test]
    fn test_capacitor_switch_out_high_v() {
        let mut cap = make_cap();
        cap.active_steps = 2;
        assert_eq!(cap.compute_switching_action(1.05), -1);
    }
    #[test]
    fn test_capacitor_limits() {
        let mut cap = make_cap();
        cap.active_steps = 4;
        assert_eq!(cap.compute_switching_action(0.90), 0);
        cap.active_steps = 0;
        assert_eq!(cap.compute_switching_action(1.06), 0);
    }
    #[test]
    fn test_capacitor_apply_switching_clamped() {
        let mut cap = make_cap();
        let kvar = cap.apply_switching(10).expect("clamped apply must succeed");
        assert_eq!(cap.active_steps, 4);
        assert!((kvar - 600.0).abs() < 1e-9);
    }
    #[test]
    fn test_voltage_profile_violations() {
        let sys = VoltageRegulationSystem::new();
        let v = vec![0.93, 1.00, 1.06, 0.98];
        let ids: Vec<usize> = (0..4).collect();
        let profile = sys.assess_voltage_profile(&v, &ids);
        assert_eq!(profile.n_violations, 2);
    }
    #[test]
    fn test_voltage_profile_min_max() {
        let sys = VoltageRegulationSystem::new();
        let v = vec![0.93, 1.00, 1.06, 0.98];
        let ids: Vec<usize> = (0..4).collect();
        let profile = sys.assess_voltage_profile(&v, &ids);
        assert!((profile.min_voltage_pu - 0.93).abs() < 1e-9);
        assert!((profile.max_voltage_pu - 1.06).abs() < 1e-9);
    }
    #[test]
    fn test_coordination_actions_nonempty() {
        let mut sys = VoltageRegulationSystem::new();
        let cap = CapacitorBank::new(1, 0, 4, 150.0);
        sys.add_capacitor_bank(cap);
        let v = vec![0.92];
        let ids = vec![0_usize];
        let profile = sys.assess_voltage_profile(&v, &ids);
        let actions = sys.compute_regulation_actions(&profile);
        assert!(!actions.is_empty());
    }
    #[test]
    fn test_coordination_prioritizes_cheapest() {
        let mut sys = VoltageRegulationSystem::new();
        let cap = CapacitorBank::new(1, 0, 4, 150.0);
        sys.add_capacitor_bank(cap);
        let oltc = OltcController::new(2, 0, -16, 16);
        sys.add_oltc(oltc);
        let v = vec![0.92];
        let ids = vec![0_usize];
        let profile = sys.assess_voltage_profile(&v, &ids);
        let actions = sys.compute_regulation_actions(&profile);
        assert!(!actions.is_empty());
        assert!(actions[0].cost_usd <= actions.last().map(|a| a.cost_usd).unwrap_or(f64::MAX));
    }
    #[test]
    fn test_ldc_compensated_voltage() {
        let v_ldc = LineDropCompensator::compute_ldc_voltage(1.0, 0.1, 0.02, 0.05, 0.8);
        assert!((v_ldc - 0.9954).abs() < 1e-9);
    }
    #[test]
    fn test_vvo_optimizer_reduces_violations() {
        let mut sys = VoltageRegulationSystem::new();
        let cap = CapacitorBank::new(1, 0, 4, 150.0);
        sys.add_capacitor_bank(cap);
        let v = vec![0.93, 1.00];
        let ids = vec![0_usize, 1];
        let profile = sys.assess_voltage_profile(&v, &ids);
        let actions = VoltageVarOptimizer::optimize_setpoints(&profile, &sys);
        assert!(!actions.is_empty());
    }
    #[test]
    fn test_sensitivity_matrix_usage() {
        let mut sys = VoltageRegulationSystem::new();
        let cap0 = CapacitorBank::new(0, 0, 4, 100.0);
        let cap1 = CapacitorBank::new(1, 0, 4, 100.0);
        sys.add_capacitor_bank(cap0);
        sys.add_capacitor_bank(cap1);
        sys.set_sensitivity_matrix(vec![vec![0.001, 0.05]]);
        let v = vec![0.93];
        let ids = vec![0_usize];
        let profile = sys.assess_voltage_profile(&v, &ids);
        let actions = VoltageVarOptimizer::optimize_setpoints(&profile, &sys);
        assert!(!actions.is_empty());
        assert_eq!(
            actions[0].device_id, 1,
            "higher-sensitivity device must be chosen"
        );
    }
    #[test]
    fn test_regulator_boost_limits() {
        let mut reg = VoltageRegulatorUnit::new(1, 0, 1);
        reg.apply_boost(0.5);
        assert!((reg.current_boost_pu - 0.1).abs() < 1e-9);
        reg.apply_boost(-0.5);
        assert!((reg.current_boost_pu - (-0.1)).abs() < 1e-9);
    }
    #[test]
    fn test_run_coordination_step() {
        let mut sys = VoltageRegulationSystem::new();
        let cap = CapacitorBank::new(1, 0, 4, 150.0);
        sys.add_capacitor_bank(cap);
        let v = vec![0.92, 1.0, 1.02];
        let ids = vec![0_usize, 1, 2];
        let (profile, actions) = sys.run_coordination_step(&v, &ids);
        assert_eq!(profile.bus_voltages_pu.len(), 3);
        assert!(!actions.is_empty());
    }
    #[test]
    fn test_capacitor_fault_no_action() {
        let mut cap = make_cap();
        cap.status = CapacitorStatus::Fault;
        assert_eq!(cap.compute_switching_action(0.90), 0);
        assert!(cap.apply_switching(1).is_err());
    }
    #[test]
    fn test_voltage_regulator_step_size() {
        let reg = VoltageRegulatorUnit::new(1, 0, 1);
        assert!((reg.step_size_pu() - 0.00625).abs() < 1e-9);
    }
    #[test]
    fn tap_ratio_at_zero() {
        let reg = StepRegulator::new(0, 500.0, 11.0, 1.0);
        assert!((reg.tap_ratio() - 1.0).abs() < 1e-12);
    }
    #[test]
    fn tap_up_after_delay() {
        let mut reg = StepRegulator::new(1, 500.0, 11.0, 1.0);
        reg.time_delay_s = 5.0;
        let action1 = reg.step_control(0.95, 3.0);
        assert_eq!(action1, TapAction::NoChange);
        let action2 = reg.step_control(0.95, 3.0);
        assert_eq!(action2, TapAction::TapUp);
        assert_eq!(reg.current_tap, 1);
        assert_eq!(reg.total_operations, 1);
    }
    #[test]
    fn tap_down_after_delay() {
        let mut reg = StepRegulator::new(2, 500.0, 11.0, 1.0);
        reg.time_delay_s = 4.0;
        let _ = reg.step_control(1.06, 5.0);
        assert_eq!(reg.current_tap, -1);
    }
    #[test]
    fn effective_voltage_range_bounds() {
        let reg = StepRegulator::new(0, 500.0, 11.0, 1.0);
        let (v_min, v_max) = reg.effective_voltage_range();
        assert!(v_min < 1.0);
        assert!(v_max > 1.0);
        assert!((v_min - 0.9).abs() < 1e-10);
        assert!((v_max - 1.1).abs() < 1e-10);
    }
    #[test]
    fn compensated_voltage_ldc() {
        let mut reg = StepRegulator::new(0, 500.0, 11.0, 1.0);
        reg.r_compensator = 0.05;
        reg.x_compensator = 0.1;
        let v_comp = reg.compensated_voltage(1.0, 0.5, 0.9);
        assert!(v_comp < 1.0);
    }
    #[test]
    fn cap_q_proportional_to_steps() {
        let mut cap = StepCapacitorBank::new(0, 1, 3.0, 11.0, 6);
        cap.current_steps = 3;
        let q = cap.q_injected_mvar();
        assert!((q - 1.5).abs() < 1e-10);
    }
    #[test]
    fn cap_q_zero_when_off() {
        let cap = StepCapacitorBank::new(0, 1, 3.0, 11.0, 6);
        assert_eq!(cap.q_injected_mvar(), 0.0);
    }
    #[test]
    fn cap_switch_in_below_v_on() {
        let mut cap = StepCapacitorBank::new(0, 1, 3.0, 11.0, 4);
        cap.time_delay_s = 5.0;
        let a1 = cap.step_control(0.95, 3.0);
        assert_eq!(a1, CapAction::NoChange);
        let a2 = cap.step_control(0.95, 3.0);
        assert_eq!(a2, CapAction::StepIn);
        assert_eq!(cap.current_steps, 1);
    }
    #[test]
    fn cap_switch_out_above_v_off() {
        let mut cap = StepCapacitorBank::new(0, 1, 3.0, 11.0, 4);
        cap.current_steps = 4;
        cap.time_delay_s = 5.0;
        let _ = cap.step_control(1.05, 3.0);
        let a = cap.step_control(1.05, 3.0);
        assert_eq!(a, CapAction::StepOut);
        assert_eq!(cap.current_steps, 3);
    }
    #[test]
    fn cap_switch_step_bounds() {
        let mut cap = StepCapacitorBank::new(0, 1, 3.0, 11.0, 4);
        assert!(cap.switch_step(2).is_ok());
        assert_eq!(cap.current_steps, 2);
        assert!(cap.switch_step(10).is_err());
        assert!(cap.switch_step(-5).is_err());
    }
    #[test]
    fn svc_q_zero_at_setpoint() {
        let svc = SvcModel::new(0, -10.0, 10.0, 1.0);
        let q = svc.q_from_voltage(1.0);
        assert!(q.abs() < 1e-10);
    }
    #[test]
    fn svc_capacitive_when_low_v() {
        let svc = SvcModel::new(0, -10.0, 10.0, 1.0);
        let q = svc.q_from_voltage(0.95);
        assert!(q > 0.0);
    }
    #[test]
    fn svc_step_dynamics() {
        let mut svc = SvcModel::new(0, -10.0, 10.0, 1.0);
        let q = svc.step(0.95, 0.05);
        assert!(q > 0.0);
        assert!(q <= svc.q_max_mvar);
    }
    #[test]
    fn coord_detects_undervoltage() {
        let ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        let voltages = vec![1.0, 0.90, 1.02, 0.94];
        let violations = ctrl.check_voltage_violations(&voltages);
        assert_eq!(violations.len(), 2);
        for v in &violations {
            assert_eq!(v.violation_type, ViolationType::UnderVoltage);
        }
    }
    #[test]
    fn coord_detects_overvoltage() {
        let ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        let voltages = vec![1.06, 1.0, 1.07];
        let violations = ctrl.check_voltage_violations(&voltages);
        assert_eq!(violations.len(), 2);
        assert!(violations
            .iter()
            .all(|v| v.violation_type == ViolationType::OverVoltage));
    }
    #[test]
    fn coord_total_reactive_support() {
        let mut ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        let mut cap = StepCapacitorBank::new(0, 0, 6.0, 11.0, 3);
        cap.current_steps = 3;
        ctrl.add_capacitor(cap);
        let mut svc = SvcModel::new(1, -5.0, 5.0, 1.0);
        svc.q_output_mvar = 2.0;
        ctrl.add_svc(svc);
        let total = ctrl.total_reactive_support_mvar();
        assert!((total - 8.0).abs() < 1e-9);
    }
    #[test]
    fn coord_step_runs_without_panic() {
        let mut ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        ctrl.add_regulator(StepRegulator::new(0, 500.0, 11.0, 1.0));
        ctrl.add_capacitor(StepCapacitorBank::new(0, 1, 3.0, 11.0, 4));
        ctrl.add_svc(SvcModel::new(2, -5.0, 5.0, 1.0));
        let mut voltages = vec![1.0, 0.94, 0.97, 1.0];
        let result = ctrl.step(&mut voltages, 1.0);
        assert!(result.svc_q_total_mvar.is_finite());
    }
    #[test]
    fn vvo_optimize_no_panic() {
        let mut ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        ctrl.add_regulator(StepRegulator::new(0, 500.0, 11.0, 1.0));
        ctrl.add_capacitor(StepCapacitorBank::new(0, 1, 6.0, 11.0, 3));
        let voltages = vec![1.0, 0.96, 0.97];
        let vvo = VvoOptimizer::new();
        let result = vvo.optimize_setpoints(&mut ctrl, &voltages, 5.0);
        assert_eq!(result.optimal_v_setpoints.len(), 1);
        assert_eq!(result.optimal_cap_steps.len(), 1);
        assert!(result.estimated_loss_reduction_pct >= 0.0);
        assert!(result.voltage_improvement_pu >= 0.0);
    }
    #[test]
    fn profile_worst_case_bus() {
        let mut analyzer = VoltageProfileAnalyzer::new(11.0);
        analyzer.bus_distances_km = vec![0.0, 1.0, 2.0, 3.0];
        let voltages = vec![1.0, 0.98, 0.94, 0.97];
        let (idx, v) = analyzer.worst_case_bus(&voltages);
        assert_eq!(idx, 2);
        assert!((v - 0.94).abs() < 1e-12);
    }
    #[test]
    fn profile_min_max_correct() {
        let mut analyzer = VoltageProfileAnalyzer::new(11.0);
        analyzer.bus_distances_km = vec![0.0, 1.0, 2.0];
        let voltages = vec![1.02, 0.97, 0.94];
        let profile = analyzer.compute_profile(&voltages);
        assert!((profile.min_voltage_pu - 0.94).abs() < 1e-12);
        assert!((profile.max_voltage_pu - 1.02).abs() < 1e-12);
    }
    #[test]
    fn profile_kv_conversion() {
        let mut analyzer = VoltageProfileAnalyzer::new(11.0);
        analyzer.bus_distances_km = vec![0.0, 1.0];
        let voltages = vec![1.0, 0.95];
        let profile = analyzer.compute_profile(&voltages);
        assert!((profile.voltages_kv[0] - 11.0).abs() < 1e-10);
        assert!((profile.voltages_kv[1] - 10.45).abs() < 1e-10);
    }
    #[test]
    fn voltage_drop_pct_correct() {
        let analyzer = VoltageProfileAnalyzer::new(11.0);
        let voltages = vec![1.0, 0.95, 0.92];
        let drop = analyzer.voltage_drop_pct(&voltages);
        assert!((drop - 8.0).abs() < 1e-10);
    }
}
