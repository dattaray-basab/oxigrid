#![cfg(feature = "battery")]
/// Property-based tests for battery ECM energy conservation.
///
/// These tests verify that the battery models satisfy fundamental physical
/// conservation laws regardless of the specific parameter values.
use oxigrid::battery::ecm::{OneRcModel, RintModel, TwoRcModel};
use oxigrid::battery::{BatteryModel, OcvSocCurve};
use oxigrid::units::{Current, Temperature};
use proptest::prelude::*;

/// Maximum Ah error allowed per cycle relative to capacity.
const ENERGY_CONSERVATION_REL_TOL: f64 = 0.02; // 2% of capacity

// ── RintModel proptest ─────────────────────────────────────────────────────────

proptest! {
    /// Coulomb counting: total charge removed during discharge equals
    /// the integral of current × time (within numerical tolerance).
    #[test]
    fn prop_rint_discharge_coulomb_counting(
        r0 in 0.005f64..0.10,
        capacity_ah in 1.0f64..10.0,
        c_rate in 0.2f64..2.0,
    ) {
        let curve = OcvSocCurve::nmc_default();
        let current_a = c_rate * capacity_ah; // positive = discharge
        let mut model = RintModel::new(curve, r0, capacity_ah).with_soc(1.0);
        let dt = 1.0_f64; // 1 second
        let temp = Temperature(298.15);

        let mut ah_extracted = 0.0_f64;
        let mut steps = 0usize;
        loop {
            let state = model.step(Current(current_a), dt, temp);
            ah_extracted += current_a * dt / 3600.0;
            steps += 1;
            if state.soc.0 < 0.01 || steps > 36_000 { break; }
        }

        // SoC should have dropped by approximately ah_extracted / capacity_ah
        let expected_soc_drop = (ah_extracted / capacity_ah).min(1.0);
        let actual_soc_drop = 1.0 - model.soc;
        let error = (actual_soc_drop - expected_soc_drop).abs();

        prop_assert!(
            error < ENERGY_CONSERVATION_REL_TOL,
            "Coulomb counting error {:.4} > {ENERGY_CONSERVATION_REL_TOL:.4} \
             (c_rate={c_rate:.2}, capacity={capacity_ah:.1} Ah)",
            error
        );
    }

    /// After a complete discharge (SoC → 0) and full recharge (SoC → 1),
    /// the net Ah throughput should balance to within tolerance.
    #[test]
    fn prop_rint_charge_discharge_balance(
        r0 in 0.005f64..0.08,
        capacity_ah in 2.0f64..8.0,
    ) {
        let curve = OcvSocCurve::nmc_default();
        let mut model = RintModel::new(curve, r0, capacity_ah).with_soc(1.0);
        let dt = 10.0_f64; // 10-second steps for speed
        let temp = Temperature(298.15);
        let i_discharge = Current(capacity_ah); // 1C discharge
        let i_charge = Current(-capacity_ah);   // 1C charge (negative = charging)

        // Discharge from 1.0 to ~0
        let mut ah_out = 0.0;
        for _ in 0..36_000 {
            let s = model.step(i_discharge, dt, temp);
            ah_out += capacity_ah * dt / 3600.0;
            if s.soc.0 < 0.01 { break; }
        }

        // Recharge from ~0 to ~1
        let mut ah_in = 0.0;
        for _ in 0..36_000 {
            let s = model.step(i_charge, dt, temp);
            ah_in += capacity_ah * dt / 3600.0;
            if s.soc.0 > 0.99 { break; }
        }

        // Charge in ≈ charge out (within tolerance)
        let balance_error = (ah_in - ah_out).abs() / capacity_ah;
        prop_assert!(
            balance_error < 0.05,
            "Charge balance error {:.4} (out={ah_out:.2}, in={ah_in:.2}, cap={capacity_ah:.1})",
            balance_error
        );
    }

    /// SoC must remain in [0, 1] for any current/capacity combination.
    #[test]
    fn prop_rint_soc_bounded(
        r0 in 0.001f64..0.15,
        capacity_ah in 0.5f64..20.0,
        current_sign in 0i32..2,
        c_rate in 0.1f64..3.0,
    ) {
        let curve = OcvSocCurve::nmc_default();
        let mut model = RintModel::new(curve, r0, capacity_ah).with_soc(0.5);
        let dt = 1.0_f64;
        let temp = Temperature(298.15);
        let sign = if current_sign == 0 { 1.0 } else { -1.0 };
        let current = Current(sign * c_rate * capacity_ah);

        for _ in 0..1000 {
            let s = model.step(current, dt, temp);
            prop_assert!(
                s.soc.0 >= 0.0 && s.soc.0 <= 1.0,
                "SoC {:.6} out of [0,1]", s.soc.0
            );
        }
    }
}

// ── OneRcModel proptest ───────────────────────────────────────────────────────

proptest! {
    #[test]
    fn prop_1rc_soc_bounded(
        r0 in 0.005f64..0.08,
        r1 in 0.001f64..0.05,
        c1 in 100.0f64..5000.0,
        capacity_ah in 1.0f64..10.0,
        current_a in -20.0f64..20.0,
    ) {
        let curve = OcvSocCurve::nmc_default();
        let mut model = OneRcModel::new(curve, r0, r1, c1, capacity_ah).with_soc(0.5);
        let temp = Temperature(298.15);

        for _ in 0..500 {
            let s = model.step(Current(current_a), 1.0, temp);
            prop_assert!(
                s.soc.0 >= 0.0 && s.soc.0 <= 1.0,
                "1RC SoC {:.6} out of [0,1]", s.soc.0
            );
        }
    }

    #[test]
    fn prop_1rc_voltage_positive_on_discharge(
        r0 in 0.005f64..0.05,
        r1 in 0.001f64..0.03,
        c1 in 500.0f64..3000.0,
        capacity_ah in 2.0f64..8.0,
    ) {
        let curve = OcvSocCurve::nmc_default();
        let mut model = OneRcModel::new(curve, r0, r1, c1, capacity_ah).with_soc(0.8);
        let temp = Temperature(298.15);
        let current = Current(capacity_ah); // 1C discharge

        for _ in 0..3600 {
            let s = model.step(current, 1.0, temp);
            prop_assert!(
                s.voltage.0 > 2.0,
                "Terminal voltage {:.4} V is unreasonably low", s.voltage.0
            );
            if s.soc.0 < 0.05 { break; }
        }
    }
}

// ── TwoRcModel proptest ───────────────────────────────────────────────────────

proptest! {
    #[test]
    fn prop_2rc_soc_bounded(
        r0 in 0.005f64..0.08,
        r1 in 0.001f64..0.04,
        c1 in 200.0f64..4000.0,
        r2 in 0.001f64..0.04,
        c2 in 50.0f64..1000.0,
        capacity_ah in 1.0f64..10.0,
        current_a in -15.0f64..15.0,
    ) {
        let curve = OcvSocCurve::nmc_default();
        let mut model = TwoRcModel::new(curve, r0, r1, c1, r2, c2, capacity_ah).with_soc(0.5);
        let temp = Temperature(298.15);

        for _ in 0..200 {
            let s = model.step(Current(current_a), 1.0, temp);
            prop_assert!(
                s.soc.0 >= 0.0 && s.soc.0 <= 1.0,
                "2RC SoC {:.6} out of [0,1]", s.soc.0
            );
        }
    }
}
