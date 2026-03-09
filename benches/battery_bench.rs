use criterion::{criterion_group, criterion_main, Criterion};
use oxigrid::battery::ecm::{OneRcModel, TwoRcModel};
use oxigrid::battery::soc::{CoulombCounter, EkfSocEstimator};
use oxigrid::battery::{BatteryModel, OcvSocCurve};
use oxigrid::units::{Current, Temperature, Voltage};

fn bench_rint_1000cycles(c: &mut Criterion) {
    use oxigrid::battery::ecm::RintModel;
    c.bench_function("rint_1000cycles_1c", |b| {
        b.iter(|| {
            let mut model = RintModel::new(OcvSocCurve::nmc_default(), 0.05, 3.0).with_soc(1.0);
            let temp = Temperature(298.15);
            let dt = 1.0;
            let mut total_energy = 0.0_f64;

            for _cycle in 0..1000 {
                // 1C discharge
                for _ in 0..3600 {
                    let state = model.step(Current(3.0), dt, temp);
                    total_energy += state.voltage.0 * 3.0 * dt;
                    if state.soc.0 < 0.01 {
                        break;
                    }
                }
                // Recharge (reset SoC for benchmark repeatability)
                model.soc = 1.0;
            }
            total_energy
        });
    });
}

fn bench_1rc_1000cycles(c: &mut Criterion) {
    c.bench_function("onerc_1000cycles_1c", |b| {
        b.iter(|| {
            let mut model =
                OneRcModel::new(OcvSocCurve::nmc_default(), 0.02, 0.05, 3000.0, 3.0).with_soc(1.0);
            let temp = Temperature(298.15);
            let dt = 1.0;
            let mut n = 0usize;

            for _cycle in 0..1000 {
                for _ in 0..3600 {
                    let state = model.step(Current(3.0), dt, temp);
                    n += 1;
                    if state.soc.0 < 0.01 {
                        break;
                    }
                }
                model.soc = 1.0;
                model.v_rc1 = 0.0;
            }
            n
        });
    });
}

fn bench_2rc_1000cycles(c: &mut Criterion) {
    c.bench_function("tworc_1000cycles_1c", |b| {
        b.iter(|| {
            let mut model = TwoRcModel::new(
                OcvSocCurve::nmc_default(),
                0.02,
                0.05,
                3000.0,
                0.03,
                500.0,
                3.0,
            )
            .with_soc(1.0);
            let temp = Temperature(298.15);
            let dt = 1.0;
            let mut n = 0usize;

            for _cycle in 0..1000 {
                for _ in 0..3600 {
                    let state = model.step(Current(3.0), dt, temp);
                    n += 1;
                    if state.soc.0 < 0.01 {
                        break;
                    }
                }
                model.soc = 1.0;
                model.v_rc1 = 0.0;
                model.v_rc2 = 0.0;
            }
            n
        });
    });
}

fn bench_ekf_1000steps(c: &mut Criterion) {
    c.bench_function("ekf_soc_1000steps", |b| {
        b.iter(|| {
            let curve = OcvSocCurve::nmc_default();
            let mut ekf = EkfSocEstimator::new(curve.clone(), 0.05, 3.0, 1.0);
            let current = Current(3.0);
            let temp = Temperature(298.15);

            for k in 0..1000 {
                let soc_true = 1.0 - k as f64 / 3600.0;
                let v = Voltage(curve.ocv(soc_true) - 3.0 * 0.05);
                ekf.update(current, v, 1.0, temp);
            }
            ekf.x
        });
    });
}

fn bench_coulomb_counter_1000cycles(c: &mut Criterion) {
    c.bench_function("coulomb_counter_1000cycles", |b| {
        b.iter(|| {
            let mut cc = CoulombCounter::new(1.0, 3.0);
            let mut n = 0usize;
            for _cycle in 0..1000 {
                for _ in 0..3600 {
                    cc.step(Current(3.0), 1.0);
                    n += 1;
                    if cc.soc < 0.01 {
                        break;
                    }
                }
                cc.soc = 1.0;
            }
            n
        });
    });
}

criterion_group!(
    benches,
    bench_rint_1000cycles,
    bench_1rc_1000cycles,
    bench_2rc_1000cycles,
    bench_ekf_1000steps,
    bench_coulomb_counter_1000cycles
);
criterion_main!(benches);
