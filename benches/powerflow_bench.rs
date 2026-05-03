use criterion::{criterion_group, criterion_main, Criterion};
use oxigrid::network::PowerNetwork;
use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod};

fn ieee14_nr_bench(c: &mut Criterion) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    let network = PowerNetwork::from_matpower(path).unwrap();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    c.bench_function("ieee14_nr", |b| {
        b.iter(|| network.solve_powerflow(&config).unwrap());
    });
}

fn ieee30_nr_bench(c: &mut Criterion) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee30.m");
    let network = PowerNetwork::from_matpower(path).unwrap();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    c.bench_function("ieee30_nr", |b| {
        b.iter(|| network.solve_powerflow(&config).unwrap());
    });
}

fn ieee14_dc_bench(c: &mut Criterion) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    let network = PowerNetwork::from_matpower(path).unwrap();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::DcApproximation,
        max_iter: 1,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    c.bench_function("ieee14_dc", |b| {
        b.iter(|| network.solve_powerflow(&config).unwrap());
    });
}

fn ieee118_nr_bench(c: &mut Criterion) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee118.m");
    let network = PowerNetwork::from_matpower(path).unwrap();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    c.bench_function("ieee118_nr", |b| {
        b.iter(|| network.solve_powerflow(&config).unwrap());
    });
}

fn ieee118_dc_bench(c: &mut Criterion) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee118.m");
    let network = PowerNetwork::from_matpower(path).unwrap();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::DcApproximation,
        max_iter: 1,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    c.bench_function("ieee118_dc", |b| {
        b.iter(|| network.solve_powerflow(&config).unwrap());
    });
}

fn ieee300_nr_dense(c: &mut Criterion) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee300.m");
    let network = PowerNetwork::from_matpower(path).unwrap();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 80,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    c.bench_function("ieee300_nr_dense", |b| {
        b.iter(|| network.solve_powerflow(&config).unwrap());
    });
}

fn ieee300_nr_sparse(c: &mut Criterion) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee300.m");
    let network = PowerNetwork::from_matpower(path).unwrap();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 80,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    c.bench_function("ieee300_nr_sparse", |b| {
        b.iter(|| network.solve_powerflow(&config).unwrap());
    });
}

fn ieee300_dc_bench(c: &mut Criterion) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee300.m");
    let network = PowerNetwork::from_matpower(path).unwrap();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::DcApproximation,
        max_iter: 1,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    c.bench_function("ieee300_dc", |b| {
        b.iter(|| network.solve_powerflow(&config).unwrap());
    });
}

criterion_group!(
    benches,
    ieee14_nr_bench,
    ieee30_nr_bench,
    ieee14_dc_bench,
    ieee118_nr_bench,
    ieee118_dc_bench,
    ieee300_nr_dense,
    ieee300_nr_sparse,
    ieee300_dc_bench
);
criterion_main!(benches);
