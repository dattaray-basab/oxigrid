use criterion::{criterion_group, criterion_main, Criterion};
use oxigrid::network::PowerNetwork;
use oxigrid::optimize::opf::dc_opf::{solve_dc_opf, GenCost};

fn make_costs(network: &PowerNetwork) -> Vec<GenCost> {
    network
        .generators
        .iter()
        .map(|g| GenCost::quadratic(0.0, 20.0, 0.05, g.pmin.max(0.0), g.pmax.max(10.0)))
        .collect()
}

fn ieee14_dc_opf_bench(c: &mut Criterion) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    let network = PowerNetwork::from_matpower(path).unwrap();
    let costs = make_costs(&network);

    c.bench_function("ieee14_dc_opf", |b| {
        b.iter(|| solve_dc_opf(&network, &costs).unwrap());
    });
}

fn ieee30_dc_opf_bench(c: &mut Criterion) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee30.m");
    let network = PowerNetwork::from_matpower(path).unwrap();
    let costs = make_costs(&network);

    c.bench_function("ieee30_dc_opf", |b| {
        b.iter(|| solve_dc_opf(&network, &costs).unwrap());
    });
}

fn ieee118_dc_opf_bench(c: &mut Criterion) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee118.m");
    let network = PowerNetwork::from_matpower(path).unwrap();
    let costs = make_costs(&network);

    c.bench_function("ieee118_dc_opf", |b| {
        b.iter(|| solve_dc_opf(&network, &costs).unwrap());
    });
}

criterion_group!(
    benches,
    ieee14_dc_opf_bench,
    ieee30_dc_opf_bench,
    ieee118_dc_opf_bench
);
criterion_main!(benches);
