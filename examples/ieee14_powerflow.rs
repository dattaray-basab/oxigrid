use oxigrid::prelude::*;

fn main() -> Result<()> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    let network = PowerNetwork::from_matpower(path)?;

    println!("IEEE 14-Bus Power Flow Example");
    println!("================================");
    println!(
        "Buses: {}, Branches: {}",
        network.bus_count(),
        network.branch_count()
    );

    // Newton-Raphson AC power flow
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    let result = network.solve_powerflow(&config)?;

    if result.converged {
        println!(
            "\nConverged in {} iterations (mismatch: {:.2e})",
            result.iterations, result.max_mismatch
        );
    } else {
        println!("\nDid NOT converge after {} iterations", result.iterations);
    }

    println!("\nBus Voltages:");
    println!("{:>5}  {:>10}  {:>12}", "Bus", "V (p.u.)", "Angle (deg)");
    println!("{}", "-".repeat(32));
    for (i, (vm, va)) in result
        .voltage_magnitude
        .iter()
        .zip(result.voltage_angle.iter())
        .enumerate()
    {
        println!("{:>5}  {:>10.4}  {:>12.3}", i + 1, vm, va.to_degrees());
    }

    println!(
        "\nSystem Losses: {:.3} MW  {:.3} MVAr",
        result.total_p_loss_mw, result.total_q_loss_mvar
    );

    println!("\nBranch Flows (top 5):");
    println!(
        "{:>8}  {:>8}  {:>12}  {:>12}",
        "From", "To", "P_from (MW)", "Q_from (MVAr)"
    );
    println!("{}", "-".repeat(48));
    for flow in result.branch_flows.iter().take(5) {
        println!(
            "{:>8}  {:>8}  {:>12.3}  {:>12.3}",
            flow.from_bus, flow.to_bus, flow.p_from_mw, flow.q_from_mvar
        );
    }

    // Also run DC approximation
    println!("\n--- DC Power Flow ---");
    let dc_config = PowerFlowConfig {
        method: PowerFlowMethod::DcApproximation,
        ..Default::default()
    };
    let dc_result = network.solve_powerflow(&dc_config)?;
    println!(
        "Bus 14 angle (DC): {:.3} deg  (NR: {:.3} deg)",
        dc_result.voltage_angle[13].to_degrees(),
        result.voltage_angle[13].to_degrees()
    );

    // Fast Decoupled Load Flow
    println!("\n--- Fast Decoupled Load Flow ---");
    let fdlf_config = PowerFlowConfig {
        method: PowerFlowMethod::FastDecoupled,
        max_iter: 50,
        tolerance: 1e-6,
        enforce_q_limits: false,
    };
    let fdlf_result = network.solve_powerflow(&fdlf_config)?;
    println!(
        "FDLF converged in {} iterations (mismatch: {:.2e})",
        fdlf_result.iterations, fdlf_result.max_mismatch
    );

    Ok(())
}
