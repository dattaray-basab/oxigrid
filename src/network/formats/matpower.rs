use crate::error::{OxiGridError, Result};
use crate::network::branch::Branch;
use crate::network::bus::{Bus, BusType};
use crate::network::topology::{Generator, PowerNetwork};
use crate::units::{Power, ReactivePower, Voltage};

pub fn parse_matpower_file(path: &str) -> Result<PowerNetwork> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| OxiGridError::ParseError(format!("Failed to read file {path}: {e}")))?;
    parse_matpower_string(&content)
}

pub fn parse_matpower_string(content: &str) -> Result<PowerNetwork> {
    let base_mva = parse_base_mva(content)?;
    let buses = parse_bus_data(content)?;
    let branches = parse_branch_data(content)?;
    let generators = parse_gen_data(content)?;

    let mut network = PowerNetwork::new(base_mva);
    network.buses = buses;
    network.branches = branches;
    network.generators = generators;

    // Set voltage magnitudes from generator data
    for gen in &network.generators {
        if !gen.status {
            continue;
        }
        if let Ok(idx) = network.bus_index(gen.bus_id) {
            if network.buses[idx].bus_type == BusType::PV
                || network.buses[idx].bus_type == BusType::Slack
            {
                network.buses[idx].vm = gen.vg;
            }
        }
    }

    network.validate()?;
    Ok(network)
}

fn parse_base_mva(content: &str) -> Result<f64> {
    for line in content.lines() {
        let line = line.trim();
        if line.starts_with("mpc.baseMVA") {
            let val = line
                .split('=')
                .nth(1)
                .and_then(|s| s.trim().trim_end_matches(';').trim().parse::<f64>().ok())
                .ok_or_else(|| OxiGridError::ParseError("Failed to parse baseMVA".into()))?;
            return Ok(val);
        }
    }
    Err(OxiGridError::ParseError("baseMVA not found".into()))
}

fn extract_matrix_data(content: &str, matrix_name: &str) -> Result<Vec<Vec<f64>>> {
    let marker = format!("mpc.{matrix_name}");
    let mut in_section = false;
    let mut rows = Vec::new();

    for line in content.lines() {
        let trimmed = line.trim();

        if !in_section {
            if trimmed.starts_with(&marker) && trimmed.contains('[') {
                in_section = true;
                // Check if data starts on same line after '['
                if let Some(after_bracket) = trimmed.split('[').nth(1) {
                    let data_part = after_bracket.trim_end_matches("];").trim();
                    if !data_part.is_empty() {
                        if let Some(row) = parse_row(data_part) {
                            rows.push(row);
                        }
                    }
                    if trimmed.ends_with("];") {
                        break;
                    }
                }
                continue;
            }
        } else {
            if trimmed.starts_with("];") || trimmed == "];" {
                break;
            }
            // Remove trailing semicolon and comments
            let data_part = trimmed
                .split('%')
                .next()
                .unwrap_or("")
                .trim()
                .trim_end_matches(';');
            if !data_part.is_empty() {
                if let Some(row) = parse_row(data_part) {
                    rows.push(row);
                }
            }
        }
    }

    if rows.is_empty() {
        return Err(OxiGridError::ParseError(format!(
            "No data found for {matrix_name}"
        )));
    }
    Ok(rows)
}

fn parse_row(line: &str) -> Option<Vec<f64>> {
    let values: std::result::Result<Vec<f64>, _> = line
        .split_whitespace()
        .filter(|s| !s.is_empty())
        .map(|s| s.trim_end_matches(';').parse::<f64>())
        .collect();
    values.ok().filter(|v| !v.is_empty())
}

fn parse_bus_data(content: &str) -> Result<Vec<Bus>> {
    let rows = extract_matrix_data(content, "bus")?;
    let mut buses = Vec::with_capacity(rows.len());

    for row in &rows {
        if row.len() < 13 {
            return Err(OxiGridError::ParseError(format!(
                "Bus data row has {} columns, expected at least 13",
                row.len()
            )));
        }

        let bus_id = row[0] as usize;
        let bus_type_code = row[1] as i32;
        let bus_type = match bus_type_code {
            3 => BusType::Slack,
            2 => BusType::PV,
            1 => BusType::PQ,
            _ => BusType::PQ,
        };

        buses.push(Bus {
            id: bus_id,
            name: format!("Bus {bus_id}"),
            bus_type,
            base_kv: Voltage(row[9]),
            vm: row[7],
            va: row[8].to_radians(),
            pd: Power(row[2]),
            qd: ReactivePower(row[3]),
            gs: row[4],
            bs: row[5],
            zone: Some(row[6] as u32),
        });
    }

    Ok(buses)
}

fn parse_branch_data(content: &str) -> Result<Vec<Branch>> {
    let rows = extract_matrix_data(content, "branch")?;
    let mut branches = Vec::with_capacity(rows.len());

    for row in &rows {
        if row.len() < 11 {
            return Err(OxiGridError::ParseError(format!(
                "Branch data row has {} columns, expected at least 11",
                row.len()
            )));
        }

        branches.push(Branch {
            from_bus: row[0] as usize,
            to_bus: row[1] as usize,
            r: row[2],
            x: row[3],
            b: row[4],
            rate_a: row[5],
            rate_b: row[6],
            rate_c: row[7],
            tap: row[8],
            shift: row[9],
            status: row[10] as i32 == 1,
        });
    }

    Ok(branches)
}

fn parse_gen_data(content: &str) -> Result<Vec<Generator>> {
    let rows = extract_matrix_data(content, "gen")?;
    let mut generators = Vec::with_capacity(rows.len());

    for row in &rows {
        if row.len() < 10 {
            return Err(OxiGridError::ParseError(format!(
                "Gen data row has {} columns, expected at least 10",
                row.len()
            )));
        }

        generators.push(Generator {
            bus_id: row[0] as usize,
            pg: row[1],
            qg: row[2],
            qmax: row[3],
            qmin: row[4],
            vg: row[5],
            mbase: row[6],
            status: row[7] as i32 > 0,
            pmax: row[8],
            pmin: row[9],
        });
    }

    Ok(generators)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_base_mva() {
        let content = "mpc.baseMVA = 100;\n";
        assert_eq!(parse_base_mva(content).unwrap(), 100.0);
    }

    #[test]
    fn test_parse_row() {
        let row = parse_row("1 2 3.0 4.5").unwrap();
        assert_eq!(row, vec![1.0, 2.0, 3.0, 4.5]);
    }

    const MINIMAL_MPC: &str = "\
function mpc = test_case
mpc.baseMVA = 100;
mpc.bus = [
1  3  0  0  0  0  1  1.0  0  138  1  1.1  0.9;
2  1  50  20  0  0  1  1.0  0  138  1  1.1  0.9;
];
mpc.gen = [
1  100  0  300  -300  1.0  100  1  200  0;
];
mpc.branch = [
1  2  0.01  0.1  0.02  250  250  250  0  0  1;
];
";

    #[test]
    fn test_parse_matpower_string_two_bus() {
        let network = parse_matpower_string(MINIMAL_MPC)
            .expect("parse_matpower_string should succeed for minimal valid input");
        assert_eq!(network.buses.len(), 2, "expected 2 buses");
        assert_eq!(network.branches.len(), 1, "expected 1 branch");
        assert!(
            (network.base_mva - 100.0).abs() < 1e-9,
            "expected base_mva == 100.0"
        );
    }

    #[test]
    fn test_parse_matpower_bus_types() {
        let network = parse_matpower_string(MINIMAL_MPC)
            .expect("parse_matpower_string should succeed for bus-type check");
        assert_eq!(
            network.buses[0].bus_type,
            BusType::Slack,
            "bus 1 should be Slack (type code 3)"
        );
        assert_eq!(
            network.buses[1].bus_type,
            BusType::PQ,
            "bus 2 should be PQ (type code 1)"
        );
    }

    #[test]
    fn test_parse_matpower_branch_impedance() {
        let network = parse_matpower_string(MINIMAL_MPC)
            .expect("parse_matpower_string should succeed for branch-impedance check");
        let branch = &network.branches[0];
        assert!(
            (branch.r - 0.01).abs() < 1e-9,
            "branch r should be 0.01, got {}",
            branch.r
        );
        assert!(
            (branch.x - 0.1).abs() < 1e-9,
            "branch x should be 0.1, got {}",
            branch.x
        );
    }

    #[test]
    fn test_parse_base_mva_missing() {
        let result = parse_base_mva("no_mva_here");
        assert!(result.is_err(), "expected Err when baseMVA is absent");
    }

    #[test]
    fn test_extract_matrix_data_single_row() {
        let content = "\
mpc.bus = [
1  3  0  0  0  0  1  1.0  0  138  1  1.1  0.9;
];
";
        let rows =
            extract_matrix_data(content, "bus").expect("extract_matrix_data should return one row");
        assert_eq!(rows.len(), 1, "expected exactly 1 row");
        assert_eq!(rows[0].len(), 13, "expected 13 columns in bus row");
    }

    #[test]
    fn test_parse_row_empty() {
        let result = parse_row("  ");
        assert!(
            result.is_none(),
            "parse_row of whitespace-only string should return None"
        );
    }

    #[test]
    fn test_parse_matpower_generator_status() {
        let network = parse_matpower_string(MINIMAL_MPC)
            .expect("parse_matpower_string should succeed for generator check");
        let gen = &network.generators[0];
        assert!(
            gen.status,
            "generator status should be true (status column = 1)"
        );
        assert!(
            (gen.pmax - 200.0).abs() < 1e-9,
            "generator pmax should be 200.0, got {}",
            gen.pmax
        );
    }

    #[test]
    fn test_branch_active_and_inactive_status() {
        let content = "\
mpc.baseMVA = 100;
mpc.bus = [
1 3 0 0 0 0 1 1.0 0 100 1 1.05 0.95
2 1 20 10 0 0 1 1.0 0 100 1 1.05 0.95
3 1 30 15 0 0 1 1.0 0 100 1 1.05 0.95
];
mpc.branch = [
1 2 0.01 0.05 0.02 100 100 100 0 0 1
2 3 0.02 0.08 0.01 100 100 100 0 0 0
];
mpc.gen = [
1 80 0 50 -50 1.02 100 1 200 0
];
";
        let network = parse_matpower_string(content)
            .expect("3-bus string with active and inactive branch should parse");
        assert_eq!(network.branches.len(), 2, "expected 2 branches");
        assert!(
            network.branches[0].status,
            "first branch (status=1) should be active"
        );
        assert!(
            !network.branches[1].status,
            "second branch (status=0) should be inactive"
        );
    }

    #[test]
    fn test_bus_row_too_few_columns_returns_error() {
        let content = "\
mpc.baseMVA = 100;
mpc.bus = [
1 3 0 0 0
];
";
        let result = parse_matpower_string(content);
        assert!(
            result.is_err(),
            "bus row with fewer than 13 columns should return Err"
        );
    }

    #[test]
    fn test_generator_active_and_inactive_status() {
        let content = "\
mpc.baseMVA = 100;
mpc.bus = [
1 3 0 0 0 0 1 1.0 0 100 1 1.05 0.95
2 1 50 30 0 0 1 1.0 0 100 1 1.05 0.95
];
mpc.branch = [
1 2 0.01 0.05 0.02 100 100 100 0 0 1
];
mpc.gen = [
1 80 0 50 -50 1.02 100 1 200 0
2 40 0 30 -30 1.01 100 0 100 0
];
";
        let network = parse_matpower_string(content)
            .expect("string with active and inactive generator should parse");
        assert_eq!(network.generators.len(), 2, "expected 2 generators");
        assert!(
            network.generators[0].status,
            "first generator (status=1) should be active"
        );
        assert!(
            !network.generators[1].status,
            "second generator (status=0) should be inactive"
        );
    }

    #[test]
    fn test_multi_bus_parse_counts() {
        let content = "\
mpc.baseMVA = 100;
mpc.bus = [
1 3 0 0 0 0 1 1.0 0 100 1 1.05 0.95
2 1 20 10 0 0 1 1.0 0 100 1 1.05 0.95
3 1 30 15 0 0 1 1.0 0 100 1 1.05 0.95
4 1 40 20 0 0 1 1.0 0 100 1 1.05 0.95
];
mpc.branch = [
1 2 0.01 0.05 0.02 100 100 100 0 0 1
2 3 0.02 0.08 0.01 100 100 100 0 0 1
3 4 0.03 0.10 0.02 100 100 100 0 0 1
];
mpc.gen = [
1 80 0 50 -50 1.02 100 1 200 0
2 40 0 30 -30 1.01 100 1 100 0
];
";
        let network = parse_matpower_string(content).expect("4-bus matpower string should parse");
        assert_eq!(network.buses.len(), 4, "expected 4 buses");
        assert_eq!(network.branches.len(), 3, "expected 3 branches");
        assert_eq!(network.generators.len(), 2, "expected 2 generators");
    }
}
