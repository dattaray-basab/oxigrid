/// IEEE Common Data Format (CDF) parser.
///
/// Parses the fixed-column IEEE CDF text format used by classic power systems
/// test cases (IEEE 14, 30, 57, 118, 300-bus systems from the original data files).
///
/// # Format specification
/// The IEEE CDF format uses fixed-column widths:
/// - Title card: columns 2-9 (date), 31-37 (MVA base), 39-42 (year)
/// - Bus data card: 16 columns of fixed-width fields
/// - Branch data card: 21 columns of fixed-width fields
///
/// # Reference
/// IEEE Committee Report, "Common Format for Exchange of Solved Load Flow Data",
/// IEEE Trans. Power Apparatus & Systems, PAS-92(6), 1973.
use crate::error::{OxiGridError, Result};
use crate::network::branch::Branch;
use crate::network::bus::{Bus, BusType};
use crate::network::topology::{Generator, PowerNetwork};
use crate::units::{Power, ReactivePower, Voltage};

/// Parse an IEEE CDF file and return a PowerNetwork.
pub fn parse_ieee_cdf_file(path: &str) -> Result<PowerNetwork> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| OxiGridError::ParseError(format!("Failed to read {path}: {e}")))?;
    parse_ieee_cdf_string(&content)
}

/// Parse IEEE CDF format from a string.
pub fn parse_ieee_cdf_string(content: &str) -> Result<PowerNetwork> {
    let mut lines = content.lines().peekable();

    // Title card: MVA base in columns 31-37 (1-indexed)
    let title = lines
        .next()
        .ok_or_else(|| OxiGridError::ParseError("Empty file".into()))?;
    let base_mva = parse_base_mva_cdf(title)?;

    let mut buses: Vec<Bus> = Vec::new();
    let mut branches: Vec<Branch> = Vec::new();
    let mut generators: Vec<Generator> = Vec::new();

    let mut section = CdfSection::None;

    for line in lines {
        // Section markers
        if line.starts_with("BUS DATA FOLLOWS") {
            section = CdfSection::Bus;
            continue;
        }
        if line.starts_with("BRANCH DATA FOLLOWS") {
            section = CdfSection::Branch;
            continue;
        }
        if line.starts_with("LOSS ZONES") || line.starts_with("INTERCHANGE DATA") {
            section = CdfSection::Skip;
            continue;
        }
        if line.starts_with("TIE LINES") || line.starts_with("END OF DATA") {
            break;
        }
        // Skip separator lines (-999... pattern)
        if line.trim_start().starts_with("-999") || line.trim().is_empty() {
            section = CdfSection::None;
            continue;
        }

        match section {
            CdfSection::Bus => {
                if let Some(b) = parse_cdf_bus(line)? {
                    // Create generator record for PV/Slack buses with generation
                    let pg = parse_f64_col(line, 59, 67).unwrap_or(0.0);
                    let qg = parse_f64_col(line, 67, 75).unwrap_or(0.0);
                    let qmax = parse_f64_col(line, 90, 98).unwrap_or(9999.0);
                    let qmin = parse_f64_col(line, 98, 106).unwrap_or(-9999.0);
                    if pg != 0.0 || b.bus_type != BusType::PQ {
                        generators.push(Generator {
                            bus_id: b.id,
                            pg,
                            qg,
                            qmax,
                            qmin,
                            vg: b.vm,
                            mbase: base_mva,
                            status: true,
                            pmax: pg.max(0.0) * 2.0 + 100.0,
                            pmin: 0.0,
                        });
                    }
                    buses.push(b);
                }
            }
            CdfSection::Branch => {
                if let Some(br) = parse_cdf_branch(line)? {
                    branches.push(br);
                }
            }
            CdfSection::Skip | CdfSection::None => {}
        }
    }

    if buses.is_empty() {
        return Err(OxiGridError::ParseError(
            "No bus data found in IEEE CDF file".into(),
        ));
    }

    let mut network = PowerNetwork::new(base_mva);
    network.buses = buses;
    network.branches = branches;
    network.generators = generators;
    network.validate()?;
    Ok(network)
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum CdfSection {
    None,
    Bus,
    Branch,
    Skip,
}

/// Parse base MVA from the CDF title card (columns 31-37, 0-indexed 30-36).
fn parse_base_mva_cdf(title: &str) -> Result<f64> {
    // MVA base is in columns 31-37 (1-indexed) = chars 30..37
    let chars: Vec<char> = title.chars().collect();
    let end = chars.len().min(37);
    let start = end.min(30);
    if start >= end {
        // Try parsing from anywhere in the title if short line
        for word in title.split_whitespace() {
            if let Ok(v) = word.parse::<f64>() {
                if v > 0.0 {
                    return Ok(v);
                }
            }
        }
        return Ok(100.0); // default
    }
    let mva_str: String = chars[start..end].iter().collect();
    mva_str
        .trim()
        .parse::<f64>()
        .map_err(|_| OxiGridError::ParseError(format!("Cannot parse MVA base from: '{mva_str}'")))
}

/// Parse a CDF bus data card (fixed-column format).
///
/// Field layout (1-indexed columns):
///  1-4   Bus number
///  6-17  Bus name
///  19    Load flow area
///  20-23 Loss zone
///  25    Type (0=PQ, 1=PQ_gen, 2=PV, 3=Slack)
///  27-33 Final voltage (p.u.)
///  34-40 Final angle (degrees)
///  41-49 Load MW
///  50-58 Load MVAr
///  59-67 Generation MW
///  68-75 Generation MVAr
///  77-83 Base kV
///  ...
fn parse_cdf_bus(line: &str) -> Result<Option<Bus>> {
    let line = line.trim_end();
    if line.len() < 25 {
        return Ok(None);
    }

    let bus_id = parse_i32_col(line, 0, 4)? as usize;
    let name = if line.len() >= 17 {
        line[5..17.min(line.len())].trim().to_string()
    } else {
        format!("Bus {bus_id}")
    };

    let bus_type_code = parse_i32_col(line, 24, 26).unwrap_or(0);
    let bus_type = match bus_type_code {
        3 => BusType::Slack,
        2 => BusType::PV,
        _ => BusType::PQ,
    };

    let vm = parse_f64_col(line, 27, 33).unwrap_or(1.0);
    let va_deg = parse_f64_col(line, 33, 40).unwrap_or(0.0);
    let pd = parse_f64_col(line, 40, 49).unwrap_or(0.0);
    let qd = parse_f64_col(line, 49, 58).unwrap_or(0.0);
    let base_kv = parse_f64_col(line, 76, 83).unwrap_or(0.0);
    let gs = parse_f64_col(line, 106, 114).unwrap_or(0.0);
    let bs = parse_f64_col(line, 114, 122).unwrap_or(0.0);

    let bus = Bus {
        id: bus_id,
        name,
        bus_type,
        base_kv: Voltage(base_kv),
        vm: vm.max(0.5),
        va: va_deg.to_radians(),
        pd: Power(pd),
        qd: ReactivePower(qd),
        gs,
        bs,
        zone: None,
    };
    Ok(Some(bus))
}

/// Parse a CDF branch data card.
///
/// Field layout (1-indexed):
///  1-4   Tap bus number
///  6-9   Z bus number
///  ...   resistance, reactance, line charging, tap ratio, angle
fn parse_cdf_branch(line: &str) -> Result<Option<Branch>> {
    let line = line.trim_end();
    if line.len() < 40 {
        return Ok(None);
    }

    let from_bus = parse_i32_col(line, 0, 4)? as usize;
    let to_bus = parse_i32_col(line, 5, 9)? as usize;

    if from_bus == 0 || to_bus == 0 {
        return Ok(None);
    }

    let r = parse_f64_col(line, 19, 29).unwrap_or(0.0);
    let x = parse_f64_col(line, 29, 40).unwrap_or(0.001);
    let b = parse_f64_col(line, 40, 50).unwrap_or(0.0);
    let rate_a = parse_f64_col(line, 50, 56).unwrap_or(0.0);
    let rate_b = parse_f64_col(line, 56, 62).unwrap_or(0.0);
    let rate_c = parse_f64_col(line, 62, 68).unwrap_or(0.0);
    let tap = parse_f64_col(line, 76, 83).unwrap_or(0.0);
    let shift = parse_f64_col(line, 83, 90).unwrap_or(0.0);
    let status_code = parse_i32_col(line, 18, 19).unwrap_or(1);

    Ok(Some(Branch {
        from_bus,
        to_bus,
        r,
        x: if x.abs() < 1e-8 { 0.001 } else { x },
        b,
        rate_a,
        rate_b,
        rate_c,
        tap,
        shift,
        status: status_code != 0,
    }))
}

/// Extract a substring by 0-indexed column range and parse as f64.
fn parse_f64_col(line: &str, start: usize, end: usize) -> Option<f64> {
    let bytes = line.as_bytes();
    let end = end.min(bytes.len());
    if start >= end {
        return None;
    }
    // Safety: we only work with ASCII
    let s = std::str::from_utf8(&bytes[start..end]).ok()?.trim();
    if s.is_empty() {
        None
    } else {
        s.parse::<f64>().ok()
    }
}

/// Extract a substring by 0-indexed column range and parse as i32.
fn parse_i32_col(line: &str, start: usize, end: usize) -> Result<i32> {
    let bytes = line.as_bytes();
    let end = end.min(bytes.len());
    if start >= end {
        return Ok(0);
    }
    let s = std::str::from_utf8(&bytes[start..end])
        .map_err(|_| OxiGridError::ParseError("UTF-8 error".into()))?
        .trim();
    if s.is_empty() {
        return Ok(0);
    }
    s.parse::<i32>()
        .map_err(|_| OxiGridError::ParseError(format!("Cannot parse int from '{s}'")))
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_CDF: &str = "\
 08/19/93 UW ARCHIVE           100.0  1962 W IEEE 14 Bus Test Case
BUS DATA FOLLOWS                            G  B
   1 Bus 1     HV  1  1  3 1.060  0.000  0.000  0.000  232.4  -16.9   132.0  1  1.060    0.000    0.000    0.000
   2 Bus 2     HV  1  1  2 1.045 -4.986  21.70   12.70  40.0   42.4   132.0  1  1.045    0.000  300.000 -300.000
   3 Bus 3     HV  1  1  0 1.010-12.725  94.20   19.00   0.0    0.0   132.0  1  0.000    0.000    0.000    0.000
-999
BRANCH DATA FOLLOWS                         R          X          B    CONT  NAME  RATE1  RATE2  RATE3
   1   2  1  1 1  1 .01938    .05917     .05280
   1   3  1  1 1  1 .05403    .22304     .04920
-999
END OF DATA
";

    #[test]
    fn test_parse_cdf_sample() {
        let net = parse_ieee_cdf_string(SAMPLE_CDF).unwrap();
        assert_eq!(net.base_mva, 100.0);
        assert_eq!(net.bus_count(), 3);
        assert_eq!(net.branch_count(), 2);
    }

    #[test]
    fn test_parse_cdf_bus_types() {
        let net = parse_ieee_cdf_string(SAMPLE_CDF).unwrap();
        let b1 = &net.buses[0];
        let b2 = &net.buses[1];
        let b3 = &net.buses[2];
        assert_eq!(b1.bus_type, BusType::Slack, "Bus 1 should be Slack");
        assert_eq!(b2.bus_type, BusType::PV, "Bus 2 should be PV");
        assert_eq!(b3.bus_type, BusType::PQ, "Bus 3 should be PQ");
    }

    #[test]
    fn test_parse_cdf_branch_impedance() {
        let net = parse_ieee_cdf_string(SAMPLE_CDF).unwrap();
        let br = &net.branches[0]; // bus 1→2
        assert!(br.r > 0.0, "Branch resistance should be positive");
        assert!(br.x > 0.0, "Branch reactance should be positive");
    }

    #[test]
    fn test_parse_f64_col() {
        let line = "   1.060  0.000  ";
        let v = parse_f64_col(line, 3, 8);
        assert!(v.is_some());
        assert!((v.unwrap() - 1.060).abs() < 1e-4);
    }

    #[test]
    fn test_base_mva_parse() {
        let title = " 08/19/93 UW ARCHIVE           100.0  1962";
        let mva = parse_base_mva_cdf(title).unwrap();
        assert!((mva - 100.0).abs() < 1e-6);
    }
}
