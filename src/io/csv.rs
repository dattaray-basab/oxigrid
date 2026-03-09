/// CSV import/export for time-series data.
///
/// Provides lightweight CSV reading/writing without external dependencies.
/// Supports:
/// - Time-series export: Vec<(f64, f64)> → "time,value\n..."
/// - Multi-column export: headers + rows of f64
/// - Simple CSV parsing back into Vec<Vec<f64>>
///
/// For production use, the `csv` crate or `polars` should be preferred.
use crate::error::{OxiGridError, Result};
use std::fmt::Write as FmtWrite;

/// A named time series column.
#[derive(Debug, Clone)]
pub struct CsvColumn {
    pub name: String,
    pub values: Vec<f64>,
}

impl CsvColumn {
    pub fn new(name: impl Into<String>, values: Vec<f64>) -> Self {
        Self {
            name: name.into(),
            values,
        }
    }
}

/// Write a single time-series to a CSV string.
///
/// Output: "time_s,<name>\n{t},{v}\n..."
pub fn time_series_to_csv(name: &str, times: &[f64], values: &[f64]) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "time_s,{name}");
    for (&t, &v) in times.iter().zip(values.iter()) {
        let _ = writeln!(out, "{t:.6},{v:.6}");
    }
    out
}

/// Write multiple columns to a CSV string.
///
/// All columns must have the same length. The first column is assumed to be
/// a time axis if `time_col` is Some.
pub fn columns_to_csv(time_col: Option<(&str, &[f64])>, columns: &[CsvColumn]) -> String {
    let n = columns.first().map(|c| c.values.len()).unwrap_or(0);
    let mut out = String::new();

    // Header
    if let Some((time_name, _)) = time_col {
        let _ = write!(out, "{time_name}");
        if !columns.is_empty() {
            let _ = write!(out, ",");
        }
    }
    let header_cols: Vec<&str> = columns.iter().map(|c| c.name.as_str()).collect();
    let _ = writeln!(out, "{}", header_cols.join(","));

    // Rows
    for i in 0..n {
        if let Some((_, times)) = time_col {
            if i < times.len() {
                let _ = write!(out, "{:.6}", times[i]);
                if !columns.is_empty() {
                    let _ = write!(out, ",");
                }
            }
        }
        let row: Vec<String> = columns
            .iter()
            .map(|c| {
                if i < c.values.len() {
                    format!("{:.6}", c.values[i])
                } else {
                    String::new()
                }
            })
            .collect();
        let _ = writeln!(out, "{}", row.join(","));
    }
    out
}

/// Write CSV to a file.
pub fn write_csv_file(path: &str, content: &str) -> Result<()> {
    std::fs::write(path, content)
        .map_err(|e| OxiGridError::ParseError(format!("Failed to write {path}: {e}")))
}

/// Parse a CSV string into rows of f64.
///
/// Skips the header row. Blank cells are treated as 0.0.
pub fn parse_csv_f64(content: &str) -> Result<Vec<Vec<f64>>> {
    let mut rows = Vec::new();
    let mut lines = content.lines();
    lines.next(); // skip header

    for (line_no, line) in lines.enumerate() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let row: std::result::Result<Vec<f64>, _> = line
            .split(',')
            .map(|cell| {
                let c = cell.trim();
                if c.is_empty() {
                    Ok(0.0)
                } else {
                    c.parse::<f64>()
                }
            })
            .collect();
        row.map(|r| rows.push(r)).map_err(|_| {
            OxiGridError::ParseError(format!("CSV parse error at line {}", line_no + 2))
        })?;
    }
    Ok(rows)
}

/// Read a CSV file into rows of f64.
pub fn read_csv_file(path: &str) -> Result<Vec<Vec<f64>>> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| OxiGridError::ParseError(format!("Failed to read {path}: {e}")))?;
    parse_csv_f64(&content)
}

/// Parse a CSV string and return the column at the given 0-indexed position.
pub fn parse_csv_column(content: &str, col: usize) -> Result<Vec<f64>> {
    let rows = parse_csv_f64(content)?;
    rows.iter()
        .map(|row| {
            row.get(col)
                .copied()
                .ok_or_else(|| OxiGridError::ParseError(format!("Column {col} out of range")))
        })
        .collect()
}

/// Compute basic statistics for a slice.
pub fn csv_stats(values: &[f64]) -> CsvStats {
    if values.is_empty() {
        return CsvStats {
            min: 0.0,
            max: 0.0,
            mean: 0.0,
            std: 0.0,
            count: 0,
        };
    }
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / n;
    CsvStats {
        min: values.iter().cloned().fold(f64::INFINITY, f64::min),
        max: values.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
        mean,
        std: variance.sqrt(),
        count: values.len(),
    }
}

/// Basic descriptive statistics.
#[derive(Debug, Clone)]
pub struct CsvStats {
    pub min: f64,
    pub max: f64,
    pub mean: f64,
    pub std: f64,
    pub count: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_time_series_to_csv_header() {
        let times = vec![0.0, 1.0, 2.0];
        let values = vec![10.0, 20.0, 30.0];
        let csv = time_series_to_csv("power_kw", &times, &values);
        assert!(csv.starts_with("time_s,power_kw\n"));
        assert!(csv.contains("1.000000,20.000000"));
    }

    #[test]
    fn test_columns_to_csv_roundtrip() {
        let cols = vec![
            CsvColumn::new("voltage_pu", vec![1.0, 0.99, 0.98]),
            CsvColumn::new("current_a", vec![100.0, 110.0, 120.0]),
        ];
        let csv = columns_to_csv(Some(("time_s", &[0.0, 1.0, 2.0])), &cols);
        assert!(csv.contains("voltage_pu,current_a"));
        assert!(csv.contains("1.000000,100.000000"));
    }

    #[test]
    fn test_parse_csv_f64_basic() {
        let csv = "time,value\n0.0,1.5\n1.0,2.5\n2.0,3.5\n";
        let rows = parse_csv_f64(csv).unwrap();
        assert_eq!(rows.len(), 3);
        assert!((rows[0][1] - 1.5).abs() < 1e-10);
        assert!((rows[2][0] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_parse_csv_column() {
        let csv = "a,b,c\n1,2,3\n4,5,6\n";
        let col1 = parse_csv_column(csv, 1).unwrap();
        assert_eq!(col1, vec![2.0, 5.0]);
    }

    #[test]
    fn test_csv_stats() {
        let v = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let stats = csv_stats(&v);
        assert!((stats.mean - 3.0).abs() < 1e-10);
        assert!((stats.min - 1.0).abs() < 1e-10);
        assert!((stats.max - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_csv_stats_empty() {
        let stats = csv_stats(&[]);
        assert_eq!(stats.count, 0);
    }

    #[test]
    fn test_parse_csv_empty_cells() {
        let csv = "a,b\n1.0,\n,2.0\n";
        let rows = parse_csv_f64(csv).unwrap();
        assert_eq!(rows.len(), 2);
        assert!((rows[0][0] - 1.0).abs() < 1e-10);
        assert!((rows[0][1] - 0.0).abs() < 1e-10);
    }
}
