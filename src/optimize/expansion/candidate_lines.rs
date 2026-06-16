//! Candidate transmission line database.
//!
//! Provides typical per-km electrical parameters and costs for standard
//! transmission voltage classes (115 kV – 765 kV).  Used to build
//! [`InvestmentCandidate`] instances for TEP studies without having to
//! look up manufacturer data manually.

use crate::error::{OxiGridError, Result};
use crate::optimize::expansion::robust_tep::InvestmentCandidate;

/// Per-km parameters for one transmission voltage class.
#[derive(Debug, Clone)]
pub struct CandidateLineEntry {
    /// Nominal voltage \[kV\]
    pub voltage_kv: f64,
    /// Human-readable conductor designation (e.g. "ACSR Bluejay")
    pub conductor_type: String,
    /// Thermal rating per circuit \[MW\]
    pub capacity_mw: f64,
    /// Series resistance \[Ω/km\]
    pub resistance_ohm_per_km: f64,
    /// Series reactance \[Ω/km\]
    pub reactance_ohm_per_km: f64,
    /// Shunt susceptance \[μS/km\]
    pub susceptance_us_per_km: f64,
    /// Total installed cost per circuit-km \[M$/km\]
    pub cost_m_per_km: f64,
}

/// Database of standard transmission line types.
pub struct CandidateLineDatabase {
    pub entries: Vec<CandidateLineEntry>,
}

impl CandidateLineDatabase {
    /// Populate with representative data for five voltage classes.
    ///
    /// Values are typical North-American overhead line parameters
    /// (ACSR conductors, flat horizontal configuration).
    pub fn standard() -> Self {
        Self {
            entries: vec![
                CandidateLineEntry {
                    voltage_kv: 115.0,
                    conductor_type: "ACSR Hawk".into(),
                    capacity_mw: 150.0,
                    resistance_ohm_per_km: 0.050,
                    reactance_ohm_per_km: 0.400,
                    susceptance_us_per_km: 2.8,
                    cost_m_per_km: 0.50,
                },
                CandidateLineEntry {
                    voltage_kv: 230.0,
                    conductor_type: "ACSR Bluejay".into(),
                    capacity_mw: 500.0,
                    resistance_ohm_per_km: 0.030,
                    reactance_ohm_per_km: 0.350,
                    susceptance_us_per_km: 3.2,
                    cost_m_per_km: 1.00,
                },
                CandidateLineEntry {
                    voltage_kv: 345.0,
                    conductor_type: "ACSR Cardinal 2x".into(),
                    capacity_mw: 900.0,
                    resistance_ohm_per_km: 0.020,
                    reactance_ohm_per_km: 0.320,
                    susceptance_us_per_km: 3.8,
                    cost_m_per_km: 1.80,
                },
                CandidateLineEntry {
                    voltage_kv: 500.0,
                    conductor_type: "ACSR Lapwing 3x".into(),
                    capacity_mw: 2000.0,
                    resistance_ohm_per_km: 0.015,
                    reactance_ohm_per_km: 0.300,
                    susceptance_us_per_km: 4.5,
                    cost_m_per_km: 2.50,
                },
                CandidateLineEntry {
                    voltage_kv: 765.0,
                    conductor_type: "ACSR Bersfort 4x".into(),
                    capacity_mw: 3000.0,
                    resistance_ohm_per_km: 0.010,
                    reactance_ohm_per_km: 0.280,
                    susceptance_us_per_km: 5.2,
                    cost_m_per_km: 4.00,
                },
            ],
        }
    }

    /// Find the entry whose voltage class is closest to `voltage_kv`.
    pub fn find_by_voltage(&self, voltage_kv: f64) -> Option<&CandidateLineEntry> {
        self.entries.iter().min_by(|a, b| {
            let da = (a.voltage_kv - voltage_kv).abs();
            let db = (b.voltage_kv - voltage_kv).abs();
            da.partial_cmp(&db).unwrap_or(core::cmp::Ordering::Equal)
        })
    }

    /// Build an [`InvestmentCandidate`] for a specific corridor.
    ///
    /// Impedance is converted from Ω to per-unit using the voltage base
    /// derived from the line's nominal voltage and a 100 MVA system base.
    ///
    /// # Arguments
    /// * `from_bus`   – sending-end bus ID
    /// * `to_bus`     – receiving-end bus ID
    /// * `voltage_kv` – nominal voltage (used to look up parameters)
    /// * `length_km`  – circuit length
    /// * `id`         – candidate identifier (unique within the TEP study)
    pub fn create_candidate(
        &self,
        from_bus: usize,
        to_bus: usize,
        voltage_kv: f64,
        length_km: f64,
        id: usize,
    ) -> Result<InvestmentCandidate> {
        let entry = self.find_by_voltage(voltage_kv).ok_or_else(|| {
            OxiGridError::InvalidParameter("CandidateLineDatabase is empty".into())
        })?;

        if length_km <= 0.0 {
            return Err(OxiGridError::InvalidParameter(format!(
                "length_km must be positive, got {length_km}"
            )));
        }

        // Base impedance: Z_base = V_kV² / S_MVA (100 MVA system base)
        let z_base = entry.voltage_kv * entry.voltage_kv / 100.0;

        let r_pu = entry.resistance_ohm_per_km * length_km / z_base;
        let x_pu = entry.reactance_ohm_per_km * length_km / z_base;

        let total_cost = entry.cost_m_per_km * length_km;
        // Typical fixed O&M: ~1 % of investment per year
        let annual_fixed = total_cost * 0.01;

        Ok(InvestmentCandidate {
            id,
            from_bus,
            to_bus,
            capacity_mw: entry.capacity_mw,
            investment_cost_m: total_cost,
            annual_fixed_cost_m: annual_fixed,
            resistance_pu: r_pu,
            reactance_pu: x_pu,
            n_parallel_max: 2,
            can_expand_existing: false,
            lead_time_years: 3.0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_candidate_database_lookup() {
        let db = CandidateLineDatabase::standard();
        let entry = db.find_by_voltage(345.0).expect("345 kV entry");
        // Typical 345 kV line: ~900 MW rating
        assert!(
            entry.capacity_mw >= 700.0 && entry.capacity_mw <= 1200.0,
            "capacity_mw = {}",
            entry.capacity_mw
        );
        assert!((entry.voltage_kv - 345.0).abs() < 1.0);
    }

    #[test]
    fn test_candidate_creation() {
        let db = CandidateLineDatabase::standard();
        let cand = db
            .create_candidate(1, 2, 500.0, 100.0, 0)
            .expect("500 kV 100 km");
        // 500 kV, 100 km: capacity ~2000 MW
        assert!(
            cand.capacity_mw >= 1500.0,
            "capacity_mw = {}",
            cand.capacity_mw
        );
        // Cost: 2.50 M$/km × 100 km = 250 M$
        assert!(
            (cand.investment_cost_m - 250.0).abs() < 1.0,
            "investment_cost_m = {}",
            cand.investment_cost_m
        );
        // Reactance > 0
        assert!(cand.reactance_pu > 0.0);
        // R in pu: 0.015 Ω/km * 100 km / (500²/100) = 1.5 / 2500 = 0.0006
        let z_base = 500.0_f64 * 500.0 / 100.0;
        let expected_r = 0.015 * 100.0 / z_base;
        assert!((cand.resistance_pu - expected_r).abs() < 1e-6);
    }

    #[test]
    fn test_candidate_creation_zero_length_fails() {
        let db = CandidateLineDatabase::standard();
        assert!(db.create_candidate(1, 2, 230.0, 0.0, 0).is_err());
    }

    #[test]
    fn test_standard_database_has_five_entries() {
        let db = CandidateLineDatabase::standard();
        assert_eq!(db.entries.len(), 5);
    }

    #[test]
    fn test_find_by_voltage_closest() {
        let db = CandidateLineDatabase::standard();
        // 400 kV is between 345 and 500; closer to 345
        let e = db.find_by_voltage(400.0).expect("entry");
        assert!((e.voltage_kv - 345.0).abs() < 1.0 || (e.voltage_kv - 500.0).abs() < 1.0);
    }

    #[test]
    fn find_by_voltage_115_kv() {
        let db = CandidateLineDatabase::standard();
        let entry = db.find_by_voltage(115.0).expect("115 kV entry must exist");
        assert!(
            (entry.voltage_kv - 115.0).abs() < 1.0,
            "expected voltage_kv ≈ 115, got {}",
            entry.voltage_kv
        );
        assert!(
            (entry.capacity_mw - 150.0).abs() < 1.0,
            "expected capacity_mw ≈ 150, got {}",
            entry.capacity_mw
        );
    }

    #[test]
    fn find_by_voltage_765_kv() {
        let db = CandidateLineDatabase::standard();
        let entry = db.find_by_voltage(765.0).expect("765 kV entry must exist");
        // 765 kV should be the highest voltage in the database
        let max_voltage = db
            .entries
            .iter()
            .map(|e| e.voltage_kv)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (entry.voltage_kv - max_voltage).abs() < 1e-9,
            "expected highest voltage entry, got {}",
            entry.voltage_kv
        );
    }

    #[test]
    fn create_candidate_115kv_50km_resistance_pu() {
        let db = CandidateLineDatabase::standard();
        let entry_115 = db.find_by_voltage(115.0).expect("115 kV entry");
        let cand = db
            .create_candidate(0, 1, 115.0, 50.0, 0)
            .expect("create 115 kV 50 km candidate");
        let z_base = 115.0_f64.powi(2) / 100.0;
        let expected_r = entry_115.resistance_ohm_per_km * 50.0 / z_base;
        assert!(
            (cand.resistance_pu - expected_r).abs() < 1e-9,
            "resistance_pu mismatch: got {}, expected {}",
            cand.resistance_pu,
            expected_r
        );
    }

    #[test]
    fn create_candidate_765kv_capacity() {
        let db = CandidateLineDatabase::standard();
        let cand = db
            .create_candidate(0, 1, 765.0, 200.0, 0)
            .expect("create 765 kV 200 km candidate");
        assert!(
            cand.capacity_mw >= 3000.0,
            "expected capacity_mw >= 3000, got {}",
            cand.capacity_mw
        );
    }

    #[test]
    fn annual_fixed_cost_is_one_percent_of_investment() {
        let db = CandidateLineDatabase::standard();
        let cand = db
            .create_candidate(0, 1, 345.0, 100.0, 0)
            .expect("create 345 kV 100 km candidate");
        let expected = 0.01 * cand.investment_cost_m;
        assert!(
            (cand.annual_fixed_cost_m - expected).abs() < 1e-9,
            "annual_fixed_cost_m {} != 1% of investment_cost_m {}",
            cand.annual_fixed_cost_m,
            cand.investment_cost_m
        );
    }

    #[test]
    fn create_candidate_id_matches() {
        let db = CandidateLineDatabase::standard();
        let cand = db
            .create_candidate(0, 1, 345.0, 100.0, 42)
            .expect("create candidate with id=42");
        assert_eq!(cand.id, 42, "expected id=42, got {}", cand.id);
    }

    #[test]
    fn create_candidate_bus_indices_match() {
        let db = CandidateLineDatabase::standard();
        let cand = db
            .create_candidate(5, 9, 500.0, 80.0, 1)
            .expect("create 500 kV 80 km candidate from 5 to 9");
        assert_eq!(
            cand.from_bus, 5,
            "expected from_bus=5, got {}",
            cand.from_bus
        );
        assert_eq!(cand.to_bus, 9, "expected to_bus=9, got {}", cand.to_bus);
    }
}
