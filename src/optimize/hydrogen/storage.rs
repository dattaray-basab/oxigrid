//! Hydrogen storage tank models.
//!
//! Supports compressed gas (700 bar, 350 bar), liquid H2, metal hydride, and underground storage.
//! Tracks SoC, enforces charge/discharge rate limits, computes boil-off (for liquid),
//! and estimates compression energy requirements.

/// Hydrogen storage technology type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HydrogenStorageType {
    /// High-pressure compressed gas at 700 bar (typical for vehicle fuelling)
    CompressedGas700bar,
    /// Medium-pressure compressed gas at 350 bar (bus/fleet applications)
    CompressedGas350bar,
    /// Cryogenic liquid hydrogen (large-scale storage)
    LiquidH2,
    /// Metal hydride absorption storage (low pressure, high gravimetric density materials)
    MetalHydride,
    /// Underground bulk storage: salt caverns or depleted natural gas reservoirs
    Underground,
}

/// Hydrogen storage tank.
///
/// Tracks the current stored mass and enforces physical constraints:
/// - Charge/discharge rate limits [kg/h]
/// - Minimum state-of-charge (safety reserve)
/// - Boil-off losses (liquid H2 only)
/// - Compression energy accounting
#[derive(Debug, Clone)]
pub struct HydrogenTank {
    /// Total storage capacity \[kg\]
    pub capacity_kg: f64,
    /// Currently stored hydrogen \[kg\]
    pub current_kg: f64,
    /// Storage technology
    pub storage_type: HydrogenStorageType,
    /// Maximum fill (charge) rate [kg/h]
    pub max_charge_rate_kg_per_h: f64,
    /// Maximum withdrawal (discharge) rate [kg/h]
    pub max_discharge_rate_kg_per_h: f64,
    /// Current tank pressure \[bar\] (derived from fill level for compressed gas)
    pub pressure_bar: f64,
    /// Maximum allowable pressure \[bar\]
    pub max_pressure_bar: f64,
    /// Minimum state-of-charge as fraction (safety reserve, default 0.05)
    pub min_soc_fraction: f64,
    /// Boil-off rate [% per day] (only relevant for liquid H2, default 0.2)
    pub boil_off_rate_pct_per_day: f64,
    /// Energy required to compress H2 from atmospheric to storage pressure [kWh/kg]
    pub compression_energy_kwh_per_kg: f64,
}

impl HydrogenTank {
    /// Create a new hydrogen tank with technology-appropriate defaults.
    pub fn new(capacity_kg: f64, storage_type: HydrogenStorageType) -> Self {
        let (max_p_bar, comp_energy, boil_off) = match storage_type {
            HydrogenStorageType::CompressedGas700bar => (700.0, 2.5, 0.0),
            HydrogenStorageType::CompressedGas350bar => (350.0, 1.5, 0.0),
            HydrogenStorageType::LiquidH2 => (10.0, 9.5, 0.2), // 9.5 kWh/kg liquefaction
            HydrogenStorageType::MetalHydride => (50.0, 0.5, 0.0),
            HydrogenStorageType::Underground => (200.0, 0.8, 0.0),
        };

        // Rate limits scale with capacity (1/8 capacity turnover per hour as default)
        let rate = capacity_kg / 8.0;

        Self {
            capacity_kg,
            current_kg: 0.0,
            storage_type,
            max_charge_rate_kg_per_h: rate,
            max_discharge_rate_kg_per_h: rate,
            pressure_bar: 1.0, // starts at atmospheric (empty)
            max_pressure_bar: max_p_bar,
            min_soc_fraction: 0.05,
            boil_off_rate_pct_per_day: boil_off,
            compression_energy_kwh_per_kg: comp_energy,
        }
    }

    /// State of charge as fraction of total capacity (0.0–1.0).
    pub fn soc(&self) -> f64 {
        if self.capacity_kg < 1e-12 {
            return 0.0;
        }
        (self.current_kg / self.capacity_kg).clamp(0.0, 1.0)
    }

    /// Minimum stored mass respecting the minimum SoC constraint \[kg\].
    pub fn min_kg(&self) -> f64 {
        self.min_soc_fraction * self.capacity_kg
    }

    /// Available storage headroom above minimum SoC \[kg\].
    pub fn available_headroom_kg(&self) -> f64 {
        (self.capacity_kg - self.current_kg).max(0.0)
    }

    /// Available hydrogen above minimum SoC that can be discharged \[kg\].
    pub fn available_to_discharge_kg(&self) -> f64 {
        (self.current_kg - self.min_kg()).max(0.0)
    }

    /// Charge the tank with the requested amount of H2 over a time interval.
    ///
    /// Returns the actual amount stored \[kg\] (limited by rate and remaining capacity).
    pub fn charge(&mut self, h2_kg: f64, dt_hours: f64) -> f64 {
        if h2_kg <= 0.0 || dt_hours <= 0.0 {
            return 0.0;
        }
        // Rate limit
        let rate_limited = (h2_kg / dt_hours).min(self.max_charge_rate_kg_per_h) * dt_hours;
        // Capacity limit
        let room = self.available_headroom_kg();
        let actual = rate_limited.min(room).max(0.0);
        self.current_kg += actual;
        self.current_kg = self.current_kg.clamp(0.0, self.capacity_kg);
        self.pressure_bar = self.pressure_from_fill();
        actual
    }

    /// Discharge the requested amount of H2 from the tank over a time interval.
    ///
    /// Returns the actual amount discharged \[kg\] (limited by rate and minimum SoC).
    pub fn discharge(&mut self, h2_kg: f64, dt_hours: f64) -> f64 {
        if h2_kg <= 0.0 || dt_hours <= 0.0 {
            return 0.0;
        }
        // Rate limit
        let rate_limited = (h2_kg / dt_hours).min(self.max_discharge_rate_kg_per_h) * dt_hours;
        // Minimum SoC limit
        let available = self.available_to_discharge_kg();
        let actual = rate_limited.min(available).max(0.0);
        self.current_kg -= actual;
        self.current_kg = self.current_kg.clamp(self.min_kg(), self.capacity_kg);
        self.pressure_bar = self.pressure_from_fill();
        actual
    }

    /// Apply boil-off losses over a time interval (relevant for liquid H2 only).
    ///
    /// Returns the mass lost to boil-off \[kg\].
    pub fn boil_off(&mut self, dt_hours: f64) -> f64 {
        if self.boil_off_rate_pct_per_day < 1e-12 || self.current_kg < 1e-12 {
            return 0.0;
        }
        // Convert daily rate to hourly rate
        let hourly_rate_frac = self.boil_off_rate_pct_per_day / 100.0 / 24.0;
        let lost = self.current_kg * hourly_rate_frac * dt_hours;
        let lost = lost.min(self.current_kg - self.min_kg()).max(0.0);
        self.current_kg -= lost;
        self.current_kg = self.current_kg.max(0.0);
        self.pressure_bar = self.pressure_from_fill();
        lost
    }

    /// Compute tank pressure from fill level \[bar\].
    ///
    /// For compressed gas: linear relation P = P_max * SoC.
    /// For other types: returns max_pressure_bar (controlled by regulator or system).
    pub fn pressure_from_fill(&self) -> f64 {
        match self.storage_type {
            HydrogenStorageType::CompressedGas700bar | HydrogenStorageType::CompressedGas350bar => {
                (self.max_pressure_bar * self.soc()).max(1.0)
            }
            _ => self.max_pressure_bar,
        }
    }

    /// Energy required to compress a given mass of H2 to the tank storage pressure \[kWh\].
    pub fn compression_energy_kwh(&self, h2_to_store_kg: f64) -> f64 {
        h2_to_store_kg.max(0.0) * self.compression_energy_kwh_per_kg
    }

    /// Is the tank effectively full (SoC above 95% of capacity)?
    pub fn is_full(&self) -> bool {
        self.current_kg >= self.capacity_kg * 0.95
    }

    /// Is the tank at minimum SoC?
    pub fn is_empty(&self) -> bool {
        self.current_kg <= self.min_kg() + 1e-9
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_tank(capacity_kg: f64) -> HydrogenTank {
        HydrogenTank::new(capacity_kg, HydrogenStorageType::CompressedGas700bar)
    }

    #[test]
    fn test_tank_initial_state() {
        let tank = make_tank(1000.0);
        assert_eq!(tank.current_kg, 0.0);
        assert!((tank.soc()).abs() < 1e-9);
    }

    #[test]
    fn test_tank_charge_discharge() {
        let mut tank = make_tank(1000.0);
        // Charge 100 kg
        let charged = tank.charge(100.0, 1.0);
        assert!(
            (charged - 100.0).abs() < 1.0,
            "Should charge ~100 kg, got {charged}"
        );
        assert!(tank.soc() > 0.0);

        // Discharge 50 kg
        let discharged = tank.discharge(50.0, 1.0);
        assert!(
            (discharged - 50.0).abs() < 1.0,
            "Should discharge ~50 kg, got {discharged}"
        );
    }

    #[test]
    fn test_tank_min_soc_respected() {
        let mut tank = make_tank(1000.0);
        // Charge to exactly min_soc equivalent
        tank.current_kg = 50.0; // exactly at min_soc = 5%
        let discharged = tank.discharge(100.0, 1.0);
        // Should not discharge anything (already at min SoC)
        assert!(
            discharged < 1e-9,
            "Should not discharge below min SoC, discharged {discharged:.6} kg"
        );
    }

    #[test]
    fn test_tank_capacity_limit() {
        let mut tank = make_tank(100.0);
        // Try to charge more than capacity
        let charged = tank.charge(200.0, 1.0);
        assert!(tank.current_kg <= 100.0);
        assert!(charged <= 100.0);
    }

    #[test]
    fn test_tank_rate_limit() {
        let mut tank = make_tank(10_000.0);
        // max_charge_rate = capacity / 8 = 1250 kg/h
        // Try to charge 2000 kg in 1 hour → limited to 1250 kg
        let charged = tank.charge(2000.0, 1.0);
        assert!(
            charged <= tank.max_charge_rate_kg_per_h + 1.0,
            "Should be rate-limited, charged {charged:.2} kg"
        );
    }

    #[test]
    fn test_tank_boil_off_liquid_h2() {
        let mut tank = HydrogenTank::new(5000.0, HydrogenStorageType::LiquidH2);
        tank.current_kg = 1000.0;
        let initial = tank.current_kg;
        let lost = tank.boil_off(24.0); // 24 hours = 1 day
                                        // Should lose ~0.2% per day = 2 kg
        assert!(lost > 0.0, "Liquid H2 should have boil-off");
        assert!(
            tank.current_kg < initial,
            "Tank level should decrease due to boil-off"
        );
    }

    #[test]
    fn test_tank_no_boil_off_compressed() {
        let mut tank = make_tank(1000.0);
        tank.current_kg = 500.0;
        let lost = tank.boil_off(24.0);
        assert_eq!(lost, 0.0, "Compressed gas has no boil-off");
    }

    #[test]
    fn test_tank_pressure_from_fill() {
        let mut tank = make_tank(1000.0);
        tank.current_kg = 500.0; // 50% fill
        let p = tank.pressure_from_fill();
        // Expected: 700 * 0.5 = 350 bar
        assert!(
            (p - 350.0).abs() < 1.0,
            "Pressure should be ~350 bar at 50% fill, got {p:.2}"
        );
    }

    #[test]
    fn test_compression_energy() {
        let tank = make_tank(1000.0);
        let energy = tank.compression_energy_kwh(10.0);
        // 10 kg * 2.5 kWh/kg = 25 kWh
        assert!(
            (energy - 25.0).abs() < 0.1,
            "Expected 25 kWh compression energy, got {energy:.3}"
        );
    }

    #[test]
    fn test_soc_clamped() {
        let mut tank = make_tank(100.0);
        tank.current_kg = 150.0; // over-filled (invalid state)
        assert_eq!(tank.soc(), 1.0); // clamped to 1.0
    }
}
