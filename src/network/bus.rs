use crate::units::{Power, ReactivePower, Voltage};
use serde::{Deserialize, Serialize};

/// Bus type classification used by the power flow solver.
///
/// Determines which quantities are known (specified) vs. solved (unknown):
///
/// | Type  | Known             | Solved       |
/// |-------|-------------------|--------------|
/// | Slack | \|V\|, ∠V         | P, Q         |
/// | PV    | P, \|V\|          | Q, ∠V        |
/// | PQ    | P, Q              | \|V\|, ∠V    |
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum BusType {
    /// Reference / swing bus.  Voltage magnitude and angle are fixed (∠V = 0).
    Slack,
    /// Generator bus.  Active power and voltage magnitude are controlled.
    PV,
    /// Load bus.  Active and reactive power demand are known.
    PQ,
}

/// AC power system bus (node) with load, generation, and shunt data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bus {
    /// External bus identifier (1-based, as in MATPOWER/IEEE-CDF files).
    pub id: usize,
    /// Human-readable bus name (defaults to `"Bus {id}"`).
    pub name: String,
    /// Bus classification for power flow (Slack / PV / PQ).
    pub bus_type: BusType,
    /// Nominal base voltage `kV` (stored as `Voltage` with inner value in kV).
    pub base_kv: Voltage,
    /// Initial / solved voltage magnitude [p.u.].
    pub vm: f64,
    /// Initial / solved voltage angle `radians`.
    pub va: f64,
    /// Real power demand (load) `MW`.
    pub pd: Power,
    /// Reactive power demand (load) `MVAr`.
    pub qd: ReactivePower,
    /// Shunt conductance [MW at V = 1.0 p.u.].
    pub gs: f64,
    /// Shunt susceptance [MVAr injected at V = 1.0 p.u.].
    pub bs: f64,
    /// Control zone (optional, informational).
    pub zone: Option<u32>,
}

impl Bus {
    /// Create a bus with default values (V = 1.0 p.u., zero load, zero shunt).
    pub fn new(id: usize, bus_type: BusType) -> Self {
        Self {
            id,
            name: format!("Bus {id}"),
            bus_type,
            base_kv: Voltage(1.0),
            vm: 1.0,
            va: 0.0,
            pd: Power(0.0),
            qd: ReactivePower(0.0),
            gs: 0.0,
            bs: 0.0,
            zone: None,
        }
    }
}
