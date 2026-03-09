//! Power Electronic Transformer (PET / Smart Transformer / SST) model.
//!
//! A PET replaces a conventional electromagnetic transformer with a cascade
//! of power-electronic conversion stages, enabling independent voltage and
//! reactive power control on each port, galvanic isolation, active harmonic
//! filtering, and DC bus access.
//!
//! # Typical three-stage architecture
//!
//! ```text
//! MV-AC ─► [AC/DC rectifier] ─► [DC/DC isolated converter] ─► [DC/AC inverter] ─► LV-AC
//!                                         │
//!                                       DC bus (optional port)
//! ```
//!
//! # Reference
//! Zhao et al., "Review of Energy Storage System for Wind Power Integration
//! Support", Applied Energy 137, 2015. (general SST reference)
use serde::{Deserialize, Serialize};

// ─── Error ────────────────────────────────────────────────────────────────────

/// Errors from the PET model.
#[derive(Debug, thiserror::Error)]
pub enum PetError {
    /// Requested transformer index is out of range.
    #[error("transformer index {0} out of range")]
    IndexOutOfRange(usize),
    /// A setpoint violates capability or physical limits.
    #[error("setpoint violation: {0}")]
    SetpointViolation(String),
    /// A requested capability is not available on this device.
    #[error("capability not available: {0}")]
    CapabilityUnavailable(String),
    /// Real power transfer exceeds the rated apparent power.
    #[error("overload: requested {requested_mva:.3} MVA exceeds rated {rated_mva:.3} MVA")]
    Overload {
        /// Requested power \[MVA\].
        requested_mva: f64,
        /// Rated power \[MVA\].
        rated_mva: f64,
    },
}

// ─── Configuration ────────────────────────────────────────────────────────────

/// Static configuration for a single PET unit.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PetConfig {
    /// Unique identifier.
    pub id: usize,
    /// Primary (medium-voltage) side rated voltage \[kV\].
    pub primary_voltage_kv: f64,
    /// Secondary (low-voltage) side rated voltage \[kV\].
    pub secondary_voltage_kv: f64,
    /// Rated apparent power \[MVA\].
    pub rated_power_mva: f64,
    /// Nameplate efficiency at full load \[%\].
    pub efficiency_pct: f64,
    /// Number of cascaded converter stages (typically 3).
    pub n_converter_stages: usize,
    /// Converter switching frequency \[kHz\].
    pub switching_freq_khz: f64,
}

// ─── Capabilities ─────────────────────────────────────────────────────────────

/// Boolean capability flags for a PET unit.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PetCapabilities {
    /// Independent voltage magnitude control on each port.
    pub voltage_regulation: bool,
    /// STATCOM-like reactive power injection on each port.
    pub reactive_power_control: bool,
    /// Active harmonic filtering on output.
    pub harmonic_filtering: bool,
    /// Accessible DC bus port between stages.
    pub dc_port: bool,
    /// Ability to operate as grid-forming source in islanded mode.
    pub islanding_capable: bool,
    /// Fault-current-limiting capability.
    pub fault_current_limiting: bool,
}

// ─── Setpoints ────────────────────────────────────────────────────────────────

/// Operator-commanded setpoints for a PET unit.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PetSetpoints {
    /// Real power transfer from primary to secondary \[MW\].
    pub p_transfer_mw: f64,
    /// Primary voltage magnitude setpoint \[pu\].
    pub v_primary_pu: f64,
    /// Secondary voltage magnitude setpoint \[pu\].
    pub v_secondary_pu: f64,
    /// Reactive power injection at primary port \[MVAr\].
    pub q_primary_mvar: f64,
    /// Reactive power injection at secondary port \[MVAr\].
    pub q_secondary_mvar: f64,
    /// DC-bus voltage setpoint \[pu\] (meaningful only when `dc_port = true`).
    pub dc_voltage_pu: f64,
}

// ─── Operating mode ───────────────────────────────────────────────────────────

/// Operating mode inferred from setpoints and capabilities.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PetMode {
    /// Bidirectional AC/AC power transfer at rated voltage.
    NormalTransfer,
    /// Independent voltage magnitude regulation on primary and/or secondary.
    VoltageRegulation,
    /// STATCOM-like reactive power support with minimal real power transfer.
    ReactiveSupport,
    /// Active harmonic filtering is the primary function.
    HarmonicFiltering,
    /// Fault-current-limiting mode.
    FaultCurrentLimiting,
    /// Grid-forming operation in islanded microgrid.
    Islanded,
}

// ─── Operating point ──────────────────────────────────────────────────────────

/// Computed steady-state operating point of a PET unit.
#[derive(Debug, Clone)]
pub struct PetOperatingPoint {
    /// Actual real power transferred primary → secondary \[MW\].
    pub p_transfer_mw: f64,
    /// Reactive power at primary port \[MVAr\].
    pub q_primary_mvar: f64,
    /// Reactive power at secondary port \[MVAr\].
    pub q_secondary_mvar: f64,
    /// Primary voltage magnitude \[pu\].
    pub v_primary_pu: f64,
    /// Secondary voltage magnitude \[pu\].
    pub v_secondary_pu: f64,
    /// Total converter losses \[MW\].
    pub losses_mw: f64,
    /// Actual efficiency at this operating point \[%\].
    pub efficiency_pct: f64,
    /// Inferred operating mode.
    pub operating_mode: PetMode,
}

// ─── Network container ────────────────────────────────────────────────────────

/// Collection of PET units forming a smart-transformer network.
pub struct PetNetwork {
    /// (config, capabilities, setpoints) tuple for each installed PET.
    pub transformers: Vec<(PetConfig, PetCapabilities, PetSetpoints)>,
}

impl Default for PetNetwork {
    fn default() -> Self {
        Self::new()
    }
}

impl PetNetwork {
    /// Create an empty PET network.
    pub fn new() -> Self {
        Self {
            transformers: Vec::new(),
        }
    }

    /// Add a PET unit to the network.
    pub fn add_pet(&mut self, config: PetConfig, caps: PetCapabilities, setpoints: PetSetpoints) {
        self.transformers.push((config, caps, setpoints));
    }

    // ── Operating point ───────────────────────────────────────────────────────

    /// Compute the steady-state operating point for transformer `idx`.
    ///
    /// Losses follow a load-dependent efficiency model:
    /// - η(p) = η_rated − (1 − p_ratio)² × 2.0   \[%\]
    /// - losses = P_transfer × (1 − η(p)/100)
    ///
    /// where `p_ratio = |P_transfer| / rated_power_mva`.
    ///
    /// # Errors
    /// Returns [`PetError::IndexOutOfRange`] if `idx` is out of bounds.
    pub fn solve_operating_point(&self, idx: usize) -> Result<PetOperatingPoint, PetError> {
        let (cfg, caps, sp) = self
            .transformers
            .get(idx)
            .ok_or(PetError::IndexOutOfRange(idx))?;

        let p = sp.p_transfer_mw.abs();
        let p_ratio = if cfg.rated_power_mva > f64::EPSILON {
            p / cfg.rated_power_mva
        } else {
            0.0
        };
        let p_ratio_clamped = p_ratio.clamp(0.0, 1.0);

        // Load-dependent efficiency (better at higher load)
        let eta = (cfg.efficiency_pct - (1.0 - p_ratio_clamped).powi(2) * 2.0).clamp(0.0, 100.0);

        let loss_fraction = 1.0 - eta / 100.0;
        let losses_mw = p * loss_fraction;

        let mode = Self::infer_mode(caps, sp);

        Ok(PetOperatingPoint {
            p_transfer_mw: sp.p_transfer_mw,
            q_primary_mvar: sp.q_primary_mvar,
            q_secondary_mvar: sp.q_secondary_mvar,
            v_primary_pu: sp.v_primary_pu,
            v_secondary_pu: sp.v_secondary_pu,
            losses_mw,
            efficiency_pct: eta,
            operating_mode: mode,
        })
    }

    // ── Setpoint validation ───────────────────────────────────────────────────

    /// Verify all setpoints are within capability and physical limits for `idx`.
    ///
    /// Checks:
    /// 1. `|p_transfer_mw|` ≤ `rated_power_mva` (apparent power limit)
    /// 2. `v_primary_pu` and `v_secondary_pu` in \[0.9, 1.1\]
    /// 3. Non-zero reactive setpoints require `reactive_power_control = true`
    ///
    /// # Errors
    /// Returns [`PetError::Overload`], [`PetError::SetpointViolation`], or
    /// [`PetError::CapabilityUnavailable`] on the first violation found.
    pub fn validate_setpoints(&self, idx: usize) -> Result<(), PetError> {
        let (cfg, caps, sp) = self
            .transformers
            .get(idx)
            .ok_or(PetError::IndexOutOfRange(idx))?;

        // 1. Overload check
        let s_requested = sp.p_transfer_mw.abs();
        if s_requested > cfg.rated_power_mva + f64::EPSILON {
            return Err(PetError::Overload {
                requested_mva: s_requested,
                rated_mva: cfg.rated_power_mva,
            });
        }

        // 2. Voltage limits \[0.9, 1.1\] pu
        if !(0.9..=1.1).contains(&sp.v_primary_pu) {
            return Err(PetError::SetpointViolation(format!(
                "v_primary_pu {:.4} is outside [0.90, 1.10] pu",
                sp.v_primary_pu
            )));
        }
        if !(0.9..=1.1).contains(&sp.v_secondary_pu) {
            return Err(PetError::SetpointViolation(format!(
                "v_secondary_pu {:.4} is outside [0.90, 1.10] pu",
                sp.v_secondary_pu
            )));
        }

        // 3. Reactive power requires capability
        if !caps.reactive_power_control
            && (sp.q_primary_mvar.abs() > f64::EPSILON || sp.q_secondary_mvar.abs() > f64::EPSILON)
        {
            return Err(PetError::CapabilityUnavailable(
                "reactive_power_control is disabled but Q setpoints are non-zero".into(),
            ));
        }

        Ok(())
    }

    // ── Stage losses ──────────────────────────────────────────────────────────

    /// Compute approximate losses per converter stage \[MW\].
    ///
    /// Returns a `Vec` of length `n_converter_stages`.  Total stage losses sum
    /// to the overall converter losses at the given `p_transfer_mw`.
    ///
    /// Uses the same load-dependent efficiency model as `solve_operating_point`:
    /// η(p) = η_rated − (1 − p/S_rated)² × 2.
    pub fn compute_stage_losses(&self, idx: usize, p_transfer_mw: f64) -> Vec<f64> {
        let (cfg, _, _) = match self.transformers.get(idx) {
            Some(t) => t,
            None => return Vec::new(),
        };

        let p = p_transfer_mw.abs();
        let n = cfg.n_converter_stages.max(1);
        let p_ratio = if cfg.rated_power_mva > f64::EPSILON {
            (p / cfg.rated_power_mva).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let eta = (cfg.efficiency_pct - (1.0 - p_ratio).powi(2) * 2.0).clamp(0.0, 100.0);
        let total_loss_fraction = 1.0 - eta / 100.0;
        let per_stage_loss = p * total_loss_fraction / n as f64;

        vec![per_stage_loss; n]
    }

    // ── Mode inference ────────────────────────────────────────────────────────

    fn infer_mode(caps: &PetCapabilities, sp: &PetSetpoints) -> PetMode {
        // Fault-current-limiting overrides everything
        if caps.fault_current_limiting {
            return PetMode::FaultCurrentLimiting;
        }
        // Reactive support: Q setpoint non-zero, P is zero → STATCOM mode
        if caps.reactive_power_control
            && (sp.q_primary_mvar.abs() > f64::EPSILON || sp.q_secondary_mvar.abs() > f64::EPSILON)
            && sp.p_transfer_mw.abs() < f64::EPSILON
        {
            return PetMode::ReactiveSupport;
        }
        // Harmonic filtering: no P, no Q
        if caps.harmonic_filtering
            && sp.p_transfer_mw.abs() < f64::EPSILON
            && sp.q_primary_mvar.abs() < f64::EPSILON
            && sp.q_secondary_mvar.abs() < f64::EPSILON
            && !caps.islanding_capable
        {
            return PetMode::HarmonicFiltering;
        }
        // Islanded: no P transfer, islanding capable
        if caps.islanding_capable && sp.p_transfer_mw.abs() < f64::EPSILON {
            return PetMode::Islanded;
        }
        // Voltage regulation: voltage setpoints differ from nominal
        if caps.voltage_regulation
            && ((sp.v_primary_pu - 1.0).abs() > 1e-4 || (sp.v_secondary_pu - 1.0).abs() > 1e-4)
        {
            return PetMode::VoltageRegulation;
        }
        PetMode::NormalTransfer
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn base_config() -> PetConfig {
        PetConfig {
            id: 0,
            primary_voltage_kv: 20.0,
            secondary_voltage_kv: 0.4,
            rated_power_mva: 1.0,
            efficiency_pct: 97.0,
            n_converter_stages: 3,
            switching_freq_khz: 10.0,
        }
    }

    fn full_caps() -> PetCapabilities {
        PetCapabilities {
            voltage_regulation: true,
            reactive_power_control: true,
            harmonic_filtering: true,
            dc_port: true,
            islanding_capable: true,
            fault_current_limiting: false,
        }
    }

    fn normal_setpoints() -> PetSetpoints {
        PetSetpoints {
            p_transfer_mw: 0.8,
            v_primary_pu: 1.0,
            v_secondary_pu: 1.0,
            q_primary_mvar: 0.0,
            q_secondary_mvar: 0.0,
            dc_voltage_pu: 1.0,
        }
    }

    /// Test 1: Normal transfer — power balance and losses are physically reasonable.
    #[test]
    fn test_normal_transfer_power_balance() {
        let mut net = PetNetwork::new();
        net.add_pet(base_config(), full_caps(), normal_setpoints());

        let op = net.solve_operating_point(0).expect("solve failed");
        assert_eq!(op.operating_mode, PetMode::NormalTransfer);
        assert!((op.p_transfer_mw - 0.8).abs() < 1e-9);
        // Losses ≥ 0 and < 10% of rated
        assert!(op.losses_mw >= 0.0);
        assert!(op.losses_mw < 0.1, "losses {:.4} MW too high", op.losses_mw);
        // Efficiency near nameplate at high load
        assert!(
            op.efficiency_pct > 95.0,
            "efficiency {:.2}% should be > 95% at 80% load",
            op.efficiency_pct
        );
    }

    /// Test 2: Voltage regulation mode — independent V on each port.
    #[test]
    fn test_voltage_regulation_mode() {
        let mut net = PetNetwork::new();
        let sp = PetSetpoints {
            p_transfer_mw: 0.5,
            v_primary_pu: 1.05,
            v_secondary_pu: 0.95,
            q_primary_mvar: 0.0,
            q_secondary_mvar: 0.0,
            dc_voltage_pu: 1.0,
        };
        net.add_pet(base_config(), full_caps(), sp.clone());

        let op = net.solve_operating_point(0).expect("solve failed");
        assert_eq!(op.operating_mode, PetMode::VoltageRegulation);
        assert!((op.v_primary_pu - 1.05).abs() < 1e-9);
        assert!((op.v_secondary_pu - 0.95).abs() < 1e-9);
    }

    /// Test 3: Reactive support (STATCOM mode) — non-zero Q at zero P.
    #[test]
    fn test_reactive_support_mode() {
        let mut net = PetNetwork::new();
        let sp = PetSetpoints {
            p_transfer_mw: 0.0,
            v_primary_pu: 1.0,
            v_secondary_pu: 1.0,
            q_primary_mvar: 0.3,
            q_secondary_mvar: 0.0,
            dc_voltage_pu: 1.0,
        };
        net.add_pet(base_config(), full_caps(), sp);

        let op = net.solve_operating_point(0).expect("solve failed");
        assert_eq!(op.operating_mode, PetMode::ReactiveSupport);
        assert!((op.q_primary_mvar - 0.3).abs() < 1e-9);
        // Zero real transfer means near-zero losses
        assert!(op.losses_mw < 1e-9);
    }

    /// Test 4: Efficiency curve — higher load → higher efficiency.
    #[test]
    fn test_efficiency_increases_with_load() {
        let mut net = PetNetwork::new();
        let mut sp_low = normal_setpoints();
        sp_low.p_transfer_mw = 0.1;
        let mut sp_high = normal_setpoints();
        sp_high.p_transfer_mw = 0.95;

        net.add_pet(base_config(), full_caps(), sp_low);
        net.add_pet(base_config(), full_caps(), sp_high);

        let op_low = net.solve_operating_point(0).expect("low-load solve failed");
        let op_high = net
            .solve_operating_point(1)
            .expect("high-load solve failed");

        assert!(
            op_high.efficiency_pct > op_low.efficiency_pct,
            "η(high={:.2}%) should exceed η(low={:.2}%)",
            op_high.efficiency_pct,
            op_low.efficiency_pct
        );
    }

    /// Test 5: Overload — validate_setpoints returns PetError::Overload.
    #[test]
    fn test_overload_returns_error() {
        let mut net = PetNetwork::new();
        let mut sp = normal_setpoints();
        sp.p_transfer_mw = 1.5; // exceeds rated 1.0 MVA
        net.add_pet(base_config(), full_caps(), sp);

        let result = net.validate_setpoints(0);
        assert!(
            matches!(result, Err(PetError::Overload { .. })),
            "Expected Overload error, got: {:?}",
            result
        );
    }

    /// Test 6: Reactive setpoint without capability → CapabilityUnavailable.
    #[test]
    fn test_reactive_setpoint_without_capability() {
        let mut net = PetNetwork::new();
        let mut caps = full_caps();
        caps.reactive_power_control = false;
        let mut sp = normal_setpoints();
        sp.q_primary_mvar = 0.2;
        net.add_pet(base_config(), caps, sp);

        let result = net.validate_setpoints(0);
        assert!(
            matches!(result, Err(PetError::CapabilityUnavailable(_))),
            "Expected CapabilityUnavailable, got: {:?}",
            result
        );
    }

    /// Test 7: Stage losses — length equals n_converter_stages, sum is consistent.
    #[test]
    fn test_stage_losses_length_and_sum() {
        let mut net = PetNetwork::new();
        net.add_pet(base_config(), full_caps(), normal_setpoints());

        let losses = net.compute_stage_losses(0, 0.8);
        assert_eq!(losses.len(), 3, "should have 3 stage losses");

        let total: f64 = losses.iter().sum();
        let op = net.solve_operating_point(0).expect("solve failed");
        // Sum of stage losses should roughly match computed losses_mw
        assert!(
            (total - op.losses_mw).abs() < 1e-9,
            "stage loss sum {total:.6} MW should match op.losses_mw {:.6} MW",
            op.losses_mw
        );
    }
}
