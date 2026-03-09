//! Power transformer detailed modeling module.
//!
//! Implements the IEC 60076 / IEEE C57 transformer equivalent circuit, admittance
//! matrix stamping, on-load tap changer (OLTC) control, IEC 60076-7 thermal aging
//! model, dissolved gas analysis (DGA) health assessment, and three-winding star
//! equivalent conversion.
//!
//! # Unit conventions
//! - Voltages: kV (unless noted as pu)
//! - Powers: MVA / MW / Mvar
//! - Currents: kA / A
//! - Temperatures: °C
//! - Times: minutes (thermal), seconds (OLTC delay)
//! - Impedances/admittances: per-unit on transformer rating base

use num_complex::Complex;
use std::fmt;

// ---------------------------------------------------------------------------
// Supporting enumerations
// ---------------------------------------------------------------------------

/// Winding vector group of a two-winding power transformer (IEC 60076-1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VectorGroup {
    /// Delta primary / star secondary with neutral, 30° lag (most common distribution).
    Dyn11,
    /// Star / star with neutral, 0° shift.
    Yny0,
    /// Delta primary / star secondary with neutral, 150° lag.
    Dyn5,
    /// Star with neutral primary / star secondary with neutral, 0° shift.
    YNyn0,
    /// Delta primary / zigzag secondary with neutral, 0° shift.
    Dzn0,
    /// Star-star-delta, 0° shift.
    Yyd0,
}

impl VectorGroup {
    /// Phase shift introduced by this vector group in degrees
    /// (positive = secondary lags primary).
    pub fn phase_shift_deg(&self) -> f64 {
        match self {
            VectorGroup::Dyn11 => 30.0,
            VectorGroup::Yny0 => 0.0,
            VectorGroup::Dyn5 => 150.0,
            VectorGroup::YNyn0 => 0.0,
            VectorGroup::Dzn0 => 0.0,
            VectorGroup::Yyd0 => 0.0,
        }
    }
}

impl fmt::Display for VectorGroup {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            VectorGroup::Dyn11 => "Dyn11",
            VectorGroup::Yny0 => "Yny0",
            VectorGroup::Dyn5 => "Dyn5",
            VectorGroup::YNyn0 => "YNyn0",
            VectorGroup::Dzn0 => "Dzn0",
            VectorGroup::Yyd0 => "Yyd0",
        };
        write!(f, "{s}")
    }
}

/// Transformer cooling classification (ONAN/ONAF/OFAF/ODAF per IEC 60076-2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoolingType {
    /// Oil Natural Air Natural — self-cooled baseline.
    Onan,
    /// Oil Natural Air Forced — fans on radiators.
    Onaf,
    /// Oil Forced Air Forced — pumped oil + fans.
    Ofaf,
    /// Oil Directed Air Forced — directed oil flow + fans.
    Odaf,
}

impl fmt::Display for CoolingType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            CoolingType::Onan => "ONAN",
            CoolingType::Onaf => "ONAF",
            CoolingType::Ofaf => "OFAF",
            CoolingType::Odaf => "ODAF",
        };
        write!(f, "{s}")
    }
}

/// Continuous rating factor for a given cooling stage.
///
/// Transformers often carry ONAN/ONAF/OFAF ratings as multiples of the ONAN MVA.
/// E.g., `RatingFactor { cooling: Onaf, factor: 1.17 }` means 17 % more capacity
/// with fans running.
#[derive(Debug, Clone)]
pub struct RatingFactor {
    /// Cooling class this factor applies to.
    pub cooling: CoolingType,
    /// Ratio of this cooling class MVA to nameplate ONAN MVA (dimensionless).
    pub factor: f64,
}

// ---------------------------------------------------------------------------
// Core transformer model (two-winding)
// ---------------------------------------------------------------------------

/// Two-winding power transformer T-equivalent circuit model.
///
/// All impedances are expressed in per-unit on the transformer's own MVA / kV base.
/// The magnetising branch is placed on the primary (HV) side, giving the classical
/// T-equivalent:
///
/// ```text
///  HV(i) ---[R+jX]--- internal node ---[R+jX (LV)]--- LV(j)
///                         |
///                       [Gfe - jBmag]
///                         |
///                        GND
/// ```
///
/// For the admittance-matrix stamp the model is converted to the π-equivalent with
/// the off-nominal tap `a = tap_ratio * exp(j * phase_shift_rad)`.
#[derive(Debug, Clone)]
pub struct TransformerModel {
    /// Unique identifier (must match branch numbering in the network).
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// Nameplate power rating \[MVA\].
    pub rated_mva: f64,
    /// Rated voltage on the high-voltage (primary) side \[kV\].
    pub v_hv_kv: f64,
    /// Rated voltage on the low-voltage (secondary) side \[kV\].
    pub v_lv_kv: f64,
    /// Winding series resistance (both windings referred to HV) \[pu\].
    pub r_pu: f64,
    /// Total leakage reactance (both windings referred to HV) \[pu\].
    pub x_pu: f64,
    /// Core loss conductance (no-load losses / V²) \[pu\].
    pub g_fe_pu: f64,
    /// Magnetising susceptance (no-load reactive current) \[pu\].
    pub b_mag_pu: f64,
    /// Off-nominal tap ratio (1.0 = nominal). Unitless; positive deviations raise
    /// the secondary voltage.
    pub tap_ratio: f64,
    /// Phase shift introduced by this transformer \[degrees\]. Non-zero for
    /// phase-shifting transformers; typically matches the vector-group shift.
    pub phase_shift_deg: f64,
    /// Winding connection vector group per IEC 60076-1.
    pub vector_group: VectorGroup,
    /// Active cooling classification.
    pub cooling: CoolingType,
    /// Thermal rating factors for each available cooling stage.
    pub rating_factors: Vec<RatingFactor>,
}

impl TransformerModel {
    /// Construct a new transformer with sensible defaults for an ONAN unit.
    ///
    /// # Arguments
    /// * `id` — network branch index
    /// * `rated_mva` — nameplate MVA rating
    /// * `v_hv_kv` / `v_lv_kv` — winding rated voltages \[kV\]
    /// * `r_pu` / `x_pu` — short-circuit impedance components \[pu\]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: usize,
        name: impl Into<String>,
        rated_mva: f64,
        v_hv_kv: f64,
        v_lv_kv: f64,
        r_pu: f64,
        x_pu: f64,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            rated_mva,
            v_hv_kv,
            v_lv_kv,
            r_pu,
            x_pu,
            g_fe_pu: 0.0,
            b_mag_pu: 0.0,
            tap_ratio: 1.0,
            phase_shift_deg: 0.0,
            vector_group: VectorGroup::Yny0,
            cooling: CoolingType::Onan,
            rating_factors: vec![RatingFactor {
                cooling: CoolingType::Onan,
                factor: 1.0,
            }],
        }
    }

    /// Rated current on the HV side \[kA\].
    ///
    /// `I_rated = S_rated / (√3 · V_hv)`
    pub fn rated_current_ka(&self) -> f64 {
        self.rated_mva / (3.0_f64.sqrt() * self.v_hv_kv)
    }

    /// Rated current on the HV side \[A\].
    pub fn rated_current_amps_hv(&self) -> f64 {
        self.rated_current_ka() * 1000.0
    }

    /// Rated current on the LV side \[A\].
    ///
    /// `I_lv = S_rated / (√3 · V_lv)`
    pub fn rated_current_amps_lv(&self) -> f64 {
        self.rated_mva / (3.0_f64.sqrt() * self.v_lv_kv) * 1000.0
    }

    /// Maximum MVA rating across all available cooling stages.
    pub fn max_mva(&self) -> f64 {
        let max_factor = self
            .rating_factors
            .iter()
            .map(|rf| rf.factor)
            .fold(1.0_f64, f64::max);
        self.rated_mva * max_factor
    }

    /// Compute the four Y-bus admittance contributions for buses `i` (HV) and `j` (LV).
    ///
    /// Uses the π-equivalent stamp derived from the off-nominal tap complex ratio
    /// `a = |tap_ratio| · exp(j · φ)` where φ = phase_shift_deg converted to radians.
    ///
    /// The series admittance is `y_s = 1 / (R + jX)` and the shunt admittance is
    /// `y_m = G_fe - j·B_mag` (placed on the HV side).
    ///
    /// Y-bus stamp (Grainger & Stevenson derivation):
    /// ```text
    /// Y_ii = (y_s + y_m) / |a|²
    /// Y_jj =  y_s
    /// Y_ij = -y_s / conj(a)
    /// Y_ji = -y_s / a
    /// ```
    pub fn y_matrix_elements(&self) -> TransformerYElements {
        let y_s = self.series_admittance();
        let y_m = Complex::new(self.g_fe_pu, -self.b_mag_pu);
        let phi = self.phase_shift_deg.to_radians();
        // Complex tap ratio a = tap_ratio * e^{jφ}
        let a = Complex::new(self.tap_ratio * phi.cos(), self.tap_ratio * phi.sin());
        let a_sq = self.tap_ratio * self.tap_ratio; // |a|²

        let y_ii = (y_s + y_m) / a_sq;
        let y_jj = y_s;
        let y_ij = -y_s / a.conj();
        let y_ji = -y_s / a;

        TransformerYElements {
            y_ii,
            y_jj,
            y_ij,
            y_ji,
        }
    }

    /// Series admittance `y_s = 1 / (r_pu + j·x_pu)` \[pu\].
    fn series_admittance(&self) -> Complex<f64> {
        let z = Complex::new(self.r_pu, self.x_pu);
        if z.norm_sqr() < f64::EPSILON {
            // Ideal (lossless) transformer — return large but finite admittance
            Complex::new(1e10, 0.0)
        } else {
            z.inv()
        }
    }
}

/// Four Y-bus element contributions from a two-winding transformer.
///
/// Bus indices: `i` = HV side, `j` = LV side.
/// Add these to the corresponding Y-bus matrix entries:
/// - `Y[i,i] += y_ii`
/// - `Y[j,j] += y_jj`
/// - `Y[i,j] += y_ij`
/// - `Y[j,i] += y_ji`
#[derive(Debug, Clone)]
pub struct TransformerYElements {
    /// HV bus self-admittance contribution \[pu\].
    pub y_ii: Complex<f64>,
    /// LV bus self-admittance contribution \[pu\].
    pub y_jj: Complex<f64>,
    /// HV→LV mutual admittance (off-diagonal) \[pu\].
    pub y_ij: Complex<f64>,
    /// LV→HV mutual admittance (off-diagonal) \[pu\].
    pub y_ji: Complex<f64>,
}

// ---------------------------------------------------------------------------
// On-Load Tap Changer (OLTC)
// ---------------------------------------------------------------------------

/// Action returned by [`OltcController::update`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OltcAction {
    /// Voltage within deadband — no tap movement needed.
    NoChange,
    /// Tap raised by the given increment \[pu\].
    TapUp(f64),
    /// Tap lowered by the given increment \[pu\].
    TapDown(f64),
    /// Voltage outside deadband but tap already at its mechanical limit.
    AtLimit,
}

/// Automatic voltage regulator using an on-load tap changer (OLTC).
///
/// Models the mechanical delay and deadband of a real tap changer per
/// IEC 60214-1 / CIGRÉ WG A2.16 recommendations.
///
/// # Typical parameter values
/// | Parameter | Typical value |
/// |-----------|--------------|
/// | `tap_min` | 0.85 pu |
/// | `tap_max` | 1.15 pu |
/// | `tap_step`| 0.00625 pu (16 steps each side) |
/// | `deadband`| ±0.01 pu |
/// | `delay_s` | 30 s (first operation), 5 s (subsequent) |
#[derive(Debug, Clone)]
pub struct OltcController {
    /// Minimum allowable tap position \[pu\].
    pub tap_min: f64,
    /// Maximum allowable tap position \[pu\].
    pub tap_max: f64,
    /// Tap increment per step \[pu\].
    pub tap_step: f64,
    /// Target (setpoint) voltage at the controlled bus \[pu\].
    pub v_setpoint_pu: f64,
    /// Voltage deadband half-width (symmetric) \[pu\].
    /// The controller acts when |V_meas - V_set| > deadband.
    pub deadband_pu: f64,
    /// Mechanical operating delay \[s\] — tap moves only after voltage has
    /// remained outside the deadband for this duration.
    pub delay_s: f64,
    /// Current tap position \[pu\].
    pub current_tap: f64,
    /// Time \[s\] that the voltage has been continuously outside the deadband.
    /// Resets to zero when voltage re-enters the deadband.
    pub pending_time: f64,
    /// Cumulative number of tap change operations (wear indicator).
    pub total_tap_operations: usize,
}

impl OltcController {
    /// Create a new OLTC controller with 30 s delay and ±0.01 pu deadband.
    pub fn new(tap_min: f64, tap_max: f64, tap_step: f64, v_setpoint_pu: f64) -> Self {
        Self {
            tap_min,
            tap_max,
            tap_step,
            v_setpoint_pu,
            deadband_pu: 0.01,
            delay_s: 30.0,
            current_tap: 1.0,
            pending_time: 0.0,
            total_tap_operations: 0,
        }
    }

    /// Advance the OLTC state by `dt_s` seconds given a measured bus voltage
    /// `v_measured_pu` \[pu\].
    ///
    /// Returns an [`OltcAction`] describing what the controller did.
    ///
    /// Algorithm:
    /// 1. If |V - V_set| ≤ deadband → reset timer, return `NoChange`.
    /// 2. Otherwise accumulate `pending_time`.
    /// 3. When `pending_time ≥ delay_s` → attempt a tap step, reset timer.
    pub fn update(&mut self, v_measured_pu: f64, dt_s: f64) -> OltcAction {
        let error = v_measured_pu - self.v_setpoint_pu;
        if error.abs() <= self.deadband_pu {
            self.pending_time = 0.0;
            return OltcAction::NoChange;
        }

        self.pending_time += dt_s;
        if self.pending_time < self.delay_s {
            return OltcAction::NoChange;
        }

        // Delay expired — attempt a tap change
        self.pending_time = 0.0;

        if error < 0.0 {
            // Voltage too low → raise tap (increases secondary voltage)
            let new_tap = self.current_tap + self.tap_step;
            if new_tap > self.tap_max + f64::EPSILON {
                OltcAction::AtLimit
            } else {
                self.current_tap = new_tap.min(self.tap_max);
                self.total_tap_operations += 1;
                OltcAction::TapUp(self.tap_step)
            }
        } else {
            // Voltage too high → lower tap
            let new_tap = self.current_tap - self.tap_step;
            if new_tap < self.tap_min - f64::EPSILON {
                OltcAction::AtLimit
            } else {
                self.current_tap = new_tap.max(self.tap_min);
                self.total_tap_operations += 1;
                OltcAction::TapDown(self.tap_step)
            }
        }
    }

    /// Number of available tap steps above the current position.
    pub fn steps_up_available(&self) -> usize {
        if self.tap_step <= 0.0 {
            return 0;
        }
        ((self.tap_max - self.current_tap) / self.tap_step).round() as usize
    }

    /// Number of available tap steps below the current position.
    pub fn steps_down_available(&self) -> usize {
        if self.tap_step <= 0.0 {
            return 0;
        }
        ((self.current_tap - self.tap_min) / self.tap_step).round() as usize
    }
}

// ---------------------------------------------------------------------------
// Thermal model (IEC 60076-7)
// ---------------------------------------------------------------------------

/// First-order thermal model of a power transformer per IEC 60076-7.
///
/// The model tracks two state variables:
/// - `θ_o` — top-oil temperature \[°C\]
/// - `Δθ_h` — hotspot-to-top-oil gradient \[°C\]
///
/// Differential equations (Euler integration):
/// ```text
/// dθ_o/dt = (1/τ_o) · (Δθ_or · K^{2n} + θ_a − θ_o)   [°C/min]
/// dΔθ_h/dt = (1/τ_w) · (Δθ_hr · K^{2m} − Δθ_h)       [°C/min]
/// ```
/// where `K` is the per-unit load factor, `n` and `m` are empirical exponents
/// depending on cooling class.
///
/// Hotspot temperature: `θ_H = θ_o + Δθ_h`
///
/// Aging acceleration factor (FAA) per Annex D of IEC 60076-7:
/// ```text
/// FAA = exp(15000/383 − 15000/(θ_H + 273))
/// ```
/// FAA = 1.0 at reference hotspot 98 °C.
#[derive(Debug, Clone)]
pub struct TransformerThermalModel {
    /// Ambient temperature \[°C\].
    pub theta_a: f64,
    /// Rated top-oil temperature rise over ambient \[°C\] (typical: 55 °C ONAN).
    pub theta_or: f64,
    /// Rated hotspot-to-top-oil gradient at rated load \[°C\] (typical: 23 °C ONAN).
    pub theta_hr: f64,
    /// Oil thermal time constant \[min\] (typical: 180 min ONAN).
    pub tau_o: f64,
    /// Winding thermal time constant \[min\] (typical: 10 min ONAN).
    pub tau_w: f64,
    /// Oil temperature-rise exponent `n` (0.8 ONAN/ONAF, 1.0 OFAF/ODAF).
    pub n: f64,
    /// Winding temperature-rise exponent `m` (0.8 ONAN, 1.0 OFAF/ODAF).
    pub m: f64,
    /// Current top-oil temperature (state variable) \[°C\].
    pub theta_o: f64,
    /// Current hotspot-to-top-oil gradient (state variable) \[°C\].
    pub delta_theta_h: f64,
}

impl TransformerThermalModel {
    /// Construct an ONAN transformer thermal model at the given ambient temperature.
    ///
    /// IEC 60076-7 Table 1 default parameters for ONAN cooling:
    /// - Δθ_or = 55 °C, Δθ_hr = 23 °C
    /// - τ_o = 180 min, τ_w = 10 min
    /// - n = 0.8, m = 0.8
    ///
    /// Initial temperatures are set to steady-state at zero load.
    pub fn new_onan(theta_ambient: f64) -> Self {
        Self {
            theta_a: theta_ambient,
            theta_or: 55.0,
            theta_hr: 23.0,
            tau_o: 180.0,
            tau_w: 10.0,
            n: 0.8,
            m: 0.8,
            theta_o: theta_ambient, // no-load: top-oil ≈ ambient
            delta_theta_h: 0.0,     // no-load: no winding gradient
        }
    }

    /// Construct an OFAF (oil forced, air forced) thermal model.
    ///
    /// IEC 60076-7 exponents: n = 1.0, m = 1.0.
    /// Rated rises are lower due to better heat exchange: Δθ_or = 45 °C, Δθ_hr = 20 °C.
    pub fn new_ofaf(theta_ambient: f64) -> Self {
        Self {
            theta_a: theta_ambient,
            theta_or: 45.0,
            theta_hr: 20.0,
            tau_o: 90.0,
            tau_w: 7.0,
            n: 1.0,
            m: 1.0,
            theta_o: theta_ambient,
            delta_theta_h: 0.0,
        }
    }

    /// Advance the thermal model by `dt_min` minutes at load factor `K` (pu).
    ///
    /// Uses forward Euler integration of the IEC 60076-7 differential equations.
    /// For accurate results use small time steps (dt_min ≤ 1 min).
    ///
    /// # Arguments
    /// * `load_factor_k` — per-unit load (1.0 = rated, 1.2 = 20 % overload)
    /// * `dt_min` — integration step \[minutes\]
    pub fn step(&mut self, load_factor_k: f64, dt_min: f64) {
        let k = load_factor_k.max(0.0);

        // Steady-state top-oil rise at load K: Δθ_or · K^{2n}
        let d_theta_o_ss = self.theta_or * k.powf(2.0 * self.n);
        // Steady-state hotspot gradient: Δθ_hr · K^{2m}
        let d_theta_h_ss = self.theta_hr * k.powf(2.0 * self.m);

        // Forward Euler integration
        let d_theta_o_dot = (d_theta_o_ss + self.theta_a - self.theta_o) / self.tau_o;
        let d_theta_h_dot = (d_theta_h_ss - self.delta_theta_h) / self.tau_w;

        self.theta_o += d_theta_o_dot * dt_min;
        self.delta_theta_h += d_theta_h_dot * dt_min;
        // Clamp gradient to non-negative (physically: hotspot ≥ top-oil)
        self.delta_theta_h = self.delta_theta_h.max(0.0);
    }

    /// Current hotspot temperature `θ_H = θ_o + Δθ_h` \[°C\].
    pub fn hotspot_temperature(&self) -> f64 {
        self.theta_o + self.delta_theta_h
    }

    /// Current top-oil temperature \[°C\].
    pub fn top_oil_temperature(&self) -> f64 {
        self.theta_o
    }

    /// Aging acceleration factor (FAA) per IEC 60076-7 Annex D.
    ///
    /// FAA = 1.0 at the reference hotspot temperature of 98 °C (371 K).
    /// FAA > 1 means faster-than-normal aging; FAA < 1 means slower.
    ///
    /// Formula: `FAA = exp(15000/371 − 15000/(θ_H + 273))`
    ///
    /// where 371 K = 98 °C + 273 (the IEC reference temperature).
    pub fn aging_acceleration_factor(&self) -> f64 {
        let theta_h = self.hotspot_temperature();
        // IEC 60076-7 Annex D equation with 98 °C reference (371 K)
        ((15000.0 / 371.0) - (15000.0 / (theta_h + 273.0))).exp()
    }

    /// Loss of life consumed per hour \[%/h\] at current operating conditions.
    ///
    /// Assumes normal insulation life L_n = 8.76 × 10⁵ h (per IEC 60076-7).
    /// `LoL [%/h] = FAA / L_n × 100`
    pub fn loss_of_life_pct_per_hour(&self) -> f64 {
        const NORMAL_INSULATION_LIFE_H: f64 = 8.76e5; // hours
        self.aging_acceleration_factor() / NORMAL_INSULATION_LIFE_H * 100.0
    }

    /// Estimate time \[minutes\] until the hotspot reaches `max_temp_c` \[°C\].
    ///
    /// Uses the current rate of temperature change (single-step look-ahead).
    /// Returns `f64::INFINITY` if the transformer is already cooling toward
    /// the limit or if it would never reach it.
    pub fn time_to_limit(&self, max_temp_c: f64) -> f64 {
        let current_hs = self.hotspot_temperature();
        if current_hs >= max_temp_c {
            return 0.0;
        }
        // Approximate: assume constant rate of rise from present state
        // Use small probe step to compute dHS/dt
        let dt_probe = 0.1_f64; // 0.1 min probe
        let mut probe = self.clone();
        // Use load factor 1.0 (rated) as conservative estimate
        probe.step(1.0, dt_probe);
        let rate = (probe.hotspot_temperature() - current_hs) / dt_probe; // °C/min
        if rate <= 0.0 {
            f64::INFINITY
        } else {
            (max_temp_c - current_hs) / rate
        }
    }

    /// Steady-state hotspot temperature at rated load \[°C\].
    ///
    /// `θ_H_rated = θ_a + Δθ_or + Δθ_hr`
    pub fn rated_hotspot_temperature(&self) -> f64 {
        self.theta_a + self.theta_or + self.theta_hr
    }
}

// ---------------------------------------------------------------------------
// Dissolved Gas Analysis (DGA)
// ---------------------------------------------------------------------------

/// Dissolved gas concentrations in transformer oil \[ppm by volume\].
///
/// Used for condition monitoring per IEC 60599 / IEEE C57.104.
/// Rogers ratio analysis identifies developing faults from key gas ratios.
#[derive(Debug, Clone)]
pub struct DgaModel {
    /// Hydrogen (H₂) \[ppm\].
    pub h2_ppm: f64,
    /// Methane (CH₄) \[ppm\].
    pub ch4_ppm: f64,
    /// Ethane (C₂H₆) \[ppm\].
    pub c2h6_ppm: f64,
    /// Ethylene (C₂H₄) \[ppm\].
    pub c2h4_ppm: f64,
    /// Acetylene (C₂H₂) \[ppm\].
    pub c2h2_ppm: f64,
    /// Carbon monoxide (CO) \[ppm\] — indicator of cellulose degradation.
    pub co_ppm: f64,
    /// Carbon dioxide (CO₂) \[ppm\] — indicator of cellulose oxidation.
    pub co2_ppm: f64,
}

/// Fault type identified by DGA Rogers ratio analysis (IEC 60599 Table 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DgaFaultType {
    /// Gas concentrations within normal limits — no fault indication.
    Normal,
    /// Thermal fault in cellulose (paper) insulation (T < 300 °C).
    PaperThermal,
    /// Thermal fault in oil, 300 °C – 700 °C range.
    OilThermal300to700C,
    /// Thermal fault in oil, above 700 °C.
    OilThermalAbove700C,
    /// Partial discharge (low-energy electrical fault).
    PartialDischarge,
    /// High-energy electrical discharge (arcing).
    Arcing,
    /// Ratio combination not covered by standard code table.
    Unknown,
}

impl fmt::Display for DgaFaultType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            DgaFaultType::Normal => "Normal",
            DgaFaultType::PaperThermal => "Thermal (paper <300°C)",
            DgaFaultType::OilThermal300to700C => "Thermal (oil 300-700°C)",
            DgaFaultType::OilThermalAbove700C => "Thermal (oil >700°C)",
            DgaFaultType::PartialDischarge => "Partial Discharge",
            DgaFaultType::Arcing => "Arcing",
            DgaFaultType::Unknown => "Unknown",
        };
        write!(f, "{s}")
    }
}

impl DgaModel {
    /// Create a DGA model with typical new-oil baseline concentrations.
    ///
    /// Values chosen so Rogers ratios (0,0,0) → Normal classification:
    /// - R1 = CH4/H2 = 2/50 = 0.04 < 0.1 → code 0
    /// - R2 = C2H4/C2H6 = 0.5/1.0 = 0.5 < 1.0 → code 0
    /// - R3 = C2H2/C2H4 = 0/0.5 = 0.0 < 0.1 → code 0
    pub fn new_baseline() -> Self {
        Self {
            h2_ppm: 50.0,
            ch4_ppm: 2.0,
            c2h6_ppm: 1.0,
            c2h4_ppm: 0.5,
            c2h2_ppm: 0.0,
            co_ppm: 100.0,
            co2_ppm: 500.0,
        }
    }

    /// Rogers ratio fault classification per IEC 60599 / IEEE C57.104.
    ///
    /// Three dimensionless ratios are computed:
    /// - R1 = CH₄ / H₂
    /// - R2 = C₂H₄ / C₂H₆
    /// - R3 = C₂H₂ / C₂H₄
    ///
    /// Each ratio is coded 0, 1, or 2 and the triplet (R1_code, R2_code, R3_code)
    /// maps to a fault type.
    pub fn rogers_ratio_code(&self) -> DgaFaultType {
        // Avoid divide-by-zero for zero denominators
        let r1 = if self.h2_ppm > 0.0 {
            self.ch4_ppm / self.h2_ppm
        } else {
            f64::INFINITY
        };
        let r2 = if self.c2h6_ppm > 0.0 {
            self.c2h4_ppm / self.c2h6_ppm
        } else {
            f64::INFINITY
        };
        let r3 = if self.c2h4_ppm > 0.0 {
            self.c2h2_ppm / self.c2h4_ppm
        } else {
            f64::INFINITY
        };

        // Encode each ratio to Rogers code digit
        let c1 = encode_r1(r1);
        let c2 = encode_r2(r2);
        let c3 = encode_r3(r3);

        // IEC 60599 Table 2 lookup
        match (c1, c2, c3) {
            (0, 0, 0) => DgaFaultType::Normal,
            (0, 0, 1) | (0, 0, 2) => DgaFaultType::PartialDischarge,
            (1, 0, 0) => DgaFaultType::PaperThermal,
            (2, 0, 0) => DgaFaultType::OilThermal300to700C,
            (2, 2, 0) => DgaFaultType::OilThermalAbove700C,
            (0, 1, 0) | (1, 1, 0) => DgaFaultType::OilThermal300to700C,
            (0, 2, 0) | (2, 1, 0) => DgaFaultType::OilThermalAbove700C,
            (0, 1, 1) | (0, 2, 2) => DgaFaultType::Arcing,
            (0, 1, 2) | (1, 1, 2) | (2, 1, 2) => DgaFaultType::Arcing,
            _ => DgaFaultType::Unknown,
        }
    }

    /// Total combustible gas (TCG) content \[ppm\].
    ///
    /// TCG = H₂ + CH₄ + C₂H₆ + C₂H₄ + C₂H₂ + CO
    /// per IEEE C57.104 Section 7.
    pub fn total_combustible_gas(&self) -> f64 {
        self.h2_ppm + self.ch4_ppm + self.c2h6_ppm + self.c2h4_ppm + self.c2h2_ppm + self.co_ppm
    }

    /// Simplified health index on a 0–100 scale (100 = perfect, 0 = failed).
    ///
    /// Combines TCG normalised against a 2000 ppm reference and acetylene
    /// as a high-weight indicator of arcing faults.
    ///
    /// This is a heuristic indicator only — full DGA assessment per IEC 60599
    /// requires trending and additional data.
    pub fn health_index(&self) -> f64 {
        const TCG_REFERENCE: f64 = 2000.0; // ppm — L3 limit per IEEE C57.104
        const C2H2_LIMIT: f64 = 35.0; // ppm — L1 alarm IEEE C57.104

        let tcg_score = 1.0 - (self.total_combustible_gas() / TCG_REFERENCE).min(1.0);
        let arc_score = 1.0 - (self.c2h2_ppm / C2H2_LIMIT).min(1.0);

        // Weighted combination: TCG 60 %, acetylene 40 %
        let raw = 0.6 * tcg_score + 0.4 * arc_score;
        (raw * 100.0).clamp(0.0, 100.0)
    }
}

/// Encode CH₄/H₂ ratio `r1` to Rogers code digit (0, 1, or 2).
fn encode_r1(r: f64) -> u8 {
    if r < 0.1 {
        0
    } else if r < 1.0 {
        1
    } else {
        2
    }
}

/// Encode C₂H₄/C₂H₆ ratio `r2` to Rogers code digit (0, 1, or 2).
fn encode_r2(r: f64) -> u8 {
    if r < 1.0 {
        0
    } else if r < 3.0 {
        1
    } else {
        2
    }
}

/// Encode C₂H₂/C₂H₄ ratio `r3` to Rogers code digit (0, 1, or 2).
fn encode_r3(r: f64) -> u8 {
    if r < 0.1 {
        0
    } else if r < 3.0 {
        1
    } else {
        2
    }
}

// ---------------------------------------------------------------------------
// Three-winding transformer
// ---------------------------------------------------------------------------

/// Three-winding power transformer model.
///
/// Converted to the classical star (T) equivalent consisting of three two-winding
/// transformers sharing an ideal (fictitious) internal node. The star equivalent
/// allows standard two-winding Y-bus stamps for each winding.
///
/// # Star-equivalent impedance allocation
/// Given open-circuit short-circuit tests between pairs of windings:
/// - Z_HM (HV-MV, LV open): `z_hm = r_hm + j·x_hm`
/// - Z_HL (HV-LV, MV open): `z_hl = r_hl + j·x_hl`
/// - Z_ML (MV-LV, HV open): `z_ml = r_ml + j·x_ml`
///
/// Star branch impedances (all referred to HV base):
/// ```text
/// Z_H = (Z_HM + Z_HL − Z_ML) / 2
/// Z_M = (Z_HM + Z_ML − Z_HL) / 2
/// Z_L = (Z_HL + Z_ML − Z_HM) / 2
/// ```
///
/// In this simplified model the per-winding `r` and `x` are provided directly
/// (already split) and used to build the star branches.
#[derive(Debug, Clone)]
pub struct ThreeWindingTransformer {
    /// Unique identifier.
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// Nameplate MVA of HV winding (reference for all pu quantities).
    pub rated_mva_hv: f64,
    /// Nameplate MVA of MV winding.
    pub rated_mva_mv: f64,
    /// Nameplate MVA of LV winding.
    pub rated_mva_lv: f64,
    /// Rated voltage of HV winding \[kV\].
    pub v_hv_kv: f64,
    /// Rated voltage of MV winding \[kV\].
    pub v_mv_kv: f64,
    /// Rated voltage of LV winding \[kV\].
    pub v_lv_kv: f64,
    /// HV winding series resistance \[pu on HV MVA base\].
    pub r_hv_pu: f64,
    /// HV winding leakage reactance \[pu on HV MVA base\].
    pub x_hv_pu: f64,
    /// MV winding series resistance \[pu on HV MVA base\].
    pub r_mv_pu: f64,
    /// MV winding leakage reactance \[pu on HV MVA base\].
    pub x_mv_pu: f64,
    /// LV winding series resistance \[pu on HV MVA base\].
    pub r_lv_pu: f64,
    /// LV winding leakage reactance \[pu on HV MVA base\].
    pub x_lv_pu: f64,
    /// HV winding off-nominal tap \[pu\].
    pub tap_hv: f64,
    /// MV winding off-nominal tap \[pu\].
    pub tap_mv: f64,
    /// LV winding off-nominal tap \[pu\].
    pub tap_lv: f64,
}

impl ThreeWindingTransformer {
    /// Construct with default unity taps.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: usize,
        name: impl Into<String>,
        rated_mva_hv: f64,
        rated_mva_mv: f64,
        rated_mva_lv: f64,
        v_hv_kv: f64,
        v_mv_kv: f64,
        v_lv_kv: f64,
        r_hv_pu: f64,
        x_hv_pu: f64,
        r_mv_pu: f64,
        x_mv_pu: f64,
        r_lv_pu: f64,
        x_lv_pu: f64,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            rated_mva_hv,
            rated_mva_mv,
            rated_mva_lv,
            v_hv_kv,
            v_mv_kv,
            v_lv_kv,
            r_hv_pu,
            x_hv_pu,
            r_mv_pu,
            x_mv_pu,
            r_lv_pu,
            x_lv_pu,
            tap_hv: 1.0,
            tap_mv: 1.0,
            tap_lv: 1.0,
        }
    }

    /// Convert to a star (T) equivalent consisting of three [`TransformerModel`]s.
    ///
    /// The returned array is `[HV branch, MV branch, LV branch]`. Each branch
    /// connects the respective terminal bus to a fictitious internal (star) node.
    /// The HV branch carries the magnetising branch; MV and LV branches are purely
    /// series elements.
    ///
    /// All three share `id = self.id`; the caller should assign unique branch IDs
    /// as needed.
    pub fn to_star_equivalent(&self) -> [TransformerModel; 3] {
        // HV branch: HV bus → star node
        let mut hv = TransformerModel::new(
            self.id,
            format!("{}_HV", self.name),
            self.rated_mva_hv,
            self.v_hv_kv,
            self.v_hv_kv, // star node has HV kV base by convention
            self.r_hv_pu,
            self.x_hv_pu,
        );
        hv.tap_ratio = self.tap_hv;

        // MV branch: star node → MV bus
        // Scale impedance by MVA ratio to keep consistent per-unit system
        let mva_ratio_mv = self.rated_mva_hv / self.rated_mva_mv.max(f64::EPSILON);
        let mut mv = TransformerModel::new(
            self.id,
            format!("{}_MV", self.name),
            self.rated_mva_mv,
            self.v_hv_kv, // star node
            self.v_mv_kv,
            self.r_mv_pu / mva_ratio_mv,
            self.x_mv_pu / mva_ratio_mv,
        );
        mv.tap_ratio = self.tap_mv;

        // LV branch: star node → LV bus
        let mva_ratio_lv = self.rated_mva_hv / self.rated_mva_lv.max(f64::EPSILON);
        let mut lv = TransformerModel::new(
            self.id,
            format!("{}_LV", self.name),
            self.rated_mva_lv,
            self.v_hv_kv, // star node
            self.v_lv_kv,
            self.r_lv_pu / mva_ratio_lv,
            self.x_lv_pu / mva_ratio_lv,
        );
        lv.tap_ratio = self.tap_lv;

        [hv, mv, lv]
    }

    /// Rated current on the HV winding \[kA\].
    pub fn rated_current_hv_ka(&self) -> f64 {
        self.rated_mva_hv / (3.0_f64.sqrt() * self.v_hv_kv)
    }

    /// Rated current on the MV winding \[kA\].
    pub fn rated_current_mv_ka(&self) -> f64 {
        self.rated_mva_mv / (3.0_f64.sqrt() * self.v_mv_kv)
    }

    /// Rated current on the LV winding \[kA\].
    pub fn rated_current_lv_ka(&self) -> f64 {
        self.rated_mva_lv / (3.0_f64.sqrt() * self.v_lv_kv)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    // -----------------------------------------------------------------------
    // Y-matrix tests
    // -----------------------------------------------------------------------

    /// For an ideal transformer (no shunt, no tap deviation, zero phase shift),
    /// Y_ii should equal the series admittance.
    #[test]
    fn test_y_matrix_ideal_no_tap() {
        let mut t = TransformerModel::new(0, "T1", 100.0, 110.0, 11.0, 0.005, 0.1);
        t.tap_ratio = 1.0;
        t.phase_shift_deg = 0.0;

        let y = t.y_matrix_elements();
        let y_s = Complex::new(0.005, 0.1).inv();

        // y_ii = (y_s + 0) / 1.0^2 = y_s
        assert!((y.y_ii.re - y_s.re).abs() < TOL);
        assert!((y.y_ii.im - y_s.im).abs() < TOL);
        // y_jj = y_s
        assert!((y.y_jj.re - y_s.re).abs() < TOL);
        // y_ij = -y_s / conj(1+0j) = -y_s
        assert!((y.y_ij.re + y_s.re).abs() < TOL);
        assert!((y.y_ij.im + y_s.im).abs() < TOL);
    }

    /// Off-nominal tap changes y_ij: y_ij = -y_s / conj(a).
    #[test]
    fn test_y_matrix_off_nominal_tap() {
        let mut t = TransformerModel::new(1, "T2", 100.0, 110.0, 11.0, 0.01, 0.12);
        t.tap_ratio = 1.05;
        t.phase_shift_deg = 0.0;

        let y = t.y_matrix_elements();
        let y_s = Complex::new(0.01_f64, 0.12_f64).inv();
        let expected_y_ij = -y_s / 1.05_f64; // conj(a) = a for real a

        assert!((y.y_ij.re - expected_y_ij.re).abs() < TOL);
        assert!((y.y_ij.im - expected_y_ij.im).abs() < TOL);
    }

    /// y_ii for off-nominal tap: (y_s + y_m) / |a|^2.
    #[test]
    fn test_y_matrix_yii_with_shunt_and_tap() {
        let mut t = TransformerModel::new(2, "T3", 100.0, 110.0, 11.0, 0.01, 0.1);
        t.tap_ratio = 1.1;
        t.g_fe_pu = 0.002;
        t.b_mag_pu = 0.04;
        t.phase_shift_deg = 0.0;

        let y = t.y_matrix_elements();
        let y_s = Complex::new(0.01_f64, 0.1_f64).inv();
        let y_m = Complex::new(0.002, -0.04);
        let expected = (y_s + y_m) / (1.1 * 1.1);

        assert!((y.y_ii.re - expected.re).abs() < 1e-8);
        assert!((y.y_ii.im - expected.im).abs() < 1e-8);
    }

    // -----------------------------------------------------------------------
    // Rated current tests
    // -----------------------------------------------------------------------

    /// Rated HV current: I = S / (√3 · V).
    #[test]
    fn test_rated_current_hv() {
        let t = TransformerModel::new(3, "T4", 100.0, 110.0, 11.0, 0.0, 0.1);
        let expected_ka = 100.0 / (3.0_f64.sqrt() * 110.0);
        assert!((t.rated_current_ka() - expected_ka).abs() < TOL);
        assert!((t.rated_current_amps_hv() - expected_ka * 1000.0).abs() < TOL);
    }

    /// Rated LV current: I_lv = S / (√3 · V_lv).
    #[test]
    fn test_rated_current_lv() {
        let t = TransformerModel::new(4, "T5", 100.0, 110.0, 11.0, 0.0, 0.1);
        let expected_a = 100.0 / (3.0_f64.sqrt() * 11.0) * 1000.0;
        assert!((t.rated_current_amps_lv() - expected_a).abs() < 1e-6);
    }

    // -----------------------------------------------------------------------
    // OLTC tests
    // -----------------------------------------------------------------------

    /// Voltage within deadband → NoChange.
    #[test]
    fn test_oltc_no_action_within_deadband() {
        let mut oltc = OltcController::new(0.85, 1.15, 0.00625, 1.0);
        let action = oltc.update(1.005, 60.0); // within ±0.01 deadband
        assert_eq!(action, OltcAction::NoChange);
        assert_eq!(oltc.total_tap_operations, 0);
    }

    /// Voltage below setpoint — after sufficient delay, tap raises.
    #[test]
    fn test_oltc_tap_up_after_delay() {
        let mut oltc = OltcController::new(0.85, 1.15, 0.00625, 1.0);
        oltc.delay_s = 30.0;

        // First call: voltage low but delay not expired
        let a1 = oltc.update(0.97, 20.0);
        assert_eq!(a1, OltcAction::NoChange);

        // Second call: delay expires
        let a2 = oltc.update(0.97, 15.0); // total 35 s > 30 s
        assert!(matches!(a2, OltcAction::TapUp(_)));
        assert_eq!(oltc.total_tap_operations, 1);
        assert!((oltc.current_tap - 1.00625).abs() < 1e-9);
    }

    /// Voltage above setpoint outside deadband → TapDown.
    #[test]
    fn test_oltc_tap_down_high_voltage() {
        let mut oltc = OltcController::new(0.85, 1.15, 0.00625, 1.0);
        oltc.delay_s = 5.0;

        let action = oltc.update(1.05, 10.0); // 10 s > 5 s delay
        assert!(matches!(action, OltcAction::TapDown(_)));
        assert_eq!(oltc.total_tap_operations, 1);
    }

    /// When already at tap_max with low voltage → AtLimit.
    #[test]
    fn test_oltc_at_limit_max() {
        let mut oltc = OltcController::new(0.85, 1.15, 0.00625, 1.0);
        oltc.current_tap = 1.15; // already at max
        oltc.delay_s = 1.0;

        let action = oltc.update(0.90, 5.0); // low voltage, delay expired
        assert_eq!(action, OltcAction::AtLimit);
        assert_eq!(oltc.total_tap_operations, 0);
    }

    /// When already at tap_min with high voltage → AtLimit.
    #[test]
    fn test_oltc_at_limit_min() {
        let mut oltc = OltcController::new(0.85, 1.15, 0.00625, 1.0);
        oltc.current_tap = 0.85; // already at min
        oltc.delay_s = 1.0;

        let action = oltc.update(1.10, 5.0); // high voltage
        assert_eq!(action, OltcAction::AtLimit);
    }

    // -----------------------------------------------------------------------
    // Thermal model tests
    // -----------------------------------------------------------------------

    /// Top-oil temperature increases when load factor > 1 (overload).
    #[test]
    fn test_thermal_step_increases_top_oil_under_overload() {
        let mut model = TransformerThermalModel::new_onan(25.0);
        let t0 = model.top_oil_temperature();
        model.step(1.2, 5.0); // 5 min at 120 % load
        assert!(model.top_oil_temperature() > t0);
    }

    /// Aging acceleration factor > 1 when hotspot > 98 °C.
    #[test]
    fn test_thermal_aging_factor_above_reference() {
        let mut model = TransformerThermalModel::new_onan(25.0);
        // Drive to rated hotspot temperature
        model.theta_o = 80.0;
        model.delta_theta_h = 30.0; // hotspot = 110 °C > 98 °C
        assert!(model.aging_acceleration_factor() > 1.0);
    }

    /// At rated steady-state: hotspot ≈ θ_a + θ_or + θ_hr.
    #[test]
    fn test_thermal_steady_state_hotspot() {
        let theta_a = 20.0;
        let mut model = TransformerThermalModel::new_onan(theta_a);
        // Simulate to steady state: many steps at K=1
        for _ in 0..5000 {
            model.step(1.0, 1.0); // 1 min steps for 5000 min
        }
        let expected = theta_a + model.theta_or + model.theta_hr; // = 20+55+23 = 98
        let actual = model.hotspot_temperature();
        assert!(
            (actual - expected).abs() < 0.5,
            "hotspot {actual} vs expected {expected}"
        );
    }

    /// Aging factor = 1.0 exactly at 98 °C (reference hotspot).
    #[test]
    fn test_thermal_aging_factor_unity_at_reference() {
        let mut model = TransformerThermalModel::new_onan(20.0);
        // 98 °C hotspot: theta_o = 75 °C, delta_theta_h = 23 °C
        model.theta_o = 75.0;
        model.delta_theta_h = 23.0;
        let faa = model.aging_acceleration_factor();
        assert!((faa - 1.0).abs() < 1e-6, "FAA at 98°C = {faa}");
    }

    // -----------------------------------------------------------------------
    // DGA tests
    // -----------------------------------------------------------------------

    /// Normal gas levels → Normal fault classification.
    #[test]
    fn test_dga_normal_gases() {
        let dga = DgaModel::new_baseline();
        assert_eq!(dga.rogers_ratio_code(), DgaFaultType::Normal);
    }

    /// High acetylene (C₂H₂) relative to C₂H₄ → Arcing.
    #[test]
    fn test_dga_high_acetylene_arcing() {
        let mut dga = DgaModel::new_baseline();
        dga.c2h2_ppm = 100.0; // C2H2/C2H4 = 100/0.5 = 200 → code 2
        dga.c2h4_ppm = 0.5;
        dga.c2h6_ppm = 0.5; // C2H4/C2H6 = 1.0 → code 1
        dga.h2_ppm = 50.0; // CH4/H2 = 2/50 = 0.04 → code 0
        dga.ch4_ppm = 2.0;
        // (c1=0, c2=1, c3=2) → Arcing
        assert_eq!(dga.rogers_ratio_code(), DgaFaultType::Arcing);
    }

    /// Health index is 100 for near-zero gas concentrations.
    #[test]
    fn test_dga_health_index_near_perfect() {
        let dga = DgaModel {
            h2_ppm: 0.0,
            ch4_ppm: 0.0,
            c2h6_ppm: 0.0,
            c2h4_ppm: 0.0,
            c2h2_ppm: 0.0,
            co_ppm: 0.0,
            co2_ppm: 0.0,
        };
        assert!((dga.health_index() - 100.0).abs() < TOL);
    }

    /// Health index is 0 (or near zero) for severely contaminated oil.
    #[test]
    fn test_dga_health_index_degraded() {
        let dga = DgaModel {
            h2_ppm: 1000.0,
            ch4_ppm: 500.0,
            c2h6_ppm: 100.0,
            c2h4_ppm: 100.0,
            c2h2_ppm: 200.0, // >> C2H2 limit → arc_score → 0
            co_ppm: 500.0,
            co2_ppm: 2000.0,
        };
        // arc_score = 0 (c2h2 >= limit), tcg_score ≈ 0 → health ~= 0
        assert!(dga.health_index() < 5.0);
    }

    // -----------------------------------------------------------------------
    // Three-winding transformer tests
    // -----------------------------------------------------------------------

    /// to_star_equivalent returns exactly 3 TransformerModel objects.
    #[test]
    fn test_three_winding_star_equivalent_count() {
        let tw = ThreeWindingTransformer::new(
            0, "3W", 100.0, 50.0, 30.0, 220.0, 110.0, 10.0, 0.005, 0.08, 0.005, 0.08, 0.005, 0.08,
        );
        let branches = tw.to_star_equivalent();
        assert_eq!(branches.len(), 3);
    }

    /// HV rated current formula: I_hv = S_hv / (√3 · V_hv).
    #[test]
    fn test_three_winding_rated_current_hv() {
        let tw = ThreeWindingTransformer::new(
            1, "3W2", 300.0, 150.0, 100.0, 400.0, 220.0, 33.0, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1,
        );
        let expected = 300.0 / (3.0_f64.sqrt() * 400.0);
        assert!((tw.rated_current_hv_ka() - expected).abs() < TOL);
    }

    /// Star branches carry correct winding voltages.
    #[test]
    fn test_three_winding_star_branch_voltages() {
        let tw = ThreeWindingTransformer::new(
            2, "3W3", 100.0, 60.0, 40.0, 110.0, 33.0, 11.0, 0.01, 0.1, 0.01, 0.1, 0.01, 0.1,
        );
        let [hv_br, mv_br, lv_br] = tw.to_star_equivalent();
        assert!((hv_br.v_hv_kv - 110.0).abs() < TOL);
        assert!((mv_br.v_lv_kv - 33.0).abs() < TOL);
        assert!((lv_br.v_lv_kv - 11.0).abs() < TOL);
    }

    /// Phase shift for Dyn11 vector group is 30°.
    #[test]
    fn test_vector_group_phase_shift() {
        assert!((VectorGroup::Dyn11.phase_shift_deg() - 30.0).abs() < TOL);
        assert!((VectorGroup::Yny0.phase_shift_deg() - 0.0).abs() < TOL);
    }

    /// Phase-shifting transformer: y_ij ≠ y_ji (asymmetric off-diagonal).
    #[test]
    fn test_y_matrix_phase_shift_asymmetry() {
        let mut t = TransformerModel::new(5, "PST", 100.0, 110.0, 110.0, 0.0, 0.1);
        t.tap_ratio = 1.0;
        t.phase_shift_deg = 30.0;

        let y = t.y_matrix_elements();
        // y_ij = -y_s / conj(a), y_ji = -y_s / a
        // Since a is not real, y_ij ≠ y_ji
        assert!((y.y_ij - y.y_ji).norm() > 1e-6);
    }
}
