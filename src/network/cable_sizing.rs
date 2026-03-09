//! Cable sizing, thermal rating, and selection module.
//!
//! Implements IEC 60287 (electric cables — calculation of current rating) and
//! IEC 60364 (low-voltage electrical installations) sizing methods for power
//! cables across LV, MV, and HV voltage classes.
//!
//! # Key capabilities
//! - Thermal current rating with derating for temperature, soil resistivity, and
//!   cable grouping (IEC 60287-2-1).
//! - Standard cable database covering LV 400 V through HV 110 kV.
//! - Automatic cable selection given a required power transfer and route length.
//! - Voltage-drop and short-circuit withstand checks.
//! - Economic evaluation (capital cost + capitalised losses).

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Insulation type of the cable, which determines maximum conductor temperature
/// and dielectric loss characteristics.
#[derive(Debug, Clone, PartialEq)]
pub enum CableInsulation {
    /// Cross-linked polyethylene — 90 °C max conductor temperature.
    Xlpe,
    /// Ethylene propylene rubber — 90 °C max, good flexibility.
    Epr,
    /// PVC for low-voltage (≤ 1 kV) — 70 °C max.
    PvcLv,
    /// PVC for medium-voltage (≤ 36 kV) — 70 °C max.
    PvcMv,
    /// Mass-impregnated paper — 85 °C max (traditional HV cables).
    MiPaper,
    /// Low Smoke Zero Halogen (LSZH) — 90 °C max, fire-safe environments.
    Lsoh,
}

impl CableInsulation {
    /// Maximum allowable conductor temperature in °C for this insulation type.
    pub fn max_temp_c(&self) -> f64 {
        match self {
            Self::Xlpe | Self::Epr | Self::Lsoh => 90.0,
            Self::PvcLv | Self::PvcMv => 70.0,
            Self::MiPaper => 85.0,
        }
    }
}

/// Method by which the cable is installed, determining the base thermal rating
/// and applicable derating factors from IEC 60287-2 tables.
#[derive(Debug, Clone, PartialEq)]
pub enum InstallationMethod {
    /// Cable buried directly in the ground (most common MV/HV method).
    DirectBuried,
    /// Cable pulled through a protective duct or conduit in the ground.
    InDuct,
    /// Cable in free air (e.g., on a cable ladder in a substation).
    InAir,
    /// Cable laid on a cable tray or rack, no enclosure.
    OnTray,
    /// Submarine cable on the seabed or buried below the seabed.
    Underwater,
    /// Cable clipped directly to a wall or structure surface.
    ClipDirect,
    /// Tree cable (aerial bundled conductor), self-supporting.
    TreedCable,
}

/// Conductor material, which sets resistivity and temperature coefficient.
#[derive(Debug, Clone, PartialEq)]
pub enum ConductorMaterial {
    /// Solid or stranded plain copper (most common for MV/HV).
    Copper,
    /// Solid or stranded aluminium (cost-effective for large LV cables).
    Aluminum,
    /// Copper-clad aluminium — intermediate properties.
    CopperClad,
    /// Fine stranded copper for flexible applications.
    StrandedCopper,
    /// Extra-flexible fine stranded copper (e.g., trailing cables).
    FlexibleStranded,
}

impl ConductorMaterial {
    /// Temperature coefficient of resistance (1/°C) at 20 °C.
    pub fn alpha(&self) -> f64 {
        match self {
            Self::Copper | Self::StrandedCopper | Self::FlexibleStranded => 0.003_93,
            Self::Aluminum => 0.004_03,
            Self::CopperClad => 0.003_98, // intermediate
        }
    }

    /// IEC 60287 k-factor for short-circuit withstand (Cu-XLPE basis).
    /// Returns `(k_cu_xlpe, k_al_xlpe)` pair; caller picks the appropriate one.
    pub fn k_factor(&self) -> f64 {
        match self {
            Self::Copper | Self::StrandedCopper | Self::FlexibleStranded => 143.0,
            Self::Aluminum => 94.0,
            Self::CopperClad => 120.0, // conservative intermediate
        }
    }
}

/// Nominal voltage class for the cable circuit.
#[derive(Debug, Clone, PartialEq, Copy)]
pub enum VoltageClass {
    /// Low voltage, 400 V (0.4 kV).
    Lv400v,
    /// Medium voltage, 6 kV.
    Mv6kv,
    /// Medium voltage, 10 kV.
    Mv10kv,
    /// Medium voltage, 20 kV.
    Mv20kv,
    /// Medium voltage, 33 kV.
    Mv33kv,
    /// High voltage, 66 kV.
    Hv66kv,
    /// High voltage, 110 kV.
    Hv110kv,
    /// High voltage, 220 kV.
    Hv220kv,
    /// High voltage, 400 kV (EHV).
    Hv400kv,
}

impl VoltageClass {
    /// Nominal line-to-line voltage in kV.
    pub fn voltage_kv(&self) -> f64 {
        match self {
            Self::Lv400v => 0.4,
            Self::Mv6kv => 6.0,
            Self::Mv10kv => 10.0,
            Self::Mv20kv => 20.0,
            Self::Mv33kv => 33.0,
            Self::Hv66kv => 66.0,
            Self::Hv110kv => 110.0,
            Self::Hv220kv => 220.0,
            Self::Hv400kv => 400.0,
        }
    }

    /// Select the voltage class closest to the given line voltage in kV.
    pub fn from_kv(kv: f64) -> Self {
        // ordered table: (upper boundary, class)
        let table: &[(f64, VoltageClass)] = &[
            (0.6, Self::Lv400v),
            (8.0, Self::Mv6kv),
            (15.0, Self::Mv10kv),
            (26.0, Self::Mv20kv),
            (50.0, Self::Mv33kv),
            (88.0, Self::Hv66kv),
            (160.0, Self::Hv110kv),
            (300.0, Self::Hv220kv),
        ];
        for &(upper, ref vc) in table {
            if kv <= upper {
                return *vc;
            }
        }
        Self::Hv400kv
    }
}

// ---------------------------------------------------------------------------
// CableSpec
// ---------------------------------------------------------------------------

/// Full technical specification of a power cable.
///
/// Data values follow IEC 60228 conductor cross-sections, IEC 60502 MV cables,
/// and IEC 60840 / IEC 62067 HV cables.
#[derive(Debug, Clone)]
pub struct CableSpec {
    /// Unique catalogue identifier.
    pub id: usize,
    /// Descriptive name (e.g., `"3×185 mm² Cu/XLPE 10 kV"`).
    pub name: String,
    /// Voltage class for which this cable is designed.
    pub voltage_class: VoltageClass,
    /// Conductor material.
    pub conductor_material: ConductorMaterial,
    /// Insulation type.
    pub insulation: CableInsulation,
    /// Conductor cross-section area in mm².
    pub conductor_cross_section_mm2: f64,
    /// Number of power-carrying cores (1, 3, or 4).
    pub n_cores: u8,
    /// AC resistance at 20 °C in Ω/km.
    pub resistivity_ohm_per_km: f64,
    /// Positive-sequence reactance in Ω/km.
    pub reactance_ohm_per_km: f64,
    /// Shunt capacitance in nF/km.
    pub capacitance_nf_per_km: f64,
    /// Maximum allowable conductor temperature in °C (set by insulation).
    pub max_conductor_temp_c: f64,
    /// Base thermal current rating in A at standard reference conditions
    /// (20 °C soil / 30 °C air, ρ_soil = 1.0 K·m/W, single circuit).
    pub rated_current_a: f64,
    /// Cable unit weight in kg/m (jacket + armour + conductor).
    pub weight_kg_per_m: f64,
    /// Overall outer diameter in mm.
    pub outer_diameter_mm: f64,
}

impl CableSpec {
    /// Approximate cost of the cable in USD/m, based on copper price and
    /// cross-section heuristic.  Intended only for order-of-magnitude comparisons.
    pub fn cost_usd_per_m(&self) -> f64 {
        // LV cables ~5–15 USD/m, MV ~20–80, HV ~100–600
        let base = match self.voltage_class {
            VoltageClass::Lv400v => 5.0,
            VoltageClass::Mv6kv | VoltageClass::Mv10kv => 20.0,
            VoltageClass::Mv20kv | VoltageClass::Mv33kv => 35.0,
            VoltageClass::Hv66kv => 80.0,
            VoltageClass::Hv110kv => 130.0,
            VoltageClass::Hv220kv => 250.0,
            VoltageClass::Hv400kv => 500.0,
        };
        base + self.conductor_cross_section_mm2 * 0.05
    }
}

// ---------------------------------------------------------------------------
// InstallationConditions
// ---------------------------------------------------------------------------

/// Environmental and installation parameters used in IEC 60287 thermal rating.
#[derive(Debug, Clone)]
pub struct InstallationConditions {
    /// Installation method (direct burial, duct, air, etc.).
    pub method: InstallationMethod,
    /// Ambient temperature in °C (soil temperature for buried, air for aerial).
    pub ambient_temp_c: f64,
    /// Soil thermal resistivity in K·m/W (typical 0.7–2.5; reference 1.0).
    pub soil_thermal_resistivity_k_m_per_w: f64,
    /// Burial depth to cable centreline in metres (used for grouping geometry).
    pub depth_of_lay_m: f64,
    /// Explicit grouping derating factor override.  Set to 0.0 to use automatic
    /// lookup from `n_cables_in_group`.
    pub grouping_factor: f64,
    /// Number of cables in the installation group (for automatic derating).
    pub n_cables_in_group: usize,
}

impl Default for InstallationConditions {
    fn default() -> Self {
        Self {
            method: InstallationMethod::DirectBuried,
            ambient_temp_c: 20.0,
            soil_thermal_resistivity_k_m_per_w: 1.0,
            depth_of_lay_m: 0.8,
            grouping_factor: 0.0, // 0 → automatic
            n_cables_in_group: 1,
        }
    }
}

impl InstallationConditions {
    /// Reference ambient temperature for this installation method (°C).
    ///
    /// IEC 60287 uses 20 °C for buried installations and 30 °C for in-air.
    pub fn reference_temp_c(&self) -> f64 {
        match self.method {
            InstallationMethod::InAir
            | InstallationMethod::OnTray
            | InstallationMethod::ClipDirect
            | InstallationMethod::TreedCable => 30.0,
            _ => 20.0,
        }
    }
}

// ---------------------------------------------------------------------------
// ThermalRating
// ---------------------------------------------------------------------------

/// Corrected thermal current rating after applying all IEC 60287 derating
/// factors to the base cable rating.
#[derive(Debug, Clone)]
pub struct ThermalRating {
    /// Thermally corrected current rating in A.
    pub rated_current_a: f64,
    /// Three-phase rated power (MW) at unity power factor.
    pub rated_power_mw: f64,
    /// Derating factor for ambient temperature deviation from reference.
    pub derating_factor_temp: f64,
    /// Derating factor for soil thermal resistivity.
    pub derating_factor_soil: f64,
    /// Derating factor for cable grouping (mutual heating).
    pub derating_factor_grouping: f64,
    /// Combined derating factor = product of all individual factors.
    pub total_derating_factor: f64,
    /// Conductor temperature at rated current after derating (≈ max_conductor_temp_c).
    pub conductor_temp_at_rated_c: f64,
    /// Joule losses in the conductor at rated current in W/m.
    pub joule_losses_w_per_m: f64,
}

// ---------------------------------------------------------------------------
// CableSizingResult
// ---------------------------------------------------------------------------

/// Complete cable selection result from [`CableSizingEngine::size_cable`].
#[derive(Debug, Clone)]
pub struct CableSizingResult {
    /// The cable type selected from the database.
    pub selected_cable: CableSpec,
    /// Thermal rating under the specified installation conditions.
    pub thermal_rating: ThermalRating,
    /// Number of parallel cable circuits required to carry the load.
    pub n_parallel_circuits: usize,
    /// Total route length in km (single circuit length; parallel circuits share route).
    pub total_length_km: f64,
    /// Estimated total capital cost in USD (cable supply only).
    pub total_capital_cost_usd: f64,
    /// Annual energy losses in MWh/year (Joule losses at full load, 8760 h).
    pub annual_losses_mwh: f64,
    /// Annualised cost of losses in USD/year at the project energy price.
    pub annual_loss_cost_usd: f64,
    /// Voltage drop percentage at rated current and power factor 0.9.
    pub voltage_drop_pct: f64,
    /// Maximum short-circuit current the cable can withstand for 1 s in kA.
    pub short_circuit_withstand_ka: f64,
    /// `true` if the selection meets voltage-drop and SC withstand criteria.
    pub recommended: bool,
}

// ---------------------------------------------------------------------------
// CableDatabase
// ---------------------------------------------------------------------------

/// Catalogue of standard power cables from which the sizing engine selects.
///
/// Populated by [`CableSizingEngine::generate_standard_database`] with typical
/// cables covering LV 400 V through HV 110 kV.
#[derive(Debug, Clone, Default)]
pub struct CableDatabase {
    /// All cable specifications in the catalogue.
    pub cables: Vec<CableSpec>,
}

impl CableDatabase {
    /// Return all cables whose voltage class matches `vc`.
    pub fn find_cables_by_voltage_class(&self, vc: VoltageClass) -> Vec<&CableSpec> {
        self.cables
            .iter()
            .filter(|c| c.voltage_class == vc)
            .collect()
    }

    /// Return the smallest cable in `voltage_class` whose base rated current
    /// is at least `required_current_a`.  Returns `None` if no cable in the
    /// database can carry the required current (caller must parallel).
    pub fn find_minimum_size(
        &self,
        required_current_a: f64,
        voltage_class: VoltageClass,
    ) -> Option<&CableSpec> {
        let mut candidates: Vec<&CableSpec> = self
            .cables
            .iter()
            .filter(|c| c.voltage_class == voltage_class && c.rated_current_a >= required_current_a)
            .collect();
        // Sort by cross-section ascending — cheapest sufficient cable.
        candidates.sort_by(|a, b| {
            a.conductor_cross_section_mm2
                .partial_cmp(&b.conductor_cross_section_mm2)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        candidates.into_iter().next()
    }

    /// Return the largest cable available in the database for a given voltage class.
    fn largest_cable(&self, voltage_class: VoltageClass) -> Option<&CableSpec> {
        self.cables
            .iter()
            .filter(|c| c.voltage_class == voltage_class)
            .max_by(|a, b| {
                a.conductor_cross_section_mm2
                    .partial_cmp(&b.conductor_cross_section_mm2)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }
}

// ---------------------------------------------------------------------------
// CableSizingEngine — helper functions
// ---------------------------------------------------------------------------

/// Grouping derating factor for `n` cables installed in close proximity.
///
/// Values from IEC 60287-2-1 Table B.1 (flat formation, touching).
fn grouping_factor(n: usize) -> f64 {
    match n {
        0 | 1 => 1.00,
        2 => 0.85,
        3 => 0.79,
        4 => 0.75,
        5 => 0.72,
        _ => 0.68,
    }
}

/// Resistance at operating temperature `t_op` using IEC 60228 formula.
///
/// `R(T) = R_20 * (1 + α * (T - 20))`
fn resistance_at_temp(r_20: f64, alpha: f64, t_op: f64) -> f64 {
    r_20 * (1.0 + alpha * (t_op - 20.0))
}

// ---------------------------------------------------------------------------
// CableSizingEngine
// ---------------------------------------------------------------------------

/// Main cable sizing and thermal rating engine.
///
/// Implements the IEC 60287 current-rating methodology plus economic evaluation.
/// Typical usage:
///
/// ```rust
/// use oxigrid::network::cable_sizing::{CableSizingEngine, InstallationConditions};
///
/// let engine = CableSizingEngine::new();
/// let cond   = InstallationConditions::default();
/// let result = engine.size_cable(2.0, 10.0, 5.0, &cond);
/// assert!(result.recommended);
/// ```
#[derive(Debug, Clone)]
pub struct CableSizingEngine {
    /// Cable catalogue used for selection.
    pub database: CableDatabase,
    /// Energy price for loss capitalisation (USD/MWh).
    pub energy_price_usd_per_mwh: f64,
    /// Project lifetime for NPV calculations (years).
    pub project_lifetime_years: f64,
    /// Nominal discount rate for NPV calculations.
    pub discount_rate: f64,
    /// Safety margin applied to the required current before database lookup
    /// (IEC 60364-5-52 recommends ≥ 1.25 for general industrial use).
    pub contingency_factor: f64,
}

impl Default for CableSizingEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl CableSizingEngine {
    /// Create a new engine pre-populated with the standard cable database and
    /// default economic parameters (energy price 60 USD/MWh, 30-year life,
    /// 6 % discount rate, 1.25 contingency factor).
    pub fn new() -> Self {
        Self {
            database: Self::generate_standard_database(),
            energy_price_usd_per_mwh: 60.0,
            project_lifetime_years: 30.0,
            discount_rate: 0.06,
            contingency_factor: 1.25,
        }
    }

    // -----------------------------------------------------------------------
    // Thermal rating
    // -----------------------------------------------------------------------

    /// Compute the thermally corrected current rating for `cable` under the
    /// given `conditions`, applying IEC 60287 derating factors for:
    ///
    /// 1. Ambient temperature deviation (`K_T`).
    /// 2. Soil thermal resistivity (`K_ρ`) — buried cables only.
    /// 3. Cable grouping mutual heating (`K_G`).
    ///
    /// **Temperature derating** (IEC 60287-1-1 §2.1):
    /// ```text
    /// K_T = sqrt((T_max - T_amb) / (T_max - T_ref))
    /// ```
    ///
    /// **Soil resistivity derating** (IEC 60287-2-1 §2.2):
    /// ```text
    /// K_ρ = sqrt(ρ_ref / ρ_soil)   (ρ_ref = 1.0 K·m/W)
    /// ```
    ///
    /// **Grouping** — see IEC 60287-2-1 Table B.1.
    pub fn compute_thermal_rating(
        cable: &CableSpec,
        conditions: &InstallationConditions,
    ) -> ThermalRating {
        let t_max = cable.max_conductor_temp_c;
        let t_ref = conditions.reference_temp_c();
        let t_amb = conditions.ambient_temp_c;

        // --- Temperature derating factor ---
        let numerator = t_max - t_amb;
        let denominator = t_max - t_ref;
        let k_t = if denominator > 0.0 && numerator > 0.0 {
            (numerator / denominator).sqrt()
        } else {
            0.0 // cable thermally overloaded at this ambient
        };

        // --- Soil resistivity derating (buried & duct only) ---
        let k_rho = match conditions.method {
            InstallationMethod::DirectBuried
            | InstallationMethod::InDuct
            | InstallationMethod::Underwater => {
                let rho = conditions.soil_thermal_resistivity_k_m_per_w.max(0.01);
                (1.0_f64 / rho).sqrt()
            }
            _ => 1.0,
        };

        // --- Grouping derating ---
        let k_g = if conditions.grouping_factor > 0.0 {
            conditions.grouping_factor
        } else {
            grouping_factor(conditions.n_cables_in_group)
        };

        let total = k_t * k_rho * k_g;
        let derated_current = cable.rated_current_a * total;

        // Joule losses at derated current (W/m), using resistance at T_max.
        let alpha = cable.conductor_material.alpha();
        let r_op = resistance_at_temp(cable.resistivity_ohm_per_km, alpha, t_max) / 1000.0; // Ω/m
        let joule = derated_current * derated_current * r_op;

        // Three-phase rated power in MW (line-to-line voltage from voltage class).
        let v_kv = cable.voltage_class.voltage_kv();
        let rated_power_mw = 3_f64.sqrt() * v_kv * derated_current / 1000.0; // MW

        ThermalRating {
            rated_current_a: derated_current,
            rated_power_mw,
            derating_factor_temp: k_t,
            derating_factor_soil: k_rho,
            derating_factor_grouping: k_g,
            total_derating_factor: total,
            conductor_temp_at_rated_c: t_max,
            joule_losses_w_per_m: joule,
        }
    }

    // -----------------------------------------------------------------------
    // Voltage drop
    // -----------------------------------------------------------------------

    /// Compute the three-phase voltage drop percentage at the given load.
    ///
    /// Formula (IEC 60364-5-52 simplified):
    /// ```text
    /// ΔV = I × L × (R × cos φ + X × sin φ)     [V, line-to-neutral]
    /// ΔV% = ΔV / (V_LL / √3) × 100
    /// ```
    ///
    /// where `R` and `X` are in Ω/km and `L` is in km.
    pub fn compute_voltage_drop(
        cable: &CableSpec,
        current_a: f64,
        length_km: f64,
        power_factor: f64,
    ) -> f64 {
        let cos_phi = power_factor.clamp(0.0, 1.0);
        let sin_phi = (1.0 - cos_phi * cos_phi).max(0.0).sqrt();

        let r = cable.resistivity_ohm_per_km;
        let x = cable.reactance_ohm_per_km;

        // Voltage drop per phase (line-to-neutral), V
        let delta_v = current_a * length_km * (r * cos_phi + x * sin_phi);

        // Phase voltage (V_LL / √3)
        let v_phase = cable.voltage_class.voltage_kv() * 1000.0 / 3_f64.sqrt();

        if v_phase < 1e-9 {
            return 0.0;
        }
        delta_v / v_phase * 100.0
    }

    // -----------------------------------------------------------------------
    // Short-circuit withstand
    // -----------------------------------------------------------------------

    /// Maximum short-circuit current the cable can withstand for `fault_duration_s`
    /// seconds without damage, in kA.
    ///
    /// IEC 60949 adiabatic formula:
    /// ```text
    /// I_sc = k × A / √t_sc
    /// ```
    /// where `k` = 143 A·s^0.5/mm² for Cu/XLPE and 94 for Al/XLPE,
    /// `A` is the cross-section in mm², and `t_sc` is in seconds.
    pub fn compute_short_circuit_withstand(cable: &CableSpec, fault_duration_s: f64) -> f64 {
        let k = cable.conductor_material.k_factor();
        let a = cable.conductor_cross_section_mm2;
        let t = fault_duration_s.max(0.01);
        k * a / (t.sqrt() * 1000.0) // convert A → kA
    }

    // -----------------------------------------------------------------------
    // Checks
    // -----------------------------------------------------------------------

    /// Check whether the cable selection meets the maximum permissible voltage
    /// drop criterion.  Returns `true` if `result.voltage_drop_pct ≤ max_drop_pct`.
    pub fn check_voltage_drop(result: &CableSizingResult, max_drop_pct: f64) -> bool {
        result.voltage_drop_pct <= max_drop_pct
    }

    // -----------------------------------------------------------------------
    // Main sizing routine
    // -----------------------------------------------------------------------

    /// Select a cable and determine the number of parallel circuits required to
    /// transfer `required_power_mw` at `voltage_kv` over a route of `length_km`.
    ///
    /// Selection logic:
    /// 1. Convert power to required current (three-phase, pf = 0.9).
    /// 2. Apply `contingency_factor` safety margin.
    /// 3. Look up the minimum-size cable in the matching voltage class.
    /// 4. If no single cable is sufficient, use the largest available and add
    ///    parallel circuits until the combined rating is met.
    /// 5. Compute thermal rating, voltage drop (pf = 0.9), SC withstand (1 s)
    ///    and economic metrics.
    pub fn size_cable(
        &self,
        required_power_mw: f64,
        voltage_kv: f64,
        length_km: f64,
        conditions: &InstallationConditions,
    ) -> CableSizingResult {
        let voltage_class = VoltageClass::from_kv(voltage_kv);
        let v_line = voltage_class.voltage_kv().max(0.001);

        // Required current at pf = 0.9 (three-phase)
        let pf = 0.9_f64;
        let required_current_a = required_power_mw * 1e6 / (3_f64.sqrt() * v_line * 1e3 * pf);

        // Design current includes contingency factor
        let design_current_a = required_current_a * self.contingency_factor;

        // ---- Cable selection ----
        let (selected_cable, n_parallel) = if let Some(cable) = self
            .database
            .find_minimum_size(design_current_a, voltage_class)
        {
            (cable.clone(), 1)
        } else {
            // No single cable is large enough — use largest and parallel
            let largest = self
                .database
                .largest_cable(voltage_class)
                .cloned()
                .unwrap_or_else(|| self.fallback_cable(voltage_class));

            // Compute derated rating of the largest cable
            let rating = Self::compute_thermal_rating(&largest, conditions);
            let single_rating = rating.rated_current_a.max(1.0);
            let n = ((design_current_a / single_rating).ceil() as usize).max(1);
            (largest, n)
        };

        // ---- Thermal rating ----
        let thermal_rating = Self::compute_thermal_rating(&selected_cable, conditions);

        // Effective rating with parallel circuits
        let effective_current_a = thermal_rating.rated_current_a * n_parallel as f64;

        // ---- Voltage drop (at actual load current, single cable per parallel circuit) ----
        let current_per_cable = required_current_a / n_parallel as f64;
        let vdrop_pct =
            Self::compute_voltage_drop(&selected_cable, current_per_cable, length_km, pf);

        // ---- Short-circuit withstand (1 s fault) ----
        let sc_ka = Self::compute_short_circuit_withstand(&selected_cable, 1.0);

        // ---- Economics ----
        // Capital cost: cost_per_m * length * n_cores * parallel (3-core cables
        // are sold per metre of complete cable, single-core are sold per metre per core)
        let cable_metres = length_km * 1000.0;
        let n_single_cables = if selected_cable.n_cores == 1 { 3 } else { 1 };
        let capital_cost_usd = selected_cable.cost_usd_per_m()
            * cable_metres
            * n_single_cables as f64
            * n_parallel as f64;

        // Annual Joule losses: W/m × km×1000 × 8760 h × 1e-6 → MWh/year
        let loss_w_per_m = thermal_rating.joule_losses_w_per_m;
        let annual_losses_mwh = loss_w_per_m * cable_metres * n_parallel as f64 * 8760.0 / 1e6;

        let annual_loss_cost_usd = annual_losses_mwh * self.energy_price_usd_per_mwh;

        // ---- Recommended flag ----
        // Passes if: load < derated capacity, vdrop < 5 %, effective rating covers load
        let capacity_ok = effective_current_a >= required_current_a;
        let vdrop_ok = vdrop_pct <= 5.0;
        let recommended = capacity_ok && vdrop_ok;

        CableSizingResult {
            selected_cable,
            thermal_rating,
            n_parallel_circuits: n_parallel,
            total_length_km: length_km,
            total_capital_cost_usd: capital_cost_usd,
            annual_losses_mwh,
            annual_loss_cost_usd,
            voltage_drop_pct: vdrop_pct,
            short_circuit_withstand_ka: sc_ka,
            recommended,
        }
    }

    // -----------------------------------------------------------------------
    // Standard database
    // -----------------------------------------------------------------------

    /// Populate a [`CableDatabase`] with representative standard cables.
    ///
    /// Covers:
    /// - LV 400 V — Cu/XLPE 3-core, 16–240 mm²
    /// - MV 10 kV — Cu/XLPE 3-core, 35–300 mm²
    /// - MV 33 kV — Cu/XLPE 1-core, 185–500 mm²
    /// - HV 110 kV — Cu/XLPE 1-core, 300–630 mm²
    ///
    /// Electrical parameters are derived from CIGRE TB 531 and typical
    /// manufacturer data (Prysmian, Nexans, NKT).
    pub fn generate_standard_database() -> CableDatabase {
        let mut cables = Vec::new();
        let mut id = 0_usize;

        // ---- LV 400 V, Cu/XLPE, 3-core ----
        // R values: IEC 60228 Class 2 Cu at 20°C (ρ = 17.241 nΩ·m)
        // X ≈ 0.08–0.09 Ω/km for multi-core LV; C ≈ 200–350 nF/km
        for &(mm2, r, x, c_nf, i_a, w, od) in &[
            (16.0, 1.150, 0.080, 200.0, 88.0, 0.60, 28.0),
            (35.0, 0.524, 0.078, 230.0, 138.0, 0.90, 35.0),
            (70.0, 0.268, 0.075, 270.0, 195.0, 1.30, 42.0),
            (120.0, 0.153, 0.073, 310.0, 255.0, 1.80, 50.0),
            (185.0, 0.099, 0.072, 350.0, 320.0, 2.50, 58.0),
            (240.0, 0.075, 0.071, 390.0, 380.0, 3.20, 65.0),
        ] {
            id += 1;
            cables.push(CableSpec {
                id,
                name: format!("3×{} mm² Cu/XLPE LV", mm2 as usize),
                voltage_class: VoltageClass::Lv400v,
                conductor_material: ConductorMaterial::Copper,
                insulation: CableInsulation::Xlpe,
                conductor_cross_section_mm2: mm2,
                n_cores: 3,
                resistivity_ohm_per_km: r,
                reactance_ohm_per_km: x,
                capacitance_nf_per_km: c_nf,
                max_conductor_temp_c: 90.0,
                rated_current_a: i_a,
                weight_kg_per_m: w,
                outer_diameter_mm: od,
            });
        }

        // ---- MV 10 kV, Cu/XLPE, 3-core ----
        // X ≈ 0.10–0.12 Ω/km for trefoil formation; C ≈ 150–280 nF/km
        for &(mm2, r, x, c_nf, i_a, w, od) in &[
            (35.0, 0.524, 0.115, 150.0, 125.0, 1.80, 48.0),
            (70.0, 0.268, 0.110, 185.0, 180.0, 2.50, 57.0),
            (120.0, 0.153, 0.107, 220.0, 240.0, 3.50, 67.0),
            (185.0, 0.099, 0.104, 255.0, 305.0, 4.80, 77.0),
            (240.0, 0.075, 0.102, 280.0, 360.0, 6.00, 86.0),
            (300.0, 0.060, 0.100, 310.0, 410.0, 7.40, 95.0),
        ] {
            id += 1;
            cables.push(CableSpec {
                id,
                name: format!("3×{} mm² Cu/XLPE 10 kV", mm2 as usize),
                voltage_class: VoltageClass::Mv10kv,
                conductor_material: ConductorMaterial::Copper,
                insulation: CableInsulation::Xlpe,
                conductor_cross_section_mm2: mm2,
                n_cores: 3,
                resistivity_ohm_per_km: r,
                reactance_ohm_per_km: x,
                capacitance_nf_per_km: c_nf,
                max_conductor_temp_c: 90.0,
                rated_current_a: i_a,
                weight_kg_per_m: w,
                outer_diameter_mm: od,
            });
        }

        // ---- MV 33 kV, Cu/XLPE, 1-core (trefoil) ----
        // X ≈ 0.12–0.15 Ω/km; C ≈ 200–350 nF/km
        for &(mm2, r, x, c_nf, i_a, w, od) in &[
            (185.0, 0.099, 0.140, 220.0, 355.0, 5.50, 82.0),
            (240.0, 0.075, 0.134, 260.0, 420.0, 6.80, 91.0),
            (300.0, 0.060, 0.130, 300.0, 485.0, 8.30, 100.0),
            (400.0, 0.047, 0.126, 340.0, 565.0, 10.50, 112.0),
            (500.0, 0.037, 0.122, 380.0, 645.0, 13.00, 124.0),
        ] {
            id += 1;
            cables.push(CableSpec {
                id,
                name: format!("1×{} mm² Cu/XLPE 33 kV", mm2 as usize),
                voltage_class: VoltageClass::Mv33kv,
                conductor_material: ConductorMaterial::Copper,
                insulation: CableInsulation::Xlpe,
                conductor_cross_section_mm2: mm2,
                n_cores: 1,
                resistivity_ohm_per_km: r,
                reactance_ohm_per_km: x,
                capacitance_nf_per_km: c_nf,
                max_conductor_temp_c: 90.0,
                rated_current_a: i_a,
                weight_kg_per_m: w,
                outer_diameter_mm: od,
            });
        }

        // ---- HV 110 kV, Cu/XLPE, 1-core ----
        // X ≈ 0.13–0.16 Ω/km; C ≈ 200–300 nF/km
        for &(mm2, r, x, c_nf, i_a, w, od) in &[
            (300.0, 0.060, 0.150, 200.0, 560.0, 15.0, 108.0),
            (400.0, 0.047, 0.145, 230.0, 650.0, 18.5, 120.0),
            (500.0, 0.037, 0.140, 260.0, 745.0, 22.5, 132.0),
            (630.0, 0.028, 0.135, 290.0, 855.0, 28.0, 146.0),
        ] {
            id += 1;
            cables.push(CableSpec {
                id,
                name: format!("1×{} mm² Cu/XLPE 110 kV", mm2 as usize),
                voltage_class: VoltageClass::Hv110kv,
                conductor_material: ConductorMaterial::Copper,
                insulation: CableInsulation::Xlpe,
                conductor_cross_section_mm2: mm2,
                n_cores: 1,
                resistivity_ohm_per_km: r,
                reactance_ohm_per_km: x,
                capacitance_nf_per_km: c_nf,
                max_conductor_temp_c: 90.0,
                rated_current_a: i_a,
                weight_kg_per_m: w,
                outer_diameter_mm: od,
            });
        }

        CableDatabase { cables }
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /// Synthesise a minimal fallback cable for a voltage class when the
    /// database contains no entries.  Used only when the database is empty.
    fn fallback_cable(&self, vc: VoltageClass) -> CableSpec {
        CableSpec {
            id: 9999,
            name: format!("Fallback {:?}", vc),
            voltage_class: vc,
            conductor_material: ConductorMaterial::Copper,
            insulation: CableInsulation::Xlpe,
            conductor_cross_section_mm2: 240.0,
            n_cores: 3,
            resistivity_ohm_per_km: 0.075,
            reactance_ohm_per_km: 0.10,
            capacitance_nf_per_km: 280.0,
            max_conductor_temp_c: 90.0,
            rated_current_a: 360.0,
            weight_kg_per_m: 5.0,
            outer_diameter_mm: 80.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_engine() -> CableSizingEngine {
        CableSizingEngine::new()
    }

    fn buried_conditions() -> InstallationConditions {
        InstallationConditions::default()
    }

    // ------------------------------------------------------------------
    // 1. Database populated
    // ------------------------------------------------------------------

    #[test]
    fn test_database_populated() {
        let engine = default_engine();
        // Must have cables in each of the four voltage classes
        for vc in [
            VoltageClass::Lv400v,
            VoltageClass::Mv10kv,
            VoltageClass::Mv33kv,
            VoltageClass::Hv110kv,
        ] {
            let found = engine.database.find_cables_by_voltage_class(vc);
            assert!(!found.is_empty(), "No cables for {:?}", vc);
        }
    }

    // ------------------------------------------------------------------
    // 2–4. Thermal rating derating factors
    // ------------------------------------------------------------------

    #[test]
    fn test_thermal_rating_derating_temp() {
        let cable = default_engine()
            .database
            .find_cables_by_voltage_class(VoltageClass::Mv10kv)[0]
            .clone();

        let cond_cool = InstallationConditions {
            ambient_temp_c: 10.0,
            ..Default::default()
        };
        let cond_hot = InstallationConditions {
            ambient_temp_c: 40.0,
            ..Default::default()
        };

        let rating_cool = CableSizingEngine::compute_thermal_rating(&cable, &cond_cool);
        let rating_hot = CableSizingEngine::compute_thermal_rating(&cable, &cond_hot);

        assert!(
            rating_cool.rated_current_a > rating_hot.rated_current_a,
            "Cooler ambient must give higher rating: {} vs {}",
            rating_cool.rated_current_a,
            rating_hot.rated_current_a
        );
    }

    #[test]
    fn test_thermal_rating_derating_soil() {
        let cable = default_engine()
            .database
            .find_cables_by_voltage_class(VoltageClass::Mv10kv)[0]
            .clone();

        let cond_good = InstallationConditions {
            soil_thermal_resistivity_k_m_per_w: 0.7,
            ..Default::default()
        };
        let cond_poor = InstallationConditions {
            soil_thermal_resistivity_k_m_per_w: 2.5,
            ..Default::default()
        };

        let rating_good = CableSizingEngine::compute_thermal_rating(&cable, &cond_good);
        let rating_poor = CableSizingEngine::compute_thermal_rating(&cable, &cond_poor);

        assert!(
            rating_good.rated_current_a > rating_poor.rated_current_a,
            "Better soil must give higher rating"
        );
    }

    #[test]
    fn test_thermal_rating_grouping() {
        let cable = default_engine()
            .database
            .find_cables_by_voltage_class(VoltageClass::Mv10kv)[0]
            .clone();

        let cond_single = InstallationConditions {
            n_cables_in_group: 1,
            ..Default::default()
        };
        let cond_group = InstallationConditions {
            n_cables_in_group: 3,
            ..Default::default()
        };

        let rating_single = CableSizingEngine::compute_thermal_rating(&cable, &cond_single);
        let rating_group = CableSizingEngine::compute_thermal_rating(&cable, &cond_group);

        assert!(
            rating_single.rated_current_a > rating_group.rated_current_a,
            "Grouped cables must have lower rating per cable"
        );
    }

    #[test]
    fn test_thermal_rating_positive() {
        let cable = default_engine()
            .database
            .find_cables_by_voltage_class(VoltageClass::Mv10kv)[2]
            .clone();
        let rating = CableSizingEngine::compute_thermal_rating(&cable, &buried_conditions());
        assert!(
            rating.rated_current_a > 0.0,
            "Rated current must be positive"
        );
        assert!(rating.rated_power_mw > 0.0, "Rated power must be positive");
    }

    // ------------------------------------------------------------------
    // 5–7. Cable sizing
    // ------------------------------------------------------------------

    #[test]
    fn test_size_cable_lv() {
        let engine = default_engine();
        // 100 kW at 400 V ≈ 160 A three-phase, pf 0.9
        let result = engine.size_cable(0.1, 0.4, 0.1, &buried_conditions());
        assert!(
            result.selected_cable.voltage_class == VoltageClass::Lv400v,
            "Should select an LV cable"
        );
        assert!(result.n_parallel_circuits >= 1);
    }

    #[test]
    fn test_size_cable_mv() {
        let engine = default_engine();
        // 5 MW at 10 kV ≈ 320 A three-phase
        let result = engine.size_cable(5.0, 10.0, 2.0, &buried_conditions());
        assert!(
            result.selected_cable.voltage_class == VoltageClass::Mv10kv,
            "Should select MV 10 kV cable"
        );
        assert!(result.selected_cable.rated_current_a > 0.0);
    }

    #[test]
    fn test_size_cable_parallel() {
        let engine = default_engine();
        // 300 MW at 110 kV — far exceeds any single cable; must parallel
        let result = engine.size_cable(300.0, 110.0, 10.0, &buried_conditions());
        assert!(
            result.n_parallel_circuits > 1,
            "Very large load must require parallel circuits, got {}",
            result.n_parallel_circuits
        );
    }

    // ------------------------------------------------------------------
    // 8–9. Voltage drop
    // ------------------------------------------------------------------

    #[test]
    fn test_voltage_drop_formula() {
        let cable = CableSpec {
            id: 1,
            name: "Test".into(),
            voltage_class: VoltageClass::Mv10kv,
            conductor_material: ConductorMaterial::Copper,
            insulation: CableInsulation::Xlpe,
            conductor_cross_section_mm2: 185.0,
            n_cores: 3,
            resistivity_ohm_per_km: 0.099,
            reactance_ohm_per_km: 0.104,
            capacitance_nf_per_km: 255.0,
            max_conductor_temp_c: 90.0,
            rated_current_a: 305.0,
            weight_kg_per_m: 4.8,
            outer_diameter_mm: 77.0,
        };

        let pf = 0.9_f64;
        let sin_phi = (1.0 - pf * pf).sqrt();
        let i = 200.0_f64;
        let l = 3.0_f64;
        let expected_delta_v =
            i * l * (cable.resistivity_ohm_per_km * pf + cable.reactance_ohm_per_km * sin_phi);
        let v_phase = 10.0e3 / 3_f64.sqrt();
        let expected_pct = expected_delta_v / v_phase * 100.0;

        let computed = CableSizingEngine::compute_voltage_drop(&cable, i, l, pf);
        let diff = (computed - expected_pct).abs();
        assert!(
            diff < 1e-9,
            "Voltage drop mismatch: {} vs {}",
            computed,
            expected_pct
        );
    }

    #[test]
    fn test_voltage_drop_pct_range() {
        let engine = default_engine();
        let result = engine.size_cable(5.0, 10.0, 5.0, &buried_conditions());
        assert!(
            result.voltage_drop_pct >= 0.0 && result.voltage_drop_pct < 10.0,
            "Voltage drop {} out of expected range",
            result.voltage_drop_pct
        );
    }

    // ------------------------------------------------------------------
    // 10–12. Short-circuit withstand
    // ------------------------------------------------------------------

    #[test]
    fn test_sc_withstand_cu_xlpe() {
        let cable = CableSpec {
            id: 1,
            name: "SC test Cu".into(),
            voltage_class: VoltageClass::Mv10kv,
            conductor_material: ConductorMaterial::Copper,
            insulation: CableInsulation::Xlpe,
            conductor_cross_section_mm2: 185.0,
            n_cores: 3,
            resistivity_ohm_per_km: 0.099,
            reactance_ohm_per_km: 0.104,
            capacitance_nf_per_km: 255.0,
            max_conductor_temp_c: 90.0,
            rated_current_a: 305.0,
            weight_kg_per_m: 4.8,
            outer_diameter_mm: 77.0,
        };
        // k=143, A=185, t=1 → I_sc = 143*185/1000 = 26.455 kA
        let sc = CableSizingEngine::compute_short_circuit_withstand(&cable, 1.0);
        let expected = 143.0 * 185.0 / 1000.0;
        assert!(
            (sc - expected).abs() < 1e-9,
            "k=143 not used: got {}, expected {}",
            sc,
            expected
        );
    }

    #[test]
    fn test_sc_withstand_time_dependence() {
        let cable = default_engine()
            .database
            .find_cables_by_voltage_class(VoltageClass::Mv10kv)[2]
            .clone();
        let sc_short = CableSizingEngine::compute_short_circuit_withstand(&cable, 0.1);
        let sc_long = CableSizingEngine::compute_short_circuit_withstand(&cable, 1.0);
        assert!(
            sc_short > sc_long,
            "Shorter fault duration must allow higher SC current"
        );
    }

    #[test]
    fn test_sc_withstand_area_dependence() {
        let db = default_engine().database;
        let cables_mv = db.find_cables_by_voltage_class(VoltageClass::Mv10kv);
        assert!(cables_mv.len() >= 2);
        // Sort by area
        let mut sorted = cables_mv.clone();
        sorted.sort_by(|a, b| {
            a.conductor_cross_section_mm2
                .partial_cmp(&b.conductor_cross_section_mm2)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let sc_small = CableSizingEngine::compute_short_circuit_withstand(sorted[0], 1.0);
        let sc_large =
            CableSizingEngine::compute_short_circuit_withstand(sorted[sorted.len() - 1], 1.0);
        assert!(
            sc_large > sc_small,
            "Larger cross-section must withstand more SC current"
        );
    }

    // ------------------------------------------------------------------
    // 13. find_minimum_size
    // ------------------------------------------------------------------

    #[test]
    fn test_find_minimum_size() {
        let db = CableSizingEngine::generate_standard_database();
        // Ask for 150 A at 10 kV — should return 3×120 mm² (240 A) or smaller that meets it
        let result = db.find_minimum_size(150.0, VoltageClass::Mv10kv);
        assert!(result.is_some(), "Should find a cable");
        let cable = result.unwrap();
        assert!(
            cable.rated_current_a >= 150.0,
            "Selected cable must meet required current"
        );
        // Verify it is the smallest that meets the criterion
        let others: Vec<&CableSpec> = db
            .find_cables_by_voltage_class(VoltageClass::Mv10kv)
            .into_iter()
            .filter(|c| {
                c.rated_current_a >= 150.0
                    && c.conductor_cross_section_mm2 < cable.conductor_cross_section_mm2
            })
            .collect();
        assert!(
            others.is_empty(),
            "Should select the minimum sufficient cable"
        );
    }

    // ------------------------------------------------------------------
    // 14–15. Economic metrics
    // ------------------------------------------------------------------

    #[test]
    fn test_capital_cost_positive() {
        let engine = default_engine();
        let result = engine.size_cable(5.0, 10.0, 5.0, &buried_conditions());
        assert!(
            result.total_capital_cost_usd > 0.0,
            "Capital cost must be positive"
        );
    }

    #[test]
    fn test_annual_losses_positive() {
        let engine = default_engine();
        let result = engine.size_cable(5.0, 10.0, 5.0, &buried_conditions());
        assert!(
            result.annual_losses_mwh > 0.0,
            "Annual losses must be positive at non-zero load"
        );
    }

    // ------------------------------------------------------------------
    // 16. Derating factor product
    // ------------------------------------------------------------------

    #[test]
    fn test_derating_factor_product() {
        let cable = default_engine()
            .database
            .find_cables_by_voltage_class(VoltageClass::Mv10kv)[0]
            .clone();
        let cond = InstallationConditions {
            ambient_temp_c: 30.0,
            soil_thermal_resistivity_k_m_per_w: 1.5,
            n_cables_in_group: 3,
            ..Default::default()
        };
        let rating = CableSizingEngine::compute_thermal_rating(&cable, &cond);
        let expected_total = rating.derating_factor_temp
            * rating.derating_factor_soil
            * rating.derating_factor_grouping;
        let diff = (rating.total_derating_factor - expected_total).abs();
        assert!(diff < 1e-12, "Total derating must equal product of factors");
    }

    // ------------------------------------------------------------------
    // 17. Conductor temperature at rated load
    // ------------------------------------------------------------------

    #[test]
    fn test_conductor_temp_at_rated() {
        let cable = default_engine()
            .database
            .find_cables_by_voltage_class(VoltageClass::Mv10kv)[0]
            .clone();
        let rating = CableSizingEngine::compute_thermal_rating(&cable, &buried_conditions());
        let diff = (rating.conductor_temp_at_rated_c - cable.max_conductor_temp_c).abs();
        assert!(
            diff < 1e-9,
            "Conductor temperature at rated current must equal max_conductor_temp_c"
        );
    }

    // ------------------------------------------------------------------
    // 18–19. recommended flag and voltage drop check
    // ------------------------------------------------------------------

    #[test]
    fn test_recommended_flag() {
        let engine = default_engine();
        // 2 MW at 10 kV over 1 km — should be well within limits
        let result = engine.size_cable(2.0, 10.0, 1.0, &buried_conditions());
        assert!(
            result.recommended,
            "Well-sized cable for low load should be recommended"
        );
    }

    #[test]
    fn test_voltage_drop_check() {
        let engine = default_engine();
        let result = engine.size_cable(2.0, 10.0, 1.0, &buried_conditions());
        let passes = CableSizingEngine::check_voltage_drop(&result, 5.0);
        // Short route, modest load — should comply
        assert!(passes, "Short route should have voltage drop < 5 %");
    }

    // ------------------------------------------------------------------
    // 20. Additional: Aluminum k-factor
    // ------------------------------------------------------------------

    #[test]
    fn test_sc_withstand_al_k94() {
        let cable = CableSpec {
            id: 2,
            name: "SC test Al".into(),
            voltage_class: VoltageClass::Mv10kv,
            conductor_material: ConductorMaterial::Aluminum,
            insulation: CableInsulation::Xlpe,
            conductor_cross_section_mm2: 300.0,
            n_cores: 3,
            resistivity_ohm_per_km: 0.100,
            reactance_ohm_per_km: 0.100,
            capacitance_nf_per_km: 280.0,
            max_conductor_temp_c: 90.0,
            rated_current_a: 320.0,
            weight_kg_per_m: 4.0,
            outer_diameter_mm: 75.0,
        };
        // k=94, A=300, t=1 → 94*300/1000 = 28.2 kA
        let sc = CableSizingEngine::compute_short_circuit_withstand(&cable, 1.0);
        let expected = 94.0 * 300.0 / 1000.0;
        assert!(
            (sc - expected).abs() < 1e-9,
            "Al k=94 not used: {} vs {}",
            sc,
            expected
        );
    }

    // ------------------------------------------------------------------
    // 21. InstallationMethod air uses 30 °C reference
    // ------------------------------------------------------------------

    #[test]
    fn test_in_air_reference_temp() {
        let cond = InstallationConditions {
            method: InstallationMethod::InAir,
            ambient_temp_c: 30.0,
            ..Default::default()
        };
        assert_eq!(cond.reference_temp_c(), 30.0);
    }

    // ------------------------------------------------------------------
    // 22. VoltageClass::from_kv round-trip
    // ------------------------------------------------------------------

    #[test]
    fn test_voltage_class_from_kv() {
        assert_eq!(VoltageClass::from_kv(0.4), VoltageClass::Lv400v);
        assert_eq!(VoltageClass::from_kv(10.0), VoltageClass::Mv10kv);
        assert_eq!(VoltageClass::from_kv(33.0), VoltageClass::Mv33kv);
        assert_eq!(VoltageClass::from_kv(110.0), VoltageClass::Hv110kv);
        assert_eq!(VoltageClass::from_kv(400.0), VoltageClass::Hv400kv);
    }
}
