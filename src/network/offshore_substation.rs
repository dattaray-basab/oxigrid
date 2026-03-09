//! Offshore substation design and optimization module for offshore wind farm electrical systems.
//!
//! This module provides tools for designing the complete electrical infrastructure
//! of offshore wind farms, including:
//! - Collector array design (MV cable systems at 33 kV or 66 kV)
//! - Offshore substation platform selection and sizing
//! - Export cable technology selection (HVAC vs HVDC)
//! - Loss computation and LCOE estimation
//! - HVAC vs HVDC system comparison
//!
//! # Design Philosophy
//! The module implements industry-standard heuristics and engineering formulas
//! used in offshore wind farm electrical system design, following IEC 61400-1,
//! CIGRE guidelines, and common practice from North Sea project data.

/// Offshore substation platform type classification.
#[derive(Debug, Clone, PartialEq)]
pub enum OffshoreSubstationType {
    /// MV collector platform only (33 kV), no HV transformation on platform.
    OssMv,
    /// High-voltage HVAC platform (132 kV or 220 kV step-up).
    OssHv,
    /// AC/DC converter platform for HVDC export systems.
    OssHvdc,
    /// Floating offshore substation for deep-water applications (> 60 m).
    FloatingOss,
    /// Shared substation platform serving multiple wind farms.
    SharedOss,
}

/// Export cable technology type.
#[derive(Debug, Clone, PartialEq)]
pub enum CableType {
    /// XLPE insulated 66 kV AC cable.
    Xlpe66kv,
    /// XLPE insulated 132 kV AC cable (standard North Sea HVAC).
    Xlpe132kv,
    /// XLPE insulated 220 kV AC cable (long-distance HVAC).
    Xlpe220kv,
    /// Mass-impregnated low-pressure paper 220 kV DC cable (legacy HVDC).
    MiLpPaper220kv,
    /// HVDC mass-impregnated 500 kV cable.
    HvdcMass500kv,
    /// HVDC XLPE 525 kV cable (modern VSC-HVDC).
    HvdcXlpe525kv,
}

/// Offshore substation platform data model.
#[derive(Debug, Clone)]
pub struct OffshoreSubstation {
    /// Unique identifier.
    pub id: usize,
    /// Human-readable name (e.g., "Alpha OSS").
    pub name: String,
    /// Geographic location as (latitude, longitude) in decimal degrees.
    pub location: (f64, f64),
    /// Platform type classification.
    pub substation_type: OffshoreSubstationType,
    /// Total rated power capacity in MVA.
    pub rated_power_mva: f64,
    /// HV-side voltage in kV (132, 220, or 400 kV).
    pub voltage_hv_kv: f64,
    /// LV-side / collector voltage in kV (33 or 66 kV).
    pub voltage_lv_kv: f64,
    /// Number of power transformers installed.
    pub n_transformers: usize,
    /// Rating of each individual transformer in MVA.
    pub transformer_mva_each: f64,
    /// Platform auxiliary and no-load losses as a percentage of rated power.
    pub losses_pct: f64,
    /// Water depth at the substation location in metres.
    pub water_depth_m: f64,
    /// Total capital expenditure in million USD.
    pub capital_cost_musd: f64,
    /// Annual operating expenditure in million USD per year.
    pub annual_opex_musd: f64,
    /// Estimated topsides weight in tonnes.
    pub platform_weight_tonnes: f64,
    /// Planned commissioning year.
    pub commissioning_year: u32,
}

/// Export cable connecting the offshore substation to the onshore grid.
#[derive(Debug, Clone)]
pub struct ExportCable {
    /// Unique identifier.
    pub id: usize,
    /// ID of the offshore substation this cable departs from.
    pub from_substation: usize,
    /// ID of the onshore grid bus this cable connects to.
    pub to_onshore_bus: usize,
    /// Cable insulation and voltage technology.
    pub cable_type: CableType,
    /// Total route length in kilometres.
    pub length_km: f64,
    /// Thermal rating per circuit in MW.
    pub rated_power_mw: f64,
    /// Conductor resistance in Ω/km (AC positive sequence or DC pole).
    pub resistance_ohm_per_km: f64,
    /// Capacitive charging reactive power in Mvar/km (AC cables only; 0 for DC).
    pub charging_mvar_per_km: f64,
    /// Installed cost per km per circuit in million USD.
    pub installation_cost_musd_per_km: f64,
    /// Number of cable circuits (for redundancy / additional capacity).
    pub n_circuits: usize,
}

/// Inter-array collector cable system connecting wind turbines to the offshore substation.
#[derive(Debug, Clone)]
pub struct CollectorArray {
    /// Total number of wind turbines in the farm.
    pub n_turbines: usize,
    /// Rated power per turbine in MW.
    pub turbine_mw: f64,
    /// Collector voltage level in kV (33 or 66 kV).
    pub string_voltage_kv: f64,
    /// Number of radial collector strings (feeders).
    pub n_strings: usize,
    /// Number of turbines connected in series on each string.
    pub turbines_per_string: usize,
    /// Total inter-array cable route length in km (sum of all strings).
    pub inter_array_cable_length_km: f64,
    /// Estimated inter-array cable losses as percentage of farm output.
    pub inter_array_losses_pct: f64,
    /// Cable thermal rating in MVA per string.
    pub cable_rating_mva: f64,
}

/// Complete offshore wind farm electrical system definition.
#[derive(Debug, Clone)]
pub struct OffshoreElectricalSystem {
    /// Wind farm project name.
    pub farm_name: String,
    /// Nameplate capacity of the entire wind farm in MW.
    pub total_capacity_mw: f64,
    /// Offshore substation platforms in the system.
    pub substations: Vec<OffshoreSubstation>,
    /// Export cables connecting substations to shore.
    pub export_cables: Vec<ExportCable>,
    /// Inter-array collector cable system.
    pub collector_array: CollectorArray,
    /// Available onshore grid connection capacity in MW.
    pub onshore_grid_capacity_mw: f64,
    /// Shortest distance from any offshore structure to the onshore connection in km.
    pub distance_to_shore_km: f64,
}

/// Breakdown of electrical losses across the offshore wind farm system.
#[derive(Debug, Clone)]
pub struct ElectricalLossBreakdown {
    /// Inter-array collector cable losses as percentage of gross output.
    pub collection_losses_pct: f64,
    /// Substation transformer and auxiliary losses as percentage of gross output.
    pub substation_transformer_losses_pct: f64,
    /// Export cable Joule and dielectric losses as percentage of gross output.
    pub export_cable_losses_pct: f64,
    /// Reactive compensation equipment losses as percentage of gross output.
    pub reactive_compensation_losses_pct: f64,
    /// Sum of all loss components (percentage).
    pub total_losses_pct: f64,
    /// Annual energy lost due to electrical losses in GWh/year.
    pub annual_energy_loss_gwh: f64,
}

/// Main offshore electrical system design and analysis tool.
///
/// Initialise with [`OffshoreSystemDesigner::new`], then call the individual
/// design methods or [`OffshoreSystemDesigner::design_complete_system`] for
/// a fully-populated [`OffshoreElectricalSystem`].
#[derive(Debug, Clone)]
pub struct OffshoreSystemDesigner {
    /// Total nameplate capacity of the wind farm in MW.
    pub farm_capacity_mw: f64,
    /// Number of wind turbines.
    pub n_turbines: usize,
    /// Rated power per turbine in MW.
    pub turbine_mw: f64,
    /// Water depth at the substation site in metres.
    pub water_depth_m: f64,
    /// Distance from offshore substation to onshore grid connection in km.
    pub distance_to_shore_km: f64,
    /// Onshore grid voltage level in kV (e.g., 132, 220, 400).
    pub onshore_voltage_kv: f64,
    /// Expected long-term capacity factor (0–1).
    pub capacity_factor: f64,
    /// Economic project lifetime in years.
    pub project_lifetime_years: f64,
    /// Nominal discount rate for NPV / LCOE calculations (0–1).
    pub discount_rate: f64,
}

impl OffshoreSystemDesigner {
    /// Create a new designer with the four primary engineering inputs.
    ///
    /// Sensible defaults are applied for economic parameters:
    /// - `onshore_voltage_kv` = 220 kV
    /// - `capacity_factor` = 0.45
    /// - `project_lifetime_years` = 25
    /// - `discount_rate` = 0.07
    ///
    /// # Arguments
    /// * `farm_capacity_mw` — Total nameplate capacity in MW.
    /// * `n_turbines` — Number of wind turbines.
    /// * `distance_to_shore_km` — Export cable route length in km.
    /// * `water_depth_m` — Water depth at the offshore substation site in m.
    pub fn new(
        farm_capacity_mw: f64,
        n_turbines: usize,
        distance_to_shore_km: f64,
        water_depth_m: f64,
    ) -> Self {
        let turbine_mw = if n_turbines > 0 {
            farm_capacity_mw / n_turbines as f64
        } else {
            0.0
        };
        Self {
            farm_capacity_mw,
            n_turbines,
            turbine_mw,
            water_depth_m,
            distance_to_shore_km,
            onshore_voltage_kv: 220.0,
            capacity_factor: 0.45,
            project_lifetime_years: 25.0,
            discount_rate: 0.07,
        }
    }

    /// Design the medium-voltage inter-array collector system.
    ///
    /// Selects 33 kV for farms < 200 MW and 66 kV for ≥ 200 MW.
    /// Sizes string count and turbines-per-string to stay within cable ratings.
    pub fn design_mv_collector_system(&self) -> CollectorArray {
        let string_voltage_kv = self.optimize_collector_voltage();

        // Turbines per string limited by cable thermal rating.
        // At 33 kV: cable rating ~40 MVA → max ~8 turbines (5 MW each).
        // At 66 kV: cable rating ~80 MVA → max ~10 turbines.
        let max_turbines_per_string: usize = if string_voltage_kv >= 66.0 { 10 } else { 8 };
        let turbines_per_string = max_turbines_per_string.min(if self.n_turbines > 0 {
            self.n_turbines
        } else {
            1
        });

        let n_strings = if turbines_per_string > 0 {
            self.n_turbines.div_ceil(turbines_per_string)
        } else {
            1
        };

        // Average spacing between turbines ~0.8 km, cable route factor 1.1.
        let avg_spacing_km = 0.8_f64;
        let route_factor = 1.1_f64;
        let inter_array_cable_length_km = self.n_turbines as f64 * avg_spacing_km * route_factor;

        // Losses: I²R model — typically 0.8–1.5% at 33 kV, 0.5–0.9% at 66 kV.
        let inter_array_losses_pct = if string_voltage_kv >= 66.0 { 0.7 } else { 1.1 };

        // Cable rating MVA per string.
        let cable_rating_mva = if string_voltage_kv >= 66.0 {
            80.0
        } else {
            40.0
        };

        CollectorArray {
            n_turbines: self.n_turbines,
            turbine_mw: self.turbine_mw,
            string_voltage_kv,
            n_strings,
            turbines_per_string,
            inter_array_cable_length_km,
            inter_array_losses_pct,
            cable_rating_mva,
        }
    }

    /// Design and size the offshore substation platform.
    ///
    /// Selection logic:
    /// - HVDC converter platform if `distance_to_shore_km` ≥ 80 km.
    /// - Floating platform if `water_depth_m` > 60 m and HVDC not required.
    /// - Fixed HVAC platform otherwise.
    ///
    /// Transformer redundancy is N-1: 2 × 50% for ≤ 600 MVA, 3 × 33% for > 600 MVA.
    pub fn design_offshore_substation(&self) -> OffshoreSubstation {
        let rated_mva = self.farm_capacity_mw * 1.05; // 5 % oversize for losses

        let (sub_type, voltage_hv_kv, n_tx, losses_pct, capital_cost_musd) =
            if self.distance_to_shore_km >= 80.0 {
                (
                    OffshoreSubstationType::OssHvdc,
                    525.0_f64,
                    2_usize,
                    1.5_f64,
                    200.0 + 4.0 * rated_mva,
                )
            } else if self.water_depth_m > 60.0 {
                let hv_kv = if self.onshore_voltage_kv >= 220.0 {
                    220.0
                } else {
                    132.0
                };
                (
                    OffshoreSubstationType::FloatingOss,
                    hv_kv,
                    2_usize,
                    1.0_f64,
                    150.0 + 3.0 * rated_mva,
                )
            } else {
                let hv_kv = if self.onshore_voltage_kv >= 220.0 {
                    220.0
                } else {
                    132.0
                };
                let n = if rated_mva > 600.0 { 3_usize } else { 2_usize };
                (
                    OffshoreSubstationType::OssHv,
                    hv_kv,
                    n,
                    1.0_f64,
                    120.0 + 2.5 * rated_mva,
                )
            };

        let n_transformers = n_tx.max(2);
        let transformer_mva_each = rated_mva / n_transformers as f64;

        // Platform weight model.
        let platform_weight_tonnes = if sub_type == OffshoreSubstationType::FloatingOss {
            800.0 + 3.5 * rated_mva
        } else {
            1200.0 + 4.5 * rated_mva
        };

        // Collector voltage (LV side of OSS transformer).
        let voltage_lv_kv = self.optimize_collector_voltage();

        // OPEX: 2.5 % of CAPEX per year.
        let annual_opex_musd = capital_cost_musd * 0.025;

        OffshoreSubstation {
            id: 1,
            name: format!("OSS-1 ({:.0} MVA)", rated_mva),
            location: (56.0, 4.0), // placeholder North Sea coordinates
            substation_type: sub_type,
            rated_power_mva: rated_mva,
            voltage_hv_kv,
            voltage_lv_kv,
            n_transformers,
            transformer_mva_each,
            losses_pct,
            water_depth_m: self.water_depth_m,
            capital_cost_musd,
            annual_opex_musd,
            platform_weight_tonnes,
            commissioning_year: 2028,
        }
    }

    /// Select the appropriate export cable technology based on distance.
    ///
    /// HVAC (XLPE) is selected for distances < 80 km where reactive compensation
    /// is manageable. HVDC (XLPE 525 kV VSC) is selected for ≥ 80 km.
    pub fn select_export_cable_technology(&self) -> CableType {
        if self.distance_to_shore_km >= 80.0 {
            CableType::HvdcXlpe525kv
        } else if self.onshore_voltage_kv >= 220.0 {
            CableType::Xlpe220kv
        } else {
            CableType::Xlpe132kv
        }
    }

    /// Design the complete offshore electrical system.
    ///
    /// Calls all sub-system design methods and assembles an [`OffshoreElectricalSystem`].
    pub fn design_complete_system(&self) -> OffshoreElectricalSystem {
        let collector = self.design_mv_collector_system();
        let substation = self.design_offshore_substation();
        let cable_type = self.select_export_cable_technology();

        let (resistance, charging, install_cost, rated_cable_mw, n_circuits) =
            export_cable_parameters(&cable_type, self.farm_capacity_mw);

        let export_cable = ExportCable {
            id: 1,
            from_substation: substation.id,
            to_onshore_bus: 1,
            cable_type,
            length_km: self.distance_to_shore_km,
            rated_power_mw: rated_cable_mw * n_circuits as f64,
            resistance_ohm_per_km: resistance,
            charging_mvar_per_km: charging,
            installation_cost_musd_per_km: install_cost,
            n_circuits,
        };

        OffshoreElectricalSystem {
            farm_name: format!(
                "OWF-{:.0}MW-{:.0}km",
                self.farm_capacity_mw, self.distance_to_shore_km
            ),
            total_capacity_mw: self.farm_capacity_mw,
            substations: vec![substation],
            export_cables: vec![export_cable],
            collector_array: collector,
            onshore_grid_capacity_mw: self.farm_capacity_mw * 1.1,
            distance_to_shore_km: self.distance_to_shore_km,
        }
    }

    /// Compute the electrical loss breakdown for the system at a given wind capacity factor.
    ///
    /// Uses simplified engineering loss models:
    /// - Collector losses: taken from [`CollectorArray::inter_array_losses_pct`]
    /// - Transformer losses: taken from [`OffshoreSubstation::losses_pct`]
    /// - Export cable losses: I²R Joule losses at the given power flow
    /// - Reactive compensation losses: 0.05–0.30 % depending on cable length
    pub fn compute_losses(
        &self,
        system: &OffshoreElectricalSystem,
        wind_cf: f64,
    ) -> ElectricalLossBreakdown {
        let wind_cf = wind_cf.clamp(0.0, 1.0);
        let p_mw = system.total_capacity_mw * wind_cf;

        let collection_losses_pct = system.collector_array.inter_array_losses_pct;

        let substation_transformer_losses_pct = system
            .substations
            .first()
            .map(|s| s.losses_pct)
            .unwrap_or(1.0);

        let export_cable_losses_pct = system
            .export_cables
            .iter()
            .map(|cable| cable_joule_loss_pct(p_mw, cable, system.total_capacity_mw))
            .sum::<f64>();

        let reactive_compensation_losses_pct =
            reactive_comp_losses_pct(system.distance_to_shore_km);

        let total_losses_pct = collection_losses_pct
            + substation_transformer_losses_pct
            + export_cable_losses_pct
            + reactive_compensation_losses_pct;

        let aep_gross_gwh = system.total_capacity_mw * wind_cf * 8760.0 / 1000.0;
        let annual_energy_loss_gwh = aep_gross_gwh * total_losses_pct / 100.0;

        ElectricalLossBreakdown {
            collection_losses_pct,
            substation_transformer_losses_pct,
            export_cable_losses_pct,
            reactive_compensation_losses_pct,
            total_losses_pct,
            annual_energy_loss_gwh,
        }
    }

    /// Estimate the reactive compensation requirement in Mvar for the export cables.
    ///
    /// XLPE cables generate significant charging current; shunt reactors or SVCs
    /// are required at both offshore and onshore ends to absorb this reactive power.
    /// Rule of thumb: ~0.15 Mvar/km for 132 kV, ~0.28 Mvar/km for 220 kV.
    pub fn compute_reactive_compensation_required(&self, system: &OffshoreElectricalSystem) -> f64 {
        system
            .export_cables
            .iter()
            .map(|cable| cable.charging_mvar_per_km * cable.length_km * cable.n_circuits as f64)
            .sum::<f64>()
    }

    /// Estimate the total capital cost of the offshore electrical system in USD.
    ///
    /// Sums substation platform CAPEX, export cable installation costs, and
    /// collector array cost estimated at 0.15 million USD/MW.
    pub fn estimate_capital_cost(&self, system: &OffshoreElectricalSystem) -> f64 {
        let substation_cost: f64 = system
            .substations
            .iter()
            .map(|s| s.capital_cost_musd)
            .sum::<f64>();

        let cable_cost: f64 = system
            .export_cables
            .iter()
            .map(|c| c.installation_cost_musd_per_km * c.length_km * c.n_circuits as f64)
            .sum::<f64>();

        // Collector array CAPEX: ~0.15 M USD/MW
        let collector_cost = system.total_capacity_mw * 0.15;

        // Convert M USD → USD
        (substation_cost + cable_cost + collector_cost) * 1_000_000.0
    }

    /// Compute the Levelised Cost of Energy (LCOE) in USD/MWh.
    ///
    /// Uses the standard annuity-based LCOE formula:
    /// `LCOE = (CAPEX × CRF + OPEX_annual) / AEP_net`
    ///
    /// where the Capital Recovery Factor (CRF) is:
    /// `CRF = r(1+r)^n / ((1+r)^n - 1)`
    pub fn compute_lcoe(&self, system: &OffshoreElectricalSystem, capacity_factor: f64) -> f64 {
        let capacity_factor = capacity_factor.clamp(0.01, 1.0);
        let capex_usd = self.estimate_capital_cost(system);

        let annual_opex_usd: f64 = system
            .substations
            .iter()
            .map(|s| s.annual_opex_musd * 1_000_000.0)
            .sum::<f64>();

        // Capital Recovery Factor
        let r = self.discount_rate;
        let n = self.project_lifetime_years;
        let crf = if r.abs() < 1e-9 {
            1.0 / n
        } else {
            r * (1.0 + r).powf(n) / ((1.0 + r).powf(n) - 1.0)
        };

        let losses = self.compute_losses(system, capacity_factor);
        let net_cf = capacity_factor * (1.0 - losses.total_losses_pct / 100.0);
        let aep_net_mwh = system.total_capacity_mw * net_cf * 8760.0;

        if aep_net_mwh <= 0.0 {
            return f64::INFINITY;
        }

        (capex_usd * crf + annual_opex_usd) / aep_net_mwh
    }

    /// Determine the optimal collector voltage level.
    ///
    /// Returns 66 kV for farms ≥ 200 MW (lower losses, fewer strings required)
    /// and 33 kV for smaller farms (lower equipment cost).
    pub fn optimize_collector_voltage(&self) -> f64 {
        if self.farm_capacity_mw >= 200.0 {
            66.0
        } else {
            33.0
        }
    }

    /// Compare HVAC and HVDC electrical system designs for this wind farm.
    ///
    /// Returns `(hvac_system, hvdc_system)`. For short distances the HVAC system
    /// will have lower capital cost; for long distances HVDC is preferable due
    /// to lower losses and elimination of reactive compensation.
    pub fn compare_hvac_vs_hvdc(&self) -> (OffshoreElectricalSystem, OffshoreElectricalSystem) {
        // HVAC designer: cap distance below HVDC breakeven
        let mut hvac_designer = self.clone();
        hvac_designer.distance_to_shore_km = self.distance_to_shore_km.min(79.9);
        let hvac_system = hvac_designer.design_complete_system();

        // HVDC designer: push distance above breakeven
        let mut hvdc_designer = self.clone();
        hvdc_designer.distance_to_shore_km = self.distance_to_shore_km.max(80.0);
        let hvdc_system = hvdc_designer.design_complete_system();

        (hvac_system, hvdc_system)
    }
}

// ─── Internal helper functions ────────────────────────────────────────────────

/// Returns (resistance_ohm_per_km, charging_mvar_per_km, install_cost_musd_per_km,
///          single_circuit_rating_mw, n_circuits) for a cable type and farm capacity.
fn export_cable_parameters(
    cable_type: &CableType,
    farm_capacity_mw: f64,
) -> (f64, f64, f64, f64, usize) {
    let (r, charging, cost_per_km, rating_mw) = match cable_type {
        CableType::Xlpe66kv => (0.0400, 0.08, 1.0, 200.0),
        CableType::Xlpe132kv => (0.0282, 0.15, 1.5, 300.0),
        CableType::Xlpe220kv => (0.0170, 0.28, 2.0, 500.0),
        CableType::MiLpPaper220kv => (0.0180, 0.05, 1.8, 500.0),
        CableType::HvdcMass500kv => (0.0120, 0.0, 2.2, 1000.0),
        CableType::HvdcXlpe525kv => (0.0110, 0.0, 1.8, 1000.0),
    };

    let n_circuits = if rating_mw > 0.0 {
        ((farm_capacity_mw / rating_mw).ceil() as usize).max(1)
    } else {
        1
    };

    (r, charging, cost_per_km, rating_mw, n_circuits)
}

/// Compute export cable Joule losses as a percentage of farm rated power.
///
/// Uses `P_loss = I² × R × L` where `I = P / (√3 × V × cos_φ)` for AC cables
/// and `I = P / (2 × V)` for bipole DC cables.
fn cable_joule_loss_pct(p_operating_mw: f64, cable: &ExportCable, farm_mw: f64) -> f64 {
    if farm_mw <= 0.0 || cable.rated_power_mw <= 0.0 {
        return 0.0;
    }

    let is_hvdc = matches!(
        cable.cable_type,
        CableType::HvdcMass500kv | CableType::HvdcXlpe525kv
    );

    let v_kv = match &cable.cable_type {
        CableType::Xlpe66kv => 66.0,
        CableType::Xlpe132kv => 132.0,
        CableType::Xlpe220kv => 220.0,
        CableType::MiLpPaper220kv => 220.0,
        CableType::HvdcMass500kv => 500.0,
        CableType::HvdcXlpe525kv => 525.0,
    };
    let cos_phi = 0.95_f64;
    let v_v = v_kv * 1000.0;

    // Current in kA flowing in the cable system
    let current_ka = if is_hvdc {
        // DC bipole: I = P / (2 × V_pole)
        p_operating_mw * 1e6 / (2.0 * v_v * 1000.0)
    } else {
        // AC three-phase: I = P / (√3 × V_LL × cos_phi)
        p_operating_mw * 1e6 / (3.0_f64.sqrt() * v_v * cos_phi * 1000.0)
    };

    let r_total_ohm = cable.resistance_ohm_per_km * cable.length_km;
    let n = cable.n_circuits.max(1) as f64;
    // Current splits evenly across parallel circuits
    let i_per_circuit_ka = current_ka / n;
    let p_loss_mw = n * (i_per_circuit_ka * 1000.0).powi(2) * r_total_ohm / 1e6;

    (p_loss_mw / farm_mw) * 100.0
}

/// Estimate reactive compensation auxiliary losses as a percentage of rated power.
///
/// Shunt reactors consume ~0.2 % of rated reactive power; this is converted
/// to an equivalent active power loss fraction.
fn reactive_comp_losses_pct(distance_km: f64) -> f64 {
    if distance_km >= 80.0 {
        // HVDC converter auxiliary losses
        0.05
    } else {
        // Linear interpolation: 0.05 % at ≤10 km to 0.30 % at 80 km
        let frac = (distance_km - 10.0).max(0.0) / 70.0;
        0.05 + 0.25 * frac
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn small_farm() -> OffshoreSystemDesigner {
        // 150 MW, 25 turbines × 6 MW, 50 km offshore, 40 m depth
        OffshoreSystemDesigner::new(150.0, 25, 50.0, 40.0)
    }

    fn large_farm() -> OffshoreSystemDesigner {
        // 600 MW, 100 turbines × 6 MW, 100 km offshore, 45 m depth
        OffshoreSystemDesigner::new(600.0, 100, 100.0, 45.0)
    }

    fn deep_farm() -> OffshoreSystemDesigner {
        // 300 MW, 50 turbines × 6 MW, 60 km offshore, 80 m depth
        OffshoreSystemDesigner::new(300.0, 50, 60.0, 80.0)
    }

    // ── Collector voltage selection ──────────────────────────────────────────

    #[test]
    fn test_collector_voltage_selection_small() {
        // 150 MW < 200 MW threshold → 33 kV
        let d = small_farm();
        assert_eq!(d.optimize_collector_voltage(), 33.0);
    }

    #[test]
    fn test_collector_voltage_selection_large() {
        // 600 MW ≥ 200 MW threshold → 66 kV
        let d = large_farm();
        assert_eq!(d.optimize_collector_voltage(), 66.0);
    }

    #[test]
    fn test_collector_voltage_boundary() {
        // Exactly 200 MW → 66 kV
        let d = OffshoreSystemDesigner::new(200.0, 40, 50.0, 40.0);
        assert_eq!(d.optimize_collector_voltage(), 66.0);
    }

    // ── Substation type selection ────────────────────────────────────────────

    #[test]
    fn test_substation_type_shallow() {
        // 40 m depth, 50 km → fixed HVAC OSS
        let d = small_farm();
        let oss = d.design_offshore_substation();
        assert_eq!(oss.substation_type, OffshoreSubstationType::OssHv);
    }

    #[test]
    fn test_substation_type_deep() {
        // 80 m depth, 60 km (< 80 km) → floating OSS
        let d = deep_farm();
        let oss = d.design_offshore_substation();
        assert_eq!(oss.substation_type, OffshoreSubstationType::FloatingOss);
    }

    #[test]
    fn test_substation_type_hvdc_platform() {
        // distance ≥ 80 km → HVDC converter platform regardless of depth
        let d = large_farm(); // 100 km offshore
        let oss = d.design_offshore_substation();
        assert_eq!(oss.substation_type, OffshoreSubstationType::OssHvdc);
    }

    // ── Export cable technology ──────────────────────────────────────────────

    #[test]
    fn test_hvdc_selection_far() {
        // 100 km ≥ 80 km → HVDC
        let d = large_farm();
        let ct = d.select_export_cable_technology();
        assert_eq!(ct, CableType::HvdcXlpe525kv);
    }

    #[test]
    fn test_hvac_selection_near() {
        // 50 km < 80 km → HVAC
        let d = small_farm();
        let ct = d.select_export_cable_technology();
        assert!(
            matches!(ct, CableType::Xlpe132kv | CableType::Xlpe220kv),
            "Expected HVAC cable type, got {:?}",
            ct
        );
    }

    // ── Complete system design ───────────────────────────────────────────────

    #[test]
    fn test_complete_system_design() {
        let d = small_farm();
        let sys = d.design_complete_system();
        assert!(
            !sys.substations.is_empty(),
            "Must have at least one substation"
        );
        assert!(
            !sys.export_cables.is_empty(),
            "Must have at least one export cable"
        );
        assert_eq!(sys.collector_array.n_turbines, 25);
        assert_eq!(sys.total_capacity_mw, 150.0);
    }

    #[test]
    fn test_system_consistency() {
        let d = large_farm();
        let sys = d.design_complete_system();
        // total_capacity_mw must equal n_turbines × turbine_mw
        let expected = sys.collector_array.n_turbines as f64 * sys.collector_array.turbine_mw;
        assert!(
            (sys.total_capacity_mw - expected).abs() < 1.0,
            "total_capacity_mw={} n_turbines*turbine_mw={}",
            sys.total_capacity_mw,
            expected
        );
    }

    // ── Loss breakdown ───────────────────────────────────────────────────────

    #[test]
    fn test_loss_breakdown_sums() {
        let d = small_farm();
        let sys = d.design_complete_system();
        let lb = d.compute_losses(&sys, 0.45);
        let sum = lb.collection_losses_pct
            + lb.substation_transformer_losses_pct
            + lb.export_cable_losses_pct
            + lb.reactive_compensation_losses_pct;
        assert!(
            (lb.total_losses_pct - sum).abs() < 1e-9,
            "total={} component_sum={}",
            lb.total_losses_pct,
            sum
        );
    }

    #[test]
    fn test_losses_within_bounds() {
        let d = small_farm();
        let sys = d.design_complete_system();
        let lb = d.compute_losses(&sys, 0.45);
        assert!(
            lb.total_losses_pct < 10.0,
            "Total losses {:.2}% exceed 10%",
            lb.total_losses_pct
        );
        assert!(lb.total_losses_pct > 0.0, "Losses should be positive");
    }

    #[test]
    fn test_annual_energy_loss() {
        let d = small_farm();
        let sys = d.design_complete_system();
        let lb = d.compute_losses(&sys, 0.45);
        assert!(
            lb.annual_energy_loss_gwh > 0.0,
            "Annual energy loss must be positive at non-zero CF"
        );
    }

    // ── Capital cost ─────────────────────────────────────────────────────────

    #[test]
    fn test_capital_cost_positive() {
        let d = small_farm();
        let sys = d.design_complete_system();
        let cost = d.estimate_capital_cost(&sys);
        assert!(cost > 0.0, "Capital cost must be positive");
    }

    #[test]
    fn test_platform_weight_model() {
        // Fixed platform (depth ≤ 60 m): W = 1200 + 4.5 × P_MVA
        let d_shallow = OffshoreSystemDesigner::new(200.0, 40, 50.0, 40.0);
        let oss = d_shallow.design_offshore_substation();
        let expected_w = 1200.0 + 4.5 * oss.rated_power_mva;
        assert!(
            (oss.platform_weight_tonnes - expected_w).abs() < 1.0,
            "Fixed OSS weight={} expected={}",
            oss.platform_weight_tonnes,
            expected_w
        );

        // Floating platform (depth > 60 m): W = 800 + 3.5 × P_MVA
        let d_deep = OffshoreSystemDesigner::new(200.0, 40, 50.0, 80.0);
        let oss_f = d_deep.design_offshore_substation();
        let expected_w_f = 800.0 + 3.5 * oss_f.rated_power_mva;
        assert!(
            (oss_f.platform_weight_tonnes - expected_w_f).abs() < 1.0,
            "Floating OSS weight={} expected={}",
            oss_f.platform_weight_tonnes,
            expected_w_f
        );
    }

    // ── LCOE ─────────────────────────────────────────────────────────────────

    #[test]
    fn test_lcoe_reasonable() {
        let d = small_farm();
        let sys = d.design_complete_system();
        let lcoe = d.compute_lcoe(&sys, 0.45);
        assert!(
            (50.0..=200.0).contains(&lcoe),
            "LCOE={:.2} USD/MWh should be in [50, 200]",
            lcoe
        );
    }

    // ── Reactive compensation ────────────────────────────────────────────────

    #[test]
    fn test_reactive_compensation_long_cable() {
        // Both HVAC and HVDC are non-negative; HVAC at 50 km should yield Mvar > 0
        let d_near = small_farm(); // 50 km HVAC
        let sys_near = d_near.design_complete_system();
        let q_near = d_near.compute_reactive_compensation_required(&sys_near);
        assert!(q_near >= 0.0, "Reactive compensation must be non-negative");

        let d_far = OffshoreSystemDesigner::new(400.0, 67, 60.0, 40.0); // 60 km still HVAC
        let sys_far = d_far.design_complete_system();
        let q_far = d_far.compute_reactive_compensation_required(&sys_far);
        assert!(
            q_far >= q_near,
            "Longer HVAC cable ({} km) should need >= Mvar than shorter ({} km)",
            60.0,
            50.0
        );
    }

    // ── Transformer redundancy ───────────────────────────────────────────────

    #[test]
    fn test_transformer_redundancy() {
        let d = small_farm();
        let oss = d.design_offshore_substation();
        assert!(
            oss.n_transformers >= 2,
            "Transformer count {} < 2 (N-1 redundancy required)",
            oss.n_transformers
        );
    }

    // ── HVAC vs HVDC comparison ──────────────────────────────────────────────

    #[test]
    fn test_hvac_vs_hvdc_comparison() {
        let d = OffshoreSystemDesigner::new(500.0, 83, 100.0, 40.0);
        let (hvac, hvdc) = d.compare_hvac_vs_hvdc();

        assert!(
            hvdc.export_cables.iter().any(|c| matches!(
                c.cable_type,
                CableType::HvdcXlpe525kv | CableType::HvdcMass500kv
            )),
            "HVDC system should use HVDC cable type"
        );

        assert!(
            hvac.export_cables.iter().any(|c| matches!(
                c.cable_type,
                CableType::Xlpe132kv | CableType::Xlpe220kv | CableType::Xlpe66kv
            )),
            "HVAC system should use HVAC cable type"
        );
    }

    // ── String count ─────────────────────────────────────────────────────────

    #[test]
    fn test_n_strings_calculation() {
        let d = small_farm(); // 25 turbines
        let col = d.design_mv_collector_system();
        let expected = col.n_turbines.div_ceil(col.turbines_per_string);
        assert_eq!(
            col.n_strings, expected,
            "n_strings={} expected={}",
            col.n_strings, expected
        );
    }

    // ── Export cable rating ──────────────────────────────────────────────────

    #[test]
    fn test_export_cable_rating() {
        let d = large_farm();
        let sys = d.design_complete_system();
        let total_rated: f64 = sys.export_cables.iter().map(|c| c.rated_power_mw).sum();
        assert!(
            total_rated >= sys.total_capacity_mw,
            "Cable rating {:.0} MW < farm capacity {:.0} MW",
            total_rated,
            sys.total_capacity_mw
        );
    }

    // ── OPEX fraction ────────────────────────────────────────────────────────

    #[test]
    fn test_opex_fraction() {
        let d = small_farm();
        let oss = d.design_offshore_substation();
        let frac = oss.annual_opex_musd / oss.capital_cost_musd;
        assert!(
            (0.02..=0.05).contains(&frac),
            "OPEX/CAPEX ratio {:.3} outside [2%, 5%]",
            frac
        );
    }

    // ── Additional edge-case tests ───────────────────────────────────────────

    #[test]
    fn test_zero_turbines_does_not_panic() {
        let d = OffshoreSystemDesigner::new(0.0, 0, 50.0, 40.0);
        let sys = d.design_complete_system();
        assert_eq!(sys.total_capacity_mw, 0.0);
    }

    #[test]
    fn test_collector_array_cable_rating_adequate() {
        // Each string's power must not exceed the cable rating
        let d = large_farm();
        let col = d.design_mv_collector_system();
        let string_power_mw = col.turbines_per_string as f64 * col.turbine_mw;
        assert!(
            string_power_mw <= col.cable_rating_mva,
            "String power {:.1} MW exceeds cable rating {:.1} MVA",
            string_power_mw,
            col.cable_rating_mva
        );
    }

    #[test]
    fn test_losses_at_zero_cf() {
        let d = small_farm();
        let sys = d.design_complete_system();
        let lb = d.compute_losses(&sys, 0.0);
        assert!(
            lb.export_cable_losses_pct < 0.01,
            "Cable losses at zero CF should be near zero"
        );
        assert_eq!(lb.annual_energy_loss_gwh, 0.0);
    }

    #[test]
    fn test_lcoe_increases_with_distance() {
        // Longer export cable → higher CAPEX → higher LCOE
        let d_near = OffshoreSystemDesigner::new(300.0, 50, 30.0, 40.0);
        let d_far = OffshoreSystemDesigner::new(300.0, 50, 70.0, 40.0);

        let sys_near = d_near.design_complete_system();
        let sys_far = d_far.design_complete_system();

        let lcoe_near = d_near.compute_lcoe(&sys_near, 0.45);
        let lcoe_far = d_far.compute_lcoe(&sys_far, 0.45);

        assert!(
            lcoe_far > lcoe_near,
            "LCOE should increase with distance: near={:.2} far={:.2}",
            lcoe_near,
            lcoe_far
        );
    }
}
