//! TSO Ancillary Services Market Clearing.
//!
//! Implements a Transmission System Operator (TSO) ancillary services market
//! covering frequency regulation (primary/secondary/tertiary), spinning and
//! non-spinning reserves, reactive reserve, and capacity market mechanisms.
//!
//! # Products
//! - **Primary Frequency Response**: Fast Frequency Response (FFR), droop
//!   response delivered within 30 s of a disturbance.
//! - **Secondary Reserve**: AGC / Load-Frequency Control, 30 s – 15 min window.
//! - **Tertiary Reserve**: Manually dispatched, 15 min – 1 h window.
//! - **Spinning Reserve**: Online and synchronised units, available immediately.
//! - **Non-Spinning Reserve**: Offline units that can start within 10 min.
//! - **Capacity Market**: Long-term resource adequacy procurement.
//! - **Reactive Reserve**: Voltage support (Q capacity).
//!
//! # Clearing mechanism
//! Merit-order (uniform-price) clearing per product:
//! bids sorted by availability price ascending; the marginal (last accepted)
//! bid sets the clearing price; all accepted bids receive that price.
//!
//! # References
//! - NERC BAL-003-1, "Frequency Response and Frequency Bias Setting", 2022
//! - FERC Order 755, "Frequency Regulation Compensation in Organised Markets", 2011
//! - Ela, E. et al., "Ancillary Services in the United States",
//!   NREL/TP-5500-62708, 2014
//! - Kundur, P., "Power System Stability and Control", McGraw-Hill, 1994

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ─────────────────────────────────────────────────────────────────────────────
// Ancillary product taxonomy
// ─────────────────────────────────────────────────────────────────────────────

/// Category of ancillary service being procured.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AncillaryProduct {
    /// Fast Frequency Response / droop, delivered in < 30 s \[MW\].
    PrimaryFrequencyResponse,
    /// AGC / LFC secondary reserve, 30 s – 15 min window \[MW\].
    SecondaryReserve,
    /// Manual tertiary reserve, 15 min – 1 h window \[MW\].
    TertiaryReserve,
    /// Online, synchronised spinning reserve \[MW\].
    SpinningReserve,
    /// Offline, fast-startable (< 10 min) non-spinning reserve \[MW\].
    NonSpinningReserve,
    /// Long-term resource-adequacy / capacity market \[MW\].
    CapacityMarket,
    /// Reactive power support \[Mvar\].
    ReactiveReserve,
}

impl std::fmt::Display for AncillaryProduct {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AncillaryProduct::PrimaryFrequencyResponse => write!(f, "PrimaryFrequencyResponse"),
            AncillaryProduct::SecondaryReserve => write!(f, "SecondaryReserve"),
            AncillaryProduct::TertiaryReserve => write!(f, "TertiaryReserve"),
            AncillaryProduct::SpinningReserve => write!(f, "SpinningReserve"),
            AncillaryProduct::NonSpinningReserve => write!(f, "NonSpinningReserve"),
            AncillaryProduct::CapacityMarket => write!(f, "CapacityMarket"),
            AncillaryProduct::ReactiveReserve => write!(f, "ReactiveReserve"),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Bid structures
// ─────────────────────────────────────────────────────────────────────────────

/// Ancillary service bid submitted by a market participant.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AncillaryBid {
    /// Unique generator / asset identifier.
    pub unit_id: String,
    /// Ancillary product being offered.
    pub product: AncillaryProduct,
    /// Offered capacity \[MW\] (or \[Mvar\] for reactive reserve).
    pub capacity_mw: f64,
    /// Availability (capacity reservation) price \[$/MW-h\].
    pub availability_price_per_mw_h: f64,
    /// Activation (energy delivery) price when the reserve is called \[$/MWh\].
    pub activation_price_per_mwh: f64,
    /// Minimum continuous delivery duration \[min\].
    pub min_delivery_min: f64,
    /// Maximum continuous delivery duration \[min\].
    pub max_delivery_min: f64,
    /// Ramp rate capability \[MW/min\].
    pub ramp_rate_mw_per_min: f64,
    /// Notice / lead time before activation \[min\].
    pub lead_time_min: f64,
}

/// Energy (day-ahead / real-time) bid used in co-optimisation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyBid {
    /// Unit identifier (must match any associated ancillary bids).
    pub unit_id: String,
    /// Total installed capacity \[MW\].
    pub capacity_mw: f64,
    /// Energy offer price \[$/MWh\].
    pub offer_price_per_mwh: f64,
    /// Minimum generation (must-run) \[MW\].
    pub p_min_mw: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Market requirements
// ─────────────────────────────────────────────────────────────────────────────

/// Procurement targets for each ancillary product in a single interval.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ProductRequirements {
    /// Required primary frequency response capacity \[MW\].
    pub primary_mw: f64,
    /// Required secondary (AGC) reserve \[MW\].
    pub secondary_mw: f64,
    /// Required tertiary reserve \[MW\].
    pub tertiary_mw: f64,
    /// Required spinning reserve \[MW\].
    pub spinning_reserve_mw: f64,
    /// Required non-spinning reserve \[MW\].
    pub non_spinning_reserve_mw: f64,
    /// Required reactive reserve \[Mvar\].
    pub reactive_mvar: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Clearing results
// ─────────────────────────────────────────────────────────────────────────────

/// One accepted bid in a clearing result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcceptedBid {
    /// Unit identifier.
    pub unit_id: String,
    /// Capacity accepted from this bid \[MW\].
    pub capacity_accepted_mw: f64,
    /// Clearing (uniform) price paid to this unit \[$/MW-h\].
    pub clearing_price_per_mw_h: f64,
    /// Original availability offer price \[$/MW-h\].
    pub offer_price_per_mw_h: f64,
    /// Producer rent (infra-marginal surplus) \[$/h\].
    pub producer_surplus_per_h: f64,
}

/// Full clearing outcome for one ancillary product.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClearingResult {
    /// Product that was cleared.
    pub product: AncillaryProduct,
    /// Bids that were accepted (partially or fully).
    pub accepted_bids: Vec<AcceptedBid>,
    /// Uniform clearing price (marginal bid's availability price) \[$/MW-h\].
    pub clearing_price_per_mw_h: f64,
    /// Total capacity cleared \[MW\].
    pub total_capacity_cleared_mw: f64,
    /// Total availability cost for the interval \[$/h\].
    pub total_cost_per_h: f64,
    /// Unmet requirement (shortfall) \[MW\]; 0 if fully procured.
    pub shortfall_mw: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Settlement
// ─────────────────────────────────────────────────────────────────────────────

/// Settlement record for one generating unit.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SettlementReport {
    /// Unit identifier.
    pub unit_id: String,
    /// Availability (capacity) payment \[$/h\] for the interval.
    pub availability_payment: f64,
    /// Activation (energy delivery) payment \[$/h\] for actual energy injected.
    pub activation_payment: f64,
    /// Total payment for the interval \[$/h\].
    pub total_payment: f64,
}

/// Actual activation record for settlement purposes.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActivationRecord {
    /// Unit identifier.
    pub unit_id: String,
    /// Energy actually delivered during activation \[MWh\].
    pub energy_delivered_mwh: f64,
    /// Agreed activation price \[$/MWh\].
    pub activation_price_per_mwh: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Frequency response metrics
// ─────────────────────────────────────────────────────────────────────────────

/// Simplified frequency dynamic metrics following an N-1 disturbance.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrequencyResponseMetrics {
    /// Rate of Change of Frequency immediately post-disturbance \[Hz/s\].
    pub rocof_hz_per_s: f64,
    /// Frequency nadir (minimum frequency) \[Hz\].
    pub frequency_nadir_hz: f64,
    /// Time from disturbance to frequency nadir \[s\].
    pub time_to_nadir_s: f64,
    /// Approximate recovery time back to quasi-steady-state \[s\].
    pub recovery_time_s: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// TSO market struct
// ─────────────────────────────────────────────────────────────────────────────

/// TSO Ancillary Services Market for a single procurement interval.
///
/// Procures primary/secondary/tertiary frequency response, spinning and
/// non-spinning reserve, reactive reserve, and capacity adequacy products
/// using merit-order uniform-price clearing.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TsoMarket {
    /// Unique market / interval identifier.
    pub market_id: String,
    /// Procurement interval length \[min\] (e.g. 60 for hourly).
    pub interval_min: f64,
    /// Procurement targets per product.
    pub requirements: ProductRequirements,
    /// Nominal system frequency \[Hz\] (50 or 60).
    pub nominal_frequency_hz: f64,
    /// Aggregate system inertia constant H \[s\].
    pub system_inertia_h: f64,
    /// Aggregate rated capacity of the system \[MW\].
    pub system_rated_mw: f64,
    /// System damping coefficient D (p.u. load change per p.u. freq change).
    pub damping_coefficient: f64,
    /// History of clearing results (one entry per `clear_market` call).
    clearing_history: Vec<ClearingResult>,
}

impl TsoMarket {
    /// Create a new TSO market instance.
    ///
    /// # Arguments
    /// - `market_id`       – unique identifier string.
    /// - `interval_min`    – interval length \[min\].
    /// - `requirements`    – procurement targets.
    /// - `nominal_frequency_hz` – system nominal frequency \[Hz\].
    /// - `system_inertia_h`    – aggregate inertia constant H \[s\].
    /// - `system_rated_mw`     – total rated capacity \[MW\].
    /// - `damping_coefficient` – load-damping factor D.
    pub fn new(
        market_id: impl Into<String>,
        interval_min: f64,
        requirements: ProductRequirements,
        nominal_frequency_hz: f64,
        system_inertia_h: f64,
        system_rated_mw: f64,
        damping_coefficient: f64,
    ) -> Self {
        Self {
            market_id: market_id.into(),
            interval_min,
            requirements,
            nominal_frequency_hz,
            system_inertia_h,
            system_rated_mw,
            damping_coefficient,
            clearing_history: Vec::new(),
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Internal helpers
    // ─────────────────────────────────────────────────────────────────────────

    /// Merit-order uniform-price clearing for a single product.
    ///
    /// Sorts bids by `availability_price_per_mw_h` ascending; accepts cheapest
    /// first until the `requirement_mw` is met.  The last accepted bid sets the
    /// clearing price (uniform).
    fn clear_single_product(
        bids: &[AncillaryBid],
        product: &AncillaryProduct,
        requirement_mw: f64,
    ) -> ClearingResult {
        // Filter and sort eligible bids
        let mut eligible: Vec<&AncillaryBid> = bids
            .iter()
            .filter(|b| &b.product == product && b.capacity_mw > 0.0)
            .collect();
        eligible.sort_by(|a, b| {
            a.availability_price_per_mw_h
                .partial_cmp(&b.availability_price_per_mw_h)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut remaining = requirement_mw;
        let mut accepted: Vec<AcceptedBid> = Vec::new();
        let mut clearing_price = 0.0_f64;
        let mut total_cleared = 0.0_f64;

        for bid in &eligible {
            if remaining <= 0.0 {
                break;
            }
            let accepted_mw = bid.capacity_mw.min(remaining);
            clearing_price = bid.availability_price_per_mw_h;
            accepted.push(AcceptedBid {
                unit_id: bid.unit_id.clone(),
                capacity_accepted_mw: accepted_mw,
                clearing_price_per_mw_h: clearing_price,
                offer_price_per_mw_h: bid.availability_price_per_mw_h,
                // Surplus computed after we know final clearing_price
                producer_surplus_per_h: 0.0,
            });
            total_cleared += accepted_mw;
            remaining -= accepted_mw;
        }

        // Back-fill clearing_price and compute producer surplus
        for ab in accepted.iter_mut() {
            ab.clearing_price_per_mw_h = clearing_price;
            ab.producer_surplus_per_h =
                (clearing_price - ab.offer_price_per_mw_h) * ab.capacity_accepted_mw;
        }

        let total_cost = total_cleared * clearing_price;
        let shortfall = requirement_mw - total_cleared;

        ClearingResult {
            product: product.clone(),
            accepted_bids: accepted,
            clearing_price_per_mw_h: clearing_price,
            total_capacity_cleared_mw: total_cleared,
            total_cost_per_h: total_cost,
            shortfall_mw: shortfall.max(0.0),
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public API
    // ─────────────────────────────────────────────────────────────────────────

    /// Clear all ancillary products in merit-order, uniform-price fashion.
    ///
    /// Products cleared: Primary, Secondary, Tertiary, Spinning, Non-Spinning,
    /// Reactive, and Capacity (if requirement > 0).  Results are appended to
    /// `self.clearing_history`.
    ///
    /// Returns a `Vec<ClearingResult>` with one entry per product cleared.
    pub fn clear_market(&mut self, bids: &[AncillaryBid]) -> Vec<ClearingResult> {
        let req = self.requirements.clone();

        let products_and_reqs: &[(AncillaryProduct, f64)] = &[
            (AncillaryProduct::PrimaryFrequencyResponse, req.primary_mw),
            (AncillaryProduct::SecondaryReserve, req.secondary_mw),
            (AncillaryProduct::TertiaryReserve, req.tertiary_mw),
            (AncillaryProduct::SpinningReserve, req.spinning_reserve_mw),
            (
                AncillaryProduct::NonSpinningReserve,
                req.non_spinning_reserve_mw,
            ),
            (AncillaryProduct::ReactiveReserve, req.reactive_mvar),
        ];

        let results: Vec<ClearingResult> = products_and_reqs
            .iter()
            .filter(|(_, req_mw)| *req_mw > 0.0)
            .map(|(product, req_mw)| Self::clear_single_product(bids, product, *req_mw))
            .collect();

        // Store in history
        for r in &results {
            self.clearing_history.push(r.clone());
        }

        results
    }

    /// Joint energy + ancillary co-optimisation (greedy sequential).
    ///
    /// Algorithm:
    /// 1. Sort energy bids by `offer_price_per_mwh` ascending (merit order).
    /// 2. Dispatch energy until `demand_mw` is met; track remaining capacity
    ///    per unit (total capacity minus energy dispatched).
    /// 3. From the **remaining** capacity, clear each ancillary product in
    ///    merit order.  A unit can only offer ancillary services from capacity
    ///    not committed to energy.
    ///
    /// # Arguments
    /// - `energy_bids`     – offers for energy (dispatched first).
    /// - `ancillary_bids`  – offers for ancillary products.
    /// - `demand_mw`       – energy demand to satisfy \[MW\].
    /// - `requirements`    – ancillary procurement targets.
    ///
    /// Returns `(energy_dispatched, ancillary_results)` where
    /// `energy_dispatched` is a map `unit_id → dispatched_mw`.
    pub fn co_optimize_energy_ancillary(
        energy_bids: &[EnergyBid],
        ancillary_bids: &[AncillaryBid],
        demand_mw: f64,
        requirements: &ProductRequirements,
    ) -> (HashMap<String, f64>, Vec<ClearingResult>) {
        // Step 1: sort energy bids cheapest first
        let mut sorted_energy: Vec<&EnergyBid> = energy_bids.iter().collect();
        sorted_energy.sort_by(|a, b| {
            a.offer_price_per_mwh
                .partial_cmp(&b.offer_price_per_mwh)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Step 2: dispatch energy, track used capacity per unit
        let mut used_capacity: HashMap<String, f64> = HashMap::new();
        let mut remaining_demand = demand_mw;

        for eb in &sorted_energy {
            if remaining_demand <= 0.0 {
                break;
            }
            let available = (eb.capacity_mw - eb.p_min_mw).max(0.0);
            let dispatched_extra = available.min(remaining_demand);
            let dispatched = eb.p_min_mw + dispatched_extra;
            *used_capacity.entry(eb.unit_id.clone()).or_insert(0.0) += dispatched;
            remaining_demand -= dispatched_extra;
        }

        // Step 3: build reduced ancillary bids (capped by remaining capacity)
        let reduced_bids: Vec<AncillaryBid> = ancillary_bids
            .iter()
            .filter_map(|ab| {
                // Find max capacity of this unit from energy bids
                let unit_max = energy_bids
                    .iter()
                    .find(|eb| eb.unit_id == ab.unit_id)
                    .map(|eb| eb.capacity_mw)
                    .unwrap_or(ab.capacity_mw);
                let energy_used = used_capacity.get(&ab.unit_id).copied().unwrap_or(0.0);
                let residual = (unit_max - energy_used).max(0.0);
                if residual < 1e-9 {
                    None
                } else {
                    let mut reduced = ab.clone();
                    reduced.capacity_mw = reduced.capacity_mw.min(residual);
                    Some(reduced)
                }
            })
            .collect();

        // Step 4: clear each ancillary product from remaining capacity
        let products_and_reqs: &[(AncillaryProduct, f64)] = &[
            (
                AncillaryProduct::PrimaryFrequencyResponse,
                requirements.primary_mw,
            ),
            (
                AncillaryProduct::SecondaryReserve,
                requirements.secondary_mw,
            ),
            (AncillaryProduct::TertiaryReserve, requirements.tertiary_mw),
            (
                AncillaryProduct::SpinningReserve,
                requirements.spinning_reserve_mw,
            ),
            (
                AncillaryProduct::NonSpinningReserve,
                requirements.non_spinning_reserve_mw,
            ),
            (
                AncillaryProduct::ReactiveReserve,
                requirements.reactive_mvar,
            ),
        ];

        let ancillary_results: Vec<ClearingResult> = products_and_reqs
            .iter()
            .filter(|(_, req_mw)| *req_mw > 0.0)
            .map(|(product, req_mw)| Self::clear_single_product(&reduced_bids, product, *req_mw))
            .collect();

        (used_capacity, ancillary_results)
    }

    /// Calculate the Herfindahl-Hirschman Index (HHI) for a clearing result.
    ///
    /// HHI = Σ (market_share_i)²  where market_share_i is expressed as a
    /// percentage (0–100).  A perfectly competitive market has HHI → 0;
    /// monopoly has HHI = 10 000.
    ///
    /// Returns 0.0 if no capacity was cleared.
    pub fn calculate_hhi(clearing_result: &ClearingResult) -> f64 {
        let total = clearing_result.total_capacity_cleared_mw;
        if total < 1e-12 {
            return 0.0;
        }
        clearing_result
            .accepted_bids
            .iter()
            .map(|ab| {
                let share_pct = 100.0 * ab.capacity_accepted_mw / total;
                share_pct * share_pct
            })
            .sum()
    }

    /// Compute per-unit settlement for one clearing interval.
    ///
    /// # Arguments
    /// - `clearing`            – the clearing result for the product.
    /// - `actual_activations`  – slice of actual activation records for this
    ///   interval (may be empty if the reserve was not called).
    /// - `interval_h`          – interval length \[h\].
    ///
    /// Returns a `Vec<SettlementReport>`, one entry per accepted unit.
    pub fn settlement(
        clearing: &ClearingResult,
        actual_activations: &[ActivationRecord],
        interval_h: f64,
    ) -> Vec<SettlementReport> {
        // Build lookup: unit_id → activation record
        let activation_map: HashMap<&str, &ActivationRecord> = actual_activations
            .iter()
            .map(|a| (a.unit_id.as_str(), a))
            .collect();

        clearing
            .accepted_bids
            .iter()
            .map(|ab| {
                let avail_payment =
                    ab.capacity_accepted_mw * clearing.clearing_price_per_mw_h * interval_h;
                let act_payment = activation_map
                    .get(ab.unit_id.as_str())
                    .map(|rec| rec.energy_delivered_mwh * rec.activation_price_per_mwh)
                    .unwrap_or(0.0);
                SettlementReport {
                    unit_id: ab.unit_id.clone(),
                    availability_payment: avail_payment,
                    activation_payment: act_payment,
                    total_payment: avail_payment + act_payment,
                }
            })
            .collect()
    }

    /// Estimate simplified frequency response metrics for an N-1 disturbance.
    ///
    /// Uses the swing equation and a lumped droop model:
    ///
    /// **ROCOF** (immediately post-disturbance, before governor response):
    /// ```text
    /// ROCOF = -f0 × ΔP / (2 × H × S_rated)   [Hz/s]
    /// ```
    ///
    /// **Frequency nadir** (steady-state frequency deviation due to droop):
    /// ```text
    /// Δf_nadir = -ΔP / (D + R)   [p.u.]
    /// f_nadir  =  f0 × (1 + Δf_nadir)   `Hz`
    /// ```
    /// where D = damping coefficient \[p.u./p.u.\], R = primary reserve
    /// response droop gain (approximated as `primary_cleared_mw / S_rated`
    /// divided by a 4 % droop setting).
    ///
    /// **Time to nadir** (simplified swing-equation estimate):
    /// ```text
    /// t_nadir ≈ sqrt(2 × H × S_rated / (f0 × ΔP))   `s`
    /// ```
    ///
    /// **Recovery time** (empirical: ~3 × t_nadir for primary-dominated systems):
    /// ```text
    /// t_recovery ≈ 3 × t_nadir   `s`
    /// ```
    ///
    /// # Arguments
    /// - `disturbance_mw`      – generation loss / load step \[MW\].
    /// - `primary_cleared_mw`  – primary frequency response capacity cleared \[MW\].
    ///
    /// # Returns
    /// `FrequencyResponseMetrics` with ROCOF, nadir, time to nadir, and
    /// recovery time.  All values are physically bounded (non-negative \[Hz\]).
    pub fn frequency_response_assessment(
        &self,
        disturbance_mw: f64,
        primary_cleared_mw: f64,
    ) -> FrequencyResponseMetrics {
        let f0 = self.nominal_frequency_hz;
        let h = self.system_inertia_h;
        let s = self.system_rated_mw;
        let d = self.damping_coefficient;

        // Normalised disturbance (p.u. on system base)
        let dp_pu = if s > 1e-9 { disturbance_mw / s } else { 0.0 };

        // ROCOF [Hz/s] — swing equation, pre-governor
        // d(ω)/dt = (P_m - P_e) / (2H) in p.u.; convert to Hz/s
        let rocof = if h > 1e-9 {
            f0 * dp_pu / (2.0 * h)
        } else {
            0.0
        };

        // Primary droop response gain R (p.u. MW / p.u. Hz)
        // Typical droop setting 4 % → governor gain = 1/0.04 = 25 p.u./p.u.
        // R_pu = (cleared_mw / s_rated) × governor_gain
        let droop_gain = 25.0_f64; // 4 % droop
        let r_pu = if s > 1e-9 {
            (primary_cleared_mw / s) * droop_gain
        } else {
            0.0
        };

        // Frequency nadir: Δf = -ΔP / (D + R)  (p.u.); convert to Hz
        let denominator = d + r_pu;
        let delta_f_pu = if denominator > 1e-12 {
            -dp_pu / denominator
        } else {
            -dp_pu // no damping or droop
        };
        let frequency_nadir = (f0 * (1.0 + delta_f_pu)).max(0.0);

        // Time to nadir [s] — simplified swing-equation estimate
        let time_to_nadir = if disturbance_mw > 1e-9 && h > 1e-9 {
            (2.0 * h * s / (f0 * disturbance_mw)).sqrt()
        } else {
            0.0
        };

        // Recovery time — empirical approximation
        let recovery_time = 3.0 * time_to_nadir;

        FrequencyResponseMetrics {
            rocof_hz_per_s: rocof,
            frequency_nadir_hz: frequency_nadir,
            time_to_nadir_s: time_to_nadir,
            recovery_time_s: recovery_time,
        }
    }

    /// Immutable view of the clearing history accumulated in this market.
    pub fn clearing_history(&self) -> &[ClearingResult] {
        &self.clearing_history
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: build a default TsoMarket for testing
    fn default_market() -> TsoMarket {
        TsoMarket::new(
            "TEST-01",
            60.0,
            ProductRequirements {
                primary_mw: 100.0,
                secondary_mw: 200.0,
                tertiary_mw: 150.0,
                spinning_reserve_mw: 250.0,
                non_spinning_reserve_mw: 100.0,
                reactive_mvar: 50.0,
            },
            50.0,   // Hz
            5.0,    // H [s]
            3000.0, // S_rated [MW]
            1.0,    // D
        )
    }

    fn spinning_bid(unit: &str, mw: f64, price: f64) -> AncillaryBid {
        AncillaryBid {
            unit_id: unit.to_string(),
            product: AncillaryProduct::SpinningReserve,
            capacity_mw: mw,
            availability_price_per_mw_h: price,
            activation_price_per_mwh: price * 2.0,
            min_delivery_min: 10.0,
            max_delivery_min: 60.0,
            ramp_rate_mw_per_min: 5.0,
            lead_time_min: 0.0,
        }
    }

    fn primary_bid(unit: &str, mw: f64, price: f64) -> AncillaryBid {
        AncillaryBid {
            unit_id: unit.to_string(),
            product: AncillaryProduct::PrimaryFrequencyResponse,
            capacity_mw: mw,
            availability_price_per_mw_h: price,
            activation_price_per_mwh: price * 1.5,
            min_delivery_min: 0.0,
            max_delivery_min: 30.0,
            ramp_rate_mw_per_min: 20.0,
            lead_time_min: 0.0,
        }
    }

    // ── Test 1: Merit order — cheapest bids selected first ────────────────────
    #[test]
    fn test_merit_order_cheapest_first() {
        let bids = vec![
            spinning_bid("G1", 100.0, 5.0),
            spinning_bid("G2", 100.0, 2.0), // cheapest
            spinning_bid("G3", 100.0, 8.0),
        ];
        let result =
            TsoMarket::clear_single_product(&bids, &AncillaryProduct::SpinningReserve, 150.0);
        // G2 (price=2) should appear first in accepted_bids
        assert_eq!(result.accepted_bids[0].unit_id, "G2");
        assert_eq!(result.accepted_bids[1].unit_id, "G1");
        // G3 should not be accepted (G2+G1 = 200 > 150)
        assert_eq!(result.accepted_bids.len(), 2);
    }

    // ── Test 2: Clearing price equals the marginal bid ────────────────────────
    #[test]
    fn test_clearing_price_is_marginal_bid() {
        // Requirement 150 MW; G1=100 MW @$2, G2=100 MW @$5
        // G1 fully accepted, G2 partially accepted → marginal = $5
        let bids = vec![
            spinning_bid("G1", 100.0, 2.0),
            spinning_bid("G2", 100.0, 5.0),
        ];
        let result =
            TsoMarket::clear_single_product(&bids, &AncillaryProduct::SpinningReserve, 150.0);
        assert!(
            (result.clearing_price_per_mw_h - 5.0).abs() < 1e-9,
            "Clearing price must equal marginal bid $5, got {}",
            result.clearing_price_per_mw_h
        );
        assert!(
            (result.total_capacity_cleared_mw - 150.0).abs() < 1e-9,
            "Must clear exactly 150 MW"
        );
    }

    // ── Test 3: Shortfall detection ───────────────────────────────────────────
    #[test]
    fn test_shortfall_when_supply_insufficient() {
        let bids = vec![spinning_bid("G1", 50.0, 3.0), spinning_bid("G2", 30.0, 4.0)];
        // Requirement 150 MW but only 80 MW offered
        let result =
            TsoMarket::clear_single_product(&bids, &AncillaryProduct::SpinningReserve, 150.0);
        assert!(
            result.shortfall_mw > 0.0,
            "Shortfall must be positive when supply < requirement"
        );
        assert!(
            (result.shortfall_mw - 70.0).abs() < 1e-9,
            "Shortfall should be 70 MW, got {}",
            result.shortfall_mw
        );
        assert!((result.total_capacity_cleared_mw - 80.0).abs() < 1e-9);
    }

    // ── Test 4: HHI ≈ 3333 for three equal bidders ───────────────────────────
    #[test]
    fn test_hhi_three_equal_bidders() {
        let bids = vec![
            spinning_bid("G1", 100.0, 3.0),
            spinning_bid("G2", 100.0, 3.0),
            spinning_bid("G3", 100.0, 3.0),
        ];
        let result =
            TsoMarket::clear_single_product(&bids, &AncillaryProduct::SpinningReserve, 300.0);
        let hhi = TsoMarket::calculate_hhi(&result);
        // Three equal shares of 33.33% each → HHI = 3 × (33.33)² ≈ 3333
        assert!(
            (hhi - 3333.33).abs() < 1.0,
            "HHI should be ~3333 for three equal bidders, got {:.2}",
            hhi
        );
    }

    // ── Test 5: Settlement — availability + activation payments ──────────────
    #[test]
    fn test_settlement_payments() {
        let bids = vec![spinning_bid("G1", 100.0, 4.0)];
        let result =
            TsoMarket::clear_single_product(&bids, &AncillaryProduct::SpinningReserve, 100.0);
        let activations = vec![ActivationRecord {
            unit_id: "G1".to_string(),
            energy_delivered_mwh: 50.0,
            activation_price_per_mwh: 8.0,
        }];
        // interval = 1 h
        let reports = TsoMarket::settlement(&result, &activations, 1.0);
        assert_eq!(reports.len(), 1);
        let rep = &reports[0];
        // Availability: 100 MW × $4/MW-h × 1 h = $400
        assert!(
            (rep.availability_payment - 400.0).abs() < 1e-9,
            "Availability payment should be $400, got {}",
            rep.availability_payment
        );
        // Activation: 50 MWh × $8/MWh = $400
        assert!(
            (rep.activation_payment - 400.0).abs() < 1e-9,
            "Activation payment should be $400, got {}",
            rep.activation_payment
        );
        assert!(
            (rep.total_payment - 800.0).abs() < 1e-9,
            "Total payment should be $800, got {}",
            rep.total_payment
        );
    }

    // ── Test 6: Frequency nadir with known inputs ─────────────────────────────
    #[test]
    fn test_frequency_nadir_estimation() {
        // 3000 MW system, H=5s, D=1, f0=50 Hz
        // Disturbance = 300 MW (10% of system)
        // primary cleared = 150 MW → R_pu = (150/3000)*25 = 1.25
        // Δf_pu = -0.1 / (1 + 1.25) ≈ -0.04444
        // f_nadir = 50 × (1 - 0.04444) ≈ 47.78 Hz
        let market = default_market();
        let metrics = market.frequency_response_assessment(300.0, 150.0);

        // ROCOF: 50 × (300/3000) / (2×5) = 50×0.1/10 = 0.5 Hz/s
        assert!(
            (metrics.rocof_hz_per_s - 0.5).abs() < 1e-6,
            "ROCOF should be 0.5 Hz/s, got {}",
            metrics.rocof_hz_per_s
        );

        assert!(
            metrics.frequency_nadir_hz > 47.0 && metrics.frequency_nadir_hz < 50.0,
            "Nadir must be between 47 and 50 Hz, got {}",
            metrics.frequency_nadir_hz
        );
        assert!(
            metrics.time_to_nadir_s > 0.0,
            "Time to nadir must be positive"
        );
        assert!((metrics.recovery_time_s - 3.0 * metrics.time_to_nadir_s).abs() < 1e-9);
    }

    // ── Test 7: Co-optimisation — energy constrains ancillary capacity ────────
    #[test]
    fn test_co_optimize_energy_constrains_ancillary() {
        // G1: 200 MW total, 100 MW in energy → only 100 MW left for spinning
        // G2: 200 MW total, no energy bid    → full 200 MW for spinning
        let energy_bids = vec![
            EnergyBid {
                unit_id: "G1".to_string(),
                capacity_mw: 200.0,
                offer_price_per_mwh: 30.0,
                p_min_mw: 0.0,
            },
            EnergyBid {
                unit_id: "G2".to_string(),
                capacity_mw: 200.0,
                offer_price_per_mwh: 50.0,
                p_min_mw: 0.0,
            },
        ];
        let ancillary_bids = vec![
            spinning_bid("G1", 200.0, 3.0), // can only offer residual after energy
            spinning_bid("G2", 200.0, 4.0),
        ];
        let req = ProductRequirements {
            spinning_reserve_mw: 250.0,
            ..Default::default()
        };

        let (energy_dispatch, anc_results) =
            TsoMarket::co_optimize_energy_ancillary(&energy_bids, &ancillary_bids, 100.0, &req);

        // G1 dispatched 100 MW in energy → residual for ancillary = 100 MW
        let g1_energy = energy_dispatch.get("G1").copied().unwrap_or(0.0);
        assert!(
            (g1_energy - 100.0).abs() < 1e-9,
            "G1 should be dispatched 100 MW in energy, got {}",
            g1_energy
        );

        // Spinning reserve result
        let spin = anc_results
            .iter()
            .find(|r| r.product == AncillaryProduct::SpinningReserve)
            .expect("Spinning reserve result must exist");

        // G1 residual = 100 MW, G2 has full 200 MW; total available = 300 MW > 250 MW
        assert!(
            spin.shortfall_mw < 1e-9,
            "No shortfall expected when combined residual ≥ requirement"
        );

        // G1's accepted capacity must be ≤ 100 MW (residual)
        let g1_spin = spin
            .accepted_bids
            .iter()
            .find(|ab| ab.unit_id == "G1")
            .map(|ab| ab.capacity_accepted_mw)
            .unwrap_or(0.0);
        assert!(
            g1_spin <= 100.0 + 1e-9,
            "G1 spinning reserve must not exceed residual capacity (100 MW), got {}",
            g1_spin
        );
    }

    // ── Test 8: Multi-product clearing in a single call ──────────────────────
    #[test]
    fn test_multi_product_clearing() {
        let mut market = TsoMarket::new(
            "MULTI-01",
            60.0,
            ProductRequirements {
                primary_mw: 50.0,
                secondary_mw: 80.0,
                tertiary_mw: 0.0, // not required
                spinning_reserve_mw: 100.0,
                non_spinning_reserve_mw: 0.0,
                reactive_mvar: 0.0,
            },
            50.0,
            5.0,
            2000.0,
            1.0,
        );

        let bids = vec![
            primary_bid("G1", 60.0, 3.0),
            primary_bid("G2", 60.0, 5.0),
            AncillaryBid {
                unit_id: "G3".to_string(),
                product: AncillaryProduct::SecondaryReserve,
                capacity_mw: 100.0,
                availability_price_per_mw_h: 4.0,
                activation_price_per_mwh: 8.0,
                min_delivery_min: 30.0,
                max_delivery_min: 900.0,
                ramp_rate_mw_per_min: 2.0,
                lead_time_min: 2.0,
            },
            spinning_bid("G4", 120.0, 6.0),
        ];

        let results = market.clear_market(&bids);

        // Must produce results for all 3 required products (primary, secondary, spinning)
        assert_eq!(results.len(), 3, "Expected 3 product results");

        let primary = results
            .iter()
            .find(|r| r.product == AncillaryProduct::PrimaryFrequencyResponse)
            .expect("Primary result missing");
        assert!(primary.total_capacity_cleared_mw >= 50.0 - 1e-9);

        let secondary = results
            .iter()
            .find(|r| r.product == AncillaryProduct::SecondaryReserve)
            .expect("Secondary result missing");
        assert!(secondary.total_capacity_cleared_mw >= 80.0 - 1e-9);

        let spinning = results
            .iter()
            .find(|r| r.product == AncillaryProduct::SpinningReserve)
            .expect("Spinning result missing");
        assert!(spinning.total_capacity_cleared_mw >= 100.0 - 1e-9);

        // History should be populated
        assert_eq!(market.clearing_history().len(), 3);
    }

    // ── Test 9: Zero-requirement products are not cleared ────────────────────
    #[test]
    fn test_zero_requirement_product_skipped() {
        let mut market = TsoMarket::new(
            "SKIP-01",
            60.0,
            ProductRequirements {
                primary_mw: 50.0,
                secondary_mw: 0.0, // zero requirement
                tertiary_mw: 0.0,
                spinning_reserve_mw: 0.0,
                non_spinning_reserve_mw: 0.0,
                reactive_mvar: 0.0,
            },
            50.0,
            5.0,
            1000.0,
            1.0,
        );
        let bids = vec![primary_bid("G1", 60.0, 3.0)];
        let results = market.clear_market(&bids);
        assert_eq!(results.len(), 1, "Only primary should be cleared");
        assert_eq!(
            results[0].product,
            AncillaryProduct::PrimaryFrequencyResponse
        );
    }

    // ── Test 10: HHI = 10000 for a monopoly ──────────────────────────────────
    #[test]
    fn test_hhi_monopoly() {
        let bids = vec![spinning_bid("G1", 100.0, 5.0)];
        let result =
            TsoMarket::clear_single_product(&bids, &AncillaryProduct::SpinningReserve, 100.0);
        let hhi = TsoMarket::calculate_hhi(&result);
        assert!(
            (hhi - 10_000.0).abs() < 1e-6,
            "Monopoly HHI must be 10 000, got {}",
            hhi
        );
    }
}
