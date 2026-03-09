//! Multi-Area Reserve Sharing
//!
//! Implements cross-border reserve exchange, emergency sharing agreements,
//! co-optimization of local vs. imported reserves, and interconnection
//! constraint enforcement.
//!
//! Units: MW, \[$/MW·h\], \[h\]

use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Public data structures
// ---------------------------------------------------------------------------

/// A balancing control area with generation, load, and reserve data.
#[derive(Debug, Clone)]
pub struct ControlArea {
    /// Unique area identifier.
    pub area_id: String,
    /// Total load \[MW\].
    pub total_load_mw: f64,
    /// Total online generation \[MW\].
    pub total_generation_mw: f64,
    /// Spinning reserve currently available \[MW\].
    pub local_reserve_mw: f64,
    /// Regulatory reserve requirement \[MW\].
    pub reserve_requirement_mw: f64,
    /// Neighbouring areas and available tie-line capacity \[MW\].
    /// Each element is `(neighbour_area_id, capacity_mw)`.
    pub interconnection_capacity_mw: Vec<(String, f64)>,
    /// Current reserve deficit (`> 0` means shortage) \[MW\].
    pub reserve_shortage_mw: f64,
}

/// Bilateral or multilateral reserve sharing agreement.
#[derive(Debug, Clone)]
pub struct ReserveSharingAgreement {
    /// Unique agreement identifier.
    pub agreement_id: String,
    /// Area IDs that are party to this agreement.
    pub areas: Vec<String>,
    /// Maximum fraction of local reserve that may be exported (default 0.50).
    pub max_shared_reserve_pct: f64,
    /// Sharing is activated when a deficit exceeds this value \[MW\].
    pub activation_trigger_mw: f64,
    /// Compensation rate \[$/MW·h\].
    pub price_per_mw_h: f64,
    /// Agreement priority (1 = highest priority).
    pub priority: u8,
    /// Agreement duration \[h\] (default 1.0).
    pub duration_h: f64,
}

/// An emergency reserve request issued by a control area.
#[derive(Debug, Clone)]
pub struct EmergencyShareRequest {
    /// Area that needs reserve.
    pub requesting_area: String,
    /// Reserve volume required \[MW\].
    pub required_mw: f64,
    /// Urgency level (1 = most urgent).
    pub urgency_level: u8,
    /// Maximum price the requesting area is willing to pay \[$/MW·h\].
    pub max_price_per_mw_h: f64,
    /// Simulation timestamp \[s\] or epoch seconds.
    pub timestamp: f64,
}

/// Configuration knobs for the reserve coordinator.
#[derive(Debug, Clone)]
pub struct ReserveSharingConfig {
    /// Optimisation method used in [`MultiAreaReserveCoordinator::optimize_sharing`].
    pub optimization_method: OptMethod,
    /// Probability that a tie-line is available when needed (default 0.95).
    pub interconnection_reliability: f64,
    /// Fraction of imported reserve credited toward local requirement (default 0.8).
    pub reserve_credit_factor: f64,
    /// Length of a settlement interval \[h\] (default 1.0).
    pub settlement_interval_h: f64,
}

impl Default for ReserveSharingConfig {
    fn default() -> Self {
        Self {
            optimization_method: OptMethod::CostMinimizing,
            interconnection_reliability: 0.95,
            reserve_credit_factor: 0.80,
            settlement_interval_h: 1.0,
        }
    }
}

/// Optimisation strategy for the sharing plan.
#[derive(Debug, Clone, PartialEq)]
pub enum OptMethod {
    /// Share proportionally to each provider's surplus capacity.
    ProRataSharing,
    /// Dispatch the cheapest providers first (merit order).
    CostMinimizing,
    /// Linear-program approximation (greedy merit order with capacity limits).
    LpBased,
}

// ---------------------------------------------------------------------------
// Internal record
// ---------------------------------------------------------------------------

/// A single executed reserve transfer.
#[derive(Debug, Clone)]
pub struct ReserveTransaction {
    /// Area supplying the reserve.
    pub provider_area: String,
    /// Area receiving the reserve.
    pub receiver_area: String,
    /// Volume transferred \[MW\].
    pub mw: f64,
    /// Agreed price \[$/MW·h\].
    pub price_per_mw_h: f64,
    /// Sequence of areas on the physical path (for multi-hop transfers).
    pub interconnection_path: Vec<String>,
}

// ---------------------------------------------------------------------------
// Output types
// ---------------------------------------------------------------------------

/// Reserve adequacy status for a single area.
#[derive(Debug, Clone)]
pub struct AreaAdequacy {
    /// Area identifier.
    pub area_id: String,
    /// Available spinning reserve \[MW\].
    pub local_reserve_mw: f64,
    /// Regulatory requirement \[MW\].
    pub requirement_mw: f64,
    /// `local_reserve_mw - requirement_mw`; negative means deficit \[MW\].
    pub surplus_mw: f64,
    /// True when surplus ≥ 0.
    pub adequate: bool,
}

/// A complete reserve-sharing plan.
#[derive(Debug, Clone)]
pub struct SharingPlan {
    /// Individual reserve transfers.
    pub transactions: Vec<ReserveTransaction>,
    /// Total volume shared across all transfers \[MW\].
    pub total_mw_shared: f64,
    /// Total deficit volume that the plan covers \[MW\].
    pub deficit_covered_mw: f64,
    /// Area IDs that remain deficient after the plan.
    pub areas_still_deficient: Vec<String>,
    /// Aggregate cost of all transfers \[$/h\].
    pub total_cost_usd_per_h: f64,
}

/// Result of an emergency reserve activation.
#[derive(Debug, Clone)]
pub struct EmergencyResponse {
    /// Identifier referencing the original request.
    pub request: String,
    /// Total reserve secured \[MW\].
    pub mw_secured: f64,
    /// Area IDs that contributed reserve.
    pub providers: Vec<String>,
    /// Estimated time until reserve is fully activated \[min\].
    pub time_to_activate_min: f64,
    /// Aggregate cost of emergency sharing \[$/h\].
    pub cost_per_h: f64,
    /// True when the full requested volume was secured.
    pub fully_covered: bool,
}

/// Financial settlement for one area over a settlement interval.
#[derive(Debug, Clone)]
pub struct Settlement {
    /// Area identifier.
    pub area_id: String,
    /// Revenue received from other areas \[$/h × duration\].
    pub payments_received: f64,
    /// Payments made to other areas \[$/h × duration\].
    pub payments_made: f64,
    /// `payments_received - payments_made` (positive = net receiver).
    pub net_settlement: f64,
}

/// Aggregated system-wide reserve adequacy metrics.
#[derive(Debug, Clone)]
pub struct SystemAdequacyReport {
    /// Estimated loss-of-load probability (0–1) accounting for sharing.
    pub system_lolp: f64,
    /// Area IDs still inadequate after sharing.
    pub areas_with_inadequacy: Vec<String>,
    /// Total spinning reserve across all areas \[MW\].
    pub total_reserve_mw: f64,
    /// Fraction of N-1 contingency events covered (0–1).
    pub coverage_ratio: f64,
}

// ---------------------------------------------------------------------------
// Coordinator
// ---------------------------------------------------------------------------

/// Coordinates multi-area reserve sharing across control areas.
pub struct MultiAreaReserveCoordinator {
    /// All participating control areas.
    pub areas: Vec<ControlArea>,
    /// Active sharing agreements.
    pub agreements: Vec<ReserveSharingAgreement>,
    /// Coordinator configuration.
    pub config: ReserveSharingConfig,
    /// Immutable log of executed transactions.
    transaction_log: Vec<ReserveTransaction>,
}

impl MultiAreaReserveCoordinator {
    /// Create a new coordinator with the given areas, agreements, and config.
    pub fn new(
        areas: Vec<ControlArea>,
        agreements: Vec<ReserveSharingAgreement>,
        config: ReserveSharingConfig,
    ) -> Self {
        Self {
            areas,
            agreements,
            config,
            transaction_log: Vec::new(),
        }
    }

    // -----------------------------------------------------------------------
    // 1. Reserve adequacy assessment
    // -----------------------------------------------------------------------

    /// Assess the reserve adequacy of every control area.
    ///
    /// Returns one [`AreaAdequacy`] per area; `surplus_mw < 0` signals a deficit.
    pub fn assess_reserve_adequacy(&self) -> Vec<AreaAdequacy> {
        self.areas
            .iter()
            .map(|a| {
                let surplus = a.local_reserve_mw - a.reserve_requirement_mw;
                AreaAdequacy {
                    area_id: a.area_id.clone(),
                    local_reserve_mw: a.local_reserve_mw,
                    requirement_mw: a.reserve_requirement_mw,
                    surplus_mw: surplus,
                    adequate: surplus >= 0.0,
                }
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // 2. Sharing plan optimisation
    // -----------------------------------------------------------------------

    /// Build an optimal (or near-optimal) reserve-sharing plan.
    ///
    /// `emergency_requests` can optionally bias which deficits are served first.
    pub fn optimize_sharing(
        &mut self,
        emergency_requests: &[EmergencyShareRequest],
    ) -> SharingPlan {
        let adequacy = self.assess_reserve_adequacy();

        // Build surplus/deficit maps (indexed by area_id).
        // available_to_export[area] = exportable headroom after covering own requirement
        let mut available_export: HashMap<String, f64> = HashMap::new();
        let mut deficit_map: HashMap<String, f64> = HashMap::new();

        for ad in &adequacy {
            if ad.surplus_mw > 0.0 {
                // Limit by max_shared_reserve_pct from the best matching agreement.
                let max_pct = self
                    .agreements
                    .iter()
                    .filter(|ag| ag.areas.contains(&ad.area_id))
                    .map(|ag| ag.max_shared_reserve_pct)
                    .fold(f64::NAN, f64::max);
                let pct = if max_pct.is_nan() { 0.5 } else { max_pct };
                // Exportable = min(surplus, pct * local_reserve)
                let exportable = ad.surplus_mw.min(pct * ad.local_reserve_mw);
                if exportable > 0.0 {
                    available_export.insert(ad.area_id.clone(), exportable);
                }
            } else if ad.surplus_mw < 0.0 {
                deficit_map.insert(ad.area_id.clone(), -ad.surplus_mw);
            }
        }

        // Elevate deficits signalled by emergency requests.
        for req in emergency_requests {
            let entry = deficit_map
                .entry(req.requesting_area.clone())
                .or_insert(0.0);
            if req.required_mw > *entry {
                *entry = req.required_mw;
            }
        }

        // Sort deficit areas so that emergency-request areas (highest urgency) go first.
        let mut deficit_areas: Vec<String> = deficit_map.keys().cloned().collect();
        {
            let urgency_map: HashMap<String, u8> = emergency_requests
                .iter()
                .map(|r| (r.requesting_area.clone(), r.urgency_level))
                .collect();
            deficit_areas.sort_by(|a, b| {
                let ua = urgency_map.get(a).copied().unwrap_or(u8::MAX);
                let ub = urgency_map.get(b).copied().unwrap_or(u8::MAX);
                ua.cmp(&ub).then(a.cmp(b))
            });
        }

        let mut transactions: Vec<ReserveTransaction> = Vec::new();
        let mut total_covered = 0.0_f64;

        for receiver in &deficit_areas {
            let mut remaining = match deficit_map.get(receiver) {
                Some(&d) => d,
                None => continue,
            };
            if remaining <= 0.0 {
                continue;
            }

            // Collect candidate providers: areas that (a) have exportable headroom
            // and (b) are connected to `receiver` via a valid tie-line.
            let mut candidates: Vec<(String, f64, f64)> = Vec::new(); // (area_id, available_mw, price)
            for (provider, &headroom) in &available_export {
                if provider == receiver {
                    continue;
                }
                let cap = self.interconnection_capacity(provider, receiver);
                if cap <= 0.0 {
                    continue;
                }
                let transferable = headroom.min(cap * self.config.interconnection_reliability);
                if transferable <= 0.0 {
                    continue;
                }
                let price = self.agreement_price(provider, receiver);
                candidates.push((provider.clone(), transferable, price));
            }

            // Sort providers according to the chosen method.
            match self.config.optimization_method {
                OptMethod::ProRataSharing => {
                    // No sort needed; we'll distribute proportionally below.
                    // Stable order by area_id for determinism.
                    candidates.sort_by(|a, b| a.0.cmp(&b.0));
                }
                OptMethod::CostMinimizing | OptMethod::LpBased => {
                    // Cheapest first (merit order).
                    candidates
                        .sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));
                }
            }

            if self.config.optimization_method == OptMethod::ProRataSharing {
                // Distribute demand proportionally to each provider's headroom.
                let total_available: f64 = candidates.iter().map(|c| c.1).sum();
                if total_available <= 0.0 {
                    continue;
                }
                for (provider, avail, price) in &candidates {
                    if remaining <= 0.0 {
                        break;
                    }
                    let share = (avail / total_available) * remaining;
                    let transferred = share.min(*avail);
                    if transferred <= 0.0 {
                        continue;
                    }
                    let credited = transferred * self.config.reserve_credit_factor;
                    let actual = credited.min(remaining);
                    if actual <= 0.0 {
                        continue;
                    }
                    transactions.push(ReserveTransaction {
                        provider_area: provider.clone(),
                        receiver_area: receiver.clone(),
                        mw: actual,
                        price_per_mw_h: *price,
                        interconnection_path: vec![provider.clone(), receiver.clone()],
                    });
                    remaining -= actual;
                    total_covered += actual;
                    if let Some(v) = available_export.get_mut(provider) {
                        *v -= transferred;
                    }
                }
            } else {
                // Merit-order (sequential) dispatch.
                for (provider, avail, price) in &candidates {
                    if remaining <= 0.0 {
                        break;
                    }
                    let transferred = avail.min(remaining / self.config.reserve_credit_factor);
                    if transferred <= 0.0 {
                        continue;
                    }
                    let credited = transferred * self.config.reserve_credit_factor;
                    let actual = credited.min(remaining);
                    transactions.push(ReserveTransaction {
                        provider_area: provider.clone(),
                        receiver_area: receiver.clone(),
                        mw: actual,
                        price_per_mw_h: *price,
                        interconnection_path: vec![provider.clone(), receiver.clone()],
                    });
                    remaining -= actual;
                    total_covered += actual;
                    if let Some(h) = available_export.get_mut(provider) {
                        *h -= transferred;
                    }
                }
            }
        }

        // Determine still-deficient areas.
        let areas_still_deficient = deficit_areas
            .iter()
            .filter(|r| {
                let served: f64 = transactions
                    .iter()
                    .filter(|t| &&t.receiver_area == r)
                    .map(|t| t.mw)
                    .sum();
                let deficit = deficit_map.get(*r).copied().unwrap_or(0.0);
                served < deficit - 1e-6
            })
            .cloned()
            .collect();

        let total_mw_shared: f64 = transactions.iter().map(|t| t.mw).sum();
        let total_cost: f64 = transactions.iter().map(|t| t.mw * t.price_per_mw_h).sum();

        // Persist transactions.
        self.transaction_log.extend(transactions.clone());

        SharingPlan {
            transactions,
            total_mw_shared,
            deficit_covered_mw: total_covered,
            areas_still_deficient,
            total_cost_usd_per_h: total_cost,
        }
    }

    // -----------------------------------------------------------------------
    // 3. Emergency activation
    // -----------------------------------------------------------------------

    /// Respond to an emergency reserve request.
    ///
    /// Activates sharing agreements in priority order, respecting
    /// interconnection availability and the max-shared-reserve fraction.
    pub fn emergency_activation(&mut self, request: &EmergencyShareRequest) -> EmergencyResponse {
        let mut remaining = request.required_mw;
        let mut mw_secured = 0.0_f64;
        let mut providers: Vec<String> = Vec::new();
        let mut total_cost = 0.0_f64;

        // Sort agreements by priority (lowest number = highest priority).
        let mut sorted_agreements = self.agreements.clone();
        sorted_agreements.sort_by_key(|ag| ag.priority);

        // Track how much we have already committed from each area.
        let mut committed: HashMap<String, f64> = HashMap::new();

        for agreement in &sorted_agreements {
            if remaining <= 0.0 {
                break;
            }
            if !agreement.areas.contains(&request.requesting_area) {
                continue;
            }
            if request.required_mw < agreement.activation_trigger_mw {
                continue;
            }
            if request.max_price_per_mw_h < agreement.price_per_mw_h {
                continue;
            }

            for provider_id in &agreement.areas {
                if remaining <= 0.0 {
                    break;
                }
                if provider_id == &request.requesting_area {
                    continue;
                }
                // Find the provider area.
                let provider_area = match self.areas.iter().find(|a| &a.area_id == provider_id) {
                    Some(a) => a.clone(),
                    None => continue,
                };

                // Check interconnection.
                let tie_cap = self.interconnection_capacity(provider_id, &request.requesting_area);
                if tie_cap <= 0.0 {
                    continue;
                }
                let effective_cap = tie_cap * self.config.interconnection_reliability;

                // How much can this provider export?
                let already_committed = committed.get(provider_id).copied().unwrap_or(0.0);
                let max_export = (provider_area.local_reserve_mw
                    * agreement.max_shared_reserve_pct)
                    - already_committed;
                if max_export <= 0.0 {
                    continue;
                }

                let available = max_export.min(effective_cap);
                let transferred = available.min(remaining / self.config.reserve_credit_factor);
                let credited = transferred * self.config.reserve_credit_factor;
                if credited <= 0.0 {
                    continue;
                }

                mw_secured += credited;
                remaining -= credited;
                total_cost += credited * agreement.price_per_mw_h;
                *committed.entry(provider_id.clone()).or_insert(0.0) += transferred;

                if !providers.contains(provider_id) {
                    providers.push(provider_id.clone());
                }

                self.transaction_log.push(ReserveTransaction {
                    provider_area: provider_id.clone(),
                    receiver_area: request.requesting_area.clone(),
                    mw: credited,
                    price_per_mw_h: agreement.price_per_mw_h,
                    interconnection_path: vec![
                        provider_id.clone(),
                        request.requesting_area.clone(),
                    ],
                });
            }
        }

        // Emergency activation target: ≤ 10 minutes.
        // Model activation time as 2 min base + 1 min per provider (capped at 10 min).
        let time_to_activate = (2.0 + providers.len() as f64).min(10.0);

        EmergencyResponse {
            request: request.requesting_area.clone(),
            mw_secured,
            providers,
            time_to_activate_min: time_to_activate,
            cost_per_h: total_cost,
            fully_covered: remaining <= 1e-6,
        }
    }

    // -----------------------------------------------------------------------
    // 4. Interconnection flow check
    // -----------------------------------------------------------------------

    /// Return `true` if `requested_mw` can physically flow from `from_area` to
    /// `to_area` accounting for tie-line reliability.
    pub fn interconnection_flow_check(
        &self,
        from_area: &str,
        to_area: &str,
        requested_mw: f64,
    ) -> bool {
        let cap = self.interconnection_capacity(from_area, to_area);
        let effective = cap * self.config.interconnection_reliability;
        requested_mw <= effective
    }

    // -----------------------------------------------------------------------
    // 5. Settlement calculation
    // -----------------------------------------------------------------------

    /// Compute financial settlements for a set of executed transactions.
    ///
    /// Each importing area pays; each exporting area receives.
    pub fn calculate_settlement(
        &self,
        executed_transactions: &[ReserveTransaction],
    ) -> Vec<Settlement> {
        let mut received: HashMap<String, f64> = HashMap::new();
        let mut paid: HashMap<String, f64> = HashMap::new();

        let duration = self.config.settlement_interval_h;

        for tx in executed_transactions {
            let payment = tx.mw * tx.price_per_mw_h * duration;
            *received.entry(tx.provider_area.clone()).or_insert(0.0) += payment;
            *paid.entry(tx.receiver_area.clone()).or_insert(0.0) += payment;
        }

        // Collect all area IDs that appear in any transaction.
        let mut all_areas: Vec<String> = executed_transactions
            .iter()
            .flat_map(|t| [t.provider_area.clone(), t.receiver_area.clone()])
            .collect::<std::collections::HashSet<_>>()
            .into_iter()
            .collect();
        all_areas.sort();

        all_areas
            .into_iter()
            .map(|area| {
                let r = received.get(&area).copied().unwrap_or(0.0);
                let p = paid.get(&area).copied().unwrap_or(0.0);
                Settlement {
                    area_id: area,
                    payments_received: r,
                    payments_made: p,
                    net_settlement: r - p,
                }
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // 6. System adequacy metrics
    // -----------------------------------------------------------------------

    /// Compute system-wide adequacy metrics taking sharing agreements into account.
    pub fn adequacy_metrics(&self) -> SystemAdequacyReport {
        let adequacy = self.assess_reserve_adequacy();

        let total_reserve_mw: f64 = adequacy.iter().map(|a| a.local_reserve_mw).sum();
        let total_requirement: f64 = adequacy.iter().map(|a| a.requirement_mw).sum();

        // Simple LOLP model:
        // Areas that can fully cover their deficit through sharing contribute 0 LOLP.
        // Areas still deficient contribute (deficit / requirement).
        let mut inadequate_areas: Vec<String> = Vec::new();
        let mut lolp_sum = 0.0_f64;
        let mut covered_events = 0_usize;
        let n_areas = adequacy.len();

        for ad in &adequacy {
            if ad.adequate {
                covered_events += 1;
                continue;
            }
            // Check whether a neighbour can cover the deficit.
            let can_be_covered = self.can_cover_via_sharing(&ad.area_id, -ad.surplus_mw);
            if can_be_covered {
                covered_events += 1;
            } else {
                inadequate_areas.push(ad.area_id.clone());
                let deficit = (-ad.surplus_mw).max(0.0);
                let req = ad.requirement_mw.max(1.0);
                lolp_sum += (deficit / req).min(1.0);
            }
        }

        let system_lolp = if n_areas == 0 {
            0.0
        } else {
            lolp_sum / n_areas as f64
        };

        let coverage_ratio = if n_areas == 0 {
            1.0
        } else {
            covered_events as f64 / n_areas as f64
        };

        // Penalise if total reserve is below total requirement.
        let system_lolp = if total_requirement > 0.0 && total_reserve_mw < total_requirement {
            let shortfall = (total_requirement - total_reserve_mw) / total_requirement;
            (system_lolp + shortfall * 0.5).min(1.0)
        } else {
            system_lolp
        };

        SystemAdequacyReport {
            system_lolp,
            areas_with_inadequacy: inadequate_areas,
            total_reserve_mw,
            coverage_ratio,
        }
    }

    // -----------------------------------------------------------------------
    // 7. Sensitivity analysis
    // -----------------------------------------------------------------------

    /// Estimate the change in adequacy surplus for every area if area
    /// `area_idx` changes its available reserve by `reserve_change_mw`.
    ///
    /// Returns a vector of adequacy-surplus deltas indexed by area position.
    pub fn sensitivity_analysis(&self, area_idx: usize, reserve_change_mw: f64) -> Vec<f64> {
        let n = self.areas.len();
        if area_idx >= n {
            return vec![0.0; n];
        }

        // Build a temporary coordinator with the perturbed area.
        let mut perturbed_areas = self.areas.clone();
        perturbed_areas[area_idx].local_reserve_mw += reserve_change_mw;

        let perturbed = MultiAreaReserveCoordinator {
            areas: perturbed_areas,
            agreements: self.agreements.clone(),
            config: self.config.clone(),
            transaction_log: Vec::new(),
        };

        let base_adequacy = self.assess_reserve_adequacy();
        let new_adequacy = perturbed.assess_reserve_adequacy();

        base_adequacy
            .iter()
            .zip(new_adequacy.iter())
            .map(|(b, n)| n.surplus_mw - b.surplus_mw)
            .collect()
    }

    // -----------------------------------------------------------------------
    // Accessor for the transaction log
    // -----------------------------------------------------------------------

    /// Return a reference to the immutable transaction log.
    pub fn transaction_log(&self) -> &[ReserveTransaction] {
        &self.transaction_log
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /// Return the bidirectional tie-line capacity \[MW\] between two areas.
    /// Checks both directions; returns 0.0 if no interconnection found.
    fn interconnection_capacity(&self, from: &str, to: &str) -> f64 {
        // Check the from-area's declared interconnections first.
        for area in &self.areas {
            if area.area_id == from {
                for (neighbour, cap) in &area.interconnection_capacity_mw {
                    if neighbour == to {
                        return *cap;
                    }
                }
            }
            if area.area_id == to {
                for (neighbour, cap) in &area.interconnection_capacity_mw {
                    if neighbour == from {
                        return *cap;
                    }
                }
            }
        }
        0.0
    }

    /// Return the reserve sharing price \[$/MW·h\] for a given provider→receiver pair.
    /// Uses the highest-priority agreement that covers both areas; falls back to 0.
    fn agreement_price(&self, provider: &str, receiver: &str) -> f64 {
        let mut best: Option<(u8, f64)> = None;
        for ag in &self.agreements {
            if ag.areas.contains(&provider.to_string()) && ag.areas.contains(&receiver.to_string())
            {
                match best {
                    None => best = Some((ag.priority, ag.price_per_mw_h)),
                    Some((p, _)) if ag.priority < p => {
                        best = Some((ag.priority, ag.price_per_mw_h));
                    }
                    _ => {}
                }
            }
        }
        best.map(|(_, price)| price).unwrap_or(0.0)
    }

    /// Check whether surplus neighbours can cover `deficit_mw` for the given area.
    fn can_cover_via_sharing(&self, area_id: &str, deficit_mw: f64) -> bool {
        let mut coverable = 0.0_f64;
        for area in &self.areas {
            if area.area_id == area_id {
                continue;
            }
            let surplus = area.local_reserve_mw - area.reserve_requirement_mw;
            if surplus <= 0.0 {
                continue;
            }
            let cap = self.interconnection_capacity(&area.area_id, area_id);
            if cap <= 0.0 {
                continue;
            }
            let exportable = surplus.min(cap * self.config.interconnection_reliability)
                * self.config.reserve_credit_factor;
            coverable += exportable;
            if coverable >= deficit_mw {
                return true;
            }
        }
        false
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a canonical two-area test scenario.
    ///
    /// - Area A: 100 MW surplus
    /// - Area B: 50 MW deficit
    /// - Tie-line A→B: 200 MW
    fn two_area_scenario() -> MultiAreaReserveCoordinator {
        let area_a = ControlArea {
            area_id: "A".to_string(),
            total_load_mw: 500.0,
            total_generation_mw: 600.0,
            local_reserve_mw: 200.0,
            reserve_requirement_mw: 100.0,
            interconnection_capacity_mw: vec![("B".to_string(), 200.0)],
            reserve_shortage_mw: 0.0,
        };
        let area_b = ControlArea {
            area_id: "B".to_string(),
            total_load_mw: 400.0,
            total_generation_mw: 380.0,
            local_reserve_mw: 50.0,
            reserve_requirement_mw: 100.0,
            interconnection_capacity_mw: vec![("A".to_string(), 200.0)],
            reserve_shortage_mw: 50.0,
        };
        let agreement = ReserveSharingAgreement {
            agreement_id: "AG1".to_string(),
            areas: vec!["A".to_string(), "B".to_string()],
            max_shared_reserve_pct: 0.5,
            activation_trigger_mw: 10.0,
            price_per_mw_h: 20.0,
            priority: 1,
            duration_h: 1.0,
        };
        let config = ReserveSharingConfig {
            optimization_method: OptMethod::CostMinimizing,
            interconnection_reliability: 1.0,
            reserve_credit_factor: 1.0,
            settlement_interval_h: 1.0,
        };
        MultiAreaReserveCoordinator::new(vec![area_a, area_b], vec![agreement], config)
    }

    // -----------------------------------------------------------------------
    // Test 1 – Adequacy: area with 100 MW surplus identified correctly
    // -----------------------------------------------------------------------
    #[test]
    fn test_adequacy_surplus_identified() {
        let coord = two_area_scenario();
        let adequacy = coord.assess_reserve_adequacy();
        let a = adequacy
            .iter()
            .find(|x| x.area_id == "A")
            .expect("Area A missing");
        assert!(a.adequate, "Area A should be adequate");
        assert!(
            (a.surplus_mw - 100.0).abs() < 1e-9,
            "Expected 100 MW surplus, got {}",
            a.surplus_mw
        );
    }

    // -----------------------------------------------------------------------
    // Test 2 – Deficit: area with -50 MW flagged as inadequate
    // -----------------------------------------------------------------------
    #[test]
    fn test_adequacy_deficit_flagged() {
        let coord = two_area_scenario();
        let adequacy = coord.assess_reserve_adequacy();
        let b = adequacy
            .iter()
            .find(|x| x.area_id == "B")
            .expect("Area B missing");
        assert!(!b.adequate, "Area B should be inadequate");
        assert!(
            (b.surplus_mw - (-50.0)).abs() < 1e-9,
            "Expected -50 MW surplus, got {}",
            b.surplus_mw
        );
    }

    // -----------------------------------------------------------------------
    // Test 3 – ProRata sharing: proportional to surplus
    // -----------------------------------------------------------------------
    #[test]
    fn test_pro_rata_sharing() {
        // Three areas: A 80 MW surplus, C 40 MW surplus, B 60 MW deficit.
        let area_a = ControlArea {
            area_id: "A".to_string(),
            total_load_mw: 300.0,
            total_generation_mw: 400.0,
            local_reserve_mw: 180.0,
            reserve_requirement_mw: 100.0,
            interconnection_capacity_mw: vec![("B".to_string(), 200.0)],
            reserve_shortage_mw: 0.0,
        };
        let area_b = ControlArea {
            area_id: "B".to_string(),
            total_load_mw: 300.0,
            total_generation_mw: 260.0,
            local_reserve_mw: 40.0,
            reserve_requirement_mw: 100.0,
            interconnection_capacity_mw: vec![("A".to_string(), 200.0), ("C".to_string(), 200.0)],
            reserve_shortage_mw: 60.0,
        };
        let area_c = ControlArea {
            area_id: "C".to_string(),
            total_load_mw: 200.0,
            total_generation_mw: 250.0,
            local_reserve_mw: 140.0,
            reserve_requirement_mw: 100.0,
            interconnection_capacity_mw: vec![("B".to_string(), 200.0)],
            reserve_shortage_mw: 0.0,
        };
        let agreement = ReserveSharingAgreement {
            agreement_id: "AG_PR".to_string(),
            areas: vec!["A".to_string(), "B".to_string(), "C".to_string()],
            max_shared_reserve_pct: 0.5,
            activation_trigger_mw: 10.0,
            price_per_mw_h: 15.0,
            priority: 1,
            duration_h: 1.0,
        };
        let config = ReserveSharingConfig {
            optimization_method: OptMethod::ProRataSharing,
            interconnection_reliability: 1.0,
            reserve_credit_factor: 1.0,
            settlement_interval_h: 1.0,
        };
        let mut coord =
            MultiAreaReserveCoordinator::new(vec![area_a, area_b, area_c], vec![agreement], config);
        let plan = coord.optimize_sharing(&[]);
        assert!(
            plan.deficit_covered_mw > 0.0,
            "Some deficit should be covered in pro-rata mode"
        );
        // Both A and C should appear as providers.
        let providers: Vec<_> = plan
            .transactions
            .iter()
            .map(|t| t.provider_area.as_str())
            .collect();
        assert!(providers.contains(&"A"), "Area A should be a provider");
        assert!(providers.contains(&"C"), "Area C should be a provider");
    }

    // -----------------------------------------------------------------------
    // Test 4 – Cost minimising: cheapest provider selected first
    // -----------------------------------------------------------------------
    #[test]
    fn test_cost_minimizing_cheapest_first() {
        // Two providers: expensive (E, $50/MW·h) and cheap (C, $10/MW·h).
        let area_exp = ControlArea {
            area_id: "Expensive".to_string(),
            total_load_mw: 200.0,
            total_generation_mw: 300.0,
            local_reserve_mw: 150.0,
            reserve_requirement_mw: 50.0,
            interconnection_capacity_mw: vec![("Deficit".to_string(), 200.0)],
            reserve_shortage_mw: 0.0,
        };
        let area_cheap = ControlArea {
            area_id: "Cheap".to_string(),
            total_load_mw: 200.0,
            total_generation_mw: 300.0,
            local_reserve_mw: 150.0,
            reserve_requirement_mw: 50.0,
            interconnection_capacity_mw: vec![("Deficit".to_string(), 200.0)],
            reserve_shortage_mw: 0.0,
        };
        let area_def = ControlArea {
            area_id: "Deficit".to_string(),
            total_load_mw: 300.0,
            total_generation_mw: 250.0,
            local_reserve_mw: 20.0,
            reserve_requirement_mw: 80.0,
            interconnection_capacity_mw: vec![
                ("Expensive".to_string(), 200.0),
                ("Cheap".to_string(), 200.0),
            ],
            reserve_shortage_mw: 60.0,
        };
        let ag_exp = ReserveSharingAgreement {
            agreement_id: "AG_EXP".to_string(),
            areas: vec!["Expensive".to_string(), "Deficit".to_string()],
            max_shared_reserve_pct: 0.6,
            activation_trigger_mw: 5.0,
            price_per_mw_h: 50.0,
            priority: 2,
            duration_h: 1.0,
        };
        let ag_cheap = ReserveSharingAgreement {
            agreement_id: "AG_CHEAP".to_string(),
            areas: vec!["Cheap".to_string(), "Deficit".to_string()],
            max_shared_reserve_pct: 0.6,
            activation_trigger_mw: 5.0,
            price_per_mw_h: 10.0,
            priority: 1,
            duration_h: 1.0,
        };
        let config = ReserveSharingConfig {
            optimization_method: OptMethod::CostMinimizing,
            interconnection_reliability: 1.0,
            reserve_credit_factor: 1.0,
            settlement_interval_h: 1.0,
        };
        let mut coord = MultiAreaReserveCoordinator::new(
            vec![area_exp, area_cheap, area_def],
            vec![ag_exp, ag_cheap],
            config,
        );
        let plan = coord.optimize_sharing(&[]);

        // Cheap should be dispatched first; expensive only if needed.
        let cheap_tx: f64 = plan
            .transactions
            .iter()
            .filter(|t| t.provider_area == "Cheap")
            .map(|t| t.mw)
            .sum();
        let exp_tx: f64 = plan
            .transactions
            .iter()
            .filter(|t| t.provider_area == "Expensive")
            .map(|t| t.mw)
            .sum();
        assert!(
            cheap_tx >= exp_tx,
            "Cheap provider should supply at least as much as expensive one"
        );
    }

    // -----------------------------------------------------------------------
    // Test 5 – Interconnection limit: can't transfer more than tie capacity
    // -----------------------------------------------------------------------
    #[test]
    fn test_interconnection_limit_enforced() {
        let mut coord = two_area_scenario();
        // Override tie capacity to 30 MW.
        coord.areas[0].interconnection_capacity_mw = vec![("B".to_string(), 30.0)];
        coord.areas[1].interconnection_capacity_mw = vec![("A".to_string(), 30.0)];

        let plan = coord.optimize_sharing(&[]);
        let transferred: f64 = plan.transactions.iter().map(|t| t.mw).sum();
        assert!(
            transferred <= 30.0 + 1e-9,
            "Transferred {} MW should not exceed tie capacity 30 MW",
            transferred
        );
    }

    // -----------------------------------------------------------------------
    // Test 6 – Emergency: urgent request activates high-priority agreement
    // -----------------------------------------------------------------------
    #[test]
    fn test_emergency_activation() {
        let mut coord = two_area_scenario();
        let request = EmergencyShareRequest {
            requesting_area: "B".to_string(),
            required_mw: 40.0,
            urgency_level: 1,
            max_price_per_mw_h: 100.0,
            timestamp: 0.0,
        };
        let response = coord.emergency_activation(&request);
        assert!(
            response.mw_secured > 0.0,
            "Emergency should secure some reserve"
        );
        assert!(
            response.time_to_activate_min <= 10.0,
            "Activation must complete within 10 minutes"
        );
    }

    // -----------------------------------------------------------------------
    // Test 7 – Settlement: importing area pays, exporting receives
    // -----------------------------------------------------------------------
    #[test]
    fn test_settlement_correct_direction() {
        let coord = two_area_scenario();
        let txs = vec![ReserveTransaction {
            provider_area: "A".to_string(),
            receiver_area: "B".to_string(),
            mw: 50.0,
            price_per_mw_h: 20.0,
            interconnection_path: vec!["A".to_string(), "B".to_string()],
        }];
        let settlements = coord.calculate_settlement(&txs);

        let a_s = settlements
            .iter()
            .find(|s| s.area_id == "A")
            .expect("A missing");
        let b_s = settlements
            .iter()
            .find(|s| s.area_id == "B")
            .expect("B missing");

        // Duration = 1 h, payment = 50 × 20 × 1 = 1000
        assert!(
            (a_s.payments_received - 1000.0).abs() < 1e-9,
            "A should receive $1000"
        );
        assert!(
            (b_s.payments_made - 1000.0).abs() < 1e-9,
            "B should pay $1000"
        );
        assert!(
            a_s.net_settlement > 0.0,
            "A net settlement should be positive (exporter)"
        );
        assert!(
            b_s.net_settlement < 0.0,
            "B net settlement should be negative (importer)"
        );
    }

    // -----------------------------------------------------------------------
    // Test 8 – System adequacy: all areas adequate → LOLP near 0
    // -----------------------------------------------------------------------
    #[test]
    fn test_system_adequacy_all_adequate() {
        // Both areas have surplus; no deficit.
        let area_a = ControlArea {
            area_id: "A".to_string(),
            total_load_mw: 400.0,
            total_generation_mw: 500.0,
            local_reserve_mw: 150.0,
            reserve_requirement_mw: 100.0,
            interconnection_capacity_mw: vec![("B".to_string(), 100.0)],
            reserve_shortage_mw: 0.0,
        };
        let area_b = ControlArea {
            area_id: "B".to_string(),
            total_load_mw: 300.0,
            total_generation_mw: 400.0,
            local_reserve_mw: 120.0,
            reserve_requirement_mw: 80.0,
            interconnection_capacity_mw: vec![("A".to_string(), 100.0)],
            reserve_shortage_mw: 0.0,
        };
        let config = ReserveSharingConfig::default();
        let coord = MultiAreaReserveCoordinator::new(vec![area_a, area_b], vec![], config);
        let report = coord.adequacy_metrics();
        assert!(
            report.system_lolp < 0.05,
            "LOLP should be near 0 when all areas adequate, got {}",
            report.system_lolp
        );
        assert!(
            report.areas_with_inadequacy.is_empty(),
            "No areas should be inadequate"
        );
    }

    // -----------------------------------------------------------------------
    // Test 9 – interconnection_flow_check helper
    // -----------------------------------------------------------------------
    #[test]
    fn test_flow_check_within_capacity() {
        let coord = two_area_scenario();
        assert!(
            coord.interconnection_flow_check("A", "B", 150.0),
            "150 MW should fit within 200 MW tie"
        );
        assert!(
            !coord.interconnection_flow_check("A", "B", 250.0),
            "250 MW should exceed 200 MW tie"
        );
    }

    // -----------------------------------------------------------------------
    // Test 10 – Sensitivity analysis
    // -----------------------------------------------------------------------
    #[test]
    fn test_sensitivity_analysis() {
        let coord = two_area_scenario();
        // Perturb area 0 (A) by +10 MW reserve.
        let deltas = coord.sensitivity_analysis(0, 10.0);
        assert_eq!(deltas.len(), 2, "Should return delta for each area");
        // Area 0 (A) should see +10 MW change in its own surplus.
        assert!(
            (deltas[0] - 10.0).abs() < 1e-9,
            "Area A surplus delta should be +10 MW"
        );
        // Area B is unaffected directly.
        assert!(
            deltas[1].abs() < 1e-9,
            "Area B surplus delta should be ~0 MW"
        );
    }
}
