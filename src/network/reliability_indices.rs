//! IEEE 1366 distribution reliability indices and bulk power system reliability metrics.
//!
//! Implements the full suite of distribution-level reliability indices as defined in
//! IEEE Std 1366-2012 (`CustomerData`, `InterruptionEvent`, `ReliabilityCalculator`),
//! plus generation-adequacy bulk-system metrics using a capacity-outage-table (COT)
//! convolution method and Monte Carlo simulation driven by an LCG PRNG.
//!
//! # Quick Start
//!
//! ```rust
//! use oxigrid::network::reliability_indices::{
//!     CustomerData, InterruptionEvent, InterruptionCause, ReliabilityCalculator,
//! };
//!
//! let customers = vec![
//!     CustomerData { id: 0, n_customers: 100, load_kw: 200.0, feeder_id: 0, substation_id: 0 },
//! ];
//! let events = vec![
//!     InterruptionEvent {
//!         id: 0, start_time: 0.0, duration_h: 1.0,
//!         affected_customers: vec![0], cause: InterruptionCause::OverheadLine,
//!         sustained: true, feeder_id: 0,
//!     },
//! ];
//! let calc = ReliabilityCalculator::new(customers, events, 1.0);
//! let idx  = calc.compute_indices();
//! assert!(idx.saidi > 0.0);
//! ```

use std::collections::{BTreeMap, HashMap};

// ═════════════════════════════════════════════════════════════════════════════
// ── IEEE 1366 Distribution Reliability Domain Types ──────────────────────────
// ═════════════════════════════════════════════════════════════════════════════

/// Root causes of a power interruption (IEEE 1366 cause codes).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InterruptionCause {
    /// Faults on overhead distribution lines.
    OverheadLine,
    /// Faults on underground cable systems.
    UndergroundCable,
    /// Substation or customer equipment failures.
    Equipment,
    /// Planned / scheduled maintenance outage.
    ScheduledMaintenance,
    /// Causes external to the utility (weather, animals, third-party).
    ExternalCause,
    /// Cause unknown or not yet determined.
    Unknown,
}

/// Demographic and electrical data for a customer group served from a feeder.
///
/// In IEEE 1366 the unit of analysis is typically a *customer count*, not an
/// individual meter.  A single `CustomerData` record may therefore represent
/// many physical meters behind the same service point.
#[derive(Debug, Clone)]
pub struct CustomerData {
    /// Unique identifier (matches IDs used in [`InterruptionEvent::affected_customers`]).
    pub id: usize,
    /// Number of individual meters / end-users represented by this record.
    pub n_customers: u64,
    /// Average connected load for this customer group (kW).
    pub load_kw: f64,
    /// Feeder segment that supplies this customer group.
    pub feeder_id: usize,
    /// Upstream substation.
    pub substation_id: usize,
}

/// A single interruption event affecting one or more customer groups.
///
/// Sustained interruptions (> 5 minutes) are used for SAIDI/SAIFI; momentary
/// interruptions (≤ 5 minutes) are counted separately for MAIFI.
#[derive(Debug, Clone)]
pub struct InterruptionEvent {
    /// Unique event identifier.
    pub id: usize,
    /// Event start time expressed as hours from the epoch of the observation period.
    pub start_time: f64,
    /// Total restoration time in hours.
    pub duration_h: f64,
    /// IDs of [`CustomerData`] records affected by this event.
    pub affected_customers: Vec<usize>,
    /// Root cause classification.
    pub cause: InterruptionCause,
    /// `true` if the event is *sustained* (duration > 5 min).
    pub sustained: bool,
    /// Feeder on which the interruption originated.
    pub feeder_id: usize,
}

// ═════════════════════════════════════════════════════════════════════════════
// ── IEEE 1366 Index Structs ───────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════

/// IEEE 1366-2012 system-level distribution reliability indices.
///
/// All frequency-based indices (SAIFI, MAIFI, …) are expressed in
/// *interruptions per customer per observation period*.
#[derive(Debug, Clone)]
pub struct ReliabilityIndices {
    /// System Average Interruption Duration Index (h / customer).
    ///
    /// `SAIDI = Σ(N_i · r_i) / N_T`
    pub saidi: f64,

    /// System Average Interruption Frequency Index (interruptions / customer).
    ///
    /// Counts *sustained* interruptions only.
    /// `SAIFI = Σ(N_i) / N_T`
    pub saifi: f64,

    /// Customer Average Interruption Duration Index (h / interruption).
    ///
    /// `CAIDI = SAIDI / SAIFI`  (defined as 0 when SAIFI = 0).
    pub caidi: f64,

    /// Average Service Availability Index (fraction, 0–1).
    ///
    /// `ASAI = 1 − SAIDI / (8760 · period_years)`
    pub asai: f64,

    /// Average Service Unavailability Index.
    ///
    /// `ASUI = 1 − ASAI`
    pub asui: f64,

    /// Momentary Average Interruption Frequency Index.
    ///
    /// Counts interruptions with `sustained = false`.
    pub maifi: f64,

    /// MAIFI Extended — momentary events per customer per year.
    pub maifi_e: f64,

    /// Customers Experiencing Multiple Interruptions (fraction of total).
    ///
    /// Default threshold N = 3 interruptions per period.
    pub cemi_n: f64,

    /// Customers Experiencing Long Interruption Durations (fraction of total).
    ///
    /// Default threshold L = 2 hours cumulative outage per period.
    pub celid: f64,

    /// Customer Damage Index — EENS (kWh) normalised by total_load_kw × SAIDI.
    /// Dimensionless; 0 when denominator is zero.
    pub cdi: f64,

    /// Expected Energy Not Supplied (MWh) over the observation period.
    ///
    /// `EENS = Σ(P_i · r_i) / 1000`  where P_i is load in kW.
    pub eens_mwh: f64,

    /// Expected Number of Non-Supplied customers (non-standard extension).
    ///
    /// Sum of customer-interruptions annualised by observation period.
    pub enns: f64,
}

impl Default for ReliabilityIndices {
    fn default() -> Self {
        ReliabilityIndices {
            saidi: 0.0,
            saifi: 0.0,
            caidi: 0.0,
            asai: 1.0,
            asui: 0.0,
            maifi: 0.0,
            maifi_e: 0.0,
            cemi_n: 0.0,
            celid: 0.0,
            cdi: 0.0,
            eens_mwh: 0.0,
            enns: 0.0,
        }
    }
}

/// Per-feeder reliability indices, complementing the system aggregate.
#[derive(Debug, Clone)]
pub struct FeederReliability {
    /// Feeder identifier.
    pub feeder_id: usize,
    /// IEEE 1366 indices computed for this feeder's customers only.
    pub indices: ReliabilityIndices,
    /// Total number of customers on this feeder.
    pub n_customers: u64,
    /// Total installed load on this feeder (kW).
    pub total_load_kw: f64,
}

// ═════════════════════════════════════════════════════════════════════════════
// ── Bulk System Reliability Types ────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════

/// Generating unit data used for bulk reliability Monte Carlo assessment.
#[derive(Debug, Clone)]
pub struct GenerationUnit {
    /// Unique unit identifier.
    pub id: usize,
    /// Nameplate capacity (MW).
    pub capacity_mw: f64,
    /// Forced Outage Rate (fraction, 0–1).
    ///
    /// Probability that the unit is unavailable at any given hour.
    pub forced_outage_rate: f64,
    /// Mean time to repair after a forced outage (hours).
    pub mean_time_to_repair_h: f64,
    /// Mean time between successive forced outages (hours).
    pub mean_time_between_failures_h: f64,
}

/// Generation-adequacy bulk reliability indices.
///
/// All annual figures assume an 8 760-hour year.
#[derive(Debug, Clone)]
pub struct BulkSystemReliability {
    /// Loss of Load Probability — fraction of hours with insufficient capacity.
    pub lolp: f64,
    /// Loss of Load Expectation (days / year).
    ///
    /// `LOLE = LOLP × 8760 / 24`
    pub lole_days: f64,
    /// Loss of Energy Expectation (MWh / year).
    pub loee_mwh: f64,
    /// Expected Unserved Energy (MWh / year) — synonym for LOEE in many standards.
    pub eue_mwh: f64,
    /// Loss of Load Frequency (events / year).
    pub lolf: f64,
    /// Loss of Load Duration (hours / event).
    ///
    /// `LOLD = LOLE × 24 / LOLF`  (0 when LOLF = 0).
    pub lold_h: f64,
    /// Reserve Shortage Capacity — average unserved power during loss-of-load hours (MW).
    pub rsc: f64,
    /// Loss of Load Cost (USD / year).
    ///
    /// `LOLC = VOLL × EUE`
    pub lolc_usd: f64,
}

// ═════════════════════════════════════════════════════════════════════════════
// ── Legacy COT / analytical bulk-reliability types (preserved) ────────────────
// ═════════════════════════════════════════════════════════════════════════════

/// A partial (derated) operating state of a generating unit.
#[derive(Debug, Clone)]
pub struct DeratedState {
    /// Available capacity in this derated state (MW).
    pub available_mw: f64,
    /// Probability of being in this derated state.
    pub probability: f64,
}

/// A single generating unit with full reliability parameters (legacy COT model).
#[derive(Debug, Clone)]
pub struct GeneratingUnit {
    /// Unique identifier string.
    pub unit_id: String,
    /// Nameplate capacity (MW).
    pub capacity_mw: f64,
    /// Forced Outage Rate = λ/(λ+μ) ∈ [0, 1].
    pub forced_outage_rate: f64,
    /// Mean time to repair (h).
    pub mean_time_to_repair_h: f64,
    /// Mean time to fail (h).
    pub mean_time_to_fail_h: f64,
    /// Optional partial-outage (derated) states; probabilities must sum ≤ 1 − FOR.
    pub derated_states: Vec<DeratedState>,
}

/// Hourly load profile for one year.
#[derive(Debug, Clone)]
pub struct LoadData {
    /// Chronological hourly load values (MW); typically 8 760 entries.
    pub hourly_load_mw: Vec<f64>,
    /// Annual peak load (MW).
    pub peak_load_mw: f64,
    /// Load factor = average load / peak load (dimensionless).
    pub load_factor: f64,
}

/// Configuration for legacy COT reliability calculations.
#[derive(Debug, Clone)]
pub struct ReliabilityConfig {
    /// Number of years to simulate in Monte Carlo.
    pub monte_carlo_years: usize,
    /// Load-not-met threshold that constitutes a loss-of-load event (MW).
    pub lolp_threshold_mw: f64,
    /// If `false`, copper-plate assumption (no transmission limits).
    pub include_transmission: bool,
    /// Seed for the internal LCG random-number generator.
    pub seed: u64,
}

impl Default for ReliabilityConfig {
    fn default() -> Self {
        Self {
            monte_carlo_years: 100,
            lolp_threshold_mw: 0.0,
            include_transmission: false,
            seed: 12345,
        }
    }
}

/// Discrete probability distribution over available system capacity.
#[derive(Debug, Clone)]
pub struct CapacityOutageTable {
    /// Discrete available-capacity levels (MW), sorted descending.
    pub capacity_mw: Vec<f64>,
    /// Probability mass at each capacity level.
    pub probability: Vec<f64>,
    /// CDF: P(available capacity ≤ c), parallel to `capacity_mw`.
    pub cumulative: Vec<f64>,
}

/// Results from a Monte Carlo generation-adequacy simulation (legacy API).
#[derive(Debug, Clone)]
pub struct MonteCarloReliabilityResult {
    /// Mean loss-of-load hours per year (h/year).
    pub lolh_per_year: f64,
    /// Mean expected energy not supplied (MWh/year).
    pub eens_mwh_per_year: f64,
    /// Loss-of-load probability (fraction of hours in outage).
    pub lolp: f64,
    /// 95 % confidence interval half-width for LOLH.
    pub lolh_ci_95: f64,
    /// 95 % confidence interval half-width for EENS.
    pub eens_ci_95: f64,
    /// Maximum hourly deficit observed across all simulated years (MW).
    pub peak_deficit_mw: f64,
    /// Number of years simulated.
    pub years_simulated: usize,
}

// ═════════════════════════════════════════════════════════════════════════════
// ── LCG helper ───────────────────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════

/// Advance the LCG state by one step and return a uniform sample in [0, 1).
#[inline]
fn lcg_next(state: &mut u64) -> f64 {
    *state = state
        .wrapping_mul(6_364_136_223_846_793_005_u64)
        .wrapping_add(1_442_695_040_888_963_407_u64);
    (*state >> 11) as f64 / (1_u64 << 53) as f64
}

// ═════════════════════════════════════════════════════════════════════════════
// ── Legacy COT helpers ────────────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════

/// Represent a single state in the outage table as (available_mw, probability).
type StateMap = BTreeMap<u64, f64>;

/// Encode a capacity in MW to a fixed-point integer key (3 decimal places).
#[inline]
fn mw_to_key(mw: f64) -> u64 {
    (mw * 1000.0).round() as u64
}

/// Decode a fixed-point key back to MW.
#[inline]
fn key_to_mw(key: u64) -> f64 {
    key as f64 / 1000.0
}

// ═════════════════════════════════════════════════════════════════════════════
// ── ReliabilityCalculator — IEEE 1366 + Bulk APIs ────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════

/// Main reliability calculator implementing IEEE 1366 distribution indices
/// and Monte Carlo bulk-system adequacy assessment.
///
/// # Example
///
/// ```rust
/// use oxigrid::network::reliability_indices::{
///     CustomerData, InterruptionEvent, InterruptionCause, ReliabilityCalculator,
/// };
/// let customers = vec![
///     CustomerData { id: 0, n_customers: 500, load_kw: 1000.0, feeder_id: 0, substation_id: 0 },
/// ];
/// let calc = ReliabilityCalculator::new(customers, vec![], 1.0);
/// let idx  = calc.compute_indices();
/// assert_eq!(idx.saidi, 0.0);
/// assert_eq!(idx.asai, 1.0);
/// ```
pub struct ReliabilityCalculator {
    /// Customer records for the study area.
    pub customers: Vec<CustomerData>,
    /// Interruption events recorded over the observation period.
    pub events: Vec<InterruptionEvent>,
    /// Length of the observation window (years).
    pub period_years: f64,
    /// Value of lost load (USD / MWh) used for bulk cost calculation.
    pub voll_usd_per_mwh: f64,
    /// Threshold N for the CEMI-N index (interruptions per period).
    pub cemi_n_threshold: usize,
    /// Threshold L for the CELID index (cumulative hours outage per period).
    pub celid_l_threshold_h: f64,
}

impl ReliabilityCalculator {
    /// Create a new calculator with default VOLL (10 000 USD/MWh) and default
    /// CEMI-N / CELID thresholds (N = 3, L = 2 h).
    pub fn new(
        customers: Vec<CustomerData>,
        events: Vec<InterruptionEvent>,
        period_years: f64,
    ) -> Self {
        ReliabilityCalculator {
            customers,
            events,
            period_years: period_years.max(f64::EPSILON),
            voll_usd_per_mwh: 10_000.0,
            cemi_n_threshold: 3,
            celid_l_threshold_h: 2.0,
        }
    }

    // ── internal helpers ─────────────────────────────────────────────────────

    /// Build a lookup map from customer ID → `&CustomerData`.
    fn customer_map(&self) -> HashMap<usize, &CustomerData> {
        self.customers.iter().map(|c| (c.id, c)).collect()
    }

    /// Total number of individual customers across all records.
    fn total_customers(&self) -> u64 {
        self.customers.iter().map(|c| c.n_customers).sum()
    }

    /// Compute per-customer-record cumulative outage duration and interruption
    /// count for CEMI-N and CELID index calculation.
    ///
    /// Returns `(interruption_count_map, cumulative_duration_map)` keyed by
    /// customer ID.
    fn per_customer_stats(
        &self,
        cmap: &HashMap<usize, &CustomerData>,
    ) -> (HashMap<usize, usize>, HashMap<usize, f64>) {
        let mut counts: HashMap<usize, usize> = HashMap::new();
        let mut durations: HashMap<usize, f64> = HashMap::new();

        for evt in &self.events {
            if !evt.sustained {
                continue;
            }
            for &cid in &evt.affected_customers {
                if cmap.contains_key(&cid) {
                    *counts.entry(cid).or_insert(0) += 1;
                    *durations.entry(cid).or_insert(0.0) += evt.duration_h;
                }
            }
        }
        (counts, durations)
    }

    // ── IEEE 1366 public API ──────────────────────────────────────────────────

    /// Compute all IEEE 1366-2012 system-level reliability indices.
    ///
    /// Returns [`ReliabilityIndices::default`] (ASAI = 1) when there are no
    /// customers or no events.
    pub fn compute_indices(&self) -> ReliabilityIndices {
        let n_t = self.total_customers();
        if n_t == 0 {
            return ReliabilityIndices::default();
        }
        let n_t_f = n_t as f64;
        let cmap = self.customer_map();

        // SAIDI / SAIFI / EENS accumulators
        let mut saidi_num = 0.0_f64; // Σ N_i · r_i
        let mut saifi_num = 0.0_f64; // Σ N_i  (sustained only)
        let mut maifi_num = 0.0_f64; // Σ N_i  (momentary only)
        let mut eens_kwh = 0.0_f64; // Σ P_i · r_i  (kWh)
        let mut enns_acc = 0.0_f64; // customer-interruptions

        for evt in &self.events {
            let mut n_affected = 0u64;
            let mut p_affected = 0.0_f64;

            for &cid in &evt.affected_customers {
                if let Some(cd) = cmap.get(&cid) {
                    n_affected += cd.n_customers;
                    p_affected += cd.load_kw * cd.n_customers as f64;
                }
            }

            let n_f = n_affected as f64;

            if evt.sustained {
                saidi_num += n_f * evt.duration_h;
                saifi_num += n_f;
                eens_kwh += p_affected * evt.duration_h;
                enns_acc += n_f;
            } else {
                maifi_num += n_f;
            }
        }

        let saidi = saidi_num / n_t_f;
        let saifi = saifi_num / n_t_f;
        let caidi = if saifi > 0.0 { saidi / saifi } else { 0.0 };
        let hours_per_period = 8760.0 * self.period_years;
        let asai = (1.0 - saidi / hours_per_period).clamp(0.0, 1.0);
        let asui = 1.0 - asai;
        let maifi = maifi_num / n_t_f;
        // MAIFI-E: same as MAIFI in this implementation (momentary events per customer)
        let maifi_e = maifi;
        let eens_mwh = eens_kwh / 1000.0;
        let enns = enns_acc / self.period_years;

        // CEMI-N and CELID
        let (int_counts, cum_dur) = self.per_customer_stats(&cmap);

        let mut cemi_affected = 0u64;
        let mut celid_affected = 0u64;
        for cd in &self.customers {
            let ic = int_counts.get(&cd.id).copied().unwrap_or(0);
            let cd_dur = cum_dur.get(&cd.id).copied().unwrap_or(0.0);
            if ic >= self.cemi_n_threshold {
                cemi_affected += cd.n_customers;
            }
            if cd_dur >= self.celid_l_threshold_h {
                celid_affected += cd.n_customers;
            }
        }
        let cemi_n = cemi_affected as f64 / n_t_f;
        let celid = celid_affected as f64 / n_t_f;

        // CDI — energy-weighted customer damage index
        let total_load_kw: f64 = self
            .customers
            .iter()
            .map(|c| c.load_kw * c.n_customers as f64)
            .sum();
        let denom_cdi = total_load_kw * saidi;
        let cdi = if denom_cdi > 0.0 {
            eens_kwh / denom_cdi
        } else {
            0.0
        };

        ReliabilityIndices {
            saidi,
            saifi,
            caidi,
            asai,
            asui,
            maifi,
            maifi_e,
            cemi_n,
            celid,
            cdi,
            eens_mwh,
            enns,
        }
    }

    /// Compute IEEE 1366 indices broken down by feeder.
    ///
    /// Each [`FeederReliability`] contains indices computed using only the
    /// customers and events belonging to that feeder.
    pub fn compute_feeder_indices(&self) -> Vec<FeederReliability> {
        let mut feeder_ids: Vec<usize> = self
            .customers
            .iter()
            .map(|c| c.feeder_id)
            .chain(self.events.iter().map(|e| e.feeder_id))
            .collect();
        feeder_ids.sort_unstable();
        feeder_ids.dedup();

        feeder_ids
            .into_iter()
            .map(|fid| {
                let customers: Vec<CustomerData> = self
                    .customers
                    .iter()
                    .filter(|c| c.feeder_id == fid)
                    .cloned()
                    .collect();

                let events: Vec<InterruptionEvent> = self
                    .events
                    .iter()
                    .filter(|e| e.feeder_id == fid)
                    .cloned()
                    .collect();

                let n_customers: u64 = customers.iter().map(|c| c.n_customers).sum();
                let total_load_kw: f64 = customers
                    .iter()
                    .map(|c| c.load_kw * c.n_customers as f64)
                    .sum();

                let sub = ReliabilityCalculator {
                    customers,
                    events,
                    period_years: self.period_years,
                    voll_usd_per_mwh: self.voll_usd_per_mwh,
                    cemi_n_threshold: self.cemi_n_threshold,
                    celid_l_threshold_h: self.celid_l_threshold_h,
                };
                let indices = sub.compute_indices();

                FeederReliability {
                    feeder_id: fid,
                    indices,
                    n_customers,
                    total_load_kw,
                }
            })
            .collect()
    }

    /// Compute bulk system reliability indices via sequential Monte Carlo.
    ///
    /// Uses an LCG PRNG (no external `rand` crate) to determine each unit's
    /// state for `n_trials` independent hour-samples.
    ///
    /// # Arguments
    ///
    /// * `units`          — Generating units in the fleet.
    /// * `peak_load_mw`   — System peak load (MW) used as the load target.
    /// * `n_trials`       — Number of Monte Carlo samples (≥ 1000 recommended).
    pub fn compute_bulk_reliability(
        &self,
        units: &[GenerationUnit],
        peak_load_mw: f64,
        n_trials: usize,
    ) -> BulkSystemReliability {
        let mut state: u64 = 0xDEAD_BEEF_CAFE_1234;

        let mut lol_count = 0u64;
        let mut total_unserved_mw = 0.0_f64;
        let mut prev_lol = false;
        let mut lolf_count = 0u64;

        let trials_f = n_trials.max(1) as f64;

        for _ in 0..n_trials.max(1) {
            let mut avail_mw = 0.0_f64;
            for unit in units {
                let u = lcg_next(&mut state);
                if u >= unit.forced_outage_rate {
                    avail_mw += unit.capacity_mw;
                }
            }

            let deficit = peak_load_mw - avail_mw;
            if deficit > 0.0 {
                lol_count += 1;
                total_unserved_mw += deficit;
                if !prev_lol {
                    lolf_count += 1;
                }
                prev_lol = true;
            } else {
                prev_lol = false;
            }
        }

        let lolp = lol_count as f64 / trials_f;
        let lole_days = lolp * 8760.0 / 24.0;
        let avg_unserved_mw = if lol_count > 0 {
            total_unserved_mw / lol_count as f64
        } else {
            0.0
        };
        let loee_mwh = avg_unserved_mw * lolp * 8760.0;
        let eue_mwh = loee_mwh;
        let lolf = lolf_count as f64 / trials_f * 8760.0;
        let lold_h = if lolf > 0.0 {
            lole_days * 24.0 / lolf
        } else {
            0.0
        };
        let rsc = avg_unserved_mw;
        let lolc_usd = self.voll_usd_per_mwh * eue_mwh;

        BulkSystemReliability {
            lolp,
            lole_days,
            loee_mwh,
            eue_mwh,
            lolf,
            lold_h,
            rsc,
            lolc_usd,
        }
    }

    /// Identify the `top_n` feeders with the highest contribution to system SAIDI.
    ///
    /// Returns a vector of `(feeder_id, saidi_contribution_h)` sorted in
    /// descending order of SAIDI contribution.
    pub fn identify_bad_actors(&self, top_n: usize) -> Vec<(usize, f64)> {
        let n_t = self.total_customers();
        if n_t == 0 {
            return Vec::new();
        }
        let n_t_f = n_t as f64;
        let cmap = self.customer_map();

        let mut feeder_saidi: HashMap<usize, f64> = HashMap::new();
        for evt in &self.events {
            if !evt.sustained {
                continue;
            }
            let n_affected: f64 = evt
                .affected_customers
                .iter()
                .filter_map(|cid| cmap.get(cid))
                .map(|cd| cd.n_customers as f64)
                .sum();
            *feeder_saidi.entry(evt.feeder_id).or_insert(0.0) +=
                n_affected * evt.duration_h / n_t_f;
        }

        let mut ranked: Vec<(usize, f64)> = feeder_saidi.into_iter().collect();
        ranked.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        ranked.truncate(top_n);
        ranked
    }

    /// Estimate the system-level SAIDI improvement resulting from reducing a
    /// single feeder's SAIDI contribution by `target_saidi_reduction` hours.
    ///
    /// The improvement is the minimum of the requested reduction and the
    /// feeder's current SAIDI contribution.
    ///
    /// Returns 0.0 if the feeder contributes nothing to system SAIDI.
    pub fn compute_improvement_benefit(
        &self,
        target_feeder: usize,
        target_saidi_reduction: f64,
    ) -> f64 {
        let n_t = self.total_customers();
        if n_t == 0 {
            return 0.0;
        }
        let n_t_f = n_t as f64;
        let cmap = self.customer_map();

        let feeder_contrib: f64 = self
            .events
            .iter()
            .filter(|e| e.sustained && e.feeder_id == target_feeder)
            .map(|evt| {
                let n_aff: f64 = evt
                    .affected_customers
                    .iter()
                    .filter_map(|cid| cmap.get(cid))
                    .map(|cd| cd.n_customers as f64)
                    .sum();
                n_aff * evt.duration_h / n_t_f
            })
            .sum();

        target_saidi_reduction.min(feeder_contrib).max(0.0)
    }

    /// Project reliability indices into future years using a linear trend.
    ///
    /// `trend_pct_per_year` is the percentage *degradation* per year applied to
    /// SAIDI and SAIFI (positive = worsening, negative = improving).
    ///
    /// # Arguments
    ///
    /// * `years`              — Number of future years to project.
    /// * `trend_pct_per_year` — Annual percentage change in SAIDI and SAIFI.
    ///
    /// Returns a `Vec` of length `years`, with index 0 representing year 1.
    pub fn forecast_reliability(
        &self,
        years: usize,
        trend_pct_per_year: f64,
    ) -> Vec<ReliabilityIndices> {
        let base = self.compute_indices();
        let factor = 1.0 + trend_pct_per_year / 100.0;
        let hours_per_period = 8760.0 * self.period_years;

        (1..=years)
            .map(|y| {
                let mult = factor.powi(y as i32);
                let saidi = base.saidi * mult;
                let saifi = base.saifi * mult;
                let caidi = if saifi > 0.0 { saidi / saifi } else { 0.0 };
                let asai = (1.0 - saidi / hours_per_period).clamp(0.0, 1.0);
                let asui = 1.0 - asai;
                let maifi = base.maifi * mult;
                let maifi_e = base.maifi_e * mult;
                let eens_mwh = base.eens_mwh * mult;
                let enns = base.enns * mult;
                let cemi_n = (base.cemi_n * mult).clamp(0.0, 1.0);
                let celid = (base.celid * mult).clamp(0.0, 1.0);
                let cdi = base.cdi; // ratio — trend not applied

                ReliabilityIndices {
                    saidi,
                    saifi,
                    caidi,
                    asai,
                    asui,
                    maifi,
                    maifi_e,
                    cemi_n,
                    celid,
                    cdi,
                    eens_mwh,
                    enns,
                }
            })
            .collect()
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// ── Legacy COT Calculator (preserved from original file) ─────────────────────
// ═════════════════════════════════════════════════════════════════════════════

/// Probabilistic reliability calculator for a generating fleet using the
/// capacity-outage-table (COT) convolution method.
///
/// # Example
/// ```rust
/// use oxigrid::network::reliability_indices::{
///     CotReliabilityCalculator, ReliabilityConfig, GeneratingUnit, LoadData,
/// };
/// let unit = GeneratingUnit {
///     unit_id: "G1".into(),
///     capacity_mw: 100.0,
///     forced_outage_rate: 0.05,
///     mean_time_to_repair_h: 50.0,
///     mean_time_to_fail_h: 950.0,
///     derated_states: vec![],
/// };
/// let load = LoadData {
///     hourly_load_mw: vec![80.0; 8760],
///     peak_load_mw: 80.0,
///     load_factor: 1.0,
/// };
/// let calc = CotReliabilityCalculator {
///     units: vec![unit],
///     load,
///     config: ReliabilityConfig::default(),
/// };
/// let indices = calc.calculate_indices();
/// assert!(indices.lolp >= 0.0 && indices.lolp <= 1.0);
/// ```
pub struct CotReliabilityCalculator {
    /// Generating units in the fleet.
    pub units: Vec<GeneratingUnit>,
    /// Annual load profile.
    pub load: LoadData,
    /// Solver configuration.
    pub config: ReliabilityConfig,
}

/// Composite set of probabilistic reliability indices (COT / legacy model).
#[derive(Debug, Clone)]
pub struct CotReliabilityIndices {
    /// Loss of Load Probability (dimensionless).
    pub lolp: f64,
    /// Loss of Load Expectation (h/year).
    pub lole_h_per_year: f64,
    /// Expected Energy Not Supplied (MWh/year).
    pub eens_mwh_per_year: f64,
    /// Energy Index of Reliability = 1 − EENS / total_annual_energy_demand.
    pub eir: f64,
    /// Reserve margin (%) = (installed − peak) / peak × 100.
    pub reserve_margin_pct: f64,
}

impl CotReliabilityCalculator {
    /// Total installed capacity (MW).
    fn total_installed_mw(&self) -> f64 {
        self.units.iter().map(|u| u.capacity_mw).sum()
    }

    /// Build the capacity outage probability table via recursive convolution.
    pub fn calculate_capacity_outage_table(&self) -> CapacityOutageTable {
        let total = self.total_installed_mw();
        let mut states: StateMap = BTreeMap::new();
        states.insert(mw_to_key(total), 1.0_f64);

        for unit in &self.units {
            let for_ = unit.forced_outage_rate.clamp(0.0, 1.0);
            let mut unit_states: Vec<(f64, f64)> = Vec::new();
            let derated_prob_sum: f64 = unit.derated_states.iter().map(|d| d.probability).sum();
            let full_service_prob = (1.0 - for_ - derated_prob_sum).max(0.0);
            unit_states.push((0.0, full_service_prob));
            for ds in &unit.derated_states {
                let removed = (unit.capacity_mw - ds.available_mw).max(0.0);
                unit_states.push((removed, ds.probability));
            }
            unit_states.push((unit.capacity_mw, for_));

            let mut new_states: StateMap = BTreeMap::new();
            for (&cap_key, &prob) in &states {
                let cap_mw = key_to_mw(cap_key);
                for &(removed_mw, u_prob) in &unit_states {
                    if u_prob <= 0.0 {
                        continue;
                    }
                    let new_cap = (cap_mw - removed_mw).max(0.0);
                    let new_key = mw_to_key(new_cap);
                    *new_states.entry(new_key).or_insert(0.0) += prob * u_prob;
                }
            }
            states = new_states;
        }

        let mut pairs: Vec<(f64, f64)> = states.iter().map(|(&k, &p)| (key_to_mw(k), p)).collect();
        pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        let capacity_mw: Vec<f64> = pairs.iter().map(|p| p.0).collect();
        let probability: Vec<f64> = pairs.iter().map(|p| p.1).collect();
        let n = capacity_mw.len();

        let mut asc_pairs = pairs.clone();
        asc_pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let mut cdf_asc = vec![0.0_f64; n];
        let mut running = 0.0_f64;
        for (i, &(_, p)) in asc_pairs.iter().enumerate() {
            running += p;
            cdf_asc[i] = running.min(1.0);
        }
        let mut cdf_map: BTreeMap<u64, f64> = BTreeMap::new();
        for (i, &(c, _)) in asc_pairs.iter().enumerate() {
            cdf_map.insert(mw_to_key(c), cdf_asc[i]);
        }
        let mut cumulative = vec![0.0_f64; n];
        for (i, &c) in capacity_mw.iter().enumerate() {
            cumulative[i] = *cdf_map.get(&mw_to_key(c)).unwrap_or(&1.0);
        }

        CapacityOutageTable {
            capacity_mw,
            probability,
            cumulative,
        }
    }

    /// Loss of Load Probability (LOLP).
    pub fn calculate_lolp(&self) -> f64 {
        let cot = self.calculate_capacity_outage_table();
        let hours = self.load.hourly_load_mw.len() as f64;
        if hours == 0.0 {
            return 0.0;
        }
        let mut lolp = 0.0_f64;
        for (i, &cap) in cot.capacity_mw.iter().enumerate() {
            let prob = cot.probability[i];
            if prob <= 0.0 {
                continue;
            }
            let hours_exceeding = self
                .load
                .hourly_load_mw
                .iter()
                .filter(|&&l| l > cap + self.config.lolp_threshold_mw)
                .count() as f64;
            lolp += prob * hours_exceeding / hours;
        }
        lolp.min(1.0)
    }

    /// Loss of Load Expectation (h/year).
    pub fn calculate_lole(&self) -> f64 {
        let cot = self.calculate_capacity_outage_table();
        let mut lole = 0.0_f64;
        for &load_h in &self.load.hourly_load_mw {
            let prob_loss: f64 = cot
                .capacity_mw
                .iter()
                .zip(cot.probability.iter())
                .filter(|(&cap, _)| cap < load_h - self.config.lolp_threshold_mw)
                .map(|(_, &p)| p)
                .sum();
            lole += prob_loss;
        }
        lole
    }

    /// Expected Energy Not Supplied (MWh/year).
    pub fn calculate_eens(&self) -> f64 {
        let cot = self.calculate_capacity_outage_table();
        let mut eens = 0.0_f64;
        for &load_h in &self.load.hourly_load_mw {
            for (i, &cap) in cot.capacity_mw.iter().enumerate() {
                let prob = cot.probability[i];
                let deficit = (load_h - cap).max(0.0);
                eens += prob * deficit;
            }
        }
        eens
    }

    /// Reserve margin (%) = (installed − peak) / peak × 100.
    pub fn calculate_reserve_margin(&self) -> f64 {
        let installed = self.total_installed_mw();
        let peak = self.load.peak_load_mw;
        if peak <= 0.0 {
            return 0.0;
        }
        (installed - peak) / peak * 100.0
    }

    /// Run a Monte Carlo generation-adequacy simulation.
    pub fn monte_carlo_simulation(&self) -> MonteCarloReliabilityResult {
        let n_years = self.config.monte_carlo_years.max(1);
        let hours_per_year = self.load.hourly_load_mw.len();
        let threshold = self.config.lolp_threshold_mw;

        let mut rng_state = self.config.seed;

        let mut lolh_per_year_vec: Vec<f64> = Vec::with_capacity(n_years);
        let mut eens_per_year_vec: Vec<f64> = Vec::with_capacity(n_years);
        let mut peak_deficit = 0.0_f64;

        for yr in 0..n_years {
            let load_offset = yr % hours_per_year.max(1);
            let mut year_lolh = 0.0_f64;
            let mut year_eens = 0.0_f64;

            for h in 0..hours_per_year {
                let load_h = self.load.hourly_load_mw[(h + load_offset) % hours_per_year];
                let mut avail = 0.0_f64;
                for unit in &self.units {
                    let u = lcg_next(&mut rng_state);
                    if u >= unit.forced_outage_rate {
                        if unit.derated_states.is_empty() {
                            avail += unit.capacity_mw;
                        } else {
                            let u2 = lcg_next(&mut rng_state);
                            let derated_prob_sum: f64 =
                                unit.derated_states.iter().map(|d| d.probability).sum();
                            let full_prob =
                                (1.0 - unit.forced_outage_rate - derated_prob_sum).max(0.0);
                            let avail_prob = 1.0 - unit.forced_outage_rate;
                            let mut cum = 0.0_f64;
                            let mut contrib = unit.capacity_mw;
                            let scaled_full = full_prob / avail_prob.max(1e-15);
                            cum += scaled_full;
                            if u2 < cum {
                                contrib = unit.capacity_mw;
                            } else {
                                let mut found = false;
                                for ds in &unit.derated_states {
                                    cum += ds.probability / avail_prob.max(1e-15);
                                    if u2 < cum {
                                        contrib = ds.available_mw;
                                        found = true;
                                        break;
                                    }
                                }
                                if !found {
                                    contrib = unit.capacity_mw;
                                }
                            }
                            avail += contrib;
                        }
                    }
                }

                let deficit = (load_h - avail - threshold).max(0.0);
                if deficit > 0.0 {
                    year_lolh += 1.0;
                    year_eens += deficit;
                    if deficit > peak_deficit {
                        peak_deficit = deficit;
                    }
                }
            }

            lolh_per_year_vec.push(year_lolh);
            eens_per_year_vec.push(year_eens);
        }

        let n = n_years as f64;
        let mean_lolh = lolh_per_year_vec.iter().sum::<f64>() / n;
        let mean_eens = eens_per_year_vec.iter().sum::<f64>() / n;

        let var_lolh = lolh_per_year_vec
            .iter()
            .map(|&v| (v - mean_lolh).powi(2))
            .sum::<f64>()
            / n.max(1.0);
        let var_eens = eens_per_year_vec
            .iter()
            .map(|&v| (v - mean_eens).powi(2))
            .sum::<f64>()
            / n.max(1.0);

        let lolh_ci = 1.96 * var_lolh.sqrt() / n.sqrt();
        let eens_ci = 1.96 * var_eens.sqrt() / n.sqrt();

        let total_hours = (hours_per_year * n_years) as f64;
        let lolp = lolh_per_year_vec.iter().sum::<f64>() / total_hours.max(1.0);

        MonteCarloReliabilityResult {
            lolh_per_year: mean_lolh,
            eens_mwh_per_year: mean_eens,
            lolp,
            lolh_ci_95: lolh_ci,
            eens_ci_95: eens_ci,
            peak_deficit_mw: peak_deficit,
            years_simulated: n_years,
        }
    }

    /// Effective Load Carrying Capability (ELCC) of a new unit (MW).
    pub fn capacity_credit(&self, new_unit: &GeneratingUnit) -> f64 {
        let base_lole = self.calculate_lole();

        let mut calc_with = CotReliabilityCalculator {
            units: self.units.clone(),
            load: self.load.clone(),
            config: self.config.clone(),
        };
        calc_with.units.push(new_unit.clone());
        let new_lole = calc_with.calculate_lole();

        if new_lole >= base_lole {
            return 0.0;
        }

        let target_lole = new_lole;
        let mut lo = 0.0_f64;
        let mut hi = new_unit.capacity_mw;

        for _ in 0..60 {
            let mid = (lo + hi) / 2.0;
            let firm_unit = GeneratingUnit {
                unit_id: "__elcc_firm__".into(),
                capacity_mw: mid,
                forced_outage_rate: 0.0,
                mean_time_to_repair_h: 0.0,
                mean_time_to_fail_h: f64::INFINITY,
                derated_states: vec![],
            };
            let mut calc_firm = CotReliabilityCalculator {
                units: self.units.clone(),
                load: self.load.clone(),
                config: self.config.clone(),
            };
            calc_firm.units.push(firm_unit);
            let lole_firm = calc_firm.calculate_lole();
            if lole_firm > target_lole {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        (lo + hi) / 2.0
    }

    /// Sweep the forced outage rate of unit at `unit_idx` over `for_values`
    /// and return `(FOR, LOLE)` pairs (h/year).
    pub fn sensitivity_analysis(&self, unit_idx: usize, for_values: &[f64]) -> Vec<(f64, f64)> {
        for_values
            .iter()
            .map(|&for_val| {
                let mut calc = CotReliabilityCalculator {
                    units: self.units.clone(),
                    load: self.load.clone(),
                    config: self.config.clone(),
                };
                if unit_idx < calc.units.len() {
                    calc.units[unit_idx].forced_outage_rate = for_val.clamp(0.0, 1.0);
                }
                let lole = calc.calculate_lole();
                (for_val, lole)
            })
            .collect()
    }

    /// Compute all composite reliability indices in one call.
    pub fn calculate_indices(&self) -> CotReliabilityIndices {
        let lolp = self.calculate_lolp();
        let lole = self.calculate_lole();
        let eens = self.calculate_eens();
        let total_energy: f64 = self.load.hourly_load_mw.iter().sum();
        let eir = if total_energy > 0.0 {
            (1.0 - eens / total_energy).clamp(0.0, 1.0)
        } else {
            1.0
        };
        let reserve_margin_pct = self.calculate_reserve_margin();

        CotReliabilityIndices {
            lolp,
            lole_h_per_year: lole,
            eens_mwh_per_year: eens,
            eir,
            reserve_margin_pct,
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// ── Tests ─────────────────────────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    // ── helpers ──────────────────────────────────────────────────────────────

    fn make_customer(id: usize, n: u64, load_kw: f64, feeder: usize) -> CustomerData {
        CustomerData {
            id,
            n_customers: n,
            load_kw,
            feeder_id: feeder,
            substation_id: 0,
        }
    }

    fn make_event(
        id: usize,
        dur: f64,
        customers: Vec<usize>,
        sustained: bool,
        feeder: usize,
    ) -> InterruptionEvent {
        InterruptionEvent {
            id,
            start_time: 0.0,
            duration_h: dur,
            affected_customers: customers,
            cause: InterruptionCause::OverheadLine,
            sustained,
            feeder_id: feeder,
        }
    }

    fn constant_load(mw: f64) -> LoadData {
        LoadData {
            hourly_load_mw: vec![mw; 8760],
            peak_load_mw: mw,
            load_factor: 1.0,
        }
    }

    fn single_unit_cot(capacity_mw: f64, for_: f64, load_mw: f64) -> CotReliabilityCalculator {
        CotReliabilityCalculator {
            units: vec![GeneratingUnit {
                unit_id: "G1".into(),
                capacity_mw,
                forced_outage_rate: for_,
                mean_time_to_repair_h: 50.0,
                mean_time_to_fail_h: 950.0,
                derated_states: vec![],
            }],
            load: constant_load(load_mw),
            config: ReliabilityConfig::default(),
        }
    }

    // ── IEEE 1366 tests (test 1–24) ───────────────────────────────────────────

    #[test]
    fn test_saidi_formula() {
        // 50 customers out for 2 h out of 100 total → SAIDI = 1.0
        let customers = vec![
            make_customer(0, 50, 100.0, 0),
            make_customer(1, 50, 100.0, 0),
        ];
        let events = vec![make_event(0, 2.0, vec![0], true, 0)];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let idx = calc.compute_indices();
        assert!((idx.saidi - 1.0).abs() < 1e-9, "SAIDI = {}", idx.saidi);
    }

    #[test]
    fn test_saifi_formula() {
        // 2 sustained events affecting 50 customers each out of 100 → SAIFI = 1.0
        let customers = vec![
            make_customer(0, 50, 100.0, 0),
            make_customer(1, 50, 100.0, 0),
        ];
        let events = vec![
            make_event(0, 1.0, vec![0], true, 0),
            make_event(1, 1.0, vec![1], true, 0),
        ];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let idx = calc.compute_indices();
        assert!((idx.saifi - 1.0).abs() < 1e-9, "SAIFI = {}", idx.saifi);
    }

    #[test]
    fn test_caidi_equals_saidi_over_saifi() {
        let customers = vec![make_customer(0, 100, 200.0, 0)];
        let events = vec![
            make_event(0, 3.0, vec![0], true, 0),
            make_event(1, 1.0, vec![0], true, 0),
        ];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let idx = calc.compute_indices();
        let expected = idx.saidi / idx.saifi;
        assert!((idx.caidi - expected).abs() < 1e-9);
    }

    #[test]
    fn test_asai_near_unity() {
        let customers = vec![make_customer(0, 1000, 500.0, 0)];
        let events = vec![make_event(0, 1.0, vec![0], true, 0)];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let idx = calc.compute_indices();
        assert!(idx.asai > 0.999, "ASAI = {}", idx.asai);
    }

    #[test]
    fn test_eens_computation() {
        // 100 kW load interrupted for 2 h → EENS = 0.2 MWh
        let customers = vec![make_customer(0, 1, 100.0, 0)];
        let events = vec![make_event(0, 2.0, vec![0], true, 0)];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let idx = calc.compute_indices();
        assert!((idx.eens_mwh - 0.2).abs() < 1e-9, "EENS = {}", idx.eens_mwh);
    }

    #[test]
    fn test_zero_events() {
        let customers = vec![make_customer(0, 500, 200.0, 0)];
        let calc = ReliabilityCalculator::new(customers, vec![], 1.0);
        let idx = calc.compute_indices();
        assert_eq!(idx.saidi, 0.0);
        assert_eq!(idx.saifi, 0.0);
        assert_eq!(idx.eens_mwh, 0.0);
        assert!((idx.asai - 1.0).abs() < 1e-12, "ASAI = {}", idx.asai);
    }

    #[test]
    fn test_momentary_excluded_from_saifi() {
        let customers = vec![make_customer(0, 100, 100.0, 0)];
        let events = vec![
            make_event(0, 1.0, vec![0], true, 0),
            make_event(1, 0.05, vec![0], false, 0), // momentary
        ];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let idx = calc.compute_indices();
        assert!((idx.saifi - 1.0).abs() < 1e-9);
        assert!((idx.maifi - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_cemi_n_threshold() {
        // Customer 0: 3 interruptions (≥N=3 → counted); Customer 1: 2 (< threshold)
        let customers = vec![
            make_customer(0, 100, 100.0, 0),
            make_customer(1, 100, 100.0, 0),
        ];
        let events = vec![
            make_event(0, 0.5, vec![0], true, 0),
            make_event(1, 0.5, vec![0], true, 0),
            make_event(2, 0.5, vec![0], true, 0),
            make_event(3, 0.5, vec![1], true, 0),
            make_event(4, 0.5, vec![1], true, 0),
        ];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let idx = calc.compute_indices();
        // CEMI-N = 100/200 = 0.5
        assert!((idx.cemi_n - 0.5).abs() < 1e-9, "CEMI-N = {}", idx.cemi_n);
    }

    #[test]
    fn test_celid_threshold() {
        // Customer 0: 3h total (> 2h threshold); Customer 1: 1h total (< threshold)
        let customers = vec![
            make_customer(0, 50, 100.0, 0),
            make_customer(1, 50, 100.0, 0),
        ];
        let events = vec![
            make_event(0, 3.0, vec![0], true, 0),
            make_event(1, 1.0, vec![1], true, 0),
        ];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let idx = calc.compute_indices();
        assert!((idx.celid - 0.5).abs() < 1e-9, "CELID = {}", idx.celid);
    }

    #[test]
    fn test_feeder_breakdown() {
        let customers = vec![
            make_customer(0, 100, 200.0, 0),
            make_customer(1, 100, 200.0, 1),
        ];
        let events = vec![
            make_event(0, 2.0, vec![0], true, 0),
            make_event(1, 4.0, vec![1], true, 1),
        ];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let sys = calc.compute_indices();
        let feeders = calc.compute_feeder_indices();

        // System SAIDI = (100*2 + 100*4) / 200 = 3.0
        assert!(
            (sys.saidi - 3.0).abs() < 1e-9,
            "System SAIDI = {}",
            sys.saidi
        );

        let f0 = feeders.iter().find(|f| f.feeder_id == 0).expect("feeder 0");
        let f1 = feeders.iter().find(|f| f.feeder_id == 1).expect("feeder 1");
        assert!((f0.indices.saidi - 2.0).abs() < 1e-9);
        assert!((f1.indices.saidi - 4.0).abs() < 1e-9);

        // Weighted sum == system SAIDI
        let weighted = (f0.indices.saidi * f0.n_customers as f64
            + f1.indices.saidi * f1.n_customers as f64)
            / (f0.n_customers + f1.n_customers) as f64;
        assert!((weighted - sys.saidi).abs() < 1e-9);
    }

    #[test]
    fn test_bad_actors_sorted() {
        let customers = vec![
            make_customer(0, 100, 100.0, 0),
            make_customer(1, 100, 100.0, 1),
            make_customer(2, 100, 100.0, 2),
        ];
        let events = vec![
            make_event(0, 1.0, vec![0], true, 0),
            make_event(1, 3.0, vec![1], true, 1), // worst
            make_event(2, 2.0, vec![2], true, 2),
        ];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let bad = calc.identify_bad_actors(3);
        assert_eq!(bad[0].0, 1, "Worst feeder should be 1");
        assert!(bad[0].1 > bad[1].1);
        assert!(bad[1].1 > bad[2].1);
    }

    #[test]
    fn test_bulk_lolp_reasonable() {
        let units: Vec<GenerationUnit> = (0..4)
            .map(|i| GenerationUnit {
                id: i,
                capacity_mw: 100.0,
                forced_outage_rate: 0.05,
                mean_time_to_repair_h: 24.0,
                mean_time_between_failures_h: 500.0,
            })
            .collect();
        let calc = ReliabilityCalculator::new(vec![], vec![], 1.0);
        let bulk = calc.compute_bulk_reliability(&units, 300.0, 5000);
        assert!(bulk.lolp >= 0.0 && bulk.lolp <= 0.1, "LOLP = {}", bulk.lolp);
    }

    #[test]
    fn test_bulk_lole_one_day_criterion() {
        let units: Vec<GenerationUnit> = (0..5)
            .map(|i| GenerationUnit {
                id: i,
                capacity_mw: 200.0,
                forced_outage_rate: 0.02,
                mean_time_to_repair_h: 24.0,
                mean_time_between_failures_h: 800.0,
            })
            .collect();
        let calc = ReliabilityCalculator::new(vec![], vec![], 1.0);
        let bulk = calc.compute_bulk_reliability(&units, 600.0, 8000);
        assert!(bulk.lole_days < 1.0, "LOLE = {} days/yr", bulk.lole_days);
    }

    #[test]
    fn test_bulk_eue_positive() {
        let units = vec![GenerationUnit {
            id: 0,
            capacity_mw: 50.0,
            forced_outage_rate: 0.3,
            mean_time_to_repair_h: 48.0,
            mean_time_between_failures_h: 200.0,
        }];
        let calc = ReliabilityCalculator::new(vec![], vec![], 1.0);
        let bulk = calc.compute_bulk_reliability(&units, 100.0, 2000);
        assert!(bulk.eue_mwh >= 0.0);
    }

    #[test]
    fn test_bulk_monte_carlo_convergence() {
        let units: Vec<GenerationUnit> = (0..3)
            .map(|i| GenerationUnit {
                id: i,
                capacity_mw: 100.0,
                forced_outage_rate: 0.08,
                mean_time_to_repair_h: 24.0,
                mean_time_between_failures_h: 300.0,
            })
            .collect();
        let calc = ReliabilityCalculator::new(vec![], vec![], 1.0);
        let bulk_a = calc.compute_bulk_reliability(&units, 200.0, 2000);
        let bulk_b = calc.compute_bulk_reliability(&units, 200.0, 5000);
        assert!(bulk_a.lolp >= 0.0 && bulk_a.lolp <= 1.0);
        assert!(bulk_b.lolp >= 0.0 && bulk_b.lolp <= 1.0);
        assert!(
            (bulk_a.lolp - bulk_b.lolp).abs() < 0.15,
            "LOLP divergence: {} vs {}",
            bulk_a.lolp,
            bulk_b.lolp
        );
    }

    #[test]
    fn test_improvement_benefit() {
        let customers = vec![
            make_customer(0, 200, 400.0, 0),
            make_customer(1, 200, 400.0, 1),
        ];
        let events = vec![
            make_event(0, 4.0, vec![0], true, 0),
            make_event(1, 1.0, vec![1], true, 1),
        ];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        // Feeder 0 contributes 200*4/400 = 2.0 h → reducing by 2.0 gives benefit 2.0
        let benefit = calc.compute_improvement_benefit(0, 2.0);
        assert!((benefit - 2.0).abs() < 1e-9, "benefit = {}", benefit);
    }

    #[test]
    fn test_forecast_reliability() {
        let customers = vec![make_customer(0, 100, 200.0, 0)];
        let events = vec![make_event(0, 2.0, vec![0], true, 0)];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let forecast = calc.forecast_reliability(3, 5.0);
        assert_eq!(forecast.len(), 3);
        assert!(forecast[1].saidi > forecast[0].saidi);
        assert!(forecast[2].saidi > forecast[1].saidi);
        let base = calc.compute_indices().saidi;
        assert!((forecast[0].saidi - base * 1.05).abs() < 1e-9);
    }

    #[test]
    fn test_multiple_feeders() {
        let customers = vec![
            make_customer(0, 100, 100.0, 0),
            make_customer(1, 100, 100.0, 1),
            make_customer(2, 100, 100.0, 2),
        ];
        let events = vec![
            make_event(0, 1.0, vec![0], true, 0),
            make_event(1, 2.0, vec![1], true, 1),
            make_event(2, 3.0, vec![2], true, 2),
        ];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let feeders = calc.compute_feeder_indices();
        assert_eq!(feeders.len(), 3);
        for f in &feeders {
            let expected_saidi = match f.feeder_id {
                0 => 1.0,
                1 => 2.0,
                2 => 3.0,
                _ => panic!("unexpected feeder"),
            };
            assert!(
                (f.indices.saidi - expected_saidi).abs() < 1e-9,
                "feeder {} SAIDI = {}",
                f.feeder_id,
                f.indices.saidi
            );
        }
    }

    #[test]
    fn test_voll_cost() {
        let units = vec![GenerationUnit {
            id: 0,
            capacity_mw: 80.0,
            forced_outage_rate: 0.5,
            mean_time_to_repair_h: 10.0,
            mean_time_between_failures_h: 10.0,
        }];
        let mut calc = ReliabilityCalculator::new(vec![], vec![], 1.0);
        calc.voll_usd_per_mwh = 10_000.0;
        let bulk = calc.compute_bulk_reliability(&units, 100.0, 3000);
        let expected_cost = calc.voll_usd_per_mwh * bulk.eue_mwh;
        assert!((bulk.lolc_usd - expected_cost).abs() < 1e-6);
    }

    #[test]
    fn test_cause_breakdown() {
        let customers = vec![make_customer(0, 100, 200.0, 0)];
        let events = vec![
            InterruptionEvent {
                id: 0,
                start_time: 0.0,
                duration_h: 3.0,
                affected_customers: vec![0],
                cause: InterruptionCause::OverheadLine,
                sustained: true,
                feeder_id: 0,
            },
            InterruptionEvent {
                id: 1,
                start_time: 5.0,
                duration_h: 1.0,
                affected_customers: vec![0],
                cause: InterruptionCause::Equipment,
                sustained: true,
                feeder_id: 0,
            },
        ];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);

        let overhead_events: Vec<InterruptionEvent> = calc
            .events
            .iter()
            .filter(|e| e.cause == InterruptionCause::OverheadLine)
            .cloned()
            .collect();
        let sub = ReliabilityCalculator::new(calc.customers.clone(), overhead_events, 1.0);
        let idx = sub.compute_indices();
        assert!((idx.saidi - 3.0).abs() < 1e-9, "SAIDI = {}", idx.saidi);
    }

    #[test]
    fn test_asui_complement() {
        let customers = vec![make_customer(0, 100, 200.0, 0)];
        let events = vec![make_event(0, 1.0, vec![0], true, 0)];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let idx = calc.compute_indices();
        assert!((idx.asai + idx.asui - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_improvement_benefit_capped_at_contribution() {
        let customers = vec![make_customer(0, 100, 100.0, 0)];
        let events = vec![make_event(0, 1.0, vec![0], true, 0)];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        // Feeder 0 contributes only 1.0 h; requesting 999.0 should return 1.0
        let benefit = calc.compute_improvement_benefit(0, 999.0);
        assert!((benefit - 1.0).abs() < 1e-9, "benefit = {}", benefit);
    }

    #[test]
    fn test_no_customers_returns_default() {
        let calc = ReliabilityCalculator::new(vec![], vec![], 1.0);
        let idx = calc.compute_indices();
        assert_eq!(idx.saidi, 0.0);
        assert_eq!(idx.saifi, 0.0);
        assert!((idx.asai - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_eens_multiple_customers_in_event() {
        // 100 kW + 200 kW for 3 h → EENS = 0.9 MWh
        let customers = vec![make_customer(0, 1, 100.0, 0), make_customer(1, 1, 200.0, 0)];
        let events = vec![make_event(0, 3.0, vec![0, 1], true, 0)];
        let calc = ReliabilityCalculator::new(customers, events, 1.0);
        let idx = calc.compute_indices();
        assert!((idx.eens_mwh - 0.9).abs() < 1e-9, "EENS = {}", idx.eens_mwh);
    }

    // ── Legacy COT tests (preserved) ─────────────────────────────────────────

    #[test]
    fn test_perfect_reliability() {
        let calc = single_unit_cot(100.0, 0.0, 80.0);
        let indices = calc.calculate_indices();
        assert!(indices.lolp < 1e-10, "LOLP = {}", indices.lolp);
        assert!(
            indices.lole_h_per_year < 1e-10,
            "LOLE = {}",
            indices.lole_h_per_year
        );
    }

    #[test]
    fn test_complete_outage() {
        let calc = single_unit_cot(100.0, 1.0, 80.0);
        let lolp = calc.calculate_lolp();
        let lole = calc.calculate_lole();
        assert!((lolp - 1.0).abs() < 1e-9, "LOLP = {}", lolp);
        assert!((lole - 8760.0).abs() < 1e-6, "LOLE = {}", lole);
    }

    #[test]
    fn test_reserve_margin() {
        let calc = single_unit_cot(120.0, 0.05, 100.0);
        let rm = calc.calculate_reserve_margin();
        assert!((rm - 20.0).abs() < 1e-9, "Reserve margin = {}", rm);
    }

    #[test]
    fn test_lole_decreases_with_lower_for() {
        let calc_high = single_unit_cot(100.0, 0.2, 80.0);
        let calc_low = single_unit_cot(100.0, 0.05, 80.0);
        assert!(calc_low.calculate_lole() < calc_high.calculate_lole());
    }

    #[test]
    fn test_monte_carlo_convergence() {
        let units = vec![
            GeneratingUnit {
                unit_id: "G1".into(),
                capacity_mw: 60.0,
                forced_outage_rate: 0.05,
                mean_time_to_repair_h: 50.0,
                mean_time_to_fail_h: 950.0,
                derated_states: vec![],
            },
            GeneratingUnit {
                unit_id: "G2".into(),
                capacity_mw: 60.0,
                forced_outage_rate: 0.08,
                mean_time_to_repair_h: 80.0,
                mean_time_to_fail_h: 920.0,
                derated_states: vec![],
            },
        ];
        let calc = CotReliabilityCalculator {
            units,
            load: constant_load(100.0),
            config: ReliabilityConfig {
                monte_carlo_years: 200,
                seed: 42,
                ..Default::default()
            },
        };
        let analytical_lole = calc.calculate_lole();
        let mc = calc.monte_carlo_simulation();
        if analytical_lole > 0.01 {
            let ratio = (mc.lolh_per_year - analytical_lole).abs() / analytical_lole;
            assert!(ratio < 0.50, "MC/analytical ratio {} exceeds 50%", ratio);
        }
    }

    #[test]
    fn test_eens_positive_when_deficit() {
        let calc = single_unit_cot(50.0, 0.5, 80.0);
        assert!(calc.calculate_eens() > 0.0);
    }

    #[test]
    fn test_capacity_outage_table_sums_to_one() {
        let units = vec![
            GeneratingUnit {
                unit_id: "A".into(),
                capacity_mw: 40.0,
                forced_outage_rate: 0.05,
                mean_time_to_repair_h: 50.0,
                mean_time_to_fail_h: 950.0,
                derated_states: vec![],
            },
            GeneratingUnit {
                unit_id: "B".into(),
                capacity_mw: 60.0,
                forced_outage_rate: 0.10,
                mean_time_to_repair_h: 100.0,
                mean_time_to_fail_h: 900.0,
                derated_states: vec![DeratedState {
                    available_mw: 30.0,
                    probability: 0.03,
                }],
            },
        ];
        let calc = CotReliabilityCalculator {
            units,
            load: constant_load(80.0),
            config: ReliabilityConfig::default(),
        };
        let cot = calc.calculate_capacity_outage_table();
        let prob_sum: f64 = cot.probability.iter().sum();
        assert!(
            (prob_sum - 1.0).abs() < 1e-9,
            "Probability sum = {}",
            prob_sum
        );
    }

    #[test]
    fn test_eir_in_range() {
        let calc = single_unit_cot(80.0, 0.10, 80.0);
        let indices = calc.calculate_indices();
        assert!(
            indices.eir >= 0.0 && indices.eir <= 1.0,
            "EIR = {}",
            indices.eir
        );
    }

    #[test]
    fn test_sensitivity_analysis_monotone() {
        let calc = single_unit_cot(100.0, 0.05, 80.0);
        let for_values: Vec<f64> = (0..=10).map(|i| i as f64 * 0.1).collect();
        let results = calc.sensitivity_analysis(0, &for_values);
        for window in results.windows(2) {
            let (_, lole_a) = window[0];
            let (_, lole_b) = window[1];
            assert!(
                lole_b >= lole_a - 1e-9,
                "LOLE not monotone: {} → {}",
                lole_a,
                lole_b
            );
        }
    }

    #[test]
    fn test_derated_states_reliability() {
        let unit = GeneratingUnit {
            unit_id: "D1".into(),
            capacity_mw: 100.0,
            forced_outage_rate: 0.05,
            mean_time_to_repair_h: 50.0,
            mean_time_to_fail_h: 950.0,
            derated_states: vec![DeratedState {
                available_mw: 50.0,
                probability: 0.10,
            }],
        };
        let calc = CotReliabilityCalculator {
            units: vec![unit],
            load: constant_load(70.0),
            config: ReliabilityConfig::default(),
        };
        let cot = calc.calculate_capacity_outage_table();
        let prob_sum: f64 = cot.probability.iter().sum();
        assert!((prob_sum - 1.0).abs() < 1e-9, "COT prob sum = {}", prob_sum);
        assert!(
            cot.capacity_mw.len() >= 3,
            "Expected ≥3 states, got {}",
            cot.capacity_mw.len()
        );
    }
}
