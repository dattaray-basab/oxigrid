//! Smart-grid communication network simulation framework.
//!
//! Models latency, packet loss, bandwidth constraints, and their impact on
//! power system control stability.  Supports:
//!
//! - Multiple smart-grid communication protocols (IEC 61850, DNP3, Modbus,
//!   MQTT, 5G URLLC, Fibre, 4G)
//! - Realistic latency sampling with LCG + Box–Muller jitter
//! - Congestion modelling (none / simple threshold / detailed queue)
//! - Cyber-attack impact estimation (DoS, Man-in-the-Middle)
//! - Control-loop delay analysis (Nyquist stability criterion)
//! - Monte-Carlo latency statistics (mean, σ, P95, P99, max)
//! - Upgrade recommendations based on measured vs. required performance
//!
//! # Units
//! All latencies are in **milliseconds `\[ms\]`**, bandwidths in **kilobits per
//! second `\[kbps\]`**, and fractions (loss, availability) are dimensionless
//! in `\[0, 1\]`.

use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────────────
// Protocol
// ─────────────────────────────────────────────────────────────────────────────

/// Communication protocol used on a link.
///
/// Each variant implies a characteristic latency band and delivery semantics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommProtocol {
    /// IEC 61850 GOOSE multicast — nominal < 4 ms.
    IEC61850Goose,
    /// IEC 61850 Sampled Values — nominal < 1 ms, continuous stream.
    IEC61850Sampled,
    /// DNP3 over serial — 50–500 ms.
    Dnp3Serial,
    /// DNP3 over TCP — 10–100 ms.
    Dnp3Tcp,
    /// Modbus TCP — 10–100 ms.
    ModbusTcp,
    /// IEC 60870-5-104 — 50–200 ms.
    IEC60870_104,
    /// MQTT 5.0 — 5–50 ms variable.
    Mqtt,
    /// 5G URLLC — < 1 ms ultra-reliable.
    FiveGUrllc,
    /// Dedicated fibre — < 2 ms.
    Fiber,
    /// 4G LTE wireless — 20–100 ms variable.
    Wireless4G,
}

impl CommProtocol {
    /// Returns a human-readable name for the protocol.
    pub fn name(&self) -> &'static str {
        match self {
            Self::IEC61850Goose => "IEC 61850 GOOSE",
            Self::IEC61850Sampled => "IEC 61850 Sampled Values",
            Self::Dnp3Serial => "DNP3 Serial",
            Self::Dnp3Tcp => "DNP3 TCP",
            Self::ModbusTcp => "Modbus TCP",
            Self::IEC60870_104 => "IEC 60870-5-104",
            Self::Mqtt => "MQTT",
            Self::FiveGUrllc => "5G URLLC",
            Self::Fiber => "Fiber",
            Self::Wireless4G => "4G Wireless",
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Message priority & type
// ─────────────────────────────────────────────────────────────────────────────

/// Message delivery priority (highest → lowest).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum MessagePriority {
    /// Must arrive within a few milliseconds; dropping is unacceptable.
    Critical = 3,
    /// Important; short deadline (tens of milliseconds).
    High = 2,
    /// Standard telemetry / control (hundreds of milliseconds tolerable).
    Normal = 1,
    /// Background data; no hard deadline.
    Low = 0,
}

/// Semantic class of a smart-grid message, with implied deadline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommMessageType {
    /// Protection relay trip — must arrive within 4 ms \[ms\].
    ProtectionTrip,
    /// Supervisory control command — 10–100 ms tolerable.
    ControlCommand,
    /// Analogue / digital measurement report — 100 ms tolerable.
    Measurement,
    /// State-estimator input/output — 500 ms tolerable.
    StateEstimation,
    /// Alarm notification.
    Alarm,
    /// Configuration or parameterisation message.
    Configuration,
}

impl CommMessageType {
    /// Hard deadline for delivery \[ms\].  `f64::INFINITY` for no hard deadline.
    pub fn deadline_ms(&self) -> f64 {
        match self {
            Self::ProtectionTrip => 4.0,
            Self::ControlCommand => 100.0,
            Self::Measurement => 100.0,
            Self::StateEstimation => 500.0,
            Self::Alarm => 1_000.0,
            Self::Configuration => f64::INFINITY,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CommLink
// ─────────────────────────────────────────────────────────────────────────────

/// A unidirectional network link with bandwidth and reliability characterisation.
#[derive(Debug, Clone)]
pub struct CommLink {
    /// Human-readable link identifier (e.g. `"substation-A → RTU-3"`).
    pub link_id: String,
    /// Communication protocol.
    pub protocol: CommProtocol,
    /// Nominal link capacity \[kbps\].
    pub bandwidth_kbps: f64,
    /// Nominal (median) one-way latency \[ms\].
    pub nominal_latency_ms: f64,
    /// Maximum permissible latency under worst-case congestion \[ms\].
    pub max_latency_ms: f64,
    /// Steady-state packet-loss probability \[0, 1\].
    pub packet_loss_rate: f64,
    /// One-sigma latency jitter (Gaussian std-dev) \[ms\].
    pub jitter_ms: f64,
    /// Long-run link availability (uptime fraction) \[0, 1\].
    pub availability: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Message (comm_network variant — distinct from protocol::Message)
// ─────────────────────────────────────────────────────────────────────────────

/// A smart-grid protocol message to be simulated over a [`CommLink`].
#[derive(Debug, Clone)]
pub struct CommMessage {
    /// Unique message identifier.
    pub message_id: u64,
    /// Delivery priority.
    pub priority: MessagePriority,
    /// Application payload size \[bytes\] (excluding protocol headers).
    pub payload_bytes: usize,
    /// Wall-clock time at which the message was injected into the network \[ms\].
    pub timestamp_sent_ms: f64,
    /// Source node label.
    pub source: String,
    /// Destination node label.
    pub destination: String,
    /// Semantic type (determines deadline).
    pub message_type: CommMessageType,
}

// ─────────────────────────────────────────────────────────────────────────────
// Congestion model
// ─────────────────────────────────────────────────────────────────────────────

/// Model used to increase latency under high link utilisation.
#[derive(Debug, Clone)]
pub enum CongestionModel {
    /// No congestion effect; latency is purely nominal + jitter.
    None,
    /// Simple threshold model: latency doubles when utilisation exceeds the
    /// threshold fraction \[0, 1\].
    Simple {
        /// Utilisation fraction above which latency doubles.
        utilization_threshold: f64,
    },
    /// Detailed first-come-first-served queue: each queued message waits for
    /// `queue_depth` predecessor transmissions before it starts.
    DetailedQueue {
        /// Maximum number of packets in the queue.
        queue_depth: usize,
    },
}

// ─────────────────────────────────────────────────────────────────────────────
// Config
// ─────────────────────────────────────────────────────────────────────────────

/// Top-level configuration for a [`CommNetworkSim`] session.
#[derive(Debug, Clone)]
pub struct CommNetworkConfig {
    /// How long the simulation runs \[ms\].
    pub simulation_duration_ms: f64,
    /// Congestion model applied to every link in this simulation.
    pub congestion_model: CongestionModel,
    /// Additional fixed latency added per message to account for encryption /
    /// protocol-security processing \[ms\] (default 0.5 ms).
    pub security_overhead_ms: f64,
}

impl Default for CommNetworkConfig {
    fn default() -> Self {
        Self {
            simulation_duration_ms: 10_000.0,
            congestion_model: CongestionModel::None,
            security_overhead_ms: 0.5,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Output types
// ─────────────────────────────────────────────────────────────────────────────

/// Result of simulating delivery of a single message over a link.
#[derive(Debug, Clone)]
pub struct DeliveryResult {
    /// `true` if the message survived packet-loss sampling.
    pub delivered: bool,
    /// Actual end-to-end latency experienced \[ms\] (0.0 if lost).
    pub latency_ms: f64,
    /// `true` if the latency is within the message type's deadline.
    pub deadline_met: bool,
    /// Number of logical hops traversed (always ≥ 1 for a single link).
    pub hops: usize,
}

/// Round-trip analysis of a cascaded sequence of links (control loop).
#[derive(Debug, Clone)]
pub struct ControlLoopAnalysis {
    /// Total one-way accumulated latency across all links \[ms\].
    pub total_latency_ms: f64,
    /// The stability-margin ceiling: controller can tolerate at most this much
    /// total round-trip delay \[ms\].
    pub stability_margin_ms: f64,
    /// `true` if `total_latency_ms` ≤ `stability_margin_ms`.
    pub stability_maintained: bool,
    /// Maximum delay allowed by the Nyquist criterion: 1/(4 × bandwidth) \[ms\].
    pub nyquist_limit_ms: f64,
}

/// Protection-system performance report.
#[derive(Debug, Clone)]
pub struct ProtectionAssessment {
    /// Fraction of messages that arrived within their required deadline.
    pub fraction_on_time: f64,
    /// Maximum latency observed across all messages \[ms\].
    pub worst_case_latency_ms: f64,
    /// Number of messages that missed their deadline.
    pub missed_deadlines: usize,
    /// Estimated reliability: delivered-on-time / total.
    pub reliability: f64,
}

/// Latency distribution statistics from a Monte-Carlo run.
#[derive(Debug, Clone)]
pub struct LatencyStats {
    /// Sample mean \[ms\].
    pub mean_ms: f64,
    /// Sample standard deviation \[ms\].
    pub std_ms: f64,
    /// 95th-percentile latency \[ms\].
    pub p95_ms: f64,
    /// 99th-percentile latency \[ms\].
    pub p99_ms: f64,
    /// Maximum sampled latency \[ms\].
    pub max_ms: f64,
}

/// Impact of a cyber attack on a link.
#[derive(Debug, Clone)]
pub struct AttackImpact {
    /// Number of messages whose latency exceeded their deadline.
    pub messages_delayed: usize,
    /// Number of messages dropped during the attack.
    pub messages_lost: usize,
    /// Maximum latency observed during the attack window \[ms\].
    pub max_latency_ms: f64,
    /// `true` if any time-critical control action was disrupted.
    pub control_disrupted: bool,
}

/// A single upgrade recommendation for a link.
#[derive(Debug, Clone)]
pub struct UpgradeRecommendation {
    /// Identifier of the affected link.
    pub link_id: String,
    /// Human-readable description of the identified issue.
    pub issue: String,
    /// Protocol that would resolve the issue.
    pub recommended_protocol: CommProtocol,
    /// Short description of the expected benefit.
    pub expected_improvement: String,
}

// ─────────────────────────────────────────────────────────────────────────────
// Attack type
// ─────────────────────────────────────────────────────────────────────────────

/// Cyber-attack scenario to inject into a link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttackType {
    /// Denial-of-Service — floods the link to 100 % utilisation; latency ×10.
    DenialOfService,
    /// Man-in-the-Middle — intercepts and re-injects; adds fixed 5 ms overhead.
    ManInTheMiddle,
}

// ─────────────────────────────────────────────────────────────────────────────
// Performance requirements (for upgrade recommendations)
// ─────────────────────────────────────────────────────────────────────────────

/// Minimum acceptable performance for a link.
#[derive(Debug, Clone)]
pub struct PerformanceRequirements {
    /// Maximum tolerable latency \[ms\].
    pub max_latency_ms: f64,
    /// Maximum tolerable packet-loss rate \[0, 1\].
    pub max_loss_rate: f64,
    /// Minimum required availability \[0, 1\].
    pub min_availability: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Main simulator
// ─────────────────────────────────────────────────────────────────────────────

/// Smart-grid communication network simulator.
///
/// Holds a set of [`CommLink`]s and a [`CommNetworkConfig`] that govern all
/// simulation runs.  Randomness is produced by an internal LCG so that results
/// are reproducible given the same seed.
pub struct CommNetworkSim {
    /// Registered network links.
    pub links: Vec<CommLink>,
    /// Simulation parameters.
    pub config: CommNetworkConfig,
    /// LCG state (mutable across all sampling calls).
    lcg_state: u64,
}

impl CommNetworkSim {
    // ── Construction ─────────────────────────────────────────────────────────

    /// Create a new simulator with the given configuration and seed.
    pub fn new(config: CommNetworkConfig, seed: u64) -> Self {
        Self {
            links: Vec::new(),
            config,
            lcg_state: seed.wrapping_add(1),
        }
    }

    /// Create a simulator with default configuration.
    pub fn with_default_config(seed: u64) -> Self {
        Self::new(CommNetworkConfig::default(), seed)
    }

    /// Register a link and return its index.
    pub fn add_link(&mut self, link: CommLink) -> usize {
        let idx = self.links.len();
        self.links.push(link);
        idx
    }

    // ── Internal LCG helpers ─────────────────────────────────────────────────

    fn lcg_next(&mut self) -> u64 {
        self.lcg_state = self
            .lcg_state
            .wrapping_mul(6_364_136_223_846_793_005_u64)
            .wrapping_add(1_442_695_040_888_963_407_u64);
        self.lcg_state
    }

    fn lcg_f64(&mut self) -> f64 {
        (self.lcg_next() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Box–Muller standard normal sample.
    fn lcg_normal(&mut self) -> f64 {
        let u1 = self.lcg_f64().max(f64::EPSILON);
        let u2 = self.lcg_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }

    // ── Core delivery simulation ─────────────────────────────────────────────

    /// Sample the latency for a single message over `link`, applying jitter,
    /// security overhead, and congestion.
    ///
    /// Returns `None` if the message is dropped by the packet-loss model or the
    /// link is down.
    fn sample_latency(
        &mut self,
        link: &CommLink,
        payload_bytes: usize,
        current_utilization: f64,
    ) -> Option<f64> {
        // --- Packet loss ---
        let loss_roll = self.lcg_f64();
        if loss_roll < link.packet_loss_rate {
            return None;
        }

        // --- Base latency with jitter ---
        let jitter = self.lcg_normal() * link.jitter_ms;
        let base = (link.nominal_latency_ms + jitter).max(0.0);

        // --- Security overhead ---
        let lat = base + self.config.security_overhead_ms;

        // --- Congestion multiplier ---
        let lat = match &self.config.congestion_model {
            CongestionModel::None => lat,
            CongestionModel::Simple {
                utilization_threshold,
            } => {
                if current_utilization > *utilization_threshold {
                    lat * 2.0
                } else {
                    lat
                }
            }
            CongestionModel::DetailedQueue { queue_depth } => {
                // Each extra byte in the queue adds serialisation delay.
                let bw_bytes_per_ms = link.bandwidth_kbps * 1_000.0 / 8.0 / 1_000.0;
                let tx_ms = if bw_bytes_per_ms > 0.0 {
                    payload_bytes as f64 / bw_bytes_per_ms
                } else {
                    0.0
                };
                // Queue occupancy as fraction of depth (capped at 1.0).
                let occupancy = (current_utilization * *queue_depth as f64)
                    .min(*queue_depth as f64)
                    / (*queue_depth as f64).max(1.0);
                lat + tx_ms * occupancy
            }
        };

        // Cap at max_latency (still counts as delivered but very late).
        Some(lat.min(link.max_latency_ms))
    }

    // ── Public API ───────────────────────────────────────────────────────────

    /// Simulate delivery of one message over link `link_idx`.
    ///
    /// # Errors
    /// Returns an error string if `link_idx` is out of range.
    pub fn simulate_message_delivery(
        &mut self,
        message: &CommMessage,
        link_idx: usize,
    ) -> Result<DeliveryResult, String> {
        let link = self
            .links
            .get(link_idx)
            .ok_or_else(|| format!("link index {link_idx} out of range"))?
            .clone();

        // Estimate instantaneous utilisation from message payload.
        let util = self.calculate_channel_utilization(
            std::slice::from_ref(message),
            link_idx,
            self.config.simulation_duration_ms,
        )?;

        match self.sample_latency(&link, message.payload_bytes, util) {
            None => Ok(DeliveryResult {
                delivered: false,
                latency_ms: 0.0,
                deadline_met: false,
                hops: 1,
            }),
            Some(lat) => {
                let deadline = message.message_type.deadline_ms();
                Ok(DeliveryResult {
                    delivered: true,
                    latency_ms: lat,
                    deadline_met: lat <= deadline,
                    hops: 1,
                })
            }
        }
    }

    /// Analyse the end-to-end delay of a control loop.
    ///
    /// The loop is modelled as a sequence of one-way links: sensor → controller
    /// → actuator.  The total latency is the sum of sampled latencies over
    /// `link_indices`.  Stability is checked against the Nyquist criterion for
    /// `control_bandwidth_hz`.
    ///
    /// # Errors
    /// Returns an error string if any link index is invalid or if
    /// `message_sequence` is shorter than `link_indices`.
    pub fn analyze_control_loop_delay(
        &mut self,
        link_indices: &[usize],
        message_sequence: &[CommMessage],
        control_bandwidth_hz: f64,
    ) -> Result<ControlLoopAnalysis, String> {
        if message_sequence.len() < link_indices.len() {
            return Err(format!(
                "message_sequence length {} < link_indices length {}",
                message_sequence.len(),
                link_indices.len()
            ));
        }

        let nyquist_limit_ms = if control_bandwidth_hz > 0.0 {
            1_000.0 / (4.0 * control_bandwidth_hz)
        } else {
            f64::INFINITY
        };

        let mut total_latency_ms = 0.0_f64;

        for (hop, &idx) in link_indices.iter().enumerate() {
            let link = self
                .links
                .get(idx)
                .ok_or_else(|| format!("link index {idx} out of range"))?
                .clone();

            let msg = &message_sequence[hop];
            let util = self.calculate_channel_utilization(
                std::slice::from_ref(msg),
                idx,
                self.config.simulation_duration_ms,
            )?;

            let lat = self
                .sample_latency(&link, msg.payload_bytes, util)
                .unwrap_or(link.max_latency_ms);

            total_latency_ms += lat;
        }

        // The stability margin equals the Nyquist limit (round-trip).
        let stability_margin_ms = nyquist_limit_ms;
        let stability_maintained = total_latency_ms <= stability_margin_ms;

        Ok(ControlLoopAnalysis {
            total_latency_ms,
            stability_margin_ms,
            stability_maintained,
            nyquist_limit_ms,
        })
    }

    /// Compute channel utilisation for a batch of messages over `link_idx`
    /// within a time window.
    ///
    /// `utilization = (total_bits) / (bandwidth_kbps × 1000 \[bps\] × window_s)`
    ///
    /// # Errors
    /// Returns an error string if `link_idx` is out of range or `window_ms` ≤ 0.
    pub fn calculate_channel_utilization(
        &self,
        messages: &[CommMessage],
        link_idx: usize,
        window_ms: f64,
    ) -> Result<f64, String> {
        let link = self
            .links
            .get(link_idx)
            .ok_or_else(|| format!("link index {link_idx} out of range"))?;

        if window_ms <= 0.0 {
            return Err("window_ms must be positive".to_string());
        }

        let total_bytes: usize = messages.iter().map(|m| m.payload_bytes).sum();
        let total_bits = total_bytes as f64 * 8.0;
        let capacity_bits = link.bandwidth_kbps * 1_000.0 * (window_ms / 1_000.0);

        if capacity_bits <= 0.0 {
            return Err("link bandwidth_kbps must be positive".to_string());
        }

        Ok((total_bits / capacity_bits).min(1.0))
    }

    /// Evaluate protection-system message performance on link `link_idx`.
    ///
    /// For IEC 61850 GOOSE links the hard deadline is 4 ms; for other protocols
    /// each [`CommMessageType`] carries its own deadline.
    ///
    /// # Errors
    /// Returns an error string if `link_idx` is out of range.
    pub fn assess_protection_performance(
        &mut self,
        protection_messages: &[CommMessage],
        link_idx: usize,
    ) -> Result<ProtectionAssessment, String> {
        let link = self
            .links
            .get(link_idx)
            .ok_or_else(|| format!("link index {link_idx} out of range"))?
            .clone();

        let is_goose = link.protocol == CommProtocol::IEC61850Goose;

        let mut on_time = 0usize;
        let mut missed = 0usize;
        let mut delivered = 0usize;
        let mut worst_latency = 0.0_f64;

        for msg in protection_messages {
            let deadline = if is_goose {
                4.0_f64
            } else {
                msg.message_type.deadline_ms()
            };

            let util = self.calculate_channel_utilization(
                std::slice::from_ref(msg),
                link_idx,
                self.config.simulation_duration_ms,
            )?;

            match self.sample_latency(&link, msg.payload_bytes, util) {
                None => {
                    missed += 1;
                }
                Some(lat) => {
                    delivered += 1;
                    if lat > worst_latency {
                        worst_latency = lat;
                    }
                    if lat <= deadline {
                        on_time += 1;
                    } else {
                        missed += 1;
                    }
                }
            }
        }

        let total = protection_messages.len();
        let fraction_on_time = if total == 0 {
            1.0
        } else {
            on_time as f64 / total as f64
        };
        let reliability = if total == 0 {
            1.0
        } else {
            delivered as f64 / total as f64
        };

        Ok(ProtectionAssessment {
            fraction_on_time,
            worst_case_latency_ms: worst_latency,
            missed_deadlines: missed,
            reliability,
        })
    }

    /// Compute the composite availability of a set of links.
    ///
    /// - **Series path** (`parallel = false`): availability = Π avail_i
    /// - **Parallel paths** (`parallel = true`): availability = 1 − Π (1 − avail_i)
    ///
    /// # Errors
    /// Returns an error string if any link index is invalid.
    pub fn network_availability(
        &self,
        link_indices: &[usize],
        parallel: bool,
    ) -> Result<f64, String> {
        if link_indices.is_empty() {
            return Ok(1.0);
        }

        let mut result = 1.0_f64;

        for &idx in link_indices {
            let link = self
                .links
                .get(idx)
                .ok_or_else(|| format!("link index {idx} out of range"))?;

            if parallel {
                result *= 1.0 - link.availability;
            } else {
                result *= link.availability;
            }
        }

        if parallel {
            Ok(1.0 - result)
        } else {
            Ok(result)
        }
    }

    /// Estimate the impact of a cyber attack on link `link_idx`.
    ///
    /// The simulator runs `messages` through the (modified) link during the
    /// attack window and reports how many were delayed or lost.
    ///
    /// # Attack models
    /// - [`AttackType::DenialOfService`]: utilisation set to 100 %; latency ×10.
    /// - [`AttackType::ManInTheMiddle`]: adds 5 ms fixed processing overhead.
    ///
    /// # Errors
    /// Returns an error string if `link_idx` is out of range.
    pub fn simulate_cyber_attack_impact(
        &mut self,
        attack_type: AttackType,
        link_idx: usize,
        messages: &[CommMessage],
        duration_ms: f64,
    ) -> Result<AttackImpact, String> {
        // Clone the link so we can modify it for the attack scenario.
        let mut attacked_link = self
            .links
            .get(link_idx)
            .ok_or_else(|| format!("link index {link_idx} out of range"))?
            .clone();

        // Apply attack modifications.
        let mitm_overhead_ms = match attack_type {
            AttackType::DenialOfService => {
                // 100 % utilisation → force congestion multiplier of 10 × nominal.
                attacked_link.nominal_latency_ms *= 10.0;
                attacked_link.nominal_latency_ms = attacked_link
                    .nominal_latency_ms
                    .min(attacked_link.max_latency_ms);
                0.0
            }
            AttackType::ManInTheMiddle => 5.0,
        };

        // Temporarily replace the link in the simulator.
        let original_link = self.links[link_idx].clone();
        self.links[link_idx] = attacked_link;

        let mut delayed = 0usize;
        let mut lost = 0usize;
        let mut max_lat = 0.0_f64;
        let mut control_disrupted = false;

        // Use 100 % utilisation for DoS.
        let forced_util = if attack_type == AttackType::DenialOfService {
            1.0
        } else {
            0.0
        };

        for msg in messages {
            let only_in_window = msg.timestamp_sent_ms < duration_ms;
            if !only_in_window {
                continue;
            }

            // Re-borrow link after potential mutation.
            let link_snapshot = self.links[link_idx].clone();
            let lat_opt = self.sample_latency(&link_snapshot, msg.payload_bytes, forced_util);

            match lat_opt {
                None => {
                    lost += 1;
                    if msg.message_type == CommMessageType::ProtectionTrip
                        || msg.message_type == CommMessageType::ControlCommand
                    {
                        control_disrupted = true;
                    }
                }
                Some(mut lat) => {
                    lat += mitm_overhead_ms;
                    if lat > max_lat {
                        max_lat = lat;
                    }
                    let deadline = msg.message_type.deadline_ms();
                    if lat > deadline {
                        delayed += 1;
                        if msg.message_type == CommMessageType::ProtectionTrip
                            || msg.message_type == CommMessageType::ControlCommand
                        {
                            control_disrupted = true;
                        }
                    }
                }
            }
        }

        // Restore the original link.
        self.links[link_idx] = original_link;

        Ok(AttackImpact {
            messages_delayed: delayed,
            messages_lost: lost,
            max_latency_ms: max_lat,
            control_disrupted,
        })
    }

    /// Estimate the latency distribution of link `link_idx` via Monte-Carlo
    /// sampling of `n_samples` independent message deliveries.
    ///
    /// Only delivered (non-lost) samples contribute to the statistics.
    ///
    /// # Errors
    /// Returns an error string if `link_idx` is out of range or `n_samples` is 0.
    pub fn monte_carlo_latency_distribution(
        &mut self,
        link_idx: usize,
        n_samples: usize,
    ) -> Result<LatencyStats, String> {
        if n_samples == 0 {
            return Err("n_samples must be > 0".to_string());
        }
        let link = self
            .links
            .get(link_idx)
            .ok_or_else(|| format!("link index {link_idx} out of range"))?
            .clone();

        let mut samples: Vec<f64> = Vec::with_capacity(n_samples);

        for _ in 0..n_samples {
            // Use a fixed small payload (64 bytes) for baseline characterisation.
            if let Some(lat) = self.sample_latency(&link, 64, 0.0) {
                samples.push(lat);
            }
        }

        if samples.is_empty() {
            // All samples were lost — return zeros (degenerate case).
            return Ok(LatencyStats {
                mean_ms: 0.0,
                std_ms: 0.0,
                p95_ms: 0.0,
                p99_ms: 0.0,
                max_ms: 0.0,
            });
        }

        let n = samples.len() as f64;
        let mean_ms = samples.iter().sum::<f64>() / n;
        let variance = samples.iter().map(|&x| (x - mean_ms).powi(2)).sum::<f64>() / n;
        let std_ms = variance.sqrt();

        let mut sorted = samples.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let p95_ms = percentile_sorted(&sorted, 95.0);
        let p99_ms = percentile_sorted(&sorted, 99.0);
        let max_ms = sorted.last().copied().unwrap_or(0.0);

        Ok(LatencyStats {
            mean_ms,
            std_ms,
            p95_ms,
            p99_ms,
            max_ms,
        })
    }

    /// Generate upgrade recommendations for links that fail to meet `requirements`.
    ///
    /// Checks latency, packet loss, and availability against the supplied
    /// threshold and recommends the most appropriate protocol upgrade.
    pub fn recommend_comm_upgrade(
        &self,
        current_analysis: &LatencyStats,
        requirements: &PerformanceRequirements,
    ) -> Vec<UpgradeRecommendation> {
        let mut recs: Vec<UpgradeRecommendation> = Vec::new();

        for link in &self.links {
            // --- Latency check ---
            if current_analysis.p99_ms > requirements.max_latency_ms {
                let (rec_proto, improvement) = recommend_faster_protocol(link.protocol);
                recs.push(UpgradeRecommendation {
                    link_id: link.link_id.clone(),
                    issue: format!(
                        "P99 latency {:.2} ms exceeds requirement {:.2} ms",
                        current_analysis.p99_ms, requirements.max_latency_ms
                    ),
                    recommended_protocol: rec_proto,
                    expected_improvement: improvement,
                });
            }

            // --- Loss rate check (critical threshold 1e-3) ---
            if link.packet_loss_rate > requirements.max_loss_rate.max(1e-3) {
                recs.push(UpgradeRecommendation {
                    link_id: link.link_id.clone(),
                    issue: format!(
                        "packet loss rate {:.4} exceeds threshold {:.4}; redundant path recommended",
                        link.packet_loss_rate,
                        requirements.max_loss_rate
                    ),
                    recommended_protocol: CommProtocol::Fiber,
                    expected_improvement:
                        "Redundant fibre path reduces effective loss to < 1e-6".to_string(),
                });
            }

            // --- Availability check ---
            if link.availability < requirements.min_availability {
                recs.push(UpgradeRecommendation {
                    link_id: link.link_id.clone(),
                    issue: format!(
                        "availability {:.4} below required {:.4}; backup link recommended",
                        link.availability, requirements.min_availability
                    ),
                    recommended_protocol: CommProtocol::FiveGUrllc,
                    expected_improvement:
                        "5G URLLC backup raises composite availability above 0.9999".to_string(),
                });
            }
        }

        recs
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the `p`-th percentile (0–100) from a **pre-sorted** slice.
fn percentile_sorted(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let idx = ((p / 100.0) * (sorted.len() - 1) as f64).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

/// Choose a faster protocol when the current one has too-high latency.
fn recommend_faster_protocol(current: CommProtocol) -> (CommProtocol, String) {
    match current {
        CommProtocol::Wireless4G => (
            CommProtocol::FiveGUrllc,
            "5G URLLC reduces latency from 20–100 ms to < 1 ms".to_string(),
        ),
        CommProtocol::Dnp3Serial => (
            CommProtocol::Dnp3Tcp,
            "DNP3/TCP reduces latency from 50–500 ms to 10–100 ms".to_string(),
        ),
        CommProtocol::Dnp3Tcp | CommProtocol::ModbusTcp | CommProtocol::IEC60870_104 => (
            CommProtocol::IEC61850Goose,
            "IEC 61850 GOOSE reduces latency to < 4 ms".to_string(),
        ),
        CommProtocol::Mqtt => (
            CommProtocol::Fiber,
            "Dedicated fibre reduces latency to < 2 ms".to_string(),
        ),
        CommProtocol::IEC61850Goose | CommProtocol::IEC61850Sampled | CommProtocol::FiveGUrllc => (
            CommProtocol::FiveGUrllc,
            "Already on a low-latency protocol; consider 5G URLLC if not already deployed"
                .to_string(),
        ),
        CommProtocol::Fiber => (
            CommProtocol::IEC61850Sampled,
            "IEC 61850 Sampled Values over existing fibre achieves < 1 ms".to_string(),
        ),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────────

    fn make_link(id: &str, protocol: CommProtocol, loss: f64, latency_ms: f64) -> CommLink {
        CommLink {
            link_id: id.to_string(),
            protocol,
            bandwidth_kbps: 100_000.0, // 100 Mbps
            nominal_latency_ms: latency_ms,
            max_latency_ms: latency_ms * 10.0 + 50.0,
            packet_loss_rate: loss,
            jitter_ms: latency_ms * 0.1,
            availability: 0.9999,
        }
    }

    fn make_message(id: u64, msg_type: CommMessageType, bytes: usize) -> CommMessage {
        CommMessage {
            message_id: id,
            priority: MessagePriority::Normal,
            payload_bytes: bytes,
            timestamp_sent_ms: id as f64,
            source: "A".to_string(),
            destination: "B".to_string(),
            message_type: msg_type,
        }
    }

    fn default_sim(seed: u64) -> CommNetworkSim {
        CommNetworkSim::with_default_config(seed)
    }

    // ── Test 1: High loss → >50 % messages lost ───────────────────────────

    #[test]
    fn test_high_loss_rate_majority_dropped() {
        let mut sim = default_sim(42);
        let idx = sim.add_link(make_link("lossy", CommProtocol::Wireless4G, 0.8, 50.0));

        let mut lost = 0usize;
        let trials = 100usize;
        for i in 0..trials {
            let msg = make_message(i as u64, CommMessageType::Measurement, 128);
            let result = sim
                .simulate_message_delivery(&msg, idx)
                .expect("simulate_message_delivery failed");
            if !result.delivered {
                lost += 1;
            }
        }

        assert!(
            lost > 50,
            "Expected >50 % packet loss with 80 % loss rate, got {lost}/{trials} lost"
        );
    }

    // ── Test 2: Zero loss → all messages delivered ────────────────────────

    #[test]
    fn test_zero_loss_all_delivered() {
        let mut sim = default_sim(1234);
        let idx = sim.add_link(make_link("perfect", CommProtocol::Fiber, 0.0, 1.0));

        let trials = 50usize;
        for i in 0..trials {
            let msg = make_message(i as u64, CommMessageType::Measurement, 64);
            let result = sim
                .simulate_message_delivery(&msg, idx)
                .expect("simulate_message_delivery failed");
            assert!(
                result.delivered,
                "Zero-loss link must deliver every message; message {i} was lost"
            );
        }
    }

    // ── Test 3: Latency stats ordering (p95 > mean, p99 > p95) ───────────

    #[test]
    fn test_latency_stats_ordering() {
        let mut sim = CommNetworkSim::new(
            CommNetworkConfig {
                simulation_duration_ms: 10_000.0,
                congestion_model: CongestionModel::None,
                security_overhead_ms: 0.0,
            },
            7,
        );
        let idx = sim.add_link(make_link("jittery", CommProtocol::Wireless4G, 0.0, 50.0));

        let stats = sim
            .monte_carlo_latency_distribution(idx, 500)
            .expect("monte_carlo failed");

        assert!(
            stats.p95_ms >= stats.mean_ms,
            "P95 ({:.3}) must be >= mean ({:.3})",
            stats.p95_ms,
            stats.mean_ms
        );
        assert!(
            stats.p99_ms >= stats.p95_ms,
            "P99 ({:.3}) must be >= P95 ({:.3})",
            stats.p99_ms,
            stats.p95_ms
        );
        assert!(
            stats.max_ms >= stats.p99_ms,
            "max ({:.3}) must be >= P99 ({:.3})",
            stats.max_ms,
            stats.p99_ms
        );
    }

    // ── Test 4: GOOSE link < 4 ms → all protection messages on time ───────

    #[test]
    fn test_protection_goose_all_on_time() {
        let mut sim = CommNetworkSim::new(
            CommNetworkConfig {
                simulation_duration_ms: 10_000.0,
                congestion_model: CongestionModel::None,
                security_overhead_ms: 0.0, // no overhead so latency stays < 4 ms
            },
            99,
        );
        // GOOSE link with 1 ms nominal latency and 0.1 ms jitter → well inside 4 ms
        let idx = sim.add_link(CommLink {
            link_id: "goose-link".to_string(),
            protocol: CommProtocol::IEC61850Goose,
            bandwidth_kbps: 100_000.0,
            nominal_latency_ms: 1.0,
            max_latency_ms: 4.0,
            packet_loss_rate: 0.0,
            jitter_ms: 0.1,
            availability: 1.0,
        });

        let msgs: Vec<CommMessage> = (0..20)
            .map(|i| make_message(i, CommMessageType::ProtectionTrip, 128))
            .collect();

        let assessment = sim
            .assess_protection_performance(&msgs, idx)
            .expect("assess_protection_performance failed");

        assert!(
            (assessment.fraction_on_time - 1.0).abs() < 1e-9,
            "GOOSE link with 1 ms latency must deliver all messages on time; fraction={:.4}",
            assessment.fraction_on_time
        );
        assert_eq!(assessment.missed_deadlines, 0);
    }

    // ── Test 5: Channel utilisation formula check ─────────────────────────

    #[test]
    fn test_channel_utilization_calculation() {
        let mut sim = default_sim(0);
        sim.add_link(CommLink {
            link_id: "bw-link".to_string(),
            protocol: CommProtocol::Fiber,
            bandwidth_kbps: 1_000.0, // 1 Mbps
            nominal_latency_ms: 1.0,
            max_latency_ms: 10.0,
            packet_loss_rate: 0.0,
            jitter_ms: 0.0,
            availability: 1.0,
        });

        // 10 messages × 1000 bytes = 10_000 bytes = 80_000 bits
        // capacity = 1_000 kbps × 1000 bps/kbps × 0.1 s = 100_000 bits
        // utilization = 80_000 / 100_000 = 0.8
        let msgs: Vec<CommMessage> = (0..10)
            .map(|i| make_message(i, CommMessageType::Measurement, 1_000))
            .collect();

        let util = sim
            .calculate_channel_utilization(&msgs, 0, 100.0) // 100 ms window
            .expect("utilization failed");

        assert!(
            (util - 0.8).abs() < 1e-9,
            "Expected utilization 0.8, got {util:.6}"
        );
    }

    // ── Test 6: Large delay → control loop instability ────────────────────

    #[test]
    fn test_control_loop_large_delay_instability() {
        let mut sim = CommNetworkSim::new(
            CommNetworkConfig {
                simulation_duration_ms: 60_000.0,
                congestion_model: CongestionModel::None,
                security_overhead_ms: 0.0,
            },
            55,
        );
        // 200 ms latency link, control bandwidth 1 Hz → Nyquist limit = 250 ms
        // Two links in series → total ≈ 400 ms > 250 ms → unstable
        let idx0 = sim.add_link(make_link("slow1", CommProtocol::Dnp3Serial, 0.0, 200.0));
        let idx1 = sim.add_link(make_link("slow2", CommProtocol::Dnp3Serial, 0.0, 200.0));

        let msgs = vec![
            make_message(0, CommMessageType::ControlCommand, 64),
            make_message(1, CommMessageType::ControlCommand, 64),
        ];

        let analysis = sim
            .analyze_control_loop_delay(&[idx0, idx1], &msgs, 1.0)
            .expect("analyze_control_loop_delay failed");

        assert!(
            !analysis.stability_maintained,
            "Expected instability with {:.1} ms total delay vs {:.1} ms limit",
            analysis.total_latency_ms, analysis.stability_margin_ms
        );
    }

    // ── Test 7: Series link availability multiplies ────────────────────────

    #[test]
    fn test_series_network_availability() {
        let mut sim = default_sim(0);
        let idx0 = sim.add_link(CommLink {
            link_id: "L0".to_string(),
            protocol: CommProtocol::Fiber,
            bandwidth_kbps: 10_000.0,
            nominal_latency_ms: 1.0,
            max_latency_ms: 5.0,
            packet_loss_rate: 0.0,
            jitter_ms: 0.0,
            availability: 0.99,
        });
        let idx1 = sim.add_link(CommLink {
            link_id: "L1".to_string(),
            protocol: CommProtocol::Fiber,
            bandwidth_kbps: 10_000.0,
            nominal_latency_ms: 1.0,
            max_latency_ms: 5.0,
            packet_loss_rate: 0.0,
            jitter_ms: 0.0,
            availability: 0.98,
        });

        let avail = sim
            .network_availability(&[idx0, idx1], false)
            .expect("network_availability failed");

        let expected = 0.99 * 0.98;
        assert!(
            (avail - expected).abs() < 1e-12,
            "Series availability must be product: expected {expected:.6}, got {avail:.6}"
        );
    }

    // ── Test 8: Monte-Carlo 500 samples within expected range ─────────────

    #[test]
    fn test_monte_carlo_stats_within_expected_range() {
        let mut sim = CommNetworkSim::new(
            CommNetworkConfig {
                simulation_duration_ms: 60_000.0,
                congestion_model: CongestionModel::None,
                security_overhead_ms: 0.0,
            },
            314,
        );
        // Fiber link: 2 ms nominal, 0.2 ms jitter
        let idx = sim.add_link(CommLink {
            link_id: "fiber".to_string(),
            protocol: CommProtocol::Fiber,
            bandwidth_kbps: 1_000_000.0,
            nominal_latency_ms: 2.0,
            max_latency_ms: 20.0,
            packet_loss_rate: 0.0,
            jitter_ms: 0.2,
            availability: 1.0,
        });

        let stats = sim
            .monte_carlo_latency_distribution(idx, 500)
            .expect("monte_carlo failed");

        // Mean should be close to nominal (2.0 ms ± 1.0 ms tolerance)
        assert!(
            (stats.mean_ms - 2.0).abs() < 1.0,
            "Mean latency {:.3} ms should be near nominal 2.0 ms",
            stats.mean_ms
        );
        // Std should be small relative to jitter (≈ 0.2 ms ± 0.3 ms)
        assert!(
            stats.std_ms < 1.5,
            "Std dev {:.3} ms is implausibly large for a 0.2 ms jitter link",
            stats.std_ms
        );
        // P99 ≥ mean (always)
        assert!(
            stats.p99_ms >= stats.mean_ms,
            "P99 ({:.3}) must be >= mean ({:.3})",
            stats.p99_ms,
            stats.mean_ms
        );
    }

    // ── Test 9: Parallel link availability (1 - product of unavailability) ─

    #[test]
    fn test_parallel_network_availability() {
        let mut sim = default_sim(0);
        let idx0 = sim.add_link(CommLink {
            link_id: "P0".to_string(),
            protocol: CommProtocol::Fiber,
            bandwidth_kbps: 10_000.0,
            nominal_latency_ms: 1.0,
            max_latency_ms: 5.0,
            packet_loss_rate: 0.0,
            jitter_ms: 0.0,
            availability: 0.9,
        });
        let idx1 = sim.add_link(CommLink {
            link_id: "P1".to_string(),
            protocol: CommProtocol::Wireless4G,
            bandwidth_kbps: 10_000.0,
            nominal_latency_ms: 50.0,
            max_latency_ms: 200.0,
            packet_loss_rate: 0.0,
            jitter_ms: 5.0,
            availability: 0.9,
        });

        let avail = sim
            .network_availability(&[idx0, idx1], true)
            .expect("network_availability failed");

        let expected = 1.0 - (0.1 * 0.1); // 0.99
        assert!(
            (avail - expected).abs() < 1e-12,
            "Parallel availability must be 1-(1-a0)(1-a1): expected {expected:.4}, got {avail:.4}"
        );
    }

    // ── Test 10: Upgrade recommendation triggered by high latency ─────────

    #[test]
    fn test_upgrade_recommendation_high_latency() {
        let mut sim = default_sim(0);
        sim.add_link(make_link("slow", CommProtocol::Dnp3Serial, 0.0, 300.0));

        // Fake stats: p99 = 400 ms
        let stats = LatencyStats {
            mean_ms: 300.0,
            std_ms: 50.0,
            p95_ms: 380.0,
            p99_ms: 400.0,
            max_ms: 500.0,
        };
        let req = PerformanceRequirements {
            max_latency_ms: 100.0,
            max_loss_rate: 1e-3,
            min_availability: 0.999,
        };

        let recs = sim.recommend_comm_upgrade(&stats, &req);

        assert!(
            !recs.is_empty(),
            "Should produce at least one recommendation for a slow link"
        );
        // At least one recommendation should mention latency.
        assert!(
            recs.iter()
                .any(|r| r.issue.to_lowercase().contains("latency")),
            "Expected a latency-related recommendation"
        );
    }
}
