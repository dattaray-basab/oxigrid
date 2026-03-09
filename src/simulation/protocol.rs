//! Smart-grid communication protocol simulator.
//!
//! Models the network-level behaviour of major substation and IoT protocols:
//!
//! | Protocol | Standard | Typical use |
//! |----------|----------|-------------|
//! | `Iec61850Goose` | IEC 61850-8-1 | Fast protection tripping (< 4 ms) |
//! | `Iec61850Sampled` | IEC 61850-9-2 | Merging-unit voltage/current streams |
//! | `Dnp3` | IEEE 1815 | SCADA telemetry, outstation control |
//! | `Modbus` | Modicon 1979 | Legacy RTU/TCP sensor polling |
//! | `Mqtt` | OASIS 5.0 | IoT device telemetry |
//! | `CimXml` | IEC 61968/61970 | CIM data exchange |
//!
//! The simulator is **purely deterministic given an LCG seed**; no external PRNG
//! crates are used.  Latency jitter is generated with a Box–Muller-like
//! approximation from uniform LCG samples.
//!
//! # GOOSE delivery guarantee
//! IEC 61850 GOOSE uses rapid retransmission (T0 → T0/2 → … down to a minimum
//! period) to achieve > 99.9 % delivery despite UDP multicast.  Use
//! [`ProtocolSimulator::generate_goose_sequence`] to build realistic sequences.

use serde::{Deserialize, Serialize};
use std::fmt;

// ─────────────────────────────────────────────────────────────────────────────
// Error
// ─────────────────────────────────────────────────────────────────────────────

/// Errors from the protocol simulator.
#[derive(Debug, Clone, PartialEq)]
pub enum SimError {
    /// The requested link does not exist.
    LinkNotFound(usize),
    /// The simulation time or message list is invalid.
    InvalidInput(String),
}

impl fmt::Display for SimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::LinkNotFound(id) => write!(f, "link {id} not found"),
            Self::InvalidInput(m) => write!(f, "invalid input: {m}"),
        }
    }
}

impl std::error::Error for SimError {}

// ─────────────────────────────────────────────────────────────────────────────
// Protocol and message types
// ─────────────────────────────────────────────────────────────────────────────

/// Communication protocol flavour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProtocolType {
    /// IEC 61850 GOOSE — Generic Object Oriented Substation Events.
    Iec61850Goose,
    /// IEC 61850 Sampled Values (SV/9-2LE).
    Iec61850Sampled,
    /// Distributed Network Protocol 3 (IEEE 1815).
    Dnp3,
    /// Modbus RTU or TCP.
    Modbus,
    /// MQTT 5.0 for IoT telemetry.
    Mqtt,
    /// CIM-based XML data exchange (IEC 61968/61970).
    CimXml,
}

/// Semantic classification of a message.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MessageType {
    /// Protection trip, close, or setpoint command.
    ControlCommand,
    /// Breaker status, measurement report.
    StatusReport,
    /// High-speed voltage/current samples (SV).
    SampledValues,
    /// Alarm or event notification.
    Alarm,
    /// Periodic keep-alive.
    Heartbeat,
    /// Time synchronisation (e.g. PPS, 1588 PTP).
    TimeSync,
}

/// A single protocol message.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Message {
    /// Unique message identifier.
    pub id: u64,
    /// Protocol type.
    pub protocol: ProtocolType,
    /// Source address / APPID.
    pub source: u32,
    /// Destination address.
    pub destination: u32,
    /// Semantic type of the message.
    pub message_type: MessageType,
    /// Simulated raw payload bytes.
    pub payload: Vec<u8>,
    /// Origination timestamp `\[µs\]`.
    pub timestamp_us: u64,
    /// IEEE 802.1p priority (0–7).
    pub priority: u8,
    /// Total message size on the wire `\[bytes\]`.
    pub size_bytes: usize,
}

// ─────────────────────────────────────────────────────────────────────────────
// Network link
// ─────────────────────────────────────────────────────────────────────────────

/// A simplex network link connecting two communication endpoints.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkLink {
    /// Unique link identifier (used as index in the simulator).
    pub id: usize,
    /// Available bandwidth `\[Mbps\]`.
    pub bandwidth_mbps: f64,
    /// Propagation + processing latency `\[ms\]`.
    pub latency_ms: f64,
    /// Fraction of packets lost due to congestion or interference `\[0, 1\]`.
    pub packet_loss_rate: f64,
    /// One-sigma latency jitter (Gaussian approximation) `\[ms\]`.
    pub jitter_ms: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Statistics
// ─────────────────────────────────────────────────────────────────────────────

/// Aggregate statistics from one simulation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtocolSimResult {
    /// Total messages submitted to the link.
    pub messages_sent: usize,
    /// Messages successfully received (not lost, within time window).
    pub messages_received: usize,
    /// Messages dropped due to packet loss or bandwidth overflow.
    pub messages_lost: usize,
    /// Mean end-to-end latency of delivered messages `\[ms\]`.
    pub avg_latency_ms: f64,
    /// Maximum observed latency `\[ms\]`.
    pub max_latency_ms: f64,
    /// 99th-percentile latency of delivered messages `\[ms\]`.
    pub p99_latency_ms: f64,
    /// Effective throughput `\[kbps\]`.
    pub throughput_kbps: f64,
    /// Fraction of GOOSE messages delivered `\[%\]`, 0–100.
    pub goose_delivery_pct: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// LCG PRNG
// ─────────────────────────────────────────────────────────────────────────────

struct Lcg {
    state: u64,
}

impl Lcg {
    const MULT: u64 = 6_364_136_223_846_793_005;
    const ADD: u64 = 1_442_695_040_888_963_407;

    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    /// Next `u64` value.
    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_mul(Self::MULT).wrapping_add(Self::ADD);
        self.state
    }

    /// Uniform `f64` in `\[0, 1)`.
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Approximate Gaussian sample via 12-uniform sum (central limit theorem).
    /// Mean = 0, σ ≈ 1.
    fn next_normal(&mut self) -> f64 {
        let sum: f64 = (0..12).map(|_| self.next_f64()).sum();
        sum - 6.0
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Simulator
// ─────────────────────────────────────────────────────────────────────────────

/// Smart-grid communication protocol simulator.
pub struct ProtocolSimulator {
    links: Vec<NetworkLink>,
    /// Log of `(message, arrival_time_ms, delivered)`.
    message_log: Vec<(Message, f64, bool)>,
    lcg: Lcg,
}

impl Default for ProtocolSimulator {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolSimulator {
    /// Create a new simulator with no links.
    pub fn new() -> Self {
        Self {
            links: Vec::new(),
            message_log: Vec::new(),
            lcg: Lcg::new(0xDEAD_BEEF_1234_5678),
        }
    }

    /// Register a network link.
    pub fn add_link(&mut self, link: NetworkLink) {
        self.links.push(link);
    }

    /// Simulate transmission of a batch of messages over link `link_id`.
    ///
    /// For each message:
    /// 1. Compute base latency = `link.latency_ms`.
    /// 2. Add Gaussian jitter bounded to ±2σ.
    /// 3. Apply packet loss: LCG uniform < `loss_rate` → drop.
    /// 4. Apply bandwidth queuing: accumulate bytes transmitted; if burst
    ///    saturates the link capacity within the simulation window the excess
    ///    messages experience additional serialisation delay.
    ///
    /// Returns aggregate statistics.
    pub fn simulate(
        &mut self,
        messages: Vec<Message>,
        link_id: usize,
        simulation_time_ms: f64,
    ) -> Result<ProtocolSimResult, SimError> {
        if simulation_time_ms <= 0.0 {
            return Err(SimError::InvalidInput(
                "simulation_time_ms must be positive".to_string(),
            ));
        }
        let link = self
            .links
            .iter()
            .find(|l| l.id == link_id)
            .ok_or(SimError::LinkNotFound(link_id))?
            .clone();

        let bandwidth_bytes_per_ms = link.bandwidth_mbps * 1_000_000.0 / 8.0 / 1_000.0;

        let mut queue_time_ms = 0.0_f64; // next available slot in the queue
        let mut latencies: Vec<f64> = Vec::new();
        let mut goose_sent = 0usize;
        let mut goose_received = 0usize;
        let mut total_bytes_delivered = 0usize;

        for msg in &messages {
            let is_goose = msg.protocol == ProtocolType::Iec61850Goose;
            if is_goose {
                goose_sent += 1;
            }

            // Packet loss check
            let loss_roll = self.lcg.next_f64();
            if loss_roll < link.packet_loss_rate {
                self.message_log.push((msg.clone(), 0.0, false));
                continue;
            }

            // Base latency + jitter (bounded to ±2σ)
            let raw_jitter = self.lcg.next_normal() * link.jitter_ms;
            let jitter = raw_jitter.clamp(-2.0 * link.jitter_ms, 2.0 * link.jitter_ms);
            let latency_base = (link.latency_ms + jitter).max(0.0);

            // Serialisation / queuing delay
            let tx_time_ms = msg.size_bytes as f64 / bandwidth_bytes_per_ms;
            let msg_dispatch_time = msg.timestamp_us as f64 / 1_000.0; // µs → ms
            let start_tx = queue_time_ms.max(msg_dispatch_time);
            queue_time_ms = start_tx + tx_time_ms;
            let arrival_ms = start_tx + tx_time_ms + latency_base;

            if arrival_ms > simulation_time_ms {
                // Arrived after window — treat as lost
                self.message_log.push((msg.clone(), arrival_ms, false));
                continue;
            }

            let total_latency = arrival_ms - msg_dispatch_time;
            latencies.push(total_latency);
            total_bytes_delivered += msg.size_bytes;
            self.message_log.push((msg.clone(), arrival_ms, true));
            if is_goose {
                goose_received += 1;
            }
        }

        let messages_sent = messages.len();
        let messages_received = latencies.len();
        let messages_lost = messages_sent - messages_received;

        let avg_latency_ms = if latencies.is_empty() {
            0.0
        } else {
            latencies.iter().sum::<f64>() / latencies.len() as f64
        };

        let max_latency_ms = latencies
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
            .max(0.0);

        let p99_latency_ms = percentile_sorted(&latencies, 99.0);

        let throughput_kbps = if simulation_time_ms > 0.0 {
            total_bytes_delivered as f64 * 8.0 / simulation_time_ms // bits/ms = kbps
        } else {
            0.0
        };

        let goose_delivery_pct = if goose_sent == 0 {
            100.0
        } else {
            goose_received as f64 / goose_sent as f64 * 100.0
        };

        Ok(ProtocolSimResult {
            messages_sent,
            messages_received,
            messages_lost,
            avg_latency_ms,
            max_latency_ms,
            p99_latency_ms,
            throughput_kbps,
            goose_delivery_pct,
        })
    }

    /// Generate a realistic GOOSE retransmission sequence.
    ///
    /// After the initial event at `event_time_ms`, IEC 61850 requires the
    /// publisher to retransmit with exponentially-increasing intervals starting
    /// at ~2 ms, doubling each time up to a maximum of 1000 ms.
    ///
    /// `n_retransmit` controls how many retransmissions follow the initial event.
    pub fn generate_goose_sequence(event_time_ms: f64, n_retransmit: usize) -> Vec<Message> {
        let mut msgs: Vec<Message> = Vec::with_capacity(n_retransmit + 1);

        // Initial event message
        msgs.push(Message {
            id: 0,
            protocol: ProtocolType::Iec61850Goose,
            source: 0x0001,
            destination: 0xFFFF,
            message_type: MessageType::ControlCommand,
            payload: vec![0x01, 0x00, 0x00, 0x00], // simulated GOOSE payload
            timestamp_us: (event_time_ms * 1_000.0) as u64,
            priority: 6,     // GOOSE uses VLAN priority 6
            size_bytes: 128, // typical GOOSE PDU
        });

        // Retransmissions with doubling interval
        let mut interval_ms = 2.0_f64;
        let max_interval_ms = 1_000.0_f64;
        let mut t = event_time_ms + interval_ms;

        for i in 1..=n_retransmit {
            msgs.push(Message {
                id: i as u64,
                protocol: ProtocolType::Iec61850Goose,
                source: 0x0001,
                destination: 0xFFFF,
                message_type: MessageType::ControlCommand,
                payload: vec![0x01, 0x00, 0x00, 0x00],
                timestamp_us: (t * 1_000.0) as u64,
                priority: 6,
                size_bytes: 128,
            });
            interval_ms = (interval_ms * 2.0).min(max_interval_ms);
            t += interval_ms;
        }

        msgs
    }

    /// Aggregate statistics over all messages transmitted so far.
    pub fn message_statistics(&self) -> ProtocolSimResult {
        let messages_sent = self.message_log.len();
        let delivered: Vec<f64> = self
            .message_log
            .iter()
            .filter(|(_, _, ok)| *ok)
            .map(|(msg, arrival, _)| {
                let dispatch_ms = msg.timestamp_us as f64 / 1_000.0;
                (*arrival - dispatch_ms).max(0.0)
            })
            .collect();

        let messages_received = delivered.len();
        let messages_lost = messages_sent - messages_received;
        let avg_latency_ms = if delivered.is_empty() {
            0.0
        } else {
            delivered.iter().sum::<f64>() / delivered.len() as f64
        };
        let max_latency_ms = delivered
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
            .max(0.0);
        let p99_latency_ms = percentile_sorted(&delivered, 99.0);

        let total_bytes: usize = self
            .message_log
            .iter()
            .filter(|(_, _, ok)| *ok)
            .map(|(msg, _, _)| msg.size_bytes)
            .sum();
        // Use max arrival as simulation window
        let t_max = self
            .message_log
            .iter()
            .map(|(_, t, _)| *t)
            .fold(0.0_f64, f64::max);
        let throughput_kbps = if t_max > 0.0 {
            total_bytes as f64 * 8.0 / t_max
        } else {
            0.0
        };

        let goose_sent: usize = self
            .message_log
            .iter()
            .filter(|(msg, _, _)| msg.protocol == ProtocolType::Iec61850Goose)
            .count();
        let goose_received: usize = self
            .message_log
            .iter()
            .filter(|(msg, _, ok)| msg.protocol == ProtocolType::Iec61850Goose && *ok)
            .count();
        let goose_delivery_pct = if goose_sent == 0 {
            100.0
        } else {
            goose_received as f64 / goose_sent as f64 * 100.0
        };

        ProtocolSimResult {
            messages_sent,
            messages_received,
            messages_lost,
            avg_latency_ms,
            max_latency_ms,
            p99_latency_ms,
            throughput_kbps,
            goose_delivery_pct,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the `p`-th percentile (0–100) from an unsorted slice.
fn percentile_sorted(data: &[f64], p: f64) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let mut sorted = data.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let idx = ((p / 100.0) * (sorted.len() - 1) as f64).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_link(id: usize, loss: f64) -> NetworkLink {
        NetworkLink {
            id,
            bandwidth_mbps: 100.0,
            latency_ms: 1.0,
            packet_loss_rate: loss,
            jitter_ms: 0.1,
        }
    }

    fn make_messages(n: usize, proto: ProtocolType) -> Vec<Message> {
        (0..n)
            .map(|i| Message {
                id: i as u64,
                protocol: proto,
                source: 1,
                destination: 2,
                message_type: MessageType::StatusReport,
                payload: vec![0u8; 64],
                timestamp_us: i as u64 * 1_000, // 1 ms apart
                priority: 4,
                size_bytes: 100,
            })
            .collect()
    }

    #[test]
    fn test_zero_loss_link_all_delivered() {
        let mut sim = ProtocolSimulator::new();
        sim.add_link(make_link(0, 0.0));
        let msgs = make_messages(50, ProtocolType::Dnp3);
        let result = sim.simulate(msgs, 0, 10_000.0).unwrap();
        assert_eq!(
            result.messages_lost, 0,
            "zero-loss link must deliver all messages"
        );
        assert_eq!(result.messages_received, 50);
    }

    #[test]
    fn test_high_loss_link_reduces_delivery() {
        let mut sim = ProtocolSimulator::new();
        // 50 % loss
        sim.add_link(make_link(1, 0.5));
        let msgs = make_messages(1000, ProtocolType::Modbus);
        let result = sim.simulate(msgs, 1, 100_000.0).unwrap();
        // Expect roughly 50 % lost — allow wide tolerance for LCG variance
        let loss_pct = result.messages_lost as f64 / 1000.0 * 100.0;
        assert!(
            loss_pct > 30.0 && loss_pct < 70.0,
            "50 % loss link should lose 30–70 % of messages, lost {:.1} %",
            loss_pct
        );
    }

    #[test]
    fn test_goose_sequence_retransmit_intervals() {
        let event_t = 100.0_f64;
        let msgs = ProtocolSimulator::generate_goose_sequence(event_t, 5);
        assert_eq!(msgs.len(), 6, "1 initial + 5 retransmissions");
        // First message at event time
        assert_eq!(msgs[0].timestamp_us, (event_t * 1_000.0) as u64);
        // All are GOOSE
        assert!(msgs
            .iter()
            .all(|m| m.protocol == ProtocolType::Iec61850Goose));
        // Retransmit intervals must be non-decreasing (doubling up to 1000 ms)
        let times_ms: Vec<f64> = msgs
            .iter()
            .map(|m| m.timestamp_us as f64 / 1_000.0)
            .collect();
        for i in 1..times_ms.len().saturating_sub(1) {
            let gap_i = times_ms[i] - times_ms[i - 1];
            let gap_ip1 = times_ms[i + 1] - times_ms[i];
            // Each gap should be ≥ the previous (doubling schedule)
            assert!(
                gap_ip1 >= gap_i - 1e-9,
                "GOOSE retransmit interval should be non-decreasing: gap[{i}]={gap_i:.2} ms, gap[{i}+1]={gap_ip1:.2} ms"
            );
        }
    }

    #[test]
    fn test_latency_statistics_consistency() {
        let mut sim = ProtocolSimulator::new();
        sim.add_link(make_link(2, 0.0));
        let msgs = make_messages(200, ProtocolType::Mqtt);
        let result = sim.simulate(msgs, 2, 100_000.0).unwrap();
        // avg ≤ max
        assert!(
            result.avg_latency_ms <= result.max_latency_ms + 1e-9,
            "avg ({:.3}) must be ≤ max ({:.3})",
            result.avg_latency_ms,
            result.max_latency_ms
        );
        // p99 between avg and max
        assert!(
            result.p99_latency_ms >= result.avg_latency_ms - 1e-9,
            "P99 ({:.3}) should be ≥ avg ({:.3})",
            result.p99_latency_ms,
            result.avg_latency_ms
        );
        assert!(
            result.p99_latency_ms <= result.max_latency_ms + 1e-9,
            "P99 ({:.3}) should be ≤ max ({:.3})",
            result.p99_latency_ms,
            result.max_latency_ms
        );
    }

    #[test]
    fn test_bandwidth_throughput_within_link_capacity() {
        let link_mbps = 1.0_f64;
        let mut sim = ProtocolSimulator::new();
        sim.add_link(NetworkLink {
            id: 3,
            bandwidth_mbps: link_mbps,
            latency_ms: 0.5,
            packet_loss_rate: 0.0,
            jitter_ms: 0.01,
        });
        // 100 messages × 1000 bytes = 100 kB
        let msgs: Vec<Message> = (0..100)
            .map(|i| Message {
                id: i,
                protocol: ProtocolType::Dnp3,
                source: 1,
                destination: 2,
                message_type: MessageType::StatusReport,
                payload: vec![0u8; 1000],
                timestamp_us: 0,
                priority: 4,
                size_bytes: 1000,
            })
            .collect();
        // Large simulation window so all messages fit
        let result = sim.simulate(msgs, 3, 10_000.0).unwrap();
        // Throughput must not exceed link capacity (1 Mbps = 1000 kbps)
        assert!(
            result.throughput_kbps <= link_mbps * 1_000.0 + 1.0,
            "Throughput ({:.1} kbps) must not exceed link capacity ({:.0} kbps)",
            result.throughput_kbps,
            link_mbps * 1_000.0
        );
    }

    #[test]
    fn test_goose_delivery_pct_with_zero_loss() {
        let mut sim = ProtocolSimulator::new();
        sim.add_link(make_link(4, 0.0));
        let msgs = ProtocolSimulator::generate_goose_sequence(0.0, 4);
        let result = sim.simulate(msgs, 4, 100_000.0).unwrap();
        assert!(
            (result.goose_delivery_pct - 100.0).abs() < 1e-9,
            "Zero-loss GOOSE delivery should be 100 %, got {:.2} %",
            result.goose_delivery_pct
        );
    }

    #[test]
    fn test_link_not_found_error() {
        let mut sim = ProtocolSimulator::new();
        let msgs = make_messages(1, ProtocolType::Dnp3);
        let err = sim.simulate(msgs, 99, 1000.0).unwrap_err();
        assert!(matches!(err, SimError::LinkNotFound(99)));
    }

    #[test]
    fn test_invalid_simulation_time() {
        let mut sim = ProtocolSimulator::new();
        sim.add_link(make_link(5, 0.0));
        let msgs = make_messages(1, ProtocolType::Dnp3);
        let err = sim.simulate(msgs, 5, -1.0).unwrap_err();
        assert!(matches!(err, SimError::InvalidInput(_)));
    }
}
