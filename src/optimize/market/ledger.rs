//! Blockchain-inspired distributed ledger for transparent energy trading.
//!
//! Implements a hash-chained block structure for auditable peer-to-peer energy
//! market settlement. FNV-1a hashing provides chain integrity without any
//! cryptographic primitives.
//!
//! # Design
//! - Participants submit `EnergyTransaction` records to a pending pool.
//! - The operator calls `settle_block` to finalize a batch of transactions into
//!   an `EnergyBlock`, updating participant balances.
//! - Each block records the FNV-1a hash of the previous block, enabling tamper
//!   detection via `validate_chain`.
//!
//! # References
//! - Nakamoto, S., "Bitcoin: A Peer-to-Peer Electronic Cash System", 2008
//! - Fowl, A. et al., "Blockchain-Based Electricity Markets", IEEE Access, 2020

use thiserror::Error;

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors produced by [`EnergyLedger`] operations.
#[derive(Debug, Error)]
pub enum LedgerError {
    /// Participant ID does not exist in the ledger.
    #[error("Participant {0} not found")]
    ParticipantNotFound(usize),
    /// Attempted transaction references negative or zero energy.
    #[error("Invalid transaction: {0}")]
    InvalidTransaction(String),
    /// Settlement attempted with no pending transactions.
    #[error("No pending transactions to settle")]
    NoPendingTransactions,
}

// ── Domain enumerations ───────────────────────────────────────────────────────

/// Classification of energy source for traded \[MWh\].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EnergyType {
    /// Conventional thermal generation.
    Conventional,
    /// Solar photovoltaic generation.
    Solar,
    /// Wind power generation.
    Wind,
    /// Hydroelectric generation.
    Hydro,
    /// Battery or pumped-hydro storage dispatch.
    Storage,
    /// Electric vehicle (V2G) discharge.
    Ev,
}

/// Lifecycle status of an energy transaction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxStatus {
    /// Submitted but not yet included in a block.
    Pending,
    /// Validated and ready for settlement (intermediate state).
    Confirmed,
    /// Included in a settled block; balances updated.
    Settled,
    /// Rejected due to validation failure.
    Rejected,
}

/// Role of a market participant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParticipantRole {
    /// Pure generator — sells energy.
    Producer,
    /// Pure load — buys energy.
    Consumer,
    /// Both generates and consumes (e.g. rooftop solar + EV).
    Prosumer,
    /// Aggregates multiple DERs into a single market entity.
    Aggregator,
    /// System/grid operator (neutral, manages constraints).
    GridOperator,
}

// ── Core data structures ──────────────────────────────────────────────────────

/// A registered participant in the energy trading market.
#[derive(Debug, Clone)]
pub struct Participant {
    /// Unique participant identifier.
    pub id: usize,
    /// Human-readable display name.
    pub name: String,
    /// Market role.
    pub role: ParticipantRole,
    /// Current cash balance \[USD\].
    pub balance_usd: f64,
    /// Net energy position \[MWh\] (positive = net seller, negative = net buyer).
    pub energy_balance_mwh: f64,
}

/// A peer-to-peer energy trade between two participants.
#[derive(Debug, Clone)]
pub struct EnergyTransaction {
    /// Unique monotonically increasing transaction identifier.
    pub tx_id: u64,
    /// Seller participant ID.
    pub seller_id: usize,
    /// Buyer participant ID.
    pub buyer_id: usize,
    /// Energy volume traded \[MWh\].
    pub energy_mwh: f64,
    /// Agreed bilateral price \[USD/MWh\].
    pub price_usd_per_mwh: f64,
    /// Target delivery hour (0–8759 for a full year horizon).
    pub delivery_hour: usize,
    /// Source technology of the traded energy.
    pub energy_type: EnergyType,
    /// Submission timestamp (simulated unix seconds).
    pub timestamp: u64,
    /// Current lifecycle status.
    pub status: TxStatus,
}

/// An immutable settled block in the energy trading chain.
///
/// Each block commits a batch of transactions and links to its predecessor
/// via `prev_block_hash`, enabling integrity verification.
#[derive(Debug, Clone)]
pub struct EnergyBlock {
    /// Sequential block number (0 = genesis).
    pub block_id: u64,
    /// Block creation timestamp (simulated unix seconds).
    pub timestamp: u64,
    /// Settled transactions included in this block.
    pub transactions: Vec<EnergyTransaction>,
    /// FNV-1a hash of the previous block's data (0 for the genesis block).
    pub prev_block_hash: u64,
    /// FNV-1a hash of this block's serialised data.
    pub block_hash: u64,
    /// Volume-weighted average settlement price \[USD/MWh\].
    pub settlement_price_usd_per_mwh: f64,
    /// Total energy settled in this block \[MWh\].
    pub total_energy_mwh: f64,
}

// ── EnergyLedger ─────────────────────────────────────────────────────────────

/// Distributed ledger for transparent, auditable energy trading.
///
/// Transactions are submitted to a pending pool and periodically settled into
/// hash-chained [`EnergyBlock`]s.  The chain can be verified at any time with
/// [`validate_chain`](EnergyLedger::validate_chain).
#[derive(Debug, Default)]
pub struct EnergyLedger {
    /// Settled blocks forming the chain (index = block_id).
    pub chain: Vec<EnergyBlock>,
    /// Transactions awaiting settlement into the next block.
    pub pending_transactions: Vec<EnergyTransaction>,
    /// All registered market participants.
    pub participants: Vec<Participant>,
    next_tx_id: u64,
}

impl EnergyLedger {
    /// Create a new empty ledger with no participants or blocks.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a participant in the ledger.
    ///
    /// If a participant with the same `id` already exists it is replaced.
    pub fn register_participant(&mut self, participant: Participant) {
        if let Some(existing) = self
            .participants
            .iter_mut()
            .find(|p| p.id == participant.id)
        {
            *existing = participant;
        } else {
            self.participants.push(participant);
        }
    }

    /// Submit a new energy transaction to the pending pool.
    ///
    /// Assigns a monotonic `tx_id` and sets status to `Pending`.
    ///
    /// # Errors
    /// - [`LedgerError::InvalidTransaction`] — `energy_mwh ≤ 0` or `price_usd_per_mwh < 0`.
    /// - [`LedgerError::ParticipantNotFound`] — seller or buyer not registered.
    ///
    /// # Returns
    /// The assigned `tx_id` of the new transaction.
    pub fn submit_transaction(&mut self, mut tx: EnergyTransaction) -> Result<u64, LedgerError> {
        if tx.energy_mwh <= 0.0 {
            return Err(LedgerError::InvalidTransaction(
                "energy_mwh must be positive".to_string(),
            ));
        }
        if tx.price_usd_per_mwh < 0.0 {
            return Err(LedgerError::InvalidTransaction(
                "price_usd_per_mwh must be non-negative".to_string(),
            ));
        }
        if !self.participants.iter().any(|p| p.id == tx.seller_id) {
            return Err(LedgerError::ParticipantNotFound(tx.seller_id));
        }
        if !self.participants.iter().any(|p| p.id == tx.buyer_id) {
            return Err(LedgerError::ParticipantNotFound(tx.buyer_id));
        }

        tx.tx_id = self.next_tx_id;
        tx.status = TxStatus::Pending;
        self.next_tx_id += 1;
        let id = tx.tx_id;
        self.pending_transactions.push(tx);
        Ok(id)
    }

    /// Settle all pending transactions into a new block.
    ///
    /// All pending transactions are moved to `Settled`, participant balances are
    /// updated, and the block is appended to the chain.
    ///
    /// # Parameters
    /// - `settlement_price_usd_per_mwh` — reference clearing price \[USD/MWh\] stored in the block header.
    ///
    /// # Errors
    /// - [`LedgerError::NoPendingTransactions`] — nothing to settle.
    pub fn settle_block(
        &mut self,
        settlement_price_usd_per_mwh: f64,
    ) -> Result<EnergyBlock, LedgerError> {
        if self.pending_transactions.is_empty() {
            return Err(LedgerError::NoPendingTransactions);
        }

        let block_id = self.chain.len() as u64;
        // Simulated unix timestamp: genesis at 1_700_000_000, one hour per block.
        let timestamp = 1_700_000_000u64 + block_id * 3600;

        let mut settled: Vec<EnergyTransaction> = self.pending_transactions.drain(..).collect();
        let mut total_energy = 0.0f64;

        for tx in settled.iter_mut() {
            tx.status = TxStatus::Settled;
            let cost = tx.energy_mwh * tx.price_usd_per_mwh;

            if let Some(seller) = self.participants.iter_mut().find(|p| p.id == tx.seller_id) {
                seller.balance_usd += cost;
                seller.energy_balance_mwh -= tx.energy_mwh;
            }
            if let Some(buyer) = self.participants.iter_mut().find(|p| p.id == tx.buyer_id) {
                buyer.balance_usd -= cost;
                buyer.energy_balance_mwh += tx.energy_mwh;
            }

            total_energy += tx.energy_mwh;
        }

        let prev_block_hash = self.chain.last().map(|b| b.block_hash).unwrap_or(0u64);

        let block_hash = Self::compute_block_hash(
            block_id,
            timestamp,
            &settled,
            prev_block_hash,
            settlement_price_usd_per_mwh,
            total_energy,
        );

        let block = EnergyBlock {
            block_id,
            timestamp,
            transactions: settled,
            prev_block_hash,
            block_hash,
            settlement_price_usd_per_mwh,
            total_energy_mwh: total_energy,
        };

        self.chain.push(block.clone());
        Ok(block)
    }

    /// Validate chain integrity by re-computing and comparing block hashes.
    ///
    /// Returns `true` if:
    /// - Every block's `prev_block_hash` matches the preceding block's `block_hash`.
    /// - Every block's `block_hash` matches its recomputed FNV-1a hash.
    pub fn validate_chain(&self) -> bool {
        for (i, block) in self.chain.iter().enumerate() {
            let expected_prev = if i == 0 {
                0u64
            } else {
                self.chain[i - 1].block_hash
            };
            if block.prev_block_hash != expected_prev {
                return false;
            }
            let recomputed = Self::compute_block_hash(
                block.block_id,
                block.timestamp,
                &block.transactions,
                block.prev_block_hash,
                block.settlement_price_usd_per_mwh,
                block.total_energy_mwh,
            );
            if recomputed != block.block_hash {
                return false;
            }
        }
        true
    }

    /// FNV-1a 64-bit hash over arbitrary bytes.
    ///
    /// Uses the standard FNV-1a parameters:
    /// - offset basis: 14695981039346656037
    /// - prime: 1099511628211
    pub fn fnv1a_hash(data: &[u8]) -> u64 {
        const FNV_PRIME: u64 = 1_099_511_628_211;
        const FNV_OFFSET: u64 = 14_695_981_039_346_656_037;
        let mut hash = FNV_OFFSET;
        for &byte in data {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(FNV_PRIME);
        }
        hash
    }

    /// Get all transactions (settled or pending) involving participant `id`.
    ///
    /// Settled transactions are returned in block order; pending ones follow.
    pub fn participant_history(&self, id: usize) -> Vec<&EnergyTransaction> {
        let mut result: Vec<&EnergyTransaction> = Vec::new();
        for block in &self.chain {
            for tx in &block.transactions {
                if tx.seller_id == id || tx.buyer_id == id {
                    result.push(tx);
                }
            }
        }
        for tx in &self.pending_transactions {
            if tx.seller_id == id || tx.buyer_id == id {
                result.push(tx);
            }
        }
        result
    }

    /// Compute min, max, and volume-weighted average price for block `block_id`.
    ///
    /// Returns `Some((min \[USD/MWh\], max \[USD/MWh\], avg \[USD/MWh\]))`, or `None`
    /// if the block does not exist or contains no transactions.
    pub fn block_statistics(&self, block_id: u64) -> Option<(f64, f64, f64)> {
        let block = self.chain.iter().find(|b| b.block_id == block_id)?;
        if block.transactions.is_empty() {
            return None;
        }

        let mut min_p = f64::MAX;
        let mut max_p = f64::MIN;
        let mut total_mwh = 0.0f64;
        let mut total_value = 0.0f64;

        for tx in &block.transactions {
            if tx.price_usd_per_mwh < min_p {
                min_p = tx.price_usd_per_mwh;
            }
            if tx.price_usd_per_mwh > max_p {
                max_p = tx.price_usd_per_mwh;
            }
            total_mwh += tx.energy_mwh;
            total_value += tx.energy_mwh * tx.price_usd_per_mwh;
        }

        let avg = if total_mwh > 1e-12 {
            total_value / total_mwh
        } else {
            0.0
        };
        Some((min_p, max_p, avg))
    }

    /// Current USD balance of participant `id`, or `None` if not registered.
    pub fn participant_balance(&self, id: usize) -> Option<f64> {
        self.participants
            .iter()
            .find(|p| p.id == id)
            .map(|p| p.balance_usd)
    }

    /// Total energy settled across all blocks \[MWh\].
    pub fn total_traded_mwh(&self) -> f64 {
        self.chain.iter().map(|b| b.total_energy_mwh).sum()
    }

    // ── Private ───────────────────────────────────────────────────────────────

    fn compute_block_hash(
        block_id: u64,
        timestamp: u64,
        transactions: &[EnergyTransaction],
        prev_hash: u64,
        settlement_price: f64,
        total_energy: f64,
    ) -> u64 {
        let mut data: Vec<u8> = Vec::with_capacity(128 + transactions.len() * 48);
        data.extend_from_slice(&block_id.to_le_bytes());
        data.extend_from_slice(&timestamp.to_le_bytes());
        data.extend_from_slice(&prev_hash.to_le_bytes());
        data.extend_from_slice(&settlement_price.to_bits().to_le_bytes());
        data.extend_from_slice(&total_energy.to_bits().to_le_bytes());
        for tx in transactions {
            data.extend_from_slice(&tx.tx_id.to_le_bytes());
            data.extend_from_slice(&(tx.seller_id as u64).to_le_bytes());
            data.extend_from_slice(&(tx.buyer_id as u64).to_le_bytes());
            data.extend_from_slice(&tx.energy_mwh.to_bits().to_le_bytes());
            data.extend_from_slice(&tx.price_usd_per_mwh.to_bits().to_le_bytes());
            data.extend_from_slice(&(tx.delivery_hour as u64).to_le_bytes());
            data.extend_from_slice(&tx.timestamp.to_le_bytes());
        }
        Self::fnv1a_hash(&data)
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_participant(id: usize, role: ParticipantRole) -> Participant {
        Participant {
            id,
            name: format!("P{id}"),
            role,
            balance_usd: 10_000.0,
            energy_balance_mwh: 0.0,
        }
    }

    fn make_tx(seller: usize, buyer: usize, mwh: f64, price: f64) -> EnergyTransaction {
        EnergyTransaction {
            tx_id: 0,
            seller_id: seller,
            buyer_id: buyer,
            energy_mwh: mwh,
            price_usd_per_mwh: price,
            delivery_hour: 10,
            energy_type: EnergyType::Solar,
            timestamp: 1_700_000_000,
            status: TxStatus::Pending,
        }
    }

    #[test]
    fn test_submit_and_confirm_transaction() {
        let mut ledger = EnergyLedger::new();
        ledger.register_participant(make_participant(0, ParticipantRole::Producer));
        ledger.register_participant(make_participant(1, ParticipantRole::Consumer));

        let tx = make_tx(0, 1, 10.0, 50.0);
        let id = ledger.submit_transaction(tx).expect("submit ok");
        assert_eq!(id, 0);
        assert_eq!(ledger.pending_transactions.len(), 1);
        assert_eq!(ledger.pending_transactions[0].status, TxStatus::Pending);
    }

    #[test]
    fn test_block_settlement_confirms_transactions() {
        let mut ledger = EnergyLedger::new();
        ledger.register_participant(make_participant(0, ParticipantRole::Producer));
        ledger.register_participant(make_participant(1, ParticipantRole::Consumer));

        ledger
            .submit_transaction(make_tx(0, 1, 5.0, 60.0))
            .expect("ok");
        ledger
            .submit_transaction(make_tx(0, 1, 10.0, 55.0))
            .expect("ok");

        let block = ledger.settle_block(57.5).expect("settle ok");
        assert_eq!(block.transactions.len(), 2);
        for tx in &block.transactions {
            assert_eq!(tx.status, TxStatus::Settled);
        }
        assert!(ledger.pending_transactions.is_empty());

        let seller_bal = ledger.participant_balance(0).expect("seller found");
        // 5*60 + 10*55 = 300 + 550 = 850 USD gained
        assert!(
            (seller_bal - 10_850.0).abs() < 1e-9,
            "seller balance wrong: {seller_bal}"
        );
    }

    #[test]
    fn test_chain_validation_valid_chain() {
        let mut ledger = EnergyLedger::new();
        ledger.register_participant(make_participant(0, ParticipantRole::Producer));
        ledger.register_participant(make_participant(1, ParticipantRole::Consumer));

        for i in 0..3u64 {
            ledger
                .submit_transaction(make_tx(0, 1, (i + 1) as f64, 50.0))
                .expect("ok");
            ledger.settle_block(50.0).expect("settle");
        }
        assert!(ledger.validate_chain(), "Valid chain must pass");
    }

    #[test]
    fn test_chain_tamper_detected() {
        let mut ledger = EnergyLedger::new();
        ledger.register_participant(make_participant(0, ParticipantRole::Producer));
        ledger.register_participant(make_participant(1, ParticipantRole::Consumer));

        ledger
            .submit_transaction(make_tx(0, 1, 5.0, 50.0))
            .expect("ok");
        ledger.settle_block(50.0).expect("settle");
        ledger
            .submit_transaction(make_tx(0, 1, 5.0, 60.0))
            .expect("ok");
        ledger.settle_block(60.0).expect("settle");

        // Tamper block 0 transaction data
        ledger.chain[0].transactions[0].energy_mwh = 999.0;
        assert!(!ledger.validate_chain(), "Tampered chain must fail");
    }

    #[test]
    fn test_block_statistics_min_max_avg() {
        let mut ledger = EnergyLedger::new();
        ledger.register_participant(make_participant(0, ParticipantRole::Producer));
        ledger.register_participant(make_participant(1, ParticipantRole::Consumer));

        // 10 MWh @ $40 and 10 MWh @ $60 → avg = $50
        ledger
            .submit_transaction(make_tx(0, 1, 10.0, 40.0))
            .expect("ok");
        ledger
            .submit_transaction(make_tx(0, 1, 10.0, 60.0))
            .expect("ok");
        ledger.settle_block(50.0).expect("settle");

        let (min_p, max_p, avg_p) = ledger.block_statistics(0).expect("stats");
        assert!((min_p - 40.0).abs() < 1e-9, "min={min_p}");
        assert!((max_p - 60.0).abs() < 1e-9, "max={max_p}");
        assert!(
            (avg_p - 50.0).abs() < 1e-9,
            "avg should be 50.0, got {avg_p}"
        );
    }

    #[test]
    fn test_participant_history_across_blocks() {
        let mut ledger = EnergyLedger::new();
        ledger.register_participant(make_participant(0, ParticipantRole::Producer));
        ledger.register_participant(make_participant(1, ParticipantRole::Consumer));
        ledger.register_participant(make_participant(2, ParticipantRole::Consumer));

        ledger
            .submit_transaction(make_tx(0, 1, 5.0, 50.0))
            .expect("ok");
        ledger.settle_block(50.0).expect("settle");
        ledger
            .submit_transaction(make_tx(0, 2, 8.0, 55.0))
            .expect("ok");
        ledger
            .submit_transaction(make_tx(0, 1, 3.0, 45.0))
            .expect("ok");
        ledger.settle_block(50.0).expect("settle");

        let hist_0 = ledger.participant_history(0);
        assert_eq!(hist_0.len(), 3, "Producer in 3 tx");
        let hist_1 = ledger.participant_history(1);
        assert_eq!(hist_1.len(), 2, "Consumer 1 in 2 tx");
        let hist_2 = ledger.participant_history(2);
        assert_eq!(hist_2.len(), 1, "Consumer 2 in 1 tx");
    }

    #[test]
    fn test_total_traded_mwh() {
        let mut ledger = EnergyLedger::new();
        ledger.register_participant(make_participant(0, ParticipantRole::Producer));
        ledger.register_participant(make_participant(1, ParticipantRole::Consumer));

        ledger
            .submit_transaction(make_tx(0, 1, 5.0, 50.0))
            .expect("ok");
        ledger
            .submit_transaction(make_tx(0, 1, 15.0, 55.0))
            .expect("ok");
        ledger.settle_block(52.0).expect("settle");

        assert!(
            (ledger.total_traded_mwh() - 20.0).abs() < 1e-9,
            "total should be 20 MWh"
        );
    }

    #[test]
    fn test_submit_unknown_participant_errors() {
        let mut ledger = EnergyLedger::new();
        ledger.register_participant(make_participant(0, ParticipantRole::Producer));
        let tx = make_tx(0, 99, 5.0, 50.0); // buyer 99 doesn't exist
        assert!(
            ledger.submit_transaction(tx).is_err(),
            "Unknown buyer must error"
        );
    }

    #[test]
    fn test_invalid_energy_rejected() {
        let mut ledger = EnergyLedger::new();
        ledger.register_participant(make_participant(0, ParticipantRole::Producer));
        ledger.register_participant(make_participant(1, ParticipantRole::Consumer));
        let tx = make_tx(0, 1, -5.0, 50.0); // negative energy
        assert!(ledger.submit_transaction(tx).is_err());
    }
}
