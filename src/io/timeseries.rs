/// Time-series data store trait and in-memory implementation.
///
/// Provides a generic `TimeSeriesStore` trait for reading and writing
/// time-indexed measurement data, with:
/// - An in-memory `MemoryStore` implementation for testing and small datasets
/// - A `CsvStore` implementation backed by the existing CSV I/O
/// - Query utilities: windowed slice, resampling, interpolation
/// - Metadata: channel name, unit, sample interval
///
/// # Example
///
/// ```rust
/// use oxigrid::io::timeseries::{TimeSeriesStore, MemoryStore, Channel};
///
/// let mut store = MemoryStore::new();
/// let ch = Channel::new("voltage_kv", "kV", 0.25);
/// store.create_channel(ch);
/// store.append("voltage_kv", 0.0, 132.5).unwrap();
/// store.append("voltage_kv", 0.25, 131.8).unwrap();
/// let data = store.range("voltage_kv", 0.0, 1.0).unwrap();
/// ```
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ────────────────────────────────────────────────────────────────────────────
// Channel metadata
// ────────────────────────────────────────────────────────────────────────────

/// Metadata for one time-series channel.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Channel {
    /// Unique channel name (e.g. "bus_1_voltage_kv")
    pub name: String,
    /// Physical unit string (e.g. "kV", "MW", "A")
    pub unit: String,
    /// Nominal sample interval `s` (0 = irregular)
    pub dt_s: f64,
    /// Description
    pub description: String,
    /// Data quality flag definitions
    pub quality_flags: Vec<String>,
}

impl Channel {
    pub fn new(name: &str, unit: &str, dt_s: f64) -> Self {
        Self {
            name: name.to_string(),
            unit: unit.to_string(),
            dt_s,
            description: String::new(),
            quality_flags: vec![],
        }
    }

    pub fn with_description(mut self, desc: &str) -> Self {
        self.description = desc.to_string();
        self
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Sample
// ────────────────────────────────────────────────────────────────────────────

/// One time-series sample.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Sample {
    /// Timestamp [s from epoch or from start]
    pub timestamp: f64,
    /// Value
    pub value: f64,
    /// Quality flag (0 = good, non-zero = quality issue)
    pub quality: u8,
}

impl Sample {
    pub fn good(timestamp: f64, value: f64) -> Self {
        Self {
            timestamp,
            value,
            quality: 0,
        }
    }

    pub fn with_quality(timestamp: f64, value: f64, quality: u8) -> Self {
        Self {
            timestamp,
            value,
            quality,
        }
    }

    pub fn is_good(&self) -> bool {
        self.quality == 0
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Trait
// ────────────────────────────────────────────────────────────────────────────

/// Errors from time-series store operations.
#[derive(Debug, Clone, PartialEq)]
pub enum TsError {
    ChannelNotFound(String),
    InvalidTimeRange,
    WriteError(String),
    EmptyChannel,
}

impl std::fmt::Display for TsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TsError::ChannelNotFound(n) => write!(f, "Channel not found: {n}"),
            TsError::InvalidTimeRange => write!(f, "Invalid time range"),
            TsError::WriteError(msg) => write!(f, "Write error: {msg}"),
            TsError::EmptyChannel => write!(f, "Channel has no data"),
        }
    }
}

/// Generic time-series store trait.
pub trait TimeSeriesStore {
    /// Create or register a channel.
    fn create_channel(&mut self, channel: Channel);

    /// Return channel metadata.
    fn channel_info(&self, name: &str) -> Option<&Channel>;

    /// List all channel names.
    fn channel_names(&self) -> Vec<String>;

    /// Append a single sample.
    fn append(&mut self, channel: &str, timestamp: f64, value: f64) -> Result<(), TsError>;

    /// Append a sample with quality flag.
    fn append_with_quality(
        &mut self,
        channel: &str,
        timestamp: f64,
        value: f64,
        quality: u8,
    ) -> Result<(), TsError>;

    /// Retrieve all samples in [t_start, t_end].
    fn range(&self, channel: &str, t_start: f64, t_end: f64) -> Result<Vec<Sample>, TsError>;

    /// Retrieve the latest N samples.
    fn latest(&self, channel: &str, n: usize) -> Result<Vec<Sample>, TsError>;

    /// Count of samples in a channel.
    fn len(&self, channel: &str) -> Result<usize, TsError>;

    /// True if the channel has no samples.
    fn is_empty(&self, channel: &str) -> Result<bool, TsError> {
        Ok(self.len(channel)? == 0)
    }

    /// Statistics over a time range.
    fn stats(&self, channel: &str, t_start: f64, t_end: f64) -> Result<TsStats, TsError> {
        let samples = self.range(channel, t_start, t_end)?;
        Ok(TsStats::from_samples(&samples))
    }
}

/// Summary statistics for a time window.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TsStats {
    pub n: usize,
    pub n_good: usize,
    pub mean: f64,
    pub min: f64,
    pub max: f64,
    pub std: f64,
    pub p50: f64,
    pub p95: f64,
}

impl TsStats {
    pub fn from_samples(samples: &[Sample]) -> Self {
        let good: Vec<f64> = samples
            .iter()
            .filter(|s| s.is_good())
            .map(|s| s.value)
            .collect();
        let n = samples.len();
        let n_good = good.len();
        if good.is_empty() {
            return Self {
                n,
                n_good: 0,
                mean: 0.0,
                min: 0.0,
                max: 0.0,
                std: 0.0,
                p50: 0.0,
                p95: 0.0,
            };
        }
        let mean = good.iter().sum::<f64>() / n_good as f64;
        let var = good.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n_good as f64;
        let min = good.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = good.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let mut sorted = good.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p50 = sorted[n_good / 2];
        let p95 = sorted[((0.95 * n_good as f64) as usize).min(n_good - 1)];

        Self {
            n,
            n_good,
            mean,
            min,
            max,
            std: var.sqrt(),
            p50,
            p95,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// In-memory store
// ────────────────────────────────────────────────────────────────────────────

/// In-memory time-series store (for testing and real-time buffers).
#[derive(Debug, Clone, Default)]
pub struct MemoryStore {
    channels: HashMap<String, Channel>,
    data: HashMap<String, Vec<Sample>>,
}

impl MemoryStore {
    pub fn new() -> Self {
        Self {
            channels: HashMap::new(),
            data: HashMap::new(),
        }
    }

    /// Bulk-insert a pre-built sample vector.
    pub fn bulk_insert(&mut self, channel: &str, samples: Vec<Sample>) -> Result<(), TsError> {
        let data = self
            .data
            .get_mut(channel)
            .ok_or_else(|| TsError::ChannelNotFound(channel.to_string()))?;
        data.extend(samples);
        Ok(())
    }

    /// Value at a specific timestamp (exact match, no interpolation).
    pub fn at(&self, channel: &str, timestamp: f64) -> Result<Option<Sample>, TsError> {
        let data = self
            .data
            .get(channel)
            .ok_or_else(|| TsError::ChannelNotFound(channel.to_string()))?;
        Ok(data
            .iter()
            .find(|s| (s.timestamp - timestamp).abs() < 1e-9)
            .copied())
    }

    /// Linear interpolation at a specific timestamp.
    pub fn interpolate(&self, channel: &str, timestamp: f64) -> Result<f64, TsError> {
        let data = self
            .data
            .get(channel)
            .ok_or_else(|| TsError::ChannelNotFound(channel.to_string()))?;
        if data.is_empty() {
            return Err(TsError::EmptyChannel);
        }

        // Find surrounding samples
        let pos = data.partition_point(|s| s.timestamp <= timestamp);
        if pos == 0 {
            return Ok(data[0].value);
        }
        if pos >= data.len() {
            return Ok(data[data.len() - 1].value);
        }

        let s0 = &data[pos - 1];
        let s1 = &data[pos];
        let dt = s1.timestamp - s0.timestamp;
        if dt.abs() < 1e-12 {
            return Ok(s0.value);
        }
        let alpha = (timestamp - s0.timestamp) / dt;
        Ok(s0.value + alpha * (s1.value - s0.value))
    }

    /// Downsample by taking mean over windows of width `window_s`.
    pub fn resample_mean(
        &self,
        channel: &str,
        t_start: f64,
        t_end: f64,
        window_s: f64,
    ) -> Result<Vec<Sample>, TsError> {
        let raw = self.range(channel, t_start, t_end)?;
        if raw.is_empty() {
            return Ok(vec![]);
        }

        let mut result = Vec::new();
        let mut t = t_start;
        while t < t_end {
            let t_next = (t + window_s).min(t_end);
            let window: Vec<f64> = raw
                .iter()
                .filter(|s| s.timestamp >= t && s.timestamp < t_next && s.is_good())
                .map(|s| s.value)
                .collect();
            if !window.is_empty() {
                let mean = window.iter().sum::<f64>() / window.len() as f64;
                result.push(Sample::good(t + window_s / 2.0, mean));
            }
            t = t_next;
        }
        Ok(result)
    }

    /// Flag samples outside [lo, hi] with given quality code.
    pub fn apply_range_check(
        &mut self,
        channel: &str,
        lo: f64,
        hi: f64,
        flag: u8,
    ) -> Result<usize, TsError> {
        let data = self
            .data
            .get_mut(channel)
            .ok_or_else(|| TsError::ChannelNotFound(channel.to_string()))?;
        let mut flagged = 0;
        for s in data.iter_mut() {
            if s.value < lo || s.value > hi {
                s.quality = flag;
                flagged += 1;
            }
        }
        Ok(flagged)
    }

    /// Export channel to (timestamp, value) CSV-format string.
    pub fn to_csv(&self, channel: &str) -> Result<String, TsError> {
        let data = self
            .data
            .get(channel)
            .ok_or_else(|| TsError::ChannelNotFound(channel.to_string()))?;
        let mut out = String::from("timestamp,value,quality\n");
        for s in data {
            out.push_str(&format!(
                "{:.6},{:.6},{}\n",
                s.timestamp, s.value, s.quality
            ));
        }
        Ok(out)
    }
}

impl TimeSeriesStore for MemoryStore {
    fn create_channel(&mut self, channel: Channel) {
        let name = channel.name.clone();
        self.channels.insert(name.clone(), channel);
        self.data.entry(name).or_default();
    }

    fn channel_info(&self, name: &str) -> Option<&Channel> {
        self.channels.get(name)
    }

    fn channel_names(&self) -> Vec<String> {
        self.channels.keys().cloned().collect()
    }

    fn append(&mut self, channel: &str, timestamp: f64, value: f64) -> Result<(), TsError> {
        let data = self
            .data
            .get_mut(channel)
            .ok_or_else(|| TsError::ChannelNotFound(channel.to_string()))?;
        data.push(Sample::good(timestamp, value));
        Ok(())
    }

    fn append_with_quality(
        &mut self,
        channel: &str,
        timestamp: f64,
        value: f64,
        quality: u8,
    ) -> Result<(), TsError> {
        let data = self
            .data
            .get_mut(channel)
            .ok_or_else(|| TsError::ChannelNotFound(channel.to_string()))?;
        data.push(Sample::with_quality(timestamp, value, quality));
        Ok(())
    }

    fn range(&self, channel: &str, t_start: f64, t_end: f64) -> Result<Vec<Sample>, TsError> {
        if t_start > t_end {
            return Err(TsError::InvalidTimeRange);
        }
        let data = self
            .data
            .get(channel)
            .ok_or_else(|| TsError::ChannelNotFound(channel.to_string()))?;
        Ok(data
            .iter()
            .filter(|s| s.timestamp >= t_start && s.timestamp <= t_end)
            .copied()
            .collect())
    }

    fn latest(&self, channel: &str, n: usize) -> Result<Vec<Sample>, TsError> {
        let data = self
            .data
            .get(channel)
            .ok_or_else(|| TsError::ChannelNotFound(channel.to_string()))?;
        let start = data.len().saturating_sub(n);
        Ok(data[start..].to_vec())
    }

    fn len(&self, channel: &str) -> Result<usize, TsError> {
        let data = self
            .data
            .get(channel)
            .ok_or_else(|| TsError::ChannelNotFound(channel.to_string()))?;
        Ok(data.len())
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Multi-channel dataset
// ────────────────────────────────────────────────────────────────────────────

/// A named, multi-channel dataset (collection of MemoryStores).
#[derive(Debug, Clone, Default)]
pub struct TimeSeriesDataset {
    pub name: String,
    store: MemoryStore,
}

impl TimeSeriesDataset {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            store: MemoryStore::new(),
        }
    }

    /// Add a channel with uniformly sampled data.
    pub fn add_uniform(&mut self, name: &str, unit: &str, t_start: f64, dt: f64, values: &[f64]) {
        let ch = Channel::new(name, unit, dt);
        self.store.create_channel(ch);
        for (i, &v) in values.iter().enumerate() {
            let _ = self.store.append(name, t_start + i as f64 * dt, v);
        }
    }

    /// Delegate to the underlying store.
    pub fn store(&self) -> &MemoryStore {
        &self.store
    }
    pub fn store_mut(&mut self) -> &mut MemoryStore {
        &mut self.store
    }

    /// Correlation coefficient between two channels over [t_start, t_end].
    pub fn correlation(
        &self,
        ch1: &str,
        ch2: &str,
        t_start: f64,
        t_end: f64,
    ) -> Result<f64, TsError> {
        let s1 = self.store.range(ch1, t_start, t_end)?;
        let s2 = self.store.range(ch2, t_start, t_end)?;

        let n = s1.len().min(s2.len());
        if n < 2 {
            return Ok(0.0);
        }

        let x: Vec<f64> = s1.iter().take(n).map(|s| s.value).collect();
        let y: Vec<f64> = s2.iter().take(n).map(|s| s.value).collect();

        let mx = x.iter().sum::<f64>() / n as f64;
        let my = y.iter().sum::<f64>() / n as f64;

        let cov: f64 = x
            .iter()
            .zip(y.iter())
            .map(|(&xi, &yi)| (xi - mx) * (yi - my))
            .sum::<f64>()
            / n as f64;
        let sx = x.iter().map(|&xi| (xi - mx).powi(2)).sum::<f64>().sqrt() / (n as f64).sqrt();
        let sy = y.iter().map(|&yi| (yi - my).powi(2)).sum::<f64>().sqrt() / (n as f64).sqrt();

        if sx < 1e-12 || sy < 1e-12 {
            return Ok(0.0);
        }
        Ok((cov / (sx * sy)).clamp(-1.0, 1.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_store_with_data() -> MemoryStore {
        let mut store = MemoryStore::new();
        store.create_channel(Channel::new("voltage", "kV", 1.0));
        for i in 0..100 {
            store
                .append("voltage", i as f64, 130.0 + i as f64 * 0.1)
                .unwrap();
        }
        store
    }

    #[test]
    fn test_create_and_append() {
        let mut store = MemoryStore::new();
        store.create_channel(Channel::new("power_mw", "MW", 0.25));
        store.append("power_mw", 0.0, 50.0).unwrap();
        store.append("power_mw", 0.25, 55.0).unwrap();
        assert_eq!(store.len("power_mw").unwrap(), 2);
    }

    #[test]
    fn test_channel_not_found() {
        let mut store = MemoryStore::new();
        let err = store.append("nonexistent", 0.0, 1.0).unwrap_err();
        assert_eq!(err, TsError::ChannelNotFound("nonexistent".to_string()));
    }

    #[test]
    fn test_range_query() {
        let store = make_store_with_data();
        let data = store.range("voltage", 10.0, 20.0).unwrap();
        assert_eq!(data.len(), 11); // inclusive: 10..=20
    }

    #[test]
    fn test_range_invalid() {
        let store = make_store_with_data();
        assert_eq!(
            store.range("voltage", 10.0, 5.0).unwrap_err(),
            TsError::InvalidTimeRange
        );
    }

    #[test]
    fn test_latest_n() {
        let store = make_store_with_data();
        let latest = store.latest("voltage", 5).unwrap();
        assert_eq!(latest.len(), 5);
        assert!((latest.last().unwrap().timestamp - 99.0).abs() < 1e-9);
    }

    #[test]
    fn test_interpolate_midpoint() {
        let mut store = MemoryStore::new();
        store.create_channel(Channel::new("temp", "°C", 1.0));
        store.append("temp", 0.0, 100.0).unwrap();
        store.append("temp", 1.0, 200.0).unwrap();
        let v = store.interpolate("temp", 0.5).unwrap();
        assert!(
            (v - 150.0).abs() < 1e-9,
            "Interpolation should give midpoint: {:.2}",
            v
        );
    }

    #[test]
    fn test_interpolate_at_endpoint() {
        let mut store = MemoryStore::new();
        store.create_channel(Channel::new("v", "V", 1.0));
        store.append("v", 0.0, 10.0).unwrap();
        store.append("v", 1.0, 20.0).unwrap();
        assert!((store.interpolate("v", 0.0).unwrap() - 10.0).abs() < 1e-9);
        assert!((store.interpolate("v", 1.0).unwrap() - 20.0).abs() < 1e-9);
    }

    #[test]
    fn test_stats_basic() {
        let store = make_store_with_data();
        let stats = store.stats("voltage", 0.0, 99.0).unwrap();
        assert!(stats.n > 0);
        assert!(stats.min < stats.max);
        assert!(stats.mean > stats.min && stats.mean < stats.max);
    }

    #[test]
    fn test_stats_std_positive() {
        let store = make_store_with_data();
        let stats = store.stats("voltage", 0.0, 99.0).unwrap();
        assert!(stats.std > 0.0, "Std should be positive for varying data");
    }

    #[test]
    fn test_quality_flag_append() {
        let mut store = MemoryStore::new();
        store.create_channel(Channel::new("freq", "Hz", 0.1));
        store.append_with_quality("freq", 0.0, 50.0, 0).unwrap();
        store.append_with_quality("freq", 0.1, 999.0, 1).unwrap(); // bad sample
        let data = store.range("freq", 0.0, 1.0).unwrap();
        assert_eq!(data.len(), 2);
        assert!(data[0].is_good());
        assert!(!data[1].is_good());
    }

    #[test]
    fn test_range_check_flags_outliers() {
        let mut store = make_store_with_data();
        // Data is 130.0 + i*0.1 for i in 0..100 → values in [130.0, 139.9]
        // Narrow range [132.0, 137.0] → i<20 and i>70 are out of range
        let flagged = store.apply_range_check("voltage", 132.0, 137.0, 2).unwrap();
        assert!(flagged > 0, "Some samples should be flagged out of range");
    }

    #[test]
    fn test_resample_mean() {
        let store = make_store_with_data();
        let resampled = store.resample_mean("voltage", 0.0, 50.0, 10.0).unwrap();
        assert!(!resampled.is_empty());
        assert!(resampled.len() <= 6);
    }

    #[test]
    fn test_to_csv_format() {
        let mut store = MemoryStore::new();
        store.create_channel(Channel::new("p", "MW", 1.0));
        store.append("p", 0.0, 42.5).unwrap();
        let csv = store.to_csv("p").unwrap();
        assert!(csv.contains("timestamp,value,quality"));
        assert!(csv.contains("42.500000"));
    }

    #[test]
    fn test_channel_names_listed() {
        let mut store = MemoryStore::new();
        store.create_channel(Channel::new("a", "V", 1.0));
        store.create_channel(Channel::new("b", "A", 1.0));
        let names = store.channel_names();
        assert!(names.contains(&"a".to_string()));
        assert!(names.contains(&"b".to_string()));
    }

    #[test]
    fn test_dataset_correlation() {
        let mut ds = TimeSeriesDataset::new("test");
        let x: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|&v| v * 2.0 + 1.0).collect();
        ds.add_uniform("x", "MW", 0.0, 1.0, &x);
        ds.add_uniform("y", "MW", 0.0, 1.0, &y);
        let corr = ds.correlation("x", "y", 0.0, 99.0).unwrap();
        assert!(
            (corr - 1.0).abs() < 1e-6,
            "Perfect correlation: {:.6}",
            corr
        );
    }

    #[test]
    fn test_dataset_add_uniform_len() {
        let mut ds = TimeSeriesDataset::new("grid");
        let values = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        ds.add_uniform("load", "MW", 0.0, 0.25, &values);
        let n = ds.store().len("load").unwrap();
        assert_eq!(n, 5);
    }

    #[test]
    fn test_is_empty_after_create() {
        let mut store = MemoryStore::new();
        store.create_channel(Channel::new("empty", "V", 1.0));
        assert!(store.is_empty("empty").unwrap());
    }

    #[test]
    fn test_bulk_insert() {
        let mut store = MemoryStore::new();
        store.create_channel(Channel::new("bulk", "A", 0.1));
        let samples: Vec<Sample> = (0..50)
            .map(|i| Sample::good(i as f64 * 0.1, i as f64))
            .collect();
        store.bulk_insert("bulk", samples).unwrap();
        assert_eq!(store.len("bulk").unwrap(), 50);
    }
}
