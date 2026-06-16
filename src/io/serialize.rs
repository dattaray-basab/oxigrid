/// Serialization helpers for OxiGrid types.
///
/// Provides convenient wrappers for JSON serialization/deserialization of
/// the core network, power flow, and battery types using serde_json.
use crate::error::{OxiGridError, Result};

/// Serialize any serde-serializable value to a JSON string.
pub fn to_json<T: serde::Serialize>(value: &T) -> Result<String> {
    serde_json::to_string(value)
        .map_err(|e| OxiGridError::ParseError(format!("JSON serialization error: {e}")))
}

/// Serialize to a pretty-printed JSON string.
pub fn to_json_pretty<T: serde::Serialize>(value: &T) -> Result<String> {
    serde_json::to_string_pretty(value)
        .map_err(|e| OxiGridError::ParseError(format!("JSON serialization error: {e}")))
}

/// Deserialize from a JSON string.
pub fn from_json<T: serde::de::DeserializeOwned>(json: &str) -> Result<T> {
    serde_json::from_str(json)
        .map_err(|e| OxiGridError::ParseError(format!("JSON parse error: {e}")))
}

/// Write a serializable value to a JSON file.
pub fn write_json_file<T: serde::Serialize>(path: &str, value: &T) -> Result<()> {
    let json = to_json_pretty(value)?;
    std::fs::write(path, json)
        .map_err(|e| OxiGridError::ParseError(format!("Failed to write {path}: {e}")))
}

/// Read and deserialize a JSON file.
pub fn read_json_file<T: serde::de::DeserializeOwned>(path: &str) -> Result<T> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| OxiGridError::ParseError(format!("Failed to read {path}: {e}")))?;
    from_json(&content)
}

/// Serialize a value to compact JSON bytes.
pub fn to_json_bytes<T: serde::Serialize>(value: &T) -> Result<Vec<u8>> {
    serde_json::to_vec(value)
        .map_err(|e| OxiGridError::ParseError(format!("JSON serialization error: {e}")))
}

/// Deserialize from JSON bytes.
pub fn from_json_bytes<T: serde::de::DeserializeOwned>(bytes: &[u8]) -> Result<T> {
    serde_json::from_slice(bytes)
        .map_err(|e| OxiGridError::ParseError(format!("JSON parse error: {e}")))
}

/// A generic serialization envelope with version metadata.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SerializedEnvelope<T> {
    /// Schema version
    pub version: String,
    /// Type tag for identification
    pub type_tag: String,
    /// The serialized data
    pub data: T,
}

impl<T: serde::Serialize + serde::de::DeserializeOwned> SerializedEnvelope<T> {
    /// Wrap a value in an envelope.
    pub fn wrap(type_tag: &str, data: T) -> Self {
        Self {
            version: "1.0".to_string(),
            type_tag: type_tag.to_string(),
            data,
        }
    }

    /// Serialize the envelope to JSON.
    pub fn to_json_string(&self) -> Result<String> {
        to_json_pretty(self)
    }

    /// Unwrap the data.
    pub fn unwrap(self) -> T {
        self.data
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    #[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
    struct TestStruct {
        x: f64,
        name: String,
        values: Vec<f64>,
    }

    fn sample() -> TestStruct {
        TestStruct {
            x: std::f64::consts::PI,
            name: "test".into(),
            values: vec![1.0, 2.0, 3.0],
        }
    }

    #[test]
    fn test_to_json_roundtrip() {
        let orig = sample();
        let json = to_json(&orig).unwrap();
        let parsed: TestStruct = from_json(&json).unwrap();
        assert_eq!(orig, parsed);
    }

    #[test]
    fn test_to_json_pretty_valid() {
        let orig = sample();
        let json = to_json_pretty(&orig).unwrap();
        assert!(json.contains('\n'), "Pretty JSON should have newlines");
        let parsed: TestStruct = from_json(&json).unwrap();
        assert_eq!(orig, parsed);
    }

    #[test]
    fn test_to_json_bytes_roundtrip() {
        let orig = sample();
        let bytes = to_json_bytes(&orig).unwrap();
        let parsed: TestStruct = from_json_bytes(&bytes).unwrap();
        assert_eq!(orig, parsed);
    }

    #[test]
    fn test_envelope_roundtrip() {
        let orig = sample();
        let env = SerializedEnvelope::wrap("TestStruct", orig.clone());
        let json = env.to_json_string().unwrap();
        let parsed_env: SerializedEnvelope<TestStruct> = from_json(&json).unwrap();
        assert_eq!(parsed_env.type_tag, "TestStruct");
        assert_eq!(parsed_env.version, "1.0");
        assert_eq!(parsed_env.unwrap(), orig);
    }

    #[test]
    fn test_from_json_invalid_returns_err() {
        let result: std::result::Result<TestStruct, _> = from_json("{invalid json}");
        assert!(result.is_err());
    }

    #[test]
    fn test_from_json_bytes_invalid_returns_err() {
        let result: std::result::Result<TestStruct, _> = from_json_bytes(b"not json");
        assert!(result.is_err());
    }

    #[test]
    fn test_serialize_empty_network() {
        use crate::network::topology::PowerNetwork;
        let net = PowerNetwork::new(100.0);
        let json = to_json(&net).expect("serialize empty PowerNetwork");
        assert!(
            !json.is_empty(),
            "JSON must be non-empty for an empty network"
        );
    }

    #[test]
    fn test_roundtrip_power_network() {
        use crate::network::bus::{Bus, BusType};
        use crate::network::topology::PowerNetwork;
        let mut net = PowerNetwork::new(100.0);
        let mut b = Bus::new(1, BusType::Slack);
        b.vm = 1.0;
        net.buses.push(b);
        let json = to_json(&net).expect("serialize PowerNetwork with one bus");
        let recovered: PowerNetwork = from_json(&json).expect("deserialize PowerNetwork");
        assert_eq!(
            recovered.buses.len(),
            net.buses.len(),
            "bus count must survive round-trip"
        );
    }

    #[test]
    fn test_json_output_contains_field_names() {
        let orig = sample();
        let json = to_json_pretty(&orig).expect("pretty-print TestStruct");
        assert!(json.contains("\"x\""), "JSON should contain field 'x'");
        assert!(
            json.contains("\"name\""),
            "JSON should contain field 'name'"
        );
        assert!(
            json.contains("\"values\""),
            "JSON should contain field 'values'"
        );
    }

    #[test]
    fn test_write_read_json_file_roundtrip() {
        let orig = sample();
        let mut path = std::env::temp_dir();
        path.push("oxigrid_serialize_test_roundtrip.json");
        let path_str = path.to_str().expect("temp path is valid UTF-8");
        write_json_file(path_str, &orig).expect("write JSON file");
        let recovered: TestStruct = read_json_file(path_str).expect("read JSON file");
        assert_eq!(orig, recovered);
        let _ = std::fs::remove_file(path_str);
    }

    #[test]
    fn test_read_nonexistent_file_returns_err() {
        let result: std::result::Result<TestStruct, _> =
            read_json_file("/tmp/oxigrid_nonexistent_xyz_12345.json");
        assert!(result.is_err(), "reading a missing file must return Err");
    }

    #[test]
    fn test_envelope_version_field() {
        let orig = sample();
        let env = SerializedEnvelope::wrap("TestStruct", orig);
        let json = env.to_json_string().expect("serialize envelope");
        assert!(
            json.contains("\"version\""),
            "JSON must contain 'version' key"
        );
        assert!(
            json.contains("\"1.0\""),
            "JSON must contain version value '1.0'"
        );
    }

    #[test]
    fn test_serialize_nested_envelope() {
        let orig = sample();
        let inner_env = SerializedEnvelope::wrap("TestStruct", orig);
        let inner_json = inner_env
            .to_json_string()
            .expect("serialize inner envelope");
        let outer_env = SerializedEnvelope::wrap("String", inner_json);
        let outer_json = outer_env
            .to_json_string()
            .expect("serialize outer envelope");
        assert!(
            !outer_json.is_empty(),
            "nested envelope JSON must be non-empty"
        );
    }

    #[test]
    fn test_empty_values_roundtrip() {
        let orig = TestStruct {
            x: 0.0,
            name: "".into(),
            values: vec![],
        };
        let json = to_json(&orig).expect("serialize TestStruct with empty fields");
        let recovered: TestStruct =
            from_json(&json).expect("deserialize TestStruct with empty fields");
        assert_eq!(orig, recovered);
        assert!(
            recovered.values.is_empty(),
            "values must remain empty after round-trip"
        );
    }
}
