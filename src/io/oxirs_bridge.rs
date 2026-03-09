/// oxirs knowledge graph integration — digital twin interface.
///
/// Provides serialisation of OxiGrid network and simulation data into a
/// structured JSON representation suitable for ingestion by a knowledge graph
/// or digital twin platform (oxirs / RDF / JSON-LD compatible).
///
/// # Design
///
/// Until the oxirs crate is published, this module produces a self-describing
/// JSON envelope that can be consumed by:
///   - oxirs (when available) for RDF triple ingestion
///   - Any JSON-LD consumer for semantic web integration
///   - Digital twin platforms (Azure DT, AWS IoT TwinMaker, etc.)
///
/// # Example
///
/// ```rust,ignore
/// use oxigrid::io::oxirs_bridge::DigitalTwinExport;
/// let dt = network.to_digital_twin("plant-001");
/// let json = dt.to_json_ld();
/// ```
use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};

// ── Schema types ──────────────────────────────────────────────────────────────

/// JSON-LD context prefix.
pub const OXIGRID_CONTEXT: &str = "https://schema.oxigrid.rs/v1/";

/// A digital twin asset representing one entity (bus, branch, generator, etc.)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DtAsset {
    /// Unique identifier (e.g., "bus:14", "branch:0-1")
    pub id: String,
    /// Asset type (e.g., "Bus", "Branch", "Generator")
    #[serde(rename = "type")]
    pub asset_type: String,
    /// Human-readable label
    pub label: String,
    /// Key-value properties
    pub properties: std::collections::BTreeMap<String, serde_json::Value>,
    /// Related asset IDs
    pub relations: Vec<DtRelation>,
}

impl DtAsset {
    pub fn new(id: impl Into<String>, asset_type: impl Into<String>) -> Self {
        let id = id.into();
        let label = id.clone();
        Self {
            id,
            asset_type: asset_type.into(),
            label,
            properties: std::collections::BTreeMap::new(),
            relations: Vec::new(),
        }
    }

    pub fn with_label(mut self, label: impl Into<String>) -> Self {
        self.label = label.into();
        self
    }

    pub fn with_property(mut self, key: impl Into<String>, value: serde_json::Value) -> Self {
        self.properties.insert(key.into(), value);
        self
    }

    pub fn with_relation(mut self, rel: DtRelation) -> Self {
        self.relations.push(rel);
        self
    }
}

/// A directed relation between two digital twin assets.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DtRelation {
    /// Relation type (e.g., "connectedTo", "controlledBy", "locatedIn")
    pub rel_type: String,
    /// Target asset ID
    pub target: String,
}

impl DtRelation {
    pub fn new(rel_type: impl Into<String>, target: impl Into<String>) -> Self {
        Self {
            rel_type: rel_type.into(),
            target: target.into(),
        }
    }
}

/// A complete digital twin model: collection of assets + metadata.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DigitalTwinModel {
    /// Model identifier (e.g., plant name, grid region)
    pub id: String,
    /// Descriptive name
    pub name: String,
    /// Schema version
    pub schema_version: String,
    /// Timestamp of export (ISO 8601)
    pub exported_at: String,
    /// All assets in this model
    pub assets: Vec<DtAsset>,
    /// Global metadata
    pub metadata: std::collections::BTreeMap<String, serde_json::Value>,
}

impl DigitalTwinModel {
    pub fn new(id: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            schema_version: "1.0.0".into(),
            exported_at: "2026-03-07T00:00:00Z".into(), // static for determinism
            assets: Vec::new(),
            metadata: std::collections::BTreeMap::new(),
        }
    }

    pub fn with_asset(mut self, asset: DtAsset) -> Self {
        self.assets.push(asset);
        self
    }

    pub fn add_asset(&mut self, asset: DtAsset) {
        self.assets.push(asset);
    }

    pub fn add_metadata(&mut self, key: impl Into<String>, value: serde_json::Value) {
        self.metadata.insert(key.into(), value);
    }

    /// Serialize to JSON string.
    pub fn to_json(&self) -> Result<String> {
        serde_json::to_string_pretty(self)
            .map_err(|e| OxiGridError::ParseError(format!("DT serialisation error: {e}")))
    }

    /// Serialize to a minimal JSON-LD envelope.
    pub fn to_json_ld(&self) -> Result<String> {
        let ld = serde_json::json!({
            "@context": {
                "@base": OXIGRID_CONTEXT,
                "id": "@id",
                "type": "@type",
                "assets": { "@id": "assets", "@container": "@list" },
            },
            "@id": &self.id,
            "@type": "DigitalTwinModel",
            "name": &self.name,
            "schemaVersion": &self.schema_version,
            "exportedAt": &self.exported_at,
            "assets": self.assets.iter().map(|a| serde_json::json!({
                "@id": &a.id,
                "@type": &a.asset_type,
                "label": &a.label,
                "properties": &a.properties,
                "relations": a.relations.iter().map(|r| serde_json::json!({
                    "relType": &r.rel_type,
                    "target": &r.target,
                })).collect::<Vec<_>>(),
            })).collect::<Vec<_>>(),
            "metadata": &self.metadata,
        });
        serde_json::to_string_pretty(&ld)
            .map_err(|e| OxiGridError::ParseError(format!("JSON-LD serialisation error: {e}")))
    }

    /// Find assets by type.
    pub fn assets_by_type(&self, asset_type: &str) -> Vec<&DtAsset> {
        self.assets
            .iter()
            .filter(|a| a.asset_type == asset_type)
            .collect()
    }

    /// Find an asset by id.
    pub fn asset_by_id(&self, id: &str) -> Option<&DtAsset> {
        self.assets.iter().find(|a| a.id == id)
    }
}

// ── PowerNetwork → DigitalTwin export ────────────────────────────────────────

/// Trait for types that can be exported to a DigitalTwinModel.
pub trait ToDigitalTwin {
    fn to_digital_twin(&self, model_id: &str) -> DigitalTwinModel;
}

#[cfg(feature = "powerflow")]
mod network_impl {
    use super::*;
    use crate::network::bus::BusType;
    use crate::network::topology::PowerNetwork;

    impl ToDigitalTwin for PowerNetwork {
        fn to_digital_twin(&self, model_id: &str) -> DigitalTwinModel {
            let mut model =
                DigitalTwinModel::new(model_id, format!("Power Network ({})", model_id));
            model.add_metadata("base_mva", serde_json::json!(self.base_mva));
            model.add_metadata("bus_count", serde_json::json!(self.bus_count()));
            model.add_metadata("branch_count", serde_json::json!(self.branch_count()));
            model.add_metadata("generator_count", serde_json::json!(self.generators.len()));

            // Export buses
            for bus in &self.buses {
                let bus_type_str = match bus.bus_type {
                    BusType::Slack => "Slack",
                    BusType::PV => "PV",
                    BusType::PQ => "PQ",
                };
                let asset = DtAsset::new(format!("bus:{}", bus.id), "Bus")
                    .with_label(bus.name.clone())
                    .with_property("busType", serde_json::json!(bus_type_str))
                    .with_property("baseKv", serde_json::json!(bus.base_kv.0))
                    .with_property("vm_pu", serde_json::json!(bus.vm))
                    .with_property("va_rad", serde_json::json!(bus.va))
                    .with_property("pd_mw", serde_json::json!(bus.pd.0))
                    .with_property("qd_mvar", serde_json::json!(bus.qd.0));
                model.add_asset(asset);
            }

            // Export branches
            for (k, branch) in self.branches.iter().enumerate() {
                let from_id = format!("bus:{}", branch.from_bus);
                let to_id = format!("bus:{}", branch.to_bus);
                let asset = DtAsset::new(format!("branch:{k}"), "Branch")
                    .with_label(format!(
                        "Branch {k}: {} → {}",
                        branch.from_bus, branch.to_bus
                    ))
                    .with_property("r_pu", serde_json::json!(branch.r))
                    .with_property("x_pu", serde_json::json!(branch.x))
                    .with_property("b_pu", serde_json::json!(branch.b))
                    .with_property("rate_a_mva", serde_json::json!(branch.rate_a))
                    .with_property("tap", serde_json::json!(branch.tap))
                    .with_property("status", serde_json::json!(branch.status))
                    .with_relation(DtRelation::new("fromBus", from_id))
                    .with_relation(DtRelation::new("toBus", to_id));
                model.add_asset(asset);
            }

            // Export generators
            for (k, gen) in self.generators.iter().enumerate() {
                let bus_id = format!("bus:{}", gen.bus_id);
                let asset = DtAsset::new(format!("gen:{k}"), "Generator")
                    .with_label(format!("Generator {k} at bus {}", gen.bus_id))
                    .with_property("pg_mw", serde_json::json!(gen.pg))
                    .with_property("qg_mvar", serde_json::json!(gen.qg))
                    .with_property("pmax_mw", serde_json::json!(gen.pmax))
                    .with_property("pmin_mw", serde_json::json!(gen.pmin))
                    .with_property("qmax_mvar", serde_json::json!(gen.qmax))
                    .with_property("qmin_mvar", serde_json::json!(gen.qmin))
                    .with_property("vg_pu", serde_json::json!(gen.vg))
                    .with_property("status", serde_json::json!(gen.status))
                    .with_relation(DtRelation::new("locatedAt", bus_id));
                model.add_asset(asset);
            }

            model
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_model() -> DigitalTwinModel {
        DigitalTwinModel::new("test-grid", "Test Power Grid")
            .with_asset(
                DtAsset::new("bus:1", "Bus")
                    .with_label("Slack Bus")
                    .with_property("vm_pu", serde_json::json!(1.06))
                    .with_property("busType", serde_json::json!("Slack")),
            )
            .with_asset(
                DtAsset::new("bus:2", "Bus")
                    .with_label("Load Bus")
                    .with_property("pd_mw", serde_json::json!(21.7))
                    .with_relation(DtRelation::new("connectedTo", "bus:1")),
            )
            .with_asset(
                DtAsset::new("branch:0", "Branch")
                    .with_property("r_pu", serde_json::json!(0.01938))
                    .with_relation(DtRelation::new("fromBus", "bus:1"))
                    .with_relation(DtRelation::new("toBus", "bus:2")),
            )
    }

    #[test]
    fn test_model_asset_count() {
        let m = sample_model();
        assert_eq!(m.assets.len(), 3);
    }

    #[test]
    fn test_assets_by_type() {
        let m = sample_model();
        assert_eq!(m.assets_by_type("Bus").len(), 2);
        assert_eq!(m.assets_by_type("Branch").len(), 1);
        assert_eq!(m.assets_by_type("Generator").len(), 0);
    }

    #[test]
    fn test_asset_by_id() {
        let m = sample_model();
        let bus = m.asset_by_id("bus:1");
        assert!(bus.is_some());
        assert_eq!(bus.unwrap().asset_type, "Bus");
    }

    #[test]
    fn test_to_json_roundtrip() {
        let m = sample_model();
        let json = m.to_json().unwrap();
        let parsed: DigitalTwinModel = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.id, "test-grid");
        assert_eq!(parsed.assets.len(), 3);
    }

    #[test]
    fn test_to_json_ld_valid() {
        let m = sample_model();
        let ld = m.to_json_ld().unwrap();
        assert!(ld.contains("@context"), "JSON-LD should contain @context");
        assert!(ld.contains("DigitalTwinModel"), "Should contain type");
        assert!(ld.contains("bus:1"), "Should contain bus id");
    }

    #[test]
    fn test_relations_serialise() {
        let m = sample_model();
        let json = m.to_json().unwrap();
        assert!(
            json.contains("connectedTo"),
            "Relations should be serialised"
        );
    }

    #[test]
    fn test_dt_relation_new() {
        let r = DtRelation::new("controlledBy", "controller:1");
        assert_eq!(r.rel_type, "controlledBy");
        assert_eq!(r.target, "controller:1");
    }

    #[test]
    fn test_metadata() {
        let mut m = DigitalTwinModel::new("g1", "Grid 1");
        m.add_metadata("region", serde_json::json!("EU"));
        assert!(m.metadata.contains_key("region"));
    }

    #[cfg(feature = "powerflow")]
    #[test]
    fn test_power_network_to_digital_twin() {
        use crate::io::oxirs_bridge::ToDigitalTwin;
        use crate::network::PowerNetwork;

        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        let net = PowerNetwork::from_matpower(path).unwrap();
        let dt = net.to_digital_twin("ieee14");

        assert_eq!(dt.id, "ieee14");
        let buses = dt.assets_by_type("Bus");
        assert_eq!(buses.len(), 14, "Should have 14 bus assets");
        let branches = dt.assets_by_type("Branch");
        assert_eq!(branches.len(), 20, "Should have 20 branch assets");
        let gens = dt.assets_by_type("Generator");
        assert_eq!(gens.len(), 5, "Should have 5 generator assets");

        // Verify JSON-LD round-trip
        let ld = dt.to_json_ld().unwrap();
        assert!(!ld.is_empty());
    }
}
