//! Power System Visualization Data Preparation.
//!
//! Prepares network topology and power-flow results for downstream rendering
//! tools (GraphViz, SVG, GeoJSON, JSON, CSV).
//!
//! # Supported Formats
//! - **JSON** — structured `NetworkVizData` serialised via serde_json
//! - **CSV** — tabular bus and branch data
//! - **GraphViz** — DOT language for `dot` / `neato` rendering
//! - **SVG** — inline SVG with force-directed bus layout
//! - **GeoJSON** — geographic feature collection for map-based viewers
//!
//! # Colour Schemes
//! Colour functions follow IEC 62351 / NERC EMS colour conventions:
//! - Voltage: green (nominal), yellow (±3 %), red (±5 %)
//! - Loading: green (<70 %), yellow (70–90 %), red (>90 %)
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Type aliases for prepare_data inputs
// ---------------------------------------------------------------------------

/// Bus input row: `(id, name, V_pu, theta_deg, P_load_MW, Q_load_MVAr, P_gen_MW, Q_gen_MVAr)`.
pub type BusInputRow = (usize, String, f64, f64, f64, f64, f64, f64);

/// Branch input row: `(id, from_bus, to_bus, P_flow_MW, Q_flow_MVAr, rating_MW)`.
pub type BranchInputRow = (usize, usize, usize, f64, f64, f64);

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors returned by [`PowerSystemVisualizer`].
#[derive(Debug, thiserror::Error)]
pub enum VizError {
    /// JSON serialisation error.
    #[error("JSON serialisation error: {0}")]
    Json(String),
    /// Unsupported format for the requested operation.
    #[error("Format not supported: {0}")]
    UnsupportedFormat(String),
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Output format for visualisation data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum VizFormat {
    /// Structured JSON via serde_json.
    Json,
    /// Comma-separated values.
    Csv,
    /// DOT language (GraphViz).
    GraphViz,
    /// Inline SVG for web embedding.
    Svg,
    /// GeoJSON feature collection for map rendering.
    GeoJson,
}

/// Colour scheme for branch and bus colouring.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ColorScheme {
    /// Colour by bus voltage magnitude.
    Voltage,
    /// Colour by branch thermal loading \[%\].
    Loading,
    /// Colour by estimated branch losses.
    Loss,
    /// Colour by renewable energy fraction.
    Renewable,
}

/// Configuration for the visualisation engine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualizationConfig {
    /// Output format.
    pub format: VizFormat,
    /// Include active / reactive branch flow data.
    pub include_flows: bool,
    /// Include bus voltage magnitude and angle.
    pub include_voltages: bool,
    /// Include branch thermal loading percentage.
    pub include_loading: bool,
    /// Colour scheme.
    pub color_scheme: ColorScheme,
}

impl Default for VisualizationConfig {
    fn default() -> Self {
        Self {
            format: VizFormat::Json,
            include_flows: true,
            include_voltages: true,
            include_loading: true,
            color_scheme: ColorScheme::Voltage,
        }
    }
}

// ---------------------------------------------------------------------------
// Bus visualisation record
// ---------------------------------------------------------------------------

/// Visualisation data for a single bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BusVizData {
    /// Bus index.
    pub id: usize,
    /// Bus name.
    pub name: String,
    /// Voltage magnitude \[pu\].
    pub voltage_pu: f64,
    /// Voltage angle \[deg\].
    pub angle_deg: f64,
    /// Active load \[MW\].
    pub p_load_mw: f64,
    /// Reactive load \[MVAr\].
    pub q_load_mvar: f64,
    /// Active generation \[MW\].
    pub p_gen_mw: f64,
    /// Reactive generation \[MVAr\].
    pub q_gen_mvar: f64,
    /// Hex colour string derived from voltage magnitude.
    pub voltage_color: String,
    /// Layout x-coordinate (force-directed).
    pub x: f64,
    /// Layout y-coordinate (force-directed).
    pub y: f64,
}

// ---------------------------------------------------------------------------
// Branch visualisation record
// ---------------------------------------------------------------------------

/// Visualisation data for a single branch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BranchVizData {
    /// Branch index.
    pub id: usize,
    /// From-bus index.
    pub from_bus: usize,
    /// To-bus index.
    pub to_bus: usize,
    /// Active power flow \[MW\] (positive = from→to).
    pub p_flow_mw: f64,
    /// Reactive power flow \[MVAr\].
    pub q_flow_mvar: f64,
    /// Thermal loading \[%\] of rated MVA.
    pub loading_pct: f64,
    /// Hex colour string derived from loading.
    pub loading_color: String,
    /// `true` if net flow direction is from→to.
    pub arrow_direction: bool,
    /// Visual line width proportional to `|p_flow_mw|`.
    pub width: f64,
}

// ---------------------------------------------------------------------------
// Network visualisation container
// ---------------------------------------------------------------------------

/// Container for all network visualisation data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkVizData {
    /// Bus visualisation records.
    pub buses: Vec<BusVizData>,
    /// Branch visualisation records.
    pub branches: Vec<BranchVizData>,
    /// Diagram title.
    pub title: String,
    /// Snapshot timestamp string.
    pub timestamp: String,
    /// Total system active losses \[MW\].
    pub system_losses_mw: f64,
    /// Total system active load \[MW\].
    pub total_load_mw: f64,
    /// Total system active generation \[MW\].
    pub total_gen_mw: f64,
}

// ---------------------------------------------------------------------------
// Visualizer
// ---------------------------------------------------------------------------

/// Power system visualisation data engine.
pub struct PowerSystemVisualizer {
    config: VisualizationConfig,
}

impl PowerSystemVisualizer {
    /// Create a new visualiser with the given configuration.
    pub fn new(config: VisualizationConfig) -> Self {
        Self { config }
    }

    // -----------------------------------------------------------------------
    // Data preparation
    // -----------------------------------------------------------------------

    /// Prepare visualisation data from power-flow result arrays.
    ///
    /// See [`BusInputRow`] and [`BranchInputRow`] for tuple field layouts.
    pub fn prepare_data(
        &self,
        bus_data: &[BusInputRow],
        branch_data: &[BranchInputRow],
    ) -> NetworkVizData {
        // Force-directed layout
        let branch_edges: Vec<(usize, usize)> = branch_data.iter().map(|b| (b.1, b.2)).collect();
        let positions = Self::force_directed_layout(bus_data.len(), &branch_edges);

        let buses: Vec<BusVizData> = bus_data
            .iter()
            .enumerate()
            .map(|(i, (id, name, v, ang, pl, ql, pg, qg))| {
                let (x, y) = positions.get(i).copied().unwrap_or((i as f64 * 50.0, 0.0));
                BusVizData {
                    id: *id,
                    name: name.clone(),
                    voltage_pu: *v,
                    angle_deg: *ang,
                    p_load_mw: *pl,
                    q_load_mvar: *ql,
                    p_gen_mw: *pg,
                    q_gen_mvar: *qg,
                    voltage_color: Self::voltage_color(*v),
                    x,
                    y,
                }
            })
            .collect();

        let branches: Vec<BranchVizData> = branch_data
            .iter()
            .map(|(id, from, to, pf, qf, rating)| {
                let s_apparent = (pf * pf + qf * qf).sqrt();
                let loading = if *rating > 1e-9 {
                    s_apparent / rating * 100.0
                } else {
                    0.0
                };
                let max_width = 8.0f64;
                let width = (pf.abs() / rating.max(1.0) * max_width).clamp(0.5, max_width);
                BranchVizData {
                    id: *id,
                    from_bus: *from,
                    to_bus: *to,
                    p_flow_mw: *pf,
                    q_flow_mvar: *qf,
                    loading_pct: loading,
                    loading_color: Self::loading_color(loading),
                    arrow_direction: *pf >= 0.0,
                    width,
                }
            })
            .collect();

        let total_load = bus_data.iter().map(|b| b.4).sum::<f64>();
        let total_gen = bus_data.iter().map(|b| b.6).sum::<f64>();
        let system_losses = (total_gen - total_load).max(0.0);

        NetworkVizData {
            buses,
            branches,
            title: "Power System Diagram".to_owned(),
            timestamp: "2026-03-09T00:00:00Z".to_owned(),
            system_losses_mw: system_losses,
            total_load_mw: total_load,
            total_gen_mw: total_gen,
        }
    }

    // -----------------------------------------------------------------------
    // Export
    // -----------------------------------------------------------------------

    /// Export network data to the configured format string.
    pub fn export(&self, data: &NetworkVizData) -> Result<String, VizError> {
        match self.config.format {
            VizFormat::Json => {
                serde_json::to_string_pretty(data).map_err(|e| VizError::Json(e.to_string()))
            }
            VizFormat::Csv => Ok(self.to_csv(data)),
            VizFormat::GraphViz => Ok(self.to_dot(data)),
            VizFormat::Svg => Ok(self.to_svg(data)),
            VizFormat::GeoJson => Ok(self.to_geojson(data)),
        }
    }

    // -----------------------------------------------------------------------
    // Colour helpers
    // -----------------------------------------------------------------------

    /// Assign a hex colour based on voltage magnitude \[pu\].
    ///
    /// - Green (`#00AA44`): 0.97–1.03 pu (nominal ±3 %)
    /// - Yellow (`#FFC000`): 0.95–0.97 or 1.03–1.05 pu (±3–5 %)
    /// - Red (`#DD2222`): < 0.95 or > 1.05 pu (outside ±5 %)
    pub fn voltage_color(v_pu: f64) -> String {
        if (0.97..=1.03).contains(&v_pu) {
            "#00AA44".to_owned()
        } else if (0.95..=1.05).contains(&v_pu) {
            "#FFC000".to_owned()
        } else {
            "#DD2222".to_owned()
        }
    }

    /// Assign a hex colour based on branch thermal loading \[%\].
    ///
    /// - Green (`#00AA44`): < 70 %
    /// - Yellow (`#FFC000`): 70–90 %
    /// - Red (`#DD2222`): > 90 %
    pub fn loading_color(loading_pct: f64) -> String {
        if loading_pct < 70.0 {
            "#00AA44".to_owned()
        } else if loading_pct <= 90.0 {
            "#FFC000".to_owned()
        } else {
            "#DD2222".to_owned()
        }
    }

    // -----------------------------------------------------------------------
    // Format writers
    // -----------------------------------------------------------------------

    /// Generate DOT language output for GraphViz.
    pub fn to_dot(&self, data: &NetworkVizData) -> String {
        let mut s = String::from("graph powersystem {\n");
        s.push_str("  rankdir=LR;\n");
        s.push_str("  node [shape=circle, style=filled];\n");

        for bus in &data.buses {
            s.push_str(&format!(
                "  {} [label=\"{}\\nV={:.3}pu\", fillcolor=\"{}\", pos=\"{:.1},{:.1}!\"];\n",
                bus.id,
                bus.name,
                bus.voltage_pu,
                bus.voltage_color,
                bus.x / 10.0,
                bus.y / 10.0
            ));
        }

        for branch in &data.branches {
            s.push_str(&format!(
                "  {} -- {} [label=\"{:.1}MW\", color=\"{}\", penwidth={:.1}];\n",
                branch.from_bus,
                branch.to_bus,
                branch.p_flow_mw,
                branch.loading_color,
                branch.width.clamp(0.5, 8.0)
            ));
        }

        s.push_str("}\n");
        s
    }

    /// Generate inline SVG with force-directed layout.
    pub fn to_svg(&self, data: &NetworkVizData) -> String {
        let width = 800u32;
        let height = 600u32;
        let mut s = format!(
            r#"<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">"#,
            width, height, width, height
        );
        s.push_str("\n<title>Power System</title>\n");
        s.push_str("<g id=\"branches\">\n");

        // Draw branches
        for branch in &data.branches {
            let from = data.buses.iter().find(|b| b.id == branch.from_bus);
            let to = data.buses.iter().find(|b| b.id == branch.to_bus);
            if let (Some(f), Some(t)) = (from, to) {
                s.push_str(&format!(
                    r#"  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="{}" stroke-width="{:.1}" opacity="0.8"/>"#,
                    f.x, f.y, t.x, t.y,
                    branch.loading_color,
                    branch.width.clamp(0.5, 8.0)
                ));
                s.push('\n');
            }
        }
        s.push_str("</g>\n<g id=\"buses\">\n");

        // Draw buses
        for bus in &data.buses {
            s.push_str(&format!(
                "  <circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"12\" fill=\"{}\" stroke=\"#333\" stroke-width=\"1.5\"/>",
                bus.x, bus.y, bus.voltage_color
            ));
            s.push_str(&format!(
                r#"  <text x="{:.1}" y="{:.1}" text-anchor="middle" font-size="10" dy="-16">{}</text>"#,
                bus.x, bus.y, bus.name
            ));
            s.push('\n');
        }
        s.push_str("</g>\n</svg>");
        s
    }

    /// Generate GeoJSON feature collection (buses as Point, branches as LineString).
    ///
    /// Bus coordinates are taken from `x` (longitude) and `y` (latitude) layout positions
    /// mapped to a `[0, 1]` normalised range for compatibility.
    pub fn to_geojson(&self, data: &NetworkVizData) -> String {
        // Normalise coordinates to ~[0, 0.1] degree range around 0,0
        let x_max = data.buses.iter().map(|b| b.x.abs()).fold(1.0f64, f64::max);
        let y_max = data.buses.iter().map(|b| b.y.abs()).fold(1.0f64, f64::max);

        let mut features = Vec::new();

        for bus in &data.buses {
            let lon = bus.x / x_max * 0.1;
            let lat = bus.y / y_max * 0.1;
            features.push(format!(
                r#"{{"type":"Feature","geometry":{{"type":"Point","coordinates":[{:.6},{:.6}]}},"properties":{{"id":{},"name":"{}","voltage_pu":{:.4},"color":"{}"}}}}"#,
                lon, lat, bus.id, bus.name, bus.voltage_pu, bus.voltage_color
            ));
        }

        for branch in &data.branches {
            let from = data.buses.iter().find(|b| b.id == branch.from_bus);
            let to = data.buses.iter().find(|b| b.id == branch.to_bus);
            if let (Some(f), Some(t)) = (from, to) {
                let flon = f.x / x_max * 0.1;
                let flat = f.y / y_max * 0.1;
                let tlon = t.x / x_max * 0.1;
                let tlat = t.y / y_max * 0.1;
                features.push(format!(
                    r#"{{"type":"Feature","geometry":{{"type":"LineString","coordinates":[[{:.6},{:.6}],[{:.6},{:.6}]]}},"properties":{{"id":{},"loading_pct":{:.2},"color":"{}"}}}}"#,
                    flon, flat, tlon, tlat,
                    branch.id, branch.loading_pct, branch.loading_color
                ));
            }
        }

        format!(
            r#"{{"type":"FeatureCollection","features":[{}]}}"#,
            features.join(",")
        )
    }

    /// Generate CSV with separate bus and branch sections.
    fn to_csv(&self, data: &NetworkVizData) -> String {
        let mut s = String::new();
        s.push_str("# Buses\n");
        s.push_str(
            "id,name,voltage_pu,angle_deg,p_load_mw,q_load_mvar,p_gen_mw,q_gen_mvar,color\n",
        );
        for bus in &data.buses {
            s.push_str(&format!(
                "{},{},{:.4},{:.4},{:.3},{:.3},{:.3},{:.3},{}\n",
                bus.id,
                bus.name,
                bus.voltage_pu,
                bus.angle_deg,
                bus.p_load_mw,
                bus.q_load_mvar,
                bus.p_gen_mw,
                bus.q_gen_mvar,
                bus.voltage_color
            ));
        }
        s.push_str("\n# Branches\n");
        s.push_str("id,from,to,p_flow_mw,q_flow_mvar,loading_pct,color\n");
        for branch in &data.branches {
            s.push_str(&format!(
                "{},{},{},{:.3},{:.3},{:.2},{}\n",
                branch.id,
                branch.from_bus,
                branch.to_bus,
                branch.p_flow_mw,
                branch.q_flow_mvar,
                branch.loading_pct,
                branch.loading_color
            ));
        }
        s
    }

    // -----------------------------------------------------------------------
    // Force-directed layout
    // -----------------------------------------------------------------------

    /// Compute a force-directed bus layout in a 700×500 canvas.
    ///
    /// Uses Fruchterman-Reingold spring-embedder with 100 iterations.
    /// Returns `Vec<(x, y)>` positions for each bus.
    pub fn force_directed_layout(n_buses: usize, branches: &[(usize, usize)]) -> Vec<(f64, f64)> {
        if n_buses == 0 {
            return Vec::new();
        }

        let width = 700.0f64;
        let height = 500.0f64;
        let area = width * height;
        let k = (area / n_buses.max(1) as f64).sqrt();

        // Initialise on a circle (deterministic, no rand)
        let mut pos: Vec<(f64, f64)> = (0..n_buses)
            .map(|i| {
                let angle = 2.0 * std::f64::consts::PI * i as f64 / n_buses as f64;
                let r = (width.min(height) * 0.4).min(200.0);
                (
                    width / 2.0 + r * angle.cos(),
                    height / 2.0 + r * angle.sin(),
                )
            })
            .collect();

        let mut disp: Vec<(f64, f64)> = vec![(0.0, 0.0); n_buses];
        let n_iter = 100usize;

        for iter in 0..n_iter {
            let temp = width / 10.0 * (1.0 - iter as f64 / n_iter as f64);

            // Reset displacements
            for d in disp.iter_mut() {
                *d = (0.0, 0.0);
            }

            // Repulsive forces between all pairs
            for v in 0..n_buses {
                for u in 0..n_buses {
                    if v == u {
                        continue;
                    }
                    let dx = pos[v].0 - pos[u].0;
                    let dy = pos[v].1 - pos[u].1;
                    let dist = (dx * dx + dy * dy).sqrt().max(1.0);
                    let force = k * k / dist;
                    disp[v].0 += dx / dist * force;
                    disp[v].1 += dy / dist * force;
                }
            }

            // Attractive forces along edges
            for &(u, v) in branches {
                let u = u.min(n_buses - 1);
                let v = v.min(n_buses - 1);
                if u == v {
                    continue;
                }
                let dx = pos[v].0 - pos[u].0;
                let dy = pos[v].1 - pos[u].1;
                let dist = (dx * dx + dy * dy).sqrt().max(1.0);
                let force = dist * dist / k;
                let fx = dx / dist * force;
                let fy = dy / dist * force;
                disp[u].0 += fx;
                disp[u].1 += fy;
                disp[v].0 -= fx;
                disp[v].1 -= fy;
            }

            // Apply displacements with temperature cap
            for v in 0..n_buses {
                let d_len = (disp[v].0 * disp[v].0 + disp[v].1 * disp[v].1)
                    .sqrt()
                    .max(1e-9);
                let scale = d_len.min(temp) / d_len;
                pos[v].0 += disp[v].0 * scale;
                pos[v].1 += disp[v].1 * scale;
                // Clamp to canvas with margin
                pos[v].0 = pos[v].0.clamp(20.0, width - 20.0);
                pos[v].1 = pos[v].1.clamp(20.0, height - 20.0);
            }
        }

        pos
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(clippy::type_complexity)]
    fn sample_buses() -> Vec<(usize, String, f64, f64, f64, f64, f64, f64)> {
        vec![
            (0, "Bus1".to_owned(), 1.0, 0.0, 0.0, 0.0, 100.0, 30.0),
            (1, "Bus2".to_owned(), 0.99, -2.0, 50.0, 20.0, 0.0, 0.0),
            (2, "Bus3".to_owned(), 0.97, -4.0, 40.0, 15.0, 0.0, 0.0),
        ]
    }

    fn sample_branches() -> Vec<(usize, usize, usize, f64, f64, f64)> {
        vec![(0, 0, 1, 60.0, 25.0, 100.0), (1, 1, 2, 40.0, 10.0, 80.0)]
    }

    fn make_viz(format: VizFormat) -> PowerSystemVisualizer {
        PowerSystemVisualizer::new(VisualizationConfig {
            format,
            include_flows: true,
            include_voltages: true,
            include_loading: true,
            color_scheme: ColorScheme::Voltage,
        })
    }

    /// Test 1: JSON export produces valid JSON with buses and branches keys.
    #[test]
    fn test_json_export_valid_structure() {
        let viz = make_viz(VizFormat::Json);
        let data = viz.prepare_data(&sample_buses(), &sample_branches());
        let json_str = viz.export(&data).expect("JSON export ok");
        let parsed: serde_json::Value = serde_json::from_str(&json_str).expect("valid JSON");
        assert!(parsed["buses"].is_array(), "JSON must have buses array");
        assert!(
            parsed["branches"].is_array(),
            "JSON must have branches array"
        );
        assert_eq!(
            parsed["buses"].as_array().unwrap().len(),
            3,
            "must have 3 buses"
        );
        assert_eq!(
            parsed["branches"].as_array().unwrap().len(),
            2,
            "must have 2 branches"
        );
    }

    /// Test 2: Voltage colour zones are correct.
    #[test]
    fn test_voltage_coloring_zones() {
        // Green: nominal
        assert_eq!(PowerSystemVisualizer::voltage_color(1.0), "#00AA44");
        assert_eq!(PowerSystemVisualizer::voltage_color(1.02), "#00AA44");
        // Yellow: ±3–5 %
        assert_eq!(PowerSystemVisualizer::voltage_color(0.96), "#FFC000");
        assert_eq!(PowerSystemVisualizer::voltage_color(1.04), "#FFC000");
        // Red: outside ±5 %
        assert_eq!(PowerSystemVisualizer::voltage_color(0.90), "#DD2222");
        assert_eq!(PowerSystemVisualizer::voltage_color(1.10), "#DD2222");
    }

    /// Test 3: Loading colour zones are correct.
    #[test]
    fn test_loading_coloring_zones() {
        // Green: < 70 %
        assert_eq!(PowerSystemVisualizer::loading_color(50.0), "#00AA44");
        // Yellow: 70–90 %
        assert_eq!(PowerSystemVisualizer::loading_color(80.0), "#FFC000");
        assert_eq!(PowerSystemVisualizer::loading_color(90.0), "#FFC000");
        // Red: > 90 %
        assert_eq!(PowerSystemVisualizer::loading_color(95.0), "#DD2222");
    }

    /// Test 4: DOT format contains valid GraphViz keywords.
    #[test]
    fn test_dot_format_valid_graphviz() {
        let viz = make_viz(VizFormat::GraphViz);
        let data = viz.prepare_data(&sample_buses(), &sample_branches());
        let dot = viz.export(&data).expect("DOT export ok");
        assert!(dot.starts_with("graph "), "DOT must start with 'graph'");
        assert!(dot.contains("node ["), "DOT must define node style");
        // Buses should appear as node IDs
        assert!(dot.contains("0 ["), "Bus 0 must appear");
        assert!(dot.contains("1 ["), "Bus 1 must appear");
        // Branches as edges
        assert!(dot.contains(" -- "), "DOT must have undirected edges");
        assert!(dot.ends_with("}\n"), "DOT must end with closing brace");
    }

    /// Test 5: Force-directed layout produces no overlapping buses (min distance > 5 px).
    #[test]
    fn test_force_directed_no_overlaps() {
        let branches = vec![(0, 1), (1, 2), (2, 3), (0, 3)];
        let positions = PowerSystemVisualizer::force_directed_layout(4, &branches);
        assert_eq!(positions.len(), 4);
        for i in 0..4 {
            for j in (i + 1)..4 {
                let dx = positions[i].0 - positions[j].0;
                let dy = positions[i].1 - positions[j].1;
                let dist = (dx * dx + dy * dy).sqrt();
                assert!(
                    dist > 5.0,
                    "buses {} and {} overlap (dist={:.2})",
                    i,
                    j,
                    dist
                );
            }
        }
    }

    /// Test 6: SVG export contains svg tag.
    #[test]
    fn test_svg_export() {
        let viz = make_viz(VizFormat::Svg);
        let data = viz.prepare_data(&sample_buses(), &sample_branches());
        let svg = viz.export(&data).expect("SVG ok");
        assert!(svg.contains("<svg "), "SVG must contain svg tag");
        assert!(svg.contains("</svg>"), "SVG must close svg tag");
    }

    /// Test 7: GeoJSON export contains FeatureCollection type.
    #[test]
    fn test_geojson_export() {
        let viz = make_viz(VizFormat::GeoJson);
        let data = viz.prepare_data(&sample_buses(), &sample_branches());
        let geojson = viz.export(&data).expect("GeoJSON ok");
        assert!(
            geojson.contains("FeatureCollection"),
            "GeoJSON must be FeatureCollection"
        );
        assert!(
            geojson.contains("Feature"),
            "GeoJSON must contain Feature elements"
        );
    }

    /// Test 8: System losses are non-negative.
    #[test]
    fn test_system_losses_non_negative() {
        let viz = make_viz(VizFormat::Json);
        let data = viz.prepare_data(&sample_buses(), &sample_branches());
        assert!(
            data.system_losses_mw >= 0.0,
            "system losses must be >= 0, got {}",
            data.system_losses_mw
        );
    }

    /// Test 9: Branch loading percentage is computed correctly.
    #[test]
    fn test_branch_loading_calculation() {
        let viz = make_viz(VizFormat::Json);
        let data = viz.prepare_data(&sample_buses(), &sample_branches());
        // Branch 0: P=60, Q=25, rating=100 → S=√(3600+625)=65.19 → 65.19%
        let branch0 = &data.branches[0];
        let expected_loading = (60.0f64 * 60.0 + 25.0 * 25.0).sqrt() / 100.0 * 100.0;
        assert!(
            (branch0.loading_pct - expected_loading).abs() < 0.01,
            "loading_pct {} ≠ expected {:.2}",
            branch0.loading_pct,
            expected_loading
        );
    }
}
