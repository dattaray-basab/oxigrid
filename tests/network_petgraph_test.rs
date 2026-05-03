//! Tests for petgraph-backed PowerNetwork topology methods.
use oxigrid::network::{Branch, Bus, BusType, PowerNetwork};

fn make_radial_5bus() -> PowerNetwork {
    let mut net = PowerNetwork::new(100.0);
    for i in 1..=5usize {
        let mut bus = Bus::new(i, BusType::PQ);
        bus.name = format!("Bus{}", i);
        net.buses.push(bus);
    }
    // Tree: 1-2, 2-3, 3-4, 4-5 (radial chain)
    for i in 1..=4usize {
        net.branches.push(Branch {
            from_bus: i,
            to_bus: i + 1,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 1.0,
            shift: 0.0,
            status: true,
        });
    }
    net
}

fn make_meshed_4bus() -> PowerNetwork {
    let mut net = PowerNetwork::new(100.0);
    for i in 1..=4usize {
        let mut bus = Bus::new(i, BusType::PQ);
        bus.name = format!("Bus{}", i);
        net.buses.push(bus);
    }
    // Ring + diagonal: 1-2, 2-3, 3-4, 4-1, 1-3 (meshed)
    let edges = [
        (1usize, 2usize, 0.01f64, 0.1f64),
        (2, 3, 0.01, 0.1),
        (3, 4, 0.01, 0.1),
        (4, 1, 0.01, 0.1),
        (1, 3, 0.02, 0.2),
    ];
    for (f, t, r, x) in edges {
        net.branches.push(Branch {
            from_bus: f,
            to_bus: t,
            r,
            x,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 1.0,
            shift: 0.0,
            status: true,
        });
    }
    net
}

#[test]
fn test_radial_is_connected() {
    let net = make_radial_5bus();
    assert!(net.is_connected());
}

#[test]
fn test_radial_is_radial() {
    let net = make_radial_5bus();
    assert!(net.is_radial());
}

#[test]
fn test_meshed_not_radial() {
    let net = make_meshed_4bus();
    assert!(net.is_connected());
    // 4 buses, 5 branches — has a cycle, not a tree
    assert!(!net.is_radial());
}

#[test]
fn test_disconnected_two_components() {
    let mut net = make_radial_5bus();
    // Remove the only edge connecting buses 2-3, splitting the chain into
    // component {1,2} and component {3,4,5}.
    net.branches.retain(|b| !(b.from_bus == 2 && b.to_bus == 3));
    assert!(!net.is_connected());
    let comps = net.connected_components_petgraph();
    assert_eq!(comps.len(), 2, "Expected 2 components, got: {:?}", comps);
}

#[test]
fn test_bfs_visit_order_radial() {
    let net = make_radial_5bus();
    // Start at internal idx 0 (bus 1)
    let order = net.bfs_visit_order(0);
    assert_eq!(order.len(), 5);
    assert_eq!(order[0], 0); // start bus is always first
}

#[test]
fn test_bfs_visit_order_out_of_range() {
    let net = make_radial_5bus();
    let order = net.bfs_visit_order(99);
    assert!(order.is_empty());
}

#[test]
fn test_spanning_tree_radial_is_itself() {
    let net = make_radial_5bus();
    let mst = net.spanning_tree();
    // A 5-bus radial tree already has n-1=4 branches — MST is the whole tree.
    assert_eq!(mst.len(), 4);
}

#[test]
fn test_spanning_tree_meshed() {
    let net = make_meshed_4bus();
    let mst = net.spanning_tree();
    // MST of 4-bus network should have exactly 3 edges (n-1).
    assert_eq!(mst.len(), 3, "MST of 4-bus network should have 3 edges");
}

#[test]
fn test_shortest_path_basic() {
    let net = make_radial_5bus();
    // Internal indices 0 through 4 (buses 1 through 5)
    let result = net.shortest_path(0, 4);
    assert!(result.is_some());
    let (cost, path) = result.unwrap();
    assert!(cost > 0.0);
    assert_eq!(path.first(), Some(&0));
    assert_eq!(path.last(), Some(&4));
}

#[test]
fn test_shortest_path_same_node() {
    let net = make_radial_5bus();
    let result = net.shortest_path(2, 2);
    assert!(result.is_some());
    let (cost, path) = result.unwrap();
    // Zero cost, single-element path
    assert!(cost.abs() < 1e-12);
    assert_eq!(path, vec![2]);
}

#[test]
fn test_shortest_path_out_of_range() {
    let net = make_radial_5bus();
    assert!(net.shortest_path(0, 99).is_none());
    assert!(net.shortest_path(99, 0).is_none());
}

#[test]
fn test_shortest_path_disconnected() {
    let mut net = make_radial_5bus();
    // Split into {0,1} and {2,3,4}
    net.branches.retain(|b| !(b.from_bus == 2 && b.to_bus == 3));
    // No path from bus 0 to bus 4
    let result = net.shortest_path(0, 4);
    assert!(result.is_none());
}

#[test]
fn test_empty_network() {
    let net = PowerNetwork::new(100.0);
    assert!(net.is_connected()); // empty is trivially connected
    assert!(net.connected_components_petgraph().is_empty());
    assert!(!net.is_radial()); // 0 buses — not radial
}

#[test]
fn test_connected_components_single_bus() {
    let mut net = PowerNetwork::new(100.0);
    net.buses.push(Bus::new(1, BusType::PQ));
    let comps = net.connected_components_petgraph();
    assert_eq!(comps.len(), 1);
    assert_eq!(comps[0], vec![0]);
}

#[test]
fn test_spanning_tree_disconnected_returns_partial() {
    let mut net = make_radial_5bus();
    // Split into two components — MST covers n-1 within each component.
    net.branches.retain(|b| !(b.from_bus == 2 && b.to_bus == 3));
    let mst = net.spanning_tree();
    // Component {0,1}: 1 edge; Component {2,3,4}: 2 edges → 3 total
    assert_eq!(mst.len(), 3);
}
