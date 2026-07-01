# Network

Purpose: Network implements the core bus/branch topologies, admittance (Y-bus) construction, and topology utilities.

Responsibilities:

- Bus/branch data models and topologies
- Y-bus / admittance matrix construction
- Topology algorithms (connected components, radial checks)
- Petgraph integration and network reductions (Kron)

This module is a dependency for `powerflow`, `stability`, and `protection`.
