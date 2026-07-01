# Powerflow

Purpose: Powerflow implements AC and DC power flow solvers, state estimation, and related utilities.

Responsibilities:

- Newton-Raphson, Fast-Decoupled, and DC approximations
- Sparse/dense Jacobian assembly and LU selection
- State estimation (AC WLS) and EKF interfaces
- Branch flow and loss computations

See `examples/ieee14_powerflow.rs` for usage.
