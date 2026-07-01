# Battery

Purpose: Battery contains models, estimators, and utilities for battery cell, module, and pack simulation and analysis.

Responsibilities:

- Equivalent circuit models (Rint, 1RC, 2RC)
- SoC/SoH estimation (Coulomb counting, EKF, UKF)
- Thermal modelling and electrothermal coupling
- Aging and degradation models
- BMS utilities, safety checks, and schedulers

See `examples/battery_cycling.rs` for a runnable example.
