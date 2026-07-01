# Units

Purpose: Units defines strongly-typed wrappers and conversions for electrical, thermal, and energy units.

Responsibilities:

- Newtype wrappers (`Voltage`, `Current`, `Power`, `Energy`, `Temperature`)
- Arithmetic and `From` trait conversions
- `no_std` compatible helpers and per-unit conversion utilities

See `src/units` for conversion implementations and tests.
