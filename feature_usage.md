# Cargo Feature Usage in oxigrid

This file explains the feature flags declared in `Cargo.toml` and how they affect the build.

## What is a feature?

A feature is a named Cargo build option. It is declared in `Cargo.toml` under `[features]` and can:

- enable or disable code via `#[cfg(feature = "...")]`
- activate optional dependencies
- enable related feature groups

A feature that is enabled is included in the crate build. A feature that is declared but unused is valid, but it has no effect unless code or dependencies reference it.

An undeclared feature cannot be enabled by Cargo.

## What `#[cfg(feature = "...")]` means

`#[cfg(feature = "foo")]` instructs Rust to compile the annotated code only when the `foo` feature is active.

Example:

```rust
#[cfg(feature = "simd")]
fn fast_kernel() { ... }
```

If `simd` is enabled, `fast_kernel` is compiled in. Otherwise it is omitted.

## oxigrid feature definitions

These are the feature flags currently declared in `Cargo.toml`.

- `std = []`
  - Enables `std`-specific code paths.
  - Allows the crate to support standard library builds.

- `powerflow = []`
  - Enables the powerflow subsystem.
  - Many modules depend on this feature.

- `stability = ["powerflow"]`
  - Enables stability features and also activates `powerflow`.

- `battery = []`
  - Enables battery subsystem code.

- `battery-p2d = ["battery"]`
  - Enables the battery P2D variant and pulls in `battery`.

- `renewable = []`
  - Enables renewable-energy capabilities.

- `optimize = ["powerflow"]`
  - Enables optimization features and requires `powerflow`.

- `harmonics = []`
  - Enables harmonic-analysis code.

- `protection = ["powerflow"]`
  - Enables protection features and requires `powerflow`.

- `powerelectronics = []`
  - Enables power electronics support.

- `simd = []`
  - Enables SIMD-accelerated kernels.
  - Used by powerflow linear algebra code.

- `parallel = ["std", "dep:rayon"]`
  - Enables `std` and the `rayon` dependency feature.
  - Activates Rayon-based parallelism where available.

## Why these are required

These feature flags are required because the code contains `#[cfg(feature = "...")]` checks for them. Some features also depend on others, for example:

- `stability` requires `powerflow`
- `optimize` requires `powerflow`
- `protection` requires `powerflow`
- `parallel` requires `std`

## Current default behavior

The crate currently enables this set of features by default:

- `std`
- `powerflow`
- `stability`
- `battery`
- `battery-p2d`
- `renewable`
- `optimize`
- `harmonics`
- `protection`
- `powerelectronics`

This means the library builds with a broad set of capabilities unless a consumer explicitly disables default features.

## Note

A declared feature can exist without any code using it. That is harmless but inert.

If you want, we can also trim the default feature set to make `oxigrid` lighter by default.
