# IO

Purpose: IO contains import/export utilities for common power system data formats and time-series handling.

Responsibilities:

- MATPOWER, pandapower, and IEEE format parsers
- CSV time-series import/export
- PMU/SCADA frame types and serializers
- Serde serialization helpers for public structs

See `tests/data/` for canonical network files used by tests.
