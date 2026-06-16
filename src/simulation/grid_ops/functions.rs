//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::constants::{LCG_ADD, LCG_MULT};

pub(super) fn lcg_next(state: &mut u64) -> f64 {
    *state = state.wrapping_mul(LCG_MULT).wrapping_add(LCG_ADD);
    (*state >> 32) as f64 / u32::MAX as f64
}
