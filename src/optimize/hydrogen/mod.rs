//! Hydrogen and Power-to-Gas (P2G) storage and dispatch modeling.
//!
//! # Sub-modules
//!
//! | Module | Description |
//! |--------|-------------|
//! | [`electrolyzer`] | PEM/Alkaline/SOEC electrolyzer models with efficiency curves and IV-curve electrochemistry |
//! | [`storage`]      | Compressed gas, liquid H2, metal hydride, and underground hydrogen tank models |
//! | [`fuel_cell`]    | PEMFC/SOFC/MCFC fuel cell models with CHP heat recovery |
//! | [`p2g_system`]   | Integrated P2G system dispatch: greedy/heuristic scheduler and portfolio optimisation |

pub mod electrolyzer;
pub mod fuel_cell;
pub mod p2g_system;
pub mod seasonal_storage;
pub mod storage;
pub mod valley;

pub use electrolyzer::{Electrolyzer, ElectrolyzerStack, ElectrolyzerType, H2_HHV_KWH_PER_KG};
pub use fuel_cell::{FuelCell, FuelCellType};
pub use p2g_system::{
    optimize_p2g_portfolio, P2gDispatchConfig, P2gDispatchResult, P2gDispatcher, P2gOptimizeMode,
    P2gSystem,
};
pub use seasonal_storage::{
    ElectrolyzerFleet, FuelCellFleet, H2EconomicsResult, SeasonalH2Optimizer,
    SeasonalStorageConfig, SeasonalStorageType, WeeklyDispatch, YearlySimResult,
};
pub use storage::{HydrogenStorageType, HydrogenTank};
pub use valley::{
    ElectrolyzerUnit, FuelCellUnit, H2Error, H2StorageTank, HydrogenValleyConfig,
    HydrogenValleyOptimizer, HydrogenValleyResult,
};
