use serde::{Deserialize, Serialize};

/// Per-branch power flow result (from-bus end).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BranchFlow {
    /// Index of the branch in the network branch list.
    pub branch_index: usize,
    pub from_bus: usize,
    pub to_bus: usize,
    pub p_from_mw: f64,
    pub q_from_mvar: f64,
    pub p_to_mw: f64,
    pub q_to_mvar: f64,
    pub p_loss_mw: f64,
    pub q_loss_mvar: f64,
    /// Branch loading as a percentage of its thermal rating.
    pub loading_pct: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerFlowResult {
    pub voltage_magnitude: Vec<f64>, // p.u.
    pub voltage_angle: Vec<f64>,     // radians
    pub p_injected: Vec<f64>,        // MW
    pub q_injected: Vec<f64>,        // MVAr
    pub branch_flows: Vec<BranchFlow>,
    pub total_p_loss_mw: f64,
    pub total_q_loss_mvar: f64,
    pub converged: bool,
    pub iterations: usize,
    pub max_mismatch: f64,
}

impl PowerFlowResult {
    pub fn voltage_angle_degrees(&self) -> Vec<f64> {
        self.voltage_angle.iter().map(|a| a.to_degrees()).collect()
    }

    /// Total real power loss (MW) — sum over all bus injections.
    pub fn total_p_loss(&self) -> f64 {
        self.total_p_loss_mw
    }

    /// Total reactive power loss (MVAr).
    pub fn total_q_loss(&self) -> f64 {
        self.total_q_loss_mvar
    }

    /// Total real power losses (MW) — sum of branch losses.
    pub fn total_losses_mw(&self) -> f64 {
        self.branch_flows.iter().map(|b| b.p_loss_mw).sum()
    }

    /// Total reactive power losses (MVAr) — sum of branch losses.
    pub fn total_losses_mvar(&self) -> f64 {
        self.branch_flows.iter().map(|b| b.q_loss_mvar).sum()
    }

    /// Maximum branch loading percentage across all branches.
    pub fn max_branch_loading_pct(&self) -> f64 {
        self.branch_flows
            .iter()
            .map(|b| b.loading_pct)
            .fold(0.0_f64, f64::max)
    }

    /// Returns references to all branches whose loading exceeds 100%.
    pub fn overloaded_branches(&self) -> Vec<&BranchFlow> {
        self.branch_flows
            .iter()
            .filter(|b| b.loading_pct > 100.0)
            .collect()
    }
}

impl std::fmt::Display for PowerFlowResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Power Flow Result:")?;
        writeln!(
            f,
            "  Converged: {} in {} iterations",
            self.converged, self.iterations
        )?;
        writeln!(f, "  Max mismatch: {:.2e}", self.max_mismatch)?;
        writeln!(f, "  Total P loss: {:.4} MW", self.total_p_loss_mw)?;
        writeln!(f, "  Total Q loss: {:.4} MVAr", self.total_q_loss_mvar)?;
        writeln!(f, "  Bus Voltages:")?;
        for (i, (vm, va)) in self
            .voltage_magnitude
            .iter()
            .zip(self.voltage_angle.iter())
            .enumerate()
        {
            writeln!(
                f,
                "    Bus {:>3}: V = {:.4} p.u., angle = {:>8.3} deg",
                i + 1,
                vm,
                va.to_degrees()
            )?;
        }
        Ok(())
    }
}
