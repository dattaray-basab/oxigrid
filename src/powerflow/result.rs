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

#[cfg(test)]
mod tests {
    use super::{BranchFlow, PowerFlowResult};
    use std::f64::consts::PI;

    fn make_branch(
        index: usize,
        from: usize,
        to: usize,
        p_loss: f64,
        q_loss: f64,
        loading: f64,
    ) -> BranchFlow {
        BranchFlow {
            branch_index: index,
            from_bus: from,
            to_bus: to,
            p_from_mw: 10.0,
            q_from_mvar: 5.0,
            p_to_mw: 10.0 - p_loss,
            q_to_mvar: 5.0 - q_loss,
            p_loss_mw: p_loss,
            q_loss_mvar: q_loss,
            loading_pct: loading,
        }
    }

    fn make_result(branch_flows: Vec<BranchFlow>) -> PowerFlowResult {
        PowerFlowResult {
            voltage_magnitude: vec![1.0, 0.98],
            voltage_angle: vec![0.0, PI / 4.0],
            p_injected: vec![50.0, -50.0],
            q_injected: vec![10.0, -10.0],
            total_p_loss_mw: 1.5,
            total_q_loss_mvar: 0.3,
            converged: true,
            iterations: 5,
            max_mismatch: 1e-6,
            branch_flows,
        }
    }

    #[test]
    fn voltage_angle_degrees_converts_pi_over_four_to_45() {
        let result = make_result(vec![]);
        let degrees = result.voltage_angle_degrees();
        assert!(
            (degrees[1] - 45.0).abs() < 1e-10,
            "expected 45.0 deg, got {}",
            degrees[1]
        );
    }

    #[test]
    fn total_p_loss_returns_stored_value() {
        let result = make_result(vec![]);
        assert!(
            (result.total_p_loss() - 1.5).abs() < 1e-10,
            "expected 1.5, got {}",
            result.total_p_loss()
        );
    }

    #[test]
    fn total_losses_mw_sums_branch_p_losses() {
        let branches = vec![
            make_branch(0, 0, 1, 0.4, 0.1, 60.0),
            make_branch(1, 1, 2, 0.6, 0.2, 80.0),
        ];
        let result = make_result(branches);
        let total = result.total_losses_mw();
        assert!((total - 1.0).abs() < 1e-10, "expected 1.0, got {}", total);
    }

    #[test]
    fn total_losses_mvar_sums_branch_q_losses() {
        let branches = vec![
            make_branch(0, 0, 1, 0.4, 0.15, 60.0),
            make_branch(1, 1, 2, 0.6, 0.25, 80.0),
        ];
        let result = make_result(branches);
        let total = result.total_losses_mvar();
        assert!((total - 0.4).abs() < 1e-10, "expected 0.4, got {}", total);
    }

    #[test]
    fn max_branch_loading_pct_returns_maximum() {
        let branches = vec![
            make_branch(0, 0, 1, 0.2, 0.05, 55.0),
            make_branch(1, 1, 2, 0.3, 0.08, 120.0),
            make_branch(2, 2, 3, 0.1, 0.03, 80.0),
        ];
        let result = make_result(branches);
        assert!(
            (result.max_branch_loading_pct() - 120.0).abs() < 1e-10,
            "expected 120.0, got {}",
            result.max_branch_loading_pct()
        );
    }

    #[test]
    fn overloaded_branches_returns_only_branches_above_100_pct() {
        let branches = vec![
            make_branch(0, 0, 1, 0.2, 0.05, 95.0),
            make_branch(1, 1, 2, 0.3, 0.08, 110.0),
            make_branch(2, 2, 3, 0.1, 0.03, 105.0),
        ];
        let result = make_result(branches);
        let overloaded = result.overloaded_branches();
        assert_eq!(
            overloaded.len(),
            2,
            "expected 2 overloaded branches, got {}",
            overloaded.len()
        );
        assert!(
            overloaded.iter().all(|b| b.loading_pct > 100.0),
            "all returned branches must have loading_pct > 100"
        );
    }

    #[test]
    fn display_contains_converged_and_bus_voltage_info() {
        let result = make_result(vec![]);
        let output = format!("{}", result);
        assert!(
            output.contains("Converged"),
            "Display output missing 'Converged': {}",
            output
        );
        assert!(
            output.contains("Bus Voltages"),
            "Display output missing 'Bus Voltages': {}",
            output
        );
        assert!(
            output.contains("1.0000"),
            "Display output missing voltage magnitude: {}",
            output
        );
    }
}
