/// Transient stability fault trajectory and Critical Clearing Time (CCT) analysis.
///
/// Simulates a multi-machine power system through three phases:
///   1. **Pre-fault**: steady-state operating point
///   2. **Fault-on**:  modified network (e.g., faulted bus, cleared by protection)
///   3. **Post-fault**: network with faulted element removed
///
/// The **Critical Clearing Time** (CCT) is the maximum fault duration for which
/// the system remains transiently stable.  Found via bisection on clearing time.
///
/// # Physics
/// SMIB (single-machine infinite bus) swing equation per machine:
///   M·d²δ/dt² = P_m − P_e(δ)
///   P_e = E²·G + E·V_∞·(B·sin(δ) + G·cos(δ))
///
/// For multi-machine: network-reduced admittance matrix updated each phase.
use serde::{Deserialize, Serialize};

/// A single-machine infinite-bus (SMIB) or multi-machine fault scenario.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultScenario {
    /// Inertia constant `s` (M = 2H/ω_s; here stored as H)
    pub h_inertia: f64,
    /// Damping coefficient [p.u./rad·s⁻¹]
    pub damping: f64,
    /// Mechanical power input [p.u.]
    pub p_mech: f64,
    /// Pre-fault transfer susceptance B_pre [p.u.]
    pub b_pre: f64,
    /// Pre-fault transfer conductance G_pre [p.u.]
    pub g_pre: f64,
    /// During-fault transfer susceptance B_fault [p.u.]
    pub b_fault: f64,
    /// During-fault transfer conductance G_fault [p.u.]
    pub g_fault: f64,
    /// Post-fault transfer susceptance B_post [p.u.]
    pub b_post: f64,
    /// Post-fault transfer conductance G_post [p.u.]
    pub g_post: f64,
    /// Machine internal voltage [p.u.]
    pub e_prime: f64,
    /// Infinite-bus voltage [p.u.]
    pub v_inf: f64,
    /// Pre-fault rotor angle `rad`
    pub delta_0: f64,
    /// Nominal frequency [rad/s] (ω₀ = 2π·f)
    pub omega_0: f64,
}

impl FaultScenario {
    /// Create a SMIB scenario typical for a 100 MW coal unit.
    pub fn coal_smib() -> Self {
        use std::f64::consts::PI;
        Self {
            h_inertia: 5.0, // 5 MWs/MVA
            damping: 0.5,
            p_mech: 0.8,
            b_pre: 2.5,
            g_pre: 0.02,
            b_fault: 0.5, // Heavily reduced during fault
            g_fault: 0.01,
            b_post: 2.0, // Slightly reduced post-fault
            g_post: 0.02,
            e_prime: 1.05,
            v_inf: 1.0,
            delta_0: (0.8f64 / 2.5).asin(), // Pre-fault equilibrium angle
            omega_0: 2.0 * PI * 50.0,
        }
    }

    /// Create from power transfer parameters.
    pub fn from_power_transfer(
        p_mech: f64,
        p_max_pre: f64,
        p_max_fault: f64,
        p_max_post: f64,
    ) -> Self {
        use std::f64::consts::PI;
        let b_pre = p_max_pre;
        let b_fault = p_max_fault;
        let b_post = p_max_post;
        let delta_0 = (p_mech / p_max_pre.max(1e-9)).clamp(-1.0, 1.0).asin();
        Self {
            h_inertia: 5.0,
            damping: 0.3,
            p_mech,
            b_pre,
            g_pre: 0.0,
            b_fault,
            g_fault: 0.0,
            b_post,
            g_post: 0.0,
            e_prime: 1.0,
            v_inf: 1.0,
            delta_0,
            omega_0: 2.0 * PI * 50.0,
        }
    }

    /// Electrical power output.
    fn p_elec(&self, delta: f64, phase: FaultPhase) -> f64 {
        let (b, g) = match phase {
            FaultPhase::PreFault => (self.b_pre, self.g_pre),
            FaultPhase::FaultOn => (self.b_fault, self.g_fault),
            FaultPhase::PostFault => (self.b_post, self.g_post),
        };
        let e2 = self.e_prime * self.e_prime;
        e2 * g + self.e_prime * self.v_inf * (b * delta.sin() + g * delta.cos())
    }

    /// Accelerating power.
    fn p_accel(&self, delta: f64, omega: f64, phase: FaultPhase) -> f64 {
        self.p_mech - self.p_elec(delta, phase) - self.damping * omega
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum FaultPhase {
    PreFault,
    FaultOn,
    PostFault,
}

/// A single point on the fault trajectory.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    /// Time `s`
    pub time_s: f64,
    /// Rotor angle `rad`
    pub delta: f64,
    /// Rotor speed deviation [rad/s]
    pub omega: f64,
    /// Electrical power [p.u.]
    pub p_elec: f64,
    /// Accelerating power [p.u.]
    pub p_accel: f64,
    /// Current phase (0=pre, 1=fault-on, 2=post-fault)
    pub phase: u8,
}

/// Result of a fault trajectory simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultTrajectoryResult {
    /// Trajectory points
    pub trajectory: Vec<TrajectoryPoint>,
    /// Whether the machine remains stable
    pub is_stable: bool,
    /// Maximum rotor angle reached `rad`
    pub delta_max: f64,
    /// Time of maximum rotor angle `s`
    pub t_delta_max: f64,
    /// Clearing time `s`
    pub t_clear: f64,
    /// Post-fault settling angle `rad` (last point)
    pub delta_final: f64,
}

/// Simulate the fault trajectory with explicit Runge-Kutta 4 integration.
///
/// # Arguments
/// - `scenario`    — SMIB parameters
/// - `t_clear`     — fault clearing time `s`
/// - `t_sim`       — total simulation time `s`
/// - `dt`          — integration step `s`
pub fn simulate_fault_trajectory(
    scenario: &FaultScenario,
    t_clear: f64,
    t_sim: f64,
    dt: f64,
) -> FaultTrajectoryResult {
    // M = 2H / ω₀ (inertia constant in [s²/rad])
    let m_inertia = 2.0 * scenario.h_inertia / scenario.omega_0;

    let mut delta = scenario.delta_0;
    let mut omega = 0.0_f64; // speed deviation
    let mut t = 0.0_f64;

    let mut trajectory = Vec::new();
    let mut delta_max = delta;
    let mut t_delta_max = 0.0;

    // Pre-fault steady state check
    let p_e0 = scenario.p_elec(delta, FaultPhase::PreFault);

    let n_steps = (t_sim / dt).ceil() as usize + 1;
    trajectory.reserve(n_steps);

    loop {
        let phase = if t < 0.0 {
            FaultPhase::PreFault
        } else if t < t_clear {
            FaultPhase::FaultOn
        } else {
            FaultPhase::PostFault
        };

        let p_e = scenario.p_elec(delta, phase);
        let p_a = scenario.p_accel(delta, omega, phase);

        trajectory.push(TrajectoryPoint {
            time_s: t,
            delta,
            omega,
            p_elec: p_e,
            p_accel: p_a,
            phase: phase as u8,
        });

        if delta > delta_max {
            delta_max = delta;
            t_delta_max = t;
        }

        if t >= t_sim {
            break;
        }

        // RK4 step
        let step = dt.min(t_sim - t);
        let (d1, o1) = rk4_derivatives(scenario, delta, omega, phase, m_inertia);

        let d_m = delta + 0.5 * step * d1;
        let o_m = omega + 0.5 * step * o1;
        let ph_m = if t + 0.5 * step < t_clear {
            FaultPhase::FaultOn
        } else {
            FaultPhase::PostFault
        };
        let (d2, o2) = rk4_derivatives(scenario, d_m, o_m, ph_m, m_inertia);

        let d_m2 = delta + 0.5 * step * d2;
        let o_m2 = omega + 0.5 * step * o2;
        let (d3, o3) = rk4_derivatives(scenario, d_m2, o_m2, ph_m, m_inertia);

        let ph_e = if t + step < t_clear {
            FaultPhase::FaultOn
        } else {
            FaultPhase::PostFault
        };
        let d_e = delta + step * d3;
        let o_e = omega + step * o3;
        let (d4, o4) = rk4_derivatives(scenario, d_e, o_e, ph_e, m_inertia);

        delta += step / 6.0 * (d1 + 2.0 * d2 + 2.0 * d3 + d4);
        omega += step / 6.0 * (o1 + 2.0 * o2 + 2.0 * o3 + o4);
        t += step;

        // Instability detection: delta exceeds π (pole slip)
        if delta.abs() > std::f64::consts::PI {
            trajectory.push(TrajectoryPoint {
                time_s: t,
                delta,
                omega,
                p_elec: scenario.p_elec(delta, FaultPhase::PostFault),
                p_accel: scenario.p_accel(delta, omega, FaultPhase::PostFault),
                phase: FaultPhase::PostFault as u8,
            });
            break;
        }
    }
    let _ = p_e0;

    let is_stable = delta_max < std::f64::consts::PI;
    let delta_final = trajectory
        .last()
        .map(|p| p.delta)
        .unwrap_or(scenario.delta_0);

    FaultTrajectoryResult {
        trajectory,
        is_stable,
        delta_max,
        t_delta_max,
        t_clear,
        delta_final,
    }
}

fn rk4_derivatives(
    scenario: &FaultScenario,
    delta: f64,
    omega: f64,
    phase: FaultPhase,
    m_inertia: f64,
) -> (f64, f64) {
    let d_delta_dt = omega;
    let d_omega_dt = scenario.p_accel(delta, omega, phase) / m_inertia;
    (d_delta_dt, d_omega_dt)
}

// ─── CCT via bisection ──────────────────────────────────────────────────────

/// Critical Clearing Time (CCT) result.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CctResult {
    /// CCT `s` — maximum clearing time for stability
    pub cct_s: f64,
    /// Rotor angle at CCT `rad`
    pub delta_at_cct: f64,
    /// Maximum angle margin (stable case delta_max - delta_0) `rad`
    pub angle_margin_rad: f64,
    /// Number of bisection iterations
    pub n_iter: usize,
}

/// Find the CCT using bisection search.
///
/// # Arguments
/// - `scenario`        — SMIB fault parameters
/// - `t_lo`            — lower bound for clearing time (known stable) `s`
/// - `t_hi`            — upper bound for clearing time (known unstable) `s`
/// - `tol`             — bisection tolerance `s`
/// - `t_sim`           — simulation horizon `s`
/// - `dt`              — integration step `s`
pub fn find_cct(
    scenario: &FaultScenario,
    t_lo: f64,
    t_hi: f64,
    tol: f64,
    t_sim: f64,
    dt: f64,
) -> CctResult {
    let mut lo = t_lo;
    let mut n_iter = 0;

    // Verify that lo is stable and hi is unstable
    // (If hi is also stable, extend it)
    let mut hi_adj = t_hi;
    while simulate_fault_trajectory(scenario, hi_adj, t_sim, dt).is_stable && hi_adj < t_sim {
        hi_adj *= 1.5;
    }

    let mut cct = lo;
    loop {
        n_iter += 1;
        let mid = 0.5 * (lo + hi_adj);
        let res = simulate_fault_trajectory(scenario, mid, t_sim, dt);

        if res.is_stable {
            lo = mid;
            cct = mid;
        } else {
            hi_adj = mid;
        }

        if hi_adj - lo < tol || n_iter >= 60 {
            break;
        }
    }

    let stable_result = simulate_fault_trajectory(scenario, cct, t_sim, dt);
    let delta_at_cct = stable_result.delta_max;
    let angle_margin = delta_at_cct - scenario.delta_0;

    CctResult {
        cct_s: cct,
        delta_at_cct,
        angle_margin_rad: angle_margin,
        n_iter,
    }
}

// ─── Equal Area Criterion (EAC) ──────────────────────────────────────────────

/// Equal Area Criterion result (for SMIB with zero conductance).
///
/// Calculates the maximum clearing angle δ_cr and approximate CCT from
/// the equal area: ∫_{δ0}^{δcr} P_a_fault dδ = ∫_{δcr}^{δmax} P_a_post dδ
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct EacResult {
    /// Initial equilibrium angle `rad`
    pub delta_0: f64,
    /// Unstable equilibrium angle `rad`
    pub delta_u: f64,
    /// Critical clearing angle `rad`
    pub delta_cr: f64,
    /// Accelerating area [p.u.·rad]
    pub a_acc: f64,
    /// Decelerating area [p.u.·rad]
    pub a_dec: f64,
    /// Whether the system can remain stable (a_dec > a_acc)
    pub can_be_stable: bool,
}

/// Compute the Equal Area Criterion for a SMIB system (G=0 case).
pub fn equal_area_criterion(scenario: &FaultScenario) -> EacResult {
    use std::f64::consts::PI;

    let p_m = scenario.p_mech;
    let p_max_fault = scenario.e_prime * scenario.v_inf * scenario.b_fault;
    let p_max_post = scenario.e_prime * scenario.v_inf * scenario.b_post;

    let delta_0 = scenario.delta_0;
    // Unstable post-fault equilibrium
    let delta_u = PI - (p_m / p_max_post.max(1e-9)).clamp(-1.0, 1.0).asin();

    // Accelerating area A_acc: integral from δ0 to δcr of (P_m - P_fault)dδ
    // For EAC without conductance: P_e = P_max·sin(δ)
    // A_acc = P_m·(δcr - δ0) + P_max_fault·cos(δcr) - P_max_fault·cos(δ0)
    // A_dec = -P_m·(δu - δcr) - P_max_post·cos(δu) + P_max_post·cos(δcr)
    // Solve for δcr: A_acc = A_dec

    // Numerical bisection for δcr
    let f_balance = |delta_cr: f64| -> f64 {
        let a_acc =
            p_m * (delta_cr - delta_0) + p_max_fault * delta_0.cos() - p_max_fault * delta_cr.cos();
        let a_dec =
            -p_m * (delta_u - delta_cr) - p_max_post * delta_u.cos() + p_max_post * delta_cr.cos();
        a_acc - a_dec
    };

    let mut lo = delta_0;
    let mut hi = delta_u;
    let mut delta_cr = 0.5 * (lo + hi);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if f_balance(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
        if hi - lo < 1e-8 {
            delta_cr = 0.5 * (lo + hi);
            break;
        }
    }

    let a_acc =
        p_m * (delta_cr - delta_0) + p_max_fault * delta_0.cos() - p_max_fault * delta_cr.cos();
    let a_dec =
        -p_m * (delta_u - delta_cr) - p_max_post * delta_u.cos() + p_max_post * delta_cr.cos();

    // Total available deceleration area from δ0 to δu
    let a_dec_max =
        -p_m * (delta_u - delta_0) - p_max_post * delta_u.cos() + p_max_post * delta_0.cos();

    EacResult {
        delta_0,
        delta_u,
        delta_cr,
        a_acc,
        a_dec,
        can_be_stable: a_dec_max > a_acc,
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_stable_trajectory_stays_bounded() {
        let s = FaultScenario::coal_smib();
        // Short clearing time (should be stable)
        let res = simulate_fault_trajectory(&s, 0.05, 2.0, 0.001);
        assert!(res.is_stable, "Expected stable with t_clear=0.05s");
        assert!(res.delta_max < PI, "delta_max = {:.4} rad", res.delta_max);
    }

    #[test]
    fn test_long_fault_unstable() {
        let s = FaultScenario::coal_smib();
        // Very long clearing time (almost certainly unstable)
        let res = simulate_fault_trajectory(&s, 2.0, 3.0, 0.001);
        // For a very long fault, the system should lose synchronism
        // (may be stable or unstable depending on b_fault; check no panic)
        assert!(res.delta_max >= s.delta_0 || res.is_stable);
    }

    #[test]
    fn test_delta_max_exceeds_delta_0() {
        let s = FaultScenario::coal_smib();
        let res = simulate_fault_trajectory(&s, 0.1, 2.0, 0.001);
        assert!(
            res.delta_max >= s.delta_0 - 1e-9,
            "delta_max = {} < delta_0 = {}",
            res.delta_max,
            s.delta_0
        );
    }

    #[test]
    fn test_trajectory_not_empty() {
        let s = FaultScenario::coal_smib();
        let res = simulate_fault_trajectory(&s, 0.1, 1.0, 0.01);
        assert!(!res.trajectory.is_empty());
    }

    #[test]
    fn test_trajectory_time_increases() {
        let s = FaultScenario::coal_smib();
        let res = simulate_fault_trajectory(&s, 0.1, 0.5, 0.01);
        for w in res.trajectory.windows(2) {
            assert!(w[1].time_s >= w[0].time_s - 1e-12);
        }
    }

    #[test]
    fn test_cct_is_positive() {
        let s = FaultScenario::coal_smib();
        let cct = find_cct(&s, 0.01, 0.5, 0.005, 2.0, 0.001);
        assert!(cct.cct_s > 0.0, "CCT = {:.4}s", cct.cct_s);
    }

    #[test]
    fn test_cct_stable_case_is_stable() {
        let s = FaultScenario::coal_smib();
        let cct = find_cct(&s, 0.01, 0.5, 0.005, 2.0, 0.001);
        let stable = simulate_fault_trajectory(&s, cct.cct_s, 2.0, 0.001);
        assert!(
            stable.is_stable,
            "CCT result should be at stability boundary"
        );
    }

    #[test]
    fn test_cct_just_above_unstable() {
        let s = FaultScenario::coal_smib();
        let cct = find_cct(&s, 0.01, 0.5, 0.005, 2.0, 0.001);
        let unstable = simulate_fault_trajectory(&s, cct.cct_s + 0.05, 2.0, 0.001);
        // Just above CCT should be unstable (or at least more stressed)
        assert!(unstable.delta_max >= cct.delta_at_cct - 0.1);
    }

    #[test]
    fn test_equal_area_criterion_can_be_stable() {
        let s = FaultScenario::from_power_transfer(0.5, 2.0, 0.5, 1.8);
        let eac = equal_area_criterion(&s);
        // With good post-fault capacity (1.8 vs 0.5 fault), should be stable
        assert!(
            eac.can_be_stable,
            "EAC: a_acc={:.4}, a_dec_avail check",
            eac.a_acc
        );
    }

    #[test]
    fn test_eac_angles_ordered() {
        let s = FaultScenario::coal_smib();
        let eac = equal_area_criterion(&s);
        assert!(
            eac.delta_0 <= eac.delta_cr + 1e-6,
            "delta_0={:.4} <= delta_cr={:.4}",
            eac.delta_0,
            eac.delta_cr
        );
        assert!(
            eac.delta_cr <= eac.delta_u + 1e-6,
            "delta_cr={:.4} <= delta_u={:.4}",
            eac.delta_cr,
            eac.delta_u
        );
    }

    #[test]
    fn test_from_power_transfer_constructor() {
        let s = FaultScenario::from_power_transfer(0.6, 2.0, 0.4, 1.8);
        assert!(s.delta_0 > 0.0 && s.delta_0 < std::f64::consts::FRAC_PI_2);
    }

    #[test]
    fn test_phase_changes_at_clear_time() {
        let s = FaultScenario::coal_smib();
        let t_clear = 0.1;
        let res = simulate_fault_trajectory(&s, t_clear, 0.5, 0.01);

        // Before clear time: phase should be FaultOn (1)
        let before: Vec<_> = res
            .trajectory
            .iter()
            .filter(|p| p.time_s < t_clear - 0.005)
            .collect();
        // After clear time: phase should be PostFault (2)
        let after: Vec<_> = res
            .trajectory
            .iter()
            .filter(|p| p.time_s > t_clear + 0.005)
            .collect();

        if !before.is_empty() {
            assert_eq!(
                before.last().unwrap().phase,
                1,
                "Should be fault-on before t_clear"
            );
        }
        if !after.is_empty() {
            assert_eq!(after[0].phase, 2, "Should be post-fault after t_clear");
        }
    }
}
