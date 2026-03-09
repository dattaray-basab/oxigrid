/// Smart EV charging algorithms: uncontrolled, TOU-optimized, V2G, frequency regulation.
///
/// Implements single-vehicle scheduling using greedy price sorting (TOU),
/// analytic dynamic programming (V2G), and AGC signal tracking (freq. reg).
use crate::error::OxiGridError;
use serde::{Deserialize, Serialize};

/// A single EV charging session at a charger.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EvSession {
    /// Unique vehicle identifier.
    pub vehicle_id: usize,
    /// Arrival time [hours from day start, e.g. 18.0 = 6 PM].
    pub arrival_time: f64,
    /// Departure time [hours from day start].
    pub departure_time: f64,
    /// State of charge on arrival (0–1).
    pub soc_arrival: f64,
    /// Required SoC at departure (0–1).
    pub soc_target: f64,
    /// Usable battery capacity \[kWh\].
    pub battery_kwh: f64,
    /// Maximum AC charge rate \[kW\].
    pub max_charge_kw: f64,
    /// Maximum V2G discharge rate \[kW\] (0.0 = no V2G capability).
    pub max_discharge_kw: f64,
    /// Charge efficiency η_c (energy stored / energy drawn from grid).
    pub eta_charge: f64,
    /// Discharge efficiency η_d (energy delivered to grid / energy drawn from battery).
    pub eta_discharge: f64,
    /// Battery degradation cost per kWh cycled [$/kWh].
    pub degradation_cost: f64,
}

impl Default for EvSession {
    fn default() -> Self {
        Self {
            vehicle_id: 0,
            arrival_time: 18.0,
            departure_time: 7.0 + 24.0, // next morning 7 AM
            soc_arrival: 0.3,
            soc_target: 0.8,
            battery_kwh: 60.0,
            max_charge_kw: 11.0,
            max_discharge_kw: 7.4,
            eta_charge: 0.92,
            eta_discharge: 0.92,
            degradation_cost: 0.05,
        }
    }
}

impl EvSession {
    /// Net energy needed to reach `soc_target` from `soc_arrival` \[kWh\].
    /// Accounts for charge efficiency.
    pub fn energy_needed_kwh(&self) -> f64 {
        let delta_soc = (self.soc_target - self.soc_arrival).max(0.0);
        // Grid must supply energy / eta_c to store enough
        (delta_soc * self.battery_kwh) / self.eta_charge
    }

    /// Total charging window \[hours\].
    pub fn window_hours(&self) -> f64 {
        self.departure_time - self.arrival_time
    }

    /// Maximum energy that can be discharged via V2G without going below soc_target \[kWh\].
    /// This is the "headroom" above soc_target for ancillary services.
    pub fn v2g_headroom_kwh(&self) -> f64 {
        // After reaching soc_target, additional charge above target is headroom
        // We assume V2G draws from the buffer above soc_target
        let soc_upper = 1.0_f64.min(self.soc_arrival.max(self.soc_target));
        (soc_upper - self.soc_target).max(0.0) * self.battery_kwh
    }

    /// Minimum charge time \[hours\] needed to reach soc_target (at max charge rate).
    pub fn min_charge_hours(&self) -> f64 {
        let e_need = self.energy_needed_kwh();
        if self.max_charge_kw > 1e-9 {
            e_need / self.max_charge_kw
        } else {
            f64::INFINITY
        }
    }

    /// Check whether the charging requirement is feasible given the window.
    pub fn is_feasible(&self) -> bool {
        self.window_hours() >= self.min_charge_hours() - 1e-9
    }
}

/// Smart charging result for one EV session.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingSchedule {
    /// Vehicle identifier.
    pub vehicle_id: usize,
    /// Time at the start of each slot [hours from day start].
    pub time_slots: Vec<f64>,
    /// Power \[kW\] at each slot: positive = charging, negative = V2G discharge.
    pub power_kw: Vec<f64>,
    /// SoC at the *start* of each slot (last element = SoC at departure).
    pub soc_trajectory: Vec<f64>,
    /// Total electricity cost paid [$ = Σ price * max(P,0) * dt].
    pub energy_cost: f64,
    /// Revenue earned from V2G [$ = Σ price * max(-P,0) * dt].
    pub v2g_revenue: f64,
    /// Battery degradation cost [$ = Σ degradation_cost/kwh * |P| * dt].
    pub degradation_cost: f64,
    /// Net cost = energy_cost - v2g_revenue + degradation_cost.
    pub net_cost: f64,
}

/// Smart charger controller for a single vehicle.
///
/// Holds the time resolution, day-ahead price profile, and charger capacity.
/// All algorithms operate on the same grid of `n_slots = 24 / dt_hours` slots.
pub struct SmartCharger {
    /// Time slot duration \[hours\] (default 0.25 = 15 min).
    pub dt_hours: f64,
    /// Day-ahead electricity price [$/kWh] per time slot.
    /// Length must equal `(24.0 / dt_hours).ceil() as usize`.
    pub price_profile: Vec<f64>,
    /// Maximum grid import/export power at this charger \[kW\].
    pub grid_capacity_kw: f64,
}

impl SmartCharger {
    /// Construct a `SmartCharger`.
    ///
    /// # Arguments
    /// - `dt_hours`         — slot duration in hours (e.g. 0.25)
    /// - `price_profile`    — $/kWh per slot (length = 24/dt_hours)
    /// - `grid_capacity_kw` — max charger power \[kW\]
    pub fn new(dt_hours: f64, price_profile: Vec<f64>, grid_capacity_kw: f64) -> Self {
        Self {
            dt_hours,
            price_profile,
            grid_capacity_kw,
        }
    }

    /// Total number of time slots in the price profile.
    pub fn n_slots(&self) -> usize {
        self.price_profile.len()
    }

    /// Convert a time \[hours\] to a slot index (clamped to valid range).
    fn time_to_slot(&self, t_hours: f64) -> usize {
        let idx = (t_hours / self.dt_hours).floor() as isize;
        idx.max(0).min(self.n_slots() as isize - 1) as usize
    }

    /// Collect slots that fall inside [arrival, departure).
    pub fn session_slots(&self, session: &EvSession) -> Vec<usize> {
        let start = self.time_to_slot(session.arrival_time);
        let end = self.time_to_slot(session.departure_time);
        (start..end.min(self.n_slots())).collect()
    }

    /// Compute metrics (cost, revenue, degradation) from a power vector and SoC trajectory.
    pub fn compute_metrics(
        &self,
        power_kw: &[f64],
        slot_indices: &[usize],
        degradation_cost_per_kwh: f64,
        dt: f64,
    ) -> (f64, f64, f64) {
        let mut energy_cost = 0.0_f64;
        let mut v2g_rev = 0.0_f64;
        let mut deg_cost = 0.0_f64;
        for (k, &p) in power_kw.iter().enumerate() {
            let slot_idx = slot_indices[k];
            let price = self.price_profile.get(slot_idx).copied().unwrap_or(0.0);
            if p > 0.0 {
                energy_cost += price * p * dt;
            } else if p < 0.0 {
                v2g_rev += price * p.abs() * dt;
            }
            deg_cost += degradation_cost_per_kwh * p.abs() * dt;
        }
        (energy_cost, v2g_rev, deg_cost)
    }

    /// Simulate SoC forward given power schedule.
    /// Returns SoC trajectory of length `n+1` (SoC at start of each slot + final).
    pub fn simulate_soc(
        &self,
        power_kw: &[f64],
        soc_init: f64,
        battery_kwh: f64,
        eta_charge: f64,
        eta_discharge: f64,
    ) -> Vec<f64> {
        let mut soc = soc_init;
        let mut traj = Vec::with_capacity(power_kw.len() + 1);
        traj.push(soc);
        for &p in power_kw {
            let delta_soc = if p > 0.0 {
                // Charging: store η_c * P * dt / E_batt
                eta_charge * p * self.dt_hours / battery_kwh
            } else {
                // Discharging: draw P * dt / (η_d * E_batt) from battery
                p * self.dt_hours / (eta_discharge * battery_kwh)
            };
            soc = (soc + delta_soc).clamp(0.0, 1.0);
            traj.push(soc);
        }
        traj
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Public charging strategies
    // ──────────────────────────────────────────────────────────────────────────

    /// **Uncontrolled (dumb) charging** — charge at maximum rate from arrival until target is met.
    ///
    /// The vehicle charges at `min(max_charge_kw, grid_capacity_kw)` every slot
    /// until `soc_target` is reached, then idles.
    pub fn uncontrolled(&self, session: &EvSession) -> Result<ChargingSchedule, OxiGridError> {
        if !session.is_feasible() {
            return Err(OxiGridError::InvalidParameter(format!(
                "EV {}: infeasible — need {:.2} h but window is {:.2} h",
                session.vehicle_id,
                session.min_charge_hours(),
                session.window_hours()
            )));
        }

        let slots = self.session_slots(session);
        if slots.is_empty() {
            return Err(OxiGridError::InvalidParameter(format!(
                "EV {}: no time slots in charging window [{:.2}, {:.2})",
                session.vehicle_id, session.arrival_time, session.departure_time
            )));
        }

        let n = slots.len();
        let p_max = session.max_charge_kw.min(self.grid_capacity_kw);
        let mut power_kw = vec![0.0_f64; n];
        let mut soc = session.soc_arrival;

        #[allow(clippy::needless_range_loop)]
        for k in 0..n {
            if soc >= session.soc_target - 1e-9 {
                break;
            }
            // Maximum energy we can add this slot
            let soc_headroom = (session.soc_target - soc).max(0.0);
            let e_max_soc = soc_headroom * session.battery_kwh / session.eta_charge;
            let p_slot = p_max.min(e_max_soc / self.dt_hours);
            power_kw[k] = p_slot;
            soc += session.eta_charge * p_slot * self.dt_hours / session.battery_kwh;
            soc = soc.clamp(0.0, 1.0);
        }

        let soc_traj = self.simulate_soc(
            &power_kw,
            session.soc_arrival,
            session.battery_kwh,
            session.eta_charge,
            session.eta_discharge,
        );

        let time_slots: Vec<f64> = slots.iter().map(|&i| i as f64 * self.dt_hours).collect();
        let (ec, vr, dc) =
            self.compute_metrics(&power_kw, &slots, session.degradation_cost, self.dt_hours);

        Ok(ChargingSchedule {
            vehicle_id: session.vehicle_id,
            time_slots,
            power_kw,
            soc_trajectory: soc_traj,
            energy_cost: ec,
            v2g_revenue: vr,
            degradation_cost: dc,
            net_cost: ec - vr + dc,
        })
    }

    /// **TOU-optimized charging** — sort available slots by price, charge cheapest first.
    ///
    /// Greedy assignment: fill slots with ascending electricity price until
    /// `soc_target` is met. No V2G.  Returns error if infeasible.
    pub fn tou_optimized(&self, session: &EvSession) -> Result<ChargingSchedule, OxiGridError> {
        if !session.is_feasible() {
            return Err(OxiGridError::InvalidParameter(format!(
                "EV {}: infeasible — need {:.2} h but window is {:.2} h",
                session.vehicle_id,
                session.min_charge_hours(),
                session.window_hours()
            )));
        }

        let slots = self.session_slots(session);
        if slots.is_empty() {
            return Err(OxiGridError::InvalidParameter(format!(
                "EV {}: no slots in window",
                session.vehicle_id
            )));
        }

        let n = slots.len();
        let p_max = session.max_charge_kw.min(self.grid_capacity_kw);
        let dt = self.dt_hours;

        // Sort slot positions by price (ascending) — break ties by earlier slot
        let mut slot_order: Vec<usize> = (0..n).collect();
        slot_order.sort_by(|&a, &b| {
            let pa = self.price_profile.get(slots[a]).copied().unwrap_or(0.0);
            let pb = self.price_profile.get(slots[b]).copied().unwrap_or(0.0);
            pa.partial_cmp(&pb)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a.cmp(&b))
        });

        let mut power_kw = vec![0.0_f64; n];
        // Track cumulative energy stored (to know when target is reached)
        let e_needed = session.energy_needed_kwh(); // grid-side kWh needed
        let mut e_assigned = 0.0_f64;

        for &k in &slot_order {
            if e_assigned >= e_needed - 1e-9 {
                break;
            }
            let e_remaining = e_needed - e_assigned;
            let e_slot_max = p_max * dt; // max grid-side energy this slot
            let e_this = e_slot_max.min(e_remaining);
            power_kw[k] = e_this / dt;
            e_assigned += e_this;
        }

        // Verify feasibility (energy-wise, should be guaranteed by is_feasible)
        if e_assigned < e_needed - 1e-6 {
            return Err(OxiGridError::InvalidParameter(format!(
                "EV {}: TOU scheduling failed to assign enough energy",
                session.vehicle_id
            )));
        }

        let soc_traj = self.simulate_soc(
            &power_kw,
            session.soc_arrival,
            session.battery_kwh,
            session.eta_charge,
            session.eta_discharge,
        );
        let time_slots: Vec<f64> = slots.iter().map(|&i| i as f64 * dt).collect();
        let (ec, vr, dc) = self.compute_metrics(&power_kw, &slots, session.degradation_cost, dt);

        Ok(ChargingSchedule {
            vehicle_id: session.vehicle_id,
            time_slots,
            power_kw,
            soc_trajectory: soc_traj,
            energy_cost: ec,
            v2g_revenue: vr,
            degradation_cost: dc,
            net_cost: ec - vr + dc,
        })
    }

    /// **V2G bi-directional optimization** via analytic Dynamic Programming.
    ///
    /// Minimises:
    ///   Σ_t [ price\[t\] * P\[t\] * dt  +  degradation * |P\[t\]| * dt ]
    ///
    /// subject to SoC dynamics, box constraints on P\[t\] and SoC\[t\],
    /// and the terminal constraint SoC\[T_dep\] ≥ soc_target.
    ///
    /// # Algorithm
    /// Single-vehicle 1-D DP: backward pass computes optimal cost-to-go from
    /// each state; forward pass reconstructs optimal power at each slot.
    /// SoC is discretised into `N_SOC = 200` bins for tractability.
    pub fn v2g_optimized(&self, session: &EvSession) -> Result<ChargingSchedule, OxiGridError> {
        if !session.is_feasible() {
            return Err(OxiGridError::InvalidParameter(format!(
                "EV {}: infeasible — need {:.2} h but window is {:.2} h",
                session.vehicle_id,
                session.min_charge_hours(),
                session.window_hours()
            )));
        }

        let slots = self.session_slots(session);
        if slots.is_empty() {
            return Err(OxiGridError::InvalidParameter(format!(
                "EV {}: no slots in window",
                session.vehicle_id
            )));
        }

        let n = slots.len();
        let dt = self.dt_hours;
        let e_batt = session.battery_kwh;
        let eta_c = session.eta_charge;
        let eta_d = session.eta_discharge;
        let p_chg = session.max_charge_kw.min(self.grid_capacity_kw);
        let p_dis = session.max_discharge_kw.min(self.grid_capacity_kw);
        let deg = session.degradation_cost;

        // SoC discretization
        const N_SOC: usize = 200;
        let soc_min = 0.0_f64;
        let soc_max = 1.0_f64;
        let dsoc = (soc_max - soc_min) / (N_SOC - 1) as f64;

        let soc_idx_of = |s: f64| -> usize {
            let idx = ((s - soc_min) / dsoc).round() as isize;
            idx.max(0).min(N_SOC as isize - 1) as usize
        };

        let soc_of = |i: usize| -> f64 { soc_min + i as f64 * dsoc };

        // Terminal constraint: SoC at step n must be >= soc_target
        let target_idx = soc_idx_of(session.soc_target);

        // Cost-to-go: V[t][s] = minimum cost from slot t to end, given SoC = soc_of(s)
        let inf = f64::INFINITY;
        let mut v_next = vec![inf; N_SOC];

        // Terminal cost: 0 if SoC >= target, else infinity
        for (s, v) in v_next.iter_mut().enumerate() {
            if s >= target_idx {
                *v = 0.0;
            }
        }

        // Discretized power actions: charge levels + discharge levels + idle
        // Use 20 actions per direction for resolution
        const N_ACT: usize = 41; // -20..=+20 relative to p_max
        let actions: Vec<f64> = (0..N_ACT)
            .map(|i| {
                let frac = i as f64 / (N_ACT - 1) as f64; // 0..1
                -p_dis + frac * (p_chg + p_dis) // -p_dis .. +p_chg
            })
            .collect();

        // Backward DP
        let mut v_table: Vec<Vec<f64>> = vec![vec![inf; N_SOC]; n + 1];
        v_table[n] = v_next.clone();

        for t_back in (0..n).rev() {
            let slot_idx = slots[t_back];
            let price = self.price_profile.get(slot_idx).copied().unwrap_or(0.0);
            let mut v_cur = vec![inf; N_SOC];

            for (s_cur, v_slot) in v_cur.iter_mut().enumerate() {
                let soc_cur = soc_of(s_cur);
                let mut best = inf;

                for &p in &actions {
                    // SoC transition
                    let delta_soc = if p >= 0.0 {
                        eta_c * p * dt / e_batt
                    } else {
                        p * dt / (eta_d * e_batt) // negative
                    };
                    let soc_next = soc_cur + delta_soc;
                    if soc_next < soc_min - 1e-9 || soc_next > soc_max + 1e-9 {
                        continue; // infeasible transition
                    }
                    let s_next = soc_idx_of(soc_next.clamp(soc_min, soc_max));
                    let v_fut = v_table[t_back + 1][s_next];
                    if v_fut >= inf {
                        continue;
                    }

                    // Immediate cost: grid cost + degradation
                    let grid_cost = if p >= 0.0 {
                        price * p * dt
                    } else {
                        -price * p.abs() * dt // revenue (negative cost)
                    };
                    let deg_cost = deg * p.abs() * dt;
                    let total_cost = grid_cost + deg_cost + v_fut;

                    if total_cost < best {
                        best = total_cost;
                    }
                }
                *v_slot = best;
            }
            v_table[t_back] = v_cur;
        }

        // Forward pass: reconstruct optimal actions
        let mut power_kw = vec![0.0_f64; n];
        let mut soc_cur = session.soc_arrival;

        for t in 0..n {
            let slot_idx = slots[t];
            let price = self.price_profile.get(slot_idx).copied().unwrap_or(0.0);
            let _s_cur = soc_idx_of(soc_cur);

            let mut best_cost = inf;
            let mut best_p = 0.0_f64;

            for &p in &actions {
                let delta_soc = if p >= 0.0 {
                    eta_c * p * dt / e_batt
                } else {
                    p * dt / (eta_d * e_batt)
                };
                let soc_next = soc_cur + delta_soc;
                if soc_next < soc_min - 1e-9 || soc_next > soc_max + 1e-9 {
                    continue;
                }
                let s_next = soc_idx_of(soc_next.clamp(soc_min, soc_max));
                let v_fut = v_table[t + 1][s_next];
                if v_fut >= inf {
                    continue;
                }

                let grid_cost = if p >= 0.0 {
                    price * p * dt
                } else {
                    -price * p.abs() * dt
                };
                let deg_cost = deg * p.abs() * dt;
                let total = grid_cost + deg_cost + v_fut;

                if total < best_cost {
                    best_cost = total;
                    best_p = p;
                }
            }

            // If still infeasible at current state, fall back to max charge
            if best_cost >= inf {
                best_p = p_chg;
            }

            power_kw[t] = best_p;
            let delta = if best_p >= 0.0 {
                eta_c * best_p * dt / e_batt
            } else {
                best_p * dt / (eta_d * e_batt)
            };
            soc_cur = (soc_cur + delta).clamp(0.0, 1.0);
        }

        let soc_traj = self.simulate_soc(
            &power_kw,
            session.soc_arrival,
            session.battery_kwh,
            eta_c,
            eta_d,
        );
        let time_slots: Vec<f64> = slots.iter().map(|&i| i as f64 * dt).collect();
        let (ec, vr, dc) = self.compute_metrics(&power_kw, &slots, session.degradation_cost, dt);

        Ok(ChargingSchedule {
            vehicle_id: session.vehicle_id,
            time_slots,
            power_kw,
            soc_trajectory: soc_traj,
            energy_cost: ec,
            v2g_revenue: vr,
            degradation_cost: dc,
            net_cost: ec - vr + dc,
        })
    }

    /// **Frequency regulation** — modulate charging around a base schedule
    /// following an Automatic Generation Control (AGC) signal.
    ///
    /// The AGC signal is normalised to [−1, 1]; ±1 means full power adjustment
    /// from the base schedule.  SoC is kept within [soc_min=0.2, soc_max=0.95].
    ///
    /// # Arguments
    /// - `session`           — the EV session parameters
    /// - `regulation_signal` — AGC signal per slot (normalised, len = n_slots in window)
    /// - `base_schedule`     — baseline schedule (typically from `tou_optimized`)
    pub fn frequency_regulation(
        &self,
        session: &EvSession,
        regulation_signal: &[f64],
        base_schedule: &ChargingSchedule,
    ) -> Result<ChargingSchedule, OxiGridError> {
        let slots = self.session_slots(session);
        let n = slots.len();

        if base_schedule.power_kw.len() != n {
            return Err(OxiGridError::InvalidParameter(format!(
                "EV {}: base schedule length {} != window slots {}",
                session.vehicle_id,
                base_schedule.power_kw.len(),
                n
            )));
        }
        if regulation_signal.len() < n {
            return Err(OxiGridError::InvalidParameter(format!(
                "EV {}: regulation_signal length {} < slots {}",
                session.vehicle_id,
                regulation_signal.len(),
                n
            )));
        }

        let soc_reg_min = 0.2_f64;
        let soc_reg_max = 0.95_f64;
        let dt = self.dt_hours;
        let e_batt = session.battery_kwh;
        let eta_c = session.eta_charge;
        let eta_d = session.eta_discharge;
        let p_chg = session.max_charge_kw.min(self.grid_capacity_kw);
        let p_dis = session.max_discharge_kw.min(self.grid_capacity_kw);

        let mut power_kw = Vec::with_capacity(n);
        let mut soc = session.soc_arrival;

        for (base_p, &sig) in base_schedule
            .power_kw
            .iter()
            .zip(regulation_signal.iter())
            .take(n)
        {
            let base_p = *base_p;
            let signal = sig.clamp(-1.0, 1.0);

            // Modulation band: signal > 0 → increase charging; signal < 0 → decrease/discharge
            let adjust = if signal >= 0.0 {
                signal * (p_chg - base_p).max(0.0)
            } else {
                signal * (base_p + p_dis).max(0.0) // reduce / go negative
            };

            let p_candidate = (base_p + adjust).clamp(-p_dis, p_chg);

            // Enforce SoC constraints: if SoC too low, prevent further discharge;
            // if SoC too high, prevent further charge
            let p_actual = if (soc <= soc_reg_min + 1e-9 && p_candidate < 0.0)
                || (soc >= soc_reg_max - 1e-9 && p_candidate > 0.0)
            {
                0.0
            } else {
                p_candidate
            };

            power_kw.push(p_actual);

            let delta = if p_actual >= 0.0 {
                eta_c * p_actual * dt / e_batt
            } else {
                p_actual * dt / (eta_d * e_batt)
            };
            soc = (soc + delta).clamp(0.0, 1.0);
        }

        // Check that we still end near soc_target (best-effort)
        let soc_traj = self.simulate_soc(&power_kw, session.soc_arrival, e_batt, eta_c, eta_d);
        let time_slots: Vec<f64> = slots.iter().map(|&i| i as f64 * dt).collect();
        let (ec, vr, dc) = self.compute_metrics(&power_kw, &slots, session.degradation_cost, dt);

        Ok(ChargingSchedule {
            vehicle_id: session.vehicle_id,
            time_slots,
            power_kw,
            soc_trajectory: soc_traj,
            energy_cost: ec,
            v2g_revenue: vr,
            degradation_cost: dc,
            net_cost: ec - vr + dc,
        })
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Unit tests
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_charger() -> SmartCharger {
        // 15-min slots, 96 per day
        // Cheap at night (slots 0-31 = 00:00-08:00 and 76-95 = 19:00-24:00)
        // Expensive during day (slots 32-75)
        let n_slots = 96usize;
        let prices: Vec<f64> = (0..n_slots)
            .map(|i| {
                if !(32..76).contains(&i) {
                    0.05_f64 // cheap night
                } else {
                    0.25_f64 // expensive day
                }
            })
            .collect();
        SmartCharger::new(0.25, prices, 22.0)
    }

    fn make_session(arrival: f64, departure: f64) -> EvSession {
        EvSession {
            vehicle_id: 1,
            arrival_time: arrival,
            departure_time: departure,
            soc_arrival: 0.3,
            soc_target: 0.8,
            battery_kwh: 60.0,
            max_charge_kw: 11.0,
            max_discharge_kw: 7.4,
            eta_charge: 0.92,
            eta_discharge: 0.92,
            degradation_cost: 0.05,
        }
    }

    #[test]
    fn test_ev_session_energy_needed() {
        let session = EvSession {
            soc_arrival: 0.3,
            soc_target: 0.8,
            battery_kwh: 60.0,
            eta_charge: 1.0, // perfect efficiency for simple math
            ..Default::default()
        };
        // (0.8 - 0.3) * 60.0 / 1.0 = 30 kWh
        assert!((session.energy_needed_kwh() - 30.0).abs() < 0.01);
    }

    #[test]
    fn test_uncontrolled_charging_meets_target() {
        let charger = make_charger();
        let session = make_session(18.0, 31.0); // 18:00 → next morning 07:00 (31h window)
        let sched = charger
            .uncontrolled(&session)
            .expect("uncontrolled should succeed");
        let final_soc = *sched.soc_trajectory.last().expect("trajectory non-empty");
        assert!(
            final_soc >= session.soc_target - 1e-3,
            "Final SoC {:.4} < target {:.4}",
            final_soc,
            session.soc_target
        );
    }

    #[test]
    fn test_tou_charges_during_cheap_hours() {
        let charger = make_charger();
        // Arrive at 08:00 (slot 32, start of expensive), depart 31h later
        let session = make_session(8.0, 31.0);
        let sched = charger.tou_optimized(&session).expect("tou should succeed");

        // Count energy assigned to cheap slots (price=0.05) vs expensive (price=0.25)
        let dt = 0.25_f64;
        let mut cheap_kwh = 0.0_f64;
        let mut exp_kwh = 0.0_f64;
        for (k, &p) in sched.power_kw.iter().enumerate() {
            if p <= 0.0 {
                continue;
            }
            let t = sched.time_slots[k];
            let slot = (t / dt).round() as usize;
            if !(32..76).contains(&slot) {
                cheap_kwh += p * dt;
            } else {
                exp_kwh += p * dt;
            }
        }
        assert!(
            cheap_kwh > exp_kwh,
            "TOU should prefer cheap slots: cheap={:.2} exp={:.2}",
            cheap_kwh,
            exp_kwh
        );
    }

    #[test]
    fn test_soc_trajectory_bounded() {
        let charger = make_charger();
        let session = make_session(18.0, 31.0);
        let sched = charger.v2g_optimized(&session).expect("v2g should succeed");
        for &s in &sched.soc_trajectory {
            assert!(
                (-1e-6..=1.0 + 1e-6).contains(&s),
                "SoC out of bounds: {:.4}",
                s
            );
        }
    }

    #[test]
    fn test_charging_window_feasibility() {
        let charger = make_charger();
        // 5 min window (0.083h), need >> that to charge from 0.3→0.8 on 60kWh bat
        let session = make_session(8.0, 8.083);
        let result = charger.uncontrolled(&session);
        assert!(result.is_err(), "Should return error for infeasible window");
    }

    #[test]
    fn test_frequency_regulation_soc_bounded() {
        let charger = make_charger();
        let session = make_session(18.0, 31.0);
        let base = charger.tou_optimized(&session).expect("tou base");
        let n = base.power_kw.len();
        // Aggressive regulation signal: alternating +1 and -1
        let signal: Vec<f64> = (0..n)
            .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let sched = charger
            .frequency_regulation(&session, &signal, &base)
            .expect("freq reg");
        for &s in &sched.soc_trajectory {
            assert!((-1e-6..=1.0 + 1e-6).contains(&s), "SoC OOB: {:.4}", s);
        }
    }
}
