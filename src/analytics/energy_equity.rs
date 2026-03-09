//! Energy poverty and equity analysis for power systems.
//!
//! This module provides metrics and tools to assess energy affordability,
//! equity, and the effectiveness of policy interventions targeting
//! energy-burdened households.
//!
//! # Key Metrics
//!
//! - **Energy Burden** — annual energy cost as a fraction of annual income
//! - **Gini Coefficient** — inequality of energy cost distribution (0 = equal, 1 = maximal)
//! - **Affordability Index** — composite measure of energy accessibility
//! - **Environmental Justice Index** — 0–100 score (higher = more equitable)
//!
//! # Reference
//!
//! - DOE / ACEEE energy burden definitions
//! - US DOE LEAD tool methodology
//! - EPA EJScreen indices

use thiserror::Error;

/// Errors from equity analysis.
#[derive(Debug, Error)]
pub enum EquityError {
    #[error("No household groups added")]
    NoGroups,
    #[error("Invalid income value: {0}")]
    InvalidIncome(String),
    #[error("Invalid energy cost: {0}")]
    InvalidCost(String),
    #[error("Insufficient data: {0}")]
    InsufficientData(String),
}

// ── Configuration ────────────────────────────────────────────────────────────

/// Configuration for an energy equity analysis.
#[derive(Debug, Clone)]
pub struct EnergyEquityConfig {
    /// Median household income for the study area \[USD/year\].
    pub median_household_income_usd: f64,
    /// Energy burden threshold above which a household is considered high-burden (e.g. 0.06 = 6 %).
    pub energy_burden_threshold_pct: f64,
    /// Method used to compute the affordability index.
    pub affordability_index_method: AffordabilityMethod,
}

impl Default for EnergyEquityConfig {
    fn default() -> Self {
        Self {
            median_household_income_usd: 60_000.0,
            energy_burden_threshold_pct: 0.06,
            affordability_index_method: AffordabilityMethod::EnergyBurden,
        }
    }
}

/// Method used to compute the affordability index.
#[derive(Debug, Clone, PartialEq)]
pub enum AffordabilityMethod {
    /// Annual energy cost divided by annual income.
    EnergyBurden,
    /// Low Income Home Energy Assistance Programme metric.
    Lihwap,
    /// Inability-to-pay metric (binary proxy from burden > threshold).
    EnergyInsecurity,
    /// Gini coefficient of energy-cost distribution.
    GiniCoefficient,
}

// ── Household groups ─────────────────────────────────────────────────────────

/// Housing tenure / dwelling type.
#[derive(Debug, Clone, PartialEq)]
pub enum HousingType {
    OwnedSingleFamily,
    RentedApartment,
    MobileHome,
    PublicHousing,
    Rural,
}

/// A cohort of households sharing similar socio-economic characteristics.
#[derive(Debug, Clone)]
pub struct HouseholdGroup {
    /// Unique group identifier.
    pub id: usize,
    /// Human-readable name (e.g. "Low-income renters").
    pub name: String,
    /// Number of households in this group.
    pub n_households: usize,
    /// Average annual household income \[USD\].
    pub avg_income_usd: f64,
    /// Average annual energy expenditure \[USD\].
    pub avg_annual_energy_cost_usd: f64,
    /// Average annual electricity + thermal consumption \[kWh\].
    pub avg_consumption_kwh: f64,
    /// Dwelling type.
    pub housing_type: HousingType,
    /// Whether the group has meaningful access to rooftop/community solar.
    pub has_renewable_access: bool,
    /// Whether the group has access to behind-the-meter battery storage.
    pub has_battery_storage: bool,
    /// Geographic area label.
    pub geographic_area: String,
}

// ── Output types ─────────────────────────────────────────────────────────────

/// Aggregate equity metrics for a collection of household groups.
#[derive(Debug, Clone)]
pub struct EquityMetrics {
    /// Population-weighted average energy burden (annual cost / annual income).
    pub energy_burden_pct: f64,
    /// Fraction of households above the high-burden threshold \[0–1\].
    pub high_burden_households_pct: f64,
    /// Gini coefficient of energy cost distribution (0 = equal, 1 = maximally unequal).
    pub gini_coefficient: f64,
    /// Affordability index \[0–1\]; 1 = fully affordable.
    pub affordability_index: f64,
    /// Fraction of low-income households with renewable energy access \[0–1\].
    pub renewable_access_equity_pct: f64,
    /// Difference in outage vulnerability between low- and high-income groups \[hours/year\].
    pub resilience_gap: f64,
    /// Environmental justice index \[0–100\]; higher = more equitable.
    pub environmental_justice_index: f64,
}

/// A policy intervention targeting one or more household groups.
#[derive(Debug, Clone)]
pub struct PolicyIntervention {
    /// Name of the programme (e.g. "LIHEAP subsidy").
    pub name: String,
    /// Target household group description.
    pub target_group: String,
    /// Average subsidy per household \[USD/year\].
    pub subsidy_usd_per_household: f64,
    /// Number of households that would benefit.
    pub n_households_benefited: usize,
    /// Cost-effectiveness of energy savings \[USD/kWh\].
    pub cost_effectiveness_usd_per_kwh_saved: f64,
    /// Expected improvement in the Gini coefficient (negative = more equal).
    pub equity_improvement: f64,
    /// Expected increase in renewable penetration among targeted group \[percentage points\].
    pub renewable_penetration_increase_pct: f64,
}

/// Full result of an energy equity analysis.
#[derive(Debug, Clone)]
pub struct EnergyEquityResult {
    /// Aggregate equity metrics.
    pub metrics: EquityMetrics,
    /// IDs of the groups with the highest energy burden.
    pub worst_affected_groups: Vec<usize>,
    /// Ranked list of recommended policy interventions.
    pub best_interventions: Vec<PolicyIntervention>,
    /// Absolute count of high-burden households.
    pub total_high_burden_households: usize,
    /// Total annual monetary cost of energy poverty across all groups \[USD\].
    pub annual_energy_poverty_cost_usd: f64,
    /// Overall equity score \[0–100\]; higher = more equitable.
    pub equity_score: f64,
}

// ── Analyzer ─────────────────────────────────────────────────────────────────

/// Performs energy poverty and equity analysis over a population of household groups.
pub struct EnergyEquityAnalyzer {
    config: EnergyEquityConfig,
    groups: Vec<HouseholdGroup>,
}

impl EnergyEquityAnalyzer {
    /// Create a new analyzer with the given configuration.
    pub fn new(config: EnergyEquityConfig) -> Self {
        Self {
            config,
            groups: Vec::new(),
        }
    }

    /// Add a household group to the analysis population.
    pub fn add_group(&mut self, group: HouseholdGroup) {
        self.groups.push(group);
    }

    /// Run the full equity analysis and return results.
    pub fn analyze(&self) -> Result<EnergyEquityResult, EquityError> {
        if self.groups.is_empty() {
            return Err(EquityError::NoGroups);
        }
        for g in &self.groups {
            if g.avg_income_usd < 0.0 {
                return Err(EquityError::InvalidIncome(g.name.clone()));
            }
            if g.avg_annual_energy_cost_usd < 0.0 {
                return Err(EquityError::InvalidCost(g.name.clone()));
            }
        }

        let metrics = self.compute_metrics()?;
        let worst_affected = self.identify_worst_affected(&metrics);
        let interventions = self.generate_interventions(&metrics);

        let total_high_burden = self.total_high_burden_households();
        let poverty_cost = self.annual_poverty_cost();
        let equity_score = self.equity_score(&metrics);

        Ok(EnergyEquityResult {
            metrics,
            worst_affected_groups: worst_affected,
            best_interventions: interventions,
            total_high_burden_households: total_high_burden,
            annual_energy_poverty_cost_usd: poverty_cost,
            equity_score,
        })
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    fn compute_metrics(&self) -> Result<EquityMetrics, EquityError> {
        let total_hh: usize = self.groups.iter().map(|g| g.n_households).sum();
        if total_hh == 0 {
            return Err(EquityError::InsufficientData(
                "Total household count is zero".into(),
            ));
        }
        let total_hh_f = total_hh as f64;

        // Population-weighted average energy burden
        let weighted_burden: f64 = self
            .groups
            .iter()
            .map(|g| {
                let burden = if g.avg_income_usd > 0.0 {
                    g.avg_annual_energy_cost_usd / g.avg_income_usd
                } else {
                    1.0
                };
                burden * g.n_households as f64
            })
            .sum();
        let energy_burden_pct = weighted_burden / total_hh_f;

        // High-burden fraction
        let high_burden_hh: usize = self
            .groups
            .iter()
            .map(|g| {
                let burden = if g.avg_income_usd > 0.0 {
                    g.avg_annual_energy_cost_usd / g.avg_income_usd
                } else {
                    1.0
                };
                if burden > self.config.energy_burden_threshold_pct {
                    g.n_households
                } else {
                    0
                }
            })
            .sum();
        let high_burden_households_pct = high_burden_hh as f64 / total_hh_f;

        // Collect individual energy costs and weights for Gini
        let costs: Vec<f64> = self
            .groups
            .iter()
            .map(|g| g.avg_annual_energy_cost_usd)
            .collect();
        let weights: Vec<usize> = self.groups.iter().map(|g| g.n_households).collect();
        let gini_coefficient = self.gini_coefficient(&costs, &weights);

        // Affordability index
        let affordability_index = self.affordability_index(energy_burden_pct);

        // Renewable access equity — fraction of low-income groups with renewable access
        let low_income_median = self.config.median_household_income_usd * 0.8;
        let (low_income_with_re, low_income_total): (usize, usize) = self
            .groups
            .iter()
            .filter(|g| g.avg_income_usd <= low_income_median)
            .fold((0usize, 0usize), |(acc_re, acc_tot), g| {
                let re = if g.has_renewable_access {
                    g.n_households
                } else {
                    0
                };
                (acc_re + re, acc_tot + g.n_households)
            });
        let renewable_access_equity_pct = if low_income_total > 0 {
            low_income_with_re as f64 / low_income_total as f64
        } else {
            0.0
        };

        // Resilience gap — proxy: higher-income groups assumed 2 h/year outage exposure,
        // lower-income groups (mobile home / rural) assumed 8 h/year.
        let resilience_gap = self.compute_resilience_gap();

        // Environmental justice index
        let environmental_justice_index = self.environmental_justice_index(
            energy_burden_pct,
            gini_coefficient,
            renewable_access_equity_pct,
        );

        Ok(EquityMetrics {
            energy_burden_pct,
            high_burden_households_pct,
            gini_coefficient,
            affordability_index,
            renewable_access_equity_pct,
            resilience_gap,
            environmental_justice_index,
        })
    }

    /// Compute the Gini coefficient from energy cost observations with household weights.
    ///
    /// Sort by cost, build the weighted Lorenz curve, and return `G = 1 − 2 × area`.
    fn gini_coefficient(&self, costs: &[f64], weights: &[usize]) -> f64 {
        if costs.is_empty() {
            return 0.0;
        }
        // Build (cost, weight) pairs and sort ascending by cost
        let mut pairs: Vec<(f64, f64)> = costs
            .iter()
            .zip(weights.iter())
            .map(|(&c, &w)| (c, w as f64))
            .collect();
        pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let total_weight: f64 = pairs.iter().map(|(_, w)| w).sum();
        let total_cost: f64 = pairs.iter().map(|(c, w)| c * w).sum();

        if total_weight <= 0.0 || total_cost <= 0.0 {
            return 0.0;
        }

        // Lorenz curve: cumulative population fraction vs cumulative income fraction
        // G = 1 - sum_i (x_i - x_{i-1}) * (L_i + L_{i-1})
        let mut lorenz_area = 0.0_f64;
        let mut cum_weight = 0.0_f64;
        let mut cum_cost = 0.0_f64;
        let mut prev_x = 0.0_f64;
        let mut prev_y = 0.0_f64;

        for (cost, weight) in &pairs {
            cum_weight += weight;
            cum_cost += cost * weight;

            let x = cum_weight / total_weight;
            let y = cum_cost / total_cost;

            // Trapezoid area
            lorenz_area += (x - prev_x) * (prev_y + y) / 2.0;
            prev_x = x;
            prev_y = y;
        }

        (1.0 - 2.0 * lorenz_area).clamp(0.0, 1.0)
    }

    fn affordability_index(&self, energy_burden_pct: f64) -> f64 {
        match self.config.affordability_index_method {
            AffordabilityMethod::EnergyBurden => {
                // Index = 1 when burden ≤ threshold; linearly declines above
                let ratio = energy_burden_pct / self.config.energy_burden_threshold_pct;
                (1.0 / ratio).clamp(0.0, 1.0)
            }
            AffordabilityMethod::Lihwap => {
                // Proportion of households below 150 % of poverty line that can afford energy
                let low_pov: usize = self
                    .groups
                    .iter()
                    .filter(|g| g.avg_income_usd < self.config.median_household_income_usd * 0.5)
                    .map(|g| g.n_households)
                    .sum();
                let total: usize = self.groups.iter().map(|g| g.n_households).sum();
                if total == 0 {
                    return 1.0;
                }
                1.0 - low_pov as f64 / total as f64
            }
            AffordabilityMethod::EnergyInsecurity => {
                let high_burden: usize = self
                    .groups
                    .iter()
                    .filter(|g| {
                        g.avg_income_usd > 0.0
                            && g.avg_annual_energy_cost_usd / g.avg_income_usd
                                > self.config.energy_burden_threshold_pct
                    })
                    .map(|g| g.n_households)
                    .sum();
                let total: usize = self.groups.iter().map(|g| g.n_households).sum();
                if total == 0 {
                    return 1.0;
                }
                1.0 - high_burden as f64 / total as f64
            }
            AffordabilityMethod::GiniCoefficient => {
                let costs: Vec<f64> = self
                    .groups
                    .iter()
                    .map(|g| g.avg_annual_energy_cost_usd)
                    .collect();
                let weights: Vec<usize> = self.groups.iter().map(|g| g.n_households).collect();
                1.0 - self.gini_coefficient(&costs, &weights)
            }
        }
    }

    fn compute_resilience_gap(&self) -> f64 {
        // Estimate outage hours per year by housing type (heuristic proxy)
        let outage_hours = |g: &HouseholdGroup| -> f64 {
            match g.housing_type {
                HousingType::OwnedSingleFamily => 2.0,
                HousingType::RentedApartment => 3.0,
                HousingType::MobileHome => 8.0,
                HousingType::PublicHousing => 5.0,
                HousingType::Rural => 6.0,
            }
        };

        let total_hh: f64 = self.groups.iter().map(|g| g.n_households as f64).sum();
        if total_hh <= 0.0 {
            return 0.0;
        }
        let median = self.config.median_household_income_usd;

        let (low_out, low_w): (f64, f64) = self
            .groups
            .iter()
            .filter(|g| g.avg_income_usd < median * 0.8)
            .fold((0.0, 0.0), |(oa, wa), g| {
                let w = g.n_households as f64;
                (oa + outage_hours(g) * w, wa + w)
            });
        let (high_out, high_w): (f64, f64) = self
            .groups
            .iter()
            .filter(|g| g.avg_income_usd >= median * 0.8)
            .fold((0.0, 0.0), |(oa, wa), g| {
                let w = g.n_households as f64;
                (oa + outage_hours(g) * w, wa + w)
            });

        let low_avg = if low_w > 0.0 { low_out / low_w } else { 0.0 };
        let high_avg = if high_w > 0.0 { high_out / high_w } else { 0.0 };
        (low_avg - high_avg).max(0.0)
    }

    fn environmental_justice_index(&self, burden: f64, gini: f64, re_access: f64) -> f64 {
        // Score components, all normalized to 0–1 (1 = most equitable)
        let burden_score =
            (1.0 - burden / (self.config.energy_burden_threshold_pct * 2.0)).clamp(0.0, 1.0);
        let gini_score = 1.0 - gini;
        let re_score = re_access;
        // Weighted composite → 0–100
        (0.4 * burden_score + 0.4 * gini_score + 0.2 * re_score) * 100.0
    }

    fn identify_worst_affected(&self, _metrics: &EquityMetrics) -> Vec<usize> {
        // Rank by energy burden descending; return top 3
        let mut groups_with_burden: Vec<(usize, f64)> = self
            .groups
            .iter()
            .map(|g| {
                let burden = if g.avg_income_usd > 0.0 {
                    g.avg_annual_energy_cost_usd / g.avg_income_usd
                } else {
                    1.0
                };
                (g.id, burden)
            })
            .collect();
        groups_with_burden
            .sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        groups_with_burden
            .into_iter()
            .take(3)
            .map(|(id, _)| id)
            .collect()
    }

    fn total_high_burden_households(&self) -> usize {
        self.groups
            .iter()
            .map(|g| {
                let burden = if g.avg_income_usd > 0.0 {
                    g.avg_annual_energy_cost_usd / g.avg_income_usd
                } else {
                    1.0
                };
                if burden > self.config.energy_burden_threshold_pct {
                    g.n_households
                } else {
                    0
                }
            })
            .sum()
    }

    fn annual_poverty_cost(&self) -> f64 {
        // Cost = excess spend above threshold for high-burden households
        self.groups
            .iter()
            .map(|g| {
                let burden = if g.avg_income_usd > 0.0 {
                    g.avg_annual_energy_cost_usd / g.avg_income_usd
                } else {
                    1.0
                };
                if burden > self.config.energy_burden_threshold_pct {
                    let affordable_cost =
                        g.avg_income_usd * self.config.energy_burden_threshold_pct;
                    let excess = (g.avg_annual_energy_cost_usd - affordable_cost).max(0.0);
                    excess * g.n_households as f64
                } else {
                    0.0
                }
            })
            .sum()
    }

    fn equity_score(&self, metrics: &EquityMetrics) -> f64 {
        metrics.environmental_justice_index
    }

    /// Generate policy interventions targeting the most burdensome groups.
    fn generate_interventions(&self, metrics: &EquityMetrics) -> Vec<PolicyIntervention> {
        let mut interventions = Vec::new();

        // 1. Direct subsidy for high-burden groups
        let high_burden_groups: Vec<&HouseholdGroup> = self
            .groups
            .iter()
            .filter(|g| {
                g.avg_income_usd > 0.0
                    && g.avg_annual_energy_cost_usd / g.avg_income_usd
                        > self.config.energy_burden_threshold_pct
            })
            .collect();

        if !high_burden_groups.is_empty() {
            let total_benefited: usize = high_burden_groups.iter().map(|g| g.n_households).sum();
            let avg_subsidy = high_burden_groups
                .iter()
                .map(|g| {
                    let affordable = g.avg_income_usd * self.config.energy_burden_threshold_pct;
                    (g.avg_annual_energy_cost_usd - affordable).max(0.0)
                })
                .sum::<f64>()
                / high_burden_groups.len() as f64;

            let avg_consumption: f64 = high_burden_groups
                .iter()
                .map(|g| g.avg_consumption_kwh)
                .sum::<f64>()
                / high_burden_groups.len() as f64;
            let cost_eff = if avg_consumption > 0.0 {
                avg_subsidy / avg_consumption
            } else {
                0.0
            };

            // Gini improvement estimate: reducing costs for poorest ≈ lowers Gini by ~5–10 %
            let equity_improvement = -metrics.gini_coefficient * 0.08;

            interventions.push(PolicyIntervention {
                name: "LIHEAP Energy Bill Subsidy".into(),
                target_group: "High energy burden households".into(),
                subsidy_usd_per_household: avg_subsidy,
                n_households_benefited: total_benefited,
                cost_effectiveness_usd_per_kwh_saved: cost_eff,
                equity_improvement,
                renewable_penetration_increase_pct: 0.0,
            });
        }

        // 2. Weatherization assistance (consumption reduction)
        {
            let mobile_rural: Vec<&HouseholdGroup> = self
                .groups
                .iter()
                .filter(|g| {
                    matches!(g.housing_type, HousingType::MobileHome | HousingType::Rural)
                        && !g.has_renewable_access
                })
                .collect();

            if !mobile_rural.is_empty() {
                let total_benefited: usize = mobile_rural.iter().map(|g| g.n_households).sum();
                // Typical weatherization saves 25 % of energy use
                let avg_savings_kwh: f64 = mobile_rural
                    .iter()
                    .map(|g| g.avg_consumption_kwh * 0.25)
                    .sum::<f64>()
                    / mobile_rural.len() as f64;
                let programme_cost_per_hh = 6_500.0_f64; // typical US DOE WAP cost
                let cost_eff = if avg_savings_kwh > 0.0 {
                    programme_cost_per_hh / avg_savings_kwh
                } else {
                    0.0
                };
                interventions.push(PolicyIntervention {
                    name: "Weatherization Assistance Program".into(),
                    target_group: "Mobile home and rural households".into(),
                    subsidy_usd_per_household: programme_cost_per_hh,
                    n_households_benefited: total_benefited,
                    cost_effectiveness_usd_per_kwh_saved: cost_eff,
                    equity_improvement: -metrics.gini_coefficient * 0.04,
                    renewable_penetration_increase_pct: 0.0,
                });
            }
        }

        // 3. Community solar programme for low-income renters without RE access
        {
            let renters_no_re: Vec<&HouseholdGroup> = self
                .groups
                .iter()
                .filter(|g| {
                    matches!(
                        g.housing_type,
                        HousingType::RentedApartment | HousingType::PublicHousing
                    ) && !g.has_renewable_access
                })
                .collect();

            if !renters_no_re.is_empty() {
                let total_benefited: usize = renters_no_re.iter().map(|g| g.n_households).sum();
                // Community solar: bill credit typically 10 % of energy bill
                let avg_credit: f64 = renters_no_re
                    .iter()
                    .map(|g| g.avg_annual_energy_cost_usd * 0.10)
                    .sum::<f64>()
                    / renters_no_re.len() as f64;
                let avg_consumption: f64 = renters_no_re
                    .iter()
                    .map(|g| g.avg_consumption_kwh)
                    .sum::<f64>()
                    / renters_no_re.len() as f64;
                let cost_eff = if avg_consumption > 0.0 {
                    avg_credit / avg_consumption
                } else {
                    0.0
                };

                interventions.push(PolicyIntervention {
                    name: "Community Solar Subscription Program".into(),
                    target_group: "Low-income renters and public housing".into(),
                    subsidy_usd_per_household: avg_credit,
                    n_households_benefited: total_benefited,
                    cost_effectiveness_usd_per_kwh_saved: cost_eff,
                    equity_improvement: -metrics.gini_coefficient * 0.03,
                    renewable_penetration_increase_pct: 15.0,
                });
            }
        }

        // Sort by cost effectiveness (lower $/kWh = better)
        interventions.sort_by(|a, b| {
            a.cost_effectiveness_usd_per_kwh_saved
                .partial_cmp(&b.cost_effectiveness_usd_per_kwh_saved)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        interventions
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> EnergyEquityConfig {
        EnergyEquityConfig {
            median_household_income_usd: 60_000.0,
            energy_burden_threshold_pct: 0.06,
            affordability_index_method: AffordabilityMethod::EnergyBurden,
        }
    }

    fn high_income_group() -> HouseholdGroup {
        HouseholdGroup {
            id: 1,
            name: "High-income owners".into(),
            n_households: 1000,
            avg_income_usd: 120_000.0,
            avg_annual_energy_cost_usd: 2_400.0, // 2 % burden
            avg_consumption_kwh: 12_000.0,
            housing_type: HousingType::OwnedSingleFamily,
            has_renewable_access: true,
            has_battery_storage: true,
            geographic_area: "Suburb".into(),
        }
    }

    fn low_income_group() -> HouseholdGroup {
        HouseholdGroup {
            id: 2,
            name: "Low-income renters".into(),
            n_households: 500,
            avg_income_usd: 20_000.0,
            avg_annual_energy_cost_usd: 2_800.0, // 14 % burden
            avg_consumption_kwh: 8_000.0,
            housing_type: HousingType::RentedApartment,
            has_renewable_access: false,
            has_battery_storage: false,
            geographic_area: "Urban core".into(),
        }
    }

    // Test 1: High-income group has low burden
    #[test]
    fn test_high_income_low_burden() {
        let config = make_config();
        let mut analyzer = EnergyEquityAnalyzer::new(config);
        analyzer.add_group(high_income_group());

        let result = analyzer.analyze().expect("analysis should succeed");
        // 2400 / 120000 = 0.02 = 2 %, below 6 % threshold
        assert!(
            result.metrics.energy_burden_pct < 0.06,
            "High-income group should have low burden: {:.4}",
            result.metrics.energy_burden_pct
        );
        assert_eq!(
            result.metrics.high_burden_households_pct, 0.0,
            "No high-burden households"
        );
    }

    // Test 2: Low-income group has high burden detected
    #[test]
    fn test_low_income_high_burden_detected() {
        let config = make_config();
        let mut analyzer = EnergyEquityAnalyzer::new(config);
        analyzer.add_group(low_income_group());

        let result = analyzer.analyze().expect("analysis should succeed");
        // 2800 / 20000 = 0.14 > 0.06 threshold
        assert!(
            result.metrics.energy_burden_pct > 0.06,
            "Low-income group should have high burden: {:.4}",
            result.metrics.energy_burden_pct
        );
        assert!(
            result.metrics.high_burden_households_pct > 0.0,
            "Should detect high-burden households"
        );
        assert!(
            result.total_high_burden_households > 0,
            "Absolute count should be positive"
        );
    }

    // Test 3: Gini coefficient is 0 for equal distribution
    #[test]
    fn test_gini_equal_distribution() {
        let analyzer = EnergyEquityAnalyzer::new(make_config());
        let costs = vec![1000.0, 1000.0, 1000.0];
        let weights = vec![100, 100, 100];
        let gini = analyzer.gini_coefficient(&costs, &weights);
        assert!(
            gini < 1e-9,
            "Equal distribution should give Gini ≈ 0, got {gini:.6}"
        );
    }

    // Test 4: Gini coefficient approaches 1 for maximum inequality
    #[test]
    fn test_gini_maximum_inequality() {
        let analyzer = EnergyEquityAnalyzer::new(make_config());
        // One household with all the cost, many with zero
        let costs = vec![0.0, 0.0, 0.0, 1_000_000.0];
        let weights = vec![999, 999, 999, 1];
        let gini = analyzer.gini_coefficient(&costs, &weights);
        assert!(
            gini > 0.9,
            "Maximum inequality should give Gini close to 1, got {gini:.4}"
        );
    }

    // Test 5: Interventions generated for high-burden population
    #[test]
    fn test_interventions_generated_for_high_burden() {
        let config = make_config();
        let mut analyzer = EnergyEquityAnalyzer::new(config);
        analyzer.add_group(high_income_group());
        analyzer.add_group(low_income_group());

        let result = analyzer.analyze().expect("analysis should succeed");
        assert!(
            !result.best_interventions.is_empty(),
            "Should generate at least one intervention for high-burden group"
        );
        // At least one intervention should target the high-burden group
        let has_subsidy = result
            .best_interventions
            .iter()
            .any(|i| i.subsidy_usd_per_household > 0.0);
        assert!(
            has_subsidy,
            "At least one intervention should offer a subsidy"
        );
    }

    // Test 6: No groups → error
    #[test]
    fn test_no_groups_error() {
        let analyzer = EnergyEquityAnalyzer::new(make_config());
        assert!(
            matches!(analyzer.analyze(), Err(EquityError::NoGroups)),
            "Empty analyzer should return NoGroups error"
        );
    }

    // Test 7: Mixed population produces intermediate metrics
    #[test]
    fn test_mixed_population_metrics() {
        let config = make_config();
        let mut analyzer = EnergyEquityAnalyzer::new(config);
        analyzer.add_group(high_income_group());
        analyzer.add_group(low_income_group());

        let result = analyzer.analyze().expect("analysis should succeed");
        // Gini should be positive (not equal distribution)
        assert!(
            result.metrics.gini_coefficient > 0.0,
            "Mixed population should have positive Gini"
        );
        // Environmental justice index should be in [0, 100]
        assert!(
            result.metrics.environmental_justice_index >= 0.0
                && result.metrics.environmental_justice_index <= 100.0,
            "EJ index out of range: {:.2}",
            result.metrics.environmental_justice_index
        );
        // Equity score should match EJ index
        assert!(
            (result.equity_score - result.metrics.environmental_justice_index).abs() < 1e-9,
            "Equity score should equal EJ index"
        );
    }

    // Test 8: Mobile home group with no renewable access triggers weatherization intervention
    #[test]
    fn test_weatherization_intervention_triggered() {
        let config = make_config();
        let mut analyzer = EnergyEquityAnalyzer::new(config);
        let mobile_group = HouseholdGroup {
            id: 3,
            name: "Mobile home rural".into(),
            n_households: 200,
            avg_income_usd: 18_000.0,
            avg_annual_energy_cost_usd: 2_500.0, // 13.9 % burden
            avg_consumption_kwh: 10_000.0,
            housing_type: HousingType::MobileHome,
            has_renewable_access: false,
            has_battery_storage: false,
            geographic_area: "Rural".into(),
        };
        analyzer.add_group(mobile_group);

        let result = analyzer.analyze().expect("analysis should succeed");
        let has_weatherization = result
            .best_interventions
            .iter()
            .any(|i| i.name.contains("Weatherization"));
        assert!(
            has_weatherization,
            "Should generate Weatherization intervention for mobile home group"
        );
    }
}
