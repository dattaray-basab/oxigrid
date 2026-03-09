#![allow(clippy::needless_range_loop)]
//! Neural-network–inspired load/renewable forecasting (pure numerical, no ML libraries).
//!
//! Implements a fully-connected feed-forward neural network trained with
//! mini-batch stochastic gradient descent and backpropagation.
//!
//! # Features
//! - Xavier/Glorot weight initialization using an LCG random number generator.
//! - Configurable hidden layer sizes and activation functions
//!   (ReLU, Sigmoid, Tanh, Leaky ReLU).
//! - MSE loss, mini-batch SGD, and forward/backward pass via chain rule.
//! - Optional validation set reporting (MAE + RMSE).
//! - Min-max feature normalization.
//!
//! # Example
//! ```rust,ignore
//! use oxigrid::renewable::forecast::nn_forecast::{
//!     NnForecastConfig, NnForecastModel, ForecastSample, ActivationFn,
//! };
//!
//! let config = NnForecastConfig {
//!     n_inputs: 4,
//!     hidden_layers: vec![8, 4],
//!     n_outputs: 1,
//!     learning_rate: 0.01,
//!     n_epochs: 50,
//!     batch_size: 8,
//!     activation: ActivationFn::Relu,
//! };
//! let mut model = NnForecastModel::new(config, 42);
//! ```

use crate::renewable::forecast::ensemble_v2::ForecastError;

// ─────────────────────────────────────────────────────────────────────────────
// LCG random number generator
// ─────────────────────────────────────────────────────────────────────────────

/// Linear congruential generator (Knuth multiplier).
struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    /// Next `u64` in the sequence.
    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.state
    }

    /// Uniform float in \[−1, 1\].
    fn next_f64_pm1(&mut self) -> f64 {
        let bits = self.next_u64();
        let unit = (bits >> 11) as f64 / (1u64 << 53) as f64; // [0, 1)
        unit * 2.0 - 1.0
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Activation function
// ─────────────────────────────────────────────────────────────────────────────

/// Supported activation functions for hidden layers.
#[derive(Debug, Clone)]
pub enum ActivationFn {
    /// Rectified linear unit: max(0, x).
    Relu,
    /// Logistic sigmoid: 1 / (1 + e^−x).
    Sigmoid,
    /// Hyperbolic tangent.
    Tanh,
    /// Leaky ReLU: x if x > 0 else alpha × x.
    LeakyRelu {
        /// Negative-slope coefficient.
        alpha: f64,
    },
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for the neural network forecasting model.
#[derive(Debug, Clone)]
pub struct NnForecastConfig {
    /// Number of input features (e.g. 24 past-hour values + weather).
    pub n_inputs: usize,
    /// Number of neurons per hidden layer (e.g. `vec![64, 32]`).
    pub hidden_layers: Vec<usize>,
    /// Number of output forecast steps (e.g. 24).
    pub n_outputs: usize,
    /// SGD learning rate.
    pub learning_rate: f64,
    /// Number of training epochs.
    pub n_epochs: usize,
    /// Mini-batch size.
    pub batch_size: usize,
    /// Activation function for all hidden layers.
    pub activation: ActivationFn,
}

// ─────────────────────────────────────────────────────────────────────────────
// Layer
// ─────────────────────────────────────────────────────────────────────────────

/// A single fully-connected layer.
#[derive(Debug, Clone)]
pub struct NnLayer {
    /// Weight matrix \[out_neurons\]\[in_neurons\].
    pub weights: Vec<Vec<f64>>,
    /// Bias vector \[out_neurons\].
    pub biases: Vec<f64>,
    /// Number of input neurons.
    pub n_inputs: usize,
    /// Number of output neurons.
    pub n_outputs: usize,
}

impl NnLayer {
    /// Xavier/Glorot uniform initialization.
    ///
    /// Samples weights from U(−limit, +limit) where
    /// `limit = sqrt(6 / (n_in + n_out))`.
    fn xavier_init(n_inputs: usize, n_outputs: usize, lcg: &mut Lcg) -> Self {
        let limit = (6.0 / (n_inputs + n_outputs) as f64).sqrt();
        let weights: Vec<Vec<f64>> = (0..n_outputs)
            .map(|_| (0..n_inputs).map(|_| lcg.next_f64_pm1() * limit).collect())
            .collect();
        let biases = vec![0.0_f64; n_outputs];
        Self {
            weights,
            biases,
            n_inputs,
            n_outputs,
        }
    }

    /// Forward pass: compute z = Wx + b (pre-activation).
    fn linear_forward(&self, input: &[f64]) -> Vec<f64> {
        (0..self.n_outputs)
            .map(|o| {
                let wx: f64 = self.weights[o]
                    .iter()
                    .zip(input.iter())
                    .map(|(w, x)| w * x)
                    .sum();
                wx + self.biases[o]
            })
            .collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Training data
// ─────────────────────────────────────────────────────────────────────────────

/// A single training/validation sample.
#[derive(Debug, Clone)]
pub struct ForecastSample {
    /// Input features (normalized to \[0, 1\] recommended).
    pub features: Vec<f64>,
    /// Target outputs (normalized to \[0, 1\] recommended).
    pub targets: Vec<f64>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Result
// ─────────────────────────────────────────────────────────────────────────────

/// Forecast result after training and validation.
#[derive(Debug, Clone)]
pub struct NnForecastResult {
    /// Point forecast values (de-normalized if normalization was applied externally).
    pub point_forecast: Vec<f64>,
    /// MSE loss on the last training epoch.
    pub training_loss_final: f64,
    /// Mean absolute error on validation set.
    pub validation_mae: f64,
    /// Root-mean-square error on validation set.
    pub validation_rmse: f64,
    /// Total number of trainable parameters.
    pub n_parameters: usize,
}

// ─────────────────────────────────────────────────────────────────────────────
// Model
// ─────────────────────────────────────────────────────────────────────────────

/// Neural-network–inspired forecast model.
pub struct NnForecastModel {
    /// Trainable layers (hidden + output).
    pub layers: Vec<NnLayer>,
    /// Training configuration.
    pub config: NnForecastConfig,
    /// MSE loss per epoch.
    pub training_loss: Vec<f64>,
}

impl NnForecastModel {
    /// Construct model with Xavier/Glorot initialization using LCG `seed`.
    pub fn new(config: NnForecastConfig, seed: u64) -> Self {
        let mut lcg = Lcg::new(seed);

        // Build layer sizes: input → hidden[0] → ... → hidden[n-1] → output
        let mut sizes: Vec<usize> = Vec::with_capacity(config.hidden_layers.len() + 2);
        sizes.push(config.n_inputs);
        sizes.extend_from_slice(&config.hidden_layers);
        sizes.push(config.n_outputs);

        let layers: Vec<NnLayer> = sizes
            .windows(2)
            .map(|w| NnLayer::xavier_init(w[0], w[1], &mut lcg))
            .collect();

        Self {
            layers,
            config,
            training_loss: Vec::new(),
        }
    }

    // ── Activation ──────────────────────────────────────────────────────────

    /// Apply activation function to a pre-activation value.
    pub fn activate(&self, x: f64) -> f64 {
        match &self.config.activation {
            ActivationFn::Relu => x.max(0.0),
            ActivationFn::Sigmoid => 1.0 / (1.0 + (-x).exp()),
            ActivationFn::Tanh => x.tanh(),
            ActivationFn::LeakyRelu { alpha } => {
                if x >= 0.0 {
                    x
                } else {
                    alpha * x
                }
            }
        }
    }

    /// Derivative of the activation function with respect to pre-activation.
    pub fn activate_derivative(&self, x: f64) -> f64 {
        match &self.config.activation {
            ActivationFn::Relu => {
                if x > 0.0 {
                    1.0
                } else {
                    0.0
                }
            }
            ActivationFn::Sigmoid => {
                let s = 1.0 / (1.0 + (-x).exp());
                s * (1.0 - s)
            }
            ActivationFn::Tanh => {
                let t = x.tanh();
                1.0 - t * t
            }
            ActivationFn::LeakyRelu { alpha } => {
                if x >= 0.0 {
                    1.0
                } else {
                    *alpha
                }
            }
        }
    }

    // ── Forward pass ────────────────────────────────────────────────────────

    /// Forward pass: returns the output vector.
    ///
    /// Hidden layers use the configured activation function; the output layer
    /// is linear (identity activation) for regression.
    pub fn predict(&self, inputs: &[f64]) -> Vec<f64> {
        let n_layers = self.layers.len();
        let mut activation = inputs.to_vec();
        for (i, layer) in self.layers.iter().enumerate() {
            let z = layer.linear_forward(&activation);
            if i < n_layers - 1 {
                // hidden layers: apply activation
                activation = z.iter().map(|&v| self.activate(v)).collect();
            } else {
                // output layer: linear
                activation = z;
            }
        }
        activation
    }

    /// Forward pass that also records pre-activation values (z) and
    /// post-activation values (a) for each layer — required by backprop.
    fn forward_with_cache(&self, inputs: &[f64]) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let n_layers = self.layers.len();
        let mut zs: Vec<Vec<f64>> = Vec::with_capacity(n_layers);
        let mut activations: Vec<Vec<f64>> = Vec::with_capacity(n_layers + 1);
        activations.push(inputs.to_vec());

        for (i, layer) in self.layers.iter().enumerate() {
            let z = layer.linear_forward(activations.last().unwrap_or(&vec![]));
            let a: Vec<f64> = if i < n_layers - 1 {
                z.iter().map(|&v| self.activate(v)).collect()
            } else {
                z.clone() // linear output
            };
            zs.push(z);
            activations.push(a);
        }
        (zs, activations)
    }

    // ── Training ────────────────────────────────────────────────────────────

    /// Train the model using mini-batch SGD with MSE loss.
    ///
    /// # Algorithm
    /// For each epoch:
    /// 1. Shuffle training samples (deterministic order for reproducibility).
    /// 2. For each mini-batch, accumulate gradients via backpropagation.
    /// 3. Update weights: `W -= lr × ∂L/∂W`.
    ///
    /// Returns [`NnForecastResult`] with the final training loss, validation
    /// MAE/RMSE, and a point forecast on the first validation sample (if any).
    pub fn train(
        &mut self,
        training_data: &[ForecastSample],
        validation_data: &[ForecastSample],
    ) -> Result<NnForecastResult, ForecastError> {
        if training_data.is_empty() {
            return Err(ForecastError::NoMembers);
        }
        for (i, s) in training_data.iter().enumerate() {
            if s.features.len() != self.config.n_inputs {
                return Err(ForecastError::LengthMismatch {
                    id: i,
                    got: s.features.len(),
                    expected: self.config.n_inputs,
                });
            }
            if s.targets.len() != self.config.n_outputs {
                return Err(ForecastError::LengthMismatch {
                    id: i,
                    got: s.targets.len(),
                    expected: self.config.n_outputs,
                });
            }
        }

        let n = training_data.len();
        let lr = self.config.learning_rate;
        self.training_loss.clear();

        // Simple deterministic "shuffle": iterate in index order each epoch.
        // (A true shuffle would require a PRNG; this keeps things reproducible.)
        for _epoch in 0..self.config.n_epochs {
            let mut epoch_loss = 0.0_f64;
            let mut n_batches = 0usize;

            let batch_size = self.config.batch_size.min(n).max(1);

            // Build weight/bias gradient accumulators (same shape as layers)
            let mut dw: Vec<Vec<Vec<f64>>> = self
                .layers
                .iter()
                .map(|l| vec![vec![0.0_f64; l.n_inputs]; l.n_outputs])
                .collect();
            let mut db: Vec<Vec<f64>> = self
                .layers
                .iter()
                .map(|l| vec![0.0_f64; l.n_outputs])
                .collect();

            let mut batch_count = 0usize;

            for (idx, sample) in training_data.iter().enumerate() {
                let (zs, activations) = self.forward_with_cache(&sample.features);

                // MSE loss gradient wrt output: dL/da_out = 2*(pred - target)/n_out
                let pred = activations.last().cloned().unwrap_or_default();
                let n_out = pred.len().max(1);
                let mut delta: Vec<f64> = pred
                    .iter()
                    .zip(sample.targets.iter())
                    .map(|(p, t)| 2.0 * (p - t) / n_out as f64)
                    .collect();

                // Accumulate MSE loss
                let sample_loss: f64 = pred
                    .iter()
                    .zip(sample.targets.iter())
                    .map(|(p, t)| (p - t).powi(2))
                    .sum::<f64>()
                    / n_out as f64;
                epoch_loss += sample_loss;

                // Backward pass (from output layer to first hidden layer)
                for l in (0..self.layers.len()).rev() {
                    let a_in = &activations[l]; // activations entering layer l
                    let z_l = &zs[l];

                    // Apply activation derivative (skip for output layer — linear)
                    let delta_z: Vec<f64> = if l < self.layers.len() - 1 {
                        delta
                            .iter()
                            .zip(z_l.iter())
                            .map(|(d, z)| d * self.activate_derivative(*z))
                            .collect()
                    } else {
                        delta.clone()
                    };

                    // Accumulate weight and bias gradients
                    for o in 0..self.layers[l].n_outputs {
                        db[l][o] += delta_z[o];
                        for i in 0..self.layers[l].n_inputs {
                            dw[l][o][i] += delta_z[o] * a_in[i];
                        }
                    }

                    // Propagate delta to previous layer
                    if l > 0 {
                        let n_prev = self.layers[l].n_inputs;
                        let mut prev_delta = vec![0.0_f64; n_prev];
                        for i in 0..n_prev {
                            for o in 0..self.layers[l].n_outputs {
                                prev_delta[i] += self.layers[l].weights[o][i] * delta_z[o];
                            }
                        }
                        delta = prev_delta;
                    }
                }

                batch_count += 1;

                // Apply gradients when batch is full or last sample
                if batch_count == batch_size || idx == n - 1 {
                    let scale = batch_count as f64;
                    for l in 0..self.layers.len() {
                        for o in 0..self.layers[l].n_outputs {
                            self.layers[l].biases[o] -= lr * db[l][o] / scale;
                            for i in 0..self.layers[l].n_inputs {
                                self.layers[l].weights[o][i] -= lr * dw[l][o][i] / scale;
                            }
                        }
                    }
                    // Reset accumulators
                    for l in 0..self.layers.len() {
                        for o in 0..self.layers[l].n_outputs {
                            db[l][o] = 0.0;
                            for i in 0..self.layers[l].n_inputs {
                                dw[l][o][i] = 0.0;
                            }
                        }
                    }
                    batch_count = 0;
                    n_batches += 1;
                }
            }

            let epoch_loss_avg = if n_batches > 0 {
                epoch_loss / n as f64
            } else {
                epoch_loss
            };
            self.training_loss.push(epoch_loss_avg);
        }

        // Validation metrics
        let (val_mae, val_rmse) = if validation_data.is_empty() {
            (0.0, 0.0)
        } else {
            let mut sum_ae = 0.0_f64;
            let mut sum_se = 0.0_f64;
            let mut n_preds = 0usize;
            for sample in validation_data {
                let pred = self.predict(&sample.features);
                for (p, t) in pred.iter().zip(sample.targets.iter()) {
                    let err = p - t;
                    sum_ae += err.abs();
                    sum_se += err * err;
                    n_preds += 1;
                }
            }
            let n_f = n_preds.max(1) as f64;
            (sum_ae / n_f, (sum_se / n_f).sqrt())
        };

        // Point forecast on first validation sample (or zeros)
        let point_forecast = if let Some(first) = validation_data.first() {
            self.predict(&first.features)
        } else {
            vec![0.0; self.config.n_outputs]
        };

        let training_loss_final = self.training_loss.last().copied().unwrap_or(0.0);
        let n_parameters = self.count_parameters();

        Ok(NnForecastResult {
            point_forecast,
            training_loss_final,
            validation_mae: val_mae,
            validation_rmse: val_rmse,
            n_parameters,
        })
    }

    // ── Utilities ───────────────────────────────────────────────────────────

    /// Count total number of trainable parameters (weights + biases).
    pub fn count_parameters(&self) -> usize {
        self.layers
            .iter()
            .map(|l| l.n_outputs * l.n_inputs + l.n_outputs)
            .sum()
    }

    /// Normalize a slice of data to \[0, 1\].
    ///
    /// Returns `(normalized_data, min_value, max_value)`.
    /// If all values are equal, returns zeros and the shared value for both
    /// min and max to avoid division by zero.
    pub fn normalize(data: &[f64]) -> (Vec<f64>, f64, f64) {
        if data.is_empty() {
            return (Vec::new(), 0.0, 1.0);
        }
        let min = data.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = data.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let range = max - min;
        if range < 1e-12 {
            return (vec![0.0; data.len()], min, max);
        }
        let normalized = data.iter().map(|&v| (v - min) / range).collect();
        (normalized, min, max)
    }

    /// De-normalize values from \[0, 1\] back to original scale.
    pub fn denormalize(normalized: &[f64], min: f64, max: f64) -> Vec<f64> {
        let range = max - min;
        normalized.iter().map(|&v| v * range + min).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_config() -> NnForecastConfig {
        NnForecastConfig {
            n_inputs: 4,
            hidden_layers: vec![8, 4],
            n_outputs: 2,
            learning_rate: 0.01,
            n_epochs: 10,
            batch_size: 4,
            activation: ActivationFn::Relu,
        }
    }

    #[test]
    fn test_forward_pass_output_shape() {
        let config = simple_config();
        let model = NnForecastModel::new(config, 42);
        let inputs = vec![0.1, 0.5, 0.3, 0.7];
        let output = model.predict(&inputs);
        assert_eq!(output.len(), 2, "Output should have n_outputs=2 elements");
    }

    #[test]
    fn test_xavier_init_reasonable_magnitude() {
        let config = NnForecastConfig {
            n_inputs: 16,
            hidden_layers: vec![32, 16],
            n_outputs: 4,
            learning_rate: 0.001,
            n_epochs: 1,
            batch_size: 8,
            activation: ActivationFn::Tanh,
        };
        let model = NnForecastModel::new(config, 1234);
        for layer in &model.layers {
            for row in &layer.weights {
                for &w in row {
                    assert!(w.abs() < 2.0, "Xavier init: weight {w:.4} should be small");
                }
            }
        }
    }

    #[test]
    fn test_training_loss_decreases() {
        let config = NnForecastConfig {
            n_inputs: 2,
            hidden_layers: vec![4],
            n_outputs: 1,
            learning_rate: 0.05,
            n_epochs: 50,
            batch_size: 4,
            activation: ActivationFn::Sigmoid,
        };
        let mut model = NnForecastModel::new(config, 99);

        // Simple linear mapping: y = 0.5 * x0 + 0.5 * x1
        let training_data: Vec<ForecastSample> = (0..16)
            .map(|i| {
                let x0 = (i as f64) / 16.0;
                let x1 = 1.0 - x0;
                ForecastSample {
                    features: vec![x0, x1],
                    targets: vec![0.5 * x0 + 0.5 * x1],
                }
            })
            .collect();

        let result = model.train(&training_data, &[]).unwrap();

        let first_loss = model.training_loss.first().copied().unwrap_or(f64::MAX);
        assert!(
            result.training_loss_final <= first_loss + 1e-6,
            "Training loss should not increase: final={} vs first={}",
            result.training_loss_final,
            first_loss
        );
    }

    #[test]
    fn test_constant_input_learns_constant_output() {
        // Feed the same input; after enough training, output should converge.
        let config = NnForecastConfig {
            n_inputs: 3,
            hidden_layers: vec![8],
            n_outputs: 1,
            learning_rate: 0.1,
            n_epochs: 200,
            batch_size: 8,
            activation: ActivationFn::Tanh,
        };
        let mut model = NnForecastModel::new(config, 7);

        let target = 0.75_f64;
        let training_data: Vec<ForecastSample> = (0..32)
            .map(|_| ForecastSample {
                features: vec![0.5, 0.5, 0.5],
                targets: vec![target],
            })
            .collect();

        let _result = model.train(&training_data, &[]).unwrap();
        let pred = model.predict(&[0.5, 0.5, 0.5]);

        assert_eq!(pred.len(), 1, "Output dimension mismatch");
        // Should learn approximately the constant target
        assert!(
            (pred[0] - target).abs() < 0.25,
            "Model should approximate constant target {target}, got {:.4}",
            pred[0]
        );
    }

    #[test]
    fn test_normalization_range() {
        let data = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let (norm, min, max) = NnForecastModel::normalize(&data);
        assert!((min - 2.0).abs() < 1e-9, "min should be 2.0");
        assert!((max - 10.0).abs() < 1e-9, "max should be 10.0");
        assert!((norm[0] - 0.0).abs() < 1e-9, "first element should be 0.0");
        assert!((norm[4] - 1.0).abs() < 1e-9, "last element should be 1.0");
    }

    #[test]
    fn test_normalization_constant_input() {
        let data = vec![5.0, 5.0, 5.0];
        let (norm, min, max) = NnForecastModel::normalize(&data);
        assert!((min - 5.0).abs() < 1e-9);
        assert!((max - 5.0).abs() < 1e-9);
        for v in norm {
            assert!(
                (v - 0.0).abs() < 1e-9,
                "Constant data should normalize to 0.0"
            );
        }
    }

    #[test]
    fn test_n_parameters_correct() {
        // 4 inputs → 8 hidden → 2 outputs
        // Layer 0: 4*8 + 8 = 40
        // Layer 1: 8*2 + 2 = 18
        // Total: 58
        let config = NnForecastConfig {
            n_inputs: 4,
            hidden_layers: vec![8],
            n_outputs: 2,
            learning_rate: 0.01,
            n_epochs: 1,
            batch_size: 4,
            activation: ActivationFn::Relu,
        };
        let model = NnForecastModel::new(config, 0);
        let expected = 4 * 8 + 8 + 8 * 2 + 2; // 58
        assert_eq!(model.count_parameters(), expected);
    }

    #[test]
    fn test_leaky_relu_activation() {
        let config = NnForecastConfig {
            n_inputs: 2,
            hidden_layers: vec![4],
            n_outputs: 1,
            learning_rate: 0.01,
            n_epochs: 1,
            batch_size: 2,
            activation: ActivationFn::LeakyRelu { alpha: 0.1 },
        };
        let model = NnForecastModel::new(config, 0);
        assert!((model.activate(1.0) - 1.0).abs() < 1e-9, "x>0: identity");
        assert!((model.activate(-2.0) - (-0.2)).abs() < 1e-9, "x<0: alpha*x");
        assert!((model.activate_derivative(-1.0) - 0.1).abs() < 1e-9);
    }

    #[test]
    fn test_denormalize_roundtrip() {
        let original = vec![10.0, 20.0, 30.0, 40.0];
        let (norm, min, max) = NnForecastModel::normalize(&original);
        let recovered = NnForecastModel::denormalize(&norm, min, max);
        for (orig, rec) in original.iter().zip(recovered.iter()) {
            assert!(
                (orig - rec).abs() < 1e-9,
                "Denormalize roundtrip failed: {orig} vs {rec}"
            );
        }
    }

    #[test]
    fn test_empty_training_data_returns_error() {
        let config = simple_config();
        let mut model = NnForecastModel::new(config, 0);
        let result = model.train(&[], &[]);
        assert!(result.is_err(), "Empty training data should return error");
    }
}
