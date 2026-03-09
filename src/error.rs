use thiserror::Error;

#[derive(Debug, Error)]
pub enum OxiGridError {
    #[error(
        "power flow did not converge after {iterations} iterations (residual: {residual:.2e})"
    )]
    Convergence { iterations: usize, residual: f64 },

    #[error("invalid network: {0}")]
    InvalidNetwork(String),

    #[error("parse error: {0}")]
    ParseError(String),

    #[error("linear algebra error: {0}")]
    LinearAlgebra(String),

    #[error("invalid parameter: {0}")]
    InvalidParameter(String),
}

pub type Result<T> = std::result::Result<T, OxiGridError>;
