use std::io;

use txengine_core::SinkError;

/// Errors produced by the hardware layer.
#[derive(Debug, thiserror::Error)]
pub enum HalError {
    #[error("{context}: {source}")]
    Io {
        context: String,
        #[source]
        source: io::Error,
    },
    #[error("unsupported: {0}")]
    Unsupported(String),
    #[error("timeout: {0}")]
    Timeout(String),
    #[error("hardware: {0}")]
    Hardware(String),
    #[error("configuration: {0}")]
    Config(String),
}

pub type Result<T, E = HalError> = std::result::Result<T, E>;

impl HalError {
    pub fn io(context: impl Into<String>, source: io::Error) -> Self {
        HalError::Io { context: context.into(), source }
    }
    pub fn last_os(context: impl Into<String>) -> Self {
        HalError::io(context, io::Error::last_os_error())
    }
}

impl From<HalError> for SinkError {
    fn from(e: HalError) -> Self {
        match e {
            HalError::Io { source, .. } => SinkError::Io(source),
            HalError::Config(m) => SinkError::Config(m),
            other => SinkError::Hardware(other.to_string()),
        }
    }
}
