//! 오류 타입.

use std::path::PathBuf;

/// core 전반의 오류.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("입출력 오류: {0}")]
    Io(#[from] std::io::Error),
    #[error("구문 오류 ({path}:{line}): {msg}")]
    Parse { path: PathBuf, line: usize, msg: String },
    #[error("형식 오류: {0}")]
    Format(String),
    #[error("잘못된 인자: {0}")]
    InvalidArgument(String),
    #[error("찾을 수 없음: {0}")]
    NotFound(String),
    #[error("이미 존재: {0}")]
    AlreadyExists(String),
    #[error("불변식 위반: {0}")]
    Invariant(String),
    #[error("지원하지 않음: {0}")]
    Unsupported(String),
}

pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    pub(crate) fn parse(path: impl Into<PathBuf>, line: usize, msg: impl Into<String>) -> Self {
        Error::Parse { path: path.into(), line, msg: msg.into() }
    }
}
