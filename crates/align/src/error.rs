//! 정렬 오류. 실패를 조용히 넘기지 않도록 모든 실패 조건을 구분한다.

/// align 크레이트 오류.
#[derive(Debug, thiserror::Error)]
pub enum AlignError {
    #[error("core 오류: {0}")]
    Core(#[from] skyrecon_core::Error),
    #[error("alignment_max_error 는 양수여야 함: {0}")]
    BadMaxError(f64),
    #[error("잘못된 인자: {0}")]
    InvalidArgument(String),
    #[error("기준 위치 부족: {found}개 (최소 {required})")]
    TooFewReferences { found: usize, required: usize },
    #[error("공통 영상 부족: {found}개 (최소 {required})")]
    TooFewCommonImages { found: usize, required: usize },
    #[error("공유 3D 점 대응 부족: {found}개 (최소 {required})")]
    TooFewCorrespondences { found: usize, required: usize },
    #[error("견고 Sim3 추정 실패 (인라이어 {inliers}개)")]
    RansacFailed { inliers: usize },
    #[error("Umeyama 해 없음(퇴화 배치)")]
    Degenerate,
}

pub type Result<T> = std::result::Result<T, AlignError>;
