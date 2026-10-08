//! 기술자 매칭과 두 뷰 기하 검증.
//!
//! - [`descriptor`]: 정수 내적 무차별 매칭(비율·거리·교차 검사), [`MatcherBackend`] 로 GPU 교체 가능.
//! - [`estimators`], [`essential`]: 5점 E, 7/8점 F, DLT H, 평행이동 해법과 잔차(core RANSAC 추정기).
//! - [`two_view`]: E/F/H LO-RANSAC 과 구성 판정, 워터마크.
//! - [`pose`]: E/H 분해, 중점 삼각측량 cheirality, 상대 자세(sfm 공용).
//! - [`pairs`], [`pipeline`]: 짝 목록 파일과 FeatureStore 기록.

pub mod descriptor;
pub mod essential;
pub mod estimators;
pub mod linalg;
pub mod pairs;
pub mod pipeline;
pub mod poly;
pub mod pose;
pub mod two_view;

pub use descriptor::{AcceptRule, CpuMatcher, MatcherBackend, DescriptorMatchOptions, Top2};
pub use essential::{essential_eight_point, essential_five_point};
pub use estimators::{
    fundamental_eight_point, fundamental_seven_point, homography_dlt, homography_transfer_error_sq, sampson_error_sq,
    EssentialFivePointEstimator, FundamentalEightPointEstimator, Fundamental7PtEstimator, HomographyEstimator,
    TranslationEstimator,
};
pub use pairs::{parse_pair_list, read_pair_list, PairList};
pub use pipeline::{match_pair_list_file, match_pairs, verify_pair, MatchingStats, PairMatchingOptions};
pub use pose::{
    decompose_essential, decompose_homography, recover_two_view_pose, pose_from_essential, pose_from_homography,
    refit_and_estimate_relative_pose, triangulate_midpoint,
};
pub use two_view::{
    decide_calibrated, decide_uncalibrated, estimate_two_view, finalize_geometry, is_watermark, Decision, MaskChoice,
    ModelOutcome, TwoViewOptions,
};
