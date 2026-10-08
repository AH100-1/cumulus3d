//! GPS→ENU, Sim3 정렬, 공유 3D 점 정렬, 점군 후처리.
//!
//! - [`geodesy`]: WGS84 LLA ↔ ECEF ↔ ENU.
//! - [`umeyama`]: Umeyama Sim3, LO-RANSAC 견고 추정, 반복 재가중(견고 Umeyama).
//! - [`model_aligner`]: 등록 영상 투영 중심 ↔ GPS ENU 정렬(강건 Sim3 추정).
//! - [`shared`]: 두 재구성 간 공유 3D 점 대응과 그 정렬.
//! - [`kdtree`], [`cloud`]: 점군 변환·근접 마스킹·추출·스냅샷 합성.
//! - [`reanchor`]: 새 정밀 모델 기준으로 이전 구역 좌표계를 다시 잇는 연쇄 Sim3.

pub mod cloud;
pub mod error;
pub mod geodesy;
pub mod kdtree;
pub mod model_aligner;
pub mod reanchor;
pub mod shared;
pub mod umeyama;

pub use cloud::{compose_snapshot, decimate, merge_clouds, remove_near, transform_cloud, SnapshotOptions};
pub use error::{AlignError, Result};
pub use geodesy::{ecef_to_lla, lla_to_ecef, EnuFrame, WGS84_A, WGS84_B, WGS84_E2, WGS84_F};
pub use kdtree::KdTree;
pub use model_aligner::{
    align_to_gps, align_to_gps_file, estimate_gps_alignment, gps_to_enu, EnuOrigin, GpsAlignment, ModelAlignerOptions,
};
pub use reanchor::AnchorChain;
pub use shared::{align_reconstructions, position_index_from_name, shared_point_correspondences, SharedPointOptions};
pub use umeyama::{
    estimate_sim3_ransac, robust_umeyama, umeyama, RankCheck, RobustUmeyamaOptions, RobustUmeyamaResult, Sim3Estimator,
};
