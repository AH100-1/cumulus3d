//! 왜곡 보정과 다시점 조밀화.
//!
//! - [`undistort`]: OPENCV → PINHOLE 카메라 계산, 카메라별 LUT 캐시, 영상 재표본, 희소 모델 변환.
//! - [`image`]: 영상 버퍼와 재표본 도구.
//! - [`scene`]: 조밀화 장면(뷰 K·자세·영상, 희소점).
//! - [`neighbors`]: 이웃 뷰 선택과 깊이 범위.
//! - [`kernel`]: PatchMatch 백엔드 경계(한 스케일의 전파·뷰 선택·정제는 백엔드가 수행).
//! - [`densify`](mod@densify): 다중 스케일 진행, 세부 복원, 필터, 융합 연결.
//! - [`fusion`]: 이웃의 이웃 확장 중앙값 융합.
//! - [`cache`]: 겹치는 구역용 깊이맵 캐시.

pub mod cache;
pub mod densify;
pub mod fusion;
pub mod fusion_score;
pub mod image;
pub mod kernel;
pub mod math;
pub mod neighbors;
pub mod params;
pub mod postproc;
pub mod scene;
pub mod stats;
pub mod synthetic;
pub mod undistort;
pub mod upsample;

pub use cache::{CachedDepth, DepthMapCache};
pub use densify::{compute_depth_maps, densify, fuse_depth_maps, pair_geometry, DenseOutput, DenseTimings, DepthMapResult, DepthMapSet};
pub use fusion::{fuse, FusionInput, FusionOutput};
pub use image::{GrayImage, ImageBuffer};
pub use kernel::{DepthSnapshot, KernelInput, KernelView, LevelImage, PairGeometry, PatchMatchBackend, PatchMatchSession, RunParams, ViewState};
pub use params::{DensifyOptions, FilterParams, FusionMode, FusionParams, FusionResidual, ResidualParams, LevelSchedule, MvsProfile, NeighborParams, PmParams};
pub use postproc::PostParams;
pub use stats::{cloud_stats, CloudStats};
pub use scene::{DenseScene, DenseView, ScenePoint, SceneOptions};
pub use undistort::{
    undistort, undistort_from_dir, undistort_reconstruction, undistorted_camera, write_undistorted_workspace, CameraUndistortion, UndistortCache,
    UndistortOptions, UndistortResult,
};
