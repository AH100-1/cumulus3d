//! 공통 기반: 카메라 모델, 기하, 재구성 자료 구조, 입출력, RANSAC.
//!
//! 규약 요약(자세한 것은 `API.md`):
//! - 자세는 `world_to_cam`(세계 → 카메라), 쿼터니언은 해밀턴, 저장 순서 w,x,y,z.
//! - 카메라 좌표계: x 오른쪽, y 아래, z 앞. 픽셀 좌표: 좌상단 화소 중심 = (0.5, 0.5).
//! - id: 카메라/rig/프레임/영상 = u32, 3D 점 = u64, 짝 = u64. 무효값은 각 자료형의 최댓값.

pub mod analyzer;
pub mod camera;
pub mod error;
pub mod features;
pub mod geometry;
pub mod graph;
pub mod ids;
pub mod interop;
pub mod io;
pub mod linalg;
pub mod ransac;
pub mod reconstruction;
pub mod store;

pub use camera::{Camera, CameraModelKind};
pub use error::{Error, Result};
pub use features::{
    Descriptors, FeatureMatch, Keypoint, TwoViewGeometry, TwoViewGeometryConfig, DESCRIPTOR_DIM,
};
pub use geometry::{Mat3, Mat3x4, Quat, Rigid3, Sim3, Vec2, Vec3};
pub use graph::{MatchGraph, MatchGraphOptions};
pub use ids::*;
pub use reconstruction::{Frame, Image, Point2D, Point3D, Reconstruction, Rig, TrackEntry};
pub use store::FeatureStore;
