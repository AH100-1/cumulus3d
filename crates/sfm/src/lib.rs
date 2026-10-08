//! 전역 SfM, 영상 등록, 삼각측량. 요약은 `API.md`.

pub mod absolute_pose;
pub mod global_mapper;
pub mod math;
pub mod positioning;
pub mod registration;
pub mod rotation_averaging;
pub mod tracks;
pub mod triangulation;
pub mod triangulator;
#[doc(hidden)]
pub mod synthetic;

pub use global_mapper::{global_mapper, GlobalSfmOptions, GlobalMapperOutput};
pub use registration::{register_images, RegistrationOptions, RegistrationOrder, RegistrationReport};
pub use triangulator::{triangulate_points, PointRefiner, PointTriangulatorOptions, TriangulationScope};
