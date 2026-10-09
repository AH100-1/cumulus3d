//! 희소 재구성: 전역 SfM(회전 평균 + 위치 추정), 기존 모델에 새 영상 등록, 삼각측량.
//!
//! 입력은 `skyrecon-core` 의 [`skyrecon_core::FeatureStore`] (특징·두 뷰 기하)와
//! [`skyrecon_core::MatchGraph`] (대응 그래프), 출력은 [`skyrecon_core::Reconstruction`].
//! 진입점은 [`global_mapper()`] (첫 모델), [`register_images`] (새 영상 등록),
//! [`triangulate_points`] (점 생성·연장)이다. 모듈별 표는 크레이트 README 에 있다.
//!
//! # 사용 예
//!
//! 합성 장면(드론 3대, 위치마다 3장)의 앞 5개 위치로 첫 모델을 만들고,
//! 6번째 위치의 영상을 등록한 뒤 새 영상만 삼각측량한다.
//!
//! ```
//! use skyrecon_core::{MatchGraph, MatchGraphOptions};
//! use skyrecon_sfm::synthetic::{generate, SceneConfig};
//! use skyrecon_sfm::{
//!     global_mapper, register_images, triangulate_points, GlobalSfmOptions, PointTriangulatorOptions,
//!     RegistrationOptions, TriangulationScope,
//! };
//!
//! let scene = generate(&SceneConfig { num_positions: 6, num_points: 1500, ..Default::default() });
//! let graph_up_to = |n: usize| {
//!     let opts = MatchGraphOptions { image_names: scene.names_up_to(n), ..Default::default() };
//!     MatchGraph::from_store(&scene.store, &opts)
//! };
//!
//! // 1) 전역 SfM 으로 첫 모델.
//! let out = global_mapper(&scene.store, &graph_up_to(5), &GlobalSfmOptions::script())?;
//! assert!(out.failure.is_none());
//! let mut rec = out.reconstruction;
//!
//! // 2) 새 위치 도착: 등록 → 새 영상만 삼각측량.
//! let graph = graph_up_to(6);
//! let report = register_images(&mut rec, &scene.store, &graph, &RegistrationOptions::default())?;
//! let new_images = report.registered();
//! let opts = PointTriangulatorOptions { scope: TriangulationScope::Images(new_images), ..Default::default() };
//! let tri = triangulate_points(&mut rec, &graph, &opts)?;
//! println!("등록 {} 장, 새 점 {}", rec.registered_image_count(), tri.num_created);
//! # Ok::<(), skyrecon_core::Error>(())
//! ```

#![warn(missing_docs)]

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
