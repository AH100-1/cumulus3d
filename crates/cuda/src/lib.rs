//! CUDA 백엔드(cudarc + NVRTC 런타임 컴파일). CUDA 가 없는 기계에서도 빌드되고(동적 로딩),
//! 실행 시 장치가 없으면 [`is_available`] 이 거짓이다.
//!
//! - [`CudaPatchMatch`]: `skyrecon_dense::PatchMatchBackend` 구현(다중 스케일 조밀화의 스케일별 실행·평가·판독).
//! - [`CudaMatcher`]: `skyrecon_matching::MatcherBackend` 구현(정수 내적 top-2).
//! - [`CudaSift`]: `skyrecon_features::SiftEngine` 구현(GPU 스케일 공간 + CPU 검출·기술자, CpuSift 와 같은 결과).
//!
//! `unsafe` 는 cudarc 가 unsafe 로 둔 곳(커널 발사, 고정 호스트 메모리 할당, `DeviceRepr` 표시)에만 쓴다.
//!
//! # 사용 예
//!
//! ```
//! use skyrecon_cuda::{is_available, CudaMatcher};
//! use skyrecon_matching::MatcherBackend;
//!
//! // CUDA 가 없는 기계에서는 거짓이므로 아무것도 하지 않는다.
//! if is_available() {
//!     let matcher = CudaMatcher::try_default().expect("CUDA 장치 0");
//!     let (d1, d2) = (vec![1u8; 128 * 4], vec![1u8; 128 * 3]); // 128바이트 SIFT 기술자 4개, 3개
//!     let (rows, cols) = matcher.top2(&d1, 4, &d2, 3);
//!     assert_eq!((rows.len(), cols.len()), (4, 3));
//! }
//! ```
//!
//! ```no_run
//! use skyrecon_cuda::CudaPatchMatch;
//! use skyrecon_dense::{densify, DenseScene, DensifyOptions};
//!
//! fn run(scene: &DenseScene) -> Result<(), Box<dyn std::error::Error>> {
//!     let pm = CudaPatchMatch::try_default()?; // 장치 0, 기본 옵션
//!     let out = densify(scene, &DensifyOptions::default(), &pm, None)?;
//!     println!("조밀 점 {}개", out.cloud.len());
//!     Ok(())
//! }
//! ```
#![warn(missing_docs)]

mod device;
mod matcher;
mod patchmatch;
mod sift;

pub use device::{is_available, CudaDevice, GpuError};
pub use matcher::CudaMatcher;
pub use patchmatch::{CudaPatchMatch, CudaPatchMatchOptions};
pub use sift::CudaSift;
