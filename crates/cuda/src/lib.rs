//! CUDA 백엔드(cudarc + NVRTC 런타임 컴파일). CUDA 가 없는 기계에서도 빌드되고(동적 로딩),
//! 실행 시 장치가 없으면 [`is_available`] 이 거짓이다.
//!
//! - [`CudaPatchMatch`]: `skyrecon_dense::PatchMatchBackend` 구현(다중 스케일 조밀화의 스케일별 실행·평가·판독).
//! - [`CudaMatcher`]: `skyrecon_matching::MatcherBackend` 구현(정수 내적 top-2).
//! - [`CudaSift`]: `skyrecon_features::SiftEngine` 구현(GPU 스케일 공간 + CPU 검출·기술자, CpuSift 와 같은 결과).
//!
//! `unsafe` 는 cudarc 가 unsafe 로 둔 곳(커널 발사, 고정 호스트 메모리 할당, `DeviceRepr` 표시)에만 쓴다.

mod device;
mod matcher;
mod patchmatch;
mod sift;

pub use device::{is_available, CudaDevice, GpuError};
pub use matcher::CudaMatcher;
pub use patchmatch::{CudaPatchMatch, CudaPatchMatchOptions};
pub use sift::CudaSift;
