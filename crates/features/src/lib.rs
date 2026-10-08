//! 이미지 읽기와 SIFT 특징 추출.
//!
//! - [`gray`]: 디코딩·회색 변환·축소·90° 회전.
//! - [`exif_info`], [`camera_init`]: EXIF 초점거리 규칙과 카메라 초기값.
//! - [`sift`]: 결정적 SIFT(레벨 단위 특징 수 제한), [`sift::SiftEngine`] 로 백엔드 교체 가능.
//! - [`extract`]: 카메라 묶기 + `FeatureStore` 증분 기록.

pub mod camera_init;
pub mod exif_info;
pub mod extract;
pub mod gray;
pub mod sift;

pub use camera_init::{infer_focal, init_camera, lookup_sensor_width, FocalEstimate};
pub use exif_info::ExifInfo;
pub use extract::{
    extract_for_camera, image_folder, read_image_list, CameraMode, ExtractionOptions, FeatureExtractor, ImageSource,
    ImageReport, ImageStatus, ReaderOptions,
};
pub use gray::{limited_size, read_gray, rgb_to_gray, rotate_keypoint_ccw, GrayImage};
pub use sift::{CpuSift, DescriptorNormalization, FeatureSelection, SiftEngine, SiftFeature, SiftOptions, SiftOutput};
