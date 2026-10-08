//! 식별자 자료형과 짝 id.

use crate::error::{Error, Result};

pub type CameraId = u32;
pub type RigId = u32;
pub type FrameId = u32;
pub type ImageId = u32;
/// 영상 내 2D 점(특징점) 순번.
pub type Point2DIdx = u32;
pub type Point3DId = u64;
pub type PairId = u64;

pub const INVALID_CAMERA_ID: CameraId = u32::MAX;
pub const INVALID_RIG_ID: RigId = u32::MAX;
pub const INVALID_FRAME_ID: FrameId = u32::MAX;
pub const INVALID_IMAGE_ID: ImageId = u32::MAX;
pub const INVALID_POINT2D_IDX: Point2DIdx = u32::MAX;
pub const INVALID_POINT3D_ID: Point3DId = u64::MAX;
pub const INVALID_PAIR_ID: PairId = u64::MAX;

/// 짝 id 상수 M = 2^31 − 1. 영상 id 는 이보다 작아야 한다.
pub const MAX_NUM_IMAGES: u64 = 2_147_483_647;

/// 센서 종류. 파일에는 i32 로 저장.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SensorKind {
    Invalid = -1,
    Camera = 0,
    Imu = 1,
}

impl SensorKind {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            -1 => Some(SensorKind::Invalid),
            0 => Some(SensorKind::Camera),
            1 => Some(SensorKind::Imu),
            _ => None,
        }
    }
    pub fn as_i32(self) -> i32 {
        self as i32
    }
    /// 텍스트 형식 이름.
    pub fn name(self) -> &'static str {
        match self {
            SensorKind::Invalid => "INVALID",
            SensorKind::Camera => "CAMERA",
            SensorKind::Imu => "IMU",
        }
    }
    pub fn from_name(s: &str) -> Option<Self> {
        match s {
            "INVALID" => Some(SensorKind::Invalid),
            "CAMERA" => Some(SensorKind::Camera),
            "IMU" => Some(SensorKind::Imu),
            _ => None,
        }
    }
}

/// 센서 식별자 (종류, id). 사전식 정렬(종류 → id).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SensorKey {
    pub sensor_type: SensorKind,
    pub id: u32,
}

impl SensorKey {
    pub fn new(sensor_type: SensorKind, id: u32) -> Self {
        Self { sensor_type, id }
    }
    /// 카메라 센서(id = 카메라 id).
    pub fn camera(camera_id: CameraId) -> Self {
        Self { sensor_type: SensorKind::Camera, id: camera_id }
    }
}

/// 데이터 식별자 (센서, 데이터 id). 카메라 데이터의 데이터 id 는 영상 id.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SensorDataKey {
    pub sensor_id: SensorKey,
    pub id: u64,
}

impl SensorDataKey {
    pub fn new(sensor_id: SensorKey, id: u64) -> Self {
        Self { sensor_id, id }
    }
    pub fn image(camera_id: CameraId, image_id: ImageId) -> Self {
        Self { sensor_id: SensorKey::camera(camera_id), id: image_id as u64 }
    }
}

/// 두 영상 id 의 순서가 "큰 → 작은" 이라 저장 시 교환이 필요한지.
pub fn swap_image_pair(image_id1: ImageId, image_id2: ImageId) -> bool {
    image_id1 > image_id2
}

/// 짝 id = M·min + max. 영상 id 가 M 이상이면 오류.
pub fn pair_id_of(image_id1: ImageId, image_id2: ImageId) -> Result<PairId> {
    if image_id1 as u64 >= MAX_NUM_IMAGES || image_id2 as u64 >= MAX_NUM_IMAGES {
        return Err(Error::InvalidArgument(format!(
            "영상 id 는 {MAX_NUM_IMAGES} 미만이어야 함: ({image_id1}, {image_id2})"
        )));
    }
    let (a, b) = if image_id1 <= image_id2 { (image_id1, image_id2) } else { (image_id2, image_id1) };
    Ok(MAX_NUM_IMAGES * a as u64 + b as u64)
}

/// 짝 id → (작은 id, 큰 id).
pub fn images_of_pair(pair_id: PairId) -> (ImageId, ImageId) {
    let id2 = pair_id % MAX_NUM_IMAGES;
    let id1 = (pair_id - id2) / MAX_NUM_IMAGES;
    (id1 as ImageId, id2 as ImageId)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pair_id_roundtrip() {
        assert_eq!(pair_id_of(1, 2).unwrap(), 2147483649);
        assert_eq!(pair_id_of(2, 1).unwrap(), 2147483649);
        assert_eq!(images_of_pair(2147483649), (1, 2));
        for (a, b) in [(0u32, 0u32), (5, 3), (2147483646, 7), (100, 2147483646)] {
            let id = pair_id_of(a, b).unwrap();
            assert_eq!(images_of_pair(id), (a.min(b), a.max(b)));
        }
        assert!(pair_id_of(2147483647, 1).is_err());
        assert!(swap_image_pair(2, 1));
        assert!(!swap_image_pair(1, 2));
    }

    #[test]
    fn sensor_ordering() {
        let a = SensorDataKey::image(1, 5);
        let b = SensorDataKey::image(2, 1);
        assert!(a < b);
        assert!(SensorKey::new(SensorKind::Camera, 9) < SensorKey::new(SensorKind::Imu, 0));
    }
}
