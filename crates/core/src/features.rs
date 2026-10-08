//! 특징·매칭 자료형.

use crate::error::{Error, Result};
use crate::geometry::{Mat3, Rigid3};

/// SIFT 기술자 차원.
pub const DESCRIPTOR_DIM: usize = 128;

/// 키포인트: 입력 영상 해상도 픽셀 좌표 (x, y) + 아핀 형상 A = [[a11, a12], [a21, a22]] (f32).
///
/// 픽셀 원점은 좌상단 모서리, 좌상단 화소 중심 = (0.5, 0.5).
/// A 의 열은 키포인트 국소 좌표축을 영상 좌표로 보낸 벡터.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Keypoint {
    pub x: f32,
    pub y: f32,
    pub a11: f32,
    pub a12: f32,
    pub a21: f32,
    pub a22: f32,
}

impl Keypoint {
    /// A = 단위행렬.
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y, a11: 1.0, a12: 0.0, a21: 0.0, a22: 1.0 }
    }
    /// (scale, orientation) → A = s·[[cos, −sin], [sin, cos]].
    pub fn from_scale_orientation(x: f32, y: f32, scale: f32, orientation: f32) -> Self {
        let (s, c) = orientation.sin_cos();
        Self { x, y, a11: scale * c, a12: -scale * s, a21: scale * s, a22: scale * c }
    }
    /// DB 행(열 수 2, 4, 6) 해석.
    pub fn from_row(row: &[f32]) -> Result<Self> {
        match row.len() {
            2 => Ok(Self::new(row[0], row[1])),
            4 => Ok(Self::from_scale_orientation(row[0], row[1], row[2], row[3])),
            6 => Ok(Self { x: row[0], y: row[1], a11: row[2], a12: row[3], a21: row[4], a22: row[5] }),
            n => Err(Error::Format(format!("키포인트 열 수 {n} 는 지원하지 않음"))),
        }
    }
    pub fn to_row(&self) -> [f32; 6] {
        [self.x, self.y, self.a11, self.a12, self.a21, self.a22]
    }
    pub fn scale_x(&self) -> f32 {
        (self.a11 * self.a11 + self.a21 * self.a21).sqrt()
    }
    pub fn scale_y(&self) -> f32 {
        (self.a12 * self.a12 + self.a22 * self.a22).sqrt()
    }
    pub fn scale(&self) -> f32 {
        0.5 * (self.scale_x() + self.scale_y())
    }
    pub fn orientation(&self) -> f32 {
        self.a21.atan2(self.a11)
    }
    pub fn shear(&self) -> f32 {
        (-self.a12).atan2(self.a22) - self.orientation()
    }
    /// 해상도 환산: x·sx, y·sy, 첫 열(a11,a21)·sx, 둘째 열(a12,a22)·sy.
    pub fn rescale(&mut self, sx: f32, sy: f32) {
        self.x *= sx;
        self.y *= sy;
        self.a11 *= sx;
        self.a21 *= sx;
        self.a12 *= sy;
        self.a22 *= sy;
    }
}

/// 영상 하나의 uint8 기술자 묶음(N × 128, 행 우선).
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Descriptors {
    data: Vec<u8>,
}

impl Descriptors {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn with_capacity(n: usize) -> Self {
        Self { data: Vec::with_capacity(n * DESCRIPTOR_DIM) }
    }
    /// 길이가 128 의 배수여야 한다.
    pub fn from_vec(data: Vec<u8>) -> Result<Self> {
        if !data.len().is_multiple_of(DESCRIPTOR_DIM) {
            return Err(Error::Format(format!("기술자 바이트 수 {} 가 128 의 배수가 아님", data.len())));
        }
        Ok(Self { data })
    }
    pub fn len(&self) -> usize {
        self.data.len() / DESCRIPTOR_DIM
    }
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
    pub fn row(&self, i: usize) -> &[u8] {
        &self.data[i * DESCRIPTOR_DIM..(i + 1) * DESCRIPTOR_DIM]
    }
    pub fn row_mut(&mut self, i: usize) -> &mut [u8] {
        &mut self.data[i * DESCRIPTOR_DIM..(i + 1) * DESCRIPTOR_DIM]
    }
    pub fn push(&mut self, d: &[u8; DESCRIPTOR_DIM]) {
        self.data.extend_from_slice(d);
    }
    pub fn as_slice(&self) -> &[u8] {
        &self.data
    }
    pub fn into_vec(self) -> Vec<u8> {
        self.data
    }
    /// 앞 n 개만 남김.
    pub fn truncate(&mut self, n: usize) {
        self.data.truncate(n * DESCRIPTOR_DIM);
    }
}

/// 두 영상 키포인트 인덱스 쌍 (영상1 idx, 영상2 idx).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct FeatureMatch {
    pub idx1: u32,
    pub idx2: u32,
}

impl FeatureMatch {
    pub fn new(idx1: u32, idx2: u32) -> Self {
        Self { idx1, idx2 }
    }
    pub fn swapped(&self) -> Self {
        Self { idx1: self.idx2, idx2: self.idx1 }
    }
}

/// 두 뷰 기하 구성 종류(DB config 값).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub enum TwoViewGeometryConfig {
    #[default]
    Undefined = 0,
    Degenerate = 1,
    Calibrated = 2,
    Uncalibrated = 3,
    Planar = 4,
    Panoramic = 5,
    PlanarOrRotation = 6,
    Watermark = 7,
    Multiple = 8,
    CalibratedRig = 9,
}

impl TwoViewGeometryConfig {
    pub fn from_i32(v: i32) -> Option<Self> {
        use TwoViewGeometryConfig::*;
        Some(match v {
            0 => Undefined,
            1 => Degenerate,
            2 => Calibrated,
            3 => Uncalibrated,
            4 => Planar,
            5 => Panoramic,
            6 => PlanarOrRotation,
            7 => Watermark,
            8 => Multiple,
            9 => CalibratedRig,
            _ => return None,
        })
    }
    pub fn as_i32(self) -> i32 {
        self as i32
    }
}

/// 짝 하나의 두 뷰 기하. 행렬은 "영상1 → 영상2" 방향(x2ᵀ F x1 = 0, x2 ~ H x1).
#[derive(Clone, Debug, PartialEq, Default)]
pub struct TwoViewGeometry {
    pub config: TwoViewGeometryConfig,
    pub e: Option<Mat3>,
    pub f: Option<Mat3>,
    pub h: Option<Mat3>,
    pub cam1_to_cam2: Option<Rigid3>,
    pub inlier_matches: Vec<FeatureMatch>,
    /// 삼각측량 각 중앙값(라디안). 저장되지 않는 부가 정보.
    pub tri_angle: Option<f64>,
}

impl TwoViewGeometry {
    /// 방향 반전: F → Fᵀ, E → Eᵀ, H → H⁻¹, 자세 → 역, 인라이어 열 교환.
    pub fn inverted(&self) -> Self {
        Self {
            config: self.config,
            e: self.e.map(|m| m.transpose()),
            f: self.f.map(|m| m.transpose()),
            h: self.h.and_then(|m| m.try_inverse()),
            cam1_to_cam2: self.cam1_to_cam2.map(|p| p.inverse()),
            inlier_matches: self.inlier_matches.iter().map(|m| m.swapped()).collect(),
            tri_angle: self.tri_angle,
        }
    }
    pub fn invert(&mut self) {
        *self = self.inverted();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keypoint_shape() {
        let k = Keypoint::from_scale_orientation(1600.5, 1200.25, 2.0, 0.3);
        assert!((k.scale() - 2.0).abs() < 1e-6);
        assert!((k.orientation() - 0.3).abs() < 1e-6);
        assert!(k.shear().abs() < 1e-6);
        let mut k2 = k;
        k2.rescale(4000.0 / 3200.0, 3000.0 / 2400.0);
        assert!((k2.x - 2000.625).abs() < 1e-5 * 2000.0);
        assert!((k2.y - 1500.3125).abs() < 1e-5 * 1500.0);
        assert!((k2.scale() - 2.5).abs() < 1e-5);
        assert!((k2.orientation() - 0.3).abs() < 1e-5);
        assert_eq!(Keypoint::from_row(&k.to_row()).unwrap(), k);
        assert!(Keypoint::from_row(&[1.0; 3]).is_err());
    }

    #[test]
    fn tvg_invert() {
        let h = Mat3::new(1.0, 0.1, 5.0, 0.0, 1.2, -3.0, 0.001, 0.0, 1.0);
        let g = TwoViewGeometry {
            config: TwoViewGeometryConfig::Calibrated,
            h: Some(h),
            f: Some(h),
            inlier_matches: vec![FeatureMatch::new(1, 2)],
            ..Default::default()
        };
        let gi = g.inverted();
        assert!((gi.h.unwrap() * h - Mat3::identity()).norm() < 1e-12);
        assert_eq!(gi.f.unwrap(), h.transpose());
        assert_eq!(gi.inlier_matches[0], FeatureMatch::new(2, 1));
        let gii = gi.inverted();
        assert!((gii.h.unwrap() - h).norm() < 1e-12);
    }
}
