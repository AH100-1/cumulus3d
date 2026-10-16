/*
 * gravity.rs
 *
 * Copyright (c) 2026 ParkSangWoo
 *
 * Author(s):
 *
 *      ParkSangWoo <dev.parksangwoo@gmail.com>
 *
 *
 * This file is part of cumulus3d.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Third-party notices: parts of the algorithms, default parameters and data
 * formats in this file follow other open-source projects. Their copyright
 * notices and licenses are reproduced in THIRD_PARTY_NOTICES.md.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//! Gravity prior for GPS ENU fits whose positions are (nearly) collinear.
//!
//! A Sim3 fitted to camera centers on a straight line leaves the rotation about that line unobservable: the scene can end
//! up rolled about the flight line. When that happens the free roll is chosen from gravity: (a) a dominant ground plane
//! fitted to the sparse points (cameras above it, normal -> +Up), or, failing that, (b) the mean optical axis pointing down.
//! Scale and translation stay those of the GPS fit. With non-degenerate GPS nothing here changes the fit.

use crate::umeyama::umeyama;
use cumulus3d_core::linalg::svd3;
use cumulus3d_core::{Mat3, Quat, Reconstruction, Sim3, Vec3};

/// Ratio of the 2nd to the 1st singular value of the centred positions below which they count as collinear.
pub const COLLINEAR_RATIO: f64 = 0.05;

/// Which gravity prior fixed the free roll.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GravityPrior {
    /// A dominant ground plane of the sparse points (inlier count).
    Ground {
        /// Points on the plane.
        inliers: usize,
    },
    /// The mean optical axis made to point down.
    ViewDirection,
    /// No prior was usable (the line is vertical or no points / cameras); the fit is left as is.
    None,
}

/// Outcome of a degenerate-GPS fit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GravityResolution {
    /// σ2/σ1 of the centred GPS positions.
    pub ratio: f64,
    /// Prior used.
    pub prior: GravityPrior,
}

/// σ2/σ1 of the centred positions (1.0 when fewer than 3 points or no spread, so those are never "degenerate" here).
pub fn collinearity_ratio(pts: &[Vec3]) -> f64 {
    line_axis(pts).map_or(1.0, |(_, r, _)| r)
}

/// `Some(ratio)` when the positions are collinear (ratio < [`COLLINEAR_RATIO`]).
pub fn degenerate_ratio(pts: &[Vec3]) -> Option<f64> {
    let r = collinearity_ratio(pts);
    (r < COLLINEAR_RATIO).then_some(r)
}

/// (principal direction, σ2/σ1, centroid).
fn line_axis(pts: &[Vec3]) -> Option<(Vec3, f64, Vec3)> {
    if pts.len() < 3 {
        return None;
    }
    let mu = pts.iter().sum::<Vec3>() / pts.len() as f64;
    let mut cov = Mat3::zeros();
    for p in pts {
        let d = p - mu;
        cov += d * d.transpose();
    }
    let (_, s, v) = svd3(&cov)?;
    if s[0] <= 1e-18 {
        return None;
    }
    Some((v.column(0).into_owned(), (s[1] / s[0]).sqrt(), mu))
}

/// Umeyama Sim3 `src -> dst`; when `dst` is collinear the roll about the line is resolved with the gravity prior of `rec`
/// (`src` are the model-frame positions). `None` if Umeyama fails.
pub fn fit_enu_sim3(rec: &Reconstruction, src: &[Vec3], dst: &[Vec3]) -> Option<(Sim3, Option<GravityResolution>)> {
    let t = umeyama(src, dst, true)?;
    match degenerate_ratio(dst) {
        None => Some((t, None)),
        Some(ratio) => {
            let (t2, prior) = resolve_roll(rec, &t, dst);
            Some((t2, Some(GravityResolution { ratio, prior })))
        }
    }
}

/// Umeyama Sim3 between two frames that already share the same Up (e.g. two ENU models). With collinear `dst` the roll
/// about the line is chosen so that +Z maps to +Z.
pub fn fit_same_up_sim3(src: &[Vec3], dst: &[Vec3]) -> Option<Sim3> {
    let t = umeyama(src, dst, true)?;
    let Some((d, ratio, mu)) = line_axis(dst) else { return Some(t) };
    if ratio >= COLLINEAR_RATIO {
        return Some(t);
    }
    let up = Vec3::z();
    let moved = t.rotation.rotate(&up);
    Some(roll_towards(&t, &d, &mu, &moved, &up).unwrap_or(t))
}

/// Resolves the free roll of `t` (model -> ENU) about the line through `dst`.
pub fn resolve_roll(rec: &Reconstruction, t: &Sim3, dst: &[Vec3]) -> (Sim3, GravityPrior) {
    let Some((d, _, mu)) = line_axis(dst) else { return (*t, GravityPrior::None) };
    let up = Vec3::z();
    if d.dot(&up).abs() > 0.98 {
        return (*t, GravityPrior::None); // vertical line: Up is already fixed by the line itself.
    }
    // Registered camera centres and optical axes in ENU.
    let (mut cams, mut axes) = (Vec::new(), Vec::new());
    for id in rec.registered_images() {
        if let Some(p) = rec.world_to_cam(id) {
            cams.push(t.transform_point(&p.center()));
            axes.push(t.rotation.rotate(&p.viewing_direction()));
        }
    }
    let pts: Vec<Vec3> = {
        let all: Vec<Vec3> = rec.points3d().map(|(_, p)| t.transform_point(&p.xyz)).filter(|p| p.iter().all(|v| v.is_finite())).collect();
        let step = (all.len() / MAX_PLANE_POINTS).max(1);
        all.into_iter().step_by(step).collect()
    };
    let mean_axis = axes.iter().sum::<Vec3>();
    if let Some((n, inliers)) = ground_plane(&pts, &cams) {
        if let Some(t2) = roll_towards(t, &d, &mu, &n, &up) {
            // Sanity: with the ground horizontal, drone cameras must look down on average.
            let a = t2.rotation.rotate(&t.rotation.conjugate().rotate(&mean_axis));
            if axes.is_empty() || a.z <= 0.0 {
                return (t2, GravityPrior::Ground { inliers });
            }
        }
    }
    if mean_axis.norm() > 1e-9 {
        if let Some(t2) = roll_towards(t, &d, &mu, &mean_axis, &(-up)) {
            return (t2, GravityPrior::ViewDirection);
        }
    }
    (*t, GravityPrior::None)
}

/// `t` followed by the rotation about the axis `d` through `mu` that takes the direction `from` closest to `to`.
/// `None` when either has (almost) no component perpendicular to the axis.
fn roll_towards(t: &Sim3, d: &Vec3, mu: &Vec3, from: &Vec3, to: &Vec3) -> Option<Sim3> {
    let perp = |v: &Vec3| v - d * d.dot(v);
    let (a, b) = (perp(from), perp(to));
    if a.norm() < 0.2 * from.norm().max(1e-12) || b.norm() < 0.2 * to.norm().max(1e-12) {
        return None;
    }
    let ang = d.dot(&a.cross(&b)).atan2(a.dot(&b));
    let q = Quat::from_axis_angle(d, ang);
    Some(Sim3::new(t.scale, q.hamilton(&t.rotation).normalized(), q.rotate(&(t.translation - mu)) + mu))
}

const MAX_PLANE_POINTS: usize = 6000;
const PLANE_ITERS: usize = 600;

/// Dominant ground plane: unit normal pointing to the cameras' side and the inlier count. RANSAC + least squares; planes
/// whose camera side is ambiguous are rejected and planes with many points behind them (roofs, walls) score low.
fn ground_plane(pts: &[Vec3], cams: &[Vec3]) -> Option<(Vec3, usize)> {
    let n = pts.len();
    if n < 30 || cams.len() < 2 {
        return None;
    }
    let med = |mut v: Vec<f64>| {
        v.sort_by(|a, b| a.total_cmp(b));
        v[v.len() / 2]
    };
    let centre =
        Vec3::new(med(pts.iter().map(|p| p.x).collect()), med(pts.iter().map(|p| p.y).collect()), med(pts.iter().map(|p| p.z).collect()));
    let thr = 0.03 * med(pts.iter().map(|p| (p - centre).norm()).collect());
    if thr.is_nan() || thr <= 0.0 {
        return None;
    }
    // Orients (n, off) so that cameras are on the positive side; None if they are not clearly above.
    let orient = |nrm: Vec3, off: f64| -> Option<(Vec3, f64)> {
        let sd: Vec<f64> = cams.iter().map(|c| nrm.dot(c) - off).collect();
        let pos = sd.iter().filter(|s| **s > 0.0).count();
        let (nrm, off, pos) = if pos * 2 < sd.len() { (-nrm, -off, sd.len() - pos) } else { (nrm, off, pos) };
        (pos as f64 >= 0.9 * sd.len() as f64).then_some((nrm, off))
    };
    let score = |nrm: &Vec3, off: f64| -> (f64, usize) {
        let (mut inl, mut behind) = (0usize, 0usize);
        for p in pts {
            let s = nrm.dot(p) - off;
            if s.abs() <= thr {
                inl += 1;
            } else if s < 0.0 {
                behind += 1;
            }
        }
        (inl as f64 - 2.0 * behind as f64, inl)
    };
    let mut state = 0x9E37_79B9_7F4A_7C15u64;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state % n as u64) as usize
    };
    let mut best: Option<(f64, Vec3, f64)> = None;
    for _ in 0..PLANE_ITERS {
        let (a, b, c) = (pts[next()], pts[next()], pts[next()]);
        let nv = (b - a).cross(&(c - a));
        let len = nv.norm();
        if len < 1e-12 {
            continue;
        }
        let nrm = nv / len;
        let Some((nrm, off)) = orient(nrm, nrm.dot(&a)) else { continue };
        let (s, _) = score(&nrm, off);
        if best.as_ref().is_none_or(|b| s > b.0) {
            best = Some((s, nrm, off));
        }
    }
    let (_, mut nrm, mut off) = best?;
    for _ in 0..3 {
        let inl: Vec<Vec3> = pts.iter().filter(|p| (nrm.dot(p) - off).abs() <= thr).copied().collect();
        if inl.len() < 10 {
            return None;
        }
        let mu = inl.iter().sum::<Vec3>() / inl.len() as f64;
        let mut cov = Mat3::zeros();
        for p in &inl {
            cov += (p - mu) * (p - mu).transpose();
        }
        let (_, _, v) = svd3(&cov)?;
        let cand = v.column(2).into_owned();
        let (n2, o2) = orient(cand, cand.dot(&mu))?;
        nrm = n2;
        off = o2;
    }
    let (s, inl) = score(&nrm, off);
    (s > 0.0 && inl >= 10 && inl as f64 >= 0.15 * n as f64).then_some((nrm, inl))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::EnuFrame;
    use crate::model_aligner::{align_to_gps, EnuOrigin, ModelAlignerOptions};
    use cumulus3d_core::io::GpsRecord;
    use cumulus3d_core::{Camera, CameraModelKind, Image, Rigid3, TrackEntry};

    struct Lcg(u64);
    impl Lcg {
        fn f(&mut self) -> f64 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
        fn r(&mut self, lo: f64, hi: f64) -> f64 {
            lo + (hi - lo) * self.f()
        }
    }

    /// Camera-to-world rotation (columns x right, y down, z = view direction) for a camera looking along `view`.
    fn look(view: Vec3) -> Mat3 {
        let z = view.normalize();
        let x = z.cross(&Vec3::z()).normalize();
        let y = z.cross(&x);
        Mat3::from_columns(&[x, y, z])
    }

    /// Cameras on one straight line (three passes, different yaw, pitch -60 deg) over a ground plane with buildings,
    /// expressed in an arbitrary model frame. Returns (reconstruction, ENU camera positions, names, enu_to_model).
    fn scene(line: &[Vec3]) -> (Reconstruction, Vec<String>, Sim3) {
        let mut rng = Lcg(7);
        let rot = Quat::from_axis_angle(&Vec3::new(0.3, -0.5, 0.8).normalize(), 2.1);
        let e2m = Sim3::new(0.37, rot, Vec3::new(5.0, -3.0, 11.0));
        let mut rec = Reconstruction::new();
        rec.add_camera_own_rig(Camera::new(1, CameraModelKind::Pinhole, 1000, 800, vec![800.0, 800.0, 500.0, 400.0]).unwrap()).unwrap();
        let mut names = Vec::new();
        for (i, c) in line.iter().enumerate() {
            let yaw = (i % 3) as f64 * 0.6 - 0.6;
            let pitch = -60f64.to_radians();
            let view = Vec3::new(yaw.sin() * pitch.cos(), yaw.cos() * pitch.cos(), pitch.sin());
            let rc = look(view); // cam -> ENU world
            let r_m = rc.transpose() * e2m.rotation.to_rotation_matrix().transpose();
            let cm = e2m.transform_point(c);
            let q = Quat::from_rotation_matrix(&r_m);
            let pts: Vec<_> = (0..4).map(|k| cumulus3d_core::Vec2::new(10.0 + k as f64, 20.0)).collect();
            let name = format!("img_{i:03}.jpg");
            rec.add_image_own_frame(Image::new(i as u32 + 1, &name, 1, pts), Some(Rigid3::new(q, -(r_m * cm)))).unwrap();
            rec.register_image(i as u32 + 1).unwrap();
            names.push(name);
        }
        // Ground z = 0 (noisy) and three buildings with walls and roofs.
        let mut pts = Vec::new();
        for _ in 0..1500 {
            pts.push(Vec3::new(rng.r(-20.0, 100.0), rng.r(10.0, 110.0), rng.r(-0.1, 0.1)));
        }
        for (cx, cy, h) in [(30.0, 60.0, 15.0), (60.0, 80.0, 20.0), (80.0, 40.0, 10.0)] {
            for _ in 0..150 {
                let (u, v, w) = (rng.r(-5.0, 5.0), rng.r(-5.0, 5.0), rng.r(0.0, h));
                pts.push(Vec3::new(cx + u, cy + v, w)); // walls (one coordinate pinned below) and interior
                pts.push(Vec3::new(cx + [-5.0, 5.0][(w as usize) % 2], cy + v, w));
                pts.push(Vec3::new(cx + u, cy + v, h)); // roof
            }
        }
        for p in pts {
            rec.add_point3d(e2m.transform_point(&p), Vec::<TrackEntry>::new(), [0, 0, 0]).unwrap();
        }
        (rec, names, e2m)
    }

    fn line_cams() -> Vec<Vec3> {
        (0..30).map(|i| Vec3::new((i % 10) as f64 * 76.0 / 9.0, 0.0, 40.0)).collect()
    }

    fn ground_normal_error_deg(rec: &Reconstruction, sim: &Sim3) -> (f64, bool) {
        // Fit the true ground (z = 0 points are the first 1500 added).
        let g: Vec<Vec3> = rec.points3d().take(1500).map(|(_, p)| sim.transform_point(&p.xyz)).collect();
        let mu = g.iter().sum::<Vec3>() / g.len() as f64;
        let mut cov = Mat3::zeros();
        for p in &g {
            cov += (p - mu) * (p - mu).transpose();
        }
        let (_, _, v) = svd3(&cov).unwrap();
        let n = v.column(2).into_owned();
        let ang = n.dot(&Vec3::z()).abs().min(1.0).acos().to_degrees();
        let above = rec.registered_images().iter().all(|i| sim.transform_point(&rec.projection_center(*i).unwrap()).z > mu.z);
        (ang, above)
    }

    #[test]
    fn collinear_gps_ground_prior_recovers_up() {
        let line = line_cams();
        let (rec, _, _) = scene(&line);
        // Cameras were generated from `line` in order, so image i+1 sits at line[i].
        let src: Vec<Vec3> = (1..=line.len() as u32).map(|i| rec.projection_center(i).unwrap()).collect();
        assert!(degenerate_ratio(&line).is_some());
        let plain = umeyama(&src, &line, true).unwrap();
        let (bad, _) = ground_normal_error_deg(&rec, &plain);
        assert!(bad > 10.0, "plain Umeyama should leave the roll arbitrary, got {bad} deg");
        let (t, g) = fit_enu_sim3(&rec, &src, &line).unwrap();
        let g = g.unwrap();
        assert!(matches!(g.prior, GravityPrior::Ground { .. }), "{g:?}");
        let (err, above) = ground_normal_error_deg(&rec, &t);
        assert!(err < 3.0 && above, "ground normal off by {err} deg, cameras above: {above}");
        assert!((t.scale - plain.scale).abs() < 1e-9);
    }

    #[test]
    fn collinear_gps_view_direction_fallback() {
        let line = line_cams();
        let (rec, _, _) = scene(&line);
        // No points: the ground prior is unavailable, the optical axis decides.
        let ids = rec.point3d_ids();
        let mut bare = Reconstruction::new();
        bare.add_camera_own_rig(rec.camera(1).unwrap().clone()).unwrap();
        for i in rec.registered_images() {
            bare.add_image_own_frame(rec.image(i).unwrap().clone(), rec.world_to_cam(i)).unwrap();
            bare.register_image(i).unwrap();
        }
        assert!(!ids.is_empty());
        let src: Vec<Vec3> = (1..=line.len() as u32).map(|i| bare.projection_center(i).unwrap()).collect();
        let (t, g) = fit_enu_sim3(&bare, &src, &line).unwrap();
        assert_eq!(g.unwrap().prior, GravityPrior::ViewDirection);
        let mean: Vec3 =
            bare.registered_images().iter().map(|i| t.rotation.rotate(&bare.world_to_cam(*i).unwrap().viewing_direction())).sum();
        assert!(mean.z < 0.0);
        // The mean axis (perpendicular to the line) is sent to straight down, so it is clearly pointing down.
        assert!(mean.z / mean.norm() < -0.9);
    }

    #[test]
    fn align_to_gps_collinear_uses_gravity() {
        let line = line_cams();
        let (mut rec, names, _) = scene(&line);
        let enu = EnuFrame::new(37.5, 127.0, 50.0);
        let gps: Vec<GpsRecord> = names
            .iter()
            .zip(&line)
            .map(|(n, c)| {
                let (lat, lon, alt) = enu.enu_to_lla(c);
                GpsRecord { name: n.clone(), lat, lon, alt }
            })
            .collect();
        let opts = ModelAlignerOptions { origin: EnuOrigin::Explicit { lat: 37.5, lon: 127.0, alt: 50.0 }, ..Default::default() };
        let a = align_to_gps(&mut rec, &gps, &opts).unwrap();
        assert!(matches!(a.gravity.map(|g| g.prior), Some(GravityPrior::Ground { .. })));
        let (err, above) = ground_normal_error_deg(&rec, &Sim3::identity());
        assert!(err < 3.0 && above, "{err} {above}");
    }

    #[test]
    fn non_degenerate_gps_unchanged() {
        let mut rng = Lcg(3);
        let pos: Vec<Vec3> = (0..24).map(|_| Vec3::new(rng.r(0.0, 80.0), rng.r(0.0, 60.0), rng.r(30.0, 45.0))).collect();
        assert!(degenerate_ratio(&pos).is_none());
        let (rec, _, _) = scene(&pos);
        let src: Vec<Vec3> = (1..=pos.len() as u32).map(|i| rec.projection_center(i).unwrap()).collect();
        let (t, g) = fit_enu_sim3(&rec, &src, &pos).unwrap();
        assert!(g.is_none());
        assert_eq!(t, umeyama(&src, &pos, true).unwrap());
    }

    #[test]
    fn same_up_fit_keeps_roll_zero() {
        let line = line_cams();
        let moved: Vec<Vec3> = line.iter().map(|p| p * 1.5 + Vec3::new(3.0, 4.0, 5.0)).collect();
        let t = fit_same_up_sim3(&line, &moved).unwrap();
        assert!((t.rotation.rotate(&Vec3::z()) - Vec3::z()).norm() < 1e-6);
    }
}
