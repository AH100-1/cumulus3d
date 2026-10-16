/*
 * sift.rs
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

//! CUDA implementation of `SiftEngine` (hybrid): the Gaussian scale space (normalization, 2x upsampling, separable blur, downscaling) is
//! made bit-identical to
//! the CPU on the GPU, octaves are downloaded only when needed (coarse ones first), and DoG, detection, orientation and descriptors are
//! computed with the CPU functions of cumulus3d-features. So the result equals `CpuSift`.
//! If the maximum feature count is reached first, the fine octave (2x upsampled resolution) is not even downloaded.

use crate::device::{CudaDevice, GpuError};
use cudarc::driver::{CudaFunction, CudaSlice, CudaStream, LaunchConfig, PinnedHostSlice, PushKernelArg};
use cumulus3d_core::{Descriptors, Error, Result};
use cumulus3d_features::sift::detect::{self, Candidate, DetectParams};
use cumulus3d_features::sift::orient::{self, Gradient, OrientParams};
use cumulus3d_features::sift::pyramid::{self, ScaleSpace};
use cumulus3d_features::{CpuSift, FeatureSelection, GrayImage, SiftEngine, SiftFeature, SiftOptions, SiftOutput};
use rayon::prelude::*;
use std::sync::{Arc, Mutex};

pub(crate) const SRC: &str = include_str!("kernels/sift.cu");
const TILE_W: u32 = 128;

/// CUDA SIFT backend.
pub struct CudaSift {
    dev: Arc<CudaDevice>,
    f_norm: CudaFunction,
    f_up: CudaFunction,
    f_bh: CudaFunction,
    f_bv: CudaFunction,
    f_dec: CudaFunction,
    /// Device use serialization (stream shared) + pinned host buffers for download (reused between images).
    lock: Mutex<HostBufs>,
}

/// Host buffers reused between images (pinned-memory pyramid, DoG work buffer).
#[derive(Default)]
struct HostBufs {
    pinned: Option<PinnedHostSlice<f32>>,
    dogs: Vec<Vec<f32>>,
}

/// Octave on the device.
struct DevOctave {
    o: i32,
    w: usize,
    h: usize,
    gauss: Vec<CudaSlice<f32>>,
}

/// Downloaded octave (filled only when needed).
/// Levels point to ranges of the pinned host buffer (passed as slices to CPU functions without copying).
struct HostOctave {
    o: i32,
    w: usize,
    h: usize,
    ranges: Vec<std::ops::Range<usize>>,
    fetched: bool,
}

impl HostOctave {
    fn level<'a>(&self, hb: &'a [f32], l: i32) -> &'a [f32] {
        &hb[self.ranges[(l + 1) as usize].clone()]
    }
}

struct LevelFeats {
    oi: usize,
    j: usize,
    items: Vec<(Candidate, u16)>,
}

fn cfg2(w: usize, h: usize) -> LaunchConfig {
    LaunchConfig { grid_dim: ((w as u32).div_ceil(32), (h as u32).div_ceil(8), 1), block_dim: (32, 8, 1), shared_mem_bytes: 0 }
}

/// Separates input errors (returned as is) from device errors (computed on the CPU instead).
enum Fail {
    Input(Error),
    Device(GpuError),
}

impl From<GpuError> for Fail {
    fn from(e: GpuError) -> Self {
        Fail::Device(e)
    }
}

impl CudaSift {
    /// Compiles the scale-space kernels on device `dev` and builds it.
    pub fn new(dev: Arc<CudaDevice>) -> std::result::Result<Self, GpuError> {
        // Turn off fused multiply-add and use IEEE division: bit-identical to the CPU pyramid.
        let m = dev.compile(SRC, false, &[])?;
        Ok(Self {
            f_norm: m.load_function("sift_normalize")?,
            f_up: m.load_function("sift_upsample2")?,
            f_bh: m.load_function("sift_blur_h")?,
            f_bv: m.load_function("sift_blur_v")?,
            f_dec: m.load_function("sift_decimate2")?,
            dev,
            lock: Mutex::new(HostBufs::default()),
        })
    }

    /// Builds with device 0.
    pub fn try_default() -> std::result::Result<Self, GpuError> {
        Self::new(CudaDevice::new(0)?)
    }

    fn blur(
        &self,
        src: &CudaSlice<f32>,
        dst: &mut CudaSlice<f32>,
        tmp: &mut CudaSlice<f32>,
        w: usize,
        h: usize,
        sigma: f32,
    ) -> std::result::Result<(), GpuError> {
        let s = &self.dev.stream;
        let k = pyramid::gaussian_kernel(sigma);
        let r = (k.len() / 2) as i32;
        let kd = s.clone_htod(&k)?;
        let (wi, hi) = (w as i32, h as i32);
        {
            let cfg = LaunchConfig { grid_dim: ((w as u32).div_ceil(TILE_W), h as u32, 1), block_dim: (TILE_W, 1, 1), shared_mem_bytes: 0 };
            let mut l = s.launch_builder(&self.f_bh);
            l.arg(src).arg(&mut *tmp).arg(&wi).arg(&hi).arg(&kd).arg(&r);
            // SAFETY: sift_blur_h(src[w·h], tmp[w·h], w, h, k[2r+1], r), r ≤ 16.
            unsafe { l.launch(cfg) }?;
        }
        let mut l = s.launch_builder(&self.f_bv);
        l.arg(&*tmp).arg(dst).arg(&wi).arg(&hi).arg(&kd).arg(&r);
        // SAFETY: same type as sift_blur_v.
        unsafe { l.launch(cfg2(w, h)) }?;
        Ok(())
    }

    /// GPU pyramid (same rules as CPU `pyramid::build_pyramid`).
    fn build(
        &self,
        s: &Arc<CudaStream>,
        image: &GrayImage,
        w: usize,
        h: usize,
        opts: &SiftOptions,
        ss: ScaleSpace,
    ) -> std::result::Result<Vec<DevOctave>, GpuError> {
        let src = s.clone_htod(&image.data)?;
        let mut base = s.alloc_zeros::<f32>(w * h)?;
        {
            let (sw, wi, hi) = (image.width as i32, w as i32, h as i32);
            let mut l = s.launch_builder(&self.f_norm);
            l.arg(&src).arg(&sw).arg(&wi).arg(&hi).arg(&mut base);
            // SAFETY: sift_normalize(src[sw·h], sw, w, h, dst[w·h]).
            unsafe { l.launch(cfg2(w, h)) }?;
        }
        let o_min = opts.first_octave;
        let (w0, h0, src0) = if o_min == -1 {
            let mut up = s.alloc_zeros::<f32>(4 * w * h)?;
            let (wi, hi) = (w as i32, h as i32);
            let mut l = s.launch_builder(&self.f_up);
            l.arg(&base).arg(&wi).arg(&hi).arg(&mut up);
            // SAFETY: sift_upsample2(src[w·h], w, h, dst[4·w·h]).
            unsafe { l.launch(cfg2(w, 2 * h)) }?;
            (2 * w, 2 * h, up)
        } else {
            (w, h, base)
        };
        let n_oct = opts.num_octaves.unwrap_or_else(|| pyramid::auto_num_octaves(w0, h0)).max(1);
        let mut tmp = s.alloc_zeros::<f32>(w0 * h0)?;
        let mut octs: Vec<DevOctave> = Vec::with_capacity(n_oct);
        let (mut w, mut h) = (w0, h0);
        for oi in 0..n_oct {
            let o = o_min + oi as i32;
            let mut gauss: Vec<CudaSlice<f32>> = Vec::with_capacity(ss.s + 3);
            if oi == 0 {
                let s_first = ss.sigma(-1.0);
                let sn = 0.5 / 2f32.powi(o_min);
                if s_first > sn {
                    let mut g0 = s.alloc_zeros::<f32>(w * h)?;
                    self.blur(&src0, &mut g0, &mut tmp, w, h, (s_first * s_first - sn * sn).sqrt())?;
                    gauss.push(g0);
                } else {
                    gauss.push(s.clone_dtod(&src0)?);
                }
            } else {
                let prev = &octs[oi - 1];
                let (pw, ph) = (prev.w, prev.h);
                let (nw, nh) = (pw / 2, ph / 2);
                let mut g0 = s.alloc_zeros::<f32>(nw * nh)?;
                let (pwi, nwi, nhi) = (pw as i32, nw as i32, nh as i32);
                let mut l = s.launch_builder(&self.f_dec);
                // Previous octave level S−1 (gauss index S).
                l.arg(&prev.gauss[ss.s]).arg(&pwi).arg(&mut g0).arg(&nwi).arg(&nhi);
                // SAFETY: sift_decimate2(src[pw·ph], pw, dst[nw·nh], nw, nh).
                unsafe { l.launch(cfg2(nw, nh)) }?;
                w = nw;
                h = nh;
                gauss.push(g0);
            }
            for l in 0..=(ss.s as i32 + 1) {
                let mut g = s.alloc_zeros::<f32>(w * h)?;
                let last = gauss.last().expect("레벨 있음");
                self.blur(last, &mut g, &mut tmp, w, h, ss.sigma_inc(l))?;
                gauss.push(g);
            }
            octs.push(DevOctave { o, w, h, gauss });
            if w / 2 < 4 || h / 2 < 4 {
                break;
            }
        }
        Ok(octs)
    }

    fn fetch(s: &Arc<CudaStream>, d: &DevOctave, h: &HostOctave, hb: &mut PinnedHostSlice<f32>) -> std::result::Result<(), GpuError> {
        let m = hb.as_mut_slice()?;
        for (g, r) in d.gauss.iter().zip(&h.ranges) {
            s.memcpy_dtoh(g, &mut m[r.clone()])?;
        }
        s.synchronize()?;
        Ok(())
    }

    fn extract_gpu(&self, image: &GrayImage, opts: &SiftOptions) -> std::result::Result<SiftOutput, Fail> {
        if opts.octave_resolution == 0 {
            return Err(Fail::Input(Error::InvalidArgument("octave_resolution 은 1 이상".into())));
        }
        if opts.first_octave != -1 && opts.first_octave != 0 {
            return Err(Fail::Input(Error::Unsupported(format!("first_octave {} (−1, 0 만 지원)", opts.first_octave))));
        }
        let w = if opts.truncate_width_to_4 { image.width - image.width % 4 } else { image.width };
        let h = image.height;
        if w < 8 || h < 8 {
            return Ok(SiftOutput::default());
        }
        let ss = ScaleSpace::new(opts.octave_resolution);
        let mut guard = self.lock.lock().unwrap_or_else(|e| e.into_inner());
        let st = self.dev.stream.clone();
        let t0 = std::time::Instant::now();
        let dev_pyr = self.build(&st, image, w, h, opts, ss)?;
        st.synchronize().map_err(GpuError::from)?;
        let t_build = t0.elapsed();
        let mut t_fetch = std::time::Duration::ZERO;
        let mut off = 0usize;
        let mut pyr: Vec<HostOctave> = dev_pyr
            .iter()
            .map(|d| {
                let ranges = (0..d.gauss.len())
                    .map(|_| {
                        off += d.w * d.h;
                        off - d.w * d.h..off
                    })
                    .collect();
                HostOctave { o: d.o, w: d.w, h: d.h, ranges, fetched: false }
            })
            .collect();
        let bufs = &mut *guard;
        if bufs.pinned.as_ref().is_none_or(|b| b.len() < off) {
            bufs.pinned = None;
            // SAFETY: reads only the ranges the device filled (octaves marked fetched).
            bufs.pinned = Some(unsafe { self.dev.ctx.alloc_pinned_with_flags::<f32>(off, 0) }.map_err(GpuError::from)?);
        }
        let hbuf = bufs.pinned.as_mut().expect("채움");
        let dog_pool = &mut bufs.dogs;

        let s = ss.s;
        let k_max = if opts.max_num_features == 0 { usize::MAX } else { opts.max_num_features };
        let dp = DetectParams {
            peak_threshold: opts.peak_threshold,
            edge_threshold: opts.edge_threshold,
            refinement_iterations: opts.refinement_iterations,
            reject_singular: opts.reject_singular_refinement,
            sigma0: ss.sigma0,
            k: ss.k,
        };
        let op = OrientParams { max_num_orientations: opts.max_num_orientations, bin_interpolation: opts.orientation_bin_interpolation };

        // Same order as CpuSift::extract: coarse octaves and high levels first, omit levels that get truncated.
        let mut levels: Vec<LevelFeats> = Vec::new();
        let (mut cum_det, mut cum_or) = (0usize, 0usize);
        'outer: for oi in (0..pyr.len()).rev() {
            let (ow, oh) = (pyr[oi].w, pyr[oi].h);
            let mut dogs: Vec<Vec<f32>> = Vec::new();
            let give = |dogs: Vec<Vec<f32>>, pool: &mut Vec<Vec<f32>>| pool.extend(dogs);
            for j in (0..s).rev() {
                let stop = match opts.selection {
                    FeatureSelection::CompatLevels => cum_det > k_max,
                    FeatureSelection::TopK => cum_or >= k_max,
                    FeatureSelection::SpatialGrid { .. } => false,
                };
                if stop {
                    give(dogs, dog_pool);
                    break 'outer;
                }
                if dogs.is_empty() {
                    if !pyr[oi].fetched {
                        let tf = std::time::Instant::now();
                        Self::fetch(&st, &dev_pyr[oi], &pyr[oi], hbuf)?;
                        pyr[oi].fetched = true;
                        t_fetch += tf.elapsed();
                    }
                    let hb = hbuf.as_slice().map_err(GpuError::from)?;
                    for d in 0..s + 2 {
                        let mut buf = dog_pool.pop().unwrap_or_default();
                        buf.resize(ow * oh, 0.0);
                        buf.truncate(ow * oh);
                        detect::dog(pyr[oi].level(hb, d as i32), pyr[oi].level(hb, d as i32 - 1), &mut buf);
                        dogs.push(buf);
                    }
                }
                let cands = detect::detect_level(&dogs[j], &dogs[j + 1], &dogs[j + 2], ow, oh, j, &dp);
                cum_det += cands.len();
                let items: Vec<(Candidate, u16)> = if opts.upright {
                    cands.into_iter().map(|c| (c, 0u16)).collect()
                } else {
                    let hb = hbuf.as_slice().map_err(GpuError::from)?;
                    let grad = Gradient::lazy(pyr[oi].level(hb, j as i32), ow, oh);
                    cands
                        .par_iter()
                        .flat_map_iter(|c| orient::orientations(&grad, c.x, c.y, c.sigma, &op).into_iter().map(move |q| (*c, q)))
                        .collect()
                };
                cum_or += items.len();
                levels.push(LevelFeats { oi, j, items });
            }
            give(dogs, dog_pool);
        }
        drop(dev_pyr);
        let t_detect = t0.elapsed();

        select(&mut levels, opts.selection, k_max, &pyr);

        levels.sort_by_key(|l| (l.oi, l.j));
        let total: usize = levels.iter().map(|l| l.items.len()).sum();
        let mut features = Vec::with_capacity(total);
        let mut desc = Descriptors::with_capacity(total);
        let hb = hbuf.as_slice().map_err(GpuError::from)?;
        for lv in &levels {
            if lv.items.is_empty() {
                continue;
            }
            let oct = &pyr[lv.oi];
            let g = oct.level(hb, lv.j as i32);
            let sig = ss.sigma(lv.j as f32);
            let work = lv.items.len() as f64 * 225.0 * (sig as f64).powi(2);
            let grad = if work > (oct.w * oct.h) as f64 { Gradient::precomputed(g, oct.w, oct.h) } else { Gradient::lazy(g, oct.w, oct.h) };
            let ds: Vec<[u8; 128]> = lv
                .items
                .par_iter()
                .map(|(c, q)| orient::descriptor(&grad, c.x, c.y, c.sigma, orient::dequantize_angle(*q), opts.normalization))
                .collect();
            let scale_o = 2f32.powi(oct.o);
            for ((c, q), d) in lv.items.iter().zip(&ds) {
                let th = orient::dequantize_angle(*q);
                let ori = (std::f32::consts::TAU - th).rem_euclid(std::f32::consts::TAU);
                features.push(SiftFeature {
                    x: scale_o * (c.x - 0.5) + 0.5,
                    y: scale_o * (c.y - 0.5) + 0.5,
                    scale: scale_o * c.sigma,
                    orientation: ori,
                    octave: oct.o,
                    level: lv.j as u32,
                    response: c.response,
                });
                desc.push(d);
            }
        }
        if std::env::var_os("CUMULUS3D_CUDA_TRACE").is_some() {
            let fetched: Vec<i32> = pyr.iter().filter(|o| o.fetched).map(|o| o.o).collect();
            eprintln!(
                "cuda sift: 피라미드 {t_build:.2?}, 내림 {t_fetch:.2?} (옥타브 {fetched:?}), 검출·방향 {:.2?}, 기술자 {:.2?}",
                t_detect - t_build - t_fetch,
                t0.elapsed() - t_detect
            );
        }
        Ok(SiftOutput { features, descriptors: desc })
    }
}

/// Maximum feature count limit (same rule as CpuSift). `levels` is in coarse → fine order.
fn select(levels: &mut [LevelFeats], mode: FeatureSelection, k: usize, pyr: &[HostOctave]) {
    let total: usize = levels.iter().map(|l| l.items.len()).sum();
    if total <= k {
        return;
    }
    match mode {
        FeatureSelection::CompatLevels => {
            let mut total = total;
            for lv in levels.iter_mut().rev() {
                if total - lv.items.len() > k {
                    total -= lv.items.len();
                    lv.items.clear();
                } else {
                    break;
                }
            }
        }
        FeatureSelection::TopK => {
            let mut remaining = k;
            for lv in levels.iter_mut() {
                if lv.items.len() <= remaining {
                    remaining -= lv.items.len();
                    continue;
                }
                let mut idx: Vec<usize> = (0..lv.items.len()).collect();
                idx.sort_by(|&a, &b| lv.items[b].0.response.abs().total_cmp(&lv.items[a].0.response.abs()).then(a.cmp(&b)));
                let mut keep = vec![false; lv.items.len()];
                for &i in idx.iter().take(remaining) {
                    keep[i] = true;
                }
                let mut it = keep.iter();
                lv.items.retain(|_| *it.next().unwrap_or(&false));
                remaining = 0;
            }
        }
        FeatureSelection::SpatialGrid { cells } => {
            let cells = cells.max(1) as usize;
            let o0 = &pyr[0];
            let sc = 2f32.powi(o0.o);
            let (iw, ih) = (o0.w as f32 * sc, o0.h as f32 * sc);
            let mut buckets: Vec<Vec<(f32, usize, usize)>> = vec![Vec::new(); cells * cells];
            for (li, lv) in levels.iter().enumerate() {
                let so = 2f32.powi(pyr[lv.oi].o);
                for (ii, (c, _)) in lv.items.iter().enumerate() {
                    let x = so * (c.x - 0.5) + 0.5;
                    let y = so * (c.y - 0.5) + 0.5;
                    let cx = ((x / iw * cells as f32) as usize).min(cells - 1);
                    let cy = ((y / ih * cells as f32) as usize).min(cells - 1);
                    buckets[cy * cells + cx].push((c.response.abs(), li, ii));
                }
            }
            for b in &mut buckets {
                b.sort_by(|a, b| b.0.total_cmp(&a.0).then((a.1, a.2).cmp(&(b.1, b.2))));
            }
            let mut keep: Vec<Vec<bool>> = levels.iter().map(|l| vec![false; l.items.len()]).collect();
            let mut taken = 0;
            let mut round = 0;
            while taken < k {
                let mut any = false;
                for b in &buckets {
                    if let Some(&(_, li, ii)) = b.get(round) {
                        any = true;
                        if taken < k {
                            keep[li][ii] = true;
                            taken += 1;
                        }
                    }
                }
                if !any {
                    break;
                }
                round += 1;
            }
            for (lv, kp) in levels.iter_mut().zip(keep) {
                let mut it = kp.into_iter();
                lv.items.retain(|_| it.next().unwrap_or(false));
            }
        }
    }
}

impl SiftEngine for CudaSift {
    fn name(&self) -> &str {
        "cuda"
    }
    fn extract(&self, image: &GrayImage, options: &SiftOptions) -> Result<SiftOutput> {
        match self.extract_gpu(image, options) {
            Ok(o) => Ok(o),
            Err(Fail::Input(e)) => Err(e),
            Err(Fail::Device(e)) => {
                eprintln!("경고: CUDA SIFT 실패({e}), CPU SIFT 로 계산");
                CpuSift::new().extract(image, options)
            }
        }
    }
}
