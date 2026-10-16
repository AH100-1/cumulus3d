/*
 * traversal_tuning.rs
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
 * SPDX-License-Identifier: Apache-2.0
 */

//! Traversal fusion on ground-truth depth maps of the synthetic scene (CPU): the fast-profile tolerances are tuned for agreement
//! fusion and must not shrink the extended (traversal) fusion.
use cumulus3d_dense::fusion::{fuse, FusionInput};
use cumulus3d_dense::neighbors::PairStats;
use cumulus3d_dense::synthetic::{make_scene, SynthConfig};
use cumulus3d_dense::{DensifyOptions, FusionMode, FusionParams};

fn run(p: &FusionParams, amp: f32) -> usize {
    let s = make_scene(&SynthConfig::default());
    let st = PairStats::new(&s.scene);
    let overlap: Vec<Vec<usize>> = (0..s.scene.views.len()).map(|v| st.select(v, p.check_num_images, 0.0)).collect();
    let depth: Vec<Vec<f32>> = s
        .depth
        .iter()
        .enumerate()
        .map(|(v, d)| {
            d.iter()
                .enumerate()
                .map(|(i, &x)| {
                    let h = (i as u64 ^ ((v as u64) << 40)).wrapping_mul(0x9e37_79b9_7f4a_7c15);
                    let u = ((h >> 40) as f32 / (1u64 << 24) as f32) * 2.0 - 1.0;
                    x * (1.0 + amp * u)
                })
                .collect()
        })
        .collect();
    let inputs: Vec<Option<FusionInput>> = depth.iter().zip(&s.normal).map(|(d, n)| Some(FusionInput::plain(d, n))).collect();
    fuse(&s.scene, &inputs, &overlap, p, 1).cloud.len()
}

#[test]
fn traversal_keeps_its_point_count_under_the_fast_profile() {
    let untuned = FusionParams { mode: FusionMode::Traversal, ..FusionParams::default() };
    let mut o = DensifyOptions::default();
    o.set_fusion_mode(FusionMode::Traversal);
    // Selecting traversal restores the untuned tolerances (same fusion as before the fast-profile tuning).
    assert_eq!(o.fusion.max_reproj_error, untuned.max_reproj_error);
    assert_eq!(o.fusion.max_depth_error, untuned.max_depth_error);
    assert_eq!(o.fusion.max_normal_error_deg, untuned.max_normal_error_deg);
    let n = run(&o.fusion, 0.003);
    assert_eq!(n, run(&untuned, 0.003));
    assert!(n > 10_000, "{n}");
    // The tuned agreement-fusion tolerances (100 px) would collapse it: that is the regression this test guards.
    let tuned = FusionParams { mode: FusionMode::Traversal, ..DensifyOptions::default().fusion };
    assert!(run(&tuned, 0.003) * 2 < n);
}
