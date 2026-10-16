/*
 * display_scale.rs
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

//! A stable display with GPS is metric: the span of the shown camera centres matches the span of the GPS positions (within 5 %)
//! for every combination of backlog batching, feature lookahead and zone previews, and every zone yields exactly one `Refined`
//! display delta.

mod common;
use common::make_dataset_n;

const NPOS: usize = 14;
use cumulus3d_cli::events::Event;
use cumulus3d_cli::intrinsics::IntrinsicsPrior;
use cumulus3d_cli::live::{DeltaKind, DisplayFeed};
use cumulus3d_cli::pipeline::Pipeline;
use cumulus3d_cli::post::LiveOptions;
use cumulus3d_cli::session::{FrameSet, Input, Session, SessionConfig};
use cumulus3d_core::io::PointCloud;
use cumulus3d_core::{Reconstruction, Vec3};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};

const CAMS: [&str; 3] = ["camF", "camR", "camL"];

fn dataset() -> &'static Path {
    static DIR: OnceLock<PathBuf> = OnceLock::new();
    DIR.get_or_init(|| {
        let d = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("display_scale_src");
        let _ = std::fs::remove_dir_all(&d);
        make_dataset_n(&d, NPOS);
        d
    })
}
/// Largest distance between two camera centres of the model (the extent of the flight).
fn span(m: &Reconstruction) -> f64 {
    let c: Vec<Vec3> = m.registered_images().into_iter().filter_map(|id| m.projection_center(id)).collect();
    let mut s: f64 = 0.0;
    for a in &c {
        for b in &c {
            s = s.max((a - b).norm());
        }
    }
    s
}

/// The same extent from the GPS fixes of the images registered in the model.
fn gps_span(m: &Reconstruction) -> f64 {
    let gps = cumulus3d_core::io::read_gps_file(dataset().join("gps_ref.txt")).unwrap();
    let o = &gps[0];
    let frame = cumulus3d_align::EnuFrame::new(o.lat, o.lon, o.alt);
    let by: HashMap<&str, Vec3> = gps.iter().map(|g| (g.name.as_str(), frame.lla_to_enu(g.lat, g.lon, g.alt))).collect();
    let c: Vec<Vec3> = m.registered_images().into_iter().filter_map(|id| by.get(m.image(id)?.name.as_str()).copied()).collect();
    let mut s: f64 = 0.0;
    for a in &c {
        for b in &c {
            s = s.max((a - b).norm());
        }
    }
    s
}

fn frames(p: usize) -> FrameSet {
    FrameSet::new(CAMS.iter().map(|c| (c.to_string(), format!("{c}/{c}_{:04}.jpg", p * 3))))
}

/// Turns `ZoneRefined { dense: false }` into a dense one with a keyed cloud (the synthetic run has no dense stage).
fn densify(e: &Event) -> Event {
    match e {
        Event::ZoneRefined { meta, range, model, dense: false, .. } => {
            let mut c = PointCloud::default();
            for (id, p) in model.points3d() {
                c.positions.push([p.xyz.x as f32, p.xyz.y as f32, p.xyz.z as f32]);
                c.colors.push(p.color);
                c.keys.push(((range.zone as u64) << 40) | id);
            }
            Event::ZoneRefined { meta: meta.clone(), range: *range, cloud: Arc::new(c), model: Arc::clone(model), dense: true }
        }
        e => e.clone(),
    }
}

/// Bundle adjustment that takes its time (a refinement slower than the foreground, as on real imagery).
struct SlowBa(u64);

impl cumulus3d_ba::BaBackend for SlowBa {
    fn name(&self) -> &str {
        "slow"
    }
    fn solve(
        &self,
        input: &cumulus3d_ba::ProblemInput,
        opts: &cumulus3d_ba::TrOptions,
    ) -> cumulus3d_core::Result<cumulus3d_ba::BaSolution> {
        std::thread::sleep(std::time::Duration::from_millis(self.0));
        cumulus3d_ba::CpuBaBackend.solve(input, opts)
    }
}

fn run(tag: &str, batch: usize, lookahead: usize, preview: bool, slow_ms: u64) {
    let mut cfg = SessionConfig::new(dataset().join("images"));
    cfg.gps = cumulus3d_core::io::read_gps_file(dataset().join("gps_ref.txt")).unwrap();
    cfg.span = 4;
    cfg.overlap = 1;
    cfg.total_positions = Some(NPOS);
    cfg.seed = Some(7);
    cfg.stable_display = true;
    cfg.fixed_enu_origin = true;
    cfg.settle_tol = 1e-12;
    cfg.settle_max_zone = 2;
    cfg.lookahead = lookahead;
    cfg.preview = preview;
    let mut prior = IntrinsicsPrior::new();
    for c in CAMS {
        prior = prior.opencv(c, [384.0, 384.0, 160.0, 120.0, 0.0, 0.0, 0.0, 0.0]);
    }
    cfg.intrinsics_prior = Some(Arc::new(prior));
    if slow_ms > 0 {
        cfg.ba_backend = Arc::new(SlowBa(slow_ms));
    }
    let mut feed = DisplayFeed::new(LiveOptions { stride: 1, ..LiveOptions::default() });
    let pl = Pipeline::new(Session::new(cfg)).sync(true).derive(move |e| feed.handle(&densify(e)).into_iter().collect());
    let rx = pl.subscribe();
    let mut pl = pl;
    let la = pl.reducer().lookahead();
    let (mut p, mut cut) = (0, 0);
    while p < NPOS {
        let n = if batch == 0 { [1usize, 2, 4, 1, 3][cut % 5] } else { batch }.min(NPOS - p);
        cut += 1;
        for q in p..(p + lookahead.max(n)).min(NPOS) {
            if lookahead == 0 || !la.hint(&frames(q)) {
                break;
            }
        }
        if n == 1 {
            pl.push(Input::Frames(frames(p)));
        } else {
            pl.push(Input::Batch((p..p + n).map(frames).collect()));
        }
        p += n;
    }
    let (s, _) = pl.finish();
    assert!(s.failed().is_none(), "{tag}: {:?}", s.failed());
    let evs: Vec<Event> = rx.try_iter().collect();
    let mut refined_models = Vec::new();
    let mut deltas: HashMap<usize, usize> = HashMap::new();
    let mut logs = Vec::new();
    for e in &evs {
        match e {
            Event::ZoneRefined { model, .. } => refined_models.push(model.clone()),
            Event::DisplayDelta { zone, kind, .. } if *kind == DeltaKind::Refined => *deltas.entry(*zone).or_default() += 1,
            Event::Log { line, .. } if line.contains("[stable]") || line.contains("[preview]") => logs.push(line.clone()),
            _ => {}
        }
    }
    assert!(!refined_models.is_empty(), "{tag}: no refined zone");
    for m in &refined_models {
        let (a, b) = (span(m), gps_span(m));
        assert!(b > 1.0, "{tag}: gps span {b}");
        assert!((a - b).abs() <= 0.05 * b, "{tag}: camera span {a} vs GPS span {b}\n{logs:#?}");
    }
    for z in 0..deltas.len() {
        assert!(deltas.get(&z).copied() == Some(1), "{tag}: zone {z} Refined deltas {deltas:?}");
    }
}

#[test]
fn display_scale_matches_gps_in_every_live_mode() {
    for batch in [1usize, 3, 0] {
        for lookahead in [0usize, 4] {
            for preview in [false, true] {
                for slow_ms in if batch == 3 && lookahead == 4 { vec![0u64, 400] } else { vec![0u64] } {
                    run(&format!("b{batch}_l{lookahead}_p{}_s{slow_ms}", preview as u8), batch, lookahead, preview, slow_ms);
                }
            }
        }
    }
}
