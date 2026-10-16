/*
 * display_delta.rs
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

//! In-memory display deltas on the synthetic flight in stable-display mode (3 cameras x 8 positions, zones 0..2).
//! The synthetic run has no dense stage, so the derive step turns every sparse zone model into a keyed "dense" cloud
//! (the same events a GPU run would produce, with `dense = true`); the deltas must rebuild exactly the display that
//! `LiveDisplay` composes from the same zone results.

mod common;
use common::{make_dataset, NPOS};
use cumulus3d_cli::compose::{CompositionHost, FrameKey, FrameState, LiveFrameSet, ReconNode};
use cumulus3d_cli::events::{Event, EventKind};
use cumulus3d_cli::live::{DeltaKind, DisplayFeed, DisplayMirror};
use cumulus3d_cli::pipeline::Pipeline;
use cumulus3d_cli::post::{LiveDisplay, LiveOptions};
use cumulus3d_cli::session::{FrameSet, Input, Session, SessionConfig};
use cumulus3d_core::io::PointCloud;
use cumulus3d_core::Reconstruction;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock};

const CAMS: [&str; 3] = ["camF", "camR", "camL"];

fn dataset() -> &'static Path {
    static DIR: OnceLock<PathBuf> = OnceLock::new();
    DIR.get_or_init(|| {
        let d = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("display_delta_src");
        let _ = std::fs::remove_dir_all(&d);
        make_dataset(&d);
        d
    })
}

fn frames(p: usize) -> FrameSet {
    FrameSet::new(CAMS.iter().map(|c| (c.to_string(), format!("{c}/{c}_{:04}.jpg", p * 3))))
}

fn config() -> SessionConfig {
    let src = dataset();
    let mut c = SessionConfig::new(src.join("images"));
    c.gps = cumulus3d_core::io::read_gps_file(src.join("gps_ref.txt")).unwrap();
    c.span = 3;
    c.overlap = 1;
    c.total_positions = Some(NPOS);
    c.seed = Some(7);
    c.stable_display = true;
    // Never settles by itself, forced at zone 2: zones 0 and 1 are re-placed (`replace`) once.
    c.settle_tol = 1e-12;
    c.settle_max_zone = 2;
    c
}

/// Keyed cloud of a zone model's sparse points (key = zone << 40 | point id).
fn cloud_of(zone: usize, m: &Reconstruction) -> PointCloud {
    let mut c = PointCloud::default();
    for (id, p) in m.points3d() {
        c.positions.push([p.xyz.x as f32, p.xyz.y as f32, p.xyz.z as f32]);
        c.colors.push(p.color);
        c.keys.push(((zone as u64) << 40) | id);
    }
    c
}

/// Turns `ZoneRefined { dense: false }` into a dense one with a keyed cloud.
fn densify(e: &Event) -> Event {
    match e {
        Event::ZoneRefined { meta, range, model, dense: false, .. } => Event::ZoneRefined {
            meta: meta.clone(),
            range: *range,
            cloud: Arc::new(cloud_of(range.zone, model)),
            model: Arc::clone(model),
            dense: true,
        },
        e => e.clone(),
    }
}

type Reference = Arc<Mutex<Vec<(PointCloud, Vec<usize>)>>>;

fn sorted(it: impl Iterator<Item = ([f32; 3], [u8; 3], u8)>) -> Vec<([u32; 3], [u8; 3], u8)> {
    let mut v: Vec<_> = it.map(|(p, c, z)| (p.map(f32::to_bits), c, z)).collect();
    v.sort();
    v
}

#[test]
fn deltas_rebuild_the_composed_display_in_stable_mode() {
    let opts = LiveOptions { stride: 2, ..LiveOptions::default() };
    let mut feed = DisplayFeed::new(opts.clone());
    // Reference composition from the same zone results, written independently of DisplayFeed.
    let reference: Reference = Arc::default();
    let r2 = reference.clone();
    let mut live = LiveDisplay::new(LiveOptions { relocate: false, ..opts });
    let mut clouds: BTreeMap<usize, Arc<PointCloud>> = BTreeMap::new();
    let mut models: BTreeMap<usize, Arc<Reconstruction>> = BTreeMap::new();

    let pl = Pipeline::new(Session::new(config())).sync(true).derive(move |e| {
        let e = densify(e);
        match &e {
            Event::ZoneRefinedPose { range, model, .. } => {
                models.insert(range.zone, model.clone());
                live.forget(range.zone);
            }
            Event::ZoneRefined { range, cloud, dense: true, .. } => {
                clouds.insert(range.zone, cloud.clone());
                let c: BTreeMap<usize, &PointCloud> = clouds.iter().map(|(k, v)| (*k, &**v)).collect();
                let m: BTreeMap<usize, &Reconstruction> = models.iter().map(|(k, v)| (*k, &**v)).collect();
                r2.lock().unwrap().push(live.compose_zoned(&c, &m));
            }
            _ => {}
        }
        feed.handle(&e).into_iter().collect()
    });
    let rx = pl.subscribe();
    let mut pl = pl;
    for p in 0..NPOS {
        pl.push(Input::Frames(frames(p)));
    }
    let (s, summary) = pl.finish();
    assert!(s.is_finished());
    // The derived events are counted like any other and arrive right after their cause.
    let evs: Vec<Event> = rx.try_iter().collect();
    let n = summary.by_kind.get(&EventKind::DisplayDelta).copied().unwrap_or(0);
    assert_eq!(evs.iter().filter(|e| e.kind() == EventKind::DisplayDelta).count(), n);
    assert_eq!(evs.len(), summary.events);

    let mut mirror = DisplayMirror::new();
    let mut kinds = Vec::new();
    let mut shown = 0;
    let refs = reference.lock().unwrap();
    for (i, e) in evs.iter().enumerate() {
        let Event::DisplayDelta { delta, zone, kind, frame, .. } = e else { continue };
        kinds.push((*zone, *kind));
        assert!(frame.origin_gps.is_none() && !frame.enu);
        mirror.apply(delta).unwrap();
        assert_eq!(mirror.len(), delta.total_points);
        if *kind == DeltaKind::Final {
            assert!(delta.is_empty());
            assert!(matches!(
                evs[i - 1].kind(),
                EventKind::AllRefinedDone | EventKind::BaseAdopted | EventKind::Log | EventKind::ZoneRefined
            ));
            continue;
        }
        assert_eq!(evs[i - 1].kind(), EventKind::ZoneRefined, "delta follows its cause");
        let (cloud, zones) = &refs[shown];
        shown += 1;
        let got = sorted(mirror.points().map(|(_, p)| (p.xyz, p.rgb, p.zone)));
        let want = sorted((0..cloud.len()).map(|j| (cloud.positions[j], cloud.colors[j], zones[j] as u8)));
        assert_eq!(got, want, "delta {} does not rebuild the composed display", delta.seq);
        assert!(!want.is_empty());
    }
    assert_eq!(shown, refs.len());
    // Zones 0, 1 first appear refined, then are re-placed by the forced settle at zone 2; last delta is the final one.
    assert!(kinds.contains(&(0, DeltaKind::Replace)) && kinds.contains(&(1, DeltaKind::Replace)), "{kinds:?}");
    assert_eq!(kinds.iter().filter(|(_, k)| *k == DeltaKind::Refined).count(), 3);
    assert_eq!(kinds.last().map(|(_, k)| *k), Some(DeltaKind::Final));
}

fn node_images() -> PathBuf {
    let dst = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("display_delta_node");
    let _ = std::fs::remove_dir_all(&dst);
    for cam in CAMS {
        let (s, d) = (dataset().join("images").join(cam), dst.join(cam));
        std::fs::create_dir_all(&d).unwrap();
        for e in std::fs::read_dir(s).unwrap() {
            let e = e.unwrap();
            std::fs::copy(e.path(), d.join(e.file_name())).unwrap();
        }
    }
    dst
}

#[test]
fn compose_node_accepts_stable_display_and_display_deltas() {
    let host = CompositionHost::new();
    let got = Arc::new(Mutex::new(Vec::new()));
    let g = got.clone();
    let node: ReconNode = ReconNode::declare()
        .id("sky")
        .images(node_images())
        .cameras(CAMS)
        .zones(3, 1)
        .seed(7)
        .stable_display(true)
        .fixed_enu_origin(true)
        .display_deltas(LiveOptions { stride: 3, ..LiveOptions::default() })
        .on(EventKind::DisplayDelta, move |e| g.lock().unwrap().push(e.clone()))
        .sync_hooks(true)
        .build(&host)
        .unwrap();
    let rx = node.subscribe().unwrap();
    let gps = cumulus3d_core::io::read_gps_file(dataset().join("gps_ref.txt")).unwrap();
    for p in 0..NPOS {
        let name = |c: &str| format!("{c}/{c}_{:04}.jpg", p * 3);
        let mut set = LiveFrameSet::new(FrameKey::new("cam", p as u64, 0));
        for c in CAMS {
            let g = gps.iter().find(|g| g.name == name(c)).unwrap();
            set = set.file(c, name(c)).gps_fix(c, g.lat, g.lon, g.alt);
        }
        node.compose(FrameState::ready(set)).unwrap();
    }
    node.close().unwrap();
    // No dense stage here, so there is nothing to diff (and no delta, no warning about it); the stable-display session ran.
    assert!(rx.try_iter().all(|e| e.kind() != EventKind::DisplayDelta));
    assert!(got.lock().unwrap().is_empty());
    let bad = ReconNode::declare()
        .id("bad")
        .images(node_images())
        .cameras(CAMS)
        .display_deltas(LiveOptions { stride: 0, ..LiveOptions::default() });
    assert!(bad.build(&host).is_err());
}
