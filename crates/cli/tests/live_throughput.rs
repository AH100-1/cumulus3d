/*
 * live_throughput.rs
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

//! Live throughput on the synthetic flight (3 cameras x 8 positions, densification off): backlog batches equal position by
//! position within tolerance, lookahead extraction changes nothing but the timing, quick previews of a stable display are placed
//! in the display frame and replaced by the refined zone, and a spawned node batches its backlog. The `#[ignore]`d
//! `bench_live_throughput` prints the per-position times of the variants (`cargo test --release -p cumulus3d-cli --test
//! live_throughput bench -- --ignored --nocapture`).

mod common;
use common::{make_dataset, make_dataset_n, NPOS};
use cumulus3d_cli::compose::{Backpressure, CompositionHost, FrameKey, FrameState, LiveFrameSet, Outcome, ReconNode};
use cumulus3d_cli::events::{Event, EventKind};
use cumulus3d_cli::live::{DeltaKind, DisplayFeed, DisplayMirror};
use cumulus3d_cli::pipeline::Pipeline;
use cumulus3d_cli::post::LiveOptions;
use cumulus3d_cli::session::{FrameSet, Input, Session, SessionConfig};
use cumulus3d_core::analyzer::ModelStats;
use cumulus3d_core::io::PointCloud;
use cumulus3d_core::Reconstruction;
use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};
use std::time::Instant;

const CAMS: [&str; 3] = ["camF", "camR", "camL"];

fn dataset() -> &'static Path {
    static DIR: OnceLock<PathBuf> = OnceLock::new();
    DIR.get_or_init(|| {
        let d = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("live_throughput_src");
        let _ = std::fs::remove_dir_all(&d);
        make_dataset(&d);
        d
    })
}

fn frames(p: usize) -> FrameSet {
    FrameSet::new(CAMS.iter().map(|c| (c.to_string(), format!("{c}/{c}_{:04}.jpg", p * 3))))
}

fn config(root: &Path) -> SessionConfig {
    let mut c = SessionConfig::new(root.join("images"));
    c.gps = cumulus3d_core::io::read_gps_file(root.join("gps_ref.txt")).unwrap();
    c.span = 4;
    c.overlap = 1;
    c.total_positions = None;
    c.seed = Some(7);
    c
}

/// Projection center per image name of a model.
fn centers(m: &Reconstruction) -> BTreeMap<String, [f64; 3]> {
    m.registered_images()
        .into_iter()
        .filter_map(|id| Some((m.image(id)?.name.clone(), m.projection_center(id).map(|c| [c.x, c.y, c.z])?)))
        .collect()
}

/// Largest camera-center difference after the best Sim3 of `b` onto `a` (the chain frame can differ by a refined model that was
/// adopted at a different position; the frame is not what is compared here).
fn aligned_difference(a: &Reconstruction, b: &Reconstruction) -> f64 {
    let (ca, cb) = (centers(a), centers(b));
    assert_eq!(ca.keys().collect::<Vec<_>>(), cb.keys().collect::<Vec<_>>());
    let v = |p: &[f64; 3]| cumulus3d_core::Vec3::new(p[0], p[1], p[2]);
    let (src, dst): (Vec<_>, Vec<_>) = cb.iter().map(|(n, p)| (v(p), v(&ca[n]))).unzip();
    let t = cumulus3d_align::umeyama(&src, &dst, true).expect("alignment");
    src.iter().zip(&dst).map(|(s, d)| (t.transform_point(s) - d).norm()).fold(0.0, f64::max)
}

fn kinds(evs: &[Event]) -> HashMap<EventKind, usize> {
    let mut m = HashMap::new();
    for e in evs {
        *m.entry(e.kind()).or_insert(0) += 1;
    }
    m
}

/// All inputs as `Input::Frames`, one after the other.
fn run_sequential(cfg: SessionConfig, n: usize) -> (Session, Vec<Event>) {
    let mut s = Session::new(cfg);
    let mut all = Vec::new();
    for p in 0..n {
        all.extend(s.apply(Input::Frames(frames(p))));
        assert!(s.failed().is_none(), "{:?}", s.failed());
    }
    all.extend(s.apply(Input::Finish));
    (s, all)
}

/// The inputs cut into batches of the given sizes.
fn run_batched(cfg: SessionConfig, cuts: &[usize]) -> (Session, Vec<Event>) {
    let mut s = Session::new(cfg);
    let mut all = Vec::new();
    let mut p = 0;
    for &c in cuts {
        let sets: Vec<FrameSet> = (p..p + c).map(frames).collect();
        all.extend(s.apply(Input::Batch(sets)));
        assert!(s.failed().is_none(), "{:?}", s.failed());
        p += c;
    }
    all.extend(s.apply(Input::Finish));
    (s, all)
}

#[test]
fn batch_equals_position_by_position_within_tolerance() {
    let root = dataset();
    let (seq, seq_evs) = run_sequential(config(root), NPOS);
    // One batch of everything, and a 3 + 5 cut (the cut falls inside the pre-model positions, the first model at position 4).
    for cuts in [vec![NPOS], vec![3, 5], vec![2, 2, 2, 2]] {
        let (bat, bat_evs) = run_batched(config(root), &cuts);
        let (a, b) = (seq.model().unwrap(), bat.model().unwrap());
        assert_eq!(a.registered_image_count(), b.registered_image_count(), "{cuts:?}");
        assert_eq!(bat.positions(), NPOS);
        // The scene spans about 10 units. The registration order changes the last digits, and whether (and at which position) the
        // refined zone 0 is adopted as the chain base depends on the timing of its background bundle adjustment, which moves the
        // later cameras a little: a few percent of the scene is the tolerance. Likewise the mean reprojection error is that of an
        // unrefined chain (about 0.5 px) when the refined zone was not ready in time, and 0.25 px when it was adopted.
        let worst = aligned_difference(a, b);
        assert!(worst < 0.3, "{cuts:?}: largest camera center difference {worst}");
        let (ra, rb) = (ModelStats::compute(a).mean_reprojection_error, ModelStats::compute(b).mean_reprojection_error);
        assert!(rb < 1.0 && rb <= ra * 3.0, "{cuts:?}: mean reprojection error {rb} vs {ra}");
        let (pa, pb) = (a.num_points3d() as f64, b.num_points3d() as f64);
        assert!((pa - pb).abs() <= 0.1 * pa.max(1.0), "{cuts:?}: points {pa} vs {pb}");
        // Same event kinds and counts (the order of the stages differs inside a batch).
        let (ka, kb) = (kinds(&seq_evs), kinds(&bat_evs));
        for k in [
            EventKind::FrameIngested,
            EventKind::FeaturesExtracted,
            EventKind::PairsMatched,
            EventKind::ModelInitialized,
            EventKind::PositionDone,
            EventKind::ZoneArrived,
            EventKind::ZonePreview,
            EventKind::RefineStarted,
            EventKind::ZoneRefined,
            EventKind::AllRefinedDone,
            EventKind::Finished,
        ] {
            assert_eq!(ka.get(&k), kb.get(&k), "{cuts:?}: {k:?}");
        }
        // Versions still count 1..n without gaps.
        for (i, e) in bat_evs.iter().enumerate() {
            assert_eq!(e.meta().version, i as u64 + 1, "{cuts:?}");
        }
    }
}

#[test]
fn a_batch_triangulates_and_registers_once_per_segment() {
    let root = dataset();
    let (_, evs) = run_batched(config(root), &[NPOS]);
    let logs: Vec<String> = evs
        .iter()
        .filter_map(|e| match e {
            Event::Log { line, .. } => Some(line.clone()),
            _ => None,
        })
        .collect();
    // Zone 0 closes at position 4 (span 4 + overlap 1): segments [0..=4] and [5..=7].
    assert!(logs.iter().any(|l| l.starts_with("[time] matching pos 0..4")), "{logs:#?}");
    assert!(logs.iter().any(|l| l.starts_with("[time] matching pos 5..7")), "{logs:#?}");
    assert_eq!(logs.iter().filter(|l| l.starts_with("[triangulate] pos 5..7")).count(), 1);
    assert_eq!(logs.iter().filter(|l| l.starts_with("[register] pos 5..7")).count(), 1);
    // Every position still reports its own match line.
    assert_eq!(logs.iter().filter(|l| l.starts_with("[match] pos ")).count(), NPOS);
}

#[test]
fn lookahead_changes_the_timing_only() {
    let root = dataset();
    let (plain, plain_evs) = run_sequential(config(root), NPOS);
    let mut cfg = config(root);
    cfg.lookahead = 3;
    let mut s = Session::new(cfg);
    let la = s.lookahead();
    assert_eq!(la.limit(), 3);
    let mut evs = Vec::new();
    for p in 0..NPOS {
        // Hint the next positions before processing this one.
        for q in p..(p + 4).min(NPOS) {
            if !la.hint(&frames(q)) {
                break;
            }
        }
        evs.extend(s.apply(Input::Frames(frames(p))));
        assert!(s.failed().is_none(), "{:?}", s.failed());
    }
    evs.extend(s.apply(Input::Finish));
    assert_eq!(la.pending(), 0);
    // Same image ids and feature events in the same order.
    let feats = |evs: &[Event]| -> Vec<(String, u32, usize, usize)> {
        evs.iter()
            .filter_map(|e| match e {
                Event::FeaturesExtracted { image, count, position, image_id, .. } => Some((image.clone(), *image_id, *count, *position)),
                _ => None,
            })
            .collect()
    };
    assert_eq!(feats(&plain_evs), feats(&evs));
    assert_eq!(feats(&evs).len(), 3 * NPOS);
    let ids = |s: &Session| s.store().images().into_iter().map(|i| (i.name, i.image_id, i.camera_id)).collect::<Vec<_>>();
    assert_eq!(ids(&plain), ids(&s));
    for im in s.store().images() {
        assert_eq!(plain.store().num_keypoints(im.image_id), s.store().num_keypoints(im.image_id));
    }
    assert_eq!(kinds(&plain_evs).get(&EventKind::PairsMatched), kinds(&evs).get(&EventKind::PairsMatched));
    // Hinting a position twice, or one that never comes, is harmless.
    let la2 = Session::new(config(root)).lookahead();
    assert!(!la2.hint(&frames(0)), "lookahead off: nothing queued");
}

#[test]
fn lookahead_survives_a_rewind() {
    let root = dataset();
    let mut cfg = config(root);
    cfg.lookahead = 2;
    cfg.history = 8;
    let mut s = Session::new(cfg);
    let la = s.lookahead();
    for p in 0..4 {
        la.hint(&frames(p + 1));
        s.apply(Input::Frames(frames(p)));
    }
    // Rewind to position 2 while position 4 is hinted, then run on.
    la.hint(&frames(4));
    s.apply(Input::Command(cumulus3d_cli::events::Command::ResetFrom(2)));
    assert_eq!(s.positions(), 2);
    for p in 2..NPOS {
        s.apply(Input::Frames(frames(p)));
        assert!(s.failed().is_none(), "{:?}", s.failed());
    }
    s.apply(Input::Finish);
    assert_eq!(s.store().num_images(), 3 * NPOS);
    assert!(s.model().unwrap().registered_image_count() >= 18);
}

// ---------------------------------------------------------------- Quick previews

fn stable_config() -> SessionConfig {
    let mut c = config(dataset());
    c.span = 3;
    c.overlap = 1;
    c.total_positions = Some(NPOS);
    c.stable_display = true;
    c.preview = true;
    // Never settles by itself, forced at zone 2: zones 0 and 1 are re-placed once.
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

fn centroid(c: &PointCloud) -> [f64; 3] {
    let n = c.len().max(1) as f64;
    let mut s = [0.0; 3];
    for p in &c.positions {
        for i in 0..3 {
            s[i] += p[i] as f64 / n;
        }
    }
    s
}

#[test]
fn preview_comes_first_in_the_display_frame_and_is_replaced_by_the_refined_zone() {
    let mut feed = DisplayFeed::new(LiveOptions { stride: 1, ..LiveOptions::default() });
    let pl = Pipeline::new(Session::new(stable_config())).sync(true).derive(move |e| {
        let e = densify(e);
        feed.handle(&e).into_iter().collect()
    });
    let rx = pl.subscribe();
    let mut pl = pl;
    for p in 0..NPOS {
        pl.push(Input::Frames(frames(p)));
    }
    let (s, _) = pl.finish();
    assert!(s.is_finished());
    let evs: Vec<Event> = rx.try_iter().collect();
    // One sparse, unkeyed-free preview per zone, in the display frame.
    let previews: Vec<&Event> = evs.iter().filter(|e| e.kind() == EventKind::ZonePreview).collect();
    assert_eq!(previews.len(), 3, "zones 0..2");
    let mut preview_clouds: BTreeMap<usize, Arc<PointCloud>> = BTreeMap::new();
    for e in &previews {
        let Event::ZonePreview { range, cloud, dense, frame, model, .. } = e else { unreachable!() };
        assert!(!dense && frame == "display");
        assert!(cloud.len() > 20 && cloud.has_keys() && cloud.keys.len() == cloud.len());
        assert!(model.registered_image_count() > 0);
        preview_clouds.insert(range.zone, cloud.clone());
    }
    // Preview deltas come before the refined delta of the same zone and only add points: nothing shown is moved or removed.
    let mut mirror = DisplayMirror::new();
    let mut order: Vec<(usize, DeltaKind)> = Vec::new();
    for e in &evs {
        let Event::DisplayDelta { delta, zone, kind, .. } = e else { continue };
        if *kind == DeltaKind::Preview {
            assert!(delta.moves.is_empty() && delta.removes.is_empty(), "a preview must not touch shown points (zone {zone})");
            assert!(!delta.adds.is_empty());
            assert!(delta.adds.iter().all(|a| a.zone as usize == *zone));
        }
        mirror.apply(delta).unwrap();
        order.push((*zone, *kind));
    }
    for z in 0..3 {
        let pv =
            order.iter().position(|o| *o == (z, DeltaKind::Preview)).unwrap_or_else(|| panic!("no preview delta for zone {z}: {order:?}"));
        let rf =
            order.iter().position(|o| *o == (z, DeltaKind::Refined)).unwrap_or_else(|| panic!("no refined delta for zone {z}: {order:?}"));
        assert!(pv < rf, "zone {z}: preview delta after the refined one: {order:?}");
    }
    assert_eq!(order.last().map(|o| o.1), Some(DeltaKind::Final));
    // At the end every preview point is gone: the display holds only the refined clouds' points (zone u8 = zone, all from the
    // refined sparse models here).
    assert!(!mirror.is_empty());
    let preview_keys: std::collections::HashSet<u64> = preview_clouds.values().flat_map(|c| c.keys.iter().copied()).collect();
    for c in preview_clouds.values() {
        assert_eq!(c.keys.iter().collect::<std::collections::HashSet<_>>().len(), c.len(), "preview keys are unique within a zone");
    }
    assert!(preview_keys.iter().all(|k| k & 0xFFFF_0000 == 0xFFFF_0000), "preview keys use the row no pixel has");
}

#[test]
fn preview_of_the_first_zone_lands_where_its_refined_zone_does() {
    let mut cfg = stable_config();
    cfg.settle_max_zone = 0;
    let (_, evs) = {
        let mut s = Session::new(cfg);
        let mut all = Vec::new();
        for p in 0..NPOS {
            all.extend(s.apply(Input::Frames(frames(p))));
        }
        all.extend(s.apply(Input::Finish));
        ((), all)
    };
    let pv = evs.iter().find_map(|e| match e {
        Event::ZonePreview { range, cloud, .. } if range.zone == 0 => Some(cloud.clone()),
        _ => None,
    });
    let rf = evs.iter().find_map(|e| match e {
        Event::ZoneRefined { range, model, .. } if range.zone == 0 => Some(model.clone()),
        _ => None,
    });
    let (pv, rf) = (pv.expect("preview of zone 0"), rf.expect("refined zone 0"));
    let refined = cloud_of(0, &rf);
    let (a, b) = (centroid(&pv), centroid(&refined));
    let d = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
    // Both are placed by the GPS ENU fit (the refined one after its bundle adjustment); the scene is about 10 units wide.
    assert!(d < 2.0, "preview centroid {a:?} vs refined {b:?}: {d}");
}

// ---------------------------------------------------------------- Compose node

fn node_images(tag: &str) -> PathBuf {
    let dst = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(format!("live_throughput_{tag}"));
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

fn live_set(seq: u64) -> LiveFrameSet {
    let gps = cumulus3d_core::io::read_gps_file(dataset().join("gps_ref.txt")).unwrap();
    let name = |c: &str| format!("{c}/{c}_{:04}.jpg", seq * 3);
    let mut set = LiveFrameSet::new(FrameKey::new("drone", seq, 0));
    for c in CAMS {
        let g = gps.iter().find(|g| g.name == name(c)).unwrap();
        set = set.file(c, name(c)).gps_fix(c, g.lat, g.lon, g.alt);
    }
    set
}

fn builder(tag: &str) -> cumulus3d_cli::compose::NodeBuilder {
    ReconNode::declare().id(tag).images(node_images(tag)).cameras(CAMS).zones(4, 1).seed(7)
}

#[test]
fn compose_batch_appends_every_set_and_matches_the_sequential_node() {
    let host = CompositionHost::new();
    let seq = builder("seq").lookahead(0).batch(0).build(&host).unwrap();
    for s in 0..NPOS as u64 {
        assert!(matches!(seq.compose(FrameState::ready(live_set(s))).unwrap(), Outcome::Appended { .. }));
    }
    let bat = builder("bat").build(&host).unwrap();
    let rx = bat.subscribe().unwrap();
    let outs = bat.compose_batch((0..NPOS as u64).map(|s| FrameState::ready(live_set(s))));
    assert_eq!(outs.len(), NPOS);
    for (i, o) in outs.iter().enumerate() {
        assert_eq!(*o.as_ref().unwrap(), Outcome::Appended { position: i, skipped: 0 });
    }
    // A duplicate and a late set inside a batch flush it first and are handled as usual.
    let outs = bat.compose_batch([FrameState::ready(live_set(3))]);
    assert!(matches!(outs[0].as_ref().unwrap(), Outcome::Duplicate { position: 3 }));
    let (a, b) = (seq.close().unwrap(), bat.close().unwrap());
    let (ma, mb) = (a.session.model().unwrap(), b.session.model().unwrap());
    assert_eq!(ma.registered_image_count(), mb.registered_image_count());
    let worst = aligned_difference(ma, mb);
    assert!(worst < 0.3, "{worst}");
    let (ra, rb) = (ModelStats::compute(ma).mean_reprojection_error, ModelStats::compute(mb).mean_reprojection_error);
    assert!(rb < 1.0 && rb <= ra * 3.0, "mean reprojection error {rb} vs {ra}");
    let logs: Vec<String> = rx
        .try_iter()
        .filter_map(|e| match e {
            Event::Log { line, .. } => Some(line),
            _ => None,
        })
        .collect();
    assert!(logs.iter().any(|l| l.starts_with("[time] matching pos 0..4")), "{logs:#?}");
}

#[test]
fn spawned_node_works_off_a_backlog_in_batches() {
    let host = CompositionHost::new();
    let n = builder("spawn").backpressure(Backpressure::Unbounded).build(&host).unwrap();
    let rx = n.subscribe().unwrap();
    let h = n.spawn();
    for s in 0..NPOS as u64 {
        h.send(FrameState::ready(live_set(s))).unwrap();
    }
    let (outs, closed) = h.close();
    let closed = closed.unwrap();
    assert!(outs.iter().all(|o| o.is_ok()), "{outs:?}");
    assert_eq!(closed.stats.appended, NPOS as u64);
    assert_eq!(closed.session.positions(), NPOS);
    assert!(closed.session.model().unwrap().registered_image_count() >= 18);
    // Where the worker found a backlog it matched several positions at once (timing dependent, so only the lines of a run are
    // checked for consistency: every position got its own match line).
    let logs: Vec<String> = rx
        .try_iter()
        .filter_map(|e| match e {
            Event::Log { line, .. } => Some(line),
            _ => None,
        })
        .collect();
    assert_eq!(logs.iter().filter(|l| l.starts_with("[match] pos ")).count(), NPOS);
}

// ---------------------------------------------------------------- Benchmark

fn stage_seconds(evs: &[Event]) -> Vec<(String, f64)> {
    evs.iter()
        .find_map(|e| match e {
            Event::Finished { stage_times, .. } => Some(stage_times.iter().map(|(k, d, _)| (k.clone(), d.as_secs_f64())).collect()),
            _ => None,
        })
        .unwrap_or_default()
}

/// Per-position throughput of the live node variants on a longer synthetic flight. Two arrival patterns per variant: every
/// position queued at once (the time to drain the backlog per position is the rate the variant can sustain), and positions
/// arriving every `C3D_BENCH_ARRIVAL_MS` (default 80 ms, faster than the serial variant works) where `lag` is how long after the
/// last arrival the node is done and `depth` the deepest queue.
#[test]
#[ignore = "benchmark: run with --release -- --ignored --nocapture"]
fn bench_live_throughput() {
    let npos: usize = std::env::var("C3D_BENCH_POSITIONS").ok().and_then(|v| v.parse().ok()).unwrap_or(40);
    let arrival_ms: u64 = std::env::var("C3D_BENCH_ARRIVAL_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(80);
    let root = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("live_throughput_bench_src");
    let _ = std::fs::remove_dir_all(&root);
    make_dataset_n(&root, npos);
    let gps = cumulus3d_core::io::read_gps_file(root.join("gps_ref.txt")).unwrap();
    let set = |seq: u64| {
        let name = |c: &str| format!("{c}/{c}_{:04}.jpg", seq * 3);
        let mut s = LiveFrameSet::new(FrameKey::new("drone", seq, 0));
        for c in CAMS {
            let g = gps.iter().find(|g| g.name == name(c)).unwrap();
            s = s.file(c, name(c)).gps_fix(c, g.lat, g.lon, g.alt);
        }
        s
    };
    println!("\nlive throughput, {npos} positions x 3 images (320x240), zones 12/2");
    println!(
        "{:<28} | {:>8} {:>10} | {:>8} {:>7} {:>6} | stages (s): feature / matching / registration / triangulation",
        "variant", "drain s", "s/position", "paced s", "lag s", "depth"
    );
    let variants: [(&str, usize, usize); 4] =
        [("serial (before)", 0, 0), ("lookahead 4", 4, 0), ("batch from 3", 0, 3), ("lookahead 4 + batch from 3", 4, 3)];
    for (label, look, batch) in variants {
        let mut row = Vec::new();
        for pace in [0u64, arrival_ms] {
            let host = CompositionHost::new();
            let n = ReconNode::declare()
                .id(format!("{label}-{pace}"))
                .images(root.join("images"))
                .cameras(CAMS)
                .zones(12, 2)
                .seed(7)
                .lookahead(look)
                .batch(batch)
                .backpressure(Backpressure::Unbounded)
                .build(&host)
                .unwrap();
            let rx = n.subscribe().unwrap();
            let t0 = Instant::now();
            let h = n.spawn();
            for s in 0..npos as u64 {
                if pace > 0 {
                    std::thread::sleep(std::time::Duration::from_millis(pace));
                }
                h.send(FrameState::ready(set(s))).unwrap();
            }
            let sent = t0.elapsed().as_secs_f64();
            let depth = h.stats().max_depth;
            let (outs, closed) = h.close();
            let wall = t0.elapsed().as_secs_f64();
            assert!(outs.iter().all(|o| o.is_ok()));
            closed.unwrap();
            let evs: Vec<Event> = rx.try_iter().collect();
            row.push((wall, wall - sent, depth, stage_seconds(&evs)));
        }
        let get = |st: &[(String, f64)], k: &str| st.iter().find(|(n, _)| n == k).map_or(0.0, |(_, v)| *v);
        let (d, p) = (&row[0], &row[1]);
        println!(
            "{label:<28} | {:>8.2} {:>10.3} | {:>8.2} {:>7.2} {:>6} | {:.2} / {:.2} / {:.2} / {:.2}",
            d.0,
            d.0 / npos as f64,
            p.0,
            p.1,
            p.2,
            get(&d.3, "feature"),
            get(&d.3, "matching"),
            get(&d.3, "registration"),
            get(&d.3, "triangulation")
        );
    }
}

/// SIFT backend with the latency of a GPU call and no CPU load: the CPU extraction plus a sleep per image, standing in for a
/// device whose extraction does not compete with the CPU stages.
struct LatencySift {
    inner: cumulus3d_features::CpuSift,
    delay: std::time::Duration,
}

impl cumulus3d_features::SiftEngine for LatencySift {
    fn name(&self) -> &str {
        "latency"
    }
    fn extract(
        &self,
        image: &cumulus3d_features::GrayImage,
        options: &cumulus3d_features::SiftOptions,
    ) -> cumulus3d_core::Result<cumulus3d_features::SiftOutput> {
        std::thread::sleep(self.delay);
        self.inner.extract(image, options)
    }
}

/// Stage pipelining on a session with a device-like extraction stage (see [`LatencySift`]): positions are fed one at a time,
/// without and with the next positions hinted. Run with `--release -- --ignored --nocapture`.
#[test]
#[ignore = "benchmark: run with --release -- --ignored --nocapture"]
fn bench_extraction_overlap() {
    let npos: usize = std::env::var("C3D_BENCH_POSITIONS").ok().and_then(|v| v.parse().ok()).unwrap_or(24);
    let delay_ms: u64 = std::env::var("C3D_BENCH_SIFT_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(250);
    let root = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("live_throughput_overlap_src");
    let _ = std::fs::remove_dir_all(&root);
    make_dataset_n(&root, npos);
    println!("\nextraction overlap, {npos} positions, extraction latency {delay_ms} ms per image (3 in parallel)");
    println!("{:<20} {:>9} {:>11}", "variant", "total s", "s/position");
    for (label, look) in [("serial (before)", 0usize), ("lookahead 2", 2), ("lookahead 4", 4)] {
        let mut cfg = config(&root);
        cfg.span = 12;
        cfg.overlap = 2;
        cfg.sift =
            Arc::new(LatencySift { inner: cumulus3d_features::CpuSift::default(), delay: std::time::Duration::from_millis(delay_ms) });
        cfg.lookahead = look;
        let mut s = Session::new(cfg);
        let la = s.lookahead();
        let t0 = Instant::now();
        for p in 0..npos {
            for q in p..(p + 1 + look).min(npos) {
                if !la.hint(&frames(q)) {
                    break;
                }
            }
            s.apply(Input::Frames(frames(p)));
            assert!(s.failed().is_none(), "{:?}", s.failed());
        }
        s.apply(Input::Finish);
        let wall = t0.elapsed().as_secs_f64();
        println!("{label:<20} {wall:>9.2} {:>11.3}", wall / npos as f64);
    }
}
