/*
 * live.rs
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

//! In-memory point-keyed display updates (no files).
//!
//! - [`KeyedDiff`]: diffs the whole composed display cloud (unique point keys + owner zone per point) against what the viewer
//!   shows and yields a [`DisplayDelta`] (adds / moves / removes with dense `u32` slot ids). This is the exact logic behind the
//!   file stream of [`crate::blocks::DiffStream`].
//! - [`DisplayMirror`]: the viewer side, applies deltas to a slot table (reference implementation for clients and tests).
//! - [`DisplayFeed`]: event driven end to end (zone results -> [`crate::post::LiveDisplay`] composition -> [`KeyedDiff`]); it is what
//!   `NodeBuilder::display_deltas` and [`crate::pipeline::Pipeline::derive`] use to emit [`Event::DisplayDelta`].
//!
//! Semantics of a delta: a key that stays keeps its id (a move when its position changed by more than [`POSITION_EPS`] or its
//! colour changed); a key that leaves frees its id; a new key takes the lowest free id or `capacity++`; an id freed in a delta is
//! not handed out in the same delta; a zone change is a remove plus an add with a new id. Apply a delta in this order: removes,
//! adds, moves.

use crate::events::{Event, Meta};
use crate::post::{LiveDisplay, LiveOptions};
use cumulus3d_core::io::PointCloud;
use cumulus3d_core::Reconstruction;
use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap, HashMap};
use std::hash::{BuildHasherDefault, Hasher};
use std::sync::{Arc, Mutex};
use std::time::SystemTime;

/// A point whose position moved by more than this (display units, any axis) is sent as a move.
pub const POSITION_EPS: f32 = 1e-4;

/// Cheap hasher for the already well-mixed 64-bit pixel keys.
#[derive(Default)]
pub struct KeyHasher(u64);

impl Hasher for KeyHasher {
    fn finish(&self) -> u64 {
        self.0 ^ (self.0 >> 32)
    }
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = (self.0.rotate_left(8) ^ b as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
        }
    }
    fn write_u64(&mut self, v: u64) {
        self.0 = v.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }
}

/// A point that appears: it takes slot `id`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointAdd {
    /// Slot id.
    pub id: u32,
    /// Position (display frame).
    pub xyz: [f32; 3],
    /// Colour.
    pub rgb: [u8; 3],
    /// Owner zone (saturated at 255).
    pub zone: u8,
}

/// A shown point whose position or colour changed (same slot id, new values).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointMove {
    /// Slot id.
    pub id: u32,
    /// New position.
    pub xyz: [f32; 3],
    /// New colour.
    pub rgb: [u8; 3],
}

/// Changes between two consecutive display clouds.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DisplayDelta {
    /// Sequence number (1-based, counts every applied cloud, also unchanged ones).
    pub seq: u64,
    /// New points.
    pub adds: Vec<PointAdd>,
    /// Changed points.
    pub moves: Vec<PointMove>,
    /// Slot ids of points that left (including the old slot of a zone change).
    pub removes: Vec<u32>,
    /// Points shown after the delta.
    pub total_points: usize,
    /// Max slot id + 1 used so far (size of the viewer's buffers).
    pub capacity: u32,
}

impl DisplayDelta {
    /// Whether nothing changed.
    pub fn is_empty(&self) -> bool {
        self.adds.is_empty() && self.moves.is_empty() && self.removes.is_empty()
    }
}

#[derive(Clone, Copy, Debug)]
struct Shown {
    id: u32,
    pos: [f32; 3],
    color: [u8; 3],
    zone: u8,
    /// Epoch that last saw this key (to find the removed ones without a second set).
    seen: u32,
}

/// A prepared (not yet committed) delta of [`KeyedDiff::prepare`]. Commit it with [`KeyedDiff::commit`] or give it back with
/// [`KeyedDiff::abort`] (e.g. when writing it somewhere failed).
#[derive(Debug)]
pub struct PendingDelta {
    delta: DisplayDelta,
    epoch: u32,
    add_keys: Vec<u64>,
    move_keys: Vec<u64>,
    removed_keys: Vec<u64>,
    replaced: usize,
    taken: Vec<u32>,
    cap0: u32,
}

impl PendingDelta {
    /// The delta that a commit will yield.
    pub fn delta(&self) -> &DisplayDelta {
        &self.delta
    }
}

/// Diff state of a point-keyed display: key -> slot id, free ids, capacity.
#[derive(Default)]
pub struct KeyedDiff {
    last: HashMap<u64, Shown, BuildHasherDefault<KeyHasher>>,
    free: BinaryHeap<Reverse<u32>>,
    capacity: u32,
    seq: u64,
    epoch: u32,
}

impl KeyedDiff {
    /// Empty state (nothing shown).
    pub fn new() -> Self {
        Self::default()
    }

    /// Sequence number of the last committed delta (0 before the first).
    pub fn seq(&self) -> u64 {
        self.seq
    }

    /// Max slot id + 1 used so far (including ids taken by an uncommitted [`PendingDelta`]).
    pub fn capacity(&self) -> u32 {
        self.capacity
    }

    /// Points shown after the last commit.
    pub fn len(&self) -> usize {
        self.last.len()
    }

    /// Whether nothing is shown.
    pub fn is_empty(&self) -> bool {
        self.last.is_empty()
    }

    /// Diffs `cloud` (the whole display; unique keys required, `zones[i]` = owner zone of point i) against the shown state and
    /// commits. Non-finite points are skipped. Errors (no keys, wrong `zones` length) leave the state untouched.
    pub fn apply(&mut self, cloud: &PointCloud, zones: &[usize]) -> Result<DisplayDelta, String> {
        let p = self.prepare(cloud, zones)?;
        Ok(self.commit(p))
    }

    /// A delta that changes nothing (the display is as before); still consumes a sequence number.
    pub fn unchanged(&mut self) -> DisplayDelta {
        self.seq += 1;
        DisplayDelta { seq: self.seq, total_points: self.last.len(), capacity: self.capacity, ..Default::default() }
    }

    /// First half of [`apply`](Self::apply): classifies and assigns ids, without changing what counts as shown. Exactly one of
    /// [`commit`](Self::commit) / [`abort`](Self::abort) must follow before the next `prepare`.
    pub fn prepare(&mut self, cloud: &PointCloud, zones: &[usize]) -> Result<PendingDelta, String> {
        let n = cloud.len();
        if n > 0 && cloud.keys.len() != n {
            return Err("점 키가 없는 점군은 스트림으로 낼 수 없음".into());
        }
        if zones.len() != n {
            return Err("점별 구역 길이가 점 수와 다름".into());
        }
        self.epoch += 1;
        let epoch = self.epoch;
        if self.last.is_empty() {
            self.last.reserve(n);
        }
        // Pass 1: classify. Kept keys are stamped; unseen keys are removals.
        let mut adds: Vec<(u64, [f32; 3], [u8; 3], u8)> = Vec::new();
        let mut moves: Vec<(u64, u32, [f32; 3], [u8; 3])> = Vec::new();
        let mut gone: Vec<u32> = Vec::new();
        let mut replaced = 0usize;
        for (i, &p) in cloud.positions.iter().enumerate() {
            if !p.iter().all(|v| v.is_finite()) {
                continue;
            }
            let key = cloud.keys[i];
            let color = cloud.colors.get(i).copied().unwrap_or([255; 3]);
            let zn = zones[i].min(255) as u8;
            match self.last.get_mut(&key) {
                Some(s) if s.seen == epoch => {}
                Some(s) => {
                    s.seen = epoch;
                    if s.zone != zn {
                        gone.push(s.id);
                        replaced += 1;
                        adds.push((key, p, color, zn));
                    } else if s.color != color || (0..3).any(|a| (s.pos[a] - p[a]).abs() > POSITION_EPS) {
                        moves.push((key, s.id, p, color));
                    }
                }
                None => adds.push((key, p, color, zn)),
            }
        }
        let mut removed_keys: Vec<u64> = Vec::new();
        for (k, s) in &self.last {
            if s.seen != epoch {
                removed_keys.push(*k);
                gone.push(s.id);
            }
        }
        // Ids for the adds: lowest free id of earlier deltas, else a new one. Undone by `abort`.
        let (cap0, mut taken) = (self.capacity, Vec::new());
        let mut add_ids: Vec<u32> = Vec::with_capacity(adds.len());
        for _ in 0..adds.len() {
            let id = match self.free.pop() {
                Some(Reverse(id)) => {
                    taken.push(id);
                    id
                }
                None => {
                    self.capacity += 1;
                    self.capacity - 1
                }
            };
            add_ids.push(id);
        }
        let total_points = self.last.len() - removed_keys.len() + adds.len() - replaced;
        let delta = DisplayDelta {
            seq: self.seq + 1,
            adds: adds.iter().zip(&add_ids).map(|((_, p, c, z), id)| PointAdd { id: *id, xyz: *p, rgb: *c, zone: *z }).collect(),
            moves: moves.iter().map(|(_, id, p, c)| PointMove { id: *id, xyz: *p, rgb: *c }).collect(),
            removes: gone,
            total_points,
            capacity: self.capacity,
        };
        Ok(PendingDelta {
            delta,
            epoch,
            add_keys: adds.into_iter().map(|a| a.0).collect(),
            move_keys: moves.into_iter().map(|m| m.0).collect(),
            removed_keys,
            replaced,
            taken,
            cap0,
        })
    }

    /// Makes a prepared delta the shown state and returns it.
    pub fn commit(&mut self, p: PendingDelta) -> DisplayDelta {
        let PendingDelta { delta, epoch, add_keys, move_keys, removed_keys, replaced, .. } = p;
        for k in &removed_keys {
            self.last.remove(k);
        }
        for (k, m) in move_keys.iter().zip(&delta.moves) {
            if let Some(s) = self.last.get_mut(k) {
                s.pos = m.xyz;
                s.color = m.rgb;
                debug_assert_eq!(s.id, m.id);
            }
        }
        self.last.reserve(add_keys.len().saturating_sub(replaced));
        for (key, a) in add_keys.into_iter().zip(&delta.adds) {
            self.last.insert(key, Shown { id: a.id, pos: a.xyz, color: a.rgb, zone: a.zone, seen: epoch });
        }
        self.free.extend(delta.removes.iter().copied().map(Reverse));
        self.seq = delta.seq;
        delta
    }

    /// Gives back a prepared delta (its ids return to the free list); the shown state is as before the `prepare`.
    pub fn abort(&mut self, p: PendingDelta) {
        self.capacity = p.cap0;
        for id in p.taken {
            self.free.push(Reverse(id));
        }
    }
}

/// One slot of a [`DisplayMirror`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MirrorPoint {
    /// Position.
    pub xyz: [f32; 3],
    /// Colour.
    pub rgb: [u8; 3],
    /// Owner zone.
    pub zone: u8,
}

/// Viewer-side slot table: applies [`DisplayDelta`]s the way a GPU-buffer viewer does (writes at index = id).
#[derive(Clone, Debug, Default)]
pub struct DisplayMirror {
    /// Slots by id (`None` = free).
    pub slots: Vec<Option<MirrorPoint>>,
}

impl DisplayMirror {
    /// Empty table.
    pub fn new() -> Self {
        Self::default()
    }

    /// Applies one delta (removes, then adds, then moves). Returns an error for an inconsistent delta (add into a live slot,
    /// move/remove of an empty one); the table may be partially updated then.
    pub fn apply(&mut self, d: &DisplayDelta) -> Result<(), String> {
        for &id in &d.removes {
            match self.slots.get_mut(id as usize) {
                Some(s @ Some(_)) => *s = None,
                _ => return Err(format!("remove of an empty slot {id}")),
            }
        }
        if self.slots.len() < d.capacity as usize {
            self.slots.resize(d.capacity as usize, None);
        }
        for a in &d.adds {
            let s = self.slots.get_mut(a.id as usize).ok_or_else(|| format!("add beyond capacity: {}", a.id))?;
            if s.is_some() {
                return Err(format!("add into a live slot {}", a.id));
            }
            *s = Some(MirrorPoint { xyz: a.xyz, rgb: a.rgb, zone: a.zone });
        }
        for m in &d.moves {
            match self.slots.get_mut(m.id as usize) {
                Some(Some(p)) => {
                    p.xyz = m.xyz;
                    p.rgb = m.rgb;
                }
                _ => return Err(format!("move of an empty slot {}", m.id)),
            }
        }
        Ok(())
    }

    /// Number of live points.
    pub fn len(&self) -> usize {
        self.slots.iter().flatten().count()
    }

    /// Whether no point is live.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Live points as `(id, point)`.
    pub fn points(&self) -> impl Iterator<Item = (u32, &MirrorPoint)> + '_ {
        self.slots.iter().enumerate().filter_map(|(i, s)| s.as_ref().map(|p| (i as u32, p)))
    }
}

/// Why a [`Event::DisplayDelta`] was emitted.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum DeltaKind {
    /// A zone's refined cloud arrived for the first time.
    Refined,
    /// A shown zone was re-densified or moved (stable display re-placement).
    Replace,
    /// All refinements finished; the display is complete (the delta is empty).
    Final,
}

impl DeltaKind {
    /// `"refined"` | `"replace"` | `"final"` (the names of the file stream).
    pub fn as_str(self) -> &'static str {
        match self {
            DeltaKind::Refined => "refined",
            DeltaKind::Replace => "replace",
            DeltaKind::Final => "final",
        }
    }
}

/// Coordinate frame of the display clouds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DisplayFrame {
    /// The display frame is the ENU frame (metres, x east, y north, z up); otherwise an arbitrary display frame.
    pub enu: bool,
    /// Geodetic origin `[lat, lon, alt]` of the ENU frame, when known.
    pub origin_gps: Option<[f64; 3]>,
}

/// Event-driven display feed: collects the zone results of the session events, composes the display
/// ([`LiveDisplay`]: key-hash thinning, sticky voxel ownership, zone composition) and diffs it ([`KeyedDiff`]).
///
/// Use it directly (`handle` every event) or through `NodeBuilder::display_deltas` / [`crate::pipeline::Pipeline::derive`].
/// Relocation of zones into the newest model's frame is on only when a preview was received (otherwise, as in stable display,
/// all zone clouds already share one frame). `Reset` / `ZoneInvalidated` drop the zone's state; the display is recomposed (and the
/// difference sent) at the next refined cloud.
pub struct DisplayFeed {
    live: LiveDisplay,
    diff: KeyedDiff,
    refined: BTreeMap<usize, Arc<PointCloud>>,
    refined_models: BTreeMap<usize, Arc<Reconstruction>>,
    previews: BTreeSet<usize>,
    t0: Option<SystemTime>,
    origin: Arc<Mutex<Option<[f64; 3]>>>,
    shown_any: bool,
}

impl DisplayFeed {
    /// With the given composition settings (`relocate` is decided from the events, see above).
    pub fn new(opts: LiveOptions) -> Self {
        Self {
            live: LiveDisplay::new(opts),
            diff: KeyedDiff::new(),
            refined: BTreeMap::new(),
            refined_models: BTreeMap::new(),
            previews: BTreeSet::new(),
            t0: None,
            origin: Arc::new(Mutex::new(None)),
            shown_any: false,
        }
    }

    /// Shares the ENU origin slot (set by the producer of GPS fixes); the frame of the deltas is ENU once it holds a value.
    pub fn with_origin(mut self, origin: Arc<Mutex<Option<[f64; 3]>>>) -> Self {
        self.origin = origin;
        self
    }

    /// Fixes the ENU origin `[lat, lon, alt]`.
    pub fn set_origin(&self, origin: Option<[f64; 3]>) {
        *self.origin.lock().unwrap_or_else(|p| p.into_inner()) = origin;
    }

    /// Current display frame.
    pub fn frame(&self) -> DisplayFrame {
        let o = *self.origin.lock().unwrap_or_else(|p| p.into_inner());
        DisplayFrame { enu: o.is_some(), origin_gps: o }
    }

    /// Slot table state (points shown, capacity, ...).
    pub fn diff(&self) -> &KeyedDiff {
        &self.diff
    }

    fn drop_zone(&mut self, k: usize) {
        self.refined.remove(&k);
        self.refined_models.remove(&k);
        self.previews.remove(&k);
        self.live.forget(k);
    }

    /// Feeds one session event; returns the [`Event::DisplayDelta`] it caused, if any. Events of other kinds (including
    /// `DisplayDelta` itself) are ignored.
    pub fn handle(&mut self, e: &Event) -> Option<Event> {
        let meta: &Meta = e.meta();
        if matches!(e, Event::Started { .. }) || self.t0.is_none() {
            self.t0 = Some(meta.at);
        }
        let rel = |at: SystemTime, t0: Option<SystemTime>| {
            let secs = |t: SystemTime| t.duration_since(SystemTime::UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64());
            secs(at) - secs(t0.unwrap_or(at))
        };
        match e {
            Event::ZonePreview { range, .. } => {
                self.previews.insert(range.zone);
                None
            }
            Event::ZoneRefinedPose { range, model, .. } => {
                self.refined_models.insert(range.zone, Arc::clone(model));
                self.live.forget(range.zone);
                None
            }
            Event::ZoneRefined { range, cloud, dense, .. } if *dense => {
                let kind = if self.refined.contains_key(&range.zone) { DeltaKind::Replace } else { DeltaKind::Refined };
                self.refined.insert(range.zone, Arc::clone(cloud));
                let clouds: BTreeMap<usize, &PointCloud> = self.refined.iter().map(|(j, c)| (*j, &**c)).collect();
                let models: BTreeMap<usize, &Reconstruction> = self.refined_models.iter().map(|(j, m)| (*j, &**m)).collect();
                self.live.set_relocate(!self.previews.is_empty());
                let (display, zones) = self.live.compose_zoned(&clouds, &models);
                match self.diff.apply(&display, &zones) {
                    Ok(delta) => {
                        self.shown_any = true;
                        Some(Event::DisplayDelta {
                            meta: meta.clone(),
                            delta: Arc::new(delta),
                            zone: range.zone,
                            kind,
                            t: rel(meta.at, self.t0),
                            frame: self.frame(),
                        })
                    }
                    // A display without point keys cannot be diffed (a session produces keys; foreign events may not).
                    Err(msg) => Some(Event::Warning { meta: meta.clone(), message: format!("display delta: {msg}") }),
                }
            }
            Event::ZoneInvalidated { zone, .. } => {
                self.drop_zone(*zone);
                None
            }
            Event::AllRefinedDone { .. } if self.shown_any => {
                self.shown_any = false;
                let zone = self.refined.keys().next_back().copied().unwrap_or(0);
                Some(Event::DisplayDelta {
                    meta: meta.clone(),
                    delta: Arc::new(self.diff.unchanged()),
                    zone,
                    kind: DeltaKind::Final,
                    t: rel(meta.at, self.t0),
                    frame: self.frame(),
                })
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cloud(n: usize) -> (PointCloud, Vec<usize>) {
        let mut c = PointCloud::default();
        let mut z = Vec::new();
        for i in 0..n {
            c.positions.push([i as f32, 0.0, 1.0]);
            c.colors.push([i as u8, 1, 2]);
            c.keys.push(100 + i as u64);
            z.push(i % 2);
        }
        (c, z)
    }

    fn keep(c: &PointCloud, z: &[usize], idx: &[usize]) -> (PointCloud, Vec<usize>) {
        let mut o = PointCloud::default();
        let mut oz = Vec::new();
        for &i in idx {
            o.positions.push(c.positions[i]);
            o.colors.push(c.colors[i]);
            o.keys.push(c.keys[i]);
            oz.push(z[i]);
        }
        (o, oz)
    }

    #[test]
    fn first_delta_adds_everything_with_dense_ids() {
        let mut d = KeyedDiff::new();
        let (c, z) = cloud(10);
        let x = d.apply(&c, &z).unwrap();
        assert_eq!((x.seq, x.adds.len(), x.moves.len(), x.removes.len(), x.total_points, x.capacity), (1, 10, 0, 0, 10, 10));
        let ids: Vec<u32> = x.adds.iter().map(|a| a.id).collect();
        assert_eq!(ids, (0..10).collect::<Vec<_>>());
        assert_eq!(x.adds[3].zone, 1);
    }

    #[test]
    fn move_on_position_or_colour_but_not_on_a_wobble() {
        let mut d = KeyedDiff::new();
        let (c, z) = cloud(10);
        d.apply(&c, &z).unwrap();
        let mut c2 = c.clone();
        c2.positions[1][0] += 5e-5;
        assert!(d.apply(&c2, &z).unwrap().is_empty());
        c2.positions[2][1] += 0.5;
        c2.colors[4] = [9, 9, 9];
        let x = d.apply(&c2, &z).unwrap();
        assert_eq!(x.seq, 3);
        assert!(x.adds.is_empty() && x.removes.is_empty());
        assert_eq!(x.moves.iter().map(|m| m.id).collect::<Vec<_>>(), vec![2, 4]);
        assert_eq!(x.moves[1].rgb, [9, 9, 9]);
        assert_eq!(x.total_points, 10);
    }

    #[test]
    fn remove_frees_ids_that_are_reused_next_delta_lowest_first() {
        let mut d = KeyedDiff::new();
        let (c, z) = cloud(10);
        d.apply(&c, &z).unwrap();
        // drop keys of slots 2 and 5, add two new keys in the same delta: they must not reuse 2 / 5.
        let (mut c2, mut z2) = keep(&c, &z, &[0, 1, 3, 4, 6, 7, 8, 9]);
        for k in [900u64, 901] {
            c2.positions.push([k as f32, 0.0, 0.0]);
            c2.colors.push([1, 1, 1]);
            c2.keys.push(k);
            z2.push(0);
        }
        let x = d.apply(&c2, &z2).unwrap();
        let mut rm = x.removes.clone();
        rm.sort();
        assert_eq!(rm, vec![2, 5]);
        assert_eq!(x.adds.iter().map(|a| a.id).collect::<Vec<_>>(), vec![10, 11]);
        assert_eq!((x.total_points, x.capacity), (10, 12));
        // next delta: two more new keys take 2 then 5.
        let (mut c3, mut z3) = (c2.clone(), z2.clone());
        for k in [902u64, 903] {
            c3.positions.push([k as f32, 0.0, 0.0]);
            c3.colors.push([1, 1, 1]);
            c3.keys.push(k);
            z3.push(0);
        }
        let y = d.apply(&c3, &z3).unwrap();
        assert_eq!(y.adds.iter().map(|a| a.id).collect::<Vec<_>>(), vec![2, 5]);
        assert_eq!(y.capacity, 12);
    }

    #[test]
    fn zone_change_is_remove_plus_add_with_a_new_id() {
        let mut d = KeyedDiff::new();
        let (c, z) = cloud(6);
        d.apply(&c, &z).unwrap();
        let mut z2 = z.clone();
        z2[1] = 7;
        let x = d.apply(&c, &z2).unwrap();
        assert_eq!(x.removes, vec![1]);
        assert_eq!(x.adds.len(), 1);
        assert_eq!((x.adds[0].id, x.adds[0].zone), (6, 7));
        assert!(x.moves.is_empty());
        assert_eq!(x.total_points, 6);
    }

    #[test]
    fn errors_leave_the_state_untouched_and_abort_gives_ids_back() {
        let mut d = KeyedDiff::new();
        let (c, z) = cloud(4);
        assert!(d.apply(&c, &z[..3]).is_err());
        let mut nokeys = c.clone();
        nokeys.keys.clear();
        assert!(d.apply(&nokeys, &z).is_err());
        assert_eq!((d.seq(), d.capacity(), d.len()), (0, 0, 0));
        let p = d.prepare(&c, &z).unwrap();
        assert_eq!(p.delta().adds.len(), 4);
        d.abort(p);
        assert_eq!((d.seq(), d.capacity(), d.len()), (0, 0, 0));
        let x = d.apply(&c, &z).unwrap();
        assert_eq!((x.seq, x.adds.iter().map(|a| a.id).collect::<Vec<_>>()), (1, vec![0, 1, 2, 3]));
    }

    #[test]
    fn mirror_reproduces_the_cloud_over_a_sequence() {
        let mut d = KeyedDiff::new();
        let mut m = DisplayMirror::new();
        let (c, z) = cloud(40);
        m.apply(&d.apply(&c, &z).unwrap()).unwrap();
        let (mut c2, mut z2) = keep(&c, &z, &(0..40).filter(|i| i % 3 != 0).collect::<Vec<_>>());
        c2.positions[4][2] = 9.0;
        z2[5] = 3;
        c2.positions.push([70.0, 0.0, 0.0]);
        c2.colors.push([5, 5, 5]);
        c2.keys.push(7000);
        z2.push(1);
        m.apply(&d.apply(&c2, &z2).unwrap()).unwrap();
        let mut got: Vec<_> = m.points().map(|(_, p)| (p.xyz.map(f32::to_bits), p.rgb, p.zone)).collect();
        let mut want: Vec<_> = (0..c2.len()).map(|i| (c2.positions[i].map(f32::to_bits), c2.colors[i], z2[i] as u8)).collect();
        got.sort();
        want.sort();
        assert_eq!(got, want);
        assert!(m.apply(&DisplayDelta { removes: vec![999], ..Default::default() }).is_err());
    }
}
