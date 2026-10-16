/*
 * blocks.rs
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

//! Live stream (`<out>/live/stream/`, format version 4): after every live event the composed display cloud (point keys and
//! the owner zone of every point) is diffed against what the viewer shows and only the changes are written. Every shown
//! point owns a dense `u32` slot id assigned here, so the viewer writes straight into its GPU buffers at index = id.
//!
//! `packs/e_<seq:04>.bin` (little endian, no header) holds three sections in this order: `n_add` records of 20 B
//! (`u32 id | f32 x y z | u8 r g b | u8 zone`), `n_move` records of 20 B (`u32 id | f32 x y z | u8 r g b | u8 pad=0`, new position and colour), `n_remove` records of
//! 4 B (`u32 id`). `manifest.json` (rewritten atomically after every event) lists the events with their counts and `capacity`
//! (max id + 1 used so far). A key that stays keeps its id (moved when its position changed by more than [`POSITION_EPS`] or its colour changed);
//! a key that leaves frees its id; a new key takes the lowest free id or `capacity++`. An id freed in an event is not handed
//! out in the same event. A zone change is a remove plus an add with a new id. Files are never overwritten.

use crate::live::KeyedDiff;
use crate::util::{py_float, Json};
use cumulus3d_core::io::PointCloud;
use std::path::{Path, PathBuf};

pub use crate::live::{KeyHasher, POSITION_EPS};

/// Manifest format version.
pub const FORMAT_VERSION: i64 = 4;

/// Bytes of an add record.
pub const ADD_BYTES: usize = 20;
/// Bytes of a move record.
pub const MOVE_BYTES: usize = 20;
/// Bytes of a remove record.
pub const REMOVE_BYTES: usize = 4;

/// Live stream settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiveStream {
    /// The display frame is the ENU frame (metres, x east, y north, z up); otherwise `"display"`.
    pub enu: bool,
    /// Geodetic origin `[lat, lon, alt]` of the ENU frame, when known.
    pub origin_gps: Option<[f64; 3]>,
}

/// What one [`DiffStream::write_event`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EventStats {
    /// Points added.
    pub n_add: usize,
    /// Points moved.
    pub n_move: usize,
    /// Points removed.
    pub n_remove: usize,
    /// Points shown after the event.
    pub points: usize,
    /// Pack size in bytes (0 when nothing changed).
    pub bytes: usize,
}

/// Writer of the live diff stream: a thin file writer over [`KeyedDiff`].
pub struct DiffStream {
    dir: PathBuf,
    cfg: LiveStream,
    diff: KeyedDiff,
    /// Manifest events written so far.
    events: Vec<Json>,
}

impl DiffStream {
    /// Stream into `dir` (`live/stream`); nothing is written until the first event.
    pub fn new(dir: PathBuf, cfg: LiveStream) -> Self {
        Self { dir, cfg, diff: KeyedDiff::new(), events: Vec::new() }
    }

    /// Number of events written so far (the last `seq`).
    pub fn events_written(&self) -> i64 {
        self.diff.seq() as i64
    }

    /// Directory of the stream.
    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// Records one event: `cloud` is the whole composed display (unique keys required), `zones[i]` the owner zone of
    /// point i, `t` seconds since the session start, `zone` the zone that triggered the event, `kind` one of `refined`,
    /// `replace`, `final`. Writes the pack and the manifest; on an error nothing of this event is recorded.
    pub fn write_event(&mut self, t: f64, zone: usize, kind: &str, cloud: &PointCloud, zones: &[usize]) -> Result<EventStats, String> {
        let pending = self.diff.prepare(cloud, zones)?;
        let d = pending.delta();
        let (n_add, n_move, n_remove) = (d.adds.len(), d.moves.len(), d.removes.len());
        let bytes = n_add * ADD_BYTES + n_move * MOVE_BYTES + n_remove * REMOVE_BYTES;
        let mut pack: Vec<u8> = Vec::with_capacity(bytes);
        for a in &d.adds {
            pack.extend_from_slice(&a.id.to_le_bytes());
            for v in a.xyz {
                pack.extend_from_slice(&v.to_le_bytes());
            }
            pack.extend_from_slice(&a.rgb);
            pack.push(a.zone);
        }
        for m in &d.moves {
            pack.extend_from_slice(&m.id.to_le_bytes());
            for v in m.xyz {
                pack.extend_from_slice(&v.to_le_bytes());
            }
            pack.extend_from_slice(&m.rgb);
            pack.push(0);
        }
        for id in &d.removes {
            pack.extend_from_slice(&id.to_le_bytes());
        }
        let seq = d.seq as i64;
        let pack_name = (bytes > 0).then(|| format!("packs/e_{seq:04}.bin"));
        let ev = Json::obj(vec![
            ("seq", Json::Int(seq)),
            ("t", Json::f(t, 2)),
            ("zone", Json::Int(zone as i64)),
            ("kind", Json::Str(kind.to_string())),
            ("points", Json::Int(d.total_points as i64)),
            ("pack", pack_name.clone().map_or(Json::Null, Json::Str)),
            ("n_add", Json::Int(n_add as i64)),
            ("n_move", Json::Int(n_move as i64)),
            ("n_remove", Json::Int(n_remove as i64)),
            ("bytes", Json::Int(bytes as i64)),
        ]);
        let written = match &pack_name {
            Some(name) => atomic_write(&self.dir.join(name), &pack),
            None => Ok(()),
        };
        self.events.push(ev);
        let written = written.and_then(|_| self.write_manifest());
        if let Err(e) = written {
            self.events.pop();
            self.diff.abort(pending);
            return Err(e);
        }
        self.diff.commit(pending);
        Ok(EventStats { n_add, n_move, n_remove, points: self.diff.len(), bytes })
    }

    fn write_manifest(&self) -> Result<(), String> {
        let origin = match self.cfg.origin_gps {
            Some(o) => Json::Arr(o.iter().map(|v| Json::Num(py_float(*v, 9))).collect()),
            None => Json::Null,
        };
        let m = Json::obj(vec![
            ("version", Json::Int(FORMAT_VERSION)),
            ("frame", Json::Str(if self.cfg.enu { "enu" } else { "display" }.into())),
            ("origin_gps", origin),
            ("capacity", Json::Int(self.diff.capacity() as i64)),
            ("events", Json::Arr(self.events.clone())),
        ]);
        atomic_write(&self.dir.join("manifest.json"), m.dump().as_bytes())
    }
}

/// Progress heartbeat `<dir>/status.json` of a running session (rewritten atomically; tiny).
#[derive(Clone, Debug, Default)]
pub struct Heartbeat {
    /// Stream folder (`live/stream`).
    pub dir: PathBuf,
    /// `registration` | `triangulation` | `ba` | `dense` | `done`.
    pub stage: &'static str,
    /// Zone being processed (or the latest one).
    pub zone: usize,
    /// Positions processed so far.
    pub positions: usize,
    /// Total positions, if known.
    pub positions_total: Option<usize>,
    /// Registered images so far.
    pub images: usize,
}

impl Heartbeat {
    /// Writes `status.json` for `t` seconds since the session start.
    pub fn write(&self, t: f64) -> Result<(), String> {
        let total = self.positions_total.map_or(Json::Null, |n| Json::Int(n as i64));
        let m = Json::obj(vec![
            ("t", Json::f(t, 1)),
            ("stage", Json::Str(self.stage.into())),
            ("zone", Json::Int(self.zone as i64)),
            ("positions", Json::Int(self.positions as i64)),
            ("positions_total", total),
            ("images", Json::Int(self.images as i64)),
        ]);
        atomic_write(&self.dir.join("status.json"), m.dump().as_bytes())
    }
}

/// Writes `bytes` to `path` through `<path>.tmp` + rename (parent folders are created).
pub(crate) fn atomic_write(path: &Path, bytes: &[u8]) -> Result<(), String> {
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d).map_err(|e| format!("{}: {e}", d.display()))?;
    }
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    std::fs::write(&tmp, bytes).map_err(|e| format!("{}: {e}", tmp.display()))?;
    std::fs::rename(&tmp, path).map_err(|e| format!("{}: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    const FROZEN_HASH: u64 = 0x3f8de9f327bd3d56;
    const FROZEN_LENS: [usize; 4] = [5760, 628, 920, 0];

    fn tmpdir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("cumulus3d_diff_{}_{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn cfg() -> LiveStream {
        LiveStream { enu: true, origin_gps: Some([37.5, 127.0, 30.0]) }
    }

    /// A 12 x 12 x 2 lattice with keys 1..=n and a zone per half.
    fn lattice() -> (PointCloud, Vec<usize>) {
        let mut c = PointCloud::default();
        let mut zones = Vec::new();
        for i in 0..12 {
            for j in 0..12 {
                for k in 0..2 {
                    c.positions.push([i as f32 * 0.7 - 3.0, j as f32 * 0.7 - 3.0, k as f32 * 0.7]);
                    c.colors.push([(i * 20) as u8, (j * 20) as u8, 7]);
                    c.keys.push(c.positions.len() as u64);
                    zones.push((i / 6) as usize);
                }
            }
        }
        (c, zones)
    }

    /// Subset of a cloud by index.
    fn pick(c: &PointCloud, z: &[usize], idx: &[usize]) -> (PointCloud, Vec<usize>) {
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

    type Ids = (Vec<u32>, Vec<u32>);
    type Slot = Option<([f32; 3], [u8; 3], u8)>;

    /// One event of the manifest: (seq, pack, n_add, n_move, n_remove, bytes, points).
    struct Ev {
        pack: Option<String>,
        n: [usize; 3],
        bytes: usize,
        points: usize,
    }

    fn manifest(dir: &Path) -> (Json, Vec<Ev>) {
        let m = Json::parse(&std::fs::read_to_string(dir.join("manifest.json")).unwrap()).unwrap();
        let Some(Json::Arr(events)) = m.get("events") else { panic!("no events") };
        let num = |e: &Json, k: &str| e.get(k).unwrap().as_f64().unwrap() as usize;
        let evs = events
            .iter()
            .map(|e| Ev {
                pack: match e.get("pack") {
                    Some(Json::Str(p)) => Some(p.clone()),
                    _ => None,
                },
                n: [num(e, "n_add"), num(e, "n_move"), num(e, "n_remove")],
                bytes: num(e, "bytes"),
                points: num(e, "points"),
            })
            .collect();
        (m, evs)
    }

    /// Applies the packs [from, to) to `slots` the way a viewer does; returns the (added, removed) ids of the last one.
    fn apply(dir: &Path, slots: &mut Vec<Slot>, ev: &Ev) -> (Vec<u32>, Vec<u32>) {
        let Some(p) = &ev.pack else { return (vec![], vec![]) };
        let b = std::fs::read(dir.join(p)).unwrap();
        assert_eq!(b.len(), ev.bytes);
        assert_eq!(b.len(), ev.n[0] * ADD_BYTES + ev.n[1] * MOVE_BYTES + ev.n[2] * REMOVE_BYTES);
        let u = |o: usize| u32::from_le_bytes(b[o..o + 4].try_into().unwrap());
        let f = |o: usize| f32::from_le_bytes(b[o..o + 4].try_into().unwrap());
        let (mut added, mut removed) = (vec![], vec![]);
        let mut o = 0;
        // removes first, as the viewer does
        let rm0 = ev.n[0] * ADD_BYTES + ev.n[1] * MOVE_BYTES;
        for r in 0..ev.n[2] {
            let id = u(rm0 + r * 4);
            removed.push(id);
            slots[id as usize] = None;
        }
        for _ in 0..ev.n[0] {
            let id = u(o) as usize;
            if slots.len() <= id {
                slots.resize(id + 1, None);
            }
            assert!(slots[id].is_none(), "add into a live slot {id}");
            slots[id] = Some(([f(o + 4), f(o + 8), f(o + 12)], [b[o + 16], b[o + 17], b[o + 18]], b[o + 19]));
            added.push(id as u32);
            o += ADD_BYTES;
        }
        for _ in 0..ev.n[1] {
            let id = u(o) as usize;
            let s = slots[id].as_mut().expect("move of an empty slot");
            s.0 = [f(o + 4), f(o + 8), f(o + 12)];
            s.1 = [b[o + 16], b[o + 17], b[o + 18]];
            assert_eq!(b[o + 19], 0);
            o += MOVE_BYTES;
        }
        (added, removed)
    }

    fn replay(dir: &Path) -> (Vec<Slot>, Vec<Ids>) {
        let (_, evs) = manifest(dir);
        let mut slots = Vec::new();
        let ids = evs.iter().map(|e| apply(dir, &mut slots, e)).collect();
        (slots, ids)
    }

    fn shown(slots: &[Slot]) -> Vec<([u32; 3], [u8; 3], u8)> {
        let mut v: Vec<_> = slots.iter().flatten().map(|s| (s.0.map(f32::to_bits), s.1, s.2)).collect();
        v.sort();
        v
    }

    fn expect(c: &PointCloud, z: &[usize]) -> Vec<([u32; 3], [u8; 3], u8)> {
        let mut v: Vec<_> = (0..c.len()).map(|i| (c.positions[i].map(f32::to_bits), c.colors[i], z[i] as u8)).collect();
        v.sort();
        v
    }

    #[test]
    fn packs_in_order_reproduce_the_composed_cloud() {
        let dir = tmpdir("repro");
        let (c, z) = lattice();
        let mut ds = DiffStream::new(dir.clone(), cfg());
        let st = ds.write_event(1.5, 0, "refined", &c, &z).unwrap();
        assert_eq!((st.n_add, st.n_move, st.n_remove, st.points), (c.len(), 0, 0, c.len()));
        // second event: shrink, move some, add new keys
        let keep: Vec<usize> = (0..c.len()).filter(|i| i % 5 != 0).collect();
        let (mut c2, mut z2) = pick(&c, &z, &keep);
        c2.positions[3][1] += 0.5;
        c2.positions[40][2] -= 0.25;
        for j in 0..30u64 {
            c2.positions.push([50.0 + j as f32, 0.0, 1.0]);
            c2.colors.push([1, 2, 3]);
            c2.keys.push(10_000 + j);
            z2.push(2);
        }
        ds.write_event(2.5, 1, "replace", &c2, &z2).unwrap();
        let (slots, _) = replay(&dir);
        assert_eq!(shown(&slots), expect(&c2, &z2));
        let (m, evs) = manifest(&dir);
        assert_eq!(m.get("version").and_then(Json::as_f64), Some(4.0));
        assert!(matches!(m.get("frame"), Some(Json::Str(s)) if s == "enu"));
        assert!(matches!(m.get("origin_gps"), Some(Json::Arr(a)) if a.len() == 3));
        assert_eq!(m.get("capacity").and_then(Json::as_f64), Some(slots.len() as f64));
        assert_eq!(evs[0].pack.as_deref(), Some("packs/e_0001.bin"));
        assert_eq!(evs[0].bytes, c.len() * ADD_BYTES);
        assert_eq!(evs[1].points, c2.len());
        assert_eq!(evs[1].n, [30, 2, c.len() - keep.len()]);
        assert!(!dir.join("manifest.json.tmp").exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn identical_event_writes_no_pack() {
        let dir = tmpdir("same");
        let (c, z) = lattice();
        let mut ds = DiffStream::new(dir.clone(), cfg());
        ds.write_event(1.0, 0, "refined", &c, &z).unwrap();
        // A sub-threshold wobble is not a change either.
        let mut c2 = c.clone();
        c2.positions[5][0] += 5e-5;
        let st = ds.write_event(2.0, 0, "replace", &c2, &z).unwrap();
        assert_eq!((st.n_add, st.n_move, st.n_remove, st.bytes), (0, 0, 0, 0));
        let (_, evs) = manifest(&dir);
        assert_eq!(evs.len(), 2);
        assert!(evs[1].pack.is_none());
        assert_eq!(evs[1].n, [0, 0, 0]);
        assert_eq!(evs[1].points, c.len());
        assert!(!dir.join("packs/e_0002.bin").exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn moving_points_only_moves() {
        let dir = tmpdir("move");
        let (c, z) = lattice();
        let mut ds = DiffStream::new(dir.clone(), cfg());
        ds.write_event(1.0, 0, "refined", &c, &z).unwrap();
        let mut c2 = c.clone();
        for i in [10usize, 200, 101] {
            c2.positions[i][0] += 0.3;
        }
        let st = ds.write_event(2.0, 1, "replace", &c2, &z).unwrap();
        assert_eq!((st.n_add, st.n_move, st.n_remove), (0, 3, 0));
        assert_eq!(st.bytes, 3 * MOVE_BYTES);
        let (slots, _) = replay(&dir);
        assert_eq!(shown(&slots), expect(&c2, &z));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn freed_ids_are_reused_only_in_the_next_event() {
        let dir = tmpdir("reuse");
        let (c, z) = lattice();
        let n = c.len();
        let mut ds = DiffStream::new(dir.clone(), cfg());
        ds.write_event(1.0, 0, "refined", &c, &z).unwrap();
        // event 2: drop the first 10, add 10 new keys
        let (mut c2, mut z2) = pick(&c, &z, &(10..n).collect::<Vec<_>>());
        let new_pt = |c: &mut PointCloud, z: &mut Vec<usize>, key: u64| {
            c.positions.push([40.0 + key as f32, 1.0, 2.0]);
            c.colors.push([9, 9, 9]);
            c.keys.push(key);
            z.push(0);
        };
        for k in 0..10 {
            new_pt(&mut c2, &mut z2, 5000 + k);
        }
        let st = ds.write_event(2.0, 0, "replace", &c2, &z2).unwrap();
        assert_eq!((st.n_add, st.n_remove), (10, 10));
        let (_, ids) = replay(&dir);
        let (added2, removed2) = &ids[1];
        assert!(added2.iter().all(|a| !removed2.contains(a)), "freed id reused in the same event");
        assert!(added2.iter().all(|&a| a as usize >= n), "adds must take new ids");
        // event 3: 10 more keys -> they take the ids freed in event 2 (lowest first)
        let (mut c3, mut z3) = (c2.clone(), z2.clone());
        for k in 0..10 {
            new_pt(&mut c3, &mut z3, 6000 + k);
        }
        let st = ds.write_event(3.0, 0, "replace", &c3, &z3).unwrap();
        assert_eq!((st.n_add, st.n_remove), (10, 0));
        let (slots, ids) = replay(&dir);
        let mut want = removed2.clone();
        want.sort();
        let mut got = ids[2].0.clone();
        got.sort();
        assert_eq!(got, want);
        assert_eq!(ids[2].0, want, "lowest free id first");
        assert_eq!(slots.len(), n + 10, "capacity does not grow when ids are recycled");
        assert_eq!(shown(&slots), expect(&c3, &z3));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn colour_change_is_a_move_with_the_same_id() {
        let dir = tmpdir("colour");
        let (c, z) = lattice();
        let mut ds = DiffStream::new(dir.clone(), cfg());
        ds.write_event(1.0, 0, "refined", &c, &z).unwrap();
        let mut c2 = c.clone();
        c2.colors[7] = [1, 1, 1];
        c2.colors[9] = [2, 2, 2];
        c2.positions[9][0] += 0.2;
        let st = ds.write_event(2.0, 0, "replace", &c2, &z).unwrap();
        assert_eq!((st.n_add, st.n_move, st.n_remove, st.points), (0, 2, 0, c.len()));
        assert_eq!(st.bytes, 2 * MOVE_BYTES);
        let (slots, _) = replay(&dir);
        assert_eq!(slots.len(), c.len());
        assert_eq!(shown(&slots), expect(&c2, &z));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn zone_change_is_remove_plus_add_with_a_new_id() {
        let dir = tmpdir("zone");
        let (c, z) = lattice();
        let mut ds = DiffStream::new(dir.clone(), cfg());
        ds.write_event(1.0, 0, "refined", &c, &z).unwrap();
        let mut z2 = z.clone();
        z2[7] = 5;
        z2[8] = 6;
        let st = ds.write_event(2.0, 0, "replace", &c, &z2).unwrap();
        assert_eq!((st.n_add, st.n_move, st.n_remove, st.points), (2, 0, 2, c.len()));
        let (slots, ids) = replay(&dir);
        assert_eq!(ids[1].1, vec![7, 8]);
        assert!(ids[1].0.iter().all(|&a| a as usize >= c.len()));
        assert_eq!(shown(&slots), expect(&c, &z2));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn rejects_clouds_without_keys() {
        let dir = tmpdir("nokeys");
        let mut c = PointCloud::default();
        c.positions.push([0.0; 3]);
        let mut ds = DiffStream::new(dir.clone(), cfg());
        assert!(ds.write_event(0.0, 0, "refined", &c, &[0]).is_err());
        assert!(!dir.join("manifest.json").exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// FNV-1a 64 over bytes.
    fn fnv(h: u64, b: &[u8]) -> u64 {
        b.iter().fold(h, |h, &x| (h ^ x as u64).wrapping_mul(0x0000_0100_0000_01b3))
    }

    /// The pack bytes of a fixed event sequence (adds, moves, colour change, zone change, removes, id reuse) are frozen:
    /// format version 4 must stay byte-identical across refactors.
    #[test]
    fn pack_bytes_are_frozen() {
        let dir = tmpdir("frozen");
        let (c, z) = lattice();
        let n = c.len();
        let mut ds = DiffStream::new(dir.clone(), cfg());
        ds.write_event(1.0, 0, "refined", &c, &z).unwrap();
        let keep: Vec<usize> = (0..n).filter(|i| i % 7 != 3).collect();
        let (mut c2, mut z2) = pick(&c, &z, &keep);
        c2.positions[5][0] += 0.4;
        c2.colors[9] = [3, 4, 5];
        z2[11] = 4;
        for j in 0..20u64 {
            c2.positions.push([60.0 + j as f32, 2.0, 1.5]);
            c2.colors.push([j as u8, 2, 3]);
            c2.keys.push(20_000 + j);
            z2.push(1);
        }
        ds.write_event(2.0, 1, "replace", &c2, &z2).unwrap();
        let (mut c3, mut z3) = pick(&c2, &z2, &(0..c2.len()).filter(|i| i % 11 != 0).collect::<Vec<_>>());
        c3.positions[2][2] -= 1.0;
        for j in 0..40u64 {
            c3.positions.push([-60.0 - j as f32, 2.0, 1.5]);
            c3.colors.push([9, j as u8, 3]);
            c3.keys.push(30_000 + j);
            z3.push(2);
        }
        ds.write_event(3.0, 2, "replace", &c3, &z3).unwrap();
        ds.write_event(4.0, 2, "final", &c3, &z3).unwrap();
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        let mut lens = Vec::new();
        for s in 1..=4 {
            let f = dir.join(format!("packs/e_{s:04}.bin"));
            let b = std::fs::read(&f).unwrap_or_default();
            lens.push(b.len());
            h = fnv(h, &b);
        }
        assert_eq!((h, lens), (FROZEN_HASH, FROZEN_LENS.to_vec()));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
