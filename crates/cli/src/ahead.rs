/*
 * ahead.rs
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

//! Feature extraction ahead of the position loop.
//!
//! The position loop of a [`crate::session::Session`] runs feature extraction -> matching -> registration -> triangulation one
//! position at a time. Feature extraction only needs the images of a position, so it can run for the positions that are about
//! to arrive while the later stages still work on the earlier ones. A [`Lookahead`] handle gets the frame sets of upcoming
//! positions ([`Lookahead::hint`]); one worker thread extracts them in arrival order (the images of a position in parallel) into the
//! session's feature store, at most `limit` positions ahead. When the session reaches such a position it takes the finished
//! result instead of extracting itself.
//!
//! The result is the same as without lookahead: image ids are issued in the same order (positions in arrival order, the images of a
//! position in frame-set order), cameras are assigned the same way, and the events (`FeaturesExtracted`, `[feature]` log lines) are
//! emitted when the position is processed, not when it was extracted.

use crate::intrinsics::IntrinsicsPrior;
use crate::session::{FrameImage, FrameSet};
use cumulus3d_core::{CameraId, FeatureStore};
use cumulus3d_features::{CameraMode, ExtractionOptions, FeatureExtractor, ImageReport};
use std::collections::{BTreeMap, VecDeque};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::time::{Duration, Instant};

/// Result of extracting the features of one position.
#[derive(Clone, Debug)]
pub(crate) struct PositionExtract {
    /// Per-image report in frame-set order.
    pub reports: Vec<ImageReport>,
    /// run.log lines to write (intrinsics prior applied to new cameras).
    pub notes: Vec<String>,
    /// Camera ids per camera name after this position.
    pub cams: BTreeMap<String, CameraId>,
    /// Wall time spent on this position (a share of the call when several positions were extracted together).
    pub elapsed: Duration,
}

struct Job {
    images: Vec<FrameImage>,
    epoch: u64,
    cancelled: AtomicBool,
    done: Mutex<Option<Result<PositionExtract, String>>>,
    cv: Condvar,
}

impl Job {
    fn fulfil(&self, r: Result<PositionExtract, String>) {
        *lock(&self.done) = Some(r);
        self.cv.notify_all();
    }

    fn wait(&self) -> Result<PositionExtract, String> {
        let mut d = lock(&self.done);
        loop {
            if let Some(r) = d.as_ref() {
                return r.clone();
            }
            d = self.cv.wait(d).unwrap_or_else(|p| p.into_inner());
        }
    }
}

struct State {
    store: Arc<FeatureStore>,
    /// Hinted jobs not yet taken by the session, in hint order.
    jobs: VecDeque<Arc<Job>>,
    /// Jobs the worker has not started.
    queue: VecDeque<Arc<Job>>,
    epoch: u64,
    worker_started: bool,
    closed: bool,
}

pub(crate) struct AheadCore {
    extractor: Arc<FeatureExtractor>,
    root: PathBuf,
    opts: ExtractionOptions,
    prior: Option<Arc<IntrinsicsPrior>>,
    limit: usize,
    /// Camera ids per camera name; held while a position is extracted, so extraction is serialized (ids in order).
    cams: Mutex<BTreeMap<String, CameraId>>,
    st: Mutex<State>,
    cv: Condvar,
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|p| p.into_inner())
}

impl AheadCore {
    pub(crate) fn new(
        extractor: Arc<FeatureExtractor>,
        root: PathBuf,
        opts: ExtractionOptions,
        prior: Option<Arc<IntrinsicsPrior>>,
        limit: usize,
        store: Arc<FeatureStore>,
    ) -> Arc<Self> {
        Arc::new(Self {
            extractor,
            root,
            opts,
            prior,
            limit,
            cams: Mutex::new(BTreeMap::new()),
            st: Mutex::new(State { store, jobs: VecDeque::new(), queue: VecDeque::new(), epoch: 0, worker_started: false, closed: false }),
            cv: Condvar::new(),
        })
    }

    /// Stops the worker thread (it ends after the job it is running).
    pub(crate) fn close(&self) {
        let mut st = lock(&self.st);
        st.closed = true;
        for j in st.jobs.drain(..) {
            j.cancelled.store(true, Ordering::Relaxed);
        }
        st.queue.clear();
        self.cv.notify_all();
    }

    /// Drops every hinted job and continues with `store` / `cams` (rewind, or a fresh session).
    pub(crate) fn reset(&self, store: Arc<FeatureStore>, cams: BTreeMap<String, CameraId>) {
        {
            let mut st = lock(&self.st);
            st.epoch += 1;
            for j in st.jobs.drain(..) {
                j.cancelled.store(true, Ordering::Relaxed);
            }
            st.queue.clear();
            st.store = store;
        }
        // Waits for an extraction that is still running on the old store.
        *lock(&self.cams) = cams;
    }

    /// Queues the extraction of a position. False when lookahead is off, the queue is full, or the position was hinted already
    /// (then it is true). Never blocks.
    pub(crate) fn hint(self: &Arc<Self>, images: &[FrameImage]) -> bool {
        if self.limit == 0 || images.is_empty() {
            return false;
        }
        let mut st = lock(&self.st);
        if st.closed {
            return false;
        }
        if st.jobs.iter().any(|j| j.images == images) {
            return true;
        }
        if st.jobs.len() >= self.limit {
            return false;
        }
        let job = Arc::new(Job {
            images: images.to_vec(),
            epoch: st.epoch,
            cancelled: AtomicBool::new(false),
            done: Mutex::new(None),
            cv: Condvar::new(),
        });
        st.jobs.push_back(Arc::clone(&job));
        st.queue.push_back(job);
        if !st.worker_started {
            st.worker_started = true;
            let core = Arc::clone(self);
            let spawned = std::thread::Builder::new().name("c3d-ahead".into()).spawn(move || core.work());
            if spawned.is_err() {
                st.worker_started = false;
                st.jobs.pop_back();
                st.queue.pop_back();
                return false;
            }
        }
        self.cv.notify_all();
        true
    }

    /// Number of hinted positions the session has not taken yet.
    pub(crate) fn pending(&self) -> usize {
        lock(&self.st).jobs.len()
    }

    fn work(self: Arc<Self>) {
        loop {
            let job = {
                let mut st = lock(&self.st);
                loop {
                    if st.closed {
                        return;
                    }
                    if let Some(j) = st.queue.pop_front() {
                        break j;
                    }
                    st = self.cv.wait(st).unwrap_or_else(|p| p.into_inner());
                }
            };
            let mut cams = lock(&self.cams);
            let (store, epoch) = {
                let st = lock(&self.st);
                (Arc::clone(&st.store), st.epoch)
            };
            let r = if job.epoch != epoch || job.cancelled.load(Ordering::Relaxed) {
                Err("cancelled".to_string())
            } else {
                // A panic in a backend must not leave the session waiting for a result that never comes.
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    self.extract_many(&store, &[job.images.as_slice()], &mut cams).map(|mut v| v.remove(0))
                }))
                .unwrap_or_else(|_| Err("특징 추출 중 패닉".to_string()))
            };
            drop(cams);
            job.fulfil(r);
        }
    }

    /// The result of a hinted position (waits until it is done); None when the position was not hinted. Jobs hinted before it
    /// that were never taken are dropped.
    pub(crate) fn take(&self, images: &[FrameImage]) -> Option<Result<PositionExtract, String>> {
        let job = {
            let mut st = lock(&self.st);
            let i = st.jobs.iter().position(|j| j.images == images)?;
            for stale in st.jobs.drain(..i) {
                stale.cancelled.store(true, Ordering::Relaxed);
            }
            st.jobs.pop_front()?
        };
        Some(job.wait())
    }

    /// Extracts `positions` now, in order, one call for all their images. Returns one result per position.
    pub(crate) fn run_now(&self, store: &FeatureStore, positions: &[&[FrameImage]]) -> Result<Vec<PositionExtract>, String> {
        let mut cams = lock(&self.cams);
        self.extract_many(store, positions, &mut cams)
    }

    /// Core of the extraction: the first bundle (no cameras known) is extracted alone with one camera per folder, every other
    /// position attaches its images to the known cameras (a new camera name gets a camera of its own). Intrinsics priors are
    /// applied to every new camera at once.
    fn extract_many(
        &self,
        store: &FeatureStore,
        positions: &[&[FrameImage]],
        cams: &mut BTreeMap<String, CameraId>,
    ) -> Result<Vec<PositionExtract>, String> {
        let mut out: Vec<PositionExtract> = Vec::with_capacity(positions.len());
        let mut rest = positions;
        if cams.is_empty() {
            let Some((first, tail)) = positions.split_first() else { return Ok(out) };
            rest = tail;
            let t0 = Instant::now();
            let names: Vec<String> = first.iter().map(|i| i.name.clone()).collect();
            let mut o = self.opts.clone();
            o.reader.camera_mode = CameraMode::PerFolder;
            let reports = self.extractor.extract_files(store, &self.root, &names, &o).map_err(|e| e.to_string())?;
            for im in first.iter() {
                let s = store.image_by_name(&im.name).ok_or_else(|| format!("{} 첫 영상 특징 추출 실패", im.camera))?;
                cams.insert(im.camera.clone(), s.camera_id);
            }
            let notes = self.apply_prior(store, cams)?;
            out.push(PositionExtract { reports, notes, cams: cams.clone(), elapsed: t0.elapsed() });
        }
        if rest.is_empty() {
            return Ok(out);
        }
        // With lookahead off, a single position keeps the original order of work: one image after the other (each one with
        // its own camera and prior handling); lookahead and batches extract all their images in parallel.
        let parallel = self.limit > 0 || rest.len() > 1;
        let extract_group =
            |group: &[&FrameImage], cams: &mut BTreeMap<String, CameraId>| -> Result<(Vec<ImageReport>, Vec<String>), String> {
                let items: Vec<(String, CameraMode)> = group
                    .iter()
                    .map(|im| (im.name.clone(), cams.get(&im.camera).copied().map_or(CameraMode::PerFolder, CameraMode::Existing)))
                    .collect();
                let reports = self.extractor.extract_ordered(store, &self.root, &items, &self.opts).map_err(|e| e.to_string())?;
                let mut new_cam = false;
                for im in group {
                    if !cams.contains_key(&im.camera) {
                        if let Some(s) = store.image_by_name(&im.name) {
                            cams.insert(im.camera.clone(), s.camera_id);
                            new_cam = true;
                        }
                    }
                }
                let notes = if new_cam { self.apply_prior(store, cams)? } else { Vec::new() };
                Ok((reports, notes))
            };
        let t0 = Instant::now();
        let mut done: Vec<(Vec<ImageReport>, Vec<String>)> = Vec::with_capacity(rest.len());
        if parallel {
            let all: Vec<&FrameImage> = rest.iter().flat_map(|p| p.iter()).collect();
            let (reports, notes) = extract_group(&all, cams)?;
            let mut reports = reports.into_iter();
            for (i, pos) in rest.iter().enumerate() {
                done.push((reports.by_ref().take(pos.len()).collect(), if i == 0 { notes.clone() } else { Vec::new() }));
            }
        } else {
            for pos in rest {
                let (mut reports, mut notes) = (Vec::new(), Vec::new());
                for im in pos.iter() {
                    let (r, n) = extract_group(&[im], cams)?;
                    reports.extend(r);
                    notes.extend(n);
                }
                done.push((reports, notes));
            }
        }
        let share = t0.elapsed() / rest.len() as u32;
        for (reports, notes) in done {
            out.push(PositionExtract { reports, notes, cams: cams.clone(), elapsed: share });
        }
        Ok(out)
    }

    /// Soft intrinsics prior: sets the store's cameras to the prior values (before any registration or model uses them).
    /// A camera without an entry is left alone; an entry whose length differs from the camera model's is an error.
    fn apply_prior(&self, store: &FeatureStore, cams: &BTreeMap<String, CameraId>) -> Result<Vec<String>, String> {
        let mut notes = Vec::new();
        let Some(pr) = &self.prior else { return Ok(notes) };
        let pr = pr.resolve(cams);
        for mut cam in store.cameras() {
            let before = cam.params.clone();
            if pr.apply(&mut cam)? && cam.params != before {
                notes.push(format!(
                    "[prior] 카메라 {}: 내부값 초기화 {} → {}",
                    cam.camera_id,
                    crate::session::fmt_params(&before),
                    crate::session::fmt_params(&cam.params)
                ));
                store.update_camera(cam).map_err(|e| e.to_string())?;
            }
        }
        Ok(notes)
    }
}

/// Handle that tells a session which positions are coming, so that their features are extracted ahead of time (see the module
/// documentation). Cheap to clone; usable from any thread without touching the session.
#[derive(Clone)]
pub struct Lookahead(pub(crate) Arc<AheadCore>);

impl Lookahead {
    /// Queues the feature extraction of the position with these images. Returns false when it was not queued (lookahead is off,
    /// `SessionConfig::lookahead` positions are already waiting, or the set is empty); the session then extracts the position
    /// itself when it gets there. Never blocks.
    pub fn hint(&self, frames: &FrameSet) -> bool {
        self.0.hint(&frames.images)
    }

    /// The configured lookahead depth (`SessionConfig::lookahead`; 0 = off).
    pub fn limit(&self) -> usize {
        self.0.limit
    }

    /// Positions queued or extracted but not yet processed by the session.
    pub fn pending(&self) -> usize {
        self.0.pending()
    }
}

impl std::fmt::Debug for Lookahead {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Lookahead").field("limit", &self.0.limit).field("pending", &self.pending()).finish()
    }
}
