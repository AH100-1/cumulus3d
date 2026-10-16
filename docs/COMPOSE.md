English | [한국어](COMPOSE.ko.md)

# Live composition (`cumulus3d_cli::compose`)

`Recon::declare()…build()?.run()` reconstructs a finite folder that is known before the run. `compose` is the counterpart for
input that keeps arriving (a decoder, a network receiver, a capture loop): a **retained node** owns the session, the pipeline and
the journal of accepted frames, and the caller only **declares the current input state**. The node reconciles every declaration
against what it already accepted and does only the incremental work that is needed. Direct `Pipeline::push` is not needed.

```rust
use cumulus3d_cli::compose::{CompositionHost, FrameKey, FrameState, LiveFrameSet, ReconNode, ReconcilePolicy};
use cumulus3d_cli::declare::Dense;

let host = CompositionHost::new();
let recon = ReconNode::declare()
    .images("live/images")                         // file payloads are relative to it; in-memory ones are spooled in it
    .cameras(["camF", "camR", "camL"])
    .zones(12, 2)
    .dense(Dense::profile("fast").fusion_min_views(3))
    .reconcile(ReconcilePolicy::AppendOnly)
    .on_zone_preview(|range, cloud| println!("preview {}: {} points", range.zone, cloud.len()))
    .build(&host)?;

recon.compose(FrameState::ready(frame_set_a))?;   // new identity: one position is ingested
recon.compose(FrameState::ready(frame_set_b))?;
recon.compose(FrameState::ready(frame_set_b))?;   // same identity and revision: no-op
let closed = recon.close()?;                      // close open zones, drain refinement, final events
```

## 1. Frame identity and payloads

| Type | Meaning |
|---|---|
| `FrameKey { source_id, sequence, revision }` | Identity of one synchronized set. `sequence` orders positions; `revision` versions corrections (0 = original). |
| `LiveFrameSet { key, timestamp, images, gps }` | One frame per camera plus GPS fixes. Builders: `.file(cam, rel)`, `.rgb8(cam, w, h, pixels)`, `.encoded(cam, bytes, format)`, `.gps_fix(cam, lat, lon, alt)`, `.at(time)`. |
| `FramePayload::File { relative_name }` | Existing image under the image folder. The producer must finish writing it before declaring it. |
| `FramePayload::Rgb8 { width, height, pixels }` | Decoded pixels; stored as PNG in the spool folder. |
| `FramePayload::Encoded { bytes, format }` | JPEG/PNG bytes; stored as-is in the spool folder. |

In-memory payloads are written to `<images>/<spool>/<camera>/<source>_<sequence>.<ext>` (spool default `.live`, set with
`.spool(rel)`) through a temporary file and a rename, so the reconstruction never reads a partial image. A GPS fix named after a
camera (`"camF"`) is renamed to that camera's stored image name.

## 2. Declared states and what the node does

| Declaration | Result (`Outcome`) |
|---|---|
| `FrameState::Ready(set)` with the next sequence | `Appended { position, skipped: 0 }` — one position ingested |
| same key and revision again | `Duplicate { position }` — no extraction, matching or position |
| `FrameState::Pending { key, missing_cameras, .. }` or a ready set with cameras missing | `Waiting { missing }` — nothing ingested until the set is complete (`ReconNode::waiting()`) |
| sequence jump (dropped frames) | `Appended { position, skipped: n }` + warning |
| older, never-seen sequence (late / out of order) | per policy (below) |
| higher revision of an accepted sequence (correction) | per policy (below) |
| lower revision than the accepted one | `Ignored(StaleRevision)` |
| other `source_id` than the node's | `Ignored(ForeignSource)` |
| unknown or repeated camera, missing file, wrong pixel buffer size | `Err(InvalidFrameSet)` |
| `FrameState::End` | `Ended` — the node closes; `close()` returns the kept summary |

Each accepted sequence becomes one position, in sequence order. Decisions that drop or reorder input are also emitted as
`Event::Warning` (`compose: …`), so they appear in hooks, the subscription channel and `run.log`.

## 3. Reconciliation policies (`ReconcilePolicy`)

| Policy | Late arrival | Correction | Gap |
|---|---|---|---|
| `AppendOnly` (default) | ignored | ignored | accepted |
| `ReplayWindow { positions: w }` | inserted if within the last `w` positions: rewind (`ResetFrom`) and replay the inserted set and every later set | same, from the corrected position | accepted |
| `InvalidateAffectedZones` | ignored | the corrected pixels are stored under the original names and every arrived zone containing the position is rebuilt (`InvalidateZone`); features, matches, poses and GPS are not recomputed | accepted |
| `Strict` | error | error | error |

`ReplayWindow` keeps `w + 1` rewind points in the session and the last `w` frame sets for replay. Anything older than the window is
`Ignored(OutsideReplayWindow)`.

## 4. Host and lookup

`CompositionHost` keeps the nodes by id (`.id("main")` on the builder, default `"default"`).
`reconstruct(&host, "main", state)` declares a state on that node, so a caller can re-declare on every input without holding the node.
`host.close(id)` closes and removes one node, `host.close_all()` all of them.

## 5. Producer thread and backpressure

`node.spawn()` runs the node on its own thread behind a queue and returns a `ComposeHandle`:

| Method | Behavior |
|---|---|
| `send(state)` | queue; under `Block` waits for room |
| `try_send(state)` | never waits; `Err(SendError::Full(state))` when a `Block` queue is full (usable from async code) |
| `send_all(iter)` | `send` for each item of an iterator or channel receiver |
| `stats()` | submitted, dropped, processed, current and largest depth |
| `close()` | sends `End`, waits for the queue to drain, returns the per-state outcomes and the close result |

| `Backpressure` | When the queue is full |
|---|---|
| `Block { max_pending }` (default 8) | the producer waits |
| `KeepLatest { max_pending }` | the oldest queued ready set is dropped (it then shows up as a sequence gap) |
| `Unbounded` | never full |

Per-kind hook queue policies are set with `.event_policy(EventKind::ZonePreview, QueuePolicy::LatestPerKey)`.

## 6. Builder (`ReconNode::declare()`)

Reconstruction settings are a `Plan` (same fields and TOML as `declare`; `.plan(plan)` replaces them). Positions, stride, the GPS
file and sinks of the plan are not used.

`.id`, `.images`, `.cameras`, `.zones`, `.align`, `.fixed_enu_origin`, `.features`, `.match_backend`, `.gpu`,
`.incremental_triangulation`, `.stable_display`, `.display_deltas`, `.dense`, `.no_dense` (default), `.seed`, `.reconcile`, `.backpressure`, `.source`, `.spool`,
`.output(dir, SinkOptions)` (standard output files), hooks `.on`, `.on_event`, `.on_frame`, `.on_zone_preview`, `.on_zone_refined`,
`.on_message`, `.event_policy`, `.sync_hooks`, and `.build(&host)`, which reports every configuration problem at once
(`ComposeError::Config`).

Node queries: `positions()`, `journal()` (sequence, revision per position), `waiting()`, `stats()`, `subscribe()`, `poll()`,
`with_session(|s| …)`.

### Intrinsics prior

`.intrinsics_prior(IntrinsicsPrior)` is the compose counterpart of the CLI's `--intrinsics-prior`, `--intrinsics-prior-sigma` and
`--intrinsics-prior-dist-sigma`: every camera starts from the given parameters (principal point kept) and the bundle adjustment is pulled towards
them (focal length: relative σ, default 0.005; other parameters: absolute σ, default 0.01). Cameras are keyed by camera name (an entry of
`.cameras`) or by camera id; parameters are in the model's order (OPENCV: `fx fy cx cy k1 k2 p1 p2`). A camera without an entry is left alone.
`.intrinsics_prior_json(path)` reads the CLI's JSON file (keys that are not integers are camera names). A plan's
`sparse.intrinsics_prior` file is used when the builder sets none. `.build` rejects non-positive or non-finite σ, an empty or non-finite prior and
names that are not node cameras.

```rust
use cumulus3d_cli::intrinsics::IntrinsicsPrior;

let prior = IntrinsicsPrior::new()
    .sigmas(0.005, 0.01)
    .opencv("camF", [1607.39, 1611.24, 1024.0, 576.0, 0.082, -0.084, 0.0013, 0.0014])
    .opencv("camR", [1601.41, 1599.46, 1024.0, 576.0, 0.077, -0.079, 0.0006, -0.0014]);
let node = ReconNode::declare()
    .images("live/images")
    .cameras(["camF", "camR"])
    .stable_display(true)
    .intrinsics_prior(prior)
    .build(&host)?;
```

## 7. In-memory display deltas

Besides the file stream of `cumulus3d stream`, a node can hand the point-keyed incremental display to the embedding application
in memory. Every shown point owns a dense `u32` slot id; after each dense zone result the library composes the display
(key-hash thinning with `stride`, sticky voxel ownership, lowest zone wins, repeated keys dropped), diffs it against what the viewer
shows and delivers `Event::DisplayDelta { delta, zone, kind, t, frame, .. }` right behind the causing event.

```rust
use cumulus3d_cli::compose::{CompositionHost, ReconNode};
use cumulus3d_cli::declare::Dense;
use cumulus3d_cli::events::{Event, EventKind};
use cumulus3d_cli::live::DisplayMirror;
use cumulus3d_cli::post::LiveOptions;

let host = CompositionHost::new();
let mut viewer = DisplayMirror::new(); // your GPU buffers: write at index = id
let node = ReconNode::declare()
    .images("frames").cameras(["camF", "camR", "camL"]).zones(12, 2)
    .gpu().dense(Dense::profile("fast"))
    .stable_display(true).fixed_enu_origin(true)
    .display_deltas(LiveOptions { stride: 6, ..LiveOptions::default() })
    .on(EventKind::DisplayDelta, move |e| {
        if let Event::DisplayDelta { delta, kind, zone, .. } = e {
            // removes, then adds, then moves; delta.capacity = buffer size to allocate
            viewer.apply(delta).unwrap();
            println!("{kind:?} zone {zone}: +{} ~{} -{} = {}", delta.adds.len(), delta.moves.len(), delta.removes.len(), delta.total_points);
        }
    })
    .build(&host)?;
```

`kind` is `Refined` (first result of a zone), `Replace` (a shown zone re-densified or re-placed, which stable display does) or
`Final` (all refinements done, empty delta). A colour change or a position change beyond `POSITION_EPS` is a move; a point whose
owner zone changes is a remove plus an add with a new id; ids freed in a delta are reused from the next one (lowest first). The
delta carries `seq`, `total_points` and `capacity`. `frame` tells whether the points are in the ENU frame (`origin_gps` = first GPS
fix declared) or the display frame.

Without a node, use the pieces directly: `LiveDisplay::compose_zoned` (composition, `LiveOptions` = stride, voxel factor,
relocate), `KeyedDiff::apply(&cloud, &zones) -> DisplayDelta` (I/O-free diff; `prepare` / `commit` / `abort` for transactional
use) and `DisplayFeed::handle(&event)` (the event driven combination), all in `cumulus3d_cli::live` / `post`. The file stream
(`blocks::DiffStream`, pack format version 4) is a thin writer over `KeyedDiff` and unchanged byte for byte.
`.stable_display(true)` is the node option for the settle-then-freeze display (`--stable-display`); the composition runs on the
composing thread, so keep `stride` coarse for large scenes.

## 8. Guarantees (covered by tests)

`crates/cli/tests/compose_synthetic.rs` runs every policy on the synthetic scene: duplicate declarations are no-ops, pending and
incomplete sets ingest nothing, gaps are reported, late arrivals and corrections are replayed inside the window and ignored outside
it, corrections rebuild exactly the affected zones, `Strict` rejects anything but the next sequence, in-memory payloads are spooled
without partial files, and a spawned node with `KeepLatest` drops only queued ready sets. The batch API
(`Recon::declare().run()`) is unchanged. `crates/cli/tests/display_delta.rs` checks that the deltas of a stable-display session rebuild
exactly the composed display.
