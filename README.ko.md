[English](README.md) | 한국어

# cumulus3d

**cumulus3d** 는 영상이 들어올수록 3D 점군이 구름처럼 쌓이는 점진 재구성 라이브러리다.
드론 전용이 아니라 영상 일반을 지향하며, 현재 기본 입력 구성은 3카메라 편대다.

영상으로 3D 점군을 **점진적으로** 만드는 Rust 워크스페이스.
영상이 위치 단위로 도착하는 동안 특징 추출 → 매칭 → SfM → GPS 정렬 → 조밀화를 한 프로세스 안에서 잇고,
구역마다 빠른 초벌(BA 없음)과 정밀본(BA 있음)을 내서 정밀 지도(ENU) 위에 단계별 스냅샷을 쌓는다.
중간 데이터베이스나 디스크 모델 폴더 없이 메모리 자료 구조로 단계를 넘긴다.

## 크레이트

| 크레이트 | 역할 |
|---|---|
| `crates/core` (`cumulus3d-core`) | 카메라 모델, 기하(Rigid3/Sim3), 재구성 자료 구조, 대응 그래프, 특징 저장소, RANSAC, PLY/GPS 입출력, 모델 파일 형식(`interop`) |
| `crates/features` (`cumulus3d-features`) | 영상 읽기·EXIF·카메라 초기화, SIFT 검출·기술자 |
| `crates/matching` (`cumulus3d-matching`) | 기술자 매칭, 짝 목록, 두 뷰 기하(E/F/H) 추정·검증 |
| `crates/ba` (`cumulus3d-ba`) | 번들 조정(신뢰 영역 LM, Schur 보완 선형 풀이), 절대 자세 정제 |
| `crates/sfm` (`cumulus3d-sfm`) | 전역 SfM(회전 평균 + 위치 추정), 영상 등록, 삼각측량 |
| `crates/align` (`cumulus3d-align`) | GPS ↔ ENU 변환, 모델 정렬(Umeyama, 견고 추정), 점군 재기준 |
| `crates/dense` (`cumulus3d-dense`) | 왜곡 보정, 장면 변환, PatchStereo 깊이맵, 필터·융합 → 조밀 점군 |
| `crates/cuda` (`cumulus3d-cuda`) | GPU 백엔드(PatchStereo, 기술자 매칭, SIFT 스케일 공간). CUDA 없는 기계에서도 빌드됨 |
| `crates/cli` (`cumulus3d-cli`) | 실행 파일 `cumulus3d`: `cumulus3d stream` 과 단계별 하위 명령 |

전체 공개 API 지도(무엇을 하려면 어떤 함수를 쓰나, 크레이트 의존 관계)는 [docs/API.ko.md](docs/API.ko.md) 에 있다.
각 크레이트의 공개 항목은 `crates/*/README.ko.md`, 실행 파일 사용법은 `crates/cli/README.ko.md` 에 있다.

## 빌드·시험

```bash
cargo build --release
cargo test --release --workspace
cargo clippy --workspace --all-targets
```

## 사용

```bash
# 점진 파이프라인(입력: images/camF|camR|camL/*.jpg + gps_ref.txt)
target/release/cumulus3d stream --src <입력 폴더> --out <출력 폴더>
# GPU 백엔드 사용(CUDA 12.x)
target/release/cumulus3d stream --src <입력> --out <출력> --gpu
# 단계별 하위 명령(기존 스크립트용): feature_extractor, matches_importer, global_mapper, image_registrator,
# point_triangulator, bundle_adjuster, model_aligner, model_analyzer, model_converter, image_deleter,
# image_undistorter, densify
target/release/cumulus3d model_analyzer --path <모델 폴더>
```

## 이벤트 기반 구조

파이프라인은 **상태 값 + 리듀서 + 이벤트 + 훅** 네 층으로 나뉜다. 내부 계산은 상태 객체가 맡고,
바깥에는 "입력을 넣으면 새 상태와 이벤트가 나온다"는 함수형 엔드포인트만 보인다.

```
새 프레임 묶음 ──▶ step(&Session, FrameSet) ──▶ (새 Session, Vec<Event>)
                                                  │
                     Pipeline (큐 · 정책 · 패닉 격리)┤
          ┌──────────────┬──────────────┬─────────┴────┬──────────────┐
   on(FrameRegistered) on_zone_preview on_zone_refined   on_any        subscribe()
      (람다 훅)          (람다 훅)       (람다 훅)    (기본 출력 훅)   (채널 수신)
```

### 1. 상태와 리듀서 (`session`)

| 함수 | 하는 일 |
|---|---|
| `step(&Session, FrameSet) -> (Session, Vec<Event>)` | 위치 하나의 프레임 묶음 처리: 특징 추출 → 짝 매칭 → 등록·삼각측량 → 구역이 닫히면 초벌 조밀화, 정밀 작업은 배경에서 시작 |
| `poll(&Session)` | 끝난 배경 정밀 작업의 결과를 이벤트로 수거(기다리지 않음) |
| `command(&Session, Command)` | `ResetFrom(position)`: 그 위치 직전 상태로 되감기, `InvalidateZone(k)`: 구역 k 를 다시 만듦 |
| `finish(&Session)` | 남은 구역을 닫고 배경 작업을 모두 기다려 수거 |

- `Session` 은 값이다. 모델·특징 저장소는 `Arc` 로 공유하고 바뀔 때만 복사하므로, 상태를 주고받아도 비용이 작다.
- 같은 입력을 넣으면 같은 이벤트가 나온다(시드 고정 시). 이벤트마다 단조 증가하는 세션 버전이 붙는다.

### 2. 이벤트 (`events`)

프레임 묶음 하나를 넣을 때마다 단계별로 나뉜 결과가 이벤트로 나온다. 점군은 매번 전체가 아니라 바뀐 구역 조각만 실린다.

| 시점 | 이벤트 |
|---|---|
| 시작 | `Started` |
| 위치마다 | `FrameIngested` → `FeaturesExtracted`(영상마다) → `PairsMatched` → `ModelInitialized`(첫 모델일 때) → `FrameRegistered`(영상마다) → `PositionDone` |
| 구역이 닫힐 때 | `ZoneArrived` → `ZonePreview`(초벌 점군, BA 없음) |
| 배경 정밀 작업 | `RefineStarted` → `ZoneAdjusted` → `ZoneRefinedPose`(BA·GPS 정렬 후 자세 통계) → `ZoneRefined`(정밀 점군, 같은 구역 초벌을 대체) |
| 정밀본 채택 | `BaseAdopted`(다음 등록부터 정밀 모델 위에서 이어 감) |
| 좌표계 갱신 | `Reanchored`(이미 보낸 구역의 Sim3 변환만), `Snapshot` |
| 명령 결과 | `Reset`, `ZoneInvalidated` |
| 끝 | `AllPositionsDone` → `AllRefinedDone` → `Finished`(단계별 시간·최종 모델 통계) |
| 기타 | `Log`, `Warning`, `Error` |

엣지 쪽은 "추가(`ZonePreview`) · 대체(`ZoneRefined`) · 변환(`Reanchored`)" 세 가지만 처리하면 화면 상태를 유지할 수 있다.

### 3. 파이프라인과 훅 (`pipeline`)

- `Pipeline::new(reducer)` 가 리듀서를 감싸 `push(input)` / `poll()` / `command(c)` / `finish()` 를 제공한다.
- 훅: `.on(EventKind, |e| …)`, `.on_any(…)`, 편의 메서드 `.on_zone_preview`, `.on_zone_refined`, `.on_snapshot`,
  `.on_position_done`, `.on_message`. 채널로 받으려면 `.subscribe() -> Receiver<Event>`.
- 실행: 기본은 비동기(종류별 작업 스레드 큐, 종류 안 순서 보존)라 느린 훅이 계산을 막지 않는다. `.sync(true)` 면 같은 스레드에서 즉시 실행.
- 큐 정책 `.policy(kind, …)`: `KeepAll`(기본), `LatestPerKey`(같은 구역의 처리 전 이전 이벤트 버림 — 초벌 화면 갱신용), `LatestOnly`.
- 훅이 패닉해도 파이프라인은 계속 돌고, 구독 채널로 `Error` 이벤트가 나간다.
- `finish()` 는 `(리듀서, Summary)` 를 돌려준다(이벤트 수, 종류별 수, 훅 호출·패닉 수, 정책으로 버린 수).
- 리듀서는 `pipeline::Reducer` trait 로 추상화되어 있어 `Session` 대신 다른 상태 기계도 같은 구동기로 돌릴 수 있다.

### 4. 기본 출력 훅 (`sinks`)

`sinks::attach(pipeline, out, &SinkOptions)` 한 줄로 `cumulus3d stream` 과 같은 파일을 만드는 훅 묶음이 붙는다.

| 훅 | 출력 |
|---|---|
| `TimelineSink` | `timeline.txt` (사건 시각 + 문구) |
| `RunLogSink` | `run.log`, `DONE` |
| `ZonePlySink` | `full/{preview,refined}/*.ply` |
| `ModelSink` (`save_models`) | `work/models/…` |
| `SnapshotSink` | `aligned/`, `snapshots/event_NN_*.ply`, `snapshots/manifest.json`, `final_frame/` |

`cumulus3d stream` 자체도 이 구조 위에서 돈다: 세션 + 기본 출력 훅을 조립하고 위치마다 `push` 한 뒤 `finish` 한다.

### 라이브러리: 파이프라인 + 람다 훅

위치 하나의 프레임 묶음을 넣을 때마다 세션이 단계별 이벤트(`ZoneArrived`, `ZonePreview`, `ZoneRefined`, …)를 내고,
등록한 훅이 그것을 받는다. `sinks::attach` 는 `cumulus3d stream` 과 같은 파일
(timeline.txt, run.log, full/, aligned/, snapshots/, manifest.json, final_frame/)을 만드는 기본 훅을 붙인다.

```rust
use cumulus3d_cli::events::{Event, EventKind};
use cumulus3d_cli::pipeline::Pipeline;
use cumulus3d_cli::session::{FrameSet, Input, Session, SessionConfig};
use cumulus3d_cli::sinks::{self, SinkOptions};

let mut cfg = SessionConfig::new("data/images");
cfg.gps = cumulus3d_core::io::read_gps_file("data/gps_ref.txt")?;
cfg.total_positions = Some(80);

let p = Pipeline::new(Session::new(cfg))
    .on_zone_preview(|zone, cloud| println!("초벌 {}: {} 점", zone.zone, cloud.len()))
    .on_zone_refined(|zone, cloud| println!("정밀 {}: {} 점", zone.zone, cloud.len()))
    .on(EventKind::PositionDone, |e: &Event| println!("{}", e.timeline_text().unwrap()));
let mut p = sinks::attach(p, "out".as_ref(), &SinkOptions { echo: false, save_models: false })?;

for pos in 0..80 {
    let frames = ["camF", "camR", "camL"].map(|c| (c, format!("{c}/{c}_{:04}.jpg", pos * 3)));
    p.push(Input::Frames(FrameSet::new(frames)));
}
let (_session, summary) = p.finish(); // 남은 정밀 작업을 기다리고 훅 큐를 비운다
```

합성 장면으로 바로 돌려 보는 예: `cargo run --release -p cumulus3d-cli --example hooks -- <출력 폴더>`.

## 라이선스

MIT 또는 Apache-2.0 중 선택(`LICENSE-MIT`, `LICENSE-APACHE`).
