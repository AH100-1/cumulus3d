[English](COMPOSE.md) | 한국어

# 실시간 합성 (`cumulus3d_cli::compose`)

`Recon::declare()…build()?.run()` 은 실행 전에 정해진 폴더 하나를 끝까지 재구성한다. `compose` 는 입력이 계속 들어오는 경우
(디코더, 네트워크 수신기, 촬영 루프)를 위한 짝이다. **유지되는 노드**가 세션·파이프라인·받아들인 프레임 기록을 갖고, 호출하는 쪽은
**지금의 입력 상태만 선언**한다. 노드는 선언을 이미 받아들인 것과 비교해 필요한 증분 작업만 한다. `Pipeline::push` 를 직접 부를 필요가 없다.

```rust
use cumulus3d_cli::compose::{CompositionHost, FrameKey, FrameState, LiveFrameSet, ReconNode, ReconcilePolicy};
use cumulus3d_cli::declare::Dense;

let host = CompositionHost::new();
let recon = ReconNode::declare()
    .images("live/images")                         // 파일 입력의 기준 폴더, 메모리 입력은 이 안에 저장
    .cameras(["camF", "camR", "camL"])
    .zones(12, 2)
    .dense(Dense::profile("fast").fusion_min_views(3))
    .reconcile(ReconcilePolicy::AppendOnly)
    .on_zone_preview(|range, cloud| println!("초벌 {}: 점 {}", range.zone, cloud.len()))
    .build(&host)?;

recon.compose(FrameState::ready(frame_set_a))?;   // 새 식별자: 위치 하나 처리
recon.compose(FrameState::ready(frame_set_b))?;
recon.compose(FrameState::ready(frame_set_b))?;   // 같은 식별자·리비전: 아무것도 안 함
let closed = recon.close()?;                      // 남은 구역 닫기, 정밀 작업 대기, 마지막 이벤트
```

## 1. 프레임 식별자와 입력

| 형식 | 뜻 |
|---|---|
| `FrameKey { source_id, sequence, revision }` | 동기화된 묶음 하나의 식별자. `sequence` 가 위치 순서, `revision` 은 정정 버전(0 = 처음 받은 것). |
| `LiveFrameSet { key, timestamp, images, gps }` | 카메라마다 한 장 + GPS. `.file(cam, rel)`, `.rgb8(cam, w, h, pixels)`, `.encoded(cam, bytes, format)`, `.gps_fix(cam, lat, lon, alt)`, `.at(time)`. |
| `FramePayload::File { relative_name }` | 영상 폴더 안의 기존 파일. 다 쓴 뒤에 선언해야 한다. |
| `FramePayload::Rgb8 { width, height, pixels }` | 디코딩된 화소. 스풀 폴더에 PNG 로 저장. |
| `FramePayload::Encoded { bytes, format }` | JPEG/PNG 바이트. 스풀 폴더에 그대로 저장. |

메모리 입력은 `<images>/<spool>/<camera>/<source>_<sequence>.<ext>` 에 임시 파일 → 이름 바꾸기로 저장되어(스풀 기본 `.live`,
`.spool(rel)` 로 변경), 재구성이 덜 쓴 영상을 읽는 일이 없다. 카메라 이름(`"camF"`)으로 된 GPS 기록은 그 카메라의 저장 이름으로 바뀐다.

## 2. 선언한 상태와 노드 동작

| 선언 | 결과(`Outcome`) |
|---|---|
| 다음 순번의 `FrameState::Ready(set)` | `Appended { position, skipped: 0 }` — 위치 하나 처리 |
| 같은 키·리비전 재선언 | `Duplicate { position }` — 특징·매칭·위치 추가 없음 |
| `FrameState::Pending { key, missing_cameras, .. }` 또는 카메라가 빠진 묶음 | `Waiting { missing }` — 다 모일 때까지 처리 안 함(`ReconNode::waiting()`) |
| 순번 건너뜀(프레임 유실) | `Appended { position, skipped: n }` + 경고 |
| 처음 보는 이전 순번(늦게 도착·순서 뒤바뀜) | 정책에 따름(아래) |
| 받아들인 순번의 더 높은 리비전(정정) | 정책에 따름(아래) |
| 받아들인 것보다 낮은 리비전 | `Ignored(StaleRevision)` |
| 노드와 다른 `source_id` | `Ignored(ForeignSource)` |
| 모르는·중복 카메라, 없는 파일, 화소 버퍼 크기 오류 | `Err(InvalidFrameSet)` |
| `FrameState::End` | `Ended` — 노드를 닫고, `close()` 가 보관한 요약을 돌려줌 |

받아들인 순번 하나가 위치 하나가 되며 순번 순서를 따른다. 입력을 버리거나 순서를 바꾼 결정은 `Event::Warning`(`compose: …`)으로도
나가므로 훅·구독 채널·`run.log` 에 남는다.

## 3. 조정 정책(`ReconcilePolicy`)

| 정책 | 늦은 도착 | 정정 | 순번 건너뜀 |
|---|---|---|---|
| `AppendOnly`(기본) | 무시 | 무시 | 받음 |
| `ReplayWindow { positions: w }` | 최근 `w` 위치 안이면 되감기(`ResetFrom`) 후 끼워 넣은 묶음과 그 뒤 묶음을 다시 처리 | 정정 위치부터 같은 방식 | 받음 |
| `InvalidateAffectedZones` | 무시 | 정정 화소를 원래 이름으로 저장하고 그 위치를 포함한 도착 구역을 다시 만든다(`InvalidateZone`). 특징·매칭·자세·GPS 는 다시 계산하지 않음 | 받음 |
| `Strict` | 오류 | 오류 | 오류 |

`ReplayWindow` 는 세션에 되감기 지점 `w + 1` 개, 다시 처리용 묶음 `w` 개를 보관한다. 범위보다 오래된 것은 `Ignored(OutsideReplayWindow)`.

## 4. 호스트와 조회

`CompositionHost` 는 노드를 id 로 보관한다(빌더 `.id("main")`, 기본 `"default"`). `reconstruct(&host, "main", state)` 로 노드를
들고 있지 않아도 입력마다 다시 선언할 수 있다. `host.close(id)` 는 노드 하나를 닫고 빼고, `host.close_all()` 은 전부.

## 5. 생산자 스레드와 역압

`node.spawn()` 은 노드를 큐 뒤의 전용 스레드에서 돌리고 `ComposeHandle` 을 돌려준다.

| 메서드 | 동작 |
|---|---|
| `send(state)` | 큐에 넣음. `Block` 이면 자리가 날 때까지 기다림 |
| `try_send(state)` | 기다리지 않음. `Block` 큐가 차면 `Err(SendError::Full(state))` (비동기 코드에서 사용) |
| `send_all(iter)` | 반복자·채널 수신기의 각 항목을 `send` |
| `stats()` | 넣은 수, 버린 수, 처리 수, 현재·최대 깊이 |
| `close()` | `End` 를 보내고 큐가 빌 때까지 기다린 뒤 상태별 결과와 종료 결과를 돌려줌 |

| `Backpressure` | 큐가 찼을 때 |
|---|---|
| `Block { max_pending }`(기본 8) | 생산자가 기다림 |
| `KeepLatest { max_pending }` | 가장 오래된 대기 묶음을 버림(순번 건너뜀으로 나타남) |
| `Unbounded` | 차지 않음 |

종류별 훅 큐 정책은 `.event_policy(EventKind::ZonePreview, QueuePolicy::LatestPerKey)`.

## 6. 빌더(`ReconNode::declare()`)

재구성 설정은 `Plan`(`declare` 와 같은 필드·TOML, `.plan(plan)` 으로 교체)이다. 계획의 위치 수·간격·GPS 파일·출력 설정은 쓰지 않는다.

`.id`, `.images`, `.cameras`, `.zones`, `.align`, `.fixed_enu_origin`, `.features`, `.match_backend`, `.gpu`,
`.incremental_triangulation`, `.stable_display`, `.display_deltas`, `.lookahead`, `.batch`, `.preview`, `.dense`, `.no_dense`(기본), `.seed`, `.reconcile`, `.backpressure`, `.source`, `.spool`,
`.output(dir, SinkOptions)`(기본 출력 파일), 훅 `.on`, `.on_event`, `.on_frame`, `.on_zone_preview`, `.on_zone_refined`,
`.on_message`, `.event_policy`, `.sync_hooks`, 그리고 설정 문제를 한 번에 모아 알려 주는 `.build(&host)`(`ComposeError::Config`).

노드 조회: `positions()`, `journal()`(위치별 순번·리비전), `waiting()`, `stats()`, `subscribe()`, `poll()`, `with_session(|s| …)`.

### 내부값 사전(intrinsics prior)

`.intrinsics_prior(IntrinsicsPrior)` 는 CLI 의 `--intrinsics-prior`, `--intrinsics-prior-sigma`, `--intrinsics-prior-dist-sigma` 에 대응한다. 모든 카메라가
주어진 값(주점 제외)에서 시작하고 번들 조정이 그 값 쪽으로 당겨진다(초점거리는 상대 σ 기본 0.005, 나머지는 절대 σ 기본 0.01).
카메라는 이름(`.cameras` 의 항목) 또는 카메라 id 로 지정하고, 파라미터는 모델 순서(OPENCV: `fx fy cx cy k1 k2 p1 p2`)다. 항목이 없는 카메라는 그대로 둔다.
`.intrinsics_prior_json(path)` 는 CLI 와 같은 JSON 파일을 읽는다(정수가 아닌 키는 카메라 이름). 빌더가 지정하지 않으면 계획의
`sparse.intrinsics_prior` 파일을 쓴다. `.build` 는 0 이하·비유한 σ, 비었거나 유한하지 않은 사전, 노드에 없는 카메라 이름을 거부한다.

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

## 7. 메모리 표시 증분(display delta)

`cumulus3d stream` 의 파일 스트림 말고도, 노드가 점 키 기반 증분 표시를 임베딩 앱에 메모리로 바로 줄 수 있다. 보이는 점마다
조밀한 `u32` 슬롯 id 가 있고, 조밀 구역 결과가 나올 때마다 라이브러리가 표시를 합성(키 해시 솎기 `stride`, 끈끈한 복셀 소유,
낮은 구역 우선, 중복 키 제거)하고 뷰어가 보는 것과 비교해 `Event::DisplayDelta { delta, zone, kind, t, frame, .. }` 를 원인 사건
바로 뒤에 전달한다.

```rust
use cumulus3d_cli::compose::{CompositionHost, ReconNode};
use cumulus3d_cli::declare::Dense;
use cumulus3d_cli::events::{Event, EventKind};
use cumulus3d_cli::live::DisplayMirror;
use cumulus3d_cli::post::LiveOptions;

let host = CompositionHost::new();
let mut viewer = DisplayMirror::new(); // 실제로는 GPU 버퍼: 색인 = id 에 쓴다
let node = ReconNode::declare()
    .images("frames").cameras(["camF", "camR", "camL"]).zones(12, 2)
    .gpu().dense(Dense::profile("fast"))
    .stable_display(true).fixed_enu_origin(true)
    .display_deltas(LiveOptions { stride: 6, ..LiveOptions::default() })
    .on(EventKind::DisplayDelta, move |e| {
        if let Event::DisplayDelta { delta, kind, zone, .. } = e {
            // 제거, 추가, 이동 순; delta.capacity = 잡아야 할 버퍼 크기
            viewer.apply(delta).unwrap();
            println!("{kind:?} zone {zone}: +{} ~{} -{} = {}", delta.adds.len(), delta.moves.len(), delta.removes.len(), delta.total_points);
        }
    })
    .build(&host)?;
```

`kind` 는 `Preview`(구역의 희소 점 초벌, 8절), `Refined`(구역의 첫 결과), `Replace`(보이던 구역을 다시 조밀화·재배치, 안정 표시가 함), `Final`(정제 전부 끝, 빈
delta). 색이 바뀌거나 위치가 `POSITION_EPS` 넘게 바뀌면 이동, 소유 구역이 바뀐 점은 제거 + 새 id 로 추가, 한 delta 에서
풀린 id 는 다음 delta 부터 재사용(작은 것 먼저). delta 에는 `seq`, `total_points`, `capacity` 가 있고 `frame` 은 점이 ENU
(`origin_gps` = 처음 선언된 GPS) 인지 표시 좌표계인지 알려 준다.

노드 없이 부품만 쓸 수도 있다: `LiveDisplay::compose_zoned`(합성), `KeyedDiff::apply(&cloud, &zones) -> DisplayDelta`(입출력
없는 비교, 트랜잭션은 `prepare`/`commit`/`abort`), `DisplayFeed::handle(&event)`(사건 구동 결합) — `cumulus3d_cli::live`/`post`.
파일 스트림(`blocks::DiffStream`, 팩 형식 4)은 `KeyedDiff` 위의 얇은 쓰기 계층이며 바이트 단위로 같다.
`.stable_display(true)` 는 정착 후 고정 표시(`--stable-display`)의 노드 옵션이다. 합성은 compose 호출 스레드에서 돌므로 큰
장면에서는 `stride` 를 크게 잡는다.

## 8. 도착 속도 따라가기 (실시간 처리량)

세션은 위치 하나를 특징 -> 매칭 -> 등록 -> 삼각측량 네 단계로 처리한다. 위치를 엄격히 하나씩 처리하면 위치당 시간이 단계 시간의
*합*이 되고, 그보다 빠르게 위치가 들어오면 밀린 일이 쌓인다. 아래 세 옵션(`node.spawn()` 한 노드)이 속도를 가장 느린 단계에 가깝게
끌어올리고 낮은 단계를 먼저 보여 준다.

| 옵션 | 기본값 | 하는 일 |
|---|---|---|
| `.lookahead(n)` | 노드 4, `Recon::run`·`cumulus3d stream` 0 (`--lookahead n`) | 큐에 있는 최대 `n` 개 위치의 특징 추출을 작업 스레드가 (한 위치의 영상은 병렬로) 앞서 해 두는 동안 앞 위치를 매칭·등록·삼각측량한다. 영상은 상태를 큐에 넣을 때(`send`) 읽으므로 파일·인코딩 페이로드가 대상이고, 원시 RGB 페이로드는 앞서 읽지 않는다(작업 스레드가 저장). 결과는 바뀌지 않는다: 영상 id, 카메라, 이벤트가 직렬 실행과 같다. |
| `.batch(n)` | 노드 3 (0 또는 1 = 끔) | 작업 스레드가 기다리는 상태를 `n` 개 이상(가져오는 것 + 큐) 보면 최대 32 개를 밀린 일 묶음(`Input::Batch`, 또는 `node.compose_batch(states)` 직접 호출)으로 가져간다: 특징 전부, 대기 중 짝 전부를 매칭 한 번, 묶음당 등록 한 번과 삼각측량 한 번. 묶음은 구역이 닫히거나 첫 모델이 만들어지는 곳에서 나뉘어 구역이 위치 하나씩 처리할 때와 같은 체인 상태를 본다. 위치별 처리와 등록 순서 오차 범위에서 같다. `ReconcilePolicy::ReplayWindow` 에서는 꺼진다(되감기가 위치마다 상태를 요구). |
| `.preview(true)` | 끔 (`--preview`) | 안정 표시 전용. 구역이 닫히는 즉시(번들 조정 전) 그 구역 고유의 등록 3D 점을 표시 좌표계에 놓아 `Event::ZonePreview { dense: false, frame: "display" }` 로 내보낸다. `.display_deltas(..)` 와 함께면 `DeltaKind::Preview` delta 로 나온다. 정제된 구역이 나중에 이를 대체한다. |

계획 키: `[live] lookahead`, `batch`, `preview` (`Plan::live`, `declare::Live`). `Session` 수준: `SessionConfig::lookahead`,
`SessionConfig::preview`, `Input::Batch`, 그리고 `Session::lookahead()` 가 돌려주는 `Lookahead` 핸들(`hint(&FrameSet)` 이 어느
스레드에서든 다가오는 위치의 추출을 시작; `Recon::run` 이 폴더의 다음 위치에 쓴다).

### Preview delta (안정 표시의 낮은 단계 먼저)

`.stable_display(true).preview(true).display_deltas(..)` 이면 구역이 두 번 보인다: 희소 초벌, 그다음 정제(조밀) 구역.

- `DeltaKind::Preview`: 추가만 있다(이동·제거 없음). 초벌 점의 소유 구역은 해당 구역이고 키의 행 필드가 `0xFFFF`
  (`image << 32 | 0xFFFF << 16 | feature`)라 깊이 지도 화소와 절대 겹치지 않는다.
- 배치: 체인이 가장 최근에 보인(정제된) 구역과 공유하는 3D 점의 견고한 Sim3, 공유 점이 적으면 공유 영상의 카메라 중심, 보인 구역이
  아직 없으면 첫 정제 구역이 시작하는 것과 같은 GPS ENU 적합. 적합이 없으면 초벌을 만들지 않는다(로그 한 줄). 이미 보인 구역은 건드리지 않는다.
- 이후 정제 구역은 `DeltaKind::Refined` 로 온다: 같은 delta 에서 그 구역의 초벌 점이 제거되고 조밀 점이 추가된다(두 키 공간이 달라 이동은 없다).
  아직 기다리는 뒤쪽 구역의 초벌은 남는다.
- 희소 결과뿐인 구역(조밀화 없음)은 초벌이 그대로 남는다.

SkyLens: `DeltaKind` 는 `#[non_exhaustive]` 이고 `Preview` 가 추가된다(`as_str()` = `"preview"`). `SlotEncoder` 는 이를 `refined` / `replace` /
`final` 옆의 새 `slot_kind` 값에 대응시킨다. 파일 스트림(`blocks::DiffStream`, 팩 형식 4)은 그대로이며 초벌을 싣지 않는다.

### 측정 (합성 비행, 40 위치 x 영상 3장, 구역 12/2, CPU, 10 코어, release)

`cargo test --release -p cumulus3d-cli --test live_throughput bench -- --ignored --nocapture --test-threads=1`:

| 변형 (모든 위치를 한꺼번에 큐에) | 위치당 초 | 삼각측량 합계 |
|---|---|---|
| 직렬(이전) | 0.165 | 3.8 초 |
| `.batch(3)` | 0.09 ~ 0.11 | 0.8 ~ 1.0 초 |

직렬 실행은 (위치마다 등록된 전 영상) 삼각측량이 지배하고, 묶음은 이를 묶음당 한 번만 치른다. CPU 를 쓰지 않는 추출 단계(영상당 250 ms
대기로 GPU 호출을 흉내)에서는 위치를 하나씩 먹이면 위치당 0.342 초, `lookahead 4` 이면 0.297 초(약 13 % 감소, 하한은 추출 자체의
0.25 초)다. CPU SIFT 는 추출이 다른 단계와 같은 코어를 놓고 겨루므로 이득이 작고, 추출이 GPU 에서 돌 때 효과가 난다. 시간은 기계
부하에 따라 달라진다.

## 9. 보장(시험으로 확인)

`crates/cli/tests/compose_synthetic.rs` 가 합성 장면으로 모든 정책을 돌린다: 재선언은 아무것도 하지 않고, 대기·불완전 묶음은 처리하지
않으며, 순번 건너뜀을 알리고, 늦은 도착과 정정은 범위 안에서 다시 처리하고 밖이면 무시하며, 정정은 해당 구역만 다시 만들고,
`Strict` 는 다음 순번 외에는 거부하고, 메모리 입력은 덜 쓴 파일 없이 저장되며, `KeepLatest` 로 띄운 노드는 대기 중인 묶음만 버린다.
배치 API(`Recon::declare().run()`)는 바뀌지 않았다.
`crates/cli/tests/live_throughput.rs` 는 밀린 일 묶음(어떤 자르기든)이 위치별 처리와 오차 범위에서 같고, lookahead 가 타이밍만 바꾸며(영상 id·특징·이벤트
동일, 되감기 후에도), 초벌이 표시 좌표계에 놓이고 자기 구역의 정제 delta 보다 먼저 오며 이미 보인 점을 옮기거나 지우지 않고, spawn 한
노드가 밀린 일을 처리함을 확인한다.
