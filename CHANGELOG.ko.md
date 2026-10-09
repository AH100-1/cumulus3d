[English](CHANGELOG.md) | 한국어

# Changelog

0.3.0 까지는 skyrecon 이라는 이름으로 개발됨.

이 프로젝트는 [유의적 버전(SemVer)](https://semver.org/lang/ko/)을 따른다. 0.x 동안에는 부 버전(0.1 → 0.2)에서 호환이 깨질 수 있다.
각 버전의 날짜는 실제 작업·커밋 날짜다.

## [0.3.1] — 2026-10-09
### 변경
- 프로젝트 이름을 cumulus3d 로 변경. 크레이트 `cumulus3d-core`/`-features`/`-matching`/`-ba`/`-sfm`/`-align`/`-dense`/`-cuda`/`-cli`,
  실행 파일 `cumulus3d`, 저장소 https://github.com/AH100-1/cumulus3d, 환경 변수 접두어 `CUMULUS3D_`.
- 특징 저장소 파일 표지를 `C3DFS` 로 변경. 옛 표지로 저장된 파일도 그대로 읽는다. 동작은 바뀌지 않았다.

## [0.3.0] — 2026-10-09
### 추가
- 이벤트 계약 `events`: `Event`/`EventKind`/`Meta`(세션 버전·발생 시각)/`ZoneRange`/`Command`.
  단계별 이벤트 `Started`, `FrameIngested`, `FeaturesExtracted`, `PairsMatched`, `ModelInitialized`, `FrameRegistered`,
  `PositionDone`, `ZoneArrived`, `ZonePreview`, `RefineStarted`, `ZoneAdjusted`, `ZoneRefinedPose`, `ZoneRefined`,
  `BaseAdopted`, `Reanchored`, `Snapshot`, `AllPositionsDone`, `AllRefinedDone`, `Finished`, `Reset`, `ZoneInvalidated`,
  `Log`, `Warning`, `Error`. 구역 이벤트는 모델·점군(`Arc`)을 함께 싣는다. `Event::timeline_text` 가 timeline.txt 문구를 준다.
- 상태 값 세션 `session`: `Session` + `step(&Session, FrameSet) -> (Session, Vec<Event>)`, `poll`, `command`, `finish`.
  위치 하나의 프레임 묶음(`FrameSet`)마다 특징 → 매칭 → 등록·삼각측량 → 구역 초벌/정밀(배경)을 이벤트로 내보내며 파일은 쓰지 않는다.
  `SessionConfig`(구역·겹침, 전체 위치 수, GPS, 조밀화 설정, 되감기 지점 수), `Command::ResetFrom`·`InvalidateZone`.
- 람다 훅 파이프라인 `pipeline::Pipeline`: `.on(EventKind, 훅)`, `.on_any`, `.on_zone_preview`/`.on_zone_refined`/
  `.on_snapshot`/`.on_position_done`/`.on_message`, 채널 수신 `subscribe`. 훅은 종류별 워커 레인에서 돌아 처리를 막지 않고
  (`sync(true)` 면 즉시 동기 실행), 종류별 큐 정책(`KeepAll`/`LatestPerKey`/`LatestOnly`), 훅 패닉 격리(`Error` 이벤트), `finish` 요약.
- 기본 출력 훅 `sinks`: `TimelineSink`(timeline.txt), `RunLogSink`(run.log·DONE), `ZonePlySink`(full/*.ply), `ModelSink`(work/models),
  `SnapshotSink`(초벌 정렬·마스킹·사건별 스냅샷·manifest.json·재고정 final_frame). `sinks::attach(pipeline, out, opts)` 로 한 번에 붙이고,
  종류별 목록 `default_sinks` 도 있다. 출력 문구·파일 이름·manifest 형식은 0.2 의 `skyrecon stream` 과 같다.
- 후처리 메모리 입력 `post::run_zones`(`PostZones`): 이벤트로 모은 구역 점군·모델을 바로 받는다. 기존 `post::run` 은 이를 감싼다.
- 예제 `examples/hooks.rs`(합성 장면에 람다 훅 + 기본 출력 훅), 시험 `tests/sinks_equivalence.rs`
  (`run_stream` 출력과, 같은 이벤트 열을 기본 훅에 흘린 출력의 timeline·run.log·파일 목록·manifest·점 수 비교).
### 변경
- `util::Logger::clock_at`/`stamped_at`: 주어진 시각으로 `[HH:MM:SS]` 머리를 붙인다(이벤트 발생 시각 기록용).

## [0.2.0] — 작업 중 (2026-10-09)
### 추가
- 조밀화 융합 실행기 `skyrecon densify --fusion-variants <파일>`: 깊이맵은 한 번만 만들고 융합 설정만 바꿔 여러 결과를 냄.
- 융합 옵션 `--fusion-residual none|release|second-pass`(남은 픽셀 2차 융합)와 `--residual-*`, 2차 점만 쓰는 `--residual-out`.
- 점수 기반 융합 `--fusion-mode score`(`--score-tau`, `--score-sigma-e`, `--score-sigma-theta`, `--score-lambda`).
- 조밀화 튜닝 기록 `docs/tuning/`: 밀도·입체감 스윕(2026-10-09), 잔여 픽셀 융합 비교(2026-10-09).
### 변경 예정
- 조밀화 기본값(융합 최소 일치 뷰, 법선 허용 각도, 깊이맵 필터, 중앙값 필터, 틈 메우기)을 스윕 결과로 갱신.

## [0.1.0] — 2026-10-08
### 추가
- 한 프로세스 점진 재구성 파이프라인 `skyrecon stream`: 위치 단위 도착 → 특징 추출 → 짝 매칭 → 전역 SfM → 영상 등록·삼각측량 →
  구역별 초벌(BA 없음)·정밀(BA) → GPS(ENU) 정렬 → 조밀화 → 단계별 스냅샷·재고정.
- 크레이트: core, features(SIFT), matching(두 뷰 기하), ba(번들 조정), sfm(전역·증분), align(GPS·Sim3), dense(왜곡 보정·GPU 다시점 스테레오),
  cuda(GPU 백엔드), cli.
- 모델 파일 형식(cameras/images/points3D) 읽기·쓰기와 단계별 호환 하위 명령(`interop`).
### 측정 (Tesla V100, 드론 3대 × 80위치 = 240장)
- 전체 스트리밍 마지막 정밀본 648초, 조밀화 240장 61초, 1,347만 점.
