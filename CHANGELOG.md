# Changelog

이 프로젝트는 [유의적 버전(SemVer)](https://semver.org/lang/ko/)을 따른다. 0.x 동안에는 부 버전(0.1 → 0.2)에서 호환이 깨질 수 있다.
각 버전의 날짜는 실제 작업·커밋 날짜다.

## [0.3.0] — 계획
### 추가 예정
- 이벤트 기반 파이프라인 API: 상태 값 `Session` + `step(&Session, frames) -> (Session, Vec<Event>)`.
- 프레임이 들어올 때마다 단계별로 분해된 이벤트(`FrameIngested`, `FeaturesExtracted`, `PairsMatched`, `FrameRegistered`,
  `ZonePreview`, `ZoneRefined`, `Reanchored`, `Snapshot`)를 내보냄.
- `Pipeline` 래퍼와 이벤트별 람다 훅(`.on(|e: &ZonePreview| …)`), 채널 방식 수신. 훅은 별도 큐에서 실행되어 처리를 막지 않음.

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
