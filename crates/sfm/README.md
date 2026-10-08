# skyrecon-sfm

희소 재구성 단계: 전역 SfM(상대 자세 → 회전 평균 → 트랙 → 위치 추정 → BA), 기존 모델에 새 영상 등록,
기존 점을 유지하는 삼각측량. 입력은 `skyrecon-core` 의 `FeatureStore`(특징·두 뷰 기하)와 `MatchGraph`,
출력은 `Reconstruction`.

## 진입점

- `global_mapper(&FeatureStore, &MatchGraph, &GlobalSfmOptions) -> Result<GlobalMapperOutput>`
  - `GlobalMapperOutput { reconstruction, view_graph: ViewGraph, summary: GlobalMapperSummary, failure: Option<String> }`.
    중간 단계가 실패해도 그때까지의 재구성을 돌려주고 `failure` 로 알린다. 모델은 항상 1개(최대 연결 성분).
  - `GlobalSfmOptions::default()`: BA 3회 + 재삼각측량. `GlobalSfmOptions::script()`: 파이프라인 설정
    (BA 0회, 재삼각측량 생략, 트랙 상한 100000).
  - 색 추출은 호출자가 `Reconstruction::extract_colors` 로 한다.
- `register_images(&mut rec, &store, &graph, &RegistrationOptions) -> Result<RegistrationReport>`:
  누락 영상 추가 → 자세 없는 영상마다 한 번 시도 → 실패 영상 삭제.
  단일 영상은 `register_image(rec, graph, id, opts) -> RegisterOutcome`.
  보조: `collect_2d3d_correspondences`, `num_visible_points3d`, `visibility_score`, `add_missing_images`.
  순서는 `RegistrationOrder::{ImageId(기본, 영상 id 오름차순) | VisibilityScore}`.
- `triangulate_points(&mut rec, &graph, &PointTriangulatorOptions) -> Result<TriangulationReport>`:
  기존 점 연장 → 새 점 생성 → 트랙 완성·병합 → 재삼각측량 → 정제 → 필터. `clear_points` 기본 거짓.
  - `scope: TriangulationScope::{AllRegistered(기본) | Images(새 영상)}` — `Images` 는 새 영상만 삼각측량하고
    바뀐 점만 완성·병합·정제·필터, 새 영상이 낀 쌍만 재삼각측량한다.
  - `refiner: PointRefiner::{BundleAdjuster(기본, skyrecon-ba) | PerPoint(점별 3변수 LM, 같은 해·더 빠름) | None}`.

## 모듈

| 모듈 | 내용 |
|---|---|
| `global_mapper` | `init_reconstruction`, `build_view_graph`(짝 병렬 상대 자세), `rotation_averaging_round`, `post_positioning_filters`, `bundle_adjustment_stage`, `retriangulation_stage`, 필터 `filter_angular_error`·`filter_angular_error_prior_focal`·`filter_normalized_reproj_error` |
| `rotation_averaging` | `ViewGraph { edges: Vec<ViewGraphEdge> }`(`largest_connected_component`, `invalidate_outside`, `filter_by_relative_rotation`), `solve_rotation_averaging`(최대 신장 트리 → L1 ADMM → IRLS Geman-McClure), `mst_initialization`, `admm_l1`, `l1_regression` |
| `tracks` | `establish_tracks(rec, graph, view_graph, TrackOptions)`: 합집합-찾기 → 영상 내 일관성 → 최소 뷰 수 → (길이, id) 내림차순 선별. `filter_candidate_tracks`, `select_tracks` |
| `positioning` | `global_positioning(rec, PositionSolverOptions)`: 점-카메라 방향 제약(회전 고정), Huber 손실, 자체 LM(점 슈어 소거 + 프레임 중심 밀집 축약 계통) |
| `absolute_pose` | `p3p`(최대 4해), `epnp`, `solve_abs_pose(cam, px, pts, AbsolutePoseOptions)`: P3P + EPnP LO-RANSAC |
| `triangulation` | `triangulate_dlt`, `triangulate_multi_view`, `triangulate_midpoint`, `has_positive_depth`, `angular_error`, `estimate_triangulation(obs, TriangulationRansacParams)` |
| `triangulator` | `TrackTriangulator`(`triangulate_image`, `complete_track(s)`, `merge_track(s)`, `retriangulate`), `TrackTriangulatorOptions`, `refine_points` |
| `math` | `umeyama`(평가용), `CsrMatrix` |
| `synthetic` | (doc(hidden)) 드론 3대 합성 장면 생성기 — 시험용 |

## 결정성

- 모든 순회는 id 오름차순: 회전 평균 루트·게이지 = 최소 활성 영상 id, 트랙 id = 정렬된 성분 순, 등록 순서 = 영상 id 오름차순.
- RANSAC 시드 기본 `Some(0)`(등록은 영상 id 로 파생). 삼각측량 RANSAC 은 조합 표본(난수 없음).
- 영상 삼각측량은 2D 점별 계획을 병렬로 계산하고, 순차 적용 때 참조 상태가 바뀐 점만 다시 계획한다 → 순차 실행과 같은 결과.
- 위치 추정의 병렬 합산은 고정 덩어리 단위라 스레드 수와 무관하게 비트 단위로 같다.

## 설계 결정

- 첫 등록 카메라의 초점 추정·내부값 정제 경로는 두지 않는다. 항상 초점 고정 경로로 등록하고, `refine_abs_pose` 는 자세만 정제한다.
- 최대 연결 성분 동률 → 최소 id 가 작은 성분. 최대 신장 트리 동률 → (id1, id2) 오름차순.
- 전역 매퍼 BA 의 Huber 척도는 1.0px.
- 위치 추정 감쇠는 단순 μ/3·×ν 규칙, 견고 손실은 IRLS 가중.
- 회전 평균 정규 행렬은 밀집 촐레스키(수백 장 이하 규모 기준).

## 시험

`cargo test --release -p skyrecon-sfm`: 최소해법 정확도(P3P·EPnP), LO-RANSAC 견고성, 회전 평균·위치 추정 정확도,
트랙·필터·등록 경계값, 그리고 드론 3대 합성 장면(48장)의 전역 매퍼·순차 등록·삼각측량 전 과정.
