# skyrecon-matching

SIFT 기술자 매칭과 두 뷰 기하 검증을 맡는다. 짝 목록 파일을 읽어 짝마다 기술자를 매칭하고,
E/F/H LO-RANSAC 으로 기하를 검증해 결과를 `FeatureStore` 에 기록한다. 매칭 연산은 `MatcherBackend`
트레이트로 분리돼 있어 GPU 백엔드(`skyrecon-cuda`)로 바꿀 수 있다.

## 파이프라인 (`pipeline`, `pairs`)
- `parse_pair_list(text, |name| Option<ImageId>) -> PairList{pairs, missing_names}` / `read_pair_list(path, lookup)`:
  trim, 빈 줄·`#` 무시, 구분자 `' '` 하나(탭 미지원), 무순서 중복은 첫 등장 방향만, 자기 짝 제외.
- `match_pairs(&FeatureStore, &[(id1,id2)], &PairMatchingOptions, &dyn MatcherBackend) -> Result<MatchingStats>`
  - 위치 도착마다 **누적 짝 목록을 그대로 넘겨도** 원시 매칭 + 기하가 모두 있는 짝은 건너뜀(기하만 있으면 다시 매칭, 매칭만 있으면 검증만).
  - 블록(1225) 단위, 짝 단위 rayon 병렬(매칭 + 검증), 기록은 입력 순서. 원시 매칭 < 15 → 빈 목록, 인라이어 < 15 → 기본 기하(UNDEFINED). 처리한 짝은 항상 두 행 기록.
  - 실제 max_num_matches = min(옵션 32768, store.largest_keypoint_count()).
- `match_pair_list_file(store, path, opts, backend) -> (PairList, MatchingStats)`, `verify_pair(store, id1, id2, matches, geom_opts)`.
- `PairMatchingOptions{sift, geometry, max_num_matches, block_size, skip_geometric_verification}`, `MatchingStats{num_skipped_existing, num_matched, num_verified_only, num_valid_geometries}`.

## 기술자 매칭 (`descriptor`)
- `trait MatcherBackend: Send+Sync { fn top2(d1,n1,d2,n2) -> (행 Vec<Top2>, 열 Vec<Top2>); fn match_descriptors(d1,d2,&DescriptorMatchOptions,max) (기본 구현) }`.
  GPU 백엔드는 `top2` 만 구현(정수 내적 GEMM + 행/열 top-2 에필로그).
- `CpuMatcher{row_block:128, col_tile:64}`: u8×u8→u32 정수 내적, 4×1 마이크로커널, rayon 행 블록 병렬, 열 top-2 는 블록 순서대로 결합 병합 → 순차 구현(`top2_naive`)과 비트 일치. 8192×8192 ≈ 55 ms(M 시리즈 CPU).
- `Top2{best, idx, second}`: 초기 0, 엄격히 큼 갱신(동점 = 앞 인덱스).
- `DescriptorMatchOptions{max_ratio 0.8, max_distance 0.7, cross_check true, rule: AcceptRule::{Gpu(기본) | CpuBruteForce}}`.
- `dot_to_angle(d) = acos(min(d/262144,1))`(f32), `apply_tests(rows, cols, opts, max)`: 영상1 인덱스 오름차순, 교차 검사(상대 쪽도 비율·거리 통과), max 개에서 자름. 입력 기술자도 앞 max 개만.

## 해법·잔차 (`essential`, `estimators`) — core `ransac::Estimator` 구현
- `essential_five_point(rays1, rays2) -> Vec<Mat3>` (N ≥ 5, Nistér 10×20 → B(z) → 10차 동반행렬, 노름 1, ≤ 10개), `essential_eight_point` (단위 광선, rank-2 만).
- `fundamental_seven_point(x1,x2) -> Vec<Mat3>` (≤ 3), `fundamental_eight_point` (하틀리 정규화, rank-2), `homography_dlt(x1,x2,normalize) -> Option<Mat3>` (4점 LU h33=1 / N점 SVD + 랭크 검사, |det| < 1e-8 거부).
- `sampson_error_sq(M,&a,&b)` (분모 0 → ∞), `homography_transfer_error_sq` (비유한 → ∞).
- 추정기: `EssentialFivePointEstimator`(광선), `Fundamental7PtEstimator`, `FundamentalEightPointEstimator`, `HomographyEstimator{normalize}`, `TranslationEstimator`.

## 두 뷰 기하 (`two_view`)
- `estimate_two_view(cam1, kps1, cam2, kps2, matches, &TwoViewOptions) -> TwoViewGeometry` (원시 결과; DEGENERATE 가능).
  두 카메라 `focal_from_prior` → 보정 경로(E/F/H 세 LO-RANSAC, E 임계 (4/f̄1+4/f̄2)/2, rayon::join 동시 실행), 아니면 비보정(F/H). 정지 매칭 필터·H 강제·다중 모델·워터마크·상대 자세 옵션 포함.
- `finalize_geometry(tvg, 15)`: 인라이어 < 15 → `TwoViewGeometry::default()`.
- 판정 함수(경계 테스트용): `decide_calibrated(e,f,h: ModelOutcome, opts) -> Decision{config, mask: Option<MaskChoice::{E,F,H}>}`, `decide_uncalibrated(f,h,opts)`.
- `is_watermark(cam1,kps1,cam2,kps2,inliers,opts)`.
- `TwoViewOptions` 기본: min_num_inliers 15, ransac{max_error 4, min_inlier_ratio 0.25, conf 0.999, ×3, trials 100..10000, seed None}, E/F 0.95, H 0.8, 워터마크 0.7/0.1/4px, compute_relative_pose false, normalize_homography false, parallel_models true.
  `ransac.random_seed = Some(s)` 이면 (s, pair_id, 모델 종류)로 파생 시드(`derive_seed`) → 재현 가능.

## 자세 (`pose`) — sfm 공용
- `decompose_essential(&E) -> Option<[(R,t);4]>` (순서 (Ra,t),(Rb,t),(Ra,−t),(Rb,−t), t 단위).
- `triangulate_midpoint(&R,&t,&r1,&r2) -> Option<Vec3>` (cam1_to_cam2, 카메라1 좌표, λ ≤ ε → None).
- `pose_from_essential(&E, rays1, rays2) -> Option<(Rigid3, Vec<Vec3>)>` (cheirality, 동점은 나중 후보).
- `decompose_homography(&H,&K1,&K2) -> Vec<(R,t,n)>` (H_n ∝ R − t nᵀ, d = 1; 순수 회전이면 1개), `pose_from_homography(...) -> Option<(Rigid3, n, pts)>`.
- `recover_two_view_pose(cam1,cam2,kps1,kps2,&mut tvg) -> bool` (PoP → PLANAR/PANORAMIC 확정, tri_angle 중앙값).
- `refit_and_estimate_relative_pose(...)`: 전역 매퍼 준비용(행렬 없으면 8점 E/8점 F/N점 DLT 재맞춤, t 단위화).
- `median_triangulation_angle`, `inlier_rays`.

## 기타
- `linalg::{svd3 (core::linalg 재노출; 단측 야코비, 중복 특이값 정확), null_space_9, hartley_normalize, …}`, `poly::{roots_companion, solve_cubic_monic}`.
- 예제: `cargo run --release -p skyrecon-matching --example bench_match` (8192² 매칭 시간).

## 설계 결정 (코드에 `// 설계 결정` 표시)
- 각도 계산 정밀도: GPU 커널처럼 f32 acos. 경계(θ1=0.7)에서만 의미.
- 실패 RANSAC 의 모델은 기록하지 않음(성공한 E/F/H 만).
- H N점 해의 수치 랭크 허용오차 = max(2N,9)·ε·σ_max.
- H det 임계는 해의 고유 스케일(4점 h33=1, N점 단위 노름)에 적용 → 평행이동 성분이 큰 픽셀 H 의 N점 국소 최적화는 거부될 수 있음(기본 동작). `normalize_homography` 켜도 같은 스케일로 맞춘 뒤 검사.
- 하틀리 정규화 H 는 결과가 미세하게 달라지므로 옵션(기본 끔).
- 다중 모델 모드에서 무시되는 워터마크 기하의 인라이어도 제거 후 계속.
- 짝수 개 삼각측량 각의 중앙값은 가운데 두 값 평균.
- 워터마크 테두리 판정: 내부 상자 경계 포함(b ≤ x ≤ w−b 이면 내부).
- skip_geometric_verification 이면 원시 매칭만 기록(기하 행 없음).
- 영상 자료(카메라·키포인트·기술자) 누락은 처리 전에 `Error::NotFound`.
