# skyrecon-dense 공개 API 요약

## 왜곡 보정 (`undistort`, `image`)
- `undistorted_camera`, `CameraUndistortion`, `UndistortCache`, `undistort_reconstruction`, `undistort`, `undistort_from_dir`,
  `write_undistorted_workspace`, `UndistortOptions`, `UndistortResult`.
- `ImageBuffer`(u8 1/3채널), `GrayImage`(f32), 면적 평균·3차 재표본.

## 장면 (`scene`)
- `DenseScene { views: Vec<DenseView>, points: Vec<ScenePoint> }`
  - `DenseScene::from_reconstruction(&Reconstruction, &BTreeMap<ImageId, Arc<ImageBuffer>>, &SceneOptions)`: 등록 순서, 핀홀 카메라만.
  - `DenseScene::from_workspace_dir(dir, &SceneOptions)`: `sparse/` + `images/`.
- `DenseView { image_id, name, width, height, k: [fx, fy, cx, cy] (정수 중심), r, t (세계→카메라), gray, color }`
  와 `center`, `to_cam`, `to_world`, `dir_to_world`, `dir_to_cam`, `project_cam`, `ray`, `geometry_hash`.
- `SceneOptions { max_image_size }`.

## 설정 (`params`)
- `DensifyOptions { profile, neighbors, pm, filter, fusion, post, max_levels, min_level_size, jbu_sigma_spatial, jbu_sigma_color, restorer_threshold, seed }`
  - `DensifyOptions::with_profile(MvsProfile::{Fast, Quality})`, `.schedule(level, top) -> LevelSchedule`, `.fingerprint()`.
- `PmParams`(창 반경·간격, 양방향 σ, 투표 상수 τ0/α/τ1/β/n1/n2, 사전확률, 기하 λ/δ, 정제 섭동, `weak_texture_var`).
- `FilterParams`(min_ncc, 필터 삼각측량각, 최소 일치 뷰, 재투영 허용, 방출 σ, 중앙값 필터).
- `FusionParams { mode: FusionMode::{Consistency, Traversal}, min_consistent_views, mark_used, inverse_variance, sigma_px0, sigma_px_slope, consistency_num_images, min/max_num_pixels, max_traversal_depth, max_reproj_error, max_depth_error, max_normal_error_deg, check_num_images }`.
- `NeighborParams { num_views (≤ 32), min_triangulation_angle_deg, direction_bins, diversity_decay }`.
- `PostParams`(작은 조각 제거, 경계 인식 틈 메우기).

## 백엔드 경계 (`kernel`)
- `trait PatchMatchBackend: Send + Sync { fn name(&self) -> String; fn begin(&self, &KernelInput) -> Result<Box<dyn PatchMatchSession>> }`
- `trait PatchMatchSession { run(views, states, &RunParams, Option<&DepthSnapshot>); evaluate(level, views, states, geometric, snapshot) -> 상위 K 평균 비용; filter(views, states, &DepthSnapshot) -> 픽셀별 일치 뷰 수 }`
- 자료: `KernelInput { views: Vec<KernelView>, num_levels, pm, filter, seed }`, `KernelView { key, levels: Vec<LevelImage>, sources, pairs: Vec<PairGeometry>, depth_min, depth_max }`,
  `ViewState { width, height, depth, normal, cost, prior }`, `RunParams { level, geometric, random_init, iterations, run_id, use_prior }`,
  `DepthSnapshot { id, level, maps }`. 커널이 지킬 비용·투표·정제·난수 규칙은 `kernel` 모듈 문서에 있다.
- `rng_state`, `rng_next`: 커널과 같은 픽셀 난수 정의.

## 진행 (`densify`)
- `compute_depth_maps(&scene, &opts, &dyn PatchMatchBackend, Option<&DepthMapCache>) -> Result<DepthMapSet>`
- `fuse_depth_maps(&scene, &DepthMapSet, &opts, threads) -> FusionOutput`
- `densify(...) -> Result<DenseOutput>`: 위 둘. `DenseOutput { cloud, visibility, timings, cache_hits, depth_views, depth_maps }`, `.write_ply(path)`.
- `DepthMapSet { maps: Vec<Option<Arc<DepthMapResult>>>, sources, baselines, cache_hits, num_levels, timings }`,
  `DepthMapResult { width, height, raw_depth, depth, normal, cost, from_cache }`.
- `DenseTimings { neighbors, prepare, levels: Vec<Duration>, upsample, filter, fusion }`, `.depth()`, `.total()`.
- `pair_geometry(&scene, ref, src) -> PairGeometry`.

## 기타
- `neighbors::{PairStats, depth_ranges, depth_range_of}`; `PairStats::{new, select, select_diverse, select_all}`.
- `fusion::{fuse, fuse_consistency, fuse_traversal, FusionInput, FusionOutput}`.
- `upsample::{num_levels, build_pyramid, joint_bilateral_upsample, median_plane_filter, downsample_depth}`.
- `postproc::{remove_speckles, fill_holes, PostParams}`.
- `stats::{cloud_stats, CloudStats}`: 이웃 점 간격 중앙값, GSD, 이상점 비율(겹침 뷰 깊이맵 중 2 px 안에서 지지하는 뷰가 2개 미만), 중복률.
- `cache::{DepthMapCache, CachedDepth}`.
- `math`: 결정적 해시, `erf`, `emission`, `emission_norm`, `visibility_probability`, `triangulation_prior`, `incident_prior`,
  `resolution_prior`, `quantile_sorted`, `median_in_place`, `plane_transfer`.
- `synthetic::{make_scene, SynthConfig, SynthScene, SynthBox}`: 시험용 레이 캐스팅 장면과 참 깊이·법선.
