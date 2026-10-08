# skyrecon-align

좌표 정렬과 점군 후처리: GPS(WGS84) → ENU 변환, 재구성을 GPS ENU 에 맞추는 강건 Sim3 정렬,
두 재구성의 공유 3D 점 정렬, 구역 좌표계 연쇄(reanchor), 점군 마스킹·추출·스냅샷 합성. 모든 Sim3 는 core 규약 `new_from_old` (`X' = sRX + t`).
의존성: core, nalgebra, rayon, thiserror (+ dev: rand, rand_pcg).

## error
- `AlignError::{Core, BadMaxError, InvalidArgument, TooFewReferences{found,required}, TooFewCommonImages{found,required}, TooFewCorrespondences{found,required}, RansacFailed{inliers}, Degenerate}`, `Result<T>`.

## geodesy
- 상수 `WGS84_A, WGS84_F, WGS84_B, WGS84_E2`.
- `lla_to_ecef(lat°, lon°, h) -> Vec3`, `ecef_to_lla(&Vec3) -> (lat°, lon°, h)` (반복법, 100회/1e−12; 극점은 닫힌 해).
- `EnuFrame::new(lat°, lon°, h)` (원점 위경도를 ECEF 에서 재계산해 회전 구성), 필드 `origin_lla, origin_ecef, rotation(행 E,N,U)`;
  `ecef_to_enu, enu_to_ecef, lla_to_enu, enu_to_lla`.

## umeyama
- `umeyama(src, dst, estimate_scale) -> Option<Sim3>` (src_to_dst, SVD = `core::linalg::svd3`, 반사 보정, `[sR|t]`→`Sim3::from_matrix`).
- `RankCheck::{Uncentered(기본), Centered(공선 배치만 거부), None}`.
- `Sim3Estimator{rank_check}`: core `Estimator` (최소 3쌍, 잔차 ‖y − Tx‖²).
- `estimate_sim3_ransac(src, dst, &RansacParams, RankCheck) -> RansacReport<Sim3>` (core `lo_ransac`, 국소 = 같은 해법).
- `robust_umeyama(src, dst, &RobustUmeyamaOptions{iterations:5, factor:3, min_threshold:0.3, estimate_scale:true}) -> Option<RobustUmeyamaResult{sim3, median_residual, num_inliers, inlier_mask, residuals}>`.

## model_aligner
- `EnuOrigin::{FirstRecord(기본: GPS 목록 첫 줄), Explicit{lat,lon,alt}}`.
- `ModelAlignerOptions{max_error:3.0, min_common_images:3, origin, ransac: RansacParams(기본; max_error 필드는 무시), rank_check: Uncentered}`.
- `gps_to_enu(&[GpsRecord], EnuOrigin) -> Result<(EnuFrame, Vec<(name, Vec3)>)>`.
- `estimate_gps_alignment(&rec, &gps, &opts) -> Result<GpsAlignment>` (모델 불변).
- `align_to_gps(&mut rec, &gps, &opts) -> Result<GpsAlignment>` (성공 시 `rec.transform(sim3)`; 실패 시 모델 불변 + Err).
- `align_to_gps_file(&mut rec, path, &opts)`.
- `GpsAlignment{sim3 (model_to_enu), enu, common: Vec<(ImageId, name, enu)>, inlier_mask, num_inliers, num_trials, errors: Vec<(name, m)>, mean_error, median_error}`.
- 실패: max_error ≤ 0 / 기준 < 3 / 공통 영상 < max(3, min_common_images) / RANSAC 실패·인라이어 < 3.

## shared
- `shared_point_correspondences(&a, &b, &SharedPointOptions) -> Vec<(Point3DId_a, Point3DId_b)>` (같은 이름 영상·같은 2D 인덱스, 정렬·중복 제거).
- `SharedPointOptions{position_range: Option<Range<u32>>, name_filter: Option<Arc<dyn Fn(&str)->bool>>}`;
  `position_index_from_name("camF/camF_0012.jpg") == Some(12)` (파일 stem 의 마지막 숫자열).
- `align_reconstructions(&src, &dst, &shared_opts, &robust_opts) -> Result<(RobustUmeyamaResult /*src_to_dst*/, pairs)>`.

## kdtree::KdTree
- `new(iter [f64;3])`, `from_f32(&[[f32;3]])`, `len, is_empty`,
  `nearest(&q) -> Option<(orig_idx, dist²)>`, `any_within(&q, r)` (≤ r), `within_radius(&q, r) -> Vec<idx>`,
  병렬: `nearest_many(&[q])`, `any_within_many_f32(&[[f32;3]], r) -> Vec<bool>`.

## cloud (core `PointCloud`)
- `transform_cloud(&mut cloud, &Sim3)` (점 sRx+t, 법선 Rn), `select(&cloud, &mask)`,
  `remove_near(&cloud, &tree, r)`, `decimate(&cloud, stride)` (인덱스 0, N, 2N…),
  `merge_clouds(&[&cloud])` (없는 법선 0, 없는 색 흰색으로 채움),
  `compose_snapshot(&[fine], &[coarse], &SnapshotOptions{mask_radius:1.5, stride:6})`.

## reanchor::AnchorChain
- `push(Option<new_from_prev>) -> zone` (Some 이면 이전 구역 전부 앞에 합성, 새 구역 항등; None 이면 이전 유지),
  `push_model(Option<&prev_rec>, &new_rec, &shared, &robust) -> Result<Option<RobustUmeyamaResult>>` (실패해도 구역은 추가),
  `zone_to_latest(k) -> Option<Sim3>`, `is_linked(k)`, `len`.

## 설계 결정 (`// 설계 결정` 주석)
- 계수 판정 임계: σ > σ_max·ε·max(3, n).
- 중앙값: 짝수 개면 가운데 둘의 평균.
- 견고 Umeyama: 반환 Sim3 = 마지막 적합, 인라이어·중앙값 = 마지막 갱신 keep 기준; keep 이 3 미만이 되면 직전 적합 유지 후 중단. 잔차는 거리(제곱 아님), 판정 `r < thr` (엄격).
- 공유 점: 두 영상의 2D 점 수가 다르면 공통 인덱스 구간만 사용.
- 스냅샷: 1/N 추출은 마스킹·합성 후 전체에 한 번.
- ecef_to_lla 극점(ρ≈0): 닫힌 해.
- 공통 영상 판정은 "자세 있음"(`projection_center` 가 Some), 등록 여부는 보지 않음.
- 정렬 오차 로그는 파일의 중복 줄도 각각 셈.

## 테스트 (24개, `cargo test --release -p skyrecon-align`)
geodesy: 상수, LLA→ECEF 3점, ENU(원점=0, 37.5001/127.0001→(8.8426,11.0988,10.0)), 동쪽 1° 정확해, ECEF↔LLA 왕복 1000점, ENU 왕복.
umeyama: 무잡음 10쌍 50회(1e−9), 반사 → det +1, 계수 검사 3방식, 이상치 30% 복원(LO-RANSAC 인라이어 마스크 정확 일치 + 견고 Umeyama), 중앙값.
model_aligner: 합성 30장 정렬(중심 1e−3 m, 관측 재투영 1e−8 px 불변, C'=sRC+t 1e−9), 줄 순서 → 원점 이동 + 명시 원점, 50장 중 30% 20 m+ 오염·σ=1 m(오염 전부 아웃라이어, 중앙 오차 < 1.5 m), 실패 경계(공통 2장, 기준 2줄, max_error 0 → Err, 모델 불변).
shared: 이름→위치 인덱스, 대응·위치 범위 필터·이상치 포함 정렬, 중복 제거.
kdtree: 무작위 1만 점 × 2000 질의 최근접/반경 무차별 대입 일치, 중복점·빈 트리.
cloud: 1.5 m 마스킹 경계(1.4/1.5 제거, 1.6 유지), 점·법선 변환, 1/6 추출, 스냅샷 합성.
reanchor: 4구역 연쇄가 모든 구역을 최신 좌표계로 사상, 연결 실패 처리.
