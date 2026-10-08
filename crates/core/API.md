# skyrecon-core

skyrecon 의 공통 기반 크레이트. 카메라 모델, 기하(쿼터니언·강체·상사 변환), 특징·매칭 자료형,
메모리 특징 저장소, 대응 그래프, 희소 재구성 자료 구조, 범용 RANSAC, 파일 입출력을 제공한다.
다른 모든 크레이트가 이 크레이트의 자료형을 공유한다.

## 규약
- 수치 f64 (키포인트만 f32). 벡터/행렬은 nalgebra: `Vec2, Vec3, Mat3, Mat3x4` (`geometry` 에서 재수출).
- id: `CameraId/RigId/FrameId/ImageId/Point2DIdx = u32`, `Point3DId/PairId = u64`. 무효값 = 최댓값
  (`INVALID_CAMERA_ID`, `INVALID_POINT3D_ID`, …). 3D 점 id 0 도 유효. 영상 id < 2^31−1.
- 자세: `Rigid3` = A_to_B, `X_B = R X_A + t`. 영상 자세는 **world_to_cam**. 중심 `C = −Rᵀt` (`Rigid3::center`).
  `a * b` = a ∘ b (A_to_C = B_to_C * A_to_B). 합성 결과 회전은 정규화.
- 쿼터니언 `Quat {w,x,y,z}`: 해밀턴 규약, 파일 순서 w,x,y,z.
- 카메라 좌표: x 오른쪽, y 아래, z 앞. 픽셀: 원점 좌상단 모서리, **좌상단 화소 중심 = (0.5, 0.5)**. 키포인트·투영 동일.
- `Sim3` new_from_old: `X' = s R X + t`; 자세에는 `transform_pose` (R' = R Rsᵀ, t' = s t − R' t_s).
- 잔차는 RANSAC 에서 **제곱** 오차, 인라이어 = 잔차 ≤ max_error².

## geometry
- `Quat::{IDENTITY,new,from_wxyz,to_wxyz,normalized,conjugate,inverse,hamilton,to_rotation_matrix,from_rotation_matrix,from_axis_angle,from_rotation_vector,to_rotation_vector,rotate,angular_distance,to_nalgebra,from_nalgebra}`, `Quat * Quat`.
- `Rigid3 {rotation, translation}`: `identity, new, from_rotation_matrix(&R,t), from_params([qw,qx,qy,qz,tx,ty,tz]), to_params, rotation_matrix, matrix()->Mat3x4, from_matrix, transform_point, inverse, compose, center, viewing_direction`; `Rigid3 * Rigid3`, `Rigid3 * Vec3`.
- `Sim3 {scale, rotation, translation}`: `identity, new, matrix, from_matrix([sR|t]; s=첫 열 노름), transform_point, inverse, compose, transform_pose`.
- `skew(&v)`, `deg_to_rad`, `rad_to_deg`, `angle_between(&a,&b)`, `triangulation_angle(&c1,&c2,&X)` (min(θ,π−θ), 노름 0→0), `max_triangulation_angle(&[C], &X)`, `baseline`.

## camera
- `CameraModelKind::{SingleFocalPinhole=0, Pinhole=1, SingleFocalRadial=2, Radial=3, OpenCv=4}`: `from_id, id, name, from_name, num_params, params_info, focal_slots, pp_slots, extra_param_slots`. id 5..17 은 `Error::Unsupported`.
- `Camera {camera_id, model, width:u64, height:u64, params:Vec<f64>, focal_from_prior}`
  - `new(id, model, w, h, params)->Result`, `from_focal(model, f, w, h)` (cx=W/2, cy=H/2, 왜곡 0, id 무효)
  - `mean_focal, focal_length_x/y, set_focal_length, principal_point_x/y, set_principal_point, extra_params, is_undistorted, calibration_matrix, has_implausible_params(min_ratio,max_ratio,max_extra)`
  - `cam_to_img(&Xc)->Option<Vec2>` (Z<ε → None), `normalized_to_img(&uv)->Vec2`,
    `img_to_normalized(&xy)->Option<Vec2>` (뉴턴 100회·신뢰영역·‖step‖²<1e−10, 실패 None), `img_to_ray(&xy)->Option<Vec3>` (단위)
  - `img_to_cam_threshold(px) = px / mean_focal`
  - `rescale(s)`, `rescale_to(w', h')` (fx·sx, fy·sy, cx·sx, cy·sy)
  - `cam_to_img_with_jacobian(&Xc, Option<&mut [f64]>) -> Option<(Vec2, Matrix2x3)>`: ∂픽셀/∂Xc 와
    (선택) ∂픽셀/∂params 를 **행 우선 2×num_params** 슬라이스에 채움(해석식).
    자세 야코비안은 체인: ∂Xc/∂P = R, ∂Xc/∂t = I, 왼쪽 섭동 회전 ∂Xc/∂δ = −[R P]×.

## features
- `Keypoint {x,y,a11,a12,a21,a22: f32}`: `new, from_scale_orientation, from_row(2/4/6열), to_row, scale, scale_x, scale_y, orientation, shear, rescale(sx,sy)`.
- `Descriptors` (N×128 u8): `new, with_capacity, from_vec, len, row(i), row_mut, push(&[u8;128]), as_slice, truncate`. `DESCRIPTOR_DIM=128`.
- `FeatureMatch {idx1, idx2: u32}`, `swapped()`.
- `TwoViewGeometryConfig::{Undefined=0, Degenerate, Calibrated, Uncalibrated, Planar, Panoramic, PlanarOrRotation, Watermark, Multiple, CalibratedRig=9}` (`from_i32, as_i32`).
- `TwoViewGeometry {config, e, f, h: Option<Mat3>, cam1_to_cam2: Option<Rigid3>, inlier_matches, tri_angle: Option<f64>}`: `inverted()/invert()` (Fᵀ, Eᵀ, H⁻¹, 자세 역, 매칭 교환).

## ids
- `pair_id_of(a,b)->Result<PairId>` (M·min+max, M=2^31−1), `images_of_pair(id)->(min,max)`, `swap_image_pair(a,b)` (a>b).
- `SensorKind::{Invalid=-1, Camera=0, Imu=1}`, `SensorKey{sensor_type,id}` (`camera(id)`), `SensorDataKey{sensor_id,id:u64}` (`image(cam,img)`). 모두 사전식 Ord.

## store::FeatureStore (스레드 안전, `&self` 메서드)
- 카메라: `add_camera(cam)->Result<id>` (id 무효면 최대+1, 빈 저장소 1), `update_camera, camera, cameras, num_cameras`.
- 영상: `add_image(name, cam)->Result<id>` (최대+1, 이름 고유), `add_image_with_id, image, image_by_name, image_id_by_name, images, image_ids, num_images, exists_image`; `set_pose_prior/pose_prior` (`PosePrior{position:(lat,lon,alt), coordinate_system, gravity}`).
- 특징: `set_keypoints(id, Vec<Keypoint>)`, `set_descriptors`, `keypoints(id)->Option<Arc<Vec<Keypoint>>>`, `descriptors(id)->Option<Arc<Descriptors>>`, `num_keypoints, largest_keypoint_count, exists_*`.
- 짝(어느 방향이든 호출 가능, 내부는 작은 id→큰 id): `write_matches/read_matches/exists_matches/delete_matches`,
  `put_two_view/get_two_view/contains_two_view/remove_two_view`,
  `two_view_geometries()` (저장 방향), `two_view_geometries_since(cursor)->(Vec, new_cursor)` (증분).
- `save(path)`, `load(path)` (자체 이진 형식 "SKYFS\0v1").

## graph::MatchGraph
- `from_store(&store, &MatchGraphOptions{min_num_matches:15, ignore_watermarks:false, image_names, keep_all_images:false})`
- `update_from_store(&store, &opts)->usize` : 새로 기록된 짝만 증분 추가(재구성 불필요).
- `add_image(id, num_points2d)`, `insert_two_view(id1,id2,&tvg)->AddPairStats` (자기 짝·범위 밖·중복 규칙).
- 조회: `find_correspondences(img, idx)->&[Correspondence]`, `matches_between(i1,i2)->Vec<FeatureMatch>`,
  `transitive_matches(img, idx, depth)`, `in_two_view_track`, `has_correspondences`,
  `observation_count_of`, `match_count_of`, `match_count_between`,
  `two_view_geometry(i1,i2)` (방향 맞춰 역변환), `replace_two_view`, `image_pairs()`, `image_ids()`, `num_points2d`.
- `union_find_tracks(|small,large| bool) -> Vec<Vec<Correspondence>>` (연결 성분, 결정적 순서; 일관성 검사는 호출자).

## reconstruction::Reconstruction (Clone 저렴: 영상·점은 Arc copy-on-write)
- 자료형: `Rig{rig_id, ref_sensor_id, sensors: BTreeMap<SensorKey, Option<Rigid3>>}` (`trivial(cam)`, `rig_to_sensor`),
  `Frame{frame_id, rig_id, world_to_rig}` (`new, attach_data, data_ids, image_ids, has_pose`),
  `Image{image_id, name, camera_id, frame_id}` (`new(id,name,cam,points)`, `points2d, point2d, num_points2d, num_points3d`),
  `Point2D{xy, point3d_id}`, `Point3D{xyz, color:[u8;3], error(−1=없음), track: Vec<TrackEntry>}`, `TrackEntry{image_id, point2d_idx}`.
- 구성: `add_camera`, `add_camera_own_rig`, `add_rig`, `add_frame`, `add_image`,
  `add_image_own_frame(image, Option<world_to_cam>)` (프레임 id = 영상 id, rig id = 카메라 id).
- 자세/등록: `world_to_cam(img)`, `projection_center`, `has_pose`, `set_world_to_cam`, `set_frame_pose`,
  `register_frame/register_image -> Result<bool>`, `deregister_frame/deregister_image` (관측 삭제 규칙 포함),
  `is_image_registered`, `registered_frames()` (등록 순서), `registered_images()`, `registered_image_count`, `registered_frame_count`.
- 점: `add_point3d(xyz, track, color)->id` (최대 id+1, 빈 모델에서 1), `add_point3d_with_id`, `add_observation`,
  `delete_observation(img, idx)->bool(점 삭제됨)`, `delete_point3d`, `merge_points3d(a,b)->new id`,
  `set_point3d_xyz/error/color`, `delete_all_points3d`, `point3d, points3d(), point3d_ids`.
  카메라 수정: `camera_mut`. 영상·점 직접 가변 참조는 없음(불변식 보호).
- 오차/필터: `squared_reprojection_error(&pose,&cam,&xy,&X)` (실패 f64::MAX), `update_point3d_errors()`, `point3d_reprojection_error`,
  `filter_observations_with_large_reprojection_error(px, Option<&[ids]>)`, `filter_points3d_with_small_triangulation_angle(deg, ids)`,
  범용 `filter_observations(ids, max_err, |cam,pose,xy,X| err, FilterErrorUpdate)`.
- 통계: `total_observations, mean_track_len, mean_obs_per_registered_image, mean_reproj_error`.
- 기타: `transform(&Sim3)`, `normalize(&NormalizeOptions)->Option<Sim3>`, `deregister_images_by_id/by_name -> 경고 Vec<String>` (image_deleter),
  `remove_unregistered()`, `extract_colors(|img| Option<sampler(x−0.5,y−0.5)>)`, `bilinear_rgb(...)`, `check_invariants()`.

## analyzer
- `ModelStats::compute(&rec)`, `.lines()` (고정 11줄 요약), `analyzer_lines(&rec, verbose)`.

## interop (외부 도구와 주고받는 파일 형식)
- `read_model(dir)` (bin 우선, 없으면 txt), `read_model_binary`, `read_model_text`,
  `write_model_binary(&rec, dir, ImageOrder::{Registration(기본)|ById})`, `write_model_text(...)` (%.17g).
  쓰기: 카메라·rig 전부, 자세 있는 프레임, **등록 영상만**, 점 전부. rigs/frames 파일이 없는 구버전 모델 읽기 지원.
- 조밀 복원 작업 폴더: `STEREO_SUBDIRS`, `create_stereo_dirs(out, rel)`, `write_stereo_configs(out, names, num_src)`
  (`stereo/patch-match.cfg`, `stereo/fusion.cfg`).

## io
- `read_ply(path)->PointCloud{positions:[f32;3], normals, colors:[u8;3]}` (속성 이름 기반, ascii/LE/BE),
  `write_ply(path, &cloud, PlyLayout::{XyzRgbNormal (float32/uint8, 27바이트/점) | XyzNormalRgb (float/uchar)})`.
- `read_gps_file(path)->Vec<GpsRecord{name,lat,lon,alt}>` (순서 유지, 첫 줄 = ENU 원점).
- `io::fmt::{format_g, g17}` (printf `%g` 와 같은 로캘 무관 서식).

## linalg
- `svd3(&Mat3) -> Option<(U, σ, V)>`: 단측 야코비 3×3 SVD(σ 내림차순, det U = +1, 겹친 특이값에서도 정확). nalgebra 0.35 의 3×3 SVD 가 특이값이 겹칠 때 틀리므로 matching(재노출)·sfm·align(umeyama) 이 이것을 쓴다.

## ransac
- `trait Estimator { type X; type Y; type Model; fn min_num_samples(&self); fn estimate(&self,x,y,&mut Vec<Model>); fn residuals(&self,x,y,&Model,&mut Vec<f64>) }` (잔차 = 제곱).
- `RansacParams {max_error, min_inlier_ratio(0.1), confidence(0.99), dyn_trials_factor(3), min_trials(0), max_trials(i32::MAX), random_seed: Option<u64>}`.
- `ransac(&est, &opts, x, y)`, `lo_ransac(&est, &local_est, &opts, x, y)`, `ransac_with_sampler(&est, Option<&local>, sampler, ...)`
  → `RansacReport{success, num_trials, support: Support{num_inliers, residual_sum}, inlier_mask, model}`.
- `Sampler` trait, `RandomSubsetSampler`(부분 피셔-예이츠 연속), `ExhaustiveSampler`(전수 조합), `compute_num_trials`, `static_max_num_trials`, `make_rng(seed)`, `n_choose_k`.

## error
- `Error::{Io, Parse{path,line,msg}, Format, InvalidArgument, NotFound, AlreadyExists, Invariant, Unsupported}`, `Result<T>`.
