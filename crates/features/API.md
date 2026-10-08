# skyrecon-features

영상 읽기(EXIF·디코딩·회색 변환·축소·방향 회전), 카메라 초기 파라미터 결정, 결정적 SIFT 특징 추출,
`FeatureStore` 증분 기록을 맡는 크레이트. SIFT 계산은 `SiftEngine` 트레이트 뒤에 있어 CPU/GPU 백엔드를 바꿔 끼울 수 있다.

## 진입점 (`extract`)
- `FeatureExtractor::new()` (CPU 백엔드) / `with_backend(Arc<dyn SiftEngine>)`. `Clone` 비용 낮음, 피라미드 버퍼는 백엔드 안에서 영상 간 재사용.
- `extract_files(&store, root, &names, &ExtractionOptions) -> Result<Vec<ImageReport>>`: 파일 읽기(EXIF + 디코딩, 병렬) → 카메라·영상 행 배정(순차, 이름 사전순) → SIFT(영상 병렬) → `FeatureStore` 기록.
- `extract_inputs(&store, Vec<ImageSource{name, gray, exif}>, &opts)`: 이미 디코딩한 영상용. 위치 하나 단위로 호출하면 됨.
- 오류: `Existing(N)` 에서 카메라 N 이 없으면 즉시 `Err(NotFound)`. 영상별 실패(디코딩·크기 불일치)는 `ImageStatus::Failed{error}`.
- `ImageStatus::{Extracted{image_id, camera_id, num_features}, AlreadyExists{image_id}, Failed{error}}`. 이름이 있고 키포인트·기술자가 모두 있으면 건너뜀. 행만 있으면 그 카메라로 특징만 채움.
- `ExtractionOptions { reader: ReaderOptions, sift: SiftOptions, sequential_images: bool(false) }`.
- `ReaderOptions { camera_model(OpenCv), camera_mode(PerFolder), camera_params(None), default_focal_length_factor(1.2), max_image_size(3200), strict_existing_camera_size(false), read_pose_priors(true) }`.
- `CameraMode::{PerImage, Single, PerFolder(기본, 위치 0), Existing(CameraId)(위치 p≥1)}`.
- EXIF GPS 가 있으면 `PosePrior{position:(lat,lon,alt), coordinate_system:0, gravity: EXIF 방향의 중력(있으면)}` 기록.
- 보조: `read_image_list(path)`, `image_folder(name)`, `extract_for_camera(backend, &gray, orientation, cam_w, cam_h, max_size, &sift) -> (Vec<Keypoint>, SiftOutput)` (축소 → EXIF 방향 회전 → SIFT → 역회전 → 카메라 크기 환산).

## 영상 (`gray`)
- `GrayImage{width, height, data: Vec<u8>}`: `new, filled, from_f32, get, rotate_ccw(k), resized(w,h)`(Lanczos3).
- `read_gray(path)` (EXIF 자동 회전 없음), `to_gray(&DynamicImage)`, `rgb_to_gray(r,g,b)` = round(0.2126R+0.7152G+0.0722B).
- `limited_size(w,h,M)`, `rotate_keypoint_ccw(&kp, k, w, h)`, `orientation_to_rotation(Option<u32>)`, `orientation_gravity`.

## 카메라 (`exif_info`, `camera_init`)
- `ExifInfo{make, model, focal_mm, focal_35mm, focal_plane_x_resolution, focal_plane_resolution_unit, orientation, gps}`: `read(path)`, `from_bytes`.
- `infer_focal(&exif, W, H, factor) -> FocalEstimate{focal, prior}` (35mm → 초점면 해상도 → 센서 폭 표 → factor·max(W,H)).
- `init_camera(model, W, H, &exif, params, factor) -> Camera` (fx=fy=f, cx=W/2, cy=H/2, 왜곡 0, id 무효 → `store.add_camera` 가 발급).
- `lookup_sensor_width(make, model)`.

## SIFT (`sift`)
- `trait SiftEngine: Send+Sync { fn name(&self)->&str; fn extract(&self, &GrayImage, &SiftOptions) -> Result<SiftOutput> }`. `CpuSift` (rayon).
- `SiftOutput{features: Vec<SiftFeature>, descriptors: Descriptors}`, `keypoints()`. `SiftFeature{x, y, scale, orientation, octave, level, response}` — 입력 영상 화소 단위(화소 중심 .5), `keypoint()` 은 아핀 키포인트.
- 출력 순서: 옥타브 오름차순 → 레벨 오름차순 → 행 우선 검출 순서, 두 번째 방향은 바로 뒤. 완전 결정적.
- `SiftOptions` 기본값: max_num_features 8192(0=무제한), first_octave −1(−1/0 지원), num_octaves None(자동 floor(log2 min)−3), octave_resolution 3, peak 0.006667, edge 10, max_num_orientations 2, upright false, normalization L1Root, truncate_width_to_4 true, refinement_iterations 1, reject_singular_refinement false, orientation_bin_interpolation false, selection CompatLevels.
- 결과를 바꾸는 개선(모두 기본 꺼짐): `truncate_width_to_4=false`, `refinement_iterations=5`(|δ|>0.6 이면 화소 이동), `reject_singular_refinement`, `orientation_bin_interpolation`, `FeatureSelection::TopK`(거친 레벨 우선 + |응답| 순 정확히 K), `FeatureSelection::SpatialGrid{cells}`(격자 라운드로빈 정확히 K), `num_octaves=Some(n)`.
- 결과 불변 최적화: 거친 레벨부터 처리해 잘릴 미세 레벨은 검출·방향·기술자 생략, DoG 를 옥타브 단위 임시 버퍼로, 기울기는 레벨별로 키포인트 수에 따라 전 영상 선계산/즉석 계산 선택(비트 동일), 분리형 블러 벡터화 + 행 병렬, 버퍼 풀.
- 저수준: `sift::pyramid::{gaussian_kernel, gaussian_blur, upsample2, decimate2, ScaleSpace, auto_num_octaves, BufferPool}`, `sift::orient::{orientations, descriptor, normalize_quantize, quantize_angle}`, `sift::detect::detect_level`.

## 설계 결정 (코드에 `// 설계 결정` 표시)
- 축소 필터: Lanczos3.
- 업샘플 마지막 행/열: 가장자리 복제. 옥타브 데시메이트: 내림 크기 (w/2, h/2).
- 방향 16비트 복원 배율 q/65535·2π (저장 배율과 동일).
- 센서 폭 표: 공개 사양의 DJI 카메라 일부만.
- PerFolder: 폴더→카메라 사전으로 묶는다(정렬 목록에서는 "직전 카메라 + 처음 본 폴더" 규칙과 같다).
- `Existing` 모드 크기 불일치: 기본은 오류 없이 카메라 크기로 환산, `strict_existing_camera_size` 로 엄격 모드.
- 0으로 나누기 방지: 히스토그램 전부 0이면 피크 없음 → 버림.

## 테스트 (cargo test --release -p skyrecon-features: 21개 통과)
- 읽기·카메라: 회색 변환 정확값, 축소 크기, 초점(f35 → 2773.284, 초점면 해상도 → 4500, 없음 → 4800/플래그 거짓, 센서 표), OPENCV 초기값, 키포인트 환산, 회전 왕복 + 영상 회전과 점 회전 일치, 카메라 묶기 시나리오(camF=1, camL=2, camR=3, existing 재사용, 재실행 건너뜀, 없는 카메라 오류), 축소 후 입력 좌표 환산, 디스크 목록 읽기(없는 파일 → Failed).
- SIFT: σ/σ_inc/커널 폭 표, 3200×2400 → 9옥타브, 업샘플 규칙, 가우시안 블롭(σ_b=8 → (256.5, 256.5), scale 7.10), 평행이동(검사 1007개 위치·기술자 비트 단위 동일), 90° 회전(869개 대응, 기술자 L2 평균 4.1, 방향 차 π/2 중앙 오차 2e−5), 기술자 노름 512±10(L1Root/L2), 직선 에지 위 0개, 개수 제한(무제한 36361 → compat 13703 = 레벨 통째, 가장 미세한 레벨 빼면 ≤ 8192; TopK/Grid 정확히 8192), 개선 옵션 동작, 결정성, 실사(있을 때만; sim_map_view.jpg 1568×759 → 2808개).

## 성능 (Apple M5, 10 스레드, release)
- 2048×1152 합성 텍스처: 기본(compat 8192) 76 ms/장(첫 실행 95 ms), 무제한 66k 특징 261 ms.
- 2048×1152 실사(sim_map_view.jpg 를 확대): 6680 특징 69 ms/장.
- 측정: `cargo run --release -p skyrecon-features --example bench_sift -- <영상|synthetic> 2048 1152`.

## 참고
- `FeatureStore` 에는 rig/frame 행이 없다. 재구성 단계에서 `Reconstruction::add_image_own_frame` 으로 영상마다 프레임을 만든다.
