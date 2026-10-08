# skyrecon-ba

번들 조정(BA)과 단일 자세 정제. 신뢰 영역 Levenberg–Marquardt 위에 점 Schur 소거, 밀집/희소 촐레스키·PCG
축소 계통 풀이기를 얹었다. 결과는 스레드 수와 무관하게 결정적이다.

## 함수
- `bundle_adjust(&mut Reconstruction, &BaConfig) -> Result<BaSummary>`
  - 대상 = `config.images`(비면 등록 영상 전부, id 오름차순). 관측 없는 영상·카메라는 제외.
  - `filter_negative_depth`(기본 켬)면 먼저 깊이 < ε 관측 삭제(트랙 ≤ 2 면 점째 삭제).
  - 트랙 일부가 대상 밖인 점, `constant_points`, `refine_points=false` → 점 상수.
  - 결과 써 넣기: 대상 프레임 자세 전부(쿼터니언 사전 정규화 반영), 가변 카메라 파라미터, 가변 점.
    `update_point_errors`(기본 켬)면 `rec.update_point3d_errors()`.
  - 풀이기 `Failure` 여도 마지막 채택 해를 써 넣고 `Ok` (단독 명령에서도 그대로 저장). `summary.is_usable()` 로 확인.
- `refine_abs_pose(cam, pts2d, pts3d, mask, &mut world_to_cam, loss, max_iter) -> Result<BaSummary>`
  - 자세 6자유도만, 점·내부 고정. 허용치: 기울기 1.0, 함수 1e−6, 파라미터 1e−8.
  - 길이 불일치·인라이어 0 → `Err(InvalidArgument)`. 풀이기 실패 → `Err(Invariant)`, 자세 그대로(= 등록 실패로 처리).
- `select_linear_solver(num_images, &cfg)`: ≤50 밀집, ≤1000 희소, 그 외 반복.
- `filter_negative_depth_observations(&mut rec, &[ImageId]) -> Result<usize>`.

## BaConfig (Default = 단독 번들 조정 기본값)
| 필드 | 기본 |
|---|---|
| refine_focal_length / principal_point / extra_params | true / **false** / true |
| refine_poses / refine_points | true / true |
| loss | `Trivial` (`SoftL1(a)`, `Cauchy(a)`, `Huber(a)`, a = 픽셀) |
| max_num_iterations | 100 |
| function / gradient / parameter_tolerance | 0 / 1e−4 / 0 |
| images, constant_poses(ImageId → 그 프레임 고정), constant_cameras, constant_points | 빈 집합 |
| auto_gauge | true ("두 영상 고정" 규칙, 상수 프레임 ≥ 2 면 생략, 실패 시 "세 점" 대체) |
| num_threads | 0 (= rayon 전역 풀, >0 이면 전용 풀) |
| (추가) constant_world_to_rig_rotation | false |
| (추가) min_track_length | 0 |
| (추가) linear_solver | `Auto` (`DenseSchur`/`SparseSchur`/`IterativeSchur` 강제 가능) |
| (추가) max_linear_solver_iterations | 200 (PCG) |
| (추가) dense_solver_image_limit / sparse | 50 / 1000 |
| (추가) filter_negative_depth / update_point_errors | true / true |

점 삼각측량 후 전역 BA 예: `refine_poses=false, refine_focal_length=false, refine_extra_params=false,
max_num_iterations=50, gradient_tolerance=1.0, max_linear_solver_iterations=100` (자세·내부가 전부 상수면 축소 계통이 비어 점별 3×3 풀이만 남음).
증분 국소 BA(Soft-L1 a=1)는 `loss: Loss::SoftL1(1.0)`.

## BaSummary
`num_iterations, initial_cost, final_cost (½Σρ), num_residuals (2×관측), converged (= Convergence)`,
추가: `termination {Convergence, NoConvergence, Failure}, num_successful_steps, linear_solver, num_images,
num_points, free_point_count, dropped_observations`, `is_usable()`, `rms_reprojection_error()`.

## 구현 요약
- LM 신뢰 영역: 초기 반경 1e4, 최대 1e16, 최소 1e−32, 성공 시 r /= max(1/3, 1−(2ρ−1)³), 실패 시 r /= f, f *= 2,
  채택 기준 ρ > 1e−3, 연속 무효 단계 10 초과 → Failure. 감쇠 = clamp(diag(JᵀJ), 1e−6, 1e32)/r.
  야코비 열 배율 1/(1+‖열‖) 은 첫 평가에서 한 번. 기울기 기준 ‖x − ⊞(x, −g)‖∞ (쿼터니언은 주변 4성분).
- 자세 = 쿼터니언+이동, 접공간 (δ회전 3, δ이동 3), q ← Exp(δ)⊗q. 고정 성분은 야코비안 열 0 + 축소 계통 대각 1 → 정확히 불변(비트 단위).
- 견고 손실: 모든 손실이 ρ'' ≤ 0 이라 트릭스 보정이 √ρ' 배율로 줄어듦. 투영 실패(Z<ε) → 잔차·야코비안 0.
- Schur: 점 3×3 소거, E 블록 = [가변 자세 6…, 가변 카메라 n…]. 조립은 점 우선(관측 연속 접근)으로 고정 개수 조각별 S 사본에
  누적 후 조각 순서로 합산(결정적, 스레드 수 무관; S 값이 64MB 를 넘으면 행 병렬 행 우선 조립으로 대체).
  풀이: 밀집 촐레스키(nalgebra), 희소 촐레스키(faer, 기호 분해 1회 재사용), PCG(Schur 대각 블록 전처리, 나시–소퍼 η=0.1).
  역대입·모형 비용 변화(−rᵀJδ − ½‖Jδ‖²)·후보 비용 모두 병렬, 합은 고정 조각 순서.

## 설계 결정 (코드에 `설계 결정` 주석)
- 쿼터니언 다양체 ⊞ 를 관례 Exp(δ) = [cos‖δ‖, sin‖δ‖·δ/‖δ‖] (실제 회전각 2‖δ‖) 로 둠 → 회전 야코비안 = −2[RP]×.
- 함수 허용치 도달 시 개선된 단계는 채택 후 종료. 모형 감소 < 0 은 무효 단계, = 0 은 실패 단계로 처리.
- 비자명 rig: 프레임 자세를 변수, rig_to_sensor 상수(센서 상대 자세 정제 미구현). 게이지 영상1·2 는 기준 센서 영상만.
- refine_abs_pose 는 6×6 정규방정식 촐레스키(정확 산술에서 밀집 QR 과 동일).
- 함수/파라미터 허용치(자세 정제) = 1e−6/1e−8.

## 테스트 (`cargo test --release -p skyrecon-ba`, 15개)
- 단위: 해석 야코비안 vs 중앙 차분(자세 회전·이동·점·OPENCV 6 파라미터, rig 센서 포함, 상대 1e−6), 투영 실패 → 정확히 0,
  ⊞ 관례, 손실 도함수, 점 우선 vs 행 우선 Schur 일치(1e−12).
- 통합(합성: 위치 × OPENCV 3대, 롤 ±25°, σ=0.5 px, 섭동 0.5°·0.5 m·점 0.5 m·초점 2%):
  - 48장·점 1만(기준 장면): RMS/축 0.465 px(이론 ML 값 0.5·√(1−P/M) 와 일치), 영상1 자세 비트 동일, 영상2 고정 이동 성분 비트 동일,
    cx·cy 비트 동일, 정답 대비 최대 회전 0.031°, RMS 중심 3.2 cm, 정답에서 출발한 ML 해와 2e−6° / 2e−10 m 일치.
  - 세 풀이기: 최종 비용 상대 1e−8 내 일치, 점 차 직접 풀이기 간 < 1e−6 m, PCG < 1e−3 m.
  - 이상치 5%(20–60 px): Cauchy/Huber/SoftL1 의 정상 관측 RMS ≈ 0.46–0.49 px vs 무손실 2.3 px, 중심 오차 ¼ 이하.
  - 고정 플래그(자세·카메라·점·전부 고정), 창 BA(밖 자세·공유 점 불변), 점만 BA, 사전 필터, 단일/다중 스레드 비트 동일,
    refine_abs_pose(Cauchy, 마스크·비마스크 이상치) 회전 0.02°·중심 3 mm, 입력 오류.

## 성능 (`cargo run --release -p skyrecon-ba --example ba_bench [위치수 점수 풀이기]`, M 계열 10코어, 다른 작업과 동시 실행)
| 장면 | 관측 | 풀이기 | 반복 | BA 1회 | 반복당 |
|---|---|---|---|---|---|
| 240장 · 20만 점 | 177만 (트랙 8.9) | 희소(자동) | 56 | **5.6 s** | 0.10 s |
| 〃 | 〃 | 밀집(강제) | 69 | 12.2 s | 0.18 s |
| 〃 | 〃 | PCG(강제) | 100(상한) | 13.4 s | 0.13 s |
| 48장 · 1만 점 | 21.8만 | 밀집(자동) | 19 | 0.12 s | 6 ms |
반복당 내역(희소): Schur 조립 ~50 ms, 야코비안·정규 블록 ~30 ms, 희소 촐레스키 ~3 ms, 역대입·후보 비용 ~15 ms.
메모리: 관측당 약 264 B(보정 야코비안 저장) → 177만 관측에서 ~0.47 GB (벤치 전체 RSS ~1 GB, 재구성 사본 2개 포함).

