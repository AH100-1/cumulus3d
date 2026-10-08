# skyrecon-cuda 공개 API 요약 (GPU 백엔드)

## 규약
- cudarc 0.19(`driver` + `nvrtc`, `dynamic-loading`, `cuda-12040` 바인딩). CUDA C 커널 문자열을 실행 시 NVRTC 로 컴파일한다(장치당 한 번, 0.9~1.5 s).
- CUDA 없는 기계(맥)에서도 빌드·테스트된다. 장치가 없으면 `is_available()` 이 거짓이고 GPU 테스트는 건너뛴다.
- 매칭·SIFT 는 장치 오류가 나면 경고를 내고 CPU 로 대신 계산한다. PatchMatch 는 오류를 돌려준다.
- `unsafe` 는 cudarc 가 unsafe 로 둔 곳에만 쓴다: 커널 발사, 고정(pinned) 호스트 메모리, 텍스처 객체, `DeviceRepr`.
- 실행 시 `LD_LIBRARY_PATH` 에 `libcuda.so`, `libnvrtc.so` 가 있어야 한다(노드: `/usr/local/cuda-12.4/lib64`).
- `SKYRECON_CUDA_TRACE=1` 이면 커널 레지스터·점유율과 호출별 준비/올림/커널/내림 시간을 stderr 에 찍는다.

## 장치 (`device`)
- `is_available() -> bool`: 드라이버·NVRTC 라이브러리가 있고 장치가 1개 이상.
- `CudaDevice::new(ordinal) -> Result<Arc<CudaDevice>, GpuError>`: 문맥 + 전용 스트림, 계산 능력에 맞춘 NVRTC 아키텍처(V100 = compute_70). `.name()`.
- `GpuError { NotAvailable, Driver, Compile }`.

## PatchMatch (`CudaPatchMatch`: `skyrecon_dense::PatchMatchBackend`)
- `CudaPatchMatch::try_default()`(장치 0), `CudaPatchMatch::new(dev, CudaPatchMatchOptions)`, `.device()`, `.options()`.
- 사용: `skyrecon_dense::densify(&scene, &opts, &cuda, cache)` 또는 `compute_depth_maps`. 다중 스케일 진행·세부 복원 판정·필터 문턱·후처리·융합은
  dense 가, 한 스케일의 적·흑 실행·단일 가설 평가·판독·상향 표본·중앙값 필터는 이 백엔드가 GPU 에서 한다. CPU PatchMatch 경로는 없다.
- `CudaPatchMatchOptions` 기본: `hw_interp: true`(원천 표본을 블록 선형 배열 텍스처의 하드웨어 쌍선형으로, 거짓이면 소프트웨어 쌍선형),
  `batch_pixels: 2_200_000`·`max_batch: 64`(여러 기준 뷰를 그리드 z 로 묶어 한 번에 발사), `min_blocks: 2`(`__launch_bounds__`),
  `fast_math: true`(근사 나눗셈·제곱근; 끄면 IEEE). 환경 변수 `SKYRECON_PM_MIN_BLOCKS`, `SKYRECON_PM_FAST_MATH` 로 바꿀 수 있다.
- 커널(`kernels/patchmatch.cu`, 창 반경·간격을 컴파일 상수로 특수화):
  `pm_half`(활성 색 픽셀당 스레드 하나, 블록 32×8 = 64×8 픽셀, 기준 패치는 공유 메모리 타일, 36 표본 이하면 픽셀별 양방향 가중치도 공유 메모리),
  `pm_eval`(실행 시작: 무작위 초기화 + 상위 K 비용 / 단일 가설 평가), `pm_filter`(판독: 픽셀별 일치 뷰 수),
  `pm_upsample`(결합 양방향 상향 표본), `pm_median`(5×5 중앙값 평면).
- 세션 동안 모든 뷰·스케일 영상, 상대 기하(스케일별 `A = K_j R K⁻¹`, `b = K_j t`)는 장치에 상주. 기하 실행 깊이 스냅숏은 스냅숏 번호가 바뀔 때만 올린다.
  상태(평면·비용)는 실행마다 고정 호스트 버퍼로 올리고 내린다.
- 난수는 dense 와 같은 카운터 기반 픽셀 수열 → 같은 장치·입력이면 비트 단위로 같은 결과.
- `SKYRECON_CUDA_TRACE=1`: 커널 레지스터·지역 메모리·SM 당 블록 수와 실행별 시간.

## 검증 (노드 V100, `cargo test --release -p skyrecon-cuda --test patchmatch_gpu`)
- 합성 장면(무늬 바닥 + 상자 2개, 카메라 3×3, 384×288, 스케일 3단) 참 깊이·법선과 직접 비교:
  fast: 유효 0.982, 깊이 상대 오차 중앙 5.3e-4(90% 1.5e-3), 법선 중앙 2.05°; quality: 0.982, 5.1e-4, 2.57°; 소프트웨어 보간 fast: 5.3e-4, 2.06°.
  융합 점의 장면 표면 거리 중앙 1.0 cm(고도 20 m), 0.1 m 초과 0.02%. 같은 입력 두 번 실행 비트 동일. 깊이맵 캐시 재사용 9/9.

## 실영상 측정 (theater 240장, 960×539, Tesla V100 + 16코어, `skyrecon densify --pm-backend cuda`)
| | fast 240장 | quality 240장 | fast 42장 |
|---|---|---|---|
| 이웃 + 깊이 범위 | 2.4 s | 2.4 s | 0.25 s |
| 준비(피라미드·업로드·컴파일) | 1.7 s | 5.1 s | 1.4 s |
| 깊이(스케일 3단) | 46.7 s | 293.7 s | 8.3 s |
| 필터·후처리 | 4.3 s | 4.6 s | 0.8 s |
| 융합 | 6.2 s | 6.2 s | 1.1 s |
| 합 | 61.2 s | 312.0 s | 11.9 s |
| 점 수 | 13,466,552 | 12,312,654 | 2,906,112 |
| 최대 메모리(RSS) | 8.3 GB | 8.4 GB | 2.7 GB |

## 매칭 (`CudaMatcher`: `skyrecon_matching::MatcherBackend`)
- `CudaMatcher::try_default()`, `::new(dev)`, `.top2_gpu(d1, n1, d2, n2)`.
- 스레드 하나가 질의 기술자 하나, 대상 기술자를 128개 타일(공유 메모리)로 0번부터 차례로 훑는다. `__dp4a` 정수 내적.
  행·열 top-2 를 같은 커널로(입력만 바꿔) 구한다. "엄격히 큼" 순차 갱신이라 CPU `Top2::push` 와 비트 단위로 같다.

## SIFT (`CudaSift`: `skyrecon_features::SiftEngine`)
- `CudaSift::try_default()`, `::new(dev)`.
- 혼합형: 스케일 공간(정규화 /255, 2배 업샘플, 분리형 가우시안(커널은 `pyramid::gaussian_kernel`), 짝수 축소)을 GPU 에서
  `--fmad=false` 로 CPU 와 같은 연산 순서로 계산 → 비트 단위로 같은 피라미드. 옥타브를 거친 쪽부터 필요할 때만 고정 메모리로 내려받아
  DoG·검출·방향·기술자·특징 수 제한은 skyrecon-features 의 CPU 함수(같은 순서)로 계산한다. 결과는 `CpuSift` 와 같다(특징·기술자 비트 동일).

## 매칭·SIFT 검증 (노드 V100, `cargo test --release -p skyrecon-cuda`)
- `tests/matcher_gpu.rs`: 1×1 ~ 8192×8192 에서 행·열 top-2 가 CPU 와 같음. 8192×8192 CPU 161 ms → GPU 6.4 ms.
- `tests/sift_gpu.rs`: 4가지 옵션(업샘플/비업샘플, TopK, SpatialGrid+upright)에서 특징·기술자가 CPU 와 같음.

## SIFT·매칭 측정
- `cargo run --release -p skyrecon-cuda --example bench_sift_gpu -- <images/camF> 6`.
