# skyrecon

드론 편대 영상으로 3D 점군을 **점진적으로** 만드는 Rust 워크스페이스.
영상이 위치 단위로 도착하는 동안 특징 추출 → 매칭 → SfM → GPS 정렬 → 조밀화를 한 프로세스 안에서 잇고,
구역마다 빠른 초벌(BA 없음)과 정밀본(BA 있음)을 내서 정밀 지도(ENU) 위에 단계별 스냅샷을 쌓는다.
중간 데이터베이스나 디스크 모델 폴더 없이 메모리 자료 구조로 단계를 넘긴다.

## 크레이트

| 크레이트 | 역할 |
|---|---|
| `crates/core` (`skyrecon-core`) | 카메라 모델, 기하(Rigid3/Sim3), 재구성 자료 구조, 대응 그래프, 특징 저장소, RANSAC, PLY/GPS 입출력, 모델 파일 형식(`interop`) |
| `crates/features` (`skyrecon-features`) | 영상 읽기·EXIF·카메라 초기화, SIFT 검출·기술자 |
| `crates/matching` (`skyrecon-matching`) | 기술자 매칭, 짝 목록, 두 뷰 기하(E/F/H) 추정·검증 |
| `crates/ba` (`skyrecon-ba`) | 번들 조정(신뢰 영역 LM, Schur 보완 선형 풀이), 절대 자세 정제 |
| `crates/sfm` (`skyrecon-sfm`) | 전역 SfM(회전 평균 + 위치 추정), 영상 등록, 삼각측량 |
| `crates/align` (`skyrecon-align`) | GPS ↔ ENU 변환, 모델 정렬(Umeyama, 견고 추정), 점군 재기준 |
| `crates/dense` (`skyrecon-dense`) | 왜곡 보정, 장면 변환, PatchStereo 깊이맵, 필터·융합 → 조밀 점군 |
| `crates/cuda` (`skyrecon-cuda`) | GPU 백엔드(PatchStereo, 기술자 매칭, SIFT 스케일 공간). CUDA 없는 기계에서도 빌드됨 |
| `crates/cli` (`skyrecon`) | 실행 파일: `skyrecon stream` 과 단계별 하위 명령 |

각 크레이트의 공개 API 는 `crates/*/API.md`, 실행 파일 사용법은 `crates/cli/README.md` 에 있다.

## 빌드·시험

```bash
cargo build --release
cargo test --release --workspace
cargo clippy --workspace --all-targets
```

## 사용

```bash
# 점진 파이프라인(입력: images/camF|camR|camL/*.jpg + gps_ref.txt)
target/release/skyrecon stream --src <입력 폴더> --out <출력 폴더>
# GPU 백엔드 사용(CUDA 12.x)
target/release/skyrecon stream --src <입력> --out <출력> --gpu
# 단계별 하위 명령(기존 스크립트용): feature_extractor, matches_importer, global_mapper, image_registrator,
# point_triangulator, bundle_adjuster, model_aligner, model_analyzer, model_converter, image_deleter,
# image_undistorter, densify
target/release/skyrecon model_analyzer --path <모델 폴더>
```

## 라이선스

MIT 또는 Apache-2.0 중 선택(`LICENSE-MIT`, `LICENSE-APACHE`).
