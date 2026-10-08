# skyrecon (crates/cli)

`skyrecon` 실행 파일. 두 가지를 제공한다.

1. `skyrecon stream` — 드론 편대 영상이 위치 단위로 도착하는 상황을 가정한 점진 재구성 파이프라인.
   특징 추출 → 매칭 → 전역/증분 SfM → 구역별 초벌·정밀 조밀화 → 정렬·스냅샷 후처리를
   **한 프로세스, 메모리 자료 구조**로 잇는다(중간 데이터베이스·모델 폴더 복사·형식 변환 없음).
2. 단계별 하위 명령(`interop` 모듈) — `feature_extractor`, `matches_importer`, `global_mapper`, `image_registrator`,
   `point_triangulator`, `bundle_adjuster`, `model_aligner`, `model_analyzer`, `model_converter`, `image_deleter`,
   `image_undistorter`, `densify`. 기존 사용자 스크립트가 쓰던 명령 이름·옵션 문자열을 그대로 받는다.
   단계 사이 상태는 `--database_path`(skyrecon 특징 저장소 이진 파일 `SKYFS`)와 모델 폴더(`skyrecon_core::interop` 형식)로 잇는다.

빌드: `cargo build --release -p skyrecon-cli` → `target/release/skyrecon`.

## 1. stream

```
skyrecon stream --src <폴더> --out <폴더> [--span 12 --overlap 2 --stride 3] [옵션]
```

입력 폴더: `images/camF|camR|camL/<cam>_NNNN.jpg`, `gps_ref.txt`(줄: `camF/camF_0000.jpg 위도 경도 고도`).

- 프레임 선택: `camF_NNNN.jpg` 중 `NNNN % stride == 0`. **위치 = 선택된 프레임 목록의 순번**(파일 번호 / stride 가 아님).
  그래서 이미 솎아 낸 미리 3장 간격으로 솎아 둔 입력 폴더(파일 번호 0,3,6,…)은 `--stride 1` 로 주면 stride 3 으로 전체 데이터를 돌린 것과 같은 위치가 된다.
- GPS 는 선택된 영상 줄만 남긴다.

### 위치 도착 루프
1. 특징: 위치 0 은 폴더당 카메라(`PerFolder`), 이후는 그 카메라 id(`Existing`). OPENCV 모델, 최대 8192 특징.
2. 짝: 같은 카메라 간격 1..5, 8, 16 / 다른 카메라 위치 차 0..4 — 새 영상이 낀 짝만 매칭·기하 검증, 대응 그래프는 증분 갱신.
3. 위치 `SPAN+OVL-1`(기본 13): 전역 SfM(`GlobalSfmOptions::script()` = BA 0회, 재삼각측량 생략, 트랙 10만) → `init_model`.
   이후 위치: 준비된 정밀 모델이 있으면 먼저 채택(`adopt refined K as base`) → 영상 등록 → 삼각측량(기존 점 유지).
   삼각측량이 실패하면 그 위치의 결과를 버리고 이전 모델을 유지한다.
4. 구역 완료(구역 k = 위치 [12k−2, 12k+14) ∩ [0,NPOS), 완료 위치 = hi−1):
   - 초벌(전경): 모델 사본(구역 0 만 GPS ENU 정렬) → 조밀화.
   - 정밀(배경 스레드): 사본 → BA → 등록 영상 GPS 로 ENU 정렬(max_error 3) → `pose_ready` → 조밀화.
   - 조밀화 = 구역 밖 영상 제외 → 왜곡 보정(max 960, 메모리) → 장면 변환 → densify(다중 스케일 PatchMatch(GPU) → 필터 → 후처리 → 일치 융합, 원천 뷰 10).
   - `Reconstruction` 사본은 `Clone`(영상·점 Arc copy-on-write)이라 싸다.

### 출력
| 경로 | 내용 |
|---|---|
| `timeline.txt` | `epoch초.나노 사건` (`start NPOS=..`, `init_model Registered images: N`, `pos P registered N/M`, `region K arrived pos[lo,hi)`, `refined K ba_start`, `refined K pose_ready Registered images: N Mean reprojection error: Xpx `, `refined K ready`, `preview K ready`, `adopt refined K as base`, `all positions done`, `all refined done`) |
| `run.log` | `[HH:MM:SS] 사건`, 단계별 `[time] <단계> ...: 초`, 등록·삼각측량·BA·정렬·조밀화 통계, 마지막에 요약 + 단계별 누적 시간 |
| `full/preview/preview_KK_pos{lo}-{hi}.ply`, `full/refined/...` | 조밀 점군(x y z, rgb, nx ny nz; float/uchar, 27 B/점) |
| `aligned/preview/*.ply` | 초벌을 그 시점의 최신 정밀 모델(없으면 GPS 정렬된 preview_0)에 공유 3D 점 + 견고 Umeyama 로 정렬 |
| `aligned/refined/*.ply` | 정밀 그대로 |
| `snapshots/event_NN_TTTT.Ts_<kind>_K.ply` | 사건 순서별 화면 상태(재기준 후): 정밀 전부 + 정밀본 없는 구역의 초벌(정밀 점 1.5 m 이내 제거), 1/6 추출 |
| `snapshots/manifest.json` | 키 `timeline`, `events`(+`frame`), `align`, `reanchor` (`indent=1` JSON) |
| `snapshots/timeline.txt` | 사본 |
| `final_frame/{preview,refined}/*.ply` | 마지막 정밀 좌표계로 맞춘 구역별 전체 해상도 점군(reanchor) |
| `work/models/{snap,preview,ba,refined}_K/`, `work/models/chain/` | `--save-models` 일 때만, 이진 희소 모델 |
| `DONE` | 완료 표시 |

요약 출력(run.log 끝, 표준 출력): `== 구역별 시간(초, 시작 기준)` 의 dict 줄, `== 정렬`, `== 사건 순서`, `정밀 a-b 겹침 차 중앙 Xm`,
reanchor 의 `a -> b: 점쌍 N, 잔차 중앙 Xm, 축척 S, 이동 Tm`, `사건 시각 종류 구역 좌표계 F 점 N`, `최종 좌표계 정밀 a-b 겹침 차 중앙 Xm (겹침 점 N)`.

### 옵션
| 옵션 | 기본 | 설명 |
|---|---|---|
| `--span/--overlap/--stride` | 12/2/3 | 구역 크기·겹침·프레임 간격 |
| `--incremental-triangulation` | 끔 | 새로 등록된 영상만 삼각측량(빠름, 결과 다를 수 있음) |
| `--depth-cache` | 끔 | 깊이맵 캐시(키 = 영상 이름 + 자세·K 해시) — 같은 자세로 다시 조밀화하는 겹침 영상만 적중 |
| `--fixed-enu-origin` | 끔 | 모든 ENU 정렬의 원점을 GPS 파일 첫 줄로 고정(기본: 정렬마다 첫 공통 기록) |
| `--pm-backend cuda` | cuda | PatchMatch 백엔드(`skyrecon-cuda`). 장치가 없으면 조밀화 단계에서 오류 |
| `--mvs-profile fast\|quality` | fast | 조밀화 프로파일(창 36/121 표본, 반복 일정) |
| `--sift-backend`, `--match-backend` | cpu | SIFT·기술자 매칭 백엔드(cpu\|cuda, 결과 동일). `--gpu` 는 세 백엔드를 모두 cuda 로 |
| `--serialize-dense` | 끔 | 초벌·정밀 조밀화 백엔드 호출을 Mutex 로 직렬화(기본: 동시 실행) |
| `--threads N` | 0 | rayon 스레드 수(0 = 코어 수) |
| `--max-positions N` | 전체 | 앞 N 위치만 |
| `--save-models` | 끔 | 구역별 희소 모델 저장 |
| `--dense-max-image-size` | 960 | 왜곡 보정 최대 크기(개발 중 320 등으로 낮추면 빠름) |
| `--number-views` | 10 | 조밀화 원천 뷰 수(≤ 32) |
| `--max-num-features` / `--sift-max-image-size` | 8192 / 3200 | SIFT |
| `--seed N` | 없음 | 두 뷰 기하·GPS 정렬 RANSAC 시드 고정(기본은 실행마다 다름) |
| `--no-dense` | 끔 | 조밀화 생략(개발용; 후처리 출력 없음) |
| `--quiet` | 끔 | run.log 에만 기록 |

예:
```
# 실데이터 일부(26위치), 기본 설정
skyrecon stream --src <input-dir> --out <output-dir> --stride 1
# 빠른 시험
skyrecon stream --src <input-dir> --out <output-dir-quick> --stride 1 --max-positions 15 --dense-max-image-size 320
```

## 2. 단계별 하위 명령

불린 옵션은 `1/0/true/false` 를 받는다. GPU 관련 옵션은 받기만 하고 무시한다.

```
skyrecon feature_extractor --database_path db.skyfs --image_path images --image_list_path list.txt \
    --ImageReader.single_camera_per_folder 1 --ImageReader.camera_model OPENCV --SiftExtraction.max_num_features 8192
skyrecon feature_extractor ... --ImageReader.existing_camera_id 1
skyrecon matches_importer --database_path db.skyfs --match_list_path pairs.txt --match_type pairs
skyrecon global_mapper --database_path db.skyfs --image_path images --output_path sg0 \
    --GlobalMapper.ba_num_iterations 0 --GlobalMapper.skip_retriangulation 1 --GlobalMapper.keep_max_num_tracks 100000
skyrecon image_registrator --database_path db.skyfs --input_path chain --output_path reg
skyrecon point_triangulator --database_path db.skyfs --image_path images --input_path reg --output_path tri --clear_points 0
skyrecon bundle_adjuster --input_path in --output_path out
skyrecon model_aligner --input_path in --output_path out --ref_images_path gps.txt --ref_is_gps 1 --alignment_type enu --alignment_max_error 3
skyrecon model_analyzer --path model [--verbose 1]
skyrecon model_converter --input_path model --output_path out --output_type BIN|TXT|PLY
skyrecon image_deleter --input_path in --output_path out --image_names_path del.txt   (또는 --image_ids_path)
skyrecon image_undistorter --image_path images --input_path in --output_path dense --max_image_size 960
skyrecon densify -i dense -o dense.ply [--mvs-profile fast|quality --number-views 10 --window-radius R --window-step S --fusion-mode consistency|traversal --stats]
skyrecon densify -i <왜곡 있는 모델> --image_path <영상> -o dense.ply [--max-images N]   # 메모리에서 왜곡 보정
skyrecon densify -i model --image_path images -o dense.ply   (왜곡 보정까지 메모리에서)
```
- `global_mapper` 는 `output_path/0` 에 모델 1개(최대 연결 성분)를 쓴다. `global_mapper`/`point_triangulator` 는 `--image_path` 가 있으면 점 색을 추출한다.
- `model_aligner` 는 `--ref_is_gps 1 --alignment_type enu` 만 지원.
- `image_undistorter` 출력 폴더: `images/`, `sparse/`, `stereo/{depth_maps,normal_maps,consistency_graphs}/`, `stereo/patch-match.cfg`, `stereo/fusion.cfg`.
- 특징 저장소는 SQLite 가 아니라 skyrecon 자체 이진 형식이다.

## 시험
- `cargo test --release -p skyrecon-cli`: 단위(구역·짝 규칙, JSON/repr 모양) +
  `tests/stream_synthetic.rs`(레이 캐스팅 합성 장면 3대 × 8위치, span 4/overlap 1 로 stream 을 끝까지: 사건 문구·순서, 출력 파일 구조,
  manifest 키, ENU 정렬된 바닥 높이, 정밀 GPS 정렬 오차) + `tests/interop_commands.rs`(단계별 명령 연쇄).
- 개발용 예제: `cargo run --release -p skyrecon-cli --example ba_probe -- <모델>` (BA 설정별 스케일·초점 변화).

## 동작 메모
- 점 색: 스트림 경로는 3D 점 색을 추출하지 않는다(조밀화 결과에는 영향 없음). 단계별 `point_triangulator` 는 `--image_path` 가 있으면 추출한다.
- GPS 정렬이 실패하면 경고 후 정렬 안 된 모델로 계속한다(설계 결정: 파이프라인을 멈추지 않음).
- 후처리에서 공유 점이 3쌍 미만이면 항등 변환을 쓰고 `reference` 에 `(실패)` 를 표시한다.
- 시각 `[HH:MM:SS]` 은 `date +%z` 로 얻은 현지 시간대를 쓴다.
