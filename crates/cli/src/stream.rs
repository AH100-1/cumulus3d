//! `skyrecon stream`: 사용자 스크립트(incremental_stream.sh)와 같은 흐름을 한 프로세스·메모리 자료 구조로 수행.
//!
//! 위치가 하나씩 도착한다고 보고: 특징 → 새 짝 매칭 → (첫 구역 완료 시 전역 매퍼 / 이후 등록 + 삼각측량)
//! → 구역 완료 시 초벌(전경) 조밀화 + 정밀(배경 스레드: BA → GPS ENU 정렬 → 조밀화).
//! 배경 정밀 모델이 준비되면 다음 위치부터 그 모델 위에 이어 등록한다.

use crate::densewrap::{dense_model, make_match_backend, make_pm_backend, make_sift_backend, parse_profile, DenseConfig};
use crate::post::{self, PostInput};
use crate::util::{timed, Logger, StageTimes, Timeline};
use skyrecon_align::{align_to_gps, EnuOrigin, ModelAlignerOptions};
use skyrecon_ba::{bundle_adjust, BaConfig};
use skyrecon_core::analyzer::ModelStats;
use skyrecon_core::interop::{write_model_binary, ImageOrder};
use skyrecon_core::io::{read_gps_file, GpsRecord};
use skyrecon_core::{CameraId, MatchGraph, MatchGraphOptions, FeatureStore, ImageId, Reconstruction};
use skyrecon_dense::DepthMapCache;
use skyrecon_features::{CameraMode, ExtractionOptions, FeatureExtractor, ImageStatus};
use skyrecon_matching::{match_pairs, PairMatchingOptions};
use skyrecon_sfm::{
    global_mapper, register_images, triangulate_points, GlobalSfmOptions, PointTriangulatorOptions, RegistrationOptions,
    TriangulationScope,
};
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Instant;

/// 카메라 폴더(스크립트 순서).
pub const CAMS: [&str; 3] = ["camF", "camR", "camL"];

/// 스트림 설정. 기본값 = 스크립트 동작.
#[derive(Clone, Debug)]
pub struct StreamConfig {
    pub src: PathBuf,
    pub out: PathBuf,
    pub span: usize,
    pub overlap: usize,
    pub stride: usize,
    /// 새로 등록된 영상만 삼각측량(개선, 기본 끔).
    pub incremental_triangulation: bool,
    /// 겹침 영상 깊이맵 재사용(개선, 기본 끔).
    pub depth_cache: bool,
    /// 첫 GPS 줄로 세션 ENU 원점 고정(개선, 기본 끔 = 정렬마다 첫 공통 기록).
    pub fixed_enu_origin: bool,
    pub pm_backend: String,
    /// 조밀화 프로파일(fast|quality).
    pub mvs_profile: String,
    pub sift_backend: String,
    pub match_backend: String,
    pub threads: usize,
    pub max_positions: Option<usize>,
    pub save_models: bool,
    pub serialize_dense: bool,
    /// 왜곡 보정 최대 크기(스크립트 960).
    pub dense_max_image_size: i64,
    /// 조밀화 원천 뷰 수.
    pub number_views: usize,
    /// SIFT 최대 특징 수(스크립트 8192).
    pub max_num_features: usize,
    /// SIFT 입력 최대 크기(기본 3200).
    pub sift_max_image_size: usize,
    /// 표준 출력에도 run.log 내용을 보임.
    pub echo: bool,
    /// 조밀화 생략(개발·시험용).
    pub no_dense: bool,
    /// 두 뷰 기하·GPS 정렬 RANSAC 시드(None = 실행마다 다름).
    pub seed: Option<u64>,
}

impl StreamConfig {
    pub fn new(src: PathBuf, out: PathBuf) -> Self {
        Self {
            src,
            out,
            span: 12,
            overlap: 2,
            stride: 3,
            incremental_triangulation: false,
            depth_cache: false,
            fixed_enu_origin: false,
            pm_backend: "cuda".into(),
            mvs_profile: "fast".into(),
            sift_backend: "cpu".into(),
            match_backend: "cpu".into(),
            threads: 0,
            max_positions: None,
            save_models: false,
            serialize_dense: false,
            dense_max_image_size: 960,
            number_views: 10,
            max_num_features: 8192,
            sift_max_image_size: 3200,
            echo: true,
            no_dense: false,
            seed: None,
        }
    }
}

/// 위치 ↔ 영상 이름. 위치 = 선택된 프레임 목록의 순번(파일 번호 / stride 가 아님).
#[derive(Clone, Debug)]
pub struct Layout {
    /// 위치별 입력 파일의 프레임 번호.
    pub frames: Vec<u32>,
    pub pos_of: HashMap<String, usize>,
}

impl Layout {
    /// `images/camF/camF_NNNN.jpg` 중 번호 % stride == 0 인 것(번호 오름차순).
    pub fn discover(src: &Path, stride: usize) -> Result<Self, String> {
        let dir = src.join("images").join(CAMS[0]);
        let rd = std::fs::read_dir(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let mut frames = Vec::new();
        for e in rd.flatten() {
            let n = e.file_name().to_string_lossy().to_string();
            let Some(stem) = n.strip_prefix(&format!("{}_", CAMS[0])).and_then(|s| s.strip_suffix(".jpg")) else { continue };
            if let Ok(i) = stem.parse::<u32>() {
                if (i as usize).is_multiple_of(stride.max(1)) {
                    frames.push(i);
                }
            }
        }
        frames.sort_unstable();
        frames.dedup();
        let mut l = Self { frames, pos_of: HashMap::new() };
        for p in 0..l.frames.len() {
            for c in CAMS {
                let n = l.name(c, p);
                l.pos_of.insert(n, p);
            }
        }
        Ok(l)
    }

    pub fn name(&self, cam: &str, p: usize) -> String {
        format!("{cam}/{cam}_{:04}.jpg", self.frames[p])
    }

    pub fn position(&self, name: &str) -> Option<usize> {
        self.pos_of.get(name).copied()
    }
}

/// 스크립트와 같은 짝 규칙: 같은 카메라 간격 1..5, 8, 16 / 다른 카메라 위치 차 0..4. 정렬된 이름 짝.
pub fn pairs_for_position(layout: &Layout, p: usize) -> Vec<(String, String)> {
    let mut pr = BTreeSet::new();
    for c in CAMS {
        for d in [1usize, 2, 3, 4, 5, 8, 16] {
            if p >= d {
                pr.insert((layout.name(c, p - d), layout.name(c, p)));
            }
        }
    }
    for a in CAMS {
        for b in CAMS {
            if a == b {
                continue;
            }
            for q in p.saturating_sub(4)..=p {
                let (x, y) = (layout.name(a, p), layout.name(b, q));
                pr.insert(if x <= y { (x, y) } else { (y, x) });
            }
        }
    }
    pr.into_iter().collect()
}

/// 구역: start = 0, SPAN, …; [start−OVL, start+SPAN+OVL) ∩ [0, NPOS). 완료 위치 = hi−1.
pub fn regions(npos: usize, span: usize, ovl: usize) -> Vec<(usize, usize, usize)> {
    let mut v = Vec::new();
    let mut start = 0;
    let mut k = 0;
    while start < npos {
        let lo = start.saturating_sub(ovl);
        let hi = (start + span + ovl).min(npos);
        v.push((k, lo, hi));
        k += 1;
        start += span.max(1);
    }
    v
}

/// 스레드 사이 공유 상태.
struct Ctx {
    cfg: StreamConfig,
    layout: Layout,
    gps_all: Vec<GpsRecord>,
    log: Logger,
    tl: Timeline,
    times: StageTimes,
    dense: DenseConfig,
    /// 정밀 자세가 준비된 모델(adopt 대상이자 후처리 입력).
    refined_ready: Mutex<BTreeMap<usize, Reconstruction>>,
    refined_ply: Mutex<BTreeMap<usize, PathBuf>>,
}

impl Ctx {
    fn ev(&self, s: &str) {
        self.tl.ev(&self.log, s);
    }

    fn work(&self) -> PathBuf {
        self.cfg.out.join("work")
    }

    /// 등록된 영상의 GPS 기록만(gps_all 순서 유지).
    fn gps_for_model(&self, rec: &Reconstruction) -> Vec<GpsRecord> {
        let names: BTreeSet<String> = rec.registered_images().into_iter().filter_map(|i| rec.image(i).map(|im| im.name.clone())).collect();
        self.gps_all.iter().filter(|g| names.contains(&g.name)).cloned().collect()
    }

    fn aligner_options(&self) -> ModelAlignerOptions {
        let mut o = ModelAlignerOptions { max_error: 3.0, ..Default::default() };
        o.ransac.random_seed = self.cfg.seed;
        if self.cfg.fixed_enu_origin {
            if let Some(g) = self.gps_all.first() {
                o.origin = EnuOrigin::Explicit { lat: g.lat, lon: g.lon, alt: g.alt };
            }
        }
        o
    }

    /// model_aligner(ref_is_gps, enu, max_error 3). 실패하면 모델 그대로 두고 경고.
    fn align_enu(&self, rec: &mut Reconstruction, what: &str) -> bool {
        let gps = self.gps_for_model(rec);
        let opts = self.aligner_options();
        let r = timed(&self.log, &self.times, "align", what, || align_to_gps(rec, &gps, &opts));
        match r {
            Ok(a) => {
                self.log.line(&format!(
                    "[align] {what}: 공통 {} 인라이어 {} 오차 평균 {:.3}m 중앙 {:.3}m 축척 {:.4}",
                    a.common.len(),
                    a.num_inliers,
                    a.mean_error,
                    a.median_error,
                    a.sim3.scale
                ));
                true
            }
            Err(e) => {
                // 설계 결정: 정렬에 실패해도 멈추지 않고 정렬 안 된 모델로 계속한다.
                self.log.line(&format!("[align] {what}: 정렬 실패({e}) — 정렬 없이 계속"));
                false
            }
        }
    }

    fn save_model(&self, rec: &Reconstruction, dir: &str) {
        if !self.cfg.save_models {
            return;
        }
        let d = self.work().join("models").join(dir);
        let _ = std::fs::create_dir_all(&d);
        if let Err(e) = write_model_binary(rec, &d, ImageOrder::Registration) {
            self.log.line(&format!("모델 저장 실패 {}: {e}", d.display()));
        }
    }

    /// 조밀화(구역 밖 영상 제외). 성공하면 PLY 경로.
    fn dense_region(&self, kind: &str, k: usize, model: &Reconstruction, lo: usize, hi: usize) -> Option<PathBuf> {
        if self.cfg.no_dense {
            return None;
        }
        let keep = |n: &str| self.layout.position(n).is_some_and(|p| p >= lo && p < hi);
        let what = format!("{kind} {k}");
        let t = Instant::now();
        let r = dense_model(model, keep, &self.cfg.src.join("images"), &self.dense);
        let total = t.elapsed();
        match r {
            Ok(run) => {
                self.times.add("undistort", run.undistort_time);
                self.times.add("densify", run.densify_time);
                let tm = &run.output.timings;
                self.log.stamped(&format!(
                    "[time] dense {what}: 전체 {:.1}s (왜곡 보정+장면 {:.1}s, 대기 {:.1}s, 조밀화 {:.1}s = 이웃 {:.1} 준비 {:.1} 깊이 {:.1} 필터 {:.1} 융합 {:.1}, 캐시 적중 {})",
                    total.as_secs_f64(),
                    run.undistort_time.as_secs_f64(),
                    run.lock_wait.as_secs_f64(),
                    run.densify_time.as_secs_f64(),
                    tm.neighbors.as_secs_f64(),
                    tm.prepare.as_secs_f64(),
                    tm.depth().as_secs_f64(),
                    tm.filter.as_secs_f64(),
                    tm.fusion.as_secs_f64(),
                    run.output.cache_hits
                ));
                let path = self.cfg.out.join("full").join(kind).join(format!("{kind}_{k:02}_pos{lo}-{hi}.ply"));
                let ok = run.output.write_ply(&path);
                self.log.line(&format!("[dense] {kind} {k} frames {} points {}", run.frames, run.output.cloud.len()));
                match ok {
                    Ok(()) => Some(path),
                    Err(e) => {
                        self.log.line(&format!("[dense] {kind} {k} PLY 쓰기 실패: {e}"));
                        None
                    }
                }
            }
            Err(e) => {
                self.log.line(&format!("[dense] {kind} {k} 실패: {e}"));
                None
            }
        }
    }

    /// 배경 정밀 작업: 사본 → BA → ENU 정렬 → (adopt 가능) → 조밀화.
    fn refine_job(&self, k: usize, lo: usize, hi: usize, mut rec: Reconstruction) {
        self.ev(&format!("refined {k} ba_start"));
        let r = timed(&self.log, &self.times, "ba", &format!("refined {k}"), || bundle_adjust(&mut rec, &BaConfig::default()));
        match r {
            Ok(s) => self.log.line(&format!(
                "[ba] refined {k}: 반복 {} 비용 {:.4e} → {:.4e} ({:?}), RMS {:.3}px",
                s.num_iterations,
                s.initial_cost,
                s.final_cost,
                s.termination,
                s.rms_reprojection_error()
            )),
            Err(e) => self.log.line(&format!("[ba] refined {k} 실패: {e}")),
        }
        self.save_model(&rec, &format!("ba_{k}"));
        self.align_enu(&mut rec, &format!("refined {k}"));
        self.save_model(&rec, &format!("refined_{k}"));
        let st = ModelStats::compute(&rec);
        if let Ok(mut m) = self.refined_ready.lock() {
            m.insert(k, rec.clone());
        }
        self.ev(&format!(
            "refined {k} pose_ready Registered images: {} Mean reprojection error: {:.6}px ",
            st.registered_image_count, st.mean_reprojection_error
        ));
        if let Some(p) = self.dense_region("refined", k, &rec, lo, hi) {
            if let Ok(mut m) = self.refined_ply.lock() {
                m.insert(k, p);
            }
        }
        self.ev(&format!("refined {k} ready"));
    }
}

fn reg_count(rec: &Option<Reconstruction>) -> usize {
    rec.as_ref().map_or(0, |r| r.registered_image_count())
}

/// 스트림 실행. 출력 폴더를 새로 만든다.
pub fn run_stream(cfg: StreamConfig) -> Result<(), String> {
    if cfg.threads > 0 {
        // 이미 만들어졌으면(테스트 등) 무시.
        let _ = rayon::ThreadPoolBuilder::new().num_threads(cfg.threads).build_global();
    }
    // 백엔드가 없으면 조밀화 단계에서 오류로 기록한다.
    let backend = make_pm_backend(&cfg.pm_backend);
    let profile = parse_profile(&cfg.mvs_profile)?;
    let out = cfg.out.clone();
    if out.exists() {
        std::fs::remove_dir_all(&out).map_err(|e| format!("{}: {e}", out.display()))?;
    }
    for d in ["full/preview", "full/refined", "work"] {
        std::fs::create_dir_all(out.join(d)).map_err(|e| e.to_string())?;
    }
    let log = Logger::new(Some(&out.join("run.log")), cfg.echo).map_err(|e| e.to_string())?;
    let tl = Timeline::new(&out.join("timeline.txt")).map_err(|e| e.to_string())?;

    let mut layout = Layout::discover(&cfg.src, cfg.stride)?;
    if let Some(m) = cfg.max_positions {
        layout.frames.truncate(m);
        let frames = layout.frames.len();
        layout.pos_of.retain(|_, p| *p < frames);
    }
    let npos = layout.frames.len();
    let gps_path = cfg.src.join("gps_ref.txt");
    let gps_all: Vec<GpsRecord> = read_gps_file(&gps_path)
        .map_err(|e| format!("{}: {e}", gps_path.display()))?
        .into_iter()
        .filter(|g| layout.pos_of.contains_key(&g.name))
        .collect();

    let mut dense = DenseConfig::new(backend, profile);
    dense.undistort.max_image_size = cfg.dense_max_image_size;
    dense.densify.neighbors.num_views = cfg.number_views;
    if cfg.serialize_dense {
        dense.lock = Some(Arc::new(Mutex::new(())));
    }
    if cfg.depth_cache {
        dense.depth_cache = Some(Arc::new(DepthMapCache::new()));
    }

    let ctx = Arc::new(Ctx {
        cfg: cfg.clone(),
        layout,
        gps_all,
        log,
        tl,
        times: StageTimes::default(),
        dense,
        refined_ready: Mutex::new(BTreeMap::new()),
        refined_ply: Mutex::new(BTreeMap::new()),
    });
    let t_start = Instant::now();
    ctx.ev(&format!("start NPOS={npos}"));

    let (span, ovl) = (cfg.span, cfg.overlap);
    let regs = regions(npos, span, ovl);
    let mut reg_end: BTreeMap<usize, Vec<(usize, usize, usize)>> = BTreeMap::new();
    for &(k, lo, hi) in &regs {
        reg_end.entry(hi - 1).or_default().push((k, lo, hi));
    }
    ctx.log.line(&format!(
        "regions: {}",
        reg_end
            .iter()
            .map(|(e, v)| format!("{e} ->{}", v.iter().map(|(k, lo, hi)| format!(" {k}:{lo}:{hi}")).collect::<String>()))
            .collect::<Vec<_>>()
            .join(", ")
    ));
    ctx.log.line(&format!(
        "설정: span {span} overlap {ovl} stride {} 증분삼각측량 {} 깊이캐시 {} 고정원점 {} pm {}({})/sift {}/match {} 직렬조밀화 {} 왜곡보정최대 {} 뷰 {} 스레드 {}",
        cfg.stride,
        cfg.incremental_triangulation,
        cfg.depth_cache,
        cfg.fixed_enu_origin,
        cfg.pm_backend,
        cfg.mvs_profile,
        cfg.sift_backend,
        cfg.match_backend,
        cfg.serialize_dense,
        cfg.dense_max_image_size,
        cfg.number_views,
        rayon::current_num_threads()
    ));

    let store = FeatureStore::new();
    let extractor = FeatureExtractor::with_backend(make_sift_backend(&cfg.sift_backend)?);
    let matcher = make_match_backend(&cfg.match_backend)?;
    let mut match_opts = PairMatchingOptions::default();
    match_opts.geometry.ransac.random_seed = cfg.seed;
    let graph_opts = MatchGraphOptions::default();
    let mut graph = MatchGraph::new();
    let mut cam_ids: HashMap<&str, CameraId> = HashMap::new();
    let image_root = cfg.src.join("images");
    let mut ext_opts = ExtractionOptions::default();
    ext_opts.sift.max_num_features = cfg.max_num_features;
    ext_opts.reader.max_image_size = cfg.sift_max_image_size;

    let mut chain: Option<Reconstruction> = None;
    let mut adopted: Option<usize> = None;
    let mut previews: BTreeMap<usize, Reconstruction> = BTreeMap::new();
    let mut preview_ply: BTreeMap<usize, PathBuf> = BTreeMap::new();
    let mut handles = Vec::new();

    for p in 0..npos {
        // 1) 특징: 첫 위치는 폴더당 카메라, 이후 같은 카메라 id.
        timed(&ctx.log, &ctx.times, "feature", &format!("pos {p}"), || -> Result<(), String> {
            if p == 0 {
                let names: Vec<String> = CAMS.iter().map(|c| ctx.layout.name(c, 0)).collect();
                let mut o = ext_opts.clone();
                o.reader.camera_mode = CameraMode::PerFolder;
                let rep = extractor.extract_files(&store, &image_root, &names, &o).map_err(|e| e.to_string())?;
                report_extraction(&ctx.log, &rep);
                for c in CAMS {
                    let im = store.image_by_name(&ctx.layout.name(c, 0)).ok_or_else(|| format!("{c} 첫 영상 특징 추출 실패"))?;
                    cam_ids.insert(c, im.camera_id);
                }
            } else {
                for c in CAMS {
                    let mut o = ext_opts.clone();
                    o.reader.camera_mode = CameraMode::Existing(cam_ids[c]);
                    let rep = extractor.extract_files(&store, &image_root, &[ctx.layout.name(c, p)], &o).map_err(|e| e.to_string())?;
                    report_extraction(&ctx.log, &rep);
                }
            }
            Ok(())
        })?;

        // 2) 새 영상이 낀 짝만 매칭.
        let pairs: Vec<(ImageId, ImageId)> = pairs_for_position(&ctx.layout, p)
            .into_iter()
            .filter_map(|(a, b)| Some((store.image_id_by_name(&a)?, store.image_id_by_name(&b)?)))
            .collect();
        let st = timed(&ctx.log, &ctx.times, "matching", &format!("pos {p} ({} pairs)", pairs.len()), || {
            match_pairs(&store, &pairs, &match_opts, matcher.as_ref())
        })
        .map_err(|e| e.to_string())?;
        graph.update_from_store(&store, &graph_opts);
        ctx.log.line(&format!(
            "[match] pos {p}: 짝 {} 매칭 {} 유효 기하 {} (그래프 영상 {})",
            pairs.len(),
            st.num_matched,
            st.num_valid_geometries,
            graph.image_ids().len()
        ));

        // 3) 등록.
        if p + 1 < span + ovl {
            continue;
        }
        if p + 1 == span + ovl {
            let o = timed(&ctx.log, &ctx.times, "mapper", &format!("pos {p}"), || {
                global_mapper(&store, &graph, &GlobalSfmOptions::script())
            })
            .map_err(|e| e.to_string())?;
            if let Some(f) = &o.failure {
                ctx.log.line(&format!("[mapper] 실패 보고: {f}"));
            }
            ctx.log.line(&format!("[mapper] {:?}", o.summary));
            chain = Some(o.reconstruction);
            ctx.ev(&format!("init_model Registered images: {}", reg_count(&chain)));
        } else {
            // 새 정밀 모델이 나왔으면 그 위에서 이어 등록.
            let mut k = adopted.map_or(0, |a| a + 1);
            loop {
                let r = ctx.refined_ready.lock().ok().and_then(|m| m.get(&k).cloned());
                let Some(r) = r else { break };
                chain = Some(r);
                adopted = Some(k);
                ctx.ev(&format!("adopt refined {k} as base"));
                k += 1;
            }
            if let Some(base) = &chain {
                let mut reg = base.clone();
                let rr = timed(&ctx.log, &ctx.times, "registration", &format!("pos {p}"), || {
                    register_images(&mut reg, &store, &graph, &RegistrationOptions::default())
                });
                match rr {
                    Ok(rep) => {
                        let new_ids = rep.registered();
                        ctx.log.line(&format!("[register] pos {p}: 시도 {} 등록 {}", rep.attempts.len(), new_ids.len()));
                        let scope = if cfg.incremental_triangulation {
                            TriangulationScope::Images(new_ids)
                        } else {
                            TriangulationScope::AllRegistered
                        };
                        let topts = PointTriangulatorOptions { clear_points: false, scope, ..Default::default() };
                        let tr = timed(&ctx.log, &ctx.times, "triangulation", &format!("pos {p}"), || {
                            triangulate_points(&mut reg, &graph, &topts)
                        });
                        match tr {
                            Ok(t) => {
                                ctx.log.line(&format!(
                                    "[triangulate] pos {p}: 생성 {} 연장 {} 완성 {} 병합 {} 필터 {} (점 {})",
                                    t.num_created,
                                    t.num_continued,
                                    t.num_completed,
                                    t.num_merged,
                                    t.num_filtered,
                                    reg.num_points3d()
                                ));
                                chain = Some(reg);
                            }
                            Err(e) => ctx.log.line(&format!("[triangulate] pos {p} 실패: {e} (이전 모델 유지)")),
                        }
                    }
                    Err(e) => ctx.log.line(&format!("[register] pos {p} 실패: {e}")),
                }
            }
        }
        ctx.ev(&format!("pos {p} registered {}/{}", reg_count(&chain), 3 * (p + 1)));

        // 4) 구역 완료 → 초벌(전경) + 정밀(배경).
        for &(k, lo, hi) in reg_end.get(&p).map(|v| v.as_slice()).unwrap_or(&[]) {
            ctx.ev(&format!("region {k} arrived pos[{lo},{hi})"));
            let snap = chain.clone().unwrap_or_default();
            ctx.save_model(&snap, &format!("snap_{k}"));
            let mut preview = snap.clone();
            if k == 0 {
                ctx.align_enu(&mut preview, "preview 0");
            }
            ctx.save_model(&preview, &format!("preview_{k}"));
            let c2 = ctx.clone();
            let snap_bg = snap;
            handles.push(std::thread::spawn(move || c2.refine_job(k, lo, hi, snap_bg)));
            if let Some(path) = ctx.dense_region("preview", k, &preview, lo, hi) {
                preview_ply.insert(k, path);
            }
            previews.insert(k, preview);
            ctx.ev(&format!("preview {k} ready"));
        }
    }
    ctx.ev("all positions done");
    for h in handles {
        let _ = h.join();
    }
    ctx.ev("all refined done");
    if let Some(c) = &chain {
        ctx.save_model(c, "chain");
    }
    let pipeline_time = t_start.elapsed();

    // 5) 정렬·사건별 스냅샷·재기준.
    let refined = ctx.refined_ready.lock().map(|m| m.clone()).unwrap_or_default();
    let refined_ply = ctx.refined_ply.lock().map(|m| m.clone()).unwrap_or_default();
    let layout = ctx.layout.clone();
    let input = PostInput {
        out: &out,
        span,
        ovl,
        events: ctx.tl.relative(),
        previews: &previews,
        refined: &refined,
        preview_ply: &preview_ply,
        refined_ply: &refined_ply,
        position: &|n: &str| layout.position(n),
    };
    let tp = Instant::now();
    let post_result = post::run(&input, &ctx.log);
    ctx.times.add("post", tp.elapsed());
    if let Err(e) = &post_result {
        ctx.log.line(&format!("[post] 실패: {e}"));
    }
    let _ = std::fs::copy(out.join("timeline.txt"), out.join("snapshots").join("timeline.txt"));

    ctx.log.line(&format!("== 단계별 시간 (파이프라인 {:.1}s, 배경 스레드와 겹친 누적)", pipeline_time.as_secs_f64()));
    for (stage, (d, n)) in ctx.times.snapshot() {
        ctx.log.line(&format!("{stage:<14} {:>9.2}s  ({n}회)", d.as_secs_f64()));
    }
    if let Some(c) = &chain {
        ctx.log.line("== 최종 체인 모델");
        for l in ModelStats::compute(c).lines() {
            ctx.log.line(&l);
        }
    }
    std::fs::write(out.join("DONE"), b"").map_err(|e| e.to_string())?;
    post_result
}

fn report_extraction(log: &Logger, rep: &[skyrecon_features::ImageReport]) {
    for r in rep {
        match &r.status {
            ImageStatus::Extracted { image_id, camera_id, num_features } => {
                log.line(&format!("[feature] {} id {image_id} cam {camera_id} 특징 {num_features}", r.name))
            }
            ImageStatus::AlreadyExists { image_id } => log.line(&format!("[feature] {} id {image_id} 이미 있음", r.name)),
            ImageStatus::Failed { error } => log.line(&format!("[feature] {} 실패: {error}", r.name)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn regions_match_script() {
        // NPOS=26, SPAN 12, OVL 2 → [0,14), [10,26), [22,26)
        assert_eq!(regions(26, 12, 2), vec![(0, 0, 14), (1, 10, 26), (2, 22, 26)]);
        assert_eq!(regions(40, 12, 2), vec![(0, 0, 14), (1, 10, 26), (2, 22, 38), (3, 34, 40)]);
    }

    #[test]
    fn pairs_rule() {
        let frames: Vec<u32> = (0..20).map(|i| i * 3).collect();
        let mut l = Layout { frames, pos_of: HashMap::new() };
        l.pos_of.insert("x".into(), 0);
        let p0 = pairs_for_position(&l, 0);
        // 위치 0: 다른 카메라끼리 3짝.
        assert_eq!(p0.len(), 3);
        let p17 = pairs_for_position(&l, 17);
        // 같은 카메라 7간격 × 3 + 다른 카메라: 6 순서쌍 × 5 위치 중 q=p 는 중복 → 3 + 6×4 = 27.
        assert_eq!(p17.len(), 21 + 27);
        assert!(p17.contains(&("camF/camF_0003.jpg".to_string(), "camF/camF_0051.jpg".to_string())));
    }
}
