//! `skyrecon stream`: 입력 폴더의 위치를 하나씩 [`Session`] 리듀서에 넣고([`Pipeline`]),
//! 기본 출력 훅([`crate::sinks`])이 timeline.txt·run.log·점군·스냅샷 파일을 만든다.
//! 계산(특징 → 짝 매칭 → 첫 모델/이어 등록 → 구역별 초벌·정밀)은 [`crate::session`] 에 있다.

use crate::densewrap::{make_match_backend, make_pm_backend, make_sift_backend, parse_profile, DenseConfig};
use crate::pipeline::Pipeline;
use crate::session::{pairs_for, zones_upto, FrameImage, FrameSet, Input, Session, SessionConfig};
use crate::sinks::{DefaultSinks, SinkOptions};
use skyrecon_core::io::{read_gps_file, GpsRecord};
use skyrecon_dense::DepthMapCache;
use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

/// 카메라 폴더(스크립트 순서).
pub const CAMS: [&str; 3] = ["camF", "camR", "camL"];

/// 스트림 설정. 기본값 = 스크립트 동작.
#[derive(Clone, Debug)]
pub struct StreamConfig {
    /// 입력 폴더(`images/<cam>/`, `gps_ref.txt`).
    pub src: PathBuf,
    /// 출력 폴더.
    pub out: PathBuf,
    /// 구역 크기(위치 수).
    pub span: usize,
    /// 구역 겹침(위치 수).
    pub overlap: usize,
    /// 프레임 간격(번호 % stride == 0 만 사용).
    pub stride: usize,
    /// 새로 등록된 영상만 삼각측량(개선, 기본 끔).
    pub incremental_triangulation: bool,
    /// 겹침 영상 깊이맵 재사용(개선, 기본 끔).
    pub depth_cache: bool,
    /// 첫 GPS 줄로 세션 ENU 원점 고정(개선, 기본 끔 = 정렬마다 첫 공통 기록).
    pub fixed_enu_origin: bool,
    /// PatchMatch 백엔드 이름(cuda).
    pub pm_backend: String,
    /// 조밀화 프로파일(fast|quality).
    pub mvs_profile: String,
    /// SIFT 백엔드(cpu|cuda).
    pub sift_backend: String,
    /// 기술자 매칭 백엔드(cpu|cuda).
    pub match_backend: String,
    /// rayon 스레드 수(0 = 코어 수).
    pub threads: usize,
    /// 앞 N 위치만 처리.
    pub max_positions: Option<usize>,
    /// 구역별 희소 모델 저장.
    pub save_models: bool,
    /// 초벌·정밀 조밀화 백엔드 호출 직렬화.
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
    /// 기본 설정(구역 12, 겹침 2, stride 3).
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
    /// 영상 이름 → 위치.
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

    /// 카메라·위치의 영상 이름(`cam/cam_NNNN.jpg`).
    pub fn name(&self, cam: &str, p: usize) -> String {
        format!("{cam}/{cam}_{:04}.jpg", self.frames[p])
    }

    /// 영상 이름의 위치.
    pub fn position(&self, name: &str) -> Option<usize> {
        self.pos_of.get(name).copied()
    }
}

/// 위치 p 의 프레임 묶음(카메라 순서 camF, camR, camL).
pub fn frame_set(layout: &Layout, p: usize) -> FrameSet {
    FrameSet::new(CAMS.iter().map(|c| (c.to_string(), layout.name(c, p))))
}

/// 스크립트와 같은 짝 규칙: 같은 카메라 간격 1..5, 8, 16 / 다른 카메라 위치 차 0..4. 정렬된 이름 짝.
pub fn pairs_for_position(layout: &Layout, p: usize) -> Vec<(String, String)> {
    let frames: Vec<Vec<FrameImage>> = (0..=p).map(|q| frame_set(layout, q).images).collect();
    pairs_for(&frames, p)
}

/// 구역: start = 0, SPAN, …; [start−OVL, start+SPAN+OVL) ∩ [0, NPOS). 완료 위치 = hi−1.
pub fn regions(npos: usize, span: usize, ovl: usize) -> Vec<(usize, usize, usize)> {
    zones_upto(npos, span, ovl).into_iter().map(|z| (z.zone, z.lo, z.hi)).collect()
}

/// 스트림 설정 → 세션 설정. `gps` = 선택된 영상의 GPS 기록(파일 순서), `dense` = None 이면 조밀화 생략.
pub fn session_config(cfg: &StreamConfig, npos: usize, gps: Vec<GpsRecord>, dense: Option<DenseConfig>) -> Result<SessionConfig, String> {
    let mut s = SessionConfig::new(cfg.src.join("images"));
    s.gps = gps;
    s.span = cfg.span;
    s.overlap = cfg.overlap;
    s.total_positions = Some(npos);
    s.incremental_triangulation = cfg.incremental_triangulation;
    s.fixed_enu_origin = cfg.fixed_enu_origin;
    s.seed = cfg.seed;
    s.extraction.sift.max_num_features = cfg.max_num_features;
    s.extraction.reader.max_image_size = cfg.sift_max_image_size;
    s.sift = make_sift_backend(&cfg.sift_backend)?;
    s.matcher = Arc::from(make_match_backend(&cfg.match_backend)?);
    s.dense = dense;
    // CLI 는 되감지 않는다(메모리 절약).
    s.history = 0;
    Ok(s)
}

/// 스트림 실행: 세션 리듀서 + 파이프라인 + 기본 출력 훅. 출력 폴더를 새로 만든다.
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
    let sinks = DefaultSinks::new(&out, &SinkOptions { echo: cfg.echo, save_models: cfg.save_models }).map_err(|e| e.to_string())?;

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

    let dense = (!cfg.no_dense).then(|| {
        let mut dense = DenseConfig::new(backend, profile);
        dense.undistort.max_image_size = cfg.dense_max_image_size;
        dense.densify.neighbors.num_views = cfg.number_views;
        if cfg.serialize_dense {
            dense.lock = Some(Arc::new(Mutex::new(())));
        }
        if cfg.depth_cache {
            dense.depth_cache = Some(Arc::new(DepthMapCache::new()));
        }
        dense
    });
    let session = Session::new(session_config(&cfg, npos, gps_all, dense)?);
    // 출력 훅은 하나(on_any)라 비동기여도 이벤트 순서대로 처리된다.
    let mut pl = Pipeline::new(session).on_any(sinks.into_hook());

    let (span, ovl) = (cfg.span, cfg.overlap);
    let mut reg_end: BTreeMap<usize, Vec<(usize, usize, usize)>> = BTreeMap::new();
    for (k, lo, hi) in regions(npos, span, ovl) {
        reg_end.entry(hi - 1).or_default().push((k, lo, hi));
    }
    pl.push(Input::Log(format!(
        "regions: {}",
        reg_end
            .iter()
            .map(|(e, v)| format!("{e} ->{}", v.iter().map(|(k, lo, hi)| format!(" {k}:{lo}:{hi}")).collect::<String>()))
            .collect::<Vec<_>>()
            .join(", ")
    )));
    pl.push(Input::Log(format!(
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
    )));
    for p in 0..npos {
        pl.push(frame_set(&layout, p).into());
        if let Some(e) = pl.reducer().failed() {
            // 치명 오류: 후처리 없이 멈춘다(훅 큐는 파이프라인을 버릴 때 비워진다).
            return Err(e.to_string());
        }
    }
    let _ = pl.finish();
    Ok(())
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
