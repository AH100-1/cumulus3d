//! 외부 SfM 도구(COLMAP) 단계별 명령 호환 하위 명령(얇은 래퍼). 하위 명령 이름과 옵션 문자열은 그 도구의 명령줄을 따른다.
//! 단계 사이 상태는 `--database_path` (FeatureStore 이진 파일)와 모델 폴더([`skyrecon_core::interop`] 형식)로 잇는다.
//! GPU 관련 옵션(`--FeatureExtraction.use_gpu` 등)은 받기만 하고 무시한다.

use crate::densewrap::{dense_model, make_pm_backend, parse_profile, DenseConfig};
use clap::{Args, Subcommand};
use skyrecon_align::{align_to_gps, ModelAlignerOptions};
use skyrecon_ba::{bundle_adjust, BaConfig};
use skyrecon_core::analyzer::analyzer_lines;
use skyrecon_core::interop::{read_model, write_model_binary, write_model_text, ImageOrder};
use skyrecon_core::io::{read_gps_file, write_ply, PointCloud, PlyLayout};
use skyrecon_core::reconstruction::bilinear_rgb;
use skyrecon_core::{CameraModelKind, MatchGraph, MatchGraphOptions, FeatureStore, ImageId, Reconstruction};
use skyrecon_dense::{densify, write_undistorted_workspace, DenseScene, UndistortCache, UndistortOptions};
use skyrecon_features::{CameraMode, ExtractionOptions, FeatureExtractor, ImageStatus};
use skyrecon_matching::{match_pair_list_file, CpuMatcher, PairMatchingOptions};
use skyrecon_sfm::{global_mapper, register_images, triangulate_points, GlobalSfmOptions, PointTriangulatorOptions, RegistrationOptions};
use std::path::{Path, PathBuf};
use std::time::Instant;

type R = Result<(), String>;

fn e<E: std::fmt::Display>(x: E) -> String {
    x.to_string()
}

/// `1/0/true/false` 를 받는다(명령줄 관례).
pub fn parse_flag(s: &str) -> Result<bool, String> {
    match s.to_ascii_lowercase().as_str() {
        "1" | "true" | "yes" | "on" => Ok(true),
        "0" | "false" | "no" | "off" => Ok(false),
        _ => Err(format!("불린 값이 아님: {s}")),
    }
}

#[derive(Subcommand, Debug)]
pub enum InteropCmd {
    #[command(name = "feature_extractor")]
    FeatureExtractor(FeatureExtractorArgs),
    #[command(name = "matches_importer")]
    MatchesImporter(MatchesImporterArgs),
    #[command(name = "global_mapper")]
    GlobalMapper(GlobalMapperArgs),
    #[command(name = "image_registrator")]
    ImageRegistrator(ImageRegistratorArgs),
    #[command(name = "point_triangulator")]
    PointTriangulator(PointTriangulatorArgs),
    #[command(name = "bundle_adjuster")]
    BundleAdjuster(BundleAdjusterArgs),
    #[command(name = "model_aligner")]
    ModelAligner(ModelAlignerArgs),
    #[command(name = "model_analyzer")]
    ModelAnalyzer(ModelAnalyzerArgs),
    #[command(name = "model_converter")]
    ModelConverter(ModelConverterArgs),
    #[command(name = "image_deleter")]
    ImageDeleter(ImageDeleterArgs),
    #[command(name = "image_undistorter")]
    ImageUndistorter(ImageUndistorterArgs),
    /// 조밀화: image_undistorter 출력 폴더 → dense.ply.
    #[command(name = "densify")]
    Densify(Box<DensifyArgs>),
}

#[derive(Args, Debug)]
pub struct FeatureExtractorArgs {
    #[arg(long = "database_path")]
    pub database_path: PathBuf,
    #[arg(long = "image_path")]
    pub image_path: PathBuf,
    #[arg(long = "image_list_path")]
    pub image_list_path: Option<PathBuf>,
    #[arg(long = "ImageReader.camera_model", default_value = "SIMPLE_RADIAL")]
    pub camera_model: String,
    #[arg(long = "ImageReader.single_camera", default_value = "0", value_parser = parse_flag, action = clap::ArgAction::Set)]
    pub single_camera: bool,
    #[arg(long = "ImageReader.single_camera_per_folder", default_value = "0", value_parser = parse_flag, action = clap::ArgAction::Set)]
    pub single_camera_per_folder: bool,
    #[arg(long = "ImageReader.existing_camera_id")]
    pub existing_camera_id: Option<i64>,
    #[arg(long = "ImageReader.camera_params")]
    pub camera_params: Option<String>,
    #[arg(long = "ImageReader.default_focal_length_factor", default_value_t = 1.2)]
    pub default_focal_length_factor: f64,
    #[arg(long = "SiftExtraction.max_num_features", default_value_t = 8192)]
    pub max_num_features: usize,
    #[arg(long = "SiftExtraction.max_image_size", default_value_t = 3200)]
    pub max_image_size: usize,
    #[arg(long = "FeatureExtraction.use_gpu", value_parser = parse_flag)]
    pub use_gpu: Option<bool>,
    #[arg(long = "SiftExtraction.use_gpu", value_parser = parse_flag)]
    pub sift_use_gpu: Option<bool>,
}

#[derive(Args, Debug)]
pub struct MatchesImporterArgs {
    #[arg(long = "database_path")]
    pub database_path: PathBuf,
    #[arg(long = "match_list_path")]
    pub match_list_path: PathBuf,
    #[arg(long = "match_type", default_value = "pairs")]
    pub match_type: String,
    #[arg(long = "FeatureMatching.use_gpu", value_parser = parse_flag)]
    pub use_gpu: Option<bool>,
    #[arg(long = "SiftMatching.use_gpu", value_parser = parse_flag)]
    pub sift_use_gpu: Option<bool>,
    #[arg(long = "SiftMatching.max_num_matches", default_value_t = 32768)]
    pub max_num_matches: usize,
}

#[derive(Args, Debug)]
pub struct GlobalMapperArgs {
    #[arg(long = "database_path")]
    pub database_path: PathBuf,
    #[arg(long = "image_path")]
    pub image_path: Option<PathBuf>,
    #[arg(long = "output_path")]
    pub output_path: PathBuf,
    #[arg(long = "GlobalMapper.ba_num_iterations")]
    pub ba_num_iterations: Option<usize>,
    #[arg(long = "GlobalMapper.skip_retriangulation", value_parser = parse_flag)]
    pub skip_retriangulation: Option<bool>,
    #[arg(long = "GlobalMapper.keep_max_num_tracks")]
    pub keep_max_num_tracks: Option<usize>,
}

#[derive(Args, Debug)]
pub struct ImageRegistratorArgs {
    #[arg(long = "database_path")]
    pub database_path: PathBuf,
    #[arg(long = "input_path")]
    pub input_path: PathBuf,
    #[arg(long = "output_path")]
    pub output_path: PathBuf,
}

#[derive(Args, Debug)]
pub struct PointTriangulatorArgs {
    #[arg(long = "database_path")]
    pub database_path: PathBuf,
    #[arg(long = "image_path")]
    pub image_path: Option<PathBuf>,
    #[arg(long = "input_path")]
    pub input_path: PathBuf,
    #[arg(long = "output_path")]
    pub output_path: PathBuf,
    #[arg(long = "clear_points", default_value = "1", value_parser = parse_flag, action = clap::ArgAction::Set)]
    pub clear_points: bool,
}

#[derive(Args, Debug)]
pub struct BundleAdjusterArgs {
    #[arg(long = "input_path")]
    pub input_path: PathBuf,
    #[arg(long = "output_path")]
    pub output_path: PathBuf,
    #[arg(long = "BundleAdjustment.max_num_iterations", default_value_t = 100)]
    pub max_num_iterations: usize,
    #[arg(long = "BundleAdjustment.refine_focal_length", default_value = "1", value_parser = parse_flag, action = clap::ArgAction::Set)]
    pub refine_focal_length: bool,
    #[arg(long = "BundleAdjustment.refine_principal_point", default_value = "0", value_parser = parse_flag, action = clap::ArgAction::Set)]
    pub refine_principal_point: bool,
    #[arg(long = "BundleAdjustment.refine_extra_params", default_value = "1", value_parser = parse_flag, action = clap::ArgAction::Set)]
    pub refine_extra_params: bool,
}

#[derive(Args, Debug)]
pub struct ModelAlignerArgs {
    #[arg(long = "input_path")]
    pub input_path: PathBuf,
    #[arg(long = "output_path")]
    pub output_path: PathBuf,
    #[arg(long = "ref_images_path")]
    pub ref_images_path: PathBuf,
    #[arg(long = "ref_is_gps", default_value = "1", value_parser = parse_flag, action = clap::ArgAction::Set)]
    pub ref_is_gps: bool,
    #[arg(long = "alignment_type", default_value = "enu")]
    pub alignment_type: String,
    #[arg(long = "alignment_max_error", default_value_t = 0.0)]
    pub alignment_max_error: f64,
    #[arg(long = "min_common_images", default_value_t = 3)]
    pub min_common_images: usize,
}

#[derive(Args, Debug)]
pub struct ModelAnalyzerArgs {
    #[arg(long = "path")]
    pub path: PathBuf,
    #[arg(long = "verbose", default_value = "0", value_parser = parse_flag, action = clap::ArgAction::Set)]
    pub verbose: bool,
}

#[derive(Args, Debug)]
pub struct ModelConverterArgs {
    #[arg(long = "input_path")]
    pub input_path: PathBuf,
    #[arg(long = "output_path")]
    pub output_path: PathBuf,
    #[arg(long = "output_type")]
    pub output_type: String,
}

#[derive(Args, Debug)]
pub struct ImageDeleterArgs {
    #[arg(long = "input_path")]
    pub input_path: PathBuf,
    #[arg(long = "output_path")]
    pub output_path: PathBuf,
    #[arg(long = "image_ids_path")]
    pub image_ids_path: Option<PathBuf>,
    #[arg(long = "image_names_path")]
    pub image_names_path: Option<PathBuf>,
}

#[derive(Args, Debug)]
pub struct ImageUndistorterArgs {
    #[arg(long = "image_path")]
    pub image_path: PathBuf,
    #[arg(long = "input_path")]
    pub input_path: PathBuf,
    #[arg(long = "output_path")]
    pub output_path: PathBuf,
    #[arg(long = "output_type", default_value = "COLMAP")]
    pub output_type: String,
    #[arg(long = "max_image_size", default_value_t = -1)]
    pub max_image_size: i64,
}

#[derive(Args, Debug)]
pub struct DensifyArgs {
    /// image_undistorter 출력 폴더(images/, sparse/). `--image_path` 를 주면 대신 왜곡 있는 모델(`--input_path`)을 메모리에서 보정.
    #[arg(long = "input_path", short = 'i')]
    pub input_path: PathBuf,
    /// 출력 PLY.
    #[arg(long = "output_path", short = 'o')]
    pub output_path: PathBuf,
    /// 주면 `input_path` 를 왜곡 있는 입력 모델로 보고 이 폴더의 영상을 메모리에서 보정한다.
    #[arg(long = "image_path")]
    pub image_path: Option<PathBuf>,
    #[arg(long = "max_image_size", default_value_t = 960)]
    pub max_image_size: i64,
    /// 기준 뷰당 원천 뷰 수(≤ 32).
    #[arg(long = "number-views", default_value_t = 10)]
    pub number_views: usize,
    /// PatchMatch 백엔드(cuda).
    #[arg(long = "pm-backend", default_value = "cuda")]
    pub pm_backend: String,
    /// 프로파일(fast|quality).
    #[arg(long = "mvs-profile", default_value = "fast")]
    pub mvs_profile: String,
    /// 비용 창 반경(기본: 프로파일 값 5).
    #[arg(long = "window-radius")]
    pub window_radius: Option<i32>,
    /// 비용 창 표본 간격 1|2(기본: fast 2, quality 1).
    #[arg(long = "window-step")]
    pub window_step: Option<i32>,
    /// 스케일 수 상한.
    #[arg(long = "max-levels")]
    pub max_levels: Option<usize>,
    /// 등록 순서로 앞 N 장만(`--image_path` 사용 시).
    #[arg(long = "max-images")]
    pub max_images: Option<usize>,
    /// 융합 방식(consistency|traversal).
    #[arg(long = "fusion-mode")]
    pub fusion_mode: Option<String>,
    /// 일치 융합: 기준 외 일치 뷰 최소 수.
    #[arg(long = "fusion-min-views")]
    pub fusion_min_views: Option<usize>,
    /// 융합 상대 깊이 허용치.
    #[arg(long = "fusion-depth-error")]
    pub fusion_depth_error: Option<f64>,
    /// 융합 법선 허용 각(도).
    #[arg(long = "fusion-normal-error")]
    pub fusion_normal_error: Option<f64>,
    /// 융합 재투영 허용치(픽셀).
    #[arg(long = "fusion-reproj-error")]
    pub fusion_reproj_error: Option<f64>,
    /// 일치 융합: 쓰인 픽셀 표시(1|0).
    #[arg(long = "fusion-mark-used", value_parser = parse_flag)]
    pub fusion_mark_used: Option<bool>,
    /// 일치 융합: 표시 반경(픽셀).
    #[arg(long = "fusion-mark-radius")]
    pub fusion_mark_radius: Option<usize>,
    /// 일치 융합: 검사할 겹침 뷰 수.
    #[arg(long = "fusion-num-images")]
    pub fusion_num_images: Option<usize>,
    /// 모든 스케일의 기하 실행 반복 수 덮어쓰기.
    #[arg(long = "geo-iters")]
    pub geo_iters: Option<u32>,
    /// 기하 실행 회차 수 덮어쓰기.
    #[arg(long = "geo-rounds")]
    pub geo_rounds: Option<u32>,
    /// 일치 융합: 역분산 가중(1|0).
    #[arg(long = "fusion-inverse-variance", value_parser = parse_flag)]
    pub fusion_inverse_variance: Option<bool>,
    /// 깊이맵 필터: 일치 뷰 최소 수.
    #[arg(long = "filter-min-views")]
    pub filter_min_views: Option<u32>,
    /// 깊이맵 필터: 순·역 재투영 허용치(픽셀).
    #[arg(long = "filter-geom-error")]
    pub filter_geom_error: Option<f32>,
    /// 깊이맵 필터: 최소 NCC.
    #[arg(long = "filter-min-ncc")]
    pub filter_min_ncc: Option<f32>,
    /// 작은 조각 제거(1|0).
    #[arg(long = "remove-speckles", value_parser = parse_flag)]
    pub remove_speckles: Option<bool>,
    /// 경계 인식 틈 메우기(1|0).
    #[arg(long = "fill-holes", value_parser = parse_flag)]
    pub fill_holes: Option<bool>,
    /// 이웃 선택 방향 다양성 감쇠(1 = 끔).
    #[arg(long = "diversity-decay")]
    pub diversity_decay: Option<f64>,
    /// 거친 스케일 후보를 더할 기준 패치 분산 문턱(0 = 끔).
    #[arg(long = "weak-texture-var")]
    pub weak_texture_var: Option<f32>,
    /// 판독 전 5×5 중앙값 평면 필터(1|0).
    #[arg(long = "median-filter", value_parser = parse_flag)]
    pub median_filter: Option<bool>,
    /// 결과 점군 통계(간격·이상점·중복) 출력.
    #[arg(long = "stats")]
    pub stats: bool,
}

/// 조밀화 하위 명령 옵션을 설정에 반영.
fn apply_densify_args(a: &DensifyArgs, o: &mut skyrecon_dense::DensifyOptions) -> R {
    if let Some(r) = a.window_radius {
        o.pm.window_radius = r;
    }
    if let Some(st) = a.window_step {
        o.pm.window_step = st;
    }
    if let Some(l) = a.max_levels {
        o.max_levels = l;
    }
    if let Some(m) = &a.fusion_mode {
        o.fusion.mode = skyrecon_dense::FusionMode::parse(m).ok_or_else(|| format!("알 수 없는 --fusion-mode {m}"))?;
    }
    let f = &mut o.fusion;
    if let Some(v) = a.fusion_min_views {
        f.min_consistent_views = v;
    }
    if let Some(v) = a.fusion_depth_error {
        f.max_depth_error = v;
    }
    if let Some(v) = a.fusion_normal_error {
        f.max_normal_error_deg = v;
    }
    if let Some(v) = a.fusion_reproj_error {
        f.max_reproj_error = v;
    }
    if let Some(v) = a.fusion_mark_used {
        f.mark_used = v;
    }
    if let Some(v) = a.fusion_mark_radius {
        f.mark_radius = v;
    }
    if let Some(v) = a.fusion_num_images {
        f.consistency_num_images = v;
    }
    if let Some(v) = a.geo_iters {
        o.geometric_iters_override = Some(v);
    }
    if let Some(v) = a.geo_rounds {
        o.geometric_rounds_override = Some(v);
    }
    if let Some(v) = a.fusion_inverse_variance {
        f.inverse_variance = v;
    }
    if let Some(v) = a.filter_min_views {
        o.filter.min_num_consistent = v;
    }
    if let Some(v) = a.filter_geom_error {
        o.filter.geom_max_cost = v;
    }
    if let Some(v) = a.filter_min_ncc {
        o.filter.min_ncc = v;
    }
    if let Some(v) = a.remove_speckles {
        o.post.remove_speckles = v;
    }
    if let Some(v) = a.fill_holes {
        o.post.fill_holes = v;
    }
    if let Some(v) = a.diversity_decay {
        o.neighbors.diversity_decay = v;
    }
    if let Some(v) = a.weak_texture_var {
        o.pm.weak_texture_var = v;
    }
    if let Some(v) = a.median_filter {
        o.filter.median_filter = v;
    }
    Ok(())
}

fn load_store(p: &Path) -> Result<FeatureStore, String> {
    if p.exists() {
        FeatureStore::load(p).map_err(|x| format!("{}: {x}", p.display()))
    } else {
        Ok(FeatureStore::new())
    }
}

fn load_model(p: &Path) -> Result<Reconstruction, String> {
    read_model(p).map_err(|x| format!("{}: {x}", p.display()))
}

fn save_model(rec: &Reconstruction, p: &Path) -> R {
    std::fs::create_dir_all(p).map_err(e)?;
    write_model_binary(rec, p, ImageOrder::Registration).map_err(e)
}

fn graph_of(store: &FeatureStore) -> MatchGraph {
    MatchGraph::from_store(store, &MatchGraphOptions::default())
}

/// 영상 폴더에서 3D 점 색 추출(모델을 쓰는 명령들의 공통 마무리 단계).
pub fn extract_colors(rec: &mut Reconstruction, image_path: &Path) {
    rec.extract_colors(|im| {
        let img = image::open(image_path.join(&im.name)).ok()?.to_rgb8();
        let (w, h) = (img.width() as usize, img.height() as usize);
        let data = img.into_raw();
        Some(move |x: f64, y: f64| bilinear_rgb(&data, w, h, x, y))
    });
}

fn list_images(root: &Path) -> Vec<String> {
    fn walk(dir: &Path, root: &Path, out: &mut Vec<String>) {
        let Ok(rd) = std::fs::read_dir(dir) else { return };
        for ent in rd.flatten() {
            let p = ent.path();
            if p.is_dir() {
                walk(&p, root, out);
            } else if let Some(ext) = p.extension().and_then(|x| x.to_str()) {
                if matches!(ext.to_ascii_lowercase().as_str(), "jpg" | "jpeg" | "png") {
                    if let Ok(rel) = p.strip_prefix(root) {
                        out.push(rel.components().map(|c| c.as_os_str().to_string_lossy()).collect::<Vec<_>>().join("/"));
                    }
                }
            }
        }
    }
    let mut v = Vec::new();
    walk(root, root, &mut v);
    v.sort();
    v
}

pub fn run(cmd: InteropCmd) -> R {
    let t = Instant::now();
    match cmd {
        InteropCmd::FeatureExtractor(a) => {
            let store = load_store(&a.database_path)?;
            let names = match &a.image_list_path {
                Some(p) => skyrecon_features::read_image_list(p).map_err(e)?,
                None => list_images(&a.image_path),
            };
            let mut o = ExtractionOptions::default();
            o.reader.camera_model = CameraModelKind::from_name(&a.camera_model).map_err(e)?;
            o.reader.camera_mode = if let Some(id) = a.existing_camera_id.filter(|x| *x >= 0) {
                CameraMode::Existing(id as u32)
            } else if a.single_camera {
                CameraMode::Single
            } else if a.single_camera_per_folder {
                CameraMode::PerFolder
            } else {
                CameraMode::PerImage
            };
            if let Some(p) = &a.camera_params {
                o.reader.camera_params = Some(p.split(',').map(|x| x.trim().parse::<f64>().map_err(e)).collect::<Result<_, _>>()?);
            }
            o.reader.default_focal_length_factor = a.default_focal_length_factor;
            o.reader.max_image_size = a.max_image_size;
            o.sift.max_num_features = a.max_num_features;
            let rep = FeatureExtractor::new().extract_files(&store, &a.image_path, &names, &o).map_err(e)?;
            for r in &rep {
                match &r.status {
                    ImageStatus::Extracted { image_id, camera_id, num_features } => {
                        println!("{} image_id={image_id} camera_id={camera_id} features={num_features}", r.name)
                    }
                    ImageStatus::AlreadyExists { image_id } => println!("{} image_id={image_id} (이미 있음)", r.name),
                    ImageStatus::Failed { error } => println!("{} 실패: {error}", r.name),
                }
            }
            store.save(&a.database_path).map_err(e)?;
        }
        InteropCmd::MatchesImporter(a) => {
            if a.match_type != "pairs" {
                return Err(format!("--match_type {} 미지원(pairs 만)", a.match_type));
            }
            let store = load_store(&a.database_path)?;
            let opts = PairMatchingOptions { max_num_matches: a.max_num_matches, ..Default::default() };
            let (list, st) = match_pair_list_file(&store, &a.match_list_path, &opts, &CpuMatcher::default()).map_err(e)?;
            for n in &list.missing_names {
                println!("경고: 이름 없음 {n}");
            }
            println!(
                "짝 {} 건너뜀 {} 매칭 {} 검증만 {} 유효 기하 {}",
                list.pairs.len(),
                st.num_skipped_existing,
                st.num_matched,
                st.num_verified_only,
                st.num_valid_geometries
            );
            store.save(&a.database_path).map_err(e)?;
        }
        InteropCmd::GlobalMapper(a) => {
            let store = load_store(&a.database_path)?;
            let graph = graph_of(&store);
            let mut o = GlobalSfmOptions::default();
            if let Some(n) = a.ba_num_iterations {
                o.ba_num_iterations = n;
            }
            if let Some(s) = a.skip_retriangulation {
                o.skip_retriangulation = s;
            }
            if let Some(n) = a.keep_max_num_tracks {
                o.tracks.max_num_tracks = n;
            }
            let out = global_mapper(&store, &graph, &o).map_err(e)?;
            if let Some(f) = &out.failure {
                println!("경고: {f}");
            }
            let mut rec = out.reconstruction;
            if let Some(ip) = &a.image_path {
                extract_colors(&mut rec, ip);
            }
            if rec.registered_image_count() > 0 {
                save_model(&rec, &a.output_path.join("0"))?;
            }
            println!("등록 {} 점 {}", rec.registered_image_count(), rec.num_points3d());
        }
        InteropCmd::ImageRegistrator(a) => {
            let store = load_store(&a.database_path)?;
            let graph = graph_of(&store);
            let mut rec = load_model(&a.input_path)?;
            let rep = register_images(&mut rec, &store, &graph, &RegistrationOptions::default()).map_err(e)?;
            println!("시도 {} 등록 {}", rep.attempts.len(), rep.registered().len());
            save_model(&rec, &a.output_path)?;
        }
        InteropCmd::PointTriangulator(a) => {
            let store = load_store(&a.database_path)?;
            let graph = graph_of(&store);
            let mut rec = load_model(&a.input_path)?;
            let o = PointTriangulatorOptions { clear_points: a.clear_points, ..Default::default() };
            let rep = triangulate_points(&mut rec, &graph, &o).map_err(e)?;
            if let Some(ip) = &a.image_path {
                extract_colors(&mut rec, ip);
            }
            println!("{rep:?}");
            save_model(&rec, &a.output_path)?;
        }
        InteropCmd::BundleAdjuster(a) => {
            let mut rec = load_model(&a.input_path)?;
            let cfg = BaConfig {
                max_num_iterations: a.max_num_iterations,
                refine_focal_length: a.refine_focal_length,
                refine_principal_point: a.refine_principal_point,
                refine_extra_params: a.refine_extra_params,
                ..Default::default()
            };
            let s = bundle_adjust(&mut rec, &cfg).map_err(e)?;
            println!("반복 {} 비용 {:.6e} → {:.6e} {:?}", s.num_iterations, s.initial_cost, s.final_cost, s.termination);
            save_model(&rec, &a.output_path)?;
        }
        InteropCmd::ModelAligner(a) => {
            if !a.ref_is_gps || a.alignment_type != "enu" {
                return Err("model_aligner: --ref_is_gps 1 --alignment_type enu 만 지원".into());
            }
            let mut rec = load_model(&a.input_path)?;
            let gps = read_gps_file(&a.ref_images_path).map_err(e)?;
            let o = ModelAlignerOptions { max_error: a.alignment_max_error, min_common_images: a.min_common_images, ..Default::default() };
            let al = align_to_gps(&mut rec, &gps, &o).map_err(e)?;
            for (n, er) in &al.errors {
                println!("{n}: {er:.4}");
            }
            println!("평균 오차 {:.4} 중앙 {:.4} 인라이어 {}/{}", al.mean_error, al.median_error, al.num_inliers, al.common.len());
            save_model(&rec, &a.output_path)?;
        }
        InteropCmd::ModelAnalyzer(a) => {
            let rec = load_model(&a.path)?;
            for l in analyzer_lines(&rec, a.verbose) {
                println!("{l}");
            }
            return Ok(());
        }
        InteropCmd::ModelConverter(a) => {
            let rec = load_model(&a.input_path)?;
            match a.output_type.to_ascii_uppercase().as_str() {
                "BIN" => save_model(&rec, &a.output_path)?,
                "TXT" => {
                    std::fs::create_dir_all(&a.output_path).map_err(e)?;
                    write_model_text(&rec, &a.output_path, ImageOrder::Registration).map_err(e)?
                }
                "PLY" => {
                    let mut c = PointCloud::default();
                    for (_, p) in rec.points3d() {
                        c.positions.push([p.xyz.x as f32, p.xyz.y as f32, p.xyz.z as f32]);
                        c.normals.push([0.0; 3]);
                        c.colors.push(p.color);
                    }
                    write_ply(&a.output_path, &c, PlyLayout::XyzNormalRgb).map_err(e)?
                }
                o => return Err(format!("--output_type {o} 미지원(BIN|TXT|PLY)")),
            }
        }
        InteropCmd::ImageDeleter(a) => {
            let mut rec = load_model(&a.input_path)?;
            let mut warn = Vec::new();
            if let Some(p) = &a.image_ids_path {
                let ids: Vec<ImageId> = std::fs::read_to_string(p).map_err(e)?.split_whitespace().filter_map(|x| x.parse().ok()).collect();
                warn.extend(rec.deregister_images_by_id(&ids));
            }
            if let Some(p) = &a.image_names_path {
                let text = std::fs::read_to_string(p).map_err(e)?;
                let names: Vec<&str> = text.lines().map(|l| l.trim()).filter(|l| !l.is_empty()).collect();
                warn.extend(rec.deregister_images_by_name(&names));
            }
            for w in warn {
                println!("{w}");
            }
            save_model(&rec, &a.output_path)?;
        }
        InteropCmd::ImageUndistorter(a) => {
            if a.output_type != "COLMAP" {
                return Err("image_undistorter: --output_type COLMAP 만 지원".into());
            }
            let rec = load_model(&a.input_path)?;
            let o = UndistortOptions { max_image_size: a.max_image_size, ..Default::default() };
            let r = skyrecon_dense::undistort_from_dir(&rec, &a.image_path, &o, &UndistortCache::new()).map_err(e)?;
            write_undistorted_workspace(&r, &a.output_path, &o).map_err(e)?;
            println!("보정 {} 실패 {}", r.images.len(), r.failed.len());
        }
        InteropCmd::Densify(a) => {
            let profile = parse_profile(&a.mvs_profile)?;
            let mut cfg = DenseConfig::new(make_pm_backend(&a.pm_backend), profile);
            cfg.undistort.max_image_size = a.max_image_size;
            cfg.densify.neighbors.num_views = a.number_views;
            apply_densify_args(&a, &mut cfg.densify)?;
            let backend = cfg.backend.clone()?;
            let tl = Instant::now();
            let (out, frames, scene) = match &a.image_path {
                Some(ip) => {
                    let model = load_model(&a.input_path)?;
                    let keep: std::collections::HashSet<String> = model
                        .registered_images()
                        .into_iter()
                        .take(a.max_images.unwrap_or(usize::MAX))
                        .filter_map(|i| model.image(i).map(|im| im.name.clone()))
                        .collect();
                    let r = dense_model(&model, |n| keep.contains(n), ip, &cfg)?;
                    eprintln!("[time] 왜곡 보정+장면 {:.2}s", r.undistort_time.as_secs_f64());
                    (r.output, r.frames, r.scene)
                }
                None => {
                    let scene = DenseScene::from_workspace_dir(&a.input_path, &cfg.scene).map_err(e)?;
                    eprintln!("[time] 장면 읽기 {:.2}s", tl.elapsed().as_secs_f64());
                    let n = scene.views.len();
                    (densify(&scene, &cfg.densify, backend.as_ref(), None).map_err(e)?, n, scene)
                }
            };
            let tm = &out.timings;
            let lv: Vec<String> = tm.levels.iter().map(|d| format!("{:.2}", d.as_secs_f64())).collect();
            eprintln!(
                "[time] 뷰 {frames}, 깊이맵 {}: 이웃 {:.2}s 준비 {:.2}s 깊이 {:.2}s (스케일별 [{}], 그중 상향표본·판정 {:.2}s) 필터 {:.2}s 융합 {:.2}s 합 {:.2}s",
                out.depth_views,
                tm.neighbors.as_secs_f64(),
                tm.prepare.as_secs_f64(),
                tm.depth().as_secs_f64(),
                lv.join(", "),
                tm.upsample.as_secs_f64(),
                tm.filter.as_secs_f64(),
                tm.fusion.as_secs_f64(),
                tm.total().as_secs_f64()
            );
            let tw = Instant::now();
            out.write_ply(&a.output_path).map_err(e)?;
            eprintln!("[time] PLY 쓰기 {:.2}s", tw.elapsed().as_secs_f64());
            println!("점 {}", out.cloud.len());
            if a.stats {
                let ts = Instant::now();
                let st = skyrecon_dense::cloud_stats(&scene, &out.depth_maps, &out.cloud, &out.visibility, 200_000, 2.0);
                println!(
                    "통계: 점 {} 표본 {} 이웃 간격 중앙 {:.4} GSD {:.4} 이상점(2px) {:.2}% 평면 이탈(>GSD·고립) {:.2}% 평면 거리 중앙 {:.4} 중복(<GSD/2) {:.2}% ({:.1}s)",
                    st.points,
                    st.sampled,
                    st.nn_spacing_median,
                    st.gsd,
                    100.0 * st.outlier_ratio,
                    100.0 * st.plane_outlier_ratio,
                    st.plane_residual_median,
                    100.0 * st.duplicate_ratio,
                    ts.elapsed().as_secs_f64()
                );
            }
        }
    }
    eprintln!("Elapsed time: {:.3} [minutes]", t.elapsed().as_secs_f64() / 60.0);
    Ok(())
}
