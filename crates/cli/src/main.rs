//! `skyrecon` 실행 파일.

use clap::{Args, Parser, Subcommand};
use skyrecon_cli::interop::{self, InteropCmd};
use skyrecon_cli::stream::{run_stream, StreamConfig};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "skyrecon", version, about = "한 프로세스 점진 재구성 + 단계별 하위 명령")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// incremental_stream.sh 를 한 프로세스로: 위치 도착 루프 + 구역별 초벌/정밀 조밀화 + 후처리.
    Stream(StreamArgs),
    #[command(flatten)]
    Interop(InteropCmd),
}

#[derive(Args, Debug)]
struct StreamArgs {
    /// 입력 폴더(images/camF|camR|camL/*.jpg, gps_ref.txt).
    #[arg(long)]
    src: PathBuf,
    /// 출력 폴더(지우고 새로 만든다).
    #[arg(long)]
    out: PathBuf,
    #[arg(long, default_value_t = 12)]
    span: usize,
    #[arg(long, default_value_t = 2)]
    overlap: usize,
    /// 파일 번호 % stride == 0 인 프레임만. 위치 = 선택된 프레임 목록의 순번.
    #[arg(long, default_value_t = 3)]
    stride: usize,
    /// 새로 등록된 영상만 삼각측량(결과가 달라질 수 있는 개선).
    #[arg(long)]
    incremental_triangulation: bool,
    /// 겹침 영상의 깊이맵 재사용.
    #[arg(long)]
    depth_cache: bool,
    /// 첫 GPS 줄로 세션 ENU 원점 고정.
    #[arg(long)]
    fixed_enu_origin: bool,
    /// PatchMatch 백엔드(cuda). 장치가 없으면 조밀화 단계에서 오류.
    #[arg(long, default_value = "cuda")]
    pm_backend: String,
    /// 조밀화 프로파일(fast|quality).
    #[arg(long, default_value = "fast")]
    mvs_profile: String,
    /// SIFT 백엔드(cpu|cuda). 결과는 cpu 와 같다.
    #[arg(long, default_value = "cpu")]
    sift_backend: String,
    /// 기술자 매칭 백엔드(cpu|cuda). 결과는 cpu 와 같다.
    #[arg(long, default_value = "cpu")]
    match_backend: String,
    /// --pm-backend/--sift-backend/--match-backend 를 모두 cuda 로.
    #[arg(long)]
    gpu: bool,
    /// rayon 스레드 수(0 = 코어 수).
    #[arg(long, default_value_t = 0)]
    threads: usize,
    /// 앞 N 위치만(빠른 시험용).
    #[arg(long)]
    max_positions: Option<usize>,
    /// 구역별 희소 모델(work/models/*) 저장.
    #[arg(long)]
    save_models: bool,
    /// 초벌·정밀 조밀화 백엔드 호출을 직렬화.
    #[arg(long)]
    serialize_dense: bool,
    /// 왜곡 보정 최대 영상 크기(스크립트 960). 낮추면 조밀화가 빨라진다.
    #[arg(long, default_value_t = 960)]
    dense_max_image_size: i64,
    /// 조밀화 원천 뷰 수(≤ 32).
    #[arg(long, default_value_t = 10)]
    number_views: usize,
    /// SIFT 최대 특징 수(스크립트 8192).
    #[arg(long, default_value_t = 8192)]
    max_num_features: usize,
    /// SIFT 입력 최대 영상 크기(기본 3200).
    #[arg(long, default_value_t = 3200)]
    sift_max_image_size: usize,
    /// 표준 출력에 진행 기록을 보이지 않음(run.log 에만).
    #[arg(long)]
    quiet: bool,
    /// 조밀화 생략(개발·시험용; 후처리 출력 없음).
    #[arg(long)]
    no_dense: bool,
    /// RANSAC 시드 고정(두 뷰 기하·GPS 정렬). 없으면 실행마다 다름.
    #[arg(long)]
    seed: Option<u64>,
}

fn main() {
    let cli = Cli::parse();
    let r = match cli.cmd {
        Cmd::Stream(a) => {
            let mut c = StreamConfig::new(a.src, a.out);
            c.span = a.span;
            c.overlap = a.overlap;
            c.stride = a.stride;
            c.incremental_triangulation = a.incremental_triangulation;
            c.depth_cache = a.depth_cache;
            c.fixed_enu_origin = a.fixed_enu_origin;
            c.mvs_profile = a.mvs_profile;
            c.pm_backend = a.pm_backend;
            c.sift_backend = a.sift_backend;
            c.match_backend = a.match_backend;
            if a.gpu {
                c.pm_backend = "cuda".into();
                c.sift_backend = "cuda".into();
                c.match_backend = "cuda".into();
            }
            c.threads = a.threads;
            c.max_positions = a.max_positions;
            c.save_models = a.save_models;
            c.serialize_dense = a.serialize_dense;
            c.dense_max_image_size = a.dense_max_image_size;
            c.number_views = a.number_views;
            c.max_num_features = a.max_num_features;
            c.sift_max_image_size = a.sift_max_image_size;
            c.echo = !a.quiet;
            c.no_dense = a.no_dense;
            c.seed = a.seed;
            run_stream(c)
        }
        Cmd::Interop(c) => interop::run(c),
    };
    if let Err(e) = r {
        eprintln!("오류: {e}");
        std::process::exit(1);
    }
}
