//! 실영상 SIFT 시간: CpuSift 대 CudaSift(같은 결과인지도 확인).
//! 실행: cargo run --release -p cumulus3d-cuda --example bench_sift -- <영상 폴더> [N=6]

use cumulus3d_cuda::{CudaMatcher, CudaSift};
use cumulus3d_features::{read_gray, CpuSift, SiftEngine, SiftOptions};
use cumulus3d_matching::{CpuMatcher, MatcherBackend, DescriptorMatchOptions};
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let dir = std::path::Path::new(&args[1]);
    let n: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(6);
    let mut files: Vec<_> = std::fs::read_dir(dir).expect("폴더").filter_map(|e| e.ok().map(|e| e.path())).filter(|p| p.extension().is_some_and(|e| e == "jpg")).collect();
    files.sort();
    files.truncate(n);
    let imgs: Vec<_> = files.iter().map(|p| read_gray(p).expect("영상")).collect();
    let o = SiftOptions::default();
    let cpu = CpuSift::new();
    let gpu = CudaSift::try_default().expect("CUDA");
    let _ = gpu.extract(&imgs[0], &o).unwrap();
    let _ = cpu.extract(&imgs[0], &o).unwrap();
    let (mut tc, mut tg) = (0.0, 0.0);
    let mut outs = Vec::new();
    for (p, img) in files.iter().zip(&imgs) {
        let t = Instant::now();
        let c = cpu.extract(img, &o).unwrap();
        let dc = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let g = gpu.extract(img, &o).unwrap();
        let dg = t.elapsed().as_secs_f64();
        tc += dc;
        tg += dg;
        let same = c.features == g.features && c.descriptors.as_slice() == g.descriptors.as_slice();
        eprintln!("{} {}×{}: CPU {:.0} ms, GPU {:.0} ms, 특징 {} / {}, 같음 {same}", p.display(), img.width, img.height, dc * 1e3, dg * 1e3, c.len(), g.len());
        outs.push(g);
    }
    eprintln!("평균: CPU {:.0} ms, GPU {:.0} ms", tc / n as f64 * 1e3, tg / n as f64 * 1e3);

    // 매칭(연속 쌍).
    let cm = CpuMatcher::default();
    let gm = CudaMatcher::try_default().expect("CUDA");
    let mo = DescriptorMatchOptions::default();
    let _ = gm.match_descriptors(&outs[0].descriptors, &outs[1].descriptors, &mo, 8192);
    let (mut tc, mut tg) = (0.0, 0.0);
    for k in 0..outs.len() - 1 {
        let t = Instant::now();
        let a = cm.match_descriptors(&outs[k].descriptors, &outs[k + 1].descriptors, &mo, 8192);
        tc += t.elapsed().as_secs_f64();
        let t = Instant::now();
        let b = gm.match_descriptors(&outs[k].descriptors, &outs[k + 1].descriptors, &mo, 8192);
        tg += t.elapsed().as_secs_f64();
        assert_eq!(a, b);
    }
    let m = (outs.len() - 1) as f64;
    eprintln!("매칭 쌍당 평균: CPU {:.1} ms, GPU {:.1} ms (결과 동일)", tc / m * 1e3, tg / m * 1e3);
}
