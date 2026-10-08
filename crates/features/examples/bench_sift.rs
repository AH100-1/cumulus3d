//! SIFT 처리 시간 측정: `cargo run --release -p skyrecon-features --example bench_sift -- <영상|synthetic> [W H]`
//! W H 를 주면 그 크기로 재표본화한 뒤 측정한다.

use skyrecon_features::*;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let src = args.get(1).cloned().unwrap_or_else(|| "synthetic".into());
    let size: Option<(usize, usize)> = match (args.get(2), args.get(3)) {
        (Some(w), Some(h)) => Some((w.parse().expect("W"), h.parse().expect("H"))),
        _ => None,
    };
    let t = Instant::now();
    let mut g = if src == "synthetic" {
        let (w, h) = size.unwrap_or((2048, 1152));
        let mut s = 1u64;
        let mut noise = vec![0f32; w * h];
        for v in noise.iter_mut() {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            *v = (s >> 40) as f32 / (1u64 << 24) as f32;
        }
        let mut out = vec![0f32; w * h];
        let mut tmp = vec![0f32; w * h];
        sift::pyramid::gaussian_blur(&noise, &mut out, &mut tmp, w, h, 2.0);
        GrayImage::from_f32(w, h, |x, y| (out[y * w + x] - 0.5) * 3.0 + 0.5)
    } else {
        read_gray(std::path::Path::new(&src)).expect("영상 읽기")
    };
    let t_read = t.elapsed();
    if let Some((w, h)) = size {
        if (w, h) != (g.width, g.height) {
            g = g.resized(w, h);
        }
    }
    println!("입력 {} ({}x{}), 읽기 {:.1} ms, 스레드 {}", src, g.width, g.height, t_read.as_secs_f64() * 1e3, rayon::current_num_threads());
    let sift = CpuSift::new();
    for (name, opts) in [
        ("compat(기본)", SiftOptions::default()),
        ("무제한", SiftOptions { max_num_features: 0, ..Default::default() }),
    ] {
        let mut times = Vec::new();
        let mut n = 0;
        for _ in 0..5 {
            let t = Instant::now();
            let out = sift.extract(&g, &opts).expect("추출");
            times.push(t.elapsed().as_secs_f64() * 1e3);
            n = out.len();
        }
        let first = times[0];
        let mut rest = times[1..].to_vec();
        rest.sort_by(|a, b| a.total_cmp(b));
        println!("{name}: 특징 {n}, 첫 실행 {first:.1} ms, 이후 중앙값 {:.1} ms", rest[rest.len() / 2]);
    }
}
