//! 매칭 커널 시간 측정: 8192 × 8192 기술자.
use rand::{RngExt, SeedableRng};
use rand_pcg::Pcg64;
use cumulus3d_core::Descriptors;
use cumulus3d_matching::*;
fn main() {
    let mut r = Pcg64::seed_from_u64(1);
    let mk = |r: &mut Pcg64| { let mut d = Descriptors::new(); for _ in 0..8192 { let mut v = [0u8; 128]; for x in v.iter_mut() { *x = r.random_range(0..60u8); } d.push(&v); } d };
    let (a, b) = (mk(&mut r), mk(&mut r));
    let m = CpuMatcher::default();
    for _ in 0..3 {
        let t = std::time::Instant::now();
        let res = m.match_descriptors(&a, &b, &DescriptorMatchOptions::default(), 8192);
        println!("{} matches, {:.1} ms", res.len(), t.elapsed().as_secs_f64() * 1e3);
    }
}
