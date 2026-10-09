//! 개발용: 모델에 BA 를 여러 설정으로 돌려 스케일·뒤쪽 관측 수를 본다.
use cumulus3d_ba::{bundle_adjust, BaConfig};
use cumulus3d_core::interop::read_model;
use cumulus3d_core::Reconstruction;

fn stats(rec: &Reconstruction) -> (f64, usize, usize) {
    let ids = rec.registered_images();
    let cs: Vec<_> = ids.iter().filter_map(|i| rec.projection_center(*i)).collect();
    let m = cs.iter().fold(cumulus3d_core::Vec3::zeros(), |a, c| a + c) / cs.len() as f64;
    let spread = (cs.iter().map(|c| (c - m).norm_squared()).sum::<f64>() / cs.len() as f64).sqrt();
    let mut behind = 0;
    let mut total = 0;
    for (_, p) in rec.points3d() {
        for t in &p.track {
            let pose = rec.world_to_cam(t.image_id).unwrap();
            total += 1;
            if pose.transform_point(&p.xyz).z <= 1e-9 {
                behind += 1;
            }
        }
    }
    (spread, behind, total)
}

fn main() {
    let path = std::env::args().nth(1).expect("모델 경로");
    let rec0 = read_model(&path).unwrap();
    println!("입력: {:?} 오차 {:.3}", stats(&rec0), rec0.mean_reproj_error());
    let cfgs: Vec<(&str, BaConfig)> = vec![
        ("기본", BaConfig::default()),
        ("초점·왜곡 고정", BaConfig { refine_focal_length: false, refine_extra_params: false, ..Default::default() }),
        ("왜곡 고정", BaConfig { refine_extra_params: false, ..Default::default() }),
    ];
    for (name, cfg) in cfgs {
        let mut r = rec0.clone();
        let s = bundle_adjust(&mut r, &cfg).unwrap();
        println!(
            "{name}: 반복 {} 비용 {:.4e}→{:.4e} {:?} / {:?} 오차 {:.3} 초점 {:?}",
            s.num_iterations,
            s.initial_cost,
            s.final_cost,
            s.termination,
            stats(&r),
            r.mean_reproj_error(),
            r.cameras().values().map(|c| (c.params[0] as i64, c.params[1] as i64)).collect::<Vec<_>>()
        );
    }
}
