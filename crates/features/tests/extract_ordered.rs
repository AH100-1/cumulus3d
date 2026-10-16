/*
 * extract_ordered.rs
 *
 * Copyright (c) 2026 ParkSangWoo
 *
 * Author(s):
 *
 *      ParkSangWoo <dev.parksangwoo@gmail.com>
 *
 *
 * This file is part of cumulus3d.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//! `FeatureExtractor::extract_ordered`: image ids follow the given order, every image has its own camera mode, repeated names count
//! once, and a missing existing camera is an error.

use cumulus3d_core::FeatureStore;
use cumulus3d_features::*;

fn write_images(root: &std::path::Path, names: &[&str]) {
    for (k, n) in names.iter().enumerate() {
        let p = root.join(n);
        std::fs::create_dir_all(p.parent().unwrap()).unwrap();
        let img = image::GrayImage::from_fn(160, 120, |x, y| {
            let v = (x * 7 + y * 13 + k as u32 * 29) ^ (x * y / 5);
            image::Luma([(v % 251) as u8])
        });
        img.save(&p).unwrap();
    }
}

#[test]
fn ids_follow_the_given_order_and_each_image_has_its_own_camera_mode() {
    let dir = std::env::temp_dir().join(format!("c3d_extract_ordered_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    write_images(&dir, &["b/b_2.png", "a/a_1.png", "c/c_3.png", "a/a_4.png", "b/b_5.png"]);
    let ex = FeatureExtractor::new();
    let store = FeatureStore::new();
    let opts = ExtractionOptions::default();

    // First bundle: one camera per folder, issued in the order given (b, a, c), not sorted.
    let items: Vec<(String, CameraMode)> =
        ["b/b_2.png", "a/a_1.png", "c/c_3.png", "b/b_2.png"].iter().map(|n| (n.to_string(), CameraMode::PerFolder)).collect();
    let rep = ex.extract_ordered(&store, &dir, &items, &opts).unwrap();
    assert_eq!(rep.len(), 3, "the repeated name counts once");
    let ids: Vec<(String, u32)> = rep
        .iter()
        .map(|r| match &r.status {
            ImageStatus::Extracted { image_id, .. } => (r.name.clone(), *image_id),
            s => panic!("{s:?}"),
        })
        .collect();
    assert_eq!(ids, vec![("b/b_2.png".to_string(), 1), ("a/a_1.png".to_string(), 2), ("c/c_3.png".to_string(), 3)]);
    let cam_of = |n: &str| store.image_by_name(n).unwrap().camera_id;
    assert_eq!(cam_of("b/b_2.png"), 1);
    assert_eq!(cam_of("a/a_1.png"), 2);

    // Next position: images attach to the existing cameras, ids continue.
    let items = vec![
        ("a/a_4.png".to_string(), CameraMode::Existing(cam_of("a/a_1.png"))),
        ("b/b_5.png".to_string(), CameraMode::Existing(cam_of("b/b_2.png"))),
    ];
    let rep = ex.extract_ordered(&store, &dir, &items, &opts).unwrap();
    assert!(rep.iter().all(|r| matches!(r.status, ImageStatus::Extracted { .. })));
    assert_eq!((store.image_by_name("a/a_4.png").unwrap().image_id, store.image_by_name("b/b_5.png").unwrap().image_id), (4, 5));
    assert_eq!((cam_of("a/a_4.png"), cam_of("b/b_5.png")), (cam_of("a/a_1.png"), cam_of("b/b_2.png")));
    assert_eq!(store.num_cameras(), 3);

    // Done images are reported as such, a missing file fails only its own entry, a missing camera fails the call.
    let items = vec![("a/a_4.png".to_string(), CameraMode::Existing(2)), ("nope.png".to_string(), CameraMode::PerFolder)];
    let rep = ex.extract_ordered(&store, &dir, &items, &opts).unwrap();
    assert!(matches!(rep[0].status, ImageStatus::AlreadyExists { image_id: 4 }));
    assert!(matches!(rep[1].status, ImageStatus::Failed { .. }));
    assert!(ex.extract_ordered(&store, &dir, &[("a/a_1.png".to_string(), CameraMode::Existing(99))], &opts).is_err());
    let _ = std::fs::remove_dir_all(&dir);
}
