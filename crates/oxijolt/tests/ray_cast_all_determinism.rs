//! Determinism of `cast_ray_all` and `QueryFilter::body_filter`: rays through two toppling
//! stacks of cubes, cast after every step, report the same hits bit for bit with 1 and 4 worker
//! threads, in one process and in two.

// The process test is compiled out on wasm32; its helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::*;
use oxijolt::*;

const TICKS: usize = 120;

/// Horizontal rays through the stacks at three heights, and vertical rays down onto them and
/// the floor.
fn rays() -> Vec<RayCast> {
    let mut rays: Vec<RayCast> = [0.5, 1.5, 2.5]
        .map(|y| RayCast::new(RVec3::new(-5.0, y, 0.0), Vec3::new(15.0, 0.0, 0.0)))
        .into();
    for i in 0..6 {
        let x = 0.5 * i as Real;
        rays.push(RayCast::new(
            RVec3::new(x, 6.0, 0.0),
            Vec3::new(0.0, -8.0, 0.0),
        ));
    }
    rays
}

/// What a run saw besides its digest.
struct Counts {
    hits: usize,
    filtered_hits: usize,
}

/// The stacks run for [`TICKS`] ticks with `threads` workers; after each step every body and
/// every hit of every ray, unfiltered and with a body filter that keeps even raw ids.
fn run(threads: u32) -> (Digest, Counts) {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), threads);
    let ids = build_stacks(&mut world);
    let even = |body: BodyId| body.to_raw().is_multiple_of(2);
    let filters = [QueryFilter::new(), QueryFilter::new().body_filter(&even)];
    let mut counts = Counts {
        hits: 0,
        filtered_hits: 0,
    };
    let mut digest = Digest::new();
    for _ in 0..TICKS {
        step(&mut world, 1);
        let tick = digest.push();
        for &id in &ids {
            record_body(&world, id, &mut tick.state);
        }
        for (index, filter) in filters.iter().enumerate() {
            for ray in rays() {
                let hits = world.cast_ray_all(&ray, filter).unwrap();
                if index == 0 {
                    counts.hits += hits.len();
                } else {
                    counts.filtered_hits += hits.len();
                }
                for hit in hits {
                    let normal: [f32; 3] = hit.normal.into();
                    tick.state.extend(hit.body.to_raw().to_le_bytes());
                    tick.state.extend(hit.sub_shape_id.to_raw().to_le_bytes());
                    tick.state.extend(hit.fraction.to_bits().to_le_bytes());
                    for value in normal {
                        tick.state.extend(value.to_bits().to_le_bytes());
                    }
                }
                tick.state.push(b'|');
            }
        }
    }
    (digest, counts)
}

fn stacks_rays(threads: u32) -> Digest {
    run(threads).0
}

#[test]
#[ignore = "child process of the ray cast determinism gate"]
fn ray_cast_all_determinism_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "stacks_rays");
    finish_child(&stacks_rays(threads));
}

#[test]
fn ray_hits_match_with_1_and_4_workers() {
    let (one, counts) = run(1);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in one process", &one, &stacks_rays(4));
    assert_ne!(one.ticks[0].state, one.ticks[TICKS - 1].state);
    // More than the floor under each vertical ray: the rays cross cubes too. The
    // body filter drops some hits but not all.
    assert!(counts.hits > 6 * TICKS, "{}", counts.hits);
    assert!(counts.filtered_hits > 0, "{}", counts.filtered_hits);
    assert!(
        counts.filtered_hits < counts.hits,
        "{}",
        counts.filtered_hits
    );
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn ray_hits_match_across_processes() {
    let one = digest_in_child("ray_cast_all_determinism_child", "stacks_rays", 1, "");
    let four = digest_in_child("ray_cast_all_determinism_child", "stacks_rays", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &stacks_rays(1), &one);
}
