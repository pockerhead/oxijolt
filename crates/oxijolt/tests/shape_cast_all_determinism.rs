//! Determinism of `cast_shape_all`: shapes cast through two toppling stacks of cubes after every
//! step report the same hits bit for bit with 1 and 4 worker threads, in one process and in two.

// The process test is compiled out on wasm32; its helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::*;
use oxijolt::*;

const TICKS: usize = 120;

/// Horizontal casts through the stacks at three heights, and vertical casts down onto them and
/// the floor, as (start, direction).
fn casts() -> Vec<(RVec3, Vec3)> {
    let mut casts: Vec<(RVec3, Vec3)> = [0.5, 1.5, 2.5]
        .map(|y| (RVec3::new(-5.0, y, 0.0), Vec3::new(15.0, 0.0, 0.0)))
        .into();
    for i in 0..6 {
        let x = 0.5 * i as Real;
        casts.push((RVec3::new(x, 6.0, 0.0), Vec3::new(0.0, -8.0, 0.0)));
    }
    casts
}

/// The stacks run for [`TICKS`] ticks with `threads` workers; after each step every body and
/// every hit of a ball and a lying capsule along every cast.
fn run(threads: u32) -> (Digest, usize) {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), threads);
    let ids = build_stacks(&mut world);
    let ball = Shape::new_sphere(0.3).unwrap();
    let capsule = Shape::new_capsule(0.4, 0.2).unwrap();
    let lying = quat_about(Vec3::new(0.0, 0.0, 1.0), 90.0_f32.to_radians());
    let shapes = [(&ball, Quat::IDENTITY), (&capsule, lying)];
    let mut hit_count = 0;
    let mut digest = Digest::new();
    for _ in 0..TICKS {
        step(&mut world, 1);
        let tick = digest.push();
        for &id in &ids {
            record_body(&world, id, &mut tick.state);
        }
        for (shape, rotation) in shapes {
            for (start, direction) in casts() {
                let cast = ShapeCast::new(shape, start, rotation, direction);
                let hits = world.cast_shape_all(&cast, &QueryFilter::new()).unwrap();
                hit_count += hits.len();
                for hit in hits {
                    let normal: [f32; 3] = hit.normal.into();
                    tick.state.extend(hit.body.to_raw().to_le_bytes());
                    tick.state.extend(hit.sub_shape_id.to_raw().to_le_bytes());
                    tick.state.extend(hit.fraction.to_bits().to_le_bytes());
                    tick.state
                        .extend(hit.penetration_depth.to_bits().to_le_bytes());
                    for value in normal {
                        tick.state.extend(value.to_bits().to_le_bytes());
                    }
                }
                tick.state.push(b'|');
            }
        }
    }
    (digest, hit_count)
}

fn stacks_shape_casts(threads: u32) -> Digest {
    run(threads).0
}

#[test]
#[ignore = "child process of the shape cast determinism gate"]
fn shape_cast_all_determinism_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "stacks_shape_casts");
    finish_child(&stacks_shape_casts(threads));
}

#[test]
fn shape_cast_hits_match_with_1_and_4_workers() {
    let (one, hit_count) = run(1);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same(
        "1 vs 4 workers in one process",
        &one,
        &stacks_shape_casts(4),
    );
    assert_ne!(one.ticks[0].state, one.ticks[TICKS - 1].state);
    // More than the floor under each vertical cast of both shapes: the casts cross cubes too.
    assert!(hit_count > 2 * 6 * TICKS, "{hit_count}");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn shape_cast_hits_match_across_processes() {
    let child = "shape_cast_all_determinism_child";
    let one = digest_in_child(child, "stacks_shape_casts", 1, "");
    let four = digest_in_child(child, "stacks_shape_casts", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &stacks_shape_casts(1), &one);
}
