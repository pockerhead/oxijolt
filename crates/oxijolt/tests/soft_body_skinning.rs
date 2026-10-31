//! Skinned soft bodies: the bind pose a new body starts in and settings shared between worlds.

mod common;

use common::soft_body::Cloth;
use common::{quat_about, step, world, DT};
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// Row 0 attached to the skin, row 1 skinned without a max distance, the rest free.
const ROWS: [Option<f32>; 2] = [Some(0.0), None];

/// A 5 x 5 cloth whose vertices outside row 0 start with a velocity.
fn moving_cloth() -> Cloth {
    let mut cloth = Cloth::new(5, 0.25);
    for vertex in &mut cloth.vertices[5..] {
        vertex.velocity = Vec3::new(0.5, 0.0, 0.25);
    }
    cloth
}

fn finite(p: RVec3) -> bool {
    p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
}

const POSITION: RVec3 = RVec3::new(1.0, 3.0, -2.0);

fn rotation() -> Quat {
    quat_about(Vec3::new(0.48, 0.8, 0.36), 0.8)
}

fn placement(make_rotation_identity: bool, activation: Activation) -> SoftBodySettings {
    SoftBodySettings::default()
        .position(POSITION)
        .rotation(rotation())
        .make_rotation_identity(make_rotation_identity)
        .activation(activation)
}

/// Where the placement puts a settings-frame `point` in the world, in `f64`.
fn placed(point: Vec3) -> [f64; 3] {
    let rotation = rotation();
    let [x, y, z, w] = [rotation.x, rotation.y, rotation.z, rotation.w].map(f64::from);
    let p = [point.x, point.y, point.z].map(f64::from);
    // v + 2w (q x v) + 2 q x (q x v)
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let t = cross([x, y, z], p).map(|c| 2.0 * c);
    let u = cross([x, y, z], t);
    let origin = [POSITION.x, POSITION.y, POSITION.z].map(wide);
    std::array::from_fn(|i| origin[i] + p[i] + w * t[i] + u[i])
}

/// `value` as `f64`; a no-op with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn wide(value: Real) -> f64 {
    f64::from(value)
}

fn distance(a: RVec3, b: [f64; 3]) -> f64 {
    let d = [wide(a.x) - b[0], wide(a.y) - b[1], wide(a.z) - b[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

#[test]
fn a_new_skinned_cloth_holds_its_bind_pose_without_skin_calls() {
    let cloth = moving_cloth();
    let skinned = cloth.skinned(&ROWS).build().unwrap();
    let control = cloth.builder().build().unwrap();
    let hard_row: Vec<u32> = (0..cloth.side).map(|x| cloth.index(x, 0)).collect();
    for make_rotation_identity in [true, false] {
        for activation in [Activation::Activate, Activation::DontActivate] {
            let case = format!("make_rotation_identity {make_rotation_identity}, {activation:?}");
            let settings = placement(make_rotation_identity, activation);
            let mut world = world(GRAVITY, 2);
            let mut control_world = common::world(GRAVITY, 2);
            let id = world.create_soft_body(&skinned, &settings).unwrap();
            let control_id = control_world.create_soft_body(&control, &settings).unwrap();
            if activation == Activation::DontActivate {
                assert!(!world.body(id).unwrap().is_active(), "{case}");
                step(&mut world, 5);
                world.body_mut(id).unwrap().activate();
                control_world.body_mut(control_id).unwrap().activate();
            }
            step(&mut world, 1);
            step(&mut control_world, 1);
            // Rows 1 and up keep their initial velocities as the control does: no constraint
            // couples them and row 1 has no max distance.
            let vertices = world.soft_body(id).unwrap().vertices();
            let control_vertices = control_world.soft_body(control_id).unwrap().vertices();
            for (index, (vertex, expected)) in
                vertices.iter().zip(&control_vertices).enumerate().skip(5)
            {
                let v = vertex.velocity;
                let e = expected.velocity;
                let difference = (v.x - e.x)
                    .abs()
                    .max((v.y - e.y).abs())
                    .max((v.z - e.z).abs());
                assert!(difference < 1.0e-4, "{case}: vertex {index} {v:?} vs {e:?}");
                assert!(v.x > 0.4, "{case}: vertex {index} lost its velocity: {v:?}");
            }
            step(&mut world, 59);
            let vertices = world.soft_body(id).unwrap().vertices();
            assert!(vertices.iter().all(|v| finite(v.position)), "{case}");
            for &index in &hard_row {
                let bind = cloth.vertices[index as usize].position;
                let expected = placed(bind);
                let error = distance(vertices[index as usize].position, expected);
                assert!(
                    error < 1.0e-4,
                    "{case}: hard vertex {index} is {error} m from its bind pose"
                );
            }
            // The free rows fell.
            let last = vertices.last().unwrap().position;
            assert!(wide(last.y) < 2.5, "{case}: {last:?}");
        }
    }
}

#[test]
fn skinned_settings_serve_two_worlds() {
    let cloth = moving_cloth();
    let skinned = cloth.skinned(&ROWS).build().unwrap();
    let settings = placement(true, Activation::Activate);
    let mut first = world(GRAVITY, 1);
    let mut second = world(GRAVITY, 1);
    let a = first.create_soft_body(&skinned, &settings).unwrap();
    let b = second.create_soft_body(&skinned, &settings).unwrap();
    // Each body keeps its own reference to the settings.
    drop(skinned);
    for _ in 0..30 {
        assert!(first.step(DT).unwrap().is_complete());
        assert!(second.step(DT).unwrap().is_complete());
    }
    let positions = |world: &PhysicsWorld, id| -> Vec<[u64; 3]> {
        world
            .soft_body(id)
            .unwrap()
            .vertices()
            .iter()
            .map(|v| [v.position.x, v.position.y, v.position.z].map(|c| wide(c).to_bits()))
            .collect()
    };
    assert_eq!(positions(&first, a), positions(&second, b));
}
