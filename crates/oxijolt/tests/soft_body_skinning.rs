//! Skinned soft bodies: the bind pose a new body starts in, settings shared between worlds,
//! skinning to joint poses, max distances, back stops, refusals and rollback.

mod common;

use common::math::{add, f3, norm, rotate as rotate_wide, scale, sub, v3, V3};
use common::ragdoll::{conj, joint_position, mul, pose_from};
use common::soft_body::{only_weight, Cloth};
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

const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);

/// An inverse bind away from the settings origin, turned about Z.
fn offset_bind() -> SoftBodyInvBind {
    SoftBodyInvBind::new(
        0,
        Vec3::new(0.1, 0.2, -0.1),
        quat_about(Vec3::new(0.0, 0.0, 1.0), 0.2),
    )
}

/// Where a joint at the world `position` with `rotation` puts the settings-frame `vertex`
/// skinned to it through `bind`: `J · B⁻¹ · v`, in `f64`.
fn skinned_position(position: V3, rotation: Quat, bind: SoftBodyInvBind, vertex: Vec3) -> V3 {
    let local = rotate_wide(conj(bind.rotation), sub(f3(vertex), f3(bind.position)));
    add(position, rotate_wide(rotation, local))
}

/// The world pose of `bind`'s joint for a body created at `position` with [`rotation`]: the
/// bind pose the body starts in.
fn bind_joint(position: RVec3, bind: SoftBodyInvBind) -> (V3, Quat) {
    let r = rotation();
    (
        add(v3(position), rotate_wide(r, f3(bind.position))),
        mul(r, bind.rotation),
    )
}

/// Rounding of a world position near `position` in `Real`.
fn world_rounding(position: RVec3) -> f64 {
    4.0 * wide(Real::EPSILON) * norm(v3(position))
}

fn is_zero(v: Vec3) -> bool {
    v.x == 0.0 && v.y == 0.0 && v.z == 0.0
}

/// A hard skin to a joint moved by (0.3, 0, 0) and turned 30 degrees puts every skinned vertex
/// on `J · B⁻¹ · v` within a rounding bound and stops it.
fn hard_skin_follows_the_joint_at(position: RVec3) {
    let cloth = Cloth::new(3, 0.5);
    let bind = offset_bind();
    let shared = cloth
        .skinned_to(bind, &[Some(0.0), None, Some(0.3)])
        .build()
        .unwrap();
    for make_rotation_identity in [true, false] {
        let mut world = world(GRAVITY, 1);
        let settings = SoftBodySettings::default()
            .position(position)
            .rotation(rotation())
            .make_rotation_identity(make_rotation_identity);
        let id = world.create_soft_body(&shared, &settings).unwrap();
        let (start, start_rotation) = bind_joint(position, bind);
        let joint = add(start, [0.3, 0.0, 0.0]);
        let joint_rotation = mul(quat_about(Y, 30.0_f32.to_radians()), start_rotation);
        world
            .soft_body_mut(id)
            .unwrap()
            .hard_skin_vertices(&pose_from(&[(joint, joint_rotation)]))
            .unwrap();
        let vertices = world.soft_body(id).unwrap().vertices();
        let translation = norm(sub(joint, v3(position)));
        for (index, vertex) in cloth.vertices.iter().enumerate() {
            let expected = skinned_position(joint, joint_rotation, bind, vertex.position);
            let tolerance = 8.0
                * f64::from(f32::EPSILON)
                * (translation + norm(f3(bind.position)) + norm(f3(vertex.position)))
                + world_rounding(position);
            let error = norm(sub(v3(vertices[index].position), expected));
            assert!(
                error <= tolerance,
                "make_rotation_identity {make_rotation_identity}: vertex {index} is {error} m \
                 off (tolerance {tolerance})"
            );
            assert!(
                is_zero(vertices[index].velocity),
                "vertex {index}: {:?}",
                vertices[index].velocity
            );
        }
    }
}

#[test]
fn hard_skin_puts_the_skinned_vertices_on_the_joint() {
    hard_skin_follows_the_joint_at(POSITION);
}

#[test]
fn hard_skin_far_from_the_origin_in_double_precision() {
    if std::mem::size_of::<Real>() == 8 {
        hard_skin_follows_the_joint_at(RVec3::new(1.0e6 as Real + 0.3, 0.0, 0.0));
    }
}

#[test]
fn attached_vertices_follow_a_moving_joint() {
    let cloth = Cloth::new(3, 0.5);
    let bind = offset_bind();
    let shared = cloth.skinned_to(bind, &[Some(0.0)]).build().unwrap();
    let mut world = world(GRAVITY, 2);
    let settings = SoftBodySettings::default()
        .position(POSITION)
        .rotation(rotation());
    let id = world.create_soft_body(&shared, &settings).unwrap();
    let (start, start_rotation) = bind_joint(POSITION, bind);
    for tick in 0..120 {
        let phase = 2.0 * std::f64::consts::PI * f64::from(tick) / 60.0;
        let joint = add(start, [0.4 * phase.sin(), 0.2 * (2.0 * phase).sin(), 0.0]);
        let joint_rotation = mul(quat_about(Y, 0.3 * phase.sin() as f32), start_rotation);
        skin(&mut world, id, &pose_from(&[(joint, joint_rotation)])).unwrap();
        step(&mut world, 1);
        let vertices = world.soft_body(id).unwrap().vertices();
        for x in 0..cloth.side {
            let index = cloth.index(x, 0) as usize;
            let expected =
                skinned_position(joint, joint_rotation, bind, cloth.vertices[index].position);
            let error = norm(sub(v3(vertices[index].position), expected));
            assert!(
                error < 1.0e-4,
                "tick {tick}: vertex {index} is {error} m off"
            );
        }
        assert!(vertices.iter().all(|v| finite(v.position)), "tick {tick}");
    }
}

/// Joint and normalised weight of every weight slot of a vertex.
type ExpectedWeights = [[(usize, f64); 4]; 4];

/// Inverse binds of joints 0, 2, 5 and 7 at the origin, and a 2 x 2 cloth whose vertex 0 has
/// two weights, vertex 1 four and vertices 2 and 3 one each, all attached.
fn weighted_cloth() -> (SoftBodySharedSettings, Vec<Vec3>, ExpectedWeights) {
    let cloth = Cloth::new(2, 0.5);
    let joints = [0, 2, 5, 7];
    let weights: [[(u32, f32); 4]; 4] = [
        [(0, 1.0), (3, 3.0), (0, 0.0), (0, 0.0)],
        [(0, 1.0), (1, 2.0), (2, 3.0), (3, 4.0)],
        [(1, 1.0), (0, 0.0), (0, 0.0), (0, 0.0)],
        [(2, 1.0), (0, 0.0), (0, 0.0), (0, 0.0)],
    ];
    let mut builder = cloth.builder().inv_binds(
        joints
            .iter()
            .map(|&joint| SoftBodyInvBind::new(joint, Vec3::ZERO, Quat::IDENTITY)),
    );
    let mut expected = [[(0, 0.0); 4]; 4];
    for (vertex, vertex_weights) in weights.iter().enumerate() {
        let total: f64 = vertex_weights.iter().map(|&(_, w)| f64::from(w)).sum();
        for (slot, &(inv_bind, weight)) in vertex_weights.iter().enumerate() {
            expected[vertex][slot] = (
                joints[inv_bind as usize] as usize,
                f64::from(weight) / total,
            );
        }
        builder = builder.skinned(SoftBodySkinned {
            max_distance: Some(0.0),
            ..SoftBodySkinned::new(
                vertex as u32,
                vertex_weights.map(|(inv_bind, weight)| SoftBodySkinWeight::new(inv_bind, weight)),
            )
        });
    }
    let positions = cloth.vertices.iter().map(|v| v.position).collect();
    (builder.build().unwrap(), positions, expected)
}

/// `count` joints, joint `j` at `POSITION + (0.1 j, 0.05 j, -0.02 j)` turned `0.1 j` about Y.
fn spread_joints(count: usize) -> Vec<(V3, Quat)> {
    (0..count)
        .map(|j| {
            let along = j as f64;
            (
                add(v3(POSITION), [0.1 * along, 0.05 * along, -0.02 * along]),
                quat_about(Y, 0.1 * j as f32),
            )
        })
        .collect()
}

#[test]
fn weighted_and_sparse_joints_blend() {
    let (shared, positions, expected) = weighted_cloth();
    assert_eq!(shared.skin_joint_count(), 8);
    let mut world = world(Vec3::ZERO, 1);
    let id = world
        .create_soft_body(&shared, &SoftBodySettings::default().position(POSITION))
        .unwrap();
    let joints = spread_joints(8);
    let mut body = world.soft_body_mut(id).unwrap();
    // A pose without joint 7 is refused.
    assert!(matches!(
        body.hard_skin_vertices(&pose_from(&joints[..7])),
        Err(BodyError::InvalidValue(_))
    ));
    body.hard_skin_vertices(&pose_from(&joints)).unwrap();
    let vertices = body.vertices();
    for (vertex, weights) in expected.iter().enumerate() {
        let target = weights.iter().filter(|&&(_, weight)| weight > 0.0).fold(
            [0.0; 3],
            |sum, &(joint, weight)| {
                let (position, rotation) = joints[joint];
                let at = add(position, rotate_wide(rotation, f3(positions[vertex])));
                add(sum, scale(at, weight))
            },
        );
        let error = norm(sub(v3(vertices[vertex].position), target));
        assert!(error < 1.0e-5, "vertex {vertex} is {error} m off");
    }
    // Joints beyond the bound ones are ignored.
    let mut more = joints.clone();
    more.extend(spread_joints(12).into_iter().skip(8));
    body.hard_skin_vertices(&pose_from(&more)).unwrap();
    assert_eq!(body.vertices(), vertices);
}

/// A 2 x 2 cloth at [`POSITION`] whose row 0 is skinned with `max_distance` and row 1 is free,
/// hanging for 60 ticks under gravity from a joint held at its bind pose, with the multiplier
/// set to `multiplier`. Returns how far each row-0 vertex is from its skinned position and how
/// far row 1 fell.
fn hang(max_distance: Option<f32>, multiplier: f32) -> (Vec<f64>, f64) {
    let cloth = Cloth::new(2, 0.5);
    let shared = cloth.skinned(&[max_distance]).build().unwrap();
    let mut world = world(GRAVITY, 1);
    let id = world
        .create_soft_body(&shared, &SoftBodySettings::default().position(POSITION))
        .unwrap();
    set_multiplier(&mut world, id, multiplier).unwrap();
    assert_eq!(
        world
            .soft_body(id)
            .unwrap()
            .skinned_max_distance_multiplier(),
        multiplier
    );
    let pose = pose_from(&[(v3(POSITION), Quat::IDENTITY)]);
    for _ in 0..60 {
        skin(&mut world, id, &pose).unwrap();
        step(&mut world, 1);
    }
    let vertices = world.soft_body(id).unwrap().vertices();
    let strays = (0..2)
        .map(|x| {
            let skinned = add(v3(POSITION), f3(cloth.vertices[x].position));
            norm(sub(v3(vertices[x].position), skinned))
        })
        .collect();
    let fall = wide(POSITION.y) - wide(vertices[3].position.y);
    (strays, fall)
}

#[test]
fn max_distance_limits_how_far_a_vertex_strays() {
    for (multiplier, reach) in [(1.0, 0.2), (2.0, 0.4), (0.0, 0.0)] {
        let (strays, fall) = hang(Some(0.2), multiplier);
        assert!(fall > 3.0, "the free row fell only {fall} m");
        for stray in strays {
            assert!(
                stray <= reach + 1.0e-4 && stray >= reach - 1.0e-2,
                "multiplier {multiplier}: stray {stray}, reach {reach}"
            );
        }
    }
    // Without a max distance no multiplier limits the vertex: it falls with the free row.
    for multiplier in [1.0, 3.0] {
        let (strays, fall) = hang(None, multiplier);
        for stray in strays {
            assert!(
                stray > 3.0 && (stray - fall).abs() < 0.1,
                "{stray} vs {fall}"
            );
        }
    }
}

/// A 2 x 2 cloth at [`POSITION`] with vertices 0, 2 and 3 attached and vertex 1 skinned with
/// a max distance of 1 m and `back_stop`, pushed down (against the faces' normal) at 2 m/s
/// without gravity. Returns the smallest `|x - c| - r` over 30 ticks, with `c` and `r` the
/// centre and radius of `stop`'s sphere.
fn push_through(back_stop: Option<SoftBodyBackStop>, stop: SoftBodyBackStop) -> f64 {
    let cloth = Cloth::new(2, 0.5);
    let mut builder = cloth
        .builder()
        .inv_bind(SoftBodyInvBind::new(0, Vec3::ZERO, Quat::IDENTITY));
    for vertex in 0..4 {
        let mut skinned = SoftBodySkinned::new(vertex, only_weight(0));
        if vertex == 1 {
            skinned.max_distance = Some(1.0);
            skinned.back_stop = back_stop;
        } else {
            skinned.max_distance = Some(0.0);
        }
        builder = builder.skinned(skinned);
    }
    let shared = builder.build().unwrap();
    let mut world = world(Vec3::ZERO, 1);
    let id = world
        .create_soft_body(&shared, &SoftBodySettings::default().position(POSITION))
        .unwrap();
    world
        .soft_body_mut(id)
        .unwrap()
        .set_vertex_velocity(1, Vec3::new(0.0, -2.0, 0.0))
        .unwrap();
    // The faces point up (+Y), so the sphere sits below the skinned position.
    let skinned = add(v3(POSITION), f3(cloth.vertices[1].position));
    let centre = sub(skinned, [0.0, f64::from(stop.distance + stop.radius), 0.0]);
    let pose = pose_from(&[(v3(POSITION), Quat::IDENTITY)]);
    let mut closest = f64::INFINITY;
    for _ in 0..30 {
        skin(&mut world, id, &pose).unwrap();
        step(&mut world, 1);
        let x = v3(world.soft_body(id).unwrap().vertices()[1].position);
        closest = closest.min(norm(sub(x, centre)) - f64::from(stop.radius));
    }
    closest
}

#[test]
fn back_stop_keeps_a_vertex_outside_its_sphere() {
    let stop = SoftBodyBackStop {
        distance: 0.1,
        radius: 0.5,
    };
    let kept = push_through(Some(stop), stop);
    assert!(
        kept >= -1.0e-4,
        "the vertex went {kept} m into the back stop"
    );
    let control = push_through(None, stop);
    assert!(
        control < -0.1,
        "without a back stop the vertex stayed out: {control}"
    );
}

/// A world with a skinned cloth (row 0 attached, row 1 within 0.2 m) and an unskinned one.
fn refusal_world() -> (PhysicsWorld, BodyId, BodyId) {
    let cloth = Cloth::new(3, 0.5);
    let mut world = world(GRAVITY, 1);
    let skinned = world
        .create_soft_body(
            &cloth.skinned(&[Some(0.0), Some(0.2)]).build().unwrap(),
            &SoftBodySettings::default().position(POSITION),
        )
        .unwrap();
    let plain = world
        .create_soft_body(
            &cloth.builder().build().unwrap(),
            &SoftBodySettings::default().position(RVec3::new(5.0, 3.0, 0.0)),
        )
        .unwrap();
    (world, skinned, plain)
}

/// Every vertex position and velocity of `id`, as bits.
fn vertex_bits(world: &PhysicsWorld, id: BodyId) -> Vec<([u64; 3], [u32; 3])> {
    world
        .soft_body(id)
        .unwrap()
        .vertices()
        .iter()
        .map(|v| {
            (
                [v.position.x, v.position.y, v.position.z].map(|c| wide(c).to_bits()),
                [v.velocity.x, v.velocity.y, v.velocity.z].map(f32::to_bits),
            )
        })
        .collect()
}

fn skin(world: &mut PhysicsWorld, id: BodyId, pose: &SkeletonPose) -> Result<(), BodyError> {
    world.soft_body_mut(id).unwrap().skin_vertices(pose)
}

fn set_multiplier(world: &mut PhysicsWorld, id: BodyId, value: f32) -> Result<(), BodyError> {
    world
        .soft_body_mut(id)
        .unwrap()
        .set_skinned_max_distance_multiplier(value)
}

fn moved_joint() -> (V3, Quat) {
    (add(v3(POSITION), [0.2, 0.1, 0.0]), quat_about(Y, 0.3))
}

/// A call on the skinned body or on the unskinned one that must be refused.
type Refusal = fn(&mut PhysicsWorld, BodyId, BodyId) -> Result<(), BodyError>;

#[test]
fn refused_skin_calls_change_nothing() {
    let refusals: [(&str, Refusal); 11] = [
        ("short pose", |w, id, _| {
            let pose = SkeletonPose {
                root_offset: POSITION,
                joints: Vec::new(),
            };
            skin(w, id, &pose)
        }),
        ("non-unit rotation", |w, id, _| {
            let turned = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);
            skin(w, id, &pose_from(&[(v3(POSITION), turned)]))
        }),
        ("NaN translation", |w, id, _| {
            let mut pose = pose_from(&[moved_joint(), moved_joint()]);
            pose.joints[1].translation.x = f32::NAN;
            skin(w, id, &pose)
        }),
        ("joint 2001 m from the centre of mass", |w, id, _| {
            let far = add(v3(POSITION), [2001.0, 0.0, 0.0]);
            skin(w, id, &pose_from(&[(far, Quat::IDENTITY)]))
        }),
        ("root beyond the world", |w, id, _| {
            let mut pose = pose_from(&[moved_joint()]);
            pose.root_offset.x = 2.0 * limits::MAX_POSITION;
            skin(w, id, &pose)
        }),
        ("unskinned body", |w, _, plain| {
            skin(w, plain, &pose_from(&[moved_joint()]))
        }),
        ("hard skin of an unskinned body", |w, _, plain| {
            w.soft_body_mut(plain)
                .unwrap()
                .hard_skin_vertices(&pose_from(&[moved_joint()]))
        }),
        ("NaN multiplier", |w, id, _| set_multiplier(w, id, f32::NAN)),
        ("negative multiplier", |w, id, _| {
            set_multiplier(w, id, -1.0)
        }),
        ("multiplier above the bound", |w, id, _| {
            set_multiplier(w, id, limits::MAX_RATIO.next_up())
        }),
        ("multiplier of an unskinned body", |w, _, plain| {
            set_multiplier(w, plain, 1.0)
        }),
    ];
    let pose = pose_from(&[moved_joint()]);
    let (mut clean, clean_id, _) = refusal_world();
    skin(&mut clean, clean_id, &pose).unwrap();
    step(&mut clean, 1);
    let expected = vertex_bits(&clean, clean_id);
    for (what, refusal) in refusals {
        let (mut world, id, plain) = refusal_world();
        assert!(
            matches!(
                refusal(&mut world, id, plain),
                Err(BodyError::InvalidValue(_))
            ),
            "{what} was not refused"
        );
        for body in [id, plain] {
            let multiplier = world
                .soft_body(body)
                .unwrap()
                .skinned_max_distance_multiplier();
            assert_eq!(multiplier, 1.0, "{what}");
        }
        skin(&mut world, id, &pose).unwrap();
        step(&mut world, 1);
        assert!(
            vertex_bits(&world, id) == expected,
            "{what} changed the body"
        );
    }
}

#[test]
fn skinning_wakes_a_sleeping_body() {
    let cloth = Cloth::new(2, 0.5);
    let shared = cloth.skinned(&[Some(0.0)]).build().unwrap();
    let settings = SoftBodySettings::default()
        .position(POSITION)
        .activation(Activation::DontActivate);
    let mut world = world(GRAVITY, 1);
    let skinned = world.create_soft_body(&shared, &settings).unwrap();
    let loosened = world.create_soft_body(&shared, &settings).unwrap();
    assert!(!world.body(skinned).unwrap().is_active());
    assert!(!world.body(loosened).unwrap().is_active());
    skin(&mut world, skinned, &pose_from(&[moved_joint()])).unwrap();
    set_multiplier(&mut world, loosened, 2.0).unwrap();
    assert!(world.body(skinned).unwrap().is_active());
    assert!(world.body(loosened).unwrap().is_active());
}

#[test]
fn attached_vertices_reach_the_joint_with_two_collision_steps() {
    // The attached row is kinematic, so the edges solved after the skin cannot move it.
    let cloth = Cloth::new(3, 0.5).pin(&[0, 1, 2]);
    let shared = cloth
        .skinned(&[Some(0.0), Some(0.2)])
        .create_constraints(
            SoftBodyBendType::Distance,
            SoftBodyVertexAttributes::default(),
        )
        .build()
        .unwrap();
    let mut world = world(GRAVITY, 2);
    let id = world
        .create_soft_body(&shared, &SoftBodySettings::default().position(POSITION))
        .unwrap();
    let (joint, joint_rotation) = moved_joint();
    for _ in 0..10 {
        skin(&mut world, id, &pose_from(&[(joint, joint_rotation)])).unwrap();
        assert!(world
            .step_with_collision_steps(DT, 2)
            .unwrap()
            .is_complete());
    }
    let vertices = world.soft_body(id).unwrap().vertices();
    assert!(vertices.iter().all(|v| finite(v.position)));
    let bind = SoftBodyInvBind::new(0, Vec3::ZERO, Quat::IDENTITY);
    for x in 0..cloth.side {
        let index = cloth.index(x, 0) as usize;
        let expected =
            skinned_position(joint, joint_rotation, bind, cloth.vertices[index].position);
        let error = norm(sub(v3(vertices[index].position), expected));
        assert!(error < 1.0e-4, "vertex {index} is {error} m off");
    }
}

#[test]
fn a_cape_follows_a_ragdoll_from_another_world() {
    use common::ragdoll::{bind_pose, humanoid_settings, ragdoll_world, transformed_pose, CHEST};
    let (mut ragdoll_world, layers) = ragdoll_world(1);
    let start = transformed_pose(&bind_pose(), Quat::IDENTITY, [0.0, 2.0, 0.0]);
    let ragdoll = ragdoll_world
        .create_ragdoll(
            &humanoid_settings(layers.ragdoll),
            Some(&start),
            Activation::Activate,
        )
        .unwrap();
    let pose = ragdoll_world.ragdoll(ragdoll).unwrap().pose();
    // A 4 x 4 cape behind and below the chest, its top row attached to the chest.
    let chest = joint_position(&pose, CHEST);
    let cape_position = RVec3::new(
        chest[0] as Real,
        (chest[1] - 0.15) as Real,
        (chest[2] - 0.2) as Real,
    );
    let binds = SoftBodyInvBind::from_pose(&pose, cape_position, Quat::IDENTITY).unwrap();
    // The attached row is kinematic, so the edges solved after the skin cannot move it.
    let cloth = Cloth::new(4, 0.1).pin(&[0, 1, 2, 3]);
    let mut builder = cloth
        .builder()
        .inv_binds(binds.iter().copied())
        .create_constraints(
            SoftBodyBendType::Distance,
            SoftBodyVertexAttributes::default(),
        );
    for x in 0..cloth.side {
        builder = builder.skinned(SoftBodySkinned {
            max_distance: Some(0.0),
            ..SoftBodySkinned::new(cloth.index(x, 0), only_weight(CHEST as u32))
        });
    }
    let shared = builder.build().unwrap();
    let mut cape_world = world(GRAVITY, 1);
    let cape = cape_world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default().position(cape_position),
        )
        .unwrap();
    let top_row: Vec<usize> = (0..cloth.side)
        .map(|x| cloth.index(x, 0) as usize)
        .collect();
    let hard_row = |world: &PhysicsWorld| -> Vec<V3> {
        let vertices = world.soft_body(cape).unwrap().vertices();
        top_row.iter().map(|&i| v3(vertices[i].position)).collect()
    };
    let at_start = hard_row(&cape_world);
    // Before the ragdoll moves the attached row stays where it was made.
    for _ in 0..10 {
        let pose = ragdoll_world.ragdoll(ragdoll).unwrap().pose();
        skin(&mut cape_world, cape, &pose).unwrap();
        step(&mut cape_world, 1);
    }
    for (now, then) in hard_row(&cape_world).iter().zip(&at_start) {
        assert!(norm(sub(*now, *then)) < 1.0e-4, "{now:?} vs {then:?}");
    }
    // Then it follows the chest.
    ragdoll_world
        .ragdoll_mut(ragdoll)
        .unwrap()
        .set_linear_and_angular_velocity(Vec3::new(1.0, 0.5, 0.0), Vec3::new(0.0, 1.0, 0.0))
        .unwrap();
    let chest_bind = binds[CHEST];
    for tick in 0..30 {
        step(&mut ragdoll_world, 1);
        let pose = ragdoll_world.ragdoll(ragdoll).unwrap().pose();
        skin(&mut cape_world, cape, &pose).unwrap();
        step(&mut cape_world, 1);
        let joint = joint_position(&pose, CHEST);
        let joint_rotation = pose.joints[CHEST].rotation;
        for (&index, now) in top_row.iter().zip(hard_row(&cape_world)) {
            let vertex = cloth.vertices[index].position;
            let expected = skinned_position(joint, joint_rotation, chest_bind, vertex);
            let error = norm(sub(now, expected));
            assert!(
                error < 1.0e-4,
                "tick {tick}: vertex {index} is {error} m off"
            );
        }
    }
    let moved = norm(sub(hard_row(&cape_world)[0], at_start[0]));
    assert!(moved > 0.3, "the cape moved only {moved} m");
}

/// The joint of the rollback scene at `tick`, or of its detour.
fn swaying_joint(tick: u32, detour: bool) -> SkeletonPose {
    let phase = 0.1 * f64::from(tick) + if detour { 1.3 } else { 0.0 };
    let joint = add(v3(POSITION), [0.3 * phase.sin(), 0.1 * phase.cos(), 0.0]);
    pose_from(&[(joint, quat_about(Y, 0.2 * phase.sin() as f32))])
}

#[test]
fn restore_and_replay_after_a_detour_is_exact() {
    let cloth = Cloth::new(4, 0.3);
    let shared = cloth
        .skinned(&[Some(0.0), Some(0.2)])
        .create_constraints(
            SoftBodyBendType::Distance,
            SoftBodyVertexAttributes::default(),
        )
        .build()
        .unwrap();
    let mut world = world(GRAVITY, 2);
    let id = world
        .create_soft_body(&shared, &SoftBodySettings::default().position(POSITION))
        .unwrap();
    let advance = |world: &mut PhysicsWorld, tick: u32, detour: bool| {
        skin(world, id, &swaying_joint(tick, detour)).unwrap();
        step(world, 1);
    };
    for tick in 0..20 {
        advance(&mut world, tick, false);
    }
    // Saved between a step and the next skinning call.
    let saved = world.save_state();
    let mut expected = Vec::new();
    for tick in 20..40 {
        advance(&mut world, tick, false);
        expected.push(vertex_bits(&world, id));
    }
    world.restore_state(&saved).unwrap();
    for tick in 20..30 {
        advance(&mut world, tick, true);
    }
    world
        .soft_body_mut(id)
        .unwrap()
        .hard_skin_vertices(&swaying_joint(0, true))
        .unwrap();
    world.restore_state(&saved).unwrap();
    for (replayed, tick) in expected.iter().zip(20..40) {
        advance(&mut world, tick, false);
        assert!(
            vertex_bits(&world, id) == *replayed,
            "replay differs at tick {tick}"
        );
    }
}
