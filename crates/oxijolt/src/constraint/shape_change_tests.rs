//! A static body's shape change keeps the world attachment points of the constraints that use
//! it, read back from Jolt's constraint-to-body matrices, with the static body on either side.

use super::id::AnyConstraintId;
use super::*;
use crate::{
    Activation, BodyId, BodySettings, CompoundChild, PhysicsWorld, Quat, Real, Shape, WorldSettings,
};

/// Where the static body's origin lies: about 100 m out, so `f32` rounding shows.
const ORIGIN: RVec3 = RVec3::new(100.1, 5.3, -7.7);
/// Half extent of the static compound's boxes and of the dynamic cube, metres.
const HALF: f32 = 0.5;
/// How far a world attachment point may move per shape change: four `f32` ulps at 128 m. A
/// shape change rounds the body's centre of mass in world space and the local point in `f32`.
/// Measured in single precision: 3.8e-6 m after one change, 7.6e-6 m after three.
const ROUNDING_PER_CHANGE: f64 = 4.0 * 7.62939453125e-6;

/// Which side of the constraint the static body is on.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Side {
    One,
    Two,
}

/// The static body's rotation: half a radian about Y.
fn rotation() -> Quat {
    Quat::from_xyzw(0.0, 0.25f32.sin(), 0.0, 0.25f32.cos())
}

/// `v` turned by the unit quaternion `q`, in `f64`.
fn rotate(q: Quat, v: [f64; 3]) -> [f64; 3] {
    let (x, y, z, w) = (q.x as f64, q.y as f64, q.z as f64, q.w as f64);
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]
}

/// `value` in `f64`; `Real` is already `f64` with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn wide(value: Real) -> f64 {
    f64::from(value)
}

/// The point at `local` in the static body's frame, in world space.
fn to_world(local: [f64; 3]) -> RVec3 {
    let r = rotate(rotation(), local);
    RVec3::new(
        (wide(ORIGIN.x) + r[0]) as Real,
        (wide(ORIGIN.y) + r[1]) as Real,
        (wide(ORIGIN.z) + r[2]) as Real,
    )
}

/// A static compound of boxes centred at `xs` along its X axis.
fn boxes_at(xs: &[f32]) -> Shape {
    let cube = Shape::new_box(Vec3::new(HALF, HALF, HALF)).unwrap();
    let children: Vec<CompoundChild<'_>> = xs
        .iter()
        .map(|&x| CompoundChild {
            shape: &cube,
            position: Vec3::new(x, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 0,
        })
        .collect();
    Shape::new_compound(&children).unwrap()
}

/// The static compound's first shape: centre of mass at x = 2.
fn first_shape() -> Shape {
    boxes_at(&[0.0, 2.0, 4.0])
}

/// The world attachment point of `id` on body `side` (1 or 2): the translation of Jolt's
/// constraint-to-body matrix, which every kind with points keeps as the point relative to the
/// body's centre of mass, put through the body's centre-of-mass transform, in `f64`.
fn world_anchor(world: &PhysicsWorld, id: AnyConstraintId, side: usize) -> [f64; 3] {
    let entry = &world.constraints[&id.raw];
    let constraint: *const JPH_TwoBodyConstraint = entry.constraint.as_ptr().cast();
    // SAFETY: an all-zero matrix is valid floats; joltc overwrites it.
    let mut matrix: JPH_Mat4 = unsafe { std::mem::zeroed() };
    // SAFETY: the world owns the live constraint, every kind here is a two-body constraint, and
    // `matrix` is a live local; the getters only read the constraint.
    unsafe {
        if side == 1 {
            JPH_TwoBodyConstraint_GetConstraintToBody1Matrix(constraint, &mut matrix);
        } else {
            JPH_TwoBodyConstraint_GetConstraintToBody2Matrix(constraint, &mut matrix);
        }
    }
    let local = matrix.column[3];
    let body = world.body(entry.bodies[side - 1]).unwrap();
    let centre = body.center_of_mass_position();
    let turned = rotate(
        body.rotation(),
        [local.x as f64, local.y as f64, local.z as f64],
    );
    [
        wide(centre.x) + turned[0],
        wide(centre.y) + turned[1],
        wide(centre.z) + turned[2],
    ]
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// The nine constraint kinds a static body may join.
const KINDS: [ConstraintType; 9] = [
    ConstraintType::Fixed,
    ConstraintType::Point,
    ConstraintType::Distance,
    ConstraintType::Hinge,
    ConstraintType::Slider,
    ConstraintType::Cone,
    ConstraintType::SwingTwist,
    ConstraintType::SixDof,
    ConstraintType::Path,
];

/// The static compound ([`first_shape`]) at [`ORIGIN`] and a dynamic cube beyond its +X end,
/// centred at x = 5.2 along the static body's X axis.
fn bodies(world: &mut PhysicsWorld) -> [BodyId; 2] {
    let support = world
        .create_body(
            &first_shape(),
            &BodySettings::new_static()
                .position(ORIGIN)
                .rotation(rotation()),
        )
        .unwrap();
    let cube = Shape::new_box(Vec3::new(HALF, HALF, HALF)).unwrap();
    let dynamic = world
        .create_body(
            &cube,
            &BodySettings::new_dynamic()
                .position(to_world([5.2, 0.0, 0.0]))
                .rotation(rotation()),
        )
        .unwrap();
    [support, dynamic]
}

/// A world with [`bodies`] and one constraint of `kind` between them with the static body on
/// `side`.
fn scene(kind: ConstraintType, side: Side) -> (PhysicsWorld, BodyId, AnyConstraintId) {
    let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    let [support, dynamic] = bodies(&mut world);
    let [body1, body2] = match side {
        Side::One => [support, dynamic],
        Side::Two => [dynamic, support],
    };
    let point = to_world([4.6, 0.1, -0.2]);
    let x = Vec3::new(0.25f32.cos(), 0.0, -0.25f32.sin());
    let y = Vec3::new(0.0, 1.0, 0.0);
    let id: AnyConstraintId = match kind {
        ConstraintType::Fixed => world
            .create_constraint(body1, body2, &FixedConstraintSettings::new(point, x, y))
            .map(Into::into),
        ConstraintType::Point => world
            .create_constraint(body1, body2, &PointConstraintSettings::new(point))
            .map(Into::into),
        ConstraintType::Distance => {
            let on_support = to_world([4.4, 0.3, 0.1]);
            let on_cube = to_world([5.0, -0.2, 0.3]);
            let [p1, p2] = match side {
                Side::One => [on_support, on_cube],
                Side::Two => [on_cube, on_support],
            };
            world
                .create_constraint(body1, body2, &DistanceConstraintSettings::new(p1, p2))
                .map(Into::into)
        }
        ConstraintType::Hinge => world
            .create_constraint(body1, body2, &HingeConstraintSettings::new(point, y, x))
            .map(Into::into),
        ConstraintType::Slider => world
            .create_constraint(body1, body2, &SliderConstraintSettings::new(point, x, y))
            .map(Into::into),
        ConstraintType::Cone => world
            .create_constraint(body1, body2, &ConeConstraintSettings::new(point, x, 0.5))
            .map(Into::into),
        ConstraintType::SwingTwist => world
            .create_constraint(
                body1,
                body2,
                &SwingTwistConstraintSettings::new(point, x, y).half_cone_angles(0.5, 0.5),
            )
            .map(Into::into),
        ConstraintType::SixDof => world
            .create_constraint(body1, body2, &SixDofConstraintSettings::new(point, x, y))
            .map(Into::into),
        ConstraintType::Path => {
            let at = |x: f32| HermitePathPoint {
                position: Vec3::new(x, 0.0, 0.0),
                tangent: Vec3::new(1.0, 0.0, 0.0),
            };
            let path = HermitePath::new(y, vec![at(0.0), at(0.5), at(1.0)], false).unwrap();
            // The path starts at the cube's origin, in body 1's frame.
            let start = match side {
                Side::One => Vec3::new(5.2, 0.0, 0.0),
                Side::Two => Vec3::ZERO,
            };
            world
                .create_constraint(
                    body1,
                    body2,
                    &PathConstraintSettings::new(path).path_position(start),
                )
                .map(Into::into)
        }
        _ => unreachable!("not a kind a static body may join"),
    }
    .unwrap();
    (world, support, id)
}

/// Both world attachment points of `id`: the static body's, then the dynamic body's.
fn anchors(world: &PhysicsWorld, id: AnyConstraintId, side: Side) -> [[f64; 3]; 2] {
    match side {
        Side::One => [world_anchor(world, id, 1), world_anchor(world, id, 2)],
        Side::Two => [world_anchor(world, id, 2), world_anchor(world, id, 1)],
    }
}

/// Disables `id` in Jolt directly; reading attachment points needs no body wake.
fn disable(world: &mut PhysicsWorld, id: AnyConstraintId) {
    let constraint = world.constraints[&id.raw].constraint.as_ptr();
    // SAFETY: the world owns the live constraint and is borrowed mutably; no step runs. The
    // setter writes a member.
    unsafe { JPH_Constraint_SetEnabled(constraint, false) };
}

/// The largest move of the static body's world attachment point over every kind and side
/// after each shape of `shapes` in turn, one maximum per change; asserts that the dynamic
/// body's point does not move at all. With `disabled`, the constraints are disabled first.
fn anchor_moves(shapes: &[Shape], disabled: bool) -> Vec<f64> {
    let mut worst = vec![0.0f64; shapes.len()];
    for kind in KINDS {
        for side in [Side::One, Side::Two] {
            let (mut world, support, id) = scene(kind, side);
            let [support_before, dynamic_before] = anchors(&world, id, side);
            if disabled {
                disable(&mut world, id);
            }
            for (change, shape) in shapes.iter().enumerate() {
                world
                    .body_mut(support)
                    .unwrap()
                    .set_shape(shape, None, Activation::DontActivate)
                    .unwrap();
                let [support_after, dynamic_after] = anchors(&world, id, side);
                assert_eq!(
                    dynamic_after.map(f64::to_bits),
                    dynamic_before.map(f64::to_bits),
                    "{kind:?} with the static body on side {side:?}, change {change}"
                );
                let moved = distance(support_after, support_before);
                worst[change] = worst[change].max(moved);
            }
        }
    }
    worst
}

#[test]
fn a_static_shape_change_keeps_the_world_anchors_of_every_kind_on_either_side() {
    let moves = anchor_moves(&[boxes_at(&[2.0, 4.0])], false);
    println!("largest anchor move: {:e} m", moves[0]);
    assert!(moves[0] <= ROUNDING_PER_CHANGE, "{moves:?}");
}

#[test]
fn repeated_static_shape_changes_and_a_set_back_keep_the_world_anchors() {
    // Centres of mass at x = 3, about 3.2 with a far box, then back at 2.
    let shapes = [
        boxes_at(&[2.0, 4.0]),
        boxes_at(&[0.5, 2.0, 4.0, 6.3]),
        first_shape(),
    ];
    for disabled in [false, true] {
        let moves = anchor_moves(&shapes, disabled);
        println!("largest anchor moves (disabled: {disabled}): {moves:?} m");
        for (change, moved) in moves.iter().enumerate() {
            assert!(
                *moved <= ROUNDING_PER_CHANGE * (change + 1) as f64,
                "disabled: {disabled}, {moves:?}"
            );
        }
    }
}

/// Points given relative to the centres of mass refer to the centres of mass at creation: after
/// the static body's centre of mass moves, the world attachment points stay where they were.
#[test]
fn points_given_relative_to_the_centres_of_mass_stay_where_they_were_in_world_space() {
    // Along the static body's X axis its centre of mass is at x = 2 and the cube's at x = 5.2;
    // the point lies at x = 4.6.
    let point = to_world([4.6, 0.1, -0.2]);
    let point = [wide(point.x), wide(point.y), wide(point.z)];
    let local = |x: Real| RVec3::new(x, 0.1, -0.2);
    let (on_support, on_cube) = (local(2.6), local(-0.6));
    for side in [Side::One, Side::Two] {
        let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
        let [support, cube] = bodies(&mut world);
        let (body1, body2, point1, point2) = match side {
            Side::One => (support, cube, on_support, on_cube),
            Side::Two => (cube, support, on_cube, on_support),
        };
        let settings = PointConstraintSettings::default()
            .space(ConstraintSpace::LocalToBodyCom)
            .point1(point1)
            .point2(point2);
        let id = world
            .create_constraint(body1, body2, &settings)
            .unwrap()
            .into();
        let [before, _] = anchors(&world, id, side);
        assert!(distance(before, point) <= ROUNDING_PER_CHANGE, "{side:?}");
        world
            .body_mut(support)
            .unwrap()
            .set_shape(&boxes_at(&[2.0, 4.0]), None, Activation::DontActivate)
            .unwrap();
        let [after, _] = anchors(&world, id, side);
        assert!(
            distance(after, point) <= 2.0 * ROUNDING_PER_CHANGE,
            "{side:?}"
        );
    }
}

/// The measurement sees a missing notification: Jolt's own `SetShape` alone moves the static
/// body's centre of mass, and with it the attachment point, by the full change.
#[test]
fn without_the_notification_the_static_anchor_moves_with_the_centre_of_mass() {
    let shape = boxes_at(&[2.0, 4.0]);
    for kind in KINDS {
        for side in [Side::One, Side::Two] {
            let (world, support, id) = scene(kind, side);
            let [before, _] = anchors(&world, id, side);
            // SAFETY: the world is live and holds the body, a static rigid body; `shape` is live
            // for the call and the body takes its own reference. This thread holds no body lock.
            unsafe {
                JPH_BodyInterface_SetShape(
                    world.body_interface.as_ptr(),
                    support.to_raw(),
                    shape.as_ptr(),
                    false,
                    Activation::DontActivate.to_jph(),
                )
            };
            let [after, _] = anchors(&world, id, side);
            let moved = distance(after, before);
            assert!((moved - 1.0).abs() < 1e-4, "{kind:?} {side:?}: {moved}");
        }
    }
}
