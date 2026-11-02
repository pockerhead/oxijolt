//! Shape changes that wake only the neighbours in given regions
//! (`BodyMut::set_shape_waking_in`).

mod common;

use common::controls::*;
use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// Where the child added to the row lies: beside the row's middle, 0.3 m higher, so that it
/// pushes up a cube overhanging it.
const ADDED_CHILD: Vec3 = Vec3::new(0.0, 0.3, 2.0);

/// The half extent of every child.
const CHILD_HALF: Vec3 = Vec3::new(1.0, 0.5, 1.0);

/// A static compound of 41 boxes in a row along x from -40 to 40 and z from -1 to 1, their
/// tops at y = 0.5, and with `extra` one more box at [`ADDED_CHILD`].
fn row(extra: bool) -> Shape {
    let block = Shape::new_box(CHILD_HALF).unwrap();
    let mut positions: Vec<Vec3> = (0..41)
        .map(|i| Vec3::new(-40.0 + 2.0 * i as f32, 0.0, 0.0))
        .collect();
    if extra {
        positions.push(ADDED_CHILD);
    }
    let children: Vec<_> = positions
        .iter()
        .enumerate()
        .map(|(index, &position)| child(&block, position, index as u32))
        .collect();
    Shape::new_compound(&children).unwrap()
}

/// The world box of the added child.
fn added_child_region() -> (RVec3, RVec3) {
    let corner = |sign: f32| {
        RVec3::new(
            Real::from(ADDED_CHILD.x + sign * CHILD_HALF.x),
            Real::from(ADDED_CHILD.y + sign * CHILD_HALF.y),
            Real::from(ADDED_CHILD.z + sign * CHILD_HALF.z),
        )
    };
    (corner(-1.0), corner(1.0))
}

/// A box of half extent 1 m around `at`.
fn region_around(at: RVec3) -> (RVec3, RVec3) {
    (
        RVec3::new(at.x - 1.0, at.y - 1.0, at.z - 1.0),
        RVec3::new(at.x + 1.0, at.y + 1.0, at.z + 1.0),
    )
}

/// The row as a static body with three cubes asleep on it: far left (x = -30), near the added
/// child (x = 0, overhanging towards it) and far right (x = 30), created in that order. The
/// world records activation events, none pending.
fn scene(workers: u32) -> (PhysicsWorld, BodyId, [BodyId; 3]) {
    let mut world = world(GRAVITY, workers);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let floor = world
        .create_body(&row(false), &BodySettings::new_static())
        .unwrap();
    let cubes = [
        RVec3::new(-30.0, 1.0, 0.0),
        RVec3::new(0.0, 1.0, 0.9),
        RVec3::new(30.0, 1.0, 0.0),
    ]
    .map(|at| add_cube(&mut world, at));
    for cube in cubes {
        fall_asleep(&mut world, cube);
    }
    assert!(cubes.iter().all(|&c| world.body(c).unwrap().is_sleeping()));
    world.take_events();
    (world, floor, cubes)
}

/// The activation events since the last call.
fn activations(world: &mut PhysicsWorld) -> Vec<ActivationEvent> {
    world.take_events().activations
}

#[test]
fn a_region_around_the_added_child_wakes_only_the_near_body() {
    let (mut world, floor, [left, near, right]) = scene(1);
    world
        .body_mut(floor)
        .unwrap()
        .set_shape_waking_in(
            &row(true),
            None,
            Activation::DontActivate,
            &[added_child_region()],
        )
        .unwrap();
    assert_eq!(activations(&mut world), [ActivationEvent::Activated(near)]);
    let near_y = world.body(near).unwrap().position().y;
    step(&mut world, 60);
    for far in [left, right] {
        assert!(world.body(far).unwrap().is_sleeping());
    }
    // The added child, 0.3 m above the row, pushed the overhanging cube up.
    let lift = world.body(near).unwrap().position().y - near_y;
    assert!(lift > 0.05, "{lift}");

    // `set_shape` wakes every body near the whole row.
    let (mut world, floor, cubes) = scene(1);
    world
        .body_mut(floor)
        .unwrap()
        .set_shape(&row(true), None, Activation::DontActivate)
        .unwrap();
    assert_eq!(
        activations(&mut world),
        cubes.map(ActivationEvent::Activated)
    );
}

#[test]
fn an_empty_slice_wakes_no_neighbours() {
    let (mut world, floor, cubes) = scene(1);
    world
        .body_mut(floor)
        .unwrap()
        .set_shape_waking_in(&row(true), None, Activation::Activate, &[])
        .unwrap();
    assert!(activations(&mut world).is_empty());
    assert!(cubes.iter().all(|&c| world.body(c).unwrap().is_sleeping()));
    // The shape changed: a ray finds the added child.
    let hit = world
        .cast_ray(
            &RayCast::new(RVec3::new(0.0, 5.0, 2.5), Vec3::new(0.0, -10.0, 0.0)),
            &QueryFilter::new(),
        )
        .unwrap()
        .unwrap();
    assert_eq!(hit.body, floor);
}

#[test]
fn several_regions_wake_their_union_once_each_in_body_id_order() {
    let (mut world, floor, [left, near, right]) = scene(1);
    let left_at = world.body(left).unwrap().position();
    let right_at = world.body(right).unwrap().position();
    // Right before left, and the left cube in two overlapping regions.
    let regions = [
        region_around(right_at),
        region_around(left_at),
        (
            RVec3::new(left_at.x - 0.5, left_at.y, left_at.z),
            RVec3::new(left_at.x + 2.0, left_at.y + 2.0, left_at.z + 0.5),
        ),
    ];
    world
        .body_mut(floor)
        .unwrap()
        .set_shape_waking_in(&row(true), None, Activation::DontActivate, &regions)
        .unwrap();
    assert_eq!(
        activations(&mut world),
        [
            ActivationEvent::Activated(left),
            ActivationEvent::Activated(right)
        ]
    );
    assert!(world.body(near).unwrap().is_sleeping());
}

#[test]
fn an_invalid_region_refuses_and_changes_nothing() {
    let edge = limits::MAX_POSITION;
    let (lo, hi) = added_child_region();
    let invalid_regions = [
        (hi, lo),
        (RVec3::new(Real::NAN, 0.0, 0.0), hi),
        (lo, RVec3::new(0.0, Real::NAN, 0.0)),
        (lo, RVec3::new(2.0 * edge, 0.0, 0.0)),
        (RVec3::new(-2.0 * edge, 0.0, 0.0), hi),
    ];
    let (mut world, floor, cubes) = scene(1);
    let saved = world.save_state();
    for bad in invalid_regions {
        // A valid region before the bad one changes nothing either.
        let result = world.body_mut(floor).unwrap().set_shape_waking_in(
            &row(true),
            None,
            Activation::Activate,
            &[(lo, hi), bad],
        );
        assert!(invalid(result), "{bad:?}");
    }
    assert!(activations(&mut world).is_empty());
    assert!(cubes.iter().all(|&c| world.body(c).unwrap().is_sleeping()));
    // The shape is the old one: no added child under the ray.
    let hit = world
        .cast_ray(
            &RayCast::new(RVec3::new(0.0, 5.0, 2.5), Vec3::new(0.0, -10.0, 0.0)),
            &QueryFilter::new(),
        )
        .unwrap();
    assert!(hit.is_none(), "{hit:?}");
    // The structure epoch is the same: the earlier state still restores.
    world.restore_state(&saved).unwrap();
    // The edge itself is accepted.
    let at_edge = (RVec3::new(edge, edge, edge), RVec3::new(edge, edge, edge));
    world
        .body_mut(floor)
        .unwrap()
        .set_shape_waking_in(&row(true), None, Activation::Activate, &[at_edge])
        .unwrap();
}

#[test]
fn a_movable_body_keeps_its_own_activation_and_is_never_its_own_neighbour() {
    let mut world = world(Vec3::ZERO, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let settings = BodySettings::new_dynamic().activation(Activation::DontActivate);
    let small = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let body = world.create_body(&small, &settings).unwrap();
    let large = Shape::new_box(Vec3::new(0.6, 0.6, 0.6)).unwrap();
    let around_itself = region_around(RVec3::ZERO);
    let change = |world: &mut PhysicsWorld, activation, regions: &[(RVec3, RVec3)]| {
        world
            .body_mut(body)
            .unwrap()
            .set_shape_waking_in(&large, None, activation, regions)
            .unwrap();
        activations(world)
    };
    assert!(change(&mut world, Activation::DontActivate, &[]).is_empty());
    assert!(change(&mut world, Activation::DontActivate, &[around_itself]).is_empty());
    assert!(world.body(body).unwrap().is_sleeping());
    assert_eq!(
        change(&mut world, Activation::Activate, &[]),
        [ActivationEvent::Activated(body)]
    );
    assert!(!world.body(body).unwrap().is_sleeping());
}

/// Whether `set_shape` (`regions == false`) or `set_shape_waking_in` with a region enclosing
/// the old and new shape wakes a small cube asleep next to a static box that grows in y. The
/// cube lies where widening by the speculative contact distance in `f32` and in `f64` differ.
fn wakes_the_cube_at_the_rounding_boundary(regions: bool) -> bool {
    let mut world = world(Vec3::ZERO, 1);
    let old = Shape::new_box(Vec3::new(0.1, 0.5, 0.5)).unwrap();
    let new = Shape::new_box(Vec3::new(0.1, 0.6, 0.5)).unwrap();
    let floor = world
        .create_body(&old, &BodySettings::new_static())
        .unwrap();
    let cube = Shape::new_box(Vec3::new(0.005, 0.005, 0.005)).unwrap();
    let settings = BodySettings::new_dynamic()
        .mass(1.0)
        .position(RVec3::new(0.12500001, 0.0, 0.0))
        .activation(Activation::DontActivate);
    let cube = world.create_body(&cube, &settings).unwrap();
    let mut floor = world.body_mut(floor).unwrap();
    if regions {
        let half_x = Real::from(0.1_f32);
        let region = (
            RVec3::new(-half_x, -0.6, -0.5),
            RVec3::new(half_x, 0.6, 0.5),
        );
        floor
            .set_shape_waking_in(&new, None, Activation::DontActivate, &[region])
            .unwrap();
    } else {
        floor
            .set_shape(&new, None, Activation::DontActivate)
            .unwrap();
    }
    !world.body(cube).unwrap().is_sleeping()
}

#[test]
fn a_region_equal_to_the_enclosing_box_wakes_what_set_shape_wakes() {
    // Only `f64` positions put the cube between the two widenings.
    if std::mem::size_of::<Real>() != 8 {
        return;
    }
    assert!(wakes_the_cube_at_the_rounding_boundary(false));
    assert!(wakes_the_cube_at_the_rounding_boundary(true));
}

/// Two regions on the scene of `workers` workers, then 60 ticks: the activation events (raw
/// ids, the runs being different worlds) and every cube's pose, velocities and sleep flag.
fn region_wakes(workers: u32) -> Digest {
    let (mut world, floor, cubes) = scene(workers);
    let left_at = world.body(cubes[0]).unwrap().position();
    world
        .body_mut(floor)
        .unwrap()
        .set_shape_waking_in(
            &row(true),
            None,
            Activation::DontActivate,
            &[added_child_region(), region_around(left_at)],
        )
        .unwrap();
    let mut digest = Digest::new();
    let events = &mut digest.push().state;
    for event in activations(&mut world) {
        events.push(u8::from(matches!(event, ActivationEvent::Activated(_))));
        events.extend_from_slice(&event.body().to_raw().to_le_bytes());
    }
    for _ in 0..60 {
        step(&mut world, 1);
        let state = &mut digest.push().state;
        for &cube in &cubes {
            let (p, q, v, w) = motion(&world, cube);
            for c in [p.x, p.y, p.z] {
                state.extend_from_slice(&c.to_le_bytes());
            }
            for c in [q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z] {
                state.extend_from_slice(&c.to_le_bytes());
            }
            state.push(u8::from(world.body(cube).unwrap().is_sleeping()));
        }
    }
    digest
}

#[test]
#[ignore = "child process of the region wake determinism gate"]
fn region_wakes_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "regions");
    finish_child(&region_wakes(threads));
}

#[test]
fn region_wakes_are_the_same_with_one_and_four_workers() {
    let one = digest_in_child("region_wakes_child", "regions", 1, "");
    let four = digest_in_child("region_wakes_child", "regions", 4, "");
    assert_eq!(one.ticks.len(), 61);
    assert_eq!(one.ticks[0].state.len(), 2 * 5, "two activations");
    assert_same("1 vs 4 workers in two processes", &one, &four);
}
