//! Shape changes that wake only the neighbours in given regions
//! (`BodyMut::set_shape_waking_in`).

mod common;

use common::controls::*;
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
fn region_wakes_are_the_same_with_one_and_four_workers() {
    let run = |workers: u32| {
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
        // Raw ids: the two runs are different worlds.
        let mut record: Vec<String> = activations(&mut world)
            .iter()
            .map(|event| {
                let activated = matches!(event, ActivationEvent::Activated(_));
                format!("{activated} {}", event.body().to_raw())
            })
            .collect();
        for _ in 0..60 {
            step(&mut world, 1);
            for &cube in &cubes {
                let (p, q, v, w) = motion(&world, cube);
                let sleeping = world.body(cube).unwrap().is_sleeping();
                let p: [Real; 3] = p.into();
                let bits = [q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z].map(f32::to_bits);
                record.push(format!("{p:?} {bits:?} {sleeping}"));
            }
        }
        record
    };
    let single = run(1);
    assert_eq!(single.len(), 2 + 60 * 3);
    assert_eq!(single, run(4));
}
