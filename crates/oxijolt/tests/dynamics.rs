//! Caller-applied gravity, continuous collision detection and independent worlds.

mod common;

use common::*;
use oxijolt::*;

fn normalize(v: Vec3) -> Vec3 {
    let length = length(v);
    Vec3::new(v.x / length, v.y / length, v.z / length)
}

// `Real` is `f64` with the `double-precision` feature, so the casts are not always no-ops.
#[allow(clippy::unnecessary_cast)]
fn to_vec3(p: RVec3) -> Vec3 {
    Vec3::new(p.x as f32, p.y as f32, p.z as f32)
}

/// A small item falls onto a static box under a radial pull the caller applies every tick
/// (engine gravity is zero) and comes to rest on it.
#[test]
fn item_settles_under_caller_radial_gravity() {
    const MASS: f32 = 1.2;
    let planet_centre = Vec3::new(0.0, -100.0, 0.0);
    let mut world = world(Vec3::ZERO, 1);
    world
        .create_body(
            &Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap(),
            &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
        )
        .unwrap();
    let item = world
        .create_body(
            &Shape::new_box(Vec3::new(0.35, 0.06, 0.04)).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 2.0, 0.0))
                .mass(MASS)
                .friction(0.8)
                .restitution(0.1)
                .motion_quality(MotionQuality::LinearCast),
        )
        .unwrap();

    let mut calm_ticks = 0;
    let mut settled = false;
    for tick in 0..600 {
        let mut body = world.body_mut(item).unwrap();
        let position = to_vec3(body.position());
        let towards_centre = normalize(Vec3::new(
            planet_centre.x - position.x,
            planet_centre.y - position.y,
            planet_centre.z - position.z,
        ));
        let pull = MASS * 9.8;
        body.reset_forces();
        body.add_force(Vec3::new(
            towards_centre.x * pull,
            towards_centre.y * pull,
            towards_centre.z * pull,
        ))
        .unwrap();
        assert!(world.step(DT).unwrap().is_complete());

        let body = world.body(item).unwrap();
        if tick == 40 {
            assert!(body.position().y < 1.0, "the item is not falling");
        }
        let calm = length(body.linear_velocity()) < 0.05 && length(body.angular_velocity()) < 0.1;
        calm_ticks = if calm { calm_ticks + 1 } else { 0 };
        if calm_ticks == 30 {
            settled = true;
            break;
        }
    }
    assert!(settled, "the item did not come to rest within 600 ticks");
    let y = world.body(item).unwrap().position().y;
    assert!(
        (0.0..=0.35).contains(&y),
        "the item rests on the box top, y = {y}"
    );
}

/// Final z of a small sphere shot at 300 m/s through a 0.1 m thick static wall at z = 0.
fn shoot_through_wall(quality: MotionQuality) -> Real {
    const RADIUS: f32 = 0.1;
    let mut world = world(Vec3::ZERO, 1);
    world
        .create_body(
            &Shape::new_box(Vec3::new(2.0, 2.0, 0.05)).unwrap(),
            &BodySettings::new_static(),
        )
        .unwrap();
    let bullet = world
        .create_body(
            &Shape::new_sphere(RADIUS).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.0, -3.0))
                .linear_velocity(Vec3::new(0.0, 0.0, 300.0))
                .motion_quality(quality),
        )
        .unwrap();
    step(&mut world, 10);
    world.body(bullet).unwrap().position().z
}

#[test]
fn linear_cast_body_does_not_tunnel_through_thin_static() {
    let wall_back_plus_radius = 0.05 + 0.1;
    let discrete = shoot_through_wall(MotionQuality::Discrete);
    assert!(
        discrete > wall_back_plus_radius,
        "the discrete control did not tunnel (z = {discrete}), so the test proves nothing"
    );
    let linear_cast = shoot_through_wall(MotionQuality::LinearCast);
    assert!(
        linear_cast < wall_back_plus_radius,
        "the linear-cast body tunnelled to z = {linear_cast}"
    );
}

/// Builds the stacks scene in a fresh world with `worker_threads` and returns it with its ids.
fn stacks_world(worker_threads: u32) -> (PhysicsWorld, Vec<BodyId>) {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), worker_threads);
    let ids = build_stacks(&mut world);
    (world, ids)
}

const SIDE_BY_SIDE_TICKS: usize = 300;

#[test]
fn two_worlds_side_by_side_do_not_interfere() {
    let (mut a, ids_a) = stacks_world(1);
    let (mut b, ids_b) = stacks_world(4);
    let start: Vec<RVec3> = ids_a
        .iter()
        .map(|&id| a.body(id).unwrap().position())
        .collect();

    let mut digest_a = Vec::new();
    let mut digest_b = Vec::new();
    for tick in 0..SIDE_BY_SIDE_TICKS {
        assert!(a.step(DT).unwrap().is_complete());
        assert!(b.step(DT).unwrap().is_complete());
        let (before_a, before_b) = (digest_a.len(), digest_b.len());
        for (&id_a, &id_b) in ids_a.iter().zip(&ids_b) {
            record_body(&a, id_a, &mut digest_a);
            record_body(&b, id_b, &mut digest_b);
        }
        assert_eq!(
            digest_a[before_a..],
            digest_b[before_b..],
            "worlds diverged at tick {tick}"
        );
    }

    let (mut alone, ids_alone) = stacks_world(2);
    assert_eq!(
        run_digest(&mut alone, &ids_alone, SIDE_BY_SIDE_TICKS),
        digest_a,
        "a world stepped alone differs from the side-by-side worlds"
    );

    let moved = ids_a.iter().zip(&start).any(|(&id, from)| {
        let to = a.body(id).unwrap().position();
        let d = to_vec3(RVec3::new(to.x - from.x, to.y - from.y, to.z - from.z));
        length(d) > 0.1
    });
    assert!(
        moved,
        "the scene is static, so the comparison proves nothing"
    );
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn worlds_step_in_parallel_threads() {
    const TICKS: usize = 200;
    let sequential = {
        let (mut world, ids) = stacks_world(1);
        run_digest(&mut world, &ids, TICKS)
    };
    let digests: Vec<Vec<u8>> = std::thread::scope(|scope| {
        let runs: Vec<_> = [1, 4]
            .into_iter()
            .map(|workers| {
                scope.spawn(move || {
                    let (mut world, ids) = stacks_world(workers);
                    run_digest(&mut world, &ids, TICKS)
                })
            })
            .collect();
        runs.into_iter().map(|run| run.join().unwrap()).collect()
    });
    for digest in digests {
        assert_eq!(digest, sequential);
    }
}

/// The game's five layers: items collide with every layer, nothing else collides. Items and
/// actors are dropped onto terrain, structure and feature floors and onto a kinematic actor;
/// items land, actors fall through.
#[test]
fn collision_layers_decide_which_bodies_touch() {
    let mut layers = CollisionLayers::new(2);
    let terrain = layers.add_object_layer(BroadPhaseLayer::NON_MOVING);
    let structure = layers.add_object_layer(BroadPhaseLayer::NON_MOVING);
    let feature = layers.add_object_layer(BroadPhaseLayer::NON_MOVING);
    let item = layers.add_object_layer(BroadPhaseLayer::MOVING);
    let actor = layers.add_object_layer(BroadPhaseLayer::MOVING);
    for other in [terrain, structure, feature, item, actor] {
        layers.enable_collision(item, other);
    }
    let mut world = PhysicsWorld::new(
        WorldSettings::default()
            .gravity(Vec3::new(0.0, -9.81, 0.0))
            .layers(layers),
    )
    .unwrap();

    let floor = Shape::new_box(Vec3::new(1.5, 0.5, 1.5)).unwrap();
    let cube = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    let mut add = |shape: &Shape, settings: BodySettings, x: Real, y: Real, z: Real| {
        world
            .create_body(shape, &settings.position(RVec3::new(x, y, z)))
            .unwrap()
    };
    let mut landing = Vec::new();
    let mut falling = Vec::new();
    let floors = [
        (BodySettings::new_static(), terrain),
        (BodySettings::new_static(), structure),
        (BodySettings::new_static(), feature),
        (BodySettings::new_kinematic(), actor),
    ];
    for (i, (settings, layer)) in floors.into_iter().enumerate() {
        let x = 10.0 * i as Real;
        add(&floor, settings.object_layer(layer), x, -0.5, 0.0);
        let dynamic = BodySettings::new_dynamic();
        landing.push(add(&cube, dynamic.clone().object_layer(item), x, 0.5, -0.5));
        falling.push(add(&cube, dynamic.object_layer(actor), x, 0.5, 0.5));
    }
    let stacked = add(
        &cube,
        BodySettings::new_dynamic().object_layer(item),
        0.0,
        1.2,
        -0.5,
    );

    step(&mut world, 120);

    for id in landing {
        let y = world.body(id).unwrap().position().y;
        assert!(y > 0.1, "item {id:?} fell through, y = {y}");
    }
    let y = world.body(stacked).unwrap().position().y;
    assert!(
        y > 0.6,
        "the upper item fell through the lower one, y = {y}"
    );
    for id in falling {
        let y = world.body(id).unwrap().position().y;
        assert!(y < -5.0, "actor {id:?} was stopped, y = {y}");
    }
}

/// What a box sliding over a compound of touching boxes shows: the largest angular speed and the
/// largest vertical speed from tick 10 on (which skips the start's depenetration), and the final
/// x. The scene is the first one of Jolt's `EnhancedInternalEdgeRemovalTest` sample.
fn slide_over_compound(enhanced_internal_edge_removal: bool) -> (f32, f32, Real) {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let cell = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    let children: Vec<CompoundChild<'_>> = (-10..10)
        .flat_map(|x| (-10..10).map(move |z| (x, z)))
        .map(|(x, z)| CompoundChild {
            shape: &cell,
            position: Vec3::new(2.0 * x as f32, 0.0, 2.0 * z as f32),
            rotation: Quat::IDENTITY,
            user_data: 0,
        })
        .collect();
    world
        .create_body(
            &Shape::new_compound(&children).unwrap(),
            &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)),
        )
        .unwrap();
    let mover = world
        .create_body(
            &Shape::new_box(Vec3::new(2.0, 2.0, 2.0)).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(-18.0, 1.9, 0.0))
                .linear_velocity(Vec3::new(20.0, 0.0, 0.0))
                .enhanced_internal_edge_removal(enhanced_internal_edge_removal),
        )
        .unwrap();
    let (mut angular, mut vertical) = (0.0_f32, 0.0_f32);
    for tick in 0..60 {
        assert!(world.step(DT).unwrap().is_complete());
        if tick >= 10 {
            let body = world.body(mover).unwrap();
            angular = angular.max(length(body.angular_velocity()));
            vertical = vertical.max(body.linear_velocity().y.abs());
        }
    }
    let x = world.body(mover).unwrap().position().x;
    (angular, vertical, x)
}

/// Enhanced internal edge removal on a box sliding over the touching children of one static
/// compound: without it the box catches on the children's internal edges and is thrown upward,
/// with it it slides smoothly. Jolt removes internal edges only within one body's shape; seams between
/// separate chunk bodies are covered by the walker's seam test.
#[test]
fn enhanced_internal_edge_removal_smooths_sliding_over_a_compound() {
    let off = slide_over_compound(false);
    let on = slide_over_compound(true);
    // Measured largest vertical speeds: 7.02 m/s off, 0.00035 m/s on; final x -0.88 off, 0.52 on.
    assert!(off.1 > 3.5, "off {off:?}, on {on:?}");
    assert!(off.1 >= 3.0 * on.1, "off {off:?}, on {on:?}");
    assert!(on.1 < 0.01, "off {off:?}, on {on:?}");
    assert!(on.2 > off.2, "off {off:?}, on {on:?}");
}
