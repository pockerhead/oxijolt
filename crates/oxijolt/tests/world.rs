//! World lifetime, settings validation, gravity, stepping and threads.

mod common;

use common::math::bits;
use common::*;
use oxijolt::*;

fn assert_send_sync<T: Send + Sync>() {}

#[test]
fn world_and_shape_are_send_and_sync() {
    assert_send_sync::<PhysicsWorld>();
    assert_send_sync::<Shape>();
    assert_send_sync::<BodyId>();
    assert_send_sync::<RayCast>();
    assert_send_sync::<RayCastHit>();
    assert_send_sync::<SubShapeId>();
    assert_send_sync::<CompoundSubShape>();
    assert_send_sync::<HeightFieldSettings>();
    assert_send_sync::<BodySettings>();
    assert_send_sync::<SoftBodySettings>();
    assert_send_sync::<CollisionGroup>();
    assert_send_sync::<GroupFilterTable>();
    assert_send_sync::<ContactCandidate>();
    assert_send_sync::<BodyVelocity>();
    assert_send_sync::<CharacterContactKey>();
    fn assert_send<T: Send>() {}
    assert_send::<GroupFilterTableBuilder>();
}

#[test]
fn default_world_steps() {
    let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    for _ in 0..10 {
        assert!(world.step(1.0 / 60.0).unwrap().is_complete());
    }
    assert_eq!(world.body_count(), 0);
}

#[test]
fn invalid_settings_are_rejected() {
    let invalid = [
        WorldSettings::default().worker_threads(0),
        WorldSettings::default().worker_threads(65),
        WorldSettings::default().worker_threads(u32::MAX),
        WorldSettings::default().max_bodies(0),
        WorldSettings::default().max_bodies((1 << 23) + 1),
        WorldSettings::default().max_body_pairs(0),
        WorldSettings::default().max_contact_constraints(0),
        WorldSettings::default().temp_allocator_size(0),
        WorldSettings::default().gravity(Vec3::new(0.0, f32::NAN, 0.0)),
        WorldSettings::default().layers(CollisionLayers::new(1)),
    ];
    for settings in invalid {
        let result = PhysicsWorld::new(settings.clone());
        assert!(
            matches!(
                result,
                Err(WorldError::InvalidValue(_) | WorldError::InvalidLayers(_))
            ),
            "{settings:?} was accepted"
        );
    }
}

#[test]
fn gravity_round_trips_including_zero() {
    for gravity in [Vec3::ZERO, Vec3::new(0.1, -9.81, 3.5)] {
        let mut world = PhysicsWorld::new(WorldSettings::default().gravity(gravity)).unwrap();
        assert_eq!(bits(world.gravity()), bits(gravity));
        assert!(world.step(1.0 / 60.0).unwrap().is_complete());

        let other = Vec3::new(-1.25, 0.0, 7.0);
        world.set_gravity(other).unwrap();
        assert_eq!(bits(world.gravity()), bits(other));

        assert!(world.set_gravity(Vec3::new(f32::NAN, 0.0, 0.0)).is_err());
        assert_eq!(bits(world.gravity()), bits(other));
    }
}

#[test]
fn a_default_world_reads_back_jolts_default_limits_and_layers() {
    let world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    assert_eq!(world.max_bodies(), 10240);
    assert_eq!(world.max_body_pairs(), 65536);
    assert_eq!(world.max_contact_constraints(), 10240);
    assert_eq!(world.temp_allocator_size(), 10 * 1024 * 1024);
    assert_eq!(world.layers(), &CollisionLayers::default());
}

#[test]
fn a_world_reads_back_the_limits_and_layers_it_was_created_with() {
    let mut layers = CollisionLayers::new(2);
    let ground = layers.add_object_layer(BroadPhaseLayer::new(0));
    let debris = layers.add_object_layer(BroadPhaseLayer::new(1));
    let player = layers.add_object_layer(BroadPhaseLayer::new(1));
    layers
        .enable_collision(ground, debris)
        .enable_collision(ground, player);
    let world = PhysicsWorld::new(
        WorldSettings::default()
            .max_bodies(777)
            .max_body_pairs(1234)
            .max_contact_constraints(4321)
            .temp_allocator_size(3 * 1024 * 1024)
            .layers(layers.clone()),
    )
    .unwrap();
    assert_eq!(world.max_bodies(), 777);
    assert_eq!(world.max_body_pairs(), 1234);
    assert_eq!(world.max_contact_constraints(), 4321);
    assert_eq!(world.temp_allocator_size(), 3 * 1024 * 1024);
    assert_eq!(world.layers(), &layers);
    assert_ne!(world.layers(), &CollisionLayers::default());
}

#[test]
fn step_rejects_bad_delta_time() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let cube = add_cube(&mut world, RVec3::new(0.0, 2.0, 0.0));
    let mut before = Vec::new();
    record_body(&world, cube, &mut before);
    for dt in [0.0, -0.0, -1.0 / 60.0, f32::NAN, f32::INFINITY] {
        assert_eq!(world.step(dt), Err(StepError::InvalidDeltaTime));
        let mut after = Vec::new();
        record_body(&world, cube, &mut after);
        assert_eq!(after, before, "the world advanced with dt = {dt}");
    }
}

#[test]
fn step_rejects_delta_time_above_the_bound() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let cube = add_cube(&mut world, RVec3::new(0.0, 2.0, 0.0));
    let mut before = Vec::new();
    record_body(&world, cube, &mut before);
    let just_above = f32::from_bits(PhysicsWorld::MAX_DELTA_TIME.to_bits() + 1);
    for dt in [f32::MAX, 1.0e30, 2.0, just_above] {
        assert_eq!(world.step(dt), Err(StepError::InvalidDeltaTime));
        let mut after = Vec::new();
        record_body(&world, cube, &mut after);
        assert_eq!(after, before, "the world advanced with dt = {dt}");
    }
    assert!(world.step(PhysicsWorld::MAX_DELTA_TIME).is_ok());
    let position: [Real; 3] = world.body(cube).unwrap().position().into();
    assert!(
        position.iter().all(|value| value.is_finite()),
        "{position:?}"
    );
}

#[test]
fn step_rejects_delta_time_below_the_bound() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let cube = add_cube(&mut world, RVec3::new(0.0, 2.0, 0.0));
    let mut before = Vec::new();
    record_body(&world, cube, &mut before);
    let just_below = f32::from_bits(PhysicsWorld::MIN_DELTA_TIME.to_bits() - 1);
    for dt in [
        f32::from_bits(1),
        f32::MIN_POSITIVE,
        PhysicsWorld::MIN_DELTA_TIME / 2.0,
        just_below,
    ] {
        assert_eq!(world.step(dt), Err(StepError::InvalidDeltaTime));
        let mut after = Vec::new();
        record_body(&world, cube, &mut after);
        assert_eq!(after, before, "the world advanced with dt = {dt}");
    }
    assert!(world.step(PhysicsWorld::MIN_DELTA_TIME).is_ok());
    let position: [Real; 3] = world.body(cube).unwrap().position().into();
    assert!(
        position.iter().all(|value| value.is_finite()),
        "{position:?}"
    );
}

/// A floor with a row of resting cubes, each touching only the floor, and one cube falling
/// from above. Returns the falling cube.
fn crowded_floor(world: &mut PhysicsWorld) -> BodyId {
    add_floor(world);
    for i in 0..16 {
        add_cube(world, RVec3::new(i as Real * 2.0 - 15.0, 0.5, 0.0));
    }
    add_cube(world, RVec3::new(0.0, 10.0, 5.0))
}

/// Steps until a report is incomplete and returns it, checking that the falling cube moved
/// during that incomplete step.
fn step_until_incomplete(world: &mut PhysicsWorld, falling: BodyId) -> StepReport {
    for _ in 0..30 {
        let before = world.body(falling).unwrap().position().y;
        let report = world.step(DT).unwrap();
        if !report.is_complete() {
            assert!(world.body(falling).unwrap().position().y < before);
            return report;
        }
    }
    panic!("no step overflowed a buffer");
}

#[test]
fn full_contact_constraint_buffer_is_reported_and_the_world_advances() {
    let mut world = PhysicsWorld::new(WorldSettings::default().max_contact_constraints(1)).unwrap();
    let falling = crowded_floor(&mut world);
    let report = step_until_incomplete(&mut world, falling);
    assert!(report.contact_constraints_full, "{report:?}");
}

#[test]
fn full_body_pair_cache_is_reported_and_the_world_advances() {
    let mut world = PhysicsWorld::new(
        WorldSettings::default()
            .max_body_pairs(1)
            .max_contact_constraints(1),
    )
    .unwrap();
    let falling = crowded_floor(&mut world);
    let report = step_until_incomplete(&mut world, falling);
    assert!(report.body_pair_cache_full, "{report:?}");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn worlds_are_created_and_dropped_from_many_threads() {
    std::thread::scope(|scope| {
        for _ in 0..4 {
            scope.spawn(|| {
                for _ in 0..25 {
                    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
                    let id = add_cube(&mut world, RVec3::new(0.0, 2.0, 0.0));
                    assert!(world.step(DT).unwrap().is_complete());
                    assert!(world.body(id).unwrap().position().y < 2.0);
                }
            });
        }
    });
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn world_is_readable_from_many_threads() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    let ids = build_stacks(&mut world);
    step(&mut world, 30);

    let read_all = |world: &PhysicsWorld| {
        let mut digest = Vec::new();
        for &id in &ids {
            record_body(world, id, &mut digest);
        }
        digest
    };
    let expected = read_all(&world);
    let shared = &world;
    std::thread::scope(|scope| {
        let readers: Vec<_> = (0..4)
            .map(|_| scope.spawn(|| (0..50).map(|_| read_all(shared)).collect::<Vec<_>>()))
            .collect();
        for reader in readers {
            for digest in reader.join().unwrap() {
                assert_eq!(digest, expected);
            }
        }
    });
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn rays_are_cast_from_many_threads() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    for i in 0..20 {
        let position = RVec3::new(
            (i % 5) as Real * 2.0 - 4.0,
            0.5,
            (i / 5) as Real * 2.0 - 3.0,
        );
        world
            .create_body(&unit_box, &BodySettings::new_static().position(position))
            .unwrap();
    }
    let rays: Vec<RayCast> = (0..100)
        .map(|i| {
            let x = (i % 10) as Real - 4.5;
            let z = (i / 10) as Real - 4.5;
            RayCast::new(RVec3::new(x, 5.0, z), Vec3::new(0.1, -10.0, 0.05))
        })
        .collect();
    let cast_all = |world: &PhysicsWorld| -> Vec<(BodyId, u32)> {
        rays.iter()
            .map(|&ray| {
                let hit = world
                    .cast_ray(&ray, &QueryFilter::new())
                    .unwrap()
                    .expect("every ray hits the floor");
                (hit.body, hit.fraction.to_bits())
            })
            .collect()
    };
    let expected = cast_all(&world);
    let shared = &world;
    std::thread::scope(|scope| {
        let casters: Vec<_> = (0..4).map(|_| scope.spawn(|| cast_all(shared))).collect();
        for caster in casters {
            assert_eq!(caster.join().unwrap(), expected);
        }
    });
}

/// Every world collection lists its ids as an iterator. The body ids are copied before
/// `body_ids` returns, so the world can be changed while they are iterated.
#[test]
fn ids_are_iterators() {
    fn ids<T>(iterator: impl Iterator<Item = T>) -> Vec<T> {
        iterator.collect()
    }
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    for id in world.body_ids() {
        world.body_mut(id).unwrap().activate();
        assert!(world.contains_body(id));
    }
    assert_eq!(ids(world.body_ids()), [floor]);
    assert_eq!(ids(world.character_ids()), []);
    assert_eq!(ids(world.constraint_ids()), []);
    assert_eq!(ids(world.constraints_of_body(floor)), []);
    assert_eq!(ids(world.vehicle_ids()), []);
    assert_eq!(ids(world.ragdoll_ids()), []);
    assert_eq!(world.constraint_count(), 0_u32);
}
