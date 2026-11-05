use oxijolt_sys::*;

use super::{BodyMotionState, BodyPose};
use crate::*;

fn world() -> PhysicsWorld {
    PhysicsWorld::new(WorldSettings::default()).unwrap()
}

fn add_floor(world: &mut PhysicsWorld) -> BodyId {
    let shape = Shape::new_box(Vec3::new(50.0, 1.0, 50.0)).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)),
        )
        .unwrap()
}

fn add_cube(world: &mut PhysicsWorld, position: RVec3, activation: Activation) -> BodyId {
    let shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_dynamic()
                .position(position)
                .activation(activation),
        )
        .unwrap()
}

/// Jolt's own list of the awake rigid bodies, in Jolt's order.
fn native_active_rigid_bodies(world: &PhysicsWorld) -> Vec<u32> {
    let system = world.system.as_ptr();
    // SAFETY: the system is this live world's, and nothing changes the active list during the
    // shared borrow; the count is read first and the copy writes at most `count` ids.
    unsafe {
        let count = JPH_PhysicsSystem_GetNumActiveBodies(system, JPH_BodyType_Rigid);
        let mut ids = vec![0; count as usize];
        JPH_PhysicsSystem_GetActiveBodies(system, JPH_BodyType_Rigid, ids.as_mut_ptr(), count);
        ids
    }
}

fn pose_ids(world: &PhysicsWorld) -> Vec<BodyId> {
    world
        .active_body_poses()
        .iter()
        .map(|pose| pose.id)
        .collect()
}

#[test]
fn poses_come_in_body_id_order_whatever_order_jolt_woke_them() {
    let mut world = world();
    add_floor(&mut world);
    let [a, b, c] = [-1.0, 0.0, 1.0].map(|x| {
        add_cube(
            &mut world,
            RVec3::new(x, 1.0, 0.0),
            Activation::DontActivate,
        )
    });
    assert!(world.active_body_poses().is_empty());
    // A non-zero velocity wakes the body (Jolt's `BodyInterface::SetLinearVelocity`).
    for id in [c, b, a] {
        world
            .body_mut(id)
            .unwrap()
            .set_linear_velocity(Vec3::new(0.0, 0.0, 1.0))
            .unwrap();
    }
    let raw = |ids: [BodyId; 3]| ids.map(BodyId::to_raw).to_vec();
    assert_eq!(native_active_rigid_bodies(&world), raw([c, b, a]));
    assert_eq!(pose_ids(&world), vec![a, b, c]);
}

/// The ids of `ids` whose bodies are awake, sorted, by the per-body readout.
fn awake_by_body(world: &PhysicsWorld, ids: &[BodyId]) -> Vec<BodyId> {
    let mut awake: Vec<BodyId> = ids
        .iter()
        .copied()
        .filter(|&id| world.body(id).unwrap().is_active())
        .collect();
    awake.sort_unstable();
    awake
}

fn assert_bit_equal_to_body(world: &PhysicsWorld, pose: &BodyPose) {
    let body = world.body(pose.id).unwrap();
    let (position, rotation) = (body.position(), body.rotation());
    let position_bits = |p: RVec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
    let rotation_bits = |q: Quat| [q.x.to_bits(), q.y.to_bits(), q.z.to_bits(), q.w.to_bits()];
    assert_eq!(
        position_bits(pose.position),
        position_bits(position),
        "{pose:?}"
    );
    assert_eq!(
        rotation_bits(pose.rotation),
        rotation_bits(rotation),
        "{pose:?}"
    );
}

fn about_x(angle: f32) -> Quat {
    let (sin, cos) = (angle / 2.0).sin_cos();
    Quat::from_xyzw(sin, 0.0, 0.0, cos)
}

fn about_y(angle: f32) -> Quat {
    let (sin, cos) = (angle / 2.0).sin_cos();
    Quat::from_xyzw(0.0, sin, 0.0, cos)
}

fn cloth(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    let mut vertices = Vec::new();
    for z in 0..3 {
        for x in 0..3 {
            vertices.push(SoftBodyVertex::new(Vec3::new(
                0.5 * x as f32,
                0.0,
                0.5 * z as f32,
            )));
        }
    }
    let mut faces = Vec::new();
    for z in 0..2 {
        for x in 0..2 {
            let i = z * 3 + x;
            faces.push([i, i + 3, i + 4]);
            faces.push([i, i + 4, i + 1]);
        }
    }
    let shared = SoftBodySharedSettings::builder(vertices, faces)
        .create_constraints(
            SoftBodyBendType::Distance,
            SoftBodyVertexAttributes::default(),
        )
        .build()
        .unwrap();
    world
        .create_soft_body(&shared, &SoftBodySettings::default().position(position))
        .unwrap()
}

/// A floor, a cube in a reused slot, a compound with an offset centre of mass, a moving kinematic
/// body, a character's inner body, a cloth and a sleeping cube, in that order.
fn mixed_scene(world: &mut PhysicsWorld) -> [BodyId; 7] {
    let floor = add_floor(world);

    // A slot removed and filled again, so that one raw id carries a bumped sequence number.
    let removed = add_cube(world, RVec3::new(4.0, 1.0, 0.0), Activation::Activate);
    world.remove_body(removed).unwrap();
    let reused = add_cube(world, RVec3::new(4.0, 1.0, 0.0), Activation::Activate);
    assert_eq!(reused.index(), removed.index());
    assert!(reused.sequence() > removed.sequence());

    // A body whose centre of mass is away from its origin and whose child is rotated.
    let cube = Shape::new_box(Vec3::new(0.2, 0.3, 0.4)).unwrap();
    let compound = Shape::new_compound(&[CompoundChild {
        shape: &cube,
        position: Vec3::new(0.3, 0.2, -0.1),
        rotation: about_y(0.7),
        user_data: 0,
    }])
    .unwrap();
    let offset = world
        .create_body(
            &compound,
            &BodySettings::new_dynamic()
                .position(RVec3::new(-3.0, 2.0, 1.0))
                .rotation(about_x(0.3)),
        )
        .unwrap();
    let kinematic = world
        .create_body(
            &cube,
            &BodySettings::new_kinematic()
                .position(RVec3::new(0.0, 3.0, -4.0))
                .linear_velocity(Vec3::new(0.5, 0.0, 0.0))
                .angular_velocity(Vec3::new(0.0, 0.4, 0.0)),
        )
        .unwrap();
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let character = world
        .create_character(
            &CharacterSettings::new(&capsule).inner_body(Some(InnerBody {
                shape: &capsule,
                object_layer: ObjectLayer::MOVING,
            })),
            RVec3::new(6.0, 1.0, 6.0),
            Quat::IDENTITY,
        )
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    let soft = cloth(world, RVec3::new(-6.0, 2.0, -6.0));
    let asleep = add_cube(world, RVec3::new(8.0, 1.0, 0.0), Activation::DontActivate);

    [floor, reused, offset, kinematic, inner, soft, asleep]
}

#[test]
fn poses_match_the_per_body_readout_bit_for_bit() {
    let mut world = world();
    let created = mixed_scene(&mut world);
    let [floor, reused, offset, kinematic, inner, soft, asleep] = created;
    for step in [false, true] {
        if step {
            assert!(world.step(1.0 / 60.0).unwrap().is_complete());
        }
        let poses = world.active_body_poses();
        let ids: Vec<BodyId> = poses.iter().map(|pose| pose.id).collect();
        assert_eq!(ids, awake_by_body(&world, &created));
        for absent in [floor, asleep] {
            assert!(!ids.contains(&absent), "{absent:?}");
        }
        for present in [reused, offset, kinematic, inner, soft] {
            assert!(ids.contains(&present), "{present:?}");
        }
        for pose in &poses {
            assert_bit_equal_to_body(&world, pose);
        }
    }
}

fn vec3_bits(v: Vec3) -> [u32; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

fn assert_state_bit_equal_to_body(world: &PhysicsWorld, state: &BodyMotionState) {
    let pose = BodyPose {
        id: state.id,
        position: state.position,
        rotation: state.rotation,
    };
    assert_bit_equal_to_body(world, &pose);
    let body = world.body(state.id).unwrap();
    assert_eq!(
        vec3_bits(state.linear_velocity),
        vec3_bits(body.linear_velocity()),
        "{state:?}"
    );
    assert_eq!(
        vec3_bits(state.angular_velocity),
        vec3_bits(body.angular_velocity()),
        "{state:?}"
    );
}

#[test]
fn states_match_the_per_body_readout_and_the_poses_bit_for_bit() {
    let mut world = world();
    let created = mixed_scene(&mut world);
    let [floor, _, offset, kinematic, _, soft, asleep] = created;
    for step in [false, true] {
        if step {
            assert!(world.step(1.0 / 60.0).unwrap().is_complete());
        }
        let states = world.active_body_states();
        let ids: Vec<BodyId> = states.iter().map(|state| state.id).collect();
        assert_eq!(ids, awake_by_body(&world, &created));
        assert_eq!(ids, pose_ids(&world));
        for absent in [floor, asleep] {
            assert!(!ids.contains(&absent), "{absent:?}");
        }
        for (state, pose) in states.iter().zip(world.active_body_poses()) {
            assert_eq!(
                (state.position, state.rotation),
                (pose.position, pose.rotation)
            );
            assert_state_bit_equal_to_body(&world, state);
        }
        let state = |id| *states.iter().find(|state| state.id == id).unwrap();
        assert_eq!(state(kinematic).linear_velocity, Vec3::new(0.5, 0.0, 0.0));
        assert_eq!(state(kinematic).angular_velocity, Vec3::new(0.0, 0.4, 0.0));
        if step {
            assert!(state(offset).linear_velocity.y < 0.0);
            assert!(state(soft).linear_velocity.y < 0.0);
        }
    }
}

#[test]
fn poses_into_clears_and_reuses_the_buffer() {
    let mut world = world();
    add_floor(&mut world);
    for x in 0..3 {
        add_cube(
            &mut world,
            RVec3::new(x as Real, 2.0, 0.0),
            Activation::Activate,
        );
    }
    let mut out = world.active_body_poses();
    out.extend_from_within(..);
    assert_eq!(out.len(), 6);
    world.active_body_poses_into(&mut out);
    assert_eq!(out, world.active_body_poses());
    let (capacity, pointer) = (out.capacity(), out.as_ptr());
    world.active_body_poses_into(&mut out);
    assert_eq!(out.len(), 3);
    assert_eq!((out.capacity(), out.as_ptr()), (capacity, pointer));

    let empty = self::world();
    empty.active_body_poses_into(&mut out);
    assert!(out.is_empty());

    world.active_body_poses_into(&mut out);
    assert_eq!(out.len(), 3);
    let mut static_only = self::world();
    add_floor(&mut static_only);
    static_only.active_body_poses_into(&mut out);
    assert!(out.is_empty());
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn concurrent_shared_reads_and_saves_agree() {
    let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO)).unwrap();
    let shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    let ids: Vec<BodyId> = (0..300)
        .map(|i| {
            let (x, y, z) = ((i % 10) as Real, ((i / 10) % 10) as Real, (i / 100) as Real);
            world
                .create_body(
                    &shape,
                    &BodySettings::new_dynamic()
                        .position(RVec3::new(x, y, z))
                        .allow_sleeping(false),
                )
                .unwrap()
        })
        .collect();
    assert!(world.step(1.0 / 60.0).unwrap().is_complete());
    let expected = world.active_body_poses();
    assert_eq!(expected.len(), ids.len());

    let world = &world;
    let expected = &expected;
    let ids = &ids;
    std::thread::scope(|scope| {
        for reader in 0..3 {
            scope.spawn(move || {
                for round in 0..200 {
                    assert_eq!(&world.active_body_poses(), expected);
                    for &id in ids.iter().skip(reader + round % 7).step_by(50) {
                        let pose = expected.iter().find(|pose| pose.id == id).unwrap();
                        assert_eq!(world.body(id).unwrap().position(), pose.position);
                    }
                    let ray = RayCast::new(RVec3::new(-5.0, 0.0, 0.0), Vec3::new(20.0, 0.0, 0.0));
                    assert!(world.cast_ray(&ray, &QueryFilter::new()).unwrap().is_some());
                }
            });
        }
        scope.spawn(move || {
            for _ in 0..200 {
                world.save_state();
            }
        });
    });
}

#[test]
fn each_call_reads_the_awake_set_again_into_the_same_buffer() {
    let mut world = world();
    add_floor(&mut world);
    let kept = add_cube(&mut world, RVec3::new(-2.0, 2.0, 0.0), Activation::Activate);
    let removed = add_cube(&mut world, RVec3::new(2.0, 2.0, 0.0), Activation::Activate);
    let mut out = Vec::new();
    world.active_body_poses_into(&mut out);
    assert_eq!(
        out.iter().map(|pose| pose.id).collect::<Vec<_>>(),
        [kept, removed]
    );

    world.remove_body(removed).unwrap();
    let reused = add_cube(&mut world, RVec3::new(0.0, 4.0, 0.0), Activation::Activate);
    assert_eq!(reused.index(), removed.index());
    world.active_body_poses_into(&mut out);
    assert_eq!(
        out.iter().map(|pose| pose.id).collect::<Vec<_>>(),
        [kept, reused]
    );
    assert_eq!(out[1].position, RVec3::new(0.0, 4.0, 0.0));
}

#[test]
fn a_world_with_only_soft_bodies_awake_reads_them() {
    let mut world = world();
    add_floor(&mut world);
    add_cube(
        &mut world,
        RVec3::new(4.0, 1.0, 0.0),
        Activation::DontActivate,
    );
    let soft = cloth(&mut world, RVec3::new(0.0, 2.0, 0.0));
    assert!(native_active_rigid_bodies(&world).is_empty());
    let poses = world.active_body_poses();
    assert_eq!(poses.len(), 1);
    assert_eq!(poses[0].id, soft);
    assert_bit_equal_to_body(&world, &poses[0]);
}

/// The sync the guide describes: the awake poses, plus the bodies reported asleep by the step.
#[test]
fn bodies_that_fall_asleep_are_synced_through_their_deactivation_event() {
    let mut world = world();
    add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0), Activation::Activate);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let mut synced = world.body(cube).unwrap().position();
    let mut poses = Vec::new();
    let mut slept = false;
    for _ in 0..600 {
        assert!(world.step(1.0 / 60.0).unwrap().is_complete());
        world.active_body_poses_into(&mut poses);
        if let Some(pose) = poses.iter().find(|pose| pose.id == cube) {
            synced = pose.position;
        }
        for event in world.take_events().activations {
            if let ActivationEvent::Deactivated(id) = event {
                assert!(poses.iter().all(|pose| pose.id != id));
                synced = world.body(id).unwrap().position();
                slept = true;
            }
        }
        if slept {
            break;
        }
    }
    assert!(slept);
    assert_eq!(synced, world.body(cube).unwrap().position());

    // A pose written to a sleeping body leaves it asleep and out of the readout.
    let moved = RVec3::new(1.0, 0.25, 0.0);
    world
        .body_mut(cube)
        .unwrap()
        .set_position(moved, Activation::DontActivate)
        .unwrap();
    assert!(world.active_body_poses().is_empty());
    assert_eq!(world.body(cube).unwrap().position(), moved);
}

#[test]
fn states_into_clears_and_reuses_the_buffer() {
    let mut world = world();
    add_floor(&mut world);
    for x in 0..3 {
        add_cube(
            &mut world,
            RVec3::new(x as Real, 2.0, 0.0),
            Activation::Activate,
        );
    }
    let mut out = world.active_body_states();
    out.extend_from_within(..);
    assert_eq!(out.len(), 6);
    world.active_body_states_into(&mut out);
    assert_eq!(out, world.active_body_states());
    let (capacity, pointer) = (out.capacity(), out.as_ptr());
    world.active_body_states_into(&mut out);
    assert_eq!(out.len(), 3);
    assert_eq!((out.capacity(), out.as_ptr()), (capacity, pointer));

    let empty = self::world();
    empty.active_body_states_into(&mut out);
    assert!(out.is_empty());

    world.active_body_states_into(&mut out);
    assert_eq!(out.len(), 3);
    let mut static_only = self::world();
    add_floor(&mut static_only);
    static_only.active_body_states_into(&mut out);
    assert!(out.is_empty());
}

#[test]
fn detached_bodies_are_absent_until_added() {
    let mut world = world();
    let shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    let settings = BodySettings::new_dynamic().linear_velocity(Vec3::new(1.0, 0.0, 0.0));
    let detached = world.create_body_detached(&shape, &settings).unwrap();
    assert!(world.active_body_states().is_empty());
    assert!(world.active_body_poses().is_empty());
    assert!(world.step(1.0 / 60.0).unwrap().is_complete());
    assert!(world.active_body_states().is_empty());

    world.add_body(detached, Activation::Activate).unwrap();
    let states = world.active_body_states();
    assert_eq!(states.len(), 1);
    assert_eq!(states[0].id, detached);
    assert_state_bit_equal_to_body(&world, &states[0]);
}

#[test]
fn a_soft_body_reads_its_own_velocities() {
    let mut world = world();
    let soft = cloth(&mut world, RVec3::new(0.0, 2.0, 0.0));
    for _ in 0..5 {
        assert!(world.step(1.0 / 60.0).unwrap().is_complete());
    }
    let states = world.active_body_states();
    assert_eq!(states.len(), 1);
    let state = states[0];
    assert_eq!(state.id, soft);
    assert_state_bit_equal_to_body(&world, &state);

    // The linear velocity is the vertices' mean (Jolt averages them in the body's own space and
    // rotates the mean, so it matches up to rounding).
    let vertices = world.soft_body(soft).unwrap().vertices();
    let mean = vertices
        .iter()
        .fold(Vec3::ZERO, |sum, vertex| {
            Vec3::new(
                sum.x + vertex.velocity.x,
                sum.y + vertex.velocity.y,
                sum.z + vertex.velocity.z,
            )
        })
        .scale(1.0 / vertices.len() as f32);
    let difference = Vec3::new(
        state.linear_velocity.x - mean.x,
        state.linear_velocity.y - mean.y,
        state.linear_velocity.z - mean.z,
    );
    assert!(state.linear_velocity.y < -0.5, "{state:?}");
    assert!(
        difference.length() <= 1e-5 * mean.length(),
        "{state:?} {mean:?}"
    );
}
