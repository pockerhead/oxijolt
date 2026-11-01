//! Ragdolls: a 12-capsule humanoid settling on a heightfield in a second world that shares the
//! main world's static shapes, with no contacts between its own parts; joint limits, poses,
//! kinematic and motor drive, masses, lifecycle guards, removal, rebase and settings shared by
//! two worlds.

mod common;

use std::thread;

use common::math::{add, f3, norm, rvec3, sub, v3};
use common::ragdoll::*;
use common::*;
use oxijolt::*;

/// Where the two-box compound stands, x and z.
const COMPOUND_AT: [f32; 2] = [8.0, 8.0];
/// Ticks within which the humanoid must settle.
const SETTLE_TIMEOUT: usize = 600;
/// Height of the pelvis above the terrain when the humanoid is dropped, metres.
const DROP_HEIGHT: f64 = 1.5;

fn two_boxes() -> Shape {
    let low = Shape::new_box(Vec3::new(1.0, 0.5, 1.0)).unwrap();
    let high = Shape::new_box(Vec3::new(0.5, 1.0, 0.5)).unwrap();
    Shape::new_compound(&[
        CompoundChild {
            shape: &low,
            position: Vec3::new(0.0, 0.5, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 0,
        },
        CompoundChild {
            shape: &high,
            position: Vec3::new(1.5, 1.0, 0.0),
            rotation: quat_about(Y, 0.4),
            user_data: 1,
        },
    ])
    .unwrap()
}

/// The static bodies of one world.
#[derive(Clone, Copy, Debug)]
struct Statics {
    terrain: BodyId,
    compound: BodyId,
}

fn add_statics(
    world: &mut PhysicsWorld,
    terrain: &Shape,
    compound: &Shape,
    layer: ObjectLayer,
) -> Statics {
    let mut at = |shape: &Shape, position: RVec3| {
        world
            .create_body(
                shape,
                &BodySettings::new_static()
                    .position(position)
                    .object_layer(layer),
            )
            .unwrap()
    };
    let [x, z] = COMPOUND_AT;
    Statics {
        terrain: at(terrain, RVec3::ZERO),
        compound: at(
            compound,
            rvec3([f64::from(x), f64::from(relief(x, z)) - 0.2, f64::from(z)]),
        ),
    }
}

/// The main world and the ragdoll world, each with its own static bodies of the same shapes.
struct Scene {
    main: PhysicsWorld,
    main_statics: Statics,
    world: PhysicsWorld,
    statics: Statics,
    layers: RagdollLayers,
}

fn scene() -> Scene {
    // Created once and handed to both worlds, then dropped here: the bodies keep their own
    // references.
    let terrain = relief_terrain();
    let compound = two_boxes();
    let mut main = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO)).unwrap();
    let main_statics = add_statics(&mut main, &terrain, &compound, ObjectLayer::NON_MOVING);
    let (mut world, layers) = ragdoll_world(2);
    let statics = add_statics(&mut world, &terrain, &compound, layers.fixed);
    Scene {
        main,
        main_statics,
        world,
        statics,
        layers,
    }
}

/// A ray straight down from 10 m above `(x, z)`.
fn down_ray(x: f64, z: f64) -> RayCast {
    RayCast::new(rvec3([x, 10.0, z]), Vec3::new(0.0, -30.0, 0.0))
}

/// Height of the static surface below `(x, z)` in `world`, seen through `layer`.
fn surface_height(world: &PhysicsWorld, layer: ObjectLayer, x: f64, z: f64) -> f64 {
    let layers = [layer];
    let ray = down_ray(x, z);
    let hit = world
        .cast_ray(&ray, &QueryFilter::new().object_layers(&layers))
        .unwrap()
        .unwrap_or_else(|| panic!("no static surface below ({x}, {z})"));
    v3(ray.point_at(hit.fraction))[1]
}

/// The drop pose: the bind pose turned 30 degrees about Z, the pelvis `DROP_HEIGHT` above the
/// terrain at the origin.
fn drop_pose(world: &PhysicsWorld, layers: &RagdollLayers) -> SkeletonPose {
    let ground = surface_height(world, layers.fixed, 0.0, 0.0);
    transformed_pose(
        &bind_pose(),
        quat_about(Vec3::new(0.0, 0.0, 1.0), 30.0_f32.to_radians()),
        [0.0, ground + DROP_HEIGHT, 0.0],
    )
}

/// Creates the humanoid at the drop pose with its death velocity.
fn drop_humanoid(scene: &mut Scene, settings: &RagdollSettings) -> RagdollId {
    let pose = drop_pose(&scene.world, &scene.layers);
    let ragdoll = scene
        .world
        .create_ragdoll(settings, Some(&pose), Activation::Activate)
        .unwrap();
    scene
        .world
        .ragdoll_mut(ragdoll)
        .unwrap()
        .set_linear_and_angular_velocity(Vec3::new(2.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 3.0))
        .unwrap();
    ragdoll
}

/// What a settling run measured.
#[derive(Debug)]
struct Settled {
    tick: usize,
    worst_violation: f32,
    worst_part: usize,
}

/// Steps the ragdoll world under radial gravity until the humanoid settles, checking every tick
/// that no two of its parts touch, that no part sinks into the terrain, and that it reaches the
/// terrain; every 10th tick the main world steps on another thread at the same time.
fn settle(scene: &mut Scene, ragdoll: RagdollId) -> Settled {
    let Scene {
        main,
        world,
        statics,
        layers,
        ..
    } = scene;
    let mut detector = SettleDetector::default();
    let mut touched_terrain = false;
    let mut worst_violation = 0.0_f32;
    let mut worst_part = 0;
    for tick in 0..SETTLE_TIMEOUT {
        apply_gravity(world, ragdoll, PLANET_CENTRE);
        if tick % 10 == 0 {
            thread::scope(|scope| {
                let main_step = scope.spawn(|| main.step(DT).unwrap());
                assert!(world.step(DT).unwrap().is_complete());
                assert!(main_step.join().unwrap().is_complete());
            });
        } else {
            assert!(world.step(DT).unwrap().is_complete());
        }

        let (violation, part) = worst_limit_violation(world, ragdoll);
        if violation > worst_violation {
            worst_violation = violation;
            worst_part = part;
        }
        let reading = world.ragdoll(ragdoll).unwrap();
        if reading.is_active() {
            let pairs = part_pairs_in_contact(world, ragdoll);
            assert!(pairs.is_empty(), "tick {tick}: parts in contact: {pairs:?}");
        }
        for (index, &part) in reading.body_ids().iter().enumerate() {
            let p = v3(world.body(part).unwrap().position());
            let surface = surface_height(world, layers.fixed, p[0], p[2]);
            assert!(
                p[1] > surface - 0.05,
                "tick {tick}: part {index} at {p:?} is below the surface at {surface}"
            );
            touched_terrain |= world.were_bodies_in_contact(part, statics.terrain).unwrap();
        }
        if detector.update(&reading) {
            assert!(
                touched_terrain,
                "the humanoid settled without touching the terrain"
            );
            return Settled {
                tick,
                worst_violation,
                worst_part,
            };
        }
    }
    panic!("the humanoid did not settle within {SETTLE_TIMEOUT} ticks");
}

/// A scene with the humanoid dropped and settled on the relief.
fn settled_scene() -> (Scene, RagdollId, Settled) {
    let mut scene = scene();
    let settings = humanoid_settings(scene.layers.ragdoll);
    let ragdoll = drop_humanoid(&mut scene, &settings);
    let settled = settle(&mut scene, ragdoll);
    (scene, ragdoll, settled)
}

fn ray_bits(world: &PhysicsWorld, x: f64, z: f64) -> u32 {
    let hit = world
        .cast_ray(&down_ray(x, z), &QueryFilter::new())
        .unwrap()
        .unwrap();
    hit.fraction.to_bits()
}

const PROBES: [[f64; 2]; 5] = [
    [0.0, 0.0],
    [-7.5, 3.25],
    [5.0, -9.0],
    [8.2, 8.1],
    [13.0, 13.0],
];

#[test]
fn humanoid_settles_on_a_heightfield_in_a_second_world() {
    let total: f32 = PARTS.iter().map(|part| part.mass_fraction).sum();
    assert!((total - 1.0).abs() < 1e-6, "{total}");

    let mut scene = scene();
    for [x, z] in PROBES {
        assert_eq!(
            ray_bits(&scene.main, x, z),
            ray_bits(&scene.world, x, z),
            "both worlds see the same statics at ({x}, {z})"
        );
    }
    let main_bodies = scene.main.body_count();
    let main_ray = ray_bits(&scene.main, 1.0, 2.0);

    let settings = humanoid_settings(scene.layers.ragdoll);
    let ragdoll = drop_humanoid(&mut scene, &settings);
    // Measured: settles at tick 141; over eight drop positions between ticks 100 and 210.
    let settled = settle(&mut scene, ragdoll);
    eprintln!("{settled:?}");
    assert!(settled.tick < SETTLE_TIMEOUT);

    // At rest every joint is within its limits. Measured: 0.0037 rad.
    let (violation, part) = worst_limit_violation(&scene.world, ragdoll);
    assert!(
        violation < REST_LIMIT_TOLERANCE,
        "at rest part {part} is {violation} rad outside its joint limits"
    );
    let pose = scene.world.ragdoll(ragdoll).unwrap().pose();
    assert!(pose.joints.iter().all(|joint| {
        <[f32; 3]>::from(joint.translation)
            .iter()
            .all(|value| value.is_finite())
    }));
    let pelvis = v3(pose.root_offset);
    let ground = surface_height(&scene.world, scene.layers.fixed, pelvis[0], pelvis[2]);
    assert!(
        pelvis[1] - ground < 0.5,
        "pelvis {pelvis:?} over ground {ground}"
    );

    assert_eq!(scene.main.body_count(), main_bodies);
    assert_eq!(ray_bits(&scene.main, 1.0, 2.0), main_ray);
    assert!(scene.main.contains_body(scene.main_statics.terrain));
    assert!(scene.main.contains_body(scene.main_statics.compound));
}

// Jolt solves contacts after constraints, so when the humanoid hits the ground the contacts win
// and joints overshoot their limits for a few dozen ticks before they recover. Measured worst
// overshoot after a step of this drop: 0.29 rad at the neck; over other drop positions up to
// 0.34 rad, at the neck, shoulders, hips and elbows. Up to 100 velocity and 50 position solver
// steps, stabilized masses, continuous collision, lower or lying drops, a box floor and other
// friction coefficients all still overshoot by more than 0.08 rad. The bound catches a
// regression of the overshoot, not a hard per-tick limit.
#[test]
fn humanoid_joints_overshoot_their_limits_only_boundedly_during_the_drop() {
    let (_, _, settled) = settled_scene();
    assert!(
        settled.worst_violation < DROP_LIMIT_OVERSHOOT,
        "part {} was {} rad outside its joint limits",
        settled.worst_part,
        settled.worst_violation
    );
}

#[test]
fn limit_check_sees_a_half_turn_swing() {
    use std::f32::consts::PI;
    let cone = Limits::Cone {
        half_angle: 0.6,
        twist: (-0.3, 0.3),
    };
    let pyramid = Limits::Pyramid {
        twist: (-0.3, 0.3),
        swing_y: (-0.3, 1.6),
        swing_z: (-0.2, 0.5),
    };
    let swing_twist = |rotation| JointReading::SwingTwist {
        rotation_in_constraint_space: rotation,
    };
    let six_dof = |rotation| JointReading::SixDof {
        rotation_in_constraint_space: rotation,
    };
    let close = |violation: f32, expected: f32| (violation - expected).abs() < 1e-3;

    // A half turn about Y or Z has no X and W part, the singular case of the decomposition.
    for half_turn in [
        Quat::from_xyzw(0.0, 1.0, 0.0, 0.0),
        Quat::from_xyzw(0.0, 0.0, 1.0, 0.0),
    ] {
        let violation = limit_violation(swing_twist(half_turn), cone);
        assert!(close(violation, PI - 0.6), "{half_turn:?}: {violation}");
    }
    let violation = limit_violation(six_dof(Quat::from_xyzw(0.0, 1.0, 0.0, 0.0)), pyramid);
    assert!(close(violation, PI - 1.6), "{violation}");
    let violation = limit_violation(six_dof(Quat::from_xyzw(0.0, 0.0, 1.0, 0.0)), pyramid);
    assert!(close(violation, PI - 0.5), "{violation}");

    // Next to the singular case.
    let near = quat_about(Y, PI - 1e-3);
    let violation = limit_violation(swing_twist(near), cone);
    assert!(close(violation, PI - 1e-3 - 0.6), "{violation}");

    // Within the limits.
    assert_eq!(limit_violation(swing_twist(quat_about(Y, 0.5)), cone), 0.0);
    assert_eq!(limit_violation(six_dof(quat_about(Y, 1.5)), pyramid), 0.0);
}

#[test]
fn the_same_parts_as_plain_bodies_collide() {
    let (mut world, layers) = ragdoll_world(1);
    let shapes = part_shapes();
    let bodies: Vec<BodyId> = (0..PART_COUNT)
        .map(|part| {
            world
                .create_body(&shapes[part], &part_body(part, layers.ragdoll))
                .unwrap()
        })
        .collect();
    assert!(world.step(DT).unwrap().is_complete());
    let touching = PARTS.iter().enumerate().any(|(part, spec)| {
        spec.parent.is_some_and(|parent| {
            world
                .were_bodies_in_contact(bodies[part], bodies[parent as usize])
                .unwrap()
        })
    });
    assert!(
        touching,
        "adjacent capsules overlap, so plain bodies collide"
    );
}

#[test]
fn two_ragdolls_collide_with_each_other_but_not_with_themselves() {
    let (mut world, layers) = ragdoll_world(1);
    let settings = humanoid_settings(layers.ragdoll);
    let first = world
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    let shifted = transformed_pose(&bind_pose(), Quat::IDENTITY, [0.05, 0.0, 0.15]);
    let second = world
        .create_ragdoll(&settings, Some(&shifted), Activation::Activate)
        .unwrap();
    assert!(world.step(DT).unwrap().is_complete());
    let a = world.ragdoll(first).unwrap().body_ids().to_vec();
    let b = world.ragdoll(second).unwrap().body_ids().to_vec();
    let crossing = a.iter().any(|&p| {
        b.iter()
            .any(|&q| world.were_bodies_in_contact(p, q).unwrap())
    });
    assert!(crossing, "the overlapping ragdolls touch");
    assert!(part_pairs_in_contact(&world, first).is_empty());
    assert!(part_pairs_in_contact(&world, second).is_empty());
}

#[test]
fn an_environment_ray_passes_through_a_resting_part() {
    let (scene, ragdoll, _) = settled_scene();
    let pelvis = scene.world.ragdoll(ragdoll).unwrap().body_ids()[PELVIS];
    let p = v3(scene.world.body(pelvis).unwrap().position());
    let ray = down_ray(p[0], p[2]);
    let fixed = [scene.layers.fixed];
    let environment = scene
        .world
        .cast_ray(&ray, &QueryFilter::new().object_layers(&fixed))
        .unwrap()
        .unwrap();
    assert_eq!(environment.body, scene.statics.terrain);
    let anything = scene
        .world
        .cast_ray(&ray, &QueryFilter::new())
        .unwrap()
        .unwrap();
    assert_eq!(scene.world.ragdoll_of_body(anything.body), Some(ragdoll));
}

fn assert_poses_match(a: &SkeletonPose, b: &SkeletonPose, tolerance: f32) {
    for index in 0..PART_COUNT {
        let (distance, angle) = transform_difference(a, b, index);
        assert!(
            distance < f64::from(tolerance) && angle < tolerance,
            "joint {index}: {distance} m, {angle} rad"
        );
    }
}

#[test]
fn poses_read_back_in_world_transforms() {
    let (mut world, layers) = ragdoll_world(1);
    let settings = humanoid_settings(layers.ragdoll);
    let fresh = world
        .create_ragdoll(&settings, None, Activation::DontActivate)
        .unwrap();
    assert_poses_match(&world.ragdoll(fresh).unwrap().pose(), &bind_pose(), 1e-5);

    let target = transformed_pose(
        &bind_pose(),
        quat_about(Vec3::new(0.0, 0.6, 0.8), 0.7),
        [3.0, 2.0, -1.0],
    );
    world.ragdoll_mut(fresh).unwrap().set_pose(&target).unwrap();
    let read = world.ragdoll(fresh).unwrap().pose();
    assert_poses_match(&read, &target, 1e-5);
    // The root offset is the pelvis, so the pelvis translation reads back as zero.
    assert_eq!(read.joints[PELVIS].translation, Vec3::ZERO);

    let placed = world
        .create_ragdoll(&settings, Some(&target), Activation::DontActivate)
        .unwrap();
    assert_poses_match(&world.ragdoll(placed).unwrap().pose(), &target, 1e-5);
    let (position, rotation) = world.ragdoll(placed).unwrap().root_transform();
    assert!(norm(sub(v3(position), joint_position(&target, PELVIS))) < 1e-5);
    assert!(angle_between(rotation, target.joints[PELVIS].rotation) < 1e-5);
}

#[test]
fn kinematic_parts_reach_the_driven_pose() {
    let (mut world, layers) = ragdoll_world(1);
    let settings = humanoid_settings(layers.ragdoll);
    let ragdoll = world
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    let mut handle = world.ragdoll_mut(ragdoll).unwrap();
    handle
        .set_motion_type(MotionType::Kinematic, Activation::Activate)
        .unwrap();
    let target = transformed_pose(&bind_pose(), quat_about(Y, 0.1), [0.2, 0.1, -0.1]);
    assert!(matches!(
        handle.drive_to_pose_using_kinematics(&target, PhysicsWorld::MIN_DELTA_TIME / 2.0),
        Err(RagdollError::InvalidValue(_))
    ));
    assert!(world.step(DT).unwrap().is_complete());
    assert_poses_match(&world.ragdoll(ragdoll).unwrap().pose(), &bind_pose(), 1e-6);

    world
        .ragdoll_mut(ragdoll)
        .unwrap()
        .drive_to_pose_using_kinematics(&target, DT)
        .unwrap();
    assert!(world.step(DT).unwrap().is_complete());
    assert_poses_match(&world.ragdoll(ragdoll).unwrap().pose(), &target, 1e-4);
    assert!(matches!(
        world
            .ragdoll_mut(ragdoll)
            .unwrap()
            .set_motion_type(MotionType::Static, Activation::Activate),
        Err(RagdollError::InvalidValue(_))
    ));
}

/// The rotation of `part` relative to its parent part, in the parent's frame.
fn relative_rotation(world: &PhysicsWorld, ragdoll: RagdollId, part: usize) -> Quat {
    let ids = world.ragdoll(ragdoll).unwrap().body_ids().to_vec();
    let parent = PARTS[part].parent.unwrap() as usize;
    let rotation = |id| world.body(id).unwrap().rotation();
    mul(conj(rotation(ids[parent])), rotation(ids[part]))
}

/// One motor case: the part, the world axis it turns about from the bind pose, the angle, and
/// the hinge reading it should reach.
struct MotorCase {
    part: usize,
    axis: Vec3,
    angle: f32,
    hinge_angle: Option<f32>,
}

#[test]
fn motors_drive_each_joint_kind_to_its_target() {
    let cases = [
        // Elbow, a hinge about -Y: flexes the left forearm forward.
        MotorCase {
            part: FOREARM_L,
            axis: neg(Y),
            angle: 1.2,
            hinge_angle: Some(1.2),
        },
        // Knee, a hinge about -X: flexes the right shin backward.
        MotorCase {
            part: SHIN_R,
            axis: neg(X),
            angle: -1.0,
            hinge_angle: Some(-1.0),
        },
        // Hip, six-DOF: flexes the right thigh forward.
        MotorCase {
            part: THIGH_R,
            axis: neg(X),
            angle: 0.8,
            hinge_angle: None,
        },
        // Neck, swing-twist: tilts the head sideways.
        MotorCase {
            part: HEAD,
            axis: Vec3::new(0.0, 0.0, 1.0),
            angle: 0.4,
            hinge_angle: None,
        },
    ];
    for case in cases {
        let (mut world, layers) = ragdoll_world(1);
        let settings = humanoid_settings(layers.ragdoll);
        let ragdoll = world
            .create_ragdoll(&settings, None, Activation::Activate)
            .unwrap();
        let part = case.part as u32;
        assert_eq!(
            world.ragdoll(ragdoll).unwrap().joint_motors_on(part),
            Some(false)
        );
        assert_eq!(world.ragdoll(ragdoll).unwrap().joint_motors_on(0), None);
        let mut target = bind_pose();
        let turn = quat_about(case.axis, case.angle);
        target.joints[case.part].rotation = mul(turn, target.joints[case.part].rotation);
        let parent = PARTS[case.part].parent.unwrap() as usize;
        let wanted = mul(
            conj(target.joints[parent].rotation),
            target.joints[case.part].rotation,
        );

        // Driven every tick, as a game does; each drive wakes the ragdoll.
        for _ in 0..180 {
            world
                .ragdoll_mut(ragdoll)
                .unwrap()
                .drive_to_pose_using_motors(&target)
                .unwrap();
            step(&mut world, 1);
        }
        let reached = angle_between(relative_rotation(&world, ragdoll, case.part), wanted);
        assert!(
            reached < 0.05,
            "part {}: {reached} rad from the target",
            case.part
        );
        let reading = world.ragdoll(ragdoll).unwrap().joint(case.part as u32);
        if let Some(angle) = case.hinge_angle {
            let Some(JointReading::Hinge { current_angle }) = reading else {
                panic!("part {} has a hinge, not {reading:?}", case.part);
            };
            assert!((current_angle - angle).abs() < 0.05, "{current_angle}");
        }

        // Without motors, a kick moves the joint away from the target.
        assert_eq!(
            world.ragdoll(ragdoll).unwrap().joint_motors_on(part),
            Some(true)
        );
        world.ragdoll_mut(ragdoll).unwrap().stop_motors();
        assert_eq!(
            world.ragdoll(ragdoll).unwrap().joint_motors_on(part),
            Some(false)
        );
        let ids = world.ragdoll(ragdoll).unwrap().body_ids().to_vec();
        // The joint's world axis now: the bind axis moved with the parent.
        let parent_turn = mul(
            world.body(ids[parent]).unwrap().rotation(),
            conj(target.joints[parent].rotation),
        );
        let kick = rotate(parent_turn, case.axis);
        let speed = case.angle.signum() * 15.0;
        world
            .body_mut(ids[case.part])
            .unwrap()
            .set_angular_velocity(Vec3::new(kick.x * speed, kick.y * speed, kick.z * speed))
            .unwrap();
        step(&mut world, 120);
        let left = angle_between(relative_rotation(&world, ragdoll, case.part), wanted);
        assert!(
            left > 0.1,
            "part {}: only {left} rad from the target",
            case.part
        );
    }
}

/// Motor targets are parent-relative rotations. A ragdoll created at the bind pose has parents
/// at identity rotation, where a world-frame target looks the same; this one is turned 0.9 rad
/// about a skewed axis first, so a target taken in the wrong frame misses by about that angle.
#[test]
fn motors_drive_joints_of_a_turned_ragdoll_in_the_parent_frame() {
    let axis = Vec3::new(0.48, 0.6, 0.64);
    let turned = transformed_pose(&bind_pose(), quat_about(axis, 0.9), [0.0; 3]);
    // Each driven part, the axis it turns about given in world space at the bind pose, and the
    // angle: the right shoulder (swing-twist) raises the arm, the left hip (six-DOF) flexes the
    // thigh forward and the left elbow (hinge) flexes the forearm.
    let cases = [
        (UPPER_ARM_R, Vec3::new(0.0, 0.0, 1.0), 0.6),
        (THIGH_L, neg(X), 0.8),
        (FOREARM_L, neg(Y), 1.2),
    ];
    let mut target = turned.clone();
    let mut wanted = Vec::new();
    for (part, bind_axis, angle) in cases {
        let parent = PARTS[part].parent.unwrap() as usize;
        let relative = mul(
            conj(turned.joints[parent].rotation),
            turned.joints[part].rotation,
        );
        // The bind-pose axis in the parent's frame, which the turn does not change.
        let parent_axis = rotate(conj(bind_rotation(&PARTS[parent])), bind_axis);
        let goal = mul(quat_about(parent_axis, angle), relative);
        target.joints[part].rotation = mul(target.joints[parent].rotation, goal);
        wanted.push((part, goal));
    }

    let (mut world, layers) = ragdoll_world(1);
    let settings = humanoid_settings(layers.ragdoll);
    let ragdoll = world
        .create_ragdoll(&settings, Some(&turned), Activation::Activate)
        .unwrap();
    // Driven every tick, as a game does; each drive wakes the ragdoll.
    for _ in 0..240 {
        world
            .ragdoll_mut(ragdoll)
            .unwrap()
            .drive_to_pose_using_motors(&target)
            .unwrap();
        step(&mut world, 1);
    }
    for (part, goal) in wanted {
        let reached = angle_between(relative_rotation(&world, ragdoll, part), goal);
        assert!(reached < 0.1, "part {part}: {reached} rad from the target");
    }
}

#[test]
fn plain_settings_keep_the_masses_and_stabilized_ones_the_total() {
    let (mut world, layers) = ragdoll_world(1);
    let masses = |world: &PhysicsWorld, ragdoll: RagdollId| -> Vec<f32> {
        world
            .ragdoll(ragdoll)
            .unwrap()
            .body_ids()
            .iter()
            .map(|&id| world.body(id).unwrap().mass().unwrap())
            .collect()
    };
    let plain = world
        .create_ragdoll(
            &humanoid_settings(layers.ragdoll),
            None,
            Activation::DontActivate,
        )
        .unwrap();
    for (part, mass) in masses(&world, plain).into_iter().enumerate() {
        let expected = MASS * PARTS[part].mass_fraction;
        assert!(
            (mass - expected).abs() <= expected * 1e-5,
            "part {part}: {mass} kg, expected {expected}"
        );
    }
    let stabilized = world
        .create_ragdoll(
            &stabilized_humanoid_settings(layers.ragdoll),
            Some(&transformed_pose(
                &bind_pose(),
                Quat::IDENTITY,
                [5.0, 0.0, 0.0],
            )),
            Activation::DontActivate,
        )
        .unwrap();
    let stabilized = masses(&world, stabilized);
    let total: f32 = stabilized.iter().sum();
    assert!((total - MASS).abs() <= MASS * 1e-3, "{total}");
    let head = MASS * PARTS[HEAD].mass_fraction;
    assert!(
        (stabilized[HEAD] - head).abs() > head * 1e-3,
        "Stabilize changes the head mass: {}",
        stabilized[HEAD]
    );
}

#[test]
fn ragdoll_parts_and_ids_are_guarded() {
    let (mut world, layers) = ragdoll_world(1);
    let settings = humanoid_settings(layers.ragdoll);
    let ragdoll = world
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    let part = world.ragdoll(ragdoll).unwrap().body_ids()[CHEST];
    assert_eq!(world.ragdoll_of_body(part), Some(ragdoll));
    assert_eq!(world.ragdoll_count(), 1);
    assert!(world.contains_ragdoll(ragdoll));
    assert_eq!(
        world.remove_body(part),
        Err(BodyError::OwnedByRagdoll(part))
    );
    let vehicle = WheeledVehicleSettings::new(
        vec![WheelSettings::new(Vec3::new(0.0, -0.2, 0.0))],
        vec![VehicleDifferentialSettings::new(Some(0), None)],
        VehicleCollisionTester::ray(layers.fixed),
    );
    assert!(matches!(
        world.create_wheeled_vehicle(part, &vehicle),
        Err(VehicleError::Body(BodyError::OwnedByRagdoll(id))) if id == part
    ));

    let (mut other, _) = ragdoll_world(1);
    let foreign = other
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    assert_eq!(
        world.ragdoll(foreign).err(),
        Some(RagdollError::WrongWorld(foreign))
    );
    assert_eq!(
        world.remove_ragdoll(foreign),
        Err(RagdollError::WrongWorld(foreign))
    );
    assert!(!world.contains_ragdoll(foreign));
    world.remove_ragdoll(ragdoll).unwrap();
    assert_eq!(world.ragdoll_count(), 0);
    assert!(!world.contains_ragdoll(ragdoll));
    assert_eq!(
        world.ragdoll(ragdoll).err(),
        Some(RagdollError::NotFound(ragdoll))
    );
    assert!(matches!(
        world.ragdoll_mut(ragdoll),
        Err(RagdollError::NotFound(_))
    ));
    assert_eq!(world.ragdoll_of_body(part), None);

    let (layers_table, small_layers) = ragdoll_layers();
    let mut small = PhysicsWorld::new(
        WorldSettings::default()
            .gravity(Vec3::ZERO)
            .max_bodies(20)
            .layers(layers_table),
    )
    .unwrap();
    let small_settings = humanoid_settings(small_layers.ragdoll);
    small
        .create_ragdoll(&small_settings, None, Activation::Activate)
        .unwrap();
    assert_eq!(
        small.create_ragdoll(&small_settings, None, Activation::Activate),
        Err(RagdollError::TooManyBodies)
    );
    assert_eq!(small.body_count(), 12);

    let unknown = humanoid_settings(ObjectLayer::new(5));
    assert_eq!(
        world.create_ragdoll(&unknown, None, Activation::Activate),
        Err(RagdollError::UnknownObjectLayer(ObjectLayer::new(5)))
    );
    let mut bad = bind_pose();
    bad.joints[HEAD].rotation = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);
    assert!(matches!(
        world.create_ragdoll(&settings, Some(&bad), Activation::Activate),
        Err(RagdollError::InvalidValue(_))
    ));
    bad.joints.pop();
    assert!(matches!(
        world.create_ragdoll(&settings, Some(&bad), Activation::Activate),
        Err(RagdollError::InvalidValue(_))
    ));
    assert_eq!(world.body_count(), 0);
    assert_eq!(world.ragdoll_ids().count(), 0);
}

#[test]
fn removing_a_ragdoll_drops_what_rests_on_it() {
    let (mut scene, ragdoll, _) = settled_scene();
    let chest = scene.world.ragdoll(ragdoll).unwrap().body_ids()[CHEST];
    let top = add(
        v3(scene.world.body(chest).unwrap().position()),
        [0.0, 0.35, 0.0],
    );
    let cube_shape = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    let cube = scene
        .world
        .create_body(
            &cube_shape,
            &BodySettings::new_dynamic()
                .object_layer(scene.layers.ragdoll)
                .position(rvec3(top))
                .mass(1.0)
                .gravity_factor(0.0),
        )
        .unwrap();
    for _ in 0..90 {
        apply_gravity(&mut scene.world, ragdoll, PLANET_CENTRE);
        apply_gravity_to(&mut scene.world, cube, PLANET_CENTRE);
        assert!(scene.world.step(DT).unwrap().is_complete());
    }
    let resting = v3(scene.world.body(cube).unwrap().position());
    let ground = surface_height(&scene.world, scene.layers.fixed, resting[0], resting[2]);
    assert!(resting[1] - ground > 0.15, "the cube rests on the ragdoll");

    let bodies = scene.world.body_count();
    scene.world.remove_ragdoll(ragdoll).unwrap();
    assert_eq!(scene.world.body_count(), bodies - 12);
    assert!(scene.world.body(cube).unwrap().is_active());
    for _ in 0..60 {
        apply_gravity_to(&mut scene.world, cube, PLANET_CENTRE);
        assert!(scene.world.step(DT).unwrap().is_complete());
    }
    let fallen = v3(scene.world.body(cube).unwrap().position());
    assert!(fallen[1] < resting[1] - 0.05, "{resting:?} -> {fallen:?}");
}

/// A sleeper resting within the speculative contact distance of a removed part is in Jolt's
/// contact range without touching the part's bounds; it wakes and falls.
#[test]
fn removing_a_ragdoll_wakes_a_sleeper_within_speculative_distance() {
    let (layers, ids) = ragdoll_layers();
    let mut world = PhysicsWorld::new(jobs::with_threads(
        WorldSettings::default()
            .gravity(Vec3::new(0.0, -9.81, 0.0))
            .layers(layers),
        1,
    ))
    .unwrap();
    let settings = humanoid_settings(ids.ragdoll);
    let ragdoll = world
        .create_ragdoll(&settings, Some(&bind_pose()), Activation::DontActivate)
        .unwrap();
    let head = &PARTS[HEAD];
    assert!(!head.along_x);
    let head_top = head.centre[1] + head.half_height + head.radius;
    let gap = 0.5 * world.speculative_contact_distance();
    let half = 0.1;
    let cube_shape = Shape::new_box(Vec3::new(half, half, half)).unwrap();
    let cube = world
        .create_body(
            &cube_shape,
            &BodySettings::new_dynamic()
                .object_layer(ids.ragdoll)
                .position(RVec3::new(
                    Real::from(head.centre[0]),
                    Real::from(head_top + gap + half),
                    Real::from(head.centre[2]),
                ))
                .activation(Activation::DontActivate),
        )
        .unwrap();
    assert!(world.body(cube).unwrap().is_sleeping());
    let start = world.body(cube).unwrap().position().y;

    world.remove_ragdoll(ragdoll).unwrap();
    assert!(world.body(cube).unwrap().is_active());
    step(&mut world, 30);
    let y = world.body(cube).unwrap().position().y;
    assert!(y < start - 0.5, "{start} -> {y}");
}

#[test]
fn dropping_a_world_with_ragdolls_is_clean() {
    let (mut world, layers) = ragdoll_world(1);
    let settings = humanoid_settings(layers.ragdoll);
    for i in 0..3 {
        let pose = transformed_pose(&bind_pose(), Quat::IDENTITY, [2.0 * f64::from(i), 0.0, 0.0]);
        world
            .create_ragdoll(&settings, Some(&pose), Activation::Activate)
            .unwrap();
    }
    step(&mut world, 5);
    assert_eq!(world.ragdoll_ids().count(), 3);
    drop(world);
}

/// Whether two readings of the same joint agree within `tolerance` radians.
fn readings_agree(a: JointReading, b: JointReading, tolerance: f32) -> bool {
    match (a, b) {
        (JointReading::Hinge { current_angle: a }, JointReading::Hinge { current_angle: b }) => {
            (a - b).abs() < tolerance
        }
        (
            JointReading::SwingTwist {
                rotation_in_constraint_space: a,
            },
            JointReading::SwingTwist {
                rotation_in_constraint_space: b,
            },
        )
        | (
            JointReading::SixDof {
                rotation_in_constraint_space: a,
            },
            JointReading::SixDof {
                rotation_in_constraint_space: b,
            },
        ) => angle_between(a, b) < tolerance,
        _ => false,
    }
}

#[test]
fn rebase_moves_a_settled_ragdoll_rigidly() {
    let (mut scene, ragdoll, _) = settled_scene();
    let reading = |world: &PhysicsWorld, part: usize| {
        world.ragdoll(ragdoll).unwrap().joint(part as u32).unwrap()
    };
    let before: Vec<JointReading> = (1..PART_COUNT)
        .map(|part| reading(&scene.world, part))
        .collect();
    let mut bodies = vec![scene.statics.terrain, scene.statics.compound];
    bodies.extend_from_slice(scene.world.ragdoll(ragdoll).unwrap().body_ids());
    let rotation = quat_about(Vec3::new(0.0, 0.0, 1.0), 0.35);
    let translation = [4.0, -2.0, 1.5];
    scene
        .world
        .rebase(&bodies, rotation, rvec3(translation))
        .unwrap();
    for (part, before) in (1..PART_COUNT).zip(before) {
        let after = reading(&scene.world, part);
        assert!(
            readings_agree(before, after, 1e-4),
            "part {part}: {before:?} -> {after:?}"
        );
    }
    let centre = add(common::math::rotate(rotation, PLANET_CENTRE), translation);
    // The rotated frame turns Jolt's friction tangents a little, which may nudge a part past
    // the calm limit for a tick or two; the ragdoll settles again at once and stays in place.
    let pelvis = scene.world.ragdoll(ragdoll).unwrap().body_ids()[PELVIS];
    let start = v3(scene.world.body(pelvis).unwrap().position());
    let mut detector = SettleDetector::default();
    let settled_again = (0..90).any(|_| {
        apply_gravity(&mut scene.world, ragdoll, centre);
        assert!(scene.world.step(DT).unwrap().is_complete());
        detector.update(&scene.world.ragdoll(ragdoll).unwrap())
    });
    assert!(settled_again, "the rebased ragdoll comes back to rest");
    let moved = norm(sub(v3(scene.world.body(pelvis).unwrap().position()), start));
    assert!(moved < 0.02, "the rebased pelvis moved {moved} m");
}

#[test]
fn one_settings_value_serves_two_worlds() {
    let (mut a, layers) = ragdoll_world(1);
    let (mut b, _) = ragdoll_world(3);
    let settings = humanoid_settings(layers.ragdoll);
    let in_a = a
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    let in_b = b
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    drop(settings);
    thread::scope(|scope| {
        scope.spawn(|| step(&mut a, 30));
        scope.spawn(|| step(&mut b, 30));
    });
    assert_eq!(a.ragdoll(in_a).unwrap().part_count(), 12);
    assert_eq!(b.ragdoll(in_b).unwrap().part_count(), 12);
    a.remove_ragdoll(in_a).unwrap();
    b.remove_ragdoll(in_b).unwrap();
    assert_eq!(a.body_count() + b.body_count(), 0);
}

/// Mass of each sphere of [`sphere_pair`], kg.
const PAIR_MASS: f32 = 2.0;
/// Radius of each sphere of [`sphere_pair`], metres.
const PAIR_RADIUS: f32 = 0.5;

/// Two dynamic spheres one above the other, joined by a free swing-twist joint whose motors
/// have `spring`.
fn sphere_pair(spring: SpringSettings, stabilized: bool) -> Result<RagdollSettings, RagdollError> {
    let (_, layers) = ragdoll_layers();
    let skeleton = Skeleton::new(&[
        SkeletonJoint {
            name: "lower",
            parent: None,
        },
        SkeletonJoint {
            name: "upper",
            parent: Some(0),
        },
    ])
    .unwrap();
    let sphere = Shape::new_sphere(PAIR_RADIUS).unwrap();
    let body = |y: Real| {
        BodySettings::new_dynamic()
            .object_layer(layers.ragdoll)
            .position(RVec3::new(0.0, y, 0.0))
            .mass(PAIR_MASS)
    };
    let motor = MotorSettings::default().spring(spring);
    let joint = SwingTwistConstraintSettings::new(
        RVec3::new(0.0, 0.5, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .half_cone_angles(1.0, 1.0)
    .twist_limits(-1.0, 1.0)
    .swing_motor(motor)
    .twist_motor(motor);
    let parts = [
        RagdollPart {
            shape: &sphere,
            body: body(0.0),
            joint: None,
        },
        RagdollPart {
            shape: &sphere,
            body: body(1.0),
            joint: Some(RagdollJoint::SwingTwist(joint)),
        },
    ];
    if stabilized {
        RagdollSettings::new_stabilized(&skeleton, &parts)
    } else {
        RagdollSettings::new(&skeleton, &parts)
    }
}

/// The effective-mass bound the settings use for [`sphere_pair`]: a part's mass and the trace of
/// its inertia, `3 * 0.4 * m * r²`; after `Stabilize`, the total mass and twice the total mass
/// times the largest trace per mass.
fn sphere_pair_mass_bound(stabilized: bool) -> f64 {
    let (m, r) = (f64::from(PAIR_MASS), f64::from(PAIR_RADIUS));
    let trace = 1.2 * m * r * r;
    if stabilized {
        let total = 2.0 * m;
        total.max(2.0 * total * trace / m)
    } else {
        m.max(trace)
    }
}

fn frequency_spring(frequency: f64, damping: f64) -> SpringSettings {
    SpringSettings::FrequencyAndDamping {
        frequency: frequency as f32,
        damping: damping as f32,
    }
}

#[test]
fn motor_springs_are_bounded_by_the_parts_effective_mass() {
    let coefficient = f64::from(limits::MAX_SPRING_COEFFICIENT);
    let two_pi = 2.0 * std::f64::consts::PI;
    for stabilized in [false, true] {
        let bound = sphere_pair_mass_bound(stabilized);
        // `B * ω² <= MAX`: the largest frequency with damping 0.
        let max_frequency = (coefficient / bound).sqrt() / two_pi;
        // `2 * B * ζ * ω <= MAX`: the largest damping ratio at 1 Hz.
        let max_damping = coefficient / (2.0 * bound * two_pi);
        let accepted = [
            frequency_spring(0.999 * max_frequency, 0.0),
            frequency_spring(1.0, 0.999 * max_damping),
        ];
        for spring in accepted {
            assert!(sphere_pair(spring, stabilized).is_ok(), "{spring:?}");
        }
        let rejected = [
            frequency_spring(1.001 * max_frequency, 0.0),
            frequency_spring(1.0, 1.001 * max_damping),
        ];
        for spring in rejected {
            assert!(
                matches!(
                    sphere_pair(spring, stabilized),
                    Err(RagdollError::InvalidValue(_))
                ),
                "{spring:?}"
            );
        }
    }
}

#[test]
fn stiffness_springs_are_bounded_by_the_coefficient() {
    let bound = limits::MAX_SPRING_COEFFICIENT;
    let spring = |stiffness, damping| SpringSettings::StiffnessAndDamping { stiffness, damping };
    assert!(sphere_pair(spring(bound, bound), false).is_ok());
    for rejected in [spring(bound.next_up(), 0.0), spring(0.0, bound.next_up())] {
        assert!(matches!(
            sphere_pair(rejected, false),
            Err(RagdollError::InvalidValue(_))
        ));
    }
}

#[test]
fn motor_spring_of_1e20_hz_is_rejected() {
    // Before the bound, a 1e20 Hz motor overflowed Jolt's spring stiffness during the step and
    // tripped the finite-velocity assertion in `MotionProperties::ClampAngularVelocity`.
    let absurd = frequency_spring(1.0e20, 1.0);
    for stabilized in [false, true] {
        assert!(matches!(
            sphere_pair(absurd, stabilized),
            Err(RagdollError::InvalidValue(_))
        ));
    }
}

#[test]
fn motor_springs_at_the_coefficient_bound_drive_finitely() {
    let bound = sphere_pair_mass_bound(false);
    let max_frequency =
        (f64::from(limits::MAX_SPRING_COEFFICIENT) / bound).sqrt() / (2.0 * std::f64::consts::PI);
    let springs = [
        frequency_spring(0.999 * max_frequency, 0.0),
        SpringSettings::StiffnessAndDamping {
            stiffness: limits::MAX_SPRING_COEFFICIENT,
            damping: limits::MAX_SPRING_COEFFICIENT,
        },
    ];
    for spring in springs {
        let settings = sphere_pair(spring, false).unwrap();
        let (mut world, _) = ragdoll_world(1);
        let ragdoll = world
            .create_ragdoll(&settings, None, Activation::Activate)
            .unwrap();
        let mut target = world.ragdoll(ragdoll).unwrap().pose();
        target.joints[1].rotation = quat_about(Vec3::new(1.0, 0.0, 0.0), 0.5);
        for delta_time in [PhysicsWorld::MAX_DELTA_TIME, PhysicsWorld::MIN_DELTA_TIME] {
            for _ in 0..10 {
                world
                    .ragdoll_mut(ragdoll)
                    .unwrap()
                    .drive_to_pose_using_motors(&target)
                    .unwrap();
                assert!(world.step(delta_time).unwrap().is_complete());
                for &id in world.ragdoll(ragdoll).unwrap().body_ids() {
                    let body = world.body(id).unwrap();
                    let state = [
                        f3(body.linear_velocity()),
                        f3(body.angular_velocity()),
                        v3(body.position()),
                    ];
                    assert!(
                        state.iter().flatten().all(|value| value.is_finite()),
                        "{spring:?} at {delta_time}: {state:?}"
                    );
                }
            }
        }
    }
}

/// Every part's linear and angular velocity as bits, in part order.
fn part_velocity_bits(world: &PhysicsWorld, ragdoll: RagdollId) -> Vec<[u32; 6]> {
    let ragdoll = world.ragdoll(ragdoll).unwrap();
    ragdoll
        .body_ids()
        .iter()
        .map(|&id| {
            let body = world.body(id).unwrap();
            let [a, b, c] = <[f32; 3]>::from(body.linear_velocity()).map(f32::to_bits);
            let [d, e, f] = <[f32; 3]>::from(body.angular_velocity()).map(f32::to_bits);
            [a, b, c, d, e, f]
        })
        .collect()
}

#[test]
fn kinematic_drive_is_bounded_by_the_velocities_it_implies() {
    let settings = sphere_pair(frequency_spring(2.0, 1.0), false).unwrap();
    let (mut world, _) = ragdoll_world(1);
    let ragdoll = world
        .create_ragdoll(&settings, None, Activation::DontActivate)
        .unwrap();
    world
        .ragdoll_mut(ragdoll)
        .unwrap()
        .set_motion_type(MotionType::Kinematic, Activation::DontActivate)
        .unwrap();
    let start = world.ragdoll(ragdoll).unwrap().pose();
    // The spheres' centres of mass are their body origins, at whole metres, so every distance
    // below is exact and the velocity Jolt writes is the move divided by the time step.
    assert_eq!(start.root_offset, RVec3::new(0.0, 0.0, 0.0));
    let initial = part_velocity_bits(&world, ragdoll);

    // Translation: a power-of-two step makes `MAX_LINEAR_VELOCITY * dt` exact.
    let dt = 1.0 / 64.0;
    let at_bound = limits::MAX_LINEAR_VELOCITY * dt;
    let moved = |moves: [f32; 2]| {
        let mut pose = start.clone();
        for (joint, x) in pose.joints.iter_mut().zip(moves) {
            joint.translation.x += x;
        }
        pose
    };
    // Rotation of the upper part about y in the small-angle branch of Jolt's
    // `Quat::GetAngularVelocity`, where the angular velocity is `(2 / dt) * xyz` and exact for a
    // power-of-two step.
    let dt_turn = 1.0 / 2048.0;
    let turned = |sin_half_angle: f32| {
        let mut pose = start.clone();
        let w = (1.0 - sin_half_angle * sin_half_angle).sqrt();
        pose.joints[1].rotation = Quat::from_xyzw(0.0, sin_half_angle, 0.0, w);
        pose
    };
    let sin_at_bound = limits::MAX_ANGULAR_VELOCITY * dt_turn / 2.0;
    let rejected = [
        // The upper part one ulp too fast; the lower part's valid move must not happen either.
        (moved([at_bound, at_bound.next_up()]), dt),
        (moved([0.0, 1.0]), PhysicsWorld::MIN_DELTA_TIME),
        (turned(sin_at_bound.next_up()), dt_turn),
        (turned(0.1), PhysicsWorld::MIN_DELTA_TIME),
    ];
    for (pose, delta_time) in &rejected {
        let result = world
            .ragdoll_mut(ragdoll)
            .unwrap()
            .drive_to_pose_using_kinematics(pose, *delta_time);
        assert!(
            matches!(result, Err(RagdollError::InvalidValue(_))),
            "{delta_time}: {result:?}"
        );
        assert_eq!(part_velocity_bits(&world, ragdoll), initial, "{delta_time}");
    }

    let mut handle = world.ragdoll_mut(ragdoll).unwrap();
    handle
        .drive_to_pose_using_kinematics(&moved([at_bound, at_bound]), dt)
        .unwrap();
    for &id in world.ragdoll(ragdoll).unwrap().body_ids() {
        let velocity = world.body(id).unwrap().linear_velocity();
        assert_eq!(velocity, Vec3::new(limits::MAX_LINEAR_VELOCITY, 0.0, 0.0));
    }
    let mut handle = world.ragdoll_mut(ragdoll).unwrap();
    handle
        .drive_to_pose_using_kinematics(&turned(sin_at_bound), dt_turn)
        .unwrap();
    let upper = world.ragdoll(ragdoll).unwrap().body_ids()[1];
    let spin = world.body(upper).unwrap().angular_velocity();
    assert_eq!(spin, Vec3::new(0.0, limits::MAX_ANGULAR_VELOCITY, 0.0));
}

#[test]
fn part_masses_and_velocities_are_bounded() {
    let (_, layers) = ragdoll_layers();
    let shapes = part_shapes();
    for (mass, ok) in [
        (limits::MIN_MASS, true),
        (limits::MAX_MASS, true),
        (limits::MIN_MASS.next_down(), false),
        (limits::MAX_MASS.next_up(), false),
    ] {
        let mut parts = humanoid_parts(&shapes, layers.ragdoll);
        parts[HEAD].body = parts[HEAD].body.clone().mass(mass);
        let result = RagdollSettings::new(&skeleton(), &parts);
        assert_eq!(result.is_ok(), ok, "mass {mass}");
    }

    let (mut world, layers) = ragdoll_world(1);
    let ragdoll = world
        .create_ragdoll(
            &humanoid_settings(layers.ragdoll),
            None,
            Activation::Activate,
        )
        .unwrap();
    let linear = Vec3::new(0.0, limits::MAX_LINEAR_VELOCITY, 0.0);
    let angular = Vec3::new(limits::MAX_ANGULAR_VELOCITY, 0.0, 0.0);
    let mut handle = world.ragdoll_mut(ragdoll).unwrap();
    handle
        .set_linear_and_angular_velocity(linear, angular)
        .unwrap();
    for (linear, angular) in [
        (
            Vec3::new(0.0, limits::MAX_LINEAR_VELOCITY.next_up(), 0.0),
            angular,
        ),
        (
            linear,
            Vec3::new(limits::MAX_ANGULAR_VELOCITY.next_up(), 0.0, 0.0),
        ),
    ] {
        assert!(matches!(
            handle.set_linear_and_angular_velocity(linear, angular),
            Err(RagdollError::InvalidValue(_))
        ));
    }
}

#[test]
fn provided_inertia_on_a_ragdoll_part() {
    let (mut world, layers) = ragdoll_world(1);
    let shapes = part_shapes();
    let mut parts = humanoid_parts(&shapes, layers.ragdoll);
    let inertia = Inertia::Principal(Vec3::new(0.03, 0.02, 0.03));
    parts[HEAD].body = parts[HEAD].body.clone().mass_and_inertia(5.0, inertia);
    let settings = RagdollSettings::new(&skeleton(), &parts).unwrap();
    let ragdoll = world
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    let head = world.ragdoll(ragdoll).unwrap().body_ids()[HEAD];
    assert!((world.body(head).unwrap().mass().unwrap() - 5.0).abs() <= 5.0e-6);
    match RagdollSettings::new_stabilized(&skeleton(), &parts) {
        Err(RagdollError::InvalidValue(rule)) => {
            assert!(
                rule.contains("stabilized ragdoll parts take no provided inertia"),
                "{rule}"
            )
        }
        other => panic!("{other:?}"),
    }
}
