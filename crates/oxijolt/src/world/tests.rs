use super::rebase::{BodyFrameState, FrameChange};
use super::*;
use crate::{Quat, RVec3, Real};

#[test]
fn any_full_buffer_makes_a_report_incomplete() {
    let complete = StepReport {
        manifold_cache_full: false,
        body_pair_cache_full: false,
        contact_constraints_full: false,
        rejected_contact_settings: 0,
    };
    assert!(complete.is_complete());
    for report in [
        StepReport {
            manifold_cache_full: true,
            ..complete
        },
        StepReport {
            body_pair_cache_full: true,
            ..complete
        },
        StepReport {
            contact_constraints_full: true,
            ..complete
        },
    ] {
        assert!(!report.is_complete(), "{report:?}");
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn single_threaded_is_chosen_last_wins_and_has_concurrency_one() {
    let settings = WorldSettings::default().worker_threads(2).single_threaded();
    assert!(
        matches!(settings.jobs, JobSystemChoice::SingleThreaded),
        "{settings:?}"
    );
    assert_eq!(settings.jobs.max_concurrency(), Ok(1));

    let settings = WorldSettings::default().single_threaded().worker_threads(2);
    assert!(
        matches!(settings.jobs, JobSystemChoice::ThreadPool(2)),
        "{settings:?}"
    );

    let world = PhysicsWorld::new(WorldSettings::default().single_threaded()).unwrap();
    assert!(matches!(
        world.job_system,
        WorldJobSystem::SingleThreaded(_)
    ));
}

fn moving_state() -> BodyFrameState {
    BodyFrameState {
        position: RVec3::new(1.0, 0.0, 0.0),
        rotation: Quat::from_xyzw(0.0, 0.0, 0.6, 0.8),
        linear_velocity: Vec3::new(1.0, 0.0, 0.0),
        angular_velocity: Vec3::new(1.0, 0.0, 0.0),
    }
}

#[test]
fn translation_only_frame_change_keeps_rotation_and_velocity_bits() {
    let frame = FrameChange {
        rotation: Quat::IDENTITY,
        translation: RVec3::new(1.0, 2.0, 3.0),
    };
    let state = moving_state();
    let moved = frame.body(state).unwrap();
    assert_eq!(moved.position, RVec3::new(2.0, 2.0, 3.0));
    let bits = |q: Quat| <[f32; 4]>::from(q).map(f32::to_bits);
    assert_eq!(bits(moved.rotation), bits(state.rotation));
    assert_eq!(moved.linear_velocity, state.linear_velocity);
    assert_eq!(moved.angular_velocity, state.angular_velocity);
}

#[test]
fn quarter_turn_about_y_maps_x_to_minus_z() {
    let half = std::f32::consts::FRAC_1_SQRT_2;
    let frame = FrameChange {
        rotation: Quat::from_xyzw(0.0, half, 0.0, half),
        translation: RVec3::ZERO,
    };
    let moved = frame.body(moving_state()).unwrap();
    let p = moved.position;
    let near =
        |a: [f32; 3]| (a[0].abs() < 1e-6) && (a[1].abs() < 1e-6) && ((a[2] + 1.0).abs() < 1e-6);
    assert!(p.x.abs() < 1e-6 && p.y.abs() < 1e-6, "{moved:?}");
    assert!((p.z + 1.0).abs() < 1e-6, "{moved:?}");
    assert!(near(moved.linear_velocity.into()), "{moved:?}");
    assert!(near(moved.angular_velocity.into()), "{moved:?}");
    assert!(moved.rotation.is_valid_rotation(), "{moved:?}");
}

#[test]
fn frame_change_rejects_overflow_and_nan() {
    let frame = FrameChange {
        rotation: Quat::IDENTITY,
        translation: RVec3::new(Real::MAX, 0.0, 0.0),
    };
    let mut state = moving_state();
    state.position.x = Real::MAX / 2.0;
    assert_eq!(frame.body(state), None);

    let frame = FrameChange {
        rotation: Quat::IDENTITY,
        translation: RVec3::ZERO,
    };
    let mut state = moving_state();
    state.linear_velocity.y = f32::NAN;
    assert_eq!(frame.body(state), None);
}

#[test]
fn contact_constraint_capacity_is_bounded() {
    let settings = |value| WorldSettings::default().max_contact_constraints(value);
    for valid in [1, WorldSettings::MAX_CONTACT_CONSTRAINTS] {
        assert_eq!(settings(valid).validate(), Ok(()));
    }
    for invalid in [0, WorldSettings::MAX_CONTACT_CONSTRAINTS + 1, u32::MAX] {
        assert!(matches!(
            settings(invalid).validate(),
            Err(WorldError::InvalidValue(_))
        ));
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn worker_thread_bounds_are_validated() {
    for valid in [1, WorldSettings::MAX_WORKER_THREADS] {
        assert_eq!(
            WorldSettings::default().worker_threads(valid).validate(),
            Ok(())
        );
    }
    for invalid in [0, WorldSettings::MAX_WORKER_THREADS + 1, u32::MAX] {
        assert!(matches!(
            WorldSettings::default().worker_threads(invalid).validate(),
            Err(WorldError::InvalidValue(_))
        ));
    }
}

#[test]
fn a_target_without_threads_refuses_every_thread_pool() {
    for workers in [0, 1, WorldSettings::MAX_WORKER_THREADS] {
        assert!(matches!(
            JobSystemChoice::ThreadPool(workers).validate(false),
            Err(WorldError::InvalidValue(_))
        ));
    }
    assert_eq!(JobSystemChoice::SingleThreaded.validate(false), Ok(()));
}

#[test]
#[cfg(target_arch = "wasm32")]
fn wasm32_defaults_to_single_threaded_and_refuses_worker_threads() {
    let settings = WorldSettings::default();
    assert_eq!(settings.job_system_kind(), JobSystemKind::SingleThreaded);
    assert_eq!(settings.validate(), Ok(()));
    assert!(matches!(
        settings.worker_threads(1).validate(),
        Err(WorldError::InvalidValue(_))
    ));
}

/// Jolt's `PhysicsSettings` defaults (`PhysicsSettings.h:36-126`) with the values oxijolt sets.
fn jolt_default_physics_settings(
    velocity_steps: u32,
    position_steps: u32,
    penetration_slop: f32,
    speculative_contact_distance: f32,
) -> JPH_PhysicsSettings {
    JPH_PhysicsSettings {
        maxInFlightBodyPairs: 16384,
        stepListenersBatchSize: 8,
        stepListenerBatchesPerJob: 1,
        baumgarte: 0.2,
        speculativeContactDistance: speculative_contact_distance,
        penetrationSlop: penetration_slop,
        linearCastThreshold: 0.75,
        linearCastMaxPenetration: 0.25,
        manifoldTolerance: 1.0e-3,
        maxPenetrationDistance: 0.2,
        bodyPairCacheMaxDeltaPositionSq: 0.001 * 0.001,
        bodyPairCacheCosMaxDeltaRotationDiv2: 1.0f64.to_radians().cos() as f32,
        contactNormalCosMaxDeltaRotation: 5.0f64.to_radians().cos() as f32,
        contactPointPreserveLambdaMaxDistSq: 0.01 * 0.01,
        numVelocitySteps: velocity_steps,
        numPositionSteps: position_steps,
        minVelocityForRestitution: 1.0,
        timeBeforeSleep: 0.5,
        pointVelocitySleepThreshold: 0.03,
        deterministicSimulation: true,
        constraintWarmStart: true,
        useBodyPairContactCache: true,
        useManifoldReduction: true,
        useLargeIslandSplitter: true,
        allowSleeping: true,
        checkActiveEdges: true,
    }
}

#[test]
fn world_settings_leave_every_other_physics_setting_at_jolts_default() {
    let cases = [
        (WorldSettings::default(), (10, 2, 0.02, 0.02)),
        (
            WorldSettings::default()
                .velocity_steps(4)
                .position_steps(0)
                .penetration_slop(0.005)
                .speculative_contact_distance(0.05),
            (4, 0, 0.005, 0.05),
        ),
        (
            WorldSettings::default()
                .velocity_steps(255)
                .position_steps(255)
                .penetration_slop(0.0)
                .speculative_contact_distance(limits::MAX_SHAPE_EXTENT),
            (255, 255, 0.0, 2000.0),
        ),
        (
            WorldSettings::default()
                .penetration_slop(limits::MAX_SHAPE_EXTENT)
                .speculative_contact_distance(0.0),
            (10, 2, 2000.0, 0.0),
        ),
    ];
    for (settings, (velocity, position, slop, distance)) in cases {
        let world = PhysicsWorld::new(settings).unwrap();
        // SAFETY: the world's system is live and nothing steps it.
        let read = unsafe { physics_settings(world.system.as_ptr()) };
        assert_eq!(
            format!("{read:?}"),
            format!(
                "{:?}",
                jolt_default_physics_settings(velocity, position, slop, distance)
            )
        );
    }
}

#[test]
fn contact_distances_are_validated() {
    type Setter = fn(WorldSettings, f32) -> WorldSettings;
    let setters: [(&str, Setter); 2] = [
        ("penetration_slop", WorldSettings::penetration_slop),
        (
            "speculative_contact_distance",
            WorldSettings::speculative_contact_distance,
        ),
    ];
    for (name, set) in setters {
        for valid in [0.0, -0.0, 0.02, limits::MAX_SHAPE_EXTENT] {
            assert_eq!(
                set(WorldSettings::default(), valid).validate(),
                Ok(()),
                "{name}({valid:?}) was refused"
            );
        }
        for invalid in [
            -f32::from_bits(1),
            -0.001,
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            f32::MAX,
            limits::MAX_SHAPE_EXTENT.next_up(),
        ] {
            assert!(
                matches!(
                    set(WorldSettings::default(), invalid).validate(),
                    Err(WorldError::InvalidValue(_))
                ),
                "{name}({invalid:?}) was accepted"
            );
        }
    }
}
