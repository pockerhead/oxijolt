//! The trait rules of `docs/api-guidelines.md` (R6, R7), checked at compile time: ids, errors,
//! settings and the settings with a `Default`. A type that loses a promised trait stops this file
//! from compiling.

use std::error::Error;
use std::fmt::Debug;
use std::hash::Hash;

use oxijolt::*;

fn id<T: Clone + Copy + Debug + PartialEq + Eq + PartialOrd + Ord + Hash>() {}
fn error<T: Clone + Copy + Debug + PartialEq + Eq + Error>() {}
fn settings<T: Clone + Debug + PartialEq>() {}
fn default<T: Default>() {}
fn copy<T: Copy>() {}
fn thread_safe<T: Send + Sync>() {}

#[test]
fn ids_compare_order_and_hash() {
    id::<BodyId>();
    id::<CharacterId>();
    id::<ConstraintId<HingeConstraint>>();
    id::<AnyConstraintId>();
    id::<VehicleId>();
    id::<VehicleId<TrackedVehicle>>();
    id::<AnyVehicleId>();
    id::<RagdollId>();
    id::<SubShapeId>();
}

#[test]
fn errors_are_copy_and_comparable() {
    error::<WorldError>();
    error::<ShapeError>();
    error::<BinaryStateError>();
    error::<ConvexHullError>();
    error::<MeshError>();
    error::<ThinTrianglesError>();
    error::<BodyError>();
    error::<StepError>();
    error::<QueryError>();
    error::<ContactSettingsError>();
    error::<CharacterError>();
    error::<VehicleError>();
    error::<RagdollError>();
    error::<ConstraintError>();
    error::<SoftBodyError>();
    error::<StateError>();
    error::<CollisionGroupError>();
    error::<oxijolt::error::Error>();
}

#[test]
fn settings_are_clone_and_comparable() {
    settings::<BodySettings>();
    settings::<Inertia>();
    copy::<Inertia>();
    settings::<BuoyancySettings>();
    settings::<ExtendedUpdateSettings>();
    settings::<CharacterContactSettings>();
    settings::<HeightFieldSettings>();
    settings::<SoftBodySettings>();
    settings::<EventSettings>();
    settings::<ContactSettings>();
    settings::<SoftBodyContactSettings>();
    for_skinning();
    settings::<PointConstraintSettings>();
    settings::<DistanceConstraintSettings>();
    settings::<FixedConstraintSettings>();
    settings::<HingeConstraintSettings>();
    settings::<SliderConstraintSettings>();
    settings::<ConeConstraintSettings>();
    settings::<SwingTwistConstraintSettings>();
    settings::<SixDofConstraintSettings>();
    settings::<PathConstraintSettings>();
    settings::<PulleyConstraintSettings>();
    settings::<GearConstraintSettings>();
    settings::<RackAndPinionConstraintSettings>();
    settings::<MotorSettings>();
    settings::<SpringSettings>();
    settings::<WheelSettings>();
    settings::<WheeledVehicleSettings>();
    settings::<VehicleEngineSettings>();
    settings::<VehicleTransmissionSettings>();
    settings::<VehicleDifferentialSettings>();
    settings::<MotorcycleSettings>();
    settings::<TrackedVehicleSettings>();
    settings::<TrackedWheelSettings>();
    settings::<VehicleTrackSettings>();
    #[cfg(feature = "debug-renderer")]
    settings::<DebugLineSettings>();
}

/// The plain inputs of soft body skinning: copyable, comparable and free to send.
fn for_skinning() {
    settings::<SoftBodyInvBind>();
    settings::<SoftBodySkinWeight>();
    settings::<SoftBodySkinned>();
    settings::<SoftBodyBackStop>();
    copy::<SoftBodyInvBind>();
    copy::<SoftBodySkinWeight>();
    copy::<SoftBodySkinned>();
    copy::<SoftBodyBackStop>();
    thread_safe::<SoftBodyInvBind>();
    thread_safe::<SoftBodySkinWeight>();
    thread_safe::<SoftBodySkinned>();
    thread_safe::<SoftBodyBackStop>();
}

#[test]
fn settings_without_required_inputs_have_defaults() {
    default::<WorldSettings>();
    default::<BodySettings>();
    default::<BuoyancySettings>();
    default::<ExtendedUpdateSettings>();
    default::<MeshSettings>();
    default::<HeightFieldSettings>();
    default::<SoftBodySettings>();
    default::<SoftBodySkinWeight>();
    default::<EventSettings>();
    default::<PointConstraintSettings>();
    default::<DistanceConstraintSettings>();
    default::<FixedConstraintSettings>();
    default::<HingeConstraintSettings>();
    default::<SliderConstraintSettings>();
    default::<ConeConstraintSettings>();
    default::<SwingTwistConstraintSettings>();
    default::<SixDofConstraintSettings>();
    default::<PulleyConstraintSettings>();
    default::<GearConstraintSettings>();
    default::<RackAndPinionConstraintSettings>();
    default::<MotorSettings>();
    default::<SpringSettings>();
    default::<VehicleEngineSettings>();
    default::<VehicleTransmissionSettings>();
}
