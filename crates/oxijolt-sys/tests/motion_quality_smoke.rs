//! Smoke tests for the stored motion quality of the joltc additions,
//! `JPH_MotionProperties_GetMotionQuality`.

mod framework;

use framework::*;
use oxijolt_sys::*;

/// The motion quality stored in body `id`'s motion properties, read under a body read lock.
fn stored_quality(world: &TestWorld, id: JPH_BodyID) -> JPH_MotionQuality {
    // SAFETY: the lock interface belongs to the live system and joltc copies the id. The body
    // exists, so the lock yields it, locked until the lock is destroyed. joltc's unchecked getter
    // takes a mutable body but only reads the pointer; the body can move (checked first), so the
    // motion properties are live while the lock is held.
    unsafe {
        let lock_interface = JPH_PhysicsSystem_GetBodyLockInterface(world.system());
        let lock = JPH_BodyLockInterface_LockMultiRead(lock_interface, &id, 1);
        let body = JPH_BodyLockMultiRead_GetBody(lock, 0);
        assert!(!body.is_null());
        assert!(JPH_Body_CanBeKinematicOrDynamic(body));
        let properties = JPH_Body_GetMotionPropertiesUnchecked(body.cast_mut());
        assert!(!properties.is_null());
        let quality = JPH_MotionProperties_GetMotionQuality(properties);
        JPH_BodyLockMultiRead_Destroy(lock);
        quality
    }
}

/// The motion quality `JPH_BodyInterface_GetMotionQuality` reports for body `id`.
fn interface_quality(world: &TestWorld, id: JPH_BodyID) -> JPH_MotionQuality {
    // SAFETY: the body interface belongs to the live world; no body lock is held here.
    unsafe { JPH_BodyInterface_GetMotionQuality(world.body_interface(), id) }
}

/// Creates a unit box body of `motion_type` with `quality`, which may become dynamic or
/// kinematic later, adds it awake to `world` and returns its id.
fn create_body(
    world: &TestWorld,
    motion_type: JPH_MotionType,
    quality: JPH_MotionQuality,
) -> JPH_BodyID {
    let (half_extent, position, rotation) =
        (vec3(0.5, 0.5, 0.5), rvec3(0.0, 0.0, 0.0), quat_identity());
    let layer = if motion_type == JPH_MotionType_Static {
        OL_NON_MOVING
    } else {
        OL_MOVING
    };
    // SAFETY: the vectors are live locals. The shape holds one reference for this function; the
    // creation settings and the body take their own, and this function's references are
    // released once each after the body was created.
    let id = unsafe {
        let shape = JPH_BoxShape_Create(&half_extent, JPH_DEFAULT_CONVEX_RADIUS as f32);
        let settings = JPH_BodyCreationSettings_Create3(
            shape.cast_const().cast(),
            &position,
            &rotation,
            motion_type,
            layer,
        );
        JPH_BodyCreationSettings_SetAllowDynamicOrKinematic(settings, true);
        JPH_BodyCreationSettings_SetMotionQuality(settings, quality);
        let id = JPH_BodyInterface_CreateAndAddBody(
            world.body_interface(),
            settings,
            JPH_Activation_Activate,
        );
        JPH_BodyCreationSettings_Destroy(settings);
        JPH_Shape_Destroy(shape.cast());
        id
    };
    assert_ne!(id, u32::MAX, "body creation failed");
    id
}

#[test]
fn static_body_keeps_its_linear_cast_quality() {
    let world = TestWorld::new(1);
    let id = create_body(&world, JPH_MotionType_Static, JPH_MotionQuality_LinearCast);

    assert_eq!(stored_quality(&world, id), JPH_MotionQuality_LinearCast);
    assert_eq!(interface_quality(&world, id), JPH_MotionQuality_Discrete);
}

#[test]
fn dynamic_quality_follows_the_interface_setter() {
    let world = TestWorld::new(1);
    let id = create_body(&world, JPH_MotionType_Dynamic, JPH_MotionQuality_Discrete);
    let set = |quality| {
        // SAFETY: the body interface belongs to the live world; no body lock is held and no
        // step runs.
        unsafe { JPH_BodyInterface_SetMotionQuality(world.body_interface(), id, quality) }
    };
    let is_active = || {
        // SAFETY: as in `set`.
        unsafe { JPH_BodyInterface_IsActive(world.body_interface(), id) }
    };

    for active in [true, false] {
        assert_eq!(is_active(), active);
        assert_eq!(stored_quality(&world, id), JPH_MotionQuality_Discrete);
        set(JPH_MotionQuality_LinearCast);
        assert_eq!(stored_quality(&world, id), JPH_MotionQuality_LinearCast);
        assert_eq!(interface_quality(&world, id), JPH_MotionQuality_LinearCast);
        set(JPH_MotionQuality_Discrete);
        assert_eq!(stored_quality(&world, id), JPH_MotionQuality_Discrete);
        assert_eq!(interface_quality(&world, id), JPH_MotionQuality_Discrete);

        // SAFETY: as in `set`.
        unsafe { JPH_BodyInterface_DeactivateBody(world.body_interface(), id) };
    }
    assert!(!is_active());
}
