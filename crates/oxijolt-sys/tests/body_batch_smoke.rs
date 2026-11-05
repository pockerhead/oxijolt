//! Smoke tests for the batch body functions of the joltc additions: preparing, finalizing and
//! aborting a batch add, removing several bodies and destroying several bodies, each refusing
//! repeated ids and bodies in the wrong state without changing anything.

mod framework;

use framework::*;
use oxijolt_sys::*;

/// Creates a dynamic box at `x` without adding it and returns its id.
fn create_detached(world: &TestWorld, x: Real) -> JPH_BodyID {
    let (half_extent, position, rotation) =
        (vec3(0.5, 0.5, 0.5), rvec3(x, 0.0, 0.0), quat_identity());
    // SAFETY: the body interface belongs to the live world. The shape is created holding one
    // reference; the settings and the body take their own, so ours and the settings' are
    // released after the body exists. The vectors are live locals.
    let id = unsafe {
        let shape = JPH_BoxShape_Create(&half_extent, JPH_DEFAULT_CONVEX_RADIUS as f32);
        let settings = JPH_BodyCreationSettings_Create3(
            shape as *const JPH_Shape,
            &position,
            &rotation,
            JPH_MotionType_Dynamic,
            OL_MOVING,
        );
        let body = JPH_BodyInterface_CreateBody(world.body_interface(), settings);
        JPH_BodyCreationSettings_Destroy(settings);
        JPH_Shape_Destroy(shape as *mut JPH_Shape);
        assert!(!body.is_null(), "body creation failed");
        JPH_Body_GetID(body)
    };
    assert!(!is_added(world, id));
    id
}

fn is_added(world: &TestWorld, id: JPH_BodyID) -> bool {
    // SAFETY: the body interface belongs to the live world; `IsAdded` locks and checks any id.
    unsafe { JPH_BodyInterface_IsAdded(world.body_interface(), id) }
}

fn is_active(world: &TestWorld, id: JPH_BodyID) -> bool {
    // SAFETY: as in `is_added`.
    unsafe { JPH_BodyInterface_IsActive(world.body_interface(), id) }
}

fn body_count(world: &TestWorld) -> u32 {
    // SAFETY: the system is live.
    unsafe { JPH_PhysicsSystem_GetNumBodies(world.system()) }
}

/// Prepares `ids`, which must be accepted.
fn prepare(world: &TestWorld, ids: &[JPH_BodyID]) -> *mut JPH_AddBodiesState {
    // SAFETY: the body interface is live and `ids` holds `ids.len()` ids for the call.
    let state = unsafe {
        JPH_BodyInterface_AddBodiesPrepare(world.body_interface(), ids.as_ptr(), ids.len() as u32)
    };
    assert!(!state.is_null(), "prepare refused {ids:?}");
    state
}

/// Whether prepare refuses `ids`; an accepted batch is aborted again.
fn prepare_refuses(world: &TestWorld, ids: &[JPH_BodyID]) -> bool {
    // SAFETY: the body interface is live and `ids` holds `ids.len()` ids for the call.
    let state = unsafe {
        JPH_BodyInterface_AddBodiesPrepare(world.body_interface(), ids.as_ptr(), ids.len() as u32)
    };
    if state.is_null() {
        return true;
    }
    // SAFETY: `state` was just prepared and goes to abort exactly once.
    unsafe { JPH_BodyInterface_AddBodiesAbort(world.body_interface(), state) };
    false
}

#[test]
fn a_prepared_batch_is_aborted_or_finalized() {
    let world = TestWorld::new(1);
    let ids = [
        create_detached(&world, 4.0),
        create_detached(&world, 0.0),
        create_detached(&world, 2.0),
    ];
    let caller = ids;

    let state = prepare(&world, &ids);
    assert_eq!(ids, caller, "the caller's array is not reordered");
    // SAFETY: `state` was prepared above and goes to abort exactly once.
    unsafe { JPH_BodyInterface_AddBodiesAbort(world.body_interface(), state) };
    assert!(
        ids.iter().all(|&id| !is_added(&world, id)),
        "abort adds nothing"
    );
    assert_eq!(body_count(&world), 3, "abort destroys nothing");

    let state = prepare(&world, &ids);
    // SAFETY: `state` was prepared above and goes to finalize exactly once.
    unsafe {
        JPH_BodyInterface_AddBodiesFinalize(world.body_interface(), state, JPH_Activation_Activate)
    };
    assert_eq!(ids, caller);
    assert!(ids.iter().all(|&id| is_added(&world, id)));
    assert!(ids.iter().all(|&id| is_active(&world, id)));
    world.step(1.0 / 60.0);
}

#[test]
fn finalize_without_activation_leaves_the_bodies_asleep() {
    let world = TestWorld::new(1);
    let ids = [create_detached(&world, 0.0), create_detached(&world, 2.0)];
    let state = prepare(&world, &ids);
    // SAFETY: `state` was prepared above and goes to finalize exactly once.
    unsafe {
        JPH_BodyInterface_AddBodiesFinalize(
            world.body_interface(),
            state,
            JPH_Activation_DontActivate,
        )
    };
    assert!(ids.iter().all(|&id| is_added(&world, id)));
    assert!(ids.iter().all(|&id| !is_active(&world, id)));
}

#[test]
fn prepare_refuses_repeated_added_and_unknown_ids() {
    let world = TestWorld::new(1);
    let detached = create_detached(&world, 0.0);
    let added = create_box(
        world.body_interface(),
        vec3(0.5, 0.5, 0.5),
        rvec3(3.0, 0.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );
    let unknown = detached + 7;

    assert!(prepare_refuses(&world, &[detached, detached]));
    assert!(prepare_refuses(&world, &[detached, added]));
    assert!(prepare_refuses(&world, &[unknown, detached]));
    assert!(prepare_refuses(&world, &[u32::MAX]));
    assert!(
        !is_added(&world, detached),
        "a refused prepare adds nothing"
    );
    assert!(!prepare_refuses(&world, &[]), "an empty batch is accepted");
    assert!(!prepare_refuses(&world, &[detached]));
}

#[test]
fn remove_and_destroy_check_every_id_and_keep_the_destroy_order() {
    let world = TestWorld::new(1);
    let ids = [
        create_detached(&world, 0.0),
        create_detached(&world, 2.0),
        create_detached(&world, 4.0),
    ];
    let state = prepare(&world, &ids);
    // SAFETY: `state` was prepared above and goes to finalize exactly once.
    unsafe {
        JPH_BodyInterface_AddBodiesFinalize(world.body_interface(), state, JPH_Activation_Activate)
    };
    let interface = world.body_interface();
    let extra = create_detached(&world, 6.0);
    let remove = |list: &[JPH_BodyID]| {
        // SAFETY: the body interface is live and `list` holds `list.len()` ids for the call.
        unsafe { JPH_BodyInterface_RemoveBodies(interface, list.as_ptr(), list.len() as u32) }
    };
    let destroy = |list: &[JPH_BodyID]| {
        // SAFETY: as in `remove`.
        unsafe { JPH_BodyInterface_DestroyBodies(interface, list.as_ptr(), list.len() as u32) }
    };

    assert!(!remove(&[ids[0], ids[0]]), "a repeated id is refused");
    assert!(
        !remove(&[ids[0], extra]),
        "a body that is not added is refused"
    );
    assert!(
        ids.iter().all(|&id| is_added(&world, id)),
        "nothing was removed"
    );
    assert!(!destroy(&[ids[0]]), "an added body is not destroyed");
    assert!(remove(&[]));

    let order = [ids[1], ids[2], ids[0]];
    let caller = order;
    assert!(remove(&order));
    assert_eq!(order, caller, "the caller's array is not reordered");
    assert!(ids.iter().all(|&id| !is_added(&world, id)));
    assert_eq!(body_count(&world), 4, "removed bodies stay created");

    assert!(!destroy(&[ids[1], ids[1]]), "a repeated id is refused");
    assert!(!destroy(&[ids[1], ids[1] + 7]), "an unknown id is refused");
    assert_eq!(body_count(&world), 4, "nothing was destroyed");
    assert!(destroy(&order));
    assert!(destroy(&[]));
    assert_eq!(body_count(&world), 1);

    // Jolt's free list hands out the last freed index first: the destroy kept the order given.
    let next = create_detached(&world, 8.0);
    assert_eq!(next & 0x7f_ffff, ids[0] & 0x7f_ffff);
}
