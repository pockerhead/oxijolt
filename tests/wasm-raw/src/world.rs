//! A world kept between export calls: a static floor, a falling box and a joltc step listener.

use std::cell::RefCell;
use std::ffi::c_void;
use std::ptr::null_mut;
use std::sync::atomic::{AtomicPtr, AtomicU32, Ordering};

use oxijolt_sys::*;

const OL_NON_MOVING: JPH_ObjectLayer = 0;
const OL_MOVING: JPH_ObjectLayer = 1;
const OBJECT_LAYER_COUNT: u32 = 2;

const BPL_NON_MOVING: JPH_BroadPhaseLayer = 0;
const BPL_MOVING: JPH_BroadPhaseLayer = 1;
const BROAD_PHASE_LAYER_COUNT: u32 = 2;

/// joltc's result for a body that could not be created.
const INVALID_BODY_ID: JPH_BodyID = u32::MAX;

/// Height the box starts falling from, in metres.
const BOX_START_HEIGHT: Real = 20.0;

/// Size of the world's temp allocator in bytes.
const TEMP_ALLOCATOR_BYTES: u32 = 10 * 1024 * 1024;

/// Step length in seconds.
const DELTA_TIME: f32 = 1.0 / 60.0;

/// Written into the user data of Jolt's static fixed-to-world body when the world is created. Its
/// C++ constructor sets the user data to 0, so the mark is gone after a second constructor run.
const FIXED_TO_WORLD_MARK: u64 = 0x6f78_696a_6f6c_7421;

thread_local! {
    static WORLD: RefCell<Option<World>> = const { RefCell::new(None) };
}

/// Number of `OnStep` calls since the world was created. Kept outside `WORLD`, which is borrowed
/// while the world steps.
static CALLS: AtomicU32 = AtomicU32::new(0);

/// The physics system `OnStep` saw last. joltc looks it up in its registry of physics systems, so
/// a world missing from the registry shows up here as null.
static LAST_SYSTEM: AtomicPtr<JPH_PhysicsSystem> = AtomicPtr::new(null_mut());

static PROCS: JPH_PhysicsStepListener_Procs = JPH_PhysicsStepListener_Procs {
    OnStep: Some(on_step),
};

/// Counts the step and records the system joltc passes.
///
/// # Safety
/// `context` is null or valid for the call, as joltc passes it.
unsafe extern "C" fn on_step(
    _user_data: *mut c_void,
    context: *const JPH_PhysicsStepListenerContext,
) {
    CALLS.fetch_add(1, Ordering::Relaxed);
    // SAFETY: joltc passes a context that lives for the duration of this call.
    let system = unsafe { context.as_ref() }.map_or(null_mut(), |context| context.physicsSystem);
    LAST_SYSTEM.store(system, Ordering::Relaxed);
}

/// Everything one world owns. Null pointers and `INVALID_BODY_ID` mark parts not created yet.
struct World {
    system: *mut JPH_PhysicsSystem,
    job_system: *mut JPH_JobSystem,
    temp_allocator: *mut JPH_TempAllocator,
    listener: *mut JPH_PhysicsStepListener,
    floor: JPH_BodyID,
    falling_box: JPH_BodyID,
    last_height: Real,
}

impl World {
    const EMPTY: World = World {
        system: null_mut(),
        job_system: null_mut(),
        temp_allocator: null_mut(),
        listener: null_mut(),
        floor: INVALID_BODY_ID,
        falling_box: INVALID_BODY_ID,
        last_height: BOX_START_HEIGHT,
    };

    /// Creates a world, or returns the `create_world` code of the part that failed.
    fn create() -> Result<World, i32> {
        let mut world = World::EMPTY;
        match world.populate() {
            Ok(()) => Ok(world),
            Err(code) => {
                world.destroy();
                Err(code)
            }
        }
    }

    fn populate(&mut self) -> Result<(), i32> {
        // SAFETY: the module is single-threaded, so no other joltc call runs meanwhile.
        if !unsafe { JPH_Init() } {
            return Err(2);
        }
        let fixed_to_world = fixed_to_world_body().ok_or(10)?;
        // SAFETY: the body is Jolt's static fixed-to-world body; its user data belongs to the
        // application.
        unsafe { JPH_Body_SetUserData(fixed_to_world, FIXED_TO_WORLD_MARK) };
        self.system = create_system();
        if self.system.is_null() {
            return Err(3);
        }
        // SAFETY: Jolt is initialised. 0 selects Jolt's `cMaxPhysicsJobs`.
        self.job_system = unsafe { JPH_JobSystemSingleThreaded_Create(0) };
        if self.job_system.is_null() {
            return Err(4);
        }
        // SAFETY: Jolt is initialised.
        self.temp_allocator = unsafe { JPH_TempAllocator_Create(TEMP_ALLOCATOR_BYTES) };
        if self.temp_allocator.is_null() {
            return Err(5);
        }
        let floor_half_extent = JPH_Vec3 {
            x: 50.0,
            y: 0.5,
            z: 50.0,
        };
        let floor_position = JPH_RVec3 {
            x: 0.0,
            y: -0.5,
            z: 0.0,
        };
        self.floor = self.add_box(
            floor_half_extent,
            floor_position,
            JPH_MotionType_Static,
            OL_NON_MOVING,
        )?;
        let box_half_extent = JPH_Vec3 {
            x: 0.5,
            y: 0.5,
            z: 0.5,
        };
        let box_position = JPH_RVec3 {
            x: 0.0,
            y: BOX_START_HEIGHT,
            z: 0.0,
        };
        self.falling_box = self.add_box(
            box_half_extent,
            box_position,
            JPH_MotionType_Dynamic,
            OL_MOVING,
        )?;
        // SAFETY: `PROCS` is a static, so it outlives every listener that reads it.
        self.listener = unsafe {
            JPH_PhysicsStepListener_SetProcs(&PROCS);
            JPH_PhysicsStepListener_Create(null_mut())
        };
        if self.listener.is_null() {
            return Err(9);
        }
        // SAFETY: system and listener are live; `destroy` removes the listener before
        // destroying either.
        unsafe { JPH_PhysicsSystem_AddStepListener(self.system, self.listener) };
        CALLS.store(0, Ordering::Relaxed);
        LAST_SYSTEM.store(null_mut(), Ordering::Relaxed);
        Ok(())
    }

    /// Adds an activated box body, or returns the `create_world` code of the failed call.
    fn add_box(
        &self,
        half_extent: JPH_Vec3,
        position: JPH_RVec3,
        motion_type: JPH_MotionType,
        layer: JPH_ObjectLayer,
    ) -> Result<JPH_BodyID, i32> {
        let rotation = JPH_Quat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        // SAFETY: the vector arguments are live locals. The shape is created holding one
        // reference, which `JPH_Shape_Destroy` releases; the settings and the body take their own.
        let shape = unsafe { JPH_BoxShape_Create(&half_extent, JPH_DEFAULT_CONVEX_RADIUS as f32) };
        if shape.is_null() {
            return Err(6);
        }
        // SAFETY: `shape` is live, the other arguments are live locals.
        let settings = unsafe {
            JPH_BodyCreationSettings_Create3(shape.cast(), &position, &rotation, motion_type, layer)
        };
        if settings.is_null() {
            // SAFETY: releases the reference `JPH_BoxShape_Create` returned, once.
            unsafe { JPH_Shape_Destroy(shape.cast()) };
            return Err(7);
        }
        // SAFETY: the system is live and owns the body interface; `settings` and `shape` are
        // released once each after the body took its own shape reference.
        let body = unsafe {
            let body_interface = JPH_PhysicsSystem_GetBodyInterface(self.system);
            let body = JPH_BodyInterface_CreateAndAddBody(
                body_interface,
                settings,
                JPH_Activation_Activate,
            );
            JPH_BodyCreationSettings_Destroy(settings);
            JPH_Shape_Destroy(shape.cast());
            body
        };
        if body == INVALID_BODY_ID {
            return Err(8);
        }
        Ok(body)
    }

    /// One step with the world's own temp allocator; see `step_world` for the codes.
    fn step(&mut self) -> i32 {
        let calls_before = CALLS.load(Ordering::Relaxed);
        // SAFETY: the world's temp allocator is live until `destroy`.
        if unsafe { self.update(self.temp_allocator) }.is_err() {
            return 2;
        }
        if CALLS.load(Ordering::Relaxed) != calls_before + 1 {
            return 3;
        }
        if LAST_SYSTEM.load(Ordering::Relaxed) != self.system {
            return 4;
        }
        let code = check_fixed_to_world();
        if code != 0 {
            return code;
        }
        let height = self.box_height();
        let falling = height.is_finite() && height < self.last_height;
        self.last_height = height;
        if !falling {
            return 6;
        }
        0
    }

    /// Steps once with `temp_allocator`.
    ///
    /// # Safety
    /// `temp_allocator` is a live joltc temp allocator.
    unsafe fn update(
        &self,
        temp_allocator: *mut JPH_TempAllocator,
    ) -> Result<(), JPH_PhysicsUpdateError> {
        // SAFETY: system and job system are live; the caller passes a live temp allocator.
        let error = unsafe {
            JPH_PhysicsSystem_Update2(self.system, DELTA_TIME, 1, temp_allocator, self.job_system)
        };
        if error == JPH_PhysicsUpdateError_None {
            Ok(())
        } else {
            Err(error)
        }
    }

    fn box_height(&self) -> Real {
        let mut position = JPH_RVec3 {
            x: 0.0,
            y: Real::NAN,
            z: 0.0,
        };
        // SAFETY: the system and its body interface are live and `falling_box` was added to it.
        unsafe {
            let body_interface = JPH_PhysicsSystem_GetBodyInterface(self.system);
            JPH_BodyInterface_GetPosition(body_interface, self.falling_box, &mut position);
        }
        position.y
    }

    /// Releases what was created, in reverse order. The layer tables belong to the system.
    fn destroy(self) {
        // SAFETY: every non-null pointer and valid id was created by `populate` and is released
        // once here; the listener is removed before the system is destroyed, the bodies before
        // the system.
        unsafe {
            if !self.listener.is_null() {
                JPH_PhysicsSystem_RemoveStepListener(self.system, self.listener);
                JPH_PhysicsStepListener_Destroy(self.listener);
            }
            if !self.system.is_null() {
                let body_interface = JPH_PhysicsSystem_GetBodyInterface(self.system);
                for body in [self.falling_box, self.floor] {
                    if body != INVALID_BODY_ID {
                        JPH_BodyInterface_RemoveAndDestroyBody(body_interface, body);
                    }
                }
                JPH_PhysicsSystem_Destroy(self.system);
            }
            if !self.job_system.is_null() {
                JPH_JobSystem_Destroy(self.job_system);
            }
            if !self.temp_allocator.is_null() {
                JPH_TempAllocator_Destroy(self.temp_allocator);
            }
        }
    }
}

/// Creates a physics system with two table-based layers; null if joltc fails.
fn create_system() -> *mut JPH_PhysicsSystem {
    // SAFETY: Jolt is initialised. Every pointer passed in comes from the `_Create` call above
    // it; `JPH_PhysicsSystem_Create` takes ownership of the three layer tables.
    unsafe {
        let pair_filter = JPH_ObjectLayerPairFilterTable_Create(OBJECT_LAYER_COUNT);
        JPH_ObjectLayerPairFilterTable_EnableCollision(pair_filter, OL_NON_MOVING, OL_MOVING);
        JPH_ObjectLayerPairFilterTable_EnableCollision(pair_filter, OL_MOVING, OL_MOVING);

        let broad_phase =
            JPH_BroadPhaseLayerInterfaceTable_Create(OBJECT_LAYER_COUNT, BROAD_PHASE_LAYER_COUNT);
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
            broad_phase,
            OL_NON_MOVING,
            BPL_NON_MOVING,
        );
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
            broad_phase,
            OL_MOVING,
            BPL_MOVING,
        );

        let object_vs_broad_phase = JPH_ObjectVsBroadPhaseLayerFilterTable_Create(
            broad_phase,
            BROAD_PHASE_LAYER_COUNT,
            pair_filter,
            OBJECT_LAYER_COUNT,
        );

        let settings = JPH_PhysicsSystemSettings {
            maxBodies: 1024,
            maxBodyPairs: 1024,
            maxContactConstraints: 1024,
            broadPhaseLayerInterface: broad_phase,
            objectLayerPairFilter: pair_filter,
            objectVsBroadPhaseLayerFilter: object_vs_broad_phase,
            ..std::mem::zeroed()
        };
        JPH_PhysicsSystem_Create(&settings)
    }
}

/// Jolt's static fixed-to-world body, built by a C++ static constructor; `None` if joltc returns
/// null.
fn fixed_to_world_body() -> Option<*mut JPH_Body> {
    // SAFETY: returns the address of a static object, which is never destroyed.
    let body = unsafe { JPH_Body_GetFixedToWorldBody() };
    (!body.is_null()).then_some(body)
}

/// Checks the fixed-to-world body for the `step_world` codes: 5 if it lacks the identity rotation
/// its constructor sets (the constructors never ran, so its memory is zero), 7 if it lost
/// `FIXED_TO_WORLD_MARK` (the constructors ran again since the world was created).
fn check_fixed_to_world() -> i32 {
    let Some(body) = fixed_to_world_body() else {
        return 5;
    };
    let mut rotation = JPH_Quat {
        x: f32::NAN,
        y: f32::NAN,
        z: f32::NAN,
        w: f32::NAN,
    };
    // SAFETY: `body` is Jolt's static fixed-to-world body and `rotation` a live local.
    let mark = unsafe {
        JPH_Body_GetRotation(body, &mut rotation);
        JPH_Body_GetUserData(body)
    };
    if (rotation.x, rotation.y, rotation.z, rotation.w) != (0.0, 0.0, 0.0, 1.0) {
        return 5;
    }
    if mark != FIXED_TO_WORLD_MARK {
        return 7;
    }
    0
}

/// Steps the live world once with `temp_allocator`, for `check_heaps`: 1 without a world, 4 on an
/// update error.
///
/// # Safety
/// `temp_allocator` is a live joltc temp allocator.
pub(crate) unsafe fn update_with(temp_allocator: *mut JPH_TempAllocator) -> Result<(), i32> {
    WORLD.with_borrow(|slot| {
        let world = slot.as_ref().ok_or(1)?;
        // SAFETY: the caller passes a live temp allocator.
        unsafe { world.update(temp_allocator) }.map_err(|_| 4)
    })
}

/// Creates the world: 0, 1 if a world exists, 2 if `JPH_Init` failed, 3 to 9 if a joltc create
/// call failed (system, job system, temp allocator, shape, body settings, body, step listener), 10
/// if joltc has no fixed-to-world body.
#[no_mangle]
pub extern "C" fn create_world() -> i32 {
    WORLD.with_borrow_mut(|slot| {
        if slot.is_some() {
            return 1;
        }
        match World::create() {
            Ok(world) => {
                *slot = Some(world);
                0
            }
            Err(code) => code,
        }
    })
}

/// Calls `JPH_Init` again: 0 when it returns true, 1 otherwise.
#[no_mangle]
pub extern "C" fn init_again() -> i32 {
    // SAFETY: the module is single-threaded, so no other joltc call runs meanwhile.
    if unsafe { JPH_Init() } {
        0
    } else {
        1
    }
}

/// Steps the world once: 0, 1 without a world, 2 on an update error, 3 if the listener did not
/// run exactly once, 4 if it did not see this world's system, 5 if the fixed-to-world body lacks
/// its identity rotation (constructors never ran), 7 if it lost the mark written at creation
/// (constructors ran again), 6 if the box is not finite and lower than after the previous step.
#[no_mangle]
pub extern "C" fn step_world() -> i32 {
    WORLD.with_borrow_mut(|slot| match slot {
        Some(world) => world.step(),
        None => 1,
    })
}

/// Number of listener calls since the world was created.
#[no_mangle]
pub extern "C" fn listener_calls() -> u32 {
    CALLS.load(Ordering::Relaxed)
}

/// Destroys the world: 0, 1 without a world.
#[no_mangle]
pub extern "C" fn destroy_world() -> i32 {
    match WORLD.with_borrow_mut(Option::take) {
        Some(world) => {
            world.destroy();
            0
        }
        None => 1,
    }
}
