//! Unsafe bindings to [Jolt Physics] 5.6.0 through the [joltc] C wrapper.
//!
//! Everything here is `bindgen` output over joltc's `include/joltc.h` plus this
//! fork's `native/joltc_ext/joltc_ext.h`, and keeps their `JPH_*` names. The output is
//! committed under `src/bindings/`, one file per ABI family and configuration, so building
//! needs no libclang; the `bindgen` feature generates it at build time instead. The
//! supported targets are the registry in `build/targets.rs`. Of the 32-bit targets only
//! `wasm32-unknown-unknown` builds. The
//! extension adds functions in joltc's naming that are compiled into the joltc
//! archive: a `JPH_StateRecorder`, `JPH_CharacterVirtual_SaveState` and
//! `RestoreState`, `JPH_CharacterVirtual_ExtendedUpdate2` and
//! `RefreshContacts2` with explicit gravity, filters and temp allocator, a
//! filter-explicit `JPH_CharacterVirtual_SetShape2`, a character's supporting volume
//! (`JPH_CharacterBase_SetSupportingVolume`, `_GetSupportingVolume`),
//! `JPH_VehicleConstraint_AsConstraint`, the `Constraint` base of a vehicle
//! constraint, and for ragdolls `JPH_RagdollSettings_SetPart`, the typed
//! `JPH_RagdollSettings_SetPartToParentSwingTwist`, `_SetPartToParentHinge` and
//! `_SetPartToParentSixDOF`, `JPH_RagdollSettings_CalculateConstraintPriorities`,
//! the swing-twist motor states (`JPH_SwingTwistConstraint_SetSwingMotorState`,
//! `_GetSwingMotorState`, `_SetTwistMotorState`, `_GetTwistMotorState`), its
//! `SetTargetOrientationBS` and `GetRotationInConstraintSpace`, and
//! `JPH_HingeConstraint_SetTargetOrientationBS`, and materials with user data
//! (`JPH_PhysicsMaterial_Create2`, `JPH_PhysicsMaterial_GetUserData`,
//! `JPH_ConvexShapeSettings_SetMaterial`, `JPH_HeightFieldShapeSettings_Create2`), the
//! sub-shape pair of a removed contact (`JPH_SubShapeIDPair_GetBody1ID` and its three siblings),
//! a contact listener whose validate callback gets the collide result without faces
//! (`JPH_ContactListener2_*`, `JPH_PhysicsSystem_SetContactListener2`) and a soft body contact
//! listener (`JPH_SoftBodyContactListener_*`,
//! `JPH_PhysicsSystem_SetSoftBodyContactListener`, `JPH_SoftBodyManifold_*`), and shapes saved
//! to bytes with their children and materials and restored (`JPH_Shape_SaveBinaryState`,
//! `JPH_Shape_RestoreBinaryState`, `JPH_ShapeBinaryState_*`). The safe API lives in the
//! `oxijolt` crate.
//!
//! # Features
//! - `asserts`: compile Jolt with its debug assertions. joltc's default handler prints a failed
//!   assertion and then executes a breakpoint (`__debugbreak` on MSVC), so raw users should
//!   install their own with `JPH_SetAssertFailureHandler` before `JPH_Init`. The handler is one
//!   process-global joltc slot, and the `oxijolt` crate installs its own.
//! - `double-precision`: world positions ([`Real`], `JPH_RVec3`, `JPH_RMat4`) use `f64`.
//! - `cross-platform-deterministic`: build Jolt with its cross-platform deterministic
//!   floating point settings (slower; same results across compilers and platforms).
//! - `debug-renderer`: compile Jolt's debug renderer into the native libraries and bind joltc's
//!   debug drawing functions.
//! - `bindgen`: generate the bindings at build time with libclang instead of using the
//!   committed ones. It does not add supported targets.
//!
//! # Native build and `JOLTC_LIB_DIR`
//! By default the build script builds joltc and Jolt from the `vendor/` submodules with
//! CMake, always in Release. Set `JOLTC_LIB_DIR` to an install prefix produced by an
//! earlier build (`OUT_DIR/joltc`) to skip CMake: it holds `lib/` with the joltc and Jolt
//! static libraries, `include/joltc.h`, `include/joltc_ext.h` and
//! `oxijolt-sys-manifest.txt`. A prefix is specific
//! to the target, the C runtime, the crate features and the pinned joltc and Jolt
//! commits; the build script validates all of these against the manifest and refuses a
//! mismatch.
//!
//! # Using the raw API
//! joltc's own contract, as observed in its source:
//! - Shape creators such as `JPH_BoxShape_Create` return a shape that holds **one**
//!   reference, and `JPH_Shape_Destroy` releases one. Bodies keep their own reference, so
//!   the creator releases its reference once it no longer needs the shape.
//! - `JPH_PhysicsSystem_Destroy` deletes the broad-phase layer interface and both layer
//!   filters passed in `JPH_PhysicsSystemSettings`. Do not destroy them yourself.
//! - `JPH_Init` is guarded by a plain `bool`, and creating or destroying a physics system
//!   writes an unsynchronised global map. Call `JPH_Init` once, and create and destroy
//!   systems from one thread at a time.
//! - `JPH_PhysicsSystem_Update` uses one global temp allocator. Use
//!   `JPH_PhysicsSystem_Update2` with a per-world `JPH_TempAllocator` instead.
//! - joltc's `JPH_CharacterVirtual_Update`, `ExtendedUpdate`, `RefreshContacts`,
//!   `WalkStairs`, `StickToFloor` and `SetShape` use joltc's one global temp
//!   allocator and are not thread-safe. Use `JPH_CharacterVirtual_ExtendedUpdate2`
//!   and `JPH_CharacterVirtual_RefreshContacts2` with a per-world allocator instead.
//! - `JPH_CharacterVirtualSettings_Init` (and `JPH_CharacterSettings_Init`) create an
//!   empty shape and its settings on every call, each holding a reference nobody
//!   releases. Fill the settings field by field instead.
//! - A recorder passed to `JPH_CharacterVirtual_RestoreState` must hold a complete
//!   stream written by `JPH_CharacterVirtual_SaveState`.
//! - `JPH_JobSystemThreadPool_Create` maps `numThreads <= 0` to "as many as there are
//!   hardware threads", so pass a positive worker count when the count matters.
//! - joltc's object-layer, body and shape filters (`JPH_ObjectLayerFilter_*`,
//!   `JPH_BodyFilter_*`, `JPH_ShapeFilter_*`) call one process-global proc table per filter
//!   type. The `oxijolt` crate installs those tables once and owns them. Code that links
//!   both crates must not call `JPH_*Filter_SetProcs` for these three types, and must not pass
//!   filters it created with `JPH_*Filter_Create` to queries, because the callbacks of
//!   `oxijolt` would receive their `userData`.
//! - joltc's body activation and character contact listeners and the extension's contact
//!   listener (`JPH_ContactListener2_*`) and soft body contact listener also call one
//!   process-global proc table each. The `oxijolt` crate installs them once and owns them: code
//!   that links both crates must not call `JPH_ContactListener2_SetProcs`,
//!   `JPH_BodyActivationListener_SetProcs`, `JPH_CharacterContactListener_SetProcs` or
//!   `JPH_SoftBodyContactListener_SetProcs`, nor create listeners of these types, whose
//!   `userData` the callbacks of `oxijolt` would receive. joltc's own contact listener (`JPH_ContactListener_*`) is not used by `oxijolt`
//!   and is free for other code.
//! - A vehicle constraint must be registered both as a constraint
//!   (`JPH_PhysicsSystem_AddConstraint` with `JPH_VehicleConstraint_AsConstraint`) and as a
//!   step listener (`JPH_PhysicsSystem_AddStepListener` with
//!   `JPH_VehicleConstraint_AsPhysicsStepListener`), and its collision tester must be set
//!   before the first step. Remove it from both before its last reference is released
//!   (`JPH_Constraint_Destroy` on the `AsConstraint` pointer) and before its body is
//!   destroyed.
//! - `JPH_VehicleEngineSettings_Init` allocates a `JPH_LinearCurve` for `normalizedTorque`
//!   that the caller must destroy with `JPH_LinearCurve_Destroy`.
//! - The `JPH_Wheel_GetContact*` getters are meaningful only while `JPH_Wheel_HasContact`
//!   returns true.
//! - `JPH_RagdollSettings_CreateRagdoll` dereferences null when the world cannot hold every
//!   part. Check `JPH_PhysicsSystem_GetNumBodies() + parts <= JPH_PhysicsSystem_GetMaxBodies()`
//!   first, with no body created concurrently. The skeleton must list parents before their
//!   children.
//! - A ragdoll destroys its bodies through its physics system when its last reference is
//!   released: remove it from the system (`JPH_Ragdoll_RemoveFromPhysicsSystem`) before that,
//!   and release it before the system is destroyed.
//! - `JPH_Ragdoll_DriveToPoseUsingMotors` drives only swing-twist and hinge parts (any other
//!   constraint is a Jolt assertion) and needs
//!   `JPH_RagdollSettings_CalculateBodyIndexToConstraintIndex` to have run.
//! - `JPH_RagdollSettings_SetPartToParent` handles swing-twist only and drops the constraint
//!   base settings, the spring modes and the motor torque limits; the typed
//!   `JPH_RagdollSettings_SetPartToParent*` functions of the extension keep every field.
//! - A motor state other than `JPH_MotorState_Off` needs valid motor settings (Jolt
//!   `MotorSettings::IsValid`).
//! - These joltc functions are not bound: `JPH_RagdollSettings_DisableParentChildCollisions`,
//!   `JPH_Ragdoll_SetPose2`, `JPH_Ragdoll_GetPose2`, `JPH_SkeletonMapper_Initialize`,
//!   `JPH_SkeletonMapper_LockAllTranslations`, `JPH_SkeletonMapper_LockTranslations`,
//!   `JPH_SkeletonMapper_Map` and `JPH_SkeletonMapper_MapReverse`. They reinterpret a
//!   4-aligned `JPH_Mat4` array as Jolt's 16-aligned `Mat44`, which is undefined
//!   behaviour for most arrays a caller can pass.
//! - The debug drawing functions (`JPH_DebugRenderer_*`, `JPH_BodyDrawFilter_*`,
//!   `JPH_PhysicsSystem_Draw*`, `JPH_Shape_Draw`) exist only with the `debug-renderer` feature,
//!   which also compiles Jolt's debug renderer into the native libraries.
//! - Jolt's `DebugRenderer` is a process singleton and joltc's `JPH_DebugRenderer_SetProcs` sets
//!   one global proc table. With the `debug-renderer` feature of `oxijolt`, that crate
//!   installs the table and creates a renderer during `PhysicsWorld::debug_lines`; code that
//!   links both crates must not call `JPH_DebugRenderer_SetProcs` or keep its own
//!   `JPH_DebugRenderer` alive.
//!
//! # wasm32-unknown-unknown
//! The build needs Rust 1.91 or newer. A release archive bundles wasi-sdk's runtime; the source
//! build needs wasi-sdk (`WASI_SDK_PATH` set to its install; 34.0 is tested) and Ninja on `PATH`.
//! On this target the module carries its own C++ runtime, which adds these rules:
//! - `JPH_Init` here is a Rust function that runs the module's C++ static constructors once and
//!   then calls joltc's `JPH_Init`. Call it before every other joltc function except
//!   `JPH_SetAssertFailureHandler`, `JPH_SetTraceHandler` and the `*_SetProcs` setters. Without
//!   it, wasm-ld runs the constructors again at the start of every export call, which resets
//!   Jolt's dynamically initialised statics, such as its fixed-to-world body, while a world uses
//!   them. The repository's Node check (`scripts/check_wasm_raw.sh`) keeps a world across export
//!   calls and a repeated `JPH_Init`, and checks that the fixed-to-world body is constructed and
//!   keeps a mark written into it, and that joltc's step listener still finds the world.
//! - No other code in the module may call `__wasm_call_ctors`: the constructors would run a second
//!   time and reset the same statics. C code that calls joltc's `JPH_Init` symbol directly bypasses
//!   the wrapper and is unsupported.
//! - Static destructors never run, so destroy every joltc object explicitly.
//! - wasi-libc's imports are defined by this crate, so a linked module imports nothing. The twelve
//!   functions are exports of every module that links the crate:
//!
//!   | WASI function | Behaviour |
//!   |---|---|
//!   | `fd_write` | fd 1 and 2: bytes are discarded and reported written (`EINVAL` if the lengths overflow); other fds: `EBADF` |
//!   | `fd_close` | `EBADF` |
//!   | `fd_fdstat_get` | `EBADF` (standard output is not a terminal) |
//!   | `fd_seek` | `ESPIPE` |
//!   | `sched_yield` | succeeds and does nothing |
//!   | `clock_time_get` | traps: the simulation must not read a clock |
//!   | `poll_oneoff` | traps: only sleeps reach it, and nothing can wake the module |
//!   | `environ_sizes_get` | writes 0 and 0: the environment is empty |
//!   | `environ_get` | succeeds and writes nothing |
//!   | `fd_prestat_get` | `EBADF`: no directory is preopened |
//!   | `fd_prestat_dir_name` | `EBADF` |
//!   | `proc_exit` | traps: there is no process to exit |
//! - Rust's allocator and wasi-libc's `malloc` both grow linear memory, and each owns its own
//!   allocations: free memory with the allocator that gave it.
//! - Panics abort on this target; they do not unwind.
//!
//! [Jolt Physics]: https://github.com/jrouwe/JoltPhysics
//! [joltc]: https://github.com/amerkoleci/joltc

mod generated;
mod layout;

pub use generated::*;

#[cfg(all(target_arch = "wasm32", target_os = "unknown"))]
mod static_constructors;
#[cfg(all(target_arch = "wasm32", target_os = "unknown"))]
mod wasi_imports;

#[cfg(all(target_arch = "wasm32", target_os = "unknown"))]
pub use static_constructors::JPH_Init;

/// Scalar type of world positions: `f64` with the `double-precision` feature, `f32` otherwise.
#[cfg(feature = "double-precision")]
pub type Real = f64;

/// Scalar type of world positions: `f64` with the `double-precision` feature, `f32` otherwise.
#[cfg(not(feature = "double-precision"))]
pub type Real = f32;

/// Whether the native library was built with Jolt's assertions (the `asserts` feature).
pub const ASSERTS_ENABLED: bool = cfg!(feature = "asserts");

/// Whether the native library was built with Jolt's cross-platform determinism (the
/// `cross-platform-deterministic` feature).
pub const CROSS_PLATFORM_DETERMINISTIC_ENABLED: bool =
    cfg!(feature = "cross-platform-deterministic");

/// Jolt Physics commit the native library is built from (the `vendor/JoltPhysics` submodule).
pub const JOLT_COMMIT: &str = env!("OXIJOLT_SYS_JOLT_COMMIT");

/// joltc commit the native library is built from (the `vendor/joltc` submodule).
pub const JOLTC_COMMIT: &str = env!("OXIJOLT_SYS_JOLTC_COMMIT");

/// Revision of this fork's joltc additions (`native/joltc_ext/`) in the native library. A
/// prebuilt prefix of another revision is refused by the build script.
pub const JOLTC_EXT_REVISION: &str = env!("OXIJOLT_SYS_JOLTC_EXT_REVISION");

/// `JPH_Mat4_RotationTranslation` under the name double precision uses.
///
/// joltc declares the `JPH_RMat4_*` functions only for double precision; in single precision
/// `JPH_RMat4` is `JPH_Mat4` and `JPH_RVec3` is `JPH_Vec3`, so this forwards to the `JPH_Mat4_*`
/// function and callers name one function in both builds.
///
/// # Safety
/// As for the joltc function: `result` is valid for writing one matrix, `rotation` and
/// `translation` are valid for reading.
#[cfg(not(feature = "double-precision"))]
#[allow(non_snake_case)]
pub unsafe fn JPH_RMat4_RotationTranslation(
    result: *mut JPH_RMat4,
    rotation: *const JPH_Quat,
    translation: *const JPH_RVec3,
) {
    // SAFETY: the caller upholds the joltc function's contract, and the types are identical in
    // single precision.
    unsafe { JPH_Mat4_RotationTranslation(result, rotation, translation) }
}
