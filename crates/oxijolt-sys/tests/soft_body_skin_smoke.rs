//! Smoke tests for the soft body skinning functions of the joltc additions: inverse binds and
//! skinned constraints in the shared settings, and skinning a live soft body to joint matrices.

mod framework;

use framework::*;
use oxijolt_sys::*;

/// Vertices per side of the test cloth.
const SIDE: u32 = 3;
/// Distance between neighbouring cloth vertices in metres.
const SPACING: f32 = 0.5;
/// Joint count of the skinned cloth: inverse binds name joints 0 and 2, joint 1 is unused.
const JOINTS: usize = 3;
/// How far joint 2 moves the vertices bound to it in the hard-skin test.
const SHIFT: f32 = 0.3;

/// Index of the cloth vertex in column `x` and row `z`.
fn vertex_index(x: u32, z: u32) -> u32 {
    z * SIDE + x
}

/// A flat `SIDE` x `SIDE` cloth in the XZ plane, two triangles per grid cell, every vertex with
/// inverse mass 1. The caller owns the returned reference.
fn cloth() -> *mut JPH_SoftBodySharedSettings {
    let mut vertices = Vec::new();
    for z in 0..SIDE {
        for x in 0..SIDE {
            vertices.push(JPH_SoftVertex {
                position: vec3(x as f32 * SPACING, 0.0, z as f32 * SPACING),
                velocity: vec3(0.0, 0.0, 0.0),
                invMass: 1.0,
            });
        }
    }
    let mut faces = Vec::new();
    for z in 0..SIDE - 1 {
        for x in 0..SIDE - 1 {
            let face = |a, b, c| JPH_SoftFace {
                vertex1: a,
                vertex2: b,
                vertex3: c,
                materialIndex: 0,
            };
            let (v00, v10) = (vertex_index(x, z), vertex_index(x + 1, z));
            let (v01, v11) = (vertex_index(x, z + 1), vertex_index(x + 1, z + 1));
            faces.push(face(v00, v01, v11));
            faces.push(face(v00, v11, v10));
        }
    }
    // SAFETY: Jolt is initialised (`init`); both arrays are live for the calls and hold the
    // counts passed. The settings are returned holding the one reference `_Create` added.
    unsafe {
        init();
        let settings = JPH_SoftBodySharedSettings_Create();
        JPH_SoftBodySharedSettings_AddVertices(settings, vertices.as_ptr(), vertices.len() as u32);
        JPH_SoftBodySharedSettings_AddFaces(settings, faces.as_ptr(), faces.len() as u32);
        settings
    }
}

fn vec4(x: f32, y: f32, z: f32, w: f32) -> JPH_Vec4 {
    JPH_Vec4 { x, y, z, w }
}

/// A translation matrix (column major, as `JPH_Mat4` stores it).
fn translation(x: f32, y: f32, z: f32) -> JPH_Mat4 {
    JPH_Mat4 {
        column: [
            vec4(1.0, 0.0, 0.0, 0.0),
            vec4(0.0, 1.0, 0.0, 0.0),
            vec4(0.0, 0.0, 1.0, 0.0),
            vec4(x, y, z, 1.0),
        ],
    }
}

fn weight(inv_bind: u32, weight: f32) -> JPH_SoftBodySkinWeight {
    JPH_SoftBodySkinWeight {
        invBindIndex: inv_bind,
        weight,
    }
}

/// A skinned constraint for `vertex` with `weights` (padded with zero weights) and no back stop.
fn skinned(
    vertex: u32,
    weights: &[JPH_SoftBodySkinWeight],
    max_distance: f32,
) -> JPH_SoftBodySkinnedConstraint {
    let mut all = [weight(0, 0.0); 4];
    all[..weights.len()].copy_from_slice(weights);
    JPH_SoftBodySkinnedConstraint {
        vertex,
        weights: all,
        maxDistance: max_distance,
        backStopDistance: f32::MAX,
        backStopRadius: 40.0,
    }
}

/// Inverse bind and skinned constraint counts.
fn skin_counts(settings: *const JPH_SoftBodySharedSettings) -> [u32; 2] {
    // SAFETY: `settings` is live; the getters only read it.
    unsafe {
        [
            JPH_SoftBodySharedSettings_GetInvBindCount(settings),
            JPH_SoftBodySharedSettings_GetSkinnedConstraintCount(settings),
        ]
    }
}

fn add_skinned(
    settings: *mut JPH_SoftBodySharedSettings,
    constraint: JPH_SoftBodySkinnedConstraint,
) -> bool {
    // SAFETY: `settings` is live and `constraint` a live local the call copies.
    unsafe { JPH_SoftBodySharedSettings_AddSkinnedConstraint(settings, &constraint) }
}

/// The cloth with every vertex hard-skinned: inverse bind 0 binds joint 0 at the origin, inverse
/// bind 1 binds joint 2 at x = 1. Row 0 follows joint 2, vertex (1, 1) is split evenly between
/// both joints and the other vertices follow joint 0. Normals are computed and the settings
/// optimised. The caller owns the returned reference.
fn skinned_cloth() -> *mut JPH_SoftBodySharedSettings {
    let settings = cloth();
    let identity = translation(0.0, 0.0, 0.0);
    let joint_2_bind = translation(-1.0, 0.0, 0.0);
    // SAFETY: `settings` is live; both matrices are live locals the calls copy.
    unsafe {
        JPH_SoftBodySharedSettings_AddInvBind(settings, 0, &identity);
        JPH_SoftBodySharedSettings_AddInvBind(settings, 2, &joint_2_bind);
    }
    for vertex in 0..SIDE * SIDE {
        let weights: &[JPH_SoftBodySkinWeight] = if vertex < SIDE {
            &[weight(1, 1.0)]
        } else if vertex == vertex_index(1, 1) {
            &[weight(0, 0.5), weight(1, 0.5)]
        } else {
            &[weight(0, 1.0)]
        };
        assert!(add_skinned(settings, skinned(vertex, weights, 0.0)));
    }
    // SAFETY: `settings` is live; every vertex has at most 6 all-skinned faces.
    unsafe {
        JPH_SoftBodySharedSettings_CalculateSkinnedConstraintNormals(settings);
        JPH_SoftBodySharedSettings_Optimize(settings);
    }
    settings
}

/// Creates a soft body from `settings` at `position` and adds it, active, to the world, through
/// `CreateSoftBody` and `AddBody`. The creation settings are destroyed before returning; the body
/// keeps its own reference to the shared settings.
fn create_soft_body(
    world: &TestWorld,
    settings: *mut JPH_SoftBodySharedSettings,
    position: JPH_RVec3,
) -> JPH_BodyID {
    let rotation = quat_identity();
    // SAFETY: `settings` is live; the creation settings take their own reference to it and are
    // deleted after the body took its own. `position` and `rotation` are live locals. The body
    // pointer is used only before `AddBody` hands the body to the world.
    let id = unsafe {
        let creation =
            JPH_SoftBodyCreationSettings_Create2(settings, &position, &rotation, OL_MOVING);
        let body = JPH_BodyInterface_CreateSoftBody(world.body_interface(), creation);
        JPH_SoftBodyCreationSettings_Destroy(creation);
        assert!(!body.is_null(), "soft body creation failed");
        let id = JPH_Body_GetID(body);
        JPH_BodyInterface_AddBody(world.body_interface(), id, JPH_Activation_Activate);
        id
    };
    id
}

/// Runs `f` with body `id` locked for writing.
fn with_body<R>(world: &TestWorld, id: JPH_BodyID, f: impl FnOnce(*mut JPH_Body) -> R) -> R {
    // SAFETY: the lock interface belongs to the live system and joltc copies the id. The body
    // exists, so the lock yields it, locked until the lock is destroyed after `f`.
    unsafe {
        let lock_interface = JPH_PhysicsSystem_GetBodyLockInterface(world.system());
        let lock = JPH_BodyLockInterface_LockMultiWrite(lock_interface, &id, 1);
        let body = JPH_BodyLockMultiWrite_GetBody(lock, 0);
        assert!(!body.is_null());
        let result = f(body);
        JPH_BodyLockMultiWrite_Destroy(lock);
        result
    }
}

/// Vertex positions relative to the centre of mass and world velocities of soft body `id`.
fn vertices(world: &TestWorld, id: JPH_BodyID) -> (Vec<JPH_Vec3>, Vec<JPH_Vec3>) {
    with_body(world, id, |body| {
        // SAFETY: `body` is locked for the call; each output holds `count` elements.
        unsafe {
            let count = JPH_Body_GetSoftBodyVertexCount(body) as usize;
            let mut positions = vec![vec3(0.0, 0.0, 0.0); count];
            let mut velocities = vec![vec3(0.0, 0.0, 0.0); count];
            JPH_Body_GetSoftBodyVertexLocalPositions(body, positions.as_mut_ptr(), count as u32);
            JPH_Body_GetSoftBodyVertices(
                body,
                std::ptr::null_mut(),
                velocities.as_mut_ptr(),
                std::ptr::null_mut(),
                count as u32,
            );
            (positions, velocities)
        }
    })
}

fn bits(vectors: &[JPH_Vec3]) -> Vec<[u32; 3]> {
    vectors
        .iter()
        .map(|v| [v.x, v.y, v.z].map(f32::to_bits))
        .collect()
}

/// Joint matrices behind a 4-byte field, so the array is 4-aligned but not 16-aligned, as the C
/// API allows.
#[repr(C)]
struct PaddedJoints {
    _pad: u32,
    joints: [JPH_Mat4; JOINTS],
}

impl PaddedJoints {
    fn new(joints: [JPH_Mat4; JOINTS]) -> Box<Self> {
        let padded = Box::new(Self { _pad: 0, joints });
        assert_ne!(
            padded.joints.as_ptr() as usize % 16,
            0,
            "the joint array must not be 16-aligned"
        );
        padded
    }
}

fn skin(
    world: &TestWorld,
    id: JPH_BodyID,
    joints: *const JPH_Mat4,
    count: u32,
    hard: bool,
) -> bool {
    with_body(world, id, |body| {
        // SAFETY: `body` is locked for writing and no step runs (the test steps on this thread
        // only); `joints` is null or holds at least `count` matrices.
        unsafe { JPH_Body_SkinSoftBodyVertices(body, joints, count, hard) }
    })
}

#[test]
fn inverse_binds_and_skinned_constraints_are_counted() {
    let settings = cloth();
    assert_eq!(skin_counts(settings), [0, 0]);
    let identity = translation(0.0, 0.0, 0.0);
    // SAFETY: `settings` is live; `identity` is a live local the calls copy.
    unsafe {
        JPH_SoftBodySharedSettings_AddInvBind(settings, 0, &identity);
        JPH_SoftBodySharedSettings_AddInvBind(settings, 5, &identity);
    }
    assert!(add_skinned(
        settings,
        skinned(0, &[weight(0, 1.0)], f32::MAX)
    ));
    assert!(add_skinned(
        settings,
        skinned(1, &[weight(0, 0.25), weight(1, 0.75)], 0.1)
    ));
    assert_eq!(skin_counts(settings), [2, 2]);
    // SAFETY: the settings hold the one reference `_Create` added, released here.
    unsafe { JPH_SoftBodySharedSettings_Destroy(settings) };
}

#[test]
fn skinned_constraints_with_bad_indices_are_refused() {
    let settings = cloth();
    let identity = translation(0.0, 0.0, 0.0);
    // SAFETY: `settings` is live; `identity` is a live local the call copies.
    unsafe { JPH_SoftBodySharedSettings_AddInvBind(settings, 0, &identity) };

    assert!(
        !add_skinned(settings, skinned(SIDE * SIDE, &[weight(0, 1.0)], 0.0)),
        "vertex out of range"
    );
    assert!(
        !add_skinned(settings, skinned(0, &[weight(0, 0.5), weight(1, 0.5)], 0.0)),
        "inverse bind out of range"
    );
    assert_eq!(
        skin_counts(settings),
        [1, 0],
        "a refused constraint adds nothing"
    );

    // Jolt stops at the first zero weight, so the index of a later weight is never read.
    let after_zero = skinned(0, &[weight(0, 1.0), weight(0, 0.0), weight(7, 1.0)], 0.0);
    assert!(add_skinned(settings, after_zero));
    assert_eq!(skin_counts(settings), [1, 1]);
    // SAFETY: the settings hold the one reference `_Create` added, released here.
    unsafe { JPH_SoftBodySharedSettings_Destroy(settings) };
}

#[test]
fn skinned_normals_are_calculated_before_optimize() {
    let settings = skinned_cloth();
    assert_eq!(skin_counts(settings), [2, SIDE * SIDE]);
    // SAFETY: the settings hold the one reference `_Create` added, released here.
    unsafe { JPH_SoftBodySharedSettings_Destroy(settings) };
}

#[test]
fn hard_skinning_moves_the_vertices_to_their_joints() {
    let world = TestWorld::new(2);
    let settings = skinned_cloth();
    let id = create_soft_body(&world, settings, rvec3(4.0, 2.0, -1.0));
    // SAFETY: the body keeps its own reference; this releases the test's.
    unsafe { JPH_SoftBodySharedSettings_Destroy(settings) };
    with_body(&world, id, |body| {
        // SAFETY: `body` is locked; the getter only reads it.
        assert_eq!(
            unsafe { JPH_Body_GetSoftBodySkinJointCount(body) },
            JOINTS as u32
        );
    });

    let velocity = vec3(0.5, 1.0, -2.0);
    with_body(&world, id, |body| {
        for vertex in 0..SIDE * SIDE {
            // SAFETY: `body` is locked for writing; `vertex` is a vertex index.
            unsafe { JPH_Body_SetSoftBodyVertexVelocity(body, vertex, &velocity) };
        }
    });
    let (bind, _) = vertices(&world, id);

    // Joint 1 is bound by nothing, so its far translation must not show anywhere.
    let joints = PaddedJoints::new([
        translation(0.0, 0.0, 0.0),
        translation(100.0, 0.0, 0.0),
        translation(1.0 + SHIFT, 0.0, 0.0),
    ]);
    assert!(skin(
        &world,
        id,
        joints.joints.as_ptr(),
        JOINTS as u32,
        true
    ));

    let (skinned, velocities) = vertices(&world, id);
    for vertex in 0..(SIDE * SIDE) as usize {
        let expected_shift = if vertex < SIDE as usize {
            SHIFT
        } else if vertex == vertex_index(1, 1) as usize {
            SHIFT / 2.0
        } else {
            0.0
        };
        let (b, s) = (bind[vertex], skinned[vertex]);
        assert!(
            (s.x - (b.x + expected_shift)).abs() <= 1.0e-6,
            "vertex {vertex}: {} vs {}",
            s.x,
            b.x
        );
        assert_eq!([s.y, s.z], [b.y, b.z], "vertex {vertex}");
        let v = velocities[vertex];
        assert_eq!(
            [v.x, v.y, v.z],
            [0.0; 3],
            "hard skinning zeroes the velocity of vertex {vertex}"
        );
    }
    // SAFETY: the body exists and nothing else removes it.
    unsafe { JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), id) };
}

#[test]
fn refused_skin_calls_change_nothing() {
    let world = TestWorld::new(1);
    let settings = skinned_cloth();
    let id = create_soft_body(&world, settings, rvec3(0.0, 1.0, 0.0));
    let unskinned_settings = cloth();
    let unskinned = create_soft_body(&world, unskinned_settings, rvec3(5.0, 1.0, 0.0));
    let rigid = create_box(
        world.body_interface(),
        vec3(0.5, 0.5, 0.5),
        rvec3(-5.0, 0.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );
    // SAFETY: each body keeps its own reference; this releases the test's.
    unsafe {
        JPH_SoftBodySharedSettings_Destroy(settings);
        JPH_SoftBodySharedSettings_Destroy(unskinned_settings);
    }

    let joints = PaddedJoints::new([translation(3.0, 0.0, 0.0); JOINTS]);
    let before = vertices(&world, id);
    let unskinned_before = vertices(&world, unskinned);
    assert!(
        !skin(&world, id, joints.joints.as_ptr(), JOINTS as u32 - 1, true),
        "short joint count"
    );
    assert!(
        !skin(&world, id, joints.joints.as_ptr(), 0, true),
        "zero joint count"
    );
    assert!(
        !skin(&world, id, std::ptr::null(), JOINTS as u32, true),
        "null joint matrices"
    );
    assert!(
        !skin(
            &world,
            unskinned,
            joints.joints.as_ptr(),
            JOINTS as u32,
            true
        ),
        "no skinned constraints"
    );
    assert!(
        !skin(&world, rigid, joints.joints.as_ptr(), JOINTS as u32, true),
        "rigid body"
    );
    let after = vertices(&world, id);
    assert_eq!(bits(&before.0), bits(&after.0));
    assert_eq!(bits(&before.1), bits(&after.1));
    assert_eq!(
        bits(&unskinned_before.0),
        bits(&vertices(&world, unskinned).0)
    );

    with_body(&world, unskinned, |body| {
        // SAFETY: `body` is locked; the getter only reads it.
        assert_eq!(unsafe { JPH_Body_GetSoftBodySkinJointCount(body) }, 0);
    });
    with_body(&world, rigid, |body| {
        // SAFETY: `body` is locked; the getter only reads it.
        assert_eq!(unsafe { JPH_Body_GetSoftBodySkinJointCount(body) }, 0);
    });
}

#[test]
fn the_skinned_max_distance_multiplier_round_trips() {
    let world = TestWorld::new(1);
    let settings = skinned_cloth();
    let id = create_soft_body(&world, settings, rvec3(0.0, 1.0, 0.0));
    // SAFETY: the body keeps its own reference; this releases the test's.
    unsafe { JPH_SoftBodySharedSettings_Destroy(settings) };
    let rigid = create_box(
        world.body_interface(),
        vec3(0.5, 0.5, 0.5),
        rvec3(-5.0, 0.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );
    with_body(&world, id, |body| {
        // SAFETY: `body` is locked for writing; 0.25 is finite and non-negative.
        unsafe {
            assert_eq!(JPH_Body_GetSoftBodySkinnedMaxDistanceMultiplier(body), 1.0);
            assert!(JPH_Body_SetSoftBodySkinnedMaxDistanceMultiplier(body, 0.25));
            assert_eq!(JPH_Body_GetSoftBodySkinnedMaxDistanceMultiplier(body), 0.25);
        }
    });
    with_body(&world, rigid, |body| {
        // SAFETY: `body` is locked for writing; the calls only read the body type.
        unsafe {
            assert!(!JPH_Body_SetSoftBodySkinnedMaxDistanceMultiplier(
                body, 0.25
            ));
            assert_eq!(JPH_Body_GetSoftBodySkinnedMaxDistanceMultiplier(body), 1.0);
        }
    });
}

#[test]
fn a_skinned_soft_body_skins_and_steps_after_its_settings_are_released() {
    let world = TestWorld::new(2);
    let settings = skinned_cloth();
    let id = create_soft_body(&world, settings, rvec3(0.0, 1.0, 0.0));
    // SAFETY: the body keeps its own reference; this releases the test's, which was the last
    // other one (the creation settings are already gone).
    unsafe { JPH_SoftBodySharedSettings_Destroy(settings) };
    let identity = PaddedJoints::new([translation(0.0, 0.0, 0.0); JOINTS]);
    // The first call hard-skins: a soft skin call moves the previous skin positions, which start
    // as NaN, into the next step.
    for tick in 0..5 {
        assert!(skin(
            &world,
            id,
            identity.joints.as_ptr(),
            JOINTS as u32,
            tick == 0
        ));
        world.step(1.0 / 60.0);
    }
    let (positions, _) = vertices(&world, id);
    assert!(positions
        .iter()
        .all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()));
    // SAFETY: the body exists and nothing else removes it.
    unsafe { JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), id) };
}
