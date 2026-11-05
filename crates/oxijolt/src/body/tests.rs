use super::access::{bounds_overlap, corners, outward_box};
use super::load::point_torque;
use super::*;
use crate::world::ensure_initialized;
use crate::PhysicsWorld;
use crate::Real;

#[test]
fn inertia_rule_texts_are_single_spaced() {
    for text in [INERTIA_RULE, crate::character::INNER_BODY_INERTIA_RULE] {
        assert!(!text.contains("  "), "{text}");
    }
}

#[test]
fn default_settings_match_jolt() {
    assert!(ensure_initialized());
    // SAFETY: Jolt is initialised, and the handle takes over the new settings.
    let jolt =
        CreationSettings(unsafe { Owned::from_raw(JPH_BodyCreationSettings_Create()) }.unwrap());
    let ours = BodySettings::default();
    let ptr = jolt.0.as_ptr();
    // SAFETY: `ptr` is the live settings object owned by `jolt`; getters only read it.
    unsafe {
        assert_eq!(JPH_BodyCreationSettings_GetFriction(ptr), ours.friction);
        assert_eq!(
            JPH_BodyCreationSettings_GetRestitution(ptr),
            ours.restitution
        );
        assert_eq!(
            JPH_BodyCreationSettings_GetGravityFactor(ptr),
            ours.gravity_factor
        );
        assert_eq!(
            JPH_BodyCreationSettings_GetAllowSleeping(ptr),
            ours.allow_sleeping
        );
        assert_eq!(
            JPH_BodyCreationSettings_GetMotionQuality(ptr),
            ours.motion_quality.to_jph()
        );
        assert_eq!(
            MotionType::from_jph(JPH_BodyCreationSettings_GetMotionType(ptr)),
            ours.motion_type
        );
        assert_eq!(
            JPH_BodyCreationSettings_GetOverrideMassProperties(ptr),
            JPH_OverrideMassProperties_CalculateMassAndInertia
        );
        assert_eq!(
            JPH_BodyCreationSettings_GetEnhancedInternalEdgeRemoval(ptr),
            ours.enhanced_internal_edge_removal
        );
        assert_eq!(
            JPH_BodyCreationSettings_GetLinearDamping(ptr),
            ours.linear_damping
        );
        assert_eq!(
            JPH_BodyCreationSettings_GetAngularDamping(ptr),
            ours.angular_damping
        );
        assert_eq!(
            AllowedDofs::from_jph(JPH_BodyCreationSettings_GetAllowedDOFs(ptr)),
            ours.allowed_dofs
        );
        assert_eq!(JPH_BodyCreationSettings_GetIsSensor(ptr), ours.sensor);
        assert_eq!(JPH_BodyCreationSettings_GetUserData(ptr), ours.user_data);
        assert!(ours.mass.is_none());
        // The one documented difference: Jolt's default layer is 0.
        assert_eq!(JPH_BodyCreationSettings_GetObjectLayer(ptr), 0);
    }
    assert_eq!(ours.object_layer, ObjectLayer::MOVING);
}

#[test]
fn velocity_limits_are_jolts_default_maxima() {
    assert!(ensure_initialized());
    let shape = Shape::new_sphere(0.5).unwrap();
    let creation = CreationSettings::new(&shape, &BodySettings::default()).unwrap();
    // SAFETY: the settings are live and owned by `creation`; the getters only read them.
    let (linear, angular) = unsafe {
        (
            JPH_BodyCreationSettings_GetMaxLinearVelocity(creation.as_ptr()),
            JPH_BodyCreationSettings_GetMaxAngularVelocity(creation.as_ptr()),
        )
    };
    assert_eq!(linear.to_bits(), limits::MAX_LINEAR_VELOCITY.to_bits());
    assert_eq!(angular.to_bits(), limits::MAX_ANGULAR_VELOCITY.to_bits());
}

#[test]
fn point_torque_rejects_overflowing_products_even_when_they_cancel() {
    let center = RVec3::ZERO;
    let torque = point_torque(Vec3::new(0.0, 2.0, 0.0), RVec3::new(3.0, 0.0, 0.0), center);
    assert_eq!(torque.map(|t| t.exact), Ok([0.0, 0.0, 6.0]));
    // Lever and force are parallel: the cross product is zero in f64, but Jolt's f32
    // products `lever_x * force_y` and `lever_y * force_x` are infinite.
    let parallel = point_torque(
        Vec3::new(1.0e20, 1.0e20, 0.0),
        RVec3::new(1.0e20, 1.0e20, 0.0),
        center,
    );
    assert!(matches!(parallel, Err(BodyError::InvalidValue(_))));
    let far = RVec3::new(Real::MAX, 0.0, 0.0);
    let lever_overflows = point_torque(
        Vec3::new(0.0, 1.0, 0.0),
        far,
        RVec3::new(-Real::MAX, 0.0, 0.0),
    );
    assert!(matches!(lever_overflows, Err(BodyError::InvalidValue(_))));
}

/// One way to evaluate `a * b - c * d` in `f32`.
type DifferenceOfProducts = fn(f32, f32, f32, f32) -> f32;

/// The three ways Jolt's `f32` cross product component `a * b - c * d` can be evaluated: both
/// products rounded, or either product fused into the subtraction.
fn f32_cross_components(lever: [f32; 3], force: [f32; 3]) -> [[f32; 3]; 3] {
    let orders: [DifferenceOfProducts; 3] = [
        |a, b, c, d| a * b - c * d,
        |a, b, c, d| a.mul_add(b, -(c * d)),
        |a, b, c, d| (-c).mul_add(d, a * b),
    ];
    orders.map(|order| {
        std::array::from_fn(|i| {
            let (a, b) = ((i + 1) % 3, (i + 2) % 3);
            order(lever[a], force[b], lever[b], force[a])
        })
    })
}

#[test]
fn point_torque_rounding_covers_every_f32_evaluation_order() {
    let mut state: u64 = 0x2545_F491_4F6C_DD1D;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        // A value with a random mantissa and sign between about 1e-6 and 1e6.
        let mantissa = (state >> 40) as f32 / (1u64 << 24) as f32 + 0.5;
        let exponent = ((state >> 8) % 40) as i32 - 20;
        let sign = if state & 1 == 0 { 1.0 } else { -1.0 };
        sign * mantissa * 2f32.powi(exponent)
    };
    let mut fused_rounding_seen = false;
    for round in 0..20_000 {
        let lever = [next(), next(), next()];
        // Every fourth force is the lever scaled by a power of two: exactly parallel, so that
        // the exact torque is 0.
        let force = if round % 4 == 0 {
            let k = 2f32.powi(round % 16 - 8);
            lever.map(|c| c * k)
        } else {
            [next(), next(), next()]
        };
        // `Real` is `f32` without the `double-precision` feature.
        #[allow(clippy::unnecessary_cast)]
        let point = RVec3::new(lever[0] as Real, lever[1] as Real, lever[2] as Real);
        let torque = point_torque(Vec3::from(force), point, RVec3::ZERO).unwrap();
        for jolt in f32_cross_components(lever, force) {
            for (i, &component) in jolt.iter().enumerate() {
                let error = (f64::from(component) - torque.exact[i]).abs();
                assert!(
                    error <= torque.rounding[i],
                    "{lever:?} x {force:?}, component {i}: {error} > {}",
                    torque.rounding[i]
                );
                fused_rounding_seen |= torque.exact[i] == 0.0 && component != 0.0;
            }
        }
    }
    // Without the rounding bound a zero exact torque would not cover Jolt's fused result.
    assert!(fused_rounding_seen);
}

#[test]
fn enhanced_internal_edge_removal_reaches_the_body() {
    let mut world = PhysicsWorld::new(crate::WorldSettings::default()).unwrap();
    let shape = Shape::new_sphere(0.5).unwrap();
    for value in [true, false] {
        let id = world
            .create_body(
                &shape,
                &BodySettings::new_dynamic().enhanced_internal_edge_removal(value),
            )
            .unwrap();
        let stored = with_locked_body(world.body_lock_interface, id, |body| {
            // SAFETY: `body` is locked for the duration of the closure; the getter only
            // reads it.
            unsafe { JPH_Body_GetEnhancedInternalEdgeRemoval(body.as_ptr()) }
        });
        assert_eq!(stored, Some(value));
    }
}

#[test]
fn damping_reaches_the_body() {
    let mut world = PhysicsWorld::new(crate::WorldSettings::default()).unwrap();
    let shape = Shape::new_sphere(0.5).unwrap();
    let settings = BodySettings::new_dynamic()
        .linear_damping(0.3)
        .angular_damping(0.7);
    let id = world.create_body(&shape, &settings).unwrap();
    let stored = with_read_locked_body(world.body_lock_interface, id, |body| {
        // SAFETY: `body` is locked for the duration of the closure and dynamic, so it has
        // motion properties; the getters only read.
        unsafe {
            let motion = JPH_Body_GetMotionProperties(body.as_ptr());
            (
                JPH_MotionProperties_GetLinearDamping(motion),
                JPH_MotionProperties_GetAngularDamping(motion),
            )
        }
    });
    assert_eq!(stored, Some((0.3, 0.7)));
}

#[test]
fn invalid_damping_is_rejected() {
    let mut world = PhysicsWorld::new(crate::WorldSettings::default()).unwrap();
    let shape = Shape::new_sphere(0.5).unwrap();
    for value in [-0.1, f32::NAN, f32::INFINITY] {
        for settings in [
            BodySettings::new_dynamic().linear_damping(value),
            BodySettings::new_dynamic().angular_damping(value),
        ] {
            assert!(matches!(
                world.create_body(&shape, &settings),
                Err(BodyError::InvalidValue(_))
            ));
        }
    }
    assert_eq!(world.body_count(), 0);
}

#[test]
fn mass_is_reported_for_dynamic_bodies_only() {
    let mut world = PhysicsWorld::new(crate::WorldSettings::default()).unwrap();
    let shape = Shape::new_sphere(0.5).unwrap();
    let dynamic = world
        .create_body(&shape, &BodySettings::new_dynamic().mass(12.5))
        .unwrap();
    let mass = world.body(dynamic).unwrap().mass().unwrap();
    assert!((mass - 12.5).abs() <= 12.5 * 1.0e-6, "{mass}");
    for settings in [BodySettings::new_static(), BodySettings::new_kinematic()] {
        let id = world.create_body(&shape, &settings).unwrap();
        assert_eq!(world.body(id).unwrap().mass(), None);
    }
}

/// Mass 1 with the inertia `R * diag(moments) * R^T`, `R` a rotation of 30 degrees about Z.
fn rotated_inertia(moments: [f32; 3]) -> JPH_MassProperties {
    let (sin, cos) = 30.0_f32.to_radians().sin_cos();
    let rotation = [[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]];
    let mut properties = JPH_MassProperties {
        mass: 1.0,
        ..ZERO_MASS_PROPERTIES
    };
    for (column, out) in properties.inertia.column.iter_mut().take(3).enumerate() {
        let entry = |row: usize| -> f32 {
            (0..3)
                .map(|k| rotation[row][k] * moments[k] * rotation[column][k])
                .sum()
        };
        *out = JPH_Vec4 {
            x: entry(0),
            y: entry(1),
            z: entry(2),
            w: 0.0,
        };
    }
    properties
}

#[test]
fn rotated_inertia_is_accepted() {
    let properties = rotated_inertia([1.0, 2.0, 3.0]);
    assert_ne!(
        properties.inertia.column[0].y, 0.0,
        "the tensor is not diagonal"
    );
    assert!(has_finite_inverse(&properties));
}

#[test]
fn non_finite_inertia_is_rejected() {
    let mut properties = rotated_inertia([1.0, 2.0, 3.0]);
    properties.inertia.column[1].x = f32::NAN;
    assert!(!has_finite_inverse(&properties));
}

#[test]
fn ill_conditioned_rotated_inertia_is_rejected() {
    assert!(!has_finite_inverse(&rotated_inertia([1.0, 1.0e-9, 1.0])));
}

fn aabox(min: [f32; 3], max: [f32; 3]) -> JPH_AABox {
    JPH_AABox {
        min: Vec3::from(min).to_jph(),
        max: Vec3::from(max).to_jph(),
    }
}

/// Whether `a` and `b` overlap, `a` taken in caller precision.
fn overlap(a: &JPH_AABox, b: &JPH_AABox) -> bool {
    let (min, max) = corners(a);
    bounds_overlap(min, max, b)
}

#[test]
fn bounds_overlap_counts_touching_and_containment() {
    let unit = aabox([0.0; 3], [1.0; 3]);
    assert!(overlap(&unit, &aabox([0.5; 3], [2.0; 3])));
    assert!(overlap(&unit, &aabox([1.0, 0.0, 0.0], [2.0, 1.0, 1.0])));
    assert!(overlap(&unit, &aabox([0.25; 3], [0.75; 3])));
    assert!(overlap(&aabox([0.25; 3], [0.75; 3]), &unit));
}

#[test]
fn bounds_separated_on_any_axis_do_not_overlap() {
    let unit = aabox([0.0; 3], [1.0; 3]);
    for axis in 0..3 {
        let mut min = [0.0; 3];
        let mut max = [1.0; 3];
        min[axis] = 1.5;
        max[axis] = 2.5;
        let apart = aabox(min, max);
        assert!(!overlap(&unit, &apart), "axis {axis}");
        assert!(!overlap(&apart, &unit), "axis {axis}");
    }
}

#[test]
fn the_broad_phase_box_encloses_the_callers_box() {
    // 0.1 and 0.3 have no exact `f32`; with `Real = f64` the nearest `f32` lies on one side.
    let min = RVec3::new(0.1, -0.1, 0.3);
    let max = RVec3::new(0.3, 0.1, 0.7);
    let outer = outward_box(min, max);
    let (outer_min, outer_max) = corners(&outer);
    for (inner, outer) in [
        (min.x, outer_min.x),
        (min.y, outer_min.y),
        (min.z, outer_min.z),
    ] {
        assert!(outer <= inner, "{outer} {inner}");
    }
    for (inner, outer) in [
        (max.x, outer_max.x),
        (max.y, outer_max.y),
        (max.z, outer_max.z),
    ] {
        assert!(outer >= inner, "{outer} {inner}");
    }
}

#[test]
fn tiny_rotated_inertia_uses_the_unit_sphere() {
    assert!(has_finite_inverse(&rotated_inertia([
        1.0e-7, 2.0e-7, 3.0e-7
    ])));
}

#[test]
fn tiny_ill_conditioned_rotated_inertia_is_rejected() {
    // Jolt decomposes it before it falls back to the unit sphere, and that decomposition
    // asserts like a large one.
    let properties = rotated_inertia([1.0e-7, 1.0e-12, 1.0e-7]);
    assert_ne!(properties.inertia.column[0].y, 0.0);
    assert!(!has_finite_inverse(&properties));
}

/// Mass 1 with the diagonal inertia `moments`.
fn diagonal_inertia(moments: [f32; 3]) -> JPH_MassProperties {
    let mut properties = JPH_MassProperties {
        mass: 1.0,
        ..ZERO_MASS_PROPERTIES
    };
    for (axis, column) in properties.inertia.column.iter_mut().take(3).enumerate() {
        let mut entries = [0.0; 3];
        entries[axis] = moments[axis];
        *column = JPH_Vec4 {
            x: entries[0],
            y: entries[1],
            z: entries[2],
            w: 0.0,
        };
    }
    properties
}

#[test]
fn diagonal_inertia_must_be_positive_unless_near_zero() {
    assert!(has_finite_inverse(&diagonal_inertia([1.0, 2.0, 3.0])));
    assert!(!has_finite_inverse(&diagonal_inertia([-1.0, 2.0, 3.0])));
    assert!(!has_finite_inverse(&diagonal_inertia([0.0, 1.0, 1.0])));
    // All zero: Jolt uses the inertia of a unit sphere.
    assert!(has_finite_inverse(&diagonal_inertia([0.0, 0.0, 0.0])));
}

#[test]
fn allowed_dofs_match_jolts_flags() {
    for (ours, jolt) in [
        (AllowedDofs::ALL, JPH_AllowedDOFs_All),
        (AllowedDofs::TRANSLATION_X, JPH_AllowedDOFs_TranslationX),
        (AllowedDofs::TRANSLATION_Y, JPH_AllowedDOFs_TranslationY),
        (AllowedDofs::TRANSLATION_Z, JPH_AllowedDOFs_TranslationZ),
        (AllowedDofs::ROTATION_X, JPH_AllowedDOFs_RotationX),
        (AllowedDofs::ROTATION_Y, JPH_AllowedDOFs_RotationY),
        (AllowedDofs::ROTATION_Z, JPH_AllowedDOFs_RotationZ),
        (AllowedDofs::PLANE_2D, JPH_AllowedDOFs_Plane2D),
    ] {
        assert_eq!(ours.to_jph(), jolt);
        assert_eq!(AllowedDofs::from_jph(jolt), ours);
    }
    let plane = AllowedDofs::TRANSLATION_X | AllowedDofs::TRANSLATION_Y | AllowedDofs::ROTATION_Z;
    assert_eq!(plane, AllowedDofs::PLANE_2D);
    assert!(AllowedDofs::ALL.contains(plane));
    assert!(!plane.contains(AllowedDofs::TRANSLATION_Z));
    assert_eq!(AllowedDofs::default(), AllowedDofs::ALL);
}

#[test]
fn dofs_without_translation_are_refused() {
    let rotation_only = AllowedDofs::ROTATION_X | AllowedDofs::ROTATION_Y | AllowedDofs::ROTATION_Z;
    let settings = BodySettings::new_dynamic().allowed_dofs(rotation_only);
    assert_eq!(
        settings.validate_values(),
        Err(BodyError::InvalidValue(DOFS_RULE))
    );
    for keeps_one in [
        AllowedDofs::TRANSLATION_X,
        AllowedDofs::TRANSLATION_Y | AllowedDofs::ROTATION_Y,
        AllowedDofs::TRANSLATION_Z | rotation_only,
    ] {
        let settings = BodySettings::new_dynamic().allowed_dofs(keeps_one);
        assert_eq!(settings.validate_values(), Ok(()));
    }
}

/// A unit quaternion turning `angle` radians about the normalized `axis`.
fn rotation_about(axis: [f32; 3], angle: f32) -> Quat {
    let length = axis.iter().map(|c| c * c).sum::<f32>().sqrt();
    let (sin, cos) = (0.5 * angle).sin_cos();
    let [x, y, z] = axis.map(|c| c / length * sin);
    Quat::from_xyzw(x, y, z, cos)
}

/// The rotation matrix of `q` in rows, in `f64`.
fn rotation_matrix(q: Quat) -> [[f64; 3]; 3] {
    let [x, y, z, w] = [q.x, q.y, q.z, q.w].map(f64::from);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

/// The inverse of `m` by cofactors, in `f64`.
fn inverse(m: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let cofactor = |i: usize, j: usize| {
        let (r0, r1) = ((i + 1) % 3, (i + 2) % 3);
        let (c0, c1) = ((j + 1) % 3, (j + 2) % 3);
        m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0]
    };
    let det: f64 = (0..3).map(|j| m[0][j] * cofactor(0, j)).sum();
    std::array::from_fn(|i| std::array::from_fn(|j| cofactor(j, i) / det))
}

/// `r · m · rᵀ` in `f64`.
fn rotate_tensor(r: [[f64; 3]; 3], m: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            (0..3)
                .flat_map(|k| (0..3).map(move |l| (k, l)))
                .map(|(k, l)| r[i][k] * m[k][l] * r[j][l])
                .sum()
        })
    })
}

/// The world inverse inertia (in rows) and the inverse mass Jolt keeps for the dynamic `id`.
fn inverse_mass_properties(world: &PhysicsWorld, id: BodyId) -> ([[f64; 3]; 3], f32) {
    with_read_locked_body(world.body_lock_interface, id, |body| {
        let mut matrix = ZERO_MASS_PROPERTIES.inertia;
        // SAFETY: `body` is locked for reading for the duration of the closure and dynamic, so it
        // has motion properties; the getters only read it and write the live local.
        let inverse_mass = unsafe {
            JPH_Body_GetInverseInertia(body.as_ptr(), &mut matrix);
            JPH_MotionProperties_GetInverseMassUnchecked(JPH_Body_GetMotionProperties(
                body.as_ptr(),
            ))
        };
        let [x, y, z, _] = matrix.column;
        let rows = [[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]];
        (rows.map(|row| row.map(f64::from)), inverse_mass)
    })
    .unwrap()
}

/// Checks that bodies of a 2 x 0.5 x 1 m box given `inertia` and 3 kg, unrotated and rotated,
/// get the world inverse inertia of the provided tensor within `tolerance` of its Frobenius norm,
/// and a different one than the same box given only the mass.
fn check_provided_inertia_reaches_jolt(inertia: Inertia, tolerance: f64) {
    let mut world = PhysicsWorld::new(crate::WorldSettings::default()).unwrap();
    let shape = Shape::new_box(Vec3::new(1.0, 0.25, 0.5)).unwrap();
    let tensor = inertia.tensor().map(|row| row.map(f64::from));
    for rotation in [Quat::IDENTITY, rotation_about([0.3, -0.5, 0.8], 0.7)] {
        let provided = BodySettings::new_dynamic()
            .rotation(rotation)
            .mass_and_inertia(3.0, inertia);
        let id = world.create_body(&shape, &provided).unwrap();
        let (actual, inverse_mass) = inverse_mass_properties(&world, id);
        let r = rotation_matrix(rotation);
        let expected = rotate_tensor(r, inverse(tensor));
        let norm = expected.iter().flatten().map(|v| v * v).sum::<f64>().sqrt();
        for (a, e) in actual.iter().flatten().zip(expected.iter().flatten()) {
            assert!(
                (a - e).abs() <= tolerance * norm,
                "{actual:?} vs {expected:?}"
            );
        }
        assert_eq!(inverse_mass, 1.0 / 3.0_f32);

        let scaled = BodySettings::new_dynamic().rotation(rotation).mass(3.0);
        let id = world.create_body(&shape, &scaled).unwrap();
        let (from_shape, _) = inverse_mass_properties(&world, id);
        let difference = from_shape
            .iter()
            .flatten()
            .zip(expected.iter().flatten())
            .map(|(a, e)| (a - e).abs())
            .fold(0.0, f64::max);
        assert!(difference > 0.1 * norm, "the shape's inertia was used");
    }
}

#[test]
fn provided_principal_inertia_reaches_the_motion_properties() {
    check_provided_inertia_reaches_jolt(Inertia::Principal(Vec3::new(1.0, 2.0, 4.0)), 1.0e-6);
}

#[test]
fn provided_tensor_inertia_reaches_the_motion_properties() {
    let tensor = [[2.0, 0.5, 0.0], [0.5, 3.0, 0.0], [0.0, 0.0, 4.0]];
    check_provided_inertia_reaches_jolt(Inertia::Tensor(tensor), 1.0e-5);
}

/// The rigid bodies in Jolt's list of awake bodies, in the list's own order.
fn active_list(world: &PhysicsWorld) -> Vec<JPH_BodyID> {
    let system = world.system.as_ptr();
    // SAFETY: the system is this live world's and nothing changes it under the shared borrow;
    // the count is read first, so joltc fills exactly `ids`.
    unsafe {
        let count = JPH_PhysicsSystem_GetNumActiveBodies(system, JPH_BodyType_Rigid);
        let mut ids = vec![0; count as usize];
        JPH_PhysicsSystem_GetActiveBodies(system, JPH_BodyType_Rigid, ids.as_mut_ptr(), count);
        ids
    }
}

/// Six awake spheres, then the spheres at `order` deactivated in one batch or one by one; Jolt's
/// list of awake bodies afterwards.
fn active_list_after_deactivating(order: &[usize], batched: bool) -> Vec<JPH_BodyID> {
    let mut world = PhysicsWorld::new(crate::WorldSettings::default()).unwrap();
    let shape = Shape::new_sphere(0.5).unwrap();
    let ids: Vec<BodyId> = (0..6)
        .map(|i| {
            let at = RVec3::new(2.0 * i as Real, 0.0, 0.0);
            let settings = BodySettings::new_dynamic().position(at);
            world.create_body(&shape, &settings).unwrap()
        })
        .collect();
    let raw: Vec<JPH_BodyID> = ids.iter().map(|id| id.raw).collect();
    assert_eq!(active_list(&world), raw);
    let chosen: Vec<BodyId> = order.iter().map(|&i| ids[i]).collect();
    if batched {
        world.deactivate_bodies(&chosen).unwrap();
    } else {
        for &id in &chosen {
            world.body_mut(id).unwrap().deactivate().unwrap();
        }
    }
    active_list(&world)
}

#[test]
fn deactivate_bodies_leaves_the_active_list_of_one_by_one_deactivation() {
    let given = active_list_after_deactivating(&[1, 4, 0], true);
    assert_eq!(given, active_list_after_deactivating(&[1, 4, 0], false));
    // Jolt moves the last awake body into each freed slot, so the order is state.
    let ascending = active_list_after_deactivating(&[0, 1, 4], true);
    assert_eq!(ascending, active_list_after_deactivating(&[0, 1, 4], false));
    assert_ne!(given, ascending);
}
