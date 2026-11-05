use super::*;

#[test]
fn many_shapes_are_created_and_released() {
    for i in 0..1000 {
        let size = 0.1 + i as f32 * 0.001;
        drop(Shape::new_box(Vec3::new(size, size, size)).unwrap());
        drop(Shape::new_sphere(size).unwrap());
        drop(Shape::new_cylinder(size, size).unwrap());
        let capsule = Shape::new_capsule(size, size).unwrap();
        drop(Shape::new_offset_center_of_mass(&capsule, Vec3::new(0.0, -size, 0.0)).unwrap());
    }
}

#[test]
fn offset_center_of_mass_moves_only_the_center() {
    let offset = Vec3::new(0.1, -0.3, 0.25);
    let shape = Shape::new_offset_center_of_mass(&unit_box(), offset).unwrap();
    assert_eq!(shape.sub_type(), JPH_ShapeSubType_OffsetCenterOfMass);
    let bits = |v: Vec3| <[f32; 3]>::from(v).map(f32::to_bits);
    assert_eq!(bits(shape.center_of_mass()), bits(offset));
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert!(matches!(
            Shape::new_offset_center_of_mass(&unit_box(), Vec3::new(0.0, bad, 0.0)),
            Err(ShapeError::InvalidValue(_))
        ));
    }
    let terrain = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let offset_terrain = Shape::new_offset_center_of_mass(&terrain, offset).unwrap();
    assert!(offset_terrain.must_be_static());
}

#[test]
fn invalid_dimensions_are_rejected() {
    let invalid =
        |result: Result<Shape, ShapeError>| matches!(result, Err(ShapeError::InvalidValue(_)));
    for bad in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        assert!(invalid(Shape::new_box(Vec3::new(1.0, bad, 1.0))));
        assert!(invalid(Shape::new_sphere(bad)));
        assert!(invalid(Shape::new_cylinder(bad, 1.0)));
        assert!(invalid(Shape::new_cylinder(1.0, bad)));
        assert!(invalid(Shape::new_capsule(bad, 1.0)));
        assert!(invalid(Shape::new_capsule(1.0, bad)));
    }
    for bad in [-0.1, f32::NAN, f32::INFINITY] {
        assert!(invalid(Shape::new_box_with_convex_radius(
            Vec3::new(1.0, 1.0, 1.0),
            bad
        )));
        assert!(invalid(Shape::new_cylinder_with_convex_radius(
            1.0, 1.0, bad
        )));
    }
    assert!(Shape::new_box_with_convex_radius(Vec3::new(1.0, 1.0, 1.0), 0.0).is_ok());
    assert!(Shape::new_cylinder_with_convex_radius(1.0, 1.0, 0.0).is_ok());
}

#[test]
fn primitive_extents_are_bounded() {
    let invalid =
        |result: Result<Shape, ShapeError>| matches!(result, Err(ShapeError::InvalidValue(_)));
    let bound = limits::MAX_SHAPE_EXTENT;
    let beyond = bound.next_up();
    assert!(Shape::new_box(Vec3::new(1.0, bound, 1.0)).is_ok());
    assert!(invalid(Shape::new_box(Vec3::new(1.0, beyond, 1.0))));
    assert!(Shape::new_sphere(bound).is_ok());
    assert!(invalid(Shape::new_sphere(beyond)));
    assert!(Shape::new_cylinder(bound, bound).is_ok());
    assert!(invalid(Shape::new_cylinder(beyond, 1.0)));
    assert!(invalid(Shape::new_cylinder(1.0, beyond)));
    assert!(Shape::new_capsule(bound - 1.0, 1.0).is_ok());
    let just_beyond = beyond - (bound - 1.0);
    assert!(invalid(Shape::new_capsule(bound - 1.0, just_beyond)));
    assert!(invalid(Shape::new_capsule(1.0, f32::MAX)));
}

#[test]
fn decorated_and_compound_extents_are_bounded() {
    let unit_box = unit_box();
    let bound = limits::MAX_SHAPE_EXTENT;
    // The bounds around the moved centre of mass reach `offset + 0.5`.
    let edge = Vec3::new(0.0, bound - 1.0, 0.0);
    assert!(Shape::new_offset_center_of_mass(&unit_box, edge).is_ok());
    assert!(matches!(
        Shape::new_offset_center_of_mass(&unit_box, Vec3::new(0.0, bound, 0.0)),
        Err(ShapeError::InvalidValue(_))
    ));
    let large = Shape::new_box(Vec3::new(bound, 1.0, 1.0)).unwrap();
    let centred = [child(&large, 0.0, 1), child(&large, 0.0, 2)];
    assert!(Shape::new_compound(&centred).is_ok());
    let beside = [child(&large, 0.0, 1), child(&large, 2.0, 2)];
    assert!(matches!(
        Shape::new_compound(&beside),
        Err(ShapeError::InvalidValue(_))
    ));
    let far = [child(&unit_box, bound.next_up(), 1)];
    assert!(matches!(
        Shape::new_compound(&far),
        Err(ShapeError::InvalidValue(_))
    ));
}

fn box_convex_radius(shape: &Shape) -> f32 {
    assert_eq!(shape.sub_type(), JPH_ShapeSubType_Box);
    // SAFETY: the shape is live and a box (checked above); the getter only reads it.
    unsafe { JPH_BoxShape_GetConvexRadius(shape.as_ptr().cast()) }
}

#[test]
fn box_convex_radius_reaches_jolt() {
    let unit = Vec3::new(1.0, 1.0, 1.0);
    let sharp = Shape::new_box_with_convex_radius(unit, 0.0).unwrap();
    assert_eq!(box_convex_radius(&sharp), 0.0);
    let default = Shape::new_box(unit).unwrap();
    assert_eq!(box_convex_radius(&default), 0.05);
    let clamped = Shape::new_box_with_convex_radius(Vec3::new(0.1, 1.0, 1.0), 0.5).unwrap();
    assert_eq!(box_convex_radius(&clamped), 0.1);
}

#[test]
fn cylinder_and_capsule_dimensions_reach_jolt() {
    let cylinder = Shape::new_cylinder(0.75, 0.3).unwrap();
    let capsule = Shape::new_capsule(0.70845, 0.4).unwrap();
    // SAFETY: both shapes are live and of the type each getter expects (their subtypes are
    // checked in `new_shapes_have_jolt_subtypes`); the getters only read them.
    unsafe {
        assert_eq!(
            JPH_CylinderShape_GetHalfHeight(cylinder.as_ptr().cast()),
            0.75
        );
        assert_eq!(JPH_CylinderShape_GetRadius(cylinder.as_ptr().cast()), 0.3);
        assert_eq!(
            JPH_CapsuleShape_GetHalfHeightOfCylinder(capsule.as_ptr().cast()),
            0.70845
        );
        assert_eq!(JPH_CapsuleShape_GetRadius(capsule.as_ptr().cast()), 0.4);
    }
}

#[test]
fn inflated_grows_spheres_and_capsules_only() {
    let sphere = Shape::new_sphere(0.05)
        .unwrap()
        .inflated(0.1)
        .unwrap()
        .unwrap();
    assert_eq!(sphere.sub_type(), JPH_ShapeSubType_Sphere);
    // SAFETY: the shape is live and a sphere; the getter only reads it.
    let radius = unsafe { JPH_SphereShape_GetRadius(sphere.as_ptr().cast()) };
    assert!((radius - 0.15).abs() < 1e-6);

    let capsule = Shape::new_capsule(0.7, 0.4)
        .unwrap()
        .inflated(0.1)
        .unwrap()
        .unwrap();
    // SAFETY: the shape is live and a capsule; the getters only read it.
    unsafe {
        assert_eq!(
            JPH_CapsuleShape_GetHalfHeightOfCylinder(capsule.as_ptr().cast()),
            0.7
        );
        assert!((JPH_CapsuleShape_GetRadius(capsule.as_ptr().cast()) - 0.5).abs() < 1e-6);
    }

    let unit_box = unit_box();
    assert!(unit_box.inflated(0.1).unwrap().is_none());
    // A query shape may grow beyond the extent bound; a non-finite size is still rejected.
    let largest = Shape::new_capsule(limits::MAX_SHAPE_EXTENT - 1.0, 1.0).unwrap();
    assert!(largest.inflated(1000.0).unwrap().is_some());
    assert!(largest.inflated(f32::INFINITY).is_err());
}

#[test]
fn static_only_and_center_of_mass_are_read_from_jolt() {
    assert!(!unit_box().must_be_static());
    let terrain = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default());
    assert!(terrain.unwrap().must_be_static());
    assert_eq!(unit_box().center_of_mass(), Vec3::ZERO);
    let unit_box = unit_box();
    let pair = Shape::new_compound(&[child(&unit_box, 0.0, 1), child(&unit_box, 2.0, 2)]).unwrap();
    assert_eq!(pair.center_of_mass(), Vec3::new(1.0, 0.0, 0.0));
}

#[test]
fn local_bounds_are_about_the_center_of_mass() {
    let v = Vec3::new;
    let half = v(0.5, 1.0, 1.5);
    let rounded = Shape::new_box_with_convex_radius(half, 0.1).unwrap();
    assert_eq!(rounded.local_bounds(), (v(-0.5, -1.0, -1.5), half));
    let sphere = Shape::new_sphere(0.7).unwrap();
    assert_eq!(
        sphere.local_bounds(),
        (v(-0.7, -0.7, -0.7), v(0.7, 0.7, 0.7))
    );

    let offset = Shape::new_offset_center_of_mass(&unit_box(), v(0.25, 0.0, -0.5)).unwrap();
    assert_eq!(
        offset.local_bounds(),
        (v(-0.75, -0.5, 0.0), v(0.25, 0.5, 1.0))
    );

    let box_shape = Shape::new_box(half).unwrap();
    let scaled = Shape::new_scaled(&box_shape, v(2.0, -1.0, 3.0)).unwrap();
    assert_eq!(
        scaled.local_bounds(),
        (v(-1.0, -1.0, -4.5), v(1.0, 1.0, 4.5))
    );

    let unit_box = unit_box();
    let pair = Shape::new_compound(&[child(&unit_box, -1.0, 1), child(&unit_box, 3.0, 2)]).unwrap();
    assert_eq!(pair.center_of_mass(), v(1.0, 0.0, 0.0));
    assert_eq!(pair.local_bounds(), (v(-2.5, -0.5, -0.5), v(2.5, 0.5, 0.5)));

    let holes = Shape::new_height_field(3, &[f32::MAX; 9], &HeightFieldSettings::default());
    let center_of_padded_grid = v(1.5, 0.0, 1.5);
    assert_eq!(
        holes.unwrap().local_bounds(),
        (center_of_padded_grid, center_of_padded_grid)
    );
}

#[test]
fn new_shapes_have_jolt_subtypes() {
    let unit = Vec3::new(1.0, 1.0, 1.0);
    assert_eq!(
        Shape::new_box(unit).unwrap().sub_type(),
        JPH_ShapeSubType_Box
    );
    assert_eq!(
        Shape::new_cylinder(1.0, 0.5).unwrap().sub_type(),
        JPH_ShapeSubType_Cylinder
    );
    assert_eq!(
        Shape::new_capsule(1.0, 0.5).unwrap().sub_type(),
        JPH_ShapeSubType_Capsule
    );
    assert_eq!(
        Shape::new_sphere(1.0).unwrap().sub_type(),
        JPH_ShapeSubType_Sphere
    );
}

#[test]
fn height_field_settings_reach_jolt() {
    let settings = HeightFieldSettings::default()
        .block_size(4)
        .bits_per_sample(12)
        .active_edge_cos_threshold_angle(0.5);
    assert_ne!(settings, HeightFieldSettings::default());
    let samples = [0.0; 81];
    assert_eq!(settings.validate_layout(9), Ok(12));
    assert!(ensure_initialized());
    // SAFETY: Jolt is initialised and the inputs are valid (checked above).
    let jolt_settings = unsafe { height_field_settings(9, &samples, &settings, None) }.unwrap();
    let ptr = jolt_settings.as_ptr();
    // SAFETY: the settings are live and were created as heightfield settings; getters only
    // read them.
    unsafe {
        assert_eq!(JPH_HeightFieldShapeSettings_GetBlockSize(ptr), 4);
        assert_eq!(JPH_HeightFieldShapeSettings_GetBitsPerSample(ptr), 12);
        assert_eq!(
            JPH_HeightFieldShapeSettings_GetActiveEdgeCosThresholdAngle(ptr),
            0.5
        );
    }
    let shape = Shape::new_height_field(9, &samples, &settings).unwrap();
    // SAFETY: the shape is live and a heightfield; the getter only reads it.
    let block_size = unsafe { JPH_HeightFieldShape_GetBlockSize(shape.as_ptr().cast()) };
    assert_eq!(block_size, 4);
}

pub(super) fn unit_box() -> Shape {
    Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap()
}

pub(super) fn child(shape: &Shape, x: f32, user_data: u32) -> CompoundChild<'_> {
    CompoundChild {
        shape,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data,
    }
}

#[test]
fn compound_subtypes_depend_on_the_child_count() {
    let unit_box = unit_box();
    let single = Shape::new_compound(&[child(&unit_box, 1.0, 7)]).unwrap();
    assert_eq!(single.sub_type(), JPH_ShapeSubType_MutableCompound);
    let pair = Shape::new_compound(&[child(&unit_box, 0.0, 1), child(&unit_box, 2.0, 2)]).unwrap();
    assert_eq!(pair.sub_type(), JPH_ShapeSubType_StaticCompound);
}

#[test]
fn out_of_range_sub_shape_ids_are_rejected() {
    let unit_box = unit_box();
    let children = [
        child(&unit_box, 0.0, 10),
        child(&unit_box, 2.0, 11),
        child(&unit_box, 4.0, 12),
    ];
    let compound = Shape::new_compound(&children).unwrap();
    // Three children need two bits; the remaining bits are the path below the child.
    for index in 0..3 {
        let id = SubShapeId::new(0xffff_fffc | index);
        assert_eq!(
            compound.compound_sub_shape(id),
            Some(CompoundSubShape {
                index,
                user_data: 10 + index
            })
        );
    }
    for raw in [3, 7, u32::MAX] {
        assert_eq!(compound.compound_sub_shape(SubShapeId::new(raw)), None);
    }
    assert_eq!(unit_box.compound_sub_shape(SubShapeId::new(0)), None);
}

/// `levels` nested two-child compounds, each holding the previous level and a box, around
/// `innermost`; `None` once a level is refused, with that level's error.
pub(super) fn nested_pairs(innermost: Shape, levels: u32) -> Result<Shape, (u32, ShapeError)> {
    let unit_box = unit_box();
    let mut shape = innermost;
    for level in 1..=levels {
        shape = Shape::new_compound(&[child(&shape, 0.0, level), child(&unit_box, 2.0, 0)])
            .map_err(|error| (level, error))?;
    }
    Ok(shape)
}

/// Jolt's rotated-translated shape of `shape`, which the safe API only makes inside compounds.
pub(super) fn rotated_translated(shape: &Shape, position: Vec3, rotation: Quat) -> Shape {
    let (position, rotation) = (position.to_jph(), rotation.to_jph());
    // SAFETY: Jolt is initialised (`shape` exists); the arguments are live locals and a live
    // shape, of which the decorator takes its own reference. The returned shape holds one
    // reference, which `Shape` takes over.
    unsafe {
        Shape::from_raw(
            JPH_RotatedTranslatedShape_Create(&position, &rotation, shape.as_ptr()).cast(),
        )
    }
    .unwrap()
}

pub(super) fn id_bits(shape: &Shape) -> u32 {
    // SAFETY: the shape is live; the getter only reads it.
    unsafe { JPH_Shape_GetSubShapeIDBitsRecursive(shape.as_ptr()) }
}

#[test]
fn a_one_child_compound_at_bit_32_is_refused() {
    let cube = unit_box();
    let single = || Shape::new_compound(&[child(&cube, 0.0, 7)]).unwrap();
    let refused = Err((32, ShapeError::InvalidValue(compound::SUB_SHAPE_ID_RULE)));

    let at_31 = nested_pairs(single(), 31).unwrap();
    assert_eq!(id_bits(&at_31), 31);
    assert_eq!(nested_pairs(single(), 32).map(|_| ()), refused);

    // Without the one-child compound the same depth is exactly Jolt's 32 bits.
    let plain = nested_pairs(unit_box(), 32).unwrap();
    assert_eq!(id_bits(&plain), 32);

    // Decorators push no index: the one-child compound still starts at bit 32.
    let scaled = Shape::new_scaled(&single(), Vec3::new(2.0, 2.0, 2.0)).unwrap();
    assert_eq!(nested_pairs(scaled, 32).map(|_| ()), refused);
    let offset = Shape::new_offset_center_of_mass(&single(), Vec3::new(0.0, 0.1, 0.0)).unwrap();
    assert_eq!(nested_pairs(offset, 32).map(|_| ()), refused);
    let turned = || rotated_translated(&single(), Vec3::new(0.0, 0.5, 0.0), quarter_turn());
    assert_eq!(id_bits(&nested_pairs(turned(), 31).unwrap()), 31);
    assert_eq!(nested_pairs(turned(), 32).map(|_| ()), refused);
}

/// A quarter turn about +y.
fn quarter_turn() -> Quat {
    let half = std::f32::consts::FRAC_1_SQRT_2;
    Quat::from_xyzw(0.0, half, 0.0, half)
}

/// `depth` levels of two-child compounds over `leaf`, each holding the level below twice: a
/// graph of `depth + 1` distinct shapes with `2^depth` paths to the leaf.
pub(super) fn shared_pairs(leaf: Shape, depth: u32) -> Shape {
    let mut shape = leaf;
    for level in 1..=depth {
        shape =
            Shape::new_compound(&[child(&shape, 0.0, level), child(&shape, 0.0, level)]).unwrap();
    }
    shape
}

/// The ids the builder computes for `shape`, with a fresh memo.
fn walked_ids(shape: &Shape) -> compound::SubShapeIds {
    // SAFETY: the shape is live for the call, and the memo starts empty.
    unsafe { compound::sub_shape_ids(shape.as_ptr(), &mut std::collections::BTreeMap::new()) }
}

#[test]
fn shared_shapes_are_walked_once() {
    use compound::walk_count;
    const DEPTH: u32 = 16;
    let graph = shared_pairs(unit_box(), DEPTH);
    let scaled = Shape::new_scaled(&graph, Vec3::new(2.0, 2.0, 2.0)).unwrap();
    let offset = Shape::new_offset_center_of_mass(&graph, Vec3::new(0.0, 0.1, 0.0)).unwrap();
    let turned = rotated_translated(&graph, Vec3::new(0.0, 0.5, 0.0), quarter_turn());

    walk_count::take();
    let ids = walked_ids(&graph);
    assert_eq!(ids.width, id_bits(&graph));
    // Each distinct shape is visited once; only the box's width is asked of Jolt.
    assert_eq!(walk_count::take(), (DEPTH + 1, 1));

    // Repeated roots and three decorators of the graph add only the decorators.
    let compound = Shape::new_compound(&[
        child(&graph, 0.0, 0),
        child(&graph, 0.0, 1),
        child(&scaled, 0.0, 2),
        child(&offset, 0.0, 3),
        child(&turned, 0.0, 4),
    ])
    .unwrap();
    assert_eq!(walk_count::take(), (DEPTH + 4, 1));
    assert_eq!(walked_ids(&compound).width, id_bits(&compound));
    assert_eq!(id_bits(&compound), DEPTH + 3);
}

#[test]
fn walked_widths_follow_jolt() {
    let cube = unit_box();
    let single = Shape::new_compound(&[child(&cube, 0.0, 0)]).unwrap();
    let (vertices, triangles) = (
        [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ],
        [[0, 2, 1]],
    );
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let shapes = [
        nested_pairs(single, 5).unwrap(),
        height_field_at_32_bits(),
        Shape::new_scaled(&mesh, Vec3::new(1.0, 2.0, 1.0)).unwrap(),
        shared_pairs(height_field_13_bits(), 3),
        rotated_translated(&widened(&mesh, 5, 0), Vec3::ZERO, quarter_turn()),
    ];
    for shape in &shapes {
        assert_eq!(walked_ids(shape).width, id_bits(shape));
    }
}

#[test]
fn sub_shape_id_arithmetic_follows_jolt() {
    use compound::{compound_ids, fits_jolt_ids, index_bits, SubShapeIds};

    let expected = [(1, 0), (2, 1), (3, 2), (4, 2), (5, 3), (64, 6), (65, 7)];
    for (count, bits) in expected {
        assert_eq!(index_bits(count), bits, "{count} children");
    }
    assert_eq!(index_bits(u32::MAX), 32);

    let leaf = SubShapeIds {
        width: 0,
        zero_width_at: None,
        expanded: 1,
    };
    let single = compound_ids([leaf].into_iter(), 1);
    assert_eq!(single.zero_width_at, Some(0));
    // A pair of a one-child compound and a leaf puts the inner push one bit further.
    let pair = compound_ids([single, leaf].into_iter(), 2);
    assert_eq!(
        pair,
        SubShapeIds {
            width: 1,
            zero_width_at: Some(1),
            expanded: 4,
        }
    );
    let at = |bit| SubShapeIds {
        width: bit,
        zero_width_at: Some(bit),
        expanded: 1,
    };
    assert!(fits_jolt_ids(at(31)));
    assert!(!fits_jolt_ids(at(32)));
    assert!(fits_jolt_ids(SubShapeIds {
        width: 32,
        zero_width_at: None,
        expanded: 1,
    }));
    let saturated = SubShapeIds {
        expanded: u32::MAX,
        ..leaf
    };
    assert_eq!(
        compound_ids([saturated, leaf].into_iter(), 2).expanded,
        u32::MAX
    );
}

#[test]
fn compounds_above_the_expansion_bound_are_refused() {
    use compound::walk_count;
    use limits::MAX_EXPANDED_SUB_SHAPES as MAX;

    // 19 shared levels over a box: 2^20 - 1 shapes once every use is counted.
    let graph = shared_pairs(unit_box(), 19);
    assert_eq!(walked_ids(&graph).expanded, MAX - 1);
    let cube = unit_box();
    let decorated = Shape::new_offset_center_of_mass(&graph, Vec3::ZERO).unwrap();
    assert_eq!(walked_ids(&decorated).expanded, MAX);

    // The bound itself: the compound and the graph.
    let at_bound = Shape::new_compound(&[child(&graph, 0.0, 0)]).unwrap();
    assert_eq!(walked_ids(&at_bound).expanded, MAX);
    let refused = |expanded| Err(ShapeError::TooManySubShapes { expanded });
    assert_eq!(
        Shape::new_compound(&[child(&graph, 0.0, 0), child(&cube, 3.0, 1)]).map(|_| ()),
        refused(MAX + 1)
    );
    assert_eq!(
        Shape::new_compound(&[child(&decorated, 0.0, 0)]).map(|_| ()),
        refused(MAX + 1)
    );

    // One more shared level is refused before Jolt walks it.
    walk_count::take();
    assert_eq!(
        Shape::new_compound(&[child(&graph, 0.0, 20), child(&graph, 0.0, 20)]).map(|_| ()),
        refused(2 * MAX - 1)
    );
    assert_eq!(walk_count::take(), (20, 1));
}

/// A compound of `deep` at the origin and `count - 1` boxes in a row beyond it.
pub(super) fn widened(deep: &Shape, count: u32, user_data: u32) -> Shape {
    let unit_box = unit_box();
    let mut children = vec![child(deep, 0.0, user_data)];
    children.extend((1..count).map(|i| child(&unit_box, 20.0 + 1.5 * i as f32, i)));
    Shape::new_compound(&children).unwrap()
}

/// A flat 33 x 33 heightfield around the origin, whose ids use 13 bits.
pub(super) fn height_field_13_bits() -> Shape {
    let settings = HeightFieldSettings::default().offset(Vec3::new(-16.0, 0.0, -16.0));
    let field = Shape::new_height_field(33, &[0.0; 33 * 33], &settings).unwrap();
    // 33 samples are stored as 34: 2 * 6 bits for the cell and 1 for the triangle.
    assert_eq!(id_bits(&field), 13);
    field
}

/// A 33 x 33 heightfield (13 id bits) inside compounds of 64, 64 and 128 children (6, 6 and
/// 7 bits): exactly 32 bits. The heightfield is child 0 at every level, around the origin.
pub(super) fn height_field_at_32_bits() -> Shape {
    let field = height_field_13_bits();
    let inner = widened(&field, 64, 101);
    let middle = widened(&inner, 64, 102);
    widened(&middle, 128, 103)
}

#[test]
fn height_field_ids_reach_exactly_32_bits() {
    let outer = height_field_at_32_bits();
    assert_eq!(id_bits(&outer), 32);

    let mut world = crate::PhysicsWorld::new(crate::WorldSettings::default()).unwrap();
    world
        .create_body(&outer, &crate::BodySettings::new_static())
        .unwrap();
    let ray = crate::RayCast::new(crate::RVec3::new(1.3, 5.0, 2.7), Vec3::new(0.0, -10.0, 0.0));
    let hit = world
        .cast_ray(&ray, &crate::QueryFilter::new())
        .unwrap()
        .expect("the ray hits the heightfield");
    assert_eq!(
        outer.compound_sub_shape(hit.sub_shape_id),
        Some(CompoundSubShape {
            index: 0,
            user_data: 103
        })
    );
}
