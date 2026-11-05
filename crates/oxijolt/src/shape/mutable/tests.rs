use super::*;
use crate::body::mass_properties;
use crate::limits::MAX_SHAPE_EXTENT;
use crate::shape::compound::walk_count;
use crate::shape::compound::{CHILD_POSITION_RULE, CHILD_ROTATION_RULE};
use crate::shape::tests::{
    height_field_13_bits, height_field_at_32_bits, nested_pairs, shared_pairs, unit_box,
};
use crate::{
    Activation, BodySettings, CompoundSubShape, HeightFieldSettings, PhysicsWorld, SubShapeId,
    WorldSettings,
};

/// Everything a compound's queries and mass depend on, as bits: sub type, centre of mass, local
/// bounds, mass properties, inner radius, id width, and per child its shape, pose (relative to
/// the centre of mass) and user data.
fn fingerprint(shape: &Shape) -> Vec<u64> {
    let ptr = shape.as_ptr();
    let mut bits = vec![shape.sub_type() as u64];
    let mut push = |values: &[f32]| bits.extend(values.iter().map(|v| u64::from(v.to_bits())));
    let center = shape.center_of_mass();
    push(&[center.x, center.y, center.z]);
    let (min, max) = shape.local_bounds();
    // SAFETY: the shape is live; the getters only read it.
    let (radius, width) = unsafe {
        (
            JPH_Shape_GetInnerRadius(ptr),
            JPH_Shape_GetSubShapeIDBitsRecursive(ptr),
        )
    };
    push(&[min.x, min.y, min.z, max.x, max.y, max.z, radius]);
    let properties = mass_properties(shape, None);
    push(&[properties.mass]);
    for column in properties.inertia.column {
        push(&[column.x, column.y, column.z, column.w]);
    }
    bits.push(u64::from(width));
    let compound: *const JPH_CompoundShape = ptr.cast();
    // SAFETY: as above; the callers pass compounds.
    let count = unsafe { JPH_CompoundShape_GetNumSubShapes(compound) };
    for index in 0..count {
        let (mut child, mut user_data) = (null(), 0);
        let mut position = Vec3::ZERO.to_jph();
        let mut rotation = Quat::IDENTITY.to_jph();
        // SAFETY: as above, `index < count`; every output is a live local.
        unsafe {
            JPH_CompoundShape_GetSubShape(
                compound,
                index,
                &mut child,
                &mut position,
                &mut rotation,
                &mut user_data,
            );
        }
        bits.push(child as u64);
        let mut push = |values: &[f32]| bits.extend(values.iter().map(|v| u64::from(v.to_bits())));
        push(&[position.x, position.y, position.z]);
        push(&[rotation.x, rotation.y, rotation.z, rotation.w]);
        bits.push(u64::from(user_data));
    }
    bits
}

/// The shapes of the fixture: a box, a sphere, a capsule and a box with its centre of mass
/// moved.
struct Parts {
    cube: Shape,
    ball: Shape,
    capsule: Shape,
    offset: Shape,
}

fn parts() -> Parts {
    let cube = Shape::new_box(Vec3::new(0.5, 0.4, 0.3)).unwrap();
    let offset = Shape::new_offset_center_of_mass(&cube, Vec3::new(0.1, 0.2, 0.0)).unwrap();
    Parts {
        cube,
        ball: Shape::new_sphere(0.3).unwrap(),
        capsule: Shape::new_capsule(0.4, 0.2).unwrap(),
        offset,
    }
}

/// Child `i` of the fixture: shapes in turn, rotations with a negative scalar part, near
/// identity and about a slanted axis.
fn fixture_child(parts: &Parts, i: u32) -> CompoundChild<'_> {
    let shape = [&parts.cube, &parts.ball, &parts.capsule, &parts.offset][i as usize % 4];
    let rotation = [
        Quat::from_xyzw(0.0, 0.0, 0.0, -1.0),
        Quat::from_xyzw(0.0, 1.0e-4, 0.0, 1.0 - 5.0e-9),
        Quat::from_xyzw(0.5, -0.5, 0.5, -0.5),
        Quat::IDENTITY,
        Quat::from_xyzw(
            0.0,
            std::f32::consts::FRAC_1_SQRT_2,
            0.0,
            std::f32::consts::FRAC_1_SQRT_2,
        ),
    ][i as usize % 5];
    let f = i as f32;
    CompoundChild {
        shape,
        position: Vec3::new(1.5 * f, 0.25 * f, -0.5 * f),
        rotation,
        user_data: 100 + i,
    }
}

fn assert_same_as_new_compound(editor: &MutableCompound, children: &[CompoundChild<'_>]) {
    let published = editor.to_shape().unwrap();
    let built = Shape::new_compound(children).unwrap();
    assert_eq!(fingerprint(&published), fingerprint(&built));
}

#[test]
fn to_shape_builds_what_new_compound_builds() {
    let parts = parts();
    for count in [1, 2, 5, 9] {
        let children: Vec<_> = (0..count).map(|i| fixture_child(&parts, i)).collect();
        let mut editor = MutableCompound::from_children(&children).unwrap();
        assert_same_as_new_compound(&editor, &children);

        // Appended one by one.
        let mut appended = MutableCompound::new().unwrap();
        for child in &children {
            appended.add_shape(child).unwrap();
        }
        assert_same_as_new_compound(&appended, &children);

        // A spare appended and removed.
        let spare = fixture_child(&parts, 7);
        editor.add_shape(&spare).unwrap();
        editor.remove_shape(count).unwrap();
        assert_same_as_new_compound(&editor, &children);

        // A pose moved away and back.
        let last = count - 1;
        let child = children[last as usize];
        editor
            .modify_shape(last, Vec3::new(9.0, 9.0, 9.0), Quat::IDENTITY, None)
            .unwrap();
        editor
            .modify_shape(last, child.position, child.rotation, None)
            .unwrap();
        assert_same_as_new_compound(&editor, &children);

        // A shape replaced and restored.
        editor
            .modify_shape(0, Vec3::ZERO, Quat::IDENTITY, Some(&parts.ball))
            .unwrap();
        let first = children[0];
        editor
            .modify_shape(0, first.position, first.rotation, Some(first.shape))
            .unwrap();
        assert_same_as_new_compound(&editor, &children);

        // A middle child removed and the same one added back at the end.
        if count > 2 {
            editor.remove_shape(1).unwrap();
            editor.add_shape(&children[1]).unwrap();
            let mut reordered = children.clone();
            let moved = reordered.remove(1);
            reordered.push(moved);
            assert_same_as_new_compound(&editor, &reordered);
        }
    }
}

/// The child shapes the holder keeps, in order.
fn holder_children(editor: &MutableCompound) -> Vec<*const JPH_Shape> {
    let holder: *const JPH_CompoundShape = editor.holder.as_ptr().cast();
    // SAFETY: the holder is live while `editor` is borrowed; the getter only reads it.
    let count = unsafe { JPH_CompoundShape_GetNumSubShapes(holder) };
    assert_eq!(count, editor.sub_shape_count());
    (0..count).map(|index| editor.child_shape(index)).collect()
}

#[test]
fn the_holder_stays_index_aligned() {
    let parts = parts();
    let shapes = [&parts.cube, &parts.ball, &parts.capsule, &parts.offset];
    let mut editor = MutableCompound::new().unwrap();
    let mut model: Vec<*const JPH_Shape> = Vec::new();
    for (i, shape) in shapes.iter().enumerate() {
        let child = CompoundChild {
            shape,
            position: Vec3::new(i as f32, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: i as u32,
        };
        assert_eq!(editor.add_shape(&child).unwrap(), i as u32);
        model.push(shape.as_ptr());
        assert_eq!(holder_children(&editor), model);
    }
    editor.remove_shape(1).unwrap();
    model.remove(1);
    assert_eq!(holder_children(&editor), model);
    editor
        .modify_shape(2, Vec3::ZERO, Quat::IDENTITY, Some(&parts.ball))
        .unwrap();
    model[2] = parts.ball.as_ptr();
    assert_eq!(holder_children(&editor), model);
    let user_data: Vec<_> = (0..3).map(|i| editor.sub_shape_user_data(i)).collect();
    assert_eq!(user_data, [Some(0), Some(2), Some(3)]);
    editor.remove_shape(0).unwrap();
    model.remove(0);
    assert_eq!(holder_children(&editor), model);
}

#[test]
fn child_count_is_bounded_by_jolts_block_arithmetic() {
    assert!(fits_child_count(0));
    assert!(fits_child_count((u32::MAX - 3) as usize));
    assert!(!fits_child_count((u32::MAX - 2) as usize));
    // Jolt's block count `(count + 3) >> 2` still holds every child at the bound.
    let blocks = |count: u32| count.wrapping_add(3) >> 2;
    assert_eq!(blocks(MAX_CHILDREN), MAX_CHILDREN.div_ceil(4));
    assert_eq!(blocks(MAX_CHILDREN + 1), 0, "one more wraps");
}

#[test]
fn sub_shape_ids_are_checked_before_every_edit() {
    let refused = Err(ShapeError::InvalidValue(SUB_SHAPE_ID_RULE));
    let cube = unit_box();
    let at = |shape, x| CompoundChild {
        shape,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    let deep = height_field_at_32_bits();

    // Alone, a 32-bit child needs no index bit.
    let mut editor = MutableCompound::from_children(&[at(&deep, 0.0)]).unwrap();
    editor.to_shape().unwrap();
    assert_eq!(editor.add_shape(&at(&cube, 300.0)).map(|_| ()), refused);
    assert_eq!(editor.sub_shape_count(), 1);

    let mut pair = MutableCompound::from_children(&[at(&cube, 0.0), at(&cube, 3.0)]).unwrap();
    assert_eq!(
        pair.modify_shape(0, Vec3::ZERO, Quat::IDENTITY, Some(&deep)),
        refused
    );
    assert_eq!(holder_children(&pair), [cube.as_ptr(), cube.as_ptr()]);
    assert_eq!(
        MutableCompound::from_children(&[at(&deep, 0.0), at(&cube, 300.0)]).map(|_| ()),
        refused
    );

    editor.remove_shape(0).unwrap();
    editor.add_shape(&at(&cube, 0.0)).unwrap();
    editor.add_shape(&at(&cube, 3.0)).unwrap();
    editor.to_shape().unwrap();

    // A one-child compound under 31 bits: alone it fits, with a sibling its 0-bit index would
    // start at bit 32.
    let single = Shape::new_compound(&[CompoundChild {
        shape: &cube,
        position: Vec3::ZERO,
        rotation: Quat::IDENTITY,
        user_data: 0,
    }])
    .unwrap();
    let chain = nested_pairs(single, 31).unwrap();
    let mut editor = MutableCompound::from_children(&[at(&chain, 0.0)]).unwrap();
    editor.to_shape().unwrap();
    assert_eq!(editor.add_shape(&at(&cube, 300.0)).map(|_| ()), refused);
    assert_eq!(editor.sub_shape_count(), 1);
}

#[test]
fn refused_edits_walk_a_shared_child_once() {
    let refused = Err(ShapeError::InvalidValue(SUB_SHAPE_ID_RULE));
    let cube = unit_box();
    let at = |shape| CompoundChild {
        shape,
        position: Vec3::ZERO,
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    // 13 bits of heightfield under 19 shared levels: 32 bits, 20 distinct shapes.
    let graph = shared_pairs(height_field_13_bits(), 19);
    let mut pair = MutableCompound::from_children(&[at(&cube), at(&cube)]).unwrap();

    walk_count::take();
    assert_eq!(pair.add_shape(&at(&graph)).map(|_| ()), refused);
    assert_eq!(walk_count::take(), (20, 1));
    assert_eq!(
        pair.modify_shape(0, Vec3::ZERO, Quat::IDENTITY, Some(&graph)),
        refused
    );
    assert_eq!(walk_count::take(), (20, 1));
    assert_eq!(
        MutableCompound::from_children(&[at(&graph), at(&graph), at(&graph)]).map(|_| ()),
        refused
    );
    assert_eq!(walk_count::take(), (20, 1));
    assert_eq!(holder_children(&pair), [cube.as_ptr(), cube.as_ptr()]);
}

#[test]
fn edits_above_the_expansion_bound_are_refused() {
    use crate::limits::MAX_EXPANDED_SUB_SHAPES as MAX;

    let refused = |expanded| Err(ShapeError::TooManySubShapes { expanded });
    let cube = unit_box();
    let at = |shape, x| CompoundChild {
        shape,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    // 18 shared levels over a box: 2^19 - 1 shapes once every use is counted.
    let graph = shared_pairs(unit_box(), 18);
    let mut editor = MutableCompound::from_children(&[at(&graph, 0.0), at(&graph, 0.0)]).unwrap();
    // The editor, two graphs and a cube: exactly the bound.
    assert_eq!(editor.add_shape(&at(&cube, 3.0)), Ok(2));
    walk_count::take();
    assert_eq!(
        editor.add_shape(&at(&cube, 5.0)).map(|_| ()),
        refused(MAX + 1)
    );
    assert_eq!(walk_count::take(), (1, 1));
    assert_eq!(editor.sub_shape_count(), 3);
    assert_eq!(
        editor.modify_shape(2, Vec3::ZERO, Quat::IDENTITY, Some(&graph)),
        refused(MAX + MAX / 2 - 2)
    );
    assert_eq!(walk_count::take(), (19, 1));
    assert_eq!(holder_children(&editor)[2], cube.as_ptr());
    editor.to_shape().unwrap();

    // Removing a graph makes room for the one the cube is replaced by.
    editor.remove_shape(0).unwrap();
    editor
        .modify_shape(1, Vec3::ZERO, Quat::IDENTITY, Some(&graph))
        .unwrap();
    assert_eq!(editor.add_shape(&at(&cube, 3.0)), Ok(2));
    assert_eq!(
        editor.add_shape(&at(&cube, 5.0)).map(|_| ()),
        refused(MAX + 1)
    );

    walk_count::take();
    assert_eq!(
        MutableCompound::from_children(&[at(&graph, 0.0), at(&graph, 0.0), at(&graph, 0.0)])
            .map(|_| ()),
        refused(3 * (MAX / 2 - 1) + 1)
    );
    assert_eq!(walk_count::take(), (19, 1));
}

#[test]
fn an_empty_holder_has_jolts_width() {
    let editor = MutableCompound::new().unwrap();
    let holder = editor.holder.as_ptr();
    // SAFETY: the holder is live while `editor` is; the memo starts empty and the getter only
    // reads the holder.
    let (walked, jolts) = unsafe {
        (
            sub_shape_ids(holder, &mut BTreeMap::new()),
            JPH_Shape_GetSubShapeIDBitsRecursive(holder),
        )
    };
    assert_eq!(walked.width, jolts);
    assert_eq!(walked.zero_width_at, None);
}

#[test]
fn every_edit_checks_its_index() {
    let cube = unit_box();
    let child = CompoundChild {
        shape: &cube,
        position: Vec3::ZERO,
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    let mut editor = MutableCompound::from_children(&[child, child]).unwrap();
    let missing = Err(ShapeError::NoSubShape { index: 2, count: 2 });
    assert_eq!(editor.remove_shape(2), missing);
    assert_eq!(
        editor.modify_shape(2, Vec3::ZERO, Quat::IDENTITY, None),
        missing
    );
    assert_eq!(
        editor.modify_shape(2, Vec3::ZERO, Quat::IDENTITY, Some(&cube)),
        missing
    );
    assert_eq!(
        editor.remove_shape(u32::MAX),
        Err(ShapeError::NoSubShape {
            index: u32::MAX,
            count: 2
        })
    );
    assert_eq!(editor.sub_shape_count(), 2);
    assert_eq!(editor.sub_shape_user_data(2), None);
}

#[test]
fn refused_edits_change_nothing() {
    let parts = parts();
    let children: Vec<_> = (0..3).map(|i| fixture_child(&parts, i)).collect();
    let mut editor = MutableCompound::from_children(&children).unwrap();
    let before = fingerprint(&editor.to_shape().unwrap());
    let beyond = MAX_SHAPE_EXTENT + 1.0;
    let positions = [
        Vec3::new(f32::NAN, 0.0, 0.0),
        Vec3::new(0.0, beyond, 0.0),
        Vec3::new(0.0, 0.0, -beyond),
        Vec3::new(f32::INFINITY, 0.0, 0.0),
    ];
    for position in positions {
        let child = CompoundChild {
            position,
            ..children[0]
        };
        let refused = Err(ShapeError::InvalidValue(CHILD_POSITION_RULE));
        assert_eq!(editor.add_shape(&child).map(|_| ()), refused);
        assert_eq!(
            editor.modify_shape(1, position, Quat::IDENTITY, Some(&parts.ball)),
            refused
        );
        assert_eq!(
            MutableCompound::from_children(&[child]).map(|_| ()),
            refused
        );
    }
    let rotations = [
        Quat::from_xyzw(0.0, 0.0, 0.0, 2.0),
        Quat::from_xyzw(0.0, 0.0, 0.0, 0.0),
        Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0),
    ];
    for rotation in rotations {
        let child = CompoundChild {
            rotation,
            ..children[0]
        };
        let refused = Err(ShapeError::InvalidValue(CHILD_ROTATION_RULE));
        assert_eq!(editor.add_shape(&child).map(|_| ()), refused);
        assert_eq!(editor.modify_shape(1, Vec3::ZERO, rotation, None), refused);
    }
    assert_eq!(editor.sub_shape_count(), 3);
    assert_eq!(fingerprint(&editor.to_shape().unwrap()), before);
}

#[test]
fn removing_to_zero_and_adding_again_works() {
    let cube = unit_box();
    let child = |user_data| CompoundChild {
        shape: &cube,
        position: Vec3::new(1.0, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data,
    };
    let mut editor = MutableCompound::from_children(&[child(1), child(2)]).unwrap();
    editor.remove_shape(1).unwrap();
    editor.remove_shape(0).unwrap();
    assert_eq!(
        editor.to_shape().map(|_| ()),
        Err(ShapeError::InvalidValue(EMPTY_COMPOUND_RULE))
    );
    assert_eq!(
        MutableCompound::new().unwrap().to_shape().map(|_| ()),
        editor.to_shape().map(|_| ())
    );

    editor.add_shape(&child(9)).unwrap();
    let single = editor.to_shape().unwrap();
    assert_eq!(single.sub_type(), JPH_ShapeSubType_MutableCompound);
    assert_eq!(
        single.compound_sub_shape(SubShapeId::new(u32::MAX)),
        Some(CompoundSubShape {
            index: 0,
            user_data: 9
        })
    );
}

#[test]
fn extent_is_checked_after_the_centre_of_mass_moves() {
    let heavy = Shape::new_box(Vec3::new(50.0, 50.0, 50.0)).unwrap();
    let light = Shape::new_box(Vec3::new(5.0, 5.0, 5.0)).unwrap();
    let child = |shape, x| CompoundChild {
        shape,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    // Both positions are valid, but the centre of mass moves near the heavy box and the light
    // box's far side ends beyond the extent.
    let mut editor =
        MutableCompound::from_children(&[child(&heavy, -1900.0), child(&light, 1990.0)]).unwrap();
    assert!(matches!(
        editor.to_shape(),
        Err(ShapeError::InvalidValue(_))
    ));
    editor.remove_shape(1).unwrap();
    editor.add_shape(&child(&light, -1700.0)).unwrap();
    editor.to_shape().unwrap();

    // Equal masses at the two ends keep the centre at the origin.
    let balanced =
        MutableCompound::from_children(&[child(&light, -1990.0), child(&light, 1990.0)]).unwrap();
    let shape = balanced.to_shape().unwrap();
    assert_eq!(shape.center_of_mass().x, 0.0);
}

#[test]
fn mass_free_children_leave_the_centre_where_mass_puts_it() {
    let cube = unit_box();
    let (vertices, triangles) = (
        [
            Vec3::new(-1.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, -1.0),
            Vec3::new(0.0, 0.0, 1.0),
        ],
        [[0, 2, 1]],
    );
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let child = |shape, x| CompoundChild {
        shape,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    let mixed = MutableCompound::from_children(&[child(&cube, 3.0), child(&mesh, -5.0)]).unwrap();
    assert_eq!(
        mixed.to_shape().unwrap().center_of_mass(),
        Vec3::new(3.0, 0.0, 0.0)
    );

    let settings = HeightFieldSettings::default();
    let field = Shape::new_height_field(4, &[0.0; 16], &settings).unwrap();
    let holes = Shape::new_height_field(4, &[f32::MAX; 16], &settings).unwrap();
    let editor = MutableCompound::from_children(&[
        child(&mesh, 4.0),
        child(&field, -8.0),
        child(&holes, 2.0),
    ])
    .unwrap();
    let terrain = editor.to_shape().unwrap();
    assert_eq!(terrain.center_of_mass(), Vec3::ZERO);

    let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    let floor = world
        .create_body(&cube, &BodySettings::new_static())
        .unwrap();
    world
        .body_mut(floor)
        .unwrap()
        .set_shape(&terrain, None, Activation::DontActivate)
        .unwrap();
    let crate_body = world
        .create_body(
            &cube,
            &BodySettings::new_dynamic().position(crate::RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    assert!(world
        .body_mut(crate_body)
        .unwrap()
        .set_shape(&terrain, None, Activation::DontActivate)
        .is_err());
}
