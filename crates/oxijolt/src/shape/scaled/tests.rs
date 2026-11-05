use super::*;
use crate::body::mass_properties;
use crate::material::shape_material;
use crate::shape::geometry::{cross, length, sub};
use crate::shape::tests::rotated_translated;
use crate::{CompoundChild, HeightFieldSettings, MeshSettings, PhysicsMaterial, Quat, SubShapeId};

fn unit_box() -> Shape {
    Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap()
}

/// The shape's local bounds as `[min, max]`.
fn bounds(shape: &Shape) -> [[f32; 3]; 2] {
    let (min, max) = shape.local_bounds();
    [min.into(), max.into()]
}

fn invalid_settings(result: Result<Shape, ShapeError>) -> bool {
    matches!(result, Err(ShapeError::InvalidValue(_)))
}

/// Whether `Shape::new_scaled` refused thin triangles, checked for the default convex extent.
fn thin_triangles(result: Result<Shape, ShapeError>) -> bool {
    matches!(
        result,
        Err(ShapeError::ThinTriangles(ThinTrianglesError {
            max_convex_extent: MeshSettings::DEFAULT_MAX_CONVEX_EXTENT,
            ..
        }))
    )
}

fn invalid_dimensions(result: Result<Shape, ShapeError>) -> bool {
    matches!(result, Err(ShapeError::InvalidValue(_)))
}

fn child(shape: &Shape, position: Vec3, rotation: Quat) -> CompoundChild<'_> {
    CompoundChild {
        shape,
        position,
        rotation,
        user_data: 0,
    }
}

/// A 1 m square in the y = 0 plane, two triangles facing up.
fn quad_mesh() -> Shape {
    let vertices = [
        Vec3::new(-0.5, 0.0, -0.5),
        Vec3::new(-0.5, 0.0, 0.5),
        Vec3::new(0.5, 0.0, 0.5),
        Vec3::new(0.5, 0.0, -0.5),
    ];
    Shape::new_mesh(&vertices, &[[0, 1, 2], [0, 2, 3]])
        .unwrap()
        .0
}

#[test]
fn box_bounds_follow_the_scale() {
    let scaled = Shape::new_scaled(&unit_box(), Vec3::new(1.0, 2.0, 3.0)).unwrap();
    assert_eq!(scaled.sub_type(), JPH_ShapeSubType_Scaled);
    assert_eq!(bounds(&scaled), [[-0.5, -1.0, -1.5], [0.5, 1.0, 1.5]]);
    let nested = Shape::new_scaled(&scaled, Vec3::new(2.0, 1.0, 1.0)).unwrap();
    assert_eq!(bounds(&nested), [[-1.0, -1.0, -1.5], [1.0, 1.0, 1.5]]);
}

#[test]
fn jolt_scale_rules_per_shape_kind() {
    let sphere = Shape::new_sphere(0.5).unwrap();
    assert!(invalid_settings(Shape::new_scaled(
        &sphere,
        Vec3::new(1.0, 2.0, 1.0)
    )));
    assert!(Shape::new_scaled(&sphere, Vec3::new(2.0, 2.0, 2.0)).is_ok());
    assert!(Shape::new_scaled(&sphere, Vec3::new(-2.0, 2.0, 2.0)).is_ok());
    let cylinder = Shape::new_cylinder(1.0, 0.5).unwrap();
    assert!(Shape::new_scaled(&cylinder, Vec3::new(2.0, 5.0, 2.0)).is_ok());
    assert!(invalid_settings(Shape::new_scaled(
        &cylinder,
        Vec3::new(2.0, 1.0, 3.0)
    )));

    let block = unit_box();
    let turned = Quat::from_xyzw(
        0.0,
        (22.5f32).to_radians().sin(),
        0.0,
        (22.5f32).to_radians().cos(),
    );
    let compound = Shape::new_compound(&[
        child(&block, Vec3::ZERO, Quat::IDENTITY),
        child(&block, Vec3::new(2.0, 0.0, 0.0), turned),
    ])
    .unwrap();
    assert!(invalid_settings(Shape::new_scaled(
        &compound,
        Vec3::new(2.0, 1.0, 1.0)
    )));
    // Scaling along the turn's axis keeps the child's axes.
    assert!(Shape::new_scaled(&compound, Vec3::new(1.0, 2.0, 1.0)).is_ok());
    assert!(Shape::new_scaled(&compound, Vec3::new(2.0, 2.0, 2.0)).is_ok());
}

#[test]
fn scale_components_are_checked() {
    let block = unit_box();
    assert!(Shape::new_scaled(&block, Vec3::new(1.0e-6, 1.0, 1.0)).is_ok());
    assert!(invalid_settings(Shape::new_scaled(
        &block,
        Vec3::new(0.99e-6, 1.0, 1.0)
    )));
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert!(invalid_dimensions(Shape::new_scaled(
            &block,
            Vec3::new(1.0, bad, 1.0)
        )));
    }
}

#[test]
fn scaled_bounds_and_centre_of_mass_stay_within_the_extent() {
    let max = limits::MAX_SHAPE_EXTENT;
    let block = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    assert!(Shape::new_scaled(&block, Vec3::new(max, 1.0, 1.0)).is_ok());
    assert!(invalid_dimensions(Shape::new_scaled(
        &block,
        Vec3::new(max.next_up(), 1.0, 1.0)
    )));

    // A one-child compound keeps its centre of mass at the child, 1500 m out.
    let far = Shape::new_compound(&[child(
        &unit_box(),
        Vec3::new(1500.0, 0.0, 0.0),
        Quat::IDENTITY,
    )])
    .unwrap();
    assert!(Shape::new_scaled(&far, Vec3::new(1.2, 1.2, 1.2)).is_ok());
    assert!(invalid_dimensions(Shape::new_scaled(
        &far,
        Vec3::new(2.0, 2.0, 2.0)
    )));

    // A field of holes has no surface; its bounds are a point, which the scale moves away.
    let holes =
        Shape::new_height_field(3, &[f32::MAX; 9], &HeightFieldSettings::default()).unwrap();
    assert!(invalid_dimensions(Shape::new_scaled(
        &holes,
        Vec3::new(1.0e30, 1.0, 1.0e30)
    )));
}

#[test]
fn scaled_box_has_the_mass_of_the_equivalent_box() {
    let scaled = Shape::new_scaled(&unit_box(), Vec3::new(1.0, 2.0, 3.0)).unwrap();
    let equivalent = Shape::new_box(Vec3::new(0.5, 1.0, 1.5)).unwrap();
    let (a, b) = (
        mass_properties(&scaled, None),
        mass_properties(&equivalent, None),
    );
    let close = |x: f32, y: f32| (x - y).abs() <= 8.0 * f32::EPSILON * y.abs().max(1.0e-30);
    assert!(close(a.mass, b.mass), "{} vs {}", a.mass, b.mass);
    for column in 0..3 {
        let (p, q) = (a.inertia.column[column], b.inertia.column[column]);
        for (x, y) in [(p.x, q.x), (p.y, q.y), (p.z, q.z)] {
            assert!(x == y || close(x, y), "{x} vs {y}");
        }
    }
}

#[test]
fn materials_survive_scaling() {
    let material = PhysicsMaterial::new(44).unwrap();
    let inner = Shape::new_box_with_material(Vec3::new(0.5, 0.5, 0.5), 0.05, &material).unwrap();
    let scaled = Shape::new_scaled(&inner, Vec3::new(2.0, 1.0, 1.0)).unwrap();
    drop((inner, material));
    let (origin, direction) = (
        Vec3::new(0.0, 2.0, 0.0).to_jph(),
        Vec3::new(0.0, -4.0, 0.0).to_jph(),
    );
    let mut hit = JPH_RayCastResult {
        bodyID: 0,
        fraction: 2.0,
        subShapeID2: 0,
    };
    // SAFETY: the shape is live; every argument is a live local.
    assert!(unsafe { JPH_Shape_CastRay(scaled.as_ptr(), &origin, &direction, &mut hit) });
    // SAFETY: the shape is live and the id came from a hit on it.
    let found = unsafe { shape_material(scaled.as_ptr(), SubShapeId::new(hit.subShapeID2)) };
    assert_eq!(found, Some(44));
}

#[test]
fn scaled_meshes_stay_kinematic_and_collidable() {
    let mesh = quad_mesh();
    let doubled = Shape::new_scaled(&mesh, Vec3::new(2.0, 2.0, 2.0)).unwrap();
    assert!(doubled.static_only_leaves_are_meshes());
    assert!(Shape::new_scaled(&mesh, Vec3::new(1.0, -1.0, 1.0)).is_ok());
    // Twice the area of a 1 m triangle scaled by s is s^2: 1e-4 at s = 0.01, 1e-6 at s = 0.001.
    assert!(Shape::new_scaled(&mesh, Vec3::new(0.01, 1.0, 0.01)).is_ok());
    assert!(thin_triangles(Shape::new_scaled(
        &mesh,
        Vec3::new(0.001, 1.0, 0.001)
    )));
    // The check reaches meshes inside compounds and other decorators.
    let compound = Shape::new_compound(&[
        child(&mesh, Vec3::ZERO, Quat::IDENTITY),
        child(&unit_box(), Vec3::new(3.0, 0.0, 0.0), Quat::IDENTITY),
    ])
    .unwrap();
    let offset = Shape::new_offset_center_of_mass(&compound, Vec3::new(0.2, 0.0, 0.0)).unwrap();
    assert!(thin_triangles(Shape::new_scaled(
        &offset,
        Vec3::new(0.001, 0.001, 0.001)
    )));
    assert!(Shape::new_scaled(&offset, Vec3::new(0.5, 0.5, 0.5)).is_ok());
    let field = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let scaled_field = Shape::new_scaled(&field, Vec3::new(2.0, 1.0, 2.0)).unwrap();
    assert!(!scaled_field.static_only_leaves_are_meshes());
    assert!(thin_triangles(Shape::new_scaled(
        &field,
        Vec3::new(0.001, 1.0, 0.001)
    )));
}

/// A quarter turn about +y: x goes to -z, z to x.
fn quarter_turn() -> Quat {
    let half = std::f32::consts::FRAC_PI_4;
    Quat::from_xyzw(0.0, half.sin(), 0.0, half.cos())
}

/// A 1 m square in the y = 0 plane around `(x, 0, 0)` of its own space, facing up.
fn quad_at(x: f32) -> Shape {
    let vertices = [
        Vec3::new(x - 0.5, 0.0, -0.5),
        Vec3::new(x - 0.5, 0.0, 0.5),
        Vec3::new(x + 0.5, 0.0, 0.5),
        Vec3::new(x + 0.5, 0.0, -0.5),
    ];
    Shape::new_mesh(&vertices, &[[0, 1, 2], [0, 2, 3]])
        .unwrap()
        .0
}

/// Checks the placements of the meshes in `inner` scaled by `scale` against Jolt: each placed
/// triangle, moved by `translation` (the leaf's position in the scaled shape's centre of mass
/// space, by Jolt's rules), is where a ray along its normal hits the scaled shape.
fn assert_placements_match_jolt(inner: &Shape, scale: Vec3, translation: [f64; 3]) {
    let scaled = Shape::new_scaled(inner, scale).unwrap();
    let leaves = triangle_leaves(inner, scale).unwrap();
    assert!(!leaves.is_empty());
    for (leaf, placement) in leaves {
        // SAFETY: `leaf` is a live mesh part of `inner`.
        for [a, b, c] in unsafe { placed_triangles(leaf, &placement) } {
            let normal = cross(sub(b, a), sub(c, a));
            let unit = normal.map(|x| x / length(normal));
            let centre = [0, 1, 2].map(|i| (a[i] + b[i] + c[i]) / 3.0 + translation[i]);
            let to_vec3 = |v: [f64; 3]| Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32);
            let origin = to_vec3([0, 1, 2].map(|i| centre[i] + 0.1 * unit[i])).to_jph();
            let direction = to_vec3(unit.map(|x| -0.2 * x)).to_jph();
            let mut hit = JPH_RayCastResult {
                bodyID: 0,
                fraction: 2.0,
                subShapeID2: 0,
            };
            // SAFETY: the shape is live; every argument is a live local.
            let found =
                unsafe { JPH_Shape_CastRay(scaled.as_ptr(), &origin, &direction, &mut hit) };
            assert!(found, "no hit at {centre:?}");
            assert!(
                (hit.fraction - 0.5).abs() < 1.0e-3,
                "{} at {centre:?}",
                hit.fraction
            );
        }
    }
}

#[test]
fn placements_match_jolts_centre_of_mass_spaces() {
    let scale = Vec3::new(2.0, 3.0, 0.5);
    let s = v3(scale);
    // Off its own origin, so that a missing turn or translation moves it off Jolt's.
    let quad = quad_at(2.0);
    let block = unit_box();

    // Scaled(offset(quad, d)): Jolt puts a point v at s * (v - d).
    let offset = Shape::new_offset_center_of_mass(&quad, Vec3::new(3.0, 1.0, -2.0)).unwrap();
    assert_placements_match_jolt(&offset, scale, [-3.0 * s[0], -s[1], 2.0 * s[2]]);
    // Offsets that cancel out.
    let back = Shape::new_offset_center_of_mass(&offset, Vec3::new(-3.0, -1.0, 2.0)).unwrap();
    assert_placements_match_jolt(&back, scale, [0.0; 3]);

    // A turned compound child at p, next to the box that holds the compound's centre of mass:
    // s * (p + R v).
    let p = Vec3::new(4.0, 0.0, 1.0);
    let compound = Shape::new_compound(&[
        child(&block, Vec3::ZERO, Quat::IDENTITY),
        child(&quad, p, quarter_turn()),
    ])
    .unwrap();
    assert_placements_match_jolt(&compound, scale, [4.0 * s[0], 0.0, s[2]]);

    // A rotated-translated shape keeps its centre of mass at its position: s * R v.
    let turned = rotated_translated(&quad, Vec3::new(5.0, 2.0, 0.0), quarter_turn());
    assert_placements_match_jolt(&turned, scale, [0.0; 3]);

    // A scaled mesh as a turned compound child, inside the scale.
    let stretched = Shape::new_scaled(&quad, Vec3::new(2.0, 1.0, 3.0)).unwrap();
    let nested = Shape::new_compound(&[
        child(&block, Vec3::ZERO, Quat::IDENTITY),
        child(&stretched, p, quarter_turn()),
    ])
    .unwrap();
    assert_placements_match_jolt(&nested, scale, [4.0 * s[0], 0.0, s[2]]);
}

#[test]
fn translations_above_a_mesh_do_not_change_its_check() {
    // A 1 cm triangle at its own origin with the centre of mass moved 1999 m away: Jolt rounds
    // the triangle's own coordinates and folds the offset into the transform it applies after.
    let vertices = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.01, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.01),
    ];
    let (tiny, _) = Shape::new_mesh(&vertices, &[[0, 2, 1]]).unwrap();
    let away = Shape::new_offset_center_of_mass(&tiny, Vec3::new(-1999.0, 0.0, 0.0)).unwrap();
    assert!(Shape::new_scaled(&away, Vec3::new(1.0, 1.0, 1.0)).is_ok());
    let compound = Shape::new_compound(&[
        child(&unit_box(), Vec3::ZERO, Quat::IDENTITY),
        child(&tiny, Vec3::new(1999.0, 0.0, 0.0), Quat::IDENTITY),
    ])
    .unwrap();
    assert!(Shape::new_scaled(&compound, Vec3::new(1.0, 1.0, 1.0)).is_ok());
}

#[test]
fn flattening_follows_the_meshs_own_coordinates() {
    // A quad built 1000 m from its own origin keeps the rounding of 1000 m coordinates wherever
    // compounds, offsets and rotated-translated shapes move it. Flattened along z, twice a
    // triangle's area is the z scale; with that rounding the floor is about 0.0019, so 0.006
    // is kept and 0.001 refused.
    let far = quad_at(1000.0);
    let block = unit_box();
    let to_origin = Vec3::new(-1000.0, 0.0, 0.0);
    let offset = Shape::new_offset_center_of_mass(&far, Vec3::new(1000.0, 0.0, 0.0)).unwrap();
    let cancelled = Shape::new_offset_center_of_mass(&offset, to_origin).unwrap();
    let in_compound = Shape::new_compound(&[
        child(&block, Vec3::ZERO, Quat::IDENTITY),
        child(&far, to_origin, Quat::IDENTITY),
    ])
    .unwrap();
    let moved = rotated_translated(&far, to_origin, Quat::IDENTITY);
    for shape in [&far, &offset, &cancelled, &in_compound, &moved] {
        assert!(Shape::new_scaled(shape, Vec3::new(1.0, 1.0, 0.006)).is_ok());
        assert!(thin_triangles(Shape::new_scaled(
            shape,
            Vec3::new(1.0, 1.0, 0.001)
        )));
    }
    // Turned a quarter about y, the quad's own z lies along x and its far coordinate along z:
    // flattening x thins the triangles at 1000 m, flattening z shrinks that coordinate too.
    let turned = Shape::new_compound(&[
        child(&block, Vec3::ZERO, Quat::IDENTITY),
        child(&far, to_origin, quarter_turn()),
    ])
    .unwrap();
    assert!(thin_triangles(Shape::new_scaled(
        &turned,
        Vec3::new(0.001, 1.0, 1.0)
    )));
    assert!(Shape::new_scaled(&turned, Vec3::new(1.0, 1.0, 0.001)).is_ok());
}

#[test]
fn scaled_meshes_are_checked_for_the_extent_they_were_built_for() {
    // A sliver 1 m by 20 um, kept for convex shapes up to 1 m and refused for the default.
    let sliver = |width: f32| {
        [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(width, 0.0, 0.5),
        ]
    };
    let small = MeshSettings::default().max_convex_extent(1.0);
    let (mesh, dropped) =
        Shape::new_mesh_with_settings(&sliver(2.0e-5), &[[0, 1, 2]], &small).unwrap();
    assert!(dropped.is_empty());
    for scale in [1.0, 2.0] {
        assert!(Shape::new_scaled(&mesh, Vec3::new(scale, scale, scale)).is_ok());
    }
    let shrunk = Vec3::new(0.2, 0.2, 0.2);
    assert_eq!(
        Shape::new_scaled(&mesh, shrunk).err(),
        Some(ShapeError::ThinTriangles(ThinTrianglesError {
            scale: shrunk,
            max_convex_extent: 1.0
        }))
    );
    // A strip 2.5 mm wide kept for 4000 m is checked for 4000 m: halved across its width it is
    // refused, although the default extent would keep it.
    let large = MeshSettings::default().max_convex_extent(4000.0);
    let (mesh, dropped) =
        Shape::new_mesh_with_settings(&sliver(2.5e-3), &[[0, 1, 2]], &large).unwrap();
    assert!(dropped.is_empty());
    let (thinned, _) = Shape::new_mesh(&sliver(1.25e-3), &[[0, 1, 2]]).unwrap();
    assert!(Shape::new_scaled(&thinned, Vec3::new(1.0, 1.0, 1.0)).is_ok());
    let halved = Vec3::new(0.5, 1.0, 1.0);
    assert_eq!(
        Shape::new_scaled(&mesh, halved).err(),
        Some(ShapeError::ThinTriangles(ThinTrianglesError {
            scale: halved,
            max_convex_extent: 4000.0
        }))
    );
    // Inside a compound each mesh keeps its own extent.
    let compound = Shape::new_compound(&[
        child(&unit_box(), Vec3::ZERO, Quat::IDENTITY),
        child(&mesh, Vec3::new(3.0, 0.0, 0.0), Quat::IDENTITY),
    ])
    .unwrap();
    assert!(Shape::new_scaled(&compound, Vec3::new(2.0, 2.0, 2.0)).is_ok());
    assert!(matches!(
        Shape::new_scaled(&compound, Vec3::new(0.5, 0.5, 0.5)),
        Err(ShapeError::ThinTriangles(ThinTrianglesError {
            max_convex_extent: 4000.0,
            ..
        }))
    ));
}
