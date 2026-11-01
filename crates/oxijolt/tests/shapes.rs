//! Shapes and the closest-hit ray.

mod common;

use common::*;
use oxijolt::*;

fn down_from(x: Real, y: Real, z: Real, length: f32) -> RayCast {
    RayCast::new(RVec3::new(x, y, z), Vec3::new(0.0, -length, 0.0))
}

#[test]
fn ray_hits_a_body_right_after_creation() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    let hit = world
        .cast_ray(&down_from(0.0, 5.0, 0.0, 10.0), &QueryFilter::new())
        .unwrap()
        .expect("the floor is hit");
    assert_eq!(hit.body, floor);
    assert!((hit.fraction - 0.5).abs() <= 1.0e-6, "{hit:?}");
}

#[test]
fn ray_misses_return_none() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    let up = RayCast::new(RVec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
    assert_eq!(world.cast_ray(&up, &QueryFilter::new()), Ok(None));
}

#[test]
fn ray_starting_inside_a_box_hits_at_zero() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    let hit = world
        .cast_ray(&down_from(0.0, -0.5, 0.0, 10.0), &QueryFilter::new())
        .unwrap()
        .expect("the floor is hit");
    assert_eq!(hit.body, floor);
    assert_eq!(hit.fraction, 0.0);
}

#[test]
fn invalid_rays_are_rejected() {
    let world = world(Vec3::ZERO, 1);
    let invalid = [
        RayCast::new(RVec3::new(Real::NAN, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0)),
        RayCast::new(RVec3::new(0.0, 0.0, 0.0), Vec3::ZERO),
        RayCast::new(
            RVec3::new(0.0, 0.0, 0.0),
            Vec3::new(f32::INFINITY, 0.0, 0.0),
        ),
    ];
    for ray in invalid {
        assert!(
            matches!(
                world.cast_ray(&ray, &QueryFilter::new()),
                Err(QueryError::InvalidValue(_))
            ),
            "{ray:?} was accepted"
        );
    }
}

/// Converts a column-major height grid (`heights_zx[j * n + i]` is the height at row `i` along
/// Z and column `j` along X) into Jolt's row-major samples (`samples[i * n + j]`).
fn column_major_to_jolt(heights_zx: &[f32], n: usize) -> Vec<f32> {
    let mut samples = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            samples[i * n + j] = heights_zx[j * n + i];
        }
    }
    samples
}

fn add_static(world: &mut PhysicsWorld, shape: &Shape, position: RVec3) -> BodyId {
    world
        .create_body(shape, &BodySettings::new_static().position(position))
        .unwrap()
}

/// World height where a downward ray from `y = 20` at `(x, z)` hits, if it does.
fn surface_height(world: &PhysicsWorld, x: Real, z: Real) -> Option<Real> {
    let ray = down_from(x, 20.0, z, 40.0);
    let hit = world.cast_ray(&ray, &QueryFilter::new()).unwrap()?;
    Some(ray.point_at(hit.fraction).y)
}

#[test]
fn height_field_rising_along_z_matches_analytic_surface() {
    const N: usize = 33;
    // Column-major source: row `i` lies at local z = -16 + i and rises 0.2 m per metre.
    let mut heights_zx = vec![0.0; N * N];
    for i in 0..N {
        for j in 0..N {
            heights_zx[j * N + i] = 0.2 * i as f32;
        }
    }
    let samples = column_major_to_jolt(&heights_zx, N);
    let settings = HeightFieldSettings::default().offset(Vec3::new(-16.0, 0.0, -16.0));
    let shape = Shape::new_height_field(N as u32, &samples, &settings).unwrap();

    let mut world = world(Vec3::ZERO, 1);
    let origin = RVec3::new(100.0, -3.0, 50.0);
    let terrain = add_static(&mut world, &shape, origin);

    // Every point has |x - z| >= 1, so a mapping without the transpose is off by 0.2 m or more.
    let points: [(Real, Real); 5] = [
        (-10.3, -12.7),
        (3.25, 0.5),
        (12.9, 9.1),
        (-0.4, 14.6),
        (7.7, -5.2),
    ];
    for (x, z) in points {
        let ray = RayCast::new(
            RVec3::new(origin.x + x, 17.0, origin.z + z),
            Vec3::new(0.0, -30.0, 0.0),
        );
        let hit = world
            .cast_ray(&ray, &QueryFilter::new())
            .unwrap()
            .expect("the terrain is hit");
        assert_eq!(hit.body, terrain);
        let expected = -3.0 + 0.2 * (z + 16.0);
        let actual = ray.point_at(hit.fraction).y;
        assert!(
            (actual - expected).abs() <= 0.02,
            "at ({x}, {z}): hit {actual}, expected {expected}"
        );
    }
}

#[test]
fn height_field_33_builds_and_matches_samples_at_nodes() {
    const N: u32 = 33;
    let height = |x: u32, y: u32| {
        let (x, y) = (x as f32, y as f32);
        1.5 * (0.4 * x).sin() * (0.3 * y).cos() + 0.05 * y
    };
    let samples: Vec<f32> = (0..N)
        .flat_map(|y| (0..N).map(move |x| height(x, y)))
        .collect();
    let settings = HeightFieldSettings::default()
        .bits_per_sample(16)
        .scale(Vec3::new(0.5, 1.0, 0.5));
    let shape = Shape::new_height_field(N, &samples, &settings).unwrap();

    for y in 0..N {
        for x in 0..N {
            let p = shape.height_field_position(x, y).expect("a stored node");
            assert!(
                (p.y - height(x, y)).abs() <= 1.0e-4,
                "node ({x}, {y}): {} vs {}",
                p.y,
                height(x, y)
            );
            assert_eq!(p.x, 0.5 * x as f32);
            assert_eq!(p.z, 0.5 * y as f32);
        }
    }
    // n = 33 is padded to 34 with holes; 34 is outside the stored grid.
    assert_eq!(shape.height_field_position(33, 0), None);
    assert_eq!(shape.height_field_position(0, 33), None);
    assert_eq!(shape.height_field_position(34, 0), None);

    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &shape, RVec3::new(0.0, 0.0, 0.0));
    assert!(
        surface_height(&world, 15.75, 15.75).is_some(),
        "cell (31, 31)"
    );
    assert!(surface_height(&world, 16.01, 8.0).is_none(), "padding cell");
}

#[test]
fn height_field_with_larger_blocks_matches_samples_at_nodes() {
    const N: u32 = 33;
    let height = |x: u32, y: u32| 0.06 * x as f32 - 0.04 * y as f32 + 0.002 * (x * y) as f32;
    let samples: Vec<f32> = (0..N)
        .flat_map(|y| (0..N).map(move |x| height(x, y)))
        .collect();
    // n = 33 is padded to 36 with block size 4 and to 40 with block size 8.
    for (block_size, padded) in [(4, 36), (8, 40)] {
        let settings = HeightFieldSettings::default()
            .block_size(block_size)
            .bits_per_sample(16);
        let shape = Shape::new_height_field(N, &samples, &settings).unwrap();
        for y in 0..N {
            for x in 0..N {
                let p = shape.height_field_position(x, y).expect("a stored node");
                assert!(
                    (p.y - height(x, y)).abs() <= 1.0e-4,
                    "block {block_size}, node ({x}, {y}): {} vs {}",
                    p.y,
                    height(x, y)
                );
            }
        }
        for padding in N..=padded {
            assert_eq!(shape.height_field_position(padding, 0), None);
            assert_eq!(shape.height_field_position(0, padding), None);
        }
        let mut world = world(Vec3::ZERO, 1);
        add_static(&mut world, &shape, RVec3::new(0.0, 0.0, 0.0));
        assert!(
            surface_height(&world, 31.5, 31.5).is_some(),
            "cell (31, 31)"
        );
        assert!(surface_height(&world, 32.5, 16.0).is_none(), "padding cell");
    }
}

#[test]
fn height_field_cell_diagonal_runs_from_x_y_to_x1_y1() {
    let mut samples = vec![0.0; 9];
    samples[3 + 1] = 1.0;
    let settings = HeightFieldSettings::default().bits_per_sample(16);
    let shape = Shape::new_height_field(3, &samples, &settings).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &shape, RVec3::new(0.0, 0.0, 0.0));
    // Along the other diagonal both points would lie on triangles of height 0.
    for (x, z) in [(0.75, 0.25), (0.25, 0.75)] {
        let y = surface_height(&world, x, z).expect("the cell is hit");
        assert!((y - 0.25).abs() <= 1.0e-3, "at ({x}, {z}): {y}");
    }
}

#[test]
fn height_field_is_hit_from_below() {
    let shape = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    let terrain = add_static(&mut world, &shape, RVec3::new(0.0, 0.0, 0.0));
    let ray = RayCast::new(RVec3::new(0.5, -1.0, 0.5), Vec3::new(0.0, 2.0, 0.0));
    let hit = world
        .cast_ray(&ray, &QueryFilter::new())
        .unwrap()
        .expect("the underside is hit");
    assert_eq!(hit.body, terrain);
    assert!(ray.point_at(hit.fraction).y.abs() <= 1.0e-3);
}

#[test]
fn invalid_height_fields_are_rejected() {
    let defaults = HeightFieldSettings::default;
    let flat = vec![0.0; 9];
    let with_samples = |changes: &[(usize, f32)]| {
        let mut samples = flat.clone();
        for &(index, value) in changes {
            samples[index] = value;
        }
        samples
    };
    let dimension_cases = [
        (1, vec![0.0], defaults()),
        (3, vec![0.0; 8], defaults()),
        (3, with_samples(&[(4, f32::NAN)]), defaults()),
        (3, with_samples(&[(4, f32::INFINITY)]), defaults()),
        (3, with_samples(&[(0, -3.0e38), (4, 3.0e38)]), defaults()),
        (3, flat.clone(), defaults().scale(Vec3::new(0.0, 1.0, 1.0))),
        (
            3,
            flat.clone(),
            defaults().scale(Vec3::new(1.0, f32::NAN, 1.0)),
        ),
        (3, flat.clone(), defaults().scale(Vec3::new(1.0, 1.0, -1.0))),
        (
            3,
            flat.clone(),
            defaults().offset(Vec3::new(f32::NAN, 0.0, 0.0)),
        ),
        (
            3,
            flat.clone(),
            defaults().offset(Vec3::new(0.0, 0.0, f32::INFINITY)),
        ),
        // Finite offset and scale whose far edge along x is not.
        (
            3,
            flat.clone(),
            defaults().scale(Vec3::new(3.0e38, 1.0, 1.0)),
        ),
        (
            3,
            flat.clone(),
            defaults()
                .offset(Vec3::new(0.0, 0.0, -3.0e38))
                .scale(Vec3::new(1.0, 1.0, 3.0e38)),
        ),
        // Finite offset and scale whose highest point is not.
        (
            3,
            with_samples(&[(4, 2.0)]),
            defaults()
                .offset(Vec3::new(0.0, 3.0e38, 0.0))
                .scale(Vec3::new(1.0, 3.0e38, 1.0)),
        ),
    ];
    for (count, samples, settings) in dimension_cases {
        let result = Shape::new_height_field(count, &samples, &settings);
        assert!(
            matches!(result, Err(ShapeError::InvalidValue(_))),
            "n = {count}, {settings:?}"
        );
    }
    let settings_cases = [
        (3, flat.clone(), defaults().block_size(1)),
        (3, flat.clone(), defaults().block_size(9)),
        (3, flat.clone(), defaults().bits_per_sample(0)),
        (3, flat.clone(), defaults().bits_per_sample(17)),
        (
            3,
            flat.clone(),
            defaults().active_edge_cos_threshold_angle(-0.1),
        ),
        (
            3,
            flat.clone(),
            defaults().active_edge_cos_threshold_angle(1.5),
        ),
        (
            3,
            flat.clone(),
            defaults().active_edge_cos_threshold_angle(f32::NAN),
        ),
        (2, vec![0.0; 4], defaults()),
        // The size check runs before the length check, so no huge buffer is needed.
        (40000, Vec::new(), defaults()),
    ];
    for (count, samples, settings) in settings_cases {
        let result = Shape::new_height_field(count, &samples, &settings);
        assert!(
            matches!(result, Err(ShapeError::InvalidValue(_))),
            "n = {count}, {settings:?}"
        );
    }
}

#[test]
fn height_field_of_holes_has_no_collision() {
    let shape =
        Shape::new_height_field(3, &[f32::MAX; 9], &HeightFieldSettings::default()).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &shape, RVec3::new(0.0, 0.0, 0.0));
    assert_eq!(surface_height(&world, 1.0, 1.0), None);
    assert_eq!(shape.height_field_position(1, 1), None);
}

#[test]
fn static_only_shapes_are_rejected_for_moving_bodies() {
    let shape = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    // An explicit mass passes the mass check, which a bare heightfield (zero mass) would fail
    // with the same error, so only the static-only check can reject these.
    for settings in [
        BodySettings::new_dynamic().mass(1.0),
        BodySettings::new_kinematic().mass(1.0),
    ] {
        assert!(matches!(
            world.create_body(&shape, &settings),
            Err(BodyError::InvalidValue(_))
        ));
    }
    assert_eq!(world.body_count(), 0);
    assert!(world
        .create_body(&shape, &BodySettings::new_static())
        .is_ok());
}

#[test]
fn compounds_containing_a_height_field_are_static_only() {
    let terrain = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let block = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let compound = Shape::new_compound(&[
        child(&block, Vec3::new(0.0, 1.0, 0.0), Quat::IDENTITY, 1),
        child(&terrain, Vec3::ZERO, Quat::IDENTITY, 2),
    ])
    .unwrap();
    let mut world = world(Vec3::ZERO, 1);
    for settings in [BodySettings::new_dynamic(), BodySettings::new_kinematic()] {
        assert!(matches!(
            world.create_body(&compound, &settings),
            Err(BodyError::InvalidValue(_))
        ));
    }
    assert_eq!(world.body_count(), 0);
    assert!(world
        .create_body(&compound, &BodySettings::new_static())
        .is_ok());
}

#[test]
fn height_field_with_custom_active_edge_threshold_builds() {
    const N: u32 = 9;
    let samples: Vec<f32> = (0..N)
        .flat_map(|y| (0..N).map(move |x| 0.1 * ((x + y) % 3) as f32))
        .collect();
    let settings = HeightFieldSettings::default()
        .offset(Vec3::new(-4.0, 0.0, -4.0))
        .active_edge_cos_threshold_angle(30.0_f32.to_radians().cos());
    let shape = Shape::new_height_field(N, &samples, &settings).unwrap();
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_static(&mut world, &shape, RVec3::new(0.0, 0.0, 0.0));
    let ball_shape = Shape::new_sphere(0.5).unwrap();
    let ball = world
        .create_body(
            &ball_shape,
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.3, 2.0, -0.2))
                .enhanced_internal_edge_removal(true),
        )
        .unwrap();
    step(&mut world, 60);
    let body = world.body(ball).unwrap();
    let position: [Real; 3] = body.position().into();
    assert!(position.iter().all(|value| value.is_finite()));
    assert!(length(body.linear_velocity()).is_finite());
    assert!(
        position[1] > 0.3,
        "the ball stays above the surface: {position:?}"
    );
}

/// A static 33 x 33 heightfield at the origin with a convex 4 degree crease along x = 0: both
/// sides fall away from it at 2 degrees, so every cell is planar. Sample `ix` lies at
/// `x = ix - 16`.
fn gentle_ridge_world(settings: HeightFieldSettings) -> PhysicsWorld {
    let fall = 2.0_f32.to_radians().tan();
    let samples: Vec<f32> = (0..33)
        .flat_map(|_| (0..33).map(move |ix| -fall * (ix as f32 - 16.0).abs()))
        .collect();
    let settings = settings
        .offset(Vec3::new(-16.0, 0.0, -16.0))
        .bits_per_sample(16);
    let shape = Shape::new_height_field(33, &samples, &settings).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &shape, RVec3::ZERO);
    world
}

/// Every hit of a sphere of radius 0.5 centred at `centre`.
fn sphere_hits(world: &PhysicsWorld, centre: [f32; 3]) -> Vec<CollideShapeHit> {
    let sphere = Shape::new_sphere(0.5).unwrap();
    let [x, y, z] = centre.map(Real::from);
    world
        .collide_shape(
            &CollideShape::new(&sphere, RVec3::new(x, y, z), Quat::IDENTITY),
            &QueryFilter::new(),
        )
        .unwrap()
}

/// The active-edge threshold decides whether a contact on a crease uses the triangle's normal
/// or the direction from the crease to the other shape. Jolt marks an edge active when the
/// angle between its two triangles exceeds the threshold (`ActiveEdges::IsEdgeActive`), and
/// the sphere-vs-triangle narrow phase replaces the contact normal by the triangle normal when
/// the closest feature is an inactive edge. Body contacts, CharacterVirtual and `collide_shape`
/// share this narrow phase.
///
/// A sphere resting on a 4 degree ridge, its closest feature the crease: with the default
/// 5 degree threshold the crease is inactive and each hit has its triangle's normal, tilted
/// 2 degrees; with a 3 degree threshold the crease is active and the normal points straight up
/// to the sphere. A sphere on a face, away from every edge, gets the same normal either way.
#[test]
fn active_edge_threshold_decides_the_normal_on_a_gentle_ridge() {
    let sin2 = 2.0_f32.to_radians().sin();
    let default = gentle_ridge_world(HeightFieldSettings::default());
    let active = gentle_ridge_world(
        HeightFieldSettings::default().active_edge_cos_threshold_angle(3.0_f32.to_radians().cos()),
    );

    // Centred over the crease between sample rows 16 and 17, overlapping it by 0.05.
    let crease = [0.0, 0.45, 0.5];
    for (world, name) in [(&default, "5 degrees"), (&active, "3 degrees")] {
        let hits = sphere_hits(world, crease);
        assert!(!hits.is_empty(), "{name}: no hit");
        for hit in &hits {
            let on_body: [Real; 3] = hit.point_on_body.into();
            assert!(on_body[0].abs() < 1e-3, "{name}: {hit:?}");
        }
    }
    for hit in sphere_hits(&default, crease) {
        assert!(hit.normal.x.abs() > 0.02, "{hit:?}");
        assert!(hit.normal.y > 0.99, "{hit:?}");
    }
    for hit in sphere_hits(&active, crease) {
        assert!(hit.normal.x.abs() < 1e-3, "{hit:?}");
        assert!(hit.normal.y > 0.9999, "{hit:?}");
    }

    // 0.48 above the centroid of the triangle (-1, 0), (0, 1), (0, 0), along its normal.
    let fall = 2.0_f32.to_radians().tan();
    let length = (fall * fall + 1.0).sqrt();
    let normal = [-fall / length, 1.0 / length, 0.0];
    let centroid = [-1.0 / 3.0, -fall / 3.0, 1.0 / 3.0];
    let face = [0, 1, 2].map(|i| centroid[i] + 0.48 * normal[i]);
    let on_default = sphere_hits(&default, face);
    let on_active = sphere_hits(&active, face);
    assert_eq!(on_default.len(), 1, "{on_default:?}");
    assert_eq!(on_active.len(), 1, "{on_active:?}");
    let (a, b) = (on_default[0].normal, on_active[0].normal);
    for (p, q) in [(a.x, b.x), (a.y, b.y), (a.z, b.z)] {
        assert!((p - q).abs() < 1e-5, "{a:?} vs {b:?}");
    }
    assert!((a.x + sin2).abs() < 2e-3, "{a:?}");
}

const X_AXIS: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Z_AXIS: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn child(shape: &Shape, position: Vec3, rotation: Quat, user_data: u32) -> CompoundChild<'_> {
    CompoundChild {
        shape,
        position,
        rotation,
        user_data,
    }
}

/// The closest hit of a downward ray from `y = 20` at `(x, z)` and the world height it hits.
fn hit_below(world: &PhysicsWorld, x: Real, z: Real) -> (RayCastHit, Real) {
    let ray = down_from(x, 20.0, z, 40.0);
    let hit = world
        .cast_ray(&ray, &QueryFilter::new())
        .unwrap()
        .expect("something is hit");
    (hit, ray.point_at(hit.fraction).y)
}

#[test]
fn compound_children_report_their_user_data() {
    let block = Shape::new_box(Vec3::new(1.0, 0.5, 1.0)).unwrap();
    let pillar = Shape::new_cylinder(1.0, 0.5).unwrap();
    let compound = Shape::new_compound(&[
        child(&block, Vec3::new(-3.0, 0.0, 0.0), Quat::IDENTITY, 2),
        child(
            &pillar,
            Vec3::new(3.0, 0.0, 0.0),
            quat_about(X_AXIS, 30.0_f32.to_radians()),
            3,
        ),
    ])
    .unwrap();
    let mut world = world(Vec3::ZERO, 1);
    let body = add_static(&mut world, &compound, RVec3::new(10.0, 1.0, -4.0));

    let (hit, y) = hit_below(&world, 7.0, -4.0);
    assert_eq!(hit.body, body);
    assert_eq!(
        world.compound_sub_shape(hit.body, hit.sub_shape_id),
        Ok(Some(CompoundSubShape {
            index: 0,
            user_data: 2
        }))
    );
    assert!((y - 1.5).abs() <= 1.0e-5, "box top at {y}");

    let (hit, y) = hit_below(&world, 13.0, -4.0);
    assert_eq!(hit.body, body);
    assert_eq!(
        world.compound_sub_shape(hit.body, hit.sub_shape_id),
        Ok(Some(CompoundSubShape {
            index: 1,
            user_data: 3
        }))
    );
    // The tilted cylinder's highest rim point is at 1 + cos 30 + 0.5 sin 30 = 2.116.
    assert!(y > 1.0 && y < 2.2, "cylinder hit at {y}");
}

#[test]
fn compound_child_order_is_insertion_order() {
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let user_data = [14, 10, 13, 11, 12];
    let children: Vec<_> = user_data
        .iter()
        .enumerate()
        .map(|(i, &value)| {
            child(
                &unit_box,
                Vec3::new(4.0 * i as f32, 0.0, 0.0),
                Quat::IDENTITY,
                value,
            )
        })
        .collect();
    let compound = Shape::new_compound(&children).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &compound, RVec3::new(0.0, 0.0, 0.0));
    for (i, &value) in user_data.iter().enumerate() {
        let (hit, _) = hit_below(&world, 4.0 * i as Real, 0.0);
        assert_eq!(
            world.compound_sub_shape(hit.body, hit.sub_shape_id),
            Ok(Some(CompoundSubShape {
                index: i as u32,
                user_data: value
            }))
        );
    }
}

#[test]
fn single_child_compound_keeps_its_user_data() {
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let rotation = quat_about(Z_AXIS, 45.0_f32.to_radians());
    let compound = Shape::new_compound(&[child(&unit_box, Vec3::ZERO, rotation, 7)]).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &compound, RVec3::new(0.0, 0.0, 0.0));
    let (hit, y) = hit_below(&world, 0.0, 0.0);
    assert_eq!(
        world.compound_sub_shape(hit.body, hit.sub_shape_id),
        Ok(Some(CompoundSubShape {
            index: 0,
            user_data: 7
        }))
    );
    // The top edge of the tilted box lies at 0.5 * sqrt(2).
    assert!((y - 0.5 * Real::sqrt(2.0)).abs() <= 1.0e-4, "hit at {y}");
}

#[test]
fn child_pose_is_applied() {
    let slab = Shape::new_box(Vec3::new(1.0, 0.25, 0.5)).unwrap();
    let rotation = quat_about(X_AXIS, 90.0_f32.to_radians());
    let compound =
        Shape::new_compound(&[child(&slab, Vec3::new(0.0, 2.0, 0.0), rotation, 0)]).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &compound, RVec3::new(0.0, 0.0, 0.0));
    // The rotation turns the z extent upright.
    let (_, y) = hit_below(&world, 0.0, 0.0);
    assert!((y - 2.5).abs() <= 1.0e-5, "hit at {y}");
}

#[test]
fn non_compound_hits_have_no_sub_shape() {
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let terrain = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &unit_box, RVec3::new(0.0, 0.0, 0.0));
    add_static(&mut world, &terrain, RVec3::new(10.0, 0.0, 0.0));
    for x in [0.0, 11.0] {
        let (hit, _) = hit_below(&world, x, 0.5);
        assert_eq!(
            world.compound_sub_shape(hit.body, hit.sub_shape_id),
            Ok(None)
        );
    }
    let (hit, _) = hit_below(&world, 0.0, 0.0);
    assert_eq!(unit_box.compound_sub_shape(hit.sub_shape_id), None);
}

#[test]
fn sub_shape_ids_of_other_shapes_never_name_a_missing_child() {
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let children: Vec<_> = (0..3)
        .map(|i| {
            child(
                &unit_box,
                Vec3::new(2.0 * i as f32, 0.0, 0.0),
                Quat::IDENTITY,
                i,
            )
        })
        .collect();
    let compound = Shape::new_compound(&children).unwrap();
    let terrain = Shape::new_height_field(5, &[0.0; 25], &HeightFieldSettings::default()).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &terrain, RVec3::new(-20.0, 0.0, -20.0));
    // Heightfield triangle ids decoded as if they came from the compound.
    for (x, z) in [
        (-19.5, -19.5),
        (-17.25, -18.75),
        (-16.5, -16.5),
        (-18.1, -16.2),
    ] {
        let (hit, _) = hit_below(&world, x, z);
        let resolved = compound.compound_sub_shape(hit.sub_shape_id);
        assert!(resolved.is_none_or(|child| child.index < 3), "{resolved:?}");
    }
}

#[test]
fn foreign_body_is_rejected_by_compound_sub_shape() {
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let compound = Shape::new_compound(&[child(&unit_box, Vec3::ZERO, Quat::IDENTITY, 1)]).unwrap();
    let mut first = world(Vec3::ZERO, 1);
    let second = world(Vec3::ZERO, 1);
    add_static(&mut first, &compound, RVec3::new(0.0, 0.0, 0.0));
    let (hit, _) = hit_below(&first, 0.0, 0.0);
    assert_eq!(
        second.compound_sub_shape(hit.body, hit.sub_shape_id),
        Err(BodyError::WrongWorld(hit.body))
    );
}

#[test]
fn empty_or_invalid_compounds_are_rejected() {
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let invalid = [
        Shape::new_compound(&[]),
        Shape::new_compound(&[child(
            &unit_box,
            Vec3::new(f32::NAN, 0.0, 0.0),
            Quat::IDENTITY,
            0,
        )]),
        Shape::new_compound(&[child(
            &unit_box,
            Vec3::ZERO,
            Quat::from_xyzw(0.0, 0.0, 0.0, 2.0),
            0,
        )]),
    ];
    for result in invalid {
        assert!(matches!(result, Err(ShapeError::InvalidValue(_))));
    }
}

#[test]
fn dynamic_compound_with_rotated_child_is_accepted() {
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let compound = Shape::new_compound(&[
        child(&unit_box, Vec3::ZERO, Quat::IDENTITY, 0),
        child(
            &unit_box,
            Vec3::new(1.0, 0.5, 0.0),
            quat_about(Z_AXIS, 30.0_f32.to_radians()),
            1,
        ),
    ])
    .unwrap();
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let body = world
        .create_body(
            &compound,
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 3.0, 0.0)),
        )
        .unwrap();
    step(&mut world, 60);
    let body = world.body(body).unwrap();
    let position: [Real; 3] = body.position().into();
    assert!(position.iter().all(|value| value.is_finite()));
    assert!(length(body.linear_velocity()).is_finite());
    assert!(length(body.angular_velocity()).is_finite());
}

#[test]
fn sharp_box_edge_ray_hits_the_exact_corner() {
    let half_extent = Vec3::new(1.0, 1.0, 1.0);
    let ray = RayCast::new(RVec3::new(2.0, 2.0, 0.0), Vec3::new(-2.0, -2.0, 0.0));
    let sharp = Shape::new_box_with_convex_radius(half_extent, 0.0).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &sharp, RVec3::new(0.0, 0.0, 0.0));
    let hit = world
        .cast_ray(&ray, &QueryFilter::new())
        .unwrap()
        .expect("the edge is hit");
    assert_eq!(hit.fraction, 0.5);
    assert_eq!(ray.point_at(hit.fraction), RVec3::new(1.0, 1.0, 0.0));

    // Jolt casts rays against the sharp box whatever the convex radius.
    let rounded = Shape::new_box(half_extent).unwrap();
    let mut world = common::world(Vec3::ZERO, 1);
    add_static(&mut world, &rounded, RVec3::new(0.0, 0.0, 0.0));
    let hit = world
        .cast_ray(&ray, &QueryFilter::new())
        .unwrap()
        .expect("the edge is hit");
    assert_eq!(hit.fraction, 0.5);
}

/// Places a resting sphere of radius 0.5 at `(1 + offset, 1 + offset, 0)`, next to an edge of
/// the static box `block` (half extent 1) at the origin, steps 10 times without gravity and
/// returns the sphere's position before and after.
fn sphere_next_to_box_edge(block: &Shape, offset: Real) -> ([Real; 3], [Real; 3]) {
    let ball_shape = Shape::new_sphere(0.5).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, block, RVec3::new(0.0, 0.0, 0.0));
    let start = 1.0 + offset;
    let ball = world
        .create_body(
            &ball_shape,
            &BodySettings::new_dynamic().position(RVec3::new(start, start, 0.0)),
        )
        .unwrap();
    let before = world.body(ball).unwrap().position().into();
    step(&mut world, 10);
    (before, world.body(ball).unwrap().position().into())
}

#[test]
fn convex_radius_rounds_box_edges_for_contacts() {
    let half_extent = Vec3::new(1.0, 1.0, 1.0);
    let sharp = Shape::new_box_with_convex_radius(half_extent, 0.0).unwrap();
    let rounded = Shape::new_box(half_extent).unwrap();
    let edge_distance = |p: [Real; 3]| ((p[0] - 1.0).powi(2) + (p[1] - 1.0).powi(2)).sqrt();

    // The sphere's centre is sqrt(2) * 0.33 = 0.467 from the sharp edge: it penetrates by
    // 0.033, more than Jolt's 0.02 penetration slop, and is pushed out.
    let (before, after) = sphere_next_to_box_edge(&sharp, 0.33);
    assert!(
        edge_distance(after) > edge_distance(before) + 0.01,
        "{before:?} -> {after:?}"
    );
    // Rounded by the default 0.05, the edge is 0.0126 inside the sphere, within the slop, so
    // the sphere stays exactly where it is.
    let (before, after) = sphere_next_to_box_edge(&rounded, 0.33);
    assert_eq!(after.map(Real::to_bits), before.map(Real::to_bits));

    // Jolt uses at most 0.05 of the convex radius for contacts
    // (`ScaleHelpers::ScaleConvexRadius`), so a radius of 0.2 collides like the default.
    let large = Shape::new_box_with_convex_radius(half_extent, 0.2).unwrap();
    let (_, after_default) = sphere_next_to_box_edge(&rounded, 0.32);
    let (_, after_large) = sphere_next_to_box_edge(&large, 0.32);
    let (_, after_sharp) = sphere_next_to_box_edge(&sharp, 0.32);
    assert_eq!(
        after_large.map(Real::to_bits),
        after_default.map(Real::to_bits)
    );
    assert_ne!(
        after_sharp.map(Real::to_bits),
        after_default.map(Real::to_bits)
    );
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn shapes_are_shared_across_bodies_and_worlds() {
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let pillar = Shape::new_cylinder(1.0, 0.3).unwrap();
    let compound = Shape::new_compound(&[
        child(&unit_box, Vec3::ZERO, Quat::IDENTITY, 1),
        child(&pillar, Vec3::new(2.0, 0.5, 0.0), Quat::IDENTITY, 2),
    ])
    .unwrap();
    drop((unit_box, pillar));
    let samples: Vec<f32> = (0..17 * 17).map(|i| 0.05 * (i % 7) as f32).collect();
    let terrain_settings = HeightFieldSettings::default().offset(Vec3::new(-8.0, -2.0, -8.0));
    let terrain = Shape::new_height_field(17, &samples, &terrain_settings).unwrap();

    let mut world_a = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let mut world_b = world(Vec3::new(0.0, -9.81, 0.0), 4);
    for world in [&mut world_a, &mut world_b] {
        add_static(world, &terrain, RVec3::new(0.0, 0.0, 0.0));
        add_static(world, &compound, RVec3::new(0.0, 0.0, 0.0));
    }
    add_static(&mut world_a, &compound, RVec3::new(-5.0, 0.0, 5.0));
    drop((compound, terrain));
    step(&mut world_a, 10);
    step(&mut world_b, 10);

    let rays: Vec<RayCast> = [(0.0, 0.0), (2.0, 0.0), (3.3, -4.1), (-6.2, 1.7), (5.5, 6.5)]
        .into_iter()
        .map(|(x, z)| down_from(x, 10.0, z, 20.0))
        .collect();
    let cast_all = |world: &PhysicsWorld| -> Vec<u32> {
        rays.iter()
            .map(|&ray| {
                let hit = world
                    .cast_ray(&ray, &QueryFilter::new())
                    .unwrap()
                    .expect("every ray hits");
                hit.fraction.to_bits()
            })
            .collect()
    };
    let expected = cast_all(&world_b);
    assert_eq!(cast_all(&world_a), expected);

    std::thread::scope(|scope| {
        scope.spawn(|| step(&mut world_a, 10));
        let reader = scope.spawn(|| cast_all(&world_b));
        assert_eq!(reader.join().unwrap(), expected);
    });
    drop(world_a);
    assert_eq!(cast_all(&world_b), expected);
}

/// Roll in degrees after 300 ticks of a box with half extents (0.9, 0.3, 2.0) and mass 1500,
/// optionally with its centre of mass moved by `offset`, released on the floor tilted by
/// `tilt_degrees` about z with its lower long edge just above the floor.
fn roll_after_release(offset: Option<Vec3>, tilt_degrees: f32) -> f32 {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let plain = Shape::new_box(Vec3::new(0.9, 0.3, 2.0)).unwrap();
    let shape = match offset {
        Some(offset) => Shape::new_offset_center_of_mass(&plain, offset).unwrap(),
        None => plain,
    };
    let tilt = tilt_degrees.to_radians();
    let lowest = 0.9 * tilt.sin() + 0.3 * tilt.cos();
    let body = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, (lowest + 0.01) as Real, 0.0))
                .rotation(quat_about(Vec3::new(0.0, 0.0, 1.0), tilt))
                .mass(1500.0),
        )
        .unwrap();
    step(&mut world, 300);
    let up = math::rotate(world.body(body).unwrap().rotation(), [0.0, 1.0, 0.0]);
    up[1].clamp(-1.0, 1.0).acos().to_degrees() as f32
}

#[test]
fn low_center_of_mass_rights_a_tilted_box() {
    // The plain box tips over its long edge once its centre of mass passes above that edge, at
    // atan(0.9 / 0.3) = 71.6 degrees by geometry; with the centre of mass moved to the bottom
    // face it would need 90 degrees. Measured by sweeping the release tilt in 1 degree steps:
    // the plain box no longer comes back level from 71 degrees on, the offset box only from
    // 89.5. Each case sits
    // more than 10 degrees from the tipping angle that decides it.
    let low = Some(Vec3::new(0.0, -0.3, 0.0));
    let righted = roll_after_release(low, 75.0);
    assert!(righted < 5.0, "offset box ends at roll {righted}");
    let fallen = roll_after_release(None, 85.0);
    assert!(fallen > 60.0, "plain box ends at roll {fallen}");
}

#[test]
fn height_field_material_lists_are_validated() {
    let materials: Vec<PhysicsMaterial> =
        (0..257).map(|i| PhysicsMaterial::new(i).unwrap()).collect();
    let refs: Vec<&PhysicsMaterial> = materials.iter().collect();
    let samples = vec![0.0; 9];
    let field = |list: &[&PhysicsMaterial], indices: &[u8]| {
        let settings = HeightFieldSettings::default().materials(list, indices);
        Shape::new_height_field(3, &samples, &settings).err()
    };
    let invalid = |error: Option<ShapeError>| matches!(error, Some(ShapeError::InvalidValue(_)));
    assert!(invalid(field(&[], &[0; 4])), "no material");
    assert!(invalid(field(&refs, &[0; 4])), "257 materials");
    assert_eq!(field(&refs[..256], &[255, 0, 0, 0]), None, "256 materials");
    assert!(invalid(field(&refs[..2], &[0; 3])), "short indices");
    assert!(invalid(field(&refs[..2], &[0; 5])), "long indices");
    assert!(invalid(field(&refs[..2], &[0, 1, 2, 0])), "index == len");
    assert_eq!(field(&refs[..2], &[0, 1, 1, 0]), None);
    // The rules of `new_height_field` come first.
    let settings = HeightFieldSettings::default().materials(&refs[..1], &[]);
    let error = Shape::new_height_field(1, &[0.0], &settings).err();
    assert!(matches!(error, Some(ShapeError::InvalidValue(_))));
}

#[test]
fn shapes_with_materials_collide_like_their_plain_siblings() {
    let material = PhysicsMaterial::new(3).unwrap();
    let half = Vec3::new(1.0, 0.5, 1.0);
    let pairs = [
        (
            Shape::new_box(half).unwrap(),
            Shape::new_box_with_material(half, 0.05, &material).unwrap(),
        ),
        (
            Shape::new_sphere(0.8).unwrap(),
            Shape::new_sphere_with_material(0.8, &material).unwrap(),
        ),
        (
            Shape::new_capsule(0.4, 0.6).unwrap(),
            Shape::new_capsule_with_material(0.4, 0.6, &material).unwrap(),
        ),
        (
            Shape::new_cylinder(0.7, 0.6).unwrap(),
            Shape::new_cylinder_with_material(0.7, 0.6, 0.05, &material).unwrap(),
        ),
    ];
    for (plain, with_material) in &pairs {
        let mut distances = Vec::new();
        for shape in [plain, with_material] {
            let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
            add_static(&mut world, shape, RVec3::ZERO);
            let hit = world
                .cast_ray(&down_from(0.1, 5.0, 0.2, 10.0), &QueryFilter::new())
                .unwrap()
                .expect("the ray hits the shape");
            distances.push(hit.distance);
        }
        assert_eq!(distances[0], distances[1]);
    }
}
