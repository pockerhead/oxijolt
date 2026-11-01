//! Point queries on shapes and in the world: Jolt's containment rule per shape, compounds and
//! decorators, filters, order and validation.

mod common;

use common::events::add_cloth;
use common::soft_body::sphere;
use common::*;
use oxijolt::*;

const UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);

fn inside(shape: &Shape, point: Vec3) -> bool {
    !shape.collide_point(point).unwrap().is_empty()
}

fn next_up(value: f32) -> f32 {
    f32::from_bits(value.to_bits() + 1)
}

#[test]
fn boxes_spheres_and_planes_have_exact_boundaries() {
    let block = Shape::new_box(Vec3::new(1.0, 0.5, 0.25)).unwrap();
    for point in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.5, 0.25),
        Vec3::new(-1.0, -0.5, -0.25),
    ] {
        assert!(inside(&block, point), "{point:?}");
    }
    assert!(!inside(&block, Vec3::new(next_up(1.0), 0.0, 0.0)));
    assert!(!inside(&block, Vec3::new(0.0, -next_up(0.5), 0.0)));

    let ball = Shape::new_sphere(0.5).unwrap();
    assert!(inside(&ball, Vec3::new(0.0, 0.0, 0.5)));
    assert!(!inside(&ball, Vec3::new(0.0, 0.0, next_up(0.5))));

    // The plane's surface is not inside; anything strictly behind it is, beyond its half extent
    // too.
    let plane = Shape::new_plane(UP, 0.0, 10.0).unwrap();
    assert!(!inside(&plane, Vec3::ZERO));
    assert!(inside(&plane, Vec3::new(0.0, -1.0e-6, 0.0)));
    assert!(inside(&plane, Vec3::new(1000.0, -1.0, 0.0)));
    assert!(!inside(&plane, Vec3::new(0.0, 1.0e-6, 0.0)));
}

#[test]
fn rounded_and_hull_shapes_hold_within_a_tenth_of_a_millimetre() {
    let near = |shape: &Shape, surface: Vec3, outward: Vec3| {
        let at = |d: f32| {
            Vec3::new(
                surface.x + outward.x * d,
                surface.y + outward.y * d,
                surface.z + outward.z * d,
            )
        };
        assert!(inside(shape, at(-1.0e-4)), "{surface:?} inside");
        assert!(!inside(shape, at(1.0e-4)), "{surface:?} outside");
    };
    let x = Vec3::new(1.0, 0.0, 0.0);
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    near(&capsule, Vec3::new(0.3, 0.0, 0.0), x);
    near(&capsule, Vec3::new(0.0, 0.8, 0.0), UP);
    let cylinder = Shape::new_cylinder(0.5, 0.3).unwrap();
    near(&cylinder, Vec3::new(0.3, 0.0, 0.0), x);
    near(&cylinder, Vec3::new(0.0, 0.5, 0.0), UP);
    // A sharp cylinder edge: just inside both the side and the top.
    assert!(inside(&cylinder, Vec3::new(0.2999, 0.4999, 0.0)));
    let tapered = Shape::new_tapered_cylinder_with_convex_radius(0.5, 0.2, 0.4, 0.05).unwrap();
    near(&tapered, Vec3::new(0.0, 0.5, 0.0), UP);
    near(&tapered, Vec3::new(0.3, 0.0, 0.0), x);
    let hull = Shape::new_convex_hull_with_convex_radius(
        &common::meshes::box_corners(Vec3::new(1.0, 0.5, 0.5)),
        0.05,
    )
    .unwrap();
    near(&hull, Vec3::new(1.0, 0.1, 0.2), x);
    near(&hull, Vec3::new(0.3, 0.5, -0.1), UP);

    let tapered_capsule = Shape::new_tapered_capsule(0.5, 0.2, 0.4).unwrap();
    assert!(inside(&tapered_capsule, Vec3::new(0.0, 0.0, 0.0)));
    assert!(inside(&tapered_capsule, Vec3::new(0.0, -0.7, 0.0)));
    assert!(!inside(&tapered_capsule, Vec3::new(0.5, 0.0, 0.0)));
    assert!(!inside(&tapered_capsule, Vec3::new(0.0, 1.0, 0.0)));
}

/// The 12 triangles of the cube `[-0.5, 0.5]^3` wound outward, the top two last but two, the
/// bottom two last.
fn cube_mesh() -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let vertices = common::meshes::box_corners(Vec3::new(0.5, 0.5, 0.5));
    // Corner index bits as `box_corners` lists them: the index of (x, y, z) with each sign.
    let index = |x: i32, y: i32, z: i32| {
        vertices
            .iter()
            .position(|v| v.x == 0.5 * x as f32 && v.y == 0.5 * y as f32 && v.z == 0.5 * z as f32)
            .unwrap() as u32
    };
    let quad = |a: u32, b: u32, c: u32, d: u32| [[a, b, c], [a, c, d]];
    let mut triangles = Vec::new();
    triangles.extend(quad(
        index(1, -1, -1),
        index(1, 1, -1),
        index(1, 1, 1),
        index(1, -1, 1),
    ));
    triangles.extend(quad(
        index(-1, -1, -1),
        index(-1, -1, 1),
        index(-1, 1, 1),
        index(-1, 1, -1),
    ));
    triangles.extend(quad(
        index(-1, -1, 1),
        index(1, -1, 1),
        index(1, 1, 1),
        index(-1, 1, 1),
    ));
    triangles.extend(quad(
        index(-1, -1, -1),
        index(-1, 1, -1),
        index(1, 1, -1),
        index(1, -1, -1),
    ));
    triangles.extend(quad(
        index(-1, 1, -1),
        index(-1, 1, 1),
        index(1, 1, 1),
        index(1, 1, -1),
    ));
    triangles.extend(quad(
        index(-1, -1, -1),
        index(1, -1, -1),
        index(1, -1, 1),
        index(-1, -1, 1),
    ));
    (vertices, triangles)
}

fn mesh(vertices: &[Vec3], triangles: &[[u32; 3]]) -> Shape {
    let (shape, dropped) = Shape::new_mesh(vertices, triangles).unwrap();
    assert!(dropped.is_empty());
    shape
}

#[test]
fn meshes_count_the_triangles_above_the_point() {
    let (vertices, triangles) = cube_mesh();
    let closed = mesh(&vertices, &triangles);
    // Off every edge and diagonal, so the upward ray crosses one triangle's interior.
    let interior = Vec3::new(0.2, 0.0, 0.1);
    assert_eq!(closed.collide_point(interior).unwrap().len(), 1);
    assert!(inside(&closed, Vec3::new(-0.3, 0.35, -0.15)));
    assert!(!inside(&closed, Vec3::new(0.2, 0.6, 0.1)));
    assert!(!inside(&closed, Vec3::new(0.7, 0.0, 0.1)));
    assert!(!inside(&closed, Vec3::new(0.2, -0.6, 0.1)));
    // The ray from the centre runs along the top face's diagonal and crosses both of its
    // triangles: an even count, so the centre of the closed cube is reported outside.
    assert!(!inside(&closed, Vec3::ZERO));

    // Without its top the ray crosses nothing: the point is outside.
    let open_top = mesh(
        &vertices,
        &triangles[..8]
            .iter()
            .chain(&triangles[10..])
            .copied()
            .collect::<Vec<_>>(),
    );
    assert!(!inside(&open_top, interior));
    // Without its bottom the ray still crosses the top: the point is inside.
    let open_bottom = mesh(&vertices, &triangles[..10]);
    assert!(inside(&open_bottom, interior));

    // A flat quad has no height: a point under it is outside its bounds.
    let quad = mesh(
        &[
            Vec3::new(-1.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(-1.0, 0.0, 1.0),
        ],
        &[[0, 2, 1], [0, 3, 2]],
    );
    assert!(!inside(&quad, Vec3::new(0.2, -0.5, 0.1)));
}

#[test]
fn heightfields_contain_nothing() {
    let terrain = flat_height_field();
    for y in [-1.0, -1.0e-3, 0.0] {
        assert!(!inside(&terrain, Vec3::new(0.2, y, 0.3)), "{y}");
    }
}

/// The ids and compound children `world` reports for `point`.
fn world_hits(
    world: &PhysicsWorld,
    point: RVec3,
    filter: &QueryFilter<'_>,
) -> Vec<CollidePointHit> {
    world.collide_point(point, filter).unwrap()
}

#[test]
fn soft_bodies_are_tested_by_their_faces() {
    let mut world = world(Vec3::ZERO, 1);
    let (vertices, faces) = sphere(0.5, 8, 12);
    let shared = SoftBodySharedSettings::builder(vertices, faces)
        .build()
        .unwrap();
    let ball = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default().position(RVec3::new(0.0, 2.0, 0.0)),
        )
        .unwrap();
    let filter = QueryFilter::new();
    let centre = world_hits(&world, RVec3::new(0.01, 2.02, 0.03), &filter);
    assert_eq!(centre.len(), 1);
    assert_eq!(centre[0].body, ball);
    assert!(world_hits(&world, RVec3::new(0.0, 2.6, 0.0), &filter).is_empty());

    // A flat cloth encloses nothing: a point in its plane, inside its bounds, crosses no face.
    let cloth = add_cloth(&mut world, RVec3::new(5.0, 1.0, 0.0), Quat::IDENTITY);
    assert!(world
        .collide_point(RVec3::new(5.31, 1.0, 0.27), &filter)
        .unwrap()
        .iter()
        .all(|hit| hit.body != cloth));
}

#[test]
fn compounds_report_every_child_that_holds_the_point() {
    let cube = cube_shape();
    let overlapping = Shape::new_compound(&[
        child(&cube, Vec3::new(0.0, 0.0, 0.0), 10),
        child(&cube, Vec3::new(0.6, 0.0, 0.0), 20),
        child(&cube, Vec3::new(3.0, 0.0, 0.0), 30),
    ])
    .unwrap();
    let ids = overlapping.collide_point(Vec3::new(0.3, 0.1, 0.0)).unwrap();
    assert_eq!(ids.len(), 2);
    let mut children: Vec<_> = ids
        .iter()
        .map(|&id| overlapping.compound_sub_shape(id).unwrap())
        .collect();
    children.sort_by_key(|child| child.index);
    assert_eq!(
        children,
        [
            CompoundSubShape {
                index: 0,
                user_data: 10
            },
            CompoundSubShape {
                index: 1,
                user_data: 20
            }
        ]
    );

    let mut world = world(Vec3::ZERO, 1);
    let body = world
        .create_body(
            &overlapping,
            &BodySettings::new_static().position(RVec3::new(10.0, 0.0, 0.0)),
        )
        .unwrap();
    let hits = world_hits(&world, RVec3::new(10.3, 0.1, 0.0), &QueryFilter::new());
    assert_eq!(hits.len(), 2);
    assert!(hits.iter().all(|hit| hit.body == body));
    assert!(hits[0].sub_shape_id < hits[1].sub_shape_id);
    let mut found: Vec<u32> = hits
        .iter()
        .map(|hit| hit.compound_child.unwrap().user_data)
        .collect();
    found.sort_unstable();
    assert_eq!(found, [10, 20]);
}

#[test]
fn shape_points_are_in_the_shapes_own_frame() {
    // A hull far off its origin: its centre of mass is at x = 2.5.
    let offset_hull = Shape::new_convex_hull_with_convex_radius(
        &common::meshes::box_corners(Vec3::new(0.5, 0.5, 0.5))
            .iter()
            .map(|c| Vec3::new(c.x + 2.5, c.y, c.z))
            .collect::<Vec<_>>(),
        0.05,
    )
    .unwrap();
    assert!(inside(&offset_hull, Vec3::new(2.9, 0.0, 0.0)));
    assert!(!inside(&offset_hull, Vec3::new(0.0, 0.0, 0.0)));
    assert!(!inside(&offset_hull, Vec3::new(-2.5, 0.0, 0.0)));

    // A compound whose centre of mass is off its origin, inside an offset centre of mass.
    let cube = cube_shape();
    let ball = Shape::new_sphere(0.5).unwrap();
    let inner = Shape::new_compound(&[
        child(&cube, Vec3::new(1.0, 0.0, 0.0), 1),
        child(&ball, Vec3::new(1.0, 2.0, 0.0), 2),
    ])
    .unwrap();
    let nested = Shape::new_compound(&[
        child(&inner, Vec3::new(0.0, 0.0, 3.0), 3),
        child(&cube, Vec3::new(-2.0, 0.0, 0.0), 4),
    ])
    .unwrap();
    let decorated = Shape::new_offset_center_of_mass(&nested, Vec3::new(0.3, -0.2, 0.1)).unwrap();
    for shape in [&nested, &decorated] {
        assert!(inside(shape, Vec3::new(1.2, 0.1, 3.1)));
        assert!(inside(shape, Vec3::new(1.0, 2.3, 3.0)));
        assert!(inside(shape, Vec3::new(-2.3, 0.0, 0.0)));
        assert!(!inside(shape, Vec3::new(1.0, 1.0, 3.0)));
        assert!(!inside(shape, Vec3::new(0.0, 0.0, 0.0)));
    }

    // Mirrored along x, the offset hull sits at x = -2.5.
    let mirrored = Shape::new_scaled(&offset_hull, Vec3::new(-1.0, 1.0, 1.0)).unwrap();
    assert!(inside(&mirrored, Vec3::new(-2.9, 0.0, 0.0)));
    assert!(!inside(&mirrored, Vec3::new(2.9, 0.0, 0.0)));
}

#[test]
fn a_plane_child_is_bounded_by_its_half_extent() {
    let plane = Shape::new_plane(UP, 0.0, 2.0).unwrap();
    let cube = cube_shape();
    let compound = Shape::new_compound(&[
        child(&plane, Vec3::ZERO, 1),
        child(&cube, Vec3::new(0.0, 3.0, 0.0), 2),
    ])
    .unwrap();
    let local = Vec3::new(10.0, -1.0, 0.0);
    assert!(inside(&plane, local));
    assert!(!inside(&compound, local));
    assert!(inside(&compound, Vec3::new(1.0, -1.0, 0.0)));

    // In the world a plane body is a broad-phase candidate only within its bounds.
    let mut world = world(Vec3::ZERO, 1);
    let floor = world
        .create_body(&plane, &BodySettings::new_static())
        .unwrap();
    let filter = QueryFilter::new();
    assert_eq!(
        world_hits(&world, RVec3::new(1.0, -1.0, 0.0), &filter)[0].body,
        floor
    );
    assert!(world_hits(&world, RVec3::new(10.0, -1.0, 0.0), &filter).is_empty());
}

#[test]
fn world_queries_apply_every_filter_part_and_sort_by_body() {
    let (mut world, layers) = five_layer_world();
    let [terrain, chunk, _, item, _] = layers;
    let cube = cube_shape();
    // Two children: a power of two, where the empty id would decode to the last child.
    let compound = Shape::new_compound(&[
        child(&cube, Vec3::ZERO, Groups::STRUCTURE),
        child(&cube, Vec3::new(0.2, 0.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    let ground = add_static_in(&mut world, &cube, RVec3::ZERO, terrain);
    let structure = add_static_in(&mut world, &compound, RVec3::ZERO, chunk);
    let crate_body = world
        .create_body(
            &cube,
            &BodySettings::new_dynamic()
                .object_layer(item)
                .activation(Activation::DontActivate),
        )
        .unwrap();
    assert!(world.body(crate_body).unwrap().is_sleeping());
    let at = RVec3::new(0.1, 0.0, 0.0);

    let all = world_hits(&world, at, &QueryFilter::new());
    let bodies: Vec<BodyId> = all.iter().map(|hit| hit.body).collect();
    assert_eq!(bodies, [ground, structure, structure, crate_body]);
    assert_eq!(all[0].object_layer, terrain);
    assert_eq!(all[3].object_layer, item);
    assert!(all[1].sub_shape_id < all[2].sub_shape_id);

    // Each filter keeps exactly the unfiltered hits it lets through.
    let chunk_only = [chunk];
    let hits = world_hits(&world, at, &QueryFilter::new().object_layers(&chunk_only));
    assert_eq!(hits, [all[1], all[2]]);
    let hits = world_hits(
        &world,
        at,
        &QueryFilter::new().child_groups(1 << Groups::FEATURE),
    );
    let feature = if all[1].compound_child.unwrap().user_data == Groups::FEATURE {
        all[1]
    } else {
        all[2]
    };
    assert_eq!(feature.compound_child.unwrap().user_data, Groups::FEATURE);
    // Bodies that are not compounds are not filtered by child.
    assert_eq!(hits, [all[0], feature, all[3]]);
    let hits = world_hits(&world, at, &QueryFilter::new().exclude_body(ground));
    assert_eq!(hits, all[1..]);

    world.remove_body(crate_body).unwrap();
    assert!(world_hits(&world, at, &QueryFilter::new())
        .iter()
        .all(|hit| hit.body != crate_body));
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn world_queries_read_from_many_threads() {
    let mut world = world(Vec3::ZERO, 1);
    let ids: Vec<BodyId> = (0..8)
        .map(|i| add_cube(&mut world, RVec3::new(2.0 * i as Real, 0.0, 0.0)))
        .collect();
    let world = &world;
    std::thread::scope(|scope| {
        for (i, &id) in ids.iter().enumerate() {
            scope.spawn(move || {
                for _ in 0..50 {
                    let hits = world
                        .collide_point(RVec3::new(2.0 * i as Real, 0.1, 0.0), &QueryFilter::new())
                        .unwrap();
                    assert_eq!(hits.len(), 1);
                    assert_eq!(hits[0].body, id);
                }
            });
        }
    });
}

#[test]
fn far_points_find_small_bodies() {
    // 4 km out, where an `f32` step is about 0.5 mm.
    let mut world = world(Vec3::ZERO, 1);
    let pebble = Shape::new_box(Vec3::new(0.05, 0.05, 0.05)).unwrap();
    let far = RVec3::new(4000.25, -3999.5, 4000.0);
    let id = world
        .create_body(&pebble, &BodySettings::new_static().position(far))
        .unwrap();
    let filter = QueryFilter::new();
    let near = RVec3::new(far.x + 0.04, far.y - 0.04, far.z + 0.01);
    assert_eq!(world_hits(&world, near, &filter)[0].body, id);
    let beside = RVec3::new(far.x + 0.06, far.y, far.z);
    assert!(world_hits(&world, beside, &filter).is_empty());
}

#[test]
fn invalid_points_and_filters_are_rejected() {
    let mut world = world(Vec3::ZERO, 1);
    add_cube(&mut world, RVec3::ZERO);
    let filter = QueryFilter::new();
    let beyond = limits::MAX_POSITION * 1.5;
    for point in [
        RVec3::new(Real::NAN, 0.0, 0.0),
        RVec3::new(0.0, Real::INFINITY, 0.0),
        RVec3::new(0.0, 0.0, beyond),
    ] {
        assert!(matches!(
            world.collide_point(point, &filter),
            Err(QueryError::InvalidValue(_))
        ));
    }
    let mut other = common::world(Vec3::ZERO, 1);
    let foreign = add_cube(&mut other, RVec3::ZERO);
    assert!(world
        .collide_point(RVec3::ZERO, &QueryFilter::new().exclude_body(foreign))
        .is_err());
    let unknown = [ObjectLayer::new(99)];
    assert!(world
        .collide_point(RVec3::ZERO, &QueryFilter::new().object_layers(&unknown))
        .is_err());

    let cube = cube_shape();
    let extent = limits::MAX_SHAPE_EXTENT;
    for point in [
        Vec3::new(f32::NAN, 0.0, 0.0),
        Vec3::new(0.0, next_up(extent), 0.0),
        Vec3::new(0.0, 0.0, -f32::INFINITY),
    ] {
        assert!(matches!(
            cube.collide_point(point),
            Err(QueryError::InvalidValue(_))
        ));
    }
    assert!(cube
        .collide_point(Vec3::new(extent, -extent, extent))
        .unwrap()
        .is_empty());
}
