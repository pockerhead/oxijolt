//! `PhysicsWorld::cast_shape_all`: every hit of a shape cast, sorted, with the rules and filters
//! of `cast_shape`.

mod common;

use common::*;
use oxijolt::*;

const ALL: QueryFilter<'static> = QueryFilter::new();

/// Jolt's closest-hit measure for a cast without target distance: the fraction, or minus the
/// depth at fraction 0.
fn closeness(hit: &ShapeCastHit) -> f32 {
    if hit.fraction > 0.0 {
        hit.fraction
    } else {
        -hit.penetration_depth
    }
}

/// Every hit of `cast` (without target distance), checked against `cast_shape`: its hit is one of the hits that share the
/// first hit's measure (the first hit itself when no other shares it), and the hits are sorted by
/// measure, body id and sub-shape id.
fn all_hits(
    world: &PhysicsWorld,
    cast: &ShapeCast<'_>,
    filter: &QueryFilter<'_>,
) -> Vec<ShapeCastHit> {
    let hits = world.cast_shape_all(cast, filter).unwrap();
    let closest = world.cast_shape(cast, filter).unwrap();
    let key = |hit: &ShapeCastHit| (closeness(hit), hit.body, hit.sub_shape_id);
    assert_eq!(
        hits.first().map(|hit| key(hit).0),
        closest.map(|hit| key(&hit).0)
    );
    if let Some(closest) = closest {
        let mut tied = hits.iter().take_while(|hit| key(hit).0 == key(&closest).0);
        assert!(
            tied.any(|hit| *hit == closest),
            "{closest:?} not in {hits:?}"
        );
    }
    for pair in hits.windows(2) {
        assert!(key(&pair[0]) <= key(&pair[1]), "{hits:?}");
    }
    hits
}

const BALL_RADIUS: f32 = 0.25;

fn ball() -> Shape {
    Shape::new_sphere(BALL_RADIUS).unwrap()
}

/// A cast of `shape` along +x from the origin, 10 m long.
fn along_x(shape: &Shape) -> ShapeCast<'_> {
    ShapeCast::new(
        shape,
        RVec3::ZERO,
        Quat::IDENTITY,
        Vec3::new(10.0, 0.0, 0.0),
    )
}

/// Unit boxes centred at x 8, 2, 6, 4 (created in that order) in the terrain, chunk, feature
/// and item layers; returned by x.
fn row_of_boxes() -> (PhysicsWorld, [BodyId; 4], [ObjectLayer; 5]) {
    let (mut world, layers) = five_layer_world();
    let [terrain, chunk, feature, item, _] = layers;
    let unit_box = cube_shape();
    let mut at =
        |x: Real, layer| add_static_in(&mut world, &unit_box, RVec3::new(x, 0.0, 0.0), layer);
    let at_8 = at(8.0, item);
    let at_2 = at(2.0, terrain);
    let at_6 = at(6.0, feature);
    let at_4 = at(4.0, chunk);
    (world, [at_2, at_4, at_6, at_8], layers)
}

#[test]
fn a_row_of_boxes_gives_every_hit_in_fraction_order() {
    let (world, boxes, [terrain, chunk, feature, item, _]) = row_of_boxes();
    let shape = ball();
    let hits = all_hits(&world, &along_x(&shape), &ALL);
    let bodies: Vec<BodyId> = hits.iter().map(|hit| hit.body).collect();
    assert_eq!(bodies, boxes);
    for (hit, face) in hits.iter().zip([1.5, 3.5, 5.5, 7.5]) {
        let distance = face - BALL_RADIUS;
        assert!((hit.distance - distance).abs() <= 1.0e-3, "{hit:?}");
        assert!((hit.fraction - distance / 10.0).abs() <= 1.0e-4, "{hit:?}");
        assert!((hit.point.x - Real::from(face)).abs() <= 1.0e-3, "{hit:?}");
        assert!(hit.normal.x < -0.99, "{hit:?}");
        assert_eq!(hit.compound_child, None);
    }
    let layers: Vec<ObjectLayer> = hits.iter().map(|hit| hit.object_layer).collect();
    assert_eq!(layers, [terrain, chunk, feature, item]);

    // A cast that misses everything, and one that ends before the second box.
    let above = ShapeCast::new(
        &shape,
        RVec3::new(0.0, 3.0, 0.0),
        Quat::IDENTITY,
        Vec3::new(10.0, 0.0, 0.0),
    );
    assert!(all_hits(&world, &above, &ALL).is_empty());
    let short = ShapeCast::new(
        &shape,
        RVec3::ZERO,
        Quat::IDENTITY,
        Vec3::new(3.0, 0.0, 0.0),
    );
    let hits = all_hits(&world, &short, &ALL);
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].body, boxes[0]);
}

#[test]
fn start_overlaps_come_deepest_first() {
    let mut world = world(Vec3::ZERO, 1);
    // Wide slabs whose tops reach 0.1, 0.3 and 0.2 m into the bottom of a unit sphere at the
    // origin, created in that order, and a fourth slab further down that the cast reaches.
    let slab = Shape::new_box(Vec3::new(5.0, 0.05, 5.0)).unwrap();
    let mut slab_with_top_at = |top: Real| {
        world
            .create_body(
                &slab,
                &BodySettings::new_static().position(RVec3::new(0.0, top - 0.05, 0.0)),
            )
            .unwrap()
    };
    let shallow = slab_with_top_at(-0.9);
    let deepest = slab_with_top_at(-0.7);
    let middle = slab_with_top_at(-0.8);
    let below = slab_with_top_at(-2.0);
    let sphere = Shape::new_sphere(1.0).unwrap();
    let cast = ShapeCast::new(
        &sphere,
        RVec3::ZERO,
        Quat::IDENTITY,
        Vec3::new(0.0, -2.0, 0.0),
    )
    .return_deepest_point(true);
    let hits = all_hits(&world, &cast, &ALL);
    let bodies: Vec<BodyId> = hits.iter().map(|hit| hit.body).collect();
    assert_eq!(bodies, [deepest, middle, shallow, below]);
    for (hit, depth) in hits.iter().zip([0.3, 0.2, 0.1]) {
        assert_eq!(hit.fraction, 0.0, "{hit:?}");
        assert!((hit.penetration_depth - depth).abs() <= 1.0e-3, "{hit:?}");
        assert!(hit.normal.y > 0.99, "{hit:?}");
    }
    assert!((hits[3].distance - 1.0).abs() <= 1.0e-3, "{hits:?}");
    // No tie at the front: `cast_shape` gives the deepest overlap.
    assert_eq!(world.cast_shape(&cast, &ALL).unwrap(), Some(hits[0]));
}

#[test]
fn the_first_hit_is_cast_shapes_hit() {
    let (world, boxes, _) = row_of_boxes();
    let capsule = Shape::new_capsule(0.5, 0.2).unwrap();
    let tilted = quat_about(Vec3::new(0.0, 0.0, 1.0), 30.0_f32.to_radians());
    let casts = [
        ShapeCast::new(
            &capsule,
            RVec3::new(0.0, 0.3, 0.1),
            tilted,
            Vec3::new(10.0, 0.0, 0.0),
        ),
        ShapeCast::new(
            &capsule,
            RVec3::new(10.0, 0.0, 0.0),
            tilted,
            Vec3::new(-10.0, 0.0, 0.0),
        )
        .target_distance(0.1),
    ];
    for (cast, first) in casts.iter().zip([boxes[0], boxes[3]]) {
        let hits = world.cast_shape_all(cast, &ALL).unwrap();
        assert_eq!(hits.len(), 4, "{hits:?}");
        assert_eq!(hits[0].body, first);
        assert_eq!(world.cast_shape(cast, &ALL).unwrap(), Some(hits[0]));
    }
}

#[test]
fn filters_apply_to_every_hit() {
    let (world, [at_2, at_4, at_6, at_8], [terrain, chunk, feature, ..]) = row_of_boxes();
    let shape = ball();
    let cast = along_x(&shape);
    let bodies = |filter: &QueryFilter<'_>| -> Vec<BodyId> {
        all_hits(&world, &cast, filter)
            .iter()
            .map(|hit| hit.body)
            .collect()
    };
    let three = [terrain, chunk, feature];
    assert_eq!(
        bodies(&QueryFilter::new().object_layers(&three)),
        [at_2, at_4, at_6]
    );
    assert_eq!(
        bodies(&QueryFilter::new().exclude_body(at_2)),
        [at_4, at_6, at_8]
    );
    let not_at_6 = |body: BodyId| body != at_6;
    assert_eq!(
        bodies(&QueryFilter::new().body_filter(&not_at_6)),
        [at_2, at_4, at_8]
    );
    let filter = QueryFilter::new()
        .object_layers(&three)
        .exclude_body(at_2)
        .body_filter(&not_at_6);
    assert_eq!(bodies(&filter), [at_4]);
}

#[test]
fn coincident_bodies_are_ordered_by_body_id() {
    let mut world = world(Vec3::ZERO, 1);
    let shape = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    let boxes: Vec<BodyId> = (0..4)
        .map(|_| {
            world
                .create_body(&shape, &BodySettings::new_static())
                .unwrap()
        })
        .collect();
    let sphere = ball();
    let cast = ShapeCast::new(
        &sphere,
        RVec3::new(-4.0, 0.0, 0.0),
        Quat::IDENTITY,
        Vec3::new(8.0, 0.0, 0.0),
    );
    for optimized in [false, true] {
        if optimized {
            world.optimize_broad_phase();
        }
        // `all_hits` checks that `cast_shape`'s hit is one of the tied ones; which one it is
        // follows Jolt's traversal.
        let hits = all_hits(&world, &cast, &ALL);
        let bodies: Vec<BodyId> = hits.iter().map(|hit| hit.body).collect();
        assert_eq!(bodies, boxes, "optimized: {optimized}");
        assert!(
            hits.iter().all(|hit| hit.fraction == hits[0].fraction),
            "{hits:?}"
        );
        assert!((hits[0].distance - 2.75).abs() <= 1.0e-3, "{hits:?}");
    }
}

#[test]
fn leaves_of_a_compound_query_shape_that_tie_are_ordered_by_leaf() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    // Child 0 at x +1, child 1 at x -1: both reach the floor after the same fraction.
    let sphere = ball();
    let pair = Shape::new_compound(&[
        child(&sphere, Vec3::new(1.0, 0.0, 0.0), 0),
        child(&sphere, Vec3::new(-1.0, 0.0, 0.0), 0),
    ])
    .unwrap();
    let cast = ShapeCast::new(
        &pair,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
        Vec3::new(0.0, -4.0, 0.0),
    );
    let hits = all_hits(&world, &cast, &ALL);
    assert_eq!(hits.len(), 2, "{hits:?}");
    assert!(hits.iter().all(|hit| hit.body == floor));
    assert_eq!(hits[0].fraction, hits[1].fraction, "{hits:?}");
    assert_eq!(hits[0].sub_shape_id, hits[1].sub_shape_id, "{hits:?}");
    // The query leaves differ: child 0 comes first.
    assert!(hits[0].point.x > 0.9 && hits[1].point.x < -0.9, "{hits:?}");
}

#[test]
fn a_cast_through_a_closed_mesh_hits_each_triangle_it_crosses() {
    let mut world = world(Vec3::ZERO, 1);
    // A closed cube mesh of half extent 1; vertex i has x, y, z = +1 for bits 0, 1, 2 of i.
    let vertices: Vec<Vec3> = (0..8)
        .map(|i| {
            let side = |bit: u32| if i & bit == 0 { -1.0 } else { 1.0 };
            Vec3::new(side(1), side(2), side(4))
        })
        .collect();
    let triangles = [
        [0, 2, 6],
        [0, 6, 4],
        [1, 7, 3],
        [1, 5, 7],
        [0, 4, 5],
        [0, 5, 1],
        [2, 3, 7],
        [2, 7, 6],
        [0, 1, 3],
        [0, 3, 2],
        [4, 6, 7],
        [4, 7, 5],
    ];
    let (mesh, dropped) = Shape::new_mesh(&vertices, &triangles).unwrap();
    assert!(dropped.is_empty());
    let body = world
        .create_body(&mesh, &BodySettings::new_static())
        .unwrap();
    // A small ball well inside each face, off the face diagonals. The triangles face inward,
    // so the ball hits one triangle where it leaves, and with back faces one more where it
    // enters.
    let sphere = Shape::new_sphere(0.05).unwrap();
    let cast = ShapeCast::new(
        &sphere,
        RVec3::new(-3.0, 0.3, 0.1),
        Quat::IDENTITY,
        Vec3::new(6.0, 0.0, 0.0),
    );
    let hits = all_hits(&world, &cast, &ALL);
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert!((hits[0].distance - 3.95).abs() <= 1.0e-3, "{hits:?}");
    assert!(hits[0].normal.x < -0.99, "{hits:?}");
    let both = cast.collide_with_back_faces(true);
    let hits = all_hits(&world, &both, &ALL);
    assert_eq!(hits.len(), 2, "{hits:?}");
    assert!(hits.iter().all(|hit| hit.body == body));
    assert_ne!(hits[0].sub_shape_id, hits[1].sub_shape_id);
    for (hit, distance) in hits.iter().zip([1.95, 3.95]) {
        assert!((hit.distance - distance).abs() <= 1.0e-3, "{hit:?}");
        assert!(hit.normal.x.abs() > 0.99, "{hit:?}");
    }

    // A wider ball aimed at the middle of a face, on its diagonal, sweeps into both of its
    // triangles.
    let wide = Shape::new_sphere(0.5).unwrap();
    let cast = ShapeCast::new(
        &wide,
        RVec3::new(-3.0, 0.0, 0.0),
        Quat::IDENTITY,
        Vec3::new(6.0, 0.0, 0.0),
    );
    let hits = all_hits(&world, &cast, &ALL);
    assert_eq!(hits.len(), 2, "{hits:?}");
    assert_ne!(hits[0].sub_shape_id, hits[1].sub_shape_id);
    assert!(
        hits.iter().all(|hit| (hit.distance - 3.5).abs() <= 1.0e-3),
        "{hits:?}"
    );
}

#[test]
fn an_invalid_cast_or_filter_fails_as_cast_shape_does() {
    let (world, _, _) = row_of_boxes();
    let mut other = common::world(Vec3::ZERO, 1);
    let foreign = add_floor(&mut other);
    let unknown = [ObjectLayer::new(5)];
    let shape = ball();
    let unit_box = cube_shape();
    let terrain = flat_height_field();
    let start = RVec3::ZERO;
    let x = Vec3::new(1.0, 0.0, 0.0);
    let casts = [
        ShapeCast::new(&shape, RVec3::new(Real::NAN, 0.0, 0.0), Quat::IDENTITY, x),
        ShapeCast::new(&shape, start, Quat::from_xyzw(0.0, 0.0, 0.0, 2.0), x),
        ShapeCast::new(&shape, start, Quat::IDENTITY, Vec3::ZERO),
        ShapeCast::new(
            &shape,
            start,
            Quat::IDENTITY,
            Vec3::new(f32::INFINITY, 0.0, 0.0),
        ),
        ShapeCast::new(&shape, start, Quat::IDENTITY, x).target_distance(-0.1),
        ShapeCast::new(&shape, start, Quat::IDENTITY, x)
            .target_distance(ShapeCast::MAX_TARGET_DISTANCE.next_up()),
        ShapeCast::new(&unit_box, start, Quat::IDENTITY, x).target_distance(0.1),
        ShapeCast::new(&terrain, start, Quat::IDENTITY, x),
    ];
    for cast in casts {
        let error = world.cast_shape(&cast, &ALL).unwrap_err();
        assert!(matches!(error, QueryError::InvalidValue(_)), "{error:?}");
        assert_eq!(world.cast_shape_all(&cast, &ALL), Err(error));
    }
    let filters = [
        QueryFilter::new().object_layers(&unknown),
        QueryFilter::new().exclude_body(foreign),
    ];
    let cast = along_x(&shape);
    for filter in filters {
        let error = world.cast_shape(&cast, &filter).unwrap_err();
        assert_eq!(world.cast_shape_all(&cast, &filter), Err(error));
    }
}
