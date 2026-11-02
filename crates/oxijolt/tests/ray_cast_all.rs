//! `PhysicsWorld::cast_ray_all`: every hit of a ray, sorted, with the rules and filters of
//! `cast_ray`.

mod common;

use common::meshes::flat_grid;
use common::*;
use oxijolt::*;

const ALL: QueryFilter<'static> = QueryFilter::new();

/// Every hit of `ray`, checked against `cast_ray`: its hit is one of the hits at the first
/// hit's fraction (the first hit itself when no other shares that fraction), and the hits are
/// sorted by fraction, body id and sub-shape id.
fn all_hits(world: &PhysicsWorld, ray: &RayCast, filter: &QueryFilter<'_>) -> Vec<RayCastHit> {
    let hits = world.cast_ray_all(ray, filter).unwrap();
    let closest = world.cast_ray(ray, filter).unwrap();
    assert_eq!(
        hits.first().map(|hit| hit.fraction),
        closest.map(|hit| hit.fraction)
    );
    if let Some(closest) = closest {
        let mut tied = hits
            .iter()
            .take_while(|hit| hit.fraction == closest.fraction);
        assert!(
            tied.any(|hit| *hit == closest),
            "{closest:?} not in {hits:?}"
        );
    }
    for pair in hits.windows(2) {
        let key = |hit: &RayCastHit| (hit.fraction, hit.body, hit.sub_shape_id);
        assert!(key(&pair[0]) <= key(&pair[1]), "{hits:?}");
    }
    hits
}

/// A ray along +x from the origin, 10 m long.
fn along_x() -> RayCast {
    RayCast::new(RVec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0))
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
    let hits = all_hits(&world, &along_x(), &ALL);
    let bodies: Vec<BodyId> = hits.iter().map(|hit| hit.body).collect();
    assert_eq!(bodies, boxes);
    for (hit, entry) in hits.iter().zip([1.5, 3.5, 5.5, 7.5]) {
        assert!((hit.distance - entry).abs() <= 1.0e-5, "{hit:?}");
        assert!((hit.fraction - entry / 10.0).abs() <= 1.0e-6, "{hit:?}");
        assert!(hit.normal.x < -0.99, "{hit:?}");
        assert_eq!(hit.compound_child, None);
    }
    let layers: Vec<ObjectLayer> = hits.iter().map(|hit| hit.object_layer).collect();
    assert_eq!(layers, [terrain, chunk, feature, item]);

    // A ray that misses everything, and one that ends before the second box.
    let above = RayCast::new(RVec3::new(0.0, 3.0, 0.0), Vec3::new(10.0, 0.0, 0.0));
    assert!(all_hits(&world, &above, &ALL).is_empty());
    let short = RayCast::new(RVec3::ZERO, Vec3::new(3.0, 0.0, 0.0));
    let hits = all_hits(&world, &short, &ALL);
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].body, boxes[0]);
}

#[test]
fn a_ray_that_starts_inside_a_box_hits_it_at_zero() {
    let (world, boxes, _) = row_of_boxes();
    let inside = RayCast::new(RVec3::new(2.1, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0));
    let hits = all_hits(&world, &inside, &ALL);
    let bodies: Vec<BodyId> = hits.iter().map(|hit| hit.body).collect();
    assert_eq!(bodies, boxes);
    assert_eq!(hits[0].fraction, 0.0);
    assert_eq!(hits[0].distance, 0.0);
    assert!(hits[1].fraction > 0.0);
}

#[test]
fn filters_apply_to_every_hit() {
    let (world, [at_2, at_4, at_6, at_8], [terrain, chunk, feature, ..]) = row_of_boxes();
    let ray = along_x();
    let bodies = |filter: &QueryFilter<'_>| -> Vec<BodyId> {
        all_hits(&world, &ray, filter)
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
fn hits_are_ordered_by_fraction_not_by_bounds() {
    let mut world = world(Vec3::ZERO, 1);
    // The ray enters the sphere's bounds at x 3 but its surface only at x 4.38; the small box
    // in front of that is entered at x 3.6.
    let sphere = world
        .create_body(
            &Shape::new_sphere(2.0).unwrap(),
            &BodySettings::new_static().position(RVec3::new(5.0, 1.9, 0.0)),
        )
        .unwrap();
    let small_box = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
    let at = RVec3::new(3.8, 0.0, 0.0);
    let small = world
        .create_body(&small_box, &BodySettings::new_static().position(at))
        .unwrap();
    // A second box in the same place: equal fractions, so the lower body id comes first.
    let twin = world
        .create_body(&small_box, &BodySettings::new_static().position(at))
        .unwrap();
    let hits = all_hits(&world, &along_x(), &ALL);
    let bodies: Vec<BodyId> = hits.iter().map(|hit| hit.body).collect();
    assert_eq!(bodies, [small, twin, sphere]);
    assert_eq!(hits[0].fraction, hits[1].fraction);
    assert!(
        (hits[2].distance - (5.0 - 0.39_f32.sqrt())).abs() <= 1.0e-4,
        "{hits:?}"
    );
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
    let from_outside = RayCast::new(RVec3::new(-4.0, 0.0, 0.0), Vec3::new(8.0, 0.0, 0.0));
    let from_inside = RayCast::new(RVec3::ZERO, Vec3::new(8.0, 0.0, 0.0));
    for optimized in [false, true] {
        if optimized {
            world.optimize_broad_phase();
        }
        for (ray, fraction) in [(&from_outside, 0.375), (&from_inside, 0.0)] {
            // `all_hits` checks that `cast_ray`'s hit is one of the tied ones; which one it is
            // follows Jolt's traversal.
            let hits = all_hits(&world, ray, &ALL);
            let bodies: Vec<BodyId> = hits.iter().map(|hit| hit.body).collect();
            assert_eq!(bodies, boxes, "optimized: {optimized}");
            assert!(hits.iter().all(|hit| hit.fraction == fraction), "{hits:?}");
        }
    }
}

#[test]
fn a_compound_reports_each_leaf_the_ray_hits() {
    let mut world = world(Vec3::ZERO, 1);
    let unit_box = cube_shape();
    // Children 0 and 2 coincide at x 2, child 1 sits at x 5.
    let compound = Shape::new_compound(&[
        child(&unit_box, Vec3::new(2.0, 0.0, 0.0), Groups::STRUCTURE),
        child(&unit_box, Vec3::new(5.0, 0.0, 0.0), Groups::FEATURE),
        child(&unit_box, Vec3::new(2.0, 0.0, 0.0), Groups::ITEM),
    ])
    .unwrap();
    let body = world
        .create_body(&compound, &BodySettings::new_static())
        .unwrap();
    let hits = all_hits(&world, &along_x(), &ALL);
    assert_eq!(hits.len(), 3, "{hits:?}");
    assert!(hits.iter().all(|hit| hit.body == body));
    // The coincident children tie at x 1.5 and are ordered by sub-shape id.
    assert_eq!(hits[0].fraction, hits[1].fraction);
    assert!(hits[0].sub_shape_id < hits[1].sub_shape_id);
    assert!((hits[0].distance - 1.5).abs() <= 1.0e-5, "{hits:?}");
    assert!((hits[2].distance - 4.5).abs() <= 1.0e-5, "{hits:?}");
    let mut tied: Vec<u32> = hits[..2]
        .iter()
        .map(|hit| hit.compound_child.unwrap().index)
        .collect();
    tied.sort_unstable();
    assert_eq!(tied, [0, 2]);
    assert_eq!(
        hits[2].compound_child,
        Some(CompoundSubShape {
            index: 1,
            user_data: Groups::FEATURE
        })
    );

    // Child groups drop leaves of the body, the body filter drops the body.
    let no_item = QueryFilter::new().child_groups(1 << Groups::STRUCTURE | 1 << Groups::FEATURE);
    let children: Vec<u32> = all_hits(&world, &along_x(), &no_item)
        .iter()
        .map(|hit| hit.compound_child.unwrap().index)
        .collect();
    assert_eq!(children, [0, 1]);
    let keep_none = |_: BodyId| false;
    let filter = QueryFilter::new().body_filter(&keep_none);
    assert!(all_hits(&world, &along_x(), &filter).is_empty());
}

#[test]
fn a_ray_through_a_closed_mesh_hits_each_triangle_it_crosses() {
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
    // Off every face diagonal: one triangle where the ray enters, one where it leaves.
    let ray = RayCast::new(RVec3::new(-3.0, 0.3, 0.1), Vec3::new(6.0, 0.0, 0.0));
    let hits = all_hits(&world, &ray, &ALL);
    assert_eq!(hits.len(), 2, "{hits:?}");
    assert!(hits.iter().all(|hit| hit.body == body));
    assert_ne!(hits[0].sub_shape_id, hits[1].sub_shape_id);
    for (hit, distance) in hits.iter().zip([2.0, 4.0]) {
        assert!((hit.distance - distance).abs() <= 1.0e-5, "{hit:?}");
        assert!(hit.normal.x.abs() > 0.99, "{hit:?}");
    }
}

#[test]
fn mesh_and_heightfield_are_hit_from_below_too() {
    let mut world = world(Vec3::ZERO, 1);
    let height_field = world
        .create_body(&flat_height_field(), &BodySettings::new_static())
        .unwrap();
    let mesh = world
        .create_body(
            &flat_grid(8, 1.0),
            &BodySettings::new_static().position(RVec3::new(0.0, -2.0, 0.0)),
        )
        .unwrap();
    // Off every triangle edge, so each surface gives one hit.
    let up = RayCast::new(RVec3::new(0.3, -5.0, 0.6), Vec3::new(0.0, 10.0, 0.0));
    let hits = all_hits(&world, &up, &ALL);
    let bodies: Vec<BodyId> = hits.iter().map(|hit| hit.body).collect();
    assert_eq!(bodies, [mesh, height_field]);
    for (hit, distance) in hits.iter().zip([3.0, 5.0]) {
        assert!((hit.distance - distance).abs() <= 1.0e-5, "{hit:?}");
        // The face's own normal, pointing away from the side the ray came from.
        assert!(hit.normal.y > 0.99, "{hit:?}");
    }

    let down = RayCast::new(RVec3::new(0.3, 5.0, 0.6), Vec3::new(0.0, -10.0, 0.0));
    let bodies: Vec<BodyId> = all_hits(&world, &down, &ALL)
        .iter()
        .map(|hit| hit.body)
        .collect();
    assert_eq!(bodies, [height_field, mesh]);
}

#[test]
fn an_invalid_ray_or_filter_fails_as_cast_ray_does() {
    let (world, _, _) = row_of_boxes();
    let mut other = common::world(Vec3::ZERO, 1);
    let foreign = add_floor(&mut other);
    let unknown = [ObjectLayer::new(5)];
    let rays = [
        RayCast::new(RVec3::new(Real::NAN, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
        RayCast::new(
            RVec3::new(limits::MAX_POSITION * 2.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        RayCast::new(RVec3::ZERO, Vec3::ZERO),
        RayCast::new(RVec3::ZERO, Vec3::new(f32::INFINITY, 0.0, 0.0)),
    ];
    for ray in rays {
        let error = world.cast_ray(&ray, &ALL).unwrap_err();
        assert!(matches!(error, QueryError::InvalidValue(_)), "{error:?}");
        assert_eq!(world.cast_ray_all(&ray, &ALL), Err(error));
    }
    let filters = [
        QueryFilter::new().object_layers(&unknown),
        QueryFilter::new().exclude_body(foreign),
    ];
    for filter in filters {
        let error = world.cast_ray(&along_x(), &filter).unwrap_err();
        assert_eq!(world.cast_ray_all(&along_x(), &filter), Err(error));
    }
}
