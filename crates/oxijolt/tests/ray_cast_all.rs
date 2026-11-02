//! `PhysicsWorld::cast_ray_all`: every hit of a ray, sorted, with the rules and filters of
//! `cast_ray`.

mod common;

use common::meshes::flat_grid;
use common::*;
use oxijolt::*;

const ALL: QueryFilter<'static> = QueryFilter::new();

/// Every hit of `ray`, checked against `cast_ray`: the first hit is its hit, and the hits are
/// sorted by fraction, body id and sub-shape id.
fn all_hits(world: &PhysicsWorld, ray: &RayCast, filter: &QueryFilter<'_>) -> Vec<RayCastHit> {
    let hits = world.cast_ray_all(ray, filter).unwrap();
    assert_eq!(hits.first().copied(), world.cast_ray(ray, filter).unwrap());
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
