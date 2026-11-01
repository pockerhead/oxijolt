//! Scene queries: ray casts, shape casts, collide shape, filters and broad-phase refresh.

mod common;

use common::*;
use oxijolt::*;

fn down_from(x: Real, y: Real, z: Real, length: f32) -> RayCast {
    RayCast::new(RVec3::new(x, y, z), Vec3::new(0.0, -length, 0.0))
}

const ALL: QueryFilter<'static> = QueryFilter::new();

fn dot(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

#[test]
fn ray_hits_chunk_and_terrain_from_twenty_metres() {
    let (mut world, [terrain, chunk, ..]) = five_layer_world();
    let block = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    let pillar = Shape::new_cylinder(1.5, 0.4).unwrap();
    let chunk_shape = Shape::new_compound(&[
        child(&block, Vec3::new(0.0, 1.0, 0.0), Groups::STRUCTURE),
        child(&block, Vec3::new(3.0, 1.0, 0.0), Groups::STRUCTURE),
        child(&pillar, Vec3::new(0.0, 1.5, 4.0), Groups::FEATURE),
    ])
    .unwrap();
    add_static_in(&mut world, &chunk_shape, RVec3::new(6.0, 0.0, 0.0), chunk);
    let ground = add_static_in(&mut world, &flat_height_field(), RVec3::ZERO, terrain);

    let hit = world
        .cast_ray(&down_from(-3.0, 20.0, 2.0, 40.0), &ALL)
        .unwrap()
        .expect("the terrain is hit");
    assert_eq!(hit.body, ground);
    assert!((hit.distance - 20.0).abs() <= 0.5, "{hit:?}");
    assert!(hit.normal.y > 0.99, "{hit:?}");
    assert_eq!(hit.object_layer, terrain);
    assert_eq!(hit.compound_child, None);

    let hit = world
        .cast_ray(&down_from(6.0, 20.0, 0.0, 40.0), &ALL)
        .unwrap()
        .expect("the chunk is hit");
    assert!((hit.distance - 18.0).abs() <= 1.0e-4, "{hit:?}");
    assert_eq!(hit.object_layer, chunk);
    assert_eq!(
        hit.compound_child,
        Some(CompoundSubShape {
            index: 0,
            user_data: Groups::STRUCTURE
        })
    );
}

#[test]
fn wall_blocks_line_of_sight_until_removed() {
    let mut world = world(Vec3::ZERO, 1);
    let wall_shape = Shape::new_box(Vec3::new(0.25, 2.0, 2.0)).unwrap();
    let wall = world
        .create_body(
            &wall_shape,
            &BodySettings::new_static().position(RVec3::new(5.0, 1.0, 0.0)),
        )
        .unwrap();
    let sight = RayCast::new(RVec3::new(0.0, 1.0, 0.0), Vec3::new(10.0, 0.0, 0.0));
    let hit = world
        .cast_ray(&sight, &ALL)
        .unwrap()
        .expect("the wall blocks");
    assert_eq!(hit.body, wall);
    assert!(hit.fraction < 1.0);
    assert!((hit.distance - 4.75).abs() <= 1.0e-5, "{hit:?}");

    world.remove_body(wall).unwrap();
    assert_eq!(world.cast_ray(&sight, &ALL), Ok(None));
}

#[test]
fn ray_starting_inside_hits_at_exactly_zero() {
    let mut world = world(Vec3::ZERO, 1);
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let block = world
        .create_body(
            &unit_box,
            &BodySettings::new_static().position(RVec3::new(0.0, 0.0, 0.0)),
        )
        .unwrap();
    let compound = Shape::new_compound(&[
        child(&unit_box, Vec3::ZERO, Groups::STRUCTURE),
        child(&unit_box, Vec3::new(2.0, 0.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    let chunk = world
        .create_body(
            &compound,
            &BodySettings::new_static().position(RVec3::new(10.0, 0.0, 0.0)),
        )
        .unwrap();

    let structure_only = QueryFilter::new().child_groups(1 << Groups::STRUCTURE);
    for filter in [ALL, structure_only] {
        let hit = world
            .cast_ray(&down_from(0.1, 0.2, 0.0, 5.0), &filter)
            .unwrap()
            .expect("the box is hit");
        assert_eq!(hit.body, block);
        assert_eq!(hit.fraction, 0.0);
        assert_eq!(hit.distance, 0.0);

        let hit = world
            .cast_ray(&down_from(10.1, 0.2, 0.0, 5.0), &filter)
            .unwrap()
            .expect("the compound child is hit");
        assert_eq!(hit.body, chunk);
        assert_eq!(hit.fraction, 0.0);
        assert_eq!(
            hit.compound_child.map(|c| c.user_data),
            Some(Groups::STRUCTURE)
        );
    }
}

#[test]
fn spawn_ground_ignores_canopy_by_group() {
    let (mut world, [terrain, chunk, ..]) = five_layer_world();
    let porch = Shape::new_box(Vec3::new(2.0, 0.5, 2.0)).unwrap();
    let canopy = Shape::new_cylinder(0.5, 2.0).unwrap();
    let house = Shape::new_compound(&[
        child(&porch, Vec3::new(0.0, 0.5, 0.0), Groups::STRUCTURE),
        child(&canopy, Vec3::new(0.0, 3.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    let house_body = add_static_in(&mut world, &house, RVec3::ZERO, chunk);
    add_static_in(&mut world, &flat_height_field(), RVec3::ZERO, terrain);

    let ray = down_from(0.0, 5.0, 0.0, 55.0);
    let layers = [terrain, chunk];
    let ground = QueryFilter::new()
        .object_layers(&layers)
        .child_groups(1 << Groups::TERRAIN | 1 << Groups::STRUCTURE);
    let hit = world.cast_ray(&ray, &ground).unwrap().expect("ground");
    let y = ray.point_at(hit.fraction).y;
    assert!((y - 1.0).abs() <= 0.01, "porch top expected, hit at {y}");
    assert_eq!(hit.body, house_body);
    assert_eq!(
        hit.compound_child.map(|c| c.user_data),
        Some(Groups::STRUCTURE)
    );

    let hit = world.cast_ray(&ray, &ALL).unwrap().expect("canopy");
    let y = ray.point_at(hit.fraction).y;
    assert!((y - 3.5).abs() <= 0.01, "canopy top expected, hit at {y}");
    assert_eq!(
        hit.compound_child.map(|c| c.user_data),
        Some(Groups::FEATURE)
    );
}

#[test]
fn group_filter_keeps_root_call_for_power_of_two_children() {
    let mut world = world(Vec3::ZERO, 1);
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let compound = Shape::new_compound(&[
        child(&unit_box, Vec3::new(0.0, 0.0, 0.0), Groups::STRUCTURE),
        child(&unit_box, Vec3::new(2.0, 0.0, 0.0), Groups::STRUCTURE),
        child(&unit_box, Vec3::new(4.0, 0.0, 0.0), Groups::STRUCTURE),
        child(&unit_box, Vec3::new(6.0, 0.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    world
        .create_body(&compound, &BodySettings::new_static())
        .unwrap();
    let structure = QueryFilter::new().child_groups(1 << Groups::STRUCTURE);
    let hit = world
        .cast_ray(&down_from(0.0, 5.0, 0.0, 10.0), &structure)
        .unwrap()
        .expect("child 0 is hit");
    assert_eq!(
        hit.compound_child,
        Some(CompoundSubShape {
            index: 0,
            user_data: Groups::STRUCTURE
        })
    );
    assert_eq!(
        world.cast_ray(&down_from(6.0, 5.0, 0.0, 10.0), &structure),
        Ok(None)
    );
}

#[test]
fn group_filter_on_one_child_and_nested_compounds() {
    let mut world = world(Vec3::ZERO, 1);
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let single = Shape::new_compound(&[child(&unit_box, Vec3::ZERO, Groups::STRUCTURE)]).unwrap();
    world
        .create_body(&single, &BodySettings::new_static())
        .unwrap();
    let structure = QueryFilter::new().child_groups(1 << Groups::STRUCTURE);
    let feature = QueryFilter::new().child_groups(1 << Groups::FEATURE);
    let ray = down_from(0.0, 5.0, 0.0, 10.0);
    assert!(world.cast_ray(&ray, &structure).unwrap().is_some());
    assert_eq!(world.cast_ray(&ray, &feature), Ok(None));

    let inner = Shape::new_compound(&[
        child(&unit_box, Vec3::new(-0.5, 0.0, 0.0), Groups::FEATURE),
        child(&unit_box, Vec3::new(0.5, 0.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    let outer = Shape::new_compound(&[
        child(&inner, Vec3::ZERO, Groups::STRUCTURE),
        child(&unit_box, Vec3::new(4.0, 0.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    world
        .create_body(
            &outer,
            &BodySettings::new_static().position(RVec3::new(20.0, 0.0, 0.0)),
        )
        .unwrap();
    let hit = world
        .cast_ray(&down_from(20.5, 5.0, 0.0, 10.0), &structure)
        .unwrap()
        .expect("the top-level group governs the nested children");
    assert_eq!(
        hit.compound_child,
        Some(CompoundSubShape {
            index: 0,
            user_data: Groups::STRUCTURE
        })
    );
    assert_eq!(
        world.cast_ray(&down_from(20.5, 5.0, 0.0, 10.0), &feature),
        Ok(None)
    );
    assert!(world
        .cast_ray(&down_from(24.0, 5.0, 0.0, 10.0), &feature)
        .unwrap()
        .is_some());
}

#[test]
fn object_layer_selection_skips_other_layers() {
    let (mut world, [terrain, chunk, feature, ..]) = five_layer_world();
    let slab = Shape::new_box(Vec3::new(1.0, 0.25, 1.0)).unwrap();
    let upper = add_static_in(&mut world, &slab, RVec3::new(0.0, 3.0, 0.0), feature);
    let lower = add_static_in(&mut world, &slab, RVec3::new(0.0, 1.0, 0.0), chunk);
    let ray = down_from(0.0, 10.0, 0.0, 20.0);

    let hit = world.cast_ray(&ray, &ALL).unwrap().unwrap();
    assert_eq!((hit.body, hit.object_layer), (upper, feature));
    let chunk_only = [chunk];
    let hit = world
        .cast_ray(&ray, &QueryFilter::new().object_layers(&chunk_only))
        .unwrap()
        .unwrap();
    assert_eq!((hit.body, hit.object_layer), (lower, chunk));
    let terrain_only = [terrain];
    assert_eq!(
        world.cast_ray(&ray, &QueryFilter::new().object_layers(&terrain_only)),
        Ok(None)
    );
    assert_eq!(
        world.cast_ray(&ray, &QueryFilter::new().object_layers(&[])),
        Ok(None)
    );
}

#[test]
fn excluded_body_is_skipped() {
    let (mut world, [terrain, .., actor]) = five_layer_world();
    let floor_shape = Shape::new_box(Vec3::new(10.0, 1.0, 10.0)).unwrap();
    let floor = add_static_in(
        &mut world,
        &floor_shape,
        RVec3::new(0.0, -1.0, 0.0),
        terrain,
    );
    let capsule = Shape::new_capsule(0.70845, 0.4).unwrap();
    let own = world
        .create_body(
            &capsule,
            &BodySettings::new_kinematic()
                .position(RVec3::new(0.0, 1.2, 0.0))
                .object_layer(actor),
        )
        .unwrap();
    let ray = down_from(0.0, 1.2, 0.0, 10.0);

    let hit = world.cast_ray(&ray, &ALL).unwrap().unwrap();
    assert_eq!((hit.body, hit.fraction), (own, 0.0));
    let hit = world
        .cast_ray(&ray, &QueryFilter::new().exclude_body(own))
        .unwrap()
        .unwrap();
    assert_eq!(hit.body, floor);
    assert!((hit.distance - 1.2).abs() <= 1.0e-5, "{hit:?}");
}

#[test]
fn filter_rejects_unknown_layer_and_foreign_body() {
    let (world, _) = five_layer_world();
    let mut other = common::world(Vec3::ZERO, 1);
    let foreign = add_floor(&mut other);
    let ray = down_from(0.0, 5.0, 0.0, 10.0);
    let unknown = [ObjectLayer::new(5)];
    assert_eq!(
        world.cast_ray(&ray, &QueryFilter::new().object_layers(&unknown)),
        Err(QueryError::UnknownObjectLayer(unknown[0]))
    );
    assert_eq!(
        world.cast_ray(&ray, &QueryFilter::new().exclude_body(foreign)),
        Err(QueryError::WrongWorld(foreign))
    );
}

#[test]
fn ray_normals_point_out_of_the_hit_surface() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    let wall_shape = Shape::new_box(Vec3::new(0.5, 2.0, 2.0)).unwrap();
    world
        .create_body(
            &wall_shape,
            &BodySettings::new_static().position(RVec3::new(0.5, 2.0, 0.0)),
        )
        .unwrap();
    let terrain = world
        .create_body(
            &flat_height_field(),
            &BodySettings::new_static().position(RVec3::new(300.0, 0.0, 0.0)),
        )
        .unwrap();

    let hit = world
        .cast_ray(&down_from(-5.0, 5.0, 0.0, 10.0), &ALL)
        .unwrap()
        .unwrap();
    assert!(hit.normal.y > 0.99, "floor: {hit:?}");

    let from_minus_x = RayCast::new(RVec3::new(-5.0, 1.0, 0.0), Vec3::new(10.0, 0.0, 0.0));
    let hit = world.cast_ray(&from_minus_x, &ALL).unwrap().unwrap();
    assert!(hit.normal.x < -0.99, "wall: {hit:?}");

    let up = Vec3::new(0.0, 10.0, 0.0);
    let from_below = RayCast::new(RVec3::new(300.5, -5.0, 0.5), up);
    let hit = world.cast_ray(&from_below, &ALL).unwrap().unwrap();
    assert_eq!(hit.body, terrain);
    assert!(dot(hit.normal, up) > 0.0, "heightfield underside: {hit:?}");
}

#[test]
fn queries_see_created_moved_and_removed_bodies_without_a_step() {
    let mut world = world(Vec3::ZERO, 1);
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let at_origin = down_from(0.0, 5.0, 0.0, 10.0);
    let at_ten = down_from(10.0, 5.0, 0.0, 10.0);
    assert_eq!(world.cast_ray(&at_origin, &ALL), Ok(None));

    let body = world
        .create_body(&unit_box, &BodySettings::new_static())
        .unwrap();
    assert_eq!(
        world.cast_ray(&at_origin, &ALL).unwrap().unwrap().body,
        body
    );

    world
        .body_mut(body)
        .unwrap()
        .set_position(RVec3::new(10.0, 0.0, 0.0), Activation::DontActivate)
        .unwrap();
    assert_eq!(world.cast_ray(&at_origin, &ALL), Ok(None));
    assert_eq!(world.cast_ray(&at_ten, &ALL).unwrap().unwrap().body, body);

    world.remove_body(body).unwrap();
    assert_eq!(world.cast_ray(&at_ten, &ALL), Ok(None));
}

// Shape casts. The capsule is 2 * (0.70845 + 0.4) = 2.2169 m tall.
const HALF_HEIGHT: f32 = 0.70845;
const RADIUS: f32 = 0.4;
const CAPSULE_HALF_EXTENT: f32 = HALF_HEIGHT + RADIUS;

fn capsule() -> Shape {
    Shape::new_capsule(HALF_HEIGHT, RADIUS).unwrap()
}

fn down(length: f32) -> Vec3 {
    Vec3::new(0.0, -length, 0.0)
}

/// A static box with its underside at y = 4.
fn add_ceiling(world: &mut PhysicsWorld) -> BodyId {
    let shape = Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_static().position(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap()
}

#[test]
fn capsule_cast_down_reports_floor_normal_and_distance() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    let shape = capsule();
    let cast = ShapeCast::new(&shape, RVec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, down(2.0));
    let hit = world.cast_shape(&cast, &ALL).unwrap().expect("the floor");
    assert_eq!(hit.body, floor);
    assert!(
        (hit.distance - (2.0 - CAPSULE_HALF_EXTENT)).abs() < 1e-3,
        "{hit:?}"
    );
    assert!(hit.normal.y > 0.99, "{hit:?}");
    assert!(hit.point.y.abs() < 1e-3, "{hit:?}");
}

#[test]
fn capsule_cast_up_reports_ceiling_normal() {
    let mut world = world(Vec3::ZERO, 1);
    let ceiling = add_ceiling(&mut world);
    let shape = capsule();
    let up = Vec3::new(0.0, 2.0, 0.0);
    let cast = ShapeCast::new(&shape, RVec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, up);
    let hit = world.cast_shape(&cast, &ALL).unwrap().expect("the ceiling");
    assert_eq!(hit.body, ceiling);
    assert!(hit.normal.y < 0.0, "{hit:?}");
    assert!(
        (hit.distance - (2.0 - CAPSULE_HALF_EXTENT)).abs() < 1e-3,
        "{hit:?}"
    );
}

#[test]
fn rotated_capsule_cast_uses_its_rotation() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    let shape = capsule();
    let lying = quat_about(Vec3::new(0.0, 0.0, 1.0), 90.0_f32.to_radians());
    let cast = ShapeCast::new(&shape, RVec3::new(0.0, 2.0, 0.0), lying, down(2.0));
    let hit = world.cast_shape(&cast, &ALL).unwrap().expect("the floor");
    assert!((hit.distance - 1.6).abs() < 1e-3, "{hit:?}");
    assert!(hit.normal.y > 0.99, "{hit:?}");
}

#[test]
fn target_distance_stops_short() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    let shape = capsule();
    let cast = ShapeCast::new(&shape, RVec3::new(0.0, 2.0, 0.0), Quat::IDENTITY, down(2.0))
        .target_distance(0.02);
    let hit = world.cast_shape(&cast, &ALL).unwrap().expect("the floor");
    let expected = 2.0 - CAPSULE_HALF_EXTENT - 0.02;
    assert!((hit.distance - expected).abs() < 1e-3, "{hit:?}");
    assert!((hit.penetration_depth + 0.02).abs() < 1e-3, "{hit:?}");

    let gap = 0.01155;
    let near = RVec3::new(0.0, (CAPSULE_HALF_EXTENT + gap) as Real, 0.0);
    let cast = ShapeCast::new(&shape, near, Quat::IDENTITY, down(2.0)).target_distance(0.02);
    let hit = world.cast_shape(&cast, &ALL).unwrap().expect("the floor");
    assert_eq!(hit.fraction, 0.0);
    assert!((hit.penetration_depth + gap).abs() < 1e-3, "{hit:?}");
    assert!(hit.normal.y > 0.99, "{hit:?}");

    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let cast = ShapeCast::new(
        &unit_box,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
        down(2.0),
    )
    .target_distance(0.02);
    assert!(matches!(
        world.cast_shape(&cast, &ALL),
        Err(QueryError::InvalidValue(_))
    ));
}

#[test]
fn target_distance_at_the_bound_gives_a_finite_depth() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    let shape = capsule();
    let gap = 0.01155;
    let near = RVec3::new(0.0, (CAPSULE_HALF_EXTENT + gap) as Real, 0.0);
    let cast = ShapeCast::new(&shape, near, Quat::IDENTITY, down(2.0))
        .target_distance(ShapeCast::MAX_TARGET_DISTANCE);
    let hit = world.cast_shape(&cast, &ALL).unwrap().expect("the floor");
    assert_eq!(hit.fraction, 0.0);
    assert!(hit.penetration_depth.is_finite(), "{hit:?}");
    assert!((hit.penetration_depth + gap).abs() < 1e-3, "{hit:?}");
    assert!(hit.normal.y > 0.99, "{hit:?}");
}

#[test]
fn inflated_cast_matches_a_larger_capsule() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    let start = RVec3::new(0.3, 2.0, -0.2);
    let tilted = quat_about(Vec3::new(1.0, 0.0, 0.0), 20.0_f32.to_radians());
    let shape = capsule();
    let padded = ShapeCast::new(&shape, start, tilted, down(2.0)).target_distance(0.1);
    let larger_shape = Shape::new_capsule(HALF_HEIGHT, RADIUS + 0.1).unwrap();
    let larger = ShapeCast::new(&larger_shape, start, tilted, down(2.0));
    let padded = world.cast_shape(&padded, &ALL).unwrap().unwrap();
    let larger = world.cast_shape(&larger, &ALL).unwrap().unwrap();
    assert!(
        (padded.fraction - larger.fraction).abs() < 1e-6,
        "{padded:?} {larger:?}"
    );
}

#[test]
fn start_penetrating_is_reported_moving_in_not_out() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    let shape = capsule();
    let sunk = RVec3::new(0.0, (CAPSULE_HALF_EXTENT - 0.1) as Real, 0.0);
    let cast = ShapeCast::new(&shape, sunk, Quat::IDENTITY, down(2.0)).return_deepest_point(true);
    let hit = world.cast_shape(&cast, &ALL).unwrap().expect("the floor");
    assert_eq!(hit.body, floor);
    assert_eq!(hit.fraction, 0.0);
    assert!((hit.penetration_depth - 0.1).abs() < 1e-3, "{hit:?}");
    assert!(hit.normal.y > 0.99, "{hit:?}");

    let up = ShapeCast::new(&shape, sunk, Quat::IDENTITY, Vec3::new(0.0, 2.0, 0.0));
    assert_eq!(world.cast_shape(&up, &ALL), Ok(None));
}

/// A chunk compound with a FEATURE canopy (top at y = 3.5) over a STRUCTURE porch (top at
/// y = 1), on the chunk layer, and a flat terrain at y = 0.
fn porch_and_canopy(world: &mut PhysicsWorld, terrain: ObjectLayer, chunk: ObjectLayer) {
    let porch = Shape::new_box(Vec3::new(2.0, 0.5, 2.0)).unwrap();
    let canopy = Shape::new_cylinder(0.5, 2.0).unwrap();
    let house = Shape::new_compound(&[
        child(&porch, Vec3::new(0.0, 0.5, 0.0), Groups::STRUCTURE),
        child(&canopy, Vec3::new(0.0, 3.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    add_static_in(world, &house, RVec3::ZERO, chunk);
    add_static_in(world, &flat_height_field(), RVec3::ZERO, terrain);
}

#[test]
fn ball_cast_terrain_only() {
    let (mut world, [terrain, chunk, ..]) = five_layer_world();
    porch_and_canopy(&mut world, terrain, chunk);
    let ball = Shape::new_sphere(0.05).unwrap();
    let cast = ShapeCast::new(&ball, RVec3::new(0.5, 5.0, 0.5), Quat::IDENTITY, down(50.0));
    let terrain_only = [terrain];
    let hit = world
        .cast_shape(&cast, &QueryFilter::new().object_layers(&terrain_only))
        .unwrap()
        .expect("the terrain");
    assert_eq!(hit.object_layer, terrain);
    assert!(hit.point.y.abs() < 1e-3, "{hit:?}");
    assert!((hit.distance - 4.95).abs() < 1e-3, "{hit:?}");
    assert!(hit.normal.y > 0.99, "{hit:?}");

    let hit = world.cast_shape(&cast, &ALL).unwrap().expect("the canopy");
    assert!((hit.point.y - 3.5).abs() < 1e-3, "{hit:?}");
}

#[test]
fn shape_cast_child_group_filter() {
    let (mut world, [terrain, chunk, ..]) = five_layer_world();
    porch_and_canopy(&mut world, terrain, chunk);
    let ball = Shape::new_sphere(0.25).unwrap();
    let cast = ShapeCast::new(&ball, RVec3::new(0.5, 6.0, 0.5), Quat::IDENTITY, down(10.0));
    let structure = QueryFilter::new().child_groups(1 << Groups::STRUCTURE);
    let hit = world.cast_shape(&cast, &structure).unwrap().expect("porch");
    assert_eq!(
        hit.compound_child.map(|c| c.user_data),
        Some(Groups::STRUCTURE)
    );
    assert!((hit.point.y - 1.0).abs() < 1e-3, "{hit:?}");
    let feature = QueryFilter::new().child_groups(1 << Groups::FEATURE);
    let hit = world.cast_shape(&cast, &feature).unwrap().expect("canopy");
    assert_eq!(
        hit.compound_child.map(|c| c.user_data),
        Some(Groups::FEATURE)
    );
}

#[test]
fn shape_cast_rejects_invalid_input() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    let shape = capsule();
    let start = RVec3::new(0.0, 2.0, 0.0);
    let terrain = flat_height_field();
    let nan_start = RVec3::new(Real::NAN, 0.0, 0.0);
    let long = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);
    let invalid = [
        ShapeCast::new(&shape, nan_start, Quat::IDENTITY, down(1.0)),
        ShapeCast::new(&shape, start, long, down(1.0)),
        ShapeCast::new(&shape, start, Quat::IDENTITY, Vec3::ZERO),
        ShapeCast::new(&shape, start, Quat::IDENTITY, Vec3::new(f32::NAN, 0.0, 0.0)),
        ShapeCast::new(&shape, start, Quat::IDENTITY, down(1.0)).target_distance(-0.1),
        ShapeCast::new(&shape, start, Quat::IDENTITY, down(1.0)).target_distance(f32::NAN),
        ShapeCast::new(&shape, start, Quat::IDENTITY, down(1.0)).target_distance(1.0e30),
        ShapeCast::new(&shape, start, Quat::IDENTITY, down(1.0)).target_distance(f32::MAX),
        ShapeCast::new(&shape, start, Quat::IDENTITY, down(1.0))
            .target_distance(ShapeCast::MAX_TARGET_DISTANCE.next_up()),
        ShapeCast::new(&terrain, start, Quat::IDENTITY, down(1.0)),
    ];
    for cast in invalid {
        assert!(
            matches!(
                world.cast_shape(&cast, &ALL),
                Err(QueryError::InvalidValue(_))
            ),
            "{cast:?}"
        );
    }
}

// Collide shape.

fn standing_capsule_at(shape: &Shape, x: Real, y: Real) -> CollideShape<'_> {
    CollideShape::new(shape, RVec3::new(x, y, 0.0), Quat::IDENTITY)
}

#[test]
fn collide_capsule_with_floor_reports_geometric_depth() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    let wall_shape = Shape::new_box(Vec3::new(1.0, 5.0, 5.0)).unwrap();
    let wall = world
        .create_body(
            &wall_shape,
            &BodySettings::new_static().position(RVec3::new(1.0, 5.0, 0.0)),
        )
        .unwrap();
    let shape = capsule();

    let hits = world
        .collide_shape(&standing_capsule_at(&shape, -5.0, 1.0), &ALL)
        .unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    let hit = hits[0];
    assert_eq!(hit.body, floor);
    assert!(
        (hit.penetration_depth - (CAPSULE_HALF_EXTENT - 1.0)).abs() < 1e-3,
        "{hit:?}"
    );
    assert!(hit.normal.y > 0.99, "{hit:?}");

    // The wall's face is at x = 0; the capsule reaches 0.1 into it from -X.
    let x = -(RADIUS as Real) + 0.1;
    let hits = world
        .collide_shape(&standing_capsule_at(&shape, x, 2.0), &ALL)
        .unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    let hit = hits[0];
    assert_eq!(hit.body, wall);
    assert!((hit.penetration_depth - 0.1).abs() < 1e-3, "{hit:?}");
    assert!(hit.normal.x < -0.99, "{hit:?}");
}

#[test]
fn collide_ceiling_normal_points_down() {
    let mut world = world(Vec3::ZERO, 1);
    let ceiling = add_ceiling(&mut world);
    let shape = capsule();
    let y = 4.0 - CAPSULE_HALF_EXTENT as Real + 0.1;
    let hits = world
        .collide_shape(&standing_capsule_at(&shape, 0.0, y), &ALL)
        .unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert_eq!(hits[0].body, ceiling);
    assert!(hits[0].normal.y < -0.99, "{hits:?}");
    assert!((hits[0].penetration_depth - 0.1).abs() < 1e-3, "{hits:?}");
}

#[test]
fn max_separation_reports_negative_depth() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    let shape = capsule();
    let hovering = standing_capsule_at(&shape, 0.0, CAPSULE_HALF_EXTENT as Real + 0.05);
    assert_eq!(world.collide_shape(&hovering, &ALL), Ok(Vec::new()));
    let near = hovering.max_separation_distance(0.1);
    let hits = world.collide_shape(&near, &ALL).unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert!((hits[0].penetration_depth + 0.05).abs() < 1e-3, "{hits:?}");
    assert!(hits[0].normal.y > 0.99, "{hits:?}");
}

#[test]
fn collide_child_group_filter_and_excluded_body() {
    let (mut world, [terrain, chunk, .., actor]) = five_layer_world();
    let pillar = Shape::new_cylinder(1.5, 0.3).unwrap();
    let block = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    let chunk_shape = Shape::new_compound(&[
        child(&pillar, Vec3::new(0.5, 1.5, 0.0), Groups::FEATURE),
        child(&block, Vec3::new(10.0, 1.0, 0.0), Groups::STRUCTURE),
    ])
    .unwrap();
    let chunk_body = add_static_in(&mut world, &chunk_shape, RVec3::ZERO, chunk);
    let shape = capsule();
    let position = RVec3::new(0.0, 1.5, 0.0);
    let own = world
        .create_body(
            &shape,
            &BodySettings::new_kinematic()
                .position(position)
                .object_layer(actor),
        )
        .unwrap();
    let query = CollideShape::new(&shape, position, Quat::IDENTITY);
    let layers = [terrain, chunk, actor];
    let controller = QueryFilter::new().object_layers(&layers).child_groups(
        1 << Groups::TERRAIN | 1 << Groups::STRUCTURE | 1 << Groups::FEATURE | 1 << Groups::ACTOR,
    );

    let mut bodies: Vec<_> = world
        .collide_shape(&query, &controller)
        .unwrap()
        .iter()
        .map(|hit| hit.body)
        .collect();
    bodies.sort();
    let mut expected = vec![chunk_body, own];
    expected.sort();
    assert_eq!(bodies, expected);

    let hits = world
        .collide_shape(&query, &controller.exclude_body(own))
        .unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert_eq!(hits[0].body, chunk_body);
    assert_eq!(hits[0].object_layer, chunk);
    assert_eq!(
        hits[0].compound_child.map(|c| c.user_data),
        Some(Groups::FEATURE)
    );
    assert!(hits[0].normal.x < -0.99, "{hits:?}");

    let without_features = controller
        .child_groups(1 << Groups::TERRAIN | 1 << Groups::STRUCTURE | 1 << Groups::ACTOR)
        .exclude_body(own);
    assert_eq!(
        world.collide_shape(&query, &without_features),
        Ok(Vec::new())
    );
}

#[test]
fn nested_compound_group_governs_shape_casts_and_collide() {
    let mut world = world(Vec3::ZERO, 1);
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let inner = Shape::new_compound(&[
        child(&unit_box, Vec3::new(-0.5, 0.0, 0.0), Groups::FEATURE),
        child(&unit_box, Vec3::new(0.5, 0.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    let outer = Shape::new_compound(&[
        child(&inner, Vec3::ZERO, Groups::STRUCTURE),
        child(&unit_box, Vec3::new(4.0, 0.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    world
        .create_body(&outer, &BodySettings::new_static())
        .unwrap();
    let structure = QueryFilter::new().child_groups(1 << Groups::STRUCTURE);
    let feature = QueryFilter::new().child_groups(1 << Groups::FEATURE);
    let top_level_structure = Some(CompoundSubShape {
        index: 0,
        user_data: Groups::STRUCTURE,
    });
    let ball = Shape::new_sphere(0.25).unwrap();

    let cast = ShapeCast::new(&ball, RVec3::new(0.5, 5.0, 0.0), Quat::IDENTITY, down(10.0));
    let hit = world
        .cast_shape(&cast, &structure)
        .unwrap()
        .expect("the top-level group governs the nested children");
    assert_eq!(hit.compound_child, top_level_structure);
    assert!((hit.point.y - 0.5).abs() < 1e-3, "{hit:?}");
    assert_eq!(world.cast_shape(&cast, &feature), Ok(None));

    let overlapping = CollideShape::new(&ball, RVec3::new(0.5, 0.6, 0.0), Quat::IDENTITY);
    let hits = world.collide_shape(&overlapping, &structure).unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert_eq!(hits[0].compound_child, top_level_structure);
    assert!((hits[0].penetration_depth - 0.15).abs() < 1e-3, "{hits:?}");
    assert_eq!(world.collide_shape(&overlapping, &feature), Ok(Vec::new()));

    let beside = CollideShape::new(&ball, RVec3::new(4.0, 0.6, 0.0), Quat::IDENTITY);
    assert_eq!(world.collide_shape(&beside, &feature).unwrap().len(), 1);
}

#[test]
fn collide_shape_rejects_invalid_input() {
    let world = world(Vec3::ZERO, 1);
    let shape = capsule();
    let terrain = flat_height_field();
    let invalid = [
        standing_capsule_at(&shape, Real::NAN, 0.0),
        CollideShape::new(&shape, RVec3::ZERO, Quat::from_xyzw(0.0, 0.0, 0.0, 2.0)),
        standing_capsule_at(&shape, 0.0, 0.0).max_separation_distance(-0.1),
        standing_capsule_at(&shape, 0.0, 0.0).max_separation_distance(f32::INFINITY),
        CollideShape::new(&terrain, RVec3::ZERO, Quat::IDENTITY),
    ];
    for query in invalid {
        assert!(
            matches!(
                world.collide_shape(&query, &ALL),
                Err(QueryError::InvalidValue(_))
            ),
            "{query:?}"
        );
    }
}

#[test]
fn compound_query_shape_is_placed_by_its_origin() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    // The compound's centre of mass is at x = 1, away from its origin.
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let pair = Shape::new_compound(&[
        child(&unit_box, Vec3::ZERO, 0),
        child(&unit_box, Vec3::new(2.0, 0.0, 0.0), 0),
    ])
    .unwrap();
    let quarter_turn = quat_about(Vec3::new(0.0, 0.0, 1.0), 90.0_f32.to_radians());
    // Rotated a quarter turn about Z, the second box sits 2 m above the first, whose bottom
    // face reaches 0.1 into the floor.
    let query = CollideShape::new(&pair, RVec3::new(0.0, 0.4, 0.0), quarter_turn);
    let hits = world.collide_shape(&query, &ALL).unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert!((hits[0].penetration_depth - 0.1).abs() < 1e-3, "{hits:?}");
    let cast = ShapeCast::new(&pair, RVec3::new(0.0, 2.0, 0.0), quarter_turn, down(4.0));
    let hit = world.cast_shape(&cast, &ALL).unwrap().unwrap();
    assert!((hit.distance - 1.5).abs() < 1e-3, "{hit:?}");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn filtered_queries_run_in_parallel() {
    let (mut world, [terrain, chunk, ..]) = five_layer_world();
    porch_and_canopy(&mut world, terrain, chunk);
    let ball = Shape::new_sphere(0.25).unwrap();
    let layers = [terrain, chunk];
    let ground = QueryFilter::new()
        .object_layers(&layers)
        .child_groups(1 << Groups::TERRAIN | 1 << Groups::STRUCTURE);
    let run = |i: usize| {
        let x = -3.0 + 0.012 * i as Real;
        let ray = world
            .cast_ray(&down_from(x, 6.0, 0.5, 10.0), &ground)
            .unwrap();
        let cast = ShapeCast::new(&ball, RVec3::new(x, 6.0, 0.5), Quat::IDENTITY, down(10.0));
        let cast = world.cast_shape(&cast, &ground).unwrap();
        let overlap = CollideShape::new(&ball, RVec3::new(x, 1.1, 0.5), Quat::IDENTITY);
        let overlap = world.collide_shape(&overlap, &ground).unwrap();
        (ray, cast, overlap)
    };
    let expected: Vec<_> = (0..500).map(run).collect();
    std::thread::scope(|scope| {
        let threads: Vec<_> = (0..8)
            .map(|_| scope.spawn(|| (0..500).map(run).collect::<Vec<_>>()))
            .collect();
        for thread in threads {
            assert!(thread.join().unwrap() == expected);
        }
    });
}

#[test]
fn optimize_broad_phase_keeps_query_results() {
    let mut world = world(Vec3::ZERO, 1);
    let unit_box = Shape::new_box(Vec3::new(0.4, 0.4, 0.4)).unwrap();
    for i in 0..200 {
        let (x, z) = ((i % 20) as Real, (i / 20) as Real);
        world
            .create_body(
                &unit_box,
                &BodySettings::new_static().position(RVec3::new(x, 0.1 * x, z)),
            )
            .unwrap();
    }
    let ball = Shape::new_sphere(0.3).unwrap();
    let queries = |world: &PhysicsWorld| {
        let rays: Vec<_> = (0..50)
            .map(|i| {
                let x = 0.37 * i as Real;
                let ray = down_from(x, 10.0, 0.21 * i as Real, 20.0);
                world.cast_ray(&ray, &ALL).unwrap()
            })
            .collect();
        let casts: Vec<_> = (0..10)
            .map(|i| {
                let start = RVec3::new(1.9 * i as Real, 10.0, i as Real);
                let cast = ShapeCast::new(&ball, start, Quat::IDENTITY, down(20.0));
                world.cast_shape(&cast, &ALL).unwrap()
            })
            .collect();
        (rays, casts)
    };
    let before = queries(&world);
    assert!(before.0.iter().filter(|hit| hit.is_some()).count() > 10);
    world.optimize_broad_phase();
    assert!(queries(&world) == before);
}

/// A ray keeps the origin and direction it was built with; `point_at` spans them.
#[test]
fn ray_cast_builder_defaults() {
    let origin = RVec3::new(1.0, 2.0, 3.0);
    let direction = Vec3::new(0.0, -4.0, 0.5);
    let ray = RayCast::new(origin, direction);
    assert_eq!(ray.origin(), origin);
    assert_eq!(ray.direction(), direction);
    assert_eq!(ray.point_at(0.0), origin);
    assert_eq!(ray.point_at(1.0), RVec3::new(1.0, -2.0, 3.5));
}
