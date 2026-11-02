//! `QueryFilter::body_filter`: a closure that rejects bodies in every query that takes a filter,
//! without hiding the bodies behind a rejected one, combined with the other filter parts.

mod common;

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Mutex;

use common::*;
use oxijolt::*;

/// Three static slabs stacked along y (tops at 4, 2 and 0) under a downward query, returned top
/// first.
fn three_slabs() -> (PhysicsWorld, [BodyId; 3]) {
    let mut world = world(Vec3::ZERO, 1);
    let slab = Shape::new_box(Vec3::new(2.0, 0.25, 2.0)).unwrap();
    let mut add = |top: Real| {
        world
            .create_body(
                &slab,
                &BodySettings::new_static().position(RVec3::new(0.0, top - 0.25, 0.0)),
            )
            .unwrap()
    };
    let slabs = [add(4.0), add(2.0), add(0.0)];
    (world, slabs)
}

fn ray_down() -> RayCast {
    RayCast::new(RVec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -20.0, 0.0))
}

fn first_ray_hit(world: &PhysicsWorld, filter: &QueryFilter<'_>) -> Option<BodyId> {
    world
        .cast_ray(&ray_down(), filter)
        .unwrap()
        .map(|hit| hit.body)
}

fn first_shape_cast_hit(world: &PhysicsWorld, filter: &QueryFilter<'_>) -> Option<BodyId> {
    let ball = Shape::new_sphere(0.3).unwrap();
    let cast = ShapeCast::new(
        &ball,
        RVec3::new(0.0, 10.0, 0.0),
        Quat::IDENTITY,
        Vec3::new(0.0, -20.0, 0.0),
    );
    world.cast_shape(&cast, filter).unwrap().map(|hit| hit.body)
}

/// The bodies a tall box through all three slabs overlaps, sorted.
fn overlapped(world: &PhysicsWorld, filter: &QueryFilter<'_>) -> Vec<BodyId> {
    let column = Shape::new_box(Vec3::new(0.5, 3.0, 0.5)).unwrap();
    let query = CollideShape::new(&column, RVec3::new(0.0, 2.0, 0.0), Quat::IDENTITY);
    let mut bodies: Vec<BodyId> = world
        .collide_shape(&query, filter)
        .unwrap()
        .into_iter()
        .map(|hit| hit.body)
        .collect();
    bodies.sort();
    bodies.dedup();
    bodies
}

#[test]
fn a_rejected_closest_body_does_not_hide_the_next_one() {
    let (world, [top, middle, bottom]) = three_slabs();
    let not_top = |body: BodyId| body != top;
    let filter = QueryFilter::new().body_filter(&not_top);

    assert_eq!(first_ray_hit(&world, &QueryFilter::new()), Some(top));
    assert_eq!(first_ray_hit(&world, &filter), Some(middle));
    assert_eq!(first_shape_cast_hit(&world, &QueryFilter::new()), Some(top));
    assert_eq!(first_shape_cast_hit(&world, &filter), Some(middle));
    assert_eq!(
        overlapped(&world, &QueryFilter::new()),
        vec![top, middle, bottom]
    );
    assert_eq!(overlapped(&world, &filter), vec![middle, bottom]);
    let point_hits = world
        .collide_point(RVec3::new(0.0, 3.9, 0.0), &filter)
        .unwrap();
    assert!(point_hits.is_empty(), "{point_hits:?}");

    let reject_all = |_: BodyId| false;
    let nothing = QueryFilter::new().body_filter(&reject_all);
    assert_eq!(first_ray_hit(&world, &nothing), None);
    assert_eq!(first_shape_cast_hit(&world, &nothing), None);
    assert_eq!(overlapped(&world, &nothing), Vec::<BodyId>::new());
}

#[test]
fn body_filter_combines_with_exclude_body_and_layers() {
    let (world, [top, middle, bottom]) = three_slabs();
    let asked = Mutex::new(Vec::new());
    let not_middle = |body: BodyId| {
        asked.lock().unwrap().push(body);
        body != middle
    };
    let filter = QueryFilter::new()
        .exclude_body(top)
        .body_filter(&not_middle);
    assert_eq!(first_ray_hit(&world, &filter), Some(bottom));
    assert_eq!(first_shape_cast_hit(&world, &filter), Some(bottom));
    assert_eq!(overlapped(&world, &filter), vec![bottom]);

    // The excluded body is skipped before the closure is asked; every id it sees is of this
    // world.
    let asked = asked.into_inner().unwrap();
    assert!(
        asked.contains(&middle) && asked.contains(&bottom),
        "{asked:?}"
    );
    assert!(!asked.contains(&top), "{asked:?}");
    assert!(asked.iter().all(|&body| world.contains_body(body)));

    let keep_all = |_: BodyId| true;
    let no_layers: [ObjectLayer; 0] = [];
    let filter = QueryFilter::new()
        .object_layers(&no_layers)
        .body_filter(&keep_all);
    assert_eq!(first_ray_hit(&world, &filter), None);
}

/// A query that runs with a filter and drops its result.
type Query = dyn Fn(&PhysicsWorld, &QueryFilter<'_>);

#[test]
fn a_panic_in_the_body_filter_resumes_after_the_query() {
    let (mut world, [top, middle, _]) = three_slabs();
    let panics = |_: BodyId| -> bool { panic!("body filter panic") };
    let filter = QueryFilter::new().body_filter(&panics);
    let queries: [(&str, &Query); 5] = [
        ("ray", &|world, filter| {
            let _ = world.cast_ray(&ray_down(), filter);
        }),
        ("all ray hits", &|world, filter| {
            let _ = world.cast_ray_all(&ray_down(), filter);
        }),
        ("shape cast", &|world, filter| {
            let _ = first_shape_cast_hit(world, filter);
        }),
        ("collide", &|world, filter| {
            let _ = overlapped(world, filter);
        }),
        ("point", &|world, filter| {
            let _ = world.collide_point(RVec3::new(0.0, 3.9, 0.0), filter);
        }),
    ];
    for (name, query) in queries {
        let payload = catch_unwind(AssertUnwindSafe(|| query(&world, &filter)))
            .expect_err("the panic resumes after the query");
        assert_eq!(
            payload.downcast_ref::<&str>(),
            Some(&"body filter panic"),
            "{name}"
        );
    }
    // The world answers queries afterwards, and no body lock is left held: removing a body
    // takes its write lock.
    assert_eq!(first_ray_hit(&world, &QueryFilter::new()), Some(top));
    world.remove_body(top).unwrap();
    assert_eq!(first_ray_hit(&world, &QueryFilter::new()), Some(middle));
}

#[test]
fn a_character_ignores_ground_the_body_filter_rejects() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let id = world
        .create_character(
            &CharacterSettings::new(&capsule).shape_offset(Vec3::new(0.0, 0.8, 0.0)),
            RVec3::ZERO,
            Quat::IDENTITY,
        )
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    assert_eq!(
        world.character(id).unwrap().ground_state(),
        GroundState::OnGround
    );
    let not_the_floor = |body: BodyId| body != floor;
    world
        .refresh_character_contacts(id, &QueryFilter::new().body_filter(&not_the_floor))
        .unwrap();
    assert_eq!(
        world.character(id).unwrap().ground_state(),
        GroundState::InAir
    );
}
