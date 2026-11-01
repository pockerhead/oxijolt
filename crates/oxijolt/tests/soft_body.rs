//! Soft bodies: draping, vertex readout and writes, forces, refusals, pressure, queries and
//! sharing settings between worlds.

mod common;

use common::soft_body::{
    cloth_attributes, distance, enclosed_volume, max_vertex_speed, sphere, tetrahedral_cube, Cloth,
    CUBE_FACES,
};
use common::{add_floor, step, world, DT};
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const ALL: QueryFilter<'static> = QueryFilter::new();

fn quat_about_y(angle: f32) -> Quat {
    let (sin, cos) = (angle / 2.0).sin_cos();
    Quat::from_xyzw(0.0, sin, 0.0, cos)
}

fn finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

fn finite_position(p: RVec3) -> bool {
    p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
}

fn add_static_sphere(world: &mut PhysicsWorld, radius: f32) -> BodyId {
    let shape = Shape::new_sphere(radius).unwrap();
    world
        .create_body(&shape, &BodySettings::new_static())
        .unwrap()
}

/// A 4 x 4 cloth without constraints at `position`, every vertex free.
fn loose_cloth(world: &mut PhysicsWorld, settings: SoftBodySettings) -> BodyId {
    let cloth = Cloth::new(4, 0.2);
    let shared = cloth.builder().build().unwrap();
    world.create_soft_body(&shared, &settings).unwrap()
}

#[test]
fn a_cloth_pinned_at_two_corners_drapes_over_a_sphere() {
    let mut world = world(GRAVITY, 2);
    const RADIUS: f32 = 0.5;
    add_static_sphere(&mut world, RADIUS);
    let cloth = Cloth::new(21, 0.1);
    let pins = cloth.first_row_corners();
    let cloth = cloth.pin(&pins);
    let shared = cloth.settings();
    let height = RADIUS + 0.15;
    // Damping 2 lets the cloth settle within the 300 steps. Measured (Windows, x64): the cloth
    // sleeps after 250 steps; 92 vertices rest within 5 cm of the sphere, the lowest hangs at
    // -0.64 m, grid edges stretch by at most 10 % and the pins drift by less than 1e-6 m.
    // With Jolt's default damping of 0.1, or stiffer bends (compliance 1e-3), vertices next to
    // the pins still moved at 0.3 to 1 m/s after 600 steps.
    let id = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default()
                .position(RVec3::new(0.0, height as Real, 0.0))
                .linear_damping(2.0),
        )
        .unwrap();
    let start = world.soft_body(id).unwrap().vertices();

    step(&mut world, 300);

    let vertices = world.soft_body(id).unwrap().vertices();
    assert!(vertices
        .iter()
        .all(|v| finite_position(v.position) && finite(v.velocity)));
    for pin in pins {
        let (before, after) = (start[pin as usize], vertices[pin as usize]);
        assert_eq!(after.inverse_mass, 0.0);
        assert_eq!(after.velocity, Vec3::ZERO);
        // The body origin moves to the centre of the vertices every step, in mixed f32 and
        // `Real` arithmetic, so a pinned vertex keeps its place only within rounding.
        let drift = distance(before.position, after.position);
        assert!(drift <= 1.0e-4, "pin {pin} drifted {drift} m");
    }
    let centre = RVec3::ZERO;
    let radius = f64::from(RADIUS);
    let closest = vertices
        .iter()
        .map(|v| distance(v.position, centre))
        .fold(f64::INFINITY, f64::min);
    assert!(
        closest >= radius - 0.02,
        "a vertex sank into the sphere: {closest}"
    );
    let resting = vertices
        .iter()
        .filter(|v| distance(v.position, centre) <= radius + 0.05 && v.position.y > 0.0)
        .count();
    assert!(resting >= 10, "{resting} vertices rest on the sphere");
    let lowest = vertices
        .iter()
        .map(|v| v.position.y)
        .fold(Real::INFINITY, Real::min);
    assert!(lowest < 0.0, "the cloth hangs below the centre: {lowest}");
    let rest = f64::from(cloth.spacing);
    for [a, b] in cloth.grid_edges() {
        let length = distance(vertices[a as usize].position, vertices[b as usize].position);
        assert!(
            (length - rest).abs() <= 0.15 * rest,
            "edge {a}-{b} is {length} m long"
        );
    }
    let body = world.body(id).unwrap();
    let speed = max_vertex_speed(&world, id);
    assert!(
        body.is_sleeping() || speed < 0.05,
        "the cloth still moves at {speed} m/s"
    );
}

#[test]
fn vertices_read_back_in_world_space() {
    let mut world = world(GRAVITY, 1);
    let cloth = Cloth::new(3, 0.5);
    let shared = cloth.builder().build().unwrap();
    let position = RVec3::new(1.0, 2.0, 3.0);
    let rotation = quat_about_y(0.5);
    let id = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default()
                .position(position)
                .rotation(rotation),
        )
        .unwrap();
    let body = world.soft_body(id).unwrap();
    assert_eq!(body.id(), id);
    assert_eq!(body.vertex_count(), 9);
    let vertices = body.vertices();
    assert_eq!(vertices.len(), 9);
    for (state, vertex) in vertices.iter().zip(&cloth.vertices) {
        let local = vertex.position;
        let (sin, cos) = 0.5_f32.sin_cos();
        let expected = RVec3::new(
            position.x + Real::from(cos * local.x + sin * local.z),
            position.y + Real::from(local.y),
            position.z + Real::from(-sin * local.x + cos * local.z),
        );
        assert!(distance(state.position, expected) < 1.0e-5, "{state:?}");
        assert_eq!(state.velocity, Vec3::ZERO);
        assert_eq!(state.inverse_mass, 1.0);
    }
    // The rotation is baked into the vertices.
    assert_eq!(world.body(id).unwrap().rotation(), Quat::IDENTITY);

    let mut reused = Vec::with_capacity(64);
    let capacity = reused.capacity();
    world.soft_body(id).unwrap().vertices_into(&mut reused);
    assert_eq!(reused, vertices);
    assert_eq!(reused.capacity(), capacity);

    add_floor(&mut world);
    let floor = add_floor(&mut world);
    assert!(matches!(
        world.soft_body(floor),
        Err(BodyError::NotSoftBody(id)) if id == floor
    ));
    assert!(matches!(
        world.soft_body_mut(floor),
        Err(BodyError::NotSoftBody(_))
    ));
    assert!(world.body(id).unwrap().is_soft_body());
    assert!(!world.body(floor).unwrap().is_soft_body());
    world.remove_body(id).unwrap();
    assert!(matches!(world.soft_body(id), Err(BodyError::NotFound(_))));
}

#[test]
fn pinning_and_unpinning_vertices() {
    let mut world = world(GRAVITY, 1);
    let cloth = Cloth::new(3, 0.5);
    let shared = cloth
        .builder()
        .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
        .build()
        .unwrap();
    let id = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default().position(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    let mass = world.body(id).unwrap().mass().unwrap();
    assert!((mass - 9.0).abs() < 1.0e-5, "{mass}");

    // Pin the whole cloth: it stays.
    for index in 0..9 {
        world
            .soft_body_mut(id)
            .unwrap()
            .set_vertex_inverse_mass(index, 0.0)
            .unwrap();
    }
    assert_eq!(world.body(id).unwrap().mass(), None);
    let before = world.soft_body(id).unwrap().vertices();
    step(&mut world, 10);
    let after = world.soft_body(id).unwrap().vertices();
    for (a, b) in before.iter().zip(&after) {
        assert!(distance(a.position, b.position) < 1.0e-5);
    }

    // Unpin one: it falls, the others hold it on its edges.
    let mut body = world.soft_body_mut(id).unwrap();
    body.set_vertex_inverse_mass(4, 0.5).unwrap();
    assert_eq!(world.body(id).unwrap().mass(), None);
    step(&mut world, 10);
    let fallen = world.soft_body(id).unwrap().vertices();
    assert!(fallen[4].position.y < after[4].position.y);
    assert_eq!(fallen[4].inverse_mass, 0.5);

    // Unpin the rest with masses at the total bound: one more would pass it and is refused.
    let heavy = 8.0 / limits::MAX_MASS;
    let mut body = world.soft_body_mut(id).unwrap();
    for index in [0, 1, 2, 3, 5, 6, 7] {
        body.set_vertex_inverse_mass(index, heavy).unwrap();
    }
    assert_eq!(
        body.set_vertex_inverse_mass(8, heavy),
        Err(BodyError::InvalidValue(
            "the masses of the movable vertices must add up to at most limits::MAX_MASS"
        ))
    );
    assert_eq!(world.soft_body(id).unwrap().vertices()[8].inverse_mass, 0.0);
    world
        .soft_body_mut(id)
        .unwrap()
        .set_vertex_inverse_mass(8, 1.0)
        .unwrap();
    let mass = world.body(id).unwrap().mass().unwrap();
    let expected = 7.0 * limits::MAX_MASS / 8.0 + 2.0 + 1.0;
    assert!((mass - expected).abs() <= expected * 1.0e-5, "{mass}");

    // A setter wakes a sleeping body.
    let sleeper = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default()
                .position(RVec3::new(10.0, 5.0, 0.0))
                .activation(Activation::DontActivate),
        )
        .unwrap();
    assert!(world.body(sleeper).unwrap().is_sleeping());
    world
        .soft_body_mut(sleeper)
        .unwrap()
        .set_vertex_inverse_mass(0, 0.0)
        .unwrap();
    assert!(world.body(sleeper).unwrap().is_active());
}

#[test]
fn invalid_vertex_writes_change_nothing() {
    let mut world = world(GRAVITY, 1);
    let id = loose_cloth(&mut world, SoftBodySettings::default());
    let before = world.soft_body(id).unwrap().vertices();
    let mut body = world.soft_body_mut(id).unwrap();
    let invalid = |result| matches!(result, Err(BodyError::InvalidValue(_)));
    assert!(invalid(body.set_vertex_velocity(16, Vec3::ZERO)));
    assert!(invalid(body.set_vertex_velocity(
        0,
        Vec3::new(limits::MAX_LINEAR_VELOCITY.next_up(), 0.0, 0.0)
    )));
    assert!(invalid(
        body.set_vertex_velocity(0, Vec3::new(f32::NAN, 0.0, 0.0))
    ));
    assert!(invalid(body.set_vertex_inverse_mass(16, 1.0)));
    for inverse_mass in [-1.0, f32::NAN, limits::MAX_VERTEX_INVERSE_MASS.next_up()] {
        assert!(invalid(body.set_vertex_inverse_mass(0, inverse_mass)));
    }
    // A movable vertex cannot be moved kinematically.
    assert!(invalid(body.move_kinematic_vertex(0, RVec3::ZERO, DT)));
    assert_eq!(world.soft_body(id).unwrap().vertices(), before);
}

#[test]
fn a_kinematic_vertex_moves_to_its_target_and_keeps_its_velocity() {
    let mut world = world(Vec3::ZERO, 1);
    let cloth = Cloth::new(3, 0.5).pin(&[0]);
    let shared = cloth.builder().build().unwrap();
    let id = world
        .create_soft_body(&shared, &SoftBodySettings::default())
        .unwrap();
    let start = world.soft_body(id).unwrap().vertices()[0].position;
    let target = RVec3::new(start.x + 0.1, start.y + 0.05, start.z);
    world
        .soft_body_mut(id)
        .unwrap()
        .move_kinematic_vertex(0, target, DT)
        .unwrap();
    step(&mut world, 1);
    let moved = world.soft_body(id).unwrap().vertices()[0];
    assert!(distance(moved.position, target) < 1.0e-5, "{moved:?}");
    step(&mut world, 1);
    let further = world.soft_body(id).unwrap().vertices()[0].position;
    assert!(further.x > target.x + 0.09, "the vertex keeps moving");
    world
        .soft_body_mut(id)
        .unwrap()
        .set_vertex_velocity(0, Vec3::ZERO)
        .unwrap();
    step(&mut world, 1);
    let stopped = world.soft_body(id).unwrap().vertices()[0].position;
    assert!(distance(stopped, further) < 1.0e-5);

    // Too fast for one step, and outside the frame.
    let mut body = world.soft_body_mut(id).unwrap();
    let far = RVec3::new(stopped.x + 100.0, stopped.y, stopped.z);
    assert!(matches!(
        body.move_kinematic_vertex(0, far, DT),
        Err(BodyError::InvalidValue(_))
    ));
    let outside = RVec3::new(limits::MAX_POSITION * 2.0, 0.0, 0.0);
    assert!(matches!(
        body.move_kinematic_vertex(0, outside, 1.0),
        Err(BodyError::InvalidValue(_))
    ));
    assert!(matches!(
        body.move_kinematic_vertex(0, stopped, 0.0),
        Err(BodyError::InvalidValue(_))
    ));
}

/// The mean vertex velocity of soft body `id`, normalised.
fn mean_velocity_direction(world: &PhysicsWorld, id: BodyId) -> Vec3 {
    let vertices = world.soft_body(id).unwrap().vertices();
    let mut sum = [0.0_f32; 3];
    for v in &vertices {
        sum[0] += v.velocity.x;
        sum[1] += v.velocity.y;
        sum[2] += v.velocity.z;
    }
    let length = (sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]).sqrt();
    assert!(length > 0.0, "the body does not move");
    Vec3::new(sum[0] / length, sum[1] / length, sum[2] / length)
}

#[test]
fn add_force_on_a_rotated_soft_body_pushes_along_the_world_force() {
    let mut world = world(GRAVITY, 1);
    let quarter = quat_about_y(std::f32::consts::FRAC_PI_2);
    let rotated = loose_cloth(
        &mut world,
        SoftBodySettings::default()
            .rotation(quarter)
            .make_rotation_identity(false)
            .gravity_factor(0.0),
    );
    assert!(world.body(rotated).unwrap().rotation() != Quat::IDENTITY);
    // Turned after creation instead.
    let turned = loose_cloth(
        &mut world,
        SoftBodySettings::default()
            .position(RVec3::new(5.0, 0.0, 0.0))
            .gravity_factor(0.0),
    );
    world
        .body_mut(turned)
        .unwrap()
        .set_rotation(quarter, Activation::Activate)
        .unwrap();
    for id in [rotated, turned] {
        world
            .body_mut(id)
            .unwrap()
            .add_force(Vec3::new(10.0, 0.0, 0.0))
            .unwrap();
    }
    step(&mut world, 1);
    for id in [rotated, turned] {
        let direction = mean_velocity_direction(&world, id);
        assert!(direction.x > 0.99, "{direction:?}");
    }
}

#[test]
fn body_level_velocity_torque_and_point_force_refuse_soft_bodies() {
    let mut world = world(GRAVITY, 1);
    let id = loose_cloth(&mut world, SoftBodySettings::default().gravity_factor(0.0));
    let before = world.soft_body(id).unwrap().vertices();
    let mut body = world.body_mut(id).unwrap();
    let refused = Err(BodyError::NotRigidBody(id));
    let push = Vec3::new(1.0, 0.0, 0.0);
    assert_eq!(body.set_linear_velocity(push), refused);
    assert_eq!(body.set_angular_velocity(push), refused);
    assert_eq!(body.add_torque(push), refused);
    assert_eq!(body.add_force_at_point(push, RVec3::ZERO), refused);
    // Refused before the value is looked at.
    assert_eq!(body.add_torque(Vec3::new(f32::NAN, 0.0, 0.0)), refused);
    step(&mut world, 1);
    assert_eq!(world.soft_body(id).unwrap().vertices(), before);

    world.body_mut(id).unwrap().add_force(push).unwrap();
    step(&mut world, 1);
    let after = world.soft_body(id).unwrap().vertices();
    assert!(after.iter().all(|v| v.velocity.x > 0.0));

    // Forces reset after a step and on request.
    world.body_mut(id).unwrap().add_force(push).unwrap();
    world.body_mut(id).unwrap().reset_forces();
    step(&mut world, 1);
    let unchanged = world.soft_body(id).unwrap().vertices();
    for (a, b) in after.iter().zip(&unchanged) {
        assert!(b.velocity.x <= a.velocity.x);
    }
}

#[test]
fn constraints_and_vehicles_refuse_soft_bodies() {
    let mut world = world(GRAVITY, 1);
    let id = loose_cloth(&mut world, SoftBodySettings::default());
    let cube = common::add_cube(&mut world, RVec3::new(3.0, 0.0, 0.0));
    let settings = PointConstraintSettings::new(RVec3::new(1.0, 0.0, 0.0));
    for (a, b) in [(id, cube), (cube, id)] {
        assert_eq!(
            world.create_constraint(a, b, &settings).err(),
            Some(ConstraintError::Body(BodyError::NotRigidBody(id)))
        );
    }
    let vehicle = common::vehicle::car_settings(VehicleCollisionTester::ray(ObjectLayer::MOVING));
    assert_eq!(
        world.create_wheeled_vehicle(id, &vehicle).err(),
        Some(VehicleError::Body(BodyError::NotRigidBody(id)))
    );
}

/// The mean distance of the vertices of soft body `id` from their centroid.
fn mean_radius(world: &PhysicsWorld, id: BodyId) -> f64 {
    let vertices = world.soft_body(id).unwrap().vertices();
    let n = vertices.len() as Real;
    let centre = vertices.iter().fold(RVec3::ZERO, |c, v| {
        RVec3::new(
            c.x + v.position.x / n,
            c.y + v.position.y / n,
            c.z + v.position.z / n,
        )
    });
    vertices
        .iter()
        .map(|v| distance(v.position, centre))
        .sum::<f64>()
        / vertices.len() as f64
}

#[test]
fn a_pressurised_ball_keeps_its_size_on_the_floor() {
    let mut world = world(GRAVITY, 2);
    add_floor(&mut world);
    let (vertices, faces) = sphere(0.5, 8, 12);
    let shared = SoftBodySharedSettings::builder(vertices, faces)
        .create_constraints(
            SoftBodyBendType::None,
            SoftBodyVertexAttributes::default().compliance(1.0e-4),
        )
        .build()
        .unwrap();
    let id = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default()
                .position(RVec3::new(0.0, 1.0, 0.0))
                .pressure(2000.0),
        )
        .unwrap();
    step(&mut world, 180);
    let radius = mean_radius(&world, id);
    assert!((radius - 0.5).abs() <= 0.1, "radius {radius}");
    let lowest = world
        .soft_body(id)
        .unwrap()
        .vertices()
        .iter()
        .map(|v| v.position.y)
        .fold(Real::INFINITY, Real::min);
    assert!(lowest > -0.05, "the ball rests on the floor: {lowest}");
}

#[test]
fn a_soft_body_without_constraints_falls_as_particles() {
    let mut world = world(GRAVITY, 1);
    let id = loose_cloth(
        &mut world,
        SoftBodySettings::default().position(RVec3::new(0.0, 10.0, 0.0)),
    );
    step(&mut world, 30);
    let vertices = world.soft_body(id).unwrap().vertices();
    assert!(vertices
        .iter()
        .all(|v| v.position.y < 10.0 && v.velocity.y < 0.0));
}

#[test]
fn soft_bodies_are_removed_and_a_full_world_refuses_one_cleanly() {
    let mut world = PhysicsWorld::new(WorldSettings::default().max_bodies(2)).unwrap();
    add_floor(&mut world);
    let cloth = Cloth::new(4, 0.2);
    let shared = cloth.builder().build().unwrap();
    let first = world
        .create_soft_body(&shared, &SoftBodySettings::default())
        .unwrap();
    let saved = world.save_state();
    assert_eq!(
        world.create_soft_body(&shared, &SoftBodySettings::default()),
        Err(BodyError::TooManyBodies)
    );
    world.restore_state(&saved).unwrap();
    world.remove_body(first).unwrap();
    assert_eq!(world.body_count(), 1);
    let second = world
        .create_soft_body(&shared, &SoftBodySettings::default())
        .unwrap();
    assert!(world.contains_body(second) && !world.contains_body(first));
}

#[test]
fn queries_find_a_cloth() {
    let mut world = world(GRAVITY, 1);
    let cloth = Cloth::new(5, 0.25);
    let shared = cloth.builder().build().unwrap();
    // Faces point up (+Y), so the ray from above hits their front side.
    let id = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default().position(RVec3::new(0.0, 1.0, 0.0)),
        )
        .unwrap();
    let ray = RayCast::new(RVec3::new(0.1, 3.0, 0.1), Vec3::new(0.0, -5.0, 0.0));
    let hit = world.cast_ray(&ray, &ALL).unwrap().expect("the cloth");
    assert_eq!(hit.body, id);
    assert!(finite(hit.normal));
    assert!((hit.distance - 2.0).abs() < 1.0e-3, "{}", hit.distance);

    let cube = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
    let overlap = CollideShape::new(&cube, RVec3::new(0.0, 1.1, 0.0), Quat::IDENTITY);
    let hits = world.collide_shape(&overlap, &ALL).unwrap();
    assert!(hits.iter().any(|hit| hit.body == id));

    let child = Shape::new_sphere(0.1).unwrap();
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &child,
            position: Vec3::new(0.15, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
        CompoundChild {
            shape: &child,
            position: Vec3::new(-0.15, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 2,
        },
    ])
    .unwrap();
    let cast = ShapeCast::new(
        &compound,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
        Vec3::new(0.0, -2.0, 0.0),
    );
    let hit = world.cast_shape(&cast, &ALL).unwrap().expect("the cloth");
    assert_eq!(hit.body, id);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn one_shared_settings_serves_worlds_on_two_threads() {
    let cloth = Cloth::new(6, 0.2).pin(&[0]);
    let shared = cloth.settings();
    let run = |shared: &SoftBodySharedSettings| {
        let mut world = world(GRAVITY, 1);
        let id = world
            .create_soft_body(
                shared,
                &SoftBodySettings::default().position(RVec3::new(0.0, 2.0, 0.0)),
            )
            .unwrap();
        step(&mut world, 60);
        world.soft_body(id).unwrap().vertices()
    };
    let (a, b) = std::thread::scope(|scope| {
        let a = scope.spawn(|| run(&shared));
        let b = scope.spawn(|| run(&shared));
        (a.join().unwrap(), b.join().unwrap())
    });
    assert_eq!(a, b);
    drop(shared);
}

#[test]
fn pressure_at_the_bound_steps_finitely() {
    let mut world = world(GRAVITY, 2);
    add_floor(&mut world);
    // Vertex masses at both mass bounds: the lightest vertices, and vertices whose masses add up
    // to the total bound.
    let (vertices, faces) = sphere(0.5, 8, 12);
    let count = vertices.len() as f32;
    for inverse_mass in [
        limits::MAX_VERTEX_INVERSE_MASS,
        (count / limits::MAX_MASS).next_up(),
    ] {
        let vertices: Vec<_> = vertices
            .iter()
            .map(|v| SoftBodyVertex { inverse_mass, ..*v })
            .collect();
        let shared = SoftBodySharedSettings::builder(vertices, faces.clone())
            .create_constraints(SoftBodyBendType::Distance, cloth_attributes())
            .build()
            .unwrap();
        let id = world
            .create_soft_body(
                &shared,
                &SoftBodySettings::default()
                    .position(RVec3::new(0.0, 1.0, 0.0))
                    .pressure(limits::MAX_SOFT_BODY_PRESSURE),
            )
            .unwrap();
        step(&mut world, 600);
        let vertices = world.soft_body(id).unwrap().vertices();
        assert!(vertices
            .iter()
            .all(|v| finite_position(v.position) && finite(v.velocity)));
        world.remove_body(id).unwrap();
    }
}

#[test]
fn a_tetrahedral_cube_keeps_its_volume_on_the_floor() {
    let mut world = world(GRAVITY, 2);
    add_floor(&mut world);
    let shared = tetrahedral_cube(1.0).build().unwrap();
    assert_eq!(shared.vertex_count(), 8);
    assert_eq!(shared.face_count(), 12);
    assert_eq!(shared.volume_constraint_count(), 6);
    // 12 cube edges, 6 face diagonals and the body diagonal.
    assert_eq!(shared.edge_constraint_count(), 19);
    let id = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default().position(RVec3::new(0.0, 1.5, 0.0)),
        )
        .unwrap();
    let start = enclosed_volume(&world, id, &CUBE_FACES);
    assert!((start - 1.0).abs() < 1.0e-5, "{start}");
    step(&mut world, 180);
    let volume = enclosed_volume(&world, id, &CUBE_FACES);
    assert!((volume - 1.0).abs() <= 0.1, "volume {volume}");
    let lowest = world
        .soft_body(id)
        .unwrap()
        .vertices()
        .iter()
        .map(|v| v.position.y)
        .fold(Real::INFINITY, Real::min);
    assert!(lowest.abs() < 0.05, "the cube rests on the floor: {lowest}");
}

#[test]
fn explicit_constraints_add_to_the_generated_ones() {
    let cloth = Cloth::new(3, 0.5);
    let generated = cloth
        .builder()
        .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
        .build()
        .unwrap();
    let edge = SoftBodyEdge {
        vertices: [0, 8],
        compliance: 0.0,
    };
    let bend = SoftBodyDihedralBend {
        vertices: [0, 4, 3, 1],
        compliance: 1.0e-3,
    };
    let both = cloth
        .builder()
        .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
        .edge(edge)
        .dihedral_bend(bend)
        .build()
        .unwrap();
    assert_eq!(
        both.edge_constraint_count(),
        generated.edge_constraint_count() + 1
    );
    assert_eq!(both.dihedral_bend_constraint_count(), 1);
    assert_eq!(both.volume_constraint_count(), 0);
}

#[test]
fn explicit_edges_without_faces_hold_a_pendulum() {
    let mut world = world(GRAVITY, 1);
    let vertices = vec![
        SoftBodyVertex::kinematic(Vec3::ZERO),
        SoftBodyVertex::new(Vec3::new(1.0, 0.0, 0.0)),
    ];
    let shared = SoftBodySharedSettings::builder(vertices, Vec::new())
        .edge(SoftBodyEdge {
            vertices: [0, 1],
            compliance: 0.0,
        })
        .build()
        .unwrap();
    assert_eq!(shared.face_count(), 0);
    let id = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default().position(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    step(&mut world, 30);
    let vertices = world.soft_body(id).unwrap().vertices();
    let length = distance(vertices[0].position, vertices[1].position);
    assert!((length - 1.0).abs() < 0.01, "{length}");
    assert!(vertices[1].position.y < 5.0, "the bob swings down");
}

#[test]
fn invalid_explicit_constraints_are_rejected() {
    let cloth = Cloth::new(3, 0.5);
    let invalid = |builder: SoftBodySharedSettingsBuilder| {
        matches!(builder.build(), Err(SoftBodyError::InvalidValue(_)))
    };
    let edge = |vertices, compliance| {
        cloth.builder().edge(SoftBodyEdge {
            vertices,
            compliance,
        })
    };
    assert!(invalid(edge([0, 9], 0.0)));
    assert!(invalid(edge([4, 4], 0.0)));
    assert!(invalid(edge([0, 1], -1.0)));
    assert!(invalid(edge([0, 1], f32::NAN)));
    assert!(invalid(edge([0, 1], limits::MAX_COMPLIANCE.next_up())));
    assert!(!invalid(edge([0, 1], limits::MAX_COMPLIANCE)));
    let bend = |vertices| {
        cloth.builder().dihedral_bend(SoftBodyDihedralBend {
            vertices,
            compliance: 0.0,
        })
    };
    assert!(invalid(bend([0, 4, 3, 3])));
    assert!(invalid(bend([0, 4, 3, 9])));
    assert!(!invalid(bend([0, 4, 3, 1])));
    // A shared edge between two vertices at the same place.
    let mut doubled = cloth.vertices.clone();
    doubled.push(doubled[4]);
    let coincident =
        SoftBodySharedSettings::builder(doubled, Vec::new()).dihedral_bend(SoftBodyDihedralBend {
            vertices: [4, 9, 3, 1],
            compliance: 0.0,
        });
    assert!(invalid(coincident));
    let volume = |vertices| {
        tetrahedral_cube(1.0).volume(SoftBodyVolume {
            vertices,
            compliance: 0.0,
        })
    };
    // Four corners of one face are coplanar.
    assert!(invalid(volume([0, 1, 2, 3])));
    assert!(invalid(volume([0, 1, 3, 3])));
    assert!(invalid(volume([0, 1, 3, 8])));
    assert!(!invalid(volume([0, 1, 3, 7])));
}
