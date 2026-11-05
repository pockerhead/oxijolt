//! Character settings changed while the character runs: read back, refused when invalid, their
//! effect on the ground state, the inner body and the weight check, shape swaps that fit or do
//! not, and a replay of live changes.

mod common;

use std::sync::Arc;

use common::math::{bits, v3};
use common::meshes::grid;
use common::*;
use oxijolt::*;

const RADIUS: f32 = 0.4;
const HALF_HEIGHT: f32 = 0.70845;
const FOOT_OFFSET: Vec3 = Vec3::new(0.0, HALF_HEIGHT + RADIUS, 0.0);
const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

fn capsule() -> Shape {
    Shape::new_capsule(HALF_HEIGHT, RADIUS).unwrap()
}

/// A world with a floor and a refreshed character standing on it, with an inner body when
/// `inner` is given.
fn standing(shape: &Shape, inner: Option<&Shape>) -> (PhysicsWorld, CharacterId) {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let settings = CharacterSettings::new(shape)
        .shape_offset(FOOT_OFFSET)
        .inner_body(inner.map(|shape| InnerBody {
            shape,
            object_layer: ObjectLayer::MOVING,
        }));
    let id = world
        .create_character(&settings, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    (world, id)
}

fn still_update(world: &mut PhysicsWorld, id: CharacterId, gravity: Vec3, delta_time: f32) {
    world
        .character_mut(id)
        .unwrap()
        .set_linear_velocity(Vec3::ZERO)
        .unwrap();
    world
        .update_character(
            id,
            delta_time,
            gravity,
            &ExtendedUpdateSettings::default(),
            &QueryFilter::new(),
        )
        .unwrap();
}

#[test]
fn live_settings_read_back() {
    let shape = capsule();
    let inner_shape = capsule();
    let (mut world, id) = standing(&shape, Some(&inner_shape));
    let mut character = world.character_mut(id).unwrap();
    character.set_mass(80.0).unwrap();
    character.set_max_strength(250.0).unwrap();
    character
        .set_max_slope_angle(30.0_f32.to_radians())
        .unwrap();
    character
        .set_supporting_volume(Vec3::new(0.0, 1.0, 0.0), -0.3)
        .unwrap();
    character
        .set_shape_offset(Vec3::new(0.0, 1.0, 0.1))
        .unwrap();
    character.set_hit_reduction_cos_max_angle(-1.0).unwrap();
    character.set_enhanced_internal_edge_removal(true);
    character.set_max_num_hits(12).unwrap();
    character.set_user_data(0xfeed_beef);

    let character = world.character(id).unwrap();
    assert_eq!(character.mass(), 80.0);
    assert_eq!(character.max_strength(), 250.0);
    assert_eq!(
        character.cos_max_slope_angle().to_bits(),
        30.0_f32.to_radians().cos().to_bits()
    );
    let (normal, constant) = character.supporting_volume();
    assert_eq!(bits(normal), bits(Vec3::new(0.0, 1.0, 0.0)));
    assert_eq!(constant, -0.3);
    assert_eq!(
        bits(character.shape_offset()),
        bits(Vec3::new(0.0, 1.0, 0.1))
    );
    assert_eq!(character.hit_reduction_cos_max_angle(), -1.0);
    assert!(character.enhanced_internal_edge_removal());
    assert_eq!(character.max_num_hits(), 12);
    assert_eq!(character.user_data(), 0xfeed_beef);
    let inner = character.inner_body().unwrap();
    assert_eq!(world.body(inner).unwrap().user_data(), 0xfeed_beef);
}

/// Everything the setters below can change, read back.
fn settings_of(world: &PhysicsWorld, id: CharacterId) -> Vec<u64> {
    let character = world.character(id).unwrap();
    let (normal, constant) = character.supporting_volume();
    let mut values = vec![
        character.mass().to_bits().into(),
        character.max_strength().to_bits().into(),
        character.cos_max_slope_angle().to_bits().into(),
        constant.to_bits().into(),
        character.hit_reduction_cos_max_angle().to_bits().into(),
        character.max_num_hits().into(),
    ];
    values.extend(bits(normal).map(u64::from));
    values.extend(bits(character.shape_offset()).map(u64::from));
    values.extend(v3(character.position()).map(f64::to_bits));
    values
}

#[test]
fn invalid_live_values_are_rejected_without_change() {
    let shape = capsule();
    let (mut world, id) = standing(&shape, None);
    let before = settings_of(&world, id);
    let plane = Shape::new_plane(Vec3::new(0.0, 1.0, 0.0), 0.0, 10.0).unwrap();
    let mut character = world.character_mut(id).unwrap();
    let invalid = |result: Result<(), CharacterError>| {
        assert!(
            matches!(result, Err(CharacterError::InvalidValue(_))),
            "{result:?}"
        )
    };
    for mass in [-1.0, f32::NAN, limits::MAX_MASS * 2.0, f32::INFINITY] {
        invalid(character.set_mass(mass));
    }
    for strength in [-1.0, f32::NAN, f32::INFINITY] {
        invalid(character.set_max_strength(strength));
    }
    for angle in [-0.1, 1.6, f32::NAN] {
        invalid(character.set_max_slope_angle(angle));
    }
    invalid(character.set_supporting_volume(Vec3::new(0.0, 2.0, 0.0), 0.0));
    invalid(character.set_supporting_volume(Vec3::new(0.0, f32::NAN, 0.0), 0.0));
    invalid(character.set_supporting_volume(Vec3::new(0.0, 1.0, 0.0), f32::INFINITY));
    invalid(character.set_shape_offset(Vec3::new(0.0, limits::MAX_SHAPE_EXTENT * 2.0, 0.0)));
    invalid(character.set_shape_offset(Vec3::new(f32::NAN, 0.0, 0.0)));
    invalid(character.set_hit_reduction_cos_max_angle(f32::NAN));
    invalid(character.set_max_num_hits(0));
    invalid(
        character
            .set_shape(&plane, 0.1, &QueryFilter::new())
            .map(drop),
    );
    invalid(
        character
            .set_shape(&shape, -0.1, &QueryFilter::new())
            .map(drop),
    );
    invalid(
        character
            .set_shape(&shape, f32::NAN, &QueryFilter::new())
            .map(drop),
    );
    let missing = [ObjectLayer::new(7)];
    assert!(matches!(
        character.set_shape(&shape, 0.1, &QueryFilter::new().object_layers(&missing)),
        Err(CharacterError::Query(_))
    ));
    assert_eq!(
        character.set_inner_body_shape(&shape),
        Err(CharacterError::NoInnerBody(id))
    );
    assert_eq!(settings_of(&world, id), before);

    let inner_shape = capsule();
    let (mut world, id) = standing(&shape, Some(&inner_shape));
    let state = world.save_state();
    let (vertices, triangles) = grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let mut character = world.character_mut(id).unwrap();
    invalid(character.set_inner_body_shape(&plane));
    // A mesh has no mass, so a kinematic body has no inverse inertia.
    invalid(character.set_inner_body_shape(&mesh));
    // Nothing changed: the state saved before is still restorable.
    world.restore_state(&state).unwrap();
    let mut character = world.character_mut(id).unwrap();
    // The bounds themselves are accepted.
    character.set_mass(limits::MAX_MASS).unwrap();
    character
        .set_shape_offset(Vec3::new(0.0, limits::MAX_SHAPE_EXTENT, 0.0))
        .unwrap();
    character
        .set_max_slope_angle(std::f32::consts::FRAC_PI_2)
        .unwrap();
    character.set_max_strength(0.0).unwrap();
    assert_eq!(world.character(id).unwrap().mass(), limits::MAX_MASS);
}

#[test]
fn a_live_mass_is_what_the_weight_check_uses() {
    let shape = capsule();
    let (mut world, id) = standing(&shape, None);
    // 70 kg * 1e4 m/s² * 1 s is within `MAX_WEIGHT_IMPULSE`, 1e6 kg is not.
    let heavy_gravity = Vec3::new(0.0, -1.0e4, 0.0);
    still_update(&mut world, id, heavy_gravity, 1.0);
    world.character_mut(id).unwrap().set_mass(1.0e6).unwrap();
    let position = world.character(id).unwrap().position();
    let refused = world.update_character(
        id,
        1.0,
        heavy_gravity,
        &ExtendedUpdateSettings::default(),
        &QueryFilter::new(),
    );
    assert!(matches!(refused, Err(CharacterError::InvalidValue(_))));
    assert_eq!(world.character(id).unwrap().position(), position);
    world.character_mut(id).unwrap().set_mass(70.0).unwrap();
    still_update(&mut world, id, heavy_gravity, 1.0);
}

/// A character resting on a 10° slope, refreshed.
fn on_a_gentle_slope(shape: &Shape) -> (PhysicsWorld, CharacterId) {
    let mut world = world(GRAVITY, 1);
    let angle = 10.0_f32.to_radians();
    let (sin, cos) = (angle * 0.5).sin_cos();
    let slope = Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap();
    world
        .create_body(
            &slope,
            &BodySettings::new_static().rotation(Quat::from_xyzw(0.0, 0.0, sin, cos)),
        )
        .unwrap();
    let top = 0.5 / angle.cos();
    let id = world
        .create_character(
            &CharacterSettings::new(shape).shape_offset(FOOT_OFFSET),
            RVec3::new(0.0, top as Real, 0.0),
            Quat::IDENTITY,
        )
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    (world, id)
}

#[test]
fn a_live_slope_limit_and_supporting_volume_change_the_ground_state() {
    let shape = capsule();
    let (mut world, id) = on_a_gentle_slope(&shape);
    let refresh = |world: &mut PhysicsWorld| {
        world
            .refresh_character_contacts(id, &QueryFilter::new())
            .unwrap();
        world.character(id).unwrap().ground_state()
    };
    assert_eq!(refresh(&mut world), GroundState::OnGround);
    let mut character = world.character_mut(id).unwrap();
    character.set_max_slope_angle(5.0_f32.to_radians()).unwrap();
    assert_eq!(refresh(&mut world), GroundState::OnSteepGround);
    world
        .character_mut(id)
        .unwrap()
        .set_max_slope_angle(50.0_f32.to_radians())
        .unwrap();
    assert_eq!(refresh(&mut world), GroundState::OnGround);

    // A plane below the feet that no contact lies under: nothing supports the character.
    world
        .character_mut(id)
        .unwrap()
        .set_supporting_volume(Vec3::new(0.0, 1.0, 0.0), 10.0)
        .unwrap();
    assert!(!refresh(&mut world).is_supported());
    world
        .character_mut(id)
        .unwrap()
        .set_supporting_volume(Vec3::new(0.0, 1.0, 0.0), -1.0e10)
        .unwrap();
    assert_eq!(refresh(&mut world), GroundState::OnGround);
}

#[test]
fn a_live_shape_offset_moves_the_inner_body() {
    let shape = capsule();
    let inner_shape = capsule();
    let (mut world, id) = standing(&shape, Some(&inner_shape));
    let inner = world.character(id).unwrap().inner_body().unwrap();
    let before = v3(world.body(inner).unwrap().position());
    world
        .character_mut(id)
        .unwrap()
        .set_shape_offset(Vec3::new(0.0, FOOT_OFFSET.y + 0.5, 0.0))
        .unwrap();
    let after = v3(world.body(inner).unwrap().position());
    assert!(
        (after[1] - before[1] - 0.5).abs() < 1e-5,
        "{before:?} -> {after:?}"
    );
    assert_eq!(after[0], before[0]);
}

#[test]
fn a_live_inner_body_shape_is_what_queries_hit() {
    let shape = capsule();
    let inner_shape = capsule();
    let (mut world, id) = standing(&shape, Some(&inner_shape));
    let inner = world.character(id).unwrap().inner_body().unwrap();
    // A ray along x at a height the capsule's sides cover, 0.6 m beside the centre line.
    let ray = RayCast::new(RVec3::new(-5.0, 1.1, 0.6), Vec3::new(10.0, 0.0, 0.0));
    let hit = |world: &PhysicsWorld| {
        world
            .cast_ray(&ray, &QueryFilter::new())
            .unwrap()
            .map(|hit| hit.body)
    };
    assert_eq!(hit(&world), None);
    let wide = Shape::new_box(Vec3::new(0.4, 1.0, 1.0)).unwrap();
    world
        .character_mut(id)
        .unwrap()
        .set_inner_body_shape(&wide)
        .unwrap();
    drop(wide);
    assert_eq!(hit(&world), Some(inner));
}

#[test]
fn update_ground_velocity_reads_the_platform_after_the_step() {
    let mut world = world(GRAVITY, 1);
    let platform_shape = Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap();
    let platform = world
        .create_body(
            &platform_shape,
            &BodySettings::new_kinematic().position(RVec3::new(0.0, -0.5, 0.0)),
        )
        .unwrap();
    world
        .body_mut(platform)
        .unwrap()
        .set_linear_velocity(Vec3::new(1.0, 0.0, 0.0))
        .unwrap();
    let shape = capsule();
    let id = world
        .create_character(
            &CharacterSettings::new(&shape).shape_offset(FOOT_OFFSET),
            RVec3::ZERO,
            Quat::IDENTITY,
        )
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    assert_eq!(world.character(id).unwrap().ground_body(), Some(platform));
    let ground_x = |world: &PhysicsWorld| world.character(id).unwrap().ground_velocity().x;
    assert!((ground_x(&world) - 1.0).abs() < 1e-5);

    world
        .body_mut(platform)
        .unwrap()
        .set_linear_velocity(Vec3::new(3.0, 0.0, 0.0))
        .unwrap();
    assert!(world.step(DT).unwrap().is_complete());
    assert!((ground_x(&world) - 1.0).abs() < 1e-5, "one step old");
    world.character_mut(id).unwrap().update_ground_velocity();
    assert!((ground_x(&world) - 3.0).abs() < 1e-5);
}

/// Reports the body with user data 1 as a belt moving at 2 m/s along x.
struct Conveyor;

impl CharacterContactListener for Conveyor {
    fn adjust_body_velocity(
        &self,
        _: CharacterId,
        _: BodyId,
        user_data: u64,
        velocity: &mut BodyVelocity,
    ) {
        if user_data == 1 {
            velocity
                .set_linear_velocity(Vec3::new(2.0, 0.0, 0.0))
                .unwrap();
        }
    }
}

#[test]
fn update_ground_velocity_asks_the_listener_as_an_update_does() {
    let mut world = world(GRAVITY, 1);
    world.set_character_contact_listener(Some(Arc::new(Conveyor)));
    let belt = Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap();
    world
        .create_body(
            &belt,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -0.5, 0.0))
                .user_data(1),
        )
        .unwrap();
    let shape = capsule();
    let id = world
        .create_character(
            &CharacterSettings::new(&shape).shape_offset(FOOT_OFFSET),
            RVec3::ZERO,
            Quat::IDENTITY,
        )
        .unwrap();
    still_update(&mut world, id, GRAVITY, DT);
    let ground = |world: &PhysicsWorld| bits(world.character(id).unwrap().ground_velocity());
    assert_eq!(ground(&world), bits(Vec3::new(2.0, 0.0, 0.0)));
    assert!(world.step(DT).unwrap().is_complete());
    world.character_mut(id).unwrap().update_ground_velocity();
    assert_eq!(ground(&world), bits(Vec3::new(2.0, 0.0, 0.0)));
}

#[test]
fn max_num_hits_set_live_bounds_the_contacts() {
    let shape = capsule();
    let (mut world, id) = standing(&shape, None);
    // A wall touching the capsule's side.
    let wall = Shape::new_box(Vec3::new(0.5, 2.0, 2.0)).unwrap();
    world
        .create_body(
            &wall,
            &BodySettings::new_static().position(RVec3::new(RADIUS as Real + 0.5, 2.0, 0.0)),
        )
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    assert!(world.character(id).unwrap().active_contacts().len() >= 2);
    assert!(!world.character(id).unwrap().max_hits_exceeded());
    world
        .character_mut(id)
        .unwrap()
        .set_max_num_hits(1)
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    assert!(world.character(id).unwrap().max_hits_exceeded());
}

const CROUCH_HALF_HEIGHT: f32 = 0.2;
const STAND_HALF_HEIGHT: f32 = 0.8;
/// The centre of both capsules above the character position (plus the padding 0.02).
const CENTRE: f32 = 0.6;

/// A crouching character floating without gravity below a ceiling whose bottom is at
/// `ceiling_bottom`. A standing capsule's top is at 1.82, a crouching one's at 1.22.
fn under_a_ceiling(crouch: &Shape, ceiling_bottom: Real) -> (PhysicsWorld, CharacterId, BodyId) {
    let mut world = world(Vec3::ZERO, 1);
    let ceiling_shape = Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap();
    let ceiling = world
        .create_body(
            &ceiling_shape,
            &BodySettings::new_static().position(RVec3::new(0.0, ceiling_bottom + 0.5, 0.0)),
        )
        .unwrap();
    let settings = CharacterSettings::new(crouch).shape_offset(Vec3::new(0.0, CENTRE, 0.0));
    let id = world
        .create_character(&settings, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    (world, id, ceiling)
}

fn touches(world: &PhysicsWorld, id: CharacterId, body: BodyId) -> bool {
    world
        .character(id)
        .unwrap()
        .active_contacts()
        .iter()
        .any(|contact| contact.body == Some(body))
}

#[test]
fn a_shape_that_does_not_fit_keeps_the_old_one() {
    let crouch = Shape::new_capsule(CROUCH_HALF_HEIGHT, RADIUS).unwrap();
    let (mut world, id, ceiling) = under_a_ceiling(&crouch, 1.5);
    assert!(!touches(&world, id, ceiling));
    let stand = Shape::new_capsule(STAND_HALF_HEIGHT, RADIUS).unwrap();
    let fits = world
        .character_mut(id)
        .unwrap()
        .set_shape(&stand, 0.05, &QueryFilter::new())
        .unwrap();
    assert!(!fits);
    // The crouching capsule is still the shape: its top stays out of contact range.
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    assert!(!touches(&world, id, ceiling));

    // A filter that leaves the ceiling out lets the standing capsule in, as does an infinite
    // depth. Either way it is the shape afterwards.
    let fits = world
        .character_mut(id)
        .unwrap()
        .set_shape(&stand, 0.05, &QueryFilter::new().exclude_body(ceiling))
        .unwrap();
    assert!(fits);
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    assert!(touches(&world, id, ceiling));
    assert!(world
        .character_mut(id)
        .unwrap()
        .set_shape(&crouch, 0.05, &QueryFilter::new())
        .unwrap());
    assert!(
        !touches(&world, id, ceiling),
        "the swap replaced the contacts"
    );
    assert!(world
        .character_mut(id)
        .unwrap()
        .set_shape(&stand, f32::INFINITY, &QueryFilter::new())
        .unwrap());
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    assert!(touches(&world, id, ceiling));
}

#[test]
fn a_shape_that_fits_replaces_the_contacts_and_is_held_by_the_character() {
    let crouch = Shape::new_capsule(CROUCH_HALF_HEIGHT, RADIUS).unwrap();
    // The standing capsule's top comes within the predictive distance of the ceiling.
    let (mut world, id, ceiling) = under_a_ceiling(&crouch, 1.85);
    assert!(!touches(&world, id, ceiling));
    {
        let stand = Shape::new_capsule(STAND_HALF_HEIGHT, RADIUS).unwrap();
        let fits = world
            .character_mut(id)
            .unwrap()
            .set_shape(&stand, 0.0, &QueryFilter::new())
            .unwrap();
        assert!(fits);
    }
    // The caller's shape is gone; the character's own reference keeps it alive.
    assert!(touches(&world, id, ceiling), "contacts of the new shape");
    for _ in 0..10 {
        world
            .character_mut(id)
            .unwrap()
            .set_linear_velocity(Vec3::new(0.5, 0.0, 0.0))
            .unwrap();
        world
            .update_character(
                id,
                DT,
                Vec3::ZERO,
                &ExtendedUpdateSettings::default(),
                &QueryFilter::new(),
            )
            .unwrap();
    }
    assert!(touches(&world, id, ceiling));
}

#[test]
fn a_shape_swap_makes_earlier_world_states_unrestorable() {
    let crouch = Shape::new_capsule(CROUCH_HALF_HEIGHT, RADIUS).unwrap();
    let (mut world, id, _) = under_a_ceiling(&crouch, 3.0);
    let state = world.save_state();
    let stand = Shape::new_capsule(STAND_HALF_HEIGHT, RADIUS).unwrap();
    assert!(world
        .character_mut(id)
        .unwrap()
        .set_shape(&stand, 0.0, &QueryFilter::new())
        .unwrap());
    assert_eq!(world.restore_state(&state), Err(StateError::WorldChanged));
}

#[test]
fn a_refused_or_repeated_shape_swap_keeps_earlier_world_states_restorable() {
    let crouch = Shape::new_capsule(CROUCH_HALF_HEIGHT, RADIUS).unwrap();
    let (mut world, id, _) = under_a_ceiling(&crouch, 1.5);
    let state = world.save_state();
    let stand = Shape::new_capsule(STAND_HALF_HEIGHT, RADIUS).unwrap();
    let mut character = world.character_mut(id).unwrap();
    assert!(!character
        .set_shape(&stand, 0.05, &QueryFilter::new())
        .unwrap());
    assert!(character
        .set_shape(&crouch, 0.05, &QueryFilter::new())
        .unwrap());
    world.restore_state(&state).unwrap();
}

#[test]
fn an_inner_body_shape_change_makes_earlier_world_states_unrestorable() {
    let shape = capsule();
    let inner_shape = capsule();
    let (mut world, id) = standing(&shape, Some(&inner_shape));
    let state = world.save_state();
    let wide = Shape::new_box(Vec3::new(0.4, 1.0, 1.0)).unwrap();
    world
        .character_mut(id)
        .unwrap()
        .set_inner_body_shape(&wide)
        .unwrap();
    assert_eq!(world.restore_state(&state), Err(StateError::WorldChanged));
}

/// One world of the replay: a floor, a cube and a character with an inner body.
fn replay_world(worker_threads: u32, shapes: &[Shape; 3]) -> (PhysicsWorld, CharacterId) {
    let mut world = world(GRAVITY, worker_threads);
    add_floor(&mut world);
    add_cube(&mut world, RVec3::new(3.0, 2.0, 0.0));
    let settings = CharacterSettings::new(&shapes[0])
        .shape_offset(FOOT_OFFSET)
        .inner_body(Some(InnerBody {
            shape: &shapes[0],
            object_layer: ObjectLayer::MOVING,
        }));
    let id = world
        .create_character(&settings, RVec3::new(0.0, 0.5, 0.0), Quat::IDENTITY)
        .unwrap();
    (world, id)
}

/// The live changes of tick `tick`, an update and a step.
fn replay_tick(world: &mut PhysicsWorld, id: CharacterId, shapes: &[Shape; 3], tick: usize) {
    let mut character = world.character_mut(id).unwrap();
    match tick {
        10 => character.set_mass(120.0).unwrap(),
        20 => character
            .set_max_slope_angle(20.0_f32.to_radians())
            .unwrap(),
        30 => character
            .set_shape_offset(Vec3::new(0.0, FOOT_OFFSET.y + 0.1, 0.0))
            .unwrap(),
        40 => {
            assert!(character
                .set_shape(&shapes[1], 0.1, &QueryFilter::new())
                .unwrap());
            character.set_inner_body_shape(&shapes[1]).unwrap();
        }
        50 => character.set_max_num_hits(8).unwrap(),
        60 => character
            .set_supporting_volume(Vec3::new(0.0, 1.0, 0.0), -0.2)
            .unwrap(),
        70 => character.set_hit_reduction_cos_max_angle(0.99).unwrap(),
        80 => character.set_enhanced_internal_edge_removal(true),
        // Reaches lower than the current shape: it would sink into the floor.
        90 => assert!(!character
            .set_shape(&shapes[2], 0.1, &QueryFilter::new())
            .unwrap()),
        _ => {}
    }
    let phase = tick as f32 * 0.1;
    character
        .set_linear_velocity(Vec3::new(phase.cos() * 1.5, -1.0, phase.sin()))
        .unwrap();
    world
        .update_character(
            id,
            DT,
            GRAVITY,
            &ExtendedUpdateSettings::default(),
            &QueryFilter::new(),
        )
        .unwrap();
    assert!(world.step(DT).unwrap().is_complete());
    world.character_mut(id).unwrap().update_ground_velocity();
}

#[test]
fn live_changes_replay_bit_for_bit_for_any_thread_count() {
    let shapes = [
        capsule(),
        Shape::new_capsule(HALF_HEIGHT * 0.5, RADIUS).unwrap(),
        Shape::new_capsule(HALF_HEIGHT * 0.75, RADIUS * 0.9).unwrap(),
    ];
    let run = |worker_threads| {
        let (mut world, id) = replay_world(worker_threads, &shapes);
        let mut digest = Vec::new();
        for tick in 0..120 {
            replay_tick(&mut world, id, &shapes, tick);
            let character = world.character(id).unwrap();
            digest.extend(character.save_state().to_bytes());
            digest.extend(
                bits(character.ground_velocity())
                    .map(u32::to_le_bytes)
                    .concat(),
            );
            let inner = character.inner_body().unwrap();
            for c in v3(world.body(inner).unwrap().position()) {
                digest.extend(c.to_bits().to_le_bytes());
            }
        }
        digest
    };
    let reference = run(1);
    assert_eq!(run(1), reference);
    assert_eq!(run(3), reference);
}
