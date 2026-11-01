//! Rollback with the body controls: every momentary input replays bit for bit after a detour,
//! configuration set at creation (the sensor flag, user data, locked axes, movement capability)
//! survives restores and structural edits, and shape and motion type changes refuse earlier states.

mod common;

use common::ragdoll::mul;
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
/// Ticks before the save point.
const BEFORE_SAVE: usize = 20;
/// Ticks of each run after the save point.
const AFTER_SAVE: usize = 60;
/// Ticks of the detour between the save and the restore.
const DETOUR: usize = 20;
/// The ticks before which the input is given: one before the save, two after it.
const INPUT_TICKS: [usize; 3] = [5, BEFORE_SAVE + 5, BEFORE_SAVE + 30];

/// One momentary input of the body controls.
#[derive(Clone, Copy, Debug)]
enum Input {
    AddImpulse,
    AddAngularImpulse,
    AddImpulseAtPoint,
    MoveKinematic,
    Activate,
    Deactivate,
    ActivateBodiesInBox,
    Nothing,
}

const INPUTS: [Input; 8] = [
    Input::AddImpulse,
    Input::AddAngularImpulse,
    Input::AddImpulseAtPoint,
    Input::MoveKinematic,
    Input::Activate,
    Input::Deactivate,
    Input::ActivateBodiesInBox,
    Input::Nothing,
];

/// A floor, two cubes resting on it, a kinematic platform with an off-centre centre of mass, a
/// static sensor the platform passes, and a cube created asleep.
struct Scene {
    world: PhysicsWorld,
    bodies: Vec<BodyId>,
    cube: BodyId,
    platform: BodyId,
    sleeper: BodyId,
}

impl Scene {
    fn new(threads: u32) -> Self {
        let mut world = world(GRAVITY, threads);
        world.set_event_settings(
            EventSettings::default()
                .contacts(true)
                .persisted_contacts(true)
                .body_activation(true),
        );
        let floor = add_floor(&mut world);
        let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
        let neighbour = add_cube(&mut world, RVec3::new(1.2, 0.5, 0.0));
        let deck = Shape::new_box(Vec3::new(1.0, 0.1, 1.0)).unwrap();
        let off_centre = Shape::new_offset_center_of_mass(&deck, Vec3::new(0.3, 0.0, 0.0)).unwrap();
        let platform = world
            .create_body(
                &off_centre,
                &BodySettings::new_kinematic().position(RVec3::new(-4.0, 1.0, 0.0)),
            )
            .unwrap();
        let sensor = world
            .create_body(
                &Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap(),
                &BodySettings::new_static()
                    .position(RVec3::new(-4.0, 1.0, 3.0))
                    .sensor(true),
            )
            .unwrap();
        let sleeper = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic()
                    .position(RVec3::new(6.0, 0.5, 0.0))
                    .activation(Activation::DontActivate),
            )
            .unwrap();
        Self {
            world,
            bodies: vec![floor, cube, neighbour, platform, sensor, sleeper],
            cube,
            platform,
            sleeper,
        }
    }

    /// Gives `input`, scaled by `variant` where it has a size.
    fn apply(&mut self, input: Input, variant: f32) {
        let cube_at = self.world.body(self.cube).unwrap().position();
        match input {
            Input::AddImpulse => self
                .world
                .body_mut(self.cube)
                .unwrap()
                .add_impulse(Vec3::new(3.0 * variant, 4.0, 0.0))
                .unwrap(),
            Input::AddAngularImpulse => self
                .world
                .body_mut(self.cube)
                .unwrap()
                .add_angular_impulse(Vec3::new(0.0, 0.5 * variant, 0.2))
                .unwrap(),
            Input::AddImpulseAtPoint => {
                let point = RVec3::new(cube_at.x + 0.3, cube_at.y + 0.5, cube_at.z);
                self.world
                    .body_mut(self.cube)
                    .unwrap()
                    .add_impulse_at_point(Vec3::new(0.0, 3.0, variant), point)
                    .unwrap()
            }
            Input::MoveKinematic => {
                let platform = self.world.body(self.platform).unwrap();
                let (at, rotation) = (platform.position(), platform.rotation());
                let target = RVec3::new(at.x + 0.1 * Real::from(variant), 1.0, at.z + 0.05);
                let turn = mul(
                    quat_about(Vec3::new(0.0, 1.0, 0.0), 0.05 * variant),
                    rotation,
                );
                self.world
                    .body_mut(self.platform)
                    .unwrap()
                    .move_kinematic(target, turn, DT)
                    .unwrap()
            }
            Input::Activate => self.world.body_mut(self.sleeper).unwrap().activate(),
            Input::Deactivate => self
                .world
                .body_mut(self.cube)
                .unwrap()
                .deactivate()
                .unwrap(),
            Input::ActivateBodiesInBox => self
                .world
                .activate_bodies_in_box(
                    RVec3::new(5.0, 0.0, -1.0),
                    RVec3::new(5.5 + Real::from(variant.abs()), 1.0, 1.0),
                )
                .unwrap(),
            Input::Nothing => {}
        }
    }

    /// Steps once and appends every body's state and the step's events to `digest`.
    fn tick(&mut self, digest: &mut Vec<Vec<u8>>) {
        step(&mut self.world, 1);
        let mut record = Vec::new();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut record);
        }
        // Body ids print without their world, and floats in their shortest exact form.
        let events = self.world.take_events();
        record.extend(format!("{:?}{:?}", events.contacts, events.activations).bytes());
        digest.push(record);
    }

    /// Runs ticks `ticks`, giving `input` before each tick of [`INPUT_TICKS`].
    fn run(&mut self, input: Input, ticks: std::ops::RangeInclusive<usize>) -> Vec<Vec<u8>> {
        let mut digest = Vec::new();
        for tick in ticks {
            if INPUT_TICKS.contains(&tick) {
                self.apply(input, 1.0);
            }
            self.tick(&mut digest);
        }
        digest
    }

    /// A run that differs from the straight one: other input sizes every fourth tick, a
    /// teleport, other gravity, and other bodies awake.
    fn detour(&mut self, input: Input) {
        self.world
            .body_mut(self.cube)
            .unwrap()
            .set_position(RVec3::new(0.0, 3.0, 0.0), Activation::Activate)
            .unwrap();
        self.world.set_gravity(Vec3::new(1.0, -5.0, 0.0)).unwrap();
        self.world.body_mut(self.sleeper).unwrap().activate();
        let mut ignored = Vec::new();
        for tick in 0..DETOUR {
            if tick % 4 == 0 {
                self.apply(input, -0.5 - tick as f32 / 10.0);
            }
            self.tick(&mut ignored);
        }
        self.world
            .body_mut(self.cube)
            .unwrap()
            .deactivate()
            .unwrap();
        // The event queue is not part of a state.
        self.world.take_events();
    }
}

/// The ticks after the save of a straight run with `input`.
fn straight(input: Input, threads: u32) -> Vec<Vec<u8>> {
    let mut scene = Scene::new(threads);
    scene.run(input, 1..=BEFORE_SAVE);
    scene.run(input, BEFORE_SAVE + 1..=BEFORE_SAVE + AFTER_SAVE)
}

/// The same ticks replayed after a save, a detour and a restore.
fn replay(input: Input, threads: u32) -> Vec<Vec<u8>> {
    let mut scene = Scene::new(threads);
    scene.run(input, 1..=BEFORE_SAVE);
    let saved = scene.world.save_state();
    scene.detour(input);
    scene.world.restore_state(&saved).unwrap();
    scene.run(input, BEFORE_SAVE + 1..=BEFORE_SAVE + AFTER_SAVE)
}

/// The first tick at which `a` and `b` differ.
fn first_difference(a: &[Vec<u8>], b: &[Vec<u8>]) -> Option<usize> {
    a.iter()
        .zip(b)
        .position(|(x, y)| x != y)
        .or_else(|| (a.len() != b.len()).then_some(a.len().min(b.len())))
}

#[test]
fn momentary_inputs_replay_after_a_detour() {
    for input in INPUTS {
        let reference = straight(input, 1);
        assert_eq!(reference.len(), AFTER_SAVE);
        for threads in [1, 4] {
            assert_eq!(
                first_difference(&reference, &straight(input, threads)),
                None,
                "{input:?}: straight run with {threads} workers"
            );
            assert_eq!(
                first_difference(&reference, &replay(input, threads)),
                None,
                "{input:?}: replay with {threads} workers"
            );
        }
    }
}

#[test]
fn the_inputs_change_the_run() {
    // Each input leaves a trace, so the replay gate compares runs that differ.
    let nothing = straight(Input::Nothing, 1);
    for input in INPUTS
        .into_iter()
        .filter(|&input| !matches!(input, Input::Nothing))
    {
        assert_ne!(straight(input, 1), nothing, "{input:?}");
    }
}

/// A box with every creation-only setting: user data, locked axes and movement capability.
fn configured(sensor: bool) -> BodySettings {
    BodySettings::new_dynamic()
        .position(RVec3::new(0.0, 2.0, 0.0))
        .sensor(sensor)
        .user_data(7)
        .allowed_dofs(AllowedDofs::PLANE_2D)
}

/// The configuration getters of `id`.
fn configuration(world: &PhysicsWorld, id: BodyId) -> (bool, u64, AllowedDofs, bool) {
    let body = world.body(id).unwrap();
    (
        body.is_sensor(),
        body.user_data(),
        body.allowed_dofs(),
        body.can_be_kinematic_or_dynamic(),
    )
}

#[test]
fn creation_only_configuration_survives_detours_and_structural_edits() {
    for sensor in [false, true] {
        let mut world = world(GRAVITY, 1);
        add_floor(&mut world);
        let id = world
            .create_body(&cube_shape(), &configured(sensor))
            .unwrap();
        let expected = (sensor, 7, AllowedDofs::PLANE_2D, true);
        let saved = world.save_state();
        let run = |world: &mut PhysicsWorld| {
            (0..120)
                .map(|_| {
                    step(world, 1);
                    let mut record = Vec::new();
                    record_body(world, id, &mut record);
                    record
                })
                .collect::<Vec<_>>()
        };
        let first = run(&mut world);
        assert_eq!(configuration(&world, id), expected);
        world.restore_state(&saved).unwrap();
        let mut body = world.body_mut(id).unwrap();
        body.set_position(RVec3::new(0.0, 10.0, 0.0), Activation::Activate)
            .unwrap();
        body.add_impulse(Vec3::new(2.0, 1.0, 0.0)).unwrap();
        step(&mut world, 3);
        world.restore_state(&saved).unwrap();
        assert_eq!(configuration(&world, id), expected);
        assert_eq!(run(&mut world), first, "sensor: {sensor}");

        // Structural changes keep the configuration and refuse the old state.
        world
            .body_mut(id)
            .unwrap()
            .set_shape(&Shape::new_sphere(0.5).unwrap(), None, Activation::Activate)
            .unwrap();
        world
            .body_mut(id)
            .unwrap()
            .set_motion_type(MotionType::Kinematic, Activation::Activate)
            .unwrap();
        assert_eq!(configuration(&world, id), expected);
        assert_eq!(world.restore_state(&saved), Err(StateError::WorldChanged));
    }
}

/// A structural change of one body.
type BodyChange<'a> = dyn Fn(&mut BodyMut<'_>) -> Result<(), BodyError> + 'a;

#[test]
fn shape_and_motion_type_changes_refuse_earlier_states() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    let wall = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(5.0, 0.5, 0.0)),
        )
        .unwrap();
    let shape = cube_shape();
    let changes: [&BodyChange<'_>; 3] = [
        // The body's own shape: Jolt does not save shapes, so this counts too.
        &|body| body.set_shape(&shape, None, Activation::Activate),
        &|body| body.set_motion_type(MotionType::Kinematic, Activation::Activate),
        &|body| body.set_motion_type(MotionType::Dynamic, Activation::Activate),
    ];
    for change in changes {
        let saved = world.save_state();
        change(&mut world.body_mut(cube).unwrap()).unwrap();
        assert_eq!(world.restore_state(&saved), Err(StateError::WorldChanged));
    }
    // Refused changes keep the state restorable.
    let saved = world.save_state();
    let mut body = world.body_mut(cube).unwrap();
    assert!(body
        .set_shape(&flat_height_field(), None, Activation::Activate)
        .is_err());
    assert!(body
        .set_shape(&shape, Some(0.0), Activation::Activate)
        .is_err());
    body.add_force(Vec3::new(1.0, 0.0, 0.0)).unwrap();
    assert!(body.set_shape(&shape, None, Activation::Activate).is_err());
    body.reset_forces();
    assert_eq!(
        world
            .body_mut(wall)
            .unwrap()
            .set_motion_type(MotionType::Dynamic, Activation::Activate),
        Err(BodyError::CannotMove(wall))
    );
    world.restore_state(&saved).unwrap();
}

#[test]
fn restore_after_deactivate_reports_no_activation_event() {
    let mut world = world(Vec3::ZERO, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let cube = add_cube(&mut world, RVec3::ZERO);
    world
        .body_mut(cube)
        .unwrap()
        .add_impulse(Vec3::new(1.0, 0.0, 0.0))
        .unwrap();
    step(&mut world, 1);
    let saved = world.save_state();
    world.take_events();
    world.body_mut(cube).unwrap().deactivate().unwrap();
    assert_eq!(
        world.take_events().activations,
        [ActivationEvent::Deactivated(cube)]
    );
    assert!(world.take_events().is_empty());
    world.restore_state(&saved).unwrap();
    assert!(world.body(cube).unwrap().is_active());
    assert!(world.take_events().is_empty());
}
