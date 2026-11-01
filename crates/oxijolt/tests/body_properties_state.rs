//! Rollback with the body property setters: Jolt saves none of the values, so a restore keeps
//! the value set last, earlier states stay restorable, and a caller that sets the saved value
//! back replays bit for bit after a detour, with 1 and 4 workers. The filter setters (object
//! layer, collision group) wake the bodies around the changed one, so their value is set back
//! before the restore. The sensor toggle is the exception: it refuses earlier states.

mod common;

use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
/// The tick before which the property is changed in every run.
const CHANGE_TICK: usize = 5;
/// Ticks before the save point.
const BEFORE_SAVE: usize = 20;
/// Ticks of each run after the save point.
const AFTER_SAVE: usize = 60;
/// Ticks of the detour between the save and the restore.
const DETOUR: usize = 20;

/// One body property setter.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Property {
    Friction,
    Restitution,
    GravityFactor,
    LinearDamping,
    AngularDamping,
    MotionQuality,
    Nothing,
}

const PROPERTIES: [Property; 6] = [
    Property::Friction,
    Property::Restitution,
    Property::GravityFactor,
    Property::LinearDamping,
    Property::AngularDamping,
    Property::MotionQuality,
];

/// Which value a setter writes: the one of the run that counts, or the one of the detour.
#[derive(Clone, Copy, PartialEq)]
enum Value {
    Played,
    Detour,
}

/// A floor with a box sliding on it without friction, a ball dropped onto it, a cube created
/// asleep on it, three spheres falling high above it (one moving sideways, one spinning), and a
/// ball shot at a thin wall fast enough to tunnel through it unless it is integrated
/// continuously.
struct Scene {
    world: PhysicsWorld,
    bodies: Vec<BodyId>,
    slider: BodyId,
    ball: BodyId,
    sleeper: BodyId,
    faller: BodyId,
    drifter: BodyId,
    spinner: BodyId,
    bullet: BodyId,
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
        let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
        let sphere = Shape::new_sphere(0.5).unwrap();
        let undamped = |x: Real, y: Real| {
            BodySettings::new_dynamic()
                .position(RVec3::new(x, y, 0.0))
                .linear_damping(0.0)
                .angular_damping(0.0)
        };
        let mut create =
            |shape: &Shape, settings: BodySettings| world.create_body(shape, &settings).unwrap();
        let slider = create(
            &cube,
            undamped(0.0, 0.5)
                .friction(0.0)
                .linear_velocity(Vec3::new(3.0, 0.0, 0.0)),
        );
        let ball = create(
            &sphere,
            BodySettings::new_dynamic().position(RVec3::new(5.0, 3.0, 0.0)),
        );
        let sleeper = create(
            &cube,
            BodySettings::new_dynamic()
                .position(RVec3::new(8.0, 0.5, 0.0))
                .activation(Activation::DontActivate),
        );
        let faller = create(&sphere, undamped(-10.0, 60.0));
        let drifter = create(
            &sphere,
            undamped(-20.0, 60.0).linear_velocity(Vec3::new(2.0, 0.0, 0.0)),
        );
        let spinner = create(
            &sphere,
            undamped(-30.0, 60.0).angular_velocity(Vec3::new(0.0, 5.0, 0.0)),
        );
        // The bullet reaches the wall about five ticks after the save.
        let wall = create(
            &Shape::new_box(Vec3::new(2.0, 2.0, 0.05)).unwrap(),
            BodySettings::new_static().position(RVec3::new(20.0, 50.0, 0.0)),
        );
        let bullet = create(
            &Shape::new_sphere(0.1).unwrap(),
            BodySettings::new_dynamic()
                .position(RVec3::new(20.0, 50.0, -125.0))
                .linear_velocity(Vec3::new(0.0, 0.0, 300.0))
                .gravity_factor(0.0)
                .linear_damping(0.0),
        );
        Self {
            world,
            bodies: vec![
                floor, slider, ball, sleeper, faller, drifter, spinner, wall, bullet,
            ],
            slider,
            ball,
            sleeper,
            faller,
            drifter,
            spinner,
            bullet,
        }
    }

    /// Sets `property` of its body to `value`.
    fn set(&mut self, property: Property, value: Value) {
        let played = value == Value::Played;
        let pick = |played_value: f32, detour_value: f32| {
            if played {
                played_value
            } else {
                detour_value
            }
        };
        let world = &mut self.world;
        let result = match property {
            Property::Friction => world
                .body_mut(self.slider)
                .unwrap()
                .set_friction(pick(1.0, 0.0)),
            Property::Restitution => world
                .body_mut(self.ball)
                .unwrap()
                .set_restitution(pick(0.9, 0.0)),
            Property::GravityFactor => world
                .body_mut(self.faller)
                .unwrap()
                .set_gravity_factor(pick(-0.5, 3.0)),
            Property::LinearDamping => world
                .body_mut(self.drifter)
                .unwrap()
                .set_linear_damping(pick(2.0, 0.0)),
            Property::AngularDamping => world
                .body_mut(self.spinner)
                .unwrap()
                .set_angular_damping(pick(3.0, 0.0)),
            Property::MotionQuality => {
                let quality = if played {
                    MotionQuality::LinearCast
                } else {
                    MotionQuality::Discrete
                };
                world
                    .body_mut(self.bullet)
                    .unwrap()
                    .set_motion_quality(quality)
            }
            Property::Nothing => Ok(()),
        };
        result.unwrap();
    }

    /// Steps once and appends every body's state, its property values and the step's events
    /// to `digest`.
    fn tick(&mut self, digest: &mut Vec<Record>) {
        step(&mut self.world, 1);
        let mut record = Record::default();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut record.states);
            let body = self.world.body(id).unwrap();
            let values = [
                Some(body.friction()),
                Some(body.restitution()),
                body.gravity_factor(),
                body.linear_damping(),
                body.angular_damping(),
            ];
            // Floats print in their shortest exact form.
            record.values += &format!("{values:?}{:?}", body.motion_quality());
        }
        // Body ids print without their world.
        let events = self.world.take_events();
        record
            .states
            .extend(format!("{:?}{:?}", events.contacts, events.activations).bytes());
        digest.push(record);
    }

    /// Runs ticks `ticks`, setting the played value of `property` before [`CHANGE_TICK`].
    fn run(&mut self, property: Property, ticks: std::ops::RangeInclusive<usize>) -> Vec<Record> {
        let mut digest = Vec::new();
        for tick in ticks {
            if tick == CHANGE_TICK {
                self.set(property, Value::Played);
            }
            self.tick(&mut digest);
        }
        digest
    }

    /// A run that differs from the straight one: the detour value of `property`, a teleport,
    /// other gravity, and the sleeper awake.
    fn detour(&mut self, property: Property) {
        self.set(property, Value::Detour);
        self.world
            .body_mut(self.ball)
            .unwrap()
            .set_position(RVec3::new(5.0, 6.0, 0.0), Activation::Activate)
            .unwrap();
        self.world.set_gravity(Vec3::new(1.0, -5.0, 0.0)).unwrap();
        self.world.body_mut(self.sleeper).unwrap().activate();
        let mut ignored = Vec::new();
        for _ in 0..DETOUR {
            self.tick(&mut ignored);
        }
        // The event queue is not part of a state.
        self.world.take_events();
    }
}

/// One tick of a run: what the simulation did (body states and events) and the property values.
#[derive(Debug, Default, PartialEq)]
struct Record {
    states: Vec<u8>,
    values: String,
}

/// The ticks after the save of a straight run with `property` changed.
fn straight(property: Property, threads: u32) -> Vec<Record> {
    let mut scene = Scene::new(threads);
    scene.run(property, 1..=BEFORE_SAVE);
    scene.run(property, BEFORE_SAVE + 1..=BEFORE_SAVE + AFTER_SAVE)
}

/// The same ticks replayed after a save, a detour and a restore, after which the saved value
/// of `property` is set back when `set_back` is true.
fn replay(property: Property, threads: u32, set_back: bool) -> Vec<Record> {
    let mut scene = Scene::new(threads);
    scene.run(property, 1..=BEFORE_SAVE);
    let saved = scene.world.save_state();
    scene.detour(property);
    scene.world.restore_state(&saved).unwrap();
    if set_back {
        scene.set(property, Value::Played);
    }
    scene.run(property, BEFORE_SAVE + 1..=BEFORE_SAVE + AFTER_SAVE)
}

/// The first tick at which `a` and `b` differ.
fn first_difference<T: PartialEq>(a: &[T], b: &[T]) -> Option<usize> {
    a.iter()
        .zip(b)
        .position(|(x, y)| x != y)
        .or_else(|| (a.len() != b.len()).then_some(a.len().min(b.len())))
}

#[test]
fn value_setters_replay_after_a_detour_and_a_set_back() {
    for property in PROPERTIES {
        let reference = straight(property, 1);
        assert_eq!(reference.len(), AFTER_SAVE);
        for threads in [1, 4] {
            assert_eq!(
                first_difference(&reference, &straight(property, threads)),
                None,
                "{property:?}: straight run with {threads} workers"
            );
            assert_eq!(
                first_difference(&reference, &replay(property, threads, true)),
                None,
                "{property:?}: replay with {threads} workers"
            );
        }
    }
}

#[test]
fn a_restore_keeps_the_detour_value() {
    // Without the set-back the replay simulates with the detour's value, so its bodies move
    // differently: the value is not part of the state, and the replay gate above depends on the
    // set-back.
    let states = |run: Vec<Record>| {
        run.into_iter()
            .map(|record| record.states)
            .collect::<Vec<_>>()
    };
    for property in PROPERTIES {
        assert_ne!(
            first_difference(
                &states(straight(property, 1)),
                &states(replay(property, 1, false))
            ),
            None,
            "{property:?}"
        );
    }
}

/// The poses, velocities and sleep flags of every body over a straight run with `property`
/// changed.
fn body_states(property: Property) -> Vec<u8> {
    let mut scene = Scene::new(1);
    let mut digest = Vec::new();
    for tick in 1..=BEFORE_SAVE + AFTER_SAVE {
        if tick == CHANGE_TICK {
            scene.set(property, Value::Played);
        }
        step(&mut scene.world, 1);
        for &id in &scene.bodies {
            record_body(&scene.world, id, &mut digest);
        }
    }
    digest
}

#[test]
fn the_setters_change_the_run() {
    let nothing = body_states(Property::Nothing);
    for property in PROPERTIES {
        assert_ne!(body_states(property), nothing, "{property:?}");
    }
}

#[test]
fn value_setters_keep_earlier_states_restorable() {
    let mut scene = Scene::new(1);
    step(&mut scene.world, 5);
    let saved = scene.world.save_state();
    for property in PROPERTIES {
        scene.set(property, Value::Played);
    }
    let mut slider = scene.world.body_mut(scene.slider).unwrap();
    assert!(slider.set_friction(-1.0).is_err());
    step(&mut scene.world, 5);
    scene.world.restore_state(&saved).unwrap();
    // The restore rewinds the poses and keeps the values set last.
    let slider = scene.world.body(scene.slider).unwrap();
    assert_eq!(slider.friction(), 1.0);
    let bullet = scene.world.body(scene.bullet).unwrap();
    assert_eq!(bullet.motion_quality(), Some(MotionQuality::LinearCast));
}

/// One filter setter of the filter scene, with the activation it passes.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Filter {
    /// The platform's object layer.
    PlatformLayer,
    /// The platform's collision group.
    PlatformGroup,
    /// The object layer of the cube lying on the base cube.
    CubeLayer,
    /// The collision group of the cube lying on the base cube.
    CubeGroup,
    Nothing,
}

const FILTERS: [Filter; 4] = [
    Filter::PlatformLayer,
    Filter::PlatformGroup,
    Filter::CubeLayer,
    Filter::CubeGroup,
];

/// The tick before which the filter is changed in every run of the filter scene.
const FILTER_CHANGE_TICK: usize = 5;
/// Ticks before the save point of the filter scene: long enough for every body the change woke
/// to fall asleep again.
const FILTER_BEFORE_SAVE: usize = 120;
/// Ticks of each filter scene run after the save point.
const FILTER_AFTER_SAVE: usize = 90;
/// The tick before which the base cube is woken, so that the cubes on it collide by their
/// filters again.
const WAKE_TICK: usize = FILTER_BEFORE_SAVE + 30;

/// A floor, a thin static platform above it with a cube created asleep on it and another
/// created asleep under it (overlapping it, on the floor), a ball dropped onto the platform that
/// arrives after the save, and a cube created asleep on a base cube, which is woken after the
/// save. The played filter values
/// let the cubes and the ball pass the platform and the upper cube pass the base cube; the
/// detour values make them collide again.
struct FilterScene {
    world: PhysicsWorld,
    bodies: Vec<BodyId>,
    platform: BodyId,
    rider: BodyId,
    embedded: BodyId,
    base: BodyId,
    top: BodyId,
    moving: ObjectLayer,
    ghost: ObjectLayer,
    /// The platform's played group: it does not collide with `cubes`.
    platform_group: CollisionGroup,
    /// The group of the cubes and the ball.
    cubes: CollisionGroup,
}

impl FilterScene {
    fn new(threads: u32) -> Self {
        let mut layers = CollisionLayers::new(2);
        let fixed = layers.add_object_layer(BroadPhaseLayer::new(0));
        let moving = layers.add_object_layer(BroadPhaseLayer::new(1));
        let ghost = layers.add_object_layer(BroadPhaseLayer::new(1));
        layers.enable_collision(moving, fixed);
        layers.enable_collision(moving, moving);
        layers.enable_collision(ghost, fixed);
        let mut world = PhysicsWorld::new(jobs::with_threads(
            WorldSettings::default().gravity(GRAVITY).layers(layers),
            threads,
        ))
        .unwrap();
        world.set_event_settings(
            EventSettings::default()
                .contacts(true)
                .persisted_contacts(true)
                .body_activation(true),
        );
        let mut builder = GroupFilterTableBuilder::new(2).unwrap();
        builder.disable_collision(0, 1).unwrap();
        let table = builder.build();
        let platform_group = CollisionGroup::new(&table, 1, 0).unwrap();
        let cubes = CollisionGroup::new(&table, 1, 1).unwrap();

        let floor = add_floor(&mut world);
        let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
        let platform = world
            .create_body(
                &Shape::new_box(Vec3::new(1.5, 0.1, 1.5)).unwrap(),
                &BodySettings::new_static().position(RVec3::new(0.0, 0.95, 0.0)),
            )
            .unwrap();
        let mut create = |shape: &Shape, settings: BodySettings| {
            world
                .create_body(shape, &settings.object_layer(moving))
                .unwrap()
        };
        let asleep = |x: Real, y: Real| {
            BodySettings::new_dynamic()
                .position(RVec3::new(x, y, 0.0))
                .activation(Activation::DontActivate)
                .collision_group(cubes.clone())
        };
        let rider = create(&cube, asleep(-0.7, 1.55));
        let embedded = create(&cube, asleep(0.7, 0.5));
        let ball = create(
            &Shape::new_sphere(0.3).unwrap(),
            BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 30.0, 1.0))
                .collision_group(cubes.clone()),
        );
        let base = create(&cube, asleep(10.0, 0.5));
        let top = create(
            &cube,
            BodySettings::new_dynamic()
                .position(RVec3::new(10.0, 1.5, 0.0))
                .activation(Activation::DontActivate),
        );
        Self {
            world,
            bodies: vec![floor, platform, rider, embedded, ball, base, top],
            platform,
            rider,
            embedded,
            base,
            top,
            moving,
            ghost,
            platform_group,
            cubes,
        }
    }

    /// Sets `filter` to `value`.
    fn set(&mut self, filter: Filter, value: Value) {
        let played = value == Value::Played;
        let fixed = ObjectLayer::new(0);
        let world = &mut self.world;
        let result = match filter {
            Filter::PlatformLayer => world.body_mut(self.platform).unwrap().set_object_layer(
                if played { self.ghost } else { fixed },
                Activation::DontActivate,
            ),
            Filter::PlatformGroup => world
                .body_mut(self.platform)
                .unwrap()
                .set_collision_group(played.then_some(&self.platform_group), Activation::Activate),
            Filter::CubeLayer => world.body_mut(self.top).unwrap().set_object_layer(
                if played { self.ghost } else { self.moving },
                Activation::Activate,
            ),
            Filter::CubeGroup => world
                .body_mut(self.top)
                .unwrap()
                .set_collision_group(played.then_some(&self.cubes), Activation::DontActivate),
            Filter::Nothing => Ok(()),
        };
        result.unwrap();
    }

    /// The bodies the played value of `filter` woke, which sleep again at the save point.
    fn sleepers(&self, filter: Filter) -> Vec<BodyId> {
        match filter {
            Filter::PlatformLayer | Filter::PlatformGroup => vec![self.rider, self.embedded],
            Filter::CubeLayer => vec![self.base, self.top],
            Filter::CubeGroup => vec![self.base],
            Filter::Nothing => Vec::new(),
        }
    }

    /// Steps once and appends every body's state, its filters and the step's events to
    /// `digest`.
    fn tick(&mut self, digest: &mut Vec<Record>) {
        step(&mut self.world, 1);
        let mut record = Record::default();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut record.states);
            let body = self.world.body(id).unwrap();
            record.values += &format!(
                "{:?}{}{}{}",
                body.object_layer(),
                body.collision_group_is(None),
                body.collision_group_is(Some(&self.platform_group)),
                body.collision_group_is(Some(&self.cubes)),
            );
        }
        let events = self.world.take_events();
        record
            .states
            .extend(format!("{:?}{:?}", events.contacts, events.activations).bytes());
        digest.push(record);
    }

    /// Runs ticks `ticks`, setting the played value of `filter` before [`FILTER_CHANGE_TICK`]
    /// and waking the base cube before [`WAKE_TICK`].
    fn run(&mut self, filter: Filter, ticks: std::ops::RangeInclusive<usize>) -> Vec<Record> {
        let mut digest = Vec::new();
        for tick in ticks {
            if tick == FILTER_CHANGE_TICK {
                self.set(filter, Value::Played);
            }
            if tick == WAKE_TICK {
                self.world.body_mut(self.base).unwrap().activate();
            }
            self.tick(&mut digest);
        }
        digest
    }

    /// A run that differs from the straight one: the detour value of `filter` and other
    /// gravity.
    fn detour(&mut self, filter: Filter) {
        self.set(filter, Value::Detour);
        self.world.set_gravity(Vec3::new(1.0, -5.0, 0.0)).unwrap();
        let mut ignored = Vec::new();
        for _ in 0..DETOUR {
            self.tick(&mut ignored);
        }
    }
}

/// Where a rollback sets the played filter value back.
#[derive(Clone, Copy, PartialEq)]
enum SetBack {
    BeforeRestore,
    AfterRestore,
    Never,
}

/// The ticks after the save of a straight run of the filter scene with `filter` changed.
fn filter_straight(filter: Filter, threads: u32) -> Vec<Record> {
    let mut scene = FilterScene::new(threads);
    scene.run(filter, 1..=FILTER_BEFORE_SAVE);
    for id in scene.sleepers(filter) {
        assert!(
            scene.world.body(id).unwrap().is_sleeping(),
            "{filter:?}: {id:?} sleeps at the save point"
        );
    }
    scene.run(
        filter,
        FILTER_BEFORE_SAVE + 1..=FILTER_BEFORE_SAVE + FILTER_AFTER_SAVE,
    )
}

/// The same ticks replayed after a save, a detour and a restore, with the played value of
/// `filter` set back as `set_back` says. Events up to the restore are dropped.
fn filter_replay(filter: Filter, threads: u32, set_back: SetBack) -> Vec<Record> {
    let mut scene = FilterScene::new(threads);
    scene.run(filter, 1..=FILTER_BEFORE_SAVE);
    let saved = scene.world.save_state();
    scene.detour(filter);
    if set_back == SetBack::BeforeRestore {
        scene.set(filter, Value::Played);
    }
    scene.world.take_events();
    scene.world.restore_state(&saved).unwrap();
    if set_back == SetBack::AfterRestore {
        scene.set(filter, Value::Played);
    }
    scene.run(
        filter,
        FILTER_BEFORE_SAVE + 1..=FILTER_BEFORE_SAVE + FILTER_AFTER_SAVE,
    )
}

#[test]
fn filter_setters_replay_when_set_back_before_the_restore() {
    for filter in FILTERS {
        let reference = filter_straight(filter, 1);
        assert_eq!(reference.len(), FILTER_AFTER_SAVE);
        for threads in [1, 4] {
            assert_eq!(
                first_difference(&reference, &filter_straight(filter, threads)),
                None,
                "{filter:?}: straight run with {threads} workers"
            );
            assert_eq!(
                first_difference(
                    &reference,
                    &filter_replay(filter, threads, SetBack::BeforeRestore)
                ),
                None,
                "{filter:?}: replay with {threads} workers"
            );
        }
    }
}

#[test]
fn filter_set_back_after_the_restore_or_never_diverges() {
    // Set back after the restore, the set-back's wake stays: the sleepers the saved state put
    // back to sleep are awake in the replay. Without a set-back the replay collides by the
    // detour's filter.
    let states = |run: Vec<Record>| {
        run.into_iter()
            .map(|record| record.states)
            .collect::<Vec<_>>()
    };
    for filter in FILTERS {
        let reference = states(filter_straight(filter, 1));
        let after_restore = states(filter_replay(filter, 1, SetBack::AfterRestore));
        assert_eq!(
            first_difference(&reference, &after_restore),
            Some(0),
            "{filter:?}: the woken sleepers move from the first tick"
        );
        let never = states(filter_replay(filter, 1, SetBack::Never));
        assert_ne!(first_difference(&reference, &never), None, "{filter:?}");
    }
}

#[test]
fn filter_setters_change_the_run() {
    let states = |filter: Filter| {
        let mut scene = FilterScene::new(1);
        let mut digest = Vec::new();
        for record in scene.run(filter, 1..=FILTER_BEFORE_SAVE + FILTER_AFTER_SAVE) {
            digest.extend(record.states);
        }
        digest
    };
    let nothing = states(Filter::Nothing);
    for filter in FILTERS {
        assert_ne!(states(filter), nothing, "{filter:?}");
    }
}

/// A floor with a cube resting on it and a static sensor zone beside it, after 0.5 s.
fn sensor_scene(threads: u32) -> (PhysicsWorld, BodyId, BodyId, BodyId) {
    let mut world = world(GRAVITY, threads);
    world.set_event_settings(
        EventSettings::default()
            .contacts(true)
            .persisted_contacts(true),
    );
    let floor = add_floor(&mut world);
    let zone = world
        .create_body(
            &Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(3.0, 1.0, 0.0))
                .sensor(true),
        )
        .unwrap();
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    step(&mut world, 30);
    (world, floor, zone, cube)
}

/// `ticks` ticks of the bodies' states and contact events.
fn sensor_run(world: &mut PhysicsWorld, ids: &[BodyId], ticks: usize) -> Vec<Vec<u8>> {
    (0..ticks)
        .map(|_| {
            step(world, 1);
            let mut record = Vec::new();
            for &id in ids {
                record_body(world, id, &mut record);
            }
            record.extend(format!("{:?}", world.take_events().contacts).bytes());
            record
        })
        .collect()
}

#[test]
fn a_sensor_toggle_refuses_earlier_states_both_ways() {
    for sensor in [true, false] {
        let (mut world, floor, zone, cube) = sensor_scene(1);
        world
            .body_mut(cube)
            .unwrap()
            .set_motion_quality(MotionQuality::LinearCast)
            .unwrap();
        let id = if sensor { floor } else { zone };
        let saved = world.save_state();
        // The same flag and a refused toggle keep the state restorable.
        world
            .body_mut(id)
            .unwrap()
            .set_is_sensor(!sensor, Activation::Activate)
            .unwrap();
        world.restore_state(&saved).unwrap();
        world
            .body_mut(cube)
            .unwrap()
            .set_is_sensor(true, Activation::Activate)
            .unwrap_err();
        world.restore_state(&saved).unwrap();

        world
            .body_mut(id)
            .unwrap()
            .set_is_sensor(sensor, Activation::Activate)
            .unwrap();
        assert_eq!(world.restore_state(&saved), Err(StateError::WorldChanged));
        world
            .body_mut(id)
            .unwrap()
            .set_is_sensor(!sensor, Activation::Activate)
            .unwrap();
        assert_eq!(world.restore_state(&saved), Err(StateError::WorldChanged));
        assert_eq!(world.body(id).unwrap().is_sensor(), !sensor);
    }
}

#[test]
fn a_state_saved_after_a_sensor_toggle_replays_after_a_detour() {
    for threads in [1, 4] {
        let (mut world, floor, zone, cube) = sensor_scene(threads);
        // Saved between the toggle and the step that discards the zone's cached contacts.
        world
            .body_mut(zone)
            .unwrap()
            .set_is_sensor(false, Activation::Activate)
            .unwrap();
        world
            .body_mut(cube)
            .unwrap()
            .set_linear_velocity(Vec3::new(6.0, 2.0, 0.0))
            .unwrap();
        world.take_events();
        let saved = world.save_state();
        let ids = [floor, zone, cube];
        let first = sensor_run(&mut world, &ids, 90);
        world.restore_state(&saved).unwrap();
        world
            .body_mut(cube)
            .unwrap()
            .set_linear_velocity(Vec3::new(-3.0, 5.0, 1.0))
            .unwrap();
        world.set_gravity(Vec3::new(0.0, -3.0, 0.0)).unwrap();
        sensor_run(&mut world, &ids, 20);
        world.set_gravity(GRAVITY).unwrap();
        world.take_events();
        world.restore_state(&saved).unwrap();
        let replay = sensor_run(&mut world, &ids, 90);
        assert_eq!(first_difference(&first, &replay), None, "{threads} workers");
    }
}
