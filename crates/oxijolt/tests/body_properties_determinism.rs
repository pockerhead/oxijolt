//! Determinism of the body property setters: a scene that changes every property of
//! `BodyMut::set_*` at fixed ticks runs bit for bit the same with 1 and 4 worker threads, in one
//! process and in two. The scene checks that each change does what it promises.

mod common;

use common::controls::contact_kinds;
use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const TICKS: usize = 240;

// Ticks at which the scene changes a property, before that tick's step. The group changes
// while the stacked pair is still awake: Jolt reports a contact removed when its bodies fall
// asleep, so a group change between sleeping bodies ends no contact.
const QUALITY_TICK: usize = 2;
const FRICTION_TICK: usize = 30;
const GROUP_TICK: usize = 20;
const REGROUP_TICK: usize = 32;
const HOPPER_TICK: usize = 100;
const HOPPER_BACK_TICK: usize = 106;
const SUPPORT_TICK: usize = 120;
const SENSOR_TICK: usize = 130;
const SOLID_TICK: usize = 200;

/// The layers of the scene: the floor and static bodies, moving bodies, and a layer that
/// collides with nothing.
struct Layers {
    moving: ObjectLayer,
    ghost: ObjectLayer,
}

fn layered_world(threads: u32) -> (PhysicsWorld, Layers) {
    let mut layers = CollisionLayers::new(2);
    let fixed = layers.add_object_layer(BroadPhaseLayer::new(0));
    let moving = layers.add_object_layer(BroadPhaseLayer::new(1));
    let ghost = layers.add_object_layer(BroadPhaseLayer::new(1));
    layers.enable_collision(moving, fixed);
    layers.enable_collision(moving, moving);
    // The default layers of `BodySettings` and `SoftBodySettings`.
    assert_eq!(
        (fixed, moving),
        (ObjectLayer::NON_MOVING, ObjectLayer::MOVING)
    );
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
    (world, Layers { moving, ghost })
}

fn add_static(world: &mut PhysicsWorld, half: Vec3, position: RVec3) -> BodyId {
    world
        .create_body(
            &Shape::new_box(half).unwrap(),
            &BodySettings::new_static().position(position),
        )
        .unwrap()
}

/// A unit cube created asleep at `position`.
fn add_sleeping_cube(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(position)
                .activation(Activation::DontActivate),
        )
        .unwrap()
}

/// A 0.1 m ball 30 m before a 10 cm static wall at `x`, flying at it at 300 m/s without gravity.
fn add_shooter(world: &mut PhysicsWorld, x: Real, quality: MotionQuality) -> BodyId {
    add_static(world, Vec3::new(2.0, 2.0, 0.05), RVec3::new(x, 20.0, 0.0));
    world
        .create_body(
            &Shape::new_sphere(0.1).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(x, 20.0, -33.0))
                .linear_velocity(Vec3::new(0.0, 0.0, 300.0))
                .gravity_factor(0.0)
                .motion_quality(quality),
        )
        .unwrap()
}

/// A floor; a sleeping pile on a static platform; a box sliding without friction; a ball; two
/// shooters at thin walls; a spinning cube in the air; a sleeping cube on the floor; a sleeping
/// cube on a static trapdoor; a pair of stacked cubes; and a cloth.
struct Scene {
    world: PhysicsWorld,
    layers: Layers,
    bodies: Vec<BodyId>,
    platform: BodyId,
    pile: Vec<BodyId>,
    slider: BodyId,
    ball: BodyId,
    /// Created discrete, switched to `LinearCast`, and the other way round.
    shooters: [BodyId; 2],
    floater: BodyId,
    hopper: BodyId,
    trapdoor: BodyId,
    passenger: BodyId,
    /// The lower and upper cube of the stacked pair; the lower one is in `groups[0]`.
    pair: [BodyId; 2],
    /// Group 1 and group 2 of one table: group 1 excludes itself.
    groups: [CollisionGroup; 2],
    cloth: BodyId,
    checks: Checks,
}

/// What the scene saw of the effects it promises.
#[derive(Default)]
struct Checks {
    slider_speed_before_friction: f32,
    ball_highest_rebound: Real,
    shooters_z: [Real; 2],
    pair_removed_after_group: bool,
    pair_added_after_regroup: bool,
    /// The floater's height when its factor turns negative and when it turns back to 1.
    floater_heights: [Real; 2],
    /// The floater's horizontal and angular speed when its factor turns negative.
    floater_damped_speeds: [f32; 2],
    hopper_sank: bool,
    cloth_start_y: Real,
}

impl Scene {
    fn new(threads: u32) -> Self {
        let (mut world, layers) = layered_world(threads);
        let mut bodies = vec![add_static(
            &mut world,
            Vec3::new(100.0, 1.0, 100.0),
            RVec3::new(0.0, -1.0, 0.0),
        )];
        let platform = add_static(
            &mut world,
            Vec3::new(2.0, 0.25, 2.0),
            RVec3::new(-10.0, 2.75, 0.0),
        );
        let pile: Vec<BodyId> = (0..4)
            .map(|i| {
                let (column, layer) = ((i % 2) as Real, (i / 2) as Real);
                add_sleeping_cube(
                    &mut world,
                    RVec3::new(-10.55 + 1.1 * column, 3.5 + layer, 0.0),
                )
            })
            .collect();
        let slider = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic()
                    .position(RVec3::new(0.0, 0.5, -10.0))
                    .linear_velocity(Vec3::new(5.0, 0.0, 0.0))
                    .friction(0.0)
                    .linear_damping(0.0)
                    .angular_damping(0.0),
            )
            .unwrap();
        let ball = world
            .create_body(
                &Shape::new_sphere(0.5).unwrap(),
                &BodySettings::new_dynamic().position(RVec3::new(0.0, 3.0, 10.0)),
            )
            .unwrap();
        let shooters = [
            add_shooter(&mut world, 20.0, MotionQuality::Discrete),
            add_shooter(&mut world, 25.0, MotionQuality::LinearCast),
        ];
        let floater = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic()
                    .position(RVec3::new(-20.0, 8.0, 0.0))
                    .linear_velocity(Vec3::new(1.0, 0.0, 0.0))
                    .angular_velocity(Vec3::new(1.0, 2.0, 3.0)),
            )
            .unwrap();
        let hopper = add_sleeping_cube(&mut world, RVec3::new(-5.0, 0.5, -10.0));
        let trapdoor = add_static(
            &mut world,
            Vec3::new(1.0, 0.1, 1.0),
            RVec3::new(10.0, 1.9, 0.0),
        );
        let passenger = add_sleeping_cube(&mut world, RVec3::new(10.0, 2.5, 0.0));
        let table = GroupFilterTableBuilder::new(1).unwrap().build();
        let groups = [1, 2].map(|id| CollisionGroup::new(&table, id, 0).unwrap());
        let lower = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic()
                    .position(RVec3::new(6.0, 0.5, 6.0))
                    .collision_group(groups[0].clone()),
            )
            .unwrap();
        let upper = add_cube(&mut world, RVec3::new(6.0, 1.6, 6.0));
        let cloth =
            common::events::add_cloth(&mut world, RVec3::new(-15.0, 2.0, 10.0), Quat::IDENTITY);
        bodies.push(platform);
        bodies.extend(&pile);
        bodies.extend([slider, ball]);
        bodies.extend(shooters);
        bodies.extend([floater, hopper, trapdoor, passenger, lower, upper, cloth]);
        let cloth_start_y = world.body(cloth).unwrap().position().y;
        Self {
            world,
            layers,
            bodies,
            platform,
            pile,
            slider,
            ball,
            shooters,
            floater,
            hopper,
            trapdoor,
            passenger,
            pair: [lower, upper],
            groups,
            cloth,
            checks: Checks {
                cloth_start_y,
                ..Checks::default()
            },
        }
    }

    fn body(&mut self, id: BodyId) -> BodyMut<'_> {
        self.world.body_mut(id).unwrap()
    }

    fn is_sleeping(&self, id: BodyId) -> bool {
        self.world.body(id).unwrap().is_sleeping()
    }

    /// The property changes of `tick`, then one step.
    fn tick(&mut self, tick: usize) {
        let (moving, ghost) = (self.layers.moving, self.layers.ghost);
        match tick {
            QUALITY_TICK => {
                let [discrete, cast] = self.shooters;
                self.body(discrete)
                    .set_motion_quality(MotionQuality::LinearCast)
                    .unwrap();
                self.body(cast)
                    .set_motion_quality(MotionQuality::Discrete)
                    .unwrap();
            }
            5 => {
                let mut floater = self.body(self.floater);
                floater.set_gravity_factor(0.0).unwrap();
                floater.set_linear_damping(1.5).unwrap();
                floater.set_angular_damping(1.5).unwrap();
                let mut cloth = self.body(self.cloth);
                cloth.set_gravity_factor(0.3).unwrap();
                cloth.set_linear_damping(0.5).unwrap();
                cloth.set_friction(0.8).unwrap();
                cloth.set_restitution(0.1).unwrap();
            }
            10 => self.body(self.ball).set_restitution(0.9).unwrap(),
            FRICTION_TICK => {
                self.checks.slider_speed_before_friction =
                    self.world.body(self.slider).unwrap().linear_velocity().x;
                self.body(self.slider).set_friction(1.0).unwrap();
            }
            GROUP_TICK => {
                let group = self.groups[0].clone();
                self.body(self.pair[1])
                    .set_collision_group(Some(&group), Activation::Activate)
                    .unwrap();
            }
            REGROUP_TICK => {
                let group = self.groups[1].clone();
                self.body(self.pair[1])
                    .set_collision_group(Some(&group), Activation::DontActivate)
                    .unwrap();
            }
            80 => self.body(self.floater).set_gravity_factor(-0.5).unwrap(),
            HOPPER_TICK => {
                assert!(self.is_sleeping(self.hopper));
                self.body(self.hopper)
                    .set_object_layer(ghost, Activation::Activate)
                    .unwrap();
                assert!(!self.is_sleeping(self.hopper));
            }
            HOPPER_BACK_TICK => {
                self.checks.hopper_sank = self.world.body(self.hopper).unwrap().position().y < 0.48;
                self.body(self.hopper)
                    .set_object_layer(moving, Activation::DontActivate)
                    .unwrap();
            }
            SUPPORT_TICK => {
                assert!(self.pile.iter().all(|&id| self.is_sleeping(id)));
                self.body(self.platform)
                    .set_object_layer(ghost, Activation::DontActivate)
                    .unwrap();
                let mut floater = self.body(self.floater);
                floater.set_gravity_factor(1.0).unwrap();
                floater.set_linear_damping(0.05).unwrap();
                floater.set_angular_damping(0.05).unwrap();
            }
            SENSOR_TICK => {
                assert!(self.is_sleeping(self.passenger));
                self.body(self.trapdoor)
                    .set_is_sensor(true, Activation::DontActivate)
                    .unwrap();
            }
            150 => self
                .body(self.pair[1])
                .set_collision_group(None, Activation::Activate)
                .unwrap(),
            160 => {
                let [discrete, cast] = self.shooters;
                self.body(discrete)
                    .set_motion_quality(MotionQuality::Discrete)
                    .unwrap();
                self.body(cast)
                    .set_motion_quality(MotionQuality::LinearCast)
                    .unwrap();
            }
            SOLID_TICK => {
                assert!(self.world.body(self.passenger).unwrap().position().y < 1.0);
                self.body(self.trapdoor)
                    .set_is_sensor(false, Activation::Activate)
                    .unwrap();
            }
            _ => {}
        }
        step(&mut self.world, 1);
        self.observe(tick);
    }

    /// Notes the effects of the changes after the step of `tick`.
    fn observe(&mut self, tick: usize) {
        let y = |id| self.world.body(id).unwrap().position().y;
        if (50..120).contains(&tick) {
            self.checks.ball_highest_rebound = self.checks.ball_highest_rebound.max(y(self.ball));
        }
        if tick == 20 {
            self.checks.shooters_z = self
                .shooters
                .map(|id| self.world.body(id).unwrap().position().z);
        }
        if tick == 79 {
            self.checks.floater_heights[0] = y(self.floater);
            let floater = self.world.body(self.floater).unwrap();
            self.checks.floater_damped_speeds = [
                floater.linear_velocity().x,
                length(floater.angular_velocity()),
            ];
        }
        if tick == SUPPORT_TICK - 1 {
            self.checks.floater_heights[1] = y(self.floater);
        }
    }

    /// Every body's state and property values, the cloth's vertices, and the step's events,
    /// whose contacts feed the group checks.
    fn record(&mut self, tick: usize, digest: &mut Digest) {
        let record = digest.push();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut record.state);
            let body = self.world.body(id).unwrap();
            let groups = [None, Some(&self.groups[0]), Some(&self.groups[1])]
                .map(|group| body.collision_group_is(group));
            // Floats print in their shortest exact form.
            let values = format!(
                "{:?} {:?} {:?} {:?} {:?} {:?} {} {:?} {}",
                body.friction(),
                body.restitution(),
                body.gravity_factor(),
                body.linear_damping(),
                body.angular_damping(),
                body.motion_quality(),
                body.object_layer().get(),
                groups,
                body.is_sensor(),
            );
            record.state.extend(values.bytes());
        }
        for vertex in self.world.soft_body(self.cloth).unwrap().vertices() {
            let position: [Real; 3] = vertex.position.into();
            let velocity: [f32; 3] = vertex.velocity.into();
            for value in position {
                record
                    .state
                    .extend_from_slice(&value.to_bits().to_le_bytes());
            }
            for value in velocity {
                record
                    .state
                    .extend_from_slice(&value.to_bits().to_le_bytes());
            }
        }
        // Body ids print without their world.
        let events = self.world.take_events();
        record
            .state
            .extend(format!("{:?}{:?}", events.contacts, events.activations).bytes());
        let [lower, upper] = self.pair;
        let kinds = contact_kinds(&events.contacts, lower, upper);
        if (GROUP_TICK..REGROUP_TICK).contains(&tick) && kinds.contains('r') {
            self.checks.pair_removed_after_group = true;
        }
        if (REGROUP_TICK..REGROUP_TICK + 10).contains(&tick) && kinds.contains('a') {
            self.checks.pair_added_after_regroup = true;
        }
    }

    /// Asserts the effect of every change, at the end of the run.
    fn assert_effects(&self) {
        let checks = &self.checks;
        let body = |id| self.world.body(id).unwrap();
        for &id in &self.pile {
            assert!(
                body(id).position().y < 2.0,
                "the pile fell off its platform"
            );
        }
        assert!(checks.slider_speed_before_friction > 4.5);
        assert!(
            length(body(self.slider).linear_velocity()) < 0.05,
            "friction stopped the box"
        );
        assert!(checks.ball_highest_rebound > 1.5, "the ball bounced");
        let [cast, discrete] = checks.shooters_z;
        assert!(
            cast < 0.0,
            "the LinearCast shooter stopped at its wall: {cast}"
        );
        assert!(discrete > 1.0, "the discrete shooter tunnelled: {discrete}");
        assert!(
            checks.pair_removed_after_group,
            "the group ended the pair's contact"
        );
        assert!(
            checks.pair_added_after_regroup,
            "the other group collides again"
        );
        let [drift, spin] = checks.floater_damped_speeds;
        assert!(
            drift < 0.3 && spin < 1.0,
            "damping slowed the floater: {drift}, {spin}"
        );
        let [hovered, rose] = checks.floater_heights;
        assert!(
            rose > hovered + 0.3,
            "the floater rose with a negative factor"
        );
        assert!(
            body(self.floater).position().y < 1.0,
            "the floater fell back"
        );
        assert!(
            checks.hopper_sank,
            "the hopper fell through the floor as a ghost"
        );
        assert!(
            body(self.passenger).position().y < 1.0,
            "the passenger fell through the sensor"
        );
        assert!(
            body(self.cloth).position().y < checks.cloth_start_y - 1.0,
            "the cloth fell"
        );
    }
}

/// The scene run for [`TICKS`] ticks with `threads` workers.
fn properties(threads: u32) -> Digest {
    let mut scene = Scene::new(threads);
    let mut digest = Digest::new();
    for tick in 1..=TICKS {
        scene.tick(tick);
        scene.record(tick, &mut digest);
    }
    scene.assert_effects();
    digest
}

#[test]
#[ignore = "child process of the body property determinism gate"]
fn body_properties_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "properties");
    finish_child(&properties(threads));
}

#[test]
fn properties_match_with_1_and_4_workers() {
    let one = properties(1);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in one process", &one, &properties(4));
}

#[test]
fn properties_match_across_processes() {
    let one = digest_in_child("body_properties_child", "properties", 1, "");
    let four = digest_in_child("body_properties_child", "properties", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &properties(1), &one);
}
