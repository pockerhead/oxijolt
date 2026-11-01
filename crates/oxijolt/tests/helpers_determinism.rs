//! Determinism of the physics helpers: bodies floating in a pool on a plane floor, with
//! buoyancy under a radial gravity, point queries every tick, and collision estimates of a rod
//! and a fast ball thrown at the floor, run bit for bit the same with 1 and 4 worker threads, in
//! one process and in two, and after a rollback with a detour.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const TICKS: usize = 240;

/// A plane floor flooded by a shallow pool, a grid of mixed bodies dropped into it, a cube
/// asleep on the floor beside them, and a rod and a ball with continuous collision detection.
struct Scene {
    world: PhysicsWorld,
    /// Every body, in creation order: the floor, the grid, the sleeper, the rod, the ball.
    bodies: Vec<BodyId>,
    grid: Vec<BodyId>,
    sleeper: BodyId,
    rod: BodyId,
    ball: BodyId,
    /// Whether the ball was thrown in the step about to be recorded.
    ball_thrown: bool,
    /// Added contacts of the rod with an estimate.
    rod_estimates: usize,
    /// Added contacts of a thrown ball with an estimate, which only the continuous stage finds.
    continuous_estimates: usize,
    /// The pool's buoyancy factor.
    buoyancy: f32,
    /// How many point query hits the run recorded.
    point_hits: usize,
    /// How many buoyancy calls found their body under water.
    submerged: usize,
}

/// The pool's surface height.
const POOL_SURFACE: Real = 0.3;

/// Gravity toward a centre 2 km below the origin, as a radial gravity field gives it at
/// `position`.
#[allow(clippy::unnecessary_cast)] // `Real` is `f32` without the `double-precision` feature.
fn radial_gravity(position: RVec3) -> Vec3 {
    let to_centre = [-position.x, -2000.0 - position.y, -position.z].map(|c| c as f64);
    let length = to_centre.iter().map(|c| c * c).sum::<f64>().sqrt();
    let [x, y, z] = to_centre.map(|c| (9.81 * c / length) as f32);
    Vec3::new(x, y, z)
}

/// The shapes the grid cycles through: box, sphere, hull and compound.
fn mixed_shapes() -> Vec<Shape> {
    let block = Shape::new_box(Vec3::new(0.3, 0.2, 0.25)).unwrap();
    let ball = Shape::new_sphere(0.25).unwrap();
    let hull = Shape::new_convex_hull_with_convex_radius(
        &[
            Vec3::new(-0.3, 0.0, -0.3),
            Vec3::new(0.3, 0.0, -0.3),
            Vec3::new(0.0, 0.0, 0.3),
            Vec3::new(0.0, 0.5, 0.0),
        ],
        0.02,
    )
    .unwrap();
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &block,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 0,
        },
        CompoundChild {
            shape: &ball,
            position: Vec3::new(0.4, 0.2, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
    ])
    .unwrap();
    vec![block, ball, hull, compound]
}

impl Scene {
    fn new(threads: u32) -> Self {
        let mut world = world(GRAVITY, threads);
        world.set_event_settings(EventSettings::default().collision_estimates(true));
        let floor = world
            .create_body(
                &Shape::new_plane(Vec3::new(0.0, 1.0, 0.0), 0.0, 30.0).unwrap(),
                &BodySettings::new_static(),
            )
            .unwrap();
        let mut bodies = vec![floor];
        let shapes = mixed_shapes();
        let mut grid = Vec::new();
        for i in 0..16 {
            let (column, row) = ((i % 4) as Real, (i / 4) as Real);
            let tilt = quat_about(Vec3::new(0.6, 0.0, 0.8), 0.3 * i as f32);
            grid.push(
                world
                    .create_body(
                        &shapes[i % shapes.len()],
                        &BodySettings::new_dynamic()
                            .position(RVec3::new(
                                1.2 * column - 1.8,
                                1.0 + 0.4 * row,
                                1.2 * row - 1.8,
                            ))
                            .rotation(tilt)
                            .angular_velocity(Vec3::new(0.0, 1.0, 0.0)),
                    )
                    .unwrap(),
            );
        }
        let sleeper = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic()
                    .position(RVec3::new(8.0, 0.5, 8.0))
                    .activation(Activation::DontActivate),
            )
            .unwrap();
        let fast = |position| {
            BodySettings::new_dynamic()
                .position(position)
                .motion_quality(MotionQuality::LinearCast)
        };
        let rod = world
            .create_body(
                &Shape::new_box(Vec3::new(0.5, 0.05, 0.05)).unwrap(),
                &fast(RVec3::new(-8.0, 0.05, -8.0)),
            )
            .unwrap();
        let ball = world
            .create_body(
                &Shape::new_sphere(0.05).unwrap(),
                &fast(RVec3::new(8.0, 0.05, -8.0)),
            )
            .unwrap();
        bodies.extend(&grid);
        bodies.extend([sleeper, rod, ball]);
        Self {
            world,
            bodies,
            grid,
            sleeper,
            rod,
            ball,
            ball_thrown: false,
            rod_estimates: 0,
            continuous_estimates: 0,
            buoyancy: 1.5,
            point_hits: 0,
            submerged: 0,
        }
    }

    /// Throws the rod tilted from 1 mm above the floor at 30 m/s (its lower end touches in the
    /// discrete stage), or the ball from 2 m above it at 200 m/s (only the continuous stage
    /// finds it), at fixed ticks.
    fn throw(&mut self, tick: usize) {
        let tilt: f32 = 20.0_f32.to_radians();
        let (body, position, rotation, speed) = match tick % 120 {
            30 => {
                let lowest = 0.5 * tilt.sin() + 0.05 * tilt.cos();
                let at = RVec3::new(-8.0, Real::from(lowest) + 0.001, -8.0);
                (
                    self.rod,
                    at,
                    quat_about(Vec3::new(0.0, 0.0, 1.0), tilt),
                    30.0,
                )
            }
            40 => {
                self.ball_thrown = true;
                (self.ball, RVec3::new(8.0, 2.0, -8.0), Quat::IDENTITY, 200.0)
            }
            _ => return,
        };
        let mut body = self.world.body_mut(body).unwrap();
        body.set_position_and_rotation(position, rotation, Activation::Activate)
            .unwrap();
        body.set_angular_velocity(Vec3::ZERO).unwrap();
        body.set_linear_velocity(Vec3::new(0.0, -speed, 0.0))
            .unwrap();
    }

    /// The throws of `tick`, then [`float`](Self::float).
    fn tick(&mut self, tick: usize) {
        self.throw(tick);
        self.float();
    }

    /// Buoyancy on every grid body with the gravity at its position, then one step.
    fn float(&mut self) {
        let pool = BuoyancySettings::default()
            .surface(RVec3::new(0.0, POOL_SURFACE, 0.0), Vec3::new(0.0, 1.0, 0.0))
            .buoyancy(self.buoyancy)
            .linear_drag(1.0)
            .angular_drag(0.1);
        for &id in &self.grid {
            let mut body = self.world.body_mut(id).unwrap();
            let gravity = radial_gravity(body.position());
            if body.apply_buoyancy_impulse(&pool, gravity, DT).unwrap() {
                self.submerged += 1;
            }
        }
        step(&mut self.world, 1);
    }

    /// Different inputs for a while: another pool, kicks and the sleeper woken.
    fn detour_tick(&mut self, tick: usize) {
        self.buoyancy = 3.0;
        if tick.is_multiple_of(10) {
            let kicked = self.grid[tick % self.grid.len()];
            self.world
                .body_mut(kicked)
                .unwrap()
                .add_impulse(Vec3::new(0.0, 50.0, 20.0))
                .unwrap();
        }
        if tick == 50 {
            self.world.body_mut(self.sleeper).unwrap().activate();
        }
        self.float();
        self.world.take_events();
    }

    fn record(&mut self, digest: &mut Digest) {
        let tick = digest.push();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut tick.state);
        }
        // Contact events print floats in their shortest exact form.
        for event in self.world.take_events().contacts {
            if let ContactEvent::Added {
                estimate: Some(_), ..
            } = &event
            {
                let bodies = [event.pair().body1, event.pair().body2];
                self.rod_estimates += usize::from(bodies.contains(&self.rod));
                self.continuous_estimates +=
                    usize::from(self.ball_thrown && bodies.contains(&self.ball));
            }
            tick.state.extend(format!("{event:?}").bytes());
        }
        self.ball_thrown = false;
        // Points where the first two grid rows land, and the last one below the floor.
        for i in 0..8 {
            let y = if i == 7 { -0.5 } else { 0.15 };
            let point = RVec3::new(1.2 * (i % 4) as Real - 1.8, y, 1.2 * (i / 4) as Real - 1.8);
            let hits = self
                .world
                .collide_point(point, &QueryFilter::new())
                .unwrap();
            self.point_hits += hits.len();
            for hit in hits {
                tick.state.extend(
                    format!("{}:{};", hit.body.to_raw(), hit.sub_shape_id.to_raw()).bytes(),
                );
            }
            tick.state.push(b'|');
        }
    }
}

/// What a run saw besides its digest.
struct Counts {
    point_hits: usize,
    submerged: usize,
    rod_estimates: usize,
    continuous_estimates: usize,
}

/// The scene run for [`TICKS`] ticks with `threads` workers.
fn run(threads: u32) -> (Digest, Counts) {
    let mut scene = Scene::new(threads);
    let mut digest = Digest::new();
    for tick in 0..TICKS {
        scene.tick(tick);
        scene.record(&mut digest);
    }
    let counts = Counts {
        point_hits: scene.point_hits,
        submerged: scene.submerged,
        rod_estimates: scene.rod_estimates,
        continuous_estimates: scene.continuous_estimates,
    };
    (digest, counts)
}

fn helpers(threads: u32) -> Digest {
    run(threads).0
}

/// The scene run with a rollback: saved at `SAVE_TICK`, sent on a detour of different inputs,
/// restored, and replayed with the original inputs; the digests of the replayed ticks.
fn replayed(threads: u32) -> Digest {
    const SAVE_TICK: usize = 60;
    let mut scene = Scene::new(threads);
    let mut digest = Digest::new();
    for tick in 0..SAVE_TICK {
        scene.tick(tick);
        scene.record(&mut digest);
    }
    let saved = scene.world.save_state();
    for tick in 0..60 {
        scene.detour_tick(tick);
    }
    assert!(!scene.world.body(scene.sleeper).unwrap().is_sleeping());
    let positions = |world: &PhysicsWorld| -> Vec<RVec3> {
        scene
            .grid
            .iter()
            .map(|&id| world.body(id).unwrap().position())
            .collect()
    };
    let detoured = positions(&scene.world);
    scene.world.restore_state(&saved).unwrap();
    assert_ne!(positions(&scene.world), detoured);
    scene.buoyancy = 1.5;
    for tick in SAVE_TICK..TICKS {
        scene.tick(tick);
        scene.record(&mut digest);
    }
    digest
}

#[test]
#[ignore = "child process of the helpers determinism gate"]
fn helpers_determinism_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "helpers");
    finish_child(&helpers(threads));
}

#[test]
fn helpers_match_with_1_and_4_workers() {
    let (one, counts) = run(1);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in one process", &one, &helpers(4));
    assert_ne!(one.ticks[0].state, one.ticks[TICKS - 1].state);
    // More than the floor below every tick: the points find landed bodies too.
    assert!(counts.point_hits > TICKS, "{}", counts.point_hits);
    // Most grid bodies float in the pool most of the time.
    assert!(counts.submerged > 8 * TICKS, "{}", counts.submerged);
    // Both throws of each body are estimated.
    assert!(counts.rod_estimates >= 2, "{}", counts.rod_estimates);
    assert_eq!(counts.continuous_estimates, 2);
}

#[test]
fn a_rollback_replays_buoyancy_queries_and_estimates_exactly() {
    for threads in [1, 4] {
        assert_same(
            "straight run vs rollback and replay",
            &helpers(threads),
            &replayed(threads),
        );
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn helpers_match_across_processes() {
    let one = digest_in_child("helpers_determinism_child", "helpers", 1, "");
    let four = digest_in_child("helpers_determinism_child", "helpers", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &helpers(1), &one);
}
