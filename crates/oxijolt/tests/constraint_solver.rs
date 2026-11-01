//! Per-constraint solver settings: priority and velocity and position step overrides on every
//! constraint kind, their bounds, how Jolt combines them over an island, and rollback.

mod common;

use common::constraint_kinds::{create_every_kind, create_every_kind_solved, Solver};
use common::constraint_rigs::{anchor, cube};
use common::{record_body, world, DT};
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const MAX: u32 = limits::MAX_SOLVER_STEPS_OVERRIDE;

/// Calls the generic function `$typed` with the view type of `$id`'s kind.
macro_rules! dispatch {
    ($id:expr, $typed:ident($($arg:expr),*)) => {
        match $id.kind() {
            ConstraintType::Fixed => $typed::<FixedConstraint>($($arg),*),
            ConstraintType::Point => $typed::<PointConstraint>($($arg),*),
            ConstraintType::Distance => $typed::<DistanceConstraint>($($arg),*),
            ConstraintType::Hinge => $typed::<HingeConstraint>($($arg),*),
            ConstraintType::Slider => $typed::<SliderConstraint>($($arg),*),
            ConstraintType::Cone => $typed::<ConeConstraint>($($arg),*),
            ConstraintType::SwingTwist => $typed::<SwingTwistConstraint>($($arg),*),
            ConstraintType::SixDof => $typed::<SixDofConstraint>($($arg),*),
            ConstraintType::Gear => $typed::<GearConstraint>($($arg),*),
            ConstraintType::RackAndPinion => $typed::<RackAndPinionConstraint>($($arg),*),
            ConstraintType::Pulley => $typed::<PulleyConstraint>($($arg),*),
            ConstraintType::Path => $typed::<PathConstraint>($($arg),*),
            kind => panic!("no constraint view listed for {kind:?}"),
        }
    };
}

/// The solver settings of constraint `id`, read through [`ConstraintRef`].
fn solver_of(world: &PhysicsWorld, id: AnyConstraintId) -> Solver {
    fn typed<K: ConstraintKind>(world: &PhysicsWorld, id: AnyConstraintId) -> Solver {
        let constraint = world.constraint(id.downcast::<K>().unwrap()).unwrap();
        Solver {
            priority: constraint.constraint_priority(),
            velocity_steps: constraint.velocity_steps_override(),
            position_steps: constraint.position_steps_override(),
        }
    }
    dispatch!(id, typed(world, id))
}

/// Sets the solver settings of constraint `id` through [`ConstraintMut`] and returns what the
/// same view reads back, or the first error.
fn set_solver(
    world: &mut PhysicsWorld,
    id: AnyConstraintId,
    solver: Solver,
) -> Result<Solver, ConstraintError> {
    fn typed<K: ConstraintKind>(
        world: &mut PhysicsWorld,
        id: AnyConstraintId,
        solver: Solver,
    ) -> Result<Solver, ConstraintError> {
        let mut constraint = world.constraint_mut(id.downcast::<K>().unwrap()).unwrap();
        constraint.set_velocity_steps_override(solver.velocity_steps)?;
        constraint.set_position_steps_override(solver.position_steps)?;
        constraint.set_constraint_priority(solver.priority);
        Ok(Solver {
            priority: constraint.constraint_priority(),
            velocity_steps: constraint.velocity_steps_override(),
            position_steps: constraint.position_steps_override(),
        })
    }
    dispatch!(id, typed(world, id, solver))
}

/// A world without gravity and two dynamic cubes that every constraint kind accepts.
fn two_cubes() -> (PhysicsWorld, BodyId, BodyId) {
    let mut world = world(Vec3::ZERO, 1);
    let first = common::add_cube(&mut world, RVec3::ZERO);
    let second = common::add_cube(&mut world, RVec3::new(1.5, 0.0, 0.0));
    (world, first, second)
}

#[test]
fn solver_settings_reach_every_kind_at_creation() {
    let (mut world, first, second) = two_cubes();
    for solver in [
        Solver::default(),
        Solver {
            priority: 7,
            velocity_steps: 30,
            position_steps: 1,
        },
        Solver {
            priority: u32::MAX,
            velocity_steps: MAX,
            position_steps: MAX,
        },
    ] {
        let created = create_every_kind_solved(&mut world, first, second, solver);
        assert_eq!(created.len(), 12);
        for id in created {
            let id = id.unwrap();
            assert_eq!(solver_of(&world, id), solver, "{:?}", id.kind());
        }
    }
    common::step(&mut world, 2);
}

#[test]
fn a_rebase_keeps_the_solver_settings() {
    // A rebase recreates pulleys from their live settings; the other kinds stay as they are.
    let (mut world, first, second) = two_cubes();
    let solver = Solver {
        priority: 5,
        velocity_steps: 25,
        position_steps: 3,
    };
    let ids: Vec<_> = create_every_kind_solved(&mut world, first, second, solver)
        .into_iter()
        .map(Result::unwrap)
        .collect();
    world
        .rebase(
            &[first, second],
            Quat::IDENTITY,
            RVec3::new(100.0, 0.0, 0.0),
        )
        .unwrap();
    for id in ids {
        assert_eq!(solver_of(&world, id), solver, "{:?}", id.kind());
    }
}

#[test]
fn live_setters_read_back_on_every_kind() {
    let (mut world, first, second) = two_cubes();
    let ids: Vec<_> = create_every_kind(&mut world, first, second)
        .into_iter()
        .map(Result::unwrap)
        .collect();
    let solver = Solver {
        priority: 3,
        velocity_steps: MAX,
        position_steps: 12,
    };
    for &id in &ids {
        assert_eq!(set_solver(&mut world, id, solver), Ok(solver));
        assert_eq!(solver_of(&world, id), solver, "{:?}", id.kind());
    }
    common::step(&mut world, 2);
    for &id in &ids {
        assert_eq!(
            set_solver(&mut world, id, Solver::default()),
            Ok(Solver::default())
        );
    }
}

#[test]
fn step_overrides_above_the_bound_are_refused() {
    let (mut world, first, second) = two_cubes();
    for solver in [
        Solver {
            velocity_steps: MAX + 1,
            ..Solver::default()
        },
        Solver {
            position_steps: MAX + 1,
            ..Solver::default()
        },
        Solver {
            velocity_steps: u32::MAX,
            position_steps: u32::MAX,
            ..Solver::default()
        },
    ] {
        for result in create_every_kind_solved(&mut world, first, second, solver) {
            assert!(
                matches!(result, Err(ConstraintError::InvalidValue(_))),
                "{solver:?}: {result:?}"
            );
        }
    }
    assert_eq!(world.constraint_count(), 0);

    let ids: Vec<_> = create_every_kind(&mut world, first, second)
        .into_iter()
        .map(Result::unwrap)
        .collect();
    let kept = Solver {
        priority: 0,
        velocity_steps: 20,
        position_steps: 4,
    };
    for &id in &ids {
        set_solver(&mut world, id, kept).unwrap();
        for value in [MAX + 1, u32::MAX] {
            for refused in [
                Solver {
                    velocity_steps: value,
                    ..kept
                },
                Solver {
                    position_steps: value,
                    ..kept
                },
            ] {
                assert!(matches!(
                    set_solver(&mut world, id, refused),
                    Err(ConstraintError::InvalidValue(_))
                ));
            }
        }
        assert_eq!(solver_of(&world, id), kept, "{:?}", id.kind());
    }
}

/// Cubes in the welded cantilever.
const LINKS: usize = 8;
/// Distance between the centres of neighbouring cubes, metres: the cubes (half extent 0.25) and
/// the anchor (half extent 0.1 at the origin of the row) never touch.
const SPACING: Real = 0.6;
/// Mass of each link and of the load at the free end, kg.
const LINK_MASS: f32 = 1.0;
const LOAD_MASS: f32 = 20.0;
/// Height of the cantilever's static anchor, metres.
const HEIGHT: Real = 10.0;

/// A horizontal cantilever: [`LINKS`] cubes in a row along +X from a static anchor, each welded
/// to the one before at the middle of the gap between them, the last one carrying the load.
struct Cantilever {
    world: PhysicsWorld,
    links: Vec<BodyId>,
    welds: Vec<ConstraintId<FixedConstraint>>,
}

impl Cantilever {
    /// The cantilever with `solver(i)` on weld `i`, counted from the anchor.
    fn new(threads: u32, solver: impl Fn(usize) -> Solver) -> Self {
        let mut world = world(GRAVITY, threads);
        let mut previous = anchor(&mut world, RVec3::new(0.0, HEIGHT, 0.0));
        let mut links = Vec::new();
        let mut welds = Vec::new();
        for i in 0..LINKS {
            let x = SPACING * (i + 1) as Real;
            let mass = if i + 1 == LINKS { LOAD_MASS } else { LINK_MASS };
            let link = cube(&mut world, RVec3::new(x, HEIGHT, 0.0), mass);
            let solver = solver(i);
            let point = RVec3::new(x - SPACING / 2.0, HEIGHT, 0.0);
            let weld = FixedConstraintSettings::new(point, X, Y)
                .constraint_priority(solver.priority)
                .velocity_steps_override(solver.velocity_steps)
                .position_steps_override(solver.position_steps);
            welds.push(world.create_constraint(previous, link, &weld).unwrap());
            links.push(link);
            previous = link;
        }
        Self {
            world,
            links,
            welds,
        }
    }

    /// How far the load hangs below the anchor's height now, metres.
    fn sag(&self) -> f64 {
        let load = self.world.body(*self.links.last().unwrap()).unwrap();
        common::math::wide(HEIGHT - load.position().y)
    }

    /// Steps once and appends every link's state to `digest`.
    fn tick(&mut self, digest: &mut Vec<u8>) {
        assert!(self.world.step(DT).unwrap().is_complete());
        for &link in &self.links {
            record_body(&self.world, link, digest);
        }
    }

    /// Every link's state after each of `ticks` steps.
    fn run(&mut self, ticks: usize) -> Vec<u8> {
        let mut digest = Vec::new();
        for _ in 0..ticks {
            self.tick(&mut digest);
        }
        digest
    }
}

/// The same override on every weld.
fn every_weld(solver: Solver) -> impl Fn(usize) -> Solver {
    move |_| solver
}

/// `solver` on the weld at the anchor only.
fn first_weld(solver: Solver) -> impl Fn(usize) -> Solver {
    move |i| if i == 0 { solver } else { Solver::default() }
}

fn velocity_steps(steps: u32) -> Solver {
    Solver {
        velocity_steps: steps,
        ..Solver::default()
    }
}

/// The load's mean sag over the second of two seconds.
fn mean_sag(solver: impl Fn(usize) -> Solver) -> f64 {
    let mut cantilever = Cantilever::new(1, solver);
    cantilever.run(60);
    let mut total = 0.0;
    for _ in 0..60 {
        cantilever.run(1);
        total += cantilever.sag();
    }
    total / 60.0
}

#[test]
fn a_velocity_steps_override_reduces_the_sag_of_a_loaded_weld_chain() {
    let default = mean_sag(every_weld(Solver::default()));
    let overridden = mean_sag(every_weld(velocity_steps(100)));
    println!("mean sag: default {default} m, 100 velocity steps {overridden} m");
    assert!(overridden.is_finite() && default.is_finite());
    assert!(
        overridden < 0.5 * default,
        "default {default} m, 100 steps {overridden} m"
    );
}

#[test]
fn the_island_uses_the_largest_override() {
    let digest = |solver: &dyn Fn(usize) -> Solver| Cantilever::new(1, solver).run(60);
    let default = digest(&every_weld(Solver::default()));
    // One weld with an override raises the count of the whole island.
    assert_eq!(
        digest(&first_weld(velocity_steps(40))),
        digest(&every_weld(velocity_steps(40)))
    );
    assert_ne!(digest(&every_weld(velocity_steps(40))), default);
    // An override below the world's 10 counts only when no constraint of the island has none.
    assert_eq!(digest(&first_weld(velocity_steps(4))), default);
    assert_ne!(digest(&every_weld(velocity_steps(4))), default);
    // The same holds for position steps.
    let position_steps = |steps| Solver {
        position_steps: steps,
        ..Solver::default()
    };
    assert_eq!(
        digest(&first_weld(position_steps(20))),
        digest(&every_weld(position_steps(20)))
    );
    assert_ne!(digest(&every_weld(position_steps(20))), default);
    assert_eq!(digest(&first_weld(position_steps(1))), default);
    assert_ne!(digest(&every_weld(position_steps(1))), default);
}

/// Ticks before the change, before the save, after the save, and of the detour.
const CHANGE_TICK: usize = 5;
const BEFORE_SAVE: usize = 20;
const AFTER_SAVE: usize = 60;
const DETOUR: usize = 20;

/// The solver settings of the run that counts, on every weld.
const PLAYED: Solver = Solver {
    priority: 3,
    velocity_steps: 40,
    position_steps: 6,
};
/// The solver settings of the detour a rollback abandons.
const DETOURED: Solver = Solver {
    priority: 9,
    velocity_steps: 3,
    position_steps: 0,
};

impl Cantilever {
    fn set_every_weld(&mut self, solver: Solver) {
        for &weld in &self.welds {
            let mut weld = self.world.constraint_mut(weld).unwrap();
            weld.set_velocity_steps_override(solver.velocity_steps)
                .unwrap();
            weld.set_position_steps_override(solver.position_steps)
                .unwrap();
            weld.set_constraint_priority(solver.priority);
        }
    }

    /// Runs to the save point with [`PLAYED`] set before [`CHANGE_TICK`].
    fn run_to_save(&mut self) {
        let mut ignored = Vec::new();
        for tick in 1..=BEFORE_SAVE {
            if tick == CHANGE_TICK {
                self.set_every_weld(PLAYED);
            }
            self.tick(&mut ignored);
        }
    }
}

/// The ticks after the save of a straight run.
fn straight(threads: u32) -> Vec<u8> {
    let mut cantilever = Cantilever::new(threads, every_weld(Solver::default()));
    cantilever.run_to_save();
    cantilever.run(AFTER_SAVE)
}

/// The same ticks replayed after a save, a detour with [`DETOURED`] and a kicked load, and a
/// restore, with [`PLAYED`] set back before the restore when `set_back` is true.
fn replay(threads: u32, set_back: bool) -> Vec<u8> {
    let mut cantilever = Cantilever::new(threads, every_weld(Solver::default()));
    cantilever.run_to_save();
    let saved = cantilever.world.save_state();
    cantilever.set_every_weld(DETOURED);
    let load = *cantilever.links.last().unwrap();
    cantilever
        .world
        .body_mut(load)
        .unwrap()
        .add_impulse(Vec3::new(0.0, 30.0, 10.0))
        .unwrap();
    cantilever.run(DETOUR);
    if set_back {
        cantilever.set_every_weld(PLAYED);
    }
    cantilever.world.restore_state(&saved).unwrap();
    cantilever.run(AFTER_SAVE)
}

#[test]
fn solver_settings_replay_after_a_detour_and_a_set_back() {
    let reference = straight(1);
    for threads in [1, 4] {
        assert_eq!(
            straight(threads),
            reference,
            "straight run, {threads} workers"
        );
        assert_eq!(
            replay(threads, true),
            reference,
            "replay, {threads} workers"
        );
    }
    // Jolt does not save the settings: without the set-back the replay runs with the detour's.
    assert_ne!(replay(1, false), reference);
}
