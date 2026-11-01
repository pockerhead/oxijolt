//! One constraint of every kind between the same two bodies.

use oxijolt::*;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// Tries to create a constraint of each of the twelve kinds between `body1` and `body2`, in a
/// fixed order, with its points between the two bodies' positions, and returns each result.
/// Gears, racks and pulleys need two dynamic bodies; with them, unit-sized bodies a metre or two
/// apart and all six degrees of freedom, every kind is accepted.
pub fn create_every_kind(
    world: &mut PhysicsWorld,
    body1: BodyId,
    body2: BodyId,
) -> Vec<Result<AnyConstraintId, ConstraintError>> {
    create_every_kind_solved(world, body1, body2, Solver::default())
}

/// The solver settings every constraint kind shares: priority and velocity and position step
/// overrides.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Solver {
    pub priority: u32,
    pub velocity_steps: u32,
    pub position_steps: u32,
}

/// [`create_every_kind`] with `solver` set on the settings of every kind.
pub fn create_every_kind_solved(
    world: &mut PhysicsWorld,
    body1: BodyId,
    body2: BodyId,
    solver: Solver,
) -> Vec<Result<AnyConstraintId, ConstraintError>> {
    macro_rules! solved {
        ($settings:expr) => {
            &$settings
                .constraint_priority(solver.priority)
                .velocity_steps_override(solver.velocity_steps)
                .position_steps_override(solver.position_steps)
        };
    }
    let p1 = world.body(body1).unwrap().position();
    let p2 = world.body(body2).unwrap().position();
    let middle = RVec3::new(
        (p1.x + p2.x) / 2.0,
        (p1.y + p2.y) / 2.0,
        (p1.z + p2.z) / 2.0,
    );
    let above = |p: RVec3| RVec3::new(p.x, p.y + 2.0, p.z);
    let point = |x: f32| HermitePathPoint {
        position: Vec3::new(x, 0.0, 0.0),
        tangent: X,
    };
    let path = HermitePath::new(Z, vec![point(0.0), point(0.5), point(1.0)], false).unwrap();
    let mut results = Vec::new();
    let mut create = |result: Result<AnyConstraintId, ConstraintError>| results.push(result);
    create(
        world
            .create_constraint(
                body1,
                body2,
                solved!(FixedConstraintSettings::default().auto_detect_point(true)),
            )
            .map(Into::into),
    );
    create(
        world
            .create_constraint(body1, body2, solved!(PointConstraintSettings::new(middle)))
            .map(Into::into),
    );
    create(
        world
            .create_constraint(
                body1,
                body2,
                solved!(DistanceConstraintSettings::new(p1, p2)),
            )
            .map(Into::into),
    );
    create(
        world
            .create_constraint(
                body1,
                body2,
                solved!(HingeConstraintSettings::new(middle, Z, X)),
            )
            .map(Into::into),
    );
    create(
        world
            .create_constraint(
                body1,
                body2,
                solved!(SliderConstraintSettings::new(middle, X, Y)),
            )
            .map(Into::into),
    );
    create(
        world
            .create_constraint(
                body1,
                body2,
                solved!(ConeConstraintSettings::new(middle, X, 0.5)),
            )
            .map(Into::into),
    );
    create(
        world
            .create_constraint(
                body1,
                body2,
                solved!(SwingTwistConstraintSettings::new(middle, X, Y).half_cone_angles(0.5, 0.5)),
            )
            .map(Into::into),
    );
    create(
        world
            .create_constraint(
                body1,
                body2,
                solved!(SixDofConstraintSettings::new(middle, X, Y)),
            )
            .map(Into::into),
    );
    create(
        world
            .create_constraint(body1, body2, solved!(PathConstraintSettings::new(path)))
            .map(Into::into),
    );
    create(
        world
            .create_constraint(
                body1,
                body2,
                solved!(GearConstraintSettings::new(Z, Z, 2.0)),
            )
            .map(Into::into),
    );
    create(
        world
            .create_constraint(
                body1,
                body2,
                solved!(RackAndPinionConstraintSettings::new(Z, X, 2.0)),
            )
            .map(Into::into),
    );
    create(
        world
            .create_constraint(
                body1,
                body2,
                solved!(PulleyConstraintSettings::new(p1, above(p1), p2, above(p2))),
            )
            .map(Into::into),
    );
    results
}
