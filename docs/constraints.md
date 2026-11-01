# Constraints

A constraint joins two bodies of a world: a hinge for a door, a slider for a piston, a gear between
two wheels. `PhysicsWorld::create_constraint(body1, body2, &settings)` creates one from a settings
value and returns a `ConstraintId<K>` typed by its kind, which selects the motor, target, limit and
readout methods of `ConstraintRef<K>` (`world.constraint(id)`) and `ConstraintMut<K>`
(`world.constraint_mut(id)`). The world owns the constraint until `remove_constraint` or until the
world is dropped.

## Kinds

| Settings | Jolt constraint | What it does |
|---|---|---|
| `FixedConstraintSettings` | `FixedConstraint` | welds two bodies |
| `PointConstraintSettings` | `PointConstraint` | a ball joint |
| `DistanceConstraintSettings` | `DistanceConstraint` | keeps two points at a distance or within a range, optionally as a spring |
| `HingeConstraintSettings` | `HingeConstraint` | one rotation axis, with limits, a motor and friction |
| `SliderConstraintSettings` | `SliderConstraint` | one translation axis, with limits, a motor and friction |
| `ConeConstraintSettings` | `ConeConstraint` | a ball joint whose twist axis stays in a cone |
| `SwingTwistConstraintSettings` | `SwingTwistConstraint` | swing and twist limits and motors, for shoulders and necks |
| `SixDofConstraintSettings` | `SixDOFConstraint` | each of the six axes free, limited or fixed, with motors and springs |
| `GearConstraintSettings` | `GearConstraint` | couples the rotation of two hinged bodies |
| `RackAndPinionConstraintSettings` | `RackAndPinionConstraint` | couples a hinged body's rotation to a sliding body's translation |
| `PulleyConstraintSettings` | `PulleyConstraint` | a rope over two fixed points between two bodies |
| `PathConstraintSettings` | `PathConstraint` | keeps a body on a `HermitePath`, with a motor along it |

The swing-twist, hinge and six-DOF settings also serve as ragdoll joints (`RagdollJoint`). Angles
are in radians, torques in N·m, forces in N; the defaults are Jolt's.

## Two gears

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO))?;
    let z = Vec3::new(0.0, 0.0, 1.0);
    let x = Vec3::new(1.0, 0.0, 0.0);

    let anchor = Shape::new_box(Vec3::new(0.1, 0.1, 0.1))?;
    let base = world.create_body(
        &anchor,
        &BodySettings::new_static().position(RVec3::new(0.0, -10.0, 0.0)),
    )?;

    // Two discs on hinges about z, held by the static base.
    let disc = Shape::new_box(Vec3::new(0.5, 0.5, 0.1))?;
    let mut hinged_disc = |at: Real| -> Result<_, Box<dyn std::error::Error>> {
        let centre = RVec3::new(at, 0.0, 0.0);
        let body = world.create_body(&disc, &BodySettings::new_dynamic().position(centre))?;
        let hinge_settings = HingeConstraintSettings::new(centre, z, x);
        let hinge = world.create_constraint(base, body, &hinge_settings)?;
        Ok((body, hinge))
    };
    let (disc1, hinge1) = hinged_disc(0.0)?;
    let (disc2, hinge2) = hinged_disc(3.0)?;

    // Gear 2 turns half as fast as gear 1, the other way. The hinges let Jolt correct drift.
    let gear = GearConstraintSettings::new(z, z, 2.0).constraints(hinge1, hinge2);
    world.create_constraint(disc1, disc2, &gear)?;

    // Drive gear 1 at 2 rad/s with the hinge's velocity motor.
    let mut motor = world.constraint_mut(hinge1)?;
    motor.set_target_angular_velocity(2.0)?;
    motor.set_motor_state(MotorState::Velocity);
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    let spin = world.body(disc2)?.angular_velocity().z;
    assert!((spin + 1.0).abs() < 0.02);
    Ok(())
}
```

## Frames

Each constraint attaches a frame to each body: a point and two perpendicular unit axes. With
`ConstraintSpace::WorldSpace` (the default) the frames are given in world space at the moment the
constraint is created, and Jolt turns them into each body's own frame then. With
`ConstraintSpace::LocalToBodyCom` they are relative to each body's centre of mass. In a constraint's
frame X is the twist or hinge axis, and Y and Z are the swing axes.

## Bodies

- Both bodies belong to the world and are different. Neither may be a character's inner body, a
  ragdoll part or a soft body (`ConstraintError::Body`).
- Except for a gear, a rack and pinion and a pulley, one body may be static or kinematic, which
  anchors the constraint to the world or to the kinematic body. Those three need two dynamic bodies
  (`ConstraintError::NotDynamic`): Jolt 5.6's solver parts for them read both bodies' motion
  properties without checking, so a static body breaks them and a kinematic one gets pushed. A
  pulley's fixed points already anchor its rope.
- Jolt still lets the two bodies collide where their shapes touch; leave a gap, or put the bodies in
  object layers that do not collide.
- While a constraint exists, `remove_body` refuses its bodies (`BodyError::UsedByConstraint`).
  `constraints_of_body` lists the constraints that hold a body.
- A new constraint, every setter and `remove_constraint` wake the constraint's bodies that can move,
  so a sleeping body follows the change at the next step.

## Limits on what is accepted

The settings are checked before anything reaches Jolt, and a failing call changes nothing.

- **Lever arm.** Each point where a constraint holds a dynamic body must have a lever-arm ratio of
  at most `limits::MAX_LEVER_ARM_RATIO` (1000): its distance from the body's centre of mass,
  measured against the body's size. A door on a hinge at its edge, a weld at the surface of a part
  and a pendulum bob of radius `a` up to about `14 · a` from its pivot pass; a longer pendulum is a
  distance constraint whose points lie on the bodies ([limits.md](limits.md#lever-arm-ratio)).
- **Springs.** A spring given as frequency and damping becomes a stiffness and damping that grow
  with the bodies' effective mass; both are bounded ([limits.md](limits.md#springs)).
- **Gear ratios** lie within `1..=limits::MAX_GEAR_RATIO` (10), body 2 being the gear that turns
  slower; swap the bodies for a gear that speeds up, and chain gears for a larger reduction. Both
  bounds come from a defect in Jolt 5.6's `GearConstraintPart`, which applies the impulse to body 2
  without the ratio: below 1 the velocity error can grow without bound, and above 10 the gear needs
  ever more steps to restore its relation after a disturbance
  ([limits.md](limits.md#coupling-ratios)). For the same reason the torque passed to body 2 is not
  `ratio` times the torque on body 1; the rotation rates follow the ratio.
- **Rack and pinion ratios**, in radians per metre, have a magnitude within
  `1 / limits::MAX_RATIO..=limits::MAX_RATIO`. A negative ratio reverses the direction of the
  coupling.
- **Pulley ratios** are positive and within `1 / limits::MAX_RATIO..=limits::MAX_RATIO`.

## Solver overrides

The settings of every kind take three solver settings of Jolt's `ConstraintSettings`, and a live
constraint reads them through `ConstraintRef` and `ConstraintMut` and changes them through
`ConstraintMut`:

- `velocity_steps_override` and `position_steps_override` (`set_velocity_steps_override`,
  `set_position_steps_override`): iterations for the island the constraint is in, at most
  `limits::MAX_SOLVER_STEPS_OVERRIDE` (255, [limits.md](limits.md#solver-step-counts)). 0, the
  default, asks for the world's `WorldSettings::velocity_steps` or `position_steps`.
- `constraint_priority` (`set_constraint_priority`): the constraint's place in the order the
  solver handles its island in. Default 0.

For each island and step, Jolt takes the largest count asked for by the island's active
constraints and by the dynamic bodies of its contacts; when any of them asks for none (0), the
world's count joins the maximum (`CalculateSolverSteps.h`, `PhysicsSystem.cpp:1513-1531`, and
`LargeIslandSplitter.cpp:327-353` for islands it splits). oxijolt gives bodies no override, so a
body asks for the world's count. Every constraint and contact of the island is then solved with
that count:

- An override above the world's count on one constraint raises the count of its whole island,
  contacts included. A welded tower standing on the ground also makes its ground contacts, and
  everything else touching it, pay for the extra iterations.
- An override below the world's count lowers the count only for an island without contacts whose
  active constraints all ask for fewer.
- A disabled constraint, or one whose bodies sleep, is not active and asks for nothing.
- In an island of fewer than 128 constraints and contacts Jolt stops iterating after an iteration
  that applies no impulse, so the count is a maximum. A larger island, which Jolt splits for
  parallel solving, always runs the full count (`LargeIslandSplitter.cpp:127-170`).

Jolt sorts the constraints of an island by priority, lowest first, and equal priorities by its
index in its constraint list (`ConstraintManager.cpp:103-115`). Within each iteration a constraint
solved later ends closer to satisfied, so the constraint that must hold best gets the highest
priority. In an island of 128 or more constraints and contacts the order holds only within each
group of constraints that share no body: Jolt solves those groups one after another in the order
it formed them, so a higher priority can run before a lower one on the same body
(`LargeIslandSplitter.cpp:236-260,343-418`). The index follows creation order until a removal, or the recreation of a pulley by a
rebase, moves the last constraint into the freed place.

In `tests/constraint_solver.rs`, a cantilever of eight 1 kg cubes welded in a row to a static
anchor, with a 20 kg cube at the end, sags 3.6 m below the anchor on average over its second
second at the default 10 velocity steps, and 0.49 m with a velocity step override of 100 on every
weld. `the_island_uses_the_largest_override` checks the rules above on the same chain.

The setters wake the constraint's bodies. A `WorldState` does not hold these settings
([state.md](state.md#what-is-not-saved)).

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let x = Vec3::new(1.0, 0.0, 0.0);
    let y = Vec3::new(0.0, 1.0, 0.0);
    let cube = Shape::new_box(Vec3::new(0.25, 0.25, 0.25))?;
    let wall = world.create_body(&cube, &BodySettings::new_static())?;
    let beam = world.create_body(
        &cube,
        &BodySettings::new_dynamic().position(RVec3::new(0.6, 0.0, 0.0)),
    )?;

    // While the weld is active its island gets a velocity step count of at least 40.
    let settings = FixedConstraintSettings::new(RVec3::new(0.3, 0.0, 0.0), x, y)
        .velocity_steps_override(40)
        .constraint_priority(1);
    let weld = world.create_constraint(wall, beam, &settings)?;
    assert_eq!(world.constraint(weld)?.velocity_steps_override(), 40);

    // Back to the world's count.
    world.constraint_mut(weld)?.set_velocity_steps_override(0)?;
    assert!(world.step(1.0 / 60.0)?.is_complete());
    Ok(())
}
```

## References between constraints

A gear can name the hinges that hold its two bodies (`GearConstraintSettings::hinges`), and a rack
and pinion the hinge and slider that hold its bodies
(`RackAndPinionConstraintSettings::constraints`). Jolt then corrects drift from their angle and
position; without them it couples the velocities only and the bodies slowly drift apart. Each
referenced constraint must hold the coupled body as its body 2, about the same axis direction. A
referenced hinge or slider cannot be removed while the coupling exists
(`ConstraintError::UsedByConstraint`); remove the coupling first.

A gear's drift correction uses hinge angles in `[-π, π]`, which is consistent across the wrap at ±π
only for an integer ratio; with another ratio Jolt corrects towards a wrong angle once gear 2 has
wrapped.

## Breakable constraints

Every constraint reads back the impulses its solver parts applied in the last collision step it
was solved in, through the `total_lambda*` methods of `ConstraintRef`: N·s for linear parts, N·m·s
for angular parts. `PhysicsWorld::step` solves one collision step per call, so dividing a readout
by the step's `delta_time` gives the mean force (N) or torque (N·m) the part applied over that
step. After `step_with_collision_steps(delta_time, n)` divide by `delta_time / n`: the readout is
the last collision step's, not a sum or a peak over the call.
Each readout is one part's share, not the net force on a body: the position part acts at the
constraint point, so a weld 0.4 m beside the centre of mass of a hanging cube reads the weight in
its position part and, in its rotation part, the torque that cancels the weight's moment about the
weld.

| Kind | Readouts | Unit | Frame | Bodies |
|---|---|---|---|---|
| fixed | `total_lambda_position`, `total_lambda_rotation` | N·s, N·m·s | world vectors | on body 2; body 1 gets the opposite |
| point | `total_lambda_position` | N·s | world vector | on body 2; body 1 the opposite |
| distance | `total_lambda_position` | N·s | along the line between the points | on body 2; body 1 the opposite |
| hinge | `total_lambda_position`; `total_lambda_rotation`; `total_lambda_motor`, `total_lambda_rotation_limits` | N·s; N·m·s; N·m·s | world vector; the two axes perpendicular to the hinge; about the hinge axis | on body 2; body 1 the opposite |
| slider | `total_lambda_position`; `total_lambda_rotation`; `total_lambda_motor`, `total_lambda_position_limits` | N·s; N·m·s; N·s | the two axes perpendicular to the slider; world vector; along the slider axis | on body 2; body 1 the opposite |
| cone | `total_lambda_position`, `total_lambda_rotation` | N·s, N·m·s | world vector; one value for the cone | on body 2; body 1 the opposite |
| swing-twist | `total_lambda_position`; `total_lambda_limits`; `total_lambda_motor` | N·s; N·m·s; N·m·s | world vector; the twist, swing Y and swing Z limit parts (below); per constraint axis (twist, swing Y, swing Z) | on body 2; body 1 the opposite |
| six-DOF | `total_lambda_position`, `total_lambda_rotation`; `total_lambda_motor_translation`, `total_lambda_motor_rotation` | N·s, N·m·s | world vectors while the three axes of that kind are fixed (for translation, also with no soft limit spring on any of them), otherwise per constraint axis for translation and the swing-twist limit parts for rotation; per constraint axis | on body 2; body 1 the opposite |
| gear | `total_lambda` | N·m·s | about each body's axis | body 1 and body 2 get the same value, not `ratio` times it |
| rack and pinion | `total_lambda` | N·m·s on the pinion | about the pinion's axis | the rack (body 2) gets `-ratio` times it, in N·s along its axis |
| pulley | `total_lambda_position` | N·s | along body 1's rope, negative while the rope pulls | body 2 gets `ratio` times it along its rope |
| path | `total_lambda_position`; `total_lambda_position_limits`, `total_lambda_motor`; `total_lambda_rotation_hinge`, `total_lambda_rotation` | N·s; N·s; N·m·s | normal and binormal; along the path; the two axes perpendicular to the free axis, or a world vector | on body 2; body 1 the opposite |

While a hinge, slider, swing-twist, six-DOF or path motor is off, its motor readout holds the
friction impulse: Jolt drives friction through the motor part. A distance constraint whose range
leaves it slack reads zero.

A limit only pushes back. Hinge, slider and path limits act along a fixed axis, so their sign
says which end holds. Swing-twist limits, and six-DOF rotation while not all three rotation axes
are fixed, act about axes Jolt picks each step:
- A twist limit, and a swing limit whose other swing axis is locked, turn their axis round at the
  minimum, so they read zero or less at either end.
- With both swing axes limited (a cone of swing) Jolt solves one swing limit, about the axis from
  the nearest allowed swing to the current one, and reads it in the swing Y slot; swing Z reads
  zero whichever way the joint swings.
- A locked axis (a zero range) reads either sign about its constraint axis.

For breaking, compare the length of the three limit values with a torque limit, not one
component.

Jolt's own docs suggest this for breakable constraints (`Constraint::SetEnabled`): after a step,
compare the impulse with a limit and disable the constraint once it is over. With oxijolt:
- After each complete `step`, divide every enabled bond's readouts by the collision step it was
  solved in (`delta_time`, or `delta_time / n` with `n` collision steps) and compare the forces and
  the torques with separate limits.
- Break with `ConstraintMut::set_enabled(false)`. The enabled flag is part of the saved state, so
  `restore_state` brings a broken bond back. `remove_constraint` changes the world's structure:
  no state saved before it restores any more.
- Skip disabled bonds. A disabled constraint, and one whose bodies sleep, keeps the readouts of the
  last collision step it was solved in: for a broken bond that is the step that broke it. With a
  varying `delta_time` or collision step count, divide a sleeping bond's readout by the length of
  the collision step it was solved in.
- Jolt's velocity solver runs a fixed number of iterations per step
  (`WorldSettings::velocity_steps`), so in a chain of bonds one step's readouts are where those
  iterations stopped, not an exact solution. Leave the limits some headroom.

The wheels of a vehicle are not constraints; their impulses are in `WheelState`
(`suspension_lambda`, `longitudinal_lambda`, `lateral_lambda`, [guide](guide.md#vehicles)).

A weld under a 5 kg cube, pulled down by a growing force, breaks once it holds more than 2 kN:

```rust
use oxijolt::*;

const DT: f32 = 1.0 / 60.0;
/// The force the weld may hold, N.
const MAX_FORCE: f32 = 2000.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let anchor = Shape::new_box(Vec3::new(0.1, 0.1, 0.1))?;
    let top = RVec3::new(0.0, 10.0, 0.0);
    let ceiling = world.create_body(&anchor, &BodySettings::new_static().position(top))?;
    let cube = Shape::new_box(Vec3::new(0.25, 0.25, 0.25))?;
    let below = RVec3::new(0.0, 9.5, 0.0);
    let settings = BodySettings::new_dynamic().position(below).mass(5.0);
    let load = world.create_body(&cube, &settings)?;
    let at = RVec3::new(0.0, 9.75, 0.0);
    let x = Vec3::new(1.0, 0.0, 0.0);
    let weld = FixedConstraintSettings::new(at, x, Vec3::new(0.0, 1.0, 0.0));
    let weld = world.create_constraint(ceiling, load, &weld)?;

    let mut broke_at = None;
    for tick in 1..=120 {
        if broke_at.is_none() {
            let pull = Vec3::new(0.0, -100.0 * tick as f32, 0.0);
            world.body_mut(load)?.add_force(pull)?;
        }
        assert!(world.step(DT)?.is_complete());
        let bond = world.constraint(weld)?;
        let p = bond.total_lambda_position();
        let force = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt() / DT;
        if bond.is_enabled() && force > MAX_FORCE {
            world.constraint_mut(weld)?.set_enabled(false);
            broke_at = Some(tick);
        }
    }
    // The weld holds 49 N of weight plus 100 N per tick of pull: it breaks at tick 20.
    assert_eq!(broke_at, Some(20));
    assert!(world.body(load)?.position().y < 0.0);
    Ok(())
}
```

## Rebase, state and ids

- Constraint frames are relative to the bodies, so `rebase` needs to change nothing, except for
  pulleys, whose fixed points are world points: a rebase recreates each pulley in the new frame
  ([guide.md](guide.md#floating-origin)).
- A `WorldState` holds each constraint's enabled flag, warm start (the impulses the
  `total_lambda*` readouts return), motor states and targets (for a path also its motor settings
  and friction), but not the rest of its configuration: limits, motor settings, friction,
  distances, pulley lengths, the cone angle, the priority and the step overrides
  ([state.md](state.md)).
- Each world numbers its constraints from 1 in creation order and never reuses an id. Dropping a
  world removes its constraints first.

## Known instabilities

Jolt's solver can diverge for reasons the lever-arm ratio does not measure: a hinge whose pin lies
off the axis of a slender body, chains of hinges with non-parallel axes, a light body held by
several constraints, and a slider whose body 1 travels far along its axis.
[coverage.md](coverage.md#not-covered) describes the measured cases; no bound in `oxijolt::limits`
excludes them.
