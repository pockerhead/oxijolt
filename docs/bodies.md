# Body controls

Rigid bodies are created from a `Shape` and `BodySettings` and changed through `BodyMut`
(`PhysicsWorld::body_mut`). This guide covers the controls beyond poses, velocities and forces:
impulses, buoyancy, kinematic moves, waking and sleeping, sensors, user data, locked axes, and
changing a body's shape or motion type. Every call checks its input first; a refused call changes nothing.

## Creating bodies

`PhysicsWorld::create_body(&shape, &settings)` takes a `Shape` and `BodySettings`. The shape can be
made in the same expression, `world.create_body(&Shape::new_sphere(0.5)?, &settings)?`, and dropped
right after, since the body holds its own reference. A `Shape` kept apart lets many bodies, worlds,
ragdoll parts and compounds share one Jolt shape: the playground's pile of 400 bodies uses nine
shapes, and its loose bricks and cannonballs one shape each. The binding has no call that creates a
body and its shape together. In the playground every body is created next to the playground's own
description of its shape for drawing, so such a call would remove no line there, and it would hide
the sharing.

`PhysicsWorld::body_ids()` lists every body of the world in ascending id order, soft bodies,
ragdoll parts and characters' inner bodies included, for example to fold the whole world into a
digest that compares runs.

## Mass and inertia

By default Jolt computes a body's mass and inertia from its shape at a density of 1000 kg/m³.
`BodySettings::mass(kg)` sets the mass and scales the shape's inertia to it.
`BodySettings::mass_and_inertia(kg, inertia)` sets both, and the shape's own mass and inertia are
not used (Jolt `EOverrideMassProperties::MassAndInertiaProvided`), for example a box hull with the
inertia of the vehicle it stands for. The centre of mass stays the shape's in either case. The last of the two
calls wins.

`Inertia::Principal(moments)` gives the moments about the body's x, y and z axes, which are then
its principal axes. `Inertia::Tensor(rows)` gives the full symmetric tensor as Jolt stores it: the
moments on the diagonal, the negated products of inertia off it. Both are in kg·m², about the
centre of mass, in the body's local axes.

`PhysicsWorld::create_body` checks the mass as for `mass` and refuses with
`BodyError::InvalidValue` an inertia that is not finite or not symmetric, has a principal moment
below `limits::MIN_PROVIDED_INERTIA` (1e-6 kg·m²) or above `6 · mass · limits::MAX_SHAPE_EXTENT²`,
or, when it is not diagonal, is not positive definite and well conditioned
([limits](limits.md#provided-inertia)).

`BodyMut::set_shape` computes the mass properties from the new shape again, with the mass it is
given. `RagdollSettings::new_stabilized` refuses parts with a provided inertia: Jolt's
stabilization rescales the inertia with the part's new mass.

`BodyRef::inverse_inertia` reads back what Jolt uses: the inverse inertia tensor about the centre
of mass in world coordinates (kg⁻¹·m⁻²), in rows like `Inertia::Tensor`. It includes the body's
current rotation, and the rows and columns of rotation axes locked with `allowed_dofs` are zero,
so the tensor times an angular impulse is the change of angular velocity
`add_angular_impulse` gives, up to Jolt's clamp at `limits::MAX_ANGULAR_VELOCITY`. It is `None` for static, kinematic and soft bodies.

`Shape::center_of_mass` is the centre of mass in the shape's own space (a compound's is its
children's mass-weighted centre), and `BodyRef::center_of_mass_position` is the body's in world
space: `position() + rotation() * shape.center_of_mass()`, for static and kinematic bodies too.
The velocity of a point `r` of a rigid body is
`linear_velocity() + angular_velocity() × (r - center_of_mass_position())`, for example for a
piece split off a compound. A soft body reads its position, which each step moves to the centre of
its vertices' bounding box unless `SoftBodySettings::update_position` is `false`.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO))?;
    let plank = Shape::new_box(Vec3::new(1.0, 0.25, 0.5))?;
    let inertia = Inertia::Principal(Vec3::new(1.0, 2.0, 4.0));
    let settings = BodySettings::new_dynamic().mass_and_inertia(3.0, inertia);
    let body = world.create_body(&plank, &settings)?;

    // An angular impulse of 2 N·m·s about z turns the body at 2 / 4 rad/s.
    world.body_mut(body)?.add_angular_impulse(Vec3::new(0.0, 0.0, 2.0))?;
    let spin = world.body(body)?.angular_velocity();
    assert!((spin.z - 0.5).abs() < 1e-6, "{spin:?}");
    Ok(())
}
```

## Impulses and forces

A force (`add_force`) acts over the next step and is cleared after it. An impulse changes the
velocity at once: `add_impulse` adds `impulse / mass`, `add_angular_impulse` the inverse inertia
times the angular impulse, and `add_impulse_at_point` both, for an impulse at a world point. Each
wakes the body. One impulse may change the velocity by at most `limits::MAX_VELOCITY_CHANGE` (twice
the speed limit, enough to reverse a body at full speed) and the angular velocity by at most
`limits::MAX_ANGULAR_VELOCITY_CHANGE`; Jolt then clamps the speed. Static and kinematic bodies
ignore impulses, as they ignore forces.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO))?;
    let ball = world.create_body(&Shape::new_sphere(0.5)?, &BodySettings::new_dynamic().mass(2.0))?;

    // A kick of 4 N·s on 2 kg: 2 m/s at once.
    world.body_mut(ball)?.add_impulse(Vec3::new(4.0, 0.0, 0.0))?;
    assert_eq!(world.body(ball)?.linear_velocity(), Vec3::new(2.0, 0.0, 0.0));
    assert!(world.step(1.0 / 60.0)?.is_complete());
    Ok(())
}
```

## Buoyancy

`apply_buoyancy_impulse` applies one step of a fluid's lift and drag to a body (Jolt
`Body::ApplyBuoyancyImpulse`). Call it once per step, before `step`, for every body that may be in
the water, with the gravity the body feels: the world's, or for radial gravity the vector at the
body. `BuoyancySettings` describes the fluid: its surface (a point and the normal pointing up out
of it), the buoyancy factor, the linear and angular drag and the fluid's velocity.

The buoyancy factor is the fluid's density over the body's, with the body's volume as Jolt computes
it: the bounding box for boxes, capsules, cylinders and tapered shapes, the shape itself for spheres
and convex hulls. A factor of 1 floats neutrally, 2 floats half under. The lift scales with the
body's gravity factor, so a body with gravity factor 0 gets none. Linear drag is quadratic, over
the area of the bounding box facing the flow, and changes the velocity by at most the body's own
speed in one call. A current therefore takes a few steps to catch a slow body, but under gravity a
floating body is never exactly at rest: a crate four times lighter than the water, starting at
rest at its draft in a 2 m/s current, drifts about 15.5 m in 10 s. Only a body exactly at rest, as
in zero gravity, stays put. The velocities change at once and are
clamped as an impulse's; the buoyant velocity change is bounded like an impulse's
([limits.md](limits.md#buoyancy)). The call returns whether the body is in the water and wakes it
when it is. Static, kinematic and dry bodies are left alone.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    // A 1 m x 0.5 m x 1 m crate, four times lighter than the water.
    let crate_shape = Shape::new_box(Vec3::new(0.5, 0.25, 0.5))?;
    let settings = BodySettings::new_dynamic()
        .position(RVec3::new(0.0, 0.5, 0.0))
        .linear_damping(1.0);
    let floating = world.create_body(&crate_shape, &settings)?;
    let water = BuoyancySettings::default()
        .surface(RVec3::ZERO, Vec3::new(0.0, 1.0, 0.0))
        .buoyancy(4.0)
        .linear_drag(2.0);
    let dt = 1.0 / 60.0;
    for _ in 0..600 {
        let gravity = world.gravity();
        world
            .body_mut(floating)?
            .apply_buoyancy_impulse(&water, gravity, dt)?;
        assert!(world.step(dt)?.is_complete());
    }
    // A quarter of its height is under water: its centre floats 0.125 m above the surface.
    let y = world.body(floating)?.position().y;
    assert!((y - 0.125).abs() < 0.01);
    Ok(())
}
```

Quadratic drag alone barely damps slow bobbing; the crate above also has linear damping.

## Kinematic platforms

`move_kinematic(position, rotation, dt)` sets the velocities that carry a kinematic body toward a
pose over the next step of `dt` seconds, so bodies resting on it ride along. The velocity stays
after the step: call it every step the platform should move, and set a zero velocity to stop it.
Jolt turns with a small-angle approximation, so the pose is approached rather than met exactly. The
implied velocities must stay within the speed limits.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let deck = Shape::new_box(Vec3::new(2.0, 0.2, 2.0))?;
    let platform = world.create_body(&deck, &BodySettings::new_kinematic())?;
    let crate_shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let cargo = world.create_body(
        &crate_shape,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 0.7, 0.0)),
    )?;

    let dt = 1.0 / 60.0;
    for tick in 1..=60 {
        let target = RVec3::new(tick as Real / 60.0, 0.0, 0.0);
        world.body_mut(platform)?.move_kinematic(target, Quat::IDENTITY, dt)?;
        assert!(world.step(dt)?.is_complete());
    }
    assert!(world.body(cargo)?.position().x > 0.5);
    Ok(())
}
```

## Waking and sleeping

`activate` wakes a body and `deactivate` puts it to sleep now, zeroing its velocities. With
`EventSettings::body_activation` both report `ActivationEvent`s for bodies that change state.
`PhysicsWorld::activate_bodies_in_box` wakes every body whose exact bounds overlap a box, in body-id
order, so the result does not depend on the broad phase's history. A restore puts bodies to sleep or
wakes them without events.

`invalidate_contact_cache` makes Jolt collide a body's pairs afresh in the next step that simulates,
instead of reusing their cached contacts, so a contact listener whose `contact_validate` answer
changed is asked again ([events.md](events.md#validating-contacts)). It also wakes the body. The
request waits for a step in which some body is awake or a vehicle exists, and a `WorldState` saved
before that holds it.

## Sensors

`BodySettings::sensor(true)` makes a trigger: Jolt reports its contacts and resolves none of them.
The contact events carry `ContactSettings::is_sensor()`; [events.md](events.md#sensors) lists which
bodies a sensor detects and how sleep, characters, vehicles and soft bodies treat it.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    world.set_event_settings(EventSettings::default().contacts(true));
    let zone = Shape::new_box(Vec3::new(1.0, 1.0, 1.0))?;
    let trigger = world.create_body(&zone, &BodySettings::new_static().sensor(true))?;
    let ball = world.create_body(
        &Shape::new_sphere(0.3)?,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 3.0, 0.0)),
    )?;

    let mut entered = false;
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
        for event in world.take_events().contacts {
            if let ContactEvent::Added { manifold, settings, .. } = event {
                let bodies = [manifold.pair.body1, manifold.pair.body2];
                entered |= settings.is_sensor() && bodies.contains(&trigger);
            }
        }
    }
    assert!(entered);
    // The sensor did not stop the ball.
    assert!(world.body(ball)?.position().y < -1.0);
    Ok(())
}
```

`BodyMut::set_is_sensor` turns the flag on or off on a live rigid body. The next step that
simulates collides the body's pairs afresh: a contact whose shapes overlap continues as
`ContactEvent::Persisted` with the new `is_sensor()`, and a speculative contact, between shapes
that are apart or just touch, ends with `ContactEvent::Removed` when the body becomes a sensor. A
sensor made solid while it overlaps a body pushes that body out over the following steps. The
toggle wakes bodies like the filter setters ([Changing body properties](#changing-body-properties))
and is a structural change: earlier `WorldState`s no longer restore. Making a body a sensor
applies the creation rules (no `MotionQuality::LinearCast`, no shape only static bodies may use);
the inner body of a character, a vehicle chassis and a ragdoll part refuse the toggle.

## User data

`BodySettings::user_data` stores a `u64` of the caller's in the body, for example the key of its
entity in an ECS, and `BodyRef::user_data` reads it back. It is fixed at creation, so no restore can
change it.

## Collision groups

A `CollisionGroup` filters which bodies the solver lets collide, beyond their object layers: for a
ragdoll that does not collide with itself, a chain whose neighbouring links overlap, or a vehicle
that ignores its driver. Build a `GroupFilterTable` of sub groups with a `GroupFilterTableBuilder`,
which says which pairs of sub groups collide; once built, a table never changes and bodies share it.
Give a body its group at creation with `BodySettings::collision_group` (or
`SoftBodySettings::collision_group`).

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let floor = Shape::new_box(Vec3::new(10.0, 0.5, 10.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)))?;

    // One sub group: bodies of the same group id never collide with each other.
    let table = GroupFilterTableBuilder::new(1)?.build();
    let crew = CollisionGroup::new(&table, 1, 0)?;
    let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let at = |y| BodySettings::new_dynamic().position(RVec3::new(0.0, y, 0.0));
    world.create_body(&cube, &at(0.5).collision_group(crew.clone()))?;
    let upper = world.create_body(&cube, &at(1.6).collision_group(crew))?;
    for _ in 0..90 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    // The upper cube fell through the lower one onto the floor.
    assert!(world.body(upper)?.position().y < 0.6);
    Ok(())
}
```

Jolt's rules for two bodies that both have a group (`GroupFilterTable::CanCollide`):
- different group ids collide;
- the same group id with different tables never collides;
- within one table the same sub group never collides, and different sub groups collide unless the
  table disables the pair.

A body without a group collides by its object layer alone. `CollisionGroup::can_collide` answers by
the same rules. Caller group ids go up to `CollisionGroup::MAX_GROUP_ID` (2^31 - 1); a table has at
most `GroupFilterTable::MAX_SUB_GROUPS` sub groups ([limits.md](limits.md#group-filter-table-size)).

Where groups apply: the discrete and continuous collision of rigid bodies (`Body.inl:74-76`,
`PhysicsSystem.cpp:1992`), soft bodies against rigid bodies (`SoftBodyMotionProperties.cpp:148`), and
sensors, which do not detect a body their group excludes. Scene queries, character movement and
vehicle wheel casts do not look at groups (a wheel's own filter skips only its chassis): a grouped
body is still found by a ray cast and still blocks a character. A character's inner body has no
group at creation.

Ragdolls give their parts the group id `2^31 + RagdollId::to_raw()`, above every caller group id,
with a table in which no two parts of one ragdoll collide; a ragdoll part cannot take a caller
group. Two ragdolls of one settings object share the table and still collide with each other,
because their group ids differ.

`BodyMut::set_collision_group` changes the group of a live body, or removes it with `None`, and
`BodyRef::collision_group_is` compares it: the same table, group id and sub group, so a clone of a
group is the same group and a group with equal ids in another table is not. The body keeps its own
reference to the table, so the caller may drop its `CollisionGroup` and `GroupFilterTable`. The
change wakes bodies like the other filter setters
([Changing body properties](#changing-body-properties)). Jolt saves no group in its state
(`Body::SaveState`, `Body.cpp:296-324`), so a restore keeps the group set last; a rollback sets
the old group back before the restore ([state.md](state.md#what-is-not-saved)).

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let floor = Shape::new_box(Vec3::new(10.0, 0.5, 10.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)))?;

    let table = GroupFilterTableBuilder::new(1)?.build();
    let crew = CollisionGroup::new(&table, 1, 0)?;
    let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let at = |y| BodySettings::new_dynamic().position(RVec3::new(0.0, y, 0.0));
    world.create_body(&cube, &at(0.5).collision_group(crew.clone()))?;
    let upper = world.create_body(&cube, &at(1.6).collision_group(crew.clone()))?;
    assert!(world.body(upper)?.collision_group_is(Some(&crew)));

    // Without a group the upper cube collides with the lower one and lands on it.
    world.body_mut(upper)?.set_collision_group(None, Activation::Activate)?;
    assert!(world.body(upper)?.collision_group_is(None));
    for _ in 0..90 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    assert!(world.body(upper)?.position().y > 1.4);
    Ok(())
}
```

## Locked axes

`BodySettings::allowed_dofs` limits a body to some of its six degrees of freedom, along and about
the world axes: `AllowedDofs::PLANE_2D` keeps a body in the XY plane, turning about Z. A value must
keep a translation axis. Constraints, vehicles, ragdolls and a rotating rebase refuse such bodies,
because their checks assume an unmasked body.

## Shape and motion type changes

`BodyMut::set_shape` gives a body a new shape and the mass properties a body created with it would
have; it wakes the bodies within the speculative contact distance of the box that encloses the old
and new bounds, which includes bodies between the two shapes when they lie apart.
`BodyMut::set_motion_type` switches a body between static, kinematic and dynamic. A body created static changes only when created with
`BodySettings::allow_dynamic_or_kinematic(true)`. Both refuse bodies that a character, vehicle,
ragdoll or constraint holds, except that `set_shape` accepts a static body that constraints use.

The body origin stays where it is and the centre of mass moves with the shape. Jolt stores the
centre of mass position and derives the origin from it, so the origin read back can move by
rounding: a static body at about 100 m moved by 7.6e-6 m (one `f32` ulp) when its centre of mass
moved 1 m. The constraints of a static body keep their world attachment points through its shape
change up to the same kind of rounding: 3.8e-6 m for that change and at most 7.6e-6 m after two
more changes that end with the first shape again, for each of the nine kinds a static body can
join, on either side (`src/constraint/shape_change_tests.rs`, single precision). Without the
notification the points would move with the centre of mass. A gear reads only the angles of the
hinges it couples, so a gear over hinges on the static body keeps coupling
(`static_shape_constraints.rs`).

Jolt saves neither shapes nor motion types, so each successful change makes earlier `WorldState`s
unrestorable ([state.md](state.md)): a rollback cannot cross it.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let floor = Shape::new_box(Vec3::new(10.0, 0.5, 10.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)))?;
    let block = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let hanging = world.create_body(
        &block,
        &BodySettings::new_static()
            .position(RVec3::new(0.0, 3.0, 0.0))
            .object_layer(ObjectLayer::MOVING)
            .allow_dynamic_or_kinematic(true),
    )?;

    let saved = world.save_state();
    world
        .body_mut(hanging)?
        .set_motion_type(MotionType::Dynamic, Activation::Activate)?;
    assert_eq!(world.restore_state(&saved), Err(StateError::WorldChanged));
    for _ in 0..90 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    assert!(world.body(hanging)?.position().y < 1.0);
    Ok(())
}
```

## Changing body properties

Most values of `BodySettings` can change on a live body through Jolt's own setters. Each setter
checks its value with the rule of the matching creation setting and returns `Result<(), BodyError>`;
a refused call changes nothing.

| Property | Read (`BodyRef`) | Change (`BodyMut`) | Bodies | Refused with |
|---|---|---|---|---|
| Friction | `friction()` | `set_friction(f32)` | every body, soft and static too | `InvalidValue` outside `0..=MAX_FRICTION` |
| Restitution | `restitution()` | `set_restitution(f32)` | every body | `InvalidValue` outside `0..=1` |
| Gravity factor | `gravity_factor() -> Option<f32>` | `set_gravity_factor(f32)` | bodies that can move, soft too | `CannotMove`, `InvalidValue` beyond `MAX_GRAVITY_FACTOR` |
| Linear damping | `linear_damping() -> Option<f32>` | `set_linear_damping(f32)` | bodies that can move, soft too | `CannotMove`, `InvalidValue` (finite, at least 0) |
| Angular damping | `angular_damping() -> Option<f32>` | `set_angular_damping(f32)` | rigid bodies that can move | `NotRigidBody`, `CannotMove`, `InvalidValue` |
| Motion quality | `motion_quality() -> Option<MotionQuality>` | `set_motion_quality(MotionQuality)` | rigid bodies that can move | `NotRigidBody`, `CannotMove`, `InvalidValue` for `LinearCast` on a sensor |
| Object layer | `object_layer()` | `set_object_layer(ObjectLayer, Activation)` | every body but a character's inner body | `OwnedByCharacter`, `UnknownObjectLayer` |
| Collision group | `collision_group_is(Option<&CollisionGroup>)` | `set_collision_group(Option<&CollisionGroup>, Activation)` | every body but a ragdoll part | `OwnedByRagdoll` |
| Sensor flag | `is_sensor()` | `set_is_sensor(bool, Activation)` | rigid bodies that no character, vehicle or ragdoll owns | `OwnedByCharacter`, `UsedByVehicle`, `OwnedByRagdoll`, `NotRigidBody`, `InvalidValue` ([Sensors](#sensors)) |

"Bodies that can move" are kinematic, dynamic and soft bodies, and static bodies created with
`BodySettings::allow_dynamic_or_kinematic(true)` (`BodyRef::can_be_kinematic_or_dynamic`); for
other bodies the getters return `None`. Soft bodies return `None` for angular damping and motion
quality, which Jolt's soft body solver does not use. A static body that may move reports the
motion quality it will move with.

Waking:
- Friction, restitution, gravity factor, damping and motion quality wake nothing, as in Jolt. The
  new value acts from the next step on; a sleeping body uses it once something wakes it
  (`BodyMut::activate`).
- The object layer, the collision group and the sensor flag change which pairs collide. Jolt
  collides a pair only when one of its bodies is awake, so these setters wake the changed body
  with `Activation::Activate` and, with either activation, every other non-static body whose
  bounds come within the speculative contact distance of its bounds, the range in which Jolt
  pairs bodies, in body-id order. A body resting on a floor
  that stops colliding with it falls. The same layer, group or flag changes nothing and wakes
  nothing.
- Queries see a new layer at once. A pair that the new layer or group excludes ends with
  `ContactEvent::Removed` in the next step in which one of its bodies is awake. Scene queries
  ignore collision groups.

Rollback ([state.md](state.md#what-is-not-saved)): Jolt saves none of these properties.
- The value setters (friction, restitution, gravity factor, damping, motion quality) change
  configuration that a restore keeps. To roll back across a change, set the old value back and
  replay.
- The layer and group are configuration too, but their set-back wakes bodies: set the old value
  back before `restore_state`, which then puts back the sleep states, and drop the events queued
  before the restore.
- The sensor toggle is a structural change: every earlier `WorldState` is refused with
  `StateError::WorldChanged`, so save again after the toggle.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let floor = Shape::new_box(Vec3::new(100.0, 1.0, 100.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    let block = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let sliding = world.create_body(
        &block,
        &BodySettings::new_dynamic()
            .position(RVec3::new(0.0, 0.5, 0.0))
            .linear_velocity(Vec3::new(5.0, 0.0, 0.0))
            .friction(0.0)
            .linear_damping(0.0)
            .angular_damping(0.0),
    )?;
    for _ in 0..10 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    assert!(world.body(sliding)?.linear_velocity().x > 4.9);

    // Jolt combines it with the floor's friction of 0.2 as sqrt(1.0 * 0.2).
    world.body_mut(sliding)?.set_friction(1.0)?;
    assert_eq!(world.body(sliding)?.friction(), 1.0);
    for _ in 0..120 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    assert!(world.body(sliding)?.linear_velocity().x.abs() < 0.05);
    Ok(())
}
```

## Compound edits

Destruction and building change compounds at run time. Shapes never change, so the edits go to a
`MutableCompound`, an editor that adds, removes, moves and replaces children and publishes the
current children as a new `Shape` with `to_shape`. A publication equals `Shape::new_compound` of
the same children, including the centre of mass Jolt moves to the children's mass-weighted centre
and the extent rule that applies after that move.

A publication reaches a body only through `BodyMut::set_shape`, which applies every rule of a shape
change above: the body rules, fresh mass properties and the wake of the bodies near the box around
the old and new bounds. On a large compound that box can hold many sleeping bodies. Each `set_shape`
changes one body; when a publication goes to several bodies and a later one refuses it, the earlier
ones keep it. Bodies, worlds and scaled shapes that hold an earlier publication keep it unchanged.
A body that a constraint, character, vehicle or ragdoll holds refuses the change; remove the
constraint, install the publication and create the constraint again.

Children are numbered in order. Removing one moves the later ones down by one, and a child count
that crosses a power of two changes the width of every child index, so a sub-shape id from before a
publication (a contact removal, a character's ground, a stored hit) may name another child, or
none, afterwards; the decoders (`compound_sub_shape` and the readers built on it) check the range,
so such an id gives another child or `None`.

The editor is the caller's state: `WorldState` does not hold it, and a state saved before an
installed publication no longer restores. A rollback that should replay an edit saves after the
`set_shape` and before the next step ([state.md](state.md)).

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let floor = Shape::new_box(Vec3::new(10.0, 0.5, 10.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)))?;

    // A ledge of two tiles at y = 2, a crate resting on the second.
    let tile = Shape::new_box(Vec3::new(1.0, 0.25, 1.0))?;
    let child = |x: f32, user_data| CompoundChild {
        shape: &tile,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data,
    };
    let mut ledge = MutableCompound::from_children(&[child(0.0, 0), child(2.5, 1)])?;
    let body = world.create_body(
        &ledge.to_shape()?,
        &BodySettings::new_static().position(RVec3::new(0.0, 2.0, 0.0)),
    )?;
    let crate_shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let falling = world.create_body(
        &crate_shape,
        &BodySettings::new_dynamic().position(RVec3::new(2.5, 2.75, 0.0)),
    )?;

    // The second tile breaks away.
    ledge.remove_shape(1)?;
    world.body_mut(body)?.set_shape(&ledge.to_shape()?, None, Activation::DontActivate)?;
    for _ in 0..90 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    assert!(world.body(falling)?.position().y < 1.0);
    Ok(())
}
```

## Fixed at creation

User data, locked axes and movement capability have no setters, so a rollback never has to restore
them. Each of these programs fails to compile only because the setter does not exist:

```rust,compile_fail,E0599
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let id = world.create_body(&Shape::new_sphere(0.5)?, &BodySettings::new_dynamic())?;
    world.body_mut(id)?.set_user_data(7);
    Ok(())
}
```

```rust,compile_fail,E0599
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let id = world.create_body(&Shape::new_sphere(0.5)?, &BodySettings::new_dynamic())?;
    world.body_mut(id)?.set_allowed_dofs(AllowedDofs::PLANE_2D);
    Ok(())
}
```
