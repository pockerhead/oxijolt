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
`add_angular_impulse` gives. It is `None` for static, kinematic and soft bodies.

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
group.

The group is fixed at creation: Jolt saves no group in its state (`Body::SaveState`,
`Body.cpp:296-324`), so a change could not be rolled back. Ragdolls give their parts the group id
`2^31 + RagdollId::to_raw()`, above every caller group id, with a table in which no two parts of one
ragdoll collide; a ragdoll part cannot take a caller group. Two ragdolls of one settings object
share the table and still collide with each other, because their group ids differ.

## Locked axes

`BodySettings::allowed_dofs` limits a body to some of its six degrees of freedom, along and about
the world axes: `AllowedDofs::PLANE_2D` keeps a body in the XY plane, turning about Z. A value must
keep a translation axis. Constraints, vehicles, ragdolls and a rotating rebase refuse such bodies,
because their checks assume an unmasked body.

## Shape and motion type changes

`BodyMut::set_shape` gives a body a new shape and the mass properties a body created with it would
have; it wakes the bodies inside the box that encloses the old and new bounds, which includes
bodies between the two shapes when they lie apart. `BodyMut::set_motion_type` switches a body
between static, kinematic and dynamic. A body created static changes only when created with
`BodySettings::allow_dynamic_or_kinematic(true)`. Both refuse bodies that a character, vehicle,
ragdoll or constraint holds.

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

## Compound edits

Destruction and building change compounds at run time. Shapes never change, so the edits go to a
`MutableCompound`, an editor that adds, removes, moves and replaces children and publishes the
current children as a new `Shape` with `to_shape`. A publication equals `Shape::new_compound` of
the same children, including the centre of mass Jolt moves to the children's mass-weighted centre
and the extent rule that applies after that move.

A publication reaches a body only through `BodyMut::set_shape`, which applies every rule of a shape
change above: the body rules, fresh mass properties and the wake of the bodies inside the box around
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

The sensor flag, user data, locked axes, movement capability and collision group have no setters, so
a rollback never has to restore them. Each of these programs fails to compile only because the setter does not
exist:

```rust,compile_fail,E0599
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let id = world.create_body(&Shape::new_sphere(0.5)?, &BodySettings::new_dynamic())?;
    world.body_mut(id)?.set_is_sensor(true);
    Ok(())
}
```

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
