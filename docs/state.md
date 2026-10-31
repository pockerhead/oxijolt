# Saving and restoring a world

`PhysicsWorld::save_state` takes a snapshot of a world's simulation state as a `WorldState`, and
`PhysicsWorld::restore_state` returns the world to it. That is what rollback netcode and replays
need: save at tick N, run on, restore, and the same calls give the same results bit for bit.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let floor = Shape::new_box(Vec3::new(10.0, 1.0, 10.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    let ball_shape = Shape::new_sphere(0.5)?;
    let ball = world.create_body(
        &ball_shape,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
    )?;

    let saved = world.save_state();
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    let first_run = world.body(ball)?.position();

    world.restore_state(&saved)?;
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    assert_eq!(world.body(ball)?.position(), first_run);
    Ok(())
}
```

## What a state holds

Jolt's saved state of the physics system (`PhysicsSystem::SaveState`):
- the previous step's delta time and the world's gravity;
- per body its pose, velocities, accumulated force and torque, sleep test data, whether it may sleep
  and whether it is awake; for a soft body its vertex positions and velocities and its bounds;
- the contact cache;
- every constraint's own state: its enabled flag, the solver parts' warm start (what the
  constraint impulse readouts return), motor states and targets, and for path constraints also
  the motor settings and maximum friction;
- for vehicles the driver input and the engine, transmission and wheel state, a tracked vehicle's
  track speeds and a motorcycle's target lean.

On top of that it holds every character's `CharacterState` and the contact-cache invalidations
(`BodyMut::invalidate_contact_cache`, and the one `BodyMut::set_shape` makes) that no step has
applied yet; `restore_state` replaces the pending ones with these. Jolt keeps its own invalidation
flag in no saved state and cannot clear it, so for `invalidate_contact_cache` the world sets it only
right before a step in which some body is awake or a vehicle exists, which clears it at its end.
`set_shape` is the exception: Jolt's `SetShape` sets the flag at once when the shape changes. That
call also changes the world's structure, so no state saved before it restores, and the pending
request sets the flag again before the next simulating step after a restore of a state saved after
it.

`save_state_of` saves only some bodies ([Choosing the bodies](#choosing-the-bodies)).

A game's own state, such as the bonds it counts as broken or its scripts, is not in a
`WorldState`; the game saves it next to the state.

## Reusing a buffer

`save_state` returns a new `WorldState` each time. For rollback, where a game saves every tick,
allocate the states once and save into them with `save_state_into`, which overwrites a state in
place and keeps every buffer it grows. A save into it allocates on the Rust heap only for what
no earlier save into that buffer needed (a longer Jolt stream, more bodies or characters, a larger
character state, more pending contact-cache invalidations); a full save also sizes the buffer for
later `Movable` and `Only` saves. The tests count no allocation over 1000 saves of a scene with
stacks, a car and a character in every selection, after a buffer saw only full saves, and over
worlds of two, one and two characters. Jolt's recorder still allocates its own stream on the C++
heap for each save. Restores make no such promise: `restore_state` and `restore_state_of`
allocate a few small buffers on each call.

`WorldState::new()` (or `Default`) is an empty state of no world: restoring it fails with
`StateError::WrongWorld`, so a ring can be allocated before the world exists. A buffer belongs to
the world that saved into it last.

```rust
use oxijolt::*;

/// Ticks a late input can reach back.
const WINDOW: usize = 8;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let ball_shape = Shape::new_sphere(0.5)?;
    let ball = world.create_body(
        &ball_shape,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 10.0, 0.0)),
    )?;

    let mut ring = vec![WorldState::new(); WINDOW];
    for tick in 0..60 {
        world.save_state_into(BodySelection::All, &mut ring[tick % WINDOW])?;
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    let at_tick_60 = world.body(ball)?.position();

    // An input for tick 55 arrives late: go back to the start of tick 55 and run it again.
    world.restore_state(&ring[55 % WINDOW])?;
    for tick in 55..60 {
        world.save_state_into(BodySelection::All, &mut ring[tick % WINDOW])?;
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    assert_eq!(world.body(ball)?.position(), at_tick_60);
    Ok(())
}
```

## Choosing the bodies

`save_state_of` and `save_state_into` take a `BodySelection`:
- `All` saves every body.
- `Movable` saves every body that is not static, asleep or awake: dynamic, kinematic and soft
  bodies, ragdoll parts and characters' inner bodies. Leaving the static bodies out saves 33 bytes
  each (45 in double precision, see [Size](#size)). A static body the caller moves after the save
  keeps its new pose when the state is restored.
- `Only(&ids)` saves the listed bodies, in any order, duplicates allowed. An id of another world or
  a removed body fails with `StateError::Body` before anything is saved.

Global state, contacts, constraints, characters and the pending invalidations are saved whole
whatever the selection. Restoring a state with `restore_state` leaves every body the state does
not hold as it is (except that a character's inner body moves to the restored character's pose),
so a replay from it is exact only when those bodies did not change since the save, for example
static bodies the caller never moved.

## Restoring some bodies

`restore_state_of(&state, selection)` restores global state, contacts, constraints, characters
and the pending invalidations from the state whole, and of the bodies only the selected ones;
every other body keeps its current state bit for bit, awake or asleep. A character's inner body
always counts as selected, because restoring the character moves it. A selected body the state
does not hold keeps its current state too. `BodySelection::All` is `restore_state`.

The same calls give the same result with 1 and 4 workers (the tests compare 120 ticks after such
a restore in two processes), but the world is not one that never left the state: the restored
contacts were made at the saved poses, also between a restored body and one that stays where it
is now.

It fails like `restore_state`, and with `StateError::Body` for an id of another world or a
removed body; the world is unchanged then.

## Which world it restores into

A state restores only into the world that saved it (`StateError::WrongWorld`), and only while that
world's structure is unchanged (`StateError::WorldChanged`). The structure changes when a body,
character, vehicle, ragdoll or constraint is created or removed, when a body's or a ragdoll's
motion type is changed (`BodyMut::set_motion_type`, `RagdollMut::set_motion_type`), when a body's
shape is set (`BodyMut::set_shape`, also with the body's current shape), and when a rebase moves
anything. Calls that fail their checks change nothing and keep states
restorable. In both refusals the world is unchanged.

Jolt saves neither which objects exist nor its body id allocator, so rolling back across a creation
or removal is not supported: even a body created and removed again leaves the next id different.
Within one structure, the same calls after a restore create bodies with the same ids as in the
original run.

`StateError::RestoreFailed` means Jolt or a character could not read the state. A state the world
accepted is not expected to cause it; if it happens, the world may be partly restored and should be
discarded.

## What is not saved

Configuration Jolt does not save stays as it is when a state is restored. A caller that changes it
during a run and rolls back must set it back and replay those calls, as Jolt's rollback
documentation asks for body friction. These setters change such configuration:
- hinge: `set_motor_settings`, `set_limits`, `set_limits_spring`, `set_max_friction_torque`;
- slider: `set_motor_settings`, `set_limits`, `remove_limits`, `set_limits_spring`, `set_max_friction_force`;
- distance: `set_distance`, `set_limits_spring`;
- pulley: `set_length`;
- cone: `set_half_cone_angle`;
- swing-twist: `set_swing_motor_settings`, `set_twist_motor_settings`, `set_max_friction_torque`;
- six-DOF: `set_motor_settings`;
- vehicle: `set_gravity`, `set_max_pitch_roll_angle`, `set_collision_tester`, `set_engine`,
  `set_transmission`, `set_differentials`, `set_tracks`, `set_tire_max_impulse_callback` and
  `clear_tire_max_impulse_callback` (a restore keeps the callback and does not rewind what it
  captured; the engine's rpm, the gear, the clutch and the track speeds are state and are restored);
- soft body: `set_vertex_inverse_mass` (a restore keeps the inverse masses set last) and
  `set_skinned_max_distance_multiplier`.

Body properties set at creation (shape, mass, friction, layers) are not saved either; the body
setters change only poses and velocities, which are saved, and the shape and motion type changes
count as structural changes. The sensor flag, user data, allowed degrees of freedom and movement
capability and collision group of `BodySettings` are fixed at creation, so no restore can find them
changed.
A `MutableCompound` is the caller's state, not the world's: its edits change no body until a
publication is installed with `set_shape`, which counts as a structural change
([bodies.md](bodies.md#compound-edits)).
Impulses, kinematic moves, activation and deactivation change only velocities and sleep state,
which are saved. A target changed after a save, such as a
hinge's target angle, is part of the saved state and is undone.

A setting the caller applies again before every step, such as a vehicle's gravity on a planet,
replays exactly as long as that gravity is not zero.

A skinned soft body saves its skinned positions and normals, but Jolt does not save whether a
skinning call is waiting for the next step (`SoftBodyMotionProperties::SaveState`). A restore and
replay is exact when the caller skins before every step and saves between a step and the next
skinning call, which `tests/soft_body_skinning.rs` checks after a detour
([soft-bodies.md](soft-bodies.md#skinning)); other save points are not checked.

A vehicle's world up is not saved, and Jolt has no setter for it, so a restore cannot put it back.
Each step sets the world up to the opposite of the gravity the vehicle uses (world gravity or
`VehicleMut::set_gravity`). In zero gravity a step keeps the world up it had. So after a restore
in zero gravity, the vehicle keeps the world up the abandoned run ended with. If that run had
gravity in another direction, a vehicle with a pitch and roll limit
(`WheeledVehicleSettings::max_pitch_roll_angle` below π) replays differently, and so does a motorcycle,
whose lean controller uses the world up even with the limit off.

A motorcycle's integrated lean angle is not saved either (Jolt's `MotorcycleController::SaveState`
writes only the target lean). It acts only through the lean spring's integration coefficient,
which `MotorcycleSettings` therefore requires to be 0. A motorcycle's lean controller and lean
steering limit switches are not saved; they are settings fixed at creation.

The wheel contacts a vehicle reports are empty right after a restore until the next step, because
Jolt clears a wheel's contact body on restore.

Listeners are configuration, not state: a `ContactListener` or `CharacterContactListener` set after
a save stays set after a restore. A replay that changed them must set the original ones again.

Events are not part of the state: `restore_state` neither clears nor rewinds the queue that
`take_events` drains, and a restore reports no activation changes. Take the events before a rollback
and drop those of the abandoned steps.

## Size

`WorldState::data_size` is the length of Jolt's stream, which makes up most of a state. Jolt
writes only what changes at run time, at the size each value has in memory, except that a
`Vec3` takes three floats and a position three `Real`s. The table gives what each object adds,
from Jolt's `SaveState` functions; `tests/state_sizes.rs` checks every row in both precisions.

| Object | Bytes, single precision | Bytes, double precision |
|---|---|---|
| Every state: the saved parts, delta time, gravity and the body, contact and constraint counts | 33 | 33 |
| Static body: id, awake flag, position, rotation | 33 | 45 |
| Dynamic or kinematic body, asleep or awake: also velocities, force, torque and sleep test data | 134 | 170 |
| Contact pair with one manifold, plus per contact point | 78 + 28 | 78 + 28 |
| Hinge | 45 | 45 |

A cube resting on a floor after one step, for example, is 33 + 33 + 134 + 78 + 4 x 28 = 390 bytes
in single precision. A soft body also saves its vertices, which the table does not cover, and
the other constraint kinds and vehicles save their own solver state.

Characters are outside `data_size`: each keeps a `CharacterState` of 97 bytes plus 73 per contact
with collision (121 and 85 in double precision), plus 12 bytes for its up. A character's inner
body is a kinematic body in the stream.

## The bytes

A `WorldState` gives neither its bytes nor `==`. Jolt writes fields it has never initialised into
its stream (a wheel's contact position, normal and lateral direction before the wheel's first
contact, `VehicleConstraint::SaveState`), so the stream may hold bytes without a defined value. A
restore copies them back as they were and Jolt uses them only for a wheel with a contact, so a
restore is not affected, but Rust may not read them as `u8`; the state keeps them as
`MaybeUninit<u8>`, which only Jolt copies. Compare what a world reports (poses, velocities,
character and vehicle state) instead. Loading a state into another world, or from disk, is not
supported.

## What the tests check

`tests/state.rs` saves a scene with stacks, a motor-driven hinge, a car, a walking character with an
inner body and a ragdoll, runs on, restores and replays every tick bit for bit, with 1 and 4
workers, in one process and across two. Further tests cover a detour with different inputs before
the restore, saved and unsaved configuration, structural changes, selected bodies and soft bodies.
`tests/state_buffers.rs` rolls a ring of eight reused buffers back over a mispredicted detour
(other inputs, a woken sleeper, a teleport, other gravity) and replays bit for bit with 1 and 4
workers, checks movable states and every refusal, and restores selected bodies, comparing 120
ticks after a filtered restore with 1 and 4 workers in two processes.
`tests/state_filtered_parts.rs` restores a cloth and a ragdoll's parts while an unselected cloth,
a body with pending forces and a body woken since the save step on as if nothing was restored.
`tests/state_buffer_allocations.rs` counts Rust allocations of `save_state_into`,
`tests/state_sizes.rs` the bytes of each object, and `tests/state_buffer_leaks.rs` (Windows)
gates the process's private bytes over saves and filtered restores.
`tests/body_controls_state.rs` replays every momentary body control (impulses, kinematic moves,
activation, deactivation, box activation) after a detour with 1 and 4 workers, and checks that the
creation-only body configuration survives restores and that shape and motion type changes refuse
earlier states.
