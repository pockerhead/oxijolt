# Events

A world can report what happened in a step: contacts that began, lasted or ended, bodies that woke
up or fell asleep, and soft body vertices that touched something. A `ContactListener` can also
change how Jolt resolves a contact, for example to make ice slippery, or reject a contact before it
forms (a one-way platform), and a `CharacterContactListener` does the same for characters. The first
sections show how to use them; the later ones describe which Jolt code paths produce each event. Line numbers refer to
the vendored Jolt 5.6 sources (`crates/oxijolt-sys/vendor/JoltPhysics/Jolt/Physics/`) and joltc
(`crates/oxijolt-sys/vendor/joltc/src/joltc.cpp`).

## Recording events

A world records nothing and installs no listener in Jolt until `set_event_settings` asks for events
(`EventSettings`: contacts, persisted contacts, body activation, soft body contacts and
validations). Jolt reports them from its worker threads during a step; the world sorts each step's
events into an order that does not depend on the thread count and queues them until `take_events`,
which a caller that records events calls after every step.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    world.set_event_settings(EventSettings::default().contacts(true));
    let floor = Shape::new_box(Vec3::new(10.0, 1.0, 10.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    let cube_shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let cube = world.create_body(
        &cube_shape,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 1.0, 0.0)),
    )?;

    let mut added = Vec::new();
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
        for event in world.take_events().contacts {
            if let ContactEvent::Added { manifold, .. } = event {
                added.push(manifold.pair.body2);
            }
        }
    }
    assert_eq!(added, [cube]);
    Ok(())
}
```

- `ContactEvent::Added` and `Persisted` carry the contact manifold (the bodies and sub-shapes that
  touch, the normal, the penetration depth, the points, and the user data of each side's
  `PhysicsMaterial`) and the settings Jolt resolves the contact with. `Removed` carries only the
  pair.
- `ActivationEvent` reports a body that woke up, fell asleep or was removed while awake, also
  between steps when a body is created, woken or removed.
- `SoftBodyValidation` reports a soft body whose bounding box overlaps a rigid body;
  `SoftBodyContacts` lists the vertices that touched a body in the step and the sensors the soft
  body overlaps. Jolt clears a soft body's vertex contacts at the end of each step, so these events
  are the only way to see them. [Sensors](#sensors) describes sensor bodies.

## Materials

A `PhysicsMaterial` carries a `u64` of the caller's, for example a surface id for footstep sounds.
Shapes made with `Shape::new_box_with_material` and its siblings, a mesh with
`MeshSettings::materials` (one material per triangle) or a heightfield with
`HeightFieldSettings::materials` (one material per cell) carry it, and contacts report it for each
side (`ContactManifold::materials`). Friction and restitution stay on the bodies. There is no call
that looks up the material of an arbitrary sub-shape id: Jolt decodes such ids without range checks,
so materials are read only for the ids a contact reports.

## Sensors

A body created with `BodySettings::sensor(true)` detects other bodies without a collision response:
Jolt reports its contacts as ordinary `ContactEvent`s whose `ContactSettings::is_sensor()` is true,
and a `ContactListener` cannot turn such a contact into an ordinary one
(`ContactSettingsError::SensorBody`). `BodyMut::set_is_sensor` turns the flag on or off on a live
body (`BodyRef::is_sensor` reads it): the next simulating step collides the body's pairs afresh, a
contact whose shapes overlap continues as `Persisted` with the new `is_sensor()`, and a speculative
contact (shapes apart or just touching) ends with `Removed` when the body becomes a sensor, because
Jolt finds sensor contacts only between overlapping shapes (`PhysicsSystem.cpp:1101`). A sensor
made solid while it overlaps a body pushes it out. A toggle counts as a structural change for
`WorldState`.

Which pairs Jolt tests (`Body::sFindCollidingPairsCanCollide`, `Body.inl:30-44`, and
`Docs/Architecture.md`, "Sensors"):
- a dynamic body pairs with any sensor;
- a kinematic body pairs with static and kinematic sensors; a kinematic sensor does not pair with a
  static body that is not a sensor;
- Jolt tests a pair only when one of its bodies is awake (`PhysicsSystem.cpp:1052-1053`). A static
  sensor therefore detects awake bodies only and loses the contact (a `Removed` event) when the body
  falls asleep. An awake kinematic or dynamic sensor also detects sleeping bodies; a sensor never
  falls asleep on its own (`Body::UpdateSleepStateInternal`, `Body.cpp:146-148`). One deactivated
  by hand leaves the active set: it stops testing pairs itself, but an awake body still pairs with
  it (`Body.inl:50-70`), so it detects awake bodies like a static sensor and stays asleep, because
  a sensor contact creates no contact constraint that would wake it
  (`ContactConstraintManager.cpp:1159-1197`). Deactivation does not turn a sensor off.

Other parts of Jolt treat sensors their own way:
- Continuous collision detection: a sensor cannot use `MotionQuality::LinearCast`
  (`create_body`, `set_is_sensor` and `set_motion_quality` refuse it), and the casts of other
  bodies skip sensors (`PhysicsSystem.cpp:1657`, `:1996`), so a fast body may pass a thin sensor
  between two steps.
- Characters: a `CharacterVirtual` is never blocked by a sensor but lists it among its contacts
  (`CharacterContact::is_sensor`, `CharacterVirtual.cpp:367`, `:761`). A character's inner body is an
  ordinary kinematic body and triggers sensors like one.
- Vehicles: the wheel collision testers ignore sensors (`VehicleCollisionTester.cpp:56`, `:170`,
  `:290`).
- Soft bodies: a soft body that overlaps a sensor lists it in `SoftBodyContacts::sensors` instead of
  colliding with it (`SoftBodyMotionProperties.cpp:152-192`).

A sensor may not use a shape that only static bodies may use (a mesh, a heightfield, or a compound
or decorated shape that contains one): a kinematic body, which may carry a mesh, pairs with a
sensor, and Jolt cannot collide a mesh with a mesh or a heightfield.

## Changing contacts: `ContactListener`

`set_contact_listener` installs a `ContactListener`, whose methods Jolt calls on its worker threads
for each new or lasting rigid contact and for each soft body overlap. A method may change the
contact's `ContactSettings`: combined friction and restitution, inverse mass and inertia scales, the
sensor flag and a surface velocity (a conveyor belt). The setters refuse values out of range
([limits.md](limits.md#contact-settings)).

```rust
use std::sync::Arc;
use oxijolt::*;

/// The material user data of ice.
const ICE: u64 = 1;

/// Frictionless contacts with ice.
struct IceIsSlippery;

impl ContactListener for IceIsSlippery {
    fn contact_added(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        self.contact_persisted(manifold, settings);
    }

    fn contact_persisted(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        if manifold.materials.contains(&Some(ICE)) {
            settings.set_combined_friction(0.0).expect("0 is a valid friction");
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    world.set_contact_listener(Some(Arc::new(IceIsSlippery)));
    let floor = Shape::new_box(Vec3::new(10.0, 0.5, 10.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)))?;

    let ice = PhysicsMaterial::new(ICE)?;
    let block = Shape::new_box_with_material(Vec3::new(0.25, 0.25, 0.25), 0.05, &ice)?;
    let sliding = world.create_body(
        &block,
        &BodySettings::new_dynamic()
            .position(RVec3::new(0.0, 0.25, 0.0))
            .linear_velocity(Vec3::new(2.0, 0.0, 0.0)),
    )?;
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    // Without friction the block keeps sliding.
    assert!(world.body(sliding)?.linear_velocity().x > 1.5);
    Ok(())
}
```

The methods run concurrently and in no fixed order while Jolt holds every body, so they must not
touch the world. For a deterministic simulation a decision must depend only on the method's
arguments and on data fixed for the step. Settings a method leaves invalid for its contact are not
applied and are reported ([below](#rejected-contact-settings)); a panic is resumed by `step`
([below](#panics)).

## Validating contacts

`ContactListener::contact_validate` decides whether Jolt keeps a hit between two rigid bodies before
it becomes a contact (Jolt `ContactListener::OnContactValidate`). It gets a `ContactCandidate` (both
bodies, their user data, the sub-shapes, the deepest points in world space, the penetration axis and
depth) and answers a `ValidateResult`. The default accepts everything, as Jolt does without a
listener.

```rust
use std::sync::Arc;
use oxijolt::*;

/// The user data of a platform that bodies pass from below.
const PLATFORM: u64 = 1;

struct OneWayPlatform;

impl ContactListener for OneWayPlatform {
    fn contact_validate(&self, contact: &ContactCandidate) -> ValidateResult {
        // Body 2 moves out of body 1 along the penetration axis.
        let pushed_up = match contact.user_data {
            [PLATFORM, _] => contact.penetration_axis.y > 0.0,
            [_, PLATFORM] => contact.penetration_axis.y < 0.0,
            _ => return ValidateResult::AcceptAllContactsForThisBodyPair,
        };
        if pushed_up {
            ValidateResult::AcceptContact
        } else {
            ValidateResult::RejectContact
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    world.set_contact_listener(Some(Arc::new(OneWayPlatform)));
    let floor = Shape::new_box(Vec3::new(10.0, 0.5, 10.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)))?;
    let platform = Shape::new_box(Vec3::new(2.0, 0.05, 2.0))?;
    world.create_body(
        &platform,
        &BodySettings::new_static().position(RVec3::new(0.0, 2.0, 0.0)).user_data(PLATFORM),
    )?;
    let cube_shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25))?;
    let cube = world.create_body(
        &cube_shape,
        &BodySettings::new_dynamic()
            .position(RVec3::new(0.0, 0.25, 0.0))
            .linear_velocity(Vec3::new(0.0, 8.0, 0.0)),
    )?;
    for _ in 0..180 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    // Fired up through the platform, the cube came to rest on top of it.
    assert!(world.body(cube)?.position().y > 2.2);
    Ok(())
}
```

- Body order: in the discrete stage `body1` has the higher motion type (dynamic over kinematic over
  static), equal types by lower id (`PhysicsSystem.cpp:1075-1077`); in the continuous stage
  (`MotionQuality::LinearCast`) `body1` is the body being cast, and both points are where the sweep puts
  `body1` at the time of impact as if `body2` did not move: Jolt shifts them by `body2`'s own motion
  only after validation (`PhysicsSystem.cpp:1864-1930`). Contact events and `SubShapeIdPair` order by id
  instead.
- `AcceptContact` and `RejectContact` decide one hit and ask again for the pair's next hit;
  `AcceptAllContactsForThisBodyPair` and `RejectAllContactsForThisBodyPair` end the asking for that
  pair in that collision pass (`PhysicsSystem.cpp:1141-1160`).
- Jolt may ask any number of times per pair and step, in no fixed order, also for hits it then
  drops (the continuous stage validates every hit of its cast with an early-out). Count or log the
  calls only for debugging; the decision must depend only on the candidate and on data fixed for the
  step.
- The contact cache: a pair whose bodies barely moved relative to each other since the last step
  reuses its cached contacts and is not asked again, and the cache remembers "no contact" too
  (`PhysicsSystem.cpp:1079-1094`). A validator whose answer changes for a resting pair takes effect
  after `BodyMut::invalidate_contact_cache` on one of the bodies. The call wakes the body and is
  applied right before the next step in which some body is awake or a vehicle exists; until then it
  is part of the world's state, which `save_state` holds and `restore_state` replaces
  ([state.md](state.md)). Jolt's own invalidation flag is in no saved state and cannot be cleared
  once set, which is why the world defers it.
- Soft body pairs go to `soft_body_contact_validate` instead, and character movement does not call
  it ([below](#character-contacts-charactercontactlistener)).
- Without a user listener the callback accepts the pair before reading anything.

## Listeners

A world installs nothing until `set_event_settings` asks for events or `set_contact_listener` sets a
listener. It then creates the native listeners these need and attaches them to its Jolt system:

| Listener | Created when | Jolt interface |
|---|---|---|
| the extension's `ManagedContactListener2` (`native/joltc_ext/joltc_ext_listener.cpp`) | `contacts`, or a contact listener is set | `ContactListener` |
| joltc's `ManagedBodyActivationListener` (`joltc.cpp:8040-8095`) | `body_activation` | `BodyActivationListener` |
| the extension's `ManagedSoftBodyContactListener` (`native/joltc_ext/joltc_ext_listener.cpp`) | `soft_body_contacts` or `soft_body_validations`, or a contact listener is set | `SoftBodyContactListener` |

The extension's contact listener takes joltc's proc table type and forwards added, persisted and
removed contacts as joltc's does, but fills the validate callback's collide result without the face
arrays that joltc's `FromJolt` allocates on every call (`joltc.cpp:428-461`).

Each listener type calls one process-global proc table, installed once; per-world state goes through
the listener's `userData`, which points at a context the world shares with the native listeners
through an `Arc`. The context's settings never change. Changing the settings detaches and destroys
the old listeners, moves the old context's unqueued events and panic to the world, and builds new
ones. Dropping a world detaches its listeners first, before it removes constraints, vehicles,
ragdolls and character inner bodies, which deactivate bodies.

## Where each event comes from

| Event | Jolt path | Thread |
|---|---|---|
| `ContactEvent::Added` / `Persisted` | `ContactConstraintManager::TemplatedAddContactConstraint` (`ContactConstraintManager.cpp:1166-1183`), `GetContactsFromCache` (`:912`), `OnCCDContactAdded` (`:1387-1454`) | Jolt's workers, in the continuous stage too (`JobFindCCDContacts`, `PhysicsSystem.cpp:1782`), and the stepping thread |
| `ContactEvent::Removed` | `FinalizeContactCacheAndCallContactPointRemovedCallbacks` (`:446-448`), called at the end of a step (`PhysicsSystem.cpp:2461`) | one job |
| `ActivationEvent` | `BodyManager::ActivateBodies` (`BodyManager.cpp:495-520`), `DeactivateBodies` (`:532-565`) under `mActiveBodiesMutex` | workers during a step; the calling thread when a body is created, woken or removed |
| `SoftBodyValidation` | `SoftBodyMotionProperties` collision collector (`SoftBodyMotionProperties.cpp:163`), and continuous collision detection of a rigid body against a soft body (`PhysicsSystem.cpp:1895`) | workers |
| `SoftBodyContacts` | `SoftBodyMotionProperties::UpdateSoftBodyState` (`SoftBodyMotionProperties.cpp:842`) | one worker per soft body |

`PhysicsSystem::RestoreState` changes which bodies are awake through `BodyManager::RestoreState`
(`BodyManager.cpp:810-823`), which does not call the activation listener: a restore reports no
activation changes.

## What the callbacks read

The callbacks copy the data Jolt hands them and nothing else. None of them locks a body, changes the
world or keeps a pointer.

- Validate reads both bodies' ids and user data (`JPH_Body_GetUserData`, Jolt's const
  `Body::GetUserData`) and the collide result the extension filled, without faces. Jolt holds both
  bodies locked during the call.
- Added and Persisted read both bodies' ids (`JPH_Body_GetID`), the manifold through joltc's getters
  (`joltc.cpp:8099-8132`), and each side's material: the body's shape (`JPH_Body_GetShape`, which
  calls Jolt's const `Body::GetShape`) and `Shape::GetMaterial(sub-shape id)`. Jolt does not change
  a body's shape during a step. For the checks of the contact settings they also read whether either
  body is a sensor (`JPH_Body_IsSensor`) and body 1's centre of mass
  (`JPH_Body_GetCenterOfMassPosition`).
- With collision estimates on, Added also runs Jolt's `EstimateCollisionResponse` on the two bodies
  and the manifold, which reads each body's motion type, velocities, inverse mass, inverse inertia
  and centre of mass. No job writes velocities while the contacts are found: gravity is applied
  before, and the solve and the continuous stage's resolve run after.
- Removed reads the four ids of the pair through the extension's `JPH_SubShapeIDPair_*` getters,
  which use Jolt's accessors.
- Activation records the id Jolt passes; Jolt calls it under its active-bodies mutex.
- Soft body validation reads the two ids and the settings only: on the continuous collision path
  another thread may be updating the soft body.
- Soft body contacts read the soft body's centre of mass and rotation and the manifold. Jolt stores
  the manifold's points relative to this step's centre of mass, which it moves right after the
  callback (`SoftBodyMotionProperties.cpp:882-893`), so the callback converts them to world space at
  once: `position = com + R * local`, `normal = R * n`.

## Canonical order

The world sorts the events of each step before it queues them; events from between steps keep the
order they happened in. A `step_with_collision_steps` call is one batch: the events of all its
collision steps are sorted once, so the order is canonical, not chronological across collision
steps.

- Contacts sort by `(body1, sub_shape1, body2, sub_shape2)` raw ids, then Added before Persisted
  before Removed, then by every payload field bit for bit: the settings, the materials, the normal,
  the penetration depth, the point count, each point's coordinates and, for Added, the collision
  estimate (none first). Only identical events compare
  equal, and each copy is kept. One key can appear once per collision step (Added in the one it starts,
  Persisted in each later one), and once more within a collision step: a
  `MotionQuality::LinearCast` body that touches in the discrete stage and is then hit by its own
  continuous cast gets Added from the first and Persisted from the second
  (`ContactConstraintManager.cpp:1421-1454`); Jolt casts each pair at most once per collision step
  (`PhysicsSystem.cpp:1983-1987`). A cube rarely does this, because
  the discrete contact already slows it below the cast threshold; a tilted rod thrown at the floor
  does, because the discrete contact stops its lower end but not its centre of mass (the event
  determinism scene uses one).
- Activations sort stably by body id. Jolt reports one body's changes under its mutex in causal
  order, which the stable sort keeps.
- Soft body validations sort by soft body id, other body id, result and settings bits; one step can
  validate a pair twice (the collision pass and a continuous cast).
- Soft body contacts sort by soft body id and then payload. Within one snapshot the vertices are in
  vertex order, and the sensors are sorted by id because Jolt removes sensors from its list by
  swapping in the last one (`SoftBodyMotionProperties.cpp:835-839`).

The event determinism test (`crates/oxijolt/tests/event_determinism.rs`) compares the serialized
events of every step, with every bit, for 1 and 4 worker threads and for caller job systems, and
checks that its scene stepped with every event on and a no-op contact listener simulates bit for bit
like the same scene with nothing installed. It requires every step to be complete
(`StepReport::is_complete`); steps in which Jolt dropped contacts because a buffer was full are not
covered.

## Impact estimates

`EventSettings::collision_estimates(true)` gives every `ContactEvent::Added` a `CollisionEstimate`:
Jolt's estimate of what the new contact does to its two bodies (Jolt `EstimateCollisionResponse`),
computed on Jolt's workers as the contact is found. It holds both bodies' velocities after the
impact, the impulse along the normal at each manifold point (in the order of the manifold's
points), the two friction directions with their impulses, and the friction impulse about the
normal. The sum of `contact_impulses` is the impact's normal impulse in N·s, a volume for an
impact sound or an input for damage.

Jolt solves the contact of the two bodies alone, with the friction and restitution Jolt resolves
the contact with (after a `ContactListener` changed them) and the world's restitution threshold
and velocity iterations. The estimate is exact only for an isolated pair: other contacts and
constraints on either body in the same step are not part of it, nor are the contact's mass and
inertia scales, surface velocities, locked axes and the speed limits. Gravity and forces are
applied before contacts are found, and the estimate bounces that changed velocity, while the
solver takes the change out of its bounce (`ContactConstraintManager.cpp:88-117`); under gravity
the estimated rebound is faster by `restitution · g · dt` along the normal, 0.08 m/s for a
restitution of 0.5 at 60 Hz. A speculative contact (negative penetration depth) is estimated as
if the bodies touched. A contact first found by the continuous stage is estimated from the velocities after the step's solve, at the time-of-impact
points, so its angular part is approximate; when the discrete stage found the pair in the same
step, the continuous stage reports it as Persisted, without an estimate. Sensor contacts and
contacts without points get `None`. With estimates off nothing is computed.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    world.set_event_settings(EventSettings::default().collision_estimates(true));
    let ground = Shape::new_plane(Vec3::new(0.0, 1.0, 0.0), 0.0, 50.0)?;
    world.create_body(&ground, &BodySettings::new_static())?;
    let crate_shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let falling = BodySettings::new_dynamic().position(RVec3::new(0.0, 3.0, 0.0));
    world.create_body(&crate_shape, &falling)?;

    let mut loudest = 0.0_f32;
    for _ in 0..120 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
        for event in world.take_events().contacts {
            if let ContactEvent::Added {
                estimate: Some(estimate),
                ..
            } = event
            {
                let impulse: f32 = estimate.contact_impulses.iter().sum();
                loudest = loudest.max(impulse);
            }
        }
    }
    assert!(loudest > 0.0);
    Ok(())
}
```

## Character contacts: `CharacterContactListener`

`set_character_contact_listener` installs a `CharacterContactListener`, which Jolt calls while a
character moves (Jolt `CharacterContactListener`):
- `adjust_body_velocity`: the velocity of a body as the character sees it, for moving platforms and
  conveyors. The character's `ground_velocity()` reports it; the caller adds it to the character's
  velocity. `BodyVelocity`'s setters refuse velocities beyond the bounds Jolt clamps bodies to
  ([limits.md](limits.md#character-contacts)).
- `contact_validate`: whether the character collides with a body or another character.
- `contact_added` and `contact_persisted` with `CharacterContactSettings`: `can_push_character`
  (whether what it touches can push the character) and `can_receive_impulses` (whether the
  character pushes dynamic bodies; the weight with which it stands on a body is applied either way,
  `CharacterVirtual.cpp:1474-1481`).
- `contact_removed`, with a `CharacterContactKey`.

```rust
use std::sync::Arc;
use oxijolt::*;

/// The user data of a conveyor belt.
const BELT: u64 = 1;

struct Conveyor;

impl CharacterContactListener for Conveyor {
    fn adjust_body_velocity(&self, _: CharacterId, _: BodyId, user_data: u64, velocity: &mut BodyVelocity) {
        if user_data == BELT {
            velocity.set_linear_velocity(Vec3::new(2.0, 0.0, 0.0)).expect("within the bounds");
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    world.set_character_contact_listener(Some(Arc::new(Conveyor)));
    let belt = Shape::new_box(Vec3::new(20.0, 0.5, 20.0))?;
    world.create_body(
        &belt,
        &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)).user_data(BELT),
    )?;
    let capsule = Shape::new_capsule(0.7, 0.4)?;
    let settings = CharacterSettings::new(&capsule).shape_offset(Vec3::new(0.0, 1.1, 0.0));
    let id = world.create_character(&settings, RVec3::ZERO, Quat::IDENTITY)?;
    world.refresh_character_contacts(id, &QueryFilter::new())?;
    for _ in 0..60 {
        let ground = world.character(id)?.ground_velocity();
        world.character_mut(id)?.set_linear_velocity(Vec3::new(ground.x, -1.0, ground.z))?;
        world.update_character(
            id,
            1.0 / 60.0,
            Vec3::new(0.0, -9.81, 0.0),
            &ExtendedUpdateSettings::default(),
            &QueryFilter::new(),
        )?;
    }
    // The character rode the belt.
    assert!(world.character(id)?.position().x > 1.5);
    Ok(())
}
```

How it runs:
- Jolt calls the methods on the thread that calls `update_character` or
  `refresh_character_contacts`, during that call, for the character it updates. A native listener
  exists only for that call: it is attached before joltc runs and detached and destroyed before the
  call returns. Methods must not touch the world; `adjust_body_velocity` runs while Jolt holds the
  body's read lock, so a method that locks something that also guards the world can deadlock.
- `adjust_body_velocity` and `contact_validate` may be called any number of times per update, in no
  fixed order, also for hits Jolt does not use (`CharacterVirtual.cpp:178-195`, `:337-381`).
- One update (with stick to floor and walk stairs) or one refresh is one tracking scope: each contact
  is reported at most once, as added when it is new and as persisted when the character touched it at
  the end of its previous update or refresh, also when no listener was set then
  (`CharacterVirtual.cpp:1385-1439`). A listener set while the character stands on the ground
  therefore first sees that contact as persisted.
- Removals are delivered after the update or refresh returned, sorted by body, character and
  sub-shape raw id ("none" last). Jolt reports them in the bucket order of a hash map whose capacity
  depends on earlier updates, which would make the order depend on history. The body or character
  of a removal may no longer exist.
- Jolt gives a contact with another character that character's velocity
  (`CharacterVirtual.cpp:243`), so `can_push_character` decides whether a character moving into this
  one pushes it. Two characters that collide with each other stop at each other's padding; whether
  one standing on the floor is then pushed along by a walker depends on a float tie with its floor
  contact and differed between single and double precision in the tests.
- A panic in a method Jolt calls is caught; Jolt keeps its own value for that call, the rest of the
  update skips the methods (and its query filters reject), the update resumes the panic after joltc
  returned, and that update's removals are not delivered. `contact_removed` is called from Rust
  after joltc returned and is not caught: its panic leaves the update at once, the removals sorted
  before it have been delivered and the rest are lost.
- Jolt's solve callbacks (`OnContactSolve`, `OnCharacterContactSolve`) are not installed.

## Lifetimes and the queue

- Events wait in the world until `take_events`; nothing bounds the queue, so a caller that records
  events takes them every step. The queue has no step boundaries.
- `restore_state` neither clears nor rewinds the queue: take the events before a rollback and drop
  the ones of the abandoned steps.
- A Removed pair or Deactivated id can name a body that was removed; it is not looked up.
- Enabling contacts while bodies touch reports their next contacts as Persisted or Removed without
  an Added, because Jolt reports against its contact cache.

## Panics

Every callback runs its body under `catch_unwind`. The first panic is kept and resumed by `step`
after the update returned (after a caller job system's `queue_job` panic, if any), or by
`take_events` when it happened between steps; later panics of the same update are dropped, also when
their payload's `Drop` panics. Recording continues after a panic, and the next step works normally.

Once a panic is kept, callbacks that start skip the user's `ContactListener` until the panic is
resumed. A worker that already passed that check, or is inside the listener, finishes its call; its
settings are still applied when they are valid. A validation that panics, or is skipped, accepts the
pair (`AcceptAllContactsForThisBodyPair`).

Character contact listeners resume their panics from `update_character` and
`refresh_character_contacts` instead ([above](#character-contacts-charactercontactlistener)).

## Rejected contact settings

A `ContactListener` can assign a whole `ContactSettings` value it kept from another contact. The
setters checked that value against the contact it was read for, so the callback checks the returned
value again, field by field, against the contact it runs for: its sensor bodies and the lever of its
surface velocity (`docs/limits.md`, "Contact settings"). Without this check a kept ordinary value
turns a sensor contact into a solid one, which Jolt asserts against
(`ContactConstraintManager.cpp:915`).

A value that fails is not written to Jolt, which keeps its own settings for that contact. The
callback records a `ContactSettingsRejection` (the pair and the first rule broken) in
`WorldEvents::rejected_contact_settings`, whatever the event settings, and `StepReport` counts the
step's rejections. A rejection is not a panic: the listener is still called for every other contact
of the step, so which contacts get its settings does not depend on the thread count. Rejections sort
by pair and error like the other events of a step.

## Leaks

`crates/oxijolt/tests/listener_leaks.rs` measures the process's private bytes over two phases:
replacing listeners and materials in one world, and creating, stepping and dropping a world with
listeners, material shapes, bodies and a cloth every round (dropped with its listeners attached and
its bodies alive). A new world per round grows private bytes by about 1 to 2 MB over its first few
thousand rounds and then stops. Measured locally (one test per process): with 50 warm-up rounds,
four consecutive blocks of 3 000 rounds grew by 1.8 MB, 0.19 MB, 0 and 0; this holds with Jolt's
thread pool (1 or 4 workers) or a caller job system, for empty, populated and stepped worlds, with
or without listeners, and spawning and joining a thread alone does not grow it. After the plateau,
blocks of 6 000 rounds moved between -1.8 MB and +0.2 MB. It is the heap settling, not an object per
world left behind.

The settling also comes late: a single window of 6 000 rounds after 4 000 warm-up rounds grew by 1.8
MB and 2.0 MB in two CI runs on `windows-latest`. Locally it grew by 1.7 to 1.8 MB in two of nine
runs, each time inside one block of 1 000 rounds while every other block stayed within ±210 kB, and
another run released 1.8 MB in one block. So each phase measures seven consecutive blocks of 1 000
rounds (the world phase after 4 000 warm-up rounds, the replacement phase after 500) and allows the
median block 100 bytes per round (100 kB). A leak grows every block, a heap step only one. The
median also passes a leak that grows only three or fewer of the seven blocks: bursts of 400 kB every
2 000 rounds or 1.5 MB every 3 000 rounds pass. Against this gate, materials that were never
released gave a median of 370 kB per block, native contact listeners that were never destroyed 780
kB, and a dropped world that forgot its listeners 1.26 MB; each failed it. A leak of one small
native object per world (a forgotten `JPH_ContactListener` alone: about 33 bytes per round) stays
below what the block noise lets this gate resolve.

`crates/oxijolt/tests/contact_control_leaks.rs` applies the same method to contact control: per
round ten validating contact listeners, a group table of 64 sub groups, four grouped cubes and a
grouped cloth, ten character updates with a character contact listener (one native listener per
update) and a contact-cache invalidation, in one world and then in a new world per round. Measured
locally, a per-update character listener that was never destroyed gave a median of 389 kB per block
of 1 000 rounds, a group table never released 348 kB, and a native contact listener never destroyed
389 kB; each failed the 100 kB budget. A table of 8 sub groups leaked every round stays at about the
budget (median 98 kB), which is why the gate uses 64. The median's blind spot is the same: a leak
that grows three or fewer of the seven blocks passes.
