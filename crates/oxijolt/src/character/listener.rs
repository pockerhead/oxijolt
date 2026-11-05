//! Character contact listeners: Jolt's `CharacterContactListener`, called while a character
//! updates or refreshes its contacts.
//!
//! A native listener exists only for the duration of one [`PhysicsWorld::update_character`] or
//! [`PhysicsWorld::refresh_character_contacts`] call: it is created and attached before joltc
//! runs, and detached and destroyed before the call returns. Its `userData` points to state on
//! that call's stack, and the callbacks catch panics into the call's
//! [`FilterState`], which resumes the first one after joltc returned.

use std::cell::RefCell;
use std::ffi::c_void;
use std::ptr::{null_mut, NonNull};
use std::sync::{Arc, OnceLock};

use oxijolt_sys::*;

use super::{CharacterContact, CharacterId, INVALID_ID};
use crate::filter::FilterState;
use crate::limits::{
    is_angular_velocity, is_linear_velocity, ANGULAR_VELOCITY_RULE, LINEAR_VELOCITY_RULE,
};
use crate::owned::{JoltObject, Owned};
use crate::world::WorldTag;
use crate::{BodyId, CharacterError, PhysicsWorld, SubShapeId, Vec3};

/// Changes how characters touch bodies and each other (Jolt `CharacterContactListener`); see
/// [`PhysicsWorld::set_character_contact_listener`].
///
/// Jolt calls the methods on the thread that calls
/// [`update_character`](PhysicsWorld::update_character) or
/// [`refresh_character_contacts`](PhysicsWorld::refresh_character_contacts), during that call,
/// for the character it updates. A method must not touch the world; one that locks a value which
/// also guards the world can deadlock. For a replayable run its decisions must depend only on its
/// arguments and on data fixed for the update.
///
/// [`adjust_body_velocity`](Self::adjust_body_velocity) and
/// [`contact_validate`](Self::contact_validate) may be called any number of times per update, in
/// no fixed order, also for hits Jolt then does not use. Added, persisted and removed contacts
/// follow the character's contacts: one update or refresh (stick to floor and walk stairs
/// included) reports each contact at most once, as added when it is new and as persisted when the
/// character touched it at the end of its previous update or refresh, also when no listener was
/// set then. Removals arrive after the update, sorted by [`CharacterContactKey`].
///
/// A panic in a method that Jolt calls is caught; Jolt then keeps its own value for that call,
/// the methods of the rest of that update are skipped (as are the query filter's, which reject),
/// and the update resumes the panic after Jolt returned. Removals of that update are not
/// delivered. [`contact_removed`](Self::contact_removed) runs after Jolt returned and is not
/// caught: its panic leaves the update at once, the removals sorted before it have been
/// delivered and the rest of that update's removals are lost. An update that panicked is outside
/// the replay guarantee. On wasm32 a panic traps instead
/// ([wasm32](crate#wasm32-unknown-unknown)).
pub trait CharacterContactListener: Send + Sync + 'static {
    /// The velocity of `body` as `character` sees it: what the character stands on moves it
    /// along (moving platforms, conveyors). Jolt passes the body's own velocity, zero for a static
    /// body; `user_data` is the body's
    /// [`BodySettings::user_data`](crate::BodySettings::user_data). Called while Jolt holds the
    /// body's read lock.
    fn adjust_body_velocity(
        &self,
        character: CharacterId,
        body: BodyId,
        user_data: u64,
        velocity: &mut BodyVelocity,
    ) {
        let _ = (character, body, user_data, velocity);
    }

    /// Whether `character` may collide with the body or character of `contact`. The default
    /// accepts every contact, as Jolt does without a listener.
    fn contact_validate(&self, character: CharacterId, contact: &CharacterContact) -> bool {
        let _ = (character, contact);
        true
    }

    /// `character` touches the body or character of `contact` for the first time; `settings`
    /// may be changed for it.
    fn contact_added(
        &self,
        character: CharacterId,
        contact: &CharacterContact,
        settings: &mut CharacterContactSettings,
    ) {
        let _ = (character, contact, settings);
    }

    /// `character` still touches the body or character of `contact`; `settings` may be changed
    /// for this update.
    fn contact_persisted(
        &self,
        character: CharacterId,
        contact: &CharacterContact,
        settings: &mut CharacterContactSettings,
    ) {
        let _ = (character, contact, settings);
    }

    /// `character` no longer touches what `contact` names. Called after the update or refresh
    /// returned; the body or character may no longer exist.
    fn contact_removed(&self, character: CharacterId, contact: CharacterContactKey) {
        let _ = (character, contact);
    }
}

/// The velocity of a body as a character sees it, in
/// [`CharacterContactListener::adjust_body_velocity`].
///
/// The setters refuse a velocity no body could have: it must be finite and at most
/// [`limits::MAX_LINEAR_VELOCITY`](crate::limits::MAX_LINEAR_VELOCITY) or
/// [`limits::MAX_ANGULAR_VELOCITY`](crate::limits::MAX_ANGULAR_VELOCITY) long, the bounds Jolt
/// clamps every body to. See [docs/limits.md#character-contacts].
///
/// [docs/limits.md#character-contacts]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#character-contacts
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyVelocity {
    linear: Vec3,
    angular: Vec3,
}

impl BodyVelocity {
    /// Linear velocity of the body's centre of mass, m/s.
    pub fn linear_velocity(&self) -> Vec3 {
        self.linear
    }

    /// Angular velocity, rad/s.
    pub fn angular_velocity(&self) -> Vec3 {
        self.angular
    }

    /// Sets the linear velocity, m/s; refused with [`CharacterError::InvalidValue`] outside the
    /// bounds above.
    pub fn set_linear_velocity(&mut self, value: Vec3) -> Result<(), CharacterError> {
        if !is_linear_velocity(value) {
            return Err(CharacterError::InvalidValue(LINEAR_VELOCITY_RULE));
        }
        self.linear = value;
        Ok(())
    }

    /// Sets the angular velocity, rad/s; refused with [`CharacterError::InvalidValue`] outside
    /// the bounds above.
    pub fn set_angular_velocity(&mut self, value: Vec3) -> Result<(), CharacterError> {
        if !is_angular_velocity(value) {
            return Err(CharacterError::InvalidValue(ANGULAR_VELOCITY_RULE));
        }
        self.angular = value;
        Ok(())
    }
}

/// How a character and what it touches push each other (Jolt `CharacterContactSettings`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CharacterContactSettings {
    /// Whether what the character touches can push it: `false` makes the character ignore the
    /// contact's velocity (a moving wall or another character does not push it).
    pub can_push_character: bool,
    /// Whether the character pushes what it touches with impulses. The weight with which it
    /// stands on a body is applied whatever this says.
    pub can_receive_impulses: bool,
}

/// What a removed character contact touched (Jolt `CharacterContactListener::OnContactRemoved`
/// and `OnCharacterContactRemoved`).
///
/// An update delivers its removals sorted by body, then character, then sub-shape, each by raw
/// value with "none" last.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CharacterContactKey {
    /// The body touched, or `None` for another character.
    pub body: Option<BodyId>,
    /// The character touched, or `None` for a body.
    pub character: Option<CharacterId>,
    /// The leaf of the touched shape.
    pub sub_shape_id: SubShapeId,
}

impl CharacterContactKey {
    /// The raw values removals are sorted by, `INVALID_ID` (Jolt's) for "none".
    fn sort_key(&self) -> (u32, u32, u32) {
        (
            self.body.map_or(INVALID_ID, BodyId::to_raw),
            self.character.map_or(INVALID_ID, CharacterId::to_raw),
            self.sub_shape_id.to_raw(),
        )
    }
}

impl PhysicsWorld {
    /// Lets `listener` change how characters touch bodies and each other from the next
    /// [`update_character`](Self::update_character) or
    /// [`refresh_character_contacts`](Self::refresh_character_contacts) on, or removes it with
    /// `None`; see [`CharacterContactListener`]. The world keeps the `Arc` while it is set.
    pub fn set_character_contact_listener(
        &mut self,
        listener: Option<Arc<dyn CharacterContactListener>>,
    ) {
        self.character_listener = listener;
    }
}

/// A removed contact of the character that was updated.
pub(super) type Removal = (CharacterId, CharacterContactKey);

/// What the callbacks of one update or refresh need, on that call's stack.
///
/// A `RefCell` suffices: Jolt runs a character update and every callback on the calling thread.
struct CallState<'a, 'f> {
    listener: &'a dyn CharacterContactListener,
    world: WorldTag,
    filter: &'a FilterState<'f>,
    removed: RefCell<Vec<Removal>>,
}

/// A native character contact listener, owned whole by one update or refresh call.
impl JoltObject for JPH_CharacterContactListener {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the listener (trait contract) and detached it from its
        // character first, so nothing calls it any more.
        unsafe { JPH_CharacterContactListener_Destroy(ptr) };
    }
}

/// Detaches the native listener from the character when dropped.
struct Attached {
    character: NonNull<JPH_CharacterVirtual>,
}

impl Drop for Attached {
    fn drop(&mut self) {
        // SAFETY: the character is live for the whole update or refresh, which holds the world
        // mutably; null detaches.
        unsafe { JPH_CharacterVirtual_SetListener(self.character.as_ptr(), null_mut()) };
    }
}

/// Runs `run` (a joltc update, refresh or shape change of `character`) with `listener`
/// attached to the character, and returns its result with the removals the update reported,
/// sorted.
///
/// # Safety
/// `character` is a live character of the world `state` belongs to, which the caller holds
/// mutably for the whole call, and `run` updates, refreshes or reshapes only that character.
pub(super) unsafe fn with_character_listener<R>(
    state: &FilterState<'_>,
    listener: &dyn CharacterContactListener,
    world: WorldTag,
    character: NonNull<JPH_CharacterVirtual>,
    run: impl FnOnce() -> R,
) -> (R, Vec<Removal>) {
    install_procs();
    let call = CallState {
        listener,
        world,
        filter: state,
        removed: RefCell::new(Vec::new()),
    };
    let user_data = (&call as *const CallState<'_, '_>)
        .cast_mut()
        .cast::<c_void>();
    // SAFETY: joltc `new`s the listener with `user_data` and the handle owns it entirely.
    // `call` outlives it: the listener is destroyed before this function returns.
    let native = unsafe { Owned::from_raw(JPH_CharacterContactListener_Create(user_data)) }
        .unwrap_or_else(|| unreachable!("joltc `new`s the listener"));
    // SAFETY: the character is live (contract). Jolt reads the listener only inside the
    // character's update and refresh (`CharacterVirtual.cpp:193, 486, 497, 1393, 1412`).
    unsafe { JPH_CharacterVirtual_SetListener(character.as_ptr(), native.as_ptr()) };
    // Declared after `native`, so it drops first: the listener is detached before it is
    // destroyed, also during an unwind.
    let attached = Attached { character };
    let result = run();
    drop(attached);
    drop(native);
    let mut removed = call.removed.into_inner();
    sort_removals(&mut removed);
    (result, removed)
}

/// Sorts an update's removals, which are all of one character, by their keys. Jolt reports them
/// in the bucket order of a hash map whose capacity depends on earlier updates
/// (`CharacterVirtual.cpp:1430-1439`).
pub(super) fn sort_removals(removed: &mut [Removal]) {
    removed.sort_by_key(|(character, key)| (key.sort_key(), character.to_raw()));
}

static CHARACTER_PROCS: JPH_CharacterContactListener_Procs = JPH_CharacterContactListener_Procs {
    OnAdjustBodyVelocity: Some(on_adjust_body_velocity),
    OnContactValidate: Some(on_contact_validate),
    OnCharacterContactValidate: Some(on_contact_validate),
    OnContactAdded: Some(on_contact_added),
    OnContactPersisted: Some(on_contact_persisted),
    OnContactRemoved: Some(on_contact_removed),
    OnCharacterContactAdded: Some(on_contact_added),
    OnCharacterContactPersisted: Some(on_contact_persisted),
    OnCharacterContactRemoved: Some(on_character_contact_removed),
    // Left unset: joltc then keeps Jolt's solver velocity. These pass the other character's
    // pointer, which the safe layer never reads.
    OnContactSolve: None,
    OnCharacterContactSolve: None,
};

/// Points joltc's global character contact listener table at these callbacks, once per process.
fn install_procs() {
    static INSTALLED: OnceLock<()> = OnceLock::new();
    INSTALLED.get_or_init(|| {
        // SAFETY: the table is an immutable static, so the pointer stays valid forever.
        // `SetProcs` only stores it in a plain global; `OnceLock` makes that one write happen
        // before every read by a thread that returns from `get_or_init`, and every listener is
        // created after this call on the creating thread. oxijolt never calls `SetProcs` again,
        // and the raw-API contract of `oxijolt-sys` forbids other code to.
        unsafe { JPH_CharacterContactListener_SetProcs(&CHARACTER_PROCS) };
    });
}

/// The call state behind a native listener's `userData`.
///
/// # Safety
/// `user_data` is the pointer [`with_character_listener`] gave a native listener, which joltc
/// calls only while that function runs.
unsafe fn call_state<'s>(user_data: *mut c_void) -> &'s CallState<'s, 's> {
    // SAFETY: the caller guarantees a live `CallState`, only ever shared while the call runs.
    unsafe { &*user_data.cast::<CallState<'s, 's>>() }
}

/// The id of the character a callback is for.
///
/// # Safety
/// `character` is the live character joltc passes to a callback.
unsafe fn character_id(character: *const JPH_CharacterVirtual, world: WorldTag) -> CharacterId {
    // SAFETY: the character is live (contract); the getter reads a member.
    CharacterId::new(unsafe { JPH_CharacterVirtual_GetID(character) }, world)
}

/// joltc's `OnAdjustBodyVelocity`.
///
/// # Safety
/// Called by joltc with the `userData` of a listener of [`with_character_listener`], the live
/// character and body, and live velocities that may be written.
unsafe extern "C" fn on_adjust_body_velocity(
    user_data: *mut c_void,
    character: *const JPH_CharacterVirtual,
    body: *const JPH_Body,
    linear: *mut JPH_Vec3,
    angular: *mut JPH_Vec3,
) {
    // SAFETY: joltc passes the listener's `userData` (contract).
    let state = unsafe { call_state(user_data) };
    state.filter.guarded((), || {
        #[cfg(test)]
        tests::panic_if_injected(Callback::AdjustBodyVelocity);
        // SAFETY: the arguments are live for the callback (contract), and Jolt holds the body's
        // read lock. joltc's `JPH_Body_GetUserData` takes a mutable pointer but only calls
        // Jolt's const `Body::GetUserData`, so nothing is written through the cast.
        let (id, body, user_data, original) = unsafe {
            (
                character_id(character, state.world),
                BodyId::new(JPH_Body_GetID(body), state.world),
                JPH_Body_GetUserData(body.cast_mut()),
                BodyVelocity {
                    linear: Vec3::from_jph(*linear),
                    angular: Vec3::from_jph(*angular),
                },
            )
        };
        let mut velocity = original;
        state
            .listener
            .adjust_body_velocity(id, body, user_data, &mut velocity);
        if velocity != original {
            // SAFETY: joltc's live locals, which it copies to Jolt after the callback.
            unsafe {
                *linear = velocity.linear.to_jph();
                *angular = velocity.angular.to_jph();
            }
        }
    });
}

/// joltc's `OnContactValidate` and `OnCharacterContactValidate`.
///
/// # Safety
/// Called by joltc with the `userData` of a listener of [`with_character_listener`], the live
/// character and its live contact.
unsafe extern "C" fn on_contact_validate(
    user_data: *mut c_void,
    character: *const JPH_CharacterVirtual,
    contact: *const JPH_CharacterContact,
) -> bool {
    // SAFETY: joltc passes the listener's `userData` (contract).
    let state = unsafe { call_state(user_data) };
    state.filter.guarded(true, || {
        #[cfg(test)]
        tests::panic_if_injected(Callback::Validate);
        // SAFETY: the arguments are live for the callback (contract); the contact is read by
        // value and its character pointer is not read.
        let (id, contact) = unsafe {
            (
                character_id(character, state.world),
                CharacterContact::from_jph(&*contact, state.world),
            )
        };
        state.listener.contact_validate(id, &contact)
    })
}

/// Which callback with settings ran.
#[derive(Clone, Copy)]
enum Kind {
    Added,
    Persisted,
}

/// joltc's added and persisted callbacks, for bodies and characters alike.
///
/// # Safety
/// As for [`on_contact_added`].
unsafe fn on_contact_with_settings(
    kind: Kind,
    user_data: *mut c_void,
    character: *const JPH_CharacterVirtual,
    contact: *const JPH_CharacterContact,
    settings: *mut JPH_CharacterContactSettings,
) {
    // SAFETY: joltc passes the listener's `userData` (contract).
    let state = unsafe { call_state(user_data) };
    state.filter.guarded((), || {
        #[cfg(test)]
        tests::panic_if_injected(match kind {
            Kind::Added => Callback::Added,
            Kind::Persisted => Callback::Persisted,
        });
        // SAFETY: the arguments are live for the callback (contract); the contact is read by
        // value and its character pointer is not read.
        let (id, contact, original) = unsafe {
            (
                character_id(character, state.world),
                CharacterContact::from_jph(&*contact, state.world),
                CharacterContactSettings {
                    can_push_character: (*settings).canPushCharacter,
                    can_receive_impulses: (*settings).canReceiveImpulses,
                },
            )
        };
        let mut changed = original;
        match kind {
            Kind::Added => state.listener.contact_added(id, &contact, &mut changed),
            Kind::Persisted => state.listener.contact_persisted(id, &contact, &mut changed),
        }
        // SAFETY: joltc's live local, which it copies to Jolt after the callback.
        unsafe {
            (*settings).canPushCharacter = changed.can_push_character;
            (*settings).canReceiveImpulses = changed.can_receive_impulses;
        }
    });
}

/// joltc's `OnContactAdded` and `OnCharacterContactAdded`.
///
/// # Safety
/// Called by joltc with the `userData` of a listener of [`with_character_listener`], the live
/// character, its live contact and live settings that may be written.
unsafe extern "C" fn on_contact_added(
    user_data: *mut c_void,
    character: *const JPH_CharacterVirtual,
    contact: *const JPH_CharacterContact,
    settings: *mut JPH_CharacterContactSettings,
) {
    // SAFETY: as the caller guarantees.
    unsafe { on_contact_with_settings(Kind::Added, user_data, character, contact, settings) };
}

/// joltc's `OnContactPersisted` and `OnCharacterContactPersisted`.
///
/// # Safety
/// As for [`on_contact_added`].
unsafe extern "C" fn on_contact_persisted(
    user_data: *mut c_void,
    character: *const JPH_CharacterVirtual,
    contact: *const JPH_CharacterContact,
    settings: *mut JPH_CharacterContactSettings,
) {
    // SAFETY: as the caller guarantees.
    unsafe { on_contact_with_settings(Kind::Persisted, user_data, character, contact, settings) };
}

/// Records a removal for delivery after the call.
///
/// # Safety
/// `user_data` and `character` are the live arguments of a removal callback of a listener of
/// [`with_character_listener`].
unsafe fn record_removal(
    user_data: *mut c_void,
    character: *const JPH_CharacterVirtual,
    key: impl FnOnce(WorldTag) -> CharacterContactKey,
) {
    // SAFETY: joltc passes the listener's `userData` (contract).
    let state = unsafe { call_state(user_data) };
    state.filter.guarded((), || {
        #[cfg(test)]
        tests::panic_if_injected(Callback::Removed);
        // SAFETY: the character is live for the callback (contract).
        let id = unsafe { character_id(character, state.world) };
        let key = key(state.world);
        state.removed.borrow_mut().push((id, key));
    });
}

/// joltc's `OnContactRemoved`.
///
/// # Safety
/// Called by joltc with the `userData` of a listener of [`with_character_listener`] and the live
/// character.
unsafe extern "C" fn on_contact_removed(
    user_data: *mut c_void,
    character: *const JPH_CharacterVirtual,
    body: JPH_BodyID,
    sub_shape: JPH_SubShapeID,
) {
    // SAFETY: as the caller guarantees.
    unsafe {
        record_removal(user_data, character, |world| CharacterContactKey {
            body: Some(BodyId::new(body, world)),
            character: None,
            sub_shape_id: SubShapeId::new(sub_shape),
        })
    };
}

/// joltc's `OnCharacterContactRemoved`.
///
/// # Safety
/// As for [`on_contact_removed`].
unsafe extern "C" fn on_character_contact_removed(
    user_data: *mut c_void,
    character: *const JPH_CharacterVirtual,
    other: JPH_CharacterID,
    sub_shape: JPH_SubShapeID,
) {
    // SAFETY: as the caller guarantees.
    unsafe {
        record_removal(user_data, character, |world| CharacterContactKey {
            body: None,
            character: Some(CharacterId::new(other, world)),
            sub_shape_id: SubShapeId::new(sub_shape),
        })
    };
}

/// The callbacks, named for the panic tests.
#[cfg(test)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Callback {
    AdjustBodyVelocity,
    Validate,
    Added,
    Persisted,
    Removed,
}

#[cfg(test)]
pub(super) mod tests {
    use std::cell::Cell;

    use super::Callback;

    thread_local! {
        /// The callback that panics on this thread. Character callbacks run on the updating
        /// thread, so tests running in parallel do not see each other's choice.
        static INJECTED_PANIC: Cell<Option<Callback>> = const { Cell::new(None) };
    }

    /// Makes `callback` panic on this thread from now on, or nothing with `None`.
    pub(in crate::character) fn inject_panic(callback: Option<Callback>) {
        INJECTED_PANIC.set(callback);
    }

    /// Panics when the running test made `callback` panic.
    pub(super) fn panic_if_injected(callback: Callback) {
        if INJECTED_PANIC.get() == Some(callback) {
            panic!("injected {callback:?} panic");
        }
    }
}
