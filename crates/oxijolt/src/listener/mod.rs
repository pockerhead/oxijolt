//! Contact, body activation and soft body contact events.
//!
//! A world records nothing until [`PhysicsWorld::set_event_settings`] asks for events. Jolt
//! reports them from its worker threads during [`PhysicsWorld::step`], and from the calling
//! thread when a body is created, woken or removed; the callbacks only copy what Jolt hands
//! them into the world's buffers. After each step, the step's events are sorted into an order
//! that does not depend on the thread count and appended to a queue, which
//! [`PhysicsWorld::take_events`] drains. `docs/events.md` lists the Jolt paths behind each
//! event and what the callbacks may read.

mod activation;
mod contact;
mod estimate;
mod order;
mod soft_body;
mod validate;

use std::any::Any;
use std::ffi::c_void;
use std::mem;
use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::ptr::null_mut;
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};

use oxijolt_sys::*;

use crate::owned::{JoltObject, Owned};
use crate::panic_slot::{first_payload, PanicSlot};
use crate::world::WorldTag;
use crate::{BodyId, PhysicsWorld};
use estimate::SolverSettings;

pub use activation::ActivationEvent;
pub use contact::{
    ContactEvent, ContactManifold, ContactPoint, ContactSettings, ContactSettingsRejection,
    SubShapeIdPair,
};
pub use estimate::CollisionEstimate;
pub use soft_body::{
    SoftBodyContactSettings, SoftBodyContacts, SoftBodyValidateResult, SoftBodyValidation,
    SoftBodyVertexContact,
};
pub use validate::{ContactCandidate, ValidateResult};

/// Changes how Jolt resolves contacts while a world steps; see
/// [`PhysicsWorld::set_contact_listener`].
///
/// Jolt calls the methods on its worker threads during [`PhysicsWorld::step`], concurrently and
/// in no fixed order, while it holds every body: a method must not touch the world. For a
/// deterministic simulation its decision must depend only on its arguments and on data fixed
/// for the step; a mutex makes shared state safe to use here, not deterministic. Jolt may call
/// [`contact_validate`](Self::contact_validate) any number of times per pair and step; only
/// the added and persisted calls follow the contacts.
///
/// A panic in a method is resumed by `step` after the update. Jolt then keeps its own settings
/// for that contact (or accepts the hit, for a validation), and calls that start after the panic skip the listener until the panic is
/// resumed; calls already running on other threads finish. The step that panicked has advanced
/// the world: it is outside the replay guarantee. On wasm32 a panic traps instead
/// ([wasm32](crate#wasm32-unknown-unknown)).
///
/// Settings a method leaves invalid for its contact (a value of another contact assigned over
/// them, see [`ContactSettings`]) are not applied: Jolt keeps its own settings for that
/// contact, the world records a [`ContactSettingsRejection`] in
/// [`WorldEvents::rejected_contact_settings`] and counts it in
/// [`StepReport::rejected_contact_settings`](crate::StepReport::rejected_contact_settings),
/// and the listener is still called for every other contact.
pub trait ContactListener: Send + Sync + 'static {
    /// Whether Jolt keeps a hit between two rigid bodies, before it becomes a contact. The
    /// default accepts every hit, as Jolt does without a listener.
    ///
    /// Jolt asks during the step's collision passes, on worker threads and concurrently for
    /// different pairs (one pair at a time), while it holds both bodies. The body order is the
    /// one [`ContactCandidate`] states. Jolt may ask any number of times per pair and step, in
    /// no fixed order, also for hits it then drops; [`AcceptContact`] and [`RejectContact`] ask
    /// again for the pair's next hit, the two "all" answers end the asking for that pair in
    /// that collision pass.
    ///
    /// A pair whose contact cache Jolt reuses (the bodies barely moved relative to each other)
    /// is not asked again and keeps its last answer, "no contact" included, until the bodies
    /// move enough relative to each other or
    /// [`BodyMut::invalidate_contact_cache`](crate::BodyMut::invalidate_contact_cache) is called
    /// for one of them. Soft body pairs go to [`soft_body_contact_validate`](Self::soft_body_contact_validate)
    /// instead, and character movement does not call it. `docs/events.md` (section
    /// "Validating contacts") has the details.
    ///
    /// [`AcceptContact`]: ValidateResult::AcceptContact
    /// [`RejectContact`]: ValidateResult::RejectContact
    fn contact_validate(&self, contact: &ContactCandidate) -> ValidateResult {
        let _ = contact;
        ValidateResult::AcceptAllContactsForThisBodyPair
    }

    /// A rigid contact appeared; `settings` may be changed for it.
    fn contact_added(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        let _ = (manifold, settings);
    }

    /// A rigid contact lasted from the previous step; `settings` may be changed for this step.
    fn contact_persisted(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        let _ = (manifold, settings);
    }

    /// A soft body's bounding box overlaps another body's; `settings` may be changed for the
    /// contacts of this step, or the contacts rejected.
    fn soft_body_contact_validate(
        &self,
        soft_body: BodyId,
        other: BodyId,
        settings: &mut SoftBodyContactSettings,
    ) -> SoftBodyValidateResult {
        let _ = (soft_body, other, settings);
        SoftBodyValidateResult::AcceptContact
    }
}

/// Which events a [`PhysicsWorld`] records. The default records nothing.
///
/// ```
/// use oxijolt::EventSettings;
///
/// let settings = EventSettings::default().contacts(true).body_activation(true);
/// assert!(settings.reports_contacts());
/// assert!(!settings.reports_persisted_contacts());
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EventSettings {
    contacts: bool,
    persisted_contacts: bool,
    collision_estimates: bool,
    body_activation: bool,
    soft_body_contacts: bool,
    soft_body_validations: bool,
}

impl EventSettings {
    /// Records [`ContactEvent::Added`] and [`ContactEvent::Removed`]. Turning it off also turns
    /// off [`persisted_contacts`](Self::persisted_contacts) and
    /// [`collision_estimates`](Self::collision_estimates).
    #[must_use]
    pub fn contacts(mut self, value: bool) -> Self {
        self.contacts = value;
        self.persisted_contacts &= value;
        self.collision_estimates &= value;
        self
    }

    /// Records [`ContactEvent::Persisted`] for every contact that lasts from one step to the
    /// next, which is most contacts of a resting scene. Turning it on also turns on
    /// [`contacts`](Self::contacts).
    #[must_use]
    pub fn persisted_contacts(mut self, value: bool) -> Self {
        self.persisted_contacts = value;
        self.contacts |= value;
        self
    }

    /// Gives every [`ContactEvent::Added`] Jolt's [`CollisionEstimate`] of the impact, computed
    /// on Jolt's worker threads as the contact is found; sensor contacts get none. Turning it on
    /// also turns on [`contacts`](Self::contacts). Off by default, and free when off.
    #[must_use]
    pub fn collision_estimates(mut self, value: bool) -> Self {
        self.collision_estimates = value;
        self.contacts |= value;
        self
    }

    /// Records [`ActivationEvent`]s.
    #[must_use]
    pub fn body_activation(mut self, value: bool) -> Self {
        self.body_activation = value;
        self
    }

    /// Records a [`SoftBodyContacts`] for every soft body that touched a rigid body in a step.
    #[must_use]
    pub fn soft_body_contacts(mut self, value: bool) -> Self {
        self.soft_body_contacts = value;
        self
    }

    /// Records a [`SoftBodyValidation`] for every soft body whose bounding box overlapped
    /// another body's in a step.
    #[must_use]
    pub fn soft_body_validations(mut self, value: bool) -> Self {
        self.soft_body_validations = value;
        self
    }

    /// Whether added and removed contacts are recorded.
    pub fn reports_contacts(&self) -> bool {
        self.contacts
    }

    /// Whether persisted contacts are recorded.
    pub fn reports_persisted_contacts(&self) -> bool {
        self.persisted_contacts
    }

    /// Whether added contacts carry collision estimates.
    pub fn reports_collision_estimates(&self) -> bool {
        self.collision_estimates
    }

    /// Whether body activation changes are recorded.
    pub fn reports_body_activation(&self) -> bool {
        self.body_activation
    }

    /// Whether soft body contacts are recorded.
    pub fn reports_soft_body_contacts(&self) -> bool {
        self.soft_body_contacts
    }

    /// Whether soft body validations are recorded.
    pub fn reports_soft_body_validations(&self) -> bool {
        self.soft_body_validations
    }

    fn needs_contact_listener(&self) -> bool {
        self.contacts
    }

    fn needs_soft_body_listener(&self) -> bool {
        self.soft_body_contacts || self.soft_body_validations
    }

    fn needs_any_listener(&self) -> bool {
        self.needs_contact_listener() || self.body_activation || self.needs_soft_body_listener()
    }
}

/// The events a world recorded, in recording order across steps; see
/// [`PhysicsWorld::take_events`].
///
/// Within the events of one step, each list is in a canonical order: contacts and rejected
/// contact settings by their [`SubShapeIdPair`] as `(body1, sub_shape1, body2, sub_shape2)`,
/// contacts then by kind, Added before Persisted before Removed; activations by body id; soft
/// body events by soft body id. Equal keys keep every event, so a pair can appear twice in one
/// step (a [`MotionQuality::LinearCast`](crate::MotionQuality) body's discrete and continuous
/// contact). Events of different steps are not separated. `docs/events.md` has the details.
#[derive(Clone, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct WorldEvents {
    /// Rigid body contacts.
    pub contacts: Vec<ContactEvent>,
    /// Body activation changes.
    pub activations: Vec<ActivationEvent>,
    /// Soft body bounding-box overlaps.
    pub soft_body_validations: Vec<SoftBodyValidation>,
    /// Soft body vertex contacts.
    pub soft_body_contacts: Vec<SoftBodyContacts>,
    /// Contact settings a [`ContactListener`] returned that Jolt did not take. Recorded
    /// whenever a listener is set, whatever the [`EventSettings`].
    pub rejected_contact_settings: Vec<ContactSettingsRejection>,
}

impl WorldEvents {
    /// Whether no event of any kind is held.
    pub fn is_empty(&self) -> bool {
        self.contacts.is_empty()
            && self.activations.is_empty()
            && self.soft_body_validations.is_empty()
            && self.soft_body_contacts.is_empty()
            && self.rejected_contact_settings.is_empty()
    }

    fn append(&mut self, mut other: WorldEvents) {
        self.contacts.append(&mut other.contacts);
        self.activations.append(&mut other.activations);
        self.soft_body_validations
            .append(&mut other.soft_body_validations);
        self.soft_body_contacts
            .append(&mut other.soft_body_contacts);
        self.rejected_contact_settings
            .append(&mut other.rejected_contact_settings);
    }

    fn sort_canonically(&mut self) {
        order::sort_contacts(&mut self.contacts);
        order::sort_activations(&mut self.activations);
        order::sort_soft_body_validations(&mut self.soft_body_validations);
        order::sort_soft_body_contacts(&mut self.soft_body_contacts);
        order::sort_rejections(&mut self.rejected_contact_settings);
    }
}

/// The solver settings of `system` that collision estimates use.
///
/// # Safety
/// `system` is a live physics system that no step is updating.
unsafe fn solver_settings(system: *mut JPH_PhysicsSystem) -> SolverSettings {
    // SAFETY: an all-zero `JPH_PhysicsSettings` is valid: integers, floats and `false`.
    let mut settings: JPH_PhysicsSettings = unsafe { mem::zeroed() };
    // SAFETY: the system is live (contract); joltc writes every field of the live local.
    unsafe { JPH_PhysicsSystem_GetPhysicsSettings(system, &mut settings) };
    SolverSettings {
        min_velocity_for_restitution: settings.minVelocityForRestitution,
        num_velocity_steps: settings.numVelocitySteps,
    }
}

/// What the native listeners of one world configuration write to. Native code holds a raw
/// pointer to it while the listeners are attached, so it is shared through an `Arc`, its
/// configuration never changes, and its buffers are behind mutexes.
pub(crate) struct ListenerContext {
    settings: EventSettings,
    listener: Option<Arc<dyn ContactListener>>,
    world: WorldTag,
    /// The world's solver settings when the listeners were configured, for collision estimates.
    /// A setter of the world's physics settings must configure the listeners again.
    solver: SolverSettings,
    /// The events recorded since they were last moved to the world's queue.
    batch: Mutex<WorldEvents>,
    panic: PanicSlot,
    /// The callback that panics on purpose, for the panic tests.
    #[cfg(test)]
    panic_in: std::sync::atomic::AtomicU8,
}

impl ListenerContext {
    fn new(
        settings: EventSettings,
        listener: Option<Arc<dyn ContactListener>>,
        world: WorldTag,
        solver: SolverSettings,
    ) -> Self {
        Self {
            settings,
            listener,
            world,
            solver,
            batch: Mutex::default(),
            panic: PanicSlot::default(),
            #[cfg(test)]
            panic_in: std::sync::atomic::AtomicU8::new(tests::NO_PANIC),
        }
    }

    /// The context behind a listener's `userData`.
    ///
    /// # Safety
    /// `user_data` is the pointer `Listeners::configure` gave a native listener, which is
    /// called only while it is attached, and the world keeps the `Arc` alive while it is.
    unsafe fn from_user_data<'a>(user_data: *mut c_void) -> &'a Self {
        // SAFETY: the caller guarantees the pointer comes from `Arc::as_ptr` of a live context;
        // the context is only ever shared, never borrowed mutably.
        unsafe { &*user_data.cast::<Self>() }
    }

    fn batch(&self) -> MutexGuard<'_, WorldEvents> {
        self.batch.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn take_batch(&self) -> WorldEvents {
        mem::take(&mut *self.batch())
    }

    /// Calls the user listener unless a callback panicked since the world last resumed a panic;
    /// `None` when it did not return.
    fn call_listener<R>(&self, call: impl FnOnce() -> R) -> Option<R> {
        if self.panic.has_panicked() {
            return None;
        }
        match catch_unwind(AssertUnwindSafe(call)) {
            Ok(result) => Some(result),
            Err(payload) => {
                self.panic.record(payload);
                None
            }
        }
    }

    /// Records that the settings a listener returned for the contact `pair` were not applied.
    fn reject(&self, pair: SubShapeIdPair, error: crate::ContactSettingsError) {
        let rejection = ContactSettingsRejection { pair, error };
        self.batch().rejected_contact_settings.push(rejection);
    }

    /// Runs a callback body; a panic is kept for the world to resume and `fallback` returned.
    fn guarded<R>(&self, fallback: R, callback: Callback, f: impl FnOnce() -> R) -> R {
        let run = || {
            #[cfg(test)]
            self.panic_if_asked(callback);
            #[cfg(not(test))]
            let _ = callback;
            f()
        };
        match catch_unwind(AssertUnwindSafe(run)) {
            Ok(result) => result,
            Err(payload) => {
                self.panic.record(payload);
                fallback
            }
        }
    }
}

/// The callbacks, named for the panic tests.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(not(test), allow(dead_code))]
enum Callback {
    ContactValidate = 1,
    ContactAdded,
    ContactPersisted,
    ContactRemoved,
    BodyActivated,
    BodyDeactivated,
    SoftBodyValidate,
    SoftBodyAdded,
}

/// A native contact listener of the extension, owned by the world.
impl JoltObject for JPH_ContactListener2 {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the listener (trait contract) and detached it from its system
        // first, so nothing calls it any more.
        unsafe { JPH_ContactListener2_Destroy(ptr) };
    }
}

/// A native body activation listener, owned by the world.
impl JoltObject for JPH_BodyActivationListener {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: as for the contact listener.
        unsafe { JPH_BodyActivationListener_Destroy(ptr) };
    }
}

/// A native soft body contact listener, owned by the world.
impl JoltObject for JPH_SoftBodyContactListener {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: as for the contact listener.
        unsafe { JPH_SoftBodyContactListener_Destroy(ptr) };
    }
}

static CONTACT_PROCS: JPH_ContactListener_Procs = JPH_ContactListener_Procs {
    OnContactValidate: Some(validate::on_contact_validate),
    OnContactAdded: Some(contact::on_contact_added),
    OnContactPersisted: Some(contact::on_contact_persisted),
    OnContactRemoved: Some(contact::on_contact_removed),
};

static ACTIVATION_PROCS: JPH_BodyActivationListener_Procs = JPH_BodyActivationListener_Procs {
    OnBodyActivated: Some(activation::on_body_activated),
    OnBodyDeactivated: Some(activation::on_body_deactivated),
};

static SOFT_BODY_PROCS: JPH_SoftBodyContactListener_Procs = JPH_SoftBodyContactListener_Procs {
    OnSoftBodyContactValidate: Some(soft_body::on_soft_body_contact_validate),
    OnSoftBodyContactAdded: Some(soft_body::on_soft_body_contact_added),
};

/// Points the three global listener proc tables at oxijolt' callbacks, once per process.
fn install_procs() {
    static INSTALLED: OnceLock<()> = OnceLock::new();
    INSTALLED.get_or_init(|| {
        // SAFETY: the tables are immutable statics, so the pointers stay valid forever.
        // `SetProcs` only stores the pointer in a plain global; `OnceLock` makes that one write
        // happen before every read by a thread that returns from `get_or_init`, and every
        // listener is created after this call on the creating thread, before a step can call
        // it. oxijolt never calls `SetProcs` again, and the raw-API contract of
        // `oxijolt-sys` forbids other code to.
        unsafe {
            JPH_ContactListener2_SetProcs(&CONTACT_PROCS);
            JPH_BodyActivationListener_SetProcs(&ACTIVATION_PROCS);
            JPH_SoftBodyContactListener_SetProcs(&SOFT_BODY_PROCS);
        }
    });
}

/// A world's listeners: the current context, the native listeners attached to the system, and
/// the queue of events not taken yet.
#[derive(Default)]
pub(crate) struct Listeners {
    // The native listeners are destroyed before the context they point to (field order).
    contact: Option<Owned<JPH_ContactListener2>>,
    activation: Option<Owned<JPH_BodyActivationListener>>,
    soft_body: Option<Owned<JPH_SoftBodyContactListener>>,
    context: Option<Arc<ListenerContext>>,
    queue: WorldEvents,
    /// A panic of a callback of a replaced context, not resumed yet.
    pending_panic: Option<Box<dyn Any + Send>>,
}

impl Listeners {
    pub(crate) fn settings(&self) -> EventSettings {
        self.context
            .as_ref()
            .map_or_else(EventSettings::default, |context| context.settings)
    }

    fn listener(&self) -> Option<Arc<dyn ContactListener>> {
        self.context
            .as_ref()
            .and_then(|context| context.listener.clone())
    }

    /// Replaces the listeners of `system` with ones for `settings` and `listener`. Events and a
    /// panic of the old ones are kept.
    ///
    /// # Safety
    /// `system` is the live system these listeners belong to, borrowed mutably by the caller,
    /// so no step runs.
    pub(crate) unsafe fn configure(
        &mut self,
        system: *mut JPH_PhysicsSystem,
        settings: EventSettings,
        listener: Option<Arc<dyn ContactListener>>,
        world: WorldTag,
    ) {
        // SAFETY: as the caller guarantees.
        unsafe { self.detach(system) };
        self.contact = None;
        self.activation = None;
        self.soft_body = None;
        if let Some(old) = self.context.take() {
            self.queue.append(old.take_batch());
            let payload = old.panic.take();
            self.pending_panic = first_payload(self.pending_panic.take(), payload);
        }
        let user = listener.is_some();
        if !settings.needs_any_listener() && !user {
            return;
        }
        install_procs();
        // SAFETY: the system is live (contract).
        let solver = unsafe { solver_settings(system) };
        let context = Arc::new(ListenerContext::new(settings, listener, world, solver));
        let user_data: *mut c_void = Arc::as_ptr(&context).cast_mut().cast();
        // SAFETY: Jolt is initialised (the system exists). joltc stores `user_data` in each new
        // listener; the listeners are destroyed before `context` (field order and `configure`),
        // and the `Arc` kept in `self.context` keeps it alive while they are attached. A null
        // listener means joltc could not allocate it, which leaves its events unrecorded.
        unsafe {
            if settings.needs_contact_listener() || user {
                self.contact = Owned::from_raw(JPH_ContactListener2_Create(user_data));
            }
            if settings.body_activation {
                self.activation = Owned::from_raw(JPH_BodyActivationListener_Create(user_data));
            }
            if settings.needs_soft_body_listener() || user {
                self.soft_body = Owned::from_raw(JPH_SoftBodyContactListener_Create(user_data));
            }
        }
        self.context = Some(context);
        // SAFETY: the system is live and no step runs (contract); the listeners stay alive and
        // attached until `detach`.
        unsafe {
            JPH_PhysicsSystem_SetContactListener2(system, raw(&self.contact));
            JPH_PhysicsSystem_SetBodyActivationListener(system, raw(&self.activation));
            JPH_PhysicsSystem_SetSoftBodyContactListener(system, raw(&self.soft_body));
        }
    }

    /// Detaches every listener from `system`, after which no callback runs.
    ///
    /// # Safety
    /// As for [`configure`](Self::configure).
    pub(crate) unsafe fn detach(&mut self, system: *mut JPH_PhysicsSystem) {
        // SAFETY: the system is live and no step runs (contract); null detaches.
        unsafe {
            JPH_PhysicsSystem_SetContactListener2(system, null_mut());
            JPH_PhysicsSystem_SetBodyActivationListener(system, null_mut());
            JPH_PhysicsSystem_SetSoftBodyContactListener(system, null_mut());
        }
    }

    /// Moves the events recorded between steps to the queue, in the order they came.
    pub(crate) fn begin_step(&mut self) {
        if let Some(context) = &self.context {
            self.queue.append(context.take_batch());
        }
    }

    /// Sorts the events of the step that just ended and appends them to the queue.
    pub(crate) fn finish_step(&mut self) -> FinishedStep {
        let Some(context) = &self.context else {
            return FinishedStep {
                rejected_contact_settings: 0,
                panic: self.pending_panic.take(),
            };
        };
        let mut batch = context.take_batch();
        batch.sort_canonically();
        let rejected = batch.rejected_contact_settings.len();
        self.queue.append(batch);
        FinishedStep {
            rejected_contact_settings: u32::try_from(rejected).unwrap_or(u32::MAX),
            panic: first_payload(self.pending_panic.take(), context.panic.take()),
        }
    }

    /// Takes the queue with the events recorded since the last step, or, when a callback
    /// panicked since then, that panic for the caller to resume, leaving the queue in place.
    fn take_events(&mut self) -> Result<WorldEvents, Box<dyn Any + Send>> {
        let payload = match &self.context {
            Some(context) => {
                self.queue.append(context.take_batch());
                first_payload(self.pending_panic.take(), context.panic.take())
            }
            None => self.pending_panic.take(),
        };
        match payload {
            Some(payload) => Err(payload),
            None => Ok(mem::take(&mut self.queue)),
        }
    }
}

/// What the listeners report about a step that just ended.
pub(crate) struct FinishedStep {
    /// How many contact settings a listener returned that were not applied.
    pub(crate) rejected_contact_settings: u32,
    /// The first panic of a callback since the last step or `take_events`, or of a replaced
    /// configuration.
    pub(crate) panic: Option<Box<dyn Any + Send>>,
}

/// The pointer of an optional native listener, null when there is none.
fn raw<T: JoltObject>(listener: &Option<Owned<T>>) -> *mut T {
    listener.as_ref().map_or(null_mut(), Owned::as_ptr)
}

impl PhysicsWorld {
    /// Chooses which events the world records from now on; see [`EventSettings`]. The
    /// default records nothing and installs no listener in Jolt.
    ///
    /// Events recorded so far stay queued for [`take_events`](Self::take_events). Turning
    /// contacts on while bodies touch reports their next contacts as Persisted or Removed
    /// without an Added, because Jolt reports changes against its contact cache.
    pub fn set_event_settings(&mut self, settings: EventSettings) {
        let listener = self.listeners.listener();
        self.configure_listeners(settings, listener);
    }

    /// Lets `listener` change contact settings from now on, or removes the listener with
    /// `None`; see [`ContactListener`]. The world keeps the `Arc` while the listener is set.
    ///
    /// Events recorded so far stay queued for [`take_events`](Self::take_events).
    pub fn set_contact_listener(&mut self, listener: Option<Arc<dyn ContactListener>>) {
        let settings = self.listeners.settings();
        self.configure_listeners(settings, listener);
    }

    fn configure_listeners(
        &mut self,
        settings: EventSettings,
        listener: Option<Arc<dyn ContactListener>>,
    ) {
        let (system, tag) = (self.system.as_ptr(), self.tag);
        // SAFETY: the system is this world's, borrowed mutably, so no step runs.
        unsafe { self.listeners.configure(system, settings, listener, tag) };
    }

    /// The events the world records; see [`set_event_settings`](Self::set_event_settings).
    pub fn event_settings(&self) -> EventSettings {
        self.listeners.settings()
    }

    /// Takes every event recorded since the last call: those of the steps since then, each
    /// step's in canonical order (see [`WorldEvents`]), and those recorded between steps
    /// (activations when bodies are created, woken or removed) in the order they happened.
    ///
    /// Events queue up until they are taken, so a caller that records events should take them
    /// after every step. [`restore_state`](Self::restore_state) neither clears nor rewinds the
    /// queue.
    ///
    /// # Panics
    /// When a callback panicked outside a step (for example while
    /// [`create_body`](Self::create_body) activated a body), resumes that panic; the events
    /// stay queued for the next call.
    pub fn take_events(&mut self) -> WorldEvents {
        match self.listeners.take_events() {
            Ok(events) => events,
            Err(payload) => resume_unwind(payload),
        }
    }
}

#[cfg(test)]
mod tests;
