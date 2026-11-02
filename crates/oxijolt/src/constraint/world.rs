//! Constraints owned by a [`PhysicsWorld`]: the world's registry, and read and write views.

use std::collections::BTreeSet;
use std::hash::Hash;
use std::ptr::NonNull;

use oxijolt_sys::*;

use super::id::{AnyConstraintId, ConstraintId};
use super::lever::check_spring;
use super::SpringSettings;
use crate::body::with_locked_bodies;
use crate::owned::{JoltObject, Owned};
use crate::{
    AllowedDofs, BodyError, BodyId, ConstraintError, MotionType, PhysicsWorld, RVec3, Vec3,
};

/// The kinds of constraint a world can hold.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum ConstraintType {
    /// [`FixedConstraint`].
    Fixed,
    /// [`PointConstraint`].
    Point,
    /// [`DistanceConstraint`].
    Distance,
    /// [`HingeConstraint`].
    Hinge,
    /// [`SliderConstraint`].
    Slider,
    /// [`ConeConstraint`].
    Cone,
    /// [`SwingTwistConstraint`].
    SwingTwist,
    /// [`SixDofConstraint`].
    SixDof,
    /// [`GearConstraint`].
    Gear,
    /// [`RackAndPinionConstraint`].
    RackAndPinion,
    /// [`PulleyConstraint`].
    Pulley,
    /// [`PathConstraint`].
    Path,
}

/// The traits users can name but not implement or call: the constraint kinds and settings are
/// the crate's own.
pub(crate) mod sealed {
    use super::*;

    /// A constraint kind marker.
    pub trait Kind {
        /// The kind as a value.
        const TYPE: ConstraintType;
        /// Jolt's subtype of the kind's constraint.
        const SUB_TYPE: JPH_ConstraintSubType;
    }

    /// Where a constraint holds one of its bodies, for the lever-arm check of
    /// [`PhysicsWorld::create_constraint`](crate::PhysicsWorld::create_constraint).
    #[derive(Clone, Copy, Debug)]
    pub enum Anchor {
        /// A point in world space, where it is when the constraint is created.
        World(RVec3),
        /// A point relative to the body's centre of mass, in the body's frame.
        CenterOfMass(Vec3),
        /// Any point within `radius` of `offset`, which is relative to body 1's origin in
        /// body 1's frame: a path fixed to body 1, or one point of it.
        OnBody1 {
            /// The point, or the centre of the points, relative to body 1's origin in body 1's
            /// frame.
            offset: Vec3,
            /// How far from `offset` the points lie at most, metres; 0 for one point.
            radius: f64,
        },
        /// A point Jolt places between the two bodies' centres of mass.
        BetweenCentersOfMass,
    }

    /// Settings a world constraint is created from.
    pub trait Settings {
        /// Whether Jolt's solver parts for the constraint need both bodies dynamic: the gear's,
        /// the rack and pinion's and the pulley's read both bodies' motion properties without
        /// checking that the body has any.
        const NEEDS_DYNAMIC_BODIES: bool = false;

        /// What Jolt asserts or silently rewrites, checked before anything is created.
        fn validate(&self) -> Result<(), &'static str>;

        /// Where the constraint holds body 1 and body 2, or `None` for a constraint that holds
        /// no point of its bodies (a gear, a rack and pinion).
        fn anchors(&self) -> Option<[Anchor; 2]> {
            None
        }

        /// Every spring whose frequency-mode coefficients depend on the bodies' effective mass.
        fn springs(&self) -> Vec<SpringSettings>;

        /// The constraints this one references (a gear's hinges, a rack and pinion's hinge and
        /// slider).
        fn references(&self) -> [Option<AnyConstraintId>; 2] {
            [None, None]
        }

        /// Creates the Jolt constraint between the two bodies and returns it holding the one
        /// reference joltc returns, or null when joltc could not allocate it.
        ///
        /// # Safety
        /// Both bodies are distinct, live bodies of one world, locked for writing for the call,
        /// and `validate` returned `Ok`.
        unsafe fn create(
            &self,
            body1: NonNull<JPH_Body>,
            body2: NonNull<JPH_Body>,
        ) -> *mut JPH_Constraint;

        /// Hands the constraints [`references`](Self::references) names to the newly created
        /// `constraint`, after checking that they fit it; the error says how they do not.
        ///
        /// # Safety
        /// `constraint` was just returned by [`create`](Self::create) for `bodies` and is not
        /// added to the system yet. `references` holds, in the order of `references`, the live
        /// constraints of the same world those ids name, of the kinds the ids are typed with.
        unsafe fn attach_references(
            &self,
            _constraint: NonNull<JPH_Constraint>,
            _bodies: [BodyId; 2],
            _references: [Option<ReferencedConstraint>; 2],
        ) -> Result<(), &'static str> {
            Ok(())
        }
    }

    /// A constraint another one references, with its bodies.
    pub struct ReferencedConstraint {
        pub(crate) constraint: NonNull<JPH_Constraint>,
        pub(crate) bodies: [BodyId; 2],
    }

    impl std::fmt::Debug for ReferencedConstraint {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.debug_struct("ReferencedConstraint")
                .field("bodies", &self.bodies)
                .finish_non_exhaustive()
        }
    }
}

/// A kind of world constraint, named by the zero-sized markers such as [`HingeConstraint`]. It
/// selects the methods of [`ConstraintRef`] and [`ConstraintMut`].
pub trait ConstraintKind: sealed::Kind {}

/// Settings a world constraint is created from with [`PhysicsWorld::create_constraint`].
pub trait ConstraintSettings: sealed::Settings {
    /// The kind of constraint these settings create.
    type Kind: ConstraintKind;
}

macro_rules! constraint_kinds {
    ($($(#[$doc:meta])* $name:ident => $kind:ident, $sub_type:ident;)*) => {
        $(
            $(#[$doc])*
            #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
            pub enum $name {}

            impl sealed::Kind for $name {
                const TYPE: ConstraintType = ConstraintType::$kind;
                const SUB_TYPE: JPH_ConstraintSubType = $sub_type;
            }

            impl ConstraintKind for $name {}
        )*
    };
}

constraint_kinds! {
    /// A fixed constraint (Jolt `FixedConstraint`): the bodies move as one.
    FixedConstraint => Fixed, JPH_ConstraintSubType_Fixed;
    /// A point constraint (Jolt `PointConstraint`): a ball joint.
    PointConstraint => Point, JPH_ConstraintSubType_Point;
    /// A distance constraint (Jolt `DistanceConstraint`): a rod or rope between two points.
    DistanceConstraint => Distance, JPH_ConstraintSubType_Distance;
    /// A hinge (Jolt `HingeConstraint`).
    HingeConstraint => Hinge, JPH_ConstraintSubType_Hinge;
    /// A slider (Jolt `SliderConstraint`): translation along one axis.
    SliderConstraint => Slider, JPH_ConstraintSubType_Slider;
    /// A cone constraint (Jolt `ConeConstraint`): a ball joint whose twist axis stays in a cone.
    ConeConstraint => Cone, JPH_ConstraintSubType_Cone;
    /// A swing-twist constraint (Jolt `SwingTwistConstraint`).
    SwingTwistConstraint => SwingTwist, JPH_ConstraintSubType_SwingTwist;
    /// A six-degree-of-freedom constraint (Jolt `SixDOFConstraint`).
    SixDofConstraint => SixDof, JPH_ConstraintSubType_SixDOF;
    /// A gear (Jolt `GearConstraint`): couples the rotation of two bodies.
    GearConstraint => Gear, JPH_ConstraintSubType_Gear;
    /// A rack and pinion (Jolt `RackAndPinionConstraint`): couples a rotation and a translation.
    RackAndPinionConstraint => RackAndPinion, JPH_ConstraintSubType_RackAndPinion;
    /// A pulley (Jolt `PulleyConstraint`): a rope over two fixed points.
    PulleyConstraint => Pulley, JPH_ConstraintSubType_Pulley;
    /// A path constraint (Jolt `PathConstraint`): body 2 moves along a path fixed to body 1.
    PathConstraint => Path, JPH_ConstraintSubType_Path;
}

/// The state of a constraint motor (Jolt `EMotorState`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum MotorState {
    /// The motor is off. The default.
    #[default]
    Off,
    /// The motor drives to a target velocity.
    Velocity,
    /// The motor drives to a target position or angle.
    Position,
}

impl MotorState {
    pub(crate) fn to_jph(self) -> JPH_MotorState {
        match self {
            Self::Off => JPH_MotorState_Off,
            Self::Velocity => JPH_MotorState_Velocity,
            Self::Position => JPH_MotorState_Position,
        }
    }

    pub(crate) fn from_jph(state: JPH_MotorState) -> Self {
        if state == JPH_MotorState_Velocity {
            Self::Velocity
        } else if state == JPH_MotorState_Position {
            Self::Position
        } else {
            Self::Off
        }
    }
}

/// A world constraint, of which the world holds the one reference `JPH_*Constraint_Create`
/// returns (joltc `AddRef`s the new object). `JPH_PhysicsSystem_AddConstraint` adds the system's
/// own reference and `JPH_PhysicsSystem_RemoveConstraint` drops it; a gear or rack and pinion
/// holds a `RefConst` on each constraint it references.
impl JoltObject for JPH_Constraint {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), which `Release` drops; the
        // world removes the constraint from the system first.
        unsafe { JPH_Constraint_Destroy(ptr) };
    }
}

/// One constraint of a world.
pub(crate) struct ConstraintEntry {
    pub(crate) constraint: Owned<JPH_Constraint>,
    pub(crate) kind: ConstraintType,
    pub(crate) bodies: [BodyId; 2],
    /// An upper bound of the effective mass (kg) or inertia (kg·m²) any part of the constraint
    /// sees, from the bodies at creation; frequency-mode springs are checked against it.
    pub(crate) effective_mass_bound: f64,
    /// Raw ids of the constraints this one references.
    pub(crate) references: [Option<u32>; 2],
    /// Raw ids of the constraints that reference this one.
    pub(crate) referenced_by: BTreeSet<u32>,
}

/// Read access to one constraint, borrowed from its world.
///
/// The `total_lambda*` readouts are the impulses Jolt's velocity solver applied in the last
/// collision step the constraint was solved in: N·s for linear parts, N·m·s for angular parts.
/// Divide by that collision step's length (`delta_time / collision_steps` of the call that solved
/// it) for the mean force (N) or torque (N·m) over it. With several collision steps the readout is
/// the last collision step's, not a sum or a peak over the call. Each readout is one solver part's
/// share, not the net force on a body: a weld off the centre of mass reads its torque partly in
/// the position part's lever arm. Each impulse acts on body 2 and its opposite on body 1;
/// couplings and pulleys say on each readout how it maps to their bodies instead. A `Vec3` part
/// that joins two points or two orientations (the position of a fixed, point, hinge, cone or
/// swing-twist constraint; the rotation of a fixed or slider constraint, and of a path in
/// [`ConstrainToPath`](crate::PathRotationConstraint::ConstrainToPath) or `FullyConstrained`
/// mode; six-DOF while all three axes of that kind are fixed, for translation also without a
/// soft limit spring) reads a world-space vector. Every
/// other readout is one value per solver part along that part's axis: the constraint axes for
/// motors, friction, the two-component readouts and six-DOF translation, the line between the
/// points for a distance, and for swing-twist limits an axis Jolt picks each step (see
/// [`total_lambda_limits`](ConstraintRef::total_lambda_limits)). A constraint whose bodies
/// sleep, or that is disabled, keeps the values of the last collision step it was solved in. See
/// [docs/constraints.md#breakable-constraints].
///
/// [docs/constraints.md#breakable-constraints]: https://github.com/pockerhead/oxijolt/blob/main/docs/constraints.md#breakable-constraints
pub struct ConstraintRef<'w, K> {
    id: ConstraintId<K>,
    entry: &'w ConstraintEntry,
}

impl<K: ConstraintKind> std::fmt::Debug for ConstraintRef<'_, K> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ConstraintRef")
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}

impl<K: ConstraintKind> ConstraintRef<'_, K> {
    /// The constraint as the joltc handle `T` of its kind, which is the same object
    /// (joltc casts the most derived object).
    pub(crate) fn ptr<T>(&self) -> *mut T {
        self.entry.constraint.as_ptr().cast()
    }

    /// The constraint's id.
    pub fn id(&self) -> ConstraintId<K> {
        self.id
    }

    /// Its two bodies, body 1 and body 2.
    pub fn bodies(&self) -> [BodyId; 2] {
        self.entry.bodies
    }

    /// The constraint this one references at `index` (0 or 1), typed as `R`, the kind the
    /// typed settings that created it require there.
    pub(super) fn reference<R: ConstraintKind>(&self, index: usize) -> Option<ConstraintId<R>> {
        let world = AnyConstraintId::from(self.id).world;
        self.entry.references[index].map(|raw| ConstraintId::new(raw, world))
    }

    /// Whether the constraint is enabled.
    pub fn is_enabled(&self) -> bool {
        // SAFETY: the world borrowed here owns the constraint; `GetEnabled` reads a member, and
        // writes need `&mut PhysicsWorld`.
        unsafe { JPH_Constraint_GetEnabled(self.ptr()) }
    }

    /// Whether Jolt simulates the constraint: enabled and with at least one awake body that can
    /// move (Jolt `IsActive`).
    pub fn is_active(&self) -> bool {
        // SAFETY: as in `is_enabled`; `IsActive` reads the constraint's flag and its bodies'
        // motion and activation state, which Jolt writes only in `step` and the body and
        // constraint setters, all behind `&mut PhysicsWorld`.
        unsafe { JPH_Constraint_IsActive(self.ptr()) }
    }
}

/// Write access to one constraint, borrowed mutably from its world.
///
/// Read the constraint through [`PhysicsWorld::constraint`] once this borrow ends.
pub struct ConstraintMut<'w, K> {
    world: &'w mut PhysicsWorld,
    id: ConstraintId<K>,
}

impl<K: ConstraintKind> std::fmt::Debug for ConstraintMut<'_, K> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ConstraintMut")
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}

impl<K: ConstraintKind> ConstraintMut<'_, K> {
    pub(crate) fn entry(&self) -> &ConstraintEntry {
        self.world
            .constraints
            .get(&self.id.raw)
            .unwrap_or_else(|| unreachable!("checked when the view was made"))
    }

    /// The constraint as the joltc handle `T` of its kind.
    pub(crate) fn ptr<T>(&self) -> *mut T {
        self.entry().constraint.as_ptr().cast()
    }

    /// The constraint's id.
    pub fn id(&self) -> ConstraintId<K> {
        self.id
    }

    /// Its two bodies, body 1 and body 2.
    pub fn bodies(&self) -> [BodyId; 2] {
        self.entry().bodies
    }

    /// Whether the constraint is enabled.
    pub fn is_enabled(&self) -> bool {
        // SAFETY: the world is borrowed through this view and owns the constraint; the getter
        // reads a member.
        unsafe { JPH_Constraint_GetEnabled(self.ptr()) }
    }

    /// Whether Jolt simulates the constraint (Jolt `IsActive`).
    pub fn is_active(&self) -> bool {
        // SAFETY: as in `is_enabled`; the getter reads the constraint and its bodies' state.
        unsafe { JPH_Constraint_IsActive(self.ptr()) }
    }

    /// Enables or disables the constraint and wakes its bodies that can move, so a sleeping body
    /// starts or stops following the constraint at the next step. A disabled constraint keeps
    /// the readouts of the last collision step it was solved in.
    pub fn set_enabled(&mut self, enabled: bool) {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. The setter writes a member.
        unsafe { JPH_Constraint_SetEnabled(self.ptr(), enabled) };
        self.wake_bodies();
    }

    /// Wakes the constraint's bodies that can move, in body id order.
    pub(crate) fn wake_bodies(&mut self) {
        let bodies = self.entry().bodies;
        self.world.wake_constraint_bodies(bodies);
    }

    /// The bound springs set on this constraint are checked against.
    pub(crate) fn effective_mass_bound(&self) -> f64 {
        self.entry().effective_mass_bound
    }
}

impl PhysicsWorld {
    /// Creates a constraint between `body1` and `body2` and returns its typed id.
    ///
    /// Frames given in [`ConstraintSpace::WorldSpace`] are world space at the time the
    /// constraint is created; Jolt turns them into each body's own frame then. Both bodies must
    /// be bodies of this world, different, neither the inner body of a character nor a part of
    /// a ragdoll, and created with all six degrees of freedom ([`ConstraintError::Body`]). Except for a gear, a rack and pinion and a
    /// pulley, one of them may be static or kinematic, which anchors the constraint to the world
    /// or to the kinematic body. Those three need two dynamic bodies
    /// ([`ConstraintError::NotDynamic`]): Jolt's solver parts for them read both bodies' motion
    /// properties without checking, so a static body breaks them and a kinematic one gets
    /// pushed. A pulley's fixed points already anchor its rope. While the constraint exists,
    /// [`BodyMut::set_motion_type`](crate::BodyMut::set_motion_type) refuses its bodies, so
    /// checking at creation is enough.
    ///
    /// Jolt still lets the two bodies collide with each other where their shapes touch. While
    /// the constraint exists, [`remove_body`](Self::remove_body) and the motion type changes of
    /// [`BodyMut`](crate::BodyMut) refuse its bodies ([`BodyError::UsedByConstraint`]), and so
    /// do shape changes of a kinematic or dynamic body. A static body may change shape: the
    /// constraint keeps its attachment points in world space (see
    /// [`BodyMut::set_shape`](crate::BodyMut::set_shape)). Points given in
    /// [`ConstraintSpace::LocalToBodyCom`] are relative to each body's centre of mass at
    /// creation.
    ///
    /// Each point where the constraint holds a dynamic body must have a lever-arm ratio of at
    /// most [`limits::MAX_LEVER_ARM_RATIO`], its distance from the body's centre of mass
    /// measured against the body's size. The check uses the bodies' poses now and covers
    /// every point of a path and, for an automatic point, the point Jolt picks: between
    /// the centres of mass, weighted by inverse mass towards the lighter body.
    ///
    /// The new constraint wakes its bodies that can move, as its setters do, so a sleeping body
    /// starts following it at the next step.
    ///
    /// Frequency-mode springs ([`SpringSettings::FrequencyAndDamping`]) become a stiffness and
    /// damping that grow with the bodies' effective mass, so they are checked against an upper
    /// bound of it, taken from the dynamic bodies' mass and inertia now: `max(mass, largest
    /// principal moment)` of each. While the constraint exists, the motion type changes of
    /// [`BodyMut`](crate::BodyMut) refuse its bodies, and so do the shape changes of its
    /// dynamic bodies, so the bound holds for the constraint's life.
    ///
    /// Fails with [`ConstraintError::InvalidValue`] when a setting or a lever-arm ratio is out of
    /// range, with [`ConstraintError::NotDynamic`] as above, with
    /// [`ConstraintError::Body`]`(`[`BodyError::NotRigidBody`]`)` for a soft body (Jolt's
    /// constraints cannot operate on soft bodies; pin a vertex and move it instead), with
    /// [`ConstraintError::NotFound`] or [`ConstraintError::WrongWorld`] for a referenced constraint
    /// that is not in this world, and with [`ConstraintError::TooManyConstraints`] when the world
    /// has run out of ids. Nothing changes on failure.
    ///
    /// # Example
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let post = Shape::new_box(Vec3::new(0.1, 1.0, 0.1))?;
    /// let frame = world.create_body(&post, &BodySettings::new_static())?;
    /// let panel = Shape::new_box(Vec3::new(0.5, 1.0, 0.05))?;
    /// // Jolt lets the two bodies of a constraint collide, so the door leaves a gap to the post.
    /// let door = world.create_body(
    ///     &panel,
    ///     &BodySettings::new_dynamic().position(RVec3::new(0.7, 0.0, 0.0)),
    /// )?;
    ///
    /// // A vertical hinge at the door's edge, driven to a quarter turn.
    /// let up = Vec3::new(0.0, 1.0, 0.0);
    /// let side = Vec3::new(1.0, 0.0, 0.0);
    /// let hinge = world.create_constraint(
    ///     frame,
    ///     door,
    ///     &HingeConstraintSettings::new(RVec3::new(0.15, 0.0, 0.0), up, side),
    /// )?;
    /// let mut motor = world.constraint_mut(hinge)?;
    /// motor.set_motor_state(MotorState::Position);
    /// motor.set_target_angle(std::f32::consts::FRAC_PI_2)?;
    /// for _ in 0..120 {
    ///     assert!(world.step(1.0 / 60.0)?.is_complete());
    /// }
    /// let angle = world.constraint(hinge)?.current_angle();
    /// assert!((angle - std::f32::consts::FRAC_PI_2).abs() < 0.05);
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// [`ConstraintSpace::WorldSpace`]: crate::ConstraintSpace::WorldSpace
    /// [`ConstraintSpace::LocalToBodyCom`]: crate::ConstraintSpace::LocalToBodyCom
    /// [`limits::MAX_LEVER_ARM_RATIO`]: crate::limits::MAX_LEVER_ARM_RATIO
    pub fn create_constraint<S: ConstraintSettings>(
        &mut self,
        body1: BodyId,
        body2: BodyId,
        settings: &S,
    ) -> Result<ConstraintId<S::Kind>, ConstraintError> {
        sealed::Settings::validate(settings).map_err(ConstraintError::InvalidValue)?;
        for body in [body1, body2] {
            self.check(body).map_err(ConstraintError::Body)?;
        }
        if body1 == body2 {
            return Err(ConstraintError::InvalidValue(
                "a constraint joins two different bodies",
            ));
        }
        for body in [body1, body2] {
            // The character and the ragdoll destroy these bodies themselves, behind the body
            // guard, which the constraint would outlive.
            if self.is_inner_body(body) {
                return Err(ConstraintError::Body(BodyError::OwnedByCharacter(body)));
            }
            if self.is_ragdoll_body(body) {
                return Err(ConstraintError::Body(BodyError::OwnedByRagdoll(body)));
            }
            // Jolt's constraints cannot operate on soft bodies (`Docs/Architecture.md:462`).
            if self
                .body(body)
                .map_err(ConstraintError::Body)?
                .is_soft_body()
            {
                return Err(ConstraintError::Body(BodyError::NotRigidBody(body)));
            }
            // The lever-arm and spring checks below assume unmasked inverse mass and inertia.
            let allowed_dofs = self
                .body(body)
                .map_err(ConstraintError::Body)?
                .allowed_dofs();
            if allowed_dofs != AllowedDofs::ALL {
                return Err(ConstraintError::Body(BodyError::RestrictedDofs(body)));
            }
            if S::NEEDS_DYNAMIC_BODIES
                && self
                    .body(body)
                    .map_err(ConstraintError::Body)?
                    .motion_type()
                    != MotionType::Dynamic
            {
                return Err(ConstraintError::NotDynamic(body));
            }
        }
        if let Some(anchors) = settings.anchors() {
            self.check_lever_arms([body1, body2], anchors)?;
        }
        let references = settings.references();
        for reference in references.into_iter().flatten() {
            self.constraint_entry(reference)?;
        }
        let raw = self.next_constraint_id;
        if raw == u32::MAX {
            return Err(ConstraintError::TooManyConstraints);
        }
        let effective_mass_bound = self.effective_mass_bound([body1, body2]);
        for spring in settings.springs() {
            check_spring(spring, effective_mass_bound)?;
        }

        let constraint =
            with_locked_bodies(self.body_lock_interface, [body1, body2], |first, second| {
                // SAFETY: both bodies are distinct live bodies of this world (checked above),
                // locked for writing for the call, and the settings are validated. Jolt bodies
                // stay at their address until destroyed, and `remove_body` refuses them while
                // the constraint exists. The handle takes over the one reference joltc returns.
                unsafe { Owned::from_raw(sealed::Settings::create(settings, first, second)) }
            })
            .unwrap_or_else(|| unreachable!("`check` found both bodies and `&mut self` keeps them"))
            // A failed native allocation is not a recoverable error here, as everywhere joltc `new`s.
            .unwrap_or_else(|| unreachable!("joltc `new`s the constraint"));
        debug_assert_eq!(
            // SAFETY: the constraint is live; the getter is a virtual call that only returns the type.
            unsafe { JPH_Constraint_GetSubType(constraint.as_ptr()) },
            <S::Kind as sealed::Kind>::SUB_TYPE
        );

        let referenced = references.map(|reference| {
            reference.map(|id| {
                let entry = &self.constraints[&id.raw];
                sealed::ReferencedConstraint {
                    constraint: entry.constraint.as_non_null(),
                    bodies: entry.bodies,
                }
            })
        });
        // SAFETY: `constraint` was just created for these bodies and is not added; `referenced`
        // holds the live constraints `references` names, which `constraint_entry` found in this
        // world, of the kinds the typed settings require.
        let attached = unsafe {
            sealed::Settings::attach_references(
                settings,
                constraint.as_non_null(),
                [body1, body2],
                referenced,
            )
        };
        // On failure, dropping `constraint` releases the only reference to it.
        attached.map_err(ConstraintError::InvalidValue)?;

        self.note_structure_change();
        // SAFETY: the system and the constraint are live, the system is borrowed mutably and no
        // step runs. The system takes its own reference; `remove_constraint` and
        // `remove_all_constraints` remove it before the world releases its own.
        unsafe { JPH_PhysicsSystem_AddConstraint(self.system.as_ptr(), constraint.as_ptr()) };
        let references = references.map(|reference| reference.map(|id| id.raw));
        for reference in references.into_iter().flatten() {
            if let Some(entry) = self.constraints.get_mut(&reference) {
                entry.referenced_by.insert(raw);
            }
        }
        for body in [body1, body2] {
            self.constraint_bodies
                .entry(body.to_raw())
                .or_default()
                .insert(raw);
        }
        self.constraints.insert(
            raw,
            ConstraintEntry {
                constraint,
                kind: <S::Kind as sealed::Kind>::TYPE,
                bodies: [body1, body2],
                effective_mass_bound,
                references,
                referenced_by: BTreeSet::new(),
            },
        );
        self.next_constraint_id += 1;
        self.wake_constraint_bodies([body1, body2]);
        Ok(ConstraintId::new(raw, self.tag))
    }

    /// The entry of `id`, if it names a constraint of this world.
    pub(crate) fn constraint_entry(
        &self,
        id: impl Into<AnyConstraintId>,
    ) -> Result<&ConstraintEntry, ConstraintError> {
        let id = id.into();
        if id.world != self.tag {
            return Err(ConstraintError::WrongWorld(id));
        }
        self.constraints
            .get(&id.raw)
            .ok_or(ConstraintError::NotFound(id))
    }

    /// Removes a constraint and wakes its bodies that can move, in body id order, so a body the
    /// constraint held does not stay asleep where it was.
    ///
    /// A hinge or slider that a gear or rack and pinion references cannot be removed while the
    /// coupling exists ([`ConstraintError::UsedByConstraint`]); remove the coupling first.
    pub fn remove_constraint(
        &mut self,
        id: impl Into<AnyConstraintId>,
    ) -> Result<(), ConstraintError> {
        let id = id.into();
        let entry = self.constraint_entry(id)?;
        if let Some(&dependent) = entry.referenced_by.first() {
            let dependent = AnyConstraintId {
                raw: dependent,
                world: self.tag,
                kind: self.constraints[&dependent].kind,
            };
            return Err(ConstraintError::UsedByConstraint(dependent));
        }
        self.note_structure_change();
        let entry = self
            .constraints
            .remove(&id.raw)
            .unwrap_or_else(|| unreachable!("checked above"));
        let bodies = entry.bodies;
        self.unregister_constraint(id.raw, entry);
        self.wake_constraint_bodies(bodies);
        Ok(())
    }

    /// Takes the constraint out of the system and of the world's indexes, then releases the
    /// world's reference.
    fn unregister_constraint(&mut self, raw: u32, entry: ConstraintEntry) {
        // SAFETY: the system and the constraint are live, the system is borrowed mutably and no
        // step runs; `create_constraint` added the constraint.
        unsafe {
            JPH_PhysicsSystem_RemoveConstraint(self.system.as_ptr(), entry.constraint.as_ptr())
        };
        for body in entry.bodies {
            if let Some(ids) = self.constraint_bodies.get_mut(&body.to_raw()) {
                ids.remove(&raw);
                if ids.is_empty() {
                    self.constraint_bodies.remove(&body.to_raw());
                }
            }
        }
        for reference in entry.references.into_iter().flatten() {
            if let Some(referenced) = self.constraints.get_mut(&reference) {
                referenced.referenced_by.remove(&raw);
            }
        }
        drop(entry);
    }

    /// Removes every constraint, newest first. Runs when the world is dropped, before anything
    /// else is released. A coupling is always newer than the constraints it references, so
    /// dependents go first; the coupling's own references would keep a referenced hinge alive
    /// in any order.
    pub(crate) fn remove_all_constraints(&mut self) {
        while let Some((raw, entry)) = self.constraints.pop_last() {
            self.unregister_constraint(raw, entry);
        }
    }

    /// Tells every constraint that uses `body`, in id order and disabled ones too, that the
    /// centre of mass of its shape moved by `delta` in the body's space (new minus old; Jolt
    /// `Constraint::NotifyShapeChanged`), so that each keeps its attachment points in world
    /// space. Jolt does not do this itself when a shape changes.
    pub(crate) fn notify_shape_changed(&mut self, body: BodyId, delta: Vec3) {
        let Some(ids) = self.constraint_bodies.get(&body.to_raw()) else {
            return;
        };
        let mut delta = delta.to_jph();
        for raw in ids {
            let constraint = self.constraints[raw].constraint.as_ptr();
            // SAFETY: the world owns the live constraint and is borrowed mutably, and no step
            // runs; `body` is one of the constraint's two bodies (`constraint_bodies`), and the
            // call only rewrites that side's local frame. joltc copies `delta`, a live local.
            unsafe { JPH_Constraint_NotifyShapeChanged(constraint, body.to_raw(), &mut delta) };
        }
    }

    /// Wakes `bodies` that can move, in body id order; Jolt skips static bodies.
    pub(crate) fn wake_constraint_bodies(&mut self, bodies: [BodyId; 2]) {
        let mut ids = bodies.map(BodyId::to_raw);
        ids.sort_unstable();
        // SAFETY: the body interface belongs to this live world, borrowed mutably; both ids name
        // bodies of this world, which `remove_body` keeps while a constraint uses them, and
        // `ids` lives for the call. This thread holds no body lock.
        unsafe { JPH_BodyInterface_ActivateBodies(self.body_interface.as_ptr(), ids.as_ptr(), 2) };
    }

    /// Read access to a constraint.
    pub fn constraint<K: ConstraintKind>(
        &self,
        id: ConstraintId<K>,
    ) -> Result<ConstraintRef<'_, K>, ConstraintError> {
        let entry = self.constraint_entry(id)?;
        debug_assert_eq!(entry.kind, K::TYPE, "ids are typed by their kind");
        Ok(ConstraintRef { id, entry })
    }

    /// Write access to a constraint.
    pub fn constraint_mut<K: ConstraintKind>(
        &mut self,
        id: ConstraintId<K>,
    ) -> Result<ConstraintMut<'_, K>, ConstraintError> {
        let entry = self.constraint_entry(id)?;
        debug_assert_eq!(entry.kind, K::TYPE, "ids are typed by their kind");
        Ok(ConstraintMut { world: self, id })
    }

    /// The ids of the world's constraints, in id order (creation order).
    pub fn constraint_ids(&self) -> impl Iterator<Item = AnyConstraintId> + '_ {
        self.constraints
            .iter()
            .map(|(&raw, entry)| AnyConstraintId {
                raw,
                world: self.tag,
                kind: entry.kind,
            })
    }

    /// The ids of the constraints that use `body`, in id order.
    pub fn constraints_of_body(&self, body: BodyId) -> impl Iterator<Item = AnyConstraintId> + '_ {
        let raws = if body.world == self.tag {
            self.constraint_bodies.get(&body.to_raw())
        } else {
            None
        };
        raws.into_iter().flatten().map(|&raw| AnyConstraintId {
            raw,
            world: self.tag,
            kind: self.constraints[&raw].kind,
        })
    }

    /// Number of constraints in the world.
    pub fn constraint_count(&self) -> u32 {
        // Every constraint has its own `u32` id, so the count fits.
        self.constraints.len() as u32
    }

    /// Whether `id` names a constraint that is in this world now.
    pub fn contains_constraint(&self, id: impl Into<AnyConstraintId>) -> bool {
        self.constraint_entry(id).is_ok()
    }

    /// Whether a constraint of this world uses `body`.
    pub(crate) fn is_constraint_body(&self, body: BodyId) -> bool {
        body.world == self.tag && self.constraint_bodies.contains_key(&body.to_raw())
    }
}
