//! The tire max impulse callback of wheeled vehicles and motorcycles (Jolt
//! `WheeledVehicleController::SetTireMaxImpulseCallback`).

use std::any::Any;
use std::ffi::c_void;
use std::panic::{catch_unwind, AssertUnwindSafe};

use oxijolt_sys::{
    JPH_WheeledVehicleController, JPH_WheeledVehicleController_ResetTireMaxImpulseCallback,
    JPH_WheeledVehicleController_SetTireMaxImpulseCallback,
};

use super::{VehicleMut, VehicleRef, WheeledControllerKind};
use crate::limits;
use crate::panic_slot::PanicSlot;

/// What Jolt hands a tire max impulse callback for one wheel with ground contact (see
/// [`VehicleMut::set_tire_max_impulse_callback`]).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct TireMaxImpulseInput {
    /// Index of the wheel in the vehicle's wheel list.
    pub wheel: u32,
    /// Impulse of the suspension in this collision step, N·s.
    pub suspension_impulse: f32,
    /// Friction coefficient in the wheel's forward direction: the wheel's longitudinal friction
    /// curve at [`longitudinal_slip`](Self::longitudinal_slip), combined with the ground's
    /// friction.
    pub longitudinal_friction: f32,
    /// Friction coefficient in the wheel's sideways direction: the wheel's lateral friction
    /// curve at [`lateral_slip`](Self::lateral_slip), combined with the ground's friction.
    pub lateral_friction: f32,
    /// `|(ω r − v) / v|`, with `ω r` the speed of the wheel's rim and `v` the forward speed of
    /// the ground relative to the wheel, at least 0.001 m/s in magnitude. 0 is rolling without
    /// slip.
    pub longitudinal_slip: f32,
    /// Angle between the wheel's forward direction and the ground velocity relative to the
    /// wheel, radians in `[0, π/2]`; 0 below a relative speed of 0.001 m/s.
    pub lateral_slip: f32,
    /// Length of the collision step, s.
    pub delta_time: f32,
}

impl TireMaxImpulseInput {
    /// Jolt's own friction coefficients, which a vehicle without a callback uses:
    /// [`longitudinal_friction`](Self::longitudinal_friction) and
    /// [`lateral_friction`](Self::lateral_friction).
    pub fn default_friction(&self) -> TireFriction {
        TireFriction {
            longitudinal: self.longitudinal_friction,
            lateral: self.lateral_friction,
        }
    }

    /// Jolt's caps on the tire impulses for `friction`: each coefficient times the suspension
    /// impulse.
    fn max_impulses(&self, friction: TireFriction) -> [f32; 2] {
        [
            friction.longitudinal * self.suspension_impulse,
            friction.lateral * self.suspension_impulse,
        ]
    }
}

/// The friction coefficients a tire max impulse callback returns for one wheel. Jolt caps the
/// impulse the tire may apply on the ground in this collision step at each coefficient times the
/// [suspension impulse](TireMaxImpulseInput::suspension_impulse), as it does with its own
/// combined friction; the impulses it applies can be lower. Each coefficient is clamped to
/// `0..=`[`limits::MAX_FRICTION`], the bound of a body's friction, and NaN counts as 0.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TireFriction {
    /// Friction coefficient along the wheel's forward direction, drive and brakes.
    pub longitudinal: f32,
    /// Friction coefficient along the wheel's sideways direction.
    pub lateral: f32,
}

impl TireFriction {
    /// The coefficients clamped to `0..=MAX_FRICTION`, NaN replaced by 0.
    fn clamped(self) -> Self {
        // `f32::clamp` keeps NaN.
        let clamp = |friction: f32| {
            if friction.is_nan() {
                0.0
            } else {
                friction.clamp(0.0, limits::MAX_FRICTION)
            }
        };
        Self {
            longitudinal: clamp(self.longitudinal),
            lateral: clamp(self.lateral),
        }
    }
}

type TireFrictionFn = dyn Fn(&TireMaxImpulseInput) -> TireFriction + Send + Sync;

/// A vehicle's tire max impulse callback, whose address is the `userData` Jolt passes to
/// [`tire_max_impulse`].
pub(super) struct TireCallback {
    callback: Box<TireFrictionFn>,
    panic: PanicSlot,
}

impl TireCallback {
    /// The first panic of the callback since the last call, after which it runs again.
    pub(super) fn take_panic(&self) -> Option<Box<dyn Any + Send>> {
        self.panic.take()
    }
}

/// The `JPH_TireMaxImpulseCallback` of every vehicle with a Rust callback. Writes both caps on
/// every path: the callback's coefficients, clamped, times the suspension impulse, or Jolt's
/// default once the callback panicked in this step. A panic is kept for the world to resume and
/// never unwinds into Jolt.
///
/// # Safety
/// `user_data` is the address of a [`TireCallback`] that lives for the call. The out pointers
/// point to Jolt's writable locals of the call (`WheeledVehicleController.cpp:670-676`).
#[allow(clippy::too_many_arguments)]
unsafe extern "C" fn tire_max_impulse(
    user_data: *mut c_void,
    wheel: u32,
    out_longitudinal: *mut f32,
    out_lateral: *mut f32,
    suspension_impulse: f32,
    longitudinal_friction: f32,
    lateral_friction: f32,
    longitudinal_slip: f32,
    lateral_slip: f32,
    delta_time: f32,
) {
    // SAFETY: the address of a live `TireCallback` (contract), which is only ever shared.
    let tire = unsafe { &*user_data.cast::<TireCallback>() };
    let input = TireMaxImpulseInput {
        wheel,
        suspension_impulse,
        longitudinal_friction,
        lateral_friction,
        longitudinal_slip,
        lateral_slip,
        delta_time,
    };
    let friction = if tire.panic.has_panicked() {
        input.default_friction()
    } else {
        match catch_unwind(AssertUnwindSafe(|| (tire.callback)(&input))) {
            Ok(friction) => friction.clamped(),
            Err(payload) => {
                tire.panic.record(payload);
                input.default_friction()
            }
        }
    };
    let [longitudinal, lateral] = input.max_impulses(friction);
    // SAFETY: both pointers are Jolt's writable locals of this call (contract).
    unsafe {
        out_longitudinal.write(longitudinal);
        out_lateral.write(lateral);
    }
}

impl<K: WheeledControllerKind> VehicleRef<'_, K> {
    /// Whether the vehicle has a tire max impulse callback (see
    /// [`VehicleMut::set_tire_max_impulse_callback`]).
    pub fn has_tire_max_impulse_callback(&self) -> bool {
        self.entry.tire_callback.is_some()
    }
}

impl<K: WheeledControllerKind> VehicleMut<'_, K> {
    fn wheeled_controller(&mut self) -> *mut JPH_WheeledVehicleController {
        // A wheeled vehicle's controller is a `WheeledVehicleController` and a motorcycle's a
        // `MotorcycleController` derived from it (`K: WheeledControllerKind`), both with single
        // inheritance from `VehicleController`.
        self.controller().cast()
    }

    /// Replaces the friction coefficients that cap the vehicle's tire impulses (Jolt
    /// `WheeledVehicleController::SetTireMaxImpulseCallback`); without a callback they are
    /// [`TireMaxImpulseInput::default_friction`], the wheel's friction curves combined with the
    /// ground's friction. A callback replaces the one set before.
    ///
    /// Jolt caps each tire impulse at a coefficient times the suspension impulse with or
    /// without a callback, and the callback's coefficients ([`TireFriction`]) are clamped to
    /// [`limits::MAX_FRICTION`]. A callback therefore gives a wheel at most the grip of a
    /// friction curve of `MAX_FRICTION` on ground of `MAX_FRICTION`
    /// ([docs/limits.md#tire-friction]).
    ///
    /// Jolt calls `callback` for each wheel with ground contact in every velocity iteration the
    /// solver runs, in every collision step. The number of calls is not fixed: the solver stops
    /// iterating early when no impulse changes. The calls run on the solver's threads,
    /// concurrently with other vehicles' callbacks, never with another call of the same
    /// vehicle. For results that do not depend on the thread count, the result must depend
    /// only on the input and on data that does not change during the step.
    ///
    /// A panic in `callback` makes the vehicle use Jolt's default coefficients for the rest of
    /// that step, and [`step`](crate::PhysicsWorld::step) resumes the panic afterwards. A panic
    /// that aborts the process is not caught; on wasm32 every panic does
    /// ([wasm32](crate#wasm32-unknown-unknown)).
    ///
    /// The callback and what it captures are not part of [`WorldState`](crate::WorldState):
    /// [`restore_state`](crate::PhysicsWorld::restore_state) keeps the callback and does not
    /// rewind its captured state. [`rebase`](crate::PhysicsWorld::rebase) keeps it too, since
    /// its input does not depend on the frame. The callback is dropped with the vehicle.
    ///
    /// # Example
    /// A tire with 80 % of Jolt's grip sideways:
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let floor = Shape::new_box(Vec3::new(50.0, 1.0, 50.0))?;
    /// world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    /// let hull = Shape::new_box(Vec3::new(0.9, 0.3, 2.0))?;
    /// let chassis = world.create_body(
    ///     &hull,
    ///     &BodySettings::new_dynamic().position(RVec3::new(0.0, 1.0, 0.0)).mass(1500.0),
    /// )?;
    /// let wheel = |x: f32, z: f32| WheelSettings::new(Vec3::new(x, -0.1, z)).radius(0.35);
    /// let settings = WheeledVehicleSettings::new(
    ///     vec![wheel(0.9, 1.4), wheel(-0.9, 1.4), wheel(0.9, -1.4), wheel(-0.9, -1.4)],
    ///     vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
    ///     VehicleCollisionTester::ray(ObjectLayer::MOVING),
    /// );
    /// let car = world.create_wheeled_vehicle(chassis, &settings)?;
    ///
    /// world.vehicle_mut(car)?.set_tire_max_impulse_callback(|input| {
    ///     let default = input.default_friction();
    ///     TireFriction { lateral: 0.8 * default.lateral, ..default }
    /// });
    /// assert!(world.vehicle(car)?.has_tire_max_impulse_callback());
    /// assert!(world.step(1.0 / 60.0)?.is_complete());
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// [docs/limits.md#tire-friction]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#tire-friction
    pub fn set_tire_max_impulse_callback(
        &mut self,
        callback: impl Fn(&TireMaxImpulseInput) -> TireFriction + Send + Sync + 'static,
    ) {
        let tire = Box::new(TireCallback {
            callback: Box::new(callback),
            panic: PanicSlot::default(),
        });
        // The replaced callback drops only after Jolt holds the new one.
        let replaced = self.entry.tire_callback.take();
        let tire: &TireCallback = self.entry.tire_callback.insert(tire);
        let user_data = std::ptr::from_ref(tire).cast_mut().cast::<c_void>();
        let controller = self.wheeled_controller();
        // SAFETY: the controller is this vehicle's (see `wheeled_controller`); the world is
        // borrowed mutably through this view, so no step runs. joltc copies the function and
        // `user_data` into the controller. `user_data` is the heap address of the entry's
        // `TireCallback`, which stays in place until the entry replaces or clears it (both
        // after installing another function) or drops it after its constraint is released.
        unsafe {
            JPH_WheeledVehicleController_SetTireMaxImpulseCallback(
                controller,
                Some(tire_max_impulse),
                user_data,
            )
        };
        drop(replaced);
    }

    /// Removes the tire max impulse callback, so that the vehicle uses Jolt's default
    /// coefficients ([`TireMaxImpulseInput::default_friction`]) from the next step on, and drops
    /// it. Does nothing without one.
    pub fn clear_tire_max_impulse_callback(&mut self) {
        if self.entry.tire_callback.is_none() {
            return;
        }
        let controller = self.wheeled_controller();
        // SAFETY: as in `set_tire_max_impulse_callback`; Jolt's default holds no pointer to the
        // callback dropped below.
        unsafe { JPH_WheeledVehicleController_ResetTireMaxImpulseCallback(controller) };
        self.entry.tire_callback = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coefficients_are_clamped_to_the_friction_bounds() {
        let max = limits::MAX_FRICTION;
        let cases = [
            (f32::NAN, 0.0),
            (f32::NEG_INFINITY, 0.0),
            (-1.0, 0.0),
            (0.5, 0.5),
            (max, max),
            (max.next_up(), max),
            (f32::MAX, max),
            (f32::INFINITY, max),
        ];
        for (value, expected) in cases {
            let longitudinal = TireFriction {
                longitudinal: value,
                lateral: 0.5,
            };
            let lateral = TireFriction {
                longitudinal: 0.5,
                lateral: value,
            };
            assert_eq!(
                longitudinal.clamped().longitudinal.to_bits(),
                expected.to_bits()
            );
            assert_eq!(lateral.clamped().lateral.to_bits(), expected.to_bits());
        }
    }
}
