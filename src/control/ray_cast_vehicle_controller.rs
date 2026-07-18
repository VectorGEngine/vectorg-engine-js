use crate::dynamics::RawRigidBodySet;
use crate::geometry::RawColliderSet;
use crate::math::RawVector;
use crate::pipeline::RawQueryPipeline;
use crate::utils::{self, FlatHandle};
use rapier::control::{
    DynamicRayCastVehicleController, VehicleControllerConfig, VehicleInput, WheelAxle, WheelRole,
    WheelTuning,
};
use rapier::math::Real;
use rapier::pipeline::{QueryFilter, QueryFilterFlags};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawVehicleControllerConfig {
    config: VehicleControllerConfig,
}

#[wasm_bindgen]
impl RawVehicleControllerConfig {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            config: VehicleControllerConfig::default(),
        }
    }

    pub fn set_engine(
        &mut self,
        horsepower: Real,
        idle_rpm: Real,
        max_rpm: Real,
        rev_limit_rpm: Real,
        inertia: Real,
        friction_torque: Option<Real>,
        engine_braking: Real,
        drivetrain_efficiency: Real,
        force_scale: Real,
        gear_force_exponent: Real,
    ) {
        let engine = &mut self.config.engine;
        engine.horsepower = horsepower;
        engine.idle_rpm = idle_rpm;
        engine.max_rpm = max_rpm;
        engine.rev_limit_rpm = rev_limit_rpm;
        engine.inertia = inertia;
        engine.friction_torque = friction_torque;
        engine.engine_braking = engine_braking;
        engine.drivetrain_efficiency = drivetrain_efficiency;
        engine.force_scale = force_scale;
        engine.gear_force_exponent = gear_force_exponent;
    }

    pub fn set_torque_curve(&mut self, rpms: js_sys::Float32Array, torques: js_sys::Float32Array) {
        let rpms = rpms.to_vec();
        let torques = torques.to_vec();
        self.config.engine.torque_curve = rpms.into_iter().zip(torques).collect();
    }

    pub fn set_transmission(
        &mut self,
        reverse_ratio: Real,
        forward_ratios: js_sys::Float32Array,
        final_drive_ratio: Real,
        automatic: bool,
        auto_reverse: bool,
        clutch_response: Real,
        shift_cooldown: Real,
        upshift_range_position: Real,
        downshift_range_position: Real,
        stopped_speed: Real,
    ) {
        let transmission = &mut self.config.transmission;
        transmission.reverse_ratio = reverse_ratio;
        transmission.forward_ratios = forward_ratios.to_vec();
        transmission.final_drive_ratio = final_drive_ratio;
        transmission.automatic = automatic;
        transmission.auto_reverse = auto_reverse;
        transmission.clutch_response = clutch_response;
        transmission.shift_cooldown = shift_cooldown;
        transmission.upshift_range_position = upshift_range_position;
        transmission.downshift_range_position = downshift_range_position;
        transmission.stopped_speed = stopped_speed;
    }

    pub fn set_turbo(
        &mut self,
        enabled: bool,
        max_boost: Real,
        spool_rate: Real,
        release_rate: Real,
    ) {
        let turbo = &mut self.config.turbo;
        turbo.enabled = enabled;
        turbo.max_boost = max_boost;
        turbo.spool_rate = spool_rate;
        turbo.release_rate = release_rate;
    }

    pub fn set_dynamics(
        &mut self,
        brake_bias: Real,
        abs_strength: Real,
        traction_control_strength: Real,
        esc_strength: Real,
        drag_coefficient: Real,
        frontal_area: Real,
        rolling_resistance: Real,
        downforce_coefficient: Real,
        base_linear_damping: Real,
        linear_damping_per_speed: Real,
        base_angular_damping: Real,
        angular_damping_per_speed: Real,
    ) {
        let dynamics = &mut self.config.dynamics;
        dynamics.brake_bias = brake_bias;
        dynamics.abs_strength = abs_strength;
        dynamics.traction_control_strength = traction_control_strength;
        dynamics.esc_strength = esc_strength;
        dynamics.drag_coefficient = drag_coefficient;
        dynamics.frontal_area = frontal_area;
        dynamics.rolling_resistance = rolling_resistance;
        dynamics.downforce_coefficient = downforce_coefficient;
        dynamics.base_linear_damping = base_linear_damping;
        dynamics.linear_damping_per_speed = linear_damping_per_speed;
        dynamics.base_angular_damping = base_angular_damping;
        dynamics.angular_damping_per_speed = angular_damping_per_speed;
    }

    pub fn set_steering(
        &mut self,
        max_angle: Real,
        speed_sensitivity: Real,
        minimum_speed_factor: Real,
        assist: bool,
        drift_correction: Real,
    ) {
        let steering = &mut self.config.steering;
        steering.max_angle = max_angle;
        steering.speed_sensitivity = speed_sensitivity;
        steering.minimum_speed_factor = minimum_speed_factor;
        steering.assist = assist;
        steering.drift_correction = drift_correction;
    }
}

#[wasm_bindgen]
pub struct RawDynamicRayCastVehicleController {
    controller: DynamicRayCastVehicleController,
}

#[wasm_bindgen]
impl RawDynamicRayCastVehicleController {
    #[wasm_bindgen(constructor)]
    pub fn new(chassis: FlatHandle, config: RawVehicleControllerConfig) -> Self {
        Self {
            controller: DynamicRayCastVehicleController::new(
                utils::body_handle(chassis),
                config.config,
            ),
        }
    }

    pub fn set_input(
        &mut self,
        throttle: Real,
        brake: Real,
        clutch: Real,
        handbrake: Real,
        steering: Real,
    ) {
        self.controller.set_input(VehicleInput {
            throttle,
            brake,
            clutch,
            handbrake,
            steering,
        });
    }

    pub fn shift_up(&mut self) {
        self.controller.shift_up();
    }

    pub fn shift_down(&mut self) {
        self.controller.shift_down();
    }

    pub fn set_gear(&mut self, gear: i32) {
        self.controller.set_gear(gear);
    }

    pub fn set_steering_assist(&mut self, enabled: bool) {
        self.controller.set_steering_assist(enabled);
    }

    pub fn set_drift_correction(&mut self, correction: Real) {
        self.controller.set_drift_correction(correction);
    }

    pub fn engine_rpm(&self) -> Real {
        self.controller.state().engine_rpm
    }
    pub fn current_gear(&self) -> i32 {
        self.controller.state().current_gear
    }
    pub fn reverse_direction(&self) -> bool {
        self.controller.state().reverse_direction
    }
    pub fn vehicle_speed(&self) -> Real {
        self.controller.state().vehicle_speed
    }
    pub fn driven_wheel_speed(&self) -> Real {
        self.controller.state().driven_wheel_speed
    }
    pub fn steering_angle(&self) -> Real {
        self.controller.state().steering_angle
    }
    pub fn engine_load(&self) -> Real {
        self.controller.state().engine_load
    }
    pub fn rev_limiter_amount(&self) -> Real {
        self.controller.state().rev_limiter_amount
    }
    pub fn turbo_load(&self) -> Real {
        self.controller.state().turbo_load
    }
    pub fn turbo_release_sequence(&self) -> u32 {
        self.controller.state().turbo_release_sequence
    }
    pub fn wheels_in_contact(&self) -> usize {
        self.controller.state().wheels_in_contact
    }
    pub fn abs_activity(&self) -> Real {
        self.controller.state().abs_activity
    }
    pub fn traction_control_activity(&self) -> Real {
        self.controller.state().traction_control_activity
    }
    pub fn force_feedback(&self) -> Real {
        self.controller.state().force_feedback
    }
    pub fn steering_friction(&self) -> Real {
        self.controller.state().steering_friction
    }

    pub fn current_vehicle_speed(&self) -> Real {
        self.controller.current_vehicle_speed
    }

    pub fn chassis(&self) -> FlatHandle {
        utils::flat_handle(self.controller.chassis.0)
    }

    pub fn index_up_axis(&self) -> usize {
        self.controller.index_up_axis
    }
    pub fn set_index_up_axis(&mut self, axis: usize) {
        self.controller.index_up_axis = axis;
    }

    pub fn index_forward_axis(&self) -> usize {
        self.controller.index_forward_axis
    }
    pub fn set_index_forward_axis(&mut self, axis: usize) {
        self.controller.index_forward_axis = axis;
    }

    pub fn esc(&self) -> Real {
        self.controller.esc
    }
    pub fn set_esc(&mut self, value: Real) {
        self.controller.esc = value;
    }

    pub fn add_wheel(
        &mut self,
        chassis_connection_cs: &RawVector,
        direction_cs: &RawVector,
        axle_cs: &RawVector,
        suspension_rest_length: Real,
        radius: Real,
        axle: u32,
        driven: bool,
        steered: bool,
    ) {
        let axle = if axle == 0 {
            WheelAxle::Front
        } else {
            WheelAxle::Rear
        };
        self.controller.add_wheel(
            chassis_connection_cs.0.into(),
            direction_cs.0,
            axle_cs.0,
            suspension_rest_length,
            radius,
            &WheelTuning::default(),
            WheelRole::new(axle, driven, steered),
        );
    }

    pub fn num_wheels(&self) -> usize {
        self.controller.wheels().len()
    }

    pub fn update_vehicle(
        &mut self,
        dt: Real,
        bodies: &mut RawRigidBodySet,
        colliders: &RawColliderSet,
        queries: &RawQueryPipeline,
        filter_flags: u32,
        filter_groups: Option<u32>,
        filter_predicate: &js_sys::Function,
    ) {
        crate::utils::with_filter(filter_predicate, |predicate| {
            let query_filter = QueryFilter {
                flags: QueryFilterFlags::from_bits(filter_flags)
                    .unwrap_or(QueryFilterFlags::empty()),
                groups: filter_groups.map(crate::geometry::unpack_interaction_groups),
                predicate,
                exclude_rigid_body: Some(self.controller.chassis),
                exclude_collider: None,
            };

            self.controller.update_vehicle(
                dt,
                &mut bodies.0,
                &colliders.0,
                &queries.0,
                query_filter,
            );
        });
    }

    /*
     *
     * Access to wheel properties.
     *
     */
    /*
     * Getters + setters
     */
    pub fn wheel_chassis_connection_point_cs(&self, i: usize) -> Option<RawVector> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.chassis_connection_point_cs.into())
    }
    pub fn set_wheel_chassis_connection_point_cs(&mut self, i: usize, value: &RawVector) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.chassis_connection_point_cs = value.0.into();
        }
    }

    pub fn wheel_suspension_rest_length(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.suspension_rest_length)
    }
    pub fn set_wheel_suspension_rest_length(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.suspension_rest_length = value;
        }
    }

    pub fn wheel_max_suspension_travel(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.max_suspension_travel)
    }
    pub fn set_wheel_max_suspension_travel(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.max_suspension_travel = value;
        }
    }

    pub fn wheel_radius(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.radius)
    }
    pub fn set_wheel_radius(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.radius = value;
        }
    }

    pub fn wheel_suspension_stiffness(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.suspension_stiffness)
    }
    pub fn set_wheel_suspension_stiffness(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.suspension_stiffness = value;
        }
    }

    pub fn wheel_suspension_compression(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.damping_compression)
    }
    pub fn set_wheel_suspension_compression(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.damping_compression = value;
        }
    }

    pub fn wheel_suspension_relaxation(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.damping_relaxation)
    }
    pub fn set_wheel_suspension_relaxation(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.damping_relaxation = value;
        }
    }

    pub fn wheel_max_suspension_force(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.max_suspension_force)
    }
    pub fn set_wheel_max_suspension_force(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.max_suspension_force = value;
        }
    }

    pub fn wheel_brake(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.brake)
    }
    pub fn set_wheel_brake(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.brake = value;
        }
    }

    pub fn wheel_steering(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.steering)
    }
    pub fn set_wheel_steering(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.steering = value;
        }
    }

    pub fn wheel_engine_force(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.engine_force)
    }
    pub fn set_wheel_engine_force(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.engine_force = value;
        }
    }

    pub fn wheel_direction_cs(&self, i: usize) -> Option<RawVector> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.direction_cs.into())
    }
    pub fn set_wheel_direction_cs(&mut self, i: usize, value: &RawVector) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.direction_cs = value.0;
        }
    }

    pub fn wheel_axle_cs(&self, i: usize) -> Option<RawVector> {
        self.controller.wheels().get(i).map(|w| w.axle_cs.into())
    }
    pub fn set_wheel_axle_cs(&mut self, i: usize, value: &RawVector) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.axle_cs = value.0;
        }
    }

    pub fn wheel_friction_slip(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.friction_slip)
    }
    pub fn set_wheel_friction_slip(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.friction_slip = value;
        }
    }

    pub fn wheel_side_friction_stiffness(&self, i: usize) -> Option<f32> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.side_friction_stiffness)
    }

    pub fn set_wheel_side_friction_stiffness(&mut self, i: usize, stiffness: f32) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.side_friction_stiffness = stiffness;
        }
    }

    pub fn wheel_target_rotation(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.target_rotation)
    }
    pub fn set_wheel_target_rotation(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.target_rotation = value;
        }
    }

    pub fn wheel_max_brake_force(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.max_brake_force)
    }
    pub fn set_wheel_max_brake_force(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.max_brake_force = value;
        }
    }

    pub fn wheel_anti_lock_brake(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.anti_lock_brake)
    }
    pub fn set_wheel_anti_lock_brake(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.anti_lock_brake = value;
        }
    }

    pub fn wheel_traction_control(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.traction_control)
    }
    pub fn set_wheel_traction_control(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.traction_control = value;
        }
    }

    pub fn wheel_anti_roll(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.anti_roll)
    }
    pub fn set_wheel_anti_roll(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.anti_roll = value;
        }
    }

    pub fn wheel_tire_type(&self, i: usize) -> Option<String> {
        self.controller.wheels().get(i).map(|w| w.tire_type.clone())
    }

    pub fn set_wheel_tire_type(&mut self, i: usize, tire_type: &str) {
        self.controller.set_wheel_tire_type(i, tire_type)
    }

    pub fn add_tire_type(&mut self, tire_type: &str, friction: f32) {
        self.controller.add_tire_type(tire_type, friction);
    }

    pub fn add_surface_to_tire_type(&mut self, tire_type: &str, surface: &str, friction: f32) {
        self.controller
            .add_surface_to_tire_type(tire_type, surface, friction);
    }

    pub fn wheel_side_factor(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.side_factor)
    }

    pub fn set_wheel_side_factor(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.side_factor = value;
        }
    }

    pub fn wheel_forward_factor(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.fwd_factor)
    }

    pub fn set_wheel_forward_factor(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.fwd_factor = value;
        }
    }

    pub fn wheel_brake_factor(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.brake_factor)
    }

    pub fn set_wheel_brake_factor(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.brake_factor = value;
        }
    }

    pub fn wheel_contact_damping(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.contact_damping)
    }

    pub fn set_wheel_contact_damping(&mut self, i: usize, value: Real) {
        if let Some(wheel) = self.controller.wheels_mut().get_mut(i) {
            wheel.contact_damping = value;
            wheel.base_contact_damping = value;
        }
    }

    /*
     * Getters only.
     */
    pub fn wheel_rotation(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.rotation)
    }

    pub fn wheel_is_anti_lock_brake(&self, i: usize) -> Option<bool> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.is_anti_lock_brake)
    }

    pub fn wheel_delta_rotation(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.delta_rotation)
    }

    pub fn wheel_skid_info(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.skid_info)
    }

    pub fn wheel_ground_friction(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.ground_friction)
    }

    pub fn wheel_ground_type(&self, i: usize) -> Option<String> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.ground_type.clone())
    }

    pub fn wheel_engine_force_feedback(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.engine_force_feedback)
    }

    pub fn wheel_suspension_compression_rate(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.suspension_compression_rate)
    }

    pub fn wheel_forward_impulse(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.forward_impulse)
    }

    pub fn wheel_side_impulse(&self, i: usize) -> Option<Real> {
        self.controller.wheels().get(i).map(|w| w.side_impulse)
    }

    pub fn wheel_suspension_force(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.wheel_suspension_force)
    }

    pub fn wheel_contact_normal_ws(&self, i: usize) -> Option<RawVector> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.raycast_info().contact_normal_ws.into())
    }

    pub fn wheel_contact_point_ws(&self, i: usize) -> Option<RawVector> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.raycast_info().contact_point_ws.into())
    }

    pub fn wheel_suspension_length(&self, i: usize) -> Option<Real> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.raycast_info().suspension_length)
    }

    pub fn wheel_hard_point_ws(&self, i: usize) -> Option<RawVector> {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.raycast_info().hard_point_ws.into())
    }

    pub fn wheel_is_in_contact(&self, i: usize) -> bool {
        self.controller
            .wheels()
            .get(i)
            .map(|w| w.raycast_info().is_in_contact)
            .unwrap_or(false)
    }

    pub fn wheel_ground_object(&self, i: usize) -> Option<FlatHandle> {
        self.controller
            .wheels()
            .get(i)
            .and_then(|w| w.raycast_info().ground_object)
            .map(|h| utils::flat_handle(h.0))
    }
}
