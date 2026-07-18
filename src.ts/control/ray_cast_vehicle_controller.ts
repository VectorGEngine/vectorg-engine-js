import {
    RawDynamicRayCastVehicleController,
    RawVehicleControllerConfig,
} from "../raw";
import {Vector, VectorOps} from "../math";
import {Collider, ColliderSet, InteractionGroups} from "../geometry";
import {QueryFilterFlags, QueryPipeline} from "../pipeline";
import {RigidBody, RigidBodyHandle, RigidBodySet} from "../dynamics";

export interface VehicleEngineConfig {
    horsepower: number;
    idleRpm: number;
    maxRpm: number;
    revLimitRpm: number;
    inertia: number;
    frictionTorque: number | null;
    engineBraking: number;
    drivetrainEfficiency: number;
    forceScale: number;
    gearForceExponent: number;
    torqueCurve: Array<{rpm: number; torque: number}>;
}

export interface VehicleTransmissionConfig {
    reverseRatio: number;
    forwardRatios: number[];
    finalDriveRatio: number;
    automatic: boolean;
    autoReverse: boolean;
    clutchResponse: number;
    shiftCooldown: number;
    upshiftRangePosition: number;
    downshiftRangePosition: number;
    stoppedSpeed: number;
}

export interface VehicleTurboConfig {
    enabled: boolean;
    maxBoost: number;
    spoolRate: number;
    releaseRate: number;
}

export interface VehicleDynamicsConfig {
    brakeBias: number;
    absStrength: number;
    tractionControlStrength: number;
    escStrength: number;
    dragCoefficient: number;
    frontalArea: number;
    rollingResistance: number;
    downforceCoefficient: number;
    baseLinearDamping: number;
    linearDampingPerSpeed: number;
    baseAngularDamping: number;
    angularDampingPerSpeed: number;
}

export interface VehicleSteeringConfig {
    maxAngle: number;
    speedSensitivity: number;
    minimumSpeedFactor: number;
    assist: boolean;
    /** Drift correction strength (`0` = none, `1` = full correction). */
    driftCorrection: number;
}

export interface VehicleControllerConfig {
    engine: VehicleEngineConfig;
    transmission: VehicleTransmissionConfig;
    turbo: VehicleTurboConfig;
    dynamics: VehicleDynamicsConfig;
    steering: VehicleSteeringConfig;
}

export interface VehicleInput {
    throttle: number;
    brake: number;
    clutch: number;
    handbrake: number;
    steering: number;
}

export interface VehicleState {
    engineRpm: number;
    currentGear: number;
    reverseDirection: boolean;
    vehicleSpeed: number;
    drivenWheelSpeed: number;
    steeringAngle: number;
    engineLoad: number;
    revLimiterAmount: number;
    turboLoad: number;
    turboReleaseSequence: number;
    wheelsInContact: number;
    absActivity: number;
    tractionControlActivity: number;
    forceFeedback: number;
    steeringFriction: number;
}

export interface VehicleWheelRole {
    axle: "front" | "rear";
    driven: boolean;
    steered: boolean;
}

/**
 * A character controller to simulate vehicles using ray-casting for the wheels.
 */
export class DynamicRayCastVehicleController {
    private raw: RawDynamicRayCastVehicleController;
    private bodies: RigidBodySet;
    private colliders: ColliderSet;
    private queries: QueryPipeline;
    private _chassis: RigidBody;
    private currentState: VehicleState;

    constructor(
        chassis: RigidBody,
        bodies: RigidBodySet,
        colliders: ColliderSet,
        queries: QueryPipeline,
        config: VehicleControllerConfig,
    ) {
        const rawConfig = new RawVehicleControllerConfig();
        const engine = config.engine;
        rawConfig.set_engine(
            engine.horsepower,
            engine.idleRpm,
            engine.maxRpm,
            engine.revLimitRpm,
            engine.inertia,
            engine.frictionTorque,
            engine.engineBraking,
            engine.drivetrainEfficiency,
            engine.forceScale,
            engine.gearForceExponent,
        );
        rawConfig.set_torque_curve(
            new Float32Array(engine.torqueCurve.map((point) => point.rpm)),
            new Float32Array(engine.torqueCurve.map((point) => point.torque)),
        );
        const transmission = config.transmission;
        rawConfig.set_transmission(
            transmission.reverseRatio,
            new Float32Array(transmission.forwardRatios),
            transmission.finalDriveRatio,
            transmission.automatic,
            transmission.autoReverse,
            transmission.clutchResponse,
            transmission.shiftCooldown,
            transmission.upshiftRangePosition,
            transmission.downshiftRangePosition,
            transmission.stoppedSpeed,
        );
        const turbo = config.turbo;
        rawConfig.set_turbo(
            turbo.enabled,
            turbo.maxBoost,
            turbo.spoolRate,
            turbo.releaseRate,
        );
        const dynamics = config.dynamics;
        rawConfig.set_dynamics(
            dynamics.brakeBias,
            dynamics.absStrength,
            dynamics.tractionControlStrength,
            dynamics.escStrength,
            dynamics.dragCoefficient,
            dynamics.frontalArea,
            dynamics.rollingResistance,
            dynamics.downforceCoefficient,
            dynamics.baseLinearDamping,
            dynamics.linearDampingPerSpeed,
            dynamics.baseAngularDamping,
            dynamics.angularDampingPerSpeed,
        );
        const steering = config.steering;
        rawConfig.set_steering(
            steering.maxAngle,
            steering.speedSensitivity,
            steering.minimumSpeedFactor,
            steering.assist,
            steering.driftCorrection,
        );
        this.raw = new RawDynamicRayCastVehicleController(
            chassis.handle,
            rawConfig,
        );
        this.bodies = bodies;
        this.colliders = colliders;
        this.queries = queries;
        this._chassis = chassis;
        this.currentState = this.readState();
    }

    /** @internal */
    public free() {
        if (!!this.raw) {
            this.raw.free();
        }

        this.raw = undefined;
    }

    /**
     * Updates the vehicle’s velocity based on its suspension, engine force, and brake.
     *
     * This directly updates the velocity of its chassis rigid-body.
     *
     * @param dt - Time increment used to integrate forces.
     * @param filterFlags - Flag to exclude categories of objects from the wheels’ ray-cast.
     * @param filterGroups - Only colliders compatible with these groups will be hit by the wheels’ ray-casts.
     * @param filterPredicate - Callback to filter out which collider will be hit by the wheels’ ray-casts.
     */
    public updateVehicle(
        dt: number,
        filterFlags?: QueryFilterFlags,
        filterGroups?: InteractionGroups,
        filterPredicate?: (collider: Collider) => boolean,
    ) {
        this.raw.update_vehicle(
            dt,
            this.bodies.raw,
            this.colliders.raw,
            this.queries.raw,
            filterFlags,
            filterGroups,
            this.colliders.castClosure(filterPredicate),
        );
        this.currentState = this.readState();
    }

    /** Sets normalized driver inputs consumed by subsequent updates. */
    public setInput(input: VehicleInput) {
        this.raw.set_input(
            input.throttle,
            input.brake,
            input.clutch,
            input.handbrake,
            input.steering,
        );
    }

    /** Requests the next higher gear. */
    public shiftUp() {
        this.raw.shift_up();
    }

    /** Requests the next lower gear. */
    public shiftDown() {
        this.raw.shift_down();
    }

    /** Selects a gear, where -1 is reverse and 0 is neutral. */
    public setGear(gear: number) {
        this.raw.set_gear(gear);
    }

    /** Enables or disables velocity-based counter-steering assistance. */
    public setSteeringAssist(enabled: boolean) {
        this.raw.set_steering_assist(enabled);
    }

    /** Sets drift correction strength (`0` = none, `1` = full correction). */
    public setDriftCorrection(correction: number) {
        this.raw.set_drift_correction(correction);
    }

    /** Current engine, transmission, driver-assistance, and feedback state. */
    public state(): VehicleState {
        return this.currentState;
    }

    private readState(): VehicleState {
        return {
            engineRpm: this.raw.engine_rpm(),
            currentGear: this.raw.current_gear(),
            reverseDirection: this.raw.reverse_direction(),
            vehicleSpeed: this.raw.vehicle_speed(),
            drivenWheelSpeed: this.raw.driven_wheel_speed(),
            steeringAngle: this.raw.steering_angle(),
            engineLoad: this.raw.engine_load(),
            revLimiterAmount: this.raw.rev_limiter_amount(),
            turboLoad: this.raw.turbo_load(),
            turboReleaseSequence: this.raw.turbo_release_sequence(),
            wheelsInContact: this.raw.wheels_in_contact(),
            absActivity: this.raw.abs_activity(),
            tractionControlActivity: this.raw.traction_control_activity(),
            forceFeedback: this.raw.force_feedback(),
            steeringFriction: this.raw.steering_friction(),
        };
    }

    /**
     * The current forward speed of the vehicle.
     */
    public currentVehicleSpeed(): number {
        return this.raw.current_vehicle_speed();
    }

    /**
     * The rigid-body used as the chassis.
     */
    public chassis(): RigidBody {
        return this._chassis;
    }

    /**
     * The chassis’ local _up_ direction (`0 = x, 1 = y, 2 = z`).
     */
    get indexUpAxis(): number {
        return this.raw.index_up_axis();
    }

    /**
     * Sets the chassis’ local _up_ direction (`0 = x, 1 = y, 2 = z`).
     */
    set indexUpAxis(axis: number) {
        this.raw.set_index_up_axis(axis);
    }

    /**
     * The chassis’ local _forward_ direction (`0 = x, 1 = y, 2 = z`).
     */
    get indexForwardAxis(): number {
        return this.raw.index_forward_axis();
    }

    /**
     * Sets the chassis’ local _forward_ direction (`0 = x, 1 = y, 2 = z`).
     */
    set indexForwardAxis(axis: number) {
        this.raw.set_index_forward_axis(axis);
    }

    /**
     * The ESC (Electronic Stability Control) value of the vehicle.
     */
    get esc(): number {
        return this.raw.esc();
    }

    /**
     * Sets the ESC (Electronic Stability Control) value of the vehicle.
     */
    set esc(value: number) {
        this.raw.set_esc(value);
    }

    /**
     * Adds a new wheel attached to this vehicle.
     * @param chassisConnectionCs  - The position of the wheel relative to the chassis.
     * @param directionCs - The direction of the wheel’s suspension, relative to the chassis. The ray-casting will
     *                      happen following this direction to detect the ground.
     * @param axleCs - The wheel’s axle axis, relative to the chassis.
     * @param suspensionRestLength - The rest length of the wheel’s suspension spring.
     * @param radius - The wheel’s radius.
     */
    public addWheel(
        chassisConnectionCs: Vector,
        directionCs: Vector,
        axleCs: Vector,
        suspensionRestLength: number,
        radius: number,
        role: VehicleWheelRole,
    ) {
        let rawChassisConnectionCs = VectorOps.intoRaw(chassisConnectionCs);
        let rawDirectionCs = VectorOps.intoRaw(directionCs);
        let rawAxleCs = VectorOps.intoRaw(axleCs);

        this.raw.add_wheel(
            rawChassisConnectionCs,
            rawDirectionCs,
            rawAxleCs,
            suspensionRestLength,
            radius,
            role.axle === "front" ? 0 : 1,
            role.driven,
            role.steered,
        );

        rawChassisConnectionCs.free();
        rawDirectionCs.free();
        rawAxleCs.free();
    }

    /**
     * The number of wheels attached to this vehicle.
     */
    public numWheels(): number {
        return this.raw.num_wheels();
    }

    /*
     *
     * Access to wheel properties.
     *
     */
    /*
     * Getters + setters
     */
    /**
     * The position of the i-th wheel, relative to the chassis.
     */
    public wheelChassisConnectionPointCs(i: number): Vector | null {
        return VectorOps.fromRaw(this.raw.wheel_chassis_connection_point_cs(i));
    }

    /**
     * Sets the position of the i-th wheel, relative to the chassis.
     */
    public setWheelChassisConnectionPointCs(i: number, value: Vector) {
        let rawValue = VectorOps.intoRaw(value);
        this.raw.set_wheel_chassis_connection_point_cs(i, rawValue);
        rawValue.free();
    }

    /**
     * The rest length of the i-th wheel’s suspension spring.
     */
    public wheelSuspensionRestLength(i: number): number | null {
        return this.raw.wheel_suspension_rest_length(i);
    }

    /**
     * Sets the rest length of the i-th wheel’s suspension spring.
     */
    public setWheelSuspensionRestLength(i: number, value: number) {
        this.raw.set_wheel_suspension_rest_length(i, value);
    }

    /**
     * The maximum distance the i-th wheel suspension can travel before and after its resting length.
     */
    public wheelMaxSuspensionTravel(i: number): number | null {
        return this.raw.wheel_max_suspension_travel(i);
    }

    /**
     * Sets the maximum distance the i-th wheel suspension can travel before and after its resting length.
     */
    public setWheelMaxSuspensionTravel(i: number, value: number) {
        this.raw.set_wheel_max_suspension_travel(i, value);
    }

    /**
     * The i-th wheel’s radius.
     */
    public wheelRadius(i: number): number | null {
        return this.raw.wheel_radius(i);
    }

    /**
     * Sets the i-th wheel’s radius.
     */
    public setWheelRadius(i: number, value: number) {
        this.raw.set_wheel_radius(i, value);
    }

    /**
     * The i-th wheel’s suspension stiffness.
     *
     * Increase this value if the suspension appears to not push the vehicle strong enough.
     */
    public wheelSuspensionStiffness(i: number): number | null {
        return this.raw.wheel_suspension_stiffness(i);
    }

    /**
     * Sets the i-th wheel’s suspension stiffness.
     *
     * Increase this value if the suspension appears to not push the vehicle strong enough.
     */
    public setWheelSuspensionStiffness(i: number, value: number) {
        this.raw.set_wheel_suspension_stiffness(i, value);
    }

    /**
     * The i-th wheel’s suspension’s damping when it is being compressed.
     */
    public wheelSuspensionCompression(i: number): number | null {
        return this.raw.wheel_suspension_compression(i);
    }

    /**
     * The i-th wheel’s suspension’s damping when it is being compressed.
     */
    public setWheelSuspensionCompression(i: number, value: number) {
        this.raw.set_wheel_suspension_compression(i, value);
    }

    /**
     * The i-th wheel’s suspension’s damping when it is being released.
     *
     * Increase this value if the suspension appears to overshoot.
     */
    public wheelSuspensionRelaxation(i: number): number | null {
        return this.raw.wheel_suspension_relaxation(i);
    }

    /**
     * Sets the i-th wheel’s suspension’s damping when it is being released.
     *
     * Increase this value if the suspension appears to overshoot.
     */
    public setWheelSuspensionRelaxation(i: number, value: number) {
        this.raw.set_wheel_suspension_relaxation(i, value);
    }

    /**
     * The maximum force applied by the i-th wheel’s suspension.
     */
    public wheelMaxSuspensionForce(i: number): number | null {
        return this.raw.wheel_max_suspension_force(i);
    }

    /**
     * Sets the maximum force applied by the i-th wheel’s suspension.
     */
    public setWheelMaxSuspensionForce(i: number, value: number) {
        this.raw.set_wheel_max_suspension_force(i, value);
    }

    /**
     * The maximum amount of braking multiplier applied on the i-th wheel to slow down the vehicle.
     */
    public wheelBrake(i: number): number | null {
        return this.raw.wheel_brake(i);
    }

    /**
     * Set the maximum amount of braking multiplier applied on the i-th wheel to slow down the vehicle.
     */
    public setWheelBrake(i: number, value: number) {
        this.raw.set_wheel_brake(i, value);
    }

    /**
     * The steering angle (radians) for the i-th wheel.
     */
    public wheelSteering(i: number): number | null {
        return this.raw.wheel_steering(i);
    }

    /**
     * Sets the steering angle (radians) for the i-th wheel.
     */
    public setWheelSteering(i: number, value: number) {
        this.raw.set_wheel_steering(i, value);
    }

    /**
     * The forward force applied by the i-th wheel on the chassis.
     */
    public wheelEngineForce(i: number): number | null {
        return this.raw.wheel_engine_force(i);
    }

    /**
     * Sets the forward force applied by the i-th wheel on the chassis.
     */
    public setWheelEngineForce(i: number, value: number) {
        this.raw.set_wheel_engine_force(i, value);
    }

    /**
     * The direction of the i-th wheel’s suspension, relative to the chassis.
     *
     * The ray-casting will happen following this direction to detect the ground.
     */
    public wheelDirectionCs(i: number): Vector | null {
        return VectorOps.fromRaw(this.raw.wheel_direction_cs(i));
    }

    /**
     * Sets the direction of the i-th wheel’s suspension, relative to the chassis.
     *
     * The ray-casting will happen following this direction to detect the ground.
     */
    public setWheelDirectionCs(i: number, value: Vector) {
        let rawValue = VectorOps.intoRaw(value);
        this.raw.set_wheel_direction_cs(i, rawValue);
        rawValue.free();
    }

    /**
     * The i-th wheel’s axle axis, relative to the chassis.
     *
     * The axis index defined as 0 = X, 1 = Y, 2 = Z.
     */
    public wheelAxleCs(i: number): Vector | null {
        return VectorOps.fromRaw(this.raw.wheel_axle_cs(i));
    }

    /**
     * Sets the i-th wheel’s axle axis, relative to the chassis.
     *
     * The axis index defined as 0 = X, 1 = Y, 2 = Z.
     */
    public setWheelAxleCs(i: number, value: Vector) {
        let rawValue = VectorOps.intoRaw(value);
        this.raw.set_wheel_axle_cs(i, rawValue);
        rawValue.free();
    }

    /**
     * Parameter controlling how much traction the tire has.
     *
     * The larger the value, the more instantaneous braking will happen (with the risk of
     * causing the vehicle to flip if it’s too strong).
     */
    public wheelFrictionSlip(i: number): number | null {
        return this.raw.wheel_friction_slip(i);
    }

    /**
     * Sets the parameter controlling how much traction the tire has.
     *
     * The larger the value, the more instantaneous braking will happen (with the risk of
     * causing the vehicle to flip if it’s too strong).
     */
    public setWheelFrictionSlip(i: number, value: number) {
        this.raw.set_wheel_friction_slip(i, value);
    }

    /**
     * The multiplier of friction between a tire and the collider it’s on top of.
     *
     * The larger the value, the stronger side friction will be.
     */
    public wheelSideFrictionStiffness(i: number): number | null {
        return this.raw.wheel_side_friction_stiffness(i);
    }

    /**
     * The multiplier of friction between a tire and the collider it’s on top of.
     *
     * The larger the value, the stronger side friction will be.
     */
    public setWheelSideFrictionStiffness(i: number, value: number) {
        this.raw.set_wheel_side_friction_stiffness(i, value);
    }

    /**
     *  The i-th wheel’s current target angle (radians) on its axle.
     */
    public wheelTargetRotation(i: number): number | null {
        return this.raw.wheel_target_rotation(i);
    }

    /**
     * The i-th wheel’s target rotation angle (radians) on its axle.
     */
    public setWheelTargetRotation(i: number, value: number) {
        this.raw.set_wheel_target_rotation(i, value);
    }

    /**
     * The i-th wheel’s brake force.
     *
     * This is the maximum amount of braking impulse applied on the i-th wheel to slow down the vehicle.
     */
    public wheelMaxBrakeForce(i: number): number | null {
        return this.raw.wheel_max_brake_force(i);
    }

    /**
     * The i-th wheel’s brake force.
     *
     * This is the maximum amount of braking impulse applied on the i-th wheel to slow down the vehicle.
     */
    public setWheelMaxBrakeForce(i: number, value: number) {
        this.raw.set_wheel_max_brake_force(i, value);
    }

    /**
     * The i-th wheel’s anti-lock brake value.
     *
     * This is the anti-lock brake value applied on the i-th wheel to prevent it from locking up.
     * 0.0 means no anti-lock brake is applied, 1.0 means the maximum anti-lock brake is applied.
     */
    public wheelAntiLockBrake(i: number): number {
        return this.raw.wheel_anti_lock_brake(i);
    }

    public wheelIsAntiLockBrake(i: number): boolean {
        return this.raw.wheel_is_anti_lock_brake(i);
    }

    /**
     * The i-th wheel’s anti-lock brake value.
     *
     * This is the anti-lock brake value applied on the i-th wheel to prevent it from locking up.
     * 0 means no anti-lock brake is applied, 1 means the maximum anti-lock brake is applied.
     */
    public setWheelAntiLockBrake(i: number, value: number) {
        this.raw.set_wheel_anti_lock_brake(i, value);
    }


    /**
     * The i-th wheel’s anti-roll value.
     *
     * This is the anti-roll value applied on the i-th wheel to prevent the vehicle from rolling over.
     * 0.0 means no anti-roll is applied, 1.0 means the maximum anti-roll is applied.
     */
    public wheelAntiRoll(i: number): number | null {
        return this.raw.wheel_anti_roll(i);
    }

    /**
     * The i-th wheel’s anti-roll value.
     *
     * This is the anti-roll value applied on the i-th wheel to prevent the vehicle from rolling over.
     * 0 means no anti-roll is applied, 1 means the maximum anti-roll is applied.
     */
    public setWheelAntiRoll(i: number, value: number) {
        this.raw.set_wheel_anti_roll(i, value);
    }

    /**
     * The i-th wheel’s traction control value.
     *
     * This is the traction control value applied on the i-th wheel to prevent it from slipping.
     * 0.0 means no traction control is applied, 1.0 means the maximum traction control is applied.
     */
    public wheelTractionControl(i: number): number | null {
        return this.raw.wheel_traction_control(i);
    }

    /**
     * Sets the i-th wheel’s traction control value.
     *
     * This is the traction control value applied on the i-th wheel to prevent it from slipping.
     * 0 means no traction control is applied, 1 means the maximum traction control is applied.
     */
    public setWheelTractionControl(i: number, value: number) {
        this.raw.set_wheel_traction_control(i, value);
    }

    /**
     * The i-th wheel’s tire type.
     *
     * The tire type defines how the tire behaves on different surfaces.
     */
    public wheelTireType(i: number): string | null {
        return this.raw.wheel_tire_type(i);
    }

    /**
     * Sets the i-th wheel’s tire type.
     *
     * The tire type defines how the tire behaves on different surfaces.
     */
    public setWheelTireType(i: number, tireType: string) {
        this.raw.set_wheel_tire_type(i, tireType);
    }

    public addTireType(tireType: string, friction:number): DynamicRayCastVehicleController {
        this.raw.add_tire_type(tireType, friction);
        return this;
    }

    public addSurfaceToTireType(tireType: string, surface: string, friction: number): DynamicRayCastVehicleController {
        this.raw.add_surface_to_tire_type(tireType, surface, friction);
        return this;
    }

    public wheelSideFactor(i: number): number | null {
        return this.raw.wheel_side_factor(i);
    }

    public setWheelSideFactor(i: number, value: number) {
        this.raw.set_wheel_side_factor(i, value);
    }

    public wheelForwardFactor(i: number): number | null {
        return this.raw.wheel_forward_factor(i);
    }

    public setWheelForwardFactor(i: number, value: number) {
        this.raw.set_wheel_forward_factor(i, value);
    }

    public wheelBrakeFactor(i: number): number | null {
        return this.raw.wheel_brake_factor(i);
    }

    public setWheelBrakeFactor(i: number, value: number) {
        this.raw.set_wheel_brake_factor(i, value);
    }

    public wheelContactDamping(i: number): number | null {
        return this.raw.wheel_contact_damping(i);
    }

    public setWheelContactDamping(i: number, value: number) {
        this.raw.set_wheel_contact_damping(i, value);
    }

    /*
     * Getters only.
     */

    /**
     *  The i-th wheel’s current rotation angle (radians) on its axle.
     */
    public wheelRotation(i: number): number | null {
        return this.raw.wheel_rotation(i);
    }

    /**
     *  The i-th wheel’s current rotation angle (radians) on its axle.
     */
    public wheelDeltaRotation(i: number): number | null {
        return this.raw.wheel_delta_rotation(i);
    }

    /**
     *  The i-th wheel’s skid info.
     *
     *  This is a value between 0.0 and 1.0 that indicates how much the wheel is skidding.
     *  A value of 0.0 means the wheel is not skidding, while a value of 1.0 means the wheel is fully skidding.
     */
    public wheelSkidInfo(i: number): number | null {
        return this.raw.wheel_skid_info(i);
    }    

    /**
     *  The i-th wheel’s ground friction.
     *
     *  This is a value between 0.0 and 1.0 that indicates how much the wheel is affected by ground friction.
     *  A value of 0.0 means no ground friction is applied, while a value of 1.0 means maximum ground friction is applied.
     */
    public wheelGroundFriction(i: number): number | null {
        return this.raw.wheel_ground_friction(i);
    }

    /**
     *  The i-th wheel’s ground type.
     *
     *  This is a numeric identifier representing the type of surface the wheel is currently in contact with.
     *  Different ground types can affect the vehicle's handling and performance.
     */
    public wheelGroundType(i: number): string {
        return this.raw.wheel_ground_type(i);
    }

    /**
     *  The i-th wheel’s suspension compression.
     *
     *  This is a value between 0.0 and 1.0 that indicates how much the wheel’s suspension is compressed.
     *  A value of 0.0 means the suspension is not compressed, while a value of 1.0 means the suspension is fully compressed.
     */
    public wheelSuspensionCompressionRate(i: number): number | null {
        return this.raw.wheel_suspension_compression_rate(i);
    }

    /**
     *  The i-th wheel’s engine force feedback.
     */
    public wheelEngineForceFeedback(i: number): number | null {
        return this.raw.wheel_engine_force_feedback(i);
    }

    /**
     *  The forward impulses applied by the i-th wheel on the chassis.
     */
    public wheelForwardImpulse(i: number): number | null {
        return this.raw.wheel_forward_impulse(i);
    }

    /**
     *  The side impulses applied by the i-th wheel on the chassis.
     */
    public wheelSideImpulse(i: number): number | null {
        return this.raw.wheel_side_impulse(i);
    }

    /**
     *  The force applied by the i-th wheel suspension.
     */
    public wheelSuspensionForce(i: number): number | null {
        return this.raw.wheel_suspension_force(i);
    }

    /**
     *  The (world-space) contact normal between the i-th wheel and the floor.
     */
    public wheelContactNormal(i: number): Vector | null {
        return VectorOps.fromRaw(this.raw.wheel_contact_normal_ws(i));
    }

    /**
     *  The (world-space) point hit by the wheel’s ray-cast for the i-th wheel.
     */
    public wheelContactPoint(i: number): Vector | null {
        return VectorOps.fromRaw(this.raw.wheel_contact_point_ws(i));
    }

    /**
     *  The suspension length for the i-th wheel.
     */
    public wheelSuspensionLength(i: number): number | null {
        return this.raw.wheel_suspension_length(i);
    }

    /**
     *  The (world-space) starting point of the ray-cast for the i-th wheel.
     */
    public wheelHardPoint(i: number): Vector | null {
        return VectorOps.fromRaw(this.raw.wheel_hard_point_ws(i));
    }

    /**
     *  Is the i-th wheel in contact with the ground?
     */
    public wheelIsInContact(i: number): boolean {
        return this.raw.wheel_is_in_contact(i);
    }

    /**
     *  The collider hit by the ray-cast for the i-th wheel.
     */
    public wheelGroundObject(i: number): Collider | null {
        return this.colliders.get(this.raw.wheel_ground_object(i));
    }
}
