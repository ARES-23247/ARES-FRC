package com.areslib.frc.marvin

import com.areslib.state.SubsystemState
import com.areslib.state.SuperstructureState

/**
 * Immutable representation of the dual-motor shooter flywheel state.
 */
data class FlywheelState(
    val velocityRpm: Double = 0.0,
    val targetVelocityRpm: Double = 0.0,
    val currentAmps: Double = 0.0,
    val tempCelsius: Double = 0.0
)

/**
 * Immutable representation of the adjustable cowl/hood angle state.
 */
data class CowlState(
    val angleDegrees: Double = 0.0,
    val targetAngleDegrees: Double = 0.0,
    val currentAmps: Double = 0.0
)

/**
 * Immutable representation of the active pivot-arm intake and roller state.
 */
data class IntakeState(
    val pivotAngleDegrees: Double = 0.0,
    val targetAngleDegrees: Double = 0.0,
    val rollerVelocityRps: Double = 0.0,
    val targetRollerVelocityRps: Double = 0.0,
    val isDeployed: Boolean = false
)

/**
 * Immutable representation of the feeder/transfer system and its beam break sensor.
 */
data class FeederState(
    val velocityRps: Double = 0.0,
    val targetVelocityRps: Double = 0.0,
    val gamePieceDetected: Boolean = false
)

/**
 * Immutable representation of the fast-climber elevator system.
 */
data class ClimberState(
    val extensionMeters: Double = 0.0,
    val targetExtensionMeters: Double = 0.0,
    val currentAmps: Double = 0.0,
    val targetVoltage: Double = 0.0
)

/**
 * Immutable representation of the floor rollers.
 */
data class FloorState(
    val velocityRps: Double = 0.0,
    val targetVelocityRps: Double = 0.0,
    val currentAmps: Double = 0.0
)

/**
 * Container holding all sub-states specific to Marvin XIX superstructure.
 */
data class MarvinState(
    val flywheel: FlywheelState = FlywheelState(),
    val cowl: CowlState = CowlState(),
    val intake: IntakeState = IntakeState(),
    val feeder: FeederState = FeederState(),
    val climber: ClimberState = ClimberState(),
    val floor: FloorState = FloorState(),
    val slamtakeActive: Boolean = false,
    val slamtakeStartTimeMs: Long = 0L,
    val flywheelActive: Boolean = false,
    val transferActive: Boolean = false,
    val inventoryCount: Int = 0
) : SubsystemState {
    val isFlywheelAtSpeed: Boolean
        get() = flywheel.velocityRpm > 100.0 && Math.abs(flywheel.velocityRpm - flywheel.targetVelocityRpm) < 150.0

    fun withFlywheelSpeed(rpm: Double) = copy(flywheel = flywheel.copy(targetVelocityRpm = rpm))
    fun withCowlAngle(degrees: Double) = copy(cowl = cowl.copy(targetAngleDegrees = degrees))
    fun withIntakePivot(deployed: Boolean) = copy(intake = intake.copy(
        isDeployed = deployed,
        targetAngleDegrees = if (deployed) 90.0 else 0.0
    ))
    fun withIntakeRollers(speedRps: Double) = copy(intake = intake.copy(targetRollerVelocityRps = speedRps))
    fun withFeederSpeed(speedRps: Double) = copy(feeder = feeder.copy(targetVelocityRps = speedRps))
    fun withFloorSpeed(speedRps: Double) = copy(floor = floor.copy(targetVelocityRps = speedRps))
    fun withClimberVoltage(volts: Double) = copy(climber = climber.copy(targetVoltage = volts))
    fun withClimberExtension(meters: Double) = copy(climber = climber.copy(targetExtensionMeters = meters))
}

/**
 * Extension property to retrieve the Marvin-specific superstructure state from the platform custom field.
 */
val SuperstructureState.marvin: MarvinState
    get() = this.custom as? MarvinState ?: MarvinState()
