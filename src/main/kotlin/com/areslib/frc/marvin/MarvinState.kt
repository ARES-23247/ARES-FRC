package com.areslib.frc.marvin

import com.areslib.state.SubsystemState
import com.areslib.state.SuperstructureState

/**
 * Immutable representation of the dual-motor shooter flywheel state.
 */
data class FlywheelState(
    /**
     * Documentation for velocityRpm
     */
    val velocityRpm: Double = 0.0,
    /**
     * Documentation for targetVelocityRpm
     */
    val targetVelocityRpm: Double = 0.0,
    /**
     * Documentation for currentAmps
     */
    val currentAmps: Double = 0.0,
    /**
     * Documentation for tempCelsius
     */
    val tempCelsius: Double = 0.0
)

/**
 * Immutable representation of the adjustable cowl/hood angle state.
 */
data class CowlState(
    /**
     * Documentation for angleRotations
     */
    val angleRotations: Double = 0.0,
    /**
     * Documentation for targetAngleRotations
     */
    val targetAngleRotations: Double = 0.0,
    /**
     * Documentation for currentAmps
     */
    val currentAmps: Double = 0.0
)

/**
 * Immutable representation of the active pivot-arm intake and roller state.
 */
data class IntakeState(
    /**
     * Documentation for pivotAngleDegrees
     */
    val pivotAngleDegrees: Double = 0.0,
    /**
     * Documentation for targetAngleDegrees
     */
    val targetAngleDegrees: Double = 0.0,
    /**
     * Documentation for rollerVelocityRps
     */
    val rollerVelocityRps: Double = 0.0,
    /**
     * Documentation for targetRollerVelocityRps
     */
    val targetRollerVelocityRps: Double = 0.0,
    /**
     * Documentation for isDeployed
     */
    val isDeployed: Boolean = false
)

/**
 * Immutable representation of the feeder/transfer system and its beam break sensor.
 */
data class FeederState(
    /**
     * Documentation for velocityRps
     */
    val velocityRps: Double = 0.0,
    /**
     * Documentation for targetVelocityRps
     */
    val targetVelocityRps: Double = 0.0,
    /**
     * Documentation for gamePieceDetected
     */
    val gamePieceDetected: Boolean = false
)

/**
 * Immutable representation of the fast-climber elevator system.
 */
data class ClimberState(
    /**
     * Documentation for extensionMeters
     */
    val extensionMeters: Double = 0.0,
    /**
     * Documentation for targetExtensionMeters
     */
    val targetExtensionMeters: Double = 0.0,
    /**
     * Documentation for currentAmps
     */
    val currentAmps: Double = 0.0,
    /**
     * Documentation for targetVoltage
     */
    val targetVoltage: Double = 0.0
)

/**
 * Immutable representation of the floor rollers.
 */
data class FloorState(
    /**
     * Documentation for velocityRps
     */
    val velocityRps: Double = 0.0,
    /**
     * Documentation for targetVelocityRps
     */
    val targetVelocityRps: Double = 0.0,
    /**
     * Documentation for currentAmps
     */
    val currentAmps: Double = 0.0
)

/**
 * Container holding all sub-states specific to Marvin XIX superstructure.
 */
data class MarvinState(
    /**
     * Documentation for flywheel
     */
    val flywheel: FlywheelState = FlywheelState(),
    /**
     * Documentation for cowl
     */
    val cowl: CowlState = CowlState(),
    /**
     * Documentation for intake
     */
    val intake: IntakeState = IntakeState(),
    /**
     * Documentation for feeder
     */
    val feeder: FeederState = FeederState(),
    /**
     * Documentation for climber
     */
    val climber: ClimberState = ClimberState(),
    /**
     * Documentation for floor
     */
    val floor: FloorState = FloorState(),
    /**
     * Documentation for slamtakeActive
     */
    val slamtakeActive: Boolean = false,
    /**
     * Documentation for slamtakeStartTimeMs
     */
    val slamtakeStartTimeMs: Long = 0L,
    /**
     * Documentation for flywheelActive
     */
    val flywheelActive: Boolean = false,
    /**
     * Documentation for transferActive
     */
    val transferActive: Boolean = false,
    /**
     * Documentation for inventoryCount
     */
    val inventoryCount: Int = 0
) : SubsystemState {
    /**
     * Documentation for isFlywheelAtSpeed
     */
    val isFlywheelAtSpeed: Boolean
        get() = flywheel.velocityRpm > 100.0 && Math.abs(flywheel.velocityRpm - flywheel.targetVelocityRpm) < 150.0
    /**
     * Documentation for withFlywheelSpeed
     */

    fun withFlywheelSpeed(rpm: Double) = copy(flywheel = flywheel.copy(targetVelocityRpm = rpm))
    /**
     * Documentation for withCowlAngle
     */
    fun withCowlAngle(rotations: Double) = copy(cowl = cowl.copy(targetAngleRotations = rotations))
    /**
     * Documentation for withIntakePivot
     */
    fun withIntakePivot(deployed: Boolean) = copy(intake = intake.copy(
        isDeployed = deployed,
        targetAngleDegrees = if (deployed) 90.0 else 0.0
    ))
    /**
     * Documentation for withIntakeRollers
     */
    fun withIntakeRollers(speedRps: Double) = copy(intake = intake.copy(targetRollerVelocityRps = speedRps))
    /**
     * Documentation for withFeederSpeed
     */
    fun withFeederSpeed(speedRps: Double) = copy(feeder = feeder.copy(targetVelocityRps = speedRps))
    /**
     * Documentation for withFloorSpeed
     */
    fun withFloorSpeed(speedRps: Double) = copy(floor = floor.copy(targetVelocityRps = speedRps))
    /**
     * Documentation for withClimberVoltage
     */
    fun withClimberVoltage(volts: Double) = copy(climber = climber.copy(targetVoltage = volts))
    /**
     * Documentation for withClimberExtension
     */
    fun withClimberExtension(meters: Double) = copy(climber = climber.copy(targetExtensionMeters = meters))
}

/**
 * Extension property to retrieve the Marvin-specific superstructure state from the platform custom field.
 */
val SuperstructureState.marvin: MarvinState
    get() = this.custom as? MarvinState ?: MarvinState()
