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
    /** Whether the velocity observation is fresh and valid this loop. */
    val velocityValid: Boolean = false,
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
    val gamePieceDetected: Boolean = false,
    /** Whether a real or explicitly configured simulated detector exists. */
    val pieceDetectionValid: Boolean = false,
    /**
     * Documentation for previousGamePieceDetected
     */
    val previousGamePieceDetected: Boolean = false
)

/**
 * Immutable representation of the fast-climber elevator system.
 */
enum class ClimberControlMode { VOLTAGE, POSITION_ROTATIONS }

data class ClimberState(
    /** Measured mechanism position in rotations. */
    val positionRotations: Double = 0.0,
    /** Closed-loop mechanism target in rotations. */
    val targetPositionRotations: Double = 0.0,
    /**
     * Documentation for currentAmps
     */
    val currentAmps: Double = 0.0,
    /**
     * Documentation for targetVoltage
     */
    val targetVoltage: Double = 0.0,
    val controlMode: ClimberControlMode = ClimberControlMode.VOLTAGE
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
     * Monotonic slamtake phase counter advanced by elapsed-time thresholds in
     * MarvinSuperstructure.readSensors. Phases: 0 = inactive, 1 = deployed (intake
     * out), 2 = retracted (intake stowed). Using a counter instead of inferring the
     * phase from the intake pivot angle means a skipped [0.5,1.5)s window (loop stall,
     * GC, exception) can no longer deadlock the sequence.
     */
    val slamtakePhase: Int = 0,
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
        get() = flywheel.velocityValid && flywheel.targetVelocityRpm > 100.0 && flywheel.velocityRpm > 100.0 && Math.abs(flywheel.velocityRpm - flywheel.targetVelocityRpm) < 150.0
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
    fun withClimberVoltage(volts: Double) = copy(climber = climber.copy(
        targetVoltage = volts,
        controlMode = ClimberControlMode.VOLTAGE
    ))

    fun withClimberPositionRotations(rotations: Double) = copy(climber = climber.copy(
        targetPositionRotations = rotations,
        controlMode = ClimberControlMode.POSITION_ROTATIONS
    ))
}

/**
 * Extension property to retrieve the Marvin-specific superstructure state from the platform custom field.
 */
val SuperstructureState.marvin: MarvinState
    get() = this.custom as? MarvinState ?: MarvinState()
