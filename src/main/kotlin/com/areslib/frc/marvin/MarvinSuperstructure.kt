package com.areslib.frc.marvin

import com.areslib.subsystem.Subsystem
import com.areslib.Store
import com.areslib.state.RobotState
import com.areslib.frc.hardware.FlywheelIO
import com.areslib.frc.hardware.CowlIO
import com.areslib.frc.hardware.IntakeIO
import com.areslib.frc.hardware.FeederIO
import com.areslib.frc.hardware.FloorIO
import com.areslib.frc.hardware.ClimberIO

/**
 * Season-specific subsystem implementation managing the Marvin 19 superstructure hardware.
 *
 * Implements [Subsystem] to register with the robot lifecycle, reading hardware sensors,
 * dispatching [SuperstructureSensorUpdate] actions, and applying voltage/closed-loop outputs
 * with brownout power scaling applied. ARESLib refreshes registered IO before [readSensors],
 * so this class consumes cached observations only; it never initiates hardware reads.
 *
 * Invalid flywheel velocity and absent/untrusted piece detection stay explicit in the
 * sensor action. Position targets retain their geometry during brownout, while velocity,
 * voltage, and closed-loop effort are scaled.
 */
class MarvinSuperstructure(
    val flywheelIO: FlywheelIO,
    val cowlIO: CowlIO,
    val intakeIO: IntakeIO,
    val feederIO: FeederIO,
    val floorIO: FloorIO,
    val climberIO: ClimberIO
) : Subsystem {

    /** Dispatches one coherent cached sensor snapshot and advances slamtake timers. */
    override fun readSensors(store: Store, timestampMs: Long) {
        val pieceDetectionValid = feederIO.pieceDetectionValid
        val pieceDetected = pieceDetectionValid && feederIO.isBeamBroken
        store.dispatch(SuperstructureSensorUpdate(
            flywheelRpm = flywheelIO.velocityRpm,
            flywheelVelocityValid = flywheelIO.velocityValid,
            cowlAngleRotations = cowlIO.angleRotations,
            intakeAngle = intakeIO.pivotAngleDegrees,
            pieceDetected = pieceDetected,
            pieceDetectionValid = pieceDetectionValid,
            floorVelocityRps = floorIO.velocityRps,
            climberPositionRotations = climberIO.positionRotations,
            floorCurrentAmps = floorIO.currentAmps,
            timestampMs = timestampMs
        ))
        
        val marvin = store.state.superstructure.marvin
        if (!marvin.slamtakeActive || (pieceDetectionValid && pieceDetected)) return

        val elapsed = (timestampMs - marvin.slamtakeStartTimeMs) / 1000.0
        when (marvin.slamtakePhase) {
            DEPLOYED_PHASE -> if (elapsed >= RETRACT_AT_SECONDS) {
                store.dispatch(SlamtakeTimerExpired(1, timestampMs))
            }
            RETRACTED_PHASE -> if (elapsed >= FINISH_AT_SECONDS) {
                store.dispatch(SlamtakeTimerExpired(2, timestampMs))
            }
        }
    }

    /** Emits outputs from immutable state using [scale] as effort/velocity power budget. */
    override fun writeOutputs(state: RobotState, scale: Double) {
        val marvin = state.superstructure.marvin
        val flywheelTargetRpm = if (marvin.flywheelActive) marvin.flywheel.targetVelocityRpm * scale else 0.0
        flywheelIO.setVelocityRpm(flywheelTargetRpm)
        // Position targets describe mechanism geometry and must not move when
        // brownout scaling changes. Velocity and voltage commands are scaled below.
        cowlIO.setTargetAngle(marvin.cowl.targetAngleRotations, scale)

        val pivotAngle = marvin.intake.targetAngleDegrees
        intakeIO.setPivotAngle(pivotAngle, scale)

        val targetRollerSpeed = marvin.intake.targetRollerVelocityRps * scale
        intakeIO.setRollerVelocityRps(targetRollerSpeed)

        val targetFeederSpeed = marvin.feeder.targetVelocityRps
        feederIO.setAppliedVoltage(FEEDER_KV_VOLTS_PER_RPS * targetFeederSpeed * scale)

        val targetFloorSpeed = marvin.floor.targetVelocityRps
        floorIO.setAppliedVoltage(FLOOR_KV_VOLTS_PER_RPS * targetFloorSpeed * scale)

        when (marvin.climber.controlMode) {
            ClimberControlMode.VOLTAGE -> climberIO.setAppliedVoltage(marvin.climber.targetVoltage * scale)
            ClimberControlMode.POSITION_ROTATIONS -> climberIO.setTargetPositionRotations(
                marvin.climber.targetPositionRotations,
                scale
            )
        }
    }

    companion object {
        private const val DEPLOYED_PHASE = 1
        private const val RETRACTED_PHASE = 2
        private const val RETRACT_AT_SECONDS = 0.5
        private const val FINISH_AT_SECONDS = 1.5
        private const val FEEDER_KV_VOLTS_PER_RPS = 0.12
        private const val FLOOR_KV_VOLTS_PER_RPS = 0.12
    }
}
