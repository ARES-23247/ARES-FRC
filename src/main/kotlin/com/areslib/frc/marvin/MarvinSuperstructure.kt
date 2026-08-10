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
 * with brownout power scaling applied.
 */
class MarvinSuperstructure(
    /**
     * Documentation for flywheelIO
     */
    val flywheelIO: FlywheelIO,
    /**
     * Documentation for cowlIO
     */
    val cowlIO: CowlIO,
    /**
     * Documentation for intakeIO
     */
    val intakeIO: IntakeIO,
    /**
     * Documentation for feederIO
     */
    val feederIO: FeederIO,
    /**
     * Documentation for floorIO
     */
    val floorIO: FloorIO,
    /**
     * Documentation for climberIO
     */
    val climberIO: ClimberIO
) : Subsystem {

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
        if (marvin.slamtakeActive && (!pieceDetectionValid || !pieceDetected)) {
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
    }

    override fun writeOutputs(state: RobotState, scale: Double) {
        /**
         * Documentation for marvin
         */
        val marvin = state.superstructure.marvin
        val flywheelTargetRpm = if (marvin.flywheelActive) marvin.flywheel.targetVelocityRpm * scale else 0.0
        flywheelIO.setVelocityRpm(flywheelTargetRpm)
        // Position targets describe mechanism geometry and must not move when
        // brownout scaling changes. Velocity and voltage commands are scaled below.
        cowlIO.setTargetAngle(marvin.cowl.targetAngleRotations, scale)
        /**
         * Documentation for pivotAngle
         */

        val pivotAngle = marvin.intake.targetAngleDegrees
        intakeIO.setPivotAngle(pivotAngle, scale)
        /**
         * Documentation for targetRollerSpeed
         */

        val targetRollerSpeed = marvin.intake.targetRollerVelocityRps * scale
        intakeIO.setRollerVelocityRps(targetRollerSpeed)
        /**
         * Documentation for targetFeederSpeed
         */

        val targetFeederSpeed = marvin.feeder.targetVelocityRps
        val FEEDER_KV = 0.12
        feederIO.setAppliedVoltage(FEEDER_KV * targetFeederSpeed * scale)
        /**
         * Documentation for targetFloorSpeed
         */

        val targetFloorSpeed = marvin.floor.targetVelocityRps
        val FLOOR_KV = 0.12
        floorIO.setAppliedVoltage(FLOOR_KV * targetFloorSpeed * scale)
        /**
         * Documentation for targetClimberVoltage
         */

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
    }
}
