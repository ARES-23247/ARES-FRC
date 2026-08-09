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
        store.dispatch(SuperstructureSensorUpdate(
            flywheelRpm = flywheelIO.velocityRpm,
            cowlAngleRotations = cowlIO.angleRotations,
            intakeAngle = intakeIO.pivotAngleDegrees,
            pieceDetected = feederIO.isBeamBroken,
            floorVelocityRps = floorIO.velocityRps,
            climberExtensionMeters = climberIO.extensionMeters,
            floorCurrentAmps = floorIO.currentAmps,
            timestampMs = timestampMs
        ))
        
        val marvin = store.state.superstructure.marvin
        if (marvin.slamtakeActive && !feederIO.isBeamBroken) {
            val elapsed = (timestampMs - marvin.slamtakeStartTimeMs) / 1000.0
            if (elapsed >= 1.5 && marvin.intake.targetAngleDegrees == 0.0) {
                store.dispatch(SlamtakeTimerExpired(2, timestampMs))
            } else if (elapsed >= 0.5 && elapsed < 1.5 && marvin.intake.targetAngleDegrees == 90.0) {
                store.dispatch(SlamtakeTimerExpired(1, timestampMs))
            }
        }
    }

    override fun writeOutputs(state: RobotState, scale: Double) {
        /**
         * Documentation for marvin
         */
        val marvin = state.superstructure.marvin
        flywheelIO.setVelocityRpm(marvin.flywheel.targetVelocityRpm * scale)
        cowlIO.setTargetAngle(marvin.cowl.targetAngleRotations * scale)
        /**
         * Documentation for pivotAngle
         */

        val pivotAngle = marvin.intake.targetAngleDegrees * scale
        intakeIO.setPivotAngle(pivotAngle)
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

        val targetClimberVoltage = marvin.climber.targetVoltage
        climberIO.setAppliedVoltage(targetClimberVoltage * scale)
    }


}
