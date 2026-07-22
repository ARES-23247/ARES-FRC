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
            timestampMs = timestampMs
        ))
    }

    override fun writeOutputs(state: RobotState, scale: Double) {
        /**
         * Documentation for marvin
         */
        val marvin = state.superstructure.marvin
        flywheelIO.setVelocityRpm(marvin.flywheel.targetVelocityRpm * scale)
        cowlIO.setTargetAngle(marvin.cowl.targetAngleRotations)
        /**
         * Documentation for pivotAngle
         */

        val pivotAngle = marvin.intake.targetAngleDegrees
        intakeIO.setPivotAngle(pivotAngle)
        /**
         * Documentation for targetRollerSpeed
         */

        val targetRollerSpeed = marvin.intake.targetRollerVelocityRps
        intakeIO.setRollerVoltage((targetRollerSpeed / 10.0) * 12.0 * scale)
        /**
         * Documentation for targetFeederSpeed
         */

        val targetFeederSpeed = marvin.feeder.targetVelocityRps
        feederIO.setAppliedVoltage((targetFeederSpeed / 12.0) * 12.0 * scale)
        /**
         * Documentation for targetFloorSpeed
         */

        val targetFloorSpeed = marvin.floor.targetVelocityRps
        floorIO.setAppliedVoltage((targetFloorSpeed / 12.0) * 12.0 * scale)
        /**
         * Documentation for targetClimberVoltage
         */

        val targetClimberVoltage = marvin.climber.targetVoltage
        climberIO.setAppliedVoltage(targetClimberVoltage * scale)
    }


}
