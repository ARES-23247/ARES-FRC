package com.areslib.frc.marvin

import com.areslib.subsystem.Subsystem
import com.areslib.subsystem.Store
import com.areslib.state.RobotState
import com.areslib.hardware.FlywheelIO
import com.areslib.hardware.CowlIO
import com.areslib.hardware.IntakeIO
import com.areslib.hardware.FeederIO
import com.areslib.hardware.FloorIO
import com.areslib.hardware.ClimberIO

/**
 * Season-specific subsystem implementation managing the Marvin 19 superstructure hardware.
 *
 * Implements [Subsystem] to register with the robot lifecycle, reading hardware sensors,
 * dispatching [SuperstructureSensorUpdate] actions, and applying voltage/closed-loop outputs
 * with brownout power scaling applied.
 */
class MarvinSuperstructure(
    val flywheelIO: FlywheelIO,
    val cowlIO: CowlIO,
    val intakeIO: IntakeIO,
    val feederIO: FeederIO,
    val floorIO: FloorIO,
    val climberIO: ClimberIO
) : Subsystem {

    override fun readSensors(store: Store, timestampMs: Long) {
        store.dispatch(SuperstructureSensorUpdate(
            flywheelRpm = flywheelIO.velocityRpm,
            cowlAngle = cowlIO.angleDegrees,
            intakeAngle = intakeIO.pivotAngleDegrees,
            pieceDetected = feederIO.isBeamBroken,
            floorVelocityRps = floorIO.velocityRps,
            climberExtensionMeters = climberIO.extensionMeters,
            timestampMs = timestampMs
        ))
    }

    override fun writeOutputs(state: RobotState, scale: Double) {
        val marvin = state.superstructure.marvin
        flywheelIO.setVelocityRpm(marvin.flywheel.targetVelocityRpm * scale)
        cowlIO.setTargetAngle(marvin.cowl.targetAngleDegrees)

        val pivotAngle = marvin.intake.targetAngleDegrees
        intakeIO.setPivotAngle(pivotAngle)

        val targetRollerSpeed = marvin.intake.targetRollerVelocityRps
        intakeIO.setRollerVoltage((targetRollerSpeed / 10.0) * 12.0 * scale)

        val targetFeederSpeed = marvin.feeder.targetVelocityRps
        feederIO.setAppliedVoltage((targetFeederSpeed / 12.0) * 12.0 * scale)

        val targetFloorSpeed = marvin.floor.targetVelocityRps
        floorIO.setAppliedVoltage((targetFloorSpeed / 12.0) * 12.0 * scale)

        val targetClimberVoltage = marvin.climber.targetVoltage
        climberIO.setAppliedVoltage(targetClimberVoltage * scale)
    }


}
