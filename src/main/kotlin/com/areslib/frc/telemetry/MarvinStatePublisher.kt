package com.areslib.frc.telemetry

import com.areslib.state.RobotState
import com.areslib.telemetry.ITelemetry
import com.areslib.frc.state.marvinXIX

object MarvinStatePublisher {
    fun publish(state: RobotState, telemetry: ITelemetry) {
        val marvin = state.superstructure.marvinXIX
        telemetry.putNumber("Superstructure/Flywheel/Velocity", marvin.flywheel.velocityRpm)
        telemetry.putNumber("Superstructure/Flywheel/TargetVelocity", marvin.flywheel.targetVelocityRpm)
        telemetry.putNumber("Superstructure/Cowl/Angle", marvin.cowl.angleDegrees)
        telemetry.putNumber("Superstructure/Cowl/TargetAngle", marvin.cowl.targetAngleDegrees)
        telemetry.putNumber("Superstructure/Intake/PivotAngle", marvin.intake.pivotAngleDegrees)
        telemetry.putNumber("Superstructure/Intake/TargetAngle", marvin.intake.targetAngleDegrees)
        telemetry.putBoolean("Superstructure/Feeder/PieceDetected", marvin.feeder.gamePieceDetected)
        telemetry.putNumber("Superstructure/Climber/Voltage", marvin.climber.targetVoltage)
        telemetry.putNumber("Superstructure/Climber/Extension", marvin.climber.extensionMeters)
        telemetry.putNumber("Superstructure/Climber/TargetExtension", marvin.climber.targetExtensionMeters)
        telemetry.putNumber("Superstructure/Floor/Velocity", marvin.floor.velocityRps)
    }
}
