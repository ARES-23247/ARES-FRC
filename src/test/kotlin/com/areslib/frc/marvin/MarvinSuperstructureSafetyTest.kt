package com.areslib.frc.marvin

import com.areslib.frc.hardware.ClimberIO
import com.areslib.frc.hardware.CowlIO
import com.areslib.frc.hardware.FeederIO
import com.areslib.frc.hardware.FloorIO
import com.areslib.frc.hardware.FlywheelIO
import com.areslib.frc.hardware.IntakeIO
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class MarvinSuperstructureSafetyTest {

    private class RecordingFlywheelIO : FlywheelIO {
        var velocityRpmCommand = Double.NaN
        override fun setVelocityRpm(rpm: Double) { velocityRpmCommand = rpm }
        override fun setAppliedVoltage(volts: Double) = Unit
    }

    private class RecordingCowlIO : CowlIO {
        var angleCommand = Double.NaN
        var effortScale = Double.NaN
        override fun setTargetAngle(rotations: Double) { angleCommand = rotations }
        override fun setTargetAngle(rotations: Double, maxEffortScale: Double) {
            angleCommand = rotations
            effortScale = maxEffortScale
        }
        override fun setAppliedVoltage(volts: Double) = Unit
    }

    private class RecordingIntakeIO : IntakeIO {
        var pivotAngleCommand = Double.NaN
        var rollerVelocityCommand = Double.NaN
        var pivotEffortScale = Double.NaN
        override fun setPivotAngle(degrees: Double) { pivotAngleCommand = degrees }
        override fun setPivotAngle(degrees: Double, maxEffortScale: Double) {
            pivotAngleCommand = degrees
            pivotEffortScale = maxEffortScale
        }
        override fun setPivotVoltage(volts: Double) = Unit
        override fun setRollerVoltage(volts: Double) = Unit
        override fun setRollerVelocityRps(rps: Double) { rollerVelocityCommand = rps }
    }

    private class RecordingFeederIO : FeederIO {
        var voltageCommand = Double.NaN
        override fun setAppliedVoltage(volts: Double) { voltageCommand = volts }
    }

    private class RecordingFloorIO : FloorIO {
        var voltageCommand = Double.NaN
        override fun setAppliedVoltage(volts: Double) { voltageCommand = volts }
    }

    private class RecordingClimberIO : ClimberIO {
        var voltageCommand = Double.NaN
        var positionCommandRotations = Double.NaN
        var effortScale = Double.NaN
        override fun setTargetPositionRotations(rotations: Double) {
            positionCommandRotations = rotations
        }
        override fun setTargetPositionRotations(rotations: Double, maxEffortScale: Double) {
            positionCommandRotations = rotations
            effortScale = maxEffortScale
        }
        override fun setAppliedVoltage(volts: Double) { voltageCommand = volts }
    }

    @Test
    fun brownoutPreservesPositionTargetsWhileScalingVelocityAndVoltage() {
        val flywheel = RecordingFlywheelIO()
        val cowl = RecordingCowlIO()
        val intake = RecordingIntakeIO()
        val feeder = RecordingFeederIO()
        val floor = RecordingFloorIO()
        val climber = RecordingClimberIO()
        val subsystem = MarvinSuperstructure(flywheel, cowl, intake, feeder, floor, climber)
        val state = RobotState(
            superstructure = SuperstructureState(
                custom = MarvinState(
                    flywheel = FlywheelState(targetVelocityRpm = 4_000.0),
                    flywheelActive = true,
                    cowl = CowlState(targetAngleRotations = 1.25),
                    intake = IntakeState(targetAngleDegrees = 90.0, targetRollerVelocityRps = 10.0),
                    feeder = FeederState(targetVelocityRps = 8.0),
                    floor = FloorState(targetVelocityRps = 6.0),
                    climber = ClimberState(targetVoltage = 10.0)
                )
            )
        )

        subsystem.writeOutputs(state, 0.4)

        assertEquals(1.25, cowl.angleCommand)
        assertEquals(0.4, cowl.effortScale)
        assertEquals(90.0, intake.pivotAngleCommand)
        assertEquals(0.4, intake.pivotEffortScale)
        assertEquals(1_600.0, flywheel.velocityRpmCommand)
        assertEquals(4.0, intake.rollerVelocityCommand)
        assertEquals(0.384, feeder.voltageCommand)
        assertEquals(0.288, floor.voltageCommand)
        assertEquals(4.0, climber.voltageCommand)

        assertEquals(1.25, state.superstructure.marvin.cowl.targetAngleRotations)
        assertEquals(90.0, state.superstructure.marvin.intake.targetAngleDegrees)
    }

    @Test
    fun climberPositionModeKeepsRotationsExplicitAndLimitsEffort() {
        val climber = RecordingClimberIO()
        val subsystem = MarvinSuperstructure(
            RecordingFlywheelIO(), RecordingCowlIO(), RecordingIntakeIO(),
            RecordingFeederIO(), RecordingFloorIO(), climber
        )
        val state = RobotState(
            superstructure = SuperstructureState(
                custom = MarvinState(
                    climber = ClimberState(
                        controlMode = ClimberControlMode.POSITION_ROTATIONS,
                        targetPositionRotations = 7.5
                    )
                )
            )
        )

        subsystem.writeOutputs(state, 0.35)

        assertEquals(7.5, climber.positionCommandRotations)
        assertEquals(0.35, climber.effortScale)
        assertTrue(climber.voltageCommand.isNaN())
    }
}
