package com.areslib.frc.reducer

import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import com.areslib.frc.marvin.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue

class MarvinReducerTest {

    @Test
    fun `test SetClimberExtension action updates climber target`() {
        val initialState = RobotState(
            superstructure = SuperstructureState(custom = MarvinState())
        )
        
        // 1. If intake is stowed (pivotAngleDegrees = 0.0), target extension should be clamped to 0.0
        val statePivotStowed = MarvinReducer.reduce(
            initialState,
            SetClimberExtension(0.25, 1000L)
        )
        assertEquals(0.0, statePivotStowed.superstructure.marvin.climber.targetExtensionMeters, "Climber extension must be clamped to 0.0 when pivot is stowed")

        // 2. If intake is deployed (pivotAngleDegrees = 90.0), target extension should be set correctly
        val statePivotDeployed = RobotState(
            superstructure = SuperstructureState().copy(
                custom = MarvinState(
                    intake = IntakeState(pivotAngleDegrees = 90.0, targetAngleDegrees = 90.0)
                )
            )
        )
        val statePivotDeployedUpdated = MarvinReducer.reduce(
            statePivotDeployed,
            SetClimberExtension(0.25, 1000L)
        )
        assertEquals(0.25, statePivotDeployedUpdated.superstructure.marvin.climber.targetExtensionMeters, "Climber extension should set correctly when pivot is deployed")
    }

    @Test
    fun `test coordinated interlocks reciprocal safety boundaries`() {
        // If climber target or physical extension > 0.02, intake pivot target angle must be >= 45.0
        val statePivotStowed = RobotState(
            superstructure = SuperstructureState().copy(
                custom = MarvinState(
                    intake = IntakeState(pivotAngleDegrees = 0.0, targetAngleDegrees = 0.0)
                )
            )
        )

        // When climber is physically extended (sensor update)
        val stateExtendedSensor = MarvinReducer.reduce(
            statePivotStowed,
            SuperstructureSensorUpdate(
                flywheelRpm = 0.0,
                cowlAngle = 0.0,
                intakeAngle = 0.0,
                pieceDetected = false,
                climberExtensionMeters = 0.03,
                timestampMs = 1000L
            )
        )
        assertEquals(45.0, stateExtendedSensor.superstructure.marvin.intake.targetAngleDegrees, "Intake pivot target must be forced to 45.0 when climber is physically extended")
        assertTrue(stateExtendedSensor.superstructure.marvin.intake.isDeployed)

        // When climber target is extended
        val statePivotDeployed = RobotState(
            superstructure = SuperstructureState().copy(
                custom = MarvinState(
                    intake = IntakeState(pivotAngleDegrees = 90.0, targetAngleDegrees = 90.0)
                )
            )
        )
        val stateClimberTargetExtended = MarvinReducer.reduce(
            statePivotDeployed,
            SetClimberExtension(0.1, 1000L)
        )
        // Now if we try to command intake pivot stowed (SetIntakePivot(false) -> targetAngleDegrees = 0.0)
        val statePivotStowAction = MarvinReducer.reduce(
            stateClimberTargetExtended,
            SetIntakePivot(deployed = false, 1100L)
        )
        assertEquals(45.0, statePivotStowAction.superstructure.marvin.intake.targetAngleDegrees, "Intake pivot target must be clamped to 45.0 when climber is commanded extended")
        assertTrue(statePivotStowAction.superstructure.marvin.intake.isDeployed)
    }
}
