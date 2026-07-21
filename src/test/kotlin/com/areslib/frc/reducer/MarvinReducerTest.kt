package com.areslib.frc.reducer

import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import com.areslib.frc.marvin.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.*

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
                cowlAngleRotations = 0.0,
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

    @Test
    fun `test all basic marvin setter actions`() {
        val initialState = RobotState(
            superstructure = SuperstructureState(custom = MarvinState())
        )

        // SetFlywheelSpeed
        val stateFlywheel = MarvinReducer.reduce(initialState, SetFlywheelSpeed(3500.0, 1000L))
        assertEquals(3500.0, stateFlywheel.superstructure.marvin.flywheel.targetVelocityRpm)

        // SetCowlAngle
        val stateCowl = MarvinReducer.reduce(initialState, SetCowlAngle(15.0, 1000L))
        assertEquals(15.0, stateCowl.superstructure.marvin.cowl.targetAngleRotations)

        // SetIntakePivot
        val stateIntakePivot = MarvinReducer.reduce(initialState, SetIntakePivot(true, 1000L))
        assertTrue(stateIntakePivot.superstructure.marvin.intake.isDeployed)
        assertEquals(90.0, stateIntakePivot.superstructure.marvin.intake.targetAngleDegrees)

        // SetIntakeRollers
        val stateIntakeRollers = MarvinReducer.reduce(initialState, SetIntakeRollers(12.5, 1000L))
        assertEquals(12.5, stateIntakeRollers.superstructure.marvin.intake.targetRollerVelocityRps)

        // SetFeederSpeed
        val stateFeeder = MarvinReducer.reduce(initialState, SetFeederSpeed(8.0, 1000L))
        assertEquals(8.0, stateFeeder.superstructure.marvin.feeder.targetVelocityRps)

        // SetFloorSpeed
        val stateFloor = MarvinReducer.reduce(initialState, SetFloorSpeed(9.5, 1000L))
        assertEquals(9.5, stateFloor.superstructure.marvin.floor.targetVelocityRps)

        // SetClimberVoltage
        val stateClimberVoltage = MarvinReducer.reduce(initialState, SetClimberVoltage(11.0, 1000L))
        assertEquals(11.0, stateClimberVoltage.superstructure.marvin.climber.targetVoltage)
    }

    @Test
    fun `test slamtake state machine transitions`() {
        val initialState = RobotState(
            superstructure = SuperstructureState(custom = MarvinState())
        )

        // 1. Start Slamtake
        val stateSlamtakeStart = MarvinReducer.reduce(initialState, StartSlamtake(1000L))
        assertTrue(stateSlamtakeStart.superstructure.marvin.slamtakeActive)
        assertEquals(1000L, stateSlamtakeStart.superstructure.marvin.slamtakeStartTimeMs)

        // 2. Sensor update: elapsed < 0.5s (e.g. 200ms -> 1200ms)
        val stateElapsed0_2 = MarvinReducer.reduce(
            stateSlamtakeStart,
            SuperstructureSensorUpdate(
                flywheelRpm = 0.0,
                cowlAngleRotations = 0.0,
                intakeAngle = 90.0,
                pieceDetected = false,
                climberExtensionMeters = 0.0,
                timestampMs = 1200L
            )
        )
        assertTrue(stateElapsed0_2.superstructure.marvin.slamtakeActive)
        assertTrue(stateElapsed0_2.superstructure.marvin.intake.isDeployed)
        assertEquals(90.0, stateElapsed0_2.superstructure.marvin.intake.targetAngleDegrees)
        assertEquals(10.0, stateElapsed0_2.superstructure.marvin.intake.targetRollerVelocityRps)
        assertEquals(10.0, stateElapsed0_2.superstructure.marvin.floor.targetVelocityRps)
        assertEquals(0.0, stateElapsed0_2.superstructure.marvin.feeder.targetVelocityRps)

        // 3. Sensor update: elapsed < 1.5s (e.g. 1.0s -> 2000ms)
        val stateElapsed1_0 = MarvinReducer.reduce(
            stateSlamtakeStart,
            SuperstructureSensorUpdate(
                flywheelRpm = 0.0,
                cowlAngleRotations = 0.0,
                intakeAngle = 0.0,
                pieceDetected = false,
                climberExtensionMeters = 0.0,
                timestampMs = 2000L
            )
        )
        assertTrue(stateElapsed1_0.superstructure.marvin.slamtakeActive)
        assertFalse(stateElapsed1_0.superstructure.marvin.intake.isDeployed)
        assertEquals(0.0, stateElapsed1_0.superstructure.marvin.intake.targetAngleDegrees)
        assertEquals(10.0, stateElapsed1_0.superstructure.marvin.intake.targetRollerVelocityRps)
        assertEquals(10.0, stateElapsed1_0.superstructure.marvin.floor.targetVelocityRps)

        // 4. Sensor update: elapsed >= 1.5s (e.g. 2.0s -> 3000ms)
        val stateElapsed2_0 = MarvinReducer.reduce(
            stateSlamtakeStart,
            SuperstructureSensorUpdate(
                flywheelRpm = 0.0,
                cowlAngleRotations = 0.0,
                intakeAngle = 0.0,
                pieceDetected = false,
                climberExtensionMeters = 0.0,
                timestampMs = 3000L
            )
        )
        assertFalse(stateElapsed2_0.superstructure.marvin.slamtakeActive, "Slamtake should be disabled after timeout")
        assertEquals(0.0, stateElapsed2_0.superstructure.marvin.intake.targetRollerVelocityRps)
        assertEquals(0.0, stateElapsed2_0.superstructure.marvin.floor.targetVelocityRps)

        // 5. Stop Slamtake action
        val stateSlamtakeStop = MarvinReducer.reduce(stateSlamtakeStart, StopSlamtake(1500L))
        assertFalse(stateSlamtakeStop.superstructure.marvin.slamtakeActive)
    }
}
