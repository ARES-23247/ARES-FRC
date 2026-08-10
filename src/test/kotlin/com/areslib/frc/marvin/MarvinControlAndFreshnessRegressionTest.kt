package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class MarvinControlAndFreshnessRegressionTest {

    private fun newStore(): Store = Store(
        RobotState(superstructure = SuperstructureState(custom = MarvinState()))
    ) { state, action -> MarvinReducer.reduce(state, action) }

    @Test
    fun `climber mode transitions preserve explicit targets and last command selects output mode`() {
        val initial = RobotState(superstructure = SuperstructureState(custom = MarvinState()))

        val positionMode = MarvinReducer.reduce(
            initial,
            SetClimberPositionRotations(rotations = 0.75, timestampMs = 1_000L)
        )
        assertEquals(ClimberControlMode.POSITION_ROTATIONS, positionMode.superstructure.marvin.climber.controlMode)
        assertEquals(0.75, positionMode.superstructure.marvin.climber.targetPositionRotations)

        val voltageMode = MarvinReducer.reduce(
            positionMode,
            SetClimberVoltage(volts = -4.0, timestampMs = 1_020L)
        )
        assertEquals(ClimberControlMode.VOLTAGE, voltageMode.superstructure.marvin.climber.controlMode)
        assertEquals(-4.0, voltageMode.superstructure.marvin.climber.targetVoltage)
        assertEquals(
            0.75,
            voltageMode.superstructure.marvin.climber.targetPositionRotations,
            "Changing mode must not reinterpret or erase the calibrated mechanism-rotation target"
        )

        val positionModeAgain = MarvinReducer.reduce(
            voltageMode,
            SetClimberPositionRotations(rotations = 0.25, timestampMs = 1_040L)
        )
        assertEquals(ClimberControlMode.POSITION_ROTATIONS, positionModeAgain.superstructure.marvin.climber.controlMode)
        assertEquals(0.25, positionModeAgain.superstructure.marvin.climber.targetPositionRotations)
        assertEquals(-4.0, positionModeAgain.superstructure.marvin.climber.targetVoltage)
    }

    @Test
    fun `flywheel freshness and heading interlocks fail closed then recover`() {
        val store = newStore()
        val flywheel = MarvinFlywheelController(store)
        val feeder = MarvinFeederController(store)

        store.dispatch(SetFlywheelSpeed(4_000.0, 1_000L))
        store.dispatch(
            SuperstructureSensorUpdate(
                flywheelRpm = 4_000.0,
                cowlAngleRotations = 0.0,
                intakeAngle = 0.0,
                pieceDetected = false,
                flywheelVelocityValid = true,
                timestampMs = 1_000L
            )
        )

        assertTrue(flywheel.isRpmAligned(4_000.0))
        feeder.updateFeeders(rpmAligned = true, headingAligned = false, runFloorRollers = true)
        assertEquals(0.0, store.state.superstructure.marvin.feeder.targetVelocityRps)
        assertEquals(0.0, store.state.superstructure.marvin.floor.targetVelocityRps)

        feeder.updateFeeders(rpmAligned = true, headingAligned = true, runFloorRollers = true)
        assertEquals(MarvinConfig.FEEDER_SHOOT_SPEED_RPS, store.state.superstructure.marvin.feeder.targetVelocityRps)
        assertEquals(MarvinConfig.FEEDER_SHOOT_SPEED_RPS, store.state.superstructure.marvin.floor.targetVelocityRps)

        // A failed refresh can carry the same numeric sample as the last good loop.
        // Validity must still force the observation to zero and close the feeder gate.
        store.dispatch(
            SuperstructureSensorUpdate(
                flywheelRpm = 4_000.0,
                cowlAngleRotations = 0.0,
                intakeAngle = 0.0,
                pieceDetected = false,
                flywheelVelocityValid = false,
                timestampMs = 1_020L
            )
        )

        assertFalse(store.state.superstructure.marvin.flywheel.velocityValid)
        assertEquals(0.0, store.state.superstructure.marvin.flywheel.velocityRpm)
        assertFalse(flywheel.isRpmAligned(4_000.0))
        feeder.updateFeeders(
            rpmAligned = flywheel.isRpmAligned(4_000.0),
            headingAligned = true,
            runFloorRollers = true
        )
        assertEquals(0.0, store.state.superstructure.marvin.feeder.targetVelocityRps)
        assertEquals(0.0, store.state.superstructure.marvin.floor.targetVelocityRps)

        store.dispatch(
            SuperstructureSensorUpdate(
                flywheelRpm = 4_000.0,
                cowlAngleRotations = 0.0,
                intakeAngle = 0.0,
                pieceDetected = false,
                flywheelVelocityValid = true,
                timestampMs = 1_040L
            )
        )
        assertTrue(flywheel.isRpmAligned(4_000.0))
        feeder.updateFeeders(
            rpmAligned = flywheel.isRpmAligned(4_000.0),
            headingAligned = true,
            runFloorRollers = false
        )
        assertEquals(MarvinConfig.FEEDER_SHOOT_SPEED_RPS, store.state.superstructure.marvin.feeder.targetVelocityRps)
        assertEquals(0.0, store.state.superstructure.marvin.floor.targetVelocityRps)
    }
}
