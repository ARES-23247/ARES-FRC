package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class MarvinControllerReduxConsistencyTest {
    @Test
    fun `shooter can stop and re-command the same targets`() {
        val store = Store(
            RobotState(superstructure = SuperstructureState(custom = MarvinState()))
        ) { state, action -> MarvinReducer.reduce(state, action) }
        val shooter = MarvinShooterSubsystem(store)

        shooter.spinUp(4000.0)
        shooter.shoot()
        assertEquals(4000.0, store.state.superstructure.marvin.flywheel.targetVelocityRpm)
        assertTrue(store.state.superstructure.marvin.flywheelActive)
        assertTrue(store.state.superstructure.marvin.transferActive)

        shooter.stop()
        assertEquals(0.0, store.state.superstructure.marvin.flywheel.targetVelocityRpm)
        assertFalse(store.state.superstructure.marvin.flywheelActive)
        assertEquals(0.0, store.state.superstructure.marvin.feeder.targetVelocityRps)
        assertFalse(store.state.superstructure.marvin.transferActive)

        shooter.spinUp(4000.0)
        shooter.shoot()
        assertEquals(4000.0, store.state.superstructure.marvin.flywheel.targetVelocityRpm)
        assertTrue(store.state.superstructure.marvin.flywheelActive)
        assertEquals(MarvinConfig.FEEDER_SHOOT_SPEED_RPS, store.state.superstructure.marvin.feeder.targetVelocityRps)
        assertTrue(store.state.superstructure.marvin.transferActive)
    }
}
