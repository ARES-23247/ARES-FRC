package com.areslib.frc.generatedruntime

import com.areslib.action.RobotAction
import com.areslib.frc.robot.FrcAutoCapabilities
import com.areslib.input.InputFrame
import com.areslib.state.RobotState
import org.junit.jupiter.api.Assertions.assertDoesNotThrow
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class FrcGeneratedControlsRuntimeTest {
    @Test
    fun `zero-scheme project is an installed no-op with truthful source`() {
        val dispatched = mutableListOf<RobotAction>()
        val sampler = object : FrcControllerPortSampler {
            override fun prepare(port: Int) = error("No port should be prepared without a scheme")
            override fun sampleInto(port: Int, frame: InputFrame, nowNanos: Long) =
                error("No port should be sampled without a scheme")
        }
        val runtime = FrcGeneratedControlsRuntime(
            stateProvider = { RobotState() },
            dispatch = dispatched::add,
            capabilities = FrcAutoCapabilities,
            portSampler = sampler,
        )

        assertEquals(0, runtime.activeControllerPortCount)
        assertEquals("hardcoded-only", runtime.controlsSource)
        assertDoesNotThrow { runtime.update() }
        assertDoesNotThrow { runtime.cancelAll("test transition") }
        assertEquals(emptyList<RobotAction>(), dispatched)
    }
}
