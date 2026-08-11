package com.areslib.frc.generatedruntime

import com.areslib.frc.robot.FrcAutonomousSelector
import com.areslib.frc.input.FrcButtonIndex
import com.areslib.frc.input.FrcHidSource
import com.areslib.frc.input.FrcInputFrameAdapter
import com.areslib.input.ControllerBindingRuntime
import com.areslib.input.DigitalBinding
import com.areslib.input.DigitalBindingListener
import com.areslib.input.RawButtonSource
import com.areslib.math.coordinate.CoordinateTransformers
import com.areslib.math.wrapAngle
import com.areslib.routine.AutonomousCatalogEntry
import com.areslib.routine.RoutineAlliance
import com.areslib.routine.RoutinePose
import com.areslib.state.Alliance
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class FrcGeneratedRoutineRuntimeTest {
    @Test
    fun `selector ignores disabled choices and deterministically falls back`() {
        val selector = FrcAutonomousSelector(
            entries = listOf(
                entry("score", order = 1),
                entry("do-nothing", order = 0),
                entry("disabled", order = -1, enabled = false)
            ),
            defaultEntryId = "disabled"
        )

        assertEquals(listOf("do-nothing", "score"), selector.availableEntryIds)
        val missing = selector.resolve("disabled")
        assertEquals("do-nothing", missing.entry.entryId)
        assertTrue(missing.usedFallback)

        val score = selector.resolve("score")
        assertEquals("score", score.entry.entryId)
        assertFalse(score.usedFallback)
    }

    @Test
    fun `opposite alliance transform mirrors only the FRC field X axis`() {
        val source = RoutinePose(2.0, 3.0, 0.4)
        val mirrored = FrcRoutinePoseTransform.apply(
            source,
            authoredAlliance = RoutineAlliance.BLUE,
            activeAlliance = Alliance.RED,
            mirrorForOppositeAlliance = true
        )

        assertEquals(CoordinateTransformers.FRC_FIELD_LENGTH - 2.0, mirrored.x, 1e-9)
        assertEquals(3.0, mirrored.y, 1e-9)
        assertEquals(wrapAngle(Math.PI - 0.4), mirrored.heading.radians, 1e-9)

        val unchanged = FrcRoutinePoseTransform.apply(
            source,
            authoredAlliance = RoutineAlliance.RED,
            activeAlliance = Alliance.RED,
            mirrorForOppositeAlliance = true
        )
        assertEquals(2.0, unchanged.x, 1e-9)
        assertEquals(3.0, unchanged.y, 1e-9)
        assertEquals(0.4, unchanged.heading.radians, 1e-9)
    }

    @Test
    fun `binding host preserves vendor buttons and canonical POV virtual buttons`() {
        val source = FakeHidSource(rawButtonCount = 17, povValue = 0).also {
            it.buttons[16] = true
        }
        var vendorPresses = 0
        var povPresses = 0
        val runtime = ControllerBindingRuntime(
            digitalBindings = listOf(
                DigitalBinding(
                    RawButtonSource(16),
                    listener = object : DigitalBindingListener {
                        override fun onPress() {
                            vendorPresses++
                        }
                    }
                ),
                DigitalBinding(
                    RawButtonSource(FrcButtonIndex.POV_UP),
                    listener = object : DigitalBindingListener {
                        override fun onPress() {
                            povPresses++
                        }
                    }
                )
            ),
            nanoTime = { 100L }
        )
        var afterUpdates = 0
        val host = FrcControllerBindingHost(
            slots = listOf(
                FrcControllerBindingSlot(
                    slotId = "driver",
                    port = 0,
                    runtime = runtime,
                    adapter = FrcInputFrameAdapter(source)
                )
            ),
            afterBindingsUpdate = { afterUpdates++ }
        )

        host.update()

        assertEquals(1, vendorPresses)
        assertEquals(1, povPresses)
        assertEquals(1, afterUpdates)
    }

    @Test
    fun `generated scheme and controller port conventions are deterministic`() {
        assertEquals("competition", selectDefaultGeneratedControlScheme(setOf("competition", "alpha")))
        assertEquals("alpha", selectDefaultGeneratedControlScheme(setOf("practice", "alpha")))
        assertEquals(null, selectDefaultGeneratedControlScheme(emptySet()))
        assertEquals(0, FrcGeneratedControllerPorts.resolve("driver"))
        assertEquals(1, FrcGeneratedControllerPorts.resolve("Operator"))
    }

    private fun entry(id: String, order: Int, enabled: Boolean = true) = AutonomousCatalogEntry(
        entryId = id,
        displayName = id,
        routineId = "do-nothing",
        startingPose = RoutinePose(1.0, 1.0, 0.0),
        sortOrder = order,
        enabled = enabled
    )

    private class FakeHidSource(
        rawButtonCount: Int,
        private val povValue: Int
    ) : FrcHidSource {
        val buttons = BooleanArray(rawButtonCount)

        override fun isConnected(): Boolean = true
        override fun axisCount(): Int = 0
        override fun buttonCount(): Int = buttons.size
        override fun povCount(): Int = 1
        override fun rawAxis(axisIndex: Int): Double = 0.0
        override fun rawButton(buttonNumber: Int): Boolean = buttons[buttonNumber - 1]
        override fun pov(povIndex: Int): Int = povValue
    }
}
