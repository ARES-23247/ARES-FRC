package com.areslib.frc.sim.field

import com.areslib.math.coordinate.CoordinateTransformers
import org.dyn4j.dynamics.Body
import org.dyn4j.world.World
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class FrcFieldGeometryContractTest {

    @Test
    fun `simulation boundary walls use canonical official FRC field extents`() {
        val world = World<Body>()
        FrcFieldBuilder.buildFrcField(world)

        // FrcFieldBuilder installs top, bottom, left, and right boundary walls first.
        val top = world.getBody(0)
        val bottom = world.getBody(1)
        val left = world.getBody(2)
        val right = world.getBody(3)
        val length = CoordinateTransformers.FRC_FIELD_LENGTH
        val width = CoordinateTransformers.FRC_FIELD_WIDTH

        assertEquals(length / 2.0, top.transform.translationX, 1e-9)
        assertEquals(width, top.transform.translationY, 1e-9)
        assertEquals(length / 2.0, bottom.transform.translationX, 1e-9)
        assertEquals(0.0, bottom.transform.translationY, 1e-9)
        assertEquals(0.0, left.transform.translationX, 1e-9)
        assertEquals(width / 2.0, left.transform.translationY, 1e-9)
        assertEquals(length, right.transform.translationX, 1e-9)
        assertEquals(width / 2.0, right.transform.translationY, 1e-9)
    }
}
