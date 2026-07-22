package com.areslib.frc.sim.field

import org.dyn4j.dynamics.Body
import org.dyn4j.world.World
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
/**
 * Documentation for FrcFieldBuilderTest
 */

class FrcFieldBuilderTest {
    /**
     * Documentation for testBuildFrcField
     */

    @Test
    fun testBuildFrcField() {
        /**
         * Documentation for world
         */
        val world = World<Body>()
        assertEquals(0, world.bodyCount)
        FrcFieldBuilder.buildFrcField(world)
        assertTrue(world.bodyCount > 0, "Should have added walls and static bodies to the world")
    }
    /**
     * Documentation for testBuildWorldWalls
     */

    @Test
    fun testBuildWorldWalls() {
        /**
         * Documentation for world
         */
        val world = World<Body>()
        assertEquals(0, world.bodyCount)
        FrcFieldBuilder.buildWorldWalls(world, 10.0, 10.0)
        assertEquals(4, world.bodyCount, "Should have added 4 boundary walls")
    }
}
