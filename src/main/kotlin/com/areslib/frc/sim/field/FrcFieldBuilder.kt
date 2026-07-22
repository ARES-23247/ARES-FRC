package com.areslib.frc.sim.field

import org.dyn4j.dynamics.Body
import org.dyn4j.geometry.Geometry
import org.dyn4j.geometry.MassType
import org.dyn4j.world.World
/**
 * Documentation for FrcFieldBuilder
 */

object FrcFieldBuilder {
    /**
     * Documentation for buildFrcField
     */

    fun buildFrcField(world: World<Body>) {
        /**
         * Documentation for width
         */
        val width = 16.541
        /**
         * Documentation for height
         */
        val height = 8.069

        // Outer bounds
        addWall(world, width / 2.0, height, width, 0.1)   // Top
        addWall(world, width / 2.0, 0.0, width, 0.1)      // Bottom
        addWall(world, 0.0, height / 2.0, 0.1, height)     // Left
        addWall(world, width, height / 2.0, 0.1, height)   // Right

        // Hubs (Static scoring centers)
        addWall(world, 4.135, 4.0345, 1.1938, 1.1938)      // Blue Hub
        addWall(world, width - 4.135, 4.0345, 1.1938, 1.1938) // Red Hub

        // Towers (Climbing truss frames or shield generator columns)
        addWall(world, width / 2.0 - 1.8, height / 2.0 - 1.8, 0.3, 0.3) // bottom-left tower
        addWall(world, width / 2.0 - 1.8, height / 2.0 + 1.8, 0.3, 0.3) // top-left tower
        addWall(world, width / 2.0 + 1.8, height / 2.0 - 1.8, 0.3, 0.3) // bottom-right tower
        addWall(world, width / 2.0 + 1.8, height / 2.0 + 1.8, 0.3, 0.3) // top-right tower

        // Trench Barriers (Long horizontal boundaries parallel to side walls forming high-speed driving lanes)
        addWall(world, width / 2.0, 1.45, 3.2, 0.15)      // Bottom Trench Wall
        addWall(world, width / 2.0, height - 1.45, 3.2, 0.15) // Top Trench Wall

        // Climb Ramps / Stations (Raised climb base blocks at side ends)
        addWall(world, 2.5, height / 2.0, 0.6, 1.4)       // Blue Climb Base
        addWall(world, width - 2.5, height / 2.0, 0.6, 1.4) // Red Climb Base
    }
    /**
     * Documentation for buildWorldWalls
     */

    fun buildWorldWalls(world: World<Body>, width: Double, height: Double) {
        addWall(world, width / 2.0, height, width, 0.1)   // Top
        addWall(world, width / 2.0, 0.0, width, 0.1)      // Bottom
        addWall(world, 0.0, height / 2.0, 0.1, height)     // Left
        addWall(world, width, height / 2.0, 0.1, height)   // Right
    }

    private fun addWall(world: World<Body>, x: Double, y: Double, w: Double, h: Double) {
        /**
         * Documentation for wall
         */
        val wall = Body()
        wall.addFixture(Geometry.createRectangle(w, h))
        wall.setMass(MassType.INFINITE)
        wall.translate(x, y)
        world.addBody(wall)
    }
}
