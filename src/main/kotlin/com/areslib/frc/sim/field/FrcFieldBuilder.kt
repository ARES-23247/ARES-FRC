package com.areslib.frc.sim.field

import org.dyn4j.dynamics.Body
import org.dyn4j.geometry.Geometry
import org.dyn4j.geometry.MassType
import org.dyn4j.world.World

/** Builds static Dyn4j collision bodies in blue-origin field meters. */
object FrcFieldBuilder {

    /**
     * Builds the season simulation layout using canonical FRC field extents.
     *
     * Interior bodies approximate the six 2024 Crescendo Stage uprights using their official
     * AprilTag anchor coordinates. Speakers, amps, and sources are integrated into the perimeter
     * and are not modeled as fictitious square obstacles inside the playable field.
     */
    fun buildFrcField(world: World<Body>) {
        val width = com.areslib.math.coordinate.CoordinateTransformers.FRC_FIELD_LENGTH
        val height = com.areslib.math.coordinate.CoordinateTransformers.FRC_FIELD_WIDTH

        // Outer bounds
        addWall(world, width / 2.0, height, width, 0.1)   // Top
        addWall(world, width / 2.0, 0.0, width, 0.1)      // Bottom
        addWall(world, 0.0, height / 2.0, 0.1, height)     // Left
        addWall(world, width, height / 2.0, 0.1, height)   // Right

        // Blue Stage uprights (AprilTags 14, 15, 16) and Red Stage uprights (11, 12, 13).
        addStagePost(world, 5.3208, 4.1051)
        addStagePost(world, 4.6413, 4.4983)
        addStagePost(world, 4.6413, 3.7132)
        addStagePost(world, 11.9047, 3.7132)
        addStagePost(world, 11.9047, 4.4983)
        addStagePost(world, 11.2202, 4.1051)
    }

    /** Adds only an axis-aligned boundary of [width] by [height] meters. */
    fun buildWorldWalls(world: World<Body>, width: Double, height: Double) {
        addWall(world, width / 2.0, height, width, 0.1)   // Top
        addWall(world, width / 2.0, 0.0, width, 0.1)      // Bottom
        addWall(world, 0.0, height / 2.0, 0.1, height)     // Left
        addWall(world, width, height / 2.0, 0.1, height)   // Right
    }

    private fun addWall(world: World<Body>, x: Double, y: Double, w: Double, h: Double) {
        val wall = Body()
        wall.addFixture(Geometry.createRectangle(w, h))
        wall.setMass(MassType.INFINITE)
        wall.translate(x, y)
        world.addBody(wall)
    }

    private fun addStagePost(world: World<Body>, x: Double, y: Double) {
        val post = Body()
        post.addFixture(Geometry.createCircle(STAGE_POST_RADIUS_METERS))
        post.setMass(MassType.INFINITE)
        post.translate(x, y)
        world.addBody(post)
    }

    private const val STAGE_POST_RADIUS_METERS = 0.18
}
