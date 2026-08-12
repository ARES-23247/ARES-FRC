package com.areslib.frc.sim

import org.dyn4j.dynamics.Body
import org.dyn4j.world.World
import org.dyn4j.geometry.Geometry
import org.dyn4j.geometry.MassType
import org.dyn4j.geometry.Vector2
import com.areslib.frc.FlyingBall
import com.areslib.frc.marvin.MarvinConfig
import com.areslib.state.RobotFieldDocument
import java.io.File

/**
 * Owns the Dyn4j world and its mutable body collections.
 *
 * Coordinates are blue-origin field meters with CCW-positive rotation. The robot body is retained
 * when a field configuration is rebuilt; all other bodies are replaced and the grounded-piece
 * index is repopulated. Dyn4j integration must occur through [step] so callers use a single `dt`.
 */
class Dyn4jPhysicsWorld {

    val world = World<Body>()
    val robotBody = Body()
    val balls = mutableListOf<Body>()
    val flyingBalls = mutableListOf<FlyingBall>()

    private val debug = java.lang.Boolean.getBoolean("ares.debug")

    init {
        world.setGravity(Vector2(0.0, 0.0))

        val robotFixture = robotBody.addFixture(
            Geometry.createRectangle(
                MarvinConfig.ROBOT_BUMPER_LENGTH_METERS,
                MarvinConfig.ROBOT_BUMPER_WIDTH_METERS
            )
        )
        robotFixture.density = 78.0
        robotBody.linearDamping = 1.0
        robotBody.angularDamping = 2.0
        robotBody.setMass(MassType.NORMAL)
        robotBody.translate(2.0, 2.0)
        world.addBody(robotBody)

        val fieldFile = listOf(
            File("src/main/deploy/paths/field.json"),
            File("../ARES-FRC/src/main/deploy/paths/field.json")
        ).firstOrNull(File::isFile)
        if (fieldFile != null) {
            buildWorld(RobotFieldDocument.decode(fieldFile.readText()))
        } else {
            com.areslib.frc.sim.field.FrcFieldBuilder.buildWorldWalls(
                world,
                com.areslib.math.coordinate.CoordinateTransformers.FRC_FIELD_LENGTH,
                com.areslib.math.coordinate.CoordinateTransformers.FRC_FIELD_WIDTH
            )
        }
    }

    /** Advances the physics world by [dt] seconds. */
    fun step(dt: Double) {
        world.step(1, dt)
    }

    /** Rebuilds static bodies and game elements from [config], retaining [robotBody]. */
    fun buildWorld(config: com.areslib.state.RobotFieldConfig) {
        val bodies = world.bodies.toList()
        for (body in bodies) {
            if (body != robotBody) {
                world.removeBody(body)
            }
        }
        balls.clear()
        flyingBalls.clear()

        val width = config.resolvedWidthMeters
        val height = config.resolvedHeightMeters

        com.areslib.frc.sim.field.FrcFieldBuilder.buildWorldWalls(world, width, height)
        com.areslib.sim.field.FieldObstacleLoader.loadObstacles(world, config.obstacles)
        
        val loadedElements = com.areslib.sim.field.FieldElementLoader.loadElements(world, config.elementTypes, config.elements)
        balls.addAll(loadedElements)
        if (debug) println("[FRC Sim] Successfully built world with ${config.obstacles.size} obstacles and ${config.elements.size} elements.")
    }

    /** Teleports the robot using field meters and a CCW-positive heading in radians. */
    fun resetPose(x: Double, y: Double, heading: Double) {
        robotBody.transform.setTranslation(x, y)
        robotBody.transform.setRotation(heading)
        robotBody.linearVelocity.set(0.0, 0.0)
        robotBody.angularVelocity = 0.0
        robotBody.isAtRest = false
    }

}
