package com.areslib.frc.sim

import org.dyn4j.dynamics.Body
import org.dyn4j.world.World
import org.dyn4j.geometry.Geometry
import org.dyn4j.geometry.MassType
import org.dyn4j.geometry.Vector2
import com.areslib.frc.FlyingBall
import com.areslib.action.RobotAction
import com.areslib.state.RobotState
/**
 * Documentation for Dyn4jPhysicsWorld
 */

class Dyn4jPhysicsWorld(seed: Long) {
    /**
     * Documentation for world
     */

    val world = World<Body>()
    /**
     * Documentation for robotBody
     */
    val robotBody = Body()
    /**
     * Documentation for balls
     */
    val balls = mutableListOf<Body>()
    /**
     * Documentation for flyingBalls
     */
    val flyingBalls = mutableListOf<FlyingBall>()

    init {
        world.setGravity(Vector2(0.0, 0.0))
        /**
         * Documentation for robotFixture
         */

        val robotFixture = robotBody.addFixture(Geometry.createRectangle(0.7, 0.7))
        robotFixture.density = 78.0
        robotBody.linearDamping = 1.0
        robotBody.angularDamping = 2.0
        robotBody.setMass(MassType.NORMAL)
        robotBody.translate(2.0, 2.0)
        world.addBody(robotBody)

        com.areslib.frc.sim.field.FrcFieldBuilder.buildFrcField(world)
        spawnFuel(seed)
    }
    /**
     * Documentation for step
     */

    fun step(dt: Double) {
        world.step(1, dt)
    }
    /**
     * Documentation for buildWorld
     */

    fun buildWorld(config: com.areslib.state.RobotFieldConfig) {
        /**
         * Documentation for bodies
         */
        val bodies = world.bodies.toList()
        for (body in bodies) {
            if (body != robotBody) {
                world.removeBody(body)
            }
        }
        balls.clear()
        /**
         * Documentation for width
         */

        val width = if (config.fieldType == com.areslib.state.FieldType.FRC) 16.541 else 3.6576
        /**
         * Documentation for height
         */
        val height = if (config.fieldType == com.areslib.state.FieldType.FRC) 8.069 else 3.6576

        com.areslib.frc.sim.field.FrcFieldBuilder.buildWorldWalls(world, width, height)
        com.areslib.sim.field.FieldObstacleLoader.loadObstacles(world, config.obstacles)
        /**
         * Documentation for loadedElements
         */
        
        val loadedElements = com.areslib.sim.field.FieldElementLoader.loadElements(world, config.elementTypes, config.elements)
        balls.addAll(loadedElements)
        println("[FRC Sim] Successfully built world with ${config.obstacles.size} obstacles and ${config.elements.size} elements.")
    }
    /**
     * Documentation for resetPose
     */

    fun resetPose(x: Double, y: Double, heading: Double) {
        robotBody.transform.setTranslation(x, y)
        robotBody.transform.setRotation(heading)
        robotBody.linearVelocity.set(0.0, 0.0)
        robotBody.angularVelocity = 0.0
        robotBody.isAtRest = false
    }

    private fun spawnFuel(seed: Long) {
        /**
         * Documentation for width
         */
        val width = 16.541
        /**
         * Documentation for height
         */
        val height = 8.069
        /**
         * Documentation for ballRadius
         */
        val ballRadius = 0.0635
        /**
         * Documentation for spawnPoints
         */

        val spawnPoints = mutableListOf<Vector2>()
        /**
         * Documentation for rHub
         */

        val rHub = 1.6
        /**
         * Documentation for angles
         */
        val angles = doubleArrayOf(Math.PI / 4.0, 3.0 * Math.PI / 4.0, 5.0 * Math.PI / 4.0, 7.0 * Math.PI / 4.0)
        /**
         * Documentation for blueHubX
         */
        
        val blueHubX = 4.135
        /**
         * Documentation for blueHubY
         */
        val blueHubY = 4.0345
        for (angle in angles) {
            spawnPoints.add(Vector2(blueHubX + rHub * Math.cos(angle), blueHubY + rHub * Math.sin(angle)))
        }
        /**
         * Documentation for redHubX
         */

        val redHubX = width - 4.135
        /**
         * Documentation for redHubY
         */
        val redHubY = 4.0345
        for (angle in angles) {
            spawnPoints.add(Vector2(redHubX + rHub * Math.cos(angle), redHubY + rHub * Math.sin(angle)))
        }
        /**
         * Documentation for bottomTrenchY
         */

        val bottomTrenchY = 0.7
        /**
         * Documentation for bottomTrenchX
         */
        val bottomTrenchX = doubleArrayOf(5.5, 7.0, 8.5, 10.0)
        for (x in bottomTrenchX) {
            spawnPoints.add(Vector2(x, bottomTrenchY))
        }
        /**
         * Documentation for topTrenchY
         */

        val topTrenchY = height - 0.7
        /**
         * Documentation for topTrenchX
         */
        val topTrenchX = doubleArrayOf(6.5, 8.0, 9.5, 11.0)
        for (x in topTrenchX) {
            spawnPoints.add(Vector2(x, topTrenchY))
        }
        /**
         * Documentation for centerX
         */

        val centerX = width / 2.0
        /**
         * Documentation for centerYs
         */
        val centerYs = doubleArrayOf(1.8, 2.4, 3.0, 3.6, 4.4, 5.0, 5.6, 6.2)
        for (y in centerYs) {
            spawnPoints.add(Vector2(centerX, y))
        }

        for (point in spawnPoints) {
            /**
             * Documentation for ball
             */
            val ball = Body()
            /**
             * Documentation for fixture
             */
            val fixture = ball.addFixture(Geometry.createCircle(ballRadius))
            fixture.friction = 0.6
            fixture.restitution = 0.4
            fixture.density = 5.92
            ball.setMass(MassType.NORMAL)
            ball.linearDamping = 2.0
            ball.angularDamping = 2.0
            ball.translate(point.x, point.y)
            world.addBody(ball)
            balls.add(ball)
        }
        println("Spawned exactly ${balls.size} structured cargo/fuel pieces.")
    }
}
