package com.areslib.frc

import com.areslib.action.RobotAction
import com.areslib.frc.hardware.FlywheelIO
import com.areslib.frc.hardware.CowlIO
import com.areslib.frc.hardware.IntakeIO
import com.areslib.frc.hardware.FeederIO
import com.areslib.frc.hardware.FloorIO
import com.areslib.frc.hardware.ClimberIO
import com.areslib.sim.model.FlywheelSim
import com.areslib.sim.model.IntakePivotSim
import com.areslib.state.RobotState
import com.areslib.telemetry.ITelemetry
import com.areslib.frc.marvin.*
import com.areslib.frc.sim.Dyn4jPhysicsWorld
import com.areslib.frc.sim.Dyn4jSimTelemetryPublisher
import com.areslib.frc.sim.Dyn4jSwerveModuleSim
import org.dyn4j.dynamics.Body
import org.dyn4j.geometry.Geometry
import org.dyn4j.geometry.MassType
import org.dyn4j.geometry.Vector2
/**
 * Documentation for FlyingBall
 */

class FlyingBall(
    /**
     * Documentation for x
     */
    var x: Double,
    /**
     * Documentation for y
     */
    var y: Double,
    /**
     * Documentation for z
     */
    var z: Double,
    /**
     * Documentation for vx
     */
    var vx: Double,
    /**
     * Documentation for vy
     */
    var vy: Double,
    /**
     * Documentation for vz
     */
    var vz: Double
)
/**
 * Documentation for Dyn4jSimulation
 */

class Dyn4jSimulation(seed: Long = 42L) {

    constructor(config: com.areslib.state.RobotFieldConfig, seed: Long = 42L) : this(seed) {
        buildWorld(config)
    }

    private val physicsWorld = Dyn4jPhysicsWorld(seed)
    private val swerveSim = Dyn4jSwerveModuleSim()
    private val telemetryPublisher = Dyn4jSimTelemetryPublisher()

    private var shootCooldownTimer = 0.0

    internal val flywheelSim = FlywheelSim()
    internal val intakePivotSim = IntakePivotSim()

    internal var simFlywheelVoltage = 0.0
    internal var simCowlVoltage = 0.0
    internal var simIntakePivotVoltage = 0.0
    internal var simIntakeRollerVoltage = 0.0
    internal var simFeederVoltage = 0.0
    internal var simFloorVoltage = 0.0
    internal var simFloorVelocityRps = 0.0
    internal var simClimberVoltage = 0.0
    internal var simClimberExtensionMeters = 0.0
    internal var simCowlAngle = 0.0
    internal var simFeederPieceDetected = false
    /**
     * Documentation for flywheelRotationAngle
     */
    var flywheelRotationAngle = 0.0
        private set
    /**
     * Documentation for flywheelIO
     */

    val flywheelIO: FlywheelIO = com.areslib.frc.sim.io.SimulatedFlywheelIO(this)
    /**
     * Documentation for cowlIO
     */
    val cowlIO: CowlIO = com.areslib.frc.sim.io.SimulatedCowlIO(this)
    /**
     * Documentation for intakeIO
     */
    val intakeIO: IntakeIO = com.areslib.frc.sim.io.SimulatedIntakeIO(this)
    /**
     * Documentation for feederIO
     */
    val feederIO: FeederIO = com.areslib.frc.sim.io.SimulatedFeederIO(this)
    /**
     * Documentation for floorIO
     */
    val floorIO: FloorIO = com.areslib.frc.sim.io.SimulatedFloorIO(this)
    /**
     * Documentation for climberIO
     */
    val climberIO: ClimberIO = com.areslib.frc.sim.io.SimulatedClimberIO(this)

    private val scratchActions = mutableListOf<RobotAction>()
    private val hubCenters = listOf(Vector2(4.135, 4.0345), Vector2(12.406, 4.0345))
    private val random = java.util.Random()
    /**
     * Documentation for step
     */

    fun step(state: RobotState, dt: Double): List<RobotAction> {
        scratchActions.clear()
        /**
         * Documentation for actions
         */
        val actions = scratchActions
        /**
         * Documentation for timestamp
         */
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()

        if (dt <= 0.0) return actions

        if (shootCooldownTimer > 0.0) {
            shootCooldownTimer -= dt
        }

        swerveSim.update(state, physicsWorld.robotBody)
        physicsWorld.step(dt)

        flywheelSim.update(simFlywheelVoltage, dt)
        intakePivotSim.update(simIntakePivotVoltage, dt)
        /**
         * Documentation for flywheelRps
         */

        val flywheelRps = flywheelSim.velocityRpm / 60.0
        flywheelRotationAngle += (flywheelRps * 2.0 * Math.PI) * dt

        simCowlAngle += (simCowlVoltage * 15.0) * dt
        simCowlAngle = simCowlAngle.coerceIn(0.0, 70.0)
        /**
         * Documentation for targetFloorVelocityRps
         */

        val targetFloorVelocityRps = (simFloorVoltage / 12.0) * 125.5
        simFloorVelocityRps += (targetFloorVelocityRps - simFloorVelocityRps) * 15.0 * dt
        simFloorVelocityRps = simFloorVelocityRps.coerceIn(-125.5, 125.5)
        /**
         * Documentation for climberVelocity
         */

        val climberVelocity = (simClimberVoltage / 12.0) * 1.0
        simClimberExtensionMeters += climberVelocity * dt
        simClimberExtensionMeters = simClimberExtensionMeters.coerceIn(0.0, 1.73)
        /**
         * Documentation for t
         */

        val t = physicsWorld.robotBody.transform
        /**
         * Documentation for robotX
         */
        val robotX = t.translationX
        /**
         * Documentation for robotY
         */
        val robotY = t.translationY
        /**
         * Documentation for robotHeading
         */
        val robotHeading = t.rotationAngle
        /**
         * Documentation for intakeDeployed
         */

        val intakeDeployed = intakePivotSim.angleDegrees > 45.0
        /**
         * Documentation for intakeSpinning
         */
        val intakeSpinning = simIntakeRollerVoltage > 1.0

        if (intakeDeployed && intakeSpinning && state.superstructure.marvin.inventoryCount < 40) {
            for (i in physicsWorld.balls.indices.reversed()) {
                /**
                 * Documentation for ball
                 */
                val ball = physicsWorld.balls[i]
                /**
                 * Documentation for bx
                 */
                val bx = ball.transform.translationX
                /**
                 * Documentation for by
                 */
                val by = ball.transform.translationY
                /**
                 * Documentation for dist
                 */
                val dist = Math.hypot(bx - robotX, by - robotY)
                if (dist < 0.5) {
                    physicsWorld.world.removeBody(ball)
                    physicsWorld.balls.removeAt(i)
                    /**
                     * Documentation for newCount
                     */
                    val newCount = state.superstructure.marvin.inventoryCount + 1
                    actions.add(com.areslib.frc.marvin.SetInventoryCount(newCount, timestamp))
                    simFeederPieceDetected = true
                    println("BALL INGESTED! Inventory: $newCount")
                    break
                }
            }
        }
        /**
         * Documentation for flywheelAtSpeed
         */

        val flywheelAtSpeed = state.superstructure.marvin.isFlywheelAtSpeed
        /**
         * Documentation for feederSpinning
         */
        val feederSpinning = simFeederVoltage > 2.0
        if (flywheelAtSpeed && feederSpinning && state.superstructure.marvin.inventoryCount > 0 && shootCooldownTimer <= 0.0) {
            shootCooldownTimer = 0.15
            /**
             * Documentation for newCount
             */
            val newCount = state.superstructure.marvin.inventoryCount - 1
            actions.add(com.areslib.frc.marvin.SetInventoryCount(newCount, timestamp))
            simFeederPieceDetected = newCount > 0
            /**
             * Documentation for vLaunch
             */

            val vLaunch = flywheelRps * 0.18
            /**
             * Documentation for hoodRad
             */
            val hoodRad = Math.toRadians(simCowlAngle)
            /**
             * Documentation for vPlanar
             */
            val vPlanar = vLaunch * kotlin.math.cos(hoodRad)
            /**
             * Documentation for vVert
             */
            val vVert = vLaunch * kotlin.math.sin(hoodRad)
            /**
             * Documentation for robotVx
             */

            val robotVx = physicsWorld.robotBody.linearVelocity.x
            /**
             * Documentation for robotVy
             */
            val robotVy = physicsWorld.robotBody.linearVelocity.y
            /**
             * Documentation for bx
             */
            
            val bx = robotX + kotlin.math.cos(robotHeading) * 0.5
            /**
             * Documentation for by
             */
            val by = robotY + kotlin.math.sin(robotHeading) * 0.5
            /**
             * Documentation for bz
             */
            val bz = 0.6
            /**
             * Documentation for vx
             */

            val vx = robotVx + kotlin.math.cos(robotHeading) * vPlanar
            /**
             * Documentation for vy
             */
            val vy = robotVy + kotlin.math.sin(robotHeading) * vPlanar
            /**
             * Documentation for vz
             */
            val vz = vVert
            /**
             * Documentation for flyingBall
             */

            val flyingBall = FlyingBall(bx, by, bz, vx, vy, vz)
            physicsWorld.flyingBalls.add(flyingBall)
            println("BALL SHOT (2.5D)! Pos: ($bx, $by, $bz), Vel: ($vx, $vy, $vz). Inventory left: $newCount")
        }
        /**
         * Documentation for g
         */

        val g = 9.80665
        for (i in physicsWorld.flyingBalls.indices.reversed()) {
            /**
             * Documentation for fb
             */
            val fb = physicsWorld.flyingBalls[i]
            fb.x += fb.vx * dt
            fb.y += fb.vy * dt
            fb.z += fb.vz * dt
            fb.vz -= g * dt
            /**
             * Documentation for scored
             */

            var scored = false
            for (hubCenter in hubCenters) {
                /**
                 * Documentation for dx
                 */
                val dx = fb.x - hubCenter.x
                /**
                 * Documentation for dy
                 */
                val dy = fb.y - hubCenter.y
                /**
                 * Documentation for dist
                 */
                val dist = Math.hypot(dx, dy)
                if (dist < 0.6 && fb.z >= 1.6 && fb.z <= 2.8) {
                    scored = true
                    break
                }
            }

            when {
                scored -> {
                    physicsWorld.flyingBalls.removeAt(i)
                    println("BALL SCORED! Ejecting to center...")
                    /**
                     * Documentation for ejectAngle
                     */
                    
                    val ejectAngle = random.nextDouble() * 2.0 * Math.PI
                    /**
                     * Documentation for ejectSpeed
                     */
                    val ejectSpeed = 1.5 + random.nextDouble() * 1.5
                    /**
                     * Documentation for evx
                     */
                    val evx = Math.cos(ejectAngle) * ejectSpeed
                    /**
                     * Documentation for evy
                     */
                    val evy = Math.sin(ejectAngle) * ejectSpeed
                    /**
                     * Documentation for ball
                     */

                    val ball = Body()
                    /**
                     * Documentation for fixture
                     */
                    val fixture = ball.addFixture(Geometry.createCircle(0.0635))
                    fixture.friction = 0.6
                    fixture.restitution = 0.4
                    fixture.density = 5.92
                    ball.setMass(MassType.NORMAL)
                    ball.linearDamping = 2.0
                    ball.angularDamping = 2.0
                    ball.translate(8.2705, 4.0345)
                    ball.linearVelocity.set(evx, evy)
                    
                    physicsWorld.world.addBody(ball)
                    physicsWorld.balls.add(ball)
                }
                fb.z <= 0.0635 -> {
                    physicsWorld.flyingBalls.removeAt(i)
                    println("BALL LANDED! Spawning back as dynamic 2D body at (${fb.x}, ${fb.y})")
                    /**
                     * Documentation for fieldWidth
                     */

                    val fieldWidth = 16.541
                    /**
                     * Documentation for fieldHeight
                     */
                    val fieldHeight = 8.069
                    /**
                     * Documentation for cx
                     */
                    val cx = fb.x.coerceIn(0.1, fieldWidth - 0.1)
                    /**
                     * Documentation for cy
                     */
                    val cy = fb.y.coerceIn(0.1, fieldHeight - 0.1)
                    /**
                     * Documentation for ball
                     */

                    val ball = Body()
                    /**
                     * Documentation for fixture
                     */
                    val fixture = ball.addFixture(Geometry.createCircle(0.0635))
                    fixture.friction = 0.6
                    fixture.restitution = 0.4
                    fixture.density = 5.92
                    ball.setMass(MassType.NORMAL)
                    ball.linearDamping = 2.0
                    ball.angularDamping = 2.0
                    ball.translate(cx, cy)
                    ball.linearVelocity.set(fb.vx, fb.vy)

                    physicsWorld.world.addBody(ball)
                    physicsWorld.balls.add(ball)
                }
            }
        }

        return actions
    }
    /**
     * Documentation for getPoseUpdate
     */

    fun getPoseUpdate(): RobotAction.PoseUpdate {
        /**
         * Documentation for t
         */
        val t = physicsWorld.robotBody.transform
        return RobotAction.PoseUpdate(
            xMeters = t.translationX,
            yMeters = t.translationY,
            headingRadians = t.rotationAngle,
            timestampMs = com.areslib.util.RobotClock.currentTimeMillis()
        )
    }
    /**
     * Documentation for publishVisualization
     */

    fun publishVisualization(state: RobotState, telemetry: ITelemetry) {
        telemetryPublisher.publishVisualization(
            state, telemetry, intakePivotSim.angleDegrees, simCowlAngle, flywheelRotationAngle, physicsWorld.balls, physicsWorld.flyingBalls
        )
    }
    /**
     * Documentation for resetPose
     */

    fun resetPose(x: Double, y: Double, heading: Double) {
        physicsWorld.resetPose(x, y, heading)
    }
    /**
     * Documentation for buildWorld
     */

    fun buildWorld(config: com.areslib.state.RobotFieldConfig) {
        physicsWorld.buildWorld(config)
    }
}
