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

class FlyingBall(
    var x: Double,
    var y: Double,
    var z: Double,
    var vx: Double,
    var vy: Double,
    var vz: Double
)

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
    var flywheelRotationAngle = 0.0
        private set

    val flywheelIO: FlywheelIO = com.areslib.frc.sim.io.SimulatedFlywheelIO(this)
    val cowlIO: CowlIO = com.areslib.frc.sim.io.SimulatedCowlIO(this)
    val intakeIO: IntakeIO = com.areslib.frc.sim.io.SimulatedIntakeIO(this)
    val feederIO: FeederIO = com.areslib.frc.sim.io.SimulatedFeederIO(this)
    val floorIO: FloorIO = com.areslib.frc.sim.io.SimulatedFloorIO(this)
    val climberIO: ClimberIO = com.areslib.frc.sim.io.SimulatedClimberIO(this)

    private val scratchActions = mutableListOf<RobotAction>()
    private val hubCenters = listOf(Vector2(4.135, 4.0345), Vector2(12.406, 4.0345))
    private val random = java.util.Random()

    fun step(state: RobotState, dt: Double): List<RobotAction> {
        scratchActions.clear()
        val actions = scratchActions
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()

        if (dt <= 0.0) return actions

        if (shootCooldownTimer > 0.0) {
            shootCooldownTimer -= dt
        }

        swerveSim.update(state, physicsWorld.robotBody)
        physicsWorld.step(dt)

        flywheelSim.update(simFlywheelVoltage, dt)
        intakePivotSim.update(simIntakePivotVoltage, dt)

        val flywheelRps = flywheelSim.velocityRpm / 60.0
        flywheelRotationAngle += (flywheelRps * 2.0 * Math.PI) * dt

        simCowlAngle += (simCowlVoltage * 15.0) * dt
        simCowlAngle = simCowlAngle.coerceIn(0.0, 70.0)

        val targetFloorVelocityRps = (simFloorVoltage / 12.0) * 125.5
        simFloorVelocityRps += (targetFloorVelocityRps - simFloorVelocityRps) * 15.0 * dt
        simFloorVelocityRps = simFloorVelocityRps.coerceIn(-125.5, 125.5)

        val climberVelocity = (simClimberVoltage / 12.0) * 1.0
        simClimberExtensionMeters += climberVelocity * dt
        simClimberExtensionMeters = simClimberExtensionMeters.coerceIn(0.0, 1.73)

        val t = physicsWorld.robotBody.transform
        val robotX = t.translationX
        val robotY = t.translationY
        val robotHeading = t.rotationAngle

        val intakeDeployed = intakePivotSim.angleDegrees > 45.0
        val intakeSpinning = simIntakeRollerVoltage > 1.0

        if (intakeDeployed && intakeSpinning && state.superstructure.marvin.inventoryCount < 40) {
            for (i in physicsWorld.balls.indices.reversed()) {
                val ball = physicsWorld.balls[i]
                val bx = ball.transform.translationX
                val by = ball.transform.translationY
                val dist = Math.hypot(bx - robotX, by - robotY)
                if (dist < 0.5) {
                    physicsWorld.world.removeBody(ball)
                    physicsWorld.balls.removeAt(i)
                    val newCount = state.superstructure.marvin.inventoryCount + 1
                    actions.add(com.areslib.frc.marvin.SetInventoryCount(newCount, timestamp))
                    simFeederPieceDetected = true
                    println("BALL INGESTED! Inventory: $newCount")
                    break
                }
            }
        }

        val flywheelAtSpeed = state.superstructure.marvin.isFlywheelAtSpeed
        val feederSpinning = simFeederVoltage > 2.0
        if (flywheelAtSpeed && feederSpinning && state.superstructure.marvin.inventoryCount > 0 && shootCooldownTimer <= 0.0) {
            shootCooldownTimer = 0.15
            val newCount = state.superstructure.marvin.inventoryCount - 1
            actions.add(com.areslib.frc.marvin.SetInventoryCount(newCount, timestamp))
            simFeederPieceDetected = newCount > 0

            val vLaunch = flywheelRps * 0.18
            val hoodRad = Math.toRadians(simCowlAngle)
            val vPlanar = vLaunch * kotlin.math.cos(hoodRad)
            val vVert = vLaunch * kotlin.math.sin(hoodRad)

            val robotVx = physicsWorld.robotBody.linearVelocity.x
            val robotVy = physicsWorld.robotBody.linearVelocity.y
            
            val bx = robotX + kotlin.math.cos(robotHeading) * 0.5
            val by = robotY + kotlin.math.sin(robotHeading) * 0.5
            val bz = 0.6

            val vx = robotVx + kotlin.math.cos(robotHeading) * vPlanar
            val vy = robotVy + kotlin.math.sin(robotHeading) * vPlanar
            val vz = vVert

            val flyingBall = FlyingBall(bx, by, bz, vx, vy, vz)
            physicsWorld.flyingBalls.add(flyingBall)
            println("BALL SHOT (2.5D)! Pos: ($bx, $by, $bz), Vel: ($vx, $vy, $vz). Inventory left: $newCount")
        }

        val g = 9.80665
        for (i in physicsWorld.flyingBalls.indices.reversed()) {
            val fb = physicsWorld.flyingBalls[i]
            fb.x += fb.vx * dt
            fb.y += fb.vy * dt
            fb.z += fb.vz * dt
            fb.vz -= g * dt

            var scored = false
            for (hubCenter in hubCenters) {
                val dx = fb.x - hubCenter.x
                val dy = fb.y - hubCenter.y
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
                    
                    val ejectAngle = random.nextDouble() * 2.0 * Math.PI
                    val ejectSpeed = 1.5 + random.nextDouble() * 1.5
                    val evx = Math.cos(ejectAngle) * ejectSpeed
                    val evy = Math.sin(ejectAngle) * ejectSpeed

                    val ball = Body()
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

                    val fieldWidth = 16.541
                    val fieldHeight = 8.069
                    val cx = fb.x.coerceIn(0.1, fieldWidth - 0.1)
                    val cy = fb.y.coerceIn(0.1, fieldHeight - 0.1)

                    val ball = Body()
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

    fun getPoseUpdate(): RobotAction.PoseUpdate {
        val t = physicsWorld.robotBody.transform
        return RobotAction.PoseUpdate(
            xMeters = t.translationX,
            yMeters = t.translationY,
            headingRadians = t.rotationAngle,
            timestampMs = com.areslib.util.RobotClock.currentTimeMillis()
        )
    }

    fun publishVisualization(state: RobotState, telemetry: ITelemetry) {
        telemetryPublisher.publishVisualization(
            state, telemetry, intakePivotSim.angleDegrees, simCowlAngle, flywheelRotationAngle, physicsWorld.balls, physicsWorld.flyingBalls
        )
    }

    fun resetPose(x: Double, y: Double, heading: Double) {
        physicsWorld.resetPose(x, y, heading)
    }

    fun buildWorld(config: com.areslib.state.RobotFieldConfig) {
        physicsWorld.buildWorld(config)
    }
}
