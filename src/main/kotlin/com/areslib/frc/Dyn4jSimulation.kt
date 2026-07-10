package com.areslib.frc

import com.areslib.action.RobotAction
import com.areslib.hardware.FlywheelIO
import com.areslib.hardware.CowlIO
import com.areslib.hardware.IntakeIO
import com.areslib.hardware.FeederIO
import com.areslib.hardware.FloorIO
import com.areslib.hardware.ClimberIO
import com.areslib.sim.FlywheelSim
import com.areslib.sim.IntakePivotSim
import com.areslib.state.RobotState
import com.areslib.telemetry.ITelemetry

import org.dyn4j.dynamics.Body
import org.dyn4j.world.World
import org.dyn4j.geometry.Geometry
import org.dyn4j.geometry.MassType
import org.dyn4j.geometry.Vector2

/**
 * Self-contained Dyn4j physics simulation for the FRC swerve robot.
 *
 * Owns the physics world, robot body, fuel balls, and simulated hardware IO objects.
 * The ARESRobot TimedRobot shell calls [step] each cycle and reads results back
 * via [getPoseUpdate] and the simulated IO properties.
 */
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

    // ── Physics World ──
    private val world = World<Body>()
    private val robotBody = Body()
    private val balls = mutableListOf<Body>()
    private val flyingBalls = mutableListOf<FlyingBall>()
    private var shootCooldownTimer = 0.0
    private val forceVector = Vector2()

    // ── Sim Models ──
    internal val flywheelSim = FlywheelSim()
    internal val intakePivotSim = IntakePivotSim()

    // ── Simulated Voltages (written by IO, consumed by step) ──
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

    // ── Simulated Hardware IO Objects ──

    val flywheelIO: FlywheelIO = com.areslib.frc.sim.io.SimulatedFlywheelIO(this)
    val cowlIO: CowlIO = com.areslib.frc.sim.io.SimulatedCowlIO(this)
    val intakeIO: IntakeIO = com.areslib.frc.sim.io.SimulatedIntakeIO(this)
    val feederIO: FeederIO = com.areslib.frc.sim.io.SimulatedFeederIO(this)
    val floorIO: FloorIO = com.areslib.frc.sim.io.SimulatedFloorIO(this)
    val climberIO: ClimberIO = com.areslib.frc.sim.io.SimulatedClimberIO(this)

    init {
        world.setGravity(Vector2(0.0, 0.0))

        // Robot body (~28 inch square)
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
     * Step the physics simulation forward by [dt] seconds using the current [state].
     * Returns a list of RobotActions to dispatch back to the Store (inventory changes, etc).
     */
    fun step(state: RobotState, dt: Double): List<RobotAction> {
        val actions = mutableListOf<RobotAction>()
        val timestamp = com.areslib.util.RobotClock.currentTimeMillis()

        if (dt <= 0.0) return actions

        // Update shoot cooldown
        if (shootCooldownTimer > 0.0) {
            shootCooldownTimer -= dt
        }

        // ── Drive Physics ──
        val kpLinear = 50.0
        val kpAngular = 20.0
        val forceX = (state.drive.xVelocityMetersPerSecond - robotBody.linearVelocity.x) * kpLinear
        val forceY = (state.drive.yVelocityMetersPerSecond - robotBody.linearVelocity.y) * kpLinear
        val torque = (state.drive.angularVelocityRadiansPerSecond - robotBody.angularVelocity) * kpAngular

        robotBody.isAtRest = false
        forceVector.set(forceX, forceY)
        robotBody.applyForce(forceVector)
        robotBody.applyTorque(torque)

        world.step(1, dt)

        // ── Superstructure Physics ──
        flywheelSim.update(simFlywheelVoltage, dt)
        intakePivotSim.update(simIntakePivotVoltage, dt)

        // Flywheel visual rotation
        val flywheelRps = flywheelSim.velocityRpm / 60.0
        flywheelRotationAngle += (flywheelRps * 2.0 * Math.PI) * dt

        // Cowl position integration
        simCowlAngle += (simCowlVoltage * 15.0) * dt
        simCowlAngle = simCowlAngle.coerceIn(0.0, 70.0)

        // Floor velocity integration
        val targetFloorVelocityRps = (simFloorVoltage / 12.0) * 125.5
        simFloorVelocityRps += (targetFloorVelocityRps - simFloorVelocityRps) * 15.0 * dt
        simFloorVelocityRps = simFloorVelocityRps.coerceIn(-125.5, 125.5)

        // Climber extension integration
        val climberVelocity = (simClimberVoltage / 12.0) * 1.0
        simClimberExtensionMeters += climberVelocity * dt
        simClimberExtensionMeters = simClimberExtensionMeters.coerceIn(0.0, 1.73)

        // ── Game Piece Collision ──
        val t = robotBody.transform
        val robotX = t.translationX
        val robotY = t.translationY
        val robotHeading = t.rotationAngle

        val intakeDeployed = intakePivotSim.angleDegrees > 45.0
        val intakeSpinning = simIntakeRollerVoltage > 1.0

        if (intakeDeployed && intakeSpinning && state.superstructure.inventoryCount < 40) {
            val ballIterator = balls.iterator()
            while (ballIterator.hasNext()) {
                val ball = ballIterator.next()
                val bx = ball.transform.translationX
                val by = ball.transform.translationY
                val dist = Math.hypot(bx - robotX, by - robotY)
                if (dist < 0.5) {
                    world.removeBody(ball)
                    ballIterator.remove()
                    val newCount = state.superstructure.inventoryCount + 1
                    actions.add(RobotAction.SetInventoryCount(newCount, timestamp))
                    simFeederPieceDetected = true
                    println("BALL INGESTED! Inventory: $newCount")
                    break
                }
            }
        }

        // ── Shooting ──
        val flywheelAtSpeed = state.superstructure.isFlywheelAtSpeed
        val feederSpinning = simFeederVoltage > 2.0
        if (flywheelAtSpeed && feederSpinning && state.superstructure.inventoryCount > 0 && shootCooldownTimer <= 0.0) {
            shootCooldownTimer = 0.15
            val newCount = state.superstructure.inventoryCount - 1
            actions.add(RobotAction.SetInventoryCount(newCount, timestamp))
            simFeederPieceDetected = newCount > 0

            val vLaunch = flywheelRps * 0.18 // At 4000 RPM, vLaunch is ~12.0 m/s
            
            val hoodRad = Math.toRadians(simCowlAngle)
            val vPlanar = vLaunch * Math.cos(hoodRad)
            val vVert = vLaunch * Math.sin(hoodRad)

            val robotVx = robotBody.linearVelocity.x
            val robotVy = robotBody.linearVelocity.y
            
            val bx = robotX + Math.cos(robotHeading) * 0.5
            val by = robotY + Math.sin(robotHeading) * 0.5
            val bz = 0.6 // Launch height from shooter

            val vx = robotVx + Math.cos(robotHeading) * vPlanar
            val vy = robotVy + Math.sin(robotHeading) * vPlanar
            val vz = vVert

            val flyingBall = FlyingBall(bx, by, bz, vx, vy, vz)
            flyingBalls.add(flyingBall)
            println("BALL SHOT (2.5D)! Pos: ($bx, $by, $bz), Vel: ($vx, $vy, $vz). Inventory left: $newCount")
        }

        // ── Flying Balls Physics & Scoring ──
        val g = 9.80665
        val flyingIterator = flyingBalls.iterator()
        val random = java.util.Random()
        while (flyingIterator.hasNext()) {
            val fb = flyingIterator.next()
            fb.x += fb.vx * dt
            fb.y += fb.vy * dt
            fb.z += fb.vz * dt
            fb.vz -= g * dt

            // 1. Check for scoring in either hub cylindrical volume
            var scored = false
            for (hubCenter in listOf(Vector2(4.135, 4.0345), Vector2(12.406, 4.0345))) {
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
                    flyingIterator.remove()
                    println("BALL SCORED! Ejecting to center...")
                    
                    val ejectAngle = random.nextDouble() * 2.0 * Math.PI
                    val ejectSpeed = 1.5 + random.nextDouble() * 1.5 // 1.5 to 3.0 m/s
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
                    
                    world.addBody(ball)
                    balls.add(ball)
                }
                fb.z <= 0.0635 -> { // Lands on the ground
                    flyingIterator.remove()
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

                    world.addBody(ball)
                    balls.add(ball)
                }
            }
        }

        return actions
    }

    /**
     * Returns a PoseUpdate action reflecting the physics body's current transform.
     */
    fun getPoseUpdate(): RobotAction.PoseUpdate {
        val t = robotBody.transform
        return RobotAction.PoseUpdate(
            xMeters = t.translationX,
            yMeters = t.translationY,
            headingRadians = t.rotationAngle,
            timestampMs = com.areslib.util.RobotClock.currentTimeMillis()
        )
    }

    /**
     * Publishes fuel/game piece 3D poses and mechanism 3D visualization to telemetry.
     */
    fun publishVisualization(state: RobotState, telemetry: ITelemetry) {
        val robotX = state.drive.odometryX
        val robotY = state.drive.odometryY
        val robotHeading = state.drive.odometryHeading

        val halfHeading = robotHeading / 2.0
        val robotQW = Math.cos(halfHeading)
        val robotQZ = Math.sin(halfHeading)

        // ── Intake 3D Pose ──
        val intakeAngleRad = Math.toRadians(intakePivotSim.angleDegrees)
        val halfIntake = intakeAngleRad / 2.0
        val intCosY = Math.cos(halfIntake)
        val intSinY = Math.sin(halfIntake)
        telemetry.putDoubleArray("Robot/Superstructure/3D/Intake", doubleArrayOf(
            robotX + 0.35 * Math.cos(robotHeading),
            robotY + 0.35 * Math.sin(robotHeading),
            0.2,
            robotQW * intCosY, -robotQZ * intSinY, robotQW * intSinY, robotQZ * intCosY
        ))

        // ── Cowl 3D Pose ──
        val cowlAngleRad = Math.toRadians(simCowlAngle)
        val halfCowl = cowlAngleRad / 2.0
        val cowlCosY = Math.cos(halfCowl)
        val cowlSinY = Math.sin(halfCowl)
        telemetry.putDoubleArray("Robot/Superstructure/3D/Cowl", doubleArrayOf(
            robotX - 0.2 * Math.cos(robotHeading),
            robotY - 0.2 * Math.sin(robotHeading),
            0.6,
            robotQW * cowlCosY, -robotQZ * cowlSinY, robotQW * cowlSinY, robotQZ * cowlCosY
        ))

        // ── Flywheel 3D Pose ──
        val halfFlywheel = flywheelRotationAngle / 2.0
        val flyCosY = Math.cos(halfFlywheel)
        val flySinY = Math.sin(halfFlywheel)
        telemetry.putDoubleArray("Robot/Superstructure/3D/Flywheel", doubleArrayOf(
            robotX - 0.1 * Math.cos(robotHeading),
            robotY - 0.1 * Math.sin(robotHeading),
            0.6,
            robotQW * flyCosY, -robotQZ * flySinY, robotQW * flySinY, robotQZ * flyCosY
        ))

        // ── Fuel 3D Poses ──
        val totalBallsCount = balls.size + flyingBalls.size
        val gamePieceData = DoubleArray(totalBallsCount * 7)
        for (i in balls.indices) {
            val idx = i * 7
            gamePieceData[idx] = balls[i].transform.translationX
            gamePieceData[idx + 1] = balls[i].transform.translationY
            gamePieceData[idx + 2] = 0.0635
            val theta = balls[i].transform.rotationAngle
            gamePieceData[idx + 3] = kotlin.math.cos(theta / 2.0)
            gamePieceData[idx + 4] = 0.0
            gamePieceData[idx + 5] = 0.0
            gamePieceData[idx + 6] = kotlin.math.sin(theta / 2.0)
        }
        val groundOffset = balls.size * 7
        for (i in flyingBalls.indices) {
            val fb = flyingBalls[i]
            val idx = groundOffset + i * 7
            gamePieceData[idx] = fb.x
            gamePieceData[idx + 1] = fb.y
            gamePieceData[idx + 2] = fb.z
            gamePieceData[idx + 3] = 1.0 // qw
            gamePieceData[idx + 4] = 0.0 // qx
            gamePieceData[idx + 5] = 0.0 // qy
            gamePieceData[idx + 6] = 0.0 // qz
        }
        telemetry.putDoubleArray("Robot/FuelPoses", gamePieceData)
    }

    /**
     * Snap the physics body to match a given pose (used at auto init).
     */
    fun resetPose(x: Double, y: Double, heading: Double) {
        robotBody.transform.setTranslation(x, y)
        robotBody.transform.setRotation(heading)
        robotBody.linearVelocity.set(0.0, 0.0)
        robotBody.angularVelocity = 0.0
        robotBody.isAtRest = false
    }

    // ── Field Setup ──

    private fun spawnFuel(@Suppress("UNUSED_PARAMETER") seed: Long) {
        val width = 16.541
        val height = 8.069
        val ballRadius = 0.0635 // 5in diameter -> 0.0635m radius

        // Let's hold coordinates for exactly 24 balls
        val spawnPoints = mutableListOf<Vector2>()

        // 1. Tarmac Rings around Hubs (4 diagonal positions around Blue Hub, 4 around Red Hub)
        val rHub = 1.6
        val angles = doubleArrayOf(Math.PI / 4.0, 3.0 * Math.PI / 4.0, 5.0 * Math.PI / 4.0, 7.0 * Math.PI / 4.0)
        
        // Around Blue Hub
        val blueHubX = 4.135
        val blueHubY = 4.0345
        for (angle in angles) {
            spawnPoints.add(Vector2(blueHubX + rHub * Math.cos(angle), blueHubY + rHub * Math.sin(angle)))
        }

        // Around Red Hub
        val redHubX = width - 4.135
        val redHubY = 4.0345
        for (angle in angles) {
            spawnPoints.add(Vector2(redHubX + rHub * Math.cos(angle), redHubY + rHub * Math.sin(angle)))
        }

        // 2. Trench Run Cargo (4 in Bottom Trench, 4 in Top Trench)
        val bottomTrenchY = 0.7
        val bottomTrenchX = doubleArrayOf(5.5, 7.0, 8.5, 10.0)
        for (x in bottomTrenchX) {
            spawnPoints.add(Vector2(x, bottomTrenchY))
        }

        val topTrenchY = height - 0.7
        val topTrenchX = doubleArrayOf(6.5, 8.0, 9.5, 11.0)
        for (x in topTrenchX) {
            spawnPoints.add(Vector2(x, topTrenchY))
        }

        // 3. Autonomous Center Line Cargo (8 pieces along central loading region/auto line)
        val centerX = width / 2.0
        val centerYs = doubleArrayOf(1.8, 2.4, 3.0, 3.6, 4.4, 5.0, 5.6, 6.2)
        for (y in centerYs) {
            spawnPoints.add(Vector2(centerX, y))
        }

        // Now spawn the structured balls in the dyn4j world
        for (point in spawnPoints) {
            val ball = Body()
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

    /**
     * Dynamically constructs the physics world (walls, obstacles, game elements) from a RobotFieldConfig.
     */
    fun buildWorld(config: com.areslib.state.RobotFieldConfig) {
        val bodies = world.bodies.toList()
        for (body in bodies) {
            if (body != robotBody) {
                world.removeBody(body)
            }
        }
        balls.clear()

        val width = if (config.fieldType == com.areslib.state.FieldType.FRC) 16.541 else 3.6576
        val height = if (config.fieldType == com.areslib.state.FieldType.FRC) 8.069 else 3.6576

        // Outer bounds
        com.areslib.frc.sim.field.FrcFieldBuilder.buildWorldWalls(world, width, height)

        // Load obstacles
        com.areslib.sim.FieldObstacleLoader.loadObstacles(world, config.obstacles)

        // Load elements
        val loadedElements = com.areslib.sim.FieldElementLoader.loadElements(world, config.elementTypes, config.elements)
        balls.addAll(loadedElements)
        println("[FRC Sim] Successfully built world with ${config.obstacles.size} obstacles and ${config.elements.size} elements.")
    }
}
