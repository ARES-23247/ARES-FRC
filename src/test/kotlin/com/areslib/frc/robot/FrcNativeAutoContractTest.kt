package com.areslib.frc.robot

import com.areslib.action.RobotAction
import com.areslib.auto.AresAutoCodec
import com.areslib.auto.AutoPose
import com.areslib.auto.AutoRoutine
import com.areslib.auto.AutoStep
import com.areslib.frc.FrcSwerveRobot
import com.areslib.frc.marvin.MarvinReducer
import com.areslib.frc.marvin.MarvinState
import com.areslib.frc.marvin.SetFlywheelSpeed
import com.areslib.frc.marvin.SuperstructureSensorUpdate
import com.areslib.frc.marvin.marvin
import com.areslib.math.coordinate.CoordinateTransformers
import com.areslib.pathing.CommandKey
import com.areslib.pathing.NamedCommandDescriptor
import com.areslib.pathing.NamedCommands
import com.areslib.state.Alliance
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import com.areslib.util.RobotClock
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import java.io.File
import java.nio.file.Path

/** End-to-end contract for GUI assets, robot capability registration, and FRC native execution. */
class FrcNativeAutoContractTest {
    private lateinit var projectRoot: File

    @BeforeEach
    fun setUp() {
        RobotClock.useMockTime(1_000L)
        projectRoot = findProjectRoot()
        NamedCommands.clear()
        FrcAutoCapabilities.register()
    }

    @AfterEach
    fun tearDown() {
        NamedCommands.clear()
        RobotClock.useSystemTime()
    }

    @Test
    fun `offline manifest source catalog and runtime registry are identical`() {
        val manifest = manifestDescriptors()
        val declared = FrcAutoCapabilities.descriptors.associateBy { it.key.value }
        val registered = NamedCommands.catalog().associateBy { it.key.value }

        assertEquals(declared, manifest, "FRC editor manifest drifted from source metadata")
        assertEquals(manifest, registered, "FRC runtime registry drifted from the editor manifest")
    }

    @Test
    fun `every deployed native auto preflights for both alliances`() {
        val directory = autosDirectory()
        val files = directory.listFiles { file -> file.isFile && file.extension == "aresauto" }
            .orEmpty()
            .sortedBy(File::getName)
        assertTrue(files.isNotEmpty(), "At least one deployable native FRC auto is required")

        val advertisedKeys = FrcAutoCapabilities.descriptors.map { it.key.value }.toSet()
        files.forEach { file ->
            val routine = AresAutoCodec.decode(file.readText())
            assertEquals(file.nameWithoutExtension, routine.documentId)
            assertTrue(
                referencedCommands(routine).all(advertisedKeys::contains),
                "${file.name} references an action absent from auto-capabilities.json"
            )

            Alliance.entries.forEach { alliance ->
                val robot = newRobot(alliance)
                val runner = runner(robot, file.nameWithoutExtension, directory)
                runner.autonomousInit()
                assertFalse(
                    runner.isFaultedForTest,
                    "${file.name} failed $alliance preflight: ${runner.statusForTest}"
                )
            }
        }
    }

    @Test
    fun `red auto seed reflects across the FRC alliance wall`() {
        val robot = newRobot(Alliance.RED)
        val runner = runner(robot, "sim-drive-and-shoot", autosDirectory())

        runner.autonomousInit()

        val pose = robot.store.state.drive.poseEstimator.estimatedPose
        assertEquals(CoordinateTransformers.FRC_FIELD_LENGTH - 2.0, pose.x, 1e-9)
        assertEquals(2.0, pose.y, 1e-9)
        assertEquals(-Math.PI, pose.heading.radians, 1e-9)
        assertFalse(runner.isFaultedForTest)
    }

    @Test
    fun `safe default preserves the current localized pose`() {
        val robot = newRobot(Alliance.BLUE)
        robot.store.dispatch(
            RobotAction.PoseUpdate(
                xMeters = 4.0,
                yMeters = 3.0,
                headingRadians = 0.4,
                timestampMs = 1_000L,
                isReset = true
            )
        )
        val runner = runner(robot, "do-nothing", autosDirectory())

        runner.autonomousInit()

        val pose = robot.store.state.drive.poseEstimator.estimatedPose
        assertEquals(4.0, pose.x, 1e-9)
        assertEquals(3.0, pose.y, 1e-9)
        assertEquals(0.4, pose.heading.radians, 1e-9)
        assertFalse(runner.isFaultedForTest)
    }

    @Test
    fun `native command timeline executes then completes with outputs stopped`(@TempDir temp: Path) {
        val routine = AutoRoutine(
            documentId = "command-timeline",
            name = "Command Timeline",
            startingPose = AutoPose(1.0, 1.0, 0.0),
            steps = listOf(
                AutoStep.command(FrcAutoCapabilities.SHOOTER_PREPARE.key),
                AutoStep(
                    kind = com.areslib.auto.AutoStepKind.WAIT,
                    durationSeconds = 0.10
                )
            )
        )
        temp.resolve("command-timeline.aresauto").toFile().writeText(AresAutoCodec.encode(routine))
        val robot = newRobot(Alliance.BLUE)
        val runner = runner(robot, routine.documentId, temp.toFile())

        runner.autonomousInit()
        runner.autonomousPeriodic()
        assertTrue(robot.store.state.superstructure.marvin.flywheelActive)
        assertEquals(4_000.0, robot.store.state.superstructure.marvin.flywheel.targetVelocityRpm)

        RobotClock.setMockTimeMs(1_120L)
        runner.autonomousPeriodic()
        assertTrue(runner.isFinishedForTest)
        assertEquals("Complete", runner.statusForTest)
        assertFalse(robot.store.state.superstructure.marvin.flywheelActive)
        assertEquals(0.0, robot.store.state.superstructure.marvin.flywheel.targetVelocityRpm)
    }

    @Test
    fun `invalid field footprint fails closed before motion`(@TempDir temp: Path) {
        val routine = AutoRoutine(
            documentId = "outside-field",
            name = "Outside Field",
            startingPose = AutoPose(0.10, 1.0, 0.0),
            steps = listOf(
                AutoStep(
                    kind = com.areslib.auto.AutoStepKind.WAIT,
                    durationSeconds = 0.0
                )
            )
        )
        temp.resolve("outside-field.aresauto").toFile().writeText(AresAutoCodec.encode(routine))
        val robot = newRobot(Alliance.BLUE)
        robot.drive.joystickDrive(2.0, -1.0, 0.5, isFieldCentric = false)
        val runner = runner(robot, routine.documentId, temp.toFile())

        runner.autonomousInit()

        assertTrue(runner.isFaultedForTest)
        assertEquals("Blocked", runner.statusForTest)
        assertEquals(0.0, robot.store.state.drive.xVelocityMetersPerSecond)
        assertEquals(0.0, robot.store.state.drive.yVelocityMetersPerSecond)
        assertEquals(0.0, robot.store.state.drive.angularVelocityRadiansPerSecond)
    }

    @Test
    fun `feed command requires fresh aligned RPM and times out safely`() {
        val robot = newRobot(Alliance.BLUE)
        robot.store.dispatch(SetFlywheelSpeed(4_000.0))
        robot.store.dispatch(sensorUpdate(valid = false))
        val blocked = requireNotNull(
            NamedCommands.create(FrcAutoCapabilities.SHOOTER_FEED_WHEN_READY.key, 1_000L)
        )
        blocked.initialize(robot.store.state)

        assertTrue(blocked.execute(robot.store.state, 1_999L).isEmpty())
        assertFalse(blocked.isCompleted(robot.store.state, 1_999L))
        assertTrue(blocked.isCompleted(robot.store.state, 2_000L))
        blocked.end(robot.store.state, interrupted = false).forEach(robot.store::dispatch)
        assertFalse(robot.store.state.superstructure.marvin.transferActive)
        assertEquals(0.0, robot.store.state.superstructure.marvin.feeder.targetVelocityRps)

        robot.store.dispatch(sensorUpdate(valid = true))
        val ready = requireNotNull(
            NamedCommands.create(FrcAutoCapabilities.SHOOTER_FEED_WHEN_READY.key, 3_000L)
        )
        ready.initialize(robot.store.state)
        ready.execute(robot.store.state, 20L).forEach(robot.store::dispatch)
        assertTrue(ready.isCompleted(robot.store.state, 20L))
        assertTrue(robot.store.state.superstructure.marvin.transferActive)
        assertEquals(
            com.areslib.frc.marvin.MarvinConfig.FEEDER_SHOOT_SPEED_RPS,
            robot.store.state.superstructure.marvin.feeder.targetVelocityRps
        )
    }

    private fun sensorUpdate(valid: Boolean) = SuperstructureSensorUpdate(
        flywheelRpm = 4_000.0,
        cowlAngleRotations = 0.0,
        intakeAngle = 0.0,
        pieceDetected = false,
        flywheelVelocityValid = valid
    )

    private fun newRobot(alliance: Alliance): FrcSwerveRobot = FrcSwerveRobot(
        isSimulation = true,
        initialState = RobotState(
            superstructure = SuperstructureState(custom = MarvinState())
        ),
        reducer = MarvinReducer::reduce
    ).also { robot -> robot.store.dispatch(RobotAction.SetAlliance(alliance)) }

    private fun runner(robot: FrcSwerveRobot, documentId: String, directory: File) =
        FRCAutoOrchestrator(
            robot = robot,
            selectionProvider = { documentId },
            directoryProvider = { listOf(directory) },
            resourceOpener = null
        )

    private fun manifestDescriptors(): Map<String, NamedCommandDescriptor> {
        val file = File(projectRoot, "src/main/deploy/ares/auto-capabilities.json")
        return MANIFEST_ACTION.findAll(file.readText()).associate { match ->
            val key = match.groupValues[1]
            key to NamedCommandDescriptor(
                key = CommandKey(key),
                displayName = match.groupValues[2],
                description = match.groupValues[3],
                category = match.groupValues[4]
            )
        }
    }

    private fun referencedCommands(routine: AutoRoutine): Set<String> = buildSet {
        fun visit(step: AutoStep) {
            step.commandKey?.let(::add)
            step.drive?.let { drive ->
                addAll(drive.duringCommands)
                addAll(drive.arrivalCommands)
                drive.markers.forEach { marker -> add(marker.commandKey) }
            }
            step.children.forEach(::visit)
        }
        routine.steps.forEach(::visit)
    }

    private fun autosDirectory(): File = File(projectRoot, "src/main/deploy/ares/autos")

    private fun findProjectRoot(): File = generateSequence(
        File(requireNotNull(System.getProperty("user.dir"))).canonicalFile
    ) { it.parentFile }.firstOrNull { candidate ->
        File(candidate, "src/main/deploy/ares/auto-capabilities.json").isFile
    } ?: error("Could not locate ARES-FRC project root from ${System.getProperty("user.dir")}")

    private companion object {
        val MANIFEST_ACTION = Regex(
            """\{\s*"key"\s*:\s*"([^"]+)"\s*,\s*"displayName"\s*:\s*"([^"]+)"\s*,\s*"description"\s*:\s*"([^"]+)"\s*,\s*"category"\s*:\s*"([^"]+)"\s*}""",
            RegexOption.DOT_MATCHES_ALL
        )
    }
}
