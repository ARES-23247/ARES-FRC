package com.areslib.frc

import com.areslib.frc.marvin.MarvinConfig
import com.areslib.frc.robot.FRCTeleOpDriveController
import com.areslib.math.coordinate.CoordinateTransformers
import com.areslib.state.Alliance
import com.areslib.util.RobotClock
import edu.wpi.first.hal.AllianceStationID
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class ARESRobotTimedBehaviorRegressionTest {

    private var timedRobot: ARESRobot? = null

    @BeforeEach
    fun setUp() {
        assertTrue(HAL.initialize(500, 0))
        RobotClock.useMockTime(1_000L)
        DriverStationSim.resetData()
        DriverStationSim.setDsAttached(true)
        DriverStationSim.setEnabled(false)
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1)
        DriverStationSim.notifyNewData()
    }

    @AfterEach
    fun tearDown() {
        timedRobot?.close()
        timedRobot = null
        DriverStationSim.resetData()
        RobotClock.useSystemTime()
    }

    @Test
    fun `TimedRobot propagates alliance and simulation ground truth into Redux state`() {
        val outer = ARESRobot()
        timedRobot = outer
        outer.robotInit()

        val facade = privateField<FrcSwerveRobot>(outer, "robot")
        val teleop = privateField<FRCTeleOpDriveController>(outer, "teleOpController")
        val sim = privateField<Dyn4jSimulation>(outer, "sim")

        assertEquals(Alliance.RED, facade.store.state.drive.alliance)
        assertEquals(MarvinConfig.FieldTargets.redSpeaker, teleop.speakerTranslation)

        sim.resetPose(4.0, 3.0, 0.75)
        RobotClock.setMockTimeMs(1_050L)
        outer.simulationPeriodic()
        assertEquals(4.0, facade.store.state.drive.odometryX, 1e-6)
        assertEquals(3.0, facade.store.state.drive.odometryY, 1e-6)
        assertEquals(0.75, facade.store.state.drive.odometryHeading, 1e-6)

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)
        DriverStationSim.notifyNewData()
        outer.robotPeriodic()
        assertEquals(Alliance.BLUE, facade.store.state.drive.alliance)
        assertEquals(MarvinConfig.FieldTargets.blueSpeaker, teleop.speakerTranslation)
    }

    @Test
    fun `canonical FRC field extents match official Crescendo dimensions`() {
        assertEquals(651.25 * 0.0254, CoordinateTransformers.FRC_FIELD_LENGTH, 1e-9)
        assertEquals(323.25 * 0.0254, CoordinateTransformers.FRC_FIELD_WIDTH, 1e-9)
    }

    @Suppress("UNCHECKED_CAST")
    private fun <T> privateField(owner: ARESRobot, name: String): T {
        return ARESRobot::class.java.getDeclaredField(name).run {
            isAccessible = true
            get(owner) as T
        }
    }
}
