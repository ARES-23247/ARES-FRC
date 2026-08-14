package com.areslib.frc.marvin

import com.areslib.Store
import com.areslib.frc.hardware.ClimberIO
import com.areslib.frc.hardware.CowlIO
import com.areslib.frc.hardware.FeederIO
import com.areslib.frc.hardware.FloorIO
import com.areslib.frc.hardware.FlywheelIO
import com.areslib.frc.hardware.IntakeIO
import com.areslib.state.DriveState
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import com.areslib.util.RobotClock
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class MarvinIntakeSubsystemTest {

    private class RecordingIntakeIO : IntakeIO {
        var pivotAngleCommand = Double.NaN
        var pivotEffortScale = Double.NaN
        var pivotVoltageCommand = Double.NaN
        var rollerVoltageCommand = Double.NaN
        var rollerVelocityCommand = Double.NaN

        override var pivotAngleDegrees: Double = 0.0
        override var pivotAngleValid: Boolean = true
        override var currentAmps: Double = 0.0
        override var rollerCurrentAmps: Double = 0.0
        override var rollerCurrentValid: Boolean = true

        override fun setPivotAngle(degrees: Double) {
            pivotAngleCommand = degrees
        }

        override fun setPivotAngle(degrees: Double, maxEffortScale: Double) {
            pivotAngleCommand = degrees
            pivotEffortScale = maxEffortScale
        }

        override fun setPivotVoltage(volts: Double) {
            pivotVoltageCommand = volts
        }

        override fun setRollerVoltage(volts: Double) {
            rollerVoltageCommand = volts
        }

        override fun setRollerVelocityRps(rps: Double) {
            rollerVelocityCommand = rps
        }
    }

    private class RecordingFlywheelIO : FlywheelIO {
        var velocityRpmCommand = Double.NaN
        var appliedVoltageCommand = Double.NaN
        override val velocityRpm: Double get() = 0.0
        override val velocityValid: Boolean get() = true
        override val currentAmps: Double get() = 0.0
        override val tempCelsius: Double get() = 25.0

        override fun setVelocityRpm(rpm: Double) {
            velocityRpmCommand = rpm
        }

        override fun setAppliedVoltage(volts: Double) {
            appliedVoltageCommand = volts
        }
    }

    private class RecordingCowlIO : CowlIO {
        var angleCommand = Double.NaN
        var effortScale = Double.NaN
        var voltageCommand = Double.NaN
        override var angleRotations: Double = 0.0
        override var angleValid: Boolean = true
        override var currentAmps: Double = 0.0

        override fun setTargetAngle(rotations: Double) {
            angleCommand = rotations
        }

        override fun setTargetAngle(rotations: Double, maxEffortScale: Double) {
            angleCommand = rotations
            effortScale = maxEffortScale
        }

        override fun setAppliedVoltage(volts: Double) {
            voltageCommand = volts
        }
    }

    private class RecordingFeederIO : FeederIO {
        var voltageCommand = Double.NaN
        override var isBeamBroken: Boolean = false
        override var pieceDetectionValid: Boolean = false
        override var currentAmps: Double = 0.0

        override fun setAppliedVoltage(volts: Double) {
            voltageCommand = volts
        }
    }

    private class RecordingFloorIO : FloorIO {
        var voltageCommand = Double.NaN
        override var velocityRps: Double = 0.0
        override var currentAmps: Double = 0.0

        override fun setAppliedVoltage(volts: Double) {
            voltageCommand = volts
        }
    }

    private class RecordingClimberIO : ClimberIO {
        var voltageCommand = Double.NaN
        var positionCommandRotations = Double.NaN
        var effortScale = Double.NaN
        override var positionRotations: Double = 0.0
        override var positionValid: Boolean = true
        override var currentAmps: Double = 0.0

        override fun setTargetPositionRotations(rotations: Double) {
            positionCommandRotations = rotations
        }

        override fun setTargetPositionRotations(rotations: Double, maxEffortScale: Double) {
            positionCommandRotations = rotations
            effortScale = maxEffortScale
        }

        override fun setAppliedVoltage(volts: Double) {
            voltageCommand = volts
        }
    }

    private lateinit var store: Store
    private lateinit var intakeIO: RecordingIntakeIO
    private lateinit var flywheelIO: RecordingFlywheelIO
    private lateinit var cowlIO: RecordingCowlIO
    private lateinit var feederIO: RecordingFeederIO
    private lateinit var floorIO: RecordingFloorIO
    private lateinit var climberIO: RecordingClimberIO
    private lateinit var superstructure: MarvinSuperstructure
    private lateinit var intakeSubsystem: MarvinIntakeSubsystem

    @BeforeEach
    fun setUp() {
        RobotClock.useMockTime(1_000L)
        store = Store(
            RobotState(
                drive = DriveState(measuredMotionValid = true),
                superstructure = SuperstructureState(custom = MarvinState())
            )
        ) { state, action -> MarvinReducer.reduce(state, action) }

        intakeIO = RecordingIntakeIO().apply {
            pivotAngleDegrees = 0.0
            pivotAngleValid = true
        }
        flywheelIO = RecordingFlywheelIO()
        cowlIO = RecordingCowlIO().apply {
            angleRotations = 0.0
            angleValid = true
        }
        feederIO = RecordingFeederIO()
        floorIO = RecordingFloorIO()
        climberIO = RecordingClimberIO().apply {
            positionRotations = 0.0
            positionValid = true
        }

        superstructure = MarvinSuperstructure(
            flywheelIO, cowlIO, intakeIO, feederIO, floorIO, climberIO
        )
        intakeSubsystem = MarvinIntakeSubsystem(store)

        // Populate initial valid sensor state in Redux store
        superstructure.readSensors(store, 1_000L)
    }

    @AfterEach
    fun tearDown() {
        RobotClock.useSystemTime()
    }

    @Test
    fun `deploy action dispatches deployed pivot target to Redux store and forwards to IntakeIO`() {
        // Initial state is stowed
        assertFalse(intakeSubsystem.isDeployed)
        assertEquals(0.0, intakeSubsystem.targetAngleDegrees)
        assertEquals(0.0, intakeSubsystem.targetRollerVelocityRps)

        // Deploy intake
        intakeSubsystem.deploy()

        // Verify Redux store state updated
        assertTrue(intakeSubsystem.isDeployed, "Intake must be marked deployed in store")
        assertEquals(90.0, intakeSubsystem.targetAngleDegrees, 1e-4, "Target angle must be 90.0 degrees")
        assertTrue(store.state.superstructure.marvin.intake.isDeployed)
        assertEquals(90.0, store.state.superstructure.marvin.intake.targetAngleDegrees, 1e-4)

        // Forward outputs to IntakeIO
        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(90.0, intakeIO.pivotAngleCommand, 1e-4, "IntakeIO must receive 90.0 degrees pivot target")
        assertEquals(1.0, intakeIO.pivotEffortScale, 1e-4, "IntakeIO effort scale must be 1.0")
    }

    @Test
    fun `stow action dispatches stowed pivot target to Redux store and forwards to IntakeIO`() {
        // Deploy first
        intakeSubsystem.deploy()
        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(90.0, intakeIO.pivotAngleCommand, 1e-4)

        // Stow intake
        intakeSubsystem.stow()

        // Verify Redux store state updated
        assertFalse(intakeSubsystem.isDeployed, "Intake must be marked stowed in store")
        assertEquals(0.0, intakeSubsystem.targetAngleDegrees, 1e-4, "Target angle must be 0.0 degrees")
        assertFalse(store.state.superstructure.marvin.intake.isDeployed)
        assertEquals(0.0, store.state.superstructure.marvin.intake.targetAngleDegrees, 1e-4)

        // Forward outputs to IntakeIO
        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(0.0, intakeIO.pivotAngleCommand, 1e-4, "IntakeIO must receive 0.0 degrees pivot target")
    }

    @Test
    fun `setRollers action dispatches roller velocity to Redux store and forwards to IntakeIO`() {
        val targetRps = 12.5

        // Set roller velocity
        intakeSubsystem.setRollers(targetRps)

        // Verify Redux store state updated
        assertEquals(targetRps, intakeSubsystem.targetRollerVelocityRps, 1e-4)
        assertEquals(targetRps, store.state.superstructure.marvin.intake.targetRollerVelocityRps, 1e-4)

        // Forward outputs to IntakeIO
        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(targetRps, intakeIO.rollerVelocityCommand, 1e-4, "IntakeIO must receive roller velocity target")
    }

    @Test
    fun `collect action deploys pivot and commands forward roller speed`() {
        val collectRps = 15.0

        intakeSubsystem.collect(collectRps)

        // Verify both pivot and roller targets in Redux store
        assertTrue(intakeSubsystem.isDeployed)
        assertEquals(90.0, intakeSubsystem.targetAngleDegrees, 1e-4)
        assertEquals(collectRps, intakeSubsystem.targetRollerVelocityRps, 1e-4)

        // Forward to IntakeIO
        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(90.0, intakeIO.pivotAngleCommand, 1e-4)
        assertEquals(collectRps, intakeIO.rollerVelocityCommand, 1e-4)
    }

    @Test
    fun `unjam action deploys pivot and commands reverse roller speed`() {
        val unjamRps = 6.0

        intakeSubsystem.unjam(unjamRps)

        // Verify unjam inverts positive input to negative RPS
        assertTrue(intakeSubsystem.isDeployed)
        assertEquals(90.0, intakeSubsystem.targetAngleDegrees, 1e-4)
        assertEquals(-6.0, intakeSubsystem.targetRollerVelocityRps, 1e-4)

        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(90.0, intakeIO.pivotAngleCommand, 1e-4)
        assertEquals(-6.0, intakeIO.rollerVelocityCommand, 1e-4)
    }

    @Test
    fun `stopRollers stops roller speed while preserving pivot deployment`() {
        intakeSubsystem.collect(10.0)
        assertTrue(intakeSubsystem.isDeployed)
        assertEquals(10.0, intakeSubsystem.targetRollerVelocityRps, 1e-4)

        intakeSubsystem.stopRollers()

        assertTrue(intakeSubsystem.isDeployed, "Pivot must remain deployed")
        assertEquals(90.0, intakeSubsystem.targetAngleDegrees, 1e-4)
        assertEquals(0.0, intakeSubsystem.targetRollerVelocityRps, 1e-4, "Roller speed must be 0.0")

        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(90.0, intakeIO.pivotAngleCommand, 1e-4)
        assertEquals(0.0, intakeIO.rollerVelocityCommand, 1e-4)
    }

    @Test
    fun `stopAndStow stops roller speed and stows pivot`() {
        intakeSubsystem.collect(10.0)

        intakeSubsystem.stopAndStow()

        assertFalse(intakeSubsystem.isDeployed, "Intake must be stowed")
        assertEquals(0.0, intakeSubsystem.targetAngleDegrees, 1e-4)
        assertEquals(0.0, intakeSubsystem.targetRollerVelocityRps, 1e-4)

        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(0.0, intakeIO.pivotAngleCommand, 1e-4)
        assertEquals(0.0, intakeIO.rollerVelocityCommand, 1e-4)
    }

    @Test
    fun `collect and stopAndStow coordinate pivot deployment and roller speed commands across Redux store and writeOutputs`() {
        intakeSubsystem.collect(45.0)

        assertTrue(intakeSubsystem.isDeployed)
        assertEquals(90.0, intakeSubsystem.targetAngleDegrees, 1e-4)
        assertEquals(45.0, intakeSubsystem.targetRollerVelocityRps, 1e-4)

        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(90.0, intakeIO.pivotAngleCommand, 1e-4)
        assertEquals(45.0, intakeIO.rollerVelocityCommand, 1e-4)

        intakeSubsystem.stopAndStow()

        assertFalse(intakeSubsystem.isDeployed)
        assertEquals(0.0, intakeSubsystem.targetAngleDegrees, 1e-4)
        assertEquals(0.0, intakeSubsystem.targetRollerVelocityRps, 1e-4)

        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(0.0, intakeIO.pivotAngleCommand, 1e-4)
        assertEquals(0.0, intakeIO.rollerVelocityCommand, 1e-4)
    }

    @Test
    fun `brownout power scaling scales roller velocity and pivot effort while preserving geometry`() {
        intakeSubsystem.collect(12.0)

        // Write outputs with 50% power budget
        superstructure.writeOutputs(store.state, scale = 0.5)

        // Target angle remains 90.0 degrees (geometry preserved), but effort scale is 0.5
        assertEquals(90.0, intakeIO.pivotAngleCommand, 1e-4, "Target angle must not move under brownout")
        assertEquals(0.5, intakeIO.pivotEffortScale, 1e-4, "Effort scale must be passed to IO")

        // Roller velocity scaled to 50% (12.0 * 0.5 = 6.0)
        assertEquals(6.0, intakeIO.rollerVelocityCommand, 1e-4, "Roller velocity must be scaled by power budget")
    }

    @Test
    fun `invalid pivot sensor reading fails closed to zero voltage output`() {
        intakeSubsystem.deploy()

        // Pivot angle sensor marked invalid
        intakeIO.pivotAngleValid = false
        superstructure.readSensors(store, 1_000L)
        assertFalse(intakeSubsystem.pivotAngleValid)
        assertEquals(0.0, intakeSubsystem.pivotAngleDegrees, 1e-4, "Invalid pivot reading zeroed by reducer")

        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(0.0, intakeIO.pivotVoltageCommand, 1e-4, "Invalid pivot must command 0V output")
    }

    @Test
    fun `mechanism safety inhibit and latched fault zero all intake outputs`() {
        intakeSubsystem.collect(10.0)

        // Assert temporary mechanism safety inhibit
        store.dispatch(SetMechanismSafetyInhibit(inhibited = true))
        assertTrue(store.state.superstructure.marvin.mechanismSafetyInhibited)

        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(0.0, intakeIO.pivotVoltageCommand, 1e-4, "Pivot voltage must be 0V when inhibited")
        assertEquals(0.0, intakeIO.rollerVoltageCommand, 1e-4, "Roller voltage must be 0V when inhibited")

        // Latch persistent mechanism safety fault
        store.dispatch(LatchMechanismSafetyFault(reason = "Intake test safety fault"))
        assertTrue(store.state.superstructure.marvin.mechanismSafetyFaultLatched)

        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(0.0, intakeIO.pivotVoltageCommand, 1e-4, "Pivot voltage must be 0V when faulted")
        assertEquals(0.0, intakeIO.rollerVoltageCommand, 1e-4, "Roller voltage must be 0V when faulted")
    }

    @Test
    fun `climber collision arbitration forces intake pivot to stowed position`() {
        intakeSubsystem.deploy()
        assertEquals(90.0, intakeSubsystem.targetAngleDegrees)

        // Set climber extended past clearance limit (blocks intake)
        climberIO.positionRotations = 0.50
        climberIO.positionValid = true
        superstructure.readSensors(store, 1_000L)

        // When climber blocks intake, requested pivot angle 90.0 is overridden to safe stowed 0.0
        superstructure.writeOutputs(store.state, scale = 1.0)
        assertEquals(0.0, intakeIO.pivotAngleCommand, 1e-4, "Intake must be forced stowed when climber blocks it")
        assertEquals(90.0, intakeSubsystem.targetAngleDegrees, 1e-4, "Redux state target angle remains intact")
    }

    @Test
    fun `pivot alignment check respects angular tolerances and sensor validity`() {
        intakeSubsystem.deploy()
        assertEquals(90.0, intakeSubsystem.targetAngleDegrees)

        // Valid sensor reading exactly at target (90.0 degrees)
        intakeIO.pivotAngleDegrees = 90.0
        intakeIO.pivotAngleValid = true
        superstructure.readSensors(store, 1_000L)
        assertTrue(intakeSubsystem.pivotAngleValid)
        assertEquals(90.0, intakeSubsystem.pivotAngleDegrees, 1e-4)
        assertTrue(intakeSubsystem.isPivotAligned(), "Pivot must be aligned at target")
        assertTrue(intakeSubsystem.isDeployedAndAligned, "Must be deployed and aligned")

        // Within 5 degree tolerance (93.0 degrees)
        intakeIO.pivotAngleDegrees = 93.0
        superstructure.readSensors(store, 1_020L)
        assertTrue(intakeSubsystem.isPivotAligned(), "Pivot at 93 deg is within 5 deg tolerance")

        // Outside tolerance (96.0 degrees)
        intakeIO.pivotAngleDegrees = 96.0
        superstructure.readSensors(store, 1_040L)
        assertFalse(intakeSubsystem.isPivotAligned(), "Pivot at 96 deg is outside 5 deg tolerance")
        assertFalse(intakeSubsystem.isDeployedAndAligned)

        // Boundary condition: exactly 5.0 deg error is within tolerance (<= 5.0)
        intakeIO.pivotAngleDegrees = 95.0
        superstructure.readSensors(store, 1_060L)
        assertTrue(intakeSubsystem.isPivotAligned(), "Exact 5.0 deg error is within tolerance")

        // Boundary condition: 5.1 deg error is outside tolerance
        intakeIO.pivotAngleDegrees = 95.1
        superstructure.readSensors(store, 1_080L)
        assertFalse(intakeSubsystem.isPivotAligned(), "5.1 deg error exceeds tolerance")

        // Invalid sensor reading fails closed
        intakeIO.pivotAngleDegrees = 90.0
        intakeIO.pivotAngleValid = false
        superstructure.readSensors(store, 1_100L)
        assertFalse(intakeSubsystem.isPivotAligned(), "Invalid sensor reading must fail closed")
        assertFalse(intakeSubsystem.isDeployedAndAligned)
    }

    @Test
    fun `sensor updates dispatch cached readings to Redux store`() {
        intakeIO.pivotAngleDegrees = 45.0
        intakeIO.pivotAngleValid = true
        superstructure.readSensors(store, 1_000L)

        assertEquals(45.0, intakeSubsystem.pivotAngleDegrees, 1e-4)
        assertTrue(intakeSubsystem.pivotAngleValid)
        assertEquals(45.0, store.state.superstructure.marvin.intake.pivotAngleDegrees, 1e-4)
        assertTrue(store.state.superstructure.marvin.intake.pivotAngleValid)
    }
}
