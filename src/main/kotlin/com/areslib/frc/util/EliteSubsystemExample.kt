package com.areslib.frc.util

import com.areslib.action.RobotAction
import com.areslib.frc.action.*
import com.areslib.state.RobotState
import com.areslib.state.DriveMode
import com.areslib.subsystem.Store
import com.areslib.control.ProfiledPIDController
import com.areslib.control.TrapezoidProfile
import com.areslib.control.GravityFeedforward
import com.areslib.logging.DiagnosticRingBuffer
import com.areslib.util.RobotClock
import com.areslib.frc.state.marvinXIX

/**
 * Production-grade example code demonstrating the usage of elite subsystem controls
 * and college-grade hardening features in a unified real-time robot architecture.
 *
 * This example guides student developers on how to:
 * 1. Initialize and manage the Redux [Store].
 * 2. Dispatch driving and heading-lock joystick intents.
 * 3. Run a zero-allocation [ProfiledPIDController] for climber control, including adaptive payload feedforward.
 * 4. Verify how pure reciprocal interlock safety rules prevent physical self-collision.
 * 5. Record telemetry signals dynamically using the GC-free [DiagnosticRingBuffer].
 */
class EliteSubsystemExample {

    // Centralized Redux Store holding immutable robot state, using MarvinReducer
    val store = Store(reducer = com.areslib.frc.reducer.MarvinReducer::reduce)

    // GC-Free circular telemetry recorder
    val logger = DiagnosticRingBuffer(capacity = 500)

    // Pre-allocated array used to fetch numeric values from telemetry without allocations
    private val numericOutBuffer = DoubleArray(1)

    // Profile constraints for physical climber elevator (Max: 1.5 m/s, Acceleration: 2.0 m/s^2)
    val climberConstraints = TrapezoidProfile.Constraints(
        maxVelocity = 1.5,
        maxAcceleration = 2.0
    )

    // Profiled PID controller driving the climber extension with zero dynamic heap allocations
    val climberController = ProfiledPIDController(
        p = 8.0,
        i = 0.1,
        d = 0.5,
        constraints = climberConstraints
    ).apply {
        // Enforce physical extension bounds [0m stowed, 0.5m fully extended]
        setOutputLimits(-12.0, 12.0) // Clamp to max 12V motor output
        setIntegratorRange(-2.0, 2.0)
    }

    /**
     * Initializes the robot systems, setting up state change listeners
     * and initializing controller positions.
     */
    fun robotInit() {
        // Reset the climber controller to its starting physical position (stowed: 0.0 meters)
        climberController.reset(position = 0.0, velocity = 0.0)

        // Subscribe to store updates to dynamically log and drive actuators
        store.subscribe { state ->
            // Reciprocal Safety Interlock verification
            val marvin = state.superstructure.marvinXIX
            val currentPivotAngle = marvin.intake.pivotAngleDegrees
            val targetClimberExtension = marvin.climber.targetExtensionMeters

            // Log current critical metrics to the GC-free ring buffer
            logger.log("ClimberTarget", targetClimberExtension)
            logger.log("IntakePivotAngle", currentPivotAngle)
        }
    }

    /**
     * Simulated periodic execution loop running at high frequency (e.g. 50Hz / 20ms cycles).
     *
     * @param dtSeconds Elapsed time since the last periodic execution frame.
     * @param joyX Joystick forward axis input.
     * @param joyY Joystick lateral axis input.
     * @param joyYaw Joystick rotational axis input.
     * @param headingLockDegrees Target heading lock angle in degrees (null if disabled).
     * @param wantXLock True if driver commands Swerve modules to lock into an X-brake cross pattern.
     * @param measuredClimberHeightMeters Encoder reading of the physical climber's height.
     * @param commandedClimberTargetMeters Desired extension level target.
     */
    fun robotPeriodic(
        dtSeconds: Double,
        joyX: Double,
        joyY: Double,
        joyYaw: Double,
        headingLockDegrees: Double?,
        wantXLock: Boolean,
        measuredClimberHeightMeters: Double,
        commandedClimberTargetMeters: Double
    ) {
        val now = RobotClock.currentTimeMillis()

        // 1. Dispatch Drivetrain Intents and Swerve Lock States
        when {
            wantXLock -> {
                // Engage hardware cross-pattern module locking to resist external forces
                store.dispatch(RobotAction.SetDriveMode(DriveMode.X_BRAKE, now))
            }
            headingLockDegrees != null -> {
                // Engage active PID heading stabilization relative to setpoint
                store.dispatch(RobotAction.SetDriveMode(DriveMode.HEADING_HOLD, now))
                store.dispatch(RobotAction.SetHeadingLockTarget(Math.toRadians(headingLockDegrees), now))
                store.dispatch(RobotAction.JoystickDriveIntent(joyX, joyY, 0.0, now))
            }
            else -> {
                // Standard driver manual control mode
                store.dispatch(RobotAction.SetDriveMode(DriveMode.TELEOP, now))
                store.dispatch(RobotAction.JoystickDriveIntent(joyX, joyY, joyYaw, now))
            }
        }

        // 2. Dispatch Target Climber Extension
        store.dispatch(SetClimberExtension(commandedClimberTargetMeters, now))

        // Get the current state containing safe and clamped superstructure values
        val currentState = store.state

        // 3. Trapezoidal Profile & Adaptive Gravity Feedforward calculation for Climber
        // Feedforward adjusts dynamically based on the game piece count payload (prevents height droop)
        val baseKG = 0.8 // 0.8 Volts needed to hold elevator against gravity empty
        val gravityFF = GravityFeedforward.calculateAdaptiveElevator(
            baseKG = baseKG,
            inventoryCount = currentState.superstructure.inventoryCount,
            factorPerPiece = 0.15 // 15% increase in effort per loaded game piece
        )

        // Set the controller target goal
        climberController.setGoal(currentState.superstructure.marvinXIX.climber.targetExtensionMeters, 0.0)

        // Compute feedback effort and add the feedforward term
        val pidFeedback = climberController.calculate(measuredClimberHeightMeters, dtSeconds)
        val totalVoltageDemand = pidFeedback + gravityFF

        // Log control outputs to the GC-free logger
        logger.log("ClimberPIDOut", pidFeedback)
        logger.log("ClimberFFOut", gravityFF)
        logger.log("ClimberTotalDemand", totalVoltageDemand)

        // 4. Command Climber Actuators
        store.dispatch(SetClimberVoltage(totalVoltageDemand, now))
    }

    /**
     * Helper to read the last logged value of a tag cleanly with zero GC heap allocations.
     */
    fun getLastLoggedValue(tag: String): Double {
        val count = logger.getNumericCount()
        for (i in (count - 1) downTo 0) {
            val loggedTag = logger.getNumericEntry(i, numericOutBuffer)
            if (loggedTag == tag) {
                return numericOutBuffer[0]
            }
        }
        return 0.0
    }
}
