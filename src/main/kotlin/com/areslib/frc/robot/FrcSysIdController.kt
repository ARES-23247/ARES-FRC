package com.areslib.frc.robot

import com.areslib.control.assist.FlywheelSysIdAdapter
import com.areslib.control.assist.SysIdManager
import com.areslib.control.assist.SysIdMechanism
import com.areslib.control.assist.SysIdRoutine
import com.areslib.hardware.actuator.FlywheelIO
import com.areslib.state.RobotState
import com.areslib.telemetry.ITelemetry

/** FRC test-mode executor for the shared flywheel SysId contract. */
class FrcSysIdController(
    private val telemetry: ITelemetry,
    private val flywheel: FlywheelIO
) {
    private val adapter = FlywheelSysIdAdapter(flywheel)
    private val manager = SysIdManager()
    private val sample = DoubleArray(5)
    private val emptySample = DoubleArray(0)
    private var lastCommand = ""
    private var lastTuning = com.areslib.state.MechanismTuningState()

    fun update(timestampMs: Long, state: RobotState, enabledForTuning: Boolean) {
        val tuning = state.tuning.subsystem.flywheel
        if (tuning != lastTuning) {
            flywheel.configureVelocityController(tuning.velocityGains, tuning.feedforward)
            lastTuning = tuning
        }

        val command = telemetry.getString("SysId/Command", "")
        if (command != lastCommand) {
            lastCommand = command
            manager.stop()
            adapter.stop()
            when {
                command == "STOP" || command.isBlank() -> Unit
                !enabledForTuning -> telemetry.putString("SysId/Error", "FRC_SYSID_REQUIRES_TEST_MODE")
                command.startsWith("START_FLYWHEEL_") -> {
                    val routine = runCatching {
                        SysIdRoutine.valueOf(command.removePrefix("START_FLYWHEEL_"))
                    }.getOrDefault(SysIdRoutine.NONE)
                    if (routine != SysIdRoutine.NONE) {
                        val pose = state.drive.poseEstimator.estimatedPose
                        manager.start(SysIdMechanism.FLYWHEEL, routine, timestampMs, pose.x, pose.y, pose.heading.radians)
                    }
                }
                command.startsWith("START_") -> telemetry.putString("SysId/Error", "UNSUPPORTED_FRC_MECHANISM")
            }
        }

        if (!manager.isActive()) {
            telemetry.putString("SysId/Status", "NONE")
            telemetry.putDoubleArray("SysId/Data", emptySample)
            return
        }
        val pose = state.drive.poseEstimator.estimatedPose
        if (!enabledForTuning || !adapter.measurementValid ||
            !manager.checkSafety(pose.x, pose.y, pose.heading.radians, timestampMs)) {
            manager.stop()
            adapter.stop()
            telemetry.putString("SysId/Status", "NONE")
            telemetry.putString("SysId/Error", if (!adapter.measurementValid) "INVALID_FLYWHEEL_MEASUREMENT" else "SYSID_ABORTED")
            return
        }

        val velocity = adapter.velocity
        val voltage = manager.update(timestampMs, velocity)
        adapter.setCharacterizationVoltage(voltage)
        sample[0] = timestampMs.toDouble()
        sample[1] = voltage
        sample[2] = manager.accumulatedPosition
        sample[3] = velocity
        sample[4] = manager.calculatedAcceleration
        telemetry.putString("SysId/Status", manager.activeRoutine.name)
        telemetry.putDoubleArray("SysId/Data", sample)
    }

    fun stop() {
        manager.stop()
        adapter.stop()
    }
}
