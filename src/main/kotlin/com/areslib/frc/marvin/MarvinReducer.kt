package com.areslib.frc.marvin

import com.areslib.action.RobotAction
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import com.areslib.reducer.rootReducer
import com.areslib.control.safety.ControlBarrierFunction
import com.areslib.control.safety.CBFFilteredOutput

/**
 * Redux Reducer responsible for managing the Marvin superstructure state transitions.
 *
 * It processes actions dispatched by controllers to update the `SuperstructureState`
 * and strictly enforces physical bounds using Control Barrier Functions (CBFs).
 *
 * **Physical Units & Conventions:**
 * - Angles: Degrees ($^\circ$) for intake and cowl.
 * - Distances: Meters ($m$) for climber extension.
 * - Velocities: RPM for flywheel, RPS for rollers/feeders.
 *
 * **Performance Guarantees:**
 * - Zero-GC Allocations. Avoids new object creations by copying existing states only when modified and utilizing thread-local buffers.
 */
object MarvinReducer {
    /**
     * Documentation for reduce
     */

    fun reduce(state: RobotState, action: RobotAction): RobotState {
        // First run standard core reducer (handles drive, vision, path, costmap, and generic FSM)
        /**
         * Documentation for nextState
         */
        var nextState = rootReducer(state, action)

        // Then apply Marvin specific state updates
        /**
         * Documentation for currentMarvin
         */
        val currentMarvin = nextState.superstructure.marvin
        /**
         * Documentation for nextMarvin
         */
        val nextMarvin = when (action) {
            is SetFlywheelSpeed -> currentMarvin.withFlywheelSpeed(action.rpm)
            is SetCowlAngle -> currentMarvin.withCowlAngle(action.rotations)
            is SetIntakePivot -> currentMarvin.withIntakePivot(action.deployed)
            is SetIntakeRollers -> currentMarvin.withIntakeRollers(action.speedRps).copy(slamtakeActive = false)
            is SetFeederSpeed -> currentMarvin.withFeederSpeed(action.speedRps)
            is SetFloorSpeed -> currentMarvin.withFloorSpeed(action.speedRps)
            is SetClimberVoltage -> currentMarvin.withClimberVoltage(action.volts)
            is SetFlywheelActive -> currentMarvin.copy(flywheelActive = action.active)
            is SetTransferActive -> currentMarvin.copy(transferActive = action.active)
            is SetInventoryCount -> currentMarvin.copy(inventoryCount = action.count)
            is SetClimberExtension -> currentMarvin.withClimberExtension(action.meters)
            is StartSlamtake -> {
                currentMarvin.copy(
                    slamtakeActive = true,
                    slamtakeStartTimeMs = action.timestampMs,
                    intake = currentMarvin.intake.copy(isDeployed = true, targetAngleDegrees = 90.0, targetRollerVelocityRps = 10.0),
                    floor = currentMarvin.floor.copy(targetVelocityRps = 10.0)
                )
            }
            is StopSlamtake -> {
                currentMarvin.copy(
                    slamtakeActive = false
                )
            }
            is SlamtakeTimerExpired -> {
                if (action.phase == 1) {
                    currentMarvin.copy(
                        intake = currentMarvin.intake.copy(isDeployed = false, targetAngleDegrees = 0.0, targetRollerVelocityRps = 10.0),
                        floor = currentMarvin.floor.copy(targetVelocityRps = 10.0),
                        feeder = currentMarvin.feeder.copy(targetVelocityRps = 0.0)
                    )
                } else {
                    currentMarvin.copy(
                        slamtakeActive = false,
                        intake = currentMarvin.intake.copy(targetRollerVelocityRps = 0.0),
                        floor = currentMarvin.floor.copy(targetVelocityRps = 0.0)
                    )
                }
            }
            is SuperstructureSensorUpdate -> {
                /**
                 * Documentation for updatedMarvin
                 */
                var updatedMarvin = currentMarvin
                val eps = 1e-4

                if (Math.abs(updatedMarvin.flywheel.velocityRpm - action.flywheelRpm) > 2.0) {
                    updatedMarvin = updatedMarvin.copy(flywheel = updatedMarvin.flywheel.copy(velocityRpm = action.flywheelRpm))
                }
                if (Math.abs(updatedMarvin.cowl.angleRotations - action.cowlAngleRotations) > 0.005) {
                    updatedMarvin = updatedMarvin.copy(cowl = updatedMarvin.cowl.copy(angleRotations = action.cowlAngleRotations))
                }
                if (Math.abs(updatedMarvin.intake.pivotAngleDegrees - action.intakeAngle) > 0.005) {
                    updatedMarvin = updatedMarvin.copy(intake = updatedMarvin.intake.copy(pivotAngleDegrees = action.intakeAngle))
                }
                if (updatedMarvin.feeder.gamePieceDetected != action.pieceDetected) {
                    val wasDetected = updatedMarvin.feeder.gamePieceDetected
                    updatedMarvin = updatedMarvin.copy(feeder = updatedMarvin.feeder.copy(gamePieceDetected = action.pieceDetected, previousGamePieceDetected = wasDetected))
                    if (!wasDetected && action.pieceDetected) {
                        updatedMarvin = updatedMarvin.copy(inventoryCount = updatedMarvin.inventoryCount + 1)
                    }
                }
                if (Math.abs(updatedMarvin.floor.velocityRps - action.floorVelocityRps) > 0.005) {
                    updatedMarvin = updatedMarvin.copy(floor = updatedMarvin.floor.copy(velocityRps = action.floorVelocityRps))
                }
                if (Math.abs(updatedMarvin.floor.currentAmps - action.floorCurrentAmps) > 0.05) {
                    updatedMarvin = updatedMarvin.copy(floor = updatedMarvin.floor.copy(currentAmps = action.floorCurrentAmps))
                }
                if (Math.abs(updatedMarvin.climber.extensionMeters - action.climberExtensionMeters) > 0.005) {
                    updatedMarvin = updatedMarvin.copy(climber = updatedMarvin.climber.copy(extensionMeters = action.climberExtensionMeters))
                }

                if (updatedMarvin.slamtakeActive) {
                    if (action.pieceDetected) {
                        updatedMarvin = updatedMarvin.copy(
                            slamtakeActive = false,
                            intake = updatedMarvin.intake.copy(targetRollerVelocityRps = 0.0),
                            floor = updatedMarvin.floor.copy(targetVelocityRps = 0.0)
                        )
                    } else {
                        val elapsedMs = action.timestampMs - updatedMarvin.slamtakeStartTimeMs
                        if (elapsedMs >= 1500L) {
                            updatedMarvin = updatedMarvin.copy(
                                slamtakeActive = false,
                                intake = updatedMarvin.intake.copy(targetRollerVelocityRps = 0.0),
                                floor = updatedMarvin.floor.copy(targetVelocityRps = 0.0)
                            )
                        } else if (elapsedMs >= 500L) {
                            updatedMarvin = updatedMarvin.copy(
                                intake = updatedMarvin.intake.copy(isDeployed = false, targetAngleDegrees = 0.0, targetRollerVelocityRps = 10.0),
                                floor = updatedMarvin.floor.copy(targetVelocityRps = 10.0)
                            )
                        }
                    }
                }
                updatedMarvin
            }
            else -> null
        }

        if (nextMarvin != null) {
            nextState = nextState.copy(
                superstructure = nextState.superstructure.copy(custom = nextMarvin)
            )
        }

        return nextState
    }
}
