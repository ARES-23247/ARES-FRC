package com.areslib.frc.marvin

import com.areslib.action.RobotAction
import com.areslib.state.RobotState
import com.areslib.state.SuperstructureState
import com.areslib.reducer.rootReducer
import com.areslib.control.safety.ControlBarrierFunction
import com.areslib.control.safety.CBFFilteredOutput

object MarvinReducer {
    private val cbf = ControlBarrierFunction(
        m = 180.0, // 180 degrees per meter of extension (90 deg / 0.25 m max extension coerced to slope + clearance)
        c = 45.0,  // Minimum 45.0 deg deployment for any climber extension
        alpha = 5.0
    )
    private val cbfOutputThreadLocal = ThreadLocal.withInitial { CBFFilteredOutput() }

    fun reduce(state: RobotState, action: RobotAction): RobotState {
        // First run standard core reducer (handles drive, vision, path, costmap, and generic FSM)
        var nextState = rootReducer(state, action)

        // Then apply Marvin specific state updates
        val currentMarvin = nextState.superstructure.marvin
        val nextMarvin = when (action) {
            is SetFlywheelSpeed -> currentMarvin.withFlywheelSpeed(action.rpm)
            is SetCowlAngle -> currentMarvin.withCowlAngle(action.degrees)
            is SetIntakePivot -> currentMarvin.withIntakePivot(action.deployed)
            is SetIntakeRollers -> currentMarvin.withIntakeRollers(action.speedRps)
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
                    slamtakeStartTimeMs = action.timestampMs
                )
            }
            is StopSlamtake -> {
                currentMarvin.copy(
                    slamtakeActive = false
                )
            }
            is SuperstructureSensorUpdate -> {
                var updatedMarvin = currentMarvin.copy(
                    flywheel = currentMarvin.flywheel.copy(velocityRpm = action.flywheelRpm),
                    cowl = currentMarvin.cowl.copy(angleDegrees = action.cowlAngle),
                    intake = currentMarvin.intake.copy(pivotAngleDegrees = action.intakeAngle),
                    feeder = currentMarvin.feeder.copy(gamePieceDetected = action.pieceDetected),
                    floor = currentMarvin.floor.copy(velocityRps = action.floorVelocityRps),
                    climber = currentMarvin.climber.copy(extensionMeters = action.climberExtensionMeters)
                )

                if (updatedMarvin.slamtakeActive) {
                    val elapsed = (action.timestampMs - updatedMarvin.slamtakeStartTimeMs) / 1000.0
                    updatedMarvin = when {
                        elapsed < 0.5 -> {
                            updatedMarvin.copy(
                                intake = updatedMarvin.intake.copy(isDeployed = true, targetAngleDegrees = 90.0, targetRollerVelocityRps = 10.0),
                                floor = updatedMarvin.floor.copy(targetVelocityRps = 10.0),
                                feeder = updatedMarvin.feeder.copy(targetVelocityRps = 0.0)
                            )
                        }
                        elapsed < 1.5 -> {
                            updatedMarvin.copy(
                                intake = updatedMarvin.intake.copy(isDeployed = false, targetAngleDegrees = 0.0, targetRollerVelocityRps = 10.0),
                                floor = updatedMarvin.floor.copy(targetVelocityRps = 10.0),
                                feeder = updatedMarvin.feeder.copy(targetVelocityRps = 0.0)
                            )
                        }
                        else -> {
                            updatedMarvin.copy(
                                slamtakeActive = false,
                                intake = updatedMarvin.intake.copy(targetRollerVelocityRps = 0.0),
                                floor = updatedMarvin.floor.copy(targetVelocityRps = 0.0)
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

        // Finally, enforce Marvin specific safety interlocks (Control Barrier Function)
        return nextState.copy(
            superstructure = enforceSafetyInterlocks(nextState.superstructure)
        )
    }

    private fun enforceSafetyInterlocks(state: SuperstructureState): SuperstructureState {
        val marvin = state.marvin
        val cbfOutput = cbfOutputThreadLocal.get()
        val activeC = if (marvin.climber.extensionMeters > 0.005 || marvin.climber.targetExtensionMeters > 0.005) 45.0 else 0.0
        cbf.filter(
            x1Target = marvin.intake.targetAngleDegrees,
            x2Target = marvin.climber.targetExtensionMeters,
            x1Current = marvin.intake.pivotAngleDegrees,
            x2Current = marvin.climber.extensionMeters,
            dtSeconds = 1.0 / cbf.alpha, // Bypass velocity-rate-limiting for static setpoint actions while strictly maintaining safe invariant set boundaries
            cOverride = activeC,
            outBuffer = cbfOutput
        )

        var finalIntake = marvin.intake
        var finalClimber = marvin.climber

        if (cbfOutput.x1Filtered != marvin.intake.targetAngleDegrees) {
            finalIntake = finalIntake.copy(
                targetAngleDegrees = cbfOutput.x1Filtered,
                isDeployed = cbfOutput.x1Filtered >= 45.0
            )
        }

        if (cbfOutput.x2Filtered != marvin.climber.targetExtensionMeters) {
            finalClimber = finalClimber.copy(
                targetExtensionMeters = cbfOutput.x2Filtered
            )
        }

        if (finalIntake === marvin.intake && finalClimber === marvin.climber) {
            return state
        }
        val updatedMarvin = marvin.copy(intake = finalIntake, climber = finalClimber)
        return state.copy(custom = updatedMarvin)
    }
}
