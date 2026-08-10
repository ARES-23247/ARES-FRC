package com.areslib.frc.marvin

import com.areslib.action.RobotAction
import com.areslib.state.RobotState
import com.areslib.reducer.rootReducer

/**
 * Redux Reducer responsible for managing the Marvin superstructure state transitions.
 *
 * Composes over the core [rootReducer] (which handles drive, vision, pathing, costmap,
 * and the generic FSM) and then applies Marvin-specific state updates for each season
 * action. The reducer is pure: it records commanded targets and cached sensor observations but
 * performs no IO. Physical bounds (e.g. joint travel limits) are enforced downstream by controller facades
 * (e.g. [MarvinCowlController]) and by TalonFX soft limits in the hardware IO layer, not
 * here.
 *
 * **Physical Units & Conventions:**
 * - Angles: Degrees ($^\circ$) for intake and rotations for cowl.
 * - Climber position: Mechanism rotations.
 * - Velocities: RPM for flywheel, RPS for rollers/feeders.
 *
 * Sensor observations use deadbands to avoid copying the state tree for insignificant
 * changes. Freshness transitions always bypass those numeric deadbands so invalid data
 * cannot remain authoritative.
 */
object MarvinReducer {

    /** Applies the core reducer first, then the Marvin-specific state transition. */
    fun reduce(state: RobotState, action: RobotAction): RobotState {
        // First run standard core reducer (handles drive, vision, path, costmap, and generic FSM)
        var nextState = rootReducer(state, action)

        // Then apply Marvin specific state updates
        val currentMarvin = nextState.superstructure.marvin
        val nextMarvin = when (action) {
            is SetFlywheelSpeed -> currentMarvin.withFlywheelSpeed(action.rpm)
            is SetCowlAngle -> currentMarvin.withCowlAngle(action.rotations)
            is SetIntakePivot -> currentMarvin.withIntakePivot(action.deployed)
            is SetIntakeRollers -> currentMarvin.withIntakeRollers(action.speedRps).copy(slamtakeActive = false, slamtakePhase = 0)
            is SetFeederSpeed -> currentMarvin.withFeederSpeed(action.speedRps)
            is SetFloorSpeed -> currentMarvin.withFloorSpeed(action.speedRps)
            is SetClimberVoltage -> currentMarvin.withClimberVoltage(action.volts)
            is SetFlywheelActive -> currentMarvin.copy(flywheelActive = action.active)
            is SetTransferActive -> currentMarvin.copy(transferActive = action.active)
            is SetInventoryCount -> currentMarvin.copy(inventoryCount = action.count)
            is SetClimberPositionRotations -> currentMarvin.withClimberPositionRotations(action.rotations)
            is StartSlamtake -> {
                currentMarvin.copy(
                    slamtakeActive = true,
                    slamtakePhase = 1,
                    slamtakeStartTimeMs = action.timestampMs,
                    intake = currentMarvin.intake.copy(isDeployed = true, targetAngleDegrees = 90.0, targetRollerVelocityRps = 10.0),
                    floor = currentMarvin.floor.copy(targetVelocityRps = 10.0)
                )
            }
            is StopSlamtake -> {
                currentMarvin.copy(
                    slamtakeActive = false,
                    slamtakePhase = 0
                )
            }
            is SlamtakeTimerExpired -> {
                if (action.phase == 1) {
                    currentMarvin.copy(
                        slamtakePhase = 2,
                        intake = currentMarvin.intake.copy(isDeployed = false, targetAngleDegrees = 0.0, targetRollerVelocityRps = 10.0),
                        floor = currentMarvin.floor.copy(targetVelocityRps = 10.0),
                        feeder = currentMarvin.feeder.copy(targetVelocityRps = 0.0)
                    )
                } else {
                    currentMarvin.copy(
                        slamtakeActive = false,
                        slamtakePhase = 0,
                        intake = currentMarvin.intake.copy(targetRollerVelocityRps = 0.0),
                        floor = currentMarvin.floor.copy(targetVelocityRps = 0.0)
                    )
                }
            }
            is SuperstructureSensorUpdate -> {
                var updatedMarvin = currentMarvin

                val flywheelVelocityValid = action.flywheelVelocityValid && action.flywheelRpm.isFinite()
                if (Math.abs(updatedMarvin.flywheel.velocityRpm - action.flywheelRpm) > 2.0 ||
                    updatedMarvin.flywheel.velocityValid != flywheelVelocityValid) {
                    updatedMarvin = updatedMarvin.copy(flywheel = updatedMarvin.flywheel.copy(
                        velocityRpm = if (flywheelVelocityValid) action.flywheelRpm else 0.0,
                        velocityValid = flywheelVelocityValid
                    ))
                }
                val cowlAngleValid = action.cowlAngleValid && action.cowlAngleRotations.isFinite()
                val cowlAngle = if (cowlAngleValid) action.cowlAngleRotations else 0.0
                if (Math.abs(updatedMarvin.cowl.angleRotations - cowlAngle) > 0.005 ||
                    updatedMarvin.cowl.angleValid != cowlAngleValid) {
                    updatedMarvin = updatedMarvin.copy(cowl = updatedMarvin.cowl.copy(
                        angleRotations = cowlAngle,
                        angleValid = cowlAngleValid
                    ))
                }
                val intakeAngleValid = action.intakeAngleValid && action.intakeAngle.isFinite()
                val intakeAngle = if (intakeAngleValid) action.intakeAngle else 0.0
                if (Math.abs(updatedMarvin.intake.pivotAngleDegrees - intakeAngle) > 0.005 ||
                    updatedMarvin.intake.pivotAngleValid != intakeAngleValid) {
                    updatedMarvin = updatedMarvin.copy(intake = updatedMarvin.intake.copy(
                        pivotAngleDegrees = intakeAngle,
                        pivotAngleValid = intakeAngleValid
                    ))
                }
                if (!action.pieceDetectionValid) {
                    if (updatedMarvin.feeder.pieceDetectionValid || updatedMarvin.feeder.gamePieceDetected) {
                        updatedMarvin = updatedMarvin.copy(feeder = updatedMarvin.feeder.copy(
                            gamePieceDetected = false,
                            previousGamePieceDetected = updatedMarvin.feeder.gamePieceDetected,
                            pieceDetectionValid = false
                        ))
                    }
                } else if (!updatedMarvin.feeder.pieceDetectionValid || updatedMarvin.feeder.gamePieceDetected != action.pieceDetected) {
                    // Preserve the last trusted reading across a transient invalid
                    // interval so detector recovery cannot count the same piece twice.
                    val wasDetected = if (updatedMarvin.feeder.pieceDetectionValid) {
                        updatedMarvin.feeder.gamePieceDetected
                    } else {
                        updatedMarvin.feeder.previousGamePieceDetected
                    }
                    updatedMarvin = updatedMarvin.copy(feeder = updatedMarvin.feeder.copy(
                        gamePieceDetected = action.pieceDetected,
                        previousGamePieceDetected = wasDetected,
                        pieceDetectionValid = true
                    ))
                    if (!wasDetected && action.pieceDetected) {
                        updatedMarvin = updatedMarvin.copy(inventoryCount = updatedMarvin.inventoryCount + 1)
                    } else if (wasDetected && !action.pieceDetected && updatedMarvin.transferActive) {
                        updatedMarvin = updatedMarvin.copy(inventoryCount = (updatedMarvin.inventoryCount - 1).coerceAtLeast(0))
                    }
                }
                val floorVelocityRps = action.floorVelocityRps.takeIf { it.isFinite() } ?: 0.0
                if (Math.abs(updatedMarvin.floor.velocityRps - floorVelocityRps) > 0.005) {
                    updatedMarvin = updatedMarvin.copy(floor = updatedMarvin.floor.copy(velocityRps = floorVelocityRps))
                }
                val floorCurrentAmps = action.floorCurrentAmps.takeIf { it.isFinite() && it >= 0.0 } ?: 0.0
                if (Math.abs(updatedMarvin.floor.currentAmps - floorCurrentAmps) > 0.05) {
                    updatedMarvin = updatedMarvin.copy(floor = updatedMarvin.floor.copy(currentAmps = floorCurrentAmps))
                }
                val climberPositionValid = action.climberPositionValid && action.climberPositionRotations.isFinite()
                val climberPosition = if (climberPositionValid) action.climberPositionRotations else 0.0
                if (Math.abs(updatedMarvin.climber.positionRotations - climberPosition) > 0.005 ||
                    updatedMarvin.climber.positionValid != climberPositionValid) {
                    updatedMarvin = updatedMarvin.copy(climber = updatedMarvin.climber.copy(
                        positionRotations = climberPosition,
                        positionValid = climberPositionValid
                    ))
                }

                if (updatedMarvin.slamtakeActive) {
                    if (action.pieceDetectionValid && action.pieceDetected) {
                        updatedMarvin = updatedMarvin.copy(
                            slamtakeActive = false,
                            slamtakePhase = 0,
                            intake = updatedMarvin.intake.copy(targetRollerVelocityRps = 0.0),
                            floor = updatedMarvin.floor.copy(targetVelocityRps = 0.0)
                        )
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
