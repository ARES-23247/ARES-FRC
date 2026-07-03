package com.areslib.frc

import com.areslib.telemetry.GamepadState
import edu.wpi.first.wpilibj.XboxController

/**
 * Converts a WPILib XboxController into a platform-agnostic GamepadState
 * for the ARESLib unified logging pipeline.
 */
fun XboxController.toState() = GamepadState(
    leftStickX = leftX.toFloat(),
    leftStickY = leftY.toFloat(),
    rightStickX = rightX.toFloat(),
    rightStickY = rightY.toFloat(),
    leftTrigger = leftTriggerAxis.toFloat(),
    rightTrigger = rightTriggerAxis.toFloat(),
    a = aButton,
    b = bButton,
    x = xButton,
    y = yButton,
    dpadUp = pov == 0,
    dpadDown = pov == 180,
    dpadLeft = pov == 270,
    dpadRight = pov == 90,
    leftBumper = leftBumperButton,
    rightBumper = rightBumperButton,
    leftStickButton = leftStickButton,
    rightStickButton = rightStickButton,
    start = startButton,
    back = backButton
)
