package com.areslib.frc

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.*

class AresFrcRemediationTest {
    @Test
    fun testShooterRearwardHeading() {
        val targetX = 1.0
        val targetY = 1.0
        val currentX = 0.0
        val currentY = 0.0

        // The target bearing is pi/4; a rearward shooter adds pi and wraps to -3pi/4.
        var targetHeadingRad = Math.atan2(targetY - currentY, targetX - currentX)
        if (com.areslib.frc.marvin.MarvinConfig.SHOT_CONFIG.shooterFacesRearward) {
            targetHeadingRad = com.areslib.math.wrapAngle(targetHeadingRad + Math.PI)
        }
        assertEquals(-2.35619, targetHeadingRad, 0.001, "Target heading should be inverted and wrapped")
    }

    @Test
    fun testCowlAngleUsesRotations() {
        // Keep the public season API explicit about mechanism rotations.
        val cowlMethods = com.areslib.frc.marvin.MarvinCowlController::class.java.declaredMethods
        val hasSetCowlAngleRotations = cowlMethods.any { it.name == "setCowlAngleRotations" }
        assertTrue(hasSetCowlAngleRotations, "MarvinCowlController should have setCowlAngleRotations method")
    }
}
