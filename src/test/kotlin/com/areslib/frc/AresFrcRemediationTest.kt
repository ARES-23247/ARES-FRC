package com.areslib.frc

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.*
/**
 * Documentation for AresFrcRemediationTest
 */

class AresFrcRemediationTest {
    @Test
    fun testShooterRearwardHeading() {
        val currentHeading = Math.PI / 2.0 // 90 degrees
        val targetX = 1.0
        val targetY = 1.0
        val currentX = 0.0
        val currentY = 0.0
        // Vector is (1, 1), angle is PI/4.
        // Since rearward, add PI -> 5*PI/4.
        // Wrap -> -3*PI/4 (-2.356)
        // current is PI/2 (1.57)
        // Error: -3*PI/4 - 2*PI/4 = -5*PI/4 -> wrap -> 3*PI/4
        
        var targetHeadingRad = Math.atan2(targetY - currentY, targetX - currentX)
        if (com.areslib.frc.marvin.MarvinConfig.SHOT_CONFIG.shooterFacesRearward) {
            targetHeadingRad = com.areslib.math.wrapAngle(targetHeadingRad + Math.PI)
        }
        assertEquals(-2.35619, targetHeadingRad, 0.001, "Target heading should be inverted and wrapped")
    }

    @Test
    fun testCowlAngleUsesRotations() {
        // Just verify that the method is now setCowlAngleRotations
        val cowlMethods = com.areslib.frc.marvin.MarvinCowlController::class.java.declaredMethods
        val hasSetCowlAngleRotations = cowlMethods.any { it.name == "setCowlAngleRotations" }
        assertTrue(hasSetCowlAngleRotations, "MarvinCowlController should have setCowlAngleRotations method")
    }
}
