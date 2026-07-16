package com.areslib.frc.marvin

import com.areslib.control.assist.ShotConfig

object MarvinConfig {
    val SHOT_CONFIG = ShotConfig(
        shooterOffsetX = -0.044704,
        shooterOffsetY = -0.055626,
        tofKeys = doubleArrayOf(1.24, 2.0, 3.0, 4.0, 5.6),
        tofValues = doubleArrayOf(0.128, 0.212, 0.345, 0.481, 0.795),
        shotKeys = doubleArrayOf(
            1.24, 2.0, 2.2, 2.5, 3.0, 3.2, 3.4, 3.63, 3.80, 4.0, 4.2, 4.4, 4.6, 4.8, 5.0, 5.2, 5.4, 5.6
        ),
        shotRpm = doubleArrayOf(
            3350.0, 3400.0, 3450.0, 3500.0, 3550.0, 3600.0, 3650.0, 3700.0, 3750.0, 3800.0, 3850.0, 3900.0, 3950.0, 4000.0, 4050.0, 4100.0, 4150.0, 4200.0
        ),
        shotCowl = doubleArrayOf(
            0.50, 0.70, 0.80, 0.95, 1.10, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40, 1.45, 1.50, 1.55, 1.60, 1.65, 1.70, 1.75
        ),
        delayCompensationSeconds = 0.05,
        shooterFacesRearward = true
    )
}
