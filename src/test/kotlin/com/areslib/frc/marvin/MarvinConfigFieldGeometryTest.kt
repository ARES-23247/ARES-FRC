package com.areslib.frc.marvin

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class MarvinConfigFieldGeometryTest {
    @Test
    fun `speaker targets match official Crescendo field dimensions`() {
        // Independent literals from the official FIRST 2024 field drawings.
        val officialFieldLengthMeters = 651.25 * 0.0254
        val officialSpeakerCenterYMeters = 218.42 * 0.0254

        assertEquals(0.0, MarvinConfig.FieldTargets.blueSpeaker.x, 1e-9)
        assertEquals(officialSpeakerCenterYMeters, MarvinConfig.FieldTargets.blueSpeaker.y, 1e-9)
        assertEquals(officialFieldLengthMeters, MarvinConfig.FieldTargets.redSpeaker.x, 1e-9)
        assertEquals(officialSpeakerCenterYMeters, MarvinConfig.FieldTargets.redSpeaker.y, 1e-9)
    }
}
