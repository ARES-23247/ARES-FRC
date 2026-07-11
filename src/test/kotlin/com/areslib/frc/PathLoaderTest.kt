package com.areslib.frc

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

class PathLoaderTest {

    @Test
    fun testLoadSimPath() {
        val path = PathLoader.loadPath("SimPath")
        assertNotNull(path)
        assertTrue(path.points.isNotEmpty(), "Loaded path should have points")
    }

    @Test
    fun testLoadNonExistentPathThrows() {
        assertThrows(IllegalArgumentException::class.java) {
            PathLoader.loadPath("NonExistentPath")
        }
    }
}
