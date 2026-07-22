package com.areslib.frc

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
/**
 * Documentation for PathLoaderTest
 */

class PathLoaderTest {
    /**
     * Documentation for testLoadSimPath
     */

    @Test
    fun testLoadSimPath() {
        /**
         * Documentation for path
         */
        val path = PathLoader.loadPath("SimPath")
        assertNotNull(path)
        assertTrue(path.points.isNotEmpty(), "Loaded path should have points")
    }
    /**
     * Documentation for testLoadNonExistentPathThrows
     */

    @Test
    fun testLoadNonExistentPathThrows() {
        assertThrows(IllegalArgumentException::class.java) {
            PathLoader.loadPath("NonExistentPath")
        }
    }
}
