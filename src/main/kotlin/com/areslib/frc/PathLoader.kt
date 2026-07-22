package com.areslib.frc

import com.areslib.pathing.Path
import com.areslib.pathing.PathPlannerParser
import java.io.BufferedReader
import java.io.InputStreamReader
/**
 * Documentation for PathLoader
 */

object PathLoader {

    /**
     * Loads and parses a PathPlanner path JSON from classpath resources.
     */
    fun loadPath(pathName: String): Path {
        /**
         * Documentation for resourcePath
         */
        val resourcePath = "/deploy/pathplanner/paths/$pathName.path"
        /**
         * Documentation for inputStream
         */
        val inputStream = javaClass.getResourceAsStream(resourcePath)
            ?: throw IllegalArgumentException("Could not find path resource at $resourcePath")
        /**
         * Documentation for jsonString
         */
        
        val jsonString = BufferedReader(InputStreamReader(inputStream, Charsets.UTF_8)).use { reader ->
            reader.readText()
        }
        
        return PathPlannerParser.parsePath(jsonString)
    }
}
