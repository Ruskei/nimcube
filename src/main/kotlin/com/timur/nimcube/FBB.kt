package com.timur.nimcube

import org.bukkit.Particle
import org.bukkit.World
import kotlin.math.ceil
import kotlin.math.max

fun Nim.FBB.showEdges(world: World, interval: Float) {
    val step = interval.toDouble()
    if (step <= 0.0) return

    val minX = minX.toDouble()
    val minY = minY.toDouble()
    val minZ = minZ.toDouble()
    val maxX = maxX.toDouble()
    val maxY = maxY.toDouble()
    val maxZ = maxZ.toDouble()

    drawEdge(world, minX, minY, minZ, maxX, minY, minZ, step)
    drawEdge(world, minX, maxY, minZ, maxX, maxY, minZ, step)
    drawEdge(world, minX, minY, maxZ, maxX, minY, maxZ, step)
    drawEdge(world, minX, maxY, maxZ, maxX, maxY, maxZ, step)

    drawEdge(world, minX, minY, minZ, minX, maxY, minZ, step)
    drawEdge(world, maxX, minY, minZ, maxX, maxY, minZ, step)
    drawEdge(world, minX, minY, maxZ, minX, maxY, maxZ, step)
    drawEdge(world, maxX, minY, maxZ, maxX, maxY, maxZ, step)

    drawEdge(world, minX, minY, minZ, minX, minY, maxZ, step)
    drawEdge(world, maxX, minY, minZ, maxX, minY, maxZ, step)
    drawEdge(world, minX, maxY, minZ, minX, maxY, maxZ, step)
    drawEdge(world, maxX, maxY, minZ, maxX, maxY, maxZ, step)
}

private fun drawEdge(
    world: World,
    startX: Double,
    startY: Double,
    startZ: Double,
    endX: Double,
    endY: Double,
    endZ: Double,
    interval: Double,
) {
    val dx = endX - startX
    val dy = endY - startY
    val dz = endZ - startZ
    val length = max(max(kotlin.math.abs(dx), kotlin.math.abs(dy)), kotlin.math.abs(dz))
    val steps = max(1, ceil(length / interval).toInt())

    for (i in 0..steps) {
        val t = i.toDouble() / steps
        world.spawnParticle(
            Particle.END_ROD,
            startX + dx * t,
            startY + dy * t,
            startZ + dz * t,
            1,
            0.0,
            0.0,
            0.0,
            0.0,
        )
    }
}
