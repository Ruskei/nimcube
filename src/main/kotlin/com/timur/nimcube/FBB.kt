package com.timur.nimcube

import org.bukkit.Color
import org.bukkit.Particle
import org.bukkit.World
import kotlin.math.ceil
import kotlin.math.max

enum class EdgeParticle {
    END_ROD,
    REDSTONE,
}

fun Nim.FBB.showEdges(world: World, interval: Float, particle: EdgeParticle = EdgeParticle.REDSTONE) {
    val step = interval.toDouble()
    if (step <= 0.0) return

    val minX = minX.toDouble()
    val minY = minY.toDouble()
    val minZ = minZ.toDouble()
    val maxX = maxX.toDouble()
    val maxY = maxY.toDouble()
    val maxZ = maxZ.toDouble()

    drawEdge(world, minX, minY, minZ, maxX, minY, minZ, step, particle)
    drawEdge(world, minX, maxY, minZ, maxX, maxY, minZ, step, particle)
    drawEdge(world, minX, minY, maxZ, maxX, minY, maxZ, step, particle)
    drawEdge(world, minX, maxY, maxZ, maxX, maxY, maxZ, step, particle)

    drawEdge(world, minX, minY, minZ, minX, maxY, minZ, step, particle)
    drawEdge(world, maxX, minY, minZ, maxX, maxY, minZ, step, particle)
    drawEdge(world, minX, minY, maxZ, minX, maxY, maxZ, step, particle)
    drawEdge(world, maxX, minY, maxZ, maxX, maxY, maxZ, step, particle)

    drawEdge(world, minX, minY, minZ, minX, minY, maxZ, step, particle)
    drawEdge(world, maxX, minY, minZ, maxX, minY, maxZ, step, particle)
    drawEdge(world, minX, maxY, minZ, minX, maxY, maxZ, step, particle)
    drawEdge(world, maxX, maxY, minZ, maxX, maxY, maxZ, step, particle)
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
    particle: EdgeParticle,
) {
    val dx = endX - startX
    val dy = endY - startY
    val dz = endZ - startZ
    val length = max(max(kotlin.math.abs(dx), kotlin.math.abs(dy)), kotlin.math.abs(dz))
    val steps = max(1, ceil(length / interval).toInt())
    val redstoneOptions = Particle.DustOptions(Color.RED, 0.5f)

    for (i in 0..steps) {
        val t = i.toDouble() / steps
        val x = startX + dx * t
        val y = startY + dy * t
        val z = startZ + dz * t
        when (particle) {
            EdgeParticle.END_ROD -> world.spawnParticle(
                Particle.END_ROD,
                x,
                y,
                z,
                1,
                0.0,
                0.0,
                0.0,
                0.0,
            )

            EdgeParticle.REDSTONE -> world.spawnParticle(
                Particle.DUST,
                x,
                y,
                z,
                1,
                0.0,
                0.0,
                0.0,
                0.0,
                redstoneOptions,
            )
        }
    }
}
