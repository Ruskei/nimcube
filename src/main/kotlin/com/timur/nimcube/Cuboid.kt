package com.timur.nimcube

import org.bukkit.Color
import org.bukkit.Particle
import org.bukkit.Location
import org.bukkit.Material
import org.bukkit.entity.BlockDisplay
import org.bukkit.entity.EntityType
import org.bukkit.util.Transformation
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import java.lang.foreign.Arena
import java.lang.foreign.MemorySegment
import kotlin.random.Random

class Cuboid(
    val nimWorld: NimWorld,
    val handle: Nim.BodyHandle,
) {
    val nim = nimWorld.nim
    val bukkitWorld = nimWorld.bukkitWorld
    val worldIndex = nimWorld.worldIndex

    private var display: BlockDisplay? = null
    private val meshColors = ArrayList<Color>()
    private val meshParticleInterval = 0.2f
    private val meshParticleSize = 0.3f

    fun init() {
        Arena.ofConfined().use { arena ->
            init(arena)
        }
    }

    fun init(arena: Arena) {
        val pos = getPos(arena)
        val rot = getRot(arena)
        val dimensions = getDimensions(arena)
        display?.remove()
        display = bukkitWorld.spawnEntity(
            Location(bukkitWorld, pos.x, pos.y, pos.z),
            EntityType.BLOCK_DISPLAY,
        ) as BlockDisplay
        display!!.block = Material.GLASS.createBlockData()
        display!!.transformation = createTransformation(rot, dimensions)
        display!!.interpolationDuration = 2
        display!!.interpolationDelay = 0
        display!!.teleportDuration = 2
    }

    fun getPos(arena: Arena): Vector3d =
        nim.getCuboidPos(arena, worldIndex, handle)

    fun getRot(arena: Arena): Quaternionf =
        nim.getCuboidRot(arena, worldIndex, handle)

    fun getDimensions(arena: Arena): Vector3f =
        nim.getCuboidDimensions(arena, worldIndex, handle)

    fun update(arena: Arena, meshBuffer: MemorySegment) {
        val display = display ?: run {
            init(arena)
            return
        }

        val pos = getPos(arena)
        val rot = getRot(arena)
        val dimensions = getDimensions(arena)
        display.interpolationDuration = 2
        display.interpolationDelay = 0
        display.teleportDuration = 2
        display.transformation = createTransformation(rot, dimensions)
        display.teleport(Location(bukkitWorld, pos.x, pos.y, pos.z))

        val meshes = nim.getCuboidMeshes(worldIndex, handle, meshBuffer) ?: return
        ensureMeshColors(meshes.size)
        for ((meshIndex, mesh) in meshes.withIndex()) {
            val dustOptions = Particle.DustOptions(meshColors[meshIndex], meshParticleSize)
            for (face in mesh.faces) {
                if (face.isEmpty()) continue

                for (vertexIdx in face.indices) {
                    val startIndex = face[vertexIdx]
                    val endIndex = face[(vertexIdx + 1) % face.size]
                    if (startIndex !in mesh.vertices.indices || endIndex !in mesh.vertices.indices) {
                        continue
                    }

                    val start = mesh.vertices[startIndex]
                    val end = mesh.vertices[endIndex]
                    drawParticleLine(
                        world = bukkitWorld,
                        startX = start.x.toDouble(),
                        startY = start.y.toDouble(),
                        startZ = start.z.toDouble(),
                        endX = end.x.toDouble(),
                        endY = end.y.toDouble(),
                        endZ = end.z.toDouble(),
                        interval = meshParticleInterval,
                        particle = EdgeParticle.REDSTONE,
                        dustOptions = dustOptions,
                    )
                }
            }
        }
    }

    fun deinit() {
        display?.remove()
        display = null
    }

    private fun createTransformation(rot: Quaternionf, dimensions: Vector3f): Transformation {
        val scale = Vector3f(dimensions.x, dimensions.y, dimensions.z)
        val displayRot = Quaternionf(rot)
        return Transformation(
            Vector3f(-0.5f).mul(scale).rotate(displayRot),
            displayRot,
            scale,
            Quaternionf(),
        )
    }

    private fun ensureMeshColors(meshCount: Int) {
        while (meshColors.size < meshCount) {
            meshColors += randomColor()
        }
    }

    private fun randomColor(): Color =
        Color.fromRGB(
            Random.nextInt(256),
            Random.nextInt(256),
            Random.nextInt(256),
        )
}
