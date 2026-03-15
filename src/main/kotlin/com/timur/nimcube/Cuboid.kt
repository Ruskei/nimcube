package com.timur.nimcube

import org.bukkit.Location
import org.bukkit.Material
import org.bukkit.entity.BlockDisplay
import org.bukkit.entity.EntityType
import org.bukkit.util.Transformation
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import java.lang.foreign.Arena

class Cuboid(
    val nimWorld: NimWorld,
    val handle: Nim.BodyHandle,
) {
    var valid = true
    val nim = nimWorld.nim
    val bukkitWorld = nimWorld.bukkitWorld
    val worldIndex = nimWorld.worldIndex

    private var display: BlockDisplay? = null

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

    fun update(arena: Arena) {
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
    }

    fun deinit() {
        valid = false
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
}
