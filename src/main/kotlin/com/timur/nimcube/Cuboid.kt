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
    val nim = nimWorld.nim
    val bukkitWorld = nimWorld.bukkitWorld
    val worldIndex = nimWorld.worldIndex
   
    private var display: BlockDisplay? = null

    fun init() {
        val pos = getPos()
        val rot = getRot()
        display?.remove()
        display = bukkitWorld.spawnEntity(
            Location(bukkitWorld, pos.x, pos.y, pos.z),
            EntityType.BLOCK_DISPLAY,
        ) as BlockDisplay
        display!!.block = Material.GLASS.createBlockData()
        display!!.transformation = createTransformation(rot)
        display!!.interpolationDuration = 2
        display!!.interpolationDelay = 0
        display!!.teleportDuration = 2
    }

    fun getPos(): Vector3d =
        Arena.ofConfined().use { arena ->
            nim.getCuboidPos(arena, worldIndex, handle)
        }

    fun getRot(): Quaternionf =
        Arena.ofConfined().use { arena ->
            nim.getCuboidRot(arena, worldIndex, handle)
        }

    fun update() {
        val display = display ?: run {
            init()
            return
        }
        val pos = getPos()
        val rot = getRot()
        display.interpolationDuration = 2
        display.interpolationDelay = 0
        display.teleportDuration = 2
        display.transformation = createTransformation(rot)
        display.teleport(Location(bukkitWorld, pos.x, pos.y, pos.z))
    }

    fun deinit() {
        display?.remove()
        display = null
    }

    private fun createTransformation(rot: Quaternionf): Transformation {
        val scale = Vector3f(1f, 1f, 1f)
        return Transformation(
            Vector3f(-0.5f).mul(scale).rotate(rot),
            Quaternionf(rot),
            scale,
            Quaternionf(),
        )
    }
}
