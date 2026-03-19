package com.timur.nimcube

import org.bukkit.Location
import org.bukkit.Material
import org.bukkit.entity.BlockDisplay
import org.bukkit.entity.EntityType
import org.bukkit.util.Transformation
import org.joml.Quaternionf
import org.joml.Vector3f
import java.lang.foreign.Arena

class Portal(
    val nimWorld: NimWorld,
    val handle: Nim.PortalHandle,
) {
    companion object {
        private const val THICKNESS = 0.05f
    }

    val nim = nimWorld.nim
    val bukkitWorld = nimWorld.bukkitWorld
    val worldIndex = nimWorld.worldIndex

    private var displayA: BlockDisplay? = null
    private var displayB: BlockDisplay? = null

    fun init() {
        Arena.ofConfined().use { arena ->
            init(arena)
        }
    }

    fun init(arena: Arena) {
        val portal = nim.getPortal(arena, worldIndex, handle)
        displayA?.remove()
        displayB?.remove()
        displayA = spawnDisplay(portal.originA, Material.NETHER_PORTAL)
        displayB = spawnDisplay(portal.originB, Material.NETHER_PORTAL)
        applyPortalData(portal)
    }

    fun update(arena: Arena) {
        val portal = nim.getPortal(arena, worldIndex, handle)
        val displayA = displayA ?: run {
            init(arena)
            return
        }
        val displayB = displayB ?: run {
            init(arena)
            return
        }

        val transformA = createTransformation(portal.quatA, portal.scaleX, portal.scaleY)
        val transformB = createTransformation(portal.quatB, portal.scaleX, portal.scaleY)

        displayA.interpolationDuration = 2
        displayA.interpolationDelay = 0
        displayA.teleportDuration = 2
        displayA.transformation = transformA
        displayA.teleport(Location(bukkitWorld, portal.originA.x.toDouble(), portal.originA.y.toDouble(), portal.originA.z.toDouble()))

        displayB.interpolationDuration = 2
        displayB.interpolationDelay = 0
        displayB.teleportDuration = 2
        displayB.transformation = transformB
        displayB.teleport(Location(bukkitWorld, portal.originB.x.toDouble(), portal.originB.y.toDouble(), portal.originB.z.toDouble()))
    }

    fun deinit() {
        displayA?.remove()
        displayB?.remove()
        displayA = null
        displayB = null
    }

    private fun spawnDisplay(origin: Vector3f, material: Material): BlockDisplay =
        (bukkitWorld.spawnEntity(
            Location(bukkitWorld, origin.x.toDouble(), origin.y.toDouble(), origin.z.toDouble()),
            EntityType.BLOCK_DISPLAY,
        ) as BlockDisplay).also {
            it.block = material.createBlockData()
        }

    private fun applyPortalData(portal: Nim.PortalData) {
        displayA?.transformation = createTransformation(portal.quatA, portal.scaleX, portal.scaleY)
        displayA?.interpolationDuration = 2
        displayA?.interpolationDelay = 0
        displayA?.teleportDuration = 2

        displayB?.transformation = createTransformation(portal.quatB, portal.scaleX, portal.scaleY)
        displayB?.interpolationDuration = 2
        displayB?.interpolationDelay = 0
        displayB?.teleportDuration = 2
    }

    private fun createTransformation(rot: Quaternionf, scaleX: Float, scaleY: Float): Transformation {
        val scale = Vector3f(scaleX, scaleY, THICKNESS)
        val displayRot = Quaternionf(rot)
        return Transformation(
            Vector3f(-0.5f).mul(scale).rotate(displayRot),
            displayRot,
            scale,
            Quaternionf(),
        )
    }
}
