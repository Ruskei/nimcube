package com.timur.nimcube

import org.bukkit.Bukkit
import org.bukkit.Color
import org.bukkit.Particle
import org.bukkit.World
import org.bukkit.scheduler.BukkitTask
import org.joml.Vector3f
import java.lang.foreign.Arena
import java.util.concurrent.ConcurrentLinkedDeque
import kotlin.math.round
import kotlin.time.measureTime

class NimWorld(val plugin: Nimcube, val bukkitWorld: World, val dt: Float, val acceleration: Vector3f) {
    private val a2aContactPointDust = Particle.DustOptions(Color.PURPLE, 0.4f)
    private val a2aContactNormalDust = Particle.DustOptions(Color.AQUA, 0.4f)
    private val a2sContactPointDust = Particle.DustOptions(Color.BLACK, 0.4f)
    private val a2sContactNormalDust = Particle.DustOptions(Color.LIME, 0.4f)
    private val physicsSubstepsPerTick = round(0.05 / dt).toInt()

    val nim = plugin.nim
    val worldIndex = nim.createWorld(dt, acceleration)
    var physicsThread: BukkitTask? = null
    var bukkitThread: BukkitTask? = null
    var physicsFrozen = false

    val potentialBodyHandles = ConcurrentLinkedDeque<Nim.PotentialBodyHandle>()
    val cuboids = mutableListOf<Cuboid>()

    fun init() {
        physicsThread = Bukkit.getScheduler().runTaskTimer(plugin, Runnable { physicsTick() }, 1, 1)
        bukkitThread = Bukkit.getScheduler().runTaskTimer(plugin, Runnable { bukkitTick() }, 1, 1)
    }

    fun physicsTick() {
        if (!physicsFrozen) {
            advancePhysicsSubsteps(physicsSubstepsPerTick)
        }

//        renderPhysicsDebug()
    }

    fun togglePhysicsFrozen(): Boolean {
        physicsFrozen = !physicsFrozen
        return physicsFrozen
    }

    fun stepPhysicsSubsteps(steps: Int) {
        require(steps > 0) { "steps must be positive" }
        advancePhysicsSubsteps(steps)
    }

    private fun advancePhysicsSubsteps(steps: Int) {
        val physicsFrameDuration = measureTime {
            repeat(steps) {
                nim.tickWorld(worldIndex)
            }
        }

        println("physicsFrameDuration=$physicsFrameDuration")
    }

    private fun renderPhysicsDebug() {
        Arena.ofConfined().use { arena ->
            val nodeCount = nim.numAabbTreeNodes(worldIndex)
            for (i in 0 until nodeCount) {
                nim.getAabbTreeNode(arena, worldIndex, i).showEdges(bukkitWorld, 0.99f)
            }

            var collisionIndex = 0
            while (true) {
                val collision = nim.getA2aCollisionResult(arena, worldIndex, collisionIndex)
                if (collision.isSentinel()) break
                showCollisionContacts(
                    collision.contactCount,
                    collision.contactPoints,
                    a2aContactPointDust,
                    a2aContactNormalDust
                )
                collisionIndex++
            }

            collisionIndex = 0
            while (true) {
                val collision = nim.getA2sCollisionResult(arena, worldIndex, collisionIndex)
                if (collision.isSentinel()) break
                showCollisionContacts(
                    collision.contactCount,
                    collision.contactPoints,
                    a2sContactPointDust,
                    a2sContactNormalDust
                )
                collisionIndex++
            }
        }
    }

    private fun showCollisionContacts(
        contactCount: Int,
        contactPoints: List<Nim.CollisionContactPoint>,
        pointDustOptions: Particle.DustOptions,
        normalDustOptions: Particle.DustOptions,
    ) {
        val clampedContactCount = minOf(contactCount, contactPoints.size)
        for (contactIndex in 0 until clampedContactCount) {
            val contact = contactPoints[contactIndex]
            val point = contact.position
            bukkitWorld.spawnParticle(
                Particle.DUST,
                point.x.toDouble(),
                point.y.toDouble(),
                point.z.toDouble(),
                1,
                0.0,
                0.0,
                0.0,
                0.0,
                pointDustOptions,
            )

            val normalEnd = Vector3f(contact.normal).mul(contact.penetrationDepth).add(point)
            drawParticleLine(
                world = bukkitWorld,
                startX = point.x.toDouble(),
                startY = point.y.toDouble(),
                startZ = point.z.toDouble(),
                endX = normalEnd.x.toDouble(),
                endY = normalEnd.y.toDouble(),
                endZ = normalEnd.z.toDouble(),
                interval = 0.15f,
                particle = EdgeParticle.REDSTONE,
                dustOptions = normalDustOptions,
            )
        }
    }

    fun bukkitTick() {
        var handle: Nim.PotentialBodyHandle? = null
        val toRetry = mutableListOf<Nim.PotentialBodyHandle>()
        while (potentialBodyHandles.poll().also { handle = it } != null) {
            val h = handle
            if (h == null) break
            val bodyHandle = h.tryGet()
            if (bodyHandle == null) toRetry += h
            else cuboids += Cuboid(this, bodyHandle).also { it.init() }
        }
        potentialBodyHandles += toRetry

        var i = cuboids.size - 1
        while (i >= 0) {
            val cuboid = cuboids[i]
            if (!nim.isCuboidValid(worldIndex, cuboid.handle)) {
                cuboid.deinit()
                cuboids.removeAt(i)
            }
            i--
        }

        Arena.ofConfined().use { arena ->
            cuboids.forEach { it.update(arena) }
        }
    }

    fun deinit() {
        cuboids.forEach { it.deinit() }
        physicsThread?.cancel()
        bukkitThread?.cancel()
    }
}
