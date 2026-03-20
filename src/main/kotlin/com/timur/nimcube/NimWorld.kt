package com.timur.nimcube

import org.bukkit.Bukkit
import org.bukkit.Color
import org.bukkit.Particle
import org.bukkit.World
import org.bukkit.scheduler.BukkitTask
import org.joml.Vector3f
import java.lang.foreign.Arena
import java.lang.foreign.MemorySegment
import java.util.concurrent.ConcurrentLinkedDeque
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.round
import kotlin.time.measureTime

class NimWorld(val plugin: Nimcube, val bukkitWorld: World, val dt: Float, val acceleration: Vector3f) {
    private val a2aContactPointDust = Particle.DustOptions(Color.PURPLE, 0.4f)
    private val a2aContactNormalDust = Particle.DustOptions(Color.AQUA, 0.4f)
    private val a2sContactPointDust = Particle.DustOptions(Color.BLACK, 0.4f)
    private val a2sContactNormalDust = Particle.DustOptions(Color.LIME, 0.4f)
    private val portalBorderContactPointDust = Particle.DustOptions(Color.YELLOW, 0.4f)
    private val portalBorderContactNormalDust = Particle.DustOptions(Color.ORANGE, 0.4f)
    private val physicsSubstepsPerTick = round(0.05 / dt).toInt()

    val nim = plugin.nim
    val worldIndex = nim.createWorld(dt, acceleration)
    val physicsBlocked = AtomicBoolean(false)
    var physicsThread: BukkitTask? = null
    var bukkitThread: BukkitTask? = null
    var physicsFrozen = false
    private val meshArena = Arena.ofShared()
    val cuboidMeshBuffer: MemorySegment = meshArena.allocate(1024 * 1024L)

    val potentialBodyHandles = ConcurrentLinkedDeque<Nim.PotentialBodyHandle>()
    val potentialPortalHandles = ConcurrentLinkedDeque<Nim.PotentialPortalHandle>()
    val cuboids = mutableListOf<Cuboid>()
    val portals = mutableListOf<Portal>()

    fun init() {
        physicsThread = Bukkit.getScheduler().runTaskTimer(plugin, Runnable { physicsTick() }, 1, 1)
        bukkitThread = Bukkit.getScheduler().runTaskTimer(plugin, Runnable { bukkitTick() }, 1, 1)
    }

    fun physicsTick() {
        if (!physicsBlocked.compareAndSet(false, true)) return
        if (!physicsFrozen) {
            advancePhysicsSubsteps(physicsSubstepsPerTick)
        }
        physicsBlocked.set(false)
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

//        println("physicsFrameDuration=$physicsFrameDuration")
    }

    private fun renderPhysicsDebug() {
        Arena.ofConfined().use { arena ->
//            val nodeCount = nim.numAabbTreeNodes(worldIndex)
//            for (i in 0 until nodeCount) {
//                nim.getAabbTreeNode(arena, worldIndex, i).showEdges(bukkitWorld, 0.99f)
//            }

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

            collisionIndex = 0
            while (true) {
                val collision = nim.getPortalBorderCollisionResult(arena, worldIndex, collisionIndex)
                if (collision.isSentinel()) break
                showCollisionContacts(
                    collision.contactCount,
                    collision.contactPoints,
                    portalBorderContactPointDust,
                    portalBorderContactNormalDust,
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
        Arena.ofConfined().use { arena ->
            var bodyHandle: Nim.PotentialBodyHandle? = null
            val bodiesToRetry = mutableListOf<Nim.PotentialBodyHandle>()
            while (potentialBodyHandles.poll().also { bodyHandle = it } != null) {
                val h = bodyHandle
                if (h == null) break
                val resolved = h.tryGet()
                if (resolved == null) bodiesToRetry += h
                else cuboids += Cuboid(this, resolved).also { it.init(arena) }
            }
            potentialBodyHandles += bodiesToRetry

            var portalHandle: Nim.PotentialPortalHandle? = null
            val portalsToRetry = mutableListOf<Nim.PotentialPortalHandle>()
            while (potentialPortalHandles.poll().also { portalHandle = it } != null) {
                val h = portalHandle
                if (h == null) break
                val resolved = h.tryGet()
                if (resolved == null) portalsToRetry += h
                else portals += Portal(this, resolved).also { it.init(arena) }
            }
            potentialPortalHandles += portalsToRetry

            var i = cuboids.size - 1
            while (i >= 0) {
                val cuboid = cuboids[i]
                if (!nim.isCuboidValid(worldIndex, cuboid.handle)) {
                    cuboid.deinit()
                    cuboids.removeAt(i)
                }
                i--
            }

            i = portals.size - 1
            while (i >= 0) {
                val portal = portals[i]
                if (nim.getPortal(arena, worldIndex, portal.handle).isSentinel()) {
                    portal.deinit()
                    portals.removeAt(i)
                }
                i--
            }

            cuboids.forEach { it.update(arena, cuboidMeshBuffer) }
            portals.forEach { it.update(arena) }
        }

//        renderPhysicsDebug()
    }

    fun deinit() {
        cuboids.forEach { it.deinit() }
        portals.forEach { it.deinit() }
        physicsThread?.cancel()
        bukkitThread?.cancel()
        meshArena.close()
    }
}
