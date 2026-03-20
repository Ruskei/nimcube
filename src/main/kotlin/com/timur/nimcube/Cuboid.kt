package com.timur.nimcube

import org.bukkit.Color
import org.bukkit.Location
import org.bukkit.Material
import org.bukkit.entity.BlockDisplay
import org.bukkit.entity.EntityType
import org.bukkit.util.Transformation
import org.joml.Matrix4f
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import java.lang.foreign.Arena
import java.lang.foreign.MemorySegment
import kotlin.random.Random

data class Triangle(
    val a: Vector3f,
    val b: Vector3f,
    val c: Vector3f,
)

fun triangulateMesh(mesh: Nim.Mesh): ArrayList<Triangle> {
    val triangles = ArrayList<Triangle>()

    for (face in mesh.faces) {
        if (face.size < 3) continue

        val baseIndex = face[0]
        if (baseIndex !in mesh.vertices.indices) continue

        val baseVertex = mesh.vertices[baseIndex]
        for (i in 1 until face.size - 1) {
            val secondIndex = face[i]
            val thirdIndex = face[i + 1]
            if (secondIndex !in mesh.vertices.indices || thirdIndex !in mesh.vertices.indices) {
                continue
            }

            triangles += Triangle(
                a = Vector3f(baseVertex),
                b = Vector3f(mesh.vertices[secondIndex]),
                c = Vector3f(mesh.vertices[thirdIndex]),
            )
        }
    }

    return triangles
}

class Cuboid(
    val nimWorld: NimWorld,
    val handle: Nim.BodyHandle,
) {
    val nim = nimWorld.nim
    val bukkitWorld = nimWorld.bukkitWorld
    val worldIndex = nimWorld.worldIndex

    private var display: BlockDisplay? = null
    private val triangleDisplays = mutableListOf<BlockDisplay>()
    private val meshColors = ArrayList<Color>()
    private val meshParticleInterval = 0.2f
    private val meshParticleSize = 0.4f

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
        display!!.block = Material.WHITE_CONCRETE.createBlockData()
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

        if (meshes.isEmpty()) display.block = Material.WHITE_CONCRETE.createBlockData()
        else display.block = Material.AIR.createBlockData()

        // this is inefficient as hell but who cares
        var neededTriangles = 0
        for (mesh in meshes) neededTriangles += triangulateMesh(mesh).size
        ensureTriangleDisplays(neededTriangles * 3)
        var triangleIndex = 0
        for (disp in triangleDisplays) disp.block = Material.AIR.createBlockData()
        for (mesh in meshes)
            for (triangle in triangulateMesh(mesh)) {
                val a = triangle.a
                val b = triangle.b
                val c = triangle.c

                val ab = b.sub(a, Vector3f())
                val ac = c.sub(a, Vector3f())
                val bc = c.sub(b, Vector3f())

                val halfAB = ab.mul(0.5f, Vector3f())
                val halfAC = ac.mul(0.5f, Vector3f())
                val halfBC = bc.mul(0.5f, Vector3f())

                val halfAB2halfAC = halfAC.sub(halfAB, Vector3f())
                val halfAB2halfBC = b.add(halfBC, Vector3f()).sub(a).sub(halfAB)

                val norm = ab.cross(ac, Vector3f()).normalize()
                val normThin = norm.mul(0.01f, Vector3f())

                val l1 = triangle.a.d
                val m1 = Matrix4f(
                    normThin.x, normThin.y, normThin.z, 0f,
                    halfAB.x, halfAB.y, halfAB.z, 0f,
                    halfAC.x, halfAC.y, halfAC.z, 0f,
                    0f, 0f, 0f, 1f,
                )

                val l2 = triangle.a.add(halfAB, Vector3f()).d
                val m2 = Matrix4f(
                    normThin.x, normThin.y, normThin.z, 0f,
                    halfAB.x, halfAB.y, halfAB.z, 0f,
                    halfAB2halfAC.x, halfAB2halfAC.y, halfAB2halfAC.z, 0f,
                    0f, 0f, 0f, 1f,
                )

                val l3 = Vector3d(l2)
                val m3 = Matrix4f(
                    normThin.x, normThin.y, normThin.z, 0f,
                    halfAB2halfBC.x, halfAB2halfBC.y, halfAB2halfBC.z, 0f,
                    halfAB2halfAC.x, halfAB2halfAC.y, halfAB2halfAC.z, 0f,
                    0f, 0f, 0f, 1f,
                )

                val disp1 = triangleDisplays[triangleIndex++]
                disp1.block = Material.WHITE_CONCRETE.createBlockData()
                disp1.setTransformationMatrix(m1)
                disp1.teleport(
                    Location(
                        bukkitWorld,
                        l1.x, l1.y, l1.z,
                    )
                )

                val disp2 = triangleDisplays[triangleIndex++]
                disp2.block = Material.WHITE_CONCRETE.createBlockData()
                disp2.setTransformationMatrix(m2)
                disp2.teleport(
                    Location(
                        bukkitWorld,
                        l2.x, l2.y, l2.z,
                    )
                )

                val disp3 = triangleDisplays[triangleIndex++]
                disp3.block = Material.WHITE_CONCRETE.createBlockData()
                disp3.setTransformationMatrix(m3)
                disp3.teleport(
                    Location(
                        bukkitWorld,
                        l3.x, l3.y, l3.z,
                    )
                )
            }
    }

    fun deinit() {
        display?.remove()
        display = null
        for (disp in triangleDisplays) disp.remove()
        triangleDisplays.clear()
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

    private fun ensureTriangleDisplays(needed: Int) {
        while (triangleDisplays.size < needed) {
            val disp = bukkitWorld.spawnEntity(
                display!!.location,
                EntityType.BLOCK_DISPLAY,
            ) as BlockDisplay
            disp.block = Material.AIR.createBlockData()
            disp.transformation = createTransformation(Quaternionf(), Vector3f())
            disp.interpolationDuration = 1
            disp.interpolationDelay = 0
            disp.teleportDuration = 1

            triangleDisplays += disp
        }
    }

    private fun randomColor(): Color =
        Color.fromRGB(
            Random.nextInt(256),
            Random.nextInt(256),
            Random.nextInt(256),
        )
}

val Vector3f.d: Vector3d
    get() = Vector3d(x.toDouble(), y.toDouble(), z.toDouble())
