package com.timur.nimcube

import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import java.lang.foreign.*
import java.lang.invoke.MethodHandle

class Nim(plugin: Nimcube) {
    private val libArena: Arena = Arena.ofShared()
    val deinit: MethodHandle

    val createWorldHandle: MethodHandle

    @JvmInline
    value class WorldIndex(val index: Int)

    val tickWorldHandle: MethodHandle

    val createCuboidHandle: MethodHandle
    val removeCuboidHandle: MethodHandle

    data class BodyHandle(val slot: Int, val generation: Int) {
        val asLong = (slot.toLong()) or (generation.toLong() shl 32)
    }

    class PotentialBodyHandle(
        private val nim: Nim,
        private val worldIndex: WorldIndex,
        private val arena: Arena,
        private val segment: MemorySegment,
    ) {
        private var isValid = false
        private var handle: BodyHandle? = null

        fun tryGet(): BodyHandle? {
            if (isValid) return handle
            val slot = segment.getAtIndex(ValueLayout.JAVA_INT, 0)
            val generation = segment.getAtIndex(ValueLayout.JAVA_INT, 1)
            if (slot == -1) return null
            val h = BodyHandle(slot, generation)
            if (!nim.isCuboidValid(worldIndex, h)) return null

            isValid = true
            arena.close()
            handle = h
            return handle
        }
    }

    val getGlobalCuboidPosHandle: MethodHandle
    val isCuboidValidHandle: MethodHandle
    val numBodiesHandle: MethodHandle
    val getBodyHandle: MethodHandle
    val getGlobalCuboidRotHandle: MethodHandle
    val getGlobalCuboidDimensionsHandle: MethodHandle
//    val getCuboidInverseMassHandle: MethodHandle

    val C_D3 = MemoryLayout.structLayout(
        ValueLayout.JAVA_DOUBLE.withName("x"),
        ValueLayout.JAVA_DOUBLE.withName("y"),
        ValueLayout.JAVA_DOUBLE.withName("z"),
    )
    val C_F3 = MemoryLayout.structLayout(
        ValueLayout.JAVA_FLOAT.withName("x"),
        ValueLayout.JAVA_FLOAT.withName("y"),
        ValueLayout.JAVA_FLOAT.withName("z"),
    )
    val C_QF = MemoryLayout.structLayout(
        ValueLayout.JAVA_FLOAT.withName("x"),
        ValueLayout.JAVA_FLOAT.withName("y"),
        ValueLayout.JAVA_FLOAT.withName("z"),
        ValueLayout.JAVA_FLOAT.withName("w"),
    )
    val C_BodyExternalData = MemoryLayout.structLayout(
        ValueLayout.JAVA_DOUBLE.withName("posX"),
        ValueLayout.JAVA_DOUBLE.withName("posY"),
        ValueLayout.JAVA_DOUBLE.withName("posZ"),
        ValueLayout.JAVA_FLOAT.withName("rotX"),
        ValueLayout.JAVA_FLOAT.withName("rotY"),
        ValueLayout.JAVA_FLOAT.withName("rotZ"),
        ValueLayout.JAVA_FLOAT.withName("rotW"),
        ValueLayout.JAVA_FLOAT.withName("dimensionsX"),
        ValueLayout.JAVA_FLOAT.withName("dimensionsY"),
        ValueLayout.JAVA_FLOAT.withName("dimensionsZ"),
        ValueLayout.JAVA_INT.withName("handleSlot"),
        ValueLayout.JAVA_INT.withName("handleGeneration"),
        MemoryLayout.paddingLayout(4),
    )

    val greedyMeshHandle: MethodHandle
    val numBBsHandle: MethodHandle
    val getBBHandle: MethodHandle

    data class BodyExternalData(
        val handle: BodyHandle,
        val pos: Vector3d,
        val rot: Quaternionf,
        val dimensions: Vector3f,
    ) {
        fun isSentinel(): Boolean =
            handle.slot == -1 &&
            handle.generation == 0 &&
            pos.x == 0.0 &&
            pos.y == 0.0 &&
            pos.z == 0.0 &&
            rot.x == 0f &&
            rot.y == 0f &&
            rot.z == 0f &&
            rot.w == 1f &&
            dimensions.x == -1f &&
            dimensions.y == -1f &&
            dimensions.z == -1f
    }

    data class FBB(
        val minX: Float,
        val minY: Float,
        val minZ: Float,
        val maxX: Float,
        val maxY: Float,
        val maxZ: Float,
    )

    val C_FBB = MemoryLayout.structLayout(
        ValueLayout.JAVA_FLOAT.withName("minX"),
        ValueLayout.JAVA_FLOAT.withName("minY"),
        ValueLayout.JAVA_FLOAT.withName("minZ"),
        ValueLayout.JAVA_FLOAT.withName("maxX"),
        ValueLayout.JAVA_FLOAT.withName("maxY"),
        ValueLayout.JAVA_FLOAT.withName("maxZ"),
    )

    init {
        val libPath = plugin.dataFolder.toPath().resolve("libmain.so").toAbsolutePath()
        val lookup = SymbolLookup.libraryLookup(libPath, libArena)
        val linker = Linker.nativeLinker()

        val libraryInit = linker.downcallHandle(
            lookup.findOrThrow("library_init"),
            FunctionDescriptor.ofVoid(),
        )
        deinit = linker.downcallHandle(
            lookup.findOrThrow("library_deinit"),
            FunctionDescriptor.ofVoid(),
        )

        createWorldHandle = linker.downcallHandle(
            lookup.findOrThrow("c_create_world"),
            FunctionDescriptor.of(
                ValueLayout.JAVA_INT,
                ValueLayout.JAVA_FLOAT,
                ValueLayout.JAVA_FLOAT, ValueLayout.JAVA_FLOAT, ValueLayout.JAVA_FLOAT,
            )
        )
        tickWorldHandle = linker.downcallHandle(
            lookup.findOrThrow("c_tick_world"),
            FunctionDescriptor.ofVoid(ValueLayout.JAVA_INT)
        )

        createCuboidHandle = linker.downcallHandle(
            lookup.findOrThrow("c_create_cuboid"),
            FunctionDescriptor.of(
                ValueLayout.JAVA_BOOLEAN,
                ValueLayout.ADDRESS,
                ValueLayout.JAVA_INT,
                ValueLayout.JAVA_DOUBLE, ValueLayout.JAVA_DOUBLE, ValueLayout.JAVA_DOUBLE,
                ValueLayout.JAVA_FLOAT, ValueLayout.JAVA_FLOAT, ValueLayout.JAVA_FLOAT,
                ValueLayout.JAVA_FLOAT, ValueLayout.JAVA_FLOAT, ValueLayout.JAVA_FLOAT,
                ValueLayout.JAVA_FLOAT, ValueLayout.JAVA_FLOAT, ValueLayout.JAVA_FLOAT, ValueLayout.JAVA_FLOAT,
                ValueLayout.JAVA_FLOAT, ValueLayout.JAVA_FLOAT, ValueLayout.JAVA_FLOAT,
                ValueLayout.JAVA_FLOAT,
            )
        )
        removeCuboidHandle = linker.downcallHandle(
            lookup.findOrThrow("c_remove_cuboid"),
            FunctionDescriptor.of(
                ValueLayout.JAVA_BOOLEAN,
                ValueLayout.JAVA_INT, ValueLayout.JAVA_LONG,
            )
        )
        getGlobalCuboidPosHandle = linker.downcallHandle(
            lookup.findOrThrow("c_get_global_cuboid_pos"),
            FunctionDescriptor.of(
                C_D3,
                ValueLayout.JAVA_INT, ValueLayout.JAVA_LONG,
            )
        )
        getGlobalCuboidRotHandle = linker.downcallHandle(
            lookup.findOrThrow("c_get_global_cuboid_rot"),
            FunctionDescriptor.of(
                C_QF,
                ValueLayout.JAVA_INT, ValueLayout.JAVA_LONG,
            )
        )
        getGlobalCuboidDimensionsHandle = linker.downcallHandle(
            lookup.findOrThrow("c_get_global_cuboid_dimensions"),
            FunctionDescriptor.of(
                C_F3,
                ValueLayout.JAVA_INT, ValueLayout.JAVA_LONG,
            )
        )
        isCuboidValidHandle = linker.downcallHandle(
            lookup.findOrThrow("c_is_valid"),
            FunctionDescriptor.of(
                ValueLayout.JAVA_BOOLEAN,
                ValueLayout.JAVA_INT, ValueLayout.JAVA_LONG,
            )
        )
        numBodiesHandle = linker.downcallHandle(
            lookup.findOrThrow("c_num_bodies"),
            FunctionDescriptor.of(
                ValueLayout.JAVA_INT,
                ValueLayout.JAVA_INT,
            )
        )
        getBodyHandle = linker.downcallHandle(
            lookup.findOrThrow("c_get_body"),
            FunctionDescriptor.of(
                C_BodyExternalData,
                ValueLayout.JAVA_INT, ValueLayout.JAVA_INT,
            )
        )

        greedyMeshHandle = linker.downcallHandle(
            lookup.findOrThrow("c_greedy_mesh"),
            FunctionDescriptor.of(
                ValueLayout.JAVA_INT,
                ValueLayout.JAVA_INT, ValueLayout.JAVA_INT, ValueLayout.JAVA_INT, ValueLayout.ADDRESS,
            )
        )
        numBBsHandle = linker.downcallHandle(
            lookup.findOrThrow("c_num_bbs"),
            FunctionDescriptor.of(
                ValueLayout.JAVA_INT,
                ValueLayout.JAVA_INT,
            )
        )
        getBBHandle = linker.downcallHandle(
            lookup.findOrThrow("c_get_bb"),
            FunctionDescriptor.of(
                C_FBB,
                ValueLayout.JAVA_INT, ValueLayout.JAVA_INT,
            )
        )

        libraryInit.invoke()
    }

    fun createWorld(dt: Float, acceleration: Vector3f): WorldIndex =
        WorldIndex(createWorldHandle.invokeExact(dt, acceleration.x, acceleration.y, acceleration.z) as Int)

    fun tickWorld(worldIndex: WorldIndex) {
        tickWorldHandle.invoke(worldIndex.index)
    }

    fun createCuboid(
        worldIndex: WorldIndex,
        pos: Vector3d,
        vel: Vector3f,
        ω: Vector3f,
        rot: Quaternionf,
        dimensions: Vector3f,
        inverseMass: Float,
    ): PotentialBodyHandle? {
        val arena = Arena.ofShared()
        val packedHandle = arena.allocate(ValueLayout.JAVA_INT, 2)
        packedHandle.setAtIndex(ValueLayout.JAVA_INT, 0, -1)
        val potentialBodyHandle = PotentialBodyHandle(this, worldIndex, arena, packedHandle)
        val success = createCuboidHandle.invokeExact(
            packedHandle,
            worldIndex.index,
            pos.x, pos.y, pos.z,
            vel.x, vel.y, vel.z,
            ω.x, ω.y, ω.z,
            rot.x, rot.y, rot.z, rot.w,
            dimensions.x, dimensions.y, dimensions.z,
            inverseMass,
        ) as Boolean

        if (!success) {
            arena.close()
            return null
        }

        return potentialBodyHandle
    }

    fun removeCuboid(worldIndex: WorldIndex, handle: BodyHandle) =
        removeCuboidHandle.invokeExact(worldIndex.index, handle.asLong) as Boolean

    fun isCuboidValid(worldIndex: WorldIndex, handle: BodyHandle) =
        isCuboidValidHandle.invokeExact(worldIndex.index, handle.asLong) as Boolean

    fun numBodies(worldIndex: WorldIndex): Int =
        numBodiesHandle.invokeExact(worldIndex.index) as Int

    fun getBody(tempArena: Arena, worldIndex: WorldIndex, bodyIndex: Int): BodyExternalData {
        val segment =
            getBodyHandle.invoke(
                tempArena as SegmentAllocator,
                worldIndex.index,
                bodyIndex
            ) as MemorySegment
        return BodyExternalData(
            BodyHandle(
                segment.get(ValueLayout.JAVA_INT, 52),
                segment.get(ValueLayout.JAVA_INT, 56),
            ),
            Vector3d(
                segment.get(ValueLayout.JAVA_DOUBLE, 0),
                segment.get(ValueLayout.JAVA_DOUBLE, 8),
                segment.get(ValueLayout.JAVA_DOUBLE, 16),
            ),
            Quaternionf(
                segment.get(ValueLayout.JAVA_FLOAT, 24),
                segment.get(ValueLayout.JAVA_FLOAT, 28),
                segment.get(ValueLayout.JAVA_FLOAT, 32),
                segment.get(ValueLayout.JAVA_FLOAT, 36),
            ),
            Vector3f(
                segment.get(ValueLayout.JAVA_FLOAT, 40),
                segment.get(ValueLayout.JAVA_FLOAT, 44),
                segment.get(ValueLayout.JAVA_FLOAT, 48),
            ),
        )
    }

    fun getCuboidPos(tempArena: Arena, worldIndex: WorldIndex, handle: BodyHandle): Vector3d {
        val segment =
            getGlobalCuboidPosHandle.invoke(
                tempArena as SegmentAllocator,
                worldIndex.index,
                handle.asLong
            ) as MemorySegment
        return Vector3d(
            segment.getAtIndex(ValueLayout.JAVA_DOUBLE, 0),
            segment.getAtIndex(ValueLayout.JAVA_DOUBLE, 1),
            segment.getAtIndex(ValueLayout.JAVA_DOUBLE, 2),
        )
    }

    fun getCuboidRot(tempArena: Arena, worldIndex: WorldIndex, handle: BodyHandle): Quaternionf {
        val segment =
            getGlobalCuboidRotHandle.invoke(
                tempArena as SegmentAllocator,
                worldIndex.index,
                handle.asLong
            ) as MemorySegment
        return Quaternionf(
            segment.getAtIndex(ValueLayout.JAVA_FLOAT, 0),
            segment.getAtIndex(ValueLayout.JAVA_FLOAT, 1),
            segment.getAtIndex(ValueLayout.JAVA_FLOAT, 2),
            segment.getAtIndex(ValueLayout.JAVA_FLOAT, 3),
        )
    }

    fun getCuboidDimensions(tempArena: Arena, worldIndex: WorldIndex, handle: BodyHandle): Vector3f {
        val segment =
            getGlobalCuboidDimensionsHandle.invoke(
                tempArena as SegmentAllocator,
                worldIndex.index,
                handle.asLong
            ) as MemorySegment
        return Vector3f(
            segment.getAtIndex(ValueLayout.JAVA_FLOAT, 0),
            segment.getAtIndex(ValueLayout.JAVA_FLOAT, 1),
            segment.getAtIndex(ValueLayout.JAVA_FLOAT, 2),
        )
    }

    fun greedyMesh(originX: Int, originY: Int, originZ: Int, chunkBinaryData: MemorySegment): Int =
        greedyMeshHandle.invokeExact(originX, originY, originZ, chunkBinaryData) as Int

    fun numBBs(chunkMeshIndex: Int): Int = numBBsHandle.invokeExact(chunkMeshIndex) as Int

    fun getBB(arena: Arena, chunkMeshIndex: Int, bbIndex: Int): FBB {
        val segment = getBBHandle.invokeExact(arena as SegmentAllocator, chunkMeshIndex, bbIndex) as MemorySegment
        return FBB(
            segment.get(ValueLayout.JAVA_FLOAT, 0),
            segment.get(ValueLayout.JAVA_FLOAT, 4),
            segment.get(ValueLayout.JAVA_FLOAT, 8),
            segment.get(ValueLayout.JAVA_FLOAT, 12),
            segment.get(ValueLayout.JAVA_FLOAT, 16),
            segment.get(ValueLayout.JAVA_FLOAT, 20),
        )
    }

    fun deinit() {
        try {
            deinit.invoke()
        } catch (_: Throwable) {
        }
        libArena.close()
    }
}
