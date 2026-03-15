package com.timur.nimcube

import org.joml.Vector3d
import java.lang.foreign.*
import java.lang.invoke.MethodHandle

class Nim(plugin: Nimcube) {
    private val libArena: Arena = Arena.ofShared()
    val deinit: MethodHandle

    val createWorldHandle: MethodHandle

    @JvmInline
    value class WorldIndex(val index: Int)

    val createCuboidHandle: MethodHandle

    data class BodyHandle(val slot: Int, val generation: Int)

    val C_BodyHandle = MemoryLayout.structLayout(
        ValueLayout.JAVA_INT.withName("slot"),
        ValueLayout.JAVA_INT.withName("generation"),
    )

    val getGlobalCuboidPos: MethodHandle

    val C_D3 = MemoryLayout.structLayout(
        ValueLayout.JAVA_DOUBLE.withName("x"),
        ValueLayout.JAVA_DOUBLE.withName("y"),
        ValueLayout.JAVA_DOUBLE.withName("z"),
    )

    val greedyMeshHandle: MethodHandle
    val numBBsHandle: MethodHandle
    val getBBHandle: MethodHandle

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
            lookup.findOrThrow("create_world"),
            FunctionDescriptor.of(ValueLayout.JAVA_INT)
        )

        createCuboidHandle = linker.downcallHandle(
            lookup.findOrThrow("c_create_cuboid"),
            FunctionDescriptor.of(
                C_BodyHandle,
                ValueLayout.JAVA_INT, C_D3
            )
        )
        getGlobalCuboidPos = linker.downcallHandle(
            lookup.findOrThrow("c_get_global_cuboid_pos"),
            FunctionDescriptor.of(
                C_D3,
                ValueLayout.JAVA_INT, C_BodyHandle
            )
        )

        greedyMeshHandle = linker.downcallHandle(
            lookup.findOrThrow("greedy_mesh"),
            FunctionDescriptor.of(
                ValueLayout.JAVA_INT,
                ValueLayout.JAVA_INT, ValueLayout.JAVA_INT, ValueLayout.JAVA_INT, ValueLayout.ADDRESS,
            )
        )
        numBBsHandle = linker.downcallHandle(
            lookup.findOrThrow("num_bbs"),
            FunctionDescriptor.of(
                ValueLayout.JAVA_INT,
                ValueLayout.JAVA_INT,
            )
        )
        getBBHandle = linker.downcallHandle(
            lookup.findOrThrow("get_bb"),
            FunctionDescriptor.of(
                C_FBB,
                ValueLayout.JAVA_INT, ValueLayout.JAVA_INT,
            )
        )

        libraryInit.invoke()
    }

    fun createWorld(): WorldIndex = WorldIndex(createWorldHandle.invokeExact() as Int)

    fun createCuboid(arena: Arena, worldIndex: WorldIndex, pos: Vector3d): BodyHandle {
        val c_f3 = arena.allocate(C_D3)
        c_f3.setAtIndex(ValueLayout.JAVA_DOUBLE, 0, pos.x)
        c_f3.setAtIndex(ValueLayout.JAVA_DOUBLE, 1, pos.y)
        c_f3.setAtIndex(ValueLayout.JAVA_DOUBLE, 2, pos.z)

        val segment = createCuboidHandle.invokeExact(arena as SegmentAllocator, worldIndex.index, c_f3) as MemorySegment
        return BodyHandle(
            segment.get(ValueLayout.JAVA_INT, 0),
            segment.get(ValueLayout.JAVA_INT, 4),
        )
    }

    fun getCuboidPos(arena: Arena, worldIndex: WorldIndex, handle: BodyHandle): Vector3d {
        val c_handle = arena.allocate(C_BodyHandle)
        c_handle.setAtIndex(ValueLayout.JAVA_INT, 0, handle.slot)
        c_handle.setAtIndex(ValueLayout.JAVA_INT, 1, handle.generation)

        val segment = getGlobalCuboidPos.invoke(arena as SegmentAllocator, worldIndex.index, c_handle) as MemorySegment
        return Vector3d(
            segment.get(ValueLayout.JAVA_DOUBLE, 0),
            segment.get(ValueLayout.JAVA_DOUBLE, 8),
            segment.get(ValueLayout.JAVA_DOUBLE, 16),
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