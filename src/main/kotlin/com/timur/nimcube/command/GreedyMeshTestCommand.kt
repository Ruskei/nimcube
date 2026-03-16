package com.timur.nimcube.command

import com.timur.nimcube.NimWorld
import com.timur.nimcube.Nimcube
import net.minecraft.core.BlockPos
import org.bukkit.Bukkit
import org.bukkit.command.Command
import org.bukkit.command.CommandExecutor
import org.bukkit.command.CommandSender
import org.bukkit.craftbukkit.CraftWorld
import org.bukkit.entity.Player
import java.lang.foreign.Arena
import java.lang.foreign.ValueLayout
import kotlin.time.measureTime

class GreedyMeshTestCommand(val plugin: Nimcube) : CommandExecutor {
    fun init() {
        Bukkit.getPluginCommand("nim_greedy_test")!!.setExecutor(this)
    }

    val nim = plugin.nim

    override fun onCommand(
        sender: CommandSender,
        command: Command,
        alias: String,
        args: Array<out String>,
    ): Boolean {
        if (sender !is Player) return true
        val world = sender.world
        val nimWorld =
            plugin.nimWorlds.getOrPut(world) {
                NimWorld(
                    plugin,
                    world,
                    plugin.Δt,
                    plugin.acceleration,
                ).also { it.init() }
            }
        var added = false
        val totalDuration = measureTime {
            val serverLevel = (sender.world as CraftWorld).handle

            Arena.ofConfined().use { arena ->
                val binarySegment = arena.allocate(CHUNK_BINARY_DATA_SIZE.toLong())
                val chunkX = Math.floorDiv(sender.location.blockX, PHYSICS_CHUNK_WIDTH)
                val chunkZ = Math.floorDiv(sender.location.blockZ, PHYSICS_CHUNK_WIDTH)
                val minX = chunkX * PHYSICS_CHUNK_WIDTH
                val minZ = chunkZ * PHYSICS_CHUNK_WIDTH

                val bp = BlockPos(0, 0, 0).mutable()
                for (y in 0..<CHUNK_HEIGHT)
                    for (z in 0..<PHYSICS_CHUNK_WIDTH)
                        for (x in 0..<PHYSICS_CHUNK_WIDTH) {
                            bp.x = minX + x
                            bp.y = MIN_HEIGHT + y
                            bp.z = minZ + z
                            val state = serverLevel.getBlockState(bp)
                            if (!state.isCollisionShapeFullBlock(serverLevel, bp)) continue
                            val index = y * PHYSICS_CHUNK_WIDTH * PHYSICS_CHUNK_WIDTH + z * PHYSICS_CHUNK_WIDTH + x
                            val longIndex = index / 64
                            val bitIndex = index - longIndex * 64
                            var long = binarySegment.getAtIndex(ValueLayout.JAVA_LONG, longIndex.toLong())
                            long = long or (1L shl bitIndex)
                            binarySegment.setAtIndex(ValueLayout.JAVA_LONG, longIndex.toLong(), long)
                        }

                added = nim.addChunkMeshToWorld(nimWorld.worldIndex, chunkX, chunkZ, binarySegment)
            }
        }

        sender.sendMessage("chunk mesh added=$added in $totalDuration")
        println("Total duration: $totalDuration")

        return true
    }
}

private const val PHYSICS_CHUNK_WIDTH = 64
private const val MIN_HEIGHT = -64
private const val CHUNK_HEIGHT = 384

// ceil int division
private const val CHUNK_BINARY_DATA_SIZE = PHYSICS_CHUNK_WIDTH * PHYSICS_CHUNK_WIDTH * CHUNK_HEIGHT / 8

/*
we just create an array of bounding boxes
struct BoundingBox{
    f32 min_x
    f32 min_y
    f32 min_z
    f32 max_x
    f32 max_y
    f32 max_z
}
we need nim to have access to all the bounding boxes, so we'll pass in the bounding boxes
for non full blocks and another buffer for the full blocks as binary
 */
