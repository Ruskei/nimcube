package com.timur.nimcube.command

import com.timur.nimcube.NimWorld
import com.timur.nimcube.Nimcube
import org.bukkit.Bukkit
import org.bukkit.command.Command
import org.bukkit.command.CommandExecutor
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
import org.joml.Quaternionf
import org.joml.Vector3d
import org.joml.Vector3f
import kotlin.math.PI
import kotlin.random.Random

class CuboidCommand(val plugin: Nimcube) : CommandExecutor {
    private data class CuboidOptions(
        val dimensions: Vector3f,
        val rotationEulerDegrees: Vector3f?,
        val angularVelocity: Vector3f,
    )

    fun init() {
        Bukkit.getPluginCommand("nim_cuboid")!!.setExecutor(this)
    }

    val nim = plugin.nim

    private fun usage(label: String) =
        "/$label [--dimensions x y z] [--rotation pitch yaw roll] [--angular-velocity x y z]"

    private fun errorMessage(label: String, message: String): String =
        if (message == "help") usage(label) else "$message. Usage: ${usage(label)}"

    private fun parseVector3(
        args: Array<out String>,
        startIndex: Int,
        flag: String,
    ): Result<Pair<Vector3f, Int>> {
        if (startIndex + 2 >= args.size) {
            return Result.failure(IllegalArgumentException("$flag requires 3 values"))
        }

        val x = args[startIndex].toFloatOrNull()
            ?: return Result.failure(IllegalArgumentException("Invalid $flag x value: ${args[startIndex]}"))
        val y = args[startIndex + 1].toFloatOrNull()
            ?: return Result.failure(IllegalArgumentException("Invalid $flag y value: ${args[startIndex + 1]}"))
        val z = args[startIndex + 2].toFloatOrNull()
            ?: return Result.failure(IllegalArgumentException("Invalid $flag z value: ${args[startIndex + 2]}"))

        return Result.success(Vector3f(x, y, z) to (startIndex + 3))
    }

    private fun parseOptions(args: Array<out String>): Result<CuboidOptions> {
        var index = 0
        var dimensions = Vector3f(1f)
        var rotationEulerDegrees: Vector3f? = null
        var angularVelocity = Vector3f()

        while (index < args.size) {
            when (args[index]) {
                "--dimensions", "-d" -> {
                    val (value, nextIndex) = parseVector3(args, index + 1, "--dimensions").getOrElse {
                        return Result.failure(it)
                    }
                    dimensions = value
                    index = nextIndex
                }
                "--rotation", "-r" -> {
                    val (value, nextIndex) = parseVector3(args, index + 1, "--rotation").getOrElse {
                        return Result.failure(it)
                    }
                    rotationEulerDegrees = value
                    index = nextIndex
                }
                "--angular-velocity", "--omega", "-w" -> {
                    val (value, nextIndex) = parseVector3(args, index + 1, "--angular-velocity").getOrElse {
                        return Result.failure(it)
                    }
                    angularVelocity = value
                    index = nextIndex
                }
                "--help", "-h" -> {
                    return Result.failure(IllegalArgumentException("help"))
                }
                else -> {
                    return Result.failure(IllegalArgumentException("Unknown argument: ${args[index]}"))
                }
            }
        }

        if (dimensions.x <= 0f || dimensions.y <= 0f || dimensions.z <= 0f) {
            return Result.failure(IllegalArgumentException("Dimensions must be positive"))
        }

        return Result.success(
            CuboidOptions(
                dimensions = dimensions,
                rotationEulerDegrees = rotationEulerDegrees,
                angularVelocity = angularVelocity,
            )
        )
    }

    override fun onCommand(
        sender: CommandSender,
        command: Command,
        label: String,
        args: Array<out String>,
    ): Boolean {
        if (sender !is Player) return true
        val options = parseOptions(args).getOrElse {
            sender.sendMessage(errorMessage(label, it.message ?: "Invalid arguments"))
            return true
        }
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
        val rotation =
            options.rotationEulerDegrees?.let {
                Quaternionf().rotateXYZ(
                    Math.toRadians(it.x.toDouble()).toFloat(),
                    Math.toRadians(it.y.toDouble()).toFloat(),
                    Math.toRadians(it.z.toDouble()).toFloat(),
                )
            } ?: Quaternionf()
        val potentialBodyHandle = nim.createCuboid(
            nimWorld.worldIndex,
            Vector3d(
                sender.location.x,
                sender.location.y,
                sender.location.z,
            ),
            Vector3f(),
            options.angularVelocity,
            rotation,
            options.dimensions,
            0.5f,
        )

        nimWorld.potentialBodyHandles += potentialBodyHandle

        return true
    }
}
