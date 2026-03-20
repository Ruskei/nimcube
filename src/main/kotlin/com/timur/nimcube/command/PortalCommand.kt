package com.timur.nimcube.command

import com.timur.nimcube.NimWorld
import com.timur.nimcube.Nimcube
import org.bukkit.Bukkit
import org.bukkit.command.Command
import org.bukkit.command.CommandExecutor
import org.bukkit.command.CommandSender
import org.bukkit.entity.Player
import org.joml.Quaternionf
import org.joml.Vector3f

class PortalCommand(private val plugin: Nimcube) : CommandExecutor {
    companion object {
        private const val DEFAULT_SEPARATION = 2f
    }

    private data class PortalOptions(
        val originA: Vector3f?,
        val originB: Vector3f?,
        val rotationAEulerDegrees: Vector3f?,
        val rotationBEulerDegrees: Vector3f?,
        val scale: Pair<Float, Float>,
    )

    fun init() {
        Bukkit.getPluginCommand("nim_portal")!!.setExecutor(this)
    }

    private val nim = plugin.nim

    private fun usage(label: String) =
        "/$label [--origin-a x y z|-oa x y z] [--origin-b x y z|-ob x y z] [--rotation-a pitch yaw roll|-ra p y r] [--rotation-b pitch yaw roll|-rb p y r] [--scale x y|-s x y]"

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

    private fun parseScale(
        args: Array<out String>,
        startIndex: Int,
        flag: String,
    ): Result<Pair<Pair<Float, Float>, Int>> {
        if (startIndex + 1 >= args.size) {
            return Result.failure(IllegalArgumentException("$flag requires 2 values"))
        }

        val x = args[startIndex].toFloatOrNull()
                ?: return Result.failure(IllegalArgumentException("Invalid $flag x value: ${args[startIndex]}"))
        val y = args[startIndex + 1].toFloatOrNull()
                ?: return Result.failure(IllegalArgumentException("Invalid $flag y value: ${args[startIndex + 1]}"))

        return Result.success((x to y) to (startIndex + 2))
    }

    private fun parseOptions(args: Array<out String>): Result<PortalOptions> {
        var index = 0
        var originA: Vector3f? = null
        var originB: Vector3f? = null
        var rotationA: Vector3f? = null
        var rotationB: Vector3f? = null
        var scale = 1f to 1f

        while (index < args.size) {
            when (args[index]) {
                "--origin-a", "-oa" -> {
                    val (value, nextIndex) = parseVector3(args, index + 1, "--origin-a").getOrElse {
                        return Result.failure(it)
                    }
                    originA = value
                    index = nextIndex
                }

                "--origin-b", "-ob" -> {
                    val (value, nextIndex) = parseVector3(args, index + 1, "--origin-b").getOrElse {
                        return Result.failure(it)
                    }
                    originB = value
                    index = nextIndex
                }

                "--rotation-a", "-ra" -> {
                    val (value, nextIndex) = parseVector3(args, index + 1, "--rotation-a").getOrElse {
                        return Result.failure(it)
                    }
                    rotationA = value
                    index = nextIndex
                }

                "--rotation-b", "-rb" -> {
                    val (value, nextIndex) = parseVector3(args, index + 1, "--rotation-b").getOrElse {
                        return Result.failure(it)
                    }
                    rotationB = value
                    index = nextIndex
                }

                "--scale", "-s" -> {
                    val (value, nextIndex) = parseScale(args, index + 1, "--scale").getOrElse {
                        return Result.failure(it)
                    }
                    scale = value
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

        if (scale.first <= 0f || scale.second <= 0f) {
            return Result.failure(IllegalArgumentException("Scale must be positive"))
        }

        return Result.success(
            PortalOptions(
                originA = originA,
                originB = originB,
                rotationAEulerDegrees = rotationA,
                rotationBEulerDegrees = rotationB,
                scale = scale,
            )
        )
    }

    private fun eulerToQuaternion(eulerDegrees: Vector3f?): Quaternionf =
        eulerDegrees?.let {
            Quaternionf().rotateXYZ(
                Math.toRadians(it.x.toDouble()).toFloat(),
                Math.toRadians(it.y.toDouble()).toFloat(),
                Math.toRadians(it.z.toDouble()).toFloat(),
            )
        } ?: Quaternionf()

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

        val originA = options.originA ?: Vector3f(
            sender.location.x.toFloat(),
            sender.location.y.toFloat(),
            sender.location.z.toFloat(),
        )
        val rotationA = eulerToQuaternion(options.rotationAEulerDegrees)
        val rotationB = options.rotationBEulerDegrees?.let(::eulerToQuaternion) ?: Quaternionf(rotationA)
        val originB = options.originB ?: Vector3f(0f, 0f, DEFAULT_SEPARATION).rotate(rotationA).add(Vector3f(originA))

        val potentialPortalHandle = nim.createPortal(
            nimWorld.worldIndex,
            originA,
            originB,
            rotationA,
            rotationB,
            options.scale.first,
            options.scale.second,
        ) ?: return true

        nimWorld.potentialPortalHandles += potentialPortalHandle
        return true
    }
}
