import physics_math
import command_queue
import cuboids
import island
import rw_lock

type
  World = ref object
    valid*: bool
    islands*: seq[Island]
    internal_data*: InternalData
    external_data*: ExternalData

    Δt*: float32
    acceleration*: F3

    command_queue*: CommandQueue

var worlds*: seq[World]

proc init_world*(Δt: float32, acceleration: F3): World =
  result = World(
    valid: true,
    islands: @[],
    internal_data: InternalData(),
    external_data: init_external_data(),
    Δt: Δt,
    acceleration: acceleration,
    command_queue: CommandQueue(),
  )

proc tick_world*(world_index: int) =
  if world_index >= worlds.len: return
  var world = worlds[world_index]
  if not world.valid: return

  let data = world.internal_data

  world.command_queue.process_command_queue(data, world.islands)

  let Δt = world.Δt
  let acceleration = world.acceleration

  for i in 0 ..< data.local_pos.len:
    data.vel[i] += acceleration * Δt
    data.local_pos[i] += data.vel[i] * Δt
    let ω = data.ω[i]
    let spin = quat(ω, 0'f32)
    let q = data.rot[i]
    data.rot[i] = normalized(q + (0.5'f32 * Δt) * (spin * q))

  data.update_external_data(world.external_data, world.islands)

proc global_pos*(world: World, handle: BodyHandle): D3 =
  with_read_lock(world.external_data.lock):
    if world.external_data.is_valid_no_lock(handle):
      result = world.external_data.pos[world.external_data.slot_to_dense[handle.slot]]

proc global_rot*(world: World, handle: BodyHandle): QF =
  with_read_lock(world.external_data.lock):
    if world.external_data.is_valid_no_lock(handle):
      result = world.external_data.rot[world.external_data.slot_to_dense[handle.slot]]

proc global_dimensions*(world: World, handle: BodyHandle): F3 =
  with_read_lock(world.external_data.lock):
    if world.external_data.is_valid_no_lock(handle):
      result = world.external_data.dimensions[world.external_data.slot_to_dense[handle.slot]]
