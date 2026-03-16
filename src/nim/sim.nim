import std/times
import std/monotimes

import physics_math
import command_queue
import cuboids
import dynamic_aabb_tree
import rw_lock

type
  World* = ref object
    valid*: bool
    internal_data*: InternalData
    external_data*: ExternalData

    Δt*: float32
    acceleration*: F3

    command_queue*: CommandQueue
    aabb_tree: DynamicAabbTree[BodyHandle]

var worlds*: seq[World]

proc init_world*(Δt: float32, acceleration: F3): World =
  result = World(
    valid: true,
    internal_data: InternalData(),
    external_data: init_external_data(),
    Δt: Δt,
    acceleration: acceleration,
    command_queue: CommandQueue(),
    aabb_tree: init_dynamic_aabb_tree[BodyHandle](fat_margin = 1'f32),
  )

proc tick_world*(world_index: int) =
  let start = get_mono_time()

  if world_index >= worlds.len: return
  var world = worlds[world_index]
  if not world.valid: return

  let data = world.internal_data

  world.command_queue.process_command_queue(data, world.aabb_tree)

  let Δt = world.Δt
  let acceleration = world.acceleration

  for i in 0 ..< data.local_pos.len:
    data.vel[i] += acceleration * Δt
    let displacement = data.vel[i] * Δt
    data.local_pos[i] += displacement
    let ω = data.ω[i]
    let spin = quat(ω, 0'f32)
    let q = data.rot[i]
    data.rot[i] = normalized(q + (0.5'f32 * Δt) * (spin * q))
    data.update_cuboid_aabb(world.aabb_tree, i, displacement)

  data.update_external_data(world.external_data)

  let finish = get_mono_time()
  # echo "physics tick took=", in_milliseconds(finish - start)

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

proc aabb_tree_leaf_count*(world: World): int =
  world.aabb_tree.leaf_count

proc query_aabb_tree*(world: World, aabb: FBB): seq[BodyHandle] =
  for node_idx in world.aabb_tree.query(aabb):
    result.add world.aabb_tree.data(node_idx)

proc validate_aabb_tree*(world: World) =
  world.aabb_tree.validate()

proc num_aabb_tree_nodes*(world: World): int =
  for node in world.aabb_tree.nodes:
    if node.storage == nsk_used:
      inc result

proc get_aabb_tree_node*(world: World, node_index: int): FBB =
  var seen = 0

  for node in world.aabb_tree.nodes:
    if node.storage != nsk_used:
      continue

    if seen == node_index:
      result.min = (node.min_x, node.min_y, node.min_z)
      result.max = (node.max_x, node.max_y, node.max_z)
      return

    inc seen

  result.min = (0'f32, 0'f32, 0'f32)
  result.max = (-1'f32, -1'f32, -1'f32)
