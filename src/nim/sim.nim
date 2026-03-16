import std/times
import std/monotimes

import physics_math
import command_queue
import cuboids
import dynamic_aabb_tree
import narrowphase
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
    narrowphase_pool: NarrowphasePool
    contact_manifolds: seq[CollisionManifold]

var worlds*: seq[World]

proc init_world*(Δt: float32, acceleration: F3): World =
  result = World(
    valid: true,
    internal_data: InternalData(),
    external_data: init_external_data(),
    Δt: Δt,
    acceleration: acceleration,
    command_queue: CommandQueue(),
    aabb_tree: init_dynamic_aabb_tree[BodyHandle](fat_margin = 0.3'f32),
    narrowphase_pool: init_narrowphase_pool(),
  )

proc tick_world*(world_index: int) =
  let start = get_mono_time()

  if world_index >= worlds.len: return
  var world = worlds[world_index]
  if not world.valid: return

  let data = world.internal_data

  world.command_queue.process_command_queue(data, world.aabb_tree)
  world.narrowphase_pool.set_body_inputs(
    data.slot_count(),
    data.local_pos.len,
    data.slot_to_dense_ptr(),
    data.cached_center_ptr(),
    data.cached_half_extents_ptr(),
    data.cached_world_axes_ptr(),
  )
  world.narrowphase_pool.clear_broadphase_results()

  for a, b in world.aabb_tree.potential_pairs:
    let handle_a = world.aabb_tree.data a
    let handle_b = world.aabb_tree.data b

    let aabb_a = data.aabb handle_a
    let aabb_b = data.aabb handle_b

    if aabb_a.overlaps(aabb_b):
      world.narrowphase_pool.add_broadphase_result((handle_a, handle_b))

  world.narrowphase_pool.dispatch_narrowphase_and_wait()
  let manifold_count = world.narrowphase_pool.total_manifold_count()
  world.contact_manifolds.setLen(manifold_count)
  var manifold_idx = 0
  for worker_idx in 0 ..< world.narrowphase_pool.worker_count:
    let worker_count = world.narrowphase_pool.worker_output_count(worker_idx)
    for output_idx in 0 ..< worker_count:
      world.contact_manifolds[manifold_idx] = world.narrowphase_pool.worker_output_at(worker_idx, output_idx)
      inc manifold_idx

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
    data.update_body_collision_cache(i)
    data.update_cuboid_aabb(world.aabb_tree, i, displacement)

  data.update_external_data(world.external_data)

  let finish = get_mono_time()
  # echo "physics tick took=", in_milliseconds(finish - start)

proc deinit_world*(world: World) =
  if world.is_nil or not world.valid:
    return

  deinit_narrowphase_pool(world.narrowphase_pool)
  deinit_rw_lock(world.external_data.lock)
  world.valid = false

proc deinit_worlds*() =
  for world in worlds:
    world.deinit_world()

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

proc narrowphase_manifold_count*(world: World): int =
  world.contact_manifolds.len

proc get_narrowphase_manifold*(world: World, manifold_index: int): CollisionManifold =
  if manifold_index < 0 or manifold_index >= world.contact_manifolds.len:
    raise newException(IndexDefect, "narrowphase manifold index out of range")
  world.contact_manifolds[manifold_index]

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
