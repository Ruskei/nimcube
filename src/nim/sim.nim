import std/hashes
import std/math
import std/tables
import std/times
import std/monotimes

import physics_math
import command_queue
import constraint_solver
import cuboids
import dynamic_aabb_tree
import meshing
import narrowphase
import rw_lock

type
  ChunkPosition* = object
    x*, z*: int32

  World* = ref object
    valid*: bool
    internal_data*: InternalData
    external_data*: ExternalData

    Δt*: float32
    acceleration*: F3

    command_queue*: CommandQueue
    aabb_tree: DynamicAabbTree[BodyHandle]
    chunk_meshes_by_position*: Table[ChunkPosition, ChunkMesh]
    narrowphase_pool: NarrowphasePool
    velocity_constraints: VelocityConstraintBuffer
    a2a_contact_manifolds: seq[A2aCollisionManifold]
    a2s_contact_manifolds: seq[A2sCollisionManifold]

var worlds*: seq[World]
const chunk_range_epsilon = 1.0e-5'f32

proc hash*(chunk_pos: ChunkPosition): Hash =
  let packed =
    ((chunk_pos.x.uint32 and 0xFFFF'u32) shl 16) or
    (chunk_pos.z.uint32 and 0xFFFF'u32)
  Hash(packed)

proc chunk_position*(chunk_x, chunk_z: int32): ChunkPosition =
  ChunkPosition(x: chunk_x, z: chunk_z)

proc floor_div(value, divisor: int32): int32 {.inline.} =
  result = value div divisor
  if value < 0 and value mod divisor != 0:
    dec result

proc chunk_position_from_world_xz*(x, z: int32): ChunkPosition =
  chunk_position(
    floor_div(x, chunk_width.int32),
    floor_div(z, chunk_width.int32),
  )

proc floor_chunk_coord(value: float32): int32 {.inline.} =
  floor(value.float64 / chunk_width.float64).int32

proc chunk_range_from_world_aabb*(aabb: FBB): tuple[min_chunk_x, max_chunk_x, min_chunk_z, max_chunk_z: int32] =
  let max_x = max(aabb.min.x, aabb.max.x - chunk_range_epsilon)
  let max_z = max(aabb.min.z, aabb.max.z - chunk_range_epsilon)
  (
    min_chunk_x: floor_chunk_coord(aabb.min.x),
    max_chunk_x: floor_chunk_coord(max_x),
    min_chunk_z: floor_chunk_coord(aabb.min.z),
    max_chunk_z: floor_chunk_coord(max_z),
  )

proc init_world*(Δt: float32, acceleration: F3): World =
  result = World(
    valid: true,
    internal_data: InternalData(),
    external_data: init_external_data(),
    Δt: Δt,
    acceleration: acceleration,
    command_queue: CommandQueue(),
    aabb_tree: init_dynamic_aabb_tree[BodyHandle](fat_margin = 0.3'f32),
    chunk_meshes_by_position: init_table[ChunkPosition, ChunkMesh](),
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
  world.narrowphase_pool.clear_narrowphase_inputs()

  for a, b in world.aabb_tree.potential_pairs:
    let handle_a = world.aabb_tree.data a
    let handle_b = world.aabb_tree.data b

    let aabb_a = data.aabb handle_a
    let aabb_b = data.aabb handle_b

    if aabb_a.overlaps(aabb_b):
      world.narrowphase_pool.add_a2a_broadphase_result((handle_a, handle_b))

  if world.chunk_meshes_by_position.len > 0:
    let min_environment_y = chunk_mesh_min_y.float32
    let max_environment_y = (chunk_mesh_min_y + chunk_height).float32

    for dense_idx in 0 ..< data.local_pos.len:
      let handle = data.body_handle_at_dense(dense_idx)
      let body_aabb = data.aabb(handle)
      if body_aabb.max.y <= min_environment_y or body_aabb.min.y >= max_environment_y:
        continue

      let chunk_range = chunk_range_from_world_aabb(body_aabb)
      for chunk_x in chunk_range.min_chunk_x .. chunk_range.max_chunk_x:
        for chunk_z in chunk_range.min_chunk_z .. chunk_range.max_chunk_z:
          let chunk_pos = chunk_position(chunk_x, chunk_z)
          if not world.chunk_meshes_by_position.hasKey(chunk_pos):
            continue

          let mesh = world.chunk_meshes_by_position[chunk_pos]
          let mesh_origin: F3 = mesh.origin
          let local_query: FBB = (
            min: body_aabb.min - mesh_origin,
            max: body_aabb.max - mesh_origin,
          )

          for leaf_idx in mesh.aabb_tree.query(local_query):
            let bb_idx = mesh.aabb_tree.data(leaf_idx)
            let local_bb = mesh.bbs[bb_idx]
            let world_bb: FBB = (
              min: local_bb.min + mesh_origin,
              max: local_bb.max + mesh_origin,
            )
            world.narrowphase_pool.add_a2s_broadphase_result((body: handle, static_bb: world_bb))

  let broadphase = get_mono_time()

  world.narrowphase_pool.dispatch_narrowphase_and_wait()

  let narrowphase = get_mono_time()

  # world.a2a_contact_manifolds.setLen(0)
  # world.a2s_contact_manifolds.setLen(0)
  # for worker_idx in 0 ..< world.narrowphase_pool.worker_count:
  #   let worker_count = world.narrowphase_pool.worker_output_count(worker_idx)
  #   for output_idx in 0 ..< worker_count:
  #     let result = world.narrowphase_pool.worker_output_at(worker_idx, output_idx)
  #     case result.kind
  #     of nrk_a2a:
  #       world.a2a_contact_manifolds.add result.a2a
  #     of nrk_a2s:
  #       world.a2s_contact_manifolds.add result.a2s

  let Δt = world.Δt
  for i in 0 ..< data.local_pos.len:
    data.vel[i] += world.acceleration * Δt

  world.velocity_constraints.precompute_velocity_constraints(data, world.narrowphase_pool, Δt)
  world.velocity_constraints.solve_velocity_constraints(data, normal_iterations, friction_iterations, velocity_solve_sor)

  let constraint_solving = get_mono_time()

  # echo "integrating"
  for i in 0 ..< data.local_pos.len:
    # echo "  vel=", data.vel[i]
    let displacement = data.vel[i] * Δt
    data.local_pos[i] += displacement
    let ω = data.ω[i]
    let spin = quat(ω, 0'f32)
    let q = data.rot[i]
    data.rot[i] = normalized(q + (0.5'f32 * Δt) * (spin * q))
    data.update_body_collision_cache(i)
    data.update_cuboid_aabb(world.aabb_tree, i, displacement)

  let integrating = get_mono_time()

  data.update_external_data(world.external_data)

  let finish = get_mono_time()

  # echo "| total=", in_microseconds(finish - start), "μs"
  # echo "| broadphase=", in_microseconds(broadphase - start), "μs"
  # echo "| narrowphase=", in_microseconds(narrowphase - broadphase), "μs"
  # echo "| constraint_solving=", in_microseconds(constraint_solving - narrowphase), "μs"
  # echo "| integrating=", in_microseconds(integrating - constraint_solving), "μs"

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

proc add_chunk_mesh*(world: World, chunk_pos: ChunkPosition, chunk_binary_data: ptr ChunkBinaryData): bool =
  if world.is_nil or not world.valid or chunk_binary_data.is_nil:
    return false

  let origin_x = (chunk_pos.x * chunk_width.int32).cint
  let origin_z = (chunk_pos.z * chunk_width.int32).cint
  var chunk_mesh = build_chunk_mesh(origin_x, chunk_mesh_min_y.cint, origin_z, chunk_binary_data)
  world.chunk_meshes_by_position[chunk_pos] = move(chunk_mesh)
  true

proc aabb_tree_leaf_count*(world: World): int =
  world.aabb_tree.leaf_count

proc query_aabb_tree*(world: World, aabb: FBB): seq[BodyHandle] =
  for node_idx in world.aabb_tree.query(aabb):
    result.add world.aabb_tree.data(node_idx)

proc validate_aabb_tree*(world: World) =
  world.aabb_tree.validate()

proc a2a_narrowphase_manifold_count*(world: World): int =
  world.a2a_contact_manifolds.len

proc get_a2a_narrowphase_manifold*(world: World, manifold_index: int): A2aCollisionManifold =
  if manifold_index < 0 or manifold_index >= world.a2a_contact_manifolds.len:
    raise newException(IndexDefect, "a2a narrowphase manifold index out of range")
  world.a2a_contact_manifolds[manifold_index]

proc a2s_narrowphase_manifold_count*(world: World): int =
  world.a2s_contact_manifolds.len

proc get_a2s_narrowphase_manifold*(world: World, manifold_index: int): A2sCollisionManifold =
  if manifold_index < 0 or manifold_index >= world.a2s_contact_manifolds.len:
    raise newException(IndexDefect, "a2s narrowphase manifold index out of range")
  world.a2s_contact_manifolds[manifold_index]

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
