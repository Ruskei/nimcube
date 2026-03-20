import std/math
import std/tables
import std/monotimes

import chunk_positions
import physics_math
import command_queue
import constraint_solver
import cuboids
import dynamic_aabb_tree
import meshing
import narrowphase
import rw_lock
import portal
import portal_object_constructor
import part

export chunk_positions

type
  WorldObj = object
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
    a2a_warm_start: TableRef[A2aWarmStartKey, A2aWarmStartEntry]
    a2s_warm_start: TableRef[A2sWarmStartKey, A2sWarmStartEntry]
    a2a_contact_manifolds: seq[A2aCollisionManifold]
    a2s_contact_manifolds: seq[A2sCollisionManifold]

    portals: Portals
    portal_aabb_tree: DynamicAabbTree[SpecificPortalsHandle]
    external_portal_data*: ExternalPortalData
    portal_border_manifolds: seq[A2sCollisionManifold]
  World* = ptr WorldObj

const portal_border_radius = 0.15
const max_worlds* = 1024
var worlds*: array[max_worlds, World]
const chunk_range_epsilon = 1.0e-5'f32

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

proc world_at*(world_index: int): World {.inline.} =
  if world_index < 0 or world_index >= max_worlds:
    return nil
  worlds[world_index]

proc init_world*(Δt: float32, acceleration: F3): World =
  result = createShared(WorldObj)
  result.valid = true
  result.internal_data = InternalData()
  init_external_data(result.external_data)
  result.Δt = Δt
  result.acceleration = acceleration
  init_command_queue(result.command_queue)
  result.aabb_tree = init_dynamic_aabb_tree[BodyHandle](fat_margin = 0.3'f32)
  result.chunk_meshes_by_position = init_table[ChunkPosition, ChunkMesh]()
  result.narrowphase_pool = init_narrowphase_pool()
  result.a2a_warm_start = newTable[A2aWarmStartKey, A2aWarmStartEntry]()
  result.a2s_warm_start = newTable[A2sWarmStartKey, A2sWarmStartEntry]()
  result.portals = Portals()
  result.portal_aabb_tree = init_dynamic_aabb_tree[SpecificPortalsHandle](fat_margin = portal_border_radius)

proc tick_world*(world_index: int) =
  let start = get_mono_time()

  let world = world_at(world_index)
  if world.is_nil or not world.valid:
    return

  let data = world.internal_data

  world.command_queue.process_command_queue(
    data,
    world.aabb_tree,
    world.portals,
    world.portal_aabb_tree,
    world.chunk_meshes_by_position,
  )
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

          for bb_leaf_idx in mesh.aabb_tree.query(local_query):
            let bb_idx = mesh.aabb_tree.data(bb_leaf_idx)
            let local_bb = mesh.bbs[bb_idx]
            let world_bb: FBB = (
              min: local_bb.min + mesh_origin,
              max: local_bb.max + mesh_origin,
            )
            world.narrowphase_pool.add_a2s_broadphase_result((body: handle, static_bb: world_bb))

  let broadphase = get_mono_time()

  world.portal_border_manifolds.setLen 0
  
  for dense_idx in 0 ..< data.local_pos.len:
    let handle = data.body_handle_at_dense(dense_idx)
    let body_aabb = data.aabb(handle)
    let parts = construct_parts_from_body(handle, data, world.portals, world.portal_aabb_tree)
    data.set_composition(handle,
      if parts.len == 1: CompositionObj(kind: ck_simple)
      else: CompositionObj(kind: ck_parted, parts: parts)
    )

    for portal_leaf_idx in world.portal_aabb_tree.query(body_aabb):
      let portal_handle = world.portal_aabb_tree.data portal_leaf_idx
      let portal = world.portals.portal(portal_handle.handle)
      let manifold = generate_a2s_cuboid_portal_border_manifold(handle, portal_handle, world.narrowphase_pool, world.portals, portal_border_radius)
      if manifold.contact_count > 0:
        world.portal_border_manifolds.add manifold

  let portals = get_mono_time()

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

  world.velocity_constraints.precompute_velocity_constraints(
    data,
    world.narrowphase_pool,
    Δt,
    world.a2a_warm_start,
    world.a2s_warm_start,
    world.portal_border_manifolds,
  )
  world.velocity_constraints.solve_velocity_constraints(data, normal_iterations, friction_iterations, velocity_solve_sor)
  world.velocity_constraints.rebuild_warm_start_cache(world.a2a_warm_start, world.a2s_warm_start)

  let constraint_solving = get_mono_time()

  for i in 0 ..< data.local_pos.len:
    let initial_pos: F3 = data.initial_pos[i]
    let prev_pos = data.local_pos[i]

    let displacement = data.vel[i] * Δt
    data.local_pos[i] += displacement
    let ω = data.ω[i]
    let spin = quat(ω, 0'f32)
    let q = data.rot[i]
    data.rot[i] = normalized(q + (0.5'f32 * Δt) * (spin * q))
    data.update_body_collision_cache(i)

    let curr_pos = data.local_pos[i]

    let aabb = data.cached_aabb[i]
    for portal_leaf_idx in world.portal_aabb_tree.query(aabb):
      let portal_handle = world.portal_aabb_tree.data portal_leaf_idx
      let portal = world.portals.portal(portal_handle.handle)
      let (portal_from_origin, portal_from_quat, portal_to_origin, portal_to_quat) = case portal_handle.which
        of wp_a: (portal.origin_a, portal.quat_a, portal.origin_b, portal.quat_b)
        of wp_b: (portal.origin_b, portal.quat_b, portal.origin_a, portal.quat_a)
      if segment_collides_with_portal_side(initial_pos + prev_pos, initial_pos + curr_pos, portal_from_origin, portal_from_quat, portal.scale_x, portal.scale_y):
        let rotation = normalized(portal_to_quat * conjugate(portal_from_quat))
        data.local_pos[i] = portal_to_origin + rotation.rotate_vector(initial_pos + curr_pos - portal_from_origin) - initial_pos
        data.rot[i] = rotation * data.rot[i]
        data.vel[i] = rotation.rotate_vector(data.vel[i])
        data.ω[i] = rotation.rotate_vector(data.ω[i])
        data.update_body_collision_cache(i)
        break

    data.update_cuboid_aabb(world.aabb_tree, i, displacement)

  let integrating = get_mono_time()

  data.update_external_data(world.external_data)
  world.portals.update_external_data(world.external_portal_data)

  let finish = get_mono_time()

  # echo "| total=", in_microseconds(finish - start), "μs"
  # echo "| broadphase=", in_microseconds(broadphase - start), "μs"
  # echo "| broadphase=", in_microseconds(portals - broadphase), "μs"
  # echo "| narrowphase=", in_microseconds(narrowphase - portals), "μs"
  # echo "| constraint_solving=", in_microseconds(constraint_solving - narrowphase), "μs"
  # echo "| integrating=", in_microseconds(integrating - constraint_solving), "μs"

proc deinit_world*(world: var World) =
  if world.is_nil:
    return

  world.valid = false
  deinit_command_queue(world.command_queue)

  if not world.narrowphase_pool.is_nil:
    deinit_narrowphase_pool(world.narrowphase_pool)
  deinit_external_data(world.external_data)

  reset world.internal_data
  reset world.aabb_tree
  reset world.chunk_meshes_by_position
  reset world.narrowphase_pool
  reset world.velocity_constraints
  reset world.a2a_warm_start
  reset world.a2s_warm_start
  reset world.a2a_contact_manifolds
  reset world.a2s_contact_manifolds

  `=destroy`(world[])
  deallocShared(world)
  world = nil

proc deinit_worlds*() =
  for idx in 0 ..< max_worlds:
    deinit_world(worlds[idx])

proc global_pos*(world: World, handle: BodyHandle): D3 =
  with_read_lock(world.external_data.lock):
    let dense = world.external_data.dense_idx_no_lock(handle)
    if dense >= 0:
      result = world.external_data.pos[dense]

proc global_rot*(world: World, handle: BodyHandle): QF =
  with_read_lock(world.external_data.lock):
    let dense = world.external_data.dense_idx_no_lock(handle)
    if dense >= 0:
      result = world.external_data.rot[dense]

proc global_dimensions*(world: World, handle: BodyHandle): F3 =
  with_read_lock(world.external_data.lock):
    let dense = world.external_data.dense_idx_no_lock(handle)
    if dense >= 0:
      result = world.external_data.dimensions[dense]

proc write_mesh_data*(world: World, handle: BodyHandle, buffer: ptr uint8, buffer_size: int32): bool =
  if world.is_nil or buffer_size < 0:
    return false

  with_read_lock(world.external_data.lock):
    let dense = world.external_data.dense_idx_no_lock(handle)
    if dense < 0:
      return false

    let mesh = world.external_data.mesh[dense]
    if mesh.data_len > buffer_size:
      return false
    if mesh.data_len > 0 and buffer.is_nil:
      return false

    if mesh.data_len > 0:
      copyMem(buffer, mesh.data, mesh.data_len.int)
    result = true

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

proc portal_border_manifold_count*(world: World): int =
  world.portal_border_manifolds.len

proc get_portal_border_manifold*(world: World, manifold_index: int): A2sCollisionManifold =
  if manifold_index < 0 or manifold_index >= world.portal_border_manifolds.len:
    raise newException(IndexDefect, "portal border manifold index out of range")
  world.portal_border_manifolds[manifold_index]

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
