import sim
import command_queue
import packed_handle
import physics_math
import meshing
import cuboids
import narrowphase
import rw_lock
import portal

type
  C_D3 {.bycopy.} = object
    x, y, z: float64
  C_F3 {.bycopy.} = object
    x, y, z: float32
  C_QF {.bycopy.} = object
    x, y, z, w: float32
  C_BodyExternalData {.bycopy.} = object
    pos_x, pos_y, pos_z: float64
    rot_x, rot_y, rot_z, rot_w: float32
    dimensions_x, dimensions_y, dimensions_z: float32
    handle_slot, handle_generation: int32
  C_PortalData {.bycopy.} = object
    origin_a_x, origin_a_y, origin_a_z: float32
    origin_b_x, origin_b_y, origin_b_z: float32
    quat_a_x, quat_a_y, quat_a_z, quat_a_w: float32
    quat_b_x, quat_b_y, quat_b_z, quat_b_w: float32
    scale_x, scale_y: float32
  C_FBB {.bycopy.} = object
    min_x, min_y, min_z, max_x, max_y, max_z: float32
  C_CollisionContactPoint {.bycopy.} = object
    pos_x, pos_y, pos_z: float32
    normal_x, normal_y, normal_z: float32
    penetration_depth: float32
  C_CollisionManifold {.bycopy.} = object
    contact_count: int32
    body_a_slot, body_a_generation: int32
    body_b_slot, body_b_generation: int32
    contact_points: array[4, C_CollisionContactPoint]
  C_A2sCollisionManifold {.bycopy.} = object
    contact_count: int32
    body_a_slot, body_a_generation: int32
    contact_points: array[4, C_CollisionContactPoint]

converter d3_to_c(v: D3): C_D3 =
  result.x = v.x
  result.y = v.y
  result.z = v.z

converter f3_to_c*(v: F3): C_F3 =
  result.x = v.x
  result.y = v.y
  result.z = v.z

converter d3_from_c(v: C_D3): D3 =
  result.x = v.x
  result.y = v.y
  result.z = v.z

converter qf_to_c*(q: QF): C_QF =
  result.x = q.x
  result.y = q.y
  result.z = q.z
  result.w = q.w

proc invalid_body_external_data(): C_BodyExternalData =
  result = C_BodyExternalData(
    pos_x: 0'f64, pos_y: 0'f64, pos_z: 0'f64,
    rot_x: 0'f32, rot_y: 0'f32, rot_z: 0'f32, rot_w: 1'f32,
    dimensions_x: -1'f32, dimensions_y: -1'f32, dimensions_z: -1'f32,
    handle_slot: -1'i32, handle_generation: 0'i32,
  )

proc invalid_packed_handle(): PackedHandle =
  PackedHandle(slot: -1'i32, generation: 0'i32)

converter to_c(portal: Portal): C_PortalData =
  result = C_PortalData(
    origin_a_x: portal.origin_a.x, origin_a_y: portal.origin_a.y, origin_a_z: portal.origin_a.z,
    origin_b_x: portal.origin_b.x, origin_b_y: portal.origin_b.y, origin_b_z: portal.origin_b.z,
    quat_a_x: portal.quat_a.x, quat_a_y: portal.quat_a.y, quat_a_z: portal.quat_a.z, quat_a_w: portal.quat_a.w,
    quat_b_x: portal.quat_b.x, quat_b_y: portal.quat_b.y, quat_b_z: portal.quat_b.z, quat_b_w: portal.quat_b.w,
    scale_x: portal.scale_x, scale_y: portal.scale_y,
  )

proc invalid_fbb(): C_FBB =
  result = C_FBB(
    min_x: 0'f32, min_y: 0'f32, min_z: 0'f32,
    max_x: -1'f32, max_y: -1'f32, max_z: -1'f32,
  )

proc invalid_collision_manifold(): C_CollisionManifold =
  result = C_CollisionManifold(
    contact_count: -1'i32,
    body_a_slot: -1'i32,
    body_a_generation: 0'i32,
    body_b_slot: -1'i32,
    body_b_generation: 0'i32,
  )

proc invalid_a2s_collision_manifold(): C_A2sCollisionManifold =
  result = C_A2sCollisionManifold(
    contact_count: -1'i32,
    body_a_slot: -1'i32,
    body_a_generation: 0'i32,
  )

converter to_c(contact_point: ContactPoint): C_CollisionContactPoint =
  result.pos_x = contact_point.position.x
  result.pos_y = contact_point.position.y
  result.pos_z = contact_point.position.z
  result.normal_x = contact_point.normal.x
  result.normal_y = contact_point.normal.y
  result.normal_z = contact_point.normal.z
  result.penetration_depth = contact_point.penetration_depth

converter to_c(manifold: A2aCollisionManifold): C_CollisionManifold =
  result.contact_count = manifold.contact_count.int32
  result.body_a_slot = manifold.body_a.slot.int32
  result.body_a_generation = manifold.body_a.generation.int32
  result.body_b_slot = manifold.body_b.slot.int32
  result.body_b_generation = manifold.body_b.generation.int32
  for idx in 0 ..< manifold.contact_points.len:
    result.contact_points[idx] = manifold.contact_points[idx]

converter to_c(manifold: A2sCollisionManifold): C_A2sCollisionManifold =
  result.contact_count = manifold.contact_count.int32
  result.body_a_slot = manifold.body_a.slot.int32
  result.body_a_generation = manifold.body_a.generation.int32
  for idx in 0 ..< manifold.contact_points.len:
    result.contact_points[idx] = manifold.contact_points[idx]

proc get_world(world_index: cint): World =
  result = world_at(world_index.int)
  if result.is_nil or not result.valid:
    result = nil

proc c_create_world(
  Δt, acceleration_x, acceleration_y, acceleration_z: float32
): cint {.cdecl, exportc, dynlib.} =
  for index in 0 ..< max_worlds:
    if worlds[index].is_nil:
      worlds[index] = init_world(Δt, (acceleration_x, acceleration_y, acceleration_z))
      return index.cint
  result = -1

proc c_tick_world(
  world_index: cint
) {.cdecl, exportc, dynlib.} =
  tick_world world_index.int

proc c_create_cuboid(
  packed_handle: ptr PackedHandle,
  world_index: cint;
  pos_x, pos_y, pos_z: float64;
  vel_x, vel_y, vel_z,
  ω_x, ω_y, ω_z,
  rot_x, rot_y, rot_z, rot_w,
  dimensions_x, dimensions_y, dimensions_z: float32;
  inverse_mass: float32;
): bool {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil:
    return false

  world.command_queue.add Command(
    kind: ck_add,
    pos: (pos_x, pos_y, pos_z),
    vel: (vel_x, vel_y, vel_z),
    ω: (ω_x, ω_y, ω_z),
    rot: (rot_x, rot_y, rot_z, rot_w),
    dimensions: (dimensions_x, dimensions_y, dimensions_z),
    inverse_mass: inverse_mass,
    packed_handle: packed_handle,
  )

  result = true

proc c_create_portal(
  packed_handle: ptr PackedHandle,
  world_index: cint;
  origin_a_x, origin_a_y, origin_a_z,
  origin_b_x, origin_b_y, origin_b_z,
  quat_a_x, quat_a_y, quat_a_z, quat_a_w,
  quat_b_x, quat_b_y, quat_b_z, quat_b_w,
  scale_x, scale_y: float32;
): bool {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil:
    return false

  world.command_queue.add Command(
    kind: ck_add_portal,
    origin_a: (origin_a_x, origin_a_y, origin_a_z),
    origin_b: (origin_b_x, origin_b_y, origin_b_z),
    quat_a: (quat_a_x, quat_a_y, quat_a_z, quat_a_w),
    quat_b: (quat_b_x, quat_b_y, quat_b_z, quat_b_w),
    scale_x: scale_x,
    scale_y: scale_y,
    portal_packed_handle: packed_handle,
  )

  result = true

proc c_remove_cuboid(world_index: cint, handle: PackedHandle): bool {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil:
    return false
  if not world.external_data.is_valid handle: return false

  world.command_queue.add Command(
    kind: ck_remove,
    handle: handle,
  )

  result = true

proc c_remove_portal(world_index: cint, handle: PackedHandle): bool {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil:
    return false
  if not world.external_portal_data.is_valid(handle):
    return false

  world.command_queue.add Command(
    kind: ck_remove_portal,
    portal_handle: handle,
  )

  result = true

proc c_is_valid(world_index: cint, handle: PackedHandle): bool {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil:
    return false

  result = world.external_data.is_valid handle

proc c_num_bodies(world_index: cint): cint {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil:
    return -1

  with_read_lock(world.external_data.lock):
    result = world.external_data.body_count.cint

proc c_get_body(world_index: cint, body_index: cint): C_BodyExternalData {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil: return invalid_body_external_data()
  if body_index < 0: return invalid_body_external_data()

  with_read_lock(world.external_data.lock):
    let dense = body_index.int
    if dense >= world.external_data.body_count:
      return invalid_body_external_data()

    let pos = world.external_data.pos[dense]
    let rot = world.external_data.rot[dense]
    let dimensions = world.external_data.dimensions[dense]
    let slot = world.external_data.dense_to_slot[dense]
    let generation = world.external_data.generation[slot]
    result = C_BodyExternalData(
      pos_x: pos.x, pos_y: pos.y, pos_z: pos.z,
      rot_x: rot.x, rot_y: rot.y, rot_z: rot.z, rot_w: rot.w,
      dimensions_x: dimensions.x, dimensions_y: dimensions.y, dimensions_z: dimensions.z,
      handle_slot: slot.int32, handle_generation: generation.int32,
    )

proc c_get_portal(world_index: cint, handle: PackedHandle): C_PortalData {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil:
    return invalid_portal
  result = world.external_portal_data.portal(handle)

proc c_get_portal_handle(world_index: cint, portal_index: cint): PackedHandle {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil:
    return invalid_packed_handle()
  if portal_index < 0:
    return invalid_packed_handle()

  with_read_lock(world.external_portal_data.lock):
    let dense = portal_index.int
    if dense >= world.external_portal_data.portal_count:
      return invalid_packed_handle()

    let slot = world.external_portal_data.dense_to_slot[dense]
    let generation = world.external_portal_data.generation[slot]
    result = PackedHandle(slot: slot.int32, generation: generation.int32)

proc c_get_global_cuboid_pos(world_index: cint, handle: PackedHandle): C_D3 {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil: return C_D3(x: 0'f64, y: 0'f64, z: 0'f64)
  result = world.global_pos handle

proc c_get_global_cuboid_rot(world_index: cint, handle: PackedHandle): C_QF {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil: return C_QF(x: 0'f32, y: 0'f32, z: 0'f32, w: 1'f32)
  result = world.global_rot handle

proc c_get_global_cuboid_dimensions(world_index: cint, handle: PackedHandle): C_F3 {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil: return C_F3(x: 0'f32, y: 0'f32, z: 0'f32)
  result = world.global_dimensions handle

proc c_get_cuboid_mesh_data(
  buffer: ptr uint8,
  buffer_size: cint,
  world_index: cint,
  handle: PackedHandle,
): bool {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil:
    return false

  result = world.write_mesh_data(handle, buffer, buffer_size.int32)

proc c_num_aabb_tree_nodes(world_index: cint): cint {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil: return -1
  result = world.num_aabb_tree_nodes().cint

proc c_get_aabb_tree_node(world_index: cint, node_index: cint): C_FBB {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil: return invalid_fbb()
  if node_index < 0: return invalid_fbb()

  let bb = world.get_aabb_tree_node(node_index.int)
  result.min_x = bb.min.x
  result.min_y = bb.min.y
  result.min_z = bb.min.z
  result.max_x = bb.max.x
  result.max_y = bb.max.y
  result.max_z = bb.max.z

proc c_get_a2a_collision_result(world_index: cint, collision_index: cint): C_CollisionManifold {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil: return invalid_collision_manifold()
  if collision_index < 0: return invalid_collision_manifold()
  if collision_index.int >= world.a2a_narrowphase_manifold_count():
    return invalid_collision_manifold()

  result = world.get_a2a_narrowphase_manifold(collision_index.int)

proc c_get_a2s_collision_result(world_index: cint, collision_index: cint): C_A2sCollisionManifold {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil: return invalid_a2s_collision_manifold()
  if collision_index < 0: return invalid_a2s_collision_manifold()
  if collision_index.int >= world.a2s_narrowphase_manifold_count():
    return invalid_a2s_collision_manifold()

  result = world.get_a2s_narrowphase_manifold(collision_index.int)

proc c_greedy_mesh*(
  origin_x, origin_y, origin_z: cint;
  chunk_binary_data: ptr ChunkBinaryData,
): cint {.cdecl, exportc, dynlib.} =
  result = greedy_mesh(
    origin_x, origin_y, origin_z,
    chunk_binary_data,
  )

proc c_add_chunk_mesh_to_world*(
  world_index, chunk_x, chunk_z: cint;
  chunk_binary_data: ptr ChunkBinaryData,
): bool {.cdecl, exportc, dynlib.} =
  let world = get_world(world_index)
  if world.is_nil:
    return false

  let chunk_data_copy = cast[ptr ChunkBinaryData](allocShared0(sizeof(ChunkBinaryData)))
  copyMem(chunk_data_copy, chunk_binary_data, sizeof(ChunkBinaryData))

  world.command_queue.add Command(
    kind: ck_add_mesh,
    chunk_x: chunk_x.int32,
    chunk_z: chunk_z.int32,
    chunk_binary_data: chunk_data_copy,
  )

proc c_num_bbs(chunk_mesh_index: cint): cint {.cdecl, exportc, dynlib.} =
  if chunk_mesh_index < 0 or chunk_mesh_index >= chunk_meshes.len:
    return -1
  let mesh = chunk_meshes[chunk_mesh_index.int]
  if mesh.is_nil:
    return -1
  result = mesh.bbs.len.cint

proc c_get_bb(chunk_mesh_index: cint, bb_index: cint): C_FBB {.cdecl, exportc, dynlib.} =
  if chunk_mesh_index < 0 or chunk_mesh_index >= chunk_meshes.len:
    return invalid_fbb()
  let mesh = chunk_meshes[chunk_mesh_index.int]
  if mesh.is_nil or bb_index < 0 or bb_index >= mesh.bbs.len:
    return invalid_fbb()

  let bb = mesh.bbs[bb_index.int]
  let origin: F3 = mesh.origin
  result.min_x = origin.x + bb.min.x
  result.min_y = origin.y + bb.min.y
  result.min_z = origin.z + bb.min.z
  result.max_x = origin.x + bb.max.x
  result.max_y = origin.y + bb.max.y
  result.max_z = origin.z + bb.max.z
