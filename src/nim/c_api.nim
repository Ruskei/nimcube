import sim
import command_queue
import packed_handle
import physics_math
import meshing
import cuboids
import rw_lock

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
  C_FBB {.bycopy.} = object
    min_x, min_y, min_z, max_x, max_y, max_z: float32

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

proc c_create_world(
  Δt, acceleration_x, acceleration_y, acceleration_z: float32
): cint {.cdecl, exportc, dynlib.} =
  let index = worlds.len
  worlds.add init_world(Δt, (acceleration_x, acceleration_y, acceleration_z))
  result = index.cint

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
  if world_index >= worlds.len: return false
  let world = worlds[world_index]
  if not world.valid: return false

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

proc c_remove_cuboid(world_index: cint, handle: PackedHandle): bool {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return false
  let world = worlds[world_index]
  if not world.valid: return false
  if not world.external_data.is_valid handle: return false

  world.command_queue.add Command(
    kind: ck_remove,
    handle: handle,
  )

  result = true

proc c_is_valid(world_index: cint, handle: PackedHandle): bool {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return false
  let world = worlds[world_index]
  if not world.valid: return false

  result = world.external_data.is_valid handle

proc c_num_bodies(world_index: cint): cint {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return -1
  let world = worlds[world_index]
  if not world.valid: return -1

  with_read_lock(world.external_data.lock):
    result = world.external_data.pos.len.cint

proc c_get_body(world_index: cint, body_index: cint): C_BodyExternalData {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return invalid_body_external_data()
  let world = worlds[world_index]
  if not world.valid: return invalid_body_external_data()
  if body_index < 0: return invalid_body_external_data()

  with_read_lock(world.external_data.lock):
    let dense = body_index.int
    if dense >= world.external_data.pos.len:
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

proc c_get_global_cuboid_pos(world_index: cint, handle: PackedHandle): C_D3 {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return C_D3(x: 0'f64, y: 0'f64, z: 0'f64)
  let world = worlds[world_index]
  if not world.valid: return C_D3(x: 0'f64, y: 0'f64, z: 0'f64)
  result = world.global_pos handle

proc c_get_global_cuboid_rot(world_index: cint, handle: PackedHandle): C_QF {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return C_QF(x: 0'f32, y: 0'f32, z: 0'f32, w: 1'f32)
  let world = worlds[world_index]
  if not world.valid: return C_QF(x: 0'f32, y: 0'f32, z: 0'f32, w: 1'f32)
  result = world.global_rot handle

proc c_get_global_cuboid_dimensions(world_index: cint, handle: PackedHandle): C_F3 {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return C_F3(x: 0'f32, y: 0'f32, z: 0'f32)
  let world = worlds[world_index]
  if not world.valid: return C_F3(x: 0'f32, y: 0'f32, z: 0'f32)
  result = world.global_dimensions handle

proc c_greedy_mesh*(
  origin_x, origin_y, origin_z: cint;
  chunk_binary_data: ptr ChunkBinaryData,
): cint {.cdecl, exportc, dynlib.} =
  result = greedy_mesh(
    origin_x, origin_y, origin_z,
    chunk_binary_data,
  )

proc c_num_bbs(chunk_mesh_index: cint): cint {.cdecl, exportc, dynlib.} =
  if chunk_mesh_index >= chunk_meshes.len:
    return -1
  result = chunk_meshes[chunk_mesh_index].bbs.len.cint

proc c_get_bb(chunk_mesh_index: cint, bb_index: cint): C_FBB {.cdecl, exportc, dynlib.} =
  let bb = chunk_meshes[chunk_mesh_index].bbs[bb_index]
  let origin: F3 = chunk_meshes[chunk_mesh_index].origin
  result.min_x = origin.x + bb.min.x
  result.min_y = origin.y + bb.min.y
  result.min_z = origin.z + bb.min.z
  result.max_x = origin.x + bb.max.x
  result.max_y = origin.y + bb.max.y
  result.max_z = origin.z + bb.max.z
