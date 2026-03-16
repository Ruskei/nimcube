## The problem this solves is having fast data storage for cuboids while needing fast read access from other threads
## This is done by having a main mutable buffer that the physics engine uses while also having a second read-only
## buffer for external threads. This is managed with a read-write lock, which is faster with double-buffering since
## we only block for the duration it takes to update the published data.

import physics_math
import rw_lock
import packed_handle
import dynamic_aabb_tree

type
  InternalData* = ref object
    initial_pos*: seq[D3]
    local_pos*: seq[F3]
    vel*: seq[F3]
    ω*: seq[F3]
    rot*: seq[QF]
    dimensions*: seq[F3]
    inverse_mass*: seq[float32]
    cached_center: seq[F3]
    cached_half_extents: seq[F3]
    cached_world_axes: seq[array[3, F3]]
    cached_aabb: seq[FBB]

    dense_to_slot: seq[int]
    slot_to_dense: seq[int]
    generation: seq[uint]
    free_slots: seq[int]
    aabb_node_idx: seq[NodeIndex]
  ExternalData* = ref object
    pos*: seq[D3]
    rot*: seq[QF]
    dimensions*: seq[F3]

    dense_to_slot*: seq[int]
    slot_to_dense*: seq[int]
    generation*: seq[uint]

    lock*: RwLock
  BodyHandle* = object
    slot*: int
    generation*: uint

const slot_invalid = -1

proc init_external_data*(): ExternalData =
  result = ExternalData()
  init_rw_lock result.lock

converter from_packed*(handle: PackedHandle): BodyHandle =
  result.slot = handle.slot
  result.generation = handle.generation.uint

proc is_valid*(data: InternalData, handle: BodyHandle): bool =
  return
    handle.slot >= 0 and
    handle.slot < data.generation.len and
    data.generation[handle.slot] == handle.generation and
    data.slot_to_dense[handle.slot] != slot_invalid

proc is_valid_no_lock*(data: ExternalData, handle: BodyHandle): bool =
  return
    handle.slot >= 0 and
    handle.slot < data.generation.len and
    data.generation[handle.slot] == handle.generation and
    data.slot_to_dense[handle.slot] != slot_invalid

proc is_valid*(data: ExternalData, handle: BodyHandle): bool =
  var is_valid = false
  with_read_lock(data.lock):
    is_valid =
      handle.slot >= 0 and
      handle.slot < data.generation.len and
      data.generation[handle.slot] == handle.generation and
      data.slot_to_dense[handle.slot] != slot_invalid
  result = is_valid

proc compute_body_aabb(center: F3, half_extents: F3, world_axes: array[3, F3]): FBB =
  let local_corners: array[8, F3] = [
    (x:  half_extents.x, y:  half_extents.y, z:  half_extents.z),
    (x:  half_extents.x, y:  half_extents.y, z: -half_extents.z),
    (x:  half_extents.x, y: -half_extents.y, z:  half_extents.z),
    (x:  half_extents.x, y: -half_extents.y, z: -half_extents.z),
    (x: -half_extents.x, y:  half_extents.y, z:  half_extents.z),
    (x: -half_extents.x, y:  half_extents.y, z: -half_extents.z),
    (x: -half_extents.x, y: -half_extents.y, z:  half_extents.z),
    (x: -half_extents.x, y: -half_extents.y, z: -half_extents.z),
  ]

  let first_corner =
    center +
    local_corners[0].x * world_axes[0] +
    local_corners[0].y * world_axes[1] +
    local_corners[0].z * world_axes[2]
  result.min = first_corner
  result.max = first_corner

  for i in 1 ..< local_corners.len:
    let corner =
      center +
      local_corners[i].x * world_axes[0] +
      local_corners[i].y * world_axes[1] +
      local_corners[i].z * world_axes[2]
    if corner.x < result.min.x: result.min.x = corner.x
    if corner.y < result.min.y: result.min.y = corner.y
    if corner.z < result.min.z: result.min.z = corner.z
    if corner.x > result.max.x: result.max.x = corner.x
    if corner.y > result.max.y: result.max.y = corner.y
    if corner.z > result.max.z: result.max.z = corner.z

proc update_body_collision_cache*(data: InternalData, dense_idx: int) =
  let center: F3 = (
    x: (data.initial_pos[dense_idx].x + data.local_pos[dense_idx].x.float64).float32,
    y: (data.initial_pos[dense_idx].y + data.local_pos[dense_idx].y.float64).float32,
    z: (data.initial_pos[dense_idx].z + data.local_pos[dense_idx].z.float64).float32,
  )
  let half_extents = data.dimensions[dense_idx] * 0.5'f32
  let q = data.rot[dense_idx]
  let axis_x = normalized(rotate_vector(q, (1'f32, 0'f32, 0'f32)))
  let axis_y = normalized(rotate_vector(q, (0'f32, 1'f32, 0'f32)))
  let axis_z = normalized(rotate_vector(q, (0'f32, 0'f32, 1'f32)))

  data.cached_center[dense_idx] = center
  data.cached_half_extents[dense_idx] = half_extents
  data.cached_world_axes[dense_idx] = [axis_x, axis_y, axis_z]
  data.cached_aabb[dense_idx] = compute_body_aabb(center, half_extents, data.cached_world_axes[dense_idx])

proc create_cuboid*(
  data: InternalData,
  aabb_tree: DynamicAabbTree[BodyHandle],
  initial_pos: D3,
  vel: F3,
  ω: F3,
  rot: QF,
  dimensions: F3,
  inverse_mass: float32,
): BodyHandle =
  var slot: int
  if data.free_slots.len > 0:
    slot = data.free_slots.pop()
  else:
    slot = data.generation.len
    data.generation.add 0
    data.slot_to_dense.add slot_invalid

  let dense = data.local_pos.len
  data.initial_pos.add initial_pos
  data.local_pos.add (0'f32, 0'f32, 0'f32)
  data.vel.add vel
  data.ω.add ω
  data.rot.add normalized(rot)
  data.dimensions.add dimensions
  data.inverse_mass.add inverse_mass
  data.cached_center.add default(F3)
  data.cached_half_extents.add default(F3)
  data.cached_world_axes.add default(array[3, F3])
  data.cached_aabb.add default(FBB)
  data.dense_to_slot.add slot
  data.aabb_node_idx.add invalid_node_index
  data.slot_to_dense[slot] = dense

  result = BodyHandle(slot: slot, generation: data.generation[slot])
  data.update_body_collision_cache(dense)
  data.aabb_node_idx[dense] = aabb_tree.insert(data.cached_aabb[dense], result)

proc remove_cuboid*(data: InternalData, aabb_tree: DynamicAabbTree[BodyHandle], handle: BodyHandle): bool =
  if not data.is_valid(handle):
    return false

  let slot = handle.slot
  let dense = data.slot_to_dense[slot]
  let last_dense = data.local_pos.len - 1
  let node_idx = data.aabb_node_idx[dense]

  if node_idx != invalid_node_index:
    discard aabb_tree.remove(node_idx)

  if dense != last_dense:
    let moved_slot = data.dense_to_slot[last_dense]
    data.initial_pos[dense] = data.initial_pos[last_dense]
    data.local_pos[dense] = data.local_pos[last_dense]
    data.vel[dense] = data.vel[last_dense]
    data.ω[dense] = data.ω[last_dense]
    data.rot[dense] = data.rot[last_dense]
    data.dimensions[dense] = data.dimensions[last_dense]
    data.inverse_mass[dense] = data.inverse_mass[last_dense]
    data.cached_center[dense] = data.cached_center[last_dense]
    data.cached_half_extents[dense] = data.cached_half_extents[last_dense]
    data.cached_world_axes[dense] = data.cached_world_axes[last_dense]
    data.cached_aabb[dense] = data.cached_aabb[last_dense]
    data.aabb_node_idx[dense] = data.aabb_node_idx[last_dense]
    data.dense_to_slot[dense] = moved_slot
    data.slot_to_dense[moved_slot] = dense

  data.initial_pos.setLen last_dense
  data.local_pos.setLen last_dense
  data.vel.setLen last_dense
  data.ω.setLen last_dense
  data.rot.setLen last_dense
  data.dimensions.setLen last_dense
  data.inverse_mass.setLen last_dense
  data.cached_center.setLen last_dense
  data.cached_half_extents.setLen last_dense
  data.cached_world_axes.setLen last_dense
  data.cached_aabb.setLen last_dense
  data.dense_to_slot.setLen last_dense
  data.aabb_node_idx.setLen last_dense

  data.slot_to_dense[slot] = slot_invalid
  inc data.generation[slot]
  data.free_slots.add slot

  result = true

proc local_pos*(data: InternalData, handle: BodyHandle): F3 =
  result = data.local_pos[data.slot_to_dense[handle.slot]]

proc rot*(data: InternalData, handle: BodyHandle): QF =
  result = data.rot[data.slot_to_dense[handle.slot]]

proc dimensions*(data: InternalData, handle: BodyHandle): F3 =
  result = data.dimensions[data.slot_to_dense[handle.slot]]

proc aabb*(data: InternalData, handle: BodyHandle): FBB =
  result = data.cached_aabb[data.slot_to_dense[handle.slot]]

proc slot_count*(data: InternalData): int =
  data.slot_to_dense.len

proc slot_to_dense_ptr*(data: InternalData): ptr UncheckedArray[int] =
  if data.slot_to_dense.len == 0:
    return nil
  cast[ptr UncheckedArray[int]](unsafeAddr data.slot_to_dense[0])

proc cached_center_ptr*(data: InternalData): ptr UncheckedArray[F3] =
  if data.cached_center.len == 0:
    return nil
  cast[ptr UncheckedArray[F3]](unsafeAddr data.cached_center[0])

proc cached_half_extents_ptr*(data: InternalData): ptr UncheckedArray[F3] =
  if data.cached_half_extents.len == 0:
    return nil
  cast[ptr UncheckedArray[F3]](unsafeAddr data.cached_half_extents[0])

proc cached_world_axes_ptr*(data: InternalData): ptr UncheckedArray[array[3, F3]] =
  if data.cached_world_axes.len == 0:
    return nil
  cast[ptr UncheckedArray[array[3, F3]]](unsafeAddr data.cached_world_axes[0])

proc inverse_mass*(data: InternalData, handle: BodyHandle): float32 =
  result = data.inverse_mass[data.slot_to_dense[handle.slot]]

proc inverse_inertia*(dimensions: F3, inverse_mass: float32): F3 =
  if inverse_mass <= 0'f32:
    return (0'f32, 0'f32, 0'f32)

  let mass = 1'f32 / inverse_mass
  let ix = (1'f32 / 12'f32) * mass * (dimensions.y * dimensions.y + dimensions.z * dimensions.z)
  let iy = (1'f32 / 12'f32) * mass * (dimensions.x * dimensions.x + dimensions.z * dimensions.z)
  let iz = (1'f32 / 12'f32) * mass * (dimensions.x * dimensions.x + dimensions.y * dimensions.y)
  result = (1'f32 / ix, 1'f32 / iy, 1'f32 / iz)

proc update_cuboid_aabb*(data: InternalData, aabb_tree: DynamicAabbTree[BodyHandle], dense_idx: int, displacement: F3) =
  let node_idx = data.aabb_node_idx[dense_idx]
  if node_idx == invalid_node_index:
    return
  discard aabb_tree.update(node_idx, data.cached_aabb[dense_idx], displacement)

proc update_external_data*(internal_data: InternalData, external_data: var ExternalData) =
  with_write_lock(external_data.lock):
    let count = internal_data.local_pos.len
    external_data.pos.setLen count
    external_data.rot = internal_data.rot
    external_data.dimensions = internal_data.dimensions

    external_data.dense_to_slot = internal_data.dense_to_slot
    external_data.slot_to_dense = internal_data.slot_to_dense
    external_data.generation = internal_data.generation

    for i in 0 ..< count:
      let local_pos: D3 = internal_data.local_pos[i]
      external_data.pos[i] = internal_data.initial_pos[i] + local_pos
