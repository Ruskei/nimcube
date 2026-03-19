## The problem this solves is having fast data storage for cuboids while needing fast read access from other threads
## This is done by having a main mutable buffer that the physics engine uses while also having a second read-only
## buffer for external threads. This is managed with a read-write lock, which is faster with double-buffering since
## we only block for the duration it takes to update the published data.

import physics_math
import rw_lock
import packed_handle
import dynamic_aabb_tree
import portal
import array_util
import stable_soa

declare_stable_soa_type InternalData:
  initial_pos: D3
  local_pos: F3
  vel: F3
  ω: F3
  rot: QF
  dimensions: F3
  inverse_mass: float32
  cached_center: F3
  cached_half_extents: F3
  cached_world_axes: array[3, F3]
  cached_inverse_inertia_diag: F3
  cached_aabb: FBB
  aabb_node_idx: NodeIndex

type
  ExternalData* = object
    pos*: ptr UncheckedArray[D3]
    rot*: ptr UncheckedArray[QF]
    dimensions*: ptr UncheckedArray[F3]

    dense_to_slot*: ptr UncheckedArray[int]
    slot_to_dense*: ptr UncheckedArray[int]
    generation*: ptr UncheckedArray[uint]

    body_count*: int
    slot_count*: int
    body_capacity: int
    slot_capacity: int

    lock*: RwLock
  BodyHandle* = object
    slot*: int
    generation*: uint

converter internal_to_body*(handle: InternalDataHandle): BodyHandle =
  BodyHandle(slot: handle.slot, generation: handle.generation.uint)

converter body_to_internal*(handle: BodyHandle): InternalDataHandle =
  InternalDataHandle(slot: handle.slot, generation: handle.generation.int)

const slot_invalid = -1

proc init_external_data*(data: var ExternalData) =
  data.pos = nil
  data.rot = nil
  data.dimensions = nil
  data.dense_to_slot = nil
  data.slot_to_dense = nil
  data.generation = nil
  data.body_count = 0
  data.slot_count = 0
  data.body_capacity = 0
  data.slot_capacity = 0
  init_rw_lock data.lock

proc deinit_external_data*(data: var ExternalData) =
  if data.pos != nil:
    deallocShared(data.pos)
    data.pos = nil
  if data.rot != nil:
    deallocShared(data.rot)
    data.rot = nil
  if data.dimensions != nil:
    deallocShared(data.dimensions)
    data.dimensions = nil
  if data.dense_to_slot != nil:
    deallocShared(data.dense_to_slot)
    data.dense_to_slot = nil
  if data.slot_to_dense != nil:
    deallocShared(data.slot_to_dense)
    data.slot_to_dense = nil
  if data.generation != nil:
    deallocShared(data.generation)
    data.generation = nil
  data.body_count = 0
  data.slot_count = 0
  data.body_capacity = 0
  data.slot_capacity = 0
  deinit_rw_lock(data.lock)

proc update_external_data*(internal_data: InternalData, external_data: var ExternalData) =
  with_write_lock(external_data.lock):
    let body_count = internal_data.local_pos.len
    let slot_count = internal_data.slot_to_dense.len

    if external_data.body_capacity < body_count:
      let new_body_capacity = grown_capacity(external_data.body_capacity, body_count)
      resize_shared_array(external_data.pos, external_data.body_capacity, new_body_capacity)
      resize_shared_array(external_data.rot, external_data.body_capacity, new_body_capacity)
      resize_shared_array(external_data.dimensions, external_data.body_capacity, new_body_capacity)
      resize_shared_array(external_data.dense_to_slot, external_data.body_capacity, new_body_capacity)
      external_data.body_capacity = new_body_capacity

    if external_data.slot_capacity < slot_count:
      let new_slot_capacity = grown_capacity(external_data.slot_capacity, slot_count)
      resize_shared_array(external_data.slot_to_dense, external_data.slot_capacity, new_slot_capacity)
      resize_shared_array(external_data.generation, external_data.slot_capacity, new_slot_capacity)
      external_data.slot_capacity = new_slot_capacity

    external_data.body_count = body_count
    external_data.slot_count = slot_count

    for i in 0 ..< body_count:
      let local_pos: D3 = internal_data.local_pos[i]
      external_data.pos[i] = internal_data.initial_pos[i] + local_pos
      external_data.rot[i] = internal_data.rot[i]
      external_data.dimensions[i] = internal_data.dimensions[i]
      external_data.dense_to_slot[i] = internal_data.dense_to_slot[i]

    for i in 0 ..< slot_count:
      external_data.slot_to_dense[i] = internal_data.slot_to_dense[i]
      external_data.generation[i] = internal_data.generation[i].uint

converter from_packed*(handle: PackedHandle): BodyHandle =
  result.slot = handle.slot
  result.generation = handle.generation.uint

proc is_valid_no_lock*(data: ExternalData, handle: BodyHandle): bool =
  return
    data.slot_to_dense != nil and
    data.generation != nil and
    handle.slot >= 0 and
    handle.slot < data.slot_count and
    data.generation[handle.slot] == handle.generation and
    data.slot_to_dense[handle.slot] != slot_invalid

proc is_valid*(data: var ExternalData, handle: BodyHandle): bool =
  with_read_lock(data.lock):
    return is_valid_no_lock(data, handle)

proc dense_idx_no_lock*(data: ExternalData, handle: BodyHandle): int =
  if not data.is_valid_no_lock(handle):
    return slot_invalid
  data.slot_to_dense[handle.slot]

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

proc inverse_inertia*(dimensions: F3, inverse_mass: float32): F3

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
  result = data.add(
    initial_pos = initial_pos,
    local_pos = default(F3),
    vel = vel,
    ω = ω,
    rot = normalized(rot),
    dimensions = dimensions,
    inverse_mass = inverse_mass,
    cached_center = default(F3),
    cached_half_extents = default(F3),
    cached_world_axes = default(array[3, F3]),
    cached_inverse_inertia_diag = inverse_inertia(dimensions, inverse_mass),
    cached_aabb = default(FBB),
    aabb_node_idx = invalid_node_index
  )

  let dense = data.slot_to_dense[result.slot]
  data.update_body_collision_cache dense
  data.aabb_node_idx[dense] = aabb_tree.insert(data.cached_aabb[dense], result)

proc remove_cuboid*(data: InternalData, aabb_tree: DynamicAabbTree[BodyHandle], handle: BodyHandle): bool =
  if not data.is_valid(handle):
    return false

  let node_idx = data.aabb_node_idx handle
  if node_idx != invalid_node_index:
    discard aabb_tree.remove(node_idx)

  result = data.remove handle

proc aabb*(data: InternalData, handle: BodyHandle): FBB =
  data.cached_aabb handle

proc body_handle_at_dense*(data: InternalData, dense_idx: int): BodyHandle =
  let slot = data.dense_to_slot[dense_idx]
  BodyHandle(slot: slot, generation: data.generation[slot].uint)

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

proc cached_aabb_ptr*(data: InternalData): ptr UncheckedArray[FBB] =
  if data.cached_aabb.len == 0:
    return nil
  cast[ptr UncheckedArray[FBB]](unsafeAddr data.cached_aabb[0])

proc cached_inverse_inertia_diag_ptr*(data: InternalData): ptr UncheckedArray[F3] =
  if data.cached_inverse_inertia_diag.len == 0:
    return nil
  cast[ptr UncheckedArray[F3]](unsafeAddr data.cached_inverse_inertia_diag[0])

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

proc apply_inverse_inertia_world*(data: InternalData, dense_idx: int, v: F3): F3 =
  let axes = data.cached_world_axes[dense_idx]
  let inverse_inertia_diag = data.cached_inverse_inertia_diag[dense_idx]
  result =
    axes[0] * (inverse_inertia_diag.x * (axes[0] ∙ v)) +
    axes[1] * (inverse_inertia_diag.y * (axes[1] ∙ v)) +
    axes[2] * (inverse_inertia_diag.z * (axes[2] ∙ v))

proc update_cuboid_aabb*(data: InternalData, aabb_tree: DynamicAabbTree[BodyHandle], dense_idx: int, displacement: F3) =
  let node_idx = data.aabb_node_idx[dense_idx]
  if node_idx == invalid_node_index:
    return
  discard aabb_tree.update(node_idx, data.cached_aabb[dense_idx], displacement)
