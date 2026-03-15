## The problem this solves is having fast data storage for cuboids while needing fast read access from other threads
## This is done by having a main mutable buffer that the physics engine uses while also having a second read-only
## buffer for external threads. This is managed with a read-write lock, which is faster with double-buffering since
## we only block for the duration it takes to update the published data.

import physics_math
import rw_lock
import packed_handle
import island

type
  InternalData* = ref object
    local_pos*: seq[F3]
    vel*: seq[F3]
    ω*: seq[F3]
    rot*: seq[QF]
    dimensions*: seq[F3]
    inverse_mass*: seq[float32]
    island_index*: seq[int]

    dense_to_slot: seq[int]
    slot_to_dense: seq[int]
    generation: seq[uint]
    free_slots: seq[int]
  ExternalData* = ref object
    pos*: seq[D3]
    rot*: seq[QF]
    dimensions*: seq[F3]

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

proc create_cuboid*(
  data: InternalData,
  vel: F3,
  ω: F3,
  rot: QF,
  dimensions: F3,
  inverse_mass: float32,
  island_index: int,
): BodyHandle =
  var slot: int
  if data.free_slots.len > 0:
    slot = data.free_slots.pop()
  else:
    slot = data.generation.len
    data.generation.add 0
    data.slot_to_dense.add slot_invalid

  let dense = data.local_pos.len
  data.local_pos.add (0'f32, 0'f32, 0'f32)
  data.vel.add vel
  data.ω.add ω
  data.rot.add normalized(rot)
  data.dimensions.add dimensions
  data.inverse_mass.add inverse_mass
  data.island_index.add island_index
  data.dense_to_slot.add slot
  data.slot_to_dense[slot] = dense

  result = BodyHandle(slot: slot, generation: data.generation[slot])

proc local_pos*(data: InternalData, handle: BodyHandle): F3 =
  result = data.local_pos[data.slot_to_dense[handle.slot]]

proc rot*(data: InternalData, handle: BodyHandle): QF =
  result = data.rot[data.slot_to_dense[handle.slot]]

proc dimensions*(data: InternalData, handle: BodyHandle): F3 =
  result = data.dimensions[data.slot_to_dense[handle.slot]]

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

proc island_index*(data: InternalData, handle: BodyHandle): int =
  result = data.island_index[data.slot_to_dense[handle.slot]]

proc update_external_data*(internal_data: InternalData, external_data: var ExternalData, islands: seq[Island]) =
  with_write_lock(external_data.lock):
    let count = internal_data.local_pos.len
    external_data.pos.setLen count
    external_data.rot = internal_data.rot
    external_data.dimensions = internal_data.dimensions

    external_data.slot_to_dense = internal_data.slot_to_dense
    external_data.generation = internal_data.generation

    for i in 0 ..< count:
      let local_pos: D3 = internal_data.local_pos[i]
      let island_index = internal_data.island_index[i]
      external_data.pos[i] = islands[island_index].pos + local_pos
