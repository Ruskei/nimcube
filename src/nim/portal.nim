import stable_soa
import physics_math
import dynamic_aabb_tree
import rw_lock
import array_util
import packed_handle

type
  ## The "base" portal is a 1x1 portal on the x-y plane with a normal in the +z direction
  WhichPortal* = enum
    wp_a, wp_b
  Portal* = object
    origin_a*: F3
    origin_b*: F3
    quat_a*: QF
    quat_b*: QF
    scale_x*: float32
    scale_y*: float32

const invalid_portal* = Portal(
  origin_a: (0'f32, 0'f32, 0'f32),
  origin_b: (0'f32, 0'f32, 0'f32),
  quat_a: (0'f32, 0'f32, 0'f32, 0'f32),
  quat_b: (0'f32, 0'f32, 0'f32, 0'f32),
  scale_x: -1'f32,
  scale_y: -1'f32,
)

declare_stable_soa_type Portals:
  portal: Portal
  aabb_index: tuple[a: NodeIndex, b: NodeIndex]

type
  ExternalPortalData* = object
    portal*: ptr UncheckedArray[Portal]

    dense_to_slot*: ptr UncheckedArray[int]
    slot_to_dense*: ptr UncheckedArray[int]
    generation*: ptr UncheckedArray[int]

    portal_count*: int
    slot_count*: int
    portal_capacity*: int
    slot_capacity*: int

    lock*: RwLock
  SpecificPortalsHandle* = object
    handle*: PortalsHandle
    which*: WhichPortal

proc init_external_portal_data*(data: var ExternalPortalData) =
  data.portal = nil

  data.dense_to_slot = nil
  data.slot_to_dense = nil
  data.generation = nil

  data.portal_count = 0
  data.slot_count = 0
  data.portal_capacity = 0
  data.slot_capacity = 0

  init_rw_lock data.lock

proc deinit_external_portal_data*(data: var ExternalPortalData) =
  if data.portal != nil:
    deallocShared(data.portal)
    data.portal = nil

  if data.dense_to_slot != nil:
    deallocShared(data.dense_to_slot)
    data.dense_to_slot = nil
  if data.slot_to_dense != nil:
    deallocShared(data.slot_to_dense)
    data.slot_to_dense = nil
  if data.generation != nil:
    deallocShared(data.generation)
    data.generation = nil

  data.portal_count = 0
  data.slot_count = 0
  data.portal_capacity = 0
  data.slot_capacity = 0
  deinit_rw_lock(data.lock)

proc is_valid_no_lock(data: ExternalPortalData, handle: PortalsHandle): bool =
  return
    data.slot_to_dense != nil and
    data.generation != nil and
    handle.slot >= 0 and
    handle.slot < data.slot_count and
    data.generation[handle.slot] == handle.generation and
    data.slot_to_dense[handle.slot] != soa_slot_invalid

proc is_valid*(data: var ExternalPortalData, handle: PortalsHandle): bool =
  with_read_lock(data.lock):
    return is_valid_no_lock(data, handle)

proc portal*(data: var ExternalPortalData, handle: PortalsHandle): Portal =
  with_read_lock(data.lock):
    if not data.is_valid_no_lock(handle): return invalid_portal
    return data.portal[data.slot_to_dense[handle.slot]]

proc update_external_data*(internal_data: Portals, external_data: var ExternalPortalData) =
  with_write_lock(external_data.lock):
    let portal_count = internal_data.portal.len
    let slot_count = internal_data.slot_to_dense.len

    if external_data.portal_capacity < portal_count:
      let new_portal_capacity = grown_capacity(external_data.portal_capacity, portal_count)
      resize_shared_array(external_data.portal, external_data.portal_capacity, new_portal_capacity)
      resize_shared_array(external_data.dense_to_slot, external_data.portal_capacity, new_portal_capacity)
      external_data.portal_capacity = new_portal_capacity

    if external_data.slot_capacity < slot_count:
      let new_slot_capacity = grown_capacity(external_data.slot_capacity, slot_count)
      resize_shared_array(external_data.slot_to_dense, external_data.slot_capacity, new_slot_capacity)
      resize_shared_array(external_data.generation, external_data.slot_capacity, new_slot_capacity)
      external_data.slot_capacity = new_slot_capacity

    external_data.portal_count = portal_count
    external_data.slot_count = slot_count

    for i in 0 ..< portal_count:
      external_data.portal[i] = internal_data.portal[i]
      external_data.dense_to_slot[i] = internal_data.dense_to_slot[i]

    for i in 0 ..< slot_count:
      external_data.slot_to_dense[i] = internal_data.slot_to_dense[i]
      external_data.generation[i] = internal_data.generation[i]

converter from_packed*(handle: PackedHandle): PortalsHandle =
  result.slot = handle.slot.int
  result.generation = handle.generation.int

proc aabb*(portal: Portal): tuple[a: FBB, b: FBB] =
  let half_extents: F3 = (portal.scale_x * 0.5'f32, portal.scale_y * 0.5'f32, 0'f32)
  let corners = [
    (x: -half_extents.x, y: -half_extents.y, z: 0'f32),
    (x: -half_extents.x, y:  half_extents.y, z: 0'f32),
    (x:  half_extents.x, y: -half_extents.y, z: 0'f32),
    (x:  half_extents.x, y:  half_extents.y, z: 0'f32),
  ]
  let quat_a = normalized(portal.quat_a)
  let quat_b = normalized(portal.quat_b)

  let first_corner_a = portal.origin_a + rotate_vector(quat_a, corners[0])
  let first_corner_b = portal.origin_b + rotate_vector(quat_b, corners[0])
  result.a.min = first_corner_a
  result.a.max = first_corner_a
  result.b.min = first_corner_b
  result.b.max = first_corner_b

  for corner in corners:
    let corner_a = portal.origin_a + rotate_vector(quat_a, corner)
    if corner_a.x < result.a.min.x: result.a.min.x = corner_a.x
    if corner_a.y < result.a.min.y: result.a.min.y = corner_a.y
    if corner_a.z < result.a.min.z: result.a.min.z = corner_a.z
    if corner_a.x > result.a.max.x: result.a.max.x = corner_a.x
    if corner_a.y > result.a.max.y: result.a.max.y = corner_a.y
    if corner_a.z > result.a.max.z: result.a.max.z = corner_a.z

    let corner_b = portal.origin_b + rotate_vector(quat_b, corner)
    if corner_b.x < result.b.min.x: result.b.min.x = corner_b.x
    if corner_b.y < result.b.min.y: result.b.min.y = corner_b.y
    if corner_b.z < result.b.min.z: result.b.min.z = corner_b.z
    if corner_b.x > result.b.max.x: result.b.max.x = corner_b.x
    if corner_b.y > result.b.max.y: result.b.max.y = corner_b.y
    if corner_b.z > result.b.max.z: result.b.max.z = corner_b.z

proc add_portal*(
  portals: Portals,
  portal_aabb_tree: DynamicAabbTree[SpecificPortalsHandle],
  portal: Portal,
): PortalsHandle =
  result = portals.add(
    portal = Portal(
      origin_a: portal.origin_a,
      origin_b: portal.origin_b,
      quat_a: normalized(portal.quat_a),
      quat_b: normalized(portal.quat_b),
      scale_x: portal.scale_x,
      scale_y: portal.scale_y,
    ),
    aabb_index = (invalid_node_index, invalid_node_index),
  )

  let dense = portals.slot_to_dense[result.slot]
  let (bb_a, bb_b) = portals.portal(result).aabb()
  let bb_index_a = portal_aabb_tree.insert(bb_a, SpecificPortalsHandle(handle: result, which: wp_a))
  let bb_index_b = portal_aabb_tree.insert(bb_b, SpecificPortalsHandle(handle: result, which: wp_b))
  portals.aabb_index[dense] = (bb_index_a, bb_index_b)
