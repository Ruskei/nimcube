import std/atomics
import std/tables

import chunk_positions
import cuboids
import dynamic_aabb_tree
import meshing
import packed_handle
import physics_math
import portal

type
  CommandKind* = enum
    ck_add, ck_remove, ck_add_mesh, ck_add_portal, ck_remove_portal
  Command* = object
    case kind*: CommandKind
    of ck_add:
      pos*: D3
      vel*: F3
      ω*: F3
      rot*: QF
      dimensions*: F3
      inverse_mass*: float32
      packed_handle*: ptr PackedHandle
    of ck_remove:
      handle*: BodyHandle
    of ck_add_mesh:
      chunk_x*, chunk_z*: int32
      chunk_binary_data*: ptr ChunkBinaryData
    of ck_add_portal:
      origin_a*: F3
      origin_b*: F3
      quat_a*: QF
      quat_b*: QF
      scale_x*: float32
      scale_y*: float32
      portal_packed_handle*: ptr PackedHandle
    of ck_remove_portal:
      portal_handle*: PortalsHandle
  Node = ptr NodeObject
  NodeObject = object
    next: Node
    command: Command
  CommandQueue* = object
    head: Atomic[Node]

proc init_command_queue*(queue: var CommandQueue) =
  queue.head.store(nil)

proc deinit_command_queue*(queue: var CommandQueue) =
  var node = queue.head.exchange(nil)
  while node != nil:
    let next = node.next
    if node.command.kind == ck_add_mesh:
      deallocShared(node.command.chunk_binary_data)
    deallocShared(node)
    node = next

proc add*(queue: var CommandQueue, command: sink Command) =
  let new_node = createShared(NodeObject)
  new_node.command = command

  var curr_head = queue.head.load()
  while true:
    new_node.next = curr_head
    if queue.head.compareExchange(curr_head, new_node): break

proc process_command_queue*(
  queue: var CommandQueue,
  data: InternalData,
  aabb_tree: DynamicAabbTree[BodyHandle],
  portals: Portals,
  portal_aabb_tree: DynamicAabbTree[PortalsHandle],
  chunk_meshes_by_position: var Table[ChunkPosition, ChunkMesh],
) =
  var node = queue.head.exchange nil

  while node != nil:
    let next = node.next
    var command = node.command

    case command.kind
    of ck_add:
      let handle = create_cuboid(
        data = data,
        aabb_tree = aabb_tree,
        initial_pos = command.pos,
        vel = command.vel,
        ω = command.ω,
        rot = command.rot,
        dimensions = command.dimensions,
        inverse_mass = command.inverse_mass,
      )
      command.packed_handle.slot = handle.slot.int32
      command.packed_handle.generation = handle.generation.int32
    of ck_remove:
      discard data.remove_cuboid(aabb_tree, command.handle)
    of ck_add_mesh:
      let pos = chunk_position(command.chunk_x, command.chunk_z)
      chunk_meshes_by_position[pos] = build_chunk_mesh(command.chunk_x * chunk_width, chunk_mesh_min_y, command.chunk_z * chunk_width, command.chunk_binary_data)
      deallocShared command.chunk_binary_data
    of ck_add_portal:
      let handle = portals.add_portal(
        portal_aabb_tree,
        Portal(
          origin_a: command.origin_a,
          origin_b: command.origin_b,
          quat_a: command.quat_a,
          quat_b: command.quat_b,
          scale_x: command.scale_x,
          scale_y: command.scale_y,
        ),
      )
      command.portal_packed_handle.slot = handle.slot.int32
      command.portal_packed_handle.generation = handle.generation.int32
    of ck_remove_portal:
      if portals.is_valid(command.portal_handle):
        let aabb_ids = portals.aabb_index(command.portal_handle)
        if aabb_ids.a != invalid_node_index:
          discard portal_aabb_tree.remove(aabb_ids.a)
        if aabb_ids.b != invalid_node_index:
          discard portal_aabb_tree.remove(aabb_ids.b)
        discard portals.remove(command.portal_handle)

    deallocShared(node)
    node = next
