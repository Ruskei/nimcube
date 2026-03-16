import std/atomics

import cuboids
import packed_handle
import physics_math

type
  CommandKind* = enum
    ck_add, ck_remove
  Command* = ref object
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
  Node = ptr NodeObject
  NodeObject = object
    next: Node
    command: Command
  CommandQueue* = ref object
    head: Atomic[Node]

proc add*(queue: CommandQueue, command: Command) =
  let new_node = cast[Node](alloc0(sizeof(NodeObject)))
  new_node.command = command

  var curr_head = queue.head.load()
  while true:
    new_node.next = curr_head
    if queue.head.compareExchange(curr_head, new_node): break

proc get_next(node: var Node, queue: CommandQueue): bool =
  node = queue.head.exchange nil
  result = node != nil

proc process_command_queue*(queue: CommandQueue, data: InternalData) =
  var node = queue.head.exchange nil

  while node != nil:
    let next = node.next
    let command = node.command

    case command.kind
    of ck_add:
      let handle = create_cuboid(
        data = data,
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
      discard data.remove_cuboid(command.handle)

    reset node.command
    dealloc node
    node = next
