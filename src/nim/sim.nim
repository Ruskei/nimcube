import physics_math

type
  Cuboids = ref object
    local_pos: seq[F3]
    island_index: seq[int]

    dense_to_slot: seq[int]
    slot_to_dense: seq[int]
    generation: seq[uint]
    free_slots: seq[int]
  BodyHandle = object
    slot: int
    generation: uint
  C_BodyHandle {.bycopy.} = object
    slot: int32
    generation: uint32
  Island = object
    pos: D3
  World = ref object
    valid: bool
    islands: seq[Island]
    cuboids: Cuboids

converter body_handle_to_c(handle: BodyHandle): C_BodyHandle =
  result.slot = handle.slot.int32
  result.generation = handle.generation.uint32

converter body_handle_from_c(handle: C_BodyHandle): BodyHandle =
  result.slot = handle.slot.int
  result.generation = handle.generation.uint

var worlds: seq[World]

proc init_world(): World =
  result = World(
    valid: true,
    islands: @[],
    cuboids: Cuboids()
  )

const slot_invalid = -1

proc is_valid(cuboids: Cuboids, handle: BodyHandle): bool =
  return
    cuboids.generation[handle.slot] == handle.generation and
    cuboids.slot_to_dense[handle.slot] != slot_invalid

proc create_cuboid(
  world: World,
  pos: D3
): BodyHandle =
  let cuboids = world.cuboids
  var slot: int
  if cuboids.free_slots.len > 0:
    slot = cuboids.free_slots.pop()
  else:
    slot = cuboids.generation.len
    cuboids.generation.add 0
    cuboids.slot_to_dense.add slot_invalid

  let dense = cuboids.local_pos.len
  let island = Island(pos: pos)
  let island_index = world.islands.len
  world.islands.add island
  cuboids.local_pos.add (0'f32, 0'f32, 0'f32)
  cuboids.island_index.add island_index
  cuboids.dense_to_slot.add slot
  cuboids.slot_to_dense[slot] = dense

  result = BodyHandle(slot: slot, generation: cuboids.generation[slot])

proc local_pos(cuboids: Cuboids, handle: BodyHandle): F3 =
  result = cuboids.local_pos[cuboids.slot_to_dense[handle.slot]]

proc island_index(cuboids: Cuboids, handle: BodyHandle): int =
  result = cuboids.island_index[cuboids.slot_to_dense[handle.slot]]

proc global_pos(world: World, handle: BodyHandle): D3 =
  let local_pos: D3 = world.cuboids.local_pos handle
  let island = world.islands[world.cuboids.island_index handle]
  result = island.pos + local_pos

proc c_create_cuboid(
  world_index: cint,
  pos: C_D3,
): C_BodyHandle {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return C_BodyHandle(slot: -1, generation: 0)
  let world = worlds[world_index]
  if not world.valid: return C_BodyHandle(slot: -1, generation: 0)
  result = create_cuboid(world, pos)

proc c_get_global_cuboid_pos(world_index: cint, handle: C_BodyHandle): C_D3 {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return C_D3(x: 0'f64, y: 0'f64, z: 0'f64)
  let world = worlds[world_index]
  if not world.valid: return C_D3(x: 0'f64, y: 0'f64, z: 0'f64)
  result = world.global_pos handle

proc create_world(): cint {.cdecl, exportc, dynlib.} =
  let index = worlds.len
  worlds.add init_world()
  result = index.cint
