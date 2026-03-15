import physics_math

type
  Cuboids = ref object
    local_pos: seq[F3]
    vel: seq[F3]
    ω: seq[F3]
    rot: seq[QF]
    dimensions: seq[F3]
    inverse_mass: seq[float32]
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

    Δt: float32
    acceleration: F3

converter body_handle_to_c(handle: BodyHandle): C_BodyHandle =
  result.slot = handle.slot.int32
  result.generation = handle.generation.uint32

converter body_handle_from_c(handle: C_BodyHandle): BodyHandle =
  result.slot = handle.slot.int
  result.generation = handle.generation.uint

var worlds: seq[World]

proc init_world(Δt: float32, acceleration: F3): World =
  result = World(
    valid: true,
    islands: @[],
    cuboids: Cuboids(),
    Δt: Δt,
    acceleration: acceleration
  )

const slot_invalid = -1

proc is_valid(cuboids: Cuboids, handle: BodyHandle): bool =
  return
    cuboids.generation[handle.slot] == handle.generation and
    cuboids.slot_to_dense[handle.slot] != slot_invalid

proc create_cuboid(
  world: World,
  pos: D3,
  vel: F3,
  ω: F3,
  rot: QF,
  dimensions: F3,
  inverse_mass: float32,
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
  cuboids.vel.add vel
  cuboids.ω.add ω
  cuboids.rot.add normalized(rot)
  cuboids.dimensions.add dimensions
  cuboids.inverse_mass.add inverse_mass
  cuboids.island_index.add island_index
  cuboids.dense_to_slot.add slot
  cuboids.slot_to_dense[slot] = dense

  result = BodyHandle(slot: slot, generation: cuboids.generation[slot])

proc local_pos(cuboids: Cuboids, handle: BodyHandle): F3 =
  result = cuboids.local_pos[cuboids.slot_to_dense[handle.slot]]

proc rot(cuboids: Cuboids, handle: BodyHandle): QF =
  result = cuboids.rot[cuboids.slot_to_dense[handle.slot]]

proc dimensions(cuboids: Cuboids, handle: BodyHandle): F3 =
  result = cuboids.dimensions[cuboids.slot_to_dense[handle.slot]]

proc inverse_mass(cuboids: Cuboids, handle: BodyHandle): float32 =
  result = cuboids.inverse_mass[cuboids.slot_to_dense[handle.slot]]

proc inverse_inertia(dimensions: F3, inverse_mass: float32): F3 =
  if inverse_mass <= 0'f32:
    return (0'f32, 0'f32, 0'f32)

  let mass = 1'f32 / inverse_mass
  let ix = (1'f32 / 12'f32) * mass * (dimensions.y * dimensions.y + dimensions.z * dimensions.z)
  let iy = (1'f32 / 12'f32) * mass * (dimensions.x * dimensions.x + dimensions.z * dimensions.z)
  let iz = (1'f32 / 12'f32) * mass * (dimensions.x * dimensions.x + dimensions.y * dimensions.y)
  result = (1'f32 / ix, 1'f32 / iy, 1'f32 / iz)

proc island_index(cuboids: Cuboids, handle: BodyHandle): int =
  result = cuboids.island_index[cuboids.slot_to_dense[handle.slot]]

proc global_pos(world: World, handle: BodyHandle): D3 =
  let local_pos: D3 = world.cuboids.local_pos handle
  let island = world.islands[world.cuboids.island_index handle]
  result = island.pos + local_pos

proc c_create_cuboid(
  world_index: cint;
  pos_x, pos_y, pos_z: float64;
  vel_x, vel_y, vel_z: float32;
  ω_x, ω_y, ω_z: float32;
  rot_x, rot_y, rot_z, rot_w: float32;
  dimensions_x, dimensions_y, dimensions_z: float32;
  inverse_mass: float32;
): C_BodyHandle {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return C_BodyHandle(slot: -1, generation: 0)
  let world = worlds[world_index]
  if not world.valid: return C_BodyHandle(slot: -1, generation: 0)
  result = create_cuboid(
    world,
    (pos_x, pos_y, pos_z),
    (vel_x, vel_y, vel_z),
    (ω_x, ω_y, ω_z),
    (rot_x, rot_y, rot_z, rot_w),
    (dimensions_x, dimensions_y, dimensions_z),
    inverse_mass,
  )

proc c_get_global_cuboid_pos(world_index: cint, handle: C_BodyHandle): C_D3 {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return C_D3(x: 0'f64, y: 0'f64, z: 0'f64)
  let world = worlds[world_index]
  if not world.valid: return C_D3(x: 0'f64, y: 0'f64, z: 0'f64)
  result = world.global_pos handle

proc c_get_cuboid_rot(world_index: cint, handle: C_BodyHandle): C_QF {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return C_QF(x: 0'f32, y: 0'f32, z: 0'f32, w: 1'f32)
  let world = worlds[world_index]
  if not world.valid: return C_QF(x: 0'f32, y: 0'f32, z: 0'f32, w: 1'f32)
  result = world.cuboids.rot(handle)

proc c_get_cuboid_dimensions(world_index: cint, handle: C_BodyHandle): C_F3 {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return C_F3(x: 0'f32, y: 0'f32, z: 0'f32)
  let world = worlds[world_index]
  if not world.valid: return C_F3(x: 0'f32, y: 0'f32, z: 0'f32)
  result = world.cuboids.dimensions(handle)

proc c_get_cuboid_inverse_mass(world_index: cint, handle: C_BodyHandle): float32 {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return 0'f32
  let world = worlds[world_index]
  if not world.valid: return 0'f32
  result = world.cuboids.inverse_mass(handle)

proc create_world(
  Δt, acceleration_x, acceleration_y, acceleration_z: float32
): cint {.cdecl, exportc, dynlib.} =
  let index = worlds.len
  worlds.add init_world(Δt, (acceleration_x, acceleration_y, acceleration_z))
  result = index.cint

proc tick_world(world_index: cint) {.cdecl, exportc, dynlib.} =
  if world_index >= worlds.len: return
  let world = worlds[world_index]
  if not world.valid: return
  let cuboids = world.cuboids
  let Δt = world.Δt
  let acceleration = world.acceleration

  for i in 0 ..< cuboids.local_pos.len:
    cuboids.vel[i] += acceleration * Δt
    cuboids.local_pos[i] += cuboids.vel[i] * Δt
    let ω = cuboids.ω[i]
    let spin = quat(ω, 0'f32)
    let q = cuboids.rot[i]
    cuboids.rot[i] = normalized(q + (0.5'f32 * Δt) * (spin * q))
