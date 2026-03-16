import std/math
import std/tables

import command_queue
import cuboids
import meshing
import packed_handle
import physics_math
import sim

proc approx_equal(a, b: float32): bool =
  abs(a - b) <= 1.0e-4'f32

proc same_handle(a, b: BodyHandle): bool =
  a.slot == b.slot and a.generation == b.generation

proc make_bb(
  min_x, min_y, min_z,
  max_x, max_y, max_z: float32,
): FBB =
  result.min = (min_x, min_y, min_z)
  result.max = (max_x, max_y, max_z)

proc fresh_world(dt = 0.1'f32, acceleration: F3 = (0'f32, 0'f32, 0'f32)): World =
  deinit_worlds()
  worlds.set_len(0)
  worlds.add init_world(dt, acceleration)
  worlds[0]

proc set_solid_voxel(data: var ChunkBinaryData, x, y, z: int) =
  let index = y * chunk_width * chunk_width + z * chunk_width + x
  let long_index = index div 64
  let bit_index = index mod 64
  data[long_index] = data[long_index] or (1'u64 shl bit_index)

proc make_chunk_binary_data(voxels: openArray[(int, int, int)]): ChunkBinaryData =
  for voxel in voxels:
    result.set_solid_voxel(voxel[0], voxel[1], voxel[2])

proc voxel_world_bb(chunk_pos: ChunkPosition, x, y, z: int): FBB =
  let origin = (
    x: (chunk_pos.x * chunk_width.int32).float32,
    y: chunk_mesh_min_y.float32,
    z: (chunk_pos.z * chunk_width.int32).float32,
  )
  result.min = origin + (x.float32, y.float32, z.float32)
  result.max = origin + ((x + 1).float32, (y + 1).float32, (z + 1).float32)

proc bb_center(bb: FBB): D3 =
  (
    x: ((bb.min.x + bb.max.x) * 0.5'f32).float64,
    y: ((bb.min.y + bb.max.y) * 0.5'f32).float64,
    z: ((bb.min.z + bb.max.z) * 0.5'f32).float64,
  )

proc enqueue_add(
  world: World,
  packed_handle: ptr PackedHandle,
  pos: D3,
  vel, ω, dimensions: F3,
  rot: QF = quat_identity(float32),
  inverse_mass = 1'f32,
) =
  world.command_queue.add Command(
    kind: ck_add,
    pos: pos,
    vel: vel,
    ω: ω,
    rot: rot,
    dimensions: dimensions,
    inverse_mass: inverse_mass,
    packed_handle: packed_handle,
  )

proc test_world_initialization() =
  let world = fresh_world()
  doAssert world.internal_data.local_pos.len == 0
  doAssert world.aabb_tree_leaf_count() == 0
  doAssert world.chunk_meshes_by_position.len == 0
  world.validate_aabb_tree()

proc test_add_chunk_mesh_inserts_into_world_table() =
  let world = fresh_world()
  var chunk_binary_data = make_chunk_binary_data(@[(1, 0, 1)])
  let chunk_pos = chunk_position(0, 0)

  doAssert world.add_chunk_mesh(chunk_pos, addr chunk_binary_data)
  doAssert world.chunk_meshes_by_position.len == 1
  doAssert world.chunk_meshes_by_position.hasKey(chunk_pos)

  let mesh = world.chunk_meshes_by_position[chunk_pos]
  doAssert mesh.origin == (0'i32, chunk_mesh_min_y.int32, 0'i32)
  doAssert mesh.bbs.len > 0
  doAssert mesh.aabb_tree.leaf_count == mesh.bbs.len

proc test_add_chunk_mesh_replaces_existing_position() =
  let world = fresh_world()
  let chunk_pos = chunk_position(0, 0)
  var first_chunk_binary_data = make_chunk_binary_data(@[(1, 0, 1)])
  var second_chunk_binary_data = make_chunk_binary_data(@[(1, 0, 1), (3, 0, 1)])

  doAssert world.add_chunk_mesh(chunk_pos, addr first_chunk_binary_data)
  let first_mesh = world.chunk_meshes_by_position[chunk_pos]
  doAssert first_mesh.bbs.len == 1

  doAssert world.add_chunk_mesh(chunk_pos, addr second_chunk_binary_data)
  doAssert world.chunk_meshes_by_position.len == 1

  let second_mesh = world.chunk_meshes_by_position[chunk_pos]
  doAssert second_mesh.bbs.len == 2
  doAssert second_mesh.aabb_tree.leaf_count == 2

proc test_add_chunk_mesh_stores_multiple_positions() =
  let world = fresh_world()
  var first_chunk_binary_data = make_chunk_binary_data(@[(1, 0, 1)])
  var second_chunk_binary_data = make_chunk_binary_data(@[(2, 0, 2)])
  let first_pos = chunk_position(0, 0)
  let second_pos = chunk_position(2, -1)

  doAssert world.add_chunk_mesh(first_pos, addr first_chunk_binary_data)
  doAssert world.add_chunk_mesh(second_pos, addr second_chunk_binary_data)
  doAssert world.chunk_meshes_by_position.len == 2
  doAssert world.chunk_meshes_by_position.hasKey(first_pos)
  doAssert world.chunk_meshes_by_position.hasKey(second_pos)

proc test_chunk_position_from_negative_world_coords() =
  let world = fresh_world()
  let chunk_pos = chunk_position_from_world_xz(-1, -1)
  var chunk_binary_data = make_chunk_binary_data(@[(1, 0, 1)])

  doAssert chunk_pos == chunk_position(-1, -1)
  doAssert world.add_chunk_mesh(chunk_pos, addr chunk_binary_data)
  doAssert world.chunk_meshes_by_position.hasKey(chunk_pos)
  doAssert world.chunk_meshes_by_position[chunk_pos].origin == (
    -chunk_width.int32,
    chunk_mesh_min_y.int32,
    -chunk_width.int32,
  )

proc test_add_command_inserts_into_tree() =
  let world = fresh_world()
  var packed = PackedHandle(slot: -1, generation: 0)

  world.enqueue_add(
    packed_handle = addr packed,
    pos = (0'f64, 0'f64, 0'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (2'f32, 2'f32, 2'f32),
  )

  tick_world(0)

  let handle: BodyHandle = packed
  doAssert world.internal_data.is_valid(handle)
  doAssert world.aabb_tree_leaf_count() == 1
  world.validate_aabb_tree()

  let hits = world.query_aabb_tree(make_bb(-1.5'f32, -1.5'f32, -1.5'f32, 1.5'f32, 1.5'f32, 1.5'f32))
  doAssert hits.len == 1
  doAssert same_handle(hits[0], handle)

proc test_remove_command_removes_tree_node() =
  let world = fresh_world()
  var packed = PackedHandle(slot: -1, generation: 0)

  world.enqueue_add(
    packed_handle = addr packed,
    pos = (0'f64, 0'f64, 0'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (2'f32, 2'f32, 2'f32),
  )
  tick_world(0)

  let handle: BodyHandle = packed
  world.command_queue.add Command(kind: ck_remove, handle: handle)
  tick_world(0)

  doAssert not world.internal_data.is_valid(handle)
  doAssert world.aabb_tree_leaf_count() == 0
  world.validate_aabb_tree()
  doAssert world.query_aabb_tree(make_bb(-1.5'f32, -1.5'f32, -1.5'f32, 1.5'f32, 1.5'f32, 1.5'f32)).len == 0

proc test_dense_swap_keeps_tree_updates_valid() =
  let world = fresh_world()
  var packed_a = PackedHandle(slot: -1, generation: 0)
  var packed_b = PackedHandle(slot: -1, generation: 0)

  world.enqueue_add(
    packed_handle = addr packed_a,
    pos = (0'f64, 0'f64, 0'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (2'f32, 2'f32, 2'f32),
  )
  world.enqueue_add(
    packed_handle = addr packed_b,
    pos = (10'f64, 0'f64, 0'f64),
    vel = (1'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (2'f32, 2'f32, 2'f32),
  )
  tick_world(0)

  let handle_a: BodyHandle = packed_a
  let handle_b: BodyHandle = packed_b
  world.command_queue.add Command(kind: ck_remove, handle: handle_a)
  tick_world(0)
  tick_world(0)

  doAssert not world.internal_data.is_valid(handle_a)
  doAssert world.internal_data.is_valid(handle_b)
  doAssert world.aabb_tree_leaf_count() == 1
  world.validate_aabb_tree()

  let hits = world.query_aabb_tree(make_bb(8'f32, -2'f32, -2'f32, 12'f32, 2'f32, 2'f32))
  doAssert hits.len == 1
  doAssert same_handle(hits[0], handle_b)

proc test_per_tick_motion_updates_tree() =
  let world = fresh_world(dt = 1'f32)
  var packed = PackedHandle(slot: -1, generation: 0)

  world.enqueue_add(
    packed_handle = addr packed,
    pos = (0'f64, 0'f64, 0'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (2'f32, 2'f32, 2'f32),
  )
  tick_world(0)

  let handle: BodyHandle = packed
  doAssert world.query_aabb_tree(make_bb(-1.5'f32, -1.5'f32, -1.5'f32, 1.5'f32, 1.5'f32, 1.5'f32)).len == 1

  world.internal_data.vel[0] = (5'f32, 0'f32, 0'f32)
  tick_world(0)
  world.validate_aabb_tree()

  doAssert world.query_aabb_tree(make_bb(-1.5'f32, -1.5'f32, -1.5'f32, 1.5'f32, 1.5'f32, 1.5'f32)).len == 0
  let hits = world.query_aabb_tree(make_bb(3.5'f32, -1.5'f32, -1.5'f32, 6.5'f32, 1.5'f32, 1.5'f32))
  doAssert hits.len == 1
  doAssert same_handle(hits[0], handle)
  doAssert world.aabb_tree_leaf_count() == 1

proc test_rotation_changes_broadphase_bounds() =
  let world = fresh_world(dt = 0.2'f32)
  var packed = PackedHandle(slot: -1, generation: 0)

  world.enqueue_add(
    packed_handle = addr packed,
    pos = (0'f64, 0'f64, 0'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (4'f32, 1'f32, 1'f32),
  )
  tick_world(0)

  let handle: BodyHandle = packed
  let probe = make_bb(-0.25'f32, 1.6'f32, -0.75'f32, 0.25'f32, 1.8'f32, 0.75'f32)
  doAssert world.query_aabb_tree(probe).len == 0

  world.internal_data.ω[0] = (0'f32, 0'f32, 2'f32)
  for _ in 0 ..< 4:
    tick_world(0)

  world.validate_aabb_tree()
  let hits = world.query_aabb_tree(probe)
  doAssert hits.len == 1
  doAssert same_handle(hits[0], handle)

proc test_narrowphase_dispatch_smoke() =
  let world = fresh_world()
  var packed_a = PackedHandle(slot: -1, generation: 0)
  var packed_b = PackedHandle(slot: -1, generation: 0)

  world.enqueue_add(
    packed_handle = addr packed_a,
    pos = (0'f64, 0'f64, 0'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (2'f32, 2'f32, 2'f32),
  )
  world.enqueue_add(
    packed_handle = addr packed_b,
    pos = (0.5'f64, 0'f64, 0'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (2'f32, 2'f32, 2'f32),
  )

  tick_world(0)

  doAssert world.valid
  doAssert world.aabb_tree_leaf_count() == 2
  doAssert world.a2a_narrowphase_manifold_count() >= 1
  doAssert world.get_a2a_narrowphase_manifold(0).contact_count > 0

proc test_tick_world_generates_a2s_contacts_from_chunk_mesh() =
  let world = fresh_world()
  let chunk_pos = chunk_position(0, 0)
  var chunk_binary_data = make_chunk_binary_data(@[(1, 0, 1)])
  var packed = PackedHandle(slot: -1, generation: 0)

  doAssert world.add_chunk_mesh(chunk_pos, addr chunk_binary_data)
  let static_bb = voxel_world_bb(chunk_pos, 1, 0, 1)
  world.enqueue_add(
    packed_handle = addr packed,
    pos = bb_center(static_bb),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (1'f32, 1'f32, 1'f32),
  )

  tick_world(0)

  let handle: BodyHandle = packed
  doAssert world.a2s_narrowphase_manifold_count() >= 1
  let manifold = world.get_a2s_narrowphase_manifold(0)
  doAssert same_handle(manifold.body_a, handle)
  doAssert manifold.contact_count > 0

proc test_tick_world_environment_broadphase_uses_chunk_local_tree_and_world_space_hit() =
  let world = fresh_world()
  let chunk_pos = chunk_position(2, -1)
  var chunk_binary_data = make_chunk_binary_data(@[(3, 0, 4)])
  var packed = PackedHandle(slot: -1, generation: 0)

  doAssert world.add_chunk_mesh(chunk_pos, addr chunk_binary_data)
  let static_bb = voxel_world_bb(chunk_pos, 3, 0, 4)
  world.enqueue_add(
    packed_handle = addr packed,
    pos = bb_center(static_bb),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (1'f32, 1'f32, 1'f32),
  )

  tick_world(0)

  doAssert world.a2s_narrowphase_manifold_count() >= 1
  let manifold = world.get_a2s_narrowphase_manifold(0)
  doAssert manifold.contact_count > 0
  for idx in 0 ..< manifold.contact_count.int:
    let contact = manifold.contact_points[idx]
    doAssert contact.position.x >= static_bb.min.x - 1.0001'f32
    doAssert contact.position.x <= static_bb.max.x + 1.0001'f32
    doAssert contact.position.y >= static_bb.min.y - 1.0001'f32
    doAssert contact.position.y <= static_bb.max.y + 1.0001'f32
    doAssert contact.position.z >= static_bb.min.z - 1.0001'f32
    doAssert contact.position.z <= static_bb.max.z + 1.0001'f32

proc test_tick_world_no_a2s_when_no_chunk_mesh_overlaps() =
  let world = fresh_world()
  let chunk_pos = chunk_position(5, 5)
  var chunk_binary_data = make_chunk_binary_data(@[(1, 0, 1)])
  var packed = PackedHandle(slot: -1, generation: 0)

  doAssert world.add_chunk_mesh(chunk_pos, addr chunk_binary_data)
  world.enqueue_add(
    packed_handle = addr packed,
    pos = (1.5'f64, -63.5'f64, 1.5'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (1'f32, 1'f32, 1'f32),
  )

  tick_world(0)

  doAssert world.a2s_narrowphase_manifold_count() == 0

proc test_tick_world_negative_chunk_coordinates_for_a2s() =
  let world = fresh_world()
  let chunk_pos = chunk_position(-1, -2)
  var chunk_binary_data = make_chunk_binary_data(@[(2, 0, 3)])
  var packed = PackedHandle(slot: -1, generation: 0)

  doAssert world.add_chunk_mesh(chunk_pos, addr chunk_binary_data)
  let static_bb = voxel_world_bb(chunk_pos, 2, 0, 3)
  world.enqueue_add(
    packed_handle = addr packed,
    pos = bb_center(static_bb),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (1'f32, 1'f32, 1'f32),
  )

  tick_world(0)

  let handle: BodyHandle = packed
  doAssert world.a2s_narrowphase_manifold_count() >= 1
  doAssert same_handle(world.get_a2s_narrowphase_manifold(0).body_a, handle)

proc test_chunk_mesh_contact_solves_velocity() =
  let world = fresh_world(dt = 1'f32 / 60'f32, acceleration = (0'f32, -9.8'f32, 0'f32))
  let chunk_pos = chunk_position(0, 0)
  var chunk_binary_data = make_chunk_binary_data(@[(1, 0, 1)])
  var packed = PackedHandle(slot: -1, generation: 0)

  doAssert world.add_chunk_mesh(chunk_pos, addr chunk_binary_data)
  world.enqueue_add(
    packed_handle = addr packed,
    pos = (1.5'f64, -62.6'f64, 1.5'f64),
    vel = (0'f32, -0.5'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (1'f32, 1'f32, 1'f32),
  )

  tick_world(0)

  let handle: BodyHandle = packed
  doAssert world.a2s_narrowphase_manifold_count() >= 1
  doAssert same_handle(world.get_a2s_narrowphase_manifold(0).body_a, handle)
  doAssert world.internal_data.vel[0].y > -0.663334'f32

proc test_floor_contact_solves_velocity() =
  let world = fresh_world(dt = 1'f32 / 60'f32, acceleration = (0'f32, -9.8'f32, 0'f32))
  var floor_packed = PackedHandle(slot: -1, generation: 0)
  var box_packed = PackedHandle(slot: -1, generation: 0)

  world.enqueue_add(
    packed_handle = addr floor_packed,
    pos = (0'f64, -1'f64, 0'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (6'f32, 2'f32, 6'f32),
    inverse_mass = 0'f32,
  )
  world.enqueue_add(
    packed_handle = addr box_packed,
    pos = (0'f64, 0.49'f64, 0'f64),
    vel = (0'f32, -0.5'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (1'f32, 1'f32, 1'f32),
  )

  tick_world(0)

  doAssert world.internal_data.vel[1].y > -0.663334'f32

proc test_head_on_collision_reduces_relative_velocity() =
  let world = fresh_world(dt = 1'f32 / 60'f32)
  var packed_a = PackedHandle(slot: -1, generation: 0)
  var packed_b = PackedHandle(slot: -1, generation: 0)

  world.enqueue_add(
    packed_handle = addr packed_a,
    pos = (0'f64, 0'f64, 0'f64),
    vel = (1'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (2'f32, 2'f32, 2'f32),
  )
  world.enqueue_add(
    packed_handle = addr packed_b,
    pos = (1.98'f64, 0'f64, 0'f64),
    vel = (-1'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (2'f32, 2'f32, 2'f32),
  )

  tick_world(0)

  doAssert abs(world.internal_data.vel[0].x - world.internal_data.vel[1].x) < 0.5'f32
  doAssert abs(world.internal_data.vel[0].x + world.internal_data.vel[1].x) < 1.0e-3'f32

proc test_rotate_vector() =
  let identity = quat_identity(float32)
  let v0 = rotate_vector(identity, (1'f32, 2'f32, 3'f32))
  doAssert approx_equal(v0.x, 1'f32)
  doAssert approx_equal(v0.y, 2'f32)
  doAssert approx_equal(v0.z, 3'f32)

  let flip_x: QF = (1'f32, 0'f32, 0'f32, 0'f32)
  let v1 = rotate_vector(flip_x, (0'f32, 1'f32, 2'f32))
  doAssert approx_equal(v1.x, 0'f32)
  doAssert approx_equal(v1.y, -1'f32)
  doAssert approx_equal(v1.z, -2'f32)

  let s = sqrt(0.5'f32)
  let quarter_turn_z: QF = normalized((0'f32, 0'f32, s, s))
  let v2 = rotate_vector(quarter_turn_z, (1'f32, 0'f32, 0'f32))
  doAssert approx_equal(v2.x, 0'f32)
  doAssert approx_equal(v2.y, 1'f32)
  doAssert approx_equal(v2.z, 0'f32)

when is_main_module:
  test_world_initialization()
  test_add_chunk_mesh_inserts_into_world_table()
  test_add_chunk_mesh_replaces_existing_position()
  test_add_chunk_mesh_stores_multiple_positions()
  test_chunk_position_from_negative_world_coords()
  test_add_command_inserts_into_tree()
  test_remove_command_removes_tree_node()
  test_dense_swap_keeps_tree_updates_valid()
  test_per_tick_motion_updates_tree()
  test_rotation_changes_broadphase_bounds()
  test_narrowphase_dispatch_smoke()
  test_tick_world_generates_a2s_contacts_from_chunk_mesh()
  test_tick_world_environment_broadphase_uses_chunk_local_tree_and_world_space_hit()
  test_tick_world_no_a2s_when_no_chunk_mesh_overlaps()
  test_tick_world_negative_chunk_coordinates_for_a2s()
  test_chunk_mesh_contact_solves_velocity()
  test_floor_contact_solves_velocity()
  test_head_on_collision_reduces_relative_velocity()
  test_rotate_vector()
  deinit_worlds()
  echo "sim aabb integration tests passed"
