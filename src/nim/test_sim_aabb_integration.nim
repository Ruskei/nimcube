import std/math

import command_queue
import cuboids
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
  world.validate_aabb_tree()

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
  doAssert world.narrowphase_manifold_count() >= 1
  doAssert world.get_narrowphase_manifold(0).contact_count > 0

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
  test_add_command_inserts_into_tree()
  test_remove_command_removes_tree_node()
  test_dense_swap_keeps_tree_updates_valid()
  test_per_tick_motion_updates_tree()
  test_rotation_changes_broadphase_bounds()
  test_narrowphase_dispatch_smoke()
  test_rotate_vector()
  deinit_worlds()
  echo "sim aabb integration tests passed"
