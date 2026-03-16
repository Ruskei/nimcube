import std/math

import dynamic_aabb_tree
import command_queue
import narrowphase
import cuboids
import packed_handle
import physics_math
import sim

type
  PoolFixture = object
    data: InternalData
    tree: DynamicAabbTree[BodyHandle]
    pool: NarrowphasePool

proc make_handle(idx: int): BodyHandle =
  BodyHandle(slot: idx, generation: idx.uint)

proc make_a2a_pair(idx: int): A2aBroadphasePair =
  (a: make_handle(idx * 2), b: make_handle(idx * 2 + 1))

proc range_len(r: Slice[int]): int =
  if r.b < r.a:
    return 0
  r.b - r.a + 1

proc approx_equal(a, b: float32): bool =
  abs(a - b) <= 1.0e-4'f32

proc approx_vec_equal(a, b: F3): bool =
  approx_equal(a.x, b.x) and approx_equal(a.y, b.y) and approx_equal(a.z, b.z)

proc same_handle(a, b: BodyHandle): bool =
  a.slot == b.slot and a.generation == b.generation

proc make_bb(
  min_x, min_y, min_z,
  max_x, max_y, max_z: float32,
): FBB =
  result.min = (min_x, min_y, min_z)
  result.max = (max_x, max_y, max_z)

proc quat_z(angle_deg: float32): QF =
  let half_angle = angle_deg * PI.float32 / 360'f32
  normalized((0'f32, 0'f32, sin(half_angle), cos(half_angle)))

proc make_a2s_hit(
  idx: int,
  min_x = 0'f32,
  min_y = -1'f32,
  min_z = -1'f32,
  max_x = 1'f32,
  max_y = 1'f32,
  max_z = 1'f32,
): A2sBroadphaseHit =
  (
    body: make_handle(idx),
    static_bb: make_bb(min_x, min_y, min_z, max_x, max_y, max_z),
  )

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

proc init_pool_fixture(): PoolFixture =
  result.data = InternalData()
  result.tree = init_dynamic_aabb_tree[BodyHandle]()
  result.pool = init_narrowphase_pool()

proc deinit(fixture: var PoolFixture) =
  if not fixture.pool.is_nil:
    fixture.pool.deinit_narrowphase_pool()
    fixture.pool = nil

proc sync_body_inputs(fixture: PoolFixture) =
  fixture.pool.set_body_inputs(
    fixture.data.slot_count(),
    fixture.data.local_pos.len,
    fixture.data.slot_to_dense_ptr(),
    fixture.data.cached_center_ptr(),
    fixture.data.cached_half_extents_ptr(),
    fixture.data.cached_world_axes_ptr(),
  )

proc add_body(
  fixture: var PoolFixture,
  pos: D3,
  dimensions: F3,
  vel: F3 = (0'f32, 0'f32, 0'f32),
  ω: F3 = (0'f32, 0'f32, 0'f32),
  rot: QF = quat_identity(float32),
  inverse_mass = 1'f32,
): BodyHandle =
  fixture.data.create_cuboid(
    aabb_tree = fixture.tree,
    initial_pos = pos,
    vel = vel,
    ω = ω,
    rot = rot,
    dimensions = dimensions,
    inverse_mass = inverse_mass,
  )

proc collect_results(pool: NarrowphasePool): seq[NarrowphaseResult] =
  for worker_idx in 0 ..< pool.worker_count:
    for output_idx in 0 ..< pool.worker_output_count(worker_idx):
      result.add pool.worker_output_at(worker_idx, output_idx)

proc test_pool_init() =
  let pool = init_narrowphase_pool()
  defer: deinit_narrowphase_pool(pool)

  doAssert pool.worker_count >= 1
  for worker_idx in 0 ..< pool.worker_count:
    doAssert pool.worker_output_count(worker_idx) == 0
    doAssert range_len(pool.worker_job_range(worker_idx)) == 0

proc test_a2a_broadphase_raw_buffer_growth() =
  let pool = init_narrowphase_pool()
  defer: deinit_narrowphase_pool(pool)

  let initial_capacity = pool.a2a_broadphase_result_capacity()
  for idx in 0 ..< max(4, pool.worker_count + 1):
    pool.add_a2a_broadphase_result(make_a2a_pair(idx))

  doAssert pool.a2a_broadphase_result_count() == max(4, pool.worker_count + 1)
  doAssert pool.a2a_broadphase_result_capacity() > initial_capacity

  for idx in 0 ..< pool.a2a_broadphase_result_count():
    doAssert pool.a2a_broadphase_result_at(idx) == make_a2a_pair(idx)

proc test_a2s_broadphase_raw_buffer_growth() =
  let pool = init_narrowphase_pool()
  defer: deinit_narrowphase_pool(pool)

  let initial_capacity = pool.a2s_broadphase_result_capacity()
  for idx in 0 ..< max(4, pool.worker_count + 1):
    pool.add_a2s_broadphase_result(make_a2s_hit(idx, min_x = idx.float32, max_x = idx.float32 + 1'f32))

  doAssert pool.a2s_broadphase_result_count() == max(4, pool.worker_count + 1)
  doAssert pool.a2s_broadphase_result_capacity() > initial_capacity

  for idx in 0 ..< pool.a2s_broadphase_result_count():
    doAssert pool.a2s_broadphase_result_at(idx) == make_a2s_hit(idx, min_x = idx.float32, max_x = idx.float32 + 1'f32)

proc test_empty_dispatch() =
  let pool = init_narrowphase_pool()
  defer: deinit_narrowphase_pool(pool)

  pool.clear_narrowphase_inputs()
  pool.dispatch_narrowphase_and_wait()

  for worker_idx in 0 ..< pool.worker_count:
    doAssert pool.worker_output_count(worker_idx) == 0
    doAssert range_len(pool.worker_job_range(worker_idx)) == 0

proc test_single_pair_dispatch() =
  let pool = init_narrowphase_pool()
  defer: deinit_narrowphase_pool(pool)

  pool.add_a2a_broadphase_result(make_a2a_pair(0))
  pool.dispatch_narrowphase_and_wait()

  doAssert range_len(pool.worker_job_range(0)) == 1
  doAssert pool.worker_output_capacity(0) >= 1
  doAssert pool.worker_output_count(0) == 0

  for worker_idx in 1 ..< pool.worker_count:
    doAssert range_len(pool.worker_job_range(worker_idx)) == 0
    doAssert pool.worker_output_count(worker_idx) == 0

proc test_mixed_dispatch_partitions_combined_work() =
  let pool = init_narrowphase_pool()
  defer: deinit_narrowphase_pool(pool)

  let pair_count = max(3, pool.worker_count)
  let body_count = max(2, pool.worker_count)
  for idx in 0 ..< pair_count:
    pool.add_a2a_broadphase_result(make_a2a_pair(idx))
  for idx in 0 ..< body_count:
    pool.add_a2s_broadphase_result(make_a2s_hit(100 + idx, min_x = idx.float32, max_x = idx.float32 + 1'f32))

  pool.dispatch_narrowphase_and_wait()

  let total_jobs = pair_count + body_count
  let active_workers = min(pool.worker_count, total_jobs)
  var next_start = 0
  for worker_idx in 0 ..< active_workers:
    let r = pool.worker_job_range(worker_idx)
    let current_len = range_len(r)
    doAssert current_len > 0
    doAssert r.a == next_start
    doAssert pool.worker_output_capacity(worker_idx) >= current_len
    next_start = r.b + 1

  doAssert next_start == total_jobs

proc test_repeated_dispatch_reuses_capacity() =
  let pool = init_narrowphase_pool()
  defer: deinit_narrowphase_pool(pool)

  let first_pair_count = max(3, pool.worker_count)
  for idx in 0 ..< first_pair_count:
    pool.add_a2a_broadphase_result(make_a2a_pair(idx))
  pool.dispatch_narrowphase_and_wait()

  let old_capacity = pool.worker_output_capacity(0)
  doAssert old_capacity >= 1

  pool.clear_narrowphase_inputs()
  pool.add_a2a_broadphase_result(make_a2a_pair(99))
  pool.dispatch_narrowphase_and_wait()

  doAssert pool.worker_output_capacity(0) == old_capacity
  doAssert pool.worker_output_count(0) == 0

proc test_pool_shutdown() =
  let pool = init_narrowphase_pool()
  pool.deinit_narrowphase_pool()

proc test_a2s_face_manifold_generation() =
  var fixture = init_pool_fixture()
  defer: fixture.deinit()

  let handle = fixture.add_body(
    pos = (0'f64, 0'f64, 0'f64),
    dimensions = (2'f32, 2'f32, 2'f32),
  )
  fixture.sync_body_inputs()

  fixture.pool.clear_narrowphase_inputs()
  fixture.pool.add_a2s_broadphase_result(
    (body: handle, static_bb: make_bb(0.5'f32, -1'f32, -1'f32, 2.5'f32, 1'f32, 1'f32))
  )
  fixture.pool.dispatch_narrowphase_and_wait()

  let results = fixture.pool.collect_results()
  doAssert results.len == 1
  doAssert results[0].kind == nrk_a2s
  doAssert same_handle(results[0].a2s.body_a, handle)
  doAssert results[0].a2s.contact_count == 4

  for idx in 0 ..< results[0].a2s.contact_count.int:
    let contact = results[0].a2s.contact_points[idx]
    doAssert contact.penetration_depth > 0'f32
    doAssert approx_equal(abs(contact.normal.x), 1'f32)
    doAssert approx_equal(contact.normal.y, 0'f32)
    doAssert approx_equal(contact.normal.z, 0'f32)

proc test_a2s_face_manifold_is_canonical_when_static_is_reference() =
  var fixture = init_pool_fixture()
  defer: fixture.deinit()

  let handle = fixture.add_body(
    pos = (-0.2'f64, 0'f64, 0'f64),
    dimensions = (2'f32, 2'f32, 2'f32),
    rot = quat_z(10'f32),
  )
  fixture.sync_body_inputs()

  fixture.pool.clear_narrowphase_inputs()
  fixture.pool.add_a2s_broadphase_result(
    (body: handle, static_bb: make_bb(0.5'f32, -1'f32, -1'f32, 2.5'f32, 1'f32, 1'f32))
  )
  fixture.pool.dispatch_narrowphase_and_wait()

  let results = fixture.pool.collect_results()
  doAssert results.len == 1
  doAssert results[0].kind == nrk_a2s
  doAssert results[0].a2s.contact_count > 0

  for idx in 0 ..< results[0].a2s.contact_count.int:
    let contact = results[0].a2s.contact_points[idx]
    let dynamic_surface_point = contact.position + contact.normal * contact.penetration_depth
    doAssert approx_equal(contact.position.x, 0.5'f32)
    doAssert approx_equal(contact.normal.x, 1'f32)
    doAssert approx_equal(contact.normal.y, 0'f32)
    doAssert approx_equal(contact.normal.z, 0'f32)
    doAssert dynamic_surface_point.x > contact.position.x
    doAssert dynamic_surface_point.x <= 1.0001'f32

proc test_a2s_manifold_id_is_stable() =
  var fixture = init_pool_fixture()
  defer: fixture.deinit()

  let handle = fixture.add_body(
    pos = (0'f64, 0'f64, 0'f64),
    dimensions = (2'f32, 2'f32, 2'f32),
  )
  fixture.sync_body_inputs()

  let broadphase_hit = (body: handle, static_bb: make_bb(0.5'f32, -1'f32, -1'f32, 2.5'f32, 1'f32, 1'f32))
  fixture.pool.clear_narrowphase_inputs()
  fixture.pool.add_a2s_broadphase_result(broadphase_hit)
  fixture.pool.dispatch_narrowphase_and_wait()
  let first_id = fixture.pool.collect_results()[0].a2s.manifold_id

  fixture.pool.clear_narrowphase_inputs()
  fixture.pool.add_a2s_broadphase_result(broadphase_hit)
  fixture.pool.dispatch_narrowphase_and_wait()
  let second_id = fixture.pool.collect_results()[0].a2s.manifold_id

  doAssert first_id == second_id

proc test_mixed_a2a_and_a2s_outputs() =
  var fixture = init_pool_fixture()
  defer: fixture.deinit()

  let handle_a = fixture.add_body(
    pos = (0'f64, 0'f64, 0'f64),
    dimensions = (2'f32, 2'f32, 2'f32),
  )
  let handle_b = fixture.add_body(
    pos = (1.5'f64, 0'f64, 0'f64),
    dimensions = (2'f32, 2'f32, 2'f32),
  )
  fixture.sync_body_inputs()

  fixture.pool.clear_narrowphase_inputs()
  fixture.pool.add_a2a_broadphase_result((handle_a, handle_b))
  fixture.pool.add_a2s_broadphase_result(
    (body: handle_a, static_bb: make_bb(0.5'f32, -1'f32, -1'f32, 2.5'f32, 1'f32, 1'f32))
  )
  fixture.pool.dispatch_narrowphase_and_wait()

  let results = fixture.pool.collect_results()
  var a2a_count = 0
  var a2s_count = 0
  for result in results:
    case result.kind
    of nrk_a2a:
      inc a2a_count
    of nrk_a2s:
      inc a2s_count

  doAssert a2a_count == 1
  doAssert a2s_count == 1

proc test_a2a_face_manifold_is_canonical_when_body_b_is_reference() =
  var fixture = init_pool_fixture()
  defer: fixture.deinit()

  let handle_a = fixture.add_body(
    pos = (-0.2'f64, 0'f64, 0'f64),
    dimensions = (2'f32, 2'f32, 2'f32),
    rot = quat_z(10'f32),
  )
  let handle_b = fixture.add_body(
    pos = (1.5'f64, 0'f64, 0'f64),
    dimensions = (2'f32, 2'f32, 2'f32),
  )
  fixture.sync_body_inputs()

  fixture.pool.clear_narrowphase_inputs()
  fixture.pool.add_a2a_broadphase_result((handle_a, handle_b))
  fixture.pool.dispatch_narrowphase_and_wait()

  let results = fixture.pool.collect_results()
  doAssert results.len == 1
  doAssert results[0].kind == nrk_a2a
  doAssert same_handle(results[0].a2a.body_a, handle_a)
  doAssert same_handle(results[0].a2a.body_b, handle_b)
  doAssert results[0].a2a.contact_count > 0

  for idx in 0 ..< results[0].a2a.contact_count.int:
    let contact = results[0].a2a.contact_points[idx]
    let point_a = contact.position + contact.normal * contact.penetration_depth
    doAssert approx_equal(contact.position.x, 0.5'f32)
    doAssert approx_equal(contact.normal.x, 1'f32)
    doAssert approx_equal(contact.normal.y, 0'f32)
    doAssert approx_equal(contact.normal.z, 0'f32)
    doAssert point_a.x > contact.position.x
    doAssert point_a.x <= 1.0001'f32

proc test_face_manifold_generation() =
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
    pos = (1.5'f64, 0'f64, 0'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    dimensions = (2'f32, 2'f32, 2'f32),
  )

  tick_world(0)

  doAssert world.a2a_narrowphase_manifold_count() == 1
  let manifold = world.get_a2a_narrowphase_manifold(0)
  let handle_a: BodyHandle = packed_a
  let handle_b: BodyHandle = packed_b

  doAssert(
    (same_handle(manifold.body_a, handle_a) and same_handle(manifold.body_b, handle_b)) or
    (same_handle(manifold.body_a, handle_b) and same_handle(manifold.body_b, handle_a))
  )
  doAssert manifold.contact_count == 4

  for idx in 0 ..< manifold.contact_count.int:
    let contact = manifold.contact_points[idx]
    doAssert contact.penetration_depth > 0'f32
    doAssert approx_equal(abs(contact.normal.x), 1'f32)
    doAssert approx_equal(contact.normal.y, 0'f32)
    doAssert approx_equal(contact.normal.z, 0'f32)
    doAssert contact.position.y >= -1.0001'f32 and contact.position.y <= 1.0001'f32
    doAssert contact.position.z >= -1.0001'f32 and contact.position.z <= 1.0001'f32
    let surface_point = contact.position + contact.normal * contact.penetration_depth
    doAssert approx_equal(min(contact.position.x, surface_point.x), 0.5'f32)
    doAssert approx_equal(max(contact.position.x, surface_point.x), 1.0'f32)

proc test_face_manifold_is_order_invariant() =
  let zero: F3 = (x: 0'f32, y: 0'f32, z: 0'f32)

  proc make_world(reverse_order: bool): A2aCollisionManifold =
    let world = fresh_world()
    var packed_a = PackedHandle(slot: -1, generation: 0)
    var packed_b = PackedHandle(slot: -1, generation: 0)

    if reverse_order:
      world.enqueue_add(
        packed_handle = addr packed_b,
        pos = (1.5'f64, 0'f64, 0'f64),
        vel = zero,
        ω = zero,
        dimensions = (x: 2'f32, y: 2'f32, z: 2'f32),
      )
      world.enqueue_add(
        packed_handle = addr packed_a,
        pos = (0'f64, 0'f64, 0'f64),
        vel = zero,
        ω = zero,
        dimensions = (x: 2'f32, y: 2'f32, z: 2'f32),
      )
    else:
      world.enqueue_add(
        packed_handle = addr packed_a,
        pos = (0'f64, 0'f64, 0'f64),
        vel = zero,
        ω = zero,
        dimensions = (x: 2'f32, y: 2'f32, z: 2'f32),
      )
      world.enqueue_add(
        packed_handle = addr packed_b,
        pos = (1.5'f64, 0'f64, 0'f64),
        vel = zero,
        ω = zero,
        dimensions = (x: 2'f32, y: 2'f32, z: 2'f32),
      )

    tick_world(0)
    doAssert world.a2a_narrowphase_manifold_count() == 1
    world.get_a2a_narrowphase_manifold(0)

  let forward = make_world(false)
  let reversed = make_world(true)

  doAssert forward.contact_count == reversed.contact_count
  for idx in 0 ..< forward.contact_count.int:
    let forward_contact = forward.contact_points[idx]
    let reversed_contact = reversed.contact_points[idx]
    let forward_surface = forward_contact.position + forward_contact.normal * forward_contact.penetration_depth
    let reversed_surface = reversed_contact.position + reversed_contact.normal * reversed_contact.penetration_depth

    doAssert approx_equal(min(forward_contact.position.x, forward_surface.x), 0.5'f32)
    doAssert approx_equal(max(forward_contact.position.x, forward_surface.x), 1.0'f32)
    doAssert approx_equal(min(reversed_contact.position.x, reversed_surface.x), 0.5'f32)
    doAssert approx_equal(max(reversed_contact.position.x, reversed_surface.x), 1.0'f32)
    doAssert approx_equal(abs(forward_contact.normal.x), 1'f32)
    doAssert approx_equal(abs(reversed_contact.normal.x), 1'f32)
    doAssert approx_vec_equal(
      (
        x: min(forward_contact.position.x, forward_surface.x),
        y: min(forward_contact.position.y, forward_surface.y),
        z: min(forward_contact.position.z, forward_surface.z),
      ),
      (
        x: min(reversed_contact.position.x, reversed_surface.x),
        y: min(reversed_contact.position.y, reversed_surface.y),
        z: min(reversed_contact.position.z, reversed_surface.z),
      ),
    )
    doAssert approx_vec_equal(
      (
        x: max(forward_contact.position.x, forward_surface.x),
        y: max(forward_contact.position.y, forward_surface.y),
        z: max(forward_contact.position.z, forward_surface.z),
      ),
      (
        x: max(reversed_contact.position.x, reversed_surface.x),
        y: max(reversed_contact.position.y, reversed_surface.y),
        z: max(reversed_contact.position.z, reversed_surface.z),
      ),
    )

when is_main_module:
  test_pool_init()
  test_a2a_broadphase_raw_buffer_growth()
  test_a2s_broadphase_raw_buffer_growth()
  test_empty_dispatch()
  test_single_pair_dispatch()
  test_mixed_dispatch_partitions_combined_work()
  test_repeated_dispatch_reuses_capacity()
  test_pool_shutdown()
  test_a2s_face_manifold_generation()
  test_a2s_face_manifold_is_canonical_when_static_is_reference()
  test_a2s_manifold_id_is_stable()
  test_mixed_a2a_and_a2s_outputs()
  test_a2a_face_manifold_is_canonical_when_body_b_is_reference()
  test_face_manifold_generation()
  test_face_manifold_is_order_invariant()
  deinit_worlds()
  echo "narrowphase tests passed"
