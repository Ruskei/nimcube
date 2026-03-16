import dynamic_aabb_tree
import constraint_solver
import cuboids
import narrowphase
import physics_math

type
  SolverFixture = object
    data: InternalData
    tree: DynamicAabbTree[BodyHandle]
    pool: NarrowphasePool
    buffer: VelocityConstraintBuffer
    handle_a: BodyHandle
    handle_b: BodyHandle

proc approx_equal(a, b: float32, epsilon = 1.0e-4'f32): bool =
  abs(a - b) <= epsilon

proc approx_vec_equal(a, b: F3, epsilon = 1.0e-4'f32): bool =
  approx_equal(a.x, b.x, epsilon) and
  approx_equal(a.y, b.y, epsilon) and
  approx_equal(a.z, b.z, epsilon)

proc sync_body_inputs(pool: NarrowphasePool, data: InternalData) =
  pool.set_body_inputs(
    data.slot_count(),
    data.local_pos.len,
    data.slot_to_dense_ptr(),
    data.cached_center_ptr(),
    data.cached_half_extents_ptr(),
    data.cached_world_axes_ptr(),
  )

proc dispatch_pair(fixture: var SolverFixture, dt: float32) =
  fixture.pool.sync_body_inputs(fixture.data)
  fixture.pool.clear_narrowphase_inputs()
  fixture.pool.add_a2a_broadphase_result((fixture.handle_a, fixture.handle_b))
  fixture.pool.dispatch_narrowphase_and_wait()
  fixture.buffer.precompute_velocity_constraints(fixture.data, fixture.pool, dt)

proc deinit(fixture: var SolverFixture) =
  if not fixture.pool.is_nil:
    fixture.pool.deinit_narrowphase_pool()
    fixture.pool = nil

proc init_fixture(
  pos_a, pos_b: D3,
  vel_a, vel_b: F3,
  dimensions_a, dimensions_b: F3,
  inverse_mass_a, inverse_mass_b: float32,
  dt = 1'f32 / 60'f32,
): SolverFixture =
  let zero_ω: F3 = (x: 0'f32, y: 0'f32, z: 0'f32)
  let identity_rot: QF = quat_identity(float32)
  result.data = InternalData()
  result.tree = init_dynamic_aabb_tree[BodyHandle]()
  result.pool = init_narrowphase_pool()
  result.handle_a = result.data.create_cuboid(
    aabb_tree = result.tree,
    initial_pos = pos_a,
    vel = vel_a,
    ω = zero_ω,
    rot = identity_rot,
    dimensions = dimensions_a,
    inverse_mass = inverse_mass_a,
  )
  result.handle_b = result.data.create_cuboid(
    aabb_tree = result.tree,
    initial_pos = pos_b,
    vel = vel_b,
    ω = zero_ω,
    rot = identity_rot,
    dimensions = dimensions_b,
    inverse_mass = inverse_mass_b,
  )
  result.dispatch_pair(dt)

proc total_residual(fixture: SolverFixture): float32 =
  for idx in 0 ..< fixture.buffer.count:
    result += fixture.data.constraint_residual(fixture.buffer.constraints[idx])

proc test_head_on_equal_mass_collision() =
  var fixture = init_fixture(
    pos_a = (0'f64, 0'f64, 0'f64),
    pos_b = (1.98'f64, 0'f64, 0'f64),
    vel_a = (1'f32, 0'f32, 0'f32),
    vel_b = (-1'f32, 0'f32, 0'f32),
    dimensions_a = (2'f32, 2'f32, 2'f32),
    dimensions_b = (2'f32, 2'f32, 2'f32),
    inverse_mass_a = 1'f32,
    inverse_mass_b = 1'f32,
  )
  defer: fixture.deinit()

  doAssert fixture.buffer.count > 0
  let initial_residual = fixture.total_residual()
  fixture.buffer.solve_velocity_constraints(fixture.data, velocity_solve_iterations, velocity_solve_sor)
  let final_residual = fixture.total_residual()

  doAssert final_residual < initial_residual
  doAssert approx_vec_equal(fixture.data.vel[0] + fixture.data.vel[1], (0'f32, 0'f32, 0'f32), 1.0e-3'f32)
  doAssert fixture.data.vel[0].x < 0'f32
  doAssert fixture.data.vel[1].x > 0'f32

proc test_dynamic_box_against_static_floor() =
  var fixture = init_fixture(
    pos_a = (0'f64, -1'f64, 0'f64),
    pos_b = (0'f64, 0.49'f64, 0'f64),
    vel_a = (0'f32, 0'f32, 0'f32),
    vel_b = (0'f32, -2'f32, 0'f32),
    dimensions_a = (6'f32, 2'f32, 6'f32),
    dimensions_b = (1'f32, 1'f32, 1'f32),
    inverse_mass_a = 0'f32,
    inverse_mass_b = 1'f32,
  )
  defer: fixture.deinit()

  doAssert fixture.buffer.count > 0
  fixture.buffer.solve_velocity_constraints(fixture.data, velocity_solve_iterations, velocity_solve_sor)

  doAssert fixture.data.vel[1].y > -0.2'f32
  doAssert approx_vec_equal(fixture.data.vel[0], (0'f32, 0'f32, 0'f32))
  doAssert approx_vec_equal(fixture.data.ω[0], (0'f32, 0'f32, 0'f32))

proc test_off_center_contact_produces_angular_response() =
  var fixture = init_fixture(
    pos_a = (0'f64, 0'f64, 0'f64),
    pos_b = (0.9'f64, 0.9'f64, 0'f64),
    vel_a = (0'f32, 0'f32, 0'f32),
    vel_b = (-1'f32, 0'f32, 0'f32),
    dimensions_a = (2'f32, 2'f32, 2'f32),
    dimensions_b = (2'f32, 2'f32, 2'f32),
    inverse_mass_a = 0'f32,
    inverse_mass_b = 1'f32,
  )
  defer: fixture.deinit()

  doAssert fixture.buffer.count > 0
  fixture.buffer.solve_velocity_constraints(fixture.data, velocity_solve_iterations, velocity_solve_sor)

  doAssert fixture.data.ω[1].is_finite
  doAssert fixture.data.ω[1].length_squared > 1.0e-4'f32

proc test_convergence_across_iterations() =
  proc residual_after_iterations(iterations: int): float32 =
    var fixture = init_fixture(
      pos_a = (0'f64, 0'f64, 0'f64),
      pos_b = (1.98'f64, 0'f64, 0'f64),
      vel_a = (1'f32, 0'f32, 0'f32),
      vel_b = (-1'f32, 0'f32, 0'f32),
      dimensions_a = (2'f32, 2'f32, 2'f32),
      dimensions_b = (2'f32, 2'f32, 2'f32),
      inverse_mass_a = 1'f32,
      inverse_mass_b = 1'f32,
    )
    defer: fixture.deinit()
    fixture.buffer.solve_velocity_constraints(fixture.data, iterations, velocity_solve_sor)
    fixture.total_residual()

  let residual_1 = residual_after_iterations(1)
  let residual_4 = residual_after_iterations(4)
  let residual_8 = residual_after_iterations(8)

  doAssert residual_4 < residual_1
  doAssert residual_8 < residual_4

proc test_static_static_contacts_are_skipped() =
  var fixture = init_fixture(
    pos_a = (0'f64, 0'f64, 0'f64),
    pos_b = (1.5'f64, 0'f64, 0'f64),
    vel_a = (0'f32, 0'f32, 0'f32),
    vel_b = (0'f32, 0'f32, 0'f32),
    dimensions_a = (2'f32, 2'f32, 2'f32),
    dimensions_b = (2'f32, 2'f32, 2'f32),
    inverse_mass_a = 0'f32,
    inverse_mass_b = 0'f32,
  )
  defer: fixture.deinit()

  doAssert fixture.buffer.count == 0

when is_main_module:
  test_head_on_equal_mass_collision()
  test_dynamic_box_against_static_floor()
  test_off_center_contact_produces_angular_response()
  test_convergence_across_iterations()
  test_static_static_contacts_are_skipped()
  echo "constraint solver tests passed"
