import cuboids
import narrowphase
import physics_math

const
  velocity_solve_iterations* = 8
  velocity_solve_sor* = 1.1'f32
  baumgarte_beta* = 0.2'f32
  baumgarte_slop* = 0.01'f32
  baumgarte_max_bias_velocity = 10'f32
  constraint_epsilon = 1.0e-6'f32

type
  A2aVelocityConstraintPoint* = object
    dense_a*: int
    dense_b*: int
    normal*: F3
    r_a*: F3
    r_b*: F3
    inv_mass_a*: float32
    inv_mass_b*: float32
    inv_inertia_times_r_a_cross_n*: F3
    inv_inertia_times_r_b_cross_n*: F3
    effective_mass*: float32
    bias_velocity*: float32
    accumulated_impulse*: float32

  A2sVelocityConstraintPoint* = object
    dense_a*: int
    normal*: F3
    r_a*: F3
    inv_mass_a*: float32
    inv_inertia_times_r_a_cross_n*: F3
    effective_mass*: float32
    bias_velocity*: float32
    accumulated_impulse*: float32

  A2aVelocityConstraintSubBuffer* = object
    constraints*: seq[A2aVelocityConstraintPoint]
    count*: int

  A2sVelocityConstraintSubBuffer* = object
    constraints*: seq[A2sVelocityConstraintPoint]
    count*: int

  VelocityConstraintBuffer* = object
    a2a*: A2aVelocityConstraintSubBuffer
    a2s*: A2sVelocityConstraintSubBuffer

proc resolve_dense_idx(data: InternalData, handle: BodyHandle, slot_to_dense: ptr UncheckedArray[int]): int =
  if slot_to_dense.is_nil or handle.slot < 0 or handle.slot >= data.slot_count():
    return -1
  result = slot_to_dense[handle.slot]
  if result < 0 or result >= data.local_pos.len:
    result = -1

proc ensure_constraint_capacity[T](constraints: var seq[T], required_capacity: int) =
  if constraints.len < required_capacity:
    constraints.setLen(required_capacity)

proc append_constraints_from_a2a_manifold(
  buffer: var A2aVelocityConstraintSubBuffer,
  data: InternalData,
  manifold: A2aCollisionManifold,
  dt: float32,
  slot_to_dense: ptr UncheckedArray[int],
  centers: ptr UncheckedArray[F3],
) =
  let dense_a = resolve_dense_idx(data, manifold.body_a, slot_to_dense)
  let dense_b = resolve_dense_idx(data, manifold.body_b, slot_to_dense)
  if dense_a < 0 or dense_b < 0:
    return

  let inv_mass_a = data.inverse_mass[dense_a]
  let inv_mass_b = data.inverse_mass[dense_b]
  if inv_mass_a <= 0'f32 and inv_mass_b <= 0'f32:
    return

  for contact_idx in 0 ..< manifold.contact_count.int:
    let contact = manifold.contact_points[contact_idx]
    if contact.normal.length_squared <= constraint_epsilon * constraint_epsilon:
      continue

    # A2A manifolds store the contact point on body B and the penetration direction from body B toward body A.
    let normal = -normalized(contact.normal)
    if normal.length_squared <= constraint_epsilon * constraint_epsilon:
      continue

    let penetration_depth = max(contact.penetration_depth, 0'f32)
    let point_b = contact.position
    let point_a = point_b - normal * penetration_depth
    let r_a = point_a - centers[dense_a]
    let r_b = point_b - centers[dense_b]
    let r_a_cross_n = r_a × normal
    let r_b_cross_n = r_b × normal
    let inv_inertia_times_r_a_cross_n =
      if inv_mass_a > 0'f32:
        data.apply_inverse_inertia_world(dense_a, r_a_cross_n)
      else:
        (0'f32, 0'f32, 0'f32)
    let inv_inertia_times_r_b_cross_n =
      if inv_mass_b > 0'f32:
        data.apply_inverse_inertia_world(dense_b, r_b_cross_n)
      else:
        (0'f32, 0'f32, 0'f32)

    let denominator =
      inv_mass_a +
      inv_mass_b +
      (r_a_cross_n ∙ inv_inertia_times_r_a_cross_n) +
      (r_b_cross_n ∙ inv_inertia_times_r_b_cross_n)
    if not denominator.is_finite or denominator <= constraint_epsilon:
      continue

    let penetration_error = max(penetration_depth - baumgarte_slop, 0'f32)
    let bias_velocity =
      if penetration_error > 0'f32 and dt > constraint_epsilon:
        min(baumgarte_max_bias_velocity, baumgarte_beta * penetration_error / dt)
      else:
        0'f32

    buffer.constraints[buffer.count] = A2aVelocityConstraintPoint(
      dense_a: dense_a,
      dense_b: dense_b,
      normal: normal,
      r_a: r_a,
      r_b: r_b,
      inv_mass_a: inv_mass_a,
      inv_mass_b: inv_mass_b,
      inv_inertia_times_r_a_cross_n: inv_inertia_times_r_a_cross_n,
      inv_inertia_times_r_b_cross_n: inv_inertia_times_r_b_cross_n,
      effective_mass: 1'f32 / denominator,
      bias_velocity: bias_velocity,
      accumulated_impulse: 0'f32,
    )
    inc buffer.count

proc append_constraints_from_a2s_manifold(
  buffer: var A2sVelocityConstraintSubBuffer,
  data: InternalData,
  manifold: A2sCollisionManifold,
  dt: float32,
  slot_to_dense: ptr UncheckedArray[int],
  centers: ptr UncheckedArray[F3],
) =
  let dense_a = resolve_dense_idx(data, manifold.body_a, slot_to_dense)
  if dense_a < 0:
    return

  let inv_mass_a = data.inverse_mass[dense_a]
  if inv_mass_a <= 0'f32:
    return

  for contact_idx in 0 ..< manifold.contact_count.int:
    let contact = manifold.contact_points[contact_idx]
    if contact.normal.length_squared <= constraint_epsilon * constraint_epsilon:
      continue

    let normal = -normalized(contact.normal)
    if normal.length_squared <= constraint_epsilon * constraint_epsilon:
      continue

    let penetration_depth = max(contact.penetration_depth, 0'f32)
    let point_b = contact.position
    let point_a = point_b - normal * penetration_depth
    let r_a = point_a - centers[dense_a]
    let r_a_cross_n = r_a × normal
    let inv_inertia_times_r_a_cross_n = data.apply_inverse_inertia_world(dense_a, r_a_cross_n)
    let denominator = inv_mass_a + (r_a_cross_n ∙ inv_inertia_times_r_a_cross_n)
    if not denominator.is_finite or denominator <= constraint_epsilon:
      continue

    let penetration_error = max(penetration_depth - baumgarte_slop, 0'f32)
    let bias_velocity =
      if penetration_error > 0'f32 and dt > constraint_epsilon:
        min(baumgarte_max_bias_velocity, baumgarte_beta * penetration_error / dt)
      else:
        0'f32

    # echo "  added A2sVelocityConstraintPoint"
    buffer.constraints[buffer.count] = A2sVelocityConstraintPoint(
      dense_a: dense_a,
      normal: normal,
      r_a: r_a,
      inv_mass_a: inv_mass_a,
      inv_inertia_times_r_a_cross_n: inv_inertia_times_r_a_cross_n,
      effective_mass: 1'f32 / denominator,
      bias_velocity: bias_velocity,
      accumulated_impulse: 0'f32,
    )
    inc buffer.count

proc precompute_velocity_constraints*(
  buffer: var VelocityConstraintBuffer,
  data: InternalData,
  pool: NarrowphasePool,
  dt: float32,
) =
  # echo "precompute_velocity_constraints"
  buffer.a2a.count = 0
  buffer.a2s.count = 0

  let slot_to_dense = data.slot_to_dense_ptr()
  let centers = data.cached_center_ptr()
  if slot_to_dense.is_nil or centers.is_nil:
    return

  var required_a2a_capacity = 0
  var required_a2s_capacity = 0
  for worker_idx in 0 ..< pool.worker_count:
    let manifold_count = pool.worker_output_count(worker_idx)
    for manifold_idx in 0 ..< manifold_count:
      let result = pool.worker_output_at(worker_idx, manifold_idx)
      case result.kind
      of nrk_a2a:
        required_a2a_capacity += result.a2a.contact_count.int
      of nrk_a2s:
        required_a2s_capacity += result.a2s.contact_count.int

  # echo "required_a2s_capacity=", required_a2s_capacity

  buffer.a2a.constraints.ensure_constraint_capacity(required_a2a_capacity)
  buffer.a2s.constraints.ensure_constraint_capacity(required_a2s_capacity)

  for worker_idx in 0 ..< pool.worker_count:
    let manifold_count = pool.worker_output_count(worker_idx)
    for manifold_idx in 0 ..< manifold_count:
      let result = pool.worker_output_at(worker_idx, manifold_idx)
      case result.kind
      of nrk_a2a:
        buffer.a2a.append_constraints_from_a2a_manifold(data, result.a2a, dt, slot_to_dense, centers)
      of nrk_a2s:
        # echo "  result.a2s=", result.a2s
        buffer.a2s.append_constraints_from_a2s_manifold(data, result.a2s, dt, slot_to_dense, centers)

proc constraint_normal_velocity*(data: InternalData, constraint: A2aVelocityConstraintPoint): float32 =
  let v_a = data.vel[constraint.dense_a] + (data.ω[constraint.dense_a] × constraint.r_a)
  let v_b = data.vel[constraint.dense_b] + (data.ω[constraint.dense_b] × constraint.r_b)
  (v_a - v_b) ∙ constraint.normal

proc constraint_normal_velocity*(data: InternalData, constraint: A2sVelocityConstraintPoint): float32 =
  let v_a = data.vel[constraint.dense_a] + (data.ω[constraint.dense_a] × constraint.r_a)
  v_a ∙ constraint.normal

proc constraint_residual*(data: InternalData, constraint: A2aVelocityConstraintPoint): float32 =
  max(constraint.bias_velocity - data.constraint_normal_velocity(constraint), 0'f32)

proc constraint_residual*(data: InternalData, constraint: A2sVelocityConstraintPoint): float32 =
  max(constraint.bias_velocity - data.constraint_normal_velocity(constraint), 0'f32)

proc solve_a2a_velocity_constraints_iteration*(
  buffer: var A2aVelocityConstraintSubBuffer,
  data: InternalData,
  sor: float32,
) =
  for constraint_idx in 0 ..< buffer.count:
    let constraint = addr buffer.constraints[constraint_idx]
    let normal_velocity = data.constraint_normal_velocity(constraint[])
    let candidate_impulse =
      constraint[].accumulated_impulse -
      sor * constraint[].effective_mass * (normal_velocity - constraint[].bias_velocity)
    let new_impulse = max(candidate_impulse, 0'f32)
    let delta_impulse = new_impulse - constraint[].accumulated_impulse
    if abs(delta_impulse) <= constraint_epsilon:
      continue

    constraint[].accumulated_impulse = new_impulse
    let linear_impulse = delta_impulse * constraint[].normal

    if constraint[].inv_mass_a > 0'f32:
      data.vel[constraint[].dense_a] += linear_impulse * constraint[].inv_mass_a
      data.ω[constraint[].dense_a] += constraint[].inv_inertia_times_r_a_cross_n * delta_impulse

    if constraint[].inv_mass_b > 0'f32:
      data.vel[constraint[].dense_b] = data.vel[constraint[].dense_b] - linear_impulse * constraint[].inv_mass_b
      data.ω[constraint[].dense_b] = data.ω[constraint[].dense_b] - constraint[].inv_inertia_times_r_b_cross_n * delta_impulse

proc solve_a2s_velocity_constraints_iteration*(
  buffer: var A2sVelocityConstraintSubBuffer,
  data: InternalData,
  sor: float32,
) =
  # echo "solve_a2s_velocity_constraints_iteration, buffer.count=", buffer.count
  for constraint_idx in 0 ..< buffer.count:
    let constraint = addr buffer.constraints[constraint_idx]
    let normal_velocity = data.constraint_normal_velocity(constraint[])
    let candidate_impulse =
      constraint[].accumulated_impulse -
      sor * constraint[].effective_mass * (normal_velocity - constraint[].bias_velocity)
    let new_impulse = max(candidate_impulse, 0'f32)
    let delta_impulse = new_impulse - constraint[].accumulated_impulse
    if abs(delta_impulse) <= constraint_epsilon:
      continue

    constraint[].accumulated_impulse = new_impulse
    let linear_impulse = delta_impulse * constraint[].normal

    # echo "  linear_impulse=", linear_impulse
    # echo "  normal_=", constraint[].normal
    # echo "  normal_velocity=", normal_velocity
    # echo "  bias_velocity=", constraint[].bias_velocity

    data.vel[constraint[].dense_a] += linear_impulse * constraint[].inv_mass_a
    data.ω[constraint[].dense_a] += constraint[].inv_inertia_times_r_a_cross_n * delta_impulse

proc solve_velocity_constraints*(
  buffer: var VelocityConstraintBuffer,
  data: InternalData,
  iterations: int,
  sor: float32,
) =
  for _ in 0 ..< iterations:
    buffer.a2a.solve_a2a_velocity_constraints_iteration(data, sor)
    buffer.a2s.solve_a2s_velocity_constraints_iteration(data, sor)
