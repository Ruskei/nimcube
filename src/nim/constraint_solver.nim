import std/hashes
import std/tables

import cuboids
import narrowphase
import physics_math

const
  normal_iterations* = 4
  friction_iterations* = 2
  velocity_solve_sor* = 1.1'f32
  baumgarte_beta* = 0.2'f32
  baumgarte_slop* = 0.01'f32
  friction_coefficient* = 0.5'f32
  baumgarte_max_bias_velocity = 10'f32
  constraint_epsilon = 1.0e-6'f32

type
  WarmStartImpulse* = object
    normal*: float32
    t1*: float32
    t2*: float32

  WarmStartContactRecord* = object
    point*: F3
    impulses*: WarmStartImpulse

  A2aWarmStartEntry* = object
    contact_count*: uint8
    contacts*: array[4, WarmStartContactRecord]

  A2sWarmStartEntry* = object
    contact_count*: uint8
    contacts*: array[4, WarmStartContactRecord]

  A2aWarmStartKey* = tuple[a, b: BodyHandle]
  A2sWarmStartKey* = tuple[body: BodyHandle, static_hash: Hash]

  A2aVelocityConstraintPoint* = object
    dense_a*: int
    dense_b*: int
    axis*: F3
    r_a*: F3
    r_b*: F3
    inv_mass_a*: float32
    inv_mass_b*: float32
    inv_inertia_times_r_a_cross_axis*: F3
    inv_inertia_times_r_b_cross_axis*: F3
    effective_mass*: float32
    bias_velocity*: float32
    accumulated_impulse*: float32
    normal_constraint_idx*: int
    body_a*: BodyHandle
    body_b*: BodyHandle
    contact_point*: F3
    friction_constraint_idx_u*: int
    friction_constraint_idx_v*: int

  A2sVelocityConstraintPoint* = object
    dense_a*: int
    axis*: F3
    r_a*: F3
    inv_mass_a*: float32
    inv_inertia_times_r_a_cross_axis*: F3
    effective_mass*: float32
    bias_velocity*: float32
    accumulated_impulse*: float32
    normal_constraint_idx*: int
    body_a*: BodyHandle
    static_hash*: Hash
    contact_point*: F3
    friction_constraint_idx_u*: int
    friction_constraint_idx_v*: int

  A2aNormalVelocityConstraintSubBuffer* = object
    constraints*: seq[A2aVelocityConstraintPoint]
    count*: int

  A2sNormalVelocityConstraintSubBuffer* = object
    constraints*: seq[A2sVelocityConstraintPoint]
    count*: int

  A2aFrictionVelocityConstraintSubBuffer* = object
    constraints*: seq[A2aVelocityConstraintPoint]
    count*: int

  A2sFrictionVelocityConstraintSubBuffer* = object
    constraints*: seq[A2sVelocityConstraintPoint]
    count*: int

  VelocityConstraintBuffer* = object
    a2a_normals*: A2aNormalVelocityConstraintSubBuffer
    a2s_normals*: A2sNormalVelocityConstraintSubBuffer
    a2a_frictions*: A2aFrictionVelocityConstraintSubBuffer
    a2s_frictions*: A2sFrictionVelocityConstraintSubBuffer

proc canonical_a2a_warm_start_key*(a, b: BodyHandle): A2aWarmStartKey =
  if a.slot < b.slot or (a.slot == b.slot and a.generation <= b.generation):
    (a: a, b: b)
  else:
    (a: b, b: a)

proc a2s_warm_start_key*(body: BodyHandle, static_hash: Hash): A2sWarmStartKey =
  (body: body, static_hash: static_hash)

proc hash*(key: A2aWarmStartKey): Hash =
  let canonical = canonical_a2a_warm_start_key(key.a, key.b)
  let packed =
    ((canonical.a.slot.uint32 and 0xFFFF'u32) shl 16) or
    (canonical.b.slot.uint32 and 0xFFFF'u32)
  Hash(packed)

proc hash*(key: A2sWarmStartKey): Hash =
  var h: Hash = 0
  h = h !& hash(key.body.slot)
  h = h !& hash(key.body.generation)
  h = h !& key.static_hash
  result = !$h

proc orthonormal_basis(normal: F3): tuple[u, v: F3] =
  let abs_nx = abs(normal.x)
  let abs_ny = abs(normal.y)
  let abs_nz = abs(normal.z)
  let perp =
    if abs_nx <= abs_ny and abs_nx <= abs_nz:
      (0'f32, -normal.z, normal.y)
    elif abs_ny <= abs_nx and abs_ny <= abs_nz:
      (normal.z, 0'f32, -normal.x)
    else:
      (-normal.y, normal.x, 0'f32)
  let len = perp.length

  result.u =
    if len > constraint_epsilon:
      perp / len
    else:
      (0'f32, 1'f32, 0'f32)
  result.v = normalized(normal × result.u)

proc resolve_dense_idx(data: InternalData, handle: BodyHandle, slot_to_dense: ptr UncheckedArray[int]): int =
  if slot_to_dense.is_nil or handle.slot < 0 or handle.slot >= data.slot_count():
    return -1
  result = slot_to_dense[handle.slot]
  if result < 0 or result >= data.local_pos.len:
    result = -1

proc ensure_constraint_capacity[T](constraints: var seq[T], required_capacity: int) =
  if constraints.len < required_capacity:
    constraints.setLen(required_capacity)

proc invalid_body_handle(): BodyHandle =
  BodyHandle(slot: -1, generation: 0'u)

proc match_warm_start_impulses[T](
  entry: T,
  contact_point: F3,
  used_contacts: var array[4, bool],
): WarmStartImpulse =
  var best_idx = -1
  var best_distance_sq = high(float32)
  let contact_count = min(entry.contact_count.int, entry.contacts.len)

  for contact_idx in 0 ..< contact_count:
    if used_contacts[contact_idx]:
      continue

    let delta = entry.contacts[contact_idx].point - contact_point
    let distance_sq = delta.length_squared
    if best_idx < 0 or distance_sq < best_distance_sq:
      best_idx = contact_idx
      best_distance_sq = distance_sq

  if best_idx >= 0:
    used_contacts[best_idx] = true
    result = entry.contacts[best_idx].impulses

proc apply_delta_impulse(data: InternalData, constraint: A2aVelocityConstraintPoint, delta_impulse: float32)
proc apply_delta_impulse(data: InternalData, constraint: A2sVelocityConstraintPoint, delta_impulse: float32)

proc apply_warm_start_impulse[T](data: InternalData, constraint: var T, impulse: float32) =
  if abs(impulse) <= constraint_epsilon:
    return

  constraint.accumulated_impulse = impulse
  data.apply_delta_impulse(constraint, impulse)

proc append_a2a_constraint(
  buffer: var seq[A2aVelocityConstraintPoint],
  count: var int,
  data: InternalData,
  dense_a: int,
  dense_b: int,
  inv_mass_a: float32,
  inv_mass_b: float32,
  r_a: F3,
  r_b: F3,
  axis: F3,
  bias_velocity: float32,
  normal_constraint_idx: int,
) =
  let r_a_cross_axis = r_a × axis
  let r_b_cross_axis = r_b × axis
  let inv_inertia_times_r_a_cross_axis =
    if inv_mass_a > 0'f32:
      data.apply_inverse_inertia_world(dense_a, r_a_cross_axis)
    else:
      (0'f32, 0'f32, 0'f32)
  let inv_inertia_times_r_b_cross_axis =
    if inv_mass_b > 0'f32:
      data.apply_inverse_inertia_world(dense_b, r_b_cross_axis)
    else:
      (0'f32, 0'f32, 0'f32)

  let denominator =
    inv_mass_a +
    inv_mass_b +
    (r_a_cross_axis ∙ inv_inertia_times_r_a_cross_axis) +
    (r_b_cross_axis ∙ inv_inertia_times_r_b_cross_axis)
  if not denominator.is_finite or denominator <= constraint_epsilon:
    return

  buffer[count] = A2aVelocityConstraintPoint(
    dense_a: dense_a,
    dense_b: dense_b,
    axis: axis,
    r_a: r_a,
    r_b: r_b,
    inv_mass_a: inv_mass_a,
    inv_mass_b: inv_mass_b,
    inv_inertia_times_r_a_cross_axis: inv_inertia_times_r_a_cross_axis,
    inv_inertia_times_r_b_cross_axis: inv_inertia_times_r_b_cross_axis,
    effective_mass: 1'f32 / denominator,
    bias_velocity: bias_velocity,
    accumulated_impulse: 0'f32,
    normal_constraint_idx: normal_constraint_idx,
    body_a: invalid_body_handle(),
    body_b: invalid_body_handle(),
    contact_point: default(F3),
    friction_constraint_idx_u: -1,
    friction_constraint_idx_v: -1,
  )
  inc count

proc append_a2s_constraint(
  buffer: var seq[A2sVelocityConstraintPoint],
  count: var int,
  data: InternalData,
  dense_a: int,
  inv_mass_a: float32,
  r_a: F3,
  axis: F3,
  bias_velocity: float32,
  normal_constraint_idx: int,
) =
  let r_a_cross_axis = r_a × axis
  let inv_inertia_times_r_a_cross_axis = data.apply_inverse_inertia_world(dense_a, r_a_cross_axis)
  let denominator = inv_mass_a + (r_a_cross_axis ∙ inv_inertia_times_r_a_cross_axis)
  if not denominator.is_finite or denominator <= constraint_epsilon:
    return

  buffer[count] = A2sVelocityConstraintPoint(
    dense_a: dense_a,
    axis: axis,
    r_a: r_a,
    inv_mass_a: inv_mass_a,
    inv_inertia_times_r_a_cross_axis: inv_inertia_times_r_a_cross_axis,
    effective_mass: 1'f32 / denominator,
    bias_velocity: bias_velocity,
    accumulated_impulse: 0'f32,
    normal_constraint_idx: normal_constraint_idx,
    body_a: invalid_body_handle(),
    static_hash: Hash(0),
    contact_point: default(F3),
    friction_constraint_idx_u: -1,
    friction_constraint_idx_v: -1,
  )
  inc count

proc append_constraints_from_a2a_manifold(
  normal_buffer: var A2aNormalVelocityConstraintSubBuffer,
  friction_buffer: var A2aFrictionVelocityConstraintSubBuffer,
  data: InternalData,
  manifold: A2aCollisionManifold,
  dt: float32,
  slot_to_dense: ptr UncheckedArray[int],
  centers: ptr UncheckedArray[F3],
  a2a_warm_start: TableRef[A2aWarmStartKey, A2aWarmStartEntry],
) =
  let dense_a = resolve_dense_idx(data, manifold.body_a, slot_to_dense)
  let dense_b = resolve_dense_idx(data, manifold.body_b, slot_to_dense)
  if dense_a < 0 or dense_b < 0:
    return

  let inv_mass_a = data.inverse_mass[dense_a]
  let inv_mass_b = data.inverse_mass[dense_b]
  if inv_mass_a <= 0'f32 and inv_mass_b <= 0'f32:
    return

  let warm_start_key = canonical_a2a_warm_start_key(manifold.body_a, manifold.body_b)
  let has_warm_start = a2a_warm_start.hasKey(warm_start_key)
  let warm_start_entry =
    if has_warm_start:
      a2a_warm_start[warm_start_key]
    else:
      default(A2aWarmStartEntry)
  var used_warm_contacts: array[4, bool]

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

    let penetration_error = max(penetration_depth - baumgarte_slop, 0'f32)
    let bias_velocity =
      if penetration_error > 0'f32 and dt > constraint_epsilon:
        min(baumgarte_max_bias_velocity, baumgarte_beta * penetration_error / dt)
      else:
        0'f32
    let warm_impulses =
      if has_warm_start:
        warm_start_entry.match_warm_start_impulses(contact.position, used_warm_contacts)
      else:
        default(WarmStartImpulse)

    let normal_constraint_idx = normal_buffer.count
    normal_buffer.constraints.append_a2a_constraint(
      normal_buffer.count,
      data,
      dense_a,
      dense_b,
      inv_mass_a,
      inv_mass_b,
      r_a,
      r_b,
      normal,
      bias_velocity,
      -1,
    )
    if normal_buffer.count == normal_constraint_idx:
      continue

    let tangents = orthonormal_basis(normal)
    let friction_constraint_idx_u = friction_buffer.count
    friction_buffer.constraints.append_a2a_constraint(
      friction_buffer.count,
      data,
      dense_a,
      dense_b,
      inv_mass_a,
      inv_mass_b,
      r_a,
      r_b,
      tangents.u,
      0'f32,
      normal_constraint_idx,
    )
    let has_friction_u = friction_buffer.count > friction_constraint_idx_u
    let friction_constraint_idx_v = friction_buffer.count
    friction_buffer.constraints.append_a2a_constraint(
      friction_buffer.count,
      data,
      dense_a,
      dense_b,
      inv_mass_a,
      inv_mass_b,
      r_a,
      r_b,
      tangents.v,
      0'f32,
      normal_constraint_idx,
    )
    let has_friction_v = friction_buffer.count > friction_constraint_idx_v

    normal_buffer.constraints[normal_constraint_idx].body_a = manifold.body_a
    normal_buffer.constraints[normal_constraint_idx].body_b = manifold.body_b
    normal_buffer.constraints[normal_constraint_idx].contact_point = contact.position
    normal_buffer.constraints[normal_constraint_idx].friction_constraint_idx_u =
      if has_friction_u:
        friction_constraint_idx_u
      else:
        -1
    normal_buffer.constraints[normal_constraint_idx].friction_constraint_idx_v =
      if has_friction_v:
        friction_constraint_idx_v
      else:
        -1

    data.apply_warm_start_impulse(normal_buffer.constraints[normal_constraint_idx], warm_impulses.normal)
    if has_friction_u:
      data.apply_warm_start_impulse(friction_buffer.constraints[friction_constraint_idx_u], warm_impulses.t1)
    if has_friction_v:
      data.apply_warm_start_impulse(friction_buffer.constraints[friction_constraint_idx_v], warm_impulses.t2)

proc append_constraints_from_a2s_manifold(
  normal_buffer: var A2sNormalVelocityConstraintSubBuffer,
  friction_buffer: var A2sFrictionVelocityConstraintSubBuffer,
  data: InternalData,
  manifold: A2sCollisionManifold,
  dt: float32,
  slot_to_dense: ptr UncheckedArray[int],
  centers: ptr UncheckedArray[F3],
  a2s_warm_start: TableRef[A2sWarmStartKey, A2sWarmStartEntry],
) =
  let dense_a = resolve_dense_idx(data, manifold.body_a, slot_to_dense)
  if dense_a < 0:
    return

  let inv_mass_a = data.inverse_mass[dense_a]
  if inv_mass_a <= 0'f32:
    return

  let warm_start_key = a2s_warm_start_key(manifold.body_a, manifold.static_hash)
  let has_warm_start = a2s_warm_start.hasKey(warm_start_key)
  let warm_start_entry =
    if has_warm_start:
      a2s_warm_start[warm_start_key]
    else:
      default(A2sWarmStartEntry)
  var used_warm_contacts: array[4, bool]

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

    let penetration_error = max(penetration_depth - baumgarte_slop, 0'f32)
    let bias_velocity =
      if penetration_error > 0'f32 and dt > constraint_epsilon:
        min(baumgarte_max_bias_velocity, baumgarte_beta * penetration_error / dt)
      else:
        0'f32
    let warm_impulses =
      if has_warm_start:
        warm_start_entry.match_warm_start_impulses(contact.position, used_warm_contacts)
      else:
        default(WarmStartImpulse)

    let normal_constraint_idx = normal_buffer.count
    normal_buffer.constraints.append_a2s_constraint(
      normal_buffer.count,
      data,
      dense_a,
      inv_mass_a,
      r_a,
      normal,
      bias_velocity,
      -1,
    )
    if normal_buffer.count == normal_constraint_idx:
      continue

    let tangents = orthonormal_basis(normal)
    let friction_constraint_idx_u = friction_buffer.count
    friction_buffer.constraints.append_a2s_constraint(
      friction_buffer.count,
      data,
      dense_a,
      inv_mass_a,
      r_a,
      tangents.u,
      0'f32,
      normal_constraint_idx,
    )
    let has_friction_u = friction_buffer.count > friction_constraint_idx_u
    let friction_constraint_idx_v = friction_buffer.count
    friction_buffer.constraints.append_a2s_constraint(
      friction_buffer.count,
      data,
      dense_a,
      inv_mass_a,
      r_a,
      tangents.v,
      0'f32,
      normal_constraint_idx,
    )
    let has_friction_v = friction_buffer.count > friction_constraint_idx_v

    normal_buffer.constraints[normal_constraint_idx].body_a = manifold.body_a
    normal_buffer.constraints[normal_constraint_idx].static_hash = manifold.static_hash
    normal_buffer.constraints[normal_constraint_idx].contact_point = contact.position
    normal_buffer.constraints[normal_constraint_idx].friction_constraint_idx_u =
      if has_friction_u:
        friction_constraint_idx_u
      else:
        -1
    normal_buffer.constraints[normal_constraint_idx].friction_constraint_idx_v =
      if has_friction_v:
        friction_constraint_idx_v
      else:
        -1

    data.apply_warm_start_impulse(normal_buffer.constraints[normal_constraint_idx], warm_impulses.normal)
    if has_friction_u:
      data.apply_warm_start_impulse(friction_buffer.constraints[friction_constraint_idx_u], warm_impulses.t1)
    if has_friction_v:
      data.apply_warm_start_impulse(friction_buffer.constraints[friction_constraint_idx_v], warm_impulses.t2)

proc precompute_velocity_constraints*(
  buffer: var VelocityConstraintBuffer,
  data: InternalData,
  pool: NarrowphasePool,
  dt: float32,
  a2a_warm_start: TableRef[A2aWarmStartKey, A2aWarmStartEntry],
  a2s_warm_start: TableRef[A2sWarmStartKey, A2sWarmStartEntry],
) =
  buffer.a2a_normals.count = 0
  buffer.a2s_normals.count = 0
  buffer.a2a_frictions.count = 0
  buffer.a2s_frictions.count = 0

  let slot_to_dense = data.slot_to_dense_ptr()
  let centers = data.cached_center_ptr()
  if slot_to_dense.is_nil or centers.is_nil:
    return

  var required_a2a_normal_capacity = 0
  var required_a2s_normal_capacity = 0
  for worker_idx in 0 ..< pool.worker_count:
    let manifold_count = pool.worker_output_count(worker_idx)
    for manifold_idx in 0 ..< manifold_count:
      let result = pool.worker_output_at(worker_idx, manifold_idx)
      case result.kind
      of nrk_a2a:
        required_a2a_normal_capacity += result.a2a.contact_count.int
      of nrk_a2s:
        required_a2s_normal_capacity += result.a2s.contact_count.int

  buffer.a2a_normals.constraints.ensure_constraint_capacity(required_a2a_normal_capacity)
  buffer.a2s_normals.constraints.ensure_constraint_capacity(required_a2s_normal_capacity)
  buffer.a2a_frictions.constraints.ensure_constraint_capacity(2 * required_a2a_normal_capacity)
  buffer.a2s_frictions.constraints.ensure_constraint_capacity(2 * required_a2s_normal_capacity)

  for worker_idx in 0 ..< pool.worker_count:
    let manifold_count = pool.worker_output_count(worker_idx)
    for manifold_idx in 0 ..< manifold_count:
      let result = pool.worker_output_at(worker_idx, manifold_idx)
      case result.kind
      of nrk_a2a:
        buffer.a2a_normals.append_constraints_from_a2a_manifold(
          buffer.a2a_frictions,
          data,
          result.a2a,
          dt,
          slot_to_dense,
          centers,
          a2a_warm_start,
        )
      of nrk_a2s:
        buffer.a2s_normals.append_constraints_from_a2s_manifold(
          buffer.a2s_frictions,
          data,
          result.a2s,
          dt,
          slot_to_dense,
          centers,
          a2s_warm_start,
        )

proc relative_point_velocity(data: InternalData, dense_idx: int, r: F3): F3 =
  data.vel[dense_idx] + (data.ω[dense_idx] × r)

proc constraint_axis_velocity(data: InternalData, constraint: A2aVelocityConstraintPoint): float32 =
  let v_a = data.relative_point_velocity(constraint.dense_a, constraint.r_a)
  let v_b = data.relative_point_velocity(constraint.dense_b, constraint.r_b)
  (v_a - v_b) ∙ constraint.axis

proc constraint_axis_velocity(data: InternalData, constraint: A2sVelocityConstraintPoint): float32 =
  let v_a = data.relative_point_velocity(constraint.dense_a, constraint.r_a)
  v_a ∙ constraint.axis

proc constraint_normal_velocity*(data: InternalData, constraint: A2aVelocityConstraintPoint): float32 =
  data.constraint_axis_velocity(constraint)

proc constraint_normal_velocity*(data: InternalData, constraint: A2sVelocityConstraintPoint): float32 =
  data.constraint_axis_velocity(constraint)

proc constraint_residual*(data: InternalData, constraint: A2aVelocityConstraintPoint): float32 =
  max(constraint.bias_velocity - data.constraint_normal_velocity(constraint), 0'f32)

proc constraint_residual*(data: InternalData, constraint: A2sVelocityConstraintPoint): float32 =
  max(constraint.bias_velocity - data.constraint_normal_velocity(constraint), 0'f32)

proc apply_delta_impulse(data: InternalData, constraint: A2aVelocityConstraintPoint, delta_impulse: float32) =
  let linear_impulse = delta_impulse * constraint.axis

  if constraint.inv_mass_a > 0'f32:
    data.vel[constraint.dense_a] += linear_impulse * constraint.inv_mass_a
    data.ω[constraint.dense_a] += constraint.inv_inertia_times_r_a_cross_axis * delta_impulse

  if constraint.inv_mass_b > 0'f32:
    data.vel[constraint.dense_b] = data.vel[constraint.dense_b] - linear_impulse * constraint.inv_mass_b
    data.ω[constraint.dense_b] = data.ω[constraint.dense_b] - constraint.inv_inertia_times_r_b_cross_axis * delta_impulse

proc apply_delta_impulse(data: InternalData, constraint: A2sVelocityConstraintPoint, delta_impulse: float32) =
  let linear_impulse = delta_impulse * constraint.axis
  data.vel[constraint.dense_a] += linear_impulse * constraint.inv_mass_a
  data.ω[constraint.dense_a] += constraint.inv_inertia_times_r_a_cross_axis * delta_impulse

proc solve_velocity_constraint_row[T](
  constraint: var T,
  data: InternalData,
  sor: float32,
  lower_impulse: float32,
  upper_impulse: float32,
) =
  let axis_velocity = data.constraint_axis_velocity(constraint)
  let candidate_impulse =
    constraint.accumulated_impulse -
    sor * constraint.effective_mass * (axis_velocity - constraint.bias_velocity)
  let new_impulse = clamp(candidate_impulse, lower_impulse, upper_impulse)
  let delta_impulse = new_impulse - constraint.accumulated_impulse
  if abs(delta_impulse) <= constraint_epsilon:
    return

  constraint.accumulated_impulse = new_impulse
  data.apply_delta_impulse(constraint, delta_impulse)

proc solve_a2a_normal_velocity_constraints_iteration*(
  buffer: var A2aNormalVelocityConstraintSubBuffer,
  data: InternalData,
  sor: float32,
) =
  for constraint_idx in 0 ..< buffer.count:
    solve_velocity_constraint_row(buffer.constraints[constraint_idx], data, sor, 0'f32, high(float32))

proc solve_a2s_normal_velocity_constraints_iteration*(
  buffer: var A2sNormalVelocityConstraintSubBuffer,
  data: InternalData,
  sor: float32,
) =
  for constraint_idx in 0 ..< buffer.count:
    solve_velocity_constraint_row(buffer.constraints[constraint_idx], data, sor, 0'f32, high(float32))

proc solve_a2a_friction_velocity_constraints_iteration*(
  buffer: var A2aFrictionVelocityConstraintSubBuffer,
  data: InternalData,
  normal_buffer: A2aNormalVelocityConstraintSubBuffer,
  sor: float32,
) =
  for constraint_idx in 0 ..< buffer.count:
    let normal_impulse = normal_buffer.constraints[buffer.constraints[constraint_idx].normal_constraint_idx].accumulated_impulse
    let friction_limit = friction_coefficient * normal_impulse
    solve_velocity_constraint_row(buffer.constraints[constraint_idx], data, sor, -friction_limit, friction_limit)

proc solve_a2s_friction_velocity_constraints_iteration*(
  buffer: var A2sFrictionVelocityConstraintSubBuffer,
  data: InternalData,
  normal_buffer: A2sNormalVelocityConstraintSubBuffer,
  sor: float32,
) =
  for constraint_idx in 0 ..< buffer.count:
    let normal_impulse = normal_buffer.constraints[buffer.constraints[constraint_idx].normal_constraint_idx].accumulated_impulse
    let friction_limit = friction_coefficient * normal_impulse
    solve_velocity_constraint_row(buffer.constraints[constraint_idx], data, sor, -friction_limit, friction_limit)

proc friction_accumulated_impulse[T](constraints: seq[T], count: int, constraint_idx: int): float32 =
  if constraint_idx < 0 or constraint_idx >= count:
    return 0'f32
  constraints[constraint_idx].accumulated_impulse

proc write_a2a_warm_start_contact(
  a2a_warm_start: TableRef[A2aWarmStartKey, A2aWarmStartEntry],
  key: A2aWarmStartKey,
  record: WarmStartContactRecord,
) =
  var entry =
    if a2a_warm_start.hasKey(key):
      a2a_warm_start[key]
    else:
      default(A2aWarmStartEntry)
  let contact_idx = entry.contact_count.int
  if contact_idx >= entry.contacts.len:
    return

  entry.contacts[contact_idx] = record
  inc entry.contact_count
  a2a_warm_start[key] = entry

proc write_a2s_warm_start_contact(
  a2s_warm_start: TableRef[A2sWarmStartKey, A2sWarmStartEntry],
  key: A2sWarmStartKey,
  record: WarmStartContactRecord,
) =
  var entry =
    if a2s_warm_start.hasKey(key):
      a2s_warm_start[key]
    else:
      default(A2sWarmStartEntry)
  let contact_idx = entry.contact_count.int
  if contact_idx >= entry.contacts.len:
    return

  entry.contacts[contact_idx] = record
  inc entry.contact_count
  a2s_warm_start[key] = entry

proc solve_velocity_constraints*(
  buffer: var VelocityConstraintBuffer,
  data: InternalData,
  normal_iterations: int,
  friction_iterations: int,
  sor: float32,
) =
  for _ in 0 ..< normal_iterations:
    buffer.a2a_normals.solve_a2a_normal_velocity_constraints_iteration(data, sor)
    buffer.a2s_normals.solve_a2s_normal_velocity_constraints_iteration(data, sor)

  for _ in 0 ..< friction_iterations:
    buffer.a2a_frictions.solve_a2a_friction_velocity_constraints_iteration(data, buffer.a2a_normals, sor)
    buffer.a2s_frictions.solve_a2s_friction_velocity_constraints_iteration(data, buffer.a2s_normals, sor)

proc rebuild_warm_start_cache*(
  buffer: VelocityConstraintBuffer,
  a2a_warm_start: TableRef[A2aWarmStartKey, A2aWarmStartEntry],
  a2s_warm_start: TableRef[A2sWarmStartKey, A2sWarmStartEntry],
) =
  a2a_warm_start.clear()
  a2s_warm_start.clear()

  for constraint_idx in 0 ..< buffer.a2a_normals.count:
    let constraint = buffer.a2a_normals.constraints[constraint_idx]
    if constraint.body_a.slot < 0 or constraint.body_b.slot < 0:
      continue

    a2a_warm_start.write_a2a_warm_start_contact(
      canonical_a2a_warm_start_key(constraint.body_a, constraint.body_b),
      WarmStartContactRecord(
        point: constraint.contact_point,
        impulses: WarmStartImpulse(
          normal: constraint.accumulated_impulse,
          t1: friction_accumulated_impulse(
            buffer.a2a_frictions.constraints,
            buffer.a2a_frictions.count,
            constraint.friction_constraint_idx_u,
          ),
          t2: friction_accumulated_impulse(
            buffer.a2a_frictions.constraints,
            buffer.a2a_frictions.count,
            constraint.friction_constraint_idx_v,
          ),
        ),
      ),
    )

  for constraint_idx in 0 ..< buffer.a2s_normals.count:
    let constraint = buffer.a2s_normals.constraints[constraint_idx]
    if constraint.body_a.slot < 0:
      continue

    a2s_warm_start.write_a2s_warm_start_contact(
      a2s_warm_start_key(constraint.body_a, constraint.static_hash),
      WarmStartContactRecord(
        point: constraint.contact_point,
        impulses: WarmStartImpulse(
          normal: constraint.accumulated_impulse,
          t1: friction_accumulated_impulse(
            buffer.a2s_frictions.constraints,
            buffer.a2s_frictions.count,
            constraint.friction_constraint_idx_u,
          ),
          t2: friction_accumulated_impulse(
            buffer.a2s_frictions.constraints,
            buffer.a2s_frictions.count,
            constraint.friction_constraint_idx_v,
          ),
        ),
      ),
    )
