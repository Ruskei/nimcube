import std/locks
import std/osproc
import std/typedthreads

import cuboids
import physics_math

const
  max_clipped_vertices = 8
  axis_epsilon = 1.0e-6'f32
  clip_epsilon = 1.0e-5'f32
  face_bias_absolute = 1.0e-4'f32
  face_bias_relative = 0.98'f32
  point_merge_epsilon = 1.0e-4'f32
  volume_epsilon = 1.0e-6'f32

type
  ContactPoint* = object
    position*: F3
    normal*: F3
    penetration_depth*: float32

  CollisionManifold* = object
    contact_points*: array[4, ContactPoint]
    contact_count*: uint8
    body_a*: BodyHandle
    body_b*: BodyHandle
    manifold_id*: uint64

  BroadphasePair* = tuple[a, b: BodyHandle]

  NarrowphaseBodyInputs* = object
    slot_count*: int
    body_count*: int
    slot_to_dense*: ptr UncheckedArray[int]
    centers*: ptr UncheckedArray[F3]
    half_extents*: ptr UncheckedArray[F3]
    world_axes*: ptr UncheckedArray[array[3, F3]]

  NarrowphaseJob* = object
    start_idx*: int
    end_idx*: int
    broadphase_results*: ptr UncheckedArray[BroadphasePair]
    body_inputs*: NarrowphaseBodyInputs

  RawBroadphaseBuffer* = object
    data*: ptr UncheckedArray[BroadphasePair]
    len*: int
    capacity*: int

  WorkerOutputBuffer* = object
    data*: ptr UncheckedArray[CollisionManifold]
    count*: int
    capacity*: int

  NarrowphaseWorkerState = object
    job: NarrowphaseJob
    output: WorkerOutputBuffer
    last_dispatch_epoch: uint64
    finished_epoch: uint64

  NarrowphasePoolObj = object
    worker_count*: int
    workers: seq[Thread[WorkerThreadArg]]
    worker_args: seq[WorkerThreadArg]
    worker_states: seq[NarrowphaseWorkerState]
    broadphase_results*: RawBroadphaseBuffer
    lock: Lock
    work_ready: Cond
    work_done: Cond
    dispatch_epoch: uint64
    finished_workers: int
    active_workers: int
    body_inputs: NarrowphaseBodyInputs
    shutting_down: bool

  NarrowphasePool* = ref NarrowphasePoolObj

  WorkerThreadArg = object
    pool: ptr NarrowphasePoolObj
    worker_idx: int

  FeatureKind = enum
    fk_none
    fk_face_a
    fk_face_b
    fk_edge_edge

  BodyObbView = object
    center: F3
    half_extents: F3
    axes: array[3, F3]

  SatAxisHit = object
    valid: bool
    kind: FeatureKind
    axis_index_a: int
    axis_index_b: int
    normal: F3
    penetration: float32

  VertexBuffer = object
    points: array[max_clipped_vertices, F3]
    count: int

  ContactCandidate = object
    position: F3
    penetration_depth: float32

  SupportEdge = object
    p0: F3
    p1: F3
    sign_mask: uint8

proc hash_mix(seed: var uint64, value: uint64) =
  seed = seed xor (value + 0x9E3779B97F4A7C15'u64 + (seed shl 6) + (seed shr 2))

proc sign_code(sign: float32): uint64 =
  if sign >= 0'f32:
    1'u64
  else:
    0'u64

proc other_axes(axis_idx: int): array[2, int] =
  case axis_idx
  of 0:
    [1, 2]
  of 1:
    [0, 2]
  of 2:
    [0, 1]
  else:
    raise newException(IndexDefect, "face axis index out of range")

proc axis_is_usable(axis: F3): bool =
  axis.is_finite and axis.length_squared > axis_epsilon * axis_epsilon

proc support_radius(body: BodyObbView, axis: F3): float32 =
  abs(body.axes[0] ∙ axis) * body.half_extents.x +
  abs(body.axes[1] ∙ axis) * body.half_extents.y +
  abs(body.axes[2] ∙ axis) * body.half_extents.z

proc load_body_obb(inputs: NarrowphaseBodyInputs, handle: BodyHandle, body: var BodyObbView): bool =
  if
    inputs.slot_to_dense.is_nil or
    inputs.centers.is_nil or
    inputs.half_extents.is_nil or
    inputs.world_axes.is_nil or
    handle.slot < 0 or
    handle.slot >= inputs.slot_count:
    return false

  let dense_idx = inputs.slot_to_dense[handle.slot]
  if dense_idx < 0 or dense_idx >= inputs.body_count:
    return false

  body.center = inputs.centers[dense_idx]
  body.half_extents = inputs.half_extents[dense_idx]
  body.axes = inputs.world_axes[dense_idx]
  result =
    body.center.is_finite and
    body.half_extents.is_finite and
    axis_is_usable(body.axes[0]) and
    axis_is_usable(body.axes[1]) and
    axis_is_usable(body.axes[2])

proc consider_axis(
  best: var SatAxisHit,
  kind: FeatureKind,
  axis_index_a: int,
  axis_index_b: int,
  axis: F3,
  delta: F3,
  penetration: float32,
) =
  if not penetration.is_finite:
    return

  var normal = axis
  if delta ∙ normal < 0'f32:
    normal = -normal

  if not best.valid or penetration < best.penetration:
    best = SatAxisHit(
      valid: true,
      kind: kind,
      axis_index_a: axis_index_a,
      axis_index_b: axis_index_b,
      normal: normal,
      penetration: penetration,
    )

proc find_separating_axis(a, b: BodyObbView, hit: var SatAxisHit): bool =
  let delta = b.center - a.center
  var best_face: SatAxisHit
  var best_edge: SatAxisHit
  var had_usable_axis = false

  for axis_idx in 0 ..< 3:
    let axis = a.axes[axis_idx]
    if not axis_is_usable(axis):
      continue
    had_usable_axis = true
    let penetration = support_radius(a, axis) + support_radius(b, axis) - abs(delta ∙ axis)
    if penetration < 0'f32:
      return false
    consider_axis(best_face, fk_face_a, axis_idx, -1, axis, delta, penetration)

  for axis_idx in 0 ..< 3:
    let axis = b.axes[axis_idx]
    if not axis_is_usable(axis):
      continue
    had_usable_axis = true
    let penetration = support_radius(a, axis) + support_radius(b, axis) - abs(delta ∙ axis)
    if penetration < 0'f32:
      return false
    consider_axis(best_face, fk_face_b, -1, axis_idx, axis, delta, penetration)

  for axis_idx_a in 0 ..< 3:
    for axis_idx_b in 0 ..< 3:
      let raw_axis = a.axes[axis_idx_a] × b.axes[axis_idx_b]
      if not axis_is_usable(raw_axis):
        continue
      let axis = normalized(raw_axis)
      if not axis_is_usable(axis):
        continue
      had_usable_axis = true
      let penetration = support_radius(a, axis) + support_radius(b, axis) - abs(delta ∙ axis)
      if penetration < 0'f32:
        return false
      consider_axis(best_edge, fk_edge_edge, axis_idx_a, axis_idx_b, axis, delta, penetration)

  if not had_usable_axis:
    return false

  if best_edge.valid and
      (not best_face.valid or best_edge.penetration + face_bias_absolute < best_face.penetration * face_bias_relative):
    hit = best_edge
  else:
    hit = best_face

  hit.valid

proc build_face_vertices(
  body: BodyObbView,
  face_axis_idx: int,
  face_sign: float32,
  vertices: var array[4, F3],
) =
  let tangents = other_axes(face_axis_idx)
  let face_center = body.center + face_sign * body.half_extents.component(face_axis_idx) * body.axes[face_axis_idx]
  let tangent_a = body.half_extents.component(tangents[0]) * body.axes[tangents[0]]
  let tangent_b = body.half_extents.component(tangents[1]) * body.axes[tangents[1]]

  vertices[0] = face_center - tangent_a - tangent_b
  vertices[1] = face_center + tangent_a - tangent_b
  vertices[2] = face_center + tangent_a + tangent_b
  vertices[3] = face_center - tangent_a + tangent_b

proc choose_incident_face(body: BodyObbView, reference_normal: F3): tuple[axis_idx: int, face_sign: float32] =
  var best_alignment = -1'f32

  for axis_idx in 0 ..< 3:
    let alignment = abs(body.axes[axis_idx] ∙ reference_normal)
    if alignment > best_alignment:
      best_alignment = alignment
      result.axis_idx = axis_idx
      result.face_sign =
        if body.axes[axis_idx] ∙ reference_normal > 0'f32:
          -1'f32
        else:
          1'f32

proc clip_polygon_against_plane(
  input: VertexBuffer,
  plane_normal: F3,
  plane_offset: float32,
): VertexBuffer =
  if input.count == 0:
    return

  var previous = input.points[input.count - 1]
  var previous_distance = (plane_normal ∙ previous) - plane_offset
  var previous_inside = previous_distance <= clip_epsilon

  for idx in 0 ..< input.count:
    let current = input.points[idx]
    let current_distance = (plane_normal ∙ current) - plane_offset
    let current_inside = current_distance <= clip_epsilon

    if current_inside != previous_inside:
      let denominator = previous_distance - current_distance
      if abs(denominator) > axis_epsilon:
        let t = clamp(previous_distance / denominator, 0'f32, 1'f32)
        if result.count < max_clipped_vertices:
          result.points[result.count] = lerp(previous, current, t)
          inc result.count

    if current_inside and result.count < max_clipped_vertices:
      result.points[result.count] = current
      inc result.count

    previous = current
    previous_distance = current_distance
    previous_inside = current_inside

proc unique_contact_candidates(
  candidates: array[max_clipped_vertices, ContactCandidate],
  candidate_count: int,
  unique_candidates: var array[max_clipped_vertices, ContactCandidate],
): int =
  for idx in 0 ..< candidate_count:
    var merged = false
    for unique_idx in 0 ..< result:
      let delta = candidates[idx].position - unique_candidates[unique_idx].position
      if delta.length_squared <= point_merge_epsilon * point_merge_epsilon:
        if candidates[idx].penetration_depth > unique_candidates[unique_idx].penetration_depth:
          unique_candidates[unique_idx] = candidates[idx]
        merged = true
        break
    if not merged:
      unique_candidates[result] = candidates[idx]
      inc result

proc orthonormal_basis(normal: F3): tuple[u, v: F3] =
  let basis_seed =
    if abs(normal.x) < 0.57735'f32:
      (1'f32, 0'f32, 0'f32)
    elif abs(normal.y) < 0.57735'f32:
      (0'f32, 1'f32, 0'f32)
    else:
      (0'f32, 0'f32, 1'f32)

  result.u = normalized(basis_seed × normal)
  result.v = normalized(normal × result.u)

proc point_score(candidate: ContactCandidate): float32 =
  candidate.penetration_depth

proc contains_index(indices: array[4, int], count: int, idx: int): bool =
  for i in 0 ..< count:
    if indices[i] == idx:
      return true

proc select_farthest_index(
  candidates: array[max_clipped_vertices, ContactCandidate],
  candidate_count: int,
  origin: F3,
  selected: array[4, int],
  selected_count: int,
): int =
  var best_score = -1'f32
  result = -1
  for idx in 0 ..< candidate_count:
    if contains_index(selected, selected_count, idx):
      continue
    let score = (candidates[idx].position - origin).length_squared
    if score > best_score:
      best_score = score
      result = idx

proc select_triangle_index(
  candidates: array[max_clipped_vertices, ContactCandidate],
  candidate_count: int,
  idx0, idx1: int,
  selected: array[4, int],
  selected_count: int,
): int =
  var best_score = -1'f32
  result = -1
  let edge = candidates[idx1].position - candidates[idx0].position

  for idx in 0 ..< candidate_count:
    if contains_index(selected, selected_count, idx):
      continue
    let score = (edge × (candidates[idx].position - candidates[idx0].position)).length_squared
    if score > best_score:
      best_score = score
      result = idx

proc select_volume_index(
  candidates: array[max_clipped_vertices, ContactCandidate],
  candidate_count: int,
  indices: array[4, int],
  normal: F3,
): int =
  let lifted0 = candidates[indices[0]].position + normal * candidates[indices[0]].penetration_depth
  let lifted1 = candidates[indices[1]].position + normal * candidates[indices[1]].penetration_depth
  let lifted2 = candidates[indices[2]].position + normal * candidates[indices[2]].penetration_depth
  let base_cross = (lifted1 - lifted0) × (lifted2 - lifted0)

  var best_score = -1'f32
  result = -1

  for idx in 0 ..< candidate_count:
    if contains_index(indices, 3, idx):
      continue
    let lifted = candidates[idx].position + normal * candidates[idx].penetration_depth
    let score = abs((lifted - lifted0) ∙ base_cross)
    if score > best_score:
      best_score = score
      result = idx

  if best_score <= volume_epsilon:
    let basis = orthonormal_basis(normal)
    var fallback_score = -1'f32
    result = -1
    let p0 = candidates[indices[0]].position
    let p1 = candidates[indices[1]].position
    let p2 = candidates[indices[2]].position
    for idx in 0 ..< candidate_count:
      if contains_index(indices, 3, idx):
        continue
      let candidates_to_score = [p0, p1, p2, candidates[idx].position]
      var min_u = Inf
      var max_u = -Inf
      var min_v = Inf
      var max_v = -Inf
      for point in candidates_to_score:
        let relative = point - p0
        let u = relative ∙ basis.u
        let v = relative ∙ basis.v
        if u < min_u: min_u = u
        if u > max_u: max_u = u
        if v < min_v: min_v = v
        if v > max_v: max_v = v
      let score = (max_u - min_u) * (max_v - min_v)
      if score > fallback_score:
        fallback_score = score
        result = idx

proc write_reduced_contacts(
  candidates: array[max_clipped_vertices, ContactCandidate],
  candidate_count: int,
  normal: F3,
  manifold: var CollisionManifold,
) =
  if candidate_count <= manifold.contact_points.len:
    manifold.contact_count = candidate_count.uint8
    for idx in 0 ..< candidate_count:
      manifold.contact_points[idx] = ContactPoint(
        position: candidates[idx].position,
        normal: normal,
        penetration_depth: candidates[idx].penetration_depth,
      )
    return

  var selected: array[4, int]
  selected[0] = 0
  for idx in 1 ..< candidate_count:
    if point_score(candidates[idx]) > point_score(candidates[selected[0]]):
      selected[0] = idx

  selected[1] = select_farthest_index(candidates, candidate_count, candidates[selected[0]].position, selected, 1)
  if selected[1] < 0:
    selected[1] = (selected[0] + 1) mod candidate_count

  selected[2] = select_triangle_index(candidates, candidate_count, selected[0], selected[1], selected, 2)
  if selected[2] < 0:
    selected[2] = select_farthest_index(candidates, candidate_count, candidates[selected[1]].position, selected, 2)

  selected[3] = select_volume_index(candidates, candidate_count, selected, normal)
  if selected[3] < 0:
    selected[3] = select_farthest_index(candidates, candidate_count, candidates[selected[2]].position, selected, 3)

  manifold.contact_count = 4
  for idx in 0 ..< 4:
    manifold.contact_points[idx] = ContactPoint(
      position: candidates[selected[idx]].position,
      normal: normal,
      penetration_depth: candidates[selected[idx]].penetration_depth,
    )

proc face_manifold_id(
  pair: BroadphasePair,
  reference_is_a: bool,
  reference_axis_idx: int,
  reference_face_sign: float32,
  incident_axis_idx: int,
  incident_face_sign: float32,
): uint64 =
  result = 0xCBF29CE484222325'u64
  result.hash_mix(pair.a.slot.uint64)
  result.hash_mix(pair.a.generation.uint64)
  result.hash_mix(pair.b.slot.uint64)
  result.hash_mix(pair.b.generation.uint64)
  result.hash_mix((if reference_is_a: 1'u64 else: 2'u64))
  result.hash_mix(reference_axis_idx.uint64)
  result.hash_mix(sign_code(reference_face_sign))
  result.hash_mix(incident_axis_idx.uint64)
  result.hash_mix(sign_code(incident_face_sign))

proc build_face_manifold(
  pair: BroadphasePair,
  a, b: BodyObbView,
  hit: SatAxisHit,
  manifold: var CollisionManifold,
): bool =
  let reference_is_a = hit.kind == fk_face_a
  let reference_body = if reference_is_a: a else: b
  let incident_body = if reference_is_a: b else: a
  let reference_axis_idx = if reference_is_a: hit.axis_index_a else: hit.axis_index_b
  let reference_normal = if reference_is_a: hit.normal else: -hit.normal
  let reference_face_sign =
    if reference_body.axes[reference_axis_idx] ∙ reference_normal >= 0'f32:
      1'f32
    else:
      -1'f32
  let incident_face = choose_incident_face(incident_body, reference_normal)

  let tangents = other_axes(reference_axis_idx)
  let face_center =
    reference_body.center +
    reference_face_sign * reference_body.half_extents.component(reference_axis_idx) * reference_body.axes[reference_axis_idx]
  let tangent_u = reference_body.axes[tangents[0]]
  let tangent_v = reference_body.axes[tangents[1]]
  let half_u = reference_body.half_extents.component(tangents[0])
  let half_v = reference_body.half_extents.component(tangents[1])

  var incident_vertices: array[4, F3]
  build_face_vertices(incident_body, incident_face.axis_idx, incident_face.face_sign, incident_vertices)

  var polygon: VertexBuffer
  polygon.count = incident_vertices.len
  for idx in 0 ..< incident_vertices.len:
    polygon.points[idx] = incident_vertices[idx]

  let plane_u_pos = (face_center ∙ tangent_u) + half_u
  let plane_u_neg = (-face_center ∙ tangent_u) + half_u
  let plane_v_pos = (face_center ∙ tangent_v) + half_v
  let plane_v_neg = (-face_center ∙ tangent_v) + half_v

  polygon = clip_polygon_against_plane(polygon, tangent_u, plane_u_pos)
  polygon = clip_polygon_against_plane(polygon, -tangent_u, plane_u_neg)
  polygon = clip_polygon_against_plane(polygon, tangent_v, plane_v_pos)
  polygon = clip_polygon_against_plane(polygon, -tangent_v, plane_v_neg)

  var candidates: array[max_clipped_vertices, ContactCandidate]
  var candidate_count = 0
  for idx in 0 ..< polygon.count:
    let signed_distance = (polygon.points[idx] - face_center) ∙ reference_normal
    let penetration_depth = -signed_distance
    if not penetration_depth.is_finite or penetration_depth <= 0'f32:
      continue
    candidates[candidate_count] = ContactCandidate(
      position: polygon.points[idx],
      penetration_depth: penetration_depth,
    )
    inc candidate_count

  var unique_candidates: array[max_clipped_vertices, ContactCandidate]
  let unique_count = unique_contact_candidates(candidates, candidate_count, unique_candidates)
  if unique_count == 0:
    return false

  manifold.manifold_id = face_manifold_id(
    pair,
    reference_is_a,
    reference_axis_idx,
    reference_face_sign,
    incident_face.axis_idx,
    incident_face.face_sign,
  )
  write_reduced_contacts(unique_candidates, unique_count, reference_normal, manifold)
  result = manifold.contact_count > 0

proc build_support_edge(body: BodyObbView, edge_axis_idx: int, support_direction: F3): SupportEdge =
  var edge_center = body.center

  for axis_idx in 0 ..< 3:
    if axis_idx == edge_axis_idx:
      continue
    let sign =
      if body.axes[axis_idx] ∙ support_direction >= 0'f32:
        1'f32
      else:
        -1'f32
    if sign > 0'f32:
      result.sign_mask = result.sign_mask or (1'u8 shl axis_idx)
    edge_center += sign * body.half_extents.component(axis_idx) * body.axes[axis_idx]

  let edge_extent = body.half_extents.component(edge_axis_idx) * body.axes[edge_axis_idx]
  result.p0 = edge_center - edge_extent
  result.p1 = edge_center + edge_extent

proc closest_points_on_segments(
  p0, p1, q0, q1: F3,
  closest_p: var F3,
  closest_q: var F3,
) =
  let d1 = p1 - p0
  let d2 = q1 - q0
  let r = p0 - q0
  let a = d1 ∙ d1
  let e = d2 ∙ d2
  let f = d2 ∙ r
  var s = 0'f32
  var t = 0'f32

  if a <= axis_epsilon and e <= axis_epsilon:
    discard
  elif a <= axis_epsilon:
    t = clamp(f / e, 0'f32, 1'f32)
  else:
    let c = d1 ∙ r
    if e <= axis_epsilon:
      s = clamp(-c / a, 0'f32, 1'f32)
    else:
      let b = d1 ∙ d2
      let denominator = a * e - b * b
      if abs(denominator) > axis_epsilon:
        s = clamp((b * f - c * e) / denominator, 0'f32, 1'f32)
      let t_nom = b * s + f
      if t_nom < 0'f32:
        t = 0'f32
        s = clamp(-c / a, 0'f32, 1'f32)
      elif t_nom > e:
        t = 1'f32
        s = clamp((b - c) / a, 0'f32, 1'f32)
      else:
        t = t_nom / e

  closest_p = p0 + s * d1
  closest_q = q0 + t * d2

proc edge_manifold_id(
  pair: BroadphasePair,
  axis_idx_a: int,
  axis_idx_b: int,
  sign_mask_a: uint8,
  sign_mask_b: uint8,
): uint64 =
  result = 0xCBF29CE484222325'u64
  result.hash_mix(pair.a.slot.uint64)
  result.hash_mix(pair.a.generation.uint64)
  result.hash_mix(pair.b.slot.uint64)
  result.hash_mix(pair.b.generation.uint64)
  result.hash_mix(3'u64)
  result.hash_mix(axis_idx_a.uint64)
  result.hash_mix(axis_idx_b.uint64)
  result.hash_mix(sign_mask_a.uint64)
  result.hash_mix(sign_mask_b.uint64)

proc build_edge_manifold(
  pair: BroadphasePair,
  a, b: BodyObbView,
  hit: SatAxisHit,
  manifold: var CollisionManifold,
): bool =
  let edge_a = build_support_edge(a, hit.axis_index_a, hit.normal)
  let edge_b = build_support_edge(b, hit.axis_index_b, -hit.normal)

  var point_a: F3
  var point_b: F3
  closest_points_on_segments(edge_a.p0, edge_a.p1, edge_b.p0, edge_b.p1, point_a, point_b)

  let contact_delta = point_a - point_b
  var contact_normal = -hit.normal
  var contact_depth = hit.penetration
  if axis_is_usable(contact_delta):
    contact_normal = normalized(contact_delta)
    contact_depth = contact_delta.length
  elif not axis_is_usable(contact_normal):
    return false

  manifold.contact_count = 1
  manifold.contact_points[0] = ContactPoint(
    position: point_b,
    normal: contact_normal,
    penetration_depth: contact_depth,
  )
  manifold.manifold_id = edge_manifold_id(
    pair,
    hit.axis_index_a,
    hit.axis_index_b,
    edge_a.sign_mask,
    edge_b.sign_mask,
  )
  true

proc generate_cuboid_manifold(
  pair: BroadphasePair,
  inputs: NarrowphaseBodyInputs,
  manifold: var CollisionManifold,
): bool =
  var body_a: BodyObbView
  var body_b: BodyObbView
  if not load_body_obb(inputs, pair.a, body_a) or not load_body_obb(inputs, pair.b, body_b):
    return false

  var hit: SatAxisHit
  if not find_separating_axis(body_a, body_b, hit):
    return false

  manifold = default(CollisionManifold)
  manifold.body_a = pair.a
  manifold.body_b = pair.b

  case hit.kind
  of fk_face_a, fk_face_b:
    build_face_manifold(pair, body_a, body_b, hit, manifold)
  of fk_edge_edge:
    build_edge_manifold(pair, body_a, body_b, hit, manifold)
  else:
    false

proc ensure_shared_capacity[T](
  data: var ptr UncheckedArray[T],
  capacity: var int,
  required_capacity: int,
) =
  if capacity >= required_capacity:
    return

  var next_capacity =
    if capacity == 0:
      1
    else:
      capacity * 2

  if next_capacity < required_capacity:
    next_capacity = required_capacity

  let old_size = Natural(capacity * sizeof(T))
  let new_size = Natural(next_capacity * sizeof(T))

  if data == nil:
    data = cast[ptr UncheckedArray[T]](allocShared0(new_size))
  else:
    data = cast[ptr UncheckedArray[T]](reallocShared0(data, old_size, new_size))

  capacity = next_capacity

proc reset_worker_state(state: var NarrowphaseWorkerState) =
  state.job = NarrowphaseJob(
    start_idx: 0,
    end_idx: 0,
    broadphase_results: nil,
    body_inputs: default(NarrowphaseBodyInputs),
  )
  state.output.count = 0

proc narrowphase_worker(arg: WorkerThreadArg) {.thread, nimcall.} =
  let pool = arg.pool
  let worker_idx = arg.worker_idx

  while true:
    var current_epoch: uint64
    var job: NarrowphaseJob
    var should_process = false

    acquire(pool[].lock)
    try:
      while not pool[].shutting_down and
          pool[].dispatch_epoch <= pool[].worker_states[worker_idx].last_dispatch_epoch:
        wait(pool[].work_ready, pool[].lock)

      if pool[].shutting_down:
        return

      current_epoch = pool[].dispatch_epoch
      pool[].worker_states[worker_idx].last_dispatch_epoch = current_epoch

      if worker_idx < pool[].active_workers:
        pool[].worker_states[worker_idx].output.count = 0
        job = pool[].worker_states[worker_idx].job
        should_process = true
    finally:
      release(pool[].lock)

    if not should_process:
      continue

    for pair_idx in job.start_idx ..< job.end_idx:
      let pair = job.broadphase_results[pair_idx]
      var manifold: CollisionManifold
      if generate_cuboid_manifold(pair, job.body_inputs, manifold):
        let output_count = pool[].worker_states[worker_idx].output.count
        pool[].worker_states[worker_idx].output.data[output_count] = manifold
        inc pool[].worker_states[worker_idx].output.count

    acquire(pool[].lock)
    try:
      pool[].worker_states[worker_idx].finished_epoch = current_epoch
      inc pool[].finished_workers
      signal(pool[].work_done)
    finally:
      release(pool[].lock)

proc init_narrowphase_pool*(): NarrowphasePool =
  new result

  result.worker_count = max(1, countProcessors() - 1)
  result.workers = newSeq[Thread[WorkerThreadArg]](result.worker_count)
  result.worker_args = newSeq[WorkerThreadArg](result.worker_count)
  result.worker_states = newSeq[NarrowphaseWorkerState](result.worker_count)
  initLock(result.lock)
  initCond(result.work_ready)
  initCond(result.work_done)

  let pool_ptr = addr(result[])
  for worker_idx in 0 ..< result.worker_count:
    result.worker_args[worker_idx] = WorkerThreadArg(
      pool: pool_ptr,
      worker_idx: worker_idx,
    )
    result.worker_states[worker_idx].reset_worker_state()
    createThread(result.workers[worker_idx], narrowphase_worker, result.worker_args[worker_idx])

proc deinit_narrowphase_pool*(pool: NarrowphasePool) =
  if pool.is_nil:
    return

  acquire(pool.lock)
  try:
    pool.shutting_down = true
    broadcast(pool.work_ready)
  finally:
    release(pool.lock)

  for worker in pool.workers:
    worker.joinThread()

  for worker_idx in 0 ..< pool.worker_states.len:
    let output_data = pool.worker_states[worker_idx].output.data
    if output_data != nil:
      deallocShared(output_data)
      pool.worker_states[worker_idx].output.data = nil
      pool.worker_states[worker_idx].output.capacity = 0
      pool.worker_states[worker_idx].output.count = 0

  if pool.broadphase_results.data != nil:
    deallocShared(pool.broadphase_results.data)
    pool.broadphase_results.data = nil
    pool.broadphase_results.capacity = 0
    pool.broadphase_results.len = 0

  deinitCond(pool.work_ready)
  deinitCond(pool.work_done)
  deinitLock(pool.lock)

proc clear_broadphase_results*(pool: NarrowphasePool) =
  pool.broadphase_results.len = 0

proc set_body_inputs*(
  pool: NarrowphasePool,
  slot_count: int,
  body_count: int,
  slot_to_dense: ptr UncheckedArray[int],
  centers: ptr UncheckedArray[F3],
  half_extents: ptr UncheckedArray[F3],
  world_axes: ptr UncheckedArray[array[3, F3]],
) =
  pool.body_inputs = NarrowphaseBodyInputs(
    slot_count: slot_count,
    body_count: body_count,
    slot_to_dense: slot_to_dense,
    centers: centers,
    half_extents: half_extents,
    world_axes: world_axes,
  )

proc add_broadphase_result*(pool: NarrowphasePool, pair: BroadphasePair) =
  let next_len = pool.broadphase_results.len + 1
  ensure_shared_capacity(
    pool.broadphase_results.data,
    pool.broadphase_results.capacity,
    next_len,
  )
  pool.broadphase_results.data[pool.broadphase_results.len] = pair
  pool.broadphase_results.len = next_len

proc dispatch_narrowphase_and_wait*(pool: NarrowphasePool) =
  acquire(pool.lock)
  try:
    for worker_idx in 0 ..< pool.worker_states.len:
      pool.worker_states[worker_idx].reset_worker_state()

    if pool.broadphase_results.len == 0:
      pool.active_workers = 0
      pool.finished_workers = 0
      return

    pool.active_workers = min(pool.worker_count, pool.broadphase_results.len)
    pool.finished_workers = 0

    let base_chunk_size = pool.broadphase_results.len div pool.active_workers
    let extra_pairs = pool.broadphase_results.len mod pool.active_workers

    for worker_idx in 0 ..< pool.active_workers:
      let start_idx = worker_idx * base_chunk_size + min(worker_idx, extra_pairs)
      let end_idx = start_idx + base_chunk_size + (if worker_idx < extra_pairs: 1 else: 0)
      pool.worker_states[worker_idx].job = NarrowphaseJob(
        start_idx: start_idx,
        end_idx: end_idx,
        broadphase_results: pool.broadphase_results.data,
        body_inputs: pool.body_inputs,
      )
      ensure_shared_capacity(
        pool.worker_states[worker_idx].output.data,
        pool.worker_states[worker_idx].output.capacity,
        end_idx - start_idx,
      )

    inc pool.dispatch_epoch
    broadcast(pool.work_ready)

    while pool.finished_workers < pool.active_workers:
      wait(pool.work_done, pool.lock)
  finally:
    release(pool.lock)

proc broadphase_result_count*(pool: NarrowphasePool): int =
  pool.broadphase_results.len

proc broadphase_result_capacity*(pool: NarrowphasePool): int =
  pool.broadphase_results.capacity

proc broadphase_result_at*(pool: NarrowphasePool, idx: int): BroadphasePair =
  doAssert idx >= 0 and idx < pool.broadphase_results.len
  pool.broadphase_results.data[idx]

proc worker_output_count*(pool: NarrowphasePool, worker_idx: int): int =
  if worker_idx < 0 or worker_idx >= pool.worker_states.len:
    return 0
  pool.worker_states[worker_idx].output.count

proc worker_output_capacity*(pool: NarrowphasePool, worker_idx: int): int =
  if worker_idx < 0 or worker_idx >= pool.worker_states.len:
    return 0
  pool.worker_states[worker_idx].output.capacity

proc worker_output_at*(pool: NarrowphasePool, worker_idx: int, manifold_idx: int): CollisionManifold =
  doAssert worker_idx >= 0 and worker_idx < pool.worker_states.len
  doAssert manifold_idx >= 0 and manifold_idx < pool.worker_states[worker_idx].output.count
  pool.worker_states[worker_idx].output.data[manifold_idx]

proc total_manifold_count*(pool: NarrowphasePool): int =
  for worker_state in pool.worker_states:
    result += worker_state.output.count

proc worker_job_range*(pool: NarrowphasePool, worker_idx: int): Slice[int] =
  if worker_idx < 0 or worker_idx >= pool.worker_states.len:
    result.a = 0
    result.b = -1
    return

  let job = pool.worker_states[worker_idx].job
  result.a = job.start_idx
  result.b =
    if job.end_idx <= job.start_idx:
      job.start_idx - 1
    else:
      job.end_idx - 1
