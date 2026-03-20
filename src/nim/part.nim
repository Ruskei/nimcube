import std/algorithm
from std/math import arctan2

import physics_math
import portal

const
  slice_axis_epsilon = 1.0e-6'f32
  slice_clip_epsilon = 1.0e-5'f32
  slice_point_merge_epsilon = 1.0e-4'f32
  slice_area_epsilon = 1.0e-6'f32

type
  Face* = seq[int]
  MeshObj* = object
    vertices*: seq[F3]
    faces*: seq[Face]
  PartObj* = object
    translation_record: F3
    quat_record: QF
    mesh*: MeshObj
  CompositionKind* = enum
    ck_simple, ck_parted
  CompositionObj* = object
    case kind*: CompositionKind
    of ck_simple: discard
    of ck_parted:
      parts*: seq[PartObj]

proc signed_distance_to_plane(point, plane_origin, plane_normal: F3): float32
proc point_is_inside(distance: float32, keep_positive: bool): bool

proc mesh_center*(mesh: MeshObj): F3 =
  if mesh.vertices.len == 0:
    return default(F3)

  for vertex in mesh.vertices:
    result += vertex

  result = result / mesh.vertices.len.float32

proc translated*(mesh: MeshObj, translation: F3): MeshObj =
  result.faces = mesh.faces
  result.vertices = newSeqOfCap[F3](mesh.vertices.len)

  for vertex in mesh.vertices:
    result.vertices.add vertex + translation

proc rotated*(mesh: MeshObj, quat: QF): MeshObj =
  result.faces = mesh.faces
  result.vertices = newSeqOfCap[F3](mesh.vertices.len)

  let normalized_quat = quat.normalized
  for vertex in mesh.vertices:
    result.vertices.add rotate_vector(normalized_quat, vertex)

proc point_is_on_plane_side*(point, plane_origin, plane_normal: F3, keep_positive: bool): bool =
  let normalized_plane_normal = plane_normal.normalized
  if not normalized_plane_normal.is_finite:
    return false
  if normalized_plane_normal.length_squared <= slice_axis_epsilon * slice_axis_epsilon:
    return false

  let signed_distance = signed_distance_to_plane(point, plane_origin, normalized_plane_normal)
  point_is_inside(signed_distance, keep_positive)

proc signed_distance_to_plane(point, plane_origin, plane_normal: F3): float32 =
  plane_normal ∙ (point - plane_origin)

proc collision_axis_is_usable(axis: F3): bool =
  axis.is_finite and axis.length_squared > slice_axis_epsilon * slice_axis_epsilon

proc points_are_close(a, b: F3, epsilon = slice_point_merge_epsilon): bool =
  let delta = a - b
  delta.length_squared <= epsilon * epsilon

proc append_unique_point(points: var seq[F3], point: F3) =
  for existing_point in points:
    if points_are_close(existing_point, point):
      return
  points.add point

proc append_unique_edge(edges: var seq[tuple[a, b: int]], a, b: int) =
  let edge_a = min(a, b)
  let edge_b = max(a, b)

  for edge in edges:
    if edge.a == edge_a and edge.b == edge_b:
      return

  edges.add (a: edge_a, b: edge_b)

proc collect_unique_edges(mesh: MeshObj): seq[tuple[a, b: int]] =
  for face in mesh.faces:
    if face.len < 2:
      continue

    for idx in 0 ..< face.len:
      let next_idx = (idx + 1) mod face.len
      result.append_unique_edge(face[idx], face[next_idx])

proc face_normal(mesh: MeshObj, face: Face): F3 =
  if face.len < 3:
    return default(F3)

  let origin = mesh.vertices[face[0]]
  for idx in 1 ..< face.len - 1:
    result += (mesh.vertices[face[idx]] - origin) × (mesh.vertices[face[idx + 1]] - origin)

proc project_mesh(mesh: MeshObj, axis: F3): tuple[min, max: float32] =
  result.min = mesh.vertices[0] ∙ axis
  result.max = result.min

  for idx in 1 ..< mesh.vertices.len:
    let projection = mesh.vertices[idx] ∙ axis
    if projection < result.min: result.min = projection
    if projection > result.max: result.max = projection

proc portal_interval(center: F3, axes: array[3, F3], half_extents: F3, axis: F3): tuple[min, max: float32] =
  let radius =
    abs(axes[0] ∙ axis) * half_extents.x +
    abs(axes[1] ∙ axis) * half_extents.y +
    abs(axes[2] ∙ axis) * half_extents.z
  let center_projection = center ∙ axis
  (min: center_projection - radius, max: center_projection + radius)

proc axis_separates_mesh_and_portal(
  mesh: MeshObj,
  portal_center: F3,
  portal_axes: array[3, F3],
  portal_half_extents: F3,
  axis: F3,
): bool =
  let normalized_axis = axis.normalized
  if not collision_axis_is_usable(normalized_axis):
    return false

  let mesh_projection = project_mesh(mesh, normalized_axis)
  let portal_projection = portal_interval(portal_center, portal_axes, portal_half_extents, normalized_axis)
  mesh_projection.max < portal_projection.min or portal_projection.max < mesh_projection.min

proc mesh_collides_with_portal_side*(
  mesh: MeshObj,
  portal_center: F3,
  portal_quat: QF,
  scale_x, scale_y: float32,
): bool =
  if mesh.vertices.len == 0 or mesh.faces.len == 0 or scale_x <= 0'f32 or scale_y <= 0'f32:
    return false

  let normalized_quat = portal_quat.normalized
  let portal_axes: array[3, F3] = [
    rotate_vector(normalized_quat, (1'f32, 0'f32, 0'f32)).normalized,
    rotate_vector(normalized_quat, (0'f32, 1'f32, 0'f32)).normalized,
    rotate_vector(normalized_quat, (0'f32, 0'f32, 1'f32)).normalized,
  ]
  let portal_half_extents: F3 = (scale_x * 0.5'f32, scale_y * 0.5'f32, 0'f32)

  for axis in portal_axes:
    if not collision_axis_is_usable(axis):
      return false
    if axis_separates_mesh_and_portal(mesh, portal_center, portal_axes, portal_half_extents, axis):
      return false

  for face in mesh.faces:
    let axis = face_normal(mesh, face)
    if axis_separates_mesh_and_portal(mesh, portal_center, portal_axes, portal_half_extents, axis):
      return false

  let edges = collect_unique_edges(mesh)
  for edge in edges:
    let edge_axis = mesh.vertices[edge.b] - mesh.vertices[edge.a]
    if not collision_axis_is_usable(edge_axis):
      continue

    for portal_axis in portal_axes:
      let cross_axis = edge_axis × portal_axis
      if axis_separates_mesh_and_portal(mesh, portal_center, portal_axes, portal_half_extents, cross_axis):
        return false

  true

proc segment_plane_intersection(
  start_point,
  end_point: F3,
  start_distance,
  end_distance: float32,
): F3 =
  let denominator = start_distance - end_distance
  if abs(denominator) <= slice_axis_epsilon:
    return start_point

  let t = clamp(start_distance / denominator, 0'f32, 1'f32)
  lerp(start_point, end_point, t)

proc point_is_inside(distance: float32, keep_positive: bool): bool =
  if keep_positive:
    distance >= -slice_clip_epsilon
  else:
    distance <= slice_clip_epsilon

proc compact_polygon_points(points: seq[F3]): seq[F3] =
  for point in points:
    if result.len == 0 or not points_are_close(result[^1], point):
      result.add point

  if result.len > 1 and points_are_close(result[0], result[^1]):
    result.setLen(result.len - 1)

proc polygon_area_vector(points: seq[F3]): F3 =
  if points.len < 3:
    return default(F3)

  let origin = points[0]
  for idx in 1 ..< points.len - 1:
    result += (points[idx] - origin) × (points[idx + 1] - origin)

proc polygon_has_area(points: seq[F3]): bool =
  polygon_area_vector(points).length_squared > slice_area_epsilon * slice_area_epsilon

proc clip_face_against_half_space(
  mesh: MeshObj,
  face: Face,
  signed_distances: seq[float32],
  keep_positive: bool,
): seq[F3] =
  if face.len == 0:
    return

  let previous_index = face[^1]
  var previous_point = mesh.vertices[previous_index]
  var previous_distance = signed_distances[previous_index]
  var previous_inside = point_is_inside(previous_distance, keep_positive)

  for current_index in face:
    let current_point = mesh.vertices[current_index]
    let current_distance = signed_distances[current_index]
    let current_inside = point_is_inside(current_distance, keep_positive)

    if current_inside != previous_inside:
      result.add segment_plane_intersection(
        previous_point,
        current_point,
        previous_distance,
        current_distance,
      )

    if current_inside:
      result.add current_point

    previous_point = current_point
    previous_distance = current_distance
    previous_inside = current_inside

  result = compact_polygon_points(result)

proc plane_basis(plane_normal: F3): tuple[u, v: F3] =
  let basis_seed =
    if abs(plane_normal.x) < 0.57735'f32:
      (1'f32, 0'f32, 0'f32)
    elif abs(plane_normal.y) < 0.57735'f32:
      (0'f32, 1'f32, 0'f32)
    else:
      (0'f32, 0'f32, 1'f32)

  result.u = normalized(basis_seed × plane_normal)
  result.v = normalized(plane_normal × result.u)

proc reversed_points(points: seq[F3]): seq[F3] =
  if points.len == 0:
    return @[]

  result = newSeqOfCap[F3](points.len)
  for idx in countdown(points.high, 0):
    result.add points[idx]

proc order_cut_polygon(cut_points: seq[F3], plane_normal: F3): seq[F3] =
  if cut_points.len == 0:
    return @[]

  var centroid = default(F3)
  for point in cut_points:
    centroid += point
  centroid = centroid / cut_points.len.float32

  let basis = plane_basis(plane_normal)
  var ordered_points = newSeqOfCap[tuple[point: F3, angle: float32]](cut_points.len)

  for point in cut_points:
    let relative = point - centroid
    ordered_points.add (
      point: point,
      angle: arctan2(relative ∙ basis.v, relative ∙ basis.u),
    )

  ordered_points.sort(
    proc (a, b: tuple[point: F3, angle: float32]): int =
      cmp(a.angle, b.angle)
  )

  result = newSeqOfCap[F3](ordered_points.len)
  for ordered_point in ordered_points:
    result.add ordered_point.point

  if (polygon_area_vector(result) ∙ plane_normal) < 0'f32:
    result = reversed_points(result)

proc collect_cut_points(mesh: MeshObj, signed_distances: seq[float32]): seq[F3] =
  let edges = collect_unique_edges(mesh)

  for edge in edges:
    let a_distance = signed_distances[edge.a]
    let b_distance = signed_distances[edge.b]
    let a_positive = a_distance > slice_clip_epsilon
    let a_negative = a_distance < -slice_clip_epsilon
    let b_positive = b_distance > slice_clip_epsilon
    let b_negative = b_distance < -slice_clip_epsilon

    if (a_positive and b_negative) or (a_negative and b_positive):
      result.append_unique_point segment_plane_intersection(
        mesh.vertices[edge.a],
        mesh.vertices[edge.b],
        a_distance,
        b_distance,
      )

  for idx, distance in signed_distances:
    if abs(distance) <= slice_clip_epsilon:
      result.append_unique_point mesh.vertices[idx]

proc collect_clipped_polygons(
  mesh: MeshObj,
  signed_distances: seq[float32],
  keep_positive: bool,
): seq[seq[F3]] =
  for face in mesh.faces:
    let clipped_face = clip_face_against_half_space(mesh, face, signed_distances, keep_positive)
    if clipped_face.len >= 3 and polygon_has_area(clipped_face):
      result.add clipped_face

proc find_or_add_vertex(vertices: var seq[F3], point: F3): int =
  for idx, existing_point in vertices:
    if points_are_close(existing_point, point):
      return idx

  result = vertices.len
  vertices.add point

proc remap_polygon(vertices: seq[F3], polygon: seq[F3]): tuple[vertices: seq[F3], face: Face] =
  result.vertices = vertices

  for point in polygon:
    let vertex_index = find_or_add_vertex(result.vertices, point)
    if result.face.len == 0 or result.face[^1] != vertex_index:
      result.face.add vertex_index

  if result.face.len > 1 and result.face[0] == result.face[^1]:
    result.face.setLen(result.face.len - 1)

proc count_unique_indices(face: Face): int =
  var unique_indices: seq[int] = @[]

  for vertex_index in face:
    var already_seen = false
    for unique_index in unique_indices:
      if unique_index == vertex_index:
        already_seen = true
        break

    if not already_seen:
      unique_indices.add vertex_index

  unique_indices.len

proc build_mesh_from_polygons(polygons: seq[seq[F3]]): MeshObj =
  for polygon in polygons:
    let compact_polygon = compact_polygon_points(polygon)
    if compact_polygon.len < 3 or not polygon_has_area(compact_polygon):
      continue

    let remapped_polygon = remap_polygon(result.vertices, compact_polygon)
    if remapped_polygon.face.len < 3:
      continue
    if count_unique_indices(remapped_polygon.face) != remapped_polygon.face.len:
      continue

    result.vertices = remapped_polygon.vertices
    result.faces.add remapped_polygon.face

proc slice_mesh_by_plane*(mesh: MeshObj, plane_origin, plane_normal: F3): seq[MeshObj] =
  if mesh.vertices.len == 0 or mesh.faces.len == 0:
    return @[mesh]

  let normalized_plane_normal = plane_normal.normalized
  if not normalized_plane_normal.is_finite:
    return @[mesh]
  if normalized_plane_normal.length_squared <= slice_axis_epsilon * slice_axis_epsilon:
    return @[mesh]

  var signed_distances = newSeqOfCap[float32](mesh.vertices.len)
  var all_positive_side = true
  var all_negative_side = true

  for vertex in mesh.vertices:
    let signed_distance = signed_distance_to_plane(vertex, plane_origin, normalized_plane_normal)
    signed_distances.add signed_distance

    if signed_distance < -slice_clip_epsilon:
      all_positive_side = false
    if signed_distance > slice_clip_epsilon:
      all_negative_side = false

  if all_positive_side or all_negative_side:
    return @[mesh]

  let cut_points = collect_cut_points(mesh, signed_distances)
  if cut_points.len < 3:
    return @[mesh]

  let cut_polygon = order_cut_polygon(cut_points, normalized_plane_normal)
  if cut_polygon.len < 3 or not polygon_has_area(cut_polygon):
    return @[mesh]

  var positive_polygons = collect_clipped_polygons(mesh, signed_distances, keep_positive = true)
  var negative_polygons = collect_clipped_polygons(mesh, signed_distances, keep_positive = false)

  positive_polygons.add reversed_points(cut_polygon)
  negative_polygons.add cut_polygon

  let positive_mesh = build_mesh_from_polygons(positive_polygons)
  let negative_mesh = build_mesh_from_polygons(negative_polygons)

  if positive_mesh.faces.len > 0 and positive_mesh.vertices.len > 0:
    result.add positive_mesh
  if negative_mesh.faces.len > 0 and negative_mesh.vertices.len > 0:
    result.add negative_mesh

  if result.len == 0:
    return @[mesh]

proc init_cuboid_part*(position: F3, quat: QF, dimensions: F3): PartObj =
  let half_extents = dimensions * 0.5'f32
  result = PartObj(
    translation_record: default(F3),
    quat_record: default(QF),
    mesh: MeshObj(
      vertices: @[
        position + quat.rotate_vector((x: -half_extents.x, y: -half_extents.y, z: -half_extents.z)),
        position + quat.rotate_vector((x:  half_extents.x, y: -half_extents.y, z: -half_extents.z)),
        position + quat.rotate_vector((x: -half_extents.x, y:  half_extents.y, z: -half_extents.z)),
        position + quat.rotate_vector((x:  half_extents.x, y:  half_extents.y, z: -half_extents.z)),
        position + quat.rotate_vector((x: -half_extents.x, y: -half_extents.y, z:  half_extents.z)),
        position + quat.rotate_vector((x:  half_extents.x, y: -half_extents.y, z:  half_extents.z)),
        position + quat.rotate_vector((x: -half_extents.x, y:  half_extents.y, z:  half_extents.z)),
        position + quat.rotate_vector((x:  half_extents.x, y:  half_extents.y, z:  half_extents.z)),
      ],
      faces: @[
        @[0, 2, 3, 1],
        @[4, 5, 7, 6],
        @[0, 1, 5, 4],
        @[2, 6, 7, 3],
        @[0, 4, 6, 2],
        @[1, 3, 7, 5],
      ],
    ),
  )

proc aabb*(part: PartObj): FBB =
  if part.mesh.vertices.len == 0:
    return default(FBB)

  result = (min: part.mesh.vertices[0], max: part.mesh.vertices[0])

  for i in 1 ..< part.mesh.vertices.len:
    let corner = part.mesh.vertices[i]
    if corner.x < result.min.x: result.min.x = corner.x
    if corner.y < result.min.y: result.min.y = corner.y
    if corner.z < result.min.z: result.min.z = corner.z
    if corner.x > result.max.x: result.max.x = corner.x
    if corner.y > result.max.y: result.max.y = corner.y
    if corner.z > result.max.z: result.max.z = corner.z
