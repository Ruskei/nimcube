import part
import physics_math

const
  test_epsilon = 1.0e-4'f32

proc approx_equal(a, b: float32, epsilon = test_epsilon): bool =
  abs(a - b) <= epsilon

proc make_cube_mesh(): MeshObj =
  MeshObj(
    vertices: @[
      (x: -1'f32, y: -1'f32, z: -1'f32),
      (x:  1'f32, y: -1'f32, z: -1'f32),
      (x: -1'f32, y:  1'f32, z: -1'f32),
      (x:  1'f32, y:  1'f32, z: -1'f32),
      (x: -1'f32, y: -1'f32, z:  1'f32),
      (x:  1'f32, y: -1'f32, z:  1'f32),
      (x: -1'f32, y:  1'f32, z:  1'f32),
      (x:  1'f32, y:  1'f32, z:  1'f32),
    ],
    faces: @[
      @[0, 2, 3, 1],
      @[4, 5, 7, 6],
      @[0, 1, 5, 4],
      @[2, 6, 7, 3],
      @[0, 4, 6, 2],
      @[1, 3, 7, 5],
    ],
  )

proc signed_distance_to_plane(point, plane_origin, plane_normal: F3): float32 =
  plane_normal ∙ (point - plane_origin)

proc point_is_on_plane(point, plane_origin, plane_normal: F3, epsilon = test_epsilon): bool =
  approx_equal(signed_distance_to_plane(point, plane_origin, plane_normal), 0'f32, epsilon)

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

proc polygon_area_measure(points: seq[F3]): float32 =
  if points.len < 3:
    return 0'f32

  let origin = points[0]
  var area_vector = default(F3)

  for idx in 1 ..< points.len - 1:
    area_vector += (points[idx] - origin) × (points[idx + 1] - origin)

  area_vector.length

proc validate_mesh(mesh: MeshObj) =
  for vertex in mesh.vertices:
    doAssert vertex.is_finite

  for face in mesh.faces:
    doAssert face.len >= 3
    doAssert count_unique_indices(face) == face.len

    var face_points: seq[F3] = @[]
    for vertex_index in face:
      doAssert vertex_index >= 0
      doAssert vertex_index < mesh.vertices.len
      face_points.add mesh.vertices[vertex_index]

    doAssert polygon_area_measure(face_points) > test_epsilon

proc all_vertices_on_expected_side(
  mesh: MeshObj,
  plane_origin,
  plane_normal: F3,
  expect_positive: bool,
): bool =
  for vertex in mesh.vertices:
    let signed_distance = signed_distance_to_plane(vertex, plane_origin, plane_normal)
    if expect_positive:
      if signed_distance < -test_epsilon:
        return false
    else:
      if signed_distance > test_epsilon:
        return false

  true

proc find_cap_face_vertex_count(mesh: MeshObj, plane_origin, plane_normal: F3): int =
  for face in mesh.faces:
    var all_points_on_plane = true
    for vertex_index in face:
      if not point_is_on_plane(mesh.vertices[vertex_index], plane_origin, plane_normal):
        all_points_on_plane = false
        break

    if all_points_on_plane:
      return face.len

  0

proc test_no_split_positive_side() =
  let cube = make_cube_mesh()
  let plane_origin: F3 = (-2'f32, 0'f32, 0'f32)
  let plane_normal: F3 = (1'f32, 0'f32, 0'f32)
  let slices = slice_mesh_by_plane(cube, plane_origin, plane_normal)

  doAssert slices == @[cube]

proc test_no_split_negative_side() =
  let cube = make_cube_mesh()
  let plane_origin: F3 = (2'f32, 0'f32, 0'f32)
  let plane_normal: F3 = (1'f32, 0'f32, 0'f32)
  let slices = slice_mesh_by_plane(cube, plane_origin, plane_normal)

  doAssert slices == @[cube]

proc test_center_cut_on_x_plane() =
  let cube = make_cube_mesh()
  let plane_origin: F3 = (0'f32, 0'f32, 0'f32)
  let plane_normal: F3 = (1'f32, 0'f32, 0'f32)
  let slices = slice_mesh_by_plane(cube, plane_origin, plane_normal)

  doAssert slices.len == 2
  doAssert all_vertices_on_expected_side(slices[0], plane_origin, plane_normal, expect_positive = true)
  doAssert all_vertices_on_expected_side(slices[1], plane_origin, plane_normal, expect_positive = false)
  doAssert find_cap_face_vertex_count(slices[0], plane_origin, plane_normal) == 4
  doAssert find_cap_face_vertex_count(slices[1], plane_origin, plane_normal) == 4

  validate_mesh(slices[0])
  validate_mesh(slices[1])

proc test_plane_through_vertices_and_edges() =
  let cube = make_cube_mesh()
  let plane_origin: F3 = (0'f32, 0'f32, 0'f32)
  let plane_normal: F3 = normalized((1'f32, 1'f32, 0'f32))
  let slices = slice_mesh_by_plane(cube, plane_origin, plane_normal)

  doAssert slices.len == 2
  doAssert all_vertices_on_expected_side(slices[0], plane_origin, plane_normal, expect_positive = true)
  doAssert all_vertices_on_expected_side(slices[1], plane_origin, plane_normal, expect_positive = false)
  doAssert find_cap_face_vertex_count(slices[0], plane_origin, plane_normal) == 4
  doAssert find_cap_face_vertex_count(slices[1], plane_origin, plane_normal) == 4

  validate_mesh(slices[0])
  validate_mesh(slices[1])

proc test_tangential_plane_coincident_with_face() =
  let cube = make_cube_mesh()
  let plane_origin: F3 = (1'f32, 0'f32, 0'f32)
  let plane_normal: F3 = (1'f32, 0'f32, 0'f32)
  let slices = slice_mesh_by_plane(cube, plane_origin, plane_normal)

  doAssert slices == @[cube]

proc test_invalid_plane_normal_returns_original() =
  let cube = make_cube_mesh()
  let plane_origin: F3 = (0'f32, 0'f32, 0'f32)
  let plane_normal: F3 = (0'f32, 0'f32, 0'f32)
  let slices = slice_mesh_by_plane(cube, plane_origin, plane_normal)

  doAssert slices == @[cube]

when is_main_module:
  test_no_split_positive_side()
  test_no_split_negative_side()
  test_center_cut_on_x_plane()
  test_plane_through_vertices_and_edges()
  test_tangential_plane_coincident_with_face()
  test_invalid_plane_normal_returns_original()
  echo "part tests passed"
