import cuboids
import dynamic_aabb_tree
import part
import physics_math

proc read_value[T](buffer: ptr UncheckedArray[uint8], offset: var int): T =
  copyMem(addr result, addr buffer[offset], sizeof(T))
  offset += sizeof(T)

proc read_vertex(buffer: ptr UncheckedArray[uint8], offset: var int): F3 =
  (
    read_value[float32](buffer, offset),
    read_value[float32](buffer, offset),
    read_value[float32](buffer, offset),
  )

proc test_simple_composition_serializes_empty_mesh_list() =
  var data = InternalData()
  var tree = init_dynamic_aabb_tree[BodyHandle]()
  var external: ExternalData
  init_external_data(external)

  discard data.create_cuboid(
    aabb_tree = tree,
    initial_pos = (0'f64, 0'f64, 0'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    rot = quat_identity(float32),
    dimensions = (2'f32, 2'f32, 2'f32),
    inverse_mass = 1'f32,
  )
  data.update_external_data(external)

  doAssert external.body_count == 1
  doAssert external.mesh[0].data_len == sizeof(int32).int32

  var offset = 0
  doAssert read_value[int32](external.mesh[0].data, offset) == 0'i32
  doAssert offset == external.mesh[0].data_len.int

  deinit_external_data(external)

proc test_parted_composition_serializes_expected_layout() =
  var data = InternalData()
  var tree = init_dynamic_aabb_tree[BodyHandle]()
  var external: ExternalData
  init_external_data(external)

  let handle = data.create_cuboid(
    aabb_tree = tree,
    initial_pos = (0'f64, 0'f64, 0'f64),
    vel = (0'f32, 0'f32, 0'f32),
    ω = (0'f32, 0'f32, 0'f32),
    rot = quat_identity(float32),
    dimensions = (2'f32, 2'f32, 2'f32),
    inverse_mass = 1'f32,
  )
  data.set_composition(handle, CompositionObj(
    kind: ck_parted,
    parts: @[
      PartObj(mesh: MeshObj(
        vertices: @[
          (1'f32, 2'f32, 3'f32),
          (4'f32, 5'f32, 6'f32),
          (7'f32, 8'f32, 9'f32),
        ],
        faces: @[
          @[2, 0, 1],
        ],
      )),
    ],
  ))
  data.update_external_data(external)

  doAssert external.body_count == 1
  doAssert external.mesh[0].data_len == (7 * sizeof(int32) + 9 * sizeof(float32)).int32

  var offset = 0
  doAssert read_value[int32](external.mesh[0].data, offset) == 1'i32
  doAssert read_value[int32](external.mesh[0].data, offset) == 3'i32
  doAssert read_vertex(external.mesh[0].data, offset) == (1'f32, 2'f32, 3'f32)
  doAssert read_vertex(external.mesh[0].data, offset) == (4'f32, 5'f32, 6'f32)
  doAssert read_vertex(external.mesh[0].data, offset) == (7'f32, 8'f32, 9'f32)
  doAssert read_value[int32](external.mesh[0].data, offset) == 1'i32
  doAssert read_value[int32](external.mesh[0].data, offset) == 3'i32
  doAssert read_value[int32](external.mesh[0].data, offset) == 2'i32
  doAssert read_value[int32](external.mesh[0].data, offset) == 0'i32
  doAssert read_value[int32](external.mesh[0].data, offset) == 1'i32
  doAssert offset == external.mesh[0].data_len.int

  deinit_external_data(external)

when is_main_module:
  test_simple_composition_serializes_empty_mesh_list()
  test_parted_composition_serializes_expected_layout()
  echo "cuboids external mesh tests passed"
