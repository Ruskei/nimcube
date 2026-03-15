type
  V3*[T] = tuple[x, y, z: T]
  F3* = V3[float32]
  D3* = V3[float64]
  I3* = V3[int32]
  BB*[T] = tuple[min, max: V3[T]]
  FBB* = BB[float32]
  C_F3* {.bycopy.} = object
    x*, y*, z*: float32
  C_FBB* {.bycopy.} = object
    min_x*, min_y*, min_z*, max_x*, max_y*, max_z*: float32

converter f3_to_c*(v: F3): C_F3 =
  result.x = v.x
  result.y = v.y
  result.z = v.z

converter f3_from_c*(v: C_F3): F3 =
  result.x = v.x
  result.y = v.y
  result.z = v.z
