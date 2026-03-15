type
  V3*[T] = tuple[x, y, z: T]
  F3* = V3[float32]
  D3* = V3[float64]
  I3* = V3[int32]
  BB*[T] = tuple[min, max: V3[T]]
  Q[T] = tuple[x, y, z, w: T]
  QF = Q[float32]
  FBB* = BB[float32]
  C_D3* {.bycopy.} = object
    x*, y*, z*: float64
  C_FBB* {.bycopy.} = object
    min_x*, min_y*, min_z*, max_x*, max_y*, max_z*: float32

converter d3_to_c*(v: D3): C_D3 =
  result.x = v.x
  result.y = v.y
  result.z = v.z

converter d3_from_c*(v: C_D3): D3 =
  result.x = v.x
  result.y = v.y
  result.z = v.z

converter f3_to_d3*(v: F3): D3 =
  result.x = v.x.float64
  result.y = v.y.float64
  result.z = v.z.float64

proc `+`*[T](a: V3[T], b: V3[T]): V3[T] =
  result.x = a.x + b.x
  result.y = a.y + b.y
  result.z = a.z + b.z
