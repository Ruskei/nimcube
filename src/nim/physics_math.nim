import std/math

type
  V3*[T] = tuple[x, y, z: T]
  F3* = V3[float32]
  D3* = V3[float64]
  I3* = V3[int32]
  BB*[T] = tuple[min, max: V3[T]]
  Q*[T] = tuple[x, y, z, w: T]
  QF* = Q[float32]
  FBB* = BB[float32]
  C_D3* {.bycopy.} = object
    x*, y*, z*: float64
  C_F3* {.bycopy.} = object
    x*, y*, z*: float32
  C_QF* {.bycopy.} = object
    x*, y*, z*, w*: float32
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

converter f3_to_c*(v: F3): C_F3 =
  result.x = v.x
  result.y = v.y
  result.z = v.z

proc `+`*[T](a: V3[T], b: V3[T]): V3[T] =
  result.x = a.x + b.x
  result.y = a.y + b.y
  result.z = a.z + b.z

proc `+=`*[T](a: var V3[T], b: V3[T]) =
  a.x += b.x
  a.y += b.y
  a.z += b.z

proc `*`*[T](a: V3[T], b: T): V3[T] =
  result.x = a.x * b
  result.y = a.y * b
  result.z = a.z * b

proc `*`*[T](a: T, b: V3[T]): V3[T] =
  b * a

proc `+`*[T](a: Q[T], b: Q[T]): Q[T] =
  result.x = a.x + b.x
  result.y = a.y + b.y
  result.z = a.z + b.z
  result.w = a.w + b.w

proc `+=`*[T](a: var Q[T], b: Q[T]) =
  a.x += b.x
  a.y += b.y
  a.z += b.z
  a.w += b.w

proc `*`*[T](a: Q[T], b: T): Q[T] =
  result.x = a.x * b
  result.y = a.y * b
  result.z = a.z * b
  result.w = a.w * b

proc `*`*[T](a: T, b: Q[T]): Q[T] =
  b * a

proc `*`*[T](a: Q[T], b: Q[T]): Q[T] =
  result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
  result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
  result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
  result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z

proc quat*[T](v: V3[T], w: T): Q[T] =
  result.x = v.x
  result.y = v.y
  result.z = v.z
  result.w = w

proc quat_identity*[T](t: typedesc[T]): Q[T] =
  result.x = T(0)
  result.y = T(0)
  result.z = T(0)
  result.w = T(1)

proc dot*[T: SomeFloat](a: Q[T], b: Q[T]): T =
  a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w

proc norm_squared*[T: SomeFloat](q: Q[T]): T =
  dot(q, q)

proc normalized*[T: SomeFloat](q: Q[T]): Q[T] =
  let n2 = norm_squared(q)
  if n2 <= T(0):
    return quat_identity(T)
  let invNorm = T(1) / sqrt(n2)
  q * invNorm

converter qf_to_c*(q: QF): C_QF =
  result.x = q.x
  result.y = q.y
  result.z = q.z
  result.w = q.w
