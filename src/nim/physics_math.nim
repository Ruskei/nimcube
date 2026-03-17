import std/hashes
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

proc hash*(bb: FBB): Hash =
  var h: Hash = 0
  h = h !& hash(bb.min.x)
  h = h !& hash(bb.min.y)
  h = h !& hash(bb.min.z)
  h = h !& hash(bb.max.x)
  h = h !& hash(bb.max.y)
  h = h !& hash(bb.max.z)
  result = !$h

converter f3_to_d3*(v: F3): D3 =
  result.x = v.x.float64
  result.y = v.y.float64
  result.z = v.z.float64

proc overlaps*[T](a, b: BB[T]): bool =
  a.min.x < b.max.x and a.max.x > b.min.x and
  a.min.y < b.max.y and a.max.y > b.min.y and
  a.min.z < b.max.z and a.max.z > b.min.z

proc is_finite*[T: SomeFloat](x: T): bool =
  classify(x) notin {fcNan, fcInf, fcNegInf}

proc is_finite*[T: SomeFloat](v: V3[T]): bool =
  v.x.is_finite and v.y.is_finite and v.z.is_finite

proc component*[T](v: V3[T], idx: int): T =
  case idx
  of 0:
    v.x
  of 1:
    v.y
  of 2:
    v.z
  else:
    raise new_exception(IndexDefect, "vector component index out of range")

proc `+`*[T](a: V3[T], b: V3[T]): V3[T] =
  result.x = a.x + b.x
  result.y = a.y + b.y
  result.z = a.z + b.z

proc `-`*[T](v: V3[T]): V3[T] =
  result.x = -v.x
  result.y = -v.y
  result.z = -v.z

proc `-`*[T](a: V3[T], b: V3[T]): V3[T] =
  result.x = a.x - b.x
  result.y = a.y - b.y
  result.z = a.z - b.z

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

proc `/`*[T: SomeFloat](a: V3[T], b: T): V3[T] =
  result.x = a.x / b
  result.y = a.y / b
  result.z = a.z / b

proc `∙`*[T: SomeFloat](a, b: V3[T]): T =
  a.x * b.x + a.y * b.y + a.z * b.z

proc `×`*[T: SomeFloat](a, b: V3[T]): V3[T] =
  result.x = a.y * b.z - a.z * b.y
  result.y = a.z * b.x - a.x * b.z
  result.z = a.x * b.y - a.y * b.x

proc length_squared*[T: SomeFloat](v: V3[T]): T =
  v ∙ v

proc length*[T: SomeFloat](v: V3[T]): T =
  sqrt(v.length_squared)

proc normalized*[T: SomeFloat](v: V3[T]): V3[T] =
  let len_sq = v.length_squared
  if len_sq <= T(0) or not len_sq.is_finite:
    return (T(0), T(0), T(0))
  v / sqrt(len_sq)

proc abs_components*[T: SomeFloat](v: V3[T]): V3[T] =
  result.x = abs(v.x)
  result.y = abs(v.y)
  result.z = abs(v.z)

proc lerp*[T: SomeFloat](a, b: V3[T], t: T): V3[T] =
  a + (b - a) * t

proc clamp*[T: SomeFloat](x, lo, hi: T): T =
  if x < lo:
    lo
  elif x > hi:
    hi
  else:
    x

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

proc conjugate*[T](q: Q[T]): Q[T] =
  result.x = -q.x
  result.y = -q.y
  result.z = -q.z
  result.w = q.w

proc `∙`*[T: SomeFloat](a: Q[T], b: Q[T]): T =
  a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w

proc dot*[T: SomeFloat](a: Q[T], b: Q[T]): T =
  a ∙ b

proc norm_squared*[T: SomeFloat](q: Q[T]): T =
  q ∙ q

proc normalized*[T: SomeFloat](q: Q[T]): Q[T] =
  let n2 = norm_squared(q)
  if n2 <= T(0):
    return quat_identity(T)
  let invNorm = T(1) / sqrt(n2)
  q * invNorm

proc rotate_vector*[T: SomeFloat](q: Q[T], v: V3[T]): V3[T] =
  let pure_v = quat(v, T(0))
  let rotated = q * pure_v * conjugate(q)
  result.x = rotated.x
  result.y = rotated.y
  result.z = rotated.z
