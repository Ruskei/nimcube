import physics_math

type
  VertexIndex = int
  Face = seq[VertexIndex]
  Mesh = ref object
    vertices: seq[F3]
    faces: seq[Face]
