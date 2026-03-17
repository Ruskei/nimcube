import std/hashes

type
  ChunkPosition* = object
    x*, z*: int32

proc hash*(chunk_pos: ChunkPosition): Hash =
  let packed =
    ((chunk_pos.x.uint32 and 0xFFFF'u32) shl 16) or
    (chunk_pos.z.uint32 and 0xFFFF'u32)
  Hash(packed)

proc chunk_position*(chunk_x, chunk_z: int32): ChunkPosition =
  ChunkPosition(x: chunk_x, z: chunk_z)
