import std/bitops
import std/times
import std/monotimes

import physics_math

const
  chunk_width = 64
  chunk_height = 384

type
  ## y-z-x order, lsb is first bit
  ChunkBinaryData = array[chunk_width * chunk_width * chunk_height div sizeof(uint64), uint64]
  ChunkMesh = object
    origin: I3
    bbs: seq[FBB] ## relative to origin

converter i3_to_f3(v: V3[int]): F3 =
  (
    x: float32(v.x), 
    y: float32(v.y), 
    z: float32(v.z),
  )

converter i3_32_to_f3(v: I3): F3 =
  (
    x: float32(v.x), 
    y: float32(v.y), 
    z: float32(v.z),
  )

var chunk_meshes: seq[ChunkMesh]

type PackedRows = UncheckedArray[uint64]

const bits_per_word = sizeof(uint64) * 8

proc run_length_from_first_set_bit(row_bits: uint64, bit_start: int): int {.inline.} =
  let inverted_shifted = not (row_bits shr bit_start)
  if inverted_shifted == 0'u64:
    bits_per_word - bit_start
  else:
    inverted_shifted.count_trailing_zero_bits

proc layer_span_contains_mask(
  solid_rows: ptr PackedRows,
  layer_base: int,
  z_start, z_end: int,
  x_mask: uint64,
): bool {.inline.} =
  var z = z_start
  while z <= z_end:
    if (solid_rows[layer_base + z] and x_mask) != x_mask:
      return false
    inc z
  true

proc clear_box_from_exposed(
  exposed_rows: ptr PackedRows,
  y_start, y_end: int,
  z_start, z_end: int,
  x_mask: uint64,
) {.inline.} =
  var y = y_start
  while y <= y_end:
    let layer_base = y * chunk_width
    var z = z_start
    while z <= z_end:
      exposed_rows[layer_base + z] = exposed_rows[layer_base + z] and (not x_mask)
      inc z
    inc y

proc build_exposed_rows(solid_rows: ptr PackedRows): seq[uint64] =
  result = newSeq[uint64](chunk_height * chunk_width)

  for y in 0 ..< chunk_height:
    let layer_base = y * chunk_width
    let above_layer_base = layer_base - chunk_width
    let below_layer_base = layer_base + chunk_width

    for z in 0 ..< chunk_width:
      let row_index = layer_base + z
      let row_bits = solid_rows[row_index]
      if row_bits == 0'u64:
        continue

      let row_above =
        if y > 0: solid_rows[above_layer_base + z]
        else: 0'u64

      let row_below =
        if y + 1 < chunk_height: solid_rows[below_layer_base + z]
        else: 0'u64

      let row_before =
        if z > 0: solid_rows[row_index - 1]
        else: 0'u64

      let row_after =
        if z + 1 < chunk_width: solid_rows[row_index + 1]
        else: 0'u64

      let exposed_x_neg = row_bits and (not (row_bits shl 1))
      let exposed_x_pos = row_bits and (not (row_bits shr 1))
      let exposed_y_neg = row_bits and (not row_above)
      let exposed_y_pos = row_bits and (not row_below)
      let exposed_z_neg = row_bits and (not row_before)
      let exposed_z_pos = row_bits and (not row_after)

      result[row_index] =
        exposed_x_neg or exposed_x_pos or
        exposed_y_neg or exposed_y_pos or
        exposed_z_neg or exposed_z_pos

## returns index of chunk mesh in chunk_meshes
proc greedy_mesh(
  origin_x, origin_y, origin_z: cint;
  chunk_binary_data: ptr ChunkBinaryData,
): cint {.cdecl, exportc, dynlib.} =
  let start = get_mono_time()

  var bbs: seq[FBB]

  let solid_rows = cast[ptr PackedRows](chunk_binary_data)
  var exposed_storage = build_exposed_rows(solid_rows)
  let exposed_rows = cast[ptr PackedRows](addr exposed_storage[0])

  for y_start in 0 ..< chunk_height:
    let layer_base = y_start * chunk_width

    for z_start in 0 ..< chunk_width:
      let seed_row_index = layer_base + z_start
      var seed_row_bits = exposed_rows[seed_row_index]

      while seed_row_bits != 0'u64:
        let x_start = seed_row_bits.count_trailing_zero_bits
        let x_length = run_length_from_first_set_bit(seed_row_bits, x_start)
        let x_mask = (high(uint64) shr (bits_per_word - x_length)) shl x_start

        var z_end = z_start
        while z_end + 1 < chunk_width and
              (solid_rows[layer_base + z_end + 1] and x_mask) == x_mask:
          inc z_end

        var y_end = y_start
        var next_layer_base = layer_base + chunk_width
        while y_end + 1 < chunk_height and
              layer_span_contains_mask(solid_rows, next_layer_base, z_start, z_end, x_mask):
          inc y_end
          next_layer_base += chunk_width

        clear_box_from_exposed(exposed_rows, y_start, y_end, z_start, z_end, x_mask)

        let bb: FBB = (min: (x_start, y_start, z_start), max: (x_start + x_length, y_end + 1, z_end + 1))
        bbs.add bb

        seed_row_bits = exposed_rows[seed_row_index]

  let idx = chunk_meshes.len
  let origin = (origin_x.int32, origin_y.int32, origin_z.int32)
  let chunk_mesh = ChunkMesh(
    origin: origin,
    bbs: move(bbs),
  )

  chunk_meshes.add chunk_mesh

  let finish = get_mono_time()

  echo "took=", in_milliseconds(finish - start), "ms"

  result = idx.cint

proc num_bbs(chunk_mesh_index: cint): cint {.cdecl, exportc, dynlib.} =
  if chunk_mesh_index >= chunk_meshes.len:
    return -1
  result = chunk_meshes[chunk_mesh_index].bbs.len.cint

proc get_bb(chunk_mesh_index: cint, bb_index: cint): C_FBB {.cdecl, exportc, dynlib.} =
  let bb = chunk_meshes[chunk_mesh_index].bbs[bb_index]
  let origin: F3 = chunk_meshes[chunk_mesh_index].origin
  result.min_x = origin.x + bb.min.x
  result.min_y = origin.y + bb.min.y
  result.min_z = origin.z + bb.min.z
  result.max_x = origin.x + bb.max.x
  result.max_y = origin.y + bb.max.y
  result.max_z = origin.z + bb.max.z
