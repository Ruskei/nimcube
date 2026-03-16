import std/heapqueue

import physics_math

type
  NodeIndex* = int

const
  invalid_node_index* = NodeIndex(-1)

type
  NodeStorageKind* = enum
    nsk_free, nsk_used

  DynamicAabbNode*[T] = object
    case storage*: NodeStorageKind
    of nsk_free:
      next_free_idx*: NodeIndex
    of nsk_used:
      parent_idx*: NodeIndex
      is_leaf*: bool
      child_1_idx*: NodeIndex
      child_2_idx*: NodeIndex
      min_x*: float32
      min_y*: float32
      min_z*: float32
      max_x*: float32
      max_y*: float32
      max_z*: float32
      explored_cost*: float32
      data*: T

  DynamicAabbTree*[T] = ref object
    nodes*: seq[DynamicAabbNode[T]]
    root_idx*: NodeIndex
    free_idx*: NodeIndex
    leaf_count*: int
    fat_margin*: float32

  Aabb = object
    min_x: float32
    min_y: float32
    min_z: float32
    max_x: float32
    max_y: float32
    max_z: float32

proc init_aabb(
  min_x, min_y, min_z,
  max_x, max_y, max_z: float32,
): Aabb =
  result.min_x = min_x
  result.min_y = min_y
  result.min_z = min_z
  result.max_x = max_x
  result.max_y = max_y
  result.max_z = max_z

proc to_aabb(bb: FBB): Aabb =
  init_aabb(bb.min.x, bb.min.y, bb.min.z, bb.max.x, bb.max.y, bb.max.z)

proc to_fbb(box: Aabb): FBB =
  result.min = (box.min_x, box.min_y, box.min_z)
  result.max = (box.max_x, box.max_y, box.max_z)

proc union_aabb(a, b: Aabb): Aabb =
  result.min_x = min(a.min_x, b.min_x)
  result.min_y = min(a.min_y, b.min_y)
  result.min_z = min(a.min_z, b.min_z)
  result.max_x = max(a.max_x, b.max_x)
  result.max_y = max(a.max_y, b.max_y)
  result.max_z = max(a.max_z, b.max_z)

proc overlaps(a, b: Aabb): bool =
  a.min_x < b.max_x and a.max_x > b.min_x and
  a.min_y < b.max_y and a.max_y > b.min_y and
  a.min_z < b.max_z and a.max_z > b.min_z

proc contains(outer_box, inner_box: Aabb): bool =
  outer_box.min_x <= inner_box.min_x and outer_box.max_x >= inner_box.max_x and
  outer_box.min_y <= inner_box.min_y and outer_box.max_y >= inner_box.max_y and
  outer_box.min_z <= inner_box.min_z and outer_box.max_z >= inner_box.max_z

proc surface_area(box: Aabb): float32 =
  let dx = max(box.max_x - box.min_x, 0'f32)
  let dy = max(box.max_y - box.min_y, 0'f32)
  let dz = max(box.max_z - box.min_z, 0'f32)
  2'f32 * ((dx * dy) + (dy * dz) + (dz * dx))

proc fatten_aabb(box: Aabb, fat_margin: float32, displacement: F3 = (0'f32, 0'f32, 0'f32)): Aabb =
  result.min_x = box.min_x - fat_margin
  result.min_y = box.min_y - fat_margin
  result.min_z = box.min_z - fat_margin
  result.max_x = box.max_x + fat_margin
  result.max_y = box.max_y + fat_margin
  result.max_z = box.max_z + fat_margin

  if displacement.x < 0'f32:
    result.min_x += displacement.x
  else:
    result.max_x += displacement.x

  if displacement.y < 0'f32:
    result.min_y += displacement.y
  else:
    result.max_y += displacement.y

  if displacement.z < 0'f32:
    result.min_z += displacement.z
  else:
    result.max_z += displacement.z

proc is_valid_index[T](tree: DynamicAabbTree[T], idx: NodeIndex): bool =
  idx >= 0 and idx < tree.nodes.len

proc is_valid_leaf*[T](tree: DynamicAabbTree[T], leaf_idx: NodeIndex): bool

proc node_aabb[T](tree: DynamicAabbTree[T], idx: NodeIndex): Aabb =
  let node = tree.nodes[idx]
  doAssert node.storage == nsk_used
  result.min_x = node.min_x
  result.min_y = node.min_y
  result.min_z = node.min_z
  result.max_x = node.max_x
  result.max_y = node.max_y
  result.max_z = node.max_z

proc set_node_aabb[T](tree: DynamicAabbTree[T], idx: NodeIndex, box: Aabb) =
  doAssert tree.is_valid_index(idx)
  doAssert tree.nodes[idx].storage == nsk_used
  tree.nodes[idx].min_x = box.min_x
  tree.nodes[idx].min_y = box.min_y
  tree.nodes[idx].min_z = box.min_z
  tree.nodes[idx].max_x = box.max_x
  tree.nodes[idx].max_y = box.max_y
  tree.nodes[idx].max_z = box.max_z

proc make_free_node[T](next_free_idx: NodeIndex): DynamicAabbNode[T] =
  DynamicAabbNode[T](storage: nsk_free, next_free_idx: next_free_idx)

proc make_leaf_node[T](parent_idx: NodeIndex, box: Aabb, data: sink T): DynamicAabbNode[T] =
  DynamicAabbNode[T](
    storage: nsk_used,
    parent_idx: parent_idx,
    is_leaf: true,
    child_1_idx: invalid_node_index,
    child_2_idx: invalid_node_index,
    min_x: box.min_x,
    min_y: box.min_y,
    min_z: box.min_z,
    max_x: box.max_x,
    max_y: box.max_y,
    max_z: box.max_z,
    explored_cost: 0'f32,
    data: data,
  )

proc make_internal_node[T](
  parent_idx, child_1_idx, child_2_idx: NodeIndex,
  box: Aabb,
): DynamicAabbNode[T] =
  DynamicAabbNode[T](
    storage: nsk_used,
    parent_idx: parent_idx,
    is_leaf: false,
    child_1_idx: child_1_idx,
    child_2_idx: child_2_idx,
    min_x: box.min_x,
    min_y: box.min_y,
    min_z: box.min_z,
    max_x: box.max_x,
    max_y: box.max_y,
    max_z: box.max_z,
    explored_cost: 0'f32,
    data: default(T),
  )

proc reset_free_range[T](
  tree: DynamicAabbTree[T],
  start_idx, end_idx: int,
  tail_idx: NodeIndex = invalid_node_index,
): NodeIndex =
  if end_idx <= start_idx:
    return tail_idx

  for idx in start_idx ..< end_idx:
    let next_idx =
      if idx + 1 < end_idx:
        idx + 1
      else:
        tail_idx
    tree.nodes[idx] = make_free_node[T](next_idx)

  start_idx

proc grow[T](tree: DynamicAabbTree[T], required_capacity: int) =
  if tree.nodes.len >= required_capacity:
    return

  let old_capacity = tree.nodes.len
  var new_capacity =
    if old_capacity == 0:
      1
    else:
      old_capacity + max(old_capacity div 2, 1)

  if new_capacity < required_capacity:
    new_capacity = required_capacity

  tree.nodes.set_len(new_capacity)
  tree.free_idx = tree.reset_free_range(old_capacity, new_capacity, tree.free_idx)

proc alloc_node[T](tree: DynamicAabbTree[T]): NodeIndex =
  if tree.free_idx == invalid_node_index:
    tree.grow(tree.nodes.len + 1)

  result = tree.free_idx
  doAssert tree.is_valid_index(result)
  doAssert tree.nodes[result].storage == nsk_free
  tree.free_idx = tree.nodes[result].next_free_idx

proc free_node[T](tree: DynamicAabbTree[T], idx: NodeIndex) =
  doAssert tree.is_valid_index(idx)
  doAssert tree.nodes[idx].storage == nsk_used
  tree.nodes[idx] = make_free_node[T](tree.free_idx)
  tree.free_idx = idx

proc replace_child[T](
  tree: DynamicAabbTree[T],
  parent_idx, old_child_idx, new_child_idx: NodeIndex,
) =
  doAssert tree.is_valid_index(parent_idx)
  doAssert tree.nodes[parent_idx].storage == nsk_used
  doAssert not tree.nodes[parent_idx].is_leaf

  if tree.nodes[parent_idx].child_1_idx == old_child_idx:
    tree.nodes[parent_idx].child_1_idx = new_child_idx
  else:
    doAssert tree.nodes[parent_idx].child_2_idx == old_child_idx
    tree.nodes[parent_idx].child_2_idx = new_child_idx

proc refit_to_children[T](tree: DynamicAabbTree[T], idx: NodeIndex) =
  doAssert tree.is_valid_index(idx)
  doAssert tree.nodes[idx].storage == nsk_used
  doAssert not tree.nodes[idx].is_leaf

  let child_1_idx = tree.nodes[idx].child_1_idx
  let child_2_idx = tree.nodes[idx].child_2_idx
  let combined_box = union_aabb(tree.node_aabb(child_1_idx), tree.node_aabb(child_2_idx))
  tree.set_node_aabb(idx, combined_box)

proc try_rotate[T](tree: DynamicAabbTree[T], idx: NodeIndex) =
  if not tree.is_valid_index(idx):
    return
  if tree.nodes[idx].storage != nsk_used or tree.nodes[idx].is_leaf:
    return

  let parent_idx = tree.nodes[idx].parent_idx
  if parent_idx == invalid_node_index:
    return

  let other_idx =
    if tree.nodes[parent_idx].child_1_idx == idx:
      tree.nodes[parent_idx].child_2_idx
    else:
      tree.nodes[parent_idx].child_1_idx

  let child_1_idx = tree.nodes[idx].child_1_idx
  let child_2_idx = tree.nodes[idx].child_2_idx
  let current_cost = surface_area(tree.node_aabb(idx))
  let child_1_swap_cost = surface_area(union_aabb(tree.node_aabb(other_idx), tree.node_aabb(child_2_idx)))
  let child_2_swap_cost = surface_area(union_aabb(tree.node_aabb(other_idx), tree.node_aabb(child_1_idx)))

  if child_1_swap_cost < current_cost and child_1_swap_cost <= child_2_swap_cost:
    tree.nodes[idx].child_1_idx = other_idx
    tree.nodes[other_idx].parent_idx = idx

    if tree.nodes[parent_idx].child_1_idx == idx:
      tree.nodes[parent_idx].child_2_idx = child_1_idx
    else:
      tree.nodes[parent_idx].child_1_idx = child_1_idx

    tree.nodes[child_1_idx].parent_idx = parent_idx
    tree.refit_to_children(idx)
  elif child_2_swap_cost < current_cost:
    tree.nodes[idx].child_2_idx = other_idx
    tree.nodes[other_idx].parent_idx = idx

    if tree.nodes[parent_idx].child_1_idx == idx:
      tree.nodes[parent_idx].child_2_idx = child_2_idx
    else:
      tree.nodes[parent_idx].child_1_idx = child_2_idx

    tree.nodes[child_2_idx].parent_idx = parent_idx
    tree.refit_to_children(idx)

proc refit_upward[T](tree: DynamicAabbTree[T], start_idx: NodeIndex) =
  var idx = start_idx
  while idx != invalid_node_index:
    doAssert tree.is_valid_index(idx)
    if not tree.nodes[idx].is_leaf:
      tree.refit_to_children(idx)
      tree.try_rotate(idx)
      tree.refit_to_children(idx)
    idx = tree.nodes[idx].parent_idx

proc find_best_sibling[T](tree: DynamicAabbTree[T], leaf_box: Aabb): NodeIndex =
  if tree.root_idx == invalid_node_index:
    return invalid_node_index
  if tree.nodes[tree.root_idx].is_leaf:
    return tree.root_idx

  let leaf_area = surface_area(leaf_box)
  let root_box = tree.node_aabb(tree.root_idx)
  var best_cost = surface_area(union_aabb(root_box, leaf_box))
  var best_idx = tree.root_idx
  var queue: HeapQueue[tuple[cost: float32, idx: NodeIndex]]

  tree.nodes[tree.root_idx].explored_cost = 0'f32
  queue.push((cost: 0'f32, idx: tree.root_idx))

  while queue.len > 0:
    let entry = queue.pop()
    if entry.cost > best_cost:
      continue

    let idx = entry.idx
    let node_box = tree.node_aabb(idx)
    let union_box = union_aabb(node_box, leaf_box)
    let node_cost = tree.nodes[idx].explored_cost + surface_area(union_box)

    if node_cost < best_cost:
      best_cost = node_cost
      best_idx = idx

    if tree.nodes[idx].is_leaf:
      continue

    let child_explored_cost = tree.nodes[idx].explored_cost + surface_area(union_box) - surface_area(node_box)
    let child_indices = [tree.nodes[idx].child_1_idx, tree.nodes[idx].child_2_idx]
    for child_idx in child_indices:
      let child_box = tree.node_aabb(child_idx)
      let lower_bound =
        child_explored_cost +
        surface_area(union_aabb(child_box, leaf_box)) +
        min(leaf_area - surface_area(child_box), 0'f32)

      if lower_bound <= best_cost:
        tree.nodes[child_idx].explored_cost = child_explored_cost
        queue.push((cost: lower_bound, idx: child_idx))

  best_idx

proc insert_leaf_topology[T](tree: DynamicAabbTree[T], leaf_idx: NodeIndex) =
  doAssert tree.is_valid_leaf(leaf_idx)
  tree.nodes[leaf_idx].parent_idx = invalid_node_index

  if tree.root_idx == invalid_node_index:
    tree.root_idx = leaf_idx
    return

  let sibling_idx = tree.find_best_sibling(tree.node_aabb(leaf_idx))
  let old_parent_idx = tree.nodes[sibling_idx].parent_idx
  let new_parent_idx = tree.alloc_node()
  let combined_box = union_aabb(tree.node_aabb(sibling_idx), tree.node_aabb(leaf_idx))

  tree.nodes[new_parent_idx] = make_internal_node[T](old_parent_idx, sibling_idx, leaf_idx, combined_box)
  tree.nodes[sibling_idx].parent_idx = new_parent_idx
  tree.nodes[leaf_idx].parent_idx = new_parent_idx

  if old_parent_idx == invalid_node_index:
    tree.root_idx = new_parent_idx
  else:
    tree.replace_child(old_parent_idx, sibling_idx, new_parent_idx)

  tree.refit_upward(new_parent_idx)

proc remove_leaf_topology[T](
  tree: DynamicAabbTree[T],
  leaf_idx: NodeIndex,
  free_leaf: bool,
): NodeIndex =
  doAssert tree.is_valid_leaf(leaf_idx)

  if leaf_idx == tree.root_idx:
    tree.root_idx = invalid_node_index
    if free_leaf:
      tree.free_node(leaf_idx)
    else:
      tree.nodes[leaf_idx].parent_idx = invalid_node_index
    return invalid_node_index

  let parent_idx = tree.nodes[leaf_idx].parent_idx
  let grandparent_idx = tree.nodes[parent_idx].parent_idx
  let sibling_idx =
    if tree.nodes[parent_idx].child_1_idx == leaf_idx:
      tree.nodes[parent_idx].child_2_idx
    else:
      tree.nodes[parent_idx].child_1_idx

  if grandparent_idx == invalid_node_index:
    tree.root_idx = sibling_idx
    tree.nodes[sibling_idx].parent_idx = invalid_node_index
  else:
    tree.replace_child(grandparent_idx, parent_idx, sibling_idx)
    tree.nodes[sibling_idx].parent_idx = grandparent_idx

  tree.free_node(parent_idx)
  if free_leaf:
    tree.free_node(leaf_idx)
  else:
    tree.nodes[leaf_idx].parent_idx = invalid_node_index

  grandparent_idx

proc approx_equal(a, b: float32): bool =
  abs(a - b) <= 1.0e-6'f32

proc init_dynamic_aabb_tree*[T](
  initial_capacity = 16,
  fat_margin = 0.2'f32,
): DynamicAabbTree[T] =
  new result
  let capacity = max(initial_capacity, 0)
  result.nodes = newSeq[DynamicAabbNode[T]](capacity)
  result.root_idx = invalid_node_index
  result.free_idx = invalid_node_index
  result.leaf_count = 0
  result.fat_margin = fat_margin
  result.free_idx = result.reset_free_range(0, capacity)

proc clear*[T](tree: DynamicAabbTree[T]) =
  tree.root_idx = invalid_node_index
  tree.leaf_count = 0
  tree.free_idx = tree.reset_free_range(0, tree.nodes.len)

proc is_valid_leaf*[T](tree: DynamicAabbTree[T], leaf_idx: NodeIndex): bool =
  tree.is_valid_index(leaf_idx) and
  tree.nodes[leaf_idx].storage == nsk_used and
  tree.nodes[leaf_idx].is_leaf

proc insert*[T](
  tree: DynamicAabbTree[T],
  min_x, min_y, min_z,
  max_x, max_y, max_z: float32,
  data: sink T,
): NodeIndex =
  let leaf_idx = tree.alloc_node()
  let fat_box = fatten_aabb(init_aabb(min_x, min_y, min_z, max_x, max_y, max_z), tree.fat_margin)
  tree.nodes[leaf_idx] = make_leaf_node[T](invalid_node_index, fat_box, data)
  inc tree.leaf_count
  tree.insert_leaf_topology(leaf_idx)
  leaf_idx

proc insert*[T](tree: DynamicAabbTree[T], aabb: FBB, data: sink T): NodeIndex =
  let box = to_aabb(aabb)
  tree.insert(box.min_x, box.min_y, box.min_z, box.max_x, box.max_y, box.max_z, data)

proc remove*[T](tree: DynamicAabbTree[T], leaf_idx: NodeIndex): bool =
  if not tree.is_valid_leaf(leaf_idx):
    return false

  let refit_idx = tree.remove_leaf_topology(leaf_idx, free_leaf = true)
  if refit_idx != invalid_node_index:
    tree.refit_upward(refit_idx)

  dec tree.leaf_count
  true

proc update*[T](
  tree: DynamicAabbTree[T],
  leaf_idx: NodeIndex,
  min_x, min_y, min_z,
  max_x, max_y, max_z: float32,
  displacement: F3 = (0'f32, 0'f32, 0'f32),
): bool =
  if not tree.is_valid_leaf(leaf_idx):
    return false

  let tight_box = init_aabb(min_x, min_y, min_z, max_x, max_y, max_z)
  let next_box = fatten_aabb(
    tight_box,
    tree.fat_margin,
    displacement,
  )

  if contains(tree.node_aabb(leaf_idx), tight_box):
    return false

  let refit_idx = tree.remove_leaf_topology(leaf_idx, free_leaf = false)
  if refit_idx != invalid_node_index:
    tree.refit_upward(refit_idx)

  tree.set_node_aabb(leaf_idx, next_box)
  tree.nodes[leaf_idx].explored_cost = 0'f32
  tree.insert_leaf_topology(leaf_idx)
  true

proc update*[T](
  tree: DynamicAabbTree[T],
  leaf_idx: NodeIndex,
  aabb: FBB,
  displacement: F3 = (0'f32, 0'f32, 0'f32),
): bool =
  let box = to_aabb(aabb)
  tree.update(
    leaf_idx,
    box.min_x, box.min_y, box.min_z,
    box.max_x, box.max_y, box.max_z,
    displacement,
  )

proc get_aabb*[T](tree: DynamicAabbTree[T], idx: NodeIndex): FBB =
  doAssert tree.is_valid_index(idx)
  doAssert tree.nodes[idx].storage == nsk_used
  to_fbb(tree.node_aabb(idx))

proc data*[T](tree: DynamicAabbTree[T], leaf_idx: NodeIndex): lent T =
  doAssert tree.is_valid_leaf(leaf_idx)
  tree.nodes[leaf_idx].data

iterator query*[T](
  tree: DynamicAabbTree[T],
  min_x, min_y, min_z,
  max_x, max_y, max_z: float32,
): NodeIndex =
  if tree.root_idx != invalid_node_index:
    let query_box = init_aabb(min_x, min_y, min_z, max_x, max_y, max_z)
    var stack = @[tree.root_idx]

    while stack.len > 0:
      let idx = stack.pop()
      if not overlaps(tree.node_aabb(idx), query_box):
        continue

      if tree.nodes[idx].is_leaf:
        yield idx
      else:
        stack.add tree.nodes[idx].child_1_idx
        stack.add tree.nodes[idx].child_2_idx

iterator query*[T](tree: DynamicAabbTree[T], aabb: FBB): NodeIndex =
  let box = to_aabb(aabb)
  for idx in tree.query(box.min_x, box.min_y, box.min_z, box.max_x, box.max_y, box.max_z):
    yield idx

iterator potential_pairs*[T](tree: DynamicAabbTree[T]): tuple[a, b: NodeIndex] =
  if tree.root_idx != invalid_node_index and not tree.nodes[tree.root_idx].is_leaf:
    var stack = @[(tree.nodes[tree.root_idx].child_1_idx, tree.nodes[tree.root_idx].child_2_idx)]

    while stack.len > 0:
      let (a_idx, b_idx) = stack.pop()
      let a_node = tree.nodes[a_idx]
      let b_node = tree.nodes[b_idx]

      if a_node.parent_idx == b_node.parent_idx:
        if not a_node.is_leaf:
          stack.add (a_node.child_1_idx, a_node.child_2_idx)
        if not b_node.is_leaf:
          stack.add (b_node.child_1_idx, b_node.child_2_idx)

      if not overlaps(tree.node_aabb(a_idx), tree.node_aabb(b_idx)):
        continue

      if a_node.is_leaf and b_node.is_leaf:
        if a_idx < b_idx:
          yield (a: a_idx, b: b_idx)
        else:
          yield (a: b_idx, b: a_idx)
        continue

      if a_node.is_leaf:
        stack.add (a_idx, b_node.child_1_idx)
        stack.add (a_idx, b_node.child_2_idx)
      elif b_node.is_leaf:
        stack.add (a_node.child_1_idx, b_idx)
        stack.add (a_node.child_2_idx, b_idx)
      else:
        stack.add (a_node.child_1_idx, b_node.child_1_idx)
        stack.add (a_node.child_1_idx, b_node.child_2_idx)
        stack.add (a_node.child_2_idx, b_node.child_1_idx)
        stack.add (a_node.child_2_idx, b_node.child_2_idx)

proc compute_depth*[T](tree: DynamicAabbTree[T]): int =
  if tree.root_idx == invalid_node_index:
    return 0

  var stack = @[(tree.root_idx, 1)]
  while stack.len > 0:
    let (idx, depth) = stack.pop()
    if depth > result:
      result = depth

    let node = tree.nodes[idx]
    if not node.is_leaf:
      stack.add (node.child_1_idx, depth + 1)
      stack.add (node.child_2_idx, depth + 1)

proc validate*[T](tree: DynamicAabbTree[T]) =
  var free_seen = newSeq[bool](tree.nodes.len)
  var free_count = 0
  var free_idx = tree.free_idx

  while free_idx != invalid_node_index:
    doAssert tree.is_valid_index(free_idx)
    doAssert not free_seen[free_idx]
    doAssert tree.nodes[free_idx].storage == nsk_free
    free_seen[free_idx] = true
    inc free_count
    free_idx = tree.nodes[free_idx].next_free_idx

  if tree.root_idx == invalid_node_index:
    doAssert tree.leaf_count == 0
    doAssert free_count == tree.nodes.len
    return

  doAssert tree.is_valid_index(tree.root_idx)
  doAssert tree.nodes[tree.root_idx].storage == nsk_used

  var reachable = newSeq[bool](tree.nodes.len)
  var stack = @[tree.root_idx]
  var reachable_count = 0
  var leaf_count = 0

  while stack.len > 0:
    let idx = stack.pop()
    doAssert tree.is_valid_index(idx)
    doAssert not reachable[idx]
    doAssert not free_seen[idx]

    reachable[idx] = true
    inc reachable_count

    let node = tree.nodes[idx]
    doAssert node.storage == nsk_used

    if idx == tree.root_idx:
      doAssert node.parent_idx == invalid_node_index
    else:
      doAssert tree.is_valid_index(node.parent_idx)

    if node.is_leaf:
      inc leaf_count
      doAssert node.child_1_idx == invalid_node_index
      doAssert node.child_2_idx == invalid_node_index
    else:
      doAssert tree.is_valid_index(node.child_1_idx)
      doAssert tree.is_valid_index(node.child_2_idx)
      doAssert node.child_1_idx != node.child_2_idx
      doAssert tree.nodes[node.child_1_idx].parent_idx == idx
      doAssert tree.nodes[node.child_2_idx].parent_idx == idx

      let combined_box = union_aabb(tree.node_aabb(node.child_1_idx), tree.node_aabb(node.child_2_idx))
      doAssert approx_equal(node.min_x, combined_box.min_x)
      doAssert approx_equal(node.min_y, combined_box.min_y)
      doAssert approx_equal(node.min_z, combined_box.min_z)
      doAssert approx_equal(node.max_x, combined_box.max_x)
      doAssert approx_equal(node.max_y, combined_box.max_y)
      doAssert approx_equal(node.max_z, combined_box.max_z)

      stack.add node.child_1_idx
      stack.add node.child_2_idx

  doAssert leaf_count == tree.leaf_count
  doAssert reachable_count + free_count == tree.nodes.len

  for idx in 0 ..< tree.nodes.len:
    if reachable[idx]:
      doAssert tree.nodes[idx].storage == nsk_used
    else:
      doAssert free_seen[idx]
