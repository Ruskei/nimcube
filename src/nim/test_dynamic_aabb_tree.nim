import std/algorithm

import dynamic_aabb_tree
import physics_math

type
  payload = object
    id: int
    label: string

  leaf_pair = tuple[a, b: NodeIndex]

proc `<`(a, b: leaf_pair): bool =
  if a.a == b.a:
    return a.b < b.b
  a.a < b.a

proc approx_equal(a, b: float32): bool =
  abs(a - b) <= 1.0e-6'f32

proc make_bb(
  min_x, min_y, min_z,
  max_x, max_y, max_z: float32,
): FBB =
  result.min = (min_x, min_y, min_z)
  result.max = (max_x, max_y, max_z)

proc collect_query[T](tree: DynamicAabbTree[T], box: FBB): seq[NodeIndex] =
  for leaf_idx in tree.query(box):
    result.add leaf_idx
  result.sort()

proc active_leaves[T](tree: DynamicAabbTree[T]): seq[NodeIndex] =
  for idx, node in tree.nodes:
    if node.storage == nsk_used and node.is_leaf:
      result.add idx
  result.sort()

proc collect_pairs[T](tree: DynamicAabbTree[T]): seq[leaf_pair] =
  for pair in tree.potential_pairs():
    result.add pair
  result.sort()

proc brute_force_pairs[T](tree: DynamicAabbTree[T]): seq[leaf_pair] =
  let leaves = active_leaves(tree)
  for i in 0 ..< leaves.len:
    for j in i + 1 ..< leaves.len:
      let a_idx = leaves[i]
      let b_idx = leaves[j]
      if overlaps(tree.get_aabb(a_idx), tree.get_aabb(b_idx)):
        result.add (a: a_idx, b: b_idx)
  result.sort()

proc test_init_and_free_list() =
  var tree = init_dynamic_aabb_tree[int](initial_capacity = 4, fat_margin = 0'f32)

  doAssert tree.root_idx == invalid_node_index
  doAssert tree.free_idx == 0
  doAssert tree.nodes.len == 4

  for idx in 0 ..< tree.nodes.len:
    doAssert tree.nodes[idx].storage == nsk_free
    let next_idx =
      if idx + 1 < tree.nodes.len:
        idx + 1
      else:
        invalid_node_index
    doAssert tree.nodes[idx].next_free_idx == next_idx

  tree.validate()

proc test_insert_query_and_grow() =
  var tree = init_dynamic_aabb_tree[payload](initial_capacity = 1, fat_margin = 0.1'f32)

  let leaf_a = tree.insert(make_bb(0'f32, 0'f32, 0'f32, 1'f32, 1'f32, 1'f32), payload(id: 1, label: "a"))
  tree.validate()

  doAssert tree.root_idx == leaf_a
  doAssert tree.leaf_count == 1
  doAssert tree.data(leaf_a).id == 1
  doAssert tree.data(leaf_a).label == "a"
  doAssert collect_query(tree, make_bb(-1'f32, -1'f32, -1'f32, 2'f32, 2'f32, 2'f32)) == @[leaf_a]

  let leaf_b = tree.insert(make_bb(3'f32, 3'f32, 3'f32, 4'f32, 4'f32, 4'f32), payload(id: 2, label: "b"))
  tree.validate()

  doAssert tree.leaf_count == 2
  doAssert tree.nodes.len >= 3
  doAssert tree.root_idx != leaf_a
  doAssert collect_query(tree, make_bb(-1'f32, -1'f32, -1'f32, 2'f32, 2'f32, 2'f32)) == @[leaf_a]
  doAssert collect_query(tree, make_bb(2.8'f32, 2.8'f32, 2.8'f32, 4.2'f32, 4.2'f32, 4.2'f32)) == @[leaf_b]
  doAssert collect_query(tree, make_bb(-10'f32, -10'f32, -10'f32, 10'f32, 10'f32, 10'f32)) == @[leaf_a, leaf_b]

proc test_remove_reuses_leaf_slot() =
  var tree = init_dynamic_aabb_tree[int](initial_capacity = 3, fat_margin = 0'f32)

  let leaf_a = tree.insert(make_bb(0'f32, 0'f32, 0'f32, 1'f32, 1'f32, 1'f32), 10)
  discard tree.insert(make_bb(4'f32, 4'f32, 4'f32, 5'f32, 5'f32, 5'f32), 20)
  tree.validate()

  doAssert tree.remove(leaf_a)
  tree.validate()

  let leaf_c = tree.insert(make_bb(8'f32, 8'f32, 8'f32, 9'f32, 9'f32, 9'f32), 30)
  tree.validate()

  doAssert leaf_c == leaf_a
  doAssert collect_query(tree, make_bb(-1'f32, -1'f32, -1'f32, 10'f32, 10'f32, 10'f32)) == active_leaves(tree)

proc test_update_keeps_stable_index() =
  var tree = init_dynamic_aabb_tree[string](initial_capacity = 2, fat_margin = 0.5'f32)

  let leaf_idx = tree.insert(make_bb(0'f32, 0'f32, 0'f32, 1'f32, 1'f32, 1'f32), "body")
  tree.validate()

  doAssert not tree.update(leaf_idx, make_bb(0.1'f32, 0.1'f32, 0.1'f32, 0.9'f32, 0.9'f32, 0.9'f32))
  tree.validate()

  doAssert tree.update(
    leaf_idx,
    make_bb(3'f32, 3'f32, 3'f32, 4'f32, 4'f32, 4'f32),
    displacement = (1'f32, 0'f32, 0'f32),
  )
  tree.validate()

  let fat_box = tree.get_aabb(leaf_idx)
  doAssert approx_equal(fat_box.min.x, 2.5'f32)
  doAssert approx_equal(fat_box.max.x, 5.5'f32)
  doAssert approx_equal(fat_box.min.y, 2.5'f32)
  doAssert approx_equal(fat_box.max.y, 4.5'f32)
  doAssert collect_query(tree, make_bb(-1'f32, -1'f32, -1'f32, 2'f32, 2'f32, 2'f32)).len == 0
  doAssert collect_query(tree, make_bb(2'f32, 2'f32, 2'f32, 6'f32, 6'f32, 6'f32)) == @[leaf_idx]

proc test_update_ignores_displacement_while_tight_aabb_still_fits() =
  var tree = init_dynamic_aabb_tree[int](initial_capacity = 1, fat_margin = 0.5'f32)

  let leaf_idx = tree.insert(make_bb(0'f32, 0'f32, 0'f32, 1'f32, 1'f32, 1'f32), 7)
  let initial_box = tree.get_aabb(leaf_idx)
  tree.validate()

  doAssert not tree.update(
    leaf_idx,
    make_bb(0.1'f32, 0.1'f32, 0.1'f32, 0.9'f32, 0.9'f32, 0.9'f32),
    displacement = (1'f32, 0'f32, 0'f32),
  )
  tree.validate()

  let updated_box = tree.get_aabb(leaf_idx)
  doAssert initial_box == updated_box

proc test_potential_pairs_match_brute_force() =
  var tree = init_dynamic_aabb_tree[int](initial_capacity = 2, fat_margin = 0'f32)

  discard tree.insert(make_bb(0'f32, 0'f32, 0'f32, 1'f32, 1'f32, 1'f32), 1)
  discard tree.insert(make_bb(0.5'f32, 0.5'f32, 0.5'f32, 1.5'f32, 1.5'f32, 1.5'f32), 2)
  discard tree.insert(make_bb(5'f32, 5'f32, 5'f32, 6'f32, 6'f32, 6'f32), 3)
  discard tree.insert(make_bb(5.4'f32, 5.4'f32, 5.4'f32, 6.4'f32, 6.4'f32, 6.4'f32), 4)
  tree.validate()

  let actual_pairs = collect_pairs(tree)
  let expected_pairs = brute_force_pairs(tree)

  doAssert actual_pairs == expected_pairs
  doAssert actual_pairs.len == 2

proc test_compute_depth() =
  var tree = init_dynamic_aabb_tree[int](initial_capacity = 1, fat_margin = 0'f32)

  doAssert tree.compute_depth() == 0

  discard tree.insert(make_bb(0'f32, 0'f32, 0'f32, 1'f32, 1'f32, 1'f32), 1)
  doAssert tree.compute_depth() == 1

  discard tree.insert(make_bb(2'f32, 2'f32, 2'f32, 3'f32, 3'f32, 3'f32), 2)
  discard tree.insert(make_bb(4'f32, 4'f32, 4'f32, 5'f32, 5'f32, 5'f32), 3)
  doAssert tree.compute_depth() >= 2

proc test_mixed_mutations_and_clear() =
  var tree = init_dynamic_aabb_tree[payload](initial_capacity = 0, fat_margin = 0.25'f32)

  let leaf_a = tree.insert(make_bb(0'f32, 0'f32, 0'f32, 1'f32, 1'f32, 1'f32), payload(id: 1, label: "a"))
  tree.validate()

  let leaf_b = tree.insert(make_bb(2'f32, 2'f32, 2'f32, 3'f32, 3'f32, 3'f32), payload(id: 2, label: "b"))
  tree.validate()

  let leaf_c = tree.insert(make_bb(4'f32, 4'f32, 4'f32, 5'f32, 5'f32, 5'f32), payload(id: 3, label: "c"))
  tree.validate()

  doAssert tree.update(
    leaf_b,
    make_bb(1.4'f32, 2'f32, 2'f32, 2.4'f32, 3'f32, 3'f32),
    displacement = (-0.5'f32, 0'f32, 0'f32),
  )
  tree.validate()

  doAssert tree.remove(leaf_a)
  tree.validate()

  let leaf_d = tree.insert(make_bb(6'f32, 6'f32, 6'f32, 7'f32, 7'f32, 7'f32), payload(id: 4, label: "d"))
  tree.validate()

  doAssert active_leaves(tree).len == 3
  doAssert collect_query(tree, make_bb(-10'f32, -10'f32, -10'f32, 10'f32, 10'f32, 10'f32)).len == 3
  doAssert tree.data(leaf_c).label == "c"
  doAssert tree.data(leaf_d).label == "d"

  tree.clear()
  tree.validate()

  doAssert tree.root_idx == invalid_node_index
  doAssert tree.leaf_count == 0
  doAssert collect_query(tree, make_bb(-1'f32, -1'f32, -1'f32, 8'f32, 8'f32, 8'f32)).len == 0

when is_main_module:
  test_init_and_free_list()
  test_insert_query_and_grow()
  test_remove_reuses_leaf_slot()
  test_update_keeps_stable_index()
  test_update_ignores_displacement_while_tight_aabb_still_fits()
  test_potential_pairs_match_brute_force()
  test_compute_depth()
  test_mixed_mutations_and_clear()
  echo "dynamic_aabb_tree tests passed"
