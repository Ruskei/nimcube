import portal
import part
import cuboids
import dynamic_aabb_tree
import narrowphase
import physics_math

proc construct_parts(parts: seq[PartObj], portals: Portals, portal_aabb_tree: DynamicAabbTree[SpecificPortalsHandle]): seq[PartObj] =
  var new_parts: seq[PartObj]

  for part in parts:
    let part_bb = part.aabb()
    var added_part = false
    for portal_leaf_idx in portal_aabb_tree.query(part_bb):
      let portal_handle = portal_aabb_tree.data portal_leaf_idx
      let portal = portals.portal(portal_handle.handle)
      let (portal_from_origin, portal_from_quat, portal_to_origin, portal_to_quat) = case portal_handle.which
        of wp_a: (portal.origin_a, portal.quat_a, portal.origin_b, portal.quat_b)
        of wp_b: (portal.origin_b, portal.quat_b, portal.origin_a, portal.quat_a)
      let portal_normal = portal_from_quat.rotate_vector((0'f32, 0'f32, 1'f32))
      let rotation = normalized(portal_to_quat * conjugate(portal_from_quat))

      let collides = part.mesh.mesh_collides_with_portal_side(portal_from_origin, portal_from_quat, portal.scale_x, portal.scale_y)
      if not collides:
        continue

      let submeshes = part.mesh.slice_mesh_by_plane(portal_from_origin, portal_normal)
      if submeshes.len == 1:
        continue

      added_part = true

      let base_side = part.mesh.mesh_center.point_is_on_plane_side(portal_from_origin, portal_normal, true)
      for submesh in submeshes:
        if base_side == submesh.mesh_center.point_is_on_plane_side(portal_from_origin, portal_normal, true):
          new_parts.add PartObj(mesh: submesh)
        else:
          new_parts.add PartObj(
            mesh: submesh
              .translated(-portal_from_origin)
              .rotated(rotation)
              .translated(portal_to_origin)
          )

    if not added_part:
      new_parts.add part
  
  result =
    if new_parts.len == parts.len: new_parts
    else: construct_parts(new_parts, portals, portal_aabb_tree)

proc construct_parts_from_body*(body_handle: BodyHandle, data: InternalData, portals: Portals, portal_aabb_tree: DynamicAabbTree[SpecificPortalsHandle]): seq[PartObj] =
  var parts: seq[PartObj] = @[data.part body_handle]
  result = construct_parts(parts, portals, portal_aabb_tree)
