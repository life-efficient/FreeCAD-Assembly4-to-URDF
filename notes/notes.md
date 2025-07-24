

# JOINT TRANSFORMS

In FreeCAD, joints are defined by transformations from the origins of the two links that the joint connects.
Each joint has:
- `from_parent_origin`: The transform from the parent link's origin to the joint location, **expressed in the joint's local frame** (not the global/world frame).
- `from_child_origin`: The transform from the child link's origin to the joint location, **expressed in the joint's local frame**.

This means:
- These transforms describe how to get from the link's origin to the joint, but the measurement is made in the coordinate system of the joint itself.
- For example, two identical servos positioned 10mm away from different revolute joints will have the same `from_parent_origin` and `from_child_origin` transforms, regardless of where the joint is in the global assembly.

In URDF, joint `<origin>` tags define the transform from the **parent link frame** to the **child link frame** at the joint. This means:
- The URDF joint transform should represent the spatial relationship between the parent and child link **at the joint location**.
- The transform is not simply from the parent link's origin to the child link's origin, but from the parent link's **joint attachment frame** to the child link's **joint attachment frame**.

**Computation:**
- For the root joint (attached to the world), use `curr_joint.from_parent_origin` directly.
- For all other joints, the transform is:
  ```
  urdf_transform = prev_joint.from_child_origin.inverse() * axis_alignment * curr_joint.from_parent_origin
  ```
  where:
  - `prev_joint.from_child_origin.inverse()` brings you from the previous joint's child frame back to the parent link's frame.
  - `axis_alignment` (see below) aligns the axes of the two joint frames.
  - `curr_joint.from_parent_origin` moves from the parent link's frame to the current joint's location.

# MESH OFFSET

Each link's mesh is modeled in its own local coordinate system, which is usually **not** aligned with the joint frame. Therefore, the mesh must be offset so that it appears in the correct position and orientation in the URDF.

The mesh offset should be:
- The transform that brings the mesh from its local modeling frame to the joint frame (the frame used by the URDF joint transform).
- This is typically:
  - The **inverse** of the parent joint's `from_child_origin` transform (to bring the mesh from its local origin to the joint frame)
  - **After being aligned**: If the joint frames are not aligned (e.g., the axes of the parent and child joint frames differ), you must apply a rotation (alignment) so that the mesh is correctly oriented in the parent link's joint frame.

**Step-by-step mesh offset calculation:**
1. Take the inverse of `from_child_origin` to move the mesh from its local frame to the joint frame.
2. Compute the alignment rotation between the parent and child joint frames (see below).
3. Apply the alignment rotation to the result of step 1.
4. The final mesh offset is:
   ```
   mesh_offset = alignment * from_child_origin.inverse()
   ```
   (or, depending on conventions, `from_child_origin.inverse() * alignment`)

# ALIGNMENT

The orientation of the transform from each link origin to the joint location do not necessarily align.
- E.g. two parts modeled in the same orientation and that should rotate around the Y axis of their modeled orientation may have placement transforms to the joint of +90X and -90X.
- So, the origins of the placements need to be aligned to create a full transformation from one origin to the other.

**How to compute alignment:**
- Compute the rotation-only transform that aligns the child joint frame to the parent joint frame.
- This is typically done by comparing the rotation components of the two placements and finding the rotation that brings one into alignment with the other.
- In code, this is often:
  ```
  alignment = get_origin_alignment(parent_joint.from_parent_origin, parent_joint.from_child_origin)
  ```
- Apply this alignment as part of the mesh offset or joint transform as needed.

---

**Summary:**
- Always check both translation and rotation when composing transforms for URDF export.
- The mesh offset is not just the inverse of the child origin; it may need to be aligned to the parent joint frame.
- Debug by logging all placements and checking that the mesh appears at the correct location and orientation in the URDF viewer.


Notes about URDF
In URDF, the <origin> transform is applied before the joint axis/rotation