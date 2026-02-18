# Views

**ArticulationView** and **RigidPrimView** in `marinegym/views/__init__.py` wrap Isaac Sim physics views to provide batch-shaped, tensor-based APIs used by RobotBase and UnderwaterVehicle (get_velocities, get_world_poses, get_masses, get_inertias, apply_forces_and_torques_at_pos, etc.).

---

## Role

- **ArticulationView** — Used for the full robot articulation (root). Provides get_velocities, get_world_poses, get_joint_positions, get_joint_velocities, get_dof_limits, get_body_masses, get_force_sensor_forces, set_world_poses, set_velocities, set_joint_position_targets, etc. Outputs are unflattened to `self.shape` (e.g. (num_envs, 1)).
- **RigidPrimView** — Used for base_link and rotor_* prims. Provides get_velocities, get_world_poses, get_masses, get_inertias, get_coms, get_net_contact_forces, get_contact_force_matrix, set_world_poses, set_velocities, set_masses, apply_forces_and_torques_at_pos. Outputs unflattened to view shape.

Both extend the underlying Isaac Sim / Omni view classes and override getters (and some setters) to:
- Convert NumPy to Torch where the sim returns NumPy (Isaac Sim 5).
- Reshape 1D velocity tensors to (N, 6) before unflatten when the sim returns (6,) for a single body (Isaac Sim 5).

---

## Isaac Sim 5 compatibility (reference only)

The **actual fixes** are in the MarineGym codebase; the **rationale and error descriptions** are documented in **Test_AUV/docs** so we don’t duplicate them here:

- **get_velocities shape:** When the physics view returns root velocities with shape `(6,)` for a single articulation, unflatten fails. Fix: if `velocities.dim() == 1`, reshape to `(-1, 6)` before calling unflatten. See **Test_AUV/docs/24-MarineGym-IsaacSim5-get-velocities-shape-fix.md**.
- **NumPy outputs:** Several getters (get_velocities, get_masses, get_world_poses, etc.) can return NumPy; views use helpers to convert to tensor before any Torch-only op (clone, unflatten, .to(device)). See **Test_AUV/docs/18-MarineGym-IsaacSim5-numpy-outputs-safety-net.md**.

---

## Usage in MarineGym

- **RobotBase.initialize** — Creates ArticulationView (or RigidPrimView for non-articulation) with prim_paths_expr and shape (-1, self.n); stores as self._view.
- **UnderwaterVehicle.initialize** — Also creates RigidPrimView for base_link and for rotor_* (rotors_view). base_link is used for get_masses, get_inertias, apply_forces_and_torques_at_pos (hydro); rotors_view for get_world_poses (torque axis) and apply_forces_and_torques_at_pos (thrust).

---

## See also

- **02-Robots.md** — How RobotBase creates and uses _view.
- **03-UnderwaterVehicle.md** — base_link and rotors_view.
- **Test_AUV/docs/24-MarineGym-IsaacSim5-get-velocities-shape-fix.md**, **18-MarineGym-IsaacSim5-numpy-outputs-safety-net.md**.
