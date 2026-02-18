# UnderwaterVehicle

**UnderwaterVehicle** extends RobotBase with YAML-driven parameters, thrusters (T200), hydrodynamics (buoyancy, damping, added mass, coriolis), **apply_action**, and **get_state**. This doc covers initialization, apply_action flow, get_state, and the built-in vehicle classes (BlueROV, BlueROVHeavy). Hydro formulas are in **10-Hydrodynamics.md**.

---

## Paths (set by subclass)

- **param_path** — YAML file path. Read in **__init__** with `yaml.safe_load` into **self.params**. Contains: volume, coBM, hydro_coef (added_mass, linear_damping, quadratic_damping), rotor_configuration (num_rotors, directions, time_constants, force_constants, moment_constants, max_rotation_velocities).
- **usd_path** — Used by RobotBase.**_create_prim** when **spawn** is called (see **02-Robots.md**).
- **fixed_usd_path** — Used by envs (e.g. Hover) for the **target visual** only; not used for the simulated drone.

---

## __init__

- Calls **super().__init__** (RobotBase).
- Opens **param_path**, loads YAML into **self.params**.
- Sets **num_rotors** from params; **action_spec** (BoundedTensorSpec -1..1 for num_rotors); **intrinsics_spec** (mass, inertia, com, KF, KM, tau_up, tau_down, drag_coef); **state_spec** (dim depends on force_sensor); **randomization** defaultdict.

---

## initialize(prim_paths_expr, track_contact_forces)

Called **after** world/simulation reset. Builds:

- **base_link** — RigidPrimView for `prim_paths_expr/base_link`; used for masses, inertias, and applying hydro forces/torques.
- **rotor_joint_indices** — DOF indices whose names start with `"rotor"`.
- **rotors_view** — RigidPrimView for `prim_paths_expr/rotor_*`; shape `(*self.shape, num_rotors)`; used to apply thruster forces at rotor positions.
- **rotors** — T200(rotor_config, dt); **rotor_params** from make_functional(rotors), expanded to self.shape.
- **Hydro and body state** — masses, gravity (masses*9.81), inertias, volumes, coBMs from params; added_mass_matrix, linear/quadratic_damping_matrix; prev_body_vels, prev_body_acc; flow_vels, max_flow_vel, flow_noise_scale.

So: **param_path** is used in __init__ and again in initialize (via self.params); **usd_path** is used only in spawn (RobotBase._create_prim).

---

## apply_action(actions)

1. **Rotor commands** — `rotor_cmds = actions.expand(*self.shape, self.num_rotors)`.
2. **Thrusts and moments** — `torch.vmap(torch.vmap(self.rotors, ...))(rotor_cmds, self.rotor_params)` → thrusts, moments.
3. **Torque axis** — From rotors_view.get_world_poses(); torque_axis from quat_axis(rotor_rot, axis=2).
4. **self.thrusts**, **self.torques** — Thrusts per rotor; net torque from moments and torque_axis.
5. **Hydro** — `apply_hydrodynamic_forces(flow_vels)` → hydro_forces, hydro_torques; added to self.forces, self.torques.
6. **Apply to sim** — rotors_view.apply_forces_and_torques_at_pos(self.thrusts); base_link.apply_forces_and_torques_at_pos(self.forces, self.torques) (both in body frame).
7. **throttle_difference** — Norm of throttle change (for reward/smoothness).

---

## get_state(check_nan, env_frame)

1. **Pose** — get_world_poses(True) → self.pos, self.rot; if env_frame and _envs_positions exist, subtract _envs_positions from pos.
2. **Velocity** — get_velocities(True) → vel_w; body-frame vel_b from quat_rotate_inverse; self.vel_w, self.vel_b updated.
3. **Heading, up** — quat_axis(rot, axis=0), quat_axis(rot, axis=2).
4. **State vector** — Concatenate [pos, rot, vel, heading, up, throttle*2-1]; optionally force_sensor readings (normalized); return tensor. If check_nan, assert no NaNs.

---

## _reset_idx(env_ids, train)

Zeros thrusts, torques, vel, acc, throttle, throttle_difference, prev_body_acc, prev_body_vels; resets flow_vels with random flow; returns env_ids.

---

## BlueROV and BlueROVHeavy

- **BlueROV** (marinegym/robots/drone/BlueROV.py): usd_path = ASSET_PATH + "/usd/BlueROV/BlueROV.usd", param_path = ASSET_PATH + "/usd/BlueROV/BlueROV.yaml", fixed_usd_path = ASSET_PATH + "/usd/BlueROV/fixed_usd/BlueROV/BlueROV.usd".
- **BlueROVHeavy** (marinegym/robots/drone/BlueROVHeavy.py): Same pattern for BlueROVHeavy USD and YAML.

Both are registered in **UnderwaterVehicle.REGISTRY** (and lower-case) via RobotBase.__init_subclass__. **UnderwaterVehicle.make(drone_model, controller, device)** looks up REGISTRY[drone_model] and returns an instance (and optionally controller).

---

## See also

- **10-Hydrodynamics.md** — Buoyancy, damping, added mass, coriolis; apply_hydrodynamic_forces.
- **05-Actuators.md** — T200 thrust/moment.
- **09-Assets.md** — YAML layout, main vs fixed USD.
