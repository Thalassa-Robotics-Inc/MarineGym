# Robots

How **RobotBase** works: spawn, _create_prim, initialize, and how **UnderwaterVehicle** extends it. Robot config (RobotCfg, rigid/articulation props) and view creation (ArticulationView / RigidPrimView). See **03-UnderwaterVehicle.md** for hydro, apply_action, and get_state.

---

## RobotBase (marinegym/robots/robot.py)

- **ASSET_PATH** — `marinegym/robots/assets` (used by BlueROV/BlueROVHeavy for usd_path, param_path, fixed_usd_path).
- **TEMPLATE_PRIM_PATH** — `/World/envs/env_0` (default base for prim paths when not overridden).

### Class attributes (subclass set these)

- **usd_path** — USD file used when **spawning** the robot (e.g. main BlueROV.usd).
- **fixed_usd_path** — Used by some envs (e.g. Hover) for a **visual-only** target copy of the robot; no physics.
- **cfg_cls** — RobotCfg (or subclass); provides rigid_props and articulation_props.

### Lifecycle

1. **Constructor** — Requires `SimulationContext._instance` to exist. Reads `cfg` (rigid_props, articulation_props), sets device, dt, gravity from simulation context.
2. **spawn(translations, orientations, prim_paths)** — Must be called **before** `simulation_context.reset()`. For each (translation, orientation, prim_path):
   - Calls **`_create_prim(prim_path, translation, orientation)`** which uses **`self.usd_path`** via `prim_utils.create_prim(..., usd_path=self.usd_path, ...)`.
   - Applies rigid body properties (linear/angular damping, max velocities, disable_gravity, etc.) and, if articulation, articulation properties (solver iterations, self-collisions).
3. **initialize(prim_paths_expr)** — Must be called **after** `simulation_context.reset()`. Creates the view:
   - If **is_articulation**: `ArticulationView(prim_paths_expr, ..., shape=(-1, self.n))`.
   - Else: `RigidPrimView(...)`.
   - Calls `_view.initialize()`, `_view.post_reset()`, sets `self.shape`, `self.prim_paths`, `self.initialized = True`.

### Methods (pose / velocity / joints)

- **get_world_poses**, **set_world_poses** — Delegate to _view.
- **get_velocities**, **set_velocities** — Delegate to _view.
- **get_joint_positions**, **set_joint_positions**, **set_joint_position_targets**, **get_joint_velocities**, **set_joint_velocities** — Articulation only; delegate to _view.
- **get_force_sensor_forces** — Articulation or rigid; returns forces from view.
- **apply_action**, **get_state**, **_reset_idx** — Abstract; implemented by UnderwaterVehicle.

---

## Config (marinegym/robots/config.py)

- **RigidBodyPropertiesCfg** — linear_damping, angular_damping, max_linear_velocity, max_angular_velocity, max_depenetration_velocity, disable_gravity, retain_accelerations.
- **ArticulationRootPropertiesCfg** — enable_self_collisions, solver_position_iteration_count, solver_velocity_iteration_count.
- **RobotCfg** — rigid_props, articulation_props. RobotBase subclasses use this (or a subclass like UnderwaterVehicleCfg) in __init__.

---

## UnderwaterVehicle (summary)

- Subclass of RobotBase; adds **param_path** (YAML with hydro and rotor config).
- In **__init__**: reads **param_path** with yaml.safe_load into **self.params**; sets num_rotors, action_spec, state_spec, intrinsics_spec.
- In **initialize**: builds base_link RigidPrimView, rotors_view, T200 rotors, hydro matrices (added mass, damping), buoyancy/volume/coBM; **param_path** is only read in __init__, **usd_path** is used in spawn (via _create_prim). See **03-UnderwaterVehicle.md**, **09-Assets.md**.

---

## See also

- **03-UnderwaterVehicle.md** — Full UnderwaterVehicle flow (apply_action, get_state, hydro).
- **04-Views.md** — ArticulationView, RigidPrimView in detail.
- **09-Assets.md** — usd_path, param_path, fixed_usd_path and BlueROV assets.
