# Assets

MarineGym ships with **BlueROV** and **BlueROVHeavy** assets under **marinegym/robots/assets/usd/**. This doc covers **usd_path**, **param_path**, **fixed_usd_path**, the **main vs fixed pair** USD variants, and the **BlueROV.yaml** layout. For using these assets from another project (e.g. Test_AUV) and path override, see **Test_AUV/docs/08-BlueROV-Asset-and-MarineGym-Integration.md** and **09-Standalone-BlueROV-Setup.md**.

---

## ASSET_PATH

- **Definition** — `marinegym/robots/assets` (in robot.py: `osp.join(osp.dirname(__file__), "assets")`).
- **Usage** — BlueROV and BlueROVHeavy set usd_path, param_path, fixed_usd_path as ASSET_PATH + relative path under assets (e.g. `/usd/BlueROV/BlueROV.usd`).

---

## Paths (usd_path, param_path, fixed_usd_path)

| Path | When it is read | What it points to (BlueROV) |
|------|------------------|-----------------------------|
| **usd_path** | When **spawn** is called (RobotBase._create_prim) | Main robot USD: `assets/usd/BlueROV/BlueROV.usd` (or .usda). Full physics and collision in USD. |
| **param_path** | When the **drone is created** (UnderwaterVehicle.__init__) | YAML: `assets/usd/BlueROV/BlueROV.yaml`. Hydro and rotor parameters. |
| **fixed_usd_path** | When the **env** creates the target visual (e.g. Hover _design_scene) | Pair USD: `assets/usd/BlueROV/fixed_usd/BlueROV/BlueROV.usd`. Visual-only copy for target; no collision in USD, root_joint in USD. |

---

## Main vs fixed pair (BlueROV)

- **Main asset** — BlueROV.usd(a) under assets/usd/BlueROV/. Contains: articulation root, base_link with **mass, full inertia, collision**, rotor_0..5 and rotor_*_joint. References **Props/instanceable_meshes.usd** for mesh/collision geometry. Used for the **simulated drone**.
- **Fixed pair** — fixed_usd/BlueROV/BlueROV.usd. Same hierarchy (BlueROV → base_link, rotor_*); base_link has **mass only** (no full inertia in USD), **visuals only** (no collision in USD); **root_joint** (PhysicsFixedJoint) to world. Used only for the **Hover target visual** so the target looks like the robot.

---

## BlueROV.yaml layout

- **name** — BlueROV.
- **drag_coef** — Scalar (e.g. 0.3).
- **volume** — m³ (e.g. 0.0113459); used in buoyancy (997*9.81*volume).
- **coBM** — Center of buoyancy offset (e.g. 0.01); used in buoyancy torque.
- **hydro_coef** — added_mass (6), linear_damping (6), quadratic_damping (6); used in added mass, damping, coriolis.
- **rotor_configuration** — num_rotors, directions (1 or -1 per rotor), time_constants, force_constants, moment_constants, max_rotation_velocities. Used by T200 and LeePositionController.

See **10-Hydrodynamics.md** for how these map to buoyancy, damping, and added mass.

---

## File layout (BlueROV)

- **assets/usd/BlueROV/BlueROV.usd** (or .usda) — Main USD; references ./Props/instanceable_meshes.usd.
- **assets/usd/BlueROV/Props/instanceable_meshes.usd** (or .usda) — Mesh and collision geometry.
- **assets/usd/BlueROV/BlueROV.yaml** — Parameters (param_path).
- **assets/usd/BlueROV/fixed_usd/BlueROV/BlueROV.usd** — Pair USD (fixed_usd_path).
- **assets/usd/BlueROV/fixed_usd/BlueROV/instanceable_meshes.usda** — Meshes for pair (same folder as pair USD).

---

## Overriding paths (e.g. Test_AUV)

To use **copied** assets from another repo (e.g. Test_AUV): set **usd_path** and **param_path** on the **class** (or a subclass) **before** creating the drone instance, because param_path is read in __init__ and usd_path is read in spawn. See **Test_AUV/docs/09-Standalone-BlueROV-Setup.md** and **08-BlueROV-Asset-and-MarineGym-Integration.md**.

---

## See also

- **03-UnderwaterVehicle.md** — param_path in __init__, usd_path in spawn.
- **07-Environments.md** — fixed_usd_path for target.
- **10-Hydrodynamics.md** — volume, coBM, hydro_coef in formulas.
