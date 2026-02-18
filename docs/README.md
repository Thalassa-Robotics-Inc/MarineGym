# MarineGym documentation

This folder documents **every part of MarineGym**. Start with **00-Overview.md** for the doc index and project files quick reference. Use **12-Project-Files-Index.md** for a file-by-file description of the whole repository.

---

## Complete doc index (every file in docs/)

| File | What it covers |
|------|----------------|
| **README.md** | This file: entry point and complete doc index. |
| **00-Overview.md** | Doc index, project files quick reference, references to Test_AUV/docs. |
| **01-Package-Structure.md** | Top-level layout: `marinegym/` (robots, views, actuators, controllers, envs, learning, sensors, utils), `scripts/`, `cfg/`. |
| **02-Robots.md** | RobotBase: spawn, _create_prim, initialize, usd_path; ArticulationView/RigidPrimView; config (RobotCfg, rigid/articulation props). |
| **03-UnderwaterVehicle.md** | UnderwaterVehicle: param_path, YAML, initialize (base_link, rotors, T200), apply_action, get_state, _reset_idx; BlueROV / BlueROVHeavy. |
| **04-Views.md** | ArticulationView and RigidPrimView; Isaac Sim 5 compatibility (get_velocities shape, NumPy); reference to Test_AUV/docs. |
| **05-Actuators.md** | T200 thruster model (throttle, rpm, thrust/moment); rotor_group. |
| **06-Controllers.md** | ControllerBase; LeePositionController; controller config files (BlueROV, etc.). |
| **07-Environments.md** | IsaacEnv; Hover (and other tasks); _design_scene, spawn, target visual (fixed_usd_path). |
| **08-Scripts-and-Training.md** | train.py, Hydra, init_simulation_app, env registry, evaluate; CWD and config paths. |
| **09-Assets.md** | BlueROV USD (main vs fixed pair), BlueROV.yaml layout, ASSET_PATH; usd_path / param_path / fixed_usd_path. |
| **10-Hydrodynamics.md** | Buoyancy, damping, added mass, coriolis; apply_hydrodynamic_forces; flow_vels; where gravity is used (or not). |
| **11-Installation-and-Fixes.md** | Installation (source, Isaac Sim Python); ForkingPickler (Isaac Sim 5); references to Test_AUV/docs for get_velocities, vmap, numpy fixes. |
| **12-Project-Files-Index.md** | Every file and folder in MarineGym with a one-line description (root, marinegym/, robots/, views/, actuators/, controllers/, envs/, learning/, sensors/, utils/, scripts/, cfg/). |
| **13-Questions-Learnings-and-Details.md** | FAQ, fun facts, design decisions: what's in the repo (only Hover active; Track/Landing commented out), ForkingPickler, main vs pair, controller configs, learning layout, cross-refs to Test_AUV/docs. |
| **14-TorchRL-CPP-Extension-Warning.md** | TorchRL C++ extension warning: what it is, why it happens (ABI mismatch), who triggers it, why we leave it (no suppression); full explanation and summary. |

---

## Where to go next

- **Replicate or understand the package:** 00-Overview.md → 01-Package-Structure.md → 02-Robots.md, 03-UnderwaterVehicle.md, etc.
- **Find a specific file or folder:** 12-Project-Files-Index.md.
- **Install MarineGym or fix Isaac Sim 5 issues:** 11-Installation-and-Fixes.md; see also **Test_AUV/docs/installation/MARINEGYM_INSTALLATION.md** for full install steps.
- **Isaac Sim 5 / Test_AUV compatibility fixes:** Test_AUV/docs (get_velocities shape, vmap, NumPy safety net, import migration, etc.); linked from 00-Overview.md and 11-Installation-and-Fixes.md.
- **TorchRL "Failed to import C++ binaries" warning:** 14-TorchRL-CPP-Extension-Warning.md (expected, left as-is; full explanation).