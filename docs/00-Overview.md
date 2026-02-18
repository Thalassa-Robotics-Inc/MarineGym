# MarineGym documentation overview

This folder documents **every part of MarineGym**: package structure, robots, underwater vehicle dynamics, views, actuators, controllers, environments, training scripts, assets, hydrodynamics, and installation/fixes. Use it to replicate behavior, integrate MarineGym into another project (e.g. Test_AUV), or debug issues.

We document **both** code/asset **changes** and **findings and learnings** -- including cases where **no change was made** in MarineGym (e.g. "this warning comes from Isaac Sim; we don't fix it here"). See **13-Questions-Learnings-and-Details.md** for FAQ and design details; for a log of findings (including Test_AUV water/sim run, TorchRL/Warp, etc.), see **Test_AUV/docs/31-Findings-and-Learnings-Log.md**.

**Official resources:** [MarineGym docs](https://marinegym.netlify.app/), [GitHub](https://github.com/Marine-RL/MarineGym).
**Isaac Sim 5 / Test_AUV fixes:** Many compatibility fixes (get_velocities shape, vmap, NumPy safety net) are documented in **Test_AUV/docs** (see [11-Installation-and-Fixes](11-Installation-and-Fixes.md)).

---

## Complete doc index (every file in docs/)

Every document in `docs/` with a one-line description:

| File | What it covers |
|------|----------------|
| **README.md** | Entry point and complete doc index; where to go next. |
| **00-Overview.md** | This file: doc index, project files quick reference, references to Test_AUV/docs. |
| **01-Package-Structure.md** | Top-level layout: marinegym/, scripts/, cfg/. |
| **02-Robots.md** | RobotBase, spawn, _create_prim, initialize, config (RobotCfg, rigid/articulation props). |
| **03-UnderwaterVehicle.md** | UnderwaterVehicle, param_path, YAML, apply_action, get_state, hydro; BlueROV / BlueROVHeavy. |
| **04-Views.md** | ArticulationView, RigidPrimView; Isaac Sim 5 compatibility (get_velocities shape, NumPy). |
| **05-Actuators.md** | T200 thruster (throttle, rpm, thrust, moment); rotor_group. |
| **06-Controllers.md** | ControllerBase; LeePositionController; controller config files. |
| **07-Environments.md** | IsaacEnv; Hover and other tasks; _design_scene, spawn, target (fixed_usd_path). |
| **08-Scripts-and-Training.md** | train.py, Hydra, init_simulation_app, env registry, evaluate; CWD and config paths. |
| **09-Assets.md** | BlueROV USD (main vs pair), BlueROV.yaml, ASSET_PATH; usd_path / param_path / fixed_usd_path. |
| **10-Hydrodynamics.md** | Buoyancy, damping, added mass, coriolis; apply_hydrodynamic_forces; gravity (or not). |
| **11-Installation-and-Fixes.md** | Installation (source, Isaac Sim Python); ForkingPickler (Isaac Sim 5); refs to Test_AUV/docs. |
| **12-Project-Files-Index.md** | Every file and folder in MarineGym with a one-line description. |
| **13-Questions-Learnings-and-Details.md** | FAQ, design decisions, what's in the repo (Hover active; Track/Landing commented out), ForkingPickler, main vs pair, cross-refs to Test_AUV/docs. Includes TorchRL ABI root cause, .kit fix, OceanSim migration findings. |
| **14-TorchRL-CPP-Extension-Warning.md** | TorchRL C++ extension warning: what it is, why it happens (ABI mismatch), who triggers it, why we leave it, no suppression; full explanation and summary table. |

---

## Documentation index

| Doc | What it covers |
|-----|----------------|
| **README.md** | Entry point; complete doc index; where to go next. |
| **00-Overview.md** | This file: doc index, project files quick reference. |
| **01-Package-Structure.md** | Top-level layout: `marinegym/` (robots, views, actuators, controllers, envs, learning, sensors, utils), `scripts/`, `cfg/`. |
| **02-Robots.md** | RobotBase: spawn, _create_prim, initialize, usd_path; ArticulationView/RigidPrimView; config (RobotCfg, rigid/articulation props). |
| **03-UnderwaterVehicle.md** | UnderwaterVehicle: param_path, YAML, initialize (base_link, rotors, T200), apply_action, get_state, _reset_idx; BlueROV / BlueROVHeavy. |
| **04-Views.md** | ArticulationView and RigidPrimView; Isaac Sim 5 compatibility (get_velocities shape, NumPy); reference to Test_AUV/docs. |
| **05-Actuators.md** | T200 thruster model (throttle, rpm, thrust/moment); rotor_group. |
| **06-Controllers.md** | ControllerBase; LeePositionController; controller config files (BlueROV, etc.). |
| **07-Environments.md** | IsaacEnv; Hover (and other tasks); _design_scene, spawn, target visual (fixed_usd_path). |
| **08-Scripts-and-Training.md** | train.py, Hydra, init_simulation_app, env registry, evaluate; CWD and config paths. |
| **09-Assets.md** | BlueROV USD (main vs fixed pair), BlueROV.yaml layout, ASSET_PATH; what usd_path / param_path / fixed_usd_path are. |
| **10-Hydrodynamics.md** | Buoyancy, damping, added mass, coriolis; apply_hydrodynamic_forces; flow_vels; where gravity is used (or not). |
| **11-Installation-and-Fixes.md** | Installation (source, Isaac Sim Python); ForkingPickler (Isaac Sim 5); references to Test_AUV/docs for get_velocities, vmap, numpy fixes. |
| **12-Project-Files-Index.md** | Every file and folder in MarineGym with a one-line description. |
| **13-Questions-Learnings-and-Details.md** | FAQ, fun facts, design decisions: what's actually in the repo (only Hover active; Track/Landing commented out), ForkingPickler, main vs pair, controller configs, learning layout, cross-refs to Test_AUV/docs. Includes TorchRL ABI root cause, .kit fix, OceanSim migration findings. |
| **14-TorchRL-CPP-Extension-Warning.md** | TorchRL C++ extension warning: what it is, ABI mismatch, who triggers it, why we leave it (no suppression), full explanation; see also Test_AUV/docs/30 §4. |

---

## Project files quick reference

| File or folder | What it is | Documented in |
|----------------|------------|---------------|
| **marinegym/__init__.py** | Package init; ForkingPickler patch; init_simulation_app; CONFIG_PATH | 01, 08, 11 |
| **marinegym/robots/robot.py** | RobotBase, spawn, _create_prim, initialize, ASSET_PATH, TEMPLATE_PRIM_PATH | 02 |
| **marinegym/robots/config.py** | RobotCfg, RigidBodyPropertiesCfg, ArticulationRootPropertiesCfg | 02 |
| **marinegym/robots/drone/underwaterVehicle.py** | UnderwaterVehicle, apply_action, get_state, hydro (buoyancy, damping, added mass, coriolis) | 03, 10 |
| **marinegym/robots/drone/BlueROV.py** | BlueROV class (usd_path, param_path, fixed_usd_path) | 03, 09 |
| **marinegym/robots/drone/BlueROVHeavy.py** | BlueROVHeavy class | 03, 09 |
| **marinegym/robots/assets/usd/BlueROV/** | BlueROV.usd(a), fixed_usd/, BlueROV.yaml, Props/instanceable_meshes | 09 |
| **marinegym/views/__init__.py** | ArticulationView, RigidPrimView; get_velocities, get_world_poses, etc. | 04 |
| **marinegym/actuators/t200.py** | T200 thruster (throttle, rpm, thrust, moment) | 05 |
| **marinegym/actuators/rotor_group.py** | RotorGroup | 05 |
| **marinegym/controllers/lee_position_controller.py** | LeePositionController | 06 |
| **marinegym/controllers/controller.py** | ControllerBase | 06 |
| **marinegym/envs/isaac_env.py** | IsaacEnv base, REGISTRY | 07 |
| **marinegym/envs/single/hover.py** | Hover task; _design_scene, target, spawn | 07 |
| **marinegym/learning/** | PPO, modules (networks, distributions, GAE, etc.) | 01 |
| **marinegym/sensors/** | Camera, config | 01 |
| **marinegym/utils/** | torch, math, kit, scene, wandb, torchrl, etc. | 01, 08 |
| **scripts/train.py** | Hydra main, init_simulation_app, env_class from REGISTRY, training loop | 08 |
| **scripts/train.yaml** | Hydra config (task, algo, sim, etc.) | 08 |
| **scripts/evaluate.py** | Evaluation entrypoint | 08 |
| **cfg/** | task (Hover, Track, Landing, disturbances, randomization), base (sim, env) | 07, 08 |
| **setup.py** | Package deps (torchrl, tensordict, hydra, etc.) | 11 |
| **docs/README.md** | Entry point; complete doc index; where to go next | -- |
| **docs/13-Questions-Learnings-and-Details.md** | FAQ, fun facts, "what's in the repo" (only Hover active), ForkingPickler, controller configs | 13 |

---

## References to Test_AUV/docs

When using MarineGym with **Isaac Sim 5** or from **Test_AUV**, the following fixes and concepts are documented in **Test_AUV/docs** (not duplicated here):

- **get_velocities shape:** Isaac Sim can return `(6,)` for a single articulation; MarineGym views reshape to `(1, 6)` before unflatten. See **Test_AUV/docs/24-MarineGym-IsaacSim5-get-velocities-shape-fix.md**.
- **torch.vmap:** Use `torch.vmap` instead of deprecated `functorch.vmap` in underwaterVehicle.py. See **Test_AUV/docs/25-MarineGym-PyTorch-vmap-fix.md**.
- **NumPy outputs safety net:** Several view getters (get_velocities, get_masses, etc.) can return NumPy; MarineGym views convert to tensor before Torch ops. See **Test_AUV/docs/18-MarineGym-IsaacSim5-numpy-outputs-safety-net.md**.
- **Gravity and hydro testing:** Buoyancy uses hardcoded 997*9.81*volume; scene gravity can be set to zero for hydro-only tests. See **Test_AUV/docs/26-Gravity-and-Hydro-Testing.md**.
- **Simulation loop and zero-action:** get_state -> apply_action -> world.step; zero_action pattern. See **Test_AUV/docs/27-Simulation-Loop-and-Zero-Action.md**.
- **Full installation and demos:** Step-by-step MarineGym install, ForkingPickler, physics fix, teleop/water demos. See **Test_AUV/docs/installation/MARINEGYM_INSTALLATION.md**.
- **BlueROV asset choice (main vs pair), path override:** See **Test_AUV/docs/08-BlueROV-Asset-and-MarineGym-Integration.md**, **09-Standalone-BlueROV-Setup.md**.
- **Unfixable system warnings (IOMMU, Warp, TorchRL ABI, pxr.Semantics, etc.):** **Test_AUV/docs/30-Unfixable-System-Warnings.md**. For the TorchRL C++ extension warning specifically (what it is, why it happens, why we leave it): **14-TorchRL-CPP-Extension-Warning.md**.
- **Isaac Sim .kit file and OceanSim deprecation fix (27 warnings eliminated):** **Test_AUV/docs/32-Isaac-Sim-Kit-and-OceanSim-Deprecation-Fix.md**.
- **Findings and learnings log** (including "no change" cases: water/primvars, sim run, TorchRL/Warp, .kit fix, OceanSim migration): **Test_AUV/docs/31-Findings-and-Learnings-Log.md**.
