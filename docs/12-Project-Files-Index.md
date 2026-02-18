# Project files index

Every file and folder in MarineGym with a one-line description. See **00-Overview.md** for the doc index and **01-Package-Structure.md** for layout.

---

## Root

| File / folder | Description |
|---------------|-------------|
| **marinegym/** | Main package (robots, views, actuators, controllers, envs, learning, sensors, utils). |
| **scripts/** | train.py, evaluate.py; run from scripts/ with Hydra. |
| **cfg/** | Hydra config: task (Hover, Track, Landing, disturbances, randomization), base (sim, env). |
| **setup.py** | Package deps (torchrl, tensordict, hydra, omegaconf, etc.). |
| **README.md** | Project overview, install, usage, citation. |
| **LICENSE** | MIT. |
| **docs/** | This documentation. |
| **conda_setup/** | Conda env activate/deactivate scripts (optional). |

---

## marinegym/

| File / folder | Description |
|---------------|-------------|
| **__init__.py** | ForkingPickler patch (Isaac Sim 5); init_simulation_app; CONFIG_PATH; TensorDict shapes/devices. |
| **robots/** | RobotBase, config, drone (UnderwaterVehicle, BlueROV, BlueROVHeavy), assets. |
| **views/__init__.py** | ArticulationView, RigidPrimView; Isaac Sim 5 compatibility (get_velocities shape, NumPy). |
| **actuators/** | T200 thruster, rotor_group. |
| **controllers/** | ControllerBase, LeePositionController, cfg (lee_controller_*.yaml). |
| **envs/** | IsaacEnv base, single tasks (Hover, Track, Landing), utils (stage, prims, helpers). |
| **learning/** | PPO, modules (networks, distributions, GAE, value norm, clip_grad), ALGOS registry. |
| **sensors/** | Camera, config (camera.yaml). |
| **utils/** | torch, math, kit, scene, wandb, torchrl (collector, transforms), bspline, image, poisson_disk, envs. |

---

## marinegym/robots/

| File / folder | Description |
|---------------|-------------|
| **robot.py** | RobotBase: spawn, _create_prim, initialize, ASSET_PATH, TEMPLATE_PRIM_PATH; get_world_poses, get_velocities, etc. |
| **config.py** | RobotCfg, RigidBodyPropertiesCfg, ArticulationRootPropertiesCfg. |
| **drone/underwaterVehicle.py** | UnderwaterVehicle: param_path, initialize, apply_action, get_state, hydro (buoyancy, damping, added mass, coriolis). |
| **drone/BlueROV.py** | BlueROV class: usd_path, param_path, fixed_usd_path (main + pair). |
| **drone/BlueROVHeavy.py** | BlueROVHeavy class: usd_path, param_path, fixed_usd_path. |
| **drone/__init__.py** | Exports UnderwaterVehicle, BlueROV, BlueROVHeavy. |
| **assets/usd/BlueROV/** | BlueROV.usd(a), BlueROV.yaml, fixed_usd/BlueROV/, Props/instanceable_meshes. |
| **assets/usd/BlueROVHeavy/** | BlueROVHeavy USD and YAML (if present). |

---

## marinegym/views/

| File | Description |
|------|-------------|
| **__init__.py** | ArticulationView, RigidPrimView; get_velocities, get_world_poses, get_masses, get_inertias, etc.; Isaac Sim 5 shape/NumPy fixes. |

---

## marinegym/actuators/

| File | Description |
|------|-------------|
| **t200.py** | T200 nn.Module: throttle → rpm → thrust, moment. |
| **rotor_group.py** | RotorGroup. |

---

## marinegym/controllers/

| File / folder | Description |
|---------------|-------------|
| **controller.py** | ControllerBase. |
| **lee_position_controller.py** | LeePositionController (Lee et al.); mixer from rotor config and inertia. |
| **cfg/** | lee_controller_BlueROVHeavy.yaml, lee_controller_firefly.yaml, lee_controller_hummingbird.yaml, lee_controller_neo11.yaml. |

---

## marinegym/envs/

| File / folder | Description |
|---------------|-------------|
| **isaac_env.py** | IsaacEnv base; REGISTRY; _design_scene; reset; step; DebugDraw. |
| **utils/__init__.py** | Stage, prims, helpers; attach_payload. |
| **single/hover.py** | Hover task: _design_scene (drone, target from fixed_usd_path), rewards, obs. |
| **single/** | Other tasks (Track, Landing) and __init__.py. |
| **__init__.py** | Exports. |

---

## marinegym/learning/

| File / folder | Description |
|---------------|-------------|
| **ppo/ppo.py** | PPO algorithm. |
| **ppo/common.py** | PPO common helpers. |
| **modules/networks.py** | Actor-critic networks. |
| **modules/distributions.py** | Action distributions. |
| **modules/rnn.py** | RNN modules. |
| **utils/gae.py** | GAE (Generalized Advantage Estimation). |
| **utils/valuenorm.py** | Value normalization. |
| **utils/clip_grad.py** | Gradient clipping. |
| **__init__.py** | ALGOS registry (ppo, etc.). |

---

## marinegym/sensors/

| File | Description |
|------|-------------|
| **camera.py** | Camera sensor. |
| **camera.yaml** | Camera config. |
| **config.py** | Sensor config. |

---

## marinegym/utils/

| File | Description |
|------|-------------|
| **torch.py** | quat_rotate, quat_rotate_inverse, normalize, quaternion_to_euler, euler_to_quaternion, etc. |
| **math.py** | Math helpers. |
| **kit.py** | set_nested_rigid_body_properties, set_articulation_properties, set_nested_collision_properties. |
| **scene.py** | Scene helpers. |
| **wandb.py** | init_wandb. |
| **torchrl/collector.py** | SyncDataCollector. |
| **torchrl/transforms.py** | FromMultiDiscreteAction, FromDiscreteAction, ravel_composite, AttitudeController, RateController. |
| **torchrl/env.py** | Env helpers. |
| **bspline.py** | B-spline. |
| **image.py** | Image helpers. |
| **poisson_disk.py** | Poisson disk sampling. |
| **envs/__init__.py** | Env utilities. |
| **__init__.py** | Exports. |

---

## scripts/

| File | Description |
|------|-------------|
| **train.py** | Hydra main; init_simulation_app; REGISTRY[task]; ALGOS[algo]; SyncDataCollector; train loop; eval; checkpoint save. |
| **train.yaml** | Hydra default config (task, algo, sim, env). |
| **evaluate.py** | Load checkpoint; run policy in env; eval. |

---

## cfg/

| File / folder | Description |
|---------------|-------------|
| **task/Hover.yaml** | Hover: env (num_envs, spacing, max_episode_length), drone_model, reward weights, action_transform. |
| **task/Track.yaml** | Track task config. |
| **task/Landing.yaml** | Landing task config. |
| **task/disturbances.yaml** | Payload, flow (enable, max_flow_velocity, flow_velocity_gaussian_noise) per mode. |
| **task/randomization.yaml** | Randomization config. |
| **base/sim_base.yaml** | Sim dt, substeps, device. |
| **base/env_base.yaml** | Env num_envs, episode length, etc. |

---

## docs/

| File | Description |
|------|-------------|
| **README.md** | Entry point; complete doc index; where to go next. |
| **00-Overview.md** | Doc index, project files quick reference, references to Test_AUV/docs. |
| **01-Package-Structure.md** | Top-level layout: marinegym/, scripts/, cfg/. |
| **02-Robots.md** | RobotBase, spawn, config. |
| **03-UnderwaterVehicle.md** | UnderwaterVehicle, BlueROV, BlueROVHeavy, hydro. |
| **04-Views.md** | ArticulationView, RigidPrimView; Isaac Sim 5 compatibility. |
| **05-Actuators.md** | T200 thruster, rotor_group. |
| **06-Controllers.md** | ControllerBase, LeePositionController. |
| **07-Environments.md** | IsaacEnv, Hover and other tasks. |
| **08-Scripts-and-Training.md** | train.py, Hydra, init_simulation_app, evaluate. |
| **09-Assets.md** | BlueROV USD, YAML, usd_path / param_path / fixed_usd_path. |
| **10-Hydrodynamics.md** | Buoyancy, damping, added mass, coriolis. |
| **11-Installation-and-Fixes.md** | Installation, ForkingPickler, refs to Test_AUV/docs. |
| **12-Project-Files-Index.md** | This file: every file and folder in MarineGym. |
| **13-Questions-Learnings-and-Details.md** | FAQ, design decisions, cross-refs to Test_AUV/docs. |

---

## See also

- **00-Overview.md** — Doc index and project files quick reference.
- **01-Package-Structure.md** — Layout and roles.
