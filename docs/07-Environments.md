# Environments

**IsaacEnv** is the base class for all MarineGym tasks. Tasks (Hover, Track, Landing) register in **IsaacEnv.REGISTRY** and are instantiated by **train.py** via **cfg.task.name**. This doc covers the env base, Hover, _design_scene, spawn, and the use of **fixed_usd_path** for the target visual.

---

## IsaacEnv (marinegym/envs/isaac_env.py)

- **env_ns** — `/World/envs`.
- **template_env_ns** — `/World/envs/env_0`.
- **REGISTRY** — Dict[str, Type[IsaacEnv]]; tasks register via __init_subclass__.

### __init__(cfg, headless)

- Calls EnvBase with device=cfg.sim.device, batch_size=[cfg.env.num_envs].
- Stores cfg; enable_render, enable_viewport; num_envs, max_episode_length, substeps; cudnn settings.
- Checks simulation is running; creates SimulationContext; calls **_design_scene()** (abstract).

### _design_scene()

Abstract. Subclasses create the world: ground, lights, **drone** (create + spawn), optional payload, optional **target** visual. Target is often a **visual-only** copy of the robot using **drone.fixed_usd_path**.

### Lifecycle

- **reset** — Resets simulation, clones envs (GridCloner), sets camera; calls _reset_idx on drone and optional payload; returns observation TensorDict.
- **step** — Drone get_state; policy/controller action; drone apply_action; simulation step; reward, done, obs; optional stats.

---

## Hover (marinegym/envs/single/hover.py)

- **defaults** in cfg: env_base, sim_base, randomization, disturbances (see **cfg/task/Hover.yaml**).
- **drone_model** — name (e.g. BlueROV), controller (e.g. LeePositionController).
- **force_sensor** — Whether to use force sensor in state.
- **time_encoding** — Whether to encode time in obs.
- **reward_effort_weight**, **reward_action_smoothness_weight**, **reward_distance_scale** — Reward weights.
- **action_transform** — Optional (e.g. multidiscrete, discrete) for train.py.

### _design_scene()

1. **Drone** — `UnderwaterVehicle.make(drone_model_cfg.name, drone_model_cfg.controller)` → drone, controller.
2. **Target visual** — `prim_utils.create_prim(prim_path="/World/envs/env_0/target", usd_path=self.drone.fixed_usd_path, translation=(0, 0, 2))`. Collision disabled; disable_gravity=True. So **fixed_usd_path** is used **only** for the target visual (pair USD), not for the simulated drone.
3. **Drone spawn** — `self.drone.spawn(translations=[(0, 0, 2)])` (single env template); then GridCloner clones to num_envs.
4. Optional **payload** — RigidPrimView for payload if disturbances enable_payload.

### After __init__

- **drone.initialize()** (no args → uses default prim_paths_expr for cloned envs).
- **target_vis.initialize()** — ArticulationView for "/World/envs/env_*/target".
- **init_poses**, **init_vels** — From drone get_world_poses, get_velocities.
- **init_pos_dist**, **init_rpy_dist**, **target_rpy_dist** — For randomizing start and target orientation.
- **target_pos**, **target_heading** — Target pose for reward/controller.

---

## Other tasks

- **Track**, **Landing** — Same base (IsaacEnv); different _design_scene, rewards, and observation/reward logic. In this repo, **Track and Landing are commented out** in `marinegym/envs/single/__init__.py` (`# from .track import Track`, `# from .landing import Landing`), so only **Hover** is currently active. cfg/task/Track.yaml and Landing.yaml exist; to use Track/Landing you must uncomment the imports and ensure the corresponding env modules (track.py, landing.py) exist. See **13-Questions-Learnings-and-Details.md**.

---

## See also

- **08-Scripts-and-Training.md** — train.py loads env from REGISTRY[cfg.task.name].
- **09-Assets.md** — fixed_usd_path (pair) vs usd_path (main).
- **03-UnderwaterVehicle.md** — UnderwaterVehicle.make.
