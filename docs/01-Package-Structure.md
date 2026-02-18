# Package structure

Top-level layout of the MarineGym repository and what each part does. See **00-Overview.md** for the doc index.

---

## Repository layout

```
MarineGym/
├── marinegym/              # Main package
│   ├── __init__.py         # ForkingPickler patch, init_simulation_app, CONFIG_PATH
│   ├── robots/             # Robot base, config, drone (UnderwaterVehicle, BlueROV)
│   ├── views/               # ArticulationView, RigidPrimView (Isaac Sim view wrappers)
│   ├── actuators/           # T200 thruster, rotor_group
│   ├── controllers/         # LeePositionController, controller base, cfg
│   ├── envs/                # IsaacEnv base, single tasks (Hover, Track, Landing)
│   ├── learning/            # PPO, modules (networks, distributions, GAE, clip_grad)
│   ├── sensors/             # Camera, config
│   └── utils/               # torch, math, kit, scene, wandb, torchrl, bspline, etc.
├── scripts/                 # train.py, evaluate.py; run from scripts/ with Hydra
├── cfg/                     # Hydra config: task (Hover, Track, Landing, disturbances), base (sim, env)
├── setup.py                 # Package deps (torchrl, tensordict, hydra, omegaconf, etc.)
├── README.md
├── LICENSE
└── docs/                    # This documentation
```

---

## marinegym/ subpackages

| Subpackage | Role |
|------------|------|
| **robots** | RobotBase (spawn, _create_prim, initialize); UnderwaterVehicle (hydro, thrusters, get_state, apply_action); BlueROV / BlueROVHeavy; config (RobotCfg, rigid/articulation props). See **02-Robots.md**, **03-UnderwaterVehicle.md**. |
| **views** | ArticulationView and RigidPrimView wrap Isaac Sim physics views; provide get_velocities, get_world_poses, get_masses, get_inertias, etc., with batch shapes and (where applied) Isaac Sim 5 compatibility. See **04-Views.md**. |
| **actuators** | T200 thruster model (throttle → rpm → thrust/moment); RotorGroup. See **05-Actuators.md**. |
| **controllers** | ControllerBase; LeePositionController (position/velocity/yaw target → rotor commands); YAML configs per robot. See **06-Controllers.md**. |
| **envs** | IsaacEnv base class; task registry (Hover, Track, Landing); _design_scene, reset, step; AgentSpec. See **07-Environments.md**. |
| **learning** | PPO algorithm; networks, distributions, GAE, value norm, clip grad; ALGOS registry. Used by train.py. |
| **sensors** | Camera sensor and config (camera.yaml). |
| **utils** | torch (quat_rotate, normalize, etc.), math, kit (set_rigid_body_properties, set_articulation_properties), scene, wandb, torchrl (collector, transforms), bspline, image, poisson_disk, envs. |

---

## scripts/

- **train.py** — Hydra entrypoint (`config_path="."`, `config_name="train"`); init_simulation_app, init_wandb, env from REGISTRY, policy from ALGOS, SyncDataCollector, train loop, eval, checkpoint save. Run from **MarineGym/scripts** (CWD matters for Hydra). See **08-Scripts-and-Training.md**.
- **evaluate.py** — Evaluation script; loads checkpoint and runs policy in env.

---

## cfg/

- **cfg/task/** — Hover.yaml, Track.yaml, Landing.yaml, disturbances.yaml, randomization.yaml. Task name, drone_model, rewards, obs/action config.
- **cfg/base/** — sim_base.yaml, env_base.yaml. Sim dt, substeps, device; env num_envs, episode length, etc.
- **scripts/train.yaml** — Hydra default config; composes task and base.

---

## See also

- **02-Robots.md** — Robots in detail.
- **08-Scripts-and-Training.md** — train.py and config paths.
- **12-Project-Files-Index.md** — Every file listed.
