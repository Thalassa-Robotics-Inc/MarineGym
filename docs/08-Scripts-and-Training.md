# Scripts and training

**train.py** is the Hydra entrypoint for training. It starts the simulation app, loads the env from **IsaacEnv.REGISTRY**, builds the policy from **ALGOS**, and runs a SyncDataCollector loop. **evaluate.py** loads a checkpoint and runs the policy in the env. Config paths (Hydra, CWD) matter.

---

## train.py (scripts/train.py)

### Entrypoint

- **@hydra.main(version_base=None, config_path=".", config_name="train")** — Hydra loads config from **scripts/** (config_path=".") and **train.yaml** (config_name="train"). So **CWD must be scripts/** when running `python train.py ...` so that Hydra finds train.yaml and the task/base configs (cfg/task, cfg/base are referenced from train.yaml).

### main(cfg)

1. **OmegaConf** — register_new_resolver("eval", eval); resolve; set_struct(False).
2. **init_simulation_app(cfg)** — From marinegym.__init__; launches Isaac Sim (SimulationApp) with headless, livestream, window size, etc. Do **not** call this again if you already have an app (e.g. from Test_AUV run.py); use MarineGym APIs inside your existing app.
3. **init_wandb(cfg)** — W&B run.
4. **setproctitle(run.name)**.
5. **Env** — `env_class = IsaacEnv.REGISTRY[cfg.task.name]`; `base_env = env_class(cfg, headless=cfg.headless)`. So **task name** (e.g. Hover, Track, Landing) selects the env class from REGISTRY.
6. **Transforms** — InitTracker; optionally ravel_obs, ravel_obs_central; optionally action_transform (multidiscrete, discrete).
7. **Policy** — `policy = ALGOS[cfg.algo.name.lower()](cfg.algo, observation_spec, action_spec, reward_spec, device=base_env.device)`.
8. **Collector** — SyncDataCollector(env, policy, frames_per_batch, total_frames, device, return_same_td=True).
9. **evaluate()** — Inline function: enable_render(True), env.eval(), rollout with RenderCallback, log video and stats.
10. **Loop** — For each batch from collector: episode_stats.add; policy.train_op; optional evaluate every eval_interval; optional save checkpoint every save_interval; run.log(info). Exit when max_iters or total_frames reached.
11. **Save final checkpoint** — checkpoint_final.pt; wandb artifact.
12. **simulation_app.close()**.

### Command example

```bash
cd MarineGym/scripts
python train.py task=Hover algo=ppo headless=false enable_livestream=false
```

With Isaac Sim’s Python (e.g. from Test_AUV or MarineGym install):

```bash
/path/to/_isaac_sim/python.sh -B train.py task=Hover algo=ppo headless=false
```

CWD must be **scripts/** so Hydra finds train.yaml and cfg.

---

## evaluate.py (scripts/evaluate.py)

Loads a checkpoint (policy state_dict), builds env and policy, runs rollout (eval mode), logs or saves results. See source for exact CLI and logging.

---

## Config paths

- **train.yaml** — Default task, algo, sim, env; can override from CLI (task=Hover, algo=ppo, etc.).
- **cfg/task/** — Hover.yaml, Track.yaml, Landing.yaml, disturbances.yaml, randomization.yaml. Task-specific defaults (num_envs, drone_model, rewards, action_transform).
- **cfg/base/** — sim_base.yaml, env_base.yaml. Sim (dt, substeps, device), env (num_envs, max_episode_length).
- **CONFIG_PATH** (marinegym.__init__) — os.path.join(marinegym dir, "cfg"). Used by code that loads config relative to package.

If you run from **another project** (e.g. Test_AUV) and only **import** MarineGym (drone, physics) and drive the scene from your own script, you do **not** need Hydra or train.yaml; paths (usd_path, param_path) are set on the drone class or subclass. If you run **train.py** from Test_AUV, CWD and config_path would need to point at MarineGym/scripts and MarineGym/cfg for Hydra to resolve correctly.

---

## See also

- **07-Environments.md** — REGISTRY, task name.
- **11-Installation-and-Fixes.md** — init_simulation_app, ForkingPickler.
- **00-Overview.md** — References to Test_AUV/docs (simulation loop, one app).
