# 13 – Questions, learnings, and details

This doc is a **reference and learning index** for MarineGym: answers to common questions, design decisions, fun facts, and details that aren't obvious from the code. Use it for replication, debugging, and integration (e.g. with Test_AUV or OceanSim).

---

## What's actually in the repo

| Question | Answer |
|----------|--------|
| Which tasks (envs) are **active**? | Only **Hover**. Track and Landing are **commented out** in `marinegym/envs/single/__init__.py` (`# from .track import Track`, `# from .landing import Landing`), so they are not loaded. REGISTRY and cfg/task still have entries for them; uncomment and implement the modules to use them. |
| Which robots are implemented? | **BlueROV** and **BlueROVHeavy** (both in `marinegym/robots/drone/`). No HAUV, LAUV, or iAUV in this repo. |
| What does the website show vs repo? | The MarineGym site may describe Track, Landing, or other vehicles; the **repo** currently ships Hover + BlueROV/BlueROVHeavy only. |

---

## Design and usage

| Question | Answer |
|----------|--------|
| Why **ForkingPickler** in `marinegym/__init__.py`? | Isaac Sim 5's bundled Python/torch can omit `torch.multiprocessing.reductions.ForkingPickler`. TensorDict (used by TorchRL) expects it when spawning processes. The patch copies it from `multiprocessing.reduction` so training/collectors don't crash. See **11-Installation-and-Fixes.md** and **Test_AUV/docs/installation/MARINEGYM_INSTALLATION.md**. |
| Why **main** vs **pair** BlueROV USD? | **Main** = full collision and inertia in USD; used for the simulated drone. **Pair** = root_joint, lighter; used in Hover only for the **target visual** (fixed_usd_path). See **09-Assets.md** and **Test_AUV/docs/08-BlueROV-Asset-and-MarineGym-Integration.md**. |
| Where is **CONFIG_PATH** used? | `marinegym/__init__.py` sets `CONFIG_PATH = os.path.join(marinegym dir, "cfg")`. Used when loading Hydra/task config relative to the package (e.g. in train.py or env defaults). |
| Why **CWD = scripts/** for train.py? | Hydra's `config_path="."` and `config_name="train"` resolve relative to CWD; train.yaml and `cfg/` are under MarineGym, so you run from `MarineGym/scripts` so Hydra finds them. See **08-Scripts-and-Training.md**. |

---

## Controllers and configs

| Question | Answer |
|----------|--------|
| What are the **lee_controller_*.yaml** files? | Lee position controller configs per vehicle: **BlueROV**, **BlueROVHeavy**, **firefly**, **hummingbird**, **neo11**. Each defines mixer (rotor layout), gains, and inertia. Only BlueROV (and BlueROVHeavy) are used in this repo; the others are for other quad/hex vehicles the codebase once supported. |
| How does **UnderwaterVehicle.make(name, controller)** work? | It imports the robot class by name (e.g. BlueROV, BlueROVHeavy) and the controller class (e.g. LeePositionController), loads the controller config YAML for that robot, and returns (drone_instance, controller_instance). |

---

## Learning and training

| Question | Answer |
|----------|--------|
| Where is **PPO** and the **ALGOS** registry? | `marinegym/learning/` — `ppo/ppo.py`, `ppo/common.py`; `modules/` (networks, distributions, rnn); `utils/` (gae.py, valuenorm.py, clip_grad.py). `learning/__init__.py` exposes **ALGOS** (e.g. ppo) for train.py. |
| What is **SyncDataCollector**? | TorchRL-style collector that runs env.step in a loop (no multiprocessing in the default path). Defined in `marinegym/utils/torchrl/collector.py`; used by train.py. |
| What is **AgentSpec** / **fake_tensordict**? | In `marinegym/utils/torchrl/env.py` and used by IsaacEnv: AgentSpec describes observation/action/reward specs for the agent; fake_tensordict() returns a TensorDict with the right shapes for testing. See isaac_env.py. |

---

## TorchRL C++ extension warning

| Question | Answer |
|----------|--------|
| Why do I see "Failed to import torchrl C++ binaries"? | TorchRL tries to load an optional C++ plugin at import time; it was built with a different C++ ABI than Isaac Sim's PyTorch, so the load fails and TorchRL prints a warning. MarineGym doesn't use that plugin (only prioritized replay buffers need it). We leave the warning as-is. Full explanation: **14-TorchRL-CPP-Extension-Warning.md**. |

---

## Files and layout (quick facts)

| Topic | Detail |
|-------|--------|
| **conda_setup/** | Optional conda env activate/deactivate scripts; not required if you use Isaac Sim's Python or a single env. |
| **learning/utils/** | GAE, ValueNorm, clip_grad live here (not under learning/modules/). See **12-Project-Files-Index.md**. |
| **omni.isaac.dynamic_control** | Deprecated in Isaac Sim 5 with **no** replacement. Still imported in `marinegym/envs/utils/prims.py` (try/except); used for some articulation helpers. Not in the Test_AUV code path. |
| **Isaac Sim 5 imports** | MarineGym was migrated from `omni.isaac.*` to `isaacsim.*` to remove deprecation warnings. Full mapping: **Test_AUV/docs/29-MarineGym-IsaacSim5-Import-Migration.md**. |

---

## Findings (no code change in MarineGym)

These are learnings that did not require changing MarineGym code but are useful to know:

| Finding | Learning |
|--------|----------|
| Test_AUV runs successfully with MarineGym + OceanSim after the import migration (doc 29). | Scene loads, AUV spawns, sim loop runs. Remaining terminal messages are from Isaac Sim internals or external deps (Test_AUV/docs/30). |
| We did not add `primvars:st:indices` to any USD assets. | A scripted attempt on water.usda broke USDA syntax; we reverted. The "primvars:st:indices not found" warning is cosmetic; MarineGym's BlueROV assets also lack indices and are left as-is. |
| TorchRL C++ binaries fail to load — **ABI mismatch**. | `_torchrl.so` (TorchRL 0.4.0) was compiled with old C++ ABI; Isaac Sim's PyTorch with new ABI. Symbol mismatch → dlopen fails. Only prioritized replay buffers are affected; MarineGym does not use them. We leave the warning (no suppression, so we don't miss other warnings). Full explanation: **14-TorchRL-CPP-Extension-Warning.md**. Technical detail: Test_AUV/docs/30 §4. |
| Warp CUDA errors (`cuDeviceGetUuid` not found) then successful init. | Warp 1.7.1 compiled against CUDA 12.8; driver 580.95.05 (CUDA 13.0) doesn't expose the exact entry point. Warp falls back. Fix: driver or Warp update. See Test_AUV/docs/30 §5. |
| 27 of 28 `omni.isaac.*` deprecation warnings eliminated by editing Isaac Sim's `.kit` files and migrating OceanSim. | Removed deprecated extension deps and `extsDeprecated` search path from `isaacsim.exp.base.kit` and `omni.isaac.sim.python.kit`. Migrated OceanSim to `isaacsim.gui.components`/`isaacsim.core.api`. Only `omni.isaac.dynamic_control` remains (hard dep of `isaacsim.core.utils`). See Test_AUV/docs/32. |
| `pxr.Semantics is deprecated` warning from `semantics.schema.property` extension. | NVIDIA-internal USD schema deprecation. Nothing we can change. See Test_AUV/docs/30 §6. |
| Remaining Kit/renderer/carb warnings (IOMMU, CCD, audio, hydra, fabric, replicator, gpu.foundation). | All internal to Isaac Sim/Kit. None fixable from user code. Full breakdown: Test_AUV/docs/30 §1-10. |

Full log with "change made? yes/no": **Test_AUV/docs/31-Findings-and-Learnings-Log.md**.

---

## Cross-references (Test_AUV/docs)

When using MarineGym with **Isaac Sim 5** or from **Test_AUV**, these are documented in **Test_AUV/docs** (not duplicated here):

- get_velocities shape fix, torch.vmap, NumPy safety net, gravity/hydro testing, simulation loop and zero-action, installation, BlueROV path override, unfixable system warnings. See **MarineGym/docs/00-Overview.md** and **11-Installation-and-Fixes.md**.
- Isaac Sim `.kit` file and OceanSim deprecation fix: **Test_AUV/docs/32-Isaac-Sim-Kit-and-OceanSim-Deprecation-Fix.md**.
- **Findings and learnings log** (including no-change cases): **Test_AUV/docs/31-Findings-and-Learnings-Log.md**.
