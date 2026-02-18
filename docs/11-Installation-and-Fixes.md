# Installation and fixes

How to install MarineGym and which fixes are required for **Isaac Sim 5** (and when using MarineGym from **Test_AUV**). The **detailed** installation steps, ForkingPickler, physics fix, and demos are in **Test_AUV/docs/installation/MARINEGYM_INSTALLATION.md**; this doc summarizes and points to Test_AUV/docs for all Isaac Sim 5–specific fixes.

---

## Installation (source, with Isaac Sim)

1. **Isaac Sim** — Must be available (workstation install or source build). MarineGym launches or uses Isaac Sim’s Python and physics.
2. **Clone MarineGym** — e.g. `git clone https://github.com/Marine-RL/MarineGym.git`.
3. **Install deps** — Use **Isaac Sim’s Python**: `path/to/isaac_sim/python.sh -m pip install -e /path/to/MarineGym`. This installs torchrl, tensordict, hydra, omegaconf, etc. (see setup.py). Do **not** upgrade torchrl/tensordict blindly; check [MarineGym FAQ](https://marinegym.netlify.app/faq).
4. **Run** — From MarineGym/scripts: `path/to/isaac_sim/python.sh -B train.py task=Hover algo=ppo headless=false`. Or use MarineGym APIs from another app (e.g. Test_AUV run.py) without calling init_simulation_app again.

Official guides: [Installation from Source](https://marinegym.netlify.app/installation_from_source), [Docker](https://marinegym.netlify.app/docker_environment).

---

## ForkingPickler (Isaac Sim 5.0)

**Problem** — `import marinegym` can fail with: `ImportError: cannot import name 'ForkingPickler' from 'torch.multiprocessing.reductions'` because tensordict expects it and Isaac Sim 5’s bundled torch may not expose it.

**Fix** — In **marinegym/__init__.py**, after `import torch` and **before** `from tensordict import TensorDict`, add:

```python
if not getattr(torch.multiprocessing.reductions, "ForkingPickler", None):
    from multiprocessing.reduction import ForkingPickler
    torch.multiprocessing.reductions.ForkingPickler = ForkingPickler
```

This is already in the MarineGym codebase. See **Test_AUV/docs/installation/MARINEGYM_INSTALLATION.md** (Critical Fix: ForkingPickler).

---

## Isaac Sim 5 compatibility fixes (reference only)

The following fixes are **implemented in MarineGym** (views, underwaterVehicle); the **documentation** of what was fixed and why lives in **Test_AUV/docs** so we don’t duplicate it here. If you maintain a fork or need to re-apply them:

| Issue | Fix location | Documented in (Test_AUV/docs) |
|-------|----------------|-------------------------------|
| get_velocities shape (6,) for single articulation | marinegym/views/__init__.py (ArticulationView, RigidPrimView get_velocities) | 24-MarineGym-IsaacSim5-get-velocities-shape-fix.md |
| functorch.vmap deprecated | marinegym/robots/drone/underwaterVehicle.py (use torch.vmap) | 25-MarineGym-PyTorch-vmap-fix.md |
| NumPy outputs from physics views | marinegym/views/__init__.py (_ensure_tensor in getters) | 18-MarineGym-IsaacSim5-numpy-outputs-safety-net.md |

Other Isaac Sim 5 fix docs (get_world_poses, get_gains, get_masses, get_inertias, etc.) are in Test_AUV/docs (11–21). For **findings and learnings** where no code change was made (e.g. water.usda revert, TorchRL/Warp warnings), see **Test_AUV/docs/31-Findings-and-Learnings-Log.md** and **30-Unfixable-System-Warnings.md**.

---

## TorchRL C++ extension warning (expected, left as-is)

When running under Isaac Sim you may see: `UserWarning: Failed to import torchrl C++ binaries. Some modules (eg, prioritized replay buffers) may not work.` This is **expected and harmless** for MarineGym: the optional C++ extension (for prioritized replay only) fails to load due to an ABI mismatch with Isaac Sim's PyTorch; we don't use that feature and we don't suppress the warning so other messages remain visible for debugging. Full explanation (what it is, why it happens, why we leave it): **14-TorchRL-CPP-Extension-Warning.md**. Technical detail: **Test_AUV/docs/30-Unfixable-System-Warnings.md** §4.

---

## Using MarineGym from Test_AUV

- **One app** — Do not call MarineGym’s **init_simulation_app()** if you already started the app in Test_AUV (run.py). Use MarineGym’s drone and physics inside your existing SimulationApp.
- **Extensions** — Test_AUV run.py enables OceanSim and others; enable what you need in one place.
- **Paths** — To use Test_AUV’s copied BlueROV assets, set **usd_path** and **param_path** on the drone class (or a Test_AUV subclass) before creating the drone. See **Test_AUV/docs/09-Standalone-BlueROV-Setup.md**, **08-BlueROV-Asset-and-MarineGym-Integration.md**.
- **Simulation loop** — get_state → apply_action → world.step; see **Test_AUV/docs/27-Simulation-Loop-and-Zero-Action.md**.

---

## See also

- **00-Overview.md** — References to Test_AUV/docs.
- **Test_AUV/docs/installation/MARINEGYM_INSTALLATION.md** — Full installation and demos.
- **08-Scripts-and-Training.md** — train.py, config paths.
