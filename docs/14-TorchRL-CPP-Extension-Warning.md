# 14 – TorchRL C++ extension warning (known, left as-is)

When running MarineGym under Isaac Sim, you see a warning (or stderr message) like:

```
UserWarning: Failed to import torchrl C++ binaries. Some modules (eg, prioritized replay buffers) may not work with your installation.
```

This doc explains what it is, why it happens, and why we leave it as-is. All of this is also summarized in **Test_AUV/docs/30-Unfixable-System-Warnings.md** §4 (with technical details); here it is collected for MarineGym users.

---

## What is the C++ extension?

TorchRL has an **optional** C++ plugin (`_torchrl.so`) that provides fast segment trees used only for **prioritized replay buffers**. The rest of TorchRL (specs, envs, PPO, collectors, normal replay buffers) works in pure Python and does not need this plugin.

---

## Does MarineGym need TorchRL?

**Yes.** MarineGym depends on TorchRL for:

- Specs (`TensorSpec`, `CompositeSpec`, etc.) in robots, envs, controllers
- Envs (`EnvBase`, `TransformedEnv`, etc.) and `IsaacEnv`
- Collectors (`SyncDataCollector`, etc.) and training utilities
- PPO and learning modules

Removing the TorchRL import would break MarineGym.

---

## Does MarineGym (or Isaac Sim, or OceanSim) use the C++ extension?

**No.** Nothing in MarineGym, Isaac Sim, or OceanSim uses prioritized replay buffers. The C++ extension is only required for that feature. So the extension is unused in this stack; everything we need from TorchRL works without it.

---

## What triggers the warning?

- **Isaac Sim** does not use or import TorchRL.
- **MarineGym** imports TorchRL (e.g. in `robot.py`, `underwaterVehicle.py`, `isaac_env.py`, collectors, PPO). So when you run a scene or script that uses MarineGym, TorchRL gets loaded.
- **TorchRL itself** tries to load the C++ extension: in `torchrl/data/replay_buffers/samplers.py`, at **import time**, it runs `from torchrl._torchrl import ...`. If that fails (e.g. ABI mismatch), it catches the exception and prints the warning. So the attempt happens whenever TorchRL’s replay-buffer/samplers code is imported, not only when you use prioritized replay. You cannot “opt out” of that import from MarineGym’s side.

So: MarineGym is why TorchRL is in the process; TorchRL’s own code is what tries to load the extension and emits the warning.

---

## Why doesn’t the C++ extension load? (ABI mismatch)

**ABI** = Application Binary Interface: the rules that compiled code uses to talk to other compiled code (how function names become symbols, how arguments are passed, etc.). If two libraries were built with different ABIs, the same logical function can have a different “name tag,” and the linker will say “undefined symbol” and refuse to load.

What’s going on:

- TorchRL’s pre-built `_torchrl.so` (e.g. 0.4.0) was compiled with the **old** GCC C++ ABI (`_GLIBCXX_USE_CXX11_ABI=0`).
- Isaac Sim’s PyTorch (e.g. 2.10.0+cu128) was compiled with the **new** C++ ABI (`_GLIBCXX_USE_CXX11_ABI=1`).
- The plugin expects a symbol like `torch::autograd::Node::name() const` with the old ABI mangling; PyTorch only exports it with the new ABI mangling. So the dynamic linker fails with “undefined symbol” and the C++ extension does not load.

You cannot fix this from MarineGym or by “changing TorchRL version” on PyPI: the issue is **how** the binary was built, not the version number. Other versions from PyPI are typically built the same way and have the same mismatch.

---

## How could it be fixed?

The only way to make the warning go away by actually fixing the load (not hiding the message) is to have a TorchRL whose C++ extension was built with the **same** ABI as Isaac Sim’s PyTorch:

1. **Build TorchRL from source** against Isaac Sim’s Python and PyTorch, with `_GLIBCXX_USE_CXX11_ABI=1`.
2. **Use a pre-built wheel** of TorchRL that was built that way for a PyTorch compatible with Isaac Sim (if such a wheel exists; none is documented here).

We do not do either in this project; we leave the warning as-is (see below).

---

## Why we leave it as-is

- **We don’t suppress the warning** — Suppressing warnings (e.g. with `warnings.filterwarnings`) could hide other, important messages. For debugging we want to see all warnings, so we do not add a filter.
- **We don’t remove TorchRL** — MarineGym needs it.
- **We don’t patch TorchRL** — Editing TorchRL so it doesn’t try to load the C++ extension would require maintaining a fork and losing upstream updates; not practical.
- **We don’t rebuild TorchRL** — Rebuilding from source (or hunting a compatible wheel) is possible but out of scope for this repo. The warning is harmless for our use (we don’t use prioritized replay).

So the warning stays. Treat it as **expected and benign** for MarineGym under Isaac Sim; focus on any other, new or different messages in the log.

---

## Summary

| Question | Answer |
|----------|--------|
| What is the C++ extension? | Optional TorchRL plugin for prioritized replay buffers only. |
| Does MarineGym need TorchRL? | Yes (specs, envs, PPO, collectors). |
| Does anything here use the C++ extension? | No. |
| Who triggers the warning? | MarineGym imports TorchRL → TorchRL’s `samplers.py` tries to load `_torchrl` at import time → load fails → warning. |
| Why does the load fail? | ABI mismatch: plugin built with old C++ ABI, Isaac Sim’s PyTorch with new ABI; symbol names don’t match. |
| Why not fix it? | We leave it: no suppression (to avoid missing other warnings), no TorchRL removal or fork; rebuild/compatible wheel out of scope. |
| Impact | None for MarineGym; only prioritized replay would be affected, and we don’t use it. |

---

## See also

- **Test_AUV/docs/30-Unfixable-System-Warnings.md** §4 — Same warning, with symbol names and technical detail.
- **Test_AUV/docs/31-Findings-and-Learnings-Log.md** — Log entry for TorchRL C++ binaries (root cause, no code change).
