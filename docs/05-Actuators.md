# Actuators

Thruster and rotor models used by UnderwaterVehicle: **T200** (main thruster model) and **RotorGroup**. See **03-UnderwaterVehicle.md** for how they are used in apply_action.

---

## T200 (marinegym/actuators/t200.py)

T200 is an `nn.Module` that models a single thruster: **command (-1..1) → throttle → rpm → thrust and moment**.

### Parameters (from rotor_config YAML)

- **force_constants**, **moment_constants** — From rotor_config.
- **max_rot_vels** — max_rotation_velocities.
- **num_rotors** — len(force_constants).
- **dt** — Simulation timestep.
- **time_up**, **time_down** — Throttle response (default 0.15); can be overridden by rotor_config time_constants.
- **throttle** — Per-rotor throttle state (nn.Parameter, zeros initially).
- **directions** — Per-rotor direction (e.g. 1 or -1).
- **rpm** — Per-rotor rpm state.
- **time_constants** — From rotor_config (first-order response for rpm).

### forward(cmds)

1. **target_throttle** — clamp(cmds, -1, 1).
2. **tau** — time constant (tau_up if target > throttle, else tau_down); throttle updated: throttle += tau * (target_throttle - throttle).
3. **target_rpm** — Piecewise linear in throttle (positive/negative thresholds ~0.075); e.g. 3659.9*throttle + 345.21 for positive.
4. **rpm** — First-order filter: alpha = exp(-dt/time_constants); rpm = alpha*rpm + (1-alpha)*target_rpm; clamped to [-3900, 3900]; optional noise.
5. **thrusts** — From rpm via quadratic/linear fit: force_constants/4.4e-7 * 9.81 * (piecewise in rpm for positive/negative).
6. **moments** — thrusts * -directions * 0 (in current code moments from thrust direction only; scaling 0 can be changed).

Returns **(thrusts, moments)**.

### Usage in UnderwaterVehicle

- One **T200** instance is created with the full rotor_config and dt; **make_functional** is used so parameters can be expanded to batch shape.
- **apply_action** uses `torch.vmap(torch.vmap(self.rotors, ...))(rotor_cmds, self.rotor_params)` to compute thrusts and moments for all envs and rotors; thrusts are applied at rotor positions via rotors_view, and net torque from moments is applied at base_link with hydro.

---

## RotorGroup (marinegym/actuators/rotor_group.py)

RotorGroup groups rotor-related logic; used in conjunction with T200 and the rotors_view. See source for any multi-rotor helpers.

---

## See also

- **03-UnderwaterVehicle.md** — apply_action flow.
- **09-Assets.md** — rotor_configuration in BlueROV.yaml.
