# Controllers

**ControllerBase** and **LeePositionController**: convert high-level targets (position, velocity, yaw) into rotor commands. Used by tasks (e.g. Hover) when a controller is configured; see **07-Environments.md**.

---

## ControllerBase (marinegym/controllers/controller.py)

Base class for controllers. Subclasses implement the policy from state + target → action (rotor commands).

---

## LeePositionController (marinegym/controllers/lee_position_controller.py)

Implements the controller from [Lee et al.](https://arxiv.org/abs/1003.2005): **root_state (position, quaternion, linear velocity, angular velocity) + control_target (position, linear velocity, yaw)** → **cmd** (num_rotors).

- **compute_parameters(rotor_config, inertia_matrix)** — Builds mixer matrix from rotor angles, arm lengths, force/moment constants, directions, max_rot_vel; used to map desired force/torque to rotor commands.
- **__init__** — Loads robot-specific YAML (e.g. lee_controller_BlueROV.yaml or lee_controller_neo11.yaml); builds mixer from inertia and rotor config.
- **forward(root_state, control_target)** — Returns rotor commands (and optional controller_state).

Config files live in **marinegym/controllers/cfg/** (e.g. lee_controller_BlueROVHeavy.yaml, lee_controller_firefly.yaml, lee_controller_hummingbird.yaml, lee_controller_neo11.yaml). Each pairs a robot with its mixer/inertia and rotor layout.

---

## Usage in envs

- **Hover** and other tasks can set **drone_model.controller** in task config; **UnderwaterVehicle.make(drone_model.name, drone_model.controller)** returns (drone, controller).
- The env step can use **controller(drone.get_state() or root_state, target)** to get actions instead of raw policy actions (e.g. for evaluation or scripted behavior).

---

## See also

- **07-Environments.md** — Hover, _design_scene, drone + controller.
- **09-Assets.md** — Inertia and rotor config (YAML).
