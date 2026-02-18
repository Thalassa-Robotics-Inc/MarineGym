# Hydrodynamics

UnderwaterVehicle implements **buoyancy**, **damping** (linear + quadratic), **added mass**, and **coriolis** in **apply_hydrodynamic_forces(flow_vels_w)**. The result is added to thrust/torque and applied to the base_link each step. This doc describes the formulas and where parameters come from (BlueROV.yaml). **Scene gravity** is not used in the buoyancy formula; see **Test_AUV/docs/26-Gravity-and-Hydro-Testing.md** for zero-gravity testing.

---

## apply_hydrodynamic_forces(flow_vels_w)

1. **Body-frame velocity** — vel_b = body velocity from get_state; flow_vels_w in world frame is rotated to body frame (quat_rotate_inverse); relative velocity = body_vels - flow_vels_b. Sign flips on [1,2,4,5] for convention.
2. **Body RPY** — From quaternion_to_euler(self.rot).
3. **Body acceleration** — calculate_acc(body_vels): filtered (alpha=0.3) finite-difference of body_vels over dt; prev_body_vels and prev_body_acc updated.
4. **Damping** — calculate_damping(body_vels): linear_damping_matrix + quadratic_damping_matrix * |body_vels|, applied to body_vels; 6D force/torque in body frame.
5. **Added mass** — calculate_added_mass(body_acc): added_mass_matrix @ body_acc.
6. **Coriolis** — calculate_corilis(body_vels): from added mass and body angular/linear velocity cross products.
7. **Buoyancy** — calculate_buoyancy(body_rpy): magnitude 997*9.81*volume; components from rpy (sin/cos); torque from coBM. **Gravity constant 9.81 is hardcoded**; scene gravity is not read.
8. **Combine** — hydro = -(added_mass + coriolis + damping); sign flips on [1,2,4,5]; buoyancy sign flips same; return (hydro + buoyancy)[0:3], (hydro + buoyancy)[3:6] as forces_w, torques_w.

---

## calculate_buoyancy(rpy)

- **File** — marinegym/robots/drone/underwaterVehicle.py (around lines 265–276).
- **Magnitude** — buoyancyForce = **997 * 9.81 * self.volumes** (water density × g × volume). Scene gravity is **not** used.
- **Force components** (body frame) — From rpy (roll, pitch, yaw): Fx = buoyancyForce*sin(pitch); Fy = -buoyancyForce*sin(roll)*cos(pitch); Fz = -buoyancyForce*cos(roll)*cos(pitch).
- **Torque components** — From coBM (dis): tau_roll = -dis*buoyancyForce*cos(pitch)*sin(roll); tau_pitch = -dis*buoyancyForce*sin(pitch); yaw torque 0 in this model.
- **Convention** — In body frame the Z component is negative (down in body); in world frame after rotation the net buoyancy is upward (+Z) when level.

---

## calculate_damping(body_vels)

- **Matrices** — linear_damping_matrix and quadratic_damping_matrix from BlueROV.yaml (hydro_coef.linear_damping, quadratic_damping), diagonal 6×6, expanded to batch.
- **Formula** — damping_matrix = linear + quadratic * |maintained_body_vels| (element-wise abs on diagonal/cross terms); damping = damping_matrix @ body_vels.

---

## calculate_added_mass(body_acc)

- **Matrix** — added_mass_matrix from hydro_coef.added_mass (6 diagonal entries), expanded to batch.
- **Formula** — added_mass = added_mass_matrix @ body_acc.

---

## calculate_corilis(body_vels)

- **Formula** — ab = added_mass_matrix @ body_vels; coriolis linear = -cross(ab_linear, body_angular); coriolis angular = -(cross(ab_linear, body_linear) + cross(ab_angular, body_angular)).

---

## flow_vels

- **self.flow_vels** — World-frame flow velocity (6D: linear + angular). Set to zero by default; can be set per env via **set_flow_velocities(env_ids, max_flow_velocity, flow_velocity_gaussian_noise)** (e.g. in Hover with disturbances).
- **apply_hydrodynamic_forces** uses flow_vels (with optional noise) to compute relative velocity; so current/flow is supported.

---

## Where gravity appears

- **Buoyancy magnitude** — Uses **9.81** hardcoded in calculate_buoyancy; **scene gravity is not read**.
- **RobotBase** — Reads **self.gravity** from SimulationContext.get_physics_context().get_gravity() for other uses (e.g. weight in reward); UnderwaterVehicle also has **self.gravity = self.masses * 9.81** for force normalization (e.g. force_sensor). So for **hydro only**, setting scene gravity to zero does not change buoyancy magnitude; it only removes PhysX gravity. See **Test_AUV/docs/26-Gravity-and-Hydro-Testing.md**.

---

## See also

- **03-UnderwaterVehicle.md** — apply_action, apply_hydrodynamic_forces call order.
- **09-Assets.md** — volume, coBM, hydro_coef in BlueROV.yaml.
- **Test_AUV/docs/23-Hydrodynamics-Validation.md** — Validation procedure; **26-Gravity-and-Hydro-Testing.md** — Zero gravity.
