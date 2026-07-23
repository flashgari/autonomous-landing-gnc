# Flight Physics

## Objective

Explain the physical model behind the autonomous landing simulator at an upper-division aerospace level. The point of the project is not only to produce a landing animation; it is to connect vehicle dynamics, guidance commands, actuator authority, mass depletion, aerodynamic disturbances, and touchdown constraints.

## State And Coordinates

The initial baseline is planar:

```text
state = [x, z, vx, vz, theta, omega, m]
```

where `x` is horizontal position relative to the landing pad, `z` is altitude, `theta` is body tilt from vertical, `omega` is pitch rate, and `m` is instantaneous vehicle mass.

The simplified 2D model is intentional. It isolates the core powered-descent coupling before adding full 6-DOF effects:

- vertical energy management
- lateral divert correction
- tilt-induced thrust projection
- TVC attitude torque
- propellant depletion
- aerodynamic drag and wind-relative velocity

## Translational Dynamics

The translational equations are:

```text
m x_ddot = T sin(theta) + D_x
m z_ddot = T cos(theta) + D_z - m g
```

The key coupling is thrust projection. Tilting the vehicle produces horizontal acceleration, but the same tilt reduces the vertical component of thrust by `cos(theta)`. This is why lateral correction late in descent is expensive: large tilt can steal vertical thrust margin exactly when the vehicle needs to arrest descent rate.

Upper-division interpretation:

- The vertical channel is an energy-management problem. The controller must remove gravitational potential and kinetic energy while preserving enough propellant and thrust margin to meet touchdown velocity constraints.
- The lateral channel is a constrained divert problem. Horizontal error cannot be corrected independently of vertical landing because lateral acceleration requires either body tilt or thrust-vector deflection.
- The landing problem is therefore under a coupled acceleration budget: the commanded acceleration vector must fit inside thrust magnitude and attitude constraints.

## Aerodynamic Drag And Wind

The drag model uses wind-relative velocity:

```text
v_rel = [vx - wind_x, vz]
D = -0.5 rho |v_rel|^2 S C_D (v_rel / |v_rel|)
```

This matters because wind changes the velocity seen by the vehicle, not just the inertial trajectory. A tailwind or crosswind changes dynamic pressure and therefore changes the aerodynamic force vector. Even in a simplified descent model, wind-relative drag can shift the landing footprint and change how much lateral correction the controller must command.

## Mass Depletion

The mass model is:

```text
m_dot = -T / (Isp g0)
```

As propellant burns, mass decreases and thrust-to-weight ratio increases. That means a fixed throttle command does not produce a fixed acceleration over the burn. Guidance must account for the fact that the plant becomes more responsive later in descent.

Upper-division interpretation:

- Early in descent, high mass lowers acceleration authority.
- Late in descent, lower mass increases acceleration authority, making aggressive gains more likely to overshoot.
- Propellant remaining is therefore not only a resource metric; it changes closed-loop dynamics.

## Rotational Dynamics And TVC

The attitude model is:

```text
I theta_ddot = T L sin(delta) - c_omega omega
```

where `delta` is engine gimbal angle and `L` is the engine moment arm. In the baseline model, body tilt determines the translational thrust direction, while gimbal deflection creates control torque to drive body attitude toward the guidance-commanded tilt.

Upper-division interpretation:

- TVC authority scales with thrust. At low throttle or engine cutoff, attitude-control authority is reduced.
- Gimbal saturation limits the maximum angular acceleration available for attitude tracking.
- Rotational damping represents unmodeled energy dissipation and prevents the first baseline from behaving like a perfectly lossless rigid body.
- Attitude tracking error becomes translational error because the thrust direction is the acceleration direction.

## Guidance Logic

The vertical guidance law uses a square-root velocity corridor:

```text
vz_ref = -sqrt(2 a_brake z)
```

with a terminal descent-rate floor. This is based on constant-acceleration stopping distance: if a vehicle descends at speed `|vz_ref|`, then acceleration `a_brake` is sufficient to reduce vertical speed before touchdown.

The lateral guidance law is PD:

```text
ax_cmd = -Kx x - Kvx vx
```

The desired body tilt comes from the acceleration vector:

```text
theta_cmd = atan2(ax_cmd, g + az_cmd)
```

Upper-division interpretation:

- The square-root vertical profile is a guidance corridor, not a trajectory optimizer. It gives a physically meaningful descent-rate bound tied to braking acceleration.
- The lateral law is intentionally conservative because high lateral acceleration demands high tilt, reducing vertical thrust projection.
- A later project phase should replace or compare this baseline with a more rigorous constrained guidance method.

## Nominal Result Interpretation

The nominal landing succeeds because the guidance law keeps the vehicle inside a feasible acceleration envelope:

- vertical velocity is reduced before ground contact
- throttle increases during terminal braking
- body tilt stays small, preserving vertical thrust margin
- gimbal remains far from saturation, so attitude tracking has authority margin
- propellant remains positive at touchdown

The result is not a claim that the vehicle is flight-realistic. It is a controlled baseline that exposes the right aerospace questions:

- How much divert can be corrected before tilt reduces vertical thrust margin?
- How sensitive is touchdown error to wind and navigation noise?
- How much propellant margin is required for dispersions?
- When do gimbal and throttle limits become active constraints?
- How does estimator delay affect terminal guidance?

## Known Simplifications

The completed planar project still does not include:

- full 6-DOF attitude dynamics
- landing leg contact dynamics
- detailed engine startup/shutdown combustion transients
- redundant sensor voting or covariance-based estimation
- atmospheric density variation with altitude
- plume-ground interaction
- convex guidance or trajectory optimization

These are not hidden. They bound what can be inferred from the current results and define the next fidelity upgrades.

## Navigation as Output Feedback

The completed navigation layer replaces exact state feedback with:

```text
y_k = h(x_k) + b + nu_k
```

where `b` is a run-constant sensor bias and `nu_k` is sample noise. The estimator propagates position and attitude between measurement epochs, then corrects them with alpha-beta innovations. Guidance therefore acts on `x_hat`, while the plant evolves according to `x`.

This distinction matters dynamically. Let the control law be `u = K(x_hat)`. A navigation error `e = x_hat - x` becomes a command perturbation approximately equal to:

```text
delta_u ~= (dK/dx) e
```

That command perturbation then passes through actuator lag before changing the true state. Estimation error, control sensitivity, and actuator bandwidth jointly determine the trajectory response; estimator RMS error alone is not a landing-performance metric.

The Monte Carlo comparison demonstrates the coupling. With flight-like actuators, truth-state corridor guidance succeeds in `95.0%` of cases, while estimated-state feedback succeeds in `66.5%`. Most added failures are pad misses, indicating terminal lateral-state uncertainty rather than loss of attitude stability.

## Actuator Bandwidth and Phase Lag

Applied throttle and gimbal follow delayed, first-order, rate-limited dynamics:

```text
u_dot = (u_cmd(t - t_delay) - u) / tau
|u_dot| <= rate_limit
```

For a sinusoidal command, first-order lag reduces magnitude and adds phase delay as frequency increases. Terminal guidance commands can therefore be mathematically feasible yet physically unrealizable if they demand acceleration changes faster than the engine or gimbal mechanism can produce.

The maximum instantaneous command-to-applied throttle difference is large at ignition because a finite-rate actuator begins near zero while guidance immediately requests braking thrust. This is a transient tracking metric, not a steady-state error. The vehicle succeeds when the remaining stopping distance absorbs that transient; it fails when time-to-go is too short.

## Fault Accommodation Physics

The altitude-bias fault is handled with an innovation gate. A rejected measurement prevents a large residual from entering the estimate, but the estimator then relies on propagation and other channels. Model error accumulates, so fault tolerance consumes performance margin rather than making the fault disappear.

The successful bias-fault case flies about `6.2 s` longer and retains roughly `454 kg` less propellant than the full-stack nominal case. The additional propellant use is primarily gravity loss: thrust must support weight during the extra time even when net acceleration is small.

The 18% delivered-thrust loss produces a pad miss with propellant remaining. The key state is not fuel mass alone but available impulse and moment over finite time:

```text
available lateral impulse = integral T sin(theta) dt
available pitch impulse   = integral T L sin(delta) dt
```

Reducing `T` decreases both. Stored propellant cannot compensate if the vehicle reaches the ground before it can deliver the required impulse.

## Hazard Geometry and Reachability

Hazard selection imposes a geometric requirement before continuous guidance begins. A candidate target must lie outside the unsafe interval with specified clearance. The guidance law then treats that target as the origin of its lateral error state.

The successful hazard-relative case lands `5.47 m` from the nearest hazard edge. That number is geometric clearance, not target error. The target-relative landing error is `-2.53 m`; both values are needed because a trajectory can satisfy its controller tolerance yet violate a hazard boundary if the target itself was selected too close to unsafe terrain.

## Sampled Feasibility Versus Formal Reachability

The altitude/offset figure is a sampled closed-loop map. Each point integrates the nonlinear plant with controller and actuator limits and then applies terminal constraints. It is not a formal reachable set because the state grid is incomplete and no guarantee is made between sampled points.

The boundary depends on initial kinetic energy, not merely position:

```text
E_vertical = 0.5 m vz^2 + m g z
```

Two cases with greater altitude can still have different margin if their initial descent speed, time spent near minimum throttle, and guidance-phase entry differ. This is why the observed boundary should be interpreted in the full state space rather than as a simple maximum-offset curve.
