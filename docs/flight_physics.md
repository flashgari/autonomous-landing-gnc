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

The current baseline does not yet include:

- full 6-DOF attitude dynamics
- landing leg contact dynamics
- engine startup/shutdown transients
- throttle slew-rate limits
- sensor noise or estimator lag
- atmospheric density variation with altitude
- plume-ground interaction
- convex guidance or trajectory optimization

These are not hidden. They define the roadmap for turning the baseline into a stronger GNC portfolio project.

