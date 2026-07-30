# 3D 6-DOF Landing Dynamics, Control, and Verification

## Scope

This milestone adds a three-dimensional rigid-body plant and a four-engine
thrust-vector-control allocation layer to the existing planar landing stack.
It is intentionally additive: the earlier planar ESKF, fault campaigns, and
constrained-guidance studies remain reproducible, while the new model isolates
the rotational dynamics and actuator-allocation questions that a planar model
cannot answer.

The state vector has 14 stored components:

```text
x = [r_I(3), v_I(3), q_BI(4), omega_B(3), m(1)]
```

The unit-norm quaternion has one algebraic constraint, so the attitude still
has three physical degrees of freedom. Position and velocity are expressed in
an inertial landing frame with `z` positive upward. The scalar-first
quaternion `q_BI` rotates body-frame vectors into that inertial frame. Body
`+z` is the nominal thrust axis from the engine plane toward the vehicle nose.
Angular velocity is expressed in body coordinates.

## Translational Dynamics

The inertial equations are

```text
r_dot_I = v_I
m v_dot_I = R_BI(q) (F_T,B + F_A,B) + m g_I
g_I = [0, 0, -g]
```

Thrust and aerodynamic loads are accumulated in the body frame because the
engine geometry and aerodynamic incidence are most naturally defined there.
The quaternion-derived direction-cosine matrix rotates those loads into the
inertial frame before gravity is added.

This transformation is the principal translation-attitude coupling. A lateral
acceleration command cannot be applied directly to `x` or `y`; the controller
must redirect the body thrust axis. If the total thrust magnitude is `T` and
the body axis is tilted by `theta`, the useful vertical and horizontal
components are

```text
T_z = T cos(theta)
T_xy = T sin(theta)
```

The same bounded thrust vector therefore performs both crossrange correction
and vertical kinetic-energy removal. This is why terminal lateral correction
is expensive even when the required horizontal displacement looks small:
late tilt reduces the vertical force margin precisely when time-to-go and
altitude margin are smallest.

## Aerodynamic Loads and Crosswind Coupling

Air-relative velocity is

```text
v_rel,I = v_I - w_I
v_rel,B = R_BI(q)^T v_rel,I
qbar = 0.5 rho |v_rel,I|^2
```

The implemented low-order force model is

```text
F_D,B = -qbar S C_D v_rel,B / |v_rel|
F_N,B = -qbar S C_Nalpha v_perp,B / |v_rel|
v_perp,B = [v_rel,B,x, v_rel,B,y, 0]
M_A,B = r_CP x F_N,B
```

For small incidence, `|v_perp|/|v_rel|` is the angle-of-attack magnitude, so
the normal-force relation is a vector form of the linear
`C_N approximately C_Nalpha alpha` model. The center-of-pressure offset turns
crossflow force into an attitude disturbance. The load is quadratic in
air-relative speed through dynamic pressure and rotates with the vehicle, so a
constant inertial wind does not produce a constant body-axis force during a
tilting maneuver.

This is a deliberately reduced aerodynamic model. It omits Mach and Reynolds
number dependence, nonlinear coefficient tables, plume-flow interaction, and
unsteady aerodynamics. Its purpose is to exercise coupled disturbance
rejection, not to claim vehicle-specific aerodynamic fidelity.

## Variable-Mass Rotational Dynamics

Angular momentum in the body frame is `H_B = I_B omega_B`. Euler's equation
for the scheduled inertia model is

```text
I_B omega_dot_B
  + I_dot_B omega_B
  + omega_B x (I_B omega_B)
  = tau_TVC,B + tau_A,B - D omega_B
```

The implementation retains all three terms on the left:

- `I_B omega_dot_B` is the direct angular-acceleration term.
- `I_dot_B omega_B` accounts for changing angular momentum as propellant is
  depleted and the modeled inertia decreases.
- `omega_B x (I_B omega_B)` is the gyroscopic coupling term. It vanishes only
  for special rate/inertia alignments and cannot generally be removed from a
  nonlinear 6-DOF plant.

The diagonal inertia is linearly scheduled between wet and dry values using
propellant fraction. The longitudinal engine-to-CM arm is also scheduled from
`14.0 m` wet to `15.5 m` dry. The same gimbal force can therefore produce a
different torque late in flight:

```text
tau_TVC,B = sum_i r_i,B(m) x F_i,B
```

Mass depletion follows

```text
m_dot = -sum_i T_i / (Isp g0)
```

until dry mass is reached. The model does not include propellant slosh,
off-diagonal products of inertia, tank-by-tank depletion, or moving flexible
modes. Those omissions bound the interpretation of the scheduled inertia.

## Quaternion Kinematics

Quaternion propagation uses body angular velocity:

```text
q_dot_BI = 0.5 q_BI tensor_product [0, omega_B]
```

The RK4 integrator renormalizes the quaternion after intermediate and complete
steps. The nominal and stress cases retain maximum norm error of approximately
`2.2e-16`, so the direction-cosine matrix remains orthonormal to floating-point
precision. Euler angles are exported only for interpretation; they are not
integrated states and therefore do not introduce a gimbal-lock singularity
into the dynamics.

## Vertical Guidance and Energy Management

The high-altitude vertical reference starts from the constant-deceleration
energy relation

```text
v_z,ref = -sqrt(2 a_b z)
```

which follows by equating the removable kinetic energy per unit mass,
`v_z^2/2`, to the available braking work per unit mass, `a_b z`. Three
altitude gates then impose approach, flare, and terminal descent limits:

```text
z < 90 m: |v_z,ref| <= 4.5 m/s
z < 25 m: |v_z,ref| <= 2.2 m/s
z <  6 m: |v_z,ref| <= 1.2 m/s
```

These gates prevent the earlier overly conservative behavior in which the
vehicle entered a long low-altitude hover. A hover consumes propellant at
approximately `mg/(Isp g0)` without reducing altitude, so it accumulates
gravity loss. The revised schedule completes nominal touchdown in `41.7 s`
with roughly `3144 kg` of modeled propellant remaining.

Horizontal acceleration combines position, velocity, and radial corridor
feedback:

```text
a_xy,cmd = -K_p e_xy - K_v v_xy - K_c e_corridor
```

and is magnitude-limited. The corridor half-width contracts with altitude.
This moves lateral impulse earlier in the descent and protects late vertical
authority. The present law is proportional-derivative and has neither
integral action nor an explicit wind-force estimate. A sufficiently large
steady crosswind can therefore produce terminal position bias even when tilt
and thrust limits are not active.

## Quaternion Attitude Control

The desired thrust force is

```text
F_des,I = m (a_cmd,I - g_I)
```

and the lateral component is projected into the tilt cone

```text
||F_des,xy|| <= F_des,z tan(theta_max).
```

The desired body `+z` axis is aligned with `F_des,I`. A fixed yaw reference
completes the desired rotation matrix, which is converted to a quaternion.
The shortest-rotation error is

```text
q_e = conjugate(q_BI) tensor_product q_des
e_R = 2 vector(q_e)
```

with quaternion sign chosen so the scalar component is nonnegative. The body
torque request is

```text
tau_cmd = K_R e_R - K_omega omega_B
          + omega_B x (I_B omega_B).
```

The final term compensates the known gyroscopic coupling. The plant retains
the scheduled-inertia term `I_dot omega`; it is not canceled in the
controller, so its effect remains part of the closed-loop robustness test.

## Four-Engine Wrench Allocation

Each engine contributes a three-component body force. Stacking the four force
vectors gives

```text
u = [F_1,B, F_2,B, F_3,B, F_4,B]
W = [F_B, tau_B] = A(m) u
A_i = [I_3; cross_product_matrix(r_i,B(m))]
```

The allocator solves a weighted, regularized least-squares problem around an
equal axial-thrust reference:

```text
minimize ||W_A (A u - W_cmd)||_2^2
         + lambda ||u - u_equal||_2^2.
```

Force rows and moment rows are nondimensionalized with characteristic engine
arms before the solve. The regularization discourages unnecessary differential
loading when multiple engine-force combinations produce a similar vehicle
wrench.

The resulting engine forces are projected into the admissible set:

- nonnegative axial thrust;
- circular gimbal cone;
- per-engine maximum thrust;
- hybrid minimum-throttle logic;
- disabled-engine constraint for the failure case.

With body `+z` along the thrust axis, the moment mechanisms must be interpreted
carefully. Axial-thrust imbalance acting through the engine-cluster `x/y`
offsets produces body-`x/y` moments. Lateral gimbal forces acting through the
longitudinal engine-to-CM arm also produce body-`x/y` moments. Body-`z` roll
moment comes from differential tangential gimbal forces acting through the
cluster radius. This geometry is why a four-engine cluster can command all
three moments and why removing one engine makes the feasible wrench set
asymmetric.

## Actuator Command Path

Static allocation feasibility is reported separately from realized actuator
tracking. Each engine command passes through:

```text
command delay
first-order throttle and gimbal lag
throttle and gimbal rate limits
hard throttle and gimbal saturation
```

The figure therefore reports two normalized residuals:

- allocator residual: requested wrench minus the statically allocated wrench;
- actuator-path residual: requested wrench minus the wrench realized after
  delay, lag, and rate limiting.

For the passing cases, p95 static allocation residual remains below `0.005`.
The larger actuator-path residual, approximately `0.08-0.09`, records transient
command-path bandwidth rather than an infeasible steady wrench. In the
engine-out case, p95 static residual rises to `0.301`, showing that the
requested six-axis wrench itself lies substantially outside the asymmetric
three-engine attainable set.

## Verification Results

| Case | Wind `m/s` | Result | Horizontal error | Vertical speed | Maximum tilt | Propellant |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| calm nominal | `[0, 0, 0]` | pass | `0.24 m` | `1.20 m/s` | `2.06 deg` | `3144 kg` |
| crosswind | `[12, -6, 0]` | pass | `2.87 m` | `1.20 m/s` | `1.68 deg` | `3149 kg` |
| high crosswind | `[18, -10, 0]` | fail | `6.16 m` | `1.20 m/s` | `1.77 deg` | `3150 kg` |
| engine 1 out at `12 s` | `[8, -3, 0]` | fail | `10.16 m` | `1.18 m/s` | `7.93 deg` | `3083 kg` |

The two failures have different mechanisms.

The high-crosswind case remains well below the tilt criterion and has low
allocation residual. Its `6.16 m` error is therefore not evidence of exhausted
instantaneous TVC authority. It exposes a guidance architecture limitation:
the scheduled PD law reduces horizontal feedback near the ground and has no
integral or wind-feedforward channel, leaving a steady disturbance offset.

The engine-out case is different. Attitude remains bounded below `12 deg`, but
the missing engine removes force and moment combinations from the attainable
wrench set. The p95 allocator residual rises by roughly two orders of magnitude
relative to nominal, horizontal touchdown speed grows, and the vehicle lands
`10.16 m` from target. Remaining propellant is not a sufficient recoverability
metric because fuel inventory does not restore missing wrench directions or
time-to-go.

## Verification Meaning and Limitations

The deterministic cases verify internal consistency of the implemented
equations and identify closed-loop boundaries. They do not constitute a
probabilistic 6-DOF certification campaign. The model still omits:

- terrain-relative optical navigation in three dimensions;
- landing-leg contact, compliance, tip-over, and surface slope;
- engine plume-ground interaction and recirculation;
- slosh, flexible modes, structural bending, and sensor/actuator mounting
  dynamics;
- nonlinear aerodynamic databases and atmosphere variation;
- engine ignition transients, combustion instability, and correlated failures;
- onboard computation timing and quantization.

The next control-development step identified here has now been implemented as
a reduced-order nonlinear MPC layer with mass, thrust-axis attitude response,
quadratic wind drag, trust-region updates, nonlinear rollout acceptance,
matched 3D dispersions, and measured solve timing. Its formulation and results
are documented in
[Attitude-Aware Nonlinear MPC for 6-DOF Powered Descent](six_dof_nmpc_guidance.md).

The remaining integration gap is navigation and contingency allocation. The
3D NMPC currently uses truth state and known wind rather than a 15-state
inertial estimator, and the three-engine attainable wrench set does not
support the nominal six-axis request after failure. Those limitations define
the next estimator-in-the-loop and allocation-aware recovery milestones.
