# Attitude-Aware Nonlinear MPC for 6-DOF Powered Descent

## Scope and Fidelity Separation

This milestone closes a receding-horizon nonlinear guidance law around the
existing 14-component rigid-body truth plant. Two models remain deliberately
separate:

- the **truth plant** propagates inertial position and velocity, a
  body-to-inertial quaternion, body angular rate, variable mass and inertia,
  aerodynamic force and moment, four individual engines, and finite-rate
  actuator dynamics;
- the **onboard prediction model** propagates the states that dominate
  powered-descent guidance: position, velocity, thrust-axis direction,
  closed-loop tilt rate, and mass.

The prediction model is therefore not described as a second 6-DOF truth
plant. It is a reduced-order nonlinear MPC model whose output is tested by
rolling the resulting command through the higher-fidelity quaternion plant.
That distinction matters because an optimizer can satisfy its internal model
while failing the controlled nonlinear system.

## Prediction State and Control

At prediction node \(k\), the reduced state and control are

$$
\mathbf{x}_k =
\begin{bmatrix}
\mathbf{r}_{I,k} &
\mathbf{v}_{I,k} &
\mathbf{b}_{3,I,k} &
\boldsymbol{\omega}_{\perp,I,k} &
m_k
\end{bmatrix}^{T},
\qquad
\mathbf{u}_k = \frac{\mathbf{T}_{I,k}}{m_k}.
$$

\(\mathbf{b}_{3,I}\) is the inertial direction of body \(+z\), the nominal
thrust axis. It has unit norm and therefore two independent coordinates.
\(\boldsymbol{\omega}_{\perp,I}\) is the reduced closed-loop angular rate
that rotates that axis. The control is specific thrust rather than inertial
acceleration. Gravity and predicted drag are added inside the dynamics:

$$
\mathbf{a}_{I,k} =
\|\mathbf{u}_k\|\mathbf{b}_{3,I,k}
+ \mathbf{g}_I
+ \frac{\mathbf{D}_{I,k}}{m_k}.
$$

Using specific thrust makes the physical constraints transparent:

$$
0 < m_k\|\mathbf{u}_k\| \le T_{\max,\mathrm{active}},
$$

$$
\|\mathbf{u}_{xy,k}\|
\le u_{z,k}\tan\theta_{\max}, \qquad u_{z,k}>0.
$$

The physical maximum specific thrust changes with mass and with the number of
active engines. The control projection uses the current replan mass as a
common bound across the horizon, which is conservative because mass decreases
during a nominal burn. The nonlinear rollout still recomputes requested force,
mass flow, and hard total-thrust saturation at every node. The tilt cone
encodes thrust projection: lateral guidance consumes a portion of the same
vector that must remove vertical kinetic energy.

## Translational and Propellant Prediction

The node update uses constant acceleration over each shooting interval:

$$
\mathbf{r}_{k+1}
=\mathbf{r}_k+\Delta t\,\mathbf{v}_k
+\frac{1}{2}\Delta t^2\mathbf{a}_k,
$$

$$
\mathbf{v}_{k+1}
=\mathbf{v}_k+\Delta t\,\mathbf{a}_k.
$$

The drag predictor uses known inertial wind and the same low-order quadratic
coefficient structure as the truth model:

$$
\mathbf{v}_{rel,k}=\mathbf{v}_k-\mathbf{w}_I,
$$

$$
\mathbf{D}_{I,k}=
-\frac{1}{2}\rho S C_D
\|\mathbf{v}_{rel,k}\|\mathbf{v}_{rel,k}.
$$

Propellant depletion is nonlinear because commanded specific thrust is
multiplied by current mass:

$$
m_{k+1}
=m_k-\Delta t\,
\frac{\min(m_k\|\mathbf{u}_k\|,T_{\max,\mathrm{active}})}
{I_{sp}g_0}.
$$

This coupling prevents the optimizer from treating thrust acceleration and
propellant cost as independent. As mass decreases, a fixed thrust produces
more acceleration, while a fixed specific-thrust command requires less
force and mass flow.

## Closed-Loop Attitude Prediction

Guidance cannot assume that the body axis instantaneously aligns with a new
thrust command. The desired axis is

$$
\mathbf{b}_{3,c,k}=
\frac{\mathbf{u}_k}{\|\mathbf{u}_k\|},
$$

and the reduced attitude response is

$$
\dot{\boldsymbol{\omega}}_{\perp}
=\omega_n^2
(\mathbf{b}_3\times\mathbf{b}_{3,c})
-2\zeta\omega_n\boldsymbol{\omega}_{\perp},
$$

$$
\dot{\mathbf{b}}_3
=\boldsymbol{\omega}_{\perp}\times\mathbf{b}_3.
$$

This is a second-order approximation of the verified quaternion attitude
loop, not an alternative attitude controller. The prediction integrator
substeps this model at no more than \(0.12\) s even when the guidance node is
several seconds long. Without substepping, explicit integration can make the
prediction numerically unstable when
\(\omega_n\Delta t\) is too large. That failure was detected by the nonlinear
rollout gate during development rather than hidden by accepting the plan.

The truth simulation still computes quaternion error, gyroscopic
compensation, six-axis wrench allocation, command delay, actuator lag, and
body rotational dynamics. Prediction-axis error is penalized because a
requested acceleration that the attitude loop cannot realize within the
horizon has little physical value.

## Boundary-Condition Reference

Each replan constructs a cubic inertial trajectory satisfying

$$
\mathbf{r}(0)=\mathbf{r}_0,\quad
\dot{\mathbf{r}}(0)=\mathbf{v}_0,
$$

$$
\mathbf{r}(t_f)=\mathbf{r}_{target},\quad
\dot{\mathbf{r}}(t_f)=
\begin{bmatrix}0&0&-v_{z,f}\end{bmatrix}^{T}.
$$

Writing

$$
\mathbf{r}(t)=\mathbf{r}_0+\mathbf{v}_0t
+\mathbf{c}_2t^2+\mathbf{c}_3t^3
$$

gives

$$
\mathbf{c}_2=
\frac{3(\mathbf{r}_f-\mathbf{r}_0)
-(2\mathbf{v}_0+\mathbf{v}_f)t_f}{t_f^2},
$$

$$
\mathbf{c}_3=
\frac{-2(\mathbf{r}_f-\mathbf{r}_0)
+(\mathbf{v}_0+\mathbf{v}_f)t_f}{t_f^3}.
$$

The reference is not itself declared feasible. It supplies a smooth
position, velocity, and acceleration schedule; the nonlinear optimizer then
projects thrust into the available cone and evaluates the resulting state
history.

## Objective and Terrain-Relative Penalty

The finite-horizon least-squares objective contains

$$
J =
\sum_{k=1}^{N}
\left(
\|W_r(\mathbf{r}_k-\mathbf{r}_{ref,k})\|_2^2
+\|W_v(\mathbf{v}_k-\mathbf{v}_{ref,k})\|_2^2
\right)
$$

$$
+\sum_{k=1}^{N}
\left(
w_R\|\mathbf{b}_{3,k}\times\mathbf{b}_{3,c,k}\|_2^2
+w_\omega\|\boldsymbol{\omega}_{\perp,k}\|_2^2
+w_T\|\mathbf{u}_k\|_2^2
\right)
$$

$$
+\sum_{k=2}^{N}
w_{\Delta T}\|\mathbf{u}_k-\mathbf{u}_{k-1}\|_2^2
+J_f+J_{corridor}+J_{reserve}.
$$

The terminal term weights position and velocity more strongly than
intermediate tracking. The terrain-relative corridor penalty is

$$
J_{corridor} =
w_c\sum_k
\max\left(
\|\mathbf{r}_{xy,k}-\mathbf{r}_{target,xy}\|
-0.65-0.020\max(z_k,0),
0
\right)^2.
$$

The corridor contracts with altitude. It encourages early crossrange
correction, when time-to-go is large, instead of carrying lateral impulse
demand into terminal braking. A soft reserve penalty discourages predictions
that approach dry mass, but positive propellant is also required by the
rollout acceptance gate.

## Trust-Region Nonlinear Solve

The implementation uses direct shooting. Every candidate control sequence is
propagated through the nonlinear reduced dynamics, so no linearized dynamics
defect is introduced. Consequently, the virtual-control device commonly used
to restore feasibility in successive convexification is not required here.

At iteration \(j\), finite differences form the residual Jacobian

$$
J_r =
\frac{\partial\mathbf{e}}{\partial\mathbf{U}}
\bigg|_{\mathbf{U}^{(j)}},
$$

and a regularized Gauss-Newton step solves

$$
(J_r^T J_r+\lambda I)\Delta\mathbf{U}
=-J_r^T\mathbf{e}.
$$

The step is clipped by an \(L_\infty\) trust region, projected back into the
thrust and tilt set, and tested at line-search scales
\(1,\;1/2,\;1/4\). A candidate is accepted only if a fresh nonlinear rollout
reduces the merit function. The trust region expands after an accepted step
and contracts after rejection.

This is local nonlinear optimization. It does not prove global optimality.
The reported evidence concerns accepted, constraint-projected trajectories
from the tested initial-condition envelope.

## Supervisory Logic

The optimizer replans every \(0.8\) s using six shooting nodes. Below \(25\)
m, guidance hands off to the verified energy-gated terminal controller. The
handoff isolates minimum-throttle switching and near-ground actuator
transients from the coarser high-altitude prediction grid.

If an engine is disabled, the nominal four-engine plan is invalidated
immediately. The system transitions to the bounded recovery law rather than
continuing to apply a command based on unavailable thrust. This is fault
detection and plan invalidation, not proof of safe three-engine landing.
Removing one engine makes the requested six-axis wrench generally
unattainable with the present allocator. The deterministic failure is
retained to show that guidance feasibility and control-allocation feasibility
are different questions.

## Verification Results

The 24-case campaign uses seed `9242`. Each baseline/NMPC pair shares the same
initial position, velocity, attitude, rate, mass, and wind draw.

| Metric | Scheduled feedback | NMPC | Change |
| --- | ---: | ---: | ---: |
| success rate | `70.8%` | `79.2%` | `+8.3 points` |
| p95 horizontal error | `3.95 m` | `3.48 m` | `-0.47 m` |
| median horizontal error | `1.57 m` | `1.26 m` | `-0.31 m` |
| median propellant remaining | `3128 kg` | `3779 kg` | `+651 kg` |
| p95 vertical touchdown speed | `1.20 m/s` | `1.20 m/s` | approximately unchanged |

All seven baseline failures and all five NMPC failures are footprint misses.
The optimizer therefore improves lateral robustness without moving the
dominant failure to vertical speed, attitude, or propellant depletion.

The deterministic high-crosswind case remains outside the requirement:

| Guidance | Error | Vertical speed | Maximum tilt | Propellant |
| --- | ---: | ---: | ---: | ---: |
| scheduled feedback | `6.16 m` | `1.20 m/s` | `1.77 deg` | `3150 kg` |
| NMPC | `4.88 m` | `1.20 m/s` | `1.53 deg` | `3849 kg` |

The miss reduction comes from known-wind drag prediction and earlier
counter-impulse scheduling. The case still fails because prediction cannot
manufacture additional thrust-vector angle, control-allocation authority, or
time-to-go.

The calm deterministic trajectory shortens from `41.65 s` to `32.40 s` while
preserving approximately `1.20 m/s` vertical touchdown speed. That explains
the propellant result. The vehicle spends less time producing force against
gravity, so gravity loss falls. The reserve increase is not obtained by
accepting a harder vertical landing.

## Timing Evidence

Solve time is measured around every replan on the local bundled Python
runtime:

| Statistic | Solve time |
| --- | ---: |
| mean | `146 ms` |
| p95 | `228 ms` |
| maximum | `275 ms` |
| replan period | `800 ms` |

The maximum observed solve remains below the update period, but this is not
an onboard worst-case execution-time proof. Processor architecture, operating
system scheduling, compiled linear algebra, memory behavior, and timing
jitter are not represented.

## What the Figure Means

Panel A retains the `18/-10 m/s` high-wind failure. Both trajectories reach
the ground softly, but NMPC reduces the miss from `6.16 m` to `4.88 m`. The
failure is therefore a lateral disturbance/reachability boundary, not a
vertical-energy or attitude-limit failure.

Panel B shows that NMPC reaches touchdown sooner while preserving terminal
descent speed. The area between the flight-time histories corresponds
physically to reduced time accumulating gravity loss. This is why modeled
propellant reserve rises.

Panel C is a matched dispersion comparison. The improvement is not inferred
from one nominal trajectory. The unchanged sample pairs isolate guidance
architecture as the independent variable.

Panel D reports propellant remaining, not merely burn time. Mass flow is
integrated from individual-engine thrust, so the result includes the effect
of thrust magnitude as well as duration.

Panel E compares measured solve time with the actual command-update period.
It establishes timing margin on the development machine while clearly
stopping short of flight-computer qualification.

## Limitations and Next Fidelity Step

The current NMPC uses truth position, velocity, attitude, mass, and known
wind. It does not yet include the planar ESKF in the 3D loop. The next
navigation step is a 15-state inertial error model with accelerometer and gyro
biases, asynchronous 3D aiding, and wind estimation.

The optimizer uses a reduced thrust-axis attitude model, finite-difference
Jacobians, soft corridor and reserve penalties, and local Gauss-Newton
iterations. A higher-fidelity continuation would use full quaternion or
rotation-vector error states, sparse analytic derivatives, explicit state
and control inequalities, and either successive convexification with virtual
controls or nonlinear programming with a certified real-time iteration
scheme.

Engine-out recovery requires an allocation-aware contingency problem over the
reduced engine set. A feasible recovery must coordinate translational thrust,
attitude moment, and terminal constraints inside the asymmetric attainable
wrench polytope. Detecting the fault and reducing total-thrust bounds is not
equivalent to solving that problem.
