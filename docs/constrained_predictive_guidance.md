# Constrained Predictive Guidance

## Engineering Question

The corridor controller improved the landing campaign by scheduling lateral
correction earlier, but it did not explicitly answer whether a commanded
trajectory respected thrust magnitude, body tilt, glide-slope geometry, and
acceleration slew over the remaining time-to-go. This phase replaces the
high-altitude heuristic with a finite-horizon constrained optimizer while
holding the ESKF, nonlinear plant, attitude loop, actuator dynamics, and
touchdown criteria fixed.

The result should be described precisely:

- it is receding-horizon guidance based on direct transcription
- it solves a convex quadratic program in planar acceleration space
- it uses a conservative polygonal approximation to the thrust disk
- it includes a hybrid minimum-throttle supervisor after optimization
- it hands off to the verified terminal corridor controller below `160 m`
- it is not a flight-ready 6-DOF successive-convexification implementation

## Prediction Model and Direct Transcription

The optimizer uses inertial acceleration as its control variable,

$$
\mathbf{u}_k =
\begin{bmatrix}
a_{x,k} & a_{z,k}
\end{bmatrix}^{T},
$$

and propagates a double-integrator model at the prediction nodes:

$$
\mathbf{r}_{k+1}
=
\mathbf{r}_k+\Delta t\,\mathbf{v}_k
+\frac{1}{2}\Delta t^2\mathbf{u}_k,
$$

$$
\mathbf{v}_{k+1}
=
\mathbf{v}_k+\Delta t\,\mathbf{u}_k.
$$

For a horizon of `N = 12` nodes, the future position and velocity histories
can be written in condensed form:

$$
\mathbf{r}=\mathbf{d}_r+\mathbf{M}_r\mathbf{U},
\qquad
\mathbf{v}=\mathbf{d}_v+\mathbf{M}_v\mathbf{U},
$$

where $\mathbf{U}$ stacks all horizontal and vertical accelerations. The
matrices are lower triangular because acceleration at node $j$ can influence
only node $j$ and later nodes. Condensing removes explicit state variables
from the numerical decision vector while retaining the direct-transcription
state equations analytically.

The horizon duration is estimated from altitude and closing speed and bounded
between `2 s` and `30 s`. The first optimized acceleration is applied, the
state estimate evolves, and the problem is solved again every `0.60 s`. This
receding-horizon update is important: aerodynamic drag, actuator lag, mass
depletion, sensor corrections, and model mismatch are not all represented in
the QP, so feedback must repeatedly repair the open-loop prediction.

## Quadratic Objective

Cubic Hermite curves connect the current estimated position and velocity to
the target terminal state. The QP penalizes tracking error along that
reference, terminal position and velocity error, acceleration magnitude,
thrust demand, and inter-node acceleration changes:

$$
\begin{aligned}
J(\mathbf U)=&
\sum_{k=1}^{N}
\left\|\mathbf r_k-\mathbf r_{ref,k}\right\|^2_{Q_r}
+\left\|\mathbf v_k-\mathbf v_{ref,k}\right\|^2_{Q_v}\\
&+\left\|\mathbf r_N-\mathbf r_f\right\|^2_{Q_f}
+\left\|\mathbf v_N-\mathbf v_f\right\|^2_{Q_{vf}}\\
&+\sum_{k=1}^{N}\left(
\left\|\mathbf u_k\right\|^2_R
+\left\|\mathbf u_k-\mathbf u_{k-1}\right\|^2_S
\right).
\end{aligned}
$$

The terminal terms are soft costs rather than exact equalities. This avoids
declaring the entire optimization infeasible when the current state lies
outside the finite-horizon terminal footprint. Physical path constraints
remain hard to the stated numerical feasibility tolerance, and any
unacceptable iterate triggers a deterministic fallback.

The acceleration-difference penalty has two roles. Numerically, it improves
conditioning and suppresses high-frequency changes in the planned sequence.
Physically, it discourages commands that the delayed, first-order, rate-limited
throttle and TVC stack could not track.

## Convex Path Constraints

### Tilt Cone

The total thrust-specific acceleration is

$$
\mathbf a_T =
\begin{bmatrix}
a_x\\
g+a_z
\end{bmatrix}.
$$

For maximum tilt $\theta_{max}$ from vertical,

$$
|a_x|
\le
\tan\theta_{max}(g+a_z).
$$

This produces two linear inequalities. The constraint is more meaningful
than an independent lateral-acceleration limit because it captures the
coupling between horizontal acceleration and vertical thrust projection.

### Maximum Thrust

The circular specific-thrust bound is

$$
\sqrt{a_x^2+(g+a_z)^2}\le\frac{T_{max}}{m}.
$$

The QP replaces that disk with an inscribed 12-sided polygon. Each face gives
a linear half-space, and the factor $\cos(\pi/12)$ makes the polygon
conservative: a point admitted by the polygon remains inside the circular
thrust limit. Current mass is held constant across one prediction horizon.
Because mass decreases during the burn, this makes the maximum-specific-thrust
model conservative over that horizon.

### Glide Slope and Ground Plane

The terrain-relative lateral corridor is

$$
|x_k-x_{target}|
\le
b_g+\gamma z_k,
$$

with $b_g=2\ \mathrm{m}$ and $\gamma=0.10$. It is represented by two linear
inequalities, with a separate constraint $z_k\ge0$. The corridor narrows as
altitude decreases, forcing crossrange error to be removed while there is
still time to counter-accelerate and reduce horizontal velocity.

This is not merely a plotting boundary. In the verified `48 m` divert, the
minimum glide-slope margin approaches zero during `63.6%` of high-altitude
replans, while the minimum tilt and thrust margins remain positive. The
binding physics is therefore terrain-relative path geometry and available
time-to-go, not saturation of body angle or engine thrust.

### Acceleration Slew

The constraints

$$
|a_{x,k}-a_{x,k-1}|\le\Delta a_{x,max},
\qquad
|a_{z,k}-a_{z,k-1}|\le\Delta a_{z,max}
$$

bound changes between prediction nodes. These are guidance-level surrogates
for command-path bandwidth. They do not replace the nonlinear actuator model;
the commanded throttle and gimbal still pass through explicit delay, lag,
deadband, slew, and saturation before reaching the plant.

## ADMM Solution and Acceptance Logic

After condensing, the problem has the standard form

$$
\min_{\mathbf U}\;
\frac12\mathbf U^T\mathbf H\mathbf U+\mathbf f^T\mathbf U,
\qquad
\mathbf l\le\mathbf A\mathbf U\le\mathbf u.
$$

The in-repository solver introduces a projected variable
$\mathbf y=\mathbf A\mathbf U$ and applies scaled ADMM:

$$
\mathbf U^{j+1}
=
\left(\mathbf H+\rho\mathbf A^T\mathbf A\right)^{-1}
\left[-\mathbf f+\rho\mathbf A^T(\mathbf y^j-\boldsymbol\lambda^j)\right],
$$

$$
\mathbf y^{j+1}
=
\Pi_{[\mathbf l,\mathbf u]}
\left(\mathbf A\mathbf U^{j+1}+\boldsymbol\lambda^j\right),
$$

$$
\boldsymbol\lambda^{j+1}
=
\boldsymbol\lambda^j+\mathbf A\mathbf U^{j+1}-\mathbf y^{j+1}.
$$

The infinity-norm primal residual measures disagreement with the projected
constraint set. The dual residual measures change in the projected variable
mapped back into decision space. Strict convergence requires both residuals
below `0.025`. A plan may still be accepted if its primal residual and
independently recomputed maximum inequality violation are below `0.10`.

Separating optimality convergence from feasibility acceptance is deliberate.
A finite-iteration real-time optimizer may produce a physically admissible
command before the dual variables settle to the tighter optimum. The software
reports both quantities. If the iterate is nonfinite or violates the
acceptance threshold, the controller discards it and uses corridor guidance
for that replan.

Across the 200-case predictive campaign, mean solution acceptance is
`99.90%`, strict convergence is `74.22%`, and four replans invoke the fallback.
The maximum recorded violation is `0.459`, which belongs to an iterate that
was rejected rather than applied. Retaining that value is important:
successful landings do not justify hiding numerical failures.

## Minimum Throttle and Terminal Handoff

The convex QP permits continuous thrust between zero and maximum. The
simulated engine instead has a nonzero minimum throttle, which makes the exact
control set disjoint:

$$
T\in\{0\}\cup[T_{min},T_{max}].
$$

Representing that set exactly would require mixed-integer optimization or a
different lossless-convexification argument. This implementation does not
claim either. A hybrid supervisor maps sub-minimum-throttle requests to coast
or minimum throttle according to the vertical braking corridor.

Below `160 m`, the controller hands off to the previously verified corridor
law. At low altitude, ignition logic, throttle lag, contact timing, and the
minimum-throttle discontinuity dominate long-horizon path shaping. The hybrid
architecture therefore assigns the optimizer the high-energy divert phase and
the corridor controller the terminal braking phase. It is an engineering
design choice validated in the nonlinear closed loop, not evidence that the
convex prediction model is valid through touchdown.

## Closed-Loop Results

Both guidance modes use the same `200` vehicle, atmosphere, wind, initial
state, and sensor-noise dispersions with seed `4242`. Both use ESKF feedback
and flight-like actuators.

| Metric | Corridor | Predictive | Change |
| --- | ---: | ---: | ---: |
| success rate | `93.0%` | `97.5%` | `+4.5 points` |
| successes | `186 / 200` | `195 / 200` | `+9 cases` |
| p95 absolute pad error | `3.18 m` | `2.63 m` | `-0.54 m` |
| p95 touchdown speed | `0.92 m/s` | `0.96 m/s` | `+0.04 m/s` |
| worst-case propellant remaining | `2415 kg` | `2449 kg` | `+35 kg` |
| maximum body tilt | `5.79 deg` | `4.41 deg` | `-1.38 deg` |
| maximum applied gimbal | `4.71 deg` | `2.07 deg` | `-2.63 deg` |

The improvement is not uniform across every metric. Predictive guidance
slightly increases p95 touchdown speed by `0.04 m/s`, although it remains far
inside the `2.5 m/s` requirement. Its primary benefit is reducing pad misses
from `14` to `5` by removing lateral velocity earlier. Lower maximum tilt and
gimbal show that the result is generated by temporal allocation of impulse,
not by demanding more peak control authority.

The nominal predictive trajectory lands in `44.05 s`, approximately `2.05 s`
before corridor guidance, and retains `153 kg` more propellant. This is
consistent with lower gravity loss: shortening powered flight reduces the
impulse expended supporting vehicle weight. It is a result of this model and
tuning, not a general proof that predictive guidance always minimizes fuel.

An `8%` delivered-thrust loss remains successful with `0.29 m` target error
and `1.09 m/s` touchdown speed. The result shows closed-loop recovery within
the tested decrement; it does not establish a formal thrust-loss robustness
margin.

## Reachability Interpretation

A deterministic sweep at the finer `0.05 s` integration step tests initial
lateral offsets in `10 m` increments. Corridor guidance passes through
`30 m`; predictive guidance passes through `40 m`. Both fail the tested
`60 m` and `70 m` cases. A separate `48 m` case passes with `+2.71 m` pad
error, `0.52 m/s` horizontal speed, and `0.94 m/s` total touchdown speed.
The `50 m` case fails with `+3.78 m` error, so no monotonic boundary is claimed
between the isolated `48 m` pass and the coarse grid.

The retained failures are physically useful. Lateral displacement requires
the time integral of $T\sin\theta/m$ followed by opposite impulse to remove
the acquired horizontal velocity. Tilt, thrust, glide slope, terminal
velocity, and finite time-to-go jointly bound that impulse. Positive
propellant at touchdown cannot recover a trajectory after the remaining
altitude and actuator response time have become insufficient.

The sweep is a sampled closed-loop map, not a formal reachable set. It does
not guarantee success between samples, under different sensor realizations,
or outside the modeled plant and actuator assumptions.

## Model Boundaries

The optimizer omits several effects that the nonlinear simulation still
applies:

- mass variation inside each prediction horizon
- wind-relative aerodynamic force in the prediction model
- attitude and gimbal transients inside the acceleration-space QP
- exact nonconvex minimum-throttle logic
- landing-leg contact and terrain uncertainty
- three-dimensional translation, quaternion attitude, and engine allocation

A higher-fidelity continuation would use 6-DOF successive convexification or
nonlinear MPC, include trust regions and virtual controls, propagate mass and
attitude states, and verify solver timing and infeasibility recovery on
hardware. The current phase establishes the formulation and closed-loop
evidence needed to justify that next step without overstating fidelity.
