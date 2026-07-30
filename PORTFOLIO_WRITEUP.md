# Autonomous Powered-Descent GNC: Engineering Writeup

## Project Definition

This project develops a reproducible powered-descent GNC simulation for a
reusable vertical-landing vehicle. The mature navigation and optimization
campaigns use a planar plant; a separate 3D 6-DOF milestone adds quaternion
attitude, variable mass properties, crosswind moments, four-engine wrench
allocation, and finite-rate actuators. The objective is not to replicate a
proprietary launch vehicle. It is to demonstrate the engineering sequence used
to mature and bound a GNC design:

```text
model dynamics -> close the nominal loop -> expose dispersions -> classify failures
-> redesign guidance -> remove perfect-state assumptions -> add actuator dynamics
-> inject faults -> retarget around hazards -> verify constraints
```

## Vehicle Model

The state contains downrange position, altitude, translational velocity, pitch angle, pitch rate, and instantaneous mass. Thrust acts along the body axis, aerodynamic drag opposes wind-relative velocity, TVC produces a pitch moment through the engine lever arm, and mass decreases according to specific impulse.

The model is nonlinear because thrust projection depends on attitude, aerodynamic force depends quadratically on relative speed, TVC torque contains $\sin\delta$, and acceleration changes as mass depletes. RK4 integration is used so the coupled translational and rotational states are advanced with fourth-order local accuracy for smooth commands.

## Guidance and Control Development

The first vertical law uses the stopping-distance relation $v_z^2=2a_bz$ to define a descent-rate corridor. Lateral acceleration begins as proportional-derivative feedback in downrange position and velocity. Desired body tilt is obtained from the commanded acceleration vector, and a pitch PD loop commands gimbal angle.

The nominal controller lands, but only `46.5%` of 200 dispersed cases satisfy all touchdown constraints. The failures separate into vertical-speed violations and pad misses. This diagnosis motivates corridor guidance: lateral error is removed earlier, terminal vertical gains are increased, and allowable late tilt is reduced when descent rate remains high. On the same random cases, success rises to `92.0%`.

The physics is a shared acceleration budget. Lateral correction requires $T\sin\theta$, while braking depends on $T\cos\theta-mg$. Guidance cannot optimize these channels independently.

## Navigation and Actuation

The next phase removes truth-state feedback. Biased, noisy measurements are sampled at 10 Hz and processed by a fixed-gain alpha-beta estimator with innovation gating. Guidance uses the estimate, while the dynamics continue to propagate the hidden truth state.

Throttle and TVC commands pass through delay, first-order lag, deadband, slew-rate limits, and saturation. These dynamics introduce phase lag and limit transient authority. The truth-feedback/full-actuator Monte Carlo succeeds in `95.0%` of cases; estimated feedback succeeds in `66.5%`. The difference quantifies the cost of navigation error in the actual closed loop.

## Error-State Inertial Navigation

The fixed-gain estimator is retained as a controlled baseline. The upgraded navigation architecture uses an eight-state planar error-state EKF containing position, velocity, pitch, two body-frame accelerometer biases, and gyro bias. The nominal trajectory propagates measured specific force through the nonlinear body-to-inertial rotation, subtracts gravity, and integrates velocity and position. The local covariance includes the derivative of inertial acceleration with respect to attitude, so pitch uncertainty correctly enters the translational error dynamics.

GPS updates position and velocity at 5 Hz, radar constrains altitude at 10 Hz, and an independent attitude aid constrains pitch at 20 Hz. Corrections use covariance-normalized innovations and Joseph-form covariance updates. The nominal mean NEES is `6.52` for eight states, while normalized NIS is near one for every aiding channel. This indicates a slightly conservative covariance model rather than a filter whose error bars collapse below observed error.

On the same 200 dispersions, ESKF feedback succeeds in `93.0%` of cases versus `66.5%` for the alpha-beta baseline. P95 landing error decreases by `1.76 m`, and p95 touchdown speed decreases by `1.04 m/s`. The improvement comes from physically propagating acceleration and estimating inertial bias, not from changing guidance gains.

A 20 s GPS outage remains landable because radar and attitude aiding preserve partial observability while horizontal covariance expands. A persistent `+12 m` radar step is rejected `344` times by the scalar NIS gate; GPS then carries altitude observability. These cases demonstrate fault exclusion under modeled redundancy, not flight-qualified sensor fault management.

## Fault Response

A persistent `+12 m` altitude bias is injected after seven seconds. The innovation gate rejects the inconsistent altitude corrections and the vehicle lands, but flight time and propellant use increase. The estimator has traded measurement availability for bounded fault contamination.

An 18% delivered-thrust decrement causes a pad miss even though speed and propellant remain acceptable. Reduced thrust also reduces TVC moment authority. This result identifies a time-and-authority boundary rather than a fuel boundary.

## Hazard-Relative Landing

A discrete target selector excludes candidate sites that violate debris-zone clearance. The chosen `+12 m` target is passed into the unchanged corridor guidance and full navigation/actuator stack. The vehicle lands `5.47 m` from the nearest hazard edge.

Keeping target selection separate from continuous guidance makes the verification traceable: geometric clearance is tested independently, then the full closed-loop trajectory is checked against terminal requirements.

## Performance and Feasibility

A same-condition target sweep shows nearly constant propellant consumption across successful 6-18 m lateral corrections. At the small body angles used, the vertical thrust penalty varies approximately with $\theta^2/2$, so vertical gravity loss dominates total impulse. A 30 m correction fails despite remaining fuel, again demonstrating that propellant inventory does not guarantee reachable lateral impulse within finite time.

A 30-point terminal-condition grid samples altitude and crossrange. It provides evidence of the closed-loop feasible region but is not claimed as a formal reachable set. The nonmonotonic boundary reflects variation in initial descent energy and guidance/actuator phase.

## Constrained Predictive Guidance

The final guidance phase converts the high-altitude landing problem into a
12-node direct-transcription QP. Future positions and velocities are condensed
analytically into horizontal and vertical acceleration decisions. The
objective tracks cubic state references and terminal conditions while
penalizing acceleration and acceleration slew.

The optimizer enforces a linear tilt cone, a conservative 12-sided
approximation to the maximum-thrust disk, nonnegative altitude, acceleration
slew, and a terrain-relative glide slope. It replans every `0.60 s` using the
ESKF state. Below `160 m`, it hands off to corridor guidance so the previously
verified terminal law manages minimum-throttle switching and actuator lag.
The architecture is therefore a planar convex relaxation with hybrid
supervision, not a claim of lossless 6-DOF powered-descent convexification.

On the same 200 ESKF/actuator dispersions, predictive guidance succeeds in
`195 / 200` cases versus `186 / 200` for corridor guidance. P95 absolute pad
error decreases from `3.18 m` to `2.63 m`, while p95 touchdown speed changes
from `0.92 m/s` to `0.96 m/s`. The remaining five failures are pad misses,
which preserves a visible finite-time lateral-authority boundary.

The `48 m` deterministic divert identifies the active physics. Its
glide-slope margin approaches zero while tilt and maximum-thrust margins
remain positive. The vehicle succeeds by spreading lateral acceleration and
counter-acceleration over the available time-to-go, not by saturating control.
The adjacent `50 m` case fails the `3 m` footprint criterion, so the result is
reported as a sampled, locally nonmonotonic closed-loop boundary.

The in-repository ADMM solver exposes both primal/dual optimality residuals and
independently evaluated constraint violation. Across the campaign, `99.90%`
of replans are accepted as feasible, `74.22%` meet the tighter strict
convergence test, and four replans use deterministic corridor fallback. The
largest rejected-iterate violation remains in the output rather than being
filtered from the portfolio evidence.

## 3D 6-DOF Rigid-Body Extension

The 3D state stores inertial position and velocity, a scalar-first
body-to-inertial quaternion, body angular velocity, and instantaneous mass.
The translational plant rotates thrust and aerodynamic force from body to
inertial coordinates. The rotational plant retains
$I\dot{\omega}$, $\dot I\omega$, and
$\omega\times(I\omega)$, while propellant fraction schedules diagonal inertia
and the engine-to-CM lever arm. RK4 integration keeps quaternion norm error
near machine precision without using Euler angles as propagated states.

The guidance layer combines an energy-derived vertical braking reference with
altitude-gated approach, flare, and terminal descent speeds. Horizontal
position, velocity, and corridor feedback create the desired inertial force;
the direction of that force defines the desired body thrust axis. Quaternion
PD feedback commands body torque with gyroscopic compensation.

Four engine-force vectors are mapped into a six-axis vehicle wrench through
the engine-position cross-product matrices. A weighted least-squares allocator
fits the commanded force and moment before projecting each engine onto
nonnegative thrust, gimbal-cone, per-engine thrust, and minimum-throttle
constraints. Static allocation residual is logged separately from the residual
after command delay, lag, and rate limiting.

The calm and `12/-6 m/s` crosswind cases pass. The high-crosswind and
mid-descent engine-out cases are retained as different failure mechanisms.
High wind leaves `6.16 m` terminal position error because the scheduled PD law
has no integral or wind-feedforward channel. Engine-out raises p95 static
allocation residual from below `0.005` to `0.301` and lands `10.16 m` from
target because the asymmetric three-engine wrench set cannot track the
requested force and moment. Both cases retain propellant, demonstrating why
fuel inventory alone is not a recoverability metric.

## 6-DOF Nonlinear MPC Guidance

The latest milestone places a reduced-order nonlinear predictor inside the
6-DOF loop. Its state contains inertial position and velocity, the inertial
body-thrust axis, closed-loop tilt rate, and mass. The control sequence is
specific thrust in inertial coordinates. Each prediction therefore retains
the principal guidance couplings:

```text
a_I = |u| b_3,I + g_I + D_I / m
m_dot = -m |u| / (Isp g0)
```

The desired thrust-axis direction is not applied instantaneously. A
second-order attitude-response model predicts finite quaternion-loop
bandwidth, and the resulting thrust axis determines the acceleration actually
used in the shooting rollout. Thrust magnitude is limited by current mass and
active-engine count; lateral specific thrust is projected into the tilt cone.

The objective tracks a cubic terminal-boundary reference while penalizing
thrust, slew, thrust-axis lag, body rate, terrain-relative corridor violation,
and loss of propellant reserve. Finite-difference Gauss-Newton steps are
bounded by an adaptive trust region. Every candidate is reprojected into the
control set and accepted only if a fresh nonlinear rollout reduces the merit
function. Direct shooting enforces the reduced dynamics in each candidate, so
the virtual-control defect used by successive-convexification formulations is
not required.

On 24 matched 3D state, mass, attitude, and wind dispersions, scheduled
feedback succeeds in `17/24` cases and NMPC in `19/24`. P95 horizontal error
falls from `3.95 m` to `3.48 m`. Median modeled propellant remaining rises
from `3128 kg` to `3779 kg` while p95 vertical touchdown speed remains near
`1.20 m/s`. The optimizer shortens powered descent and reduces gravity loss;
the reserve improvement is not obtained by accepting a harder vertical
landing.

The deterministic `18/-10 m/s` wind case remains a failure. NMPC reduces its
miss from `6.16 m` to `4.88 m`, demonstrating improved wind compensation but
not elimination of the finite thrust/time-to-go boundary. Engine loss
invalidates the nominal plan immediately, yet the present three-engine
allocator still cannot realize the requested six-axis wrench. This failure is
retained because plan feasibility does not imply control-allocation
feasibility.

Observed mean, p95, and maximum solve times are `146`, `228`, and `275 ms`
against the `800 ms` replan period. These measurements establish timing margin
on the development runtime, not a certified onboard worst-case execution
time.

## Verification Summary

- `29` deterministic unit and system tests cover planar and 6-DOF dynamics,
  quaternion conventions, variable inertia, engine allocation, both guidance
  architectures, QP transcription and feasibility, nonlinear MPC trust-region
  projection and rollout acceptance, Monte Carlo
  reproducibility, both estimator architectures, covariance propagation,
  innovation rejection, sensor dropout, actuator rates, hazard geometry, and
  advanced scenarios.
- All plotted evidence is regenerated from committed CSV/JSON outputs.
- Monte Carlo campaigns use a fixed seed and identical dispersion draws for controlled comparisons.
- Failure cases are retained and explained rather than removed from the presentation.
- Assumptions and non-modeled physics are explicitly listed.

## What I Would Improve Next

1. Promote the planar ESKF into the 6-DOF loop with a 15-state inertial error
   model, asynchronous 3D aiding, wind estimation, and terrain-relative
   measurements.
2. Replace the reduced thrust-axis predictor with a sparse full-attitude
   successive-convexification or real-time-iteration NMPC formulation using
   analytic derivatives and explicit state constraints.
3. Add terrain-relative sensing and probabilistic hazard-map uncertainty.
4. Solve an allocation-aware engine-out contingency problem over the
   asymmetric three-engine attainable wrench set.
5. Add timestamp jitter, delayed measurements, out-of-sequence updates, and processor timing.
6. Build a hardware-in-the-loop version using the separate two-axis TVC test-stand project.

The project is strongest as an engineering development record: each added layer changes measurable closed-loop behavior, and each limitation points to a testable next model rather than an unsupported claim.
