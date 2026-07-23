# Autonomous Powered-Descent GNC: Engineering Writeup

## Project Definition

This project develops a reproducible planar simulation of autonomous powered descent for a reusable vertical-landing vehicle. The objective is not to replicate a proprietary launch vehicle. It is to demonstrate the engineering sequence used to mature a GNC design:

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

## Fault Response

A persistent `+12 m` altitude bias is injected after seven seconds. The innovation gate rejects the inconsistent altitude corrections and the vehicle lands, but flight time and propellant use increase. The estimator has traded measurement availability for bounded fault contamination.

An 18% delivered-thrust decrement causes a pad miss even though speed and propellant remain acceptable. Reduced thrust also reduces TVC moment authority. This result identifies a time-and-authority boundary rather than a fuel boundary.

## Hazard-Relative Landing

A discrete target selector excludes candidate sites that violate debris-zone clearance. The chosen `+12 m` target is passed into the unchanged corridor guidance and full navigation/actuator stack. The vehicle lands `5.47 m` from the nearest hazard edge.

Keeping target selection separate from continuous guidance makes the verification traceable: geometric clearance is tested independently, then the full closed-loop trajectory is checked against terminal requirements.

## Performance and Feasibility

A same-condition target sweep shows nearly constant propellant consumption across successful 6-18 m lateral corrections. At the small body angles used, the vertical thrust penalty varies approximately with $\theta^2/2$, so vertical gravity loss dominates total impulse. A 30 m correction fails despite remaining fuel, again demonstrating that propellant inventory does not guarantee reachable lateral impulse within finite time.

A 30-point terminal-condition grid samples altitude and crossrange. It provides evidence of the closed-loop feasible region but is not claimed as a formal reachable set. The nonmonotonic boundary reflects variation in initial descent energy and guidance/actuator phase.

## Verification Summary

- `12` deterministic unit and system tests cover dynamics, guidance, Monte Carlo reproducibility, estimator behavior, innovation rejection, actuator rates, hazard geometry, and advanced scenarios.
- All plotted evidence is regenerated from committed CSV/JSON outputs.
- Monte Carlo campaigns use a fixed seed and identical dispersion draws for controlled comparisons.
- Failure cases are retained and explained rather than removed from the presentation.
- Assumptions and non-modeled physics are explicitly listed.

## What I Would Improve Next

1. Replace fixed-gain navigation with an error-state EKF using IMU propagation and asynchronous radar/GNSS updates.
2. Replace heuristic corridor guidance with a constrained optimizer or MPC formulation that handles thrust, tilt, glide-slope, and terminal-state limits explicitly.
3. Extend the plant to 6-DOF with engine allocation, aerodynamic moments, inertia variation, and mass-property motion.
4. Add terrain-relative sensing and probabilistic hazard-map uncertainty.
5. Build a hardware-in-the-loop version using the separate two-axis TVC test-stand project.

The project is strongest as an engineering development record: each added layer changes measurable closed-loop behavior, and each limitation points to a testable next model rather than an unsupported claim.
