# Navigation and State Estimation

## Engineering Question

How much landing performance is lost when guidance no longer receives the exact simulated state?

The comparison holds the vehicle dispersions, corridor guidance, actuator model, Monte Carlo seed, and terminal constraints fixed. The only change is whether feedback uses the propagated truth state or a sampled, noisy navigation estimate.

## Measurement Model

The navigation suite emulates position/altitude, translational velocity, attitude, and angular-rate measurements:

$$
\mathbf{y}_k = \mathbf{h}(\mathbf{x}_k) + \mathbf{b} + \boldsymbol{\nu}_k
$$

where $\mathbf{b}$ is a run-constant bias and $\boldsymbol{\nu}_k$ is zero-mean sample noise. Treating bias separately from white noise matters. White noise can be attenuated by filtering; an unobservable constant bias survives averaging and shifts the closed-loop equilibrium.

The simulated sensor update period is `0.10 s`. The dynamics still integrate at the smaller simulation step. Therefore, navigation must predict between measurement epochs instead of simply forwarding the most recent sample.

## Alpha-Beta Estimator

For each translational axis, the estimator uses a constant-velocity prediction:

$$
\hat p^-_k = \hat p_{k-1} + \Delta t\,\hat v_{k-1}, \qquad
\hat v^-_k = \hat v_{k-1}
$$

The position innovation is:

$$
r_{p,k} = y_{p,k} - \hat p^-_k
$$

and the corrected state is:

$$
\hat p_k = \hat p^-_k + \alpha r_{p,k}
$$

$$
\hat v_{p,k} = \hat v^-_k + \frac{\beta}{\Delta t}r_{p,k}
$$

The velocity estimate then blends this position-derived correction with the direct velocity measurement. Attitude and angular rate use the same predictor-corrector structure. This is not presented as an EKF: no covariance is propagated, and the gains are fixed rather than derived from process and measurement covariance.

## Innovation Gating

A measurement is rejected when its innovation exceeds a physically selected gate:

$$
|r_k| > r_{\max} \Rightarrow \text{measurement correction omitted}
$$

The gate prevents one implausible sample from immediately entering the feedback path. It also has a cost: if a valid maneuver or accumulated model error produces a large innovation, the same gate can reject useful information. A production design would use covariance-normalized residuals, sensor redundancy, fault persistence logic, and explicit mode management.

## Nominal Estimation Result

![Navigation estimation comparison](../figures/navigation_estimation_comparison.svg)

The nominal estimated-state landing succeeds with these RMS errors:

| Estimated state | RMS error |
| --- | ---: |
| horizontal position | `0.40 m` |
| altitude | `0.32 m` |
| horizontal velocity | `0.10 m/s` |
| vertical velocity | `0.51 m/s` |
| pitch attitude | `0.056 deg` |

The vertical-velocity error is the most consequential quantity. Terminal throttle demand is driven by descent-rate error, so an error in $\hat v_z$ appears directly in commanded vertical acceleration. Position noise mainly perturbs the lateral corridor; vertical-velocity error perturbs the rate at which kinetic energy is removed before contact.

## Monte Carlo Consequence

Both feedback modes use the same `200` vehicle/environment dispersions and flight-like actuator model.

| Metric | Truth-state feedback | Estimated-state feedback |
| --- | ---: | ---: |
| success rate | `95.0%` | `66.5%` |
| p95 landing error | `2.96 m` | `4.94 m` |
| p95 touchdown speed | `0.88 m/s` | `1.96 m/s` |
| dominant estimated-state failure | none beyond pad misses | `64` pad misses |

The `28.5` percentage-point reduction is not merely “noise makes control worse.” It identifies a specific closed-loop mechanism:

1. Position and velocity errors perturb the estimated lateral corridor state.
2. The guidance law changes desired lateral acceleration and body tilt.
3. Actuator delay and rate limiting prevent instantaneous attitude correction.
4. The true vehicle accumulates crossrange error while guidance acts on a biased or lagged state.
5. Near touchdown, the tilt schedule intentionally protects vertical thrust margin, so late lateral error cannot always be recovered.

This explains why pad misses dominate while tilt remains within limits. The design is constraint-limited, not unstable.

## What the Result Does and Does Not Prove

The result proves that navigation quality materially changes terminal-constraint satisfaction in this model. It does not validate a flight navigation system. The present filter omits covariance propagation, sensor frame transformations, asynchronous sensor timing beyond a common sample period, accelerometer bias dynamics, radar-altimeter terrain geometry, and navigation-frame initialization.

The appropriate next estimator upgrade is an error-state EKF with IMU propagation and discrete position/altitude updates. Its performance should be judged by normalized innovation consistency and landing metrics, not only by lower RMS state error.
