# Verification Matrix

Every requirement is connected to executable evidence. “Pass” means the implemented model satisfies the stated test criterion; it does not mean flight qualification.

| ID | Requirement | Verification method | Evidence | Status |
| --- | --- | --- | --- | --- |
| DYN-01 | Integrate planar translation, rotation, and mass depletion | nominal simulation and unit test | `landing_gnc/dynamics.py`, `tests/test_simulation.py` | pass |
| GNC-01 | Land the nominal baseline with positive propellant | trajectory simulation | `outputs/nominal_landing_metrics.json` | pass |
| GNC-02 | Improve dispersion performance with corridor guidance | same-seed 200-case Monte Carlo comparison | `figures/guidance_mode_comparison.svg` | pass |
| GNC-03 | Predict future position and velocity from a transcribed acceleration sequence | analytical constant-acceleration comparison | `tests/test_constrained_guidance.py` | pass |
| GNC-04 | Enforce tilt, maximum thrust, glide slope, altitude, and acceleration slew in the high-altitude plan | QP feasibility test and margin history | `figures/predictive_constraint_activity.svg` | pass to numerical acceptance tolerance |
| GNC-05 | Improve full-stack landing robustness without changing navigation or actuator models | matched-seed 200-case corridor/predictive comparison | `figures/predictive_guidance_comparison.svg` | success improves from 93.0% to 97.5% |
| GNC-06 | Preserve a deterministic fallback when a QP iterate is unacceptable | injected solver acceptance logic and campaign diagnostics | `landing_gnc/constrained_guidance.py`, `outputs/predictive_guidance_campaign.json` | four fallback replans, no hidden application |
| GNC-07 | Demonstrate a large divert near the sampled footprint boundary | deterministic 48 m initial-offset simulation at 0.05 s | `outputs/predictive_48m_divert.csv` | pass at +2.71 m target error |
| NAV-01 | Generate biased, noisy sampled navigation measurements | deterministic seeded simulation | `landing_gnc/navigation.py` | pass |
| NAV-02 | Estimate position, velocity, attitude, and rate between samples | noise-free tracking and nominal RMS checks | `tests/test_navigation.py`, `figures/navigation_estimation_comparison.svg` | pass |
| NAV-03 | Reject implausible altitude innovations | injected +12 m step fault | `outputs/advanced_scenarios.json` | pass |
| NAV-04 | Propagate planar inertial state and accelerometer/gyro biases | deterministic hover and covariance tests | `landing_gnc/ekf.py`, `tests/test_ekf.py` | pass |
| NAV-05 | Fuse asynchronous GPS, radar-altimeter, and attitude aiding | seeded nominal ESKF simulation | `figures/ekf_consistency.svg` | pass |
| NAV-06 | Maintain covariance consistency at the modeled fidelity | NIS, NEES, and three-sigma coverage checks | `outputs/ekf_navigation_campaign.json` | pass, slightly conservative |
| NAV-07 | Preserve a valid landing through a 20 s GPS outage | deterministic dropout injection | `outputs/ekf_gps_dropout.csv` | pass |
| FDIR-03 | Exclude a persistent +12 m radar-altimeter bias | NIS-gated deterministic fault injection | `outputs/ekf_radar_bias.csv` | pass |
| ACT-01 | Enforce command delay, lag, deadband, slew, and saturation | unit test and full-stack scenario | `tests/test_actuators.py` | pass |
| ROB-01 | Quantify robustness under vehicle, environment, and initial-state dispersions | fixed-seed 200-case campaigns | `outputs/navigation_comparison.json` | pass |
| ROB-02 | Compare alpha-beta and ESKF feedback on identical dispersions | matched-seed 200-case campaign | `figures/ekf_navigation_robustness.svg` | ESKF improves success by 26.5 points |
| FDIR-01 | Preserve touchdown after a large altitude-channel bias | deterministic fault scenario | `figures/advanced_scenario_comparison.svg` | pass |
| FDIR-02 | Identify loss of landing authority after major thrust decrement | deterministic 18% thrust-loss scenario | `docs/actuator_fault_response.md` | boundary identified |
| HAZ-01 | Select a target outside the hazard interval with at least 3 m clearance | geometry unit test | `tests/test_hazards.py` | pass |
| HAZ-02 | Land the full-stack simulation outside the hazard interval | deterministic divert scenario | `media/hazard_divert_landing_animation.html` | pass |
| PERF-01 | Quantify propellant use across lateral divert demand | controlled target sweep | `figures/propellant_performance.svg` | pass |
| PERF-02 | Sample touchdown feasibility over altitude/offset conditions | 30-case deterministic grid | `figures/landing_feasibility_envelope.svg` | pass |
| SW-01 | Reproduce seeded Monte Carlo outputs | repeated campaigns in unit tests | `tests/test_monte_carlo.py` | pass |
| SW-02 | Report optimizer convergence separately from constraint-feasible acceptance | ADMM residual and violation telemetry | `outputs/monte_carlo_guidance_predictive_ekf_summary.json` | 74.22% strict convergence, 99.90% acceptance |

## Acceptance Criteria

A successful touchdown requires all of the following:

```text
final altitude <= 0.05 m
|target-relative position error| < 3.0 m
|horizontal touchdown velocity| < 1.0 m/s
|vertical touchdown velocity| < 2.5 m/s
maximum body tilt < 12 deg
propellant remaining > 0 kg
```

The criteria are model-level design requirements chosen to make comparisons repeatable. They are not copied from a specific launch vehicle or operational landing requirement.
