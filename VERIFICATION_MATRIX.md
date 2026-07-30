# Verification Matrix

Every requirement is connected to executable evidence. “Pass” means the implemented model satisfies the stated test criterion; it does not mean flight qualification.

| ID | Requirement | Verification method | Evidence | Status |
| --- | --- | --- | --- | --- |
| DYN-01 | Integrate planar translation, rotation, and mass depletion | nominal simulation and unit test | `landing_gnc/dynamics.py`, `tests/test_simulation.py` | pass |
| DYN-02 | Integrate 3D translation, quaternion attitude, body rates, variable inertia, and mass depletion | deterministic RK4 trajectory and quaternion/inertia tests | `landing_gnc/sixdof_dynamics.py`, `tests/test_sixdof_landing.py` | pass |
| DYN-03 | Map wind-relative aerodynamic force through CP/CM separation into a body moment | crosswind trajectory and force/moment telemetry | `outputs/sixdof_crosswind.csv`, `docs/six_dof_landing_dynamics_and_control.md` | pass at modeled fidelity |
| GNC-01 | Land the nominal baseline with positive propellant | trajectory simulation | `outputs/nominal_landing_metrics.json` | pass |
| GNC-02 | Improve dispersion performance with corridor guidance | same-seed 200-case Monte Carlo comparison | `figures/guidance_mode_comparison.svg` | pass |
| GNC-03 | Predict future position and velocity from a transcribed acceleration sequence | analytical constant-acceleration comparison | `tests/test_constrained_guidance.py` | pass |
| GNC-04 | Enforce tilt, maximum thrust, glide slope, altitude, and acceleration slew in the high-altitude plan | QP feasibility test and margin history | `figures/predictive_constraint_activity.svg` | pass to numerical acceptance tolerance |
| GNC-05 | Improve full-stack landing robustness without changing navigation or actuator models | matched-seed 200-case corridor/predictive comparison | `figures/predictive_guidance_comparison.svg` | success improves from 93.0% to 97.5% |
| GNC-06 | Preserve a deterministic fallback when a QP iterate is unacceptable | injected solver acceptance logic and campaign diagnostics | `landing_gnc/constrained_guidance.py`, `outputs/predictive_guidance_campaign.json` | four fallback replans, no hidden application |
| GNC-07 | Demonstrate a large divert near the sampled footprint boundary | deterministic 48 m initial-offset simulation at 0.05 s | `outputs/predictive_48m_divert.csv` | pass at +2.71 m target error |
| GNC-08 | Land the 3D nominal and moderate-crosswind cases inside the terminal corridor | deterministic calm and 12/-6 m/s wind cases | `figures/sixdof_landing_verification.svg` | pass |
| GNC-09 | Identify rather than conceal the high-crosswind terminal-bias boundary | deterministic 18/-10 m/s wind case | `outputs/sixdof_verification_summary.json` | fail boundary retained at 6.16 m error |
| NAV-01 | Generate biased, noisy sampled navigation measurements | deterministic seeded simulation | `landing_gnc/navigation.py` | pass |
| NAV-02 | Estimate position, velocity, attitude, and rate between samples | noise-free tracking and nominal RMS checks | `tests/test_navigation.py`, `figures/navigation_estimation_comparison.svg` | pass |
| NAV-03 | Reject implausible altitude innovations | injected +12 m step fault | `outputs/advanced_scenarios.json` | pass |
| NAV-04 | Propagate planar inertial state and accelerometer/gyro biases | deterministic hover and covariance tests | `landing_gnc/ekf.py`, `tests/test_ekf.py` | pass |
| NAV-05 | Fuse asynchronous GPS, radar-altimeter, and attitude aiding | seeded nominal ESKF simulation | `figures/ekf_consistency.svg` | pass |
| NAV-06 | Maintain covariance consistency at the modeled fidelity | NIS, NEES, and three-sigma coverage checks | `outputs/ekf_navigation_campaign.json` | pass, slightly conservative |
| NAV-07 | Preserve a valid landing through a 20 s GPS outage | deterministic dropout injection | `outputs/ekf_gps_dropout.csv` | pass |
| FDIR-03 | Exclude a persistent +12 m radar-altimeter bias | NIS-gated deterministic fault injection | `outputs/ekf_radar_bias.csv` | pass |
| ACT-01 | Enforce command delay, lag, deadband, slew, and saturation | unit test and full-stack scenario | `tests/test_actuators.py` | pass |
| ACT-02 | Allocate a requested 3D force and all three body moments across four gimbaled engines | wrench-reconstruction unit test and residual telemetry | `landing_gnc/sixdof_control.py`, `tests/test_sixdof_landing.py` | pass |
| ACT-03 | Identify the reduced attainable wrench set after a mid-descent engine failure | engine 1 disabled at 12 s and allocator residual audited | `outputs/sixdof_engine_out.csv`, `figures/sixdof_landing_verification.svg` | boundary identified |
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

The 3D cases use the same numerical limits with vector horizontal quantities:

```text
final altitude <= 0.05 m
sqrt(target_error_x^2 + target_error_y^2) < 3.0 m
sqrt(v_x^2 + v_y^2) < 1.0 m/s
|v_z| < 2.5 m/s
maximum angle between body +z and inertial +z < 12 deg
propellant remaining > 0 kg
```

Static allocator residual and delayed actuator-path residual are reported
separately. A passing trajectory requires terminal-state compliance; low
allocator residual alone is not treated as a landing success criterion.
