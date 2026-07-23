# Project Plan

## Portfolio Objective

Create a zero-cost software project that demonstrates reusable-launch-vehicle landing GNC: powered descent dynamics, guidance laws, control allocation, actuator constraints, navigation effects, robustness testing, and clear visual evidence.

## Week 1 Baseline

- 2D powered-descent dynamics
- vehicle mass depletion
- aerodynamic drag with wind-relative velocity
- baseline vertical descent corridor
- baseline lateral divert guidance
- TVC attitude-control torque
- nominal landing output CSV/JSON
- summary plot and animation

## Week 2 Robustness

- [x] randomized initial position/velocity dispersions
- [x] randomized wind dispersions
- [x] randomized mass/thrust/drag uncertainty
- [x] Monte Carlo landing success metrics
- [x] landing footprint plot
- [x] physical interpretation of failure modes

## Week 3 Navigation

Before navigation modeling, the project added a guidance-improvement comparison:

- [x] baseline vs corridor guidance mode
- [x] same-seed Monte Carlo comparison
- [x] success/failure mode comparison plot
- [x] upper-division physical interpretation of the guidance trade

## Week 4 Navigation

- [x] simulated position/altitude, velocity, attitude, and rate measurements
- [x] sensor sample noise and run-constant bias
- [x] innovation-gated alpha-beta state estimator
- [x] truth-state vs estimated-state Monte Carlo comparison
- [x] upper-division analysis of estimator lag and terminal guidance

## Week 5 Advanced Control

- [x] throttle/gimbal delay, lag, deadband, rate limits, and saturation
- [x] deterministic thrust-loss and sensor-bias faults
- [x] hazard-relative target selection and divert scenario
- [x] propellant/divert sweep and terminal-condition map
- [x] compare corridor guidance with constrained predictive guidance
- [x] replace fixed-gain estimator with an IMU-driven error-state EKF

## Week 6 Inertial Navigation

- [x] body-frame accelerometer and gyro measurement model
- [x] accelerometer- and gyro-bias random walks
- [x] eight-state nonlinear strapdown propagation and covariance linearization
- [x] asynchronous GPS, radar-altimeter, and attitude updates
- [x] Joseph-form covariance correction and NIS gating
- [x] NEES, normalized NIS, and three-sigma consistency evidence
- [x] deterministic GPS-dropout and radar-bias fault cases
- [x] matched-seed 200-case alpha-beta versus ESKF comparison

## Week 7 Portfolio Polish

- [x] recruiter-facing README with animation first
- [x] figure index with result-by-result physical interpretation
- [x] complete flight-physics and subsystem writeups
- [x] advanced animation and verification visuals
- [x] requirement-to-evidence verification matrix
- [x] final engineering narrative

## Week 8 Constrained Predictive Guidance

- [x] 12-node condensed direct-transcription prediction model
- [x] quadratic tracking, terminal-state, control, and slew objective
- [x] explicit tilt-cone, thrust, altitude, glide-slope, and slew constraints
- [x] in-repository ADMM solver with warm start and residual telemetry
- [x] feasible-plan acceptance separated from strict optimality convergence
- [x] deterministic corridor fallback for rejected QP iterates
- [x] hybrid minimum-throttle supervisor and 160 m terminal handoff
- [x] matched-seed 200-case corridor/predictive comparison
- [x] fine-step reachability sweep and retained failure boundary
- [x] upper-division formulation, active-constraint, and result interpretation
