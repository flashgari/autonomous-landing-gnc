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
- [ ] compare corridor guidance with constrained MPC or convex guidance
- [ ] replace fixed-gain estimator with an error-state EKF

## Week 6 Portfolio Polish

- [x] recruiter-facing README with animation first
- [x] figure index with result-by-result physical interpretation
- [x] complete flight-physics and subsystem writeups
- [x] advanced animation and verification visuals
- [x] requirement-to-evidence verification matrix
- [x] final engineering narrative
