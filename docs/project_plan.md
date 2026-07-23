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

- simulated barometer/IMU/GNSS-like measurements
- sensor noise and bias
- alpha-beta or Kalman-style state estimator
- comparison of truth-state guidance vs estimated-state guidance
- discussion of estimator lag during terminal descent

## Week 5 Advanced Control

- compare baseline PD guidance with LQR/MPC-inspired terminal control
- add throttle/gimbal rate limits
- add glide-slope/tilt/throttle constraints
- compare propellant use and landing dispersion

## Week 6 Portfolio Polish

- recruiter-facing README
- figure index
- complete physics writeup
- animation and Monte Carlo visuals
- final engineering narrative
