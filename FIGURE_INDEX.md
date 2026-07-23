# Figure Index

Use this page for a fast technical review. Every figure is generated from committed simulator outputs.

## 1. Full-Stack Hazard-Relative Animation

**[Open the interactive landing animation](media/hazard_divert_landing_animation.html)**

Blue is the continuous integrated truth state; purple is the discrete navigation estimate supplied to guidance. The purple corrections are estimator innovations produced by noisy sampled measurements, not physical vehicle zigzags.

The green target is at `x = 12 m`, outside the `[-4, 4] m` debris interval. The S-shaped path is the expected geometry of a lateral impulse-and-brake maneuver. The first curvature builds lateral velocity through $a_x=T\sin\theta/m$; the second reverses lateral acceleration to remove that velocity before touchdown. As altitude decreases, corridor guidance reduces allowable tilt so $T\cos\theta$ is recovered for vertical-energy removal. The result is `2.53 m` target error, `1.09 m/s` touchdown speed, and `5.47 m` hazard clearance.

## 2. Error-State EKF Consistency

![Error-state EKF consistency](figures/ekf_consistency.svg)

The state-error traces are plotted against the filter's own `+/-3 sigma` bounds. Between aiding epochs, IMU noise and bias random walks propagate through the state-transition matrix and expand uncertainty. GPS, radar, and attitude measurements contract the state directions they observe. The small covariance modulation is therefore the expected predictor-corrector cycle, not an oscillation of the physical vehicle.

The nominal mean NEES is `6.52` for eight estimated error states. Mean normalized NIS is near one for GPS (`0.95`), radar (`1.03`), and attitude (`0.97`). Three-sigma containment is at least `99.5%` for horizontal position, altitude, and pitch. The filter is slightly conservative in this nominal case; it is not falsely claiming millimeter accuracy through a collapsed covariance.

## 3. ESKF Architecture and Sensor Faults

![ESKF robustness evidence](figures/ekf_navigation_robustness.svg)

The matched-seed comparison changes only the navigation architecture. ESKF feedback raises success from `66.5%` to `93.0%`, lowers p95 landing error from `4.94 m` to `3.18 m`, and lowers p95 touchdown speed from `1.96 m/s` to `0.92 m/s`. Bias estimation and nonlinear specific-force propagation reduce the guidance error that otherwise survives the fixed-gain alpha-beta filter.

The deterministic cases expose the mechanism. A 20 s GPS outage increases horizontal drift because acceleration-bias uncertainty integrates twice into position, while radar and attitude aiding keep the vertical and rotational channels bounded. A `+12 m` radar step is rejected `344` times by the covariance-normalized scalar innovation gate, leaving GPS to carry altitude observability. Both cases pass, but only within the stated outage duration and sensor-noise assumptions.

## 4. Aiding-Sensor Fault Response

![ESKF aiding-sensor fault response](figures/ekf_sensor_fault_response.svg)

The GPS-outage panel shows why inertial dead reckoning is an uncertainty-growth problem. Horizontal `1 sigma` uncertainty expands from `0.14 m` at loss of GPS to `0.70 m` after 20 s because uncertain acceleration bias integrates into velocity and then position. GPS reacquisition contracts it to `0.17 m` within four seconds. Radar and attitude aiding remain active, so this is partial rather than total loss of observability.

The radar panel uses logarithmic NIS. Before the fault, scalar NIS remains near order one. The `+12 m` step drives the residual many standard deviations beyond the predicted innovation covariance, pushing NIS to order `10^3` above the gate at `9`. Rejecting `344` radar updates prevents the biased channel from contaminating guidance while GPS maintains altitude information.

## 5. Guidance Mode Comparison

![Guidance mode comparison](figures/guidance_mode_comparison.svg)

Success rises from `46.5%` to `92.0%` on identical dispersions. The p95 touchdown speed falls from `2.66 m/s` to `0.82 m/s`, while maximum tilt and gimbal both decrease slightly. This rules out increased control amplitude as the explanation.

The mechanism is altitude scheduling. Baseline guidance carries too much crossrange demand into the terminal phase, where tilting the vehicle reduces vertical thrust projection and competes with descent-rate control. Corridor guidance generates the lateral impulse earlier, when time-to-go is larger, then constrains late tilt. The shift from pad and vertical-speed failures to `184/200` successes is therefore a coupled energy-management and control-allocation result.

## 6. Alpha-Beta Navigation Estimation Comparison

![Navigation estimation comparison](figures/navigation_estimation_comparison.svg)

Truth-state feedback with finite actuators succeeds in `95.0%` of the 200 cases; estimated-state feedback succeeds in `66.5%`. The dominant new failure is target error. Estimator bias and lag perturb the lateral corridor, and the remaining error cannot always be removed after terminal tilt limits prioritize $T\cos\theta$.

## 7. Actuator Fault and Hazard Scenarios

![Advanced scenario comparison](figures/advanced_scenario_comparison.svg)

The trajectories initially overlap because their initial states, navigation realization, guidance law, and actuator model are identical. They diverge only after a fault changes delivered thrust or measured altitude, or after hazard logic selects a different target.

The `8%` thrust decrement remains recoverable; the `18%` decrement crosses the finite-time reachable-set boundary and lands `4.86 m` from target. Its `3304 kg` residual propellant is not contradictory: reduced delivered thrust also reduces mass flow, and the vehicle reaches the ground `4.80 s` before nominal. Fuel inventory is not equivalent to acceleration authority integrated over the remaining time-to-go.

The `+12 m` altitude step is rejected `435` times by innovation gating. The estimator remains bounded, but the descent extends to `50.55 s`, adding gravity loss and consuming about `454 kg` more propellant than nominal. The green hazard-divert path uses an early acceleration and later counter-acceleration to reach a safe site while suppressing touchdown lateral velocity.

## 8. Divert Demand and Propellant

![Propellant performance](figures/propellant_performance.svg)

Successful target changes use nearly the same total propellant because the required body angles are small and the vertical projection penalty is approximately second order in tilt. The largest correction fails with positive propellant, demonstrating that fuel inventory is not equivalent to reachable lateral impulse.

## 9. Sampled Terminal-Condition Map

![Landing feasibility envelope](figures/landing_feasibility_envelope.svg)

The grid tests 30 altitude/offset combinations with flight-like actuators. It is intentionally labeled as a sampled map: each altitude also changes initial descent energy, so nonmonotonic points reflect full-state and guidance-phase dependence rather than a geometric altitude rule.

## 10. Baseline Monte Carlo Dispersion

![Monte Carlo landing dispersion](figures/monte_carlo_landing_dispersion.svg)

The original controller fails through both pad misses and vertical-speed violations while retaining substantial propellant. This figure establishes the failure distribution that motivates corridor guidance.

## 11. Nominal State History

![Nominal landing summary](figures/nominal_landing_summary.svg)

Altitude, descent rate, lateral error, throttle, gimbal, and propellant show the initial closed-loop baseline. It remains useful as a controlled reference, but the later figures carry stronger robustness evidence.

## Supporting Analysis

- [Flight Physics](docs/flight_physics.md)
- [Alpha-Beta Navigation Baseline](docs/navigation_estimation.md)
- [Error-State EKF and Inertial Navigation](docs/error_state_ekf.md)
- [Actuator Dynamics and Fault Response](docs/actuator_fault_response.md)
- [Hazard Divert and Landing Feasibility](docs/hazard_divert_feasibility.md)
- [Verification Matrix](VERIFICATION_MATRIX.md)
- [Complete Engineering Writeup](PORTFOLIO_WRITEUP.md)
