# Actuator Dynamics and Fault Response

## Why the Command Path Matters

An ideal simulation applies throttle and gimbal commands instantaneously. A physical engine and TVC mechanism cannot. The implemented command path includes first-order response, command delay, finite slew rate, deadband, and hard saturation.

For an actuator state $u$ and command $u_c$:

$$
\dot u = \frac{u_c(t-t_d)-u}{\tau}
$$

subject to:

$$
|\dot u| \leq \dot u_{\max}, \qquad u_{\min} \leq u \leq u_{\max}
$$

The delay contributes phase lag, the time constant attenuates high-frequency commands, the slew limit bounds transient authority, and deadband creates a region where small commands do not move the actuator. These effects are qualitatively different and are modeled separately.

## Coupling to Vehicle Dynamics

Throttle affects translational acceleration and propellant flow:

$$
T = u_T T_{\max}, \qquad \dot m = -\frac{T}{I_{sp}g_0}
$$

Gimbal affects pitch angular acceleration:

$$
I_y\dot\omega = TL\sin\delta - c_\omega\omega
$$

Because TVC moment authority scales with thrust, throttle and attitude-control authority are coupled. A gimbal command issued during low thrust cannot produce the same angular acceleration as the same command during high thrust.

The truth-feedback Monte Carlo success rate changes from `92.0%` with ideal actuators to `95.0%` with the flight-like command path. This small increase must not be interpreted as proof that lag improves a real vehicle. In this simplified model, the actuator dynamics smooth the discontinuous minimum-throttle switching logic and alter terminal phase. The controlled navigation comparison therefore keeps the same actuator model in both branches; only the feedback state source changes.

## Deterministic Scenario Results

![Advanced scenario comparison](../figures/advanced_scenario_comparison.svg)

| Scenario | Result | Target error | Touchdown speed | Propellant remaining |
| --- | --- | ---: | ---: | ---: |
| full estimated-state stack | pass | `-2.08 m` | `1.10 m/s` | `2938 kg` |
| 8% thrust-loss step | pass | `-2.30 m` | `1.16 m/s` | `2950 kg` |
| 18% thrust-loss step | fail | `-4.86 m` | `1.96 m/s` | `3304 kg` |
| +12 m altitude-bias fault | pass | `0.28 m` | `0.80 m/s` | `2484 kg` |

## Thrust-Loss Interpretation

At `t = 5 s`, delivered thrust is reduced in two cases. A reduction to `92%` remains recoverable and lands inside the terminal corridor. A reduction to `82%` crosses the pad-error limit. In a lumped single-engine model these cases represent equivalent engine-cluster authority loss; they are not detailed multi-engine allocation models.

The 18% loss case reaches the ground with acceptable speed but misses the target corridor. More than `3300 kg` of modeled propellant remains, so the failure is not depletion. Reduced thrust lowers both translational acceleration and TVC moment authority. The vehicle spends more of its finite time-to-go servicing vertical braking, while lateral acceleration and attitude response weaken. The result crosses the pad-error boundary before crossing the speed or fuel boundary. The successful 8% case and failed 18% case bracket a fault-tolerance transition without claiming that the exact boundary has been solved continuously.

This is a useful failure because it locates an authority limit. “More propellant remaining” is not evidence of better performance: the reduced-thrust vehicle cannot convert that propellant into sufficient acceleration quickly enough within the available time and thrust envelope.

## Altitude-Bias Fault Interpretation

At `t = 7 s`, the altitude channel receives a persistent `+12 m` bias. The innovation gate rejects `435` inconsistent updates, and the vehicle still lands successfully. The altitude RMS estimation error rises to `3.68 m`, flight time increases from `44.4 s` to `50.6 s`, and remaining propellant falls by approximately `454 kg` relative to the full-stack nominal case.

The physical cost follows from gravity loss. While the altitude channel is rejected, the estimator must rely more heavily on propagation and the unaffected velocity channel. Conservative vertical guidance extends the burn, and every additional second of supported flight requires thrust near vehicle weight. Fault accommodation therefore preserves touchdown constraints at the expense of propellant margin.

## Limits of the Fault Model

- The thrust-loss case does not model engine-out center-of-thrust shift, differential gimbal allocation, plume interaction, or engine restart logic.
- The sensor-fault case uses one persistent bias step rather than a full fault taxonomy.
- Innovation gating is not sufficient by itself for flight fault detection, isolation, and recovery.
- No flight computer scheduling, quantization, communication latency, or redundant voting is modeled.

These limits are explicit so the evidence is interpreted as subsystem-level GNC analysis rather than flight qualification.
