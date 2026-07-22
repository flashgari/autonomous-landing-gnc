# Monte Carlo Robustness

## Objective

Evaluate whether the baseline powered-landing guidance law survives realistic dispersions in initial condition, wind, thrust, propellant loading, air density, and drag coefficient.

Monte Carlo testing is important because a nominal landing only proves that one carefully selected trajectory works. Robust landing GNC must tolerate uncertainty:

```text
initial-state dispersions + environment uncertainty + vehicle uncertainty -> landing footprint
```

## Dispersion Set

The Week 2 campaign uses `200` randomized cases with fixed seed `4242`.

Randomized quantities:

| Quantity | Range / distribution | Physical meaning |
| --- | --- | --- |
| initial crossrange | nominal plus Gaussian dispersion | navigation/targeting error before landing burn |
| initial horizontal velocity | nominal plus Gaussian dispersion | residual divert velocity |
| initial vertical velocity | nominal plus Gaussian dispersion | entry/descent energy uncertainty |
| initial tilt | nominal plus Gaussian dispersion | attitude initialization error |
| wind | uniform crosswind/tailwind range | wind-relative drag and footprint shift |
| thrust scale | uniform `+/-3%` | engine performance uncertainty |
| propellant scale | uniform mass uncertainty | burnout mass and thrust-to-weight variation |
| drag scale | uniform aerodynamic uncertainty | simplified `C_D` uncertainty |
| density scale | uniform atmosphere uncertainty | dynamic-pressure uncertainty |

## Result

Current baseline:

```text
cases:                 200
success rate:          46.5%
p50 |landing error|:   2.39 m
p95 |landing error|:   5.26 m
p50 touchdown speed:   2.44 m/s
p95 touchdown speed:   2.66 m/s
min prop remaining:    4153.5 kg
max tilt:              6.06 deg
max gimbal:            5.19 deg
```

Failure modes:

```text
success:        93
vertical_speed: 57
pad_miss:       50
```

## Figure

![Monte Carlo landing dispersion](../figures/monte_carlo_landing_dispersion.svg)

## Upper-Division Physical Interpretation

The baseline controller is not propellant-limited. The minimum remaining propellant is still above `4,100 kg`, so most failures are not caused by running out of fuel. The limiting issue is guidance accuracy and terminal constraint satisfaction.

The two dominant failure classes are physically different:

- **Vertical-speed failures** occur when the descent-rate corridor and throttle law do not remove enough vertical kinetic energy before touchdown. This points to vertical guidance aggressiveness, throttle floor/cutoff logic, and sensitivity to initial `vz`.
- **Pad misses** occur when lateral divert correction is too conservative for the dispersed crossrange and horizontal-velocity errors. Increasing lateral authority can reduce pad misses, but it also demands more body tilt, which reduces vertical thrust projection through `T cos(theta)`.

This creates a real GNC trade:

```text
more lateral correction -> more tilt -> less vertical thrust margin
```

The current Monte Carlo result therefore motivates a better terminal guidance law rather than simply increasing gains. A strong next version should coordinate vertical and lateral acceleration commands inside a shared thrust/tilt envelope.

## Why This Is Portfolio-Relevant

This result is valuable because it is honest. The nominal trajectory lands, but the Monte Carlo campaign shows that the first guidance law is not robust enough. That is how flight software is actually developed: a baseline controller is exposed to dispersions, failure modes are classified, and the next design iteration is driven by the dominant physical constraint.

## Next Design Response

Rev B guidance should focus on:

- improving vertical braking near touchdown
- adding an explicit glide-slope or landing corridor
- increasing lateral correction earlier in descent
- limiting late lateral tilt commands when vertical margin is tight
- comparing baseline PD guidance against an LQR/MPC-inspired terminal controller

