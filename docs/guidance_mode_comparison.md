# Guidance Mode Comparison

## Objective

Compare the Week 2 baseline guidance law against a Week 3 corridor guidance law using the same Monte Carlo dispersions.

The engineering sequence is:

```text
nominal landing -> Monte Carlo failure modes -> revised guidance law -> same dispersions -> measured improvement
```

This is the core of a credible GNC project. A controller is not judged only by one successful trajectory; it is judged by how its failure distribution changes under uncertainty.

## Guidance Modes

### Baseline Guidance

The baseline law uses:

```text
vz_ref = -sqrt(2 a_brake z)
ax_cmd = -Kx x - Kvx vx
theta_cmd = atan2(ax_cmd, g + az_cmd)
```

It lands the nominal case, but Monte Carlo showed two dominant failure modes:

- pad misses
- vertical-speed misses

### Corridor Guidance

The corridor law adds altitude scheduling and an explicit lateral corridor:

```text
|x|_allowed = 0.65 + 0.020 z
```

If the vehicle is outside that corridor, it increases lateral correction earlier in descent. Near the ground, it reduces allowable tilt when vertical speed is still high. It also tightens terminal vertical braking below low altitude.

This directly attacks the failure modes:

- lateral errors are corrected earlier, before the vehicle is close to touchdown
- late lateral tilt is limited so vertical thrust margin is preserved
- terminal descent-rate tracking is tightened near the pad

## Result

Both modes used the same `200` dispersions and seed `4242`.

![Guidance mode comparison](../figures/guidance_mode_comparison.svg)

| Metric | Baseline | Corridor | Change |
| --- | ---: | ---: | ---: |
| success rate | `46.5%` | `92.0%` | `+45.5 percentage points` |
| successes | `93 / 200` | `184 / 200` | `+91 cases` |
| p95 landing error | `5.26 m` | `3.63 m` | `-1.63 m` |
| p95 touchdown speed | `2.66 m/s` | `0.82 m/s` | `-1.84 m/s` |
| max tilt | `6.06 deg` | `5.92 deg` | `-0.14 deg` |
| max gimbal | `5.19 deg` | `5.07 deg` | `-0.13 deg` |

Failure modes changed from:

```text
baseline: success 93, pad_miss 50, vertical_speed 57
corridor: success 184, pad_miss 16
```

## Upper-Division Physical Interpretation

The improvement is not magic gain tuning. It comes from respecting the coupled acceleration budget.

For planar landing:

```text
m x_ddot = T sin(theta) + D_x
m z_ddot = T cos(theta) + D_z - m g
```

Lateral correction requires nonzero `theta`, but vertical braking depends on `T cos(theta)`. A guidance law that waits too long to correct lateral error must demand larger late tilt. That makes pad error smaller but consumes vertical thrust projection, increasing hard-landing risk.

Corridor guidance improves robustness by moving lateral correction earlier in the descent, when there is more altitude and time available. Near touchdown, it protects the vertical channel by reducing allowed tilt if descent rate is still high. This is why vertical-speed failures disappear in the 200-case campaign.

The p95 touchdown-speed reduction is especially important. It shows that the new guidance law did not merely trade hard landings for better pad accuracy. It reduced terminal energy error while also improving landing dispersion.

## Limitation Exposed for the Predictive Phase

Corridor guidance still has 16 pad misses. That means the controller is not fully robust to the sampled lateral dispersions. The next project phase therefore added navigation errors and actuator dynamics rather than simply raising lateral gains. That phase showed that estimated-state feedback becomes the dominant limitation.

The later predictive phase addresses this result directly with a
finite-horizon acceleration QP. It allocates lateral and vertical acceleration
inside explicit tilt, thrust, glide-slope, altitude, and slew constraints,
then compares both architectures with the ESKF and actuator stack held fixed.
That continuation preserves the GNC development sequence:

```text
failure classification -> physical cause -> guidance redesign -> robustness comparison
```

See [Constrained Predictive Guidance](constrained_predictive_guidance.md) for
the formulation, solver diagnostics, active-constraint interpretation, and
matched 200-case result.
