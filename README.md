# Autonomous Landing GNC Simulator

Zero-cost software portfolio project for powered descent guidance, navigation, and control.

## Project Goal

Build a reusable-booster landing simulator that models planar powered descent, throttle/gimbal commands, vehicle attitude, mass depletion, drag, wind, and landing constraints. The project is designed to show GNC reasoning through equations, simulation data, plots, animation, and upper-division physical interpretation.

The engineering story is:

```text
powered-descent dynamics -> guidance law -> attitude/TVC control -> landing metrics -> robustness expansion
```

## Current Status

The initial baseline is a 2D powered-landing simulation with a successful nominal touchdown.

| Item | Status | Evidence |
| --- | --- | --- |
| Planar dynamics model | complete | [landing_gnc/dynamics.py](landing_gnc/dynamics.py) |
| Guidance/control baseline | complete | [landing_gnc/guidance.py](landing_gnc/guidance.py) |
| Nominal simulation runner | complete | [scripts/run_nominal_landing.py](scripts/run_nominal_landing.py) |
| SVG plotting pipeline | complete | [scripts/plot_nominal_landing.py](scripts/plot_nominal_landing.py) |
| Landing animation | complete | [media/nominal_landing_animation.html](media/nominal_landing_animation.html) |
| Physics writeup | complete | [docs/flight_physics.md](docs/flight_physics.md) |
| Monte Carlo robustness | complete | [docs/monte_carlo_robustness.md](docs/monte_carlo_robustness.md) |
| Tests | complete | [tests/](tests/) |

## Visual Evidence

### Nominal Landing Summary

![Nominal landing summary](figures/nominal_landing_summary.svg)

The plot shows the closed-loop descent from initial terminal conditions to touchdown. Altitude decreases smoothly, vertical velocity is reduced before ground contact, horizontal error is driven toward the pad, throttle rises during the braking phase, gimbal remains well inside its authority limit, and propellant remains positive at touchdown.

Upper-division interpretation: the guidance law is shaping a feasible acceleration command while the vehicle mass decreases through `m_dot = -T/(Isp g0)`. The vertical channel is dominated by the trade between gravity loss, thrust-to-weight ratio, and terminal descent-rate constraint. The lateral channel is intentionally conservative: it removes crossrange error with small tilt rather than demanding large attitude excursions that would reduce vertical thrust margin.

### Landing Animation

Open the browser-viewable animation:

[media/nominal_landing_animation.html](media/nominal_landing_animation.html)

The animation shows why landing GNC is a coupled problem: body tilt creates horizontal acceleration but also projects thrust away from the vertical axis. A guidance law that asks for too much lateral correction late in descent can consume vertical thrust margin and produce a hard landing.

For a fast visual review, open [FIGURE_INDEX.md](FIGURE_INDEX.md).

### Monte Carlo Landing Dispersion

![Monte Carlo landing dispersion](figures/monte_carlo_landing_dispersion.svg)

The robustness campaign shows the baseline controller's real limitation. Out of 200 randomized dispersions, the current guidance law succeeds in 46.5% of cases. Failures are dominated by vertical-speed misses and pad misses, not propellant depletion.

Upper-division interpretation: this is the expected next layer after a nominal landing. A single successful trajectory does not prove a landing controller is robust. The Monte Carlo footprint shows how uncertainty in initial state, wind, drag, thrust, and mass maps into terminal constraint violations. The dominant trade is lateral divert versus vertical thrust margin: more tilt helps remove crossrange error, but it reduces `T cos(theta)` available for braking.

## Baseline Result

The nominal run currently lands with:

```text
horizontal error: 2.62 m
touchdown vx:    -0.25 m/s
touchdown vz:    -2.47 m/s
prop remaining:  4588.9 kg
max tilt:        1.59 deg
max gimbal:      0.65 deg
```

This is a first baseline, not the final project. The next steps are to add navigation noise, Monte Carlo wind/mass dispersions, a stronger divert case, and a more rigorous guidance method such as convex-style powered-descent guidance or LQR/MPC-inspired terminal control.

## Run It

```bash
python3 scripts/run_nominal_landing.py
python3 scripts/plot_nominal_landing.py
python3 scripts/make_landing_animation.py
python3 scripts/run_monte_carlo.py
python3 scripts/plot_monte_carlo.py
python3 -m unittest discover tests
```

Outputs:

```text
outputs/nominal_landing.csv
outputs/nominal_landing_metrics.json
outputs/nominal_landing_config.json
figures/nominal_landing_summary.svg
media/nominal_landing_animation.html
outputs/monte_carlo_landing.csv
outputs/monte_carlo_summary.json
figures/monte_carlo_landing_dispersion.svg
```

## Repository Layout

```text
landing_gnc/   dynamics, guidance, models, simulation helpers
scripts/       runnable simulation, plotting, animation generation
docs/          physics and project planning
outputs/       generated CSV/JSON result files
figures/       generated plots
media/         browser-viewable animation
tests/         unit tests
```

## Roadmap

- [x] Planar powered-descent dynamics
- [x] Baseline guidance/control law
- [x] Nominal landing CSV/metrics output
- [x] Summary plot
- [x] Landing animation
- [x] Upper-division physics writeup
- [x] Figure index
- [x] Monte Carlo dispersion campaign
- [ ] Navigation sensor noise and estimator
- [ ] LQR/MPC-inspired terminal controller comparison
- [ ] Landing corridor and constraint visualization
- [ ] Recruiter-facing final writeup
