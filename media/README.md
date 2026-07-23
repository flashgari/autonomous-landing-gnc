# Media

Browser-viewable media for the landing GNC project.

| File | Purpose |
| --- | --- |
| `nominal_landing_animation.html` | Interactive playback of the nominal powered landing trajectory. |
| `hazard_divert_landing_animation.html` | Full-stack hazard-relative landing with true and estimated trajectories. |
| `hazard_divert_landing_preview.gif` | GitHub-renderable preview of the full-stack landing. |

The animation is generated from `outputs/nominal_landing.csv` by:

```bash
python3 scripts/make_landing_animation.py
python3 scripts/make_advanced_landing_animation.py
python3 scripts/make_landing_gif.py
```

Each animation is generated directly from a simulator CSV. The advanced version displays navigation separation and hazard clearance, but it remains a visual explanation of the numerical result rather than a separate physics model.
