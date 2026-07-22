# Media

Browser-viewable media for the landing GNC project.

| File | Purpose |
| --- | --- |
| `nominal_landing_animation.html` | Interactive playback of the nominal powered landing trajectory. |

The animation is generated from `outputs/nominal_landing.csv` by:

```bash
python3 scripts/make_landing_animation.py
```

It is a visual explanation of the simulated trajectory, not a separate physics model.

