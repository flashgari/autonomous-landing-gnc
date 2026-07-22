#!/usr/bin/env python3
"""Generate a browser-viewable nominal landing animation."""

import csv
import json
from pathlib import Path


def load_rows(path):
    with path.open() as f:
        raw = list(csv.DictReader(f))
    stride = max(1, len(raw) // 180)
    rows = raw[::stride]
    if rows[-1] is not raw[-1]:
        rows.append(raw[-1])
    keys = ["time_s", "x_m", "z_m", "theta_deg", "throttle", "gimbal_deg", "prop_remaining_kg"]
    return [{k: round(float(row[k]), 4) for k in keys} for row in rows]


def main():
    rows = load_rows(Path("outputs/nominal_landing.csv"))
    payload = json.dumps(rows, separators=(",", ":"))
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Nominal Autonomous Landing Animation</title>
  <style>
    :root {{
      color-scheme: light dark;
      --bg: #f8fafc;
      --fg: #0f172a;
      --muted: #475569;
      --panel: #ffffff;
      --border: #cbd5e1;
      --blue: #2563eb;
      --red: #dc2626;
      --green: #059669;
      --purple: #7c3aed;
    }}
    @media (prefers-color-scheme: dark) {{
      :root {{
        --bg: #0b1120;
        --fg: #f8fafc;
        --muted: #cbd5e1;
        --panel: #111827;
        --border: #334155;
      }}
    }}
    * {{ box-sizing: border-box; }}
    body {{ margin: 0; background: var(--bg); color: var(--fg); font-family: Arial, Helvetica, sans-serif; }}
    main {{ max-width: 1100px; margin: 0 auto; padding: 28px; }}
    h1 {{ margin: 0 0 8px; font-size: clamp(28px, 5vw, 46px); }}
    p {{ margin: 0 0 18px; color: var(--muted); font-size: 17px; line-height: 1.45; }}
    .controls {{ display: flex; gap: 12px; flex-wrap: wrap; align-items: center; margin: 18px 0; }}
    button, select {{ min-height: 42px; border: 1px solid var(--border); border-radius: 8px; background: var(--panel); color: var(--fg); padding: 0 14px; font: inherit; font-weight: 700; }}
    input {{ width: min(560px, 66vw); accent-color: var(--blue); }}
    label {{ color: var(--muted); font-weight: 700; display: flex; gap: 10px; align-items: center; }}
    .stats {{ display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; margin-bottom: 14px; }}
    .stat {{ border: 1px solid var(--border); background: var(--panel); border-radius: 8px; padding: 10px 12px; }}
    .stat span {{ display: block; color: var(--muted); font-size: 13px; font-weight: 700; }}
    .stat b {{ display: block; font-size: 22px; margin-top: 3px; }}
    svg {{ width: 100%; height: auto; display: block; border: 1px solid var(--border); border-radius: 10px; background: var(--panel); }}
    text {{ fill: var(--fg); font-family: Arial, Helvetica, sans-serif; }}
    .muted {{ fill: var(--muted); font-size: 14px; }}
    .grid {{ stroke: var(--border); stroke-width: 1; opacity: 0.7; }}
    .trajectory {{ fill: none; stroke: var(--blue); stroke-width: 3; }}
    .vehicle {{ fill: var(--red); stroke: currentColor; stroke-width: 2; }}
    .flame {{ fill: var(--green); opacity: 0.85; }}
    .thrust {{ stroke: var(--purple); stroke-width: 4; stroke-linecap: round; }}
    @media (max-width: 760px) {{
      main {{ padding: 18px; }}
      .stats {{ grid-template-columns: 1fr 1fr; }}
      label, input {{ width: 100%; }}
    }}
  </style>
</head>
<body>
  <main>
    <h1>Nominal Autonomous Landing</h1>
    <p>Playback of the baseline powered-descent simulation. Body tilt provides lateral acceleration, throttle manages vertical energy, and gimbal command supplies attitude-control torque.</p>
    <div class="controls">
      <button id="play" type="button">Pause</button>
      <label for="slider">time <input id="slider" type="range" min="0" max="1000" value="0"></label>
      <label for="speed">speed <select id="speed"><option value="0.5">0.5x</option><option value="1" selected>1x</option><option value="2">2x</option></select></label>
    </div>
    <section class="stats" aria-label="Simulation state">
      <div class="stat"><span>altitude</span><b id="alt">0 m</b></div>
      <div class="stat"><span>crossrange</span><b id="xerr">0 m</b></div>
      <div class="stat"><span>throttle</span><b id="thr">0%</b></div>
      <div class="stat"><span>gimbal</span><b id="gim">0 deg</b></div>
    </section>
    <svg viewBox="0 0 980 560" role="img" aria-label="Landing trajectory animation">
      <line x1="80" y1="500" x2="920" y2="500" class="grid"/>
      <line x1="490" y1="70" x2="490" y2="500" class="grid"/>
      <rect x="448" y="500" width="84" height="10" fill="var(--green)"/>
      <text x="444" y="532" class="muted">landing pad</text>
      <path id="trail" class="trajectory" d=""/>
      <g id="vehicle">
        <polygon class="flame" points="-10,28 0,62 10,28"/>
        <rect class="vehicle" x="-12" y="-32" width="24" height="64" rx="4"/>
        <polygon class="vehicle" points="-12,-32 0,-52 12,-32"/>
        <line id="thrustLine" x1="0" y1="34" x2="0" y2="76" class="thrust"/>
      </g>
      <text x="80" y="50" class="muted">Trajectory scale is normalized to the simulated initial altitude and crossrange.</text>
    </svg>
  </main>
  <script>
    const rows = {payload};
    const xMax = Math.max(...rows.map(r => Math.abs(r.x_m)), 1);
    const zMax = Math.max(...rows.map(r => r.z_m), 1);
    const vehicle = document.getElementById("vehicle");
    const trail = document.getElementById("trail");
    const slider = document.getElementById("slider");
    const play = document.getElementById("play");
    const speed = document.getElementById("speed");
    const alt = document.getElementById("alt");
    const xerr = document.getElementById("xerr");
    const thr = document.getElementById("thr");
    const gim = document.getElementById("gim");
    let playing = true;
    let idx = 0;
    let last = performance.now();

    function sx(x) {{ return 490 + x / xMax * 310; }}
    function sy(z) {{ return 500 - z / zMax * 420; }}
    function setFrame(i) {{
      idx = Math.max(0, Math.min(rows.length - 1, i));
      const r = rows[idx];
      vehicle.setAttribute("transform", `translate(${{sx(r.x_m).toFixed(1)}} ${{sy(r.z_m).toFixed(1)}}) rotate(${{r.theta_deg.toFixed(2)}})`);
      const d = rows.slice(0, idx + 1).map((p, j) => `${{j === 0 ? "M" : "L"}}${{sx(p.x_m).toFixed(1)}},${{sy(p.z_m).toFixed(1)}}`).join(" ");
      trail.setAttribute("d", d);
      alt.textContent = `${{r.z_m.toFixed(1)}} m`;
      xerr.textContent = `${{r.x_m.toFixed(1)}} m`;
      thr.textContent = `${{Math.round(r.throttle * 100)}}%`;
      gim.textContent = `${{r.gimbal_deg.toFixed(2)}} deg`;
      slider.value = Math.round(idx / (rows.length - 1) * 1000);
    }}
    function tick(now) {{
      const dt = now - last;
      last = now;
      if (playing && dt > 0) {{
        const step = Math.max(1, Math.round(dt / 55 * Number(speed.value)));
        setFrame((idx + step) % rows.length);
      }}
      requestAnimationFrame(tick);
    }}
    play.addEventListener("click", () => {{
      playing = !playing;
      play.textContent = playing ? "Pause" : "Play";
      last = performance.now();
    }});
    slider.addEventListener("input", () => {{
      playing = false;
      play.textContent = "Play";
      setFrame(Math.round(Number(slider.value) / 1000 * (rows.length - 1)));
    }});
    setFrame(0);
    requestAnimationFrame(tick);
  </script>
</body>
</html>
"""
    out = Path("media/nominal_landing_animation.html")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()

