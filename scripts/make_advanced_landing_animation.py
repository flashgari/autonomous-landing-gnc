#!/usr/bin/env python3
"""Generate an interactive hazard-divert animation with navigation estimates."""

import csv
import json
from pathlib import Path


def load_rows(path):
    with path.open() as f:
        raw = list(csv.DictReader(f))
    stride = max(1, len(raw) // 240)
    sampled = raw[::stride]
    if sampled[-1] != raw[-1]:
        sampled.append(raw[-1])
    keys = [
        "time_s",
        "x_m",
        "z_m",
        "vx_mps",
        "vz_mps",
        "theta_deg",
        "throttle",
        "gimbal_deg",
        "target_x_m",
        "target_error_m",
        "prop_remaining_kg",
        "estimated_x_m",
        "estimated_z_m",
    ]
    return [{key: round(float(row[key]), 4) for key in keys} for row in sampled]


def main():
    rows = load_rows(Path("outputs/scenario_hazard_divert.csv"))
    summary = json.loads(Path("outputs/advanced_scenarios.json").read_text())
    hazard = summary["hazards"][0]
    metrics = summary["scenarios"]["hazard_divert"]["metrics"]
    payload = json.dumps(rows, separators=(",", ":"))
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Hazard-Relative Autonomous Landing</title>
  <style>
    :root {{ color-scheme: light dark; --bg:#f8fafc; --fg:#0f172a; --muted:#475569; --panel:#fff; --border:#cbd5e1; --truth:#2563eb; --estimate:#7c3aed; --safe:#059669; --danger:#dc2626; --thrust:#ea580c; }}
    @media (prefers-color-scheme: dark) {{ :root {{ --bg:#0b1120; --fg:#f8fafc; --muted:#cbd5e1; --panel:#111827; --border:#334155; }} }}
    * {{ box-sizing:border-box; }}
    body {{ margin:0; background:var(--bg); color:var(--fg); font-family:Arial,Helvetica,sans-serif; }}
    main {{ max-width:1180px; margin:0 auto; padding:28px; }}
    h1 {{ margin:0 0 8px; font-size:clamp(28px,5vw,46px); letter-spacing:0; }}
    .subtitle,.explanation {{ color:var(--muted); font-size:16px; line-height:1.5; }}
    .controls {{ display:flex; gap:12px; flex-wrap:wrap; align-items:center; margin:18px 0; }}
    button,select {{ min-height:42px; border:1px solid var(--border); border-radius:6px; background:var(--panel); color:var(--fg); padding:0 14px; font:inherit; font-weight:700; }}
    input {{ width:min(590px,64vw); accent-color:var(--truth); }}
    label {{ color:var(--muted); font-weight:700; display:flex; gap:10px; align-items:center; }}
    .stats {{ display:grid; grid-template-columns:repeat(5,minmax(0,1fr)); gap:8px; margin-bottom:12px; }}
    .stat {{ border-top:2px solid var(--border); padding:9px 4px; }}
    .stat span {{ display:block; color:var(--muted); font-size:12px; font-weight:700; }}
    .stat b {{ display:block; font-size:20px; margin-top:3px; }}
    svg {{ width:100%; height:auto; display:block; border:1px solid var(--border); border-radius:6px; background:var(--panel); }}
    text {{ fill:var(--fg); font-family:Arial,Helvetica,sans-serif; }}
    .muted {{ fill:var(--muted); font-size:13px; }}
    .grid {{ stroke:var(--border); stroke-width:1; opacity:.72; }}
    .truthTrail {{ fill:none; stroke:var(--truth); stroke-width:3; }}
    .estimateTrail {{ fill:none; stroke:var(--estimate); stroke-width:2; stroke-dasharray:7 5; }}
    .vehicle {{ fill:var(--truth); stroke:var(--fg); stroke-width:1.5; }}
    .estimate {{ fill:var(--estimate); opacity:.72; }}
    .thrust {{ stroke:var(--thrust); stroke-width:4; stroke-linecap:round; }}
    .explanation {{ margin:18px 0 0; max-width:980px; }}
    .live-physics,.physics-grid {{ display:grid; gap:12px; margin-top:14px; }}
    .live-physics {{ grid-template-columns:180px 1fr; border:1px solid var(--border); border-radius:6px; padding:14px; background:var(--panel); }}
    .live-physics b {{ font-size:18px; }}
    .live-physics p {{ margin:0; color:var(--muted); line-height:1.5; }}
    .physics-grid {{ grid-template-columns:repeat(3,minmax(0,1fr)); }}
    .physics-grid section {{ border-top:2px solid var(--border); padding:12px 4px 0; }}
    .physics-grid h2 {{ margin:0 0 7px; font-size:16px; }}
    .physics-grid p {{ margin:0; color:var(--muted); font-size:14px; line-height:1.5; }}
    @media (max-width:760px) {{ main {{ padding:16px; }} .stats {{ grid-template-columns:1fr 1fr; }} .live-physics,.physics-grid {{ grid-template-columns:1fr; }} label,input {{ width:100%; }} }}
  </style>
</head>
<body>
  <main>
    <h1>Hazard-Relative Autonomous Landing</h1>
    <p class="subtitle">Simulated corridor guidance using noisy state estimates, finite-rate throttle/TVC actuators, and a retargeted safe landing site.</p>
    <div class="controls">
      <button id="play" type="button">Pause</button>
      <label for="slider">time <input id="slider" type="range" min="0" max="1000" value="0"></label>
      <label for="speed">speed <select id="speed"><option value="0.5">0.5x</option><option value="1" selected>1x</option><option value="2">2x</option></select></label>
    </div>
    <section class="stats" aria-label="Simulation state">
      <div class="stat"><span>altitude</span><b id="alt">0 m</b></div>
      <div class="stat"><span>target error</span><b id="err">0 m</b></div>
      <div class="stat"><span>speed</span><b id="vel">0 m/s</b></div>
      <div class="stat"><span>applied throttle</span><b id="thr">0%</b></div>
      <div class="stat"><span>navigation separation</span><b id="nav">0 m</b></div>
    </section>
    <svg viewBox="0 0 1040 590" role="img" aria-label="True and estimated hazard-divert landing trajectories">
      <line x1="70" y1="510" x2="970" y2="510" class="grid"/>
      <line x1="520" y1="55" x2="520" y2="510" class="grid"/>
      <rect id="hazard" y="486" height="24" fill="var(--danger)" opacity=".72"/>
      <text id="hazardLabel" y="480" class="muted">debris hazard</text>
      <rect id="pad" y="504" width="62" height="12" fill="var(--safe)"/>
      <text id="padLabel" y="540" class="muted">selected safe target</text>
      <path id="trueTrail" class="truthTrail" d=""/>
      <path id="estTrail" class="estimateTrail" d=""/>
      <circle id="estimate" class="estimate" r="6"/>
      <g id="vehicle">
        <rect class="vehicle" x="-11" y="-31" width="22" height="62" rx="3"/>
        <polygon class="vehicle" points="-11,-31 0,-49 11,-31"/>
        <line id="thrustLine" x1="0" y1="34" x2="0" y2="76" class="thrust"/>
      </g>
      <line x1="748" y1="66" x2="774" y2="66" stroke="var(--truth)" stroke-width="3"/><text x="783" y="71" class="muted">true trajectory</text>
      <line x1="748" y1="91" x2="774" y2="91" stroke="var(--estimate)" stroke-width="2" stroke-dasharray="7 5"/><text x="783" y="96" class="muted">navigation estimate</text>
    </svg>
    <section class="live-physics" aria-live="polite">
      <b id="phaseTitle">Divert acquisition</b>
      <p id="phaseText">The guidance law tilts the thrust vector early, while time-to-go is large, to build the lateral impulse required by the new target.</p>
    </section>
    <div class="physics-grid">
      <section>
        <h2>Why the path is S-shaped</h2>
        <p>The first bend creates lateral impulse through a_x = T sin(theta)/m. The second reverses lateral acceleration so horizontal velocity is arrested before touchdown rather than carried through the target.</p>
      </section>
      <section>
        <h2>Why tilt is reduced late</h2>
        <p>Tilting spends the shared thrust vector on crossrange control and reduces vertical authority to T cos(theta). The terminal corridor therefore tightens attitude demand as vertical kinetic-energy removal becomes dominant.</p>
      </section>
      <section>
        <h2>Why purple is jagged</h2>
        <p>The dashed line is an estimated state, not a second trajectory. Discrete noisy measurements produce innovation corrections, while the blue truth state evolves continuously under the integrated rigid-body equations.</p>
      </section>
    </div>
    <p class="explanation"><strong>Verified outcome.</strong> The selected target is x = {metrics['target_x_m']:.1f} m, outside the [-4, 4] m debris interval. The vehicle lands at x = {metrics['landing_x_m']:.2f} m with target error {metrics['landing_x_error_m']:.2f} m, touchdown speed {metrics['touchdown_speed_mps']:.2f} m/s, hazard clearance {metrics['hazard_clearance_m']:.2f} m, and {metrics['propellant_remaining_kg']:.0f} kg of modeled propellant remaining.</p>
  </main>
  <script>
    const rows={payload};
    const hazard={{left:{hazard['left_m']},right:{hazard['right_m']}}};
    const target=rows[0].target_x_m;
    const xmin=Math.min(...rows.flatMap(r=>[r.x_m,r.estimated_x_m]),hazard.left,target)-4;
    const xmax=Math.max(...rows.flatMap(r=>[r.x_m,r.estimated_x_m]),hazard.right,target)+4;
    const zmax=Math.max(...rows.map(r=>r.z_m),1);
    const sx=x=>70+(x-xmin)/(xmax-xmin)*900;
    const sy=z=>510-z/zmax*440;
    const vehicle=document.getElementById('vehicle'), estimate=document.getElementById('estimate');
    const trueTrail=document.getElementById('trueTrail'), estTrail=document.getElementById('estTrail');
    const slider=document.getElementById('slider'), play=document.getElementById('play'), speed=document.getElementById('speed');
    const alt=document.getElementById('alt'), err=document.getElementById('err'), vel=document.getElementById('vel'), thr=document.getElementById('thr'), nav=document.getElementById('nav');
    const phaseTitle=document.getElementById('phaseTitle'), phaseText=document.getElementById('phaseText');
    const hazardRect=document.getElementById('hazard'), hazardLabel=document.getElementById('hazardLabel'), pad=document.getElementById('pad'), padLabel=document.getElementById('padLabel');
    hazardRect.setAttribute('x',sx(hazard.left)); hazardRect.setAttribute('width',sx(hazard.right)-sx(hazard.left)); hazardLabel.setAttribute('x',sx(hazard.left));
    pad.setAttribute('x',sx(target)-31); padLabel.setAttribute('x',sx(target)-58);
    let playing=true,idx=0,last=performance.now();
    function path(keyX,keyZ,end){{return rows.slice(0,end+1).map((r,i)=>`${{i?'L':'M'}}${{sx(r[keyX]).toFixed(1)}},${{sy(r[keyZ]).toFixed(1)}}`).join(' ')}}
    function phase(r){{
      const ratio=r.z_m/Math.max(zmax,1);
      if(ratio>0.68)return['Divert acquisition','Early tilt builds lateral impulse while time-to-go is large, avoiding an authority-intensive correction near touchdown.'];
      if(ratio>0.25)return['Crossrange braking','The lateral acceleration changes sign. This counter-tilt removes horizontal velocity so the vehicle can approach the selected target without overshoot.'];
      if(ratio>0.06)return['Terminal alignment','The allowable tilt contracts, restoring vertical thrust projection T cos(theta) for descent-energy removal while residual target error is closed.'];
      return['Touchdown shaping','Finite throttle and gimbal dynamics now limit correction rate; the controller regulates vertical speed and drives lateral velocity toward zero.'];
    }}
    function frame(i){{
      idx=Math.max(0,Math.min(rows.length-1,i)); const r=rows[idx];
      vehicle.setAttribute('transform',`translate(${{sx(r.x_m).toFixed(1)}} ${{sy(r.z_m).toFixed(1)}}) rotate(${{r.theta_deg.toFixed(2)}})`);
      estimate.setAttribute('cx',sx(r.estimated_x_m)); estimate.setAttribute('cy',sy(r.estimated_z_m));
      trueTrail.setAttribute('d',path('x_m','z_m',idx)); estTrail.setAttribute('d',path('estimated_x_m','estimated_z_m',idx));
      alt.textContent=`${{r.z_m.toFixed(1)}} m`; err.textContent=`${{r.target_error_m.toFixed(2)}} m`;
      vel.textContent=`${{Math.hypot(r.vx_mps,r.vz_mps).toFixed(2)}} m/s`; thr.textContent=`${{Math.round(r.throttle*100)}}%`;
      nav.textContent=`${{Math.hypot(r.estimated_x_m-r.x_m,r.estimated_z_m-r.z_m).toFixed(2)}} m`;
      const currentPhase=phase(r); phaseTitle.textContent=currentPhase[0]; phaseText.textContent=currentPhase[1];
      slider.value=Math.round(idx/(rows.length-1)*1000);
    }}
    function tick(now){{const dt=now-last;last=now;if(playing&&dt>0){{frame((idx+Math.max(1,Math.round(dt/55*Number(speed.value))))%rows.length)}}requestAnimationFrame(tick)}}
    play.addEventListener('click',()=>{{playing=!playing;play.textContent=playing?'Pause':'Play';last=performance.now()}});
    slider.addEventListener('input',()=>{{playing=false;play.textContent='Play';frame(Math.round(Number(slider.value)/1000*(rows.length-1)))}});
    frame(0);requestAnimationFrame(tick);
  </script>
</body>
</html>
"""
    output = Path("media/hazard_divert_landing_animation.html")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html)
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
