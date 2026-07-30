#!/usr/bin/env python3
"""Generate an interactive 3D 6-DOF landing animation."""

import csv
import json
from pathlib import Path


CASE_ORDER = [
    "nominal",
    "crosswind",
    "high_crosswind",
    "engine_out",
]
KEYS = [
    "time_s",
    "x_m",
    "y_m",
    "z_m",
    "vx_mps",
    "vy_mps",
    "vz_mps",
    "qw",
    "qx",
    "qy",
    "qz",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "tilt_deg",
    "quaternion_norm",
    "horizontal_error_m",
    "propellant_remaining_kg",
    "allocation_residual",
    "actuator_tracking_residual",
    "total_thrust_n",
    "engine_1_throttle",
    "engine_2_throttle",
    "engine_3_throttle",
    "engine_4_throttle",
    "engine_1_gimbal_x_deg",
    "engine_1_gimbal_y_deg",
    "engine_2_gimbal_x_deg",
    "engine_2_gimbal_y_deg",
    "engine_3_gimbal_x_deg",
    "engine_3_gimbal_y_deg",
    "engine_4_gimbal_x_deg",
    "engine_4_gimbal_y_deg",
]


def load_rows(name):
    with Path(f"outputs/sixdof_{name}.csv").open() as stream:
        raw = list(csv.DictReader(stream))
    stride = max(1, len(raw) // 320)
    sampled = raw[::stride]
    if sampled[-1] != raw[-1]:
        sampled.append(raw[-1])
    return [
        {
            key: round(float(row[key]), 6)
            for key in KEYS
        }
        for row in sampled
    ]


def main():
    summary = json.loads(
        Path("outputs/sixdof_verification_summary.json").read_text()
    )
    histories = {
        name: load_rows(name)
        for name in CASE_ORDER
    }
    payload = json.dumps(
        {
            "histories": histories,
            "summary": summary,
        },
        separators=(",", ":"),
    )
    html = build_html(payload)
    output = Path("media/sixdof_landing_animation.html")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html)
    print(f"Wrote {output}")


def build_html(payload):
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>3D 6-DOF Powered Landing</title>
  <style>
    :root {{
      color-scheme: light dark;
      --bg:#f8fafc; --panel:#ffffff; --fg:#0f172a; --muted:#475569;
      --border:#cbd5e1; --blue:#2563eb; --green:#059669;
      --orange:#ea580c; --red:#dc2626; --violet:#7c3aed;
    }}
    @media (prefers-color-scheme:dark) {{
      :root {{ --bg:#0b1120; --panel:#111827; --fg:#f8fafc;
        --muted:#cbd5e1; --border:#334155; }}
    }}
    * {{ box-sizing:border-box; }}
    body {{ margin:0; background:var(--bg); color:var(--fg);
      font-family:Arial,Helvetica,sans-serif; }}
    main {{ width:min(1220px,100%); margin:0 auto; padding:28px; }}
    h1 {{ margin:0; font-size:clamp(28px,5vw,46px); letter-spacing:0; }}
    .subtitle {{ color:var(--muted); line-height:1.5; margin:8px 0 18px; }}
    .controls {{ display:grid; grid-template-columns:auto minmax(180px,1fr) auto auto;
      gap:12px; align-items:center; margin-bottom:14px; }}
    button,select {{ min-height:42px; border:1px solid var(--border);
      border-radius:6px; background:var(--panel); color:var(--fg);
      padding:0 13px; font:inherit; font-weight:700; }}
    input {{ width:100%; accent-color:var(--blue); }}
    label {{ color:var(--muted); font-weight:700; display:flex;
      gap:8px; align-items:center; white-space:nowrap; }}
    .status {{ display:grid; grid-template-columns:repeat(6,minmax(0,1fr));
      gap:8px; margin:12px 0; }}
    .stat {{ border-top:2px solid var(--border); padding:9px 3px 4px; }}
    .stat span {{ display:block; color:var(--muted); font-size:11px;
      font-weight:700; text-transform:uppercase; }}
    .stat b {{ display:block; font-size:19px; margin-top:4px; }}
    .stage {{ position:relative; border:1px solid var(--border);
      border-radius:6px; overflow:hidden; background:var(--panel); }}
    canvas {{ width:100%; height:auto; display:block; aspect-ratio:11/6; }}
    .legend {{ display:flex; flex-wrap:wrap; gap:16px; padding:10px 14px;
      border-top:1px solid var(--border); color:var(--muted);
      font-size:12px; font-weight:700; }}
    .swatch {{ display:inline-block; width:22px; height:3px;
      vertical-align:middle; margin-right:6px; }}
    .telemetry {{ display:grid; grid-template-columns:1.35fr 1fr;
      gap:14px; margin-top:14px; }}
    .phase,.engines {{ border:1px solid var(--border); border-radius:6px;
      background:var(--panel); padding:16px; }}
    .phase h2,.engines h2,.physics h2 {{ margin:0 0 8px; font-size:17px; }}
    .phase p,.physics p {{ margin:0; color:var(--muted); line-height:1.55; }}
    .engine-row {{ display:grid; grid-template-columns:30px 1fr 64px;
      gap:9px; align-items:center; margin:8px 0; font-size:12px;
      font-weight:700; }}
    .bar {{ height:10px; background:color-mix(in srgb,var(--border) 75%,transparent);
      position:relative; }}
    .bar i {{ display:block; height:100%; background:var(--violet); }}
    .physics {{ display:grid; grid-template-columns:repeat(3,minmax(0,1fr));
      gap:14px; margin-top:14px; }}
    .physics section {{ border-top:2px solid var(--border); padding-top:12px; }}
    .result {{ margin-top:16px; padding:14px 0; border-top:1px solid var(--border);
      color:var(--muted); line-height:1.55; }}
    .pass {{ color:var(--green); }} .fail {{ color:var(--red); }}
    @media (max-width:820px) {{
      main {{ padding:16px; }}
      .controls {{ grid-template-columns:auto 1fr; }}
      .controls label {{ grid-column:1/-1; }}
      .status {{ grid-template-columns:repeat(2,minmax(0,1fr)); }}
      .telemetry,.physics {{ grid-template-columns:1fr; }}
    }}
  </style>
</head>
<body>
<main>
  <h1>3D 6-DOF Powered Landing</h1>
  <p class="subtitle">A synchronized view of inertial translation, quaternion attitude,
  mass depletion, crosswind aerodynamics, four-engine thrust allocation, and finite-bandwidth actuator response.</p>
  <div class="controls">
    <button id="play" type="button">Pause</button>
    <input id="time" aria-label="simulation time" type="range" min="0" max="1000" value="0">
    <label for="case">case
      <select id="case">
        <option value="nominal">calm nominal</option>
        <option value="crosswind">12/-6 m/s crosswind</option>
        <option value="high_crosswind">18/-10 m/s crosswind</option>
        <option value="engine_out">engine 1 out at 12 s</option>
      </select>
    </label>
    <label for="speed">speed
      <select id="speed"><option value="0.5">0.5x</option>
        <option value="1" selected>1x</option><option value="2">2x</option></select>
    </label>
  </div>
  <section class="status" aria-label="live state">
    <div class="stat"><span>time</span><b id="s-time">0 s</b></div>
    <div class="stat"><span>altitude</span><b id="s-alt">0 m</b></div>
    <div class="stat"><span>target error</span><b id="s-error">0 m</b></div>
    <div class="stat"><span>vertical speed</span><b id="s-vz">0 m/s</b></div>
    <div class="stat"><span>body tilt</span><b id="s-tilt">0 deg</b></div>
    <div class="stat"><span>propellant</span><b id="s-prop">0 kg</b></div>
  </section>
  <section class="stage">
    <canvas id="view" width="1100" height="600"
      aria-label="Projected 3D powered-landing trajectory and vehicle attitude"></canvas>
    <div class="legend">
      <span><i class="swatch" style="background:var(--blue)"></i>traversed trajectory</span>
      <span><i class="swatch" style="background:var(--border)"></i>complete trajectory</span>
      <span><i class="swatch" style="background:var(--red)"></i>body x</span>
      <span><i class="swatch" style="background:var(--green)"></i>body y</span>
      <span><i class="swatch" style="background:var(--blue)"></i>body z / thrust axis</span>
      <span><i class="swatch" style="background:var(--orange)"></i>exhaust direction</span>
    </div>
  </section>
  <div class="telemetry">
    <section class="phase">
      <h2 id="phase-title">High-altitude braking</h2>
      <p id="phase-text"></p>
    </section>
    <section class="engines">
      <h2>Engine allocation</h2>
      <div id="engine-bars"></div>
      <p style="color:var(--muted);font-size:12px;margin:8px 0 0">
        <span id="residuals"></span>
      </p>
    </section>
  </div>
  <div class="physics">
    <section>
      <h2>Rigid-body coupling</h2>
      <p>The translational plant uses m r_ddot = R_BI(F_T+F_A)+mg.
      The attitude that redirects thrust therefore changes inertial acceleration;
      position and attitude are coupled through the rotation matrix.</p>
    </section>
    <section>
      <h2>Variable-mass rotation</h2>
      <p>The rotational plant retains I omega_dot + I_dot omega +
      omega x (I omega) = tau. Propellant depletion schedules both inertia and
      the engine-to-CM moment arm, so control effectiveness changes during flight.</p>
    </section>
    <section>
      <h2>Allocation and limits</h2>
      <p>The allocator solves a weighted six-axis wrench fit. Axial-thrust imbalance
      and lateral gimbal force supply body-x/y moments; differential tangential
      gimbal force supplies body-z roll moment. Projection enforces engine limits.</p>
    </section>
  </div>
  <p class="result" id="result"></p>
</main>
<script>
const data={payload};
const colors={{nominal:"#2563eb",crosswind:"#059669",
  high_crosswind:"#ea580c",engine_out:"#dc2626"}};
const view=document.getElementById("view"),ctx=view.getContext("2d");
const play=document.getElementById("play"),slider=document.getElementById("time");
const selector=document.getElementById("case"),speed=document.getElementById("speed");
let active="nominal",index=0,playing=true,last=performance.now(),playbackTime=0;

function css(name){{return getComputedStyle(document.documentElement).getPropertyValue(name).trim();}}
function project(x,y,z){{return [550+8.0*x-5.0*y,535-0.60*z+1.15*x+0.70*y];}}
function matrix(r){{
  const w=r.qw,x=r.qx,y=r.qy,z=r.qz;
  return [[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
    [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
    [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]];
}}
function axisVector(R,column,scale){{
  return [R[0][column]*scale,R[1][column]*scale,R[2][column]*scale];
}}
function drawLine(a,b,color,width=2,dash=[]){{
  ctx.save();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);
  ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();ctx.restore();
}}
function trajectory(rows,end,color,width){{
  ctx.save();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.lineJoin="round";
  ctx.beginPath();
  rows.slice(0,end+1).forEach((r,i)=>{{
    const p=project(r.x_m,r.y_m,r.z_m);
    if(i)ctx.lineTo(...p);else ctx.moveTo(...p);
  }});ctx.stroke();ctx.restore();
}}
function drawGrid(){{
  ctx.clearRect(0,0,view.width,view.height);
  ctx.fillStyle=css("--panel");ctx.fillRect(0,0,view.width,view.height);
  for(let value=-20;value<=45;value+=5){{
    drawLine(project(value,-20,0),project(value,45,0),css("--border"),1);
    drawLine(project(-20,value,0),project(45,value,0),css("--border"),1);
  }}
  drawLine(project(-20,0,0),project(45,0,0),css("--muted"),1.5);
  drawLine(project(0,-20,0),project(0,45,0),css("--muted"),1.5);
  const target=project(0,0,0);
  ctx.strokeStyle=css("--fg");ctx.lineWidth=2;ctx.beginPath();
  ctx.arc(target[0],target[1],11,0,Math.PI*2);ctx.stroke();
  ctx.fillStyle=css("--fg");ctx.font="700 12px Arial";ctx.fillText("target",target[0]+15,target[1]+4);
  ctx.fillStyle=css("--muted");ctx.font="12px Arial";
  ctx.fillText("inertial x/y landing plane",54,566);
}}
function drawWind(name){{
  const wind=data.summary.cases[name].wind_inertial_mps;
  const magnitude=Math.hypot(wind[0],wind[1]);
  if(magnitude<0.1)return;
  const start=project(-12,30,0),scale=1.6;
  const end=project(-12+wind[0]*scale,30+wind[1]*scale,0);
  drawLine(start,end,css("--violet"),4);
  const angle=Math.atan2(end[1]-start[1],end[0]-start[0]);
  ctx.fillStyle=css("--violet");ctx.beginPath();ctx.moveTo(...end);
  ctx.lineTo(end[0]-12*Math.cos(angle-.45),end[1]-12*Math.sin(angle-.45));
  ctx.lineTo(end[0]-12*Math.cos(angle+.45),end[1]-12*Math.sin(angle+.45));
  ctx.closePath();ctx.fill();
  ctx.font="700 12px Arial";ctx.fillText(`wind ${{magnitude.toFixed(1)}} m/s`,start[0]-5,start[1]-12);
}}
function drawVehicle(r){{
  const origin=project(r.x_m,r.y_m,r.z_m),R=matrix(r);
  const axes=[axisVector(R,0,5),axisVector(R,1,5),axisVector(R,2,8)];
  const axisColors=[css("--red"),css("--green"),css("--blue")];
  axes.forEach((v,i)=>{{
    const tip=project(r.x_m+v[0],r.y_m+v[1],r.z_m+v[2]);
    drawLine(origin,tip,axisColors[i],i===2?4:3);
  }});
  const body=axisVector(R,2,13);
  const tail=project(r.x_m-body[0]*.45,r.y_m-body[1]*.45,r.z_m-body[2]*.45);
  const nose=project(r.x_m+body[0]*.55,r.y_m+body[1]*.55,r.z_m+body[2]*.55);
  drawLine(tail,nose,css("--fg"),10);
  ctx.fillStyle=colors[active];ctx.beginPath();ctx.arc(nose[0],nose[1],6,0,Math.PI*2);ctx.fill();
  const thrust=Math.max(r.engine_1_throttle,r.engine_2_throttle,
    r.engine_3_throttle,r.engine_4_throttle);
  const plume=axisVector(R,2,-11*thrust);
  const plumeEnd=project(r.x_m-body[0]*.45+plume[0],
    r.y_m-body[1]*.45+plume[1],r.z_m-body[2]*.45+plume[2]);
  drawLine(tail,plumeEnd,css("--orange"),5);
}}
function phase(r,name){{
  if(name==="engine_out" && r.time_s>=12){{
    return ["Engine-out reallocation",
      "Engine 1 thrust drops to zero. The remaining three engines must satisfy force and all three moment demands inside a smaller, asymmetric wrench set. Rising allocator residual and differential throttle expose the lost control authority."];
  }}
  if(r.z_m>90)return ["High-altitude energy removal",
    "The square-root braking reference follows |v_z,ref| proportional to sqrt(2 a_b z), an energy-derived schedule. Large time-to-go permits horizontal impulse while thrust margin removes vertical kinetic energy."];
  if(r.z_m>25)return ["Approach gate",
    "Descent is limited to 4.5 m/s. The horizontal corridor contracts with altitude, forcing accumulated crossrange error to be removed before tilt would materially reduce the vertical projection T cos(theta)."];
  if(r.z_m>6)return ["Flare",
    "The 2.2 m/s flare reference reduces vertical kinetic energy before contact. Attitude demand is kept small because lateral thrust and vertical braking compete for the same bounded thrust vector."];
  return ["Terminal descent",
    "The final 1.2 m/s reference trades touchdown load against gravity loss. Quaternion feedback holds the thrust axis near vertical while differential engine commands null residual body rates."];
}}
function render(){{
  const rows=data.histories[active],r=rows[index];
  drawGrid();drawWind(active);
  trajectory(rows,rows.length-1,css("--border"),2);
  trajectory(rows,index,colors[active],4);
  drawVehicle(r);
  const [title,body]=phase(r,active);
  document.getElementById("phase-title").textContent=title;
  document.getElementById("phase-text").textContent=body;
  document.getElementById("s-time").textContent=`${{r.time_s.toFixed(1)}} s`;
  document.getElementById("s-alt").textContent=`${{r.z_m.toFixed(1)}} m`;
  document.getElementById("s-error").textContent=`${{r.horizontal_error_m.toFixed(2)}} m`;
  document.getElementById("s-vz").textContent=`${{r.vz_mps.toFixed(2)}} m/s`;
  document.getElementById("s-tilt").textContent=`${{r.tilt_deg.toFixed(2)}} deg`;
  document.getElementById("s-prop").textContent=`${{r.propellant_remaining_kg.toFixed(0)}} kg`;
  document.getElementById("engine-bars").innerHTML=[1,2,3,4].map(i=>{{
    const throttle=r[`engine_${{i}}_throttle`],gx=r[`engine_${{i}}_gimbal_x_deg`],
      gy=r[`engine_${{i}}_gimbal_y_deg`],g=Math.hypot(gx,gy);
    return `<div class="engine-row"><span>E${{i}}</span><span class="bar"><i style="width:${{100*throttle}}%"></i></span><span>${{(100*throttle).toFixed(0)}}% / ${{g.toFixed(1)}} deg</span></div>`;
  }}).join("");
  document.getElementById("residuals").textContent=
    `allocation residual ${{r.allocation_residual.toFixed(3)}} | actuator-path residual ${{r.actuator_tracking_residual.toFixed(3)}} | |q|-1 = ${{Math.abs(r.quaternion_norm-1).toExponential(1)}}`;
  const m=data.summary.cases[active].metrics,passed=m.success;
  document.getElementById("result").innerHTML=
    `<strong class="${{passed?"pass":"fail"}}">${{passed?"PASS":"FAIL"}}</strong> under the stated model-level criteria. `+
    `Touchdown horizontal error is ${{m.horizontal_target_error_m.toFixed(2)}} m, vertical speed is ${{m.touchdown_vertical_speed_mps.toFixed(2)}} m/s, `+
    `maximum tilt is ${{m.maximum_tilt_deg.toFixed(2)}} deg, and ${{m.propellant_remaining_kg.toFixed(0)}} kg propellant remains. `+
    `${{passed?"The closed-loop trajectory remains inside the tested terminal corridor.":active==="high_crosswind"?"The vehicle remains attitude-bounded, but the scheduled PD law has no integral or wind-feedforward channel; steady aerodynamic load therefore leaves terminal position bias.":"The failed engine makes the attainable wrench set asymmetric; positive propellant cannot replace the lost force/moment directions over the remaining time-to-go."}}`;
  slider.value=Math.round(1000*index/(rows.length-1));
}}
function tick(now){{
  if(playing){{
    const rows=data.histories[active];
    playbackTime+=(now-last)/1000*Number(speed.value);
    while(index+1<rows.length && rows[index+1].time_s<=playbackTime)index++;
    if(playbackTime>rows[rows.length-1].time_s){{playbackTime=0;index=0;}}
    render();
  }}
  last=now;
  requestAnimationFrame(tick);
}}
play.addEventListener("click",()=>{{playing=!playing;play.textContent=playing?"Pause":"Play";}});
slider.addEventListener("input",()=>{{const rows=data.histories[active];
  index=Math.round(Number(slider.value)/1000*(rows.length-1));
  playbackTime=rows[index].time_s;render();}});
selector.addEventListener("change",()=>{{active=selector.value;index=0;playbackTime=0;render();}});
render();requestAnimationFrame(tick);
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()
