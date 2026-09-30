# Stallion-parameter simulation

For matched steady/periodic missions and actual-flight energy logging, see
[ENERGY_EXPERIMENT.md](ENERGY_EXPERIMENT.md).

This is a separate, **unvalidated generic aircraft** using the supplied scalar
parameters, schematic geometry and estimated rotational dynamics. It is not a
manufacturer-validated Stallion digital twin. The stock `rc_cessna` model and its
parameter store are unchanged. PX4 code is also unchanged.

## Launch in WSL / VS Code

```bash
bash ~/stallion_experiment/launch.sh
```

The `stallion` tmux session has PX4 in window 0, the streamer in window 1 and
the Gazebo GUI in window 2. Wait for **READY** in window 1, then press Ctrl+B,
release, press 0, and enter at the `pxh>` prompt:

```text
commander mode offboard
commander arm
```

The streamer selects runway Takeoff, then returns to Offboard and stabilizes
at 130 m above the initial position before streaming two 120 m-radius circles. The
endurance reference moves at 13.66768 m/s; the nominal circle segment is about
110.3 simulation seconds. On completion PX4 enters Hold, the aircraft keeps
loitering, and the plot and CSV files are saved under `stallion_experiment/results`.
When present, `tuning.json` supplies the tested controller/entry settings. Physical
model inputs remain in `parameters.json` and are not changed by controller tuning.

Do not use `fixedwing_demo/launch.sh` for this aircraft: that is the stock-model
launcher. Stop a running simulation before starting another one. The new
runtime is `~/stallion_sitl`; it has its own parameters and logs. The custom
plugin is built into `~/stallion_build`.

After editing `parameters.json`, stop the simulation and rebuild:

```bash
bash ~/stallion_experiment/setup.sh
```

If restarting the stream manually, retain the model profile:

```bash
~/venvs/px4/bin/python -u ~/fixedwing_demo/stream_mission.py \
  --model-config ~/stallion_sitl/model_report.json \
  --tuning-config ~/stallion_experiment/tuning.json \
  --results ~/stallion_experiment/results
```

Open the latest flown plot:

```bash
code "$(cat ~/stallion_experiment/results/latest.txt)/trajectory.png"
```

Disarmed connection checks create logs but no fabricated flight plots.

## How the supplied inputs are applied

| Input | Implementation |
|---|---|
| Name/source/60–70 km/h manufacturer speed | Preserved as user-supplied metadata; manufacturer claim not independently verified |
| `m=3 kg` | Sum of all links, including motors, wheels and sensors, is 3 kg |
| `g=9.81` | Separate world's gravity is `(0,0,-9.81)` |
| `b=1.340`, `S=0.265` | Wing collision/visual span and planform area; aero reference span/area |
| `cbar=0.211` | Pitch-moment reference chord; rectangular visual chord is S/b, not a measured planform |
| `AR=5.6`, `e=0.85` | Retained for induced drag: k=1/(pi e AR)=0.0668718; not replaced by geometric AR=6.77585 |
| `rho=1.225` | Constant density in aerodynamic forces; no altitude-density correction to this prescribed value |
| `CL0=0.45`, `CLa=5.5/rad` | CL=CL0+CLa*alpha within the linear range |
| `CL_max=1.1` | Lift-coefficient cap; assumed symmetric lower cap because negative CL limit was absent |
| `CD0=0.021`, `k` | CD=CD0+k CL^2 inside the alpha envelope |
| `alpha_min=-6 deg`, `alpha_max=10 deg` | Polar validity bounds; generic bounded lift decay and added drag outside them. They do not constrain aircraft attitude |
| `V_min=12.3`, `V_max=28` | PX4 airspeed limits; do not physically clamp velocity |
| `cruise_mode=endurance` | Ideal minimum-power speed computed from the supplied polar (13.66768 m/s) and applied as FW_AIRSPD_TRIM |
| `h_cruise=130` | Trajectory altitude above starting point; tuned runway-takeoff clearance is separately 30 m |
| `n_propellers=2` | Two separate rotor links/motor plugins and two PX4 motor outputs |
| `T_min=0`, `T_max=20 N` | Assumed total thrust: 0–10 N per motor at 0–1000 rad/s, motorConstant=1e-5. Counter-rotating |
| `nominal_battery_series=4` | PX4 BAT1_N_CELLS=4; capacity/discharge curve unavailable |
| `eta_prop=.70`, `eta_motor=.85`, `eta_esc=.96` | Product .5712 used in analytical electrical cruise-power estimate (60.94 W); not a calibrated battery-drain model |

Efficiencies are not multiplied into 20 N a second time: thrust was already
specified at the propellers. They estimate electrical power from useful
propulsive power in steady forward flight. Motor/ESC thermal losses, static
propeller efficiency, battery capacity and voltage-dependent thrust are not
identified by the supplied inputs and are not simulated from invented data.

## Explicit assumptions and inconsistencies

- **Inertia/CG:** stock generic mass distribution scaled to 3 kg and wider
  span; stock CG and gear/sensor offsets. Not measured Stallion inertia.
- **Moment/control derivatives:** explicit generic estimates in
  `model_report.json` / generated SDF, expressed in forward-right-down axes.
  These are required to make a six-degree-of-freedom aircraft respond to controls.
- **Propulsion:** 20 N total (not per motor), generic motor lag and reaction
  torque from the stock motor plugin; no measured motor/propeller map.
- **Geometry:** schematic wing/fuselage/tail and inherited landing gear;
  detailed fuselage and tail dimensions were not supplied. No manufacturer mesh.
- At 3 kg, the supplied CL_max and wing area imply a **12.84 m/s level-flight
  stall speed**, above V_min=12.3. Turning needs additional lift. Endurance
  cruise offers little margin; values were preserved rather than silently changed.
- The linear lift law reaches CL_max at **6.77 degrees**, before alpha_max=10.
  CL is capped from there, and the generic post-envelope extension begins at 10.
- The alpha bounds describe the model envelope, not a measured stall curve.

`model_report.json` records all inputs, assumptions, derived values and startup
PX4 parameters. Each custom-model run embeds a copy in its run.json so results
can be traced to the settings actually requested.

## Checks performed

- Aerodynamic plugin compiled against installed Gazebo Harmonic 8.15.0.
- Polar tests cover linear slope, lift cap, drag polar and finite post-envelope coefficients.
- Generated SDF validated with `gz sdf -k` (stock sensor-frame extension warnings only).
- Generated total mass, two motors, wing dimensions and motor constants checked.
- Disarmed PX4 launch confirmed `stallion_world` / `stallion_0`, correct motor
  count, 4S, 130 m takeoff altitude and 13.66768 m/s cruise.
- Live telemetry and offboard-stream priming passed. No flight tracking or
  tuning claims are made from a disarmed check.

The aerodynamic diagnostics topic is `/model/stallion_0/aero_diagnostics`
(`gz.msgs.Double_V`): simulation seconds, air-relative speed, alpha, beta,
CL, CD, lift N, drag N. It publishes above 0.2 m/s. Wind is subtracted from
body velocity when the world's wind-velocity component is available.

## Entry handling and controller tuning

See [TUNING_RESULTS.md](TUNING_RESULTS.md) for the completed flight comparison
and the settings now loaded automatically by the launcher.

The revised streamer waits for confirmed Offboard mode and a stable straight
segment before defining the circle's center/start point. Stable means height
within 2 m, horizontal speed within 0.8 m/s, and vertical speed below 0.5 m/s
for five seconds. The commanded circle remains time-driven after that point;
its phase is not changed to follow the aircraft. Takeoff and stabilization
telemetry remain in actual.csv, but circle metrics exclude those phases.

The tuning profile uses a 30 m runway-takeoff clearance, followed by an
Offboard climb to the original 130 m cruise altitude. Full runway power is
retained. The former 130 m clearance held runway power far too long for this
model and caused large entry overspeed. Controller gains and trim throttle
are recorded per run under `tuning` and `parameter_changes` in run.json.

Battery setup is unchanged by these tests: PX4's default timed battery
simulator supplies battery status. No Gazebo battery plugin is configured.
The power estimate in model_report.json is analytical, not battery simulation.
