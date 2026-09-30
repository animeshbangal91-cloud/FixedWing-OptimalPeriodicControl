# Steady versus periodic flight

This workflow uses the same Stallion model and controller tuning for both flights.
It compares electrical propulsion-energy estimates derived from **actual Gazebo
rotor speeds and airspeed**, not from the imported reference thrust or PX4's
timed battery percentage. It does not yet predict battery SOC, voltage sag or
capacity loss. The physical aircraft and existing battery configuration are unchanged.

## Prepare the supplied one-period trajectory

Run in WSL after the user provides the final CSV:

```bash
~/venvs/px4/bin/python ~/stallion_experiment/prepare_missions.py /path/to/period.csv \
  --out ~/stallion_experiment/missions/tau235 --periods 5 --average-speed 13.67
```

Required CSV columns: `time_s,X_m,Z_m,gamma_deg,V_mps`. MATLAB's additional
angle-of-attack, thrust, power and energy columns are retained but are not used
as measured flight data. Time and X must increase; altitude, speed and flight-path
angle must close at the period boundary. Kinematic consistency is checked.

The builder preserves the waveform and its exact average forward speed. For
example, 13.67375 m/s is accepted as a rounded 13.67 request, and **both missions
then use 13.67375**, rather than silently retiming the optimized trajectory.
Differences greater than 0.01 m/s are rejected. Outputs include the original CSV,
its SHA-256 and separate `periodic.json` / `steady.json` mission descriptions.

Five 235-second periods give a **1,175-second scored segment**. By default one
additional unscored period establishes the motion after the initial level-flight
entry. Thus the reference runs for 1,410 seconds plus takeoff/stabilization.
Use `--warmup-periods 0` only when intentionally including the entry transient.
The steady flight receives the same warmup time, scored duration and forward
distance. Its level altitude matches the periodic waveform's initial altitude.

X advances by the period distance at every repetition; altitude and speed repeat.
Cubic Hermite interpolation uses the exported position and velocity for smooth
position/velocity transitions. This is a straight longitudinal mission, not an orbit.

## Fly

Rebuild the diagnostic plugin once with the simulator stopped:

```bash
bash ~/stallion_experiment/setup.sh
bash ~/stallion_experiment/launch.sh --mission ~/stallion_experiment/missions/tau235/steady.json
```

Wait for READY, then enter `commander mode offboard` and `commander arm` at
`pxh>` in tmux window 0. The launcher still requires manual arming. Run the
periodic JSON in a fresh simulation after stopping the steady simulation.
Both missions save plots and enter Hold after completion; Hold keeps flying.

For an explicitly requested automatic local flight test, the existing runner
starts, arms and stops its own simulator:

```bash
~/venvs/px4/bin/python ~/stallion_experiment/run_trial.py \
  ~/stallion_experiment/tuning.json \
  --mission ~/stallion_experiment/missions/tau235/steady.json
```

## Controller interface

The installed PX4 revision clears `cruising_speed` when processing each Offboard
position setpoint. The experiment therefore updates `FW_AIRSPD_TRIM` through
MAVLink at 2 Hz, logs parameter echoes, and requires acknowledgements. This is
a simulator-specific fallback, not a claim that streamed horizontal velocity
automatically controls fixed-wing airspeed. The reference itself remains driven
by simulation time; it never follows the aircraft's actual phase.

A bounded correction of 0.06 times along-track position error (maximum 1.5 m/s)
is added to the speed request to correct forward progress. Applied airspeed
requests remain inside the model's PX4 airspeed limits. Both the unmodified
reference speed and the corrected request are logged. This is needed because
the inherited simulated pitot signal and true Gazebo airspeed need not agree.

PX4 receives the reference altitude and horizontal tangent. Its attitude/throttle
controllers remain responsible for tracking. CSV thrust is not forced onto the
motors. Achieving the MATLAB energy saving depends on actual tracking and on the
additional dynamics in Gazebo; matching geometry alone is not sufficient.

## Energy calculation and results

The aerodynamic plugin publishes `/model/stallion_0/energy_diagnostics` at up to
50 Hz. Rotor joint velocities are multiplied by the configured slowdown factor
of 10, then converted to thrust using the motor model's signed squared-speed
law and its 1e-5 N/(rad/s)^2 coefficient. This is the same law used by the installed
[Gazebo 8.15 motor model](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/systems/multicopter_motor_model/MulticopterMotorModel.cc).

Two estimates are recorded:

- `power_matlab_W = (T0 + T1) * airspeed / eta`: matches MATLAB's scalar model.
- `power_axis_W = (T0 + T1) * body_forward_airspeed / eta`: accounts for thrust
  direction relative to air-relative velocity.

`eta=0.70*0.85*0.96=0.5712`. These constant-efficiency estimates omit motor idle
losses, avionics load and battery losses, just as the active MATLAB energy model
does. No charge regeneration is modeled.

Each run writes commands.csv, actual.csv, energy.csv, speed_control.csv, run.json,
`battery.csv`, the source CSV snapshot, energy_metrics.json and
energy_tracking.png/.pdf. Battery telemetry includes remaining percentage,
pack voltage/current and PX4-reported consumed capacity/energy when available.
It is explicitly labeled as PX4 SITL battery telemetry and is not substituted
for the rotor-derived energy measurement.

For prepared energy missions, `COM_LOW_BAT_ACT=0` prevents the default timed
SITL battery source from commanding Return during a long run. Battery messages
and warnings remain logged. This setting affects the automatic failsafe action,
not propulsion, battery telemetry, or the scored energy calculation.
Energy integration uses Gazebo simulation timestamps, with an estimated constant
offset to PX4 boot time. Arrival jitter and missing-sample gaps are checked.
Actual positions are PX4 estimates; actual airspeed and rotor speeds come from Gazebo.

Compare two completed runs:

```bash
~/venvs/px4/bin/python ~/stallion_experiment/analyze_energy.py \
  /path/to/steady/run /path/to/periodic/run --out ~/stallion_experiment/energy_comparison
```

The report includes energy in J/Wh, average power, per-period energy, actual
average forward speed, distance error, altitude/airspeed tracking, and endpoint
mechanical-energy change. Comparison requires matching source, period counts,
model, tuning and PX4 revision. Tracking limits are explicitly reported; failing
them marks the observed energy difference as **not a validated mission saving**.
Incomplete flights never produce a savings result. `battery_savings_pct` remains
null because no load-dependent battery-discharge model has been installed.

The synthetic 40-second waveform under `validation_missions` is only a pipeline
test, not the user's optimized trajectory and not evidence of optimized savings.
