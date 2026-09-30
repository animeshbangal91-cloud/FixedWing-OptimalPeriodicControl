# Stallion flight tuning results

The selected tuning completed the two-circle reference and confirmed Hold.
PX4 and Gazebo were stopped after the test. Physical model inputs and battery
configuration were unchanged during this tuning work.

| Circle tracking metric | Original run | Tuned run |
|---|---:|---:|
| Radial RMS error | 77.87 m | 1.50 m |
| Altitude RMS error | 1.42 m | 0.25 m |
| Time-aligned 3D position RMS error | 104.75 m | 6.40 m |
| Peak radial error | 202.09 m | 2.64 m |
| Peak time-aligned 3D position error | 256.07 m | 10.80 m |

Original: `results/20260929_201826_618522`.
Tuned: `results/20260929_202930_994019`.
See [comparison plot](tuning_comparison.png), [comparison PDF](tuning_comparison.pdf)
and [tuned flight plot](results/20260929_202930_994019/trajectory.png).

Both runs use a 120 m radius, 130 m altitude above the initial position,
13.66768 m/s reference speed and two nominal laps (110.3 seconds). Metrics
cover the circle phase, excluding takeoff and stabilization. Actual positions
are PX4 local-position estimates, not independent Gazebo ground truth. The
comparison aligns intended circle centers and entry headings; it does not fit
the flown paths. Geographic circle locations differ between runs.

## What changed

The original takeoff ended at 44.24 m/s and the circle began at 36.95 m/s.
The entry point was also captured before Offboard confirmation, leaving about
120 m between the aircraft and the first circle command. This caused the large
outward loop. The revised entry uses a 30 m runway clearance, climbs to 130 m
in Offboard and waits for stable speed/height before defining the circle.
The reference remains driven by time after entry.

The launcher now loads these settings from [tuning.json](tuning.json):

| Parameter | Value |
|---|---:|
| RWTO_MAX_THR | 1.0 |
| MIS_TAKEOFF_ALT | 30 m |
| FW_THR_TRIM | 0.36 |
| NPFG_PERIOD | 8 s |
| FW_T_ALT_TC | 3 s |
| FW_T_TAS_TC | 3 s |
| FW_T_THR_INTEG | 0.08 |
| FW_T_I_GAIN_PIT | 0.15 |

Stabilization requires height within 2 m, horizontal speed within 0.8 m/s and
vertical speed below 0.5 m/s for five seconds. The tuned circle began at
13.42 m/s and 128.97 m above the initial position. An earlier trial limiting
runway throttle to 0.6 failed to take off and was aborted; full runway throttle
is retained in the selected profile.

The improvement combines entry corrections and controller tuning; this test
does not isolate the contribution of each gain. Geometric following is much
better, but timing is not exact: peak time-aligned error remains 10.80 m.
One completed flight validates this particular calm simulation case, not
robustness across wind, payloads or other trajectories. Estimated inertia,
control derivatives and other model assumptions remain as documented in README.

## Run again

```bash
bash ~/stallion_experiment/launch.sh
```

Wait for READY, then enter `commander mode offboard` and `commander arm` in
the PX4 console. The normal launcher requires manual arming. At completion,
Hold keeps the aircraft loitering and plots are saved; it does not close Gazebo.
The separate `run_trial.py` test runner arms its own local simulation and stops
its processes after the test.

Battery behavior remains PX4's existing timed simulator. No battery model was
added or changed in this work.
