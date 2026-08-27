# Code change record

This file records changes made by Codex during development. It should be
updated whenever Codex modifies the project.

## 2026-08-19

### Energy-endurance configuration

- Changed `run_periodic_analysis.m` from `fuel_rate` to `energy` for both
  the OCP and the frequency-domain PI test.
- Enabled the endurance optimization and disabled the range optimization.
- Replaced the malformed fixed-period vector with the valid exploratory set
  `[7.24, 10, 20, 30, 40, 50, 60]` seconds.

### Stall-aware angle-of-attack limit

- Added the user setting `STALL_MARGIN = 0.96` to
  `run_periodic_analysis.m`.
- For airframes that define `CL_max`, the driver now computes

  `alpha_at_CLmax = (CL_max - CL0) / CLa`

  and limits the OCP angle of attack to

  `min(alpha_max, STALL_MARGIN * alpha_at_CLmax)`.

- For the current Stallion parameters, the nominal `CL_max` angle is about
  6.77 degrees and the 0.96 margin gives an effective limit of about
  6.50 degrees.
- The driver prints the effective stall-aware angle limit when it starts.

### Reason for the constraint

The aerodynamic model uses a linear lift curve and has no post-stall model.
Without this constraint, the optimizer can request lift beyond the configured
`CL_max`, making aggressive periodic trajectories physically inconsistent.

### Load-factor constraint and 20-degree test configuration

- Added optional `load_factor_min` and `load_factor_max` path constraints to
  `ocp_casadi_fixed_T.m`.
- Load factor is calculated at every Radau collocation point as
  `n = L/(m*g)` and constrained there, rather than only checked afterward at
  mesh nodes.
- Added `sol.load_factor` to the returned solution for diagnostics.
- Added `LOAD_FACTOR_BOUNDS = [0.5, 2.0]` to
  `run_periodic_analysis.m`. These are provisional research bounds and are
  not claimed to be validated Stallion structural limits.
- Changed the Stallion flight-path-angle envelope from plus/minus 10 degrees
  to plus/minus 20 degrees for the requested aggressive-path test.
- Changed the sine-periodic warm-start amplitude from 0.10 to 0.50 so the
  driver can reach the tested nontrivial 20-degree solution branch more
  reliably.

## 2026-08-20

### Generic 6S 10 Ah battery model

- Added `battery_params.m` with a clearly labeled generic 6S 10 Ah LiPo
  parameter set: 80% initial SOC, 20% minimum SOC, 0.030-ohm assumed pack
  resistance, provisional 100 A current limit, and 3.30 V/cell cutoff.
- Added `battery_power_model.m`, a quasi-static Thevenin/Rint model with an
  SOC-dependent open-circuit-voltage curve, terminal-voltage sag, current,
  chemical power, and ohmic loss.
- Added `USE_BATTERY_MODEL = true` to the analysis driver and attached the
  generic pack parameters to the aircraft structure.
- Updated `steady_cruise.m` so an energy baseline includes battery internal
  resistance losses when the battery model is enabled.
- Augmented the energy OCP state with battery SOC. SOC begins at the configured
  initial value and is depleted by integrated pack current; it is deliberately
  not included in the periodic flight-state boundary conditions.
- Changed the energy running cost to battery chemical power `Voc*I` and added
  pack-current and terminal-voltage-cutoff constraints at every collocation
  point.
- Added solution diagnostics for SOC use, battery current, terminal voltage,
  and internal-resistance power loss.

### Smoother climb configuration

- Increased the Stallion flight-path-angle envelope from 20 degrees to the
  requested 40-degree test envelope.
- Reduced the driver battery-current limit from the generic pack default of
  100 A to 15 A.
- Reduced the angle-of-attack slew limit from 60 deg/s to 15 deg/s.
- Reduced the thrust slew limit from 500 N/s to 5 N/s.
- In the battery-aware 50-second test with a 40-degree allowed flight-path
  envelope, the smoothed solution used a maximum current of about 15.01 A,
  reached a 31.34-degree climb angle, and saved 0.83345% relative to steady
  battery power. Average modeled battery resistance loss was 0.917 W.
- More restrictive tests produced 0.66486% at 15 A and 2 N/s, 0.65357% at
  12 A and 5 N/s, 0.61667% at 12 A and 2 N/s, and 0.50530% at 10 A and
  2 N/s.

## 2026-08-25

### Return to constant-efficiency energy model

- Set `USE_BATTERY_MODEL = false` in `run_periodic_analysis.m` at the user's
  request.
- The active energy model is again the original constant-efficiency relation
  `P_elec = T*V/eta_total`; battery SOC, voltage sag, current limits, and
  internal-resistance losses are not active in the analysis.
- The battery-model files and optional OCP support remain in the project so
  the model can be re-enabled later without reconstructing it.
- Restored the pre-battery-experiment actuator-rate settings of 60 deg/s for
  angle of attack and 500 N/s for thrust. The 40-degree flight-path envelope,
  stall-aware angle limit, and load-factor bounds remain active.

### Additional provisional safety constraints

- Added absolute altitude bounds of 30 to 250 m, preventing the previous
  excursion-only bound from permitting negative altitude.
- Added vertical-speed bounds of -5 to 8 m/s for sink and climb rate.
- Added a flight-path-angle-rate limit of 15 deg/s, enforced from the model's
  `gamma_dot` at every collocation point.
- Added a 900 W electrical propulsion-power ceiling, enforced as
  `T*V/eta_total` at every collocation point.
- All four limits are exposed as driver settings and explicitly labeled as
  provisional research assumptions pending airframe-specific data.
- A 50-second, 40-degree-envelope validation solve succeeded with 2.44651%
  ideal-model energy saving. It used a vertical-speed range of about -2.16 to
  8.00 m/s, load factor 0.582 to 1.320, altitude 113.5 to 165.2 m, maximum
  electrical propulsion power 446.2 W, and maximum angle of attack 6.5005
  degrees. The climb-rate constraint was active; altitude and power limits
  were inactive.

## 2026-08-26

### Solver entry-point guidance

- Added an explicit input check to `ocp_casadi_fixed_T.m`. Running the solver
  function without its required arguments now directs the user to run
  `run_periodic_analysis` instead of failing later while accessing `opts`.

## 2026-08-27

### Period extension beyond 3% saving

- Extended the fixed-period sweep with 70 and 80 seconds; the active list is
  now `[7.24, 10, 20, 30, 40, 50, 60, 70, 80]` seconds.
- The safety-constrained 80-second solution produced 3.07616% ideal-model
  energy saving with 533 intervals and 3.09378% after continuation refinement
  to 800 intervals.
- The refined solution retained all active constraints: load factor 0.585 to
  1.315, vertical speed about -2.13 to 8.00 m/s, speed 12.30 to 15.75 m/s,
  maximum flight-path angle 39.61 degrees, maximum angle of attack 6.5005
  degrees, and maximum electrical propulsion power about 448.7 W.
- No stall, load-factor, altitude, vertical-speed, flight-path-rate, or power
  constraint was relaxed to cross the 3% target.
