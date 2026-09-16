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

### Less aggressive free-average-speed study and detailed report

- Reduced the Stallion flight-path-angle envelope from 40 degrees to 30
  degrees.
- Set `PIN_VAVG = false`, so the OCP no longer forces distance traveled to
  equal the steady-reference speed times the period.
- Confirmed `dt_target = 0.1` s for the new sweep, avoiding the previous
  4000-interval, multi-hour long-period solves.
- Expanded the best-trajectory figure from six to ten panels. The PDF/PNG now
  includes altitude, speed, flight-path angle, path, angle of attack, thrust,
  load factor, vertical speed, electrical propulsion power, and cumulative
  energy.
- Added a CSV export beside each best-trajectory PDF containing the exact time
  histories for time, X, Z, gamma, speed, angle of attack, thrust, load factor,
  vertical speed, electrical power, and cumulative energy.

## 2026-08-28

### Step 1: load- and airspeed-dependent propulsion efficiency

- Added `propulsion_map_params.m` containing a generic, smooth research map
  for propeller efficiency versus airspeed and motor efficiency versus shaft
  power. The assumed values are explicitly not measured Stallion data.
- Added `propulsion_power_model.m`, which supports both the original constant
  efficiency and the optional differentiable map used by CasADi.
- Added `USE_PROPULSION_EFFICIENCY_MAP = true` to the driver.
- Updated steady cruise, OCP running cost, propulsion-power path constraint,
  trajectory diagnostics, and report power calculation to use the same model.
- Updated `aircraft_dynamics.m` auxiliary electrical power and the PI-test
  running cost to use the selected propulsion model consistently.
- Added a separate efficiency-history CSV beside the detailed trajectory data.
- A controlled 100-second comparison at 500 intervals gave 2.2863% saving
  with constant efficiency and 16.9202% with the generic map. The mapped case
  shifted steady endurance to 14.1055 m/s and 69.2924 W, then found a periodic
  solution at 57.5679 W and 13.6778 m/s average speed.
- The generic map's total efficiency ranged from about 0.390 to 0.600. The
  large saving is therefore an efficiency-map sensitivity result, not a
  validated Stallion prediction; measured motor/propeller data are required
  before using it as a paper result.

### Step 1 update: supplied EMAX/Gemfan bench data

- Extracted the EMAX ECO II 2807 1300KV, 6S, Gemfan 7042 table from the
  user-supplied `EmaxEcoII-2807-1300KV.pdf`.
- Recorded the measured thrust, voltage, current, and electrical-power rows
  from 10% through 90% ESC command. The 100% test row is unavailable.
- Added a monotone seventh-order fit from thrust in newtons to measured
  electrical power. Its RMSE over the supplied bench points is about 2.56 W.
- Changed the active propulsion model from the earlier generic efficiency
  assumption to this `bench_poly` fit and limited maximum thrust to the last
  measured point, approximately 19.94 N.
- This is static test-stand data near 25 V. It is substantially more relevant
  than the generic map, but it does not capture propeller behavior in forward
  flight and must be described as a static-map approximation.
- A direct use of static power at forward speed implied an impossible cruise
  efficiency of about 1.071. Added a conservative combined forward-flight
  efficiency ceiling of 0.75, implemented as the smooth maximum of measured
  static-map power and `T*V/0.75`. This preserves CasADi differentiability and
  prevents the static fit from violating energy conservation.
- With that correction, a coarse 100-second/200-interval free-speed test gave
  a 46.3654 W steady baseline and 46.2905 W periodic result, or about 0.162%
  saving. The trajectory used only about 2.27 degrees maximum flight-path
  angle. This replaces the earlier 16.92% generic-map sensitivity result for
  the supplied hardware approximation.

### Step 1 update: online forward-flight surrogate

- At the user's request, stopped using the supplied EMAX static table as the
  active propulsion model.
- Downloaded the primary University of Illinois UIUC Propeller Database and
  selected its closest forward-flight surrogate: the Master Airscrew
  glass-filled two-blade 7x4 propeller at approximately 6998 RPM.
- Added the source `J`, `CT`, `CP`, and efficiency samples to
  `uiuc_magf_7x4_6998.txt` for reproducibility.
- Fit propeller efficiency as a smooth function of nondimensional loading
  `T/(rho*V^2*D^2)`. Fit RMSE is approximately 0.025 efficiency points.
- Added the `uiuc_7x4` propulsion mode and made it active in the driver.
  Motor and ESC efficiencies remain the Stallion assumptions (0.85 and 0.96).
- This is public forward-flight wind-tunnel data for a similarly sized 7x4
  propeller, not exact Gemfan 7042 data; all reports must identify it as a
  surrogate.
- A coarse 100-second/200-interval test with the UIUC surrogate converged to
  essentially steady flight: 82.7049 W steady versus 82.7047 W optimized,
  only 0.00026% nominal saving. The mapped cruise efficiency was about 0.423.
  Multi-start testing remains necessary before concluding that no periodic
  branch exists for this surrogate.

### Central optional-feature controls

- Added a `FEATURES` settings block near the top of
  `run_periodic_analysis.m`. Battery, propulsion model, propeller drag,
  multi-start, period refinement, aerodynamic sensitivity, fair-comparison
  audit, and atmosphere effects can now be selected without editing solver
  internals.
- Propulsion selection now exposes all implemented modes: `constant`,
  `generic`, `uiuc_7x4`, and `bench_poly`.
- Preserved the user's current `AIRFRAME = 'aerosonde'` setting.

## 2026-08-31

### Stallion selection and propulsion enable/disable control

- Set `AIRFRAME = 'stallion'` in `run_periodic_analysis.m` as requested.
- Added `FEATURES.propulsion_enabled`. When `true`, the model named by
  `FEATURES.propulsion_model` is active; when `false`, the driver restores
  the original constant-efficiency propulsion model without requiring any
  solver edits.
- Added manufacturer-source metadata to `aircraft_params.m`: the Flightory
  Stallion product URL, published 60-70 km/h optimal-speed range, two-propeller
  layout, and clarification that the modeled 3.0 kg mass is the upper end of
  the published 1.5-3.0 kg all-up-weight range.
- Corrected the UIUC 7x4 surrogate loading calculation for the Stallion's
  twin-motor layout. Total aircraft thrust is now divided between two
  propellers before evaluating the single-propeller efficiency curve.
- Corrected the stored UIUC source filename to `uiuc_magf_7x4_6998.txt`.
- Added `propeller_drag_params.m` so the central settings block is complete
  and the default `FEATURES.propeller_drag = 'off'` configuration loads
  cleanly. Its nonzero coefficients remain documented placeholders and are
  not active manufacturer data.
- The Flightory page recommends 7x4/7x5/7x6 propellers and 3S or 4S power.
  Therefore, `uiuc_7x4` is a geometrically relevant forward-flight surrogate,
  but it is not an exact Gemfan/motor/voltage map and is not a validated
  Stallion propulsion prediction.
- MATLAB R2024b validation completed for all modified files. At an example
  total thrust of 8 N and airspeed of 17 m/s, the twin-propeller UIUC mode
  returned 324.981 W with modeled total efficiency 0.4185; disabling the
  feature returned the legacy constant-efficiency value of 238.095 W.

### One-at-a-time study model completion

- Connected the optional powered, windmilling, stopped, and folding-propeller
  drag modes to the aerodynamic drag polar using a smooth thrust-dependent
  transition. The coefficients remain explicit sensitivity assumptions.
- Connected optional constant vertical wind to the altitude dynamics.
- Updated steady cruise to use the same propeller drag and vertical-wind
  assumptions as the periodic model, including the required flight-path
  angle for constant ground-relative altitude. This prevents an unfair
  periodic-versus-steady comparison when either option is enabled.
- Added early-pulse, late-pulse, smooth-pulse, and double-pulse thrust
  warm-start modes to the fixed-period OCP for controlled waveform and
  multistart testing. These are initial-guess structures; thrust remains a
  continuously optimized control subject to the existing constraints.
- Added `run_feature_screening.m`, a reproducible one-at-a-time Stallion
  comparison. It holds the safety envelope and energy objective fixed, uses
  a documented coarse 200-interval mesh, reports both global and same-speed
  savings, and writes CSV/MAT results. It screens propulsion mapping, four
  propeller-drag states, four waveform starts, multistart, local period
  refinement, six aerodynamic sensitivities, and a 1 m/s updraft separately.

### One-at-a-time screening results (200 intervals)

- The all-options-off, constant-efficiency baseline at 100 s produced 2.8362%
  global saving and 3.0692% saving against steady flight at the same speed.
- UIUC 7x4 propulsion surrogate alone: 0.000257% for both comparisons.
- Powered, windmilling, stopped, and folding propeller-drag cases respectively
  produced global savings of 2.9396%, 0.4862%, 1.2866%, and 3.7672%; their
  same-speed savings were 3.1229%, 0.6190%, 1.4551%, and 3.8738%.
- Early, late, and smooth single-pulse starts converged to the same solution
  as the baseline. The double-pulse start converged to a worse local solution
  at 2.1360% global / 2.3076% same-speed.
- Seven-way multistart found no improvement over the baseline: 2.8362%
  global / 3.0692% same-speed.
- Period refinement from 90 to 110 s increased monotonically from 2.7824% to
  2.8881% global (3.0156% to 3.1210% same-speed). Because the best result was
  at the 110 s boundary, this screen did not locate an interior best period.
- One-at-a-time aerodynamic variants at 100 s gave global/same-speed savings:
  CD0 -10%: 2.7138/2.9787%; aspect ratio +10%: 2.6387/2.7831%; Oswald
  efficiency +10%: 2.6387/2.7831%; wing area +10%: 2.8090/2.9600%; mass
  -10%: 2.8077/2.9419%; and CLmax +10%: 2.9407/3.1735%.
- A constant 1 m/s updraft alone produced 0.8517% global / 0.8977%
  same-speed periodic saving. Its steady baseline fell to 9.1744 W, so this
  case represents environmental energy harvesting and is not comparable to
  the still-air absolute power level.
- These are screening results, not final mesh-converged values. Exact rows
  are stored in `feature_screening_results.csv` and the full MATLAB table in
  `feature_screening_results.mat`.

### Linear-propulsion rerun configuration

- Disabled the propulsion efficiency map in the main Stallion driver by
  setting `FEATURES.propulsion_enabled = false`. Active electrical power is
  again the original constant-efficiency relation `P = T*V/eta_total`.
- Removed the UIUC-map case from `run_feature_screening.m` so every rerun row,
  including every optional feature, uses the same linear propulsion model.
- Completed the 22-case linear-propulsion rerun successfully. The results
  reproduce the earlier constant-model rows: the all-off baseline is 2.8362%
  global / 3.0692% same-speed, and the largest screened value is the assumed
  folding-propeller case at 3.7672% / 3.8738%. Updated exact outputs replaced
  `feature_screening_results.csv` and `feature_screening_results.mat`; neither
  output now contains a propulsion-map case.

### Flight-path-angle envelope reduced to 15 degrees

- Reduced the Stallion flight-path-angle constraint from plus/minus 30 degrees
  to plus/minus 15 degrees in `run_periodic_analysis.m`.
- Applied the same plus/minus 15-degree limit in `run_feature_screening.m` so
  future one-at-a-time comparisons use the same envelope as the main driver.
- Previous screening savings remain results for the earlier plus/minus
  30-degree configuration and must not be presented as 15-degree results.
- Preserved those earlier results as `feature_screening_results_gamma30.csv`
  and `feature_screening_results_gamma30.mat`, then successfully reran all 22
  linear-propulsion screening cases with the plus/minus 15-degree limit.
- The 15-degree all-off baseline is 1.5004% global / 1.5759% same-speed,
  compared with 2.8362% / 3.0692% at 30 degrees. The assumed folding-propeller
  case remains the largest result but falls from 3.7672% / 3.8738% to
  2.3154% / 2.3499%.
- Added `feature_screening_gamma15_vs_gamma30.csv`, containing both results
  and the percentage-point change for every case. The active
  `feature_screening_results.csv` and `.mat` now contain the 15-degree data.

### Final production-run period extension

- Extended the main fixed-period list to `[7.24, 20, 40, 60, 80, 100, 120,
  140, 160, 180, 200]` seconds. This retains the 7.24-second PI-test period
  while extending the nonlinear search beyond the former 100/110-second
  boundary.
- The final configuration uses the plus/minus 15-degree flight-path limit,
  linear constant-efficiency propulsion, no propulsion map, no optional
  propeller drag or atmosphere, free average speed, and the production
  `dt_target = 0.1` second mesh.
- Completed all 11 full-resolution solves successfully. Average power fell
  monotonically from 60.9435 W at 7.24 s to 59.9612 W at 200 s. The best
  tested 200-second point gives 1.61194% global saving and 1.68735% same-speed
  saving at 13.3647 m/s average speed. Because the best point remains at the
  upper period boundary, 200 s is the best tested period, not a proven finite
  optimum; the improvement is approaching a shallow asymptote.
- The 200-second trajectory satisfies the active sampled envelope: flight-path
  angle -6.954 to 15.000 degrees, angle of attack 3.853 to 6.500 degrees,
  airspeed 12.865 to 14.364 m/s, altitude 51.95 to 225.85 m, vertical speed
  -1.648 to 3.436 m/s, load factor 0.821 to 1.152, and propulsion power 0 to
  234.96 W.
- Saved final outputs as `results_stallion_energy.mat`,
  `traj_stallion_energy_endurance_objective_tau200.{png,pdf,csv}`, and
  `sweep_stallion_energy.{png,pdf}`.

## 2026-09-01

### Flight-path angle increased to 25 degrees

- Increased the Stallion flight-path-angle envelope from plus/minus 15 degrees
  to plus/minus 25 degrees in both the main analysis and feature-screening
  driver.
- Retained linear constant-efficiency propulsion with the propulsion map off.
- Began a staged combination study that keeps still air and the existing safety
  constraints, distinguishes physical assumptions from optimizer warm starts,
  and recomputes the steady baseline for every airframe/drag combination.
- Added `run_combination_search.m`. It screens all eight combinations of
  folding-propeller drag, 10% higher CLmax, and 10% greater wing area at 200
  seconds, refines the winner over 100/150/200/250 seconds, and applies a
  seven-way multistart check at the best screened period. It exports separate
  CSV and MAT results for reproducibility.
- Added `run_combination_confirmation.m` to confirm the screened winner at
  1,000 intervals and export its complete solution and constraint summary.
- Added `confirm_combination_case.m` for equal-resolution confirmation of
  close competing folding-propeller combinations.
- The 300-interval screen initially favored folding-propeller drag plus 10%
  wing area at 3.7028% global saving, but equal-resolution confirmation showed
  that result was mesh-optimistic. At 1,000 intervals the confirmed candidates
  were: folding only 3.6330%, folding plus 10% CLmax 3.6464%, folding plus 10%
  wing area 3.1540%, and all three 3.5957% global saving.
- Selected folding-propeller drag plus 10% CLmax as the best confirmed tested
  combination. At 200 seconds and 1,000 intervals it gives 3.64641% global and
  3.77763% same-speed saving, with average power 60.0499 W and average speed
  12.9758 m/s.
- Its trajectory uses gamma -8.245 to 25.000 degrees, alpha 3.526 to 7.601
  degrees, speed 12.300 to 14.624 m/s, altitude 42.48 to 235.69 m, vertical
  speed -1.926 to 5.198 m/s, load factor 0.713 to 1.252, and propulsion power
  0 to 322.67 W. The gamma, alpha, and minimum-speed constraints are active.
- Made this tested combination active in `run_periodic_analysis.m` through
  `FEATURES.propeller_drag = 'folding'`, `FEATURES.aero_sensitivity = true`,
  and explicit `AERO_MODIFIERS` scales. Both folding drag and higher CLmax
  remain unvalidated sensitivity assumptions, not measured Stallion data.

### Standard windmilling-drag assumption for Stallion VTOL

- Rejected the folding-propeller configuration as the active Stallion VTOL
  model because the user's aircraft does not have folding propellers.
- Set `FEATURES.propeller_drag = 'windmilling'` and restored the original
  Stallion aerodynamic model with `FEATURES.aero_sensitivity = false`, so
  CLmax returns to 1.10.
- Retained a nominal equivalent-aircraft windmilling increment of
  `delta CD = 0.012` and added low/high sensitivity values 0.006 and 0.018.
  These are generic engineering assumptions, not Flightory measurements.
  Exact windmilling drag depends on propeller geometry, advance ratio, motor
  torque, and the state/orientation of all cruise and VTOL rotors.
- Added `run_windmilling_combination_search.m` to retest combinations using
  low/nominal/high equivalent windmilling drag, original or +10% CLmax, and
  original or +10% wing area at the common plus/minus 25-degree envelope.
  It also refines the screened winner over 100/150/200/250-second periods.
- Added `run_windmilling_confirmation.m` to confirm the optimistic low-drag
  screened winner at 250 seconds and 1,000 intervals.
- Completed the 12-case windmilling combination screen. With nominal
  `delta CD = 0.012` or high `0.018`, every aerodynamic combination converged
  to essentially steady flight (approximately zero periodic saving). With the
  optimistic low `delta CD = 0.006`, the best screen used +10% CLmax and the
  original wing area.
- Refined that low-drag case over 100/150/200/250 seconds; savings continued
  increasing to the 250-second boundary. Its 1,000-interval confirmation gave
  1.2212% global / 1.3843% same-speed saving, average power 61.674 W, and
  average speed 12.877 m/s. It used gamma -5.760 to 25.000 degrees, alpha
  4.002 to 7.601 degrees, speed 12.300 to 14.227 m/s, and load factor 0.732
  to 1.230.
- Kept the active main driver conservative at the nominal windmilling value
  `delta CD = 0.012` and original CLmax. The 1.2212% result is explicitly an
  optimistic low-drag plus higher-CLmax sensitivity case, not the active
  standard Stallion VTOL prediction.

## 2026-09-02

### Windmilling drag removed from active baseline

- Set `FEATURES.propeller_drag = 'off'` in `run_periodic_analysis.m` at the
  user's request. The main analysis now uses the clean-airframe theoretical
  drag polar without an added powered, stopped, windmilling, or folding
  propeller increment.
- Retained the windmilling implementation and prior sensitivity-result files
  for optional comparison, but they no longer affect the active analysis.

### Actual-endurance objective support

- Added `total_electrical_power.m`, which augments propulsion power with an
  optional constant onboard avionics/payload load `p.auxiliary_power_W`.
- Updated steady cruise, OCP energy integration, and trajectory diagnostics to
  use the same total electrical load. Existing runs are unchanged when the
  auxiliary-power field is absent.
- Added a `long_glide` OCP warm start with configurable powered-climb duty
  fraction for searching short-climb/long-glide solution branches.
- Added `run_actual_endurance_search.m`. It screens periods from 100 to 400
  seconds using sinusoidal and 10/20/30% powered-climb duty initializations,
  clean-airframe drag, plus/minus 25 degrees, an assumed 8 W onboard load,
  and a configurable 177.6 Wh usable-energy assumption for a 6S 10 Ah pack.
  It reports power savings, endurance hours, and minutes gained.
- Completed all 28 actual-endurance screening solves. With an assumed 8 W
  continuous onboard load and 177.6 Wh usable energy, steady endurance is
  2.576 h. The best screened point is 200 seconds at 67.339 W, 2.3267% global
  / 2.5049% same-speed saving, 2.637 h periodic endurance, and 3.682 minutes
  gained. The 20% climb-duty long-glide start won at 100 and 250 seconds, but
  the sinusoidal start found lower-power solutions at the other periods.
- The coarse 250-400 second results were non-monotonic, indicating local-solve
  and mesh sensitivity. The 200-second value is the best screened point, not
  yet a mesh-converged proof of the globally optimal period.

### Reverted actual-endurance reporting to power

- Removed the auxiliary-load/endurance-hours calculation and deleted
  `total_electrical_power.m` and `run_actual_endurance_search.m` at the user's
  request.
- Restored steady cruise, OCP energy integration, and trajectory diagnostics
  to `propulsion_power_model`. Results are again expressed strictly as average
  electrical propulsion power and percentage power saving.
- Kept the `long_glide` warm-start mode for future power-based trajectory and
  period searches; it does not calculate battery endurance or delta time.

### Average periodic speed pinned

- Confirmed `PIN_VAVG = true` in `run_periodic_analysis.m` at the user's
  request.
- For the endurance sweep, the OCP enforces
  `X(T)-X(0) = V_ref*T`, where `V_ref` is the best steady-endurance speed
  computed by `steady_cruise` (approximately 13.67 m/s for the current clean
  Stallion model).
- Global and same-speed savings should now be nearly identical because the
  periodic and steady reference conditions use the same average speed.
- Added `run_pinned_vavg_confirmation.m` for a targeted 200-second,
  1,000-interval clean-Stallion confirmation at the pinned steady-endurance
  reference speed.
- Completed that confirmation successfully. The periodic average speed equals
  the 13.674 m/s reference, average power is 59.423 W versus 60.944 W steady,
  and the pinned-speed power saving is 2.4955%. The trajectory uses gamma
  -7.530 to 25.000 degrees, alpha 2.749 to 6.500 degrees, speed 12.995 to
  15.126 m/s, and load factor 0.726 to 1.236.

### Generic Flightory Stallion efficiency model

- Reframed the active aircraft as the generic manufacturer fixed-wing
  Stallion rather than the user's particular 6S VTOL build; recorded the
  manufacturer-generic 4S configuration and retained two cruise motors.
- Added selectable propulsion mode `flightory_generic`, anchored to an
  inferred nominal combined cruise efficiency near 0.67 from Flightory's
  approximate 4 A at 60-70 km/h statement and the current drag model.
- The smooth sensitivity map peaks near ordinary cruise thrust/speed, reduces
  efficiency at high climb thrust, and never depends directly on flight-path
  angle. During a true zero-thrust glide, propulsion power is zero and
  efficiency has no effect.
- Enabled the new generic Flightory map in `run_periodic_analysis.m`. It is a
  manufacturer-anchored sensitivity assumption, not measured dynamometer or
  in-flight efficiency data.
- Added `run_flightory_efficiency_search.m` to screen 50-300-second periods
  and sinusoidal versus 10/20/30/40% powered-climb-duty initializations with
  pinned average speed and the generic Flightory efficiency map.
- Completed the 30-case screen. The coarse best is 200 seconds with a
  30%-powered/70%-glide initialization, 53.299 W versus 53.577 W steady, or
  0.5184% saving. Added `run_flightory_efficiency_confirmation.m` for a
  1,000-interval confirmation of this candidate.
- Completed the confirmation: 53.352 W periodic versus 53.577 W steady,
  giving 0.41873% saving at the exactly pinned 14.459 m/s average speed.
  The refined trajectory is mild rather than an aggressive long glide:
  gamma -5.778 to +1.768 degrees, alpha 3.980 to 5.653 degrees, speed 13.380
  to 14.839 m/s, load factor 0.946 to 1.053, thrust 0 to 2.984 N, and about
  18.9% of nodes below 0.05 N thrust. Modeled efficiency spans 0.616-0.657.

### Comprehensive saving-mechanism screen

- Added a dedicated `efficiency_ratio` sensitivity mode with independently
  configurable cruise and high-load climb efficiencies. This is a threshold
  study, not a claimed hardware map.
- Extended the `long_glide` warm start with a configurable climb-thrust
  fraction so thrust level and powered duty cycle can be screened separately.
- Added `run_comprehensive_savings_study.m` to record one-at-a-time savings
  for efficiency ratios, climb-thrust fractions, altitude envelopes, pinned
  speeds, free-speed range cost, a 1 m/s updraft, and powered/stopped/
  windmilling propeller states on a common 150-interval screening mesh.
- Completed all 24 comprehensive screening cases at a 200-second period. The
  full machine-readable results are in `comprehensive_savings_screen.csv` and
  `comprehensive_savings_screen.mat`. Every nonlinear solve reported success.
- The generic Flightory-map baseline saved 0.5051% on this coarse mesh. The
  climb-thrust warm-start fractions of 20/40/60/80/100% saved
  0.4091/0.4091/0.5429/0.4091/0.5051%, respectively. These settings change
  the initial guess, not a hard constraint on the final optimized waveform;
  the 60% start found the best local solution in this group.
- Altitude envelopes of plus/minus 20/40/60/100 m saved
  0.3941/0.4278/0.3962/0.3962%. Pinned average speeds of 14/16/18/20 m/s
  saved 0.5279/0.2699/0.1730/0.2814%. The free-speed range objective saved
  0.2082% in energy per distance rather than average power.
- The 1 m/s updraft case saved 4.1773%, but this is atmospheric energy
  harvesting and is not comparable to a still-air onboard-control result.
  Generic powered/stopped/windmilling propeller-state cases saved
  0.4425/-0.00014/-0.00017%, respectively. The stopped and windmilling
  assumptions eliminated the apparent periodic benefit in this screen.
- The independent efficiency-ratio sensitivity cases, with climb efficiency
  divided by cruise efficiency equal to 0.75/1.00/1.10/1.20/1.30, saved
  -0.0003/1.9088/11.4682/19.1954/25.3062%, respectively. Values above 1.0
  intentionally assume propulsion is more efficient under high climb load.
  They demonstrate how strong that mechanism would need to be; they are not
  predictions for the Stallion without measured motor/propeller data.
- These results are a one-at-a-time, 150-interval local-optimizer screen.
  They should be used to select candidates for a mesh-refined confirmation,
  not as certified global optima or aircraft safety/performance guarantees.

### Returned active analysis to constant propulsion efficiency

- Disabled `FEATURES.propulsion_enabled` in `run_periodic_analysis.m` at the
  user's request. The active power equation is again the original constant-
  efficiency relation `P_elec = T*V/eta_total`.
- Retained `FEATURES.propulsion_model = 'flightory_generic'` as the model that
  will be selected if the propulsion enhancement is re-enabled later. The
  map implementation and its prior result files were not deleted.
- All other active optional mechanisms remain disabled: battery, propeller
  drag, multistart, period refinement, aerodynamic sensitivity, and
  atmosphere. Average speed remains pinned and the Stallion flight-path-angle
  limit remains plus/minus 25 degrees.
- The applicable previously completed 1,000-interval confirmation for this
  configuration saved 2.4955%: 59.423 W periodic versus 60.944 W steady at
  13.674 m/s average speed. It is an idealized constant-efficiency result.

### Full constant-efficiency improvement study

- Added `run_constant_efficiency_full_study.m` to test the remaining proposed
  improvements under the active still-air, clean-propeller, constant-efficiency
  model with average speed pinned and the plus/minus 25-degree gamma limit.
- The study includes eleven waveform/timing starts, a 100-300 second period
  sweep, altitude allowances, pinned speeds, six aerodynamic design
  sensitivities, stricter minimum-airspeed margins, a best operational
  combination, and 240/480-interval mesh checks. Safety constraints remain
  active and each case recomputes its matching steady-flight reference.
- Aerodynamic cases record both the periodic saving relative to their own
  redesigned steady baseline and the steady-power change relative to the
  original airframe, preventing design improvement from being mislabeled as
  periodic-control saving.
- Corrected the new study's diagnostic extraction to use the solver's
  lowercase `sol.x` and `sol.alpha` trajectory fields after its first reporting
  attempt exposed the naming mismatch; no solver equations were changed.
- Added `run_constant_efficiency_followup.m` after the first sweep revealed
  that the preliminary combination selector compared only the explicitly
  screened plus/minus 20-100 m altitude cases and omitted the existing
  plus/minus 300 m allowance. The follow-up records all eleven waveform starts
  individually, searches periods 210-240 seconds in 5-second increments, adds
  200/300 m altitude cases, and confirms the corrected best combination.
- Added `run_constant_efficiency_N960_confirmation.m` for a final high-resolution
  check of the selected 235-second, 13 m/s candidate after the 120/240/480
  interval results showed material mesh sensitivity.
- Completed the entire constant-efficiency study. All recorded solves reported
  success. The 200-second waveform starts steady/sine/single/early/late/smooth
  and long-glide 20/30% all converged to 2.4342%; double-pulse and long-glide
  10/40% converged to the inferior 2.1832% local solution.
- The 100/125/150/175/200/225/250/275/300-second coarse period savings were
  2.2264/2.3115/2.3569/2.3801/2.4342/2.4660/2.3700/2.3620/2.3706%. A finer
  210/215/220/225/230/235/240-second sweep gave
  2.4477/2.4540/2.4601/2.4660/2.4858/2.5048/2.4605%, so 235 seconds was the
  best screened period at the steady-optimal pinned speed.
- Plus/minus 20/40/60/100/200/300 m altitude settings saved
  1.6534/2.0316/2.2353/2.3576/2.4660/2.4660%. The absolute altitude bounds of
  30-250 m make allowances above roughly 100 m progressively inactive.
- Pinned average speeds 13.0/13.5/13.674/14.5/15.5/17.0 m/s saved
  2.9569/2.5823/2.4660/1.8920/1.3044/0.4240% on the screening mesh. Each was
  compared with steady flight at that same speed; 13 m/s is not the globally
  optimal steady-flight speed and its steady reference is 61.161 W rather
  than the global steady minimum of 60.944 W.
- Ten-percent design sensitivities CD0 reduction, aspect-ratio increase,
  Oswald-efficiency increase, wing-area increase, mass reduction, and CLmax
  increase produced periodic savings of
  2.3410/2.3141/2.3141/2.4735/2.4875/2.4661% relative to their respective
  steady baselines. Their steady-power changes versus the original design
  were +2.5996/+6.8986/+6.8986/+4.6536/+14.6185/0.0000%, respectively.
- Raising the minimum airspeed by 0/0.5/1.0/1.5 m/s gave
  2.4660/2.4658/2.5186/2.3934%. The apparent 2.5186% at Vmin=13.3 m/s is a
  separate local solution, not evidence that an added restriction creates
  physical energy.
- The corrected coarse operational combination used T=235 s, pinned average
  speed 13 m/s, plus/minus 300 m allowance, and a 20% long-glide start. It
  saved 3.0019% at N=120. Mesh results were 2.7194% at N=240, 2.8066% at
  N=480, and 3.0124% at N=960. The N=960 case used 59.319 W versus 61.161 W
  steady, gamma -5.580 to +25.000 degrees, alpha up to 6.5005 degrees, load
  factor 0.741-1.171, and minimum airspeed 12.3015 m/s.
- Because the selected trajectory essentially touches the 12.3 m/s minimum
  airspeed and the mesh sequence is not monotonic, 3.0124% is an aggressive
  constrained-model result rather than a robust flight-test prediction. The
  previously confirmed 2.4955% at the steady-optimal 13.674 m/s average speed
  remains the cleaner reference result.
- Saved complete results in `constant_efficiency_full_study.csv/.mat`,
  `constant_efficiency_followup.csv/.mat`, and
  `constant_efficiency_best_N960.csv/.mat`.
- Added `run_three_speed_comparisons.m` and completed a common T=235 s,
  N=960 comparison: both vehicles at 13.674 m/s saved 2.3473%, both at
  13 m/s saved 3.0124%, and periodic at 13 m/s versus globally optimal steady
  flight at 13.674 m/s saved 2.6661%.
- Added `compile_requested_speed_comparisons.m` to separately report the
  best refined result already found at each pinned speed. This uses the
  T=200 s, N=1000 result at 13.674 m/s and the T=235 s, N=960 result at
  13 m/s rather than forcing both trajectories to have the same period.
- Corrected its first reporting attempt to use the older result table's
  `period_s` and `intervals` column names; calculated trajectory results were
  unaffected.
- Restored `propeller_drag_params.m` and `propeller_drag_cd.m` after the final
  cleanup removed dependencies that are called by the active dynamics and
  steady-cruise code. The final `off` setting returns exactly zero added drag;
  nonzero coefficients remain explicitly generic sensitivity assumptions.
- Converted `run_periodic_analysis.m` to a selectable final three-case workflow.
  With `FINAL_THREE_CASES=true`, it runs and exports the endurance PI test,
  solves the optimal-speed matched case at T=200/N=1000, solves the 13 m/s
  periodic trajectory at T=235/N=960, and reports: both at optimal speed,
  both at 13 m/s, and periodic 13 m/s versus globally optimal steady flight.
  It exports a CSV/MAT summary, one comparison plot, and three separately
  labeled ten-panel trajectory plots. No analysis was launched while making
  these edits, per the user's request.

### How the 13 m/s comparison speed was selected

- The 13 m/s value was not produced by allowing the final OCP to choose an
  unrestricted average speed. It was selected from a controlled discrete
  pinned-speed sensitivity sweep at a 225-second period and 120-interval
  screening resolution.
- The tested pinned average speeds were 13.0, 13.5, 13.674, 14.5, 15.5, and
  17.0 m/s. Their periodic savings relative to steady flight at the same
  respective speed were 2.9569%, 2.5823%, 2.4660%, 1.8920%, 1.3044%, and
  0.4240%. Therefore, 13.0 m/s was the best tested same-speed condition.
- A subsequent fine period search selected 235 seconds. At 13 m/s and the
  960-interval confirmation mesh, periodic power was 59.319 W versus
  61.161 W for steady flight at 13 m/s, giving 3.0124% same-speed saving.
- The globally optimal steady-endurance speed remains 13.67375 m/s, with
  steady power 60.944 W. Comparing the 13 m/s periodic result against that
  global steady optimum gives 2.6661% saving. Thus 13 m/s is a deliberately
  tested periodic operating condition, not the optimal steady-cruise speed.
- The confirmed 13 m/s trajectory reaches a minimum airspeed of approximately
  12.302 m/s, nearly activating the 12.3 m/s constraint. The 13 m/s result is
  consequently treated as an aggressive theoretical sensitivity case.
- Improved final-run console progress reporting. Before each OCP solve,
  `run_periodic_analysis.m` now prints the case number, period, interval count,
  approximate time step, and pinned average speed. It prints elapsed solve time
  afterward and explicitly states that Case 3 reuses the Case 2 trajectory.
- Added `FINAL_RUN_PERIOD_SWEEP=true` to the final workflow. Before the three
  final comparison cases, the driver now solves the fixed list in its displayed
  order `[7.24, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200]` seconds.
- For every fixed-period solve, the console prints T, N, dt, solver status,
  average power, energy per distance, average speed, global and same-speed
  savings, and min/max airspeed, gamma, alpha, load factor, vertical speed,
  thrust, and electrical power before advancing to the next period. A final
  sweep table and `final_fixed_period_sweep.mat` are also saved.
- Corrected the final workflow after clarifying that the requested three cases
  are three complete period sweeps, not three single-period comparisons.
  Sweep 1 pins periodic flight at 13 m/s and compares it with steady 13 m/s;
  Sweep 2 pins periodic flight at the 13.67375 m/s optimal steady speed and
  compares it with steady flight at that speed; Sweep 3 compares the Sweep 1
  periodic trajectories with globally optimal steady flight at 13.67375 m/s.
- Sweep 3 reuses Sweep 1 trajectories because its OCP and periodic speed are
  identical; only the reporting baseline changes. The driver saves one CSV,
  PNG, and PDF per sweep, a combined long-format CSV/MAT result, and the best
  trajectory plot for each comparison.
- Enabled targeted multistart for the plus/minus 15-degree final sweeps. Each
  period now tries steady, sinusoidal, early-pulse, and 20%-powered long-glide
  initializations, prints the outcome of every attempt, and retains the
  successful solution with the lowest average power. The long-glide percentage
  remains an initial-guess duty fraction rather than a final trajectory
  constraint.
- Replaced full-resolution multistart with staged multistart after the first
  attempt showed that four large meshes per period would require excessive
  runtime. Every guess is now screened at up to 120 intervals; the lowest-power
  screening guess is then rerun once at the requested full resolution. The
  initial direct run was stopped during the 100-second case and produced no
  final result files; the staged run supersedes it.
- During the first staged attempt, a numerically negligible coarse-mesh
  difference selected an early-pulse branch at 60 seconds that became worse
  after refinement. Added a 0.01 W screening tolerance and placed the smooth
  sinusoidal start first, so near-tied candidates retain the stable smooth
  branch rather than switching on coarse solver noise. That incomplete run was
  stopped before result files were written and is superseded by the rerun.
- Reduced the final three-sweep screening target from dt=0.1 s to dt=0.25 s
  after the 22-period, staged-multistart run remained impractically slow.
  This gives up to 800 intervals at 200 seconds. The intent is to identify
  the best plus/minus 15-degree period first and then refine only that winner;
  the interrupted partial run wrote no final sweep result files.
- Completed the staged-multistart plus/minus 15-degree final run successfully.
  For both periodic and steady at 13 m/s, savings over periods
  7.24/20/40/60/80/100/120/140/160/180/200 s were
  -0.0071/0.8107/1.4137/1.5971/1.6874/1.7417/1.7782/1.8040/
  1.8227/1.8374/1.8494%.
- For both periodic and steady at the 13.67375 m/s optimal steady speed, the
  corresponding savings were approximately
  0/0.7271/1.1791/1.3286/1.4033/1.4480/1.4779/1.4993/1.5153/
  1.5278/1.5378%.
- Comparing the 13 m/s periodic trajectories with globally optimal steady
  flight gave -0.3642/0.4565/1.0616/1.2458/1.3364/1.3909/1.4274/
  1.4534/1.4721/1.4869/1.4989% over the same period list.
- The 200-second boundary was best in all three displayed sweeps, so this run
  does not prove that 200 seconds is the true plus/minus 15-degree optimum.
  The best 200-second trajectories respected the coded envelope; the 13 m/s
  case used V=12.552-14.115 m/s, gamma=-7.429 to +15 degrees, alpha up to
  6.500 degrees, and load factor 0.815-1.150. The optimal-speed case used
  V=13.231-14.600 m/s, gamma=-6.655 to +15 degrees, alpha up to 6.500 degrees,
  and load factor 0.825-1.162.
- Multistart changed which branch was retained at several periods. For example,
  the long-glide start was best in the 80-second optimal-speed screening, while
  the steady start beat sine at 60 and 180 seconds. At the final 200-second
  points, the sinusoidal start was selected for both speed sweeps.
- Changed the final Stallion flight-path-angle limit from plus/minus 15 degrees
  to plus/minus 20 degrees for a directly comparable rerun. The period list,
  dt=0.25 s screening mesh, three speed comparisons, constant propulsion
  efficiency, staged multistart, and all other constraints remain unchanged.
- Returned the final flight-path-angle limit to plus/minus 25 degrees at the
  user's request. This does not equate gamma with angle of attack; the 6.5-degree
  stall-aware alpha constraint and 12.3 m/s minimum airspeed remain active.
- Extended the final sweep through 300 seconds using periods 220, 230, 235,
  240, 260, 280, and 300 seconds in addition to the existing list, so a peak
  and subsequent decline can be identified rather than assuming 200 seconds
  is optimal. Set the extended screening mesh target to dt=0.5 s to control
  runtime; the winning period should be confirmed on a finer mesh.
- Added a separate saving-versus-period PNG/PDF for each of the three speed
  comparisons, with the maximum marked and reported in the title. These plots
  show electrical propulsion-power saving because the battery model is off;
  they must not be described as nonlinear battery-state predictions. The
  existing best-trajectory plot is still generated for each comparison.
- Completed the extended plus/minus 25-degree run. All three comparisons peak
  at 230 seconds on the dt=0.5 s screening mesh, demonstrating an actual dip
  after the maximum. Savings at 230 seconds are 3.0232% for both at 13 m/s,
  2.5127% for both at 13.67375 m/s, and 2.6769% for periodic 13 m/s versus
  globally optimal steady flight. At 240 seconds these fall to 2.9436%,
  2.4253%, and 2.5970%, respectively.
- The best 13 m/s trajectory uses 59.312 W versus 61.161 W steady. It reaches
  gamma=25 degrees, alpha=6.5 degrees, and minimum airspeed about 12.338 m/s,
  so it remains feasible in the coded model but close to active aerodynamic
  limits. The best optimal-speed trajectory uses 59.412 W versus 60.944 W.
- Temporarily reduced the Stallion flight-path-angle constraint from
  plus/minus 25 degrees to plus/minus 15 degrees for a comparison study. The
  active setting was subsequently returned to plus/minus 25 degrees; results
  from the two angle limits must not be mixed.
- At the user's request, reverted the experimental 80--180 m altitude and
  -3/+3 m/s vertical-speed limits to the preceding 30--250 m and -5/+8 m/s
  settings. The active Stallion gamma limit remains plus/minus 25 degrees.
  The previously recorded 3.0232% saving belongs to this restored envelope;
  the interrupted tighter-envelope run is not a replacement for it.
- Simplified the final analysis workflow to one comparison only. The periodic
  trajectory is pinned to the globally optimal steady-endurance speed, and its
  average electrical power is compared with steady cruise at that identical
  speed. The driver no longer solves or plots either 13 m/s comparison. New
  outputs use the `final_optimal_speed_sweep_results` and
  `both_optimal_speed` names so they are distinguishable from older three-case
  files that may still exist in the folder.
