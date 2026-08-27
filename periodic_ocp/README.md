# Periodic optimal control — fixed-wing UAV

MATLAB and CasADi: Π(ω) frequency test and Radau direct collocation for a longitudinal point-mass model. Code for the paper *Periodic Range Optimal Control for a Fixed-Wing UAV* (Stallion, fuel-rate cost, pinned average speed).

## Requirements

- MATLAB; Symbolic Math Toolbox (`pi_test_core.m`)
- CasADi for MATLAB; path in `add_casadi_path.m`
- IPOPT (included with most CasADi builds)
- Optional: Parallel Computing Toolbox; `USE_PARALLEL` in `run_periodic_analysis.m`

## Run

1. `cd` to this folder in MATLAB.
2. In `run_periodic_analysis.m` set: `AIRFRAME = 'stallion'`, `COST = 'fuel_rate'`, `PI_MODE_RANGE = 'distance_density'`, `PIN_VAVG = true`, and `SOLVE_RANGE` / `SOLVE_ENDURANCE` as needed.
3. Run `run_periodic_analysis`.

`distance_density`: Π-test uses \(\ell_d = \sigma T / (V \cos \gamma + \varepsilon)\) at range trim.  
`PIN_VAVG`: enforces \(X(\tau) - X(0) = V_{\mathrm{ref}} \tau\).

Output: `results_<airframe>_<cost>.mat` (figures depend on driver options).

## Model

| | |
|--|--|
| States | \(x = [X, Z, \gamma, V]^\top\) |
| Controls | \(u = [\alpha, T]^\top\) |
| Dynamics | `aircraft_dynamics.m` |
| Steady grid | `steady_cruise.m` (`p.cruise_mode` = range or endurance) |
| Periodic NLP | `ocp_casadi_fixed_T.m` (Radau, degree 3, IPOPT) |

## Π(ω) (`pi_test_core.m`)

- `pi_mode`: `'running_cost'`, `'isoperimetric'`, `'distance_density'`
- `state_mode`: `'reduced2'` ( \([\gamma, V]\) )

## Other files

| File | |
|------|------|
| `compare_pi_variants.m` | Π variants |
| `debug_warmstart.m` | NLP warm-start |
| `simulate_warmstart.m` | Simulation |
| `thrust_max.m` | \(T_{\max}(V)\) if `p.thrust_model` ≠ `'constant'` |
| `resample_seed_guess.m` | Resample prior solution for continuation |

## Parameters

- `aircraft_params.m`: Stallion  
- `aerosonde_params.m`: Aerosonde (paper parameters)  
- `mk30_params.m`: MK30 placeholder  

Tables in the paper follow the driver: `dt_target`, `T_list`, and Stallion envelope bounds.

## Authors

Sanjay Maharjan and Adrian Stein, Mechanical & Industrial Engineering, Louisiana State University.
