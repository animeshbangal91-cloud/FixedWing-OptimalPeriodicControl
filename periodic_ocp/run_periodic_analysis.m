%% run_periodic_analysis.m
% =========================================================================
% Unified periodic optimal control analysis.
%
% Outputs ACC-ready figures:
%   sweep_stallion_fuel_rate_pinV.png/.pdf
%   traj_stallion_fuel_rate_range_objective_tau17.png/.pdf
%
% Assumes these functions are on path:
%   add_casadi_path.m
%   aircraft_params.m / aerosonde_params.m / mk30_params.m
%   steady_cruise.m
%   pi_test_core.m
%   ocp_casadi_fixed_T.m
% =========================================================================

clear; clc; close all;

here = fileparts(mfilename('fullpath'));
addpath(here);

%% =================== USER CHOICES ===========================
AIRFRAME  = 'stallion';          % 'stallion' | 'aerosonde' | 'mk30'
COST      = 'energy';            % 'energy' | 'fuel_rate'
PI_COST   = 'energy';            % PI-test cost mode
PI_MODE_ENDUR = 'running_cost';  % PI mode for endurance trim
PI_MODE_RANGE = 'distance_density';

SOLVE_ENDURANCE = true;
SOLVE_RANGE     = false;

% Final workflow: sweep periodic flight at the globally optimal steady-
% endurance speed and compare it with steady cruise at that same speed.
FINAL_OPTIMAL_SPEED_ONLY = true;

dt_target = 0.50; % extended-period screening; refine only the winner later

USE_PARALLEL = false;
N_WORKERS = [];

USE_CONTINUATION = false;
PIN_VAVG = true;

% Keep the linear lift model below the specified maximum lift coefficient.
% A value below 1 adds margin relative to the nominal CL_max boundary.
STALL_MARGIN = 0.96;

% Provisional maneuvering envelope. Replace these values with validated
% airframe limits before using the result as a flight-safety claim.
LOAD_FACTOR_BOUNDS = [0.5, 2.0];

% ---------------- Optional models / studies -----------------
% Change only these settings to turn individual enhancements on or off.
FEATURES = struct();
FEATURES.battery_model       = false;
FEATURES.propulsion_enabled  = false;       % legacy constant eta_total model
FEATURES.propulsion_model    = 'flightory_generic'; % retained for optional re-enable
FEATURES.propeller_drag      = 'off';       % clean-airframe theoretical baseline
FEATURES.multistart          = true;
FEATURES.refine_period       = false;
FEATURES.aero_sensitivity    = false;
FEATURES.fair_comparison     = true;
FEATURES.atmosphere          = false;

% Optional-study settings (used only when the corresponding switch is on).
MULTISTART_MODES = {'sine_periodic','steady','long_glide_20','early_pulse'};
MULTISTART_AMPS = 0.50;
REFINE_PERIOD_HALF_WIDTH = 10; % [s]
REFINE_PERIOD_STEP = 2;        % [s]
ATMOSPHERE_VERTICAL_WIND = 1.0; % [m/s], positive upward

% Applied only when FEATURES.aero_sensitivity is true. Values other than 1
% are design sensitivities, not manufacturer-validated Stallion parameters.
AERO_MODIFIERS.CL_max_scale = 1.10;
AERO_MODIFIERS.wing_area_scale = 1.00;

% Additional provisional flight-envelope limits. Replace with measured or
% manufacturer-qualified Stallion limits when available.
ALTITUDE_BOUNDS = [30, 250];             % [m]
VERTICAL_SPEED_BOUNDS = [-5, 8];         % [m/s], [maximum sink, maximum climb]
GAMMA_RATE_MAX = deg2rad(15);             % [rad/s]
PROPULSION_POWER_MAX = 900;               % [W] electrical input
%% ============================================================

%% ---------------- CasADi path -------------------------------
add_casadi_path();
import casadi.*

%% ---------------- Load airframe -----------------------------
fprintf('\n');
fprintf('============================================================\n');
fprintf('  Periodic Optimal Control Analysis (fresh)\n');
fprintf('  Airframe: %s   |   Cost: %s\n', upper(AIRFRAME), COST);
fprintf('  PI test cost mode: %s\n', PI_COST);
fprintf('  PI mode (endurance trim): %s\n', PI_MODE_ENDUR);
fprintf('  PI mode (range trim): %s\n', PI_MODE_RANGE);
fprintf('============================================================\n');

switch lower(AIRFRAME)
    case 'stallion'
        p = aircraft_params();
        ocp_z_exc = 300;
        ocp_gamma = deg2rad(25);
        ocp_alpha = [p.alpha_min, p.alpha_max];

    case 'aerosonde'
        p = aerosonde_params();
        p.alpha_max = deg2rad(25);
        p.alpha_min = -deg2rad(25);
        ocp_z_exc = 20;
        ocp_gamma = deg2rad(45);
        ocp_alpha = [p.alpha_min, p.alpha_max];

    case 'mk30'
        p = mk30_params();
        ocp_z_exc = 80;
        ocp_gamma = deg2rad(20);
        ocp_alpha = [p.alpha_min, p.alpha_max];

    otherwise
        error('Unknown airframe: %s.', AIRFRAME);
end

if FEATURES.aero_sensitivity
    p.CL_max = AERO_MODIFIERS.CL_max_scale*p.CL_max;
    p.S = AERO_MODIFIERS.wing_area_scale*p.S;
    fprintf('  Aero sensitivity: CL_max x %.3f, wing area x %.3f\n', ...
        AERO_MODIFIERS.CL_max_scale,AERO_MODIFIERS.wing_area_scale);
end

if FEATURES.battery_model
    p.battery = battery_params('I_max', 15.0);
    fprintf('  Battery model: generic %dS %.1f Ah %s (initial SOC %.0f%%)\n', ...
        p.battery.n_series, p.battery.capacity_Ah, p.battery.chemistry, ...
        100*p.battery.soc0);
end

if ~FEATURES.propulsion_enabled
    active_propulsion_model = 'constant';
else
    active_propulsion_model = lower(FEATURES.propulsion_model);
end

switch active_propulsion_model
    case 'constant'
        p.propulsion_efficiency_model = 'constant';
        fprintf('  Propulsion model: constant efficiency\n');
    case 'generic'
        p.propulsion_efficiency_model = 'map';
        p.propulsion_map = propulsion_map_params();
        fprintf('  Propulsion model: generic sensitivity map\n');
    case 'flightory_generic'
        p.propulsion_efficiency_model = 'flightory_generic';
        p.propulsion_map = propulsion_map_params();
        fprintf('  Propulsion model: generic Flightory Stallion 4S sensitivity map\n');
    case 'uiuc_7x4'
        p.propulsion_efficiency_model = 'uiuc_7x4';
        p.propulsion_map = propulsion_map_params();
        fprintf('  Propulsion model: UIUC Master Airscrew 7x4 surrogate\n');
    case 'bench_poly'
        p.propulsion_efficiency_model = 'bench_poly';
        p.propulsion_map = propulsion_map_params();
        p.T_max = min(p.T_max, p.propulsion_map.T_max_measured);
        fprintf('  Propulsion model: supplied EMAX/Gemfan static bench fit\n');
    otherwise
        error('Unknown FEATURES.propulsion_model: %s', FEATURES.propulsion_model);
end

if FEATURES.propulsion_enabled
    fprintf('  Propulsion enhancement: ENABLED\n');
else
    fprintf('  Propulsion enhancement: DISABLED (legacy constant efficiency)\n');
end

p.propeller_drag_model = lower(FEATURES.propeller_drag);
p.propeller_drag = propeller_drag_params();

if FEATURES.atmosphere
    p.vertical_wind = ATMOSPHERE_VERTICAL_WIND;
else
    p.vertical_wind = 0;
end

% Convert CL_max into an angle-of-attack limit because the aerodynamic
% model remains linear in alpha and does not model post-stall lift loss.
if isfield(p, 'CL_max') && ~isempty(p.CL_max)
    alpha_at_CLmax = (p.CL_max - p.CL0) / p.CLa;
    alpha_stall_limit = STALL_MARGIN * alpha_at_CLmax;
    ocp_alpha(2) = min(ocp_alpha(2), alpha_stall_limit);
    fprintf('  Stall-aware alpha upper bound: %.3f deg (margin %.1f%% of CL_max angle)\n', ...
        rad2deg(ocp_alpha(2)), 100*STALL_MARGIN);
end

%% ---------------- Steady cruise baselines -------------------
p.cruise_mode = 'range';
base_range = steady_cruise(p, false, COST);

p.cruise_mode = 'endurance';
base_endur = steady_cruise(p, false, COST);

[time_lbl, dist_lbl] = cost_labels(COST);
[time_grid_endur, dist_grid_endur] = cost_grids(base_endur, COST);
[time_grid_range, dist_grid_range] = cost_grids(base_range, COST);

fprintf('\n--- Steady Cruise Summary (cost = %s) ---\n', COST);
fprintf('  Range mode:     V = %.2f m/s, %s = %.4f, %s = %.4f\n', ...
    base_range.V_op, time_lbl, time_grid_range(base_range.i_op), ...
    dist_lbl, dist_grid_range(base_range.i_op));
fprintf('  Endurance mode: V = %.2f m/s, %s = %.4f, %s = %.4f\n', ...
    base_endur.V_op, time_lbl, time_grid_endur(base_endur.i_op), ...
    dist_lbl, dist_grid_endur(base_endur.i_op));

P_steady_endur   = time_grid_endur(base_endur.i_op);
JpM_steady_range = dist_grid_range(base_range.i_op);

fprintf('\n  Steady baselines for comparison:\n');
fprintf('    Best endurance: %s = %.4f  (at V = %.2f m/s)\n', ...
    time_lbl, P_steady_endur, base_endur.V_op);
fprintf('    Best range:     %s = %.4f  (at V = %.2f m/s)\n', ...
    dist_lbl, JpM_steady_range, base_range.V_op);

%% ---------------- Phase 1: pi(omega) test -------------------
fprintf('\n--- Phase 1: pi(omega) test ---\n');

pi_endur = [];
if SOLVE_ENDURANCE
    p.cruise_mode = 'endurance';
    base_pi_endur = steady_cruise(p, false, PI_COST);
    pi_endur = pi_test_core(p, base_pi_endur, PI_COST, 'endurance trim', ...
        'reduced2', PI_MODE_ENDUR);
    if FINAL_OPTIMAL_SPEED_ONLY
        exportgraphics(gcf,fullfile(here,'final_pi_test.png'),'Resolution',300,'BackgroundColor','white');
        exportgraphics(gcf,fullfile(here,'final_pi_test.pdf'),'ContentType','vector','BackgroundColor','white');
    end
    fprintf('  Endurance trim PI: T*=%.2f s, lambda_min=%.4e\n', ...
        pi_endur.T_star, pi_endur.lam_worst);
    if pi_endur.min_at_boundary
        fprintf('    note: minimum is at frequency-grid boundary (no clear interior dip)\n');
    end
end

pi_range = [];
if SOLVE_RANGE
    p.cruise_mode = 'range';
    base_pi_range = steady_cruise(p, false, PI_COST);
    pi_range = pi_test_core(p, base_pi_range, PI_COST, 'range trim', ...
        'reduced2', PI_MODE_RANGE);
    fprintf('  Range trim PI:     T*=%.2f s, lambda_min=%.4e\n', ...
        pi_range.T_star, pi_range.lam_worst);
    if pi_range.min_at_boundary
        fprintf('    note: minimum is at frequency-grid boundary (no clear interior dip)\n');
    end
end

if SOLVE_RANGE
    pi_result = pi_range;
elseif SOLVE_ENDURANCE
    pi_result = pi_endur;
else
    error('At least one of SOLVE_ENDURANCE / SOLVE_RANGE must be true.');
end

%% ---------------- Phase 2: OCP sweep ------------------------
USE_FIXED_TLIST = true;
FIXED_TLIST = [7.24, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, ...
               220, 230, 235, 240, 260, 280, 300];

if USE_FIXED_TLIST
    T_list = FIXED_TLIST;
else
    T_seed  = pi_result.T_star;
    T_short = T_seed * [0.6 0.8 0.9 1.0 1.1 1.2 1.5];
    T_long  = T_seed * [2.0 3.0 4.0 5.0];
    T_list  = unique(sort([T_seed, T_short, T_long]));
    T_list  = T_list(T_list >= 3.0 & T_list <= 20.0);
    if isempty(T_list)
        warning('T_list empty after T>=3s filter; falling back to [T_seed].');
        T_list = T_seed;
    end
end

fprintf('\n--- Phase 2: Periodic OCP Sweep ---\n');
fprintf('  Cost mode: %s\n', COST);
if PIN_VAVG
    fprintf('  V_avg: PINNED via distance constraint (D = V_ref*T)\n');
else
    fprintf('  V_avg: FREE (no distance constraint)\n');
end
fprintf('  T_list = [%s] s\n', strjoin(string(round(T_list,2)), ', '));
fprintf('  dt_target = %.3f s\n\n', dt_target);

opts = struct();
opts.print_level      = 0;
opts.max_iter         = 5000;
opts.cost_mode        = COST;
opts.z_excursion_max  = ocp_z_exc;
opts.gamma_max        = ocp_gamma;
opts.alpha_min        = ocp_alpha(1);
opts.alpha_max        = ocp_alpha(2);
opts.load_factor_min  = LOAD_FACTOR_BOUNDS(1);
opts.load_factor_max  = LOAD_FACTOR_BOUNDS(2);
opts.z_min            = ALTITUDE_BOUNDS(1);
opts.z_max            = ALTITUDE_BOUNDS(2);
opts.vertical_speed_min = VERTICAL_SPEED_BOUNDS(1);
opts.vertical_speed_max = VERTICAL_SPEED_BOUNDS(2);
opts.gamma_rate_max   = GAMMA_RATE_MAX;
opts.propulsion_power_max = PROPULSION_POWER_MAX;
opts.enforce_distance = PIN_VAVG;
opts.u_guess_mode     = 'sine_periodic';
opts.perturb_amp      = 0.50;

opts.w_rate = 0;
opts.alpha_rate_max = deg2rad(60);
opts.T_rate_max     = 500;
opts.multistart = FEATURES.multistart;
opts.multistart_modes = MULTISTART_MODES;
opts.multistart_amps = MULTISTART_AMPS;
opts.report_global_steady_power = P_steady_endur;
opts.report_speed_grid = base_endur.V_grid;
opts.report_power_grid = time_grid_endur;

if FINAL_OPTIMAL_SPEED_ONLY
    final_results = run_final_optimal_speed_sweep(p,base_endur,base_range,opts, ...
        AIRFRAME,COST,cost_labels(COST),airframe_paper_name(AIRFRAME), ...
        T_list,dt_target,P_steady_endur,JpM_steady_range);
    save(fullfile(here,'final_optimal_speed_sweep_results.mat'), ...
        'final_results','p','pi_result','T_list');
    fprintf('\nOptimal-speed sweep complete. Saved CSV, MAT, PI, sweep plots, and the best trajectory figure.\n');
    return
end

if ~SOLVE_ENDURANCE && ~SOLVE_RANGE
    error('Both SOLVE_ENDURANCE and SOLVE_RANGE are false; nothing to do.');
end

end_res = [];
if SOLVE_ENDURANCE
    end_res = sweep_ocp(T_list, dt_target, base_endur.V_op, p, opts, ...
        'endurance', USE_PARALLEL, N_WORKERS, USE_CONTINUATION);
    end_res = postprocess_savings(end_res, base_range, COST, ...
        P_steady_endur, JpM_steady_range);
else
    fprintf('\nEndurance-OCP sweep: SKIPPED (SOLVE_ENDURANCE = false)\n');
end

range_res = [];
if SOLVE_RANGE
    range_res = sweep_ocp(T_list, dt_target, base_range.V_op, p, opts, ...
        'range', USE_PARALLEL, N_WORKERS, USE_CONTINUATION);
    range_res = postprocess_savings(range_res, base_range, COST, ...
        P_steady_endur, JpM_steady_range);
else
    fprintf('\nRange-OCP sweep: SKIPPED (SOLVE_RANGE = false)\n');
end

%% ---------------- Paper labels ------------------------------
if strcmpi(COST,'fuel_rate')
    cost_label_paper = 'fuel-rate cost';
elseif strcmpi(COST,'energy')
    cost_label_paper = 'energy cost';
else
    cost_label_paper = COST;
end
airframe_paper = airframe_paper_name(AIRFRAME);

%% ---------------- Summary table -----------------------------
fprintf('\n======================== RESULTS ========================\n');

if SOLVE_ENDURANCE
    fprintf('--- Endurance-OCP solutions (objective = cost/time) ---\n');
    print_table(T_list, end_res);
end

if SOLVE_RANGE
    fprintf('\n--- Range-OCP solutions (objective = cost/distance) ---\n');
    print_table(T_list, range_res);
end

fprintf('------------------------------------------------------------\n');
fprintf('Steady baselines:  %s_endur = %.4f,  %s_range = %.4f\n', ...
    time_lbl, P_steady_endur, dist_lbl, JpM_steady_range);

i_best_endur = NaN;
i_best_range = NaN;

if SOLVE_ENDURANCE
    [~, i_best_endur] = min(end_res.P_avg);
    fprintf('\nBest endurance-OCP (min %s): T=%.2f s, %s=%.4f, saving=%.2f%% (fair: %.2f%%)\n', ...
        time_lbl, T_list(i_best_endur), time_lbl, end_res.P_avg(i_best_endur), ...
        end_res.sav_endur(i_best_endur), end_res.sav_endur_fair(i_best_endur));
end

if SOLVE_RANGE
    [~, i_best_range] = min(range_res.J_per_m);
    fprintf('Best range-OCP     (min %s):   T=%.2f s, %s=%.4f, saving=%.2f%% (fair: %.2f%%)\n', ...
        dist_lbl, T_list(i_best_range), dist_lbl, range_res.J_per_m(i_best_range), ...
        range_res.sav_range(i_best_range), range_res.sav_range_fair(i_best_range));
end

fprintf('=========================================================\n');

%% ---------------- ACC-ready plots ---------------------------
if SOLVE_ENDURANCE && ~isnan(i_best_endur)
    plot_best_solution(end_res, i_best_endur, T_list, p, ...
        AIRFRAME, COST, 'Endurance objective', cost_label_paper, airframe_paper);
end

if SOLVE_RANGE && ~isnan(i_best_range)
    plot_best_solution(range_res, i_best_range, T_list, p, ...
        AIRFRAME, COST, 'Range objective', cost_label_paper, airframe_paper);
end

plot_sweep(T_list, range_res, end_res, ...
           JpM_steady_range, P_steady_endur, ...
           AIRFRAME, COST, PIN_VAVG, ...
           cost_label_paper, airframe_paper);

%% ---------------- Save --------------------------------------
save(fullfile(here, sprintf('results_%s_%s.mat', AIRFRAME, COST)), ...
    'AIRFRAME','COST','T_list', ...
    'end_res','range_res', ...
    'SOLVE_ENDURANCE','SOLVE_RANGE', ...
    'P_steady_endur','JpM_steady_range','p','pi_result');

fprintf('\nSaved results_%s_%s.mat\n', AIRFRAME, COST);

%% ================= Local helper functions ===================

function res = sweep_ocp(T_list, dt_target, V_ref, p, opts_in, objective_mode, use_parallel, n_workers, use_continuation)
    n_T = numel(T_list);

    J_obj   = nan(n_T,1);
    J_int   = nan(n_T,1);
    P_avg   = nan(n_T,1);
    J_per_m = nan(n_T,1);
    V_avg   = nan(n_T,1);
    flag    = strings(n_T,1);
    sol_all = cell(n_T,1);

    opts = opts_in;
    opts.objective_mode = objective_mode;

    fprintf('\nSolving %s-OCP sweep...\n', upper(objective_mode));

    if nargin < 7 || isempty(use_parallel), use_parallel = false; end
    if nargin < 8, n_workers = []; end
    if nargin < 9 || isempty(use_continuation), use_continuation = false; end

    if use_continuation
        use_parallel = false;
    end

    if use_parallel
        try
            pool = gcp('nocreate');
            if isempty(pool)
                if isempty(n_workers)
                    pool = parpool('IdleTimeout', 120);
                else
                    pool = parpool(n_workers, 'IdleTimeout', 120);
                end
            end
            fprintf('  Parallel pool active: %d workers\n', pool.NumWorkers);
        catch ME
            warning('Parallel pool unavailable (%s). Falling back to serial.', ME.message);
            use_parallel = false;
        end
    end

    status_lines = strings(n_T,1);

    if use_parallel
        parfor ii = 1:n_T
            add_casadi_path(false);
            import casadi.*

            T = T_list(ii);
            N_i = max(80, min(4000, round(T / dt_target)));
            D_dummy = V_ref * T;

            try
                sol = solve_period_multistart(p,T,D_dummy,N_i,opts);
                flag(ii) = string(sol.flag);
                sol_all{ii} = sol;

                if sol.success
                    J_obj(ii)   = sol.J_obj;
                    J_int(ii)   = sol.J_int;
                    P_avg(ii)   = sol.P_avg;
                    J_per_m(ii) = sol.J_per_m;
                    V_avg(ii)   = sol.V_avg;

                    status_lines(ii) = sprintf('[%2d/%2d] T=%6.2f  OK  P_avg=%.4f  J/m=%.4f  Vavg=%.2f  (%s)', ...
                        ii, n_T, T, sol.P_avg, sol.J_per_m, sol.V_avg, sol.flag);
                else
                    status_lines(ii) = sprintf('[%2d/%2d] T=%6.2f  FAIL (%s)', ii, n_T, T, sol.flag);
                end
            catch ME
                status_lines(ii) = sprintf('[%2d/%2d] T=%6.2f  ERROR: %s', ii, n_T, T, ME.message);
            end
        end
    else
        sol_prev = [];

        for ii = 1:n_T
            T = T_list(ii);
            N_i = max(80, min(4000, round(T / dt_target)));
            D_dummy = V_ref * T;

            fprintf('[%2d/%2d]  T = %6.2f s  (N=%d, dt=%.3f) ... ', ...
                ii, n_T, T, N_i, T/N_i);

            try
                if use_continuation && ~isempty(sol_prev) && isfield(sol_prev,'success') && sol_prev.success
                    opts.init_sol = sol_prev;
                else
                    if isfield(opts,'init_sol')
                        opts = rmfield(opts,'init_sol');
                    end
                end

                sol = solve_period_multistart(p,T,D_dummy,N_i,opts);
                flag(ii) = string(sol.flag);
                sol_all{ii} = sol;

                if sol.success
                    J_obj(ii)   = sol.J_obj;
                    J_int(ii)   = sol.J_int;
                    P_avg(ii)   = sol.P_avg;
                    J_per_m(ii) = sol.J_per_m;
                    V_avg(ii)   = sol.V_avg;

                    fprintf('OK  P_avg=%.4f  J/m=%.4f  V_avg=%.2f  (%s)\n', ...
                        sol.P_avg, sol.J_per_m, sol.V_avg, sol.flag);

                    if isfield(opts,'report_global_steady_power')
                        saving_global=100*(opts.report_global_steady_power-sol.P_avg)/opts.report_global_steady_power;
                        [~,ir]=min(abs(opts.report_speed_grid-sol.V_avg));
                        P_same=opts.report_power_grid(ir);
                        saving_same=100*(P_same-sol.P_avg)/P_same;
                        gam_deg=rad2deg(sol.x(:,3));alpha_deg=rad2deg(sol.alpha);
                        vz=sol.x(:,4).*sin(sol.x(:,3));
                        fprintf('          savings: global=%+.4f%% | same-speed=%+.4f%% | steady(same V)=%.4f W\n', ...
                            saving_global,saving_same,P_same);
                        fprintf('          trajectory: V=[%.3f, %.3f] m/s | gamma=[%.3f, %.3f] deg | alpha=[%.3f, %.3f] deg\n', ...
                            min(sol.x(:,4)),max(sol.x(:,4)),min(gam_deg),max(gam_deg),min(alpha_deg),max(alpha_deg));
                        fprintf('          constraints: n=[%.3f, %.3f] | Vz=[%.3f, %.3f] m/s | thrust=[%.3f, %.3f] N | Pmax=%.3f W\n', ...
                            min(sol.load_factor),max(sol.load_factor),min(vz),max(vz), ...
                            min(sol.T_thrust),max(sol.T_thrust),max(sol.P_elec));
                    end

                    sol_prev = sol;
                else
                    fprintf('FAIL (%s)\n', sol.flag);
                end
            catch ME
                fprintf('ERROR: %s\n', ME.message);
            end
        end
    end

    if use_parallel
        for ii = 1:n_T
            if strlength(status_lines(ii)) > 0
                fprintf('%s\n', status_lines(ii));
            end
        end
    end

    res = struct();
    res.J_obj   = J_obj;
    res.J_int   = J_int;
    res.P_avg   = P_avg;
    res.J_per_m = J_per_m;
    res.V_avg   = V_avg;
    res.flag    = flag;
    res.sol_all = sol_all;
end

function best=solve_period_multistart(p,T,D,N,opts)
% Run configured initial guesses and retain the lowest-power feasible result.
if ~isfield(opts,'multistart')||~opts.multistart
    best=ocp_casadi_fixed_T(p,T,D,N,opts);return
end
modes=opts.multistart_modes;best=[];best_trial=[];screen_N=min(N,120);
for j=1:numel(modes)
    trial=opts;label=string(modes{j});trial.perturb_amp=.50;
    if startsWith(label,"long_glide_")
        trial.u_guess_mode='long_glide';
        trial.climb_duty=str2double(extractAfter(label,"long_glide_"))/100;
        trial.climb_thrust_fraction=1;
    else
        trial.u_guess_mode=char(label);
    end
    fprintf('\n          trying initial guess %-16s ... ',label);
    try
        candidate=ocp_casadi_fixed_T(p,T,D,screen_N,trial);
        if candidate.success
            fprintf('OK, P_avg=%.5f W\n',candidate.P_avg);
            % Treat sub-0.01 W differences as a numerical tie. In a tie,
            % retain the earlier, smoother candidate rather than switching
            % full-resolution branches because of coarse-mesh solver noise.
            if isempty(best)||candidate.P_avg<best.P_avg-0.01
                best=candidate;best.initial_guess_used=label;best_trial=trial;
            end
        else
            fprintf('FAILED (%s)\n',candidate.flag);
        end
    catch ME
        fprintf('ERROR: %s\n',ME.message);
    end
end
if isempty(best),error('All initial guesses failed for T=%.3f s.',T);end
selected_label=best.initial_guess_used;
fprintf('          selected screening guess: %s (N=%d, P_avg=%.5f W)\n',selected_label,screen_N,best.P_avg);
if N>screen_N
    fprintf('          refining selected guess at full N=%d ... ',N);
    best=ocp_casadi_fixed_T(p,T,D,N,best_trial);
    if ~best.success,error('Full-resolution solve failed for T=%.3f s (%s).',T,best.flag);end
    best.initial_guess_used=selected_label;
    fprintf('OK, P_avg=%.5f W\n',best.P_avg);
end
end

function res = postprocess_savings(res, base_range, cost_mode, P_steady_endur, JpM_steady_range)
    n_T = numel(res.P_avg);

    res.sav_endur = 100 * (P_steady_endur - res.P_avg) / P_steady_endur;
    res.sav_range = 100 * (JpM_steady_range - res.J_per_m) / JpM_steady_range;

    [time_grid, dist_grid] = cost_grids(base_range, cost_mode);

    res.sav_endur_fair = nan(n_T,1);
    res.sav_range_fair = nan(n_T,1);

    for ii = 1:n_T
        if ~isnan(res.V_avg(ii))
            [~, iV] = min(abs(base_range.V_grid - res.V_avg(ii)));
            P_st = time_grid(iV);
            J_st = dist_grid(iV);

            res.sav_endur_fair(ii) = 100 * (P_st - res.P_avg(ii)) / P_st;
            res.sav_range_fair(ii) = 100 * (J_st - res.J_per_m(ii)) / J_st;
        end
    end
end

function [time_grid, dist_grid] = cost_grids(base, cost_mode)
    switch lower(cost_mode)
        case 'energy'
            time_grid = base.P_elec_grid;
            dist_grid = base.E_per_m_grid;
        case 'fuel_rate'
            time_grid = base.Fdot_grid;
            dist_grid = base.F_per_m_grid;
        otherwise
            error('cost_grids: unknown cost_mode "%s"', cost_mode);
    end
end

function [time_lbl, dist_lbl] = cost_labels(cost_mode)
    switch lower(cost_mode)
        case 'energy'
            time_lbl = 'P [W]';
            dist_lbl = 'J/m [J/m]';
        case 'fuel_rate'
            time_lbl = 'sigma*T [N]';
            dist_lbl = 'sigma*T/V [N*s/m]';
        otherwise
            error('cost_labels: unknown cost_mode "%s"', cost_mode);
    end
end

function print_table(T_list, res)
    fprintf('%6s %6s %10s %10s %8s %8s %8s %8s\n', ...
        'T[s]', 'V_avg', 'P_avg', 'J/m', 'endur%', 'range%', 'fair_e%', 'fair_r%');
    fprintf('%6s %6s %10s %10s %8s %8s %8s %8s\n', ...
        '', 'm/s', '(cost/s)', '(cost/m)', 'vs best', 'vs best', '@sameV', '@sameV');

    for ii = 1:numel(T_list)
        if ~isnan(res.P_avg(ii))
            fprintf('%6.2f %6.2f %10.4f %10.4f %+8.2f %+8.2f %+8.2f %+8.2f\n', ...
                T_list(ii), res.V_avg(ii), res.P_avg(ii), res.J_per_m(ii), ...
                res.sav_endur(ii), res.sav_range(ii), ...
                res.sav_endur_fair(ii), res.sav_range_fair(ii));
        else
            fprintf('%6.2f %6s %10s %10s %8s %8s %8s %8s\n', ...
                T_list(ii), '---', '---', '---', '---', '---', '---', '---');
        end
    end
end

function plot_best_solution(res, idx, T_list, p, AIRFRAME, COST, label_str, cost_label_paper, airframe_paper)
%PLOT_BEST_SOLUTION Detailed trajectory report and machine-readable data.
% No overall title, no subplot titles. LaTeX math axis labels.

if nargin < 8 || isempty(cost_label_paper), cost_label_paper = COST; end %#ok<NASGU>
if nargin < 9 || isempty(airframe_paper), airframe_paper = AIRFRAME; end

if isempty(res) || isempty(res.sol_all) || ...
        idx < 1 || idx > numel(res.sol_all) || isempty(res.sol_all{idx})
    fprintf('plot_best_solution: no stored solution for %s.\n', label_str);
    return;
end

s = res.sol_all{idx};

if ~isfield(s,'success') || ~s.success
    fprintf('plot_best_solution: solution not successful for %s.\n', label_str);
    return;
end

tau  = T_list(idx);
t    = s.t(:);
Z    = s.x(:,2);
gam  = rad2deg(s.x(:,3));
V    = s.x(:,4);
X    = s.x(:,1);
tu   = t(1:end-1);
alph = rad2deg(s.u(:,1));
Thr  = s.u(:,2);
Vavg = s.V_avg;
if isfield(s,'load_factor')
    nload = s.load_factor;
else
    nload = nan(size(t));
end
Vz = V .* sin(deg2rad(gam));
[Pprop, eta_total_hist] = propulsion_power_model(s.T_thrust, V, p);
Ecum = cumtrapz(t, Pprop);

NAVY  = [0.118 0.153 0.380];
MID   = [0.290 0.435 0.647];
AMBER = [0.910 0.627 0.125];
RED   = [0.753 0.224 0.169];
GREY  = [0.420 0.498 0.600];

LW = 1.6;
LW_REF = 0.9;
FS_AX = 11;
FS_TK = 10;
FS_PANEL = 11;
FIG_W = 17;
FIG_H = 22;

fig = figure('Color','w', ...
    'Units','centimeters', 'Position',[2 2 FIG_W FIG_H], ...
    'PaperUnits','centimeters', 'PaperSize',[FIG_W FIG_H], ...
    'PaperPosition',[0 0 FIG_W FIG_H]);

set(fig, 'DefaultAxesFontName','Times New Roman', ...
         'DefaultAxesFontSize',FS_TK, ...
         'DefaultTextFontName','Times New Roman', ...
         'DefaultTextInterpreter','latex', ...
         'DefaultLegendInterpreter','latex', ...
         'DefaultAxesTickLabelInterpreter','latex');

tiledlayout(5,2,'TileSpacing','compact','Padding','compact');

    function ax = new_ax()
        ax = nexttile;
        set(ax, 'Box','on', ...
                'GridColor',[0.85 0.85 0.85], ...
                'GridAlpha',1, ...
                'GridLineStyle','-', ...
                'GridLineWidth',0.4, ...
                'TickLength',[0.015 0.015], ...
                'FontSize',FS_TK, ...
                'TickLabelInterpreter','latex');
        grid(ax,'on');
        hold(ax,'on');
    end

    function add_panel(ax, panel_str)
        text(ax, 0.025, 0.95, panel_str, ...
            'Units','normalized', ...
            'HorizontalAlignment','left', ...
            'VerticalAlignment','top', ...
            'FontSize',FS_PANEL, ...
            'FontWeight','bold', ...
            'Interpreter','latex', ...
            'BackgroundColor','w', ...
            'Margin',1);
    end

ax1 = new_ax();
plot(ax1, t, Z, 'Color',NAVY, 'LineWidth',LW);
yline(ax1, p.h_cruise, '--', 'Color',GREY, 'LineWidth',LW_REF);
xlabel(ax1,'$t~[\mathrm{s}]$','FontSize',FS_AX);
ylabel(ax1,'$Z~[\mathrm{m}]$','FontSize',FS_AX);
add_panel(ax1,'(a)');

ax2 = new_ax();
plot(ax2, t, V, 'Color',MID, 'LineWidth',LW);
yline(ax2, Vavg, '--', 'Color',GREY, 'LineWidth',LW_REF);
xlabel(ax2,'$t~[\mathrm{s}]$','FontSize',FS_AX);
ylabel(ax2,'$V~[\mathrm{m/s}]$','FontSize',FS_AX);
add_panel(ax2,'(b)');

ax3 = new_ax();
plot(ax3, t, gam, 'Color',NAVY, 'LineWidth',LW);
yline(ax3, 0, '-', 'Color',GREY, 'LineWidth',LW_REF);
xlabel(ax3,'$t~[\mathrm{s}]$','FontSize',FS_AX);
ylabel(ax3,'$\gamma~[\mathrm{deg}]$','FontSize',FS_AX);
add_panel(ax3,'(c)');

ax4 = new_ax();
plot(ax4, X, Z, 'Color',MID, 'LineWidth',LW);
plot(ax4, X(1), Z(1), 'o', 'Color',NAVY, ...
    'MarkerSize',6, 'MarkerFaceColor',NAVY);
plot(ax4, X(end), Z(end), 's', 'Color',AMBER, ...
    'MarkerSize',6, 'MarkerFaceColor',AMBER);
xlabel(ax4,'$X~[\mathrm{m}]$','FontSize',FS_AX);
ylabel(ax4,'$Z~[\mathrm{m}]$','FontSize',FS_AX);
add_panel(ax4,'(d)');

ax5 = new_ax();
stairs(ax5, tu, alph, 'Color',NAVY, 'LineWidth',LW);
yline(ax5, rad2deg(p.alpha_max), '--', 'Color',RED, 'LineWidth',LW_REF);
yline(ax5, rad2deg(p.alpha_min), '--', 'Color',RED, 'LineWidth',LW_REF);
xlabel(ax5,'$t~[\mathrm{s}]$','FontSize',FS_AX);
ylabel(ax5,'$\alpha~[\mathrm{deg}]$','FontSize',FS_AX);
add_panel(ax5,'(e)');

ax6 = new_ax();
stairs(ax6, tu, Thr, 'Color',AMBER, 'LineWidth',LW);
yline(ax6, p.T_max, '--', 'Color',RED, 'LineWidth',LW_REF);
yline(ax6, 0, '-', 'Color',GREY, 'LineWidth',LW_REF*0.7);
xlabel(ax6,'$t~[\mathrm{s}]$','FontSize',FS_AX);
ylabel(ax6,'$T~[\mathrm{N}]$','FontSize',FS_AX);
ylim(ax6,[-0.05*p.T_max, 1.15*p.T_max]);
add_panel(ax6,'(f)');

ax7 = new_ax();
plot(ax7, t, nload, 'Color',MID, 'LineWidth',LW);
yline(ax7, 0.5, '--', 'Color',RED, 'LineWidth',LW_REF);
yline(ax7, 2.0, '--', 'Color',RED, 'LineWidth',LW_REF);
xlabel(ax7,'$t~[\mathrm{s}]$','FontSize',FS_AX);
ylabel(ax7,'$n=L/(mg)$','FontSize',FS_AX);
add_panel(ax7,'(g)');

ax8 = new_ax();
plot(ax8, t, Vz, 'Color',NAVY, 'LineWidth',LW);
yline(ax8, -5, '--', 'Color',RED, 'LineWidth',LW_REF);
yline(ax8, 8, '--', 'Color',RED, 'LineWidth',LW_REF);
xlabel(ax8,'$t~[\mathrm{s}]$','FontSize',FS_AX);
ylabel(ax8,'$\dot Z~[\mathrm{m/s}]$','FontSize',FS_AX);
add_panel(ax8,'(h)');

ax9 = new_ax();
plot(ax9, t, Pprop, 'Color',AMBER, 'LineWidth',LW);
yline(ax9, 900, '--', 'Color',RED, 'LineWidth',LW_REF);
xlabel(ax9,'$t~[\mathrm{s}]$','FontSize',FS_AX);
ylabel(ax9,'$P_{\mathrm{elec}}~[\mathrm{W}]$','FontSize',FS_AX);
add_panel(ax9,'(i)');

ax10 = new_ax();
plot(ax10, t, Ecum, 'Color',MID, 'LineWidth',LW);
xlabel(ax10,'$t~[\mathrm{s}]$','FontSize',FS_AX);
ylabel(ax10,'$E~[\mathrm{J}]$','FontSize',FS_AX);
add_panel(ax10,'(j)');

safe = @(str) regexprep(lower(str),'[^a-z0-9]+','_');
fname = sprintf('traj_%s_%s_%s_tau%.0f', ...
    safe(airframe_paper), safe(COST), safe(label_str), round(tau));

exportgraphics(fig, [fname '.png'], 'Resolution',300, 'BackgroundColor','white');
exportgraphics(fig, [fname '.pdf'], 'ContentType','vector', 'BackgroundColor','white');

data_table = table(t, X, Z, gam, V, s.alpha, s.T_thrust, nload, Vz, Pprop, Ecum, ...
    'VariableNames', {'time_s','X_m','Z_m','gamma_deg','V_mps','alpha_rad', ...
    'thrust_N','load_factor','vertical_speed_mps','electrical_power_W','cumulative_energy_J'});
writetable(table(t, eta_total_hist, 'VariableNames', {'time_s','propulsion_efficiency'}), ...
    [fname '_efficiency.csv']);
writetable(data_table, [fname '.csv']);

fprintf('  Saved: %s.png  |  %s.pdf  |  %s.csv\n', fname, fname, fname);
end

function plot_sweep(T_list, range_res, end_res, ...
                    JpM_steady_range, P_steady_endur, ...
                    AIRFRAME, COST, PIN_VAVG, ...
                    cost_label_paper, airframe_paper) %#ok<INUSD>
%PLOT_SWEEP ACC-ready 1x2 figure: cost vs tau and savings vs tau.
% No title. LaTeX math axis labels.

if nargin < 9 || isempty(cost_label_paper), cost_label_paper = COST; end %#ok<NASGU>
if nargin < 10 || isempty(airframe_paper), airframe_paper = AIRFRAME; end

NAVY  = [0.118 0.153 0.380];
AMBER = [0.910 0.627 0.125];
GREY  = [0.420 0.498 0.600];
RED   = [0.753 0.224 0.169];

LW = 1.6;
LW_REF = 0.9;
MS = 5;
FS_AX = 11;
FS_TK = 10;
FS_PANEL = 11;
FIG_W = 17;
FIG_H = 7;

fig = figure('Color','w', ...
    'Units','centimeters', 'Position',[2 2 FIG_W FIG_H], ...
    'PaperUnits','centimeters', 'PaperSize',[FIG_W FIG_H], ...
    'PaperPosition',[0 0 FIG_W FIG_H]);

set(fig, 'DefaultAxesFontName','Times New Roman', ...
         'DefaultAxesFontSize',FS_TK, ...
         'DefaultTextFontName','Times New Roman', ...
         'DefaultTextInterpreter','latex', ...
         'DefaultLegendInterpreter','latex', ...
         'DefaultAxesTickLabelInterpreter','latex');

tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

    function ax = new_ax()
        ax = nexttile;
        set(ax, 'Box','on', ...
                'GridColor',[0.85 0.85 0.85], ...
                'GridAlpha',1, ...
                'GridLineStyle','-', ...
                'GridLineWidth',0.4, ...
                'TickLength',[0.015 0.015], ...
                'FontSize',FS_TK, ...
                'TickLabelInterpreter','latex');
        grid(ax,'on');
        hold(ax,'on');
    end

    function add_panel(ax, panel_str)
        text(ax, 0.03, 0.95, panel_str, ...
            'Units','normalized', ...
            'HorizontalAlignment','left', ...
            'VerticalAlignment','top', ...
            'FontSize',FS_PANEL, ...
            'FontWeight','bold', ...
            'Interpreter','latex', ...
            'BackgroundColor','w', ...
            'Margin',1);
    end

switch lower(COST)
    case 'fuel_rate'
        ylab_cost = '$\sigma T/V~[\mathrm{N\,s/m}]$';
    case 'energy'
        ylab_cost = '$E/X~[\mathrm{J/m}]$';
    otherwise
        ylab_cost = sprintf('$%s$', COST);
end

ax1 = new_ax();
hL = gobjects(0,1);
hN = strings(0,1);

if ~isempty(range_res) && any(~isnan(range_res.J_per_m))
    h = plot(ax1, T_list, range_res.J_per_m, '-o', ...
        'Color',NAVY, 'MarkerFaceColor',NAVY, ...
        'MarkerSize',MS, 'LineWidth',LW);
    hL(end+1) = h;
    hN(end+1) = "Range OCP";
end

yline(ax1, JpM_steady_range, '--', ...
    'Color',GREY, ...
    'LineWidth',LW_REF, ...
    'Label','steady', ...
    'LabelHorizontalAlignment','left', ...
    'Interpreter','latex', ...
    'FontSize',8);

if ~isempty(range_res) && any(~isnan(range_res.J_per_m))
    [~, ib] = min(range_res.J_per_m);
    plot(ax1, T_list(ib), range_res.J_per_m(ib), 'p', ...
        'Color',RED, 'MarkerFaceColor',RED, 'MarkerSize',8);
end

xlabel(ax1,'$\tau~[\mathrm{s}]$','FontSize',FS_AX);
ylabel(ax1, ylab_cost, 'FontSize',FS_AX);
if ~isempty(hL)
    legend(ax1, hL, cellstr(hN), 'Location','best', 'FontSize',8);
end
add_panel(ax1,'(a)');

ax2 = new_ax();
hL = gobjects(0,1);
hN = strings(0,1);

if ~isempty(range_res) && any(~isnan(range_res.sav_range_fair))
    h = plot(ax2, T_list, range_res.sav_range_fair, '-o', ...
        'Color',NAVY, 'MarkerFaceColor',NAVY, ...
        'MarkerSize',MS, 'LineWidth',LW);
    hL(end+1) = h;
    hN(end+1) = "Range (fair)";
end

if ~isempty(end_res) && any(~isnan(end_res.sav_endur_fair))
    h = plot(ax2, T_list, end_res.sav_endur_fair, '-s', ...
        'Color',AMBER, 'MarkerFaceColor',AMBER, ...
        'MarkerSize',MS, 'LineWidth',LW);
    hL(end+1) = h;
    hN(end+1) = "Endurance (fair)";
end

yline(ax2, 0, '-', 'Color',GREY, 'LineWidth',LW_REF*0.8);

xlabel(ax2,'$\tau~[\mathrm{s}]$','FontSize',FS_AX);
ylabel(ax2,'savings vs.\ steady $[\%]$','FontSize',FS_AX);
if ~isempty(hL)
    legend(ax2, hL, cellstr(hN), 'Location','best', 'FontSize',8);
end
add_panel(ax2,'(b)');

safe = @(str) regexprep(lower(str),'[^a-z0-9]+','_');
suffix = '';
if PIN_VAVG
    suffix = '_pinV';
end

fname = sprintf('sweep_%s_%s%s', safe(airframe_paper), safe(COST), suffix);

exportgraphics(fig, [fname '.png'], 'Resolution',300, 'BackgroundColor','white');
exportgraphics(fig, [fname '.pdf'], 'ContentType','vector', 'BackgroundColor','white');

fprintf('  Saved: %s.png  |  %s.pdf\n', fname, fname);
end

function name = airframe_paper_name(AIRFRAME)
    switch lower(AIRFRAME)
        case 'stallion'
            name = 'Stallion';
        case 'aerosonde'
            name = 'Aerosonde';
        case 'mk30'
            name = 'MK30';
        otherwise
            name = char(AIRFRAME);
    end
end

function results=run_final_optimal_speed_sweep(p,base,base_range,opts,AIRFRAME,COST,cost_label,airframe_paper,T_list,dt_target,Popt,Jrange)
% Sweep periodic flight only at the globally optimal steady-endurance speed.
Vopt=base.V_op;
opts.objective_mode='endurance';opts.enforce_distance=true;

fprintf('\n============================================================\n');
fprintf(' OPTIMAL-SPEED SWEEP: periodic %.5f m/s versus steady %.5f m/s\n',Vopt,Vopt);
fprintf(' Period order: [%s] s\n',strjoin(string(T_list),', '));
fprintf('============================================================\n');
opts.report_global_steady_power=Popt;
sweepopt=sweep_ocp(T_list,dt_target,Vopt,p,opts,'endurance',false,[],false);
sweepopt=postprocess_savings(sweepopt,base_range,COST,Popt,Jrange);
save_sweep_table('both_optimal_speed',T_list,sweepopt,Popt,'global');

period_s=T_list(:);
steady_speed_mps=repmat(Vopt,numel(T_list),1);
periodic_speed_mps=sweepopt.V_avg;
steady_power_W=repmat(Popt,numel(T_list),1);
periodic_power_W=sweepopt.P_avg;
saving_pct=100*(steady_power_W-periodic_power_W)./steady_power_W;
results=table(period_s,steady_speed_mps,periodic_speed_mps,steady_power_W,periodic_power_W,saving_pct);
writetable(results,'final_optimal_speed_sweep_results.csv');disp(results);

plot_final_sweep(T_list,sweepopt.P_avg,Popt,saving_pct,'both_optimal_speed');

[~,ibest]=min(sweepopt.P_avg);
plot_best_solution(sweepopt,ibest,T_list,p,AIRFRAME,COST, ...
    'Both periodic and steady at optimal speed',cost_label,airframe_paper);
end

function save_sweep_table(tag,T_list,r,Psteady,mode)
if strcmp(mode,'same_speed'),saving=100*(Psteady-r.P_avg)/Psteady;else,saving=100*(Psteady-r.P_avg)/Psteady;end
tbl=table(T_list(:),r.V_avg,r.P_avg,repmat(Psteady,numel(T_list),1),saving,r.flag, ...
 'VariableNames',{'period_s','periodic_speed_mps','periodic_power_W','steady_power_W','saving_pct','solver_status'});
writetable(tbl,['final_sweep_' tag '.csv']);
fprintf('\nCompleted %s:\n',strrep(tag,'_',' '));disp(tbl);
end

function plot_final_sweep(T,Pperiodic,Psteady,saving,tag)
fig=figure('Color','w','Name',strrep(tag,'_',' '));tiledlayout(1,2,'TileSpacing','compact');
nexttile;plot(T,Pperiodic,'-o','LineWidth',1.5);hold on;yline(Psteady,'--','Steady');grid on;
xlabel('Period [s]');ylabel('Average electrical power [W]');
nexttile;plot(T,saving,'-o','LineWidth',1.5);yline(0,'-');grid on;
xlabel('Period [s]');ylabel('Power saving [%]');
exportgraphics(fig,['final_sweep_' tag '.png'],'Resolution',300,'BackgroundColor','white');
exportgraphics(fig,['final_sweep_' tag '.pdf'],'ContentType','vector','BackgroundColor','white');

% Separate saving-only figure for directly seeing the peak/dip with period.
fig2=figure('Color','w','Name',['Saving ' strrep(tag,'_',' ')]);
plot(T,saving,'-o','LineWidth',1.8,'MarkerFaceColor',[0.12 0.32 0.62]);grid on;hold on;
yline(0,'k-');[bestSaving,ib]=max(saving);plot(T(ib),bestSaving,'p','MarkerSize',12, ...
 'MarkerFaceColor',[0.85 0.45 0.10],'MarkerEdgeColor',[0.85 0.45 0.10]);
xlabel('Period [s]');ylabel('Electrical power saving [%]');
title(sprintf('Best: %.3f%% at T = %.1f s',bestSaving,T(ib)));
exportgraphics(fig2,['final_savings_' tag '.png'],'Resolution',300,'BackgroundColor','white');
exportgraphics(fig2,['final_savings_' tag '.pdf'],'ContentType','vector','BackgroundColor','white');
end
