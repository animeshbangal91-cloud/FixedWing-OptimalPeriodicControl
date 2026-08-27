%% compare_pi_variants.m
% Compare multiple PI formulations to diagnose sign differences.

clear; clc; close all;

AIRFRAME = 'stallion';
COST_MODE = 'fuel_rate';             % 'fuel_rate' or 'energy'
STATE_MODE = 'augmented3';           % 'reduced2' | 'augmented3' | 'full4'

PI_MODES = {'isoperimetric', 'running_cost', 'distance_density'};
TRIMS = {'endurance', 'range'};

switch lower(AIRFRAME)
    case 'stallion'
        p = aircraft_params();
    case 'aerosonde'
        p = aerosonde_params();
    case 'mk30'
        p = mk30_params();
    otherwise
        error('Unknown AIRFRAME: %s', AIRFRAME);
end

pi_opts = struct();
pi_opts.omega_min = 1e-3;
pi_opts.omega_max = 1e3;
pi_opts.n_omega   = 2000;
pi_opts.r_alpha   = 1e-8;
pi_opts.r_T       = 1e-8;
pi_opts.do_plot   = true;

fprintf('\n=== PI Variant Comparison ===\n');
fprintf('Airframe=%s, Cost=%s, StateMode=%s\n', upper(AIRFRAME), COST_MODE, STATE_MODE);
fprintf('omega range: [%.1e, %.1e], n=%d\n\n', ...
    pi_opts.omega_min, pi_opts.omega_max, pi_opts.n_omega);

fprintf('%10s  %18s  %14s  %10s  %8s\n', 'trim', 'pi_mode', 'lambda_min', 'T* [s]', 'neg?');
fprintf('%s\n', repmat('-',1,70));

for it = 1:numel(TRIMS)
    p.cruise_mode = TRIMS{it};
    base = steady_cruise(p, false, COST_MODE);

    for im = 1:numel(PI_MODES)
        mode = PI_MODES{im};
        lbl = sprintf('%s | %s', TRIMS{it}, mode);
        r = pi_test_core(p, base, COST_MODE, lbl, STATE_MODE, mode, pi_opts);
        fprintf('%10s  %18s  %14.4e  %10.4f  %8s\n', ...
            TRIMS{it}, mode, r.lam_worst, r.T_star, tf_str(r.lam_worst < 0));
    end
end

function s = tf_str(tf)
if tf, s = 'yes'; else, s = 'no'; end
end
