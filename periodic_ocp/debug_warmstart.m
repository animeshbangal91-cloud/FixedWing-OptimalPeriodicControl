%% debug_warmstart.m
% Sweep perturb_amp at a single T close to the PI-predicted optimum and print
% (warm-start cost, solution cost, iterations) so we can see whether IPOPT
% is starting above the steady-cruise cost (and "improving" by going to
% steady) or below it (and finding a non-trivial periodic optimum).

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(here);
add_casadi_path();
import casadi.*

p = aircraft_params();
p.cruise_mode = 'range';
base = steady_cruise(p, false, 'energy');
fprintf('Steady (range): V=%.3f, P=%.4f W, J/m=%.4f, T=%.4f N, alpha=%.3f rad\n', ...
    base.V_op, base.T_op*base.V_op/p.eta_total, base.T_op/p.eta_total, ...
    base.T_op, base.alpha_op);

T_period = 11.0;
dt_target = 0.04;
N = round(T_period/dt_target);

opts = struct();
opts.print_level   = 0;
opts.max_iter      = 5000;
opts.cost_mode     = 'energy';
opts.objective_mode = 'range';
opts.z_excursion_max = 300;
opts.gamma_max       = deg2rad(10);
opts.alpha_min       = p.alpha_min;
opts.alpha_max       = p.alpha_max;
opts.enforce_distance = true;          % PIN_VAVG = true
opts.u_guess_mode    = 'sine_periodic';
opts.w_rate          = 0;
opts.alpha_rate_max  = deg2rad(60);
opts.T_rate_max      = 500;
opts.debug_warmstart_cost = true;

D_dist = base.V_op * T_period;

amps = [0.0, 0.02, 0.05, 0.10, 0.20, 0.40, 0.70];
fprintf('\nT = %.2f s, D = %.2f m\n', T_period, D_dist);
fprintf('amp_frac | J(warm)       J(soln)        P_avg     V_avg     iters  flag\n');
for a = amps
    opts.perturb_amp = a;
    sol = ocp_casadi_fixed_T(p, T_period, D_dist, N, opts);
    fprintf('  %.3f  | %12s  | P_avg=%.4f  J/m=%.4f  V_avg=%.3f  flag=%s\n', ...
        a, '(see [WS])', sol.P_avg, sol.J_per_m, sol.V_avg, sol.flag);
end
