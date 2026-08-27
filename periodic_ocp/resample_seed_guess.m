function [x_guess, u_guess] = resample_seed_guess(sol_prev, T_new, N_new)
%RESAMPLE_SEED_GUESS  Resample previous OCP solution onto new grid (N_new intervals).
%
% Inputs:
%   sol_prev : struct with .t (Np+1 x 1), .x (Np+1 x nx), .u (Np x nu)
%   T_new    : new period [s]
%   N_new    : new number of control intervals
%
% Outputs:
%   x_guess  : (N_new+1) x nx initial guess for state nodes
%   u_guess  : N_new     x nu initial guess for piecewise-constant controls
%
% State is interpolated linearly in normalized time tau = t/T.
% Control is held with previous-sample (zero-order hold) at interval midpoints.

    t_prev = sol_prev.t(:);
    x_prev = sol_prev.x;
    u_prev = sol_prev.u;
    if size(x_prev,1) ~= numel(t_prev)
        error('resample_seed_guess: seed x/t size mismatch');
    end

    % previous control time grid (zero-order hold on [t_k, t_{k+1}))
    tu_prev = t_prev(1:end-1);

    tau_prev_x = t_prev / t_prev(end);
    tau_prev_u = tu_prev / t_prev(end);

    t_new  = linspace(0, T_new, N_new+1).';
    tu_new = t_new(1:end-1) + 0.5*(T_new/N_new);
    tau_new_x = t_new  / T_new;
    tau_new_u = tu_new / T_new;

    x_guess = interp1(tau_prev_x, x_prev, tau_new_x, 'linear',  'extrap');
    u_guess = interp1(tau_prev_u, u_prev, tau_new_u, 'previous','extrap');
end
