function x_guess = simulate_warmstart(u_fn, x0, T_period, N, p)
%SIMULATE_WARMSTART  Forward-integrate the OCP dynamics under a control profile
% to generate a dynamically-consistent state warm-start for collocation.
%
% Inputs:
%   u_fn     : function handle u = u_fn(t) returning [alpha; T]
%   x0       : initial state [X; Z; gamma; V] (steady trim recommended)
%   T_period : period [s]
%   N        : number of intervals (warm-start grid size)
%   p        : aircraft params struct (passed to aircraft_dynamics)
%
% Output:
%   x_guess  : (N+1) x 4 array of state guesses on the uniform grid
%              t_k = (k-1) * T_period / N, k = 1..N+1
%
% Notes:
% - Uses a fixed-step RK4 integrator with substeps for accuracy.
% - The trajectory will *not* be exactly periodic (Z(T) ~= Z(0), etc.) but it
%   will be dynamically self-consistent, which is what the NLP needs as a warm
%   start so IPOPT's feasibility step doesn't collapse the guess to the trim.
% - Controls are clipped to physical limits ([alpha_min, alpha_max], [T_min, T_max]).
% - States are clipped to safe ranges (V > 0.5*V0, |gamma| < 60 deg) to keep
%   the guess physically meaningful even under large open-loop forcing.
% - Mean altitude is re-centered to x0(2) so the guess stays near cruise.

if nargin < 5
    error('simulate_warmstart: need u_fn, x0, T_period, N, p.');
end

% Control & state clipping helpers
alpha_lo = p.alpha_min;
alpha_hi = p.alpha_max;
T_lo     = p.T_min;
T_hi     = p.T_max;
V0       = x0(4);
V_lo     = max(p.V_min, 0.4 * V0);   % don't allow V to crash to zero
V_hi     = min(p.V_max, 1.8 * V0);
g_lim    = deg2rad(60);              % keep gamma in a safe band

clip_u = @(u) [ ...
    min(max(u(1), alpha_lo), alpha_hi); ...
    min(max(u(2), T_lo), T_hi)];
clip_x = @(x) [x(1); x(2); ...
    min(max(x(3), -g_lim), g_lim); ...
    min(max(x(4), V_lo), V_hi)];

dt = T_period / N;
n_sub = 4;                  % RK4 sub-steps per interval
h = dt / n_sub;

x = clip_x(x0(:));
x_guess = zeros(N+1, 4);
x_guess(1,:) = x.';

for k = 1:N
    t = (k-1) * dt;
    for s = 1:n_sub
        ts = t + (s-1)*h;
        u1 = clip_u(u_fn(ts));
        u2 = clip_u(u_fn(ts + 0.5*h));
        u3 = clip_u(u_fn(ts + 0.5*h));
        u4 = clip_u(u_fn(ts + h));

        k1 = aircraft_dynamics(x,            u1, p);
        k2 = aircraft_dynamics(x + 0.5*h*k1, u2, p);
        k3 = aircraft_dynamics(x + 0.5*h*k2, u3, p);
        k4 = aircraft_dynamics(x +     h*k3, u4, p);

        x = x + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
        x = clip_x(x);
    end
    x_guess(k+1,:) = x.';
end

% Re-center altitude around x0(2) so the guess stays near cruise altitude
% (otherwise long-period sine forcing can drift Z monotonically).
Z_mean = mean(x_guess(:,2));
x_guess(:,2) = x_guess(:,2) - (Z_mean - x0(2));

end

