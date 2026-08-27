function sol = ocp_casadi_fixed_T(p, T_period, D_dist, N, opts)
%OCP_CASADI_FIXED_T  Periodic OCP at fixed period T (2D point-mass UAV).
%
% States:   x = [X; Z; gamma; V]
% Controls: u = [alpha; T]
%
% Cost (opts.cost_mode):
%   'energy'    -> L = (T*V)/eta_total
%   'fuel_rate' -> L = sigma*T
%
% Objective mode (opts.objective_mode):
%   'endurance' -> minimize total cost over the period:  J = ∫ L dt
%                 (equivalently minimizes average cost rate since T is fixed)
%   'range'     -> minimize cost per distance:           J = (∫ L dt) / (X(T)-X(0))
%
% Constraints:
%   periodicity: Z(T)=Z(0), gamma(T)=gamma(0), V(T)=V(0)
%   optional distance: X(T)-X(0) = D_dist (disable to let V_avg float)
%   bounds on alpha, thrust, altitude excursion, gamma, V, and load factor

if nargin < 4
    error('ocp_casadi_fixed_T:NotEnoughInputs', [ ...
        'This is a solver function, not the main script. Run ', ...
        'run_periodic_analysis instead, or call ', ...
        'ocp_casadi_fixed_T(p,T_period,D_dist,N,opts).']);
end

import casadi.*

if nargin < 5, opts = struct(); end
if ~isfield(opts,'print_level'), opts.print_level = 0; end
if ~isfield(opts,'max_iter'),    opts.max_iter    = 1500; end
if ~isfield(opts,'perturb_amp'), opts.perturb_amp = 0.30; end
if ~isfield(opts,'cost_mode'),   opts.cost_mode   = 'energy'; end
if ~isfield(opts,'enforce_distance'), opts.enforce_distance = true; end
if ~isfield(opts,'objective_mode'), opts.objective_mode = 'endurance'; end
% Optional continuation warm-start from a previous solution struct (sol.t, sol.x, sol.u)
if ~isfield(opts,'init_sol'), opts.init_sol = []; end
use_battery = strcmpi(opts.cost_mode,'energy') && isfield(p,'battery') && ~isempty(p.battery);

%% ---------------- Steady cruise warm start -------------------
base  = steady_cruise(p, false, opts.cost_mode);
V0_ss = base.V_op;
a0_ss = base.alpha_op;
T0_ss = base.T_op;

if ~isfield(opts,'x0_guess')
    opts.x0_guess = [0; p.h_cruise; 0; V0_ss];
end

if ~isfield(opts,'alpha_min'), opts.alpha_min = p.alpha_min; end
if ~isfield(opts,'alpha_max'), opts.alpha_max = p.alpha_max; end

if ~isfield(opts,'u_guess_mode'), opts.u_guess_mode = 'sine_periodic'; end
switch lower(opts.u_guess_mode)
    case 'steady'
        opts.u_guess_fn = @(t) [a0_ss; T0_ss];

    case 'single_pulse'
        % Pulse in thrust near end of period; keep alpha modest until pulse.
        pulse_start = 0.85 * T_period;
        opts.u_guess_fn = @(t) [ ...
            a0_ss * 0.6 + a0_ss * 0.4 * (t > pulse_start); ...
            double(t > pulse_start) * p.T_max];

    case 'sine_periodic'
        % Sinusoidal control kick around the steady trim. Steady level flight
        % is itself a KKT point of the periodic NLP, so a near-trim warm-start
        % will land IPOPT at the trim — but a *too-large* kick is also bad,
        % because the cost rises with amplitude and IPOPT then "improves" by
        % squashing the oscillation. We want a kick small enough to live in
        % the second-order regime where the PI test predicts a dip.
        % opts.perturb_amp controls the fraction of available alpha / thrust
        % headroom used for the swing (0.05 = small, 0.5 = aggressive).
        amp_frac = max(0.01, min(opts.perturb_amp, 0.7));
        a_head = max(0.05, min(opts.alpha_max - a0_ss, a0_ss - opts.alpha_min));
        a_amp  = amp_frac * a_head;
        T_head = max(0.05*max(p.T_max,1), min(p.T_max - T0_ss, T0_ss - p.T_min));
        T_amp  = amp_frac * T_head;
        omega  = 2*pi/T_period;
        opts.u_guess_fn = @(t) [ ...
            a0_ss + a_amp * sin(omega*t); ...
            T0_ss + T_amp * sin(omega*t + pi)];   % thrust 180deg out of phase

    otherwise
        if ~isfield(opts,'u_guess_fn')
            opts.u_guess_fn = @(t) [a0_ss; T0_ss];
        end
end

% If continuation seed is provided, override the guess function with resampled profiles
u_guess_from_seed = [];
x_guess_from_seed = [];
if ~isempty(opts.init_sol) && isstruct(opts.init_sol) && isfield(opts.init_sol,'t') && isfield(opts.init_sol,'x') && isfield(opts.init_sol,'u')
    try
        [x_guess_from_seed, u_guess_from_seed] = resample_seed_guess(opts.init_sol, T_period, N);
    catch
        x_guess_from_seed = [];
        u_guess_from_seed = [];
    end
end

% If no continuation seed and we're using a non-trivial control guess, build a
% dynamically-consistent state guess by forward-simulating the sine controls.
% Without this, the (X,Z,gamma,V) guess is incoherent with the (alpha,T) guess
% and IPOPT's first feasibility correction collapses everything to the steady
% trim (which is itself a KKT point of the periodic NLP).
if isempty(x_guess_from_seed) && ~strcmpi(opts.u_guess_mode,'steady')
    try
        x_guess_from_seed = simulate_warmstart(opts.u_guess_fn, ...
            opts.x0_guess, T_period, N, p);
    catch
        x_guess_from_seed = [];
    end
end

% Add a battery SOC column to four-state flight warm starts.
if use_battery
    if numel(opts.x0_guess) == 4
        opts.x0_guess = [opts.x0_guess(:); p.battery.soc0];
    end
    if ~isempty(x_guess_from_seed) && size(x_guess_from_seed,2) == 4
        x_guess_from_seed(:,5) = p.battery.soc0;
    end
end

%% ---------------- CasADi dynamics + running cost -------------
nx = 4 + double(use_battery); nu = 2;
x_sym = MX.sym('x', nx);
u_sym = MX.sym('u', nu);

xdot_flight = aircraft_dynamics(x_sym(1:4), u_sym, p);

switch lower(opts.cost_mode)
    case 'energy'
        Pbus_sym = u_sym(2) * x_sym(4) / p.eta_total;
        if use_battery
            [I_sym, Vterm_sym, L_sym, Ploss_sym] = ...
                battery_power_model(Pbus_sym, x_sym(5), p.battery);
            socdot_sym = -I_sym / (3600*p.battery.capacity_Ah);
            xdot_sym = [xdot_flight; socdot_sym];
            I_fun = Function('I_batt', {x_sym,u_sym}, {I_sym});
            Vterm_fun = Function('Vterm_batt', {x_sym,u_sym}, {Vterm_sym});
            Ploss_fun = Function('Ploss_batt', {x_sym,u_sym}, {Ploss_sym});
        else
            L_sym = Pbus_sym;
            xdot_sym = xdot_flight;
        end
    case 'fuel_rate'
        if ~isfield(p,'sigma') || isempty(p.sigma), p.sigma = 1.0; end
        L_sym = p.sigma * u_sym(2);
        xdot_sym = xdot_flight;
    otherwise
        error('Unknown cost_mode: %s', opts.cost_mode);
end
f_dyn = Function('f_dyn', {x_sym, u_sym}, {xdot_sym});
L_fun = Function('L_fun', {x_sym, u_sym}, {L_sym});

%% ---------------- Radau collocation setup -------------------
% Default: cubic Radau (d=3) is a common sweet spot.
if ~isfield(opts,'collocation_degree') || isempty(opts.collocation_degree)
    opts.collocation_degree = 3;
end
d = opts.collocation_degree;

dt = T_period / N;

% Collocation points (Radau)
tau_root = [0, collocation_points(d, 'radau')];

% Collocation coefficients
C = zeros(d+1, d+1);
D = zeros(d+1, 1);
B = zeros(d+1, 1);

for j = 1:(d+1)
    % Construct Lagrange polynomials to get the coefficients
    coeff = 1;
    for r = 1:(d+1)
        if r ~= j
            coeff = conv(coeff, [1, -tau_root(r)]);
            coeff = coeff / (tau_root(j) - tau_root(r));
        end
    end

    % Continuity equation coefficient
    D(j) = polyval(coeff, 1.0);

    % Derivative of Lagrange poly at collocation points
    pder = polyder(coeff);
    for r = 1:(d+1)
        C(j,r) = polyval(pder, tau_root(r));
    end

    % Quadrature coefficient
    pint = polyint(coeff);
    B(j) = polyval(pint, 1.0) - polyval(pint, 0.0);
end

%% ---------------- NLP build ---------------------------------
w = {}; w0 = []; lbw = []; ubw = [];
g = {}; lbg = []; ubg = [];
J = 0;

% Optional smoothing / actuator realism (helps avoid chatter)
% - Hard rate limits: |dalpha/dt| <= alpha_rate_max, |dT/dt| <= T_rate_max
% - Soft penalty: w_rate * sum( (Δalpha)^2 + (ΔT/Tmax)^2 )
if ~isfield(opts,'alpha_rate_max'), opts.alpha_rate_max = inf; end   % [rad/s]
if ~isfield(opts,'T_rate_max'),     opts.T_rate_max     = inf; end   % [N/s]
if ~isfield(opts,'w_rate'),         opts.w_rate         = 0.0; end
J_rate = 0;
U_prev = [];

% Bounds
if ~isfield(opts,'z_excursion_max'), opts.z_excursion_max = inf; end
if ~isfield(opts,'gamma_max'), opts.gamma_max = inf; end
if ~isfield(opts,'load_factor_min'), opts.load_factor_min = -inf; end
if ~isfield(opts,'load_factor_max'), opts.load_factor_max =  inf; end
if ~isfield(opts,'z_min'), opts.z_min = -inf; end
if ~isfield(opts,'z_max'), opts.z_max = inf; end
if ~isfield(opts,'vertical_speed_min'), opts.vertical_speed_min = -inf; end
if ~isfield(opts,'vertical_speed_max'), opts.vertical_speed_max = inf; end
if ~isfield(opts,'gamma_rate_max'), opts.gamma_rate_max = inf; end
if ~isfield(opts,'propulsion_power_max'), opts.propulsion_power_max = inf; end

z_lower = max(p.h_cruise - opts.z_excursion_max, opts.z_min);
z_upper = min(p.h_cruise + opts.z_excursion_max, opts.z_max);
xlb = [-inf; z_lower; -opts.gamma_max; p.V_min];
xub = [ inf; z_upper;  opts.gamma_max; p.V_max];
if use_battery
    xlb = [xlb; p.battery.soc_min];
    xub = [xub; p.battery.soc_max];
end
ulb = [opts.alpha_min; p.T_min];
uub = [opts.alpha_max; p.T_max];

% Initial state decision variable
X0 = MX.sym('X0', nx);
w = {w{:}, X0};
lbw = [lbw; 0; xlb(2:end)];
ubw = [ubw; 0; xub(2:end)];
if use_battery
    lbw(end) = p.battery.soc0;
    ubw(end) = p.battery.soc0;
end

% Z swing for warm-start. Scaled to a fraction of kinetic-equivalent altitude
% V0^2/(2g) so the kick is meaningful at any airframe scale (~10m for V=26 m/s
% with perturb_amp=0.3). Capped to stay inside z_excursion_max.
pert = opts.perturb_amp * V0_ss^2 / (2*p.g);
pert = max(2.0, min(pert, 0.8 * opts.z_excursion_max));
if ~isempty(x_guess_from_seed)
    % First-node guess comes from the simulated/continuation trajectory so it
    % is consistent with the rest of the warm-start.
    w0  = [w0; 0; x_guess_from_seed(1,2:end).'];
else
    x0w = opts.x0_guess(:);
    x0w(1) = 0;
    x0w(2) = x0w(2) + pert;
    w0 = [w0; x0w];
end

Xk = X0;

% Keep variable ordering for extraction
X_nodes = cell(N+1,1);
U_nodes = cell(N,1);
X_nodes{1} = X0;

% Decide whether to add the state-dependent T_max(V) path constraint.
% Skip if model is 'constant' (the static p.T_max already bounds U via ubw).
use_thrust_curve = isfield(p,'thrust_model') && ~strcmpi(p.thrust_model,'constant');

for k = 0:N-1
    Uk = MX.sym(['U_' num2str(k)], nu);
    w = {w{:}, Uk};
    lbw = [lbw; ulb];
    ubw = [ubw; uub];

    if ~isempty(u_guess_from_seed)
        w0 = [w0; u_guess_from_seed(k+1,:).'];
    else
        t_mid = (k + 0.5) * dt;
        w0 = [w0; opts.u_guess_fn(t_mid)];
    end
    U_nodes{k+1} = Uk;

    % State-dependent thrust ceiling: T(t) <= thrust_max(V(t))
    % Apply at the interval's left state (V = Xk(4)). For piecewise-constant U,
    % evaluating at one point per interval is sufficient and avoids constraint bloat.
    if use_thrust_curve
        Tcap_k = thrust_max(Xk(4), p);
        g   = {g{:}, Uk(2) - Tcap_k};
        lbg = [lbg; -inf];
        ubg = [ubg;  0];
    end

    % Rate penalty + optional rate-limit constraints (between piecewise-constant controls)
    if k >= 1
        dU = Uk - U_prev;
        dalpha = dU(1) / dt;
        dT     = dU(2) / dt;

        if isfinite(opts.alpha_rate_max)
            g = {g{:}, dalpha};
            lbg = [lbg; -opts.alpha_rate_max];
            ubg = [ubg;  opts.alpha_rate_max];
        end
        if isfinite(opts.T_rate_max)
            g = {g{:}, dT};
            lbg = [lbg; -opts.T_rate_max];
            ubg = [ubg;  opts.T_rate_max];
        end

        if opts.w_rate > 0
            J_rate = J_rate + opts.w_rate * (dU(1)^2 + (dU(2)/p.T_max)^2);
        end
    end
    U_prev = Uk;

    % Collocation states for this interval
    Xc = cell(d,1);
    for j = 1:d
        Xkj = MX.sym(sprintf('X_%d_%d', k, j), nx);
        Xc{j} = Xkj;
        w = {w{:}, Xkj};
        lbw = [lbw; xlb];
        ubw = [ubw; xub];

        tau_j = tau_root(j+1);
        if ~isempty(x_guess_from_seed)
            % Interpolate node guess linearly within interval for collocation points
            xk0 = x_guess_from_seed(k+1,:).';
            xk1 = x_guess_from_seed(k+2,:).';
            xj  = xk0 + tau_j * (xk1 - xk0);
            w0 = [w0; xj];
        else
            % Default: linear X, small Z wobble, constant gamma/V
            X_lin = opts.x0_guess(1) + V0_ss * (k + tau_j) * dt;
            Z_wob = opts.x0_guess(2) + pert * cos(2*pi*(k + tau_j)*dt / T_period);
            xjw = [X_lin; Z_wob; opts.x0_guess(3); opts.x0_guess(4)];
            if use_battery, xjw = [xjw; p.battery.soc0]; end
            w0 = [w0; xjw];
        end
    end

    % Collocation equations
    Xk_end = D(1) * Xk;
    for j = 1:d
        % Expression for state derivative at collocation point
        xp = C(1, j+1) * Xk;
        for r = 1:d
            xp = xp + C(r+1, j+1) * Xc{r};
        end

        % Collocation residual: dt * f(xc, u) - xp = 0
        fj = f_dyn(Xc{j}, Uk);
        g = {g{:}, dt * fj - xp};
        lbg = [lbg; zeros(nx,1)];
        ubg = [ubg; zeros(nx,1)];

        % Normal load factor n = L/W. Enforcing this at every collocation
        % point prevents the optimizer from using structurally or
        % aerodynamically excessive pull-up loads between mesh nodes.
        Vj = Xc{j}(4);
        alphaj = Uk(1);
        qj = 0.5 * p.rho * Vj^2;
        CLj = p.CL0 + p.CLa * alphaj;
        nj = qj * p.S * CLj / (p.m * p.g);
        if isfinite(opts.load_factor_min) || isfinite(opts.load_factor_max)
            g = {g{:}, nj};
            lbg = [lbg; opts.load_factor_min];
            ubg = [ubg; opts.load_factor_max];
        end

        % Kinematic and maneuver-rate envelope.
        vertical_speed_j = Vj*sin(Xc{j}(3));
        if isfinite(opts.vertical_speed_min) || isfinite(opts.vertical_speed_max)
            g = {g{:}, vertical_speed_j};
            lbg = [lbg; opts.vertical_speed_min];
            ubg = [ubg; opts.vertical_speed_max];
        end
        if isfinite(opts.gamma_rate_max)
            g = {g{:}, fj(3)};
            lbg = [lbg; -opts.gamma_rate_max];
            ubg = [ubg;  opts.gamma_rate_max];
        end

        % Electrical propulsion demand before optional battery losses.
        if isfinite(opts.propulsion_power_max)
            propulsion_power_j = Uk(2)*Vj/p.eta_total;
            g = {g{:}, propulsion_power_j};
            lbg = [lbg; 0];
            ubg = [ubg; opts.propulsion_power_max];
        end

        if use_battery
            Ij = I_fun(Xc{j}, Uk);
            Vtermj = Vterm_fun(Xc{j}, Uk);
            g = {g{:}, Ij, Vtermj};
            lbg = [lbg; 0; p.battery.n_series*p.battery.V_cell_cutoff];
            ubg = [ubg; p.battery.I_max; inf];
        end

        % Contribution to end state
        Xk_end = Xk_end + D(j+1) * Xc{j};

        % Quadrature contribution to integral cost
        J = J + (B(j+1) * dt) * L_fun(Xc{j}, Uk);
    end

    Xk1 = MX.sym(['X_' num2str(k+1)], nx);
    w = {w{:}, Xk1};
    lbw = [lbw; xlb];
    ubw = [ubw; xub];

    if ~isempty(x_guess_from_seed)
        w0 = [w0; x_guess_from_seed(k+2,:).'];
    else
        % Warm start: linear X, small Z wobble
        X_lin = opts.x0_guess(1) + V0_ss * (k+1)*dt;
        Z_wob = opts.x0_guess(2) + pert * cos(2*pi*(k+1)*dt / T_period);
        xnw = [X_lin; Z_wob; opts.x0_guess(3); opts.x0_guess(4)];
        if use_battery, xnw = [xnw; p.battery.soc0]; end
        w0 = [w0; xnw];
    end

    % Continuity (end of interval)
    g   = {g{:}, Xk1 - Xk_end};
    lbg = [lbg; zeros(nx,1)];
    ubg = [ubg; zeros(nx,1)];

    Xk = Xk1;
    X_nodes{k+2} = Xk1;
end

% Wraparound rate constraints/penalty to avoid a hidden discontinuity at period boundary
if N >= 2 && ~isempty(U_nodes{1}) && ~isempty(U_nodes{end})
    dU_wrap = U_nodes{1} - U_nodes{end};
    dalpha_wrap = dU_wrap(1) / dt;
    dT_wrap     = dU_wrap(2) / dt;

    if isfinite(opts.alpha_rate_max)
        g = {g{:}, dalpha_wrap};
        lbg = [lbg; -opts.alpha_rate_max];
        ubg = [ubg;  opts.alpha_rate_max];
    end
    if isfinite(opts.T_rate_max)
        g = {g{:}, dT_wrap};
        lbg = [lbg; -opts.T_rate_max];
        ubg = [ubg;  opts.T_rate_max];
    end
    if opts.w_rate > 0
        J_rate = J_rate + opts.w_rate * (dU_wrap(1)^2 + (dU_wrap(2)/p.T_max)^2);
    end
end

% Periodicity: Z, gamma, V
g = {g{:}, Xk(2) - X0(2)}; lbg = [lbg; 0]; ubg = [ubg; 0];
g = {g{:}, Xk(3) - X0(3)}; lbg = [lbg; 0]; ubg = [ubg; 0];
g = {g{:}, Xk(4) - X0(4)}; lbg = [lbg; 0]; ubg = [ubg; 0];

% Optional distance constraint
if opts.enforce_distance
    g = {g{:}, Xk(1) - X0(1) - D_dist};
    lbg = [lbg; 0]; ubg = [ubg; 0];
end

% Objective: endurance vs range
dist_expr = Xk(1) - X0(1);  % X(T) - X(0)
switch lower(opts.objective_mode)
    case 'endurance'
        J_obj = J + J_rate;
    case 'range'
        % Avoid division by ~0 (also encourages forward progress)
        J_obj = (J + J_rate) / (dist_expr + 1e-6);
    otherwise
        error('Unknown objective_mode: %s', opts.objective_mode);
end

%% ---------------- Solve -------------------------------------
w_vec = vertcat(w{:});
g_vec = vertcat(g{:});
prob  = struct('f', J_obj, 'x', w_vec, 'g', g_vec);

ipopt_opts = struct;
ipopt_opts.ipopt.print_level    = opts.print_level;
ipopt_opts.ipopt.max_iter       = opts.max_iter;
ipopt_opts.ipopt.tol            = 1e-8;
ipopt_opts.ipopt.acceptable_tol = 1e-6;
ipopt_opts.ipopt.mu_strategy    = 'adaptive';
ipopt_opts.print_time           = 0;

solver = nlpsol('solver','ipopt', prob, ipopt_opts);

% Optional warm-start diagnostic (set opts.debug_warmstart_cost = true).
% Prints J_obj at the warm-start vs at the solution -- useful for telling
% whether IPOPT starts above or below the steady-cruise cost, and whether
% the converged optimum is meaningfully different from steady.
if isfield(opts,'debug_warmstart_cost') && opts.debug_warmstart_cost
    f_eval = Function('f_eval', {w_vec}, {J_obj});
    J_warm = full(f_eval(w0));
    fprintf('    [WS] cost at warm-start = %.6f\n', J_warm);
end

res    = solver('x0',w0,'lbx',lbw,'ubx',ubw,'lbg',lbg,'ubg',ubg);

flag    = solver.stats().return_status;
success = strcmp(flag,'Solve_Succeeded') || strcmp(flag,'Solved_To_Acceptable_Level');

if isfield(opts,'debug_warmstart_cost') && opts.debug_warmstart_cost
    fprintf('    [WS] cost at solution    = %.6f  (iters=%d)\n', ...
        full(res.f), solver.stats().iter_count);
end

%% ---------------- Extract trajectory ------------------------
w_opt = full(res.x);

x_traj = zeros(N+1, nx);
u_traj = zeros(N,   nu);

offset = 0;
% Variable order is:
%   X0,
%   for k=0..N-1: Uk, Xc(k,1..d), X_{k+1}
x_traj(1,:) = w_opt(offset+1:offset+nx).'; offset = offset + nx;
for k = 1:N
    u_traj(k,:) = w_opt(offset+1:offset+nu).'; offset = offset + nu;
    offset = offset + d*nx; % skip collocation states
    x_traj(k+1,:) = w_opt(offset+1:offset+nx).'; offset = offset + nx;
end

t_traj = linspace(0, T_period, N+1).';

P_elec = zeros(N+1,1);
L_vec  = zeros(N+1,1);
D_vec  = zeros(N+1,1);
I_batt = nan(N+1,1);
V_batt = nan(N+1,1);
P_loss_batt = nan(N+1,1);
for k = 1:N+1
    if k <= N
        uk = u_traj(k,:).';
    else
        uk = u_traj(N,:).';
    end
    [~, aux]  = aircraft_dynamics(x_traj(k,1:4).', uk, p);
    if use_battery
        [I_batt(k), V_batt(k), P_elec(k), P_loss_batt(k)] = ...
            battery_power_model(aux.P_elec, x_traj(k,5), p.battery);
    else
        P_elec(k) = aux.P_elec;
    end
    L_vec(k)  = aux.L;
    D_vec(k)  = aux.D;
end

%% ---------------- Pack result -------------------------------
sol = struct();
sol.success = success;
sol.flag = flag;
sol.J_obj = full(res.f);      % objective value actually minimized

sol.D_actual = x_traj(end,1) - x_traj(1,1);
% Always report physical integrals/metrics (independent of objective_mode).
% With collocation, J is already an approximation of ∫L dt using Radau quadrature.
% Note: res.f is J_obj, not necessarily J_int for 'range'. We recompute a
% consistent J_int from the extracted node trajectory (left-endpoint rule).
J_int = 0.0;
for kk = 1:N
    xk = x_traj(kk,:).';
    uk = u_traj(kk,:).';
    if strcmpi(opts.cost_mode,'energy')
        Pbus_k = uk(2) * xk(4) / p.eta_total;
        if use_battery
            [~, ~, Lk] = battery_power_model(Pbus_k, xk(5), p.battery);
        else
            Lk = Pbus_k;
        end
    else
        if ~isfield(p,'sigma') || isempty(p.sigma), p.sigma = 1.0; end
        Lk = p.sigma * uk(2);
    end
    J_int = J_int + Lk * dt;
end
sol.J_int = J_int;
sol.J_per_m  = sol.J_int / max(sol.D_actual, 1e-6);
sol.P_avg    = sol.J_int / T_period;
sol.V_avg    = sol.D_actual / T_period;

sol.t = t_traj;
sol.x = x_traj;
sol.u = u_traj;          % [alpha, T]
% Hold last control across the period boundary so length matches sol.t (N+1).
sol.alpha    = [u_traj(:,1); u_traj(end,1)];
sol.T_thrust = [u_traj(:,2); u_traj(end,2)];
sol.P_elec = P_elec;
sol.L = L_vec;
sol.D = D_vec;
sol.load_factor = L_vec / (p.m * p.g);
if use_battery
    sol.soc = x_traj(:,5);
    sol.soc_used = sol.soc(1) - sol.soc(end);
    sol.I_batt = I_batt;
    sol.V_batt = V_batt;
    sol.P_loss_batt = P_loss_batt;
end
sol.T_period = T_period;
sol.D_dist = D_dist;
sol.cost_mode = opts.cost_mode;
sol.objective_mode = opts.objective_mode;

end
