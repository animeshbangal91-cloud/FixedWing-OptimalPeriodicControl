function pi_res = pi_test_core(p, base, cost_mode, plot_label, state_mode, pi_mode, pi_opts)
%PI_TEST_CORE  Frequency-domain PI(omega) test with selectable formulation.
%
% state_mode:
%   'reduced2'   -> [gamma; V]
%   'augmented3' -> [Z; gamma; V]
%   'full4'      -> [X; Z; gamma; V]
%
% pi_mode:
%   'running_cost'    -> Lcore = Lrun                (default form: H = L + lambda^T f)
%   'isoperimetric'   -> Lcore = Lrun - mu*Vground   (range-style test)
%   'distance_density'-> Lcore = Lrun/(Vground+eps)  (direct per-distance
%   density) ( for range)

if nargin < 3 || isempty(cost_mode),  cost_mode = 'energy'; end
if nargin < 4 || isempty(plot_label), plot_label = 'trim'; end
if nargin < 5 || isempty(state_mode), state_mode = 'reduced2'; end
if nargin < 6 || isempty(pi_mode),    pi_mode = 'running_cost'; end
if nargin < 7 || isempty(pi_opts),    pi_opts = struct(); end

% Options with defaults
r_alpha   = get_opt(pi_opts, 'r_alpha', 1e-6);
r_T       = get_opt(pi_opts, 'r_T', 1e-6);
omega_min = get_opt(pi_opts, 'omega_min', 1e-2);
omega_max = get_opt(pi_opts, 'omega_max', 1e2);
n_omega   = get_opt(pi_opts, 'n_omega', 1200);
v_eps     = get_opt(pi_opts, 'v_eps', 1e-3);
do_plot   = get_opt(pi_opts, 'do_plot', true);

if ~isfield(p,'sigma') || isempty(p.sigma), p.sigma = 1.0; end

V0 = base.V_op;
a0 = base.alpha_op;
T0 = base.T_op;

syms X Z gma V alph Thr real
syms m_sym g_sym rho_sym S_sym CL0_sym CLa_sym CD0_sym k_sym eta_sym sigma_sym mu_sym v_eps_sym real
syms a_bar T_bar ra rT real

q = sym(1)/2 * rho_sym * V^2;
CL = CL0_sym + CLa_sym*alph;
CD = CD0_sym + k_sym*CL^2;
Lift = q*S_sym*CL;
Drag = q*S_sym*CD;

Xdot   = V*cos(gma);
Zdot   = V*sin(gma);
gmadot = (Lift - m_sym*g_sym*cos(gma))/(m_sym*V);
Vdot   = (Thr - Drag)/m_sym - g_sym*sin(gma);

P_elec  = Thr*V/eta_sym;
Vground = V*cos(gma);

switch lower(cost_mode)
    case 'energy'
        Lrun = P_elec;
        mu = T0 / p.eta_total;
    case 'fuel_rate'
        Lrun = sigma_sym * Thr;
        mu = (p.sigma*T0) / V0;
    otherwise
        error('Unknown cost_mode: %s', cost_mode);
end

switch lower(pi_mode)
    case 'isoperimetric'
        Lcore = Lrun - mu_sym * Vground;
    case 'running_cost'
        Lcore = Lrun;
        mu = 0;
    case 'distance_density'
        Lcore = Lrun / (Vground + v_eps_sym);
        mu = 0;
    otherwise
        error('Unknown pi_mode: %s', pi_mode);
end

Liso = Lcore ...
    + sym(1)/2 * ra * (alph - a_bar)^2 ...
    + sym(1)/2 * rT * (Thr - T_bar)^2;

switch lower(state_mode)
    case 'reduced2'
        state = [gma; V];
        f = [gmadot; Vdot];
    case 'augmented3'
        state = [Z; gma; V];
        f = [Zdot; gmadot; Vdot];
    case 'full4'
        state = [X; Z; gma; V];
        f = [Xdot; Zdot; gmadot; Vdot];
    otherwise
        error('Unknown state_mode: %s', state_mode);
end
ctrl = [alph; Thr];
nx = numel(state);
nu = numel(ctrl);

lam_sym = sym('lam', [nx,1], 'real');
H = Liso + lam_sym.' * f;

fx  = jacobian(f, state);
fu  = jacobian(f, ctrl);
Hxx = jacobian(jacobian(H, state).', state);
Huu = jacobian(jacobian(H, ctrl ).', ctrl );
Hxu = jacobian(jacobian(H, state).', ctrl );
Hux = Hxu.';

subs_pairs = { ...
    m_sym,      p.m; ...
    g_sym,      p.g; ...
    rho_sym,    p.rho; ...
    S_sym,      p.S; ...
    CL0_sym,    p.CL0; ...
    CLa_sym,    p.CLa; ...
    CD0_sym,    p.CD0; ...
    k_sym,      p.k; ...
    eta_sym,    p.eta_total; ...
    sigma_sym,  p.sigma; ...
    mu_sym,     mu; ...
    v_eps_sym,  v_eps; ...
    a_bar,      a0; ...
    T_bar,      T0; ...
    ra,         r_alpha; ...
    rT,         r_T; ...
    X,          0; ...
    Z,          p.h_cruise; ...
    gma,        0; ...
    V,          V0; ...
    alph,       a0; ...
    Thr,        T0};

Lu = double(subs(jacobian(Liso, ctrl).', [subs_pairs{:,1}], [subs_pairs{:,2}]));
Bu = double(subs(fu,                    [subs_pairs{:,1}], [subs_pairs{:,2}]));
lam = -pinv(Bu.') * Lu;

lam_pairs = cell(nx,2);
for k = 1:nx
    lam_pairs{k,1} = lam_sym(k);
    lam_pairs{k,2} = lam(k);
end
subs_full = [subs_pairs; lam_pairs];

A = double(subs(fx,  [subs_full{:,1}], [subs_full{:,2}]));
B = double(subs(fu,  [subs_full{:,1}], [subs_full{:,2}]));
Hxx_n = double(subs(Hxx, [subs_full{:,1}], [subs_full{:,2}]));
Huu_n = double(subs(Huu, [subs_full{:,1}], [subs_full{:,2}]));
Hxu_n = double(subs(Hxu, [subs_full{:,1}], [subs_full{:,2}]));
Hux_n = double(subs(Hux, [subs_full{:,1}], [subs_full{:,2}]));

omega_vec = logspace(log10(omega_min), log10(omega_max), n_omega);
lam_min = nan(size(omega_vec));

I = eye(nx);
for i = 1:numel(omega_vec)
    w = omega_vec(i);
    G = (1j*w*I - A) \ B;
    Pi = (G')*Hxx_n*G + (G')*Hxu_n + Hux_n*G + Huu_n;
    Pi = 0.5*(Pi + Pi');
    ev = sort(real(eig(Pi)));
    lam_min(i) = ev(1);
end

[lam_worst, idx] = min(lam_min);
omega_star = omega_vec(idx);
T_star = 2*pi/omega_star;
profitable = lam_worst < -1e-9;
min_at_boundary = (idx == 1) || (idx == numel(omega_vec));

if do_plot
    figure('Name', sprintf('PI(omega) - %s', plot_label), 'Color','w','NumberTitle','off');
    semilogx(omega_vec, lam_min, 'LineWidth', 2); grid on;
    yline(0,'k-');
    xline(omega_star, ':', '\omega^*');
    xlabel('Frequency $\omega$ [rad/s]','Interpreter','latex');
    ylabel('$\lambda_{\min}(\Pi(\omega))$','Interpreter','latex');
    title(sprintf('Frequency-domain PI test (%s)', plot_label));
end

pi_res = struct();
pi_res.A = A; pi_res.B = B;
pi_res.Hxx = Hxx_n; pi_res.Huu = Huu_n; pi_res.Hxu = Hxu_n; pi_res.Hux = Hux_n;
pi_res.costates = lam;
pi_res.omega_vec = omega_vec;
pi_res.lam_min = lam_min;
pi_res.lam_worst = lam_worst;
pi_res.omega_star = omega_star;
pi_res.T_star = T_star;
pi_res.profitable = profitable;
pi_res.min_at_boundary = min_at_boundary;
pi_res.state_mode = state_mode;
pi_res.pi_mode = pi_mode;
pi_res.cost_mode = cost_mode;
pi_res.nx = nx;
pi_res.nu = nu;
pi_res.r_alpha = r_alpha;
pi_res.r_T = r_T;

fprintf('  PI mode: %s | state mode: %s (nx=%d, nu=%d)\n', pi_mode, state_mode, nx, nu);
fprintf('  Trim costates norm: %.4g\n', norm(lam));
fprintf('  Min lambda_min(Pi) = %.4e at omega = %.4g rad/s (T=%.4g s)\n', ...
    lam_worst, omega_star, T_star);
if min_at_boundary
    fprintf('  NOTE: PI minimum is on omega-grid boundary; no clear interior dip in scan.\n');
end
end

function val = get_opt(s, name, default_val)
if isfield(s, name) && ~isempty(s.(name))
    val = s.(name);
else
    val = default_val;
end
end

