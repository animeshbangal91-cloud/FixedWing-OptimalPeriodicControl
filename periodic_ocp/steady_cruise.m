function base = steady_cruise(p, do_plot, cost_mode)
%STEADY_CRUISE  Compute steady level cruise grid and pick best operating point.
%
% Outputs base struct with fields used by run_periodic_analysis.m and OCP warm-start:
%   V_grid, alpha_grid, T_grid
%   P_elec_grid, E_per_m_grid
%   i_op, V_op, alpha_op, T_op
%
% p.cruise_mode:
%   'range'     -> minimize E_per_m (J/m)
%   'endurance' -> minimize P_elec (W)

if nargin < 2, do_plot = false; end
if nargin < 3 || isempty(cost_mode)
    cost_mode = 'energy';
end

V_grid = linspace(p.V_min, p.V_max, 401).';
nV = numel(V_grid);

alpha = nan(nV,1);
Treq  = nan(nV,1);
Pelec = nan(nV,1);
Epm   = nan(nV,1);
Fdot  = nan(nV,1);
Fpm   = nan(nV,1);
feas  = true(nV,1);

for i = 1:nV
    V = V_grid(i);
    q = 0.5*p.rho*V^2;

    % Lift balance for level flight (gamma=0): L = m g
    CL_req = (p.m*p.g) / (q*p.S);
    alpha_req = (CL_req - p.CL0) / p.CLa;

    % Drag from parabolic polar
    CD_req = p.CD0 + p.k*CL_req^2;
    D_req = q*p.S*CD_req;

    T_needed = D_req;

    if alpha_req < p.alpha_min || alpha_req > p.alpha_max
        feas(i) = false;
    end
    T_max_here = thrust_max(V, p);
    if T_needed < p.T_min || T_needed > T_max_here
        feas(i) = false;
    end

    alpha(i) = alpha_req;
    Treq(i)  = T_needed;

    Pbus = (T_needed * V) / p.eta_total;
    if isfield(p,'battery') && ~isempty(p.battery)
        [~, ~, Pelec(i)] = battery_power_model(Pbus, p.battery.soc0, p.battery);
    else
        Pelec(i) = Pbus;
    end
    Epm(i)   = Pelec(i) / V; % = T/eta_total

    if ~isfield(p,'sigma') || isempty(p.sigma), p.sigma = 1.0; end
    Fdot(i) = p.sigma * T_needed;
    Fpm(i)  = Fdot(i) / V;
end

switch lower(cost_mode)
    case 'energy'
        time_metric = Pelec;      % W
        dist_metric = Epm;        % J/m
    case 'fuel_rate'
        time_metric = Fdot;       % sigma*T per second (proxy)
        dist_metric = Fpm;        % sigma*T per meter (proxy)
    otherwise
        error('Unknown cost_mode: %s', cost_mode);
end

P_valid = time_metric; P_valid(~feas) = inf;
Epm_valid = dist_metric; Epm_valid(~feas) = inf;

switch lower(p.cruise_mode)
    case 'range'
        [~, i_op] = min(Epm_valid);
    case 'endurance'
        [~, i_op] = min(P_valid);
    otherwise
        error('Unknown cruise_mode: %s', p.cruise_mode);
end

base = struct();
base.V_grid = V_grid;
base.alpha_grid = alpha;
base.T_grid = Treq;
base.P_elec_grid = Pelec;
base.E_per_m_grid = Epm;
base.Fdot_grid = Fdot;
base.F_per_m_grid = Fpm;
base.feasible = feas;
base.i_op = i_op;
base.V_op = V_grid(i_op);
base.alpha_op = alpha(i_op);
base.T_op = Treq(i_op);
base.cost_mode = cost_mode;
base.cost_time_op = time_metric(i_op);
base.cost_dist_op = dist_metric(i_op);

if do_plot
    figure('Name',sprintf('steady_cruise (%s)', p.cruise_mode),'Color','w');
    subplot(2,1,1);
    plot(V_grid, Pelec,'LineWidth',2); grid on; hold on;
    plot(base.V_op, base.P_elec_grid(i_op), 'ko','MarkerFaceColor','k');
    xlabel('V [m/s]'); ylabel('P_{elec} [W]');
    title('Steady power');
    subplot(2,1,2);
    plot(V_grid, Epm,'LineWidth',2); grid on; hold on;
    plot(base.V_op, base.E_per_m_grid(i_op), 'ko','MarkerFaceColor','k');
    xlabel('V [m/s]'); ylabel('J/m [J/m]');
    title('Steady energy per distance');
end

end
