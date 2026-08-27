function p = aircraft_params(varargin)
% AIRCRAFT_PARAMS  Parameter struct for Flightory Stallion VTOL (cruise model).
%
% 2D point-mass model parameters used by:
%   - steady_cruise.m
%   - pi_test_core.m
%   - ocp_casadi_fixed_T.m (CasADi)
%
% Operating point selector:
%   p.cruise_mode = 'range' or 'endurance'

%% ---------------- Identity ---------------------------------
p.name = 'Flightory Stallion VTOL (fixed-wing cruise)';

%% ---------------- Operating-point selection -----------------
p.cruise_mode = 'endurance'; % 'range' or 'endurance'

%% ---------------- Geometry / mass ---------------------------
p.m    = 3.0;     % [kg]
p.g    = 9.81;

p.b    = 1.340;   % [m]
p.S    = 0.265;   % [m^2]
p.cbar = 0.211;   % [m]
p.AR   = 5.6;     % [-]

%% ---------------- Aerodynamics ------------------------------
p.rho     = 1.225;
p.CL0     = 0.45;
p.CLa     = 5.5;          % [1/rad]
p.e       = 0.85;
p.CD0     = 0.021;
p.k       = 1 / (pi * p.e * p.AR);

p.CL_max    = 1.1;
p.alpha_max = deg2rad(10);
p.alpha_min = deg2rad(-6);

p.V_min     = 12.3;       % [m/s]
p.V_max     = 28.0;       % [m/s]

p.h_cruise  = 130;        % [m]

%% ---------------- Propulsion / efficiency -------------------
p.T_min = 0.0;
p.T_max = 20.0;           % [N]

p.eta_prop  = 0.70;
p.eta_motor = 0.85;
p.eta_esc   = 0.96;
p.eta_total = p.eta_prop * p.eta_motor * p.eta_esc;

% Thrust curve (default constant -> legacy behavior).
% To enable a state-dependent T_max(V), set p.thrust_model = 'power_limited'
% and provide p.T_static + p.P_max_elec.
p.thrust_model = 'constant';

%% ---------------- Paper-style fuel-rate scale ----------------
% Used only when COST='fuel_rate': L = sigma*T.
% Scaling doesn't affect argmin if sigma is constant, but we keep it explicit.
p.sigma = 1.0;

%% ---------------- Apply overrides ---------------------------
if mod(numel(varargin), 2) ~= 0
    error('aircraft_params: overrides must be name-value pairs.');
end
for i = 1:2:numel(varargin)
    key = varargin{i};
    val = varargin{i+1};
    p.(key) = val;
end

% Recompute derived
p.k         = 1 / (pi * p.e * p.AR);
p.eta_total = p.eta_prop * p.eta_motor * p.eta_esc;

end

