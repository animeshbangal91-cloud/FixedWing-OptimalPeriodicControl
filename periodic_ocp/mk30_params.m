function p = mk30_params(varargin)
% MK30_PARAMS  First-pass 2D cruise-model parameters for Amazon Prime Air MK30.
%
% This file is for a simplified longitudinal point-mass / wing-borne cruise model:
%   states  x = [X; Z; gamma; V]
%   inputs  u = [alpha; T]   or later [theta; delta] depending on your solver
%
% PUBLICLY KNOWN / REPORTED MK30 FACTS USED HERE:
%   - MTOW about 83.2 lb = 37.7 kg
%   - max payload about 5 lb = 2.27 kg
%   - max cruise speed about 73 mph = 32.6 m/s
%   - max operating range about 7.5 mi = 12.1 km
%
% IMPORTANT:
% Many aerodynamic / propulsion quantities below are NOT public and are
% engineering estimates for research use only. Replace them if you get
% better data.

%% ---------------- Identity ---------------------------------
p.name = 'Amazon Prime Air MK30 (wing-borne cruise model)';

%% ---------------- Operating-point selection -----------------
p.cruise_mode = 'endurance';   % 'range' or 'endurance'

%% ---------------- Geometry / mass ---------------------------
% Public MTOW:
%   83.2 lb = 37.74 kg
%
% Use near-MTOW cruise case for conservative delivery analysis.
p.m    = 37.7;        % [kg]  near-MTOW case
p.g    = 9.81;        % [m/s^2]

% Dimensions:
% Public dimensions are limited. A recent public report described the MK30
% as having a wingspan of about 5.5 ft (~1.68 m), but this is not from an
% official Amazon specification sheet. Use as a first-pass estimate only.
p.b    = 1.68;        % [m] estimated wingspan
p.S    = 0.75;        % [m^2] estimated effective wing area
p.cbar = 0.45;        % [m] estimated mean aerodynamic chord
p.AR   = p.b^2 / p.S; % [-]

%% ---------------- Aerodynamics ------------------------------
p.rho     = 1.225;    % [kg/m^3] sea-level ISA

% First-pass cruise-aircraft estimates.
% These are NOT public MK30 values.
p.CL0     = 0.35;
p.CLa     = 5.2;      % [1/rad]
p.e       = 0.82;
p.CD0     = 0.045;    % higher than small RC glider; conservative
p.k       = 1 / (pi * p.e * p.AR);

p.CL_max    = 1.4;
p.alpha_max = deg2rad(12);
p.alpha_min = deg2rad(-6);

% Publicly reported max cruise speed:
%   73 mph = 32.63 m/s
p.V_max     = 32.6;   % [m/s]

% Lower-speed bound: pick just above the trim stall envelope so steady
% level flight is feasible across [V_min, V_max].
%   V_stall ~ sqrt(2*m*g/(rho*S*CL_max))
%           ~ sqrt(2*37.7*9.81/(1.225*0.75*1.4)) ~ 24 m/s
% With CL_max,trim = CL0 + CLa*alpha_max ~ 1.44, V_stall_trim ~ 23.7 m/s.
% Add a small margin -> 23 m/s.
p.V_min     = 23.0;   % [m/s] just above trim stall (level-flight feasibility)
p.h_cruise  = 200;    % [m] nominal cruise altitude (raised to give zoom budget)
p.V_cruise_target = 25.0;  % [m/s] estimated nominal cruise target (informational)

%% ---------------- Propulsion / efficiency -------------------
% Public cruise thrust / power curves are not available.
% These are aggregate wing-borne-cruise estimates for a hybrid VTOL delivery drone.

p.T_min = 0.0;
p.T_max = 120.0;      % [N] structural / static thrust ceiling

% Combined propulsion efficiency estimate
p.eta_prop  = 0.72;
p.eta_motor = 0.90;
p.eta_esc   = 0.97;
p.eta_total = p.eta_prop * p.eta_motor * p.eta_esc;

%% ---------------- Thrust curve T_max(V) ---------------------
% Power-limited model: T_max(V) = min(T_static, P_max_elec * eta_total / V).
% At low V the prop is static-thrust-limited; at high V the battery/ESC
% power cap dominates and thrust falls off as 1/V.
%
% Switch to 'constant' to recover the legacy behavior (T_max = p.T_max).
p.thrust_model = 'power_limited';
p.T_static     = p.T_max;       % [N]
p.P_max_elec   = 3000.0;        % [W] estimated peak electrical power available
p.V_min_prop   = 1.0;           % [m/s] smoothing floor on V to avoid 1/V blow-up

%% ---------------- Paper-style fuel-rate scale ----------------
% Used only if COST='fuel_rate': L = sigma*T
% MK30 is electric, so this is only a proxy for comparative studies.
p.sigma = 1.0;

%% ---------------- Optional legacy / convenience fields -------
% Included for compatibility with older scripts.
p.Ixx = 1.2;          % [kg m^2] rough placeholder
p.Iyy = 2.0;          % [kg m^2] rough placeholder
p.Izz = 3.0;          % [kg m^2] rough placeholder
p.Ixz = 0.0;

%% ---------------- Apply overrides ---------------------------
if mod(numel(varargin), 2) ~= 0
    error('mk30_params: overrides must be name-value pairs.');
end
for i = 1:2:numel(varargin)
    key = varargin{i};
    val = varargin{i+1};
    if ~isfield(p, key)
        warning('mk30_params: unknown field "%s" — adding it.', key);
    end
    p.(key) = val;
end

%% ---------------- Recompute derived -------------------------
p.AR        = p.b^2 / p.S;
p.k         = 1 / (pi * p.e * p.AR);
p.eta_total = p.eta_prop * p.eta_motor * p.eta_esc;
% Keep T_static in sync if user overrode T_max but not T_static.
if isfield(p,'T_static') && p.T_static <= 0
    p.T_static = p.T_max;
end

end