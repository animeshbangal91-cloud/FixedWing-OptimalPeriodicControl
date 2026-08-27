function p = aerosonde_params(varargin)
%AEROSONDE_PARAMS  Parameter struct matching the ACC'19 paper's UAV numbers.
%
% Based on the parameter set shown in the paper excerpt:
%   m = 13.5 kg, S = 0.55 m^2, AR = 15.2445, e = 0.90,
%   CL0 = 0.28, CLa = 3.45, CD0 = 0.03, rho = 1.2682,
%   alpha bounds approx +/- pi/18 (paper), thrust [0, 140] N (paper).

p.name = 'Paper UAV (Aerosonde-like, ACC 2019 numbers)';
p.cruise_mode = 'endurance';

p.g   = 9.81;
p.rho = 1.2682;

p.m  = 13.5;
p.S  = 0.55;
p.AR = 15.2445;
p.e  = 0.90;

p.CL0 = 0.28;
p.CLa = 3.45;
p.CD0 = 0.03;
p.k   = 1 / (pi * p.e * p.AR);

% Small-angle model validity bounds (paper uses +/- pi/18)
p.alpha_min = -pi/18;
p.alpha_max =  pi/18;

p.V_min = 8.0;
p.V_max = 80.0;

p.T_min = 0.0;
p.T_max = 140.0;

% Thrust curve (default constant -> legacy/paper behavior).
% Aerosonde paper assumes flat T_max envelope.
p.thrust_model = 'constant';

% If you want to interpret 'energy' cost for this UAV, keep an efficiency.
% (Not used in the paper-style fuel-rate cost.)
p.eta_total = 0.7;

p.h_cruise = 130;

% Paper endurance cost uses sigma*T (TSFC * thrust)
p.sigma = 1.0;

% Apply overrides
if mod(numel(varargin), 2) ~= 0
    error('aerosonde_params: overrides must be name-value pairs.');
end
for i = 1:2:numel(varargin)
    p.(varargin{i}) = varargin{i+1};
end

% Recompute derived
p.k = 1 / (pi * p.e * p.AR);

end

