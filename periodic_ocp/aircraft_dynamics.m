function [xdot, aux] = aircraft_dynamics(x, u, p)
%AIRCRAFT_DYNAMICS  2D point-mass fixed-wing model (X-Z plane).
%
% States:  x = [X; Z; gamma; V]
% Inputs:  u = [alpha; T]
%
% Dynamics:
%   Xdot     = V cos(gamma)
%   Zdot     = V sin(gamma)
%   gammadot = L/(mV) - g cos(gamma)/V
%   Vdot     = (T - D)/m - g sin(gamma)

X = x(1); %#ok<NASGU>
Z = x(2); %#ok<NASGU>
gamma = x(3);
V = x(4);

alpha = u(1);
T = u(2);

q  = 0.5 * p.rho * V.^2;
CL = p.CL0 + p.CLa * alpha;
CD = p.CD0 + p.k * CL.^2;
CD_prop = propeller_drag_cd(T, p);
CD = CD + CD_prop;

L = q * p.S * CL;
D = q * p.S * CD;

Xdot = V .* cos(gamma);
if isfield(p,'vertical_wind'), vertical_wind = p.vertical_wind; else, vertical_wind = 0; end
Zdot = V .* sin(gamma) + vertical_wind;
gammadot = L ./ (p.m .* V) - p.g .* cos(gamma) ./ V;
Vdot = (T - D) ./ p.m - p.g .* sin(gamma);

xdot = [Xdot; Zdot; gammadot; Vdot];

if nargout > 1
    aux = struct();
    aux.q = q;
    aux.CL = CL;
    aux.CD = CD;
    aux.CD_prop = CD_prop;
    aux.L = L;
    aux.D = D;
    aux.alpha = alpha;
    aux.T = T;
    aux.P_elec = propulsion_power_model(T, V, p);
end

end
