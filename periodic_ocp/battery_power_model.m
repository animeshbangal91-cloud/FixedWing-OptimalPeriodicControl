function [I, Vterm, Pchem, Ploss, Voc] = battery_power_model(Pbus, soc, b)
%BATTERY_POWER_MODEL Quasi-static Thevenin/Rint battery discharge model.
%
% Pbus = Vterm*I is the electrical power requested by propulsion.
% Vterm = Voc(soc) - R*I. The low-current quadratic root is used.

% Smooth generic LiPo OCV curve: 3.3 V empty, 4.2 V full, with a plateau.
Voc_cell = 3.3 + 0.9*soc + 0.4*soc.*(1-soc);
Voc = b.n_series * Voc_cell;

disc = Voc.^2 - 4*b.R_pack*Pbus;
I = (Voc - sqrt(disc + 1e-12)) / (2*b.R_pack);
Vterm = Voc - b.R_pack*I;
Ploss = b.R_pack*I.^2;
Pchem = Voc.*I;            % Pbus + ohmic loss
end
