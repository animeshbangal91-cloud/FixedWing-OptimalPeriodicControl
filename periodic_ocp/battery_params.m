function b = battery_params(varargin)
%BATTERY_PARAMS Generic first-order model for a 6S 10 Ah LiPo pack.
%
% Values are representative assumptions, not manufacturer-qualified data.
% Replace the resistance, current rating, and voltage curve when measured or
% manufacturer data become available.

b.model = 'thevenin_rint';
b.chemistry = 'LiPo';
b.n_series = 6;
b.capacity_Ah = 10.0;
b.soc0 = 0.80;
b.soc_min = 0.20;
b.soc_max = 1.00;
b.R_pack = 0.030;          % [ohm], assumed pack DC resistance
b.I_max = 100.0;           % [A], provisional 10C continuous limit
b.V_cell_cutoff = 3.30;    % [V/cell]

if mod(numel(varargin),2) ~= 0
    error('battery_params: overrides must be name-value pairs.');
end
for k = 1:2:numel(varargin)
    b.(varargin{k}) = varargin{k+1};
end
end
