function pd = propeller_drag_params(varargin)
%PROPELLER_DRAG_PARAMS Optional equivalent-aircraft propeller drag settings.
% Coefficients are generic sensitivities, not measured Stallion data. The
% active final configuration uses mode 'off', for which delta CD is exactly 0.
pd.delta_cd_powered = 0.004;
pd.delta_cd_stopped = 0.008;
pd.delta_cd_windmilling = 0.012;
pd.delta_cd_windmilling_low = 0.006;
pd.delta_cd_windmilling_high = 0.018;
pd.delta_cd_folding = 0.001;
pd.thrust_transition_fraction = 0.05;
pd.thrust_transition_width = 0.015;
if mod(numel(varargin),2)~=0,error('Overrides must be name-value pairs.');end
for k=1:2:numel(varargin),pd.(varargin{k})=varargin{k+1};end
end
