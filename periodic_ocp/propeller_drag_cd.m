function delta_cd = propeller_drag_cd(T,p)
%PROPELLER_DRAG_CD Smooth optional propeller-drag increment.
mode='off';if isfield(p,'propeller_drag_model'),mode=lower(p.propeller_drag_model);end
if strcmp(mode,'off')||strcmp(mode,'none'),delta_cd=0*T;return,end
if isfield(p,'propeller_drag'),pd=p.propeller_drag;else,pd=propeller_drag_params();end
switch mode
 case 'powered', glide_cd=pd.delta_cd_powered;
 case 'stopped', glide_cd=pd.delta_cd_stopped;
 case 'windmilling', glide_cd=pd.delta_cd_windmilling;
 case 'windmilling_low', glide_cd=pd.delta_cd_windmilling_low;
 case 'windmilling_high', glide_cd=pd.delta_cd_windmilling_high;
 case 'folding', glide_cd=pd.delta_cd_folding;
 otherwise,error('Unknown propeller drag model: %s',mode);
end
if strcmp(mode,'powered'),delta_cd=0*T+pd.delta_cd_powered;return,end
f=T./max(p.T_max,eps);c=pd.thrust_transition_fraction;w=pd.thrust_transition_width;
powered_blend=0.5*(1+tanh((f-c)./max(w,eps)));
delta_cd=glide_cd+(pd.delta_cd_powered-glide_cd).*powered_blend;
end
