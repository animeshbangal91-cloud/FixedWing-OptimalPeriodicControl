function [Pelec, eta_total, eta_prop, eta_motor] = propulsion_power_model(T, V, p)
%PROPULSION_POWER_MODEL Electrical power for constant or mapped efficiency.

Puseful = T.*V;

if isfield(p,'propulsion_efficiency_model') && ...
        strcmpi(p.propulsion_efficiency_model,'efficiency_ratio') && ...
        isfield(p,'propulsion_map')
    pm=p.propulsion_map;
    load_fraction=T./p.T_max;
    blend=0.5*(1+tanh((load_fraction-pm.ratio_transition_load)./ ...
        pm.ratio_transition_width));
    eta_total=pm.ratio_eta_cruise+(pm.ratio_eta_climb-pm.ratio_eta_cruise).*blend;
    eta_prop=eta_total;eta_motor=0*T+1;
    Pelec=Puseful./(eta_total+1e-9);
    return;

elseif isfield(p,'propulsion_efficiency_model') && ...
        strcmpi(p.propulsion_efficiency_model,'flightory_generic') && ...
        isfield(p,'propulsion_map')
    pm = p.propulsion_map;
    cruise_shape = exp(-((T-pm.flightory_T_design)./pm.flightory_T_width).^2) .* ...
        exp(-((V-pm.flightory_V_design)./pm.flightory_V_width).^2);
    load_fraction = T./p.T_max;
    high_load = 0.5*(1+tanh((load_fraction- ...
        pm.flightory_high_load_center)./pm.flightory_high_load_width));
    eta_total = pm.flightory_eta_low + ...
        (pm.flightory_eta_peak-pm.flightory_eta_low).*cruise_shape - ...
        pm.flightory_high_load_loss.*high_load;
    eta_prop = eta_total;
    eta_motor = 0*T + 1;
    Pelec = Puseful./(eta_total+1e-9);
    return;

elseif isfield(p,'propulsion_efficiency_model') && ...
        strcmpi(p.propulsion_efficiency_model,'uiuc_7x4') && ...
        isfield(p,'propulsion_map')
    pm = p.propulsion_map;
    Vsafe = sqrt(V.^2 + 1e-6);
    if isfield(p,'n_propellers')
        n_propellers = p.n_propellers;
    else
        n_propellers = 1;
    end
    % T is total aircraft thrust; the UIUC curve describes one propeller.
    loading = (T./n_propellers) ./ ...
        (p.rho.*Vsafe.^2.*pm.prop_diameter_m.^2);
    eta_prop = pm.uiuc_eta_scale .* ...
        (1-exp(-pm.uiuc_eta_rise.*loading)) .* ...
        exp(-pm.uiuc_eta_fall.*loading);
    eta_motor = 0*V + p.eta_motor;
    eta_total = eta_prop.*eta_motor.*p.eta_esc;
    Pelec = Puseful ./ (eta_total + 1e-9);
    return;

elseif isfield(p,'propulsion_efficiency_model') && ...
        strcmpi(p.propulsion_efficiency_model,'bench_poly') && ...
        isfield(p,'propulsion_map')
    pm = p.propulsion_map;
    Pstatic = 0*T;
    for j = 1:numel(pm.bench_power_coeff)
        Pstatic = Pstatic + pm.bench_power_coeff(j).*T.^j;
    end
    Pforward_floor = Puseful ./ pm.eta_forward_max;
    dP = Pstatic - Pforward_floor;
    Pelec = 0.5*(Pstatic + Pforward_floor + ...
        sqrt(dP.^2 + pm.smooth_max_W^2));
    eta_total = Puseful ./ (Pelec + 1e-9);
    eta_prop = nan;
    eta_motor = nan;
    return;

elseif isfield(p,'propulsion_efficiency_model') && ...
        strcmpi(p.propulsion_efficiency_model,'map') && ...
        isfield(p,'propulsion_map')
    pm = p.propulsion_map;

    eta_prop = pm.eta_prop_min + ...
        (pm.eta_prop_peak-pm.eta_prop_min) .* ...
        exp(-((V-pm.V_design)./pm.V_width).^2);

    Pshaft = Puseful ./ eta_prop;
    motor_shape = (1-exp(-Pshaft./pm.P_rise)) .* exp(-Pshaft./pm.P_fall);
    eta_motor = pm.eta_motor_min + ...
        (pm.eta_motor_peak-pm.eta_motor_min).*motor_shape;

    eta_total = eta_prop.*eta_motor.*pm.eta_esc;
else
    eta_prop = 0*V + p.eta_prop;
    eta_motor = 0*V + p.eta_motor;
    eta_total = 0*V + p.eta_total;
end

Pelec = Puseful ./ eta_total;
end
