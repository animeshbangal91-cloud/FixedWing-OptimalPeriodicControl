function Tm = thrust_max(V, p)
%THRUST_MAX  Available maximum thrust as a function of airspeed V.
%
% Selected by p.thrust_model:
%   'constant'      Tm = p.T_max                          (default; legacy behavior)
%   'linear'        Tm = p.T_static * max(eps, 1 - V/p.V_pitch)
%                   (fixed-pitch prop drops to ~0 at the pitch speed)
%   'power_limited' Tm = min(p.T_static, p.P_max_elec * p.eta_total / max(V, p.V_min_prop))
%                   (battery/ESC sets a power ceiling -> T ~ P/V at cruise)
%
% Required fields by model:
%   constant      : p.T_max
%   linear        : p.T_static (>= 0), p.V_pitch (> 0)
%   power_limited : p.T_static, p.P_max_elec, p.eta_total, p.V_min_prop (small > 0)
%
% V can be a numeric scalar/vector OR a CasADi MX symbolic expression.
% The output type matches V.

    if ~isfield(p,'thrust_model') || isempty(p.thrust_model)
        p.thrust_model = 'constant';
    end

    switch lower(p.thrust_model)
        case 'constant'
            % Return a value of the same "shape" as V (works with MX).
            Tm = 0*V + p.T_max;

        case 'linear'
            if ~isfield(p,'T_static'), p.T_static = p.T_max; end
            if ~isfield(p,'V_pitch'),  error('thrust_max: linear model needs p.V_pitch'); end
            % Smooth-ish floor at 0 to keep IPOPT happy
            ratio = 1 - V/p.V_pitch;
            % soft max(ratio, 0): use (ratio + sqrt(ratio^2 + eps^2))/2 if differentiability matters
            eps_s = 1e-3;
            ratio_pos = 0.5*(ratio + sqrt(ratio.^2 + eps_s^2));
            Tm = p.T_static * ratio_pos;

        case 'power_limited'
            if ~isfield(p,'T_static'),    p.T_static    = p.T_max; end
            if ~isfield(p,'P_max_elec'),  error('thrust_max: power_limited needs p.P_max_elec'); end
            if ~isfield(p,'eta_total'),   p.eta_total   = 1.0; end
            if ~isfield(p,'V_min_prop'),  p.V_min_prop  = 1.0; end  % avoids divide-by-zero
            V_safe = sqrt(V.^2 + p.V_min_prop^2);   % smooth floor on V
            T_pow  = p.P_max_elec * p.eta_total ./ V_safe;
            % Smooth min(T_static, T_pow): use min approximation
            % min(a,b) ~= 0.5*(a+b - sqrt((a-b)^2 + eps^2))
            eps_s = 0.5;  % [N], smoothing scale
            Tm = 0.5*(p.T_static + T_pow - sqrt((p.T_static - T_pow).^2 + eps_s^2));

        otherwise
            error('thrust_max: unknown thrust_model "%s"', p.thrust_model);
    end
end
