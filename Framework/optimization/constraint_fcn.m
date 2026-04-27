function [c, ceq] = constraint_fcn(x, cfg_or_names, d_fixed)
% CONSTRAINT_FCN  Nonlinear constraints for fmincon / ga.
%
% Two calling conventions:
%
%   NEW (cfg-based):
%     [c, ceq] = constraint_fcn(x_phys, cfg, d_fixed)
%       x_phys : physical design variable vector (active vars only)
%       cfg    : mdo_config struct (active vars extracted automatically)
%
%   LEGACY (backward compatible):
%     [c, ceq] = constraint_fcn(x_phys, param_names, d_fixed)
%       x_phys     : physical design variable vector
%       param_names: cell array of field names  {'Lx','Lyi',...}
%       d_fixed    : base design struct
%
% CONSTRAINTS (c ≤ 0 = feasible)
%   c(1): Hover margin ≥ 10%  →  1.10*m*g - 6*T_max ≤ 0
%   c(2): Outer arm > inner + 0.1 m  →  -(Lyo-Lyi-0.1) ≤ 0
%   c(3): Worst-case ACS retention ≥ 5%  →  0.05 - WCFR ≤ 0
%   c(4): Hover feasibility under worst single fault  →  max_util - 1 ≤ 0

ceq = [];

%% ── Detect calling convention ────────────────────────────────────────────
if isstruct(cfg_or_names) && isfield(cfg_or_names, 'vars')
    cfg    = cfg_or_names;
    active = cfg.vars.active;
    names  = cfg.vars.names(active);
else
    names = cfg_or_names;
end

%% ── Map x into design struct ────────────────────────────────────────────
d = d_fixed;
for k = 1:numel(names)
    d.(names{k}) = x(k);
end

g = 9.81;
c = zeros(4, 1);

%% c(1): Total hover thrust margin ≥ 10%
c(1) = 1.10 * d.m * g - 6 * d.T_max;

%% c(2): Geometric separation (outer arm > inner + 0.1 m)
if isfield(d,'Lyo') && isfield(d,'Lyi')
    c(2) = -(d.Lyo - d.Lyi - 0.1);
else
    c(2) = 0;
end

%% c(3): Worst-case single-fault ACS volume retention ≥ 5%
[UAM, Prop, ~] = hexacopter_params(d);
B_mat  = UAM.B;
T_max  = Prop.T_max;
loe0   = zeros(6,1);

vol_nom = compute_acs_volume(B_mat, T_max, loe0);
WCFR    = 1.0;
if vol_nom > 0
    for k = 1:6
        loe     = loe0;  loe(k) = 1;
        vol_k   = compute_acs_volume(B_mat, T_max, loe);
        WCFR    = min(WCFR, vol_k / vol_nom);
    end
end
c(3) = 0.05 - WCFR;

%% c(4): Hover feasibility utilization under worst single fault
max_util = 0;
for k = 1:6
    loe = loe0;  loe(k) = 1;
    [~, util] = hover_feasibility(B_mat, T_max, d.m, g, loe);
    if isfinite(util)
        max_util = max(max_util, util);
    else
        max_util = inf;
    end
end
if ~isfinite(max_util)
    c(4) = 10.0;   % finite large value (GA requires no Inf/NaN in constraints)
else
    c(4) = max_util - 1.0;
end
end
