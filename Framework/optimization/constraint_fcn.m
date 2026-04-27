function [c, ceq] = constraint_fcn(x, param_names, d_fixed)
% CONSTRAINT_FCN  Nonlinear constraint function for fmincon / ga.
%
%   Defines physical feasibility constraints as nonlinear inequalities
%   c(x) ≤ 0 and equalities ceq(x) = 0.
%
%   Constraints enforced
%   --------------------
%   c(1): Hover requires at least 10% total thrust margin
%         → 6 * T_max ≥ 1.10 * m * g
%         reformulated: 1.10 * m * g - 6 * T_max ≤ 0
%
%   c(2): Outer arm must be strictly wider than inner arm
%         → Lyo - Lyi ≥ 0.1 m  (minimum geometric separation)
%         reformulated: -(Lyo - Lyi - 0.1) ≤ 0
%
%   c(3): Minimum single-motor ACS volume retention ≥ 0.05
%         (design must retain at least 5% of ACS after any single failure)
%         reformulated: 0.05 - WCFR ≤ 0
%
%   c(4): Hover feasibility: all single-motor faults must allow hover
%         Encoded as: max(hover_util) ≤ 1  (utilization within limits)
%         Only checked if linprog toolbox is available.
%
%   Inputs:
%     x           - [Nvar×1] design variable vector
%     param_names - {Nvar×1} cell of field names
%     d_fixed     - base design struct with fixed parameters
%
%   Outputs:
%     c    - [Nc×1] inequality constraints (≤ 0 to be feasible)
%     ceq  - [] (no equality constraints)

ceq = [];

% Map x into design struct
d = d_fixed;
for k = 1:numel(param_names)
    d.(param_names{k}) = x(k);
end

g = 9.81;
c = zeros(4, 1);

% c(1): Hover margin
c(1) = 1.10 * d.m * g - 6 * d.T_max;

% c(2): Geometric separation (outer > inner + 0.1 m)
c(2) = -(d.Lyo - d.Lyi - 0.1);

% c(3): Worst-case single-motor ACS volume retention
[UAM, Prop, ~] = hexacopter_params(d);
B     = UAM.B;
T_max = Prop.T_max;

vol_nom = compute_acs_volume(B, T_max, zeros(6,1));

WCFR = 1.0;
if vol_nom > 0
    for k = 1:6
        loe = zeros(6,1);  loe(k) = 1;
        vol_k = compute_acs_volume(B, T_max, loe);
        WCFR  = min(WCFR, vol_k / vol_nom);
    end
end
c(3) = 0.05 - WCFR;   % require WCFR ≥ 0.05 (at least 5% ACS retained)

% c(4): Hover feasibility under worst single motor fault
% (utilization must stay ≤ 1)
max_util = 0;
for k = 1:6
    loe = zeros(6,1);  loe(k) = 1;
    [~, util] = hover_feasibility(B, T_max, d.m, g, loe);
    if isfinite(util)
        max_util = max(max_util, util);
    else
        max_util = inf;
    end
end
% Cap at finite value (GA requires real, finite constraint outputs)
if ~isfinite(max_util)
    c(4) = 10.0;   % large positive = infeasible, but finite for GA
else
    c(4) = max_util - 1.0;
end
end
