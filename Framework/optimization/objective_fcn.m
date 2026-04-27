function [J, result] = objective_fcn(x, param_names, d_fixed, eval_options)
% OBJECTIVE_FCN  Scalar objective function for single-objective MDO.
%
%   Wraps eval_design.m for use with MATLAB optimizers (fmincon, ga, etc.).
%   Maps an optimizer variable vector x → scalar J.
%
%   Penalty method handles hard constraints: if the design violates physical
%   feasibility constraints, a large penalty is added to J.
%
%   Inputs:
%     x           - [Nvar×1] design variable vector (optimizer parameter)
%     param_names - {Nvar×1} cell array of field names in d corresponding to x
%                   e.g. {'Lx','Lyi','Lyo','T_max','cT'}
%     d_fixed     - Base design struct with all fixed parameters
%     eval_options- Options passed to eval_design.m
%
%   Output:
%     J      - scalar objective value (lower = better)
%     result - full eval_design result struct

% Map optimizer vector x into design struct
d = d_fixed;
for k = 1:numel(param_names)
    d.(param_names{k}) = x(k);
end

% Physical constraints: T_max must sustain hover with margin
g         = 9.81;
hover_min = d.m * g * 1.05 / 6;   % minimum T_max per motor (5% margin)
arm_min   = 0.5;                    % minimum arm length [m]
cT_min    = 0.001;                  % minimum moment-to-thrust

% Penalty: large positive value if design is infeasible
INFEAS_PENALTY = 1e6;
penalty = 0;

% Check basic bounds violations
if d.Lx  < arm_min || d.Lyi < arm_min || d.Lyo < arm_min
    penalty = penalty + INFEAS_PENALTY;
end
if d.Lyo <= d.Lyi
    penalty = penalty + INFEAS_PENALTY;   % outer must be wider than inner
end
if d.T_max < hover_min
    penalty = penalty + INFEAS_PENALTY;
end
if d.cT < cT_min
    penalty = penalty + INFEAS_PENALTY;
end

if penalty > 0
    J      = penalty;
    result = [];
    return
end

% Evaluate design
result = eval_design(d, eval_options);

if ~result.feasible
    J = INFEAS_PENALTY;
    return
end

J = result.J_combined;

% Guard against NaN (happens if all modes return NaN)
if isnan(J)
    J = INFEAS_PENALTY;
end
end
