function [J, result] = objective_fcn(x, cfg, d_fixed)
% OBJECTIVE_FCN  Scalar Stage 1 objective in physical design space.
%
%   [J, result] = objective_fcn(x, cfg, d_fixed)

if nargin < 3 || isempty(d_fixed)
    d_fixed = design_default();
end

names = cfg.vars.names(cfg.vars.active);
d = d_fixed;
for k = 1:numel(names)
    d.(names{k}) = x(k);
end
if isfield(cfg, 'model') && isfield(cfg.model, 'use_vehicle_model')
    d.use_vehicle_model = logical(cfg.model.use_vehicle_model);
end

INFEAS = 1e6;
if isfield(d, 'Lx') && d.Lx < 0.5
    J = INFEAS;
    result = [];
    return;
end
if isfield(d, 'Lyi') && d.Lyi < 0.5
    J = INFEAS;
    result = [];
    return;
end
if isfield(d, 'Lyo') && isfield(d, 'Lyi') && d.Lyo <= d.Lyi + 0.1
    J = INFEAS;
    result = [];
    return;
end

[uam, ~, env] = hexacopter_params(d);
if isfield(d, 'T_max') && d.T_max < uam.m * env.g * 1.05 / 6
    J = INFEAS;
    result = [];
    return;
end

eval_opt = UAMOptions(cfg, ...
    'Mode', cfg.eval.mode, ...
    'Objectives', cfg.objectives.stage1, ...
    'Model', get_field_or_default(cfg, 'model', struct()));
result = eval_design(d, eval_opt);
if ~result.feasible || ~isfinite(result.J_combined)
    J = INFEAS;
else
    J = result.J_combined;
end
end

function value = get_field_or_default(s, name, default_value)
if isfield(s, name)
    value = s.(name);
else
    value = default_value;
end
end
