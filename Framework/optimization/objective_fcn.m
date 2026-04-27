function [J, result] = objective_fcn(x, cfg_or_names, d_fixed, eval_options)
% OBJECTIVE_FCN  Scalar objective for MATLAB optimizers.

if isstruct(cfg_or_names) && isfield(cfg_or_names, 'vars')
    cfg = cfg_or_names;
    if nargin < 3 || isempty(d_fixed)
        d_fixed = design_default();
    end

    active = cfg.vars.active;
    names = cfg.vars.names(active);
    lb = cfg.vars.lb(active)';
    ub = cfg.vars.ub(active)';
    x_phys = lb + x(:) .* (ub - lb);

    eval_opts.mode = cfg.eval.mode;
    eval_opts.weights = [cfg.weights.FII, cfg.weights.hover, ...
                         cfg.weights.mission, cfg.weights.cost];
    eval_opts.fault_config = cfg.fault;
    eval_opts.sim_config = cfg.sim;
    eval_opts.verbose = false;
    if isfield(cfg, 'model')
        eval_opts.model = cfg.model;
    end
else
    names = cfg_or_names;
    x_phys = x(:);
    if nargin < 4 || isempty(eval_options)
        eval_opts.mode = 'acs';
        eval_opts.weights = [0.35, 0.40, 0.00, 0.25];
        eval_opts.verbose = false;
    else
        eval_opts = eval_options;
    end
end

d = d_fixed;
for k = 1:numel(names)
    d.(names{k}) = x_phys(k);
end
if isfield(eval_opts, 'model') && isfield(eval_opts.model, 'use_vehicle_model')
    d.use_vehicle_model = logical(eval_opts.model.use_vehicle_model);
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

[UAM, ~, Env] = hexacopter_params(d);
if isfield(d, 'T_max') && d.T_max < UAM.m * Env.g * 1.05 / 6
    J = INFEAS;
    result = [];
    return;
end

result = eval_design(d, eval_opts);
if ~result.feasible || ~isfinite(result.J_combined)
    J = INFEAS;
else
    J = result.J_combined;
end
end
