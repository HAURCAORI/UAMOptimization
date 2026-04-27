function [J, result] = objective_fcn(x, cfg_or_names, d_fixed, eval_options)
% OBJECTIVE_FCN  Scalar objective for MATLAB optimizers (fmincon, ga, etc.).
%
% Two calling conventions are supported:
%
%   NEW (cfg-based, recommended):
%     [J, r] = objective_fcn(x_norm, cfg, d_fixed)
%       x_norm : normalized design vector in [0,1] for each ACTIVE variable
%       cfg    : mdo_config struct
%       d_fixed: (optional) base design; defaults to design_default()
%
%   LEGACY (backward compatible):
%     [J, r] = objective_fcn(x_phys, param_names, d_fixed, eval_options)
%       x_phys      : physical design variable vector
%       param_names : cell array of field names  {'Lx','Lyi',...}
%       d_fixed     : base design struct
%       eval_options: options for eval_design

%% ── Detect calling convention ────────────────────────────────────────────
if isstruct(cfg_or_names) && isfield(cfg_or_names, 'vars')
    %% NEW: cfg-based interface ────────────────────────────────────────────
    cfg = cfg_or_names;
    if nargin < 3 || isempty(d_fixed)
        d_fixed = design_default();
    end

    active = cfg.vars.active;
    names  = cfg.vars.names(active);
    lb     = cfg.vars.lb(active)';
    ub     = cfg.vars.ub(active)';

    % Denormalize x from [0,1] to physical units
    x_phys = lb + x(:) .* (ub - lb);

    % Build eval options from cfg
    eval_opts.mode         = cfg.eval.mode;
    eval_opts.weights      = [cfg.weights.FII, cfg.weights.hover, ...
                               cfg.weights.mission, cfg.weights.cost];
    eval_opts.fault_config = cfg.fault;
    eval_opts.verbose      = false;
    eval_opts.sim_config   = cfg.sim;

else
    %% LEGACY: param_names interface ──────────────────────────────────────
    names  = cfg_or_names;   % cell array
    x_phys = x(:);
    if nargin < 4 || isempty(eval_options)
        eval_opts.mode    = 'acs';
        eval_opts.weights = [0.35, 0.40, 0.00, 0.25];
        eval_opts.verbose = false;
    else
        eval_opts = eval_options;
    end
end

%% ── Map variables into design struct ────────────────────────────────────
d = d_fixed;
for k = 1:numel(names)
    d.(names{k}) = x_phys(k);
end

%% ── Fast feasibility checks (avoids expensive eval for obvious failures) ──
g = 9.81;
INFEAS = 1e6;

if isfield(d,'Lx')  && d.Lx  < 0.5, J = INFEAS; result = []; return; end
if isfield(d,'Lyi') && d.Lyi < 0.5, J = INFEAS; result = []; return; end
if isfield(d,'Lyo') && isfield(d,'Lyi') && d.Lyo <= d.Lyi + 0.1
    J = INFEAS; result = []; return;
end
if isfield(d,'T_max') && d.T_max < d.m * g * 1.05 / 6
    J = INFEAS; result = []; return;
end

%% ── Evaluate design ──────────────────────────────────────────────────────
result = eval_design(d, eval_opts);

if ~result.feasible
    J = INFEAS;
    return;
end

J = result.J_combined;
if ~isfinite(J)
    J = INFEAS;
end
end
