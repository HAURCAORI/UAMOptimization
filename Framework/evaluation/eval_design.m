function result = eval_design(d, options)
% EVAL_DESIGN  Master evaluator for ACS and simulation metrics.
%
%   result = eval_design(d)
%   result = eval_design(d, UAMOptions(...))
%
%   Stage 1 objectives: mass, power, fault_thrust, fault_alloc, hover_nom
%   Stage 2 objective:  mission (closed-loop simulation RMSE)

if nargin < 2
    options = UAMOptions();
end
options = normalize_call_options('eval_design', options);
if ~isfield(options, 'mode')
    options.mode = 'acs';
end
if ~isfield(options, 'fault_config')
    options.fault_config = struct();
end
if ~isfield(options, 'sim_config')
    options.sim_config = struct();
end
if ~isfield(options, 'loe_for_sim')
    options.loe_for_sim = [1;0;0;0;0;0];
end
if ~isfield(options, 'verbose')
    options.verbose = false;
end
if ~isfield(options, 'model')
    options.model = struct();
end
if ~isfield(options, 'objectives')
    options.objectives = default_objective_config(options);
end

if ~isfield(options.sim_config, 'loe_vec')
    options.sim_config.loe_vec = options.loe_for_sim;
end
options.sim_config = normalize_sim_config(options.sim_config);

d_eval = d;
if isfield(options.model, 'use_vehicle_model')
    d_eval.use_vehicle_model = logical(options.model.use_vehicle_model);
end

[UAM, Prop, Env] = hexacopter_params(d_eval);

geo_ok = (d_eval.Lx > 0.5) && (d_eval.Lyi > 0.5) && ...
         (d_eval.Lyo > d_eval.Lyi + 0.1) && (Prop.cT > 0);
feasible = geo_ok;

acs = [];
sim = [];
stage1 = [];
stage2 = [];
J_FII = NaN;
J_hover = NaN;
J_mission = NaN;

if ismember(options.mode, {'acs', 'full'})
    stage1 = eval_stage1(d_eval, options.fault_config);
    acs = stage1.acs;
    J_FII = stage1.J_FII;
    J_hover = stage1.J_hover;
    feasible = feasible && stage1.feasible;
end

if ismember(options.mode, {'sim', 'full'})
    stage2 = eval_stage2(d_eval, options.sim_config);
    sim = stage2.sim;
    J_mission = stage2.J_mission;
    feasible = feasible && stage2.feasible;
end

if isfield(UAM, 'model')
    mdl    = UAM.model;
    J_cost = mdl.cost_total / mdl.cost_ref;
    J_mass = mdl.m / mdl.m_ref;
else
    d_base     = design_default();
    arm_ratio  = (2*d_eval.Lx + 2*d_eval.Lyi + 2*d_eval.Lyo) / ...
                 (2*d_base.Lx + 2*d_base.Lyi + 2*d_base.Lyo);
    J_cost     = arm_ratio * (d_eval.T_max / d_base.T_max)^1.5;
    J_mass     = UAM.m / d_base.m;
end

objective_values = struct();
if ~isempty(stage1) && isfield(stage1, 'objectives')
    objective_values = stage1.objectives;
end
objective_values.mission = J_mission;
objective_values.cost    = J_cost;
objective_values.mass    = merge_value_or_default(objective_values, 'mass', J_mass);
if ~isempty(acs)
    objective_values.WCFR = acs.WCFR;
    objective_values.PFWAR = acs.PFWAR;
    objective_values.hover_margin_deficit = max(0, -acs.hover_margin);
end

[J_combined, objective_vector, objective_names, objective_weights] = ...
    combine_objectives(objective_values, options.objectives);

result.feasible = feasible;
result.J_combined = J_combined;
result.J_FII = J_FII;
result.J_hover = J_hover;
result.J_mission = J_mission;
result.J_cost = J_cost;
result.J_mass = merge_value_or_default(objective_values, 'mass', J_mass);
result.J_power = merge_value_or_default(objective_values, 'power', NaN);
result.J_fault_thrust = merge_value_or_default(objective_values, 'fault_thrust', NaN);
result.J_fault_alloc = merge_value_or_default(objective_values, 'fault_alloc', NaN);
result.J_fault = merge_value_or_default(objective_values, 'fault', NaN);
result.J_hover_nom = merge_value_or_default(objective_values, 'hover_nom', NaN);
result.acs = acs;
result.sim = sim;
result.stage1 = stage1;
result.stage2 = stage2;
result.d = d_eval;
result.m_total = UAM.m;
result.cT_effective = Prop.cT;
result.objectives = objective_values;
result.objective_vector = objective_vector;
result.objective_names = objective_names;
result.objective_weights = objective_weights;

if options.verbose
    fprintf('\n=== Design Evaluation Summary ===\n');
    fprintf('  Lx=%.2fm  Lyi=%.2fm  Lyo=%.2fm  T_max=%.0fN  cT=%.4f  m=%.0fkg\n', ...
        d_eval.Lx, d_eval.Lyi, d_eval.Lyo, d_eval.T_max, Prop.cT, UAM.m);
    if ~isempty(acs)
        fprintf('  FII           = %.4f  (J_FII=%.4f)\n', acs.FII, J_FII);
        fprintf('  WCFR          = %.4f\n', acs.WCFR);
        fprintf('  hover_margin  = %.4f  (J_hover=%.4f)\n', acs.hover_margin, J_hover);
        fprintf('  T_hover_worst = %.0f N  (ratio=%.2fx)\n', ...
            acs.T_hover_worst, acs.T_hover_worst / (UAM.m * Env.g / 6));
        fprintf('  Hover ok      = %d/6\n', sum(acs.hover_ok_single));
        fprintf('  PFWAR         = %.4f\n', acs.PFWAR);
    end
    if isfield(result, 'J_mass')
        fprintf('  Stage1 mass   = %.4f\n', result.J_mass);
    end
    if isfield(result, 'J_power') && isfinite(result.J_power)
        fprintf('  Stage1 power  = %.4f\n', result.J_power);
    end
    if isfield(result, 'J_fault_thrust') && isfinite(result.J_fault_thrust)
        fprintf('  fault thrust  = %.4f\n', result.J_fault_thrust);
    end
    if isfield(result, 'J_fault_alloc') && isfinite(result.J_fault_alloc)
        fprintf('  fault alloc   = %.4f\n', result.J_fault_alloc);
    end
    if isfield(result, 'J_hover_nom') && isfinite(result.J_hover_nom)
        fprintf('  hover nom     = %.4f\n', result.J_hover_nom);
    end
    if ~isempty(sim)
        fprintf('  alt_RMSE      = %.2f m  (J_mission=%.3f)\n', sim.alt_rmse, J_mission);
        if isfield(sim, 'path_rmse')
            fprintf('  path_RMSE     = %.2f m\n', sim.path_rmse);
        end
        fprintf('  max att       = %.1f deg  diverged=%d\n', sim.max_att_excurs, sim.diverged);
    end
    fprintf('  J_cost        = %.4f\n', J_cost);
    fprintf('  J_combined    = %.4f\n', J_combined);
    if ~isempty(objective_names)
        fprintf('  Objectives    = ');
        for k = 1:numel(objective_names)
            fprintf('%s=%.4f ', objective_names{k}, objective_vector(k));
        end
        fprintf('\n');
    end
    fprintf('  Feasible      = %d\n', feasible);
end
end

function objective_cfg = default_objective_config(options)
if strcmpi(options.mode, 'sim')
    objective_cfg.names = {'mission'};
    objective_cfg.weights = 1.0;
else
    objective_cfg.names = {'mass', 'power', 'fault_thrust', 'fault_alloc', 'hover_nom'};
    objective_cfg.weights = [0.20, 0.20, 0.25, 0.25, 0.10];
end
end


function [J_combined, objective_vector, objective_names, objective_weights] = ...
    combine_objectives(objective_values, objective_cfg)
objective_names = {};
objective_weights = [];
objective_vector = [];
J_combined = NaN;

if isempty(objective_cfg) || ~isfield(objective_cfg, 'names') || isempty(objective_cfg.names)
    return;
end

objective_names = objective_cfg.names(:)';
objective_weights = objective_cfg.weights(:)';
if numel(objective_names) ~= numel(objective_weights)
    error('eval_design: objective names/weights size mismatch.');
end

keep = false(size(objective_names));
vals = nan(size(objective_names));
for k = 1:numel(objective_names)
    name = objective_names{k};
    if isfield(objective_values, name)
        vals(k) = objective_values.(name);
        keep(k) = isfinite(vals(k));
    end
end

objective_names = objective_names(keep);
objective_weights = objective_weights(keep);
objective_vector = vals(keep);
if isempty(objective_vector)
    return;
end

w_sum = sum(objective_weights);
if w_sum <= 0
    error('eval_design: objective weights must sum to a positive value.');
end

J_combined = (objective_weights / w_sum) * objective_vector';
end

function value = merge_value_or_default(s, name, default_value)
if isfield(s, name) && isfinite(s.(name))
    value = s.(name);
else
    value = default_value;
end
end
