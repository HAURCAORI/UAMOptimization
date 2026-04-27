function result = eval_design(d, options)
% EVAL_DESIGN  Master evaluator for ACS and simulation metrics.

if nargin < 2
    options = struct();
end
if ~isfield(options, 'mode')
    options.mode = 'acs';
end
if ~isfield(options, 'fault_config')
    options.fault_config = struct();
end
if ~isfield(options, 'sim_config')
    options.sim_config = struct();
end
if ~isfield(options, 'weights')
    options.weights = [0.35, 0.40, 0.10, 0.15];
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

if isstruct(options.weights)
    ws = options.weights;
    options.weights = [ws.FII, ws.hover, ws.mission, ws.cost];
end

if ~isfield(options.sim_config, 'loe_vec')
    options.sim_config.loe_vec = options.loe_for_sim;
end

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
    mdl = UAM.model;
    J_cost = mdl.cost_total / mdl.cost_ref;
    J_struct = mdl.cost_frame / mdl.cost_ref;
    J_motor = mdl.cost_motor / mdl.cost_ref;
else
    d_base = design_default();
    arm_span = 2*d_eval.Lx + 2*d_eval.Lyi + 2*d_eval.Lyo;
    arm_span_base = 2*d_base.Lx + 2*d_base.Lyi + 2*d_base.Lyo;
    J_struct = arm_span / arm_span_base;
    J_motor = (d_eval.T_max / d_base.T_max)^1.5;
    J_cost = J_struct * J_motor;
end

w = options.weights;
terms = [J_FII, J_hover, J_mission, J_cost];
mask = ~isnan(terms);
if any(mask)
    J_combined = (w(mask) / sum(w(mask))) * terms(mask)';
else
    J_combined = NaN;
end

result.feasible = feasible;
result.J_combined = J_combined;
result.J_FII = J_FII;
result.J_hover = J_hover;
result.J_mission = J_mission;
result.J_cost = J_cost;
result.J_struct = J_struct;
result.J_motor = J_motor;
result.J_fault = J_hover;
result.J_isotropy = J_FII;
result.J_mass = J_struct;
result.acs = acs;
result.sim = sim;
result.d = d_eval;
result.m_total = UAM.m;
result.cT_effective = Prop.cT;

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
    if ~isempty(sim)
        fprintf('  alt_RMSE      = %.2f m  (J_mission=%.3f)\n', sim.alt_rmse, J_mission);
        fprintf('  max att       = %.1f deg  diverged=%d\n', sim.max_att_excurs, sim.diverged);
    end
    fprintf('  J_cost        = %.4f  (struct=%.3f  motor=%.3f)\n', J_cost, J_struct, J_motor);
    fprintf('  J_combined    = %.4f\n', J_combined);
    fprintf('  Feasible      = %d\n', feasible);
end
end
