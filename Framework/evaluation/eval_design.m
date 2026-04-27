function result = eval_design(d, options)
% EVAL_DESIGN  Master design evaluator: ACS metrics + optional simulation.
%
%   PRIMARY DESIGN INSIGHTS (derived from analysis)
%   ------------------------------------------------
%   This tandem hexacopter has two fundamental properties:
%
%   (A) ACS RETENTION INVARIANCE:
%       The mean single-fault ACS volume retention is always exactly 1/3,
%       regardless of arm geometry (mathematical zonotope identity).
%       Therefore, aggregate ACS metrics (PFWAR) are ~geometry-invariant.
%
%   (B) HOVER THRESHOLD CONSTRAINT:
%       Motors 3 and 5 (mid-row motors) always require T_max ≥ 3.0×(m*g/6)
%       for hover feasibility after failure, regardless of arm geometry.
%       This sets the minimum required motor oversizing.
%
%   Given these findings, the objective is redesigned around:
%     - FII  : Fault Isotropy Index  → minimized by geometry choice
%     - J_hover_margin : penalizes insufficient T_max for hover safety
%     - J_cost : motor + structural cost → penalizes over-oversizing
%
%   COMBINED OBJECTIVE
%   ------------------
%   J = w1*FII + w2*J_hover + w3*J_mission + w4*J_cost
%
%   where:
%     J_hover   = max(0, 1 - hover_margin)  (0 when T_max covers worst fault)
%     J_mission = normalized altitude RMSE (simulation mode only)
%     J_cost    = (T_max/T_hover_worst)^1.5 * arm_span_ratio
%
%   EVALUATION MODES
%   ----------------
%   'acs'   : ACS/FCR metrics only (~0.2 s per call).  Optimizer mode.
%   'sim'   : Simulation-based metrics only.
%   'full'  : Both ACS and simulation.
%
%   Inputs:
%     d       - Design struct (see design_default.m)
%     options - (optional) struct with fields:
%                 .mode        : 'acs' | 'sim' | 'full'    (default: 'acs')
%                 .fault_config: passed to eval_acs
%                 .sim_config  : passed to eval_simulation
%                 .weights     : [w_FII, w_hover, w_mission, w_cost]
%                                default: [0.35, 0.40, 0.10, 0.15]
%                 .loe_for_sim : [6×1] LOE for simulation evaluation
%                 .verbose     : print summary (default: false)

% ── Defaults ──────────────────────────────────────────────────────────────
if nargin < 2, options = struct(); end
if ~isfield(options,'mode'),         options.mode         = 'acs';    end
if ~isfield(options,'fault_config'), options.fault_config = struct(); end
if ~isfield(options,'sim_config'),   options.sim_config   = struct(); end
if ~isfield(options,'weights'),      options.weights      = [0.35, 0.40, 0.10, 0.15]; end
if ~isfield(options,'loe_for_sim'),  options.loe_for_sim  = [1;0;0;0;0;0]; end
if ~isfield(options,'verbose'),      options.verbose      = false; end

% Accept weights as struct (from mdo_config) or vector (legacy)
if isstruct(options.weights)
    ws = options.weights;
    options.weights = [ws.FII, ws.hover, ws.mission, ws.cost];
end

if ~isfield(options.sim_config,'loe_vec')
    options.sim_config.loe_vec = options.loe_for_sim;
end

% ── Build physical model (needed for mass, inertia, cost metrics) ──────────
[UAM, Prop, Env] = hexacopter_params(d);

% ── Feasibility pre-check ─────────────────────────────────────────────────
hover_min_Tmax = UAM.m * Env.g / 6 * 1.05;   % need at least 1.05x hover per motor
cT_val  = Prop.cT;
geo_ok  = (d.Lx > 0.5) && (d.Lyi > 0.5) && (d.Lyo > d.Lyi + 0.1) && ...
          (cT_val > 0) && (d.T_max > hover_min_Tmax);
feasible = geo_ok;

% ── ACS evaluation (Stage 1) ──────────────────────────────────────────────
if ismember(options.mode, {'acs','full'})
    acs = eval_acs(d, options.fault_config);
else
    acs = [];
end

% ── Simulation evaluation (Stage 2) ───────────────────────────────────────
if ismember(options.mode, {'sim','full'})
    sim = eval_simulation(d, options.sim_config);
else
    sim = [];
end

% ── Cost metric ───────────────────────────────────────────────────────────
if isfield(UAM, 'model')
    % Physics-based cost: actual computed motor + frame mass (vehicle_model path)
    mdl    = UAM.model;
    J_cost = mdl.cost_total / mdl.cost_ref;   % normalized to baseline
    J_struct = mdl.m_frame / (mdl.cost_ref);  % frame share (for verbose)
    J_motor  = mdl.cost_motor / mdl.cost_ref; % motor share (for verbose)
else
    % Legacy cost: geometric proxy
    d_base       = design_default();
    arm_span     = 2*d.Lx + 2*d.Lyi + 2*d.Lyo;
    arm_span_base= 2*d_base.Lx + 2*d_base.Lyi + 2*d_base.Lyo;
    J_struct     = arm_span / arm_span_base;
    J_motor      = (d.T_max / d_base.T_max)^1.5;
    J_cost       = J_struct * J_motor;
end

% ── Scalar objectives ─────────────────────────────────────────────────────
w = options.weights;

% (a) Fault Isotropy Index: geometry-sensitive, minimized at optimal Lyi
if ~isempty(acs)
    J_FII = acs.FII;

    % (b) Hover margin objective: penalizes insufficient T_max
    %     hover_margin = T_max / T_hover_worst - 1  (≥0 = safe)
    %     J_hover = 0 when T_max exceeds worst threshold; penalize otherwise
    if acs.hover_margin >= 0
        J_hover = 0;     % all single faults can hover
    else
        % Exponential penalty: stronger near threshold
        J_hover = 1 - exp(5 * acs.hover_margin);   % ∈ (0,1) when margin < 0
    end
else
    J_FII   = NaN;
    J_hover = NaN;
end

% (c) Mission objective
if ~isempty(sim)
    alt_cmd = 10;
    if isfield(options.sim_config,'alt_cmd'), alt_cmd = options.sim_config.alt_cmd; end
    J_mission = min(sim.alt_rmse / alt_cmd, 20.0);
else
    J_mission = NaN;
end

% (d) Combined objective (skip NaN terms)
terms  = [J_FII, J_hover, J_mission, J_cost];
w_use  = w(~isnan(terms));
t_use  = terms(~isnan(terms));
if isempty(t_use)
    J_combined = NaN;
else
    J_combined = (w_use / sum(w_use)) * t_use';
end

% ── Pack result ───────────────────────────────────────────────────────────
result.feasible    = feasible;
result.J_combined  = J_combined;
result.J_FII       = J_FII;
result.J_hover     = J_hover;
result.J_mission   = J_mission;
result.J_cost      = J_cost;
result.J_struct    = J_struct;
result.J_motor     = J_motor;
% Legacy field names (for compatibility with other functions)
result.J_fault     = J_hover;       % J_hover is the fault safety objective
result.J_isotropy  = J_FII;
result.J_mass      = J_struct;
result.acs         = acs;
result.sim         = sim;
result.d           = d;

% ── Verbose output ────────────────────────────────────────────────────────
if options.verbose
    fprintf('\n=== Design Evaluation Summary ===\n');
    fprintf('  Lx=%.2fm  Lyi=%.2fm  Lyo=%.2fm  T_max=%.0fN  cT=%.4f  m=%.0fkg\n', ...
            d.Lx, d.Lyi, d.Lyo, d.T_max, Prop.cT, UAM.m);
    if ~isnan(J_FII)
        fprintf('  FII           = %.4f  (J_FII=%.4f)\n', acs.FII, J_FII);
        fprintf('  WCFR          = %.4f\n', acs.WCFR);
        fprintf('  hover_margin  = %.4f  (J_hover=%.4f)\n', acs.hover_margin, J_hover);
        fprintf('  T_hover_worst = %.0f N  (ratio=%.2fx)\n', acs.T_hover_worst, acs.T_hover_worst/(UAM.m*9.81/6));
        fprintf('  Hover ok      = %d/6\n', sum(acs.hover_ok_single));
        fprintf('  PFWAR         = %.4f\n', acs.PFWAR);
    end
    if ~isnan(J_mission)
        fprintf('  alt_RMSE      = %.2f m  (J_mission=%.3f)\n', sim.alt_rmse, J_mission);
        fprintf('  max att       = %.1f deg  diverged=%d\n', sim.max_att_excurs, sim.diverged);
    end
    fprintf('  J_cost        = %.4f  (struct=%.3f  motor=%.3f)\n', J_cost, J_struct, J_motor);
    fprintf('  J_combined    = %.4f\n', J_combined);
    fprintf('  Feasible      = %d\n', feasible);
end
end
