function cfg = mdo_config()
% MDO_CONFIG  Master configuration for the UAM hexacopter MDO framework.
%
% All hyperparameters live here. Change this file to adjust the optimizer,
% objective weights, design variable bounds, or evaluation mode — no other
% files need to be edited.
%
% USAGE
%   cfg = mdo_config();
%   [d_opt, J_opt, history] = run_soo(design_default(), cfg);
%
% EXTENDING DESIGN VARIABLES
%   To add a new variable (e.g. cT):
%     cfg.vars.names{end+1} = 'cT';
%     cfg.vars.lb(end+1)    = 0.005;
%     cfg.vars.ub(end+1)    = 0.10;
%     cfg.vars.x0(end+1)    = 0.03;
%     cfg.vars.units{end+1} = '-';
%     cfg.vars.active(end+1) = true;
%
% FREEZING A VARIABLE
%   Set cfg.vars.active(k) = false — it stays at its x0 value.

%% ── OBJECTIVE WEIGHTS ─────────────────────────────────────────────────────
% Stage 1 default objective set for the rough design optimization pass.
% These names/weights are intended to be easy to change later.
cfg.objectives.stage1.names     = {'FII', 'hover', 'cost'};
cfg.objectives.stage1.weights   = [0.35, 0.40, 0.25];
cfg.objectives.stage1.moo_names = {'FII', 'hover', 'cost'};
%
% Stage 2 objective registry is reserved for later mission/control work.
cfg.objectives.stage2.names     = {'mission'};
cfg.objectives.stage2.weights   = 1.0;
%
% Legacy weight struct kept for compatibility with older scripts.
cfg.weights.FII     = 0.35;   % Fault Isotropy Index: std(r_i)/mean(r_i)
cfg.weights.hover   = 0.40;   % Hover safety penalty (0 when T_max ≥ 3× threshold)
cfg.weights.mission = 0.00;   % Mission tracking RMSE (needs eval.mode='sim'/'full')
cfg.weights.cost    = 0.25;   % Motor + structural cost index

%% ── DESIGN VARIABLES ──────────────────────────────────────────────────────
% Each column: one design variable.
%   names  : field name in the design struct d (must match design_default.m)
%   lb/ub  : lower/upper bounds in physical units
%   x0     : initial/baseline value (used for normalization and warm-start)
%   units  : string label for reporting
%   active : if false, variable is frozen at x0 during optimization
%
% EXTENSIBILITY: to add a new design variable, append one entry to each row.
% Example — activate propeller diameter as a design variable:
%   Set cfg.vars.active(5) = true;
%
cfg.vars.names  = {'Lx',  'Lyi', 'Lyo', 'T_max', 'd_prop'};
cfg.vars.lb     = [ 1.0,   1.0,   2.5,   8000,    0.30   ];
cfg.vars.ub     = [ 5.0,   5.0,   9.0,  16000,    0.60   ];
cfg.vars.x0     = [ 2.65,  2.65,  5.50,  7327,    0.40   ];  % baseline
cfg.vars.units  = {'m',   'm',   'm',   'N',     'm'    };
cfg.vars.active = [true,  true,  true,  true,    false  ];  % d_prop frozen

%% ── OPTIMIZER SETTINGS ────────────────────────────────────────────────────
cfg.opt.method     = 'cmaes';   % 'cmaes' | 'ga' | 'fmincon' | 'patternsearch'
%
% CMA-ES settings (ignored for other methods):
cfg.opt.max_evals  = 1500;      % total evaluation budget
cfg.opt.sigma0     = 0.25;      % initial step size in normalized [0,1] space
%                               % 0.25 → initial search radius = 25% of each range
cfg.opt.pop_size   = 0;         % CMA-ES offspring count λ; 0 = auto (4+floor(3*log(n)))
%
% GA settings (ignored for CMA-ES):
cfg.opt.max_iter   = 80;        % max generations
cfg.opt.ga_pop     = 50;        % GA population size
%
% Shared:
cfg.opt.tol_fun    = 1e-6;      % convergence tolerance on objective value
cfg.opt.use_polish = true;      % run fmincon SQP polish after global search
cfg.opt.verbose    = true;      % print per-generation progress
cfg.opt.plot_live  = false;      % show live convergence figure

%% ── VEHICLE MODEL ─────────────────────────────────────────────────────────
% Physics-based parameterized vehicle model (Delbecq 2020 scaling laws).
% When active, motor mass and frame mass are derived from design variables,
% making total vehicle mass a self-consistent computed quantity.
%
% This is the novel DATCOM-like contribution: larger T_max → heavier motors
% → heavier vehicle → higher hover threshold (self-consistent coupling).
%
% Deactivate only for legacy fixed-mass testing:
cfg.model.use_vehicle_model = true;   % true: physics-based mass; false: fixed d.m

%% ── TWO-STAGE EVALUATION ──────────────────────────────────────────────────
% Stage 1 (eval_stage1): ACS feasibility + hover screening (~15 ms/eval)
%   - Used during optimization (fast path)
%   - Computes: FII, WCFR, PFWAR, hover_margin
%
% Stage 2 (eval_stage2): Mission simulation (~200–500 ms/eval)
%   - Called only for designs passing Stage 1 (for validation)
%   - Computes: alt_rmse, attitude excursion, recovery_time
%
cfg.stage.run_stage2_in_opt = false;  % set true to include mission in optimization
%                                      % (much slower, ~30× cost per eval)

%% ── EVALUATION MODE ───────────────────────────────────────────────────────
cfg.eval.mode = 'acs';  % 'acs'  : Stage 1 only (~15 ms/eval) — use for optimization
%                        % 'sim'  : Stage 2 simulation only
%                        % 'full' : Stage 1 + Stage 2 (for final validation)

%% ── FAULT CONFIGURATION ───────────────────────────────────────────────────
cfg.fault.include_double = false;  % include double-motor faults in ACS (slower)
cfg.fault.p_motor        = 0.05;   % per-motor failure probability for PFWAR

%% ── SIMULATION CONFIGURATION (used when eval.mode is 'sim' or 'full') ────
cfg.sim.loe_vec    = [1;0;0;0;0;0]; % LOE vector: which motor fails (1=100% loss)
cfg.sim.T_end      = 30;            % simulation duration [s]
cfg.sim.fault_time = 5;             % time of fault injection [s]
cfg.sim.dt         = 0.01;          % timestep [s] (use 0.005 for accuracy)
cfg.sim.alt_cmd    = 10;            % commanded altitude [m]
cfg.sim.scenario   = 'hover';       % 'hover' | 'figure8'
cfg.sim.mission.A        = 100;     % figure-8 amplitude [m]
cfg.sim.mission.T_period = 120;     % figure-8 period [s]
cfg.sim.mission.z_cruise = 50;      % mission altitude [m]
cfg.sim.mission.t_start  = 20;      % climb duration before mission [s]
cfg.sim.mission.n_laps   = 1;       % number of figure-8 laps
cfg.sim.mission.ramp_time = 0;      % reference blend-in time [s], 0 matches Src script

end
