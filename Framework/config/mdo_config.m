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
% Components of the combined objective J = sum(w_i * J_i) / sum(w_i)
cfg.weights.FII     = 0.35;   % Fault Isotropy Index: std(r_i)/mean(r_i)
cfg.weights.hover   = 0.40;   % Hover safety penalty (0 when T_max ≥ 3× threshold)
cfg.weights.mission = 0.00;   % Mission tracking RMSE (needs eval.mode='sim'/'full')
cfg.weights.cost    = 0.25;   % Motor + structural cost index

%% ── DESIGN VARIABLES ──────────────────────────────────────────────────────
% Each column: one design variable
%   names  : field name in the design struct d (must match design_default.m)
%   lb/ub  : lower/upper bounds in physical units
%   x0     : initial/baseline value (used for normalization and warm-start)
%   units  : string label for reporting
%   active : if false, variable is frozen at x0 during optimization
%
cfg.vars.names  = {'Lx',  'Lyi', 'Lyo', 'T_max'};
cfg.vars.lb     = [ 1.0,   1.0,   2.5,   8000  ];
cfg.vars.ub     = [ 5.0,   5.0,   9.0,  16000  ];
cfg.vars.x0     = [ 2.65,  2.65,  5.50,  7327  ];  % baseline design
cfg.vars.units  = {'m',   'm',   'm',   'N'   };
cfg.vars.active = [true,  true,  true,  true  ];    % all active

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
cfg.opt.plot_live  = true;      % show live convergence figure

%% ── EVALUATION MODE ───────────────────────────────────────────────────────
cfg.eval.mode = 'acs';  % 'acs'  : ACS/FCR metrics only (~15 ms/eval) — use for optimization
%                        % 'sim'  : closed-loop simulation metrics only
%                        % 'full' : both ACS + simulation (for final validation)

%% ── FAULT CONFIGURATION ───────────────────────────────────────────────────
cfg.fault.include_double = false;  % include double-motor faults in ACS (slower)
cfg.fault.p_motor        = 0.05;   % per-motor failure probability for PFWAR

%% ── SIMULATION CONFIGURATION (used when eval.mode is 'sim' or 'full') ────
cfg.sim.loe_vec    = [1;0;0;0;0;0]; % LOE vector: which motor fails (1=100% loss)
cfg.sim.T_end      = 30;            % simulation duration [s]
cfg.sim.fault_time = 5;             % time of fault injection [s]
cfg.sim.dt         = 0.01;          % timestep [s] (use 0.005 for accuracy)
cfg.sim.alt_cmd    = 10;            % commanded altitude [m]

end
