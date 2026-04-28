function cfg = mdo_config()
% MDO_CONFIG  Master configuration for the UAM hexacopter MDO framework.
%
%   cfg = mdo_config();
%   [d_opt, J_opt, history] = run_soo(design_default(), cfg);
%
%   This file is the single source of truth for:
%     - Stage 1 objective names, weights, and objective-specific parameters
%     - Design variable names, bounds, initial point, and active flags
%     - Optimizer settings
%     - Simulation settings
%
%   -----------------------------------------------------------------------
%   HOW TO ADD A NEW OBJECTIVE
%     1. Append a name to cfg.objectives.stage1.names and its weight to
%        cfg.objectives.stage1.weights.
%     2. Compute the value in eval_stage1.m and store it in
%        result.objectives.<name>.
%     3. Done. eval_design.m picks it up automatically via combine_objectives.
%
%   HOW TO ADD A NEW DESIGN VARIABLE
%     1. Append to cfg.vars.names / lb / ub / x0 / units / active.
%     2. Make sure hexacopter_params.m (or your physics model) reads d.<name>.
%     3. Done. run_soo / run_cmaes iterate over all active variables.
%
%   HOW TO FREEZE A VARIABLE
%     Set the corresponding cfg.vars.active entry to false.
%   -----------------------------------------------------------------------

%% ========== STAGE 1 OBJECTIVES ==========
% Weighted sum: J = sum(w_i * f_i(d)) / sum(w_i)
% Each f_i is computed in eval_stage1.m.
%
%   mass         : m_tot / m_ref                              (minimize total mass)
%   power        : P_hover / P_ref                            (minimize induced hover power)
%   fault_thrust : max(0, gamma_req - gamma_worst)^2          (fault thrust margin penalty)
%   fault_alloc  : sigma_ref / sigma_worst                    (control effectiveness ratio)
%   hover_nom    : (T_bar_hover / T_max)^2                    (nominal hover load fraction)
%
cfg.objectives.stage1.names   = {'mass', 'power', 'fault_thrust', 'fault_alloc', 'hover_nom'};
cfg.objectives.stage1.weights = [0.20,   0.20,    0.25,           0.25,          0.10];

% gamma_req: required post-fault thrust-to-weight ratio for J_fault_thrust.
% Increase to demand greater fault margin (higher value → more optimization pressure).
% At gamma_req = 1.0 the term is a pure feasibility penalty (0 when hover-feasible).
% At gamma_req = 1.5 the term also penalizes low-margin feasible designs.
cfg.fault.gamma_T_req = 1.5;

% Three-objective names for the MOO Pareto front (must be a subset of stage1.names
% or computed as combinations; 'fault' = 0.5*(fault_thrust + fault_alloc)).
cfg.objectives.stage1.moo_names = {'mass', 'power', 'fault'};

%% ========== STAGE 2 OBJECTIVES ==========
% Used when eval.mode = 'sim' or 'full'.  Currently a single mission-tracking RMSE.
cfg.objectives.stage2.names   = {'mission'};
cfg.objectives.stage2.weights = 1.0;

%% ========== DESIGN VARIABLES ==========
% Each column of the table below defines one variable.
% Set active = false to freeze at x0 (optimizer ignores it).
%
%   name    lb      ub      x0      units   active
%   ------  ------  ------  ------  ------  ------
%   Lx      1.0     5.0     2.65    m       true     fore/aft arm length
%   Lyi     1.0     5.0     2.65    m       true     inner lateral arm (front/rear rotors)
%   Lyo     2.5     9.0     5.50    m       true     outer lateral arm (mid rotors)
%   T_max   8000    16000   8000    N       true     max thrust per motor
%   d_prop  0.30    0.60    0.40    m       false    propeller diameter (frozen at baseline)
%
cfg.vars.names  = {'Lx',  'Lyi', 'Lyo', 'T_max', 'd_prop'};
cfg.vars.lb     = [ 1.0,   1.0,   2.5,   8000,    0.30   ];
cfg.vars.ub     = [ 5.0,   5.0,   9.0,  16000,    0.60   ];
cfg.vars.x0     = [ 2.65,  2.65,  5.50,  8000,    0.40   ];
cfg.vars.units  = {'m',   'm',   'm',   'N',     'm'    };
cfg.vars.active = [true,  true,  true,  true,    false  ];

%% ========== OPTIMIZER SETTINGS ==========
cfg.opt.method     = 'cmaes';   % 'cmaes' | 'ga' | 'fmincon' | 'patternsearch'
cfg.opt.max_evals  = 1500;      % CMA-ES function evaluation budget
cfg.opt.sigma0     = 0.25;      % CMA-ES initial step size in normalized [0,1] space
cfg.opt.pop_size   = 0;         % CMA-ES offspring per generation (0 = auto: 4+floor(3*log(n)))
cfg.opt.max_iter   = 80;        % GA / fmincon / patternsearch iteration budget
cfg.opt.ga_pop     = 50;        % GA population size
cfg.opt.tol_fun    = 1e-6;      % convergence tolerance on sigma (CMA-ES)
cfg.opt.use_polish = true;      % run fmincon polish after CMA-ES
cfg.opt.verbose    = true;
cfg.opt.plot_live  = false;

%% ========== VEHICLE MODEL ==========
cfg.model.use_vehicle_model = true;   % use physics-based mass via vehicle_model.m

%% ========== EVALUATION MODE ==========
cfg.eval.mode = 'acs';   % 'acs' | 'sim' | 'full'

%% ========== FAULT SETTINGS ==========
cfg.fault.include_double = false;   % include double-motor faults in ACS eval
cfg.fault.p_motor        = 0.05;    % per-motor failure probability (for PFWAR)
% cfg.fault.gamma_T_req is defined in the objectives section above

%% ========== SIMULATION SETTINGS ==========
% Used when eval.mode = 'sim' or 'full'.
cfg.sim.loe_vec  = [1;0;0;0;0;0];   % loss-of-effectiveness vector (1 = full LOE)
cfg.sim.t_end    = 30;               % simulation end time [s]
cfg.sim.t_fault  = 5;                % fault injection time [s]
cfg.sim.dt       = 0.01;             % integration time step [s]
cfg.sim.alt_cmd  = 10;               % altitude command [m]
cfg.sim.scenario = 'hover';          % 'hover' | 'figure8'

cfg.sim.mission.A         = 100;     % figure-8 semi-axis [m]
cfg.sim.mission.T_period  = 120;     % figure-8 period [s]
cfg.sim.mission.z_cruise  = 50;      % cruise altitude [m]
cfg.sim.mission.t_start   = 20;      % mission start time after climb [s]
cfg.sim.mission.n_laps    = 1;
cfg.sim.mission.ramp_time = 0;
end
