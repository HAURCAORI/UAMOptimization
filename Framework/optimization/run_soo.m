function [d_opt, J_opt, history] = run_soo(d_init, options)
% RUN_SOO  Single-Objective Optimization of hexacopter fault-tolerant design.
%
%   PROBLEM
%   -------
%   Minimize:   J(d) = w1*J_FII + w2*J_hover + w3*J_mission + w4*J_cost
%   Subject to: Physical feasibility constraints (see constraint_fcn.m)
%               Box bounds on selected design variables
%
%   METHODS
%   -------
%   'ga'          Genetic Algorithm – global, gradient-free. Default choice.
%                 Handles discontinuous hover-feasibility threshold.
%   'fmincon'     Sequential Quadratic Programming – fast local polish.
%                 Use after GA to tighten the result.
%   'patternsearch' Pattern Search – gradient-free local method.
%
%   VISUAL PROGRESS
%   ---------------
%   A live figure is maintained during the run showing:
%     • Convergence of the best objective value per generation / iteration
%     • Current best design variable values (bar chart)
%     • FII, hover_margin, J_cost per generation
%
%   Inputs:
%     d_init  - Initial design struct (see design_default.m)
%     options - (optional) struct with fields:
%       .method      : 'ga' | 'fmincon' | 'patternsearch'  (default: 'ga')
%       .var_names   : {Nvar×1} fields of d to optimize
%                      default: {'Lx','Lyi','Lyo','T_max'}
%       .lb          : [Nvar×1] lower bounds
%       .ub          : [Nvar×1] upper bounds
%       .eval_mode   : 'acs'|'sim'|'full'  passed to eval_design (default:'acs')
%       .weights     : [w_FII, w_hover, w_mission, w_cost]
%                      default: [0.35, 0.40, 0.00, 0.25]
%       .max_iter    : max generations (GA) or iterations  (default: 80)
%       .pop_size    : GA population size                  (default: 50)
%       .plot_live   : show live convergence figure        (default: true)
%       .verbose     : print per-generation summary        (default: true)
%
%   Outputs:
%     d_opt   - Optimal design struct
%     J_opt   - Optimal objective value (scalar)
%     history - Struct with convergence data:
%                 .J_best  : best J per generation
%                 .J_all   : all evaluated J values
%                 .x_best  : design vars at best J
%                 .n_evals : total function evaluations

% ── Accept cfg struct (from mdo_config) OR legacy options struct ───────────
if nargin < 2, options = struct(); end

% Detect if second arg is a full mdo_config struct
if isstruct(options) && isfield(options, 'vars') && isfield(options, 'opt')
    cfg     = options;
    options = cfg_to_soo_options(cfg);
end

% ── Defaults (legacy path) ────────────────────────────────────────────────
if ~isfield(options,'method'),    options.method    = 'ga';              end
if ~isfield(options,'var_names'), options.var_names = {'Lx','Lyi','Lyo','T_max'}; end
if ~isfield(options,'eval_mode'), options.eval_mode = 'acs';             end
if ~isfield(options,'weights'),   options.weights   = [0.35,0.40,0.00,0.25]; end
if ~isfield(options,'max_iter'),  options.max_iter  = 80;                end
if ~isfield(options,'pop_size'),  options.pop_size  = 50;                end
if ~isfield(options,'plot_live'), options.plot_live = true;              end
if ~isfield(options,'verbose'),   options.verbose   = true;              end

% ── Route CMA-ES to dedicated runner ─────────────────────────────────────
if strcmpi(options.method, 'cmaes')
    if exist('cfg','var')
        [d_opt, J_opt, history] = run_cmaes(d_init, cfg);
    else
        % Build minimal cfg from options struct
        cfg_tmp = build_cfg_from_options(d_init, options);
        [d_opt, J_opt, history] = run_cmaes(d_init, cfg_tmp);
    end
    return;
end

var_names = options.var_names;
Nvar      = numel(var_names);

% ── Variable bounds ────────────────────────────────────────────────────────
defaults_lb = struct('Lx',1.0,'Lyi',1.0,'Lyo',2.5,'T_max',8000,'cT',0.005,'m',d_init.m);
defaults_ub = struct('Lx',5.0,'Lyi',5.0,'Lyo',9.0,'T_max',16000,'cT',0.10,'m',d_init.m);

lb = zeros(Nvar,1);  ub = zeros(Nvar,1);
for k = 1:Nvar
    lb(k) = getfield_safe(options,'lb',k, defaults_lb.(var_names{k}));
    ub(k) = getfield_safe(options,'ub',k, defaults_ub.(var_names{k}));
end

% Initial point from d_init
x0 = zeros(Nvar,1);
for k = 1:Nvar
    x0(k) = d_init.(var_names{k});
end

% ── Evaluation options ────────────────────────────────────────────────────
eval_opts.mode    = options.eval_mode;
eval_opts.weights = options.weights;
eval_opts.verbose = false;
if isfield(options,'model')
    eval_opts.model = options.model;
    if isfield(options.model,'use_vehicle_model')
        d_init.use_vehicle_model = logical(options.model.use_vehicle_model);
    end
end

% ── History tracking ──────────────────────────────────────────────────────
history.J_best  = [];
history.J_all   = [];
history.x_best  = x0;
history.n_evals = 0;
best_J = inf;
best_x = x0;
gen_count = 0;

% ── Live figure setup ─────────────────────────────────────────────────────
if options.plot_live
    hfig = figure('Name','SOO Convergence','NumberTitle','off',...
                  'Position',[50 50 1100 500]);
    subplot(1,3,1); hax_conv = gca;
    title('Convergence'); xlabel('Evaluation'); ylabel('J (best so far)');
    grid on; hold on;

    subplot(1,3,2); hax_vars = gca;
    title('Current Best Design Variables (normalized)');
    grid on;

    subplot(1,3,3); hax_metrics = gca;
    title('Key Metrics at Best Design');
    grid on;
    drawnow;
end

% ── Wrapped objective (tracks history + updates live plot) ────────────────
function J_out = tracked_obj(x)
    history.n_evals = history.n_evals + 1;

    % Penalty for out-of-bounds or bad geometry before calling eval_design
    d_tmp = d_init;
    for ki = 1:Nvar, d_tmp.(var_names{ki}) = x(ki); end
    if isfield(d_tmp,'Lyo') && isfield(d_tmp,'Lyi') && d_tmp.Lyo <= d_tmp.Lyi+0.1
        J_out = 1e6;
        history.J_all(end+1) = J_out;
        return
    end

    r      = eval_design(d_tmp, eval_opts);
    J_out  = r.J_combined;
    if isnan(J_out) || ~isfinite(J_out), J_out = 1e6; end

    history.J_all(end+1) = J_out;

    if J_out < best_J
        best_J          = J_out;
        best_x          = x;
        history.J_best(end+1) = best_J;
        history.x_best  = x;

        % Update live figure
        if options.plot_live && ishandle(hfig)
            % Convergence curve
            axes(hax_conv); cla;
            plot(history.J_all, '.', 'Color',[0.7 0.7 0.7], 'MarkerSize',3);
            plot(cummin(history.J_all), 'b-', 'LineWidth',2);
            xlabel('Evaluation'); ylabel('J');
            title(sprintf('Convergence  [best=%.5f]', best_J));
            grid on;

            % Design variable bar (normalized to bounds)
            axes(hax_vars); cla;
            x_norm = (x - lb) ./ max(ub - lb, 1e-9);
            bar(x_norm, 0.6, 'FaceColor',[0.3 0.6 0.9]);
            set(gca,'XTick',1:Nvar,'XTickLabel',var_names,'XTickLabelRotation',30);
            ylim([0 1]); ylabel('Normalized value [0=lb, 1=ub]');
            title('Best Design Variables'); grid on;

            % Key metrics
            axes(hax_metrics); cla;
            if ~isempty(r.acs)
                metric_vals = [r.acs.FII, max(0,-r.acs.hover_margin), r.J_cost];
                metric_lbls = {'FII','Hover deficit','J\_cost'};
                bar(metric_vals, 0.5, 'FaceColor',[0.9 0.4 0.2]);
                set(gca,'XTick',1:3,'XTickLabel',metric_lbls);
                title(sprintf('Metrics  [J=%.4f]', best_J)); grid on;
            end
            drawnow limitrate;
        end

        if options.verbose
            fprintf('  [eval %4d] J=%.5f | ', history.n_evals, best_J);
            for ki = 1:Nvar
                fprintf('%s=%.3g ', var_names{ki}, x(ki));
            end
            fprintf('\n');
        end
    end
end

% ── Run optimizer ─────────────────────────────────────────────────────────
con_fun = @(x) constraint_fcn(x, var_names, d_init);

if options.verbose
    fprintf('\n%s\n', repmat('=',1,65));
    fprintf('  Single-Objective Optimization  |  Method: %s\n', upper(options.method));
    fprintf('  Variables : '); fprintf('%s ', var_names{:}); fprintf('\n');
    fprintf('  Eval mode : %s\n', options.eval_mode);
    fprintf('  Weights   : FII=%.2f  hover=%.2f  mission=%.2f  cost=%.2f\n', options.weights);
    fprintf('  Pop/Iter  : %d  |  Max gen: %d\n', options.pop_size, options.max_iter);
    fprintf('%s\n', repmat('=',1,65));
    t_start = tic;
end

switch lower(options.method)
    case 'ga'
        ga_opts = optimoptions('ga', ...
            'PopulationSize',       options.pop_size, ...
            'MaxGenerations',       options.max_iter, ...
            'Display',              'off', ...
            'UseParallel',          false, ...
            'FunctionTolerance',    1e-6, ...
            'ConstraintTolerance',  1e-4, ...
            'OutputFcn',            @ga_progress_fcn);

        [x_opt, J_opt] = ga(@tracked_obj, Nvar, [], [], [], [], lb, ub, con_fun, ga_opts);

    case 'fmincon'
        fmc_opts = optimoptions('fmincon', ...
            'Algorithm',              'sqp', ...
            'Display',                'off', ...
            'MaxIterations',          options.max_iter, ...
            'MaxFunctionEvaluations', options.max_iter*20, ...
            'OptimalityTolerance',    1e-6, ...
            'ConstraintTolerance',    1e-4);

        [x_opt, J_opt] = fmincon(@tracked_obj, x0, [], [], [], [], lb, ub, con_fun, fmc_opts);

    case 'patternsearch'
        ps_opts = optimoptions('patternsearch', ...
            'Display',                'off', ...
            'MaxIterations',          options.max_iter*10, ...
            'MaxFunctionEvaluations', options.max_iter*100, ...
            'MeshTolerance',          1e-6);

        [x_opt, J_opt] = patternsearch(@tracked_obj, x0, [], [], [], [], lb, ub, con_fun, ps_opts);

    otherwise
        error('run_soo: unknown method ''%s''. Use ''ga'', ''fmincon'', or ''patternsearch''.', options.method);
end

% ── Reconstruct optimal design ─────────────────────────────────────────────
d_opt = d_init;
for k = 1:Nvar
    d_opt.(var_names{k}) = x_opt(k);
end

if options.verbose
    elapsed = toc(t_start);
    fprintf('\n%s\n', repmat('-',1,65));
    fprintf('  Optimization complete  |  %.1f s  |  %d evaluations\n', ...
            elapsed, history.n_evals);
    fprintf('  J_opt = %.6f\n', J_opt);
    fprintf('  Optimal design:\n');
    for k = 1:Nvar
        fprintf('    %-8s = %.4g\n', var_names{k}, x_opt(k));
    end
    fprintf('%s\n', repmat('=',1,65));
end

% Final convergence figure
if options.plot_live && ishandle(hfig)
    axes(hax_conv); cla;
    plot(history.J_all, '.', 'Color',[0.75 0.75 0.75], 'MarkerSize',3);
    hold on;
    plot(cummin(history.J_all), 'b-', 'LineWidth',2.5);
    xlabel('Function Evaluation'); ylabel('Objective J');
    title(sprintf('SOO Convergence  [final J=%.5f, %d evals]', J_opt, history.n_evals));
    legend({'All evaluations','Best-so-far'},'Location','northeast');
    grid on;
    drawnow;
end

% ── Nested helper: GA output function for per-generation logging ───────────
function [state, opts_out, changed] = ga_progress_fcn(options_in, state, flag)
    opts_out = options_in;
    changed  = false;
    if strcmp(flag,'iter') && options.verbose
        gen_count = gen_count + 1;
        if mod(gen_count,5) == 0
            fprintf('  [gen %3d/%d]  best=%.5f  evals=%d\n', ...
                    gen_count, options.max_iter, best_J, history.n_evals);
        end
    end
end
end

% ── Utility: safe field access with default ────────────────────────────────
function v = getfield_safe(s, fname, idx, default)
    if isfield(s, fname) && numel(s.(fname)) >= idx
        v = s.(fname)(idx);
    else
        v = default;
    end
end

% ── Convert mdo_config struct → legacy run_soo options struct ──────────────
function options = cfg_to_soo_options(cfg)
    active = cfg.vars.active;
    options.method    = cfg.opt.method;
    options.var_names = cfg.vars.names(active);
    options.lb        = cfg.vars.lb(active);
    options.ub        = cfg.vars.ub(active);
    options.eval_mode = cfg.eval.mode;
    options.weights   = [cfg.weights.FII, cfg.weights.hover, ...
                         cfg.weights.mission, cfg.weights.cost];
    options.max_iter  = cfg.opt.max_iter;
    options.pop_size  = cfg.opt.ga_pop;
    options.plot_live = cfg.opt.plot_live;
    options.verbose   = cfg.opt.verbose;
    if isfield(cfg, 'model')
        options.model = cfg.model;
    end
end

% ── Build minimal mdo_config from a legacy options struct ──────────────────
function cfg = build_cfg_from_options(d_init, options)
    cfg = mdo_config();  % start from defaults

    % Override variable registry
    if isfield(options, 'var_names')
        n = numel(options.var_names);
        cfg.vars.names  = options.var_names;
        cfg.vars.active = true(1, n);
        % Rebuild x0 from d_init
        x0 = zeros(1, n);
        for k = 1:n
            if isfield(d_init, options.var_names{k})
                x0(k) = d_init.(options.var_names{k});
            end
        end
        cfg.vars.x0 = x0;
        if isfield(options, 'lb'), cfg.vars.lb = options.lb(:)'; end
        if isfield(options, 'ub'), cfg.vars.ub = options.ub(:)'; end
        % units: fill with '' for any unrecognised names
        known_units = containers.Map({'Lx','Lyi','Lyo','T_max','cT','m','d_prop','m_payload'}, ...
                                     {'m',  'm',  'm',  'N',   '-', 'kg', 'm', 'kg'});
        units = cell(1, n);
        for k = 1:n
            if isKey(known_units, options.var_names{k})
                units{k} = known_units(options.var_names{k});
            else
                units{k} = '';
            end
        end
        cfg.vars.units = units;
    end

    % Override optimizer settings
    if isfield(options, 'max_iter'),  cfg.opt.max_iter  = options.max_iter;  end
    if isfield(options, 'pop_size'),  cfg.opt.ga_pop    = options.pop_size;  end
    if isfield(options, 'plot_live'), cfg.opt.plot_live = options.plot_live; end
    if isfield(options, 'verbose'),   cfg.opt.verbose   = options.verbose;   end

    % Derive CMA-ES budget from max_iter × pop_size (mirrors GA effort)
    cfg.opt.max_evals = cfg.opt.max_iter * max(cfg.opt.ga_pop, 10);

    % Override weights
    if isfield(options, 'weights') && numel(options.weights) == 4
        w = options.weights;
        cfg.weights.FII     = w(1);
        cfg.weights.hover   = w(2);
        cfg.weights.mission = w(3);
        cfg.weights.cost    = w(4);
    end

    % Override eval mode
    if isfield(options, 'eval_mode'), cfg.eval.mode = options.eval_mode; end
end
