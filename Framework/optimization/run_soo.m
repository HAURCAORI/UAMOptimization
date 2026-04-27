function [d_opt, J_opt, history] = run_soo(d_init, cfg)
% RUN_SOO  Single-Objective Optimization of hexacopter fault-tolerant design.
%
%   PROBLEM
%   -------
%   Minimize:   J(d) = weighted combination of configurable objective terms
%   Subject to: Physical feasibility constraints (see constraint_fcn.m)
%               Box bounds on selected design variables
%
%   METHODS (cfg.opt.method)
%   -------
%   'ga'          Genetic Algorithm – global, gradient-free. Default choice.
%                 Handles discontinuous hover-feasibility threshold.
%   'cmaes'       CMA-ES – routed to run_cmaes.m (normalized variable space).
%   'fmincon'     Sequential Quadratic Programming – fast local polish.
%   'patternsearch' Pattern Search – gradient-free local method.
%
%   VISUAL PROGRESS
%   ---------------
%   A live figure is maintained during the run showing:
%     • Convergence of the best objective value per generation / iteration
%     • Current best design variable values (bar chart)
%     • Objective metrics at the best design
%
%   Inputs:
%     d_init  - Initial design struct (see design_default.m)
%     cfg     - mdo_config struct (from mdo_config.m)
%
%   Outputs:
%     d_opt   - Optimal design struct
%     J_opt   - Optimal objective value (scalar)
%     history - Struct with convergence data:
%                 .J_best  : best J per generation
%                 .J_all   : all evaluated J values
%                 .x_best  : design vars at best J
%                 .n_evals : total function evaluations

if nargin < 2, cfg = mdo_config(); end

% ── Route CMA-ES to dedicated runner ─────────────────────────────────────
if strcmpi(cfg.opt.method, 'cmaes')
    [d_opt, J_opt, history] = run_cmaes(d_init, cfg);
    return;
end

options = soo_params_from_cfg(cfg);

var_names = options.var_names;
Nvar      = numel(var_names);

lb = options.lb(:);
ub = options.ub(:);

% Initial point from d_init
x0 = zeros(Nvar,1);
for k = 1:Nvar
    x0(k) = d_init.(var_names{k});
end

% ── Evaluation options ────────────────────────────────────────────────────
eval_opts.mode    = options.eval_mode;
eval_opts.objectives = options.objectives;
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
            if ~isempty(r.objective_vector)
                metric_vals = r.objective_vector;
                metric_lbls = r.objective_names;
                bar(metric_vals, 0.5, 'FaceColor',[0.9 0.4 0.2]);
                set(gca,'XTick',1:numel(metric_vals),'XTickLabel',metric_lbls);
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
con_fun = @(x) constraint_fcn(x, cfg, d_init);

if options.verbose
    fprintf('\n%s\n', repmat('=',1,65));
    fprintf('  Single-Objective Optimization  |  Method: %s\n', upper(options.method));
    fprintf('  Variables : '); fprintf('%s ', var_names{:}); fprintf('\n');
    fprintf('  Eval mode : %s\n', options.eval_mode);
    fprintf('  Objectives: ');
    for i_obj = 1:numel(options.objectives.names)
        fprintf('%s(%.2f) ', options.objectives.names{i_obj}, options.objectives.weights(i_obj));
    end
    fprintf('\n');
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

% ── Extract GA/fmincon/patternsearch parameters from mdo_config ────────────
function options = soo_params_from_cfg(cfg)
    active = cfg.vars.active;
    options.method    = cfg.opt.method;
    options.var_names = cfg.vars.names(active);
    options.lb        = cfg.vars.lb(active);
    options.ub        = cfg.vars.ub(active);
    options.eval_mode = cfg.eval.mode;
    if isfield(cfg, 'objectives') && isfield(cfg.objectives, 'stage1')
        options.objectives = cfg.objectives.stage1;
    else
        options.objectives.names = {'FII', 'hover', 'cost'};
        options.objectives.weights = [0.35, 0.40, 0.25];
    end
    options.max_iter  = cfg.opt.max_iter;
    options.pop_size  = cfg.opt.ga_pop;
    options.plot_live = cfg.opt.plot_live;
    options.verbose   = cfg.opt.verbose;
    if isfield(cfg, 'model')
        options.model = cfg.model;
    end
end

