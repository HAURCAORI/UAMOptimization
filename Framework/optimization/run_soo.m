function [d_opt, J_opt, history] = run_soo(d_init, cfg)
% RUN_SOO  Single-objective Stage 1 optimization.

if nargin < 2
    cfg = mdo_config();
end

if strcmpi(cfg.opt.method, 'cmaes')
    [d_opt, J_opt, history] = run_cmaes(d_init, cfg);
    return;
end

options = soo_params_from_cfg(cfg);
var_names = options.var_names;
Nvar = numel(var_names);
lb = options.lb(:);
ub = options.ub(:);

x0 = zeros(Nvar, 1);
for k = 1:Nvar
    x0(k) = d_init.(var_names{k});
end

eval_opt = UAMOptions(cfg, ...
    'Mode', options.eval_mode, ...
    'Objectives', options.objectives, ...
    'Model', get_field_or_default(options, 'model', struct()), ...
    'Verbose', false);
if isfield(options, 'model') && isfield(options.model, 'use_vehicle_model')
    d_init.use_vehicle_model = logical(options.model.use_vehicle_model);
end

history.J_best = [];
history.J_all = [];
history.x_best = x0;
history.n_evals = 0;
best_J = inf;
gen_count = 0;

if options.plot_live
    hfig = figure('Name', 'SOO Convergence', 'NumberTitle', 'off', ...
        'Position', [50 50 1100 500]);
    subplot(1, 3, 1); hax_conv = gca;
    title('Convergence'); xlabel('Evaluation'); ylabel('J (best so far)');
    grid on; hold on;

    subplot(1, 3, 2); hax_vars = gca;
    title('Current Best Design Variables (normalized)');
    grid on;

    subplot(1, 3, 3); hax_metrics = gca;
    title('Key Metrics at Best Design');
    grid on;
    drawnow;
else
    hfig = [];
    hax_conv = [];
    hax_vars = [];
    hax_metrics = [];
end

    function J_out = tracked_obj(x)
        history.n_evals = history.n_evals + 1;

        d_tmp = d_init;
        for ki = 1:Nvar
            d_tmp.(var_names{ki}) = x(ki);
        end
        if isfield(d_tmp, 'Lyo') && isfield(d_tmp, 'Lyi') && d_tmp.Lyo <= d_tmp.Lyi + 0.1
            J_out = 1e6;
            history.J_all(end + 1) = J_out;
            return;
        end

        r = eval_design(d_tmp, eval_opt);
        J_out = r.J_combined;
        if ~isfinite(J_out)
            J_out = 1e6;
        end

        history.J_all(end + 1) = J_out;
        if J_out < best_J
            best_J = J_out;
            history.J_best(end + 1) = best_J;
            history.x_best = x;
            update_live_plot(r, x);

            if options.verbose
                fprintf('  [eval %4d] J=%.5f | ', history.n_evals, best_J);
                for ki = 1:Nvar
                    fprintf('%s=%.3g ', var_names{ki}, x(ki));
                end
                fprintf('\n');
            end
        end
    end

con_fun = @(x) constraint_fcn(x, cfg, d_init);

if options.verbose
    fprintf('\n%s\n', repmat('=', 1, 65));
    fprintf('  Single-Objective Optimization  |  Method: %s\n', upper(options.method));
    fprintf('  Variables : %s\n', strjoin(var_names, ' '));
    fprintf('  Eval mode : %s\n', options.eval_mode);
    fprintf('  Objectives: ');
    for i_obj = 1:numel(options.objectives.names)
        fprintf('%s(%.2f) ', options.objectives.names{i_obj}, options.objectives.weights(i_obj));
    end
    fprintf('\n');
    fprintf('  Pop/Iter  : %d  |  Max gen: %d\n', options.pop_size, options.max_iter);
    fprintf('%s\n', repmat('=', 1, 65));
    t_start = tic;
end

switch lower(options.method)
    case 'ga'
        ga_opts = optimoptions('ga', ...
            'PopulationSize', options.pop_size, ...
            'MaxGenerations', options.max_iter, ...
            'Display', 'off', ...
            'UseParallel', false, ...
            'FunctionTolerance', 1e-6, ...
            'ConstraintTolerance', 1e-4, ...
            'OutputFcn', @ga_progress_fcn);
        [x_opt, J_opt] = ga(@tracked_obj, Nvar, [], [], [], [], lb, ub, con_fun, ga_opts);

    case 'fmincon'
        fmc_opts = optimoptions('fmincon', ...
            'Algorithm', 'sqp', ...
            'Display', 'off', ...
            'MaxIterations', options.max_iter, ...
            'MaxFunctionEvaluations', options.max_iter * 20, ...
            'OptimalityTolerance', 1e-6, ...
            'ConstraintTolerance', 1e-4);
        [x_opt, J_opt] = fmincon(@tracked_obj, x0, [], [], [], [], lb, ub, con_fun, fmc_opts);

    case 'patternsearch'
        ps_opts = optimoptions('patternsearch', ...
            'Display', 'off', ...
            'MaxIterations', options.max_iter * 10, ...
            'MaxFunctionEvaluations', options.max_iter * 100, ...
            'MeshTolerance', 1e-6);
        [x_opt, J_opt] = patternsearch(@tracked_obj, x0, [], [], [], [], lb, ub, con_fun, ps_opts);

    otherwise
        error('run_soo: unknown method ''%s''.', options.method);
end

d_opt = d_init;
for k = 1:Nvar
    d_opt.(var_names{k}) = x_opt(k);
end

if options.verbose
    elapsed = toc(t_start);
    fprintf('\n%s\n', repmat('-', 1, 65));
    fprintf('  Optimization complete  |  %.1f s  |  %d evaluations\n', elapsed, history.n_evals);
    fprintf('  J_opt = %.6f\n', J_opt);
    fprintf('  Optimal design:\n');
    for k = 1:Nvar
        fprintf('    %-8s = %.4g\n', var_names{k}, x_opt(k));
    end
    fprintf('%s\n', repmat('=', 1, 65));
end

if options.plot_live && ishandle(hfig)
    axes(hax_conv); cla;
    plot(history.J_all, '.', 'Color', [0.75 0.75 0.75], 'MarkerSize', 3);
    hold on;
    plot(cummin(history.J_all), 'b-', 'LineWidth', 2.5);
    xlabel('Function Evaluation');
    ylabel('Objective J');
    title(sprintf('SOO Convergence  [final J=%.5f, %d evals]', J_opt, history.n_evals));
    legend({'All evaluations', 'Best-so-far'}, 'Location', 'northeast');
    grid on;
    drawnow;
end

    function update_live_plot(r, x)
        if isempty(hfig) || ~ishandle(hfig)
            return;
        end
        axes(hax_conv); cla;
        plot(history.J_all, '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 3);
        hold on;
        plot(cummin(history.J_all), 'b-', 'LineWidth', 2);
        xlabel('Evaluation');
        ylabel('J');
        title(sprintf('Convergence  [best=%.5f]', best_J));
        grid on;

        axes(hax_vars); cla;
        x_norm = (x - lb) ./ max(ub - lb, 1e-9);
        bar(x_norm, 0.6, 'FaceColor', [0.3 0.6 0.9]);
        set(gca, 'XTick', 1:Nvar, 'XTickLabel', var_names, 'XTickLabelRotation', 30);
        ylim([0 1]);
        ylabel('Normalized value [0=lb, 1=ub]');
        title('Best Design Variables');
        grid on;

        axes(hax_metrics); cla;
        if ~isempty(r.objective_vector)
            bar(r.objective_vector, 0.5, 'FaceColor', [0.9 0.4 0.2]);
            set(gca, 'XTick', 1:numel(r.objective_vector), 'XTickLabel', r.objective_names);
            title(sprintf('Metrics  [J=%.4f]', best_J));
            grid on;
        end
        drawnow limitrate;
    end

    function [state, opts_out, changed] = ga_progress_fcn(options_in, state, flag)
        opts_out = options_in;
        changed = false;
        if strcmp(flag, 'iter') && options.verbose
            gen_count = gen_count + 1;
            if mod(gen_count, 5) == 0
                fprintf('  [gen %3d/%d]  best=%.5f  evals=%d\n', ...
                    gen_count, options.max_iter, best_J, history.n_evals);
            end
        end
    end
end

function options = soo_params_from_cfg(cfg)
active = cfg.vars.active;
options.method = cfg.opt.method;
options.var_names = cfg.vars.names(active);
options.lb = cfg.vars.lb(active);
options.ub = cfg.vars.ub(active);
options.eval_mode = cfg.eval.mode;
if isfield(cfg, 'objectives') && isfield(cfg.objectives, 'stage1')
    options.objectives = cfg.objectives.stage1;
else
    options.objectives = struct('names', {{'mass', 'power', 'fault_thrust', 'fault_alloc', 'hover_nom'}}, ...
        'weights', [0.20, 0.20, 0.25, 0.25, 0.10]);
end
options.max_iter = cfg.opt.max_iter;
options.pop_size = cfg.opt.ga_pop;
options.plot_live = cfg.opt.plot_live;
options.verbose = cfg.opt.verbose;
if isfield(cfg, 'model')
    options.model = cfg.model;
end
end

function value = get_field_or_default(s, name, default_value)
if isfield(s, name)
    value = s.(name);
else
    value = default_value;
end
end
