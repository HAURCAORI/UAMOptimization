function [pareto_designs, pareto_J, pareto_front, ga_output] = run_moo(d_init, cfg)
% RUN_MOO  Multi-objective Stage 1 optimization using gamultiobj.
%
%   [pareto_designs, pareto_J, pareto_front, ga_output] = run_moo(d_init, cfg)

if nargin < 2
    cfg = mdo_config();
end

options = cfg_to_moo_options(cfg);
var_names = options.var_names;
obj_names = options.objective_names;
Nvar = numel(var_names);

if numel(obj_names) < 2
    error('run_moo: at least two objectives are required.');
end

lb = options.lb(:);
ub = options.ub(:);
eval_opt = UAMOptions(cfg, ...
    'Mode', options.eval_mode, ...
    'Objectives', options.objectives, ...
    'Model', get_field_or_default(options, 'model', struct()));

if options.plot_live
    hfig = figure('Name', 'MOO Pareto Front (live)', 'NumberTitle', 'off', ...
        'Position', [100 100 900 420]);
    subplot(1, 2, 1);
    hax_par = gca;
    hold on;
    grid on;
    xlabel(obj_label(obj_names, 1));
    ylabel(obj_label(obj_names, 2));
    title('Pareto: Obj1 vs Obj2');
    subplot(1, 2, 2);
    hax_cost = gca;
    hold on;
    grid on;
    xlabel(obj_label(obj_names, 1));
    ylabel(obj_label(obj_names, min(3, numel(obj_names))));
    title('Pareto: Obj1 vs Obj3');
    sgtitle('MOO Pareto Front (updating...)', 'FontSize', 12, 'FontWeight', 'bold');
    drawnow;
else
    hfig = [];
    hax_par = [];
    hax_cost = [];
end

n_evals = 0;
gen_count = 0;

    function F = moo_fcn(x)
        n_evals = n_evals + 1;
        d = assign_design_vars(d_init, var_names, x);

        if isfield(d, 'Lyo') && isfield(d, 'Lyi') && d.Lyo <= d.Lyi + 0.1
            F = ones(1, numel(obj_names));
            F(min(3, numel(obj_names))) = 2;
            return;
        end
        [uam_tmp, ~, env_tmp] = hexacopter_params(d);
        if d.T_max < uam_tmp.m * env_tmp.g * 1.05 / 6
            F = ones(1, numel(obj_names));
            F(min(2, numel(obj_names))) = 2;
            return;
        end

        r = eval_design(d, eval_opt);
        F = objective_vector_from_result(r, obj_names);
        F = max(F, 0);
        F(~isfinite(F)) = 1;
    end

    function [state, opts_out, changed] = moo_output_fcn(options_in, state, flag)
        opts_out = options_in;
        changed = false;
        gen_count = gen_count + 1;

        if ~strcmp(flag, 'iter')
            return;
        end

        pf_idx = state.Rank == 1;
        F_pf = state.Score(pf_idx, :);

        if options.verbose && mod(gen_count, 10) == 0
            fprintf('  [gen %3d/%d]  Pareto size=%d  evals=%d\n', ...
                gen_count, options.max_gen, sum(pf_idx), n_evals);
        end

        if options.plot_live && ~isempty(F_pf) && ishandle(hfig)
            axes(hax_par); cla; hold on; grid on;
            scatter(F_pf(:,1), F_pf(:,2), 30, F_pf(:,min(3, size(F_pf, 2))), 'filled');
            colorbar;
            xlabel(obj_label(obj_names, 1));
            ylabel(obj_label(obj_names, 2));
            title(sprintf('Pareto: Obj1 vs Obj2  [gen %d, N=%d]', gen_count, size(F_pf,1)));

            axes(hax_cost); cla; hold on; grid on;
            if size(F_pf, 2) >= 3
                scatter(F_pf(:,1), F_pf(:,3), 30, F_pf(:,2), 'filled');
                ylabel(obj_label(obj_names, 3));
            else
                scatter(F_pf(:,1), F_pf(:,2), 30, F_pf(:,2), 'filled');
                ylabel(obj_label(obj_names, 2));
            end
            colorbar;
            xlabel(obj_label(obj_names, 1));
            title(sprintf('Pareto Progress  [gen %d]', gen_count));
            drawnow limitrate;
        end
    end

if options.verbose
    fprintf('\n%s\n', repmat('=', 1, 65));
    fprintf('  Multi-Objective Optimization  (gamultiobj)\n');
    fprintf('  Objectives : %s\n', strjoin(obj_names, ' '));
    fprintf('  Variables  : %s\n', strjoin(var_names, ' '));
    fprintf('  Pop: %d  |  MaxGen: %d\n', options.pop_size, options.max_gen);
    fprintf('%s\n', repmat('=', 1, 65));
    t_start = tic;
end

ga_opts = optimoptions('gamultiobj', ...
    'PopulationSize', options.pop_size, ...
    'MaxGenerations', options.max_gen, ...
    'ParetoFraction', 0.40, ...
    'CrossoverFraction', 0.80, ...
    'FunctionTolerance', 1e-5, ...
    'Display', 'off', ...
    'UseParallel', false, ...
    'OutputFcn', @moo_output_fcn);

[X_pareto, F_pareto, ~, ga_output] = gamultiobj(@moo_fcn, Nvar, ...
    [], [], [], [], lb, ub, ga_opts);

if options.verbose
    elapsed = toc(t_start);
    fprintf('\n  MOO complete  |  %.1f s  |  %d evals  |  %d Pareto solutions\n', ...
        elapsed, n_evals, size(X_pareto, 1));
    for i_obj = 1:size(F_pareto, 2)
        fprintf('  f%d (%s) range: [%.4f, %.4f]\n', i_obj, obj_names{i_obj}, ...
            min(F_pareto(:, i_obj)), max(F_pareto(:, i_obj)));
    end
    fprintf('%s\n', repmat('=', 1, 65));
end

N_pareto = size(X_pareto, 1);
pareto_J = F_pareto;
pareto_front = true(N_pareto, 1);
pareto_designs = cell(N_pareto, 1);
for k = 1:N_pareto
    dk = assign_design_vars(d_init, var_names, X_pareto(k, :));
    pareto_designs{k} = dk;
end

if options.plot_live && ishandle(hfig)
    axes(hax_par); cla; hold on; grid on;
    scatter(F_pareto(:,1), F_pareto(:,2), 50, F_pareto(:,min(3, size(F_pareto, 2))), ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.3);
    colorbar;
    xlabel(obj_label(obj_names, 1));
    ylabel(obj_label(obj_names, 2));
    title(sprintf('Final Pareto: Obj1 vs Obj2  [N=%d]', N_pareto));

    axes(hax_cost); cla; hold on; grid on;
    if size(F_pareto, 2) >= 3
        scatter(F_pareto(:,1), F_pareto(:,3), 50, F_pareto(:,2), ...
            'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.3);
        ylabel(obj_label(obj_names, 3));
    else
        scatter(F_pareto(:,1), F_pareto(:,2), 50, F_pareto(:,2), ...
            'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.3);
        ylabel(obj_label(obj_names, 2));
    end
    colorbar;
    xlabel(obj_label(obj_names, 1));
    title(sprintf('Final Pareto Front  [N=%d]', N_pareto));
    drawnow;
end
end

function options = cfg_to_moo_options(cfg)
active = cfg.vars.active;
options.var_names = cfg.vars.names(active);
options.lb = cfg.vars.lb(active);
options.ub = cfg.vars.ub(active);
options.eval_mode = cfg.eval.mode;
options.pop_size = cfg.opt.ga_pop;
options.max_gen = cfg.opt.max_iter;
options.plot_live = cfg.opt.plot_live;
options.verbose = cfg.opt.verbose;
if isfield(cfg, 'objectives') && isfield(cfg.objectives, 'stage1')
    options.objectives = cfg.objectives.stage1;
    if isfield(cfg.objectives.stage1, 'moo_names')
        options.objective_names = cfg.objectives.stage1.moo_names;
    else
        options.objective_names = cfg.objectives.stage1.names;
    end
else
    options.objectives = struct('names', {{'mass','power','fault_thrust','fault_alloc','hover_nom'}}, ...
        'weights', [0.20, 0.20, 0.25, 0.25, 0.10]);
    options.objective_names = {'mass','power','fault'};
end
if isfield(cfg, 'model')
    options.model = cfg.model;
end
end

function F = objective_vector_from_result(r, obj_names)
F = nan(1, numel(obj_names));
for k = 1:numel(obj_names)
    name = obj_names{k};
    if isfield(r.objectives, name)
        F(k) = r.objectives.(name);
    end
end
end

function label = obj_label(obj_names, idx)
if idx <= numel(obj_names)
    label = sprintf('J_{%s}', obj_names{idx});
else
    label = sprintf('Obj%d', idx);
end
end

function value = get_field_or_default(s, name, default_value)
if isfield(s, name)
    value = s.(name);
else
    value = default_value;
end
end

function d = assign_design_vars(d, names, values)
for idx = 1:numel(names)
    d.(names{idx}) = values(idx);
end
end
