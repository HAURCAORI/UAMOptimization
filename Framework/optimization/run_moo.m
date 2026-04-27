function [pareto_designs, pareto_J, pareto_front, ga_output] = run_moo(d_init, options)
% RUN_MOO  Multi-Objective Optimization: Pareto front for fault-tolerant design.
%
%   Computes the Pareto-optimal design front using MATLAB's gamultiobj.
%
%   THREE-OBJECTIVE FORMULATION
%   ---------------------------
%   f1 = J_FII    Fault Isotropy Index      (minimize → balanced failure impact)
%   f2 = J_hover  Hover feasibility penalty (minimize → adequate T_max margin)
%   f3 = J_cost   Structural + motor cost   (minimize → compact, light design)
%
%   TRADE-OFF INSIGHT (from design-space analysis)
%   -----------------------------------------------
%   • FII is minimized at Lyi ≈ 4m (geometry-driven, independent of T_max)
%   • Hover feasibility requires T_max ≥ 3× (m·g/6) for this configuration
%   • Structural cost decreases with shorter arms (smaller Lyi) and lower T_max
%   • Key Pareto trade-off: FII (wants larger Lyi) vs Cost (wants smaller Lyi)
%
%   VISUAL PROGRESS
%   ---------------
%   Live Pareto front scatter plot updated every generation.
%
%   Inputs:
%     d_init  - Baseline design struct (see design_default.m)
%     options - (optional) struct:
%       .var_names  : variables to optimize  (default: {'Lx','Lyi','Lyo','T_max'})
%       .lb, .ub    : bounds per variable
%       .eval_mode  : 'acs'|'full'           (default: 'acs')
%       .pop_size   : population size        (default: 100)
%       .max_gen    : max generations        (default: 150)
%       .plot_live  : live Pareto figure     (default: true)
%       .verbose    : print progress         (default: true)
%
%   Outputs:
%     pareto_designs - {N×1} cell of Pareto-optimal design structs
%     pareto_J       - [N×3] Pareto objective matrix [f1, f2, f3]
%     pareto_front   - [N×1] logical index (all true; gamultiobj output)
%     ga_output      - raw gamultiobj output struct

% ── Defaults ──────────────────────────────────────────────────────────────
if nargin < 2, options = struct(); end
if isstruct(options) && isfield(options, 'vars') && isfield(options, 'opt')
    cfg = options;
    options = cfg_to_moo_options(cfg);
end
if ~isfield(options,'var_names'),  options.var_names  = {'Lx','Lyi','Lyo','T_max'}; end
if ~isfield(options,'eval_mode'),  options.eval_mode  = 'acs';   end
if ~isfield(options,'pop_size'),   options.pop_size   = 100;     end
if ~isfield(options,'max_gen'),    options.max_gen    = 150;     end
if ~isfield(options,'plot_live'),  options.plot_live  = true;    end
if ~isfield(options,'verbose'),    options.verbose    = true;    end
if ~isfield(options,'objective_names'), options.objective_names = {'FII','hover','cost'}; end

var_names = options.var_names;
Nvar      = numel(var_names);
obj_names  = options.objective_names;
if numel(obj_names) < 2
    error('run_moo requires at least 2 objective names for gamultiobj.');
end

defaults_lb = struct('Lx',1.0,'Lyi',1.0,'Lyo',2.5,'T_max',8000,'cT',0.005);
defaults_ub = struct('Lx',5.0,'Lyi',5.0,'Lyo',9.0,'T_max',16000,'cT',0.10);

lb = zeros(Nvar,1);  ub = zeros(Nvar,1);
for k = 1:Nvar
    if isfield(options,'lb'), lb(k) = options.lb(k);
    else, lb(k) = defaults_lb.(var_names{k}); end
    if isfield(options,'ub'), ub(k) = options.ub(k);
    else, ub(k) = defaults_ub.(var_names{k}); end
end

eval_opts.mode    = options.eval_mode;
eval_opts.verbose = false;
if isfield(options,'objectives')
    eval_opts.objectives = options.objectives;
end
if isfield(options,'model')
    eval_opts.model = options.model;
    if isfield(options.model,'use_vehicle_model')
        d_init.use_vehicle_model = logical(options.model.use_vehicle_model);
    end
end

% Live figure
if options.plot_live
    hfig = figure('Name','MOO Pareto Front (live)','NumberTitle','off',...
                  'Position',[100 100 900 420]);
    subplot(1,2,1); hax_par  = gca; hold on; grid on;
    xlabel(obj_label(obj_names, 1)); ylabel(obj_label(obj_names, 2)); title('Pareto: Obj1 vs Obj2');
    subplot(1,2,2); hax_cost = gca; hold on; grid on;
    xlabel(obj_label(obj_names, 1)); ylabel(obj_label(obj_names, min(3, numel(obj_names)))); title('Pareto: Obj1 vs Obj3');
    sgtitle('MOO Pareto Front (updating...)','FontSize',12,'FontWeight','bold');
    drawnow;
end

n_evals   = 0;
gen_count = 0;

% ── Multi-objective function ───────────────────────────────────────────────
function F = moo_fcn(x)
    n_evals = n_evals + 1;
    d = d_init;
    for ki = 1:Nvar, d.(var_names{ki}) = x(ki); end

    % Fast feasibility pre-check
    if isfield(d,'Lyo') && isfield(d,'Lyi') && d.Lyo <= d.Lyi + 0.1
        F = [1, 1, 2];   % infeasible penalty
        return
    end
    [UAM_tmp, ~, Env_tmp] = hexacopter_params(d);
    if d.T_max < UAM_tmp.m * Env_tmp.g * 1.05 / 6
        F = [1, 2, 1];
        return
    end

    r = eval_design(d, eval_opts);
    F = objective_vector_from_result(r, obj_names);
    F = max(F, 0);
    F(isnan(F)) = 1;
end

% ── gamultiobj output function (live plot) ─────────────────────────────────
function [state, opts_out, changed] = moo_output_fcn(options_in, state, flag)
    opts_out = options_in;
    changed  = false;
    gen_count = gen_count + 1;

    if ~strcmp(flag,'iter'), return; end

    % Extract current Pareto front from state
    pf_idx = state.Rank == 1;
    F_pf   = state.Score(pf_idx, :);

    if options.verbose && mod(gen_count,10) == 0
        fprintf('  [gen %3d/%d]  Pareto size=%d  evals=%d\n', ...
                gen_count, options.max_gen, sum(pf_idx), n_evals);
    end

    if options.plot_live && ~isempty(F_pf) && ishandle(hfig)
        axes(hax_par); cla; hold on; grid on;
        scatter(F_pf(:,1), F_pf(:,2), 30, F_pf(:,3), 'filled');
        colorbar; xlabel(obj_label(obj_names, 1)); ylabel(obj_label(obj_names, 2));
        title(sprintf('Pareto: Obj1 vs Obj2  [gen %d, N=%d]', gen_count, size(F_pf,1)));

        axes(hax_cost); cla; hold on; grid on;
        scatter(F_pf(:,1), F_pf(:,3), 30, F_pf(:,2), 'filled');
        colorbar; xlabel(obj_label(obj_names, 1)); ylabel(obj_label(obj_names, min(3, numel(obj_names))));
        title(sprintf('Pareto: Obj1 vs Obj3  [gen %d]', gen_count));

        sgtitle(sprintf('MOO Pareto Front  [gen %d/%d | evals=%d]', ...
                gen_count, options.max_gen, n_evals), ...
                'FontSize',11,'FontWeight','bold');
        drawnow limitrate;
    end
end

% ── Run gamultiobj ────────────────────────────────────────────────────────
if options.verbose
    fprintf('\n%s\n', repmat('=',1,65));
    fprintf('  Multi-Objective Optimization  (gamultiobj)\n');
    fprintf('  Objectives : ');
    fprintf('%s ', obj_names{:});
    fprintf('\n');
    fprintf('  Variables  : '); fprintf('%s ', var_names{:}); fprintf('\n');
    fprintf('  Pop: %d  |  MaxGen: %d\n', options.pop_size, options.max_gen);
    fprintf('%s\n', repmat('=',1,65));
    t_start = tic;
end

ga_opts = optimoptions('gamultiobj', ...
    'PopulationSize',       options.pop_size, ...
    'MaxGenerations',       options.max_gen, ...
    'ParetoFraction',       0.40, ...
    'CrossoverFraction',    0.80, ...
    'FunctionTolerance',    1e-5, ...
    'Display',              'off', ...
    'UseParallel',          false, ...
    'OutputFcn',            @moo_output_fcn);

[X_pareto, F_pareto, ~, ga_output] = gamultiobj(@moo_fcn, Nvar, ...
    [], [], [], [], lb, ub, ga_opts);

if options.verbose
    elapsed = toc(t_start);
    fprintf('\n  MOO complete  |  %.1f s  |  %d evals  |  %d Pareto solutions\n', ...
            elapsed, n_evals, size(X_pareto,1));
    for i_obj = 1:size(F_pareto,2)
        fprintf('  f%d (%s) range: [%.4f, %.4f]\n', i_obj, obj_names{i_obj}, ...
            min(F_pareto(:,i_obj)), max(F_pareto(:,i_obj)));
    end
    fprintf('%s\n', repmat('=',1,65));
end

% ── Reconstruct design structs ─────────────────────────────────────────────
N_pareto       = size(X_pareto, 1);
pareto_J       = F_pareto;
pareto_front   = true(N_pareto, 1);
pareto_designs = cell(N_pareto, 1);
for k = 1:N_pareto
    dk = d_init;
    for ki = 1:Nvar, dk.(var_names{ki}) = X_pareto(k,ki); end
    pareto_designs{k} = dk;
end

% Final Pareto plot
if options.plot_live && ishandle(hfig)
    axes(hax_par); cla; hold on; grid on;
    scatter(F_pareto(:,1), F_pareto(:,2), 50, F_pareto(:,3), 'filled', ...
            'MarkerEdgeColor','k','LineWidth',0.3);
    colorbar; c = colorbar; c.Label.String = obj_label(obj_names, min(3, numel(obj_names)));
    xlabel(obj_label(obj_names, 1)); ylabel(obj_label(obj_names, 2));
    title(sprintf('Final Pareto: Obj1 vs Obj2  [N=%d]', N_pareto));

    axes(hax_cost); cla; hold on; grid on;
    scatter(F_pareto(:,1), F_pareto(:,3), 50, F_pareto(:,2), 'filled', ...
            'MarkerEdgeColor','k','LineWidth',0.3);
    colorbar; c = colorbar; c.Label.String = obj_label(obj_names, 2);
    xlabel(obj_label(obj_names, 1)); ylabel(obj_label(obj_names, min(3, numel(obj_names))));
    title(sprintf('Final Pareto: Obj1 vs Obj3  [N=%d]', N_pareto));

    sgtitle(sprintf('MOO Final Pareto Front  [%d solutions]', N_pareto), ...
            'FontSize',12,'FontWeight','bold');
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
    options.objective_names = {'FII','hover','cost'};
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
