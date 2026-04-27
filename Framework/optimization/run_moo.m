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
if ~isfield(options,'var_names'),  options.var_names  = {'Lx','Lyi','Lyo','T_max'}; end
if ~isfield(options,'eval_mode'),  options.eval_mode  = 'acs';   end
if ~isfield(options,'pop_size'),   options.pop_size   = 100;     end
if ~isfield(options,'max_gen'),    options.max_gen    = 150;     end
if ~isfield(options,'plot_live'),  options.plot_live  = true;    end
if ~isfield(options,'verbose'),    options.verbose    = true;    end

var_names = options.var_names;
Nvar      = numel(var_names);

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

% Live figure
if options.plot_live
    hfig = figure('Name','MOO Pareto Front (live)','NumberTitle','off',...
                  'Position',[100 100 900 420]);
    subplot(1,2,1); hax_par  = gca; hold on; grid on;
    xlabel('J_{FII}'); ylabel('J_{hover}'); title('Pareto: FII vs Hover');
    subplot(1,2,2); hax_cost = gca; hold on; grid on;
    xlabel('J_{FII}'); ylabel('J_{cost}'); title('Pareto: FII vs Cost');
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
    if d.T_max < d.m * 9.81 / 6
        F = [1, 2, 1];
        return
    end

    r = eval_design(d, eval_opts);

    f1 = r.J_FII;
    f2 = r.J_hover;
    f3 = r.J_cost;

    F = [max(f1,0), max(f2,0), max(f3,0)];
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
        colorbar; xlabel('J_{FII}'); ylabel('J_{hover}');
        title(sprintf('Pareto: FII vs Hover  [gen %d, N=%d]', gen_count, size(F_pf,1)));

        axes(hax_cost); cla; hold on; grid on;
        scatter(F_pf(:,1), F_pf(:,3), 30, F_pf(:,2), 'filled');
        colorbar; xlabel('J_{FII}'); ylabel('J_{cost}');
        title(sprintf('Pareto: FII vs Cost  [gen %d]', gen_count));

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
    fprintf('  Objectives : [J_FII, J_hover, J_cost]\n');
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
    fprintf('  f1 (FII)   range: [%.4f, %.4f]\n', min(F_pareto(:,1)), max(F_pareto(:,1)));
    fprintf('  f2 (hover) range: [%.4f, %.4f]\n', min(F_pareto(:,2)), max(F_pareto(:,2)));
    fprintf('  f3 (cost)  range: [%.4f, %.4f]\n', min(F_pareto(:,3)), max(F_pareto(:,3)));
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
    colorbar; c = colorbar; c.Label.String = 'J_{cost}';
    xlabel('J_{FII}'); ylabel('J_{hover}');
    title(sprintf('Final Pareto: FII vs Hover  [N=%d]', N_pareto));

    axes(hax_cost); cla; hold on; grid on;
    scatter(F_pareto(:,1), F_pareto(:,3), 50, F_pareto(:,2), 'filled', ...
            'MarkerEdgeColor','k','LineWidth',0.3);
    colorbar; c = colorbar; c.Label.String = 'J_{hover}';
    xlabel('J_{FII}'); ylabel('J_{cost}');
    title(sprintf('Final Pareto: FII vs Cost  [N=%d]', N_pareto));

    sgtitle(sprintf('MOO Final Pareto Front  [%d solutions]', N_pareto), ...
            'FontSize',12,'FontWeight','bold');
    drawnow;
end
end
