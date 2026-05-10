function [d_opt, J_opt, history] = run_cmaes(d_init, cfg)
% RUN_CMAES  CMA-ES optimizer in normalized variable space.

active_mask = cfg.vars.active;
names = cfg.vars.names(active_mask);
lb = cfg.vars.lb(active_mask)';
ub = cfg.vars.ub(active_mask)';
x0_phys = cfg.vars.x0(active_mask)';
units_all = cfg.vars.units;
active_idx = find(active_mask);
n = numel(names);

x0 = (x0_phys - lb) ./ (ub - lb);
x0 = max(0.01, min(0.99, x0));

lambda = cfg.opt.pop_size;
if lambda <= 0
    lambda = 4 + floor(3 * log(n));
end
mu = floor(lambda / 2);

raw_w = log(mu + 0.5) - log(1:mu)';
weights = raw_w / sum(raw_w);
mu_eff = 1 / sum(weights.^2);

cs = (mu_eff + 2) / (n + mu_eff + 5);
ds = 1 + cs + 2 * max(0, sqrt((mu_eff - 1) / (n + 1)) - 1);
cc = (4 + mu_eff / n) / (n + 4 + 2 * mu_eff / n);
c1 = 2 / ((n + 1.3)^2 + mu_eff);
cmu = min(1 - c1, 2 * (mu_eff - 2 + 1 / mu_eff) / ((n + 2)^2 + mu_eff));
chiN = sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n^2));
eig_period = max(1, floor(lambda / (c1 + cmu) / n / 10));

m = x0;
sigma = cfg.opt.sigma0;
pc = zeros(n, 1);
ps = zeros(n, 1);
B = eye(n);
D = ones(n, 1);
C = eye(n);
invsqrtC = eye(n);
eigen_eval = 0;

history.J_all = [];
history.J_best = [];
history.x_best = x0;
history.n_evals = 0;
best_J = inf;
best_x_norm = x0;
last_best_eval = 0;

eval_opt = UAMOptions(cfg, ...
    'Mode', cfg.eval.mode, ...
    'Objectives', cfg.objectives.stage1, ...
    'Model', get_field_or_default(cfg, 'model', struct()), ...
    'Verbose', false);
if isfield(cfg, 'model') && isfield(cfg.model, 'use_vehicle_model')
    d_init.use_vehicle_model = logical(cfg.model.use_vehicle_model);
end

if cfg.opt.plot_live
    hfig = figure('Name', 'CMA-ES Convergence', 'NumberTitle', 'off', ...
        'Position', [50 50 1100 500]);
    subplot(1, 3, 1); hax_conv = gca;
    subplot(1, 3, 2); hax_vars = gca;
    subplot(1, 3, 3); hax_met = gca;
    drawnow;
else
    hfig = [];
    hax_conv = [];
    hax_vars = [];
    hax_met = [];
end

if cfg.opt.verbose
    fprintf('\n%s\n', repmat('=', 1, 70));
    fprintf('  CMA-ES Optimization\n');
    fprintf('  Variables : %s\n', strjoin(names, ' '));
    fprintf('  n=%d  lambda=%d  mu=%d  sigma0=%.3f  budget=%d evals\n', ...
        n, lambda, mu, cfg.opt.sigma0, cfg.opt.max_evals);
    fprintf('  Adaptation rates: cs=%.3f  cc=%.3f  c1=%.4f  cmu=%.4f\n', cs, cc, c1, cmu);
    fprintf('%s\n', repmat('=', 1, 70));
    t_start = tic;
end

gen = 0;
while history.n_evals < cfg.opt.max_evals
    gen = gen + 1;

    arz = randn(n, lambda);
    arx_raw = m(:, ones(1, lambda)) + sigma * (B * diag(D) * arz);
    arx = reflect_bounds_matrix(arx_raw, 0, 1);

    arfitness = zeros(1, lambda);
    for k = 1:lambda
        arfitness(k) = penalized_obj(arx(:, k));
    end
    history.J_all = [history.J_all, arfitness];
    history.n_evals = history.n_evals + lambda;

    [~, arindex] = sort(arfitness);

    if arfitness(arindex(1)) < best_J
        best_J = arfitness(arindex(1));
        best_x_norm = arx(:, arindex(1));
        history.J_best(end + 1) = best_J;
        history.x_best = best_x_norm;
        last_best_eval = history.n_evals;
        update_live_plot();

        if cfg.opt.verbose
            x_phys = lb + best_x_norm .* (ub - lb);
            fprintf('  [gen %3d | %4d evals] J=%.5f | ', gen, history.n_evals, best_J);
            print_named_values(names, x_phys);
            fprintf('\n');
        end
    end

    xold = m;
    m = arx(:, arindex(1:mu)) * weights;

    ps = (1 - cs) * ps + ...
        sqrt(cs * (2 - cs) * mu_eff) * (invsqrtC * (m - xold) / sigma);
    hsig = norm(ps) / sqrt(1 - (1 - cs)^(2 * gen)) / chiN < 1.4 + 2 / (n + 1);
    pc = (1 - cc) * pc + ...
        hsig * sqrt(cc * (2 - cc) * mu_eff) * (m - xold) / sigma;

    artmp = (1 / sigma) * (arx(:, arindex(1:mu)) - xold(:, ones(1, mu)));
    C = (1 - c1 - cmu) * C + ...
        c1 * (pc * pc' + (1 - hsig) * cc * (2 - cc) * C) + ...
        cmu * (artmp * diag(weights) * artmp');
    C = (C + C') / 2;

    sigma = sigma * exp((cs / ds) * (norm(ps) / chiN - 1));

    if gen - eigen_eval >= eig_period
        eigen_eval = gen;
        [B, D_eig] = eig(C);
        D = sqrt(max(real(diag(D_eig)), 1e-20));
        invsqrtC = B * diag(1 ./ D) * B';
    end

    if sigma < cfg.opt.tol_fun
        if cfg.opt.verbose
            fprintf('  CMA-ES converged: sigma=%.2e < tol_fun=%.2e\n', sigma, cfg.opt.tol_fun);
        end
        break;
    end

    if (history.n_evals - last_best_eval) > 0.3 * cfg.opt.max_evals && gen > 20
        if cfg.opt.verbose
            fprintf('  CMA-ES stagnated: no improvement for %d evals\n', ...
                history.n_evals - last_best_eval);
        end
        break;
    end
end

if cfg.opt.use_polish && best_J < 1e5
    x_phys_best = lb + best_x_norm .* (ub - lb);
    polish_con = @(xp) constraint_fcn(xp, cfg, d_init);
    fmc_opts = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off', ...
        'MaxFunctionEvaluations', 300, ...
        'OptimalityTolerance', 1e-7, ...
        'ConstraintTolerance', 1e-5);
    try
        [xp_opt, Jp] = fmincon(@polish_obj, x_phys_best, ...
            [], [], [], [], lb, ub, polish_con, fmc_opts);
        if Jp < best_J
            best_J = Jp;
            best_x_norm = (xp_opt - lb) ./ (ub - lb);
            history.J_best(end + 1) = best_J;
            if cfg.opt.verbose
                fprintf('  fmincon polish improved: J=%.6f\n', best_J);
            end
        end
    catch ME
        if cfg.opt.verbose
            fprintf('  fmincon polish failed: %s\n', ME.message);
        end
    end
end

J_opt = best_J;
d_opt = d_init;
x_opt_phys = lb + best_x_norm .* (ub - lb);
d_opt = assign_design_vars(d_opt, names, x_opt_phys);

if cfg.opt.verbose
    elapsed = toc(t_start);
    fprintf('\n%s\n', repmat('-', 1, 70));
    fprintf('  CMA-ES complete | %.1f s | %d evals | J_opt = %.6f\n', ...
        elapsed, history.n_evals, J_opt);
    fprintf('  Optimal design:\n');
    for k = 1:n
        fprintf('    %-8s = %10.4g  %s\n', names{k}, x_opt_phys(k), units_all{active_idx(k)});
    end
    fprintf('%s\n', repmat('=', 1, 70));
end

    function Jout = penalized_obj(x_norm)
        x_phys = lb + x_norm .* (ub - lb);
        d_tmp = assign_design_vars(d_init, names, x_phys);

        if isfield(d_tmp, 'Lyo') && isfield(d_tmp, 'Lyi')
            geo_gap = d_tmp.Lyo - d_tmp.Lyi;
            if geo_gap < 0.1
                Jout = 1e6 + 1e4 * max(0, 0.1 - geo_gap);
                return;
            end
        end

        r = eval_design(d_tmp, eval_opt);
        if ~r.feasible || ~isfinite(r.J_combined)
            Jout = 1e6;
        else
            Jout = r.J_combined;
        end
    end

    function Jout = polish_obj(x_phys)
        d_tmp = assign_design_vars(d_init, names, x_phys);
        r = eval_design(d_tmp, eval_opt);
        if ~r.feasible || ~isfinite(r.J_combined)
            Jout = 1e6;
        else
            Jout = r.J_combined;
        end
    end

    function update_live_plot()
        if isempty(hfig) || ~ishandle(hfig)
            return;
        end
        axes(hax_conv); cla;
        plot(history.J_all, '.', 'Color', [0.82 0.82 0.82], 'MarkerSize', 3);
        hold on;
        plot(cummin(history.J_all), 'b-', 'LineWidth', 2);
        xlabel('Evaluation');
        ylabel('J');
        title(sprintf('CMA-ES  [best=%.5f, gen=%d]', best_J, gen));
        grid on;

        axes(hax_vars); cla;
        bar(best_x_norm, 0.6, 'FaceColor', [0.2 0.6 0.85]);
        set(gca, 'XTick', 1:n, 'XTickLabel', names, 'XTickLabelRotation', 30);
        ylim([0 1]);
        ylabel('Normalized [0=lb, 1=ub]');
        title('Best Design Variables');
        grid on;

        axes(hax_met); cla;
        x_phys = lb + best_x_norm .* (ub - lb);
        d_tmp = assign_design_vars(d_init, names, x_phys);
        r = eval_design(d_tmp, eval_opt);
        if ~isempty(r.objective_vector)
            bar(r.objective_vector, 0.5, 'FaceColor', [0.9 0.4 0.2]);
            set(gca, 'XTick', 1:numel(r.objective_vector), 'XTickLabel', r.objective_names);
            title(sprintf('Metrics  [J=%.4f]', best_J));
            grid on;
        end
        drawnow limitrate;
    end
end

function arx = reflect_bounds_matrix(arx_raw, lo, hi)
range = hi - lo;
arx = lo + mod(arx_raw - lo, 2 * range);
mask = arx > hi;
arx(mask) = 2 * hi - arx(mask);
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

function print_named_values(names, values)
for idx = 1:numel(names)
    fprintf('%s=%.3g ', names{idx}, values(idx));
end
end
