function [d_opt, J_opt, history] = run_cmaes(d_init, cfg)
% RUN_CMAES  CMA-ES optimizer for the UAM hexacopter MDO problem.
%
% Implements the (mu/mu_w, lambda)-CMA-ES (Covariance Matrix Adaptation
% Evolution Strategy) entirely in normalized variable space [0,1], making
% it scale-invariant across variables with different physical units.
%
% Reference: Hansen (2016) "The CMA Evolution Strategy: A Tutorial"
%            https://arxiv.org/abs/1604.00772
%
% ADVANTAGES OVER GA FOR THIS PROBLEM
%   - Adapts search distribution shape (covariance) to problem geometry
%   - Handles correlated variables (arm lengths ARE correlated by constraints)
%   - Typically converges in ~300-600 evals vs ~4000 for GA
%   - No manual tuning: all adaptation rates derived from problem dimension
%
% INPUTS
%   d_init  - Initial design struct (see design_default.m)
%   cfg     - Configuration struct from mdo_config.m
%
% OUTPUTS
%   d_opt   - Optimal design struct
%   J_opt   - Best objective value found
%   history - Struct with fields:
%               .J_all   : objective value of every evaluated candidate
%               .J_best  : best J found up to each generation
%               .x_best  : best normalized design vector (active vars only)
%               .n_evals : total function evaluations used

%% ── Extract active design variables from cfg ──────────────────────────────
active_mask = cfg.vars.active;
names       = cfg.vars.names(active_mask);
lb          = cfg.vars.lb(active_mask)';
ub          = cfg.vars.ub(active_mask)';
x0_phys     = cfg.vars.x0(active_mask)';
units_all   = cfg.vars.units;
all_names   = cfg.vars.names;
n = numel(names);

% Build the full active index list for unit labels
active_idx = find(active_mask);

% Normalize initial point to [0,1]
x0 = (x0_phys - lb) ./ (ub - lb);
x0 = max(0.01, min(0.99, x0));  % keep away from hard bounds

%% ── CMA-ES hyperparameters (Hansen 2016, Section 3) ─────────────────────
lambda = cfg.opt.pop_size;
if lambda <= 0
    lambda = 4 + floor(3 * log(n));   % default offspring count
end
mu      = floor(lambda / 2);          % number of parents

% Recombination weights
raw_w    = log(mu + 0.5) - log(1:mu)';
weights  = raw_w / sum(raw_w);        % normalized, sum = 1
mu_eff   = 1 / sum(weights.^2);       % variance-effective selection mass

% Step-size control
cs   = (mu_eff + 2) / (n + mu_eff + 5);
ds   = 1 + cs + 2 * max(0, sqrt((mu_eff - 1)/(n + 1)) - 1);

% Covariance matrix adaptation
cc   = (4 + mu_eff/n) / (n + 4 + 2*mu_eff/n);
c1   = 2 / ((n + 1.3)^2 + mu_eff);
cmu  = min(1 - c1, 2*(mu_eff - 2 + 1/mu_eff) / ((n + 2)^2 + mu_eff));

% Expected length of N(0,I)
chiN = sqrt(n) * (1 - 1/(4*n) + 1/(21*n^2));

% Eigendecomposition refresh rate
eig_period = max(1, floor(lambda / (c1 + cmu) / n / 10));

%% ── Initialize CMA-ES state ───────────────────────────────────────────────
m        = x0;           % mean (normalized)
sigma    = cfg.opt.sigma0;
pc       = zeros(n, 1); % evolution path for covariance
ps       = zeros(n, 1); % evolution path for step size
B        = eye(n);       % eigenvectors of C
D        = ones(n, 1);  % sqrt of eigenvalues of C
C        = eye(n);       % covariance matrix
invsqrtC = eye(n);       % C^(-1/2)
eigen_eval = 0;

%% ── History tracking ──────────────────────────────────────────────────────
history.J_all   = [];
history.J_best  = [];
history.x_best  = x0;
history.n_evals = 0;
best_J     = inf;
best_x_norm = x0;
last_best_eval = 0;

%% ── Evaluation options ────────────────────────────────────────────────────
eval_opts.mode    = cfg.eval.mode;
eval_opts.weights = weights_struct_to_vec(cfg.weights);
eval_opts.verbose = false;
if isfield(cfg, 'model')
    eval_opts.model = cfg.model;
    if isfield(cfg.model, 'use_vehicle_model')
        d_init.use_vehicle_model = logical(cfg.model.use_vehicle_model);
    end
end

% Fault config
fault_cfg.include_double = cfg.fault.include_double;
fault_cfg.p_motor        = cfg.fault.p_motor;
eval_opts.fault_config   = fault_cfg;

% Sim config (only used in 'sim'/'full' mode)
sim_cfg.loe_vec    = cfg.sim.loe_vec;
sim_cfg.T_end      = cfg.sim.T_end;
sim_cfg.fault_time = cfg.sim.fault_time;
sim_cfg.dt         = cfg.sim.dt;
sim_cfg.alt_cmd    = cfg.sim.alt_cmd;
eval_opts.sim_config = sim_cfg;

%% ── Live figure setup ─────────────────────────────────────────────────────
hfig = [];
if cfg.opt.plot_live
    hfig = figure('Name','CMA-ES Convergence','NumberTitle','off',...
                  'Position',[50 50 1100 500]);
    subplot(1,3,1); hax_conv = gca;
    subplot(1,3,2); hax_vars = gca;
    subplot(1,3,3); hax_met  = gca;
    drawnow;
end

%% ── Banner ────────────────────────────────────────────────────────────────
if cfg.opt.verbose
    fprintf('\n%s\n', repmat('=',1,70));
    fprintf('  CMA-ES Optimization  |  (mu/mu_w, lambda)-CMA-ES\n');
    fprintf('  Variables : '); fprintf('%s ', names{:}); fprintf('\n');
    fprintf('  n=%d  lambda=%d  mu=%d  sigma0=%.3f  budget=%d evals\n', ...
            n, lambda, mu, cfg.opt.sigma0, cfg.opt.max_evals);
    fprintf('  Adaptation rates: cs=%.3f  cc=%.3f  c1=%.4f  cmu=%.4f\n', cs, cc, c1, cmu);
    fprintf('%s\n', repmat('=',1,70));
    t_start = tic;
end

%% ── Main CMA-ES loop ──────────────────────────────────────────────────────
gen = 0;
while history.n_evals < cfg.opt.max_evals

    gen = gen + 1;

    %% Sample lambda offspring in normalized space
    arz = randn(n, lambda);
    arx_raw = m(:, ones(1,lambda)) + sigma * (B * diag(D) * arz);

    % Bounds handling via reflection (preserves CMA-ES geometry better than clamping)
    arx = reflect_bounds_matrix(arx_raw, 0, 1);

    %% Evaluate offspring
    arfitness = zeros(1, lambda);
    for k = 1:lambda
        arfitness(k) = penalized_obj(arx(:, k));
    end
    history.J_all   = [history.J_all, arfitness];
    history.n_evals = history.n_evals + lambda;

    %% Sort by fitness
    [~, arindex] = sort(arfitness);

    %% Track best
    if arfitness(arindex(1)) < best_J
        best_J      = arfitness(arindex(1));
        best_x_norm = arx(:, arindex(1));
        history.J_best(end+1) = best_J;
        history.x_best        = best_x_norm;
        last_best_eval        = history.n_evals;
        update_live_plot();

        if cfg.opt.verbose
            x_phys = lb + best_x_norm .* (ub - lb);
            fprintf('  [gen %3d | %4d evals] J=%.5f | ', gen, history.n_evals, best_J);
            for ki = 1:n
                fprintf('%s=%.3g ', names{ki}, x_phys(ki));
            end
            fprintf('\n');
        end
    end

    xold = m;

    %% Update mean
    m = arx(:, arindex(1:mu)) * weights;

    %% Update step-size evolution path (ps)
    ps = (1 - cs) * ps + ...
         sqrt(cs * (2 - cs) * mu_eff) * (invsqrtC * (m - xold) / sigma);

    %% Heaviside indicator (suppress rank-one update when ps is long)
    hsig = norm(ps) / sqrt(1 - (1-cs)^(2*gen)) / chiN < 1.4 + 2/(n+1);

    %% Update rank-one evolution path (pc)
    pc = (1 - cc) * pc + ...
         hsig * sqrt(cc * (2-cc) * mu_eff) * (m - xold) / sigma;

    %% Update covariance matrix
    artmp = (1/sigma) * (arx(:, arindex(1:mu)) - xold(:, ones(1,mu)));
    C = (1 - c1 - cmu) * C + ...
        c1 * (pc * pc' + (1 - hsig) * cc * (2 - cc) * C) + ...
        cmu * (artmp * diag(weights) * artmp');
    C = (C + C') / 2;   % enforce symmetry (numerical safety)

    %% Update step size
    sigma = sigma * exp((cs / ds) * (norm(ps) / chiN - 1));

    %% Eigendecomposition (amortized)
    if gen - eigen_eval >= eig_period
        eigen_eval = gen;
        [B, D_eig] = eig(C);
        D = sqrt(max(real(diag(D_eig)), 1e-20));  % guard negative eigenvalues
        invsqrtC = B * diag(1 ./ D) * B';
    end

    %% Convergence check
    if sigma < cfg.opt.tol_fun
        if cfg.opt.verbose
            fprintf('  CMA-ES converged: sigma=%.2e < tol_fun=%.2e\n', sigma, cfg.opt.tol_fun);
        end
        break;
    end

    % Stagnation: no improvement in last 30% of budget
    if (history.n_evals - last_best_eval) > 0.3 * cfg.opt.max_evals && gen > 20
        if cfg.opt.verbose
            fprintf('  CMA-ES stagnated: no improvement for %d evals\n', ...
                    history.n_evals - last_best_eval);
        end
        break;
    end
end

%% ── Optional fmincon polish (in physical space) ───────────────────────────
if cfg.opt.use_polish && best_J < 1e5
    x_phys_best = lb + best_x_norm .* (ub - lb);
    polish_con  = @(xp) constraint_fcn(xp, names, d_init);
    fmc_opts    = optimoptions('fmincon', 'Algorithm','sqp', 'Display','off', ...
                               'MaxFunctionEvaluations', 300, ...
                               'OptimalityTolerance', 1e-7, ...
                               'ConstraintTolerance', 1e-5);
    try
        [xp_opt, Jp] = fmincon(@(xp) polish_obj(xp), x_phys_best, ...
                               [], [], [], [], lb, ub, polish_con, fmc_opts);
        if Jp < best_J
            best_J      = Jp;
            best_x_norm = (xp_opt - lb) ./ (ub - lb);
            history.J_best(end+1) = best_J;
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

%% ── Reconstruct optimal design ────────────────────────────────────────────
J_opt = best_J;
d_opt = d_init;
x_opt_phys = lb + best_x_norm .* (ub - lb);
for k = 1:n
    d_opt.(names{k}) = x_opt_phys(k);
end

if cfg.opt.verbose
    elapsed = toc(t_start);
    fprintf('\n%s\n', repmat('-',1,70));
    fprintf('  CMA-ES complete | %.1f s | %d evals | J_opt = %.6f\n', ...
            elapsed, history.n_evals, J_opt);
    fprintf('  Optimal design:\n');
    for k = 1:n
        fprintf('    %-8s = %10.4g  %s\n', names{k}, x_opt_phys(k), ...
                units_all{active_idx(k)});
    end
    fprintf('%s\n', repmat('=',1,70));
end

%% ── Nested: penalized objective (normalized space) ────────────────────────
function Jout = penalized_obj(x_norm)
    x_phys = lb + x_norm .* (ub - lb);
    d_tmp  = d_init;
    for ki = 1:n
        d_tmp.(names{ki}) = x_phys(ki);
    end

    % Geometric pre-check (fast, avoids ACS computation for invalid designs)
    if isfield(d_tmp,'Lyo') && isfield(d_tmp,'Lyi')
        geo_gap = d_tmp.Lyo - d_tmp.Lyi;
        if geo_gap < 0.1
            Jout = 1e6 + 1e4 * max(0, 0.1 - geo_gap);
            return;
        end
    end

    r = eval_design(d_tmp, eval_opts);
    if ~r.feasible || ~isfinite(r.J_combined)
        Jout = 1e6;
    else
        Jout = r.J_combined;
    end
end

%% ── Nested: polish objective (physical space) ─────────────────────────────
function Jout = polish_obj(x_phys)
    d_tmp = d_init;
    for ki = 1:n
        d_tmp.(names{ki}) = x_phys(ki);
    end
    r = eval_design(d_tmp, eval_opts);
    if ~r.feasible || ~isfinite(r.J_combined)
        Jout = 1e6;
    else
        Jout = r.J_combined;
    end
end

%% ── Nested: update live figure ────────────────────────────────────────────
function update_live_plot()
    if isempty(hfig) || ~ishandle(hfig)
        return;
    end
    axes(hax_conv); cla;
    plot(history.J_all, '.', 'Color',[0.82 0.82 0.82], 'MarkerSize',3);
    hold on;
    plot(cummin(history.J_all), 'b-', 'LineWidth',2);
    xlabel('Evaluation'); ylabel('J');
    title(sprintf('CMA-ES  [best=%.5f, gen=%d]', best_J, gen));
    grid on;

    axes(hax_vars); cla;
    bar(best_x_norm, 0.6, 'FaceColor',[0.2 0.6 0.85]);
    set(gca, 'XTick',1:n, 'XTickLabel',names, 'XTickLabelRotation',30);
    ylim([0 1]);
    ylabel('Normalized [0=lb, 1=ub]');
    title('Best Design Variables'); grid on;

    axes(hax_met); cla;
    % Quick eval for metric bars (re-use last result if available)
    x_phys = lb + best_x_norm .* (ub - lb);
    d_tmp  = d_init;
    for ki = 1:n, d_tmp.(names{ki}) = x_phys(ki); end
    r = eval_design(d_tmp, eval_opts);
    if ~isempty(r.acs)
        vals = [r.acs.FII, max(0, -r.acs.hover_margin), r.J_cost];
        bar(vals, 0.5, 'FaceColor',[0.9 0.4 0.2]);
        set(gca, 'XTick',1:3, 'XTickLabel',{'FII','Hover deficit','J\_cost'});
        title(sprintf('Metrics  [J=%.4f]', best_J)); grid on;
    end
    drawnow limitrate;
end

end  % run_cmaes

%% ── Helpers (file-level, not nested) ─────────────────────────────────────

function arx = reflect_bounds_matrix(arx_raw, lo, hi)
% Reflect out-of-bounds values back into [lo, hi].
% Preserves search distribution shape better than clamping.
    range = hi - lo;
    arx   = lo + mod(arx_raw - lo, 2 * range);
    mask  = arx > hi;
    arx(mask) = 2*hi - arx(mask);
end

function w_vec = weights_struct_to_vec(w_struct)
% Convert weights struct to [FII, hover, mission, cost] vector.
    w_vec = [w_struct.FII, w_struct.hover, w_struct.mission, w_struct.cost];
end
