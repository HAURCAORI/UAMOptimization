%% MAIN_MDO.M  —  MDO Framework: Fault-Tolerant Hexacopter UAM Design
%
%   AE50001 MDO Course Project  /  Team 5  /  2026 Spring
%   Topic: "Systematic Design of Hexacopter UAM under Rotor Fault Situations"
%
% =========================================================================
%   THEORETICAL BACKGROUND
% =========================================================================
%   This framework evaluates hexacopter designs using two complementary
%   approaches:
%
%   1. ATTAINABLE CONTROL SET (ACS) ANALYSIS
%      The ACS = {B·T : 0 ≤ T_i ≤ T_max} is the set of all achievable
%      virtual controls [Fz; L; M; N].  Under motor fault (LOE), the ACS
%      shrinks, reducing control authority.
%
%   2. CLOSED-LOOP SIMULATION
%      A cascade PI controller runs a fault-injection scenario.
%      Performance metrics (altitude RMSE, attitude excursion, recovery
%      time) quantify mission-level fault tolerance.
%
% =========================================================================
%   KEY DESIGN FINDINGS (from design-space analysis)
% =========================================================================
%   (A) ZONOTOPE RETENTION IDENTITY
%       For this tandem hexacopter, the mean single-fault ACS volume
%       retention equals exactly 1/3 regardless of arm geometry.
%       Proof: each of 6 generators contributes equally to the zonotope
%       volume in expectation; removing one reduces the mean by 1/6.
%
%   (B) FAULT ISOTROPY INDEX (FII) — novel metric
%       While the mean retention is fixed, its variance across motors is
%       geometry-dependent.  FII = std(r_i)/mean(r_i) ∈ [0,∞) measures
%       how uniformly fault tolerance is distributed.  FII=0 means all
%       motor failures are equally impactful (ideal balanced design).
%       FII is minimized at Lyi ≈ 4.0 m for this vehicle.
%
%   (C) HOVER FEASIBILITY THRESHOLD
%       Motors 3 and 5 (mid-row) require T_max ≥ 3.0×(m·g/6) for hover
%       after failure.  This is geometry-invariant for this configuration.
%       Baseline T_max = 2.0×(m·g/6) → motors 3,5 cannot hover after fault.
%
%   (D) MDO PARETO TRADE-OFF
%       f1 = FII  (wants Lyi ≈ 4m)
%       f2 = hover_margin  (wants T_max ≥ 3×hover share)
%       f3 = motor+structural cost  (wants small Lyi, small T_max)
%
% =========================================================================
%   FRAMEWORK STRUCTURE
% =========================================================================
%   core/            design_default, hexacopter_params, build_B_matrix, eom_hex
%   evaluation/      eval_acs, eval_simulation, eval_design
%   metrics/         compute_acs_volume, hover_feasibility,
%                    compute_hover_threshold, fault_isotropy_index
%   optimization/    run_soo (GA+fmincon), run_moo (gamultiobj)
%                    objective_fcn, constraint_fcn
%   analysis/        sweep_design_space, pareto_analysis,
%                    compare_designs, visualize_results
%
% =========================================================================
%   HOW TO RUN
% =========================================================================
%   Run sections in order.  Each section saves its result to the workspace.
%   Sections 3 and 4 (optimization) may take several minutes.
%   Section timer is printed at the end of each section.

%% Initialization
clc; clear; close all;
set(0,'DefaultFigureWindowStyle','docked');

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   UAM Hexacopter Fault-Tolerant MDO Framework               ║\n');
fprintf('║   AE50001 Team 5  │  2026 Spring                            ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

% Add framework paths
fw = fileparts(mfilename('fullpath'));
addpath(fullfile(fw,'config'));
addpath(fullfile(fw,'core'));
addpath(fullfile(fw,'evaluation'));
addpath(fullfile(fw,'metrics'));
addpath(fullfile(fw,'optimization'));
addpath(fullfile(fw,'analysis'));

% ── Master configuration (all hyperparameters in one place) ──────────────
cfg = mdo_config();

%% ════════════════════════════════════════════════════════════════════════
%  SECTION 1 — Baseline Design Evaluation
% ════════════════════════════════════════════════════════════════════════
fprintf('━━━ Section 1: Baseline Evaluation ━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
t_sec = tic;

d_base = design_default();

% ── ACS metrics ────────────────────────────────────────────────────────
eval_acs_opts.mode    = 'acs';
eval_acs_opts.verbose = true;
eval_acs_opts.fault_config.include_double = true;   % full analysis for baseline
result_base_acs = eval_design(d_base, eval_acs_opts);

% ── Simulation (motor 1 fault — hover-feasible case) ──────────────────
fprintf('\n  Running simulation: motor 1 fault (feasible case)...\n');
eval_sim_opts1.mode                    = 'sim';
eval_sim_opts1.loe_for_sim             = [1;0;0;0;0;0];
eval_sim_opts1.sim_config.t_fault      = 10;
eval_sim_opts1.sim_config.t_end        = 40;
eval_sim_opts1.sim_config.dt           = 0.005;
eval_sim_opts1.sim_config.alt_cmd      = 10;
result_base_sim1 = eval_design(d_base, eval_sim_opts1);

% ── Simulation (motor 3 fault — hover-infeasible case) ────────────────
fprintf('  Running simulation: motor 3 fault (infeasible case)...\n');
eval_sim_opts3                         = eval_sim_opts1;
eval_sim_opts3.loe_for_sim             = [0;0;1;0;0;0];
result_base_sim3 = eval_design(d_base, eval_sim_opts3);

% ── Visualize baseline ─────────────────────────────────────────────────
visualize_results(result_base_acs, 'acs');
result_base_sim1.sim.t_fault = 10;  % pass for plotting
visualize_results(result_base_sim1, 'sim');

fprintf('\n  Section 1 complete [%.1f s]\n\n', toc(t_sec));

%% ════════════════════════════════════════════════════════════════════════
%  SECTION 2 — Design Space Sensitivity Sweeps
% ════════════════════════════════════════════════════════════════════════
fprintf('━━━ Section 2: Sensitivity Sweeps ━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
t_sec = tic;

sw_opts.eval_mode = 'acs';
sw_opts.verbose   = false;

% 2a. Lyi sweep: geometry driver for FII
fprintf('  Sweeping Lyi...\n');
sw_Lyi = sweep_design_space(d_base, 'Lyi', linspace(1.0, 5.49, 40), sw_opts);

% 2b. T_max sweep: hover feasibility driver
fprintf('  Sweeping T_max...\n');
sw_Tmax = sweep_design_space(d_base, 'T_max', linspace(5000, 18000, 40), sw_opts);

% 2c. 2D sweep: Lyi × T_max (the two main independent design levers)
fprintf('  Running 2D sweep: Lyi × T_max...\n');
sw_2d = sweep_design_space(d_base, ...
    {'Lyi','T_max'}, {linspace(1.5,5.5,20), linspace(7000,16000,20)}, sw_opts);

visualize_results(sw_Lyi,  'sweep');
visualize_results(sw_Tmax, 'sweep');
visualize_results(sw_2d,   'sweep');

fprintf('\n  Section 2 complete [%.1f s]\n\n', toc(t_sec));

%% ════════════════════════════════════════════════════════════════════════
%  SECTION 3 — Single-Objective Optimization (CMA-ES, config-driven)
% ════════════════════════════════════════════════════════════════════════
fprintf('━━━ Section 3: Single-Objective Optimization ━━━━━━━━━━━━━━━━━\n');
t_sec = tic;

% All settings come from mdo_config.m — edit that file to change optimizer,
% weights, variable bounds, or evaluation mode.
[d_opt_soo, J_soo, hist_soo] = run_soo(d_base, cfg);

% Full ACS + sim validation of optimal design
fprintf('\n  Full validation of SOO-optimal design:\n');
eval_full.mode         = 'full';
eval_full.verbose      = true;
eval_full.loe_for_sim  = [0;0;1;0;0;0];   % motor 3 (previously infeasible at baseline)
eval_full.sim_config   = eval_sim_opts3.sim_config;
result_soo = eval_design(d_opt_soo, eval_full);

fprintf('\n  Section 3 complete [%.1f s]\n\n', toc(t_sec));

%% ════════════════════════════════════════════════════════════════════════
%  SECTION 4 — Multi-Objective Optimization (Pareto Front)
% ════════════════════════════════════════════════════════════════════════
fprintf('━━━ Section 4: Multi-Objective Optimization (Pareto) ━━━━━━━━━\n');
t_sec = tic;

moo_opts.var_names  = {'Lx','Lyi','Lyo','T_max'};
moo_opts.lb         = [1.0,  1.0, 2.5,  8000];
moo_opts.ub         = [5.0,  5.0, 9.0, 16000];
moo_opts.eval_mode  = 'acs';
moo_opts.pop_size   = 100;
moo_opts.max_gen    = 150;
moo_opts.plot_live  = true;
moo_opts.verbose    = true;

[pareto_designs, pareto_J, ~, ~] = run_moo(d_base, moo_opts);

% Pareto analysis: filter, find knee, visualize
[pareto_J_nd, pareto_nd, knee_idx] = pareto_analysis(pareto_designs, pareto_J, ...
    struct('plot', true, 'f_labels', {{'J_{FII}','J_{hover}','J_{cost}'}}));

d_knee = pareto_nd{knee_idx};
fprintf('\n  Full validation of Pareto knee design:\n');
result_knee = eval_design(d_knee, struct('mode','full','verbose',true,...
    'loe_for_sim',[0;0;1;0;0;0],'sim_config',eval_sim_opts3.sim_config));

fprintf('\n  Section 4 complete [%.1f s]\n\n', toc(t_sec));

%% ════════════════════════════════════════════════════════════════════════
%  SECTION 5 — Three-Way Design Comparison
% ════════════════════════════════════════════════════════════════════════
fprintf('━━━ Section 5: Design Comparison ━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
t_sec = tic;

compare_opts.eval_mode  = 'full';
compare_opts.sim_loe    = [0;0;1;0;0;0];   % motor 3 (worst case)
compare_opts.sim_config = eval_sim_opts3.sim_config;
compare_opts.plot       = true;

compare_designs( ...
    {d_base, d_opt_soo, d_knee}, ...
    {'Baseline (2× hover)', 'SOO-Optimal', 'Pareto Knee'}, ...
    compare_opts);

fprintf('\n  Section 5 complete [%.1f s]\n\n', toc(t_sec));

%% ════════════════════════════════════════════════════════════════════════
%  SECTION 6 — ACS Geometry Visualization
% ════════════════════════════════════════════════════════════════════════
fprintf('━━━ Section 6: ACS Geometry Visualization ━━━━━━━━━━━━━━━━━━━\n');
t_sec = tic;

figure('Name','ACS [L,M,N] Projection — All Three Designs');
design_set  = {d_base, d_opt_soo, d_knee};
design_lbls = {'Baseline','SOO-Optimal','Pareto Knee'};
face_colors = {[0.25 0.55 0.90], [0.15 0.72 0.27], [0.92 0.42 0.14]};

for di = 1:3
    subplot(2,3,di);
    [UAM_d, Prop_d] = hexacopter_params(design_set{di});
    plot_acs_3d(UAM_d.B, Prop_d.T_max, zeros(6,1), face_colors{di}, 0.30);
    title(sprintf('%s  (nominal)', design_lbls{di}));
    xlabel('L'); ylabel('M'); zlabel('N'); grid on; view(30,20);

    subplot(2,3,di+3);
    [~,worst_k] = min(eval_acs(design_set{di}).single_retention);
    loe_w = zeros(6,1); loe_w(worst_k) = 1;
    plot_acs_3d(UAM_d.B, Prop_d.T_max, loe_w, [0.90 0.25 0.20], 0.40);
    title(sprintf('%s  (M%d fault)', design_lbls{di}, worst_k));
    xlabel('L'); ylabel('M'); zlabel('N'); grid on; view(30,20);
end
sgtitle('ACS [L,M,N] Projection: Nominal vs Worst-Case Fault', ...
        'FontSize',12,'FontWeight','bold');

fprintf('\n  Section 6 complete [%.1f s]\n\n', toc(t_sec));

%% ════════════════════════════════════════════════════════════════════════
%  SUMMARY
% ════════════════════════════════════════════════════════════════════════
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   MDO SUMMARY                                                ║\n');
fprintf('╠══════════════════════════════════════════════════════════════╣\n');

designs_all = {d_base, d_opt_soo, d_knee};
names_all   = {'Baseline','SOO-Opt','Pareto-Knee'};
for k = 1:3
    r = eval_design(designs_all{k}, struct('mode','acs','verbose',false));
    fprintf('║  %-12s │ FII=%.4f │ H_margin=%+.3f │ J=%.4f     ║\n', ...
            names_all{k}, r.acs.FII, r.acs.hover_margin, r.J_combined);
end
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

%% ── Local helper: plot 3D ACS projection [L,M,N] ────────────────────────
function plot_acs_3d(B, T_max, loe_vec, fcolor, alpha)
    [~, pts4d] = compute_acs_volume(B, T_max, loe_vec);
    pts = unique(round(pts4d(:,[2,3,4]),10),'rows');
    if size(pts,1) >= 4 && rank(pts - mean(pts)) >= 3
        K = convhulln(pts,{'Qt','Qx'});
        trisurf(K, pts(:,1), pts(:,2), pts(:,3), ...
                'FaceColor', fcolor, 'FaceAlpha', alpha, ...
                'EdgeColor', fcolor * 0.55, 'LineWidth', 0.4);
    end
end
