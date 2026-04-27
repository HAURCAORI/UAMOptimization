%% MAIN_MDO
% Demonstration script for the UAM hexacopter MDO framework.
%
% The script is organized around the current project workflow:
%   1. Baseline evaluation
%   2. Stage 1 design sweeps
%   3. Stage 1 single-objective optimization
%   4. Stage 1 multi-objective optimization
%   5. Design comparison
%   6. Stage 2 mission verification
%   7. ACS geometry visualization

%% Initialization
clc; clear; close all;
set(0,'DefaultFigureWindowStyle','docked');

fprintf('===============================================================\n');
fprintf(' UAM Hexacopter Fault-Tolerant MDO Framework\n');
fprintf(' AE50001 Team 5 / 2026 Spring\n');
fprintf('===============================================================\n\n');

fw = fileparts(mfilename('fullpath'));
addpath(fullfile(fw,'config'));
addpath(fullfile(fw,'core'));
addpath(fullfile(fw,'evaluation'));
addpath(fullfile(fw,'metrics'));
addpath(fullfile(fw,'optimization'));
addpath(fullfile(fw,'analysis'));

cfg = mdo_config();

%% Section 1: Baseline Evaluation
fprintf('Section 1: Baseline Evaluation\n');
t_sec = tic;

d_base = design_default();

eval_acs_opts.mode = 'acs';
eval_acs_opts.verbose = true;
eval_acs_opts.fault_config.include_double = true;
result_base_acs = eval_design(d_base, eval_acs_opts);

fprintf('\n  Running simulation: motor 1 fault (hover-feasible case)...\n');
eval_sim_opts1.mode = 'sim';
eval_sim_opts1.loe_for_sim = [1;0;0;0;0;0];
eval_sim_opts1.sim_config = cfg.sim;
eval_sim_opts1.sim_config.t_fault = 10;
eval_sim_opts1.sim_config.t_end = 40;
eval_sim_opts1.sim_config.dt = 0.005;
result_base_sim1 = eval_design(d_base, eval_sim_opts1);

fprintf('  Running simulation: motor 3 fault (hover-infeasible baseline case)...\n');
eval_sim_opts3 = eval_sim_opts1;
eval_sim_opts3.loe_for_sim = [0;0;1;0;0;0];
result_base_sim3 = eval_design(d_base, eval_sim_opts3);

visualize_results(result_base_acs, 'acs');
result_base_sim1.sim.t_fault = 10;
visualize_results(result_base_sim1, 'sim');

fprintf('\n  Section 1 complete [%.1f s]\n\n', toc(t_sec));

%% Section 2: Sensitivity Sweeps
fprintf('Section 2: Sensitivity Sweeps\n');
t_sec = tic;

sw_opts.eval_mode = 'acs';
sw_opts.verbose = false;

fprintf('  Sweeping Lyi...\n');
sw_Lyi = sweep_design_space(d_base, 'Lyi', linspace(1.0, 5.49, 40), sw_opts);

fprintf('  Sweeping T_max...\n');
sw_Tmax = sweep_design_space(d_base, 'T_max', linspace(5000, 18000, 40), sw_opts);

fprintf('  Running 2D sweep: Lyi x T_max...\n');
sw_2d = sweep_design_space(d_base, ...
    {'Lyi','T_max'}, {linspace(1.5,5.5,20), linspace(7000,16000,20)}, sw_opts);

visualize_results(sw_Lyi, 'sweep');
visualize_results(sw_Tmax, 'sweep');
visualize_results(sw_2d, 'sweep');

fprintf('\n  Section 2 complete [%.1f s]\n\n', toc(t_sec));

%% Section 3: Stage 1 Single-Objective Optimization
fprintf('Section 3: Single-Objective Optimization\n');
t_sec = tic;

[d_opt_soo, J_soo, hist_soo] = run_soo(d_base, cfg); %#ok<NASGU,ASGLU>

fprintf('\n  Full validation of SOO-optimal design:\n');
eval_full.mode = 'full';
eval_full.verbose = true;
eval_full.loe_for_sim = [0;0;1;0;0;0];
eval_full.sim_config = eval_sim_opts3.sim_config;
result_soo = eval_design(d_opt_soo, eval_full);

fprintf('\n  Section 3 complete [%.1f s]\n\n', toc(t_sec));

%% Section 4: Stage 1 Multi-Objective Optimization
fprintf('Section 4: Multi-Objective Optimization\n');
t_sec = tic;

moo_cfg = cfg;
moo_cfg.opt.verbose = true;
moo_cfg.opt.ga_pop = 100;
moo_cfg.opt.max_iter = 150;
moo_cfg.eval.mode = 'acs';

[pareto_designs, pareto_J, ~, ~] = run_moo(d_base, moo_cfg);

moo_labels = strcat('J_{', cfg.objectives.stage1.moo_names, '}');
[pareto_J_nd, pareto_nd, knee_idx] = pareto_analysis(pareto_designs, pareto_J, ...
    struct('plot', true, 'f_labels', {moo_labels})); %#ok<NASGU>

d_knee = pareto_nd{knee_idx};
fprintf('\n  Full validation of Pareto knee design:\n');
result_knee = eval_design(d_knee, struct( ...
    'mode', 'full', ...
    'verbose', true, ...
    'loe_for_sim', [0;0;1;0;0;0], ...
    'sim_config', eval_sim_opts3.sim_config));

fprintf('\n  Section 4 complete [%.1f s]\n\n', toc(t_sec));

%% Section 5: Design Comparison
fprintf('Section 5: Design Comparison\n');
t_sec = tic;

compare_opts.eval_mode = 'full';
compare_opts.sim_loe = [0;0;1;0;0;0];
compare_opts.sim_config = eval_sim_opts3.sim_config;
compare_opts.plot = true;

compare_designs( ...
    {d_base, d_opt_soo, d_knee}, ...
    {'Baseline (2x hover)', 'SOO-Optimal', 'Pareto Knee'}, ...
    compare_opts);

fprintf('\n  Section 5 complete [%.1f s]\n\n', toc(t_sec));

%% Section 6: Stage 2 Mission Verification
fprintf('Section 6: Stage 2 Mission Verification\n');
t_sec = tic;

% Purpose:
%   Stage 1 finds promising designs using fast ACS/hover metrics.
%   This block performs a secondary mission-level check on those candidates.
%
% Scenario:
%   - source-like figure-8 mission settings from Src/sim_path_following.m
%   - motor 1 fault injected at mission start
%   - no-fault control group is plotted automatically in visualize_results
mission_verify_cfg = cfg.sim;
mission_verify_cfg.scenario = 'figure8';
mission_verify_cfg.t_fault = 0;    % fault active from t=0 (matches Src/sim_path_following.m)
mission_verify_cfg.dt = 0.01;
mission_verify_cfg.t_end = 140;
mission_verify_cfg.mission.A = 100;
mission_verify_cfg.mission.T_period = 120;
mission_verify_cfg.mission.z_cruise = 50;
mission_verify_cfg.mission.t_start = 20;
mission_verify_cfg.mission.n_laps = 1;
mission_verify_cfg.mission.ramp_time = 0;

compare_mission_opts.eval_mode = 'full';
compare_mission_opts.sim_loe = [1;0;0;0;0;0];
compare_mission_opts.sim_config = mission_verify_cfg;
compare_mission_opts.plot = false;
compare_mission_opts.objectives = cfg.objectives.stage2;

compare_designs( ...
    {d_base, d_opt_soo, d_knee}, ...
    {'Baseline', 'SOO-Optimal', 'Pareto Knee'}, ...
    compare_mission_opts);

% Plot the Pareto-knee mission response as one representative Stage 2 case.
result_knee_mission = eval_design(d_knee, struct( ...
    'mode', 'full', ...
    'verbose', true, ...
    'loe_for_sim', compare_mission_opts.sim_loe, ...
    'sim_config', mission_verify_cfg));
visualize_results(result_knee_mission, 'sim');

fprintf('\n  Section 6 complete [%.1f s]\n\n', toc(t_sec));

%% Section 7: ACS Geometry Visualization
fprintf('Section 7: ACS Geometry Visualization\n');
t_sec = tic;

figure('Name','ACS [L,M,N] Projection - All Three Designs');
design_set = {d_base, d_opt_soo, d_knee};
design_lbls = {'Baseline','SOO-Optimal','Pareto Knee'};
face_colors = {[0.25 0.55 0.90], [0.15 0.72 0.27], [0.92 0.42 0.14]};

for di = 1:3
    subplot(2,3,di);
    [UAM_d, Prop_d] = hexacopter_params(design_set{di});
    plot_acs_3d(UAM_d.B, Prop_d.T_max, zeros(6,1), face_colors{di}, 0.30);
    title(sprintf('%s (nominal)', design_lbls{di}));
    xlabel('L'); ylabel('M'); zlabel('N'); grid on; view(30,20);

    subplot(2,3,di+3);
    acs_di = eval_acs(design_set{di});
    [~, worst_k] = min(acs_di.single_retention);
    loe_w = zeros(6,1);
    loe_w(worst_k) = 1;
    plot_acs_3d(UAM_d.B, Prop_d.T_max, loe_w, [0.90 0.25 0.20], 0.40);
    title(sprintf('%s (M%d fault)', design_lbls{di}, worst_k));
    xlabel('L'); ylabel('M'); zlabel('N'); grid on; view(30,20);
end

sgtitle('ACS [L,M,N] Projection: Nominal vs Worst-Case Fault', ...
    'FontSize', 12, 'FontWeight', 'bold');

fprintf('\n  Section 7 complete [%.1f s]\n\n', toc(t_sec));

%% Summary
fprintf('===============================================================\n');
fprintf('MDO SUMMARY\n');
fprintf('===============================================================\n');

designs_all = {d_base, d_opt_soo, d_knee};
names_all = {'Baseline','SOO-Opt','Pareto-Knee'};
for k = 1:3
    r = eval_design(designs_all{k}, struct('mode','acs','verbose',false));
    fprintf('  %-12s FII=%.4f  H_margin=%+.3f  J=%.4f\n', ...
        names_all{k}, r.acs.FII, r.acs.hover_margin, r.J_combined);
end
fprintf('===============================================================\n');

%% Local helper
function plot_acs_3d(B, T_max, loe_vec, fcolor, alpha)
    [~, pts4d] = compute_acs_volume(B, T_max, loe_vec);
    pts = unique(round(pts4d(:,[2,3,4]),10), 'rows');
    if size(pts,1) >= 4 && rank(pts - mean(pts)) >= 3
        K = convhulln(pts, {'Qt','Qx'});
        trisurf(K, pts(:,1), pts(:,2), pts(:,3), ...
            'FaceColor', fcolor, ...
            'FaceAlpha', alpha, ...
            'EdgeColor', fcolor * 0.55, ...
            'LineWidth', 0.4);
    end
end
