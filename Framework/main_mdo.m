%% MAIN_MDO
% UAM hexacopter fault-tolerant MDO demonstration.
%
% Workflow:
%   1. Baseline evaluation  – ACS metrics and fault simulation
%   2. Sensitivity sweeps   – understand design-space landscape
%   3. SOO                  – single-objective fault-tolerant optimization
%   4. MOO                  – multi-objective Pareto front (mass/power/fault)
%   5. Design comparison    – baseline vs SOO-optimal vs Pareto-knee
%   6. Mission verification – optimized designs in full figure-8 mission
%   7. ACS visualization    – control polytopes for each design

%% Initialization
clc; clear; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

fprintf('===============================================================\n');
fprintf(' UAM Hexacopter Fault-Tolerant MDO Framework\n');
fprintf(' AE50001 Team 5 / 2026 Spring\n');
fprintf('===============================================================\n\n');

fw = fileparts(mfilename('fullpath'));
addpath(fullfile(fw, 'config'));
addpath(fullfile(fw, 'core'));
addpath(fullfile(fw, 'evaluation'));
addpath(fullfile(fw, 'metrics'));
addpath(fullfile(fw, 'optimization'));
addpath(fullfile(fw, 'analysis'));

cfg    = mdo_config();
d_base = design_default();

fault_m1 = [1;0;0;0;0;0];   % motor 1 – hover-feasible for baseline
fault_m3 = [0;0;1;0;0;0];   % motor 3 – hover-infeasible for baseline

hover_sim_cfg        = cfg.sim;
hover_sim_cfg.scenario = 'hover';
hover_sim_cfg.dt       = 0.005;
hover_sim_cfg.t_end    = 40;
hover_sim_cfg.t_fault  = 10;    % fault injected at t = 10 s
hover_sim_cfg.alt_cmd  = 10;

%% Section 1: Baseline Evaluation
fprintf('Section 1: Baseline Evaluation\n');
t_sec = tic;

% ACS evaluation with double-fault analysis enabled for the baseline report
result_base_acs = eval_design(d_base, UAMOptions(cfg, ...
    'Mode', 'acs', ...
    'Verbose', true, ...
    'FaultConfig', struct('include_double', true, 'p_motor', cfg.fault.p_motor)));

% Simulate motor 1 fault (hover-feasible case – baseline can recover)
fprintf('\n  Running simulation: motor 1 fault (hover-feasible)...\n');
result_base_m1 = eval_design(d_base, UAMOptions(cfg, ...
    'Mode', 'sim', ...
    'Fault', fault_m1, ...
    'SimConfig', hover_sim_cfg));
result_base_m1.sim.t_fault = hover_sim_cfg.t_fault;   % annotation for plots

% Simulate motor 3 fault (hover-infeasible – baseline diverges, used as a bound)
fprintf('  Running simulation: motor 3 fault (hover-infeasible baseline)...\n');
result_base_m3 = eval_design(d_base, UAMOptions(cfg, ...
    'Mode', 'sim', ...
    'Fault', fault_m3, ...
    'SimConfig', hover_sim_cfg));

visualize_results(result_base_acs, 'acs');
visualize_results(result_base_m1,  'sim');

fprintf('\n  Section 1 complete [%.1f s]\n\n', toc(t_sec));

%% Section 2: Sensitivity Sweeps
fprintf('Section 2: Sensitivity Sweeps\n');
t_sec = tic;

sweep_opt = UAMOptions(cfg, 'Mode', 'acs', 'Verbose', false);

fprintf('  Sweeping Lyi...\n');
sw_Lyi = sweep_design_space(d_base, 'Lyi', linspace(1.0, 5.49, 40), sweep_opt);

fprintf('  Sweeping T_max...\n');
sw_Tmax = sweep_design_space(d_base, 'T_max', linspace(5000, 18000, 40), sweep_opt);

fprintf('  Running 2-D sweep: Lyi × T_max...\n');
sw_2d = sweep_design_space(d_base, ...
    {'Lyi', 'T_max'}, {linspace(1.5, 5.5, 20), linspace(7000, 16000, 20)}, sweep_opt);

visualize_results(sw_Lyi,  'sweep');
visualize_results(sw_Tmax, 'sweep');
visualize_results(sw_2d,   'sweep');

fprintf('\n  Section 2 complete [%.1f s]\n\n', toc(t_sec));

%% Section 3: Stage 1 Single-Objective Optimization
% Optimize for all five fault-tolerant Stage 1 objectives using CMA-ES.
% Default cfg.objectives.stage1: mass(0.20) power(0.20) fault_thrust(0.25)
%                                  fault_alloc(0.25) hover_nom(0.10)
fprintf('Section 3: Single-Objective Optimization (fault-tolerant)\n');
t_sec = tic;

[d_soo, J_soo, ~] = run_soo(d_base, cfg); %#ok<ASGLU>

fprintf('\n  Full validation of SOO-optimal design (motor 3 fault, hover-infeasible baseline):\n');
full_opt_m3 = UAMOptions(cfg, ...
    'Mode', 'full', ...
    'Verbose', true, ...
    'Fault', fault_m3, ...
    'SimConfig', hover_sim_cfg);
result_soo = eval_design(d_soo, full_opt_m3);

fprintf('\n  Section 3 complete [%.1f s]\n\n', toc(t_sec));

%% Section 4: Stage 1 Multi-Objective Optimization
% Three-objective Pareto front: mass vs power vs aggregate fault score.
fprintf('Section 4: Multi-Objective Optimization (mass / power / fault)\n');
t_sec = tic;

moo_cfg              = cfg;
moo_cfg.eval.mode    = 'acs';
moo_cfg.opt.verbose  = true;
moo_cfg.opt.ga_pop   = 100;
moo_cfg.opt.max_iter = 150;

[pareto_designs, pareto_J, ~, ~] = run_moo(d_base, moo_cfg);

pareto_opt = UAMOptions(moo_cfg, ...
    'Plot',   true, ...
    'Labels', strcat('J_{', moo_cfg.objectives.stage1.moo_names, '}'));
[~, pareto_nd, knee_idx] = pareto_analysis(pareto_designs, pareto_J, pareto_opt); %#ok<NASGU>

d_knee = pareto_nd{knee_idx};
fprintf('\n  Full validation of Pareto-knee design (motor 3 fault):\n');
result_knee = eval_design(d_knee, full_opt_m3);

fprintf('\n  Section 4 complete [%.1f s]\n\n', toc(t_sec));

%% Section 5: Design Comparison
% Side-by-side comparison of baseline, SOO-optimal, and Pareto-knee designs
% evaluated under the hover-infeasible motor 3 fault.
fprintf('Section 5: Design Comparison\n');
t_sec = tic;

compare_designs( ...
    {d_base, d_soo, d_knee}, ...
    {'Baseline', 'SOO-Optimal', 'Pareto Knee'}, ...
    UAMOptions(cfg, ...
        'Mode',      'full', ...
        'Fault',     fault_m3, ...
        'SimConfig', hover_sim_cfg, ...
        'Plot',      true));

fprintf('\n  Section 5 complete [%.1f s]\n\n', toc(t_sec));

%% Section 6: Stage 2 Mission Verification
% Validate optimized designs in a full figure-8 mission with a mid-mission
% motor 1 fault (hover-feasible, most realistic in-flight failure scenario).
% Fault is injected at t = 20 s, when the figure-8 begins after climb.
fprintf('Section 6: Stage 2 Mission Verification\n');
t_sec = tic;

mission_cfg              = cfg.sim;
mission_cfg.scenario     = 'figure8';
mission_cfg.dt           = 0.01;
mission_cfg.t_end        = 140;
mission_cfg.t_fault      = 20;   % fault injected at mission start (after climb)
mission_cfg.mission.A         = 100;
mission_cfg.mission.T_period  = 120;
mission_cfg.mission.z_cruise  = 50;
mission_cfg.mission.t_start   = 20;
mission_cfg.mission.n_laps    = 1;
mission_cfg.mission.ramp_time = 0;

mission_cmp_opt = UAMOptions(cfg, ...
    'Mode',         'full', ...
    'Fault',        fault_m1, ...
    'SimConfig',    mission_cfg, ...
    'ObjectiveSet', 'stage2', ...
    'Plot',         false);

compare_designs( ...
    {d_base, d_soo, d_knee}, ...
    {'Baseline', 'SOO-Optimal', 'Pareto Knee'}, ...
    mission_cmp_opt);

result_knee_mission = eval_design(d_knee, UAMOptions(cfg, ...
    'Mode',         'full', ...
    'Verbose',      true, ...
    'Fault',        fault_m1, ...
    'SimConfig',    mission_cfg, ...
    'ObjectiveSet', 'stage2'));
visualize_results(result_knee_mission, 'sim');

fprintf('\n  Section 6 complete [%.1f s]\n\n', toc(t_sec));

%% Section 7: ACS Geometry Visualization
fprintf('Section 7: ACS Geometry Visualization\n');
t_sec = tic;

figure('Name', 'ACS [L,M,N] Projection – Nominal vs Worst-Case Fault');
design_set  = {d_base, d_soo, d_knee};
design_lbls = {'Baseline', 'SOO-Optimal', 'Pareto Knee'};
face_colors = {[0.25 0.55 0.90], [0.15 0.72 0.27], [0.92 0.42 0.14]};

for di = 1:3
    [uam_d, prop_d] = hexacopter_params(design_set{di});
    acs_di = eval_acs(design_set{di});
    [~, worst_k] = min(acs_di.single_retention);
    loe_worst = zeros(6, 1);  loe_worst(worst_k) = 1;

    subplot(2, 3, di);
    plot_acs_3d(uam_d.B, prop_d.T_max, zeros(6,1), face_colors{di}, 0.30);
    title(sprintf('%s (nominal)', design_lbls{di}));
    xlabel('L'); ylabel('M'); zlabel('N'); grid on; view(30, 20);

    subplot(2, 3, di + 3);
    plot_acs_3d(uam_d.B, prop_d.T_max, loe_worst, [0.90 0.25 0.20], 0.40);
    title(sprintf('%s (M%d fault)', design_lbls{di}, worst_k));
    xlabel('L'); ylabel('M'); zlabel('N'); grid on; view(30, 20);
end
sgtitle('ACS [L,M,N] Projection: Nominal vs Worst-Case Fault', ...
    'FontSize', 12, 'FontWeight', 'bold');

fprintf('\n  Section 7 complete [%.1f s]\n\n', toc(t_sec));

%% Summary
fprintf('===============================================================\n');
fprintf('MDO SUMMARY\n');
fprintf('===============================================================\n');
summary_opt = UAMOptions(cfg, 'Mode', 'acs', 'Verbose', false);
designs_all = {d_base, d_soo, d_knee};
names_all   = {'Baseline', 'SOO-Opt', 'Pareto-Knee'};
for k = 1:3
    r = eval_design(designs_all{k}, summary_opt);
    fprintf('  %-12s  FII=%6.4f  H_margin=%+.3f  J_combined=%.4f\n', ...
        names_all{k}, r.acs.FII, r.acs.hover_margin, r.J_combined);
end
fprintf('===============================================================\n');

%% Local helper
function plot_acs_3d(B, T_max, loe_vec, face_color, face_alpha)
[~, pts4d] = compute_acs_volume(B, T_max, loe_vec);
pts = unique(round(pts4d(:, [2,3,4]), 10), 'rows');
if size(pts, 1) >= 4 && rank(pts - mean(pts)) >= 3
    K = convhulln(pts, {'Qt', 'Qx'});
    trisurf(K, pts(:,1), pts(:,2), pts(:,3), ...
        'FaceColor', face_color, ...
        'FaceAlpha', face_alpha, ...
        'EdgeColor', face_color * 0.55, ...
        'LineWidth', 0.4);
end
end
