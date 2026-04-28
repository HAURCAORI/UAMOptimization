%% MAIN_COMPARISON
% Compare a nominal-optimized design against a fault-tolerant design.
%
% Scientific question:
%   What is the performance cost of designing for fault tolerance?
%
% Workflow:
%   1. Build two optimized designs:
%        nominal-opt  – minimizes mass/power/hover efficiency only
%        fault-opt    – minimizes all five Stage 1 objectives including fault terms
%   2. Evaluate both designs in:
%        (a) no-fault hover     (b) faulted hover  (motor 1, t_fault = 10 s)
%        (c) no-fault figure-8  (d) faulted figure-8 (motor 1, t_fault = 40 s)
%   3. Print a Stage 1 summary table.

clc; clear; close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

fprintf('===============================================================\n');
fprintf(' Design Comparison: Nominal vs Fault-Tolerant Optimization\n');
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

no_fault   = zeros(6, 1);
fault_m1   = [1;0;0;0;0;0];   % motor 1 LOE – hover-feasible for all three designs

%% Section 1: Build Optimized Designs
fprintf('Section 1: Build Optimized Designs\n');
t_sec = tic;

% Nominal-optimal: mass + power + hover efficiency. No fault terms.
% Expected outcome: lighter, more efficient, but less fault-tolerant.
cfg_nom                          = cfg;
cfg_nom.eval.mode                = 'acs';
cfg_nom.objectives.stage1.names  = {'mass', 'power', 'hover_nom'};
cfg_nom.objectives.stage1.weights = [0.45, 0.35, 0.20];
[d_nom, J_nom, ~] = run_soo(d_base, cfg_nom); %#ok<ASGLU>

% Fault-tolerant: full five-objective set including thrust and allocation margins.
% Expected outcome: heavier, costlier, but survives single-motor failures.
cfg_fault                          = cfg;
cfg_fault.eval.mode                = 'acs';
cfg_fault.objectives.stage1.names  = {'mass', 'power', 'fault_thrust', 'fault_alloc', 'hover_nom'};
cfg_fault.objectives.stage1.weights = [0.20, 0.20, 0.25, 0.25, 0.10];
[d_fault, J_fault, ~] = run_soo(d_base, cfg_fault); %#ok<ASGLU>

fprintf('  Nominal-opt   Stage 1 J = %.4f\n', J_nom);
fprintf('  Fault-opt     Stage 1 J = %.4f\n', J_fault);
fprintf('\n  Section 1 complete [%.1f s]\n\n', toc(t_sec));

designs = {d_base, d_nom, d_fault};
labels  = {'Baseline', 'Nominal-Opt', 'Fault-Opt'};

%% Section 2: Hover Comparison
% Fault injected at t = 10 s in a 40 s hover simulation.
fprintf('Section 2: Hover Comparison\n');
t_sec = tic;

hover_cfg          = cfg.sim;
hover_cfg.scenario = 'hover';
hover_cfg.dt       = 0.005;
hover_cfg.t_end    = 40;
hover_cfg.t_fault  = 10;
hover_cfg.alt_cmd  = 10;

fprintf('  No-fault hover\n');
compare_designs(designs, labels, UAMOptions(cfg, ...
    'Mode',         'full', ...
    'Fault',        no_fault, ...
    'SimConfig',    hover_cfg, ...
    'ObjectiveSet', 'stage2', ...
    'Plot',         true));

fprintf('\n  Faulted hover (motor 1, t_fault = 10 s)\n');
compare_designs(designs, labels, UAMOptions(cfg, ...
    'Mode',         'full', ...
    'Fault',        fault_m1, ...
    'SimConfig',    hover_cfg, ...
    'ObjectiveSet', 'stage2', ...
    'Plot',         true));

fprintf('\n  Section 2 complete [%.1f s]\n\n', toc(t_sec));

%% Section 3: Figure-8 Mission Comparison
% Fault injected at t = 40 s (20 s after the figure-8 begins at t = 20 s),
% giving the vehicle time to establish the mission trajectory before failure.
fprintf('Section 3: Figure-8 Mission Comparison\n');
t_sec = tic;

mission_cfg                    = cfg.sim;
mission_cfg.scenario           = 'figure8';
mission_cfg.dt                 = 0.01;
mission_cfg.t_end              = 140;
mission_cfg.t_fault            = 40;    % fault 20 s into figure-8
mission_cfg.mission.A          = 100;
mission_cfg.mission.T_period   = 120;
mission_cfg.mission.z_cruise   = 50;
mission_cfg.mission.t_start    = 20;
mission_cfg.mission.n_laps     = 1;
mission_cfg.mission.ramp_time  = 0;

fprintf('  No-fault figure-8\n');
compare_designs(designs, labels, UAMOptions(cfg, ...
    'Mode',         'full', ...
    'Fault',        no_fault, ...
    'SimConfig',    mission_cfg, ...
    'ObjectiveSet', 'stage2', ...
    'Plot',         false));

fprintf('\n  Faulted figure-8 (motor 1, t_fault = 40 s)\n');
compare_designs(designs, labels, UAMOptions(cfg, ...
    'Mode',         'full', ...
    'Fault',        fault_m1, ...
    'SimConfig',    mission_cfg, ...
    'ObjectiveSet', 'stage2', ...
    'Plot',         false));

fprintf('\n  Plotting mission trajectories...\n');
plot_mission_pair(d_base,  cfg, no_fault, fault_m1, mission_cfg, 'Baseline');
plot_mission_pair(d_nom,   cfg, no_fault, fault_m1, mission_cfg, 'Nominal-Opt');
plot_mission_pair(d_fault, cfg, no_fault, fault_m1, mission_cfg, 'Fault-Opt');

fprintf('\n  Section 3 complete [%.1f s]\n\n', toc(t_sec));

%% Stage 1 Summary
% Each design is scored under BOTH objective sets so you can see:
%   J_nom   – how well it satisfies the efficiency-only objective
%   J_fault – how well it satisfies the fault-tolerant objective
fprintf('===============================================================\n');
fprintf('STAGE 1 SUMMARY\n');
fprintf('%-14s  %-10s  %-10s  %-8s  %-10s\n', ...
    'Design', 'J_nom', 'J_fault', 'FII', 'H_margin');
fprintf('%s\n', repmat('-', 1, 56));
for k = 1:numel(designs)
    r_n = eval_design(designs{k}, UAMOptions(cfg_nom,   'Mode', 'acs', 'Verbose', false));
    r_f = eval_design(designs{k}, UAMOptions(cfg_fault, 'Mode', 'acs', 'Verbose', false));
    fprintf('  %-12s  %10.4f  %10.4f  %8.4f  %+10.3f\n', ...
        labels{k}, r_n.J_combined, r_f.J_combined, r_f.acs.FII, r_f.acs.hover_margin);
end
fprintf('===============================================================\n');

%% Local helpers
function plot_mission_pair(d, cfg, no_fault, fault_case, mission_cfg, design_name)
% Simulate and plot XY path, tracking error, altitude, and attitude
% for no-fault and faulted conditions side by side.

% No-fault: same sim config but zero LOE
nf_cfg         = mission_cfg;
nf_cfg.loe_vec = no_fault;
nf_cfg.t_fault = inf;

result_nf = eval_design(d, UAMOptions(cfg, ...
    'Mode', 'full', 'Fault', no_fault, 'SimConfig', nf_cfg, 'ObjectiveSet', 'stage2'));
result_f  = eval_design(d, UAMOptions(cfg, ...
    'Mode', 'full', 'Fault', fault_case, 'SimConfig', mission_cfg, 'ObjectiveSet', 'stage2'));

IDX = state_indices();
t_nf  = result_nf.sim.t_vec(:);
t_f   = result_f.sim.t_vec(:);
x_nf  = result_nf.sim.X_hist(IDX.x, :).';
y_nf  = result_nf.sim.X_hist(IDX.y, :).';
z_nf  = -result_nf.sim.X_hist(IDX.z, :).';
phi_nf    = rad2deg(result_nf.sim.X_hist(IDX.phi, :).');
theta_nf  = rad2deg(result_nf.sim.X_hist(IDX.theta, :).');
ref_nf    = result_nf.sim.pos_ref.';

x_f   = result_f.sim.X_hist(IDX.x, :).';
y_f   = result_f.sim.X_hist(IDX.y, :).';
z_f   = -result_f.sim.X_hist(IDX.z, :).';
phi_f     = rad2deg(result_f.sim.X_hist(IDX.phi, :).');
theta_f   = rad2deg(result_f.sim.X_hist(IDX.theta, :).');
ref_f     = result_f.sim.pos_ref.';

figure('Name', ['Mission: ' design_name]);

subplot(2,2,1);
plot(ref_nf(:,1), ref_nf(:,2), 'k--', 'LineWidth', 1.2); hold on;
plot(x_nf, y_nf, 'b-', 'LineWidth', 1.4);
plot(x_f,  y_f,  'r-', 'LineWidth', 1.4);
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]');
title([design_name ' – XY Path']);
legend({'Reference', 'No Fault', 'Faulted'}, 'Location', 'best');

subplot(2,2,2);
err_nf = vecnorm([x_nf, y_nf] - ref_nf(:,1:2), 2, 2);
err_f  = vecnorm([x_f,  y_f]  - ref_f(:,1:2),  2, 2);
plot(t_nf, err_nf, 'b-', 'LineWidth', 1.3); hold on;
plot(t_f,  err_f,  'r-', 'LineWidth', 1.3);
grid on; xlabel('t [s]'); ylabel('Horizontal error [m]');
title([design_name ' – Tracking Error']);
legend({'No Fault', 'Faulted'}, 'Location', 'best');

subplot(2,2,3);
plot(t_nf, z_nf, 'b-', 'LineWidth', 1.3); hold on;
plot(t_f,  z_f,  'r-', 'LineWidth', 1.3);
plot(t_nf, ref_nf(:,3), 'k--', 'LineWidth', 1.1);
grid on; xlabel('t [s]'); ylabel('z [m]');
title([design_name ' – Altitude']);
legend({'No Fault', 'Faulted', 'Reference'}, 'Location', 'best');

subplot(2,2,4);
plot(t_nf, phi_nf,   'b-',  'LineWidth', 1.2); hold on;
plot(t_f,  phi_f,    'r-',  'LineWidth', 1.2);
plot(t_nf, theta_nf, 'b--', 'LineWidth', 1.2);
plot(t_f,  theta_f,  'r--', 'LineWidth', 1.2);
grid on; xlabel('t [s]'); ylabel('Angle [deg]');
title([design_name ' – Attitude']);
legend({'phi no-fault','phi faulted','theta no-fault','theta faulted'}, 'Location', 'best');
end

function IDX = state_indices()
IDX.phi = 7; IDX.theta = 8; IDX.x = 10; IDX.y = 11; IDX.z = 12;
end
