function visualize_results(result_or_sweep, vtype, options)
% VISUALIZE_RESULTS  Unified visualization for MDO framework outputs.
%
%   Produces publication-quality figures for design evaluation results,
%   parameter sweeps, and Pareto fronts.
%
%   Usage:
%     visualize_results(eval_result, 'acs')     - ACS plots for one design
%     visualize_results(eval_result, 'sim')     - Simulation time histories
%     visualize_results(sweep_result, 'sweep')  - Sweep sensitivity curves
%     visualize_results(pareto_J,    'pareto')  - Pareto front scatter
%
%   Inputs:
%     result_or_sweep - eval_design result, sweep result, or pareto_J matrix
%     vtype           - string: 'acs' | 'sim' | 'sweep' | 'pareto'
%     options         - (optional) visualization options struct

if nargin < 3, options = struct(); end

switch lower(vtype)
    case 'acs',    plot_acs(result_or_sweep, options);
    case 'sim',    plot_sim(result_or_sweep, options);
    case 'sweep',  plot_sweep(result_or_sweep, options);
    case 'pareto', plot_pareto(result_or_sweep, options);
    otherwise
        error('visualize_results: unknown vtype ''%s''', vtype);
end
end


%% ── ACS visualization ──────────────────────────────────────────────────
function plot_acs(r, ~)
if isempty(r.acs)
    warning('visualize_results: no ACS data in result'); return
end
acs = r.acs;
d   = r.d;
[UAM, Prop, ~] = hexacopter_params(d);
B     = UAM.B;
T_max = Prop.T_max;

c_nom   = [0.20 0.50 0.90];
c_fault = [0.95 0.30 0.20];
alph_n  = 0.25;  alph_f = 0.45;

% ── Figure 1: Bar chart of retention per fault scenario (single motor) ────
figure('Name','ACS Retention per Fault');
ret   = acs.single_retention;
x_bar = 1:6;
b = bar(x_bar, ret * 100, 0.6, 'FaceColor', 'flat');
b.CData = repmat([0.3 0.6 0.9], 6, 1);
hold on
yline(100*acs.PFWAR, 'r--', 'LineWidth', 1.5, 'Label','PFWAR', 'LabelHorizontalAlignment','right');
yline(100*acs.WCFR,  'k:',  'LineWidth', 1.5, 'Label','WCFR',  'LabelHorizontalAlignment','right');
ylim([0, 110]);
xlabel('Failed Motor Index');
ylabel('ACS Volume Retention [%]');
title(sprintf('Single-Motor Fault ACS Retention  (FII = %.4f)', acs.FII));
xticks(1:6);
xticklabels({'M1','M2','M3','M4','M5','M6'});
grid on;

% Mark hover-feasibility
hold on
for k = 1:6
    clr = [0.1 0.7 0.1] * acs.hover_ok_single(k) + [0.9 0.1 0.1] * ~acs.hover_ok_single(k);
    text(k, ret(k)*100+3, sprintf('%.1f%%', ret(k)*100), ...
         'HorizontalAlignment','center', 'FontSize', 8, 'Color', clr);
end
legend('Retention','PFWAR','WCFR','Location','southeast');

% ── Figure 2: LMN projection of nominal vs worst-case fault ACS ───────────
[~, worst_idx] = min(acs.single_retention);
loe_worst = acs.fault_list(worst_idx,:)';

pts_nom   = get_acs_pts_3d(B, T_max, zeros(6,1), [2,3,4]);
pts_fault = get_acs_pts_3d(B, T_max, loe_worst,  [2,3,4]);

figure('Name','ACS Projection: Roll-Pitch-Yaw (LMN)');
hold on; grid on;
xlabel('L (Roll) [Nm]'); ylabel('M (Pitch) [Nm]'); zlabel('N (Yaw) [Nm]');
title(sprintf('ACS: [L,M,N] projection  —  Motor %d failure', worst_idx));
view(30, 20);

if size(pts_nom,1) >= 4 && rank(pts_nom - mean(pts_nom)) >= 3
    K = convhulln(pts_nom, {'Qt','Qx'});
    trisurf(K, pts_nom(:,1), pts_nom(:,2), pts_nom(:,3), ...
            'FaceColor', c_nom, 'FaceAlpha', alph_n, ...
            'EdgeColor', c_nom * 0.55, 'LineWidth', 0.4);
end
if size(pts_fault,1) >= 4 && rank(pts_fault - mean(pts_fault)) >= 3
    K = convhulln(pts_fault, {'Qt','Qx'});
    trisurf(K, pts_fault(:,1), pts_fault(:,2), pts_fault(:,3), ...
            'FaceColor', c_fault, 'FaceAlpha', alph_f, ...
            'EdgeColor', c_fault * 0.55, 'LineWidth', 0.4);
end
legend({'Nominal ACS', sprintf('Motor %d faulted', worst_idx)}, 'Location','best');

% ── Figure 3: Hover feasibility per single fault (polar-style) ────────────
figure('Name','Hover Utilization per Fault');
util = acs.hover_util(1:6);
util(isinf(util)) = 1.5;  % cap for display
theta_m = linspace(0, 2*pi, 7);
theta_m(end) = [];

r_ax = axes; hold on;
r_ax.Visible = 'off'; axis equal;
xlim([-1.8, 1.8]); ylim([-1.8, 1.8]);
title('Hover Actuator Utilization per Motor Failure', 'FontSize', 11);

% Draw rings
for ring = [0.5, 1.0, 1.5]
    xr = ring * cos(linspace(0,2*pi,100));
    yr = ring * sin(linspace(0,2*pi,100));
    ltype = '-'; if ring == 1.0, ltype = 'r-'; end
    plot(xr, yr, ltype, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8 + (ring==1)*0.7);
end
text(1.05, 0, 'util=1', 'Color','r','FontSize',8);

% Plot utilization as radial bars
motor_colors = lines(6);
for k = 1:6
    ang = theta_m(k);
    u   = min(util(k), 1.5);
    clr = motor_colors(k,:);
    plot([0, u*cos(ang)], [0, u*sin(ang)], '-', 'Color', clr, 'LineWidth', 4);
    text(1.65*cos(ang), 1.65*sin(ang), sprintf('M%d\n%.2f', k, util(k)), ...
         'HorizontalAlignment','center', 'FontSize', 9, 'Color', clr);
end
end


%% ── Simulation visualization ───────────────────────────────────────────
function plot_sim(r, ~)
if isempty(r.sim)
    warning('visualize_results: no simulation data'); return
end
s   = r.sim;
t   = s.t_vec;
IDX_phi=7; IDX_theta=8; IDX_psi=9;
IDX_z=12; IDX_vz=15;

alt_h   = -s.X_hist(IDX_z,:);
phi_h   = rad2deg(s.X_hist(IDX_phi,:));
theta_h = rad2deg(s.X_hist(IDX_theta,:));

% Find t_fault approximation (when LOE kicks in)
sc = r.sim;
if isfield(sc,'t_fault')
    t_fault = sc.t_fault;
else
    t_fault = 10;
end

figure('Name','Post-Fault Response');
subplot(3,1,1);
plot(t, alt_h, 'b', 'LineWidth', 1.5); hold on;
yline(10, 'r--', 'Cmd', 'LabelHorizontalAlignment','right');
xline(t_fault, 'k:', 'Fault', 'LabelVerticalAlignment','top');
grid on; ylabel('Altitude [m]'); title('Post-Fault Simulation Response');
legend('Actual','Cmd','Fault','Location','best');

subplot(3,1,2);
plot(t, phi_h,   'b', 'LineWidth', 1.5, 'DisplayName','\phi'); hold on;
plot(t, theta_h, 'r', 'LineWidth', 1.5, 'DisplayName','\theta');
xline(t_fault, 'k:');
yline(5,  '--', 'Color',[0.5 0.5 0.5]);
yline(-5, '--', 'Color',[0.5 0.5 0.5]);
grid on; ylabel('Attitude [deg]'); legend('Location','best');

subplot(3,1,3);
thrust_total = sum(s.T_hist, 1);
plot(t, thrust_total / max(thrust_total), 'k', 'LineWidth', 1.5);
xline(t_fault, 'k:');
grid on; ylabel('Total Thrust / T_{peak}'); xlabel('Time [s]');
title('Normalized Thrust Effort');
end


%% ── Sweep visualization ─────────────────────────────────────────────────
function plot_sweep(sw, ~)
if isfield(sw,'var_name') && ~iscell(sw.var_name)
    % 1D sweep
    x   = sw.var_values;
    vn  = sw.var_name;

    figure('Name', sprintf('Sweep: %s', vn));
    subplot(2,2,1);
    plot(x, sw.PFWAR, 'b-o', 'LineWidth',1.5,'MarkerSize',4);
    grid on; xlabel(vn); ylabel('PFWAR'); title('Fault-Weighted ACS Retention');

    subplot(2,2,2);
    plot(x, sw.FII, 'r-s', 'LineWidth',1.5,'MarkerSize',4);
    grid on; xlabel(vn); ylabel('FII'); title('Fault Isotropy Index (lower=better)');

    subplot(2,2,3);
    plot(x, sw.WCFR, 'm-^', 'LineWidth',1.5,'MarkerSize',4);
    grid on; xlabel(vn); ylabel('WCFR'); title('Worst-Case Fault Retention');

    subplot(2,2,4);
    plot(x, sw.J_combined, 'k-d', 'LineWidth',1.5,'MarkerSize',4);
    grid on; xlabel(vn); ylabel('J\_combined'); title('Combined Objective');

    sgtitle(sprintf('Design Space Sweep: %s', vn));

else
    % 2D sweep — use surf plots
    var1   = sw.var_names{1};
    var2   = sw.var_names{2};
    x1     = sw.var_values{1};
    x2     = sw.var_values{2};
    [X1,X2]= meshgrid(x1,x2);

    figure('Name', sprintf('2D Sweep: %s × %s', var1, var2));
    subplot(1,3,1);
    surf(X1, X2, sw.PFWAR', 'EdgeColor','none');
    xlabel(var1); ylabel(var2); zlabel('PFWAR'); title('PFWAR');
    colorbar; view(45,30);

    subplot(1,3,2);
    surf(X1, X2, sw.FII', 'EdgeColor','none');
    xlabel(var1); ylabel(var2); zlabel('FII'); title('FII');
    colorbar; view(45,30);

    subplot(1,3,3);
    surf(X1, X2, sw.J_combined', 'EdgeColor','none');
    xlabel(var1); ylabel(var2); zlabel('J\_combined'); title('J\_combined');
    colorbar; view(45,30);

    sgtitle(sprintf('2D Design Sweep: %s × %s', var1, var2));
end
end


%% ── Pareto visualization ────────────────────────────────────────────────
function plot_pareto(F, opts)
if ~isfield(opts,'labels')
    f_labels = {'J_{fault} (1-PFWAR)', 'J_{isotropy} / J_{mission}', 'J_{cost}'};
else
    f_labels = opts.labels;
end

figure('Name','Pareto Front');

% 3D scatter
subplot(1,2,1);
scatter3(F(:,1), F(:,2), F(:,3), 60, F(:,1), 'filled');
xlabel(f_labels{1}); ylabel(f_labels{2}); zlabel(f_labels{3});
title('3D Pareto Front'); colorbar; grid on; view(45,30);

% 2D projections
subplot(1,2,2);
scatter(F(:,1), F(:,2), 40, F(:,3), 'filled');
xlabel(f_labels{1}); ylabel(f_labels{2});
title('Pareto: f1 vs f2  (colored by f3)');
colorbar; cblabel = f_labels{3};
c = colorbar; c.Label.String = cblabel;
grid on;
end


%% ── Helper: get 3D ACS projection ───────────────────────────────────────
function pts = get_acs_pts_3d(B, T_max, loe_vec, dims)
% Extract 3 columns from the 4D ACS vertex set
[~, pts_4d] = compute_acs_volume(B, T_max, loe_vec);
pts = pts_4d(:, dims);
pts = unique(round(pts, 10), 'rows');
end
