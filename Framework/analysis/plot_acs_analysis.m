function plot_acs_analysis(d, loe_vec, label)
% PLOT_ACS_ANALYSIS  Attainable Control Set (ACS) analysis for a hexacopter design.
%
%   Generates five figures comparing the nominal and fault-case ACS polytopes:
%
%     Fig 1 : Full [L, M, N] projection          (3D convex hull)
%     Fig 2 : Full [L, M, Fz] projection         (3D convex hull)
%     Fig 3 : Slice at Fz = -W  →  [L, M, N]    *** HOVER CAPABILITY ***
%     Fig 4 : Slice at N  =  0  →  [L, M, Fz]
%     Fig 5 : Double slice Fz=-W, N=0  →  [L, M] (2D moment map at hover)
%
%   The Fz = -W slice (Fig 3 & 5) is the most physically meaningful: it shows
%   the range of roll/pitch/yaw moments available while maintaining hover thrust,
%   which directly represents the vehicle's fault-tolerant manoeuvrability.
%
%   Usage:
%     plot_acs_analysis(d)                      % motor 1 fault by default
%     plot_acs_analysis(d, [0;0;1;0;0;0])       % motor 3 fault
%     plot_acs_analysis(d, loe_vec, 'My Design')
%
%   Inputs:
%     d       - design struct (from design_default() or optimizer)
%     loe_vec - [6×1] LOE vector, ∈ [0,1] per motor (default: motor 1 full LOE)
%     label   - figure title prefix string (default: 'Design')

if nargin < 2 || isempty(loe_vec), loe_vec = [1;0;0;0;0;0]; end
if nargin < 3 || isempty(label),   label   = 'Design'; end
loe_vec = loe_vec(:);

[UAM, Prop, Env] = hexacopter_params(d);
B     = UAM.B;
T_max = Prop.T_max;
m     = UAM.m;
g     = Env.g;
W     = m * g;   % vehicle weight [N]

% Identify faulted motor index for labels (first motor with LOE > 0)
fault_motor_idx = find(loe_vec > 0, 1);
if isempty(fault_motor_idx)
    fault_label = 'No fault';
else
    fault_label = sprintf('Motor %d LOE=%.0f%%', fault_motor_idx, loe_vec(fault_motor_idx)*100);
end

fprintf('\n=== ACS Analysis: %s ===\n', label);
fprintf('  m=%.1f kg   T_max=%.0f N   W=%.0f N\n', m, T_max, W);
fprintf('  Fault case: %s\n\n', fault_label);

%% Build 4D ACS vertex sets  [N_pts × 4] columns = [Fz, L, M, N]
P4_nom   = acs_pts_4d(B, T_max, zeros(6,1));
P4_fault = acs_pts_4d(B, T_max, loe_vec);

%% Compute 4D convex hulls (Qx = exact arithmetic for accurate edge enumeration)
[K4_nom,   vol4_nom]   = hull4d(P4_nom);
[K4_fault, vol4_fault] = hull4d(P4_fault);

retention = 0;
if vol4_nom > 0
    retention = vol4_fault / vol4_nom;
end
fprintf('  4D hull volume (nominal): %.4g\n', vol4_nom);
fprintf('  4D hull volume (fault):   %.4g\n', vol4_fault);
fprintf('  4D fault retention ratio: %.4f\n\n', retention);

% Extract unique edge lists from each hull
E4_nom   = hull_edges(K4_nom,   4);
E4_fault = hull_edges(K4_fault, 4);

%% Slicing: Fz = -W  (hover constraint)
tol = max(1e-6 * max(abs(P4_nom(:))), 1e-6);

% Slice at Fz = -W: output columns = [L, M, N]
P_Fz_nom   = slice_at(P4_nom,   E4_nom,   1, -W, 2:4, tol);
P_Fz_fault = slice_at(P4_fault, E4_fault, 1, -W, 2:4, tol);

% Slice at N = 0: output columns = [L, M, Fz] (reorder for plotting)
P_N_nom   = slice_at(P4_nom,   E4_nom,   4,  0, [2 3 1], tol);
P_N_fault = slice_at(P4_fault, E4_fault, 4,  0, [2 3 1], tol);

%% Double slice (Fz=-W then N=0): output columns = [L, M]
%  Start from the [L, M, N] Fz=-W slice, then cut at N=0
[K3_nom,   ~] = hull4d(P_Fz_nom);
[K3_fault, ~] = hull4d(P_Fz_fault);
E3_nom   = hull_edges(K3_nom,   3);
E3_fault = hull_edges(K3_fault, 3);
P_LM_nom   = slice_at(P_Fz_nom,   E3_nom,   3, 0, 1:2, tol);   % [L, M]
P_LM_fault = slice_at(P_Fz_fault, E3_fault, 3, 0, 1:2, tol);

%% Color and style settings
c_nom   = [0.20 0.50 0.90];
c_fault = [0.95 0.30 0.20];
a_nom   = 0.25;
a_fault = 0.50;
lw_cvx  = 0.5;
lw_ax   = 2.0;
vc      = 0.999;   % slight contraction for fault hull visibility

%% ── FIGURE 1: [L, M, N] projection ──────────────────────────────────────
figure('Name', sprintf('%s – Fig1: LMN Projection', label));
hold on; grid on; view(30, 30);
xlabel('L [Nm]'); ylabel('M [Nm]'); zlabel('N [Nm]');
title(sprintf('%s | ACS [L,M,N] Projection', label));

P_LMN_nom   = unique(round(P4_nom(:,2:4),   12), 'rows');
P_LMN_fault = unique(round(P4_fault(:,2:4), 12), 'rows');

[K_nom,   V_nom]   = hull4d(P_LMN_nom);
[K_fault, V_fault] = hull4d(P_LMN_fault);
fprintf('  LMN projection volume – nominal: %.4f  fault: %.4f\n\n', V_nom, V_fault);

if ~isempty(K_nom)
    trisurf(K_nom, P_LMN_nom(:,1), P_LMN_nom(:,2), P_LMN_nom(:,3), ...
        'FaceColor', c_nom, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', a_nom);
end
if ~isempty(K_fault)
    trisurf(K_fault, P_LMN_fault(:,1)*vc, P_LMN_fault(:,2)*vc, P_LMN_fault(:,3)*vc, ...
        'FaceColor', c_fault, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', a_fault);
end
draw_axes_3d(lw_ax);
legend({'Nominal', fault_label}, 'Location', 'best');

%% ── FIGURE 2: [L, M, Fz] projection ─────────────────────────────────────
figure('Name', sprintf('%s – Fig2: LMFz Projection', label));
hold on; grid on; view(30, 30);
xlabel('L [Nm]'); ylabel('M [Nm]'); zlabel('Fz [N]');
title(sprintf('%s | ACS [L,M,Fz] Projection', label));

P_LMFz_nom   = unique(round(P4_nom(:,[2 3 1]),   12), 'rows');
P_LMFz_fault = unique(round(P4_fault(:,[2 3 1]), 12), 'rows');

[K_nom,   V_nom]   = hull4d(P_LMFz_nom);
[K_fault, V_fault] = hull4d(P_LMFz_fault);
fprintf('  LMFz projection volume – nominal: %.4f  fault: %.4f\n\n', V_nom, V_fault);

if ~isempty(K_nom)
    trisurf(K_nom, P_LMFz_nom(:,1), P_LMFz_nom(:,2), P_LMFz_nom(:,3), ...
        'FaceColor', c_nom, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', a_nom);
end
if ~isempty(K_fault)
    trisurf(K_fault, P_LMFz_fault(:,1)*vc, P_LMFz_fault(:,2)*vc, P_LMFz_fault(:,3)*vc, ...
        'FaceColor', c_fault, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', a_fault);
end
draw_axes_3d(lw_ax);
legend({'Nominal', fault_label}, 'Location', 'best');

%% ── FIGURE 3: Slice at Fz = -W  →  [L, M, N]  (HOVER CAPABILITY) ───────
figure('Name', sprintf('%s – Fig3: LMN slice at Fz=-W (HOVER)', label));
hold on; grid on; view(30, 30);
xlabel('L [Nm]'); ylabel('M [Nm]'); zlabel('N [Nm]');
title(sprintf('%s | LMN slice at Fz = -W = %.0f N  (hover capability)', label, -W));

[K_s_nom,   V_s_nom]   = hull4d(P_Fz_nom);
[K_s_fault, V_s_fault] = hull4d(P_Fz_fault);
fprintf('  LMN slice at Fz=-W – nominal volume: %.4f  fault volume: %.4f\n', V_s_nom, V_s_fault);
if V_s_nom > 0
    fprintf('  Hover-slice fault retention: %.4f\n\n', V_s_fault / V_s_nom);
end

if ~isempty(K_s_nom) && size(P_Fz_nom, 1) >= 4
    trisurf(K_s_nom, P_Fz_nom(:,1), P_Fz_nom(:,2), P_Fz_nom(:,3), ...
        'FaceColor', c_nom, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', a_nom);
else
    fprintf('  WARNING: nominal hover slice is degenerate or empty.\n');
end
if ~isempty(K_s_fault) && size(P_Fz_fault, 1) >= 4
    trisurf(K_s_fault, P_Fz_fault(:,1)*vc, P_Fz_fault(:,2)*vc, P_Fz_fault(:,3)*vc, ...
        'FaceColor', c_fault, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', a_fault);
else
    fprintf('  WARNING: fault hover slice is degenerate or empty.\n\n');
end
draw_axes_3d(lw_ax);
legend({'Nominal', fault_label}, 'Location', 'best');

%% ── FIGURE 4: Slice at N = 0  →  [L, M, Fz] ────────────────────────────
figure('Name', sprintf('%s – Fig4: LMFz slice at N=0', label));
hold on; grid on; view(30, 30);
xlabel('L [Nm]'); ylabel('M [Nm]'); zlabel('Fz [N]');
title(sprintf('%s | [L,M,Fz] slice at N = 0', label));

[K_s_nom,   V_s_nom]   = hull4d(P_N_nom);
[K_s_fault, V_s_fault] = hull4d(P_N_fault);
fprintf('  LMFz slice at N=0 – nominal volume: %.4f  fault volume: %.4f\n\n', V_s_nom, V_s_fault);

if ~isempty(K_s_nom) && size(P_N_nom, 1) >= 4
    trisurf(K_s_nom, P_N_nom(:,1), P_N_nom(:,2), P_N_nom(:,3), ...
        'FaceColor', c_nom, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', a_nom);
end
if ~isempty(K_s_fault) && size(P_N_fault, 1) >= 4
    trisurf(K_s_fault, P_N_fault(:,1)*vc, P_N_fault(:,2)*vc, P_N_fault(:,3)*vc, ...
        'FaceColor', c_fault, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', a_fault);
end
draw_axes_3d(lw_ax);
legend({'Nominal', fault_label}, 'Location', 'best');

%% ── FIGURE 5: Double slice Fz=-W, N=0  →  [L, M]  (2D moment polygon) ──
figure('Name', sprintf('%s – Fig5: LM slice at Fz=-W, N=0', label));
hold on; grid on; axis equal;
xlabel('L [Nm]'); ylabel('M [Nm]');
title(sprintf('%s | [L,M] slice at Fz = -W, N = 0  (hover moment polygon)', label));

if size(P_LM_nom, 1) >= 3 && rank(P_LM_nom - mean(P_LM_nom)) >= 2
    K_2d_nom = convhull(P_LM_nom(:,1), P_LM_nom(:,2));
    area_nom = polyarea(P_LM_nom(K_2d_nom,1), P_LM_nom(K_2d_nom,2));
    fprintf('  LM hover polygon area – nominal: %.4f Nm^2\n', area_nom);
    patch(P_LM_nom(K_2d_nom,1), P_LM_nom(K_2d_nom,2), c_nom, ...
        'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', a_nom);
else
    fprintf('  WARNING: nominal LM slice is degenerate.\n');
end

if size(P_LM_fault, 1) >= 3 && rank(P_LM_fault - mean(P_LM_fault)) >= 2
    K_2d_fault = convhull(P_LM_fault(:,1), P_LM_fault(:,2));
    area_fault = polyarea(P_LM_fault(K_2d_fault,1), P_LM_fault(K_2d_fault,2));
    fprintf('  LM hover polygon area – fault:   %.4f Nm^2\n', area_fault);
    if exist('area_nom', 'var') && area_nom > 0
        fprintf('  LM hover polygon retention:      %.4f\n\n', area_fault / area_nom);
    end
    patch(P_LM_fault(K_2d_fault,1)*vc, P_LM_fault(K_2d_fault,2)*vc, c_fault, ...
        'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', a_fault);
else
    fprintf('  WARNING: fault LM slice is degenerate.\n\n');
end

xline(0, 'k-', 'LineWidth', 0.8); yline(0, 'k-', 'LineWidth', 0.8);
legend({'Nominal', fault_label}, 'Location', 'best');
end

%% ── Local helpers ─────────────────────────────────────────────────────────

function pts = acs_pts_4d(B, T_max, loe_vec)
% Generate the 4D ACS vertex point cloud [N_pts × 4] = [Fz, L, M, N].
n  = 6;
nv = 2^n;
T_ub = T_max * (1 - loe_vec(:));
T_verts = zeros(n, nv);
for k = 1:n
    period  = 2^(k-1);
    pattern = repmat([zeros(1,period), ones(1,period)], 1, nv/(2*period));
    T_verts(k,:) = T_ub(k) * pattern;
end
V   = B * T_verts;              % [4 × nv]
pts = unique(round(V', 10), 'rows');   % [N_pts × 4]
end

function [hull_idx, vol] = hull4d(pts)
% Compute convex hull of a d-dimensional point set.
% Uses Qx (exact arithmetic) for accurate edge enumeration in visualization.
% Falls back to Qt+Pp if Qx hangs or fails.
vol = 0; hull_idx = [];
n = size(pts, 1);
d = size(pts, 2);
if n < d + 1, return; end
pts_c = pts - mean(pts, 1);
sv    = svd(pts_c);
if sv(end) / max(sv(1), 1e-12) < 1e-9, return; end   % near-degenerate
try
    [hull_idx, vol] = convhulln(pts, {'Qt', 'Qx'});
catch
    try
        [hull_idx, vol] = convhulln(pts, {'Qt', 'Pp'});
    catch
        hull_idx = []; vol = 0;
    end
end
end

function E = hull_edges(hull_idx, d)
% Extract all unique edges from a d-dimensional convex hull.
% Each simplex facet has C(d, 2) edges.
if isempty(hull_idx), E = zeros(0,2); return; end
pairs = nchoosek(1:d, 2);
E = [];
for k = 1:size(pairs, 1)
    E = [E; hull_idx(:, pairs(k,:))]; %#ok<AGROW>
end
E = unique(sort(E, 2), 'rows');
end

function out_pts = slice_at(pts_4d, edges, dim_cut, val_cut, out_cols, tol)
% Slice a convex polytope at pts_4d(:, dim_cut) == val_cut.
% Returns intersection points with columns reordered by out_cols.
out_pts = zeros(0, numel(out_cols));
for i = 1:size(edges, 1)
    p1 = pts_4d(edges(i,1), :);
    p2 = pts_4d(edges(i,2), :);
    v1 = p1(dim_cut) - val_cut;
    v2 = p2(dim_cut) - val_cut;
    if abs(v1) < tol && abs(v2) < tol
        out_pts(end+1, :) = p1(out_cols); %#ok<AGROW>
        out_pts(end+1, :) = p2(out_cols); %#ok<AGROW>
    elseif abs(v1) < tol
        out_pts(end+1, :) = p1(out_cols); %#ok<AGROW>
    elseif abs(v2) < tol
        out_pts(end+1, :) = p2(out_cols); %#ok<AGROW>
    elseif v1 * v2 < 0
        a = -v1 / (v2 - v1);
        pint = p1 + a * (p2 - p1);
        out_pts(end+1, :) = pint(out_cols); %#ok<AGROW>
    end
end
out_pts = unique(round(out_pts, 10), 'rows');
end

function draw_axes_3d(lw)
% Draw coordinate axes on the current 3D axes.
xl = xlim; yl = ylim; zl = zlim;
plot3([xl(1) xl(2)], [0 0], [0 0], 'k-', 'LineWidth', lw);
plot3([0 0], [yl(1) yl(2)], [0 0], 'k-', 'LineWidth', lw);
plot3([0 0], [0 0], [zl(1) zl(2)], 'k-', 'LineWidth', lw);
end
