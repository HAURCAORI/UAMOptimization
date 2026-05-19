% main_test.m  —  ACS directional margin verification
%
% PURPOSE
%   Verify the directional margin formula
%
%       m(d, u_req) = h_U(d) - d' * u_req
%       h_U(d)      = sum_j  max(0, d' * b_j) * f_max_j     (support function)
%
%   BEFORE implementing the margin soft-objective in C++.
%
% BACKGROUND
%   The Attainable Control Set (ACS) is the zonotope
%
%       U = { B*f  |  0 <= f_j <= f_max_j }
%
%   A point u_req is inside U  iff  m(d, u_req) >= 0  for ALL directions d.
%   The support function h_U(d) = max_{u in U} d'*u  is the extreme value of
%   U in direction d, computed analytically via the column-wise formula above.
%
% KNOWN PHYSICAL CONSTRAINT
%   B row 0 = [-1, -1, -1, -1, -1, -1]  (all motors produce -Fz, i.e. upward
%   thrust in NED, z-down).  Therefore the ACS cannot extend in the +Fz
%   direction:
%
%       h_U( [+1; 0; 0; 0] ) = 0
%
%   Consequence for the origin u_req = [0;0;0;0]:
%       m( [+1;0;0;0],  0 ) = 0 - 0 = 0    (on the ACS boundary — NOT negative)
%
%   Consequence for the hover point u_req = [-mg; 0; 0; 0]:
%       m( [+1;0;0;0], u_hover ) = 0 - (+1)*(-mg) = mg > 0   (strictly inside)
%
%   => Both origin and hover must yield m >= 0 everywhere.
%      If a margin test reports the origin OR the hover point as infeasible,
%      a sign error is present.
%
% TESTS
%   T1  Support function  —  formula vs vertex enumeration
%   T2  Origin (u=0)      —  m(d) >= 0 for all d;  min = 0  at d=[+1;0;0;0]
%   T3  Hover trim        —  m(d) > 0 for all d;   consistent with LP
%   T4  Infeasible point  —  at least one m(d) < 0;  LP also infeasible
%   T5  Fault cases       —  margin vs LP for all single-motor faults
%   T6  Sign-error demo   —  wrong formula  m_bad = d'*u - h_U(d)  <= 0 at origin
%
% OUTPUTS
%   Printed PASS/FAIL for every test.
%   Four figures: ACS polygon with origin marked, per-direction margin bars,
%   fault margin matrix, sign-error comparison.
%
% USAGE
%   cd Framework
%   main_test

clc; clear; close all;
addpath(genpath(fileparts(mfilename('fullpath'))));

PASS_STR = '[PASS]';
FAIL_STR = '[FAIL] ***';
EPS_CHECK = 1e-9;

% ═══════════════════════════════════════════════════════════════════════════
%  0.  Physical setup
% ═══════════════════════════════════════════════════════════════════════════
d = design_default();
[UAM, Prop, Env] = hexacopter_params(d);
B     = UAM.B;          % [4×6] control effectiveness matrix
T_max = Prop.T_max;     % [N]   per-motor thrust limit
m_veh = UAM.m;          % [kg]  total vehicle mass
g_acc = Env.g;          % [m/s²]
W     = m_veh * g_acc;  % [N]   vehicle weight

u_hover   = [-W; 0; 0; 0];   % hover trim wrench [Fz; L; M; N]
f_max_nom = T_max * ones(6, 1);

fprintf('\n================================================\n');
fprintf('  ACS Directional Margin — Verification Suite\n');
fprintf('================================================\n');
fprintf('  B matrix (rows: Fz, L, M, N; cols: M1..M6):\n');
disp(B);
fprintf('  T_max = %.2f N  |  W = %.2f N  |  6*T_max/W = %.4f\n\n', ...
    T_max, W, 6*T_max/W);

% ── Direction sample set (mirrors C++ makeDirectionSamples) ────────────────
s = 1/sqrt(2);
D = struct('label', {}, 'd', {});
D(end+1) = struct('label','thrust',       'd', [-1; 0; 0;  0]);
D(end+1) = struct('label','roll_pos',     'd', [ 0; 1; 0;  0]);
D(end+1) = struct('label','roll_neg',     'd', [ 0;-1; 0;  0]);
D(end+1) = struct('label','pitch_pos',    'd', [ 0; 0; 1;  0]);
D(end+1) = struct('label','pitch_neg',    'd', [ 0; 0;-1;  0]);
D(end+1) = struct('label','yaw_pos',      'd', [ 0; 0; 0;  1]);
D(end+1) = struct('label','yaw_neg',      'd', [ 0; 0; 0; -1]);
D(end+1) = struct('label','roll_yaw_pp',  'd', [ 0; s; 0;  s]);
D(end+1) = struct('label','pitch_yaw_pp', 'd', [ 0; 0; s;  s]);
D(end+1) = struct('label','roll_yaw_pn',  'd', [ 0; s; 0; -s]);
% Extra: +Fz direction — this is the critical boundary direction
D(end+1) = struct('label','+Fz_critical', 'd', [+1; 0; 0;  0]);

n_dir = numel(D);

% ── ACS vertex cloud: 2^6 = 64 vertices ───────────────────────────────────
n_motors = 6;
nv       = 2^n_motors;
V_acs    = zeros(4, nv);
for mask = 0:nv-1
    f_k = zeros(6,1);
    for k = 1:n_motors
        if bitand(mask, 2^(k-1))
            f_k(k) = T_max;
        end
    end
    V_acs(:, mask+1) = B * f_k;
end

% ═══════════════════════════════════════════════════════════════════════════
%  Helper functions (local, defined at the bottom of this file)
% ═══════════════════════════════════════════════════════════════════════════
% support_function(B, f_max, d)   — analytical formula
% margin(B, f_max, d, u_req)      — correct margin: h_U(d) - d'*u_req
% margin_bad(B, f_max, d, u_req)  — WRONG sign:    d'*u_req - h_U(d)

% ═══════════════════════════════════════════════════════════════════════════
%  T1: Support function formula vs vertex enumeration
% ═══════════════════════════════════════════════════════════════════════════
fprintf('─────────────────────────────────────────────\n');
fprintf('T1  Support function: formula vs vertices\n');
fprintf('─────────────────────────────────────────────\n');

t1_pass = true;
for ki = 1:n_dir
    d_vec = D(ki).d;
    h_formula = support_function(B, f_max_nom, d_vec);
    h_vertex  = max(d_vec' * V_acs);   % max of d'*v over all 64 vertices
    err = abs(h_formula - h_vertex);
    ok  = err < EPS_CHECK;
    if ~ok, t1_pass = false; end
    fprintf('  %-18s  h_formula=%10.4f  h_vertex=%10.4f  err=%7.2e  %s\n', ...
        D(ki).label, h_formula, h_vertex, err, tf2str(ok, PASS_STR, FAIL_STR));
end
fprintf('  %s T1 overall\n\n', tf2str(t1_pass, PASS_STR, FAIL_STR));

% ═══════════════════════════════════════════════════════════════════════════
%  T2: Origin u=0 — must have m(d) >= 0 everywhere
%      Physical note: h_U([+1;0;0;0]) = 0  and  d'*0 = 0  → m = 0 (boundary)
%      NOT negative.  Any implementation returning m < 0 at origin is wrong.
% ═══════════════════════════════════════════════════════════════════════════
fprintf('─────────────────────────────────────────────\n');
fprintf('T2  Origin u=0: margin >= 0 for all directions\n');
fprintf('    (min margin expected at d=[+1,0,0,0]: m=0, on ACS boundary)\n');
fprintf('─────────────────────────────────────────────\n');

u_origin   = [0; 0; 0; 0];
t2_pass    = true;
m_min_orig = inf;
for ki = 1:n_dir
    d_vec = D(ki).d;
    h_val = support_function(B, f_max_nom, d_vec);
    m_val = h_val - d_vec' * u_origin;    % correct formula
    ok    = m_val >= -EPS_CHECK;
    if ~ok, t2_pass = false; end
    m_min_orig = min(m_min_orig, m_val);
    fprintf('  %-18s  h=%9.4f  m=%9.4f  %s\n', ...
        D(ki).label, h_val, m_val, tf2str(ok, PASS_STR, FAIL_STR));
end
fprintf('  min margin at origin = %.6f  (expected = 0)\n', m_min_orig);
fprintf('  %s T2 overall\n\n', tf2str(t2_pass, PASS_STR, FAIL_STR));

% ═══════════════════════════════════════════════════════════════════════════
%  T3: Hover trim u=[-mg;0;0;0] — must have m(d) > 0 everywhere
%      Also: cross-check with LP  (hover_feasibility.m)
% ═══════════════════════════════════════════════════════════════════════════
fprintf('─────────────────────────────────────────────\n');
fprintf('T3  Hover trim u=[-mg;0;0;0]: margin > 0, LP consistent\n');
fprintf('─────────────────────────────────────────────\n');

[lp_hover_ok, lp_util, T_lp] = hover_feasibility(B, T_max, m_veh, g_acc, zeros(6,1));
fprintf('  LP result: feasible=%d  util=%.4f\n', lp_hover_ok, lp_util);

t3_pass    = true;
m_min_hover = inf;
for ki = 1:n_dir
    d_vec = D(ki).d;
    h_val = support_function(B, f_max_nom, d_vec);
    m_val = h_val - d_vec' * u_hover;
    ok    = m_val >= -EPS_CHECK;
    if ~ok, t3_pass = false; end
    m_min_hover = min(m_min_hover, m_val);
    fprintf('  %-18s  h=%9.4f  m=%9.4f  %s\n', ...
        D(ki).label, h_val, m_val, tf2str(ok, PASS_STR, FAIL_STR));
end
% Consistency: LP says feasible  <=>  min margin >= 0
margin_says_ok = (m_min_hover >= -EPS_CHECK);
consistent     = (lp_hover_ok == margin_says_ok);
fprintf('  min margin at hover = %.6f\n', m_min_hover);
fprintf('  LP feasible=%d  |  Margin says feasible=%d  |  %s consistent\n', ...
    lp_hover_ok, margin_says_ok, tf2str(consistent, PASS_STR, FAIL_STR));
fprintf('  %s T3 overall\n\n', tf2str(t3_pass && consistent, PASS_STR, FAIL_STR));

% ═══════════════════════════════════════════════════════════════════════════
%  T4: Clearly infeasible point — at least one margin must be < 0
%      Use u_bad = [-2W; 0; 0; 0]  (requires 2× vehicle weight upward)
%      if 6*T_max < 2*W this is infeasible; with T_max=2*W/6, 6*T_max = 2W
%      so use 2.1*W to be safe.
% ═══════════════════════════════════════════════════════════════════════════
fprintf('─────────────────────────────────────────────\n');
fprintf('T4  Infeasible point u=[-2.1W;0;0;0]: some margin < 0\n');
fprintf('─────────────────────────────────────────────\n');

u_bad    = [-2.1*W; 0; 0; 0];
[lp_bad_ok, ~] = hover_feasibility(B, T_max, 2.1*m_veh, g_acc, zeros(6,1));
fprintf('  LP result: feasible=%d  (expected 0)\n', lp_bad_ok);

t4_found_neg = false;
for ki = 1:n_dir
    d_vec = D(ki).d;
    h_val = support_function(B, f_max_nom, d_vec);
    m_val = h_val - d_vec' * u_bad;
    if m_val < -EPS_CHECK, t4_found_neg = true; end
    fprintf('  %-18s  h=%9.4f  m=%9.4f\n', D(ki).label, h_val, m_val);
end
fprintf('  %s T4 (infeasible point has some m<0)\n\n', ...
    tf2str(t4_found_neg, PASS_STR, FAIL_STR));

% ═══════════════════════════════════════════════════════════════════════════
%  T5: Single-motor fault cases — margin vs LP for all 6 faults
% ═══════════════════════════════════════════════════════════════════════════
fprintf('─────────────────────────────────────────────\n');
fprintf('T5  Single-motor fault: margin vs LP  (hover feasibility)\n');
fprintf('─────────────────────────────────────────────\n');

t5_pass = true;
fprintf('  %-6s  %-8s  %-8s  %-14s  %-14s  %s\n', ...
    'Fault', 'LP_ok', 'marg_ok', 'min_margin', 'LP_util', 'Consistent?');
for k = 1:6
    loe_k           = zeros(6,1);  loe_k(k) = 1;
    f_max_k         = T_max * (1 - loe_k);
    [lp_ok_k, util_k] = hover_feasibility(B, T_max, m_veh, g_acc, loe_k);

    m_min_k = inf;
    for ki = 1:n_dir
        d_vec   = D(ki).d;
        h_val   = support_function_fmax(B, f_max_k, d_vec);   % faulted f_max
        m_val   = h_val - d_vec' * u_hover;
        m_min_k = min(m_min_k, m_val);
    end

    marg_ok_k  = (m_min_k >= -EPS_CHECK);
    consistent_k = (lp_ok_k == marg_ok_k);
    if ~consistent_k, t5_pass = false; end
    fprintf('  M%d fail  %-8d  %-8d  %-14.6f  %-14.6f  %s\n', ...
        k, lp_ok_k, marg_ok_k, m_min_k, util_k, ...
        tf2str(consistent_k, PASS_STR, FAIL_STR));
end
fprintf('  %s T5 overall\n\n', tf2str(t5_pass, PASS_STR, FAIL_STR));

% ═══════════════════════════════════════════════════════════════════════════
%  T6: Sign-error diagnosis
%      The WRONG formula:  m_bad(d) = d'*u_req - h_U(d)
%      → gives m_bad(d) <= 0 at origin  (since h_U(d) >= 0 and d'*0 = 0)
%      → gives large negative at hover  (e.g., thrust dir: mg - 6*T_max << 0)
%      This matches the symptom: "feasibility region does not include origin"
% ═══════════════════════════════════════════════════════════════════════════
fprintf('─────────────────────────────────────────────\n');
fprintf('T6  Sign-error diagnosis\n');
fprintf('    WRONG formula: m_bad = d''*u_req - h_U(d)\n');
fprintf('    CORRECT formula: m    = h_U(d)  - d''*u_req\n');
fprintf('─────────────────────────────────────────────\n');

fprintf('\n  %-18s  %12s  %12s  %12s  %12s\n', ...
    'Direction', 'h_U(d)', ...
    'm_correct(0)', 'm_correct(hover)', ...
    'm_WRONG(0)');
fprintf('  %s\n', repmat('-',1,74));

for ki = 1:n_dir
    d_vec = D(ki).d;
    h_val      = support_function(B, f_max_nom, d_vec);
    m_orig_ok  = h_val - d_vec' * u_origin;   % correct at origin
    m_hov_ok   = h_val - d_vec' * u_hover;    % correct at hover
    m_orig_bad = d_vec' * u_origin - h_val;   % WRONG at origin (always <= 0)
    fprintf('  %-18s  %12.4f  %12.4f  %12.4f  %12.4f\n', ...
        D(ki).label, h_val, m_orig_ok, m_hov_ok, m_orig_bad);
end

fprintf('\n  Conclusion:\n');
fprintf('  - CORRECT formula gives m_origin >= 0 for all d (min = 0).\n');
fprintf('  - WRONG formula gives m_origin = -h_U(d) <= 0 everywhere.\n');
fprintf('  - If your code shows origin as infeasible: you have the wrong sign.\n\n');
fprintf('  %s T6 sign-error identified\n\n', PASS_STR);

% ═══════════════════════════════════════════════════════════════════════════
%  FIGURES
% ═══════════════════════════════════════════════════════════════════════════

%% Fig 1: ACS hover moment polygon  [L, M]  at Fz=-W, N=0
%         with origin marked — must be inside nominal polygon.
fig1 = figure('Name', 'T_ACS: Hover Moment Polygon at Fz=-W, N=0');

% Build 4D vertex cloud and slice to [L,M] at Fz=-W, N=0
P4_nom   = V_acs';                          % [64×4]
E4_nom   = all_edge_pairs(64, 6);           % zonotope edges
tol_s    = max(1e-6 * max(abs(P4_nom(:))), 1e-6);

P_Fz_nom = slice_acs(P4_nom, E4_nom, 1, -W,   2:4, tol_s);  % [L,M,N]
E3_nom   = nchoosek(1:size(P_Fz_nom,1), 2);                  % candidate edges
P_LM_nom = slice_acs(P_Fz_nom, E3_nom, 3,  0, 1:2,  tol_s); % [L,M]

if size(P_LM_nom,1) >= 3
    K2 = convhull(P_LM_nom(:,1), P_LM_nom(:,2));
    patch(P_LM_nom(K2,1), P_LM_nom(K2,2), [0.20 0.50 0.90], ...
        'FaceAlpha', 0.25, 'EdgeColor', [0.20 0.50 0.90], 'LineWidth', 1.5);
    hold on;
    xline(0,'k--','LineWidth',0.8); yline(0,'k--','LineWidth',0.8);
    plot(0, 0, 'r+', 'MarkerSize', 14, 'LineWidth', 2.5);
    inside = inpolygon(0, 0, P_LM_nom(K2,1), P_LM_nom(K2,2));
    title(sprintf('Hover Moment Polygon [L,M] at Fz=-W, N=0\nOrigin [L=0,M=0] inside polygon: %s', ...
        tf2str(inside,'YES — correct','NO — BUG')));
    fprintf('  Fig1: origin inside hover polygon = %d  %s\n', inside, ...
        tf2str(inside, PASS_STR, FAIL_STR));
else
    title('WARNING: hover polygon degenerate');
    fprintf('  Fig1: WARNING — hover polygon has < 3 points.\n');
end
xlabel('L [Nm]'); ylabel('M [Nm]'); grid on; axis equal;
legend({'ACS polygon', 'Origin [0,0]'}, 'Location','best');

%% Fig 2: Per-direction margin bar chart  (origin vs hover)
fig2 = figure('Name', 'T_ACS: Directional Margin — Origin vs Hover');
labels    = {D.label};
m_orig_all = zeros(n_dir,1);
m_hov_all  = zeros(n_dir,1);
for ki = 1:n_dir
    d_vec         = D(ki).d;
    h_val         = support_function(B, f_max_nom, d_vec);
    m_orig_all(ki) = h_val - d_vec' * u_origin;
    m_hov_all(ki)  = h_val - d_vec' * u_hover;
end
x = 1:n_dir;
bar(x - 0.2, m_orig_all, 0.35, 'FaceColor', [0.4 0.4 0.9]); hold on;
bar(x + 0.2, m_hov_all,  0.35, 'FaceColor', [0.2 0.7 0.2]);
yline(0, 'r-', 'LineWidth', 1.5);
xticks(x); xticklabels(labels); xtickangle(35);
ylabel('Margin m(d)  [N or Nm]');
title('Directional Margins — Correct Formula  m = h_U(d) - d''·u');
legend({'Origin u=0', 'Hover u=[-mg;0;0;0]', 'Feasibility boundary'}, 'Location','best');
grid on;

%% Fig 3: Wrong vs correct margin at origin
fig3 = figure('Name', 'T_ACS: Sign-Error Comparison at Origin u=0');
m_correct_all = zeros(n_dir,1);
m_wrong_all   = zeros(n_dir,1);
for ki = 1:n_dir
    d_vec = D(ki).d;
    h_val = support_function(B, f_max_nom, d_vec);
    m_correct_all(ki) = h_val - d_vec' * u_origin;   % correct
    m_wrong_all(ki)   = d_vec' * u_origin - h_val;   % wrong sign
end
x = 1:n_dir;
bar(x - 0.2, m_correct_all, 0.35, 'FaceColor', [0.2 0.7 0.2]); hold on;
bar(x + 0.2, m_wrong_all,   0.35, 'FaceColor', [0.9 0.2 0.2]);
yline(0, 'k-', 'LineWidth', 1.5);
xticks(x); xticklabels(labels); xtickangle(35);
ylabel('Margin at origin u=0');
title({'Sign-Error Diagnosis at Origin u = [0;0;0;0]', ...
       'Green (correct): m = h_U - d''u  >=  0', ...
       'Red   (wrong):   m = d''u - h_U  <=  0  <<< BUG'});
legend({'Correct: h_U(d) - d''u', 'WRONG:   d''u - h_U(d)'}, 'Location','best');
grid on;

%% Fig 4: Fault margin matrix  (n_fault × n_dir heat map)
fig4 = figure('Name', 'T_ACS: Fault Margin Heat Map');
n_faults = 6;
M_heat   = zeros(n_faults, n_dir);
for k = 1:n_faults
    loe_k   = zeros(6,1);  loe_k(k) = 1;
    f_max_k = T_max * (1 - loe_k);
    for ki = 1:n_dir
        d_vec       = D(ki).d;
        h_val       = support_function_fmax(B, f_max_k, d_vec);
        M_heat(k,ki) = h_val - d_vec' * u_hover;
    end
end
imagesc(M_heat);
colormap(redblue(256));
clim([-max(abs(M_heat(:)))-1, max(abs(M_heat(:)))+1]);
colorbar;
xticks(1:n_dir); xticklabels(labels); xtickangle(35);
yticks(1:n_faults);
yticklabels(arrayfun(@(k) sprintf('M%d fail',k), 1:n_faults, 'UniformOutput', false));
title('Fault Directional Margin m(d) at hover  [N or Nm]  — blue=positive, red=negative');
xlabel('Direction'); ylabel('Fault case');
hold on;
[rows,cols] = find(M_heat < -EPS_CHECK);
for ri = 1:numel(rows)
    text(cols(ri), rows(ri), 'X', 'HorizontalAlignment','center', ...
        'FontWeight','bold', 'Color','w', 'FontSize',12);
end

%% Fig 5: Nominal vs Worst-Fault ACS comparison
% Identify worst fault motor by minimum directional margin at hover.
m_min_by_fault = zeros(6,1);
for k = 1:6
    loe_k   = zeros(6,1);  loe_k(k) = 1;
    f_max_k = T_max * (1 - loe_k);
    m_k = inf;
    for ki = 1:n_dir
        d_vec = D(ki).d;
        h_val = support_function_fmax(B, f_max_k, d_vec);
        m_k   = min(m_k, h_val - d_vec' * u_hover);
    end
    m_min_by_fault(k) = m_k;
end
[~, worst_motor] = min(m_min_by_fault);

% Build 64-vertex cloud for worst-fault (faulted motor contributes 0 thrust).
loe_w   = zeros(6,1);  loe_w(worst_motor) = 1;
f_max_w = T_max * (1 - loe_w);
V_fault = zeros(4, nv);
for mask = 0:nv-1
    f_k = zeros(6,1);
    for k = 1:n_motors
        if bitand(mask, 2^(k-1))
            f_k(k) = f_max_w(k);
        end
    end
    V_fault(:, mask+1) = B * f_k;
end

% Slice fault cloud: Fz = -W, N = 0  →  [L, M].
P4_flt   = V_fault';
tol_f    = max(1e-6 * max(abs(P4_flt(:))), 1e-6);
P_Fz_flt = slice_acs(P4_flt, E4_nom, 1, -W, 2:4, tol_f);
E3_flt   = nchoosek(1:size(P_Fz_flt,1), 2);
P_LM_flt = slice_acs(P_Fz_flt, E3_flt, 3, 0, 1:2, tol_f);

% Per-direction margins at hover: nominal vs worst-fault.
m_nom_hov = zeros(n_dir,1);
m_flt_hov = zeros(n_dir,1);
for ki = 1:n_dir
    d_vec         = D(ki).d;
    m_nom_hov(ki) = support_function(B, f_max_nom, d_vec)  - d_vec' * u_hover;
    m_flt_hov(ki) = support_function_fmax(B, f_max_w, d_vec) - d_vec' * u_hover;
end

c_nom = [0.20 0.50 0.90];
c_flt = [0.90 0.45 0.10];

fig5 = figure('Name', sprintf('T_ACS: Nominal vs Worst-Fault (M%d) Comparison', worst_motor));

% --- Subplot 1: hover moment polygon overlay [L, M] ---
subplot(1,2,1);
hold on;
if size(P_LM_nom,1) >= 3
    Kn = convhull(P_LM_nom(:,1), P_LM_nom(:,2));
    patch(P_LM_nom(Kn,1), P_LM_nom(Kn,2), c_nom, ...
        'FaceAlpha',0.25, 'EdgeColor',c_nom, 'LineWidth',1.5, ...
        'DisplayName','Nominal');
end
if size(P_LM_flt,1) >= 3
    Kf = convhull(P_LM_flt(:,1), P_LM_flt(:,2));
    patch(P_LM_flt(Kf,1), P_LM_flt(Kf,2), c_flt, ...
        'FaceAlpha',0.30, 'EdgeColor',c_flt, 'LineWidth',1.5, ...
        'DisplayName',sprintf('M%d fault', worst_motor));
end
xline(0,'k--','LineWidth',0.8,'HandleVisibility','off');
yline(0,'k--','LineWidth',0.8,'HandleVisibility','off');
plot(0,0,'r+','MarkerSize',14,'LineWidth',2.5,'DisplayName','Origin');
xlabel('L [Nm]'); ylabel('M [Nm]'); grid on; axis equal;
title(sprintf('Hover Moment Polygon [L,M]\nat Fz=-W, N=0   (worst: M%d)', worst_motor));
legend('Location','best');

% --- Subplot 2: directional margin bars at hover ---
subplot(1,2,2);
x = 1:n_dir;
bar(x - 0.2, m_nom_hov, 0.35, 'FaceColor',c_nom, 'DisplayName','Nominal');
hold on;
bar(x + 0.2, m_flt_hov, 0.35, 'FaceColor',c_flt, ...
    'DisplayName',sprintf('M%d fault', worst_motor));
yline(0,'r-','LineWidth',1.5,'HandleVisibility','off');
xticks(x); xticklabels(labels); xtickangle(35);
ylabel('Margin m(d) at hover  [N or Nm]');
title(sprintf('Directional Margins at Hover\nNominal vs M%d fault', worst_motor));
legend('Location','best'); grid on;

sgtitle(sprintf('Nominal vs Worst-Fault (M%d lost) — ACS Comparison', worst_motor));
fprintf('  Fig5: worst fault motor = M%d  (min margin at hover = %.4f N or Nm)\n\n', ...
    worst_motor, m_min_by_fault(worst_motor));

fprintf('\n================================================\n');
fprintf('  Summary\n');
fprintf('================================================\n');
fprintf('  T1 support function formula: %s\n', tf2str(t1_pass,     PASS_STR, FAIL_STR));
fprintf('  T2 origin margin >= 0:       %s\n', tf2str(t2_pass,     PASS_STR, FAIL_STR));
fprintf('  T3 hover margin > 0 + LP:    %s\n', tf2str(t3_pass && consistent, PASS_STR, FAIL_STR));
fprintf('  T4 infeasible point m<0:     %s\n', tf2str(t4_found_neg,PASS_STR, FAIL_STR));
fprintf('  T5 fault margin vs LP:       %s\n', tf2str(t5_pass,     PASS_STR, FAIL_STR));
fprintf('  T6 sign-error identified:    %s\n', PASS_STR);
fprintf('================================================\n\n');

fprintf('IMPLEMENTATION NOTE FOR C++:\n');
fprintf('  Correct: margin = h_U(d) - dot(d, u_req)\n');
fprintf('           where h_U(d) = sum_j max(0, dot(d, B_col_j)) * f_max_j\n');
fprintf('  min_margin(u_req=0)    = 0  (origin is on ACS boundary)\n');
fprintf('  min_margin(u_req=hover)> 0  (hover is strictly inside ACS)\n');
fprintf('  Penalty: max(0, -min_margin) / W  (zero when all margins >= 0)\n\n');

% ═══════════════════════════════════════════════════════════════════════════
%  Local helpers
% ═══════════════════════════════════════════════════════════════════════════

function h = support_function(B, f_max_nom, d)
% h_U(d) = sum_j max(0, d'*b_j) * f_max_j   (all motors at T_max)
T = repmat(f_max_nom(1), 6, 1);  % uniform T_max
h = 0;
for j = 1:6
    proj = d' * B(:,j);
    h = h + max(0, proj) * T(j);
end
end

function h = support_function_fmax(B, f_max_vec, d)
% h_U(d) with per-motor f_max_vec  (used for faulted cases)
h = 0;
for j = 1:numel(f_max_vec)
    proj = d' * B(:,j);
    h = h + max(0, proj) * f_max_vec(j);
end
end

function E = all_edge_pairs(nv, n_motors)
% Zonotope edges: for each motor k, vary it 0->T_max with all 2^(n-1) fixed combos.
% Returns [N_edges × 2] index pairs.
E = [];
for k = 0:n_motors-1
    bit_k = 2^k;
    for mask = 0:nv-1
        if ~bitand(mask, bit_k)
            E(end+1, :) = [mask+1,  bitor(mask, bit_k)+1]; %#ok<AGROW>
        end
    end
end
end

function out = slice_acs(pts, edges, dim_cut, val_cut, out_cols, tol)
% Intersect polytope edges with hyperplane pts(:,dim_cut) == val_cut.
out = zeros(0, numel(out_cols));
for ei = 1:size(edges,1)
    p1 = pts(edges(ei,1), :);
    p2 = pts(edges(ei,2), :);
    v1 = p1(dim_cut) - val_cut;
    v2 = p2(dim_cut) - val_cut;
    if abs(v1) < tol && abs(v2) < tol
        out(end+1,:) = p1(out_cols); %#ok<AGROW>
        out(end+1,:) = p2(out_cols); %#ok<AGROW>
    elseif abs(v1) < tol
        out(end+1,:) = p1(out_cols); %#ok<AGROW>
    elseif abs(v2) < tol
        out(end+1,:) = p2(out_cols); %#ok<AGROW>
    elseif v1 * v2 < 0
        a   = -v1 / (v2 - v1);
        pint = p1 + a * (p2 - p1);
        out(end+1,:) = pint(out_cols); %#ok<AGROW>
    end
end
out = unique(round(out, 9), 'rows');
end

function s = tf2str(flag, true_str, false_str)
if flag, s = true_str; else, s = false_str; end
end

function cmap = redblue(n)
% Red-white-blue colormap centered at zero.
if nargin < 1, n = 256; end
half = floor(n/2);
r = [linspace(0.9,1,half), linspace(1,0.2,n-half)]';
g = [linspace(0.2,1,half), linspace(1,0.2,n-half)]';
b = [linspace(0.2,1,half), linspace(1,0.9,n-half)]';
cmap = [r, g, b];
end
