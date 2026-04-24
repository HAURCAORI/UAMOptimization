% 2026 Spring AE50001 / Team5
% ACS analysis

%% MATLAB Initialization
clc; clear; close all;
set(0,'DefaultFigureWindowStyle','docked');

%% Parameters
% Reference: "Nonlinear Model Predictive Control ... ", p.6

% Aircraft Parameters
UAM.m = 2240.73;                        % Mass of the body [kg]
UAM.Ix = 12000;                         % Moment of inertia about X-axis [kg m^2]
UAM.Iy = 9400;                          % Moment of inertia about Y-axis [kg m^2]
UAM.Iz = 20000;                         % Moment of inertia about Z-axis [kg m^2]
               
UAM.Lx = 2.65;                          % Arm length along X-axis [m]
UAM.Lyi = 2.65;                         % Inner motor arm length along Y-axis [m]
UAM.Lyo = 5.5;                          % Outer motor arm length along Y-axis [m]

% Environment Paramters            
Env.g = 9.81;                           % Gravitational acceleration [m/s^2]
Env.rho = 1.225;                        % Air density [kg/m^3]
               
% Propulsive Paramters
Prop.T_max = UAM.m * 2 / 6 * Env.g;     % Maximum thrust [N]
Prop.T_min = 0;                         % Minimum thrust [N]
Prop.cT = 0.02;                         % Moment to thrust ratio

% Control Effectiveness Matrix
% Configuration 1
UAM.B1 = [    -1       -1       -1       -1       -1       -1;
         -UAM.Lyi  UAM.Lyi -UAM.Lyo  UAM.Lyo -UAM.Lyi  UAM.Lyi;
          UAM.Lx   UAM.Lx       0        0   -UAM.Lx  -UAM.Lx;
          Prop.cT -Prop.cT -Prop.cT  Prop.cT  Prop.cT -Prop.cT];

% Configuration 2
UAM.B2 = [    -1       -1       -1       -1       -1       -1;
         -UAM.Lyi  UAM.Lyi -UAM.Lyo  UAM.Lyo -UAM.Lyi  UAM.Lyi;
          UAM.Lx   UAM.Lx       0        0   -UAM.Lx  -UAM.Lx;
         -Prop.cT -Prop.cT  Prop.cT  Prop.cT -Prop.cT  Prop.cT];

UAM.B = UAM.B2;

% Loss of Effectivenss
LOE = [1 0 0 0 0 0];

% Weight
W = UAM.m * Env.g;

%% ACS VERTICES
T_values = linspace(0, Prop.T_max, 2);
[T1, T2, T3, T4, T5, T6] = ndgrid(T_values, T_values, T_values, T_values, T_values, T_values);

T_all   = [T1(:), T2(:), T3(:), T4(:), T5(:), T6(:)]';
T_norm  = T_all;
T_fault = diag(1-LOE) * T_all;

% Compute achievable virtual controls
V_norm  = UAM.B * T_norm;
V_fault = UAM.B * T_fault;

% Extract components
IDX_FZ = 1;
IDX_L  = 2;
IDX_M  = 3;
IDX_N  = 4;

Fz_norm = V_norm(IDX_FZ,:);
L_norm  = V_norm(IDX_L,:);
M_norm  = V_norm(IDX_M,:);
N_norm  = V_norm(IDX_N,:);

Fz_fault = V_fault(IDX_FZ,:);
L_fault  = V_fault(IDX_L,:);
M_fault  = V_fault(IDX_M,:);
N_fault  = V_fault(IDX_N,:);

%% PLOT SETTINGS
c_norm     = [0.20 0.50 0.90];
c_fault    = [0.95 0.30 0.20];
lw_cvx     = 0.5;
lw_axis    = 2.0;
alph_norm  = 0.25;
alph_fault = 0.5;

vc  = 0.999;   % visualization contraction for fault set
vc2 = 1;   % small Fz shift for visibility

tolFz = 1e-10;
tolN  = 1e-10;

%% COMMON 4D POINT CLOUDS
P4_norm  = unique(round([Fz_norm',  L_norm',  M_norm',  N_norm'],  12), 'rows');
P4_fault = unique(round([Fz_fault', L_fault', M_fault', N_fault'], 12), 'rows');

%% ── FIGURE 1: Projection → [L, M, N] ────────────────────────────────────
figure('Name','Proj LMN')
xlabel('L [Nm]'); ylabel('M [Nm]'); zlabel('N [Nm]')
view(30, 30)
grid on
hold on

P_proj_LMN_norm  = unique(round([L_norm',  M_norm',  N_norm'],  12), 'rows');
P_proj_LMN_fault = unique(round([L_fault', M_fault', N_fault'], 12), 'rows');

[K_LMN_norm, Vol_LMN_norm] = convhulln(P_proj_LMN_norm, {'Qt','Qx'});
fprintf('Projection volume of the [L, M, N] normal 3D convex hull: %.4f N^3·m^3\n', Vol_LMN_norm);
trisurf(K_LMN_norm, P_proj_LMN_norm(:,1), P_proj_LMN_norm(:,2), P_proj_LMN_norm(:,3), ...
        'FaceColor', c_norm, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', alph_norm);

[K_LMN_fault, Vol_LMN_fault] = convhulln(P_proj_LMN_fault, {'Qt','Qx'});
fprintf('Projection volume of the [L, M, N] fault 3D convex hull: %.4f N^3·m^3\n\n', Vol_LMN_fault);
trisurf(K_LMN_fault, P_proj_LMN_fault(:,1)*vc, P_proj_LMN_fault(:,2)*vc, P_proj_LMN_fault(:,3)*vc, ...
        'FaceColor', c_fault, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', alph_fault);

x_lim = xlim; y_lim = ylim; z_lim = zlim;
plot3([x_lim(1) x_lim(2)], [0 0], [0 0], 'k-', 'LineWidth', lw_axis);
plot3([0 0], [y_lim(1) y_lim(2)], [0 0], 'k-', 'LineWidth', lw_axis);
plot3([0 0], [0 0], [z_lim(1) z_lim(2)], 'k-', 'LineWidth', lw_axis);


%% ── FIGURE 2: Projection → [L, M, Fz] ───────────────────────────────────
figure('Name','Proj LMFz')
xlabel('L [Nm]'); ylabel('M [Nm]'); zlabel('Fz [N]')
view(30, 30)
grid on
hold on

P_proj_LMFz_norm  = unique(round([L_norm',  M_norm',  Fz_norm'],  12), 'rows');
P_proj_LMFz_fault = unique(round([L_fault', M_fault', Fz_fault'], 12), 'rows');

[K_LMFz_norm, Vol_LMFz_norm] = convhulln(P_proj_LMFz_norm, {'Qt','Qx'});
fprintf('Projection volume of the [L, M, Fz] normal 3D convex hull: %.4f N^3·m^2\n', Vol_LMFz_norm);
trisurf(K_LMFz_norm, P_proj_LMFz_norm(:,1), P_proj_LMFz_norm(:,2), P_proj_LMFz_norm(:,3), ...
        'FaceColor', c_norm, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', alph_norm);

[K_LMFz_fault, Vol_LMFz_fault] = convhulln(P_proj_LMFz_fault, {'Qt','Qx'});
fprintf('Projection volume of the [L, M, Fz] fault 3D convex hull: %.4f N^3·m^2\n\n', Vol_LMFz_fault);
trisurf(K_LMFz_fault, P_proj_LMFz_fault(:,1)*vc, P_proj_LMFz_fault(:,2)*vc, P_proj_LMFz_fault(:,3)*vc-vc2, ...
        'FaceColor', c_fault, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', alph_fault);

x_lim = xlim; y_lim = ylim; z_lim = zlim;
plot3([x_lim(1) x_lim(2)], [0 0], [0 0], 'k-', 'LineWidth', lw_axis);
plot3([0 0], [y_lim(1) y_lim(2)], [0 0], 'k-', 'LineWidth', lw_axis);
plot3([0 0], [0 0], [z_lim(1) z_lim(2)], 'k-', 'LineWidth', lw_axis);

%% ── FIGURE 3: Slice at Fz = -W  → [L, M, N] ─────────────────────────────
figure('Name','Section LMN at Fz=-W')
xlabel('L [Nm]'); ylabel('M [Nm]'); zlabel('N [Nm]')
title(sprintf('LMN slice at Fz = -W = %.2f N', -W))
view(30, 30)
grid on
hold on

% ---- Normal
K4_norm = convhulln(P4_norm, {'Qt','Qx'});
E4_norm = [K4_norm(:,[1 2]);
           K4_norm(:,[1 3]);
           K4_norm(:,[1 4]);
           K4_norm(:,[2 3]);
           K4_norm(:,[2 4]);
           K4_norm(:,[3 4])];
E4_norm = sort(E4_norm, 2);
E4_norm = unique(E4_norm, 'rows');

P_FzW_LMN_norm = [];

for i = 1:size(E4_norm,1)
    p1 = P4_norm(E4_norm(i,1),:);   % [Fz L M N]
    p2 = P4_norm(E4_norm(i,2),:);

    f1 = p1(1) + W;
    f2 = p2(1) + W;

    if abs(f1) < tolFz && abs(f2) < tolFz
        P_FzW_LMN_norm = [P_FzW_LMN_norm; p1(2:4); p2(2:4)];
    elseif abs(f1) < tolFz
        P_FzW_LMN_norm = [P_FzW_LMN_norm; p1(2:4)];
    elseif abs(f2) < tolFz
        P_FzW_LMN_norm = [P_FzW_LMN_norm; p2(2:4)];
    elseif f1 * f2 < 0
        a = -f1 / (f2 - f1);
        pint = p1 + a*(p2-p1);
        P_FzW_LMN_norm = [P_FzW_LMN_norm; pint(2:4)];
    end
end

P_FzW_LMN_norm = unique(round(P_FzW_LMN_norm, 12), 'rows');

if size(P_FzW_LMN_norm,1) >= 4 && rank(P_FzW_LMN_norm - mean(P_FzW_LMN_norm,1)) >= 3
    [K_FzW_LMN_norm, Vol_FzW_LMN_norm] = convhulln(P_FzW_LMN_norm, {'Qt','Qx'});
    fprintf('Slice volume of the normal LMN set at Fz = -W: %.4f N^3·m^3\n', Vol_FzW_LMN_norm);

    trisurf(K_FzW_LMN_norm, ...
            P_FzW_LMN_norm(:,1), P_FzW_LMN_norm(:,2), P_FzW_LMN_norm(:,3), ...
            'FaceColor', c_norm, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', alph_norm);
else
    fprintf('Normal case: no full 3D LMN slice exists at Fz = -W.\n');
end

% ---- Fault
K4_fault = convhulln(P4_fault, {'Qt','Qx'});
E4_fault = [K4_fault(:,[1 2]);
            K4_fault(:,[1 3]);
            K4_fault(:,[1 4]);
            K4_fault(:,[2 3]);
            K4_fault(:,[2 4]);
            K4_fault(:,[3 4])];
E4_fault = sort(E4_fault, 2);
E4_fault = unique(E4_fault, 'rows');

P_FzW_LMN_fault = [];

for i = 1:size(E4_fault,1)
    p1 = P4_fault(E4_fault(i,1),:);   % [Fz L M N]
    p2 = P4_fault(E4_fault(i,2),:);

    f1 = p1(1) + W;
    f2 = p2(1) + W;

    if abs(f1) < tolFz && abs(f2) < tolFz
        P_FzW_LMN_fault = [P_FzW_LMN_fault; p1(2:4); p2(2:4)];
    elseif abs(f1) < tolFz
        P_FzW_LMN_fault = [P_FzW_LMN_fault; p1(2:4)];
    elseif abs(f2) < tolFz
        P_FzW_LMN_fault = [P_FzW_LMN_fault; p2(2:4)];
    elseif f1 * f2 < 0
        a = -f1 / (f2 - f1);
        pint = p1 + a*(p2-p1);
        P_FzW_LMN_fault = [P_FzW_LMN_fault; pint(2:4)];
    end
end

P_FzW_LMN_fault = unique(round(P_FzW_LMN_fault, 12), 'rows');

if size(P_FzW_LMN_fault,1) >= 4 && rank(P_FzW_LMN_fault - mean(P_FzW_LMN_fault,1)) >= 3
    [K_FzW_LMN_fault, Vol_FzW_LMN_fault] = convhulln(P_FzW_LMN_fault, {'Qt','Qx'});
    fprintf('Slice volume of the fault LMN set at Fz = -W: %.4f N^3·m^3\n\n', Vol_FzW_LMN_fault);

    trisurf(K_FzW_LMN_fault, ...
            P_FzW_LMN_fault(:,1)*vc, P_FzW_LMN_fault(:,2)*vc, P_FzW_LMN_fault(:,3)*vc, ...
            'FaceColor', c_fault, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', alph_fault);
else
    fprintf('Fault case: no full 3D LMN slice exists at Fz = -W.\n\n');
end

x_lim = xlim; y_lim = ylim; z_lim = zlim;
plot3([x_lim(1) x_lim(2)], [0 0], [0 0], 'k-', 'LineWidth', lw_axis);
plot3([0 0], [y_lim(1) y_lim(2)], [0 0], 'k-', 'LineWidth', lw_axis);
plot3([0 0], [0 0], [z_lim(1) z_lim(2)], 'k-', 'LineWidth', lw_axis);

%% ── FIGURE 4: Slice at N = 0  → [L, M, Fz] ──────────────────────────────
figure('Name','Section LMFz at N=0')
xlabel('L [Nm]'); ylabel('M [Nm]'); zlabel('Fz [N]')
view(30, 30)
grid on
hold on

% ---- Normal
P_N0_LMFz_norm = [];

for i = 1:size(E4_norm,1)
    p1 = P4_norm(E4_norm(i,1),:);   % [Fz L M N]
    p2 = P4_norm(E4_norm(i,2),:);

    n1 = p1(4);
    n2 = p2(4);

    if abs(n1) < tolN && abs(n2) < tolN
        P_N0_LMFz_norm = [P_N0_LMFz_norm; p1([2 3 1]); p2([2 3 1])]; % [L M Fz]
    elseif abs(n1) < tolN
        P_N0_LMFz_norm = [P_N0_LMFz_norm; p1([2 3 1])];
    elseif abs(n2) < tolN
        P_N0_LMFz_norm = [P_N0_LMFz_norm; p2([2 3 1])];
    elseif n1 * n2 < 0
        a = -n1 / (n2 - n1);
        pint = p1 + a*(p2-p1);
        P_N0_LMFz_norm = [P_N0_LMFz_norm; pint([2 3 1])];
    end
end

P_N0_LMFz_norm = unique(round(P_N0_LMFz_norm, 12), 'rows');

if size(P_N0_LMFz_norm,1) >= 4 && rank(P_N0_LMFz_norm - mean(P_N0_LMFz_norm,1)) >= 3
    [K_N0_LMFz_norm, Vol_N0_LMFz_norm] = convhulln(P_N0_LMFz_norm, {'Qt','Qx'});
    fprintf('Slice volume of the normal [L, M, Fz] set at N = 0: %.4f N^3·m^2\n', Vol_N0_LMFz_norm);

    trisurf(K_N0_LMFz_norm, ...
            P_N0_LMFz_norm(:,1), P_N0_LMFz_norm(:,2), P_N0_LMFz_norm(:,3), ...
            'FaceColor', c_norm, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', alph_norm);
else
    fprintf('Normal case: no full 3D [L, M, Fz] slice exists at N = 0.\n');
end

% ---- Fault
P_N0_LMFz_fault = [];

for i = 1:size(E4_fault,1)
    p1 = P4_fault(E4_fault(i,1),:);   % [Fz L M N]
    p2 = P4_fault(E4_fault(i,2),:);

    n1 = p1(4);
    n2 = p2(4);

    if abs(n1) < tolN && abs(n2) < tolN
        P_N0_LMFz_fault = [P_N0_LMFz_fault; p1([2 3 1]); p2([2 3 1])];
    elseif abs(n1) < tolN
        P_N0_LMFz_fault = [P_N0_LMFz_fault; p1([2 3 1])];
    elseif abs(n2) < tolN
        P_N0_LMFz_fault = [P_N0_LMFz_fault; p2([2 3 1])];
    elseif n1 * n2 < 0
        a = -n1 / (n2 - n1);
        pint = p1 + a*(p2-p1);
        P_N0_LMFz_fault = [P_N0_LMFz_fault; pint([2 3 1])];
    end
end

P_N0_LMFz_fault = unique(round(P_N0_LMFz_fault, 12), 'rows');

if size(P_N0_LMFz_fault,1) >= 4 && rank(P_N0_LMFz_fault - mean(P_N0_LMFz_fault,1)) >= 3
    [K_N0_LMFz_fault, Vol_N0_LMFz_fault] = convhulln(P_N0_LMFz_fault, {'Qt','Qx'});
    fprintf('Slice volume of the fault [L, M, Fz] set at N = 0: %.4f N^3·m^2\n\n', Vol_N0_LMFz_fault);

    trisurf(K_N0_LMFz_fault, ...
            P_N0_LMFz_fault(:,1)*vc, P_N0_LMFz_fault(:,2)*vc, P_N0_LMFz_fault(:,3)*vc-vc2, ...
            'FaceColor', c_fault, 'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', alph_fault);
else
    fprintf('Fault case: no full 3D [L, M, Fz] slice exists at N = 0.\n\n');
end

x_lim = xlim; y_lim = ylim; z_lim = zlim;
plot3([x_lim(1) x_lim(2)], [0 0], [0 0], 'k-', 'LineWidth', lw_axis);
plot3([0 0], [y_lim(1) y_lim(2)], [0 0], 'k-', 'LineWidth', lw_axis);
plot3([0 0], [0 0], [z_lim(1) z_lim(2)], 'k-', 'LineWidth', lw_axis);

%% ── FIGURE 5: From Figure 3, slice again at N = 0  → [L, M] ────────────
figure('Name','Section LM at Fz=-W, N=0')
xlabel('L [Nm]'); ylabel('M [Nm]')
axis equal
grid on
hold on

% ---- Normal: use ONLY Figure 3 result [L M N]
P3_norm = P_FzW_LMN_norm;

if size(P3_norm,1) >= 4 && rank(P3_norm - mean(P3_norm,1)) >= 3
    K3_norm = convhulln(P3_norm, {'Qt','Qx'});

    E3_norm = [K3_norm(:,[1 2]);
               K3_norm(:,[1 3]);
               K3_norm(:,[2 3])];
    E3_norm = sort(E3_norm, 2);
    E3_norm = unique(E3_norm, 'rows');

    P_FzW_N0_LM_norm = [];

    for i = 1:size(E3_norm,1)
        p1 = P3_norm(E3_norm(i,1),:);   % [L M N]
        p2 = P3_norm(E3_norm(i,2),:);

        n1 = p1(3);
        n2 = p2(3);

        if abs(n1) < tolN && abs(n2) < tolN
            P_FzW_N0_LM_norm = [P_FzW_N0_LM_norm; p1(1:2); p2(1:2)];
        elseif abs(n1) < tolN
            P_FzW_N0_LM_norm = [P_FzW_N0_LM_norm; p1(1:2)];
        elseif abs(n2) < tolN
            P_FzW_N0_LM_norm = [P_FzW_N0_LM_norm; p2(1:2)];
        elseif n1 * n2 < 0
            a = -n1 / (n2 - n1);
            pint = p1 + a*(p2-p1);
            P_FzW_N0_LM_norm = [P_FzW_N0_LM_norm; pint(1:2)];
        end
    end

    P_FzW_N0_LM_norm = unique(round(P_FzW_N0_LM_norm, 12), 'rows');

    if size(P_FzW_N0_LM_norm,1) >= 3 && rank(P_FzW_N0_LM_norm - mean(P_FzW_N0_LM_norm,1)) >= 2
        K_LM_norm = convhull(P_FzW_N0_LM_norm(:,1), P_FzW_N0_LM_norm(:,2));
        area_LM_norm = polyarea(P_FzW_N0_LM_norm(K_LM_norm,1), P_FzW_N0_LM_norm(K_LM_norm,2));
        fprintf('Slice area of the normal LM set at Fz = -W and N = 0: %.4f N^2·m^2\n', area_LM_norm);

        patch(P_FzW_N0_LM_norm(K_LM_norm,1), P_FzW_N0_LM_norm(K_LM_norm,2), c_norm, ...
              'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', alph_norm);
    else
        fprintf('Normal case: no full 2D LM slice exists at Fz = -W and N = 0.\n');
    end
else
    fprintf('Normal case: Figure 3 LMN slice is not a full 3D body.\n');
end

% ---- Fault: use ONLY Figure 3 result [L M N]
P3_fault = P_FzW_LMN_fault;

if size(P3_fault,1) >= 4 && rank(P3_fault - mean(P3_fault,1)) >= 3
    K3_fault = convhulln(P3_fault, {'Qt','Qx'});

    E3_fault = [K3_fault(:,[1 2]);
                K3_fault(:,[1 3]);
                K3_fault(:,[2 3])];
    E3_fault = sort(E3_fault, 2);
    E3_fault = unique(E3_fault, 'rows');

    P_FzW_N0_LM_fault = [];

    for i = 1:size(E3_fault,1)
        p1 = P3_fault(E3_fault(i,1),:);   % [L M N]
        p2 = P3_fault(E3_fault(i,2),:);

        n1 = p1(3);
        n2 = p2(3);

        if abs(n1) < tolN && abs(n2) < tolN
            P_FzW_N0_LM_fault = [P_FzW_N0_LM_fault; p1(1:2); p2(1:2)];
        elseif abs(n1) < tolN
            P_FzW_N0_LM_fault = [P_FzW_N0_LM_fault; p1(1:2)];
        elseif abs(n2) < tolN
            P_FzW_N0_LM_fault = [P_FzW_N0_LM_fault; p2(1:2)];
        elseif n1 * n2 < 0
            a = -n1 / (n2 - n1);
            pint = p1 + a*(p2-p1);
            P_FzW_N0_LM_fault = [P_FzW_N0_LM_fault; pint(1:2)];
        end
    end

    P_FzW_N0_LM_fault = unique(round(P_FzW_N0_LM_fault, 12), 'rows');

    if size(P_FzW_N0_LM_fault,1) >= 3 && rank(P_FzW_N0_LM_fault - mean(P_FzW_N0_LM_fault,1)) >= 2
        K_LM_fault = convhull(P_FzW_N0_LM_fault(:,1), P_FzW_N0_LM_fault(:,2));
        area_LM_fault = polyarea(P_FzW_N0_LM_fault(K_LM_fault,1), P_FzW_N0_LM_fault(K_LM_fault,2));
        fprintf('Slice area of the fault LM set at Fz = -W and N = 0: %.4f N^2·m^2\n\n', area_LM_fault);

        patch(P_FzW_N0_LM_fault(K_LM_fault,1)*vc, P_FzW_N0_LM_fault(K_LM_fault,2)*vc, c_fault, ...
              'EdgeColor', 'k', 'LineWidth', lw_cvx, 'FaceAlpha', alph_fault);
    else
        fprintf('Fault case: no full 2D LM slice exists at Fz = -W and N = 0.\n\n');
    end
else
    fprintf('Fault case: Figure 3 LMN slice is not a full 3D body.\n\n');
end
