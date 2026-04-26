% 2026 Spring AE50001 / Team5
% sim_path_following.m
% Figure-8 Trajectory Following Simulation
%
% Controller Architecture:
%   Position P  →  Velocity PI (+FF)  →  phi/theta cmd  →  Attitude PI
%   Altitude P  →  Vz PI (+FF)        →  Fz cmd
%
% Trajectory:
%   Lissajous 1:2 figure-8:  x(t) = A*sin(wt),  y(t) = A*sin(2wt)
%   Derivatives computed analytically for velocity/acceleration feedforward.

%% Initialization
clc; clear; close all;
set(0,'DefaultFigureWindowStyle','docked');

%% ── Vehicle Parameters ───────────────────────────────────────────────────
UAM.m   = 2240.73;  % [kg]
UAM.Ix  = 12000;    % [kg·m²]
UAM.Iy  =  9400;    % [kg·m²]
UAM.Iz  = 20000;    % [kg·m²]

UAM.Lx  = 2.65;    % [m] arm length along x
UAM.Lyi = 2.65;    % [m] inner arm along y
UAM.Lyo = 5.50;    % [m] outer arm along y

Env.g   = 9.81;    % [m/s²]
Env.rho = 1.225;   % [kg/m³]

Prop.T_max = UAM.m * 2 / 6 * Env.g;  % [N] per motor
Prop.T_min = 0;
Prop.cT    = 0.03;   % moment-to-thrust ratio
Prop.tau   = 0.04;   % [s] 1st-order motor time constant

% Control Effectiveness Matrix  (PPNNPN configuration)
UAM.B = [    -1        -1        -1        -1        -1        -1;
         -UAM.Lyi   UAM.Lyi  -UAM.Lyo   UAM.Lyo  -UAM.Lyi   UAM.Lyi;
          UAM.Lx    UAM.Lx       0          0      -UAM.Lx   -UAM.Lx;
         -Prop.cT  -Prop.cT   Prop.cT    Prop.cT  -Prop.cT   Prop.cT];

LOE = [1 0 0 0 0 0];   % Loss-of-effectiveness (0 = no fault)

%% ── Trajectory Parameters ────────────────────────────────────────────────
fig8.A       = 100;              % [m]      figure-8 amplitude (x and y)
fig8.T_period = 120;             % [s]      one full figure-8 period
fig8.omega   = 2*pi / fig8.T_period;   % [rad/s]
fig8.z_cruise = 50;              % [m]      cruise altitude (positive-up)
fig8.t_start  = 20;              % [s]      vertical climb before figure-8 begins
fig8.n_laps   = 2;               % number of figure-8 laps to fly

%% ── Simulation Configuration ─────────────────────────────────────────────
CONFIG.act_saturation = 1;
CONFIG.fault          = 1;   % 0: healthy,  1: apply LOE

LOE_diag = diag(1 - LOE);
if CONFIG.fault
    B_eff = UAM.B * LOE_diag;
else
    B_eff = UAM.B;
end

tf       = fig8.t_start + fig8.n_laps * fig8.T_period;  % total sim time [s]
dt       = 0.002;
num_step = round(tf / dt);
t_vec    = (1:num_step) * dt;

% State indices
IDX.u = 1; IDX.v = 2; IDX.w = 3;
IDX.p = 4; IDX.q = 5; IDX.r = 6;
IDX.phi = 7; IDX.theta = 8; IDX.psi = 9;
IDX.x  = 10; IDX.y  = 11; IDX.z  = 12;
IDX.vx = 13; IDX.vy = 14; IDX.vz = 15;

% Initial state  [u v w | p q r | phi theta psi | x y z | vx vy vz]
X = zeros(15,1);
T = zeros(6,1);

% History buffers
outsim.X          = zeros(15, num_step);
outsim.X_dot      = zeros(15, num_step);
outsim.Ctrl_cmd   = zeros(4,  num_step);   % [Fz; L; M; N] commanded
outsim.Att_cmd    = zeros(3,  num_step);   % [phi; theta; psi] cmd  [rad]
outsim.Rate_cmd   = zeros(3,  num_step);   % [p; q; r] cmd          [rad/s]
outsim.V_cmd      = zeros(3,  num_step);   % [vx; vy; vz] cmd  (NED)[m/s]
outsim.pos_ref    = zeros(3,  num_step);   % [x_ref; y_ref; alt_ref] [m], alt positive-up
outsim.vel_ref    = zeros(3,  num_step);   % [vx_ref; vy_ref; vz_ref_up] [m/s]
outsim.Thrust     = zeros(6,  num_step);   % T_eff (after LOE)
outsim.Thrust_cmd = zeros(6,  num_step);   % T_cmd (allocator output)

%% ── Control Gains ────────────────────────────────────────────────────────
% Inner rate loop (PI)
Kp_p = 60000;   Kp_q = 50000;   Kp_r = 20000;
Ki_p = 30000;   Ki_q = 20000;   Ki_r = 1000;

% Outer attitude loop (P)
tau_phi   = 1.0;   % [s]
tau_theta = 1.0;   % [s]
tau_psi   = 5.0;   % [s]
r_max     = 0.1;   % [rad/s] yaw rate limit

% Altitude (P outer + PI inner)
tau_z   = 5.0;     % [s]
Kp_vz   = 2000;
Ki_vz   = 600;
Vz_max  = 3.0;     % [m/s]

% Horizontal position (P)
Kp_x    = 0.15;    % [1/s]  -- 1/tau_pos
Kp_y    = 0.15;
Vxy_max = 15.0;    % [m/s] horizontal speed saturation

% Horizontal velocity (PI + accel feedforward)
Kp_vx_h = 0.5;   Ki_vx_h = 0.05;
Kp_vy_h = 0.5;   Ki_vy_h = 0.05;
a_max_h  = 3.0;   % [m/s²] horizontal accel saturation

% Attitude angle limits from position controller output
phi_max   = deg2rad(15);   % [rad]
theta_max = deg2rad(15);   % [rad]

% Heading: hold constant (north)
psi_cmd = 0;   % [rad]

% Integral states
i_p = 0;  i_q = 0;  i_r = 0;
i_vz = 0;
i_vx = 0;  i_vy = 0;

%% ── Main Loop ────────────────────────────────────────────────────────────
fprintf('Running simulation  (tf = %.0f s, %d steps) ...\n', tf, num_step);
tic

for i = 1:num_step
    time = t_vec(i);

    % Unpack state
    u = X(IDX.u);  v = X(IDX.v);  w = X(IDX.w);
    p = X(IDX.p);  q = X(IDX.q);  r = X(IDX.r);
    phi   = X(IDX.phi);   theta = X(IDX.theta);  psi = X(IDX.psi);
    x  = X(IDX.x);   y  = X(IDX.y);   z  = X(IDX.z);
    vx = X(IDX.vx);  vy = X(IDX.vy);  vz = X(IDX.vz);

    % ── Reference Trajectory ─────────────────────────────────────────────
    t_traj = max(time - fig8.t_start, 0);   % time since figure-8 began

    if time < fig8.t_start
        % ── Climb phase: hover (x,y)=(0,0), ramp altitude ────────────────
        x_ref  = 0;   y_ref  = 0;
        vx_ref = 0;   vy_ref = 0;
        ax_ref = 0;   ay_ref = 0;

        alt_ref    = fig8.z_cruise * (time / fig8.t_start);   % [m, +up]
        vz_ref_alt = fig8.z_cruise / fig8.t_start;            % [m/s, +up]

    else
        % ── Figure-8 (Lissajous 1:2) ─────────────────────────────────────
        %   x(t) = A·sin(ω·t)         vx = A·ω·cos(ω·t)         ax = -A·ω²·sin(ω·t)
        %   y(t) = A·sin(2ω·t)        vy = 2A·ω·cos(2ω·t)       ay = -4A·ω²·sin(2ω·t)
        x_ref  =     fig8.A *              sin(    fig8.omega * t_traj);
        y_ref  =     fig8.A *              sin(2 * fig8.omega * t_traj);
        vx_ref =     fig8.A * fig8.omega * cos(    fig8.omega * t_traj);
        vy_ref = 2 * fig8.A * fig8.omega * cos(2 * fig8.omega * t_traj);
        ax_ref =    -fig8.A * fig8.omega^2      * sin(    fig8.omega * t_traj);
        ay_ref = -4 * fig8.A * fig8.omega^2     * sin(2 * fig8.omega * t_traj);

        alt_ref    = fig8.z_cruise;
        vz_ref_alt = 0;
    end

    % NED sign: vz_ref_NED < 0 means climbing
    vz_ref_NED = -vz_ref_alt;

    % ── Horizontal Position Controller  (P + velocity FF) ────────────────
    ex = x_ref - x;
    ey = y_ref - y;

    vx_cmd = min(max(Kp_x * ex + vx_ref, -Vxy_max), Vxy_max);
    vy_cmd = min(max(Kp_y * ey + vy_ref, -Vxy_max), Vxy_max);

    % ── Horizontal Velocity Controller  (PI + accel FF) ──────────────────
    evx = vx_cmd - vx;
    evy = vy_cmd - vy;
    i_vx = i_vx + evx * dt;
    i_vy = i_vy + evy * dt;

    ax_des = Kp_vx_h * evx + Ki_vx_h * i_vx + ax_ref;
    ay_des = Kp_vy_h * evy + Ki_vy_h * i_vy + ay_ref;
    ax_des = min(max(ax_des, -a_max_h), a_max_h);
    ay_des = min(max(ay_des, -a_max_h), a_max_h);

    % Desired acceleration → attitude command  (NED, ZYX, small angle)
    %   ax_NED ≈ -g·(θ·cosψ + φ·sinψ)
    %   ay_NED ≈ -g·(θ·sinψ - φ·cosψ)
    theta_cmd = min(max(-(ax_des*cos(psi) + ay_des*sin(psi)) / Env.g, -theta_max), theta_max);
    phi_cmd   = min(max((-ax_des*sin(psi) + ay_des*cos(psi)) / Env.g, -phi_max),   phi_max);

    % ── Altitude Controller  (P + vz FF) ─────────────────────────────────
    z_err  = -alt_ref - z;   % z_NED_des = -alt_ref
    vz_cmd = min(max(z_err / tau_z + vz_ref_NED, -Vz_max), Vz_max);

    % ── Vertical Velocity Controller  (PI) ───────────────────────────────
    evz   = vz_cmd - vz;
    i_vz  = i_vz + evz * dt;
    tilt  = max(cos(phi)*cos(theta), 0.5);
    Fz_cmd = (Kp_vz * evz + Ki_vz * i_vz - UAM.m * Env.g) / tilt;

    % ── Attitude Controller  (P outer → rate cmd) ────────────────────────
    phi_err   = phi_cmd   - phi;
    theta_err = theta_cmd - theta;
    psi_err   = wrapToPi(psi_cmd - psi);

    p_cmd = phi_err   / tau_phi;
    q_cmd = theta_err / tau_theta;
    r_cmd = min(max(psi_err / tau_psi, -r_max), r_max);

    % ── Rate Controller  (PI inner) ───────────────────────────────────────
    p_err = p_cmd - p;
    q_err = q_cmd - q;
    r_err = r_cmd - r;
    i_p = i_p + p_err * dt;
    i_q = i_q + q_err * dt;
    i_r = i_r + r_err * dt;

    L_cmd = Kp_p * p_err + Ki_p * i_p;
    M_cmd = Kp_q * q_err + Ki_q * i_q;
    N_cmd = Kp_r * r_err + Ki_r * i_r;

    % ── Control Allocation  (pseudoinverse, min ‖T_cmd‖²) ────────────────
    virtual_ctrl_cmd = [Fz_cmd; L_cmd; M_cmd; N_cmd];
    T_cmd = pinv(B_eff) * virtual_ctrl_cmd;

    % ── Motor Dynamics  (1st-order) ───────────────────────────────────────
    T_dot = (1/Prop.tau) * (T_cmd - T);
    T = T + T_dot * dt;
    if CONFIG.act_saturation
        T = min(Prop.T_max, max(T, Prop.T_min));
    end

    % ── Equations of Motion  (Euler integration) ─────────────────────────
    T_eff = LOE_diag * T;
    X_dot = eom(X, T_eff, UAM, Env);
    X     = X + X_dot * dt;
    X(IDX.psi) = wrapToPi(X(IDX.psi));

    % Inertial velocity  (recompute from body velocity after integration)
    phi_n = X(IDX.phi);  theta_n = X(IDX.theta);  psi_n = X(IDX.psi);
    sphi = sin(phi_n);  cphi = cos(phi_n);
    sth  = sin(theta_n); cth  = cos(theta_n);
    spsi = sin(psi_n);   cpsi = cos(psi_n);
    R_nb = [cpsi*cth,  cpsi*sth*sphi-spsi*cphi,  cpsi*sth*cphi+spsi*sphi;
            spsi*cth,  spsi*sth*sphi+cpsi*cphi,  spsi*sth*cphi-cpsi*sphi;
           -sth,        cth*sphi,                  cth*cphi              ];
    X(IDX.vx:IDX.vz) = R_nb * X(IDX.u:IDX.w);

    % ── Record ───────────────────────────────────────────────────────────
    outsim.X(:, i)          = X;
    outsim.X_dot(:, i)      = X_dot;
    outsim.Ctrl_cmd(:, i)   = virtual_ctrl_cmd;
    outsim.Att_cmd(:, i)    = [phi_cmd; theta_cmd; psi_cmd];
    outsim.Rate_cmd(:, i)   = [p_cmd; q_cmd; r_cmd];
    outsim.V_cmd(:, i)      = [vx_cmd; vy_cmd; vz_cmd];
    outsim.pos_ref(:, i)    = [x_ref;  y_ref;  alt_ref];   % alt positive-up
    outsim.vel_ref(:, i)    = [vx_ref; vy_ref; vz_ref_alt]; % vz positive-up
    outsim.Thrust(:, i)     = T_eff;
    outsim.Thrust_cmd(:, i) = T_cmd;
end

elapsed = toc;
fprintf('Simulation done.  Elapsed: %.1f s\n', elapsed);

%% ── Post-processing ──────────────────────────────────────────────────────
pos_x   = outsim.X(IDX.x,   :);
pos_y   = outsim.X(IDX.y,   :);
pos_z   = outsim.X(IDX.z,   :);
alt_h   = -pos_z;   % [m] positive-up

vel_x   = outsim.X(IDX.vx, :);
vel_y   = outsim.X(IDX.vy, :);
vel_z   = outsim.X(IDX.vz, :);   % NED (+down)

phi_h   = rad2deg(outsim.X(IDX.phi,   :));
theta_h = rad2deg(outsim.X(IDX.theta, :));
psi_h   = rad2deg(outsim.X(IDX.psi,   :));
p_h     = rad2deg(outsim.X(IDX.p, :));
q_h     = rad2deg(outsim.X(IDX.q, :));
r_h     = rad2deg(outsim.X(IDX.r, :));

phi_cmd_h   = rad2deg(outsim.Att_cmd(1,:));
theta_cmd_h = rad2deg(outsim.Att_cmd(2,:));
psi_cmd_h   = rad2deg(outsim.Att_cmd(3,:));
p_cmd_h     = rad2deg(outsim.Rate_cmd(1,:));
q_cmd_h     = rad2deg(outsim.Rate_cmd(2,:));
r_cmd_h     = rad2deg(outsim.Rate_cmd(3,:));

x_ref_h    = outsim.pos_ref(1,:);
y_ref_h    = outsim.pos_ref(2,:);
alt_ref_h  = outsim.pos_ref(3,:);
vx_ref_h   = outsim.vel_ref(1,:);
vy_ref_h   = outsim.vel_ref(2,:);
vz_ref_h   = outsim.vel_ref(3,:);   % positive-up

% Tracking RMSE (figure-8 phase only)
idx_track = t_vec > fig8.t_start;
rmse_x = sqrt(mean((pos_x(idx_track) - x_ref_h(idx_track)).^2));
rmse_y = sqrt(mean((pos_y(idx_track) - y_ref_h(idx_track)).^2));
rmse_z = sqrt(mean((alt_h(idx_track) - alt_ref_h(idx_track)).^2));
fprintf('Tracking RMSE (figure-8 phase):  X=%.2f m,  Y=%.2f m,  Z=%.2f m\n', ...
        rmse_x, rmse_y, rmse_z);

%% ── Figures ──────────────────────────────────────────────────────────────

% ── Figure 1: 3D Trajectory ──────────────────────────────────────────────
figure('Name','3D Trajectory')
plot3(x_ref_h, y_ref_h, alt_ref_h, 'r--', 'LineWidth', 1.2); hold on
plot3(pos_x,   pos_y,   alt_h,     'b-',  'LineWidth', 1.5)
plot3(pos_x(1),  pos_y(1),  alt_h(1),  'go', 'MarkerSize', 8, 'MarkerFaceColor','g')
plot3(pos_x(end),pos_y(end),alt_h(end),'rs', 'MarkerSize', 8, 'MarkerFaceColor','r')
grid on; xlabel('X [m]'); ylabel('Y [m]'); zlabel('Altitude [m]')
title('3D Trajectory – Figure-8')
legend('Reference','Actual','Start','End','Location','best')
view(45, 30)

% ── Figure 2: XY Top-down (figure-8 shape) ───────────────────────────────
figure('Name','XY Trajectory')
plot(x_ref_h, y_ref_h, 'r--', 'LineWidth', 1.2); hold on
plot(pos_x,   pos_y,   'b-',  'LineWidth', 1.5)
plot(pos_x(1),  pos_y(1),  'go', 'MarkerSize', 8, 'MarkerFaceColor','g')
plot(pos_x(end),pos_y(end),'rs', 'MarkerSize', 8, 'MarkerFaceColor','r')
grid on; axis equal
xlabel('X [m]'); ylabel('Y [m]')
title('Top-down View – Figure-8')
legend('Reference','Actual','Start','End','Location','best')

% ── Figure 3: Position vs. Reference ─────────────────────────────────────
figure('Name','Position')
subplot(3,2,1)
plot(t_vec, pos_x,   'b',  'LineWidth',1.2); hold on
plot(t_vec, x_ref_h, 'r--','LineWidth',1.0)
xline(fig8.t_start,'k:'); grid on; ylabel('X [m]'); title('Position vs. Reference')
legend('Actual','Reference','Location','best')

subplot(3,2,2)
plot(t_vec, pos_x - x_ref_h, 'm', 'LineWidth',1.2)
xline(fig8.t_start,'k:'); grid on; ylabel('e_x [m]'); title('Tracking Error')

subplot(3,2,3)
plot(t_vec, pos_y,   'b',  'LineWidth',1.2); hold on
plot(t_vec, y_ref_h, 'r--','LineWidth',1.0)
xline(fig8.t_start,'k:'); grid on; ylabel('Y [m]')
legend('Actual','Reference','Location','best')

subplot(3,2,4)
plot(t_vec, pos_y - y_ref_h, 'm', 'LineWidth',1.2)
xline(fig8.t_start,'k:'); grid on; ylabel('e_y [m]')

subplot(3,2,5)
plot(t_vec, alt_h,    'b',  'LineWidth',1.2); hold on
plot(t_vec, alt_ref_h,'r--','LineWidth',1.0)
xline(fig8.t_start,'k:'); grid on
ylabel('Altitude [m]'); xlabel('Time [s]')
legend('Actual','Reference','Location','best')

subplot(3,2,6)
plot(t_vec, alt_h - alt_ref_h, 'm', 'LineWidth',1.2)
xline(fig8.t_start,'k:'); grid on
ylabel('e_z [m]'); xlabel('Time [s]')

% ── Figure 4: Velocity ────────────────────────────────────────────────────
figure('Name','Velocity')
subplot(3,1,1)
plot(t_vec, vel_x,              'b',  'LineWidth',1.2); hold on
plot(t_vec, vx_ref_h,           'r--','LineWidth',1.0)
plot(t_vec, outsim.V_cmd(1,:),  'g:', 'LineWidth',0.8)
xline(fig8.t_start,'k:'); grid on
ylabel('Vx [m/s]'); title('Inertial Velocity')
legend('Actual','Reference','Cmd','Location','best')

subplot(3,1,2)
plot(t_vec, vel_y,              'b',  'LineWidth',1.2); hold on
plot(t_vec, vy_ref_h,           'r--','LineWidth',1.0)
plot(t_vec, outsim.V_cmd(2,:),  'g:', 'LineWidth',0.8)
xline(fig8.t_start,'k:'); grid on
ylabel('Vy [m/s]')
legend('Actual','Reference','Cmd','Location','best')

subplot(3,1,3)
plot(t_vec, -vel_z,  'b',  'LineWidth',1.2); hold on   % NED→positive-up
plot(t_vec, vz_ref_h,'r--','LineWidth',1.0)
xline(fig8.t_start,'k:'); grid on
ylabel('Vz [m/s, +up]'); xlabel('Time [s]')
legend('Actual','Reference','Location','best')

% ── Figure 5: Attitude ────────────────────────────────────────────────────
figure('Name','Attitude')
subplot(3,1,1)
plot(t_vec, phi_h,       'b',  'LineWidth',1.2); hold on
plot(t_vec, phi_cmd_h,   'r--','LineWidth',1.0)
xline(fig8.t_start,'k:'); grid on
ylabel('\phi [deg]'); title('Attitude vs. Command')
legend('Actual','Command','Location','best')

subplot(3,1,2)
plot(t_vec, theta_h,     'b',  'LineWidth',1.2); hold on
plot(t_vec, theta_cmd_h, 'r--','LineWidth',1.0)
xline(fig8.t_start,'k:'); grid on
ylabel('\theta [deg]')
legend('Actual','Command','Location','best')

subplot(3,1,3)
plot(t_vec, psi_h,     'b',  'LineWidth',1.2); hold on
plot(t_vec, psi_cmd_h, 'r--','LineWidth',1.0)
xline(fig8.t_start,'k:'); grid on
ylabel('\psi [deg]'); xlabel('Time [s]')
legend('Actual','Command','Location','best')

% ── Figure 6: Angular Rate ────────────────────────────────────────────────
figure('Name','Angular Rate')
subplot(3,1,1)
plot(t_vec, p_h,     'b',  'LineWidth',1.2); hold on
plot(t_vec, p_cmd_h, 'r--','LineWidth',1.0)
xline(fig8.t_start,'k:'); grid on
ylabel('p [deg/s]'); title('Angular Rate vs. Command')
legend('Actual','Command','Location','best')

subplot(3,1,2)
plot(t_vec, q_h,     'b',  'LineWidth',1.2); hold on
plot(t_vec, q_cmd_h, 'r--','LineWidth',1.0)
xline(fig8.t_start,'k:'); grid on
ylabel('q [deg/s]')
legend('Actual','Command','Location','best')

subplot(3,1,3)
plot(t_vec, r_h,     'b',  'LineWidth',1.2); hold on
plot(t_vec, r_cmd_h, 'r--','LineWidth',1.0)
xline(fig8.t_start,'k:'); grid on
ylabel('r [deg/s]'); xlabel('Time [s]')
legend('Actual','Command','Location','best')

% ── Figure 7: Thrust ──────────────────────────────────────────────────────
figure('Name','Thrust')
motor_labels = {'Motor 1','Motor 2','Motor 3','Motor 4','Motor 5','Motor 6'};
for mi = 1:6
    subplot(3,2,mi)
    plot(t_vec, outsim.Thrust(mi,:),     'b',  'LineWidth',1.2); hold on
    plot(t_vec, outsim.Thrust_cmd(mi,:), 'r--','LineWidth',1.0)
    yline(Prop.T_max, 'k--', 'LineWidth',0.8)
    xline(fig8.t_start,'k:')
    grid on; ylabel('T [N]'); title(motor_labels{mi})
    if mi >= 5, xlabel('Time [s]'); end
    if mi == 1, legend('Actual','Command','T_{max}','Location','best'); end
end

% ── Figure 8: Virtual Control Cmd vs. Actual ─────────────────────────────
vc_actual = UAM.B * outsim.Thrust;
vc_cmd    = outsim.Ctrl_cmd;
vc_labels = {'Fz [N]','L [Nm]','M [Nm]','N [Nm]'};
vc_titles = {'Fz','L','M','N'};

figure('Name','Virtual Control')
for vi = 1:4
    subplot(2,2,vi)
    plot(t_vec, vc_actual(vi,:), 'b',  'LineWidth',1.2); hold on
    plot(t_vec, vc_cmd(vi,:),    'r--','LineWidth',1.0)
    xline(fig8.t_start,'k:'); grid on
    ylabel(vc_labels{vi}); title(vc_titles{vi})
    if vi >= 3, xlabel('Time [s]'); end
    if vi == 1, legend('Actual','Command','Location','best'); end
end


%% ── Local Function: Equations of Motion ─────────────────────────────────
function X_dot = eom(X, T_vec, UAM, Env)
    u = X(1);  v = X(2);  w = X(3);
    p = X(4);  q = X(5);  r = X(6);
    phi = X(7);  theta = X(8);  psi = X(9);

    % Virtual controls from thrust vector
    vc      = UAM.B * T_vec;
    Fz_prop = vc(1);
    tau     = vc(2:4);   % [L; M; N]

    % Gravity components in body frame (NED, z+ down)
    gx = -Env.g * sin(theta);
    gy =  Env.g * cos(theta) * sin(phi);
    gz =  Env.g * cos(theta) * cos(phi);

    % Translational dynamics (body frame)
    u_dot = r*v - q*w + gx;
    v_dot = p*w - r*u + gy;
    w_dot = q*u - p*v + gz + Fz_prop / UAM.m;

    % Rotational dynamics (Euler's equations)
    p_dot = (tau(1) + (UAM.Iy - UAM.Iz)*q*r) / UAM.Ix;
    q_dot = (tau(2) + (UAM.Iz - UAM.Ix)*p*r) / UAM.Iy;
    r_dot = (tau(3) + (UAM.Ix - UAM.Iy)*p*q) / UAM.Iz;

    % Euler angle kinematics
    phi_dot   = p + (q*sin(phi) + r*cos(phi)) * tan(theta);
    theta_dot = q*cos(phi) - r*sin(phi);
    psi_dot   = (q*sin(phi) + r*cos(phi)) / cos(theta);

    % Rotation matrix: body → NED
    sphi = sin(phi);  cphi = cos(phi);
    sth  = sin(theta); cth = cos(theta);
    spsi = sin(psi);   cpsi = cos(psi);
    R_nb = [cpsi*cth,  cpsi*sth*sphi-spsi*cphi,  cpsi*sth*cphi+spsi*sphi;
            spsi*cth,  spsi*sth*sphi+cpsi*cphi,  spsi*sth*cphi-cpsi*sphi;
           -sth,        cth*sphi,                  cth*cphi              ];

    % Position kinematics (NED)
    pos_dot = R_nb * [u; v; w];

    % vx,vy,vz are updated externally after integration
    X_dot = [u_dot; v_dot; w_dot;
             p_dot; q_dot; r_dot;
             phi_dot; theta_dot; psi_dot;
             pos_dot;
             zeros(3,1)];
end
