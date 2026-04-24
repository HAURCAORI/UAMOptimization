% 2026 Spring AE50001 / Team5
% Main simulation file

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
Prop.cT = 0.03;                         % Moment to thrust ratio
Prop.tau = 0.04;                        % Time constant [s]

% Control Effectiveness Matrix (PPNNPN Configuration)
UAM.B = [    -1       -1       -1       -1       -1       -1;
        -UAM.Lyi  UAM.Lyi -UAM.Lyo  UAM.Lyo -UAM.Lyi  UAM.Lyi;
         UAM.Lx   UAM.Lx       0        0   -UAM.Lx  -UAM.Lx;
        -Prop.cT -Prop.cT  Prop.cT  Prop.cT -Prop.cT  Prop.cT];

% Loss of Effectivenss
LOE = [0 0 0 0 0 0];

%% Command Sequence
CMD1 = [10  deg2rad(  0)  deg2rad(  0)  deg2rad(  0)];  % t  <  10 s
CMD2 = [10  deg2rad(-10)  deg2rad(  0)  deg2rad(  0)];  % 10 ≤ t < 15 s
CMD3 = [10  deg2rad( 10)  deg2rad(  0)  deg2rad(  0)];  % 15 ≤ t < 20 s
CMD4 = [10  deg2rad(  0)  deg2rad(-10)  deg2rad(  0)];  % 20 ≤ t < 25 s
CMD5 = [10  deg2rad(  0)  deg2rad( 10)  deg2rad(  0)];  % 25 ≤ t < 30 s
CMD6 = [10  deg2rad(  0)  deg2rad(  0)  deg2rad(  0)];  % 30 ≤ t < 35 s
CMD7 = [10  deg2rad(  0)  deg2rad(  0)  deg2rad( -5)];  % 35 ≤ t < 40 s
CMD8 = [ 0  deg2rad(  0)  deg2rad(  0)  deg2rad( -5)];  % 40 ≤ t < 45 s
CMD9 = [ 0  deg2rad(  0)  deg2rad(  0)  deg2rad(  0)];  % 45 ≤ t < 50 s

%% Simulation Setting
% Simulation Config.
CONFIG.act_saturation = 1; % Activate Thrust Saturation

% Simulation Time
tf = 60;
dt = 0.002;
num_step = tf / dt;
t_vec    = (1:num_step) * dt;

% Initial States
X0.uvw = [0; 0; 0];
X0.pqr = [0; 0; 0];
X0.att = [0; 0; 0];
X0.pos = [0; 0; 0];
X0.vel = [0; 0; 0];

X = [X0.uvw; X0.pqr; X0.att; X0.pos; X0.vel];  % Initial states
T = zeros(6,1);                                % Initial thrust

% Index
IDX.u = 1; IDX.v = 2; IDX.w = 3;
IDX.p = 4; IDX.q = 5; IDX.r = 6;
IDX.phi = 7; IDX.theta = 8; IDX.psi = 9;
IDX.x = 10; IDX.y = 11; IDX.z = 12;
IDX.vx = 13; IDX.vy = 14; IDX.vz = 15;

% History record
outsim.X = zeros(15, num_step);
outsim.X_dot = zeros(15, num_step);
outsim.Ctrl_cmd = zeros(4, num_step);
outsim.Rate_cmd = zeros(3, num_step);
outsim.V_cmd = zeros(3, num_step);
outsim.Thrust = zeros(6, num_step);
outsim.Thrust_cmd = zeros(6, num_step);


%% Control Gain
Kp_p = 60000;   Kp_q = 50000;   Kp_r = 20000;
Ki_p = 30000;   Ki_q = 20000;   Ki_r = 1000;

tau_phi = 1.0;
tau_theta = 1.0;
tau_psi = 5.0;
r_max = 0.1;

Kp_vz = 2000;       Ki_vz = 600;
tau_z = 5.0;
Vz_max = 3.0;

% I Term Initialization
i_p = 0;    i_q = 0;    i_r = 0;
i_vz = 0;


%% Main Loop
for i = 1:num_step
    % State
    time = t_vec(i);
    u = X(IDX.u); v = X(IDX.v); w = X(IDX.w);
    p = X(IDX.p); q = X(IDX.q); r = X(IDX.r);
    phi = X(IDX.phi); theta = X(IDX.theta); psi = X(IDX.psi);
    x = X(IDX.x); y = X(IDX.y); z = X(IDX.z);
    vx = X(IDX.vx); vy = X(IDX.vy); vz = X(IDX.vz);

    % Command
    if     time < 10, CMD = CMD1;
    elseif time < 15, CMD = CMD2;
    elseif time < 20, CMD = CMD3;
    elseif time < 25, CMD = CMD4;
    elseif time < 30, CMD = CMD5;
    elseif time < 35, CMD = CMD6;
    elseif time < 40, CMD = CMD7;
    elseif time < 45, CMD = CMD8;
    else            , CMD = CMD9;
    end
    alt_cmd   = CMD(1);
    phi_cmd   = CMD(2);
    theta_cmd = CMD(3);
    psi_cmd   = CMD(4);

    % Attitude Controller
    phi_err = phi_cmd - phi;
    theta_err = theta_cmd - theta;
    psi_err   = wrapToPi(psi_cmd - psi);

    p_cmd = phi_err / tau_phi;
    q_cmd = theta_err / tau_theta;
    r_cmd = max(min(psi_err / tau_psi, r_max), -r_max);

    % Rate Controller
    p_err = p_cmd - p;
    q_err = q_cmd - q;
    r_err = r_cmd - r;

    i_p = i_p + p_err * dt;
    i_q = i_q + q_err * dt;
    i_r = i_r + r_err * dt;

    L_cmd = Kp_p * p_err + Ki_p * i_p;
    M_cmd = Kp_q * q_err + Ki_q * i_q;
    N_cmd = Kp_r * r_err + Ki_r * i_r;

    % Altitude Controller
    z_err  = -alt_cmd - z;
    vz_cmd = max(min(z_err / tau_z, Vz_max), -Vz_max);

    % Vz Controller
    vz_err = vz_cmd - vz;
    i_vz = i_vz + vz_err * dt;
    tilt  = max(cos(phi) * cos(theta), 0.5);
    Fz_cmd = (Kp_vz * vz_err + Ki_vz * i_vz - UAM.m * Env.g) / tilt;

    % Control Allocation
    virtual_ctrl_cmd = [Fz_cmd; L_cmd; M_cmd; N_cmd];
    T_cmd = pinv(UAM.B) * virtual_ctrl_cmd;

    % Motor Dynamics (1st-order)
    T_dot = (1/Prop.tau)*(T_cmd - T);
    T = T + T_dot * dt;
    if (CONFIG.act_saturation)
        T = min(Prop.T_max, max(T, Prop.T_min));
    end

    % Equations of Motion & Euler Integration
    X_dot = eom(X, T, UAM, Env);
    X     = X + X_dot * dt;
    X(IDX.psi) = wrapToPi(X(IDX.psi));

    % Inertial velocity
    phi_n = X(IDX.phi); theta_n = X(IDX.theta); psi_n = X(IDX.psi);
    sphi = sin(phi_n); cphi = cos(phi_n);
    sth  = sin(theta_n); cth = cos(theta_n);
    spsi = sin(psi_n);   cpsi = cos(psi_n);
    R_nb = [cpsi*cth,  cpsi*sth*sphi-spsi*cphi,  cpsi*sth*cphi+spsi*sphi;
            spsi*cth,  spsi*sth*sphi+cpsi*cphi,  spsi*sth*cphi-cpsi*sphi;
           -sth,       cth*sphi,                  cth*cphi              ];
    X(IDX.vx:IDX.vz) = R_nb * X(IDX.u:IDX.w);

    % Record history
    outsim.X(:, i)          = X;
    outsim.X_dot(:, i)      = X_dot;
    outsim.Ctrl_cmd(:, i)   = virtual_ctrl_cmd;
    outsim.Rate_cmd(:, i)   = [p_cmd; q_cmd; r_cmd];
    outsim.V_cmd(:, i)      = [0; 0; vz_cmd];
    outsim.Thrust(:, i)     = T;
    outsim.Thrust_cmd(:, i) = T_cmd;
end


%% Plot Results

% Unpack history
pos_x   = outsim.X(IDX.x,   :);
pos_y   = outsim.X(IDX.y,   :);
pos_z   = outsim.X(IDX.z,   :);
vel_x   = outsim.X(IDX.vx,  :);
vel_y   = outsim.X(IDX.vy,  :);
vel_z   = outsim.X(IDX.vz,  :);
phi_h   = rad2deg(outsim.X(IDX.phi,   :));
theta_h = rad2deg(outsim.X(IDX.theta, :));
psi_h   = rad2deg(outsim.X(IDX.psi,   :));
p_h     = rad2deg(outsim.X(IDX.p, :));
q_h     = rad2deg(outsim.X(IDX.q, :));
r_h     = rad2deg(outsim.X(IDX.r, :));

p_cmd_h = rad2deg(outsim.Rate_cmd(1,:));
q_cmd_h = rad2deg(outsim.Rate_cmd(2,:));
r_cmd_h = rad2deg(outsim.Rate_cmd(3,:));

alt_h = -pos_z;   % altitude (positive up)

% Reconstruct command history
alt_cmd_h = zeros(1, num_step);
att_cmd_h = zeros(3, num_step);
for k = 1:num_step
    if     t_vec(k) < 10,  c = CMD1;
    elseif t_vec(k) < 15,  c = CMD2;
    elseif t_vec(k) < 20,  c = CMD3;
    elseif t_vec(k) < 25,  c = CMD4;
    elseif t_vec(k) < 30,  c = CMD5;
    elseif t_vec(k) < 35,  c = CMD6;
    elseif t_vec(k) < 40,  c = CMD7;
    elseif t_vec(k) < 45,  c = CMD8;
    else,                   c = CMD9;
    end
    alt_cmd_h(k)    = c(1);
    att_cmd_h(:, k) = rad2deg([c(2); c(3); c(4)]);
end

% Figure 1: 3D Position Trajectory
figure('Name','3D Trajectory')
plot3(pos_x, pos_y, -pos_z, 'b-', 'LineWidth', 1.5)
hold on
plot3(pos_x(1), pos_y(1), -pos_z(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g')
plot3(pos_x(end), pos_y(end), -pos_z(end), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r')
grid on
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Altitude [m]')
title('3D Position Trajectory')
legend('Trajectory', 'Start', 'End', 'Location', 'best')
view(45, 30)

% Figure 2: XYZ Position Subplots
figure('Name','Position')
subplot(3,1,1)
plot(t_vec, pos_x, 'b', 'LineWidth', 1.2)
grid on; ylabel('X [m]'); title('Position')

subplot(3,1,2)
plot(t_vec, pos_y, 'b', 'LineWidth', 1.2)
grid on; ylabel('Y [m]')

subplot(3,1,3)
plot(t_vec, alt_h,     'b',  'LineWidth', 1.2); hold on
plot(t_vec, alt_cmd_h, 'r--','LineWidth', 1.0)
grid on; ylabel('Altitude [m]'); xlabel('Time [s]')
legend('Actual','Command','Location','best')

% Figure 3: Velocity Trajectory
figure('Name','Velocity')
subplot(3,1,1)
plot(t_vec, vel_x, 'b', 'LineWidth', 1.2)
grid on; ylabel('Vx [m/s]'); title('Inertial Velocity')

subplot(3,1,2)
plot(t_vec, vel_y, 'b', 'LineWidth', 1.2)
grid on; ylabel('Vy [m/s]')

subplot(3,1,3)
plot(t_vec, vel_z,              'b',  'LineWidth', 1.2); hold on
plot(t_vec, outsim.V_cmd(3,:), 'r--','LineWidth', 1.0)
grid on; ylabel('Vz [m/s]'); xlabel('Time [s]')
legend('Actual','Command','Location','best')

% Figure 4: Attitude

figure('Name','Attitude')
subplot(3,1,1)
plot(t_vec, phi_h,           'b',  'LineWidth', 1.2); hold on
plot(t_vec, att_cmd_h(1,:),  'r--','LineWidth', 1.0)
grid on; ylabel('\phi [deg]'); title('Attitude')
legend('Actual','Command','Location','best')

subplot(3,1,2)
plot(t_vec, theta_h,         'b',  'LineWidth', 1.2); hold on
plot(t_vec, att_cmd_h(2,:),  'r--','LineWidth', 1.0)
grid on; ylabel('\theta [deg]')
legend('Actual','Command','Location','best')

subplot(3,1,3)
plot(t_vec, psi_h,           'b',  'LineWidth', 1.2); hold on
plot(t_vec, att_cmd_h(3,:),  'r--','LineWidth', 1.0)
grid on; ylabel('\psi [deg]'); xlabel('Time [s]')
legend('Actual','Command','Location','best')

% Figure 5: Angular Rate
figure('Name','Angular Rate')
subplot(3,1,1)
plot(t_vec, p_h,     'b',  'LineWidth', 1.2); hold on
plot(t_vec, p_cmd_h, 'r--','LineWidth', 1.0)
grid on; ylabel('p [deg/s]'); title('Angular Rate')
legend('Actual','Command','Location','best')

subplot(3,1,2)
plot(t_vec, q_h,     'b',  'LineWidth', 1.2); hold on
plot(t_vec, q_cmd_h, 'r--','LineWidth', 1.0)
grid on; ylabel('q [deg/s]')
legend('Actual','Command','Location','best')

subplot(3,1,3)
plot(t_vec, r_h,     'b',  'LineWidth', 1.2); hold on
plot(t_vec, r_cmd_h, 'r--','LineWidth', 1.0)
grid on; ylabel('r [deg/s]'); xlabel('Time [s]')
legend('Actual','Command','Location','best')

% Figure 6: Thrust 
figure('Name','Thrust')
motor_labels = {'Motor 1','Motor 2','Motor 3','Motor 4','Motor 5','Motor 6'};
for mi = 1:6
    subplot(3, 2, mi)
    plot(t_vec, outsim.Thrust(mi,:),     'b',  'LineWidth', 1.2); hold on
    plot(t_vec, outsim.Thrust_cmd(mi,:), 'r--','LineWidth', 1.0)
    yline(Prop.T_max, 'k--', 'LineWidth', 0.8)
    grid on
    ylabel('T [N]')
    title(motor_labels{mi})
    if mi >= 5, xlabel('Time [s]'); end
    if mi == 1
        legend('Actual','Command','T_{max}','Location','best')
    end
end



%% Local Functions
function X_dot = eom(X, T_vec, UAM, Env)
    u = X(1); v = X(2); w = X(3);
    p = X(4); q = X(5); r = X(6);
    phi = X(7); theta = X(8); psi = X(9);

    % Virtual controls from thrust
    vc      = UAM.B * T_vec;
    Fz_prop = vc(1);
    tau     = vc(2:4);   % [L; M; N]

    % Gravity in body frame (NED, z positive down)
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
    sphi = sin(phi); cphi = cos(phi);
    sth  = sin(theta); cth = cos(theta);
    spsi = sin(psi);   cpsi = cos(psi);
    R_nb = [cpsi*cth,  cpsi*sth*sphi-spsi*cphi,  cpsi*sth*cphi+spsi*sphi;
            spsi*cth,  spsi*sth*sphi+cpsi*cphi,  spsi*sth*cphi-cpsi*sphi;
           -sth,       cth*sphi,                  cth*cphi              ];

    % Position kinematics (NED)
    pos_dot = R_nb * [u; v; w];

    % vx,vy,vz updated externally after each step to stay consistent
    X_dot = [u_dot; v_dot; w_dot;
             p_dot; q_dot; r_dot;
             phi_dot; theta_dot; psi_dot;
             pos_dot;
             zeros(3,1)];
end
