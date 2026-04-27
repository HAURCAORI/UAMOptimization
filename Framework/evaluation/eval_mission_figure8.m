function metrics = eval_mission_figure8(d, sim_config)
% EVAL_MISSION_FIGURE8  Mission evaluator using a climb plus figure-8 path.
%
%   This is a framework version of the legacy mission script in Src/.
%   The control structure is intentionally simple and readable:
%   position P -> velocity PI -> attitude command
%   altitude P -> vertical velocity PI -> Fz command
%   attitude P -> rate PI -> virtual control command
%   pseudo-inverse control allocation

sim_config = normalize_sim_config(sim_config);

[UAM, Prop, Env] = hexacopter_params(d);
IDX = state_indices();

mission = default_mission(sim_config);
loe_vec = sim_config.loe_vec(:);
loe_diag = diag(1 - loe_vec);

gains = mission_gains(UAM, d, sim_config);
pos = base_position_gains();
t_fault = sim_config.t_fault;
dt = sim_config.dt;
t_end = mission_end_time(mission, sim_config);
num_step = round(t_end / dt);
t_vec = (1:num_step) * dt;

X = zeros(15, 1);
T = zeros(6, 1);

history = init_history(num_step);
integral = struct('p',0,'q',0,'r',0,'vz',0,'vx',0,'vy',0);

diverged = false;
idx_diverge = num_step;
DIV_ATT_LIM = deg2rad(80);
DIV_ALT_LIM = 500;

for i = 1:num_step
    time = t_vec(i);
    ref = figure8_reference(time, mission);

    if time >= t_fault
        B_eff = UAM.B * loe_diag;
    else
        B_eff = UAM.B;
    end

    x = X(IDX.x); y = X(IDX.y); z = X(IDX.z);
    vx = X(IDX.vx); vy = X(IDX.vy); vz = X(IDX.vz);
    p = X(IDX.p); q = X(IDX.q); r = X(IDX.r);
    phi = X(IDX.phi); theta = X(IDX.theta); psi = X(IDX.psi);

    vx_cmd = clamp(pos.Kp_x * (ref.x - x) + ref.vx, -pos.Vxy_max, pos.Vxy_max);
    vy_cmd = clamp(pos.Kp_y * (ref.y - y) + ref.vy, -pos.Vxy_max, pos.Vxy_max);

    integral.vx = integral.vx + (vx_cmd - vx) * dt;
    integral.vy = integral.vy + (vy_cmd - vy) * dt;
    ax_cmd = pos.Kp_vx * (vx_cmd - vx) + pos.Ki_vx * integral.vx + ref.ax;
    ay_cmd = pos.Kp_vy * (vy_cmd - vy) + pos.Ki_vy * integral.vy + ref.ay;
    ax_cmd = clamp(ax_cmd, -pos.a_max, pos.a_max);
    ay_cmd = clamp(ay_cmd, -pos.a_max, pos.a_max);

    theta_cmd = clamp(-(ax_cmd * cos(psi) + ay_cmd * sin(psi)) / Env.g, ...
        -pos.theta_max, pos.theta_max);
    phi_cmd = clamp((-ax_cmd * sin(psi) + ay_cmd * cos(psi)) / Env.g, ...
        -pos.phi_max, pos.phi_max);
    psi_cmd = 0;

    z_des = -ref.alt;
    vz_ref_ned = -ref.vz_up;
    vz_cmd = clamp((z_des - z) / gains.tau_z + vz_ref_ned, -gains.Vz_max, gains.Vz_max);
    integral.vz = integral.vz + (vz_cmd - vz) * dt;
    tilt = max(cos(phi) * cos(theta), 0.5);
    Fz_cmd = (gains.Kp_vz * (vz_cmd - vz) + gains.Ki_vz * integral.vz - UAM.m * Env.g) / tilt;

    p_cmd = (phi_cmd - phi) / gains.tau_phi;
    q_cmd = (theta_cmd - theta) / gains.tau_theta;
    r_cmd = clamp(wrapToPi(psi_cmd - psi) / gains.tau_psi, -gains.r_max, gains.r_max);

    integral.p = integral.p + (p_cmd - p) * dt;
    integral.q = integral.q + (q_cmd - q) * dt;
    integral.r = integral.r + (r_cmd - r) * dt;
    L_cmd = gains.Kp_p * (p_cmd - p) + gains.Ki_p * integral.p;
    M_cmd = gains.Kp_q * (q_cmd - q) + gains.Ki_q * integral.q;
    N_cmd = gains.Kp_r * (r_cmd - r) + gains.Ki_r * integral.r;

    ctrl_cmd = [Fz_cmd; L_cmd; M_cmd; N_cmd];
    T_cmd = pinv(B_eff) * ctrl_cmd;
    T = T + (1 / Prop.tau) * (T_cmd - T) * dt;
    T = min(Prop.T_max, max(T, Prop.T_min));

    if time >= t_fault
        T_eff = loe_diag * T;
    else
        T_eff = T;
    end

    X_dot = eom_hex(X, T_eff, UAM, Env);
    X = X + X_dot * dt;
    X(IDX.psi) = wrapToPi(X(IDX.psi));
    X(IDX.vx:IDX.vz) = body_to_ned_velocity(X);

    history.X_hist(:, i) = X;
    history.T_hist(:, i) = T_eff;
    history.Ctrl_cmd(:, i) = ctrl_cmd;
    history.Att_cmd(:, i) = [phi_cmd; theta_cmd; psi_cmd];
    history.Rate_cmd(:, i) = [p_cmd; q_cmd; r_cmd];
    history.V_cmd(:, i) = [vx_cmd; vy_cmd; vz_cmd];
    history.pos_ref(:, i) = [ref.x; ref.y; ref.alt];
    history.vel_ref(:, i) = [ref.vx; ref.vy; ref.vz_up];

    if abs(X(IDX.phi)) > DIV_ATT_LIM || abs(X(IDX.theta)) > DIV_ATT_LIM || abs(X(IDX.z)) > DIV_ALT_LIM
        diverged = true;
        idx_diverge = i;
        break;
    end
end

metrics = summarize_mission_metrics(history, t_vec, idx_diverge, diverged, t_fault, mission, Prop, dt);
metrics.gains = gains;
metrics.t_fault = t_fault;
metrics.sim_config = sim_config;
metrics.mission = mission;
end

function mission = default_mission(sim_config)
mission.A = 100;
mission.T_period = 120;
mission.z_cruise = 50;
mission.t_start = 20;
mission.n_laps = 1;
mission.ramp_time = 0;

if isfield(sim_config, 'mission')
    mission_cfg = sim_config.mission;
    fields = fieldnames(mission_cfg);
    for k = 1:numel(fields)
        mission.(fields{k}) = mission_cfg.(fields{k});
    end
end

mission.omega = 2 * pi / mission.T_period;
end

function t_end = mission_end_time(mission, sim_config)
t_end = mission.t_start + mission.n_laps * mission.T_period;
if isfield(sim_config, 't_end')
    t_end = max(t_end, sim_config.t_end);
end
end

function gains = mission_gains(UAM, d, sim_config)
gains_ref = base_rate_altitude_gains();
if ~sim_config.use_scaled_gains
    gains = gains_ref;
    return;
end

d_ref = design_default();
if isfield(d, 'use_vehicle_model')
    d_ref.use_vehicle_model = d.use_vehicle_model;
end
[UAM_ref, ~, ~] = hexacopter_params(d_ref);
gains = tune_sim_gains(UAM, gains_ref, UAM_ref);
end

function gains = base_rate_altitude_gains()
gains.Kp_p = 60000;
gains.Ki_p = 30000;
gains.Kp_q = 50000;
gains.Ki_q = 20000;
gains.Kp_r = 20000;
gains.Ki_r = 1000;
gains.Kp_vz = 2000;
gains.Ki_vz = 600;
gains.tau_phi = 1.0;
gains.tau_theta = 1.0;
gains.tau_psi = 5.0;
gains.r_max = 0.1;
gains.tau_z = 5.0;
gains.Vz_max = 3.0;
end

function pos = base_position_gains()
pos.Kp_x = 0.15;
pos.Kp_y = 0.15;
pos.Kp_vx = 0.5;
pos.Ki_vx = 0.05;
pos.Kp_vy = 0.5;
pos.Ki_vy = 0.05;
pos.Vxy_max = 15.0;
pos.a_max = 3.0;
pos.phi_max = deg2rad(15);
pos.theta_max = deg2rad(15);
end

function IDX = state_indices()
IDX.u=1; IDX.v=2; IDX.w=3;
IDX.p=4; IDX.q=5; IDX.r=6;
IDX.phi=7; IDX.theta=8; IDX.psi=9;
IDX.x=10; IDX.y=11; IDX.z=12;
IDX.vx=13; IDX.vy=14; IDX.vz=15;
end

function history = init_history(num_step)
history.X_hist = zeros(15, num_step);
history.T_hist = zeros(6, num_step);
history.Ctrl_cmd = zeros(4, num_step);
history.Att_cmd = zeros(3, num_step);
history.Rate_cmd = zeros(3, num_step);
history.V_cmd = zeros(3, num_step);
history.pos_ref = zeros(3, num_step);
history.vel_ref = zeros(3, num_step);
end

function v_ned = body_to_ned_velocity(X)
phi = X(7); theta = X(8); psi = X(9);
u = X(1); v = X(2); w = X(3);

sphi = sin(phi); cphi = cos(phi);
sth = sin(theta); cth = cos(theta);
spsi = sin(psi); cpsi = cos(psi);
R_nb = [cpsi*cth,  cpsi*sth*sphi-spsi*cphi,  cpsi*sth*cphi+spsi*sphi; ...
        spsi*cth,  spsi*sth*sphi+cpsi*cphi,  spsi*sth*cphi-cpsi*sphi; ...
       -sth,       cth*sphi,                 cth*cphi];
v_ned = R_nb * [u; v; w];
end

function metrics = summarize_mission_metrics(history, t_vec, idx_diverge, diverged, t_fault, mission, Prop, dt)
IDX = state_indices();
valid_idx = 1:idx_diverge;
track_idx = valid_idx(t_vec(valid_idx) >= mission.t_start);
fault_idx = valid_idx(t_vec(valid_idx) >= max(t_fault, mission.t_start));

alt_h = -history.X_hist(IDX.z, :);
phi_deg = rad2deg(history.X_hist(IDX.phi, :));
theta_deg = rad2deg(history.X_hist(IDX.theta, :));

x_err = history.X_hist(IDX.x, track_idx) - history.pos_ref(1, track_idx);
y_err = history.X_hist(IDX.y, track_idx) - history.pos_ref(2, track_idx);
z_err = alt_h(track_idx) - history.pos_ref(3, track_idx);

if isempty(track_idx)
    metrics.path_rmse = Inf;
    metrics.alt_rmse = Inf;
    metrics.total_rmse = Inf;
else
    metrics.path_rmse = sqrt(mean(x_err.^2 + y_err.^2));
    metrics.alt_rmse = sqrt(mean(z_err.^2));
    metrics.total_rmse = sqrt(mean(x_err.^2 + y_err.^2 + z_err.^2));
end

if isempty(fault_idx)
    fault_idx = valid_idx;
end

metrics.att_rmse_phi = sqrt(mean(phi_deg(fault_idx).^2));
metrics.att_rmse_theta = sqrt(mean(theta_deg(fault_idx).^2));
metrics.max_att_excurs = max(max(abs(phi_deg(fault_idx))), max(abs(theta_deg(fault_idx))));
T_pf = history.T_hist(:, fault_idx);
metrics.ctrl_effort = sqrt(mean(T_pf(:).^2)) / Prop.T_max;
metrics.recovery_time = recovery_time_from_attitude(phi_deg(fault_idx), theta_deg(fault_idx), ...
    t_vec(fault_idx), t_fault, dt);
metrics.diverged = diverged;
metrics.t_vec = t_vec;
metrics.X_hist = history.X_hist;
metrics.T_hist = history.T_hist;
metrics.pos_ref = history.pos_ref;
metrics.vel_ref = history.vel_ref;
metrics.Ctrl_cmd = history.Ctrl_cmd;
metrics.Att_cmd = history.Att_cmd;
metrics.Rate_cmd = history.Rate_cmd;
metrics.V_cmd = history.V_cmd;

if diverged
    metrics.path_rmse = 200;
    metrics.alt_rmse = 200;
    metrics.total_rmse = 200;
    metrics.att_rmse_phi = max(metrics.att_rmse_phi, 180);
    metrics.att_rmse_theta = max(metrics.att_rmse_theta, 180);
    metrics.max_att_excurs = 180;
end
end

function t_recovery = recovery_time_from_attitude(phi_deg, theta_deg, t_local, t_fault, dt)
band_deg = 5;
hold_n = max(1, ceil(0.5 / dt));
in_band = (abs(phi_deg) < band_deg) & (abs(theta_deg) < band_deg);

left_band_idx = find(~in_band, 1, 'first');
if isempty(left_band_idx)
    t_recovery = 0;
    return;
end

recovery_idx = [];
last_start = numel(in_band) - hold_n + 1;
for k = left_band_idx+1:last_start
    if all(in_band(k:k+hold_n-1))
        recovery_idx = k;
        break;
    end
end

if isempty(recovery_idx)
    t_recovery = t_local(end) - t_fault;
else
    t_recovery = t_local(recovery_idx) - t_fault;
end
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end
