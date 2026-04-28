function metrics = eval_simulation(d, sim_config)
% EVAL_SIMULATION  Run a closed-loop hover simulation and extract metrics.
%
%   The simulator supports both nominal and faulted hover using the same
%   code path.  A true no-fault case is obtained by setting loe_vec = 0,
%   not by moving the fault time outside the simulation window.

if nargin < 2
    sim_config = struct();
end
sim_config = normalize_sim_config(sim_config);

[UAM, Prop, Env] = hexacopter_params(d);
B     = UAM.B;
m     = UAM.m;
g     = Env.g;

loe_vec  = sim_config.loe_vec(:);
has_fault = any(abs(loe_vec) > 1e-12);
LOE_diag = diag(1 - loe_vec);

dt       = sim_config.dt;
t_fault  = sim_config.t_fault;
t_end    = sim_config.t_end;
alt_cmd  = sim_config.alt_cmd;

num_step = round(t_end / dt);
t_vec    = (1:num_step) * dt;

% Controller gains (cascade PI, tuned for UAM mass scale)
gains_ref.Kp_p  = 60000;  gains_ref.Kp_q  = 50000;  gains_ref.Kp_r  = 20000;
gains_ref.Ki_p  = 30000;  gains_ref.Ki_q  = 20000;  gains_ref.Ki_r  = 1000;
gains_ref.Kp_vz = 2000;   gains_ref.Ki_vz = 600;
tau_phi = 1.0;  tau_theta = 1.0; tau_psi = 5.0;
r_max = 0.1;
tau_z = 5.0;  Vz_max = 3.0;

if sim_config.use_scaled_gains
    d_ref = design_default();
    if isfield(d, 'use_vehicle_model')
        d_ref.use_vehicle_model = d.use_vehicle_model;
    end
    [UAM_ref, ~, ~] = hexacopter_params(d_ref);
    gains = tune_sim_gains(UAM, gains_ref, UAM_ref);
else
    gains = gains_ref;
end

Kp_p = gains.Kp_p;   Kp_q = gains.Kp_q;   Kp_r = gains.Kp_r;
Ki_p = gains.Ki_p;   Ki_q = gains.Ki_q;   Ki_r = gains.Ki_r;
Kp_vz = gains.Kp_vz; Ki_vz = gains.Ki_vz;

% Integral states
i_p = 0;  i_q = 0;  i_r = 0;  i_vz = 0;

% State index constants
IDX.u=1; IDX.v=2; IDX.w=3; IDX.p=4; IDX.q=5; IDX.r=6;
IDX.phi=7; IDX.theta=8; IDX.psi=9;
IDX.x=10; IDX.y=11; IDX.z=12; IDX.vx=13; IDX.vy=14; IDX.vz=15;

% Initial state: settled hover equilibrium
X = zeros(15,1);
X(IDX.z) = -alt_cmd;

T  = ones(6,1) * (m * g / 6);
B_nom = B;

X_hist = zeros(15, num_step);
T_hist = zeros(6,  num_step);

DIV_ATT_LIM = deg2rad(80);
DIV_ALT_LIM = 500;
diverged = false;
idx_diverge = num_step;

for i = 1:num_step
    time = t_vec(i);

    if has_fault && (time >= t_fault)
        B_eff = B * LOE_diag;
    else
        B_eff = B_nom;
    end

    p   = X(IDX.p);   q = X(IDX.q);   r = X(IDX.r);
    phi = X(IDX.phi); theta = X(IDX.theta); psi = X(IDX.psi);
    z   = X(IDX.z);   vz = X(IDX.vz);

    phi_cmd   = 0;  theta_cmd = 0;  psi_cmd = 0;
    p_cmd     = (phi_cmd   - phi)   / tau_phi;
    q_cmd     = (theta_cmd - theta) / tau_theta;
    r_cmd     = max(min(wrapToPi(psi_cmd - psi) / tau_psi, r_max), -r_max);

    i_p = i_p + (p_cmd - p) * dt;
    i_q = i_q + (q_cmd - q) * dt;
    i_r = i_r + (r_cmd - r) * dt;
    L_cmd = Kp_p*(p_cmd - p) + Ki_p*i_p;
    M_cmd = Kp_q*(q_cmd - q) + Ki_q*i_q;
    N_cmd = Kp_r*(r_cmd - r) + Ki_r*i_r;

    z_des   = -alt_cmd;
    vz_cmd  = max(min((z_des - z) / tau_z, Vz_max), -Vz_max);
    i_vz    = i_vz + (vz_cmd - vz) * dt;
    tilt    = max(cos(phi)*cos(theta), 0.5);
    Fz_cmd  = (Kp_vz*(vz_cmd - vz) + Ki_vz*i_vz - m*g) / tilt;

    v_cmd = [Fz_cmd; L_cmd; M_cmd; N_cmd];
    T_cmd = pinv(B_eff) * v_cmd;

    T = T + (1/Prop.tau) * (T_cmd - T) * dt;
    T = min(Prop.T_max, max(T, Prop.T_min));

    if has_fault && (time >= t_fault)
        T_eff = LOE_diag * T;
    else
        T_eff = T;
    end

    X_dot = eom_hex(X, T_eff, UAM, Env);
    X     = X + X_dot * dt;
    X(IDX.psi) = wrapToPi(X(IDX.psi));

    phi_n = X(IDX.phi); theta_n = X(IDX.theta); psi_n = X(IDX.psi);
    sphi = sin(phi_n);  cphi = cos(phi_n);
    sth  = sin(theta_n); cth  = cos(theta_n);
    spsi = sin(psi_n);   cpsi = cos(psi_n);
    R_nb = [cpsi*cth,  cpsi*sth*sphi-spsi*cphi,  cpsi*sth*cphi+spsi*sphi; ...
            spsi*cth,  spsi*sth*sphi+cpsi*cphi,  spsi*sth*cphi-cpsi*sphi; ...
           -sth,       cth*sphi,                 cth*cphi];
    X(IDX.vx:IDX.vz) = R_nb * X(IDX.u:IDX.w);

    X_hist(:,i) = X;
    T_hist(:,i) = T_eff;

    if abs(X(IDX.phi)) > DIV_ATT_LIM || abs(X(IDX.theta)) > DIV_ATT_LIM || abs(X(IDX.z)) > DIV_ALT_LIM
        diverged = true;
        idx_diverge = i;
        break
    end
end

% Evaluation window
if has_fault
    idx_eval = (t_vec > t_fault) & (t_vec <= t_vec(idx_diverge));
else
    t_eval_start = min(sim_config.nominal_eval_start, max(0.0, t_vec(idx_diverge) - dt));
    idx_eval = (t_vec > t_eval_start) & (t_vec <= t_vec(idx_diverge));
end

if sum(idx_eval) > 5
    alt_actual  = -X_hist(IDX.z, idx_eval);
    phi_deg     = rad2deg(X_hist(IDX.phi,   idx_eval));
    theta_deg   = rad2deg(X_hist(IDX.theta, idx_eval));
    T_eval      = T_hist(:, idx_eval);

    alt_rmse       = sqrt(mean((alt_actual - alt_cmd).^2));
    att_rmse_phi   = sqrt(mean(phi_deg.^2));
    att_rmse_theta = sqrt(mean(theta_deg.^2));
    max_att_excurs = max(max(abs(phi_deg)), max(abs(theta_deg)));
    ctrl_effort    = sqrt(mean(T_eval(:).^2)) / max(Prop.T_max, 1e-9);

    if has_fault
        recovery_time = compute_recovery_time(phi_deg, theta_deg, t_vec(idx_eval), t_fault, dt, t_end);
    else
        recovery_time = NaN;
    end
else
    alt_rmse       = Inf;
    att_rmse_phi   = Inf;
    att_rmse_theta = Inf;
    max_att_excurs = Inf;
    ctrl_effort    = Inf;
    if has_fault
        recovery_time = max(t_end - t_fault, 0);
    else
        recovery_time = NaN;
    end
end

if diverged
    alt_rmse       = 200;
    att_rmse_phi   = max(att_rmse_phi,   180);
    att_rmse_theta = max(att_rmse_theta, 180);
    max_att_excurs = 180;
    ctrl_effort    = min(ctrl_effort, 2.0);
    if has_fault && ~isfinite(recovery_time)
        recovery_time = max(t_end - t_fault, 0);
    end
end

metrics.alt_rmse       = alt_rmse;
metrics.att_rmse_phi   = att_rmse_phi;
metrics.att_rmse_theta = att_rmse_theta;
metrics.max_att_excurs = max_att_excurs;
metrics.ctrl_effort    = ctrl_effort;
metrics.recovery_time  = recovery_time;
metrics.diverged       = diverged;
metrics.t_vec          = t_vec;
metrics.X_hist         = X_hist;
metrics.T_hist         = T_hist;
metrics.t_fault        = t_fault;
metrics.has_fault      = has_fault;
metrics.eval_start_time = has_fault * t_fault + (~has_fault) * sim_config.nominal_eval_start;
metrics.sim_config     = sim_config;
metrics.gains          = gains;
end

function recovery_time = compute_recovery_time(phi_deg, theta_deg, t_local, t_fault, dt, t_end)
ATT_BAND = 5;
HOLD_SEC = 0.5;
hold_n   = max(1, ceil(HOLD_SEC / dt));
in_band  = (abs(phi_deg) < ATT_BAND) & (abs(theta_deg) < ATT_BAND);
left_band_idx = find(~in_band, 1, 'first');

if isempty(left_band_idx)
    recovery_time = 0;
    return;
end

recovery_idx = [];
last_start = numel(in_band) - hold_n + 1;
for kk = left_band_idx+1:last_start
    if all(in_band(kk:kk+hold_n-1))
        recovery_idx = kk;
        break;
    end
end

if isempty(recovery_idx)
    recovery_time = max(t_end - t_fault, 0);
else
    recovery_time = max(t_local(recovery_idx) - t_fault, 0);
end
end
