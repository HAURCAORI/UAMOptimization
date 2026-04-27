function metrics = eval_simulation(d, sim_config)
% EVAL_SIMULATION  Run a closed-loop simulation and extract performance metrics.
%
%   Executes a fault-injection scenario and returns scalar performance
%   indicators for use in MDO evaluation or standalone analysis.
%
%   SCENARIO
%   --------
%   Phase 1 (t = 0 … t_fault):
%     Nominal hover.  The vehicle starts from equilibrium and tracks a
%     constant altitude command.
%
%   Phase 2 (t = t_fault … t_end):
%     One or more motors suffer Loss-of-Effectiveness (LOE).
%     The vehicle attempts to maintain hover using the remaining actuators.
%
%   CONTROLLER
%   ----------
%   Cascade PI rate/attitude + PI altitude:
%     Altitude outer P → Vz PI → Fz command
%     Attitude outer P → rate PI → [L, M, N] command
%     Control allocation: pseudo-inverse of B_eff (Moore-Penrose)
%
%   METRICS RETURNED
%   ----------------
%   alt_rmse       - altitude RMSE over post-fault window [m]
%   att_rmse_phi   - roll RMSE over post-fault window [deg]
%   att_rmse_theta - pitch RMSE over post-fault window [deg]
%   ctrl_effort    - normalized RMS actuator effort (post-fault)
%   max_att_excurs - maximum attitude excursion post-fault [deg]
%   diverged       - logical: did the vehicle crash/diverge?
%   recovery_time  - time to re-enter ±5° attitude window after fault [s]
%   t_vec          - time vector [1×N]
%   X_hist         - state history [15×N]
%
%   Inputs:
%     d          - Design struct (see design_default.m)
%     sim_config - (optional) struct with fields:
%                    .loe_vec    : [6×1] LOE vector (default: motor 1 fails)
%                    .t_fault    : fault injection time [s]  (default: 10)
%                    .t_end      : simulation end time [s]   (default: 40)
%                    .dt         : time step [s]             (default: 0.01)
%                    .alt_cmd    : altitude command [m]      (default: 10)

% ── Defaults ──────────────────────────────────────────────────────────────
if nargin < 2, sim_config = struct(); end
sim_config = normalize_sim_config(sim_config);

% ── Physical model ────────────────────────────────────────────────────────
[UAM, Prop, Env] = hexacopter_params(d);
B     = UAM.B;
m     = UAM.m;
g     = Env.g;

loe_vec  = sim_config.loe_vec(:);
LOE_diag = diag(1 - loe_vec);

dt       = sim_config.dt;
t_fault  = sim_config.t_fault;
t_end    = sim_config.t_end;
alt_cmd  = sim_config.alt_cmd;

num_step = round(t_end / dt);
t_vec    = (1:num_step) * dt;

% ── Controller gains (cascade PI, tuned for UAM mass scale) ───────────────
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

% ── Initial state: settled hover equilibrium ──────────────────────────────
X = zeros(15,1);
X(IDX.z) = -alt_cmd;        % NED: altitude = -z_NED

% Initial thrust: distribute weight evenly (no fault yet)
T  = ones(6,1) * (m * g / 6);   % approximate hover thrust per motor
B_nom = B;   % nominal effectiveness for pre-fault phase

% History buffers
X_hist   = zeros(15, num_step);
T_hist   = zeros(6,  num_step);

% ── Divergence guard ──────────────────────────────────────────────────────
DIV_ATT_LIM = deg2rad(80);   % crash if |phi| or |theta| > 80 deg
DIV_ALT_LIM = 500;           % crash if |z| > 500 m
diverged = false;
idx_diverge = num_step;

% ── Main simulation loop ──────────────────────────────────────────────────
for i = 1:num_step
    time = t_vec(i);

    % Switch effectiveness matrix at fault injection time
    if time >= t_fault
        B_eff = B * LOE_diag;
    else
        B_eff = B_nom;
    end

    % Unpack state
    p   = X(IDX.p);   q = X(IDX.q);   r = X(IDX.r);
    phi = X(IDX.phi); theta = X(IDX.theta); psi = X(IDX.psi);
    z   = X(IDX.z);   vz = X(IDX.vz);

    % ── Attitude controller (P outer → rate command) ──────────────────────
    phi_cmd   = 0;  theta_cmd = 0;  psi_cmd = 0;
    p_cmd     = (phi_cmd   - phi)   / tau_phi;
    q_cmd     = (theta_cmd - theta) / tau_theta;
    r_cmd     = max(min(wrapToPi(psi_cmd - psi) / tau_psi, r_max), -r_max);

    % ── Rate controller (PI inner) ────────────────────────────────────────
    i_p = i_p + (p_cmd - p) * dt;
    i_q = i_q + (q_cmd - q) * dt;
    i_r = i_r + (r_cmd - r) * dt;
    L_cmd = Kp_p*(p_cmd - p) + Ki_p*i_p;
    M_cmd = Kp_q*(q_cmd - q) + Ki_q*i_q;
    N_cmd = Kp_r*(r_cmd - r) + Ki_r*i_r;

    % ── Altitude controller (P outer + PI inner) ──────────────────────────
    z_des   = -alt_cmd;          % NED: desired z_NED
    vz_cmd  = max(min((z_des - z) / tau_z, Vz_max), -Vz_max);
    i_vz    = i_vz + (vz_cmd - vz) * dt;
    tilt    = max(cos(phi)*cos(theta), 0.5);
    Fz_cmd  = (Kp_vz*(vz_cmd - vz) + Ki_vz*i_vz - m*g) / tilt;

    % ── Control allocation (pseudo-inverse of faulted B) ──────────────────
    v_cmd = [Fz_cmd; L_cmd; M_cmd; N_cmd];
    T_cmd = pinv(B_eff) * v_cmd;

    % ── Motor dynamics (1st-order lag) ────────────────────────────────────
    T = T + (1/Prop.tau) * (T_cmd - T) * dt;
    T = min(Prop.T_max, max(T, Prop.T_min));

    % ── Apply LOE (physical fault) ────────────────────────────────────────
    if time >= t_fault
        T_eff = LOE_diag * T;
    else
        T_eff = T;
    end

    % ── Equations of motion (Euler integration) ───────────────────────────
    X_dot = eom_hex(X, T_eff, UAM, Env);
    X     = X + X_dot * dt;
    X(IDX.psi) = wrapToPi(X(IDX.psi));

    % ── Recompute NED velocity from body velocity ─────────────────────────
    phi_n = X(IDX.phi); theta_n = X(IDX.theta); psi_n = X(IDX.psi);
    sphi = sin(phi_n);  cphi = cos(phi_n);
    sth  = sin(theta_n); cth  = cos(theta_n);
    spsi = sin(psi_n);   cpsi = cos(psi_n);
    R_nb = [cpsi*cth,  cpsi*sth*sphi-spsi*cphi,  cpsi*sth*cphi+spsi*sphi;
            spsi*cth,  spsi*sth*sphi+cpsi*cphi,  spsi*sth*cphi-cpsi*sphi;
           -sth,        cth*sphi,                  cth*cphi              ];
    X(IDX.vx:IDX.vz) = R_nb * X(IDX.u:IDX.w);

    % ── Record ───────────────────────────────────────────────────────────
    X_hist(:,i) = X;
    T_hist(:,i) = T_eff;

    % ── Divergence check ─────────────────────────────────────────────────
    if abs(X(IDX.phi)) > DIV_ATT_LIM || abs(X(IDX.theta)) > DIV_ATT_LIM ...
            || abs(X(IDX.z)) > DIV_ALT_LIM
        diverged    = true;
        idx_diverge = i;
        break
    end
end

% ── Performance metrics (post-fault window) ───────────────────────────────
idx_pf = (t_vec > t_fault) & (t_vec <= t_vec(idx_diverge));

if sum(idx_pf) > 5
    alt_actual  = -X_hist(IDX.z, idx_pf);          % positive-up altitude
    phi_deg     = rad2deg(X_hist(IDX.phi,   idx_pf));
    theta_deg   = rad2deg(X_hist(IDX.theta, idx_pf));
    T_pf        = T_hist(:, idx_pf);

    alt_rmse       = sqrt(mean((alt_actual - alt_cmd).^2));
    att_rmse_phi   = sqrt(mean(phi_deg.^2));
    att_rmse_theta = sqrt(mean(theta_deg.^2));
    max_att_excurs = max(max(abs(phi_deg)), max(abs(theta_deg)));

    % Normalized control effort: RMS thrust / T_max
    ctrl_effort = sqrt(mean(T_pf(:).^2)) / Prop.T_max;

    % Recovery time: first time after fault that attitude stays within ±5 deg
    ATT_BAND = 5;      % [deg]
    HOLD_SEC = 0.5;    % [s]
    hold_n   = max(1, ceil(HOLD_SEC / dt));
    in_band  = (abs(phi_deg) < ATT_BAND) & (abs(theta_deg) < ATT_BAND);
    t_pf     = t_vec(idx_pf);
    left_band_idx = find(~in_band, 1, 'first');
    if isempty(left_band_idx)
        recovery_time = 0;
    else
        recovery_idx = [];
        last_start = numel(in_band) - hold_n + 1;
        for kk = left_band_idx+1:last_start
            if all(in_band(kk:kk+hold_n-1))
                recovery_idx = kk;
                break;
            end
        end
        if isempty(recovery_idx)
            recovery_time = t_end - t_fault;
        else
            recovery_time = t_pf(recovery_idx) - t_fault;
        end
    end
else
    % Immediate divergence
    alt_rmse       = Inf;
    att_rmse_phi   = Inf;
    att_rmse_theta = Inf;
    max_att_excurs = Inf;
    ctrl_effort    = Inf;
    recovery_time  = t_end - t_fault;
end

% If diverged, penalize strongly
if diverged
    alt_rmse       = 200;   % large penalty [m]
    att_rmse_phi   = max(att_rmse_phi,   180);
    att_rmse_theta = max(att_rmse_theta, 180);
    max_att_excurs = 180;
    ctrl_effort    = min(ctrl_effort, 2.0);
end

% ── Pack output ───────────────────────────────────────────────────────────
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
metrics.sim_config     = sim_config;
metrics.gains          = gains;
end
