function result = eval_stage2(d, cfg_or_sim)
% EVAL_STAGE2  Stage 2 evaluator: closed-loop mission performance metrics.
%
%   Stage 2 is called only for designs that passed Stage 1 feasibility
%   screening.  It runs the closed-loop fault-injection simulation and
%   returns mission-level performance metrics suitable as optimization
%   objectives or validation figures.
%
%   Stage 2 metrics:
%     - Altitude tracking RMSE after fault injection
%     - Maximum attitude excursion (roll + pitch) during mission
%     - Recovery time (time to re-enter ±5° attitude band)
%     - Normalized total thrust effort
%     - Divergence flag (true if attitude exceeds hard limits)
%
%   Inputs:
%     d           - Design struct (see design_default.m)
%     cfg_or_sim  - (optional) mdo_config struct OR sim_config struct
%                   If mdo_config, reads cfg.sim fields automatically.
%
%   Output:
%     result — struct with fields:
%       .feasible        - true if simulation did not diverge
%       .sim             - full simulation output struct (from eval_simulation)
%       .J_mission       - normalized altitude tracking RMSE ∈ [0,∞)
%       .alt_rmse        - altitude RMSE [m]
%       .max_att_excurs  - max roll+pitch excursion [deg]
%       .recovery_time   - time to re-enter ±5° band after fault [s]; Inf if none
%       .diverged        - true if attitude exceeded 45° at any point

if nargin < 2, cfg_or_sim = struct(); end

% ── Resolve sim config ────────────────────────────────────────────────────
if isfield(cfg_or_sim, 'vars')
    % mdo_config struct — map cfg.sim fields to eval_simulation convention
    sim_config.loe_vec  = cfg_or_sim.sim.loe_vec;
    sim_config.t_end    = cfg_or_sim.sim.T_end;
    sim_config.t_fault  = cfg_or_sim.sim.fault_time;   % mdo_config → eval_simulation
    sim_config.dt       = cfg_or_sim.sim.dt;
    sim_config.alt_cmd  = cfg_or_sim.sim.alt_cmd;
else
    % Raw sim_config struct — accept both naming conventions
    sim_config = cfg_or_sim;
    if ~isfield(sim_config, 'loe_vec'),  sim_config.loe_vec  = [1;0;0;0;0;0]; end
    if ~isfield(sim_config, 't_end'),    sim_config.t_end    = 30;   end
    if ~isfield(sim_config, 'dt'),       sim_config.dt       = 0.01; end
    if ~isfield(sim_config, 'alt_cmd'),  sim_config.alt_cmd  = 10;   end
    % Unify fault_time → t_fault
    if ~isfield(sim_config, 't_fault') && isfield(sim_config, 'fault_time')
        sim_config.t_fault = sim_config.fault_time;
    elseif ~isfield(sim_config, 't_fault')
        sim_config.t_fault = 5;
    end
end

% ── Run simulation ────────────────────────────────────────────────────────
sim = eval_simulation(d, sim_config);

% ── Scalar objectives ─────────────────────────────────────────────────────
alt_cmd   = sim_config.alt_cmd;
J_mission = min(sim.alt_rmse / alt_cmd, 20.0);   % cap at 20 to bound penalty

% Recovery time: first time attitude re-enters ±5° after fault
t_fault = sim_config.t_fault;
IDX_phi   = 7;  IDX_theta = 8;
t_vec  = sim.t_vec;
phi_h  = rad2deg(sim.X_hist(IDX_phi,:));
tht_h  = rad2deg(sim.X_hist(IDX_theta,:));
att_excurs = abs(phi_h) + abs(tht_h);   % combined excursion [deg]

post_fault = t_vec >= t_fault;
att_post   = att_excurs(post_fault);
t_post     = t_vec(post_fault);

recovered = att_post < 5.0;
if any(recovered)
    recovery_time = t_post(find(recovered, 1, 'first')) - t_fault;
else
    recovery_time = Inf;
end

% ── Pack result ───────────────────────────────────────────────────────────
result.feasible       = ~sim.diverged;
result.sim            = sim;
result.J_mission      = J_mission;
result.alt_rmse       = sim.alt_rmse;
result.max_att_excurs = sim.max_att_excurs;
result.recovery_time  = recovery_time;
result.diverged       = sim.diverged;
end
