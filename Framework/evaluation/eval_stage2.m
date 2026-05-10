function result = eval_stage2(d, cfg_or_sim)
% EVAL_STAGE2  Closed-loop simulation metrics for hover or figure-8 missions.
%
%   Stage 2 is scenario-aware:
%     - hover   : combines altitude, attitude, effort, and recovery metrics
%     - figure8 : uses tracking RMSE over the mission trajectory

if nargin < 2
    cfg_or_sim = struct();
end

if isfield(cfg_or_sim, 'vars')
    sim_config = cfg_or_sim.sim;
else
    sim_config = cfg_or_sim;
end

sim_config = normalize_sim_config(sim_config);
if ~isfield(sim_config, 'scenario')
    sim_config.scenario = 'hover';
end

switch lower(sim_config.scenario)
    case 'figure8'
        sim = eval_mission_figure8(d, sim_config);
        tol = max(sim_config.alt_cmd, 1);
        J_mission = min(sim.total_rmse / tol, 20.0);

    otherwise
        sim = eval_simulation(d, sim_config);
        J_mission = hover_stage2_score(sim, sim_config);
end

result.feasible = ~sim.diverged;
result.sim = sim;
result.J_mission = J_mission;
result.alt_rmse = sim.alt_rmse;
result.max_att_excurs = sim.max_att_excurs;
result.recovery_time = sim.recovery_time;
result.diverged = sim.diverged;
end

function J = hover_stage2_score(sim, sim_config)
if sim.diverged || ~isfinite(sim.alt_rmse)
    J = 20.0;
    return;
end

alt_scale = max(sim_config.alt_cmd, 1.0);
att_scale = 5.0; % deg
recovery_horizon = max(sim_config.t_end - sim_config.t_fault, sim_config.dt);

j_alt   = min(sim.alt_rmse / alt_scale, 20.0);
j_phi   = min(sim.att_rmse_phi / att_scale, 20.0);
j_theta = min(sim.att_rmse_theta / att_scale, 20.0);
j_eff   = min(sim.ctrl_effort, 20.0);

if isfield(sim, 'has_fault') && sim.has_fault && isfinite(sim.recovery_time)
    j_rec = min(sim.recovery_time / recovery_horizon, 20.0);
else
    j_rec = 0.0;
end

J = 0.45 * j_alt + 0.20 * j_phi + 0.20 * j_theta + 0.10 * j_eff + 0.05 * j_rec;
J = min(J, 20.0);
end
