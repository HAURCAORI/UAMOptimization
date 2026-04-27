function result = eval_stage2(d, cfg_or_sim)
% EVAL_STAGE2  Stage 2 evaluator: closed-loop mission performance metrics.
%
%   This wrapper normalizes sim-config naming and reuses the scalar metrics
%   returned by eval_simulation so Stage 2 and the core simulator stay
%   consistent.

if nargin < 2
    cfg_or_sim = struct();
end

if isfield(cfg_or_sim, 'vars')
    sim_config.loe_vec = cfg_or_sim.sim.loe_vec;
    sim_config.t_end   = cfg_or_sim.sim.T_end;
    sim_config.dt      = cfg_or_sim.sim.dt;
    sim_config.alt_cmd = cfg_or_sim.sim.alt_cmd;
    if isfield(cfg_or_sim.sim, 'scenario')
        sim_config.scenario = cfg_or_sim.sim.scenario;
    end
    if isfield(cfg_or_sim.sim, 'mission')
        sim_config.mission = cfg_or_sim.sim.mission;
    end
    if isfield(cfg_or_sim.sim, 'use_scaled_gains')
        sim_config.use_scaled_gains = cfg_or_sim.sim.use_scaled_gains;
    end
    if isfield(cfg_or_sim.sim, 'fault_time')
        sim_config.t_fault = cfg_or_sim.sim.fault_time;
    elseif isfield(cfg_or_sim.sim, 't_fault')
        sim_config.t_fault = cfg_or_sim.sim.t_fault;
    end
    % If neither field present, normalize_sim_config provides the default (10s)
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
        % Normalize by alt_cmd so diverged (total_rmse=200, alt_cmd=10) → J=20,
        % matching the hover-mode penalty scale exactly.
        tol = max(sim_config.alt_cmd, 1);
        J_mission = min(sim.total_rmse / tol, 20.0);
    otherwise
        sim = eval_simulation(d, sim_config);
        alt_cmd = sim_config.alt_cmd;
        J_mission = min(sim.alt_rmse / alt_cmd, 20.0);
end

result.feasible       = ~sim.diverged;
result.sim            = sim;
result.J_mission      = J_mission;
result.alt_rmse       = sim.alt_rmse;
result.max_att_excurs = sim.max_att_excurs;
result.recovery_time  = sim.recovery_time;
result.diverged       = sim.diverged;
end
