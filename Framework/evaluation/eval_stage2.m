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
    if isfield(cfg_or_sim.sim, 'fault_time')
        sim_config.t_fault = cfg_or_sim.sim.fault_time;
    elseif isfield(cfg_or_sim.sim, 't_fault')
        sim_config.t_fault = cfg_or_sim.sim.t_fault;
    else
        sim_config.t_fault = 5;
    end
else
    sim_config = cfg_or_sim;
    if ~isfield(sim_config, 'loe_vec')
        sim_config.loe_vec = [1;0;0;0;0;0];
    end
    if ~isfield(sim_config, 't_end')
        sim_config.t_end = 30;
    end
    if ~isfield(sim_config, 'dt')
        sim_config.dt = 0.01;
    end
    if ~isfield(sim_config, 'alt_cmd')
        sim_config.alt_cmd = 10;
    end
    if ~isfield(sim_config, 't_fault')
        if isfield(sim_config, 'fault_time')
            sim_config.t_fault = sim_config.fault_time;
        else
            sim_config.t_fault = 5;
        end
    end
end

sim = eval_simulation(d, sim_config);

alt_cmd = sim_config.alt_cmd;
result.feasible       = ~sim.diverged;
result.sim            = sim;
result.J_mission      = min(sim.alt_rmse / alt_cmd, 20.0);
result.alt_rmse       = sim.alt_rmse;
result.max_att_excurs = sim.max_att_excurs;
result.recovery_time  = sim.recovery_time;
result.diverged       = sim.diverged;
end
