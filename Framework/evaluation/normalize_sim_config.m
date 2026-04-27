function sim_config = normalize_sim_config(sim_config)
% NORMALIZE_SIM_CONFIG  Apply defaults and unify simulation config aliases.

if nargin < 1 || isempty(sim_config)
    sim_config = struct();
end

if ~isfield(sim_config, 'loe_vec')
    sim_config.loe_vec = [1;0;0;0;0;0];
end
if ~isfield(sim_config, 't_end')
    if isfield(sim_config, 'T_end')
        sim_config.t_end = sim_config.T_end;
    else
        sim_config.t_end = 40;
    end
end
if ~isfield(sim_config, 'dt')
    sim_config.dt = 0.01;
end
if ~isfield(sim_config, 'alt_cmd')
    sim_config.alt_cmd = 10;
end
if ~isfield(sim_config, 'use_scaled_gains')
    sim_config.use_scaled_gains = true;
end

if isfield(sim_config, 't_fault')
    sim_config.fault_time = sim_config.t_fault;
elseif isfield(sim_config, 'fault_time')
    sim_config.t_fault = sim_config.fault_time;
else
    sim_config.t_fault = 10;
    sim_config.fault_time = 10;
end
end
