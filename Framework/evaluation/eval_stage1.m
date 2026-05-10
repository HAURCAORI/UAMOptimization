function result = eval_stage1(d, cfg_or_fault)
% EVAL_STAGE1  Stage 1: ACS + nominal/fault hover objectives (no simulation).
%
%   Objective equations (see OPTIMIZATION.md for full specification):
%
%     J_mass         = m_tot / m_ref
%     J_power        = P_hover / P_ref            (induced-power proxy)
%     J_fault_thrust = max(0, gamma_req - gamma_T)^2
%     J_fault_alloc  = sigma_ref / sigma_worst     (control effectiveness ratio)
%     J_hover_nom    = (T_bar_hover / T_max)^2     (nominal load fraction squared)
%     J_fault        = 0.5 * (J_fault_thrust + J_fault_alloc)
%
%   All parameters (gamma_req, weights) are set in mdo_config.m.
%
%   Outputs:
%     result.feasible   - geometric + hover feasibility flag
%     result.acs        - ACS struct (FII, WCFR, PFWAR, hover_margin, ...)
%     result.J_FII      - fault isotropy index
%     result.J_hover    - hover safety penalty (0 when feasible)
%     result.objectives - struct with all computed values (primary objectives
%                         + diagnostics); add new objectives here

if nargin < 2
    cfg_or_fault = struct();
end

%% Parse config: accept either a full mdo_config struct or a fault_config struct
if isfield(cfg_or_fault, 'vars')
    % Full cfg passed
    fault_config.include_double = cfg_or_fault.fault.include_double;
    fault_config.p_motor        = cfg_or_fault.fault.p_motor;
    fault_config.gamma_T_req    = cfg_or_fault.fault.gamma_T_req;
else
    fault_config = cfg_or_fault;
    if ~isfield(fault_config, 'include_double'), fault_config.include_double = false; end
    if ~isfield(fault_config, 'p_motor'),        fault_config.p_motor = 0.05; end
    if ~isfield(fault_config, 'gamma_T_req'),    fault_config.gamma_T_req = 1.5; end
end

gamma_T_req = fault_config.gamma_T_req;

[UAM, Prop, Env] = hexacopter_params(d);
B     = UAM.B;
T_max = Prop.T_max;
m     = UAM.m;
g     = Env.g;

%% ACS metrics
acs   = eval_acs(d, fault_config);
J_FII = acs.FII;
if acs.hover_margin >= 0
    J_hover = 0;
else
    J_hover = 1 - exp(5 * acs.hover_margin);
end

%% Reference design for normalization
ref = design_default();
if isfield(d, 'use_vehicle_model'), ref.use_vehicle_model = d.use_vehicle_model; end
if isfield(d, 'm_payload'),         ref.m_payload = d.m_payload; end
[UAM_ref, Prop_ref, Env_ref] = hexacopter_params(ref);

%% J_mass  =  m_tot / m_ref
if isfield(UAM, 'model') && isfield(UAM.model, 'm_ref') && UAM.model.m_ref > 0
    m_ref = UAM.model.m_ref;
else
    m_ref = UAM_ref.m;
end
J_mass = m / max(m_ref, 1e-9);

%% J_hover_nom  =  (T_bar_hover / T_max)^2
%  T_bar_hover: mean motor thrust in nominal hover
[hover_ok_nom, hover_util_nom, T_hover_nom] = hover_feasibility(B, T_max, m, g, zeros(6,1));
if hover_ok_nom
    J_hover_nom  = (mean(T_hover_nom) / max(T_max, 1e-9))^2;
    power_proxy  = hover_power_proxy(T_hover_nom, d);
else
    J_hover_nom = inf;
    power_proxy = inf;
end

%% J_power  =  P_hover / P_ref
[hover_ok_ref, ~, T_hover_ref] = hover_feasibility(UAM_ref.B, Prop_ref.T_max, UAM_ref.m, Env_ref.g, zeros(6,1));
if hover_ok_ref
    power_proxy_ref = hover_power_proxy(T_hover_ref, ref);
else
    power_proxy_ref = 1.0;
end
J_power = power_proxy / max(power_proxy_ref, 1e-9);

%% J_fault_thrust  =  max(0, gamma_req - gamma_worst)^2
%  gamma_worst: minimum post-fault thrust-to-weight ratio over all single-motor failures
%% J_fault_alloc   =  sigma_ref / sigma_worst
%  sigma_worst: minimum singular value of the dimensionless control matrix under fault
N_motors    = 6;
fault_gamma = zeros(N_motors, 1);
fault_sigma = zeros(N_motors, 1);

for k = 1:N_motors
    loe_k          = zeros(N_motors, 1);  loe_k(k) = 1;
    T_ub           = T_max * (1 - loe_k);
    fault_gamma(k) = sum(T_ub) / max(m * g, 1e-9);
    fault_sigma(k) = ctrl_effectiveness(B, T_ub, m, g, d);
end

gamma_worst = min(fault_gamma);
sigma_worst = min(fault_sigma);

sigma_ref = inf;
for k = 1:N_motors
    loe_k_ref = zeros(N_motors, 1);  loe_k_ref(k) = 1;
    T_ub_ref  = Prop_ref.T_max * (1 - loe_k_ref);
    sigma_ref = min(sigma_ref, ctrl_effectiveness(UAM_ref.B, T_ub_ref, UAM_ref.m, Env_ref.g, ref));
end
if ~isfinite(sigma_ref) || sigma_ref <= 0, sigma_ref = 1.0; end

J_fault_thrust = max(0, gamma_T_req - gamma_worst)^2;
J_fault_alloc  = sigma_ref / max(sigma_worst, 1e-9);
J_fault        = 0.5 * (J_fault_thrust + J_fault_alloc);

%% Feasibility
geo_ok = (d.Lx > 0.5) && (d.Lyi > 0.5) && (d.Lyo > d.Lyi + 0.1);

result.feasible = geo_ok && (acs.WCFR >= 0.05) && all(acs.hover_ok_single) && ...
                  hover_ok_nom && isfinite(J_hover_nom);
result.acs     = acs;
result.J_FII   = J_FII;
result.J_hover = J_hover;

% Primary objectives (used by combine_objectives in eval_design)
% Diagnostic fields (hover_util_nom, fault_gamma_worst, etc.) are here for
% analysis and compare_designs tables, not for the optimizer.
result.objectives = struct( ...
    'mass',                 J_mass, ...
    'power',                J_power, ...
    'fault_thrust',         J_fault_thrust, ...
    'fault_alloc',          J_fault_alloc, ...
    'fault',                J_fault, ...
    'hover_nom',            J_hover_nom, ...
    'FII',                  J_FII, ...
    'hover',                J_hover, ...
    'WCFR',                 acs.WCFR, ...
    'PFWAR',                acs.PFWAR, ...
    'hover_margin_deficit', max(0, -acs.hover_margin), ...
    'hover_util_nom',       hover_util_nom, ...
    'fault_gamma_worst',    gamma_worst, ...
    'fault_sigma_worst',    sigma_worst);
end

function P = hover_power_proxy(T_hover, d)
% Ideal induced-power proxy: sum(T_i^1.5) / sqrt(A_disk)
% Derived from momentum theory; proportional to profile-drag-free hover power.
if isfield(d, 'd_prop') && ~isempty(d.d_prop)
    d_prop = d.d_prop;
else
    d_prop = 0.40;
end
A = pi * (0.5 * d_prop)^2;
P = sum(max(T_hover, 0) .^ 1.5) / max(sqrt(A), 1e-9);
end

function s_min = ctrl_effectiveness(B, T_ub, m, g, d)
% Minimum singular value of the dimensionless scaled control-effectiveness matrix.
% Row scales: [1/(mg), 1/(mg*L), 1/(mg*L), 1/yaw_max] make rows commensurate.
L_ref   = max([abs(d.Lx), abs(d.Lyi), abs(d.Lyo), 1e-3]);
yaw_ref = max(sum(abs(B(4, :)) .* T_ub.'), 1e-9);
scale   = diag([1/max(m*g, 1e-9), 1/max(m*g*L_ref, 1e-9), 1/max(m*g*L_ref, 1e-9), 1/yaw_ref]);
sv      = svd(scale * B * diag(T_ub));
s_min   = sv(end);
end
