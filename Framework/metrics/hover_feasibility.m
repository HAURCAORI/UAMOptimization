function [feasible, utilization, T_opt] = hover_feasibility(B, T_max, m, g, loe_vec)
% HOVER_FEASIBILITY  Determine if the vehicle can hover under a fault.
%
%   Hover requires producing virtual control v_hover = [-m*g; 0; 0; 0]:
%     - Fz = -m*g  (upward thrust balances gravity in NED, z-down)
%     - L = M = N = 0  (zero moments → no acceleration)
%
%   This function solves the LP:
%       min  sum(T)          (minimum total thrust)
%       s.t. B * T = v_hover
%            0 ≤ T_i ≤ T_max_i
%
%   If the LP is feasible, hover is achievable.
%   The "utilization" is max(T_i / T_max_i), representing how close the
%   system is to actuator saturation.
%
%   Inputs:
%     B       - [4×6] control effectiveness matrix
%     T_max   - scalar per-motor max thrust [N]
%     m       - vehicle mass [kg]
%     g       - gravitational acceleration [m/s²]
%     loe_vec - [6×1] LOE vector per motor, ∈ [0,1]  (default: zeros)
%
%   Outputs:
%     feasible    - logical: true if hover is achievable
%     utilization - max T_i / T_max_i at optimal solution ∈ [0,1]
%                   (= inf if infeasible)
%     T_opt       - [6×1] optimal thrust solution (zeros if infeasible)

if nargin < 5
    loe_vec = zeros(6,1);
end
loe_vec = loe_vec(:);

% Target virtual control for hover (NED, Fz negative = upward)
v_hover = [-m * g; 0; 0; 0];

% Effective per-motor thrust limits
T_ub = T_max * (1 - loe_vec);   % [6×1]
T_lb = zeros(6, 1);

% LP: minimize sum(T) subject to B*T = v_hover, T_lb ≤ T ≤ T_ub
f_lp = ones(6, 1);   % objective: sum(T)

opts = optimoptions('linprog', 'Display', 'none', 'Algorithm', 'dual-simplex');
[T_opt, ~, exitflag] = linprog(f_lp, [], [], B, v_hover, T_lb, T_ub, opts);

if exitflag == 1
    % Feasible solution found
    feasible    = true;
    utilization = max(T_opt ./ max(T_ub, 1e-12));   % max load fraction
else
    feasible    = false;
    utilization = inf;
    T_opt       = zeros(6, 1);
end
end
