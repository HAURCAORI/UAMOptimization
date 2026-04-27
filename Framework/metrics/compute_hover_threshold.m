function [T_thresh, T_ratios, hover_lambda] = compute_hover_threshold(B, m, g, loe_vec_list)
% COMPUTE_HOVER_THRESHOLD  Find the minimum T_max for hover under each fault.
%
%   Uses a single LP per fault scenario instead of binary search.
%   Formulation: find the maximum hover fraction λ achievable within T_max:
%
%       max   λ
%       s.t.  B_fault * T = λ * [-m*g; 0; 0; 0]
%             0 ≤ T_i ≤ T_max_eff_i
%
%   Rearranged as standard LP with variables z = [T (6×1); λ (1×1)]:
%       min   -λ
%       s.t.  [B_fault, [m*g; 0; 0; 0]] * z = 0
%             0 ≤ T_i ≤ T_max_eff_i,   0 ≤ λ ≤ 1
%
%   The required T_max for hover = T_max_used / λ_opt.
%   If λ_opt ≥ 1 → hover is feasible at the given T_max.
%
%   This replaces the previous 40-iteration binary search with ONE LP call,
%   giving ~40x speedup per fault scenario.
%
%   Inputs:
%     B            - [4×6] control effectiveness matrix
%     m            - vehicle mass [kg]
%     g            - gravity [m/s²]
%     loe_vec_list - [N_fault × 6] LOE matrix, one row per scenario
%
%   Outputs:
%     T_thresh     - [N_fault × 1] minimum T_max per scenario [N]
%                    (computed at reference T_max = m*g/6, then scaled)
%     T_ratios     - [N_fault × 1] ratios normalized by m*g/6
%     hover_lambda - [N_fault × 1] max hover fraction at T_max = m*g/6

N_fault       = size(loe_vec_list, 1);
T_hover_share = m * g / 6;   % nominal hover thrust per motor [N]

T_thresh     = zeros(N_fault, 1);
T_ratios     = zeros(N_fault, 1);
hover_lambda = zeros(N_fault, 1);

% Use T_ref = T_hover_share as the reference T_max for LP
% The LP finds max λ within T_ref, then T_thresh = T_ref / λ
T_ref = T_hover_share;

lp_opts = optimoptions('linprog','Display','none','Algorithm','dual-simplex');

for k = 1:N_fault
    loe_k    = loe_vec_list(k,:)';
    T_ub_eff = T_ref * (1 - loe_k);   % [6×1] upper bounds

    % LP variables: z = [T1..T6, λ]  (7 variables)
    % Objective: minimize -λ  (maximize λ)
    f_lp = [zeros(6,1); -1];

    % Equality: B_fault * T - λ*(m*g) * e1 = 0
    %   [B, [m*g; 0; 0; 0]] * z = 0
    v_hover = [-m*g; 0; 0; 0];
    A_eq    = [B, -v_hover];   % [4×7]
    b_eq    = zeros(4,1);

    % Bounds: 0 ≤ T ≤ T_ub_eff, 0 ≤ λ ≤ 1
    lb_lp   = zeros(7,1);
    ub_lp   = [T_ub_eff; 1.0];

    [z_opt, ~, exitflag] = linprog(f_lp, [], [], A_eq, b_eq, lb_lp, ub_lp, lp_opts);

    if exitflag == 1
        lambda_opt       = z_opt(7);
        hover_lambda(k)  = lambda_opt;
        if lambda_opt > 1e-9
            T_thresh(k) = T_ref / lambda_opt;
        else
            T_thresh(k) = inf;   % hover not achievable even at high T_max
        end
    else
        hover_lambda(k) = 0;
        T_thresh(k)     = inf;
    end
end

% Replace inf with 6*m*g cap for numerical handling
T_thresh(~isfinite(T_thresh)) = 6 * m * g;
T_ratios = T_thresh / T_hover_share;
end
