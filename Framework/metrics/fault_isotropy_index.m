function [FII, retention_vec, fault_labels] = fault_isotropy_index(B, T_max, vol_nominal)
% FAULT_ISOTROPY_INDEX  Compute the Fault Isotropy Index (FII) for a design.
%
%   DEFINITION
%   ----------
%   The Fault Isotropy Index (FII) quantifies how uniformly fault tolerance
%   is distributed across all motors. For each single-motor failure, the
%   ACS volume retention ratio r_i = V_fault_i / V_nominal is computed.
%
%   FII is defined as the coefficient of variation of {r_i}:
%
%       FII = std(r_i) / mean(r_i)
%
%   Interpretation:
%     FII = 0 : perfectly isotropic — any motor failure causes identical
%               degradation (ideal from a balanced-reliability perspective)
%     FII > 0 : some motors are more critical than others
%     FII → 1 : highly anisotropic — one motor failure is catastrophic
%               while others are benign
%
%   MOTIVATION
%   ----------
%   A design with small worst-case ACS may still be acceptable if all
%   failures are roughly equally bad (predictable, plannable).  FII
%   captures this distributional property, which is absent from worst-case
%   metrics alone.  A low FII design is also more amenable to symmetric
%   fault reconfiguration strategies.
%
%   Inputs:
%     B           - [4×6] control effectiveness matrix
%     T_max       - scalar per-motor max thrust [N]
%     vol_nominal - nominal (no-fault) ACS volume (pre-computed scalar)
%                   If not provided or ≤ 0, it is computed internally.
%
%   Outputs:
%     FII          - Fault Isotropy Index (scalar ≥ 0)
%     retention_vec - [6×1] ACS volume retention ratio per motor failure
%     fault_labels  - {6×1} cell array of motor failure labels

if nargin < 3 || vol_nominal <= 0
    vol_nominal = compute_acs_volume(B, T_max);
end

n_motors     = 6;
retention_vec = zeros(n_motors, 1);
fault_labels  = cell(n_motors, 1);

for k = 1:n_motors
    loe_k = zeros(n_motors, 1);
    loe_k(k) = 1.0;   % full failure of motor k

    vol_k          = compute_acs_volume(B, T_max, loe_k);

    if vol_nominal > 0
        retention_vec(k) = vol_k / vol_nominal;
    else
        retention_vec(k) = 0;
    end

    fault_labels{k} = sprintf('Motor %d fail', k);
end

% Coefficient of variation: std / mean
mu  = mean(retention_vec);
sig = std(retention_vec);

if mu > 0
    FII = sig / mu;
else
    FII = 0;   % all retentions are zero (fully catastrophic — degenerate case)
end
end
