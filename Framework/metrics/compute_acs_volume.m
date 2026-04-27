function [vol, pts_4d, hull_idx] = compute_acs_volume(B, T_max, loe_vec)
% COMPUTE_ACS_VOLUME  Compute the 4D Attainable Control Set volume.
%
%   The ACS for a hexacopter is a zonotope:
%       ACS = { B * T  :  T_min_i ≤ T_i ≤ T_max_i }
%
%   Under a Loss-of-Effectiveness (LOE) fault, T_i_max → (1-LOE_i)*T_max.
%   The vertex set of the ACS is B applied to the 2^6 = 64 extreme thrust
%   combinations. The 4D convex hull of these vertices yields the ACS.
%
%   Inputs:
%     B       - [4×6] control effectiveness matrix
%     T_max   - scalar per-motor max thrust [N]
%     loe_vec - [6×1] loss-of-effectiveness per motor, ∈ [0,1]
%               (0 = healthy, 1 = fully failed)
%
%   Outputs:
%     vol      - 4D convex hull volume of the ACS (scalar)
%                Units: [N·Nm^3] (Fz·L·M·N space)
%     pts_4d   - [N_pts × 4] matrix of unique ACS vertices [Fz, L, M, N]
%     hull_idx - convex hull facet index matrix (from convhulln)
%
%   Notes:
%     If the ACS is degenerate (rank < 4, e.g., ≥3 motors failed), the
%     returned volume is 0 and hull_idx is empty.

% Default: no fault
if nargin < 3
    loe_vec = zeros(6,1);
end
loe_vec = loe_vec(:);

% Effective T_max per motor after LOE
T_max_eff = T_max * (1 - loe_vec);   % [6×1]

% Generate all 2^6 = 64 extreme thrust combinations (vertices of the box)
n_motors  = 6;
n_vertices = 2^n_motors;
T_vertices = zeros(n_motors, n_vertices);
for k = 1:n_motors
    % Pattern: alternating 0/T_max with period 2^(k-1)
    period = 2^(k-1);
    pattern = repmat([zeros(1,period), ones(1,period)], 1, n_vertices/(2*period));
    T_vertices(k,:) = T_max_eff(k) * pattern;
end

% Map thrust vertices to virtual control space: v = B * T
V_all = B * T_vertices;   % [4 × 64]

% Round and keep unique points (numerical precision)
pts_4d = unique(round(V_all', 10), 'rows');   % [N_pts × 4]

% Compute 4D convex hull and volume
n_pts = size(pts_4d, 1);

% SVD-based degeneracy check — faster and safer than rank() for near-degenerate inputs.
% 'Qx' (exact arithmetic) in convhulln can hang indefinitely when the 4th singular
% value is small (near-planar polytope), so we gate on the condition ratio first.
% Threshold 1e-7: accounts for N-channel (cT*T_max) being ~200x smaller than Fz.
pts_c  = pts_4d - mean(pts_4d, 1);
sv     = svd(pts_c);
well_conditioned = (n_pts >= 5) && (sv(4) / max(sv(1), 1e-12) > 1e-7);

if well_conditioned
    try
        % 'Qt' triangulates output; 'Pp' suppresses precision warnings.
        % 'Qx' removed — it enables exact arithmetic which can hang on
        % near-degenerate inputs encountered during optimisation sweeps.
        [hull_idx, vol] = convhulln(pts_4d, {'Qt', 'Pp'});
    catch
        vol      = 0;
        hull_idx = [];
    end
else
    % Near-degenerate ACS: too few independent virtual-control directions
    vol      = 0;
    hull_idx = [];
end
end
