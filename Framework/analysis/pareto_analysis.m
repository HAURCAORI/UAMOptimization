function [pareto_J, pareto_designs, knee_idx] = pareto_analysis(pareto_designs_in, pareto_J_in, options)
% PARETO_ANALYSIS  Post-process and analyze a Pareto front from run_moo.
%
%   Performs:
%   1. Pareto domination filtering (removes dominated solutions)
%   2. Identification of the "knee point" (best balanced design)
%   3. Trade-off sensitivity: Δf1/Δf2 along the Pareto curve
%   4. Visualization of the front
%
%   KNEE POINT IDENTIFICATION
%   -------------------------
%   The knee point is the Pareto solution with maximum normal distance from
%   the line connecting the two extreme Pareto points.  This is the design
%   that offers the best trade-off between objectives.
%
%   Mathematically, for each Pareto point p, the knee score is:
%       knee_score(p) = min distance from p to the hyperplane defined by
%                       the extreme points of the Pareto front
%
%   For 2D projection (f1-f2), we use:
%       d(p) = ||(p - p_min) - t*(p_max - p_min)||
%   maximizing d over all p identifies the knee.
%
%   Inputs:
%     pareto_designs_in - {N×1} cell of design structs from run_moo
%     pareto_J_in       - [N×3] objective matrix from run_moo
%     options           - (optional) struct:
%                           .weights   : [3×1] objective importance for
%                                        weighted-sum knee identification
%                           .plot      : generate plots (default: true)
%                           .f_labels  : {3×1} objective axis labels
%
%   Outputs:
%     pareto_J       - [N_pareto × 3] filtered non-dominated objectives
%     pareto_designs - {N_pareto × 1} corresponding designs
%     knee_idx       - index into pareto_J of the knee design

if nargin < 3, options = struct(); end
if ~isfield(options,'weights'),  options.weights  = [1, 1, 1]/3; end
if ~isfield(options,'plot'),     options.plot     = true;  end
if ~isfield(options,'f_labels'), options.f_labels = {'J_{fault}','J_{isotropy}','J_{cost}'}; end

% ── Step 1: Filter truly non-dominated solutions ──────────────────────────
N       = size(pareto_J_in, 1);
is_nd   = true(N, 1);   % non-dominated flag

for i = 1:N
    for j = 1:N
        if i ~= j && is_nd(i)
            % j dominates i if: all objectives of j ≤ i, and at least one <
            if all(pareto_J_in(j,:) <= pareto_J_in(i,:)) && ...
               any(pareto_J_in(j,:) <  pareto_J_in(i,:))
                is_nd(i) = false;
                break
            end
        end
    end
end

pareto_J       = pareto_J_in(is_nd, :);
pareto_designs = pareto_designs_in(is_nd);
N_nd           = size(pareto_J, 1);

fprintf('Pareto analysis: %d / %d solutions are non-dominated\n', N_nd, N);

% ── Step 2: Knee point identification (normalized distance method) ─────────
% Normalize each objective to [0, 1] across the Pareto front
f_min = min(pareto_J, [], 1);
f_max = max(pareto_J, [], 1);
f_rng = f_max - f_min;
f_rng(f_rng < 1e-12) = 1;
F_norm = (pareto_J - f_min) ./ f_rng;   % [N_nd × 3]

% Line from ideal (0,0,0) to anti-ideal (1,1,1)
line_dir = ones(1,3) / sqrt(3);

% Distance from each point to the ideal-antiideal line
knee_score = zeros(N_nd, 1);
for k = 1:N_nd
    p = F_norm(k,:);
    proj = dot(p, line_dir) * line_dir;
    knee_score(k) = norm(p - proj);
end

[~, knee_idx] = max(knee_score);

fprintf('Knee point: design %d  (J = [%.4f, %.4f, %.4f])\n', ...
        knee_idx, pareto_J(knee_idx,1), pareto_J(knee_idx,2), pareto_J(knee_idx,3));

% ── Step 3: Trade-off analysis ────────────────────────────────────────────
% Sort by f1 (fault objective) and compute trade-off slope df2/df1
[~, sort_idx] = sort(pareto_J(:,1));
J_sorted = pareto_J(sort_idx, :);

if N_nd >= 3
    tradeoff_12 = diff(J_sorted(:,2)) ./ (diff(J_sorted(:,1)) + 1e-12);
    fprintf('Trade-off df2/df1 range: [%.3f, %.3f]\n', min(tradeoff_12), max(tradeoff_12));
end

% ── Step 4: Visualization ─────────────────────────────────────────────────
if options.plot
    lbl = options.f_labels;

    figure('Name','Pareto Front Analysis');

    % 3D Pareto front
    subplot(2,2,1);
    scatter3(pareto_J(:,1), pareto_J(:,2), pareto_J(:,3), 50, ...
             (1:N_nd)', 'filled', 'MarkerEdgeColor','k','LineWidth',0.5);
    hold on;
    scatter3(pareto_J(knee_idx,1), pareto_J(knee_idx,2), pareto_J(knee_idx,3), ...
             150, 'r', 'pentagram', 'filled', 'MarkerEdgeColor','k');
    xlabel(lbl{1}); ylabel(lbl{2}); zlabel(lbl{3});
    title('3D Pareto Front  (★ = knee)');
    colorbar; grid on; view(45,25);

    % f1 vs f2 projection
    subplot(2,2,2);
    scatter(pareto_J(:,1), pareto_J(:,2), 50, pareto_J(:,3), 'filled');
    hold on;
    plot(pareto_J(knee_idx,1), pareto_J(knee_idx,2), 'rp', ...
         'MarkerSize', 14, 'MarkerFaceColor','r');
    xlabel(lbl{1}); ylabel(lbl{2});
    title('f_1 vs f_2  (colored by f_3)');
    c = colorbar; c.Label.String = lbl{3}; grid on;

    % f1 vs f3 projection
    subplot(2,2,3);
    scatter(pareto_J(:,1), pareto_J(:,3), 50, pareto_J(:,2), 'filled');
    hold on;
    plot(pareto_J(knee_idx,1), pareto_J(knee_idx,3), 'rp', ...
         'MarkerSize', 14, 'MarkerFaceColor','r');
    xlabel(lbl{1}); ylabel(lbl{3});
    title('f_1 vs f_3  (colored by f_2)');
    c = colorbar; c.Label.String = lbl{2}; grid on;

    % Knee score distribution
    subplot(2,2,4);
    bar(sort(knee_score, 'descend'), 'FaceColor', [0.3 0.6 0.9]);
    xlabel('Pareto Solution (sorted)'); ylabel('Knee Score');
    title('Knee Scores (distance to ideal-antiideal line)');
    grid on;
    xline(1, 'r--', 'Knee', 'LabelHorizontalAlignment','right');

    sgtitle('Multi-Objective Pareto Analysis', 'FontSize', 13, 'FontWeight', 'bold');
end
end
