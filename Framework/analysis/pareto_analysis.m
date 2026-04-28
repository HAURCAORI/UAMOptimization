function [pareto_J, pareto_designs, knee_idx] = pareto_analysis(pareto_designs_in, pareto_J_in, options)
% PARETO_ANALYSIS  Filter, summarize, and plot a Pareto front.
%
%   [pareto_J, pareto_designs, knee_idx] = pareto_analysis(designs, F)
%   [pareto_J, pareto_designs, knee_idx] = pareto_analysis(designs, F, UAMOptions(...))

if nargin < 3
    options = UAMOptions();
end
options = normalize_call_options('pareto_analysis', options);
if ~isfield(options, 'weights'),  options.weights = []; end
if ~isfield(options, 'plot'),     options.plot = true; end
if ~isfield(options, 'f_labels'), options.f_labels = {'J_{mass}','J_{power}','J_{fault}'}; end

N = size(pareto_J_in, 1);
is_nd = true(N, 1);
for i = 1:N
    for j = 1:N
        if i ~= j && is_nd(i)
            if all(pareto_J_in(j, :) <= pareto_J_in(i, :)) && ...
               any(pareto_J_in(j, :) < pareto_J_in(i, :))
                is_nd(i) = false;
                break;
            end
        end
    end
end

pareto_J = pareto_J_in(is_nd, :);
pareto_designs = pareto_designs_in(is_nd);
N_nd = size(pareto_J, 1);

fprintf('Pareto analysis: %d / %d solutions are non-dominated\n', N_nd, N);

f_min = min(pareto_J, [], 1);
f_max = max(pareto_J, [], 1);
f_rng = f_max - f_min;
f_rng(f_rng < 1e-12) = 1;
F_norm = (pareto_J - f_min) ./ f_rng;

line_dir = ones(1, size(F_norm, 2)) / sqrt(size(F_norm, 2));
knee_score = zeros(N_nd, 1);
for k = 1:N_nd
    p = F_norm(k, :);
    proj = dot(p, line_dir) * line_dir;
    knee_score(k) = norm(p - proj);
end

[~, knee_idx] = max(knee_score);
fprintf('Knee point: design %d  (J = [%s])\n', knee_idx, ...
    strjoin(compose('%.4f', pareto_J(knee_idx, :)), ', '));

[~, sort_idx] = sort(pareto_J(:, 1));
J_sorted = pareto_J(sort_idx, :);
if N_nd >= 3
    tradeoff_12 = diff(J_sorted(:, 2)) ./ (diff(J_sorted(:, 1)) + 1e-12);
    fprintf('Trade-off df2/df1 range: [%.3f, %.3f]\n', min(tradeoff_12), max(tradeoff_12));
end

if options.plot
    plot_pareto_front(pareto_J, knee_idx, knee_score, options.f_labels);
end
end

function plot_pareto_front(pareto_J, knee_idx, knee_score, labels)
N_nd  = size(pareto_J, 1);
N_obj = size(pareto_J, 2);

% Pad labels if fewer were provided than objectives
while numel(labels) < N_obj
    labels{end+1} = sprintf('f_{%d}', numel(labels)+1);
end

figure('Name', 'Pareto Front Analysis');

if N_obj >= 3
    subplot(2, 2, 1);
    scatter3(pareto_J(:,1), pareto_J(:,2), pareto_J(:,3), 50, ...
        (1:N_nd)', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
    hold on;
    scatter3(pareto_J(knee_idx,1), pareto_J(knee_idx,2), pareto_J(knee_idx,3), ...
        150, 'r', 'pentagram', 'filled', 'MarkerEdgeColor', 'k');
    xlabel(labels{1}); ylabel(labels{2}); zlabel(labels{3});
    title('3D Pareto Front'); colorbar; grid on; view(45, 25);

    subplot(2, 2, 2);
    scatter(pareto_J(:,1), pareto_J(:,2), 50, pareto_J(:,3), 'filled');
    hold on;
    plot(pareto_J(knee_idx,1), pareto_J(knee_idx,2), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
    xlabel(labels{1}); ylabel(labels{2}); title('f_1 vs f_2');
    c = colorbar; c.Label.String = labels{3}; grid on;

    subplot(2, 2, 3);
    scatter(pareto_J(:,1), pareto_J(:,3), 50, pareto_J(:,2), 'filled');
    hold on;
    plot(pareto_J(knee_idx,1), pareto_J(knee_idx,3), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
    xlabel(labels{1}); ylabel(labels{3}); title('f_1 vs f_3');
    c = colorbar; c.Label.String = labels{2}; grid on;

    subplot(2, 2, 4);
else
    subplot(1, 2, 1);
    scatter(pareto_J(:,1), pareto_J(:,2), 50, (1:N_nd)', 'filled', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
    hold on;
    plot(pareto_J(knee_idx,1), pareto_J(knee_idx,2), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
    xlabel(labels{1}); ylabel(labels{2}); title('Pareto Front'); colorbar; grid on;

    subplot(1, 2, 2);
end

bar(sort(knee_score, 'descend'), 'FaceColor', [0.3 0.6 0.9]);
xlabel('Pareto solution (sorted)'); ylabel('Knee score'); title('Knee Scores'); grid on;
xline(1, 'r--', 'Knee', 'LabelHorizontalAlignment', 'right');

sgtitle('Multi-Objective Pareto Analysis', 'FontSize', 13, 'FontWeight', 'bold');
end
