function compare_designs(designs, labels, options)
% COMPARE_DESIGNS  Side-by-side comparison of multiple designs.
%
%   compare_designs(designs, labels)
%   compare_designs(designs, labels, UAMOptions(...))

if nargin < 3
    options = UAMOptions();
end
options = normalize_call_options('compare_designs', options);
if ~isfield(options, 'eval_mode'), options.eval_mode = 'full'; end
if ~isfield(options, 'sim_loe'),   options.sim_loe = [1;0;0;0;0;0]; end
if ~isfield(options, 'plot'),      options.plot = true; end
if ~isfield(options, 'verbose'),   options.verbose = true; end

N = numel(designs);
if numel(labels) ~= N
    error('compare_designs: number of designs (%d) must match number of labels (%d).', ...
        N, numel(labels));
end

eval_opts = struct();
eval_opts.mode = options.eval_mode;
eval_opts.verbose = false;
eval_opts.loe_for_sim = options.sim_loe;
if isfield(options, 'objectives')
    eval_opts.objectives = options.objectives;
end
if isfield(options, 'model')
    eval_opts.model = options.model;
end
if isfield(options, 'sim_config')
    eval_opts.sim_config = options.sim_config;
else
    eval_opts.sim_config = struct();
end
eval_opts.sim_config.loe_vec = options.sim_loe;
eval_opts.sim_config = normalize_sim_config(eval_opts.sim_config);

if options.verbose
    fprintf('\n=== Evaluating %d designs ===\n', N);
end

results = cell(N, 1);
for k = 1:N
    if options.verbose
        fprintf('  [%d/%d] %s ... ', k, N, labels{k});
    end
    results{k} = eval_design(designs{k}, build_eval_options(eval_opts));
    if options.verbose
        fprintf('done (J=%.4f)\n', results{k}.J_combined);
    end
end

fprintf('\n%s\n', repmat('=', 1, 90));
fprintf('%-20s', 'Metric');
for k = 1:N
    fprintf('  %-14s', labels{k});
end
fprintf('\n%s\n', repmat('-', 1, 90));

print_design_rows(designs, N);
fprintf('%s\n', repmat('-', 1, 90));
print_acs_rows(results, N);
fprintf('%s\n', repmat('-', 1, 90));

if ismember(options.eval_mode, {'sim', 'full'})
    print_sim_rows(results, N);
    fprintf('%s\n', repmat('-', 1, 90));
end

print_objective_rows(results, N);
fprintf('%s\n', repmat('=', 1, 90));
fprintf('%-20s', '  Active objectives');
for k = 1:N
    if isempty(results{k}.objective_names)
        txt = 'N/A';
    else
        txt = strjoin(results{k}.objective_names, ',');
    end
    fprintf('  %-14s', txt);
end
fprintf('\n');
fprintf('%s\n', repmat('=', 1, 90));

if options.plot && ismember(options.eval_mode, {'acs', 'full'})
    make_radar_chart(results, labels, N);
end
end

function opt = build_eval_options(eval_opts)
opt = UAMOptions( ...
    'Mode', eval_opts.mode, ...
    'Fault', eval_opts.loe_for_sim, ...
    'Objectives', get_field_or_default(eval_opts, 'objectives', struct()), ...
    'Model', get_field_or_default(eval_opts, 'model', struct()), ...
    'SimConfig', get_field_or_default(eval_opts, 'sim_config', struct()));
end

function value = get_field_or_default(s, name, default_value)
if isfield(s, name)
    value = s.(name);
else
    value = default_value;
end
end

function print_design_rows(designs, N)
params = {'Lx','Lyi','Lyo','T_max','cT','m'};
units = {'m','m','m','N','-','kg'};
for pi = 1:numel(params)
    fprintf('%-20s', sprintf('  d.%s [%s]', params{pi}, units{pi}));
    for k = 1:N
        if strcmp(params{pi}, 'cT')
            [~, prop_k] = hexacopter_params(designs{k});
            val = prop_k.cT;
        elseif strcmp(params{pi}, 'm')
            [uam_k, ~] = hexacopter_params(designs{k});
            val = uam_k.m;
        else
            val = designs{k}.(params{pi});
        end
        fprintf('  %-14.3f', val);
    end
    fprintf('\n');
end
end

function print_acs_rows(results, N)
acs_rows = {'vol_nominal','PFWAR','FII','WCFR'};
acs_lbl = {'ACS vol [N*N*m]', 'PFWAR [-]', 'FII [-]', 'WCFR [-]'};
for pi = 1:numel(acs_rows)
    fprintf('%-20s', sprintf('  %s', acs_lbl{pi}));
    for k = 1:N
        if ~isempty(results{k}.acs)
            fprintf('  %-14.4f', results{k}.acs.(acs_rows{pi}));
        else
            fprintf('  %-14s', 'N/A');
        end
    end
    fprintf('\n');
end

fprintf('%-20s', '  Hover ok [/6]');
for k = 1:N
    if ~isempty(results{k}.acs)
        fprintf('  %-14s', sprintf('%d/6', sum(results{k}.acs.hover_ok_single)));
    else
        fprintf('  %-14s', 'N/A');
    end
end
fprintf('\n');
end

function print_sim_rows(results, N)
sim_rows = {'alt_rmse','att_rmse_phi','att_rmse_theta','max_att_excurs', ...
    'recovery_time','ctrl_effort','diverged'};
sim_lbl = {'alt RMSE [m]','phi RMSE [deg]','theta RMSE [deg]', ...
    'max att [deg]','t_recovery [s]','ctrl effort','diverged'};

if ~isempty(results) && ~isempty(results{1}.sim) && isfield(results{1}.sim, 'path_rmse')
    sim_rows = [{'path_rmse'}, sim_rows];
    sim_lbl = [{'path RMSE [m]'}, sim_lbl];
end

for pi = 1:numel(sim_rows)
    fprintf('%-20s', sprintf('  %s', sim_lbl{pi}));
    for k = 1:N
        if ~isempty(results{k}.sim)
            val = results{k}.sim.(sim_rows{pi});
            if islogical(val)
                fprintf('  %-14s', mat2str(val));
            elseif isinf(val)
                fprintf('  %-14s', 'Inf');
            else
                fprintf('  %-14.3f', val);
            end
        else
            fprintf('  %-14s', 'N/A');
        end
    end
    fprintf('\n');
end
end

function print_objective_rows(results, N)
obj_rows = {'J_mass','J_power','J_fault_thrust','J_fault_alloc','J_hover_nom', ...
    'J_FII','J_hover','J_mission','J_cost','J_combined'};
for pi = 1:numel(obj_rows)
    fprintf('%-20s', sprintf('  %s', obj_rows{pi}));
    for k = 1:N
        if isfield(results{k}, obj_rows{pi})
            val = results{k}.(obj_rows{pi});
        else
            val = NaN;
        end
        if isnan(val)
            fprintf('  %-14s', 'N/A');
        else
            fprintf('  %-14.4f', val);
        end
    end
    fprintf('\n');
end
end

function make_radar_chart(results, labels, N)
categories = {'PFWAR','1-FII','WCFR','Hover OK','1-J_cost'};
n_cat = numel(categories);

raw = zeros(N, n_cat);
for k = 1:N
    r = results{k};
    raw(k,1) = isempty(r.acs) * 0 + ~isempty(r.acs) * r.acs.PFWAR;
    raw(k,2) = 1 - (isempty(r.acs) * 1 + ~isempty(r.acs) * r.acs.FII);
    raw(k,3) = isempty(r.acs) * 0 + ~isempty(r.acs) * r.acs.WCFR;
    raw(k,4) = isempty(r.acs) * 0 + ~isempty(r.acs) * mean(double(r.acs.hover_ok_single));
    raw(k,5) = 1 - min(r.J_cost, 2) / 2;
end

col_min = min(raw, [], 1);
col_max = max(raw, [], 1);
rng_v = col_max - col_min;
rng_v(rng_v < 1e-10) = 1;
norm_vals = (raw - col_min) ./ rng_v;

theta = linspace(0, 2*pi, n_cat + 1);
theta(end) = [];

figure('Name', 'Design Comparison Radar');
ax = axes;
hold on;
ax.Visible = 'off';
axis equal;
xlim([-1.4, 1.4]);
ylim([-1.4, 1.4]);
title('Design Comparison Radar Chart', 'FontSize', 13);

for ring = 0.25:0.25:1.0
    xr = ring * cos(theta([1:end,1]));
    yr = ring * sin(theta([1:end,1]));
    plot(xr, yr, ':', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
end
for ti = 1:n_cat
    plot([0, cos(theta(ti))], [0, sin(theta(ti))], '-', 'Color', [0.7 0.7 0.7]);
    text(1.15*cos(theta(ti)), 1.15*sin(theta(ti)), categories{ti}, ...
        'HorizontalAlignment', 'center', 'FontSize', 9);
end

cmap = lines(N);
for k = 1:N
    v = norm_vals(k, :);
    xp = v .* cos(theta);
    yp = v .* sin(theta);
    patch([xp, xp(1)], [yp, yp(1)], cmap(k, :), ...
        'FaceAlpha', 0.15, 'EdgeColor', cmap(k, :), 'LineWidth', 2);
end
legend(labels, 'Location', 'southoutside', 'Orientation', 'horizontal');
hold off;
end
