function compare_designs(designs, labels, options)
% COMPARE_DESIGNS  Side-by-side comparison of multiple hexacopter designs.
%
%   Evaluates each design in full (ACS + simulation) and produces a
%   comprehensive comparison table and radar chart.
%
%   Metrics compared
%   ----------------
%   Design parameters:  Lx, Lyi, Lyo, T_max, cT
%   ACS metrics:        vol_nominal, PFWAR, FII, WCFR, hover_ok_rate
%   Simulation metrics: alt_rmse, recovery_time, max_att, ctrl_effort
%   Objectives:         J_fault, J_isotropy, J_mission, J_cost, J_combined
%
%   Inputs:
%     designs  - {N×1} cell array of design structs
%     labels   - {N×1} cell array of design names (e.g. {'Baseline','Optimized'})
%     options  - (optional) struct:
%                  .eval_mode : 'acs'|'sim'|'full' (default: 'full')
%                  .sim_loe   : [6×1] LOE for simulation  (default: [1;0;0;0;0;0])
%                  .plot      : generate comparison plots  (default: true)

if nargin < 3, options = struct(); end
if ~isfield(options,'eval_mode'), options.eval_mode = 'full'; end
if ~isfield(options,'sim_loe'),   options.sim_loe   = [1;0;0;0;0;0]; end
if ~isfield(options,'plot'),      options.plot      = true; end

N = numel(designs);
if numel(labels) ~= N
    error('compare_designs: number of designs (%d) must match number of labels (%d)', N, numel(labels));
end

eval_opts.mode    = options.eval_mode;
eval_opts.verbose = false;
eval_opts.loe_for_sim = options.sim_loe;
if isfield(options,'sim_config')
    eval_opts.sim_config = options.sim_config;
end
eval_opts.sim_config.loe_vec = options.sim_loe;

% ── Evaluate all designs ──────────────────────────────────────────────────
fprintf('\n=== Evaluating %d designs ===\n', N);
results = cell(N,1);
for k = 1:N
    fprintf('  [%d/%d] %s ... ', k, N, labels{k});
    results{k} = eval_design(designs{k}, eval_opts);
    fprintf('done (J=%.4f)\n', results{k}.J_combined);
end

% ── Print comparison table ────────────────────────────────────────────────
fprintf('\n%s\n', repmat('=',1,90));
fprintf('%-20s', 'Metric');
for k = 1:N, fprintf('  %-14s', labels{k}); end
fprintf('\n%s\n', repmat('-',1,90));

% Design parameters
params = {'Lx','Lyi','Lyo','T_max','cT','m'};
units  = {'m',  'm',  'm',  'N',    '-', 'kg'};
for pi = 1:numel(params)
    fprintf('%-20s', sprintf('  d.%s [%s]', params{pi}, units{pi}));
    for k = 1:N
        val = designs{k}.(params{pi});
        fprintf('  %-14.3f', val);
    end
    fprintf('\n');
end

fprintf('%s\n', repmat('-',1,90));

% ACS metrics
acs_rows = {'vol_nominal','PFWAR','FII','WCFR'};
acs_lbl  = {'ACS vol [N·Nm³]', 'PFWAR [-]', 'FII [-]', 'WCFR [-]'};
for pi = 1:numel(acs_rows)
    fprintf('%-20s', sprintf('  %s', acs_lbl{pi}));
    for k = 1:N
        if ~isempty(results{k}.acs)
            val = results{k}.acs.(acs_rows{pi});
            fprintf('  %-14.4f', val);
        else
            fprintf('  %-14s', 'N/A');
        end
    end
    fprintf('\n');
end

% Hover success rate
fprintf('%-20s', '  Hover ok [/6]');
for k = 1:N
    if ~isempty(results{k}.acs)
        fprintf('  %-14s', sprintf('%d/6', sum(results{k}.acs.hover_ok_single)));
    else
        fprintf('  %-14s', 'N/A');
    end
end
fprintf('\n');

fprintf('%s\n', repmat('-',1,90));

% Simulation metrics
if ismember(options.eval_mode, {'sim','full'})
    sim_rows = {'alt_rmse','att_rmse_phi','att_rmse_theta','max_att_excurs','recovery_time','ctrl_effort','diverged'};
    sim_lbl  = {'alt RMSE [m]','phi RMSE [deg]','theta RMSE [deg]','max att [deg]','t_recovery [s]','ctrl effort','diverged'};
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
    fprintf('%s\n', repmat('-',1,90));
end

% Objectives
obj_rows = {'J_fault','J_isotropy','J_mission','J_mass','J_combined'};
for pi = 1:numel(obj_rows)
    fprintf('%-20s', sprintf('  %s', obj_rows{pi}));
    for k = 1:N
        val = results{k}.(obj_rows{pi});
        if isnan(val)
            fprintf('  %-14s', 'N/A');
        else
            fprintf('  %-14.4f', val);
        end
    end
    fprintf('\n');
end
fprintf('%s\n', repmat('=',1,90));

% ── Radar chart ───────────────────────────────────────────────────────────
if options.plot && ismember(options.eval_mode, {'acs','full'})
    make_radar_chart(results, labels, N);
end
end


function make_radar_chart(results, labels, N)
% Radar chart of normalized metrics
categories = {'PFWAR','1-FII','WCFR','Hover OK','1-J_cost'};
n_cat = numel(categories);

% Collect raw values
raw = zeros(N, n_cat);
for k = 1:N
    r = results{k};
    raw(k,1) = isempty(r.acs)*0 + ~isempty(r.acs) * r.acs.PFWAR;
    raw(k,2) = 1 - (isempty(r.acs)*1 + ~isempty(r.acs)*r.acs.FII);
    raw(k,3) = isempty(r.acs)*0 + ~isempty(r.acs) * r.acs.WCFR;
    raw(k,4) = isempty(r.acs)*0 + ~isempty(r.acs) * mean(double(r.acs.hover_ok_single));
    J_c = r.J_mass * r.J_motor;  raw(k,5) = 1 - min(J_c, 2)/2;
end

% Normalize to [0,1] across designs
col_min = min(raw,[],1);
col_max = max(raw,[],1);
rng_v   = col_max - col_min;
rng_v(rng_v < 1e-10) = 1;  % avoid div-by-zero
norm_vals = (raw - col_min) ./ rng_v;

% Angles for radar
theta = linspace(0, 2*pi, n_cat+1);
theta(end) = [];

figure('Name','Design Comparison Radar');
ax = axes; hold on; ax.Visible = 'off'; axis equal;
xlim([-1.4, 1.4]); ylim([-1.4, 1.4]);
title('Design Comparison Radar Chart', 'FontSize', 13);

% Draw background spokes and rings
for ring = 0.25:0.25:1.0
    xr = ring * cos(theta([1:end,1]));
    yr = ring * sin(theta([1:end,1]));
    plot(xr, yr, ':', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
end
for ti = 1:n_cat
    plot([0, cos(theta(ti))], [0, sin(theta(ti))], '-', 'Color', [0.7 0.7 0.7]);
    text(1.15*cos(theta(ti)), 1.15*sin(theta(ti)), categories{ti}, ...
         'HorizontalAlignment','center', 'FontSize', 9);
end

% Plot each design
cmap = lines(N);
for k = 1:N
    v = norm_vals(k,:);
    xp = v .* cos(theta);
    yp = v .* sin(theta);
    patch([xp, xp(1)], [yp, yp(1)], cmap(k,:), ...
          'FaceAlpha', 0.15, 'EdgeColor', cmap(k,:), 'LineWidth', 2);
end
legend(labels, 'Location', 'southoutside', 'Orientation', 'horizontal');
hold off
end
