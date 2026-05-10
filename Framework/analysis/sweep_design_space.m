function results = sweep_design_space(d_base, sweep_var, sweep_range, options)
% SWEEP_DESIGN_SPACE  One- or two-dimensional design sweep.
%
%   results = sweep_design_space(d0, 'Lyo', linspace(3, 9, 25))
%   results = sweep_design_space(d0, {'Lx','Lyo'}, {x_vals, y_vals}, UAMOptions(...))

if nargin < 4
    options = UAMOptions();
end
options = normalize_call_options('sweep_design_space', options);
if ~isfield(options, 'eval_mode'), options.eval_mode = 'acs'; end
if ~isfield(options, 'verbose'),   options.verbose = true; end

eval_opt = UAMOptions( ...
    'Mode', options.eval_mode, ...
    'Objectives', get_field_or_default(options, 'objectives', struct()), ...
    'Model', get_field_or_default(options, 'model', struct()), ...
    'SimConfig', get_field_or_default(options, 'sim_config', struct()));

if ~iscell(sweep_var)
    results = run_1d_sweep(d_base, sweep_var, sweep_range, eval_opt, options.verbose);
else
    results = run_2d_sweep(d_base, sweep_var, sweep_range, eval_opt, options.verbose);
end
end

function results = run_1d_sweep(d_base, sweep_var, sweep_range, eval_opt, verbose)
N = numel(sweep_range);
PFWAR = zeros(1, N);
FII = zeros(1, N);
WCFR = zeros(1, N);
vol_nom = zeros(1, N);
Jc = zeros(1, N);
h_rate = zeros(1, N);
designs = cell(1, N);

if verbose
    fprintf('Sweeping %s over %d points...\n', sweep_var, N);
end

for k = 1:N
    d_k = d_base;
    d_k.(sweep_var) = sweep_range(k);
    r = eval_design(d_k, eval_opt);

    if ~isempty(r.acs)
        PFWAR(k) = r.acs.PFWAR;
        FII(k) = r.acs.FII;
        WCFR(k) = r.acs.WCFR;
        vol_nom(k) = r.acs.vol_nominal;
        h_rate(k) = mean(double(r.acs.hover_ok_single));
    end
    Jc(k) = r.J_combined;
    designs{k} = d_k;

    if verbose && mod(k, 5) == 0
        fprintf('  [%d/%d] %s=%.3f  PFWAR=%.4f  FII=%.4f  J=%.4f\n', ...
            k, N, sweep_var, sweep_range(k), PFWAR(k), FII(k), Jc(k));
    end
end

results.var_name = sweep_var;
results.var_values = sweep_range;
results.PFWAR = PFWAR;
results.FII = FII;
results.WCFR = WCFR;
results.vol_nominal = vol_nom;
results.J_combined = Jc;
results.hover_rate = h_rate;
results.designs = designs;
end

function results = run_2d_sweep(d_base, sweep_var, sweep_range, eval_opt, verbose)
var1 = sweep_var{1};
var2 = sweep_var{2};
range1 = sweep_range{1};
range2 = sweep_range{2};
N1 = numel(range1);
N2 = numel(range2);

PFWAR = zeros(N1, N2);
FII = zeros(N1, N2);
WCFR = zeros(N1, N2);
vol_nom = zeros(N1, N2);
Jc = zeros(N1, N2);
h_rate = zeros(N1, N2);
designs = cell(N1, N2);

total = N1 * N2;
count = 0;
if verbose
    fprintf('2D sweep: %s x %s  (%d x %d = %d evaluations)\n', ...
        var1, var2, N1, N2, total);
end

for i = 1:N1
    for j = 1:N2
        d_k = d_base;
        d_k.(var1) = range1(i);
        d_k.(var2) = range2(j);

        if isfield(d_k, 'Lyo') && isfield(d_k, 'Lyi') && d_k.Lyo <= d_k.Lyi + 0.1
            PFWAR(i, j) = 0;
            FII(i, j) = 1;
            Jc(i, j) = 1e3;
            designs{i, j} = d_k;
            count = count + 1;
            continue;
        end

        r = eval_design(d_k, eval_opt);
        if ~isempty(r.acs)
            PFWAR(i, j) = r.acs.PFWAR;
            FII(i, j) = r.acs.FII;
            WCFR(i, j) = r.acs.WCFR;
            vol_nom(i, j) = r.acs.vol_nominal;
            h_rate(i, j) = mean(double(r.acs.hover_ok_single));
        end
        Jc(i, j) = r.J_combined;
        designs{i, j} = d_k;

        count = count + 1;
        if verbose && mod(count, 20) == 0
            fprintf('  [%d/%d] %s=%.2f, %s=%.2f  PFWAR=%.4f\n', ...
                count, total, var1, range1(i), var2, range2(j), PFWAR(i, j));
        end
    end
end

results.var_names = sweep_var;
results.var_values = sweep_range;
results.PFWAR = PFWAR;
results.FII = FII;
results.WCFR = WCFR;
results.vol_nominal = vol_nom;
results.J_combined = Jc;
results.hover_rate = h_rate;
results.designs = designs;
end

function value = get_field_or_default(s, name, default_value)
if isfield(s, name)
    value = s.(name);
else
    value = default_value;
end
end
