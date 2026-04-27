function results = sweep_design_space(d_base, sweep_var, sweep_range, options)
% SWEEP_DESIGN_SPACE  1D/2D design-space sweep for sensitivity analysis.
%
%   Evaluates the design at each point in a parameter grid and collects
%   all metrics.  Useful for understanding single-parameter and
%   pairwise-parameter effects before running the full optimizer.
%
%   Usage examples
%   --------------
%   % 1D sweep: vary Lyo from 3 to 9 m
%   results = sweep_design_space(d0, 'Lyo', linspace(3,9,25));
%
%   % 2D sweep: vary Lx and Lyo simultaneously
%   results = sweep_design_space(d0, {'Lx','Lyo'}, {linspace(1,5,15), linspace(3,9,15)});
%
%   Inputs:
%     d_base      - Baseline design struct
%     sweep_var   - String or {1×2} cell of design variable names to sweep
%     sweep_range - linspace vector (1D) or {1×2} cell of vectors (2D)
%     options     - (optional) struct with fields:
%                     .eval_mode : 'acs'|'sim'|'full'  (default: 'acs')
%                     .verbose   : show progress        (default: true)
%
%   Output:
%     results - Struct with fields:
%                 .var_values  : sweep values used
%                 .PFWAR       : [N] or [N1×N2]
%                 .FII         : [N] or [N1×N2]
%                 .WCFR        : [N] or [N1×N2]
%                 .vol_nominal : [N] or [N1×N2]
%                 .J_combined  : [N] or [N1×N2]
%                 .hover_rate  : fraction of fault scenarios where hover is ok
%                 .designs     : cell array of design structs

if nargin < 4, options = struct(); end
if ~isfield(options,'eval_mode'), options.eval_mode = 'acs'; end
if ~isfield(options,'verbose'),   options.verbose   = true;  end

eval_options.mode    = options.eval_mode;
eval_options.verbose = false;

is_2d = iscell(sweep_var);

if ~is_2d
    % ── 1D sweep ─────────────────────────────────────────────────────────
    N       = numel(sweep_range);
    PFWAR   = zeros(1,N);  FII  = zeros(1,N);  WCFR  = zeros(1,N);
    vol_nom = zeros(1,N);  Jc   = zeros(1,N);  h_rate = zeros(1,N);
    designs = cell(1,N);

    if options.verbose
        fprintf('Sweeping %s over %d points...\n', sweep_var, N);
    end

    for k = 1:N
        d_k           = d_base;
        d_k.(sweep_var) = sweep_range(k);

        r = eval_design(d_k, eval_options);

        if ~isempty(r.acs)
            PFWAR(k)   = r.acs.PFWAR;
            FII(k)     = r.acs.FII;
            WCFR(k)    = r.acs.WCFR;
            vol_nom(k) = r.acs.vol_nominal;
            h_rate(k)  = mean(double(r.acs.hover_ok_single));
        end
        Jc(k)       = r.J_combined;
        designs{k}  = d_k;

        if options.verbose && mod(k,5)==0
            fprintf('  [%d/%d] %s=%.3f  PFWAR=%.4f  FII=%.4f  J=%.4f\n', ...
                    k, N, sweep_var, sweep_range(k), PFWAR(k), FII(k), Jc(k));
        end
    end

    results.var_name   = sweep_var;
    results.var_values = sweep_range;
    results.PFWAR      = PFWAR;
    results.FII        = FII;
    results.WCFR       = WCFR;
    results.vol_nominal= vol_nom;
    results.J_combined = Jc;
    results.hover_rate = h_rate;
    results.designs    = designs;

else
    % ── 2D sweep ─────────────────────────────────────────────────────────
    var1   = sweep_var{1};   var2   = sweep_var{2};
    range1 = sweep_range{1}; range2 = sweep_range{2};
    N1 = numel(range1);  N2 = numel(range2);

    PFWAR   = zeros(N1,N2); FII  = zeros(N1,N2); WCFR  = zeros(N1,N2);
    vol_nom = zeros(N1,N2); Jc   = zeros(N1,N2); h_rate = zeros(N1,N2);
    designs = cell(N1,N2);

    total = N1*N2;
    cnt   = 0;
    if options.verbose
        fprintf('2D sweep: %s × %s  (%d × %d = %d evaluations)\n', ...
                var1, var2, N1, N2, total);
    end

    for i = 1:N1
        for j = 1:N2
            d_k        = d_base;
            d_k.(var1) = range1(i);
            d_k.(var2) = range2(j);

            % Skip infeasible geometry
            if isfield(d_k,'Lyo') && isfield(d_k,'Lyi')
                if d_k.Lyo <= d_k.Lyi + 0.1
                    PFWAR(i,j) = 0;  FII(i,j) = 1;  Jc(i,j) = 1e3;
                    designs{i,j} = d_k;
                    cnt = cnt + 1;
                    continue
                end
            end

            r = eval_design(d_k, eval_options);

            if ~isempty(r.acs)
                PFWAR(i,j)   = r.acs.PFWAR;
                FII(i,j)     = r.acs.FII;
                WCFR(i,j)    = r.acs.WCFR;
                vol_nom(i,j) = r.acs.vol_nominal;
                h_rate(i,j)  = mean(double(r.acs.hover_ok_single));
            end
            Jc(i,j)      = r.J_combined;
            designs{i,j} = d_k;

            cnt = cnt + 1;
            if options.verbose && mod(cnt,20)==0
                fprintf('  [%d/%d] %s=%.2f, %s=%.2f  PFWAR=%.4f\n', ...
                        cnt, total, var1, range1(i), var2, range2(j), PFWAR(i,j));
            end
        end
    end

    results.var_names  = sweep_var;
    results.var_values = sweep_range;
    results.PFWAR      = PFWAR;
    results.FII        = FII;
    results.WCFR       = WCFR;
    results.vol_nominal= vol_nom;
    results.J_combined = Jc;
    results.hover_rate = h_rate;
    results.designs    = designs;
end
end
