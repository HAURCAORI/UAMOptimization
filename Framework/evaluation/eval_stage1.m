function result = eval_stage1(d, cfg_or_fault)
% EVAL_STAGE1  Stage 1 evaluator: ACS feasibility and fault-tolerance metrics.
%
%   Stage 1 screens designs for structural safety and fault tolerance before
%   any mission simulation is attempted.  It runs in ~15 ms per call and is
%   the evaluation mode used during optimization.
%
%   Stage 1 checks:
%     - Single-fault hover feasibility for all 6 motors
%     - Worst-case ACS volume retention (WCFR ≥ 5%)
%     - Fault Isotropy Index (FII) — objective to minimize
%     - Probabilistic Fault-Weighted ACS Retention (PFWAR)
%     - Hover margin relative to worst single-fault threshold
%
%   Inputs:
%     d            - Design struct (see design_default.m)
%     cfg_or_fault - (optional) mdo_config struct OR fault_config struct
%                    If mdo_config, reads cfg.fault fields automatically.
%
%   Output:
%     result — struct with fields:
%       .feasible       - true if all hard constraints pass
%       .acs            - full ACS metric struct (from eval_acs)
%       .hover_ok_all   - true if all single-fault hovers are feasible
%       .J_FII          - Fault Isotropy Index
%       .J_hover        - hover margin penalty ∈ [0,1] (0 = safe)
%       .WCFR           - worst-case fault retention
%       .PFWAR          - probabilistic fault-weighted retention
%       .hover_margin   - T_max / T_hover_worst - 1 (≥0 = safe)

if nargin < 2, cfg_or_fault = struct(); end

% ── Resolve fault config ──────────────────────────────────────────────────
if isfield(cfg_or_fault, 'vars')
    % mdo_config struct
    fault_config.include_double = cfg_or_fault.fault.include_double;
    fault_config.p_motor        = cfg_or_fault.fault.p_motor;
else
    % fault_config struct (or empty)
    fault_config = cfg_or_fault;
    if ~isfield(fault_config, 'include_double'), fault_config.include_double = false; end
    if ~isfield(fault_config, 'p_motor'),        fault_config.p_motor = 0.05; end
end

% ── ACS metrics ───────────────────────────────────────────────────────────
acs = eval_acs(d, fault_config);

% ── Hover feasibility ─────────────────────────────────────────────────────
hover_ok_all = all(acs.hover_ok_single);

% ── Scalar objectives ─────────────────────────────────────────────────────
J_FII = acs.FII;

if acs.hover_margin >= 0
    J_hover = 0;
else
    J_hover = 1 - exp(5 * acs.hover_margin);   % exponential penalty
end

% ── Hard feasibility check ────────────────────────────────────────────────
% A design passes Stage 1 if:
%   (a) WCFR ≥ 5%   (ACS doesn't collapse completely under worst fault)
%   (b) Geometry is non-degenerate (Lyo > Lyi + 0.1 m)
geo_ok     = (d.Lx > 0.5) && (d.Lyi > 0.5) && (d.Lyo > d.Lyi + 0.1);
wcfr_ok    = (acs.WCFR >= 0.05);
feasible   = geo_ok && wcfr_ok && hover_ok_all;

% ── Pack result ───────────────────────────────────────────────────────────
result.feasible      = feasible;
result.acs           = acs;
result.hover_ok_all  = hover_ok_all;
result.J_FII         = J_FII;
result.J_hover       = J_hover;
result.WCFR          = acs.WCFR;
result.PFWAR         = acs.PFWAR;
result.hover_margin  = acs.hover_margin;
result.T_hover_worst = acs.T_hover_worst;
result.objectives    = struct( ...
    'FII', J_FII, ...
    'hover', J_hover, ...
    'WCFR', acs.WCFR, ...
    'PFWAR', acs.PFWAR, ...
    'hover_margin_deficit', max(0, -acs.hover_margin));
end
