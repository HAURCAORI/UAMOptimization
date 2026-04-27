function metrics = eval_acs(d, fault_config)
% EVAL_ACS  Evaluate ACS-based fault tolerance metrics for a design.
%
%   This function computes all internal-capability metrics derived from the
%   Attainable Control Set (ACS), which represents the set of achievable
%   virtual controls [Fz; L; M; N] given actuator limits.
%
%   METRICS RETURNED
%   ----------------
%   vol_nominal   - ACS volume under no fault [N·Nm^3]
%   vol_fault     - ACS volume per fault scenario [N_fault × 1]
%   retention     - ACS retention ratio per scenario = vol_fault/vol_nominal
%   PFWAR         - Probabilistic Fault-Weighted ACS Retention (novel)
%   FII           - Fault Isotropy Index (novel, see fault_isotropy_index.m)
%   WCFR          - Worst-Case Fault Retention (min over single-motor faults)
%   hover_ok      - logical [N_fault × 1], hover feasible after each fault
%   hover_util    - actuator utilization at hover [N_fault × 1]
%   fault_list    - [N_fault × 6] LOE matrix (one scenario per row)
%   fault_labels  - {N_fault × 1} string labels
%
%   PROBABILISTIC MODEL (for PFWAR)
%   --------------------------------
%   Each motor fails independently with probability p_fail = fault_config.p_motor.
%   Scenario probabilities:
%     P(motor k fails alone) ≈ p_fail * (1-p_fail)^5   [single fault]
%     P(motors i,j fail)     ≈ p_fail^2 * (1-p_fail)^4 [double fault]
%   These are normalized to sum to 1 over all considered scenarios.
%
%   Inputs:
%     d            - Design struct (see design_default.m)
%     fault_config - (optional) struct with fields:
%                      .include_double  : include double-motor faults (default: true)
%                      .p_motor         : per-motor failure probability (default: 0.05)
%
%   Output:
%     metrics - struct with all fields listed above

% ── Defaults ──────────────────────────────────────────────────────────────
if nargin < 2, fault_config = struct(); end
if ~isfield(fault_config,'include_double'), fault_config.include_double = false; end
if ~isfield(fault_config,'p_motor'),        fault_config.p_motor = 0.05; end

p  = fault_config.p_motor;
n  = 6;   % number of motors

% ── Build physical model ──────────────────────────────────────────────────
[UAM, Prop, Env] = hexacopter_params(d);
B     = UAM.B;
T_max = Prop.T_max;
m     = UAM.m;
g     = Env.g;

% ── Enumerate fault scenarios ─────────────────────────────────────────────
% Row k of fault_list: LOE vector for scenario k (0 = healthy, 1 = failed)
fault_list   = [];
fault_labels = {};
fault_probs  = [];

% Single-motor faults
for k = 1:n
    loe = zeros(1,n);  loe(k) = 1;
    fault_list   = [fault_list; loe];
    fault_labels = [fault_labels; {sprintf('Motor %d fail', k)}];
    fault_probs  = [fault_probs; p * (1-p)^(n-1)];
end

% Double-motor faults (optional)
if fault_config.include_double
    pairs = nchoosek(1:n, 2);
    for ki = 1:size(pairs,1)
        loe = zeros(1,n);
        loe(pairs(ki,:)) = 1;
        fault_list   = [fault_list; loe];
        fault_labels = [fault_labels; {sprintf('Motors %d+%d fail', pairs(ki,1), pairs(ki,2))}];
        fault_probs  = [fault_probs; p^2 * (1-p)^(n-2)];
    end
end

% Normalize probabilities
fault_probs = fault_probs / sum(fault_probs);
N_fault     = size(fault_list, 1);

% ── Nominal ACS ───────────────────────────────────────────────────────────
vol_nominal = compute_acs_volume(B, T_max, zeros(n,1));

% ── Faulted ACS metrics ───────────────────────────────────────────────────
vol_fault  = zeros(N_fault, 1);
retention  = zeros(N_fault, 1);
hover_ok   = false(N_fault, 1);
hover_util = inf(N_fault, 1);

for k = 1:N_fault
    loe_k = fault_list(k,:)';

    % ACS volume under this fault
    vol_fault(k) = compute_acs_volume(B, T_max, loe_k);
    if vol_nominal > 0
        retention(k) = vol_fault(k) / vol_nominal;
    end

    % Hover feasibility
    [hover_ok(k), hover_util(k)] = hover_feasibility(B, T_max, m, g, loe_k);
end

% ── Novel aggregate metrics ───────────────────────────────────────────────

% 1. PFWAR: Probabilistic Fault-Weighted ACS Retention
%    Note: for this tandem hexacopter, mean single-fault retention = 1/3
%    (mathematical zonotope identity). PFWAR captures the full distribution
%    weighted by fault probability including double faults.
PFWAR = sum(fault_probs .* retention);

% 2. FII: Fault Isotropy Index (single-motor faults only)
%    FII = std(r_single) / mean(r_single).  Minimum FII = 0 when all
%    single-fault retentions are equal.  FII is geometry-sensitive and
%    identifies the arm layout that gives the most balanced fault impact.
ret_single = retention(1:n);   % first n rows are single-motor faults
mu_r  = mean(ret_single);
sig_r = std(ret_single);
FII   = (mu_r > 0) * (sig_r / mu_r);

% 3. WCFR: Worst-Case Fault Retention (single-motor)
WCFR = min(ret_single);

% 4. Hover Threshold Analysis
%    Compute minimum T_max required for hover feasibility under each
%    single-motor fault.  This characterizes the "motor oversizing budget"
%    needed to guarantee safe recovery from any single failure.
single_fault_loe = fault_list(1:n, :);   % [6×6] single-fault LOE matrix
[T_hover_thresh, T_hover_ratios] = compute_hover_threshold(B, m, g, single_fault_loe);

% 5. Hover margin: how much T_max exceeds the worst-case hover threshold
%    margin_hover = T_max / max(T_hover_thresh) - 1  (≥0 = feasible for all faults)
T_hover_worst  = max(T_hover_thresh);
hover_margin   = T_max / T_hover_worst - 1;   % ≥0 = all faults feasible

% ── Pack output ───────────────────────────────────────────────────────────
metrics.vol_nominal     = vol_nominal;
metrics.vol_fault       = vol_fault;
metrics.retention       = retention;
metrics.PFWAR           = PFWAR;
metrics.FII             = FII;
metrics.WCFR            = WCFR;
metrics.hover_ok        = hover_ok;
metrics.hover_util      = hover_util;
metrics.fault_list      = fault_list;
metrics.fault_labels    = fault_labels;
metrics.fault_probs     = fault_probs;
metrics.T_hover_thresh  = T_hover_thresh;    % [6×1] min T_max per single fault [N]
metrics.T_hover_ratios  = T_hover_ratios;    % [6×1] ratios normalized by m*g/6
metrics.T_hover_worst   = T_hover_worst;     % worst-case threshold [N]
metrics.hover_margin    = hover_margin;      % T_max / T_hover_worst - 1

% Convenience: summary table (single-motor)
metrics.single_retention = ret_single;
metrics.hover_ok_single  = hover_ok(1:n);
end
