function options = normalize_call_options(kind, options)
% NORMALIZE_CALL_OPTIONS  Convert UAMOptions into internal option structs.
%
%   This helper accepts either:
%     - empty / omitted input  -> default UAMOptions()
%     - UAMOptions object      -> converted to the internal struct
%     - struct                 -> passed through unchanged
%
%   The pass-through behavior keeps the framework tolerant to internal
%   calls that already built a normalized struct.

if nargin < 2 || isempty(options)
    options = UAMOptions();
end

if isstruct(options)
    return;
end

if ~isa(options, 'UAMOptions')
    error('normalize_call_options: %s expects a UAMOptions object or struct.', kind);
end

switch lower(kind)
    case 'eval_design'
        options = options.toEvalStruct();

    case 'compare_designs'
        options = options.toCompareStruct();

    case 'sweep_design_space'
        options = options.toSweepStruct();

    case 'pareto_analysis'
        options = options.toParetoStruct();

    otherwise
        error('normalize_call_options: unknown kind ''%s''.', kind);
end
end
