classdef UAMOptions
    % UAMOPTIONS  Single public option carrier for framework call-level behavior.
    %
    %   opt = UAMOptions('Mode', 'acs');
    %   opt = UAMOptions(cfg, 'Mode', 'full', 'Fault', [1;0;0;0;0;0]);
    %
    %   Properties
    %     Config       - mdo_config struct (sets all defaults when provided)
    %     Mode         - 'acs' | 'sim' | 'full'
    %     FaultConfig  - struct with include_double, p_motor
    %     SimConfig    - sim config struct (t_end, t_fault, dt, loe_vec, scenario, ...)
    %     Fault        - [6x1] LOE vector (sets SimConfig.loe_vec)
    %     ObjectiveSet - 'stage1' | 'stage2' (shorthand; overridden by Objectives)
    %     Objectives   - struct with .names and .weights
    %     Model        - struct with model flags (e.g. use_vehicle_model)
    %     Plot         - logical
    %     Verbose      - logical
    %     Labels       - cell array of strings (for pareto axis labels)
    %     Weights      - numeric vector (for pareto knee weighting)

    properties
        Config       = struct()
        Mode         = ''
        FaultConfig  = struct()
        SimConfig    = struct()
        Fault        = []
        ObjectiveSet = ''
        Objectives   = struct()
        Model        = struct()
        Plot         = []
        Verbose      = []
        Labels       = {}
        Weights      = []
    end

    methods
        function obj = UAMOptions(varargin)
            if nargin == 0
                return;
            end

            start_idx = 1;
            if isstruct(varargin{1}) && isfield(varargin{1}, 'vars') && isfield(varargin{1}, 'opt')
                obj.Config = varargin{1};
                start_idx = 2;
            end

            if mod(nargin - start_idx + 1, 2) ~= 0
                error('UAMOptions: use cfg followed by name/value pairs, or name/value pairs only.');
            end

            for k = start_idx:2:nargin
                name  = lower(string(varargin{k}));
                value = varargin{k+1};
                switch name
                    case "mode"
                        obj.Mode = char(value);
                    case "faultconfig"
                        obj.FaultConfig = value;
                    case "simconfig"
                        obj.SimConfig = value;
                    case "fault"
                        obj.Fault = value;
                    case "objectiveset"
                        obj.ObjectiveSet = char(value);
                    case "objectives"
                        obj.Objectives = value;
                    case "model"
                        obj.Model = value;
                    case "plot"
                        obj.Plot = logical(value);
                    case "verbose"
                        obj.Verbose = logical(value);
                    case "labels"
                        obj.Labels = value;
                    case "weights"
                        obj.Weights = value;
                    otherwise
                        error('UAMOptions: unknown property ''%s''.', char(string(name)));
                end
            end
        end

        function s = toEvalStruct(obj)
            s = cfg_defaults('eval_design', obj.Config);
            if ~isempty(obj.Mode),            s.mode = obj.Mode; end
            if has_fields(obj.FaultConfig),   s.fault_config = obj.FaultConfig; end
            if has_fields(obj.SimConfig),     s.sim_config = obj.SimConfig; end
            if ~isempty(obj.Fault)
                s.loe_for_sim = obj.Fault;
                if ~has_fields(s.sim_config), s.sim_config = struct(); end
                s.sim_config.loe_vec = obj.Fault;
            end
            if has_fields(obj.Objectives)
                s.objectives = obj.Objectives;
            elseif ~isempty(obj.ObjectiveSet)
                s.objectives = objective_set_struct_local(obj.ObjectiveSet, obj.Config);
            end
            if has_fields(obj.Model),   s.model = obj.Model; end
            if ~isempty(obj.Verbose),   s.verbose = obj.Verbose; end
        end

        function s = toCompareStruct(obj)
            s = cfg_defaults('compare_designs', obj.Config);
            if ~isempty(obj.Mode),        s.eval_mode = obj.Mode; end
            if has_fields(obj.SimConfig), s.sim_config = obj.SimConfig; end
            if ~isempty(obj.Fault)
                s.sim_loe = obj.Fault;
                if ~has_fields(s.sim_config), s.sim_config = struct(); end
                s.sim_config.loe_vec = obj.Fault;
            end
            if has_fields(obj.Objectives)
                s.objectives = obj.Objectives;
            elseif ~isempty(obj.ObjectiveSet)
                s.objectives = objective_set_struct_local(obj.ObjectiveSet, obj.Config);
            end
            if has_fields(obj.Model),   s.model = obj.Model; end
            if ~isempty(obj.Verbose),   s.verbose = obj.Verbose; end
            if ~isempty(obj.Plot),      s.plot = obj.Plot; end
        end

        function s = toSweepStruct(obj)
            s = cfg_defaults('sweep_design_space', obj.Config);
            if ~isempty(obj.Mode),        s.eval_mode = obj.Mode; end
            if has_fields(obj.SimConfig), s.sim_config = obj.SimConfig; end
            if has_fields(obj.Objectives)
                s.objectives = obj.Objectives;
            elseif ~isempty(obj.ObjectiveSet)
                s.objectives = objective_set_struct_local(obj.ObjectiveSet, obj.Config);
            end
            if has_fields(obj.Model),   s.model = obj.Model; end
            if ~isempty(obj.Verbose),   s.verbose = obj.Verbose; end
        end

        function s = toParetoStruct(obj)
            s = cfg_defaults('pareto_analysis', obj.Config);
            if ~isempty(obj.Plot),    s.plot = obj.Plot; end
            if ~isempty(obj.Labels),  s.f_labels = obj.Labels; end
            if ~isempty(obj.Weights), s.weights = obj.Weights; end
        end
    end
end

function tf = has_fields(s)
tf = isstruct(s) && ~isempty(fieldnames(s));
end

function s = cfg_defaults(kind, cfg)
s = struct();
if isempty(cfg) || ~isstruct(cfg) || isempty(fieldnames(cfg))
    return;
end

switch lower(kind)
    case 'eval_design'
        s.mode       = cfg.eval.mode;
        s.verbose    = false;
        s.fault_config = cfg.fault;
        s.sim_config = cfg.sim;
        if isfield(cfg, 'model'), s.model = cfg.model; end
        if strcmpi(cfg.eval.mode, 'sim')
            s.objectives = objective_set_struct_local('stage2', cfg);
        else
            s.objectives = objective_set_struct_local('stage1', cfg);
        end

    case 'compare_designs'
        s.eval_mode  = cfg.eval.mode;
        s.sim_loe    = cfg.sim.loe_vec;
        s.sim_config = cfg.sim;
        s.plot       = true;
        s.verbose    = true;
        if strcmpi(cfg.eval.mode, 'sim')
            s.objectives = objective_set_struct_local('stage2', cfg);
        else
            s.objectives = objective_set_struct_local('stage1', cfg);
        end
        if isfield(cfg, 'model'), s.model = cfg.model; end

    case 'sweep_design_space'
        s.eval_mode  = cfg.eval.mode;
        s.verbose    = false;
        s.objectives = objective_set_struct_local('stage1', cfg);
        if isfield(cfg, 'model'), s.model = cfg.model; end

    case 'pareto_analysis'
        s.plot = true;
        if isfield(cfg, 'objectives') && isfield(cfg.objectives, 'stage1')
            if isfield(cfg.objectives.stage1, 'moo_names')
                names = cfg.objectives.stage1.moo_names;
            else
                names = cfg.objectives.stage1.names;
            end
            s.f_labels = strcat('J_{', names, '}');
        end
end
end

function obj = objective_set_struct_local(name, cfg)
if nargin < 2
    cfg = struct();
end
switch lower(char(name))
    case 'stage2'
        if isfield(cfg, 'objectives') && isfield(cfg.objectives, 'stage2')
            obj = cfg.objectives.stage2;
        else
            obj = struct('names', {{'mission'}}, 'weights', 1.0);
        end
    otherwise
        if isfield(cfg, 'objectives') && isfield(cfg.objectives, 'stage1')
            obj = cfg.objectives.stage1;
        else
            obj = struct('names', {{'mass','power','fault_thrust','fault_alloc','hover_nom'}}, ...
                'weights', [0.20, 0.20, 0.25, 0.25, 0.10]);
        end
end
end
