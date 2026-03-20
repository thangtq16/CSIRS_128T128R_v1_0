classdef Format1ConfigBase < nr5g.internal.pucch.CommonConfig ...
        & nr5g.internal.FrequencyHoppingConfig & nr5g.internal.pucch.Formats01Common ...
        & nr5g.internal.pucch.Formats1234Common
%Format1ConfigBase Class offering properties common between nrPUCCH1Config
%and nrWavegenPUCCH1Config
%
%   Note: This is an internal undocumented class and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    properties(Hidden, SetAccess = 'private')
        SymbolAllocationDefault = [0 14];
    end

    methods
        % Constructor
        function obj = Format1ConfigBase(varargin)
            obj = obj@nr5g.internal.pucch.CommonConfig(varargin{:});
            if ~isempty(coder.target) % Allow to be var-size in codegen
                % Get the value of additional var-sized properties from the
                % name-value pairs
                obj.FrequencyHopping = nr5g.internal.parseProp('FrequencyHopping','neither',varargin{:});
                obj.GroupHopping = nr5g.internal.parseProp('GroupHopping','neither',varargin{:});
            end
        end
    end
    
    methods(Access = public)
        % Validate configuration
        function validateConfig(obj)
            % SymbolAllocation(2) must be at least 4, if SymbolAllocation is not empty
            if ~isempty(obj.SymbolAllocation)
                numSymbols = obj.SymbolAllocation(2);
                coder.internal.errorIf(...
                    (numSymbols ~= 0) && (numSymbols < 4),...
                    'nr5g:nrPUCCH:InvalidNumSymbolsLongPUCCH',numSymbols,'1');
            end
        end
    end

    methods (Access = protected)
        % Controls the conditional display of properties
        function inactive = isInactiveProperty(obj, prop)
            % Call base class method to determine the visibility of
            % RBSetIndex and InterlaceIndex properties
            inactive = isInactiveProperty@nr5g.internal.interlacing.InterlacingConfig(obj, prop);

            % If interlacing is on, hide PRBSet, FrequencyHopping, and SecondHopStartPRB
            inactive = inactive || (any(strcmp(prop,{'PRBSet','FrequencyHopping','SecondHopStartPRB'})) && obj.Interlacing);
        end
    end
end