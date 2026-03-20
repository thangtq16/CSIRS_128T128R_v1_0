classdef Format3ConfigBase < nr5g.internal.pucch.CommonConfig ...
        & nr5g.internal.FrequencyHoppingConfig & nr5g.internal.pucch.Formats34Common ...
        & nr5g.internal.pucch.Formats1234Common & nr5g.internal.interlacing.InterlacingConfig
%Format3ConfigBase Class offering properties common between nrPUCCH3Config
%and nrWavegenPUCCH3Config
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
        function obj = Format3ConfigBase(varargin)
            obj = obj@nr5g.internal.pucch.CommonConfig(varargin{:});
            if ~isempty(coder.target) % Allow to be var-size in codegen
                % Get the value of additional var-sized properties from the
                % name-value pairs
                obj.Modulation = nr5g.internal.parseProp('Modulation','QPSK',varargin{:});
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
                    'nr5g:nrPUCCH:InvalidNumSymbolsLongPUCCH',numSymbols,'3');
            end

            % OCCI must be less than SpreadingFactor if spreading is active
            if obj.Interlacing && numel(obj.InterlaceIndex) == 1
                coder.internal.errorIf(obj.OCCI >= obj.SpreadingFactor,...
                    'nr5g:nrPUCCH:InvalidOCCIPUCCH234', obj.OCCI, obj.SpreadingFactor);
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

            % If no interlacing or number of interlaces > 1, hide
            % SpreadingFactor and OCCI
            inac = ~obj.Interlacing || numel(obj.InterlaceIndex)>1;
            inactive = inactive || (any(strcmp(prop,{'SpreadingFactor','OCCI'})) && inac);
        end

        function validateSpreadingFactor(~,sf)
            coder.internal.errorIf(~any(sf == [1 2 4]),'nr5g:nrPUCCH:InvalidSFFormat3',sf)
        end
    end
end