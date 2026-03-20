classdef Format2ConfigBase < nr5g.internal.pucch.CommonConfig ...
        & nr5g.internal.FrequencyHoppingConfig & nr5g.internal.pucch.Formats234Common ...
        & nr5g.internal.pucch.Formats1234Common & nr5g.internal.interlacing.InterlacingConfig
%Format2ConfigBase Class offering properties common between nrPUCCH2Config
%and nrWavegenPUCCH2Config
%
%   Note: This is an internal undocumented class and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    properties(Hidden, SetAccess = 'private')
        SymbolAllocationDefault = [13 1];
    end
    
    methods
        % Constructor
        function obj = Format2ConfigBase(varargin)
            obj = obj@nr5g.internal.pucch.CommonConfig(varargin{:});
            if ~isempty(coder.target) % Allow to be var-size in codegen
                % Get the value of additional var-sized properties from the
                % name-value pairs
                obj.FrequencyHopping = nr5g.internal.parseProp('FrequencyHopping','neither',varargin{:});
            end
        end
    end
    
    methods(Access = public)
        % Validate configuration
        function validateConfig(obj)
            % SymbolAllocation(2) must be 1 or 2, if SymbolAllocation is not empty
            if ~isempty(obj.SymbolAllocation)
                numSymbols = obj.SymbolAllocation(2);
                coder.internal.errorIf(...
                    (numSymbols ~= 0) && ((numSymbols < 1) || (numSymbols > 2)),...
                    'nr5g:nrPUCCH:InvalidNumSymbolsShortPUCCH',numSymbols,'2');
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
            coder.internal.errorIf(~any(sf == [2 4]),'nr5g:nrPUCCH:InvalidSFFormats24',sf)
        end
    end
end