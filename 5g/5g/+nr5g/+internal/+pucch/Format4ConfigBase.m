classdef Format4ConfigBase < nr5g.internal.pucch.CommonConfig ...
        & nr5g.internal.FrequencyHoppingConfig & nr5g.internal.pucch.Formats34Common ...
        & nr5g.internal.pucch.Formats1234Common
%Format4ConfigBase Class offering properties common between nrPUCCH4Config
%and nrWavegenPUCCH4Config
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
        function obj = Format4ConfigBase(varargin)
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
                    'nr5g:nrPUCCH:InvalidNumSymbolsLongPUCCH',numSymbols,'4');
            end
                
            % OCCI must be less than SpreadingFactor
            coder.internal.errorIf(obj.OCCI >= obj.SpreadingFactor,...
                    'nr5g:nrPUCCH:InvalidOCCIPUCCH234', obj.OCCI, obj.SpreadingFactor);
        end
    end

    methods (Access = protected)
        function validateSpreadingFactor(~,sf)
            coder.internal.errorIf(~any(sf == [2 4]),'nr5g:nrPUCCH:InvalidSFFormats24',sf)
        end
    end
end