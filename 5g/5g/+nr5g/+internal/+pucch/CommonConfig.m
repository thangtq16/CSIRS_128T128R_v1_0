classdef (Abstract) CommonConfig < comm.internal.ConfigBase
    %CommonConfig Common configuration object for PUCCH objects
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   CommonConfig properties (configurable):
    %
    %   SymbolAllocation - OFDM symbols allocated for physical shared
    %                      channel within a slot
    %   PRBSet           - PRBs allocated for PUCCH within the BWP

    %   Copyright 2021-2022 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %SymbolAllocation Physical uplink control channel symbol allocation within a slot
        %   Specify the physical uplink control channel symbol allocation
        %   within a slot as a two-element vector of nonnegative integers
        %   representing the starting symbol and symbol length. For formats
        %   0 and 2, the default is [13 1], for formats 1, 3, and 4, the
        %   default is [0 14].
        SymbolAllocation;

        %PRBSet PRBs allocated for physical uplink control channel within the BWP
        %   Specify the PRBs (0-based) allocated for physical uplink
        %   control channel within the BWP as a vector of nonnegative
        %   integers in the range 0...274. The default is 0.
        PRBSet = 0;
    end

    properties(Abstract, Hidden, SetAccess = 'private')
        SymbolAllocationDefault;
    end

    methods

        % Constructor
        function obj = CommonConfig(varargin)
            obj@comm.internal.ConfigBase(varargin{:});

            % Make sure to set correctly the default value for SymbolAllocation
            obj.SymbolAllocation = nr5g.internal.parseProp('SymbolAllocation',obj.SymbolAllocationDefault,varargin{:});
        end

        function obj = set.SymbolAllocation(obj,val)
            prop = 'SymbolAllocation';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[2 2],[1 1]);
            if ~isempty(temp)
                validateattributes(temp,{'numeric'},{'vector','integer','nonnegative','numel',2},[class(obj) '.' prop],prop);
                coder.internal.errorIf((temp(1) + temp(2)) > 14,'nr5g:nrPUCCH:SymAllocationSumExceed',temp(1),temp(2),14);
            end
            obj.(prop) = temp;
        end

        function obj = set.PRBSet(obj,val)
            prop = 'PRBSet';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'vector','integer','nonnegative','<=',274},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end
    end
end
