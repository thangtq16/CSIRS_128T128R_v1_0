classdef nrPDSCHReservedConfig < comm.internal.ConfigBase
%nrPDSCHReservedConfig PDSCH reserved PRB configuration object
    %   RESERVEDPRB = nrPDSCHReservedConfig creates a configuration object
    %   to configure the reserved physical resource block (PRB) pattern for
    %   physical downlink shared channel (PDSCH), as described in TS 38.214
    %   Section 5.1.4.1. By default, the object configures the empty
    %   reserved PRB pattern.
    %
    %   RESERVEDPRB = nrPDSCHReservedConfig(Name,Value) creates a PDSCH
    %   reserved PRB configuration object RESERVEDPRB with the specified
    %   property Name set to the specified Value. You can specify
    %   additional name-value arguments in any order as
    %   (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPDSCHReservedConfig properties (configurable):
    %
    %   PRBSet    - Reserved PRB indices within the bandwidth part (BWP)
    %               (0-based) (default [])
    %   SymbolSet - OFDM symbols associated with reserved PRBs (default [])
    %   Period    - Total number of slots in the pattern period (default [])
    %
    %   % Example 1:
    %   % Create a PDSCH reserved PRB configuration object with default
    %   % properties.
    %
    %   reservedPRB = nrPDSCHReservedConfig;
    %
    %   % Example 2:
    %   % Create a PDSCH reserved PRB configuration object that configures
    %   % the reserved PRBs from 0 to 15, reserved symbols from 0 to 3, and
    %   % pattern period as 5 slots.
    %
    %   reservedPRB = nrPDSCHReservedConfig;
    %   reservedPRB.PRBSet = (0:15);
    %   reservedPRB.SymbolSet = (0:3);
    %   reservedPRB.Period = 5;
    %
    %   See also nrPDSCHConfig.

    %   Copyright 2019-2022 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %PRBSet Reserved PRB indices within BWP
        %   Specify the reserved PRB indices (0-based) in BWP as a vector
        %   of nonnegative integers. The default value is [].
        PRBSet = [];

        %SymbolSet OFDM symbols associated with reserved PRBs
        %   Specify the OFDM symbols associated with reserved PRBs spanning
        %   over one or more slots as a vector of nonnegative integers. The
        %   symbols that form the time-domain locations of the reserved
        %   pattern can be greater than 13 and therefore cover multiple
        %   slots. The default value is [].
        SymbolSet = [];

        %Period Total number of slots in the pattern period
        %   Specify the total number of slots in the pattern period as a
        %   scalar positive integer. The overall OFDM symbols pattern
        %   provided by 'SymbolSet' property will repeat itself for every
        %   'Period' slots. If this field is empty then the pattern will
        %   not cyclically repeat itself. The default value is [].
        Period = [];

    end

    methods

        % Constructor
        function obj = nrPDSCHReservedConfig(varargin)
            obj@comm.internal.ConfigBase(varargin{:});
            if ~isempty(coder.target) % Allow PRBSet and SymbolSet to be var-size in codegen
                obj.PRBSet = nr5g.internal.parseProp('PRBSet',[],varargin{:});
                obj.SymbolSet = nr5g.internal.parseProp('SymbolSet',[],varargin{:});
                obj.Period = nr5g.internal.parseProp('Period',[],varargin{:});
            end
        end

        % Self-validate and set properties
        function obj = set.PRBSet(obj,val)
            prop = 'PRBSet';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'vector','integer','nonnegative'},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end

        function obj = set.SymbolSet(obj,val)
            prop = 'SymbolSet';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'vector','integer','nonnegative'},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end

        function obj = set.Period(obj,val)
            prop = 'Period';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'scalar','integer','positive'},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end
    end

end
