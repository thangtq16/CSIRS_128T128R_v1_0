classdef Formats234Common
    %Formats234Common Common configuration object for PUCCH formats 2, 3, and 4
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   Formats234Common properties (configurable):
    %
    %   SpreadingFactor - Spreading factor (1,2,4) (default 2)
    %   NID  - Data scrambling identity (0...1023) (default [])
    %   RNTI - Radio network temporary identifier (0...65535) (default 1)
    %   NID0 - DM-RS scrambling identity (0...65535) (default [])

    %   Copyright 2020-2023 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties

        %SpreadingFactor Spreading factor
        %   Specify the spreading factor as 2 or 4. The value 1 is also
        %   supported for format 3. The default value is 2. The spreading
        %   factor is only applicable for format 2 and 3 when
        %   InterlaceIndex is a scalar.
        SpreadingFactor (1,1) = 2;

        %NID Data scrambling identity
        %   Specify the data scrambling identity as a scalar nonnegative
        %   integer. The value must be in the range 0...1023. It is the
        %   dataScramblingIdentityPUSCH (0...1023), if configured, else it
        %   is the physical layer cell identity (0...1007). Use empty ([])
        %   to make this property equal to the NCellID property of the
        %   carrier configuration object. The default value is [].
        NID = [];

        %RNTI Radio network temporary identifier
        %   Specify the radio network temporary identifier (RNTI) as a
        %   scalar integer in range 0...65535. The default value is 1.
        RNTI (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(RNTI, 65535)} = 1;

        %NID0 Scrambling identity for DM-RS
        %   Specify the scrambling identity for demodulation reference
        %   signal (DM-RS) as a scalar nonnegative integer in the range
        %   0...65535. NID0 is the scramblingID0 (0...65535), if
        %   configured. Otherwise, NID0 is the physical layer cell identity
        %   (0...1007). To use the NCellID property of the carrier
        %   configuration object as the scrambling identity for DM-RS, set
        %   NID0 to empty ([]). The default value is [].
        NID0 = [];

    end

    methods

        % Self-validate and set properties
        function obj = set.SpreadingFactor(obj,val)
            validateattributes(val,{'numeric'},{'scalar','integer','nonnegative'});
            validateSpreadingFactor(obj,val);
            obj.SpreadingFactor = val;
        end

        function obj = set.NID(obj,val)
            prop = 'NID';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},...
                    {'scalar','integer','nonnegative','<=',1023},...
                    [class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end

        function obj = set.NID0(obj,val)
            prop = 'NID0';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},...
                    {'scalar','integer','nonnegative','<=',65535},...
                    [class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end

    end

    methods (Access = protected, Abstract)
        validateSpreadingFactor();
    end

end
