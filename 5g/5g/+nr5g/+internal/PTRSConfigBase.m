classdef (Abstract) PTRSConfigBase < comm.internal.ConfigBase
    %PTRSConfigBase Base object for PT-RS configuration object
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.

    % Copyright 2019-2023 The MathWorks, Inc.

    %#codegen

    properties
        % TS 38.211 sections 6.4.1.2 and 7.4.1.2

        %TimeDensity Time density (L_PT-RS)
        %   Specify the time density of PT-RS as a scalar positive integer.
        %   The value must be one of {1, 2, 4}. The default value is 1.
        TimeDensity (1,1) {mustBeMember(TimeDensity, [1 2 4])} = 1;

        %FrequencyDensity Frequency density (K_PT-RS)
        %   Specify the frequency density of PT-RS as a scalar positive
        %   integer. The value must be one of {2, 4}. The default value is 2.
        FrequencyDensity (1,1) {mustBeMember(FrequencyDensity, [2 4])} = 2;

        %REOffset Resource element offset
        %   Specify the subcarrier offset as a character array or a scalar
        %   string. The value must be one of {'00', '01', '10', '11'},
        %   provided by higher-layer parameter resourceElementOffset. The
        %   default value is '00'.
        REOffset = '00';
    end

    properties(Constant, Hidden)
        REOffset_Values = {'00','01','10','11'};
    end

    methods
        function obj = PTRSConfigBase(varargin)
            %PTRSConfigBase Create a PTRSConfigBase object
            %   Set the property values from any name-value pairs input to
            %   the object

            % Call the base class constructor method with all the
            % name-value pairs input
            obj@comm.internal.ConfigBase(varargin{:});
        end

        function obj = set.REOffset(obj,val)
            prop = 'REOffset';
            val = validatestring(val, obj.REOffset_Values,...
                [class(obj) '.' prop], prop);
            obj.(prop) = val;
        end
    end
end