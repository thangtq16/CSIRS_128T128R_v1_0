classdef FrequencyHoppingConfig
    %FrequencyHoppingConfig Common configuration object for frequency hopping
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   FrequencyHoppingConfig properties (configurable):
    %
    %   FrequencyHopping  - Frequency hopping configuration
    %                       ('neither' (default), 'intraSlot', 'interSlot')
    %   SecondHopStartPRB - Starting PRB of second hop relative to the BWP
    %                       (0...274) (default 1)

    %   Copyright 2020-2023 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties

        %FrequencyHopping Frequency hopping configuration
        %   Specify the frequency hopping configuration as one of
        %   {'intraSlot', 'interSlot', 'neither'}. The default value is
        %   'neither'.
        FrequencyHopping = 'neither';

        %SecondHopStartPRB Starting PRB index of second hop relative to the
        %bandwidth part (BWP)
        %   Specify the starting PRB of second hop relative to the BWP as a
        %   scalar nonnegative integer. It must be in range 0...274. The
        %   default value is 1.
        SecondHopStartPRB (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(SecondHopStartPRB, 274)} = 1;

    end

    properties (Constant, Hidden)
        FrequencyHopping_Values  = {'intraSlot', 'interSlot', 'neither'};
    end

    methods

        % Self-validate and set properties
        function obj = set.FrequencyHopping(obj,val)
            prop = 'FrequencyHopping';
            val = validatestring(val,obj.FrequencyHopping_Values,...
                [class(obj) '.' prop],prop);
            obj.(prop) = '';
            obj.(prop) = val;
        end

    end

end
