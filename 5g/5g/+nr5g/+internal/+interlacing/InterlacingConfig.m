classdef InterlacingConfig
    %InterlacingConfig Common configuration for interlacing of PUCCH and PUSCH
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   InterlacingConfig properties (configurable):
    %
    %   Interlacing    - Enable interlacing
    %   RBSetIndex     - Indices of the RB sets between guard bands
    %   InterlaceIndex - Interlace indices

    %   Copyright 2023 The MathWorks, Inc.

    %#codegen
    
    % Public, tunable properties
    properties

        %Interlacing Enable interlacing
        %   Enable or disable interlaced resource allocation. This property
        %   corresponds to the useInterlacePUCCH-PUSCH higher-layer
        %   parameter. When Interlacing = true, use the properties
        %   RBSetIndex and InterlaceIndex to configure the frequency
        %   resource allocation. When Interlacing = true, the properties
        %   PRBSet, FrequencyHopping, and SecondHopStartPRB are ignored.
        %   The default value is false.
        Interlacing (1,1) logical = false;

        %InterlaceIndex Interlace indices
        %   Specify the interlace indices as a vector of up to M elements
        %   in the range (0...M-1), where M = 10 or M = 5 for 15 kHz or 30
        %   kHz subcarrier spacing, respectively. When Interlacing = true,
        %   the interlace indices determine the frequency resource
        %   allocation and the properties PRBSet, FrequencyHopping, and
        %   SecondHopStartPRB are ignored. The default value is 0.
        InterlaceIndex {mustBeVector, mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThan(InterlaceIndex,10)} = 0;

        %RBSetIndex Index of the RB set between intracell guard bands
        %   Specify the 0-based indices of the RB sets for interlaced
        %   transmissions as a vector of nonnegative integers. This index
        %   corresponds to the RB sets between intracell guard bands. When
        %   Interlacing = true, RB set indices determine the frequency
        %   resource allocation and the properties PRBSet,
        %   FrequencyHopping, and SecondHopStartPRB are ignored. The
        %   default value is 0.
        RBSetIndex {mustBeVector, mustBeNumeric, mustBeInteger, mustBeNonnegative} = 0;

    end

    methods (Access = protected)
        % Controls the conditional display of properties
        function inactive = isInactiveProperty(obj, prop)
            % RBSetIndex and InterlaceIndex inactive if Interlacing = false
            inactive = any(strcmp(prop,{'RBSetIndex','InterlaceIndex'})) && ~obj.Interlacing;
        end
    end

end

