classdef DMRSPowerCommon
    %DMRSPowerCommon Common wavegen configuration object for PXSCH, PDCCH and PUCCH formats 1,2,3,4
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   DMRSPowerCommon properties (configurable):
    %
    %   DMRSPower - Power scaling of the DM-RS in dB (default 0)

    %   Copyright 2021-2023 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %DMRSPower Power scaling of the DM-RS in dB
        % Specify DMRSPower in dB as a real scalar. The power of DM-RS
        % within the physical channel is scaled within the 5G waveform
        % according to this value. This scaling is additional to the
        % channel-wide power scaling determined by the Power property. The
        % default is 0 dB.
        DMRSPower (1,1) {mustBeNumeric, mustBeReal, mustBeFinite} = 0;
    end
end
