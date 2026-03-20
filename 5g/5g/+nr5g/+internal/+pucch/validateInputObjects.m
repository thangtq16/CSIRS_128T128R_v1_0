function [formatPUCCH,interlacing,freqHopping] = validateInputObjects(carrier,pucch)
%validateInputObjects Validate the input configuration objects
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [FORMATPUCCH,INTERLACING,FREQHOPPING] = validateInputObjects(CARRIER,PUCCH)
%   validates the inputs carrier configuration CARRIER and uplink control
%   channel configuration PUCCH. CARRIER must be a scalar, nrCarrierConfig
%   object. PUCCH must be a scalar, and one of nrPUCCH0Config,
%   nrPUCCH1Config, nrPUCCH2Config, nrPUCCH3Config, or nrPUCCH4Config
%   object. The function also provides the PUCCH format, FORMATPUCCH,
%   INTERLACING flag, and frequency hopping configuration FREQHOPPING.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    % Validate carrier input
    coder.internal.errorIf(~(isa(carrier,'nrCarrierConfig') && isscalar(carrier)),...
        'nr5g:nrPUCCH:InvalidCarrierInput');

    % Validate pucch input
    expectedPUCCH = {'nrPUCCH0Config','nrPUCCH1Config','nrPUCCH2Config',...
        'nrPUCCH3Config','nrPUCCH4Config'};
    pucchValidFlag = isa(pucch,expectedPUCCH{1}) || isa(pucch,expectedPUCCH{2}) || ...
        isa(pucch,expectedPUCCH{3}) || isa(pucch,expectedPUCCH{4}) || isa(pucch,expectedPUCCH{5});
    coder.internal.errorIf(~(pucchValidFlag && isscalar(pucch)),...
        'nr5g:nrPUCCH:InvalidPUCCHInput');
    validateConfig(pucch);

    % Get the configured format of physical uplink control channel
    formatPUCCH = nr5g.internal.pucch.getPUCCHFormat(pucch);

    % Determine if interlacing is applicable and active
    interlacing = nr5g.internal.interlacing.isInterlaced(pucch);

    if interlacing
        freqHopping = 'neither';
    else
        freqHopping = pucch.FrequencyHopping;
    end

end
