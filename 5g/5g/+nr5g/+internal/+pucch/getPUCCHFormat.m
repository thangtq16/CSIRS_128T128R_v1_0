function formatPUCCH = getPUCCHFormat(pucch)
%getPUCCHFormat get PUCCH format from class
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    
    % Get the configured format of physical uplink control channel
    expectedPUCCH = {'nrPUCCH0Config','nrWavegenPUCCH0Config';...
                     'nrPUCCH1Config','nrWavegenPUCCH1Config';...
                     'nrPUCCH2Config','nrWavegenPUCCH2Config';...
                     'nrPUCCH3Config','nrWavegenPUCCH3Config';...
                     'nrPUCCH4Config','nrWavegenPUCCH4Config'};
    classPUCCH = class(pucch);
    [formatPUCCH,~] = find(strcmpi(classPUCCH,expectedPUCCH));

    formatPUCCH = formatPUCCH - 1;

end