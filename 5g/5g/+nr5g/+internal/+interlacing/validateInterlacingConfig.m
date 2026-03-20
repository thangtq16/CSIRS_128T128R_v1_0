function validateInterlacingConfig(carrier,channel)
%validateInterlacingConfig validate channel RBSetIndex and InterlaceIndex
%   validateInterlacingConfig validates PUSCH or PUCCH channel RBSetIndex
%   and InterlaceIndex against a carrier configuration.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    % We add coder.ignoreConst(true) to these error conditions to force the error checks to be deferred to runtime,
    % since this function should only be called at runtime for an interlacing config.

    % Validate carrier SCS
    coder.internal.errorIf(coder.ignoreConst(true) && ~any(carrier.SubcarrierSpacing == [15 30]),'nr5g:interlacing:InvalidSCS',double(carrier.SubcarrierSpacing));
    
    % RBSetIndex must not exceed the available number of RB sets
    gbsize = nr5g.internal.interlacing.guardBandSize(carrier);
    numGuardBands = size(gbsize,1);
    if numGuardBands > 0
        errcond = max(channel.RBSetIndex) > numGuardBands;
        coder.internal.errorIf(coder.ignoreConst(true) && errcond,'nr5g:interlacing:InvalidRBSetIndex',max(channel.RBSetIndex),numGuardBands);
    end
    
    % InterlaceIndex must not exceed the number of interlaces for the SCS
    maxInterlaceIndex = max(channel.InterlaceIndex);
    M = 10*(15/carrier.SubcarrierSpacing);
    coder.internal.errorIf(coder.ignoreConst(true) && maxInterlaceIndex>M-1,'nr5g:interlacing:InvalidInterlaceIndex',maxInterlaceIndex,M,carrier.SubcarrierSpacing);

end