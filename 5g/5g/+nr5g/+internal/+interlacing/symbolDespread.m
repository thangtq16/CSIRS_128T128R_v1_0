function sym = symbolDespread(carrier,pucch,symbols)
%symbolDespread Symbol despreading for interlaced PUCCH format 2
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    % Interlaced resource block index
    nIRB = nr5g.internal.interlacing.interlacedResourceBlockSet(carrier,pucch);

    % NIRB-dependent spreading sequence for each PUCCH modulated symbol
    nRE = 8; % Number of RE per PRB available for PUCCH
    sf = double(pucch.SpreadingFactor);
    occi = double(pucch.OCCI);
    numOFDMSymbols = double(pucch.SymbolAllocation(2));
    wn = nr5g.internal.interlacing.interlacedSpreadingSequences(nIRB,nRE,sf,occi,numOFDMSymbols);

    % Align symbol phases and combine
    sym = mean(conj(wn).*reshape(symbols,sf,[]).',2);

end