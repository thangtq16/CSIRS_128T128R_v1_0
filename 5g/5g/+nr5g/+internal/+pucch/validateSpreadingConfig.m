function validateSpreadingConfig(seqLength,modulation,Mrb,nRE,sf,formatPUCCH)
%validateSpreadingConfig Validate spreading configuration for PUCCH 2, 3 and 4
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    % Modulation order
    Q = nr5g.internal.getQm(modulation);

    SF = sf(1);
    Msc = Mrb*nRE;
    nSymbols = SF * seqLength/Msc;
    switch formatPUCCH
        case 2
            coder.internal.errorIf(nSymbols ~= fix(nSymbols),'nr5g:nrPUCCH:InvalidNumOfModSymbolsF2',seqLength*Q,SF,Mrb);
        case 3
            coder.internal.errorIf(nSymbols ~= fix(nSymbols),'nr5g:nrPUCCH:InvalidNumOfModSymbolsF3',seqLength*Q,modulation,SF,Mrb);
        case 4
            coder.internal.errorIf(nSymbols ~= fix(nSymbols),'nr5g:nrPUCCH:InvalidNumOfModSymbolsF4',seqLength*Q,modulation,SF,Mrb);
    end

end