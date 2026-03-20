function validateInputs(carrier,pdcch,fcnName)
%validateInputs Validate inputs for PDCCH functions
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%   See also nrPDCCHResources, nrPDCCHSpace.

%  Copyright 2019-2021 The MathWorks, Inc.

%#codegen

    validateattributes(carrier,{'nrCarrierConfig'},{'scalar'},fcnName);
    validateattributes(pdcch,{'nrPDCCHConfig'},{'scalar'},fcnName);

    if strcmp(fcnName, 'nrPDCCHResources')
        validateConfig(pdcch,'resources');
    else    % nrPDCCHSpace
        validateConfig(pdcch,'space');
    end

    % Check carrier and BWP parameters
    coder.internal.errorIf(carrier.NSizeGrid<pdcch.NSizeBWP, ...
        'nr5g:nrPDCCHResources:InvBWPSize',pdcch.NSizeBWP,carrier.NSizeGrid);
    coder.internal.errorIf(carrier.NStartGrid>pdcch.NStartBWP, ...
        'nr5g:nrPDCCHResources:InvCarrierBWPStarts',pdcch.NStartBWP, ...
        carrier.NStartGrid);
    cband = uint32(carrier.NStartGrid)+uint32(carrier.NSizeGrid);
    coder.internal.errorIf(cband <= ...
        pdcch.NStartBWP,'nr5g:nrPDCCHResources:InvBWPStart', ...
        pdcch.NStartBWP,cband);
    bwpBand = uint32(pdcch.NStartBWP)+uint32(pdcch.NSizeBWP);
    coder.internal.errorIf(cband < bwpBand, ...
        'nr5g:nrPDCCHResources:InvBWPwrtCarrier',bwpBand,cband);
    
    % Check first symbol and CORESET Duration are within a slot for
    % extended cyclic prefix
    if strcmpi(carrier.CyclicPrefix,'extended')
        firstSymLoc = uint32(pdcch.SearchSpace.StartSymbolWithinSlot); % 0-based
        coder.internal.errorIf((firstSymLoc+uint32(pdcch.CORESET.Duration) >  ...
        carrier.SymbolsPerSlot),'nr5g:nrPDCCHResources:InvCORESETinSlotECP', ...
        pdcch.CORESET.Duration,firstSymLoc);
    end

end
