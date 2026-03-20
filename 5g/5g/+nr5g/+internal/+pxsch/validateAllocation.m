function validateAllocation(nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,prbset,symbolAllocation,nslotsymb)
%validateAllocation Validates the time and frequency allocation
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   nr5g.internal.pxsch.validateAllocation(NSIZEGRID,NSTARTGRID,NSIZEBWP,NSTARTBWP,PRBSET,SYMBOLALLOCATION,NSLOTSYMB)
%   validates the time and frequency allocation of the shared channel,
%   provided the inputs, carrier dimensions (NSIZEGRID, NSTARTGRID), BWP
%   dimensions (NSIZEBWP, NSTARTBWP), the set of physical resource blocks
%   PRBSET, the symbol allocation SYMBOLALLOCATION and the number of OFDM
%   symbols in a slot NSLOTSYMB.

%   Copyright 2019 The MathWorks, Inc.

%#codegen

    % BWP start must be greater than or equal to starting resource block of
    % carrier
    coder.internal.errorIf(nStartBWP < nStartGrid,...
        'nr5g:nrPXSCH:InvalidNStartBWP',nStartBWP,nStartGrid);

    % BWP must lie within the limits of carrier
    coder.internal.errorIf((nSizeBWP + nStartBWP)>(nStartGrid + nSizeGrid),...
        'nr5g:nrPXSCH:InvalidBWPLimits',nStartBWP,nSizeBWP,nStartGrid,nSizeGrid);

    % PRB set must lie within the BWP size
    if ~isempty(prbset)
        maxprb = max(reshape(prbset,1,[]));
        coder.internal.errorIf((maxprb >= nSizeBWP),...
            'nr5g:nrPXSCH:InvalidPRBSet',maxprb,nSizeBWP);
    end

    % Symbol allocation must not exceed the number of symbols in a slot
    if ~isempty(symbolAllocation)
        coder.internal.errorIf(any((symbolAllocation(1)+symbolAllocation(end)) > nslotsymb),...
            'nr5g:nrPXSCH:InvalidSymbolAllocation',symbolAllocation(1),symbolAllocation(end),nslotsymb);
    end

end