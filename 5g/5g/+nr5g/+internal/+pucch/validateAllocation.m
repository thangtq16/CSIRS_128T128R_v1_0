function [nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,prbset,symbperslot] = validateAllocation(carrier,pucch)
%validateAllocation Validates the time and frequency allocation
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [NSIZEGRID,NSTARTGRID,NSIZEBWP,NSTARTBWP,PRBSET,SYMBPERSLOT] = validateAllocation(CARRIER,PUCCH)
%   validates the time and frequency allocation of the control channel,
%   provided the inputs, carrier configuration CARRIER and uplink control
%   channel configuration PUCCH. The function provides carrier dimensions
%   (NSIZEGRID, NSTARTGRID), BWP dimensions (NSIZEBWP, NSTARTBWP), the set
%   of unique and sorted resource blocks PRBSET, and the number of
%   OFDM symbols in a slot SYMBPERSLOT.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    % Initialize parameters
    nStartGrid = double(carrier.NStartGrid);
    nSizeGrid = double(carrier.NSizeGrid);
    if isempty(pucch.NStartBWP)
        % If nStartBWP is empty, it is set to the starting resource block
        % of the carrier resource grid
        nStartBWP = nStartGrid;
    else
        nStartBWP = double(pucch.NStartBWP(1));
    end
    if isempty(pucch.NSizeBWP)
        % If nSizeBWP is empty, it is set to the size of carrier resource
        % grid
        nSizeBWP = nSizeGrid;
    else
        nSizeBWP = double(pucch.NSizeBWP(1));
    end
    symbperslot = carrier.SymbolsPerSlot;

    % Validate interdependent parameters
    % BWP start must be greater than or equal to starting resource block of
    % carrier
    coder.internal.errorIf(nStartBWP < nStartGrid,...
        'nr5g:nrPUCCH:InvalidNStartBWP',nStartBWP,nStartGrid);

    % BWP must lie within the limits of carrier
    coder.internal.errorIf((nSizeBWP + nStartBWP)>(nStartGrid + nSizeGrid),...
        'nr5g:nrPUCCH:InvalidBWPLimits',nStartBWP,nSizeBWP,nStartGrid,nSizeGrid);

    % Validate PRBSet, FrequencyHopping and SecondHopStartPRB if
    % interlacing is off
    interlacing = nr5g.internal.interlacing.isInterlaced(pucch);
    if ~interlacing
        % PRB set must lie within the BWP size
        if ~isempty(pucch.PRBSet)
            maxprb = max(reshape(pucch.PRBSet,1,[]));
            coder.internal.errorIf((maxprb >= nSizeBWP),...
                'nr5g:nrPUCCH:InvalidPRBSet',maxprb,nSizeBWP);
        end
        if strcmpi(pucch.FrequencyHopping,'intraSlot') || ...
                (strcmpi(pucch.FrequencyHopping,'interSlot') && (mod(carrier.NSlot,2) == 1))
            maxPRB = max(double(pucch.PRBSet(:)) - min(double(pucch.PRBSet(:))) ...
                + double(pucch.SecondHopStartPRB));
            coder.internal.errorIf(maxPRB >= nSizeBWP,...
                'nr5g:nrPUCCH:InvalidPRBSetHopping',pucch.SecondHopStartPRB,maxPRB,nSizeBWP);
        end
    else
        % Validate interlacing configuration against the carrier and
        % intracell guard bands
        nr5g.internal.interlacing.validateInterlacingConfig(carrier,pucch);
    end

    % Get PRB set
    prbset = nr5g.internal.allocatedPhysicalResourceBlocks(carrier,pucch);

    % Symbol allocation must not exceed the number of symbols in a slot
    if ~isempty(pucch.SymbolAllocation)
        coder.internal.errorIf((pucch.SymbolAllocation(1)+pucch.SymbolAllocation(end)) > symbperslot,...
            'nr5g:nrPUCCH:SymAllocationSumExceed',pucch.SymbolAllocation(1),pucch.SymbolAllocation(end),symbperslot);
    end

end
