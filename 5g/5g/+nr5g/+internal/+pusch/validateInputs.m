function [nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,symbperslot,freqHopping] = validateInputs(carrier,pusch)
%validateInputs Validate the PUSCH inputs
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [NSIZEGRID,NSTARTGRID,NSIZEBWP,NSTARTBWP,SYMBPERSLOT,FREQHOPPING] = validateInputs(CARRIER,PUSCH)
%   validates the inputs carrier configuration object CARRIER and physical
%   uplink shared channel configuration PUSCH. The function also provides
%   the frequency aspects of carrier, bandwidth part, number of OFDM
%   symbols per slot, and frequency hopping configuration.

%   Copyright 2019-2023 The MathWorks, Inc.

%#codegen

    % Validate inputs
    coder.internal.errorIf(~(isa(carrier,'nrCarrierConfig') && isscalar(carrier)),'nr5g:nrPXSCH:InvalidCarrierInput');
    coder.internal.errorIf(~(isa(pusch,'nrPUSCHConfig') && isscalar(pusch)),'nr5g:nrPUSCH:InvalidPUSCHInput');

    % Initialize parameters
    nStartGrid = double(carrier.NStartGrid);
    nSizeGrid = double(carrier.NSizeGrid);
    if isempty(pusch.NStartBWP)
        % If nStartBWP is empty, it is set to the starting resource block
        % of the carrier resource grid
        nStartBWP = nStartGrid;
    else
        nStartBWP = double(pusch.NStartBWP(1));
    end
    if isempty(pusch.NSizeBWP)
        % If nSizeBWP is empty, it is set to the size of carrier resource
        % grid
        nSizeBWP = nSizeGrid;
    else
        nSizeBWP = double(pusch.NSizeBWP(1));
    end
    symbperslot = carrier.SymbolsPerSlot;
    
    if pusch.Interlacing
        % Validate interlacing configuration against the carrier and
        % intracell guard bands and other allocation parameters
        nr5g.internal.interlacing.validateInterlacingConfig(carrier,pusch);
        nr5g.internal.pxsch.validateAllocation(nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,[],pusch.SymbolAllocation,symbperslot);
        freqHopping = 'neither';
    else
        % Validate interdependent parameters
        nr5g.internal.pxsch.validateAllocation(nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,pusch.PRBSet,pusch.SymbolAllocation,symbperslot);
        if strcmpi(pusch.FrequencyHopping,'intraSlot') || ...
                (strcmpi(pusch.FrequencyHopping,'interSlot') && (mod(carrier.NSlot,2) == 1))
            maxPRB = max(double(pusch.PRBSet(:))-min(double(pusch.PRBSet(:)))+double(pusch.SecondHopStartPRB));
            coder.internal.errorIf(maxPRB >= nSizeBWP,'nr5g:nrPUSCH:InvalidPRBSetHopping',pusch.SecondHopStartPRB,maxPRB,nSizeBWP);
        end
        freqHopping = pusch.FrequencyHopping;
    end

    validateConfig(pusch);
end