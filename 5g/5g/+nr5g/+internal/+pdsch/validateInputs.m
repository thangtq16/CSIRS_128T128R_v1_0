function [nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,symbperslot] = validateInputs(carrier,pdsch)
%validateInputs Validate the inputs
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [NSIZEGRID,NSTARTGRID,NSIZEBWP,NSTARTBWP,SYMBPERSLOT] = validateInputs(CARRIER,PDSCH)
%   validates the inputs carrier configuration object CARRIER and physical
%   downlink shared channel PDSCH. The function also provides the
%   frequency aspects of carrier, bandwidth part and the number of OFDM
%   symbols per slot.

%   Copyright 2019 The MathWorks, Inc.

%#codegen

    % Validate inputs
    coder.internal.errorIf(~(isa(carrier,'nrCarrierConfig') && isscalar(carrier)),'nr5g:nrPXSCH:InvalidCarrierInput');
    coder.internal.errorIf(~(isa(pdsch,'nrPDSCHConfig') && isscalar(pdsch)),'nr5g:nrPDSCH:InvalidPDSCHInput');

    % Initialize parameters
    nStartGrid = double(carrier.NStartGrid);
    nSizeGrid = double(carrier.NSizeGrid);
    if isempty(pdsch.NStartBWP)
        % If nStartBWP is empty, it is set to the default value (i.e.,
        % start of the carrier)
        nStartBWP = nStartGrid;
    else
        nStartBWP = double(pdsch.NStartBWP(1));
    end
    if isempty(pdsch.NSizeBWP)
        % If nSizeBWP is empty, it is set to the default value (i.e.,
        % size of the carrier)
        nSizeBWP = nSizeGrid;
    else
        nSizeBWP = double(pdsch.NSizeBWP(1));
    end
    symbperslot = carrier.SymbolsPerSlot;

    % Validate interdependent parameters
    nr5g.internal.pxsch.validateAllocation(nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,pdsch.PRBSet,pdsch.SymbolAllocation,symbperslot);
    validateConfig(pdsch);
end