function [prbset,symbolset,dmrssymbolset,ldash] = initializeResources(carrier,pxsch,nSizeBWP,varargin)
%initializeResources Resource allocation of the physical shared channel
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [PRBSET,SYMBOLSET,DMRSSYMBOLSET,LDASH] = initializeResources(CARRIER,PDSCH,NSIZEBWP)
%   returns the frequency and time allocation of the physical shared
%   channel in terms of physical resource blocks PRBSET, the set of OFDM
%   symbols allocated SYMBOLSET, the OFDM symbol locations of DM-RS signal
%   DMRSSYMBOLSET and the indication of DM-RS double-symbol LDASH, given
%   the input carrier configuration object CARRIER, the physical channel
%   configuration object PDSCH and the size of bandwidth part NSIZEBWP.
%
%   [PRBSET,SYMBOLSET,DMRSSYMBOLSET,LDASH] = initializeResources(...,FTABLE)
%   provides additional control over the output DM-RS symbols, with the
%   input structure FTABLE. The FTABLE contains the fields:
%   ChannelName              - Physical shared channel name
%                              ('PDSCH' (default), 'PUSCH')
%   MappingTypeB             - Flag to indicate if mapping type is set to
%                              'B' or not (0 or 1). Default is the
%                              indication of mapping type B for PDSCH
%   DMRSSymbolSet            - Function handle to generate the DM-RS
%                              symbols within the symbol allocation.
%                              Default is @nr5g.internal.pdsch.lookupPDSCHDMRSSymbols
%   IntraSlotFreqHoppingFlag - Flag to indicate the presence of intra-slot
%                              frequency hopping (Default 0)
%
%   The default values of the fields are used when there is no input
%   FTABLE, implies, for 3 input arguments. For 4 input arguments, all the
%   fields are mandatory.
%
%   Example:
%   % Get the time, frequency allocation of the physical shared channel
%   % with the default settings.
%
%   carrier = nrCarrierConfig;
%   pdsch = nrPDSCHConfig;
%   nSizeBWP = carrier.NSizeGrid;
%   [prbset,symbolset] = nr5g.internal.pxsch.initializeResources(carrier,pdsch,nSizeBWP);

%   Copyright 2019-2024 The MathWorks, Inc.

%#codegen

    % Number of OFDM symbols in a slot
    symbperslot = carrier.SymbolsPerSlot;

    % Set of OFDM symbols allocated for the shared channel
    if ~(isempty(pxsch.SymbolAllocation) || (pxsch.SymbolAllocation(end)==0))
        nPDSCHStart = pxsch.SymbolAllocation(1);
        nPDSCHSym = pxsch.SymbolAllocation(end);
        symbolset = nPDSCHStart:nPDSCHStart+nPDSCHSym-1;
        symbolset = symbolset(symbolset < symbperslot);
    else
        symbolset = zeros(1,0);
    end

    % Get the optional inputs
    if nargin == 3
        ftable.ChannelName = 'PDSCH';
        ftable.MappingTypeB = strcmpi(pxsch.MappingType,'B');
        ftable.DMRSSymbolSet = @nr5g.internal.pdsch.lookupPDSCHDMRSSymbols;
        ftable.IntraSlotFreqHoppingFlag = 0;
    else
        ftable = varargin{1};
    end
    isDownlink = isa(pxsch,'nrPDSCHConfig');

    % DM-RS symbol set
    if ~isempty(pxsch.DMRS.CustomSymbolSet)
        % Custom symbol locations
        tempSymbolSet = zeros(1,symbperslot);
        tempSymbolSet(symbolset+1) = 1;
        dmrsSymInd = pxsch.DMRS.CustomSymbolSet(:);
        dmrsSymbolSet = dmrsSymInd(dmrsSymInd < symbperslot);
        tempSymbolSet(dmrsSymbolSet+1) = tempSymbolSet(dmrsSymbolSet+1) + 1;
        dmrssymbolset = find(tempSymbolSet == 2) - 1;
        ldash = zeros(1,length(dmrssymbolset)); % Treat as single-symbol
        warningId = 'nr5g:nrPXSCH:CustomSymbolSetNoSymbols';
    else
        % Table look up
        [dmrssymbolset,ldash] = ftable.DMRSSymbolSet(symbolset,ftable.MappingTypeB,pxsch.DMRS.DMRSTypeAPosition,pxsch.DMRS.DMRSLength,pxsch.DMRS.DMRSAdditionalPosition,ftable.IntraSlotFreqHoppingFlag);
        warningId = 'nr5g:nrPXSCH:DMRSParametersNoSymbols';
    end
    if isempty(dmrssymbolset) && ~isempty(symbolset)
        str = string(ftable.ChannelName);
        coder.internal.warning(warningId,str);
    end

    % Get allocated PRB set
    prbset = nr5g.internal.allocatedPhysicalResourceBlocks(carrier,pxsch);

    % Compute PRBs when the resource block allocation type is VRB
    if isDownlink && pxsch.VRBToPRBInterleaving && strcmpi(pxsch.PRBSetType,'VRB')
        % Get the RB reference point for the interleaver. If the value of
        % pxsch.DMRS.DMRSReferencePoint is PRB0, the RB reference point is
        % set to 0, assuming that the PDSCH is signaled via CORESET 0, as
        % described in TS 38.211 Section 7.3.1.6.
        rbrefpoint = nr5g.internal.pdsch.getRBReferencePoint(carrier.NStartGrid,pxsch.NStartBWP,pxsch.DMRS.DMRSReferencePoint);
        % Reference PRB order for all the resource blocks in BWP
        mapIndices = nr5g.internal.pdsch.vrbToPRBInterleaver(nSizeBWP,rbrefpoint,double(pxsch.VRBBundleSize));
        % PDSCH VRB-To-PRB interleaving
        prbset = mapIndices(1,prbset+1);
    end

end