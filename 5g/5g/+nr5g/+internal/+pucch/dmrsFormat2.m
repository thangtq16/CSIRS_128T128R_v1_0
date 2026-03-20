function sym = dmrsFormat2(carrier,pucch)
%dmrsFormat2 DM-RS symbols for PUCCH format 2
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   SYM = dmrsFormat2(CARRIER,PUCCH) returns the demodulation reference
%   signal (DM-RS) symbols, SYM, of physical uplink control channel format 2,
%   given the carrier configuration CARRIER and physical uplink control
%   channel configuration for format 2, PUCCH. CARRIER is a nrCarrierConfig
%   object. PUCCH is a nrPUCCH2Config object.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    % OFDM symbol locations
    symAllocation = double(pucch.SymbolAllocation);
    lastPUCCHSym = symAllocation(1) + symAllocation(2) - 1;
    nsym = symAllocation(1):lastPUCCHSym;

    % Relative slot number
    nslot = mod(double(carrier.NSlot),carrier.SlotsPerFrame);

    % DM-RS scrambling identity
    if isempty(pucch.NID0)
        nid0 = double(carrier.NCellID);
    else
        nid0 = double(pucch.NID0(1));
    end

    % Get the resource block offset
    if isempty(pucch.NStartBWP)
        nStartBWP = double(carrier.NStartGrid);
    else
        nStartBWP = double(pucch.NStartBWP(1));
    end

    % If interlacing is enabled, calculate the spreading sequence for each
    % interlaced RB (wn). The index n of wn depends on the OCCI and the
    % index of the RB in the interlace. Each row corresponds to 1 IRB and
    % each column to a subcarrier. The DM-RS symbols will be spread along
    % SF subcarriers.
    interlacing = nr5g.internal.interlacing.isInterlaced(pucch);
    if interlacing 
        % Calculate physical resource blocks in the interlace(s)
        [nIRB,prbset] = nr5g.internal.interlacing.interlacedResourceBlockSet(carrier,pucch);
        freqHopping = 'neither';
    else
        nIRB = [];
        prbset = unique(reshape(double(pucch.PRBSet),1,[]));
        freqHopping = pucch.FrequencyHopping;
    end

    spreading = interlacing && numel(pucch.InterlaceIndex)==1;
    if spreading
        
        % Calculate the spreading sequence for each interlaced RB (wn). The
        % index n of wn depends on the OCCI and the index of the RB in the
        % interlace. Each row corresponds to 1 IRB and each column to a
        % subcarrier. The QPSK modulated symbols will be spread along SF
        % subcarriers.
        nRE = 4;
        numOFDMSymbols = 1;
        [sf,occi] = nr5g.internal.pucch.occConfiguration(pucch,2);
        wn = nr5g.internal.interlacing.interlacedSpreadingSequences(nIRB,nRE,sf,occi,numOFDMSymbols);

    else

        % No spreading required. Set spreading sequence to ones for
        % uniformity in the process below.
        nRE = 4;
        sf = 1;
        wn = ones(length(prbset)*nRE,1);

    end

    % Get the set of physical resource blocks for each hop depending on
    % frequency hopping configuration
    prbsetHop = nr5g.internal.prbSetTwoHops(prbset,freqHopping,pucch.SecondHopStartPRB,nslot);

    % Get DM-RS symbols for first OFDM symbol
    dmrs = nr5g.internal.prbsDMRSSequence(struct('NIDNSCID',nid0,'NSCID',0),...
        nRE/sf(1),prbsetHop(1,:),nStartBWP,nslot,nsym(1),carrier.SymbolsPerSlot);

    % Spread DM-RS sequence for the first OFDM symbol
    dmrs1 = dmrs.*wn;

    if length(nsym) == 2

        % Get DM-RS symbols for second OFDM symbol and apply spreading sequence
        dmrs = nr5g.internal.prbsDMRSSequence(struct('NIDNSCID',nid0,'NSCID',0),...
            nRE/sf(1),prbsetHop(2,:),nStartBWP,nslot,nsym(2),carrier.SymbolsPerSlot);

        % Spread DM-RS sequence for the second OFDM symbol
        dmrs2 = dmrs.*wn;

        % DM-RS symbols in each OFDM symbol
        sym = [dmrs1 dmrs2];

    else

        sym = dmrs1;

    end

end
