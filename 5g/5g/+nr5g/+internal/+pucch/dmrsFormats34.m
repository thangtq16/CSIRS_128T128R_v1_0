function sym = dmrsFormats34(carrier,pucch)
%dmrsFormats34 DM-RS symbols for PUCCH format 3 and 4
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   SYM = dmrsFormat34(CARRIER,PUCCH) returns the demodulation reference
%   signal (DM-RS) symbols, SYM, of physical uplink control channel format
%   3 or 4, given the carrier configuration CARRIER and physical uplink
%   control channel configuration, PUCCH, for format 3 or 4. CARRIER is a
%   nrCarrierConfig object. PUCCH is a nrPUCCH3Config or a nrPUCCH4Config
%   object.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    if isa(pucch,'nrPUCCH3Config')
        formatPUCCH = 3;
    else
        formatPUCCH = 4;
    end

    % Get the DM-RS OFDM symbol locations. Disable frequency hopping for
    % interlacing  as it is not applicable (TS 38.213 Section 9.2.1)
    interlacing = nr5g.internal.interlacing.isInterlaced(pucch);
    intraSlotFreqHopping = ~interlacing && strcmpi(pucch.FrequencyHopping,'intraSlot');
    ldmrs = nr5g.internal.pucch.dmrsSymbolIndicesFormats34(...
        double(pucch.SymbolAllocation),intraSlotFreqHopping,pucch.AdditionalDMRS);

    % Relative slot number
    nslot = mod(double(carrier.NSlot),carrier.SlotsPerFrame);

    % Hopping identity
    if isempty(pucch.HoppingID)
        nid = double(carrier.NCellID);
    else
        nid = double(pucch.HoppingID(1));
    end

    % Get the cyclic shift index
    [sf,occi] = nr5g.internal.pucch.occConfiguration(pucch,formatPUCCH);
    if ~isempty(sf)
        if sf == 2
            csTable = [0 6];
        else
            csTable = [0 6 3 9];
        end
        initialCS = csTable(1,occi+1);
    else
        initialCS = 0;
    end

    % Get the PUCCH hopping information
    seqCS = 0;
    info = nrPUCCHHoppingInfo(carrier.CyclicPrefix,nslot,nid,pucch.GroupHopping,...
        initialCS,seqCS);

    % Get the number of OFDM symbols allocated for DM-RS in first hop
    numDMRS = length(ldmrs);
    if intraSlotFreqHopping
        nSF0 = floor(numDMRS/2);
    else
        nSF0 = numDMRS;
    end

    % Determine the number of subcarriers allocated
    prbset = nr5g.internal.allocatedPhysicalResourceBlocks(carrier, pucch);
    Msc = numel(prbset)*12;

    lps2 = [];
    if pucch.DMRSUplinkTransformPrecodingR16 && strcmpi(pucch.Modulation,'pi/2-BPSK')
        % Get the DM-RS symbols with low-PAPR type 2 sequence

        % DM-RS scrambling identity
        if isempty(pucch.NID0)
            nid0 = double(carrier.NCellID);
        else
            nid0 = double(pucch.NID0(1));
        end

        symbperslot = carrier.SymbolsPerSlot;
        cinit = getCinit(ldmrs(1:nSF0));
        lps1 = nrLowPAPRS(info.U(1),cinit,Msc);
        if intraSlotFreqHopping
            cinit = getCinit(ldmrs(nSF0+1:end));
            lps2 = nrLowPAPRS(info.U(2),cinit,Msc);
        end
    else
        % Get the DM-RS symbols with low-PAPR type 1 sequence
        lps1 = nrLowPAPRS(info.U(1),info.V(1),info.Alpha(ldmrs(1:nSF0)+1),Msc);
        if intraSlotFreqHopping
            lps2 = nrLowPAPRS(info.U(2),info.V(2),info.Alpha(ldmrs(nSF0+1:end)+1),Msc);
        end
    end
    sym = [lps1 lps2];

    function cinit = getCinit(nsymbol)
        % Compute cinit, as discussed in TS 38.211 Section 6.4.1.3.3.1
        cinit = mod(2^17*(symbperslot*nslot + nsymbol + 1)*(2*nid0 + 1) + 2*nid0,2^31);
    end
end
