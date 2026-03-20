function sym = dmrsFormat1(carrier,pucch)
%dmrsFormat1 DM-RS symbols for PUCCH format 1
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   SYM = dmrsFormat1(CARRIER,PUCCH) returns the demodulation reference
%   signal (DM-RS) symbols, SYM, of physical uplink control channel format 1,
%   given the carrier configuration CARRIER and physical uplink control
%   channel configuration for format 1, PUCCH. CARRIER is a nrCarrierConfig
%   object. PUCCH is a nrPUCCH1Config object.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    % Initialize parameters
    occi = double(pucch.OCCI);
    if isempty(pucch.HoppingID)
        nid = double(carrier.NCellID);
    else
        nid = double(pucch.HoppingID(1));
    end
    % Relative slot number
    nslot = mod(double(carrier.NSlot),carrier.SlotsPerFrame);

    interlacing = pucch.Interlacing;
    if interlacing
        [nIRB,~,Mrb] = nr5g.internal.interlacing.interlacedResourceBlockSet(carrier,pucch);
    else
        nIRB = [];
        Mrb = numel(unique(pucch.PRBSet(:)));
    end

    % Number of subcarriers allocated for PUCCH
    Msc = Mrb*12;

    intraSlotFreqHopping = ~interlacing && strcmpi(pucch.FrequencyHopping,'intraSlot');
    
    % Get the starting OFDM symbol location and number of OFDM symbol
    % locations of PUCCH
    symAllocation = double(pucch.SymbolAllocation);
    symStart = symAllocation(1);
    nPUCCHSym = symAllocation(2);

    % Get the number of symbols allocated for DM-RS in each hop
    nSF = ceil(nPUCCHSym/2);
    if intraSlotFreqHopping
        if rem(nPUCCHSym,2) == 1
            nSF0 = floor(nSF/2);
        else
            nSF0 = ceil(nSF/2);
        end
        nSFmin = floor(nPUCCHSym/4);
    else
        nSF0 = nSF;
        nSFmin = floor(nPUCCHSym/2);
    end
    nSF1 = nSF - nSF0;

    % Check if occi is greater than or equal to nSFmin, then error has to be
    % thrown
    coder.internal.errorIf(occi>=nSFmin,'nr5g:nrPUCCH:InvalidOCCIPUCCH1',occi,nSFmin);

    % Get the PUCCH hopping information
    seqCS = 0;
    info = nrPUCCHHoppingInfo(carrier.CyclicPrefix,nslot,nid,pucch.GroupHopping,...
        pucch.InitialCyclicShift,seqCS,nIRB);

    % Get the low-PAPR sequence for OFDM symbols in both hops
    ind = symStart+1:2:14;
    alpha1 = info.Alpha(:,ind(1:nSF0));
    lps1 = nrLowPAPRS(info.U(1),info.V(1),alpha1(:),Msc);
    
    if intraSlotFreqHopping
        alpha2 = reshape(info.Alpha(1:end,ind(nSF0+1:nSF0+nSF1)),1,[]);
        lps2 = reshape(nrLowPAPRS(info.U(2),info.V(2),alpha2,Msc),[],nSF1);
        r = [lps1 lps2];
    else
        r = reshape(lps1,[],nSF0);
    end

    % Get the orthogonal sequence from spreading factor and orthogonal
    % cover code index
    oSeq1 = nr5g.internal.PUCCH1Spreading(nSF0,occi);
    if intraSlotFreqHopping
        oSeq2 = nr5g.internal.PUCCH1Spreading(nSF1,occi);
        oSeq = [oSeq1 oSeq2];
    else
        oSeq = oSeq1;
    end

    % Get the PUCCH format 1 DM-RS sequence
    Msc = size(r,1);
    sym = r.*repmat(oSeq,Msc,1);

end
