function [cw,info] = dataAndControlMapping(pusch,GULSCH,GACK,GCSI1,GCSI2,GACKRvd)
%dataAndControlMapping Uplink shared channel data and control mapping
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [CW,INFO] = dataAndControlMapping(PUSCHSTR,GULSCH,GACK,GCSI1,GCSI2,GACKRVD)
%   returns the codeword by performing data and control mapping, with
%   pre-defined values for coded UL-SCH and coded UCI type(s), as defined
%   in TS 38.212 Section 6.2.7. The inputs required to perform this
%   functionality are physical uplink shared channel configuration
%   structure PUSCHSTR, bit capacity of UL-SCH GULSCH, bit capacity of each
%   UCI type (GACK, GCSI1, GCSI2), and number of bits reserved for
%   HARQ-ACK GACKRVD. CW contains the values of {-1, -2, -3, -4}
%   corresponding to coded {HARQ-ACK, CSI part 1, CSI part 2, UL-SCH} bits.
%
%   PUSCHSTR is a structure with the fields:
%   MULSCH           - Number of resource elements available for data
%                      transmission in each OFDM symbol
%   MUCI             - Number of resource elements available for UCI
%                      transmission in each OFDM symbol
%   PUSCHSymbolSet   - 1-based OFDM symbol locations carrying data relative
%                      to the first OFDM symbol of PUSCH allocation
%                      (excluding DM-RS)
%   DMRSSymbolSet    - 1-based OFDM symbol locations carrying DM-RS relative
%                      to the first OFDM symbol of PUSCH allocation
%   PRBSet           - Physical resource blocks allocated for PUSCH
%   Modulation       - Modulation scheme
%                      ('pi/2-BPSK', 'QPSK', '16QAM', '64QAM', '256QAM')
%   NumLayers        - Number of transmission layers (1...4)
%   FrequencyHopping - Frequency hopping configuration
%                      ('neither', 'intraSlot', 'interSlot')
%
%   INFO contains the fields:
%   ULSCHIndices    - Locations of UL-SCH bits in the codeword
%   CSI1Indices     - Locations of CSI part 1 bits in the codeword
%   CSI2Indices     - Locations of CSI part 2 bits in the codeword
%   ACKIndices      - Locations of HARQ-ACK bits in the codeword
%   ULSCHACKIndices - Overlapped locations of UL-SCH and HARQ-ACK bits
%   CSI2ACKIndices  - Overlapped locations of CSI part 2 and HARQ-ACK bits

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    % Assign the fields of pusch structure to variables
    mULSCH = pusch.MULSCH;
    mUCI = pusch.MUCI;
    puschsymbols = pusch.PUSCHSymbolSet;
    dmrssymbols = pusch.DMRSSymbolSet;
    prbset = pusch.PRBSet;

    % Set the resource element locations of each OFDM symbol for data
    % transmission
    nPUSCHsymall = length(mULSCH);
    phiULSCH = cell(1,nPUSCHsymall);
    for i = 1:nPUSCHsymall
        phiULSCH{i} = (0:mULSCH(i)-1)';
    end

    % Set the resource element locations of each OFDM symbol for UCI
    % transmission
    phiUCI = phiULSCH;
    for i = 1:length(dmrssymbols)
        phiUCI{dmrssymbols(i)} = zeros(0,1);
    end

    % Get the parameters from PUSCH configuration structure
    isFreqHopEnabled = strcmpi(pusch.FrequencyHopping,'intraSlot');
    nlayers = double(pusch.NumLayers);
    qm    = nr5g.internal.getQm(pusch.Modulation);
    nlqm  = nlayers*qm;
    nlqm2 = 2*nlqm;

    % Initialize some parameters
    gBar = zeros(nPUSCHsymall,12*length(prbset),nlqm);
    GACK1 = GACK;
    GCSI11 = GCSI1;
    GCSI21 = GCSI2;
    GACK2 = 0;
    GCSI12 = 0;
    GCSI22 = 0;
    m3 = sum(mUCI);

    % Get the parameter sets of each hop
    Nhop = 1;
    l1 = zeros(1,0);
    l2 = zeros(1,0);
    lcsi1 = zeros(1,0);
    lcsi2 = zeros(1,0);
    if isFreqHopEnabled

        % Find the OFDM symbol locations in each hop, excluding DM-RS
        % symbol locations
        nPUSCHSymFirstHop = floor(nPUSCHsymall/2);
        puschHopIndex = puschsymbols > nPUSCHSymFirstHop;
        pusch1 = puschsymbols(~puschHopIndex);
        pusch2 = puschsymbols(puschHopIndex);

        % Get the first OFDM symbol location in each hop, which is
        % available for UCI transmission
        if ~isempty(pusch1)
            lcsi1 = pusch1(1);
        end
        if ~isempty(pusch2)
            lcsi2 = pusch2(1);
        end

        % Get the first OFDM symbol (l_1, l_2) that does not carry DM-RS in
        % each hop, after the first DM-RS consecutive symbol(s) in the
        % PUSCH transmission
        l1 = lcsi1;
        l2 = lcsi2;
        if ~isempty(dmrssymbols)
            dmrsIndex = dmrssymbols > (nPUSCHSymFirstHop);
            dmrs2 = dmrssymbols(dmrsIndex);
            dmrs1 = dmrssymbols(~dmrsIndex);

            if ~isempty(dmrs1) && any(dmrs1(1)<pusch1)
                l1(:) = pusch1(find(dmrs1(1)<pusch1,1));
                m3 = sum(mUCI(l1(1):end));
            end
            if ~isempty(dmrs2) && any(dmrs2(1)<pusch2)
                l2(:) = pusch2(find(dmrs2(1)<pusch2,1));
            end
        end

        % Calculate the bit capacities in each hop for each UCI type, only
        % if there is a possibility of UCI transmission in both hops
        cond = ~([isempty(l1) isempty(l2) isempty(lcsi1) isempty(lcsi2)]);
        if all(cond) && (l2(1) == pusch2(1))
            Nhop = 2;

            % Get the number of resource elements available for UCI in each
            % hop
            m1 = sum(mUCI(1:nPUSCHSymFirstHop));    % First hop
            m2 = sum(mUCI(nPUSCHSymFirstHop+1:end));% Second hop
            m3 = sum(mUCI(l1(1):nPUSCHSymFirstHop));   % 1st DM-RS symbol to last symbol in first hop

            % HARQ-ACK with UL-SCH
            if GACK && GULSCH
                GACK1 = nlqm*floor(GACK/nlqm2);
                GACK2 = nlqm*ceil(GACK/nlqm2);

                % Update the values, if GACK1 is larger than the resource
                % elements available for HARQ-ACK in first hop
                if GACK1 > m3*nlqm
                    GACK1 = m3*nlqm;
                    GACK2 = GACK - GACK1;
                end
            end

            % CSI part 1 with UL-SCH
            if GCSI1 && GULSCH
                GCSI11 = nlqm*floor(GCSI1/nlqm2);
                GCSI12 = nlqm*ceil(GCSI1/nlqm2);

                % Check if there are any resource elements available for
                % CSI part 1 transmission, in case of oack > 2. Update the
                % values to place CSI part 1 in first hop itself
                if (m2*nlqm <= GACK2) && ~GACKRvd
                    GCSI11 = GCSI1;
                    GCSI12 = 0;
                end
            end

            % CSI part 2 with UL-SCH
            if GCSI2 && GULSCH
                GCSI21 = nlqm*floor(GCSI2/nlqm2);
                GCSI22 = nlqm*ceil(GCSI2/nlqm2);

                % Check if there are any resource elements available for
                % CSI part 2 transmission, in case of oack > 2. Update the
                % values to place CSI part 2 in first hop itself
                if (m2*nlqm <= GACK2) && ~GACKRvd
                    GCSI21 = GCSI2;
                    GCSI22 = 0;
                end
            end

            if ~GULSCH
                % No UL-SCH transmission
                % CSI part 1 and CSI part 2 without HARQ-ACK
                if ~GACK
                    if GCSI1
                        GACKRvd1 = nlqm*floor(GACKRvd/nlqm2);
                        GCSI11 = min(nlqm*floor(GCSI1/nlqm2), ...
                            m1*nlqm-GACKRvd1);
                        GCSI12 = GCSI1 - GCSI11;
                    end
                    if GCSI2
                        GCSI21 = m1*nlqm - GCSI11;
                        GCSI22 = m2*nlqm - GCSI12;
                    end
                else
                    % CSI part 1 and CSI part 2 with HARQ-ACK
                    GACK1 = min(nlqm*floor(GACK/nlqm2),...
                        m3*nlqm);
                    GACK2 = GACK - GACK1;

                    % CSI part 1
                    if GCSI1 && ~GCSI2
                        GCSI11 = m1*nlqm - GACK1;
                        GCSI12 = GCSI1 - GCSI11;
                    end
                    % CSI part 1 and CSI part 2
                    if GCSI2
                        % CSI part 1
                        if ~GACKRvd % oACK > 2
                            GackCSI1 = GACK1;
                        else
                            GACKRvdCSI1 = nlqm*floor(GACKRvd/nlqm2);
                            GackCSI1 = min(GACKRvdCSI1,m3*nlqm);
                        end
                        GCSI11 = min(nlqm*floor(GCSI1/nlqm2), ...
                            m1*nlqm-GackCSI1);
                        GCSI12 = GCSI1 - GCSI11;
                        % Check if there are any resource elements
                        % available for CSI part 1 transmission, in case of
                        % oack > 2. Update the values to place CSI part 1
                        % in first hop itself
                        if (GCSI12 > m2*nlqm-GACK2) && ~GACKRvd
                            GCSI12 = m2*nlqm-GACK2;
                            GCSI11 = GCSI1-GCSI12;
                        end

                        % CSI part 2
                        if ~GACKRvd % oACK > 2
                            GCSI21 = m1*nlqm - GACK1 - GCSI11;
                            GCSI22 = m2*nlqm - GACK2 - GCSI12;
                        else
                            GCSI21 = m1*nlqm - GCSI11;
                            GCSI22 = m2*nlqm - GCSI12;
                        end
                    end
                end % if ~Gack
            end % if ~GULSCH
        end % if all(cond)
    end
    if Nhop == 1
        % Get the starting OFDM symbol location of UCI transmission, before
        % and after the first set of consecutive DM-RS locations
        if ~isempty(puschsymbols)
            lcsi1 = puschsymbols(1);
        end
        if ~isempty(dmrssymbols) && any(dmrssymbols(1)<puschsymbols)
            l1 = puschsymbols(find(dmrssymbols(1)<puschsymbols,1));
            m3 = sum(mUCI(l1(1):end));
        else
            l1 = lcsi1;
        end
        l2 = zeros(1,0);
        lcsi2 = zeros(1,0);
    end

    % Step 1
    phiBarULSCH = phiULSCH;
    mBarULSCH = mULSCH;

    % Initialize phiBarRvd to get the set of reserved elements in each OFDM
    % symbol
    phiBarRvd = cell(nPUSCHsymall,1);
    for i = 1:nPUSCHsymall
        phiBarRvd{i} = zeros(0,1);
    end

    % Set of reserved elements
    if GACKRvd % Check if GackRvd is greater than 0
        lprime = [l1 l2];
        if Nhop == 2
            GACKRvd1 = nlqm*floor(GACKRvd/nlqm2);
            GACKRvd2 = nlqm*ceil(GACKRvd/nlqm2);

            % Update the values, if GACKRvd1 is greater than the resource
            % elements available for HARQ-ACK in first hop
            if GACKRvd1 > m3*nlqm
                GACKRvd1 = m3*nlqm;
                GACKRvd2 = GACKRvd - GACKRvd1;
            end

            % Check CSI part 1 value in second hop and update accordingly
            mCSI2 = sum(mUCI(floor(nPUSCHsymall/2)+1:end));
            if GCSI12 > mCSI2*nlqm-GACKRvd2
                GCSI12 = mCSI2*nlqm - GACKRvd2;
                GCSI11 = GCSI1 - GCSI12;
            end
        else
            GACKRvd1 = GACKRvd;
            GACKRvd2 = 0;
        end

        mACKcount = [0 0];
        GACKRvdTemp = [GACKRvd1 GACKRvd2];

        for i = 1:numel(lprime)
            sym = lprime(i);
            while mACKcount(i) < GACKRvdTemp(i)
                if sym > nPUSCHsymall
                    % Check for the symbol number greater than the number
                    % of PUSCH allocated symbols in each hop and avoid out
                    % of bounds indexing
                    break;
                end

                if mUCI(sym) > 0
                    % Total number of reserved elements remaining per hop
                    numACKRvd = GACKRvdTemp(i)-mACKcount(i);

                    if numACKRvd >= mUCI(sym)*nlqm
                        d = 1;
                        mREcount = mBarULSCH(sym);
                    else
                        d = floor((mUCI(sym)*nlqm)/numACKRvd);
                        mREcount = ceil(numACKRvd/nlqm);
                    end
                    phiBarRvd{sym} = phiBarULSCH{sym}((0:mREcount-1)*d+1);
                    mACKcount(i) = mACKcount(i) + mREcount*nlqm;
                end
                sym = sym+1;
            end % while
        end % for Nhop
    end % if oACK

    % Number of reserved elements in each OFDM symbol
    mPhiSCRVD = zeros(nPUSCHsymall,1);
    for i = 1:nPUSCHsymall
        mPhiSCRVD(i) = length(phiBarRvd{i});
    end

    % Step 2
    % ACK (oACK > 2)
    if ~GACKRvd && GACK
        GACKTemp = [GACK1 GACK2];
        lprime = [l1 l2];
        mACKcount = [0 0];
        mACKcountall = 0;
        for i = 1:numel(lprime)
            sym = lprime(i);
            while mACKcount(i) < GACKTemp(i)
                if sym > nPUSCHsymall
                    % Check for the symbol number greater than the number
                    % of PUSCH allocated symbols in each hop and avoid out
                    % of bounds indexing
                    break;
                end

                if mUCI(sym) > 0
                    % Total number of remaining HARQ-ACK bits to be
                    % accommodated per hop
                    numACK = GACKTemp(i)-mACKcount(i);

                    if numACK >= mUCI(sym)*nlqm
                        d = 1;
                        mREcount = mUCI(sym);
                    else
                        d = floor((mUCI(sym)*nlqm)/numACK);
                        mREcount = ceil(numACK/nlqm);
                    end

                    % Place coded HARQ-ACK bits in gBar at relevant
                    % positions
                    k = phiUCI{sym}((0:mREcount-1)*d+1);
                    gBar(sym,k+1,1:nlqm) = -1; % -1 for HARQ-ACK
                    mACKcountall = mACKcountall+(mREcount*nlqm);
                    mACKcount(i) = mACKcount(i)+(mREcount*nlqm);

                    phiUCItemp = phiUCI{sym}((0:mREcount-1)*d+1);
                    phiUCI{sym} = setdifference(phiUCI{sym},phiUCItemp);
                    phiBarULSCH{sym} = setdifference(phiBarULSCH{sym},phiUCItemp);
                    mUCI(sym) = length(phiUCI{sym});
                    mBarULSCH(sym) = length(phiBarULSCH{sym});
                end
                sym = sym+1;
            end % while
        end % for Nhop
    end % if oACK > 2

    % Step 3
    % CSI part 1
    if GCSI1
        lprime = [lcsi1 lcsi2];
        mCSIcount = [0 0];
        mCSIcountall = 0;
        GCSI1Temp = [GCSI11 GCSI12];
        for i = 1:numel(lprime)
            sym = lprime(i);
            while mUCI(sym)-mPhiSCRVD(sym) <= 0
                sym = sym+1;
                if sym > nPUSCHsymall
                    break; % exit loop at end of symbols
                end
            end
            while mCSIcount(i) < GCSI1Temp(i)
                if sym > nPUSCHsymall
                    % Check for the symbol number greater than the number
                    % of PUSCH allocated symbols in each hop and avoid out
                    % of bounds indexing
                    break;
                end

                % Number of resource elements available for CSI part 1 in
                % each symbol
                mUCIDiffmPhiRvd = mUCI(sym)-mPhiSCRVD(sym);

                % Total number of remaining CSI part 1 bits to be
                % accommodated per hop
                numCSI1 = GCSI1Temp(i)-mCSIcount(i);

                if mUCIDiffmPhiRvd > 0
                    if numCSI1 >= mUCIDiffmPhiRvd*nlqm
                        d = 1;
                        mREcount = mUCIDiffmPhiRvd;
                    else
                        d = floor((mUCIDiffmPhiRvd*nlqm)/numCSI1);
                        mREcount = ceil(numCSI1/nlqm);
                    end
                    phitemp = setdifference(phiUCI{sym},phiBarRvd{sym});

                    % Place coded CSI part 1 bits in gBar at relevant
                    % positions
                    k = phitemp((0:mREcount-1)*d+1);
                    gBar(sym,k+1,1:nlqm) = -2; % -2 for CSI part 1
                    mCSIcountall = mCSIcountall+(mREcount*nlqm);
                    mCSIcount(i) = mCSIcount(i)+(mREcount*nlqm);

                    phiUCItemp = phitemp((0:mREcount-1)*d+1);
                    phiUCI{sym} = setdifference(phiUCI{sym},phiUCItemp);
                    phiBarULSCH{sym} = setdifference(phiBarULSCH{sym},phiUCItemp);
                    mUCI(sym) = length(phiUCI{sym});
                    mBarULSCH(sym) = length(phiBarULSCH{sym});
                end % end if
                sym = sym+1;
            end % while
        end % for Nhop
    end % if Gcsi1

    % CSI part 2
    if GCSI2
        lprime = [lcsi1 lcsi2];
        mCSIcount = [0 0];
        mCSIcountall = 0;
        GCSI2Temp = [GCSI21 GCSI22];
        for i = 1:numel(lprime)
            sym = lprime(i);
            while mUCI(sym) <= 0
                sym = sym+1;
                if sym > nPUSCHsymall
                    break; % exit loop at end of symbols
                end
            end
            while mCSIcount(i)<GCSI2Temp(i)
                if sym > nPUSCHsymall
                    % Check for the symbol number greater than the
                    % number of PUSCH allocated symbols in each hop and
                    % avoid out of bounds indexing
                    break;
                end

                if mUCI(sym) > 0
                    % Total number of CSI part 2 bits remaining to be
                    % accommodated per hop
                    numCSI2 = GCSI2Temp(i)-mCSIcount(i);

                    if numCSI2 >= mUCI(sym)*nlqm
                        d = 1;
                        mREcount = mUCI(sym);
                    else
                        d = floor((mUCI(sym)*nlqm)/numCSI2);
                        mREcount = ceil(numCSI2/nlqm);
                    end

                    % Place coded CSI part 2 bits in gBar at relevant
                    % positions
                    k = phiUCI{sym}((0:mREcount-1)*d+1);
                    gBar(sym,k+1,1:nlqm) = -3; % -3 for CSI part 2
                    mCSIcountall = mCSIcountall+(mREcount*nlqm);
                    mCSIcount(i) = mCSIcount(i)+(mREcount*nlqm);

                    phiUCItemp = phiUCI{sym}((0:mREcount-1)*d+1);
                    phiUCI{sym} = setdifference(phiUCI{sym},phiUCItemp);
                    phiBarULSCH{sym} = setdifference(phiBarULSCH{sym},phiUCItemp);
                    mUCI(sym) = length(phiUCI{sym});
                    mBarULSCH(sym) = length(phiBarULSCH{sym});
                end % if
                sym = sym+1;
            end % while
        end  % for Nhop
    end % if GCSI2

    % Step 4
    % UL-SCH
    mULSCHcount = 0;
    if GULSCH
        for sym = 0:nPUSCHsymall-1
            if mBarULSCH(sym+1) > 0

                % Place coded UL-SCH bits in gBar at relevant positions
                k = phiBarULSCH{sym+1}(1:mBarULSCH(sym+1));
                gBar(sym+1,k+1,1:nlqm) = -4; % -4 for UL-SCH
                mULSCHcount = mULSCHcount+(mBarULSCH(sym+1)*nlqm);

            end % if
        end % for sym
    end % if Gulsch

    % Step 5
    % ACK (oACK <= 2)
    if GACKRvd && GACK
        lprime = [l1 l2];
        mACKcount = [0 0];
        mACKcountall = 0;
        GACKTemp = [GACK1 GACK2];
        for i = 1:numel(lprime)
            sym = lprime(i);
            while mACKcount(i) < GACKTemp(i)
                if sym > nPUSCHsymall
                    % Check for the symbol number greater than the
                    % number of PUSCH allocated symbols in each hop and
                    % avoid out of bounds indexing
                    break;
                end

                if mPhiSCRVD(sym)>0
                    % Total number of remaining HARQ-ACK bits to be
                    % accommodated per hop
                    numACK = GACKTemp(i)-mACKcount(i);
                    if numACK >= mPhiSCRVD(sym)*nlqm
                        d = 1;
                        mREcount = mPhiSCRVD(sym);
                    else
                        d = floor((mPhiSCRVD(sym)*nlqm)/numACK);
                        mREcount = ceil(numACK/nlqm);
                    end

                    % Place coded HARQ-ACK bits in gBar at relevant
                    % positions
                    k = phiBarRvd{sym}((0:mREcount-1)*d+1);
                    gBar(sym,k+1,1:nlqm) = gBar(sym,k+1,1:nlqm)+5; % Add 5 to get the locations of ACK, overlapped with UL-SCH or CSI2
                    mACKcountall = mACKcountall+(mREcount*nlqm);
                    mACKcount(i) = mACKcount(i)+(mREcount*nlqm);

                end % if
                sym = sym+1;
            end % while
        end % for Nhop
    end % if oack <= 2

    % Step 6
    % Return the multiplexed output
    cackInd = zeros(0,1);
    ccsi1Ind = zeros(0,1);
    ccsi2Ind = zeros(0,1);
    culschInd = zeros(0,1);
    culschAckInd = zeros(0,1);
    ccsi2AckInd = zeros(0,1);
    if GULSCH || GACK || GCSI1 || GCSI2
        t = 0;
        cwLen = sum(mULSCH(:))*nlqm;
        cw = zeros(cwLen,1); % Initialize temporary codeword
        for sym = 0:nPUSCHsymall-1
            for j = 0:mULSCH(sym+1)-1
                k = phiULSCH{sym+1}(j+1);
                cw(t+1:t+nlqm) = gBar(sym+1,k+1,:);
                t = t+nlqm;
            end
        end
        cackInd = sort([find(cw == -1); find(cw > 0)]);
        ccsi1Ind = find(cw == -2);
        ccsi2Ind = find(cw == -3);
        culschInd = find(cw == -4);
        culschAckInd = find(cw == 1);
        ccsi2AckInd = find(cw == 2);
    else
        % Inputs ulsch, ack, csi1, csi2 are empty, return empty
        cw = zeros(0,1);
    end

    % Combine information into output structure
    info = struct;
    info.ULSCHIndices = uint32(culschInd);
    info.CSI1Indices = uint32(ccsi1Ind);
    info.CSI2Indices = uint32(ccsi2Ind);
    info.ACKIndices = uint32(cackInd);
    info.ULSCHACKIndices = uint32(culschAckInd);
    info.CSI2ACKIndices = uint32(ccsi2AckInd);

end

function out = setdifference(firstSet,secondSet)
% Returns the values of firstSet that are not present in secondSet with no
% repetitions
    temp = zeros(max(firstSet)+1,1);
    temp(firstSet+1) = 1;
    temp(secondSet+1) = temp(secondSet+1)+1;
    out = find(temp == 1) - 1;
end
