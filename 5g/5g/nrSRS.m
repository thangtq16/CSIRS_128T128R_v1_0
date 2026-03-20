function [sym, info] = nrSRS(carrier,srs,varargin)
%nrSRS Uplink sounding reference signal
%   [SYM,INFO] = nrSRS(CARRIER,SRS) returns a complex matrix containing
%   uplink sounding reference signal (SRS) values corresponding to the
%   carrier-specific configuration object CARRIER and SRS-specific
%   configuration object SRS as defined in TS 38.211 section 6.4.1.4.2. The
%   symbols for each antenna port are in the columns of SYM, with the
%   number of columns determined by the number of configured transmission
%   antenna ports. The function also provides additional information INFO
%   regarding the SRS generation process.
%   
%   CARRIER is a carrier-specific configuration object as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> with properties:
%
%   SubcarrierSpacing     - Subcarrier spacing in kHz
%   CyclicPrefix          - Cyclic prefix (CP) type
%   NSlot                 - Absolute slot number
%   NFrame                - Absolute system frame number
%
%   SRS is an SRS-specific configuration object as described in
%   <a href="matlab:help('nrSRSConfig')">nrSRSConfig</a> with properties:
%   
%   NumSRSPorts              - Number of SRS antenna ports (1,2,4,8)
%   SymbolStart              - First SRS symbol in a slot (0...13)
%   NumSRSSymbols            - Number of consecutive OFDM symbols allocated 
%                              to the SRS (1,2,4,8,10,12,14)
%   SRSPeriod                - Slot periodicity and offset of the SRS 
%                              resource ('on','off',[Tsrs Toffset]).
%                              Accepted values for Tsrs are 1, 2, 4, 5, 8,
%                              10, 16, 20, 32, 40, 64, 80, 160, 320, 640,
%                              1280, 2560. The value of Toffset must be
%                              within the range (0...Tsrs-1). When SRSPeriod
%                              = 'on', the function returns symbols
%                              regardless of the slot and frame numbers.
%                              When SRSPeriod = 'off', the function returns
%                              an empty array of symbols. If SRSPeriod =
%                              [Tsrs Toffset], the function returns a
%                              nonempty array of symbols only for the
%                              candidate slots specified in TS 38.211
%                              Section 6.4.1.4.4
%   CSRS                     - Bandwidth configuration index C_SRS (0...63). 
%                              It controls the SRS bandwidth and frequency
%                              hopping, as defined in TS 38.211 Table
%                              6.4.1.4.3-1
%   BSRS                     - Bandwidth configuration index B_SRS (0...3). 
%                              It controls the SRS bandwidth and frequency
%                              hopping, as defined in TS 38.211 Table
%                              6.4.1.4.3-1
%   KTC                      - Transmission comb number (2,4,8). The SRS is 
%                              transmitted every KTC subcarriers
%   FrequencyScalingFactor   - Scaling factor for partial frequency sounding 
%                              (1,2,4)
%   CyclicShift              - Cyclic shift number offset (0...NCSmax-1). 
%                              The maximum number of cyclic shifts is NCSmax
%                              = 6 if KTC = 8, NCSmax = 12 if KTC = 4, and
%                              NCSmax = 8 if KTC = 2
%   GroupSeqHopping          - Group or sequence hopping configuration.
%                              ('neither','groupHopping','sequenceHopping')
%   NSRSID                   - SRS scrambling identity (0...65535). It
%                              determines the group number when
%                              GroupSeqHopping is set to 'neither'.
%                              Otherwise, it initializes the pseudorandom
%                              binary sequence for group or sequence hopping
%   EnableEightPortTDM       - Enable 8-port time division multiplexing (false, true)
%   CyclicShiftHopping       - Enable cyclic shift hopping (false, true)
%   CyclicShiftHoppingID     - Cyclic shift hopping identity
%   CyclicShiftHoppingSubset - Cyclic shift hopping subset
%   HoppingFinerGranularity  - Enable cyclic shift hopping finer granularity (false, true)
%
%   INFO is a structure containing the fields:
%   SeqGroup   - Base sequence group number per OFDM symbol (u)
%   NSeq       - Base sequence number per OFDM symbol (v)
%   Alpha      - Reference signal cyclic shift per symbol and port. When 
%                CyclicShiftHopping is set to false, Alpha is a
%                1-by-NumSRSPorts vector. Otherwise, it is a
%                NumSRSSymbols-by-NumSRSPorts matrix.
%   SeqLength  - Zadoff Chu sequence length (MRSSC)
% 
%   [SYM,INFO] = nrSRS(...,NAME,VALUE) specifies additional options
%   as NAME,VALUE pairs to allow control over the data type and format of
%   the output symbols:
%
%   'OutputDataType'       - 'double' for double precision (default)
%                            'single' for single precision
%
%   Example: 
%   % Generate SRS symbols for a 2-port SRS transmission of 4 OFDM symbols.  
%   carrier = nrCarrierConfig;
% 
%   srs = nrSRSConfig;
%   srs.NumSRSPorts = 2;
%   srs.NumSRSSymbols = 4;
%   srs.SymbolStart = 8;
%   srs.CSRS = 5; 
%   srs.BSRS = 0;
% 
%   [sym,info] = nrSRS(carrier,srs);
%
%   See also nrCarrierConfig, nrSRSConfig, nrSRSIndices.

%   Copyright 2019-2023 The MathWorks, Inc.

%#codegen

    narginchk(2,4);
    
    srs = validateInputs(carrier,srs);
    
    % PV pair check
    opts = nr5g.internal.parseOptions('nrSRS',{'OutputDataType'},varargin{:});

    % Carrier configuration parameters 
    NSlot = mod(double(carrier.NSlot),carrier.SlotsPerFrame);
    SymPerSlot = carrier.SymbolsPerSlot;
    
    % SRS config parameters
    NPorts = srs.NumSRSPorts;
    NSym = srs.NumSRSSymbols;
    TStart = srs.SymbolStart;
    KTC = srs.KTC;
    NID = srs.NSRSID;
    PF = srs.FrequencyScalingFactor;

    % Cyclic shift alpha for each port and OFDM symbol
    alpha = portSymbolCyclicShift(carrier,srs);
    
    % Number of RBs allocated to the SRS when FrequencyScalingFactor = 1
    mSRS_BSRS = nr5g.internal.srs.SRSBandwidthConfiguration(srs.CSRS,srs.BSRS);
    
    % SRS length in subcarriers
    NRBsc = 12;
    Msc = mSRS_BSRS*NRBsc/(KTC*PF);

    % Set group and sequence numbers (u,v) per OFDM symbol
    switch lower(srs.GroupSeqHopping)
        case 'grouphopping'
            u = groupNumberHopping(NSlot,SymPerSlot,NID,NSym,TStart);
            v = zeros(1,NSym);
        case 'sequencehopping'
            u = mod(NID,30)*ones(1,NSym);
            v = sequenceNumberHopping(NSlot,SymPerSlot,NID,NSym,TStart,Msc);
        otherwise % 'neither'
            u = mod(NID,30)*ones(1,NSym);
            v = zeros(1,NSym);
    end
    
    % WTDM sequence. This sequence can result in zero-valued SRS in some
    % resource elements, including full OFDM symbols and SRS ports. Obtain
    % the number of SRS ports in use after the TDM sequence is applied.
    wtdm = nr5g.internal.srs.getWTDM(srs.NumSRSSymbols,NPorts,srs.EnableEightPortTDM);

    % Determine if the slot must contain SRS (TS 38.211 Section 6.4.1.4.4)
    isCandidateSlot = nr5g.internal.isRSCandidateSlot(carrier.NSlot,carrier.NFrame,carrier.SlotsPerFrame,srs.SRSPeriod);

    if isCandidateSlot
        % Generate low-PAPR sequence per OFDM symbol only in candidate slots
        seqT = complex(zeros(Msc,NPorts,NSym,opts.OutputDataType));
        for s = 1:NSym
            seqT(:,:,s) = nrLowPAPRS(u(s),v(s),alpha(:,s),Msc,'OutputDataType',opts.OutputDataType);
        end
        seq = permute(seqT,[1 3 2]); % Msc-by-NumSRSSymbols-by-NumSRSPorts

        % Apply WTDM
        seq = seq.*wtdm;

        % Remove zeros and reshape to return as many columns as SRS ports
        sym = reshape(nonzeros(seq),[],NPorts); % Msc*NumSRSSymbols-by-NumSRSPorts
    else
        sym = zeros(0,NPorts,opts.OutputDataType);
        Msc = 0;
    end

    % Collect information
    info.SeqGroup = u;
    info.NSeq = v;
    info.Alpha = alpha(:,1:1+(NSym-1)*srs.CyclicShiftHopping).';
    info.SeqLength = Msc;
end

% Cyclic shift (alpha) per port
function alpha = portSymbolCyclicShift(carrier,srs)

    NPorts = srs.NumSRSPorts;
    KTC = srs.KTC;
    nCS = srs.CyclicShift;
    ports8tdm = srs.EnableEightPortTDM;

    % Max number of cyclic shifts based on comb type
    maxCS = [8 12 6];
    nCSmax = maxCS(log2(KTC));
    
    % Port numbers
    p = 1000 + (0:NPorts-1).';
    
    if ports8tdm
        NBarAP = 4;
        pBar = 1000 + mod(p,2);
        pBar(5:8) = pBar(5:8) + 2;
    else
        NBarAP = NPorts;
        pBar = p;
    end
    
    if NBarAP == 8 && nCSmax == 6
        scaling = 4;
    elseif (NBarAP == 4 && nCSmax == 6) || (NBarAP == 8 && nCSmax == 12)
        scaling = 2;
    else
        scaling = 1;
    end

    % Number of cyclic shift for each port
    nCSp = mod(nCS + nCSmax*floor((pBar-1000)/scaling)/(NBarAP/scaling),nCSmax);

    % Cyclic shift hopping term
    [fcsh,K] = getfcsh(carrier,srs,nCSmax);

    % Cyclic shift for each port and OFDM symbol as a matrix of size
    % NPorts-by-NumSRSSymbols
    alpha = (2*pi/nCSmax) * (nCSp + fcsh/K);
end

% Cyclic shift hopping
function [fcsh,K] = getfcsh(carrier,srs,nCSmax)

    coder.varsize('cyclicShiftHoppingSubset',[Inf Inf],[1 1]);

    NumSRSSymbols = srs.NumSRSSymbols;
    SymbolStart = srs.SymbolStart;
    cyclicShiftHopping = srs.CyclicShiftHopping;
    cyclicShiftHoppingID = srs.CyclicShiftHoppingID;
    cyclicShiftHoppingSubset = srs.CyclicShiftHoppingSubset;
    hoppingFinerGranularity = srs.HoppingFinerGranularity;

    if cyclicShiftHopping
        % Determine the cyclic shift hopping subset, its cardinality and K
        K = 1;
        if isempty(cyclicShiftHoppingSubset)
            if hoppingFinerGranularity
                K = 2;
            end
            ncsh = K*nCSmax;
            cyclicShiftHoppingSubset = 0:K*nCSmax-1;
        else
            ncsh = numel(cyclicShiftHoppingSubset);
        end
        
        % Relative slot and frame numbers
        [nsf,nf] = nr5g.internal.getRelativeNSlotAndSFN(double(carrier.NSlot),double(carrier.NFrame),carrier.SlotsPerFrame);
        symPerSlot = carrier.SymbolsPerSlot;
        symbolsPerFrame = carrier.SlotsPerFrame*symPerSlot;
        
        % Index of the first bit of the PRBS
        firstBitIndex = 8*(mod(nf,128)*symbolsPerFrame + nsf*symPerSlot + SymbolStart);
        
        % Pseudorandom binary sequence
        PRBS = nrPRBS(cyclicShiftHoppingID, [firstBitIndex 8*NumSRSSymbols]);
        
        % Reorganize PRBS indices for OFDM symbol processing
        PRBSm = reshape(PRBS,8,NumSRSSymbols);
        
        % Cyclic shift for each OFDM symbol by converting 8-bit blocks of
        % the PRBS to mod-ncsh integer
        m = (0:7)';
        cshInd = mod(2.^m'*PRBSm, ncsh);
        fcsh = cyclicShiftHoppingSubset(1+cshInd);

    else
        K = 1;
        fcsh = zeros(1,NumSRSSymbols);
    end

end

% Calculate the group number (u) of low PAPR sequence for SRS group hopping
function u = groupNumberHopping(NSlot,SymbolsPerSlot,NID,NSym,SymbolStart)

    % Index of the first bit of the PRBS   
    firstBitIndex = 8*(NSlot*SymbolsPerSlot + SymbolStart);

    % Length of pseudorandom binary sequence
    PRBSLen = 8*NSym;

    % Pseudorandom binary sequence
    PRBS = nrPRBS(NID, [firstBitIndex PRBSLen]); 

    % Reorganize PRBS indices for OFDM symbol processing
    PRBSm = reshape(PRBS,8,NSym);

    % Group number (u) per OFDM symbol by converting 8-bit blocks of the
    % PRBS to mod-30 integer
    m = (0:7)';
    f = mod(2.^m'*PRBSm, 30);
    u = mod(f + NID, 30);
end

% Calculate base sequences (v) of low PAPR sequence for SRS sequence number hopping
function v = sequenceNumberHopping(NSlot,SymbolsPerSlot,nID,NSym,SymbolStart,Mscs)

    % SRS occupies less than 6 RBs
    NRBsc = 12;
    if Mscs < 6*NRBsc
        v = zeros(1,NSym);
    else 
        % Index of the first bit of the PRBS
        firstBitIndex = NSlot*SymbolsPerSlot + SymbolStart;
        
        % Sequence number sequence
        v = double(nrPRBS(nID, [firstBitIndex NSym])');
    end
end

function srs = validateInputs(carrier,srs)
    fcnName = 'nrSRS';
    validateattributes(carrier, {'nrCarrierConfig'}, {'scalar'}, fcnName, 'Carrier-specific configuration object');
    validateattributes(srs, {'nrSRSConfig'}, {'scalar'}, fcnName, 'SRS-specific configuration object');
    
    srs = validateConfig(srs);

    % Cross validation of properties
    SymbolStart = srs.SymbolStart;
    NumSRSSymbols = srs.NumSRSSymbols;
    if SymbolStart + NumSRSSymbols > carrier.SymbolsPerSlot
        cpOffset = 2*(carrier.SymbolsPerSlot==12);
        coder.internal.error('nr5g:nrSRS:InconsistentTimeAllocation', SymbolStart,NumSRSSymbols,SymbolStart+NumSRSSymbols-1,13-cpOffset,carrier.CyclicPrefix);
    end
end