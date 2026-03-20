function [ind,info] = nrSRSIndices(carrier,srs,varargin)
%nrSRSIndices Uplink SRS resource element indices 
%   [IND,INFO] = nrSRSIndices(CARRIER,SRS) returns a matrix containing
%   resource element indices for the uplink sounding reference signal (SRS)
%   corresponding to the carrier configuration object CARRIER and the SRS
%   configuration object SRS, as defined in TS 38.211 section 6.4.1.4.3. By
%   default, the indices are returned in 1-based linear form that directly
%   index the elements of a resource matrix. The number of columns in IND
%   is equal to the number of antenna ports configured. These indices are
%   ordered as the SRS modulation symbols should be mapped.
% 
%   CARRIER is a carrier-specific configuration object as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> with properties:
%
%   SubcarrierSpacing     - Subcarrier spacing in kHz
%   CyclicPrefix          - Cyclic prefix (CP) type
%   NSlot                 - Absolute slot number
%   NFrame                - Absolute system frame number
%   NSizeGrid             - Size of the carrier resource grid in terms of
%                           number of resource blocks (RBs)
%
%   SRS is an SRS-specific configuration object as described in
%   <a href="matlab:help('nrSRSConfig')">nrSRSConfig</a> with properties:
%
%   NumSRSPorts             - Number of SRS antenna ports (1,2,4,8)
%   SymbolStart             - First SRS symbol in a slot (0...13)
%   NumSRSSymbols           - Number of consecutive OFDM symbols allocated 
%                             to the SRS (1,2,4,8,10,12,14)
%   SRSPeriod               - Slot periodicity and offset of the SRS 
%                             resource ('on','off',[Tsrs Toffset]).
%                             Accepted values for Tsrs are 1, 2, 4, 5, 8,
%                             10, 16, 20, 32, 40, 64, 80, 160, 320, 640,
%                             1280, 2560. The value of Toffset must be
%                             within the range (0...Tsrs-1). When SRSPeriod
%                             = 'on', the function returns symbols
%                             regardless of the slot and frame numbers.
%                             When SRSPeriod = 'off', the function returns
%                             an empty array of symbols. If SRSPeriod =
%                             [Tsrs Toffset], the function returns a
%                             nonempty array of symbols only for the
%                             candidate slots specified in TS 38.211
%                             Section 6.4.1.4.4
%   ResourceType            - Time domain behavior of SRS resource.
%                             ('periodic', 'semi-persistent', 'aperiodic').
%                             When ResourceType = 'aperiodic', the
%                             SRSPeriod parameter is interpreted as the
%                             periodicity and offset of the downlink
%                             control information signal triggering
%                             aperiodic SRS transmissions
%   FrequencyStart          - Starting position of the SRS in frequency in
%                             PRBs when NRRC = 0
%   NRRC                    - Frequency domain position (0...67). 
%                             Additional frequency offset to FrequencyStart
%                             specified in blocks of 4 RBs. The resulting
%                             location of the SRS in frequency depends on
%                             FrequencyStart and the parameters in TS
%                             38.211 Table 6.4.1.4.3-1
%   CSRS                    - Bandwidth configuration index C_SRS (0...63). 
%                             It controls the SRS bandwidth and frequency
%                             hopping, as defined in TS 38.211 Table
%                             6.4.1.4.3-1
%   BSRS                    - Bandwidth configuration index B_SRS (0...3). 
%                             It controls the SRS bandwidth and frequency
%                             hopping, as defined in TS 38.211 Table
%                             6.4.1.4.3-1
%   BHop                    - Frequency hopping configuration (0...3). Set 
%                             BHop >= BSRS to disable frequency hopping
%   Repetition              - Repetition factor (1,2,4,5,6,7,8,10,12,14).
%                             Repetition must be <= NumSRSSymbols when
%                             frequency hopping is enabled. When frequency
%                             hopping is disabled, Repetition is ignored.
%   KTC                     - Transmission comb number (2,4,8). The SRS is 
%                             transmitted every KTC subcarriers
%   KBarTC                  - Transmission comb offset in subcarriers
%                             (0...KTC-1)
%   FrequencyScalingFactor  - Scaling factor for partial frequency sounding 
%                             (1,2,4)
%   StartRBIndex            - Index of the partial frequency sounding
%                             frequency block (0...FrequencyScalingFactor-1)
%   EnableStartRBHopping    - Enable frequency hopping of first RB when
%                             FrequencyScalingFactor > 1 (true,false)
%   CyclicShift             - Cyclic shift number (0...NCSmax-1). 
%                             The maximum number of cyclic shifts is NCSmax
%                             = 6 if KTC = 8, NCSmax = 12 if KTC = 4, and
%                             NCSmax = 8 if KTC = 2
%   SRSPositioning          - Enable SRS for user positioning (true,false).
%                             A alue of true corresponds to the
%                             higher-layer parameter 'SRS-PosResource-r16'
%                             and false corresponds to 'SRS-Resource'.
%   EnableEightPortTDM      - Enable 8-port time division multiplexing (false, true)
%   CombOffsetHopping       - Enable comb offset hopping (false, true)
%   CombOffsetHoppingID     - Comb offset hopping identity
%   CombOffsetHoppingSubset - Comb offset hopping subset
%   HoppingWithRepetition   - Enable comb offset hopping with repetition (false, true)
%
%   INFO is a structure containing the fields:
%   SubcarrierOffset  - Frequency starting position of the SRS in 
%                       subcarriers per antenna port and OFDM symbol (k_0).
%                       SubcarrierOffset is a matrix of size
%                       NumSRSPorts-by-NumSRSSymbols.
%   FreqIndex         - Frequency position index (n_b) per OFDM symbol 
%                       for b = 0...BSRS. FreqIndex is a matrix of size
%                       (BSRS+1)-by-NumSRSSymbols.
%   HoppingOffset     - Hopping offset (F_b) per OFDM symbol for 
%                       b = (BHop+1)...BSRS. HoppingOffset is a matrix of
%                       size (BSRS-BHop)-by-NumSRSSymbols for BHop < BSRS.
%   PRBSet            - Resource blocks allocated for SRS per OFDM symbol.
%                       PRBSet is a matrix of size NRB-by-NumSRSSymbols,
%                       where NRB is the number of RBs per OFDM symbol.
%    
%   IND = nrSRSIndices(...,NAME,VALUE,...) specifies additional options as
%   NAME,VALUE pairs to allow control over the format of the indices:
%
%   'IndexStyle'     - 'index' for linear indices (default)
%                      'subscript' for [subcarrier, symbol, antenna] 
%                       subscript row form
%
%   'IndexBase'      - '1based' for 1-based indices (default) 
%                      '0based' for 0-based indices
%
%   Example: 
%   % Generate indices for a 2-port SRS transmission of 4 OFDM symbols.
%   
%   carrier = nrCarrierConfig;
% 
%   srs = nrSRSConfig;
%   srs.NumSRSPorts = 2;
%   srs.NumSRSSymbols = 4;
%   srs.SymbolStart = 8;
%   srs.FrequencyStart = 2;
%   srs.CSRS = 5; 
%   srs.BSRS = 0;
% 
%   indices = nrSRSIndices(carrier,srs);
%
%   See also nrCarrierConfig, nrSRSConfig, nrSRS.

%   Copyright 2019-2023 The MathWorks, Inc.

%#codegen

    narginchk(2,6);
    
    srs = validateInputs(carrier,srs);
    
    % PV pair check
    opts = nr5g.internal.parseOptions('nrSRSIndices',{'IndexStyle','IndexBase','MultiColumnIndex'},varargin{:},'MultiColumnIndex',true);

    % Lowest subcarrier allocated to SRS for each antenna port
    k0bar = frequencyPositionOffset(carrier,srs);

    % Frequency hopping offset in subcarriers and other intermediate
    % calculations for the information output
    [nFH,nb,Fb,mSRS_BSRS] = frequencyHoppingOffset(carrier,srs);

    % Frequency offset associated to Rel-17 start RB hopping in subcarriers
    nRPFS = frequencyScalingOffset(carrier,srs);
    
    % Frequency-domain starting position for each antenna port (rows) and
    % OFDM symbol (columns) in subcarriers
    k0 = k0bar + nFH + nRPFS;
    
    % Number of subcarriers per resource block
    NRBsc = 12;

    % Validate location of the SRS within the resource grid
    PF = srs.FrequencyScalingFactor;
    FSRS = [floor(min(k0(:))/NRBsc),floor(max(k0(:))/NRBsc+mSRS_BSRS/PF)-1];
    lastCRB = carrier.NSizeGrid-1;
    if FSRS(2)-FSRS(1) > lastCRB
        coder.internal.error('nr5g:nrSRS:InconsistentBandwidthConfiguration',lastCRB,FSRS(1),FSRS(2),"CSRS");
    elseif FSRS(2) > lastCRB
        coder.internal.error('nr5g:nrSRS:InconsistentBandwidthConfiguration',lastCRB,FSRS(1),FSRS(2),"FrequencyStart");
    end

    % WTDM sequence. This sequence can result in zero-valued SRS in some
    % resource elements, including full OFDM symbols and SRS ports. Obtain
    % the number of SRS ports in use after the TDM sequence is applied.
    NPorts = srs.NumSRSPorts;
    wtdm = nr5g.internal.srs.getWTDM(srs.NumSRSSymbols,NPorts,srs.EnableEightPortTDM);

    % Determine if the slot must contain SRS (TS 38.211 Section 6.4.1.4.4)
    isCandidateSlot = nr5g.internal.isRSCandidateSlot(carrier.NSlot,carrier.NFrame,carrier.SlotsPerFrame,srs.SRSPeriod);

    if isCandidateSlot
        % Extract parameters required for subscripts calculation. Use
        % NumSRSPorts before applying WTDM.
        KTC = srs.KTC;
        NSym = srs.NumSRSSymbols;

        % Calculate 0-based subscripts
        Msc = mSRS_BSRS*NRBsc/(KTC*PF);
        K = permute(k0,[3 2 1]) + (0:Msc-1)' * KTC;
        L = repmat( srs.SymbolStart + (0:NSym-1), [Msc,1,NPorts] );
        P = repmat(permute(0:NPorts-1,[1 3 2]),[Msc NSym]);
        PRBSet = floor(k0(1,:)/NRBsc) + (0:floor((Msc-1)*KTC/NRBsc)).';

        % Apply WTDM sequence to indices and remove zeros
        K = nonzeros((K+1).*wtdm)-1;
        L = nonzeros((L+1).*wtdm)-1;
        P = nonzeros((P+1).*wtdm)-1;
    else
        K = [];
        L = [];
        P = [];
        PRBSet = [];
    end

    % Apply indices options
    gridsize = [double(carrier.NSizeGrid)*NRBsc,carrier.SymbolsPerSlot,NPorts];
    ind = nr5g.internal.applyIndicesOptions(gridsize,opts,K(:),L(:),P(:));

    % Create info structure
    info = struct();
    info.SubcarrierOffset = k0;
    info.FreqIndex = nb; 
    info.HoppingOffset = Fb;
    info.PRBSet = PRBSet;
end

% Calculate the frequency position offset k0bar measured in subcarriers for
% each antenna port based on the comb number, comb offset, number of cyclic
% shifts and frequency start.
function k0bar = frequencyPositionOffset(carrier,srs)

    NPorts = srs.NumSRSPorts;
    KTC    = srs.KTC;
    kBarTC = srs.KBarTC;
    nShift = srs.FrequencyStart;
    NCS    = srs.CyclicShift;

    % Number of subcarriers per resource block
    NRBsc = 12; 

    % Max number of cyclic shifts based on comb type    
    maxcs = [8 12 6];
    nCSmax = maxcs(log2(KTC));

    ports8tdm = srs.EnableEightPortTDM;
    p = 0:NPorts-1;
    if ports8tdm
        NBarAP = 4;
        pBar = 1000 + mod(p,2);
        pBar(5:8) = pBar(5:8) + 2;
    else
        NBarAP = NPorts;
        pBar = p;
    end

    % All antenna ports have the same frequency position by default, i.e.,
    % kTC = kBarTC. For some combinations of the number of SRS ports and
    % number of cyclic shifts, additional frequency shifts are applied to
    % some SRS ports.
    freqShiftPort = zeros(1,NPorts);
    if NBarAP == 4 && ((nCSmax == 6) || (any(nCSmax == [8 12]) && (NCS >= nCSmax/2)))
        freqShiftPort = 2*mod(pBar,2);
    elseif NBarAP == 8
        if nCSmax == 6
            freqShiftPort = mod(pBar,4);
        elseif (nCSmax == 12) || ((nCSmax == 8) && (NCS >= nCSmax/2))
            freqShiftPort = 2*mod(pBar,2);
        end
    end
    kTC = mod(kBarTC + freqShiftPort*KTC/4, KTC);
    
    if srs.SRSPositioning
        kOff = nr5g.internal.srs.SRSOffsetK(srs.KTC,srs.NumSRSSymbols);
    else
        kOff = zeros(1,srs.NumSRSSymbols);
    end

    % Frequency comb offset hopping
    fcoh = getfcoh(carrier,srs);
    
    k0bar = nShift*NRBsc + mod(kTC(:) + kOff + fcoh,KTC);
end

% Comb offset hopping
function fcoh = getfcoh(carrier,srs)

    coder.varsize('combOffsetHoppingSubset',[Inf Inf],[1 1]);

    SymbolStart = srs.SymbolStart;
    KTC = srs.KTC;
    combOffsetHopping = srs.CombOffsetHopping;
    combOffsetHoppingID = srs.CombOffsetHoppingID;
    combOffsetHoppingSubset = srs.CombOffsetHoppingSubset;
    hoppingWithRepetition = srs.HoppingWithRepetition;

    if combOffsetHopping

        % Determine the comb offset hopping subset and its cardinality
        if isempty(combOffsetHoppingSubset)
            ncoh = KTC;
            combOffsetHoppingSubset = 0:KTC-1;
        else
            ncoh = numel(combOffsetHoppingSubset);
        end
        
        % Relative slot and frame numbers
        [nsf,nf] = nr5g.internal.getRelativeNSlotAndSFN(double(carrier.NSlot),double(carrier.NFrame),carrier.SlotsPerFrame);
        symPerSlot = carrier.SymbolsPerSlot;
        symbolsPerFrame = carrier.SlotsPerFrame*symPerSlot;
        
        % Index of the first bit of the PRBS
        firstBitIndex = 8*(mod(nf,128)*symbolsPerFrame + nsf*symPerSlot + SymbolStart);
        
        % Relative SRS symbol position with repetition (l'') and length of
        % the PRBS sequence used for comb offset hopping. This length of
        % the sequence is equal to 8 times the number of SRS symbols except
        % when comb offset hopping with repetition in enabled, which may be
        % reduced by the repetition factor.
        lp = (0:srs.NumSRSSymbols-1);
        if ~hoppingWithRepetition
            lpp = lp;
        else
            R = srs.Repetition;
            lpp = floor(lp/R)*R;
        end
        prbsSeqLength = 8*(lpp(end)+1);

        % Pseudorandom binary sequence
        PRBS = nrPRBS(combOffsetHoppingID, [firstBitIndex prbsSeqLength]);

        % Reorganize PRBS indices for OFDM symbol processing
        PRBSm = reshape(PRBS,8,lpp(end)+1);

        % Index PRBS sequence in the dimension corresponding to unique OFDM
        % symbols (l'')
        PRBSm = PRBSm(:,lpp+1);
        
        % Comb offset for each OFDM symbol by converting 8-bit blocks of
        % the PRBS to mod-ncoh integer
        m = (0:7)';
        cohInd = mod(2.^m'*PRBSm, ncoh);
        fcoh = combOffsetHoppingSubset(1+cohInd);

    else
        fcoh = 0;
    end

end

% Frequency hopping offset in subcarriers and other intermediate
% calculations for the information output
function [nFH,nb,Fb,mSRS_BSRS] = frequencyHoppingOffset(carrier,srs)

    [nb,Fb] = frequencyPositionIndex(carrier,srs);
    
    % NRBs for all b = 0:BSRS
    mSRSb = nr5g.internal.srs.SRSBandwidthConfiguration(srs.CSRS,0:srs.BSRS); 
    mSRS_BSRS = mSRSb(end);

    % Number of subcarriers per resource block
    NRBsc = 12; 

    % Frequency hopping offset
    nFH = NRBsc*mSRSb'*nb;

end


% Calculate the frequency position index n_b and hopping offset Fb of the SRS
% (see TS 38.211 Section 6.4.1.4.3)
function [nb,Fb] = frequencyPositionIndex(carrier,srs)

    CSRS = srs.CSRS; 
    BSRS = srs.BSRS;
    BHop = srs.BHop;
    NRRC = srs.NRRC;

    % Number of OFDM symbols containing SRS symbols in the current slot 
    NSym = srs.NumSRSSymbols; 

    % Bandwidth configuration
    b = 0:BSRS;
    [mSRSb,Nb] = nr5g.internal.srs.SRSBandwidthConfiguration(CSRS,b);
    
    nb = repmat(mod(floor(4*NRRC./mSRSb), Nb),1,NSym);
    
    Fb = [];
    if BHop < BSRS % Frequency hopping        
        idx = BHop < b;
        
        % Number of SRS transmissions (SRS symbol counter)
        nSRS = numberSRSTransmissions(carrier,srs);
    
        % Function Fb
        Fb = hoppingOffset(b(idx),nSRS,CSRS,BHop);
        nb(idx,:) = mod(Fb+nb(idx,:), Nb(idx));
    end
    
end

% Calculate the frequency offset associated to the Rel-17 partial frequency
% sounding
function nRPFS = frequencyScalingOffset(carrier,srs)

    % Number of subcarriers per resource block
    NRBsc = 12;

    % Extract relevant parameters
    PF = srs.FrequencyScalingFactor;
    kF = srs.StartRBIndex;
    BHop = srs.BHop;

    % SRS sequence length and number of subbands for b = 0:BSRS
    b = 0:srs.BSRS;
    [mSRSb,Nb] = nr5g.internal.srs.SRSBandwidthConfiguration(srs.CSRS,b);
    mSRS_BSRS = mSRSb(end);

    if srs.EnableStartRBHopping && (PF>1)
        % Calculate the frequency hopping offset kBarHop and read the
        % associated RB offset (kHop) from TS 38.211 Table 6.4.1.4.3-3
        Nbp = Nb(b>=BHop);
        Nbp(1) = 1;
        nSRS = numberSRSTransmissions(carrier,srs);
        kBarHop = mod(floor(nSRS/prod(Nbp)),PF);
        kHop = nr5g.internal.srs.SRSStartRBHoppingOffset(kBarHop,PF);
    else
        kHop = 0;
    end
    
    % Partial frequency sounding hopping offset in subcarriers
    nRPFS = NRBsc*mSRS_BSRS*mod(kF+kHop,PF)/PF;
end

% Return the 0-based number of unique SRS transmissions nSRS measured in
% OFDM symbols. nSRS is measured from the origin of the radio frame for
% periodic and semi-persistent resource type configurations, and from the
% first SRS OFDM symbol transmitted in the current slot for aperiodic
% resource type configurations.
function nSRS = numberSRSTransmissions(carrier,srs)

    [Tsrs,Toff] = nr5g.internal.getRSPeriodicityAndOffset(srs.SRSPeriod);

    % Number of slots per frame 
    Nsf = carrier.SlotsPerFrame;

    NSym = srs.NumSRSSymbols;
    Rep  = srs.Repetition;
    
    % OFDM sym number in SRS resource
    LPrime = 0:NSym-1;
    
    % Relative frame and slot numbers
    NFrame = double(carrier.NFrame);
    NSlot  = double(carrier.NSlot);
    [NSlot,NFrame] = nr5g.internal.getRelativeNSlotAndSFN(NSlot,NFrame,carrier.SlotsPerFrame);

    % Scaling factor accounting for TDM
    if srs.EnableEightPortTDM
        s = 2;
    else
        s = 1;
    end

    % Number of SRS transmissions (repeated transmissions count as 1)
    if strcmpi(srs.ResourceType,'aperiodic')
        nSRS = floor(LPrime/(s*Rep)); 
    else
        nSRS = (NSym/(s*Rep))*(Nsf*NFrame + NSlot - Toff)/Tsrs + floor(LPrime/(s*Rep));
    end
end

% Calculate the frequency hopping offset function Fb as described in TS
% 38.211 Section 6.4.1.4.3. The parameter b (0...3) denotes the subscript
% of the function Fb and nSRS the number of SRS transmissions.
function Fb = hoppingOffset(b,nSRS,CSRS,BHop)

    LenB = length(b);
    Fb = zeros(LenB, length(nSRS));
    for i = 1:LenB
        % Bandwidth configuration parameters
        [~,Nbp] = nr5g.internal.srs.SRSBandwidthConfiguration(CSRS, BHop:b(i));

        Nb = Nbp(end);

        % Nbhop = 1 regardless of the value of Nb
        Nbp(1) = 1;
        
        NbOdd = mod(Nb,2);
        if NbOdd % Function Fb for odd values of Nb
            Fb(i,:) = floor(Nb/2)*floor(nSRS/prod(Nbp(1:end-1)));
        else
            argument = mod(nSRS, prod(Nbp))/prod(Nbp(1:end-1));
            Fb(i,:) = Nb/2*floor(argument) + floor(argument/2);
        end
    end
    
end

function srs = validateInputs(carrier,srs)
    fcnName = 'nrSRSIndices';
    validateattributes(carrier, {'nrCarrierConfig'}, {'scalar'}, fcnName, 'Carrier-specific configuration object');
    validateattributes(srs, {'nrSRSConfig'}, {'scalar'}, fcnName, 'SRS-specific configuration object');
    
    srs = validateConfig(srs);

    % Cross validation of properties
    SymbolStart = srs.SymbolStart;
    NumSRSSymbols = srs.NumSRSSymbols;
    if (SymbolStart + NumSRSSymbols) > carrier.SymbolsPerSlot
        cpOffset = 2*(carrier.SymbolsPerSlot==12);
        coder.internal.error('nr5g:nrSRS:InconsistentTimeAllocation',SymbolStart,NumSRSSymbols,SymbolStart+NumSRSSymbols-1,13-cpOffset,carrier.CyclicPrefix);
    end 
end