function cdBlk = nrBCH(trblk,sfn,hrf,Lssb,idxOffset,NcellID)
%nrBCH Broadcast transport channel
%   CDBLK = nrBCH(TRBLK,SFN,HRF,LSSB,IDXOFFSET,NCELLID) encodes the input
%   transport block TRBLK, as per TS 38.212 Section 7.1, to output the BCH
%   transport channel coded block CDBLK.
%   The input TRBLK represents the BCCH-BCH-Message (containing the MIB)
%   and must be a binary column vector of length 24. BCCH-BCH-Message is
%   defined in TS 38.331 Section 6.2.1 and the MIB in Section 6.2.2. Other
%   inputs are:
%   SFN       - An integer representing the System Frame Number
%   HRF       - Half frame bit. Set to 0 for SS/PBCH block transmissions in
%               the first half of a frame, or 1 for transmissions in the 
%               second half of a frame (See TS 38.214 Section 4.1)
%   LSSB      - Number of candidate SS/PBCH blocks in a half frame (4/8/64)
%   IDXOFFSET - SS block index (0...63) for LSSB equal to 64, or
%               Kssb (subcarrier offset, 0...31) for LSSB equal to 4 or 8
%   NCELLID   - An integer representing NCellID (0...1007)
%   
%   CDBLK is the output coded block of 864 bits, returned as a binary
%   column vector.
%
%   Example:
%   % Encode a transport block with Lssb as 8 and 64. The transport block
%   % created is a vector of random bits, but in the NR system this would 
%   % be a fully formatted BCCH-BCH-Message.
%
%   nid = 321;                        % NCellID
%   trblk = randi([0 1],24,1,'int8'); % BCH transport block
%   sfn = 10;                         % SFN, as a decimal 
%   hrf = 1;                          % Half frame bit
%
%   lssb = 8;                         % Number of SS/PBCH blocks
%   kssb = 18;                        % Subcarrier offset (0...31)
%   bchCW = nrBCH(trblk,sfn,hrf,lssb,kssb,nid);
%
%   lssb = 64;                        % Number of SS/PBCH blocks
%   ssbIdx = 13;                      % SS Block index (0...63)
%   bchCW2 = nrBCH(trblk,sfn,hrf,lssb,ssbIdx,nid);
%
%   See also nrBCHDecode, nrPBCHDecode, nrPBCH.

%   Copyright 2018-2021 The MathWorks, Inc.

%#codegen

%   References:
%   [1] 3GPP TS 38.211, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Physical Channel and
%   Modulation (Release 15). Section 7.3.3, 7.4.3.
%   [2] 3GPP TS 38.212, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Multiplexing and channel
%   coding (Release 15). Section 7.1.
%   [3] 3GPP TS 38.213, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Physical layer
%   procedures for control (Release 15). Section 4.1.
%   [4] 3GPP TS 38.331, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Radio Resource Control 
%   (RRC) protocol specification (Release 15). Section 6.2.

    % Validate inputs
    validateInputs(trblk,sfn,hrf,Lssb,idxOffset,NcellID);

    % BCH payload generation, Section 7.1.1, [2]
    typein = class(trblk);
    A = length(trblk)+8;
    aBar = zeros(A,1,typein);
    aBar(1:24,1)  = trblk;
    aBar(25:28,1) = comm.internal.utilities.convertInt2Bit(mod(sfn,16),4);
    aBar(29,1)    = hrf;
    if Lssb==64
        % MSB (6,5,4th) of SS Block index
        aBar(30:32,1) = comm.internal.utilities.convertInt2Bit(floor(idxOffset/8),3);
    else % Lssb is either 4 or 8
        % MSB of Kssb (4 LSBs are in MIB as ssbOffset), reserved are set to 0
        aBar(30:32,1) = [comm.internal.utilities.convertInt2Bit(floor(idxOffset/16),1); 0; 0];
    end

    % G(j) as per Table 7.1.1-1, [2], 0-based
    G = [16 23 18 17 8 30 10 6 24 7 0 5 3 2 1 4 ...
          9 11 12 13 14 15 19 20 21 22 25 26 27 28 29 31];
    jSFN = 0;
    jHRF = 10;
    jSSB = 11;
    jOTH = 14;

    a = zeros(A,1,typein);
    isScrambled = true(A,1);
    for idx = 0:A-1     % 0-based
        if ((idx>0 && idx<=6) || (idx>=24 && idx<=27)) % SFN
            % Bits with idx 1...6 are the 6 MSBs of the SFN in the MIB, see
            % TS 38.331 Section 6.2.2. Bits with idx 24...27 are the 4 LSBs
            % of the SFN conveyed in additional PBCH payload bits, see TS
            % 38.212 Section 7.1.1
            a(G(jSFN+1)+1,1) = aBar(idx+1,1);
            if (idx==25) || (idx==26)
                isScrambled(G(jSFN+1)+1) = false;
            end
            jSFN = jSFN+1;
        elseif (idx==28)                    % HRF
            a(G(jHRF+1)+1,1) = aBar(idx+1,1);
            isScrambled(G(jHRF+1)+1) = false;
        elseif (idx>=29 && idx<=31)         % SSB
            a(G(jSSB+1)+1,1) = aBar(idx+1,1);
            if (Lssb==64)
                isScrambled(G(jSSB+1)+1) = false;
            end
            jSSB = jSSB+1;
        else                                % Other
            a(G(jOTH+1)+1,1) = aBar(idx+1,1);
            jOTH = jOTH+1;
        end
    end

    % Scrambling, Section 7.1.2, [2]
    if Lssb==64
        M = A-6;                       % M is the number of bits scrambled
    else    % Lssb is either 4 or 8
        M = A-3;
    end
    v = comm.internal.utilities.convertBit2Int(double(aBar(26:27,1)),2);
    seqSet = nrPRBS(NcellID,[v*M M]);  % binary, logical values
    seq = false(A,1);
    seq(isScrambled,1) = seqSet;       % Scramble only the ones set earlier
    trBlk = xor(logical(a),seq);

    % CRC attachment, Section 7.1.3, [2]
    msgcrc = nrCRCEncode(cast(trBlk,'int8'),'24C');

    % Channel coding, Section 7.1.4, [2]
    K = length(msgcrc);     % K must be equal to 56
    E = 864;
    encOut = nrPolarEncode(msgcrc,E);

    % Rate matching, Section 7.1.5, [2]
    cdBlk = nrRateMatchPolar(encOut,K,E);
    cdBlk = cast(cdBlk,typein);

end

function validateInputs(trblk,sfn,hrf,Lssb,idxOffset,NcellID)
% Check inputs

    fcnName = 'nrBCH';

    % Validate transport block input data of length 24
    validateattributes(trblk,{'int8','double'},{'binary','column'}, ...
        fcnName,'TRBLK');
    coder.internal.errorIf(length(trblk)~=24, 'nr5g:nrBCHShared:InvalidTrBlk');

    % Validate system frame number (SFN)
    validateattributes(sfn, {'numeric'}, ...
        {'scalar','nonnegative','integer','finite'},fcnName,'SFN');

    % Validate half frame bit (HRF)
    validateattributes(hrf, {'numeric'},{'scalar'},fcnName,'HRF');
    coder.internal.errorIf(~(hrf==1 || hrf==0),'nr5g:nrBCHShared:InvalidHRF');

    % Validate number of SS/PBCH blocks in a half frame (4/8/64)
    validateattributes(Lssb, {'numeric'}, ...
        {'scalar','integer','finite'},fcnName,'LSSB');
    coder.internal.errorIf(~any(Lssb==[4 8 64]),'nr5g:nrBCHShared:InvalidLSSB');

    % Validate SS block index/subcarrier offset
    if Lssb==64 % ssbIdx must be an integer in range (0...63)
        validateattributes(idxOffset, {'numeric'}, ...
            {'scalar','nonnegative','integer','<=',63},fcnName,'IDXOFFSET');
    else % Lssb is either 4 or 8, Kssb must be an integer in range (0...31)
        validateattributes(idxOffset, {'numeric'}, ...
            {'scalar','nonnegative','integer','<=',31},fcnName,'IDXOFFSET');
    end

    % Validate physical layer cell identity (0...1007)
    validateattributes(NcellID, {'numeric'}, ...
        {'scalar','nonnegative','integer','<=',1007},fcnName,'NCELLID');

end