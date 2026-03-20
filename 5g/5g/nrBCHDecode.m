function varargout = nrBCHDecode(cdBlk,L,varargin)
%nrBCHDecode Broadcast transport channel decoder
%   SCRBLK = nrBCHDecode(SOFTBITS,L) decodes the input log-likelihood
%   ratios SOFTBITS, as per TS 38.212 Sections 7.1 to output the decoded
%   scrambled transport block SCRBLK. L is the specified list length used
%   for polar decoding. L must be a power of two and a value of 8 is
%   commonly acceptable for the block lengths decoded.
%   The input SOFTBITS are the log-likelihood ratio values representing the
%   received coded block and the output SCRBLK is the decoded, scrambled,
%   transport block of 32 bits.
%
%   [SCRBLK,ERRFLAG] = nrBCHDecode(...) also outputs an error flag to
%   indicate if the block was decoded in error or not (true indicates an
%   error).
%
%   [SCRBLK,ERRFLAG,TRBLK,SFN4LSB,HRF,MSBIDXOFFSET] = nrBCHDecode(SOFTBITS,
%   L,LSSB,NCELLID) also outputs the unscrambled transport block TRBLK and
%   other information elements, given the additional inputs to unscramble
%   the decoded transport block, if the latter was decoded with no errors.
%   Additional outputs are:
%   TRBLK        - 24-bit column vector comprising the decoded transport
%                  block. The transport block represents a 
%                  BCCH-BCH-Message (TS 38.331 Section 6.2.1), containing
%                  the MIB (TS 38.331 Section 6.2.2)
%   SFN4LSB      - 4 LSBs of SFN as a column vector
%   HRF          - Half frame bit
%   MSBIDXOFFSET - 3 MSBs SS Block index or 1 MSB Kssb, LSSB dependent
%   Additional inputs are:
%   LSSB    - number of candidate SS/PBCH blocks in a half frame (4/8/64)
%   NCELLID - an integer representing NCellID (0...1007)
%
%   Example:
%   % Decode an encoded BCH transport block and check the recovered 
%   % information elements. The transport block created is a vector of 
%   % random bits, but in the NR system this would be a fully formatted 
%   % BCCH-BCH-Message.
%
%   nid = 321;                        % NCellID
%   trblk = randi([0 1],24,1,'int8'); % BCH transport block
%   sfn = 10;                         % SFN as a decimal 
%   hrf = 1;                          % Half frame bit
%   lssb = 8;                         % Number of SS/PBCH blocks
%   kssb = 18;                        % Subcarrier offset (0...31)
%   bch = nrBCH(trblk,sfn,hrf,lssb,kssb,nid);
% 
%   % Decode and recover information
%   listLen = 8;                      % Polar decoding list length
%   [~,errFlag,rxtrblk,rxSFN4lsb,rxHRF,rxKssb] = nrBCHDecode( ...
%       double(1-2*bch),listLen,lssb,nid);
%
%   % Check outputs
%   errFlag
%   isequal(trblk,rxtrblk)
%   isequal(bit2int(rxSFN4lsb,4),mod(sfn,16))
%   [isequal(hrf,rxHRF) isequal(int2bit(floor(kssb/16),1,false),rxKssb)]
%
%   See also nrBCH, nrPBCH, nrPBCHDecode.

%   Copyright 2018-2021 The MathWorks, Inc.

%#codegen

%   References:
%   [1] 3GPP TS 38.211, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Physical Channel and
%   Modulation (Release 15). Section 7.3.3.
%   [2] 3GPP TS 38.212, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Multiplexing and channel
%   coding (Release 15). Section 7.1.
%   [3] 3GPP TS 38.213, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Physical layer
%   procedures for control (Release 15). Section 4.1.
%   [4] 3GPP TS 38.331, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Radio Resource Control 
%   (RRC) protocol specification (Release 15). Section 6.2.

    narginchk(2,4);
    nargoutchk(0,6);

    % Validate mandatory inputs
    validateInputs(cdBlk,L);

    E = length(cdBlk); % E must be equal to 864
    K = 56;
    N = 512; 

    % Rate recovery, Section 7.1.5, [2]
    recBlk = nrRateRecoverPolar(cdBlk,K,N);

    % Polar decoding, Section 7.1.4, [2]
    decBlk = nrPolarDecode(recBlk,K,E,L);

    % CRC decoding, Section 7.1.3, [2]
    [scrBlk,errFlag] = nrCRCDecode(decBlk,'24C');

    varargout{1} = scrBlk;
    varargout{2} = errFlag;    
    if nargin>2  
        coder.internal.errorIf( nargin~=4,'nr5g:nrBCHShared:InvalidNumIn');

        Lssb = varargin{1};
        NcellID = varargin{2};
        validateOptInputs(Lssb,NcellID);

        A = length(scrBlk); % A is equal to 32
        aBar = false(A,1);
        if ~errFlag % No error
            % Assumes more information can be decoded from scrBlk

            % G(j) as per Table 7.1.1-1, [2], 0-based
            G = [16 23 18 17 8 30 10 6 24 7 0 5 3 2 1 4 ...
                  9 11 12 13 14 15 19 20 21 22 25 26 27 28 29 31];

            % Get v from scrBlk, which is not scrambled, just interleaved
            jSFN = 7;
            v = comm.internal.utilities.convertBit2Int(scrBlk([G(jSFN+1)+1 G(jSFN+2)+1],1),2);

            % Descrambling, Section 7.1.2, [2]
            isScrambled = true(A,1);
            jSFN = 0;
            jHRF = 10;
            jSSB = 11;
            for idx = 0:A-1     % 0-based
                if ((idx>0 && idx<=6) || (idx>=24 && idx<=27)) % SFN
                    % Bits with idx 1...6 are the 6 MSBs of the SFN in the
                    % MIB, see TS 38.331 Section 6.2.2. Bits with idx
                    % 24...27 are the 4 LSBs of the SFN conveyed in
                    % additional PBCH payload bits, see TS 38.212 Section
                    % 7.1.1
                    if (idx==25) || (idx==26)
                        isScrambled(G(jSFN+1)+1) = false;
                    end
                    jSFN = jSFN+1;
                elseif (idx==28)                    % HRF
                    isScrambled(G(jHRF+1)+1) = false;
                elseif (idx>=29 && idx<=31)         % SSB
                    if (Lssb==64)
                        isScrambled(G(jSSB+1)+1) = false;
                    end
                    jSSB = jSSB+1;
                end
            end

            if Lssb==64 
                M = A-6;               % M is the number of bits scrambled
            else    % Lssb is either 4 or 8
                M = A-3;
            end
            seqSet = nrPRBS(NcellID,[v*M M]); % binary, logical values
            seq = false(A,1);
            seq(isScrambled,1) = seqSet;      % Scramble the ones needed
            trBlk = xor(logical(scrBlk),seq);

            % Decode the payload
            jSFN = 0;
            jHRF = 10;
            jSSB = 11;
            jOTH = 14;
            for idx = 0:A-1     % 0-based
                if ((idx>0 && idx<=6) || (idx>=24 && idx<=27)) % SFN
                    % Bits with idx 1...6 are the 6 MSBs of the SFN in the
                    % MIB, see TS 38.331 Section 6.2.2. Bits with idx
                    % 24...27 are the 4 LSBs of the SFN conveyed in
                    % additional PBCH payload bits, see TS 38.212 Section
                    % 7.1.1
                    aBar(idx+1,1) = trBlk(G(jSFN+1)+1,1);
                    jSFN = jSFN+1;
                elseif (idx==28)                    % HRF
                    aBar(idx+1,1) = trBlk(G(jHRF+1)+1,1);
                elseif (idx>=29 && idx<=31)         % SSB
                    aBar(idx+1,1) = trBlk(G(jSSB+1)+1,1);
                    jSSB = jSSB+1;
                else                                % Other 
                    aBar(idx+1,1) = trBlk(G(jOTH+1)+1,1);
                    jOTH = jOTH+1;
                end
            end
        end

        if nargout>2
            % Assign outputs, individually below
            varargout{3} = aBar(1:24,1);  % BCCH-BCH-Message (containing MIB)
            varargout{4} = aBar(25:28,1); % SFN 4 LSBs
            varargout{5} = aBar(29,1);    % HRF
            if Lssb==64
                % MSB (6,5,4th) of SS Block index
                varargout{6} = aBar(30:32,1);
            else
                % MSB of Kssb 
                varargout{6} = aBar(30,1); 
            end
            % In errant case, all would be false.
        end
    end

end

function validateInputs(cdBlk,L)
% Check mandatory inputs

    fcnName = 'nrBCHDecode';

    % Validate input soft data, length must be greater than or equal to 512
    validateattributes(cdBlk,{'single','double'},{'real','2d','column'}, ...
        fcnName,'SOFTBITS');
    coder.internal.errorIf( length(cdBlk)<512,'nr5g:nrBCHShared:InvalidIn');

    % Validate decoding list length
    nr5g.internal.validateParameters('ListLength',L,fcnName);

end

function validateOptInputs(Lssb,NcellID)
% Check optional inputs

    fcnName = 'nrBCHDecode';

    % Validate number of SS/PBCH blocks in a half frame (4/8/64)
    validateattributes(Lssb, {'numeric'}, ...
        {'scalar','integer','finite'},fcnName,'LSSB');
    coder.internal.errorIf(~any(Lssb==[4 8 64]),'nr5g:nrBCHShared:InvalidLSSB');

    % Validate physical layer cell identity (0...1007)
    validateattributes(NcellID, {'numeric'}, ...
        {'scalar','nonnegative','integer','<=',1007},fcnName,'NCELLID');

end
