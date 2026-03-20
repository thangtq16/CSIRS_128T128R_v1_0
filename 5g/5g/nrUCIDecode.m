function [uciBits,errVec] = nrUCIDecode(uciLLRs,A,varargin)
%nrUCIDecode Uplink control information decoding
%   UCIBITS = nrUCIDecode(UCILLRS,A) decodes the input soft bits UCILLRS as
%   per TS 38.212 Sections 6.3.1.5, 6.3.1.4, 6.3.1.3, and 6.3.1.2 (for
%   PUCCH), to output the decoded UCI bits UCIBITS of length A. The
%   processing includes rate recovery, channel decoding and CRC decoding
%   per code-block. Corresponding sections for PUSCH (6.3.2.5, 6.3.2.4,
%   6.3.2.3, and 6.3.2.2) are also covered by the same processing.
%
%   The decoding scheme employed is based on the number of output decoded
%   bits. This is specified by A, and is as per the following table:
%     A          De-concatenation         Decoding             CRC bits
%    1...11             NA                ML                   NA
%    12...19            NA                CRC-Aided SCL        6
%    20...1706   Conditioned on A,EUCI    CRC-Aided SCL        11
%
%   The input UCILLRS must be a column vector of soft bits (LLRs) of length
%   EUCI and the output UCIBITS is the decoded UCI message of length A.
%
%   UCIBITS = nrUCIDecode(UCILLRS,A,MODULATION) decodes the input soft bits
%   UCILLRS for 1 or 2 output bits, for the modulation scheme MODULATION
%   specified as one of 'pi/2-BPSK','QPSK','16QAM','64QAM','256QAM'.
%   MODULATION applies when A is equal to 1 or 2 only, and if not
%   specified, defaults to 'QPSK'.
%
%   UCIBITS = nrUCIDecode(UCILLRS,A,'ListLength',L) or
%   UCIBITS = nrUCIDecode(UCILLRS,A,MODULATION,'ListLength',L) decodes the
%   input soft bits UCILLRS with the specified list length L for polar
%   decoding. A default value of 8 is used when list length is not
%   specified.
%
%   [UCIBITS,ERRVEC] = nrUCIDecode(...) also outputs an error flag(s) to
%   indicate if the block(s) was decoded in error or not (true indicates an
%   error). This output applies only for schemes that use a CRC and
%   represents a boolean value per codeblock decoded.
%
%   % Example 1:
%   % Decode an encoded UCI block using a list length of 4 and check the
%   % recovered elements.
%
%   A = 32;
%   EUCI = 120;
%   L = 4;
%   uciBits = randi([0 1],A,1);
%   uciCW   = nrUCIEncode(uciBits,EUCI);
%   [recBits,err] = nrUCIDecode(1-2*uciCW,A,'ListLength',L);
%   isequal(recBits,uciBits)
%   err
%
%   % Example 2:
%   % Decode a 2-bit UCI encoded block for 16QAM modulation with AWGN.
%
%   snrdB = 0;
%   K = 2;
%   modScheme = '16QAM';
%   EUCI = 4*3;
%   uci = randi([0 1],K,1,'int8');
%   encUCI = nrUCIEncode(uci,EUCI,modScheme);
%   % Replace placeholder bits, via scrambling
%   encUCI(encUCI==-1) = 1;
%   encUCI(encUCI==-2) = encUCI(find(encUCI==-2)-1);
%
%   modOut = nrSymbolModulate(encUCI,modScheme);
%   rxSig = awgn(modOut,snrdB);
%   rxSoftBits = nrSymbolDemodulate(rxSig,modScheme);
%
%   decBits = nrUCIDecode(rxSoftBits,K,modScheme);
%   isequal(decBits,uci)
%
%   See also nrUCIEncode, nrRateRecoverPolar, nrPolarDecode, nrCRCDecode.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

%   Reference:
%   [1] 3GPP TS 38.212, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Multiplexing and channel
%   coding (Release 15). Section 6.3.

    % Parse inputs
    narginchk(2,5);
    if nargin==2            % nrUCIDecode(uciLLRs,A)
        modScheme = 'QPSK';
        prmL = 'ListLength';
        L = 8;
    elseif nargin==3        % nrUCIDecode(uciLLRs,A,modScheme)
        modScheme = varargin{1};
        prmL = 'ListLength';
        L = 8;
    elseif nargin==4        % nrUCIDecode(uciLLRs,A,'ListLength',L)
        modScheme = 'QPSK';
        prmL = varargin{1};     % should be 'ListLength'
        L = varargin{2};
    else                    % nrUCIDecode(uciLLRs,A,modScheme,'ListLength',L)
        modScheme = varargin{1};
        prmL = varargin{2};     % should be 'ListLength'
        L = varargin{3};
    end

    % Empty in, empty out
    if isempty(uciLLRs)
        uciBits = zeros(0,1,'int8');
        errVec = false;
        return;
    end

    % Validate inputs
    modulation = validateInputs(uciLLRs,A,modScheme,prmL,L);

    Euci = length(uciLLRs);

    if A>=12 % Polar decoding (both Parity-Check and CRC-Aided)

        % Check for multiple code-blocks in input
        if (A>=1013) || (A>=360 && Euci>=1088) % Use Euci here
            C = 2;
            Ap = ceil(A/C);
        else
            C = 1;
            Ap = A;
        end

        % Get CRC lengths
        if Ap>=20
            Lcrc = 11;
            crcPoly = '11';
        else % 12<=A<=19
            Lcrc = 6;
            crcPoly = '6';
        end

        K = Ap+Lcrc;             % K includes CRC bits
        Er = floor(Euci/C);
        % Check Er limits, and feedback in terms of input length
        % higher in general, lower for PC-Polar
        coder.internal.errorIf(Er>8192 || Er<=21, ...
            'nr5g:nrUCIDecode:InvalidInputLength',Er);

        nMax = 10;               % for uplink
        N = nr5g.internal.polar.getN(K,Er,nMax);
        iIL = false;
        iBIL = true;

        decBlk = zeros(Ap+Lcrc,C);
        for cIdx = 1:C
            % Code block de-concatenation
            uciCW = uciLLRs((cIdx-1)*Er+(1:Er),1);

            % Rate recovery, Sections 6.3.1.4.1/6.3.2.4.1, 5.4.1 [1]
            recBlk = nrRateRecoverPolar(uciCW,K,N,iBIL);

            % Polar decoding, Sections 6.3.1.3.1/6.3.2.3.1, 5.3.1 [1]
            decBlk(:,cIdx) = nrPolarDecode(recBlk,K,Er,L,nMax,iIL,Lcrc);
        end

        % CRC decoding, Sections 6.3.1.2/6.3.2.2, 5.1 [1]
        [uciCBs,err] = nrCRCDecode(int8(decBlk),crcPoly);
        errVec = (err~=0);  % logical output

        % Code block desegmentation
        tmp = uciCBs(:);
        uciBits = tmp(end-A+1:end); % remove the filler, if present

    else % A<12

        % For small block lengths
        if A<3  % for 1,2
            uciBits = smallDecode12(uciLLRs,A,modulation);
        else    % for 3...11
            uciBits = smallDecode311(uciLLRs,A);
        end
        errVec = false(0); % unused, not defined.
    end

end

function modulation = validateInputs(uciLLRs,A,modScheme,prmL,L)
% Check inputs

    fcnName = 'nrUCIDecode';

    % Validate input soft bits
    validateattributes(uciLLRs,{'single','double'},{'real','column'}, ...
        fcnName,'UCILLRS');

    % Validate length of output bits, less than or equal to 1706
    validateattributes(A,{'numeric'}, ...
        {'scalar','integer','positive','<=',1706},fcnName,'A');

    % Validate input length which must be greater than or equal to minE
    % (based on A)
    minE = nr5g.internal.getMinUCIBitCapacity(A);
    validateattributes(length(uciLLRs),{'double'},...
        {'scalar','>=',minE},fcnName,'length of UCILLRS');

    % Validate modScheme only if needed
    if A<3
        % Validate modScheme only if needed
        modlist = {'pi/2-BPSK','QPSK','16QAM','64QAM','256QAM'};
        modulation = validatestring(modScheme,modlist,fcnName,'MODULATION');
    else
        modulation = 'QPSK';

        if A>=20
            % Validate decoding list length: param name and value, only if
            % needed
            validatestring(prmL,{'ListLength'},fcnName,'ListLength');
            nr5g.internal.validateParameters('ListLength',L,fcnName);
        end
    end

end

function decBits = smallDecode12(uciLLRs,A,modScheme)
%smallDecode12 Small block length decoding for 1 or 2 bits

    % Get modulation order
    Qm = nr5g.internal.getQm(modScheme);

    if A==1
        N = Qm;
    else
        N = 3*Qm;
    end

    % Rate recovery
    typeIn = class(uciLLRs);
    E = length(uciLLRs);
    rec = zeros(N,1,typeIn);
    if E>=N
        % Stack block repetitions in columns and soft combine by adding
        % columns together
        inRep = zeros(N,ceil(E/N),typeIn);
        inRep(1:E) = uciLLRs;
        rec = sum(inRep,2);
    else
        rec(1:E) = uciLLRs;
    end

    % Decode: ML or exhaustive search
    %   Form all soft messages
    m = 2^A;
    allMsgs = int2bit(0:m-1,A,false)'; % one per row
    softEncMsgs = zeros(N,m,typeIn);
    for i = 1:m
        encMsg = nr5g.internal.smallEncode12(allMsgs(i,:).',modScheme);
        % Replace placeholder bits
        encMsg(encMsg==-1) = 1;
        encMsg(encMsg==-2) = encMsg(find(encMsg==-2)-1);
        softEncMsgs(:,i) = 1-2.*cast(encMsg,typeIn); % negative mapping
    end
    %   Euclidean distance metric
    distMet = sum(abs(repmat(rec,1,m) - softEncMsgs).^2,1)./sum(abs(softEncMsgs).^2,1);
    %   Select the msg bits corresponding to the minimum
    decBits = cast(allMsgs(distMet==min(distMet),:).','int8');

end

function decBits = smallDecode311(uciLLRs,A)
%smallDecode311 Small block length decoding for 3...11 bits

    N = 32; % codeword length

    % Rate recovery
    typeIn = class(uciLLRs);
    E = length(uciLLRs);
    rec = zeros(N,1,typeIn);
    if E>=N
        % Stack block repetitions in columns and soft combine by adding
        % columns together
        inRep = zeros(N,ceil(E/N),typeIn);
        inRep(1:E) = uciLLRs;
        rec = sum(inRep,2);
    else
        rec(1:E) = uciLLRs;
    end

    % Decode: ML or exhaustive search
    %   Form all soft messages
    m = 2^A;
    allMsgs = int2bit(0:m-1,A,false)'; % one per row
    softEncMsgs = zeros(N,m,typeIn);
    for i = 1:m
        encMsg = nr5g.internal.smallEncode311(allMsgs(i,:));
        softEncMsgs(:,i) = 1-2.*cast(encMsg,typeIn); % negative mapping
    end
    %   Euclidean distance metric
    distMet = sum(abs(repmat(rec,1,m) - softEncMsgs).^2,1)./sum(abs(softEncMsgs).^2,1);
    %   Select the msg bits corresponding to the minimum
    decBits = cast(allMsgs(distMet==min(distMet),:).','int8');

end
