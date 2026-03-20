function out = nrPolarDecode(in,K,E,L,varargin)
%nrPolarDecode Polar decode
%   DECBITS = nrPolarDecode(REC,K,E,L) decodes the rate-recovered input,
%   REC, for a (N,K) Polar code, using a CRC-aided successive-cancellation
%   list decoder, with the list length specified by L. The input REC is a
%   column vector of length N (a power of 2), representing the
%   log-likelihood ratios as soft inputs to the decoder. K is the number of
%   message bits, and E is the rate-matched output length. The output
%   DECBITS is a column vector of length K.
%
%   DECBITS = nrPolarDecode(...,PADCRC) specifies whether the input was
%   prepadded by ones prior to the CRC encoding with all-zeros register
%   state on the transmit end. PADCRC must be a boolean scalar where for a
%   true value, the input is assumed to be prepadded with ones, while a
%   false value indicates no prepadding was used. The default is false.
%
%   DECBITS = nrPolarDecode(...,PADCRC,RNTI) also specifies the RNTI value
%   that may have been used at the transmit end for masking. The default is
%   0.
%
%   DECBITS = nrPolarDecode(REC,K,E,L,NMAX,IIL,CRCLEN) specifies the three
%   parameter set of: NMAX (an integer value of either 9 or 10), IIL (a
%   boolean scalar) and CRCLEN (an integer value one of 24, 11 or 6). The
%   allowed value sets of {9,true,24} and {10,false,11},{10,false,6} for
%   {NMAX,IIL,CRCLEN} apply for downlink and uplink configurations. When
%   the three parameters are not specified, the value set for the downlink
%   configuration is used. PADCRC is assumed false and RNTI to be 0 for
%   this syntax.
%
%   % Example:
%   % Transmit polar-encoded block of data and decode using
%   % successive-cancellation list decoder.
%
%   K = 132;            % Message length
%   E = 256;            % Rate matched output length
%   nVar = 1.0;         % Noise variance
%   L = 8;              % Decoding list length
%
%   % Object construction
%   chan   = comm.AWGNChannel('NoiseMethod','Variance','Variance',nVar);
%
%   % Simulate a frame
%   msg    = randi([0 1],K,1,'int8');               % Generate random message
%   enc    = nrPolarEncode(msg,E);                  % Polar encode
%   mod    = nrSymbolModulate(enc,'QPSK');          % Modulate
%   rSig   = chan(mod);                             % Add WGN
%   rxLLR  = nrSymbolDemodulate(rSig,'QPSK',nVar);  % Soft demodulate
%   rxBits = nrPolarDecode(rxLLR,K,E,L);            % Polar decode
%
%   % Get bit errors
%   numBitErrs = biterr(rxBits, msg);
%   disp(['Number of bit errors: ' num2str(numBitErrs)]);
%
%   See also nrPolarEncode, nrRateRecoverPolar, nrCRCDecode, nrDCIDecode,
%   nrBCHDecode, nrUCIDecode.

%   Copyright 2018-2023 The MathWorks, Inc.

%#codegen

    narginchk(4,7);

    if nargin==4
        % Downlink parameters
        % nrPolarDecode(in,K,E,L)
        nMax = 9;
        iIL = true;
        crcLen = 24;

        padCRC = false;           % default, for BCH and UCI
        rnti = 0;                 % default, unused

    elseif nargin == 5
        % Downlink parameters for which padCRC applies
        % nrPolarDecode(in,K,E,L,padCRC)
        nMax = 9;
        iIL = true;
        crcLen = 24;

        padCRC = varargin{1};     % true for DCI
        rnti = 0;                 % default, unused

    elseif nargin == 6
        % Downlink parameters for which padCRC, RNTI apply
        % nrPolarDecode(in,K,E,L,padCRC,RNTI)
        nMax = 9;
        iIL = true;
        crcLen = 24;
        padCRC = varargin{1};     % true for DCI
        rnti = varargin{2};       % user-specified, only for DCI

    elseif nargin == 7
        % Support both downlink and uplink
        % nrPolarDecode(in,K,E,L,nMax,iIL,crcLen)
        nMax = varargin{1};
        iIL = varargin{2};
        crcLen = varargin{3};

        padCRC = false;           % default, for BCH and UCI
        rnti = 0;                 % default, unused
    end

    % Input is a single code block and assumes CRC bits are included in K
    % output
    validateInputs(in,K,E,L,nMax,iIL,crcLen,padCRC,rnti);

    % F accounts for nPC bits, if present
    [F,qPC] = nr5g.internal.polar.construct(K,E,nMax);

    % CA-SCL decode
    if isempty(coder.target)
        if (L > 64)
            % Use MATLAB code for simulation
            outkpc = nr5g.internal.polar.decode(in,F,L,iIL,crcLen,padCRC,rnti,qPC);
        else
            % Use pre-compiled generated code for simulation
            L = double(L);
            crcLen = double(crcLen);
            rnti = double(rnti);
            outkpc = nr5g.internal.polar.cg_decode(in,F,L,iIL,crcLen,padCRC,rnti,qPC);
        end
    else
        % Generate code from MATLAB code
        outkpc = nr5g.internal.polar.decode(in,F,L,iIL,crcLen,padCRC,rnti,qPC);
    end

    % Remove nPC bits from output, if present
    if ~isempty(qPC)
        % Extract the information only bits
        qI = find(F==0)-1;
        k = 1;
        out = zeros(length(outkpc)-3,1);
        for idx = 1:length(qI)
            if ~any(qI(idx)==qPC)
                out(k) = outkpc(idx);
                k = k+1;
            end
        end
    else
        out = outkpc;
    end
end

function validateInputs(in,K,E,L,nMax,iIL,crcLen,padCRC,rnti)
% Check inputs

    fcnName = 'nrPolarDecode';

    % Validate rate-recovered input for a single code-block
    validateattributes(in,{'single','double'},{'real','column'}, ...
        fcnName,'REC');
    N = length(in);
    coder.internal.errorIf( floor(log2(N))~=log2(N), ...
        'nr5g:nrPolar:InvalidInputDecLength');

    % Validate the number of message bits which must be less than or equal to N
    validateattributes(K,{'numeric'}, ...
        {'real','scalar','integer','nonempty','finite','<=',N}, ...
        fcnName,'K');

    % Validate base-2 logarithm of rate-recovered input's maximum length
    % (9 or 10)
    validateattributes(nMax,{'numeric'},{'scalar','integer'}, ...
        fcnName,'NMAX');
    coder.internal.errorIf( ~any(nMax == [9 10]),'nr5g:nrPolar:InvalidnMax');

    % Validate output deinterleaving flag
    validateattributes(iIL,{'logical'},{'scalar'},fcnName,'IIL');

    % Validate the number of appended CRC bits (6, 11 or 24)
    validateattributes(crcLen,{'numeric'},{'scalar','integer','nonempty', ...
        'finite'},fcnName,'CRCLEN');
    coder.internal.errorIf( ~any(crcLen == [6 11 24]), ...
        'nr5g:nrPolar:InvalidCRCLen');

    % Validate CRC prepadding flag
    validateattributes(padCRC,{'logical'},{'scalar'},fcnName,'PADCRC');

    % Validate RNTI
    validateattributes(rnti,{'numeric'},{'scalar','integer','>=',0, ...
        '<=',65535},fcnName,'RNTI');

    % A restriction for downlink (for up to 12 bits padding)
    % length K must be greater than or equal to 36 and less than or equal
    % to 164
    coder.internal.errorIf( nMax==9 && iIL && crcLen==24 && ...
        (K < 36 || K > 164), 'nr5g:nrPolar:InvalidKDL');

    % A restriction for uplink (for CA-Polar)
    % length K must be greater than 30 and less than or equal to 1023
    coder.internal.errorIf( nMax==10 && ~iIL && crcLen==11 && ~padCRC && ...
        (K <= 30 || K > 1023), 'nr5g:nrPolar:InvalidKUL');

    % A restriction for uplink (for PC-Polar)
    % length K must be greater than 17 and less than or equal to 25
    coder.internal.errorIf( nMax==10 && ~iIL && crcLen==6 && ~padCRC && ...
        (K < 18 || K > 25), 'nr5g:nrPolar:InvalidKULPC');

    % Validate rate-matched output length which must be less than or equal
    % to 8192 and greater than or equal to K
    validateattributes(E,{'numeric'}, ...
        {'real','scalar','integer','nonempty','finite','>=',K,'<=',8192}, ...
        fcnName,'E');

    % Validate decoding list length
    nr5g.internal.validateParameters('ListLength',L,fcnName);

end
