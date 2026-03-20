function out = nrPolarEncode(in,E,varargin)
%nrPolarEncode Polar encoding
%   ENC = nrPolarEncode(IN,E) returns the polar encoded output for an input
%   message, IN, and rate-matched output length, E, for a downlink
%   configuration. IN is a column vector of length K, including the CRC
%   bits, as applicable. ENC is the output column vector of length N, where
%   N is the number of encoded bits defined in TS 38.212 Section 5.3.1. For
%   downlink, nMax = 9 and iIL = true. The function includes the
%   determination of the frozen bits, input interleaving and encoding as
%   per TS 38.212 Section 5.3.1.
%
%   ENC = nrPolarEncode(IN,E,NMAX,IIL) encodes the input using the
%   specified NMAX (an integer value of 9 or 10) and IIL (a boolean scalar)
%   parameters. The allowed value sets of {9,true} and {10,false} for
%   {NMAX,IIL} apply for downlink and uplink configurations respectively.
%
%   % Example 1:
%   % Polar-encode a block of data
%
%   K = 132;            % Message length
%   E = 256;            % Rate matched output length
%   msg = randi([0 1],K,1);                % Generate random message
%   enc = nrPolarEncode(msg,E);            % Polar encode
%   size(enc,1)
%
%   % Example 2:
%   % Transmit polar-encoded block of data and decode using
%   % successive-cancellation list decoder.
%
%   K = 132;            % Message length
%   E = 256;            % Rate matched output length
%   nVar = 1.0;         % Noise variance
%   L = 8;              % Decoding List length
%
%   % Object construction
%   chan   = comm.AWGNChannel('NoiseMethod','Variance','Variance',nVar);
%
%   % Simulate a frame
%   msg    = randi([0 1],K,1);                      % Generate random message
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
%   See also nrPolarDecode, nrCRCEncode, nrRateMatchPolar, nrDCIEncode,
%   nrBCH, nrUCIEncode.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

    narginchk(2,4);

    % Parse inputs
    if nargin==2
        % Downlink params
        % nrPolarEncode(in,E)
        nMax = 9;       % maximum n value for N
        iIL = true;     % input interleaving
    elseif nargin==3
        coder.internal.errorIf(1,'nr5g:nrPolar:InvalidNumInputs');
    else
        % nrPolarEncode(in,E,nMax,iIL)
        nMax = varargin{1};
        iIL = varargin{2};
    end

    % Validate inputs
    validateInputs(in,E,nMax,iIL);

    % Input is a single code block and assumes CRC bits are included
    K = length(in);

    % Interleave input, if specified
    if iIL
        pi = nr5g.internal.polar.interleaveMap(K);
        inIntr = in(pi+1);
    else
        inIntr = in;
    end

    % Get frozen bit indices and parity-check bit locations
    [F,qPC] = nr5g.internal.polar.construct(K,E,nMax);
    N = length(F);
    nPC = length(qPC);

    % Generate u
    u = zeros(N,1);     % doubles only
    if nPC > 0
        % Parity-Check Polar (PC-Polar)
        y0 = 0; y1 = 0; y2 = 0; y3 = 0; y4 = 0;
        k = 1;
        for idx = 1:N
            yt = y0; y0 = y1; y1 = y2; y2 = y3; y3 = y4; y4 = yt;
            if F(idx)   % frozen bits
                u(idx) = 0;
            else        % info bits
                if any(idx==(qPC+1))
                    u(idx) = y0;
                else
                    u(idx) = inIntr(k); % Set information bits (interleaved)
                    k = k+1;
                    y0 = double(xor(y0,u(idx)));
                end
            end
        end
    else
        % CRC-Aided Polar (CA-Polar)
        u(F==0) = inIntr;   % Set information bits (interleaved)
    end

    % Get G, nth Kronecker power of kernel
    n = log2(N);
    ak0 = [1 0; 1 1];   % Arikan's kernel
    allG = cell(n,1);   % Initialize cells
    for i = 1:n
        allG{i} = zeros(2^i,2^i);
    end
    allG{1} = ak0;      % Assign cells
    for i = 1:n-1
        allG{i+1} = kron(allG{i},ak0);
    end
    G = allG{n};

    % Encode using matrix multiplication
    outd = mod(u'*G,2)';
    out = cast(outd,class(in));

end

function validateInputs(in,E,nMax,iIL)
% Check inputs

    fcnName = 'nrPolarEncode';

    % Validate single code-block input message
    validateattributes(in,{'int8','double'},{'binary','column'}, ...
        fcnName,'IN');
    K = length(in);

    % Validate base-2 logarithm of encoded message's maximum length
    % (9 or 10)
    validateattributes(nMax,{'numeric'},{'scalar','integer'}, ...
        fcnName,'NMAX');
    coder.internal.errorIf( ~any(nMax == [9 10]),'nr5g:nrPolar:InvalidnMax');

    % Validate input interleaving flag
    validateattributes(iIL, {'logical'}, {'scalar'}, fcnName, 'IIL');

    % A restriction for downlink (for up to 12 bits padding)
    % length K must be greater than or equal to 36 and less than or equal
    % to 164
    coder.internal.errorIf( nMax==9 && iIL && (K < 36 || K > 164), ...
        'nr5g:nrPolar:InvalidInputEncDLLength',K);

    % A restriction for uplink (for CA-Polar and PC-Polar)
    % length K must be greater than or equal to 18 and less than or equal
    % to 1023, with interim range from 26<=K<=30 not allowed
    coder.internal.errorIf( nMax==10 && ~iIL && (K<18 || (K>25 && K<31) ...
        || K>1023), 'nr5g:nrPolar:InvalidInputEncULLength',K);
    if (K>=18 && K<=25) % for PC-Polar
        nPC = 3;
    else
        nPC = 0;
    end

    % Validate rate-matched output length which must be less than or equal
    % to 8192 and greater than or equal to K+nPC
    validateattributes(E, {'numeric'}, ...
        {'real','scalar','integer','finite','>=',K+nPC,'<=',8192}, ...
        fcnName,'E');

end
