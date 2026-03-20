function out = nrRateMatchPolar(in,K,E,varargin)
%nrRateMatchPolar Polar rate matching
%   OUT = nrRateMatchPolar(IN,K,E) returns the rate-matched output, OUT,
%   for a polar encoded input, IN, for an information block length K. The
%   output is of length E. Coded-bit interleaving is disabled (IBIL is set
%   to false) for downlink configurations.
%
%   OUT = nrRateMatchPolar(...,IBIL) allows the enabling of coded-bit
%   interleaving by specifying a boolean scalar (IBIL as true). This
%   setting is used for uplink configurations.
%
%   % Example:
%   % Rate match a polar encoded code block of length 512 to a vector of
%   % length 864.
%
%   N = 2^9;            % Polar encoded block length
%   K = 56;             % Number of information bits
%   E = 864;            % Number of rate-matched output bits
%   iBIL = false;       % Interleaving of rate-matched coded bits
%
%   in = randi([0 1],N,1);
%   out = nrRateMatchPolar(in,K,E,iBIL);
%
%   See also nrRateRecoverPolar, nrPolarEncode, nrCRCEncode, nrDCIEncode,
%   nrBCH.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

%   Reference:
%   [1] 3GPP TS 38.212, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Multiplexing and channel
%   coding (Release 15). Section 5.4.1.

    narginchk(3,4);
    if nargin==3
        iBIL = false;
    else
        iBIL = varargin{1};
    end

    % Validate inputs
    validateInputs(in,K,E,iBIL);

    % Sub-block interleaving, Section 5.4.1.1
    y = subBlockInterleave(in);

    % Bit selection, Section 5.4.1.2
    N = length(in);
    outE = zeros(E,1,class(in));
    if E >= N
        % Bit repetition
        for k = 0:E-1
            outE(k+1) = y(mod(k,N)+1);
        end
    else
        if K/E <= 7/16
            % puncturing (take from the end)
            outE = y(end-E+1:end);
        else
            % shortening (take from the start)
            outE = y(1:E);
        end
    end

    % Interleaving, Section 5.4.1.3
    if iBIL
        % Specified for uplink only
        out = iBILInterl(outE);
    else
        % No interleaving
        out = outE;
    end

end

function validateInputs(in,K,E,iBIL)
% Check inputs

    fcnName = 'nrRateMatchPolar';

    % Validate polar-encoded message, length must be a power of two
    validateattributes(in,{'int8','double'},{'2d','binary','column'}, ...
        fcnName,'IN');
    N = length(in);
    coder.internal.errorIf( floor(log2(N))~=log2(N), ...
        'nr5g:nrPolar:InvalidInputRMLength');

    % Validate coded-bit interleaving flag
    validateattributes(iBIL,{'logical'},{'scalar'},fcnName,'IBIL');

    if iBIL % for Uplink
        % Validate the information block length which must be greater than
        % or equal to 18 (12+6) and less than or equal to N. Also, 25<K<31
        % is invalid.
        validateattributes(K, {'numeric'}, ...
            {'real','scalar','integer','nonempty','finite','<=',N,'>=',18}, ...
            fcnName,'K');
        coder.internal.errorIf(K>25 && K<31,'nr5g:nrPolar:UnsupportedKforUL');

    else % for Downlink
        % Validate the information block length which must be greater than
        % or equal to 36 (12+24) and less than or equal to N
        validateattributes(K, {'numeric'}, ...
            {'real','scalar','integer','nonempty','finite','<=',N,'>=',36}, ...
            fcnName,'K');
    end
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

function out = subBlockInterleave(in)
% Sub-block interleaver
%   OUT = subBlockInterleave(IN) returns the sub-block interleaved output.
%
%   Reference: TS 38.212, Section 5.4.1.1.

    N = length(in);
    jn = nr5g.internal.polar.subblockInterleaveMap(N);
    out = in(jn+1);

end

function out = iBILInterl(in)
% Triangular interleaver
%
%   OUT = iBILInterl(IN) performs triangular interleaving on the input, IN,
%   writing in the input E elements row-wise and returns the output, OUT,
%   by reading them out column-wise.
%
%   Reference: TS 38.212, Section 5.4.1.3.

    % Get T off E
    E = length(in);
    T = getT(E);

    % Write input to buffer row-wise
    v = -1*ones(T,T,class(in));   % <NULL> bits
    k = 0;
    for i = 0:T-1
        for j = 0:T-1-i
            if k < E
                v(i+1,j+1) = in(k+1);
            end
            k = k+1;
        end
    end

    % Read output from buffer column-wise
    out = zeros(size(in),class(in));
    k = 0;
    for j = 0:T-1
        for i = 0:T-1-j
            if v(i+1,j+1) ~= -1
                out(k+1) = v(i+1,j+1);
                k = k+1;
            end
        end
    end

end

function t = getT(E)

    % Use quadratic solution with ceil for >= in expression.
    t = ceil((-1+sqrt(1+8*E))/2);

end
