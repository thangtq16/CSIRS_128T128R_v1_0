function out = nrRateRecoverPolar(in,K,N,varargin)
%nrRateRecoverPolar Polar rate matching recovery
%   OUT = nrRateRecoverPolar(IN,K,N) returns the rate-recovered output,
%   OUT, for an input, IN, of length E. The output, OUT, is of length N and
%   K represents the information block length. Coded-bit interleaving is
%   disabled (iBIL is set to false) for downlink configurations.
%
%   OUT = nrRateRecoverPolar(...,IBIL) allows the enabling of coded-bit
%   interleaving by specifying a boolean scalar (IBIL as true). This
%   setting is used for uplink configurations.
%
%   % Example:
%   % Rate match a polar encoded code block of length 512 to a vector of
%   % length 864 and then recover it.
%
%   N = 2^9;            % Polar encoded block length
%   K = 56;             % Number of information bits
%   E = 864;            % Number of rate matched output bits
%   iBIL = false;       % Deinterleaving of input bits
%
%   in = randi([0 1],N,1);
%   chIn = nrRateMatchPolar(in,K,E,iBIL);
%   out = nrRateRecoverPolar(1-2*chIn,K,N,iBIL);
%   isequal(out<1,in)
%
%   See also nrRateMatchPolar, nrPolarDecode, nrCRCDecode, nrDCIDecode,
%   nrBCHDecode.

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
    validateInputs(in,K,N,iBIL);

    % Channel deinterleaving, Section 5.4.1.3
    if iBIL
        % Specified for uplink only
        inE = iBILDeinterl(in);
    else
        % No deinterleaving
        inE = in;
    end

    % Bit selection, Section 5.4.1.2
    E = length(in);
    if E >= N
        % Stack block repetitions in columns and soft combine by adding
        % columns together
        inRep = zeros(N,ceil(E/N),class(in));
        inRep(1:E) = inE;
        outN = sum(inRep,2);
    else
        if K/E <= 7/16
            % puncturing (put at the end)
            outN = zeros(N,1,class(in));          % 0s for punctures
            outN(end-E+1:end) = inE;
        else
            % shortening (put at the start)
            outN = 1e20*ones(N,1,class(in));      % use a large value for 0s
            outN(1:E) = inE;
        end
    end

    % Sub-block deinterleaving, Section 5.4.1.1
    out = subBlockDeinterleave(outN);

end

function validateInputs(in,K,N,iBIL)
% Check inputs

    fcnName = 'nrRateRecoverPolar';

    % Validate log-likelihood ratio value input, length must be less than
    % or equal to 8192
    validateattributes(in,{'single','double'},{'2d','column','real'}, ...
        fcnName,'IN');
    E = length(in);
    coder.internal.errorIf( E>8192,'nr5g:nrPolar:InvalidInputRRLength');

    % Validate coded-bit deinterleaving flag
    validateattributes(iBIL,{'logical'},{'scalar'},fcnName,'iBIL');

    if iBIL % for Uplink
        % Validate the information block length which must be greater than
        % or equal to 18 (12+6) and less than or equal to E. Also, 25<K<31
        % is invalid.
        validateattributes(K, {'numeric'}, ...
            {'real','scalar','integer','nonempty','finite','<=',E,'>=',18}, ...
            fcnName,'K');
        coder.internal.errorIf(K>25 && K<31,'nr5g:nrPolar:UnsupportedKforUL');

    else % for Downlink
        % Validate the information block length which must be greater than
        % or equal to 36 (12+24) and less than or equal to E
        validateattributes(K, {'numeric'}, ...
            {'real','scalar','integer','nonempty','finite','<=',E,'>=',36}, ...
            fcnName,'K');
    end

    % Validate the polar-encoded output length which must be a power of two
    % and greater than or equal to K
    validateattributes(N, {'numeric'}, ...
        {'real','scalar','integer','nonempty','finite','>=',K}, ...
        fcnName,'N');
    coder.internal.errorIf( floor(log2(N))~=log2(N), ...
        'nr5g:nrPolar:InvalidN');

end

function out = subBlockDeinterleave(in)
% Sub-block deinterleaver
%   OUT = subBlockDeinterleave(IN) returns the sub-block deinterleaved
%   input.
%
%   Reference: TS 38.212, Section 5.4.1.1.

    N = length(in);
    jn = nr5g.internal.polar.subblockInterleaveMap(N);
    out = zeros(N,1,class(in));
    out(jn+1) = in;

end

function out = iBILDeinterl(in)
% Triangular deinterleaver
%
%   OUT = iBILDeinterl(IN) performs triangular deinterleaving on the input,
%   IN, and returns the output, OUT.
%
%   Reference: TS 38.212, Section 5.4.1.3.

    % Get T off E
    E = length(in);
    T = getT(E);

    % Create the table with nulls (filled in row-wise)
    vTab = zeros(T,T,class(in));
    k = 0;
    for i = 0:T-1
        for j = 0:T-1-i
            if k < E
                vTab(i+1,j+1) = k+1;
            end
            k = k+1;
        end
    end

    % Write input to buffer column-wise, respecting vTab
    v = Inf*ones(T,T,class(in));
    k = 0;
    for j = 0:T-1
        for i = 0:T-1-j
            if k < E && vTab(i+1,j+1) ~= 0
                v(i+1,j+1) = in(k+1);
                k = k+1;
            end
        end
    end

    % Read output from buffer row-wise
    out = zeros(size(in),class(in));
    k = 0;
    for i = 0:T-1
        for j = 0:T-1-i
            if ~isinf(v(i+1,j+1))
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
