function out = nrRateMatchLDPC(in,outlen,rv,modulation,nlayers,varargin)
%nrRateMatchLDPC LDPC rate matching
%   OUT = nrRateMatchLDPC(IN,OUTLEN,RV,MODULATION,NLAYERS) rate matches the
%   input data IN to create vector OUT of length OUTLEN. This function
%   includes the stages of bit selection and interleaving defined for LDPC
%   encoded data and code block concatenation (see TS 38.212 Sections 5.4.2
%   and 5.5).
%   The input data is a matrix, each column of which is assumed to be an
%   LDPC-encoded codeword. Filler <NULL> bits are represented by -1 in the
%   input. The number of columns in IN is the number of scheduled code
%   blocks of a transport block. Each column is rate matched separately and
%   the results are concatenated into the single output vector OUT. The
%   redundancy version (RV) of the output is controlled by the RV parameter
%   (0,1,2,3). MODULATION specifies the modulation type as one of
%   {'pi/2-BPSK','QPSK','16QAM','64QAM','256QAM','1024QAM'}, NLAYERS is 
%   the total number of transmission layers associated with the transport
%   block (1...4). This syntax assumes no limit is placed on the number of
%   soft bits.
%
%   OUT = nrRateMatchLDPC(...,NREF) allows configuration of the soft buffer
%   size for limited buffer rate matching. NREF is the parameter defined in
%   TS 38.212 Section 5.4.2.1.
%
%   % Example:
%   % Rate match two LDPC encoded code blocks of length 3960 to a vector of
%   % length 8000.
%
%   encoded = ones(3960,2);
%   outlen = 8000;
%   rv = 0;                 % Redundancy version
%   modulation = 'QPSK';    % Modulation type
%   nlayers = 1;            % Number of layers
%
%   ratematched = nrRateMatchLDPC(encoded,outlen,rv,modulation,nlayers);
%   size(ratematched)
%
%   See also nrRateRecoverLDPC, nrLDPCEncode, nrCodeBlockSegmentLDPC,
%   nrCRCEncode.

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

    narginchk(5,6);
    if nargin==5
        Nref = [];
    else
        Nref = varargin{1};
    end

    % Validate inputs
    modulation = validateInputs(in,outlen,rv,modulation,nlayers);

    % Output empty if input is empty or outlen is 0
    if isempty(in) || ~outlen
        out = zeros(0,1,"like",in);
        return;
    end

    % Validate input data length
    [N,C] = size(in);
    % Check against all possible lifting sizes
    ZcVec = [2:16 18:2:32 36:4:64 72:8:128 144:16:256 288:32:384];
    coder.internal.errorIf(~(any(N==(ZcVec.*66)) || any(N==(ZcVec.*50))), ...
        'nr5g:nrLDPC:InvalidInputNumRows',N);

    % Determine base graph number from N
    if any(N==(ZcVec.*66))
        bgn = 1;
        ncwnodes = 66;
    else % must be one of ZcVec.*50
        bgn = 2;
        ncwnodes = 50;
    end
    Zc = N/ncwnodes;

    % Get modulation order
    switch modulation
        case {'pi/2-BPSK', 'BPSK'}
            Qm = 1;
        case 'QPSK'
            Qm = 2;
        case '16QAM'
            Qm = 4;
        case '64QAM'
            Qm = 6;
        case '256QAM'
            Qm = 8;
        otherwise   % '1024QAM'
            Qm = 10;
    end

    % Get code block soft buffer size
    if ~isempty(Nref)
        fcnName = 'nrRateMatchLDPC';
        validateattributes(Nref, {'numeric'}, ...
            {'scalar','integer','positive'},fcnName,'NREF');

        Ncb = min(N,Nref);
    else    % No limit on buffer size
        Ncb = N;
    end

    % Get starting position in circular buffer
    if bgn == 1
        if rv == 0
            k0 = 0;
        elseif rv == 1
            k0 = floor(17*Ncb/N)*Zc;
        elseif rv == 2
            k0 = floor(33*Ncb/N)*Zc;
        else % rv is equal to 3
            k0 = floor(56*Ncb/N)*Zc;
        end
    else
        if rv == 0
            k0 = 0;
        elseif rv == 1
            k0 = floor(13*Ncb/N)*Zc;
        elseif rv == 2
            k0 = floor(25*Ncb/N)*Zc;
        else % rv is equal to 3
            k0 = floor(43*Ncb/N)*Zc;
        end
    end

    % Get rate matching output for all scheduled code blocks and perform
    % code block concatenation according to Section 5.4.2 and 5.5
    r = 0:C-1;
    E1 = nlayers*Qm*floor(outlen/(nlayers*Qm*C)); %(r <= C-mod(outlen/(nlayers*Qm),C)-1)
    E2 = nlayers*Qm*ceil(outlen/(nlayers*Qm*C)); %(r > C-mod(outlen/(nlayers*Qm),C)-1)
    if E1 == E2         
        out = cbsRateMatch(in,E1,k0,Ncb,Qm);
    else
        out = cbsRateMatch(in(:,r <= C-mod(outlen/(nlayers*Qm),C)-1),E1,k0,Ncb,Qm);
        out = [out;cbsRateMatch(in(:,r > C-mod(outlen/(nlayers*Qm),C)-1),E2,k0,Ncb,Qm)];
    end

end

function e = cbsRateMatch(d,E,k0,Ncb,Qm)
% Rate match a single code block segment as per TS 38.212 Section 5.4.2

    if isempty(d)
        e = zeros(0,0,"like",d);
        return
    end

    % Bit selection, Section 5.4.2.1 
    % Get number of filler bits inside the circular buffer
    NFillerBits = sum(d(1:Ncb,1) == -1,1); 

    % Duplicate data if more than one iteration around the circular
    % buffer is required to obtain a total of E bits
    repetitions = ceil(E/(Ncb-NFillerBits));
    d = repmat(d(1:Ncb,:),repetitions,1);
    
    % Shift data to start from selected redundancy version
    if k0 ~= 0
        % circshift is not a no-op if k0 = 0 but d is a gpuArray
        d = circshift(d,-k0);
    end

    % Avoid filler bits and provide an empty vector if E is 0
    d(d(:,1)==-1,:) = [];
    e = d(1:E,:);
    
    % Bit interleaving, Section 5.4.2.2
    e = reshape(e,E/Qm,Qm,[]);
    e = pagetranspose(e);
    e = e(:); 

end

function modulation = validateInputs(in,outlen,rv,modulation,nlayers)
% Check inputs

    fcnName = 'nrRateMatchLDPC';

    % Validate LDPC-encoded input data
    validateattributes(in,{'int8','double'},{'2d'},fcnName,'IN');

    % Validate the output length
    validateattributes(outlen, {'numeric'}, ...
        {'scalar','integer','nonnegative','finite',},fcnName,'OUTLEN');  

    % Validate redundancy version (0...3)
    validateattributes(rv, {'numeric'}, ...
        {'scalar','integer','nonnegative','<=',3},fcnName,'RV');  

    % Validate modulation scheme
    modulation = validatestring(modulation,{'pi/2-BPSK','BPSK','QPSK', ...
        '16QAM','64QAM','256QAM','1024QAM'},fcnName,'MODULATION');

    % Validate the number of transmission layers (1...4)
    validateattributes(nlayers, {'numeric'}, ...
        {'real','scalar','integer','positive','<=',4},fcnName,'NLAYERS');

end
