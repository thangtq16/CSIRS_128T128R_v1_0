function out = nrRateRecoverLDPC(in,trblklen,R,rv,modulation,nlayers,varargin)
%nrRateRecoverLDPC LDPC rate recovery
%   OUT = nrRateRecoverLDPC(IN,TRBLKLEN,R,RV,MODULATION,NLAYERS) rate
%   matches the input vector IN to create a matrix OUT representing the
%   LDPC encoded code blocks. This function includes the inverse of the
%   code block concatenation, bit interleaving and bit selection stages for
%   LDPC encoded data (see TS 38.212 Sections 5.4.2 and 5.5). The number of
%   rows in OUT is deduced from TRBLKLEN which represents the length of the
%   original transport block, the number of columns in OUT is the number of
%   code block segments. Filler bits are set to Inf to correspond to 0s
%   used during their encoding. R is the target code rate, numeric scalar
%   valued between 0 and 1. TRBLKLEN and R are required in order to recover
%   the number of code blocks, their LDPC encoded lengths, the locations of
%   any filler bits and the lifting size. The redundancy version (RV) used
%   to recover the data is controlled by the RV parameter (0,1,2,3).
%   MODULATION specifies the modulation type as one of
%   {'pi/2-BPSK','QPSK','16QAM','64QAM','256QAM','1024QAM'}, NLAYERS is the
%   total number of transmission layers associated with the transport block
%   (1...4). This syntax assumes no limit is placed on the number of soft
%   bits and outputs the original number of code blocks.
% 
%   OUT = nrRateRecoverLDPC(...,NUMCB) specifies the number of code blocks
%   to be recovered. NUMCB is a scalar integer less than or equal to the
%   original number of code blocks as determined from TRBLKLEN and R.
%
%   OUT = nrRateRecoverLDPC(...,NUMCB,NREF) also allows specification of
%   the soft buffer size for limited buffer rate matching. NREF is the
%   parameter defined in TS 38.212 Section 5.4.2.1. Both NUMCB and NREF can
%   be specified as empties to allow the default full code block, unlimited
%   buffer processing.
%   
%   % Example:
%   % Rate recover an input vector of 4500 bits to one encoded code block
%   % of length 12672 bits.
% 
%   trblklen = 4000;        % Transport block length
%   rate = 0.5;             % Target code rate
%   rv = 0;                 % Redundancy version
%   modulation = 'QPSK';    % Modulation type
%   nlayers = 1;            % Number of layers
%
%   sbits = ones(4500,1);
%   raterec = nrRateRecoverLDPC(sbits,trblklen,rate,rv,modulation,nlayers);
%   size(raterec)
%
%   See also nrRateMatchLDPC, nrLDPCDecode, nrCodeBlockDesegmentLDPC,
%   nrCRCDecode.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

    narginchk(6,8);
    % Get scheduled code blocks and limited buffers for HARQ combining
    if nargin==6
        numCB = [];
        Nref = [];
    elseif nargin==7
        numCB = varargin{1};
        Nref = [];
    else
       numCB = varargin{1};
       Nref = varargin{2};
    end
    modulation = validateInputs(in,trblklen,R,rv,modulation,nlayers);

    % Output empty if the input is empty or trblklen is 0
    if isempty(in) || ~trblklen
        out = zeros(0,1,"like",in);
        return;
    end

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
        otherwise % '1024QAM'
            Qm = 10;
    end

    % Get base graph and code block segmentation parameters
    cbsinfo = nrDLSCHInfo(trblklen,R);
    bgn = cbsinfo.BGN;
    Zc = cbsinfo.Zc;
    N = cbsinfo.N;

    % Get number of scheduled code block segments
    if ~isempty(numCB)
        fcnName = 'nrRateRecoverLDPC';
        validateattributes(numCB, {'numeric'}, ...
            {'scalar','integer','positive','<=',cbsinfo.C},fcnName,'NUMCB');  

        C = numCB;      % scheduled code blocks
    else
        C = cbsinfo.C;  % all code blocks
    end

    % Get code block soft buffer size
    if ~isempty(Nref)
        fcnName = 'nrRateRecoverLDPC';
        validateattributes(Nref, {'numeric'}, ...
            {'scalar','integer','positive'},fcnName,'Nref');

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
        else % rv == 3
            k0 = floor(56*Ncb/N)*Zc;
        end
    else
        if rv == 0
            k0 = 0;
        elseif rv == 1
            k0 = floor(13*Ncb/N)*Zc;
        elseif rv == 2
            k0 = floor(25*Ncb/N)*Zc;
        else % rv == 3
            k0 = floor(43*Ncb/N)*Zc;
        end
    end

    G = length(in);
    r = 0:C-1;
    E1 = nlayers*Qm*floor(G/(nlayers*Qm*C)); % r <= C-mod(G/(nlayers*Qm),C)-1
    E2 = nlayers*Qm*ceil(G/(nlayers*Qm*C));  % r > C-mod(G/(nlayers*Qm),C)-1
    if G < E1
        % Pad "unknown" bits to support insufficient input
        zeroPad = zeros(E1-G,1,"like",in);
        deconcatenated = [in; zeroPad];
        out = cbsRateRecover(deconcatenated,cbsinfo,k0,Ncb,Qm,E1);
    elseif G < E2
        % Pad "unknown" bits to support insufficient input
        zeroPad = zeros(E2-G,1,"like",in);
        deconcatenated = [in; zeroPad];
        out = cbsRateRecover(deconcatenated,cbsinfo,k0,Ncb,Qm,E2);
    else
        if E1 == E2
            out = cbsRateRecover(in,cbsinfo,k0,Ncb,Qm,E1);
        else
            blocks = E1*sum(r <= C-mod(G/(nlayers*Qm),C)-1);
            out = cbsRateRecover(in(1:blocks),cbsinfo,k0,Ncb,Qm,E1);
            out = cat(2,out,cbsRateRecover(in(blocks+1:end),cbsinfo,k0,Ncb,Qm,E2));
        end
    end
end

function out = cbsRateRecover(in,cbsinfo,k0,Ncb,Qm,E)
% Rate recovery for a single code block segment

    if isempty(in)
        out = zeros(0,0,"like",in);
        return;
    end

    in = reshape(in,E,[]);
    % Perform bit de-interleaving according to TS 38.212 5.4.2.2
    in = reshape(in,Qm,E/Qm,[]);
    in = pagetranspose(in);
    in = reshape(in,E,[]);

    numBlocks = size(in,2);

    % Calculate soft buffer size according to TS 38.212 5.4.2
    [NBuffer,K,Kd] = nr5g.internal.ldpc.softBufferSize(cbsinfo,Ncb);

    % Perform reverse of bit selection according to TS 38.212 5.4.2.1
    
    % Duplicate data if more than one iteration around the circular
    % buffer is required to obtain a total of E bits
    if isa(in, "gpuArray")
        idx = reshape(gpuArray.colon(1,Ncb*numBlocks),Ncb,numBlocks);
    else
        idx = reshape(1:Ncb*numBlocks,Ncb,numBlocks);
    end
    idx = repmat(idx,ceil(E/NBuffer),1);

    % Shift data to start from selected redundancy version
    idx = circshift(idx,-k0);

    % Avoid filler bits indices. The first column is used as the filler bits
    % are in the same position in every column.
    idx(idx(:,1)>Kd & idx(:,1)<=K,:) = [];
    indices = idx(1:E,1); 

    % Initialize output
    out = zeros(cbsinfo.N,numBlocks,"like",in);

    % Fill in circular buffer
    if E > NBuffer
        % Stack block repetitions in columns and soft combine by adding
        % columns together
        for x = 1:numBlocks
            inRep = zeros(NBuffer,ceil(E/NBuffer),"like",in);
            inRep(1:E) = in(:,x);
            out(indices(1:NBuffer),x) = sum(inRep,2);
        end
    else
        out(indices,:) = in;
    end
    
    % Filler bits are treated as 0 bits when encoding, 0 bits correspond to
    % Inf in received soft bits, this step improves error-correction
    % performance in the LDPC decoder
    out(Kd+1:K,:) = Inf;
    
end

function modulation = validateInputs(in,trblklen,R,rv,modulation,nlayers)
% Check inputs

    fcnName = 'nrRateRecoverLDPC';

    % Validate input soft data
    validateattributes(in,{'double','single'},{'real','column'},fcnName,'IN');

    % Validate transport block length
    validateattributes(trblklen,{'numeric'}, ...
        {'scalar','integer','nonnegative','finite'},fcnName,'TRBLKLEN');

    % Validate target code rate
    validateattributes(R,{'numeric'}, ...
        {'real','scalar','>',0,'<',1},fcnName,'RATE');

    % Validate redundancy version (0...3)
    validateattributes(rv,{'numeric'}, ...
        {'scalar','integer','nonnegative','<=',3},fcnName,'RV');

    % Validate modulation scheme
    modulation = validatestring(modulation,{'pi/2-BPSK','BPSK','QPSK', ...
        '16QAM','64QAM','256QAM','1024QAM'},fcnName,'MODULATION');

    % Validate the number of transmission layers (1...4)
    validateattributes(nlayers,{'numeric'}, ...
        {'scalar','integer','positive','<=',4},fcnName,'NLAYERS');
end
