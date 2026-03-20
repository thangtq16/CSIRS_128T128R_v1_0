function [out,actNumIter,finalParityChecks] = nrLDPCDecode(in,bgn, ...
        maxNumIter,varargin)
%nrLDPCDecode LDPC decoding
%   [OUT,ACTNUMITER,FINALPARITYCHECKS] = nrLDPCDecode(IN,BGN,MAXNUMITER)
%   returns the result of LDPC decoding in the matrix OUT based on soft
%   input matrix IN, base graph number BGN (1 or 2), and a maximum number
%   of decoding iterations MAXNUMITER. The decoder uses the sum-product
%   message passing algorithm and expects the data bits to be LDPC encoded 
%   as defined in TS 38.212 Section 5.3.2.
%
%   Each column in IN and OUT represents a code block segment before and
%   after LDPC decoding, respectively. The number of columns in IN or OUT
%   is equal to the number of scheduled code block segments. The number of
%   rows in IN is the length of codeword with some information bits
%   punctured, the number of rows in OUT is the number of information bits
%   in an LDPC codeword. The decoding is terminated when the parity checks
%   are satisfied, using MAXNUMITER as the maximum number of decoding
%   iterations. The decoding procedure includes padding of the punctured
%   bits to recover the full codeword and decoding to get the complete LDPC
%   information bits (including the punctured bits).
%
%   ACTNUMITER is a row vector of positive integer(s) whose length is equal
%   to the number of columns of IN. The i-th element corresponds to the
%   actual number of decoding iterations executed for the i-th column of
%   IN.
%
%   FINALPARITYCHECKS is a matrix that holds the final parity checks. The
%   i-th column corresponds to the final parity checks for the i-th
%   codeword. The number of rows is equal to the number of parity-check
%   bits in an LDPC codeword.
%
%   [OUT,ACTNUMITER,FINALPARITYCHECKS] = nrLDPCDecode(...,Name,Value)
%   specifies additional name-value pair arguments described below:
%
%   'OutputFormat' - One of 'info', 'whole', specifies the output format.
%                    OUT contains decoded information bits (default) or
%                    whole LDPC codeword bits. For 'info', the number of
%                    rows in OUT is the length of the information bits and
%                    for 'whole', the number of rows in OUT is the codeword
%                    length.
%   'DecisionType' - One of 'hard', 'soft', specifies the decision type
%                    used for decoding. For 'hard' (default), output is
%                    decoded bits of 'int8' type. For 'soft', output is
%                    log-likelihood ratios with the same type as input.
%   'Algorithm'    - One of 'Belief propagation', 'Layered belief
%                    propagation', 'Normalized min-sum', 'Offset min-sum',
%                    specifies the decoding algorithm used. The default is
%                    'Belief propagation'.
%   'ScalingFactor'- Specifies the scaling factor for 'Normalized min-sum'
%                    decoding algorithm as a real scalar greater than 0 and
%                    less than or equal to 1. The default is 0.75. The
%                    value is only applicable when Algorithm is set to
%                    'Normalized min-sum'.
%   'Offset'       - Specifies the offset for 'Offset min-sum' decoding 
%                    algorithm as a finite real scalar greater than or 
%                    equal to 0. The default is 0.5. The value is only 
%                    applicable when Algorithm is set to 'Offset min-sum'.
%   'Termination'  - One of 'early', 'max', specifies the decoding
%                    termination criteria. For 'early' (default), decoding
%                    is terminated when all parity checks are satisfied, up
%                    to a maximum number of iterations given by MAXNUMITER.
%                    For 'max', decoding continues till MAXNUMITER
%                    iterations are completed.
%
%   % Example:
%   % Two code blocks of length 2560 are encoded to obtain two code blocks
%   % of length 12800, the encoded bits are then converted to soft bits to
%   % be decoded by the LDPC decoder.
%
%   bgn = 2;                    % Base graph number
%   K = 2560;                   % Code block segment length
%   F = 36;                     % Number of filler bits per code block
%   C = 2;                      % Number of code blocks
%   txcbs = ones(K-F,C);
%   fillers = -1*ones(F,C);
%   txcbs = [txcbs;fillers];                % Add fillers
%   txcodedcbs = nrLDPCEncode(txcbs,bgn);   % Encode
%   
%   rxcodedcbs = double(1-2*txcodedcbs);    % Convert to soft bits
%   FillerIndices = find(txcodedcbs(:,1) == -1);
%   rxcodedcbs(FillerIndices,:) = 0;    % Fillers have no LLR information
% 
%   % Decode with a maximum of 25 iterations
%   [rxcbs, actualniters] = nrLDPCDecode(rxcodedcbs,bgn,25);
% 
%   txcbs(end-F+1:end,:) = 0;           % Replace filler bits with 0
% 
%   isequal(rxcbs,txcbs)                
%   actualniters
%
%   See also nrLDPCEncode, nrRateRecoverLDPC, nrCodeBlockDesegmentLDPC,
%   nrCRCDecode, nrDLSCHInfo.

%   Copyright 2018-2023 The MathWorks, Inc.

%#codegen

    narginchk(3,15);

    [params,algChoice,alphaBeta] = validateInputs(in,bgn,maxNumIter,varargin{:});
    isHardDec = int8(strcmp(params.DecisionType,'hard'));
    
    % Obtain input/output size
    [N,C] = size(in);

    % Output empty if the input data is empty
    if isempty(in)
        if isHardDec
            out = cast(zeros(0,C,'like',in),'int8');
        else
            out = zeros(0,C,'like',in);
        end
        actNumIter = 0;
        finalParityChecks = zeros(0,C,'like',in);
        return;
    end

    % LDPC decoding parameters
    if bgn==1
        ncwnodes = 66;
    else
        ncwnodes = 50;
    end
    Zc = N/ncwnodes;
    % Validate input data length
    coder.internal.errorIf(fix(Zc)~=Zc, ...
        'nr5g:nrLDPC:InvalidInputLength',N,ncwnodes,bgn);
    % Check Zc among all possible lifting sizes
    ZcVec = [2:16 18:2:32 36:4:64 72:8:128 144:16:256 288:32:384];
    coder.internal.errorIf( ~any(Zc==ZcVec),'nr5g:nrLDPC:InvalidZc',Zc,bgn);

    algStr = {'bp','layered-bp','norm-min-sum','offset-min-sum'};
    
    % Get LDPC parameters for the bgn, Zc value pair
    cfg = nr5g.internal.ldpc.getParams(bgn,Zc);
    cfg.Algorithm = algStr{algChoice+1};

    % Add punctured 2*Zc information bits to recover the full codeword
    in = [zeros(2*Zc,C,'like',in); in];

    % Decode
    if nargout < 3
        % Specify two outputs to tell ldpcDecode not to compute finalParityChecks
        [out,actNumIter] = ldpcDecode(in,cfg,maxNumIter,'OutputFormat',params.OutputFormat,'DecisionType',params.DecisionType,'Termination',params.Termination,'MinSumScalingFactor',alphaBeta,'MinSumOffset',alphaBeta);
    else
        [out,actNumIter,finalParityChecks] = ldpcDecode(in,cfg,maxNumIter,'OutputFormat',params.OutputFormat,'DecisionType',params.DecisionType,'Termination',params.Termination,'MinSumScalingFactor',alphaBeta,'MinSumOffset',alphaBeta);
    end
end

function [params,alg,alphaBeta] = validateInputs(in,bgn,maxNumIter,varargin)
    
    fcnName = 'nrLDPCDecode';

    % Validate the input soft data
    validateattributes(in,{'double','single'},{'2d'},fcnName,'IN');

    % Validate the input base graph number (1 or 2)
    validateattributes(bgn,{'numeric'},{'scalar','integer'},fcnName,'BGN');  
    coder.internal.errorIf(~(bgn==1||bgn==2),'nr5g:nrLDPC:InvalidBGN',bgn);

    % Validate the maximum number of iterations
    validateattributes(maxNumIter, {'numeric'}, ...
        {'real','scalar','integer','>',0},fcnName,'MAXNUMITER');  

    % Parse and validate the optional params
    params = nr5g.internal.parseOptions(fcnName, ...
        {'OutputFormat','DecisionType','Algorithm','ScalingFactor', ...
         'Offset','Termination'},varargin{:});        
    
    switch params.Algorithm
        case 'Belief propagation'
            alg = 0;
            alphaBeta = 1; % Unused
        case 'Layered belief propagation'
            alg = 1;
            alphaBeta = 1; % Unused
        case 'Normalized min-sum'
            alg = 2;
            alphaBeta = params.ScalingFactor;
        otherwise % "Offset min-sum"
            alg = 3;
            alphaBeta = params.Offset;
    end
end
