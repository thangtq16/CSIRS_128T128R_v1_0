function out = nrLDPCEncode(in,bgn)
%nrLDPCEncode LDPC encoding
%   OUT = nrLDPCEncode(IN,BGN) returns the result of LDPC encoding in the
%   matrix OUT, based on input data matrix IN and base graph number (1 or
%   2), according to the rules of TS 38.212 Section 5.3.2.
%
%   Each column in IN and OUT represents a code block segment before and
%   after LDPC encoding, respectively. The number of columns in IN or OUT
%   is equal to the number of scheduled code block segments. The number of
%   rows in IN is the number of information bits in an LDPC codeword; the
%   number of rows in OUT is the length of codeword with some information
%   bits punctured. The encoding procedure includes replacing filler bits
%   (represented by -1) with 0 bits, encoding to generate the full LDPC
%   codeword, replacing the filler bit locations in codeword with -1 and
%   information bits puncturing.
%
%   % Example:
%   % LDPC encode two code block segments of length 2560 each (include 36
%   % filler bits at the end) to obtain two encoded code blocks of length
%   % 12800, respectively.
%
%   bgn = 2;               % Base graph number
%   K = 2560;              % Code block segment length
%   F = 36;                % Number of filler bits per code block segment
%   C = 2;                 % Number of code blocks
%   cbs = ones(K-F,C);  
%   fillers = -1*ones(F,C);
%   cbs = [cbs;fillers];   % Code block segments with filler bits
%   codedcbs = nrLDPCEncode(cbs,bgn);
%   size(codedcbs)
%
%   See also nrLDPCDecode, nrCodeBlockSegmentLDPC, nrCRCEncode,
%   nrRateMatchLDPC, nrDLSCHInfo.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    % Validate input
    fcnName = 'nrLDPCEncode';
    validateattributes(in,{'int8','double'},{'2d','real'},fcnName,'IN');

    % Validate base graph number (1 or 2)
    validateattributes(bgn,{'numeric'},{'scalar','integer'},fcnName,'BGN');  
    coder.internal.errorIf(~(bgn==1 || bgn==2), ...
        'nr5g:nrLDPC:InvalidBGN',bgn);

    % Empty in, empty out
    outputProto = zeros(0,0,'like',in);
    if isempty(in)
        out = zeros(0,size(in,2),'like',outputProto);
        return;
    end

    % Obtain input/output size
    [K,C] = size(in);
    if bgn==1
        nsys = 22;
        ncwnodes = 66;
    else
        nsys = 10;
        ncwnodes = 50;
    end
    Zc = K/nsys;
    % Validate input data length
    coder.internal.errorIf(fix(Zc)~=Zc, ...
        'nr5g:nrLDPC:InvalidInputLength',K,nsys,bgn);
    %   Check against all supported lifting sizes
    ZcVec = [2:16 18:2:32 36:4:64 72:8:128 144:16:256 288:32:384];
    coder.internal.errorIf(~any(Zc==ZcVec),'nr5g:nrLDPC:InvalidZc',Zc,bgn);
    N = Zc*ncwnodes;

    % Find filler bits (shortening bits) and replace them with 0 bits
    locs = find(in(:,1)==-1);
    in(locs,:) = 0;

    % Encode all code blocks
    outCBall = nr5g.internal.ldpc.encode(double(in),bgn,Zc);

    % Put filler bits back
    outCBall(locs,:) = -1;

    % Puncture first 2*Zc systematic bits and output
    out = zeros(N,C,'like',outputProto);
    out(:,:) = cast(outCBall(2*Zc+1:end,:),underlyingType(outputProto));
    
end
