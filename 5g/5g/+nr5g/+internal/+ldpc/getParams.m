function cfg = getParams(bgn,Zc)
%getParams NR LDPC Parameters
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   See also nrLDPCDecode, nrLDPCEncode.

%   Copyright 2018-2021 The MathWorks, Inc.

%#codegen

persistent bgs
if isempty(bgs)
    bgs = coder.load('nr5g/internal/ldpc/baseGraph');
end

persistent decoderCfg
if isempty(decoderCfg)
    decoderCfg = coder.nullcopy(cell(2,8,384)); % bgn, setIdx, Zc
end

persistent doInit
if isempty(doInit)
    doInit = true(2,8,384); % bgn, setIdx, Zc
end

% Get lifting set number
ZSets = {[2  4  8  16  32  64 128 256],... % Set 1
         [3  6 12  24  48  96 192 384],... % Set 2
         [5 10 20  40  80 160 320],...     % Set 3
         [7 14 28  56 112 224],...         % Set 4
         [9 18 36  72 144 288],...         % Set 5
         [11 22 44  88 176 352],...        % Set 6
         [13 26 52 104 208],...            % Set 7
         [15 30 60 120 240]};              % Set 8

coder.unroll();
for setIdx = 1:8    % LDPC lifting size set index
    if any(Zc==ZSets{setIdx})
        break;
    end
end

% Get the matrix with base graph number 'bgn' and set number 'setIdx'.
% The element of matrix V in the following is H_BG(i,j)*V(i,j), where
% H_BG(i,j) and V(i,j) are defined in TS 38.212 5.3.2; if V(i,j) is not
% defined in Table 5.3.2-2 or Table 5.3.2-3, the elements are -1.
switch bgn
    case 1
        switch setIdx
            case 1
                V = bgs.BG1S1;
            case 2
                V = bgs.BG1S2;
            case 3
                V = bgs.BG1S3;
            case 4
                V = bgs.BG1S4;
            case 5
                V = bgs.BG1S5;
            case 6
                V = bgs.BG1S6;
            case 7
                V = bgs.BG1S7;
            otherwise % 8
                V = bgs.BG1S8;
        end
    otherwise % bgn = 2
        switch setIdx
            case 1
                V = bgs.BG2S1;
            case 2
                V = bgs.BG2S2;
            case 3
                V = bgs.BG2S3;
            case 4
                V = bgs.BG2S4;
            case 5
                V = bgs.BG2S5;
            case 6
                V = bgs.BG2S6;
            case 7
                V = bgs.BG2S7;
            otherwise % 8
                V = bgs.BG2S8;
        end
end

if doInit(bgn,setIdx,Zc)
    doInit(bgn,setIdx,Zc) = false;
    % Get shift values matrix
    P = nr5g.internal.ldpc.calcShiftValues(V,Zc);
    decoderCfg{bgn,setIdx,Zc} = ldpcDecoderConfig(ldpcQuasiCyclicMatrix(Zc,P));
end

cfg = decoderCfg{bgn,setIdx,Zc};
