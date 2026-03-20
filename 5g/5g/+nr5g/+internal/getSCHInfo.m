function info = getSCHInfo(tbs,tcr)
%getSCHInfo Shared channel segmentation information
%   INFO = nr5g.internal.getSCHInfo(TBS,TCR) returns the structure INFO
%   containing the Shared Channel (DL-SCH or UL-SCH) CRC attachment, code
%   block segmentation and channel coding related information for a given
%   transport block length TBS and target code rate TCR.
%
%   INFO contains the following fields:
%   CRC - CRC polynomial selection ('16' or '24A')
%   L   - Number of CRC bits (16 or 24)
%   BGN - LDPC base graph selection (1 or 2)
%   C   - Number of code blocks
%   Lcb - Number of parity bits per code block (0 or 24)
%   F   - Number of <NULL> filler bits per code block
%   Zc  - Lifting size selection
%   K   - Number of bits per code block after CBS
%   N   - Number of bits per code block after LDPC coding
%
%   Example:
%   % Show DL-SCH information before rate matching for an input transport
%   % block of length 8456 and target code rate 517/1024. The info
%   % structure fields show that there are 312 filler bits, the total size
%   % of the one segment after code block segmentation is 4576 and after
%   % LDPC coding is 13728 as well as other DL-SCH related information.
%
%   nr5g.internal.getSCHInfo(8456,517/1024)
%
%   % The above example returns:
%   %      CRC: '24A'
%   %        L: 24
%   %      BGN: 1
%   %        C: 2
%   %      Lcb: 24
%   %        F: 312
%   %       Zc: 208
%   %        K: 4576
%   %        N: 13728
%
%   See also nrDLSCHInfo, nrULSCHInfo.

%   Copyright 2018-2020 The MathWorks, Inc.

%#codegen

    % Validate inputs
    fcnName = 'getSCHInfo';
    validateattributes(tbs,{'numeric'},{'scalar','integer','nonnegative'},fcnName,'TBS');
    validateattributes(tcr,{'numeric'},{'real','scalar','>',0,'<',1},fcnName,'TCR');

    % Get base graph number and CRC information
    bgInfo = nr5g.internal.getBGNInfo(tbs,tcr);

    % Get code block segment information
    cbInfo = nr5g.internal.getCBSInfo(bgInfo.B,bgInfo.BGN);

    % Get number of bits (including filler bits) to be encoded by LDPC
    % encoder
    if bgInfo.BGN == 1
        N = 66*cbInfo.Zc;
    else
        N = 50*cbInfo.Zc;
    end

    % Combine information into the output structure
    info.CRC      = bgInfo.CRC;             % CRC polynomial
    info.L        = bgInfo.L;               % Number of CRC bits
    info.BGN      = bgInfo.BGN;             % Base graph number
    info.C        = cbInfo.C;               % Number of code block segments
    info.Lcb      = cbInfo.Lcb;             % Number of parity bits per code block
    info.F        = cbInfo.F;               % Number of <NULL> filler bits per code block
    info.Zc       = cbInfo.Zc;              % Selected lifting size
    info.K        = cbInfo.K;               % Number of bits per code block after CBS
    info.N        = N;                      % Number of bits per code block after LDPC coding

    % Modify the output fields if tbs is empty or zero
    if ~tbs
        info.L    = 0;
        info.F    = 0;
        info.Zc   = 2;
        info.K    = 0;
        info.N    = 0;
    end

end
