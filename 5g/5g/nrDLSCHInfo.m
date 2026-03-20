function info = nrDLSCHInfo(tbs,tcr)
%nrDLSCHInfo 5G DL-SCH segmentation information
%   INFO = nrDLSCHInfo(TBS,TCR) returns the structure INFO containing the
%   Downlink Shared Channel (DL-SCH) CRC attachment, code block
%   segmentation and channel coding related information for a given
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
%   nrDLSCHInfo(8456,517/1024)
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
%   See also nrDLSCH, nrDLSCHDecoder, nrPDSCH, nrPDSCHDecode.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    narginchk(2,2);

    % Call the shared channel utility
    info = nr5g.internal.getSCHInfo(tbs,tcr);

end
