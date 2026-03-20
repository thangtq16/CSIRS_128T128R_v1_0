function wn = blockWiseSpreadingSequence(sf,occi)
%blockWiseSpreadingSequence Blockwise orthogonal spreading sequence for PUCCH
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   WN = blockWiseSpreadingSequence(SF,OCCI) returns the orthogonal cover
%   code spreading sequence, WN, according to TS 38.211 Section 6.3.2.6.3,
%   for these inputs.
%     SF   - Spreading factor. It must be either 2 or 4
%     OCCI - Orthogonal cover code sequence index. It must be greater than
%            or equal to zero and less than SF

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    if sf == 2
        % TS 38.211 Table 6.3.2.6.3-1
        w = [1  1;  % occi equals 0
             1 -1]; % occi equals 1
    else
        % TS 38.211 Table 6.3.2.6.3-2
        w = [1  1   1  1;   % occi equals 0
             1 -1j -1  1j;  % occi equals 1
             1 -1   1 -1;   % occi equals 2
             1  1j -1 -1j]; % occi equals 3
    end

    % Extract the orthogonal cover code sequence based on the orthogonal
    % cover code index
    wn = w(occi+1,:);

end
