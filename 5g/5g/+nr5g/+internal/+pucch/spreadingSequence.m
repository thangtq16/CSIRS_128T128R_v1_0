function wn = spreadingSequence(sf,occi)
%spreadingSequence Orthogonal spreading sequence for PUCCH format 2
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   WN = spreadingSequence(SF,OCCI) returns the orthogonal cover code
%   spreading sequence, WN, according to TS 38.211 Tables 6.3.2.5A-1 and
%   6.3.2.5A-2 for these inputs.
%     SF   - Spreading factor. It must be either 2 or 4
%     OCCI - Orthogonal cover code sequence index. It must be greater than
%            or equal to zero and less than SF

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    if sf == 2
        % TS 38.211 6.3.2.5A-1
        w = [+1 +1;  % OCCI equals 0
             +1 -1]; % OCCI equals 1
    else
        % TS 38.211 Table 6.3.2.5A-2
        w = [+1 +1 +1 +1;  % OCCI equals 0
             +1 -1 +1 -1;  % OCCI equals 1
             +1 +1 -1 -1;  % OCCI equals 2
             +1 -1 -1 +1]; % OCCI equals 3
    end
    
    % Extract the orthogonal cover code sequence based on the orthogonal
    % cover code index
    wn = w(occi+1,:);

end
