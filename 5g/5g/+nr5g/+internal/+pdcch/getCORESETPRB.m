% getCORESETPRB Get PRB indices (0-based) associated with CORESET in a BWP
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2021-2022 The MathWorks, Inc.

%#codegen

function coresetprb = getCORESETPRB(coreset,NStartBWP)

    % Expand the 6RBs in the CORESET
    fr = coreset.FrequencyResources~=0;        % Create a logical bitmap (each bit represents 6 RB)
    allPRB = reshape(0:6*length(fr)-1,6,[]);   % 0-based indices spanning all the RB associated with bitmap
    prbIdx = reshape(allPRB(:,fr),[],1);       % Select column groups of 6 RB indices using bitmap, then reshape result into single column

    % Calculate PRB offset to first RB associated with signalled bitmap
    if coreset.CORESETID ~= 0

        if isempty(coreset.RBOffset)
            % Place FreqRes, relative to NStartBWP, at 6*ceil(NStartBWP/6), the first common RB that can be used by the CORESET
            nStartBWP = double(NStartBWP);
            nrb0 = 6*ceil(nStartBWP/6) - nStartBWP;   % Offset of CORESET frequency resources in BWP (first group of 6 PRB of CORESET)
        else
            % Or, if RBOffset is provided, then the first CORESET PRB does not get aligned to a block of 6 CRB, and can be offset according to rb-Offset value
            nrb0 = coreset.RBOffset;
        end

    else
        % For CORESET 0, the size of the BWP is assumed equal to the size
        % of the CORESET. As CORESET0 should fit in a BWP of the same size,
        % don't shift CORESET in BWP.
        nrb0 = 0;
    end

    % Apply PRB offset to get (0-based) PRB indices of CORESET
    coresetprb = nrb0 + prbIdx;
    
end

