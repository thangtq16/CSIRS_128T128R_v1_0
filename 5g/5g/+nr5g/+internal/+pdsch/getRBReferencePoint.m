function rbrefpoint = getRBReferencePoint(NStartGrid,NStartBWP,DMRSReferencePoint)
%getRBReferencePoint Get the resource block reference point
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2022 The MathWorks, Inc.

%#codegen

    if strcmp(DMRSReferencePoint,'CRB0')
        % The reference point is subcarrier 0 of common resource block 0
        % (CRB 0)
        if isempty(NStartBWP)
            % If nStartBWP is empty, set the reference point to the start of the carrier
            rbrefpoint = double(NStartGrid);
        else
            rbrefpoint = double(NStartBWP(1));
        end
    else % PRB0
        % The reference point is subcarrier 0 of the first PRB of the BWP
        % (PRB 0)
        rbrefpoint = 0;
    end

end