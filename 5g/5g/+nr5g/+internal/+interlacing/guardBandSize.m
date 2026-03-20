function gbsize = guardBandSize(carrier)
%guardBandSize return intracell guard band sizes from carrier
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023-2024 The MathWorks, Inc.

%#codegen

    if isnumeric(carrier.IntraCellGuardBands)
        gbsiz = carrier.IntraCellGuardBands;
    else
        % Use the guard bands corresponding to the SCS of the carrier
        scs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carrier.IntraCellGuardBands,'SubcarrierSpacing');
        idx = scs==carrier.SubcarrierSpacing;
        if any(idx)
            gbsiz = carrier.IntraCellGuardBands{idx}.GuardBandSize;
        else
            gbsiz = zeros(0,2);
        end
    end

    % Trim guard bands outside the carrier frequency span
    if ~isempty(gbsiz)
        
        % Lowest and highest RB of the carrier relative to NStartGrid
        crb1 = 0;
        crb2 = carrier.NSizeGrid-1;

        % Lowest and highest RB of the guard bands
        gb1 = gbsiz(:,1);
        gb2 = sum(gbsiz,2)-1;

        % Guard bands partially inside the carrier
        in = (gb2>=crb1) & (gb1<=crb2);
        gbi1 = gb1(in);
        gbi2 = gb2(in);

        % Limit the guard bands to the frequency range of the carrier
        gbi1 = min(max(gbi1,crb1),crb2);
        gbi2 = max(min(gbi2,crb2),crb1);

        % Change format back to [start len]
        gbsize = [gbi1 (gbi2-gbi1+1)];
        
    else
        gbsize = gbsiz;
    end

end

