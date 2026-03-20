function intraCellGuardBands = adjustGuardbandsFrequencyReference(intraCellGuardBands,carrier,bwp)
%adjustGuardbandsFrequencyReference adjust GB frequency reference point from carrier to BWP
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2024 The MathWorks, Inc.

%#codegen

    scs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(intraCellGuardBands,'SubcarrierSpacing');
    idx = scs==carrier.SubcarrierSpacing;
    
    % Change the RB reference point of guard bands from NStartGrid to
    % NStartBWP
    if any(idx) && ~isempty(intraCellGuardBands{idx}.GuardBandSize)
        gb = intraCellGuardBands{idx}.GuardBandSize;
        guardBandsRBBWP = [gb(:,1) gb(:,1)+gb(:,2)] + carrier.NStartGrid - bwp.NStartBWP;

        % Limit the first and last RB of the guardbands to be positive,
        % this is, within the BWP
        guardBandsRBBWP = max(guardBandsRBBWP,0);
        gbstart = guardBandsRBBWP(:,1);
        gbwidth = guardBandsRBBWP(:,2) - gbstart;
        
        intraCellGuardBands{idx}.GuardBandSize = [gbstart gbwidth];
    end

end