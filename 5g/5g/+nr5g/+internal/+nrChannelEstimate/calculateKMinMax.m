function [kmin,kmax,kpatternmin,kpatternmax] = calculateKMinMax(kmin,kmax,kpatternmin,kpatternmax,p,n,k,eK,prgKRanges,policy,fracPRGInd,pattern)
%CALCULATEKMINMAX calculates the minimum and maximum subcarrier subscripts
%for all PRGs in the current OFDM symbol of the current port and write into
%kmin and kmax respectively.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    nPRG = size(prgKRanges,1);

    if policy
        % For non-fractional PRGs, store the common kmin and kmax
        % corresponding to pattern in kpatternmin and kpattternmax

        kpattern = pattern(:,1);
        [kminPattern,kmaxPattern] = calculateKMinMaxCore(kpattern,eK);

        kpatternmin(n,p) = kminPattern;
        kpatternmax(n,p) = kmaxPattern;

    end

    for g = 1:nPRG
        % For fractional PRGs, use the actual subscripts which is relative
        % to the whole grid

        if policy && ~any(fracPRGInd==g,'all')
            % Do not calculate kmin or kmax for this PRG if it is to be
            % processed in parallel as it is unnecessary
            continue;
        end

        kThisPRG = k(k>=prgKRanges(g,1) & k<=prgKRanges(g,2));
        [kminThisPRG,kmaxThisPRG] = calculateKMinMaxCore(kThisPRG,eK);

        kmin(n,p,g) = kminThisPRG;
        kmax(n,p,g) = kmaxThisPRG;

    end

end

%% Local function

% Core function for calculating kmin and kmax
function [kmin,kmax] = calculateKMinMaxCore(k,eK)

    if isempty(k)
        kmin = 0;
        kmax = 0;
    else
        ke = k+eK/2;
        rbsubs = unique(floor((ke-1)/12));
        kmin = rbsubs(1)*12-(eK/2)+1;
        kmax = rbsubs(end)*12+(eK/2)+12;
    end

end