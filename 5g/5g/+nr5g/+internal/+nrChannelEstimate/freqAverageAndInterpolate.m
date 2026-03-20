function H = freqAverageAndInterpolate(H,n,r,p,kmin,kmax,kpatternmin,kpatternmax,freqAveraging,prgKranges,policy,fracPRGInd)
%FREQAVERAGEANDINTERPOLATE performs the frequency direction averaging and
%interpolation.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    nPRG = size(prgKranges,1);

    for g = 1:numel(fracPRGInd)
        % Process the PRGs that need to be processed individually

        prg = fracPRGInd(g);
        krangeThisPRG = (kmin(n,p,prg):kmax(n,p,prg)).';
        prgStart = prgKranges(prg,1);
        H = freqAverageAndInterpolateCore(H,n,r,p,krangeThisPRG, ...
            freqAveraging(prg),0,prgStart);

    end

    if policy && isempty(coder.target)
        % Process the PRGs that can be processed in parallel

        prgIndices = true(nPRG,1);
        prgIndices(fracPRGInd) = false;
        allPRGs = (1:nPRG)';
        allPRGs = allPRGs(prgIndices);

        if ~isempty(allPRGs)
            % In corner cases, non-fractional PRGs may have no reference
            % symbols

            krangeThisPRG = (kpatternmin(n,p):kpatternmax(n,p)).';
            prgStarts = prgKranges(allPRGs,1);
            H = freqAverageAndInterpolateCore(H,n,r,p,krangeThisPRG, ...
                freqAveraging(allPRGs),1,prgStarts);

        end

    end

end

%% Local functions

% Core function for frequency direction averaging and interpolation
function H = freqAverageAndInterpolateCore(H,n,r,p,krange,freqAveraging,policy,prgStarts)

    if freqAveraging==1
        return;
    elseif ~isscalar(freqAveraging)
        % Skip PRGs whose frequency averaging window is 1
        prgStarts = prgStarts(freqAveraging~=1);
    end

    k = krange(3:6:end);
    nPRG = size(prgStarts,1);

    % Expand subcarrier subscripts when processing multiple PRBs in
    % parallel
    if policy
        kFull = nr5g.internal.nrChannelEstimate.expandSubcarrierSubscripts(k,prgStarts);
        krangeFull = nr5g.internal.nrChannelEstimate.expandSubcarrierSubscripts(krange,prgStarts);
    else
        kFull = k;
        krangeFull = krange;
    end

    % Extract channel estimate for those subscripts. 'Hlrp' is a column
    % vector containing the denoised estimates for 2 subcarriers per
    % RB for the current port, OFDM symbol and receive antenna, and reshape
    % into multiple pages when processing multiple PRGs
    Hlrp = H(kFull,n,r,p);
    Hlrp = reshape(Hlrp,[],1,nPRG);

    % Perform averaging in the frequency direction for PRGs whose
    % corresponding frequency averaging window is not 1
    Hlrp = nr5g.internal.nrChannelEstimate.averageColumn(Hlrp,freqAveraging(freqAveraging~=1));

    % Perform interpolation of estimates in the frequency direction to give
    % a channel estimate for all subcarriers, and assign the estimate into
    % the appropriate region of the overall channel estimate array
    Hout = nr5g.internal.nrChannelEstimate.interpolateGrid(1,k,n,Hlrp,krange,n,'spline',1);
    Hout = reshape(Hout,[],1);
    H(krangeFull,n,r,p) = Hout(:,1);

end