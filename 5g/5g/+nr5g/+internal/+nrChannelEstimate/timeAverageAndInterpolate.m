function H = timeAverageAndInterpolate(H,refRBs,p,NRB,R,tdCDM,timeAveraging,N,policy,fracPRGInd,prgKranges)
%TIMEAVERAGEANDINTERPOLATE performs the time direction averaging and
%interpolation.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    nPRG = size(prgKranges,1);

    % Establish labels 'z' for the OFDM symbols containing reference
    % symbols in each RB for this port. The labels are integers which
    % identify different OFDM symbol sets. 'z' is a column vector
    % containing one label for each RB
    [~,~,iz] = unique([zeros([1 N]); refRBs(:,:,p)],'rows','stable');
    z = iz(2:end) - 1;

    % Form unique set of labels 'uz', corresponding to unique OFDM symbol
    % sets, excluding the case of the empty set of OFDM symbols (having
    % label 0)
    uz = 1:max(z);

    % Check if PRGs can be processed in parallel and update fractional PRG
    % indices if necessary
    policyThisPort = policy(p,~isnan(policy(p,:)));
    parallel = (~isempty(policyThisPort) && all(policyThisPort==2));
    if ~parallel
        fracPRGIndThisPort = (1:nPRG);
    else
        symInd = find(~isnan(policy(p,:)));
        fracPRGIndThisPort = fracPRGInd{p,symInd(1)};
    end

    for g = 1:numel(fracPRGIndThisPort)
        % Process the PRGs that need to be processed individually

        prg = fracPRGIndThisPort(g);
        prgStart = prgKranges(prg,1);
        krangeThisPRG = prgKranges(prg,:);
        H = timeAverageAndInterpolateCore(H,refRBs,p,z,uz,NRB,R,tdCDM, ...
            timeAveraging(prg),N,0,krangeThisPRG,prgStart);

    end

    if parallel
        % Process the PRGs that can be processed in parallel

        prgIndices = true(nPRG,1);
        prgIndices(fracPRGIndThisPort) = false;
        allPRGs = (1:nPRG)';
        allPRGs = allPRGs(prgIndices);

        if ~isempty(allPRGs)
            % In corner cases, non-fractional PRGs may have no reference
            % symbols

            prgStarts = prgKranges(allPRGs,1);
            krangeThisPRG = prgKranges(allPRGs(1),:)-prgStarts(1)+1;
            H = timeAverageAndInterpolateCore(H,refRBs,p,z,uz,NRB,R,tdCDM, ...
                timeAveraging(allPRGs),N,1,krangeThisPRG,prgStarts);

        end

    end

end

%% Local functions

% Core function for time direction averaging and interpolation
function H = timeAverageAndInterpolateCore(H,refRBs,p,z,uz,NRB,R,tdCDM,timeAveraging,N,parallel,krangeThisPRG,prgStarts)

    % Get RB range for this block
    rbRange = floor(((krangeThisPRG+parallel*(prgStarts(1)-1))-1)/12);

    nPRG = size(prgStarts,1);

    % For each unique OFDM symbol set
    for zi = 1:numel(uz)

        % Get RBs having this OFDM symbol set
        RBs = find(z==uz(zi))-1;
        RBsThisPRG = RBs(RBs>=rbRange(1) & RBs<=rbRange(2));

        if isempty(RBsThisPRG)
            continue;
        end

        % Calculate OFDM symbol subscripts 'n' for this OFDM symbol set.
        % Also store these as 'n0' which is used to reset these subscripts
        % for each RB block / receive antenna
        n = find(refRBs(RBsThisPRG(1) + 1,:,p)~=0);
        n0 = n;
        numSym = numel(n0);

        % Split into blocks of contiguous RBs. Each column of 'rbidx' will
        % have two rows, giving the index of the first and last RB of the
        % block within 'RBs'
        rbidx = contiguousRBs(RBsThisPRG,NRB);

        % For each contiguous block of RBs
        for b = 1:size(rbidx,2)

            % Calculate frequency subscripts 'k' for the block
            k = ((RBsThisPRG(rbidx(1,b))*12 + 1):(RBsThisPRG(rbidx(2,b))*12 + 12)).';

            % Expand k if necessary
            if parallel
                kFull = nr5g.internal.nrChannelEstimate.expandSubcarrierSubscripts(k-prgStarts(1)+1,prgStarts);
            else
                kFull = k;
            end

            % For each receive antenna
            for r = 1:R

                % Initialize OFDM symbol subscripts 'n' to be used for
                % interpolation
                n = n0;

                % Extract channel estimate. 'Hrp' is a matrix of denoised
                % and frequency averaged / interpolated estimates for the
                % current RB block, port and receive antenna. The rows of
                % 'Hrp' correspond to the subcarriers within the current RB
                % block, the columns to OFDM symbols containing reference
                % symbols
                Hrp = H(kFull,n,r,p);
                Hrp = nr5g.internal.nrChannelEstimate.foldMultiplePRG(Hrp,nPRG);

                % If TD-CDM despreading is configured, replace groups of
                % estimates with a single estimate in the position of the
                % average OFDM symbol index
                if (tdCDM>1)
                    Hrp = Hrp(:,1:tdCDM:end,:);
                    nlCDM = min(tdCDM,numSym);
                    m = mod(numSym,nlCDM);
                    nm = n(end-m+1:end);
                    n = reshape(n(1:end-m),nlCDM,[]);
                    n = [mean(n,1) repmat(mean(nm),1,double(m~=0))];
                end

                % Perform averaging in the time direction
                if parallel
                    avgInd = find(timeAveraging~=1);
                    if ~isempty(avgInd)
                        Havg = Hrp(:,:,avgInd);
                        Havg = pagetranspose(nr5g.internal.nrChannelEstimate.averageColumn(pagetranspose(Havg),timeAveraging(avgInd)));
                        Hrp(:,:,avgInd) = Havg(:,:,:);
                    end
                else
                    if timeAveraging~=1
                        % Explicitly use scalar timeAveraging to avoid
                        % codegen issue
                        Hrp = pagetranspose(nr5g.internal.nrChannelEstimate.averageColumn(pagetranspose(Hrp),timeAveraging));
                    end
                end

                % Obtain a channel estimate for all OFDM symbols. After
                % this step, the columns of 'Hrp' correspond to all OFDM
                % symbols in the slot
                if (numel(n)>1)
                    % For multiple reference OFDM symbols, perform 2-D
                    % interpolation of estimates
                    Hrp = nr5g.internal.nrChannelEstimate.interpolateGrid(2,k,n,Hrp,k,1:N,'linear',0);
                else
                    % For a single reference OFDM symbol, repeat the single
                    % estimate
                    Hrp = repmat(Hrp,[1 N]);
                end
                % Assign the estimate into the appropriate region of the
                % overall channel estimate array
                Hrp = unfoldMultiplePRG(Hrp);
                H(kFull,:,r,p) = Hrp(:,:);

            end

        end

    end
    
end

% Get indices of contiguous sets of RBs in 'RBs', which have maximum value
% 'NRB'
function rbidx = contiguousRBs(RBs,NRB)

    d = [-2; RBs; NRB+1];
    d = find(diff(d)~=1);
    rbidx = [d(1:end-1) d(2:end)-1].';
    
end

% Unfold multiple PRGs
function Aout = unfoldMultiplePRG(Ain)

    nCol = size(Ain,2);
    if nCol==1
        Aout = reshape(Ain,[],1);
    else
        Aout = permute(reshape(permute(Ain,[2 1 3]),nCol,[],1),[2 1]);
    end

end