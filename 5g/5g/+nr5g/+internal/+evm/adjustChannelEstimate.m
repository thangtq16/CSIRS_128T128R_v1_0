function Hest = adjustChannelEstimate(Hest,locationMap,rsRefIndex,nSymbSlot)
%adjusctChannelEstimate Extrapolate channel estimates for the RBs where
% there is channel allocation with no channel estimate.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

% Copyright 2024 The MathWorks, Inc.

%#codegen

    % Channel REs may not always be present on the same set of RBs as
    % DM-RS. For each such slot, extrapolate the channel coefficients to
    % span the location of these channel allocation regions. This is needed
    % for these scenarios:
    % 1. With ReservedREs of PDSCH
    % 2. DC subcarrier location in any channel
    [numSCs,nSym,R,P,E] = size(Hest);
    nSlots = nSym/nSymbSlot;
    for edgeIdx = 1:E
        H1 = Hest(:,:,:,:,edgeIdx);
        % Ensure slot has channel coefficients for the corresponding
        % channel allocation
        for s = 1:nSlots
            symIdx = (s-1)*nSymbSlot+1:s*nSymbSlot;
            % Find the first DM-RS OFDM symbol location using all the
            % layers
            lmGrid = locationMap(:,symIdx,:);
            firstDmrsLocInSlot = find(any(sum(lmGrid == rsRefIndex,3)));
            % Find the resource blocks allocated for channel in this slot
            scRow = find(any(sum(lmGrid == 1,3),2));
            rxRb = unique(floor((scRow-1)/12));

            % Locate RBs where channel coefficients are present
            [row,~] = find(H1(:,symIdx,1,1));
            HestRb = unique(floor((row-1)/12));

            % Set extrapolateHest to true if allocated RB list does not
            % match list of RBs containing channel coefficients
            extrapolateHest = false;
            for rbIdx = 1:length(rxRb)
                if ~any(HestRb == rxRb(rbIdx))
                    extrapolateHest = true;
                    break;
                end
            end

            % Process only for slots where channel RBs do not contain
            % channel estimates. Using the 'nearest' interpolation method,
            % extrapolate the channel coefficients over the slot span. This
            % method ensures the same channel coefficient is extrapolated
            % over the neighboring frequency region.
            if extrapolateHest && ~isempty(HestRb) && any(firstDmrsLocInSlot)
                for p = 1:P
                    for r = 1:R
                        H_tmp = H1(:,symIdx(firstDmrsLocInSlot),r,p);
                        if sum(abs(H_tmp)) == 0
                            interpEqCoeff = 1e-16.*ones(size(H_tmp,1),1,like=H_tmp);
                        else
                            interpEqCoeff = interp1(find(H_tmp~=0),H_tmp(H_tmp~=0),(1:numSCs).',"nearest","extrap");
                        end
                        interpEqCoeff = repmat(interpEqCoeff,1,nSymbSlot);
                        H1(:,symIdx,r,p) = interpEqCoeff;
                    end
                end
            end
        end
        Hest(:,:,:,:,edgeIdx) = H1;
    end

end
