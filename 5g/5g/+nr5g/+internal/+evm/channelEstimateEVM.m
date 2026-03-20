function Hest = channelEstimateEVM(rxGrids,refGrid,cdmLengths,L,dlFlag)
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%  HEST = channelEstimateEVM(RXGRIDS,REFGRID,CDMLENGTHS,L,DLFLAG)
%  Estimates the channel returning the channel coefficients HEST.
%  HEST is a K-by-N-by-R-by-P-by-E array where K is the number of
%  subcarriers, N is the number of symbols, and R is the number of
%  receive antennas, P is the number of reference signal ports, and E is
%  the number of edges. The channel array for E equals to 1 represents the
%  low edge and the channel array for E equals to 2 represents the high
%  edge.
%  * RXGRIDS is an array of size K-by-N-by-R-by-E.
%  * REFGRID is a predefined K-by-N-by-P reference array with nonzero
%  elements representing the reference symbols in their appropriate
%  locations. REFGRID can span multiple slots.
%  * CDMLENGTHS is a 2-element row vector [FD TD] specifying the length
%  of FD-CDM and TD-CDM despreading to perform.
%  * L is the number of symbols in a slot.
%  * DLFLAG when set to true, enables smoothening of the channel
%  coefficients in the frequency direction, using a moving average
%  filter. Smoothening is performed as described in TS 38.104 Annex B.6
%  (FR1) or C.6 (FR2). When this parameter is not specified, the time
%  averaging across the duration of REFGRID is enabled.

% Copyright 2024 The MathWorks, Inc.

%#codegen

    [nSC,nSym,nRx,nEdges] = size(rxGrids);
    nLayers = size(refGrid,3);
    Hest = zeros([nSC,nSym,nRx,nLayers,nEdges],like=rxGrids);

    % Extract channel estimation mode
    evm3GPP = (nEdges == 2);
    nSlots = floor(nSym/L);

    % For each slot, estimate the channel coefficients
    for slotIdx = 1:nSlots
        % If a symbol index exceeds the length of the reference grid,
        % remove it
        symIdx = (slotIdx-1)*L+1:slotIdx*L;
        symIdx(symIdx>nSym) = [];

        % Use a smoothing filter in the frequency direction when dlFlag
        % is true
        if evm3GPP
            if dlFlag
                Hest(:,symIdx,:,:,1) = nr5g.internal.evm.channelEstimateEVM3GPP( ...
                    rxGrids(:,symIdx,:,1),refGrid(:,symIdx,:),'movingAvgFilter',cdmLengths);
                Hest(:,symIdx,:,:,2) = nr5g.internal.evm.channelEstimateEVM3GPP( ...
                    rxGrids(:,symIdx,:,2),refGrid(:,symIdx,:),'movingAvgFilter',cdmLengths);
            else
                Hest(:,symIdx,:,:,1) = nr5g.internal.evm.channelEstimateEVM3GPP( ...
                    rxGrids(:,symIdx,:,1),refGrid(:,symIdx,:),'',cdmLengths);
                Hest(:,symIdx,:,:,2) = nr5g.internal.evm.channelEstimateEVM3GPP( ...
                    rxGrids(:,symIdx,:,2),refGrid(:,symIdx,:),'',cdmLengths);
            end
        else
            Hest(:,symIdx,:,:,1) = nrChannelEstimate(rxGrids(:,symIdx,:,1), ...
                refGrid(:,symIdx,:),CDMLengths=cdmLengths);
        end
    end

end
