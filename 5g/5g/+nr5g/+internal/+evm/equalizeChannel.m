function [eqGrid,csiGrid] = equalizeChannel(rxGrids,Hest,notionalGridAllSlots,noiseEst)
%equalizeChannnel Performs equalization to remove channel effects
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

% Copyright 2024 The MathWorks, Inc.

%#codegen

    [numSCs,nSym,~,nLayers,nEVMWindowLocations] = size(Hest);
    gridDimsWithEdges = [numSCs nSym nLayers nEVMWindowLocations];
    csiGrid = zeros(gridDimsWithEdges,class(rxGrids));
    eqGrid = zeros(gridDimsWithEdges,like=rxGrids);

    channelAllInd = find(notionalGridAllSlots);
    if any(channelAllInd)
        for edgeIdx = 1:nEVMWindowLocations
            [pxschAllRx,pxschAllHest,~,~,~,reInd] = nrExtractResources( ...
                channelAllInd,rxGrids(:,:,:,edgeIdx),Hest(:,:,:,:,edgeIdx),notionalGridAllSlots);
            [eqRxLow,csiRxLow] = nrEqualizeMMSE(pxschAllRx,pxschAllHest,noiseEst);
            eqGridEdge = eqGrid(:,:,:,edgeIdx);
            eqGridEdge(reInd) = eqRxLow;
            eqGrid(:,:,:,edgeIdx) = eqGridEdge;
            csiGridLow = csiGrid(:,:,:,edgeIdx);
            csiGridLow(reInd) = real(csiRxLow);
            csiGrid(:,:,:,edgeIdx) = csiGridLow;
        end
    end

end
