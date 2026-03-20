function [rmsEVM,peakEVM,ev] = evm(eqgrid,idealgrid,locationmap,cp,scs)
%evm Error vector magnitude (EVM) measurement
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [RMSEVM,PEAKEVM,EV] = ...
%   nr5g.internal.evm.evm(EQGRID,IDEALGRID,LOCATIONMAP,CP,SCS) calculates
%   the normalized error vector EV for each slot, each layer and across all
%   edges given the equalized grid EQGRID, the reference grid IDEALGRID,
%   the location identifier in the grid LOCATIONMAP, cyclic prefix CP, and
%   subcarrier spacing SCS. This EV is used to measure the root mean square
%   and peak error vector magnitude (EVM) for the available frames of 1 ms.
%   For each frame, the EVM comprising of higher edge is considered for
%   overall EVM across frames as mentioned in TS 38.104 Appendix B and C.
%   When CP and SCS are not provided, all slots are considered as part of
%   single frame.

%   Copyright 2024-2025 The MathWorks, Inc.

%#codegen

narginchk(3,5);
[numSCs,numSym,nLayers,nEdge] = size(eqgrid);
if nargin == 3
    nSlots = 1;
    nSymbSlot = numSym;
    nFrames = 1;
    numSymInFrame = numSym;
else
    narginchk(5,5);
    nSymbSlot = strcmpi(cp,{'normal','extended'})*[14 12].';
    nSlotsPerSubframe = scs/15;
    nSlotsPerFrame = 10*nSlotsPerSubframe;
    nSlots = numSym/nSymbSlot;                                  % The function expects integer number of slots
    nFrames = max(1,floor(nSlots/nSlotsPerFrame));
    numSymInFrame = nSlotsPerFrame*nSymbSlot;
end

ev = zeros([numSCs,numSym,nLayers,nEdge],like=idealgrid);
locationMapUniqueValues = unique(locationmap(:));
locationMapPositiveValues = locationMapUniqueValues(locationMapUniqueValues>0);

% Find error vector grid of all the entities in the grid
for nIdx = numel(locationMapPositiveValues):-1:1
    notionalRefValue = locationMapPositiveValues(nIdx);
    for slotIdx = 1:nSlots
        % Find if there are any index values in notional grid
        symIdx = (slotIdx-1)*nSymbSlot + (1:nSymbSlot);
        for edgeIdx = 1:nEdge
            for layerIdx = 1:nLayers
                % Find the EV for all the valid symbols within a slot
                locationMapToProcess = locationmap(:,symIdx,layerIdx,1);
                idealGridToProcess = idealgrid(:,symIdx,layerIdx,edgeIdx);
                eqGridToProcess = eqgrid(:,symIdx,layerIdx,edgeIdx);
                evGridToProcess = ev(:,symIdx,layerIdx,edgeIdx);
                indices = find(locationMapToProcess == notionalRefValue);
                if any(indices,"all")
                    idealSymbols = idealGridToProcess(indices);
                    eqSymbols = eqGridToProcess(indices);
                    rawEVMInfo = nr5g.internal.evm.rawEVM(eqSymbols,idealSymbols);
                    evGridToProcess(indices) = rawEVMInfo.EV;
                end
                ev(:,symIdx,layerIdx,edgeIdx) = evGridToProcess;
            end
        end
    end
end

evFrame = NaN([numSCs numSym nLayers],like=eqgrid);
for frameIdx = 0:nFrames-1
    refSym = frameIdx*numSymInFrame;
    symIdx = refSym + (1:numSymInFrame);
    symIdx = symIdx(symIdx <= numSym);
    if any(symIdx)
        evFrameTemp = evFrame(:,symIdx,:);
        rmsEVM = -1*ones(1,class(eqgrid));
        locationMapToProcess = locationmap(:,symIdx,:,1);
        indices = find(locationMapToProcess == 1);
        if any(indices,"all")
            for edgeIdx = 1:nEdge
                evGridToProcess = ev(:,symIdx,:,edgeIdx);
                frameEV = nr5g.internal.evm.rawEVM(evGridToProcess(indices));
                if frameEV.RMS > rmsEVM
                    rmsEVM = frameEV.RMS;
                    ev2 = frameEV.EV;
                    evFrameTemp(indices) = ev2;
                end
            end
        end
        evFrame(:,symIdx,:) = evFrameTemp;
    end
end

% Overall EVM
overallIndices = ~isnan(evFrame);
overallEVM = nr5g.internal.evm.rawEVM(evFrame(overallIndices));
rmsEVM = overallEVM.RMS;
peakEVM = overallEVM.Peak;

end
