function [idealSlotsAllEdges,equalizedSlotsAllEdges,llrs] = getI1ReferenceSignalGrid( ...
        carrier,channel,eqGrid,csiGrid,refGridAllRS,notionalGridAllSlots,noiseEst,expSlots)
%getI1ReferenceSignalGrid Re-construct the i1(v) reference signal in
%frequency domain
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   The first ideal signal i1(v) is constructed by the measuring equipment
%   according to the relevant TX specifications, using the following
%   parameters:
%   - demodulated data content,
%   - nominal carrier frequency,
%   - nominal amplitude and phase for each subcarrier

% Copyright 2024-2025 The MathWorks, Inc.

%#codegen

    % Initialize variables
    [numSCs,nSym,nLayers,nEVMWindowLocations] = size(eqGrid);
    nSlots = numel(expSlots);
    nSymbSlot = carrier.SymbolsPerSlot;
    gridDimsOneSlot = [numSCs nSymbSlot nLayers];
    gridDimsAllSym = [numSCs nSym nLayers];
    gridDimsWithEdges = [gridDimsAllSym nEVMWindowLocations];
    inDataType = class(eqGrid);
    [equalizedSlotsAllEdges,idealSlotsAllEdges] = deal(zeros(gridDimsWithEdges,like=eqGrid));
    transformPrecoding = isprop(channel,"TransformPrecoding") && channel.TransformPrecoding;
    llrs = cell(nSlots,nEVMWindowLocations);
    for cellIdx = 1:numel(llrs)
        tmpInit = zeros(0,0,inDataType);
        coder.varsize("tmpInit")
        if isa(channel,"nrPDSCHConfig") || isa(channel,"nrPUSCHConfig")
            llrs{cellIdx} = repmat({tmpInit},1,channel.NumCodewords);
        else
            llrs{cellIdx} = tmpInit;
        end
    end

    % Loop through each allocated slot to re-generate the reference symbols
    % by following these steps:
    % 1. Perform physical channel demodulation or decoding to get the LLRs
    % 2. Scale the LLRs based on CSI
    % 3. Re-generate the symbols by hard slicing the LLRs
    for slotIdx = 1:nSlots

        % Extract the relevant slot, channel estimates, and allocated REs
        nslot = expSlots(slotIdx);
        if nslot < 0
            continue
        end
        carrier.NSlot = nslot;
        for e = 1:nEVMWindowLocations
            currentSlotIdx = slotIdx-1;
            symNumbers = currentSlotIdx*nSymbSlot+(1:nSymbSlot);
            notionalGridToProcess = notionalGridAllSlots(:,symNumbers,:);
            resGridToUse = refGridAllRS(:,symNumbers,:);
            channelIndicesLoc1 = find(notionalGridToProcess == 1);
            if transformPrecoding
                % Channel indices are combination of both data and PT-RS
                tmpChannelIndices = find(notionalGridToProcess == 1 | notionalGridToProcess == 3);
            else
                tmpChannelIndices = channelIndicesLoc1;
            end
            dmrsIndices = find(notionalGridToProcess == 2);
            ptrsIndices = find(notionalGridToProcess == 3);
            % Reshape the channel indices in-terms of number of layers to
            % pass it to the decode functions appropriately
            numChannelIndices = numel(tmpChannelIndices);
            channelIndices = ones(numChannelIndices/nLayers,nLayers);
            if numChannelIndices
                channelIndices = reshape(tmpChannelIndices,[],nLayers);
            end

            % Extract all the channel elements for low edge
            if numChannelIndices
                eqGridToProcess = eqGrid(:,symNumbers,:,e);
                csiGridToProcess = csiGrid(:,symNumbers,:,e);
                eqGridEdge = eqGridToProcess(channelIndices);
                csiEdge = csiGridToProcess(channelIndices);
                dmrsEq = eqGridToProcess(dmrsIndices);
                ptrsEq = eqGridToProcess(ptrsIndices);
                if transformPrecoding && ~isempty(ptrsIndices)
                    % De-precode the channel
                    prbset = nr5g.internal.allocatedPhysicalResourceBlocks(carrier,channel);
                    deprecode = nrTransformDeprecode(eqGridEdge,numel(prbset));
                    % Extract the PT-RS symbols
                    tempGrid = zeros(gridDimsOneSlot,like=eqGrid);
                    tempGrid(channelIndices) = deprecode;
                    ptrsEq = tempGrid(ptrsIndices);
                end

                % Perform decoding and re-encoding if all the symbols are
                % valid
                channelEq = eqGridEdge;
                channelRef = zeros(size(channelEq),like=channelEq);
                reencodeFlag = any(strcmpi(class(channel),{'nrPDSCHConfig','nrPUSCHConfig','nrPDCCHConfig'}));
                if all(isfinite(eqGridEdge(:)))
                    if reencodeFlag
                        switch class(channel)
                            case "nrPDSCHConfig"
                                [pxychLLRs,rxSymbols] = nrPDSCHDecode(carrier,channel, ...
                                    eqGridEdge,noiseEst);
                                % Store LLRs for current slot
                                llrs{slotIdx,e} = pxychLLRs;
                            case "nrPUSCHConfig"
                                % In uplink, with transform precoding and PT-RS
                                % enabled, eqGridEdge contains both data and
                                % PT-RS. Therefore, use the second output of
                                % nrPUSCHDecode function to get the equalized
                                % data symbols for all cases. Typecast to
                                % double to ensure no codegen issues from
                                % nrPUSCHDecode
                                channel.TransmissionScheme = "nonCodeBook";
                                [pxychLLRs,rxSymbols] = nrPUSCHDecode(carrier,channel, ...
                                    eqGridEdge,noiseEst,UniformCellOutput=true);
                                % Store LLRs for current slot
                                llrs{slotIdx,e} = pxychLLRs;
                            otherwise % nrPDCCHConfig
                                % Get the value of nRNTI and nID. When PDCCH is
                                % in UE-specific search space (USS) and
                                % higher-layer parameter
                                % pdcch-DMRS-ScramblingID is configured, use
                                % the C-RNTI which is the value of RNTI
                                % property and use pdcch-DMRS-ScramblingID for
                                % the value of nID. Else, use 0 for nRNTI and
                                % NCellID for nID. For reference, see Section
                                % 7.3.2.3 of 3GPP TS 38.211.
                                if strcmpi(channel.SearchSpace.SearchSpaceType,"ue") && ...
                                        ~isempty(channel.DMRSScramblingID)
                                    % In this case, RNTI of PDCCH is considered
                                    % as C-RNTI and nRNTI is C-RNTI. Validate
                                    % nRNTI to be positive. The upper limit of
                                    % nRNTI is validated in the nrPDCCHDecode
                                    % function. For the range of C-RNTI, see
                                    % table 7.1-1 of 3GPP TS 38.321.
                                    validateattributes(channel.RNTI,{'numeric'}, ...
                                        {"scalar","positive","integer"},"","NRNTI")
                                    nRNTI = channel.RNTI;
                                    nID = channel.DMRSScramblingID;
                                else
                                    nRNTI = 0;
                                    nID = carrier.NCellID;
                                end
                                pdcchLLRs = nrPDCCHDecode(eqGridEdge,nID,nRNTI,noiseEst);
                                pxychLLRs = {pdcchLLRs};
                                rxSymbols = {eqGridEdge};
                                % Store LLRs for current slot
                                llrs{slotIdx,e} = pdcchLLRs;
                        end

                        % Get the CSI based on transform precoding flag
                        if transformPrecoding
                            % Transform de-precode CSI
                            prbset = nr5g.internal.allocatedPhysicalResourceBlocks(carrier,channel);
                            MRB = length(prbset);
                            MSC = MRB*12;
                            csiDeprecode = nrTransformDeprecode(csiEdge,MRB) / sqrt(MSC);
                            csiRepmat = repmat(csiDeprecode(1:MSC:end,:).',1,MSC).';
                            % Only single codeword is supported for transform
                            % precoding. Remove the PT-RS indices from CSI and
                            % reshape it.
                            tmp = zeros(gridDimsOneSlot,inDataType);
                            tmp(channelIndices) = real(csiRepmat);
                            csiTemp = tmp(notionalGridToProcess==1);
                            csi = reshape(csiTemp,size(rxSymbols{1}));
                        else
                            csi = csiEdge;
                        end

                        % Scale LLRs by CSI. When CSI is not finite, ignore
                        % CSI scaling. This case arises when Hest is
                        % all-zeros and having multiple streams.
                        if all(isfinite(csi(:)))
                            numCWs = size(pxychLLRs,2);
                            currentCsi = nrLayerDemap(csi);    % CSI layer demapping
                            for cwIdx = 1:numCWs
                                Qm = length(pxychLLRs{cwIdx})/length(rxSymbols{cwIdx});             % bits per symbol
                                currentCsi{cwIdx} = reshape(repmat(currentCsi{cwIdx}.',Qm,1),[],1); % expand by each bit per symbol
                                pxychLLRs{cwIdx} = pxychLLRs{cwIdx}.*currentCsi{cwIdx}(:);          % scale
                            end
                        end

                        % Obtain reference symbols using hard slicing of the
                        % PDSCH LLRs
                        hardBits = cellfun(@(x) double(x<0),pxychLLRs,UniformOutput=false);
                        switch class(channel)
                            case "nrPDSCHConfig"
                                channelSymbols = nrPDSCH(carrier,channel,hardBits, ...
                                    OutputDataType=inDataType);
                            case "nrPUSCHConfig"
                                % In uplink, the data reference symbols are
                                % just before transform precoding stage. In
                                % case transform precoding is enabled along
                                % with PT-RS, the data symbols contain the
                                % information related to PT-RS as well.
                                % Therefore, use transform precoding false to
                                % get the reference symbols containing only
                                % data and no PT-RS.
                                channel.TransformPrecoding = false;
                                channelSymbols = nrPUSCH(carrier,channel,hardBits, ...
                                    OutputDataType=inDataType);
                                channel.TransformPrecoding = transformPrecoding;
                            otherwise % nrPDCCHConfig
                                channelSymbols = nrPDCCH(hardBits{1},nID,nRNTI, ...
                                    OutputDataType=inDataType);
                        end
                        channelEq = nrLayerMap(rxSymbols,nLayers);
                        channelRef = channelSymbols;
                    else
                        channelRef = resGridToUse(channelIndicesLoc1);
                    end
                end

                % Map all the relevant channel entities to the equalized
                % and reference grids
                if any(channelIndicesLoc1)
                    channelIndices = reshape(channelIndicesLoc1,[],nLayers);
                    tmpGrid = equalizedSlotsAllEdges(:,symNumbers,:,e);
                    tmpGrid(channelIndices) = channelEq;
                    tmpGrid(dmrsIndices) = dmrsEq;
                    tmpGrid(ptrsIndices) = ptrsEq;
                    equalizedSlotsAllEdges(:,symNumbers,:,e) = tmpGrid;

                    tmpGrid = idealSlotsAllEdges(:,symNumbers,:,e);
                    tmpGrid(channelIndices) = channelRef;
                    tmpGrid(dmrsIndices) = resGridToUse(dmrsIndices);
                    tmpGrid(ptrsIndices) = resGridToUse(ptrsIndices);
                    idealSlotsAllEdges(:,symNumbers,:,e) = tmpGrid;
                end
            end
        end
    end

end
