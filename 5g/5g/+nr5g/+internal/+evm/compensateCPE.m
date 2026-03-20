function eqGrid = compensateCPE(channel,eqGrid,refGridAllRS,notionalGridAllSlots,expSlots,cp)
%compensateCPE Performs CPE compensation using PT-RS
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

% Copyright 2024-2025 The MathWorks, Inc.

%#codegen

    % Perform CPE compensation, only when PT-RS is present
    if any(notionalGridAllSlots(:) == 3)
        % Initialize variables
        nSymbSlot = 12;
        if strcmpi(cp,"normal")
            nSymbSlot = 14;
        end
        nLayers = 1;
        if isprop(channel,"NumLayers")
            nLayers = channel.NumLayers;
        end
        nPTRSPorts = 1;
        if isprop(channel,"PTRS") && ~isempty(channel.PTRS.PTRSPortSet)
            nPTRSPorts = numel(channel.PTRS.PTRSPortSet);
        end
        transformPrecoding = isprop(channel,"TransformPrecoding") && channel.TransformPrecoding;
        if transformPrecoding
            nPTRSPorts = nLayers;
        end
        numSC = size(eqGrid,1);
        nSlots = numel(expSlots);
        % Loop over each edge and each slot to compensate CPE
        for edgeIdx = 1:size(eqGrid,4)
            for slotIdx = 1:nSlots
                if expSlots(slotIdx) < 0
                    continue
                end
                symNumbers = ((slotIdx-1)*nSymbSlot+1):(slotIdx*nSymbSlot);
                eqGridToProcess = eqGrid(:,symNumbers,:,edgeIdx);
                notionalGridToUse = notionalGridAllSlots(:,symNumbers,:);
                resGridToUse = refGridAllRS(:,symNumbers,:);
                ptrsIndices = reshape(find(notionalGridToUse == 3),[],nPTRSPorts);
                ptrsSymbols = resGridToUse(ptrsIndices);
                % When a DC subcarrier contains PT-RS and is excluded,
                % it is possible that the PT-RS symbols at that
                % subcarrier location in the reference grid are marked
                % zeros. Exclude those indices in channel estimation.
                nonZeroPTRSSymbolsLogicalArray = ptrsSymbols ~= 0;
                nonZeroPTRSSymbols = ptrsSymbols(nonZeroPTRSSymbolsLogicalArray);
                nonZeroPTRSIndices = ptrsIndices(nonZeroPTRSSymbolsLogicalArray);
                zeroPTRSIndinces = ptrsIndices(~nonZeroPTRSSymbolsLogicalArray);
                if ~isempty(ptrsIndices)
                    if transformPrecoding
                        logicalMatrix = (notionalGridToUse == 1)  ...
                            | (notionalGridToUse == 3);
                        channelIndices = reshape(find(logicalMatrix),[],nLayers); % Channel indices are same across layers
                        channelSymbols = eqGridToProcess(channelIndices);
                        % Transform deprecode the grid
                        symCol = find(any(sum(logicalMatrix,3)));
                        scRow = find(any(logicalMatrix(:,symCol(1)),2));
                        numPRB = numel(unique(floor((scRow-1)/12)));
                        deprecode = nrTransformDeprecode(channelSymbols,numPRB);
                        % Map the deprecoded symbols to tempGrid and
                        % reference PT-RS symbols to ptrsGrid
                        [tempGrid,ptrsGrid] = deal(zeros([numSC nSymbSlot nLayers],like=eqGrid));
                        tempGrid(channelIndices) = deprecode;
                        ptrsGrid(ptrsIndices) = ptrsSymbols;
                        % Get the channel estimate at the PT-RS locations
                        % to compute the common phase error. Since, angle
                        % is not impacted by normalization, ignore the
                        % reference signal normalization.
                        H = tempGrid.*conj(ptrsGrid);
                        H(H==0) = nan;
                        cpe = angle(sum(H,[1 3 4],"omitmissing"));
                        ptrsSymIndices = ~isnan(cpe);
                        if nnz(ptrsSymIndices) > 1
                            % When there are at least two PT-RS OFDM
                            % symbols, compute CPE at all the missing OFDM
                            % symbols locations using interpolation.
                            cpe = interp1(find(ptrsSymIndices),cpe(ptrsSymIndices),1:length(cpe),"linear","extrap");
                        end
                        % Update the tempGrid with equalized symbols. This
                        % overrides the deprecoded symbols with symbols
                        % prior to transform deprecoding.
                        tempGrid(channelIndices) = channelSymbols;
                    else
                        % Assign channelIndices to have variable definition
                        % in all execution paths
                        channelIndices = reshape(find(notionalGridToUse == 1),[],nLayers);
                        tempGrid = eqGridToProcess;
                        % Perform channel estimation to get the common phase
                        cpe = nrChannelEstimate(tempGrid,nonZeroPTRSIndices,nonZeroPTRSSymbols,CyclicPrefix=cp);
                        % Sum estimates across subcarriers, receive antennas,
                        % and layers. Then, get the CPE by taking the angle of
                        % the resultant sum
                        cpe = angle(sum(cpe,[1 3 4]));
                    end
                    % Correct CPE in each OFDM symbol within the range of
                    % reference PT-RS OFDM symbols
                    notionalGridToUse(zeroPTRSIndinces) = 0;
                    ptrsSymbolSet = find(any(sum(notionalGridToUse==3,3)));
                    if numel(ptrsSymbolSet)
                        symLoc = ptrsSymbolSet(1):ptrsSymbolSet(end);
                        tempGrid(:,symLoc,:) = tempGrid(:,symLoc,:).*exp(-1i*cpe(symLoc));
                    end
                    % Update the grid with compensated symbols
                    if transformPrecoding
                        eqGridToProcess(channelIndices) = tempGrid(channelIndices);
                    else
                        eqGridToProcess = tempGrid;
                    end
                    eqGrid(:,symNumbers,:,edgeIdx) = eqGridToProcess;
                end
            end
        end
    end

end
