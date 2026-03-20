function [refGridAllRS,locationMapAllSlots,rsRefIndex] = ...
    getI2ReferenceSignalGrid(carrier,channel,expSlots,reservedRE,inDataType)
%getI2ReferenceSignalGrid Provides the i2(v) ideal signal which contains
%only reference signals and all other locations are set to 0.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   The second ideal signal i2(v) is constructed by the measuring equipment
%   according to the relevant TX specifications, using the following
%   parameters for FR1 and FR2:
%   - nominal demodulation reference signal and nominal PT-RS if present
%   (all other modulation symbols are set to 0 V),
%   - nominal carrier frequency,
%   - nominal amplitude and phase for each applicable subcarrier,
%   - nominal timing.

% Copyright 2024 The MathWorks, Inc.

%#codegen

    % Initialize grid dimensions and arrays
    numSCs = double(carrier.NSizeGrid)*12;
    nSymbSlot = carrier.SymbolsPerSlot;
    nSlots = numel(expSlots);
    nSym = nSymbSlot*nSlots;
    nLayers = 1;
    if isprop(channel,"NumLayers")
        nLayers = channel.NumLayers;
    end
    tmpComplex = cast(1i,inDataType);
    gridDimsOneSlot = [numSCs nSymbSlot nLayers];
    gridDimsAllSym = [numSCs nSym nLayers];
    locationMapAllSlots = zeros(gridDimsAllSym);
    refGridAllRS = zeros(gridDimsAllSym,like=tmpComplex);
    rsRefIndex = 2;
    transformPrecoding = isprop(channel,"TransformPrecoding") && channel.TransformPrecoding;

    % Populate the grid with reference signals and their location pointers
    ptrsIndices = zeros(0,1,"uint32");
    ptrsSymbols = zeros(0,1,like=tmpComplex);
    for slotIdx = 1:nSlots
        % Get current slot number and update the carrier/cell settings
        nslot = expSlots(slotIdx);
        if nslot < 0
            continue
        end
        carrier.NSlot = nslot;

        switch class(channel)
            case "nrPDSCHConfig"
                if ~isempty(reservedRE)
                    channel.ReservedRE = reservedRE{slotIdx};
                end
                channelIndices = nrPDSCHIndices(carrier,channel);
                dmrsIndices = nrPDSCHDMRSIndices(carrier,channel);
                ptrsIndices = nrPDSCHPTRSIndices(carrier,channel);
                dmrsSymbols = nrPDSCHDMRS(carrier,channel,OutputDataType=inDataType);
                ptrsSymbols = nrPDSCHPTRS(carrier,channel,OutputDataType=inDataType);
            case "nrPUSCHConfig"
                channel.TransmissionScheme = "nonCodeBook";
                [channelIndices,~,ptrsIndices] = nrPUSCHIndices(carrier,channel);
                dmrsIndices = nrPUSCHDMRSIndices(carrier,channel);
                dmrsSymbols = nrPUSCHDMRS(carrier,channel,OutputDataType=inDataType);
                ptrsSymbols = nrPUSCHPTRS(carrier,channel,OutputDataType=inDataType);
                if transformPrecoding
                    % Scale PT-RS symbols based on modulation scheme
                    if iscell(channel.Modulation)
                        modulation = channel.Modulation{1};
                    else
                        modulation = channel.Modulation;
                    end
                    betaPTRS = nr5g.internal.pusch.ptrsPowerFactorDFTsOFDM(modulation);
                    ptrsSymbols = ptrsSymbols.*betaPTRS;
                end
            otherwise % nrPDCCHConfig
                [channelIndices,dmrsSymbols,dmrsIndices] = ...
                    nrPDCCHResources(carrier,channel,OutputDataType=inDataType);
        end

        % Map the reference signal to a resource grid of one slot and map
        % the one slot grid to the grid containing all slots
        tmpGrid = zeros(gridDimsOneSlot,like=tmpComplex);
        tmpGrid(dmrsIndices) = dmrsSymbols;
        symbolNumbers = (((slotIdx-1)*nSymbSlot)+1):(slotIdx*nSymbSlot);
        tmpGrid(ptrsIndices) = ptrsSymbols;
        refGridAllRS(:,symbolNumbers,:) = tmpGrid;
        % Map the channel, DM-RS, and PT-RS in the notional grid with 1, 2,
        % and 3, respectively. Map PT-RS at the last, so that in case of
        % transform precoding, this grid will represent the transform
        % de-precoded indices.
        if any(dmrsIndices)
            % Only if there are DM-RS indices, consider slot as valid
            slotGrid = zeros(gridDimsOneSlot);
            slotGrid(channelIndices) = 1;
            slotGrid(dmrsIndices) = rsRefIndex;
            slotGrid(ptrsIndices) = 3;
            locationMapAllSlots(:,symbolNumbers,:) = slotGrid;
        end
    end

end
