function [CSIReport, CSIInfo] = myCSIReport(carrier, csirs, reportConfig, dmrsConfig, H, nVar)
%myCSIReport  CSI report (RI/PMI/CQI) via nr5g internal functions.
%   Replicates nrCSIReportCSIRS logic without the matlab.internal.feature gate.
%   Requires nr5g.internal.* on the MATLAB path (local 5g/ copy).

    % Validate inputs and get CSI-RS indices
    [reportConfig, csirsInd] = nr5g.internal.validateCSIInputs( ...
        carrier, csirs, reportConfig, dmrsConfig, H, nVar);

    % Subband partitioning info
    PMISubbandInfo = nr5g.internal.getPMISubbandInfo(carrier, reportConfig);

    Pcsirs  = size(H, 4);
    nRxAnts = size(H, 3);

    % Max rank per codebook type
    if strcmpi(reportConfig.CodebookType, 'Type1SinglePanel')
        maxRank = min([nRxAnts, Pcsirs, 8]);
    elseif strcmpi(reportConfig.CodebookType, 'Type2') || ...
           (strcmpi(reportConfig.CodebookType, 'eType2') && ...
            any(reportConfig.ParameterCombination == [7 8]))
        maxRank = min(nRxAnts, 2);
    else
        maxRank = min(nRxAnts, 4);
    end

    % Apply RI restriction
    if ~isempty(reportConfig.RIRestriction)
        validRanks = intersect(find(reportConfig.RIRestriction), 1:maxRank);
    else
        validRanks = 1:maxRank;
    end

    % Initialize outputs (NaN / empty)
    CSIReport = struct('RI', NaN, 'PMISet', [], 'CQI', NaN);
    CSIInfo   = struct('W', [], 'SINRPerSubband', [], 'EffectiveSINR', []);

    if isempty(validRanks) || isempty(csirsInd)
        return;
    end

    % CQI table spectral efficiency array
    cqiTableClass = nrCQITables;
    tableMap = {'Table1','Table2','Table3','Table4'};
    tName = tableMap{strcmpi(reportConfig.CQITable, tableMap)};
    SpecEffArray = cqiTableClass.(['CQI' tName]).SpectralEfficiency;

    % Loop over candidate ranks, compute CQI/PMI
    % Use cell arrays to avoid struct-array field-mismatch errors
    cqi     = cell(max(validRanks), 1);
    pmi     = cell(max(validRanks), 1);
    cqiInfo = cell(max(validRanks), 1);
    pmiInfo = cell(max(validRanks), 1);

    efficiency = NaN(max(validRanks), 1);
    for rank = validRanks
        [cqi{rank}, pmi{rank}, cqiInfo{rank}, pmiInfo{rank}] = ...
            nr5g.internal.nrCQIReport(carrier, csirs, reportConfig, dmrsConfig, rank, H, nVar);

        cqiWideband = cqi{rank}(1, :);
        if all(cqiWideband ~= 0) && ~any(isnan(cqiWideband))
            blerWideband = cqiInfo{rank}.TransportBLER(1, :);
            ncw      = numel(cqiWideband);
            cwLayers = floor((rank + (0:ncw-1)) / ncw);
            SpecEffValue = SpecEffArray(cqiWideband + 1);
            efficiency(rank) = cwLayers .* (1 - blerWideband) * SpecEffValue;
        else
            efficiency(rank) = 0;
        end
    end

    % Select rank that maximises spectral efficiency
    [maxEff, maxEffIdx] = max(efficiency);
    if ~isnan(maxEff)
        CSIReport.RI     = maxEffIdx;
        CSIReport.PMISet = pmi{maxEffIdx};
        CSIReport.CQI    = cqi{maxEffIdx};
        CSIInfo.W              = pmiInfo{maxEffIdx}.W;
        CSIInfo.SINRPerSubband = cqiInfo{maxEffIdx}.SINRPerSubbandPerCW;
        CSIInfo.EffectiveSINR  = cqiInfo{maxEffIdx}.EffectiveSINR;
    end
end
