function csirsInd = getCSIRSIndicesForCSI(carrier,csirs)
% CSIRSIND = getCSIRSIndicesForCSI(CARRIER,CSIRS) calculates Non-
% zero-power(NZP) CSI-RS indices CSIRSIND, for the given CARRIER and CSIRS
% configuration. Zero-power (ZP) CSI-RS resources are ignored, as they are
% not used for CSI estimation.

%   Copyright 2024 The MathWorks, Inc.

    if ~iscell(csirs.CSIRSType)
        csirs.CSIRSType = {csirs.CSIRSType};
    end
    if ~iscell(csirs.CDMType)        
        csirs.CDMType = {csirs.CDMType};
    end
    numZPCSIRSRes = sum(strcmpi(csirs.CSIRSType,'zp'));
    tempInd = nrCSIRSIndices(carrier,csirs,"IndexStyle","subscript","OutputResourceFormat","cell");
    tempInd = tempInd(numZPCSIRSRes+1:end)'; % NZP-CSI-RS indices
    % Extract the NZP-CSI-RS indices corresponding to first port
    for nzpResIdx = 1:numel(tempInd)
        nzpInd = tempInd{nzpResIdx};
        tempInd{nzpResIdx} = nzpInd(nzpInd(:,3) == 1,:);
    end
    % Extract the indices corresponding to the lowest RE of each CSI-RS CDM
    % group. This improves the computational speed by limiting the number
    % of CSI-RS REs
    cdmType = csirs.CDMType;
    if ~strcmpi(cdmType{1},'noCDM')
        for resIdx = 1:numel(tempInd)
            totIndices = size(tempInd{resIdx},1);
            if strcmpi(cdmType{1},'FD-CDM2')
                indicesPerSym = totIndices;
            elseif strcmpi(cdmType{1},'CDM4')
                indicesPerSym = totIndices/2;
            elseif strcmpi(cdmType{1},'CDM8')
                indicesPerSym = totIndices/4;
            end
            tempIndInOneSymbol = tempInd{resIdx}(1:indicesPerSym,:);
            tempInd{resIdx} = tempIndInOneSymbol(1:2:end,:);
        end
    end
    csirsInd = zeros(0,3);
    if ~isempty(tempInd)
        csirsInd = cell2mat(tempInd);
    end
end