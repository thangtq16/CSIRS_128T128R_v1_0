% getNumPorts Calculate the number of ports per BWP required by a component
% carrier configuration cfgObj
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%

%   Copyright 2021-2024 The MathWorks, Inc.

%#codegen

function [numPorts,numPrePrecodingPorts] = getNumPorts(cfgObj)

    isDownlink = isa(cfgObj,'nrDLCarrierConfig');

    if isDownlink
        pxsch = cfgObj.PDSCH;
        pxcch = cfgObj.PDCCH;
        xrs   = cfgObj.CSIRS;
    else % Uplink
        pxsch = cfgObj.PUSCH;
        pxcch = cfgObj.PUCCH;
        xrs   = cfgObj.SRS;
    end

    % Find channel/signal enable/disable status
    enablePXSCH = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(pxsch,'Enable','double');
    enablePXCCH = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(pxcch,'Enable','double');
    enableXRS = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(xrs,'Enable','double');

    % Find number of ports
    npPXSCH = enablePXSCH .* ...
        nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(pxsch,'NumColumns','double');
    npPXCCH = enablePXCCH .* ...
        nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(pxcch,'NumColumns','double');
    npXRS = enableXRS .* ...
        nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(xrs,'NumColumns','double');

    % Find BWP ID
    bwpIDPXSCH = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(pxsch,'BandwidthPartID','double');
    bwpIDPXCCH = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(pxcch,'BandwidthPartID','double');
    bwpIDXRS = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(xrs,'BandwidthPartID','double');

    % Find number of ports per BWP and number of pre-precoding ports per
    % BWP
    bwps = cfgObj.BandwidthParts;
    if isempty(bwps)
        % No BWP

        numPorts = 1;
        numPrePrecodingPorts = 1;

    else
        % Loop over all BWPs

        numPorts = ones(1,numel(bwps));
        numPrePrecodingPorts = ones(1,numel(bwps));

        if isDownlink
            nlPXSCH = enablePXSCH .* ...
                nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(pxsch,'NumLayers','double');
        else
            nSRSPorts = enableXRS .* ...
                nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(xrs,'NumSRSPorts','double');
        end

        for bwpIdx = 1:numel(bwps)

            % Find BWP ID
            bwpID = bwps{bwpIdx}.BandwidthPartID;

            % Find number of ports, i.e., number of columns required for
            % output waveform

            % Max number of ports required by PXSCH
            maxNPPXSCH = max([npPXSCH(bwpIDPXSCH==bwpID) 1]);

            % Max number of ports required by PXCCH
            maxNPPXCCH = max([npPXCCH(bwpIDPXCCH==bwpID) 1]);

            % Max number of ports required by XRS
            maxNPXRS = max([npXRS(bwpIDXRS==bwpID) 1]);

            % Number of ports required by this BWP
            numPorts(bwpIdx) = max([maxNPPXSCH maxNPPXCCH maxNPXRS 1]);

            %--------------------------------------------------------------
 
            % Find number of ports before precoding and antenna mapping,
            % this will be used for conflict detection

            % Max number of pre-precoding ports required by XRS
            if isDownlink

                % Max number of pre-precoding ports required by PDSCH
                % Find number of PXSCH layers
                maxNLPXSCH = max([nlPXSCH(bwpIDPXSCH==bwpID) 1]);

                % Max number of pre-precoding ports required by CSIRS
                maxNXRSPorts = 1;
                for csirsIdx = 1:numel(xrs)
                    if xrs{csirsIdx}.BandwidthPartID==bwpID && xrs{csirsIdx}.Enable
                        maxNXRSPorts = max([maxNXRSPorts xrs{csirsIdx}.NumCSIRSPorts]);
                    end
                end

            else

                % Max number of pre-precoding ports required by PUSCH
                maxNLPXSCH = 1;
                for puschIdx = 1:numel(pxsch)
                    if pxsch{puschIdx}.BandwidthPartID==bwpID && pxsch{puschIdx}.Enable
                        if strcmp(pxsch{puschIdx}.TransmissionScheme,'codebook')
                            nl = pxsch{puschIdx}.NumAntennaPorts;
                        else
                            nl = pxsch{puschIdx}.NumLayers;
                        end
                        maxNLPXSCH = max(maxNLPXSCH,nl);
                    end
                end

                % Max number of pre-precoding ports required by CSIRS
                maxNXRSPorts = max([nSRSPorts(bwpIDXRS==bwpID) 1]);

            end

            % Number of pre-precoding ports required by this BWP
            numPrePrecodingPorts(bwpIdx) = max([maxNLPXSCH maxNXRSPorts 1]);

        end
    end

end

