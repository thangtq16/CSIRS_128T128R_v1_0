function [waveform, info] = nrWaveformGenerator(cfgObj)
%nrWaveformGenerator Generate 5G New Radio (NR) baseband waveform
%   [WAVE, INFO] = nrWaveformGenerator(CFG) generates the 5G NR baseband
%   waveform WAVE specified by the configuration object CFG. When CFG is an
%   <a href="matlab: help('nrDLCarrierConfig')"
%   >nrDLCarrierConfig</a> object, WAVE is a 5G Downlink waveform. When CFG is an
%   <a href="matlab:help('nrULCarrierConfig')"
%   >nrULCarrierConfig</a> object, WAVE is a 5G Uplink waveform. The input object
%   describes multiple configurations of a 5G waveform, such as the SCS
%   carriers, bandwidth parts, SS burst (downlink only), CORESET (downlink
%   only), search spaces (downlink only), PDCCH or PUCCH, PDSCH or PUSCH
%   and associated DM-RS and PT-RS, and CSI-RS or SRS. Additionally, INFO
%   contains metadata specific to the contained bandwidth parts and the
%   contained PDCCH or PUCCH, PDSCH or PUSCH, and SRS (uplink only). For
%   more details on the info output, see <a href="matlab:doc('nrWaveformGenerator')"
%   >nrWaveformGenerator</a> in the documentation.
%
%   nrWaveformGenerator (without input or output arguments) launches the
%   5G Waveform Generator app for graphical configuration, generation,
%   visualization, and transmission of 5G waveforms. For more details, see
%   <a href="matlab:doc('5G Waveform Generator')"</a> in the documentation.
%
%   Example 1: 
%   % Generate a single-numerology (15 kHz), single-user 5G downlink waveform 
%
%   cfg = nrDLCarrierConfig('FrequencyRange', 'FR1', 'ChannelBandwidth', 40, 'NumSubframes', 20);
%   cfg.SCSCarriers{1}.NSizeGrid = 100;    % default SCS is 15 kHz
%   cfg.BandwidthParts{1}.NStartBWP = cfg.SCSCarriers{1}.NStartGrid + 10;
%   cfg.SSBurst.BlockPattern = 'Case A'; % 15 kHz
%   cfg.CORESET{1}.Duration = 3;
%   cfg.CORESET{1}.FrequencyResources = [1 1 1 1];
%   cfg.SearchSpaces{1}.NumCandidates = [8 4 0 0 0];
%   cfg.PDCCH{1}.AggregationLevel = 2;
%   cfg.PDCCH{1}.AllocatedCandidate = 4;
%   cfg.PDSCH{1}.Modulation = '16QAM';
%   cfg.PDSCH{1}.TargetCodeRate = 658/1024;
%   cfg.PDSCH{1}.DMRS.DMRSTypeAPosition = 3;
%   cfg.PDSCH{1}.EnablePTRS = true;
%   cfg.PDSCH{1}.PTRS.TimeDensity = 2;
%   cfg.CSIRS{1}.RowNumber = 4;
%   cfg.CSIRS{1}.RBOffset = 10;
%
%   dlWave = nrWaveformGenerator(cfg);
%
%   Example 2: 
%   % Create a configuration for a mixed-numerology, multiuser 5G downlink 
%   % waveform; then generate the waveform.
%
%   % SCS Carriers:
%   scscarriers = {nrSCSCarrierConfig('SubcarrierSpacing', 15, 'NStartGrid', 10, 'NSizeGrid', 100), ...
%                  nrSCSCarrierConfig('SubcarrierSpacing', 30, 'NStartGrid', 0, 'NSizeGrid', 70)};
%   % Bandwidth parts:
%   bwp = {nrWavegenBWPConfig('BandwidthPartID', 0, 'SubcarrierSpacing', 15, 'NStartBWP', 10, 'NSizeBWP', 80), ...
%          nrWavegenBWPConfig('BandwidthPartID', 1, 'SubcarrierSpacing', 30, 'NStartBWP', 0, 'NSizeBWP', 60)};
%   % SS burst:
%   ssburst = nrWavegenSSBurstConfig('BlockPattern', 'Case A'); % 15 kHz
%   % Control (CORESET/Search space/PDCCH):
%   coreset = {nrCORESETConfig('CORESETID', 1, 'FrequencyResources', [1 1 1 1 1 0 0 0 0 0 1], 'Duration', 3), ...
%              nrCORESETConfig('CORESETID', 2, 'FrequencyResources', [0 0 0 0 0 0 0 0 1 1])};
%   ss = {nrSearchSpaceConfig('SearchSpaceID', 1, 'CORESETID', 1, 'StartSymbolWithinSlot', 4), ...
%         nrSearchSpaceConfig('SearchSpaceID', 2, 'CORESETID', 2, 'NumCandidates', [8 8 4 0 0])};
%   pdcch = {nrWavegenPDCCHConfig('SearchSpaceID', 1, 'BandwidthPartID', 0, 'RNTI', 1, 'DMRSScramblingID', 1), ...
%            nrWavegenPDCCHConfig('SearchSpaceID', 2, 'BandwidthPartID', 1, 'RNTI', 2, 'DMRSScramblingID', 2, 'AggregationLevel', 4)};
%   % PDSCH:
%   pdsch = {nrWavegenPDSCHConfig('BandwidthPartID', 0, 'Modulation', '16QAM', 'RNTI', 1, 'NID', 1), ...
%            nrWavegenPDSCHConfig('BandwidthPartID', 1, 'Modulation', 'QPSK', 'RNTI', 2, 'NID', 2, 'PRBSet', 50:59)};
%   % CSI-RS:
%   csirs = {nrWavegenCSIRSConfig('BandwidthPartID', 0, 'RowNumber', 2, 'RBOffset', 10), ... 
%           nrWavegenCSIRSConfig('BandwidthPartID', 1, 'Density', 'one', 'RowNumber', 4, 'NumRB', 5)};
%
%   % Combine everything together:
%   cfg = nrDLCarrierConfig('FrequencyRange', 'FR1', 'ChannelBandwidth', 40, 'NumSubframes', 20);
%   cfg.SCSCarriers = scscarriers;
%   cfg.BandwidthParts = bwp;
%   cfg.SSBurst = ssburst;
%   cfg.CORESET = coreset;
%   cfg.SearchSpaces = ss;
%   cfg.PDCCH = pdcch;
%   cfg.PDSCH = pdsch;
%   cfg.CSIRS = csirs;
%
%   % Generate waveform:
%   waveform = nrWaveformGenerator(cfg);
%
%   Example 3: 
%   % Create a configuration for a mixed-numerology, multiuser 5G uplink 
%   % waveform; then generate the waveform.
%
%   % SCS Carriers:
%   scscarriers = {nrSCSCarrierConfig('SubcarrierSpacing', 15, 'NStartGrid', 10, 'NSizeGrid', 100), ...
%                  nrSCSCarrierConfig('SubcarrierSpacing', 30, 'NStartGrid', 0, 'NSizeGrid', 70)};
%   % Bandwidth parts:
%   bwp = {nrWavegenBWPConfig('BandwidthPartID', 0, 'SubcarrierSpacing', 15, 'NStartBWP', 30, 'NSizeBWP', 80), ...
%          nrWavegenBWPConfig('BandwidthPartID', 1, 'SubcarrierSpacing', 30, 'NStartBWP', 0, 'NSizeBWP', 60)};
%   % PUSCH:
%   pusch = {nrWavegenPUSCHConfig('BandwidthPartID', 0, 'Modulation', '16QAM', 'RNTI', 1, 'NID', 1, 'SymbolAllocation', [0 13]), ...
%            nrWavegenPUSCHConfig('BandwidthPartID', 1, 'Modulation', 'QPSK', 'RNTI', 2, 'NID', 2, 'PRBSet', 50:59, 'SymbolAllocation', [0 10])};
%   % PUCCH:
%   pucch = {nrWavegenPUCCH0Config('BandwidthPartID', 1, 'SlotAllocation', 0:9, 'PRBSet', 2, 'DataSourceUCI', 'PN9')};
%   % SRS:
%   srs = {nrWavegenSRSConfig('BandwidthPartID', 0, 'NumSRSPorts', 2), ... 
%          nrWavegenSRSConfig('BandwidthPartID', 1, 'FrequencyStart', 4)};
%
%   % Combine everything together:
%   cfg = nrULCarrierConfig('FrequencyRange', 'FR1', 'ChannelBandwidth', 40, 'NumSubframes', 20);
%   cfg.SCSCarriers = scscarriers;
%   cfg.BandwidthParts = bwp;
%   cfg.PUSCH = pusch;
%   cfg.PUCCH = pucch;
%   cfg.SRS = srs;
%
%   % Generate waveform:
%   waveform = nrWaveformGenerator(cfg);
%
%   See also nrDLCarrierConfig, nrULCarrierConfig.

%   Copyright 2020-2025 The MathWorks, Inc.

%#codegen

    %% Input processing
    narginchk(0, 1);
    if nargin == 0
        nargoutchk(0, 0);
        
        % Launch the Wireless Waveform Generator App (with a 5G Downlink default)
        wirelessWaveformGenerator('Downlink');
        return;
    else
        validateattributes(cfgObj, {'nrDLCarrierConfig','nrULCarrierConfig'}, {'scalar'}, 'nrWaveformGenerator', 'input');
    end
    
    % Cross-object checks performed here:
    validateConfig(cfgObj);
    
    % Define the link direction
    isDownlink = isa(cfgObj,'nrDLCarrierConfig');
    
    %% Setup
    [cfgObj, extWaveInfo, sr, numPorts, waveform] = setup(cfgObj);
    
    %% Reference Signals: CSI-RS (Downlink), SRS (Uplink)

    % Data Container Formats
    % reservedREs container
    % This is a list of common resources (RE) that are already reserved for other uses (particularly reference signals) in a BWP, and which may impact on each PDSCH sequence
    %
    % Its format is a cell matrix (numPxSCH by maxNumSlotsInWaveform) where each row is the common RE already used across the slots of the waveform, 
    % which may impact the associated PxSCH channel sequence (connected by position in PDSCH/PUSCH definition cell array). Each row is therefore
    % the used resources in the BWP associated with that channel instance sequence, for all slots in that BWP of the entire waveform. 
    % Each cell contains a single vector of linearized RE indices indicating the used RE in that slot. The position of the cell in the row is the slot number.

    if isDownlink
        % Process and generate CSI-RS
        [ResourceElementGridsXRS, reservedREs, extWaveInfo] = processAndGenerateCSIRS(cfgObj, numPorts,extWaveInfo);
    else % Uplink
        % Process and generate SRS
        [ResourceElementGridsXRS, reservedREs, extWaveInfo] = processAndGenerateSRS(cfgObj, numPorts, extWaveInfo);
    end

    %% Control: CORESET & PDCCH (Downlink), PUCCH (Uplink)

    % Data Container Formats
    % 
    % controlReservedPRB container 
    % This is a list of (control) resources that each PDSCH sequence might have to rate match around
    % Its format is a cell array, where each cell is associated with the respective PDSCH configuration (connected by position in PDSCH definition cell array),
    % and each individual cell is also a cell array containing each of the PDCCH transmission instance resources (PRB/absolute symbols), 
    % captured as a 'reserved resources' objects. 
    % The processing of each PDSCH will look up all the associated reservation entries and create a union for each individual PDSCH instance that might be impacted

    maxNumPorts = max(numPorts);
    if isDownlink
        % Process and generate PDCCH
        [ResourceElementGridsPXCCH, controlReservedPRB, reservedREs, extWaveInfo] = processAndGeneratePDCCH(cfgObj, extWaveInfo, maxNumPorts, reservedREs);
    else % Uplink
        % Process and generate PUCCH
        [ResourceElementGridsPXCCH, controlReservedPRB, extWaveInfo] = processAndGeneratePUCCH(cfgObj, extWaveInfo, maxNumPorts);
    end

    %% Shared Channel: PDSCH or PUSCH
    [ResourceElementGridsPXSCH, extWaveInfo] = processAndGeneratePXSCH(cfgObj, extWaveInfo, controlReservedPRB, reservedREs, maxNumPorts, isDownlink);
    
    %% Error for conflicts (RE overlaps) among channels and signals
    conflicts = nr5g.internal.wavegen.detectConflict(cfgObj,extWaveInfo);
    nr5g.internal.wavegen.reportConflicts(conflicts);
    
    %% OFDM modulation and combination of BWP parts
    [waveform, gridset] = ofdmModAndCombineBWP(cfgObj, ResourceElementGridsXRS, ResourceElementGridsPXCCH, ResourceElementGridsPXSCH, sr, waveform);
    
    %% SS Burst addition to waveform
    if isDownlink
        % SSBurst added for downlink only
        ssburst =  nr5g.internal.wavegen.mapSSBObj2Struct(cfgObj.SSBurst, cfgObj.SCSCarriers);
        waveform = addSSBurst(cfgObj, ssburst, sr, waveform);
    end
    
    %% Output handling
    info.ResourceGrids = gridset;
    info.WaveformResources = extWaveInfo;
    
end

%% Setup
function [cfgObj, waveinfo, maxsr, numPorts, waveform] = setup(cfgObj)

    % Define the PXSCH instrumentation info structure
    initEmpty = zeros(0,1);
    coder.varsize('initEmpty',[inf,inf],[1,1]);
    codewordInit = {{int8(initEmpty), int8(initEmpty)}}; % cover the biggest possible container
    coder.varsize('codewordInit{1}', [1,2], [0,1]);
    trblkInit = {{initEmpty initEmpty}};
    unitStructPXSCH = struct('NSlot', initEmpty, 'TransportBlockSize', [0 0], 'TransportBlock', trblkInit, ...
        'RV', 0, 'Codeword', codewordInit, 'G', 0, 'Gd', 0, ...
        'ChannelIndices', uint32(initEmpty), 'ChannelSymbols', complex(initEmpty), ...
        'DMRSIndices', uint32(initEmpty), 'DMRSSymbols', complex(initEmpty), 'DMRSSymbolSet', complex(initEmpty'), ...
        'PTRSIndices', uint32(initEmpty), 'PTRSSymbols', complex(initEmpty), 'PTRSSymbolSet', complex(initEmpty'));
    coder.varsize('unitStructPXSCH.NSlot', 'unitStructPXSCH.RV', 'unitStructPXSCH.G', 'unitStructPXSCH.Gd');
    
    outStructPXSCH = struct('Name', '', 'PrecodingMatrix', complex(initEmpty), 'CDMLengths', [0 0], 'Resources', unitStructPXSCH);
    coder.varsize('outStructPXSCH.Name', 'outStructPXSCH.Resources', 'outStructPXSCH.CDMLengths', 'outStructPXSCH.PrecodingMatrix');
    
    % Define the XRS instrumentation info structure
    unitStructXRS = struct('NSlot', initEmpty, 'SignalIndices', uint32(initEmpty), 'SignalSymbols', complex(initEmpty));
    
    outStructXRS = struct('Name', '', 'PrecodingMatrix', complex(initEmpty), 'Resources', unitStructXRS);
    coder.varsize('outStructXRS.Name', 'outStructXRS.Resources', 'outStructXRS.PrecodingMatrix');
    
    isDownlink = isa(cfgObj,'nrDLCarrierConfig');
    
    if isDownlink
        % Define the PDCCH instrumentation info structure
        unitStructPDCCH = struct('NSlot', initEmpty, 'DCIBits', initEmpty, 'Codeword', initEmpty, ...
            'G', 0, 'Gd', 0, 'ChannelIndices', uint32(initEmpty), 'ChannelSymbols', complex(initEmpty), ...
            'DMRSIndices', uint32(initEmpty), 'DMRSSymbols', complex(initEmpty));
        
        outStructPDCCH = struct('Name', '', 'PrecodingMatrix', complex(initEmpty), 'CDMLengths', [0 0], 'Resources', unitStructPDCCH);
        coder.varsize('outStructPDCCH.Name', 'outStructPDCCH.Resources', 'outStructPDCCH.PrecodingMatrix');

        % Define the waveform instrumentation info variable
        waveinfo = struct('PDCCH', repmat(outStructPDCCH,  1, numel(cfgObj.PDCCH)), ...
                          'PDSCH', repmat(outStructPXSCH,  1, numel(cfgObj.PDSCH)), ...
                          'CSIRS', repmat(outStructXRS,  1, numel(cfgObj.CSIRS)));
        
    else % Uplink
        % Define the PUCCH instrumentation info structure
        unitStructPUCCH = struct('NSlot', initEmpty, 'SRBit', int8(initEmpty), 'UCIBits', int8(initEmpty), 'UCI2Bits', int8(initEmpty), ...
            'Codeword', codewordInit, 'G', 0, 'Gd', 0, 'ChannelIndices', uint32(initEmpty), 'ChannelSymbols', complex(initEmpty), ...
            'DMRSIndices', uint32(initEmpty), 'DMRSSymbols', complex(initEmpty));
        
        outStructPUCCH = struct('Name', '', 'PrecodingMatrix', complex(initEmpty), 'Format', [], 'CDMLengths', [0 0], 'Resources', unitStructPUCCH);
        coder.varsize('outStructPUCCH.Name', 'outStructPUCCH.Format', 'outStructPUCCH.Resources', 'outStructPUCCH.PrecodingMatrix');
        
        % Define the waveform instrumentation info variable
        waveinfo = struct('PUSCH', repmat(outStructPXSCH,  1, numel(cfgObj.PUSCH)), ...
                          'PUCCH', repmat(outStructPUCCH,  1, numel(cfgObj.PUCCH)), ...
                          'SRS', repmat(outStructXRS,  1, numel(cfgObj.SRS)));
    end
    
    maxsr = nr5g.internal.wavegen.maxSampleRate(cfgObj);
    
    % Calculate the number of ports required by this configuration 
    numPorts = nr5g.internal.wavegen.getNumPorts(cfgObj);

    % Update BWP resource element grids based on XRS ports
    maxNumPorts = max(numPorts);
    if isDownlink
        % For DL, consider ports required by precoded SSB
        numPortsSSB = cfgObj.SSBurst.NumColumns;
        maxNumPorts = max([maxNumPorts numPortsSSB]);
    end
    numSamples = ceil(maxsr*1e-3*cfgObj.NumSubframes); % Waveform length in samples
    waveform = complex(zeros(numSamples,maxNumPorts));
end


%% CSIRS
function [ResourceElementGridsCSIRS, reservedREs, waveinfo] = processAndGenerateCSIRS(cfgObj, numPorts, waveinfo)

    carriers = cfgObj.SCSCarriers;
    bwps = cfgObj.BandwidthParts;
    csirs = cfgObj.CSIRS;
    pdsch = cfgObj.PDSCH;
    maxNumPorts =  max(numPorts);

    numPDSCH = max([1 numel(pdsch)]);
    numSubframes = cfgObj.NumSubframes;
    carrierscs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers, 'SubcarrierSpacing', 'double');
    maxNumSlots = numSubframes*(max(carrierscs)/15);
    reservedREs = cell(numPDSCH, maxNumSlots);
    for nch = 1:numPDSCH
        % Initialization needed for expansion at the last command of this
        % function, in codegen:
        for idx = 1:maxNumSlots
            tmpInit = uint32([]);
            coder.varsize('tmpInit')
            reservedREs{nch, idx} = tmpInit;
        end
    end

    ResourceElementGridsCSIRS = createREGrid(bwps, numSubframes, maxNumPorts);

    initEmpty = zeros(0,1);
    coder.varsize('initEmpty',[inf,inf],[1,1]);
    unitStruct = struct('NSlot', 0, 'SignalIndices', uint32(initEmpty), 'SignalSymbols', complex(initEmpty));

    % Unbundle CSI-RS specific parameter objects
    csirsInstanceIndex = 1;
    for nsig = 1:numel(csirs)
        % Only process configuration if enabled
        sig = csirs{nsig};
        if ~sig.Enable
            % Capture this CSIRS sequence's label for the resource info name
            waveinfo.CSIRS(nsig).Name = sig.Label;
            continue;
        end
        
        % Recreate CSI-RS specific configuration objects using the
        % parameters provided by CSIRS field of waveconfig
        csirsCfg = nr5g.internal.wavegen.getCSIRSObject(sig);

        bwpIdx = getBWPIdxByID(bwps, sig.BandwidthPartID);

        % Recreate carrier specific configuration objects using carrier
        % specific parameters
        carrier = carriers{nr5g.internal.wavegen.getCarrierIDByBWPID(carriers, bwps, sig.BandwidthPartID)};
        carrierCfg = nr5g.internal.wavegen.getCarrierCfgObject(carrier, cfgObj.NCellID, bwps{bwpIdx}.CyclicPrefix);

        % Get the number of CSI-RS resources configured
        if iscell(csirsCfg.CSIRSType)
            % Single or multiple resources
            numRes = numel(csirsCfg.CSIRSType);
        else
            % Single resource
            numRes = 1;
        end

        % Extract the power scalings of channel state information reference signals
        powerCSIRS = sig.Power;
        powerCSIRS = db2mag(powerCSIRS); % dB to magnitude conversion

        nrb = bwps{bwpIdx}.NSizeBWP;
        symbperslot = carrierCfg.SymbolsPerSlot;
        bwpGridSize = [12*nrb symbperslot max(sig.NumCSIRSPorts)];
        
        numSlots = carrierCfg.SlotsPerSubframe*numSubframes;
        initSlot = cfgObj.InitialNSubframe*carrierCfg.SlotsPerSubframe;

        % Storage for CSI-RS instance information
        datastore = repmat(unitStruct, numRes, numSlots);

        % Get the precoding and antenna mapping matrix, number of output
        % columns and max number of CSIRS ports
        Wpa = sig.Wpa;
        nColumns = size(Wpa,1);
        antGridSize = [12*nrb symbperslot nColumns];
        
        % Loop over initSlot to numSlots+initSlot-1
        for slotIdx = 0:numSlots-1
            carrierCfg.NSlot = slotIdx+initSlot;

            % Generate 1-based carrier oriented CSI-RS indices in
            % subscript form
            [csirsIndCell,~] = nrCSIRSIndices(carrierCfg,csirsCfg,'IndexStyle','subscript','OutputResourceFormat','cell');
            
            % Conflicts within the same CSI-RS config object
            for idx = 1:numRes
                if ~isempty(csirsIndCell{idx})
                    % The REs used by CSI-RS in one port should be reserved in
                    % other ports except for those in CDM. Check that the use
                    % of CSI-RS REs across multiple ports is consistent with
                    % its CDM configuration
                    cdmLengths = getCSIRSCDMLengths(csirsCfg,idx);
                    numPortCDMRatio = csirsCfg.NumCSIRSPorts(idx)/prod(cdmLengths);
                    numCSIRSReservedREAllPorts = size(unique(csirsIndCell{idx}(:,1:2),'rows'),1)*csirsCfg.NumCSIRSPorts(idx);
                    overlap = numCSIRSReservedREAllPorts/size(csirsIndCell{idx},1) ~= numPortCDMRatio;
                    coder.internal.errorIf(overlap,'nr5g:nrWaveformGenerator:CSIRSInvalidConfig', nsig);
                end
            end

            % Generate CSI-RS symbols
            sym = nrCSIRS(carrierCfg,csirsCfg,'OutputResourceFormat','cell');
            
            % Create an empty slot grid
            slotAntGrid = complex(zeros(antGridSize));

            % Limit the span of the CSI-RS to the BWP
            allCSIRSLinearInd = uint32([]);
            for idx = 1:numRes
                csirsSym = sym{idx}*powerCSIRS; % Apply power boosting
                
                % Change the orientation of CSI-RS indices to BWP
                csirsInd = csirsIndCell{idx};
                bwpRBOffset = bwps{bwpIdx}.NStartBWP - carrierCfg.NStartGrid;
                offsetSubc = csirsInd(:,1)-bwpRBOffset*12;
                bwpCSIRSInd = [offsetSubc csirsInd(:,2) csirsInd(:,3)];

                % Trim CSI-RS within BWP boundaries
                ind2rmv = bwpCSIRSInd(:, 1)<=0 | bwpCSIRSInd(:, 1)>bwpGridSize(1);
                bwpCSIRSInd(ind2rmv, :) = [];
                csirsSym(ind2rmv) = [];

                % Linearize BWP oriented CSI-RS indices
                % CSI-RS can now extend beyond the BWP
                csirsLinInd = uint32(sub2ind(bwpGridSize,bwpCSIRSInd(:,1),bwpCSIRSInd(:,2),bwpCSIRSInd(:,3)));

                % Collect the indices of all CSI-RS resources for RE reservation
                allCSIRSLinearInd = [allCSIRSLinearInd; csirsLinInd(:)];

                % Precoding and antenna mapping
                [csirsAntSym,csirsAntInd] = precodeAndMapAntenna(bwpGridSize,csirsSym,csirsLinInd,Wpa.');

                % Write the CSI-RS symbols in the slot grid
                slotAntGrid(csirsAntInd) = csirsAntSym;

                % Capture resource info for this CSIRS instance
                datastore(idx,slotIdx+1).NSlot = carrierCfg.NSlot;
                datastore(idx,slotIdx+1).SignalIndices = csirsLinInd;
                datastore(idx,slotIdx+1).SignalSymbols = csirsSym;
                
            end

            % Combine CSI-RS instance with the rest of the BWP grid
            ResourceElementGridsCSIRS{bwpIdx}(:,slotIdx*symbperslot+(1:symbperslot),1:nColumns) = ResourceElementGridsCSIRS{bwpIdx}(:,slotIdx*symbperslot+(1:symbperslot),1:nColumns) + slotAntGrid;
            
            % Determine RE reservation for CSI-RS
            if numel(pdsch) > 0
                % Perform the for loop and assign the values to reservedREs
                % only if there is at least one PDSCH instance
                for nch = 1:numPDSCH
                    dch = pdsch{nch};
                    if dch.Enable
                        if (dch.BandwidthPartID == bwps{bwpIdx}.BandwidthPartID)
                            % 0-based CSI-RS REs
                            reservedREs{nch, slotIdx+1} = [reservedREs{nch, slotIdx+1}; allCSIRSLinearInd - 1];   % Concatenate with other RE in the slot
                        end
                    end
                end
            end
            
        end

        % Capture all resources info for this CSIRS sequence
        for i = 1:numRes
            csirsResources.Name = sig.Label;
            csirsResources.PrecodingMatrix = complex(Wpa);
            csirsResources.Resources = datastore(i,:);
            waveinfo.CSIRS(csirsInstanceIndex) = csirsResources;
            csirsInstanceIndex = csirsInstanceIndex + 1;
        end
    end
end


%% SRS
function [ResourceElementGridsSRS, reservedREs, waveinfo] = processAndGenerateSRS(cfgObj, numPorts, waveinfo)

    carriers = cfgObj.SCSCarriers;
    bwps = cfgObj.BandwidthParts; 
    srs = cfgObj.SRS;
    pusch = cfgObj.PUSCH;
    maxNumPorts =  max(numPorts);
    
    numPUSCH = max([1 numel(pusch)]);
    numSubframes = cfgObj.NumSubframes;
    carrierscs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers, 'SubcarrierSpacing', 'double');
    maxNumSlots = numSubframes*(max(carrierscs)/15);
    % There are no reserved REs for SRS but reservedREs still needs to be
    % created and each cell populated for codegen
    reservedREs = cell(numPUSCH, maxNumSlots);
    for nch = 1:numPUSCH
        for idx = 1:maxNumSlots
            reservedREs{nch, idx} = uint32([]);
        end
    end
    
    ResourceElementGridsSRS = createREGrid(bwps, numSubframes, maxNumPorts);
    
    initEmpty = zeros(0,1);
    coder.varsize('initEmpty',[inf,inf],[1,1]);
    unitStruct = struct('NSlot', 0, 'SignalIndices', uint32(initEmpty), 'SignalSymbols', complex(initEmpty));
    
    % Unbundle SRS specific parameter objects
    for nsig = 1:numel(srs)
        % Only process configuration if enabled
        sig = srs{nsig};
        if ~sig.Enable || isempty(sig.SlotAllocation)
            % Capture this SRS sequence's label for the resource info name
            waveinfo.SRS(nsig).Name = sig.Label;
            continue;
        end

        % Recreate SRS specific configuration objects using the
        % parameters provided by SRS field of waveconfig
        srsCfg = nr5g.internal.wavegen.getSRSObject(sig);
        
        bwpIdx = getBWPIdxByID(bwps, sig.BandwidthPartID);
        
        % Recreate carrier specific configuration objects using carrier
        % specific parameters
        carrierCfg = nr5g.internal.wavegen.getCarrierCfgObject(bwps{bwpIdx}, cfgObj.NCellID);
        symbperslot = carrierCfg.SymbolsPerSlot;
        
        [SRSAllocatedSlots,initSlot] = nr5g.internal.wavegen.expandbyperiod(sig.SlotAllocation,sig.Period,numSubframes,bwps{bwpIdx}.SubcarrierSpacing,cfgObj.InitialNSubframe);
        
        % Expand the power to match the number of unique slots in a period
        [powerVec,uniqueSlotsInPeriod] = expandPower(sig.Power,sig.SlotAllocation,sig.Period);
        powerVec = db2mag(powerVec);

        % Storage for SRS instance information
        datastore = repmat(unitStruct,  1, numel(SRSAllocatedSlots));

        % Get the precoding and antenna mapping matrix
        Wpa = sig.Wpa;
        nColumns = size(Wpa,1);
        siz = [12*carrierCfg.NSizeGrid symbperslot sig.NumSRSPorts];
        
        % Loop over the allocated slots
        for s = 1:numel(SRSAllocatedSlots)
            % Create an empty slot grid to contain a single SRS instance
            slotAntGrid = nrResourceGrid(carrierCfg,nColumns);
            
            % Set slot number
            slotIdx = SRSAllocatedSlots(s);
            carrierCfg.NSlot = slotIdx;

            % Index of power vector
            powerInd = getPowerIndex(s,uniqueSlotsInPeriod);
            
            if isempty(sig.Period)
                srsIdx = 1;
            else
                srsIdx = find(mod(slotIdx,sig.Period) == sig.SlotAllocation, 1);
            end
            
            % Generate SRS indices and sequence
            srsInd = nrSRSIndices(carrierCfg,srsCfg{srsIdx(1)});
            srsSymbols = nrSRS(carrierCfg,srsCfg{srsIdx(1)})*powerVec(powerInd);

            % Precoding and antenna mapping
            [srsAntSym,srsAntInd] = precodeAndMapAntenna(siz,srsSymbols,srsInd,Wpa.');

            % Write SRS symbols into grid
            slotAntGrid(srsAntInd) = srsAntSym;
            
            % Combine SRS instance with the rest of the RE carrier grid
            ResourceElementGridsSRS{bwpIdx}(:,(slotIdx-initSlot)*symbperslot+(1:symbperslot),1:nColumns) = ResourceElementGridsSRS{bwpIdx}(:,(slotIdx-initSlot)*symbperslot+(1:symbperslot),1:nColumns) + slotAntGrid;
            
            % Capture resource info for this SRS instance
            datastore(s).NSlot = slotIdx;
            datastore(s).SignalIndices = srsInd;
            datastore(s).SignalSymbols = srsSymbols;
        end
        
        % Capture all resources info for this SRS sequence
        srsResources.Name = sig.Label;
        srsResources.PrecodingMatrix = complex(Wpa);
        srsResources.Resources = datastore;
        waveinfo.SRS(nsig) = srsResources;
    end
end
    
    
%% Control (CORESET and PDCCH)
function [ResourceElementGridsPDCCH, reservedPRB, reservedRE, waveinfo] = processAndGeneratePDCCH(cfgObj, waveinfo, maxNumPorts,reservedRE)

    bwps = cfgObj.BandwidthParts;
    coreset = cfgObj.CORESET;
    searchSpaces = cfgObj.SearchSpaces;
    pdcch = cfgObj.PDCCH;
    pdsch = cfgObj.PDSCH;
    
    % Create a cell array of empty RE grids for the defined set of BWP, across all the subframes 
    ResourceElementGridsPDCCH = createREGrid(bwps, cfgObj.NumSubframes, maxNumPorts);
    
    initEmpty = zeros(0,1);
    coder.varsize('initEmpty',[inf,inf],[1,1]);
    unitStruct = struct('NSlot', 0, 'DCIBits', initEmpty, 'Codeword', initEmpty, ...
        'G', 0, 'Gd', 0, 'ChannelIndices', uint32(initEmpty), 'ChannelSymbols', complex(initEmpty), ...
        'DMRSIndices', uint32(initEmpty), 'DMRSSymbols', complex(initEmpty));
    
    %% Process the set of PDCCH transmission sequences
    % Preallocate a cell array of nrPDSCHReservedConfig objects for each PDSCH instance
    reservedPRB = allocatePDSCHReservedConfigForControl(bwps,coreset,searchSpaces,pdcch,pdsch,cfgObj.NumSubframes,cfgObj.InitialNSubframe);
    numPDSCH = numel(pdsch);
    reservedCounter = ones(1,numPDSCH); % Counter to accumulate through the nrPDSCHReservedConfig per PDSCH
    
    for nch = 1:numel(pdcch)
        
        % Get a copy of the current PDCCH channel parameters
        ch = pdcch{nch};
        bwpIdx = getBWPIdxByID(bwps, ch.BandwidthPartID);
        
        % Only process the PDCCH configuration if enabled
        if ~ch.Enable || isempty(ch.SlotAllocation)
            % Capture this PDCCH sequence's label for the resource info name
            waveinfo.PDCCH(nch).Name = ch.Label;
            continue;
        end
        
        % Get the number of symbols per slot for the associated BWP (CP dependent)
        symbperslot = nr5g.internal.wavegen.symbolsPerSlot(bwps{bwpIdx});
        
        % Create nrPDCCHConfig and nrCarrierConfig inputs for nrPDCCHIndices
        [pdcchInput, ss] = nr5g.internal.wavegen.getPDCCHObject(ch, bwps{bwpIdx}, coreset, searchSpaces, 0);
        carrierInput = nr5g.internal.wavegen.getCarrierCfgObject(bwps{bwpIdx}, cfgObj.NCellID);

        % Exclude any CORESET occasions that would fall outside a slot
        startSymbs = ss.StartSymbolWithinSlot;
        
        % Calculate the initial symbol and slot numbers for the CORESET/search space
        % monitoring occasions expanding by the period across the waveform length
        [potentialslots,initSlot] = nr5g.internal.wavegen.expandbyperiod(ch.SlotAllocation,ch.Period,cfgObj.NumSubframes,bwps{bwpIdx}.SubcarrierSpacing,cfgObj.InitialNSubframe);
        potentialsymbols = reshape(symbperslot*potentialslots + startSymbs',1,[]);
        
        % Also need to expand the indices of the allocated monitoring locations
        % that will be used by the current PDCCH sequence
        % Expand by period so that it covers all the potential symbols
        allocslotindices = 0:numel(potentialsymbols)-1;
        
        % Identify the absolute initial symbols associated with the CORESET (duration) instances that carry the PDCCH
        allocatedsymbols = potentialsymbols(1+allocslotindices);
        
        % Create a data source for this PDCCH sequence
        if coder.target('MATLAB')
            if ch.Coding
                maxSize = ch.DataBlockSize;
            else
                maxSize = 2*ch.AggregationLevel*6*12*3/4;
            end
        else
            maxSize = 2*864; % for max AggregationLevel = 16
        end
        datasource = nr5g.internal.wavegen.hVectorDataSource(ch.DataSource, maxSize);
        
        % Storage for PDCCH instance information
        controlstore = repmat(unitStruct,  1, length(allocatedsymbols));
        
        % Compute n_RNTI for nrPDCCH:
        if ch.RNTI > 0 && ch.RNTI <= 65519 && strcmp(ss.SearchSpaceType, 'ue')
            n_RNTI = ch.RNTI;
        else
            n_RNTI = 0;
        end
        pdcchInput.RNTI = n_RNTI;
        
        % Scrambling NID value for DM-RS (pdcch-DMRS-ScramblingID or NCellID)
        if isempty(ch.DMRSScramblingID)
            nID = cfgObj.NCellID;
        else
            nID = ch.DMRSScramblingID;
        end

        % Expand the power to match the number of unique slots in a period
        [powerVec,uniqueSlotsInPeriod] = expandPower(ch.Power,ch.SlotAllocation,ch.Period);
        powerVecDMRS = db2mag(powerVec + ch.DMRSPower);
        powerVec = db2mag(powerVec);

        % Get the precoding and antenna mapping matrix
        Wpa = ch.Wpa;
        nColumns = size(Wpa,1);
        siz = [12*carrierInput.NSizeGrid carrierInput.SymbolsPerSlot 1];
        
        % Loop over all the PDCCH transmission occasions and write the encoded
        % DCI payloads into the resource elements of the associated PDCCH instances
        for idx = 1:length(allocatedsymbols)

            % 'Absolute' symbol number of current instance
            s = allocatedsymbols(idx);
            
            % Update NSlot/NFrame (turn this absolute number into slot/frame numbers pair)
            nslot = mod(fix(s/symbperslot), bwps{bwpIdx}.SubcarrierSpacing/15 * 10);  % Slot number, in a 10ms frame
            carrierInput.NSlot = nslot;
            numSlotsPerSubframe = carrierInput.SlotsPerSubframe;
            carrierInput.NFrame = floor(s/(symbperslot*numSlotsPerSubframe*10));

            % Index of power vector
            powerInd = getPowerIndex(idx,uniqueSlotsInPeriod);
            
            % Get PDCCH and DM-RS indices, and DM-RS symbols for current instance
            [pdcchIndices, dmrssym, dmrsIndices] = nrPDCCHResources(carrierInput, pdcchInput,'IndexOrientation', 'bwp');
            
            % PDCCH instance symbol/bit capacity
            Gd = length(pdcchIndices);
            G = Gd*2;
            
            if ~isempty(pdcchIndices)
                if ch.Coding
                    % Get the DCI payload bits from the data source
                    dcibits = datasource.getPacket(ch.DataBlockSize);

                    % Validate DCI bit capacity that must be greater than
                    % or equal to K+CRC
                    K = length(dcibits);
                    coder.internal.errorIf(G<(K+24),'nr5g:nrWaveformGenerator:DCISmallRateMatchedLength',nch,K,G,K+24);

                    % Encode the DCI payload to match the PDCCH bit capacity
                    codeword = nrDCIEncode(dcibits,ch.RNTI,G);
                else
                    % Get the PDCCH codeword directly from the data source
                    codeword = datasource.getPacket(G);
                    dcibits = [];
                end
                
                % Get the PDCCH QPSK symbols
                symbols = nrPDCCH(codeword, nID, n_RNTI);
            else
                % A PDCCH monitoring occasion does not exist in current slot
                dcibits = [];
                codeword = [];
                symbols = [];
            end

            % Precoding and antenna mapping of PDCCH and DM-RS symbols
            [pdcchAntSym,pdcchAntInd] = precodeAndMapAntenna(siz,symbols,pdcchIndices,Wpa.');
            [dmrsAntSym,dmrsAntInd] = precodeAndMapAntenna(siz,dmrssym,dmrsIndices,Wpa.');

            % Write PDCCH and DM-RS symbols into the slot grid
            slotAntGrid = nrResourceGrid(carrierInput,nColumns);
            slotAntGrid(pdcchAntInd) = pdcchAntSym*powerVec(powerInd);
            slotAntGrid(dmrsAntInd) = dmrsAntSym*powerVecDMRS(powerInd);

            % Write the slot grid into the entire BWP grid
            slotIdx = floor(s/symbperslot)-initSlot;
            symIdx = (1:symbperslot)+slotIdx*symbperslot;
            ResourceElementGridsPDCCH{bwpIdx}(:,symIdx,1:nColumns) = ResourceElementGridsPDCCH{bwpIdx}(:,symIdx,1:nColumns) + slotAntGrid;
            
            % If this PDCCH sequence is associated with any of the
            % PDSCHs (same RNTI and BWP) and the associated CORESET is
            % not reserved, configure RB reservation for control (turn the PDCCH RE into for this PRB)
            slotN = fix(s/symbperslot);  % 'Absolute' slot number
            resPRBSet = unique(mod(floor(double(pdcchIndices-1)/12),bwps{bwpIdx}.NSizeBWP));                % PRB used in current PDCCH/slot
            resSymSet = slotN*symbperslot+unique(floor(double(pdcchIndices-1)/12/bwps{bwpIdx}.NSizeBWP));   % Symbols used in current PDCCH/slot slot ('absolute' symbol values wrt entire waveform) 

            % Run across all the PDSCH configurations to see if each *might* be affected by this one PDCCH instance
            % If an association (RNTI/BWP) is made then store the PDCCH instance resources in the list for the associated PDSCH
            for dch = 1:numPDSCH
                rc = reservedCounter(dch);  % Counter of nrPDSCHReservedConfig for this data channel

                % Link the PDCCH ('ch') with a PDSCH if they share the same RNTI and BWP 
                % If a link is made then create a PRB reservation for the PDCCH PRB in the BWP (only if the entire containing CORESET hasn't been reserved anyway)
                if (pdsch{dch}.RNTI == ch.RNTI) && (pdsch{dch}.BandwidthPartID == ch.BandwidthPartID) ...
                        && ~isequal(pdsch{dch}.ReservedCORESET,ss.CORESETID)   % Reserve PDCCH resources only if containing CORESET is not already reserved
                    
                    reservedPRB{dch}{rc} = nrPDSCHReservedConfig('PRBSet',resPRBSet,'SymbolSet',resSymSet);
                    reservedCounter(dch) = rc+1;

                    % Include the DM-RS resources in the RE reservation since these can be outside PDCCH PRB.
                    % Assume that the waveform starts at slot 0, and that the reservedRE container
                    % starts at the start of waveform i.e. slot 0.
                    reservedRE{dch, slotN-initSlot+1} = [reservedRE{dch, slotN-initSlot+1}; dmrsIndices - 1];   % Concatenate with other RE for the slot

                end
            end

            % Capture resource info for this PDCCH instance
            controlstore(idx).NSlot = fix(s/symbperslot);
            controlstore(idx).DCIBits = dcibits;
            controlstore(idx).Codeword = codeword;
            controlstore(idx).G = G;
            controlstore(idx).Gd = Gd;
            controlstore(idx).ChannelIndices = pdcchIndices;
            controlstore(idx).ChannelSymbols = symbols*db2mag(ch.Power);
            controlstore(idx).DMRSIndices = dmrsIndices;
            controlstore(idx).DMRSSymbols = dmrssym*db2mag(ch.Power+ch.DMRSPower);
            
        end
        
        % Capture all resources info for this PDCCH sequence
        waveinfo.PDCCH(nch).Name = ch.Label;
        waveinfo.PDCCH(nch).PrecodingMatrix = complex(Wpa);
        waveinfo.PDCCH(nch).CDMLengths = [1 1];
        waveinfo.PDCCH(nch).Resources = controlstore;
        
        % End of PDCCH sequence processing
    end
end


%% Control (PUCCH)
function [ResourceElementGridsPUCCH, controlReservedPRB, waveinfo] = processAndGeneratePUCCH(cfgObj, waveinfo, maxNumPorts)
    
    bwps = cfgObj.BandwidthParts;
    pucch = cfgObj.PUCCH;
    
    ResourceElementGridsPUCCH = createREGrid(bwps, cfgObj.NumSubframes, maxNumPorts);
    
    initEmpty = zeros(0,1);
    coder.varsize('initEmpty',[inf,inf],[1,1]);
    codewordInit = {{int8(initEmpty), int8(initEmpty)}}; % cover the biggest possible container
    coder.varsize('codewordInit{1}',[1,2],[0,1]);
    unitStructPUCCH = struct('NSlot', 0, 'SRBit', int8(initEmpty), 'UCIBits', int8(initEmpty), 'UCI2Bits', int8(initEmpty), ...
        'Codeword', codewordInit, 'G', 0, 'Gd', 0, 'ChannelIndices', uint32(initEmpty), 'ChannelSymbols', complex(initEmpty), ...
        'DMRSIndices', uint32(initEmpty), 'DMRSSymbols', complex(initEmpty));
    
    controlReservedPRB = {}; % No reserved PRBs for uplink
    
    isMATLABPath = coder.target('MATLAB');
    if ~isMATLABPath
        % In the codegen path, the size of the vector data source must be
        % constant.

        % Compute the maximum allowed bit capacity among all PUCCH formats
        % in case coding is disabled or it does not apply
        maxNPRB = 275;        % Maximum number of resource blocks for each OFDM symbol in a resource grid
        maxNRE = 12*maxNPRB;  % Maximum number of resource elements for each OFDM symbol in a resource grid
        maxNumSymbols = 14-2; % Maximum number of symbols allocated for PUCCH for any format
        maxM = 2;             % Maximum applicable modulation order for PUCCH (QPSK - 2 bits per symbol)
        maxSizeUCI = maxNRE * maxNumSymbols * maxM; % Maximum allowed bit capacity in case of no coding

        % TS 38.212 Section 5.2.1 specifies 1706 as the maximum size for
        % the UCI payload. For this reason, we extend this to be the upper
        % limit for UCI part 1 in the codegen path in case coding is enabled.
        maxSizeUCI1 = 1706;
    end

    %% Process the set of PUCCH transmission sequences
    for nch = 1:numel(pucch)
        
        % Get a copy of the current PUCCH channel parameters
        ch = pucch{nch};

        % Get PUCCH format
        formatPUCCH = nr5g.internal.pucch.getPUCCHFormat(ch);

        % Capture high-level resources info for this PUCCH sequence
        waveinfo.PUCCH(nch).Name = ch.Label;
        waveinfo.PUCCH(nch).Format = formatPUCCH;

        % Only process configuration if enabled
        if ~ch.Enable
            % Capture all resources info for this PUCCH sequence
            controlstore = waveinfo.PUCCH(nch).Resources;
            waveinfo.PUCCH(nch).Resources = getPUCCHResourceInfo(controlstore,formatPUCCH);

            continue;
        end

        % Find the appropriate BWP for this channel
        bwpIdx = getBWPIdxByID(bwps, ch.BandwidthPartID);
        bwp = bwps{bwpIdx};

        % Create nrCarrierConfig and nrPUCCHXConfig from wavegen objects
        carrierCfg = nr5g.internal.wavegen.getCarrierCfgObject(bwp, cfgObj.NCellID, cfgObj.IntraCellGuardBands);
        symbperslot = carrierCfg.SymbolsPerSlot;
        pucchObj = nr5g.internal.wavegen.getPUCCHObject(ch, formatPUCCH, symbperslot);

        % Determine the number of RB allocated to this channel
        Mrb = numel(nr5g.internal.allocatedPhysicalResourceBlocks(carrierCfg, pucchObj));

        % Only process configuration if channels have allocated resources
        if Mrb == 0 || isempty(ch.SlotAllocation) || ...
                isempty(ch.SymbolAllocation) || ch.SymbolAllocation(2) == 0

            % Capture all resources info for this PUCCH sequence
            controlstore = waveinfo.PUCCH(nch).Resources;
            waveinfo.PUCCH(nch).Resources = getPUCCHResourceInfo(controlstore,formatPUCCH);

            continue;
        end
        
        % Ensure the allocated PRBs are within the bandwidth part
        interlacing = nr5g.internal.interlacing.isInterlaced(ch);
        coder.internal.errorIf( ~interlacing && any(ch.PRBSet >= bwp.NSizeBWP, 'all'), ...
            'nr5g:nrWaveformGenerator:InvalidPRBSetInBWP',max(ch.PRBSet,[],'all'),'PUCCH',nch,bwp.NSizeBWP,ch.BandwidthPartID);

        % Validate use of sequence hopping for PUCCH
        [pucchObj, isSeqHopValid] = nr5g.internal.wavegen.validatePUCCHSequenceHopping(pucchObj,formatPUCCH,Mrb);
        if ~isSeqHopValid
            coder.internal.warning('nr5g:nrWaveformGenerator:InvalidGroupHopping',nch,formatPUCCH);
        end
        % Only process channels with allocated resources. If there are OFDM
        % symbols configured outside the boundaries of the slot when CP =
        % 'Extended', warn about OFDM symbol allocation.
        if isempty(pucchObj.SymbolAllocation) || pucchObj.SymbolAllocation(2) ~= ch.SymbolAllocation(2)
            coder.internal.warning('nr5g:nrWaveformGenerator:InvalidSymbolAllocation','PUCCH',nch,ch.BandwidthPartID,symbperslot-1);
            if isempty(pucchObj.SymbolAllocation)
                % No symbols are allocated
                % Capture all resources info for this PUCCH sequence and
                % skip processing this configuration
                controlstore = waveinfo.PUCCH(nch).Resources;
                waveinfo.PUCCH(nch).Resources = getPUCCHResourceInfo(controlstore,formatPUCCH);
                continue;
            end
        end
        
        % Locally store value for the number of UCI bits as a double
        numUCIBits = double(ch.NumUCIBits);
        if formatPUCCH > 2
            numUCI2Bits = double(ch.NumUCI2Bits);
        end

        % Create a data source for UCI part 1 for this PUCCH sequence
        dsUCI = ch.DataSourceUCI;
        coder.varsize('dsUCI');
        if formatPUCCH > 1
            if isMATLABPath
                [~, pucchInfo] = nrPUCCHIndices(carrierCfg, pucchObj);
                maxSizeUCI1 = numUCIBits; % Maximum allowed bit capacity with coding
                maxSizeUCI = pucchInfo.G; % Maximum allowed bit capacity in case of no coding
            end
            datasourceUCI1 = nr5g.internal.wavegen.hVectorDataSource(dsUCI, maxSizeUCI1);
        else % PUCCH format 0 and 1
            if isMATLABPath
                maxSizeUCI = 2; % The payload size for PUCCH formats 0 and 1 is at most 2 bits
            end
            if formatPUCCH==0
                % Set maxSizeSR to 2 to overcome a codegen limitation for
                % scalar PN sequences
                maxSizeSR = 2;
                datasourceSR = nr5g.internal.wavegen.hVectorDataSource(ch.DataSourceSR, maxSizeSR);
            end
        end
        datasource = nr5g.internal.wavegen.hVectorDataSource(dsUCI, maxSizeUCI);
        
        % Create a data source for UCI part 2 for this PUCCH sequence
        if formatPUCCH > 2
            if isMATLABPath
                maxSizeUCI2 = numUCI2Bits;
            else
                % TS 38.212 Section 5.2.1 specifies 1706 as the maximum
                % size for the UCI payload. For this reason, we extend this
                % to be the upper limit for UCI part 2 in the codegen case.
                maxSizeUCI2 = 1706;
            end
            datasourceUCI2 = nr5g.internal.wavegen.hVectorDataSource(ch.DataSourceUCI2, maxSizeUCI2);
        end
        
        % Expand the allocated slot sequence by the repetition period,
        % across the length of the waveform
        [allocatedSlots,initSlot] = nr5g.internal.wavegen.expandbyperiod(ch.SlotAllocation,ch.Period,cfgObj.NumSubframes,bwp.SubcarrierSpacing,cfgObj.InitialNSubframe);

        % Expand the power to match the number of unique slots in a period
        [powerVec,uniqueSlotsInPeriod] = expandPower(ch.Power,ch.SlotAllocation,ch.Period);
        if formatPUCCH > 0
            powerVecDMRS = db2mag(powerVec + ch.DMRSPower);
        else
            powerVecDMRS = db2mag(powerVec);
        end
        powerVec = db2mag(powerVec);

        % Storage for PUCCH instance information
        controlstore = repmat(unitStructPUCCH,  1, length(allocatedSlots));
        
        % Initialization for codegen
        codeword = codewordInit; %#ok<NASGU>

        % Get the precoding and antenna mapping matrix
        Wpa = ch.Wpa;
        siz = [12*carrierCfg.NSizeGrid carrierCfg.SymbolsPerSlot 1];
        nColumns = size(Wpa,1);
        
        % Loop over all the allocated slots
        for idx = 1:length(allocatedSlots)
            
            % Get current slot number
            nslot = allocatedSlots(idx);
            carrierCfg.NSlot = nslot;

            % Index of power vector
            powerInd = getPowerIndex(idx,uniqueSlotsInPeriod);
            
            % Create an empty slot grid to contain a single PUCCH instance
            slotgrid = nrResourceGrid(carrierCfg,nColumns);
            
            % Get the slot-oriented PUCCH indices, DM-RS indices and DM-RS
            % symbols, and structural information
            [pucchREindices, pucchInfo] = nrPUCCHIndices(carrierCfg, pucchObj);
            dmrsREindices = nrPUCCHDMRSIndices(carrierCfg, pucchObj);
            dmrsSymbols = nrPUCCHDMRS(carrierCfg, pucchObj);
            
            % Initialize UCI sources for codegen
            srBit = int8(initEmpty);
            uci1Bits = int8(initEmpty);
            uci2Bits = int8(initEmpty);
            coder.varsize('srBit',[1,1],[1,0]);
            coder.varsize('uci1Bits',[maxSizeUCI,1],[1,0]);
            coder.varsize('uci2Bits',[1706,1],[1,0]);
            
            % Process UCI sources and generate codeword
            if formatPUCCH > 1
                if ch.Coding
                    % Get the rate matching value for each UCI part
                    G1 = pucchInfo.G;
                    G2 = 0;
                    
                    if numUCIBits
                        % Get the UCI part 1 bits
                        uci1Bits = int8(datasourceUCI1.getPacket(numUCIBits));
                        L = getCRC(numUCIBits); % Get the CRC length for UCI part 1
                        
                        % Get UCI part 2 bits, if present
                        if (formatPUCCH > 2) && numUCI2Bits
                            % If the length of UCI part 2 is less than 3
                            % bits, zeros are appended to the UCI bit
                            % sequence until its length equals 3 (TS 38.212
                            % Section 6.3.1.1.3)
                            uci2Bits = int8([datasourceUCI2.getPacket(numUCI2Bits); zeros(max(0,3-numUCI2Bits),1)]);
                            L2 = getCRC(length(uci2Bits));
                            
                            % UCI multiplexing happens for format 3 and 4
                            qm = nr5g.internal.getQm(ch.Modulation);
                            G1 = min(pucchInfo.G,ceil((numUCIBits + L)/ch.TargetCodeRate/qm)*qm);
                            G2 = pucchInfo.G - G1;
                        
                            % Check if the number of input bits is larger
                            % than the bit capacity of this PUCCH resource
                            coder.internal.errorIf((numUCIBits+L)>G1,'nr5g:nrWaveformGenerator:InvalidNumUCI1Bits',nch,num2str(formatPUCCH),numUCIBits,L,G1);
                            coder.internal.errorIf((numUCI2Bits+L2)>G2,'nr5g:nrWaveformGenerator:InvalidNumUCI2Bits',nch,num2str(formatPUCCH),numUCI2Bits,L2,G2);
                            
                            % Validate UCI bit capacity
                            validateUCIRateMatch(formatPUCCH, numUCIBits, G1, nch, '1'); % UCI part 1
                            validateUCIRateMatch(formatPUCCH, numUCI2Bits, G2, nch, '2'); % UCI part 2
                        else
                            % Check if the number of input bits is larger
                            % than the bit capacity of this PUCCH resource
                            if formatPUCCH == 2
                                errorID = 'InvalidNumUCIBitsF2';
                            else
                                errorID = 'InvalidNumUCIBitsF34';
                            end
                            coder.internal.errorIf((numUCIBits+L)>G1,['nr5g:nrWaveformGenerator:' errorID],nch,formatPUCCH,numUCIBits,L,G1);
                            
                            % Validate UCI bit capacity for formats 3 and 4
                            if formatPUCCH > 2
                                validateUCIRateMatch(formatPUCCH, numUCIBits, G1, nch, '');
                            end
                        end
                    end
                    
                    % Encode the UCI payload to match the PUCCH bit capacity
                    codedUCI1 = nrUCIEncode(uci1Bits,G1); % Encode UCI part 1 for format 2, 3, and 4
                    codedUCI2 = nrUCIEncode(uci2Bits,G2); % Encode UCI part 2 for format 3 and 4
                    
                    % Multiplex the encoded UCI part 1 and UCI part 2,
                    % assign to a codeword
                    codeword = {nr5g.internal.wavegen.UCIMultiplex(carrierCfg,pucchObj,codedUCI1,codedUCI2)};
                else % No coding
                    % Get the PUCCH codeword directly from the data source
                    uci1Bits = int8(datasource.getPacket(pucchInfo.G));
                    codeword = {uci1Bits};
                end
            else % PUCCH format 0 or 1
                % For PUCCH formats 0 and 1, there is no coding and the UCI
                % bits come directly from the data source.
                uci1Bits = int8(datasource.getPacket(numUCIBits));
                if formatPUCCH==0
                    % For format 0, the codeword is a cell array containing
                    % HARQ-ACK and SR bits.
                    srBit = int8(datasourceSR.getPacket(1));
                    codeword = {uci1Bits srBit};
                else
                    codeword = {uci1Bits};
                end
            end
            
            % PUCCH processing to create the PUCCH symbols
            symbols = nrPUCCH(carrierCfg, pucchObj, codeword);

            % Precoding and antenna mapping
            [pucchAntSym,pucchAntInd] = precodeAndMapAntenna(siz,symbols,pucchREindices,Wpa.');
            [dmrsAntSym,dmrsAntInd] = precodeAndMapAntenna(siz,dmrsSymbols,dmrsREindices,Wpa.');
            
            % Write PUCCH and DM-RS symbols in the slot grid
            if ~isempty(symbols)
                slotgrid(pucchAntInd) = pucchAntSym*powerVec(powerInd);
            end
            slotgrid(dmrsAntInd) = dmrsAntSym*powerVecDMRS(powerInd);
            
            % Combine PUCCH instance with the rest of the BWP grid
            ResourceElementGridsPUCCH{bwpIdx}(:,(nslot-initSlot)*symbperslot+(1:symbperslot),1:nColumns) = ...
                ResourceElementGridsPUCCH{bwpIdx}(:,(nslot-initSlot)*symbperslot+(1:symbperslot),1:nColumns) + slotgrid;
            
            % Capture resource info for this PUCCH instance
            controlstore(idx).NSlot = nslot;
            if formatPUCCH == 0
                controlstore(idx).SRBit = srBit;
            end
            controlstore(idx).UCIBits = uci1Bits;
            if formatPUCCH > 2
                controlstore(idx).UCI2Bits = uci2Bits;
            end
            if formatPUCCH == 0
                controlstore(idx).Codeword = {codeword{1} codeword{2}};
            else
                if isMATLABPath
                    controlstore(idx).Codeword = codeword{1};
                else
                    controlstore(idx).Codeword = {codeword{1} int8(initEmpty)};
                end
            end
            controlstore(idx).G = pucchInfo.G;
            controlstore(idx).Gd = pucchInfo.Gd;
            controlstore(idx).ChannelIndices = pucchREindices;
            controlstore(idx).ChannelSymbols = symbols*powerVec(powerInd);
            controlstore(idx).DMRSIndices = dmrsREindices;
            controlstore(idx).DMRSSymbols = dmrsSymbols*powerVecDMRS(powerInd);
        end
        
        % Capture all resources info for this PUCCH sequence
        waveinfo.PUCCH(nch).CDMLengths = [1 1];
        waveinfo.PUCCH(nch).PrecodingMatrix = complex(Wpa);
        waveinfo.PUCCH(nch).Resources = getPUCCHResourceInfo(controlstore,formatPUCCH);
        
        % End of PUCCH sequence processing
    end
end


%% Shared channel (PDSCH or PUSCH)
function [ResourceElementGridsPXSCH, waveinfo] = processAndGeneratePXSCH(cfgObj, waveinfo, controlReservedPRB, reservedREs, maxNumPorts, isDownlink)

    % Unbundle the channel specific configuration objects for easier
    % access and calculate SS Burst resources reservation
    if isDownlink
        pxsch = cfgObj.PDSCH;
        
        % Get the set of RB level resources, in each BWP, that overlap with
        % the SS burst
        ssburst =  nr5g.internal.wavegen.mapSSBObj2Struct(cfgObj.SSBurst, cfgObj.SCSCarriers);
        ssbreserved = nr5g.internal.wavegen.ssburstResources(ssburst,cfgObj.SCSCarriers,cfgObj.BandwidthParts);

    else % Uplink
        pxsch = cfgObj.PUSCH;
    end

    bwps = cfgObj.BandwidthParts;
    ResourceElementGridsPXSCH = createREGrid(bwps, cfgObj.NumSubframes, maxNumPorts);
    
    % Process the set of PXSCH transmission sequences
    % Create a single DL-SCH or UL-SCH transport channel processing object
    % for use with all the PXSCH sequences
    if isDownlink
        xlsch = nrDLSCH('UniformCellOutput',true,'TargetCodeRate',526/1024*ones(1,2));
        pxschString = 'PDSCH';
    else % Uplink
        xlsch = nrULSCH('UniformCellOutput',true,'TargetCodeRate',526/1024*ones(1,2));
        pxschString = 'PUSCH';
    end
    xlsch.MultipleHARQProcesses = false;
    defaultBufferSize = xlsch.LimitedBufferSize;

    initEmpty = zeros(0,1);
    coder.varsize('initEmpty',[inf,inf],[1,1]);
    codewordInit = {{int8(initEmpty), int8(initEmpty)}}; % cover the biggest possible container
    coder.varsize('codewordInit{1}',[1,2],[0,1]);
    trblkInit = {{initEmpty initEmpty}};
    unitStruct = struct('NSlot', 0, 'TransportBlockSize', [0 0], 'TransportBlock', trblkInit, ...
              'RV', 0, 'Codeword', codewordInit, 'G', 0, 'Gd', 0, ...
              'ChannelIndices',  uint32(initEmpty), 'ChannelSymbols', complex(initEmpty), ...
              'DMRSIndices',  uint32(initEmpty), 'DMRSSymbols', complex(initEmpty), 'DMRSSymbolSet', complex(initEmpty'), ...
              'PTRSIndices',  uint32(initEmpty), 'PTRSSymbols', complex(initEmpty), 'PTRSSymbolSet', complex(initEmpty'));
    coder.varsize('unitStruct.RV', 'unitStruct.G', 'unitStruct.Gd');

    % Process each shared channel sequence configuration
    for nch = 1:numel(pxsch)
        
        % Get a copy of the current PXSCH channel parameters
        ch = pxsch{nch};
        is2CW = ch.NumLayers > 4;

        % Capture this PXSCH sequence's label for the resource info name
        waveinfo.(pxschString)(nch).Name = ch.Label;

        % Only process configuration if enabled
        if ~ch.Enable
            continue;
        end

        % Find the appropriate BWP for this channel
        bwpIdx = getBWPIdxByID(bwps, ch.BandwidthPartID);
        bwp = bwps{bwpIdx};

        % Create nrCarrierConfig and nrPXSCHConfig from wavegen objects.
        % For downlink, no intracell guard bands are defined
        if isDownlink
            intraCellGuardBands = {nrIntraCellGuardBandsConfig};
        else
            % All the resource grids here are BWP oriented, so adjust guard
            % bands frequency reference point from carrier to BWP origin.
            intraCellGuardBands = cfgObj.IntraCellGuardBands;
            scscarriers = cfgObj.SCSCarriers;
            carrierID = nr5g.internal.wavegen.getCarrierIDByBWPIndex(scscarriers, bwps, bwpIdx);
            scscarrier = scscarriers{carrierID};
            intraCellGuardBands = nr5g.internal.interlacing.adjustGuardbandsFrequencyReference(intraCellGuardBands,scscarrier,bwp);
        end
        carrierCfg = nr5g.internal.wavegen.getCarrierCfgObject(bwp, cfgObj.NCellID, intraCellGuardBands);
        symbperslot = carrierCfg.SymbolsPerSlot;
        pxschObj = nr5g.internal.wavegen.getPXSCHObject(ch, symbperslot, {}, uint32([]), isDownlink);

        % Determine the number of RB allocated to this channel
        Mrb = numel(nr5g.internal.allocatedPhysicalResourceBlocks(carrierCfg, pxschObj));

        % Only process configuration if channels have allocated resources
        if Mrb == 0 || isempty(ch.SlotAllocation) || ...
                isempty(ch.SymbolAllocation) || ch.SymbolAllocation(2) == 0

            continue;
        end
        
        % Ensure the allocated PRBs are within the bandwidth part
        interlacing = nr5g.internal.interlacing.isInterlaced(ch);
        coder.internal.errorIf( ~interlacing && any(ch.PRBSet >= bwp.NSizeBWP, 'all'), ...
            'nr5g:nrWaveformGenerator:InvalidPRBSetInBWP',max(ch.PRBSet,[],'all'),pxschString,nch,bwp.NSizeBWP,ch.BandwidthPartID);
        
        % If there are OFDM symbols configured outside the boundaries of
        % the slot when CP = 'Extended', warn about OFDM symbol allocation.
        if isempty(pxschObj.SymbolAllocation) || pxschObj.SymbolAllocation(2) ~= ch.SymbolAllocation(2)
            coder.internal.warning('nr5g:nrWaveformGenerator:InvalidSymbolAllocation',pxschString,nch,ch.BandwidthPartID,symbperslot-1);
        end

        % Process buffer size for limited buffer rate matching
        if ch.Coding
            % Only need to process when Coding is on. When Coding is off,
            % xlsch doesn't get used and doesn't need updating

            if ~isDownlink
                % For UL, turn LBRM on/off. For DL, nrDLSCH doesn't have
                % LimitedBufferRateMatching property as LBRM is expected to
                % be always on.
                xlsch.LimitedBufferRateMatching = ch.LimitedBufferRateMatching;
            end
            if ch.LimitedBufferRateMatching
                % When LimitedBufferRateMatching is on, validate related
                % properties and calculate NRef for the encoder.
                validateLBRMProperties(ch,pxschString,nch);
                nRef = nr5g.internal.wavegen.getNRef(ch,bwp.NSizeBWP);
                xlsch.LimitedBufferSize = nRef;
            elseif isDownlink
                % When LimitedBufferRateMatching is manually turned off for
                % DL, use the default NRef for the encoder to have the same
                % behavior as before R2024b.
                xlsch.LimitedBufferSize = defaultBufferSize;
            end

        end
        
        % Get the PXSCH resource element indices, structural information,
        % and the number of layers or antenna ports
        if isDownlink
            [pxschindtmp,modinfotmp] = nrPDSCHIndices(carrierCfg, pxschObj);
            tbScaling = ch.TBScaling;
            prbSet = pxschObj.PRBSet(:).';
        else
            [pxschindtmp,modinfotmp] = nrPUSCHIndices(carrierCfg, pxschObj);
            tbScaling = 1;
            prbSet = modinfotmp.PRBSet;
        end
        nPorts = size(pxschindtmp,2);
        
        % Create a data source for this PXSCH sequence
        if coder.target('MATLAB')
            if ch.Coding
                maxSize = max(nrTBS(ch.Modulation, ch.NumLayers, numel(prbSet), modinfotmp.NREPerPRB, ch.TargetCodeRate, ch.XOverhead, tbScaling));
            else
                maxSize = max(modinfotmp.G,[],'all');
            end
        else
            maxScaling = 1;
            maxNPRB = 275;
            maxNRE = 156*maxNPRB;
            maxRate = 1;
            maxM = 10; % Up to 1024QAM
            maxNumLayers = 8;
            maxSize = maxScaling * maxNRE * maxRate * maxM * maxNumLayers;
        end
        datasource = nr5g.internal.wavegen.hVectorDataSource(ch.DataSource, maxSize);
        
        if ~isDownlink
            numACKBits = double(ch.NumACKBits);
            numCSI1Bits = double(ch.NumCSI1Bits);
            numCSI2Bits = double(ch.NumCSI2Bits);
            numCGUCIBits = double(ch.NumCGUCIBits);
            
            % Create a data source for UCI on PUSCH for this PUSCH sequence
            ackFlag = ch.Coding && ch.EnableACK && (numACKBits>0);
            csi1Flag = ch.Coding && ch.EnableCSI1 && (numCSI1Bits>0);
            csi2Flag = csi1Flag && ch.EnableCSI2 && (numCSI2Bits>0);
            cguciFlag = ch.Coding && ch.EnableCGUCI && (numCGUCIBits>0);
            if coder.target('MATLAB')
                maxSizeACK = numACKBits;
                maxSizeCSI1 = numCSI1Bits;
                maxSizeCSI2 = numCSI2Bits;
                maxSizeCGUCI = numCGUCIBits;
            else
                % TS 38.212 Section 5.2.1 specifies 1706 as the maximum
                % size for the UCI payload. For this reason, we extend this
                % to be the upper limit for ACK, CSI1, CSI2, and CG-UCI in
                % the codegen case.
                maxSizeACK = 1706;
                maxSizeCSI1 = 1706;
                maxSizeCSI2 = 1706;
                maxSizeCGUCI = 1706;
            end
            datasourceACK = nr5g.internal.wavegen.hVectorDataSource(ch.DataSourceACK, maxSizeACK);
            datasourceCSI1 = nr5g.internal.wavegen.hVectorDataSource(ch.DataSourceCSI1, maxSizeCSI1);
            datasourceCSI2 = nr5g.internal.wavegen.hVectorDataSource(ch.DataSourceCSI2, maxSizeCSI2);
            datasourceCGUCI = nr5g.internal.wavegen.hVectorDataSource(ch.DataSourceCGUCI, maxSizeCGUCI);
        end
        
        % Initialize modinfo
        modinfo = struct('G',0,'Gd',0,'NREPerPRB',0,'DMRSSymbolSet',initEmpty','PTRSSymbolSet',initEmpty','CDMGroups',[],'CDMLengths',[0 0]);
        coder.varsize('modinfo.G', 'modinfo.DMRSSymbolSet', 'modinfo.CDMGroups', 'modinfo.CDMLengths');
        
        % Expand the allocated slot sequence by the repetition period, across
        % the length of the waveform
        [allocatedSlots,initSlot] = nr5g.internal.wavegen.expandbyperiod(ch.SlotAllocation,ch.Period,cfgObj.NumSubframes,bwp.SubcarrierSpacing,cfgObj.InitialNSubframe);

        % Expand the power to match the number of unique slots in a period
        [powerVec,uniqueSlotsInPeriod] = expandPower(ch.Power,ch.SlotAllocation,ch.Period);
        powerVecDMRS = db2mag(powerVec + ch.DMRSPower);
        powerVecPTRS = db2mag(powerVec + ch.PTRSPower);
        powerVec = db2mag(powerVec);

        % Storage for PXSCH instance information
        datastore = repmat(unitStruct,  1, length(allocatedSlots));
        
        % Initialization for codegen
        codeword = codewordInit{1};
        trblksize = [0 0]; % Initialize to 2 codewords
        
        % Get the total set of reserved PRB-level resources that might be 
        % associated with this PDSCH transmission sequence
        if isDownlink
            pxschObj.ReservedPRB = processReservedPRB(ch, cfgObj, nch, ssbreserved, controlReservedPRB{nch});
        end

        % Get the precoding and antenna mapping matrix and number of output
        % columns
        Wpa = ch.Wpa;
        nColumns = size(Wpa,1);

        % Loop over all the transmission instances/allocated slots for 
        % this configuration sequence and create each instance
        for i = 1:length(allocatedSlots)
            
            % Get current slot number and update the carrier/cell settings
            nslot = allocatedSlots(i);
            carrierCfg.NSlot = nslot;

            % Index of power vector
            powerInd = getPowerIndex(i,uniqueSlotsInPeriod);
            
            % Create an empty slot grid to contain a single PXSCH instance
            slotgrid = nrResourceGrid(carrierCfg,nColumns);
            
            % Get the slot-oriented PXSCH indices, DM-RS indices and DM-RS
            % symbols, PT-RS indices and symbols, and structural information
            if isDownlink
                pxschObj.ReservedRE = reservedREs{nch, nslot-initSlot+1};
            end
            [pxschREindices,dmrsREindices,dmrsSymbols,ptrsREindices,ptrsSymbols,modinfo] = ...
                nr5g.internal.wavegen.PXSCHResources(carrierCfg, pxschObj);
            
            % Transport channel processing
            trblk = trblkInit{1};
            if ch.Coding
                % Get the RV value for this transmission instance
                if is2CW % 2 codewords
                    if iscell(ch.RVSequence) % nrWavegenPXSCHConfig ensures cell has 2 elements
                        rvidx1 = mod(i-1,length(ch.RVSequence{1}))+1;
                        rvidx2 = mod(i-1,length(ch.RVSequence{2}))+1;
                        rv = [ch.RVSequence{1}(rvidx1) ch.RVSequence{2}(rvidx2)];
                    else
                        % Scalar expansion
                        rvidx1 = mod(i-1,length(ch.RVSequence))+1;
                        rv = [ch.RVSequence(rvidx1) ch.RVSequence(rvidx1)];
                    end
                else
                    if iscell(ch.RVSequence)
                        rvidx1 = mod(i-1,length(ch.RVSequence{1}))+1;
                        rv = ch.RVSequence{1}(rvidx1);
                    else
                        rvidx1 = mod(i-1,length(ch.RVSequence))+1;
                        rv = ch.RVSequence(rvidx1);
                    end
                end
                
                % For the first RV in a sequence, get a new transport block
                % from the data source and pass it to the XL-SCH processing
                if rvidx1 == 1
                    trblksize = nrTBS(ch.Modulation, ch.NumLayers, numel(prbSet), modinfo.NREPerPRB, ch.TargetCodeRate, ch.XOverhead, tbScaling);
                    trblk{1} = datasource.getPacket(trblksize(1));
                    if is2CW % 2 codewords
                        trblk{2} = datasource.getPacket(trblksize(2));
                    end
                    setTransportBlock(xlsch,trblk);
                    % Set TargetCodeRate. Use scalar expansion to make sure
                    % that TargetCodeRate is always a two-element array to
                    % avoid codegen issues
                    xlsch.TargetCodeRate = ch.TargetCodeRate .* ones(1,2);
                end
                
                % Create a codeword
                if isDownlink
                    % DL-SCH processing to create a codeword
                    codeword = getCodeword(ch, xlsch, modinfo.G, rv);
                else
                    % When both HARQ-ACK and CG-UCI are defined,
                    % Section 6.3.2.1.4 of TS 38.212 specifies the UCI
                    % bit sequence as the union of the CG-UCI bits
                    % and the HARQ-ACK bits. For this reason, the
                    % function considers any active CG-UCI source as an
                    % extension to HARQ-ACK.
                    ack = int8(datasourceACK.getPacket(numACKBits*ackFlag)); % Get ACK payload
                    cguci = int8(datasourceCGUCI.getPacket(numCGUCIBits*cguciFlag)); % Get CG-UCI payload
                    % Get the overall ACK
                    if ackFlag && cguciFlag % HARQ-ACK and CG-UCI
                        effACK = [cguci; ack];
                    elseif ~ackFlag && cguciFlag % CG-UCI only
                        % In case of CG-UCI only transmission, treat CG-UCI as ACK
                        effACK = cguci;
                        pxschObj.BetaOffsetACK = ch.BetaOffsetCGUCI;
                    else % HARQ-ACK only or neither HARQ-ACK or CG-UCI
                        effACK = ack;
                    end
                    oeffACK = length(effACK);
                    
                    % Update the transport block for UCI on PUSCH without UL-SCH
                    % The value of EnableULSCH is considered only if at
                    % least one UCI source is present
                    if (~ch.EnableULSCH && (ackFlag || csi1Flag || cguciFlag))
                        trblk = trblkInit{1};
                        trblksize = [numel(trblk{1}) numel(trblk{2})];
                        setTransportBlock(xlsch,trblk);
                    end
                    
                    % Create a copy of the PUSCH configuration object for
                    % UCI on PUSCH multiplexing. In case of interlacing,
                    % set the PRBSet of this object from the RB set of the
                    % configured interlaces and RBSetIndex. This avoids
                    % limitations of nrULSCHInfo and nrULSCHMultiplex as
                    % they do not support interlacing.
                    puschUCICfg = pxschObj;
                    if pxschObj.Interlacing
                        puschUCICfg.Interlacing = false;
                        puschUCICfg.PRBSet = prbSet;
                        puschUCICfg.FrequencyHopping = 'neither';
                    end

                    % Get the UL-SCH and UCI coding information
                    rmInfo = nrULSCHInfo(puschUCICfg,ch.TargetCodeRate,trblksize,oeffACK,numCSI1Bits*csi1Flag,numCSI2Bits*csi2Flag);
                    [QUCI,~,~,ModUCI] = nr5g.internal.pusch.getUCIMultiplexInfo(puschUCICfg,ch.TargetCodeRate);

                    % Validate UCI bits against rate match length
                    numBits = [numACKBits*ackFlag, numCSI1Bits*csi1Flag, numCSI2Bits*csi2Flag, numCGUCIBits*cguciFlag];
                    G = [rmInfo.GACK(QUCI+1), rmInfo.GCSI1(QUCI+1), rmInfo.GCSI2(QUCI+1), rmInfo.GACK(QUCI+1)];
                    validateUCIOnPUSCHRateMatch(numBits, G, nch);

                    % Get CSI part 1 and CSI part 2 bits
                    csi1 = int8(datasourceCSI1.getPacket(numCSI1Bits*csi1Flag)); % Get CSI part 1 payload
                    csi2 = int8(datasourceCSI2.getPacket(numCSI2Bits*csi2Flag)); % Get CSI part 2 payload

                    % Encode UCI
                    codedACK  = nrUCIEncode(effACK,rmInfo.GACK(QUCI+1),ModUCI);
                    codedCSI1 = nrUCIEncode(csi1,rmInfo.GCSI1(QUCI+1),ModUCI);
                    codedCSI2 = nrUCIEncode(csi2,rmInfo.GCSI2(QUCI+1),ModUCI);

                    % Encode UL-SCH
                    codedULSCH = getCodeword(ch, xlsch, rmInfo.GULSCH, rv);

                    % Multiplex UL-SCH and UCI to create a codeword
                    codeword = nrULSCHMultiplex(puschUCICfg,ch.TargetCodeRate,trblksize,codedULSCH,codedACK,codedCSI1,codedCSI2);
                end
                
                % PXSCH physical channel processing to create the PXSCH symbols
                if isDownlink
                    symbols = nrPDSCH(carrierCfg, pxschObj, codeword);
                else
                    [symbols, ptrsSymbols] = nrPUSCH(carrierCfg, pxschObj, codeword);
                end
            else
                % If transport coding is not enabled then get the codeword
                % directly from the data source
                codeword{1} = int8(datasource.getPacket(modinfo.G(1)));
                if is2CW
                    codeword{2} = int8(datasource.getPacket(modinfo.G(2)));
                end
                rv = [];
                trblk = trblkInit{1};
                if isDownlink
                    symbols = nrPDSCH(carrierCfg, pxschObj, codeword);
                else
                    [symbols, ptrsSymbols] = nrPUSCH(carrierCfg, pxschObj, codeword);
                end
            end

            % Precoding and antenna mapping of PXSCH, DM-RS, and PT-RS
            % symbols
            siz = [size(slotgrid,1) size(slotgrid,2) nPorts];
            [antSym,antInd] = precodeAndMapAntenna(siz,symbols,pxschREindices,Wpa.');
            [dmrsAntSym,dmrsAntInd] = precodeAndMapAntenna(siz,dmrsSymbols,dmrsREindices,Wpa.');
            [ptrsAntSym,ptrsAntInd] = precodeAndMapAntenna(siz,ptrsSymbols,ptrsREindices,Wpa.');
            
            % Write the PXSCH, DM-RS, and PT-RS symbols in the slot grid
            if ~isempty(antSym)
                slotgrid(antInd) = antSym*powerVec(powerInd);
            end
            slotgrid(dmrsAntInd) = dmrsAntSym*powerVecDMRS(powerInd);
            if isDownlink || ~ch.TransformPrecoding
                % For downlink and uplink with no transform precoding, the
                % PT-RS power scaling is in addition to the power scaling
                % of the channel
                ptpower = powerVecPTRS(powerInd);
            else
                % For uplink with transform precoding, the symbols
                % generated by nrPUSCH comprise of both the PUSCH data and
                % the PT-RS so that the overall symbols can be mapped to
                % the resource grid. Thus, the PT-RS specific power scaling
                % does not apply in this case.
                ptpower = powerVec(powerInd);
            end
            slotgrid(ptrsAntInd) = ptrsAntSym*ptpower;
            
            % Combine PXSCH instance with the rest of the BWP grid
            ResourceElementGridsPXSCH{bwpIdx}(:,(nslot-initSlot)*symbperslot+(1:symbperslot),1:nColumns) = ...
                ResourceElementGridsPXSCH{bwpIdx}(:,(nslot-initSlot)*symbperslot+(1:symbperslot),1:nColumns) + slotgrid;
            
            % Capture resource information for this PXSCH instance
            datastore(i).NSlot = nslot;
            if coder.target('MATLAB') && ~is2CW
                datastore(i).TransportBlockSize = numel(trblk{1});
                datastore(i).TransportBlock = trblk{1};
                datastore(i).Codeword = codeword{1};
            else % 2 codewords or codegen path
                datastore(i).TransportBlockSize = [numel(trblk{1}) numel(trblk{2})];
                datastore(i).TransportBlock = trblk;
                datastore(i).Codeword = codeword;
            end
            datastore(i).RV = rv;
            datastore(i).G = modinfo.G;
            datastore(i).Gd = modinfo.Gd;
            datastore(i).ChannelIndices =  uint32(pxschREindices);
            datastore(i).ChannelSymbols = symbols*powerVec(powerInd);
            datastore(i).DMRSSymbolSet = modinfo.DMRSSymbolSet;
            datastore(i).DMRSIndices = uint32(dmrsREindices);
            datastore(i).DMRSSymbols = dmrsSymbols*powerVecDMRS(powerInd);
            datastore(i).PTRSSymbolSet = modinfo.PTRSSymbolSet;
            datastore(i).PTRSIndices =  uint32(ptrsREindices);
            datastore(i).PTRSSymbols = ptrsSymbols*ptpower;
        end
        
        % Capture all resources information for this PXSCH sequence
        waveinfo.(pxschString)(nch).Name = ch.Label;
        if ~isDownlink && strcmpi(ch.TransmissionScheme,'codebook')
            waveinfo.(pxschString)(nch).PrecodingMatrix = (nrPUSCHCodebook(ch.NumLayers,ch.NumAntennaPorts,ch.TPMI,ch.TransformPrecoding,ch.CodebookType)).';
        else
            waveinfo.(pxschString)(nch).PrecodingMatrix = complex(Wpa);
        end
        waveinfo.(pxschString)(nch).CDMLengths = modinfo.CDMLengths;
        waveinfo.(pxschString)(nch).Resources = datastore;
        
        % End of PXSCH sequence processing
    end
end

%% OFDM modulation and combination of BWP parts
function [waveform, gridset] = ofdmModAndCombineBWP(cfgObj, ResourceElementGridsXRS, ResourceElementGridsPXCCH, ResourceElementGridsPXSCH, sr, waveform)
    
    carriers = cfgObj.SCSCarriers;
    carrierscs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers, 'SubcarrierSpacing', 'double');
    bwps = cfgObj.BandwidthParts;
    bwpscs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(bwps, 'SubcarrierSpacing', 'double');
    carrierFreq = cfgObj.CarrierFrequency;
    
    % Initialize output variables for the baseband waveform and info structure
    unitStruct = struct('BandwidthPartID',0,...
        'ResourceGridBWP',complex(zeros(1, 1, 0)),'ResourceGridInCarrier',complex(zeros(1, 1, 0)), ...
        'Info', struct('Nfft', 0, 'SampleRate', 0, 'CyclicPrefixLengths', zeros(1, 14), ...
        'SymbolLengths', zeros(1, 14), 'Windowing', 0, 'SymbolPhases', zeros(1, 14), ...
        'SymbolsPerSlot',  0, 'SlotsPerSubframe', 0, 'SlotsPerFrame', 0, 'k0', 0));
    coder.varsize('unitStruct.ResourceGridBWP', 'unitStruct.ResourceGridInCarrier', ...
        'unitStruct.Info.CyclicPrefixLengths', 'unitStruct.Info.SymbolLengths', 'unitStruct.Info.SymbolPhases');
    gridset = repelem(unitStruct, 1, numel(cfgObj.BandwidthParts));
    
    % Establish the k0 of each SCS carrier
    k0c = nr5g.internal.wavegen.getFrequencyOffsetk0(carriers);
    
    % Modulate all the BWP grids and combine all into a single baseband waveform matrix
    for bp = 1:numel(bwps)
        
        % Get the current BWP RE grid
        bgrid = ResourceElementGridsXRS{bp} + ResourceElementGridsPXCCH{bp} + ResourceElementGridsPXSCH{bp};
        
        % Get a copy of the SCS carrier config associated with the BWP numerology
        carrierID = nr5g.internal.wavegen.getCarrierIDByBWPIndex(carriers, bwps, bp);
        carrier = carriers{carrierID};
        nrb = carrier.NSizeGrid;
        
        bwpOff = bwps{bp}.NStartBWP-carriers{carrierID}.NStartGrid;
        % Check BWP dimensions relative to SCS carrier
        coder.internal.errorIf((bwpOff+bwps{bp}.NSizeBWP) > nrb, ...
            'nr5g:nrWaveformGenerator:InvalidBWPInSCSCarrier', bp,bwps{bp}.NSizeBWP,bwpOff,bwps{bp}.SubcarrierSpacing,nrb);
        
        % Modulate the entire grid
        % Create nrCarrierConfig object, as flat nrOFDMInfo signature has
        % extra coder.const requirements in codegen
        c = nr5g.internal.wavegen.getCarrierCfgObject(carrier, cfgObj.NCellID, bwps{bp}.CyclicPrefix);
        c.NSlot = cfgObj.InitialNSubframe*c.SlotsPerSubframe;
        c.NFrame = floor(cfgObj.InitialNSubframe/10);
        
        % Create empty SCS carrier grid and assign in the BWP
        cgrid = repmat(nrResourceGrid(c,size(bgrid,3)),1,cfgObj.NumSubframes*c.SlotsPerSubframe);
        cgrid(12*bwpOff+ (1:size(bgrid,1)),:,:) = bgrid;
        
        % Calculate numerology dependent k0 offset
        k0tmp = k0c(carrierscs == bwpscs(bp));
        k0 = k0tmp(1);

        % Generate the baseband waveform, including phase compensation
        % for carrier frequency and k0 offset
        f_k0 = k0 *carrier.SubcarrierSpacing * 1e3;
        [bwpwave, minfo] = nr5g.internal.wavegen.nrOFDMModulateCallForCodegen(c, cgrid, ...
            bwps{bp}.SubcarrierSpacing, bwps{bp}.CyclicPrefix, ...
            cfgObj.WindowingPercent, sr, carrierFreq + f_k0);
        
        % Apply frequency shift by f_k0, if required
        if f_k0~=0
            t = repmat((0:size(bwpwave,1)-1)' / sr, 1, size(bwpwave,2));
            bwpwave = bwpwave .* exp(1j*2*pi*f_k0*t);
        end
        
        % Add k0 value to the info
        minfo.k0 = k0;
        
        % Combine this BWP with the rest of the waveform
        nColumns = size(bwpwave,2);
        waveform(:,1:nColumns) = waveform(:,1:nColumns) + bwpwave;
        
        % Capture the intermediate grids and modulation info
        gridset(bp).BandwidthPartID = bwps{bp}.BandwidthPartID;
        gridset(bp).ResourceGridBWP = bgrid;
        gridset(bp).ResourceGridInCarrier = cgrid;
        gridset(bp).Info = minfo;
    end
end


%% SS Burst
function  waveform = addSSBurst(cfgObj, ssburst, sr, waveform)
    
    % Add SS burst sequence
    if ssburst.Enable
        % The nr5g.internal.wavegen.hSSBurst function creates a 5ms half
        % frames of data and the waveform is parameterized in terms of 1ms
        % subframes so we can work out how many complete instances are
        % required, then generate and extract portion required in the
        % output waveform
        ssburst.SampleRate = sr;

        % Get the SSB precoding and antenna mapping matrix
        Wpa = calculatePrecodeAndMapMatrix(cfgObj.SSBurst,size(waveform,2));

        % Number of complete half frames required to cover the waveform
        nhframestart = floor(cfgObj.InitialNSubframe/5);
        nhframeend = floor((cfgObj.InitialNSubframe+cfgObj.NumSubframes)/5);

        % Burst waveform variable
        burstwaveform = [];
        for i=nhframestart:nhframeend
            % Create the half frame sequences and concatenate
            ssburst.NHalfFrame = mod(i,2);
            ssburst.NFrame = fix(i/2);
            burstwavetmp = nr5g.internal.wavegen.hSSBurst(ssburst, cfgObj.NCellID, ...
                cfgObj.WindowingPercent, cfgObj.CarrierFrequency, cfgObj.ChannelBandwidth, Wpa);
            burstwaveform = [burstwaveform; burstwavetmp];
        end

        % Clip the burstwaveform to the portion required by the waveform,
        % and combine it with the rest of the waveform
        startpos = sr*mod(cfgObj.InitialNSubframe,5)*1e-3;
        burstwaveform = burstwaveform(startpos+1:startpos+size(waveform,1),:);
        waveform = waveform + burstwaveform;

    end
end


%% Helper subfunctions
function ResourceElementGrids = createREGrid(bwp, nsf, maxNumPorts)
    numBWP = numel(bwp);
    ResourceElementGrids = cell(1, numBWP);
    for idx = 1:numBWP
        symbPerSlot = nr5g.internal.wavegen.symbolsPerSlot(bwp{idx});
        ResourceElementGrids{idx} = complex(zeros(bwp{idx}.NSizeBWP*12, nsf*1*symbPerSlot*fix(bwp{idx}.SubcarrierSpacing/15), maxNumPorts));
    end
end

function bwpIdx = getBWPIdxByID(bwp, ID)
    bwpIdx = NaN;
    for idx = 1:numel(bwp)
        if ID == bwp{idx}.BandwidthPartID
            bwpIdx = idx;
            return;
        end
    end
end

% Validate UCI rate matched length for formats 3 and 4 which must be
% greater than the minimum value derived from the UCI source payload size
% and smaller than 8192
function validateUCIRateMatch(formatPUCCH, numBits, G, n, uciPart)
%  Validate UCI rate matched lengths, based on the PUCCH format, the input
%  length NUMBITS and rate match lengths G, when UCI is to be multiplexed
%  on PUCCH N. UCIPART is a char of value '', '1', or '2'. If UCIPART is an
%  empty character array, the PUCCH configuration has no UCI part 2.

    if numBits > 0
        % Validate rate matched output length that must be in [minG, 8192]
        minG = nr5g.internal.getMinUCIBitCapacity(numBits);

        errorFlagTooSmall = G<minG;
        errorFlagTooLarge = G>8192;
        errorFiller = {n, formatPUCCH, numBits, G, minG};
        if isempty(uciPart) % Format 3 or 4 with no UCI part 2
            errorIDSmall = 'UCISmallRateMatchedLength';
            errorIDLarge = 'UCILargeRateMatchedLength';
        else
            errorIDSmall = ['UCISmallRateMatchedLengthPart' uciPart];
            errorIDLarge = ['UCILargeRateMatchedLengthPart' uciPart];
        end
        coder.internal.errorIf(errorFlagTooSmall, ['nr5g:nrWaveformGenerator:' errorIDSmall], errorFiller{1,:});
        coder.internal.errorIf(errorFlagTooLarge, ['nr5g:nrWaveformGenerator:' errorIDLarge], errorFiller{1,:});
    end
end

% Validate UCI on PUSCH rate matched length which must be greater than the
% minimum value derived from the UCI source payload size and smaller than 8192
function validateUCIOnPUSCHRateMatch(bits,allG,n)
%  Validate UCI on PUSCH rate matched lengths, based on the input length
%  BITS and rate match lengths ALLG, when UCI is to be multiplexed on PUSCH
%  N. BITS is a 4-element vector containing bits used by HARQ-ACK, CSI part
%  1, CSI part 2, and CG-UCI, respectively. ALLG is a 4-element vector
%  containing the bit capacity for the codeword containing UCI relative to
%  HARQ-ACK, CSI part 1, CSI part 2, and CG-UCI.

    if bits(1)>0 && bits(4)>0
        % If both HARQ-ACK and CG-UCI are active, they are considered
        % together in the UCI rate match validation. The total number of
        % UCI bits is the sum of HARQ-ACK bits and CG-UCI bits.
        source = {'', 'CSI1', 'CSI2'}; % The first string is not used so leaving it empty
        sourceBetaOffset = {'BetaOffsetACK', 'BetaOffsetCSI1', 'BetaOffsetCSI2'};
        bits(1) = bits(1) + bits(4);
        ackAndCguci = true;
    else
        source = {'HARQ-ACK', 'CSI1', 'CSI2', 'CG-UCI'};
        sourceBetaOffset = {'BetaOffsetACK', 'BetaOffsetCSI1', 'BetaOffsetCSI2', 'BetaOffsetCGUCI'};
        ackAndCguci = false;
    end

    % Validate the bit capacity
    for s = 1:length(source)
        G = allG(s);
        numBits = bits(s);
        
        if numBits > 0
            % Validate UCI rate matched length for this UCI source
            minG = nr5g.internal.getMinUCIBitCapacity(numBits);
            errorFlagTooSmall = G<minG;
            errorFlagTooLarge = G>8192;
            errorFiller = {n, bits(1), bits(1)-bits(4), bits(4), G,      minG;                 % For the case of HARQ-ACK and CG-UCI
                           n, numBits, source{s},       G,       minG+1, sourceBetaOffset{s}}; % For all cases other than HARQ-ACK and CG-UCI
            if s==1 && ackAndCguci
                coder.internal.errorIf(errorFlagTooSmall,'nr5g:nrWaveformGenerator:ACKAndCGUCISmallRateMatchLength',errorFiller{1,:});
                coder.internal.errorIf(errorFlagTooLarge,'nr5g:nrWaveformGenerator:ACKAndCGUCILargeRateMatchLength',errorFiller{1,:});
            else
                coder.internal.errorIf(errorFlagTooSmall,'nr5g:nrWaveformGenerator:UCIOnPUSCHSmallRateMatchedLength',errorFiller{2,:});
                coder.internal.errorIf(errorFlagTooLarge,'nr5g:nrWaveformGenerator:UCIOnPUSCHLargeRateMatchedLength',errorFiller{2,:});
            end
        end
    end
end

% XL-SCH processing to create a codeword
function codeword = getCodeword(ch, xlsch, G, rv)

    % Initialization for codegen
    codeword = coder.nullcopy({int8(zeros(0,1)) int8(zeros(0,1))}); %#ok<NASGU>
    
    % Multiple calls with const numlayers for codegen
    if ch.NumLayers == 1
        codeword = cat(2,xlsch(ch.Modulation, 1, G, rv), {int8(zeros(0,1))});
    elseif ch.NumLayers == 2
        codeword = cat(2,xlsch(ch.Modulation, 2, G, rv), {int8(zeros(0,1))});
    elseif ch.NumLayers == 3
        codeword = cat(2,xlsch(ch.Modulation, 3, G, rv), {int8(zeros(0,1))});
    elseif ch.NumLayers == 4
        codeword = cat(2,xlsch(ch.Modulation, 4, G, rv), {int8(zeros(0,1))});
    elseif ch.NumLayers == 5
        codeword = xlsch(ch.Modulation, 5, G, rv);
    elseif ch.NumLayers == 6
        codeword = xlsch(ch.Modulation, 6, G, rv);
    elseif ch.NumLayers == 7
        codeword = xlsch(ch.Modulation, 7, G, rv);
    else % ch.NumLayers == 8
        codeword = xlsch(ch.Modulation, 8, G, rv);
    end
end

% Process ReservedPRB
% Create a cell array of PRB level reservation objects for the set of PRB that
% the current PDSCH transmission should rate match around
% Each cell in the array is for a different PRB 'source' (channel/signal origin type)
function reservedPRB = processReservedPRB(ch, cfgObj, nch, ssbreserved, controlReservedPRB)
    
    bwps = cfgObj.BandwidthParts;
    bwpIdx = getBWPIdxByID(bwps, ch.BandwidthPartID);
    bwp = bwps{bwpIdx};

    % Initialise the output variable with the SS burst usage
    % Reserved PRB-level resources associated with SS burst
    reservedPRB = {ssbreserved{bwpIdx}};
    
    % Turn reserved CORESET indices into reserved patterns
    % with nrPDSCHReservedConfig format
    numReservedCSET = numel(ch.ReservedCORESET);
    for csetIdx = 1:numReservedCSET

        % Expand and project CORESET into the BWP
        % Pattern representation is single vector of PRB across all symbols

        % Get a copy of the CORESET configuration
        % init for codegen;
        propsCSET = {'CORESETID', 'Duration', 'FrequencyResources','RBOffset'};
        for idx = 1:length(propsCSET)
            cs.(propsCSET{idx}) = cfgObj.CORESET{1}.(propsCSET{idx});
        end
        coder.varsize('coreset.FrequencyResources',[1 45],[0 1]);
        coder.varsize('coreset.RBOffset',[1 1],[1 1]);
        % nrDLCarrierConfig ensures csetID is linked to an existing CORESET
        for idx = 1:numel(cfgObj.CORESET)
            if cfgObj.CORESET{idx}.CORESETID == ch.ReservedCORESET(csetIdx)
                for idx2 = 1:length(propsCSET)
                    cs.(propsCSET{idx2}) = cfgObj.CORESET{idx}.(propsCSET{idx2});
                end
            end
        end
        % nrDLCarrierConfig already guarantees that a CORESET exists with each ID in ReservedCORESET

        coder.varsize('rmallocatedsymbols');
        numSS = numel(cfgObj.SearchSpaces);
        for idx = 1:numSS
            if cs.CORESETID ~= cfgObj.SearchSpaces{idx}.CORESETID
                continue;
            end
            searchSpace = cfgObj.SearchSpaces{idx};

            % Identify all symbols included in this CORESET sequence
            rmallocatedsymbols = nr5g.internal.wavegen.getCORESETSymbols(cfgObj.NumSubframes,bwp,cs,searchSpace,cfgObj.InitialNSubframe);
            
            % PRB occupied by the CORESET in BWP
            allocatedPRB = nr5g.internal.pdcch.getCORESETPRB(cs,bwp.NStartBWP);

            % Check that the associated PRB set fits within the associated BWP NRB
            coder.internal.errorIf(max(allocatedPRB) >= bwp.NSizeBWP, ...
                'nr5g:nrWaveformGenerator:InvalidReservedCORESETInBWP', nch,cs.CORESETID,max(allocatedPRB),bwp.NSizeBWP,ch.BandwidthPartID);

            % Create reserved configuration object and push it onto the copy of the PDSCH parameters
            thisRsv = nrPDSCHReservedConfig('SymbolSet',rmallocatedsymbols(:)); % SymbolSet needs to be defined at construction time for codegen
            thisRsv.PRBSet = allocatedPRB; % Reserved PRB (0-based indices, defined as a vector)
            thisRsv.Period = [];           % Total number of slots in the pattern period (empty means don't cyclically repeat)
            reservedPRB{end + 1} = thisRsv;
        end
    end

    % Process the ReservedPRB property associated with the PDSCH configuration
    symbperslot = nr5g.internal.wavegen.symbolsPerSlot(bwp);
    numReservedPRB = numel(ch.ReservedPRB);
    for idx = 1:numReservedPRB
        % Check that the associated PRB set fits within the associated BWP NRB
        coder.internal.errorIf(~isempty(ch.ReservedPRB{idx}.PRBSet) && max(ch.ReservedPRB{idx}.PRBSet, [], 'all') >= bwp.NSizeBWP, ...
            'nr5g:nrWaveformGenerator:InvalidReservedPRBInBWP', nch,bwp.NSizeBWP,ch.BandwidthPartID);

        % Need to combine allocated symbols and allocated slots into a single list
        rmallocslots = nr5g.internal.wavegen.expandbyperiod(0,ch.ReservedPRB{idx}.Period,cfgObj.NumSubframes,bwp.SubcarrierSpacing,cfgObj.InitialNSubframe);
        rmallocsymbols = nr5g.internal.wavegen.addRowAndColumn(symbperslot*rmallocslots, ch.ReservedPRB{idx}.SymbolSet'); % OFDM symbols associated with reserved PRB (0-based indices, spanning one or more slots)

        % Create reserved configuration object and push it onto the copy of the PDSCH parameters
        thisRsv = nrPDSCHReservedConfig('SymbolSet',rmallocsymbols(:)'); % SymbolSet needs to be defined at construction for codegen
        thisRsv.PRBSet = ch.ReservedPRB{idx}.PRBSet; % Reserved PRB (0-based indices, defined as a vector)
        thisRsv.Period = [];                         % Total number of slots in the pattern period (empty means no repetition)
        reservedPRB{end + 1} = thisRsv; %#ok<*AGROW>
    end
    
    % Process the ControlReservedPRB input
    % Add all of this input array to the list
    numReservedPRB = numel(controlReservedPRB);
    for idx = 1:numReservedPRB
        reservedPRB{end + 1} = controlReservedPRB{idx}; %#ok<*AGROW>
    end
    
end

% CDM lengths for a CSI-RS config object
function cdmLengths = getCSIRSCDMLengths(csirs,idx)
    
    if ~iscell(csirs.CDMType)
        CDMType = {csirs.CDMType};
    else
        CDMType = csirs.CDMType;
    end
    CDMTypeOpts = {'noCDM','fd-CDM2','CDM4','CDM8'};
    CDMLengthOpts = {[1 1],[2 1],[2 2],[2 4]};
    cdmLengths = CDMLengthOpts{strcmpi(CDMTypeOpts,CDMType{idx})};
    
end
    
% Preallocate a cell array with blank nrPDSCHReservedConfig objects for all control channel instances per PDSCH configuration
function pdschReserved = allocatePDSCHReservedConfigForControl(bwp,coreset,searchSpaces,pdcch,pdsch,numSubframes,initnsf)
    
    numPDSCH = numel(pdsch);
    numReservedConfigs = zeros(1,numPDSCH);  % Number of potential reservations needed for each PDSCH
    for nch = 1:numel(pdcch)
    
        % Get a copy of the current PDCCH channel parameters
        ch = pdcch{nch};

        % Only process the PDCCH configuration if it's enabled
        if ~ch.Enable
            continue;
        end

        % Get a working index from the channel's BWP ID
        bwpIdx = getBWPIdxByID(bwp, ch.BandwidthPartID);

        % Get a copy of the SS for this PDCCH sequence
        [~, ss] = nr5g.internal.wavegen.getCORESETAndSearchSpace(coreset, searchSpaces, ch);
    
        % Calculate slot numbers for the CORESET/search space monitoring
        % occasions expanding by the period across the waveform length
        scs = bwp{bwpIdx}.SubcarrierSpacing;
        controlSlots = nr5g.internal.wavegen.expandbyperiod(ch.SlotAllocation,ch.Period,numSubframes,scs,initnsf);
    
        % For each PDSCH sequence configuration, establish the maximum number of associated PDCCH instances (linked by the same RNTI and BWP)
        % whose resources might affect that PDSCH sequence. At this point, this is the total number of linked PDCCH, and does not account
        % for whether these PDCCH instances are actually in the same PDSCH slots or not
        % 
        % Count the number of nrPDSCHReservedConfig required for this PDCCH
        for pdschIdx = 1:numPDSCH
            dch = pdsch{pdschIdx};
            % Reserve PDCCH PRB resources for data channels sharing
            % BWP and RNTI. If the data channel already reserves the associated CORESET
            % then there is no need to reserve the contained PDCCH also
            if (dch.RNTI == ch.RNTI) && (dch.BandwidthPartID == ch.BandwidthPartID) ...
                    && ~isequal(dch.ReservedCORESET,ss.CORESETID)
                numReservedConfigs(pdschIdx) = numReservedConfigs(pdschIdx)+length(controlSlots);  % Add the number of PDCCH slots/instances in the waveform that affect this PDSCH 
            end
        end
    end
    
    % Allocate a ragged cell array with nrPDSCHReservedConfig per PDSCH
    % Each cell/PDSCH contains its own cell array of placeholder reservations for all the PDCCH instances
    % that can be associated with that PDSCH
    pdschReserved = cell(1,numPDSCH);
    for pdschIdx = 1:numPDSCH
        pdschReserved{pdschIdx} = repmat({nrPDSCHReservedConfig},1,numReservedConfigs(pdschIdx));  % Create a cell array of default reservation objects
    end
end

% Get CRC bits
function L = getCRC(A)
% CRC bits for UCI information for input length A.

    if A <= 11
        L = 0;
    elseif A <= 19
        L = 6;
    else % A > 19
        L = 11;
    end
end

% Remove extra info fields that do not apply to this PUCCH format and
% capture all resources info for this PUCCH sequence
function resources = getPUCCHResourceInfo(controlstore,formatPUCCH)
    initEmpty = zeros(0,1);
    if coder.target('MATLAB')
        if formatPUCCH == 0
            resources = rmfield(controlstore,{'UCI2Bits','DMRSIndices','DMRSSymbols'});
        else
            if ~isempty(controlstore) && iscell(controlstore(1).Codeword)
                % Update codeword field of the resource info output
                controlstore.Codeword = int8(initEmpty);
            end
            
            if formatPUCCH <= 2
                resources = rmfield(controlstore,{'SRBit','UCI2Bits'});
            else % Format 3 or 4
                resources = rmfield(controlstore,{'SRBit'});
            end
        end
    else
        resources = controlstore;
        numPUCCH = length(controlstore);
        for idx = 1:numPUCCH
            if formatPUCCH == 0
                resources(idx).DMRSIndices = uint32(initEmpty);
                resources(idx).DMRSSymbols = complex(initEmpty);
                resources(idx).UCI2Bits = int8(initEmpty);
            else
                resources(idx).SRBit = int8(initEmpty);
                if formatPUCCH <= 2
                    resources(idx).UCI2Bits = int8(initEmpty);
                end
            end
        end
    end
end

% Scalar expansion of power to match number of unique slots in a period.
function [powerVec,uniqueSlotsInPeriod] = expandPower(power,slotAllocation,period)
    % If the period is not empty, the unique slots in a period is the
    % number of slots less than the period. nr5g.internal.wavegen.expandbypower
    % makes sure there are no duplicated slot allocations.
    %
    % If the period is empty, the unique slots in a period is just the
    % allocated slots. 
    if ~isempty(period)
        uniqueSlotsInPeriod = nnz(slotAllocation<period);
    else
        uniqueSlotsInPeriod = numel(slotAllocation);
    end

    % If the power is a scalar, then expand out to the number of unique
    % slots in a period. Otherwise, return it as is. nr5g.internal.wavegen.EnablePower 
    % ensures that the number of powers is the same as the number of 
    % allocated slots in a period when power is not a scalar.
    if isscalar(power)
        powerVec = repmat(power,1,uniqueSlotsInPeriod(1));
    else
        powerVec = power;
    end
end

% Get the correct index for the power, based on the current slot and the unique slots in a period
function powerInd = getPowerIndex(idx,uniqueSlotsInPeriod)
    powerInd = mod(idx-1,uniqueSlotsInPeriod)+1;
end

function [antSym,antInd] = precodeAndMapAntenna(siz,sym,ind,W)

    if length(size(W))==2 && isequal(W,eye(size(W)))
        % When W is equal to eye(size(W)), due to the way how W is
        % constructed from AntennaMapping and PrecodingMatrix properties,
        % this suggests no effective precoding and antenna mapping, i.e.
        % default behavior
        proto = zeros(0,0,"like",sym);
        protoInd = cast(real(proto),underlyingType(ind));
        if isempty(sym)
            antSym = zeros(size(sym),"like",proto); % maintain size for empty inputs 
        else
            antSym = sym;
        end
        antInd = cast(ind,"like",protoInd);
    else
        [antSym,antInd] = nr5g.internal.precode(siz,sym,ind,W);
    end

end