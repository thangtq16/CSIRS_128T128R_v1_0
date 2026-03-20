function [CSIReport,CSIInfo] = nrCSIReportCSIRS(carrier,csirs,reportConfig,dmrsConfig,H,nVar)
% nrCSIReportCSIRS Downlink channel state infomration report calculation
%   [CSIREPORT,CSIINFO] = nrCSIReportCSIRS(CARRIER,CSIRS,REPORTCONFIG,DMRSCONFIG,H,NVAR)
%   returns the CSIREPORT structure  containing downlink channel rank
%   indicator (RI) value RI, corresponding precoding matrix indicator (PMI)
%   values PMISET and channel quality indicator(CQI) values CQI, as defined
%   in TS 38.214 Section 5.2.2.2, for the specified carrier configuration
%   CARRIER, CSI-RS configuration CSIRS, Channel State Information
%   Reference Signal (CSI-RS) reporting configuration object REPORTCONFIG,
%   for the specified demodulation reference signal (DM-RS) configuration
%   object for the physical downlink shared channel (PDSCH), estimated
%   channel information H and noise variance NVAR.The function also returns
%   the additional information about the signal to interference and noise
%   ratio (SINR) values SINRPERSUBBAND for each subband for selected CSI
%   configuration, Effective SINR EFFECTIVESINR and precoder matrix W as
%   fields of CSIINFO structure.
%   
%   CARRIER is a carrier specific configuration object, as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a>. Only
%   these object properties are relevant for this function:
%
%   SubcarrierSpacing - Subcarrier spacing in kHz
%   CyclicPrefix      - Cyclic prefix type
%   NSizeGrid         - Number of resource blocks (RBs) in
%                       carrier resource grid
%   NStartGrid        - Start of carrier resource grid relative to common
%                       resource block 0 (CRB 0)
%   NSlot             - Slot number
%   NFrame            - System frame number
%
%   CSIRS is a CSI-RS specific configuration object to specify one or more
%   CSI-RS resources, as described in <a href="matlab:help('nrCSIRSConfig')">nrCSIRSConfig</a>. Only these object
%   properties are relevant for this function:
%
%   CSIRSType           - Type of a CSI-RS resource {'ZP', 'NZP'}
%   CSIRSPeriod         - CSI-RS slot periodicity and offset
%   RowNumber           - Row number corresponding to a CSI-RS resource, as
%                         defined in TS 38.211 Table 7.4.1.5.3-1
%   Density             - CSI-RS resource frequency density
%   SymbolLocations     - Time-domain locations of a CSI-RS resource
%   SubcarrierLocations - Frequency-domain locations of a CSI-RS resource
%   NumRB               - Number of RBs allocated for a CSI-RS resource
%   RBOffset            - Starting RB index of CSI-RS allocation relative
%                         to carrier resource grid
%   For better results, it is recommended to use the same CSI-RS
%   resource(s) that are used for channel estimate, because the resource
%   elements (REs) that does not contain the CSI-RS may have the
%   interpolated channel estimates. Note that the CDM lengths and the
%   number of ports configured for all the CSI-RS resources must be same.
%
%   REPORTCONFIG is a CSI reporting configuration object. More information is
%   present in <a href="matlab:help('nrCSIReportConfig.m')">nrCSIReportConfig</a> function.
%
%   DMRSCONFIG is DM-RS configuration object for the PDSCH channel, as
%   described in <a href="matlab:help('nrPDSCHDMRSConfig')">nrPDSCHDMRSConfig</a>.
%   Only these object properties are relevant for this function:
%   DMRSLength              - DM-RS length (1 (default), 2)
%   DMRSAdditionalPosition  - Maximum number of DM-RS additional positions
%                             (0...3) (default 0)
%
%   H is the channel estimation matrix. It is of size
%   K-by-L-by-nRxAnts-by-Pcsirs, where K is the number of subcarriers in
%   the carrier resource grid, L is the number of orthogonal frequency
%   division multiplexing (OFDM) symbols spanning one slot, nRxAnts is the
%   number of receive antennas, and Pcsirs is the number of CSI-RS antenna
%   ports.
%
%   NVAR is covariance matrix of interference plus noise. I is of size
%   K-by-L-by-nRxAnts-by-nRxAnts.
% 
% 
%   CSIReport is an output structure with these fields:
%   RI - RI is a scalar which gives the best possible number of transmission
%       layers for the given channel and noise variance conditions. It is
%       in the range 1...8 when CodebookType is specified as
%       'type1SinglePanel' and in the range 1...4 when CodebookType is
%       specified as 'type1MultiPanel'.
%   PMISET - Precoding matrix indicator set is a structure representing the
%       set of PMI indices (1-based). The detailed explanation of PMISET is
%       available in the <a href="matlab:help('nr5g.internal.nrPMIReport')">nrPMIReport</a> function.
%   CQI - Channel quality indicator is a 2-dimensional matrix of size
%       1-by-numCodewords when CQI reporting mode is 'Wideband' and
%       (numSubbands+1)-by-numCodewords when CQI reporting mode is
%       'Subband'. numSubbands is the number of subbands and numCodewords
%       is the number of codewords. The first row consists of 'Wideband'
%       CQI value and if the CQI mode is 'Subband', the 'Wideband' CQI
%       value is followed by the subband differential CQI values for each
%       subband. The subband differential values are scalars ranging from 0
%       to 3 and these values are computed based on the offset level, as
%       defined in TS 38.214 Table 5.2.2.1-1, where subband CQI offset
%       level = subband CQI index - wideband CQI index.
%
%   CSIINFO is an output structure with these fields:
% 
%       W - W is precoder matrix corresponds to the PMISet in the codebook.
%
%       SINRPerSubband      - It represents the linear SINR values in each
%                             subband for all the codewords. It is a
%                             two-dimensional matrix of size
%                                 - 1-by-numCodewords, when CQI reporting 
%                                   mode is 'Wideband'
%                                 - (numSubbands +1)-by-numCodewords, when
%                                   CQI reporting mode is 'Subband'
%                             Each column of the matrix contains wideband
%                             SINR value (the average SINR value across all
%                             subbands) followed by the SINR values of each
%                             subband. The SINR value in each subband is
%                             taken as an average of SINR values of all the
%                             REs across the particular subband spanning
%                             one slot
%
%        EffectiveSINR      - Calculated Effective SINR value from SINR
%                             values using Link to System mapping function.
%
%   Example:
%   % This example demonstrates how to calculate RI for 4-by-4 MIMO
%   % scenario over TDL Channel.
%
%   % Carrier configuration
%   carrier = nrCarrierConfig;
%
%   % CSI-RS configuration
%   csirs = nrCSIRSConfig;
%   csirs.CSIRSType = {'nzp','nzp'};
%   csirs.RowNumber = [4 4];
%   csirs.Density = {'one','one'};
%   csirs.SubcarrierLocations = {0 0};
%   csirs.SymbolLocations = {0,5};
%   csirs.NumRB = 52;
%   csirs.RBOffset = 0;
%   csirs.CSIRSPeriod = [4 0];
%
%   % PDSCH configuration
%   pdsch = nrPDSCHConfig;
%
%   % Configure the number of transmit and receive antennas
%   nTxAnts = max(csirs.NumCSIRSPorts);
%   nRxAnts = nTxAnts;
%
%   % Configure the number of transmission layers
%   numLayers = 1;
%
%   % Generate CSI-RS indices and symbols
%   csirsInd = nrCSIRSIndices(carrier,csirs);
%   csirsSym = nrCSIRS(carrier,csirs);
%
%   % Resource element mapping
%   txGrid = nrResourceGrid(carrier,nTxAnts);
%   txGrid(csirsInd) = csirsSym;
%
%   % Get OFDM modulation related information
%   OFDMInfo = nrOFDMInfo(carrier);
%
%   % Perform OFDM modulation
%   txWaveform = nrOFDMModulate(carrier,txGrid);
%
%   % Configure the channel parameters.
%   channel = nrTDLChannel;
%   channel.NumTransmitAntennas = nTxAnts;
%   channel.NumReceiveAntennas = nRxAnts;
%   channel.SampleRate = OFDMInfo.SampleRate;
%   channel.DelayProfile = 'TDL-C';
%   channel.DelaySpread = 300e-9;
%   channel.MaximumDopplerShift = 5;
%   chInfo = info(channel);
%
%   % Calculate the maximum channel delay
%   maxChDelay = ceil(max(chInfo.PathDelays*OFDMInfo.SampleRate)) + chInfo.ChannelFilterDelay;
%
%   % Pass the time-domain waveform through the channel
%   rxWaveform = channel([txWaveform; zeros(maxChDelay,nTxAnts)]);
%
%   % Calculate the timing offset
%   offset = nrTimingEstimate(carrier,rxWaveform,csirsInd,csirsSym);
%
%   % Perform timing synchronization
%   rxWaveform = rxWaveform(1+offset:end,:);
%
%   % Add AWGN
%   SNRdB = 20;          % in dB
%   SNR = 10^(SNRdB/10); % Linear value
%   sigma = 1/(sqrt(2.0*channel.NumReceiveAntennas*double(OFDMInfo.Nfft)*SNR)); % Noise standard deviation
%   rng('default');
%   noise = sigma*complex(randn(size(rxWaveform)),randn(size(rxWaveform)));
%   rxWaveform = rxWaveform + noise;
%   rxGrid = nrOFDMDemodulate(carrier,rxWaveform);
%
%   % Perform the channel estimate
%   [H,nVar] = nrChannelEstimate(rxGrid,csirsInd,csirsSym,'CDMLengths',[2 1]);
%
%   % Configure the csi reporting configuration parameters
%   reportConfig = nrCSIReportConfig;
% 
%   [CSI,CSIInfo] = nrCSIReportCSIRS(carrier,csirs,reportConfig,pdsch.DMRS,H,nVar)

%   Copyright 2024 The MathWorks, Inc.

    if ~matlab.internal.feature('nrCSIReportCSIRS')
        % Throw error
        errorMsg = string(message("nr5g:nrCSIReportCSIRS:UndefinedFunction"));
        throwAsCaller(MException('MATLAB:UndefinedFunction',errorMsg));   
    end

    narginchk(6,6);
 
    % Validate inputs
    [reportConfig,csirsInd] = nr5g.internal.validateCSIInputs(carrier,csirs,reportConfig,dmrsConfig,H,nVar);

    % Calculate the number of subbands and size of each subband for the
    % given configuration
    PMISubbandInfo = nr5g.internal.getPMISubbandInfo(carrier,reportConfig);

    % Get the number of CSI-RS ports and receive antennas from the
    % dimensions of the channel estimate
    Pcsirs = size(H,4);
    nRxAnts = size(H,3);

    % Calculate the maximum possible transmission rank according to
    % codebook type
    if strcmpi(reportConfig.CodebookType,'Type1SinglePanel')
        % Maximum possible rank is 8 for Type I single-panel codebooks, as
        % defined in TS 38.214 Section 5.2.2.2.1
        maxRank = min([nRxAnts Pcsirs 8]);
    elseif strcmpi(reportConfig.CodebookType,'Type2') ||...
            (strcmpi(reportConfig.CodebookType,'eType2') && any(reportConfig.ParameterCombination == [7 8]))
        % Maximum possible rank is 2 for:
        % - Type II codebooks, as defined in TS 38.214 Section 5.2.2.2.3
        % - Enhanced type II codebooks with parameter combination value
        %   as one of {7, 8}, as defined in TS 38.214 Table 5.2.2.2.5-1
        maxRank = min(nRxAnts,2);
    else
        % Maximum possible rank is 4 for:
        % - Type I multi-panel codebooks, as defined in TS 38.214 Section 5.2.2.2.2
        % - Enhanced type II codebooks with parameter combination value in
        %   the range 1:6, as defined in TS 38.214 Table 5.2.2.2.5-1
        maxRank = min(nRxAnts,4);
    end

    % Check the rank indicator restriction parameter and derive the
    % ranks that are not restricted from usage
    if(~isempty(reportConfig.RIRestriction))
        unRestrictedRanks = find(reportConfig.RIRestriction);
        validRanks = intersect(unRestrictedRanks,1:maxRank);
    else
        validRanks = 1:maxRank;
    end

    % Initialize outputs
    [CSIReport,CSIInfo] = initOutputs(reportConfig,PMISubbandInfo);

    if ~isempty(validRanks) && ~isempty(csirsInd)
        [CSIReport,CSIInfo] = getCSIReport(carrier,csirs,reportConfig,dmrsConfig,H,nVar,validRanks,PMISubbandInfo);
    end
end

% Selection of rank indicator based on maximizing spectral efficiency
function [CSI,CSIInfo] = getCSIReport(carrier,csirs,reportConfig,dmrsConfig,H,nVar,validRanks,PMISubbandInfo)
    
    % Get the spectral Efficiency from the CQI table
    persistent SpecEffArray tableName;
    if (isempty(SpecEffArray)||(~strcmpi(tableName,reportConfig.CQITable)))
        tableName = reportConfig.CQITable;
        cqiTableClass = nrCQITables;
        TableCell = {'Table1','Table2','Table3','Table4'};        
        SpecEffArray= cqiTableClass.(['CQI' TableCell{strcmpi(tableName,TableCell)}]).SpectralEfficiency;
    end
   
    % Initialize outputs
    [CSI,CSIInfo] = initOutputs(reportConfig,PMISubbandInfo);

    % For each valid rank, select the best CQI. Then, find the rank
    % that maximizes modulation and coding efficiency
    maxRank = max(validRanks);
    efficiency = NaN(maxRank,1);
    for rank = validRanks
        % Determine the CQI and PMI for the current rank
        [cqi{rank},pmi(rank),cqiInfo(rank),pmiInfo(rank)] = nr5g.internal.nrCQIReport(carrier,csirs,reportConfig,dmrsConfig,rank,H,nVar); %#ok<AGROW>
    
        % Get wideband CQI
        cqiWideband = cqi{rank}(1,:);
    
        % If the wideband CQI is appropriate, calculate the efficiency
        if all(cqiWideband ~= 0)
            if ~any(isnan(cqiWideband))
                % Calculate throughput-related metric using number of
                % layers, code rate and modulation, and estimated BLER
                blerWideband = cqiInfo(rank).TransportBLER(1,:);
                ncw = numel(cqiWideband);
                cwLayers = floor((rank + (0:ncw-1)) / ncw);
                SpecEffValue = SpecEffArray(cqiWideband+1);
                eff = cwLayers .* (1 - blerWideband) * SpecEffValue;
                efficiency(rank) = eff;
            end
        else
            efficiency(rank) = 0;
        end
    end
    
    % Return the rank that maximizes the spectral efficiency and the
    % corresponding PMI.
    [maxEff,maxEffIndx] = max(efficiency);
    if ~isnan(maxEff)
        CSI.RI = maxEffIndx;
        CSI.PMISet = pmi(CSI.RI);
        CSI.CQI = cqi{CSI.RI};
        CSIInfo.W = pmiInfo(CSI.RI).W;
        CSIInfo.SINRPerSubband = cqiInfo(CSI.RI).SINRPerSubbandPerCW;
        CSIInfo.EffectiveSINR = cqiInfo(CSI.RI).EffectiveSINR;
    end

end

function [CSI,CSIInfo] = initOutputs(reportConfig,PMISubbandInfo)
%   [CSI,CSIInfo] = initOutputs(REPORTCONFIG,PMISUBBANDINFO) initializes the
%   rank and PMI set values with NaNs.

    CSI.RI = NaN;
    isType1SinglePanel = strcmpi(reportConfig.CodebookType,'Type1SinglePanel');
    isType2 = strcmpi(reportConfig.CodebookType,'Type2');
    isEnhType2 = strcmpi(reportConfig.CodebookType,'eType2');
    % Generate PMI set and output information structure with NaNs
    if isType2
        numI1Indices = 3 + (1 + 2*reportConfig.NumberOfBeams);
        numI2Columns = (1+reportConfig.SubbandAmplitude);
        numI2Rows = 2*reportConfig.NumberOfBeams;
        CSI.PMISet.i1 = NaN(1,numI1Indices);
        CSI.PMISet.i2 = NaN(numI2Rows,numI2Columns,PMISubbandInfo.NumSubbands);        
    elseif isEnhType2
        pv = reportConfig.Tables.EnhancedType2Configurations{reportConfig.ParameterCombination,4};
        Mv = ceil(pv*PMISubbandInfo.NumSubbands/reportConfig.NumberOfPMISubbandsPerCQISubband);
        numI1Indices = 4 + (1 + 2*reportConfig.NumberOfBeams*Mv + 1);
        numI2Values = (2 + 2*reportConfig.NumberOfBeams*Mv + 2*reportConfig.NumberOfBeams*Mv);
        CSI.PMISet.i1 = NaN(1,numI1Indices);
        CSI.PMISet.i2 = NaN(1,numI2Values,PMISubbandInfo.NumSubbands);        
    elseif isType1SinglePanel
        CSI.PMISet.i1 = NaN(1,3);
        CSI.PMISet.i2 = NaN(1,PMISubbandInfo.NumSubbands);
    else
        CSI.PMISet.i1 = NaN(1,6);
        CSI.PMISet.i2 = NaN(3,PMISubbandInfo.NumSubbands);
    end
    % Initialize structure for CSIInfo
    CSIInfo = struct('W',[],'SINRPerSubband',[],'EffectiveSINR',[]);

end