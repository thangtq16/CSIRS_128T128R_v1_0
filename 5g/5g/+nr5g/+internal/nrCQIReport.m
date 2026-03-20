function [CQI,PMISet,CQIInfo,PMIInfo] = nrCQIReport(carrier,csirs,reportConfig,dmrsConfig,nLayers,H,nVar)
% nrCQIReport PDSCH Channel quality indicator calculation
%   [CQI,PMISET,CQIINFO,PMIINFO] = nrCQIReport(CARRIER,CSIRS,REPORTCONFIG,DMRSCONFIG,NLAYERS,H,NVAR)
%   returns channel quality indicator (CQI) values CQI and precoding matrix
%   indicator (PMI) values PMISET, as defined in TS 38.214 Section 5.2.2.2,
%   for the specified carrier configuration CARRIER, CSI-RS configuration
%   CSIRS, channel state information (CSI) reporting configuration
%   REPORTCONFIG, number of transmission layers NLAYERS, and estimated
%   channel information H. The function also returns the additional
%   information about the signal to interference and noise ratio (SINR)
%   values that are used for the CQI computation and PMI computation.
%
%   CARRIER is a carrier specific configuration object, as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a>. Only these object properties are relevant for this
%   function:
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
%   REPORTCONFIG is a CSI-RS report configuration object to specify about
%   reporting configuraiton, as described <a href="matlab:help('nrCSIRSReportConfig')">nrCSIRSReportConfig</a>.
%
%   The detailed explanation of the CodebookSubsetRestriction field is
%   present in <a href="matlab:help('nr5g.internal.nrDLPMISelect')">nr5g.internal.nrDLPMISelect</a> function.
%
%   NLAYERS is a scalar representing the number of transmission layers.
%   When CodebookType is specified as 'Type1SinglePanel', its value must be
%   in the range of 1...8. When CodebookType is specified as
%   'Type1MultiPanel', its value must be in the range of 1...4. When
%   CodebookType is specified as 'Type2', its value must be in the range of
%   1...2. When CodebookType is specified as 'eType2', its value must be in
%   the range of 1...4.
%
%   H is the channel estimation matrix. It is of size
%   K-by-L-by-nRxAnts-by-Pcsirs, where K is the number of subcarriers in
%   the carrier resource grid, L is the number of orthogonal frequency
%   division multiplexing (OFDM) symbols spanning one slot, nRxAnts is the
%   number of receive antennas, and Pcsirs is the number of CSI-RS antenna
%   ports. Note that the number of transmission layers provided must be
%   less than or equal to min(nRxAnts,Pcsirs).
%
%   CQI output is a 2-dimensional matrix of size 1-by-numCodewords when CQI
%   reporting mode is 'Wideband' and (numSubbands+1)-by-numCodewords when
%   CQI reporting mode is 'Subband'. numSubbands is the number of subbands
%   and numCodewords is the number of codewords. The first row consists of
%   'Wideband' CQI value and if the CQI mode is 'Subband', the 'Wideband'
%   CQI value is followed by the subband differential CQI values for each
%   subband. The subband differential values are scalars ranging from 0 to
%   3 and these values are computed based on the offset level, as defined
%   in TS 38.214 Table 5.2.2.1-1, where
%   subband CQI offset level = subband CQI index - wideband CQI index.
%
%   Note that when the PRGBundleSize property in the reportConfig is
%   configured as other than empty, it is assumed that the report quantity
%   as reported by the higher layers is 'cri-RI-i1-CQI'. In this case the
%   SINR values for the CQI computation are chosen based on the i1 values
%   reported in PMISet and a valid random i2 value from all the reported i2
%   values in the PMISet. In this case, i2 values reported in the PMISet
%   correspond to each PRG. When CQI reporting mode is 'Wideband', one i2
%   value is chosen randomly, for the entire BWP, from the set of i2 values
%   of all PRGs. When CQI reporting mode is subband, one i2 value is chosen
%   randomly, for each subband, from the set of PRGs that span the
%   particular subband. Considering this set of i2 values for indexing, the
%   corresponding SINR values are used for CQI computation.
%
%   PMISET output is a structure representing the set of PMI indices
%   (1-based). The detailed explanation of PMISET is available in the
%   <a href="matlab:help('nr5g.internal.nrDLPMISelect')">nrDLPMISelect</a> 
%   function.
%
%   CQIINFO is an output structure for the CQI information with these
%   fields:
%   SINRPerSubbandPerCW - It represents the linear SINR values in each
%                         subband for all the codewords. It is a
%                         two-dimensional matrix of size
%                            - 1-by-numCodewords, when CQI reporting mode
%                              is 'Wideband'
%                            - (numSubbands + 1)-by-numCodewords, when
%                              CQI reporting mode is 'Subband'
%                         Each column of the matrix contains wideband SINR
%                         value (the average SINR value across all
%                         subbands) followed by the SINR values of each
%                         subband. The SINR value in each subband is taken
%                         as an average of SINR values of all the REs
%                         across the particular subband spanning one slot
%   SINRPerRBPerCW      - It represents the linear SINR values in each
%                         RB for all the codewords. It is a
%                         three-dimensional matrix of size
%                         NSizeBWP-by-L-by-numCodewords. The SINR value in
%                         each RB is taken as an average of SINR values of
%                         all the REs across the RB spanning one slot
%   SubbandCQI          - It represents the subband CQI values. It is a
%                         two-dimensional matrix of size
%                            - 1-by-numCodewords, when CQI reporting mode
%                              is 'Wideband' 
%                            - (numSubbands + 1)-by-numCodewords, when
%                              CQI reporting mode is 'Subband'
%                         Each column of the matrix contains the absolute
%                         CQI value of wideband followed by the absolute
%                         CQI values corresponding to each subband
%   TransportBLER       - Estimated transport block error rate (BLER) for
%                         each element in SubbandCQI
%
%   Note that the CQI output and all the fields of CQIINFO are returned as
%   NaNs for these cases:
%      - When CSI-RS is not present in the operating slot or in the BWP
%      - When the reported PMISet is all NaNs
%   Also note that the subband differential CQI value or SubbandCQI value
%   is reported as NaNs in the subbands where CSI-RS is not present.
%
%   PMIINFO is an output structure with the information about SINR values,
%   codebook, and the precoding matrix. The detailed explanation for
%   PMIINFO is given under INFO output in the <a href="matlab:help('nr5g.internal.nrDLPMISelect')">nrDLPMISelect</a> function.
%
%
%   CQI by definition, is a scalar value ranging from 0 to 15 which
%   indicates highest modulation and coding scheme (MCS), suitable for the
%   downlink transmission in order to achieve the required BLER condition.
%
%   According to TS 38.214 Section 5.2.2.1, the user equipment (UE) reports
%   highest CQI index which satisfies the condition where a single physical
%   downlink shared channel (PDSCH) transport block with a combination of
%   modulation scheme, target code rate and transport block size
%   corresponding to the CQI index, and occupying a group of downlink PRBs
%   termed the CSI reference resource (as defined in TS 38.214 Section
%   5.2.2.5), could be received with a transport block error probability
%   not exceeding:
%      -   0.1, when the higher layer parameter cqi-Table in
%          CSI-ReportConfig configures 'table1' (corresponding to TS 38.214
%          Table 5.2.2.1-2), or 'table2' (corresponding to TS 38.214 Table
%          5.2.2.1-3)
%      -   0.00001, when the higher layer parameter cqi-Table in
%          CSI-ReportConfig configures 'table3' (corresponding to TS 38.214
%          Table 5.2.2.1-4)
%
%   The CQI indices and their interpretations are given in TS 38.214 Table
%   5.2.2.1-2 or TS 38.214 Table 5.2.2.1-4, for reporting CQI based on
%   QPSK, 16QAM, 64QAM. The CQI indices and their interpretations are given
%   in TS 38.214 Table 5.2.2.1-3, for reporting CQI based on QPSK, 16QAM,
%   64QAM, 256QAM and 1024 QAM.
%
%   % Example:
%   % This example demonstrates how to calculate CQI for the 4-by-4 MIMO
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
%   % Configure the number of transmit and receive antennas
%   nTxAnts = max(csirs.NumCSIRSPorts);
%   nRxAnts = nTxAnts;
%
%   % Configure the number of transmission layers
%   numLayers = 1;
%   
%   % PDSCH configuration
%   pdsch = nrPDSCHConfig;
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
%   % Configure the required CQI configuration parameters
%   reportConfig = nrCSIReportConfig;
%   reportConfig.NStartBWP = 2;
%   reportConfig.NSizeBWP = 40;
%   reportConfig.PanelDimensions = [1 2 1];
%   reportConfig.CodebookMode = 1;
%   reportConfig.PMIFormatIndicator = 'wideband';
%   reportConfig.CodebookSubsetRestriction = [];
%   reportConfig.CQIFormatIndicator = 'subband';
%   reportConfig.SubbandSize = 4;
%   [CQI,PMISet,CQIInfo,PMIInfo] = nr5g.internal.nrCQIReport(carrier,csirs,reportConfig,pdsch.DMRS,numLayers,H,nVar)

%   Copyright 2024 The MathWorks, Inc.
    
    narginchk(7,7);

    % Validate inputs
    numCSIRSPorts = csirs.NumCSIRSPorts(1);
    nr5g.internal.validateLayerDependentParams(carrier,reportConfig,numCSIRSPorts,H,nLayers);
    nStartGrid = double(carrier.NStartGrid);
    nSizeGrid = double(carrier.NSizeGrid);
    if isempty(reportConfig.NSizeBWP)
        nSizeBWP = nSizeGrid;
    else
        nSizeBWP = double(reportConfig.NSizeBWP);
    end
    if isempty(reportConfig.NStartBWP)
        nStartBWP = nStartGrid;
    else
        nStartBWP = double(reportConfig.NStartBWP);
    end
 
    % Calculate the number of subbands and size of each subband for the
    % given CQI configuration and the PMI configuration. If PRGSize
    % parameter is present and configured with a value other than empty,
    % the PMISubbandInfo consists of PRG related information, otherwise it
    % contains PMI subbands related information
    [CQISubbandInfo,PMISubbandInfo] = getDownlinkCSISubbandInfo(reportConfig,nStartBWP,nSizeBWP);

    % Calculate the number of codewords for the given number of layers. For
    % number of layers greater than 4, there are two codewords, else one
    % codeword
    numCodewords = ceil(nLayers/4);

    % Calculate the start of BWP relative to the carrier
    bwpStart = nStartBWP - carrier.NStartGrid;

    % Calculate the SINR and CQI values
    csirsInd = nr5g.internal.getCSIRSIndicesForCSI(carrier,csirs);
    csirsIndSubs_kTemp = csirsInd(:,1);
    csirsIndSubs_lTemp = csirsInd(:,2);
    % Consider the CSI-RS indices present only in the BWP
    indInBWP = (csirsIndSubs_kTemp >= bwpStart*12 + 1) & csirsIndSubs_kTemp <= (bwpStart + nSizeBWP)*12;
    csirsIndSubs_k = csirsIndSubs_kTemp(indInBWP);
    csirsIndSubs_l = csirsIndSubs_lTemp(indInBWP);

    % Make the CSI-RS subscripts relative to BWP
    csirsIndSubs_k = csirsIndSubs_k - bwpStart*12;
    % Get the PMI and SINR values from the PMI selection function
    [PMISetForCQI,PMIInfoForCQI] = nr5g.internal.nrPMIReport(carrier,csirs,reportConfig,nLayers,H,nVar);

    if (isempty(csirsIndSubs_k) || (nVar == 0) || (all(isnan(PMISetForCQI.i1)) && all(isnan(PMISetForCQI.i2(:)))))
        if CQISubbandInfo.NumSubbands == 1
            % Convert the numSubbands to 0 to report only the wideband CQI
            % index in case of wideband mode
            numSubbands = 0;
        else
            numSubbands = CQISubbandInfo.NumSubbands;
        end
        % Report CQI and the CQI information structure parameters as NaN
        CQI = NaN(numSubbands+1,numCodewords);
        CQIInfo.SINRPerSubbandPerCW = NaN(numSubbands+1,numCodewords);
        CQIInfo.SINRPerRBPerCW = NaN(nSizeBWP,carrier.SymbolsPerSlot,numCodewords);
        CQIInfo.SubbandCQI = NaN(numSubbands+1,numCodewords);
        CQIInfo.EffectiveSINRDB = NaN(numSubbands+1,numCodewords);
        PMISet = PMISetForCQI;
        PMIInfo = PMIInfoForCQI;
        return;
    end

    sinrPerREPMI = PMIInfoForCQI.SINRPerREPMI;
    if any(strcmpi(reportConfig.CodebookType,{'Type2','eType2'}))
        SINRperSubband = PMIInfoForCQI.SINRPerSubband;
        if strcmpi(reportConfig.CQIFormatIndicator,'Subband') && strcmpi(reportConfig.PMIFormatIndicator,'Wideband')
            SINRperSubband = getSubbandSINR(sinrPerREPMI,CQISubbandInfo,csirsIndSubs_k);
        end

        % Get the SINR values corresponding to the PMISet in RB level
        % granularity. These values are not directly used for CQI
        % computation. These are just for information purpose
        SINRsperRBperCW = getSINRperRB(sinrPerREPMI,csirsIndSubs_k,csirsIndSubs_l,nSizeBWP,carrier.SymbolsPerSlot);
        PMISet = PMISetForCQI;
        PMIInfo = PMIInfoForCQI;
    else
        SINRperSubband = NaN(CQISubbandInfo.NumSubbands,nLayers);
        if  ~isempty(reportConfig.PRGBundleSize) && strcmpi(reportConfig.CodebookType,'type1SinglePanel')
            % When PRGSize field is configured as other than empty, the CQI
            % computation is done by choosing one random i2 value from all
            % the i2 values corresponding to the PRGs spanning the subband
            % or the wideband based on the CQI mode, as defined in TS
            % 38.214 Section 5.2.1.4.2
            rng(0); % Set RNG state for repeatability
            randomi2 = zeros(1,CQISubbandInfo.NumSubbands);
            if strcmpi(reportConfig.CQIFormatIndicator,'Subband')
                % Map the PRGs to subbands
                index = 1;
                thisSubbandSize = CQISubbandInfo.SubbandSizes(1);
                % Get the starting position of each PRG with respect to the
                % current subband. It helps to compute the number of PRGs
                % in the respective subband
                startPRG = ones(1,CQISubbandInfo.NumSubbands+1);
                for prgIdx = 1:numel(PMISubbandInfo.SubbandSizes)
                    if (thisSubbandSize - PMISubbandInfo.SubbandSizes(prgIdx) == 0) && (index < CQISubbandInfo.NumSubbands)
                        % Go to the next subband index and replace the
                        % current subband size
                        index = index + 1;
                        thisSubbandSize = CQISubbandInfo.SubbandSizes(index);
                        % Mark the corresponding PRG index as the start of
                        % subband
                        startPRG(index) = prgIdx + 1;
                    else
                        thisSubbandSize = thisSubbandSize - PMISubbandInfo.SubbandSizes(prgIdx);
                    end
                end
                % Append the total number of PRGs + 1 value to the
                % startPRG vector. The value points to the last PRG at the
                % end of the BWP, to know the number of PRGs in the last
                % subband
                startPRG(index+1) = PMISubbandInfo.NumSubbands+1;
                % Loop over all the subbands and choose an i2 value
                % randomly from the i2 values corresponding to all the PRGs
                % spanning each subband
                for idx = 2:numel(startPRG)
                    i2Set = PMISetForCQI.i2(startPRG(idx-1):startPRG(idx)-1);
                    randomi2(idx-1) = i2Set(randi(numel(i2Set)));
                    if ~isnan(randomi2(idx-1))
                        SINRperSubband(idx-1,:) = mean(PMIInfoForCQI.SINRPerSubband(startPRG(idx-1):startPRG(idx)-1,:,randomi2(idx-1),PMISetForCQI.i1(1),PMISetForCQI.i1(2),PMISetForCQI.i1(3)),'omitnan');
                    end
                end
                SINRsperRECQI = getSINRperRECQI(PMIInfoForCQI.SINRPerRE,struct('i1',PMISetForCQI.i1,'i2',randomi2),CQISubbandInfo.SubbandSizes,csirsIndSubs_k);
            else
                % Choose an i2 value randomly from the i2 values other than
                % NaNs corresponding to all the PRGs in the BWP
                i2Set = PMISetForCQI.i2(~isnan(PMISetForCQI.i2));
                randomi2 = i2Set(randi(numel(i2Set)));
                SINRperSubband(:,:) = mean(PMIInfoForCQI.SINRPerSubband(:,:,randomi2,PMISetForCQI.i1(1),PMISetForCQI.i1(2),PMISetForCQI.i1(3)),'omitnan');
                SINRsperRECQI = PMIInfoForCQI.SINRPerRE(:,:,randomi2,PMISetForCQI.i1(1),PMISetForCQI.i1(2),PMISetForCQI.i1(3));
            end
            % Get the SINR values in RB level granularity, based on the
            % random i2 values selected. These values are not directly used
            % for CQI computation. These are just for information purpose
            SINRsperRBperCW = getSINRperRB(SINRsperRECQI,csirsIndSubs_k,csirsIndSubs_l,nSizeBWP,carrier.SymbolsPerSlot);
            % Regenerate the PMI values with PRGBundleSize as []
            reportConfigTmp = reportConfig;
            reportConfigTmp.PRGBundleSize = [];
            [PMISet,PMIInfo] = nr5g.internal.nrPMIReport(carrier,csirs,reportConfigTmp,nLayers,H,nVar);
        else
            % If PRGSize is not configured, the output from PMI selection
            % function is either in wideband or subband level granularity
            % based on the PMIFormatIndicator

            % Get the SINR values corresponding to the PMISet in RB level
            % granularity. These values are not directly used for CQI
            % computation. These are just for information purpose
            SINRsperRBperCW = getSINRperRB(sinrPerREPMI,csirsIndSubs_k,csirsIndSubs_l,nSizeBWP,carrier.SymbolsPerSlot);

            % Deduce the SINR values for the CQI computation based on the
            % CQI mode, as the SINRPerSubband field in the PMI information
            % output has the SINR values according to the PMIFormatIndicator
            if strcmpi(reportConfig.PMIFormatIndicator,'Wideband')
                % If PMI mode is 'Wideband', only one i2 value is reported
                % and the SINR values are obtained for the entire BWP in
                % the SINRPerSubband field of PMIInfo output. In this case
                % compute the SINR values corresponding to subband or
                % wideband based on the CQI mode
                SINRperSubband = getSubbandSINR(sinrPerREPMI,CQISubbandInfo,csirsIndSubs_k);
            else
                % If PMI mode is 'Subband', when codebook type is specified
                % as 'Type1SinglePanel', one i2 value is reported per
                % subband and when codebook type is specified as
                % 'Type1MultiPanel', a set of three indices [i20; i21; i22]
                % are reported per subband. The SINR values are obtained in
                % subband level granularity from PMI selection function.
                % Extract the SINR values accordingly
                for subbandIdx = 1:size(PMISetForCQI.i2,2)
                    if ~any(isnan(PMISetForCQI.i2(:,subbandIdx)))
                        if strcmpi(reportConfig.CodebookType,'Type1MultiPanel')
                            SINRperSubband(subbandIdx,:) = PMIInfoForCQI.SINRPerSubband(subbandIdx,:,PMISetForCQI.i2(1,subbandIdx),PMISetForCQI.i2(2,subbandIdx),PMISetForCQI.i2(3,subbandIdx),PMISetForCQI.i1(1),PMISetForCQI.i1(2),PMISetForCQI.i1(3),PMISetForCQI.i1(4),PMISetForCQI.i1(5),PMISetForCQI.i1(6));
                        else
                            SINRperSubband(subbandIdx,:) = PMIInfoForCQI.SINRPerSubband(subbandIdx,:,PMISetForCQI.i2(subbandIdx),PMISetForCQI.i1(1),PMISetForCQI.i1(2),PMISetForCQI.i1(3));
                        end
                    end
                end
            end
            PMISet = PMISetForCQI;
            PMIInfo = PMIInfoForCQI;
        end
    end
    % Get SINR per subband
    SINRperSubbandperCW = zeros(CQISubbandInfo.NumSubbands,numCodewords);
    for subbandIdx = 1:CQISubbandInfo.NumSubbands
        % Get the SINR values per layer and calculate the SINR values
        % corresponding to each codeword
        layerSINRs = squeeze(SINRperSubband(subbandIdx,:));

        if ~any(isnan(layerSINRs))
            codewordSINRs = cellfun(@sum,nrLayerDemap(layerSINRs));
        else
            % If the linear SINR values of the codeword are NaNs, which
            % implies, there are no CSI-RS resources in the current
            % subband. So, the SINR values for the codewords are
            % considered as NaNs for the particular subband
            codewordSINRs = NaN(1,numCodewords);
        end
        SINRperSubbandperCW(subbandIdx,:) = codewordSINRs;
    end

    if size(SINRperSubbandperCW,1) > 1
        % Compute the wideband SINR value as a mean of the subband SINRs,
        % if either CQI or PMI are configured in subband mode
        SINRperSubbandperCW = [mean(SINRperSubbandperCW,1,'omitnan'); SINRperSubbandperCW];
    end

    BLERForAllSubbands = zeros(CQISubbandInfo.NumSubbands,numCodewords);

    %Initialize L2SM for CQISelection calculation
    l2sm = nr5g.internal.L2SM.initialize(carrier);

    % Get CSI reference resource for CQI selection, as defined in TS
    % 38.214 Section 5.2.2.5
    [pdsch,pdschExt] = nrCSIReferenceResource(carrier,reportConfig,nLayers,dmrsConfig,nStartBWP,nSizeBWP);

    % Get CQI, effective SINR and estimated BLER per subband
    SINRperSubbandperCW = zeros(CQISubbandInfo.NumSubbands,numCodewords);
    EffectiveSINRDB = zeros(CQISubbandInfo.NumSubbands,numCodewords);
    CQIForAllSubbands = NaN(CQISubbandInfo.NumSubbands,numCodewords);
    subbandStart = 0;
    for subbandIdx = 1:CQISubbandInfo.NumSubbands
        % Subcarrier indices for this subband
        subbandInd = (csirsIndSubs_k>subbandStart*12) & (csirsIndSubs_k<(subbandStart+ CQISubbandInfo.SubbandSizes(subbandIdx))*12+1);
        % Compute CQI, effective SINR and estimated BLER
        [l2sm,CQIForAllSubbands(subbandIdx,:),SINRperSubbandperCW(subbandIdx,:),BLERForAllSubbands(subbandIdx,:),EffectiveSINRDB(subbandIdx,:)] = ...
            cqiSelect(l2sm,carrier,pdsch,pdschExt.XOverhead,PMIInfoForCQI.SINRPerREPMI(subbandInd,:,:),reportConfig.CQITable);
        % Compute the starting position of next subband
        subbandStart = subbandStart + CQISubbandInfo.SubbandSizes(subbandIdx);
    end

    if size(SINRperSubbandperCW,1) > 1
        % Compute the wideband CQI, effective SINR and estimated BLER, if
        % either CQI or PMI are configured in subband mode
        [l2sm,wbCQI,wbEffectiveSINR,wbBLER,wbEffectiveSINRdB] = cqiSelect(l2sm,carrier,pdsch,pdschExt.XOverhead,PMIInfoForCQI.SINRPerREPMI,reportConfig.CQITable);
        CQIForAllSubbands = [wbCQI; CQIForAllSubbands];
        BLERForAllSubbands = [wbBLER; BLERForAllSubbands];
        SINRperSubbandperCW = [wbEffectiveSINR; SINRperSubbandperCW];
        EffectiveSINRDB = [wbEffectiveSINRdB;EffectiveSINRDB];
    end

    % Compute the subband differential CQI value in case of subband
    % mode
    if strcmpi(reportConfig.CQIFormatIndicator,'Subband')
        % Map the subband CQI values to their subband differential
        % value as defined in TS 38.214 Table 5.2.2.1-1. According to
        % this table, a subband differential CQI value is reported for
        % each subband based on the offset level, where the offset
        % level = subband CQI index - wideband CQI index
        CQIdiff = CQIForAllSubbands(2:end,:) - CQIForAllSubbands(1,:);

        % If the CQI value in any subband is NaN, consider the
        % corresponding subband differential CQI as NaN. It indicates
        % that there are no CSI-RS resources present in that particular
        % subband
        CQIOffset(isnan(CQIdiff)) = NaN;
        CQIOffset(CQIdiff == 0) = 0;
        CQIOffset(CQIdiff == 1) = 1;
        CQIOffset(CQIdiff >= 2) = 2;
        CQIOffset(CQIdiff <= -1) = 3;

        CQIOffset = reshape(CQIOffset,[],numCodewords);
        % Form an output CQI array to include wideband CQI value
        % followed by subband differential values
        CQI = [CQIForAllSubbands(1,:); CQIOffset];
    else
        % In 'Wideband' CQI mode, report only the wideband CQI index
        CQI = CQIForAllSubbands(1,:);
    end

    % Form the output CQI information structure
    CQIInfo.SINRPerRBPerCW = SINRsperRBperCW;
    CQIInfo.SINRPerSubbandPerCW = SINRperSubbandperCW;
    CQIInfo.EffectiveSINR = EffectiveSINRDB;
    if strcmpi(reportConfig.CQIFormatIndicator,'Wideband')
        % Output wideband CQI value, if CQIFormatIndicator is 'Wideband'
        CQIInfo.SubbandCQI = CQIForAllSubbands(1,:);
        CQIInfo.SINRPerSubbandPerCW = SINRperSubbandperCW(1,:);
        CQIInfo.TransportBLER = BLERForAllSubbands(1,:);
        CQIInfo.EffectiveSINR = EffectiveSINRDB(1,:);
    else
        % Output wideband CQI value followed by subband CQI values, if
        % CQIFormatIndicator is 'Subband'
        CQIInfo.SubbandCQI = CQIForAllSubbands;
        CQIInfo.TransportBLER = BLERForAllSubbands;
    end
end

function SINRsperRECQI = getSINRperRECQI(SINRsperRE,PMISet,subbandSizes,csirsIndSubs_k)
%   SINRSPERRECQI = getSINRperRECQI(SINRSPERRE,PMISET,SUBBANDSIZES,CSIRSINDSUBS_K) returns
%   the SINR values corresponding to the PMISet in RE level granularity
%   spanning one slot, by considering these inputs:
%
%   SINRSPERRE   - The SINR values per RE for all PMI indices
%   PMISET       - The PMI value reported
%   SUBBANDSIZES - The array representing size of each subband

    numSubbands = size(PMISet.i2,2);
    % Get SINR values per RE based on the PMI values
    start = 0;
    SINRsperRECQI = NaN(size(SINRsperRE,1),size(SINRsperRE,2));
    for idx = 1:numSubbands
        if ~any(isnan(PMISet.i2(:,idx)))
            subbandInd = (csirsIndSubs_k>start*12) & (csirsIndSubs_k<(start+ subbandSizes(idx))*12+1);
            if numel(PMISet.i1) == 6
               % In this case the codebook type is 'Type1MultiPanel'
               SINRsperRECQI(subbandInd,:) = SINRsperRE(subbandInd,:,PMISet.i2(1,idx),PMISet.i2(2,idx),PMISet.i2(3,idx),PMISet.i1(1),PMISet.i1(2),PMISet.i1(3),PMISet.i1(4),PMISet.i1(5),PMISet.i1(6));
            else
               SINRsperRECQI(subbandInd,:) = SINRsperRE(subbandInd,:,PMISet.i2(idx),PMISet.i1(1),PMISet.i1(2),PMISet.i1(3));
            end
        end
        start = start + subbandSizes(idx);
    end
end

function SINRsperRBperCW = getSINRperRB(SINRsperRECQI,csirsIndSubs_k,csirsIndSubs_l,NSizeBWP,SymbolsPerSlot)
%   SINRSPERRBPERCW = getSINRperRB(SINRSPERRECQI,CSIRSINDSUBS_K,CSIRSINDSUBS_L,NSIZEBWP,SYMBOLSPERSLOT)
%   returns the SINR values corresponding to the PMISet in RB level
%   granularity spanning one slot.

    % Calculate SINR value per RE per each codeword
    nLayers = size(SINRsperRECQI,2);
    numCodewords = ceil(nLayers/4);
    SINRsperREperCW = NaN(size(SINRsperRECQI,1),numCodewords);
    for k = 1:size(SINRsperRECQI,1)
        temp = reshape(SINRsperRECQI(k,:),1,[]);
        if ~all(isnan(temp))
            SINRsperREperCW(k,:) = cellfun(@sum,nrLayerDemap(temp));
        end
    end

    % Calculate the SINR value per RB by averaging the SINR values per
    % RE within RB spanning one slot
    SINRsperRBperCW = NaN(NSizeBWP,SymbolsPerSlot,numCodewords);
    for RBidx = 1:NSizeBWP
        % Consider the mean of SINR values over each RB
        RBSCIndices = (csirsIndSubs_k>=((RBidx-1)*12+1))&(csirsIndSubs_k<=(RBidx*12));
        RBSymbolIndices = csirsIndSubs_l(RBSCIndices);
        uniqueSymbols = unique(RBSymbolIndices);
        for SymIdx = 1:length(uniqueSymbols)
            % Get the indices of REs of each symbol in current RB
            SymIndInRB = csirsIndSubs_l==uniqueSymbols(SymIdx);
            SCIndInRB = SymIndInRB & RBSCIndices;
            RBSINRs = SINRsperREperCW(SCIndInRB,:);
            SINRsperRBperCW(RBidx,uniqueSymbols(SymIdx),:) = mean(RBSINRs,1);
        end
    end
end

function SubbandSINRs = getSubbandSINR(SINRsperREPMI,SubbandInfo,csirsIndSubs_k)
%   SUBBANDSINRS = (SINRSPERREPMI,SUBBANDINFO,CSIRSINDSUBS_K) returns
%   the SINR values per subband by averaging the SINR values across all the
%   REs within the subband spanning one slot, corresponding to the reported
%   PMI indices, by considering these inputs:
%
%   SINRSPERREPMI  - SINR values per RE for the reported PMI
%   SUBBANDINFO    - Subband information related structure with these 
%   fields:
%      NumSubbands  - Number of subbands
%      SubbandSizes - Size of each subband

    SubbandSINRs = NaN(SubbandInfo.NumSubbands,size(SINRsperREPMI,2));
    % Consider the starting position of first subband as start of BWP
    subbandStart = 0;
    for SubbandIdx = 1:SubbandInfo.NumSubbands
        subbandInd = (csirsIndSubs_k>subbandStart*12) & (csirsIndSubs_k<(subbandStart+ SubbandInfo.SubbandSizes(SubbandIdx))*12+1);
        sinrTmp = SINRsperREPMI(subbandInd,:,:);
        if ~all(isnan(sinrTmp(:)))
            SubbandSINRs(SubbandIdx,:) = mean(sinrTmp,1);
        end
        subbandStart = subbandStart+ SubbandInfo.SubbandSizes(SubbandIdx);
    end
end

function [cqiSubbandInfo,pmiSubbandInfo] = getDownlinkCSISubbandInfo(reportConfig,nStartBWP,nSizeBWP)
%   [CQISUBBANDINFO,PMISUBBANDINFO] = getDownlinkCSISubbandInfo(REPORTCONFIG,NSTARTBWP,NSIZEBWP)
%   returns the CQI subband related information CQISUBBANDINFO and PMI
%   subband or precoding resource block group (PRG) related information
%   PMISUBBANDINFO, by considering CSI reporting configuration structure
%   REPORTCONFIG, BWP start and size.

    % Validate 'SubbandSize'
    NSBPRB = reportConfig.SubbandSize;
    PMISubbandSize = NSBPRB;

    % If PRGSize is present, consider the subband size as PRG size
    if ~isempty(reportConfig.PRGBundleSize)
        reportConfig.PMIFormatIndicator = 'subband';
        PMISubbandSize = reportConfig.PRGBundleSize;
        ignoreBWPSize = true; % To ignore the BWP size for the validation of PRG size
    else
        ignoreBWPSize = false; % To consider the BWP size for the validation of subband size
    end

    % Get the subband information for CQI and PMI reporting
    cqiSubbandInfo = getSubbandInfo(reportConfig.CQIFormatIndicator,nStartBWP,nSizeBWP,NSBPRB,false);
    pmiSubbandInfo = getSubbandInfo(reportConfig.PMIFormatIndicator,nStartBWP,nSizeBWP,PMISubbandSize,ignoreBWPSize);
end

function info = getSubbandInfo(reportingMode,nStartBWP,nSizeBWP,NSBPRB,ignoreBWPSize)
%   INFO = getSubbandInfo(REPORTINGMODE,NSTARTBWP,NSIZEBWP,NSBPRB,IGNOREBWPSIZE)
%   returns the CSI subband information.

    % Get the subband information
    if strcmpi(reportingMode,'Wideband') || (~ignoreBWPSize && nSizeBWP < 24)
        % According to TS 38.214 Table 5.2.1.4-2, if the size of BWP is
        % less than 24 PRBs, the division of BWP into subbands is not
        % applicable. In this case, the number of subbands is considered as
        % 1 and the subband size is considered as the size of BWP
        numSubbands = 1;
        NSBPRB = nSizeBWP;
        subbandSizes = NSBPRB;
    else
        % Calculate the size of first subband
        firstSubbandSize = NSBPRB - mod(nStartBWP,NSBPRB);

        % Calculate the size of last subband
        if mod(nStartBWP + nSizeBWP,NSBPRB) ~= 0
            lastSubbandSize = mod(nStartBWP + nSizeBWP,NSBPRB);
        else
            lastSubbandSize = NSBPRB;
        end

        % Calculate the number of subbands
        numSubbands = (nSizeBWP - (firstSubbandSize + lastSubbandSize))/NSBPRB + 2;

        % Form a vector with each element representing the size of a subband
        subbandSizes = NSBPRB*ones(1,numSubbands);
        subbandSizes(1) = firstSubbandSize;
        subbandSizes(end) = lastSubbandSize;
    end
    % Place the number of subbands and subband sizes in the output
    % structure
    info.NumSubbands = numSubbands;
    info.SubbandSizes = subbandSizes;
end

% Create a PDSCH configuration for the CSI reference resource, as defined
% in TS 38.214 Section 5.2.2.5
function [pdsch,pdschExt] = nrCSIReferenceResource(carrier,reportConfig,numLayers,dmrsConfig,nStartBWP,nSizeBWP)

    pdsch = nrPDSCHConfig;
    pdsch.NStartBWP = nStartBWP;
    pdsch.NSizeBWP = nSizeBWP;
    pdsch.PRBSet = 0:nSizeBWP-1;
    pdsch.SymbolAllocation = [2 carrier.SymbolsPerSlot-2];
    pdsch.ReservedRE = [];
    pdsch.ReservedPRB = {};
    pdsch.NumLayers = numLayers;
    
    pdsch.DMRS.NumCDMGroupsWithoutData = 3;
    pdsch.DMRS.DMRSConfigurationType = 2;
    pdsch.DMRS.DMRSEnhancedR18 = true;
    % Update DMRSLength, DMRSAdditionalPosition and DMRSEnhancedR18 from
    % PDSCH DM-RS configuration
    pdsch.DMRS.DMRSLength = dmrsConfig.DMRSLength;
    pdsch.DMRS.DMRSAdditionalPosition = dmrsConfig.DMRSAdditionalPosition;
    
    pdschExt = struct();
    pdschExt.PRGBundleSize = 2;
    pdschExt.RVSeq = 0;
    pdschExt.XOverhead = 0;

end

function [l2sm,cqiIndex,effectiveSINR,transportBLER,effectiveSINRdB] = cqiSelect(l2sm,carrier,pdsch,xOverhead,SINRs,cqiTableName)

    % Initialize outputs
    ncw = pdsch.NumCodewords;
    cqiIndex = NaN(1,ncw);
    effectiveSINR = NaN(1,ncw);
    effectiveSINRdB = NaN(1,ncw);
    transportBLER = NaN(1,ncw);

    % SINR per layer without NaN
    SINRs = reshape(SINRs,[],pdsch.NumLayers);
    SINRs = 10*log10(SINRs+eps(SINRs));
    nonnan = ~any(isnan(SINRs),2);
    if ~any(nonnan,'all')
        return;
    end
    SINRs = SINRs(nonnan,:);

    % Get modulation orders and target code rates from CQI table
    cqiTable = nr5g.internal.nrCQITables(cqiTableName);
    cqiTable = cqiTable(:,2:3);

    % Use different BLER thresholds for different CQI tables
    % TS 38.214 Section 5.2.2.1
    if strcmpi(cqiTableName,'Table3')
        blerThreshold = 0.00001;
    else
        blerThreshold = 0.1;
    end
    
    [l2sm,cqiIndex,cqiInfo] = nr5g.internal.L2SM.cqiSelect(l2sm,carrier,pdsch,xOverhead,SINRs,cqiTable,blerThreshold);
    effectiveSINR = db2pow(cqiInfo.EffectiveSINR);
    effectiveSINRdB = cqiInfo.EffectiveSINR;
    transportBLER = cqiInfo.TransportBLER;
end
