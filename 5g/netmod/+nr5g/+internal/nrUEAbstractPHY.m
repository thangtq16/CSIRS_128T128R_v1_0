classdef nrUEAbstractPHY < nr5g.internal.nrUEPHY
    %nrUEAbstractPHY Implements abstract physical (PHY) layer for user equipment(UE)
    %   The class implements the abstraction specific aspects of UE PHY.
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.

    %   Copyright 2022-2024 The MathWorks, Inc.

    properties (Access = private)
        %L2SM Link-to-system-mapping (L2SM) context
        L2SM
    end

    methods
        function obj = nrUEAbstractPHY(param, notificationFcn)
            %nrUEAbstractPHY Construct a UE PHY object
            %   OBJ = nrUEAbstractPHY(PARAM,NOTIFICATIONFCN) constructs a UE PHY object.
            %
            %   PARAM is a structure with the fields:
            %     TransmitPower       - UE Tx power in dBm
            %     NumTransmitAntennas - Number of Tx antennas on the UE
            %     NumReceiveAntennas  - Number of Rx antennas on the UE
            %     NoiseFigure         - Noise figure
            %     ReceiveGain         - Receiver gain at UE in dBi
            %
            %   NOTIFICATIONFCN - It is a handle of the node's processEvents
            %   method

            obj = obj@nr5g.internal.nrUEPHY(param, notificationFcn); % Call base class constructor

            % NR packet param
            obj.PacketStruct.Abstraction = true; % Abstracted PHY
            obj.PacketStruct.Metadata = struct('NCellID', [], 'RNTI', [], 'PrecodingMatrix', [], ...
                'NumSamples', [], 'Channel', obj.PacketStruct.Metadata.Channel);
        end

        function addConnection(obj, connectionConfig)
            %addConnection Configures the UE PHY with connection information
            %   connectionConfig is a structure including the following
            %   fields:
            %       RNTI                     - Radio network temporary identifier
            %                                  specified within [1, 65522]. Refer
            %                                  table 7.1-1 in 3GPP TS 38.321 version 18.1.0
            %       NCellID                  - Physical cell ID. values: 0 to 1007 (TS 38.211, sec 7.4.2.1)
            %       DuplexMode               - "FDD" or "TDD"
            %       SubcarrierSpacing        - Subcarrier spacing
            %       NumResourceBlocks        - Number of resource blocks (RBs)
            %       NumHARQ                  - Number of HARQ processes on UE
            %       ChannelBandwidth         - DL or UL channel bandwidth in Hz
            %       DLCarrierFrequency       - DL carrier frequency
            %       ULCarrierFrequency       - UL carrier frequency
            %       CSIReportConfiguration   - CSI report configuration

            addConnection@nr5g.internal.nrUEPHY(obj, connectionConfig);
            % Initialize L2SM to hold PDSCH, CSI-RS and inter-user interferer context
            obj.L2SM = nr5g.internal.L2SM.initialize(obj.CarrierConfig, connectionConfig.NumHARQ, 1);
            obj.L2SMCSI = nr5g.internal.L2SM.initialize(obj.CarrierConfig);
            obj.L2SMIUI = nr5g.internal.L2SM.initialize(obj.CarrierConfig);
        end

        function data = puschData(obj, puschInfo, macPDU)
            % Return the MAC packet without any PHY processing

            if isempty(macPDU)
                % MAC PDU not sent by MAC, which indicates retransmission. Get the MAC PDU
                % from the HARQ buffers
                data = obj.HARQBuffers{puschInfo.HARQID+1};
            else
                % New transmission. Buffer the transport block
                data = macPDU;
                obj.HARQBuffers{puschInfo.HARQID+1} = macPDU;
            end
        end

        function data = srsData(~, ~)
            % Return empty as abstract PHY does not send any SRS waveform

            data = [];
        end

        function [macPDU, crcFlag, sinr] = decodePDSCH(obj, pdschInfo, pktStartTime, pktEndTime, carrierConfigInfo)
            % Return the decoded MAC PDU along with the crc result

            % Read all the relevant packets (i.e either of interest or sent on same
            % carrier frequency) received during the PDSCH reception
            packetInfoList = packetList(obj.RxBuffer, pktStartTime, pktEndTime);

            % Eliminate any PDSCH packets which are not sent on the overlapping resource
            % blocks as the PDSCH of interest. Also, separate out PDSCH of interest
            numPkts = numel(packetInfoList);
            interferingPackets = packetInfoList;
            prbSetPacket = pdschInfo.PDSCHConfig.PRBSet;
            numRBPacket = numel(prbSetPacket);
            packetOfInterest = [];
            intfPktCount=0;
            for pktIdx = 1:numPkts
                metadata = packetInfoList(pktIdx).Metadata;
                if (metadata.PacketType == obj.PXSCHPacketType) % Check for PDSCH
                    if (carrierConfigInfo.NCellID == metadata.NCellID) && ... % Check for PDSCH of interest
                            (pdschInfo.PDSCHConfig.RNTI == metadata.RNTI)
                        packetOfInterest = packetInfoList(pktIdx);
                    else
                        prbSetInterferer = metadata.PacketConfig.PRBSet;
                        isMatched = false;
                        numRBInterferer = numel(prbSetInterferer);
                        % Check for interfering packets
                        for i=1:numRBPacket
                            rbOfInterest = prbSetPacket(i);
                            for j=1:numRBInterferer
                                interferingRB = prbSetInterferer(j);
                                if interferingRB == rbOfInterest
                                    isMatched = true; % Packet is an interfering one
                                    intfPktCount = intfPktCount+1;
                                    interferingPackets(intfPktCount) = packetInfoList(pktIdx);
                                    break;
                                elseif interferingRB>rbOfInterest
                                    % The other values in the interfering
                                    % RB set are only going to be bigger
                                    % than current interfering RB
                                    break;
                                end
                            end
                            if isMatched
                                break;
                            end
                        end
                    end
                end
            end
            interferingPackets = interferingPackets(1:intfPktCount);

            % Estimate channel for all for relevant packets i.e. packet of interest and
            % interferers (inter-cell interferers and inter-user interferers)
            [estChannelGrid, estChannelGridIntf] = estimateChannelGrid(obj, packetOfInterest, ...
                interferingPackets, carrierConfigInfo);

            % Read MAC PDU
            macPDU = packetOfInterest.Data;

            % Calculate crc result using l2sm
            [crcFlag, sinr] = l2smCRC(obj, packetOfInterest, interferingPackets, estChannelGrid, ...
                estChannelGridIntf, pdschInfo, carrierConfigInfo);
        end

        function [dlRank, pmiSet, cqi, precodingMatrix, sinr] = decodeCSIRS(obj, csirsConfig, pktStartTime, pktEndTime, carrierConfigInfo)
            % Return CSI-RS measurement.
            %
            % ThangTQ23_128T128R_Rel19: For 128T, four Row-18 (32-port) CSI-RS
            % resources are spread across 2 slots (R0/R1 at slot+0, R2/R3 at
            % slot+1).  Each call to decodeCSIRS accumulates one resource into
            % obj.CSIRSChannelBuffer.  PMI selection (nrRISelect + LQM + CQI)
            % runs only when all 4 resources are buffered.  Before that, the
            % function returns dlRank = -1 (sentinel) so that csirsRx() defers
            % the MAC indication.

            % ── Step 1: find packet + estimate channel ──────────────────────
            [csirsPacket, interferingPackets] = packetListIntfBuffer(obj, obj.CSIRSPacketType, ...
                pktStartTime, pktEndTime);

            nVar = calculateThermalNoise(obj);
            obj.GNBReceivedPower = csirsPacket.Power;

            rnti = obj.RNTI;
            packetOfInterest = [];
            for pktIdx = 1:numel(csirsPacket)
                if any(rnti == csirsPacket(pktIdx).Metadata.RNTI)
                    packetOfInterest = csirsPacket(pktIdx);
                    break;
                end
            end

            [estChannelGrid, estChannelGridIntf] = estimateChannelGrid(obj, packetOfInterest, ...
                interferingPackets, carrierConfigInfo);

            intf = prepareLQMInputIntf(obj, obj.L2SMIUI, interferingPackets, estChannelGridIntf, ...
                carrierConfigInfo, nVar);

            % ── Step 2: 128T accumulation path (Row 18 = 32-port CDM8 resource)
            % ThangTQ23_128T128R_Rel19: aggregate H across all resources
            % before running PMI.  numResources128T = 4 for 128T.
            numResources128T = 4;
            if csirsConfig.RowNumber == 18 && csirsConfig.NumCSIRSPorts == 32
                % ThangTQ23_128T128R_Rel19: In abstract PHY, nrPerfectChannelEstimate
                % already returns the full 128-port channel [K x L x nRx x 128]
                % for every CSI-RS resource event (gNB has 128 antennas).
                % DO NOT concatenate along dim-4; instead accumulate as a running
                % average across all 4 resources for improved noise averaging.
                buf = obj.CSIRSChannelBuffer;
                if buf.count == 0
                    buf.H = estChannelGrid;
                else
                    buf.H = (buf.H * buf.count + estChannelGrid) / (buf.count + 1);
                end
                buf.count  = buf.count + 1;
                buf.nVar   = (buf.nVar * (buf.count - 1) + nVar) / buf.count;
                buf.intf   = intf;        % approximation: use latest resource intf
                buf.carrier = carrierConfigInfo;
                obj.CSIRSChannelBuffer = buf;

                if buf.count < numResources128T
                    % Not enough resources yet — return sentinel to defer report
                    dlRank = -1;
                    pmiSet = struct('i1', NaN(1,3), 'i2', NaN);
                    cqi = 0;  precodingMatrix = [];  sinr = -Inf;
                    return;
                end

                % All resources received: use averaged 128-port H for PMI
                estChannelGrid  = buf.H;         % [K x L x nRx x 128]
                nVar            = buf.nVar;
                intf            = buf.intf;
                carrierConfigInfo = buf.carrier;

                % Reset buffer for next period
                obj.CSIRSChannelBuffer = struct('H', [], 'count', 0, 'nVar', 0, ...
                                                'intf', [], 'carrier', []);

                % Build eTypeII-r19 report config for 128 ports.
                % Phase 3 will inject this via connectUE; here we derive a
                % default so that Phase 2 is immediately testable.
                reportCfg128 = obj.CSIReportConfig;
                if ~strcmpi(reportCfg128.CodebookType, 'eTypeII-r19')
                    % Build 128T eTypeII-r19 config as a struct so that all
                    % mandatory fields (NStartBWP, NSizeBWP, CQITable, etc.)
                    % are carried over from the validated base config.
                    reportCfg128.CodebookType                   = 'eTypeII-r19';
                    reportCfg128.PanelDimensions                = [1 16 4]; % N1=16,N2=4 → 128 ports
                    reportCfg128.NumberOfBeams                  = 2;        % L=2 (PC1, tractable)
                    reportCfg128.ParameterCombination           = 1;
                    reportCfg128.SubbandAmplitude               = false;    % wideband
                    % eTypeII-r19 specific fields not present in TypeI struct
                    reportCfg128.NumberOfPMISubbandsPerCQISubband = 1;
                end

                % Run PMI selection on aggregated 128-port channel
                [dlRank, pmiSet, pmiInfo] = nr5g.internal.nrRISelect(carrierConfigInfo, ...
                    csirsConfig, reportCfg128, estChannelGrid, nVar, 'MaxSE');
                reportCfgForLQM = reportCfg128;
            else
                % ── Standard path (≤32T, single resource) ──────────────────
                if csirsConfig.NumCSIRSPorts > 1
                    [dlRank, pmiSet, pmiInfo] = nr5g.internal.nrRISelect(carrierConfigInfo, ...
                        csirsConfig, obj.CSIReportConfig, estChannelGrid, nVar, 'MaxSE');
                else
                    dlRank = 1;
                    pmiSet = struct('i1', [1 1 1], 'i2', 1);
                    pmiInfo.W = 1;
                end
                reportCfgForLQM = obj.CSIReportConfig;
            end

            % ── Step 3: CQI via LQM ─────────────────────────────────────────
            blerThreshold = 0.1;
            overhead = 0;
            if obj.CSIReferenceResource.NumLayers ~= dlRank
                obj.CSIReferenceResource.NumLayers = dlRank;
            end
            precodingMatrix = pmiInfo.W;

            % ThangTQ23_128T128R_Rel19 Phase 4: pmiInfo.W may be 3-D
            % [numPorts×rank×numSubbands] when PMIMode='Subband'.  MATLAB's
            % .' operator requires 2-D input, so extract subband 1 as the
            % wideband proxy for the L2SM LQM (CQI is always wideband here).
            if ndims(pmiInfo.W) == 3
                W_lqm = pmiInfo.W(:,:,1).';   % [rank × numPorts] from subband 1
            else
                W_lqm = pmiInfo.W.';           % [rank × numPorts] wideband
            end
            [obj.L2SMCSI, sig] = nr5g.internal.L2SM.prepareLQMInput(obj.L2SMCSI, ...
                carrierConfigInfo, csirsConfig, estChannelGrid, nVar, W_lqm);
            [obj.L2SMCSI, sinr] = nr5g.internal.L2SM.linkQualityModel(obj.L2SMCSI, sig, intf);
            [obj.L2SMCSI, cqi, cqiInfo] = nr5g.internal.L2SM.cqiSelect(obj.L2SMCSI, ...
                carrierConfigInfo, obj.CSIReferenceResource, overhead, sinr, obj.CQITableValues, blerThreshold);
            cqi = max([cqi, 1]);
            sinr = cqiInfo.EffectiveSINR;
        end
    end

    methods (Access = private)
        function [crcFlag, sinr] = l2smCRC(obj, packetOfInterest, interferingPackets, estChannelGrid, ...
                estChannelGridsIntf, pdschInfo, carrierConfigInfo)
            % Return CRC flag (0 - success, 1 - failure)

            % Noise Variance
            nVar = calculateThermalNoise(obj);

            % Prepare LQM input for interferers
            intf = prepareLQMInputIntf(obj, obj.L2SMIUI, interferingPackets, estChannelGridsIntf, carrierConfigInfo, nVar);

            % Prepare LQM input for packet of interest
            % Extract PDSCH Indices for the packet of interest
            [~, info] = nrPDSCHIndices(carrierConfigInfo, pdschInfo.PDSCHConfig);
            % Prepare HARQ context for the packet of interest
            harqInfo = struct('HARQProcessID', pdschInfo.HARQID, 'RedundancyVersion', pdschInfo.RV, ...
                'TransportBlockSize', pdschInfo.TBS*8, 'NewData', pdschInfo.NewData);
            obj.L2SM = nr5g.internal.L2SM.txHARQ(obj.L2SM, harqInfo, pdschInfo.TargetCodeRate, info.G);
            % Prepare Link Quality Model inputs for the packet of interest
            [obj.L2SM, sig] = nr5g.internal.L2SM.prepareLQMInput(obj.L2SM,carrierConfigInfo, ...
                packetOfInterest.Metadata.PacketConfig, estChannelGrid, nVar, packetOfInterest.Metadata.PrecodingMatrix);

            % Link Quality Model (LQM) with signal of interest and interference
            [obj.L2SM,sinr] = nr5g.internal.L2SM.linkQualityModel(obj.L2SM,sig,intf);

            % Link Performance Model
            [obj.L2SM, crcFlag, cqiInfo] = nr5g.internal.L2SM.linkPerformanceModel(obj.L2SM,harqInfo,pdschInfo.PDSCHConfig,sinr);
            sinr = cqiInfo.EffectiveSINR;
        end
    end
end