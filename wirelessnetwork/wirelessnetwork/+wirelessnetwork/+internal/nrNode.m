classdef (Abstract) nrNode < wirelessnetwork.internal.wirelessNode
    %nrNode Node class containing properties and components common for
    % both gNB node and UE node
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.

    % Copyright 2022-2024 The MathWorks, Inc.

    properties(Hidden)
        %TrafficManager Traffic manager
        TrafficManager

        %RLCEntity RLC layer entity
        RLCEntity

        %MACEntity MAC layer entity
        MACEntity

        %PhyEntity Physical layer entity
        PhyEntity
    end

    properties (Access = protected)
        %LastRunTime Timestamp (in seconds) when the node last ran
        % This gets updated every time the node runs
        LastRunTime = [];

        %LastRunTimeInNanoseconds Timestamp (in nanoseconds) when the node last ran
        LastRunTimeInNanoseconds = 0;

        %PHYAbstraction PHY abstraction flag as true or false
        PHYAbstraction

        %RxInfo Rx information structure returned by packet relevance check
        RxInfo = struct(ID=0, Position=[0 0 0], Velocity=[0 0 0], NumReceiveAntennas=1);
    end

    properties (SetAccess = protected, Hidden)
        %DLCarrierFrequency Downlink carrier frequency in Hz
        DLCarrierFrequency

        %ULCarrierFrequency Uplink carrier frequency in Hz
        ULCarrierFrequency

        %FullBufferTraffic Full buffer traffic configuration for connected
        %nodes
        % Array of strings where each element represents the full buffer
        % traffic configuration for a connected node
        FullBufferTraffic = ""

        %MUMIMOEnabled MU-MIMO enabled (Value true) or not (Value false)
        MUMIMOEnabled = false;
    end

    properties (Access = protected, Constant)
        % MaxLogicalChannels Maximum number of logical channels
        %   Maximum number of logical channels that can be configured
        %   between a UE and its associated gNB, specified in the [1, 32]
        %   range. For more details, refer 3GPP TS 38.321 Table 6.2.1-1
        MaxLogicalChannels = 4;
    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        PHYAbstraction_Values  = ["linkToSystemMapping","none"];
    end

    events(Hidden)
        %AppDataReceived Packet reception at application layer
        %   This event is triggered when data is received at application
        %   layer from the layer below. It passes the event notification
        %   along with structure containing these fields to the registered
        %   callback:
        %   CurrentTime    - Current simulation time in seconds
        %   Packet         - Received application data in decimal bytes,
        %                    returned as vector of integers in the range [0,
        %                    255]
        %   PacketLength   - Length of data in bytes
        AppDataReceived

        %PacketTransmissionStarted Start of packet transmission by PHY
        %   This event is triggered when packet is transmitted from PHY layer
        %   of the node. It passes the event notification along with
        %   structure containing these fields to the registered callback:
        %   CurrentTime  - Current simulation time in seconds
        %   RNTI         - RNTI of the UE associated with the transmission
        %   DuplexMode   - Duplex mode "FDD" or "TDD"
        %   TimingInfo   - Timing information as vector of 3 elements of
        %                  the form [NFrame NSlot NSymbol]
        %   LinkType     - Link direction "DL" or "UL"
        %   HARQID       - HARQ process ID associated with the MAC PDU
        %   SignalType   - The transmission signal type specified as
        %   "PUSCH" (only UL data signal), "PDSCH" (only DL data signal),
        %   "SRS" (only UL reference signal), "CSIRS" (only DL reference
        %   signal), "PUSCH+SRS" (both data and reference signals in UL),
        %   or "PDSCH+CSIRS" (both data and reference signals in DL)
        %   TransmissionType - The transmission is new transmission("NewTx") or
        %   retransmission ("ReTx")
        %   Duration     - Duration the transmission (in seconds)
        %   PDU          - Column vector of decimal octets
        PacketTransmissionStarted

        %PacketReceptionEnded End of packet reception by PHY
        %   This event is triggered when packet reception is ended in PHY
        %   layer of the node. It passes the event notification along with
        %   structure containing these fields to the registered callback:
        %   CurrentTime  - Current simulation time in seconds
        %   RNTI         - RNTI of the UE associated with the reception
        %   DuplexMode   - Duplex mode "FDD" or "TDD"
        %   TimingInfo   - Timing information as vector of 3 elements of
        %                  the form [NFrame NSlot NSymbol]
        %   LinkType     - Link direction "DL" or "UL"
        %   HARQID       - HARQ process ID associated with the MAC PDU
        %   SignalType   - The reception signal type specified as "PUSCH"
        %   (only UL data signal), "PDSCH" (only DL data signal), "SRS"
        %   (only UL reference signal), "CSIRS" (only DL reference signal),
        %   "PUSCH+SRS" (both data and reference signals in UL), or
        %   "PDSCH+CSIRS" (both data and reference signals in DL)
        %   PDU          - Column vector of decimal octets
        %   CRCFlag      - Flag to indicates whether the packet is decoded
        %   correctly by PHY layer or not
        %   SINR      - Signal-to-Interference-plus-Noise Ratio for the data packet
        %   ChannelMeasurements - Information about the measurement report
        %   for the reference signals.
        %   In DL, it is a structure with below fields
        %       SINR - SINR measured on CSI-RS
        %       RI   - Rank Indicator
        %       PMI  - Precoding Matrix Indicator
        %       CQI  - Channel Quality Indicator
        %       W    - Precoder
        %   In UL, it is a structure with below fields
        %       SINR - SINR measured on SRS
        %       SRSBasedULMeasurements is a structure with below fields
        %           RI        - Rank Indicator
        %           TPMI      - Transmitted PMI
        %           MCSIndex  - Modulation and Coding Scheme index
        %       SRSBasedDLMeasurements is a structure with below fields
        %           RI      - Rank Indicator
        %           MCSIndex - Modulation and Coding Scheme Index
        %           W        - Precoder
        %   By default it is [], if channel measurements are not available.
        %   Note: The SINR presented here is the effective SINR of the
        %   packet. SINR is not measured for PDSCH and PUSCH data packets
        %   when abstraction method is set to "none". The default value is -Inf
        PacketReceptionEnded
    end

    methods

        function obj = nrNode()
            %nrNode Initialize the object properties with default values
        end

        function addTrafficSource(obj, trafficSource, varargin)
            %addTrafficSource Add data traffic source to 5G NR node
            %   addTrafficSource(OBJ, TRAFFICSOURCE) adds a data traffic source object,
            %   TRAFFICSOURCE, to the node, OBJ. TRAFFICSOURCE is an object of type
            %   <a href="matlab:help('networkTrafficOnOff')">networkTrafficOnOff</a>, <a href="matlab:help('networkTrafficFTP')">networkTrafficFTP</a>, <a href="matlab:help('networkTrafficVoIP')">networkTrafficVoIP</a>, or
            %   <a href="matlab:help('networkTrafficVideoConference')">networkTrafficVideoConference</a>. Because an NR node always generates
            %   an application packet with payload, the GeneratePacket property
            %   of a traffic source object is not applicable to an NR node.
            %   OBJ is an object of type <a href="matlab:help('nrGNB')">nrGNB</a> or <a
            %   href="matlab:help('nrUE')">nrUE</a>. addTrafficSource(...,Name=Value)
            %   specifies additional name-value argument as described below.
            %
            %   DestinationNode  - Specify the destination node as an
            %                      object of type <a
            %                      href="matlab:help('nrUE')">nrUE</a>. Set this N-V
            %                      argument only if OBJ is of type <a
            %                      href="matlab:help('nrGNB')">nrGNB</a>. It is
            %                      automatically set as gNB to which UE is connected,
            %                      if OBJ is of type <a
            %                      href="matlab:help('nrUE')">nrUE</a>.
            %
            %   LogicalChannelID - Specify the logical channel identifier
            %                      as an integer scalar within the range [4-32]. The
            %                      added traffic will be mapped to the specified
            %                      logical channel. If no logical channel is specified,
            %                      the traffic will be mapped to the logical channel
            %                      with the smallest ID. If the traffic is mapped to a
            %                      logical channel which is not yet established, error
            %                      will be thrown.

            % First argument must be scalar object
            validateattributes(obj, {'nrGNB', 'nrUE'}, {'nonempty', 'scalar'}, mfilename, 'obj');

            coder.internal.errorIf(~isempty(obj.LastRunTime), 'nr5g:nrNode:NotSupportedOperation', 'addTrafficSource');

            % Validate data source object
            coder.internal.errorIf(~isa(trafficSource, 'wirelessnetwork.internal.networkTraffic') || ~isscalar(trafficSource), 'wirelessnetwork:networkTraffic:InvalidTrafficSource');

            % Name-value pair check
            coder.internal.errorIf(mod(numel(varargin), 2) == 1, 'MATLAB:system:invalidPVPairs');

            [upperLayerDataInfo, rlcEntity] = nr5g.internal.nrNodeValidation.validateNVPairAddTrafficSource(obj, varargin);
            % Add the traffic source to traffic manager
            addTrafficSource(obj.TrafficManager, trafficSource, upperLayerDataInfo, @rlcEntity.enqueueSDU);
        end
    end

    methods(Hidden)
        function nextInvokeTime = run(obj, currentTime)
            %run Run the 5G NR node
            %
            %   NEXTINVOKETIME = run(OBJ, CURRENTTIME) runs the 5G NR node
            %   at current time and returns the time at which the node must
            %   be invoked again.
            %
            %   NEXTINVOKETIME is the time (in seconds) at which node must
            %   be invoked again.
            %
            %   OBJ is an object of type nrGNB or nrUE.
            %
            %   CURRENTTIME is the current simulation time in seconds.

            % First argument must be scalar object

            obj.LastRunTime = currentTime;
            obj.LastRunTimeInNanoseconds = round(currentTime * 1e9);  % Convert time into nanoseconds
            if obj.ReceiveBufferIdx ~= 0 % Rx buffer has data to be processed
                % Pass the data to layers for processing
                nextInvokeTime = runLayers(obj, obj.LastRunTimeInNanoseconds, [obj.ReceiveBuffer{1:obj.ReceiveBufferIdx}]);
                obj.ReceiveBufferIdx = 0;
            else % Rx buffer has no data to process
                % Update the current time for all the layers
                nextInvokeTime = runLayers(obj, obj.LastRunTimeInNanoseconds, {});
            end
        end

        function addToTxBuffer(obj, packet)
            %addToTxBuffer Adds the packet to Tx buffer
            %
            % OBJ is an object of type nrGNB or nrUE. PACKET is the 5G
            % packet to be transmitted. It is a structure of the format <a
            % href="matlab:help('wirelessnetwork.internal.wirelessPacket')">wirelessPacket</a>

            packet.TransmitterID = obj.ID;
            packet.TransmitterPosition = obj.Position;
            packet.TransmitterVelocity = obj.Velocity;
            obj.TransmitterBuffer = [obj.TransmitterBuffer packet];
        end

        function pushReceivedData(obj, packet)
            %pushReceivedData Push the received packet to node
            %
            % OBJ is an object of type nrGNB or nrUE. PACKET is the
            % received packet. It is a structure of the format <a href="matlab:help('wirelessnetwork.internal.wirelessPacket')">wirelessPacket</a>

            % Check if PHY flavor matches for the node and the received packet
            if ~packet.DirectToDestination && (packet.Abstraction ~= obj.PHYAbstraction)
                coder.internal.error('nr5g:nrNode:MixedPHYFlavorNotSupported')
            end
            % Copy the received packet to the buffer
            obj.ReceiveBufferIdx = obj.ReceiveBufferIdx + 1;
            obj.ReceiveBuffer{obj.ReceiveBufferIdx} = packet;
        end

        function processEvents(obj, eventName, data)
            % Send the event notification to listeners

            if event.hasListener(obj, eventName)
                data.CurrentTime = obj.LastRunTime;
                % Create an event data object
                eventDataObj = wirelessnetwork.internal.nodeEventData;
                eventDataObj.Data = data;
                % Notify listeners about the event
                notify(obj, eventName, eventDataObj);
            end
        end
    end

    methods (Access = protected)
        function addRLCBearer(obj, rlcConnectionInfo)
            %addRLCBearer Add RLC entity to node and its associated logical
            %channel to MAC

            % Check if full buffer is configured
            if rlcConnectionInfo.FullBufferTraffic ~= "off"
                addRLCBearerForFullBuffer(obj, rlcConnectionInfo);
            else
                addRLCBearerForCustomTraffic(obj, rlcConnectionInfo);
            end
        end

        function nextInvokeTime = runLayers(obj, currentTime, packets)
            %runLayers Run the node with the received packet and returns the next invoke time (in seconds)

            % Run the application traffic manager
            nextAppTime = run(obj.TrafficManager, currentTime);

            % Run the RLC layer
            nextRLCTime = runRLCLayer(obj, currentTime);

            % Run the MAC layer operations
            nextMACTime = run(obj.MACEntity, currentTime, packets);

            % Run the PHY operations
            nextPHYTime = run(obj.PhyEntity, currentTime, packets);

            % Find the next invoke time (in seconds) for the node
            nextInvokeTime = min([nextAppTime nextRLCTime nextMACTime nextPHYTime]) * 1e-9;
        end

        function addRLCBearerForFullBuffer(obj, rlcConnectionInfo)
            %addRLCBearerForFullBuffer Add RLC bearer when full buffer is
            %enabled

            macEntity = obj.MACEntity;
            rlcBearerConfig = nrRLCBearerConfig();
            % Establish an RLC passthrough entity and associated logical
            % channel when full buffer is enabled
            if macEntity.MACType == 0 % In case of gNB
                if rlcConnectionInfo.FullBufferTraffic ~= "ul"
                    % Establish an RLC passthrough entity with transmitting
                    % capability when full buffer is not exclusively
                    % enabled on UL. This indicates it is enabled using
                    % value 'on' or 'DL'
                    rlcEntity = nr5g.internal.nrRLCPassthrough(rlcConnectionInfo.RNTI, ...
                        rlcBearerConfig.LogicalChannelID, @macEntity.updateBufferStatus);
                else
                    rlcEntity = nr5g.internal.nrRLCPassthrough(rlcConnectionInfo.RNTI, ...
                        rlcBearerConfig.LogicalChannelID, []);
                end
            else %  In case of UE
                if rlcConnectionInfo.FullBufferTraffic ~= "dl"
                    % Establish an RLC passthrough entity with transmitting
                    % capability when full buffer is not exclusively
                    % enabled on DL. This indicates it is enabled using
                    % value 'on' or 'UL'
                    rlcEntity = nr5g.internal.nrRLCPassthrough(rlcConnectionInfo.RNTI, ...
                        rlcBearerConfig.LogicalChannelID, @macEntity.updateBufferStatus);
                else
                    rlcEntity = nr5g.internal.nrRLCPassthrough(rlcConnectionInfo.RNTI, ...
                        rlcBearerConfig.LogicalChannelID, []);
                end
            end
            obj.RLCEntity{end+1} = rlcEntity;
            % Use weak-references for cross-linking handle objects
            rlcWeakRef = matlab.lang.WeakReference(rlcEntity);

            % Add MAC logical channel configuration for full buffer traffic
            % case
            addLogicalChannelInfo(macEntity, rlcBearerConfig, rlcConnectionInfo.RNTI);
            registerRLCInterfaceFcn(macEntity, rlcConnectionInfo.RNTI, rlcBearerConfig.LogicalChannelID, ...
                @(varargin) rlcWeakRef.Handle.sendPDUs(varargin{:}), ...
                @(varargin) rlcWeakRef.Handle.receivePDUs(varargin{:}));
        end

        function addRLCBearerForCustomTraffic(obj, rlcConnectionInfo)
            %addRLCBearerForCustomTraffic Add RLC bearer for custom traffic

            macEntity = obj.MACEntity;
            % Use weak-references for cross-linking handle objects
            macWeakRef = matlab.lang.WeakReference(macEntity);

            trafficManager = obj.TrafficManager;
            rnti = rlcConnectionInfo.RNTI;
            maxReassemblySDU = macEntity.NumHARQ;
            % Set up a default RLC bearer if no RLC bearer configuration is
            % provided
            rlcBearerConfigSet = rlcConnectionInfo.RLCBearerConfig;
            if isempty(rlcBearerConfigSet)
                rlcBearerConfigSet = nrRLCBearerConfig();
            end
            % Establish RLC entities and their associated logical channel at
            % MAC by iterating through the given RLC bearer configuration objects
            for rlcBearerIdx = 1:size(rlcBearerConfigSet,1)
                rlcBearerConfig = rlcBearerConfigSet(rlcBearerIdx);

                % Set up RLC entity at RLC layer
                if rlcBearerConfig.RLCEntityType == "AM"
                    rlcEntity = nr5g.internal.nrRLCAM(rnti, rlcBearerConfig, maxReassemblySDU, ...
                        @(varargin) macWeakRef.Handle.updateBufferStatus(varargin{:}), ...
                        @trafficManager.receivePacket);
                elseif rlcBearerConfig.RLCEntityType == "UM"
                    rlcEntity = nr5g.internal.nrRLCUM(rnti, rlcBearerConfig, maxReassemblySDU, ...
                        @(varargin) macWeakRef.Handle.updateBufferStatus(varargin{:}), ...
                        @trafficManager.receivePacket);
                else
                     % Get the MAC type of node where the transmitting or receiving RLC entity
                     % is present
                    if rlcBearerConfig.RLCEntityType == "UMDL"
                        macEntityType = 0;
                    else
                        macEntityType = 1;
                    end
                    if macEntityType == macEntity.MACType
                        rlcEntity = nr5g.internal.nrRLCUM(rnti, rlcBearerConfig, maxReassemblySDU, ...
                            @(varargin) macWeakRef.Handle.updateBufferStatus(varargin{:}), []);
                    else
                        rlcEntity = nr5g.internal.nrRLCUM(rnti, rlcBearerConfig, maxReassemblySDU, [], @trafficManager.receivePacket);
                    end
                end
                obj.RLCEntity{end+1} = rlcEntity;
                % Use weak-references for cross-linking handle objects
                rlcWeakRef = matlab.lang.WeakReference(rlcEntity);

                % Set up logical channel at MAC layer
                addLogicalChannelInfo(macEntity, rlcBearerConfig, rlcConnectionInfo.RNTI);
                registerRLCInterfaceFcn(macEntity, rlcConnectionInfo.RNTI, rlcBearerConfig.LogicalChannelID, ...
                    @(varargin) rlcWeakRef.Handle.sendPDUs(varargin{:}), ...
                    @(varargin) rlcWeakRef.Handle.receivePDUs(varargin{:}));
            end
        end
    end

    methods (Access = private)
        function nextRLCTime = runRLCLayer(obj, currentTime)
            %runRLCLayer Invoke the run method of RLC entities to take timer-based actions

            numRLCEntities = numel(obj.RLCEntity);
            nextRLCTime = Inf;
            for idx = 1:numRLCEntities
                % Run and get the next invoke times of RLC entity
                nextInvokeTime = run(obj.RLCEntity{idx}, currentTime);
                % Find the next invoke time, which is the smallest among all the RLC
                % entities' invoke times, of RLC layer
                if nextInvokeTime < nextRLCTime
                    nextRLCTime = nextInvokeTime;
                end
            end
        end
    end

    methods (Hidden)
        function sendPacketToRLC(obj, packetInfo)
            %sendPacketToRLC Send a packet received from user to RLC queue
            %   sendPacketToRLC(OBJ, PACKETINFO) sends a packet received
            %   from user to RLC queue.
            %
            %   OBJ is an object of type nrGNB or nrUE.
            %
            %   PACKETINFO is a structure with these mandatory fields.
            %       RNTI   - Radio network temporary identifier of a UE.
            %       Packet - Array of octets in decimal format.
            %       DestinationNodeID - Destination node ID.
            %       LogicalChannelID  - Logical channel ID.

            packetInfo.PacketLength = size(packetInfo.Packet,1);
            % Get the corresponding RLC entity for the received packet
            rlcEntity = nr5g.internal.nrNodeValidation.getRLCEntity(obj, packetInfo);
            if ~isempty(rlcEntity)
                 % Send the packet to the RLC entity
                enqueueSDU(rlcEntity, packetInfo);
            end
        end
    end
end