classdef wlanNode < wirelessnetwork.internal.wirelessNode
%wlanNode WLAN node
%   NODEOBJ = wlanNode creates a default WLAN node object.
%
%   NODEOBJ = wlanNode(Name=Value) creates one or more similar WLAN node
%   objects with the specified property Name set to the specified Value.
%   You can specify additional name-value arguments in any order as
%   (Name1=Value1, ..., NameN=ValueN). The number of rows in the "Position"
%   property defines the number of nodes created. "Position" must be an
%   N-by-3 matrix where N(>=1) is the number of nodes, and each row must
%   contain three numeric values representing the [X, Y, Z] position of a
%   node in meters. The output, NODEOBJ, is an object or a row vector of
%   objects of the type wlanNode. You can also specify multiple names for
%   "Name" property corresponding to number of nodes created. Multiple
%   names must be specified either as a vector of strings or a cell array
%   of character vectors. If you do not specify the name, the object uses a
%   default name "NodeX", where 'X' is ID of the node. Assuming 'N' nodes
%   are created and 'M' names are supplied, if (M>N) then trailing (M-N)
%   names are ignored, and if (N>M) then trailing (N-M) nodes are set to
%   default names. You can set the "Position" and "Name" properties for
%   multiple nodes simultaneously when you specify them as N-V arguments
%   while creating the object(s). After creating the nodes, you can only
%   set the "Position" and "Name" properties for one node object at a time.
%
%   wlanNode properties (configurable through N-V pair as well as public settable):
%
%   Name                    - Name of the node
%   Position                - Position of the node
%
%   wlanNode properties (configurable through N-V pair only):
%
%   MACFrameAbstraction     - Flag indicating MAC frame is abstracted
%   PHYAbstractionMethod    - PHY abstraction method
%   DeviceConfig            - Device configuration
%
%   wlanNode properties (read-only):
%
%   ID                      - Node identifier
%
%   wlanNode methods:
%
%   associateStations       - Associate stations to WLAN node
%   addTrafficSource        - Add data traffic source to WLAN node
%   addMeshPath             - Add mesh path to WLAN node
%   update                  - Update configuration of WLAN node
%   statistics              - Get the statistics of WLAN node
%   addMobility             - Add random waypoint mobility model to WLAN node
%
%   % Example 1:
%   %   Create a wlanNode object with the name "MyNode".
%
%   myNode = wlanNode(Name="MyNode");
%   disp(myNode)
%
%   % Example 2:
%   %   Create an access point (AP) node with packet transmission format 
%   %   set to "VHT" and modulation and coding scheme (MCS) value set to 7.
%
%   deviceCfg = wlanDeviceConfig(Mode="AP", ...
%                                TransmissionFormat="VHT", ...
%                                MCS=7);
%   apNode = wlanNode(DeviceConfig=deviceCfg);
%
%   % Example 3:
%   %   Create three station (STA) nodes operating on 5 GHz band and 
%   %   channel number 44.
%
%   % 3 positions for 3 STA nodes
%   staPositions = [0 0 0; 10 0 0; 20 0 0];
%   % Set band and channel
%   deviceCfg = wlanDeviceConfig(Mode="STA", BandAndChannel=[5 44]);
%   % Create an array of 3 STA node objects
%   staNodes = wlanNode(Position=staPositions, DeviceConfig=deviceCfg);
%
%   % Example 4:
%   %   Create, Configure, and Simulate Wireless Local Area Network with one
%   %   AP and one STA.
% 
%   % Check if the 'Communications Toolbox (TM) Wireless Network Simulation
%   % Library' support package is installed. If the support package is not 
%   % installed, MATLAB(R) returns an error with a link to download and 
%   % install the support package.
%   wirelessnetworkSupportPackageCheck;
% 
%   % Initialize wireless network simulator
%   networksimulator = wirelessNetworkSimulator.init;
% 
%   % Create a WLAN node with AP device configuration
%   apDeviceCfg = wlanDeviceConfig(Mode="AP");
%   apNode = wlanNode(Name="AP",DeviceConfig=apDeviceCfg);
% 
%   % Create a WLAN node with STA device configuration
%   staDeviceCfg = wlanDeviceConfig(Mode="STA");
%   staNode = wlanNode(Name="STA",DeviceConfig=staDeviceCfg);
% 
%   % Associate the STA to the AP and configure downlink full buffer traffic
%   associateStations(apNode,staNode,FullBufferTraffic="DL");
% 
%   % Add nodes to the simulation
%   addNodes(networksimulator,[apNode,staNode]);
% 
%   % Run simulation for 1 second
%   run(networksimulator,1);
% 
%   % Retrieve and display statistics of AP and STA
%   apStats = statistics(apNode);
%   staStats = statistics(staNode);
%   disp(apStats)
%   disp(staStats)
%
%   See also wlanDeviceConfig, wlanMultilinkDeviceConfig

%   Copyright 2022-2025 The MathWorks, Inc.

    properties(SetAccess = private)
        %MACFrameAbstraction MAC frame abstraction
        %   Set this property to true to indicate MAC frame is abstracted. After
        %   the object is created this property is read-only. If this property is
        %   set to true, MAC frame bits are not generated and a structure with MAC
        %   frame information is passed in the packet from transmitting node to
        %   receiving node. If this property is set to false, MAC frame bits are
        %   generated and the frame bits are passed in the packet from transmitting
        %   node to receiving node. The default value is true.
        MACFrameAbstraction (1, 1) logical = true;

        %PHYAbstractionMethod PHY abstraction method
        %   Specify the PHY abstraction method as
        %   "tgax-evaluation-methodology", "tgax-mac-calibration", or
        %   "none". After the object is created this property is read-only.
        %   The value "tgax-evaluation-methodology" corresponds to the
        %   abstraction mentioned in the Appendix-1 of IEEE
        %   802.11-14/0571r12 TGax evaluation methodology document and
        %   "tgax-mac-calibration" corresponds to the abstraction mentioned
        %   in the IEEE 802.11-14/0980r16 TGax simulation scenarios
        %   document. The value "none" corresponds to the full physical
        %   layer processing. The default value is
        %   "tgax-evaluation-methodology".
        PHYAbstractionMethod = "tgax-evaluation-methodology";

        %DeviceConfig Device configuration
        %   Specify the device configuration as a scalar object of type <a
        %   href="matlab:help('wlanDeviceConfig')">wlanDeviceConfig</a>
        %   or <a href="matlab:help('wlanMultilinkDeviceConfig')">wlanMultilinkDeviceConfig</a>. If you want to configure more than one
        %   non-multilink devices (non-MLD), specify this value as a vector of
        %   wlanDeviceConfig objects. After you create the object, this property is
        %   read-only. When you configure multiple non-MLDs, the <a
        %   href="matlab:help('wlanDeviceConfig/Mode')">Mode</a> property of
        %   objects in the vector must be set to either "AP" or "mesh". The default
        %   value is an object of the type wlanDeviceConfig with default parameters.
        DeviceConfig (1, :) {mustBeA(DeviceConfig, ["wlanDeviceConfig" "wlanMultilinkDeviceConfig"])} = wlanDeviceConfig;
    end

    events(Hidden)
        %TransmissionStatus is triggered after decoding the response frames
        % or after waiting for response timeout and determining the transmission
        % status of RTS/MU-RTS and data frames. This event is triggered for each
        % user in the transmission. TransmissionStatus passes the event
        % notification along with this structure as input to the registered
        % callback:
        %   DeviceID           - Scalar representing device identifier.
        %   CurrentTime        - Scalar representing current simulation
        %                        time in seconds.
        %   FrameType          - String representing frame type as one of
        %                        "QoS Data", "RTS", or "MU-RTS".
        %   ReceiverNodeID     - Scalar representing ID of the node to
        %                        which frame is transmitted
        %   MPDUSuccess        - Logical scalar when transmitted frame is
        %                        an MPDU and vector when it is an A-MPDU.
        %                        Each element represents transmission
        %                        status as:
        %                          'true'  - Transmission success
        %                          'false' - Transmission failure
        %   MPDUDiscarded      - Logical scalar when transmitted frame
        %                        is an MPDU and vector when it is an
        %                        A-MPDU. Each element represents whether
        %                        the MPDU is discarded:
        %                          'true'  - MPDU discarded
        %                          'false' - MPDU not discarded
        %                        When FrameType is "RTS" or "MU-RTS",
        %                        MPDUDiscarded flag indicates the status of
        %                        discard of data packets from transmission
        %                        queues.
        %   TimeInQueue        - Scalar when transmitted frame is an
        %                        MPDU and vector when it is an A-MPDU. Each
        %                        element represents time in seconds spent
        %                        by packet in MAC queue. This is applicable
        %                        for MPDUs whose MPDUDiscarded flag is set
        %                        to true.
        %   AccessCategory     - Scalar when transmitted frame is an
        %                        MPDU and vector when it is an A-MPDU. Each
        %                        element represents access category of the
        %                        MPDU, where 0, 1, 2 and 3 represents
        %                        Best-Effort, Background, Video and Voice
        %                        respectively. When FrameType is "RTS" or
        %                        "MU-RTS", it indicates the access category
        %                        of the corresponding "QoS Data".
        %   ResponseRSSI       - Scalar value representing the signal
        %                        strength of the received response in the
        %                        form of an Ack frame, a Block Ack frame,
        %                        or a CTS frame.
        TransmissionStatus;

        %MPDUGenerated is triggered on generation of an MPDU in the MAC
        % layer. This event is triggered only in case of full MAC frame generation.
        % For A-MPDUs, this is triggered when all MPDU(s) in the A-MPDU are
        % generated. MPDUGenerated passes the event notification along with this
        % structure as input to the registered callback:
        %   DeviceID    - Scalar representing device identifier.
        %   CurrentTime - Scalar representing current simulation time in seconds.
        %   MPDU        - Cell array of MPDU(s) where each element is a vector
        %                 containing MPDU bytes in decimal format.
        %   Frequency   - Scalar representing center frequency of transmitting
        %                 PPDU in Hz.
        MPDUGenerated;

        %MPDUDecoded is triggered either when a decode failure is indicated by PHY
        % to MAC layer or on decoding of an MPDU in the MAC layer. In first case,
        % the decode failure may be due to failed preamble decoding or failed
        % header decoding or filtered PPDU or carrier lost. In second case, for
        % A-MPDUs, this is triggered when all MPDU(s) in the A-MPDU are decoded.
        % MPDUDecoded passes the event notification along with this structure as
        % input to the registered callback:
        %   DeviceID        - Scalar representing device identifier.
        %   CurrentTime     - Scalar representing current simulation time in seconds.
        %   MPDU            - Cell array of MPDU(s) where each element is a vector
        %                     containing MPDU bytes in decimal format in case of full
        %                     MAC frames.
        %                     Structure containing information of all MPDUs in a MAC
        %                     frame in case of abstract MAC frames
        %   FCSFail         - Flag representing frame check sequence (FCS) failure at
        %                     MAC. In case of multiple MPDUs, it is a vector with
        %                     values for each MPDU.
        %   PHYDecodeFail   - Logical scalar representing a decode failure at PHY,
        %                     when set to true. When set to true, MPDU and FCSFail
        %                     fields are not applicable.
        %   PPDUStartTime   - Scalar representing PPDU start time in seconds.
        %   Frequency       - Scalar representing center frequency of PPDU in Hz.
        %   Bandwidth       - Scalar representing bandwidth of PPDU in Hz.
        MPDUDecoded;

        %AppDataReceived is triggered after the decoded packet is received
        % by the application from the MAC layer. AppDataReceived passes the event
        % notification along with this structure as input to the registered
        % callback:
        %   Packet               - Vector of data bytes. When MAC packet is
        %                          abstracted, Data contains empty value.
        %   PacketLength         - Length of the packet in bytes.
        %   PacketID             - Unique identifier for the packet assigned by
        %                          the source node, to identify the packet.
        %   PacketGenerationTime - Timestamp of the packet generation in seconds.
        %   SourceNodeID         - Source transmitter node identifier.
        %   AccessCategory       - Scalar representing access category of
        %                          transmitted frame. This value can be 0, 1, 2, or
        %                          3 representing Best-Effort, Background, Video,
        %                          or Voice respectively. Applicable only when
        %                          'FrameType' is 'QoS Data'.
        %   CurrentTime          - Scalar representing the current simulation
        %                          time in seconds.
        AppDataReceived

        %StateChanged is triggered on any change in the state of the device.
        % StateChanged passes the event notification along with this structure as
        % input to the registered callback:
        %   DeviceID    - Scalar representing device identifier.
        %   CurrentTime - Scalar representing current simulation time in seconds.
        %   State       - State of device specified as "Idle", "Sleep", "Contention",
        %                 "Transmission", or "Reception".
        %   Duration    - Scalar representing state duration.
        %   Frequency   - Scalar representing center frequency of transmitted
        %                 waveform in Hz. Applicable only when State is
        %                 "Transmission.
        %   Bandwidth   - Scalar representing bandwidth of transmitted waveform
        %                 in Hz. Applicable only when State is "Transmission".
        StateChanged;
    end

    properties (Hidden)
        %MeshBridge Mesh bridging object
        %   This property is an object of type <a
        %   href="matlab:help('wlan.internal.meshBridge')">wlan.internal.meshBridge</a>. This object
        %   contains methods and properties related to mesh forwarding.
        MeshBridge

        %Application WLAN application layer object
        %   Specify this property as an object of type <a
        %   href="matlab:help('wlan.internal.trafficManager')">wlan.internal.trafficManager</a>.
        %   This object contains methods and properties related to
        %   application layer.
        Application;

        %SharedMAC WLAN shared MAC layer object
        %   This property is a vector of objects of type <a
        %   href="matlab:help('wlan.internal.sharedMAC')">wlan.internal.sharedMAC</a>.
        %   This object performs functionalities like sequence number assignment,
        %   shared queue maintenance, association context maintenance and link
        %   management. This is a scalar when the node contains a multilink device
        %   (MLD) and is common to all the links in the MLD. This is also a scalar
        %   when the node supports only a single non-MLD. Otherwise, this property
        %   is specified as a vector of objects.
        SharedMAC;

        %MAC WLAN EDCA MAC layer object
        %   This property is a vector of objects of type <a
        %   href="matlab:help('wlan.internal.edcaMAC')">wlan.internal.edcaMAC</a>.
        %   This object maintains WLAN MAC layer state machine and is responsible
        %   for contention, transmit, and receive operations. This is a scalar when
        %   the node supports only a single non-MLD or an MLD with single link .
        %   Otherwise, this property is specified as a vector of objects.
        MAC;

        %PHYTx WLAN physical layer transmitter object
        %   This property is a vector of abstracted PHY objects <a
        %   href="matlab:help('wlan.internal.sls.phyTxAbstract')">wlan.internal.sls.phyTxAbstract</a>.
        %   This object contains methods and properties related to WLAN PHY
        %   transmitter. This is a scalar when the node supports only a
        %   single device. Otherwise, this property is specified as a
        %   vector of objects.
        PHYTx;

        %PHYRx WLAN physical layer receiver object
        %   This property is a vector of abstracted PHY objects <a
        %   href="matlab:help('wlan.internal.sls.phyRxAbstract')">wlan.internal.sls.phyRxAbstract</a>.
        %   This object contains methods and properties related to WLAN PHY
        %   receiver. This is a scalar when the node supports only a single
        %   device. Otherwise, this property is specified as a vector of
        %   objects.
        PHYRx;

        %PacketLatency Packet latency of each application packet received
        %   This property is a vector of numeric values. Each value
        %   specifies the latency computed for every packet received in
        %   microseconds.
        PacketLatency = 0;

        %PacketLatencyIdx Current index of the packet latency vector
        %   This property is a numeric value. This property specifies current index
        %   of the packet latency vector.
        PacketLatencyIdx = 0;

        %WLANSignal WLAN signal structure
        %   The WLAN signal is a structure of type <a
        %   href="matlab:help('wirelessnetwork.internal.wirelessPacket')">wirelessnetwork.internal.wirelessPacket</a>.
        WLANSignal;

        %AssociationInfo Information of associated STAs in BSS, if the node is an
        %AP and associated AP, if the node is a STA
        %   This property is an array of structures of size N x 1. N is the number
        %   of associated STAs in case of AP. N equals 1 in case of STA. Each
        %   structure contains following fields:
        %     NodeID          - Node identifier of associated STA or AP
        %     MACAddress      - MAC address of associated STA or AP. Contains one
        %                       or multiple addresses if STA and AP are MLDs due to
        %                       multilink
        %     DeviceID        - Device index or link index/indices on which
        %                       AP is connected to the STA or vice-versa
        %     AID             - Association identifier (AID) assigned to STA.
        %                       Not applicable for AP.
        %     IsMLD           - Flag indicating whether associated STA or AP is
        %                       a multilink device (MLD)
        %     EnhancedMLMode  - Scalar indicating mode of enhanced multilink
        %                       operation (MLO). Applicable only when the
        %                       associated STA is an MLD. 0 and 1 represents STR
        %                       and EMLSR respectively. Not applicable for AP.
        %     NumEMLPadBytes  - Number of padding bytes to include in initial
        %                       control frame (ICF). Applicable only for associated
        %                       EMLSR STA.
        %     Bandwidth       - Bandwidth to use for communication with associated
        %                       STA or AP. If AP and STA are MLDs, this field is a
        %                       scalar or vector. Units are in MHz.
        %   In case of mesh nodes, this property contains information of peer mesh
        %   nodes. Applicable fields for mesh are NodeID (peer mesh node ID),
        %   MACAddress (peer mesh node MAC address) and DeviceID (device ID on
        %   which a mesh node is connected to its peer mesh node).
        AssociationInfo = struct([]);

        %AssociationInfoTemplate Structure template of the association information
        %to be stored
        AssociationInfoTemplate = struct('NodeID', 0, 'MACAddress', '000000000000', ...
            'DeviceID', 0, 'AID', 0, 'IsMLD', false, 'EnhancedMLMode', 0, ...
            'NumEMLPadBytes', 0, 'Bandwidth', 0, 'MaxSupportedStandard', wlan.internal.networkUtils.Std80211be);

        %IncludeRxVector Flag indicating whether to include Rx vector in
        %MPDUDecoded event notification data
        %   Specify this property as true to include Rx vector in MPDUDecoded event
        %   notification data. The default value is false.
        IncludeRxVector = false;

        %MeshNeighbors Mesh neighbor node IDs
        %   This property is an array of the IDs of mesh nodes that are identified
        %   as neighbors.
        MeshNeighbors;

        %InterferenceFidelity Fidelity level of modeling the interference
        %   This property is an array of size 1-by-N, where N is the number
        %   of devices. Each element represents the type of interference
        %   modeling:
        %   0   -   'co-channel'
        %   1   -   'overlapping-adjacent-channel'
        %   2   -   'non-overlapping-adjacent-channel'
        InterferenceFidelity;

        %IsMeshNode Is mesh capable node
        IsMeshNode = false;

        %IsAPNode Is an AP node
        IsAPNode = false;

        %IsMLDNode Is the node with multilink device (MLD)
        IsMLDNode = false;

        %MaxSupportedStandard Max supported standard
        %   Specify this property as an integer value in the range [0, 5]
        %   representing standards 802.11a, 802.11g, 802.11n, 802.11ac,
        %   802.11ax, 802.11be. This property takes the enumerated constant
        %   values from wlan.internal.networkUtils.Std80211XX.
        MaxSupportedStandard = wlan.internal.networkUtils.Std80211be;

        %RxAppLatencyStats Latency statistics captured at the application layer of
        %the receiver
        %   This property is an array of structures. Each element represents a
        %   structure for a unique source. Each structure includes the following
        %   fields:
        %     SourceNodeID              - Node identifier of a specific source
        %     ReceivedPackets           - Total number of packets received from the
        %                                 source node
        %     ReceivedBytes             - Total number of bytes received from the
        %                                 source node
        %     AggregatePacketLatency    - Total latency of all packets received
        %                                 from the source node in seconds
        %     AveragePacketLatency      - Average latency of all packets received
        %                                 from the source node in seconds
        RxAppLatencyStats = struct([]);

        %RxAppLatencyStatsTemplate Structure template of the latency information to
        %be stored
        %   Upon receiving the initial packet from a source node, the node creates
        %   a structure and stores the node ID of the source Node by initializing
        %   the SourceNodeID field. The object then adds this structure to the
        %   RxAppLatencyStats property.
        RxAppLatencyStatsTemplate = struct('SourceNodeID', 0, ...
            'ReceivedPackets', 0, ...
            'ReceivedBytes', 0, ...
            'AggregatePacketLatency', 0, ...
            'AveragePacketLatency', 0);
    end

    properties (Access = protected)
        %IsPHYAbstracted PHY abstraction is true
        IsPHYAbstracted = true;

        %RxInfo Receiver information
        RxInfo;

        %PacketInfo Structure containing App packet info
        PacketInfo = wlan.internal.sls.defaultAppPacket;

        %PHYIndication Structure containing the indication passed between
        %MAC and PHY
        PHYIndication;

        %NumAssociatedSTAsPerDevice Number of associated STAs on each device if
        %node is an AP
        NumAssociatedSTAsPerDevice = 0;

        %HasStarted Indicates if the node has started running in the simulation
        HasStarted = false;
    end

    properties(Hidden)
        %FullBufferTrafficEnabled Indicates whether full buffer traffic is enabled
        FullBufferTrafficEnabled = false;

        %FullBufferAppPacket Structure containing full buffer application packet info
        FullBufferAppPacket = wlan.internal.sls.defaultAppPacket(true);

        %FullBufferContextTemplate Template structure for full buffer traffic context 
        FullBufferContextTemplate = struct('DestinationID', 0, 'DestinationName', '', 'MACQueuePacket', [], 'SourceDeviceIdx', 1, 'IsGroupAddress', false, 'IsMLDDestination', false);

        %FullBufferContext Structure containing context for full buffer traffic
        FullBufferContext;

        %PacketIDCounter Packet ID counter for full buffer traffic
        PacketIDCounter;

        %CurrentTime Current simulation time
        CurrentTime = 0;

        % Scalar value indicating total packet latency at application layer for all
        % the applications.
        TotalPacketLatency = 0;

        %MaxUsers Maximum number of users a node can support in downlink MU
        MaxUsers = 9;

        %FullBufferPacketSize Packet size for full buffer traffic
        FullBufferPacketSize = 1500;

        % Frame formats
        NonHT;
        HTMixed;
        VHT;
        HE_SU;
        HE_EXT_SU;
        HE_MU;
        HE_TB;
        EHT_SU;

        % Data is empty
        PacketTypeEmpty;

        % Data containing IQ samples (Full MAC + Full PHY)
        DataTypeIQData;

        % Data containing MAC PPDU bits (Full MAC + ABS PHY)
        DataTypeMACFrameBits;

        % Data containing MAC configuration structure (ABS MAC + ABS PHY)
        DataTypeMACFrameStruct;

        % Maximum number of STAs that can be associated on an AP device
        AssociationLimit = 2007;
    end

    properties (Hidden, Constant)
        PHYAbstractionMethod_Values = ["tgax-evaluation-methodology", "tgax-mac-calibration", "none"];

        BroadcastID = 65535;
    end

    methods
        function obj = wlanNode(varargin)
            % Name-value pair check
            coder.internal.errorIf(mod(nargin,2) == 1, 'wlan:ConfigBase:InvalidPVPairs');

            % Initialize with defaults, in case user doesn't configure
            obj.SharedMAC = wlan.internal.sharedMAC.empty;
            obj.MAC = wlan.internal.edcaMAC.empty;
            obj.MeshBridge = wlan.internal.meshBridge(obj.MAC);
            obj.ReceiveFrequency = zeros(1, 0);
            obj.DeviceConfig = wlanDeviceConfig;
            obj.FullBufferContext = obj.FullBufferContextTemplate;

            % Initialize constant properties
            obj.NonHT = wlan.internal.frameFormats.NonHT;
            obj.HTMixed = wlan.internal.frameFormats.HTMixed;
            obj.VHT = wlan.internal.frameFormats.VHT;
            obj.HE_SU = wlan.internal.frameFormats.HE_SU;
            obj.HE_EXT_SU = wlan.internal.frameFormats.HE_EXT_SU;
            obj.HE_MU = wlan.internal.frameFormats.HE_MU;
            obj.HE_TB = wlan.internal.frameFormats.HE_TB;
            obj.EHT_SU = wlan.internal.frameFormats.EHT_SU;
            obj.PacketTypeEmpty = wlan.internal.networkUtils.PacketTypeEmpty;
            obj.DataTypeIQData = wlan.internal.networkUtils.DataTypeIQData;
            obj.DataTypeMACFrameBits = wlan.internal.networkUtils.DataTypeMACFrameBits;
            obj.DataTypeMACFrameStruct = wlan.internal.networkUtils.DataTypeMACFrameStruct;

            numNodes = 1;
            if nargin > 0
                % Identify number of nodes user intends to create based on
                % Position value
                for idx = 1:2:nargin-1
                    % Search the presence of 'Position' N-V pair argument
                    if strcmp(varargin{idx},"Position")
                        validateattributes(varargin{idx+1}, {'numeric'}, {'nonempty', 'ncols', 3, 'finite'}, mfilename, 'Position');
                        positionValue = varargin{idx+1};
                        numNodes = size(varargin{idx+1}, 1);
                    end
                    % Search the presence of 'Name' N-V pair argument
                    if strcmp(varargin{idx},"Name")
                        nameValue = string(varargin{idx+1});
                    end
                end

                obj = repmat(obj, 1, numNodes);
                for idx = 2:numNodes
                    obj(idx) = wlanNode;
                end

                % Set the configuration of nodes as per the N-V pairs
                for idx = 1:2:nargin-1
                    name = varargin{idx};
                    value = varargin{idx+1};
                    switch (name)
                        case 'Position'
                            % Set position for nodes
                            for j = 1:numNodes
                                obj(j).Position = positionValue(j, :);
                            end
                        case 'Name'
                            % Set name for nodes. If name is not supplied
                            % for all nodes, then leave the trailing nodes
                            % with default names
                            nameCount = min(numel(nameValue), numNodes);
                            for j=1:nameCount
                                obj(j).Name = nameValue(j);
                            end
                        otherwise
                            % Make all the nodes identical by setting same
                            % value for all the configurable properties,
                            % except position and name
                            [obj.(char(name))] = deal(value);
                    end
                end
            end

            coder.internal.errorIf(isa(obj(1).DeviceConfig(1), 'wlanMultilinkDeviceConfig') && numel(obj(1).DeviceConfig) > 1, ...
                'wlan:wlanNode:InvalidNumMLD');

            [obj.IsMLDNode] = deal(isa(obj(1).DeviceConfig(1), 'wlanMultilinkDeviceConfig'));
            validateMultipleOperatingFreq(obj);
            
            appPacketContext = struct(AccessCategory=0, DestinationNodeID=0, DestinationNodeName=""); % App packet context fields
            for idx = 1:numNodes
                % Application

                % Use weak-references for cross-linking handle objects
                objWeakRef = matlab.lang.WeakReference(obj(idx));
                notificationFcn = @(eventName, eventData) objWeakRef.Handle.triggerEvent(eventName, eventData); % Function handle for event notification
                sendPacketFcn = @(packet) objWeakRef.Handle.sendPacketToMAC(packet);                            % Function handle for pushing packets from App into MAC
                obj(idx).Application = wirelessnetwork.internal.trafficManager(obj(idx).ID, sendPacketFcn, ...
                    notificationFcn, PacketContext=appPacketContext, DataAbstraction=obj(idx).MACFrameAbstraction); %#ok<*AGROW>

                % Validate the configuration
                setFrequencies(obj(idx));

                % Mode flags
                obj(idx).IsMeshNode = ~obj(idx).IsMLDNode && any([obj(idx).DeviceConfig(:).IsMeshDevice]);
                obj(idx).IsAPNode = any([obj(idx).DeviceConfig(:).IsAPDevice]);

                if strcmpi(obj(idx).PHYAbstractionMethod, 'none')
                    obj(idx).IsPHYAbstracted = false;
                    obj(idx).PHYTx = wlan.internal.sls.phyTx.empty;
                    obj(idx).PHYRx = wlan.internal.sls.phyRx.empty;
                else
                    obj(idx).PHYTx = wlan.internal.sls.phyTxAbstract.empty;
                    obj(idx).PHYRx = wlan.internal.sls.phyRxAbstract.empty;
                end

                % Flag indicating that the object to be initialized is a copy of first node
                isCopy = idx > 1;
                % Initialize
                init(obj(idx), isCopy);
            end
        end

        function set.PHYAbstractionMethod(obj, value)
            value = validateEnumProperties(obj, 'PHYAbstractionMethod', value);
            obj.PHYAbstractionMethod = value;
        end

        function set.IncludeRxVector(obj, value)
            obj.IncludeRxVector = value;
            updateMACParameter(obj, "IncludeRxVector", value);
        end
    end

    methods
        function associateStations(obj, associatedSTAs, varargin)
        %associateStations Associate stations to WLAN node
        %
        %   associateStations(OBJ,ASSOCIATEDSTAS) associates the list of stations
        %   given in ASSOCIATEDSTAS to the AP node represented by the OBJ.
        %
        %   OBJ is an object of type <a
        %   href="matlab:help('wlanNode')">wlanNode</a>. The Mode must be set to "AP"
        %   within the DeviceConfig property of this object. When multiple devices
        %   are configured for this node object, at least one object in the
        %   DeviceConfig property must have Mode set to "AP".
        %
        %   ASSOCIATEDSTAS is a scalar or a vector of objects corresponding to
        %   STA(s) in the BSS. Each object is of type wlanNode. The Mode must be
        %   set to "STA" within the DeviceConfig property of each of these objects.
        %
        %   associateStations(...,Name=Value) specifies additional name-value
        %   arguments described below. When a name-value argument is not specified,
        %   the function uses its default value.
        %
        %   BandAndChannel        - Band and channel to be used to create a BSS.
        %                           For association between non-multilink device
        %                           (non-MLD) AP and non-MLD STAs or MLD AP and
        %                           non-MLD STAs,
        %                           * Specify the value as a row vector containing
        %                             two elements. The first element represents
        %                             band and accepted values are 2.4, 5 and 6
        %                             (GHz).
        %                             The second element represents any valid
        %                             channel number in the specified band.
        %                           * The default value is automatically determined
        %                             by the node by finding the band and channel
        %                             at AP such that the primary 20 MHz subchannel
        %                             is included in operating frequency range of
        %                             STAs.
        %                           For association between MLD AP and MLD STAs,
        %                           * Specify the value as an N-by-2 matrix with
        %                             each row containing a band and channel
        %                             number.
        %                           * The default value is a matrix that the
        %                             node creates by placing band and channel of
        %                             each configured link of AP in a row.
        %
        %   FullBufferTraffic     - Set full buffer traffic between the AP and
        %                           the given list of stations. Following are the
        %                           allowed values for this parameter:
        %                           "off"   - Full buffer traffic is disabled.
        %                           "on"    - Configures two-way full buffer
        %                                     traffic between the given AP and
        %                                     stations.
        %                           "DL"    - Configures full buffer downlink
        %                                     traffic from AP to stations.
        %                           "UL"    - Configures full buffer uplink
        %                                     traffic from stations to the AP.
        %                           When full buffer traffic is enabled, the packet
        %                           size is 1500 and the access category is 0. If
        %                           full buffer traffic is enabled, custom traffic
        %                           source cannot be added for access category 0
        %                           through <a
        %                           href="matlab:help('wlanNode.addTrafficSource')">addTrafficSource</a>. The default value is
        %                           "off".

            narginchk(2, 6);

            % Validate inputs
            associationNVParams = validateAssociationParams(obj, associatedSTAs, varargin);
            % Find the AP and STA device/link indices on which association must be
            % performed. Also, get the primary20 index in STA and bandwidth used for
            % communication between AP and STA.
            [apDeviceIdx, staDeviceIdx, staPrimary20Idx, commonBandwidth] = wlan.internal.findDevicesToAssociate(obj, associatedSTAs, associationNVParams);

            if ~obj.IsMLDNode % Non-MLD AP
                % Association is done only on one frequency (link)
                numLinks = 1;
            else
                % Association is done on multiple frequencies (links)
                numLinks = numel(obj.DeviceConfig.LinkConfig);
            end

            numSTA = numel(associatedSTAs);
            assocIndices = [];
            % Configure information of AP at associated STA and vice-versa
            for staIdx = 1:numSTA
                staNode = associatedSTAs(staIdx);

                if ~staNode.IsMLDNode
                    % Association is performed with only one AP device and the corresponding AP
                    % device index is present at 'staIdx' index in apDeviceIdx variable.
                    numAssociationsPerSTA = 1;
                    if ~isempty(assocIndices)
                        assocIndices = assocIndices(end)+1;
                    else
                        assocIndices = staIdx;
                    end
                    % Check if non-MLD STA is already associated
                    existingAssociation = false;
                    if ~isempty(obj.AssociationInfo) && any(staNode.ID == [obj.AssociationInfo(:).NodeID])
                        idxLogical = (staNode.ID == [obj.AssociationInfo(:).NodeID]);
                        existingAssociation = strcmp(staNode.MAC.MACAddress, obj.AssociationInfo(idxLogical).MACAddress) && (apDeviceIdx(staIdx) == obj.AssociationInfo(idxLogical).DeviceID);
                    end
                else
                    % Association is performed on multiple links of AP. Get the indices to
                    % access the corresponding AP links from apDeviceIdx variable.
                    numAssociationsPerSTA = numLinks;
                    if ~isempty(assocIndices)
                        assocIndices = assocIndices(end)+1:assocIndices(end)+numLinks;
                    else
                        assocIndices = (staIdx-1)*numLinks + 1:staIdx*numLinks;
                    end
                    % Check if MLD STA is already associated
                    existingAssociation = ~isempty(obj.AssociationInfo) && any(staNode.ID == [obj.AssociationInfo(:).NodeID]);
                end

                coder.internal.errorIf(existingAssociation, 'wlan:wlanNode:ExistingAssociation', staNode.Name, obj.Name);

                % Association information
                associatedSTAMACAddress = repmat('0', numAssociationsPerSTA, 12);
                associatedAPDeviceIDs = zeros(numAssociationsPerSTA, 1);
                associatedSTAAID = 0;
                associatedAPMACAddress = repmat('0', numAssociationsPerSTA, 12);
                associatedSTADeviceIDs = zeros(numAssociationsPerSTA, 1);
                associatedBandwidthInHz = zeros(numAssociationsPerSTA, 1);

                for idx = 1:numAssociationsPerSTA
                    assocIdx = assocIndices(idx);

                    % Check if association limit exceeded at AP
                    if obj.IsMLDNode
                        coder.internal.errorIf(obj.NumAssociatedSTAsPerDevice+1 > obj.AssociationLimit, 'wlan:wlanNode:AssociationLimitExceeded', staNode.Name, obj.Name, obj.AssociationLimit);
                    else
                        coder.internal.errorIf(obj.NumAssociatedSTAsPerDevice(apDeviceIdx(assocIdx))+1 > obj.AssociationLimit, 'wlan:wlanNode:AssociationLimitExceeded', staNode.Name, obj.Name, obj.AssociationLimit);
                    end

                    % Set BSSID property at AP MAC
                    obj.MAC(apDeviceIdx(assocIdx)).BSSID = obj.MAC(apDeviceIdx(assocIdx)).MACAddress;

                    % Add connection info to the station node (BSSID and Basic
                    % rates)
                    bssid = obj.MAC(apDeviceIdx(assocIdx)).MACAddress;
                    basicRates = obj.MAC(apDeviceIdx(assocIdx)).BasicRates;
                    bssColor = obj.MAC(apDeviceIdx(assocIdx)).BSSColor;
                    if obj.IsMLDNode
                        if idx == 1
                            % Assign AID during setup on first link. An AP MLD assigns single AID value
                            % to STA MLD. Reference: Section 35.3.5 of of IEEE P802.11be/D5.0.
                            obj.NumAssociatedSTAsPerDevice = obj.NumAssociatedSTAsPerDevice + 1;
                            associatedSTAAID = obj.NumAssociatedSTAsPerDevice;
                        end
                    else
                        obj.NumAssociatedSTAsPerDevice(apDeviceIdx(assocIdx)) = obj.NumAssociatedSTAsPerDevice(apDeviceIdx(assocIdx)) + 1;
                        associatedSTAAID = obj.NumAssociatedSTAsPerDevice(apDeviceIdx(assocIdx));
                    end

                    % Add connection specific information (BSSID, AID, etc.) to MAC/PHY
                    addConnection(staNode.MAC(staDeviceIdx(assocIdx)), bssid, basicRates, bssColor, associatedSTAAID);
                    addConnection(staNode.PHYRx(staDeviceIdx(assocIdx)), associatedSTAAID, "STA");
                    addConnection(obj.PHYRx(apDeviceIdx(assocIdx)), associatedSTAAID, "AP");

                    % Add rate control information
                    setRateControlContext(obj, staNode, apDeviceIdx(assocIdx), staDeviceIdx(assocIdx), basicRates);

                    % Add primary channel information at AP and STA MAC and phy modules
                    devCfg = getDeviceConfig(obj, apDeviceIdx(assocIdx));
                    [~,primaryChannelFrequency] = wlanNode.getPrimaryChannel(devCfg.BandAndChannel, devCfg.ChannelBandwidth, devCfg.PrimaryChannelIndex);
                    wlan.internal.setPrimaryChannelInfoAtLayers(obj, apDeviceIdx(assocIdx), devCfg.PrimaryChannelIndex, primaryChannelFrequency);
                    wlan.internal.setPrimaryChannelInfoAtLayers(staNode, staDeviceIdx(assocIdx), staPrimary20Idx(assocIdx), primaryChannelFrequency);

                    % Store association information
                    associatedSTAMACAddress(idx, :) = staNode.MAC(staDeviceIdx(assocIdx)).MACAddress;
                    associatedAPDeviceIDs(idx) = apDeviceIdx(assocIdx);
                    associatedAPMACAddress(idx, :) = obj.MAC(apDeviceIdx(assocIdx)).MACAddress;
                    associatedSTADeviceIDs(idx) = staDeviceIdx(assocIdx);
                    associatedBandwidthInHz(idx) = commonBandwidth(assocIdx);

                    % If UL OFDMA is enabled at AP, assume that all the stations in BSS would
                    % support trigger based transmissions. Indicate the stations that AP is
                    % configured to trigger UL OFDMA transmissions.
                    if obj.MAC(apDeviceIdx(assocIdx)).ULOFDMAEnabled
                        staNode.MAC(staDeviceIdx(assocIdx)).ULOFDMAEnabledAtAP = true;
                    end

                    staMLDMode = 0; % 0 indicates STR. Applicable only if STA is an MLD STA.
                    if staNode.IsMLDNode && strcmp(staNode.DeviceConfig.EnhancedMultilinkMode, "EMLSR")
                        staMLDMode = 1; % 1 indicates EMLSR (currently supported enhanced multilink mode)
                        coder.internal.errorIf(obj.DeviceConfig.LinkConfig(apDeviceIdx(assocIdx)).NumTransmitAntennas ~= staNode.MAC(staDeviceIdx(assocIdx)).NumTransmitAntennas, ...
                            'wlan:wlanNode:UnsupportedNumTxAntennasEMLSR', obj.Name, staNode.Name);
                        % In case of EMLSR STA MLD associated to an AP MLD, store medium sync delay
                        % information
                        if obj.IsMLDNode
                            addMediumSyncDelayInfo(staNode.MAC(staDeviceIdx(assocIdx)), round(obj.DeviceConfig.MediumSyncDuration*32e3), ...
                                obj.DeviceConfig.MediumSyncEDThreshold, obj.DeviceConfig.MediumSyncMaxTXOPs);
                        end
                    end
                end

                % Beacon transmissions are not supported at AP associated with an EMLSR STA
                coder.internal.errorIf(staMLDMode && obj.IsMLDNode && ~any([obj.DeviceConfig.LinkConfig(:).BeaconInterval] == inf), ...
                    'wlan:wlanNode:UnsupportedBeaconAPMLD', obj.Name);

                % Add information of STA at AP
                associationInfo = obj.AssociationInfoTemplate;
                associationInfo.NodeID = staNode.ID;
                associationInfo.MACAddress = associatedSTAMACAddress;
                associationInfo.DeviceID = associatedAPDeviceIDs;
                associationInfo.AID = associatedSTAAID;
                associationInfo.IsMLD = staNode.IsMLDNode;
                associationInfo.EnhancedMLMode = staMLDMode;
                associationInfo.NumEMLPadBytes = staNode.SharedMAC.NumPadBytesICF;
                associationInfo.Bandwidth = associatedBandwidthInHz/1e6; % In MHz
                associationInfo.MaxSupportedStandard = staNode.MaxSupportedStandard;
                obj.AssociationInfo = [obj.AssociationInfo associationInfo];

                % Add association information at shared MAC
                if obj.IsMLDNode % AP is an MLD
                    addAssociationInfo(obj.SharedMAC, associationInfo);

                else % Non-MLD
                    % Add required information of STA at non-MLD AP
                    addAssociationInfo(obj.SharedMAC(apDeviceIdx(staIdx)), associationInfo);
                end

                % Configure association information at mesh bridge of AP to handle
                % forwarding from AP.
                if obj.IsAPNode
                    addAssociationInfo(obj.MeshBridge, associationInfo);
                end

                % Add information of AP at STA
                associationInfo.NodeID = obj.ID;
                associationInfo.MACAddress = associatedAPMACAddress;
                associationInfo.DeviceID = associatedSTADeviceIDs;
                associationInfo.AID = 0; % Not applicable for AP
                associationInfo.IsMLD = obj.IsMLDNode;
                associationInfo.EnhancedMLMode = 0; % Not applicable for AP
                associationInfo.NumEMLPadBytes = 0; % Not applicable for AP
                associationInfo.Bandwidth = associatedBandwidthInHz/1e6; % In MHz
                associationInfo.MaxSupportedStandard = obj.MaxSupportedStandard;
                staNode.AssociationInfo = associationInfo;

                % Add association information at shared MAC
                addAssociationInfo(staNode.SharedMAC, associationInfo);
            end

            % Configure full buffer traffic based on the input 'FullBufferTraffic' parameter
            wlan.internal.sls.configureFullBufferTraffic(obj, associationNVParams.FullBufferTraffic, associatedSTAs);
        end

        function addTrafficSource(obj, trafficSource, varargin)
        %addTrafficSource Add data traffic source to WLAN node
        %
        %   addTrafficSource(OBJ,TRAFFICSOURCE) adds a data source object,
        %   TRAFFICSOURCE, to the node, OBJ, that generates broadcast traffic.
        %
        %   TRAFFICSOURCE is an object of type <a
        %   href="matlab:help('networkTrafficOnOff')">networkTrafficOnOff</a>,
        %   <a href="matlab:help('networkTrafficFTP')">networkTrafficFTP</a>, <a
        %   href="matlab:help('networkTrafficVideoConference')">networkTrafficVideoConference</a> or <a href="matlab:help('networkTrafficVoIP')">networkTrafficVoIP</a>.
        %   "GeneratePacket" property of TRAFFICSOURCE object is not applicable
        %   and is overridden according to the "MACFrameAbstraction" value of the 
        %   node, OBJ.
        %
        %   OBJ is an object of type <a
        %   href="matlab:help('wlanNode')">wlanNode</a>.
        %
        %   addTrafficSource(...,Name=Value) specifies additional name-value
        %   arguments described below. When a name-value argument is not specified,
        %   the function uses its default value.
        %
        %   DestinationNode       - Specify the destination node of the traffic
        %                           as an object of type <a
        %                           href="matlab:help('wlanNode')">wlanNode</a>. If you do
        %                           not specify this argument, the source
        %                           node broadcasts its traffic. If source is a STA
        %                           MLD operating in EMLSR mode or an AP MLD
        %                           with at least one associated EMLSR STA, you
        %                           must specify this argument.
        %
        %   AccessCategory        - Access category of the generated traffic,
        %                           specified as an integer in the range [0, 3].
        %                           The four possible values respectively
        %                           correspond to the Best Effort, Background,
        %                           Video, and Voice access categories.
        %                           The default value is 0.

            validateattributes(obj, {'wlanNode'}, {'scalar'}, mfilename, 'obj');
            % Dynamic traffic addition is not supported
            coder.internal.errorIf(obj.HasStarted,"wlan:wlanNode:NotSupportedOperation","addTrafficSource");
            % Validate the traffic parameters
            upperLayerDataInfo = validateTrafficParams(obj, trafficSource, varargin);

            % Do not allow custom traffic if full buffer traffic is enabled
            coder.internal.errorIf(obj.FullBufferTrafficEnabled && (upperLayerDataInfo.AccessCategory == 0), 'wlan:wlanNode:FullBufferEnabled', obj.Name);

            % Add the traffic source to the application traffic manager
            for idx = 1:numel(upperLayerDataInfo)
                addTrafficSource(obj.Application, trafficSource, upperLayerDataInfo(idx));
            end
        end

        function addMeshPath(obj, destinationNode, varargin)
        %addMeshPath Add mesh path to WLAN node
        %
        %   addMeshPath(OBJ,DESTINATIONNODE) specifies that the destination node,
        %   DESTINATIONNODE, is an immediate mesh receiver for the source node,
        %   OBJ.
        %
        %   OBJ is an object of type <a href="matlab:help('wlanNode')">wlanNode</a>.
        %
        %   DESTINATIONNODE is an object of type <a href="matlab:help('wlanNode')">wlanNode</a>.
        %
        %   addMeshPath(OBJ,DESTINATIONNODE,MESHPATHNODE) specifies the mesh node,
        %   MESHPATHNODE, to which the source node, OBJ, sends the packets in an
        %   attempt to communicate with the destination node, DESTINATIONNODE.
        %
        %   MESHPATHNODE is an object of type <a href="matlab:help('wlanNode')">wlanNode</a>, specifying one of these
        %   roles:
        %       * Next hop node - If the destination node is a mesh node, then this
        %         input specifies the next hop node. The next hop node refers to an
        %         immediate mesh receiver to which the source node forwards the
        %         packets.
        %
        %       * Proxy mesh gate - If the destination node is a non-mesh node,
        %         then this input specifies the proxy mesh gate. The proxy mesh
        %         gate refers to any mesh node that can forward packets to a
        %         non-mesh node.
        %
        %   addMeshPath(...,Name=Value) specifies additional name-value arguments
        %   described below. When a name-value argument is not specified, the
        %   function uses its default value.
        %
        %   SourceBandAndChannel       - Band and channel on which the source
        %                                node must transmit packets to the next hop
        %                                node. Specify this input as a vector of
        %                                two values. The first value must be 2.4,
        %                                5, or 6 and the second value must be a
        %                                valid channel number in the band.
        %
        %                                The input uses this default configuration:
        %                                * If the mesh path node is the next hop
        %                                  node, the function selects the common
        %                                  band and channel between the source node
        %                                  and next hop node. If there are multiple
        %                                  common band-channel pairs, you must
        %                                  specify a value for this input.
        %                                * If the mesh path node is the proxy mesh
        %                                  gate, the function selects the band and
        %                                  channel belonging to a mesh device. If
        %                                  there are multiple mesh devices, you
        %                                  must specify a value for this input.
        %
        %   MeshPathBandAndChannel     - Band and channel on which the mesh path
        %                                node must receive the packets. Specify
        %                                this input as a vector of two values. The
        %                                first value must be 2.4, 5, or 6 and the
        %                                second value must be a valid channel
        %                                number in the band.
        %
        %                                The input uses this default configuration:
        %                                * If the mesh path node is the next hop
        %                                  node, the function selects the common
        %                                  band and channel between the source node
        %                                  and next hop node.
        %                                * If the mesh path node is the proxy mesh
        %                                  gate, the function selects the band and
        %                                  channel belonging to a mesh device. If
        %                                  there are multiple mesh devices, you
        %                                  must specify a value for this input.
        %
        %   DestinationBandAndChannel  - Band and channel on which the destination
        %                                node should receive the packets. Specify
        %                                this input as a vector of two values. The
        %                                first value must be 2.4, 5, or 6 and the
        %                                second value must be a valid channel
        %                                number in the band.
        %
        %                                The input uses this default configuration:
        %                                * If the destination node is a mesh node,
        %                                  the function selects the band and
        %                                  channel belonging to a mesh device. If
        %                                  there are multiple mesh devices, you
        %                                  must specify a value for this input.
        %                                * If the destination node is a non-mesh
        %                                  node and only one device is present, the
        %                                  function selects the band and channel of
        %                                  that device in the node. If there are
        %                                  multiple devices, you must specify a
        %                                  value for this input.

            narginchk(2, 9);

            validateattributes(obj, {'wlanNode'}, {'scalar'}, mfilename, 'obj');
            % Validate the input parameters
            [meshPathNode, params] = validateMeshPathParams(obj, nargin, destinationNode, varargin{:});
            [sourceDeviceID, meshPathDevID, destDevID] = findDeviceIDs(obj, destinationNode, meshPathNode, params);

            destinationID = destinationNode.ID;
            destinationAddress = wlan.internal.nodeID2MACAddress([destinationNode.ID destDevID]);
            meshPathAddress = wlan.internal.nodeID2MACAddress([meshPathNode.ID meshPathDevID]);

            if destinationNode.IsMeshNode % Forwarding information
                % Add next hop (meshPathAddress) information
                addPath(obj.MeshBridge, destinationID, destinationAddress, meshPathAddress, sourceDeviceID);
                addPeerMeshSTAInfo(obj, meshPathNode, sourceDeviceID, meshPathDevID);
                setBiDirectionalPaths(obj, destinationNode, meshPathNode, sourceDeviceID, destDevID, meshPathDevID);
                % Set rate control context
                basicRates = [6 12 24]; % No configuration option for mesh nodes yet
                setRateControlContext(obj, meshPathNode, sourceDeviceID, meshPathDevID, basicRates);
            else % Proxy information
                % Add proxy mesh (meshPathAddress) information
                addProxyInfo(obj.MeshBridge, destinationID, destinationAddress, meshPathAddress);
            end

            % Update Mesh Neighbors in MAC
            obj.MAC(sourceDeviceID).MeshNeighbors{end+1} = [meshPathNode.ID meshPathDevID];
            meshPathNode.MAC(meshPathDevID).MeshNeighbors{end+1} = [obj.ID sourceDeviceID];
        end

        function update(obj, deviceID, varargin)
        %update Update configuration of WLAN Node
        %
        %   update(OBJ,Name=Value) updates the configuration of the node. You can
        %   update the following properties through this method.
        %
        %   CWMin     - Minimum range of contention window for the four
        %               access categories (ACs), specified as a vector of
        %               four integers in the range [1, 1023]. The four 
        %               entries are the minimum ranges for the Best Effort,
        %               Background, Video, and Voice ACs, respectively.
        %
        %   CWMax     - Maximum range of contention window for the four ACs,
        %               specified as a vector of four integers in the range
        %               [1, 1023]. The four entries are the maximum ranges
        %               for the Best Effort, Background, Video, and Voice
        %               ACs, respectively.
        %
        %   AIFS      - Arbitrary interframe space values for the four ACs,
        %               specified as a vector of four integers in the range
        %               [2, 15]. The entries of the vector represent the AIFS
        %               values, in slots, for the Best Effort, Background,
        %               Video, and Voice ACs, respectively.
        %
        %   update(OBJ,DEVICEID,Name=Value) updates the configuration for a
        %   specific device in a non-MLD node or a specific link in a device in an
        %   MLD node. For a non-MLD node, DEVICEID is a scalar and specifies the
        %   device ID which is the array index in the DeviceConfig property of the
        %   OBJ. If DEVICEID is not specified, the default value is 1. For an MLD
        %   node, DEVICEID is a vector in which the first element specifies the
        %   device ID. The second element specifies the link ID which is the array
        %   index in the LinkConfig property of DeviceConfig property of the OBJ.
        %   If DEVICEID is not specified, the default value is [1 1].

            validateattributes(obj, {'wlanNode'}, {'scalar'}, mfilename, 'obj');
            coder.internal.errorIf((nargin == 1), 'wlan:wlanNode:NoUpdate');

            linkID = 1; % Default
            if mod(nargin, 2) == 1
                nvPairs = [{deviceID}, varargin];
                deviceID = 1;
            else
                nvPairs = varargin;
                if obj.IsMLDNode
                    coder.internal.errorIf(numel(deviceID)~=2, 'wlan:wlanNode:UpdateInvalidDeviceID');
                    devID = deviceID;
                    deviceID = devID(1);
                    linkID = devID(2);
                    coder.internal.errorIf(~(isnumeric(deviceID) && isreal(deviceID) && (deviceID==floor(deviceID))) ... % Integer check
                        || deviceID < 1 || deviceID > numel(obj.DeviceConfig), ...
                        'wlan:wlanNode:UpdateInvalidLink', 'First', 'multilink devices');
                    coder.internal.errorIf(~(isnumeric(linkID) && isreal(linkID) && (linkID==floor(linkID))) ... % Integer check
                        || linkID < 1 || linkID > numel(obj.DeviceConfig.LinkConfig), ...
                        'wlan:wlanNode:UpdateInvalidLink', 'Second', 'links in the specified multilink device');
                else
                    validateattributes(deviceID, {'numeric'}, {'scalar', 'integer', 'positive', '<=', numel(obj.DeviceConfig)}, '', 'device ID');
                end
            end

            if ~obj.IsMLDNode % Non MLD
                cfg = obj.DeviceConfig(deviceID);
                macIdx = deviceID;
            else % MLD
                cfg = obj.DeviceConfig(deviceID).LinkConfig(linkID);
                macIdx = linkID;
            end

            numParamUpdates = 0;
            for idx = 1:2:numel(nvPairs)
                switch nvPairs{idx}
                    case {'CWMin', 'CWMax', 'AIFS'}
                        cfg.(nvPairs{idx}) = nvPairs{idx+1};
                        obj.MAC(macIdx).(nvPairs{idx}) = nvPairs{idx+1};
                        numParamUpdates = numParamUpdates + 1;
                    otherwise
                        coder.internal.errorIf(true, 'wlan:wlanNode:InvalidUpdateParameter');
                end
            end

            if ~obj.IsMLDNode % Non MLD
                obj.DeviceConfig(deviceID) = cfg;
                obj.SharedMAC(macIdx).IsEDCAParamsUpdated = true;
                obj.SharedMAC(macIdx).EDCAParamsCount = numParamUpdates;
            else % MLD
                obj.DeviceConfig(deviceID) = updateLinkConfig(obj.DeviceConfig(deviceID), linkID, cfg);
                obj.SharedMAC.IsEDCAParamsUpdated(:, macIdx) = true;
                obj.SharedMAC.EDCAParamsCount(macIdx) = numParamUpdates;
            end
        end

        function stats = statistics(obj, varargin)
        %statistics Returns statistics of WLAN Node
        %
        %   [STATISTICS] = statistics(OBJ) returns the statistics as a structure
        %   for the given node object, OBJ. If the input OBJ is a vector, the
        %   output is a row vector of structures corresponding to statistics of each
        %   node.
        %
        %   STATISTICS is a structure containing the statistics of the node. If 
        %   the input OBJ is a vector of nodes, STATISTICS is a vector of 
        %   structures corresponding to the input vector of nodes. For information on 
        %   fields in the structure, see <a href="matlab:helpview('wlan','wlanNodeStatistics')">WLAN Node Statistics</a>.

            % Validate that input is a vector
            validateattributes(obj, {'wlanNode'}, {'vector'}, mfilename, '', 1);

            % Return the output stats as a row vector
            stats = repmat(struct, 1, numel(obj));

            option = [];
            if ~isempty(varargin)
                option = validatestring(varargin{1}, "all", mfilename, '');
            end
            % Calculate the number of unique frequencies
            for idx = 1:numel(obj)
                node = obj(idx);

                % Initialize
                mac = node.MAC;
                phyTx = node.PHYTx;
                phyRx = node.PHYRx;
                meshBridge = node.MeshBridge;

                % App statistics in App sub-structure
                stats(idx).Name = node.Name;
                stats(idx).ID = node.ID;
                stats(idx).App = getAppStats(node, option);

                for deviceID = 1:numel(obj(idx).DeviceConfig)
                    isMLD = isa(obj(idx).DeviceConfig(deviceID), 'wlanMultilinkDeviceConfig');
                    if ~isMLD
                        % MAC statistics in MAC sub-structure
                        stats(idx).MAC(deviceID) = statistics(mac(deviceID), option);
                    else
                        % MLD MAC statistics in MAC sub-structure
                        numLinks = numel(obj(idx).DeviceConfig.LinkConfig);
                        for linkID = 1:numLinks
                            linkMACStats(linkID) = statistics(mac(linkID), option);
                        end
                        stats(idx).MAC(deviceID) = wlanNode.getMLDMACStats(linkMACStats);

                        % MLD per-link statistics in MAC sub-structure when "all" is provided
                        if ~isempty(option)
                            stats(idx).MAC(deviceID).Link = linkMACStats;
                        end
                    end

                    if ~isMLD
                        % PHY statistics in PHY sub-structure. Merge the PHYTx and
                        % PHYRx statistics structures into one structure
                        phyTxStats = statistics(phyTx(deviceID));
                        phyRxStats = statistics(phyRx(deviceID));
                        stats(idx).PHY(deviceID) = cell2struct([struct2cell(phyTxStats); struct2cell(phyRxStats)], [fieldnames(phyTxStats); fieldnames(phyRxStats)]);
                    else
                        % MLD PHY statistics in PHY sub-structure.
                        numLinks = numel(obj(idx).DeviceConfig.LinkConfig);
                        for linkID = 1:numLinks
                            phyTxStats = statistics(phyTx(linkID));
                            phyRxStats = statistics(phyRx(linkID));
                            linkPHYStats(linkID) = cell2struct([struct2cell(phyTxStats); struct2cell(phyRxStats)], [fieldnames(phyTxStats); fieldnames(phyRxStats)]);
                        end
                        stats(idx).PHY(deviceID) = wlanNode.getMLDPHYStats(linkPHYStats);

                        % MLD per-link statistics in PHY sub-structure when "all" is provided
                        if ~isempty(option)
                            stats(idx).PHY(deviceID).Link = linkPHYStats;
                        end
                    end

                    % Mesh statistics in Mesh sub-structure
                    stats(idx).Mesh(deviceID) = statistics(meshBridge, deviceID);
                end
            end
        end
    end

    methods(Hidden)
        function nextInvokeTime = run(obj, currentTime)
            %run Runs the WLAN node
            %
            %   NEXTINVOKETIME = run(OBJ, CURRENTTIME) runs the
            %   functionality of WLAN node and returns the time at which
            %   this node should be run again.
            %
            %   NEXTINVOKETIME is the time in seconds at which the run
            %   function must be invoked again. The simulator may invoke
            %   this function earlier than this time if required, for
            %   example when a packet is added to the receive buffer of
            %   this node.
            %
            %   OBJ is an object of type <a
            %   href="matlab:help('wlanNode')">wlanNode</a>.
            %
            %   CURRENTTIME is the current simulation time in seconds.

            % Initialize
            nextInvokeTimes = zeros(1, 0);
            nextIdx = 1;

            % Update simulation time
            obj.CurrentTime = currentTime;
            currentTimeInNS = round(currentTime*1e9); % current time in nano seconds
            % Check for event listeners
            if ~obj.HasStarted
                % Perform the actions needed in the initial run of the node
                checkEventListeners(obj);
                obj.HasStarted = true;
            end

            if obj.FullBufferTrafficEnabled
                nextAppInvokeTime = Inf;
            else
                % Run the application layer
                nextAppInvokeTime = run(obj.Application, currentTimeInNS);
            end

            for deviceIdx = 1:obj.NumDevices
                % Rx buffer has data to be processed
                if obj.ReceiveBufferIdx(deviceIdx) ~= 0
                    rxBuffer = obj.ReceiveBuffer{deviceIdx};
                    for idx = 1:obj.ReceiveBufferIdx(deviceIdx)
                        % Process the received data
                        deviceInvokeTime = runLayers(obj, deviceIdx, currentTimeInNS, rxBuffer{idx});
                        nextInvokeTimes(nextIdx:nextIdx+1) = deviceInvokeTime;
                        % Increment the nextInvokeTimes vector index by 2 to
                        % fill MAC and PHY invoke times in next iteration.
                        nextIdx = nextIdx+2;
                    end
                    obj.ReceiveBufferIdx(deviceIdx) = 0;
                else % Rx buffer has no data to process
                    % Update the time to the MAC and PHY layers
                    deviceInvokeTime = runLayers(obj, deviceIdx, currentTimeInNS, []);
                    nextInvokeTimes(nextIdx:nextIdx+1) = deviceInvokeTime;
                    nextIdx = nextIdx+2;
                end
            end

            % Get the next invoke time
            nextInvokeTime = min([nextInvokeTimes nextAppInvokeTime]);
            nextInvokeTime = round(nextInvokeTime/1e9, 9);
        end

        function init(obj, isCopy)
            %init Initialize and setup the node stack

            % Extract node ID
            nodeIdx = obj.ID;

            if ~obj.IsMLDNode
                % In case of non-MLD node, devCfg contains per device configuration
                devCfg = obj.DeviceConfig;
            else
                % In case of MLD node, devCfg contains per link configuration
                devCfg = obj.DeviceConfig.LinkConfig;
            end

            validateMACAndPHYAbstraction(obj, devCfg);

            % Number of devices in the node in case of non-MLD node. Number of links in
            % case of MLD node.
            numDevices = numel(devCfg);

            if obj.IsMLDNode
                % Get EMLSR padding delay and transition delay in nanoseconds
                emlsrPaddingDelay = round(obj.DeviceConfig.EnhancedMultilinkPaddingDelay*1e9); % in nanoseconds
                emlsrTransitionDelay = round(obj.DeviceConfig.EnhancedMultilinkTransitionDelay*1e9); % in nanoseconds

                % In case of MLD, create one shared MAC
                maxSubframes = max([devCfg(:).MPDUAggregationLimit]);
                obj.SharedMAC = wlan.internal.sharedMAC(obj.DeviceConfig.TransmitQueueSize, maxSubframes, ...
                    ShortRetryLimit=obj.DeviceConfig.ShortRetryLimit, ...
                    NumLinks=numDevices, IsMLD=obj.IsMLDNode, ...
                    EMLPaddingDelay=emlsrPaddingDelay, EMLTransitionDelay=emlsrTransitionDelay);

                % Store AC to link mapping information in shared MAC
                acs = cell(0, numDevices);
                % Store band and channel of each link
                bandsAndChannels = zeros(numDevices, 2);
                primaryChannelNums = zeros(numDevices, 1);
                primaryChannelFreqs = zeros(numDevices, 1);
                primaryChannelIndices = zeros(numDevices, 1);
                % Get the information of AC to link mapping, band and channel and primary
                % channel from each link config
                for linkIdx = 1:numel(obj.DeviceConfig.LinkConfig)
                    linkCfg = obj.DeviceConfig.LinkConfig(linkIdx);
                    acs{linkIdx} = linkCfg.MappedACs;
                    bandsAndChannels(linkIdx, :) = linkCfg.BandAndChannel;
                    primaryChannelIndices(linkIdx) = getPrimaryChannelIndex(obj, linkCfg);
                    [primaryChannelNums(linkIdx), primaryChannelFreqs(linkIdx)] = wlanNode.getPrimaryChannel(linkCfg.BandAndChannel, ...
                        linkCfg.ChannelBandwidth, primaryChannelIndices(linkIdx));
                end
                obj.SharedMAC.Link2ACMap = acs;
                obj.SharedMAC.BandAndChannel = bandsAndChannels;
                obj.SharedMAC.PrimaryChannel = primaryChannelNums; % This information is needed at AP for Beacon fields

                sharedMAC = obj.SharedMAC;

                % Store the parameters which are applicable per device in case of non-MLD
                % and configured common to all links in case of MLD
                txQueueSize = obj.DeviceConfig.TransmitQueueSize;
                isMeshDevice = obj.DeviceConfig.IsMeshDevice;
                isAPDevice = obj.DeviceConfig.IsAPDevice;
            end

            % For the following properties, assign defaults because these capabilities
            % are not supported in case of MLD. These values are updated with device
            % configuration values later in case of non-MLD.
            maxMUStations = 1;
            dlOfdmaFrameSequence = 2;
            bssColor = 0;
            obssPDThreshold = -82;
            obj.InterferenceFidelity = zeros(1,numDevices); % Type of interference modeling

            % Configure and add the devices with MAC and PHY layers
            for devIdx = 1:numDevices
                isEMLSRSTA = false;
                % Get number of space time streams
                if obj.IsMLDNode && strcmp(obj.DeviceConfig.Mode, "STA") && ...
                        strcmp(obj.DeviceConfig.EnhancedMultilinkMode, "EMLSR") % EMLSR STA
                    isEMLSRSTA = true;
                end

                % Configure the rate control algorithm at MAC for DL transmissions from the device
                if strcmp(devCfg(devIdx).RateControl,'fixed')
                    % Create a new rate control object for each node and each device/link,
                    % because rate control is a handle class.
                    rateControlAlgorithm = wlan.internal.rateControlFixed;
                elseif strcmp(devCfg(devIdx).RateControl,'auto-rate-fallback')
                    % Create a new rate control object for each node and each device/link,
                    % because rate control is a handle class.
                    rateControlAlgorithm = wlan.internal.rateControlARF;
                else % Custom rate control
                    if isCopy || isObjectReused(devCfg(devIdx).RateControl)
                        rateControlAlgorithm = copy(devCfg(devIdx).RateControl);
                        if ~obj.IsMLDNode
                            obj.DeviceConfig(devIdx).RateControl = rateControlAlgorithm;
                        else
                            linkCfg = obj.DeviceConfig.LinkConfig(devIdx);
                            linkCfg.RateControl = rateControlAlgorithm;
                            obj.DeviceConfig = updateLinkConfig(obj.DeviceConfig, devIdx, linkCfg);
                        end
                    else
                        rateControlAlgorithm = devCfg(devIdx).RateControl;
                    end
                end

                % Configure the rate control algorithm at MAC for UL transmissions triggered by the device
                ulRateControlAlgorithm = wlan.internal.rateControlFixed;

                % Configure the power control algorithm at MAC
                powerControl = devCfg(devIdx).PowerControl;
                assert(strcmp(powerControl, 'FixedPower'));
                powerControlAlgorithm = wlan.internal.powerControlFixed(Power=devCfg(devIdx).TransmitPower);

                % Initialize the scheduler
                macScheduler = wlan.internal.schedulerRoundRobin;

                % Initialize values related to EMLSR
                EMLSRListenAntennas = 0;

                % Determine whether UL OFDMA is enabled
                ulOFDMAEnabled = false;
                % DL and UL MU OFDMA are not yet supported in an MLD. Hence, check whether
                % it is an MLD node.
                if ~obj.IsMLDNode
                    % EnableUplinkOFDMA flag is applicable only for an AP when the
                    % TransmissionFormat is either HE-SU or HE-MU-OFDMA
                    if strcmp(devCfg(devIdx).Mode, "AP") && any(strcmp(devCfg(devIdx).TransmissionFormat, ["HE-SU", "HE-MU-OFDMA"])) && ...
                            devCfg(devIdx).EnableUplinkOFDMA
                        ulOFDMAEnabled = true;
                        % Full MAC frame generation and decoding is not supported for triggered
                        % multiuser transmissions
                        coder.internal.errorIf(~obj.MACFrameAbstraction && devCfg(devIdx).EnableUplinkOFDMA, ...
                            'wlan:wlanNode:UnsupportedMACFrameAbstractionForOFDMA');
                    end

                    % In case of non-MLD, create one shared MAC for each device.
                    primaryChannelIdx = getPrimaryChannelIndex(obj, devCfg(devIdx));
                    [primaryChannelNum,primaryChannelFreq] = wlanNode.getPrimaryChannel(devCfg(devIdx).BandAndChannel, devCfg(devIdx).ChannelBandwidth, primaryChannelIdx);
                    sharedMAC = wlan.internal.sharedMAC(devCfg(devIdx).TransmitQueueSize, devCfg(devIdx).MPDUAggregationLimit, ...
                        ShortRetryLimit=devCfg(devIdx).ShortRetryLimit, ...
                        NumLinks=1, IsMLD=obj.IsMLDNode, BandAndChannel=devCfg(devIdx).BandAndChannel, ...
                        PrimaryChannel=primaryChannelNum); % This information is needed at AP for Beacon fields
                    obj.SharedMAC(devIdx) = sharedMAC;

                    % Set rate control context
                    setDeviceConfig(rateControlAlgorithm, devCfg(devIdx), devIdx);
                    setDeviceConfig(ulRateControlAlgorithm, devCfg(devIdx), devIdx);

                    isMeshDevice = devCfg(devIdx).IsMeshDevice;
                    isAPDevice = devCfg(devIdx).IsAPDevice;
                    txQueueSize = devCfg(devIdx).TransmitQueueSize;
                    maxMUStations = devCfg(devIdx).MaxMUStations;
                    dlOfdmaFrameSequence = devCfg(devIdx).DLOFDMAFrameSequence;
                    bssColor = 0;
                    obssPDThreshold = -82;
                    if ~isMeshDevice
                        if isAPDevice % AP node
                            bssColor = devCfg(devIdx).BSSColor;
                        end
                        obssPDThreshold = devCfg(devIdx).OBSSPDThreshold;
                    end
                    numTransmitAntennas = devCfg(devIdx).NumTransmitAntennas;
                    numReceiveAntennas = numTransmitAntennas;
                else
                    % Set rate control context
                    setDeviceConfig(rateControlAlgorithm, obj.DeviceConfig, devIdx);
                    setDeviceConfig(ulRateControlAlgorithm, obj.DeviceConfig, devIdx);

                    if isEMLSRSTA
                        numTransmitAntennas = sum([obj.DeviceConfig.LinkConfig(:).NumTransmitAntennas]);
                        EMLSRListenAntennas = devCfg(devIdx).NumTransmitAntennas;
                        numReceiveAntennas = EMLSRListenAntennas; % Initial value
                    else
                        numTransmitAntennas = devCfg(devIdx).NumTransmitAntennas;
                        numReceiveAntennas = numTransmitAntennas;
                    end
                    primaryChannelIdx = primaryChannelIndices(devIdx);
                    primaryChannelFreq = primaryChannelFreqs(devIdx);
                end

                % Validate channel bandwidth supported for beacon transmission
                if obj.IsMLDNode
                    isBeaconEnabled = strcmp(obj.DeviceConfig.Mode, "AP") && isfinite(devCfg(devIdx).BeaconInterval);
                else
                    isBeaconEnabled = ~strcmp(devCfg(devIdx).Mode, "STA") && isfinite(devCfg(devIdx).BeaconInterval);
                end
                coder.internal.errorIf(isBeaconEnabled && ~strcmp(obj.PHYAbstractionMethod, "none") && devCfg(devIdx).ChannelBandwidth ~= 20e6, ...
                    'wlan:wlanNode:InvalidBandwidthForBeacon');

                % MAC layer
                mac = wlan.internal.edcaMAC(NodeID=nodeIdx, ...
                        DeviceID=devIdx, ...
                        ChannelBandwidth=devCfg(devIdx).ChannelBandwidth/1e6, ...
                        TransmissionFormat=wlan.internal.networkUtils.getFrameFormatConstant(devCfg(devIdx).TransmissionFormat), ...
                        MPDUAggregation=wlan.internal.sls.isMPDUAggregationEnabled(obj, devIdx), ...
                        DisableAck=devCfg(devIdx).DisableAck, ...
                        CWMin=devCfg(devIdx).CWMin, ...
                        CWMax=devCfg(devIdx).CWMax, ...
                        AIFS=devCfg(devIdx).AIFS, ...
                        TXOPLimit=devCfg(devIdx).TXOPLimit*32e3, ...
                        NumTransmitAntennas=numTransmitAntennas, ...
                        NumReceiveAntennas=numReceiveAntennas, ...
                        DisableRTS=devCfg(devIdx).DisableRTS, ...
                        RTSThreshold=devCfg(devIdx).RTSThreshold,...
                        Use6MbpsForControlFrames=devCfg(devIdx).Use6MbpsForControlFrames, ...
                        BasicRates=devCfg(devIdx).BasicRates, ...
                        RateControl=rateControlAlgorithm, ...
                        PowerControl=powerControlAlgorithm, ...
                        FrameAbstraction=obj.MACFrameAbstraction, ...
                        IsMeshDevice=isMeshDevice, ...
                        IsAPDevice=isAPDevice, ...
                        SharedMAC=sharedMAC, ...
                        Scheduler=macScheduler, ...
                        SharedEDCAQueues=sharedMAC.EDCAQueues, ...
                        MaxMUStations=maxMUStations, ...
                        DLOFDMAFrameSequence=dlOfdmaFrameSequence, ...
                        ULOFDMAEnabled=ulOFDMAEnabled, ...
                        ULRateControl=ulRateControlAlgorithm, ...
                        MaxSubframes=devCfg(devIdx).MPDUAggregationLimit, ...
                        MaxQueueLength=txQueueSize, ...
                        SIFSTime=16e3, ...
                        IncludeRxVector=obj.IncludeRxVector, ...
                        BSSColor=bssColor, ...
                        OBSSPDThreshold=obssPDThreshold, ...
                        BeaconInterval=devCfg(devIdx).BeaconInterval, ...
                        InitialBeaconOffset=devCfg(devIdx).InitialBeaconOffset, ...
                        IsEMLSRSTA=isEMLSRSTA, ...
                        NumEMLSRListenAntennas=EMLSRListenAntennas, ...
                        MaxSupportedStandard=obj.MaxSupportedStandard, ...
                        PrimaryChannelIndex=primaryChannelIdx);

                switch devCfg(devIdx).InterferenceModeling
                    case 'co-channel'
                        obj.InterferenceFidelity(devIdx) = 0;
                    case 'overlapping-adjacent-channel'
                        obj.InterferenceFidelity(devIdx) = 1;
                    otherwise % 'non-overlapping-adjacent-channel'
                        obj.InterferenceFidelity(devIdx) = 2;
                end

                if strcmp(obj.PHYAbstractionMethod,'none')
                    if obj.InterferenceFidelity(devIdx) == 0
                        % Modeling co-channel interference
                        osf = 1;
                    else
                        % Oversampling waveform for modeling ACI
                        osf = 1.125;
                    end
                    phyTx = wlan.internal.sls.phyTx( ...
                        IsNodeTypeAP=isAPDevice, ...
                        TxGain=devCfg(devIdx).TransmitGain, ...
                        DeviceID=devIdx, ...
                        OversamplingFactor=osf, ...
                        OperatingBandwidth=devCfg(devIdx).ChannelBandwidth/1e6, ... % Configured bandwidth
                        PrimaryChannelIndex=primaryChannelIdx);
                    phyRx = wlan.internal.sls.phyRx(NodeID=nodeIdx, ...
                        EDThreshold=devCfg(devIdx).EDThreshold, ...
                        RxGain=devCfg(devIdx).ReceiveGain, ...
                        NoiseFigure=devCfg(devIdx).NoiseFigure, ...
                        ChannelBandwidth=devCfg(devIdx).ChannelBandwidth/1e6, ...
                        DeviceID=devIdx, ...
                        NumReceiveAntennas=numReceiveAntennas, ...
                        PrimaryChannelFrequency=primaryChannelFreq, ...
                        PrimaryChannelIndex=primaryChannelIdx);
                else
                    % Physical layer transmitter
                    phyTx = wlan.internal.sls.phyTxAbstract( ...
                        IsNodeTypeAP=isAPDevice, ...
                        TxGain=devCfg(devIdx).TransmitGain, ...
                        MaxSubframes=devCfg(devIdx).MPDUAggregationLimit, ...
                        ChannelBandwidth=devCfg(devIdx).ChannelBandwidth/1e6, ...
                        DeviceID=devIdx, ...
                        PrimaryChannelIndex=primaryChannelIdx);

                    % Physical layer receiver
                    phyRx = wlan.internal.sls.phyRxAbstract(NodeID=nodeIdx, ...
                        EDThreshold=devCfg(devIdx).EDThreshold, ...
                        RxGain=devCfg(devIdx).ReceiveGain, ...
                        AbstractionType=obj.PHYAbstractionMethod, ...
                        NoiseFigure = devCfg(devIdx).NoiseFigure, ...
                        SubcarrierSubsampling = 4, ...
                        MaxSubframes=devCfg(devIdx).MPDUAggregationLimit, ...
                        ChannelBandwidth=devCfg(devIdx).ChannelBandwidth/1e6, ...
                        DeviceID=devIdx, ...
                        BSSColor=bssColor, ...
                        OBSSPDThreshold=obssPDThreshold, ...
                        NumReceiveAntennas=numReceiveAntennas, ...
                        PrimaryChannelFrequency=primaryChannelFreq, ...
                        PrimaryChannelIndex=primaryChannelIdx);
                end

                % Use weak-references for cross-linking handle objects
                objWeakRef = matlab.lang.WeakReference(obj);
                % Register function handle at MAC for pushing packets from node to MAC queue
                mac.PushPacketToQueueFcn = @(destIdx, ac) objWeakRef.Handle.pushPacketToQueue(destIdx, ac);
                % Register function handle at MAC for handling packets after MAC processing
                mac.HandleReceivePacketFcn = @(deviceID, packetToApp, isMeshDevice, macAddress) ...
                    objWeakRef.Handle.handleReceivedPacket(deviceID, packetToApp, isMeshDevice, macAddress);

                % Register function handle at MAC to notify the PHY mode change to phy Rx
                mac.SetPHYModeFcn = @(phyMode)phyRx.setPHYMode(phyMode);
                % Register function handle at MAC to send CCA reset request to phy Rx
                mac.ResetPHYCCAFcn = @phyRx.resetPHYCCA;
                % Register function handle at MAC to send Trigger request to phy Rx
                if ~obj.IsMLDNode && devCfg(devIdx).EnableUplinkOFDMA || strcmp(devCfg(devIdx).TransmissionFormat, "HE-MU-OFDMA")
                    mac.SendTrigRequestFcn = @(expiryTime)phyRx.handleTrigRequest(expiryTime);
                end
                % Register function handle at MAC to notify phy Rx about medium sync delay (MSD) timer
                % start and reset at MAC
                mac.MSDTimerStartFcn = @(msdOFDMEDThreshold)phyRx.msdTimerStart(msdOFDMEDThreshold);
                mac.MSDTimerResetFcn = @phyRx.msdTimerReset;
                % Register function handle at MAC to notify phy Rx about the number of active receive antennas
                mac.SetNumRxAntennasFcn = @(numAntennas)phyRx.updateNumActiveRxAntennas(numAntennas);

                % Register function handle for events notification
                eventNotificationFcn = @(eventName, eventData) objWeakRef.Handle.triggerEvent(eventName, eventData);
                mac.EventNotificationFcn = eventNotificationFcn;
                phyTx.EventNotificationFcn = eventNotificationFcn;
                phyRx.EventNotificationFcn = eventNotificationFcn;

                % Register function handle for sending packets from PHY Tx
                sendPacketFcn = @(packet) objWeakRef.Handle.addToTxBuffer(packet);
                phyTx.SendPacketFcn = sendPacketFcn;

                % Add the device
                addDevice(obj, devIdx, devCfg(devIdx).BandAndChannel, mac, phyTx, phyRx);

                if ~obj.IsMLDNode
                    % In case of non-MLD, store EDCA MAC layer object in the corresponding
                    % shared MAC layer object.
                    obj.SharedMAC(devIdx).MAC = mac;
                end
            end

            if obj.IsMLDNode % MLD node
                meshTTL = 31; % Assign default value as mesh is not supported in MLD
            else % Non-MLD
                meshTTL = [devCfg(:).MeshTTL];
            end

            % Mesh bridge
            obj.MeshBridge = wlan.internal.meshBridge(obj.MAC, MeshTTL=meshTTL, SharedMAC=obj.SharedMAC);

            maxSubframes = obj.MAC(1).MaxSubframes;

            wlanSignal = wirelessnetwork.internal.wirelessPacket;
            wlanSignal.Metadata = wlan.internal.sls.defaultMetadata(obj.MaxUsers, maxSubframes);
            obj.PHYIndication = struct(MessageType=0, Vector=wlan.internal.sls.defaultTxVector);

            for idx = 1:numDevices
                % Generate a separate address for each device in the node
                macAddressHex = wlan.internal.nodeID2MACAddress([obj.ID idx]);
                obj.MAC(idx).MACAddress = macAddressHex;
            end

            % Initialize the receiving buffers for each device within the node. The
            % corresponding frequencies for each device are stored in
            % 'ReceiveFrequency'.
            obj.ReceiveBuffer = cell(numDevices, 1);
            obj.ReceiveBufferIdx = zeros(1, numDevices);

            % Initialize receiver information
            obj.RxInfo = struct(ID=0, Position=[0 0 0], Velocity=[0 0 0]);

            % Initialize association information
            if obj.IsMLDNode
                % An MLD node currently supports only one multi-link device. So, initialize
                % as a scalar.
                obj.NumAssociatedSTAsPerDevice = 0;
            else
                obj.NumAssociatedSTAsPerDevice = zeros(numDevices, 1);
            end

            if obj.IsMLDNode
                % In case of multi link, store the EDCA MAC layer objects in the shared
                % MAC layer object.
                obj.SharedMAC.MAC = obj.MAC;
                % Generate an MLD MAC address by setting the device index input to 0
                obj.SharedMAC.MLDMACAddress = wlan.internal.nodeID2MACAddress([obj.ID 0]);
                % The AID value assigned to a non-AP MLD associated with AP MLD is in the
                % range of 1-2006. Reference: Section 9.4.1.8 of IEEE P802.11be/D5.0.
                % Hence, set the maximum number of associations to 2006.
                obj.AssociationLimit = 2006;
            end
        end

        function [flag, rxInfo] = isPacketRelevant(obj, packet)
        %isPacketRelevant Return flag to indicate if the input packet
        %is relevant for this node
        %
        %   [FLAG, RXINFO] = isPacketRelevant(OBJ, PACKET) checks
        %   whether the packet, PACKET, is relevant for this node,
        %   before applying channel model. If the output FLAG is true,
        %   the packet is of interest and the RXINFO specifies the
        %   receiver information needed for applying channel on the
        %   incoming packet, PACKET.
        %
        %   FLAG is a logical scalar value indicating whether to invoke
        %   channel or not.
        %
        %   The object function returns the output, RXINFO, and is
        %   valid only when the FLAG value is 1 (true). The structure
        %   of this output contains these fields:
        %
        %   ID       - Node identifier of the receiver
        %   Position - Current receiver position in Cartesian coordinates,
        %              specified as a real-valued vector of the form [x y
        %              z]. Units are in meters.
        %   Velocity - Current receiver velocity (v) in the x-, y-, and
        %              z-directions, specified as a real-valued vector of
        %              the form [vx vy vz]. Units are in meters per second.
        %
        %   OBJ is an object of type <a
        %   href="matlab:help('wlanNode')">wlanNode</a>.
        %
        %   PACKET is the packet received from the channel. This is a
        %   structure of type <a
        %   href="matlab:help('wlanNode/WLANSignal')">WLANSignal</a>.

            % Initialize
            flag = false;
            rxInfo = obj.RxInfo;

            % If it is self-packet (transmitted by this node) do not get this
            % packet
            if packet.TransmitterID == obj.ID
                return;
            end

            for deviceID = 1:obj.NumDevices
                flag = wlan.internal.sls.isFrequencyOverlapping(obj, packet, deviceID);
                if flag
                    rxInfo.ID = obj.ID;
                    rxInfo.Position = obj.Position;
                    rxInfo.Velocity = obj.Velocity;
                    % Use the maximum number of receive antennas
                    rxInfo.NumReceiveAntennas = obj.MAC(deviceID).NumTransmitAntennas;
                    break;
                end
            end
        end

        function pushReceivedData(obj, packet)
            %pushReceivedData Push the received packet to node
            %
            % OBJ is an object of type wlanNode, nrGNB, nrUE,
            % bluetoothLENode, bluetoothNode, or any other node type
            % derived from this class.
            %
            % PACKET is the received packet. It is a structure of the
            % format <a href="matlab:help('wirelessnetwork.internal.wirelessPacket')">wirelessPacket</a>

            if isempty(packet)
                return;
            end

            % Copy the received packet to the device (network interface)
            % buffers of the node only if the frequency is overlapping
            for deviceID = 1:obj.NumDevices
                if wlan.internal.sls.isFrequencyOverlapping(obj, packet, deviceID)
                    obj.ReceiveBufferIdx(deviceID) = obj.ReceiveBufferIdx(deviceID) + 1;
                    obj.ReceiveBuffer{deviceID}{obj.ReceiveBufferIdx(deviceID)} = packet;
                end
            end
        end

        function isPacketQueued = sendPacketToMAC(obj, packet)
        %sendPacketToMAC Send a packet received from user to MAC queue

            [macQueuePacket, sourceDeviceIdx] = wlan.internal.sls.addDestinationInfo(obj, packet);  % Add destination information to the packet
            isGroupAddress = wlan.internal.sls.isGroupAddress(macQueuePacket.ReceiverAddress);            % Check if the destination is a groupcast address
            isMLDDestination = wlan.internal.sls.isDestinationMLD(obj, packet.DestinationNodeID);   % Check if the destination is an MLD

            isPacketQueued = wlan.internal.sls.pushDataToMAC(obj, macQueuePacket, sourceDeviceIdx, isGroupAddress, isMLDDestination);
        end

        function pushPacketToQueue(obj, destIdx, ac)
        %pushPacketToQueue Generate and push application packet to the MAC queue
            wlan.internal.sls.fillPacketInMACQueue(obj, destIdx, ac);
        end

        function setAPFullBufferTrafficContext(obj, associatedStations)
        %setAPFullBufferTrafficContext Initialize full buffer traffic context
            numStations = numel(associatedStations);
            obj.PacketIDCounter = [obj.PacketIDCounter, zeros(1, numStations)];
            obj.FullBufferTrafficEnabled = true;
            fullBufferAppPacket = obj.FullBufferAppPacket;
            fullBufferAppPacket.PacketLength = obj.FullBufferPacketSize;
            fullBufferAppPacket.Packet = ones(obj.FullBufferPacketSize, 1);
            fullBufferAppPacket.AccessCategory = 0;
            fullBufferAppPacket.SourceNodeID = obj.ID;
            fullBufferContext = repmat(obj.FullBufferContextTemplate, 1, numStations);

            for idx = 1:numStations
                fullBufferAppPacket.DestinationNodeID = associatedStations(idx).ID;
                fullBufferAppPacket.DestinationNodeName = string(associatedStations(idx).Name);
                fullBufferContext(idx).DestinationID = associatedStations(idx).ID;
                fullBufferContext(idx).DestinationName = string(associatedStations(idx).Name);
                fullBufferContext(idx).IsMLDDestination = associatedStations(idx).IsMLDNode;   % Check if the destination is an MLD
                [fullBufferContext(idx).MACQueuePacket, fullBufferContext(idx).SourceDeviceIdx] = wlan.internal.sls.addDestinationInfo(obj, fullBufferAppPacket);  % Add destination information to the packet
                fullBufferContext(idx).IsGroupAddress = wlan.internal.sls.isGroupAddress(fullBufferContext(idx).MACQueuePacket.ReceiverAddress);            % Check if the destination is a groupcast address
            end
            numDestinations = numel(obj.FullBufferContext);
            startIdx = 1 + numDestinations * ~((numDestinations==1) && (obj.FullBufferContext.DestinationID==0));
            endIdx = startIdx+numStations-1;
            obj.FullBufferContext(startIdx:endIdx) = fullBufferContext;
        end

        function setSTAFullBufferTrafficContext(obj, associatedAP)
        %setSTAFullBufferTrafficContext Initialize full buffer traffic context
            obj.PacketIDCounter = 0;
            obj.FullBufferTrafficEnabled = true;

            % Full buffer MAC packet and its context
            fullBufferContext = obj.FullBufferContextTemplate;
            fullBufferAppPacket = obj.FullBufferAppPacket;
            fullBufferAppPacket.PacketLength = obj.FullBufferPacketSize;
            fullBufferAppPacket.Packet = ones(obj.FullBufferPacketSize, 1);
            fullBufferAppPacket.AccessCategory = 0;
            fullBufferAppPacket.SourceNodeID = obj.ID;
            fullBufferAppPacket.DestinationNodeID = associatedAP.ID;
            fullBufferAppPacket.DestinationNodeName = associatedAP.Name;
            fullBufferContext.IsMLDDestination = (obj.IsMLDNode) && (associatedAP.IsMLDNode);   % Check if the destination is an MLD
            fullBufferContext.DestinationID = associatedAP.ID;
            fullBufferContext.DestinationName = associatedAP.Name;
            [fullBufferContext.MACQueuePacket, fullBufferContext.SourceDeviceIdx] = wlan.internal.sls.addDestinationInfo(obj, fullBufferAppPacket);  % Add destination information to the packet
            fullBufferContext.IsGroupAddress = wlan.internal.sls.isGroupAddress(fullBufferContext.MACQueuePacket.ReceiverAddress);            % Check if the destination is a groupcast address
            obj.FullBufferContext = fullBufferContext;
        end

        function packetID = packetIDCounter(obj, destIdx)
        %packetIDCounter Returns packet ID for app packets (used for full buffer
        %traffic)
            obj.PacketIDCounter(destIdx) = obj.PacketIDCounter(destIdx) + 1;
            packetID = obj.PacketIDCounter(destIdx);
        end

        function updateMACParameter(obj, parameter, value)
            for idx = 1:numel(obj.MAC)
                obj.MAC(idx).(parameter) = value;
            end
        end

        function value = getPrimaryChannelIndex(obj, cfg)
            value = ones(1, numel(cfg));
            for idx = 1:numel(cfg)
                if obj.IsMLDNode
                    isAP = strcmp(obj.DeviceConfig.Mode, "AP");
                else
                    isAP = strcmp(cfg(idx).Mode, "AP");
                end

                if isAP && cfg(idx).ChannelBandwidth > 20e6
                    value(idx) = cfg(idx).PrimaryChannelIndex;
                end
            end
        end

        function kpiValue = kpi(srcNode, destNode, kpiString, options)
            %kpi Returns key performance indicators (KPIs) for WLAN nodes
            %
            %   KPIVALUE = kpi(SRCNODE, DESTNODE, KPISTRING, OPTIONS) returns the KPI
            %   value, KPIVALUE, specified by KPISTRING, from the source node, SRCNODE
            %   to the destination node, DESTNODE. The function calculates KPIs where
            %   either the source node or the destination node can be a vector,
            %   enabling multiple KPI calculations across different node pairs.
            %
            %   KPIVALUE is the calculated value of the specified kpi string,
            %   KPISTRING. If you provide multiple source-destination pairs, kpiValue
            %   is a row vector containing the KPI value for each pair.
            %
            %   SRCNODE is a scalar or a vector of objects of type <a
            %   href="matlab:help('wlanNode')">wlanNode</a>.
            %
            %   DESTNODE is a scalar or a vector of objects of type <a
            %   href="matlab:help('wlanNode')">wlanNode</a>.
            %
            %   KPISTRING specifies the name of the KPI to measure, specified as
            %   "throughput", "PLR" or "latency".
            %
            %   OPTIONS is a structure with the following fields -
            %
            %   Layer           - This field specifies the layer at which you want to
            %                     measure the KPI. Valid values of this field are: "MAC"
            %                     and "App".
            %
            %   BandAndChannel  - Specify this field when you want to measure the KPI
            %                     exclusively for a specific Band and Channel between
            %                     the source and destination node. If you do not
            %                     specify BandAndChannel, this object function
            %                     calculates the total KPI between the source node and
            %                     destination node.

            arguments
                srcNode (1,:) wlanNode
                destNode (1,:) wlanNode
                kpiString (1,1) string {mustBeMember(kpiString,["throughput", "PLR", "latency"])}
                options.Layer (1,1) string {mustBeMember(options.Layer, ["MAC", "App"])}
                options.BandAndChannel (1,2)
            end

            % Check that Layer parameter has been specified in options
            coder.internal.errorIf(~isfield(options, "Layer"), 'wlan:wlanNode:KPIMustHaveLayerNV');

            bandAndChannel = [];
            % Check if BandAndChannel parameter has been specified in options
            if isfield(options, "BandAndChannel")
                bandAndChannel = options.BandAndChannel;
            end

            % Validate invalid input combinations
            invalidCombination = false;
            layer = options.Layer;
            if strcmp(layer, "App") && ~strcmp(kpiString, "latency")
                invalidCombination = true;
            elseif strcmp(layer, "MAC") && (strcmp(kpiString, "latency"))
                invalidCombination = true;
            end

            coder.internal.errorIf(invalidCombination,...
                'wlan:wlanNode:KPIInvalidInputCombination', kpiString, layer);

            coder.internal.errorIf(~isempty(bandAndChannel) && (strcmp(kpiString, "latency")),...
                'wlan:wlanNode:UnsupportedLinkLevelLatency');

            validateKPINodes(srcNode, destNode, bandAndChannel);

            kpiValue = [];
            if strcmp(kpiString, 'throughput')
                if ~isempty(bandAndChannel)
                    kpiValue = calculateLinkThroughput(srcNode, destNode, bandAndChannel);
                else
                    kpiValue = calculateThroughput(srcNode, destNode);
                end
            elseif strcmp(kpiString, 'PLR')
                if ~isempty(bandAndChannel)
                    kpiValue = calculateLinkPLR(srcNode, destNode, bandAndChannel);
                else
                    kpiValue = calculatePLR(srcNode, destNode);
                end
            elseif strcmp(kpiString, 'latency')
                kpiValue = calculateLatency(srcNode, destNode);
            end
        end
    end

    methods (Access = protected)
        function addDevice(obj, deviceID, bandAndChannel, mac, phyTx, phyRx)
            %addDevice Add a device to the node
            %
            %   addDevice(OBJ, DEVICEID, BANDANDCHANNEL, MAC, PHYTX,
            %   PHYRX) adds a device to the node with the given MAC and
            %   PHY objects at the specified band and channel,
            %   BANDANDCHANNEL.
            %
            %   OBJ is an object of type <a
            %   href="matlab:help('wlanNode')">wlanNode</a>
            %
            %   DEVICEID is the identifier of the device.
            %
            %   BANDANDCHANNEL is the operating band and channel number of
            %   the device. It is a cell array of vector in the format
            %   {[x, y]} where x = band, y = channel number. The value of x
            %   can be 2.4, 5, or 6. The value of y can be any valid
            %   channel number.
            %
            %   MAC is an object of type <a
            %   href="matlab:help('wlan.internal.edcaMAC')">edcaMAC</a>.
            %   This object contains methods and properties related to WLAN
            %   MAC layer.
            %
            %   PHYTX is an abstracted PHY object of type <a
            %   href="matlab:help('wlan.internal.sls.phyTxAbstract')">phyTxAbstract</a>.
            %   This object contains methods and properties related to WLAN
            %   PHY transmitter.
            %
            %   PHYRX is an abstracted PHY object of type <a
            %   href="matlab:help('wlan.internal.sls.phyRxAbstract')">phyRxAbstract</a>.
            %   This object contains methods and properties related to WLAN
            %   PHY receiver.

            % Validate the frequency
            frequency = wlanChannelFrequency(bandAndChannel(2), bandAndChannel(1));

            % Update the device information
            mac.OperatingFrequency = frequency;
            phyTx.OperatingFrequency = frequency;
            phyRx.OperatingFrequency = frequency;
            obj.MAC(deviceID) = mac;
            obj.PHYTx(deviceID) = phyTx;
            obj.PHYRx(deviceID) = phyRx;

            if ~obj.IsMLDNode
                cfg = obj.DeviceConfig(deviceID);
            else
                cfg = obj.DeviceConfig.LinkConfig(deviceID);
            end
            obj.ReceiveBandwidth(deviceID) = cfg.ChannelBandwidth;
        end

        function nextInvokeTime = runLayers(obj, deviceIdx, currentTime, rxPacket)
            %runLayers Runs the layers of the node with the received signal
            %and returns the next invoke time in microseconds

            % MAC object
            mac = obj.MAC(deviceIdx);
            % PHY Tx object
            phyTx = obj.PHYTx(deviceIdx);
            % PHY Rx object
            phyRx = obj.PHYRx(deviceIdx);

            % Invoke the PHY receiver module
            [nextPHYInvokeTime, indicationToMAC, frameToMAC] = run(phyRx, currentTime, rxPacket);

            % Invoke the MAC layer
            [nextMACInvokeTime, macReqToPHY, frameToPHY] = run(mac, currentTime, indicationToMAC, frameToMAC);

            % Invoke the PHY transmitter module (pass MAC requests to PHY)
            if (macReqToPHY.MessageType == wlan.internal.phyPrimitives.TxStartRequest) || ~isempty(frameToPHY)
                run(phyTx, currentTime, macReqToPHY, frameToPHY);
            end

            % Return the next invoke times of PHY and MAC modules
            nextInvokeTime = [nextPHYInvokeTime nextMACInvokeTime];
        end

        function addToTxBuffer(obj, packet)
            %addToTxBuffer Adds the packet to Tx buffer

            packet.TransmitterID = obj.ID;
            packet.TransmitterPosition = obj.Position;
            packet.TransmitterVelocity = obj.Velocity;
            obj.TransmitterBuffer = [obj.TransmitterBuffer packet];
        end

        function triggerEvent(obj, eventName, eventData)
            %triggerEvent Trigger the event to notify all the listeners

            if event.hasListener(obj, eventName)
                eventData.CurrentTime = obj.CurrentTime;
                eventDataObj = wirelessnetwork.internal.nodeEventData;
                eventDataObj.Data = eventData;
                notify(obj, eventName, eventDataObj);
            end
        end

        function receiveAppData(obj, macPacket)
            %receiveAppData Calculate the received application packet latency

            obj.PacketLatencyIdx = obj.PacketLatencyIdx + 1;
            obj.PacketLatency(obj.PacketLatencyIdx) = round(obj.CurrentTime - macPacket.PacketGenerationTime, 9); % In seconds
            % Update the packet latency
            obj.TotalPacketLatency = obj.TotalPacketLatency + obj.PacketLatency(obj.PacketLatencyIdx);
            packetInfo = obj.PacketInfo;
            packetInfo.AccessCategory = macPacket.AC;
            if ~isempty(macPacket.Data)
                packetInfo.Packet = macPacket.Data;
            end
            packetInfo.PacketLength = macPacket.MSDULength;
            packetInfo.PacketID = macPacket.PacketID;
            packetInfo.PacketGenerationTime = macPacket.PacketGenerationTime;
            packetInfo.SourceNodeID = wlan.internal.macAddress2NodeID(macPacket.SourceAddress);
            obj.Application.receivePacket(packetInfo);

            % Update the RxAppLatencyStats
            sourceNodeID = wlan.internal.macAddress2NodeID(macPacket.SourceAddress);
            destinationNodeID = wlan.internal.macAddress2NodeID(macPacket.DestinationAddress);
            % Check to see if the packet was broadcasted
            if ~(destinationNodeID(1) == obj.BroadcastID)
                % Check if the RxAppLatencyStats has a structure to store latency values from
                % source node
                if (isempty(obj.RxAppLatencyStats) || ~any([obj.RxAppLatencyStats.SourceNodeID] == sourceNodeID(1)))
                    % Add a structure to store latency values from source node
                    rxAppLatencyStats = obj.RxAppLatencyStatsTemplate;
                    rxAppLatencyStats.SourceNodeID = sourceNodeID(1);
                    obj.RxAppLatencyStats = [obj.RxAppLatencyStats rxAppLatencyStats];
                end
                % Update the values in the structure associated with the source node.
                idxLogical = ([obj.RxAppLatencyStats.SourceNodeID] == sourceNodeID(1));
                obj.RxAppLatencyStats(idxLogical).AggregatePacketLatency = ...
                    obj.RxAppLatencyStats(idxLogical).AggregatePacketLatency + ...
                    round(obj.CurrentTime - macPacket.PacketGenerationTime, 9);
                obj.RxAppLatencyStats(idxLogical).ReceivedPackets = ...
                    obj.RxAppLatencyStats(idxLogical).ReceivedPackets + 1;
                obj.RxAppLatencyStats(idxLogical).ReceivedBytes = ...
                    obj.RxAppLatencyStats(idxLogical).ReceivedBytes + packetInfo.PacketLength;
            end
        end

        function handleReceivedPacket(obj, deviceID, packetToApp, isMeshDevice, macAddress)
            %handleReceivedPacket Handle each decoded MSDU received from MAC

            % Check whether the immediate destination is group address or not
            isGroupAddr = wlan.internal.sls.isGroupAddress(packetToApp.ReceiverAddress);
            isFourAddressFrame = packetToApp.FourAddressFrame;

            % Broadcast
            if isGroupAddr
                if isMeshDevice % Packet received on mesh
                    sourceID = wlan.internal.macAddress2NodeID(packetToApp.MeshSourceAddress);
                    % Check whether the packet is already received or not
                    isDuplicate = isDuplicateFrame(obj.MeshBridge, packetToApp.MeshSourceAddress, ...
                        packetToApp.MeshSequenceNumber, deviceID);

                    if ~isDuplicate && ~(sourceID(1) == obj.ID)
                        receiveAppData(obj, packetToApp);
                        % Forward the packet in all the MAC devices if
                        % remaining mesh forward hops are greater than 1
                        forwardAppData(obj.MeshBridge, packetToApp, deviceID, isGroupAddr);
                    end
                else
                    sourceID = wlan.internal.macAddress2NodeID(packetToApp.SourceAddress);
                    if sourceID(1) ~= obj.ID
                        % Send to app if this node is not the source of packet
                        receiveAppData(obj, packetToApp);
                    end
                end
            else
                if isFourAddressFrame && isMeshDevice % Four address frame received on mesh
                    % Check whether the packet is already received or not
                    isDuplicate = isDuplicateFrame(obj.MeshBridge, packetToApp.MeshSourceAddress, ...
                        packetToApp.MeshSequenceNumber, deviceID);
                end

                % Non-duplicate mesh packet (MSDU)
                if isFourAddressFrame && isMeshDevice && ~isDuplicate
                    % Packet reached mesh destination address (DA)
                    if strcmp(macAddress, packetToApp.MeshDestinationAddress) && ...
                            strcmp(packetToApp.DestinationAddress, packetToApp.MeshDestinationAddress)
                        % Give packet to application layer if the mesh DA is final DA
                        receiveAppData(obj, packetToApp);
                    else
                        forwardAppData(obj.MeshBridge, packetToApp, deviceID, isGroupAddr);
                    end
                elseif ~isMeshDevice % Non-mesh packet
                    if strcmp(macAddress, packetToApp.DestinationAddress)
                        % Give packet to application layer if it is destined to this node
                        receiveAppData(obj, packetToApp);
                    elseif obj.IsAPNode
                        isGroupAddr = wlan.internal.sls.isGroupAddress(packetToApp.DestinationAddress);
                        if isGroupAddr
                            % Give broadcast packet received by AP to application layer
                            receiveAppData(obj, packetToApp);
                        end
                        forwardAppData(obj.MeshBridge, packetToApp, deviceID, isGroupAddr);
                    end
                end
            end
        end

        function appStats = getAppStats(obj, varargin)
        %getAppStats Get application statistics

            appStats = statistics(obj.Application);

            perTrafficSourceStats = appStats.TrafficSources;
            appStats = rmfield(appStats, 'TrafficSources');
            % Fill Destinations sub-structure if "all" is provided in input
            if ~isempty(varargin) && strcmp(varargin{1}, "all") && ~isempty(perTrafficSourceStats)
                [destinationIDs, tsIndices] = unique([perTrafficSourceStats(:).DestinationNodeID]);
                destinationNames = [perTrafficSourceStats(tsIndices).DestinationNodeName];
                numDestinations = numel(destinationIDs);
                appStats.Destinations = repmat(struct('NodeID', [], ...
                    'NodeName', [], 'TransmittedPackets', 0, ...
                    'TransmittedBytes', 0), 1, numDestinations);

                for dstIdx = 1:numDestinations
                    appStats.Destinations(dstIdx).NodeID = destinationIDs(dstIdx);
                    appStats.Destinations(dstIdx).NodeName = destinationNames(dstIdx);

                    % Loop over each traffic source and add the stats number to
                    % the corresponding destination
                    for idx=1:numel(perTrafficSourceStats)
                        trafficSourceStat = perTrafficSourceStats(idx);
                        appDestinationID = trafficSourceStat.DestinationNodeID;
                        if appDestinationID == destinationIDs(dstIdx)
                            appStats.Destinations(dstIdx).TransmittedPackets = ...
                                appStats.Destinations(dstIdx).TransmittedPackets + trafficSourceStat.TransmittedPackets;
                            appStats.Destinations(dstIdx).TransmittedBytes = ...
                                appStats.Destinations(dstIdx).TransmittedBytes + trafficSourceStat.TransmittedBytes;
                        end
                    end
                end
            end

            if obj.FullBufferTrafficEnabled
                appStats.TransmittedPackets = sum(obj.PacketIDCounter);
                appStats.TransmittedBytes = appStats.TransmittedPackets*obj.FullBufferPacketSize;
                numDestinations = numel(obj.FullBufferContext);
                % Fill Destinations sub-structure if "all" is provided in input
                if ~isempty(varargin) && strcmp(varargin{1}, "all")
                    appStats.Destinations = repmat(struct('NodeID', [], ...
                        'NodeName', [], 'TransmittedPackets', 0, ...
                        'TransmittedBytes', 0), 1, numDestinations);
                    for idx = 1:numDestinations
                        appStats.Destinations(idx).NodeID = obj.FullBufferContext(idx).DestinationID;
                        appStats.Destinations(idx).NodeName = obj.FullBufferContext(idx).DestinationName;
                        appStats.Destinations(idx).TransmittedPackets = obj.PacketIDCounter(idx);
                        appStats.Destinations(idx).TransmittedBytes = obj.PacketIDCounter(idx)*obj.FullBufferPacketSize;
                    end
                end
            end
        end

        function setFrequencies(obj)
        %setFrequencies Sets the frequencies from the given band and
        %channel values for each device/link config

            if ~obj.IsMLDNode % Non-MLD
                obj.NumDevices = numel(obj.DeviceConfig);
            else % MLD
                % Consider each link as a device
                obj.NumDevices = obj.DeviceConfig.NumLinks;
                % Validate the multilink device
                validateConfig(obj.DeviceConfig);
            end

            for idx = 1:obj.NumDevices
                if ~obj.IsMLDNode % Non-MLD
                    cfg = obj.DeviceConfig(idx);
                    % Validate device config
                    cfg = validateConfig(cfg);
                    % Assign the validated device config back to DeviceConfig property.
                    obj.DeviceConfig(idx) = cfg;
                else % MLD
                    cfg = obj.DeviceConfig.LinkConfig(idx);
                end

                % Calculate frequencies from band and channel
                obj.ReceiveFrequency(idx) = wlanChannelFrequency(cfg.BandAndChannel(2), cfg.BandAndChannel(1));
            end
        end

        function [sourceDeviceID, meshPathDevID, destDevID] = findDeviceIDs(obj, destinationNode, meshPathNode, params)
        %findDeviceIDs Returns the device IDs for source, destination, and
        %mesh path nodes.

            % Extract the user given parameter values (if any).
            sourceBandAndChannel = params.SourceBandAndChannel;
            destBandAndChannel = params.DestinationBandAndChannel;
            meshPathBandAndChannel = params.MeshPathBandAndChannel;

            if destinationNode.IsMeshNode % Forwarding path information
            % * Source node & mesh path node (next hop) are neighbors
            % * Destination node may or may not be neighbor of next hop node

                if isempty(sourceBandAndChannel) && isempty(meshPathBandAndChannel)
                    % Source & mesh path node band and channels are not given by the user

                    % Find common mesh operating frequency between the source and mesh path nodes
                    sourceMeshDevIDs = find([obj.MAC(:).IsMeshDevice]);
                    meshPathMeshDevIDs = find([meshPathNode.MAC(:).IsMeshDevice]);
                    commonFreq = intersect(obj.ReceiveFrequency(sourceMeshDevIDs), meshPathNode.ReceiveFrequency(meshPathMeshDevIDs));
                    coder.internal.errorIf(numel(commonFreq) > 1, 'wlan:wlanNode:MultipleCommonMeshBandAndChannel');
                    coder.internal.errorIf(isempty(commonFreq), 'wlan:wlanNode:NoCommonMeshBandAndChannel');
                    % Find device IDs
                    sourceDeviceID = find(commonFreq == obj.ReceiveFrequency);
                    meshPathDevID = find(commonFreq == meshPathNode.ReceiveFrequency);

                elseif isempty(meshPathBandAndChannel)
                    % Mesh path node band and channel is not given by the user, source band and
                    % channel is given by the user

                    % Find source device ID
                    sourceDeviceID = wlanNode.getDeviceID(obj, sourceBandAndChannel);
                    coder.internal.errorIf(isempty(sourceDeviceID), 'wlan:wlanNode:InvalidSourceBandAndChannel', 'source');
                    % Next hop node should receive packets on the source band and channel.
                    meshPathDevID = wlanNode.getDeviceID(meshPathNode, sourceBandAndChannel);
                    coder.internal.errorIf(isempty(meshPathDevID), 'wlan:wlanNode:InvalidSourceBandAndChannel', 'mesh path');

                elseif isempty(sourceBandAndChannel)
                    % Source band and channel is not given by the user, mesh path node band and
                    % channel is given by the user

                    % Find mesh path device ID
                    meshPathDevID = wlanNode.getDeviceID(meshPathNode, meshPathBandAndChannel);
                    coder.internal.errorIf(isempty(meshPathDevID), 'wlan:wlanNode:InvalidMeshPathBandAndChannel', 'mesh path');
                    % Source node should send packets on the mesh path band and channel.
                    sourceDeviceID = wlanNode.getDeviceID(obj, meshPathBandAndChannel);
                    coder.internal.errorIf(isempty(sourceDeviceID), 'wlan:wlanNode:InvalidMeshPathBandAndChannel', 'source');

                else % Source & mesh path nodes band and channel are given by the user
                    % Find source device ID
                    sourceDeviceID = wlanNode.getDeviceID(obj, sourceBandAndChannel);
                    coder.internal.errorIf(isempty(sourceDeviceID), 'wlan:wlanNode:InvalidSourceBandAndChannel', 'source');
                    % Find mesh path device ID
                    meshPathDevID = wlanNode.getDeviceID(obj, meshPathBandAndChannel);
                    coder.internal.errorIf(isempty(meshPathDevID), 'wlan:wlanNode:InvalidMeshPathBandAndChannel', 'mesh path');

                    % Check if source and mesh path (next hop) band and channels are the same
                    coder.internal.errorIf(~all(sourceBandAndChannel == meshPathBandAndChannel), 'wlan:wlanNode:BandAndChannelMismatch');
                end

                if isempty(destBandAndChannel)
                    % Destination band and channel is not given by the user
                    destDevID = 1;
                    if numel(destinationNode.DeviceConfig) > 1
                        if meshPathNode.ID == destinationNode.ID
                            destDevID = meshPathDevID;
                        else
                            % Find mesh device ID in the destination node if there are multiple devices
                            destDevID = find([destinationNode.MAC(:).IsMeshDevice]);
                            if ~isempty(destDevID) && any(destinationNode.ID == meshPathNode.MeshNeighbors)
                                % If mesh and destination are neighbors, find the common mesh frequency
                                meshPathMeshDevIDs = find([meshPathNode.MAC(:).IsMeshDevice]);
                                commonFreq = intersect(destinationNode.ReceiveFrequency(destDevID), meshPathNode.ReceiveFrequency(meshPathMeshDevIDs));
                                if ~isempty(commonFreq)
                                    [~, ~, destDevID] = intersect(commonFreq, destinationNode.ReceiveFrequency);
                                end
                            end
                            coder.internal.errorIf(numel(destDevID) ~= 1, 'wlan:wlanNode:NeedBandAndChannelParameter', 'DestinationBandAndChannel', 'destination');
                        end
                    end
                else % Destination band and channel is given by the user
                    % Find destination device ID
                    destDevID = wlanNode.getDeviceID(destinationNode, destBandAndChannel);
                    coder.internal.errorIf(isempty(destDevID), 'wlan:wlanNode:InvalidDestinationBandAndChannel');
                end

            else % Proxy mesh information
                % * Destination node is not a mesh node
                % * Mesh path node is a mesh node
                % * Destination node and mesh path node (proxy mesh node) are neighbors
                % * Source node and mesh path node may or may not be neighbors

                if isempty(sourceBandAndChannel) && isempty(meshPathBandAndChannel)
                    % Source & mesh path node band and channels are not given by the user

                    % Try to find a mesh device ID on the source node
                    sourceDeviceID = find([obj.MAC(:).IsMeshDevice]);
                    coder.internal.errorIf(numel(sourceDeviceID) ~= 1, 'wlan:wlanNode:NeedBandAndChannelParameter', 'SourceBandAndChannel', 'source');
                    % Try to find a mesh device ID on the mesh path node
                    meshPathDevID = find([meshPathNode.MAC(:).IsMeshDevice]);
                    coder.internal.errorIf(numel(meshPathDevID) ~= 1, 'wlan:wlanNode:NeedBandAndChannelParameter', 'MeshPathBandAndChannel', 'mesh path');

                elseif isempty(meshPathBandAndChannel)
                    % Mesh path node band and channel is not given by the user, source band and
                    % channel is given by the user

                    % Find source device ID
                    sourceDeviceID = wlanNode.getDeviceID(obj, sourceBandAndChannel);
                    coder.internal.errorIf(isempty(sourceDeviceID), 'wlan:wlanNode:InvalidSourceBandAndChannel', 'source');
                    % Try to find a mesh device ID on the mesh path node
                    meshPathDevID = find([meshPathNode.MAC(:).IsMeshDevice]);
                    coder.internal.errorIf(numel(meshPathDevID) ~= 1, 'wlan:wlanNode:NeedBandAndChannelParameter', 'MeshPathBandAndChannel', 'mesh path');

                elseif isempty(sourceBandAndChannel)
                    % Source band and channel is not given by the user, mesh path node band and
                    % channel is given by the user

                    % Find mesh path device ID
                    meshPathDevID = wlanNode.getDeviceID(meshPathNode, meshPathBandAndChannel);
                    coder.internal.errorIf(isempty(meshPathDevID), 'wlan:wlanNode:InvalidMeshPathBandAndChannel', 'mesh path');
                    % Try to find a mesh device ID on the source node
                    sourceDeviceID = find([obj.MAC(:).IsMeshDevice]);
                    coder.internal.errorIf(numel(sourceDeviceID) ~= 1, 'wlan:wlanNode:NeedBandAndChannelParameter', 'SourceBandAndChannel', 'source');

                else % Source & mesh path nodes band and channel are given by the user
                    % Find source device ID
                    sourceDeviceID = wlanNode.getDeviceID(obj, sourceBandAndChannel);
                    coder.internal.errorIf(isempty(sourceDeviceID), 'wlan:wlanNode:InvalidSourceBandAndChannel', 'source');
                    % Find mesh path device ID
                    meshPathDevID = wlanNode.getDeviceID(obj, meshPathBandAndChannel);
                    coder.internal.errorIf(isempty(meshPathDevID), 'wlan:wlanNode:InvalidMeshPathBandAndChannel', 'mesh path');
                end

                if isempty(destBandAndChannel)
                    % Destination band and channel is not given by the user
                    coder.internal.errorIf(numel(destinationNode.DeviceConfig) > 1, 'wlan:wlanNode:NeedBandAndChannelParameter', 'DestinationBandAndChannel', 'destination');
                    destDevID = 1;
                else % Destination band and channel is given by the user
                    % Find destination device ID
                    destDevID = wlanNode.getDeviceID(destinationNode, destBandAndChannel);
                    coder.internal.errorIf(isempty(destDevID), 'wlan:wlanNode:InvalidDestinationBandAndChannel');
                end
            end
        end

        function setBiDirectionalPaths(obj, destinationNode, meshPathNode, sourceDeviceID, destDevID, meshPathDevID)
        % Auto-find neighbor nodes and set bi-directional paths if a
        % path is not already set

            meshPathAddress = wlan.internal.nodeID2MACAddress([meshPathNode.ID meshPathDevID]);

            % Add backward path for direct neighbors and one-hop
            % neighbors
            if (destinationNode.ID == meshPathNode.ID)
                % Add neighbor nodes
                if ~any(destinationNode.ID == obj.MeshNeighbors)
                    obj.MeshNeighbors(end+1) = destinationNode.ID;
                end
                if ~any(obj.ID == destinationNode.MeshNeighbors)
                    destinationNode.MeshNeighbors(end+1) = obj.ID;
                end
                % Backward path for neighbor node
                if ~any([destinationNode.MeshBridge.ForwardTable{:, 1}] == obj.ID)
                    addPath(destinationNode.MeshBridge, obj.ID, obj.MAC(sourceDeviceID).MACAddress, obj.MAC(sourceDeviceID).MACAddress, destDevID);
                    addPeerMeshSTAInfo(destinationNode, obj, destDevID, sourceDeviceID);
                end
            else % Non-neighbors

                % The source node and next hop node (mesh path node)
                % are implicitly neighbors

                % Add neighbor nodes
                if ~any(meshPathNode.ID == obj.MeshNeighbors)
                    obj.MeshNeighbors(end+1) = meshPathNode.ID;
                end
                if ~any(obj.ID == meshPathNode.MeshNeighbors)
                    meshPathNode.MeshNeighbors(end+1) = obj.ID;
                end
                % Add path from source node to next hop node
                if ~any([obj.MeshBridge.ForwardTable{:, 1}] == meshPathNode.ID)
                    addPath(obj.MeshBridge, meshPathNode.ID, meshPathAddress, meshPathAddress, sourceDeviceID);
                end
                % Add path from next hop node to source node
                if ~any([meshPathNode.MeshBridge.ForwardTable{:, 1}] == obj.ID)
                    addPath(meshPathNode.MeshBridge, obj.ID, obj.MAC(sourceDeviceID).MACAddress, obj.MAC(sourceDeviceID).MACAddress, meshPathDevID);
                    addPeerMeshSTAInfo(meshPathNode, obj, meshPathDevID, sourceDeviceID);
                end

                if ~any([destinationNode.MeshBridge.ForwardTable{:, 1}] == obj.ID)
                    % Check if destination and mesh path nodes are
                    % neighbors. Since mesh path node (i.e. next hop node)
                    % is implicitly a neighbor for the source node,
                    % destination is within 2 hops.
                    twoHopNeighbor = any(destinationNode.ID == meshPathNode.MeshNeighbors);

                    % If destination is a two hop neighbor, add backward
                    % path if there is no entry already
                    if twoHopNeighbor
                        addPath(destinationNode.MeshBridge, obj.ID, obj.MAC(sourceDeviceID).MACAddress, meshPathAddress, destDevID);
                        addPeerMeshSTAInfo(destinationNode, meshPathNode, destDevID, meshPathDevID);
						% Set rate control context
                        basicRates = [6 12 24]; % No configuration option for mesh yet
                        setRateControlContext(destinationNode, meshPathNode, destDevID, meshPathDevID, basicRates);
                    end
                end
            end
        end


        function setRateControlContext(obj, rxNode, txDevID, rxDevID, basicRates)
            % Add operational device configuration
            setAssociationConfig(obj.MAC(txDevID).RateControl, basicRates);
            setAssociationConfig(obj.MAC(txDevID).ULRateControl, basicRates);
            setAssociationConfig(rxNode.MAC(rxDevID).RateControl, basicRates);
            setAssociationConfig(rxNode.MAC(rxDevID).ULRateControl, basicRates);

            % Set receiver context and add mutually supported capabilities
            apCapabilities = struct('MaxMCS',13,'MaxNumSpaceTimeStreams',8);
            staCapabilities = struct('MaxMCS',13,'MaxNumSpaceTimeStreams',8);
            capabilities = struct('MaxMCS',min(apCapabilities.MaxMCS,staCapabilities.MaxMCS), ...
                'MaxNumSpaceTimeStreams',min(apCapabilities.MaxNumSpaceTimeStreams,staCapabilities.MaxNumSpaceTimeStreams));
            setReceiverContext(obj.MAC(txDevID).RateControl, rxNode.ID, capabilities);
            setReceiverContext(obj.MAC(txDevID).ULRateControl, rxNode.ID, capabilities);
            setReceiverContext(rxNode.MAC(rxDevID).RateControl, obj.ID, capabilities);
            setReceiverContext(rxNode.MAC(rxDevID).ULRateControl, obj.ID, capabilities);
        end

        function devCfg = getDeviceConfig(obj, devIdx)
            if obj.IsMLDNode
                devCfg = obj.DeviceConfig.LinkConfig(devIdx);
            else
                devCfg = obj.DeviceConfig(devIdx);
            end
        end

        function checkEventListeners(obj)
        %checkEventListeners Checks whether events have listeners and returns a
        %structure with event names as field names holding flags indicating true if
        %it has a listener.

            hasListenerEvtStruct = wlan.internal.sls.defaultEventList;
    
            if event.hasListener(obj,'MPDUGenerated')
                hasListenerEvtStruct.MPDUGenerated = true;
            end
            if event.hasListener(obj,'MPDUDecoded')
                hasListenerEvtStruct.MPDUDecoded = true;
            end
            if event.hasListener(obj,'TransmissionStatus')
                hasListenerEvtStruct.TransmissionStatus = true;
            end
            if event.hasListener(obj,'StateChanged')
                hasListenerEvtStruct.StateChanged = true;
            end
            if event.hasListener(obj,'AppDataReceived')
                hasListenerEvtStruct.AppDataReceived = true;
            end
            for idx = 1:numel(obj.MAC)
                obj.MAC(idx).HasListener = hasListenerEvtStruct;
                obj.PHYTx(idx).HasListener = hasListenerEvtStruct;
                obj.PHYRx(idx).HasListener = hasListenerEvtStruct;
            end
        end

        function out = validateTrafficParams(obj, trafficSource, nvPair)
        %validateTrafficParams validate the NV pairs for addTrafficSource method

            % Validate data source object
            coder.internal.errorIf(~isa(trafficSource, 'wirelessnetwork.internal.networkTraffic') || ~isscalar(trafficSource), 'wirelessnetwork:networkTraffic:InvalidTrafficSource');
            % Validate MSDU Size
            if isa(trafficSource, 'networkTrafficOnOff')
                validateattributes(trafficSource.PacketSize, {'numeric'}, {'scalar', 'positive', '<=', 2304}, mfilename, 'PacketSize in the traffic source');
            end

            coder.internal.errorIf(mod(numel(nvPair),2) == 1, 'wlan:ConfigBase:InvalidPVPairs');
            out = struct(DestinationNodeID=obj.BroadcastID, DestinationNodeName="", AccessCategory=0);

            % NV pairs
            params = struct(DestinationNode=[], AccessCategory=0);
            for idx = 1:2:numel(nvPair)
                paramName = validatestring(nvPair{idx}, ["DestinationNode" "AccessCategory"], mfilename, 'parameter name');
                switch paramName
                    case 'DestinationNode'
                        paramValue = nvPair{idx+1};
                        validateattributes(paramValue, {'wlanNode'}, {'scalar'}, mfilename, paramName);
                    otherwise % 'AccessCategory'
                        paramValue = nvPair{idx+1};
                        validateattributes(paramValue, {'numeric'}, {'integer', 'scalar', '>=', 0, '<=', 3}, mfilename, paramName);
                end
                params.(paramName) = paramValue;
            end

            out.AccessCategory = params.AccessCategory;
            if ~isempty(params.DestinationNode)
                out = repmat(out, 1, numel(params.DestinationNode));
                for nodeIdx = 1:numel(params.DestinationNode)
                    % Error when source is AP and destination is not an
                    % associated station. Allow when the node has a mesh
                    % device (AP+Mesh node).
                    coder.internal.errorIf((obj.IsAPNode && ~obj.IsMeshNode) && (isempty(obj.AssociationInfo) || ~any(params.DestinationNode(nodeIdx).ID == [obj.AssociationInfo(:).NodeID])), 'wlan:wlanNode:APDestinationUnassociated');
                    % Error when source is STA and
                    %   * Unassociated
                    %   * Destination is not in the same BSS
                    coder.internal.errorIf(~obj.IsAPNode && ~obj.IsMeshNode && ...
                        (~obj.MAC(1).IsAssociatedSTA || ~any(arrayfun(@(x) strcmp(x.BSSID, obj.MAC(1).BSSID), params.DestinationNode(nodeIdx).MAC))), 'wlan:wlanNode:STADestinationUnassociated');
                    % Error for duplicate traffic source
                    coder.internal.errorIf(any((params.DestinationNode(nodeIdx).ID == [obj.Application.PacketInfo(:).DestinationNodeID]) & ...
                        (params.AccessCategory == [obj.Application.PacketInfo(:).AccessCategory])), 'wlan:wlanNode:DuplicateTrafficSource', params.DestinationNode(nodeIdx).Name, params.AccessCategory);

                    out(nodeIdx).DestinationNodeID = params.DestinationNode(nodeIdx).ID;
                    out(nodeIdx).DestinationNodeName = params.DestinationNode(nodeIdx).Name;
                end
                
            else % No value provided for DestinationNode argument              
                devCfg = obj.DeviceConfig;
                devType = "device";

                if obj.IsMLDNode
                    if obj.IsAPNode % AP MLD
                        % Check whether any of the associated STAs is an EMLSR STA
                        coder.internal.errorIf(any([obj.AssociationInfo(:).EnhancedMLMode]), ...
                            'wlan:wlanNode:UnsupportedBroadcastAPMLD', obj.Name);
                    else % STA MLD
                        coder.internal.errorIf(strcmp(obj.DeviceConfig.EnhancedMultilinkMode, "EMLSR"), ... % EMLSR STA
                            'wlan:wlanNode:UnsupportedBroadcastEMLSRSTA', obj.Name);
                    end

                    devCfg = devCfg.LinkConfig;
                    devType = "link";
                end

                % Broadcast traffic is not supported with OFDMA transmissions
                for idx=1:numel(devCfg)
                    coder.internal.errorIf(strcmp(devCfg(idx).TransmissionFormat, "HE-MU-OFDMA"), 'wlan:wlanNode:BroadcastUnsupportedWithOFDMA', devType, idx, obj.Name);
                end
            end
        end

        function [meshPathNode, params] = validateMeshPathParams(obj, nInputs, destinationNode, varargin)
        %validateMeshPathParams Validate the inputs for addMeshPath method

            coder.internal.errorIf(~obj.IsMeshNode, 'wlan:wlanNode:MustBeMesh', 'first');
            validateattributes(destinationNode, {'wlanNode'}, {'scalar'}, '', 'destination node');
            if (mod(nInputs, 2) == 0) % mesh path node not given as input
                coder.internal.errorIf(~destinationNode.IsMeshNode, 'wlan:wlanNode:NeedProxyNode');
                meshPathNode = destinationNode;
                nvPair = varargin;
            else % mesh path node given as input
                meshPathNode = varargin{1};
                validateattributes(meshPathNode, {'wlanNode'}, {'scalar'}, '', 'mesh path node');
                coder.internal.errorIf(~meshPathNode.IsMeshNode, 'wlan:wlanNode:MustBeMesh', 'third');
                nvPair = varargin(2:end);
            end

            % NV pairs
            params = struct(SourceBandAndChannel=[], MeshPathBandAndChannel=[], DestinationBandAndChannel=[]);
            for idx = 1:2:numel(nvPair)
                paramName = validatestring(nvPair{idx}, ["SourceBandAndChannel" "MeshPathBandAndChannel" "DestinationBandAndChannel"], mfilename, 'parameter name');
                validateattributes(nvPair{idx+1}, {'numeric'}, {'row','vector','numel',2}, mfilename, paramName);
                wlanDeviceConfig.validateBandAndChannel(nvPair{idx+1}, paramName);
                params.(paramName) = nvPair{idx+1};
            end
        end

        function params = validateAssociationParams(obj, associatedSTAs, nvPair)
        %validateAssociationParams validate inputs to associationStations method

            % Validate inputs
            validateattributes(obj, {'wlanNode'}, {'scalar'}, mfilename, 'AP node object');
            coder.internal.errorIf(~obj.IsAPNode, 'wlan:wlanNode:MustBeAP');
            validateattributes(associatedSTAs, {'wlanNode'}, {'vector'}, mfilename, '', 2);
            coder.internal.errorIf(any(arrayfun(@(x) x.IsAPNode || x.IsMeshNode, associatedSTAs, UniformOutput=true)), 'wlan:wlanNode:NonSTAInSTAList');
            if ~obj.IsMLDNode % Non-MLD
                coder.internal.errorIf(any(arrayfun(@(x) x.IsMLDNode, associatedSTAs, UniformOutput=true)), 'wlan:wlanNode:InvalidSTADeviceType', obj.Name);
            else % MLD
                numLinks = numel(obj.DeviceConfig.LinkConfig);
                mldSTAs = associatedSTAs([associatedSTAs(:).IsMLDNode]);
                coder.internal.errorIf(any(arrayfun(@(x) numel(x.DeviceConfig.LinkConfig), mldSTAs, UniformOutput=true) ~= numLinks), 'wlan:wlanNode:UnequalNumLinks');
            end

            % Validate NV pairs
            coder.internal.errorIf(mod(numel(nvPair),2) == 1, 'wlan:ConfigBase:InvalidPVPairs');
            params = struct(BandAndChannel=[], FullBufferTraffic="off");
            for idx = 1:2:numel(nvPair)
                paramName = validatestring(nvPair{idx}, ["BandAndChannel" "FullBufferTraffic"], mfilename, 'parameter name');
                switch paramName
                    case 'BandAndChannel'
                        paramValue = nvPair{idx+1};
                        validateattributes(paramValue, {'numeric'}, {'nonempty', 'ncols',2}, mfilename, paramName);
                        numRows = size(paramValue, 1);
                        for rowIdx = 1:numRows
                            wlanDeviceConfig.validateBandAndChannel(paramValue(rowIdx, :), paramName);
                        end
                    otherwise % 'FullBufferTraffic'
                        paramValue = validatestring(nvPair{idx+1}, ["on", "off", "DL", "UL"], mfilename, paramName);
                end
                params.(paramName) = paramValue;
            end
        end
    end

    methods(Static, Hidden)
        function deviceID = getDeviceID(node, bandAndChannel)
            frequency = wlanChannelFrequency(bandAndChannel(2), bandAndChannel(1));
            deviceID = find(node.ReceiveFrequency == frequency, 1);
        end

        function macStats = getMLDMACStats(linkStats)
            % Return the summary of metrics for all links within MAC layer

            macStats = struct;

            statNames = fieldnames(linkStats(1));
            for statIdx = 1:numel(statNames)
                if ~strcmp(statNames{statIdx}, "AccessCategories")
                    totalStatValue = 0;
                    for linkIdx = 1:numel(linkStats)
                        totalStatValue = totalStatValue + linkStats(linkIdx).(statNames{statIdx});
                    end
                    macStats.(statNames{statIdx}) = totalStatValue;
                else
                    perACStatNames = fieldnames(linkStats(1).(statNames{statIdx}));
                    for acIdx = 1:4
                        for perACStatIdx = 1:numel(perACStatNames)
                            totalStatValue = 0;
                            for linkIdx = 1:numel(linkStats)
                                totalStatValue = totalStatValue + ...
                                    linkStats(linkIdx).(statNames{statIdx})(acIdx).(perACStatNames{perACStatIdx});
                            end
                            macStats.(statNames{statIdx})(acIdx).(perACStatNames{perACStatIdx}) = totalStatValue;
                        end
                    end
                end
            end
        end

        function phyStats = getMLDPHYStats(linkStats)
            % Return the summary of metrics for all links within PHY layer

            phyStats = struct;

            statNames = fieldnames(linkStats(1));
            for statIdx = 1:numel(statNames)
                totalStatValue = 0;
                for linkIdx = 1:numel(linkStats)
                    totalStatValue = totalStatValue + linkStats(linkIdx).(statNames{statIdx});
                end
                phyStats.(statNames{statIdx}) = totalStatValue;
            end
        end

        function [primaryChannelNum,primary20Freq] = getPrimaryChannel(bandAndChannel, bandwidth, primaryChannelIdx)
            % Return primary channel number and center frequency

            operatingFreq = wlanChannelFrequency(bandAndChannel(2), bandAndChannel(1));
            startingFreq = operatingFreq - bandwidth/2;
            primary20Freq = startingFreq + (primaryChannelIdx-1)*20e6 + 10e6;

            % Get channel number from center frequency
            switch bandAndChannel(1)
                case 2.4
                    primaryChannelNum = (primary20Freq-2407e6)/5e6;
                case 5
                    primaryChannelNum = (primary20Freq-5e9)/5e6;
                case 6
                    primaryChannelNum = (primary20Freq-5950e6)/5e6;
            end
        end
    end

    methods(Access=private)
         function validateMultipleOperatingFreq(obj)
             % Validate that the operating frequencies of devices/links must be
             % different

             % Validate multi-band combination
             if ~obj(1).IsMLDNode && (numel(obj(1).DeviceConfig) > 1)
                 % Multiple devices are not supported for a STA node
                 coder.internal.errorIf(ismember("STA", [obj(1).DeviceConfig(:).Mode]), 'wlan:wlanNode:UnsupportedMultiDeviceCombo');
                 % Two devices in the same node cannot operate in the same frequency
                 for idx = 1:numel(obj(1).DeviceConfig)-1
                     bandAndChannels = [obj(1).DeviceConfig(idx+1:end).BandAndChannel];
                     coder.internal.errorIf(any((obj(1).DeviceConfig(idx).BandAndChannel(1) == bandAndChannels(1:2:end-1)) & (obj(1).DeviceConfig(idx).BandAndChannel(2) == bandAndChannels(2:2:end))), 'wlan:wlanNode:UnsupportedMultiDeviceFreq');
                 end
             end

             % Validate multilink combination
             if obj(1).IsMLDNode && (numel(obj(1).DeviceConfig.LinkConfig) > 1)
                 linkCfg = obj(1).DeviceConfig.LinkConfig;
                 % Two links in the same MLD cannot operate in the same frequency
                 for idx = 1:numel(linkCfg)-1
                     bandAndChannels = [linkCfg(idx+1:end).BandAndChannel];
                     coder.internal.errorIf(any((linkCfg(idx).BandAndChannel(1) == bandAndChannels(1:2:end-1)) & (linkCfg(idx).BandAndChannel(2) == bandAndChannels(2:2:end))), 'wlan:wlanNode:UnsupportedMultiLinkFreq');
                 end
             end
         end

         function validateMACAndPHYAbstraction(obj, macAndPHYCfg)
             % Validate MAC abstraction w.r.t the link/device configuration

             % MAC frame abstraction is allowed only when PHY is abstracted
             coder.internal.errorIf((strcmp(obj.PHYAbstractionMethod, 'none') && obj.MACFrameAbstraction), 'wlan:wlanNode:InvalidMACandPHYCombination');

             % Full MAC frame generation and decoding is not supported for multiuser
             % transmission
             coder.internal.errorIf(~obj.MACFrameAbstraction && any(strcmp([macAndPHYCfg(:).TransmissionFormat], "HE-MU-OFDMA")), 'wlan:wlanNode:UnsupportedMACFrameAbstractionForOFDMA');

             % ACI modeling is only supported for full-PHY
             coder.internal.errorIf((~any(strcmpi([macAndPHYCfg(:).InterferenceModeling], 'co-channel')) && ~strcmp(obj.PHYAbstractionMethod, 'none')), 'wlan:wlanNode:InterferenceModelingMustBeFullPHY');

             % Full PHY simulation is not supported if the STA is operating
             % in EMLSR mode.
             coder.internal.errorIf(strcmp(obj.PHYAbstractionMethod, 'none') && obj.IsMLDNode && ~obj.IsAPNode && strcmp(obj.DeviceConfig.EnhancedMultilinkMode, "EMLSR"), ...
                 'wlan:wlanNode:UnsupportedFullPHYEMLSR');
         end

         function addPeerMeshSTAInfo(obj, peerNode, selfDeviceID, peerDeviceID)
             % Add peer mesh STA info. Information includes peer mesh node
             % ID, peer mesh node MAC address, device ID on which this node
             % is connected to its peer mesh node and the bandwidth used
             % for communication with peer node.

             peerNodeID = peerNode.ID;
             peerNodeAddress = peerNode.MAC(peerDeviceID).MACAddress;
             % Add the information of peer mesh STA if it is not already present
             if isempty(obj.AssociationInfo) || ~any(peerNodeID == [obj.AssociationInfo(:).NodeID])
                 associationInfo = obj.AssociationInfoTemplate;
                 associationInfo.NodeID = peerNodeID;
                 associationInfo.MACAddress = peerNodeAddress;
                 associationInfo.DeviceID = selfDeviceID;
                 bwToUseInHz = min(obj.DeviceConfig(selfDeviceID).ChannelBandwidth, ...
                     peerNode.DeviceConfig(peerDeviceID).ChannelBandwidth);
                 associationInfo.Bandwidth = bwToUseInHz/1e6; % In MHz
                 associationInfo.MaxSupportedStandard = peerNode.MaxSupportedStandard;
                 obj.AssociationInfo = [obj.AssociationInfo associationInfo];
                 addAssociationInfo(obj.SharedMAC(selfDeviceID), associationInfo);

                 % Add primary channel information at MAC and phy modules
                 primaryChannelIdx = 1; % Not configurable for mesh. Hence consider default
                 devCfg = getDeviceConfig(obj, selfDeviceID);
                 [~,primaryChannelFrequency] = wlanNode.getPrimaryChannel(devCfg.BandAndChannel, devCfg.ChannelBandwidth, primaryChannelIdx);
                 wlan.internal.setPrimaryChannelInfoAtLayers(obj, selfDeviceID, primaryChannelIdx, primaryChannelFrequency);
                 % As peer node is also of same BW and center frequency, configuring same
                 % primary index as this node
                 wlan.internal.setPrimaryChannelInfoAtLayers(peerNode, peerDeviceID, primaryChannelIdx, primaryChannelFrequency);

             elseif any(peerNodeID == [obj.AssociationInfo(:).NodeID])
                 % Update the information if it is already present
                 peerNodeIdxLogical = (peerNodeID == [obj.AssociationInfo(:).NodeID]);
                 associationInfo = obj.AssociationInfo(peerNodeIdxLogical);
                 if ~any(selfDeviceID == associationInfo.DeviceID)
                     idx = numel(associationInfo.DeviceID)+1;
                     associationInfo.MACAddress(idx, :) = peerNodeAddress;
                     associationInfo.DeviceID(idx) = selfDeviceID;
                     bwToUseInHz = min(obj.DeviceConfig(selfDeviceID).ChannelBandwidth, ...
                         peerNode.DeviceConfig(peerDeviceID).ChannelBandwidth);
                     associationInfo.Bandwidth = bwToUseInHz/1e6; % In MHz
                     obj.AssociationInfo(peerNodeIdxLogical) = associationInfo;
                     addAssociationInfo(obj.SharedMAC(selfDeviceID), associationInfo);

                     % Add primary channel information at MAC and phy modules
                     primaryChannelIdx = 1; % Not configurable for mesh. Hence consider default
                     devCfg = getDeviceConfig(obj, selfDeviceID);
                     [~,primaryChannelFrequency] = wlanNode.getPrimaryChannel(devCfg.BandAndChannel, devCfg.ChannelBandwidth, primaryChannelIdx);
                     wlan.internal.setPrimaryChannelInfoAtLayers(obj, selfDeviceID, primaryChannelIdx, primaryChannelFrequency);
                     % As peer node is also of same BW and center frequency, configuring same
                     % primary index as this node
                     wlan.internal.setPrimaryChannelInfoAtLayers(peerNode, peerDeviceID, primaryChannelIdx, primaryChannelFrequency);
                 end
             end
         end

         function validateKPINodes(srcNode, destNode, bandAndChannel)
             %validateKPINodes Validates inputs to kpi method

             % Check if atleast one of the inputs is scalar
             coder.internal.errorIf(~isscalar(srcNode) && ~isscalar(destNode),...
                 'wlan:wlanNode:KPIInvalidSignature');

             % Store information about mode: AP or STA or MESH
             srcMode = [];
             destMode = [];

             for i = 1:numel(srcNode)
                 srcMode = [srcMode operatingMode(srcNode(i))];
             end

             for i = 1:numel(destNode)
                 destMode = [destMode operatingMode(destNode(i))];
             end

             % Check if any of the source or destination nodes is a mesh node
             coder.internal.errorIf(any(strcmp(srcMode, 'mesh')) || any(strcmp(destMode, 'mesh')),...
                 'wlan:wlanNode:MustBeAPOrSTA');

             % Check if all nodes in vector are operating in same mode
             coder.internal.errorIf(~all(strcmp(srcMode(1), srcMode)),...
                 'wlan:wlanNode:KPIMustHaveSameMode', 'source nodes');

             coder.internal.errorIf(~all(strcmp(destMode(1), destMode)),...
                 'wlan:wlanNode:KPIMustHaveSameMode', 'destination nodes');

             % Check if one STA is mapped to multiple AP
             coder.internal.errorIf((~isscalar(srcNode) &&  isequal(destMode(1),'STA')) || ...
                 (~isscalar(destNode) && isequal(srcMode(1),'STA')),...
                 'wlan:wlanNode:UnsupportedInputCombination');

             % Check if source and destination nodes are a valid pair
             % While the kpi function doesnt allow STA/STA pair, internal support for
             % calculation of latency between two STA nodes is present
             modePair = strcat(srcMode(1), "/", destMode(1));
             validModePairs = ["AP/STA", "STA/AP"];
             coder.internal.errorIf(~any(strcmp(modePair,validModePairs)),...
                 'wlan:wlanNode:KPIInvalidModeCombination', modePair, validModePairs(1), validModePairs(2));

             if ~isempty(bandAndChannel)
                 % Validate the bandAndChannel values
                 validateattributes(bandAndChannel, {'numeric'}, {'ncols', 2}, mfilename, 'BandAndChannel');
                 wlanDeviceConfig.validateBandAndChannel(bandAndChannel, 'BandAndChannel');

                 % Check if there is atleast one source node operating on specified
                 % bandAndChannel
                 srcValid = false;
                 for i = 1:numel(srcNode)
                     if any(all([srcNode(i).SharedMAC.BandAndChannel] == bandAndChannel, 2))
                         srcValid = true;
                     end
                 end

                 % Check if there is atleast one destination node operating on specified
                 % bandAndChannel
                 destValid = false;
                 for i = 1:numel(destNode)
                     if any(all([destNode(i).SharedMAC.BandAndChannel] == bandAndChannel, 2))
                         destValid = true;
                     end
                 end

                 coder.internal.errorIf(~(srcValid && destValid),...
                     'wlan:wlanNode:KPINoCommonBandAndChannel', string(bandAndChannel(1)), bandAndChannel(2));
             end
         end

         function mode = operatingMode(node)
             %operatingMode Returns the operation mode of the node

             if node.IsAPNode
                 mode = "AP";
             elseif node.IsMeshNode
                 mode = "mesh";
             else
                 mode = "STA";
             end
         end

         function throughput = calculateThroughput(srcNode, destNode)
             %calculateThroughput Calculate the throughput between source and
             %destination nodes

             throughput = zeros(1, max(numel(srcNode), numel(destNode)));
             for srcNodeIdx = 1:numel(srcNode)
                 % Store the perSTAStats of each device of source node
                 perSTAStatsCell = {};
                 % Store the ID of nodes associated with the device
                 srcAssociationInfo = {};
                 for macIdx = 1:numel(srcNode(srcNodeIdx).MAC)
                     macObj = srcNode(srcNodeIdx).MAC(macIdx);
                     perSTAStats = getPerSTAStatistics(macObj);
                     if ~isempty(perSTAStats)
                         perSTAStatsCell = [perSTAStatsCell perSTAStats];
                         srcAssociationInfo = [srcAssociationInfo [perSTAStats.AssociatedNodeID]];
                     end
                 end

                 for destNodeIdx = 1:numel(destNode)
                     % Intialize as 0 for a specific source-destination pair
                     transmittedPayloadBytes = 0;
                     for idx = 1: numel(perSTAStatsCell)
                         % Find the index at which the values associated to the destination node are
                         % stored in the perSTAStatsCell
                         idxLogical = (srcAssociationInfo{idx} == destNode(destNodeIdx).ID);
                         if any(idxLogical) % Increment only if nodes are associated
                             perSTAStats = perSTAStatsCell{idx};
                             % Add the values to transmitted payload bytes
                             transmittedPayloadBytes = transmittedPayloadBytes + perSTAStats(idxLogical).TransmittedPayloadBytes;
                         end
                     end
                     simulationTime = srcNode(srcNodeIdx).CurrentTime; %Time in seconds
                     % Calculate the throughput
                     tputIdx = max(srcNodeIdx, destNodeIdx);
                     throughput(tputIdx) = (transmittedPayloadBytes*8*1e-6)/simulationTime;
                 end
             end
         end

         function throughput = calculateLinkThroughput(srcNode, destNode, bandAndChannel)
             %calculateLinkThroughput Compute the throughput of the links specified by
             %bandAndChannel between the source and destination nodes.

             throughput = zeros(1, max(numel(srcNode), numel(destNode)));
             for srcNodeIdx = 1:numel(srcNode)
                 % Find the index of the specified link
                 macIdx = all([srcNode(srcNodeIdx).SharedMAC.BandAndChannel] == bandAndChannel, 2);
                 macObj = srcNode(srcNodeIdx).MAC(macIdx);

                 % Check that the source node has atleast one device operating on the
                 % specified band and channel
                 if ~isempty(macObj)
                     perSTAStats = getPerSTAStatistics(macObj);

                     % perSTAStats will be an empty structure if the device; macObj has no
                     % associations
                     if ~isempty(perSTAStats)
                         associatedNodeID = [perSTAStats.AssociatedNodeID];

                         for destNodeIdx = 1:numel(destNode)
                             % Intialize as 0 for a specific source-destination pair
                             transmittedPayloadBytes = 0;
                             if any(associatedNodeID == destNode(destNodeIdx).ID) % Increment only if nodes are associated
                                 % Add the values to transmitted payload bytes
                                 transmittedPayloadBytes = transmittedPayloadBytes + ...
                                     perSTAStats(associatedNodeID == destNode(destNodeIdx).ID).TransmittedPayloadBytes;
                             end
                             simulationTime = srcNode(srcNodeIdx).CurrentTime; %Time in seconds
                             tputIdx = max(srcNodeIdx, destNodeIdx);
                             throughput(tputIdx) = (transmittedPayloadBytes*8*1e-6)/simulationTime;
                         end
                     end
                 end
             end
         end

         function plr = calculatePLR(srcNode, destNode)
             %calculatePLR Calculate the packet loss ratio between source and
             %destination nodes

             plr = zeros(1, max(numel(srcNode), numel(destNode)));
             for srcNodeIdx = 1:numel(srcNode)
                 % Store the perSTAStats of each device/link of source node
                 perSTAStatsCell = {};
                 % Store the ID of nodes associated with the device/link
                 srcAssociationInfo = {};
                 for macIdx = 1:numel(srcNode(srcNodeIdx).MAC)
                     macObj = srcNode(srcNodeIdx).MAC(macIdx);
                     perSTAStats = getPerSTAStatistics(macObj);
                     if ~isempty(perSTAStats)
                         perSTAStatsCell = [perSTAStatsCell perSTAStats];
                         srcAssociationInfo = [srcAssociationInfo [perSTAStats.AssociatedNodeID]];
                     end
                 end

                 for destNodeIdx = 1:numel(destNode)
                     % Intialize as 0 for a specific source-destination pair
                     transmittedDataFrames = 0;
                     retransmittedDataFrames = 0;
                     % Iterate through all the perSTAStats
                     for idx = 1: numel(perSTAStatsCell)
                         % Find the index at which the values associated to the destination node are
                         % stored in the perSTAStats
                         idxLogical = (srcAssociationInfo{idx} == destNode(destNodeIdx).ID);
                         if any(idxLogical) % Increment only if nodes are associated
                             perSTAStats = perSTAStatsCell{idx};
                             transmittedDataFrames = transmittedDataFrames + sum([perSTAStats(idxLogical).TransmittedDataFrames]);
                             retransmittedDataFrames = retransmittedDataFrames + sum([perSTAStats(idxLogical).RetransmittedDataFrames]);
                         end
                     end
                     plrIdx = max(srcNodeIdx, destNodeIdx);
                     if transmittedDataFrames > 0
                         plr(plrIdx) = retransmittedDataFrames/transmittedDataFrames;
                     end
                 end
             end
         end

         function plr = calculateLinkPLR(srcNode, destNode, bandAndChannel)
             %calculateLinkPLR Compute the packet loss ratio of the links specified by
             %bandAndChannel between the source and destination nodes.

             plr = zeros(1, max(numel(srcNode), numel(destNode)));
             for srcNodeIdx = 1:numel(srcNode)
                 % Find the index of the specified link
                 macIdx = all([srcNode(srcNodeIdx).SharedMAC.BandAndChannel] == bandAndChannel, 2);
                 macObj = srcNode(srcNodeIdx).MAC(macIdx);

                 % Check that the source node has atleast one device operating on the
                 % specified band and channel
                 if ~isempty(macObj)
                     perSTAStats = getPerSTAStatistics(macObj);

                     % perSTAStats will be an empty structure if the device; macObj has no
                     % associations
                     if ~isempty(perSTAStats)
                         associatedNodeID = perSTAStats.AssociatedNodeID;

                         for destNodeIdx = 1:numel(destNode)
                             % Intialize as 0 for a specific source-destination pair
                             transmittedDataFrames = 0;
                             retransmittedDataFrames = 0;
                             if any(associatedNodeID == destNode(destNodeIdx).ID) % Increment only if nodes are associated
                                 transmittedDataFrames = transmittedDataFrames + ...
                                     perSTAStats(associatedNodeID == destNode(destNodeIdx).ID).TransmittedDataFrames;
                                 retransmittedDataFrames = retransmittedDataFrames + ...
                                     perSTAStats(associatedNodeID == destNode(destNodeIdx).ID).RetransmittedDataFrames;
                             end
                             plrIdx = max(srcNodeIdx, destNodeIdx);
                             if transmittedDataFrames > 0
                                 plr(plrIdx) = retransmittedDataFrames/transmittedDataFrames;
                             end
                         end
                     end
                 end
             end
         end

         function avgLatency = calculateLatency(srcNode, destNode)
             %calculateLatency Calculate the average latency of the packes transmitted
             %to destination nodes from source nodes

             avgLatency = zeros(1, max(numel(srcNode), numel(destNode)));
             for destNodeIdx = 1:numel(destNode)
                 % Store the latency stats of the destination node
                 rxAppLatencyStats = destNode(destNodeIdx).RxAppLatencyStats;

                 % If the destination node has not received any packets, RxAppLatencyStats
                 % is an empty structure
                 if ~isempty(rxAppLatencyStats)
                     for srcNodeIdx = 1:numel(srcNode)
                         % Find the index at which the values associated to the source node are
                         % stored in the rxAppLatencyStats
                         idxLogical = ([rxAppLatencyStats.SourceNodeID] == srcNode(srcNodeIdx).ID);
                         % Calculate average latency only if destination node has received at least
                         % one packet from source node
                         if any(idxLogical) && rxAppLatencyStats(idxLogical).ReceivedPackets > 0
                             latency = rxAppLatencyStats(idxLogical).AggregatePacketLatency/rxAppLatencyStats(idxLogical).ReceivedPackets;
                             latencyIdx = max(srcNodeIdx, destNodeIdx);
                             avgLatency(latencyIdx) = latency;
                             destNode(destNodeIdx).RxAppLatencyStats(idxLogical).AveragePacketLatency = latency;
                         end
                     end
                 end
             end
         end
    end
end