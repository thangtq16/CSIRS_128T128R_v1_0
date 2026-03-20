classdef wirelessNetworkSimulator < handle
    %wirelessNetworkSimulator Implements wireless network simulator
    %
    %   SIMULATOR = wirelessNetworkSimulator.init() returns an object to
    %   simulate a wireless network.
    %   This class implements functionality to
    %       - Simulate a multinode wireless network for a given duration
    %       - Schedule or cancel actions to process during a simulation
    %
    %   wirelessNetworkSimulator properties (read-only):
    %
    %   CurrentTime     - Current simulation time in seconds
    %   ChannelFunction - Channel model for network simulation
    %   EndTime         - Simulation end time in seconds
    %   Nodes           - Cell array of nodes added to the simulator
    %
    %   wirelessNetworkSimulator static methods:
    %
    %   init            - Create or reset the simulator object
    %   getInstance     - Get the simulator object
    %
    %   wirelessNetworkSimulator object methods:
    %
    %   addChannelModel - Add custom channel and path loss model
    %   addNodes        - Add nodes to the simulator
    %   run             - Run the simulation
    %   scheduleAction  - Schedule an action to process at a specified
    %                     simulation time
    %   cancelAction    - Cancel a scheduled action
    %
    %  Note: By default, the simulator supports single-input
    %  single-output (SISO) scenarios. A channel model must be added using <a href="matlab:help('wirelessNetworkSimulator/addChannelModel')">addChannelModel</a>
    %  for simulating non-SISO scenarios

    %   Copyright 2022-2024 The MathWorks, Inc.

    properties (SetAccess = protected)
        %CurrentTime Current simulation time in seconds
        CurrentTime = 0

        %EndTime Simulation end time in seconds
        EndTime = 0

        %ChannelFunction Channel model for network simulation
        % By default, the simulator uses the free space path loss (fspl) model. To
        % specify a custom channel model, call the addChannelModel object
        % function. The default value is "fspl".
        ChannelFunction = "fspl"

        %Nodes Cell array of nodes added to the simulator
        Nodes = {}
    end

    properties (Access = protected)
        %Actions List of actions queued for future processing
        Actions

        %ActionInvokeTimes List of next invoke times in seconds for the actions in
        %Actions property
        ActionInvokeTimes

        %NodeNextInvokeTimes List of next invoke time of the nodes in network
        NodeNextInvokeTimes

        %TimeAdvanceActions List of actions to be performed on every time advance
        TimeAdvanceActions

        %NumNodes Number of nodes in the simulation
        NumNodes = 0

        %ActionCounter Counter for assigning unique identifier for the
        %scheduled action
        ActionCounter = 0

        %NodeIdxList Mapping of node IDs to index in the Nodes array
        NodeIdxList

        %ResetRequired Flag to indicate whether simulator needs reset or
        %not before next run call
        ResetRequired = false

        %DefaultChannelModel Flag to indicate whether simulator is using
        %default channel model or custom channel model
        DefaultChannelModel = true

        %NewActionsAdded Flag to indicate whether new action is added during the
        %simuation
        NewActionsAdded = false
    end

    methods(Static)
        function obj = getInstance()
            %getInstance Get the simulator object
            %
            %   OBJ = wirelessNetworkSimulator.getInstance() returns the simulator
            %   object, OBJ, of type wirelessNetworkSimulator if it exists or else it
            %   throws an error.

            obj = wirelessNetworkSimulator.getState(1);
        end

        function obj = init()
            %init Create or reset the simulator object
            %
            %   OBJ = wirelessNetworkSimulator.init() creates or resets the simulator
            %   object and returns the object, OBJ, of type wirelessNetworkSimulator.
            %   You must call this method before any other simulator method and also
            %   before creating the nodes.

            obj = wirelessNetworkSimulator.getState(0);
        end
    end

    methods
        function addChannelModel(obj, customChannelFcn)
            %addChannelModel Add custom channel and path loss model
            %
            %   addChannelModel(OBJ,CUSTOMCHANNELFCN) adds the function handle,
            %   CUSTOMCHANNELFCN, of a custom channel and path loss model for all the
            %   links in the simulation.
            %
            %   OBJ is an object of type wirelessNetworkSimulator.
            %
            %   CUSTOMCHANNELFCN is a function handle with the signature:
            %     RXPACKET = customChannelFcn(RXINFO,TXPACKET)
            %        RXINFO is a structure containing the fields listed below:
            %           ID       - Receiver node ID
            %           Position - Receiver node position in 3-D Cartesian
            %                      coordinates, representing the [x y z] position in
            %                      meters.
            %           Velocity - Receiver node velocity in 3-D Cartesian
            %                      coordinates, representing the [x y z] position in
            %                      meters per second.
            %           NumReceiveAntennas - Number of antennas at the receiver.
            %        TXPACKET is the packet transmitted by a node and RXPACKET is the
            %        resultant packet after undergoing channel impairments. To model
            %        packet drops at the channel return the RXPACKET as []. TXPACKET
            %        and RXPACKET are structures and must contain the fields listed
            %        below:
            %           Type                - Type of the signal. Accepted values
            %                                 are 0, 1, 2, 3, and 4, which represent
            %                                 invalid packet, WLAN, 5G, Bluetooth LE,
            %                                 and Bluetooth BR/EDR packets,
            %                                 respectively.
            %           TransmitterID       - Transmitter node identifier. It is a
            %                                 positive scalar integer.
            %           TransmitterPosition - Position of transmitter, specified as a
            %                                 real-valued vector in Cartesian
            %                                 coordinates [x y z] in meters.
            %           TransmitterVelocity - Velocity (v) of transmitter in the x-,
            %                                 y-, and z-directions, specified as a
            %                                 real-valued vector of the form [vx vy vz]
            %                                 in meters per second.
            %           NumTransmitAntennas - Number of antennas at the transmitter.
            %           StartTime           - Packet transmission start time at the
            %                                 transmitter or packet arrival time at the
            %                                 receiver in seconds.
            %           Duration            - Duration of the packet in seconds.
            %           Power               - Average power of the packet in dBm.
            %           CenterFrequency     - Center frequency of the carrier in Hz.
            %           Bandwidth           - Carrier bandwidth in Hz. It is the
            %                                 bandwidth around the center frequency.
            %           Abstraction         - A logical scalar representing the
            %                                 abstraction type. It takes a value of
            %                                 true or false which represents abstracted
            %                                 PHY or full PHY, respectively. The
            %                                 default value is false.
            %           SampleRate          - Sample rate of the packet, in samples per
            %                                 second. It is only applicable when
            %                                 Abstraction value is set to false.
            %           DirectToDestination - A numeric integer scalar. A value of 0
            %                                 indicates it is a normal packet and is
            %                                 transmitted over the channel. A nonzero
            %                                 value represents a destination node ID
            %                                 and also indicates that it is a special
            %                                 packet, where the channel model is
            %                                 bypassed and transmitted directly to the
            %                                 destination node.
            %           Data     - Contains time samples (full PHY) or frame
            %                      information (abstracted PHY). If Abstraction is set
            %                      to false, this field contains time-domain samples of
            %                      the packet represented as a T-by-R matrix of complex
            %                      values. T is the number of time-domain samples. R is
            %                      the number of transmitter antennas if the packet
            %                      represents the transmitted packet or number of
            %                      receiver antennas if the packet represents the
            %                      received packet. If Abstraction is set to true, this
            %                      field contains the frame information.
            %           Metadata - A structure representing the technology-specific,
            %           abstraction-specific, and channel information. It contains the
            %           following fields.
            %               Channel - It is a structure representing the
            %               information about the channel. It contains
            %               following fields.
            %                   PathGains - Complex path gains at each snapshot in
            %                   time. It is a matrix of size Ncs-by-Np-by-Nt-by-Nr.
            %                   PathDelays - Delays in seconds corresponding to each
            %                   path. It is a vector of size 1-by-Np.
            %                   PathFilters - Filter coefficients for each path. It is
            %                   a matrix of size Np-by-Nf.
            %                   SampleTimes - Simulation time in seconds corresponding
            %                   to each path gains snapshot. It is a vector of size
            %                   Ncs-by-1.
            %               Here Ncs, Np, Nt, Nr, and Nf represents number of channel
            %               snapshots, number of paths, number of transmit antennas,
            %               number of receive antennas, and number of filter
            %               coefficients respectively.
            %
            %   The CUSTOMCHANNELFCN must update the following fields in the transmitted
            %   packet to include channels effects.
            %       Power - To include large scale effects
            %       Data  - If Abstraction = false, scale the data to include small and
            %               large scale effects.
            %       Metadata.Channel - Include all small scale effects if the value of
            %                          the Abstraction field is true or if the value of
            %                          the Abstraction field is false and the receiver
            %                          uses perfect channel estimation.
            %       Duration - Set the value as actual packet duration + final
            %                  transient (delay spread + filter length - implementation
            %                  delay). Updating this field is optional.

            narginchk(2,2);
            validateattributes(customChannelFcn, {'function_handle'}, {'nonempty'}, mfilename, 'customChannelFcn');
            obj.ChannelFunction = customChannelFcn;
            obj.DefaultChannelModel = false;
        end

        function addNodes(obj, nodes)
            %addNodes Add nodes to the simulator
            %
            %   addNodes(OBJ, NODES) adds the specified wireless nodes, NODES, to the
            %   wirelessNetworkSimulator object, OBJ. You must add the nodes to the
            %   simulator before running the simulation. Specify NODES as one of these
            %   options.
            %       - A vector of objects of type bluetoothLENode object, bluetoothNode
            %       object, wlanNode object, nrGNB object, or nrUE object.
            %       - A cell array, where each cell can contain a bluetoothLENode
            %       object, bluetoothNode object, wlanNode object, nrGNB object, or
            %       nrUE object.

            narginchk(2,2);
            nodes = reshape(nodes,[],1);
            if ~iscell(nodes)
                nodes = num2cell(nodes);
            else
                coder.internal.errorIf(sum(~cellfun(@isscalar,nodes))>0 ||  sum(cellfun(@iscell,nodes))>0, 'wirelessnetwork:wirelessNetworkSimulator:InvalidCellVector');
            end
            coder.internal.errorIf(obj.ResetRequired, 'wirelessnetwork:wirelessNetworkSimulator:DynamicAddNodesNotSupported');
            newNodes = numel(nodes);

            % Add new entries for node ID=>Index mapping
            for idx=1:newNodes
                coder.internal.errorIf(~isa(nodes{idx}, 'wirelessnetwork.internal.wirelessNode'), 'wirelessnetwork:wirelessNetworkSimulator:InvalidNodeType', class(nodes{idx}));
                nodeID = nodes{idx}.ID;
                if numel(obj.NodeIdxList) < nodeID
                    obj.NodeIdxList = [obj.NodeIdxList; zeros(nodeID-numel(obj.NodeIdxList), 1)];
                end
                if obj.NodeIdxList(nodeID) == 0
                    obj.NodeIdxList(nodeID) = obj.NumNodes+idx;
                else
                    if obj.Nodes{obj.NodeIdxList(nodeID)} == nodes{idx}
                        coder.internal.error('wirelessnetwork:wirelessNetworkSimulator:DuplicateNodeID', nodeID);
                    else
                        coder.internal.error('wirelessnetwork:wirelessNetworkSimulator:InvalidState');
                    end
                end
                obj.Nodes = [obj.Nodes; nodes(idx)];
                % Store the simulator reference in the node object. It is needed to query
                % current simulation time from the node.
                nodes{idx}.Simulator = obj;
            end

            obj.NumNodes = obj.NumNodes + newNodes;
            obj.NodeNextInvokeTimes = [obj.NodeNextInvokeTimes zeros(1, newNodes)];
        end

        function run(obj, simulationDuration)
            %run Run the simulation
            %
            %   run(OBJ, SIMULATIONDURATION) runs the multinode network simulation for
            %   the given simulation duration and performs the scheduled actions.
            %   SIMULATIONDURATION is the duration of the simulation in seconds. The
            %   SIMULATIONDURATION is rounded to nearest nanosecond. Call this method only
            %   once after invoking the <a href="matlab:help('wirelessNetworkSimulator/init')">init</a> method.
            %
            %   OBJ is an object of type wirelessNetworkSimulator.

            narginchk(2,2);
 
            validateattributes(simulationDuration, {'numeric'}, {'nonempty', 'scalar', 'positive', 'finite'}, mfilename, 'simulationDuration');
            simulationDuration = max(round(simulationDuration, 9), 1e-9);
            % Check whether simulator needs reset or not
            coder.internal.errorIf(obj.ResetRequired, 'wirelessnetwork:wirelessNetworkSimulator:InvalidState');
            obj.ResetRequired = true;
            obj.EndTime = simulationDuration;

            % Initialize simulation parameters
            recentlyRunNodesFlag = false(obj.NumNodes, 1); % List of nodes ran in the current step
            lastRunTime = 0;
            % Run simulator
            while(obj.CurrentTime <= simulationDuration)
                % Run nodes which are required to run at current time with 1 nanosecond
                % precision
                for nodeIdx = 1:obj.NumNodes
                    if obj.NodeNextInvokeTimes(nodeIdx) - obj.CurrentTime < 1e-9
                        obj.NodeNextInvokeTimes(nodeIdx) = run(obj.Nodes{nodeIdx}, obj.CurrentTime);
                        recentlyRunNodesFlag(nodeIdx) = true;
                    end
                end

                % Distribute the transmitted packets (if any) and reset
                % NodeNextInvokeTimes of the receiver nodes
                recentlyRanNodesIdx = find(recentlyRunNodesFlag);
                distributePackets(obj, recentlyRanNodesIdx);
                recentlyRunNodesFlag(:) = false;

                % Process actions scheduled at current time
                processActions(obj, lastRunTime);

                % Calculate invoke time for next run
                nextRunTime = nextInvokeTime(obj);

                % Advance the simulation time
                lastRunTime = obj.CurrentTime;
                obj.CurrentTime = round(nextRunTime,9);
            end

            % Set the current time to match the simulation duration
            obj.CurrentTime = simulationDuration;
        end

        function actionIdentifier = scheduleAction(obj, callbackFcn, userData, callAt, periodicity)
            %scheduleAction Schedule an action to process at a specific simulation time
            %
            %   ACTIONIDENTIFIER = scheduleAction(OBJ,CALLBACKFCN, USERDATA,CALLAT)
            %   schedules an action by invoking the callback function, CALLBACKFCN, by
            %   passing the data, USERDATA, at the specified time, CALLAT. It returns
            %   the identifier, ACTIONIDENTIFIER, to identify the action in the
            %   simulation. The action is added to the scheduled actions list, and is
            %   processed at a specific time during the simulation.
            %
            %   OBJ is an object of type wirelessNetworkSimulator.
            %
            %   CALLBACKFCN is a function handle associated with the action.
            %   CALLBACKFCN is a function handle with the signature:
            %     CALLBACKFCN(ACTIONIDENTIFIER,USERDATA)
            %
            %   USERDATA is the data to be passed to the callback function,
            %   CALLBACKFCN, associated with the action. If multiple parameters are to
            %   be passed as inputs to the callback function, use a structure or a cell
            %   array. Pass the USERDATA as [], if there is no data to be passed to the
            %   callback function
            %
            %   ACTIONIDENTIFIER is an integer which represents the unique identifier
            %   for the scheduled action. This value can be used to cancel the
            %   scheduled action.
            %
            %   CALLAT is the absolute simulation time (in seconds) to process the action.
            %
            %   ACTIONIDENTIFIER = scheduleAction(OBJ,CALLBACKFCN,
            %   USERDATA,CALLAT,PERIODICITY) also specifies the periodicity of the
            %   scheduled action, in seconds. To schedule a periodic action (an action
            %   called periodically in the simulation), set periodicity to a nonzero
            %   value. The periodicity value is rounded to nearest nanosecond. To
            %   schedule a time advance action (an action called at every time advance
            %   in a simulation), set periodicity to 0. CALLAT is considered as 0 for
            %   time advance action.

            narginchk(4, 5);
            validateattributes(callbackFcn, {'function_handle'}, {'nonempty'}, mfilename, 'callbackFcn');

            % Create action
            action.CallbackFcn = callbackFcn;
            action.UserData = userData;
            obj.ActionCounter = obj.ActionCounter + 1;
            action.ActionIdentifier = obj.ActionCounter;
            actionIdentifier = action.ActionIdentifier;

            % One-time action (no periodicity)
            if nargin == 4
                action.CallbackPeriodicity = Inf;
            else
                validateattributes(periodicity, {'numeric'}, {'nonempty', 'scalar', 'finite', '>=', 0}, mfilename, 'periodicity');
                action.CallbackPeriodicity = periodicity;
            end

            % Add action to actions queue
            if action.CallbackPeriodicity == 0
                % Add a time advance action to the actions list
                obj.TimeAdvanceActions = [obj.TimeAdvanceActions action];
            else
                % Add periodic or one-time action to the actions list
                action.CallbackPeriodicity = max(action.CallbackPeriodicity, 1e-9);
                callAt = round(callAt, 9);
                validateattributes(callAt, {'numeric'}, {'nonempty', 'scalar', 'finite', '>=', obj.CurrentTime}, mfilename, 'callAt');
                obj.ActionInvokeTimes = [obj.ActionInvokeTimes callAt];
                action.CalculatedActionTime = callAt; % Time at which action must be invoked
                obj.Actions = [obj.Actions action];
            end

            % If simulation did not start
            if ~obj.ResetRequired
                sortActions(obj); % Sort actions in order of time
            else
                obj.NewActionsAdded = true;
            end
        end

        function cancelAction(obj, actionIdentifier)
            %cancelAction Cancel scheduled action
            %
            %   cancelAction(OBJ,ACTIONIDENTIFIER) cancels the scheduled action
            %   associated with the action identifier, ACTIONIDENTIFIER.
            %   ACTIONIDENTIFIER is a unique identifier for the scheduled action,
            %   specified as an integer.
            %
            %   OBJ is an object of type wirelessNetworkSimulator.

            narginchk(2,2);
            validateattributes(actionIdentifier, {'numeric'}, {'nonempty', 'scalar', 'positive', 'finite'}, mfilename, 'actionIdentifier');

            % Cancel periodic or one-time action
            for actionIdx = 1:numel(obj.Actions)
                if obj.Actions(actionIdx).ActionIdentifier == actionIdentifier
                    obj.Actions(actionIdx) = [];
                    obj.ActionInvokeTimes(actionIdx) = [];
                    return
                end
            end

            % Cancel time advance action
            for actionIdx = 1:numel(obj.TimeAdvanceActions)
                if obj.TimeAdvanceActions(actionIdx).ActionIdentifier == actionIdentifier
                    obj.TimeAdvanceActions(actionIdx) = [];
                    return
                end
            end

            % Not a valid action identifier
            coder.internal.warning('wirelessnetwork:wirelessNetworkSimulator:InvalidActionID', actionIdentifier);
        end
    end

    methods(Static, Access = protected)
        function simObj = getState(flag)
            %getState Return the simulator object
            %
            % SIMSTATE = getState(FLAG) Returns the simulator object based on the flag
            %
            %   FLAG = 1, Return the simulator object if it exists FLAG = 0, Reset and
            %   return the simulator object
            %
            %   SIMOBJ - Simulator object

            persistent simulatorInstance;
            if flag == 1 && isempty(simulatorInstance) % Get the simulator object
                coder.internal.error('wirelessnetwork:wirelessNetworkSimulator:InvalidState');
            elseif flag == 0 % Reset the simulator
                if isempty(simulatorInstance)
                    simulatorInstance = wirelessNetworkSimulator();
                else
                    reset(simulatorInstance);
                end
            end
            simObj = simulatorInstance;
        end
    end

    methods(Access = protected)
        % Constructor
        function obj = wirelessNetworkSimulator()
            reset(obj);
        end

        function reset(simulatorObj)
            %reset Reset the simulator
            simulatorObj.Nodes = {};
            simulatorObj.ChannelFunction = "fspl";
            simulatorObj.DefaultChannelModel = true;
            simulatorObj.CurrentTime = 0;
            simulatorObj.Actions = [];
            simulatorObj.ActionInvokeTimes = [];
            simulatorObj.NodeNextInvokeTimes = [];
            simulatorObj.TimeAdvanceActions = [];
            simulatorObj.NumNodes = 0;
            simulatorObj.ActionCounter = 0;
            simulatorObj.NodeIdxList = zeros(0, 1);
            simulatorObj.ResetRequired = false;
            simulatorObj.EndTime = 0;
            simulatorObj.NewActionsAdded = false;

            % Reset the wireless node ID counter
            wirelessnetwork.internal.wirelessNode.reset();
        end

        % Sort actions in time order
        function sortActions(obj)
            [obj.ActionInvokeTimes, sIdx] = sort(obj.ActionInvokeTimes);
            obj.Actions = obj.Actions(sIdx);
        end

        % Calculate time in seconds, for advancing the simulation
        function dt = nextInvokeTime(obj)
            % Get minimum time from next invoke times of nodes and actions
            if obj.NumNodes == 0
                nextNodeDt = inf;
            else
                nextNodeDt = min(obj.NodeNextInvokeTimes);
            end
            if ~isempty(obj.ActionInvokeTimes)
                nextActionTimes = obj.ActionInvokeTimes(obj.ActionInvokeTimes ~= obj.CurrentTime);
                dt = min(nextActionTimes(1), nextNodeDt);
            else
                dt = nextNodeDt;
            end
        end

        % Process actions scheduled at current time. If an action is
        % periodic, update its next invocation time based on periodicity.
        % Otherwise, remove the action from action list.
        function processActions(obj, lastRunTime)
            % Process all time advance actions
            if obj.CurrentTime > lastRunTime
                for actionIdx = 1:numel(obj.TimeAdvanceActions)
                    action = obj.TimeAdvanceActions(actionIdx);
                    action.CallbackFcn(action.ActionIdentifier, action.UserData);
                end
            end

            % Process periodic or one-time actions
            processRemainingActions(obj)
        end

        function processRemainingActions(obj)
            %processRemainingActions Process periodic or one-time actions

            numActions = numel(obj.Actions);
            sortingRequired = false;
            actionIdx = 1;
            while actionIdx <= numActions
                if obj.ActionInvokeTimes(actionIdx) <= obj.CurrentTime
                    % Process current action
                    action = obj.Actions(actionIdx);
                    action.CallbackFcn(action.ActionIdentifier, action.UserData);

                    % Update next invocation time if it is periodic action
                    if action.CallbackPeriodicity ~= Inf
                        obj.Actions(actionIdx).CalculatedActionTime = obj.Actions(actionIdx).CalculatedActionTime+obj.Actions(actionIdx).CallbackPeriodicity;
                        obj.ActionInvokeTimes(actionIdx) = round(obj.Actions(actionIdx).CalculatedActionTime, 9);
                        sortingRequired = true;
                        actionIdx = actionIdx + 1; % Update the loop counter
                    else % Remove this one-time action from the list
                        obj.Actions(actionIdx) = [];
                        obj.ActionInvokeTimes(actionIdx) = [];
                        % Update the loop limit after one time action is
                        % removed. Loop counter update is not required as
                        % loop limit is updated
                        numActions = numel(obj.Actions);
                    end
                else % Ignore the rest of actions, which are of future time
                    break
                end
            end

            % Handle all the newly added actions at the end
            if obj.NewActionsAdded
                obj.NewActionsAdded = false;
                sortActions(obj);
                processRemainingActions(obj);
            else
                % Sort action in order of time
                if sortingRequired
                    sortActions(obj);
                end
            end
        end

        function distributePackets(obj, recentlyRunNodesIdx)
            %distributePackets Distribute the data from transmitter nodes to
            %receiver nodes
            %
            %   distributePackets(OBJ, RECENTLYRUNNODESIDX) distributes the
            %   data from the transmitting nodes into the receiving buffers of
            %   the nodes for which the packets are relevant
            %
            %   RECENTLYRUNNODESIDX - Index of the recently ran nodes that
            %   might have transmitted packets

            recentlyRunNodes = obj.Nodes(recentlyRunNodesIdx);
            numRecentlyRunNodes = numel(recentlyRunNodesIdx);
            numNodes = obj.NumNodes;
            rxNodes = obj.Nodes;

            % Get transmitted data (if any) from all the nodes recently run
            for txIdx = 1:numRecentlyRunNodes
                txNode = recentlyRunNodes{txIdx};
                txData = pullTransmittedData(txNode);

                % Distribute each of the transmitted packets in the transmit node
                for pktIdx = 1:numel(txData)
                    txPacket = txData(pktIdx);

                    if txPacket.DirectToDestination == 0
                        % Apply the channel on the packet

                        for rxIdx = 1:numNodes
                            % Copy Tx data to the relevant Rx nodes,
                            % after passing through the shared channel
                            rxNode = rxNodes{rxIdx};

                            [flag, rxInfo] = isPacketRelevant(rxNode, txPacket);
                            if flag
                                % Packet is relevant for the receiver node
                                if obj.DefaultChannelModel
                                    rxPacket = freeSpacePathLoss(obj, rxInfo, txPacket);
                                else
                                    rxPacket = obj.ChannelFunction(rxInfo, txPacket);
                                    % To support packet drops at the channel
                                    if isempty(rxPacket)
                                        continue;
                                    end
                                end
                                pushReceivedData(rxNode, rxPacket);

                                % For immediate reception, set the next invoke time as current time for receiver nodes
                                obj.NodeNextInvokeTimes(rxIdx) = obj.CurrentTime;
                            end
                        end
                    else % Send packet directly to a destination node, without applying channel
                        % Check whether destination node is added to the simulator
                        if numel(obj.NodeIdxList) < txPacket.DirectToDestination || txPacket.DirectToDestination < 0 || obj.NodeIdxList(txPacket.DirectToDestination) == 0
                            coder.internal.error('wirelessnetwork:wirelessNetworkSimulator:NodeNotAdded', txPacket.DirectToDestination)
                        end
                        idx = obj.NodeIdxList(txPacket.DirectToDestination);
                        pushReceivedData(obj.Nodes{idx}, txPacket);

                        % For immediate reception, set the next invoke time as current time for receiver nodes
                        obj.NodeNextInvokeTimes(idx) = obj.CurrentTime;
                    end
                end
            end
        end

        function outputData = freeSpacePathLoss(~, rxInfo, txData)
            %freeSpacePathLoss Apply free space path loss on the packet and
            %update the relevant fields of the output packet
            %
            % NOTE: This path loss function works only for SISO

            if (txData.NumTransmitAntennas > 1 || rxInfo.NumReceiveAntennas > 1)
                coder.internal.error('wirelessnetwork:wirelessNetworkSimulator:NoMIMOInDefaultChannel')
            end
            outputData = txData;
            % Calculate distance between transmitter and receiver in meters
            distance = norm(outputData.TransmitterPosition - rxInfo.Position);
            % Apply free space path loss (light speed 299,792,458 m/s)
            lambda = 299792458/(outputData.CenterFrequency);
            % Calculate free space path loss (in dB)
            pathLoss = fspl(distance, lambda);
            % Apply path loss on the power of the packet
            outputData.Power = outputData.Power - pathLoss;

            if outputData.Abstraction == 0
                % Apply the path loss effect on IQ samples
                scale = 10.^(-pathLoss/20);
                [numSamples, ~] = size(outputData.Data);
                outputData.Data(1:numSamples,:) = outputData.Data(1:numSamples,:)*scale;
            end
            outputData.Metadata.Channel.PathGains = 1;
            outputData.Metadata.Channel.PathDelays = 0;
            outputData.Metadata.Channel.PathFilters = 1;
            outputData.Metadata.Channel.SampleTimes = 0;
        end
    end
end