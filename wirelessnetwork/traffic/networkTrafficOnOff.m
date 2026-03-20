classdef networkTrafficOnOff < wirelessnetwork.internal.networkTraffic
    %networkTrafficOnOff Model On-Off application traffic pattern
    %   ONOFFOBJ = networkTrafficOnOff creates an object to generate On-Off
    %   application traffic pattern using default values. This object
    %   generates the On-Off application traffic pattern by specifying the
    %   configuration parameters.
    %
    %   ONOFFOBJ = networkTrafficOnOff(Name,Value) creates an object,
    %   ONOFFOBJ, with specified property Name set to the specified Value.
    %   You can specify additional name-value pair arguments in any order
    %   as (Name1,Value1,...,NameN,ValueN).
    %
    %   networkTrafficOnOff properties:
    %
    %   OnTime             - On state duration in seconds
    %   OffTime            - Off state duration in seconds
    %   OnExponentialMean  - Exponential distribution mean value to calculate
    %                        On state duration in seconds
    %   OffExponentialMean - Exponential distribution mean value to calculate
    %                        Off state duration in seconds
    %   DataRate           - Packet generation rate during the On state in
    %                        Kbps
    %   PacketSize         - Length of the packet to be generated in bytes
    %   GeneratePacket     - Flag to indicate whether to generate an
    %                        application packet with payload
    %   ApplicationData    - Application data to be added in the packet
    %
    %   networkTrafficOnOff methods:
    %
    %   generate - Generate next On-Off application traffic packet
    %
    %   % Example 1:
    %   %   Generate On-Off application traffic pattern using default values
    %
    %   onOffObj = networkTrafficOnOff; % Create object
    %   [dt, packetSize] = generate(onOffObj); % Generate pattern
    %
    %   % Example 2:
    %   %   Generate On-Off application traffic pattern with an On state mean
    %   %   value of 5
    %
    %   onOffObj = networkTrafficOnOff('OnExponentialMean',5); % Create object
    %   [dt, packetSize] = generate(onOffObj); % Generate pattern
    %
    %   % Example 3:
    %   %   Generate On-Off application traffic pattern and the data packet
    %
    %   onOffObj = networkTrafficOnOff('GeneratePacket',true); % Create object
    %   [dt, packetSize, packet] = generate(onOffObj); % Generate packet
    %
    %   % Example 4:
    %   %   Generate On-Off application traffic pattern to visualize packet
    %   %   sizes and packet intervals
    %
    %   onOffObj = networkTrafficOnOff; % Create object
    %   %   Call traffic generator function in a loop to generate 200 packets
    %   for i = 1:200
    %       [dt(i), packetSize(i)] = generate(onOffObj);
    %   end
    %   stem(dt); % Stem graph to see On-Off application traffic pattern
    %   title('dt vs Packet number');
    %   xlabel('Packet number');
    %   ylabel('dt in milliseconds');
    %   figure;
    %   stem(packetSize); % Stem graph to see different packet sizes
    %   title('Packet size vs Packet number');
    %   xlabel('Packet number');
    %   ylabel('Packet size in bytes');
    %
    %   See also generate, networkTrafficVoIP, networkTrafficFTP,
    %   networkTrafficVideoConference.

    %   Copyright 2020-2024 The MathWorks, Inc.

    %#codegen

    properties
        %OnTime On state duration in seconds
        %   Specify the On time value as a positive scalar. To specify a
        %   customized value for On time, set this property. If you do not
        %   specify this property, the object uses the exponential
        %   distribution to calculate the On time. The default value is [].
        OnTime

        %OffTime Off state duration in seconds
        %   Specify the Off time value as a nonnegative scalar. To specify
        %   a customized value for Off time, set this property. If you do
        %   not specify this property, the object uses the exponential
        %   distribution to calculate the Off time. The default value is
        %   [].
        OffTime

        %OnExponentialMean Exponential distribution mean value to calculate
        %On state duration in seconds
        %   Specify the On exponential value as a positive scalar.
        %   This property is applicable when the value of <a href="matlab:help('networkTrafficOnOff.OnTime')">OnTime</a>
        %   is not specified. The default value is 1.
        OnExponentialMean (1, 1) {mustBeReal, mustBeNumeric, mustBeGreaterThan(OnExponentialMean, 0)} = 1

        %OffExponentialMean Exponential distribution mean value to calculate
        %Off state duration in seconds
        %   Specify the Off exponential mean value as a nonnegative scalar.
        %   This property is applicable when the value of <a href="matlab:help('networkTrafficOnOff.OffTime')">OffTime</a>
        %   is not specified. The default value is 2.
        OffExponentialMean (1, 1) {mustBeNumeric, mustBeReal, mustBeFinite, mustBeGreaterThanOrEqual(OffExponentialMean, 0)} = 2

        %DataRate Packet generation rate during the On state in kilo bits
        %per second (Kbps)
        %   Specify the data rate value as a positive scalar greater than
        %   or equal to 1. If the data rate is low and the packet size is
        %   large, the object might not generate packets in On state. The
        %   default value is 5.
        DataRate (1, 1) {mustBeReal, mustBeFinite, mustBeGreaterThanOrEqual(DataRate, 1)} = 5

        %PacketSize Length of the packet to be generated in bytes
        %   Specify the packet size value as a positive scalar. If
        %   packet size is larger than the data generated in On time, the
        %   data is accumulated across multiple 'On' times to generate a
        %   packet. The default value is 1500.
        PacketSize (1, 1) {mustBeNumeric, mustBeInteger, mustBeGreaterThan(PacketSize, 0)} = 1500

        %GeneratePacket Flag to indicate whether to generate an application
        % packet with payload
        %   Specify generate packet value as true or false. To generate a
        %   packet with payload, set this property to true. If you set this
        %   property to false, the generate object function generates no
        %   application data packet. The default value is false.
        GeneratePacket (1, 1) logical = false

        %ApplicationData Application data to be added in the packet
        %   Specify application data as a column vector of integer values
        %   in the range [0, 255]. This property is applicable when the
        %   value of <a href="matlab:help('networkTrafficOnOff.GeneratePacket')">GeneratePacket</a> is set to true.
        %   If the size of the application data is greater than the packet
        %   size, the object truncates the application data. If the size of
        %   the application data is smaller than the packet size, the
        %   object appends zeros. The default value is a 1500-by-1 vector
        %   of ones.
        ApplicationData (:, 1) {mustBeInteger, mustBeInRange(ApplicationData, 0, 255)} = ones(1500, 1)
    end

    properties (Access = private)
        %pNextInvokeTime Time after which the generate method
        % should be invoked again
        pNextInvokeTime = 0;

        %pOnStateDuration On state duration in seconds 
        pOnStateDuration = 0;

        %pOffStateDuration Off state duration in seconds
        pOffStateDuration = 0;

        %pRemOnStateDuration Time remaining in On state before going to Off
        % state in seconds
        pRemOnStateDuration = 0;

        %pTransmissionTime Time to generate a packet at given data rate
        %in seconds
        pTransmissionTime = 0;

        %pAppDataUpdated Flag to indicate whether the application data is
        %updated
        pAppDataUpdated = false;

        %pOnTimeProvided Flag to indicate whether the fixed On state duration is provided
        pOnTimeProvided = false;

        %pOffTimeProvided Flag to indicate whether the fixed Off state duration is provided
        pOffTimeProvided = false;

        %pInitOnOffDurations Flag to indicate whether On state duration and Off state
        %duration are initialized
        pInitOnOffDurations = false;
    end

    properties(Hidden)
        %pAppData Application data to be added in the packet as a column vector
        %of integer values in the range [0, 255]
        pAppData = ones(1500, 1);
    end

    methods(Access = protected)
        function flag = isInactiveProperty(obj, prop)
            flag = false;
            if strcmp(prop,'OnTime')
                flag = isempty(obj.OnTime);
            end
            if strcmp(prop,'OffTime')
                flag = isempty(obj.OffTime);
            end
            if strcmp(prop,'OnExponentialMean')
                flag = ~isempty(obj.OnTime);
            end
            if strcmp(prop,'OffExponentialMean')
                flag = ~isempty(obj.OffTime);
            end
            if strcmp(prop,'ApplicationData')
                flag = ~(obj.GeneratePacket);
            end
        end
    end

    methods
        function obj = networkTrafficOnOff(varargin)
            obj@wirelessnetwork.internal.networkTraffic(varargin{:});

            % Calculate transmission time
            obj.pTransmissionTime = (obj.PacketSize*8)/(obj.DataRate*1000);
        end

        function set.DataRate(obj, value)
            obj.DataRate = value;
            obj.pTransmissionTime = (obj.PacketSize*8)/(value*1000);
        end

        function set.PacketSize(obj, value)
            obj.PacketSize = value;
            obj.pTransmissionTime = (value*8)/(obj.DataRate*1000);
            % Packet size has changed, update the application data to be added
            % in the packet as per given packet size
            updateAppData(obj);
        end

        function set.ApplicationData(obj, data)
            obj.ApplicationData = data;
            obj.pAppDataUpdated = true; %#ok<*MCSUP>
            % Application data has changed, update the application data to be
            % added in the packet as per given application data
            updateAppData(obj);
        end

        function set.OnTime(obj, value)
            if ~isempty(value)
                % Validate On time value
                validateattributes(value, {'numeric'}, {'scalar','>', 0}, ...
                    '', 'OnTime')
            end
            obj.OnTime = value;
            obj.pRemOnStateDuration = value;
            obj.pOnStateDuration = value;
            obj.pOnTimeProvided = true;
        end

        function set.OffTime(obj, value)
            if ~isempty(value)
                % Validate Off time value
                validateattributes(value, {'numeric'}, {'scalar', 'finite', ...
                    '>=', 0}, '', 'OffTime')
            end
            obj.OffTime = value;
            obj.pOffStateDuration = value;
            obj.pOffTimeProvided = true;
        end

        function [dt, packetSize, varargout] = generate(obj, elapsedTime)
            %generate Generate next On-Off application traffic packet
            %
            %   [DT, PACKETSIZE] = generate(OBJ) returns DT and PACKETSIZE,
            %   where DT is the time remaining to generate next packet in
            %   milliseconds and PACKETSIZE is the size of the current
            %   packet in bytes.
            %
            %   [DT, PACKETSIZE] = generate(OBJ, ELAPSEDTIME) specifies
            %   elapsed time, ELAPSEDTIME, in milliseconds, and returns DT
            %   and PACKETSIZE, where DT is the time remaining to generate
            %   next packet in milliseconds and PACKETSIZE is the size of
            %   the current packet in bytes.
            %
            %   [..., PACKET] = generate(...) also returns the application
            %   packet, PACKET, that contains a column vector of integer
            %   values in the range [0, 255]. The function returns PACKET
            %   only when the <a href="matlab:help('networkTrafficOnOff.GeneratePacket')">GeneratePacket</a> property is set to true.
            %   PACKET contains the application data specified by the
            %   <a href="matlab:help('networkTrafficOnOff.ApplicationData')">ApplicationData</a> property. If ApplicationData property is
            %   not specified, PACKET is a column vector of ones.
            %
            %   % Example 1:
            %   %   Generate On-Off application traffic pattern
            %   %   using default values
            %
            %   onoffObj = networkTrafficOnOff; % Create object
            %   [dt, packetSize] = generate(onoffObj); % Generate pattern
            %
            %   See also networkTrafficOnOff.

            narginchk(1, 2);
            nargoutchk(2, 3);

            % Check if the On/Off durations need to be initialized
            if ~obj.pInitOnOffDurations

                % Check if the On state duration is not provided as a fixed value
                if ~obj.pOnTimeProvided
                    % If On time is not provided, calculate it using an exponential distribution
                    obj.pOnStateDuration = obj.OnExponentialMean * -log(rand);

                    % Set the remaining On state duration to the calculated On state duration
                    obj.pRemOnStateDuration = obj.pOnStateDuration;
                end

                % Check if the Off state duration is not provided as a fixed value
                if ~obj.pOffTimeProvided
                    % If Off time is not provided, calculate it using an exponential distribution
                    obj.pOffStateDuration = obj.OffExponentialMean * -log(rand);
                end

                % Mark the On/Off durations as initialized
                obj.pInitOnOffDurations = true;
            end

            if nargin == 1
                obj.pNextInvokeTime = 0;
            else
                % Validate elapsed time value
                validateattributes(elapsedTime, {'numeric'}, {'real', ...
                    'scalar', 'finite', '>=', 0}, '', 'ElapsedTime');
                % Calculate time remaining before generating next packet
                % and limiting it to nanoseconds accuracy
                obj.pNextInvokeTime = obj.pNextInvokeTime - elapsedTime;
                obj.pNextInvokeTime = round(obj.pNextInvokeTime*1e6)/1e6;
            end

            if nargout == 3
                varargout{1} = []; % Application packet
            end

            if obj.pNextInvokeTime <= 0
                %  If On state duration is inf or Off state duration is
                %  zero, it always stays in On state
                if obj.pRemOnStateDuration == Inf || ~obj.pOffStateDuration
                    dt = obj.pTransmissionTime * 1000;
                elseif obj.pRemOnStateDuration < obj.pTransmissionTime
                    % If remaining On state duration is not big enough to
                    % generate the packet of required size then packet
                    % generation takes multiple On times

                    % Calculate the remaining transmission time by subtracting the remaining 
                    % On state duration from the total transmission time
                    remTransmissionTime = obj.pTransmissionTime - obj.pRemOnStateDuration;
                    % Check if on time is not provided as a fixed value
                    if ~obj.pOnTimeProvided
                        % Initialize the counter for the number of required Off states. This will track
                        % the number of Off states needed, which are placed between On states
                        requiredOffStates = 0;

                        while (remTransmissionTime >= 0)
                            % Generate a random On state duration using an exponential distribution
                            obj.pOnStateDuration = obj.OnExponentialMean*-log(rand);
                            % Subtract the generated On state duration from the remaining transmission time
                            remTransmissionTime = remTransmissionTime - obj.pOnStateDuration;
                            % Increment the counter for the required Off states. Note that there is one Off
                            % state between each pair of On states.
                            requiredOffStates = requiredOffStates + 1;
                        end
                        % Calculate the remaining time in the On state duration
                        obj.pRemOnStateDuration = -remTransmissionTime;
                    else
                        % Required additional On state duration to generate next packet
                        requiredOnStates = remTransmissionTime/obj.pOnStateDuration;
                        % Remaining time in On state
                        obj.pRemOnStateDuration = (ceil(requiredOnStates) - requiredOnStates) * obj.pOnStateDuration;
                        % Calculate the number of Off states needed
                        requiredOffStates = floor(requiredOnStates+1);
                    end

                    % Check if Off time is not provided as a fixed value
                    if ~obj.pOffTimeProvided
                        % Generate random Off state durations using an exponential distribution
                        offStateDurations = [obj.pOffStateDuration; obj.OffExponentialMean*-log(rand(requiredOffStates-1,1))];
                        obj.pOffStateDuration = obj.OffExponentialMean*-log(rand);
                        % Sum up the Off state durations to get the required Off state duration
                        requiredOffStateDuration = sum(offStateDurations);
                    else
                        % Calculate the required Off state duration to generate the next packet
                        requiredOffStateDuration = requiredOffStates*obj.pOffStateDuration;
                    end

                    % Next packet generation time in milliseconds
                    dt = (obj.pTransmissionTime + requiredOffStateDuration)*1000;
                else
                    % Remaining time in On state is used to generate next
                    % packet
                    obj.pRemOnStateDuration = obj.pRemOnStateDuration - obj.pTransmissionTime;
                    if ~obj.pRemOnStateDuration
                        % Next packet generation time in milliseconds
                        dt = (obj.pTransmissionTime + obj.pOffStateDuration) * 1000;
                        obj.pRemOnStateDuration = obj.pOnStateDuration;
                    else
                        % Next packet generation time in milliseconds
                        dt = obj.pTransmissionTime * 1000;
                    end
                end

                % Update remaining On state duration with configured On
                % state duration, if it is zero
                if ~obj.pRemOnStateDuration
                    obj.pRemOnStateDuration = obj.pOnStateDuration;
                end
                dt = round(dt*1e6)/1e6; % Limiting dt to nanoseconds accuracy
                % Return packet size and update next invoke time
                packetSize = obj.PacketSize;
                obj.pNextInvokeTime = dt;

                % If the flag to generate next packet is true, generate the packet
                if nargout == 3 && obj.GeneratePacket
                    varargout{1} = obj.pAppData;
                end
            else % Next packet generation time has not come yet
                % Return dt and packet size
                dt = obj.pNextInvokeTime;
                packetSize = 0;
            end
        end
    end

    methods(Access = private)

        function updateAppData(obj)
            %updateAppData Set the application data based on the given packet
            %size and application data. This function is called whenever the
            %user sets either packet size or application data.

            % Update application data to be added in the packet
            if ~obj.pAppDataUpdated % Use default data as application data
                % Default data is not enough for application data, update the
                % application data as per the given packet size
                obj.pAppData = ones(obj.PacketSize, 1);
            else  % Use given application data
                % Size of the given application data
                length = numel(obj.ApplicationData);
                obj.pAppData = zeros(obj.PacketSize, 1);
                obj.pAppData(1 : min(length, obj.PacketSize)) = obj.ApplicationData(1 : min(length, obj.PacketSize));
            end
        end
    end

end
