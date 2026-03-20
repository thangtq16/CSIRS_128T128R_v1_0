classdef networkTrafficVideoConference < wirelessnetwork.internal.networkTraffic
%networkTrafficVideoConference Model video conference application traffic
%pattern
%   VIDEOOBJ = networkTrafficVideoConference creates an object to generate
%   a video conference application traffic pattern using default values.
%   This object specifies the configuration parameters to generate the
%   video conference application traffic pattern based on IEEE 802.11ax
%   Evaluation Methodology.
%
%   VIDEOOBJ = networkTrafficVideoConference(Name,Value) creates an object,
%   VIDEOOBJ, with the specified property Name set to the specified Value.
%   You can specify additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN,ValueN).
%
%   networkTrafficVideoConference properties:
%
%   FrameInterval     - Time interval between two consecutive video frames
%                       in milliseconds
%   FrameSizeMethod   - Option to set source for video frame size
%   FixedFrameSize    - Size of a video frame in bytes
%   WeibullScale      - Scale parameter for Weibull distribution to
%                       calculate video frame size
%   WeibullShape      - Shape parameter for Weibull distribution to
%                       calculate video frame size
%   HasJitter         - Flag to indicate whether to model jitter
%   GammaShape        - Shape parameter for Gamma distribution to calculate
%                       jitter
%   GammaScale        - Scale parameter for Gamma distribution to calculate
%                       jitter
%   ProtocolOverhead  - Adds protocol overheads to the traffic in bytes
%   GeneratePacket    - Flag to indicate whether to generate video packet
%                       with payload
%   ApplicationData   - Application data to be filled in the video output
%                       packet
%
%   networkTrafficVideoConference methods:
%
%   generate - Generate the next video conference traffic packet
%
%   % Example 1:
%   %   Generate video conference traffic using default values
%
%   videoObj = networkTrafficVideoConference; % Create object
%   [dt, packetSize] = generate(videoObj); % Generate packet
%
%   % Example 2:
%   %   Generate video conference traffic with a frame interval of 60
%   %   milliseconds
%
%   videoObj = networkTrafficVideoConference('FrameInterval',60,'FrameSizeMethod','FixedSize','FixedFrameSize',1000); % Create and configure object
%   [dt, packetSize] = generate(videoObj); % Generate packet
%
%   % Example 3:
%   %   Generate video conference traffic and the data packet
%
%   videoObj = networkTrafficVideoConference('GeneratePacket',true); % Create and configure object
%   [dt, packetSize, packet] = generate(videoObj); % Generate packet with payload
%
%   % Example 4:
%   % Invoke video traffic object every 10 milliseconds to generate 5 packets
%
%   videoObj = networkTrafficVideoConference('FrameSizeMethod','FixedSize','FixedFrameSize',400); % Create and configure object
%   elapsedTime = 10; % Elapsed time in milliseconds
%   for i = 1:5 % Generate 5 packets
%       while true
%           [dt, packetSize] = generate(videoObj,elapsedTime);
%           if packetSize
%               disp('Packet generated');
%               break;
%           end
%       end
%   end
%
%   % Example 5:
%   %   Generate video conference traffic pattern to visualize
%   %   packet sizes and packet intervals
%
%   videoObj = networkTrafficVideoConference; % Create object
%   %   Call generate object function in a loop to generate 200 packets
%   for i = 1:200
%       [dt(i), packetSize(i)] = generate(videoObj);
%   end
%   stem(dt); % Stem graph to see video conference traffic pattern
%   title('dt versus Packet number');
%   xlabel('Packet number');
%   ylabel('dt in milliseconds');
%   figure;
%   stem(packetSize); % Stem graph to visualize different packet sizes
%   title('Packet size versus Packet number');
%   xlabel('Packet number');
%   ylabel('Packet size in bytes');
%
%   See also generate, networkTrafficVoIP, networkTrafficOnOff,
%   networkTrafficFTP.

%   Copyright 2020-2024 The MathWorks, Inc.

%#codegen

properties
    %FrameInterval Time interval between two consecutive video frames in
    % milliseconds
    %   Specify FrameInterval as a positive integer. The default value is
    %   40.
    FrameInterval (1, 1) {mustBeNumeric, mustBeInteger, mustBeGreaterThan(FrameInterval, 0)} = 40;

    %FrameSizeMethod Option to set source for video frame size
    %   Specify FrameSizeMethod as 'WeibullDistribution' or 'FixedSize'. If
    %   you set the FrameSizeMethod to 'WeibullDistribution', the object
    %   uses the Weibull distribution to calculate the video frame size.
    %   If you set this property to 'FixedSize', the object uses <a href="matlab:help('networkTrafficVideoConference.FixedFrameSize')">FixedFrameSize</a>
    %   as the video frame size. The default value is 'WeibullDistribution'.
    FrameSizeMethod {mustBeMember(FrameSizeMethod,{'WeibullDistribution','FixedSize'})} = 'WeibullDistribution';

    %FixedFrameSize Size of a video frame in bytes
    %   Specify FixedFrameSize as a integer in the range [1, 15000]. To
    %   enable this property set the <a href="matlab:help('networkTrafficVideoConference.FrameSizeMethod')">FrameSizeMethod</a>
    %   property to 'FixedSize'. The video frame can be segmented into
    %   multiple network packets based on the frame size. The default value
    %   is 5000.
    FixedFrameSize (1, 1) {mustBeNumeric, mustBeInteger, mustBeInRange(FixedFrameSize, 1, 15000)} = 5000;

    %WeibullScale Scale parameter for Weibull distribution to calculate
    % video frame size
    %   Specify WeibullScale as a positive scalar in the range (0, 54210].
    %   This property is applicable when the <a href="matlab:help('networkTrafficVideoConference.FrameSizeMethod')">FrameSizeMethod</a> value is set to
    %   'WeibullDistribution'. The default value is 6950.
    WeibullScale (1, 1) {mustBeReal, mustBeInRange(WeibullScale, 0, 54210, 'exclude-lower')} = 6950;

    %WeibullShape Shape parameter for Weibull distribution to calculate
    % video frame size
    %   Specify WeibullShape as a positive scalar in the range (0, 1]. This
    %   property is applicable when the <a href="matlab:help('networkTrafficVideoConference.FrameSizeMethod')">FrameSizeMethod</a> value is set to
    %   'WeibullDistribution'. The default value is 0.8099.
    WeibullShape (1, 1) {mustBeReal, mustBeInRange(WeibullShape, 0, 1, 'exclude-lower')} = 0.8099;

    %HasJitter Flag to indicate whether to model network jitter
    %   Specify HasJitter as true or false. The object applies jitter
    %   between the segmented packets. If you set this property to true,
    %   the object models jitter using the Gamma distribution function. To
    %   model the traffic coming from the network towards the end device,
    %   set this property to true. The default value is true.
    HasJitter (1, 1) logical = true;

    %GammaShape Shape parameter for Gamma distribution to calculate network
    % jitter
    %   Specify GammaShape as a positive scalar in the range (0, 5]. This
    %   property is applicable when the <a
    %   href="matlab:help('networkTrafficVideoConference.HasJitter')">HasJitter</a> flag is set to true. The
    %   default value is 0.2463.
    GammaShape (1, 1) {mustBeReal, mustBeInRange(GammaShape, 0, 5, 'exclude-lower')} = 0.2463;

    %GammaScale Scale parameter for Gamma distribution to calculate network
    % jitter
    %   Specify GammaScale as a positive scalar in the range (0, 100]. This
    %   property is applicable when the <a
    %   href="matlab:help('networkTrafficVideoConference.HasJitter')">HasJitter</a> flag is set to true. The
    %   default value is 60.227.
    GammaScale (1, 1) {mustBeReal, mustBeInRange(GammaScale, 0, 100, 'exclude-lower')} = 60.227;

    %ProtocolOverhead Adds protocol overheads to the traffic in bytes
    %   Specify ProtocolOverheads as an integer in the range [0, 60]. To
    %   accommodate layer 3, layer 4, and application protocol overheads in
    %   the network traffic, set this property. The default value is 28.
    ProtocolOverhead (1, 1) {mustBeReal, mustBeInRange(ProtocolOverhead, 0, 60)} = 28;
        % Validate protocol overheads value

    %GeneratePacket Flag to indicate whether to generate video packet with
    %payload
    %   Specify GeneratePacket value as true or false. To generate a video
    %   packet with payload, set this property to true. If you set this
    %   property to false, the generate object function generates no
    %   application data packet. The default value is false.
    GeneratePacket (1, 1) logical = false;

    %ApplicationData Application data to be filled in the packet
    %   Specify ApplicationData as a column vector of integers in the range
    %   [0, 255]. This property is applicable when the <a href="matlab:help('networkTrafficVideoConference.GeneratePacket')">GeneratePacket</a> value
    %   is set to true. If the size of the application data is greater than
    %   the packet size, the object truncates the application data. If the
    %   size of the application data is smaller than the packet size, the
    %   object appends zeros. The default value is a 1500-by-1 vector of
    %   ones.
    ApplicationData (:, 1) {mustBeInteger, mustBeInRange(ApplicationData, 0, 255)} = ones(1500, 1);
end

properties (Access = public, Hidden)
    %FrameSizeUpperLimit Upper limit to calculate video frame size in bytes
    %   Specify the frame size upper limit value as a positive integer. The
    %   generated frame size value must be less than or equal to the upper
    %   limit. If the generated frame size value is greater than the upper
    %   limit, the object discards the frame size and creates a new one.
    %   The default value is 15000.
    FrameSizeUpperLimit (1, 1) {mustBeInteger, mustBeGreaterThan(FrameSizeUpperLimit, 0)} = 15000;
end

properties (Access = private)
    %pVideoFrameSize Remaining size of the video frame yet to be packetized
    %   Remaining amount of frame in bytes which is not yet packetized by
    %   the generate object function.
    pVideoFrameSize = 0;

    %pNextInvokeTime Time in milliseconds after which the generate method
    % should be invoked again
    pNextInvokeTime = 0;

    %pAppData Application data to be filled in the packet as a column vector
    % of integers in the range [0, 255]
    %   If size of the input application data is greater than the packet
    %   size, the object truncates the input application data. If size of
    %   the application data is smaller than the packet size, the object
    %   appends zeros.
    pAppData = ones(1500, 1);

    %pSegmentsCount Number of segments in the video frame
    pSegmentsCount;

    %pCurrSegmentNum Current segment number in the video frame
    pCurrSegmentNum = 1;

    %pJitters Jitter values of all the segments in the video frame
    %   Jitter is the time gap between two consecutive segments of a frame
    %   in milliseconds. The maximum size of the jitter vector is limited by
    %   the video frame size and the maximum payload size. The upper limit
    %   of the jitter vector size will be equal to obj.pSegmentsCount - 1
    %   obj.pSegmentsCount = ceil(obj.pVideoFrameSize/obj.pMaxPayloadSize)
    pJitters = zeros(10, 1);

    %pMaxPayloadSize Maximum payload size in bytes excluding protocol overhead
    pMaxPayloadSize = 1472;

    %pSize Packet size in bytes
    pDataSize = 1500;
end

methods (Access = protected)
    function flag = isInactiveProperty(obj, prop)
        flag = false;
        if strcmp(prop, 'FixedFrameSize')
            flag = strcmp(obj.FrameSizeMethod, 'WeibullDistribution');
        end
        if strcmp(prop, 'WeibullScale')
            flag = strcmp(obj.FrameSizeMethod, 'FixedSize');
        end
        if strcmp(prop, 'WeibullShape')
            flag = strcmp(obj.FrameSizeMethod, 'FixedSize');
        end
        if strcmp(prop, 'GammaScale')
            flag = ~(obj.HasJitter);
        end
        if strcmp(prop, 'GammaShape')
            flag = ~(obj.HasJitter);
        end
        if strcmp(prop, 'ApplicationData')
            flag = ~(obj.GeneratePacket);
        end
    end
end

methods
    function obj = networkTrafficVideoConference(varargin)
        obj@wirelessnetwork.internal.networkTraffic(varargin{:});
    end

    function set.ProtocolOverhead(obj, value)
        % Update maximum payload size based on given protocol overhead value
        obj.ProtocolOverhead = value;
        obj.pMaxPayloadSize = obj.pDataSize - obj.ProtocolOverhead; %#ok<*MCCSOP>
    end

    function set.ApplicationData(obj, data)
        % Size of the given application data
        appDataSize = min(numel(data), 1500);
        obj.pAppData = zeros(1500, 1);
        obj.pAppData(1 : appDataSize) = data(1 : appDataSize); %#ok<*MCSUP>
        obj.ApplicationData = data;
    end

    function [dt, packetSize, varargout] = generate(obj, elapsedTime)
        %generate Generate next video conference application traffic packet
        %
        %   [DT, PACKETSIZE] = generate(OBJ) returns DT and PACKETSIZE,
        %   where DT is the time remaining to generate the next packet in
        %   milliseconds and PACKETSIZE is the size of the current packet
        %   in bytes.
        %
        %   [DT, PACKETSIZE] = generate(OBJ, ELAPSEDTIME) returns time
        %   remaining to generate the next packet in milliseconds, DT, and
        %   the size of the current packet in bytes if a packet is
        %   generated, otherwise zero, PACKETSIZE, based on the time
        %   elapsed since the previous call of this object function,
        %   ELAPSEDTIME.
        %
        %   [..., PACKET] = generate(...) also returns the application
        %   packet, PACKET, that contains a column vector of integers in
        %   the range [0, 255]. The object function returns PACKET only
        %   when the <a href="matlab:help('networkTrafficVideoConference.GeneratePacket')">GeneratePacket</a> property is set to true.
        %   PACKET contains the application data specified by the
        %   <a href="matlab:help('networkTrafficVideoConference.ApplicationData')">ApplicationData</a> property. If ApplicationData property is
        %   not specified, PACKET is a column vector of ones.
        %
        %   % Example 1:
        %   %   Generate video conference traffic pattern using default
        %   %   values
        %
        %   videoObj = networkTrafficVideoConference; % Create object
        %   [dt, packetSize] = generate(videoObj); % Generate packet
        %
        %   See also networkTrafficVideoConference.

        narginchk(1, 2);
        nargoutchk(2, 3);

        if nargin == 1
            obj.pNextInvokeTime = 0;
        else
            % Validate elapsed time value
            validateattributes(elapsedTime, {'numeric'}, {'real', 'scalar', ...
                'finite', '>=', 0}, '', 'ElapsedTime');
            % Calculate time remaining before generating next packet
            obj.pNextInvokeTime = obj.pNextInvokeTime - elapsedTime;
        end

        if nargout == 3
            varargout{1} = []; % Application packet
        end

        if obj.pNextInvokeTime <= 0
            % New video frame is generated only if all the segments of the
            % last generated frame are sent
            if ~obj.pVideoFrameSize
                if strcmp(obj.FrameSizeMethod, 'FixedSize') % Fixed frame size
                    obj.pVideoFrameSize = obj.FixedFrameSize;
                else % Use Weibull distribution
                    while true
                        r = rand();
                        % Generate new video frame size using Weibull distribution
                        obj.pVideoFrameSize = round(obj.WeibullScale * (-log(r)) ^ (1/obj.WeibullShape));
                        % If the frame size value is greater than the upper
                        % limit, discard the value and generate a new value
                        if obj.pVideoFrameSize <= obj.FrameSizeUpperLimit
                            break;
                        end
                    end
                end
                % Calculate number of segments for a frame
                obj.pSegmentsCount = ceil(obj.pVideoFrameSize/obj.pMaxPayloadSize);

                % The maximum size of the jitter vector is limited by the
                % obj.pVideoFrameSize and the obj.pMaxPayloadSize. The upper
                % limit for the array size is hence set to 10
                obj.pJitters = zeros(10, 1); % No jitter
                if obj.HasJitter && obj.pSegmentsCount > 1 % Model jitter
                    latencies = zeros(obj.pSegmentsCount, 1);
                    % Calculate latency for each packet in milliseconds
                    % using Gamma distribution
                    for i = 1:obj.pSegmentsCount
                        while true
                            latencies(i) = obj.gammaLatency();
                            % If the latency value is greater than the
                            % frame interval, discard the value and
                            % generate a new value
                            if latencies(i) <= obj.FrameInterval
                                break;
                            end
                        end
                    end
                    latencies = sort(latencies);
                    % Calculate jitter values for all the segments in the
                    % video frame
                    for i = 1 : obj.pSegmentsCount-1
                        obj.pJitters(i) = latencies(i+1) - latencies(i);
                    end
                end
            end
            if obj.pVideoFrameSize > obj.pMaxPayloadSize
                % Current packet size
                packetSize = obj.pMaxPayloadSize + obj.ProtocolOverhead;
                dt = obj.pJitters(obj.pCurrSegmentNum); % Add jitter
                obj.pCurrSegmentNum = obj.pCurrSegmentNum + 1;
                % Update the remaining video frame size
                obj.pVideoFrameSize = obj.pVideoFrameSize - obj.pMaxPayloadSize;
            else % Last segment of the video frame
                % Current packet size
                packetSize = obj.pVideoFrameSize + obj.ProtocolOverhead;
                dt = obj.FrameInterval - sum(obj.pJitters); % Time to generate the next video frame
                obj.pCurrSegmentNum = 1;
                obj.pVideoFrameSize = 0;
            end
            dt = round(dt*1e6)/1e6; % Limiting dt to nanoseconds accuracy
            obj.pNextInvokeTime = dt;

            % If the flag to generate a packet is true, generate the packet
            if nargout == 3 && obj.GeneratePacket
                varargout{1} = obj.pAppData(1:packetSize);
            end
        else % Time is still remaining to generate the next packet
            dt = obj.pNextInvokeTime;
            packetSize = 0;
        end
    end
end

methods(Access = private)
    function latency = gammaLatency(obj)
        % Generate random number using Gamma distribution
        %
        % References:
        %   [1]  Marsaglia, G. and Tsang, W.W. (2000) "A Simple Method
        %   for Generating Gamma Variables", ACM Trans. Math. Soft. 26(3):363-372.
        x = 0;
        latency = 0;
        if obj.GammaShape >= 1
            d = obj.GammaShape - 1/3;
            % Generate u from uniform distribution
            u = rand(1);
        else
            d = (obj.GammaShape+1)-1/3;
            % Generate u from uniform distribution
            u = rand(2);
        end
        c = 1/sqrt(9*d);
        % Generate v with x normal
        % Continue generating new v, if v <=0 which happens if 
        % x <-sqrt(9*shape-3) (rare condition)
        for k = 1:10000000 % Maximum number of times v can be generated in rare condition
            v = 0;
            while v <= 0
                x = randn();
                v = 1 + c * x;
            end
            v = v^3;
            if (u(1) < 1 - 0.331*(x*x)*(x*x) || ...
                    log(u(1))<(0.5*x*x+d*(1-v+log(v))))
                if obj.GammaShape >= 1
                    latency = obj.GammaScale * (d * v);
                else
                    latency = obj.GammaScale * (d * v * power(u(2),1/obj.GammaShape));
                end
                break;
            end
            u(1) = rand(1);
        end
    end
end
end