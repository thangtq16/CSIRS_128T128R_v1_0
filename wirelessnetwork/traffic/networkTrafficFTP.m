classdef networkTrafficFTP < wirelessnetwork.internal.networkTraffic
%networkTrafficFTP Model FTP application traffic pattern at source
%   FTPOBJ = networkTrafficFTP creates a default file transfer protocol
%   (FTP) application traffic pattern object. This object specifies the
%   configuration parameters to generate the FTP application traffic
%   pattern based on the IEEE 802.11ax Evaluation Methodology and 3GPP TR
%   36.814. This object supports three FTP models, namely local FTP (see
%   11ax Evaluation Methodology), FTP model 2 (see 3GPP TR 38.814), and FTP
%   model 3 (see 3GPP TR 36.889).
%
%   FTPOBJ = networkTrafficFTP(Name=Value) sets properties using one or
%   more name-value arguments. You can specify additional name-value pair
%   arguments in any order as (Name1=Value1,...,NameN=ValueN).
%
%   networkTrafficFTP properties:
%
%   FixedFileSize   - Custom size of the file to be transmitted in
%                     megabytes. This parameter is applicable for FTP model
%                     2 and FTP model 3.
%   LogNormalMu     - Truncated Lognormal distribution mu value to
%                     calculate file size. This parameter is applicable for
%                     the local FTP model.
%   LogNormalSigma  - Truncated Lognormal distribution sigma value to
%                     calculate file size. This parameter is applicable for
%                     the local FTP model.
%   UpperLimit      - Truncated Lognormal distribution upper limit to
%                     calculate file size in megabytes. This parameter is
%                     applicable for the local FTP model.
%   ReadingTime     - Time interval between two consecutive file transfers
%                     in milliseconds.
%   ExponentialMean - Exponential distribution mean value to calculate
%                     reading time in milliseconds. This parameter is
%                     applicable for the local FTP model and FTP model 2.
%   PoissonMean     - Poisson distribution mean value to calculate reading
%                     time in milliseconds. This parameter is applicable
%                     for FTP model 3.
%   GeneratePacket  - Flag to indicate whether to generate an FTP packet
%                     with payload.
%   ApplicationData - Application data to be added in the FTP packet.
%
%   networkTrafficFTP methods:
%
%   generate        - Generate next FTP application traffic packet.
%
%   % Example 1:
%   %   Generate FTP application traffic pattern using default values
%
%   ftpObj = networkTrafficFTP; % Create object
%   [dt, packetSize] = generate(ftpObj); % Generate pattern
%
%   % Example 2:
%   %   Generate FTP application traffic pattern with a reading time of 5
%   %   milliseconds
%
%   ftpObj = networkTrafficFTP(ReadingTime=5); % Create object
%   [dt, packetSize] = generate(ftpObj); % Generate pattern
%
%   % Example 3:
%   %   Generate FTP application traffic pattern and the data packet
%
%   ftpObj = networkTrafficFTP(GeneratePacket=true); % Create object
%   [dt, packetSize, packet] = generate(ftpObj); % Generate packet
%
%   % Example 4:
%   %   Generate FTP application traffic pattern to visualize packet
%   %   sizes and packet intervals
%
%   ftpObj = networkTrafficFTP; % Create object
%   %   Call traffic generator function in a loop to generate 20000 packets
%   for i = 1:20000
%       [dt(i), packetSize(i)] = generate(ftpObj);
%   end
%   stem(dt); % Stem graph to see FTP traffic pattern
%   title('dt vs Packet number');
%   xlabel('Packet number');
%   ylabel('dt in milliseconds');
%   figure;
%   stem(packetSize); % Stem graph to see different packet sizes
%   title('Packet size vs Packet number');
%   xlabel('Packet number');
%   ylabel('Packet size in bytes');
%
%   See also generate, networkTrafficVoIP, networkTrafficOnOff,
%   networkTrafficVideoConference.

%   Copyright 2020-2024 The MathWorks, Inc.

%#codegen

properties
    %FixedFileSize Custom size of the file to be transmitted in megabytes
    %   Specify the fixed file size value as a positive scalar. To specify
    %   a customized value for the file size, enable this property. If you
    %   do not specify this value, the object uses the truncated Lognormal
    %   distribution to calculate the file size. The default value is [].
    FixedFileSize

    %LogNormalMu Truncated Lognormal distribution mu value to calculate
    % file size
    %   Specify the lognormal mu value as a positive scalar. This property
    %   is applicable when the value of <a href="matlab:help('networkTrafficFTP.FixedFileSize')">FixedFileSize</a>
    %   is not specified. The default value is 14.45.
    LogNormalMu (1, 1) {mustBeFinite, mustBeReal, mustBeGreaterThan(LogNormalMu, 0)} = 14.45

    %LogNormalSigma Truncated Lognormal distribution sigma value to
    % calculate file size
    %   Specify the lognormal sigma value as a positive scalar. This
    %   property is applicable when the value of <a href="matlab:help('networkTrafficFTP.FixedFileSize')">FixedFileSize</a>
    %   is not specified. The default value is 0.35.
    LogNormalSigma (1, 1) {mustBeFinite, mustBeReal, mustBeGreaterThan(LogNormalSigma, 0)} = 0.35

    %UpperLimit Truncated Lognormal distribution upper limit to calculate
    % file size in megabytes
    %   Specify the upper limit value as a positive scalar. This property
    %   is applicable when the value of <a href="matlab:help('networkTrafficFTP.FixedFileSize')">FixedFileSize</a>
    %   is not specified. The generated file size value must be less than
    %   or equal to the upper limit. If the generated file size value is
    %   greater than the upper limit, the object discards the file size and
    %   creates a new one. The default value is 5.
    UpperLimit = 5

    %ReadingTime Time interval between two consecutive file transfers in
    % milliseconds
    %   Specify the reading time value as a positive scalar. To specify a
    %   customized value for the reading time, enable this property. If you
    %   do not specify this value, the object uses the exponential
    %   distribution to calculate the reading time. The default value is
    %   [].
    ReadingTime

    %ExponentialMean Exponential distribution mean value to calculate
    % reading time in milliseconds
    %   Specify the exponential mean value as a nonnegative scalar. This
    %   property is applicable when the value of <a href="matlab:help('networkTrafficFTP.ReadingTime')">ReadingTime</a>
    %   is not specified. The default value is 180000.
    ExponentialMean (1, 1) {mustBeInteger, mustBeGreaterThanOrEqual(ExponentialMean, 0)} = 180000

    %PoissonMean Poisson distribution mean value to calculate packet
    % inter-arrival time in milliseconds
    %   Specify the Poisson mean value as a nonnegative scalar. The default
    %   value is [].
    PoissonMean

    %GeneratePacket Flag to indicate whether to generate an FTP packet with
    %payload
    %   Specify generate packet value as true or false. To generate an FTP
    %   packet with payload, set this property to true. If you set this
    %   property to false, the generate object function generates no
    %   application data packet. The default value is false.
    GeneratePacket (1, 1) logical = false

    %ApplicationData Application data to be added in the packet
    %   Specify application data as a column vector of integer values in
    %   the range [0, 255]. This property is applicable when the
    %   value of <a href="matlab:help('networkTrafficFTP.GeneratePacket')">GeneratePacket</a> is set to true.
    %   If the size of the application data is greater than the packet
    %   size, the object truncates the application data. If the size of the
    %   application data is smaller than the packet size, the object
    %   appends zeros. The default value is a 1500-by-1 vector of ones.
    ApplicationData (:, 1) {mustBeInteger, mustBeInRange(ApplicationData, 0, 255)} = ones(1500, 1)
end

properties (Access = private)
    %pRemPktsInFile Remaining packets in a file to be generated.
    pRemPktsInFile = 0;

    %pMaxPacketSize Size of packet excluding TCP and IP overheads
    % Max packet size will be either 536 or 1460 excluding 40 byte header.
    % Files are transferred using an MTU size of 1500 bytes or 576 bytes,
    % as specified in 11ax evaluation methodology.
    pMaxPacketSize = 0;

    %pFileSize Size of a file to be transmitted
    % File size value is generated using different probability distribution
    % functions. The value can also be a customized input given by the
    % user.
    pFileSize = 0;

    %pNextInvokeTime Time after which the generate method
    % should be invoked again.
    pNextInvokeTime = 0;

    %pAppData Application data to be added in the packet as a column vector
    % of integer values in the range [0, 255].
    pAppData = ones(1500,1);
end

properties (Hidden)
    %PacketInterArrivalTime Time interval between two consecutively generated
    % packets, specified in milliseconds
    %   Specify the packet inter-arrival time value as a nonnegative
    %   scalar. This property is applicable when the value of <a href="matlab:help('networkTrafficFTP.PoissonMean')">PoissonMean</a>
    %   is not specified. The default value is 0.
    PacketInterArrivalTime (1, 1) {mustBeNumeric, mustBeFinite, mustBeReal, mustBeGreaterThanOrEqual(PacketInterArrivalTime, 0)} = 0
end

properties (Constant, Hidden)
    %TCPIPOverhead TCP/IP overhead in bytes
    TCPIPOverhead = 40;
end

methods (Access = protected)
    function flag = isInactiveProperty(obj, prop)
        flag = false;
        if strcmp(prop,'FixedFileSize')
            flag = isempty(obj.FixedFileSize);
        end
        if strcmp(prop,'LogNormalMu')
            flag = ~isempty(obj.FixedFileSize);
        end
        if strcmp(prop,'LogNormalSigma')
            flag = ~isempty(obj.FixedFileSize);
        end
        if strcmp(prop,'UpperLimit')
            flag = ~isempty(obj.FixedFileSize);
        end
        if strcmp(prop,'ExponentialMean')
            flag = ~isempty(obj.ReadingTime);
        end
        if strcmp(prop,'ReadingTime')
            flag = isempty(obj.ReadingTime);
        end
        if strcmp(prop,'PacketInterArrivalTime')
            flag = ~isempty(obj.PoissonMean);
        end
        if strcmp(prop,'PoissonMean')
            flag = isempty(obj.PoissonMean);
        end
        if strcmp(prop,'ApplicationData')
            flag = ~(obj.GeneratePacket);
        end
    end
end

methods
    function obj = networkTrafficFTP(varargin)
        obj@wirelessnetwork.internal.networkTraffic(varargin{:});
    end

    function set.FixedFileSize(obj, value)
        if ~isempty(value)
            % Validate fixed file size value
            validateattributes(value, {'numeric'}, {'real', 'scalar', ...
                'finite', '>', 0}, '', 'FixedFileSize');
        end
        obj.FixedFileSize = value;
    end

    function set.UpperLimit(obj, value)
        FileSizes = zeros(1,1000);
        for i = 1:1000
            % Use truncated Lognormal distribution to generate file size
            FileSizes(i) = exp(obj.LogNormalMu + (obj.LogNormalSigma * randn)); %#ok<MCSUP>
        end
        % Number of generated file sizes that are greater than the given
        % upper limit
        numOfFileSizes = sum(FileSizes(:)>(value*1e6));
        % Percentage of the generated file sizes that are greater than the
        % given upper limit
        percOfFileSizes = numOfFileSizes/10;
        if percOfFileSizes <= 50
            % Validate upper limit value
            validateattributes(value, {'numeric'}, {'real', 'scalar'}, ...
                '', 'UpperLimit');
            obj.UpperLimit = value;
        else
            error('wirelessnetwork:networkTrafficFTP:InvalidUpperLimit','File size is exceeding the upper limit with given LogNormalSigma and LogNormalMu');
        end
    end

    function set.ReadingTime(obj, value)
        if ~isempty(value)
            % Validate reading time value
            validateattributes(value, {'numeric'}, {'real', 'scalar', ...
                'finite', '>', 0}, '', 'ReadingTime');
        end
        obj.ReadingTime = value;
    end

    function set.PoissonMean(obj, value)
        if ~isempty(value)
            % Validate Poisson mean value
            validateattributes(value, {'numeric'}, {'real', 'scalar', ...
                'finite', '>=', 0}, '', 'PoissonMean');
        end
        obj.PoissonMean = value;
    end

    function set.ApplicationData(obj, data)
        % Size of the given application data
        appDataSize = min(numel(data), 1500);
        obj.pAppData = zeros(1500, 1); %#ok<MCSUP>
        obj.pAppData(1 : appDataSize) = data(1 : appDataSize); %#ok<MCSUP>
        obj.ApplicationData = data;
    end

    function [dt, packetSize, varargout] = generate(obj, elapsedTime)
            %generate Generate next FTP application traffic packet
            %
            %   [DT, PACKETSIZE] = generate(OBJ) returns DT and PACKETSIZE,
            %   where DT is the time remaining to generate next packet in
            %   milliseconds and PACKETSIZE is the size of the current
            %   packet in bytes.
            %
            %   [DT, PACKETSIZE] = generate(OBJ, ELAPSEDTIME) specifies
            %   elapsed time, ELAPSEDTIME in milliseconds, and returns DT
            %   and PACKETSIZE, where DT is the time remaining to generate
            %   next packet in milliseconds and PACKETSIZE is the size of
            %   the current packet in bytes.
            %
            %   [..., PACKET] = generate(...) also returns the application
            %   packet, PACKET, that contains a column vector of integer
            %   values in the range [0, 255]. The function returns PACKET
            %   only when the <a href="matlab:help('networkTrafficFTP.GeneratePacket')">GeneratePacket</a> property is set to true.
            %   PACKET contains the application data specified by the
            %   <a href="matlab:help('networkTrafficFTP.ApplicationData')">ApplicationData</a> property. If ApplicationData property is
            %   not specified, PACKET is a column vector of ones. FTP models 2 (TR
            %   36.814) and 3 (TR 36.889) have a fixed file size of 0.5 MB. In
            %   contrast, in the case of 11ax Evaluation Methodology, the file size
            %   distribution follows a Lognormal pattern.
            %
            %   % Example 1:
            %   %   Generate FTP application traffic pattern
            %   %   using default values
            %
            %   ftpObj = networkTrafficFTP; % Create object
            %   [dt, packetSize] = generate(ftpObj); % Generate pattern
            %
            %   See also networkTrafficFTP.

            narginchk(1, 2);
            nargoutchk(2, 3);

            if nargin == 1
                obj.pNextInvokeTime = 0;
            else
                % Validate elapsed time value
                validateattributes(elapsedTime, {'numeric'}, {'real', ...
                    'scalar', 'finite', '>=', 0}, '', 'ElapsedTime');
                % Calculate the remaining time before generating next packet
                obj.pNextInvokeTime = obj.pNextInvokeTime - elapsedTime;
            end

            if nargout == 3
                varargout{1} = []; % Application packet
            end

            if obj.pNextInvokeTime <= 0
                if ~obj.pRemPktsInFile
                    % Generate a random number to select maximum packet
                    % size for a new file
                    r = rand;
                    % According to the 11ax Evaluation Methodology
                    % specification, MTU size (including IP and TCP header)
                    % can be 1500 or 576 bytes.
                    if r <= 0.24
                        % MTU size is 576 bytes. This implies that 24% of
                        % files are transferred using this MTU size
                        obj.pMaxPacketSize = 536;
                    else % MTU size is 1500 bytes. This implies that 76% of
                        % files are transferred using this MTU size
                        obj.pMaxPacketSize = 1460;
                    end

                    if ~isempty(obj.FixedFileSize)
                        % Use user provided file size
                        obj.pFileSize = round(obj.FixedFileSize*1000*1000); % Converting file size in bytes
                    else
                        while true
                            % Use truncated Lognormal distribution to generate file size
                            obj.pFileSize = exp(obj.LogNormalMu + (obj.LogNormalSigma * randn));
                            if obj.pFileSize <= obj.UpperLimit*1000*1000
                                break;
                            end
                        end
                    end
                    % Calculate packets required to transmit this file
                    obj.pRemPktsInFile = ceil(obj.pFileSize/obj.pMaxPacketSize);
                end
                % Set packet size
                if obj.pFileSize >= obj.pMaxPacketSize
                    packetSize = obj.pMaxPacketSize + obj.TCPIPOverhead;
                    % Update remaining file size
                    obj.pFileSize = obj.pFileSize - obj.pMaxPacketSize;
                    dt = obj.PacketInterArrivalTime;
                else
                    packetSize = obj.pFileSize + obj.TCPIPOverhead;
                    % Update file size
                    obj.pFileSize = 0;
                    if ~isempty(obj.ReadingTime)
                        readingTime = obj.ReadingTime; % Use user provided value as a reading time
                    else
                        if isempty(obj.PoissonMean)
                            % Generate a random number using Exponential distribution as defined in 3GPP
                            % 36.814 FTP Model 2
                            readingTime = obj.ExponentialMean * -log(1-rand);
                        else % FTP Model 3
                            % Poisson random number generator based on the Knuth, Donald Ervin (1997).
                            % Seminumerical Algorithms. The Art of Computer Programming. Vol. 2 (3rd
                            % ed.). The algorithm is updated to address issue with large lambda values
                            % (lambda > 745), which will result in e^(-lambda) represented as 0.
                            lambda = obj.PoissonMean;
                            L = -lambda;
                            i=0;
                            p=0;
                            while p > L
                                i=i+1;
                                randUniform=rand;
                                p=p+log(randUniform);
                            end
                            % Number of iterations 'i' is the random number generated with Poisson
                            % distribution
                            readingTime = i-1;
                        end
                    end
                    dt = readingTime;
                end
                packetSize = round(packetSize);
                dt = round(dt);
                % Update next invoke time with dt
                obj.pNextInvokeTime = dt;
                % Update remaining packets to be generated in the current file
                obj.pRemPktsInFile = obj.pRemPktsInFile - 1;

                % If the flag to generate next packet is true, generate the packet
                if nargout == 3 && obj.GeneratePacket
                    varargout{1} = obj.pAppData(1:packetSize);
                end
            else % Next packet generation time has not come yet
                % Return dt and packet size
                dt = obj.pNextInvokeTime;
                packetSize = 0;
            end
    end
end
end
