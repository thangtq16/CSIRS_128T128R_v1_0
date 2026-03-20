classdef interferenceBuffer < comm.internal.ConfigBase & handle
    %interferenceBuffer Create an object to model interference in the PHY receiver
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   OBJ = interferenceBuffer creates a default object to model
    %   interference in the PHY receiver.
    %
    %   OBJ = interferenceBuffer(Name=Value) creates an object to model
    %   interference in the PHY receiver, OBJ, with the specified property Name
    %   set to the specified Value. You can specify additional name-value
    %   arguments in any order as (Name1=Value1, ..., NameN=ValueN).
    %
    %   interferenceBuffer methods:
    %
    %   addPacket           - Add packet to the buffer
    %   resultantWaveform   - Return the resultant waveform after combining
    %                         all the packets. This method is applicable
    %                         only for full PHY
    %   packetList          - Return the list of packets overlapping
    %                         in time domain and frequency domain (based on
    %                         InterferenceModeling value)
    %   receivedPacketPower - Total power of the packets on the channel
    %   bufferChangeTime    - Return the time at which there is a change
    %                         in the state of the buffer
    %   retrievePacket      - Return the packet stored in the specified buffer index
    %   removePacket        - Remove the packet stored in the specified buffer index
    %
    %   interferenceBuffer properties (configurable):
    %
    %   CenterFrequency       - Receiver center frequency in Hz
    %   Bandwidth             - Receiver bandwidth in Hz
    %   SampleRate            - Receiver sampling rate, in samples per second
    %   InterferenceModeling  - Type of interference modeling
    %   MaxInterferenceOffset - Maximum frequency offset to determine the
    %                           interfering signal
    %   ResultantWaveformDataType - Data type of resultant waveform
    %
    % Limitation: For abstracted PHY, the power calculation method
    % 'receivedPacketPower' assumes that all the packets in the buffer have
    % same center frequency and bandwidth.

    %   Copyright 2021-2024 The MathWorks, Inc.

    properties(GetAccess = public, SetAccess = private)
        %CenterFrequency Center frequency of the receiver
        %   Specify the center frequency as a nonnegative scalar. The
        %   default is 5.18e9 Hz.
        CenterFrequency (1,1) {mustBeNumeric, mustBeReal, mustBeNonnegative, mustBeFinite} = 5.18e9

        %Bandwidth Bandwidth of the receiver
        %   Specify the bandwidth as a positive scalar. The default
        %   is 20e6 Hz.
        Bandwidth (1,1) {mustBeNumeric, mustBeReal, mustBePositive, mustBeFinite} = 20e6

        %SampleRate Sampling rate
        %   Specify the sample rate of the receiver, in samples per second.
        %   It is a positive scalar integer. The default value is 40e6 Hz.
        %   If the SampleRate is too low during signal combining,
        %   suitable sample rate is calculated automatically to avoid
        %   signal folding.
        SampleRate (1,1) {mustBeNumeric, mustBeInteger, mustBePositive, mustBeFinite} = 40e6

        %InterferenceModeling Type of interference modeling
        %   Specify the type of interference modeling as "co-channel",
        %   "overlapping-adjacent-channel", or "non-overlapping-adjacent-channel".
        %   If you set this property to "co-channel", the object considers signals
        %   with the same center frequency and bandwidth as the signal of interest
        %   (SOI), to be interference. If you set this property to
        %   "overlapping-adjacent-channel", the object considers signals
        %   overlapping in time and frequency, to be interference. If you set this
        %   property to "non-overlapping-adjacent-channel", the object considers
        %   all the signals overlapping in time and with frequency in the range
        %   [f1-MaxInterferenceOffset, f2+MaxInterferenceOffset], to be
        %   interference. f1 and f2 are the starting and ending frequencies of SOI,
        %   respectively. The default value is "overlapping-adjacent-channel".
        InterferenceModeling (1,1) string {mustBeMember(InterferenceModeling,["co-channel","overlapping-adjacent-channel","non-overlapping-adjacent-channel"])} = "overlapping-adjacent-channel"
    
        %MaxInterferenceOffset Maximum frequency offset to determine the interfering signal
        %   Specify the maximum interference offset as a nonnegative scalar. Units
        %   are in Hz. This property specifies the offset between the edge of the
        %   SOI frequency and the edge of the interfering signal. This property
        %   applies only when the InterferenceModeling property is set to
        %   "non-overlapping-adjacent-channel". If you specify this property as
        %   Inf, the object considers all the signals that overlap in time,
        %   regardless of their frequency, to be interference. If you specify this
        %   property as a finite nonnegative scalar, the object considers all the
        %   signals overlapping in time and with frequency in the range
        %   [f1-MaxInterferenceOffset, f2+MaxInterferenceOffset], to be
        %   interference. The default value is Inf.
        MaxInterferenceOffset (1,1) {mustBeNumeric,mustBeNonnegative} = Inf

        %ResultantWaveformDataType Data type of resultant waveform
        %    Specify the data type of the matrix returned by the
        %    resultantWaveform method as "single" or "double". The default
        %    is "double".
        ResultantWaveformDataType (1,1) string {mustBeMember(ResultantWaveformDataType,["single","double"])} = "double"
    end

    properties (Access = private)
        %BufferSize Initial buffer size
        % Number of packets that can be stored in the packet buffer initially
        % The default value is 20.
        BufferSize = 20

        %BufferStepUpSize Step size to expand the packet buffer when the existing buffer is filled up
        % The default value is 20.
        BufferStepUpSize = 20

        %PacketBuffer Array containing the details of all the packets being received
        PacketBuffer

        %IsActive Array of flags that maps to elements in 'PacketBuffer'
        % It represents whether packet entries in 'PacketBuffer' are active
        % or inactive (expired)
        IsActive = []

        %PacketEndTimes Array indicating the end time of each packet in the PacketBuffer
        PacketEndTimes = []

        %ACPRObject Adjacent channel power ratio (ACPR) calculation object
        ACPRObject

        %MinTimeOverlapThreshold Dependent property on MinTimeOverlap property
        % A packet must overlap more than this value to consider it as
        % interference. This value is calculated as MinTimeOverlap-eps to
        % handle floating point issues in comparisions.
        MinTimeOverlapThreshold

        %InterferenceFidelity Fidelity level of interference modeling
        % The values 0 ("co-channel"), 1 ("overlapping-adjacent-channel"),
        % and 2 ("non-overlapping-adjacent-channel") represent
        % progressively increasing fidelity levels.
        InterferenceFidelity = 0

        %PacketStruct Holds the packet structure of the wireless packet
        PacketStruct

        %EmptyPacketStruct Holds the empty packet structure of the wireless packet
        EmptyPacketStruct

        %FileName Current feature file name
        FileName = mfilename

        %NextBufferCleanupTime Next time at which buffer cleanup should be performed
        NextBufferCleanupTime = 0

        %ResamplingCache Matrix stores the resampling information
        % Resampling information, specified as an N-by-4 matrix. N is the
        % number of rows. The columns 1,2,3 and 4 represent the required
        % sample rate, the actual sample rate, the resampling factor
        % numerator, and resampling factor denominator, respectively.
        ResamplingCache = zeros(0,4);

        %% Properties used for memory optimization. This functionality is  enabled only if 'WaveformBufferDuration' is greater than 0 and there is no PHY abstraction (Full PHY i.e Abstraction = false)
        %WaveformBuffer Circular buffer to hold the combined waveform IQ samples
        % To hold the combined waveform IQ samples to optimize memory. All
        % the waveforms combined and stored in this buffer must have the same center
        % frequency, bandwidth, sample rate, and 'Data' field column count.
        WaveformBuffer

        %WaveformBufferInfo Contains information related to WaveformBuffer
        % It is a structure and contains below fields.
        %   CenterFrequency - Center frequency of the packets (Hz)
        %   Bandwidth - Bandwidth of the packets (Hz)
        %   SampleRate - Sample rate of the packets
        %   NumReceiveAntennas - Number of receive antennas
        %   NumSamples - Size of WaveformBuffer in terms of number of
        %   samples (i.e SampleRate*WaveformBufferDuration)
        % All the above field values must be same for all the packets when
        % WaveformBufferDuration is configured. These values are decided based on
        % the first packet added to the buffer using addPacket method
        WaveformBufferInfo

        %LastWBCleanUpTime Last time when WaveformBuffer was cleared
        LastWBCleanUpTime = 0

        %NextWBCleanUpTime Next time at which WaveformBuffer should be cleared
        NextWBCleanUpTime = 0
    end

    properties(Hidden, SetAccess=protected)
        %MinTimeOverlap A packet must overlap at least this value to consider it as interference
        % Specify this property as a positive scalar. The default value is 1e-9 seconds.
        MinTimeOverlap {mustBeNumeric, mustBeReal, mustBePositive, mustBeFinite} = 1e-9

        %BufferCleanupCheckGap Minimum gap time between consecutive buffer cleanup checks (i.e to remove the obsolete packets) when addPacket method is invoked
        % Set this property as a positive scalar. The default value is 0.001 seconds.
        BufferCleanupCheckGap {mustBeNonempty, mustBeGreaterThan(BufferCleanupCheckGap, 0)} = 0.001
    end
    properties(Hidden)
        %DisableValidation Disable the validation for input arguments of each method
        % Specify this property as a scalar logical. When true,
        % validation is not performed on the input arguments.
        DisableValidation (1, 1) logical = false

        %BufferCleanupTime Minimum time a packet must to be buffered
        % Specify this property as a non negative scalar. This value gets
        % updated when newly recieved packet duration is greater than this
        % value at that point. The default value is 0.
        BufferCleanupTime {mustBeNonempty, mustBeGreaterThanOrEqual(BufferCleanupTime, 0)} = 0

        %% Properties used for memory optimization. This functionality is enabled only if 'WaveformBufferDuration' is greater than 0 and there is no PHY abstraction (Full PHY i.e Abstraction = false)
        %WaveformBufferDuration Size of circular buffer in seconds
        % Size of the circular buffer in terms of time (seconds), specified
        % as one of these options.
        % Positive scalar - Facilitates combining waveforms into a single
        % waveform for memory optimization when center frequency, bandwidth, sample
        % rate, and 'Data' field column count are same across packets. Set
        % the value to at least twice the maximum packet duration.
        % 0 - Enables storing waveforms individually. This is the default value for WaveformBufferDuration.
        WaveformBufferDuration {mustBeNonempty, mustBeGreaterThanOrEqual(WaveformBufferDuration, 0)} = 0
    end

    methods
        function obj =  interferenceBuffer(varargin)
            %interferenceBuffer Construct an object of this class

            % Name-value pair check
            if mod(nargin, 2) == 1
                error(message('MATLAB:system:invalidPVPairs'));
            end

            % Set name-value pairs
            for idx = 1:2:nargin
                obj.(varargin{idx}) = varargin{idx+1};
            end

            % Allocate buffer
            obj.PacketStruct = wirelessnetwork.internal.wirelessPacket;
            obj.EmptyPacketStruct = repmat(obj.PacketStruct, 0, 1);
            obj.PacketBuffer = repmat(obj.PacketStruct, obj.BufferSize, 1);

            % Set the minimum overlap threshold value
            obj.MinTimeOverlapThreshold = obj.MinTimeOverlap - eps;

            % Initialize the properties
            obj.IsActive = false(obj.BufferSize, 1);
            obj.PacketEndTimes = -1 * ones(obj.BufferSize, 1);

            % Initialize comm.ACPR object to return the power measure at the adjacent channel
            obj.ACPRObject = comm.ACPR('MainChannelPowerOutputPort', false, ...
                'AdjacentChannelPowerOutputPort', true, 'SampleRate', obj.SampleRate);

            updateInterferenceType(obj);
        end

        function bufferIdx = addPacket(obj, packet)
            %addPacket Add packet to the buffer and return the buffer element index
            %
            %   BUFFERIDX = addPacket(OBJ, PACKET) adds a packet to the
            %   buffer. It assumes that all the packets added to the buffer
            %   are from same PHY abstraction type.
            %
            %   BUFFERIDX - Index of the buffer element in which packet is stored
            %
            %   OBJ is an instance of class interferenceBuffer.
            %
            %   PACKET is a structure created using
            %   <a href="matlab:help('wirelessnetwork.internal.wirelessPacket')">wirelessPacket</a>.

            pktStartTime = packet.StartTime;
            if ~obj.DisableValidation
                currFileName = obj.FileName;

                % Validate start time
                validateattributes(pktStartTime, {'numeric'}, ...
                    {'scalar', 'real', 'nonnegative', 'finite'}, currFileName, 'StartTime');

                % Validate duration
                validateattributes(packet.Duration, {'numeric'}, ...
                    {'scalar', 'real', 'positive', 'finite'}, currFileName, 'Duration');

                % Validate center frequency
                validateattributes(packet.CenterFrequency, {'numeric'}, ...
                    {'scalar', 'real', 'nonnegative', 'finite'}, currFileName, 'CenterFrequency');

                % Validate bandwidth
                validateattributes(packet.Bandwidth, {'numeric'}, ...
                    {'scalar', 'real', 'positive', 'finite'}, currFileName, 'Bandwidth');

                % Validate abstraction type
                validateattributes(packet.Abstraction, {'logical', 'numeric'}, ...
                    {'scalar'}, currFileName, 'Abstraction');

                if packet.Abstraction % Abstracted PHY
                    % Validate power
                    validateattributes(packet.Power, {'numeric'}, ...
                        {'scalar', 'real', 'finite'}, currFileName, 'Power');
                else % Full PHY
                    % Validate sample rate
                    validateattributes(packet.SampleRate, {'numeric'}, ...
                        {'scalar', 'integer', 'positive', 'finite'}, currFileName, 'SampleRate');
                    % Validate data
                    validateattributes(packet.Data, {'double', 'single'}, {'nonempty'}, currFileName, 'Data');
                end
            end

            % Optimize the memory by combining all the packets into a single waveform
            if obj.WaveformBufferDuration > 0 % Memory optimization is enabled
                if ~packet.Abstraction
                    numRxAnts = size(packet.Data,2);
                    if obj.NextWBCleanUpTime == 0 % Initialize the circular buffer when first packet is received
                        if obj.BufferCleanupTime > 0
                            error(message('wirelessnetwork:interferenceBuffer:MemoryOptimizationNotSupported'));
                        end
                        obj.NextWBCleanUpTime = obj.WaveformBufferDuration;
                        numSamples = round(obj.WaveformBufferDuration * packet.SampleRate);
                        obj.WaveformBuffer = zeros(numSamples, numRxAnts, obj.ResultantWaveformDataType);
                        obj.WaveformBufferInfo = struct('CenterFrequency', packet.CenterFrequency, 'Bandwidth', packet.Bandwidth, 'SampleRate', packet.SampleRate, 'NumReceiveAntennas', numRxAnts, 'NumSamples', numSamples);
                    end

                    wbInfo = obj.WaveformBufferInfo;
                    if wbInfo.CenterFrequency ~= packet.CenterFrequency || wbInfo.Bandwidth ~= packet.Bandwidth || wbInfo.SampleRate ~= packet.SampleRate || wbInfo.NumReceiveAntennas ~= numRxAnts
                        error(message('wirelessnetwork:interferenceBuffer:InvalidMemoryOptimizationParam'));
                    end

                    if packet.Duration > obj.WaveformBufferDuration
                        error(message('wirelessnetwork:interferenceBuffer:InvalidPacketDuration', sprintf('%.9f', packet.Duration),sprintf('%.9f', obj.WaveformBufferDuration)));
                    end

                    % Clear the circular buffer
                    bufferSize = wbInfo.NumSamples;
                    if obj.NextWBCleanUpTime < pktStartTime+packet.Duration
                        endTime = pktStartTime-obj.BufferCleanupTime;
                        % Check whether time elapsed between two consecutive buffer
                        % clearing operations is more than circular buffer duration or not
                        if endTime - obj.LastWBCleanUpTime < obj.WaveformBufferDuration
                            % Calculate the obsolete indices to clear in the buffer
                            startObsoleteIndex = round(mod(obj.LastWBCleanUpTime, obj.WaveformBufferDuration) * packet.SampleRate) + 1;
                            endObsoleteIndex = round(mod(endTime, obj.WaveformBufferDuration) * packet.SampleRate);
                        else % All the indices in the buffer are obsolete
                            startObsoleteIndex = 1;
                            endObsoleteIndex = bufferSize;
                            endTime = pktStartTime;
                        end

                        if startObsoleteIndex < endObsoleteIndex  % Clear the buffer in forward direction
                            obj.WaveformBuffer(startObsoleteIndex:endObsoleteIndex, :) = zeros(1, obj.ResultantWaveformDataType);
                        else % Clear the buffer in forward and backward directions
                            obj.WaveformBuffer(startObsoleteIndex:bufferSize, :) = zeros(1, obj.ResultantWaveformDataType);
                            obj.WaveformBuffer(1:endObsoleteIndex, :) = zeros(1, obj.ResultantWaveformDataType);
                        end
                        obj.LastWBCleanUpTime = endTime;
                        obj.NextWBCleanUpTime = endTime+obj.WaveformBufferDuration;
                    end

                    % Add the waveform to the circular buffer
                    waveformLength = size(packet.Data,1);
                    % Calculate the overlapping start and end index of
                    % the resultant waveform time-domain samples
                    overlapStartIndex = round(mod(pktStartTime, obj.WaveformBufferDuration) * packet.SampleRate) + 1;
                    if overlapStartIndex > bufferSize
                        overlapStartIndex = 1;
                    end
                    overlapEndIndex = overlapStartIndex + waveformLength - 1;
                    if overlapEndIndex <= bufferSize % If the waveform fits within the buffer without wrapping
                        obj.WaveformBuffer(overlapStartIndex:overlapEndIndex, :) = obj.WaveformBuffer(overlapStartIndex:overlapEndIndex, :) + packet.Data;
                    else % If the waveform wraps around the buffer
                        numWrapAroundSamples = overlapEndIndex - bufferSize;
                        obj.WaveformBuffer(overlapStartIndex:bufferSize, :) = obj.WaveformBuffer(overlapStartIndex:bufferSize, :) + packet.Data(1:waveformLength-numWrapAroundSamples, :);
                        obj.WaveformBuffer(1:numWrapAroundSamples, :) = obj.WaveformBuffer(1:numWrapAroundSamples, :) + packet.Data(waveformLength-numWrapAroundSamples+1:waveformLength, :);
                    end
                    packet.Data = []; % Clear the IQ data as waveform is added to the circular buffer
                end
            end

            if obj.NextBufferCleanupTime < pktStartTime
                obj.NextBufferCleanupTime = pktStartTime+obj.BufferCleanupCheckGap;
                removeObsoletePackets(obj, pktStartTime-obj.BufferCleanupTime);
            end

            if obj.BufferCleanupTime < packet.Duration
                obj.BufferCleanupTime = packet.Duration;
            end

            % Store the received packet
            bufferIdx = find(~obj.IsActive, 1); % Find an inactive buffer index
            if isempty(bufferIdx)
                bufferIdx = autoResizePacketBuffer(obj, pktStartTime);
            end
            obj.IsActive(bufferIdx) = true;
            obj.PacketEndTimes(bufferIdx) = pktStartTime + packet.Duration; % End time of the packet
            obj.PacketBuffer(bufferIdx) = packet;
        end

        function [rxWaveform, numPackets, sampleRate] = resultantWaveform(obj, startTime, endTime, varargin)
            %resultantWaveform Return the resultant waveform for the reception duration
            %
            %   [RXWAVEFORM, NUMPACKETS] = resultantWaveform(OBJ,
            %   STARTTIME, ENDTIME) returns the resultant waveform for the
            %   given start and end times.
            %
            %   RXWAVEFORM Resultant of all the waveforms. It is a T-by-R
            %   matrix of complex values. Here T represents number of
            %   time-domain samples and N represents the number of receive
            %   antennas.
            %
            %   NUMPACKETS Number of overlapping packets in time domain and
            %   frequency domain (based on InterferenceModeling value).
            %
            %   SAMPLERATE Sample rate of the resultant waveform.
            %
            %   STARTTIME is the start time of reception in seconds.
            %   It is a nonnegative scalar.
            %
            %   ENDTIME is the end time of reception in seconds. It
            %   is a positive scalar. It must be greater than STARTTIME.
            %
            %   [RXWAVEFORM, NUMPACKETS] = resultantWaveform(OBJ,
            %   STARTTIME, ENDTIME, Name=Value) specifies additional
            %   name-value arguments described below. When a name-value
            %   argument is not specified, the object function uses the
            %   default value of the object.
            %
            %   'CenterFrequency' is the center frequency of the receiver
            %   in Hz. It is a nonnegative scalar.
            %
            %   'Bandwidth' is the bandwidth of the receiver in Hz. It is a
            %   positive scalar.

            % Check whether the validation is enabled
            if ~obj.DisableValidation
                narginchk(3, 7);
                % Name-value pair check
                if mod(nargin-3, 2) == 1
                    error(message('MATLAB:system:invalidPVPairs'));
                end

                % Validate start time
                validateattributes(startTime, {'numeric'}, ...
                    {'scalar', 'real', 'nonnegative', 'finite'}, obj.FileName, 'startTime');

                % Validate end time
                validateattributes(endTime, {'numeric'}, ...
                    {'scalar', 'real', 'positive', '>', startTime, 'finite'}, obj.FileName, 'endTime');

                [centerFrequency, bandwidth] = validateInputs(obj, varargin);
            else
                centerFrequency = obj.CenterFrequency;
                bandwidth = obj.Bandwidth;

                for idx = 1:2:nargin-3
                    if strcmp(varargin{idx}, "CenterFrequency")
                        centerFrequency = varargin{idx+1};
                    else % "Bandwidth"
                        bandwidth = varargin{idx+1};
                    end
                end
            end

            % Get indices of the overlapping packets
            [receivedPackets, numPackets] = getOverlappingPackets(obj, startTime, endTime, centerFrequency, bandwidth);
            % Return the combined waveform
            rxWaveform = [];
            sampleRate = [];
            if numPackets > 0
                if obj.WaveformBufferDuration > 0 % When memory optimization is enabled
                    % Center frequency and bandwidth should be same as
                    % all the other packets in the buffer
                    if obj.WaveformBufferInfo.CenterFrequency ~= centerFrequency || obj.WaveformBufferInfo.Bandwidth ~= bandwidth
                        error(message('wirelessnetwork:interferenceBuffer:InvalidResultantWaveformInputs'));
                    end
                    if obj.WaveformBufferDuration < endTime-startTime % Resultant waveform duration should be less than or equal to the WaveformBufferDuration
                        error(message('wirelessnetwork:interferenceBuffer:InvalidWaveformDuration'));
                    end
                    duration = endTime-startTime;
                    sampleRate = obj.WaveformBufferInfo.SampleRate;

                    % Extract the resultant waveform from the circular buffer
                    bufferSize = obj.WaveformBufferInfo.NumSamples;
                    waveformLength = round(duration*sampleRate);
                    if waveformLength > bufferSize % Maximum duration of the resultant waveform is limited by circular buffer size
                        waveformLength = bufferSize;
                    end
                    % Calculate the overlapping start and end index
                    bufferStartIndex = round(mod(startTime, obj.WaveformBufferDuration) * sampleRate) + 1;
                    if bufferStartIndex > bufferSize
                        bufferStartIndex = 1;
                    end
                    bufferEndIndex = bufferStartIndex + waveformLength - 1;

                    if bufferEndIndex <= bufferSize % If the waveform is not wrapped around
                        rxWaveform = obj.WaveformBuffer(bufferStartIndex:bufferEndIndex, :);
                    else % If the waveform is wrapped around the buffer
                        numWrapAroundSamples = bufferEndIndex - bufferSize;
                        rxWaveform = [obj.WaveformBuffer(bufferStartIndex:bufferSize, :); obj.WaveformBuffer(1:numWrapAroundSamples, :)];
                    end
                else
                    sampleRate = calculateSampleRate(centerFrequency, receivedPackets, obj.SampleRate);
                    rxWaveform = combineWaveforms(obj, startTime, endTime, centerFrequency, receivedPackets, sampleRate);
                end
            end
        end

        function packets = packetList(obj, startTime, endTime, varargin)
            %packetList Return the list of packets which are overlapping in time/frequency
            %
            %   PACKETS = packetList(OBJ, STARTTIME, ENDTIME) returns a structure array
            %   containing packets within the buffer. If InterferenceModeling =
            %   "overlapping-adjacent-channel", it returns the packets overlapping in
            %   both time and frequency. If InterferenceModeling =
            %   "non-overlapping-adjacent-channel", it returns the packets overlapping
            %   in time whose frequency is until MaxInterferenceOffset frequency offset
            %   from SOI, modeling adjacent channel interference. If
            %   InterferenceModeling = "co-channel", it returns all packets overlapping
            %   time having same center frequency and bandwidth.If there is no matching
            %   packet, it returns empty <a
            %   href="matlab:help('wirelessnetwork.internal.wirelessPacket')">wirelessPacket</a>.
            %
            %   STARTTIME is the start time of reception in seconds.
            %   It is a nonnegative scalar.
            %
            %   ENDTIME is the end time of reception in seconds. It is a
            %   nonnegative scalar. It must be greater than or equal to STARTTIME.
            %
            %   PACKETS = packetList(OBJ, STARTTIME, ENDTIME, Name=Value) specifies
            %   additional name-value arguments described below. When a
            %   name-value argument is not specified, the object function
            %   uses the default value of the object.
            %
            %   'CenterFrequency' is the center frequency of the receiver
            %    in Hz. It is a nonnegative numeric scalar.
            %
            %   'Bandwidth' is the bandwidth of the receiver in Hz. It is a
            %   positive numeric scalar.

            % Check whether the validation is enabled
            if ~obj.DisableValidation
                narginchk(3, 7);

                % Name-value pair check
                if mod(nargin-3, 2) == 1
                    error(message('MATLAB:system:invalidPVPairs'));
                end

                % Validate start time
                validateattributes(startTime, {'numeric'}, ...
                    {'scalar', 'real', 'nonnegative', 'finite'}, obj.FileName, 'startTime');

                % Validate end time
                validateattributes(endTime, {'numeric'}, ...
                    {'scalar', 'real', 'nonnegative', '>=', startTime, 'finite'}, obj.FileName, 'endTime');

                [centerFrequency, bandwidth] = validateInputs(obj, varargin);
            else
                centerFrequency = obj.CenterFrequency;
                bandwidth = obj.Bandwidth;

                for idx = 1:2:nargin-3
                    if strcmp(varargin{idx}, "CenterFrequency")
                        centerFrequency = varargin{idx+1};
                    else % "Bandwidth"
                        bandwidth = varargin{idx+1};
                    end
                end
            end

            % To query the packets at a time instant
            if startTime == endTime
                % Set the end time as more than min overlap time
                endTime = endTime+obj.MinTimeOverlap;
            end

            packets = getOverlappingPackets(obj, startTime, endTime, centerFrequency, bandwidth);
        end

        function currentPower = receivedPacketPower(obj, currentTime, varargin)
            %receivedPacketPower Return the current power in the channel
            %
            %   CURRENTPOWER = receivedPacketPower(OBJ, CURRENTTIME)
            %   returns the power of the packets on each subchannel.
            %
            %   CURRENTPOWER - Total power of the packets on the channel in
            %   dBm. If there is no power in the channel it returns -Inf.
            %
            %   CURRENTTIME is the current time at the receiver in
            %   seconds. It is a nonnegative scalar.
            %
            %   CURRENTPOWER = receivedPacketPower(OBJ, CURRENTTIME,
            %   Name=Value) specifies additional name-value arguments
            %   described below. When a name-value argument is not
            %   specified, the object function uses the default value of
            %   the object.
            %
            %   'CenterFrequency' is the center frequency of the receiver
            %    in Hz. It is a nonnegative scalar.
            %
            %   'Bandwidth' is the bandwidth of the receiver in Hz. It is a
            %    positive scalar.
            %
            %   'SubchannelBandwidth' is the bandwidth of the subchannel of
            %   the receiver in Hz. It is a positive scalar. When
            %   specifying 'SubchannelBandwidth', receivedPacketPower
            %   returns the power measurement for each subchannel. When not
            %   specifying, the value of 'SubchannelBandwidth' is equal to
            %   that of 'Bandwidth'.

            % Check whether the validation is enabled
            if ~obj.DisableValidation
                narginchk(2, 8);

                % Name-value pair check
                if mod(nargin-2, 2) == 1
                    error(message('MATLAB:system:invalidPVPairs'));
                end

                % Validate start time
                validateattributes(currentTime, {'numeric'}, ...
                    {'scalar', 'real', 'nonnegative', 'finite'}, obj.FileName, 'currentTime');

                [centerFrequency, bandwidth, subchannelBandwidth] = validateInputs(obj, varargin);
            else
                centerFrequency = obj.CenterFrequency;
                bandwidth = obj.Bandwidth;
                subchannelBandwidth = bandwidth; % By default

                for idx = 1:2:nargin-2
                    if strcmp(varargin{idx}, "CenterFrequency")
                        centerFrequency = varargin{idx+1};
                    elseif strcmp(varargin{idx}, "Bandwidth") % "Bandwidth"
                        bandwidth = varargin{idx+1};
                    else % "SubchannelBandwidth"
                        subchannelBandwidth = varargin{idx+1};
                    end
                end
            end

            numSubchannels = bandwidth/subchannelBandwidth;
            activeSignalIdx = obj.IsActive & ((obj.PacketEndTimes - currentTime) > obj.MinTimeOverlapThreshold);
            currentPower = -Inf(numSubchannels, 1); % dBm
            minEndTime = min(obj.PacketEndTimes(activeSignalIdx));

            [overlappingPackets, numPackets, acprRequiredFlag] = getOverlappingPackets(obj, currentTime, minEndTime, centerFrequency, bandwidth);
            if numPackets == 0
                return;
            end
            % Determine the packet type
            phyAbstractionType = overlappingPackets(1).Abstraction;
            if phyAbstractionType % Abstracted PHY
                signalPowset = [overlappingPackets.Power];
                currentPowerFullBand = sum(10.0 .^ (signalPowset / 10.0)); % Power in milliwatts
                currentPowerFullBand = 10 * log10(currentPowerFullBand); % Power in dBm
                % Power scaling factor for each subchannel - 3 dB if doubling the number of subchannels
                scalingFactorNumSubchannels = 3 * log2(numSubchannels);
                currentPower = (currentPowerFullBand - scalingFactorNumSubchannels) * ones(numSubchannels,1);
            else % Full PHY
                % Passing bandwidth as minimum required sample rate when
                % calculating the desired samplerate
                sampleRate = calculateSampleRate(centerFrequency, overlappingPackets, bandwidth);

                % Overlap duration is not more than 1 sample duration or no partial frequency overlap
                % Using the eps threshold to work consistently during
                % floating point comparisons
                if ((minEndTime-currentTime) - (1/sampleRate)) < eps || ~acprRequiredFlag
                    signalPowset = [overlappingPackets.Power];
                    currentPowerFullBand = sum(10.0 .^ (signalPowset / 10.0)); % Power in milliwatts
                    currentPowerFullBand = 10 * log10(currentPowerFullBand); % Power in dBm
                    scalingFactorNumSubchannels = 3 * log2(numSubchannels);
                    currentPower = (currentPowerFullBand - scalingFactorNumSubchannels) * ones(numSubchannels,1);
                else
                    % Calculate the power when there is partial frequency overlap
                    rxWaveform = combineWaveforms(obj, currentTime, minEndTime, centerFrequency, overlappingPackets, sampleRate);
                    if size(rxWaveform, 1) <= 1 % ACPR accepts column vector as input
                        signalPowset = [overlappingPackets.Power];
                        currentPowerFullBand = sum(10.0 .^ (signalPowset / 10.0)); % Power in milliwatts
                        currentPowerFullBand = 10 * log10(currentPowerFullBand); % Power in dBm
                        scalingFactorNumSubchannels = 3 * log2(numSubchannels);
                        currentPower = (currentPowerFullBand - scalingFactorNumSubchannels) * ones(numSubchannels,1);
                    else
                        release(obj.ACPRObject);
                        if numSubchannels == 1
                            % Only single subchannel
                            centerFreqOffsetPerSubchannel = 0;
                        else
                            centerFreqOffset = subchannelBandwidth / 2; % center frequency offset of subchannel assuming the center frequency is at 0 Hz
                            centerFreqOffsetPerSubchannel = centerFreqOffset * (-numSubchannels+1:2:numSubchannels);
                        end
                        if numel(obj.ACPRObject.AdjacentMeasurementBandwidth) ~= numel(subchannelBandwidth) || any(obj.ACPRObject.AdjacentMeasurementBandwidth ~= subchannelBandwidth)
                            obj.ACPRObject.AdjacentMeasurementBandwidth = subchannelBandwidth;
                        end
                        if numel(obj.ACPRObject.AdjacentChannelOffset) ~= numel(centerFreqOffsetPerSubchannel) || any(obj.ACPRObject.AdjacentChannelOffset ~= centerFreqOffsetPerSubchannel)
                            obj.ACPRObject.AdjacentChannelOffset = centerFreqOffsetPerSubchannel;
                        end
                        if obj.ACPRObject.SampleRate ~= sampleRate
                            obj.ACPRObject.SampleRate = sampleRate;
                        end
                        numRxAnts = size(rxWaveform, 2);
                        currentPower = zeros(numSubchannels, 1);
                        for idx=1:numRxAnts
                            [~, subChannelPowerdBm] = obj.ACPRObject(rxWaveform(:, idx));
                            % Sum of powers at each receive antenna
                            currentPower = currentPower + (10.0 .^ (double(subChannelPowerdBm.')/ 10.0)); % Power in milliwatts. Force power measurement to double so consistent type
                        end
                        currentPower = 10 * log10(currentPower); % Power in dBm
                    end
                end
            end
        end

        function t = bufferChangeTime(obj, currentTime)
            %bufferChangeTime Returns the time after which the state of the buffer is expected to change
            %
            %   T = bufferChangeTime(OBJ, CURRENTTIME) returns the time
            %   after which the state of the buffer is expected to change
            %
            %   T - Returns the time duration in seconds after which the
            %   state of the buffer is expected to change. If there is no
            %   change in the state of the buffer it returns Inf.
            %
            %   CURRENTTIME is the current time at the receiver in
            %   seconds. It is a nonnegative scalar.

            % Check whether the validation is enabled
            if ~obj.DisableValidation
                narginchk(2, 2);
                % Validate current time
                validateattributes(currentTime, {'numeric'}, ...
                    {'scalar', 'real', 'nonnegative', 'finite'}, obj.FileName, 'currentTime');
            end

            activeSignalIdx = obj.IsActive & ((obj.PacketEndTimes - currentTime) > obj.MinTimeOverlapThreshold);
            t =  min(obj.PacketEndTimes(activeSignalIdx)) - currentTime;
            if isempty(t)
                t = inf;
            end
        end
    
        function packets = retrievePacket(obj, bufferIdx)
            %retrievePacket Return the packet stored in the specified buffer index
            %
            %   PACKET = retrievePacket(OBJ, BUFFERIDX) returns the packets
            %   stored in the specified buffer element indices, BUFFERIDX.
            %   If there is no stored packet, it returns empty
            %   <a href="matlab:help('wirelessnetwork.internal.wirelessPacket')">wirelessPacket</a> type
            %
            %   BUFFERIDX - Indices of the buffer elements at which the
            %   packets are stored. It is a positive integer vector.
            %
            %   OBJ is an instance of class interferenceBuffer.
            %
            %   PACKET is a structure created using
            %   <a href="matlab:help('wirelessnetwork.internal.wirelessPacket')">wirelessPacket</a>.

            % Check whether the validation is enabled
            if ~obj.DisableValidation
                narginchk(2, 2);
                % Validate buffer index
                validateattributes(bufferIdx, {'numeric'}, ...
                    {'vector', 'integer', 'positive', '<=', numel(obj.IsActive)}, obj.FileName, 'bufferIdx');
            end
            packets = obj.PacketBuffer(bufferIdx(obj.IsActive(bufferIdx)));
        end

        function removePacket(obj, bufferIdx)
            %removePacket Remove the packet stored in the specified buffer index
            %
            %   removePacket(OBJ, BUFFERIDX) removes the packets from the
            %   specified buffer element indices, BUFFERIDX, if it exists.
            %
            %   BUFFERIDX - Indices of the buffer elements at which the
            %   packets are stored. It is a positive integer vector.
            %
            %   OBJ is an instance of class interferenceBuffer.

            % Check whether the validation is enabled
            if ~obj.DisableValidation
                narginchk(2, 2);
                % Validate buffer index
                validateattributes(bufferIdx, {'numeric'}, ...
                    {'vector', 'integer', 'positive', '<=', numel(obj.IsActive)}, obj.FileName, 'bufferIdx');
            end
            obj.IsActive(bufferIdx) = false;
            obj.PacketEndTimes(bufferIdx) = -1;
        end
    end

    methods(Access = private)
        function rxWaveform = combineWaveforms(obj, startTime, endTime, centerFrequency, receivedPackets, sampleRate)
            %combineWaveforms Return the combined waveform

            numPackets = numel(receivedPackets);
            % Initialize the waveform
            duration = endTime - startTime;
            nRxAnts = size(receivedPackets(1).Data, 2);
            waveformLength = round(duration * sampleRate);
            rxWaveform = complex(zeros(waveformLength, nRxAnts, obj.ResultantWaveformDataType));
            prevPacketSampleRate = Inf; % To reduce the calls to rat function
            for idx = 1:numPackets
                packet = receivedPackets(idx);

                if ~obj.DisableValidation
                    % Verify all the packets are from full PHY (Abstraction = false)
                    if packet.Abstraction
                        error(message('wirelessnetwork:interferenceBuffer:MethodNotApplicable'));
                    end

                    % Verify that the number of columns in packet.Data
                    % field must be same for all the packets
                    if nRxAnts ~= size(packet.Data, 2)
                        error(message('wirelessnetwork:interferenceBuffer:InvalidWaveformSize'));
                    end
                end

                % Calculate the number of overlapping samples
                overlapStartTime = max(startTime, packet.StartTime);
                overlapEndTime = min(endTime, packet.StartTime + packet.Duration);
                % Using ceil/floor results one extra/less sample. So, using
                % the round helps to consider an extra sample only if it
                % overlaps with signal of interest for more than half of the
                % sample period.
                numSOIOverlapSamples = round((overlapEndTime - overlapStartTime) * sampleRate);
                numInterfererOverlapSamples = round((overlapEndTime - overlapStartTime) * packet.SampleRate);
                if numInterfererOverlapSamples == 0 || numSOIOverlapSamples == 0
                    continue;
                end
                
                % Calculate the overlapping start and end index of
                % the resultant waveform time-domain samples
                soiStartIdx = round((overlapStartTime - startTime) * sampleRate) + 1;
                soiEndIdx = soiStartIdx + numSOIOverlapSamples - 1;
                % Overlapping end index should not exceed the resultant waveform length
                if soiEndIdx > waveformLength
                    numSOIOverlapSamples = numSOIOverlapSamples - (soiEndIdx - waveformLength);
                    soiEndIdx = waveformLength;
                end

                % Calculate the overlapping start and end index of
                % the interferer waveform time-domain samples
                iStartIdx = round((overlapStartTime - packet.StartTime) * packet.SampleRate) + 1;
                iEndIdx = iStartIdx + numInterfererOverlapSamples - 1;

                numPadding = 0;
                packetWaveformLength = size(packet.Data, 1);
                % Overlapping end index should not exceed the interfering waveform length
                if iEndIdx > packetWaveformLength
                    numPadding = iEndIdx - packetWaveformLength;
                    iEndIdx = packetWaveformLength;
                end
                if numSOIOverlapSamples ~= numInterfererOverlapSamples
                    waveform = packet.Data;
                    % Calculate the resampling factor and cache it
                    if packet.SampleRate ~= prevPacketSampleRate
                        newItem = true;
                        cacheSize = size(obj.ResamplingCache,1);
                        for rIdx=1:cacheSize
                            if obj.ResamplingCache(rIdx,1) == sampleRate && obj.ResamplingCache(rIdx,2) == packet.SampleRate
                                L = obj.ResamplingCache(rIdx,3); % L value
                                M = obj.ResamplingCache(rIdx,4); % M value
                                newItem = false;
                                break;
                            end
                        end
                        % Cache the L,M for the new sample rate combination
                        if newItem
                            [L, M] = rat(sampleRate/packet.SampleRate);
                            obj.ResamplingCache(cacheSize+1,:) = [sampleRate packet.SampleRate L M];
                        end
                        prevPacketSampleRate = packet.SampleRate;
                    end
                    % When number of rows in the input is 1, resample
                    % function returns row vector. So, resample each column
                    % separately
                    if numInterfererOverlapSamples == 1
                        col1 = resample(waveform(iStartIdx:iEndIdx, 1), L, M)';
                        interfererWaveform = [col1 zeros(numel(col1), nRxAnts-1)];
                        for colIdx=2:nRxAnts
                            interfererWaveform(:, colIdx) = resample(waveform(iStartIdx:iEndIdx, colIdx), L, M)';
                        end
                    else
                        interfererWaveform = resample(waveform(iStartIdx:iEndIdx, :), L, M);
                    end
                    if size(interfererWaveform, 1) < numSOIOverlapSamples
                        numSOIOverlapSamples = size(interfererWaveform, 1);
                        soiEndIdx = soiStartIdx + numSOIOverlapSamples - 1;
                    end
                    iStartIdx = 1;
                else
                    interfererWaveform = packet.Data;
                    soiEndIdx = soiEndIdx - numPadding;
                    numSOIOverlapSamples = numSOIOverlapSamples-numPadding;
                end
                iEndIdx = iStartIdx+numSOIOverlapSamples-1;
                % Shift the interfering waveform in frequency if the
                % center frequency does not match with required center
                % frequency
                frequencyOffset = (-centerFrequency + packet.CenterFrequency);
                if frequencyOffset ~= 0
                    t = ((0:numSOIOverlapSamples-1) / sampleRate)';
                    interfererWaveform(iStartIdx:iEndIdx,:) = interfererWaveform(iStartIdx:iEndIdx,:) .* exp(1i*2*pi*frequencyOffset*t);
                end

                % Combine the time-domain samples
                rxWaveform(soiStartIdx:soiEndIdx, 1:nRxAnts) = ...
                    rxWaveform(soiStartIdx:soiEndIdx, 1:nRxAnts) + ...
                    interfererWaveform(iStartIdx:iEndIdx,1:nRxAnts);
            end
        end

        function [overlappingPackets , numPackets, acprRequiredFlag] = getOverlappingPackets(obj, startTime, endTime, centerFrequency, bandwidth)
            %getOverlappingPackets Return overlapping packets, the count of overlapping
            %packets, and a flag which indicates all the overlapping
            %packets are of same center frequency and bandwidth or not

            % Find the active packets
            minTimeOverlapThreshold = obj.MinTimeOverlapThreshold;
            packetIndices = find(obj.IsActive & ((obj.PacketEndTimes - startTime) > minTimeOverlapThreshold));
            numPackets = 0;
            acprRequiredFlag = false;
            overlappingPackets = obj.EmptyPacketStruct;
            if ~isempty(packetIndices)
                numActivePackets = numel(packetIndices);
                packetIdxList = zeros(numActivePackets, 1);
                % Filter the overlapping packets based on InterferenceModeling value
                soiStartFrequency = centerFrequency - bandwidth/2;
                soiEndFrequency = centerFrequency + bandwidth/2;
                activePacketList = obj.PacketBuffer(packetIndices);
                for idx = 1:numActivePackets

                    % Get the packet
                    packet = activePacketList(idx);
                    pktCenterFreq = packet.CenterFrequency;
                    pktBW = packet.Bandwidth;

                    % Find the active packets between the given time period
                    if (min(endTime, packet.StartTime+packet.Duration) - max(startTime, packet.StartTime) > minTimeOverlapThreshold)
                        % Find the matching packets in frequency domain
                        % based on interference modeling
                        switch obj.InterferenceFidelity
                            case 0 % co-channel
                                if (pktCenterFreq==centerFrequency) && (pktBW==bandwidth) % Packet with same center frequency and bandwidth similar to SOI
                                    numPackets = numPackets + 1;
                                    packetIdxList(numPackets) = idx;
                                end
                            case 1 % overlapping-adjacent-channel
                                if min(soiEndFrequency, pktCenterFreq+pktBW*0.5) - max(soiStartFrequency, pktCenterFreq-pktBW*0.5) > 0 % Packet overlapping fully/partially in frequency with SOI
                                    numPackets = numPackets + 1;
                                    packetIdxList(numPackets) = idx;
                                end
                            case 2 % non-overlapping-adjacent-channel
                                if min(soiEndFrequency,  pktCenterFreq+pktBW*0.5) - max(soiStartFrequency, pktCenterFreq-pktBW*0.5) > -obj.MaxInterferenceOffset % Packet overlapping in frequency is until MaxInterferenceOffset frequency offset from SOI
                                    numPackets = numPackets + 1;
                                    packetIdxList(numPackets) = idx;
                                end
                        end
                        % Check whether all the packets are of different center frequency or
                        % bandwidth
                        if pktCenterFreq ~= centerFrequency || pktBW ~= bandwidth
                            acprRequiredFlag = true;
                        end
                    end
                end
                overlappingPackets = activePacketList(packetIdxList(1:numPackets));
            end
        end

        function removeObsoletePackets(obj, endTime)
            %removeObsoletePackets Remove the packets from the buffer which
            %have ended on or before the specified time

            expiredSignalIdx = obj.IsActive & (obj.PacketEndTimes <= endTime);
            if any(expiredSignalIdx)
                obj.IsActive(expiredSignalIdx) = false;
                obj.PacketEndTimes(expiredSignalIdx) = -1;
                obj.PacketBuffer(expiredSignalIdx) = obj.PacketStruct;
            end
        end

        function bufferIdx = autoResizePacketBuffer(obj, currentTime)
            %autoResizePacketBuffer Return the next inactive buffer index after resizing the packet buffer

            % Remove the obsolete packets
            obj.NextBufferCleanupTime = currentTime+obj.BufferCleanupCheckGap;
            removeObsoletePackets(obj, currentTime-obj.BufferCleanupTime);

             bufferIdx = find(~obj.IsActive, 1);
             if isempty(bufferIdx) % Increase the buffer size
                 bufferIdx = size(obj.IsActive, 1) + 1; % Next empty buffer index
                 stepSize = obj.BufferStepUpSize;
                 obj.IsActive = [obj.IsActive; false(stepSize, 1)];
                 obj.PacketEndTimes = [obj.PacketEndTimes; zeros(stepSize, 1)-1];
                 obj.PacketBuffer = [obj.PacketBuffer; repmat(obj.PacketStruct,stepSize,1)];
             end
        end

        function [centerFrequency, bandwidth, subchannelBandwidth] = validateInputs(obj, inputParam)
            %validateInputs Parse and validate the inputs

            centerFrequency = obj.CenterFrequency;
            bandwidth = obj.Bandwidth;

            isSpecifyingSubchannelBandwidth = false;
            for idx = 1:2:numel(inputParam)
                if strcmp(inputParam{idx}, "CenterFrequency")
                    centerFrequency = inputParam{idx+1};
                elseif strcmp(inputParam{idx}, "Bandwidth")
                    bandwidth = inputParam{idx+1};
                elseif strcmp(inputParam{idx}, "SubchannelBandwidth")
                    subchannelBandwidth = inputParam{idx+1};
                    isSpecifyingSubchannelBandwidth = true;
                else
                    error(message('wirelessnetwork:interferenceBuffer:UnrecognizedStringChoice', inputParam{idx}));
                end
            end

            if ~isSpecifyingSubchannelBandwidth
                % subchannelBandwidth is set to bandwidth by default
                subchannelBandwidth = bandwidth;
            end

            % Validate center frequency
            validateattributes(centerFrequency, {'numeric'}, ...
                {'scalar', 'real', 'nonnegative', 'finite'}, obj.FileName, 'CenterFrequency');

            % Validate bandwidth
            validateattributes(bandwidth, {'numeric'}, ...
                {'scalar', 'real', 'positive', 'finite'}, obj.FileName, 'Bandwidth');

            % Validate subchannelBandwidth
            validateattributes(subchannelBandwidth, {'numeric'}, ...
                {'scalar', 'real', 'positive', 'finite'}, obj.FileName, 'SubchannelBandwidth');

            % Validate the number of subchannels
            numSubchannels = bandwidth/subchannelBandwidth;
            if floor(numSubchannels) ~= numSubchannels
                error(message('wirelessnetwork:interferenceBuffer:InvalidNumSubchannels'));
            end
        end

        function updateInterferenceType(obj)
            %updateInterferenceType Update the InterferenceFidelity property based on
            %InterferenceModeling property

            switch obj.InterferenceModeling
                case "co-channel"
                    obj.InterferenceFidelity = 0;
                case "overlapping-adjacent-channel"
                    obj.InterferenceFidelity = 1;
                case "non-overlapping-adjacent-channel"
                    obj.InterferenceFidelity = 2;
            end
        end
    end
end

%Local function
function sampleRate = calculateSampleRate(centerFrequency, receivedPackets, actSampleRate)
%calculateSampleRate Return the desired sample rate for combining waveforms

inputSampleRates = [receivedPackets.SampleRate];
frequencyOffsets = (-centerFrequency + [receivedPackets.CenterFrequency]);
bandEdge = inputSampleRates./2 + abs(frequencyOffsets); % Edge of each frequency shifted band
sampleRate = 2 * max(bandEdge); % output sample rate
if sampleRate < actSampleRate % Determine the maximum sample rate
    sampleRate = actSampleRate;
end
end