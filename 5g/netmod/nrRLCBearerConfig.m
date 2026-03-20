classdef nrRLCBearerConfig < comm.internal.ConfigBase
%nrRLCBearerConfig Radio link control (RLC) bearer configuration parameters
%   CONFIG = nrRLCBearerConfig creates a default RLC bearer configuration
%   object, CONFIG.
%
%   CONFIG = nrRLCBearerConfig(Name=Value) creates an RLC bearer
%   configuration object, CONFIG, with the specified property Name set to
%   the specified Value. You can specify additional name-value arguments in
%   any order as (Name1=Value1, ...,NameN=ValueN). The nrRLCBearerConfig
%   object implements the information element (IE) RLC-BearerConfig, as
%   defined in TS 38.331 Section 6.3.2.
%
%   nrRLCBearerConfig properties:
%   LogicalChannelID    - Logical channel identifier
%   RLCEntityType       - Type of the RLC entity
%   SNFieldLength       - Number of bits in sequence number field of RLC
%                         entity
%   BufferSize          - RLC Transmitter buffer size in terms of number of
%                         packets
%   PollPDU             - Allowable number of acknowledged mode data (AMD)
%                         protocol data unit (PDU) transmissions before
%                         requesting the status PDU
%   PollByte            - Allowable number of service data unit (SDU) byte
%                         transmissions before requesting the status PDU
%   PollRetransmitTimer - Waiting time (in milliseconds) before
%                         retransmitting the status PDU request
%   MaxRetxThreshold    - Maximum number of retransmissions of an AMD PDU
%   ReassemblyTimer     - Waiting time (in milliseconds) before declaring
%                         the reassembly failure of SDUs in the reception
%                         buffer
%   StatusProhibitTimer - Waiting time (in milliseconds) before
%                         transmitting the status PDU following the
%                         previous status PDU transmission
%   LogicalChannelGroup - Logical channel group identifier
%   Priority            - Priority of the logical channel
%   PrioritizedBitRate  - Prioritized bit rate (in kilobytes per second) of
%                         the logical channel
%   BucketSizeDuration  - Bucket size duration (in milliseconds) of the
%                         logical channel
%
%   % Example 1:
%   %  Create an nrRLCBearerConfig object for a bidirectional RLC UM bearer
%   %  with reassembly timer of 100 milliseconds.
%
%   rlcBearerConfig = nrRLCBearerConfig(ReassemblyTimer=100);
%   disp(rlcBearerConfig)
%
%   % Example 2:
%   %  Create an nrRLCBearerConfig object for a unidirectional RLC UM bearer
%   %  in the UL direction.
%
%   rlcBearerConfig = nrRLCBearerConfig(RLCEntityType="UMUL");
%   disp(rlcBearerConfig)
%
%   % Example 3:
%   %  Create an nrRLCBearerConfig object for a unidirectional RLC UM bearer
%   %  in the DL direction.
%
%   rlcBearerConfig = nrRLCBearerConfig(RLCEntityType="UMDL");
%   disp(rlcBearerConfig)
%
%   % Example 4:
%   %  Create an nrRLCBearerConfig object for a bidirectional RLC UM bearer
%   %  with these parameters:
%   %  Logical channel ID: 5
%   %  Priority: 16
%   %  Logical channel group: 2
%   %  PrioritizedBitRate: 256 kilobytes per second
%   %  BucketSizeDuration: 300 milliseconds
%
%   rlcBearerConfig = ...
%   nrRLCBearerConfig(LogicalChannelID=5,LogicalChannelGroup=2,Priority=16,...
%                      PrioritizedBitRate=256,BucketSizeDuration=300);
%   disp(rlcBearerConfig)
%
%   % Example 5:
%   %  Create an nrRLCBearerConfig object for an RLC AM bearer with these
%   %  parameters:
%   %  Maximum retransmission threshold: 2
%   %  Poll PDU: 64
%   %  Poll byte: 25e3 bytes
%   %  Poll retransmit timer: 100 milliseconds
%   %  Status prohibit timer: 50 milliseconds
%
%   rlcBearerConfig = ...
%   nrRLCBearerConfig(RLCEntityType="AM",MaxRetxThreshold=2,PollPDU=64,PollByte=25e3,...
%                      PollRetransmitTimer=100,StatusProhibitTimer=50);
%   disp(rlcBearerConfig)
%
%   % Example 6:
%   %  Establish an RLC bearer between gNB and UE.
%
%   %  Check if the Communications Toolbox Wireless Network Simulation
%   %  Library support package is installed. If the support package is not
%   %  installed, MATLAB returns an error with a link to download and install
%   %  the support package.
%   wirelessnetworkSupportPackageCheck
%   %  Create a default gNB node
%   gNBNode = nrGNB;
%   %  Create a default UE node
%   ueNode = nrUE;
%   %  Create a default RLC bearer configuration object
%   rlcBearerConfig = nrRLCBearerConfig;
%   %  Establish an RLC bearer between the gNB and UE node
%   connectUE(gNBNode,ueNode,RLCBearerConfig=rlcBearerConfig)
%
%   See also nrGNB, nrUE.

%   Copyright 2022-2024 The MathWorks, Inc.

properties
    %LogicalChannelID Logical channel identifier
    %   Specify the value of LogicalChannelID as an integer in the range of [4,
    %   32]. For more information, see 3GPP TS 38.321 Table 6.2.1-1.  The
    %   default value is 4.
    LogicalChannelID (1, 1) {mustBeInteger, mustBeInRange(LogicalChannelID, 4, 32)} = 4

    %RLCEntityType Type of the RLC entity
    %   Specify the value of RLCEntityType as "UM", "UMDL", "UMUL", or "AM".
    %   The values "UM", "UMDL", "UMUL", and "AM" indicate the unacknowledged
    %   mode (UM) bidirectional entity, UM unidirectional downlink (DL) entity,
    %   UM unidirectional uplink (UL) entity, and acknowledged mode (AM)
    %   entity, respectively. The default value is "UM".
    RLCEntityType {mustBeTextScalar} = "UM"

    %BufferSize Maximum capacity of the RLC transmit buffer in terms of
    %number of packets
    %   Specify the value of BufferSize for an RLC entity as a positive
    %   integer. The default value is 400.
    BufferSize (1, 1) {mustBeInteger, mustBeGreaterThan(BufferSize, 0)} = 400

    %SNFieldLength Number of bits in sequence number field of RLC entity
    %   Specify the value of SNFieldLength as 6, 12, or 18. When the RLC entity
    %   type is "UM", "UMDL", or "UMUL", the sequence number field length can
    %   be 6 or 12 bits. On the other hand, the sequence number field length
    %   can be 12 or 18 bits for the RLC AM entity. The default value of
    %   SNFieldLength is 12.
    SNFieldLength (1, 1) {mustBeInteger} = 12

    %PollPDU Allowable number of acknowledged mode data (AMD) protocol data
    %unit (PDU) transmissions before requesting the status PDU
    %   Specify the value of PollPDU as 4, 8, 16, 32, 64, 128, 256, 512, 1024,
    %   2048, 4096, 6144, 8192, 12288, 16384,20480, 24576, 28672, 32768, 40960,
    %   49152, 57344, 65536, or Inf. To enable this property, set the
    %   RLCEntityType value to "AM". The default value is 32.
    PollPDU (1, 1) {mustBeNumeric} = 32

    %PollByte Allowable number of service data unit (SDU) byte transmissions
    %before requesting the status PDU
    %   Specify the value of PollByte as 1e3, 2e3, 5e3, 8e3, 10e3, 15e3, 25e3,
    %   50e3, 75e3, 100e3, 125e3, 250e3, 375e3, 500e3, 750e3, 1e6, 1.25e6,
    %   1.5e6, 2e6, 3e6, 4e6, 4.5e6, 5e6, 5.5e6, 6e6, 6.5e6, 7e6, 7.5e6, 8e6,
    %   9e6, 10e6, 11e6, 12e6, 13e6, 14e6, 15e6, 16e6, 17e6, 18e6, 20e6, 25e6,
    %   30e6, 40e6, or Inf. The units are in bytes. To enable this property,
    %   set the RLCEntityType value to "AM". The default value is Inf.
    PollByte (1, 1) {mustBeNumeric} = Inf

    %MaxRetxThreshold Maximum number of retransmissions of an AMD PDU
    %   Specify the value of MaxRetxThreshold as 1, 2, 3, 4, 6, 8, 16, or 32.
    %   To enable this property, set the RLCEntityType value to "AM". The
    %   default value is 4.
    MaxRetxThreshold (1, 1) {mustBeMember(MaxRetxThreshold, [1, 2, 3, 4, 6, 8, 16, 32])} = 4

    %PollRetransmitTimer Waiting time (in milliseconds) before retransmitting
    %the status PDU request
    %   Specify the value of PollRetransmitTimer as 5, 10, 15, 20, 25, 30, 35,
    %   40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115,
    %   120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180, 185,
    %   190, 195, 200, 205, 210, 215, 220, 225, 230, 235, 240, 245, 250, 300,
    %   350, 400, 450, 500, 800, 1000, 2000, or 4000. To enable this property,
    %   set the RLCEntityType value to "AM". The default value is 80.
    PollRetransmitTimer (1, 1) {mustBeMember(PollRetransmitTimer, ...
        [5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, ...
        85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, ...
        150, 155, 160, 165, 170, 175, 180, 185, 190, 195, 200, 205, 210, ...
        215, 220, 225, 230, 235, 240, 245, 250, 300, 350, 400, 450, 500, ...
        800, 1000, 2000, 4000])} = 80

    %ReassemblyTimer Waiting time (in milliseconds) before declaring the
    %reassembly failure of SDUs in the reception buffer
    %   Specify the value of ReassemblyTimer as 0, 5, 10, 15, 20, 25, 30, 35,
    %   40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 110, 120, 130,
    %   140, 150, 160, 170, 180, 190, or 200. The default value is 30.
    ReassemblyTimer (1, 1) {mustBeMember(ReassemblyTimer, ...
        [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, ...
        80, 85, 90, 95, 100, 110, 120, 130, 140, 150, 160, 170, 180, ...
        190, 200])} = 30

    %StatusProhibitTimer Waiting time (in milliseconds) before transmitting the
    %status PDU following the previous status PDU transmission
    %   Specify the value of StatusProhibitTimer as 0, 5, 10, 15, 20, 25, 30,
    %   35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115,
    %   120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180, 185,
    %   190, 195, 200, 205, 210, 215, 220, 225, 230, 235, 240, 245, 250, 300,
    %   350, 400, 450, 500, 800, 1000, 1200, 1600, 2000, or 2400. To enable
    %   this property, set the RLCEntityType value to "AM". The default value
    %   is 40.
    StatusProhibitTimer (1, 1) {mustBeMember(StatusProhibitTimer, ...
        [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, ...
        80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, ...
        145, 150, 155, 160, 165, 170, 175, 180, 185, 190, 195, 200, ...
        205, 210, 215, 220, 225, 230, 235, 240, 245, 250, 300, 350, ...
        400, 450, 500, 800, 1000, 1200, 1600, 2000, 2400])} = 40

    %LogicalChannelGroup Logical channel group identifier
    %   Specify the value of LogicalChannelGroup as an integer in the range of
    %   [1, 7]. For more information, see 3GPP TS 38.321 Table 6.2.1-1. The
    %   data radio bearers use the logical channel group IDs from 1. The
    %   default value is 1.
    LogicalChannelGroup (1, 1) {mustBeInteger, mustBeInRange(LogicalChannelGroup, 1, 7)} = 1

    %Priority Priority of the logical channel
    %   Specify the value of Priority as an integer in the range of [1, 16].
    %   The default value is 1.
    Priority (1, 1) {mustBeInteger, mustBeInRange(Priority, 1, 16)} = 1

    %PrioritizedBitRate Prioritized bit rate (in kilobytes per second) of the
    %logical channel
    %   Specify the value of PrioritizedBitRate as 0, 8, 16, 32, 64, 128, 256,
    %   512, 1024, 2048, 4096, 8192, 16384, 32768, 65536, or Inf. The default
    %   value is 8.
    PrioritizedBitRate (1, 1) {mustBeNumeric} = 8

    %BucketSizeDuration Bucket size duration (in milliseconds) of the logical
    %channel
    %   Specify the value of BucketSizeDuration as 5, 10, 20, 50, 100, 150,
    %   300, 500, or 1000. The default value is 1000.
    BucketSizeDuration (1, 1) {mustBeMember(BucketSizeDuration, ...
        [5, 10, 20, 50, 100, 150, 300, 500, 1000])} = 1000
end

properties (Constant, Hidden)
    % To allow tab completion with the values when dot indexing the object,
    % and to use in the validation of RLCEntityType
    RLCEntityType_Values = ["UM", "UMDL", "UMUL", "AM"];
end

methods
    function obj = nrRLCBearerConfig(varargin)
        names = varargin(1:2:end);
        % Search the presence of 'RLCEntityType' N-V argument to decide on the
        % valid SN field length values
        rlcEntityIndices = find(strcmp([names{:}], 'RLCEntityType'));
        if ~isempty(rlcEntityIndices)
            obj.RLCEntityType = varargin{2*rlcEntityIndices(end)};
        end

        % Search the presence of 'SNFieldLength' N-V argument to use the last given
        % SN field length name-value pair
        snFieldLengthIndices = find(strcmp([names{:}], 'SNFieldLength'));
        if ~isempty(snFieldLengthIndices)
            obj.SNFieldLength = varargin{2*snFieldLengthIndices(end)};
        end

        % Assign the values to remaining parameters
        for idx = 1:size(names,2)
            if any(idx == rlcEntityIndices) || any(idx == snFieldLengthIndices)
                continue;
            end
            obj.(char(names{idx})) = varargin{2*idx};
        end
    end

    function obj = set.SNFieldLength(obj, value)
        if obj.RLCEntityType == "AM"
            validValues = [12, 18];
            coder.internal.errorIf(~any(value == validValues), ...
            'nr5g:nrRLCBearerConfig:IncompatibleEntityTypeSNFieldLength', obj.RLCEntityType, "12 or 18");
        else
            validValues = [6, 12];
            coder.internal.errorIf(~any(value == validValues), ...
            'nr5g:nrRLCBearerConfig:IncompatibleEntityTypeSNFieldLength', obj.RLCEntityType, "6 or 12");
        end
        obj.SNFieldLength = value;
    end

    function obj = set.RLCEntityType(obj, value)
        val = validatestring(value, obj.RLCEntityType_Values,...
            'nrRLCBearerConfig', 'RLCEntityType');
        if val == "AM"
            validValues = [12, 18];
            coder.internal.errorIf(~any(obj.SNFieldLength == validValues), ...
            'nr5g:nrRLCBearerConfig:IncompatibleEntityTypeSNFieldLength', val, "12 or 18");
        else
            validValues = [6, 12];
            coder.internal.errorIf(~any(obj.SNFieldLength == validValues), ...
            'nr5g:nrRLCBearerConfig:IncompatibleEntityTypeSNFieldLength', val, "6 or 12");
        end
        obj.RLCEntityType = val;
    end

    function obj = set.PrioritizedBitRate(obj, val)
        validValues = [0, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536, Inf];
        coder.internal.errorIf(~any(val == validValues), ...
            'nr5g:nrRLCBearerConfig:InvalidPrioritizedBitRate');
        obj.PrioritizedBitRate = val;
    end

    function obj = set.PollPDU(obj, val)
        validValues = [4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 6144, 8192, 12288, 16384,20480, ...
            24576, 28672, 32768, 40960, 49152, 57344, 65536, Inf];
        coder.internal.errorIf(~any(val == validValues), ...
            'nr5g:nrRLCBearerConfig:InvalidPollPDU');
        obj.PollPDU = val;
    end

    function obj = set.PollByte(obj, val)
        validValues = [1, 2, 5, 8, 10, 15, 25, 50, 75, 100, 125, 250, 375, 500, 750, 1000, 1250, 1500, 2000, ...
            3000, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 9000, 10000, 11000, 12000, 13000, ...
            14000, 15000, 16000, 17000, 18000, 20000, 25000, 30000, 40000, Inf] * 1e3;
        coder.internal.errorIf(~any(val == validValues), ...
            'nr5g:nrRLCBearerConfig:InvalidPollByte');
        obj.PollByte = val;
    end
end

methods(Access = protected)
    function flag = isInactiveProperty(obj, prop)
        switch(prop)
            case {'PollPDU', 'PollByte', 'PollRetransmitTimer', 'MaxRetxThreshold', 'StatusProhibitTimer'}
                flag = obj.RLCEntityType ~= "AM";
            otherwise
                flag = false;
        end
    end
end
end