classdef nrPCAPWriter < comm.internal.ConfigBase & comm_sysmod.internal.pcapCommon
    %nrPCAPWriter Create a 5G NR PCAP or PCAPNG file writer object
    %
    %   NRPCAPW = nrPCAPWriter creates a 5G new radio (NR) packet capture
    %   (PCAP) or packet capture next generation (PCAPNG) file writer
    %   object, NRPCAPW, for writing the 5G NR MAC packets into a file
    %   with the .pcap or .pcapng extension, respectively. To write 5G NR
    %   MAC packets, no native link type is available for NR. The object
    %   writes the 5G NR MAC packets to the PCAP or PCAPNG file by
    %   encapsulating the packets into a pseudo protocol having a link
    %   type. Each MAC packet is prefixed with socket address link layer
    %   (SLL), internet protocol (IP), and user datagram protocol (UDP)
    %   headers followed by per packet information. Due to the
    %   encapsulation of packets, the object can write a MAC packet with a
    %   maximum size of 65482 bytes at a time. If the size of MAC packet is
    %   greater than 65482 bytes, the object truncates the remaining bytes.
    %
    %   NRPCAPW = nrPCAPWriter(Name1=Value1, ..., NameN=ValueN) creates a
    %   5G NR PCAP or PCAPNG file writer object, NRPCAPW, with properties
    %   specified by one or more name-value arguments. You can specify
    %   additional name-value arguments in any order as
    %   (Name1=Value1,...,NameN=ValueN). When you do not specify a property
    %   name and value, the object uses the default value.
    %
    %   NRPCAPW = nrPCAPWriter(PCAPWriter=PCAPW) creates a 5G NR PCAP or PCAPNG
    %   file writer object, NRPCAPW, by using the configuration specified in PCAPW.
    %   PCAPW is a <a href="matlab:help('pcapWriter')">pcapWriter</a> or <a href="matlab:help('pcapngWriter')">pcapngWriter</a> object.
    %
    %   nrPCAPWriter methods:
    %
    %   write         - Write 5G NR MAC packet into PCAP or PCAPNG format
    %
    %   nrPCAPWriter Name-Value pairs:
    %
    %   FileName      - File name, specified as a character row vector or
    %                   a string. The default file name is 'capture'.
    %   ByteOrder     - Byte order, specified as 'little-endian' or
    %                   'big-endian'. The default value is 'little-endian'.
    %   FileExtension - Extension of the PCAP or PCAPNG file, specified as
    %                   'pcap' or 'pcapng'. The default value is 'pcap'.
    %   FileComment   - Additional info given by the user as a comment for
    %                   the file, specified as a character row vector or a
    %                   string. The default value is an empty character
    %                   array. To enable this property, set the
    %                   'FileExtension' property to 'pcapng'.
    %   Interface     - Name of the device that captures packets, specified
    %                   as a character row vector or a string. The default
    %                   value is '5GNR'. To enable this property, set the
    %                   'FileExtension' property to 'pcapng'.
    %   PCAPWriter    - Object of type <a href="matlab:help('pcapWriter')">pcapWriter</a> or <a href="matlab:help('pcapngWriter')">pcapngWriter</a>.
    %                   When you set this property, NRPCAPW derives the
    %                   FileName, FileExtension, FileComment, and ByteOrder
    %                   properties in accordance with the PCAPW input.
    %
    %   % Example 1:
    %   % Write a 5G NR MAC packet to the PCAP format file.
    %
    %   % Create a nrPCAPWriter object with file name as nrPCAPExample1.pcap
    %       nrpcapw = nrPCAPWriter(FileName='nrPCAPExample1', FileExtension='pcap');
    %
    %   % Create a 5G NR MAC packet
    %       macPDU = [6; 68; 64; 0; ones(66,1); 62; 4; 7; 74; 96; 102];
    %
    %   % Set the timestamp for the packet
    %       timestamp = 1000000; % In microseconds
    %
    %   % Create the packet information structure for the MAC packet by using the
    %   % constants defined in nrpcapw object
    %       packetInfo = struct();
    %       packetInfo.RadioType = nrpcapw.RadioFDD;
    %       packetInfo.LinkDir = nrpcapw.Uplink;
    %       packetInfo.RNTIType = nrpcapw.CellRNTI;
    %
    %   % Write the 5G NR MAC packet to the PCAP format file
    %       write(nrpcapw, macPDU, timestamp, PacketInfo=packetInfo);
    %
    %   % Example 2:
    %   % Write the system information block 1 (SIB1) packet to the PCAPNG format file with comment.
    %
    %   % Create a nrPCAPWriter object with file name as nrPCAPExample2.pcapng
    %       nrpcapw = nrPCAPWriter(FileName='nrPCAPExample2', FileExtension='pcapng', ...
    %                     FileComment='SIB1 Packet');
    %
    %   % Create a SIB1 packet
    %       sib1Octets = [64; 0; 0; 36; 104; 21; 0; 10; 156; 1; 15; zeros(13,1)];
    %
    %   % Set the timestamp for the packet
    %       timestamp = 2000000; % In microseconds
    %
    %   % Create the packet information structure required for SIB1 packet
    %       packetInfo = struct();
    %       packetInfo.RadioType = nrpcapw.RadioFDD;
    %       packetInfo.LinkDir = nrpcapw.Downlink;
    %       packetInfo.RNTIType = nrpcapw.SystemInfoRNTI;
    %
    %   % Write the SIB1 packet to the PCAPNG format file
    %       write(nrpcapw, sib1Octets, timestamp, PacketInfo=packetInfo);
    %
    %   % Example 3:
    %   % Write a 5G NR MAC packet to the PCAPNG format file, specifying the file
    %   % comment, packet comment, time stamp, and per packet information (radio type,
    %   % link direction, radio network temporary identifier (RNTI) type,
    %   % RNTI, user equipment identifier (UEID), and system frame number).
    %
    %   % Create a nrPCAPWriter object with file name as nrPCAPExample3.pcapng
    %       nrpcapw = nrPCAPWriter(FileName='nrPCAPExample3', ...
    %       FileExtension='pcapng', FileComment='This is a sample file');
    %
    %  % Create a 5G NR MAC packet which contains short truncated buffer status report (BSR)
    %       macPDU = [59; 205];
    %
    %   % Set the timestamp for the packet
    %       timestamp = 100000; % In microseconds
    %
    %   % Create the packet information structure for the MAC packet
    %       packetInfo = struct();
    %       packetInfo.RadioType = nrpcapw.RadioFDD;
    %       packetInfo.LinkDir = nrpcapw.Uplink;
    %       packetInfo.RNTIType = nrpcapw.CellRNTI;
    %       packetInfo.RNTI = 15;
    %       packetInfo.UEID = 1022;
    %       packetInfo.SystemFrameNumber = 10;
    %
    %   % Write the 5G NR MAC packet to the PCAPNG file
    %       write(nrpcapw, macPDU, timestamp, PacketInfo=packetInfo, ...
    %           PacketComment='This is a NR MAC BSR');
    %
    %   See also pcapWriter, pcapngWriter

    %   Copyright 2021-2024 The MathWorks, Inc.

    %#codegen

    properties(Dependent, SetAccess = private)
        %FileName Name of the PCAP or PCAPNG file
        %   Specify file name as a character row vector or a string. The
        %   default file name is 'capture'.
        FileName

        %ByteOrder Byte order type
        %   Specify the byte order as 'little-endian' or 'big-endian'. The
        %   default value is 'little-endian'.
        ByteOrder

        %FileComment Comment for the file
        %   Specify any additional comment for the file as a character row
        %   vector or a string. To enable this property, set the
        %   'FileExtension' property to 'pcapng'. The default value is an
        %   empty character array.
        FileComment
    end

    properties(GetAccess = public, SetAccess = private)
        %FileExtension Extension of the PCAP or PCAPNG file
        %   Specify the file extension as 'pcap' or 'pcapng'. The default
        %   value is 'pcap'.
        FileExtension = 'pcap'

        %Interface Name of the device used to capture data
        %   Specify interface as a character vector or a string in UTF-8
        %   format. To enable this property, set the 'FileExtension'
        %   property to 'pcapng'. The default value is '5GNR'.
        Interface = '5GNR'

        %PCAPWriter Packet writer object
        %   Specify the packet write object as <a href="matlab:help('pcapWriter')">pcapWriter</a> or <a href="matlab:help('pcapngWriter')">pcapngWriter</a>.
        %   If you enable this property, the object retrieves the values of
        %   FileName, FileExtension, FileComment, and ByteOrder properties
        %   from the object specified in the PCAPWriter.
        PCAPWriter
    end

    properties(Access = private)
        %InterfaceID Interface identifier
        %   Unique identifier assigned by PCAP or PCAPNG writer object for
        %   the interface
        InterfaceID = 0

        %IsPCAPNG PCAPNG file format flag
        %   Set this property to true to indicate that the file format is
        %   PCAPNG. The default value is false.
        IsPCAPNG(1, 1) logical = false;

        %PCAPPacketWriter PCAP packet writer object
        %   pcapWriter handle class object
        PCAPPacketWriter

        %PCAPNGPacketWriter PCAPNG packet writer object
        %   pcapngWriter handle class object
        PCAPNGPacketWriter
    end

    properties(Hidden)
        %DisableValidation Disable the validation for input arguments of write method
        %   Specify this property as a scalar logical. When true,
        %   validation is not performed on the input arguments and the
        %   packet is expected to be octets in decimal format with size
        %   less than or equal to 65482 bytes.
        DisableValidation(1, 1) logical = false
    end

    properties(Constant)
        % Choices for the mandatory field values required for the NR MAC signature

        %RadioFDD Frequency Division Duplex Radio type
        %   It is used for representing duplexing mode as frequency division
        %   duplexing (FDD)
        RadioFDD = 1;

        %RadioTDD Time Division Duplex Radio type
        %   It is used for representing duplexing mode as time division
        %   duplexing (TDD)
        RadioTDD = 2;

        %Uplink Direction is uplink
        %   It is used for representing uplink MAC packets
        Uplink = 0;

        %Downlink Direction is downlink
        %   It is used for representing downlink MAC packets
        Downlink = 1;

        %NoRNTI No RNTI
        %   It is used for broadcast of system information in broadcast
        %   control channel (BCCH) over broadcast channel (BCH).
        NoRNTI = 0;

        %PagingRNTI Paging RNTI
        %   It is used by the UEs for the reception of paging. It is not
        %   allocated to any UE explicitly.
        PagingRNTI = 1;

        %RandomAccessRNTI Random Access RNTI
        %   It is used during Random Access procedure.
        RandomAccessRNTI = 2;

        %CellRNTI Cell RNTI
        %   It is used for identifying radio resource control (RRC)
        %   connection and scheduling which is dedicated to a particular UE.
        %   The gNB assigns unique Cell-RNTI to the UEs
        CellRNTI = 3;

        %SystemInfoRNTI System Information RNTI
        %   It is used for broadcast of system information in BCCH over
        %   downlink shared channel (DL-SCH).
        SystemInfoRNTI = 4;

        %ConfiguredSchedulingRNTI Configured Scheduling RNTI
        %   It is used for semi-persistent scheduling (SPS) in the downlink
        %   and configured grant in the uplink.
        ConfiguredSchedulingRNTI = 5;
    end

    properties(Constant, Hidden)
        %LinkType Unique identifier for SLL packet used for encapsulation of NR MAC packets
        LinkType = 113;

        %StartString Tag which indicates the beginning of NR MAC signature
        StartString = [109;97;99;45;110;114]; % mac-nr

        %FileExtensionValues Values which the 'FileExtension' property can take
        FileExtension_Values = {'pcap', 'pcapng'};

        %MAXPDUSize Maximum MAC PDU size (in bytes) which can be captured
        MAXPDUSize = 65482;
    end

    methods (Access = protected)
        function flag = isInactiveProperty(obj, prop)
            flag = false;
            if strcmp(prop,'FileComment') || strcmp(prop,'Interface')
                flag = strcmp(obj.FileExtension, 'pcap');
            end
        end

        function groups = getPropertyGroups(obj)
            groups = getPropertyGroups@comm.internal.ConfigBase(obj);
            groups = groups(2); % Display read-only properties
        end
    end

    methods(Access = private)
        function setFileExtension(obj, value)
            propName = 'FileExtension';
            value = validateEnumProperties(obj, propName, value);
            obj.(propName) = value;
        end

        function setInterface(obj, value)
            validateattributes(value, {'char', 'string'}, {'row'}, ...
                mfilename, 'Interface')
            obj.Interface = char(value);
        end

        function setPCAPWriter(obj, value)
            validateattributes(value, {'pcapngWriter', 'pcapWriter'}, ...
                {'scalar'}, mfilename, 'PCAPWriter')
            obj.PCAPWriter = value;
        end

        function packetInfoHeader = extractPacketInfo(obj, packetInfo)
            %extractPacketInfo Return packet information to be attached to the MAC packet

            if isfield(packetInfo, 'RadioType')
                radioType = packetInfo.RadioType;
            else
                radioType = obj.RadioFDD;
            end
            if isfield(packetInfo, 'LinkDir')
                direction = packetInfo.LinkDir;
            else
                direction = obj.Uplink;
            end
            if isfield(packetInfo, 'RNTIType')
                rntiType = packetInfo.RNTIType;
            else
                rntiType = obj.CellRNTI;
            end
            if isfield(packetInfo, 'RNTI')
                rnti = packetInfo.RNTI;
            else
                rnti = [];
            end
            if isfield(packetInfo, 'UEID')
                ueId = packetInfo.UEID;
            else
                ueId = [];
            end
            if isfield(packetInfo, 'PHRType2OtherCell')
                phrType2OtherCell = packetInfo.PHRType2OtherCell;
            else
                phrType2OtherCell = [];
            end
            if isfield(packetInfo, 'HARQID')
                harqId = packetInfo.HARQID;
            else
                harqId = [];
            end
            if isfield(packetInfo, 'SystemFrameNumber')
                systemFrameNumber = packetInfo.SystemFrameNumber;
            else
                systemFrameNumber = [];
            end
            if isfield(packetInfo, 'SlotNumber')
                slotNumber = packetInfo.SlotNumber;
            else
                slotNumber = [];
            end

            if ~obj.DisableValidation
                % Validate RadioType
                validateattributes(radioType, {'numeric'}, {'integer', 'scalar', '>=', 1, '<=', 2}, mfilename, 'packetInfo.RadioType');

                % Validate LinkDir (link direction)
                validateattributes(direction, {'numeric'}, {'binary', 'nonempty'}, mfilename, 'packetInfo.LinkDir');

                % Validate RNTIType
                validateattributes(rntiType, {'numeric'}, {'integer', 'scalar', '>=', 0, '<=', 5}, mfilename, 'packetInfo.RNTIType');

                % Validate RNTI
                if ~isempty(rnti)
                    validateattributes(rnti, {'numeric'}, {'integer', 'scalar', '>=', 0, '<=', 65535}, mfilename, 'packetInfo.RNTI');
                end

                % Validate UEID
                if ~isempty(ueId)
                    validateattributes(ueId, {'numeric'}, {'integer', 'scalar', '>=', 0, '<=', 65535}, mfilename, 'packetInfo.UEID');
                end

                % Validate PHRType2OtherCell
                if ~isempty(phrType2OtherCell)
                    validateattributes(phrType2OtherCell, {'logical', 'numeric'}, {'binary'}, mfilename, 'packetInfo.PHRType2OtherCell');
                end

                % Validate HARQID
                if ~isempty(harqId)
                    validateattributes(harqId, {'numeric'}, {'integer', 'scalar', '>=', 0, '<=', 15}, mfilename, 'packetInfo.HARQID');
                end

                % Validate SystemFrameNumber
                if ~isempty(systemFrameNumber)
                    validateattributes(systemFrameNumber, {'numeric'}, {'integer', 'scalar', '>=', 0, '<=', 1023}, mfilename, 'packetInfo.SystemFrameNumber');
                end

                % Validate SlotNumber
                if ~isempty(slotNumber)
                    validateattributes(slotNumber, {'numeric'}, {'integer', 'scalar', '>=', 0, '<=', 159}, mfilename, 'packetInfo.SlotNumber');
                end
            end

            % Create the packet information header to be attached to the MAC packet
            packetInfoHeader = createPacketInfoHeader(obj, radioType, direction, ...
                rntiType, rnti, ueId, phrType2OtherCell, ...
                harqId, systemFrameNumber, slotNumber);
        end

        function packetInfoHeader = createPacketInfoHeader(obj, radioType, direction, ...
                rntiType, rnti, ueId, phrType2OtherCell, ...
                harqId, systemFrameNumber, slotNumber)
            %createPacketInfoHeader Return packet information to be attached to the MAC packet

            % Initialize the packet information
            rntiData = [];
            ueIdData = [];
            phrType2OtherCellData = [];
            harqIdData = [];
            frameData = [];

            % Check if 'rnti' is set
            if ~isempty(rnti)
                rntiTag = 2; % RNTI tag value
                % 2-bytes followed by 'rntiTag' form the RNTI value
                rntiData = [rntiTag; floor(rnti / 256); mod(rnti, 256)];
            end

            % Check if 'ueId' is set
            if ~isempty(ueId)
                ueIdTag = 3; % UEID tag value
                % 2-bytes followed by 'ueIdTag' form the UEID value
                ueIdData = [ueIdTag; floor(ueId / 256); mod(ueId, 256)];
            end

            % Check if 'phrType2OtherCell' is set
            if ~isempty(phrType2OtherCell)
                phrType2OtherCellTag = 5; % PHRType2OtherCell tag value
                % 1-byte followed by 'phrType2OtherCellTag' forms the PHRType2OtherCell value
                phrType2OtherCellData = [phrType2OtherCellTag; phrType2OtherCell];
            end

            % Check if 'harqId' is set
            if ~isempty(harqId)
                harqIdTag = 6; % HARQID tag value
                % 1-byte followed by 'harqIdTag' forms the HARQID value
                harqIdData = [harqIdTag; harqId];
            end

            % Check if at least one of 'systemFrameNumber' or 'slotNumber' is set
            if ~isempty(systemFrameNumber)||~isempty(slotNumber)
                if systemFrameNumber
                    systemFrameNumBytes = [floor(systemFrameNumber / 256); mod(systemFrameNumber, 256)];
                else
                    systemFrameNumBytes = [0;0];
                end
                if slotNumber
                    slotNumBytes = [floor(slotNumber / 256); mod(slotNumber, 256)];
                else
                    slotNumBytes = [0;0];
                end
                frameSlotTag = 7; % Frame and slot information tag value
                % 2-bytes followed by 'frameSlotTag' form the system frame number
                % and the next 2-bytes form the slot number
                frameData = [frameSlotTag; systemFrameNumBytes; slotNumBytes];
            end

            % Create packet information header
            packetInfoHeader = [obj.StartString; radioType; direction; ...
                rntiType; rntiData; ueIdData; phrType2OtherCellData; harqIdData; frameData];
        end
    end

    methods
        function obj = nrPCAPWriter(varargin)
            %nrPCAPWriter Create a NR PCAP packet writer object

            % Name-value pair check
            coder.internal.errorIf(mod(nargin, 2) == true, ...
                'MATLAB:system:invalidPVPairs');

            % Initialize packetWriterFlag to false to indicate
            % 'PacketWriter' name-value pair is not given as input.
            packetWriterFlag = false;

            % Dependency check
            nvPairsFlag = false;

            % File name for dummy pcapWriter or pcapngWriter for codegen,
            % which will be used to create the object. No file will be
            % created using this dummy file name.
            dummyFileName = 'sample5GNRCapture';

            % Initialize to default values
            fileName = 'capture';
            byteOrder = 'little-endian';
            fileComment = blanks(0);

            if isempty(coder.target) % MATLAB path
                % Apply name-value pairs
                for idx = 1:2:nargin

                    name = validatestring(varargin{idx}, {'FileName', ...
                        'ByteOrder', 'FileComment', 'FileExtension', ...
                        'Interface', 'PCAPWriter'}, ...
                        mfilename);

                    switch(name)
                        case 'FileName'
                            fileName = varargin{idx+1};
                            nvPairsFlag = true;
                        case 'FileComment'
                            fileComment = varargin{idx+1};
                            nvPairsFlag = true;
                        case 'ByteOrder'
                            byteOrder = varargin{idx+1};
                            nvPairsFlag = true;
                        case 'FileExtension'
                            setFileExtension(obj, varargin{idx+1});
                            nvPairsFlag = true;
                        case 'Interface'
                            setInterface(obj, varargin{idx+1});
                        otherwise % PCAPWriter
                            setPCAPWriter(obj, varargin{idx+1});
                            packetWriterFlag = true;
                    end

                    coder.internal.errorIf(packetWriterFlag && nvPairsFlag, ...
                        'nr5g:nrPCAPWriter:InvalidParameters');
                end
            else %Codegen path
                nvPairs = struct('FileName', uint32(0), ...
                    'FileComment', uint32(0), ...
                    'ByteOrder', uint32(0), ...
                    'FileExtension', uint32(0), ...
                    'Interface', uint32(0), ...
                    'PCAPWriter', uint32(0));

                % Select parsing options
                popts = struct('PartialMatching', true, 'CaseSensitivity', ...
                    false);

                % Parse inputs
                pStruct = coder.internal.parseParameterInputs(nvPairs, ...
                    popts, varargin{:});

                if pStruct.PCAPWriter
                    packetWriterFlag = true;
                end

                coder.internal.errorIf(packetWriterFlag && ...
                    (pStruct.FileName || ...
                    pStruct.FileExtension || ...
                    pStruct.ByteOrder || ...
                    pStruct.FileComment), 'nr5g:nrPCAPWriter:InvalidParameters');

                % Get values for the N-V pair or set defaults for the
                % optional arguments
                byteOrder = coder.internal.getParameterValue(pStruct.ByteOrder, ...
                    coder.const('little-endian'), varargin{:});

                fileName = coder.internal.getParameterValue(pStruct.FileName, ...
                    'capture', varargin{:});

                fileComment = coder.internal.getParameterValue(pStruct.FileComment, ...
                    blanks(0), varargin{:});

                setFileExtension(obj, coder.internal.getParameterValue(pStruct.FileExtension, ....
                    coder.const('pcap'),varargin{:}));

                setInterface(obj, coder.internal.getParameterValue(pStruct.Interface, ...
                    coder.const('pcap'), varargin{:}));

                defaultVal = pcapWriter('FileName', dummyFileName);
                setPCAPWriter(obj, coder.internal.getParameterValue(pStruct.PCAPWriter, ...
                    defaultVal, varargin{:}));
            end

            % File name for pcap and pcapng files
            if (~packetWriterFlag && strcmp(obj.FileExtension, 'pcap')) || ...
                    (packetWriterFlag && isa(obj.PCAPWriter, 'pcapWriter'))
                pcapFileName = fileName;
                pcapngFileName = dummyFileName;
            else
                pcapFileName = dummyFileName;
                pcapngFileName = fileName;
                obj.IsPCAPNG = true;
            end

            % Check if 'PCAPWriter' object is passed as a name-value pair
            if packetWriterFlag
                if isa(obj.PCAPWriter, 'pcapWriter')
                    obj.FileExtension = 'pcap';
                    obj.PCAPPacketWriter = obj.PCAPWriter;

                    % Initialize for codegen support
                    obj.PCAPNGPacketWriter = pcapngWriter('FileName', pcapngFileName,...
                        'ByteOrder', byteOrder, 'FileComment', fileComment);

                    coder.internal.errorIf(obj.PCAPPacketWriter.GlobalHeaderPresent, ...
                        'nr5g:nrPCAPWriter:MultipleInterfacesNotAccepted');
                else
                    obj.FileExtension = 'pcapng';
                    obj.PCAPNGPacketWriter = obj.PCAPWriter;

                    % Initialize for codegen support
                    obj.PCAPPacketWriter = pcapWriter('FileName', pcapFileName,...
                        'ByteOrder', byteOrder);
                end
            else
                % Initialize for codegen support
                obj.PCAPPacketWriter = pcapWriter('FileName', pcapFileName, ...
                    'ByteOrder', byteOrder);
                obj.PCAPNGPacketWriter = pcapngWriter('FileName', pcapngFileName, ...
                    'ByteOrder', byteOrder, 'FileComment', fileComment);
                if strcmp(obj.FileExtension, 'pcap')
                    if ~isempty(fileComment)
                        coder.internal.warning('nr5g:nrPCAPWriter:IgnoreFileComment');
                    end
                end
            end

            if obj.IsPCAPNG
                % Write the interface description block
                obj.InterfaceID = writeInterfaceDescriptionBlock(obj.PCAPNGPacketWriter, obj.LinkType, ...
                    obj.Interface);
            else
                % Write the Global header block
                writeGlobalHeader(obj.PCAPPacketWriter, obj.LinkType);
            end
        end

        function value = get.FileName(obj)
            if obj.IsPCAPNG
                value = obj.PCAPNGPacketWriter.FileName;
            else
                value = obj.PCAPPacketWriter.FileName;
            end
        end

        function value = get.FileComment(obj)
            if obj.IsPCAPNG
                value = obj.PCAPNGPacketWriter.FileComment;
            else
                value = blanks(0);
            end
        end

        function value = get.ByteOrder(obj)
            if obj.IsPCAPNG
                value = obj.PCAPNGPacketWriter.ByteOrder;
            else
                value = obj.PCAPPacketWriter.ByteOrder;
            end
        end

        function set.DisableValidation(obj, value)
            obj.DisableValidation = value;
            if obj.IsPCAPNG %#ok<MCSUP>
                obj.PCAPNGPacketWriter.DisableValidation = value; %#ok<MCSUP>
            else
                obj.PCAPPacketWriter.DisableValidation = value; %#ok<MCSUP>
            end
        end

        function write(obj, packet, timestamp, varargin)
            %WRITE Write 5G NR MAC packet to a file with the .pcap or .pcapng extension
            %
            %   WRITE(OBJ, PACKET, TIMESTAMP) writes a 5G NR MAC packet
            %   into a file with .pcap or .pcapng extension
            %
            %   PACKET is the 5G NR MAC packet specified as one of these
            %   types:
            %    - A binary vector representing bits
            %    - A character vector representing octets in hexadecimal
            %      format
            %    - A string scalar representing octets in hexadecimal
            %      format
            %    - A numeric vector, where each element is in the range
            %      [0, 255], representing octets in decimal format
            %    - An n-by-2 character array, where each row represents
            %      an octet in hexadecimal format
            %
            %   TIMESTAMP is specified as a scalar integer greater than or
            %   equal to 0. Timestamp is the packet arrival time in
            %   microseconds since 1/1/1970.
            %
            %   WRITE(OBJ, PACKET, TIMESTAMP, Name=Value) specifies
            %   additional name-value arguments described below. When a
            %   name-value argument is not specified, the object function
            %   uses the default value.
            %
            %   'PacketInfo' contains information about the packet. If
            %   PacketInfo is not present, the object function adds the
            %   default packet information to the MAC packet. The default
            %   packet information includes the default values of
            %   RadioType, LinkDir, and RNTIType fields. PacketInfo is a
            %   structure which contains these fields. All the fields in
            %   the structure are case-sensitive.
            %       RadioType         - Mode of duplex, specified as one of
            %                           these values. 
            %                           1: RadioFDD
            %                           2: RadioTDD
            %                           The default value is 1 (RadioFDD)
            %
            %       LinkDir           - Direction of the link, specified as one of
            %                           these values. 
            %                           0: Uplink
            %                           1: Downlink
            %                           The default value is 0 (Uplink)
            %
            %       RNTIType          - Type of RNTI, specified as one of
            %                           these values.
            %                           0: No RNTI
            %                           1: Paging RNTI
            %                           2: Random access RNTI
            %                           3: Cell RNTI
            %                           4: System information RNTI
            %                           5: Configured scheduling RNTI
            %                           The default value is 3 (Cell RNTI)
            %
            %       RNTI              - Radio network temporary identifier,
            %                           specified as a 2-byte value (in decimal)
            %                           in the range [0, 65535].
            %
            %       UEID              - User equipment identifier, specified
            %                           as a 2-byte value (in decimal)
            %                           in the range [0, 65535].
            %
            %       PHRType2OtherCell - Binary value which decides the
            %                           presence of type 2 power headroom
            %                           field for special cell in case of
            %                           multiple entry power headroom report
            %                           MAC control element.
            %
            %       HARQID            - Hybrid automatic repeat request
            %                           process identifier, specified as a
            %                           1-byte value (in decimal) in the
            %                           range [0, 15].
            %
            %       SystemFrameNumber - System frame number, specified in
            %                           the range [0, 1023].
            %
            %       SlotNumber        - Slot number, specified in
            %                           the range [0, 159]. This value
            %                           identifies the slot in the 10 ms frame.
            %
            %   'PacketComment' is a comment to a packet, specified as a
            %   character vector or a string scalar. If you specify the
            %   'FileExtension' property as 'pcap', the object function
            %   ignores the packet comment. The default value is an empty
            %   character array.
            %
            %   'PacketFormat' specifies the format of an input data packet
            %   as 'bits' or 'octets'. If you specify this value as
            %   'octets', the packet can be a numeric vector representing
            %   octets in decimal format. Alternatively, this value can be a
            %   character array or string scalar representing octets in
            %   hexadecimal format. If you specify this value as 'bits',
            %   packet is a binary vector. In this vector, the object
            %   function assumes that every nth bit (n is a multiple of 8)
            %   is the right most significant bit (msb). The default value
            %   is 'octets'.

            % Check whether the validation is enabled
            if ~obj.DisableValidation
                narginchk(3, 9);

                % Name-value pair check
                coder.internal.errorIf(mod(numel(varargin), 2) == true, ...
                    'MATLAB:system:invalidPVPairs');
            end

            if isempty(coder.target) % MATLAB path
                % Initialize with default values
                packetInfo = [];
                packetFormat = 'octets';
                packetComment = '';

                % Apply name-value pairs
                for idx = 1:2:numel(varargin)

                    pIdx = find(strncmpi(varargin{idx}, {'PacketInfo', 'PacketFormat', 'PacketComment'}, strlength(varargin{idx})), 2);
                    if ~isempty(pIdx) && numel(pIdx) == 1
                        switch(pIdx)
                            case 1 % PacketInfo
                                packetInfo = varargin{idx+1};
                            case 2 % PacketFormat
                                packetFormat = varargin{idx+1};
                            otherwise % PacketComment
                                packetComment = varargin{idx+1};
                        end
                    else
                        coder.internal.errorIf(true,'nr5g:nrPCAPWriter:UnrecognizedStringChoice', varargin{idx});
                    end
                end
            else %Codegen path
                nvPairs = {'PacketInfo', 'PacketFormat', 'PacketComment'};

                % Select parsing options
                popts = struct('PartialMatching', true, ...
                    'CaseSensitivity', false);

                % Parse inputs
                pStruct = coder.internal.parseParameterInputs(nvPairs, ...
                    popts, varargin{:});

                % Get values for the N-V pair or set defaults for the optional arguments
                packetInfo = coder.internal.getParameterValue(pStruct.PacketInfo, ...
                    [], varargin{:});

                packetFormat = coder.internal.getParameterValue(pStruct.PacketFormat, ...
                    coder.const('octets'), varargin{:});

                packetComment = coder.internal.getParameterValue(pStruct.PacketComment, ...
                    blanks(0), varargin{:});
            end

            % Check whether the validation is enabled
            if ~obj.DisableValidation
                if ~isempty(packetInfo)
                    % Validate the fields in the 'PacketInfo' structure
                    validFlds = {'RadioType', 'LinkDir', 'RNTIType', 'RNTI', ...
                        'UEID', 'PHRType2OtherCell', 'HARQID', 'SystemFrameNumber', 'SlotNumber'};
                    fields = fieldnames(packetInfo);
                    for m = 1:numel(fields)
                        thisFld = fields{m};
                        if ~any(strcmp(thisFld,validFlds))
                            coder.internal.errorIf(true,'nr5g:nrPCAPWriter:InvalidField', thisFld);
                        end
                    end
                end

                % Validate packet and return octets in decimal format
                packetData = validatePayloadFormat(obj, packet, ...
                    validatestring(packetFormat, {'bits', 'octets'}, mfilename, 'PacketFormat'));
                % Truncate the packet if it exceeds the maximum allowed size (in bytes)
                if numel(packetData) > obj.MAXPDUSize
                    coder.internal.warning('nr5g:nrPCAPWriter:TruncateMACPDUBytes');
                    packetData = packetData(1:obj.MAXPDUSize);
                end

                % Validate timestamp
                validateattributes(timestamp, {'numeric'}, ...
                    {'scalar','integer','nonnegative'}, mfilename, 'timestamp');

                % Validate PacketComment
                if ~isempty(packetComment)
                    validateattributes(packetComment, {'char', 'string'}, {'row'}, ...
                        mfilename, 'PacketComment');
                end
            else
                packetData = packet;
            end

            % Create packet information header
            if isempty(packetInfo)
                packetInfoHeader = [obj.StartString; obj.RadioFDD; obj.Uplink; obj.CellRNTI];
            else
                packetInfoHeader = extractPacketInfo(obj, packetInfo);
            end

            % Create encapsulation header
            payloadLength = size(packetInfoHeader,1) + size(packetData,1) + 1; % 1 byte for payload tag
            encapHeader = encapsulationHeader(payloadLength);
            % Add encapsulation header, packet information to the packet
            payloadTag = 1; % Indicates that the bytes following this tag contains the NR MAC packet
            packet = [encapHeader; packetInfoHeader; payloadTag; packetData];

            if obj.IsPCAPNG
                % Write packet into PCAPNG format file
                if isempty(packetComment)
                    write(obj.PCAPNGPacketWriter, packet, timestamp, obj.InterfaceID);
                else
                    write(obj.PCAPNGPacketWriter, packet, timestamp, obj.InterfaceID, 'PacketComment', packetComment);
                end
            else
                % Write packet into PCAP format file
                write(obj.PCAPPacketWriter, packet, timestamp);
            end
        end
    end
end

function encapsulationHeader = encapsulationHeader(packetLength)
%encapsulationHeader Return the encapsulation headers (SLL header + IP header + UDP header)

% Construct the UDP header
udpHeader = [163;76; ...% Source port number (41804)
    39;15; ...% Destination port number (9999)
    floor((8+packetLength)/256); mod(8+packetLength, 256); ...% Length of header and packet. Length of header is 8 bytes
    0;0]; % Checksum

% Calculate the UDP packet length
udpPacketLength = size(udpHeader,1) + packetLength;

% Construct the IP header
ipHeader = [69; ...% Version of IP protocol and Priority/Traffic Class
    0; ... % Type of Service
    floor((20+udpPacketLength)/256); mod(20+udpPacketLength, 256); ...% Total Length of the IPv4 packet
    0;1; ...% Identification
    0;0; ...% Flags and Fragmentation Offset
    64; ...% Time to Live in seconds
    17; ...% Protocol number
    0;0; ...% Header Checksum
    127;0;0;1; ...% Source IP address
    127;0;0;1]; % Destination IP address

% Construct the SLL Header
sllHeader = [0;0; % Packet Type
    3;4; % ARPHRD Type
    0;0; % Link Layer address length
    0;0;0;0;0;0;0;0; % Link Layer address
    8;0]; % Protocol Type

% Concatenate the headers
encapsulationHeader = [sllHeader; ipHeader; udpHeader];
end