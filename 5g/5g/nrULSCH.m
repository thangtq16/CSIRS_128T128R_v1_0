classdef nrULSCH < nr5g.internal.TransportChannel
%nrULSCH Uplink Shared Channel (UL-SCH)
%   ULSCH = nrULSCH creates an Uplink Shared Channel System object, ULSCH.
%   This object takes in transport block(s) and processes the transport
%   block(s) through the components of the uplink shared channel (CRC,
%   code-block segmentation, LDPC encoding, and rate matching).
%   This object implements the following aspects of 3GPP TS 38.212:
%   * Section 6.2 Uplink shared channel
%   ** Sections 6.2.1 to 6.2.6, *without* the 6.2.7 Data and control
%   multiplexing
%
%   ULSCH = nrULSCH(Name,Value) creates a UL-SCH object, ULSCH, with the
%   specified property Name set to the specified Value. You can specify
%   additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN,ValueN).
%
%   setTransportBlock method syntaxes
%
%   setTransportBlock(ULSCH,TRBLK) and
%   setTransportBlock(ULSCH,TRBLK,HARQID) load transport block TRBLK into
%   ULSCH for the specified HARQ process HARQID. Input HARQID is optional,
%   with a default value of 0. This method must be used prior to the step
%   method so that there is data to process. For a single transport block
%   loaded per invocation, TRBLK must be a column vector or a cell array of
%   one column vector. For two transport blocks, TRBLK must be a cell array
%   of two column vectors.
%   setTransportBlock(ULSCH,TRBLK,NAME=VALUE) specifies loading options
%   using one or more NAME-VALUE arguments:
%
%   'HARQID'    - HARQ process ID, an integer in the range [0, 31] (default 0)
%   'BlockID'   - Transport block ID, 0 (default) or 1
%
%   Step method syntaxes
%
%   CODEDBITS = step(ULSCH,MODULATION,NLAYERS,OUTCWLEN,RV), and
%   CODEDBITS = step(ULSCH,MODULATION,NLAYERS,OUTCWLEN,RV,HARQID)
%   both apply the UL-SCH processing chain to the transport block(s) that
%   you previously loaded by using the setTransportBlock method. MODULATION
%   is one of {'pi/2-BPSK','QPSK','16QAM','64QAM','256QAM'} character
%   arrays or string specifying the modulation scheme. NLAYERS is a scalar
%   between 1 and 8 specifying the number of transmission layers. For
%   NLAYERS>4, a two-codeword transmission is assumed and the parameters
%   MODULATION and OUTCWLEN can be two-valued tuples to signify the
%   parameters for the two codewords individually. OUTCWLEN is a positive
%   integer scalar or vector of length two that specifies the output
%   codeword length in bits. RV is an integer value between 0 and 3 to
%   specify which redundancy version is to be used with this transmission.
%   For two codewords, RV must be a two-element vector. HARQID is an
%   integer scalar between 0 and 31, specifying the ID of the HARQ process
%   used with the current transmission. HARQID input is enabled when
%   MultipleHARQProcesses property is true, else there is only one HARQ
%   process used. CODEDBITS is a column vector of length OUTCWLEN and is
%   the encoded, rate-matched output.  For two codewords, CODEDBITS is a
%   cell array of two elements.
%
%   [CODEDBITS,CBGTIOUT] = step(...,CBGTIIN) specifies the code block group
%   (CBG) transmission information (CBGTI) used with the current
%   transmission in addition to the input arguments and outputs the CBGTI
%   after the encoding, where bits corresponding to the nonexisting CBGs
%   are set to 0. To use this syntax, set the CBGTransmission property to
%   true.
%
%   Usage of the setTransportBlock method for loading new data makes it
%   natural for using this object with Hybrid ARQ processing. Once a
%   transport block is loaded, it will be retained for the specified HARQ
%   process until it is overwritten using the setTransportBlock method.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj,x) and y = obj(x) are
%   equivalent.
%
%   getTransportBlock method syntaxes
%
%   TRBLK = getTransportBlock(ULSCH) and
%   TRBLK = getTransportBlock(ULSCH,HARQID) both return the transport block
%   that you previously loaded into UL-SCH using the setTransportBlock
%   method for the specified HARQ process HARQID. Input HARQID is optional
%   and defaults to a value of 0 when not used.
%   TRBLK = getTransportBlock(ULSCH,NAME=VALUE) returns the transport block
%   with options specified by the following NAME-VALUE arguments:
%
%   'HARQID'    - HARQ process ID, an integer in the range [0, 31] (default 0)
%   'BlockID'   - Transport block ID, 0 (default) or 1
%
%   nrULSCH methods:
%
%   setTransportBlock - Load next transport block into the UL-SCH for
%                       processing. Retransmission is facilitated by
%                       calling or not calling this method between
%                       successive step function calls.
%   step              - Encode the data contained in the internal buffer to
%                       produce the next codeword for subsequent PUSCH
%                       processing (see above)
%   release           - Allow property value and input characteristics
%                       changes
%   clone             - Create a UL-SCH encoder object with same property
%                       values
%   isLocked          - Locked status (logical)
%   getTransportBlock - Get the transport block that was loaded into the
%                       UL-SCH for processing.
%
%   nrULSCH properties:
%
%   MultipleHARQProcesses         - Enable multiple HARQ processes
%   CBGTransmission               - Enable code block group based
%                                   transmission
%   TargetCodeRate                - Target code rate
%   LimitedBufferRateMatching     - Enable limited buffer rate matching
%   LimitedBufferSize             - Limited buffer size (Nref)
%   UniformCellOutput             - Enable cell array output data type for
%                                   the returned DL-SCH codeword(s)
%                                   regardless of the number of codewords
%   TransportBlockSizesPerProcess - Read-only. This property displays the
%                                   transport block sizes per HARQ process
%
%   Example 1:
%   % Use a UL-SCH Encoder and Decoder system object back to back.
%
%   targetCodeRate = 567/1024;
%   modulation = '64QAM';
%   nlayers = 1;
%   trBlkLen = 5120;
%   outCWLen = 10240;
%   rv = 0;
%
%   % Construct and configure encoder system object
%   encUL = nrULSCH;
%   encUL.MultipleHARQProcesses = false;
%   encUL.TargetCodeRate = targetCodeRate;
%
%   % Construct and configure decoder system object
%   decUL = nrULSCHDecoder;
%   decUL.MultipleHARQProcesses = false;
%   decUL.TargetCodeRate = targetCodeRate;
%   decUL.TransportBlockLength = trBlkLen;
%
%   % Construct random data and send it through encoder and decoder, back
%   % to back.
%   trBlk = randi([0 1],trBlkLen,1);
%   setTransportBlock(encUL,trBlk);
%   codedTrBlock = encUL(modulation,nlayers,outCWLen,rv);
%   rxSoftBits = 1.0 - 2.0*double(codedTrBlock);
%   [decbits,blkerr] = decUL(rxSoftBits,modulation,nlayers,rv);
%
%   % Check that the decoder output matches the original data
%   isequal(decbits,trBlk)
%
%   Example 2:
%   % Configure UL-SCH Encoder for use with multiple HARQ processes
%
%   encUL = nrULSCH;
%   encUL.MultipleHARQProcesses = true;
%   harqID = 2;
%   rv = 3;
%   modulation = 'QPSK';
%   nlayers = 3;
%   trBlkLen = 5120;
%   outCWLen = 10002;
%
%   trBlk = randi([0 1],trBlkLen,1);
%   % Latch new data into UL-SCH
%   setTransportBlock(encUL,trBlk,harqID);
%   codedTrBlock = encUL(modulation,nlayers,outCWLen,rv,harqID);
%
%   % Check that output has proper size
%   isequal(length(codedTrBlock),outCWLen)
%
%   See also nrULSCHDecoder, nrULSCHInfo, nrPUSCH.

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

    % Public, tunable property
    properties

        %LimitedBufferRateMatching Enable limited buffer for rate matching
        %   If set to true, the size of the internal buffer used for rate
        %   matching can be specified by the LimitedBufferSize property. If
        %   set to false, the internal buffer size used is the full coded
        %   length. The default value of this property is false.
        LimitedBufferRateMatching (1, 1) logical = false;

    end

    % Definition of abstract properties in base class
    properties (Access = protected, Constant = true)
        ModList = {'pi/2-BPSK','QPSK','16QAM','64QAM','256QAM'};
    end

    % Public methods
    methods

        function obj = nrULSCH(varargin)
            % Constructor

            obj@nr5g.internal.TransportChannel(varargin{:});
            obj.pEnableLBRM = obj.LimitedBufferRateMatching; % make sure pEnableLBRM is explicitly set

        end

        function set.LimitedBufferRateMatching(obj,val)

            obj.LimitedBufferRateMatching = val;
            obj.pEnableLBRM = val;

        end

        function setTransportBlock(obj,trBlk,varargin)
        %setTransportBlock Set UL-SCH transport block data
        %
        %   setTransportBlock(ULSCH,TRBLK) and
        %   setTransportBlock(ULSCH,TRBLK,HARQID) load transport block
        %   TRBLK into ULSCH for the specified HARQ process HARQID. HARQID
        %   defaults to a value of 0 when not specified. When
        %   MultipleHARQProcesses property is set to true, HARQID must be
        %   an integer in [0, 31]. When TRBLK specifies a single transport
        %   block, TRBLK is loaded into transport block 0.
        %
        %   setTransportBlock(ULSCH,TRBLK,NAME=VALUE) loads transport block
        %   TRBLK into ULSCH with parameters specified by the following
        %   NAME-VALUE arguments:
        %
        %   'HARQID'    - HARQ process ID, an integer in the range [0, 31]
        %                 (default 0)
        %   'BlockID'   - Transport block ID, 0 (default) or 1
        %
        %   This method must be used prior to the step method so that there
        %   is data to process. TRBLK must be a column vector for a single
        %   transport block loaded per invocation or a cell array of one or
        %   two elements, with each element being a column vector.

            narginchk(2,6);

            % Check that BlockID is not provided as optional input, i.e.,
            % setTransportBlock(obj,TrBlk,HarqID,BlockID) is not supported
            coder.internal.errorIf(nargin>=4 && isnumeric(varargin{1}) && isnumeric(varargin{2}), ...
                'nr5g:nrXLSCH:UnsupportedBlockIDULSCH');

            setTransportBlock@nr5g.internal.TransportChannel(obj,false,trBlk,varargin{:});

        end

        function trBlk = getTransportBlock(obj,varargin)
        %getTransportBlock Get UL-SCH transport block data
        %
        %   TRBLK = getTransportBlock(ULSCH), and
        %   TRBLK = getTransportBlock(ULSCH,HARQID) both return the
        %   transport block 0 that you previously loaded into UL-SCH using
        %   the setTransportBlock method for the specified HARQ process
        %   HARQID. HARQID defaults to a value of 0 when not specified and
        %   must be an integer in [0, 31] otherwise. When ULSCH is loaded
        %   with 2 transport blocks, transport block 0 is returned.
        %
        %   TRBLK = getTransportBlock(ULSCH,NAME=VALUE) returns the
        %   transport block with the parameters specified by the following
        %   NAME-VALUE arguments:
        %
        %   'HARQID'    - HARQ process ID, an integer in the range [0, 31]
        %                 (default 0)
        %   'BlockID'   - Transport block ID, 0 (default) or 1

            narginchk(1,5);

            % Check that BlockID is not provided as optional input, i.e.,
            % getTransportBlock(obj,HarqID,BlockID) is not supported
            coder.internal.errorIf(nargin>=3 && isnumeric(varargin{1}) && isnumeric(varargin{2}), ...
                'nr5g:nrXLSCH:UnsupportedBlockIDULSCH');

            trBlk = getTransportBlock@nr5g.internal.TransportChannel(obj,false,varargin{:});

        end

    end % end methods

    % Implementation of abstract methods in base class
    methods (Static = true, Access = protected)
        function groups = getPropertyGroupsImpl
            % Managing the display order of properties
            propList = {'MultipleHARQProcesses', ...
                        'CBGTransmission', ...
                        'TargetCodeRate', ...
                        'LimitedBufferRateMatching', ...
                        'LimitedBufferSize', ...
                        'UniformCellOutput', ...
                        'TransportBlockSizesPerProcess'};
            groups = matlab.system.display.Section('PropertyList',propList);
        end
    end
    methods (Access = protected)
        function flag = isInactivePropertyImpl(obj,prop)
            % Flagging for inactive property
            % Defined in matlab.System
            flag = false;
            switch prop
                case 'LimitedBufferSize'
                    if ~obj.LimitedBufferRateMatching
                        flag = true;
                    end
            end
        end
    end

end % end classdef
