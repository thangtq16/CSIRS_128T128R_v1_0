classdef nrDLSCH < nr5g.internal.TransportChannel
%nrDLSCH Downlink Shared Channel (DL-SCH)
%   DLSCH = nrDLSCH creates a Downlink Shared Channel System object, DLSCH.
%   This object takes in a transport block and processes it through the
%   components of the downlink shared channel (CRC, code-block
%   segmentation, LDPC encoding, and rate matching). This object implements
%   the following aspects of 3GPP TS 38.212:
%   * Section 7.2 Downlink shared channel and paging channel
%   The object supports both single and multiple codewords.
%
%   DLSCH = nrDLSCH(Name,Value) creates a DL-SCH object, DLSCH, with the
%   specified property Name set to the specified Value. You can specify
%   additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN,ValueN).
%
%   setTransportBlock method syntaxes
%
%   setTransportBlock(DLSCH,TRBLK),
%   setTransportBlock(DLSCH,TRBLK,TBID), and
%   setTransportBlock(DLSCH,TRBLK,TBID,HARQID) all load transport block
%   TRBLK into DLSCH for the specified transport block TBID and HARQ
%   process HARQID. Input TBID is optional, with a default value of 0 and
%   possible values of 0 and 1. Input HARQID is optional, with a default
%   value of 0. This method must be used prior to the step method so that
%   there is data to process. TRBLK must be a column vector for a single
%   transport block loaded per invocation (when TBID is specified) or a
%   cell array of one or two elements, with each element being a column
%   vector, when TBID is not specified.
%   setTransportBlock(DLSCH,TRBLK,NAME=VALUE) specifies loading options
%   using one or more NAME-VALUE arguments:
%
%   'HARQID'    - HARQ process ID, an integer in the range [0, 31] (default 0)
%   'BlockID'   - Transport block ID, 0 (default) or 1
%
%   Step method syntaxes
%
%   PDSCHBITS = step(DLSCH,MODULATION,NLAYERS,OUTCWLEN,RV), and
%   PDSCHBITS = step(DLSCH,MODULATION,NLAYERS,OUTCWLEN,RV,HARQID)
%   both apply the DL-SCH processing chain to the transport block(s) that
%   was(were) previously loaded using the setTransportBlock method.
%   MODULATION is one of {'QPSK','16QAM','64QAM','256QAM','1024QAM'} 
%   character arrays or string specifying the modulation scheme. NLAYERS is
%   a scalar between 1 and 8 specifying the number of transmission layers.
%   For NLAYERS>4, a two-codeword transmission is assumed and the 
%   parameters MODULATION and OUTCWLEN can be two-valued tuples to signify
%   the parameters for the two codewords individually. OUTCWLEN is a 
%   positive integer scalar or vector of length two that specifies the
%   output codeword length in bits. RV is an integer value between 0 and 3
%   to specify which redundancy version is to be used with this 
%   transmission. For two codewords, RV must be a two-element vector. 
%   HARQID is an integer scalar between 0 and 31, specifying the ID of the
%   HARQ process used with the current transmission.  HARQID input is
%   enabled when MultipleHARQProcesses property is true, else there is only
%   one HARQ process used. PDSCHBITS is a column vector of length OUTCWLEN
%   and is the encoded, rate-matched output.  For two codewords, PDSCHBITS
%   is a cell array of two elements.
%
%   [PDSCHBITS,CBGTIOUT] = step(...,CBGTIIN) specifies the code block group
%   (CBG) transmission information (CBGTI) used with the current
%   transmission in addition to the input arguments and outputs the CBGTI
%   after the encoding, where bits corresponding to the nonexisting CBGs
%   are set to 0. To use this syntax, set the CBGTransmission property to
%   true.
%
%   Usage of the setTransportBlock method for loading new data makes it
%   natural for using this object with Hybrid ARQ processing. Once a
%   transport block(s) is(are) loaded, it will be retained for the
%   specified HARQ process until it is overwritten using the
%   setTransportBlock method.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj,x) and y = obj(x) are
%   equivalent.
%
%   getTransportBlock method syntaxes
%
%   TRBLK = getTransportBlock(DLSCH,TBID), and
%   TRBLK = getTransportBlock(DLSCH,TBID,HARQID) both return the transport
%   block that was loaded into DL-SCH using the setTransportBlock method
%   for the specified transport block TBID and the HARQ process HARQID.
%   Input HARQID is optional and defaults to a value of 0 when not
%   specified.
%   TRBLK = getTransportBlock(DLSCH,NAME=VALUE) returns the transport block
%   with options specified by the following NAME-VALUE arguments:
%
%   'HARQID'    - HARQ process ID, an integer in the range [0, 31] (default 0)
%   'BlockID'   - Transport block ID, 0 (default) or 1
%
%   nrDLSCH methods:
%
%   setTransportBlock - Load next transport block(s) into the DL-SCH for
%                       processing. Retransmission is facilitated by
%                       calling or not calling this method between
%                       successive step function calls.
%   step              - Encode the data contained in the internal buffer to
%                       produce the next codeword for subsequent PDSCH
%                       processing (see above)
%   release           - Allow property value and input characteristics
%                       changes
%   clone             - Create a DL-SCH encoder object with same property
%                       values
%   isLocked          - Locked status (logical)
%   getTransportBlock - Get the transport block that was loaded into the
%                       DL-SCH for processing.
%
%   nrDLSCH properties:
%
%   MultipleHARQProcesses         - Enable multiple HARQ processes
%   CBGTransmission               - Enable code block group based
%                                   transmission
%   TargetCodeRate                - Target code rate
%   LimitedBufferSize             - Limited buffer size (Nref) for rate
%                                   matching
%   UniformCellOutput             - Enable cell array output data type for
%                                   the returned DL-SCH codeword(s)
%                                   regardless of the number of codewords
%   TransportBlockSizesPerProcess - Read-only. This property displays the
%                                   transport block sizes per HARQ process
%
%   Example 1:
%   % Use the DL-SCH Encoder and Decoder system objects back to back.
%
%   targetCodeRate = 526 / 1024;
%   modulation = 'QPSK';
%   nlayers = 2;
%   trBlkLen = 5120;
%   outCWLen = 10240;
%   rv = 0;
%
%   % Construct and configure encoder system object
%   encDL = nrDLSCH;
%   encDL.TargetCodeRate = targetCodeRate;
%
%   % Construct and configure decoder system object
%   decDL = nrDLSCHDecoder;
%   decDL.TargetCodeRate = targetCodeRate;
%   decDL.TransportBlockLength = trBlkLen;
%
%   % Construct random data and send it through the encoder and decoder,
%   % back to back.
%   trBlk = randi([0 1],trBlkLen,1);
%   setTransportBlock(encDL,trBlk);
%   codedTrBlock = encDL(modulation,nlayers,outCWLen,rv);
%   rxSoftBits = (1.0 - 2.0*double(codedTrBlock));
%   [decbits,blkerr] = decDL(rxSoftBits,modulation,nlayers,rv);
%
%   % Check that the decoder output matches the original data
%   isequal(decbits,trBlk)
%
%   Example 2:
%   % Configure DL-SCH Encoder for use with multiple HARQ processes
%
%   encDL = nrDLSCH;
%   encDL.MultipleHARQProcesses = true;
%   encDL.TargetCodeRate = 490/1024;
%   tbID = 0;
%   harqID = 7;
%   rv = 2;
%   modulation = '16QAM';
%   nlayers = 2;
%   trBlkLen = 5120;
%   outCWLen = 10496;
%
%   trBlk = randi([0 1],trBlkLen,1);
%   % Latch new data into DL-SCH
%   setTransportBlock(encDL,trBlk,tbID,harqID);
%   codedTrBlock = encDL(modulation,nlayers,outCWLen,rv,harqID);
%
%   % Check that output has proper size
%   isequal(length(codedTrBlock),outCWLen)
%
%   See also nrDLSCHDecoder, nrDLSCHInfo, nrPDSCH.

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

    % Definition of abstract properties in base class
    properties (Access = protected, Constant = true)
        ModList = {'QPSK','16QAM','64QAM','256QAM','1024QAM'};
    end

    % Public methods
    methods

        function obj = nrDLSCH(varargin)
            % Constructor

            obj@nr5g.internal.TransportChannel(varargin{:});

            obj.pEnableLBRM = true;

        end

        function setTransportBlock(obj,trBlk,varargin)
        %setTransportBlock Set DL-SCH transport block data
        %
        %   setTransportBlock(DLSCH,TRBLK),
        %   setTransportBlock(DLSCH,TRBLK,TBID), and
        %   setTransportBlock(DLSCH,TRBLK,TBID,HARQID) all load transport
        %   block TRBLK into DLSCH for the specified transport block TBID
        %   and HARQ process HARQID. Input TBID is optional, with a default
        %   value of 0 and possible values of 0 and 1. HARQID defaults to a
        %   value of 0 when not specified. When MultipleHARQProcesses
        %   property is set to true, HARQID must be an integer in [0, 31].
        %
        %   setTransportBlock(DLSCH,TRBLK,NAME=VALUE) loads transport block
        %   TRBLK into DLSCH with parameters specified by the following
        %   NAME-VALUE arguments:
        %
        %   'HARQID'    - HARQ process ID, an integer in the range [0, 31]
        %                 (default 0)
        %   'BlockID'   - Transport block ID, 0 (default) or 1
        %
        %   This method must be used prior to the step method so that there
        %   is data to process. TRBLK must be a column vector for a single
        %   transport block loaded per invocation (when TBID is specified)
        %   or a cell array of one or two elements, with each element being
        %   a column vector, when TBID is not specified.

            narginchk(2,6);

            setTransportBlock@nr5g.internal.TransportChannel(obj,true,trBlk,varargin{:});

        end

        function trBlk = getTransportBlock(obj,varargin)
        %getTransportBlock Get DL-SCH transport block data
        %
        %   TRBLK = getTransportBlock(DLSCH,TBID) returns the
        %   transport block that was loaded into DL-SCH using the
        %   setTransportBlock method for the specified transport block
        %   TBID. TBID must be either 0 or 1.
        %
        %   TRBLK = getTransportBlock(...,HARQID) returns the transport
        %   block for the specified HARQ process HARQID. HARQID defaults to
        %   a value of 0 when not specified. When MultipleHARQProcesses
        %   property is set to true, HARQID must be an integer in [0, 31].
        %
        %   TRBLK = getTransportBlock(DLSCH,NAME=VALUE) returns the
        %   transport block that was loaded into DL-SCH using the
        %   setTransportBlock method with the parameters specified by the
        %   following NAME-VALUE arguments:
        %
        %   'HARQID'    - HARQ process ID, an integer in the range [0, 31]
        %                 (default 0)
        %   'BlockID'   - Transport block ID, 0 (default) or 1

            narginchk(2,5);

            trBlk = getTransportBlock@nr5g.internal.TransportChannel(obj,true,varargin{:});

        end

    end % end methods

    % Implementation of abstract methods in base class
    methods (Static = true, Access = protected)
        function groups = getPropertyGroupsImpl
            propList = {'MultipleHARQProcesses', ...
                        'CBGTransmission', ...
                        'TargetCodeRate', ...
                        'LimitedBufferSize', ...
                        'UniformCellOutput', ...
                        'TransportBlockSizesPerProcess'};
            groups = matlab.system.display.Section('PropertyList',propList);
        end
    end

end % end classdef
