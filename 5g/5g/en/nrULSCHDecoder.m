classdef nrULSCHDecoder< nr5g.internal.TransportChannelDecoder
%nrULSCHDecoder Uplink Shared Channel (UL-SCH) Decoder
%   ULSCHDEC = nrULSCHDecoder creates an Uplink Shared Channel Decoder
%   System object, ULSCHDEC. This object takes demultiplexed data from
%   PUSCH and processes it through the components of the uplink shared
%   channel (UL-SCH) decoder (rate recovery, LDPC decoding, desegmentation,
%   and CRC decoding). It decodes signals that were encoded according to
%   3GPP TS 38.212:
%   * Section 6.2 Uplink shared channel
%   ** Sections 6.2.1 to 6.2.6, *without* the 6.2.7 Data and control
%   multiplexing
%
%   ULSCHDEC = nrULSCHDecoder(Name,Value) creates a UL-SCH decoder object,
%   ULSCHDEC, with the specified property Name set to the specified Value.
%   You can specify additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN,ValueN).
%
%   Step method syntaxes
%
%   TRBLKOUT = step(ULSCHDEC,RXSOFTBITS,MODULATION,NLAYERS,RV), and
%   TRBLKOUT = step(ULSCHDEC,RXSOFTBITS,MODULATION,NLAYERS,RV,HARQID)
%   both apply the UL-SCH decoding chain to the RXSOFTBITS input.
%   RXSOFTBITS is a cell array or column vector of received log-likelihood
%   ratio (LLR) values corresponding to the received codeword(s). When a
%   cell array, it can have at most two elements, with each element a
%   column vector. MODULATION is a one of {'pi/2-BPSK','QPSK','16QAM',
%   '64QAM','256QAM'} character arrays or strings specifying the modulation 
%   scheme. NLAYERS is a scalar between 1 and 8 specifying the number of
%   transmission layers. For NLAYERS>4, a two-codeword transmission is 
%   assumed. MODULATION can be a two-element cell array to specify the
%   value for each codeword for a two-codeword transmission. RV is an 
%   integer value between 0 and 3 specifying which redundancy version is
%   used with this transmission. For two codewords, RV must be a 
%   two-element vector. HARQID is an integer scalar between 0 and 31, 
%   specifying the ID of the HARQ process used for the transport block(s).
%   HARQID input is enabled when MultipleHARQProcesses property is true,
%   else there is only one HARQ process in use. The output TRBLKOUT is a
%   cell array of at most two elements or a column vector of length
%   TransportBlockLength, a public property of nrULSCHDecoder, representing
%   the decoded bits per transport block.
%
%   The object uses soft buffer state retention to combine the different
%   redundancy version received codewords for an individual HARQ process.
%   When multiple processes are enabled, independent buffers per process
%   are maintained.
%
%   [TRBLKOUT,BLKERR] = step(ULSCHDEC,...) also returns an error flag
%   BLKERR to indicate if the transport block was decoded in error or not
%   (true indicates an error).
%
%   [TRBLKOUT,BLKERR,CBGERR] = step(...,CBGTI) specifies the code block
%   group (CBG) transmission information (CBGTI) used in the current
%   transmission in addition to the input arguments in the above syntaxes
%   and also outputs the CRC error flags corresponding to each CBG. To use
%   this syntax, set the CBGTransmission property to true.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj,x) and y = obj(x) are
%   equivalent.
%
%   nrULSCHDecoder methods:
%
%   step            - Decode demultiplexed data from PUSCH output to
%                     retrieve the transport block (see above)
%   release         - Allow property value and input characteristics changes
%   clone           - Create a UL-SCH decoder object with same property values
%   <a href="matlab:help matlab.System/info   ">info</a>            - UL-SCH decoder status information
%   isLocked        - Locked status (logical)
%   <a href="matlab:help nrULSCHDecoder/reset">reset</a>           - Reset buffers for all HARQ processes
%   resetSoftBuffer - Reset soft buffer for specified HARQ process
%
%   nrULSCHDecoder properties:
%
%   MultipleHARQProcesses         - Enable multiple HARQ processes
%   CBGTransmission               - Enable code block group based
%                                   transmission
%   AutoFlushSoftBuffer           - Enable automatic flushing of the soft
%                                   buffer on transport block CRC pass
%   TargetCodeRate                - Target code rate
%   TransportBlockLength          - Length of decoded transport block(s)
%                                   (in bits)
%   LimitedBufferRateRecovery     - Enable limited buffer rate recovery
%   LimitedBufferSize             - Limited buffer size (Nref)
%   LDPCDecodingAlgorithm         - LDPC decoding algorithm
%   ScalingFactor                 - Scaling factor for normalized min-sum
%                                   LDPC decoding
%   Offset                        - Offset for offset min-sum LDPC decoding
%   MaximumLDPCIterationCount     - Maximum number of LDPC decoding
%                                   iterations
%   UniformCellOutput             - Enable cell array output data type for
%                                   the returned decoded DL-SCH codeword
%                                   regardless of the number of codewords
%   TransportBlockSizesPerProcess - Read-only. The sizes of the processed
%                                   transport blocks per HARQ process
%
%   Example 1:
%   % Use a UL-SCH Encoder and Decoder system object back to back.
%
%   targetCodeRate = 602/1024;
%   modulation = 'QPSK';
%   nlayers = 2;
%   trBlkLen = 5120;
%   outCWLen = 8704;
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
%   % Use UL-SCH Encoder and Decoder system objects with different
%   % transport block length processing
%
%   encUL = nrULSCH('MultipleHARQProcesses',true);
%   harqID = 1;
%   modSch = 'QPSK';
%   nlayers = 1;
%   rv = 0;
%
%   trBlkLen1 = 5120;
%   trBlk1 = randi([0 1],trBlkLen1,1,'int8');
%   setTransportBlock(encUL,trBlk1,harqID);
%   outCWLen1 = 10240;
%   codedTrBlock1 = step(encUL,modSch,nlayers,outCWLen1,rv,harqID);
%
%   decUL = nrULSCHDecoder('MultipleHARQProcesses',true);
%   decUL.TransportBlockLength = trBlkLen1;
%
%   rxBits1 = awgn(1-2*double(codedTrBlock1),5);
%   [decBits1,blkErr1] = step(decUL,rxBits1,modSch,nlayers,rv,harqID);
%
%   % Switch to a different transport block length for same HARQ process
%   trBlkLen2 = 4400;
%   trBlk2 = randi([0 1],trBlkLen2,1,'int8');
%   setTransportBlock(encUL,trBlk2,harqID);
%   outCWLen2 = 8800;
%   codedTrBlock2 = step(encUL,modSch,nlayers,outCWLen2,rv,harqID);
%
%   rxBits2 = awgn(1-2*double(codedTrBlock2),8);
%   decUL.TransportBlockLength = trBlkLen2;
%   if blkErr1
%       % Reset decoder if there was an error for first transport block
%       resetSoftBuffer(decUL,harqID);
%   end
%   [decBits2,blkErr2] = step(decUL,rxBits2,modSch,nlayers,rv,harqID);
%   [blkErr1 blkErr2]
%
%   See also nrULSCH, nrULSCHInfo, nrPUSCHDecode.

 
%   Copyright 2018-2024 The MathWorks, Inc.

    methods
        function out=nrULSCHDecoder
        end

        function out=getPropertyGroupsImpl(~) %#ok<STOUT>
            % Managing the display order of properties
        end

        function out=resetSoftBuffer(~) %#ok<STOUT>
            %resetSoftBuffer Reset soft buffer for specified HARQ process
            %
            %   resetSoftBuffer(ULSCHDEC) and resetSoftBuffer(ULSCHDEC,HARQID)
            %   reset the internal soft buffer for the HARQ process specified
            %   by HARQID and codeword 0. HARQID defaults to a value of 0 when
            %   not specified. When MultipleHARQProcesses property is set to
            %   true, HARQID must be an integer in [0, 31].
            %
            %   resetSoftBuffer(ULSCHDEC,NAME=VALUE) resets the internal soft
            %   buffer with the parameters specified by the following
            %   NAME-VALUE arguments:
            %
            %   'HARQID'    - HARQ process ID, an integer in the range [0, 31]
            %                 (default 0)
            %   'BlockID'   - Transport block ID, 0 (default) or 1
        end

    end
    properties
        %LimitedBufferRateRecovery Enable limited buffer for rate recovery
        %   If set to true, the size of the internal buffer used for rate
        %   recovery can be specified by the LimitedBufferSize property. If
        %   set to false, the internal buffer size used is the full coded
        %   length. The default value of this property is false.
        LimitedBufferRateRecovery;

        ModList;

    end
end
