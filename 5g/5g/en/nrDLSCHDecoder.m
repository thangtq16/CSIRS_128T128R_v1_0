classdef nrDLSCHDecoder< nr5g.internal.TransportChannelDecoder
%nrDLSCHDecoder Downlink Shared Channel (DL-SCH) Decoder
%   DLSCHDEC = nrDLSCHDecoder creates a Downlink Shared Channel Decoder
%   System object, DLSCHDEC. This object takes PDSCH output and processes
%   it through the components of the downlink shared channel (DL-SCH)
%   decoder (rate recovery, LDPC decoding, desegmentation, and CRC
%   decoding). It decodes signals that were encoded according to 3GPP TS
%   38.212:
%   * Section 7.2 Downlink shared channel and paging channel
%
%   DLSCHDEC = nrDLSCHDecoder(Name,Value) creates a DL-SCH decoder object,
%   DLSCHDEC, with the specified property Name set to the specified Value.
%   You can specify additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN,ValueN).
%
%   Step method syntaxes
%
%   TRBLKOUT = step(DLSCHDEC,RXSOFTBITS,MODULATION,NLAYERS,RV), and
%   TRBLKOUT = step(DLSCHDEC,RXSOFTBITS,MODULATION,NLAYERS,RV,HARQID)
%   both apply the DL-SCH decoding chain to the RXSOFTBITS input.
%   RXSOFTBITS is a cell array or column vector of received log-likelihood
%   ratio (LLR) values corresponding to the received codeword(s). When a
%   cell array, it can have at most two elements, with each element a
%   column vector. MODULATION is a one of {'QPSK','16QAM','64QAM','256QAM',
%   '1024QAM'} character arrays or strings specifying the modulation 
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
%   TransportBlockLength, a public property of nrDLSCHDecoder, representing
%   the decoded bits per transport block.
%
%   The object uses soft buffer state retention to combine the different
%   redundancy version received codewords for an individual HARQ process.
%   When multiple processes are enabled, independent buffers per process
%   are maintained. For multi-codeword transmissions, independent buffers
%   per codeword are maintained.
%
%   [TRBLKOUT,BLKERR] = step(DLSCHDEC,...) also returns an error flag
%   BLKERR to indicate if the transport block(s) was decoded in error or
%   not (true indicates an error). BLKERR is a logical row vector of length
%   2 for two-codeword processing.
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
%   nrDLSCHDecoder methods:
%
%   step            - Decode PDSCH output to get the current transport
%                     block(s) (see above)
%   release         - Allow property value and input characteristics changes
%   clone           - Create a DL-SCH decoder object with same property values
%   <a href="matlab:help matlab.System/info   ">info</a>            - DL-SCH decoder status information
%   isLocked        - Locked status (logical)
%   <a href="matlab:help nrDLSCHDecoder/reset">reset</a>           - Reset buffers for all HARQ processes
%   resetSoftBuffer - Reset buffer for specified HARQ process
%
%   nrDLSCHDecoder properties:
%
%   MultipleHARQProcesses         - Enable multiple HARQ processes
%   CBGTransmission               - Enable code block group based
%                                   transmission
%   AutoFlushSoftBuffer           - Enable automatic flushing of the soft
%                                   buffer on transport block CRC pass
%   TargetCodeRate                - Target code rate
%   TransportBlockLength          - Length of decoded transport block(s)
%                                   (in bits)
%   LimitedBufferSize             - Limited buffer size (Nref) for rate
%                                   recovery
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
%   Example 1:
%   % Use a DL-SCH Encoder and Decoder system object back to back.
%
%   targetCodeRate = 526/1024;
%   modulation = 'QPSK';
%   nlayers = 2;
%   trBlkLen = 5120;
%   outCWLen = 10240;
%   rv = 0;
%
%   % Construct and configure encoder system object
%   enc = nrDLSCH;
%   enc.TargetCodeRate = targetCodeRate;
%
%   % Construct and configure decoder system object
%   dec = nrDLSCHDecoder;
%   dec.TargetCodeRate = targetCodeRate;
%   dec.TransportBlockLength = trBlkLen;
%
%   % Construct random data and send it through encoder and decoder, back
%   % to back.
%   trBlk = randi([0 1],trBlkLen,1);
%   setTransportBlock(enc,trBlk);
%   codedTrBlock = enc(modulation,nlayers,outCWLen,rv);
%   rxSoftBits = 1.0 - 2.0*double(codedTrBlock);
%   [decbits,blkerr] = dec(rxSoftBits,modulation,nlayers,rv);
%
%   % Check that the decoder output matches the original data
%   isequal(decbits,trBlk)
%
%   Example 2:
%   % Use DL-SCH Encoder and Decoder system objects with different
%   % transport block length processing
%
%   encDL = nrDLSCH('MultipleHARQProcesses',true);
%   cwID = 0;
%   harqID = 1;
%   modSch = 'QPSK';
%   nlayers = 1;
%   rv = 0;
%
%   trBlkLen1 = 5120;
%   trBlk1 = randi([0 1],trBlkLen1,1,'int8');
%   setTransportBlock(encDL,trBlk1,cwID,harqID);
%   outCWLen1 = 10240;
%   codedTrBlock1 = step(encDL,modSch,nlayers,outCWLen1,rv,harqID);
%
%   decDL = nrDLSCHDecoder('MultipleHARQProcesses',true);
%   decDL.TransportBlockLength = trBlkLen1;
%
%   rxBits1 = awgn(1-2*double(codedTrBlock1),5);
%   [decBits1,blkErr1] = step(decDL,rxBits1,modSch,nlayers,rv,harqID);
%
%   % Switch to a different transport block length for same HARQ process
%   trBlkLen2 = 4400;
%   trBlk2 = randi([0 1],trBlkLen2,1,'int8');
%   setTransportBlock(encDL,trBlk2,cwID,harqID);
%   outCWLen2 = 8800;
%   codedTrBlock2 = step(encDL,modSch,nlayers,outCWLen2,rv,harqID);
%
%   rxBits2 = awgn(1-2*double(codedTrBlock2),8);
%   decDL.TransportBlockLength = trBlkLen2;
%   if blkErr1
%       % Reset decoder if there was an error for first transport block
%       resetSoftBuffer(decDL,cwID,harqID);
%   end
%   [decBits2,blkErr2] = step(decDL,rxBits2,modSch,nlayers,rv,harqID);
%   [blkErr1 blkErr2]
%
%   See also nrDLSCH, nrDLSCHInfo, nrPDSCHDecode.

 
%   Copyright 2018-2024 The MathWorks, Inc.

    methods
        function out=nrDLSCHDecoder
            % Constructor
        end

        function out=getPropertyGroupsImpl(~) %#ok<STOUT>
            % Managing the display order of properties
        end

        function out=resetSoftBuffer(~) %#ok<STOUT>
            %resetSoftBuffer Reset soft buffer per codeword for HARQ process
            %
            %   resetSoftBuffer(DLSCHDEC,CWID) and
            %   resetSoftBuffer(DLSCHDEC,CWID,HARQID) resets the internal soft
            %   buffer for specified codeword CWID (either of 0 or 1) and HARQ
            %   process HARQID. HARQID defaults to a value of 0 when not
            %   specified. When MultipleHARQProcesses property is set to true,
            %   HARQID must be an integer in [0, 31].
            %
            %   resetSoftBuffer(DLSCHDEC,NAME=VALUE) resets the internal soft
            %   buffer with the parameters specified by the following
            %   NAME-VALUE arguments:
            %
            %   'HARQID'    - HARQ process ID, an integer in the range [0, 31]
            %                 (default 0)
            %   'BlockID'   - Transport block ID, 0 (default) or 1
        end

    end
    properties
        ModList;

    end
end
