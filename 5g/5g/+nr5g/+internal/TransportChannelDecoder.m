classdef (Abstract) TransportChannelDecoder < matlab.System
%Base class for nrULSCHDecoder and nrDLSCHDecoder
%
%   Note: This is an internal undocumented class and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023-2024 The MathWorks, Inc.

%#codegen

    % Public, tunable properties
    properties

        %TargetCodeRate Target Code Rate
        %   Specify target code rate as a numeric scalar or vector of
        %   length two, each value between 0 and 1.0. The default value of
        %   this property is 526/1024.
        TargetCodeRate = 526 / 1024;

        %TransportBlockLength Size of transport block output
        %   Specify the transport block size (in bits) to be output from
        %   decoder, as a scalar or vector of length two, positive
        %   integer. The default value of this property is 5120.
        TransportBlockLength = 5120;

        %LimitedBufferSize Limited buffer size for rate recovery
        %   Specify the size of the internal buffer used for rate recovery
        %   as a scalar, positive integer. The default value of this
        %   property is 25344 that corresponds to the maximum codeword
        %   length.
        LimitedBufferSize = 25344;

    end

    % Public, nontunable properties
    properties (Nontunable)

        %MultipleHARQProcesses Enable multiple HARQ processes
        %   Enable multiple HARQ processes when set to true. A maximum of
        %   32 processes can be enabled. When set to false, a single
        %   process is used. In both cases, for each process, the rate
        %   recovered input is buffered and combined with previous
        %   receptions of that process, before passing on for LDPC
        %   decoding. The default value of this property is false.
        MultipleHARQProcesses (1, 1) logical = false;

        %CBGTransmission Enable code block group (CBG) based transmission
        %   Enable code block group (CBG) based transmission when set to
        %   true. The default value of this property is false
        CBGTransmission (1, 1) logical = false;

        %AutoFlushSoftBuffer Enable automatic flushing of the soft buffer
        %   Enable automatic flushing of the soft buffer on on transport
        %   block CRC pass when set to true. The default value of this
        %   property is true
        AutoFlushSoftBuffer (1, 1) logical = true;

        %LDPCDecodingAlgorithm LDPC decoding algorithm
        %   Specify the LDPC decoding algorithm as one of 'Belief propagation', 
        %   'Layered belief propagation', 'Normalized min-sum', or
        %   'Offset min-sum'. The default value of this property is
        %   'Belief propagation'.
        LDPCDecodingAlgorithm = 'Belief propagation';

        %ScalingFactor Scaling factor for normalized min-sum decoding
        %   Specify the scaling factor as a scalar real value greater than
        %   0 and less than or equal to 1. The default value of this
        %   property is 0.75. This property only applies when
        %   LDPCDecodingAlgorithm is set to 'Normalized min-sum'.
        ScalingFactor = 0.75;

        %Offset Offset for offset min-sum decoding
        %   Specify the offset as a scalar real value greater than
        %   or equal to 0. The default value of this property is 0.5. This
        %   property only applies when LDPCDecodingAlgorithm is set
        %   to 'Offset min-sum'.
        Offset = 0.5;

        %MaximumLDPCIterationCount Maximum LDPC decoding iterations
        %   Specify the maximum number of LDPC decoding iterations as a
        %   scalar positive integer. The default value of this property is
        %   12.
        MaximumLDPCIterationCount = 12;

        %UniformCellOutput Enable cell array output regardless of number of
        %codeword
        %   If set to true, the output data type of the returned decoded
        %   codeword will be a cell array in both single- and double-
        %   codeword cases. If set to false, the output data type of the
        %   returned decoded codeword in single-codeword cases will be a
        %   column vector. The default is false.
        UniformCellOutput (1,1) logical = false;

    end

    % Public, read-only property
    properties (Dependent = true, SetAccess = private)
        TransportBlockSizesPerProcess
    end

    % Private constant properties
    properties (Access=private, Constant)
        MaxNumHARQProcesses = 32;  % Maximum number of HARQ processes supported by class
    end

    % Protected state property
    properties (Access = protected)
        pEnableLBRR; % Flag if limited buffer rate recovery is enabled
    end

    % Private properties
    properties (Access=private)
        % Copies of public properties, scalar expanded, if necessary
        pTargetCodeRate;
        pTransportBlockLength;
        % Internal cache
        pCWSoftBuffer = repmat({{0,0}},nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses,1); % soft buffer per CW per HARQ processes
        pTBCRCError = zeros(nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses,2); % TB CRC results buffer per CW per HARQ processes
        pCBGCRCError = repmat({{0,0}},nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses,1); % CBG CRC results buffer per CW per HARQ processes
        pCBCRCError = repmat({{0,0}},nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses,1); % CB CRC results buffer per CW per HARQ processes
        pTBSPerProcess = zeros(nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses,2); % TBS per process
        pSoftBufferFlushed = false(nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses,2); % soft buffer flushing status per CW per HARQ
        pLastHARQID = nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses+1; % HARQ id processed from last call
        pLastNumCW = 1; % number of CW processed in last call
    end

    % Supported values for LDPCDecodingAlgorithm
    properties (Constant, Hidden)
        LDPCDecodingAlgorithmSet = matlab.system.StringSet({ ...
            'Belief propagation','Layered belief propagation', ...
            'Normalized min-sum','Offset min-sum'});
    end

    % Protected Constant Abstract property
    properties(Access = protected, Constant = true, Abstract = true)
        ModList; % list of supported modulation schemes
    end
    
    % Constructor, setters and getters
    methods

        function obj = TransportChannelDecoder(varargin)
            % Constructor
            % Set property values from any name-value pairs input to the
            % constructor
            setProperties(obj,nargin,varargin{:});
            setPrivateProperties(obj);
        end

        function set.TargetCodeRate(obj,value)
            % Real scalar, or vector of length 2, 0<r<1
            propName = 'TargetCodeRate';
            validateattributes(length(value(:)), {'numeric'}, ...
                {'scalar','>',0,'<',3}, ...
                [class(obj) '.' propName], 'Length of TargetCodeRate');

            validateattributes(value, {'numeric'}, ...
                {'real','<',1,'>',0}, ...
                [class(obj) '.' propName], propName);

            obj.TargetCodeRate = value;
        end

        function set.TransportBlockLength(obj,value)
            % Real scalar, or vector of length 2, integer > 0
            propName = 'TransportBlockLength';
            validateattributes(length(value(:)), {'numeric'}, ...
                {'scalar','>',0,'<',3}, ...
                [class(obj) '.' propName], 'Length of TransportBlockLength');
            validateattributes(value, {'numeric'}, ...
                {'integer','>',0}, ...
                [class(obj) '.' propName], propName);

            obj.TransportBlockLength = value;
        end

        function set.LimitedBufferSize(obj,value)
            % Scalar, integer > 0
            propName = 'LimitedBufferSize';
            validateattributes(value, {'numeric'}, ...
                {'scalar','integer','>',0}, ...
                [class(obj) '.' propName], propName);

            obj.LimitedBufferSize = value;
        end
        
        function set.ScalingFactor(obj,value)
            % Scalar, 0<x<=1
            propName = 'ScalingFactor';
            validateattributes(value, {'numeric'}, ...
                {'scalar','real','>',0,'<=',1}, ...
                [class(obj) '.' propName], propName);

            obj.ScalingFactor = value;
        end

        function set.Offset(obj,value)
            % Scalar, >= 0
            propName = 'Offset';
            validateattributes(value, {'numeric'}, ...
                {'scalar','real','finite','>=',0}, ...
                [class(obj) '.' propName], propName);

            obj.Offset = value;
        end
        
        function set.MaximumLDPCIterationCount(obj,value)
            % Scalar, integer > 0
            propName = 'MaximumLDPCIterationCount';
            validateattributes(value, {'numeric'}, ...
                {'scalar','integer','>',0}, ...
                [class(obj) '.' propName], propName);

            obj.MaximumLDPCIterationCount = value;
        end

        function tbsPerProcess = get.TransportBlockSizesPerProcess(obj)

            if obj.MultipleHARQProcesses
                tbsPerProcess = obj.pTBSPerProcess;
            else
                tbsPerProcess = obj.pTBSPerProcess(1,:);
            end

        end

    end

    % Public methods
    methods (Access = public)

        function resetSoftBuffer(obj,supportCWIDSyntax,varargin)
        %resetSoftBuffer Reset soft buffer per codeword for HARQ process

            % Input parsing and validation
            fcnName = [class(obj) '/resetSoftBuffer'];
            if nargin > 2 && isnumeric(varargin{1})
                % UL: resetSoftBuffer(obj,HARQID)
                % DL: resetSoftBuffer(obj,cwID,__)
                if supportCWIDSyntax
                    cwID = varargin{1};
                    validateattributes(cwID,{'numeric'}, ...
                        {'integer','scalar','>=',0,'<=',1},fcnName,'CWID')
                    if nargin == 4
                        harqID = varargin{2};
                        nr5g.internal.validateParameters('HARQID',harqID,fcnName);
                    else
                        harqID = 0;
                    end
                else
                    harqID = varargin{1};
                    nr5g.internal.validateParameters('HARQID',harqID,fcnName);
                    cwID = 0;
                end
            else
                % resetSoftBuffer(obj,NAME,VALUE)
                pvstruct = nr5g.internal.parseOptions(fcnName,{'HARQID','BlockID'},varargin{:});
                harqID = pvstruct.HARQID;
                cwID = pvstruct.BlockID;
            end
            
            obj.pCWSoftBuffer{harqID+1}{cwID+1} = [];
            obj.pSoftBufferFlushed(harqID+1,cwID+1) = true;
        end

    end % methods public

    % Implementation of abstract methods defined in parent classes
    methods(Access = protected)

        function decoderInfo = infoImpl(obj)

            decoderInfo = struct();

            if obj.pLastHARQID>nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses
                % Decoder not used yet - set all fields to []
                decoderInfo.HARQID = [];
                decoderInfo.TransportBlockError = [];
                if obj.CBGTransmission
                    decoderInfo.CodeBlockGroupError = [];
                end
                decoderInfo.CodeBlockError = [];
                decoderInfo.SoftBufferFlushed = [];
            else
                decoderInfo.HARQID = obj.pLastHARQID;
                if obj.pLastNumCW==1
                    decoderInfo.TransportBlockError = logical(obj.pTBCRCError(obj.pLastHARQID+1,1));
                    if obj.CBGTransmission
                        decoderInfo.CodeBlockGroupError = logical(obj.pCBGCRCError{obj.pLastHARQID+1}{1});
                    end
                    decoderInfo.CodeBlockError = {logical(obj.pCBCRCError{obj.pLastHARQID+1}{1})};
                    decoderInfo.SoftBufferFlushed = obj.pSoftBufferFlushed(obj.pLastHARQID+1,1);
                else
                    decoderInfo.TransportBlockError = [logical(obj.pTBCRCError(obj.pLastHARQID+1,1)),logical(obj.pTBCRCError(obj.pLastHARQID+1,2))];
                    if obj.CBGTransmission
                        decoderInfo.CodeBlockGroupError = [logical(obj.pCBGCRCError{obj.pLastHARQID+1}{1}),logical(obj.pCBGCRCError{obj.pLastHARQID+1}{2})];
                    end
                    decoderInfo.CodeBlockError = {logical(obj.pCBCRCError{obj.pLastHARQID+1}{1}),logical(obj.pCBCRCError{obj.pLastHARQID+1}{2})};
                    decoderInfo.SoftBufferFlushed = obj.pSoftBufferFlushed(obj.pLastHARQID+1,:);
                end
            end

        end

        function num = getNumInputsImpl(obj)
            % Number of Inputs based on property for varargin in step
            num = 4 + double(obj.MultipleHARQProcesses) + double(obj.CBGTransmission);
        end

        function setupImpl(obj)
            setPrivateProperties(obj);
        end

        function [rxBits,blkCRCErr,cbgCRCErr] = stepImpl(obj,rxSoftBits,modulation, ...
                nlayers,rv,varargin)
            % Implementation of step method
            % Supported syntaxes:
            %   [rxBits,blkCRCErr] = step(obj,rxSoftBits,modulation,nlayers,rv)
            %   [rxBits,blkCRCErr] = step(obj,rxSoftBits,modulation,nlayers,rv,harqID)
            %   [rxBits,blkCRCErr] = step(obj,rxSoftBits,modulation,nlayers,rv,cbgti)
            %   [rxBits,blkCRCErr] = step(obj,rxSoftBits,modulation,nlayers,rv,harqID,cbgti)

            narginchk(5,7)
            fcnName = [class(obj) '/step'];
            
            % Validate inputs
            validateattributes(nlayers, {'numeric'}, ...
                {'scalar','integer','<=',8,'>=',1},fcnName,'NLAYERS');
            coder.internal.prefer_const(nlayers);
            coder.internal.assert(coder.internal.isConst(nlayers),'nr5g:nrXLSCH:nLayersNeedsToBeConstant');
            if nlayers>4  % Key check
                is2CW = true;
                nl1 = floor(nlayers/2);
                nl2 = ceil(nlayers/2);
                ncw = 2;
            else
                is2CW = false;
                nl1 = nlayers;
                ncw = 1;
            end

            modlist = obj.ModList;
            modScheme = nr5g.internal.validatePXSCHModulation(fcnName,modulation,ncw,modlist);
            if iscell(modulation)
                 modLen = length(modulation);
                 validateattributes(modLen,{'numeric'}, ...
                     {'scalar','>',0,'<',3},fcnName, ...
                     'Length of MODULATION specified as a cell');
            end

            % Vector RV value
            coder.internal.errorIf( is2CW && length(rv)~=2, ...
                'nr5g:nrXLSCH:InvalidRVLength');
            if is2CW
                % Vector RV value
                validateattributes(rv,{'numeric'}, ...
                    {'integer','>=',0,'<=',3},fcnName,'RV')
            else
                % Check RV input is a scalar
                nr5g.internal.validateParameters('RV',rv,fcnName);
            end

            % Conditional inputs - harqID and cbgti
            if obj.MultipleHARQProcesses % step(obj,rxSoftBits,modulation,nlayers,rv,harqID,__)
                harqID = varargin{1};
                nr5g.internal.validateParameters('HARQID',harqID,fcnName);
            else % step(obj,rxSoftBits,modulation,nlayers,rv,__)
                harqID = 0;
            end
            if obj.CBGTransmission % step(obj,rxSoftBits,modulation,nlayers,rv,__,cbgti)
                cbgti = varargin{1+double(obj.MultipleHARQProcesses)};
                nr5g.internal.validateParameters('CBGTI',cbgti,fcnName);
            else % step(obj,rxSoftBits,modulation,nlayers,rv,__)
                cbgti = 1;
            end
            % Expand to 2 columns in case only 1 CBGTI is provided for 2
            % codewords
            if iscolumn(cbgti)
                cbgtiIn = [cbgti cbgti];
            else
                cbgtiIn = cbgti;
            end

            % Process decoder for two codeword and handle empty inputs.
            if is2CW && iscell(rxSoftBits)
                coder.internal.errorIf(numel(rxSoftBits)~=2,'nr5g:nrXLSCH:InvalidMCWInput');

                rxSoftBits1 = rxSoftBits{1};
                rxSoftBits2 = rxSoftBits{2};

                if isempty(rxSoftBits1) && isempty(rxSoftBits2)
                    rxBits1 = cast(zeros(0,1,"like",rxSoftBits1),'int8');
                    rxBits2 = cast(zeros(0,1,"like",rxSoftBits2),'int8');
                    rxBits = {rxBits1, rxBits2};
                    blkCRCErr = false(1,2);  % no error
                    cbgCRCErr = false(size(cbgtiIn)); % no error
                else
                    validateattributes(rxSoftBits{1},{'double','single'}, ...
                        {'real','column'},fcnName,'codeword 1');
                    validateattributes(rxSoftBits{2},{'double','single'}, ...
                        {'real','column'},fcnName,'codeword 2');
                    % Process codeword 0
                    [rxBits1,blkCRCErr1,cbgCRCErr1] = decodeTransportBlock(obj,rxSoftBits1, ...
                        modScheme{1},nl1,rv(1),harqID,1,cbgtiIn(:,1));
                    % Process codeword 1
                    [rxBits2,blkCRCErr2,cbgCRCErr2] = decodeTransportBlock(obj,rxSoftBits2, ...
                        modScheme{2},nl2,rv(2),harqID,2,cbgtiIn(:,2));

                    rxBits = {rxBits1,rxBits2};
                    blkCRCErr = [blkCRCErr1,blkCRCErr2];
                    cbgCRCErr = [cbgCRCErr1,cbgCRCErr2];
                end

            else
            % Process decoder for single codeword and extract from cell if
            % necessary. Handle empty inputs as well.
                coder.internal.errorIf(is2CW,'nr5g:nrXLSCH:InvalidMCWInput');
                if iscell(rxSoftBits)
                    coder.internal.errorIf(numel(rxSoftBits)~=1,'nr5g:nrXLSCH:InvalidSCWInput');
                    rxSoftBits1 = rxSoftBits{1};
                else
                    rxSoftBits1 = rxSoftBits;
                end
                if isempty(rxSoftBits1)
                    rxBitsTmp = cast(zeros(size(rxSoftBits1),"like",rxSoftBits1),'int8');
                    blkCRCErr = false;  % no error
                    cbgCRCErrTemp = false(size(cbgtiIn,1),1); % no error
                else
                    validateattributes(rxSoftBits1,{'double','single'}, ...
                        {'real','column'},fcnName,'codeword 1');

                    % Process codeword 0
                    [rxBitsTmp,blkCRCErr,cbgCRCErrTemp] = decodeTransportBlock(obj,rxSoftBits1, ...
                        modScheme{1},nl1,rv(1),harqID,1,cbgtiIn(:,1));
                end
                if obj.UniformCellOutput
                    % Uniform cell output
                    rxBits = {rxBitsTmp};
                else
                    % Vector output for 1 CW
                    rxBits = rxBitsTmp;
                end
                if iscolumn(cbgti)
                    cbgCRCErr = cbgCRCErrTemp;
                else
                    cbgCRCErr = [cbgCRCErrTemp,false(numel(cbgCRCErrTemp),1)];
                end
            end

            % Store decoder status information of this step call
            obj.pLastHARQID = harqID;
            obj.pLastNumCW = 1+double(is2CW);

            % Update processed TBS
            obj.pTBSPerProcess(harqID+1,:) = obj.pTransportBlockLength.*[1 double(is2CW)];

        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            for cwIdx = 1:2
                info = nr5g.internal.getSCHInfo(obj.pTransportBlockLength(cwIdx), ...
                    obj.pTargetCodeRate(cwIdx));
                obj.pCWSoftBuffer{1}{cwIdx} = zeros(info.N,info.C);
                obj.pCBGCRCError{1}{cwIdx} = 0;
                obj.pCBCRCError{1}{cwIdx} = 0;
                if obj.MultipleHARQProcesses
                    for harqIdx = 2:nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses
                        obj.pCWSoftBuffer{harqIdx}{cwIdx} = zeros(info.N,info.C);
                        obj.pCBGCRCError{harqIdx}{cwIdx} = 0;
                        obj.pCBCRCError{harqIdx}{cwIdx} = 0;
                    end
                end
            end
            obj.pLastHARQID = nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses+1;
            obj.pTBCRCError = zeros(nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses,2);
            obj.pTBSPerProcess = zeros(nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses,2);
            obj.pSoftBufferFlushed = false(nr5g.internal.TransportChannelDecoder.MaxNumHARQProcesses,2);
            obj.pLastNumCW = 1;
        end

        function processTunedPropertiesImpl(obj)
            setPrivateProperties(obj);
        end

        function flag = isInactivePropertyImpl(obj,prop)
            flag = false;
            switch prop
                case 'LimitedBufferSize'
                    if strcmpi(class(obj),'nrULSCHDecoder') && ... 
                       ~obj.LimitedBufferRateRecovery
                        flag = true;
                    end
                case 'ScalingFactor'
                    if ~strcmp(obj.LDPCDecodingAlgorithm,'Normalized min-sum')
                        flag = true;
                    end
                case 'Offset'
                    if ~strcmp(obj.LDPCDecodingAlgorithm,'Offset min-sum')
                        flag = true;
                    end
            end
        end

        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            if isLocked(obj)
                s.pCWSoftBuffer         = obj.pCWSoftBuffer;
                s.pTargetCodeRate       = obj.pTargetCodeRate;
                s.pTransportBlockLength = obj.pTransportBlockLength;
                s.pTBCRCError           = obj.pTBCRCError;
                s.pCBGCRCError          = obj.pCBGCRCError;
                s.pCBCRCError           = obj.pCBCRCError;
                s.pLastHARQID           = obj.pLastHARQID;
                s.pSoftBufferFlushed    = obj.pSoftBufferFlushed;
                s.pTBSPerProcess        = obj.pTBSPerProcess;
                s.pLastNumCW            = obj.pLastNumCW;
            end
        end

        function loadObjectImpl(obj,s,wasLocked)
            if wasLocked
                if ~iscell(s.pCWSoftBuffer{1})
                    % nrULSCHDecoder saved before 24a has only one CW buffer
                    for i = 1:numel(s.pCWSoftBuffer)
                        % Load into first CW buffer
                        obj.pCWSoftBuffer{i}{1} = s.pCWSoftBuffer{i};
                    end
                else
                    obj.pCWSoftBuffer           = s.pCWSoftBuffer;
                end
                if isfield(s,'pTargetCodeRate')
                    % nrULSCHDecoder saved before 24a has no such property
                    obj.pTargetCodeRate         = s.pTargetCodeRate;
                end
                if isfield(s,'pTransportBlockLength')
                    % nrULSCHDecoder saved before 24a has no such property
                    obj.pTransportBlockLength   = s.pTransportBlockLength;
                end
                if isfield(s,'pEnableLBRR')
                    % Object saved before 24a has no pEnableLBRR property
                    obj.pEnableLBRR             = s.pEnableLBRR;
                end
                if isfield(s,'pTBCRCError')
                    % Object saved before 25a has no pTBCRCError property
                    obj.pTBCRCError             = s.pTBCRCError;
                end
                if isfield(s,'pCBGCRCError')
                    % Object saved before 25a has no pCBGCRCError property
                    obj.pCBGCRCError            = s.pCBGCRCError;
                end
                if isfield(s,'pCBCRCError')
                    % Object saved before 25a has no pCBCRCError property
                    obj.pCBCRCError             = s.pCBCRCError;
                end
                if isfield(s,'pLastHARQID')
                    % Object saved before 25a has no pLastHARQID property
                    obj.pLastHARQID             = s.pLastHARQID;
                end
                if isfield(s,'pSoftBufferFlushed')
                    % Object saved before 25a has no pSoftBufferFlushed
                    % property
                    obj.pSoftBufferFlushed      = s.pSoftBufferFlushed;
                end
                if isfield(s,'pTBSPerProcess')
                    % Object saved before 25a has no pTBSPerProcesso property
                    obj.pTBSPerProcess          = s.pTBSPerProcess;
                end
                if isfield(s,'pLastNumCW')
                    % Object saved before 25a has no pLastNumCW property
                    obj.pLastNumCW              = s.pLastNumCW;
                end
            end
            loadObjectImpl@matlab.System(obj,s);
        end

    end

    % Private methods
    methods (Access = private)

        function [rxBits,blkCRCErr,cbgCRCErr] = decodeTransportBlock(obj,rxSoftBits, ...
                modScheme,nlayers,rv,harqID,cwIdx,cbgti)
            % Decode per codeword

            targetCodeRate = obj.pTargetCodeRate(cwIdx);
            trBlkLen = obj.pTransportBlockLength(cwIdx);
            info = nr5g.internal.getSCHInfo(trBlkLen,targetCodeRate);

            % Determine which CBs are transmitted and perform rate recovery
            ncb = info.C;
            [transmittedCBs,cbgtiOut,cb2cbg] = nr5g.internal.getCodeblockIndices(ncb,cbgti(:));
            ncbt = numel(transmittedCBs);
            if obj.pEnableLBRR
                raterecovered = nrRateRecoverLDPC(rxSoftBits,trBlkLen, ...
                                    targetCodeRate,rv,modScheme,nlayers,ncbt, ...
                                    obj.LimitedBufferSize);
            else
                raterecovered = nrRateRecoverLDPC(rxSoftBits,trBlkLen, ...
                                    targetCodeRate,rv,modScheme,nlayers,ncbt);
            end

            % Combining all CBs
            allCB = zeros(size(raterecovered,1),ncb,class(raterecovered));
            allCB(:,transmittedCBs) = raterecovered;

            % Combining soft buffer
            if isequal(size(obj.pCWSoftBuffer{harqID+1}{cwIdx}),[info.N,ncb])
                combined = double(allCB) + obj.pCWSoftBuffer{harqID+1}{cwIdx};
            else
                combined = double(allCB);
            end

            % LDPC decoding: set to early terminate, within max iterations
            decoded = nrLDPCDecode(combined,info.BGN, ...
                obj.MaximumLDPCIterationCount,'Algorithm',obj.LDPCDecodingAlgorithm, ...
                'ScalingFactor',obj.ScalingFactor,'Offset',obj.Offset);

            % Code block desegmentation and code block CRC decoding
            [desegmented,cbCRCErrTemp] = nrCodeBlockDesegmentLDPC(decoded,info.BGN,trBlkLen+info.L);
            if isempty(cbCRCErrTemp)
                % When there is only one CB, nrCodeBlockDesegmentLDPC
                % return empty CB CRC Error. Set it to false for further
                % processing
                cbCRCErrTemp = zeros(1,ncb,'like',cbCRCErrTemp);
            end

            % Transport block CRC decoding
            [rxBits,blkErr] = nrCRCDecode(desegmented,info.CRC);

            % Logic to reset in case no more RVs are available is not here.
            % Calling code would reset this object in that case.
            errflg = any(blkErr ~= 0); % errored
            if (~obj.AutoFlushSoftBuffer) || errflg
                % If TB errors, or auto flushing is disabled, write into
                % soft buffer
                obj.pCWSoftBuffer{harqID+1}{cwIdx} = combined;
                obj.pSoftBufferFlushed(harqID+1,cwIdx) = false;
            else
                % Flush soft buffer on CRC pass if AutoFlushSoftBuffer is
                % true
                obj.pCWSoftBuffer{harqID+1}{cwIdx} = [];
                obj.pSoftBufferFlushed(harqID+1,cwIdx) = true;
            end
            blkCRCErr = errflg;

            % CBG CRC error handling
            [cbgCRCErr,cbCRCErr] = getCBGCRCError(cbCRCErrTemp,cb2cbg,cbgtiOut);

            % Cache the CRC results
            obj.pTBCRCError(harqID+1,cwIdx) = double(blkCRCErr);
            obj.pCBGCRCError{harqID+1}{cwIdx} = double(cbgCRCErr);
            obj.pCBCRCError{harqID+1}{cwIdx} = double(cbCRCErr);

        end

        function setPrivateProperties(obj)
            % Scalar expand properties, if needed.
            if isscalar(obj.TargetCodeRate)
                obj.pTargetCodeRate = obj.TargetCodeRate.*ones(1,2);
            else
                obj.pTargetCodeRate = obj.TargetCodeRate;
            end

            if isscalar(obj.TransportBlockLength)
                obj.pTransportBlockLength = obj.TransportBlockLength.*ones(1,2);
            else
                obj.pTransportBlockLength = obj.TransportBlockLength;
            end
        end

    end % methods private

end % classdef

%% Local function

% CB, CBG, TB error handling
function [cbgCRCErr,cbCRCErrOut] = getCBGCRCError(cbCRCErrIn,cb2cbg,cbgti)

    nCBG = numel(unique(cb2cbg));
    cbgCRCErr = false(numel(cbgti),1);
    cbCRCErrOut = false(1,numel(cbCRCErrIn));
    for i = 1:numel(cbCRCErrIn)
        cbCRCErrOut(i) = logical(cbCRCErrIn(i));
    end
    for i = 1:nCBG
        % CBG CRC error is only flagged when:
        % 1) This CBG is transmitted
        % 2) At least one CB in this CBG fails CRC check
        cbgCRCErr(i) = (cbgti(i) && any(cbCRCErrOut(cb2cbg==i)));
    end

end