classdef (Abstract) TransportChannel < matlab.System
%Base class for nrULSCH and nrDLSCH
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

        %LimitedBufferSize Limited buffer size
        %   Specify the size of the internal buffer used for rate matching
        %   per codeblock as a scalar, positive integer. The default value
        %   of this property is 25344 that corresponds to the maximum
        %   codeword length.
        LimitedBufferSize = 25344;

    end

    % Public, nontunable properties
    properties (Nontunable)

        %MultipleHARQProcesses Enable multiple HARQ processes
        %   Enable multiple HARQ processes when set to true. A maximum of
        %   32 processes can be enabled. When set to false, a single
        %   process is used. In both cases, input data is buffered to
        %   enable retransmissions in case of failures. The default value
        %   of this property is false.
        MultipleHARQProcesses (1, 1) logical = false;

        %CBGTransmission Enable code block group (CBG) based transmission
        %   Enable code block group (CBG) based transmission when set to
        %   true. The default value of this property is false
        CBGTransmission (1, 1) logical = false;

        %UniformCellOutput Enable cell array output regardless of number of
        %codeword
        %   If set to true, the output data type of the returned codeword
        %   will be a cell array in both single- and double-codeword cases.
        %   If set to false, the output data type of the returned codeword
        %   in single-codeword cases will be a column vector. The default
        %   is false.
        UniformCellOutput = false;

    end

    % Public, read-only property
    properties (Dependent = true, SetAccess = private)
        TransportBlockSizesPerProcess
    end

    % Protected properties
    properties (Access = protected)
        pEnableLBRM; % Flag if limited buffer rate matching is enabled
    end

    % Protected, constant properties
    properties (Access = protected, Constant = true)
        MaxNumHARQProcesses = 32; % Maximum number of HARQ processes supported by class
    end

    % Private property
    properties (Access = private)
        pTBdata = repmat({{int8(0),int8(0)}},nr5g.internal.TransportChannel.MaxNumHARQProcesses,1); % Initialize transport block per CW storage cache for max number of processes
        pTargetCodeRate; % Copies of public properties, scalar expanded, if necessary.
    end
    
    % Abstract constant properties
    properties (Access = protected, Constant = true, Abstract = true)
        ModList; % list of supported modulation schemes
    end

    % Constructor, setters, and getters
    methods

        function obj = TransportChannel(varargin)
            % Constructor
            % Set property values from any name-value pairs input to the
            % constructor
            setProperties(obj,nargin,varargin{:});
            setPrivateProperties(obj);
        end

        function set.TargetCodeRate(obj,value)
            % Real scalar, or vector of length 2, 0<r<1
            propName = 'TargetCodeRate';
            validateattributes(length(value(:)),{'numeric'}, ...
                {'scalar','>',0,'<',3}, ...
                [class(obj) '.' propName],'Length of TargetCodeRate');

            validateattributes(value, {'numeric'}, ...
                {'real','<',1,'>',0},[class(obj) '.' propName], propName);

            obj.TargetCodeRate = value;
        end

        function set.LimitedBufferSize(obj,value)
            % Scalar, integer > 0
            propName = 'LimitedBufferSize';
            validateattributes(value,{'numeric'}, ...
                {'scalar','integer','>',0}, ...
                [class(obj) '.' propName],propName);

            obj.LimitedBufferSize = value;
        end

        function tbsPerProcess = get.TransportBlockSizesPerProcess(obj)

            if obj.MultipleHARQProcesses

                tbsPerProcess = zeros(nr5g.internal.TransportChannel.MaxNumHARQProcesses,2);
                for harqIdx = 1:nr5g.internal.TransportChannel.MaxNumHARQProcesses
                    tbsPerProcess(harqIdx,:) = [numel(obj.pTBdata{harqIdx}{1}), numel(obj.pTBdata{harqIdx}{2})];
                end
                
            else

                tbsPerProcess = [numel(obj.pTBdata{1}{1}), numel(obj.pTBdata{1}{2})];

            end

        end

    end

    % Public methods
    methods (Access = public)

        function setTransportBlock(obj,supportTBIDSyntax,trBlk,varargin)
        %setTransportBlock Set transport block data

            % Input parsing and validation
            fcnName = [class(obj) '/setTransportBlock'];
            if nargin < 4
                % setTransportBlock(obj,trBlk)
                harqID = 0;
                tbID = 0;
            else
                if isnumeric(varargin{1})
                    % UL: setTransportBlock(obj,trBlk,HARQID)
                    % DL: setTransportBlock(obj,trBlk,tbID,__)
                    if supportTBIDSyntax
                        tbID = varargin{1};
                        validateattributes(tbID,{'numeric'}, ...
                            {'scalar','integer','>=',0,'<=',1},fcnName,'TBID');
                        if nargin == 5
                            harqID = varargin{2};
                            nr5g.internal.validateParameters('HARQID',harqID,fcnName);
                        else
                            harqID = 0;
                        end
                    else
                        harqID = varargin{1};
                        nr5g.internal.validateParameters('HARQID',harqID,fcnName);
                        tbID = 0;
                    end
                else
                    % setTransportBlock(obj,trBlk,NAME,VALUE)
                    pvstruct = nr5g.internal.parseOptions(fcnName,{'HARQID','BlockID'},varargin{:});
                    harqID = pvstruct.HARQID;
                    tbID = pvstruct.BlockID;
                end
            end
            
            % Allow both cell and vector as input for TB
            if iscell(trBlk)
                coder.internal.errorIf(numel(trBlk)>2 || numel(trBlk)==0, ...
                        'nr5g:nrXLSCH:InvalidInputCellLength');
                if length(trBlk)==2
                    validateattributes(trBlk{1},{'double','int8','logical'}, ...
                        {'binary','column'},fcnName,'TRBLK 0');
                    validateattributes(trBlk{2},{'double','int8','logical'}, ...
                        {'binary','column'},fcnName,'TRBLK 1');

                    obj.pTBdata{harqID+1}{1} = cast(trBlk{1},'int8');
                    obj.pTBdata{harqID+1}{2} = cast(trBlk{2},'int8');

                else
                    validateattributes(trBlk{1},{'double','int8','logical'}, ...
                        {'binary','column'},fcnName,'TrBlk');

                    obj.pTBdata{harqID+1}{tbID+1} = cast(trBlk{1},'int8');
                end
            else
                validateattributes(trBlk,{'double','int8','logical'}, ...
                    {'binary','column'},fcnName,'TrBlk');

                obj.pTBdata{harqID+1}{tbID+1} = cast(trBlk,'int8');
            end

        end

        function trBlk = getTransportBlock(obj,supportTBIDSyntax,varargin)
        %getTransportBlock Get transport block data

            % Input parsing and validation
            fcnName = [class(obj) '/getTransportBlock'];
            if nargin < 3
                % getTransportBlock(obj)
                harqID = 0;
                tbID = 0;
            else
                if isnumeric(varargin{1})
                    % UL: getTransportBlock(obj,HARQID)
                    % DL: getTransportBlock(obj,tbID,__)
                    if supportTBIDSyntax
                        tbID = varargin{1};
                        validateattributes(tbID,{'numeric'}, ...
                            {'integer','scalar','>=',0,'<=',1},fcnName,'TBID')
                        if nargin == 4
                            harqID = varargin{2};
                            nr5g.internal.validateParameters('HARQID',harqID,fcnName);
                        else
                            harqID = 0;
                        end
                    else
                        harqID = varargin{1};
                        nr5g.internal.validateParameters('HARQID',harqID,fcnName);
                        tbID = 0;
                    end
                else
                    % getTransportBlock(obj,NAME,VALUE)
                    pvstruct = nr5g.internal.parseOptions(fcnName,{'HARQID','BlockID'},varargin{:});
                    harqID = pvstruct.HARQID;
                    tbID = pvstruct.BlockID;
                end
            end

            trBlk = obj.pTBdata{harqID+1}{tbID+1};

        end

    end % methods public

    % Implementation of Abstract methods defined in parent classes
    methods (Access = protected)

        function num = getNumInputsImpl(obj)
            % Number of Inputs based on property for varargin in step
            num = 4 + double(obj.MultipleHARQProcesses) + double(obj.CBGTransmission);
        end

        function setupImpl(obj, varargin)
            % Always scalar expand the properties
            setPrivateProperties(obj);
        end

        function [cwout,cbgtiOut] = stepImpl(obj,modulation,nlayers,outCWLen,rv,varargin)
            % Implementation of step method
            % Supported syntaxes:
            %   step(obj,modulation,nlayers,outCWLen,rv)
            %   step(obj,modulation,nlayers,outCWLen,rv,cbgti)
            %   step(obj,modulation,nlayers,outCWLen,rv,harqID)
            %   step(obj,modulation,nlayers,outCWLen,rv,harqID,cbgti)

            narginchk(5,7);
            fcnName = [class(obj) '/step'];

            % Validate inputs
            validateattributes(nlayers,{'numeric'}, ...
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

            validateattributes(length(outCWLen(:)),{'numeric'}, ...
                {'scalar','>',0,'<',3},fcnName,'Length of OUTCWLEN');
            validateattributes(outCWLen,{'numeric'}, ...
                {'integer','>=',0},fcnName,'OUTCWLEN');
            if is2CW && isscalar(outCWLen) % Scalar expand, if necessary
                outCWLength = outCWLen.*ones(1,2);
            else
                outCWLength = outCWLen;
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
            if obj.MultipleHARQProcesses % step(obj,modulation,nlayers,outCWLen,rv,harqID,__)
                harqID = varargin{1};
                nr5g.internal.validateParameters('HARQID',harqID,fcnName);
            else % step(obj,modulation,nlayers,outCWLen,rv,__)
                harqID = 0;
            end
            if obj.CBGTransmission % step(obj,modulation,nlayers,outCWLen,rv,__,cbgti)
                cbgti = varargin{1+double(obj.MultipleHARQProcesses)};
                nr5g.internal.validateParameters('CBGTI',cbgti,fcnName);
            else % step(obj,modulation,nlayers,outCWLen,rv,__)
                cbgti = 1;
            end
            % Expand to 2 columns in case only 1 CBGTI is provided for 2
            % codewords
            if iscolumn(cbgti)
                cbgtiIn = [cbgti cbgti];
            else
                cbgtiIn = cbgti;
            end

            if is2CW
                % Empty in, empty out
                if isempty(obj.pTBdata{harqID+1}{1}) && ...
                        isempty(obj.pTBdata{harqID+1}{2})
                    cw1 = cast(zeros(0,1,"like",obj.pTBdata{harqID+1}{1}),'int8');
                    cw2 = cast(zeros(0,1,"like",obj.pTBdata{harqID+1}{2}),'int8');
                    cwout = {cw1, cw2};
                    cbgtiOut = zeros(size(cbgtiIn));
                else
                    % Process codeword 0
                    [cw1,cbgtiOut1] = encodeTransportBlock(obj,modScheme{1},nl1,outCWLength(1), ...
                        rv(1),harqID,1,cbgtiIn(:,1));
                    % Process codeword 1
                    [cw2,cbgtiOut2] = encodeTransportBlock(obj,modScheme{2},nl2,outCWLength(2), ...
                        rv(2),harqID,2,cbgtiIn(:,2));
                    cwout = {cw1, cw2};
                    cbgtiOut = [cbgtiOut1 cbgtiOut2];
                end
            else    % Single codeword
                % Empty in, empty out
                if isempty(obj.pTBdata{harqID+1}{1})
                    cwoutTmp = cast(zeros(0,1,"like",obj.pTBdata{harqID+1}{1}),'int8');
                    cbgtiOutTemp = zeros(size(cbgtiIn,1),1);
                else
                    % Process codeword 0
                    [cwoutTmp,cbgtiOutTemp] = encodeTransportBlock(obj,modScheme{1},nl1,outCWLength(1), ...
                        rv(1),harqID,1,cbgtiIn(:,1));
                end
                if obj.UniformCellOutput
                    % Uniform cell output
                    cwout = {cwoutTmp};
                else
                    % Vector output for 1 CW
                    cwout = cwoutTmp;
                end
                if iscolumn(cbgti)
                    cbgtiOut = cbgtiOutTemp;
                else
                    cbgtiOut = [cbgtiOutTemp,zeros(numel(cbgtiOutTemp),1)];
                end
            end

        end

        function resetImpl(~)
            % Must be a no-op, if setTransportBlock method is used
            % as else the whole object gets reset, since setup calls reset.
        end

        function processTunedPropertiesImpl(obj)
            % Perform calculations if tunable properties change while
            % system is running
            setPrivateProperties(obj);
        end

        function s = saveObjectImpl(obj)
            % Implementation of save method
            s = saveObjectImpl@matlab.System(obj);
            if isLocked(obj)
                s.pTBdata         = obj.pTBdata;
                s.pTargetCodeRate = obj.pTargetCodeRate;
                s.pEnableLBRM     = obj.pEnableLBRM;
            end
        end

        function loadObjectImpl(obj,s,wasLocked)

            % Implementation of load method
            if wasLocked
                if isfield(s,'pEnableLBRM')
                    % Object saved before 24a has no such property
                    obj.pEnableLBRM       = s.pEnableLBRM;
                end
                if isfield(s,'pTargetCodeRate')
                    % nrULSCH saved before 24a has no such property
                    obj.pTargetCodeRate   = s.pTargetCodeRate;
                end
                if ~iscell(s.pTBdata{1})
                    % nrULSCH saved before 24a has only one TB
                    for i = 1:numel(s.pTBdata)
                        % Load into first TB
                        obj.pTBdata{i}{1} = s.pTBdata{i};
                    end
                else
                    obj.pTBdata           = s.pTBdata;
                end
            end
            loadObjectImpl@matlab.System(obj,s);
        end

    end % methods protected

    % Private methods
    methods (Access = private)

        function [codeword,cbgtiOut] = encodeTransportBlock(obj,modulation,nlayers,outCWLen, ...
                rv,harqID,tbIdx,cbgti)
        % Encode per codeword

            % Get transport block
            trBlk = obj.pTBdata{harqID+1}{tbIdx};

            % Create the informational output 'chinfo'
            chinfo = nr5g.internal.getSCHInfo(length(trBlk),obj.pTargetCodeRate(tbIdx));

            % Transport block CRC attachment
            crced = nrCRCEncode(trBlk,chinfo.CRC);

            % Code block segmentation and code block CRC attachment
            segmented = nrCodeBlockSegmentLDPC(crced,chinfo.BGN);

            % Select CBGs to be transmitted in the current transmission
            [transmittedCBs,cbgtiOut] = nr5g.internal.getCodeblockIndices(chinfo.C,cbgti);
            selected = segmented(:,transmittedCBs);

            % Channel coding
            encoded = nrLDPCEncode(selected,chinfo.BGN);

            % Rate matching
            if obj.pEnableLBRM
                codeword = nrRateMatchLDPC(encoded,outCWLen,rv,modulation, ...
                    nlayers,obj.LimitedBufferSize);
            else % Full buffer
                codeword = nrRateMatchLDPC(encoded,outCWLen,rv,modulation, ...
                    nlayers);
            end

        end

        function setPrivateProperties(obj)
            % Scalar expand relevant properties
            if isscalar(obj.TargetCodeRate)
                obj.pTargetCodeRate = obj.TargetCodeRate.*ones(1,2);
            else
                obj.pTargetCodeRate = obj.TargetCodeRate;
            end
        end

    end % methods private

end % classdef
