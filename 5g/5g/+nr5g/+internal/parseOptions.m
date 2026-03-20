function out = parseOptions(fcnName,options,varargin)
%parseOptions parse options from a space-separated options string
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   OUT = parseOptions(FCNNAME,OPTIONS,NAME,VALUE,...) parses NAME,VALUE
%   pair options for the function FCNNAME, where FCNNAME is the calling
%   function name and OPTIONS is the set of option names to be parsed in a
%   cell row vector format. Supported NAME,VALUE pairs are:
%
%   'OutputDataType'       - 'double' for double precision (default)
%                            'single' for single precision
%
%   'IndexStyle'           - 'index' for linear indices (default)
%                            'subscript' for [subcarrier, symbol, antenna]
%                            subscript row form
%
%   'IndexBase'            - '1based' for 1-based indices (default)
%                            '0based' for 0-based indices
%
%   'IndexOrientation'     - 'carrier' for indices within Carrier grid (default)
%                            'bwp' for indices within BWP grid
%
%   'MappingType'          - 'binary' to map true to  1, false to 0 (default)
%                            'signed' to map true to -1, false to 1
%
%   'DecisionType'         - 'soft' for soft-decision demodulation/decoding (default)
%                            'hard' for hard-decision demodulation/decoding
%                            Note: for nrLDPCDecode, the default is 'hard'
%
%   'ExtractionMethod'     - 'allplanes' (default) or 'direct'
%
%   'OutputFormat'         - 'info' for information part output (default)
%                            'whole' for whole codeword output
%
%   'Algorithm'            - 'Belief propagation' (default),
%                            'Layered belief propagation', 'Normalized
%                            min-sum', and 'Offset min-sum', specify the
%                            different LDPC decoding algorithm choices.
%
%   'ScalingFactor'        - For Normalized min-sum LDPC decoding.
%                            Input value is numeric, real scalar, 0<x<=1
%                            (default 0.75)
%
%   'Offset'               - For Offset min-sum LDPC decoding.
%                            Input value is numeric, real scalar >=0
%                            (default 0.5)
%
%   'Termination'          - 'early' for decoding termination when
%                            parity-checks are satisfied (default) 'max'
%                            for decoding termination after maximum
%                            iterations
%
%   'CyclicPrefix'         - 'normal' for normal cyclic prefix (default)
%                            'extended' for extended cyclic prefix
%
%   'CDMLengths'           - A 2-element row vector [FD TD] specifying the
%                            length of FD-CDM and TD-CDM despreading to
%                            perform (default [1 1])
%
%   'AveragingWindow'      - A 2-element row vector [F T] specifying the
%                            number of adjacent reference symbols in the
%                            frequency domain F and time domain T over
%                            which to average prior to interpolation
%                            (default [0 0])
%
%   'PRGBundleSize'        - A scalar integer or [] specifying the PDSCH
%                            PRG bundle size (default [])
%
%   'OutputResourceFormat' - 'concatenated' for output of all CSI-RS
%                            resources concatenated into a single column
%                            (default) 'cell' for cell array output with
%                            each cell corresponding to an individual
%                            CSI-RS resource
%
%   'LinkEnd'              - 'Both' to display transmitter and receiver (default)
%                            'Tx' to display the transmitter only 'Rx' to
%                            display the receiver only
%
%   'Polarization'         - Polarization angle of the antenna elements
%                            'on' (default), 'off'
%
%   'ElementPattern'       - Directivity radiation pattern of the antenna
%                            elements 'on' (default), 'off'
%
%   'ClusterPaths'         - Direction and average gain of cluster paths
%                            'on' (default), 'off'
%
%   'Windowing'            - A scalar specify the number of time-domain
%                            samples over which windowing and overlapping
%                            of OFDM symbols is applied (default [])
%
%   'CarrierFrequency'     - A scalar specifying the carrier frequency
%                            (default 0)
%
%   'Nfft'                 - A scalar specifying the FFT size (default [])
%
%   'SampleRate'           - A scalar specifying the sample rate
%                            (default [])
%
%   'CyclicPrefixFraction' - A scalar specifying the CP fraction for OFDM
%                            demodulation (default 0.5)
%
%   'DetectionThreshold'   - A scalar specifying the detection threshold
%                            (default [])
%
%   'PreambleIndex'        - A vector specifying the PRACH detection
%                            preambles to look for (default 0:63)
%
%   'MultiColumnIndex'     - A numeric or logical scalar value specifying
%                            if multi-column indices should be used.
%                            (default true) (internal NV pair only)
%
%   'UniformCellOutput'    - A numeric or logical scalar value specifying
%                            if the output should always be a cell array
%                            (default false)
%
%   'HARQID'               - A scalar integer between 0 and 31 specifying
%                            the HARQ process ID for UL-SCH or DL-SCH
%                            processing. (default 0)
%
%   'BlockID'              - A scalar interger between 0 and 1 specifying
%                            the codeword or transport block ID for UL-SCH
%                            or DL-SCH processing. (default 0)
%   'TxDirectCurrentLocation' - A non negative integer used to specify the
%                               location of the DC subcarrier in the grid
%                               to exclude the contents on the DC subcarrier.
%                               (default [], which imply the DC subcarrier
%                               is not excluded)
%   'EnablePhaseCorrection'   - 0 to disable the phase correction (default)
%                               1 to enable the phase correction

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen
    persistent cache

    % If the input is a structure, assume it contains options already
    % parsed and return immediately
    if nargin==3 && isstruct(varargin{1})
        out = varargin{1};
        return;
    end

    isMATLAB = isempty(coder.target);

    if isMATLAB
        % Gather values from GPU in order to generate hash key
        [varargin{:}] = gather(varargin{:});

        hk = keyHash([options,varargin]);
        if isempty(cache)
            cache = configureDictionary("uint64","struct");
        elseif isKey(cache,hk)
            out = cache(hk);
            return;
        end
    end

    % Option definitions, default first, validation function second
    optionDef.OutputDataType = {'double' @(x,y)validateStringOption(x,y,{'double' 'single'})};
    optionDef.IndexBase = {'1based' @(x,y)validateStringOption(x,y,{'1based' '0based'})};
    optionDef.IndexStyle = {'index' @(x,y)validateStringOption(x,y,{'index' 'subscript'})};
    optionDef.IndexOrientation = {'carrier' @(x,y)validateStringOption(x,y,{'carrier' 'bwp'})};
    optionDef.MappingType = {'binary' @(x,y)validateStringOption(x,y,{'binary' 'signed'})};
    optionDef.DecisionType = {'soft' @(x,y)validateStringOption(x,y,{'soft' 'hard'})};
    optionDef.ExtractionMethod = {'allplanes' @(x,y)validateStringOption(x,y,{'allplanes' 'direct'})};
    optionDef.OutputFormat = {'info' @(x,y)validateStringOption(x,y,{'info' 'whole'})};
    optionDef.Algorithm = {'Belief propagation' @(x,y)validateStringOption(x,y,{'Belief propagation' ...
                                                                                'Layered belief propagation' 'Normalized min-sum' 'Offset min-sum'})};
    optionDef.ScalingFactor = {0.75 @validateScalingFactor};
    optionDef.Offset = {0.5 @validateOffset};
    optionDef.Termination = {'early' @(x,y)validateStringOption(x,y,{'early' 'max'})};
    optionDef.CyclicPrefix = {'normal' @(x,y)validateStringOption(x,y,{'normal' 'extended'})};
    optionDef.CDMLengths = {[1 1] @validateCDMLengths};
    optionDef.AveragingWindow = {[0 0] @validateAveragingWindow};
    optionDef.PRGBundleSize = {[] @validatePRGBundleSize};
    optionDef.OutputResourceFormat = {'concatenated' @(x,y)validateStringOption(x,y,{'concatenated' 'cell'})};
    optionDef.LinkEnd = {'Both' @(x,y)validateStringOption(x,y,{'Both' 'Tx' 'Rx'})};
    onOffDefault = {'on' @(x,y)validateStringOption(x,y,{'on' 'off'})};
    optionDef.Polarization = onOffDefault;
    optionDef.ElementPattern = onOffDefault;
    optionDef.ClusterPaths = onOffDefault;
    optionDef.Interpolation = onOffDefault;
    optionDef.Windowing = {[] @validateWindowing};
    optionDef.CarrierFrequency = {0 @validateCarrierFrequency};
    optionDef.Nfft = {[] @validateNfft};
    optionDef.SampleRate = {[] @validateSampleRate};
    optionDef.CyclicPrefixFraction = {0.5 @validateCPFraction};
    optionDef.DetectionThreshold = {[] @validateDetectionThreshold};
    optionDef.PreambleIndex = {(0:63) @validatePreambleIndex};
    optionDef.MultiColumnIndex = {true @validateMultiColumnIndex};
    optionDef.UniformCellOutput = {false @validateUniformCellOutput};
    optionDef.HARQID = {0 @validateHARQID};
    optionDef.BlockID = {0 @validateBlockID};
    optionDef.TxDirectCurrentLocation = {[] @validateTxDirectCurrentLocation};
    optionDef.EnablePhaseCorrection = {false @validateEnablePhaseCorrection};

    % For nrLDPCDecode, set the default to 'hard'
    if strcmpi(fcnName,'nrLDPCDecode')
        optionDef.DecisionType{1} = 'hard';
    end

    % parseInputs options
    parseOptions = struct( ...
        'CaseSensitivity',false, ...
        'PartialMatching','unique', ...
        'StructExpand',true, ...
        'IgnoreNulls',false, ...
        'SupportOverrides',true);

    % For MATLAB code path, the following two try,catch blocks cannot be
    % refactored into a function since we want the caught error to be
    % thrown by the function that called nr5g.internal.parseOptions.

    % Perform parsing of supplied NV pairs.
    if isMATLAB
        % In MATLAB, throw parser errors from the calling function
        try %#ok<*EMTC>
            pstruct = coder.internal.parseInputs({},options,parseOptions, ...
                                                 varargin{:});
        catch e
            throwAsCaller(e)
        end
    else
        pstruct = coder.internal.parseInputs({},options,parseOptions, ...
                                             varargin{:});
    end

    % Extract parsed value or default value
    numNVs = numel(options);
    parsedNV = cell(1,numNVs*2);
    for k = 1:numNVs
        i = 1+2*(k-1);

        name = options{k};
        parsedNV{i} = name;
        defaultValue = coder.const(optionDef.(name){1});
        validationHandle = optionDef.(name){2};

        value = coder.internal.getParameterValue( ...
            pstruct.(name),defaultValue,varargin{:});

        % Validate value if supplied by the user. For MATLAB, throw
        % validation error as function that called
        % nr5g.internal.parseOptions
        if pstruct.(name) ~= 0
            if isMATLAB
                % In MATLAB, throw validation errors from the calling function
                try
                    parsedNV{i+1} = validationHandle(value,name);
                catch e
                    throwAsCaller(e)
                end
            else
                parsedNV{i+1} = validationHandle(value,name);
            end
        else
            parsedNV{i+1} = value;
        end
    end

    out = coder.internal.constantPreservingStruct(parsedNV{:});
    if isMATLAB
        cache(hk) = out;
    end
end

%% Local Functions
function matchedStr = validateStringOption(in,name,validstr)
    amendedName = "'" + string(name) + "'";
    validateattributes(in,{'char' 'string'},{},'',amendedName);
    matchedStr = validatestring(in,validstr,'',amendedName);

end

function in = validateCDMLengths(in,~)

    validateattributes(in,{'numeric'}, ...
                       {'row','ncols',2,'positive','integer'},'','''CDMLengths''');

end

function in = validateAveragingWindow(in,~)

    validateattributes(in,{'numeric'}, ...
                       {'row','ncols',2,'nonnegative','integer'},'','''AveragingWindow''');
    validateattributes(in(in~=0),{'numeric'},{'odd'},'','''AveragingWindow''');

end

function in = validatePRGBundleSize(in,~)

    validateattributes(in,{'numeric'}, ...
                       {'integer','positive'},'','''PRGBundleSize''');
    
end

function in = validateScalingFactor(in,~)

    validateattributes(in,{'numeric'},{'scalar','real','>',0,'<=',1}, ...
                       '','''ScalingFactor''');

end

function in = validateOffset(in,~)

    validateattributes(in,{'numeric'},{'scalar','real','finite','>=',0}, ...
                       '','''Offset''');

end

function in = validateWindowing(in,~)

    if (~isempty(in))
        validateattributes(in,{'numeric'}, ...
                           {'scalar','real','nonnegative','integer'},'','''Windowing''');
    end

end

function in = validateCarrierFrequency(in,~)

    validateattributes(in,{'numeric'}, ...
                       {'scalar','real','finite'},'','''CarrierFrequency''');

end

function in = validateNfft(in,~)

    if (~isempty(in))
        validateattributes(in,{'numeric'}, ...
                           {'scalar','real','positive','integer'},'','''Nfft''');
    end

end

function in = validateSampleRate(in,~)

    if (~isempty(in))
        validateattributes(in,{'numeric'}, ...
                           {'scalar','real','positive','integer'},'','''SampleRate''');
    end

end

function in = validateCPFraction(in,~)

    validateattributes(in,{'numeric'}, ...
                       {'scalar','real','>=',0,'<=',1},'','''CyclicPrefixFraction''');

end

function in = validateDetectionThreshold(in,~)

    if ~isempty(in)
        validateattributes(in,{'double','single'},...
                           {'scalar','real','>=',0,'<=',1},'','''DetectionThreshold''');
    end

end

function in = validatePreambleIndex(in,~)

    if ~isempty(in)
        validateattributes(in,{'numeric'},...
                           {'vector','>=',0,'<=',63,'integer'},'','''PreambleIndex''');
    end

end

function in = validateMultiColumnIndex(in,~)
    mustBeNumericOrLogical(in)
end

function out = validateUniformCellOutput(in,~)
    validateattributes(in,{'numeric','logical'}, {'scalar'},'','''UniformCellOutput''');
    out = logical(in);
end

function in = validateHARQID(in,~)
    nr5g.internal.validateParameters('HARQID',in,'');
end

function in = validateBlockID(in,~)
    validateattributes(in,{'numeric'}, ...
                       {'scalar','integer','>=',0,'<=',1},'','''BlockID''');
end

function in = validateTxDirectCurrentLocation(in,~)
    if ~isempty(in)
        validateattributes(in,{'numeric'},...
            {'scalar','integer','nonnegative'},'','''TxDirectCurrentLocation''');
    end
end

function in = validateEnablePhaseCorrection(in,~)
    validateattributes(in,{'numeric','logical'},...
        {'scalar','binary'},'','''EnablePhaseCorrection''');
end
