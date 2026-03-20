classdef ConfigBase < nr5g.internal.pxsch.CommonConfig & nr5g.internal.FrequencyHoppingConfig ...
        & nr5g.internal.interlacing.InterlacingConfig
    %ConfigBase Class containing properties common between nrWavegenPUSCHConfig and nrPUSCHConfig
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.

    %   Copyright 2020-2024 The MathWorks, Inc.

    %#codegen

    % Public, writable properties
    properties
        %Modulation Modulation scheme
        %   Specify the modulation scheme for the codeword(s). It must be
        %   specified as one of {'pi/2-BPSK','QPSK', '16QAM', '64QAM', '256QAM'}.
        %   Modulation scheme for single codeword can be specified as a
        %   character array or string scalar. Two codewords can be
        %   configured with single modulation scheme, or cell array or
        %   string array of modulation schemes where each value corresponds
        %   to a separate codeword. The default value is 'QPSK'.
        Modulation = 'QPSK';

        %TransformPrecoding Flag to enable transform precoding
        %   Specify the flag to enable or disable transform precoding as a
        %   logical scalar. Set this value to false to indicate that
        %   transform precoding is disabled and the waveform type is
        %   CP-OFDM. Set this value to true to indicate that transform
        %   precoding is enabled and the waveform type is DFT-s-OFDM. The
        %   default value is false.
        TransformPrecoding (1,1) logical = false;

        %TransmissionScheme PUSCH transmission scheme
        %   Specify the PUSCH transmission scheme as a character vector or
        %   string scalar. The value must be one of {'nonCodebook', 'codebook'}.
        %   The default value is 'nonCodebook'.
        TransmissionScheme = 'nonCodebook';

        %NumAntennaPorts Number of antenna ports
        %   Specify the number of antenna ports for codebook based
        %   transmission scheme as a scalar positive integer. The value
        %   must be one of {1, 2, 4, 8}. The value must be greater than or
        %   equal to the number of transmission layers. The default value
        %   is 1.
        NumAntennaPorts (1,1) {mustBeMember(NumAntennaPorts, [1 2 4 8])} = 1;

        %TPMI Transmitted precoding matrix indicator
        %   Specify the transmitted precoding matrix indicator as a scalar
        %   nonnegative integer in the range 0...304. The default value is 0.
        TPMI (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(TPMI, 304)} = 0;

        %CodebookType Codebook type
        %   Specify the codebook type as one of {'codebook1_ng1n4n1',
        %   'codebook1_ng1n2n2', 'codebook2', 'codebook3', 'codebook4'}.
        %   Together with NumLayers and TPMI, CodebookType is used in
        %   codebook transmissions with 8 antenna ports to choose the
        %   precoding matrix W from TS 38.211 Tables 6.3.1.5-9 to
        %   6.3.1.5-47. Use the table below to identify which codebook type
        %   to use for a specific number of antenna groups (Ng) and
        %   specific table from TS 38.211. For single antenna group (Ng=1),
        %   the table also shows the geometrical distribution of the
        %   antenna ports in each antenna group [N1 N2]. N1 is the number
        %   of antenna ports in the horizontal direction and N2 is the
        %   number of antenna ports in the vertical direction.
        %
        %   CodebookType        |   Ng  |  [N1 N2]  |        Tables
        %   --------------------|-------|-----------|--------------------------
        %   'codebook1_ng1n4n1' |   1   |   [4 1]   |  6.3.1.5-9  to 6.3.1.5-16
        %   'codebook1_ng1n2n2' |   1   |   [2 2]   |  6.3.1.5-17 to 6.3.1.5-24
        %   'codebook2'         |   2   |     -     |  6.3.1.5-25 to 6.3.1.5-36
        %   'codebook3'         |   4   |     -     |  6.3.1.5-37 to 6.3.1.5-46
        %   'codebook4'         |   8   |     -     |  6.3.1.5-47
        %
        %   The default value is 'codebook1_ng1n4n1'.
        CodebookType = 'codebook1_ng1n4n1';

        %BetaOffsetACK Beta offset for HARQ-ACK
        %   Specify the beta offset for HARQ-ACK as a real scalar
        %   positive value. The default value is 20.
        BetaOffsetACK (1,1) {mustBeNumeric, mustBeReal, mustBePositive, mustBeFinite} = 20;

        %BetaOffsetCSI1 Beta offset for CSI part 1
        %   Specify the beta offset for CSI part 1 as a real scalar
        %   positive value. The default value is 6.25.
        BetaOffsetCSI1 (1,1) {mustBeNumeric, mustBeReal, mustBePositive, mustBeFinite} = 6.25;

        %BetaOffsetCSI2 Beta offset for CSI part 2
        %   Specify the beta offset for CSI part 2 as a real scalar
        %   positive value. The default value is 6.25.
        BetaOffsetCSI2 (1,1) {mustBeNumeric, mustBeReal, mustBePositive, mustBeFinite} = 6.25;

        %UCIScaling Scaling factor for UCI-on-PUSCH resource elements
        %   Specify the scaling factor to limit the number of resource
        %   elements allocated for UCI on PUSCH as a real scalar positive
        %   value less than or equal to 1. The nominal value is one of
        %   {0.5, 0.65, 0.8, 1}. The default value is 1.
        UCIScaling (1,1) {mustBeNumeric, mustBeReal, mustBePositive, mustBeLessThanOrEqual(UCIScaling, 1)} = 1;

        %NRAPID Random access preamble index to initialize the scrambling sequence for msgA on PUSCH
        % Specify this index as a scalar nonnegative integer. The value
        % must be in the range 0...63. It is used to initialize the
        % scrambling sequence for msgA on PUSCH, as discussed in TS 38.211
        % Section 6.3.1.1. Use empty ([]) to disable this property. The
        % default value is [].
        NRAPID = [];

        %DMRS PUSCH-specific demodulation reference signal (DM-RS) configuration object
        %   Specify the DM-RS configuration object associated with PUSCH.
        %   The default value is a default <a href="matlab:help('nrPUSCHDMRSConfig')"
        %   >nrPUSCHDMRSConfig</a> object.
        DMRS = nrPUSCHDMRSConfig;

        %PTRS PUSCH-specific phase tracking reference signal (PT-RS)
        %configuration object
        %   Specify the PT-RS configuration object associated with PUSCH.
        %   The default value is a default <a href="matlab:help('nrPUSCHPTRSConfig')"
        %   >nrPUSCHPTRSConfig</a> object.
        PTRS = nrPUSCHPTRSConfig;
    end

    % Hidden properties
    properties (Hidden)
        %Mode Transmission mode indicates the control over visibility of
        %the properties based on the waveform type
        % Specify the transmission mode as one of (0, 1, 2). 0 displays the
        % properties specific to CP-OFDM. 1 displays the properties
        % specific to DFT-s-OFDM. 2 displays the list of properties which
        % is the union of both CP-OFDM and DFT-s-OFDM specific properties.
        Mode = 0;

        %NumPorts Number of ports
        %   Number of ports. This is a hidden property and is equal to the
        %   number of layers. This is used in updating the hidden property
        %   NLayers of DMRS property.
        NumPorts = 1;
    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        Modulation_Values = {'QPSK','pi/2-BPSK','16QAM','64QAM','256QAM'};
        TransmissionScheme_Values = {'nonCodebook','codebook'};
        CodebookType_Values = {'codebook1_ng1n4n1','codebook1_ng1n2n2','codebook2','codebook3','codebook4'};
    end

    methods
        % Constructor
        function obj = ConfigBase(varargin)
            % Get the value of DMRS from the name-value pairs
            dmrs = nr5g.internal.parseProp('DMRS',nrPUSCHDMRSConfig,varargin{:});
            % Get the value of PTRS from the name-value pairs
            ptrs = nr5g.internal.parseProp('PTRS',nrPUSCHPTRSConfig,varargin{:});

            % Support name-value pair arguments when constructing object
            obj@nr5g.internal.pxsch.CommonConfig(...
                'DMRS', dmrs,...
                'PTRS', ptrs,...
                'TransmissionScheme', 'nonCodebook', ...
                'FrequencyHopping', 'neither', ...
                varargin{:});
        end

        % Self-validate and set properties
        function obj = set.Modulation(obj,val)
            prop = 'Modulation';
            modulation = validateModulation(obj,val);
            % Initialize to empty for varying length in codegen
            if iscell(modulation)
                s = size(modulation);
                obj.(prop) = repmat({''},s(1),s(2));
            else
                obj.(prop) = '';
            end
            obj.(prop) = modulation;
        end

        function obj = set.TransformPrecoding(obj,val)
            obj.TransformPrecoding = val; % val is already validated
            obj = setMode(obj);
        end

        function obj = set.TransmissionScheme(obj,val)
            prop = 'TransmissionScheme';
            val = validatestring(val,obj.TransmissionScheme_Values,[class(obj) '.' prop],prop);
            obj.(prop) = '';
            obj.(prop) = val;
        end

        function obj = set.CodebookType(obj,val)
            prop = 'CodebookType';
            val = validatestring(val,obj.CodebookType_Values,[class(obj) '.' prop],prop);
            obj.(prop) = '';
            obj.(prop) = val;
        end

        function obj = set.NRAPID(obj,val)
            prop = 'NRAPID';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'scalar','integer','nonnegative','<=',63},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end

        function obj = set.DMRS(obj,val)
            prop = 'DMRS';
            validateattributes(val,{'nrPUSCHDMRSConfig'},{'scalar'},...
                [class(obj) '.' prop],prop);
            obj.(prop) = val;
            % Update the NLayers hidden property in DMRS object with the
            % value of NumPorts
            obj.(prop).NLayers = obj.NumPorts;  %#ok<MCSUP>

            % Update the Mode hidden property in DMRS object with the value
            % of Mode
            obj.(prop).Mode = obj.Mode; %#ok<MCSUP>
        end

        function obj = set.PTRS(obj,val)
            prop = 'PTRS';
            validateattributes(val,{'nrPUSCHPTRSConfig'},{'scalar'},...
                [class(obj) '.' prop],prop);
            obj.(prop) = val;

            % Update the Mode hidden property in PTRS object with the value
            % of Mode
            obj.(prop).Mode = obj.Mode; %#ok<MCSUP>
        end

    end

    methods(Access = public)
        function validateConfig(obj)

            % Check whether the number of layers is equal to the length of
            % DM-RS port set
            emptyDMRSPortSet = isempty(obj.DMRS.DMRSPortSet);
            if ~emptyDMRSPortSet
                numDMRSPorts = numel(obj.DMRS.DMRSPortSet);
                errFlag1 = (obj.NumLayers ~= numDMRSPorts);
                coder.internal.errorIf(errFlag1,'nr5g:nrPXSCHConfig:InvalidNumLayers',obj.NumLayers,numDMRSPorts);
            end

            % Validate full DM-RS configuration object
            validateConfig(obj.DMRS);

            % Error if number of antenna ports is less than the number of
            % transmission layers, when transmission scheme is set to 'codebook'
            errFlag2 = (strcmpi(obj.TransmissionScheme,'codebook')) && (obj.NumAntennaPorts < obj.NumLayers);
            coder.internal.errorIf(errFlag2,'nr5g:nrPUSCHConfig:TooManyTxLayers',obj.NumLayers,obj.NumAntennaPorts);

            % Error if double-symbol DM-RS is configured, when intra-slot
            % frequency hopping is enabled
            intraSlotHopEnabled = strcmpi(obj.FrequencyHopping,'intraSlot') & ~obj.Interlacing;
            errFlag3 = (intraSlotHopEnabled && (isempty(obj.DMRS.CustomSymbolSet) && (obj.DMRS.DMRSLength == 2)));
            coder.internal.errorIf(errFlag3,'nr5g:nrPUSCHConfig:InvalidDMRSLength',obj.DMRS.DMRSLength);

            % Error if DMRSAdditionalPosition is configured as other than 0
            % or 1, when intra-slot frequency hopping is enabled
            errFlag4 = (intraSlotHopEnabled && (~any(obj.DMRS.DMRSAdditionalPosition == [0 1])));
            coder.internal.errorIf(errFlag4,'nr5g:nrPUSCHConfig:InvalidDMRSAdditionalPosition',obj.DMRS.DMRSAdditionalPosition);

            % Validate the combination of DM-RS port set and PT-RS port
            % set, when PT-RS is enabled and PTRSPortSet is not empty
            if obj.EnablePTRS
                validateConfig(obj.PTRS);
                validatePTRSPortCompatible(obj.DMRS,obj.PTRS);
            end

            % Error if transform precoding is enabled when there are 2 codewords
            errFlag5 = obj.TransformPrecoding && (obj.NumCodewords==2);
            coder.internal.errorIf(errFlag5,'nr5g:nrPUSCHConfig:InvalidTPFor2CW');
        end
    end

    methods (Access = private)
        function obj = setMode(obj)
            %setMode Sets the value of property Mode based on the transform
            %precoding. When transform precoding is set to 1, the mode is
            %set to 1 indicating DFT-s-OFDM waveform. When transform
            %precoding is set to 0, the mode is set to 0 indicating CP-OFDM
            %waveform
            if obj.TransformPrecoding
                obj.Mode = 1;
            else
                obj.Mode = 0;
            end

            % Set the hidden property of DMRS object with the value of the
            % Mode
            obj.DMRS.Mode = obj.Mode;

            % Set the hidden property of PTRS object with the value of the
            % Mode
            obj.PTRS.Mode = obj.Mode;
        end
        
    end

    methods (Access = protected)

        % Controls the conditional display of properties
        function inactive = isInactiveProperty(obj, prop)
            % Call base class method to determine the visibility of
            % RBSetIndex and InterlaceIndex properties
            inactive = isInactiveProperty@nr5g.internal.interlacing.InterlacingConfig(obj, prop);

            % If interlacing is on, hide PRBSet, FrequencyHopping, and SecondHopStartPRB
            inactive = inactive || (any(strcmp(prop,{'PRBSet','FrequencyHopping','SecondHopStartPRB'})) && obj.Interlacing);
        end

        % TBS lookup for the interlaced PUSCH case, where the active PRB
        % allocation may involve additional PUSCH parameters and a carrier context
        function tbs = getTBSEntryInterlaced(obj,tcr,xOh,channel,carrier)

            % The supplemental 'channel' input requires for follow data fields present:
            % channel.NStartBWP
            % channel.NSizeBWP
            % channel.RBSetIndex
            % channel.InterlaceIndex
            % These will override any of the same properties present in this class instance
            % 
            % The supplemental 'carrier' input requires the follow data fields present:
            % carrier.NStartGrid
            % carrier.NSizeGrid
            % carrier.SubcarrierSpacing
            % carrier.IntraCellGuardBands

            [~,prbset] = nr5g.internal.interlacing.interlacedResourceBlockSet(carrier,channel);
            nPRBOverride = numel(prbset);
            tbs = nr5g.internal.TBSDetermination.getTBSEntry(obj,tcr,xOh,1,nPRBOverride);
        end

    end
end
