classdef nrPUSCHDMRSConfig < nr5g.internal.DMRSConfigBase
    %nrPUSCHDMRSConfig NR PUSCH DM-RS configuration
    %   DMRS = nrPUSCHDMRSConfig creates a demodulation reference signal
    %   (DM-RS) configuration object for a physical uplink shared channel
    %   (PUSCH), as described in TS 38.211 Section 6.4.1.1. This object
    %   bundles all the properties involved in PUSCH-specific DM-RS symbols
    %   and indices generation, along with the resource elements pattern
    %   which is unavailable for data in DM-RS symbol locations. Given the
    %   DM-RS configuration type and antenna port set, the object includes
    %   read-only properties describing the DM-RS subcarrier locations
    %   within a resource block, code division multiplexing (CDM) groups,
    %   and time and frequency weights to be applied for DM-RS symbols. By
    %   default, the object defines a single-symbol DM-RS located at symbol
    %   index 2 (0-based) with configuration type 1 and antenna port 0.
    %
    %   DMRS = nrPUSCHDMRSConfig(Name,Value) creates a DM-RS configuration
    %   object with the specified property Name set to the specified
    %   Value. The additional Name-Value pair arguments can be specified in
    %   any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPUSCHDMRSConfig properties:
    %
    %   DMRSConfigurationType   - DM-RS configuration type (1 (default), 2).
    %                             When transform precoding for PUSCH is
    %                             enabled, the value must be 1
    %   DMRSTypeAPosition       - Position of first DM-RS OFDM symbol in a
    %                             slot (2 (default), 3)
    %   DMRSAdditionalPosition  - Maximum number of DM-RS additional positions
    %                             (0...3) (default 0). When intra-slot
    %                             frequency hopping for PUSCH is enabled,
    %                             the value must be either 0 or 1
    %   DMRSLength              - DM-RS length (1 (default), 2). When
    %                             intra-slot frequency hopping for PUSCH is
    %                             enabled, the value must be 1
    %   CustomSymbolSet         - DM-RS symbol locations (0-based)
    %                             (default [])
    %   DMRSPortSet             - DM-RS antenna port set (0...11)
    %                             (default [])
    %   NIDNSCID                - DM-RS scrambling identities (0...65535)
    %                             (default []). This property is used only
    %                             when transform precoding for PUSCH is
    %                             disabled
    %   NSCID                   - DM-RS scrambling initialization
    %                             (0 (default), 1). This property is used
    %                             only when transform precoding for PUSCH
    %                             is disabled
    %   GroupHopping            - Group hopping configuration
    %                             (0 (default), 1). This property is used
    %                             only when transform precoding for PUSCH
    %                             is enabled
    %   SequenceHopping         - Sequence hopping configuration
    %                             (0 (default), 1). This property is used
    %                             only when transform precoding for PUSCH
    %                             is enabled
    %   NRSID                   - DM-RS scrambling identity (0...1007)
    %                             (default []). This property is used only
    %                             when transform precoding for PUSCH is
    %                             enabled
    %   NumCDMGroupsWithoutData - Number of CDM groups without data (1...3)
    %                             (default 2). When transform precoding for
    %                             PUSCH is enabled, the value must be 2
    %   DMRSUplinkR16           - Enable low PAPR DM-RS sequence for CP-OFDM
    %                             (0 (default), 1). This property is used
    %                             only when transform precoding for PUSCH
    %                             is disabled
    %   DMRSUplinkTransformPrecodingR16 - Enable low PAPR DM-RS sequence
    %                             for DFT-s-OFDM (0 (default), 1). This
    %                             property is used only when transform
    %                             precoding for PUSCH is enabled and the
    %                             PUSCH modulation is pi/2-BPSK
    %   DMRSEnhancedR18         - Enable enhanced DM-RS multiplexing
    %                             (0 (default), 1)
    %
    %   nrPUSCHDMRSConfig properties (read-only):
    %
    %   CDMGroups               - CDM group number(s) corresponding to each
    %                             port according to TS 38.211 Table
    %                             6.4.1.1.3-1 or 6.4.1.1.3-2
    %   DeltaShifts             - Delta shift(s) corresponding to each CDM
    %                             group according to TS 38.211 Table
    %                             6.4.1.1.3-1 or 6.4.1.1.3-2
    %   FrequencyWeights        - Frequency weights (w_f) according to
    %                             TS 38.211 Table 6.4.1.1.3-1 or 6.4.1.1.3-2
    %   TimeWeights             - Time weights (w_t) according to TS 38.211
    %                             Table 6.4.1.1.3-1 or 6.4.1.1.3-2
    %   DMRSSubcarrierLocations - Subcarrier locations in a resource block
    %                             for each port
    %   CDMLengths              - CDM lengths specifying the length of
    %                             FD-CDM and TD-CDM despreading
    %
    %   Example 1:
    %   % Create a default object specifying a single-symbol DM-RS symbol
    %   % located at symbol index 2 and number of CDM groups without data
    %   % set to 2 for the physical uplink shared channel.
    %
    %   dmrs = nrPUSCHDMRSConfig
    %
    %   Example 2:
    %   % Configure a PUSCH DM-RS object with DM-RS configuration type set
    %   % to 1, DM-RS length set to 2, DM-RS additional position set to 1.
    %
    %   dmrs = nrPUSCHDMRSConfig('DMRSConfigurationType',1,...
    %            'DMRSLength',2,'DMRSAdditionalPosition',1)
    %
    %   Example 3:
    %   % Configure a PUSCH DM-RS object with group hopping set to 1, and
    %   % NRSID set to 10.
    %
    %   dmrs = nrPUSCHDMRSConfig('GroupHopping',1,...
    %            'NRSID',10)
    %
    %   See also nrPUSCHConfig, nrPUSCHPTRSConfig, nrCarrierConfig.

    %   Copyright 2019-2024 The MathWorks, Inc.

    %#codegen

    properties (Constant,Access=protected)

        % Cyclic shift DM-RS frequency OCC vectors (each column is different cover)
        FDOCCTable = ...     % [wf(0) wf(1) wf(2) wf(3)]
            [1   1  1   1;
            1  -1  1  -1;
            1  1i -1 -1i;
            1 -1i -1  1i].';

    end

    properties
        %DMRSConfigurationType DM-RS configuration type
        %   Specify the DM-RS configuration type as a scalar positive
        %   integer. The value must be one of {1, 2}, provided by
        %   higher-layer parameter dmrs-Type. Note that the value must be
        %   1, when transform precoding for PUSCH is enabled. The default
        %   value is 1.
        DMRSConfigurationType = 1;

        %GroupHopping Group hopping configuration
        %   Specify the group hopping configuration as a numeric or logical
        %   scalar. The value must be logical. False indicates that group
        %   hopping is disabled. True indicates that group hopping is enabled.
        %   This property is used only when transform precoding for PUSCH
        %   is enabled. The default value is false.
        GroupHopping (1,1) logical = false;

        %SequenceHopping Sequence hopping configuration
        %   Specify the sequence hopping configuration as a numeric or
        %   logical scalar. The value must be logical. False indicates that
        %   sequence hopping is disabled. True indicates that sequence hopping
        %   is enabled. Note that both group hopping and sequence hopping
        %   must not be enabled simultaneously. This property is used only
        %   when transform precoding for PUSCH is enabled. The default
        %   value is false.
        SequenceHopping (1,1) logical = false;

        %NRSID DM-RS scrambling identity
        %   Specify the DM-RS scrambling identity as a scalar nonnegative
        %   integer. The value must be an integer in range 0...1007 when
        %   provided by higher-layer parameter nPUSCH-Identity, else, it is
        %   equal to the physical layer cell identity NCellID. Use empty
        %   ([]) to allow this property to be equal to either <a href="matlab:help('nrCarrierConfig/NCellID')">NCellID</a> of
        %   nrCarrierConfig, or if DMRSUplinkTransformPrecodingR16 is set
        %   and type 2 DM-RS used, equal to the NID selected from NIDNSCID.
        %   This property is used only when transform precoding for PUSCH
        %   is enabled. The default value is [].
        NRSID = [];

        %NumCDMGroupsWithoutData Number of DM-RS code division multiplexing
        %(CDM) groups without data
        %   Specify the number of DM-RS CDM groups that are not used to
        %   transmit data, as a scalar positive integer. The value must be
        %   one of {1, 2, 3}, corresponding to CDM groups {{0}, {0,1},
        %   {0,1,2}} respectively. Note that the value must be 2, when
        %   transform precoding for PUSCH is enabled. The default value is 2.
        NumCDMGroupsWithoutData = 2;

        %DMRSUplinkR16 Low PAPR DM-RS for CP-OFDM
        %   Specify the use of low PAPR DM-RS for CP-OFDM. This property is
        %   used only when transform precoding for PUSCH is disabled (CP-OFDM).
        %   When the property is set to 1, the DM-RS sequence generation is
        %   dependent on the antenna port indices as well as the NSCID.
        %   The default value is 0.
        DMRSUplinkR16 (1,1) logical = false;

        %DMRSUplinkTransformPrecodingR16 Low PAPR DM-RS for DFT-s-OFDM
        %   Specify the use of low PAPR DM-RS for DFT-s-OFDM. This property is
        %   used only when transform precoding for PUSCH (DFT-s-OFDM) and
        %   pi/2-BPSK is enabled. When the property is set to 1, the DM-RS
        %   sequence generation uses type 2 low PAPR sequences, otherwise
        %   type 1 low PAPR sequences are used. The default value is 0.
        DMRSUplinkTransformPrecodingR16 (1,1) logical = false;
    end

    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'DMRSConfigurationType','DMRSTypeAPosition',...
            'DMRSAdditionalPosition','DMRSLength','CustomSymbolSet',...
            'DMRSPortSet','NIDNSCID','NSCID','GroupHopping','SequenceHopping',...
            'NRSID','NumCDMGroupsWithoutData','DMRSUplinkR16','DMRSUplinkTransformPrecodingR16','DMRSEnhancedR18','CDMGroups','DeltaShifts',...
            'FrequencyWeights','TimeWeights','DMRSSubcarrierLocations','CDMLengths'};

        %Mode Transmission mode indicates the control over visibility of
        %the DM-RS properties based on the waveform type
        % Specify the transmission mode as one of (0, 1, 2). 0 displays the
        % properties specific to CP-OFDM. 1 displays the properties
        % specific to DFT-s-OFDM. 2 displays the list of properties which
        % is the union of both CP-OFDM and DFT-s-OFDM specific properties.
        Mode = 2;
    end

    properties(Transient,Access=protected)
        % For code generation, the data type of this property needs to
        % remain constant as it is accessed by updateSupportedPortNumbers
        % during object construction
        DMRSConfigurationTypeInternal = 1;
    end

    methods
        function obj = nrPUSCHDMRSConfig(varargin)
            %nrPUSCHDMRSConfig Create nrPUSCHDMRSConfig object
            %   Set the property values from any name-value pairs input to
            %   the object
            obj@nr5g.internal.DMRSConfigBase(varargin{:});
            if ~isempty(coder.target) % Allow NRSID to be var-size in codegen
                obj.NRSID =  nr5g.internal.parseProp('NRSID',[],varargin{:});
            end
        end

        function obj = set.DMRSConfigurationType(obj,val)
            validateTPProperty(obj,val,1,'nr5g:nrPUSCHDMRSConfig:InvalidDMRSConfigTypeWithTP');
            obj.DMRSConfigurationType = val; % val is already validated
            obj.DMRSConfigurationTypeInternal = double(val); %#ok<*MCSUP>
            obj = updateSupportedPortNumbers(obj);
        end

        function val = get.DMRSConfigurationType(obj)
            val = obj.DMRSConfigurationType;
            if obj.Mode == 1
                % Provide the value of DMRSConfigurationType as 1, when
                % transform precoding for PUSCH is enabled
                val = cast(1,class(obj.DMRSConfigurationType));
            end
        end

        function val = get.DMRSConfigurationTypeInternal(obj)
             if obj.Mode == 1
                 val = 1;
             else
                 val = obj. DMRSConfigurationTypeInternal;
             end
        end

        function obj = set.NRSID(obj,val)
            prop = 'NRSID';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp, {'numeric'},...
                    {'scalar','integer','nonnegative','<=',1007},...
                    [class(obj) '.' prop], prop);
            end
            obj.(prop) = temp;
        end

        function obj = set.NumCDMGroupsWithoutData(obj,val)
            validateTPProperty(obj,val,2,'nr5g:nrPUSCHDMRSConfig:InvalidNumCDMGrpsWODataWithTP');
            obj.NumCDMGroupsWithoutData = val; % val is already validated
        end

        function val = get.NumCDMGroupsWithoutData(obj)
            val = obj.NumCDMGroupsWithoutData;
            if obj.Mode == 1
                % Provide the value of NumCDMGroupsWithoutData as 2, when
                % transform precoding for PUSCH is enabled
                val = cast(2,class(obj.NumCDMGroupsWithoutData));
            end
        end

        function validateConfig(obj)
            % Check that both the group hopping and sequence hopping are
            % not enabled simultaneously, when transform precoding for
            % PUSCH is enabled
            coder.internal.errorIf((obj.Mode == 1) && (obj.GroupHopping && obj.SequenceHopping),...
                'nr5g:nrPUSCHDMRSConfig:InvalidHoppingConfiguration');

            % Validate the general DM-RS configuration state
            validateConfig@nr5g.internal.DMRSConfigBase(obj);
        end
    end

    methods (Access = protected)

        function flag = isInactiveProperty(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line
            flag = false;

            % NIDNSCID, NSCID - only required when transform precoding for PUSCH is
            % disabled, or transform precoding and Rel 16 type 2 low-PAPR sequence required (& pi/2 BPSK)
            if strcmp(prop,'NIDNSCID') || strcmp(prop,'NSCID')
                flag = (obj.Mode == 1) && ~obj.DMRSUplinkTransformPrecodingR16;    % Inactive when TP & ~R16 type 2
            end

            % DMRSUplinkR16 - only required when transform precoding for PUSCH is
            % disabled
            if strcmp(prop,'DMRSUplinkR16')
                flag = (obj.Mode == 1);
            end

            % DMRSUplinkTransformPrecodingR16 - only required when transform precoding for PUSCH is
            % enabled
            if strcmp(prop,'DMRSUplinkTransformPrecodingR16')
                flag = (obj.Mode == 0);
            end

            % NRSID - only required when transform precoding for PUSCH is
            % enabled
            if strcmp(prop,'NRSID')
                flag = (obj.Mode == 0);
            end

            % GroupHopping - only required when transform precoding for
            % PUSCH is enabled
            if strcmp(prop,'GroupHopping')
                flag = (obj.Mode == 0);
            end

            % SequenceHopping - only required when transform precoding for
            % PUSCH is enabled
            if strcmp(prop,'SequenceHopping')
                flag = (obj.Mode == 0);
            end
        end
    end
end

% File local functions

function validateTPProperty(obj,val,refVal,errID)
    %validateTPProperty Validates the value of a property against the reference value
    %   validateTPProperty(OBJ,VAL,REFVAL,ERRID) validates the value VAL of a
    %   property against the reference value REFVAL, when transform precoding
    %   for PUSCH is enabled.

    flag = (obj.Mode == 1 && val ~= refVal); % For transform precoding enabled case
    coder.internal.errorIf(flag,errID,sprintf('%g',double(val)));

end
