classdef nrPDSCHDMRSConfig < nr5g.internal.DMRSConfigBase
    %nrPDSCHDMRSConfig NR PDSCH DM-RS configuration
    %   DMRS = nrPDSCHDMRSConfig creates a demodulation reference signal
    %   (DM-RS) configuration object for a physical downlink shared channel
    %   (PDSCH), as described in TS 38.211 Section 7.4.1.1. This object
    %   bundles all the properties involved in PDSCH-specific DM-RS symbols
    %   and indices generation, along with the resource elements pattern
    %   which is unavailable for data in DM-RS symbol locations. Given the
    %   DM-RS configuration type and antenna port set, the object includes
    %   read-only properties describing the DM-RS subcarrier locations
    %   within a resource block, code division multiplexing (CDM) groups,
    %   and time and frequency weights to be applied for DM-RS symbols. By
    %   default, the object defines a single-symbol DM-RS located at symbol
    %   index 2 (0-based) with configuration type 1 and antenna port 0.
    %
    %   DMRS = nrPDSCHDMRSConfig(Name,Value) creates a DM-RS configuration
    %   object with the specified property Name set to the specified
    %   Value. The additional Name-Value pair arguments can be specified in
    %   any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPDSCHDMRSConfig properties (configurable):
    %
    %   DMRSConfigurationType   - DM-RS configuration type (1 (default), 2)
    %   DMRSReferencePoint      - The reference point for the DM-RS
    %                             sequence to subcarrier resource mapping
    %                             ('CRB0' (default), 'PRB0')
    %   DMRSTypeAPosition       - Position of first DM-RS OFDM symbol in a
    %                             slot (2 (default), 3)
    %   DMRSAdditionalPosition  - Maximum number of DM-RS additional positions
    %                             (0...3) (default 0)
    %   DMRSLength              - DM-RS length (1 (default), 2)
    %   CustomSymbolSet         - Custom DM-RS symbol set (0-based)
    %                             (default [])
    %   DMRSPortSet             - DM-RS antenna port set (0...11)
    %                             (default [])
    %   NIDNSCID                - DM-RS scrambling identities (0...65535)
    %                             (default [])
    %   NSCID                   - DM-RS scrambling initialization
    %                             (0 (default), 1)
    %   NumCDMGroupsWithoutData - Number of CDM groups without data (1...3)
    %                             (default 2)
    %   DMRSDownlinkR16         - Enable low PAPR DM-RS sequence
    %                             (0 (default), 1)
    %   DMRSEnhancedR18         - Enable enhanced DM-RS multiplexing
    %                             (0 (default), 1)
    %
    %   nrPDSCHDMRSConfig properties (read-only):
    %
    %   CDMGroups               - CDM group number(s) corresponding to each
    %                             port according to TS 38.211 Table
    %                             7.4.1.1.2-1 or 7.4.1.1.2-2
    %   DeltaShifts             - Delta shift(s) corresponding to each CDM
    %                             group according to TS 38.211 Table
    %                             7.4.1.1.2-1 or 7.4.1.1.2-2
    %   FrequencyWeights        - Frequency weights (w_f) according to
    %                             TS 38.211 Table 7.4.1.1.2-1 or
    %                             7.4.1.1.2-2
    %   TimeWeights             - Time weights (w_t) according to TS 38.211
    %                             Table 7.4.1.1.2-1 or 7.4.1.1.2-2
    %   DMRSSubcarrierLocations - Subcarrier locations in a resource block
    %                             for each port
    %   CDMLengths              - CDM lengths in frequency and time domain
    %
    %   Example 1:
    %   % Create a default object specifying a single-symbol DM-RS located
    %   % at symbol index 2 and number of CDM groups without data set to 2
    %   % for the physical downlink shared channel.
    %
    %   dmrs = nrPDSCHDMRSConfig
    %
    %   Example 2:
    %   % Configure a PDSCH-specific DM-RS object with DM-RS configuration
    %   % type set to 1, DM-RS length set to 2, and DM-RS additional
    %   % position set to 1.
    %
    %   dmrs = nrPDSCHDMRSConfig('DMRSConfigurationType',1,...
    %            'DMRSLength',2,'DMRSAdditionalPosition',1)
    %
    %   See also nrPDSCHConfig, nrPDSCHPTRSConfig.

    %   Copyright 2019-2024 The MathWorks, Inc.

    %#codegen

    properties (Constant,Access=protected)

        % Walsh-Hadamard DM-RS frequency OCC vectors (each column is different cover)
        FDOCCTable = ...     % [wf(0) wf(1) wf(2) wf(3)]
            [1   1  1   1;
            1  -1  1  -1;
            1   1 -1  -1;
            1  -1 -1   1].';

    end

    properties

        %DMRSConfigurationType DM-RS configuration type
        %   Specify the DM-RS configuration type as a scalar positive
        %   integer. The value must be one of {1, 2}, provided by
        %   higher-layer parameter dmrs-Type. The default value is 1.
        DMRSConfigurationType = 1;

        %DMRSReferencePoint The reference point for the DM-RS sequence to
        %subcarrier resource mapping
        %   Specify the reference point for the DM-RS sequence to
        %   subcarrier resource mapping. The value must be specified as one
        %   of {'CRB0', 'PRB0'}. Use 'CRB0', if the subcarrier reference
        %   point for DM-RS sequence mapping is subcarrier 0 of common
        %   resource block 0 (CRB 0). Use 'PRB0', if the reference point is
        %   subcarrier 0 of the first PRB of the BWP (PRB 0). The latter
        %   should be used when the PDSCH is signaled via CORESET 0. In
        %   this case the BWP parameters should also be aligned with this
        %   CORESET. The default value is 'CRB0'.
        DMRSReferencePoint = 'CRB0';

        %NumCDMGroupsWithoutData Number of DM-RS code division multiplexing
        %(CDM) groups without data
        %   Specify the number of DM-RS CDM groups that are not used to
        %   transmit data, as a scalar positive integer. The value must be
        %   one of {1, 2, 3}, corresponding to CDM groups {{0}, {0,1},
        %   {0,1,2}} respectively. The default value is 2.
        NumCDMGroupsWithoutData = 2;

        %DMRSDownlinkR16 Low PAPR DM-RS
        %   Specify the use of low PAPR DM-RS. When enabled, the DM-RS sequence
        %   generation is dependent on the antenna port indices as well as
        %   the NSCID. The default value is 0.
        DMRSDownlinkR16 (1,1) logical = false;
    end

    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'DMRSConfigurationType','DMRSReferencePoint',...
            'DMRSTypeAPosition','DMRSAdditionalPosition','DMRSLength',...
            'CustomSymbolSet','DMRSPortSet','NIDNSCID','NSCID',...
            'NumCDMGroupsWithoutData','DMRSDownlinkR16','DMRSEnhancedR18','CDMGroups','DeltaShifts',...
            'FrequencyWeights','TimeWeights','DMRSSubcarrierLocations','CDMLengths'};
    end

    properties(Transient,Access=protected)
        % For code generation, the data type of this property needs to
        % remain constant as it is accessed by updateSupportedPortNumbers
        % during object construction
        DMRSConfigurationTypeInternal = 1;
    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        DMRSReferencePoint_Values = {'CRB0','PRB0'};
    end

    methods
        function obj = nrPDSCHDMRSConfig(varargin)
            %nrPDSCHDMRSConfig Create nrPDSCHDMRSConfig object
            %   Set the property values from any name-value pairs input to
            %   the object

            % Call the base class constructor method with all the
            % name-value pair inputs
            obj@nr5g.internal.DMRSConfigBase('DMRSReferencePoint','CRB0',varargin{:});
        end

        function obj = set.DMRSReferencePoint(obj,val)
            prop = 'DMRSReferencePoint';
            val = validatestring(val,obj.DMRSReferencePoint_Values,...
                [class(obj) '.' prop],prop);
            obj.(prop) = val;
        end

        function obj = set.DMRSConfigurationType(obj,value)
            obj.DMRSConfigurationType = value;
            obj.DMRSConfigurationTypeInternal = double(value); %#ok<*MCSUP>
            obj = updateSupportedPortNumbers(obj);
        end

    end

    methods (Access = protected)

        function val = validateVRBBundleSize(~,val)
            mustBeMember(val,[2 4]);
        end

    end

end
