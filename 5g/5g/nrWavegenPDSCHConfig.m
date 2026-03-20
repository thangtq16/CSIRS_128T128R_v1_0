classdef nrWavegenPDSCHConfig < nr5g.internal.wavegen.EnablePower & ...
                                nr5g.internal.nrPDSCHConfigBase & ...
                                nr5g.internal.wavegen.PXSCHConfigBase
    %nrWavegenPDSCHConfig PDSCH configuration object for 5G waveform generation
    %   CFGPDSCH = nrWavegenPDSCHConfig creates a physical downlink shared
    %   channel (PDSCH) configuration object. Use this object in a
    %   <a href="matlab:help('nrDLCarrierConfig')"
    %   >nrDLCarrierConfig</a> object that describes a 5G downlink waveform generated
    %   by <a href="matlab:help('nrWaveformGenerator')"
    %   >nrWaveformGenerator</a>. Use this object to set PDSCH configuration
    %   parameters, such as, the modulation scheme, the target code rate,
    %   the time and frequency allocation, as well as the PDSCH DM-RS and
    %   PT-RS signals (among other configurations).
    %
    %   The default nrWavegenPDSCHConfig object configures a single-layer
    %   PDSCH with mapping type A, QPSK modulation, a resource allocation
    %   of 52 resource blocks and 14 OFDM symbols in a slot, and
    %   transmission in all slots. This corresponds to full resource
    %   allocation if used in combination with a default nrWavegenBWPConfig
    %   object. By default, nrWavegenPDSCHConfig object configures
    %   single-symbol DM-RS configuration type 1.
    %
    %   CFGPDSCH = nrWavegenPDSCHConfig(Name,Value) creates a PDSCH
    %   configuration object, CFGPDSCH, with the specified property Name
    %   set to the specified Value. You can specify additional name-value
    %   pair arguments in any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrWavegenPDSCHConfig properties:
    %
    %   Enable                    - Flag turning this PDSCH on or off (default true)
    %   Label                     - Alphanumeric description for this PDSCH (default 'PDSCH1')
    %   Power                     - Power scaling in dB (default 0)
    %   BandwidthPartID           - ID of bandwidth part containing this PDSCH (default 1)
    %   Modulation                - Modulation scheme(s) of codeword(s) ('QPSK' (default), '16QAM', '64QAM', '256QAM', '1024QAM')
    %   NumLayers                 - Number of transmission layers (1...8) (default 1)
    %   MappingType               - PDSCH mapping type ('A' (default), 'B')
    %   ReservedPRB               - Reserved PRBs and OFDM symbols pattern(s) as a cell array of object(s), of class <a href="matlab:help('nrPDSCHReservedConfig')">nrPDSCHReservedConfig</a> with the properties:
    %      <a href="matlab:help('nrPDSCHReservedConfig/PRBSet')">PRBSet</a>    - Reserved PRB indices in BWP (0-based) (default [])
    %      <a href="matlab:help('nrPDSCHReservedConfig/SymbolSet')">SymbolSet</a> - OFDM symbols associated with reserved PRBs over one or more slots (default [])
    %      <a href="matlab:help('nrPDSCHReservedConfig/Period')">Period</a>    - Total number of slots in the pattern period (default [])
    %   ReservedCORESET           - CORESET (and associated search spaces) to rate match around
    %   SymbolAllocation          - OFDM symbol allocation of PDSCH within a slot (default [0 14])
    %   SlotAllocation            - Time-domain location of PDSCH (in slots) (default 0:9)
    %   Period                    - Period of slot allocation (default 10)
    %   PRBSet                    - Resource block allocation (VRB or PRB indices) (default 0:51)
    %   PRBSetType                - Type of indices used in the PRBSet property ('VRB' (default), 'PRB')
    %   VRBToPRBInterleaving      - Virtual resource blocks (VRB) to physical resource blocks interleaving (0 (default), 1)
    %   VRBBundleSize             - Bundle size in terms of number of RBs (2 (default), 4)
    %   NID                       - PDSCH DM-RS scrambling identity (0...1023) (default [])
    %   RNTI                      - Radio network temporary identifier (0...65535) (default 1)
    %   Coding                    - Flag to enable channel coding
    %   TargetCodeRate            - Target code rate (0...1) (default 526/1024)
    %   TBScaling                 - Scaling factor for transport block size calculation (0.25, 0.5, 1 (default))
    %   XOverhead                 - Rate matching overhead (0 (default), 6, 12, 18)
    %   LimitedBufferRateMatching - Flag to enable limited buffer size for rate matching (default: true)
    %   MaxNumLayers              - Maximum number of layers configured for or supported by the UE (1...8) (default 8)
    %   MCSTable                  - Higher layer parameter 'mcs-Table' configured by the appropriate L3 RRC IE ('qam64','qam256','qam1024') (default 'qam256')
    %   RVSequence                - Redundancy version sequence (default [0 2 3 1])
    %   DataSource                - Source of transport block contents (pseudo-noise (PN) or custom)
    %   DMRS                      - PDSCH-specific DM-RS configuration object, as described in <a href="matlab:help('nrPDSCHDMRSConfig')">nrPDSCHDMRSConfig</a> with properties:
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSConfigurationType')">DMRSConfigurationType</a>   - DM-RS configuration type (1 (default), 2)
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSReferencePoint')">DMRSReferencePoint</a>      - The reference point for the DM-RS
    %                                sequence to subcarrier resource mapping
    %                                ('CRB0' (default), 'PRB0')
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSTypeAPosition')">DMRSTypeAPosition</a>       - Position of first DM-RS OFDM symbol
    %                                (2 (default), 3)
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSAdditionalPosition')">DMRSAdditionalPosition</a>  - Maximum number of DM-RS additional positions
    %                                (0...3) (default 0)
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSLength')">DMRSLength</a>              - Number of consecutive DM-RS OFDM symbols (1 (default), 2)
    %      <a href="matlab:help('nrPDSCHDMRSConfig/CustomSymbolSet')">CustomSymbolSet</a>         - Custom DM-RS symbol locations (0-based) (default [])
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSPortSet')">DMRSPortSet</a>             - DM-RS antenna port set (0...11) (default []).
    %                                The default value ([]) implies that the values
    %                                are in the range from 0 to NumLayers-1
    %      <a href="matlab:help('nrPDSCHDMRSConfig/NIDNSCID')">NIDNSCID</a>                - DM-RS scrambling identities (0...65535) (default [])
    %      <a href="matlab:help('nrPDSCHDMRSConfig/NSCID')">NSCID</a>                   - DM-RS scrambling initialization (0 (default), 1)
    %      <a href="matlab:help('nrPDSCHDMRSConfig/NumCDMGroupsWithoutData')">NumCDMGroupsWithoutData</a> - Number of CDM groups without data (1...3) (default 2)
    %   DMRSPower                 - Scaling of PDSCH DM-RS power in dB (default 0)
    %   EnablePTRS                - Enable or disable the PT-RS configuration (0 (default), 1)
    %   PTRS                      - PDSCH-specific PT-RS configuration object,
    %                           as described in <a href="matlab:help('nrPDSCHPTRSConfig')">nrPDSCHPTRSConfig</a> with properties:
    %      <a href="matlab:help('nrPDSCHPTRSConfig/TimeDensity')">TimeDensity</a>      - PT-RS time density (1 (default), 2, 4)
    %      <a href="matlab:help('nrPDSCHPTRSConfig/FrequencyDensity')">FrequencyDensity</a> - PT-RS frequency density (2 (default), 4)
    %      <a href="matlab:help('nrPDSCHPTRSConfig/REOffset')">REOffset</a>         - Resource element offset ('00' (default), '01', '10', '11')
    %      <a href="matlab:help('nrPDSCHPTRSConfig/PTRSPortSet')">PTRSPortSet</a>      - PT-RS antenna port set (default [])
    %   PTRSPower                 - Scaling of PDSCH PT-RS power in dB (default 0)
    %
    %   nrWavegenPDSCHConfig properties (read-only):
    %
    %   NumCodewords              - Number of codewords
    %   TransportBlockSize        - Size of the transport block(s)
    %
    %   nrWavegenPDSCHConfig methods:
    %   
    %   nrTBS - Transport block size(s) associated with transmission
    %
    %   Example 1:
    %   %  Create a custom nrWavegenPDSCHConfig object, pass it to nrDLCarrierConfig
    %
    %   pdsch = nrWavegenPDSCHConfig('BandwidthPartID', 0, ...
    %                    'Modulation', '16QAM', 'TargetCodeRate', 658/1024, ...
    %                    'SymbolAllocation', [0 7], 'SlotAllocation', [0 2], 'Period', 3, ...
    %                    'PRBSet', 0:20, 'EnablePTRS', true);
    %
    %   cfg = nrDLCarrierConfig;
    %   cfg.PDSCH = {pdsch};
    %
    %   Example 2:
    %   %  Create 2 PDSCH configurations for 2 different bandwidth parts, pass these to nrDLCarrierConfig
    %
    %   carrier1 = nrSCSCarrierConfig('SubcarrierSpacing', 15);
    %   carrier2 = nrSCSCarrierConfig('SubcarrierSpacing', 30);
    %   bwp1 = nrWavegenBWPConfig('BandwidthPartID', 0, 'SubcarrierSpacing', 15);
    %   bwp2 = nrWavegenBWPConfig('BandwidthPartID', 1, 'SubcarrierSpacing', 30);
    %   pdsch1 = nrWavegenPDSCHConfig('RNTI', 1, 'BandwidthPartID', 0, 'Modulation', 'QPSK');
    %   pdsch2 = nrWavegenPDSCHConfig('RNTI', 2, 'BandwidthPartID', 1, 'Modulation', '16QAM');
    %
    %   cfg = nrDLCarrierConfig;
    %   cfg.SCSCarriers = {carrier1, carrier2};
    %   cfg.BandwidthParts = {bwp1, bwp2};
    %   cfg.PDSCH = {pdsch1, pdsch2};
    %
    %   See also nrDLCarrierConfig, nrWaveformGenerator.
    
    %   Copyright 2020-2024 The MathWorks, Inc.
    
    %#codegen
    properties
        %ReservedCORESET CORESET (and associated search spaces) to rate match around
        % Specify ReservedCORESET as a scalar or vector of nonnegative
        % integers. These numbers must correspond to IDs of nrCORESETConfig
        % objects specified in the CORESET property of <a
        % href="matlab:help('nrDLCarrierConfig.CORESET')">nrDLCarrierConfig</a>.
        % If a CORESET is included in ReservedCORESET, then the PDSCH will
        % rate match around this CORESET and its associated search spaces.
        % The default is [].
        ReservedCORESET = [];
        
        %NID Physical shared channel scrambling identity
        % Specify the physical shared channel scrambling identity as a scalar
        % nonnegative integer. The value must be in the range 0...1023. It is
        % the dataScramblingIdentityPDSCH (0...1023) if configured, else it is
        % the physical layer cell identity (0...1007). Use empty ([]) to make
        % this property equal to the <a href="matlab:help('nrDLCarrierConfig/NCellID')"
        % >NCellID</a> property of nrDLCarrierConfig. The
        % default value is [].
        NID = [];
        
        %TBScaling Scaling factor for transport block size
        % Specify TBScaling as a scalar or two-element vector with values
        % in {0.25, 0.5, 1}. The second scaling value applies only for a
        % second codeword. This property applies when Coding is true. The
        % default is 1.
        TBScaling = 1;
        
        %PTRSPower Power scaling of the PT-RS in dB
        % Specify PTRSPower in dB as a real scalar. The power of the PT-RS
        % within the PDSCH is scaled within the 5G waveform according to
        % this value. This scaling is additional to the channel-wide power
        % scaling determined by the Power property. The default is 0 dB.
        PTRSPower (1,1) {mustBeNumeric, mustBeReal, mustBeFinite} = 0;

    end

    properties (Constant,Hidden)
        MCSTable_Values = {'qam64','qam256','qam1024'};
    end
    
    properties (Dependent = true, Hidden = true)
        NumColumns
        Wpa
    end
    
    properties (Hidden)
        CustomPropList = {'Enable', 'Label', 'Power', 'BandwidthPartID', ...
            'Modulation','NumLayers','MappingType', 'ReservedPRB', 'ReservedCORESET', ...
            'SymbolAllocation',  'SlotAllocation', 'Period', 'PRBSet', 'PRBSetType', ...
            'VRBToPRBInterleaving', 'VRBBundleSize', 'AntennaMapping', 'PrecodingMatrix', ...
            'NID', 'RNTI', 'Coding', ...
            'TargetCodeRate', 'TBScaling', 'XOverhead', 'LimitedBufferRateMatching', ...
            'MaxNumLayers', 'MCSTable', 'RVSequence', 'DataSource', ...
            'DMRS', 'DMRSPower', 'EnablePTRS', 'PTRS', 'PTRSPower','NumCodewords','TransportBlockSize'};
    end
    

    properties (Dependent,SetAccess=private)
        %TransportBlockSize Transport block sizes
        TransportBlockSize;
    end

    methods

        % Constructor
        function obj = nrWavegenPDSCHConfig(varargin)
            % Get nid and EnableLBRM value from the name-value pairs
            nid = nr5g.internal.parseProp('NID',[],varargin{:});
            lbrm = nr5g.internal.parseProp('LimitedBufferRateMatching',true,varargin{:});
            
            % Support name-value pair arguments when constructing object
            obj@nr5g.internal.nrPDSCHConfigBase( ...
                'Label', 'PDSCH1', ...
                'NID', nid, ...
                varargin{:});

            obj.LimitedBufferRateMatching = lbrm;
        end
        
        % Self-validate and set properties
        function obj = set.ReservedCORESET(obj,val)
            prop = 'ReservedCORESET';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'vector','integer','nonnegative'},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end
        
        function obj = set.NID(obj,val)
            prop = 'NID';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'scalar','integer','nonnegative','<=',1023},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end
        
        function obj = set.TBScaling(obj,val)
            prop = 'TBScaling';
            coder.internal.errorIf(~any(numel(val) == [1 2]), ...
                'nr5g:nrWaveformGenerator:InvalidSize',prop);
            validateattributes(val, {'numeric'}, ...
                {'real'}, ...
                [class(obj) '.' prop], prop);
            for idx = 1:numel(val)
                coder.internal.errorIf(~any(val(idx)==[0.25 0.5 1]), ...
                    'nr5g:nrWaveformGenerator:InvalidTBScaling');
            end
            
            obj.(prop) = val;
        end

        function tbs = get.TransportBlockSize(obj)
            tbs = nrTBS(obj);
        end

        function tbs = nrTBS(obj)
            tbs = nr5g.internal.TBSDetermination.getTBSEntry(obj,obj.TargetCodeRate,obj.XOverhead,obj.TBScaling);
        end

        function numCols = get.NumColumns(obj)
            numCols = obj.getNumCols(obj.NumLayers);
        end

        function P = get.Wpa(obj)
            P = obj.calculatePrecodeAndMapMatrix(obj.NumLayers);
        end

    end

    methods (Access = public)

        % Method to check cross dependencies
        function validateConfig(obj)
            % Call PDSCH ConfigBase validator
            validateConfig@nr5g.internal.nrPDSCHConfigBase(obj);

            % Check Power
            validatePower(obj,obj.SlotAllocation,obj.Period);
        end

    end
    
    methods (Access=protected)
        % Controls the conditional display of properties
        function flag = isInactiveProperty(obj, prop)
            % Call PXSCHConfigBase method
            flag = isInactiveProperty@nr5g.internal.wavegen.PXSCHConfigBase(obj, prop);
            
            % TBScaling only if Coding is 1
            if any(strcmp(prop,{'TBScaling'}))
                flag = ~obj.Coding;
            end
            
            % VRBBundleSize only if VRBToPRBInterleaving is 1
            if strcmp(prop,'VRBBundleSize')
                flag = ~obj.VRBToPRBInterleaving;
            end

            % MaxNumLayers and MCSTable only if Coding and
            % LimitedBufferRateMatching are 1
            if any(strcmp(prop,{'MaxNumLayers','MCSTable'}))
                flag = ~(obj.Coding && obj.LimitedBufferRateMatching);
            end

        end
    end
end
