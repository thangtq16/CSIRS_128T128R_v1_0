classdef nrWavegenPUSCHConfig < nr5g.internal.wavegen.EnablePower & ...
                                nr5g.internal.pusch.ConfigBase & ...
                                nr5g.internal.wavegen.PXSCHConfigBase
    %nrWavegenPUSCHConfig PUSCH configuration object for 5G waveform generation
    %   CFGPUSCH = nrWavegenPUSCHConfig creates a physical uplink shared
    %   channel (PUSCH) configuration object. Use this object in a
    %   <a href="matlab:help('nrULCarrierConfig')"
    %   >nrULCarrierConfig</a> object that describes a 5G uplink waveform generated
    %   by <a href="matlab:help('nrWaveformGenerator')"
    %   >nrWaveformGenerator</a>. Use this object to set PUSCH configuration
    %   parameters, such as, the modulation scheme, the target code rate,
    %   the time and frequency allocation, as well as the PUSCH DM-RS and
    %   PT-RS signals (among other configurations).
    %
    %   The default nrWavegenPUSCHConfig object configures a single-layer
    %   PUSCH with CP-OFDM, mapping type A, QPSK modulation, a resource
    %   allocation of 52 resource blocks and 14 OFDM symbols in a slot, and
    %   transmission in all slots. This corresponds to full resource
    %   allocation if used in combination with a default nrWavegenBWPConfig
    %   object. Frequency hopping, transform precoding, PT-RS, and UCI on
    %   PUSCH are disabled. By default, nrWavegenPDSCHConfig object
    %   configures single-symbol DM-RS configuration type 1.
    %
    %   CFGPUSCH = nrWavegenPUSCHConfig(Name,Value) creates a PUSCH
    %   configuration object, CFGPUSCH, with the specified property Name
    %   set to the specified Value. You can specify additional name-value
    %   pair arguments in any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrWavegenPUSCHConfig properties:
    %
    %   Enable             - Flag turning this PUSCH on or off (default true)
    %   Label              - Alphanumeric description for this PUSCH (default 'PUSCH1')
    %   Power              - Power scaling in dB (default 0)
    %   BandwidthPartID    - ID of bandwidth part containing this PUSCH (default 1)
    %   Modulation         - Modulation scheme(s) of codeword(s)
    %                        ('QPSK' (default), 'pi/2-BPSK', '16QAM', '64QAM', '256QAM')
    %   NumLayers          - Number of transmission layers (1...8) (default 1)
    %   MappingType        - PUSCH mapping type ('A' (default), 'B')
    %   SymbolAllocation   - OFDM symbol allocation of PUSCH within a slot
    %                        (default [0 14])
    %   SlotAllocation     - Time-domain location of PUSCH (in slots) (default 0:9)
    %   Period             - Period of slot allocation (default 10)
    %   PRBSet             - PRBs allocated for PUSCH within the BWP (default 0:51)
    %   TransformPrecoding - Flag to enable transform precoding (0(default), 1).
    %                        0 indicates that transform precoding is
    %                        disabled and the waveform type is CP-OFDM. 1
    %                        indicates that transform precoding is enabled
    %                        and the waveform type is DFT-s-OFDM
    %   TransmissionScheme - PUSCH transmission scheme ('nonCodebook' (default), 'codebook')
    %   NumAntennaPorts    - Number of antenna ports (1 (default), 2, 4, 8)
    %   TPMI               - Transmitted precoding matrix indicator (0...304) (default 0)
    %   CodebookType       - Codebook type ('codebook1_ng1n4n1' (default),
    %                        'codebook1_ng1n2n2', 'codebook2', 'codebook3', 'codebook4')
    %   FrequencyHopping   - Flag to enable frequency hopping
    %                        ('neither' (default), 'intraSlot', 'interSlot')
    %   SecondHopStartPRB  - Starting PRB of second hop relative to the
    %                        BWP (0...274) (default 1)
    %   Interlacing        - Enable interlacing (default false)
    %   RBSetIndex         - Resource block set index (default 0)
    %   InterlaceIndex     - Interlace indices (0...9) (default 0)
    %   NID                - PUSCH scrambling identity (0...1023) (default [])
    %   RNTI               - Radio network temporary identifier (0...65535)
    %                        (default 1)
    %   NRAPID             - Random access preamble index to initialize the
    %                        scrambling sequence for msgA on PUSCH (0...63)
    %                        (default [])
    %   Coding             - Flag to enable transport channel coding (default true)
    %   TargetCodeRate     - Target code rate (0...1) (default 526/1024)
    %   XOverhead          - Rate matching overhead (0 (default), 6, 12, 18)
    %   LimitedBufferRateMatching - Flag to enable limited buffer size for 
    %                        rate matching (default: false)
    %   MaxNumLayers       - Maximum number of layers configured for or
    %                        supported by the UE (1...8) (default 8)
    %   MCSTable           - Higher layer parameter 'mcs-Table' configured
    %                        by the appropriate L3 RRC IE ('qam64',
    %                        'qam256') (default 'qam256')
    %   RVSequence         - Redundancy version sequence (default [0 2 3 1])
    %   DataSource         - Source of transport block contents
    %                        (pseudo-noise (PN) or custom) (default 'PN9-ITU')
    %   EnableACK          - Flag to enable or disable the HARQ-ACK
    %                        transmission on PUSCH (false (default), true)
    %   NumACKBits         - Number of HARQ-ACK bits (0...1706) (default 10)
    %   BetaOffsetACK      - Beta offset for HARQ-ACK (default 20)
    %   DataSourceACK      - Source of HARQ-ACK contents
    %                        (PN or custom) (default 'PN9-ITU')
    %   EnableCSI1         - Flag to enable or disable the CSI part 1
    %                        transmission on PUSCH (false (default), true)
    %   NumCSI1Bits        - Number of CSI part 1 bits (0...1706) (default 10)
    %   BetaOffsetCSI1     - Beta offset for CSI part 1 (default 6.25)
    %   DataSourceCSI1     - Source of CSI part 1 contents
    %                        (PN or custom) (default 'PN9-ITU')
    %   EnableCSI2         - Flag to enable or disable the CSI part 2
    %                        transmission on PUSCH (false (default), true)
    %   NumCSI2Bits        - Number of CSI part 2 bits (0...1706) (default 10)
    %   BetaOffsetCSI2     - Beta offset for CSI part 2 (default 6.25)
    %   DataSourceCSI2     - Source of CSI part 2 contents
    %                        (PN or custom) (default 'PN9-ITU')
    %   EnableCGUCI        - Flag to enable or disable the CG-UCI
    %                        transmission on PUSCH (false (default), true)
    %   NumCGUCIBits       - Number of CG-UCI bits (7...1706). Set it to
    %                        0 for no CG-UCI transmission (default 7)
    %   BetaOffsetCGUCI    - Beta offset for CG-UCI (default 20)
    %   DataSourceCGUCI    - Source of CG-UCI contents
    %                        (PN or custom) (default 'PN9-ITU')
    %   EnableULSCH        - Flag turning UL-SCH for UCI transmission on
    %                        PUSCH on or off (false, true (default))
    %   UCIScaling         - Scaling factor to limit the number of resource
    %                        elements for UCI on PUSCH (0.5, 0.65, 0.8, 1 (default))
    %   DMRS               - PUSCH-specific DM-RS configuration object, as
    %                        described in <a href="matlab:
    %                        help('nrPUSCHDMRSConfig')">nrPUSCHDMRSConfig</a>
    %   DMRSPower          - Scaling of PUSCH DM-RS power in dB (default 0)
    %   EnablePTRS         - Enable or disable the PT-RS configuration (0 (default), 1)
    %   PTRS               - PUSCH-specific PT-RS configuration object, as
    %                        described in <a href="matlab:
    %                        help('nrPUSCHPTRSConfig')">nrPUSCHPTRSConfig</a>
    %   PTRSPower          - Scaling of PUSCH PT-RS power in dB (default 0)
    %
    %   nrWavegenPUSCHConfig properties (read-only):
    %
    %   NumCodewords       - Number of codewords
    %   TransportBlockSize - Size of the transport block(s)
    %
    %   nrWavegenPUSCHConfig methods:
    %
    %   nrTBS - Transport block size(s) associated with transmission
    %
    %   Example 1:
    %   %  Create a custom nrWavegenPUSCHConfig object, pass it to nrULCarrierConfig
    %
    %   PUSCH = nrWavegenPUSCHConfig('BandwidthPartID', 0, ...
    %                    'Modulation', '16QAM', 'TargetCodeRate', 658/1024, ...
    %                    'SymbolAllocation', [0 7], 'SlotAllocation', [0 2], 'Period', 3, ...
    %                    'PRBSet', 0:20, 'EnablePTRS', true);
    %
    %   cfg = nrULCarrierConfig;
    %   cfg.PUSCH = {PUSCH};
    %
    %   Example 2:
    %   %  Create 2 PUSCH configurations for 2 different bandwidth parts, pass these to nrULCarrierConfig
    %
    %   carrier1 = nrSCSCarrierConfig('SubcarrierSpacing', 15);
    %   carrier2 = nrSCSCarrierConfig('SubcarrierSpacing', 30);
    %   bwp1 = nrWavegenBWPConfig('BandwidthPartID', 0, 'SubcarrierSpacing', 15);
    %   bwp2 = nrWavegenBWPConfig('BandwidthPartID', 1, 'SubcarrierSpacing', 30);
    %   PUSCH1 = nrWavegenPUSCHConfig('RNTI', 1, 'BandwidthPartID', 0, 'Modulation', 'QPSK');
    %   PUSCH2 = nrWavegenPUSCHConfig('RNTI', 2, 'BandwidthPartID', 1, 'Modulation', '16QAM');
    %
    %   cfg = nrULCarrierConfig;
    %   cfg.SCSCarriers = {carrier1, carrier2};
    %   cfg.BandwidthParts = {bwp1, bwp2};
    %   cfg.PUSCH = {PUSCH1, PUSCH2};
    %
    %   See also nrULCarrierConfig, nrWaveformGenerator.
    
    %   Copyright 2020-2025 The MathWorks, Inc.
    
    %#codegen
    properties
        %NID Physical uplink shared channel scrambling identity
        % Specify the physical uplink shared channel scrambling identity as
        % a scalar nonnegative integer. The value must be in the range
        % 0...1023. It is the dataScramblingIdentityPUSCH (0...1023) if
        % configured, else it is the physical layer cell identity
        % (0...1007). Use empty ([]) to make this property equal to the
        % <a href="matlab:help('nrULCarrierConfig/NCellID')"
        % >NCellID</a> property of nrULCarrierConfig. The default value is [].
        NID = [];
        
        %EnableACK Enable HARQ-ACK for UCI on PUSCH
        % Specify EnableACK as a logical scalar. This flag determines the
        % presence of HARQ-ACK transmission on PUSCH. When set to false, no
        % HARQ-ACK transmission is enabled on PUSCH. This property applies
        % when Coding is true. The default is false.
        EnableACK (1,1) logical = false;
        
        %NumACKBits Number of HARQ-ACK bits in UCI on PUSCH
        % Specify the number of HARQ-ACK bits in UCI on PUSCH as a scalar
        % nonnegative integer up to 1706. For no HARQ-ACK transmission, set
        % the value to 0. This property applies when EnableACK is true. The
        % default is 10.
        NumACKBits (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(NumACKBits, 1706)} = 10;
        
        %DataSourceACK Source of HARQ-ACK contents
        % Specify DataSourceACK as one of {'PN9-ITU', 'PN9', 'PN11',
        % 'PN15', 'PN23'}, as a cell array containing one of the
        % abovementioned options and a numeric scalar that is the random
        % seed (for example, {'PN9',7}), or as a binary vector. If the
        % seed is not specified, then all shift registers are initialized
        % with an active state. This property applies when EnableACK is
        % true. The default is 'PN9-ITU'.
        DataSourceACK = 'PN9-ITU';
        
        %EnableCSI1 Enable CSI part 1 for UCI on PUSCH
        % Specify EnableCSI1 as a logical scalar. This flag determines the
        % presence of CSI part 1 transmission on PUSCH. When set to false,
        % no CSI part 1 transmission is enabled on PUSCH. This property
        % applies when Coding is true. The default is false.
        EnableCSI1 (1,1) logical = false;
        
        %NumCSI1Bits Number of CSI part 1 bits in UCI on PUSCH
        % Specify the number of CSI part 1 bits in UCI on PUSCH as a
        % scalar nonnegative integer up to 1706. For no CSI part 1
        % transmission, set the value to 0. This property applies when
        % EnableCSI1 is true. The default is 10.
        NumCSI1Bits (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(NumCSI1Bits, 1706)} = 10;
        
        %DataSourceCSI1 Source of CSI part 1 contents
        % Specify DataSourceCSI1 as one of {'PN9-ITU', 'PN9', 'PN11',
        % 'PN15', 'PN23'}, as a cell array containing one of the
        % abovementioned options and a numeric scalar that is the random
        % seed (for example, {'PN9',7}), or as a binary vector. If the
        % seed is not specified, then all shift registers are initialized
        % with an active state. This property applies when EnableCSI1 is
        % true and NumCSI1Bits > 0. The default is 'PN9-ITU'.
        DataSourceCSI1 = 'PN9-ITU';
        
        %EnableCSI2 Enable CSI part 2 for UCI on PUSCH
        % Specify EnableCSI2 as a logical scalar. This flag determines the
        % presence of CSI part 2 transmission on PUSCH. When set to false,
        % no CSI part 2 transmission is enabled on PUSCH. This property
        % applies when EnableCSI1 is true. This property applies when
        % Coding is true and CSI part 1 is active. The default is false.
        EnableCSI2 (1,1) logical = false;
        
        %NumCSI2Bits Number of CSI part 2 bits in UCI on PUSCH
        % Specify the number of CSI part 2 bits in UCI on PUSCH as a
        % scalar nonnegative integer up to 1706. For no CSI part 2
        % transmission, set the value to 0. The value is ignored when there
        % are no CSI part 1 bits. This property applies when CSI part 1 is
        % active and EnableCSI2 is true. The default is 10.
        NumCSI2Bits (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(NumCSI2Bits, 1706)} = 10;
        
        %DataSourceCSI2 Source of CSI part 2 contents
        % Specify DataSourceCSI2 as one of {'PN9-ITU', 'PN9', 'PN11',
        % 'PN15', 'PN23'}, as a cell array containing one of the
        % abovementioned options and a numeric scalar that is the random
        % seed (for example, {'PN9',7}), or as a binary vector. If the
        % seed is not specified, then all shift registers are initialized
        % with an active state. This property applies when CSI part 1 is
        % active, EnableCSI2 is true, and NumCSI2Bits > 0. The default is
        % 'PN9-ITU'.
        DataSourceCSI2 = 'PN9-ITU';
        
        %EnableCGUCI Enable CG-UCI for UCI on PUSCH
        % Specify EnableCGUCI as a logical scalar. This flag determines the
        % presence of CG-UCI transmission on PUSCH. When set to false, no
        % CG-UCI transmission is enabled on PUSCH. This property applies
        % when Coding is true. The default is false.
        EnableCGUCI (1,1) logical = false;
        
        %NumCGUCIBits Number of CG-UCI bits in UCI on PUSCH
        % Specify the number of CG-UCI bits in UCI on PUSCH as a scalar
        % integer in 7...1706. For no CG-UCI transmission, set the value to
        % 0. This property applies when EnableCGUCI is true. The default is 7.
        NumCGUCIBits (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(NumCGUCIBits, 1706), validateNumCGUCIBits(NumCGUCIBits)} = 7;
        
        %BetaOffsetCGUCI Beta offset for CG-UCI
        % Specify the beta offset for CG-UCI as a real scalar positive
        % value. If both HARQ-ACK and CG-UCI are active, the value of
        % BetaOffsetACK is used instead. This property applies when
        % EnableCGUCI is true and NumCGUCIBits > 0. The default value is 20.
        BetaOffsetCGUCI (1,1) {mustBeNumeric, mustBeReal, mustBePositive, mustBeFinite} = 20;
        
        %DataSourceCGUCI Source of CG-UCI contents
        % Specify DataSourceCGUCI as one of {'PN9-ITU', 'PN9', 'PN11',
        % 'PN15', 'PN23'}, as a cell array containing one of the
        % abovementioned options and a numeric scalar that is the random
        % seed (for example, {'PN9',7}), or as a binary vector. If the
        % seed is not specified, then all shift registers are initialized
        % with an active state. This property applies when EnableCGUCI is
        % true and NumCGUCIBits > 0. The default is 'PN9-ITU'.
        DataSourceCGUCI = 'PN9-ITU';
        
        %EnableULSCH Enable UL-SCH for UCI on PUSCH
        % Specify EnableULSCH as a logical scalar. This flag determines the
        % presence of UL-SCH transmission on the slots used for UCI on
        % PUSCH. When set to true, UL-SCH and UCI are multiplexed together.
        % This property applies when at least one UCI source is active.
        % The default is true.
        EnableULSCH (1,1) logical = true;
        
        %PTRSPower Power scaling of the PT-RS in dB
        % Specify PTRSPower in dB as a real scalar. The power of the PT-RS
        % within the PUSCH is scaled within the 5G waveform according to
        % this value. This scaling is additional to the channel-wide power
        % scaling determined by the Power property. This value is not used
        % if transform precoding is enabled for this PUSCH. The default is
        % 0 dB.
        PTRSPower (1,1) {mustBeNumeric, mustBeReal, mustBeFinite} = 0;

    end

    properties (Dependent,SetAccess=private)
        %TransportBlockSize Transport block sizes
        % 
        TransportBlockSize;
    end

    properties (Constant,Hidden)
        MCSTable_Values = {'qam64','qam256'};
    end
    
    properties (Hidden)
        DataSourceACK_Values = nr5g.internal.wavegen.DataSourceCommon.DataSource_Options;
        DataSourceCSI1_Values = nr5g.internal.wavegen.DataSourceCommon.DataSource_Options;
        DataSourceCSI2_Values = nr5g.internal.wavegen.DataSourceCommon.DataSource_Options;
        DataSourceCGUCI_Values = nr5g.internal.wavegen.DataSourceCommon.DataSource_Options;
        CustomPropList = {'Enable', 'Label', 'Power', 'BandwidthPartID', ...
            'Modulation','NumLayers','MappingType', ...
            'SymbolAllocation',  'SlotAllocation', 'Period', 'PRBSet',...
            'TransformPrecoding', 'TransmissionScheme', 'NumAntennaPorts', ...
            'TPMI', 'CodebookType', 'FrequencyHopping', 'SecondHopStartPRB', ...
            'Interlacing','RBSetIndex','InterlaceIndex',...
            'AntennaMapping', 'PrecodingMatrix', ...
            'NID', 'RNTI', 'NRAPID', 'Coding', 'TargetCodeRate', 'XOverhead', ...
            'LimitedBufferRateMatching', 'MaxNumLayers', 'MCSTable', 'RVSequence', 'DataSource', ...
            'EnableACK', 'NumACKBits', 'BetaOffsetACK', 'DataSourceACK', ...
            'EnableCSI1', 'NumCSI1Bits', 'BetaOffsetCSI1', 'DataSourceCSI1', ...
            'EnableCSI2', 'NumCSI2Bits', 'BetaOffsetCSI2', 'DataSourceCSI2', ...
            'EnableCGUCI', 'NumCGUCIBits', 'BetaOffsetCGUCI', 'DataSourceCGUCI', ...
            'EnableULSCH', 'UCIScaling', ...
            'DMRS', 'DMRSPower', 'EnablePTRS', 'PTRS', 'PTRSPower','NumCodewords','TransportBlockSize'};
    end
    
    properties (Dependent = true, Hidden = true)
        NumColumns
        Wpa
    end
    
    methods

        % Constructor
        function obj = nrWavegenPUSCHConfig(varargin)
            % Get the value of NID from the name-value pairs
            nid = nr5g.internal.parseProp('NID',[],varargin{:});
            % Get the value of NRAPID from the name-value pairs
            nrapid = nr5g.internal.parseProp('NRAPID',[],varargin{:});
            % Get the value of LimitedBufferRateMatching from the
            % name-value pairs
            lbrm = nr5g.internal.parseProp('LimitedBufferRateMatching',false,varargin{:});
            
            % Support name-value pair arguments when constructing object
            obj@nr5g.internal.pusch.ConfigBase( ...
                'Label', 'PUSCH1', ...
                'NID', nid, ...
                'NRAPID', nrapid, ...
                varargin{:});

            obj.LimitedBufferRateMatching = lbrm;
        end
        
        % Self-validate and set properties
        function obj = set.NID(obj,val)
            prop = 'NID';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'}, ...
                    {'scalar','integer','nonnegative','<=',1023}, ...
                    [class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end
        
        function obj = set.DataSourceACK(obj,val)
            prop = 'DataSourceACK';
            temp = getDataSource(obj, prop, val);
            obj.(prop) = temp;
        end
        
        function obj = set.DataSourceCSI1(obj,val)
            prop = 'DataSourceCSI1';
            temp = getDataSource(obj, prop, val);
            obj.(prop) = temp;
        end
        
        function obj = set.DataSourceCSI2(obj,val)
            prop = 'DataSourceCSI2';
            temp = getDataSource(obj, prop, val);
            obj.(prop) = temp;
        end
        
        function obj = set.DataSourceCGUCI(obj,val)
            prop = 'DataSourceCGUCI';
            temp = getDataSource(obj, prop, val);
            obj.(prop) = temp;
        end
        
        function tbs = get.TransportBlockSize(obj)
            if ~obj.Interlacing
                tbs = nrTBS(obj);
            else
                tbs = [];
            end
        end

        function tbs = nrTBS(obj,varargin)
            % Signatures supported are,
            % nrTBS(wavegenpusch)
            % nrTBS(wavegenpusch,ulcarrier) - Mandatory in the interlaced PRB allocation case
            narginchk(1,2);
    
            if nargin == 2
                validateattributes(varargin{1},"nrULCarrierConfig","scalar","nrTBS","CARRIER");
            end

            interlacing = obj.Interlacing;
            if interlacing
    
                % Check input signature for nrTBS(channel,carrier:nrULCarrierConfig) syntax
                errFlag = interlacing && nargin < 2;
                coder.internal.errorIf(errFlag,'nr5g:nrTBS:InvalidSigForInterlacedPUSCHWavegen');
    
                ulcarrier = varargin{1};
                
                % Find the BWP used by the PUSCH
                associatedBWP = 0;
                for i = 1:numel(ulcarrier.BandwidthParts)
                    if obj.BandwidthPartID == ulcarrier.BandwidthParts{i}.BandwidthPartID
                         associatedBWP = i;
                         break;
                    end
                end
                coder.internal.errorIf(associatedBWP == 0,'nr5g:nrTBS:CHNotInBWP',obj.BandwidthPartID);
    
                % Find the SCS carrier associated with the BWP for the PRB dimensions
                associatedCarrier = 0;
                scs = ulcarrier.BandwidthParts{associatedBWP}.SubcarrierSpacing;
                for j = 1:numel(ulcarrier.SCSCarriers)
                    if scs == ulcarrier.SCSCarriers{j}.SubcarrierSpacing
                         associatedCarrier = j;
                         break;
                    end
                end
               % Instead of a full blown ulcarrier.validateConfig call
               coder.internal.errorIf(associatedCarrier == 0, ...
                        'nr5g:nrWaveformGenerator:BWP2SCSLinkBroken', obj.BandwidthPartID, scs);
    
                % nrCarrierConfig proxy
                carriermvparams =  struct(...
                    'NStartGrid',ulcarrier.SCSCarriers{associatedCarrier}.NStartGrid, ...
                    'NSizeGrid',ulcarrier.SCSCarriers{associatedCarrier}.NSizeGrid, ...
                    'SubcarrierSpacing',scs, ...
                    'IntraCellGuardBands',{ulcarrier.IntraCellGuardBands});
    
                % nrPUSCHConfig proxy
                channelmvparams = struct('NStartBWP',ulcarrier.BandwidthParts{associatedBWP}.NStartBWP,...
                                         'NSizeBWP',ulcarrier.BandwidthParts{associatedBWP}.NSizeBWP,...
                                         'RBSetIndex',obj.RBSetIndex,'InterlaceIndex',obj.InterlaceIndex);
    
                % Dispatch to interlaced PUSCH specific function to include the PRB allocation for this case
                tbs = getTBSEntryInterlaced(obj,obj.TargetCodeRate,obj.XOverhead,channelmvparams,carriermvparams);
            else
                tbs = nr5g.internal.TBSDetermination.getTBSEntry(obj,obj.TargetCodeRate,obj.XOverhead,1);
            end
        end

        % Validate configuration
        function validateConfig(obj)
            
            % Call PUSCH ConfigBase validator
            validateConfig@nr5g.internal.pusch.ConfigBase(obj);

            % Check Power
            validatePower(obj,obj.SlotAllocation,obj.Period);
            
            if ~isempty(obj.SymbolAllocation) && obj.SymbolAllocation(2) > 0
                % For mapping type 'A', ensure that the PUSCH allocation starts
                % with symbol 0 and has minimum length of 4, as per TS 38.214,
                % Section 6.1.2
                flag = strcmpi(obj.MappingType,'A') && (obj.SymbolAllocation(1) || obj.SymbolAllocation(2) < 4);
                coder.internal.errorIf(flag,'nr5g:nrWaveformGenerator:InvalidSymbAllocMappingTypeA',obj.SymbolAllocation(1),obj.SymbolAllocation(2));
            end
            
            % For PUSCH with mapping type B, intra-slot frequency hopping,
            % and less than 3 symbols, there is no resource element
            % available for UCI transmission
            ackFlag = obj.Coding && obj.EnableACK && (obj.NumACKBits>0);
            csi1Flag = obj.Coding && obj.EnableCSI1 && (obj.NumCSI1Bits>0);
            csi2Flag = csi1Flag && obj.EnableCSI2 && (obj.NumCSI2Bits>0);
            cguciFlag = obj.Coding && obj.EnableCGUCI && (obj.NumCGUCIBits>0);
            numSymbols = 0;
            if ~isempty(obj.SymbolAllocation)
                numSymbols = obj.SymbolAllocation(2);
            end
            flag = (ackFlag || csi1Flag || cguciFlag) && strcmpi(obj.MappingType,'B') && ...
                (strcmpi(obj.FrequencyHopping,'intraSlot') && numSymbols < 3 && ~obj.Interlacing);
            coder.internal.errorIf(flag, 'nr5g:nrWaveformGenerator:InvalidUCIOnPUSCHMapTypeB', ...
                numSymbols);
            
            % The total UCI payload size must be at most 1706 bits, as
            % per TS 38.212, Section 5.2.1. Verify this only for the active
            % UCI sources.
            uciBits = (ackFlag * double(obj.NumACKBits)) + (csi1Flag * double(obj.NumCSI1Bits)) + ...
                      (csi2Flag * double(obj.NumCSI2Bits)) + (cguciFlag * double(obj.NumCGUCIBits));
            coder.internal.errorIf(uciBits>1706,'nr5g:nrWaveformGenerator:InvalidUCIOnPUSCHPayloadSize',uciBits);
        end

        function numCols = get.NumColumns(obj)

            if strcmpi(obj.TransmissionScheme,'codebook')
                np = obj.NumAntennaPorts;
            else
                np = obj.NumLayers;
            end
            numCols = obj.getNumCols(np);

        end

        function P = get.Wpa(obj)

            if strcmpi(obj.TransmissionScheme,'codebook')
                % For codebook PUSCH, ignore non-empty precoding matrix
                P = obj.calculatePrecodeAndMapMatrix(obj.NumAntennaPorts,[]);
            else
                P = obj.calculatePrecodeAndMapMatrix(obj.NumLayers);
            end

    end
    
    end
    
    methods (Access=protected)
        % Controls the conditional display of properties
        function flag = isInactiveProperty(obj, prop)
            % Call PXSCHConfigBase method
            flag = isInactiveProperty@nr5g.internal.wavegen.PXSCHConfigBase(obj, prop);
            
            % Call PUSCH config base method
            flag = flag || isInactiveProperty@nr5g.internal.pusch.ConfigBase(obj, prop);

            % NumAntennaPorts and TPMI only if TransmissionScheme is 'codebook'
            if any(strcmp(prop,{'NumAntennaPorts', 'TPMI'}))
                flag = ~strcmpi(obj.TransmissionScheme,'codebook');
            end

            % CodebookType only if TransmissionScheme is 'codebook' and
            % NumAntennaPorts is 8
            if any(strcmp(prop,{'CodebookType'}))
                flag = ~(strcmpi(obj.TransmissionScheme,'codebook') && obj.NumAntennaPorts==8);
            end
            
            % SecondHopStartPRB only if FrequencyHopping is not 'neither'
            if strcmp(prop,'SecondHopStartPRB')
                flag = strcmpi(obj.FrequencyHopping,'neither') || obj.Interlacing;
            end
            
            % PTRSPower only if EnablePTRS is 1 and TransformPrecoding is 0
            if strcmp(prop,'PTRSPower')
                flag = ~(obj.EnablePTRS && ~obj.TransformPrecoding);
            end
            
            % UCI-on-PUSCH properties
            
            % EnableACK, EnableCSI1, and EnableCGUCI only if Coding is 1
            if any(strcmp(prop,{'EnableACK', 'EnableCSI1', 'EnableCGUCI'}))
                flag = ~obj.Coding;
            end
            
            % NumACKBits only if Coding is 1 and EnableACK is 1
            if strcmp(prop, 'NumACKBits')
                flag = ~(obj.Coding && obj.EnableACK);
            end
            
            % BetaOffsetACK and DataSourceACK only if Coding is 1,
            % EnableACK is 1, and NumACKBits > 0
            if any(strcmp(prop, {'BetaOffsetACK', 'DataSourceACK'}))
                flag = ~(obj.Coding && obj.EnableACK && obj.NumACKBits);
            end
            
            % NumCSI1Bits only if Coding is 1 and EnableCSI1 is 1
            if strcmp(prop, 'NumCSI1Bits')
                flag = ~(obj.Coding && obj.EnableCSI1);
            end
            
            % BetaOffsetCSI1, DataSourceCSI1 and EnableCSI2 only if Coding
            % is 1, EnableCSI1 is 1, and NumCSI1Bits > 0
            if any(strcmp(prop, {'BetaOffsetCSI1', 'DataSourceCSI1', 'EnableCSI2'}))
                flag = ~(obj.Coding && obj.EnableCSI1 && obj.NumCSI1Bits);
            end
            
            % NumCSI2Bits only if Coding is 1, EnableCSI1 and
            % EnableCSI2 are 1, and NumCSI1Bits > 0
            if strcmp(prop, 'NumCSI2Bits')
                flag = ~(obj.Coding && obj.EnableCSI1 && obj.EnableCSI2 && obj.NumCSI1Bits);
            end
            
            % BetaOffsetCSI2 and DataSourceCSI2 only if Coding is 1,
            % EnableCSI1 and EnableCSI2 are 1, NumCSI1Bits > 0, and
            % NumCSI2Bits > 0
            if any(strcmp(prop, {'BetaOffsetCSI2', 'DataSourceCSI2'}))
                flag = ~(obj.Coding && obj.EnableCSI1 && obj.EnableCSI2 && ...
                         obj.NumCSI1Bits && obj.NumCSI2Bits);
            end
            
            % NumCGUCIBits only if Coding is 1 and EnableCGUCI is 1
            if strcmp(prop, 'NumCGUCIBits')
                flag = ~(obj.Coding && obj.EnableCGUCI);
            end
            
            % DataSourceCGUCI only if Coding is 1, EnableCGUCI is 1, and
            % NumCGUCIBits > 0
            if strcmp(prop, 'DataSourceCGUCI')
                flag = ~(obj.Coding && obj.EnableCGUCI && obj.NumCGUCIBits);
            end
            
            % BetaOffsetCGUCI only if Coding is 1, EnableCGUCI is 1,
            % NumCGUCIBits > 0, and EnableACK is 0 or NumACKBits = 0
            if strcmp(prop, 'BetaOffsetCGUCI')
                flag = ~(obj.Coding && obj.EnableCGUCI && obj.NumCGUCIBits && ...
                    (~obj.EnableACK || obj.NumACKBits==0));
            end
            
            % EnableULSCH and UCIScaling only if Coding is 1 and at
            % least one UCI source is active
            if any(strcmpi(prop, {'EnableULSCH', 'UCIScaling'}))
                flag = ~(obj.Coding && ...
                        ((obj.EnableACK && obj.NumACKBits) || ...
                         (obj.EnableCSI1 && obj.NumCSI1Bits) || ...
                         (obj.EnableCGUCI && obj.NumCGUCIBits)));
            end

            % LimitedBufferRateMatching only if Coding is 1
            if strcmpi(prop,'LimitedBufferRateMatching')
                flag = ~obj.Coding;
            end

            % MaxNumLayers and MCSTable only if Coding and
            % LimitedBufferRateMatching are 1
            if any(strcmp(prop,{'MaxNumLayers','MCSTable'}))
                flag = ~(obj.Coding && obj.LimitedBufferRateMatching);
            end

            % Do not display TBS in the interlaced PUSCH case because the TBS
            % calculation required additional parameters to be supplied
            % outside of the PUSCH
            if strcmp(prop,'TransportBlockSize')
                flag = obj.Interlacing;
            end

            % PrecodingMatrix is only applicable when TransmissionScheme is
            % 'nonCodebook'
            if strcmpi(prop,'PrecodingMatrix')
                flag = strcmpi(obj.TransmissionScheme,'codebook');
            end

        end
    end
end

% File local functions

function validateNumCGUCIBits(val)
    coder.internal.errorIf(val>0 && val<7, 'nr5g:nrWaveformGenerator:InvalidNumCGUCIBits',val);
end
