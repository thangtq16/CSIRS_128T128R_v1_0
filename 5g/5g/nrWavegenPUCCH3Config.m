classdef nrWavegenPUCCH3Config < nr5g.internal.wavegen.EnablePower & ...
                                 nr5g.internal.pucch.Format3ConfigBase & ...
                                 nr5g.internal.wavegen.PUCCH34Common
    %nrWavegenPUCCH3Config PUCCH format 3 configuration object for 5G waveform generation
    %   PUCCH = nrWavegenPUCCH3Config creates a physical uplink control
    %   channel (PUCCH) configuration object. This object contains all the
    %   parameters of PUCCH format 3.
    %
    %   The default nrWavegenPUCCH3Config object configures a PUCCH format
    %   3 allocated in the first resource block and spanning over 14 OFDM
    %   symbols in a slot, and transmission in all slots. The PUCCH carries
    %   10 uplink control information (UCI) part 1 bits and no UCI part 2
    %   bits.
    %
    %   PUCCH = nrWavegenPUCCH3Config(Name,Value) creates a physical uplink
    %   control channel configuration object, PUCCH, with the specified
    %   property Name set to the specified Value. You can specify
    %   additional name-value pair arguments in any order as
    %   (Name1,Value1,...,NameN,ValueN).
    %
    %   nrWavegenPUCCH3Config properties:
    %
    %   Enable             - Flag turning this PUCCH on or off (default true)
    %   Label              - Alphanumeric description for this PUCCH
    %                        (default 'PUCCH format 3')
    %   Power              - Power scaling in dB (default 0)
    %   BandwidthPartID    - ID of bandwidth part containing this PUCCH (default 1)
    %   Modulation         - Modulation scheme ('QPSK' (default), 'pi/2-BPSK')
    %   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
    %                        (default [0 14])
    %   SlotAllocation     - Time-domain location of PUCCH (in slots) (default 0:9)
    %   Period             - Period of slot allocation (default 10)
    %   PRBSet             - PRBs allocated for PUCCH within the BWP
    %                        (default 0)
    %   FrequencyHopping   - Frequency hopping configuration
    %                        ('intraSlot', 'interSlot', 'neither' (default))
    %   SecondHopStartPRB  - Starting PRB of second hop relative to the
    %                        BWP (0...274) (default 1)
    %   Interlacing        - Enable interlacing (default false)
    %   RBSetIndex         - Resource block set index (default 0)
    %   InterlaceIndex     - Interlace indices (0...9) (default 0)
    %   GroupHopping       - Group hopping configuration
    %                        ('enable', 'disable', 'neither' (default))
    %   HoppingID          - Hopping identity (0...1023) (default [])
    %   SpreadingFactor    - Spreading factor (1, 2 (default), 4)
    %   OCCI               - Orthogonal cover code index (0...3) (default 0)
    %   NID                - Data scrambling identity (0...1023) (default [])
    %   RNTI               - Radio network temporary identifier (0...65535)
    %                        (default 0)
    %   Coding             - Flag to enable channel coding
    %   TargetCodeRate     - Target code rate (default 0.15)
    %   NumUCIBits         - Number of UCI part 1 bits (0...1706) (default 10)
    %   DataSourceUCI      - Source of UCI part 1 contents
    %                        (pseudo-noise (PN) or custom) (default 'PN9-ITU')
    %   NumUCI2Bits        - Number of UCI part 2 bits (0...1706) (default 0)
    %   DataSourceUCI2     - Source of UCI part 2 contents (PN or custom) (default 'PN9-ITU')
    %   NID0               - Scrambling identity for demodulation reference
    %                        signal (DM-RS) (0...65535) (default [])
    %   DMRSUplinkTransformPrecodingR16 - Enable low PAPR DM-RS sequence for
    %                        DFT-s-OFDM (0 (default), 1). To enable this
    %                        property, set the Modulation property to 'pi/2-BPSK'.
    %   AdditionalDMRS     - Additional demodulation reference signal (DM-RS)
    %                        configuration flag (0 (default), 1)
    %   DMRSPower          - Scaling of PUCCH DM-RS power in dB (default 0)
    %
    %   Example: 
    %   % Display the default properties of PUCCH format 3, in case of
    %   % no frequency hopping.
    %
    %   pucch = nrWavegenPUCCH3Config;
    %   pucch.FrequencyHopping = 'neither'
    %
    %   See also nrULCarrierConfig, nrWaveformGenerator,
    %   nrWavegenPUCCH0Config, nrWavegenPUCCH1Config,
    %   nrWavegenPUCCH2Config, nrWavegenPUCCH4Config.

    %   Copyright 2021-2024 The MathWorks, Inc.

    %#codegen
    
    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'Enable', 'Label', 'Power', 'BandwidthPartID', ...
            'Modulation', 'SymbolAllocation', 'SlotAllocation', 'Period', ...
            'PRBSet', 'FrequencyHopping', 'SecondHopStartPRB', 'Interlacing', ...
            'RBSetIndex', 'InterlaceIndex', 'GroupHopping', 'HoppingID', ...
            'SpreadingFactor', 'OCCI', ...
            'AntennaMapping', 'PrecodingMatrix', 'NID', 'RNTI', 'Coding', ...
            'TargetCodeRate', 'NumUCIBits', 'DataSourceUCI', ...
            'NumUCI2Bits', 'DataSourceUCI2', ...
            'NID0', 'DMRSUplinkTransformPrecodingR16', 'AdditionalDMRS', 'DMRSPower'};
    end

    methods
        % Constructor
        function obj = nrWavegenPUCCH3Config(varargin)
            obj = obj@nr5g.internal.pucch.Format3ConfigBase(...
                  'Label', 'PUCCH format 3', ...
                  'NumUCIBits', 10, ...
                  varargin{:});
        end
        
        % Validate configuration
        function validateConfig(obj)
            % Call Format3ConfigBase validator
            validateConfig@nr5g.internal.pucch.Format3ConfigBase(obj);
            % Call PUCCH34Common validator
            validateConfig@nr5g.internal.wavegen.PUCCH34Common(obj,'3');
            % Check Power
            validatePower(obj,obj.SlotAllocation,obj.Period);
        end
    end
    
    methods (Access=protected)
        % Controls the conditional display of properties
        function inactive = isInactiveProperty(obj, prop)
            % Call PUCCHConfigBase and PUCCH34Common methods
            inactive = isInactiveProperty@nr5g.internal.wavegen.PUCCH34Common(obj, prop);
            inactive = inactive || isInactiveProperty@nr5g.internal.pucch.Format3ConfigBase(obj, prop);
        end
    end
end