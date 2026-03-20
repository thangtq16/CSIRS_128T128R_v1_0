classdef nrWavegenPUCCH1Config < nr5g.internal.wavegen.EnablePower & nr5g.internal.wavegen.PUCCHConfigBase & ...
                                 nr5g.internal.pucch.Format1ConfigBase & ...
                                 nr5g.internal.wavegen.DMRSPowerCommon
    %nrWavegenPUCCH1Config PUCCH format 1 configuration object for 5G waveform generation
    %   PUCCH = nrWavegenPUCCH1Config creates a physical uplink control
    %   channel (PUCCH) configuration object. This object contains all the
    %   parameters of PUCCH format 1.
    %
    %   The default nrWavegenPUCCH1Config object configures a PUCCH format
    %   1 allocated in the first resource block and spanning over 14 OFDM
    %   symbols in a slot, and transmission in all slots. The PUCCH carries
    %   a single uplink control information (UCI) bit.
    %
    %   PUCCH = nrWavegenPUCCH1Config(Name,Value) creates a physical uplink
    %   control channel configuration object, PUCCH, with the specified
    %   property Name set to the specified Value. You can specify
    %   additional name-value pair arguments in any order as
    %   (Name1,Value1,...,NameN,ValueN).
    %
    %   nrWavegenPUCCH1Config properties:
    %
    %   Enable             - Flag turning this PUCCH on or off (default true)
    %   Label              - Alphanumeric description for this PUCCH
    %                        (default 'PUCCH format 1')
    %   Power              - Power scaling in dB (default 0)
    %   BandwidthPartID    - ID of bandwidth part containing this PUCCH
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
    %   InitialCyclicShift - Initial cyclic shift (0...11) (default 0)
    %   OCCI               - Orthogonal cover code index (0...6) (default 0)
    %   NumUCIBits         - Number of UCI bits (0...2) (default 1)
    %   DataSourceUCI      - Source of UCI contents
    %                        (pseudo-noise (PN) or custom) (default 'PN9-ITU')
    %   DMRSPower          - Scaling of PUCCH DM-RS power in dB
    %
    %   Example: 
    %   % Display the default properties of PUCCH format 1, in case of
    %   % no frequency hopping.
    %
    %   pucch = nrWavegenPUCCH1Config;
    %   pucch.FrequencyHopping = 'neither'
    %
    %   See also nrULCarrierConfig, nrWaveformGenerator,
    %   nrWavegenPUCCH0Config, nrWavegenPUCCH2Config,
    %   nrWavegenPUCCH3Config, nrWavegenPUCCH4Config.

    %   Copyright 2021-2024 The MathWorks, Inc.

    %#codegen
    
    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'Enable', 'Label', 'Power', 'BandwidthPartID', ...
            'SymbolAllocation', 'SlotAllocation', 'Period', 'PRBSet', ...
            'FrequencyHopping', 'SecondHopStartPRB', 'Interlacing',...
            'RBSetIndex', 'InterlaceIndex', 'GroupHopping', 'HoppingID',...
            'AntennaMapping', 'PrecodingMatrix', ...
            'InitialCyclicShift', 'OCCI', 'NumUCIBits', 'DataSourceUCI', ...
            'DMRSPower'};
    end

    methods
        % Constructor
        function obj = nrWavegenPUCCH1Config(varargin)
            obj = obj@nr5g.internal.pucch.Format1ConfigBase(...
                  'Label', 'PUCCH format 1', ...
                  varargin{:});
        end
        
        % Validate configuration
        function validateConfig(obj)
            % Call Format1ConfigBase validator
            validateConfig@nr5g.internal.pucch.Format1ConfigBase(obj);
            % Check that NumUCIBits<=2
            coder.internal.errorIf(obj.NumUCIBits > 2,'nr5g:nrWaveformGenerator:InvalidUCIBits01','1',obj.NumUCIBits);
            % Check Power
            validatePower(obj,obj.SlotAllocation,obj.Period);
        end
    end

    methods (Access=protected)
        % Controls the conditional display of properties
        function inactive = isInactiveProperty(obj, prop)
            % Call base class methods
            inactive = isInactiveProperty@nr5g.internal.wavegen.PUCCHConfigBase(obj, prop);
            inactive = inactive || isInactiveProperty@nr5g.internal.pucch.Format1ConfigBase(obj, prop);
        end
    end
end
