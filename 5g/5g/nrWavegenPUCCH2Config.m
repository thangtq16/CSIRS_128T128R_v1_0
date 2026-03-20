classdef nrWavegenPUCCH2Config < nr5g.internal.wavegen.EnablePower & nr5g.internal.wavegen.PUCCHConfigBase & ...
                                 nr5g.internal.pucch.Format2ConfigBase & ...
                                 nr5g.internal.wavegen.CodingCommon & nr5g.internal.wavegen.DMRSPowerCommon
    %nrWavegenPUCCH2Config PUCCH format 2 configuration object for 5G waveform generation
    %   PUCCH = nrWavegenPUCCH2Config creates a physical uplink control
    %   channel (PUCCH) configuration object. This object contains all the
    %   parameters of PUCCH format 2.
    %
    %   The default nrWavegenPUCCH2Config object configures a PUCCH format
    %   2 allocated in the first resource block and the last OFDM symbol in
    %   the slot of 14 OFDM symbols, and transmission in all slots. The
    %   PUCCH carries 10 uplink control information (UCI) bits.
    %
    %   PUCCH = nrWavegenPUCCH2Config(Name,Value) creates a physical uplink
    %   control channel configuration object, PUCCH, with the specified
    %   property Name set to the specified Value. You can specify
    %   additional name-value pair arguments in any order as
    %   (Name1,Value1,...,NameN,ValueN).
    %
    %   nrWavegenPUCCH2Config properties:
    %
    %   Enable             - Flag turning this PUCCH on or off (default true)
    %   Label              - Alphanumeric description for this PUCCH
    %                        (default 'PUCCH format 2')
    %   Power              - Power scaling in dB (default 0)
    %   BandwidthPartID    - ID of bandwidth part containing this PUCCH (default 1)
    %   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
    %                        (default [13 1])
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
    %   SpreadingFactor    - Spreading factor (2 (default), 4)
    %   OCCI               - Orthogonal cover code index (0...3) (default 0)
    %   NID                - Data scrambling identity (0...1023) (default [])
    %   RNTI               - Radio network temporary identifier (0...65535)
    %                        (default 0)
    %   Coding             - Flag to enable channel coding (default true)
    %   NumUCIBits         - Number of UCI bits (0...1706) (default 10)
    %   DataSourceUCI      - Source of UCI contents
    %                        (pseudo-noise (PN) or custom) (default 'PN9-ITU')
    %   NID0               - DM-RS scrambling identity (0...65535) (default [])
    %   DMRSPower          - Scaling of PUCCH DM-RS power in dB (default 0)
    %
    %   Example: 
    %   % Display the default properties of PUCCH format 2, in case of
    %   % no frequency hopping.
    %
    %   pucch = nrWavegenPUCCH2Config;
    %   pucch.FrequencyHopping = 'neither'
    %
    %   See also nrULCarrierConfig, nrWaveformGenerator,
    %   nrWavegenPUCCH0Config, nrWavegenPUCCH1Config,
    %   nrWavegenPUCCH3Config, nrWavegenPUCCH4Config.

    %   Copyright 2021-2024 The MathWorks, Inc.

    %#codegen
    
    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'Enable', 'Label', 'Power', 'BandwidthPartID', ...
            'SymbolAllocation', 'SlotAllocation', 'Period', 'PRBSet', ...
            'FrequencyHopping', 'SecondHopStartPRB', 'Interlacing', ...
            'RBSetIndex', 'InterlaceIndex', 'SpreadingFactor', 'OCCI',...
            'AntennaMapping', 'PrecodingMatrix', ...
            'NID', 'RNTI', 'Coding', 'NumUCIBits', 'DataSourceUCI', ...
            'NID0', 'DMRSPower'};
    end

    methods
        % Constructor
        function obj = nrWavegenPUCCH2Config(varargin)
            obj = obj@nr5g.internal.pucch.Format2ConfigBase(...
                  'Label', 'PUCCH format 2', ...
                  'NumUCIBits', 10, ...
                  varargin{:});
        end
        
        % Validate configuration
        function validateConfig(obj)
            % Call Format2ConfigBase validator
            validateConfig@nr5g.internal.pucch.Format2ConfigBase(obj);
            % Check that NumUCIBits>2
            coder.internal.errorIf(obj.NumUCIBits > 0 && obj.NumUCIBits <= 2,...
                'nr5g:nrWaveformGenerator:InvalidUCIBits234','2',obj.NumUCIBits);
            % Check Power
            validatePower(obj,obj.SlotAllocation,obj.Period);
        end
    end
    
    methods (Access=protected)
        % Controls the conditional display of properties
        function inactive = isInactiveProperty(obj, prop)
            % Call base class methods
            inactive = isInactiveProperty@nr5g.internal.wavegen.PUCCHConfigBase(obj, prop);
            inactive = inactive || isInactiveProperty@nr5g.internal.pucch.Format2ConfigBase(obj, prop);
            
            % If no coding, hide NumUCIBits
            inactive = inactive || (strcmp(prop,'NumUCIBits') && ~obj.Coding);

            % If coding, but no UCI bits (NumUCIBits = 0), hide DataSourceUCI
            inactive = inactive || (strcmp(prop,'DataSourceUCI') && obj.Coding && ~obj.NumUCIBits);
        end
    end
end
