classdef nrWavegenPUCCH0Config < nr5g.internal.wavegen.EnablePower & ...
                                 nr5g.internal.wavegen.PUCCHConfigBase & ...
                                 nr5g.internal.pucch.Format0ConfigBase
    %nrWavegenPUCCH0Config PUCCH format 0 configuration object for 5G waveform generation
    %   PUCCH = nrWavegenPUCCH0Config creates a physical uplink control
    %   channel (PUCCH) configuration object. This object contains all the
    %   parameters of PUCCH format 0.
    %
    %   The default nrWavegenPUCCH0Config object configures a PUCCH format
    %   0 allocated in the first resource block and the last OFDM symbol in
    %   the slot of 14 OFDM symbols, and transmission in all slots. The
    %   PUCCH carries a single uplink control information (UCI) bit and no
    %   scheduling request (SR).
    %
    %   PUCCH = nrWavegenPUCCH0Config(Name,Value) creates a physical uplink
    %   control channel configuration object, PUCCH, with the specified
    %   property Name set to the specified Value. You can specify
    %   additional name-value pair arguments in any order as
    %   (Name1,Value1,...,NameN,ValueN).
    %
    %   nrWavegenPUCCH0Config properties:
    %
    %   Enable             - Flag turning this PUCCH on or off (default true)
    %   Label              - Alphanumeric description for this PUCCH
    %                        (default 'PUCCH format 0')
    %   Power              - Power scaling in dB (default 0)
    %   BandwidthPartID    - ID of bandwidth part containing this PUCCH
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
    %   GroupHopping       - Group hopping configuration
    %                        ('enable', 'disable', 'neither' (default))
    %   HoppingID          - Hopping identity (0...1023) (default [])
    %   InitialCyclicShift - Initial cyclic shift (0...11) (default 0)
    %   NumUCIBits         - Number of HARQ-ACK bits (0...2) (default 1)
    %   DataSourceUCI      - Source of HARQ-ACK contents
    %                        (pseudo-noise (PN) or custom) (default 'PN9-ITU')
    %   DataSourceSR       - Source of SR content (PN or custom) (default 0)
    %
    %   Example: 
    %   % Display the default properties of PUCCH format 0, in case of
    %   % no frequency hopping.
    %
    %   pucch = nrWavegenPUCCH0Config;
    %   pucch.FrequencyHopping = 'neither'
    %
    %   See also nrULCarrierConfig, nrWaveformGenerator,
    %   nrWavegenPUCCH1Config, nrWavegenPUCCH2Config,
    %   nrWavegenPUCCH3Config, nrWavegenPUCCH4Config.

    %   Copyright 2021-2024 The MathWorks, Inc.

    %#codegen
    
    properties (Hidden)
        DataSourceSR_Values = nr5g.internal.wavegen.DataSourceCommon.DataSource_Options;
        % Custom property list to change the order of display properties
        CustomPropList = {'Enable', 'Label', 'Power', 'BandwidthPartID', ...
            'SymbolAllocation', 'SlotAllocation', 'Period', 'PRBSet', ...
            'FrequencyHopping', 'SecondHopStartPRB', 'Interlacing',...
            'RBSetIndex', 'InterlaceIndex', 'GroupHopping', 'HoppingID',...
            'AntennaMapping', 'PrecodingMatrix', ...
            'InitialCyclicShift', 'NumUCIBits', 'DataSourceUCI', 'DataSourceSR'};
    end

    % Public, tunable properties
    properties
        %DataSourceSR Source of SR content
        % Specify DataSourceSR as one of {'PN9-ITU', 'PN9', 'PN11',
        % 'PN15', 'PN23'}, as a cell array containing one of the
        % abovementioned options and a numeric scalar that is the random
        % seed (for example, {'PN9',7}), or as a binary vector. If the seed
        % is not specified, then all shift registers are initialized with
        % an active state. The default is 0.
        DataSourceSR = 0;
    end
    
    methods
        % Constructor
        function obj = nrWavegenPUCCH0Config(varargin)
            obj = obj@nr5g.internal.pucch.Format0ConfigBase(...
                  'Label', 'PUCCH format 0', ...
                  varargin{:});
        end
        
        % Self-validate and set properties
        function obj = set.DataSourceSR(obj,val)
            prop = 'DataSourceSR';
            temp = getDataSource(obj, prop, val);
            obj.(prop) = temp;
        end
        
        % Validate configuration
        function validateConfig(obj)
            % Call Format0ConfigBase validator
            validateConfig@nr5g.internal.pucch.Format0ConfigBase(obj);
            % Check that NumUCIBits<=2
            coder.internal.errorIf(obj.NumUCIBits > 2,'nr5g:nrWaveformGenerator:InvalidUCIBits01','0',obj.NumUCIBits);
            % Check Power
            validatePower(obj,obj.SlotAllocation,obj.Period);
        end
    end

    methods (Access=protected)
        % Controls the conditional display of properties
        function inactive = isInactiveProperty(obj, prop)
            % Call base class methods
            inactive = isInactiveProperty@nr5g.internal.wavegen.PUCCHConfigBase(obj, prop);
            inactive = inactive || isInactiveProperty@nr5g.internal.pucch.Format0ConfigBase(obj, prop);
        end
    end
end
