classdef nrULCarrierConfig < nr5g.internal.wavegen.CarrierConfigBase
    %nrULCarrierConfig 5G uplink waveform configuration
    %   CFG = nrULCarrierConfig creates a configuration object for a
    %   single-component-carrier 5G uplink waveform. This object contains
    %   parameters defining the frequency range, channel bandwidth, cell
    %   identity, waveform duration (in subframes), SCS carriers, bandwidth
    %   parts, PUSCH and associated DM-RS and PT-RS, PUCCH and associated
    %   DM-RS, and SRS.
    %
    %   CFG = nrULCarrierConfig(Name,Value) creates a 5G uplink waveform
    %   configuration object with the specified property Name set to the
    %   specified Value. You can specify additional name-value pair arguments
    %   in any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrULCarrierConfig methods:
    %   
    %   openInGenerator     - Open this uplink carrier configuration in the 5G Waveform Generator 
    %
    %   nrULCarrierConfig properties:
    %
    %   Label               - Alphanumeric description for this uplink carrier
    %                         configuration object (default 'Uplink carrier 1')
    %   FrequencyRange      - Frequency range ('FR1' (default) or 'FR2')
    %   ChannelBandwidth    - Channel bandwidth in MHz (default 50)
    %   NCellID             - Physical layer cell identity (0...1007)
    %                         (default 1)
    %   NumSubframes        - Number of subframes (default 10)
    %   InitialNSubframe    - Initial subframe number (default 0)
    %   WindowingPercent    - Percentage of windowing relative to FFT length (default 0)
    %   SampleRate          - Sample rate of the OFDM modulated waveform (default [])
    %   CarrierFrequency    - Carrier frequency in Hz (default 0)
    %   SCSCarriers         - Configuration of SCS carrier(s) (default {nrSCSCarrierConfig})
    %   BandwidthParts      - Configuration of bandwidth part(s) (default {nrWavegenBWPConfig})
    %   IntraCellGuardBands - Configuration of intracell guard bands 
    %                         (default {nrIntraCellGuardBandsConfig})
    %   PUSCH               - Configuration of PUSCH channel(s) (default {nrWavegenPUSCHConfig})
    %   PUCCH               - Configuration of PUCCH channel(s)
    %                         (default {nrWavegenPUCCH0Config('Enable',0)})
    %   SRS                 - Configuration of SRS signal(s)
    %                         (default {nrWavegenSRSConfig('Enable',0)})
    %
    %   Example 1:
    %   % Create a configuration for a single-numerology (15 kHz), single-user
    %   % 5G uplink waveform with no SRS; then generate the waveform.
    %   % In nrULCarrierConfig, PUSCH is enabled, while PUCCH and SRS are
    %   % disabled by default.
    %
    %   cfg = nrULCarrierConfig('FrequencyRange', 'FR1', 'ChannelBandwidth', 40, 'NumSubframes', 20);
    %   cfg.SCSCarriers{1}.NSizeGrid = 100; % default SCS is 15 kHz
    %   cfg.BandwidthParts{1}.NStartBWP = cfg.SCSCarriers{1}.NStartGrid + 10;
    %
    %   waveform = nrWaveformGenerator(cfg);
    %
    %   Example 2:
    %   % Create a configuration for a mixed-numerology, multi-user 5G uplink
    %   % waveform; then generate the waveform.
    %
    %   % SCS Carriers:
    %   scscarriers = {nrSCSCarrierConfig('SubcarrierSpacing', 15, 'NStartGrid', 10, 'NSizeGrid', 100), ...
    %                  nrSCSCarrierConfig('SubcarrierSpacing', 30, 'NStartGrid', 0, 'NSizeGrid', 70)};
    %   % Bandwidth parts:
    %   bwp = {nrWavegenBWPConfig('BandwidthPartID', 0, 'SubcarrierSpacing', 15, 'NStartBWP', 30, 'NSizeBWP', 80), ...
    %          nrWavegenBWPConfig('BandwidthPartID', 1, 'SubcarrierSpacing', 30, 'NStartBWP', 0, 'NSizeBWP', 60)};
    %   % PUSCH:
    %   pusch = {nrWavegenPUSCHConfig('BandwidthPartID', 0, 'Modulation', '16QAM', 'SlotAllocation', 0:2:9, 'PRBSet', 0:19, 'RNTI', 1, 'NID', 1), ...
    %            nrWavegenPUSCHConfig('BandwidthPartID', 1, 'Modulation', 'QPSK', 'RNTI', 2, 'NID', 2, 'PRBSet', 50:59)};
    %   % PUCCH:
    %   % In nrWavegenPUCCH0Config, PUCCH is enabled by default.
    %   pucch = {nrWavegenPUCCH0Config('BandwidthPartID', 1, 'SlotAllocation', 0:9, 'PRBSet', 2, 'DataSourceUCI', 'PN9')};
    %   % SRS:
    %   % In nrWavegenSRSConfig, SRS is enabled by default.
    %   srs = {nrWavegenSRSConfig('BandwidthPartID', 0, 'SlotAllocation', 1:2:9, 'NumSRSPorts', 2), ...
    %          nrWavegenSRSConfig('BandwidthPartID', 1, 'FrequencyStart', 4)};
    %
    %   % Combine everything together:
    %   cfg = nrULCarrierConfig('FrequencyRange', 'FR1', 'ChannelBandwidth', 40, 'NumSubframes', 20);
    %   cfg.SCSCarriers = scscarriers;
    %   cfg.BandwidthParts = bwp;
    %   cfg.PUSCH = pusch;
    %   cfg.PUCCH = pucch;
    %   cfg.SRS = srs;
    %
    %   % Generate waveform:
    %   waveform = nrWaveformGenerator(cfg);
    %
    %   See also nrWaveformGenerator, nrSCSCarrierConfig,
    %   nrWavegenBWPConfig, nrIntraCellGuardBandsConfig,
    %   nrWavegenPUSCHConfig, nrPUSCHDMRSConfig, nrPUSCHPTRSConfig,
    %   nrWavegenPUCCH0Config, nrWavegenPUCCH1Config,
    %   nrWavegenPUCCH2Config, nrWavegenPUCCH3Config,
    %   nrWavegenPUCCH4Config, nrWavegenSRSConfig, nrDLCarrierConfig.

    %   Copyright 2020-2024 The MathWorks, Inc.

    %#codegen

    properties
        
        %WindowingPercent Percentage of windowing relative to FFT length
        % Specify WindowingPercent as a scalar or seven-element or 
        % five-element vector of real doubles in the range [0 50] or as [].
        % This property configures the number of time-domain samples over
        % which windowing and overlapping of OFDM symbols is applied, 
        % as a percentage of the FFT length. A scalar value establishes the
        % same windowing for all combinations of subcarrier spacing and
        % cyclic prefix. If set to [], a default value is automatically
        % selected based on other parameters (see <a href="matlab: doc('nrOFDMModulate')">nrOFDMModulate</a>). 
        % If WindowingPercent is the seven-element vector [w1 w2 w3 w4 w5 w6 w7],
        % then w1% is the windowing percentage for the 15 kHz carrier, 
        % w2% for 30 kHz, w3% for 60 kHz and normal cyclic prefix, 
        % w4% for 60 kHz and extended cyclic prefix, w5% for 120 kHz, 
        % w6% for 460 kHz, w7% for 960 kHz. If WindowingPercent has five
        % elements then w5 applies to 120 kHz, 460 kHz, and 960 kHz.
        % The default is 0.
        WindowingPercent = 0;

        %IntraCellGuardBands Configuration of intracell guard bands
        %   Specify the intracell guard bands for operation with shared
        %   spectrum channel access for FR1 as a cell array of 
        %   <a href="matlab:help('nrIntraCellGuardBandsConfig')">nrIntraCellGuardBandsConfig</a> objects. The default value is
        %   {nrIntraCellGuardBandsConfig}, which specifies that guard bands
        %   are not configured and all resource blocks of all SCS carriers
        %   are available.
        IntraCellGuardBands {mustBeA(IntraCellGuardBands,{'cell'})} = {nrIntraCellGuardBandsConfig};

        %PUSCH Configuration of PUSCH
        % Specify PUSCH as a cell array of <a href="matlab:
        % help('nrWavegenPUSCHConfig')">nrWavegenPUSCHConfig</a> objects.
        % This property configures different physical uplink shared
        % channels (PUSCH) and associated DM-RS and PT-RS signals. The
        % default is {nrWavegenPUSCHConfig}.
        PUSCH             = nrULCarrierConfig.getDefault('PUSCH');

        %PUCCH Configuration of PUCCH
        % Specify PUCCH as a cell array consisting of any combination of
        % <a href="matlab:help('nrWavegenPUCCH0Config')",
        % >nrWavegenPUCCH0Config</a>, <a href="matlab:help('nrWavegenPUCCH1Config')"
        % >nrWavegenPUCCH1Config</a>, <a href="matlab:help('nrWavegenPUCCH2Config')"
        % >nrWavegenPUCCH2Config</a>,
        % <a href="matlab:help('nrWavegenPUCCH3Config')"
        % >nrWavegenPUCCH3Config</a>, or <a href="matlab:help('nrWavegenPUCCH4Config')"
        % >nrWavegenPUCCH4Config</a> objects. This property
        % configures different physical uplink control channel (PUCCH) and associated
        % DM-RS signals. The default value is {nrWavegenPUCCH0Config('Enable',0)},
        % which disables the PUCCH.
        PUCCH             = nrULCarrierConfig.getDefault('PUCCH');

        %SRS Configuration of SRS
        % Specify SRS as a cell array of <a href="matlab:
        % help('nrWavegenSRSConfig')">nrWavegenSRSConfig</a> objects. This
        % property configures different sounding reference signals (SRS).
        % The default value is {nrWavegenSRSConfig('Enable',0)}, which
        % disables the SRS.
        SRS               = nrULCarrierConfig.getDefault('SRS');
    end

    properties (Hidden)
        CustomPropList = {'Label', 'FrequencyRange', 'ChannelBandwidth', 'NCellID', 'NumSubframes', 'InitialNSubframe', ...
            'WindowingPercent', 'SampleRate', 'CarrierFrequency', ...
            'SCSCarriers', 'BandwidthParts', 'IntraCellGuardBands', ...
            'PUSCH', 'PUCCH', 'SRS'};
    end

    methods
        % Constructor
        function obj = nrULCarrierConfig(varargin)

            pusch = nr5g.internal.parseProp('PUSCH', ...
                nrULCarrierConfig.getDefault('PUSCH'),varargin{:});
            pucch = nr5g.internal.parseProp('PUCCH', ...
                nrULCarrierConfig.getDefault('PUCCH'),varargin{:});
            srs = nr5g.internal.parseProp('SRS', ...
                nrULCarrierConfig.getDefault('SRS'),varargin{:});

            obj@nr5g.internal.wavegen.CarrierConfigBase( ...
                'PUSCH', pusch, ...
                'PUCCH', pucch, ...
                'SRS', srs, ...
                'Label', 'Uplink carrier 1', ...
                varargin{:});
        end

        % Self-validate and set properties
        function obj = set.WindowingPercent(obj,val)
            prop = 'WindowingPercent';

            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 7],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))

                % If Windowing is a vector, it must have 5 or 7 elements
                coder.internal.errorIf(~any(numel(temp)==[1 5 7]), ...
                    'nr5g:nrWaveformGenerator:InvalidWindowingVector', 'uplink', 5, 7);

                validateattributes(temp,{'numeric'},...
                    {'real','nonnegative', '<=', 50},...
                    [class(obj) '.' prop],prop);
            end

            obj.WindowingPercent = temp;
        end

        function obj = set.IntraCellGuardBands(obj,val)
            validateCellObjProp(obj, 'IntraCellGuardBands', {'nrIntraCellGuardBandsConfig'}, val);
            scs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(val,'SubcarrierSpacing');
            coder.internal.errorIf(numel(unique(scs(:))) ~= numel(val),'nr5g:nrIntraCellGuardBandsConfig:MultipleSCS');
            obj.IntraCellGuardBands = val;
        end
        function obj = set.PUSCH(obj,val)
            validateCellObjProp(obj, 'PUSCH', {'nrWavegenPUSCHConfig'}, val);
            obj.PUSCH = val;
        end

        function obj = set.PUCCH(obj,val)
            validateCellObjProp(obj, 'PUCCH', {'nrWavegenPUCCH0Config','nrWavegenPUCCH1Config',...
                'nrWavegenPUCCH2Config','nrWavegenPUCCH3Config','nrWavegenPUCCH4Config'}, val);
            obj.PUCCH = val;
        end

        function obj = set.SRS(obj,val)
            validateCellObjProp(obj, 'SRS', {'nrWavegenSRSConfig'}, val);
            obj.SRS = val;
        end

        function validateConfig(obj)

            % Call nrCarrierConfigBase validator
            validateConfig@nr5g.internal.wavegen.CarrierConfigBase(obj);

            %% PUSCH
            % Make sure all PUSCH link to a valid bandwidth part
            checkResource2BWPLinks(obj, obj.PUSCH, 'PUSCH');

            for idx = 1:numel(obj.PUSCH)
                % Validate each PUSCH
                if obj.PUSCH{idx}.Enable
                    % No validation for disabled PUSCH
                    validateConfig(obj.PUSCH{idx});

                    % Validate properties related to MIMO precoding and
                    % antenna mapping
                    if strcmpi(obj.PUSCH{idx}.TransmissionScheme,'nonCodebook')
                        % Validate PUSCH MIMO precoding configuration for
                        % non-codebook PUSCH
                        validateMIMOPrecoding(obj.PUSCH{idx},'PUSCH',obj.PUSCH{idx}.NumLayers,idx);
                    else
                        % Validate the length of AntennaMapping against
                        % NumAntennaPorts for codebook PUSCH
                        if ~isempty(obj.PUSCH{idx}.AntennaMapping)
                            coder.internal.errorIf(numel(obj.PUSCH{idx}.AntennaMapping)~=obj.PUSCH{idx}.NumAntennaPorts, ...
                                'nr5g:nrWaveformGenerator:InvalidAntennaMappingCodebookPUSCH',idx, ...
                                numel(obj.PUSCH{idx}.AntennaMapping),obj.PUSCH{idx}.NumAntennaPorts);
                        end
                    end
                end
            end

            %% PUCCH
            % Make sure all PUCCH link to a valid bandwidth part
            checkResource2BWPLinks(obj, obj.PUCCH, 'PUCCH');

            for idx = 1:numel(obj.PUCCH)
                % Validate each PUCCH
                if obj.PUCCH{idx}.Enable
                    % No validation for disabled PUCCH
                    validateConfig(obj.PUCCH{idx});

                    % Validate PUCCH MIMO precoding confirguation
                    validateMIMOPrecoding(obj.PUCCH{idx},'PUCCH',1,idx);
                end
            end

            %% SRS
            % SRS must link to an existing Bandwidth Part
            checkResource2BWPLinks(obj, obj.SRS, 'SRS');

            for idx = 1:numel(obj.SRS)
                % Validate each SRS
                if obj.SRS{idx}.Enable
                    % No validation for disabled SRS
                    validateConfig(obj.SRS{idx});

                    % Validate SRS MIMO precoding configuration
                    validateMIMOPrecoding(obj.SRS{idx},'SRS',obj.SRS{idx}.NumSRSPorts,idx);
                end
            end
        end
    end

    methods (Static, Access = protected)
        % Default values of PUSCH, PUCCH, and SRS properties.
        function out = getDefault(propName)
            switch propName
                case 'PUSCH'
                    out = {nrWavegenPUSCHConfig};
                case 'PUCCH'
                    out = {nrWavegenPUCCH0Config('Enable',0)};
                case 'SRS'
                    out = {nrWavegenSRSConfig('Enable',0)};
            end
        end
    end

end