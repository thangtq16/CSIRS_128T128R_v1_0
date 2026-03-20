classdef nr5G_TM_Dialog < wirelessWaveformGenerator.nr5G_Presets_Dialog
    %
    
    %   Copyright 2019-2024 The MathWorks, Inc.
    
    properties (Dependent)
        TMN               % Test Model Name/Number
    end
    
    properties (Hidden)
        TitleString = getString(message('nr5g:waveformApp:nrTMTitle'))
        
        TMNType = 'charPopup'
        TMNDropDown
        TMNGUI
        TMNLabel
        
        % Properties used to open a copy of this preset definition in a
        % Downlink/Uplink waveform type
        ThisWaveformType = 'NR Test Model';
        NewWaveformType = 'Downlink';
    end
    
    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_TM_Dialog(parent)
            obj@wirelessWaveformGenerator.nr5G_Presets_Dialog(parent); % call base constructor
            
            obj.TMNGUI.(obj.Callback)    = @(a,b) TMNChanged(obj);
        end
        
        function adjustSpec(obj)
            obj.TMNDropDown = obj.getTMNText('FR1');
            
            % For MATLAB script generation
            obj.configGenFcn  = @nrDLCarrierConfig;
            obj.configGenVar  = 'cfgDLTM';
        end
        
        function str = getIconDrawingCommand(obj)
            str = ['disp([' obj.configGenVar '.Label newline ...' newline ...
                '''SCS: '' num2str(' obj.configGenVar '.BandwidthParts{1}.SubcarrierSpacing) '' kHz'' newline ...' newline ...
                '''Bandwidth: '' num2str(' obj.configGenVar '.ChannelBandwidth) '' MHz'' newline ...' newline ...
                '''Subframes: '' num2str(' obj.configGenVar '.NumSubframes) ]);'];
        end
        
        function [blockName, maskTitleName, waveNameText] = getMaskTextWaveName(~)
            blockName = '5G NR Test Model';
            maskTitleName = '5G NR Test Model Waveform Generator';
            waveNameText = blockName;
        end
        
        function props = displayOrder(~)
            props = {'FrequencyRange'; 'TMN'; 'SubcarrierSpacing'; 'ChannelBandwidth'; 'DuplexMode'; ...
                'NumSubframes'; 'NCellID'; ...
                'WindowingSource'; 'WindowingPercent'; 'SampleRateSource'; ...
                'SampleRate'; 'PhaseCompensation'; 'CarrierFrequency'};
        end
        
        % Reset the dialog's backing parameters
        function restoreDefaults(obj)
            restoreDefaults@wirelessWaveformGenerator.nr5G_Dialog(obj);
            obj.TMN          = 'NR-FR1-TM1.1';
            obj.DuplexMode   = 'TDD'; % The standard only specified NR-TM FDD modes for FR1, so TDD is a better common choice across FR1 and FR2
            obj.NumSubframes = 20;    % TS 38.141-1 (FR1) Section 4.9.2.2: Duration is 1 radio frame (10 ms) for FDD and 2 radio frames for TDD (20 ms)
            obj.NCellID      = 1;     % Cell identity
            restoreMods(obj);         % Set modulation related parameters
            
        end
        
        % Apply 'configuration' that may be presented from previously saved 'dialog state'
        function applyConfiguration(obj, cfg)
            
            % If 'cfg' does not include 'NumSubframes' as a parameter, then the
            % session may come from an earlier version (pre-R2022b) where
            % 'NumSubframes' was not configurable. In that case, use the
            % 'DuplexMode' parameter to determine the value of 'NumSubframes'.
            if ~isfield(cfg,'NumSubframes')
                if strcmp(cfg.DuplexMode,'TDD')
                    % TS 38.141-1 (FR1) Section 4.9.2.2: Duration is 1 radio frame (10 ms) for FDD and 2 radio frames for TDD (20 ms)
                    cfg.NumSubframes = '20'; % TDD mode requires 20 subframes to generate a whole DL frame
                else
                    cfg.NumSubframes = '10'; % FDD mode only requires 10 subframes to generate a whole DL frame
                end
            end
            
            % Use base class method
            applyConfiguration@wirelessWaveformGenerator.nr5G_Presets_Dialog(obj,cfg);
            
        end
        
        % Get parameter 'configuration' for the dialog state
        function cfg = getConfiguration(obj)
            
            % Get preset parameter definitions
            refObj = wirelessWaveformApp.internal.hNRReferenceWaveformGenerator( ...
                obj.TMN, obj.ChannelBandwidth, obj.SubcarrierSpacing, obj.DuplexMode,obj.NCellID, ...
                wirelessWaveformApp.internal.getStandardVersionInUse('TM'));
            cfg = refObj.Config;
            
            % Applied additional properties
            cfg.NumSubframes = double(obj.NumSubframes);
            
            % Set the modulation related parameters
            cfg = getConfigurationMods(obj,cfg);
            
        end
        
        % Get 'configuration' that will be persistent in a save
        function config = getConfigurationForSave(obj)
            config.waveform = getConfigurationForSave@wirelessWaveformGenerator.nr5G_Presets_Dialog(obj);
            config.filtering = getConfigurationForSave(obj.Parent.FilteringDialog);
        end
        
        % Generate waveform from the configuration
        function waveform = generateWaveform(obj)
            cfg = getConfiguration(obj);
            [waveform, obj.gridSet] = nrWaveformGenerator(cfg);
        end
        
        function cellDialogs = getDialogsPerColumn(obj)
            cellDialogs{1} = {obj obj.Parent.FilteringDialog};
        end
        
        function channelIndex = getChannelForREMapping(obj)

            try
                cfg = getConfiguration(obj);
            catch
                % Initialization; possibly inconsistent state
                channelIndex = 1;
                return;
            end

            ch = cfg.PDSCH;

            % Select a target PDSCH 
            idx = cellfun(@(x) contains(x.Label,"target") & ~contains(x.Label,"non-"), ch);
            channelIndex = find(idx,1,'first');

            % There are single-PRB Test Model configs that don't have
            % target in their label. In this case, pick the full-slot ones.
            if isempty(channelIndex)
                idx = cellfun(@(x) contains(x.Label,"Full"), ch);
                channelIndex = find(idx,1,'first');
            end
            
        end
    end
    
    % Methods specific to this class and that are currently used by
    % neighboring classes
    methods (Access = public)
        function frChanged(obj, ~)
            % update list of TMs for this frequency range:
            obj.TMNGUI.(obj.DropdownValues) = obj.getTMNText(obj.FrequencyRange);
            
            frChanged@wirelessWaveformGenerator.nr5G_Presets_Dialog(obj, []);
        end
    end
    
    % Private methods
    methods (Access = private)
        function tmnDropdown = getTMNText(~, fr)
            % construct customer visible text from TMNs, e.g., "NR-FR1-TM1.1 (Full-band, uniform QPSK)"
            tmnDropdown = wirelessWaveformApp.internal.getTMOptions(fr);
        end
        
        function TMNChanged(obj)
            updateGrid(obj);
        end
    end
    
    methods (Access = protected)
        function out = getChannelName(~)
            out = 'Target PDSCH';
        end
    end

    % Getters/setters
    methods
        function tmn = get.TMN(obj)
            tmn = getDropdownVal(obj, 'TMN');
            
            % trim everything after 1st space, e.g., (Full-band, uniform QPSK)
            idx = strfind(tmn, ' ');
            tmn = tmn(1:idx(1)-1);
        end
        function set.TMN(obj, val)
            % add space in the end, as string '2' is contained within '2a' (false positive)
            obj.TMNGUI.Value = obj.TMNGUI.Items{contains(obj.TMNGUI.Items, [val ' '])};
        end
    end

end
