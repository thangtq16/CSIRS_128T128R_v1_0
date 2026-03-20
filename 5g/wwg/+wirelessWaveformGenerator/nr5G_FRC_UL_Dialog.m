classdef nr5G_FRC_UL_Dialog < wirelessWaveformGenerator.nr5G_FRC_Dialog
    % Uplink FRC extension for the Wireless Waveform Generator App

    %   Copyright 2019-2025 The MathWorks, Inc.

    properties (Hidden)
        TitleString = getString(message('nr5g:waveformApp:GeneralConfigTitle'))

        % Properties used to open a copy of this preset definition in a
        % Downlink/Uplink waveform type
        ThisWaveformType = 'Uplink FRC';
        NewWaveformType = 'Uplink';

        FRCSelectButton
        FRCSelectButtonType = 'uibutton'
        FRCSelectButtonGUI
        FRCSelectButtonLabel
        FRCSelectButtonIconID = 'select_signalMultiple'

        SelectedFRCType = 'charText'
        SelectedFRCLabel
        SelectedFRCGUI

        ChannelBWRBsType = 'numericText'
        ChannelBWRBsLabel
        ChannelBWRBsGUI

    end

    properties (Access = private)
        pConfigCache
        pIsCustom
        pSelectedFRCName
        pSelectedFRCMCS
        pSelectedFRCDuplexMode
        pSkipUpdateGrid
    end

    properties (Access = private, Constant = true)

        pPUSCHClassName = 'wirelessWaveformGenerator.nr5G_FRC_PUSCH_Dialog'
        pDefaultConfig = wirelessWaveformGenerator.nr5G_FRC_UL_Dialog.getDefaultConfig

    end

    % Constructor and public methods defined in the base app
    methods (Access = public)

        function obj = nr5G_FRC_UL_Dialog(parent)

            % Call base constructor
            obj@wirelessWaveformGenerator.nr5G_FRC_Dialog(parent, false);

            % Add PUSCH dialog to map
            puschClassName = wirelessWaveformGenerator.nr5G_FRC_UL_Dialog.pPUSCHClassName;
            if ~isKey(obj.Parent.DialogsMap, puschClassName)
                obj.Parent.DialogsMap(puschClassName) = eval([puschClassName '(obj.Parent)']);
            end

            % Add callbacks and listeners
            obj.NumSubframesGUI.(obj.Callback)            = @(src,evnt) framesChangedGUI(obj,src,evnt);
            obj.FRCSelectButtonGUI.(obj.UIButtonCallback) = @(a,b)frcSelectButtonCallback(obj);
            obj.FRCSelectButton = wirelessWaveformGenerator.internal.frcSelectionWindow;
            obj.FRCSelectButton.addlistener('FRCSelected',@(src,event)obj.frcSelected(src,event));

            % Update info
            obj.updateInfo();

        end

        function cellDialogs = getDialogsPerColumn(obj)
            cellDialogs{1} = {obj obj.Parent.GenerationDialog obj.Parent.FilteringDialog};
            cellDialogs{2} = {obj.Parent.DialogsMap(obj.pPUSCHClassName)};
        end

        function setupDialog(obj)
            setupDialog@wirelessWaveformGenerator.nr5G_Presets_Dialog(obj);
            layoutUIControls(obj);
        end

        function str = getIconDrawingCommand(obj)
            str = ['disp([' obj.configGenVar '.Label newline ...' newline ...
                '''Modulation: '' ' obj.configGenVar '.PUSCH{1}.Modulation newline ...' newline ...
                '''Bandwidth: '' num2str(' obj.configGenVar '.ChannelBandwidth) '' MHz'' newline ...' newline ...
                '''Subframes: '' num2str(' obj.configGenVar '.NumSubframes) ]);'];
        end

        function [blockName, maskTitleName, waveNameText] = getMaskTextWaveName(obj)
            blockName = ['5G NR ' obj.Parent.WaveformGenerator.pCurrentExtensionType];
            maskTitleName = ['5G NR ' obj.Parent.WaveformGenerator.pCurrentExtensionType ' Waveform Generator'];
            waveNameText = blockName;
        end

        function adjustSpec(obj)

            obj.LabelWidth = 130;

            % For MATLAB script generation
            obj.configGenFcn  = @nrULCarrierConfig;
            obj.configGenVar  = 'cfgULFRC';

        end

        function props = displayOrder(~)

            props = {'FRCSelectButton'; 'SelectedFRC'; 'FrequencyRange'; 'SubcarrierSpacing'; ...
                'ChannelBandwidth'; 'ChannelBWRBs'; 'NumSubframes'; 'NCellID'; 'WindowingSource'; ...
                'WindowingPercent'; 'SampleRateSource'; 'SampleRate'; ...
                'PhaseCompensation'; 'CarrierFrequency'};

        end

        function restoreDefaults(obj)

            % Restore internal cache and flags
            obj.pConfigCache = wirelessWaveformGenerator.nr5G_FRC_UL_Dialog.pDefaultConfig;
            obj.pSelectedFRCName = obj.pConfigCache.Label;
            obj.pSelectedFRCMCS = 'QPSK, R=1/3';
            obj.pSelectedFRCDuplexMode = 'FDD';
            obj.pSkipUpdateGrid = false;
            obj.updateErrorCache([],[]);

            % Restore general config
            obj.NumSubframes      = obj.pConfigCache.NumSubframes;
            obj.NCellID           = obj.pConfigCache.NCellID;
            obj.FrequencyRange    = obj.pConfigCache.FrequencyRange;
            obj.ChannelBandwidth  = obj.pConfigCache.ChannelBandwidth;
            obj.SubcarrierSpacing = obj.pConfigCache.SCSCarriers{1}.SubcarrierSpacing;

            % Restore filtering and bit source
            restoreMods(obj);

            % Restore PUSCH config
            puschDlg = obj.getPUSCHDialog();
            if ~(isprop(obj,'Initializing') && obj.Initializing) && ~isempty(puschDlg)
                % When App is not initializing, need to call
                % restoreDefaults for the PUSCH dialog manually
                % This is necessary when opening new session or loading
                % saved sessions
                puschDlg.restoreDefaults();
            end

            % Restore info display
            obj.pIsCustom = false;
            obj.updateInfo();

        end

        function panels = getExtraPanels(obj)
            panels = {obj.getPUSCHDialog()};
        end

        function waveform = generateWaveform(obj)
            waveform = generateWaveform@wirelessWaveformGenerator.nr5G_FRC_Dialog(obj);
        end

        function addConfigCode(obj,sw)
            addConfigCode@wirelessWaveformGenerator.nr5G_Dialog(obj,sw);
        end

        % Get waveform parameter configuration
        function cfg = getConfiguration(obj)

            if isempty(obj.pErrorCache)
                % Get configuration from cache
                cfg = obj.pConfigCache;

                % Set the PUSCH data source
                cfg = getConfigurationDataSource(obj,'PUSCH',cfg);
                % Set the modulation related parameters
                cfg = getConfigurationMods(obj,cfg);
            else
                % Rethrow newest error
                rethrow(obj.pErrorCache{end});
            end

        end

        function applyConfiguration(obj, cfg)

            obj.updateErrorCache([],[]);
            % Config information
            isCustom = strcmpi(cfg.FRC,'N/A');

            % Set PUSCHLocation for pre-R2023a sessions
            if ~isfield(cfg,'PUSCHLocation') && ~isfield(cfg,'PUSCH')
                cfg.PUSCHLocation = 'Bandwidth center';
            end

            % Match the saved configuration to the app
            if isCustom
                % Start with G-FR1-A1-1 which captures all non-configurable
                % parameters
                waveCfg = wirelessWaveformGenerator.nr5G_FRC_UL_Dialog.pDefaultConfig;
            else
                % Get the full config from the FRC definition directly
                refObj = wirelessWaveformApp.internal.hNRReferenceWaveformGenerator(cfg.FRC,cfg.ChannelBandwidth,[],cfg.DuplexMode,str2double(cfg.NCellID),"15.2.0");
                waveCfg = refObj.Config;
                obj.pSelectedFRCName = cfg.FRC;
            end
            % Go over all fields and map to the config cache
            waveCfg = mapStructToConfig(waveCfg,cfg);
            % Update carrier size if necessary
            if isCustom
                maxNRB = nr5g.internal.wavegen.getNumRB(waveCfg.FrequencyRange,waveCfg.SCSCarriers{1}.SubcarrierSpacing,waveCfg.ChannelBandwidth);
                waveCfg.SCSCarriers{1}.NSizeGrid = maxNRB;
                waveCfg.BandwidthParts{1}.NSizeBWP = maxNRB;
            end

            % Populate the left column of GUI
            obj.pSkipUpdateGrid = true;
            applyConfiguration@wirelessWaveformGenerator.nr5G_Presets_Dialog(obj,cfg);
            obj.pSkipUpdateGrid = false;

            % Populate the right column of GUI
            obj.pIsCustom = isCustom;
            puschDlg = obj.getPUSCHDialog();
            puschDlg.loadSavedSession(waveCfg,cfg,isCustom);
            obj.updateErrorCache([],[]);

        end

        % Properties to exclude when exporting
        function props = props2ExcludeFromConfig(~)
            % Read-only field do not get exported

            props = {'FRCSelectButton','SelectedFRC','ChannelBWRBs'};

        end

        function config = getConfigurationForSave(obj,~)

            if isempty(obj.pErrorCache)

                % Get config for save from the left column (General, Bit
                % Source, Filtering)
                config = getConfigurationForSave@wirelessWaveformGenerator.nr5G_FRC_Dialog(obj);
                struct = config.waveform;

                % Get config for save from the right column (PUSCH)
                puschDlg = obj.getPUSCHDialog();
                struct = puschDlg.getConfigurationForSave(struct);

                if ~obj.pIsCustom

                    struct.MCS = obj.pSelectedFRCMCS;
                    struct.FRC = obj.pSelectedFRCName;
                    struct.DuplexMode = obj.pSelectedFRCDuplexMode;
                    if isfield(struct,'PUSCHModulation')
                        struct = rmfield(struct,{'PUSCHModulation','PUSCHTargetCodeRate'});
                    end

                else

                    struct.FRC = 'N/A';

                end

                config.waveform = struct;

            else

                % When config is invalid, do not allow saving session
                rethrow(obj.pErrorCache{end});

            end

        end

        function channelIndex = getChannelForREMapping(obj,~)
            puschDlg = obj.getPUSCHDialog();
            PUSCH1 = strcmpi(puschDlg.SelectPUSCH,'PUSCH 1');
            if PUSCH1
                channelIndex = 1;
            else
                channelIndex = 2;
            end
        end

    end

    % Methods specific to this class and that are currently used by
    % neighboring classes
    methods (Access = public)

        % Update cached config and then use the config to update grid,
        % channel bandwidth view and info
        function updateAppStatus(obj,cfg,isCustom)

            obj.pConfigCache = cfg;

            % Do not use input to set/reset pIsCustom directly. pIsCustom
            % should only be reset to false by frcSelected
            if isCustom
                obj.pIsCustom = true;
                obj.pConfigCache.Label = 'Single PUSCH waveform';
                obj.pConfigCache.PUSCH{1}.Label = 'PUSCH 1';
            end

            updateGrid(obj);
            updateInfo(obj);
            updateChannelBandwidthView(obj);

        end

        % Custom callback for FrequencyRange
        function frChanged(obj, ~)
            % This method is called by the setter of FrequencyRange

            % Update SCS and bandwidth options (no need to update MCS
            % options to allow extended control)
            obj.frChangedBase();
            obj.scsChanged();

            if ~obj.Parent.AppObj.Initializing && ~obj.pSkipUpdateGrid

                % Update PUSCH resource allocation when necessary, and then
                % update the cached config, grid, info and channel
                % bandwidth view (no need to call updates here as PUSCH
                % dialog does it)
                obj.updatePUSCHResourceAllocation();

            end

        end

    end

    % Protect methods for derived classes and overridden methods of
    % superclasses
    methods (Access = protected)

        % Custom callback of SubcarrierSpacing
        function scsChanged(obj,~)
            % This method is called by the setter of SubcarrierSpacing

            % Update channel bandwidth options
            scsChanged@wirelessWaveformGenerator.nr5G_FRC_Dialog(obj);

            if ~obj.Parent.AppObj.Initializing && ~obj.pSkipUpdateGrid

                % Update PUSCH resource allocation when necessary, and then
                % update the cached config, grid, info and channel
                % bandwidth view (no need to call updates here as PUSCH
                % dialog does it)
                obj.updatePUSCHResourceAllocation();

            end

        end

        % Custom callback of NumSubframes
        function framesChangedGUI(obj,src,evnt)

            prop = 'NumSubframes';
            e = updateProperty(obj, src, evnt, prop, Config=obj.pConfigCache);
            if isempty(e)
                obj.pConfigCache.NumSubframes = obj.NumSubframes;
                puschDlg = obj.getPUSCHDialog();
                if ~isempty(puschDlg) % to prevent error when initializing
                    puschDlg.updatePUSCHConfig(obj.pConfigCache,false,false);
                end
            end

        end

        % Custom callback of NCellID
        function nCellChanged(obj,src,evnt)

            prop = 'NCellID';
            e = updateProperty(obj, src, evnt, prop, Config=obj.pConfigCache);

            if isempty(e)
                obj.pConfigCache.NCellID = obj.NCellID;
                puschDlg = obj.getPUSCHDialog();
                if ~isempty(puschDlg) % to prevent error when initializing
                    puschDlg.updatePUSCHConfig(obj.pConfigCache,true);
                end
            end

        end

        % Custom callback of WindowingSource
        function windowingChangedGUI(obj,~)

            if strcmpi(obj.WindowingSource,'Auto') && any(strcmpi(obj.pErrorProp,'WindowingPercent'))
                % When turning WindowingSource to Auto and WindowingPercent
                % has an error, clear the error and return WindowingPercent
                % to the default
                obj.WindowingPercent = obj.pDefaultConfig.WindowingPercent;
                obj.updateErrorCache([],'WindowingPercent');
            end

            windowingChangedGUI@wirelessWaveformGenerator.nr5G_FRC_Dialog(obj);

        end

        % Custom callback of WindowingPercent
        function windowingPercentChangedGUI(obj,src,evnt)

            prop = 'WindowingPercent';
            e = updateProperty(obj, src, evnt, prop, Config=obj.pConfigCache);

            if isempty(e)
                obj.pConfigCache.WindowingPercent = obj.WindowingPercent;
                puschDlg = obj.getPUSCHDialog();
                if ~isempty(puschDlg) % to prevent error when initializing
                    puschDlg.updatePUSCHConfig(obj.pConfigCache,true,obj.pIsCustom);
                end
            end

        end

        % Custom callback of SampleRateSource
        function srChangedGUI(obj,~)

            if strcmpi(obj.SampleRateSource,'Auto') && any(strcmpi(obj.pErrorProp,'SampleRate'))
                % When turning SampleRateSource to Auto and SampleRate has
                % an error, clear the error and return SampleRate to the
                % default
                obj.SampleRate = obj.pDefaultConfig.SampleRate;
                obj.updateErrorCache([],'SampleRate');
            end

            srChangedGUI@wirelessWaveformGenerator.nr5G_FRC_Dialog(obj);

        end

        % Custom callback of SampleRate
        function sampleRateChangedGUI(obj,src,evnt)

            prop = 'SampleRate';
            e = updateProperty(obj, src, evnt, prop, Config=obj.pConfigCache);

            if isempty(e)
                obj.pConfigCache.SampleRate = obj.SampleRate;
                puschDlg = obj.getPUSCHDialog();
                if ~isempty(puschDlg) % to prevent error when initializing
                    puschDlg.updatePUSCHConfig(obj.pConfigCache,true,obj.pIsCustom);
                end
            end

        end

        % Custom callback of PhaseCompensation
        function phaseCompChangedGUI(obj, ~)

            if ~obj.PhaseCompensation && any(strcmpi(obj.pErrorProp,'CarrierFrequency'))
                % When turning off PhaseCompensation and CarrierFrequency
                % has an error, clear the error and return CarrierFrequency
                % to the default
                obj.CarrierFrequency = obj.pDefaultConfig.CarrierFrequency;
                obj.updateErrorCache([],'CarrierFrequency');
            end

            phaseCompChangedGUI@wirelessWaveformGenerator.nr5G_FRC_Dialog(obj);

        end

        % Custom callback of CarrierFrequency
        function carrierFrequencyChangedGUI(obj,src,evnt)

            prop = 'CarrierFrequency';
            e = updateProperty(obj, src, evnt, prop, Config=obj.pConfigCache);

            if isempty(e)
                obj.pConfigCache.CarrierFrequency = obj.CarrierFrequency;
                puschDlg = obj.getPUSCHDialog();
                if ~isempty(puschDlg) % to prevent error when initializing
                    puschDlg.updatePUSCHConfig(obj.pConfigCache,true,obj.pIsCustom);
                end
            end

        end

        function updateInfo(obj)
            try
                try
                    cfg = getConfiguration(obj);
                catch
                    % Initialization or possibly inconsistent state
                    return
                end

                wid = ["nr5g:nrPXSCH:DMRSParametersNoSymbols";...
                    "nr5g:nrPXSCH:CustomSymbolSetNoSymbols"];
                c = wirelessWaveformApp.internal.suppressWarning(wid); %#ok<NASGU>

                ch = cfg.PUSCH{1};
                puschDlg = obj.getPUSCHDialog();
                if ~isempty(puschDlg)
                    PUSCH1 = strcmpi(puschDlg.SelectPUSCH,'PUSCH 1');
                    if PUSCH1
                        puschNumber = 1;
                    else
                        puschNumber = 2;
                    end
                    ch = cfg.PUSCH{puschNumber};
                end

                %% Update info displayed in General Configuration

                % Fixed reference channel
                if obj.pIsCustom
                    obj.SelectedFRC = 'N/A';
                else
                    obj.SelectedFRC = obj.pSelectedFRCName;
                end

                % Channel bandwidth (RBs)
                obj.ChannelBWRBs = cfg.BandwidthParts{1}.NSizeBWP;

                %% Update info displayed in PUSCH Configuration

                
                % Allocated PRB set
                puschDlg.AllocatedPRBSet = sprintf('[%d:%d]',ch.PRBSet(1),ch.PRBSet(end));

                % Payload (bits/slot)
                bwp = cfg.BandwidthParts{1};
                carrier = nrCarrierConfig('NCellID', cfg.NCellID, ...
                    'SubcarrierSpacing', bwp.SubcarrierSpacing, ...
                    'CyclicPrefix', bwp.CyclicPrefix, ...
                    'NSizeGrid', bwp.NSizeBWP, ...
                    'NStartGrid', bwp.NStartBWP);
                pusch = nrPUSCHConfig('NSizeBWP', bwp.NSizeBWP, ...
                    'NStartBWP', bwp.NStartBWP, ...
                    'Modulation', ch.Modulation, ...
                    'NumLayers', ch.NumLayers, ...
                    'NumAntennaPorts', ch.NumAntennaPorts, ...
                    'MappingType', ch.MappingType, ...
                    'SymbolAllocation', ch.SymbolAllocation, ...
                    'TransformPrecoding', ch.TransformPrecoding, ...
                    'TransmissionScheme', ch.TransmissionScheme, ...
                    'PRBSet', ch.PRBSet, ...
                    'NID', ch.NID, ...
                    'RNTI', ch.RNTI, ...
                    'DMRS', ch.DMRS);
                [~, modinfo] = nrPUSCHIndices(carrier, pusch);
                puschDlg.PayloadSize  = nrTBS(ch.Modulation, ch.NumLayers, length(ch.PRBSet), modinfo.NREPerPRB, ch.TargetCodeRate, ch.XOverhead);

            catch exc
                obj.errorFromException(exc);
            end

        end

        function frcSelected(obj,~,event)
            % Called when an FRC is selected through the pop-up window

            obj.updateErrorCache([],[]);
            frc = event.Data.FRC;
            duplexMode = event.Data.DuplexMode;

            obj.pSelectedFRCName = frc;
            obj.pSelectedFRCMCS = event.Data.MCS;
            obj.pSelectedFRCDuplexMode = duplexMode;
            obj.pIsCustom = event.Data.isCustom;

            customTDD = struct();
            customTDD.referenceSubcarrierSpacing = event.Data.ReferenceSubcarrierSpacing;
            customTDD.dl_UL_TransmissionPeriodicity = event.Data.TransmissionPeriodicity;
            customTDD.nrofDownlinkSlots = event.Data.NumDownlinkSlots;
            customTDD.nrofUplinkSlots = event.Data.NumUplinkSlots;
            customTDD.nrofDownlinkSymbols = event.Data.NumDownlinkSym;
            customTDD.nrofUplinkSymbols = event.Data.NumUplinkSym;
            customTDD.useDefault = false;
            customTDD.transmitSpecialSlot = event.Data.EnableSpecialSlots;

            % Try to maintain original channel bandwidth
            refObj = wirelessWaveformApp.internal.hNRReferenceWaveformGenerator(frc,[],[],duplexMode,1,"15.2.0",[],[],customTDD);
            cfg = refObj.Config;
            maxNRB = nr5g.internal.wavegen.getNumRB(cfg.FrequencyRange,cfg.SCSCarriers{1}.SubcarrierSpacing,obj.ChannelBandwidth);
            numRBs = length(cfg.PUSCH{1}.PRBSet);
            if numRBs<maxNRB
                % Keep old channel bandwidth as PRB set is valid
                cfg.ChannelBandwidth = obj.ChannelBandwidth;
            else
                % Use new channel bandwidth from cfg
                maxNRB = nr5g.internal.wavegen.getNumRB(cfg.FrequencyRange,cfg.SCSCarriers{1}.SubcarrierSpacing,cfg.ChannelBandwidth);
            end

            % Update General Configuration accordingly
            obj.pSkipUpdateGrid = true; % no need to update grid and info here, as PUSCH is not updated yet
            obj.FrequencyRange = cfg.FrequencyRange;
            obj.SubcarrierSpacing = cfg.SCSCarriers{1}.SubcarrierSpacing;
            obj.ChannelBandwidth = cfg.ChannelBandwidth;
            obj.NumSubframes = cfg.NumSubframes;
            obj.NCellID = cfg.NCellID;
            obj.pSkipUpdateGrid = false;

            % Update carrier in cfg if necessary
            cfg.SCSCarriers{1}.NSizeGrid = maxNRB;
            cfg.SCSCarriers{1}.NStartGrid = 0;
            cfg.BandwidthParts{1}.NSizeBWP = maxNRB;
            cfg.BandwidthParts{1}.NStartBWP = 0;

            % Apply PUSCH configuration to GUI
            obj.applyFRCPUSCHConfiguration(cfg);

        end

        % Custom callback of ChannelBandwidth
        function bwChanged(obj,~)
            % This method is called by the setter of ChannelBandwidth

            if ~obj.Parent.AppObj.Initializing
                if ~obj.pSkipUpdateGrid
                    % Update PUSCH resource allocation when necessary, and then
                    % update the cached config, grid, info and channel
                    % bandwidth view (no need to call updates here as PUSCH
                    % dialog does it)
                    obj.updatePUSCHResourceAllocation(true);
                else
                    % Check the current configuration object before updating the
                    % channel view
                    updateAppConfigDiagnostic(obj, []); % This performs config checks under the hood already
                end
            end

        end

        function out = getChannelName(obj)
            puschDlg = obj.getPUSCHDialog();
            out = getPUSCHFRCChannelName(puschDlg);
        end
    end

    methods(Access=private)

        % Apply PUSCH configuration of selected FRC
        function applyFRCPUSCHConfiguration(obj,cfg)

            % Update config cache
            obj.pConfigCache = cfg;

            % Update PUSCH Configuration
            puschDlg = obj.getPUSCHDialog();
            if ~isempty(puschDlg) % to prevent error when initializing
                if numel(cfg.PUSCH) > 1
                    puschDlg.EnablePUSCH2GUI.Value = true;
                end
                puschDlg.applyPUSCHConfig(cfg,true);
            end

        end

        % Update PUSCH resource allocation when carrier changes
        function updatePUSCHResourceAllocation(obj,bwUpdated)
            % Called when carrier is changed, so PUSCH resource allocation
            % should be updated when necessary

            if nargin<2
                bwUpdated = false;
            end
            cfg = obj.pConfigCache;

            cfg.FrequencyRange = obj.FrequencyRange;
            cfg.ChannelBandwidth = obj.ChannelBandwidth;
            cfg.NumSubframes = obj.NumSubframes;

            maxNRB = nr5g.internal.wavegen.getNumRB(obj.FrequencyRange,obj.SubcarrierSpacing,obj.ChannelBandwidth);

            numPUSCH = numel(cfg.PUSCH);
            M = zeros(numPUSCH,1);
            invalidPRB = false(numPUSCH,1);
            for x = 1:numPUSCH
                % Check if PRBSet is too big to fit the new bandwidth
                invalidPRB(x) = numel(cfg.PUSCH{x}.PRBSet) > maxNRB;
                % Get the max of PRBSet value for the popup message
                M(x) = max(cfg.PUSCH{x}.PRBSet);
            end

            if any(invalidPRB)
                % When carrier is not big enough, pop-up to ask whether
                % PUSCH should be updated automatically
                confirmText = getString(message('nr5g:waveformGeneratorApp:FRCConfirmUpdatePUSCHText',max(M),maxNRB));
                confirmTitle = getString(message('nr5g:waveformGeneratorApp:FRCConfirmUpdatePUSCHTitle'));
                confirmOptions = {getString(message('nr5g:waveformApp:Yes')),getString(message('nr5g:waveformApp:No'))};
                fig = obj.getParent.AppObj.pParameters.Layout.Parent;
                selection = uiconfirm(fig, confirmText, confirmTitle, 'Options', confirmOptions);
                if strcmpi(selection,getString(message('nr5g:waveformApp:No')))
                    % Forfeit the change and stop further processing
                    obj.pSkipUpdateGrid = true;
                    obj.FrequencyRange = obj.pConfigCache.FrequencyRange;
                    obj.SubcarrierSpacing = obj.pConfigCache.SCSCarriers{1}.SubcarrierSpacing;
                    obj.ChannelBandwidth = obj.pConfigCache.ChannelBandwidth;
                    obj.pSkipUpdateGrid = false;
                    return;
                end
            end

            cfg.SCSCarriers{1}.SubcarrierSpacing = obj.SubcarrierSpacing;
            cfg.SCSCarriers{1}.NSizeGrid = maxNRB;
            cfg.SCSCarriers{1}.NStartGrid = 0;

            cfg.BandwidthParts{1}.SubcarrierSpacing = obj.SubcarrierSpacing;
            cfg.BandwidthParts{1}.NSizeBWP = maxNRB;
            cfg.BandwidthParts{1}.NStartBWP = 0;

            puschDlg = obj.getPUSCHDialog();
            if ~isempty(puschDlg) % to prevent error when initializing
                % As only valid BWs are offered, changing BW will not
                % extend beyond FRC
                puschDlg.updatePUSCHConfig(cfg,false,~bwUpdated);
            end


        end

        function dlg = getPUSCHDialog(obj)

            className = wirelessWaveformGenerator.nr5G_FRC_UL_Dialog.pPUSCHClassName;
            if isKey(obj.Parent.DialogsMap,className)
                dlg = obj.Parent.DialogsMap(className);
            else
                dlg = []; % app initializing
            end

        end

        function frcSelectButtonCallback(obj)
            frcname = obj.pSelectedFRCName;
            dplex = obj.pSelectedFRCDuplexMode;
            obj.FRCSelectButton.popup(frcname,dplex);
        end

    end

    methods (Access = private, Static = true)

        function config = getDefaultConfig(~)

            refObj = wirelessWaveformApp.internal.hNRReferenceWaveformGenerator('G-FR1-A1-1',40,15,'FDD',1,"15.2.0");
            config = refObj.Config;

        end

    end

end

%% Local functions
function config = mapStructToConfig(config,struct)

    % Configurable parameters

    config.FrequencyRange       = struct.FrequencyRange;
    config.ChannelBandwidth     = struct.ChannelBandwidth;
    config.NumSubframes         = str2double(struct.NumSubframes);
    config.NCellID              = str2double(struct.NCellID);
    config.WindowingPercent     = str2double(struct.WindowingPercent);
    config.SampleRate           = str2double(struct.SampleRate);
    config.CarrierFrequency     = str2double(struct.CarrierFrequency);

    if isfield(struct,'PUSCH')
        % Post-R2024b sessions

        config.PUSCH{1} = struct.PUSCH;
        if isfield(struct,'PUSCH2')
            config.PUSCH{2} = struct.PUSCH2;
        end
        config.SCSCarriers{1}.SubcarrierSpacing         = struct.SubcarrierSpacing;
        config.BandwidthParts{1}.SubcarrierSpacing      = struct.SubcarrierSpacing;

    elseif isfield(struct,'PUSCHLayers')
        % R2024b sessions

        config.SCSCarriers{1}.SubcarrierSpacing         = struct.SubcarrierSpacing;
        config.BandwidthParts{1}.SubcarrierSpacing      = struct.SubcarrierSpacing;
        config.PUSCH{1}.RNTI                            = str2double(struct.RNTI);
        config.PUSCH{1}.EnablePTRS                      = struct.PTRS;
        config.PUSCH{1}.NumLayers                       = struct.PUSCHLayers;
        config.PUSCH{1}.NumAntennaPorts                 = config.PUSCH{1}.NumLayers;
        config.PUSCH{1}.MappingType                     = struct.PUSCHMappingType;
        config.PUSCH{1}.SymbolAllocation                = [struct.PUSCHStartSymbol struct.PUSCHSymbolLength];
        config.PUSCH{1}.SlotAllocation                  = evalin('base',struct.PUSCHSlotAllocation);
        config.PUSCH{1}.Period                          = str2double(struct.PUSCHPeriod);
        config.PUSCH{1}.PRBSet                          = str2double(struct.RBOffset)+(0:(str2double(struct.PUSCHNRB)-1));
        config.PUSCH{1}.TransformPrecoding              = struct.PUSCHTransformPrecoding;
        config.PUSCH{1}.DMRS.DMRSConfigurationType      = struct.DMRSConfigurationType;
        config.PUSCH{1}.DMRS.DMRSTypeAPosition          = struct.DMRSTypeAPosition;
        config.PUSCH{1}.DMRS.DMRSAdditionalPosition     = struct.DMRSAdditionalPosition;

        if isfield(struct,'PUSCHModulation')
            % Modulation and TargetCodeRate is only saved for custom
            % configuration, as for FRCs such information is already stored
            % in the FRC name
            config.PUSCH{1}.Modulation                  = struct.PUSCHModulation;
            config.PUSCH{1}.TargetCodeRate              = evalin('base',struct.PUSCHTargetCodeRate);
        end

    
    else
        % Pre-R2024b sessions

        config.PUSCH{1}.PRBSet                          = str2double(struct.RBOffset)+(0:(numel(config.PUSCH{1}.PRBSet)-1));
        config.PUSCH{1}.RNTI                            = str2double(struct.RNTI);
        config.PUSCH{1}.EnablePTRS                      = struct.PTRS;
    end

end
