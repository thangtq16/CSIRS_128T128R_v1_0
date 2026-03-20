classdef nr5G_Main_Base_Dialog < handle & ...
        wirelessWaveformApp.internal.ComponentBanner & ... % Banner to display error/warning messages
        wirelessWaveformApp.internal.UpdateDialogProperty & ... % Facility to update the properties in the dialog after having validated them
        wirelessWaveformApp.internal.UpdateAppStatus % Utility features like updateGrid/updateCache
    % Common base dialog for properties in Main tab of all configurations (Full
    % & presets)

    %   Copyright 2018-2024 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
        FrequencyRange    % FR1 (410 MHz - 7.125 GHz) or FR2 (24.25 GHz - 71 GHz)
        ChannelBandwidth  % Channel bandwidth
        SubcarrierSpacing % Subcarrier spacing
        DuplexMode        % Duplexing mode
    end

    properties (Constant, Hidden)
        CBWFR1 = cellstr(string([5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100]));
        CBWFR2 = cellstr(string([50, 100, 200, 400, 800, 1600, 2000]));

        SCSFR1 = cellstr(string([15, 30, 60]));
        SCSFR2 = cellstr(string([60, 120, 480, 960]));
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Dependent, Access = public)
        % Properties needed by the base class
        CurrentDialog
    end

    properties (Hidden)
        configFcn = @struct
        configGenFcn = @struct
        configGenVar = 'var'

        % Cache the current nrXLCarrierConfig to avoid getting the
        % configuration every time updateGrid and updateREVisual are called
        cachedCfg

        LabelType = 'charEdit'
        LabelLabel
        LabelGUI

        FrequencyRangeType = 'charPopup'
        FrequencyRangeDropDown = {'FR1 (410 MHz - 7.125 GHz)', 'FR2 (24.25 GHz - 71 GHz)'}
        FrequencyRangeGUI
        FrequencyRangeLabel

        ChannelBandwidthType = 'numericPopup'
        ChannelBandwidthDropDown = wirelessWaveformApp.nr5G_Main_Base_Dialog.CBWFR1
        ChannelBandwidthGUI
        ChannelBandwidthLabel

        SubcarrierSpacingType = 'numericPopup'
        SubcarrierSpacingDropDown = wirelessWaveformApp.nr5G_Main_Base_Dialog.SCSFR1
        SubcarrierSpacingGUI
        SubcarrierSpacingLabel

        DuplexModeType = 'charPopup'
        DuplexModeDropDown = {'FDD', 'TDD'}
        DuplexModeGUI
        DuplexModeLabel

        NumSubframesType = 'numericEdit'
        NumSubframesGUI
        NumSubframesLabel

        InitialNSubframeType = 'numericEdit'
        InitialNSubframeGUI
        InitialNSubframeLabel

        NCellIDType = 'numericEdit'
        NCellIDGUI
        NCellIDLabel

        RNTIType = 'numericEdit'
        RNTIGUI
        RNTILabel

        AllocatedPRBType = 'numericEdit'
        AllocatedPRBGUI
        AllocatedPRBLabel

        NLayersType = 'numericPopup'
        NLayersDropDown = {'1', '2', '3', '4'}
        NLayersGUI
        NLayersLabel

        WindowingSourceType = 'charPopup'
        WindowingSourceDropDown = {'Auto', 'Custom'}
        WindowingSourceGUI
        WindowingSourceLabel
        WindowingPercentType = 'numericEdit'
        WindowingPercentGUI
        WindowingPercentLabel

        SampleRateSourceType = 'charPopup'
        SampleRateSourceDropDown = {'Auto', 'Custom'}
        SampleRateSourceGUI
        SampleRateSourceLabel
        SampleRateType = 'numericEdit'
        SampleRateGUI
        SampleRateLabel

        PhaseCompensationType = 'checkbox'
        PhaseCompensationGUI
        PhaseCompensationLabel
        CarrierFrequencyType = 'numericEdit'
        CarrierFrequencyGUI
        CarrierFrequencyLabel

        gridSet
        isDownlink = true;
    end

    properties (Abstract)
        Callback
    end

    methods (Abstract)
        getParent
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_Main_Base_Dialog(parent)
            cfg = nrDLCarrierConfig;
            obj@wirelessWaveformApp.internal.UpdateDialogProperty(parent,cfg); % call base constructor

            obj.FrequencyRangeGUI.(obj.Callback)  = @(a,b) frChangedGUI(obj, []);
            if any(contains(displayOrder(obj), 'SubcarrierSpacing'))
                % UL FRC does not have SCS control
                obj.SubcarrierSpacingGUI.(obj.Callback)  = @(a,b) scsChangedGUI(obj, []);
                scsChanged(obj);  % to initiate available options
            end
            obj.cachedCfg = cfg;

            % Add callbacks
            obj.NumSubframesGUI.(obj.Callback)        = @(src,evnt) updateProperty(obj,src,evnt,"NumSubframes");
            obj.InitialNSubframeGUI.(obj.Callback)    = @(src,evnt) updateProperty(obj,src,evnt,"InitialNSubframe");
            obj.NCellIDGUI.(obj.Callback)             = @(src,evnt) nCellChanged(obj,src,evnt);
            obj.RNTIGUI.(obj.Callback)                = @(src,evnt) rntiChanged(obj, src, evnt);
            obj.ChannelBandwidthGUI.(obj.Callback)    = @(src,evnt) bwChanged(obj, []);
            obj.WindowingSourceGUI.(obj.Callback)     = @(src,evnt) windowingChangedGUI(obj, []);
            obj.WindowingPercentGUI.(obj.Callback)    = @(src,evnt) windowingPercentChangedGUI(obj, src, evnt);
            obj.SampleRateSourceGUI.(obj.Callback)    = @(src,evnt) srChangedGUI(obj, []);
            obj.SampleRateGUI.(obj.Callback)          = @(src,evnt) sampleRateChangedGUI(obj, src, evnt);
            obj.PhaseCompensationGUI.(obj.Callback)   = @(src,evnt) phaseCompChangedGUI(obj, []);
            obj.CarrierFrequencyGUI.(obj.Callback)    = @(src,evnt) carrierFrequencyChangedGUI(obj, src, evnt);
            obj.DuplexModeGUI.(obj.Callback)          = @(src,evnt) DuplexModeChanged(obj);

        end

        function adjustDialog(obj)
            obj.FrequencyRangeGUI.(obj.Callback)  = @(a,b) frChanged(obj, []);
            if any(contains(displayOrder(obj), 'SubcarrierSpacing'))
                % UL FRC does not have SCS control
                obj.SubcarrierSpacingGUI.(obj.Callback)  = @(a,b) scsChanged(obj, []);
                scsChanged(obj);  % to initiate available options
            end
            % Make sure the Label tag is unique
            obj.LabelGUI.Tag = 'WaveLabel';
        end

        function restoreDefaults(obj)
            % Label is taken care of in nr5G_DL_dialog and nr5G_UL_Dialog
            obj.FrequencyRange    = 'FR1';
            obj.ChannelBandwidth  = 40;
            obj.SubcarrierSpacing = 15;
            obj.DuplexMode        = 'FDD';
        end

        function sr = getSampleRate(obj)
            cfgObj = getConfiguration(obj);
            sr = nr5g.internal.wavegen.maxSampleRate(cfgObj);
        end
    end

    % Methods specific to this class and that are currently used by
    % neighboring classes
    methods (Access = public)
        function updateAppConfigDiagnostic(obj, e)
            % Update the banner to display any error related to the current
            % configuration or to clear any previous error that no longer apply
            arguments
                obj
                e {mustBeScalarOrEmpty} = [];
            end
            try
                if isempty(e)
                    cfg = getConfiguration(obj);
                    validateConfig(cfg);
                    updateConfigDiagnostic(obj, ""); % Clear any configuration-related message
                else
                    rethrow(e);
                end
            catch ME
                % Add the message to the banner
                updateConfigDiagnostic(obj, ME.message, MessageType="error");
            end
        end
    end

    % Protected methods
    methods (Access = protected)
        function nCellChanged(obj, src, evnt)
            % First, check that the standalone value is correct
            ME = updateProperty(obj, src, evnt, "NCellID");

            % If there is no error and this is a DL waveform type, check if SIB1 needs updating
            if isempty(ME) && (isa(obj.CurrentDialog,'wirelessWaveformGenerator.nr5G_DL_Dialog'))
                ssbDataSourceClassName = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
                if isKey(obj.getParent.DialogsMap,ssbDataSourceClassName)
                    dataSourceDLG = obj.getParent.DialogsMap(ssbDataSourceClassName);
                    % Update Sib1 with new Shift Index when NCell ID
                    % changes. Noop if SIB1 not enabled.
                    sib1NCellIDUpdate(dataSourceDLG)
                end
            end
        end

        function rntiChanged(obj, src, evnt)
            function validationFcn(in)
                validateattributes(in, {'numeric'}, {'real', 'integer', 'nonnegative', 'scalar', '<=', 2^16-1}, '', 'radio network temporary identifier (RNTI)');
            end
            updateProperty(obj, src, evnt, "RNTI", ValidationFunction=@validationFcn);
        end

        function DuplexModeChanged(obj)
            updateInfo(obj);

            updateGrid(obj);
        end

    end

    % Protect methods for derived classes
    methods (Access=protected)

        function bwChanged(obj, ~)
            % Check the current configuration object before updating the
            % channel view
            updateAppConfigDiagnostic(obj, []); % This performs config checks under the hood already

            updateInfo(obj);

            updateGrid(obj);
            updateChannelBandwidthView(obj);
        end

        function updateInfo(~)
            % no op for TM
        end

        function frChangedBase(obj)
            if strcmp(obj.FrequencyRange, 'FR1')
                obj.ChannelBandwidthGUI.(obj.DropdownValues) = wirelessWaveformApp.nr5G_Main_Base_Dialog.CBWFR1;
                obj.SubcarrierSpacingGUI.(obj.DropdownValues) = wirelessWaveformApp.nr5G_Main_Base_Dialog.SCSFR1;
            else % FR2
                obj.ChannelBandwidthGUI.(obj.DropdownValues) = wirelessWaveformApp.nr5G_Main_Base_Dialog.CBWFR2;
                obj.SubcarrierSpacingGUI.(obj.DropdownValues) = wirelessWaveformApp.nr5G_Main_Base_Dialog.SCSFR2;
            end
            obj.ChannelBandwidth = 50; %MHz
        end

        function scsChanged(obj, ~)
            % remove invalid Channel Bandwidth options
            if strcmp(obj.FrequencyRange, 'FR1')
                if obj.SubcarrierSpacing == 15
                    cbwFR1Range = 1:10;
                elseif obj.SubcarrierSpacing == 30
                    cbwFR1Range = 1:length(wirelessWaveformApp.nr5G_Main_Base_Dialog.CBWFR1);
                else % 60
                    cbwFR1Range = 2:length(wirelessWaveformApp.nr5G_Main_Base_Dialog.CBWFR1);
                end
                obj.ChannelBandwidthGUI.(obj.DropdownValues) = wirelessWaveformApp.nr5G_Main_Base_Dialog.CBWFR1(cbwFR1Range);
            else % FR2
                if obj.SubcarrierSpacing == 60
                    cbwFR2Range = 1:3;
                elseif obj.SubcarrierSpacing == 120
                    cbwFR2Range = 1:4;
                elseif obj.SubcarrierSpacing == 480
                    cbwFR2Range = 4:6;
                else % 960
                    cbwFR2Range = 4:length(wirelessWaveformApp.nr5G_Main_Base_Dialog.CBWFR2);
                end
                obj.ChannelBandwidthGUI.(obj.DropdownValues) = wirelessWaveformApp.nr5G_Main_Base_Dialog.CBWFR2(cbwFR2Range);
            end
        end

        function scsChangedGUI(obj, ~)
            % SCS has changed via a direct GUI interaction: Update everything
            % related to it, including the info panel and the grid and channel
            % visualizations
            scsChanged(obj);
            updateInfo(obj);
            updateGrid(obj);
            updateChannelBandwidthView(obj);
        end

        function windowingChangedGUI(obj, ~)
            setVisible(obj, 'WindowingPercent', ~strcmp(obj.WindowingSource, 'Auto'));

            % Ensure that any potential error due to invalid windowing is
            % shown only if windowing is custom
            updateAppConfigDiagnostic(obj, []); % This performs config checks under the hood already

            layoutUIControls(obj);
        end

        function windowingPercentChangedGUI(obj, src, evnt)
            updateProperty(obj,src,evnt,"WindowingPercent");
        end

        function srChanged(obj, ~)
            % Custom sample rate edit field shows up conditionally (non-Auto SR source)
            customSR = ~strcmp(obj.SampleRateSource, 'Auto');
            if customSR
                if isempty(obj.SampleRate)  % This will read indirectly the GUI text to get a dialog sample rate
                    obj.SampleRate = 5e7;     % And this will write the GUI text... don't show Auto ([]) when on Custom
                end
            end
            setVisible(obj, 'SampleRate', customSR);
        end
        function srChangedGUI(obj, ~)
            srChanged(obj, []);

            % Ensure that any potential error due to invalid sample rate is
            % shown only if sample rate is custom
            updateAppConfigDiagnostic(obj, []); % This performs config checks under the hood already

            layoutUIControls(obj);
        end

        function sampleRateChangedGUI(obj, src, evnt)
            updateProperty(obj,src,evnt,"SampleRate");
        end

        function phaseCompChanged(obj, ~)
            % Carrier Frequency edit field only shows when Phase Compensation
            % checkbox is checked.
            setVisible(obj, 'CarrierFrequency', obj.PhaseCompensation);
        end
        function phaseCompChangedGUI(obj, ~)
            phaseCompChanged(obj, []);

            % Ensure that any potential error due to invalid carrier center
            % frequency is shown only if phase compensation is on
            updateAppConfigDiagnostic(obj, []); % This performs config checks under the hood already

            layoutUIControls(obj);
        end

        function carrierFrequencyChangedGUI(obj, src, evnt)
            updateProperty(obj,src,evnt,"CarrierFrequency");
        end

        % Set modulation step related common values
        function restoreMods(obj)
            obj.WindowingSource   = 'Custom';   % Auto or Custom
            obj.WindowingPercent  = 0;          % Percentage of Windowing Samples / FFTLength
            obj.SampleRateSource  = 'Auto';     % Auto or Custom
            obj.SampleRate        = 5e7;        % Sample rate
            obj.PhaseCompensation = false;      % Checkbox
            obj.CarrierFrequency  = 3.5e9;      % Carrier Frequency for Phase Compensation (Hz)
        end

        % Set the modulation step related common values
        function cfg = getConfigurationMods(obj,cfg)
            if strcmp(obj.WindowingSource, 'Auto')
                windowing = [];
            else
                windowing = obj.WindowingPercent;
            end
            % Also account for difference in Sample Rate API
            if strcmp(obj.SampleRateSource, 'Auto')
                sr = [];
            else
                sr = obj.SampleRate;
            end
            if obj.PhaseCompensation
                cf = obj.CarrierFrequency;
            else
                cf = 0;
            end
            cfg.WindowingPercent = windowing;
            cfg.SampleRate = sr;
            cfg.CarrierFrequency = cf;
        end

        function out = getChannelName(~)
            out = 'Main';
        end
    end

    % Getters/setters
    methods
        function dlg = get.CurrentDialog(obj)
            dlg = obj.getParent.AppObj.pParameters.CurrentDialog;
        end

        function fr = get.FrequencyRange(obj)
            fr = getDropdownVal(obj, 'FrequencyRange');
            fr = fr(1:3); % FR1 or FR2, drop parenthesis containing value
        end
        function set.FrequencyRange(obj, val)
            oldVal = obj.FrequencyRange;
            setDropdownStartingVal(obj, 'FrequencyRange', val)
            if ~strcmp(val, oldVal)
                % avoid expensive setScopeLayout calls if they are not needed
                frChanged(obj);
            end
        end

        function cbw = get.ChannelBandwidth(obj)
            cbw = getDropdownNumVal(obj, 'ChannelBandwidth');
        end
        function set.ChannelBandwidth(obj, val)
            setDropdownNumVal(obj, 'ChannelBandwidth', val);
        end

        function scs = get.SubcarrierSpacing(obj)
            scs = getSCS(obj);
        end
        function scs = getSCS(obj) % allow overrides
            scs = getDropdownNumVal(obj, 'SubcarrierSpacing');
        end
        function set.SubcarrierSpacing(obj, val)
            setDropdownNumVal(obj, 'SubcarrierSpacing', val);
            scsChanged(obj);
        end

        function fr = get.DuplexMode(obj)
            if ~isa(obj.DuplexModeGUI, 'matlab.ui.control.Label')
                fr = obj.DuplexModeGUI.Value;
            else
                fr = obj.DuplexModeGUI.(obj.TextValue);
            end
        end

        function set.DuplexMode(obj, val)
            if ~isa(obj.DuplexModeGUI, 'matlab.ui.control.Label')
                obj.DuplexModeGUI.Value = val;
            else
                obj.DuplexModeGUI.(obj.TextValue) = val;
            end
        end

        function out = get.ChannelName(obj)
            out = getChannelName(obj);
        end
    end

end
