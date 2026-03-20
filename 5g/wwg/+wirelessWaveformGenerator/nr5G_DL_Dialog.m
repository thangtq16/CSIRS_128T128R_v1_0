classdef nr5G_DL_Dialog < wirelessWaveformGenerator.nr5G_Full_Base_Dialog & ...
        wirelessWaveformApp.nr5G_DL_Tabs
    % Downlink-specific content of Full DL 5G wavegen

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Hidden)
        TitleString =  getString(message('nr5g:waveformApp:nrDLTitle'))

        ssBurstFig % The fig of the 2nd tab. Other figs are brought from subclass mixins
        SSBTabName = 'SSB/SIB1';

        mainDLFig
        % Properties that control the visibility of data entries in
        % dialog/table at the time of App creation
        InvisibleSSBurstEntries = {}
        InvisiblePDSCHEntries = {}
        InvisiblePDCCHEntries = {}

        paramSSB % Object to host (parallel) layouts outside the main AppObj.pParameters object
    end

    methods (Static)
        function b = hasLeftFigurePanel(~)
            % Left-side panel can be avoided during launch
            b = false;
        end
        function tag = paramFigureTag(~)
            tag = 'mainDLFig';
        end
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_DL_Dialog(parent)
            obj@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(parent, true); % call base constructor
            obj@wirelessWaveformApp.nr5G_DL_Tabs(); % call base constructor

            % Initialize value of cached configuration object
            obj.cachedCfg = obj.DefaultCfg;

            %% SS Burst:
            % The infrastructure takes care of the Main (1st) tab's placement.
            % Placement of secondary tabs need to be taken care of manually.

            % create 2nd param object, so that there is no interference with
            % 1st one (Main tab) in layoutPanels()
            obj.paramSSB = wirelessAppContainer.Parameters(obj.Parent.WaveformGenerator);

            % Create 2ndary SS Burst Dialog (Data Source configuration) first.
            % This is to ensure the dialog is already present in the DialogsMap,
            % to be used in SS-Burst Dialog
            classNameDS = {'wirelessWaveformGenerator.nr5G_SSB_DataSource'};
            obj.Parent.DialogsMap(classNameDS{1}) = eval([classNameDS{1} '(obj.paramSSB, obj.ssBurstFig)']);
            dlg2 = obj.Parent.DialogsMap(classNameDS{1});

            % create SIB1 Dialog
            classNameSIB1 = {'wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog'};
            obj.Parent.DialogsMap(classNameSIB1{1}) = eval([classNameSIB1{1} '(obj.paramSSB, obj.ssBurstFig)']);
            dlg3 = obj.Parent.DialogsMap(classNameSIB1{1});

            % Create Main SS Burst Dialog
            className = 'wirelessWaveformApp.nr5G_SSB_Dialog';
            obj.Parent.DialogsMap(className) = eval([className '(obj.paramSSB, obj.ssBurstFig, obj.InvisibleSSBurstEntries, [classNameDS, classNameSIB1])']); %#ok<*EVLDOT>
            dlg = obj.Parent.DialogsMap(className);
            obj.paramSSB.CurrentDialog = dlg; % needed for layoutPanels()

            % we won't be using the FilteringDialog created in paramSSB, so remove:
            className = 'wirelessAppContainer.FilteringDialog';
            delete(obj.paramSSB.DialogsMap(className).getPanels);

            % Fix placement of both panels:
            layoutUIControls(dlg);
            layoutUIControls(dlg2);
            layoutUIControls(dlg3);
            layoutPanels(dlg); % same for both

            %% Listeners
            % Create listeners for knowing when any of the configurations stored
            % in table objects changes
            arrayfun(@(x)addlistener(obj.(x),'TableChanged',@(src,event)obj.tableChanged(src,event)),obj.tableObjName);
            arrayfun(@(x)addlistener(obj.(x),'Selection','PostSet',@(src,event)obj.tableChanged(src,event)),obj.tableObjName);

            % Add listener to changes in SCSCarriers as these should
            % trigger an update in the SIB1 configuration, if enabled
            addlistener(obj.bwpTable,'SCSGridValuesChanged',@(src,event)updateSib1(obj,src,event));
        end

        function cleanupDlg(obj)
            % Dialog-specific cleanup when app is closing
            cleanupDlg@wirelessWaveformApp.nr5G_DL_Tabs(obj);
        end

        function extraPanels = getExtraPanels(obj)
            extraPanels = getExtraPanels@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(obj);
            extraPanelTabs = getExtraPanels@wirelessWaveformApp.nr5G_DL_Tabs(obj);
            extraPanels = cat(1,extraPanels,extraPanelTabs);
            dlgSSB{1,1} = obj.Parent.AppObj.pParameters.DialogsMap('wirelessWaveformApp.nr5G_SSB_Dialog');
            dlgSSB{2,1} = obj.Parent.AppObj.pParameters.DialogsMap('wirelessWaveformGenerator.nr5G_SSB_DataSource');
            dlgSSB{3,1} = obj.Parent.AppObj.pParameters.DialogsMap('wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog');
            extraPanels = cat(1,extraPanels,dlgSSB);
        end

        function setupDialog(obj)
            % Actions performed when entering this Full DL wavegen extension

            % Call baseclass method
            setupDialog@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(obj);
        end

        function cfg = getConfiguration(obj)
            % Map all graphical content into an equivalent nrDLCarrierConfig object

            % Call base-class
            cfg = getConfiguration@wirelessWaveformApp.nr5G_DL_Tabs(obj);
        end

        function str = getIconDrawingCommand(obj)
            str = ['disp([''Freq. range: '' ' obj.configGenVar '.FrequencyRange newline ...' newline ...
                '''Bandwidth: '' num2str(' obj.configGenVar '.ChannelBandwidth) '' MHz'' newline ...' newline ...
                '''Subframes: '' num2str(' obj.configGenVar '.NumSubframes) ]);'];
        end

        function [blockName, maskTitleName, waveNameText] = getMaskTextWaveName(obj)
            blockName = ['5G NR ' obj.Parent.WaveformGenerator.pCurrentExtensionType];
            maskTitleName = ['5G NR ' obj.Parent.WaveformGenerator.pCurrentExtensionType ' Waveform Generator'];
            waveNameText = blockName;
        end

        function adjustSpec(obj)
            adjustSpec@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(obj);
            obj.panelFixedSize = true; % prevents panel containing top-level
            % properties (NumSubframes etc.) from
            % filling horizontal space and covering
            % SCS, BWP tables

            % For MATLAB Code generation:
            obj.configGenFcn  = @nrDLCarrierConfig;
            obj.configGenVar  = 'cfgDL';
        end

        function restoreDefaults(obj)
            restoreDefaults@wirelessWaveformApp.nr5G_DL_Tabs(obj);
        end

        function applyConfiguration(dlg, waveConfig)
            % Used on New, Open Session, and openInGenerator

            % Map an nrDLCarrierConfig object to all tables and UI elements.
            % Adjust the elements of the configuration that are not supported.
            waveConfig = applyConfiguration@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(dlg, waveConfig);

            applyConfiguration@wirelessWaveformApp.nr5G_DL_Tabs(dlg, waveConfig);

        end

        function applyConfigSSBurst(obj, cfg)
            % Map an nrDLCarrierConfig.SSBurst = nrWavegenSSBConfig object to all
            % UI elements under the SS Burst tab.
            ssb = cfg.SSBurst;

            % SS Burst main dialog
            className = 'wirelessWaveformApp.nr5G_SSB_Dialog';
            if ~isKey(obj.Parent.DialogsMap, className)
                return; % App initialization
            end

            % Mapping for properties that have 1-1 equivalence between GUI and programmatic API:
            dlgSSB = obj.Parent.DialogsMap(className);
            applyConfigSSBurst@wirelessWaveformApp.nr5G_DL_Tabs(obj,cfg);

            % SS Burst data source dialog
            className = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
            dlgSSBDataSource = obj.Parent.DialogsMap(className);
            % Mapping for properties that have 1-1 equivalence between GUI and programmatic API:
            props = {'CellBarred', 'IntraFreqReselection', ...
                'PDCCHConfigSIB1'};
            for idx = 1:numel(props)
                dlgSSBDataSource.(props{idx}) = ssb.(props{idx});
            end

            % DMRSTypeAPosition is exposed differently in GUI than programmatic API
            dlgSSBDataSource.SSBDMRSTypeAPosition = ssb.DMRSTypeAPosition;

            % Custom data sources are exposed differently in GUI than programmatic API
            if ischar(ssb.DataSource)
                dlgSSBDataSource.DataSource = ssb.DataSource;
            else
                dlgSSBDataSource.DataSource = getString(message('nr5g:waveformGeneratorApp:SSDataSourceUserDefined'));
                dlgSSBDataSource.Payload = ssb.DataSource;
            end

            % Update visibility of SSB data source and SSB controls. The order
            % ensures that the right SubcarrierSpacingCommon is used for the
            % units of the KSSB label.
            updateControlsVisibility(dlgSSBDataSource);
            updateControlsVisibility(dlgSSB);

        end

        function customLoadActions(obj,newData)
            % Apply SIB1 config if it exists
            if isfield(newData.Waveform.Configuration,'SIB1')
                sib1Config = newData.Waveform.Configuration.SIB1;

                dialogs = obj.Parent.WaveformGenerator.pParameters.DialogsMap;
                ssbDataSourceDialog = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
                ssbDialog = dialogs(ssbDataSourceDialog);
                sib1IDsUpdated = updateSIB1ChannelIDs(ssbDialog,newData.Waveform.Configuration); % Update SIB1 channel row ID for internal tracking
                if ~sib1IDsUpdated
                    % Something went wrong with the update of the SIB1
                    % channel row IDs. Likely, the saved session did not
                    % have all the required information.
                    % For safety, disable SIB1.
                    sib1Config.SIB1Enabled = false;
                end
                ssbDialog.Sib1CheckGUI.Value = sib1Config.SIB1Enabled;
                sib1dialog = dialogs('wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog');
                applyConfiguration(sib1dialog,sib1Config);
            end
        end

        function frChanged(obj, ~)
            % Executes when Frequency Range changes (FR1<->FR2)
            frChanged@wirelessWaveformApp.nr5G_DL_Tabs(obj);
            %% All the rest are handled commonly between DL and UL App:
            frChanged@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(obj);
        end

        function w = getColumnWeights(obj, numScopes)
            % The right-side column (PDSCH Advanced config or Spectrum Analyzer)
            % is a bit smaller.
            w = getColumnWeights@wirelessWaveformApp.nr5G_DL_Tabs(obj, numScopes);
        end

        function parent = getParent(obj)
            % needed for distributed classes to fetch protected property / method
            parent = obj.Parent;
        end

        function childDialogs = getDialogs2Reset(obj)
          childDialogs = getDialogs2Reset@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(obj);
          dialogsMap = obj.getParent.DialogsMap;
          if ~isempty(obj.paramSSB)
            childDialogs = [childDialogs {dialogsMap('wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog') ...
              dialogsMap('wirelessWaveformGenerator.nr5G_SSB_DataSource') dialogsMap('wirelessWaveformApp.nr5G_SSB_Dialog')}];
          end
          if ~isempty(obj.paramPXSCH)
            childDialogs = [childDialogs  {dialogsMap(obj.classNamePXSCHAdv) ...
              dialogsMap(obj.classNamePXSCHDMRS) dialogsMap(obj.classNamePXSCHPTRS)}];
          end
        end
    end

    % Protect methods for derived classes
    methods (Access = protected)
        %% Visualization
        function markAllBrokenLinks(obj)
            % Update all broken links by highlighting red color in invalid rows
            markAllBrokenLinks@wirelessWaveformApp.nr5G_DL_Tabs(obj);
        end
        % Link-specific actions when FR changes
        function frChangedForLink(obj)
            frChangedForLink@wirelessWaveformApp.nr5G_DL_Tabs(obj);
        end
    end

    methods (Access = private)
        function updateSib1(obj,~,~)
            if ~obj.Parent.AppObj.pIsLoadingData
                % Avoid firing up the listener callback while loading a
                % saved session
                className = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
                if isKey(obj.Parent.AppObj.pParameters.DialogsMap, className)
                    dlgSSBDS = obj.Parent.AppObj.pParameters.DialogsMap(className);
                    updateSIB1Config(dlgSSBDS);
                end
            end
        end
    end
end

