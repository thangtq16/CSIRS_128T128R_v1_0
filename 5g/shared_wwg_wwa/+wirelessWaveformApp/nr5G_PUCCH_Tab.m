classdef nr5G_PUCCH_Tab < handle
    % Dialog class that handles every graphical aspect related to PUCCH

    % Copyright 2024 The MathWorks, Inc.

    properties
        % Object-specific properties
        pucchFig % Figure containing the dialog
        pxcchSingleChannelFig % Figure containing the side panel
        pucchGridLayout; % Grid containing the table
        pucchTable % Table object

        paramPUCCH % Object to host (parallel) layouts outside the main AppObj.pParameters object
    end

    properties (AbortSet)
        pucchWaveConfig % Cached PUCCH config object
    end

    properties (Constant)
        PXCCHfigureName   = 'PUCCH'; % Channel and figure name
        pxcchExtraFigTag  = 'pxcchSingleChannelFig'; % Side panel figure tag
        classNamePUCCHAdv = 'wirelessWaveformApp.nr5G_PUCCHAdvanced_Dialog';
        classNamePUCCHUCI = 'wirelessWaveformApp.nr5G_PUCCHUCI_Dialog';

        % Store default PUCCH config objects for quicker loading
        defaultPUCCH0Config = nrWavegenPUCCH0Config(Label='PUCCH1');
        defaultPUCCH1Config = nrWavegenPUCCH1Config(Label='PUCCH1');
        defaultPUCCH2Config = nrWavegenPUCCH2Config(Label='PUCCH1');
        defaultPUCCH3Config = nrWavegenPUCCH3Config(Label='PUCCH1');
        defaultPUCCH4Config = nrWavegenPUCCH4Config(Label='PUCCH1');
    end

    properties (Access = private)
        DefaultConfigPUCCH % Store default wavegen config object
    end

    properties (Access = private, Dependent)
        % Format of the selected PUCCH instance
        Format

        % Information about configuration error. If the wavegen
        % configuration for this channel represented by the app has an
        % error, ConfigError stores the MException thrown when trying to
        % update the wavegen configuration object with the app data. If
        % there is no error, ConfigError is empty.
        ConfigError
    end

    methods (Abstract)
        createTableGridLayout % Implemented in nr5G_Full_Layout
        getParent % Implemented in nr5G_UL_Dialog
    end

    methods (Static)
        function chWaveCfg = getDefaultConfigObject(format)
            % Get the default PUCCH configuration object for the given
            % FORMAT.

            switch format
                case 0
                    chWaveCfg = wirelessWaveformApp.nr5G_PUCCH_Tab.defaultPUCCH0Config;
                case 1
                    chWaveCfg = wirelessWaveformApp.nr5G_PUCCH_Tab.defaultPUCCH1Config;
                case 2
                    chWaveCfg = wirelessWaveformApp.nr5G_PUCCH_Tab.defaultPUCCH2Config;
                case 3
                    chWaveCfg = wirelessWaveformApp.nr5G_PUCCH_Tab.defaultPUCCH3Config;
                case 4
                    chWaveCfg = wirelessWaveformApp.nr5G_PUCCH_Tab.defaultPUCCH4Config;
            end
        end
    end

    % Constructor and public methods
    methods (Access = public)
        function obj = nr5G_PUCCH_Tab(defaultWaveConfig, invisibleEntries)
            % Create grid layout that contains the table
            obj.pucchGridLayout = createTableGridLayout(obj,obj.pucchFig,'pucch',1);

            % Construct the table object
            defaultConfigPUCCH = defaultWaveConfig.PUCCH;
            obj.pucchTable = wirelessWaveformApp.nr5G_PUCCH_Table(obj.pucchGridLayout, defaultConfigPUCCH, invisibleEntries);

            % Initialize the cached configuration object
            obj.pucchWaveConfig = defaultConfigPUCCH;
            obj.DefaultConfigPUCCH = defaultConfigPUCCH;
        end

        %% PUCCH Configuration
        function waveConfig = updateCachedConfigPUCCH(obj, action, changedConfigIndex)
            % Update the nrWavegenXConfig configuration object and the XL
            % cached configuration object from the table and the potential
            % side panel.

            if action=="ConfigChange"
                % If one of the current PUCCH instances has changed, update
                % the cached configuration from the table
                waveConfig = getConfiguration(obj);
            else
                % If a PUCCH instance has been added, removed, or duplicated,
                % simply update the cached configuration object as there is
                % no need to read the whole table
                waveConfig = wirelessWaveformApp.internal.Utility.updateConfigDimension(obj.pucchWaveConfig, obj.DefaultConfigPUCCH, action, changedConfigIndex, obj.pucchTable.AllIDs);
            end

            % Turn off interlacing in case SCS and BWP tables have been
            % modified and interlacing is no longer supported for some
            % instances
            waveConfig = turnOffUnsupportedInterlacing(obj, waveConfig);

            % Update the cache
            obj.pucchWaveConfig = waveConfig;
        end

        function applyConfigPUCCH(obj, chWaveCfg)
            % Apply configuration when starting a new session, loading an
            % existing session, or using openInGenerator

            if isempty(obj.pucchTable)
                return; % App initialization
            end

            % Force the input configuration to be a row vector
            ch = chWaveCfg.PUCCH(:)';

            % Disabled default if empty. This is possible only when using
            % openInGenerator.
            if isempty(ch)
                ch = obj.DefaultConfigPUCCH;
                ch{1}.Enable = false;
            end

            % Make sure the advanced panel exists.
            % This is a no-op method if the side panel already exists but
            % it does create one if this is the point of ingress in the app
            % (e.g., if the user opened the app through openInGenerator).
            % For PUCCH, this is important to ensure that the creation of
            % the side panel does not wipe out the input configuration.
            createSidePanel(obj, 'PUCCH');

            % Update the cache
            obj.pucchWaveConfig = ch;

            % Map the channel configuration to the table
            applyConfiguration(obj.pucchTable, ch);

            % Update the side panel
            mapCache2SidePanelPUCCH(obj);
            updateControlsVisibilityPUCCH(obj);
        end

        function out = hasConfigErrorPUCCH(obj)
            % Return the state of the configuration for this channel. The
            % output is true if the app represents an invalid wavegen
            % configuration for the specified channel, and false otherwise.
            out = hasConfigError(obj.pucchTable);
        end

        function e = getConfigErrorPUCCH(obj)
            % Return the wavegen configuration error corresponding to this
            % channel, if any. If the app represents a valid wavegen
            % configuration for this channel, the output is empty.
            e = obj.ConfigError;
        end

        %% Side panel
        function createAdvancedTabPUCCH(obj)
            % Create 2nd param object, so that there is no interference with
            % 1st one in layoutPanels()
            appObj = getParent(obj);
            obj.paramPUCCH = wirelessAppContainer.Parameters(appObj.WaveformGenerator);
            % Get the list of invisible properties, if any, passed as an
            % input to the nr5G_DL_Tabls/nr5G_UL_Tabs constructor. This is
            % saved in the property InvisiblePUCCHEntries.
            invisibleProps = obj.InvisiblePUCCHEntries; %#ok<NASGU>

            % Add UCI panel first, to ensure that the dialog is already
            % present in the DialogsMap, to be used in Advanced Dialog
            appObj.DialogsMap(obj.classNamePUCCHUCI) = eval([obj.classNamePUCCHUCI '(obj.paramPUCCH, obj.pxcchSingleChannelFig, invisibleProps)']);
            dlg2 = appObj.DialogsMap(obj.classNamePUCCHUCI);

            % Advanced properties
            appObj.DialogsMap(obj.classNamePUCCHAdv) = eval([obj.classNamePUCCHAdv '(obj.paramPUCCH, obj.pxcchSingleChannelFig, invisibleProps)']);
            dlg = appObj.DialogsMap(obj.classNamePUCCHAdv);
            obj.paramPUCCH.CurrentDialog = dlg; % needed for layoutPanels()

            % Arrange placements of all right-side PUCCH panels
            layoutUIControls(dlg);
            layoutUIControls(dlg2);
            layoutPanels(dlg); % same for all
        end

        function mapCache2SidePanelPUCCH(obj)
            % This method is invoked when there is a need to update the right
            % side panel with contents corresponding to a newly selected row

            % Update advanced panel and get the configuration object of the
            % selected PUCCH instance from the cache
            updateSidePanelTitles(obj);
            cfgIdx = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pucchTable.Selection);
            pucchCfg = obj.pucchWaveConfig{cfgIdx};
            format = obj.Format;

            % Set common properties
            dlgAdv = getSidePanelDialog(obj, "Advanced");
            dlgAdv.Label             = pucchCfg.Label;
            dlgAdv.FrequencyHopping  = pucchCfg.FrequencyHopping;
            dlgAdv.SecondHopStartPRB = pucchCfg.SecondHopStartPRB;

            % Format-specific properties
            switch format
                case 0
                    dlgAdv.PUCCHGroupHopping = pucchCfg.GroupHopping;
                    dlgAdv.HoppingID = pucchCfg.HoppingID;
                    dlgAdv.InitialCyclicShift = pucchCfg.InitialCyclicShift;
                    dlgAdv.Interlacing_f0123 = pucchCfg.Interlacing;
                    dlgAdv.InterlaceIndex_f0123 = pucchCfg.InterlaceIndex;
                    dlgAdv.RBSetIndex_f0123 = pucchCfg.RBSetIndex;
                case 1
                    dlgAdv.PUCCHGroupHopping = pucchCfg.GroupHopping;
                    dlgAdv.HoppingID = pucchCfg.HoppingID;
                    dlgAdv.InitialCyclicShift = pucchCfg.InitialCyclicShift;
                    dlgAdv.OCCI_f1 = pucchCfg.OCCI;
                    dlgAdv.DMRSPower = pucchCfg.DMRSPower;
                    dlgAdv.Interlacing_f0123 = pucchCfg.Interlacing;
                    dlgAdv.InterlaceIndex_f0123 = pucchCfg.InterlaceIndex;
                    dlgAdv.RBSetIndex_f0123 = pucchCfg.RBSetIndex;
                case 2
                    dlgAdv.NID0 = pucchCfg.NID0;
                    dlgAdv.DMRSPower = pucchCfg.DMRSPower;
                    dlgAdv.Interlacing_f0123 = pucchCfg.Interlacing;
                    dlgAdv.InterlaceIndex_f0123 = pucchCfg.InterlaceIndex;
                    dlgAdv.RBSetIndex_f0123 = pucchCfg.RBSetIndex;
                    dlgAdv.SpreadingFactor_f23 = pucchCfg.SpreadingFactor;
                    dlgAdv.OCCI_f23 = pucchCfg.OCCI;
                case 3
                    dlgAdv.Modulation_f34 = pucchCfg.Modulation;
                    dlgAdv.PUCCHGroupHopping = pucchCfg.GroupHopping;
                    dlgAdv.HoppingID = pucchCfg.HoppingID;
                    dlgAdv.AdditionalDMRS = pucchCfg.AdditionalDMRS;
                    dlgAdv.DMRSUplinkTransformPrecodingR16 = pucchCfg.DMRSUplinkTransformPrecodingR16;
                    dlgAdv.NID0 = pucchCfg.NID0;
                    dlgAdv.DMRSPower = pucchCfg.DMRSPower;
                    dlgAdv.Interlacing_f0123 = pucchCfg.Interlacing;
                    dlgAdv.InterlaceIndex_f0123 = pucchCfg.InterlaceIndex;
                    dlgAdv.RBSetIndex_f0123 = pucchCfg.RBSetIndex;
                    dlgAdv.SpreadingFactor_f23 = pucchCfg.SpreadingFactor;
                    dlgAdv.OCCI_f23 = pucchCfg.OCCI;
                otherwise % format 4
                    dlgAdv.Modulation_f34 = pucchCfg.Modulation;
                    dlgAdv.PUCCHGroupHopping = pucchCfg.GroupHopping;
                    dlgAdv.HoppingID = pucchCfg.HoppingID;
                    dlgAdv.SpreadingFactor = pucchCfg.SpreadingFactor;
                    dlgAdv.OCCI_f4 = pucchCfg.OCCI;
                    dlgAdv.AdditionalDMRS = pucchCfg.AdditionalDMRS;
                    dlgAdv.DMRSUplinkTransformPrecodingR16 = pucchCfg.DMRSUplinkTransformPrecodingR16;
                    dlgAdv.NID0 = pucchCfg.NID0;
                    dlgAdv.DMRSPower = pucchCfg.DMRSPower;
            end

            % UCI panel
            dlgUCI = getSidePanelDialog(obj, "UCI");
            switch format
                case {0, 1, 2}
                    numUCIBits = 'NumUCIBits_f01';
                    if format == 0
                        if ischar(pucchCfg.DataSourceSR)
                            dlgUCI.DataSourceSR = pucchCfg.DataSourceSR;
                        else
                            dlgUCI.DataSourceSR = 'User-defined';
                            dlgUCI.CustomDataSourceSR = pucchCfg.DataSourceSR;
                        end
                    elseif format == 2
                        numUCIBits = 'NumUCIBits_f2';
                    end
                    dlgUCI.(numUCIBits)  = pucchCfg.NumUCIBits;
                    % Data source is handled slightly different from programmatic API
                    if ischar(pucchCfg.DataSourceUCI)
                        dlgUCI.DataSourceUCI_f012 = pucchCfg.DataSourceUCI;
                    else
                        dlgUCI.DataSourceUCI_f012 = 'User-defined';
                        dlgUCI.CustomDataSourceUCI_f012 = pucchCfg.DataSourceUCI;
                    end
                otherwise % format 3 or 4
                    dlgUCI.TargetCodeRate  = pucchCfg.TargetCodeRate;
                    dlgUCI.NumUCIBits_f34  = pucchCfg.NumUCIBits;
                    % Data source is handled slightly different from programmatic API
                    if ischar(pucchCfg.DataSourceUCI)
                        dlgUCI.DataSourceUCI_f34 = pucchCfg.DataSourceUCI;
                    else
                        dlgUCI.DataSourceUCI_f34 = 'User-defined';
                        dlgUCI.CustomDataSourceUCI_f34 = pucchCfg.DataSourceUCI;
                    end
                    dlgUCI.NumUCI2Bits = pucchCfg.NumUCI2Bits;
                    if ischar(pucchCfg.DataSourceUCI2)
                        dlgUCI.DataSourceUCI2 = pucchCfg.DataSourceUCI2;
                    else
                        dlgUCI.DataSourceUCI2 = 'User-defined';
                        dlgUCI.CustomDataSourceUCI2 = pucchCfg.DataSourceUCI2;
                    end
            end

            % Update the side panels with the current format
            dlgAdv.Format = format;
            dlgUCI.Format = format;
        end

        % Public getters for the private properties that represents the
        % side panel class names
        function className = getClassNamePUCCHAdv(obj)
            className = obj.classNamePUCCHAdv;
        end
        function className = getClassNamePUCCHUCI(obj)
            className = obj.classNamePUCCHUCI;
        end

        %% Update visibility of dependent parameters in the side panel
        function updateControlsVisibilityPUCCH(obj)
            % Update the visibility of all dependent parameters in the side
            % panel.

            dlgAdv = getSidePanelDialog(obj, "Advanced");
            dlgUCI = getSidePanelDialog(obj, "UCI");
            needRepaint = false;
            needRepaint = updateControlsVisibility(dlgAdv) || needRepaint;
            needRepaint = updateControlsVisibility(dlgUCI) || needRepaint;
            needRepaint = updateCodingDependentVisibility(obj) || needRepaint;

            % Update the visibility, if needed
            if needRepaint
                layoutUIControls(dlgAdv);
                layoutUIControls(dlgUCI);
            end
        end
    end

    % Getters/Setters
    methods
        function val = get.Format(obj)
            currPUCCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pucchTable.Selection);
            val = double(extract(string(class(obj.pucchWaveConfig{currPUCCH})),digitsPattern));
        end

        function e = get.ConfigError(obj)
            % Retrieve configuration error from the table, as the side
            % panel does not hold any configuration that is invalid at set
            % time
            e = getConfigError(obj.pucchTable);
        end
    end

    methods (Access = private)
        function waveCfg = getConfiguration(obj)
            % Get the nrWavegenXConfig configuration object from the table
            % and the potential side panel.

            % Update cache with latest edits
            currPUCCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pucchTable.Selection);
            newFormat = obj.pucchTable.Format(currPUCCH);
            waveCfg = obj.pucchWaveConfig;
            if obj.Format~=newFormat
                % If PUCCH format has changed because of user interaction,
                % set the cached configuration to the default one for the
                % new format
                waveCfg{currPUCCH} = formatChanged(obj, newFormat, currPUCCH);
            else
                waveCfg = updateConfiguration(obj.pucchTable, waveCfg);
                waveCfg = mapSidePanel2CfgObj(obj, waveCfg);
            end
        end

        %% Side panel interaction
        function dlg = getSidePanelDialog(obj, dialogName)
            % Make sure the advanced panel exists
            createSidePanel(obj, 'PUCCH');

            % Retrieve the side panel
            appObj = getParent(obj);
            switch dialogName
                case "Advanced"
                    className = obj.classNamePUCCHAdv;
                case "UCI"
                    className = obj.classNamePUCCHUCI;
            end
            dlg = appObj.DialogsMap(className);
        end

        function updateSidePanelTitles(obj)
            % Update titles of right-side panels based on displayed PUCCH

            % Make sure the advanced panel exists
            createSidePanel(obj, 'PUCCH');

            % Update titles of each panel
            appObj = getParent(obj);
            classes = {obj.classNamePUCCHAdv, obj.classNamePUCCHUCI};
            cfgIdx = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pucchTable.Selection);
            pucchID = num2str(obj.pucchTable.AllIDs(cfgIdx));
            for idx = 1:length(classes)
                thisClass = classes{idx};
                dialog = appObj.WaveformGenerator.pParameters.DialogsMap(thisClass);

                currTitle = getTitle(dialog);

                currID = extract(string(currTitle),digitsPattern);
                currTitle = strrep(currTitle,currID,pucchID);
                dialog.setTitle(currTitle);
                if idx == 1
                    obj.pxcchSingleChannelFig.Name = erase(strrep(currTitle, '(', '- '), ')');
                end
            end
        end

        function cfg = mapSidePanel2CfgObj(obj, cfg)
            % This method is invoked when there is a need to store the edits at
            % the right side panel and store these internally in the cache.

            % Make sure the advanced panel exists
            createSidePanel(obj, 'PUCCH');

            % Get cached configuration
            cfgIdx = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pucchTable.Selection);
            pucchCfg = cfg{cfgIdx};
            format = double(extract(string(class(pucchCfg)),digitsPattern));

            % Set common properties
            dlgAdv = getSidePanelDialog(obj, "Advanced");
            pucchCfg.FrequencyHopping  = dlgAdv.FrequencyHopping;
            pucchCfg.SecondHopStartPRB = dlgAdv.SecondHopStartPRB;

            % Format-specific properties
            switch format
                case 0
                    pucchCfg.GroupHopping = dlgAdv.PUCCHGroupHopping;
                    pucchCfg.HoppingID = dlgAdv.HoppingID;
                    pucchCfg.InitialCyclicShift = dlgAdv.InitialCyclicShift;
                    pucchCfg.Interlacing = dlgAdv.Interlacing_f0123;
                    pucchCfg.InterlaceIndex = dlgAdv.InterlaceIndex_f0123;
                    pucchCfg.RBSetIndex = dlgAdv.RBSetIndex_f0123;
                case 1
                    pucchCfg.GroupHopping = dlgAdv.PUCCHGroupHopping;
                    pucchCfg.HoppingID = dlgAdv.HoppingID;
                    pucchCfg.InitialCyclicShift = dlgAdv.InitialCyclicShift;
                    pucchCfg.OCCI = dlgAdv.OCCI_f1;
                    pucchCfg.DMRSPower = dlgAdv.DMRSPower;
                    pucchCfg.Interlacing = dlgAdv.Interlacing_f0123;
                    pucchCfg.InterlaceIndex = dlgAdv.InterlaceIndex_f0123;
                    pucchCfg.RBSetIndex = dlgAdv.RBSetIndex_f0123;
                case 2
                    pucchCfg.NID0 = dlgAdv.NID0;
                    pucchCfg.DMRSPower = dlgAdv.DMRSPower;
                    pucchCfg.Interlacing = dlgAdv.Interlacing_f0123;
                    pucchCfg.InterlaceIndex = dlgAdv.InterlaceIndex_f0123;
                    pucchCfg.RBSetIndex = dlgAdv.RBSetIndex_f0123;
                    pucchCfg.SpreadingFactor = dlgAdv.SpreadingFactor_f23;
                    pucchCfg.OCCI = dlgAdv.OCCI_f23;
                case 3
                    pucchCfg.Modulation = dlgAdv.Modulation_f34;
                    pucchCfg.GroupHopping = dlgAdv.PUCCHGroupHopping;
                    pucchCfg.HoppingID = dlgAdv.HoppingID;
                    pucchCfg.AdditionalDMRS = dlgAdv.AdditionalDMRS;
                    pucchCfg.DMRSUplinkTransformPrecodingR16 = dlgAdv.DMRSUplinkTransformPrecodingR16;
                    pucchCfg.NID0 = dlgAdv.NID0;
                    pucchCfg.DMRSPower = dlgAdv.DMRSPower;
                    pucchCfg.Interlacing = dlgAdv.Interlacing_f0123;
                    pucchCfg.InterlaceIndex = dlgAdv.InterlaceIndex_f0123;
                    pucchCfg.RBSetIndex = dlgAdv.RBSetIndex_f0123;
                    pucchCfg.SpreadingFactor = dlgAdv.SpreadingFactor_f23;
                    pucchCfg.OCCI = dlgAdv.OCCI_f23;
                otherwise % format 4
                    pucchCfg.Modulation = dlgAdv.Modulation_f34;
                    pucchCfg.GroupHopping = dlgAdv.PUCCHGroupHopping;
                    pucchCfg.HoppingID = dlgAdv.HoppingID;
                    pucchCfg.SpreadingFactor = dlgAdv.SpreadingFactor;
                    pucchCfg.OCCI = dlgAdv.OCCI_f4;
                    pucchCfg.AdditionalDMRS = dlgAdv.AdditionalDMRS;
                    pucchCfg.DMRSUplinkTransformPrecodingR16 = dlgAdv.DMRSUplinkTransformPrecodingR16;
                    pucchCfg.NID0 = dlgAdv.NID0;
                    pucchCfg.DMRSPower = dlgAdv.DMRSPower;
            end

            % UCI panel
            dlgUCI = getSidePanelDialog(obj, "UCI");
            % Format-specific properties
            switch format
                case {0, 1, 2}
                    numUCIBits = dlgUCI.NumUCIBits_f01;
                    if format==0
                        if strcmpi(dlgUCI.DataSourceSR, 'User-defined')
                            pucchCfg.DataSourceSR = dlgUCI.CustomDataSourceSR;
                        else
                            pucchCfg.DataSourceSR = dlgUCI.DataSourceSR;
                        end
                    elseif format==2
                        numUCIBits = dlgUCI.NumUCIBits_f2;
                    end
                    pucchCfg.NumUCIBits  = numUCIBits;
                    % Data source is handled slightly different from programmatic API
                    if strcmpi(dlgUCI.DataSourceUCI_f012, 'User-defined')
                        pucchCfg.DataSourceUCI = dlgUCI.CustomDataSourceUCI_f012;
                    else
                        pucchCfg.DataSourceUCI = dlgUCI.DataSourceUCI_f012;
                    end
                otherwise % format 3 or 4
                    pucchCfg.TargetCodeRate = dlgUCI.TargetCodeRate;
                    pucchCfg.NumUCIBits     = dlgUCI.NumUCIBits_f34;
                    % Data source is handled slightly different from programmatic API
                    if strcmpi(dlgUCI.DataSourceUCI_f34, 'User-defined')
                        pucchCfg.DataSourceUCI = dlgUCI.CustomDataSourceUCI_f34;
                    else
                        pucchCfg.DataSourceUCI = dlgUCI.DataSourceUCI_f34;
                    end
                    pucchCfg.NumUCI2Bits = dlgUCI.NumUCI2Bits;
                    if strcmpi(dlgUCI.DataSourceUCI2, 'User-defined')
                        pucchCfg.DataSourceUCI2 = dlgUCI.CustomDataSourceUCI2;
                    else
                        pucchCfg.DataSourceUCI2 = dlgUCI.DataSourceUCI2;
                    end
            end

            cfg{cfgIdx} = pucchCfg;
        end

        %% Dependent controls visibility
        function needRepaint = updateCodingDependentVisibility(obj)
            % TargetCodeRate, NumUCIBits, NumUCI2Bits, and DataSourceUCI2
            % in UCI only applicable when Coding = true

            needRepaint = false;

            % Adjust visuals only if current row is displayed in
            % "advanced", as the user can toggle the Coding checkbox
            % without actually selecting the row
            currPUCCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pucchTable.Selection);
            if obj.pucchTable.AllIDs(currPUCCH) == getAdvancedPUCCHID(obj)

                % Get Coding value
                format = obj.Format;
                if any(format == [0, 1])
                    % No format 0 or 1 property depends on Coding
                    return;
                end
                coding = obj.pucchWaveConfig{currPUCCH}.Coding;

                % Get UCI property panel
                dlgUCI = getSidePanelDialog(obj, "UCI");
                if format == 2
                    oldVis = isVisible(dlgUCI, 'NumUCIBits_f2');
                    needRepaint = xor(oldVis, coding);
                    if needRepaint
                        setVisible(dlgUCI, 'NumUCIBits_f2', coding);
                    end
                else % format 3 and 4
                    % Update the visibility of Num UCI Bits, UCI Data Source, Num
                    % UCI2 Bits, UCI 2 Data Source and custom data sources for both
                    % UCI and UCI 2
                    needRepaint = updateNumUCIBits_f34Vis(dlgUCI, coding);
                end
            end
        end

        function id = getAdvancedPUCCHID(obj)
            % Get the ID of the PUCCH that is currently displayed at the
            % side panel

            % Look at the number in the title:
            if isKey(obj.getParent.DialogsMap, obj.classNamePUCCHAdv) && isgraphics(obj.getParent.DialogsMap(obj.classNamePUCCHAdv).getPanels)
                dlgAdv = getSidePanelDialog(obj, "Advanced");
                str = getTitle(dlgAdv);
                id = double(extract(string(str),digitsPattern));
            else
                id = NaN;
            end
        end

        function waveConfig = turnOffUnsupportedInterlacing(obj, waveConfig)
            % Check whether interlacing is applicable throughout the config
            % and turn off those not supported

            needUpdate = false;
            for idx = 1:numel(waveConfig)
                % Only update the interlacing property is this instance has
                % interlacing flag on and does not support interlacing
                % anymore
                if ~supportInterlacing(obj, waveConfig{idx})
                    if isprop(waveConfig{idx}, 'Interlacing')
                        if waveConfig{idx}.Interlacing
                            waveConfig{idx}.Interlacing = false; % turn off interlacing
                            needUpdate = true;
                        end
                    else
                        needUpdate = true;
                    end
                    if (((isempty(obj.pucchTable.Selection) && idx == 1) || ...
                            (~isempty(obj.pucchTable.Selection) && idx == obj.pucchTable.Selection(1))) && ...
                            isKey(obj.getParent.DialogsMap,obj.classNamePUCCHAdv))
                        dlg = obj.getParent.DialogsMap(obj.classNamePUCCHAdv);
                        dlg.Interlacing_f0123 = false; % untick interlacing checkbox if currently selected
                    end
                end
            end
            if needUpdate
                applyConfiguration(obj.pucchTable, waveConfig, AllowUIChange=false);
            end
        end

        function flag = supportInterlacing(obj, pucchWaveConfig)
            % Returns true or false whether the current PUCCH instance supports
            % interlacing or not. Only PUCCH instances linked to bandwidth parts
            % with 15 kHz or 30 kHz subcarrier spacing support interlacing.

            if obj.Format == 4
                % PUCCH format 4 does not support interlacing
                flag = false;
            else
                bwpID = pucchWaveConfig.BandwidthPartID;
                availableBWPIDs = cellfun(@(x)(cat(1,[],x.BandwidthPartID)),obj.bwpWaveConfig);
                bwpRowNum = (availableBWPIDs == bwpID);
                if any(bwpRowNum)
                    scs = obj.bwpWaveConfig{bwpRowNum}.SubcarrierSpacing;
                    flag = (scs == 15 || scs == 30);
                else
                    flag = false; % BWP no longer exists - treat as not supporting interlacing
                end
            end
        end

        function chWaveCfg = formatChanged(obj, format, cfgIdx)
            % PUCCH format has changed. Update the side panel to ensure it
            % correctly reflects the new format.

            % Update the wavegen config object to the new format, keeping
            % the enable state and label unmodified
            chWaveCfg = obj.pucchWaveConfig{cfgIdx};
            enable = chWaveCfg.Enable;
            label = chWaveCfg.Label;
            chWaveCfg = wirelessWaveformApp.nr5G_PUCCH_Tab.getDefaultConfigObject(format);
            chWaveCfg.Enable = enable;
            chWaveCfg.Label = label;
            obj.pucchWaveConfig{cfgIdx} = chWaveCfg;

            % Map the updated config object to the side panel
            mapCache2SidePanelPUCCH(obj);
            updateControlsVisibilityPUCCH(obj);
            % Update interlacing-related properties back in the PUCCH table
            dlgAdv = getSidePanelDialog(obj, "Advanced");
            updateTableDisplayInterlacing(dlgAdv);
        end
    end
end
