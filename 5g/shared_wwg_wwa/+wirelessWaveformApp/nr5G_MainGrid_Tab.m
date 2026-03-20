classdef nr5G_MainGrid_Tab < handle
    % Dialog class that handles every graphical aspect related to the grid
    % in the right-side of the Main tab, containing SCS Carriers and BWP,
    % and ICGB button in Uplink

    % Copyright 2024 The MathWorks, Inc.

    properties (Abstract)
        % Properties that are not defined in this class
        FrequencyRange
        ChannelBandwidth
        mainGridLayout % Grid layout container used in the Main panel
    end

    properties
        % Object-specific properties
        carrierGridLayout % Grid containing SCS carrier and BWP
        scsCarriersTable % SCS carriers table object
        bwpTable % BWP table object
    end

    properties (AbortSet)
        scsCarriersWaveConfig % Cached SCS Carriers config object
        bwpWaveConfig % Cached BWP config object
    end

    properties (Access = private)
        % Store default wavegen config objects
        DefaultConfigSCS
        DefaultConfigBWP
        isDownlink
    end

    properties (Access = private, Dependent)
        % Information about configuration error. If the wavegen
        % configuration for this channel represented by the app has an
        % error, ConfigError stores the MException thrown when trying to
        % update the wavegen configuration object with the app data. If
        % there is no error, ConfigError is empty.
        ConfigErrorSCSCarriers
        ConfigErrorBWP
    end

    methods(Abstract)
        % Methods implemented in other classes
        createTableGridLayout % Implemented in nr5G_Full_Layout
        getParent % Implemented in nr5G_DL_Dialog/nr5G_UL_Dialog
        updateChannelBandwidthView % Implemented in nr5G_Full_Base_Dialog/nr5G_Dialog
        updateBWPoptions % Implemented in nr5G_Full_Layout
        markBrokenBWPLinks % Implemented in nr5G_DL_Tabs/nr5G_UL_Tabs
        setVisualState % Implemented in base comms Dialog
        updateConfigDiagnostic % Implemented in ComponentBanner
    end

    % Constructor and public methods
    methods (Access = public)
        function obj = nr5G_MainGrid_Tab(defaultWaveConfig, isDownlink)
            % Create grid layout that contains the SCS Carriers and BWP
            % tables
            createCarrierGridLayout(obj);

            % Construct the SCS Carrier table object
            defaultConfigSCS = defaultWaveConfig.SCSCarriers;
            obj.scsCarriersTable = wirelessWaveformApp.nr5G_SCSCarriers_Table(obj.carrierGridLayout, defaultConfigSCS, isDownlink);
            % Place SCS Carrier table in position (1,1) of the carrier grid layout
            setLayout(obj.scsCarriersTable, Row=1, Column=1);
            % Initialize the cached configuration object
            obj.scsCarriersWaveConfig = defaultConfigSCS;
            obj.DefaultConfigSCS = defaultConfigSCS;
            % Initialize the FrequencyRange internal property of the SCS
            % Carriers table
            updatePropertyValues(obj.scsCarriersTable, PropertyName="FrequencyRange", NewList=obj.FrequencyRange);

            % Construct the BWP table object
            defaultConfigBWP = defaultWaveConfig.BandwidthParts;
            obj.bwpTable = wirelessWaveformApp.nr5G_BWP_Table(obj.carrierGridLayout, defaultConfigBWP, isDownlink);
            % Place BWP table in position (1,2) of the carrier grid layout
            setLayout(obj.bwpTable, Row=1, Column=2);
            % Initialize the cached configuration object
            obj.bwpWaveConfig = defaultConfigBWP;
            obj.DefaultConfigBWP = defaultConfigBWP;
            % Initialize the FrequencyRange internal property of the BWP
            % table
            updatePropertyValues(obj.bwpTable, PropertyName="FrequencyRange", NewList=obj.FrequencyRange);

            obj.isDownlink = isDownlink;
        end

        %% Update Configurations
        function waveConfig = updateCachedConfigSCSCarriers(obj, action, changedConfigIndex)
            % Update the nrSCSCarrierConfig configuration object from the
            % table and the potential side panel.

            if action=="ConfigChange"
                % If one of the current SCS Carriers instances has changed,
                % update the cached configuration from the table.
                waveConfig = obj.scsCarriersWaveConfig;
                waveConfig = updateConfiguration(obj.scsCarriersTable, waveConfig);
            else
                % If an SCS Carrier instance has been added or removed,
                % simply update the cached configuration object as there is
                % no need to read the whole table
                cfg = obj.DefaultConfigSCS;
                if action=="Add"
                    % Update the default configuration for the next
                    % available SCS value

                    % Get SCS value
                    existingSCS = [cat(1,[],obj.scsCarriersWaveConfig{:}).SubcarrierSpacing];
                    if obj.FrequencyRange=="FR1"
                        scsList = extract(wirelessWaveformApp.nr5G_SCSCarriers_Table.DefaultSCSListFR1,digitsPattern);
                    else
                        if obj.isDownlink
                            scsList = extract(wirelessWaveformApp.nr5G_SCSCarriers_Table.DefaultSCSListFR2_DL,digitsPattern);
                        else % Uplink
                            scsList = extract(wirelessWaveformApp.nr5G_SCSCarriers_Table.DefaultSCSListFR2_UL,digitsPattern);
                        end
                    end
                    possibleSCS = cellfun(@(x)str2double(x),scsList);
                    candidateSCS = setdiff(possibleSCS, existingSCS, 'stable'); % remaining
                    scs = candidateSCS(1);
                    cfg{1}.SubcarrierSpacing = scs;

                    % Get grid size and start values
                    [gridSize,gridStart] = getSCSDefaultGridData(obj.scsCarriersTable,string(scs));
                    cfg{1}.NSizeGrid = gridSize;
                    cfg{1}.NStartGrid = gridStart;
                end
                waveConfig = wirelessWaveformApp.internal.Utility.updateConfigDimension(obj.scsCarriersWaveConfig, cfg, action, changedConfigIndex, obj.scsCarriersTable.AllIDs);
            end

            % Update the cache
            obj.scsCarriersWaveConfig = waveConfig;

            % Update the SCS list in the dropdown of the BWP table and
            % update the table itself, if the shown SCS has changed
            previousBWP = obj.bwpWaveConfig;
            scsTableInteraction = (isempty(changedConfigIndex) || ~isnan(changedConfigIndex));
            updateCachedSCSCarrierInBWP(obj,SCSTableInteraction=scsTableInteraction);
            newBWP = obj.bwpWaveConfig;
            if ~isequal(newBWP, previousBWP)
                % The internal cached SCS info for BWP has changed. Update
                % the table accordingly.

                applyConfiguration(obj.bwpTable, obj.bwpWaveConfig, AllowUIChange=false);
            end
        end

        function waveConfig = updateCachedConfigBWP(obj, action, changedConfigIndex)
            % Get the nrWavegenBWPConfig configuration object from the table
            % and the potential side panel.

            updateBWPCfgSizeFcn = @()wirelessWaveformApp.internal.Utility.updateConfigDimension(obj.bwpWaveConfig, obj.DefaultConfigBWP, action, changedConfigIndex, obj.bwpTable.AllIDs, 'BandwidthPartID');
            if any(action==["ConfigChange", "Add"])
                % If one of the current BWP instances has changed, update
                % the cached configuration from the table.
                waveConfig = obj.bwpWaveConfig;
                if action=="Add"
                    % Update the number of BWP instances
                    waveConfig = updateBWPCfgSizeFcn();
                end
                waveConfig = updateConfiguration(obj.bwpTable, waveConfig);
            else
                % If a BWP instance has been removed, simply update the
                % cached configuration object as there is no need to read
                % the whole table
                waveConfig = updateBWPCfgSizeFcn();
                if action=="Remove"
                    % Remove the BWP grid for the deleted BWP ID, before
                    % updating the cached configuration.
                    updateBWPFigure(obj, action, changedConfigIndex);
                end
            end

            % Update the cache
            obj.bwpWaveConfig = waveConfig;

            if action~="ConfigChange"
                if action~="Remove"
                    % If at least a BWP was added, add a new figure for
                    % each new BWP
                    updateBWPFigure(obj, action, changedConfigIndex);
                end

                % If a BWP instance has been added, removed, or duplicated,
                % update the list of allowed BWP ID for all channels once
                % the cached BWP config object has been updated. Note that
                % the function to mark potential broken BWP links is called
                % in the tableChanged callback anyway.
                updateBWPoptions(obj);
            end
        end

        %% Update visuals related to SCS carriers or BWP
        function updateBWPFigure(obj, action, changedBWPIndex)
            % Update BWP visualization when new BWPs are added or removed

            arguments
                obj
                action (1,1) string {mustBeMember(action,["Add", "Duplicate", "Remove"])}
                changedBWPIndex (1,:) uint8
            end

            appObj = getParent(obj).AppObj;
            if appObj.pShowVisualizations && isprop(obj,'maxNumBWPFigs')
                figAdded = action~="Remove"; % True for Add and Duplicate, false for Remove
                bwpIndices = changedBWPIndex;
                if action=="Duplicate"
                    allIndices = 1:numel(obj.bwpWaveConfig);
                    bwpIndices = allIndices(end-numel(changedBWPIndex)+1:end);
                end
                for n = 1:length(bwpIndices)
                    bwpID = obj.bwpWaveConfig{bwpIndices(n)}.BandwidthPartID; % The ID of the selected BWP
                    if bwpID <= obj.maxNumBWPFigs
                        figName = ['Resource Grid (BWP#' num2str(bwpID) ')'];
                        setVisualState(obj, figName, figAdded);
                    end
                end
                % Position the added figure(s) in the tiles or remove the
                % figures that are no longer needed
                if ~isempty(appObj.pParameters.CurrentDialog)
                    updateActiveVisual = false; % Do not update the figures (i.e., do not call updateGrid)
                    disableStatus = true; % Do not update the status bar within setScopeLayout
                    setScopeLayout(appObj, updateActiveVisual, disableStatus);
                end
            end
        end

        function frChangedForSCSBWP(obj)
            % Once FR changes, update SCS values in SCS and BWP tables
            % Note that this method only works from within the app.

            frValue = obj.FrequencyRange;

            % SCS Carriers
            % Update internal cache of frequency range value in the tables
            updatePropertyValues(obj.scsCarriersTable, PropertyName="FrequencyRange", NewList=frValue);
            updatePropertyValues(obj.bwpTable, PropertyName="FrequencyRange", NewList=frValue);

            % Update the cached configurations
            if frValue == "FR1"
                % When moving to FR1, the default 15kHz carrier is restored
                obj.scsCarriersWaveConfig = obj.DefaultConfigSCS;
            else % FR2
                if obj.isDownlink
                    % When moving to FR2, two carriers, 60kHz and 120kHz,
                    % are set by default for downlink
                    obj.scsCarriersWaveConfig = repmat(obj.DefaultConfigSCS,1,2);
                else
                    % Use the same default 15 kHz carrier. This is updated
                    % in the next steps before the method ends.
                    obj.scsCarriersWaveConfig = obj.DefaultConfigSCS;
                end
            end
            % Update the SCS carriers cached configuration without updating
            % the channel bandwidth view, since at this stage the BWP is
            % not in sync with the SCS carrier yet.
            updateCachedConfigSCSCarriers(obj, "ConfigChange", nan);

            % BWP
            % Ensure the BWP spans the full first SCS carrier when
            % frequency range changes
            obj.bwpWaveConfig = obj.DefaultConfigBWP; % When changing FR, a single BWP is set
            obj.bwpWaveConfig{1}.SubcarrierSpacing = obj.scsCarriersWaveConfig{1}.SubcarrierSpacing;
            obj.bwpWaveConfig{1}.NSizeBWP = obj.scsCarriersWaveConfig{1}.NSizeGrid;
            obj.bwpWaveConfig{1}.NStartBWP = obj.scsCarriersWaveConfig{1}.NStartGrid;
            applyConfiguration(obj.bwpTable, obj.bwpWaveConfig);

            % Make sure all channels are notified of the new BWP
            updateBWPoptions(obj);
            markBrokenBWPLinks(obj);

            % Update SCS and BWP related visualizations (i.e., resource
            % grid and channel bandwidth view)
            appObj = getParent(obj).AppObj;
            if appObj.pShowVisualizations

                if isprop(obj,'maxNumBWPFigs')
                    % Update resource grid plot.
                    % If maxNumBWPFigs is not a defined property of this
                    % object, it means that this app variant does not have
                    % a resource grid plot
                    figName = "Resource Grid (BWP#" + string(1:obj.maxNumBWPFigs) + ")";
                    if any(arrayfun(@(x)getVisualState(obj, char(x)), figName))
                        % If resource grid visualization is disabled, don't
                        % re-enable it for the user here.
                        setVisualState(obj, char(figName(1)), true);
                        arrayfun(@(x)setVisualState(obj, char(x), false), figName(2:end));

                        % Take care of new tiling
                        if ~isempty(appObj.pParameters.CurrentDialog)
                            updateActiveVisual = false; % Do not update the figures (i.e., do not call updateGrid)
                            setScopeLayout(appObj, updateActiveVisual);
                        end
                    end
                end

                % Update channel bandwidth view
                updateChannelBandwidthView(obj);
            end
        end

        %% Apply Configurations
        function applyConfigSCSCarriers(obj, waveCfg)
            % Apply configuration when starting a new session, loading an
            % existing session, or using openInGenerator

            if isempty(obj.scsCarriersTable)
                return; % App initialization
            end

            % Force the input configuration to be a row vector
            ch = waveCfg.SCSCarriers(:)';

            % Update the cache
            obj.scsCarriersWaveConfig = ch;

            % Map the channel configuration to the SCS Carrier table
            applyConfiguration(obj.scsCarriersTable, ch);

            % Update the internal cached SCS info in the BWP table
            updateCachedSCSCarrierInBWP(obj);

        end

        function applyConfigBWP(obj, waveCfg, nvargs)
            % Apply configuration when starting a new session, loading an
            % existing session, or using openInGenerator

            arguments
                % Mandatory inputs
                obj % This object
                waveCfg % The wavegen configuration object

                % Name-Value arguments
                % AllowUIChange is used to update what the user sees on
                % the table programmatically. Set it to false if you don't
                % want to wipe out the visual features of the table, like
                % the selected row and the row IDs.
                % In these cases AllowUIChange is considered true
                % irrespective of the user input:
                % 1. numel(nvargs.IDs) < numel(waveCfg.BandwidthParts)
                % 2. isempty(nvargs.IDs) && (numel(obj.bwpTable.AllIDs)<numel(waveCfg.BandwidthParts))
                nvargs.AllowUIChange (1,1) logical = true;

                % List of row IDs. If AllowUIChange is false, the value of
                % IDs is used to specify the desired IDs of the rows. If
                % IDs is empty, the existing list given by AllIDs is used.
                % In these cases AllowUIChange is considered true
                % irrespective of the user input:
                % 1. numel(nvargs.IDs) < numel(waveCfg.BandwidthParts)
                % 2. isempty(nvargs.IDs) && (numel(obj.bwpTable.AllIDs)<numel(waveCfg.BandwidthParts))
                nvargs.IDs (1,:) uint8 = [];
            end

            if isempty(obj.bwpTable)
                return; % App initialization
            end

            % Force the input configuration to be a row vector
            ch = waveCfg.BandwidthParts(:)';

            % If no BWP is defined, create one with the same dimensions as
            % those of the SCS carrier. This is possible only when using
            % openInGenerator.
            if isempty(ch)
                ch = obj.DefaultConfigBWP;
                ch{1}.SubcarrierSpacing = waveCfg.SCSCarriers{1}.SubcarrierSpacing;
                ch{1}.NStartBWP = waveCfg.SCSCarriers{1}.NStartGrid;
                ch{1}.NSizeBWP = waveCfg.SCSCarriers{1}.NSizeGrid;
            end

            % Update the cache
            obj.bwpWaveConfig = ch;

            % Map the channel configuration to the BWP table
            applyConfiguration(obj.bwpTable, ch, AllowUIChange=nvargs.AllowUIChange, IDs=nvargs.IDs);

            % Update the list of allowed BWP ID for all channels
            updateBWPoptions(obj);
        end

        %% Check Configuration
        function out = hasConfigErrorSCSCarriers(obj)
            % Return the state of the configuration for this channel. The
            % output is true if the app represents an invalid wavegen
            % configuration for the specified channel, and false otherwise.
            out = hasConfigError(obj.scsCarriersTable);
        end

        function e = getConfigErrorSCSCarriers(obj)
            % Return the wavegen configuration error corresponding to this
            % channel, if any. If the app represents a valid wavegen
            % configuration for this channel, the output is empty.
            e = obj.ConfigErrorSCSCarriers;
        end

        function out = hasConfigErrorBWP(obj)
            % Return the state of the configuration for this channel. The
            % output is true if the app represents an invalid wavegen
            % configuration for the specified channel, and false otherwise.
            out = hasConfigError(obj.bwpTable);
        end

        function e = getConfigErrorBWP(obj)
            % Return the wavegen configuration error corresponding to this
            % channel, if any. If the app represents a valid wavegen
            % configuration for this channel, the output is empty.
            e = obj.ConfigErrorBWP;
        end

    end

    methods
        function e = get.ConfigErrorSCSCarriers(obj)
            % Retrieve configuration error from the table
            e = getConfigError(obj.scsCarriersTable);
        end

        function e = get.ConfigErrorBWP(obj)
            % Retrieve configuration error from the table
            e = getConfigError(obj.bwpTable);
        end
    end

    methods (Access = private)
        %% Layout
        function createCarrierGridLayout(obj)
            % Create a grid layout to properly position the two tables in
            % the Main tab
            % For the Main tab, all tables are shown in their entirety and
            % the scrollbar is placed in this grid layout.
            obj.carrierGridLayout = uigridlayout(obj.mainGridLayout,'Tag','carrierGridLayout');
            obj.carrierGridLayout.ColumnWidth = repmat({'fit'},1,2);
            if obj.isDownlink
                obj.carrierGridLayout.RowHeight = {'fit'};
            else
                obj.carrierGridLayout.RowHeight = {'fit','fit'}; % 2 rows for ICGB button
            end
            obj.carrierGridLayout.Scrollable = false;
            % Place SCS/BWP grid layout in position (1,2) of the main grid layout
            obj.carrierGridLayout.Layout.Row = 1;
            obj.carrierGridLayout.Layout.Column = 2;
        end

        %% Cross-table interactions
        function updateCachedSCSCarrierInBWP(obj,nvargs)
            % Update the cached SCS, size, and start grid of the SCSCarrier
            % in the BWP table.

            arguments
                obj
                nvargs.SCSTableInteraction (1,1) logical = false;
            end

            % Get the list of all SCS, start grid, and grid size
            scsNum = [cat(1,[],obj.scsCarriersWaveConfig{:}).SubcarrierSpacing];
            scsSize  = [cat(1,[],obj.scsCarriersWaveConfig{:}).NSizeGrid];
            scsStart = [cat(1,[],obj.scsCarriersWaveConfig{:}).NStartGrid];

            % Remove 240 kHz; it is only for SS Burst, not for BWP
            scs240 = (scsNum==240);
            scsNum(scs240) = [];
            scsSize(scs240) = [];
            scsStart(scs240) = [];

            % Set the available SCS as options in the dropdown of the BWP table,
            % i.e., make it difficult for a BWP to link to a non-present carrier.
            % Provide the options in increasing SCS order:
            scsCell = cellstr(sort(scsNum) + " kHz");
            updatePropertyValues(obj.bwpTable, PropertyName="SubcarrierSpacing", NewList=scsCell);

            % Update the internal cached SCS grid start and size in the BWP
            % table
            updatePropertyValues(obj.bwpTable, PropertyName="SCSCarriers", NewList={scsNum, scsSize, scsStart});

            try
                % Ensure that the BWP config object is updated accordingly
                updateCachedConfig(obj, 'BWP');

                if nvargs.SCSTableInteraction
                    % Only if this call comes from a direct interaction with
                    % the SCSCarriers table, check if the new combination of
                    % SCSCarrier and BWP is correct. Proceed with the rest of
                    % the functionality only if there is no issue
                    if obj.isDownlink
                        cfg = nrDLCarrierConfig;
                    else
                        cfg = nrULCarrierConfig;
                    end
                    cfg.FrequencyRange = obj.FrequencyRange;
                    cfg.ChannelBandwidth = obj.ChannelBandwidth;
                    cfg.SCSCarriers = obj.scsCarriersWaveConfig;
                    cfg.BandwidthParts = obj.bwpWaveConfig;
                    validateConfig(cfg);
                end

                % Update table display and side panel visibility related to
                % interlacing if necessary
                updateInterlacingRelatedVis(obj);

            catch ME
                updateConfigDiagnostic(obj, ME.message, MessageType="error");
            end
        end

        function updateInterlacingRelatedVis(obj)
            % Update table display and side panel related to interlacing,
            % in case SCS and/or BWP changes make it necessary to do so

            if obj.isDownlink
                return; % no-op for downlink
            end

            waveParamClass = getParent(obj);
            classNamePUSCH = getClassNamePXSCHAdv(obj);
            classNamePUCCH = wirelessWaveformApp.nr5G_PUCCH_Tab.classNamePUCCHAdv;
            classNames = {classNamePUSCH, classNamePUCCH};
            for c = 1:length(classNames)
                className = classNames{c};
                if isKey(waveParamClass.DialogsMap, className)
                    dlg = waveParamClass.DialogsMap(className);
                    updateTableDisplayInterlacing(dlg);
                    needRepaint = updateInterlacingVis(dlg);
                    if needRepaint
                        layoutUIControls(dlg);
                    end
                end
            end
        end
    end

    methods(Access = {?wirelessWaveformGenerator.nr5G_SSB_DataSource})
        function amendCachedConfigBWP(obj, chWaveCfg, cfgIndex)
            % Update the cached BWP wave config object with the input
            % CHWAVECFG object.

            arguments
                obj
                chWaveCfg (:,1) cell
                cfgIndex (1,1) uint8
            end

            % Ensure the BWP cache is updated with the new wavegen
            % configuration object
            if numel(obj.bwpWaveConfig)~=numel(obj.bwpTable.AllIDs)
                % A new configuration object needs to be appended
                obj.bwpWaveConfig = cat(2, obj.bwpWaveConfig, chWaveCfg);
            else
                % Update the relevant BWP config objects
                obj.bwpWaveConfig(cfgIndex) = chWaveCfg;
            end
        end
    end
end
