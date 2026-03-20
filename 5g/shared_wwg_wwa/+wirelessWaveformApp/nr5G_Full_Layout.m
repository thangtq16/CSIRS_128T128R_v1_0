classdef nr5G_Full_Layout < handle
% Common functionality for both Downlink and Uplink Full 5G
% parameterization

%   Copyright 2020-2024 The MathWorks, Inc.

    properties (Hidden)
        % Handle tab transitions:
        ClientActionListener;   % Listener tracking clicks on specific tabs
        pxschExtraTile = false  % internal state indicating if right-side panel under PXSCH has/should be launched
        pxcchExtraTile = false  % internal state indicating if right-side panel under PUCCH has/should be launched
        xrsExtraTile = false    % internal state indicating if right-side panel under SRS has/should be launched
        lastFigInFocus = 'Main' % last figure to be in focus (before any possible future clicks)
        mainGridLayout          % Grid layout container used in the Main panel
    end

    properties (Abstract, Hidden)
        tableObjName % List the name of all table objects contained in the waveform type
    end

    properties (Abstract)
        % This property is used in table creation and to get the
        % configuration of ICGB
        isDownlink
        Figure
        % Cache the current nrXLCarrierConfig to avoid getting the
        % configuration every time updateGrid and updateREVisual are called
        cachedCfg
    end

    properties (Constant, Abstract)
        PXSCHfigureName
        PXCCHfigureName
        XRSfigureName
        pxschExtraFigTag
        pxcchExtraFigTag
        xrsExtraFigTag
    end

    methods (Abstract)
        % DL/UL subclasses must implement these methods:
        createFigures(obj)
        setExtraConfigFigVisibility(obj, visibility)
        names = figureNames(obj)
        tag = figTag(obj, name)
        getParent
        getPanel
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_Full_Layout(~, isDownlink)

            obj.isDownlink = isDownlink;

            %% Figures:
            createFigures(obj);
            mainFig = obj.getParent.AppObj.pParametersFig;
            mainFig.AutoResizeChildren = 'off';
            obj.setExtraConfigFigVisibility(true);
            obj.mainGridLayout = mainFig.Children;
            obj.mainGridLayout.Scrollable = true;
            obj.mainGridLayout.Tag = strrep(figTag(obj,"Main"),"Fig","GridLayout"); % Tag depends on the waveform type: mainDLGridLayout and mainULGridLayout

            % Create listener for understanding which tab we click:
            weakObj = matlab.lang.WeakReference(obj);
            obj.ClientActionListener = addlistener(obj.getParent.AppObj.AppContainer, 'PropertyChanged', @(event, data) propertyChanged(weakObj.Handle, event, data));

            %% Tables:
            restoreDefaults(obj); % set ChannelBandwidth before creating tables, which need Carriers.NRB

            % establish Top-level parameters & filtering to Main tab:
            obj.getParent.Layout.Parent = obj.Figure;
        end

        function adjustSpec(obj)

            % Main document has been created in extensionTypeChange, before
            % construction, to prevent unnecessary creation of left-side figure
            % panel. Here we just need to update properties.
            mainTag = figTag(obj,"Main");
            doc = obj.getParent.AppObj.AppContainer.getDocument(getTag(obj.getParent.AppObj) + "DocumentGroup", mainTag);
            doc.Title = 'Main';
            obj.(mainTag) = doc.Figure;
            obj.(mainTag).AutoResizeChildren = 'off';
            obj.(mainTag).SizeChangedFcn = @(a, b) resizeBanner(obj);
            obj.(mainTag).Scrollable = 'on'; % allows horizontal/vertical scrolling when not all items fit in figure

            updateParametersFig(obj);
        end

        function updateParametersFig(obj)
            obj.Figure = obj.(figTag(obj,"Main")); % this is where create UI Controls will place items
        end

        function createFig(obj, figTitle, figName, varargin)
            if isempty(varargin)
                figTag = figName;
            else
                figTag = varargin{1};
            end

            % Reusable figure constructor for all tabs
            if isempty(obj.(figName))
                appObj = obj.getParent.AppObj;
                doc = appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", figTag);
                if isempty(doc)
                    document = matlab.ui.internal.FigureDocument(...
                        'Title',  figTitle, ...
                        'Tag', figTag, ...
                        'DocumentGroupTag', getTag(appObj) + "DocumentGroup", ...
                        'Closable', false);
                    addDocument(obj.getParent.AppObj.AppContainer, document);
                    obj.(figName) = document.Figure;
                    obj.(figName).Tag = figTag;
                    obj.(figName).AutoResizeChildren = 'off';
                    obj.(figName).SizeChangedFcn = @(a, b) resizeBanner(obj);
                    obj.(figName).Scrollable = 'on'; % allows horizontal/vertical scrolling when not all items fit in figure
                else
                    doc.Visible = true;
                    obj.(figName) = doc.Figure;
                end
            end
        end

        function setupDialog(obj)
            % Actions performed when entering this Full DL/UL wavegen extension:

            % turn off the listener while figures are enabled, so that client
            % actions (similar to clicking) are not triggered
            obj.ClientActionListener.Enabled = false;

            % Clear any configuration-related message
            updateConfigDiagnostic(obj, "");

            obj.lastFigInFocus = 'Main'; % set this now, as PXSCH/PUCCH Advanced visibility is determined by last fig in focus
            obj.setExtraConfigFigVisibility(true);

            appObj = obj.getParent.AppObj;
            % for AppContainer, arrange the layout. No need for a drawnow this way:
            obj.pxschExtraTile = false;
            obj.pxcchExtraTile = false;
            obj.xrsExtraTile = false;
            if isfield(appObj.AppContainer.DocumentLayout, 'tileOccupancy')
                % Create a fresh-new instance of tileOccupancy to make sure that
                % invisible documents do not affect the integrity of the structure
                [~, ~, str] = obj.getTileLayout([]);
                % Set Main as the currently-visible document
                configTile = 1;
                str(configTile).showingChildId = getTag(appObj) + "DocumentGroup_" + figTag(obj,"Main");
                str(configTile).showingChildTitle = 'Main';
                appObj.AppContainer.DocumentLayout.tileOccupancy = str;
                % else initializing
            end

            obj.ClientActionListener.Enabled = true; % now we can reenable the listener

            % Make SCS & BWP tables re-appear
            if ~isempty(obj.scsCarriersTable)
                obj.carrierGridLayout.Visible = true;
            end

            % Set padding on the top to create more real estate so that the
            % banner doesn't cover anything
            setPaddingForBanner(obj);

            % Ensure that all tabs are properly rendered before moving
            % forward with the rest of the layout. This avoids a potential
            % race condition between the layout drawn here and the layout
            % drawn in setScopeLayout() due to the asynchronicity of these
            % actions.
            drawnow;
        end

        function resetCustomVisuals(obj)
            % Executed during New session, to initialize desired state of visuals

            markAllBrokenLinks(obj);

            % Make sure the Advanced configuration tabs for PXSCH, PXCCH, and XRS
            % are not visible
            updateSidePanel(obj,obj.PXSCHfigureName,false);
            updateSidePanel(obj,obj.PXCCHfigureName,false);
            updateSidePanel(obj,obj.XRSfigureName,false);
        end

        function updateDisabled(obj)
            % Reset the Enable property of the 5G DL/UL table buttons after the
            % app disabled them (e.g., after waveform generation or export to
            % Simulink)

            arrayfun(@(x)updateDisabled(obj.(x)),obj.tableObjName);
        end

        function outro(obj, newDialog)
            % Executed when moving to a new waveform type (e.g., 5G -> QAM), i.e., Cleanup

            % Revert extra padding added for the banner to the original
            % value
            revertPaddingForBanner(obj);

            updateParametersFig(newDialog);
            obj.getParent.CurrentDialog = newDialog; %#ok<*MCNPR>

            % Turn off figures specific to this feature when switching to a different waveform type
            obj.setExtraConfigFigVisibility(false);

            % Remove the extra column added for SCS and BWP tables
            obj.mainGridLayout.ColumnWidth = {'1x' 0};
            obj.mainGridLayout.RowHeight = {'1x'};
            obj.carrierGridLayout.Visible = false;

            % Clear any configuration-related message
            updateConfigDiagnostic(obj, "");

        end

        function cfg = applyConfiguration(obj, cfg, keyTag)
            % Used on New, Open Session, and openInGenerator.

            % Check if the waveform configuration is supported, retrieve warning
            % message, and adjust configuration by disabling unsupported features
            [supported, msg, cfg] = wirelessWaveformApp.internal.isSupportedInWaveformApp(cfg,keyTag);

            % Show a warning window if the current configuration is not supported
            if ~supported
                throwErrorPopup(obj, msg, Icon="warning");
            end

            % Map a saved nrXLCarrierConfig object to the GUI elements of the
            % Main tab (only - for right here).
            obj.Label             = cfg.Label;
            obj.FrequencyRange    = cfg.FrequencyRange;
            obj.ChannelBandwidth  = cfg.ChannelBandwidth;
            obj.NCellID           = cfg.NCellID;
            obj.NumSubframes      = cfg.NumSubframes;
            obj.InitialNSubframe  = cfg.InitialNSubframe;

            % There are 3 differences between programmatic API and GUI:
            % 1. Windowing
            if isempty(cfg.WindowingPercent)
                obj.WindowingSource   = 'Auto';
                obj.WindowingPercent  = [];
            else
                obj.WindowingSource   = 'Custom';
                obj.WindowingPercent  = cfg.WindowingPercent;
            end
            setVisible(obj, 'WindowingPercent', ~isequal(cfg.WindowingPercent, []));
            % 2. Sample Rate
            if isempty(cfg.SampleRate)
                obj.SampleRateSource   = 'Auto';
                obj.SampleRate         = [];
            else
                obj.SampleRateSource   = 'Custom';
                obj.SampleRate         = cfg.SampleRate;
            end
            setVisible(obj, 'SampleRate', ~isequal(cfg.SampleRate, []));

            % 3. Carrier Frequency
            if cfg.CarrierFrequency == 0
                obj.PhaseCompensation  = false;
                obj.CarrierFrequency   = 3.5e9; % 3.5 GHz
            else
                obj.PhaseCompensation  = true;
                obj.CarrierFrequency   = cfg.CarrierFrequency;
            end
            setVisible(obj, 'CarrierFrequency', cfg.CarrierFrequency ~= 0);

            layoutUIControls(obj);

            % Enable New Session button since a config has been loaded.
            % This is appropriate after Open Session and openInGenerator.
            % For a New Session action, the app will adjust the New Session
            % button as required.
            obj.getParent.AppObj.pNewSessionBtn.Enabled = true;
            
        end

        %% Tile Handling
        function cols = getNumTileColumns(~, ~)
            % Tile architecture always uses 2 columns (under all tabs)
            cols = 2 + 1; % AppContainer layout removes 1 column, which is for the left-side panel
        end
        function rows = getNumTileRows(~, ~)
            % Tile architecture always uses 2 rows (under all tabs)
            rows = 2;
        end

        function w = getRowWeights(~, ~)
            % Rows are split evenly
            w = [0.5 0.5];
        end

        function tiles = getNumTiles(obj, ~)
            configTab = 1;
            advancedRightPane = obj.pxschExtraTile || obj.pxcchExtraTile || obj.xrsExtraTile;
            tiles = configTab + double(advancedRightPane);
        end

        function n = numVisibleFigs(obj)
            n = obj.pxschExtraTile;
        end

        function [tileCount, tileCoverage, tileOccupancy, tileID] = getTileLayout(obj, ~, varargin)

            tileCount = getNumTiles(obj);

            advancedRightPane = obj.pxschExtraTile || obj.pxcchExtraTile || obj.xrsExtraTile;
            if advancedRightPane
                tileCoverage = [1 2; 3 2];
                tileCount = 3;
            else
                if tileCount == 3
                    tileCoverage = [1 1; 2 3];
                else
                    tileCoverage = [1; 2];
                end
            end

            tileOccupancy = repmat(struct('children', []), tileCount, 1);

            parentTag = getTag(obj.getParent.AppObj);
            tileID = 1;
            channelNames = obj.figureNames;
            for idx = 1:numel(channelNames)
                tag = obj.figTag(channelNames{idx});
                childOrder = idx;
                tileOccupancy = addTile(tileOccupancy, tileID, childOrder, parentTag, tag);
                if idx==1 % Assign only once
                    tileOccupancy(tileID).showingChildId = parentTag + "DocumentGroup_" + figTag(obj, obj.lastFigInFocus);
                end
            end

            tileID = tileID + 1;

            if obj.pxschExtraTile
                childIDTag = obj.figTag(obj.PXSCHfigureName);
                [tileOccupancy, tileID] = addTile(tileOccupancy, tileID, 1, parentTag, obj.pxschExtraFigTag, childIDTag);
            end

            if obj.pxcchExtraTile
                childIDTag = obj.figTag(obj.PXCCHfigureName);
                [tileOccupancy, tileID] = addTile(tileOccupancy, tileID, 1, parentTag, obj.pxcchExtraFigTag, childIDTag);
            end

            if obj.xrsExtraTile
                childIDTag = obj.figTag(obj.XRSfigureName);
                [tileOccupancy, tileID] = addTile(tileOccupancy, tileID, 1, parentTag, obj.xrsExtraFigTag, childIDTag);
            end

        end

        %% Visualization
        function layoutPanels(obj)
            % Update the main grid to be fit on all rows and columns.
            % The carrier grid is placed in the second column of the main grid,
            % as follows:
            %
            % Configuration|| SCS Table | BWP Table ||
            %              ||           |           ||
            % Filtering    ||           |           ||

            obj.mainGridLayout.ColumnWidth = {'fit', 'fit'};
            obj.mainGridLayout.RowHeight = {'fit'};
            obj.carrierGridLayout.Visible = true;
        end

        function extraPanels = getExtraPanels(obj)
            extraPanels = mat2cell(obj.carrierGridLayout.Children,ones(length(obj.carrierGridLayout.Children),1));
            if isKey(obj.getParent.DialogsMap, obj.getClassNamePXSCHAdv) && isgraphics(obj.getParent.DialogsMap(obj.getClassNamePXSCHAdv).getPanels)
                dlgAdv{1,1} = obj.getParent.DialogsMap(obj.getClassNamePXSCHAdv);
                dlgAdv{2,1} = obj.getParent.DialogsMap(obj.getClassNamePXSCHDMRS);
                dlgAdv{3,1} = obj.getParent.DialogsMap(obj.getClassNamePXSCHPTRS);
                extraPanels = cat(1,extraPanels,dlgAdv);
            end
        end

        function cellDialogs = getDialogsPerColumn(obj)
            cellDialogs{1} = {obj};
        end

    end

    % Methods specific to this class and that are currently used by
    % neighboring classes
    methods (Access = public)
        function frChanged(obj, ~)
            % Executed when frequency range changes (FR1<->FR2)

            % Actions specific to link type (DL/UL)
            frChangedForLink(obj);

            % Actions that specific to Full 5G wavegen:
            frChangedForSCSBWP(obj);

            % Make first PXSCH full-band:
            if ~isempty(obj.scsCarriersTable) % not init
                gridSize = obj.scsCarriersWaveConfig{1}.NSizeGrid;
                obj.pxschWaveConfig{1}.PRBSet = 0:gridSize-1;
                applyConfiguration(obj.pxschTable, obj.pxschWaveConfig);
            end

            updateGrid(obj);
            updateChannelBandwidthView(obj);
        end
        function frChangedGUI(obj, ~)
            % For the custom 5G Downlink/Uplink, this performs the same actions
            % as frChanged.
            frChanged(obj);
        end

        %% Tables
        function tableGridLayout = createTableGridLayout(~,parent,tag,siz)
            % By default, the outer grid is not scrollable, as the scrollbar
            % should be in the uitables, to be as close as possible to the unit
            % the user is currently working on. Each tab can implement their
            % own exception to this default behaviour.
            tableGridLayout = uigridlayout(parent,'Tag',[tag,'GridLayout']);
            tableGridLayout.Scrollable = false;
            tableGridLayout.ColumnWidth = repmat({'1x'},1,siz);
            tableGridLayout.RowHeight = repmat({'1x'},1,siz);
            tableGridLayout.Padding = 8*ones(1,4); % Reduce the outside padding to better use the available real estate
        end


        %% Visualization
        function brokenLinks = getBrokenLinks(obj, chToLink, chThatLinks, propName, brokenLinks)

            chToLinkWaveCfgObj = getWaveConfig(obj, chToLink, CheckConfigError=false);
            if isempty(chToLinkWaveCfgObj)
                % Initialization
                return;
            end
            possibleValues = cellfun(@(x) cat(1,[],x.(propName)), chToLinkWaveCfgObj);

            if ~iscell(chThatLinks)
                chThatLinks = {chThatLinks};
            end

            for c = 1:length(chThatLinks)
                chThatLinksName = chThatLinks{c};
                chThatLinks5GTableObj = get5GTableObject(obj, chThatLinksName);
                if ~isempty(chThatLinks5GTableObj)
                    chThatLinksWaveCfgObj = getWaveConfig(obj, chThatLinksName, CheckConfigError=false);
                    currValues = cellfun(@(x) cat(1,[],x.(propName)), chThatLinksWaveCfgObj);
                    d = setdiff(currValues, possibleValues);
                    if isempty(d)
                        idx = [];
                    else
                        idx = find(ismember(currValues, d));
                    end

                    % Update the broken link indices for this table
                    brokenLinks.(chThatLinksName) = cat(1,brokenLinks.(chThatLinksName),idx);
                end
            end
        end

        function markBrokenLinks(obj, brokenLinks)
            % Given the structure containing the broken links, paint each
            % row with a broken link in red

            tables = fieldnames(brokenLinks);
            for t = 1:length(tables)
                tableObj = get5GTableObject(obj, tables{t});
                if ~isempty(tableObj)
                    % Reset background color and highlight rows with broken links
                    highlightRows(tableObj,Reason="BrokenLink",Rows=brokenLinks.(tables{t}));
                end
            end
        end

        function updateBWPoptions(obj)
            % Update the offered dropdown values for the BWP ID column in
            % the input tables
            bwpIDList = cellfun(@(x)(cat(1,[],x.BandwidthPartID)),obj.bwpWaveConfig);
            bwpIDList = cellstr(string(sort(bwpIDList)));
            tables = obj.tableObjName(contains(obj.tableObjName,["PXSCH",obj.PXCCHfigureName,obj.XRSfigureName],IgnoreCase=true));
            arrayfun(@(x)updatePropertyValues(obj.(x), PropertyName="BandwidthPartID", NewList=bwpIDList), tables);
        end

        function out = sidePanelExists(obj, channelName)
            className = getAdvancedDialogClassName(obj,channelName);
            out = isKey(obj.getParent.DialogsMap, className) && isgraphics(obj.getParent.DialogsMap(className).getPanels);
        end

        %% Configuration
        function updateCachedConfig(obj, channelName, varargin)
            % Update the cached configuration property for this channel

            if (nargin==2 || varargin{1}.Action=="ConfigChange")
                action = "ConfigChange";
                changedConfigIndex = [];
            else
                action = varargin{1}.Action;
                changedConfigIndex = varargin{:}.ChangedSelection;
            end

            switch channelName
                case 'SCSCarriers'
                    waveConfigSCSCarriers = updateCachedConfigSCSCarriers(obj, action, changedConfigIndex);
                    obj.cachedCfg.SCSCarriers = waveConfigSCSCarriers; % Update the XL cached configuration object
                case 'BWP'
                    waveConfigBWP = updateCachedConfigBWP(obj, action, changedConfigIndex);
                    obj.cachedCfg.BandwidthParts = waveConfigBWP; % Update the XL cached configuration object
                case 'CORESET'
                    waveConfigCORESET = updateCachedConfigCORESET(obj, action, changedConfigIndex);
                    obj.cachedCfg.CORESET = waveConfigCORESET; % Update the DL cached configuration object
                case 'SearchSpaces'
                    waveConfigSearchSpaces = updateCachedConfigSearchSpaces(obj, action, changedConfigIndex);
                    obj.cachedCfg.SearchSpaces = waveConfigSearchSpaces; % Update the DL cached configuration object
                case 'PDCCH'
                    waveConfigPDCCH = updateCachedConfigPDCCH(obj, action, changedConfigIndex);
                    obj.cachedCfg.PDCCH = waveConfigPDCCH; % Update the DL cached configuration object
                case 'PDSCH'
                    waveConfigPDSCH = updateCachedConfigPXSCH(obj, action, changedConfigIndex);
                    obj.cachedCfg.PDSCH = waveConfigPDSCH; % Update the DL cached configuration object
                case 'CSIRS'
                    waveConfigCSIRS = updateCachedConfigCSIRS(obj, action, changedConfigIndex);
                    obj.cachedCfg.CSIRS = waveConfigCSIRS; % Update the DL cached configuration object
                case 'PUSCH'
                    waveConfigPUSCH = updateCachedConfigPXSCH(obj, action, changedConfigIndex);
                    obj.cachedCfg.PUSCH = waveConfigPUSCH; % Update the UL cached configuration object
                case 'PUCCH'
                    waveConfigPUCCH = updateCachedConfigPUCCH(obj, action, changedConfigIndex);
                    obj.cachedCfg.PUCCH = waveConfigPUCCH; % Update the UL cached configuration object
                case 'SRS'
                    waveConfigSRS = updateCachedConfigSRS(obj, action, changedConfigIndex);
                    obj.cachedCfg.SRS = waveConfigSRS; % Update the UL cached configuration object
            end

        end

        % Get configuration object from the table and potential side panel
        function cfg = getWaveConfig(obj, channelName, nvargs)

            arguments
                % Mandatory inputs
                obj
                channelName (1,1) string {mustBeMember(channelName, ...
                    ["scscarriers", "bwp", "pdsch", "pusch", "pxsch", "pdcch", "coreset", "searchspaces", "pucch", "csirs", "srs"])}
                % Optional Name-Value arguments
                % If set to false, retrieve the cached configuration for
                % this channel without caring as to whether there is an
                % error for this channel or not
                nvargs.CheckConfigError (1,1) logical = true
            end

            % Retrieve the cached wavegen config object
            switch lower(channelName)
                case 'scscarriers'
                    cfg = obj.scsCarriersWaveConfig;
                case 'bwp'
                    cfg = obj.bwpWaveConfig;
                case {'pdsch', 'pusch', 'pxsch'}
                    cfg = obj.pxschWaveConfig;
                case 'pdcch'
                    cfg = obj.pdcchWaveConfig;
                case 'coreset'
                    cfg = obj.coresetWaveConfig;
                case 'searchspaces'
                    cfg = obj.searchSpacesWaveConfig;
                case 'pucch'
                    cfg = obj.pucchWaveConfig;
                case 'csirs'
                    cfg = obj.csirsWaveConfig;
                case 'srs'
                    cfg = obj.srsWaveConfig;
            end

            if ~(isprop(obj, 'Initializing') && obj.Initializing) && ...
                    nvargs.CheckConfigError && hasConfigError(obj, channelName)
                % The requested channel represents an invalid wavegen
                % configuration.
                % Display the error in a popup dialog
                e = getConfigError(obj, channelName);
                rethrow(e);
            end
        end

        % Get the 5G-Table object from the channel name
        function waveTableObj = get5GTableObject(obj, channelName)
            switch lower(channelName)
                case 'scscarriers'
                    waveTableObj = obj.scsCarriersTable;
                case 'bwp'
                    waveTableObj = obj.bwpTable;
                case {'pdsch', 'pusch', 'pxsch'}
                    waveTableObj = obj.pxschTable;
                case 'pdcch'
                    waveTableObj = obj.pdcchTable;
                case 'coreset'
                    waveTableObj = obj.coresetTable;
                case 'searchspaces'
                    waveTableObj = obj.searchSpacesTable;
                case 'pucch'
                    waveTableObj = obj.pucchTable;
                case {'csirs','csi-rs'}
                    waveTableObj = obj.csirsTable;
                case 'srs'
                    waveTableObj = obj.srsTable;
            end
        end

        %% Side panel
        function createSidePanel(obj, channelName)
            if hasSidePanel(channelName) && ~sidePanelExists(obj, channelName)
                switch lower(channelName)
                    case {'pdsch', 'pusch', 'pxsch'}
                        createAdvancedTabPXSCH(obj);
                    case 'pucch'
                        createAdvancedTabPUCCH(obj);
                    case 'srs'
                        createAdvancedTabSRS(obj);
                end
            end
        end
    end

    % Protect methods for derived classes
    methods (Access = protected)
        %% Visualization
        function newPanel = updateSidePanel(obj, figName, setVisible)
            % Update the visibility of the input side panel containing the
            % advanced configuration tab

            newPanel = false;

            % Return if this channel/signal has no side panel
            if ~hasSidePanel(figName)
                return;
            end

            % Get extra tile property name, extra figure tag, and create
            % advanced tab method for this channel or signal
            [extraTilePropName,extraFigTag] = getSidePanelPropNamesAndMethods(obj,figName);

            % Create the right-side panels if this is the 1st time we reach
            % here.
            if setVisible && ~sidePanelExists(obj,figName)
                createSidePanel(obj,figName);
                newPanel = true;
            end

            % Change visibility of side panel according to input
            doc = obj.getParent.AppObj.AppContainer.getDocument(getTag(obj.getParent.AppObj) + "DocumentGroup", extraFigTag);
            if ~isempty(doc)
                doc.Visible = setVisible;
                obj.(extraTilePropName) = setVisible;
            end
        end

        % Get extra tile property name, extra figure tag, and create advanced
        % tab method for this channel or signal
        function [extraTilePropName,extraFigTag] = getSidePanelPropNamesAndMethods(obj,figName)

            switch figName
                case obj.PXSCHfigureName
                    extraTilePropName = 'pxschExtraTile';
                    extraFigTag = obj.pxschExtraFigTag;
                case obj.PXCCHfigureName
                    extraTilePropName = 'pxcchExtraTile';
                    extraFigTag = obj.pxcchExtraFigTag;
                case obj.XRSfigureName
                    extraTilePropName = 'xrsExtraTile';
                    extraFigTag = obj.xrsExtraFigTag;
            end
        end

        function markConflictInConfigTables(dialog,conflicts)
            % Change color of tables involved in a conflict among
            % channels/signals

            % Create a dictionary that contains all the channels with the
            % indices of their conflicts
            % This is used for the channels defined through the nr5G_Table class
            d = dictionary(dialog.tableObjName,cell(1,numel(dialog.tableObjName)));

            for c = 1:length(conflicts)
                cfl = conflicts(c);

                chName = replace(lower([cfl.ChannelType{:}]),["pdsch","pusch"],"pxsch");
                if ~any(chName=="") % Do not do anything if there is no conflict
                    for chIdx = 1:length(chName)
                        % For each channel in conflict, update the
                        % dictionary that stores the conflict for the
                        % nr5G_Table objects
                        idx = contains(dialog.tableObjName, chName{chIdx});
                        if any(idx)
                            channelIdx = d(dialog.tableObjName(idx)); % Existing indices
                            d(dialog.tableObjName(idx)) = {cat(2,channelIdx{:},cfl.ChannelIdx(chIdx))};
                        end
                    end
                end
            end

            % Update the background color of each table, except for SCS and
            % BWP, which don't have conflicts by definition
            tables = dialog.tableObjName;
            tables(matches(tables,["scsCarriersTable", "bwpTable"])) = [];
            for t = 1:length(tables)
                chName = tables(t);
                channelIdx = d(chName);
                nrTableObj = dialog.(chName);
                if ~isempty(nrTableObj) % nr5G_Table object is already initialized
                    highlightRows(nrTableObj,Reason="Conflict",Rows=channelIdx{:});
                end
            end

        end

        % Make input side panel visible and others not visible. This order
        % must be respected to avoid re-layouts.
        function newPanel = bringSidePanelToFocus(obj,newTabName)

            figNameList = {obj.PXSCHfigureName, obj.PXCCHfigureName, obj.XRSfigureName};
            figNameList = [intersect(figNameList,newTabName) setxor(figNameList,newTabName)];

            newPanel = false;
            for i = 1:numel(figNameList)
                visible = (i == 1);
                np = updateSidePanel(obj,figNameList{i},visible);
                newPanel = newPanel | np;
            end

            % Update the the last figure in focus with the new figure
            obj.lastFigInFocus = newTabName;
        end

        function markAllBrokenLinks(~)
            % can be overriden by children classes
        end

        function bwChanged(obj, ~)
            % Check the current configuration object before updating the
            % channel bandwidth view
            updateAppConfigDiagnostic(obj, []); % This performs config checks under the hood already

            % Update the channel bandwidth view plot
            updateChannelBandwidthView(obj);
        end

        function className = getAdvancedDialogClassName(obj, channelName)
            switch lower(channelName)
                case {'pdsch', 'pusch'}
                    className = getClassNamePXSCHAdv(obj);
                case 'pucch'
                    className = wirelessWaveformApp.nr5G_PUCCH_Tab.classNamePUCCHAdv;
                case 'srs'
                    className = wirelessWaveformApp.nr5G_SRS_Tab.classNameSRSAdv;
                otherwise
                    className = '';
            end
        end

        function frChangedForLink(~)
            % no-op by default
        end

        %% Listener handling
        function propertyChanged(obj, ~, ~)

            if ...
                    ~isa(obj.getParent.CurrentDialog,class(obj)) || ...
                    ~isvalid(obj.getParent.AppObj) || ... % closing
                    isvalid(obj.getParent.AppObj.FreezeHandle) || ... % still launching
                    ~isfield(obj.getParent.AppObj.AppContainer.SelectedChild, 'tag') % minimized excessively

                return;
            end
            %
            % Do actions related to tab changes when required, e.g, layout.
            newTab = obj.getParent.AppObj.AppContainer.SelectedChild.title;

            % Intersect the title of the tab with all figure names so that
            % only clicks in tabs are processed but subsequent re-layouts
            % in onTabChange (e.g., side panel creation) triggering
            % additional property changes are filtered out.
            tabName = intersect(newTab,figureNames(obj));
            if ~isempty(tabName)
                onTabChange(obj,tabName{1});
            end

        end

        function onClientAction(obj, ~, ev)
            % Executes when, e.g., a click happens. Need to filter the
            % clicks that correspond to a tab change, so that we possibly
            % change the PDSCH layout. Additional conditions on went to
            % execute are handled in child classes.

            if strcmpi(ev.EventData.EventType, 'ACTIVATED')
                newTabName = ev.EventData.Client.Name;
                onTabChange(obj, newTabName);
            end

        end

        function onTabChange(obj, newTabName)

            if strcmp(obj.lastFigInFocus, newTabName)
                % If the tab has not changed, no need to do anything
                return;
            end

            % Do the re-layout only when it is needed
            isChannelOrSignal = ~any(startsWith(newTabName, {'Spectrum', 'Resource', 'Channel'}));
            if isChannelOrSignal

                appObj = obj.getParent.AppObj;
                lastFig = obj.lastFigInFocus;
                newRightPanel = bringSidePanelToFocus(obj,newTabName);
                [~, ~, tileOccupancy] = getTileLayout(obj,[]);
                layoutChanged = ~isequal(appObj.AppContainer.DocumentLayout.tileOccupancy,tileOccupancy);

                if newRightPanel || xor(hasSidePanel(lastFig), hasSidePanel(newTabName)) || (hasSidePanel(newTabName) && layoutChanged)
                    % Transitions from Main <-> SSBurst <-> PDCCH <-> CSIRS do not cause layout changes
                    % Similarly, PUSCH <-> PUCCH do not cause layout changes
                    % Now we either entered PXSCH or PUCCH tab, or we left it for one of the
                    % other configuration tabs.

                    freezeApp(obj.getParent.AppObj);

                    % Re-layout the tiles, but only if we are not in a startup/shut down state
                    if isvalid(appObj)
                        % do not attempt re-layout during startup/closing
                        % the spectrum analyzer check is also used to identify closing states
                        if any(strcmpi(newTabName, figureNames(obj)))
                            % do not do these when someone clicks on a scope
                            updateActiveVisual = false; % Do not update the figures (i.e., do not call updateGrid)
                            disableStatus = true; % Do not update the status bar message that mentions the scopes initialization
                            setScopeLayout(appObj,updateActiveVisual,disableStatus);
                            if isgraphics(obj.getPanel) % avoid calling layoutPanels during shutdown
                                layoutPanels(obj);
                                if sidePanelExists(obj,newTabName)
                                    className = getAdvancedDialogClassName(obj,newTabName);
                                    dlg = obj.getParent.DialogsMap(className);
                                    layoutPanels(dlg); % prevent messed up layout during render
                                end
                            end
                            % Resize the banners, if any
                            resizeBanner(obj);
                        end
                    end
                    unfreezeApp(obj.getParent.AppObj);
                end
            end

        end

        function tableChanged(obj, src, event)
            % A channel table object has changed. Update everything that needs
            % updating.

            if event.EventName=="PostSet"
                affectedObj = event.AffectedObject;
            else
                affectedObj = src; % Object notifying of the property change
            end
            if (isprop(obj, 'Initializing') && obj.Initializing) || isempty(affectedObj)
                % The table object has not been properly instantiated yet
                return;
            end

            % Get the name of the channel that triggered the update
            c = extractBetween(class(affectedObj),'_','_');
            channelName = c{1};

            if event.EventName=="TableChanged"
                % The table data has changed

                % If a BWP was added, freeze the app
                if lower(channelName)=="bwp" && any(event.Action==["Add", "Duplicate"])
                    freezeApp(obj.getParent.AppObj);
                end

                setDirty(obj);

                isErrorFree = isempty(event.Error);

                if isErrorFree
                    % Perform the following actions only if the callback has
                    % not been activated by an error in the user input

                    % Clear the banner since there is no error up to
                    % this point. The subsequent code will populate any new
                    % errors.
                    updateConfigDiagnostic(obj, "");

                    % Update the cache configuration object
                    updateCachedConfig(obj, channelName, event);

                    % Update linking status for all tables. Not the minimum set of
                    % operations, but delay is not noticeable
                    markAllBrokenLinks(obj);

                    % If SCS has been modified, update the channel bandwidth view
                    if lower(channelName)=="scscarriers"
                        updateChannelBandwidthView(obj);
                    end

                    % Update grid and retrieve config conflicts
                    updateGrid(obj);
                end
            end

            if (event.EventName=="TableChanged" && isErrorFree) || ...
                    (event.EventName=="PostSet" && src.Name=="Selection")
                % The configuration or the selected row have changed

                % Update side panel with wavegen config object data
                mapCache2SidePanel(obj, channelName);

                % Update the visibility of all dependent parameters
                updateControlsVisibility(obj, channelName);

                % Update RE mapping plot
                updateREVisual(obj, channelName, '');
            end

            % If a BWP was added, unfreeze the app
            if event.EventName=="TableChanged" && lower(channelName)=="bwp" && any(event.Action==["Add", "Duplicate"])
                unfreezeApp(obj.getParent.AppObj);
            end

            if event.EventName=="TableChanged" && ~isErrorFree
                % An error was triggered by the user interaction
                updateConfigDiagnostic(obj, event.Error.message, MessageType="error");
            end
        end

    end

    methods (Access = private)
        %% Configuration
        function e = getConfigError(obj, channelName)
            % Return the wavegen configuration error corresponding to this
            % CHANNELNAME, if any. If the app represents a valid wavegen
            % configuration for this channel, the output is empty.

            % Retrieve the information from the right tab
            switch lower(channelName)
                case 'scscarriers'
                    e = getConfigErrorSCSCarriers(obj);
                case 'bwp'
                    e = getConfigErrorBWP(obj);
                case {'pdsch', 'pusch', 'pxsch'}
                    e = getConfigErrorPXSCH(obj);
                case 'pdcch'
                    e = getConfigErrorPDCCH(obj);
                case 'coreset'
                    e = getConfigErrorCORESET(obj);
                case 'searchspaces'
                    e = getConfigErrorSearchSpaces(obj);
                case 'pucch'
                    e = getConfigErrorPUCCH(obj);
                case 'csirs'
                    e = getConfigErrorCSIRS(obj);
                case 'srs'
                    e = getConfigErrorSRS(obj);
            end
        end

        %% Interaction with side panels
        % Update the visibility of all dependent parameters
        function updateControlsVisibility(obj, channelName)
            switch lower(channelName)
                case {'pdsch', 'pusch'}
                    updateControlsVisibilityPXSCH(obj);
                case 'pdcch'
                    updateControlsVisibilityPDCCH(obj);
                case 'pucch'
                    updateControlsVisibilityPUCCH(obj);
                case 'csirs'
                    updateControlsVisibilityCSIRS(obj);
                case 'srs'
                    updateControlsVisibilitySRS(obj);
            end
        end

        % Update side panel with wavegen config object data
        function mapCache2SidePanel(obj, channelName)
            switch lower(channelName)
                case {'pdsch', 'pusch'}
                    mapCache2SidePanelPXSCH(obj);
                case 'pdcch'
                    mapCache2SidePanelPDCCH(obj);
                case 'pucch'
                    mapCache2SidePanelPUCCH(obj);
                case 'csirs'
                    mapCache2SidePanelCSIRS(obj);
                case 'srs'
                    mapCache2SidePanelSRS(obj);
            end
        end
    end

    % Methods that are private but can be accessed only by nr5G_SSB_Dialog
    methods (Access = {?wirelessWaveformGenerator.nr5G_SSB_DataSource})
        function out = hasConfigError(obj, channelName)
            % Return the state of the configuration for CHANNELNAME. The
            % output is true if the app represents a valid wavegen
            % configuration for the specified channel, and false otherwise.

            % Retrieve the information from the right tab
            switch lower(channelName)
                case 'scscarriers'
                    out = hasConfigErrorSCSCarriers(obj);
                case 'bwp'
                    out = hasConfigErrorBWP(obj);
                case {'pdsch', 'pusch', 'pxsch'}
                    out = hasConfigErrorPXSCH(obj);
                case 'pdcch'
                    out = hasConfigErrorPDCCH(obj);
                case 'coreset'
                    out = hasConfigErrorCORESET(obj);
                case 'searchspaces'
                    out = hasConfigErrorSearchSpaces(obj);
                case 'pucch'
                    out = hasConfigErrorPUCCH(obj);
                case 'csirs'
                    out = hasConfigErrorCSIRS(obj);
                case 'srs'
                    out = hasConfigErrorSRS(obj);
            end
        end
    end
end

function out = hasSidePanel(tabName)

    out = any(strcmpi(tabName,{'pdsch','pusch','pucch','srs'}));

end

function [tileOccupancy, tileID] = addTile(tileOccupancy, tileID, childOrder, parentTag, tag, varargin)
    documentID = parentTag + "DocumentGroup_" + tag;
    str = struct('showOrder', childOrder, 'id', documentID);
    tileOccupancy(tileID).children = [tileOccupancy(tileID).children str];
    
    if nargin > 5
        childIDTag = varargin{1};
        tileOccupancy(tileID).showingChildId = parentTag + "DocumentGroup_" + childIDTag;
    end
    
    tileID = tileID + 1;
end
