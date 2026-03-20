classdef nr5G_Full_Base_Dialog < wirelessWaveformGenerator.nr5G_Dialog & ...
                                 wirelessWaveformApp.nr5G_Full_Layout
    % Common functionality for both Downlink and Uplink Full 5G wavegen

    %   Copyright 2020-2025 The MathWorks, Inc.

    properties (Hidden)
        % Handle tab transitions
        lastResourceGridInFocus = 'Resource Grid (BWP#1)';
        maxNumBWPFigs = 9;       % An upper bound on BWP figs to be visualized

        % Resource grid color
        GridConflictColor = [1 0.5 0.5]; % Red
        Initializing = true;    % prevents updateGrid/updateChannelBandwidthView from being called multiple times during initialization
    end

    methods 
        function hPropDb = getPropertySet(obj)
            % Define visualizations that are specific to this waveform extension.
            % In this case, we have
            % - one resource grid plot for each BWP (up to maxNumBWPFigs)
            % - Plus the "Channel Bandwidth View" plot
            resGrids = cellstr(strcat('Resource Grid (BWP#', num2str((1:obj.maxNumBWPFigs)'), ')'))';
            hPropDb = extmgr.PropertySet(...
                'Visualizations',   'mxArray', [resGrids {'Channel Bandwidth View'}]);
        end

    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_Full_Base_Dialog(parent, isDownlink)
            obj@wirelessWaveformGenerator.nr5G_Dialog(parent); % call base constructor
            obj@wirelessWaveformApp.nr5G_Full_Layout(parent, isDownlink);

            % SR is Auto ([]) by default:
            srChanged(obj, []);
            phaseCompChanged(obj, []);
        end

        function adjustSpec(obj)
            adjustSpec@wirelessWaveformGenerator.nr5G_Dialog(obj);
            obj.LabelWidth = 130;
            adjustSpec@wirelessWaveformApp.nr5G_Full_Layout(obj);
        end

        function config = getConfigurationForSave(obj)
            % Info to save is:
            % - 1 nrXLCarrierConfig object (workspace variables are not saved)
            % - plus filtering info (which retains workspace variables)
            % - SIB1 configuration info
            config.waveform = getConfiguration(obj);
            % Add SIB1 dialog info if required
            ssbDataSourceDialog = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
            dialogs = obj.Parent.WaveformGenerator.pParameters.DialogsMap;
            if isKey(dialogs,ssbDataSourceDialog)
                ssbDialog = dialogs(ssbDataSourceDialog);
                sib1Enabled = ssbDialog.Sib1CheckGUI.Value;
                sib1dialog = dialogs('wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog');
                config.SIB1 = getConfigurationForSave(sib1dialog,sib1Enabled);
            end
            % Filtering
            config.filtering = getConfigurationForSave(obj.Parent.FilteringDialog);
        end

        function setupDialog(obj)
            % Actions performed when entering this Full DL/UL wavegen extension:

            % turn off the listener while figures are enabled, so that client
            % actions (similar to clicking) are not triggered
            obj.ClientActionListener.Enabled = false;

            obj.lastFigInFocus = 'Main'; % set this now, as PXSCH/PUCCH Advanced visibility is determined by last fig in focus
            obj.setExtraConfigFigVisibility(true);
            % start from the Main tab if user goes back to DL 5G from the in-App gallery:
            if ~isempty(obj.scsCarriersTable)
                setupDialog@wirelessWaveformGenerator.nr5G_Dialog(obj);
            end
            setupDialog@wirelessWaveformApp.nr5G_Full_Layout(obj);

            % Customize displayed visualization options (1 fig per BWP) + no TimeScope/ConstellationDiagram.
            % 5G-specific behavior can possibly be merged with
            % wirelessWaveformGenerator.waveformGenerator.updateScopeOptions
            visualizeBtn = find(obj.Parent.AppObj.pPrimaryTab, 'plots');
            visualizeBtn.DynamicPopupFcn =  @(a, b) updateScopeOptions5GFull(obj, []);
        end

        function resetCustomVisuals(obj)
            % Executed during New session, to initialize desired state of visuals
            resetCustomVisuals@wirelessWaveformGenerator.nr5G_Dialog(obj);
            resetCustomVisuals@wirelessWaveformApp.nr5G_Full_Layout(obj);
        end

        function updateParametersFig(obj)
            updateParametersFig@wirelessWaveformApp.nr5G_Full_Layout(obj); % this is where create UI Controls will place items
        end

        function updateDisabled(obj)
            updateDisabled@wirelessWaveformApp.nr5G_Full_Layout(obj);
        end

        function outro(obj, newDialog)
            % Executed when moving to a new waveform type (e.g., 5G -> QAM), i.e., Cleanup

            outro@wirelessWaveformApp.nr5G_Full_Layout(obj,newDialog);
            outro@wirelessWaveformGenerator.nr5G_Dialog(obj);
            clearConflictResourceGrids(obj);

        end

        function b = mayHaveEmptyTimePeriods(~)
            % This is needed to avoid finishing with a non-meaningful Spectrum visualization
            b = true;
        end

        function enableBWPVisual(obj,bwpIdx)
            % Enable BWP resource grid figures by BWP index
            bwpIdx = unique(min(bwpIdx,obj.maxNumBWPFigs));
            enable = false(1,obj.maxNumBWPFigs);
            enable(bwpIdx) = true;
            for idx = 1:obj.maxNumBWPFigs
                obj.setVisualState(['Resource Grid (BWP#' num2str(idx) ')'], enable(idx));
            end
        end

        function defaultVisualLayout(obj)
            % Reset the custom visualization state
            % By default, plot all the resource grids corresponding to all
            % existing BWPs
            if isprop(obj,'bwpWaveConfig') && ~isempty(obj.bwpWaveConfig) % switching back from other extensions - plot all BWPs
                bwpIndices = arrayfun((@(x) x.BandwidthPartID),[obj.bwpWaveConfig{:}]);
            else % initializing
                bwpIndices = 1;
            end
            enableBWPVisual(obj,bwpIndices);

            % Turn Channel Bandwidth View on by default
            obj.setVisualState('Channel Bandwidth View', true);
        end

        function props = displayOrder(~)
            props = {'Label'; 'FrequencyRange'; 'ChannelBandwidth'; 'NCellID'; 'NumSubframes';...
                     'InitialNSubframe'; 'WindowingSource'; 'WindowingPercent'; 'SampleRateSource'; ...
                     'SampleRate'; 'PhaseCompensation'; 'CarrierFrequency'};
        end

        function waveform = generateWaveform(obj)
            % The actual waveform generation! Using nrWaveformGenerator.
            cfg = getConfiguration(obj);

            % Suppress these warnings in the command window:
            % 1. DM-RS warnings for PXSCH
            % 2. Group hopping warning for PUCCH
            wid = ["nr5g:nrPXSCH:DMRSParametersNoSymbols";...
                   "nr5g:nrPXSCH:CustomSymbolSetNoSymbols";...
                   "nr5g:nrWaveformGenerator:InvalidGroupHopping";...
                   "nr5g:nrWaveformGenerator:SmallMaxNumLayers";...
                   "nr5g:nrWaveformGenerator:SmallMaxQm"];
            c = wirelessWaveformApp.internal.suppressWarning(wid); %#ok<NASGU>

            % Generate waveform
            [waveform, obj.gridSet] = nrWaveformGenerator(cfg);
        end

        function cfg = applyConfiguration(obj, cfg)
            % Used on New, Open Session, and openInGenerator.

            cfg = applyConfiguration@wirelessWaveformApp.nr5G_Full_Layout(obj, cfg, 'WWG');
        end

        %% Tile Handling
        function cols = getNumTileColumns(obj, val)
            % Tile architecture always uses 2 columns (under all tabs)
            cols = getNumTileColumns@wirelessWaveformApp.nr5G_Full_Layout(obj, val);
        end
        function rows = getNumTileRows(obj, val)
            % Tile architecture always uses 2 rows (under all tabs)
            rows = getNumTileRows@wirelessWaveformApp.nr5G_Full_Layout(obj, val);
        end

        function w = getRowWeights(obj, val)
            % Rows are split evenly
            w = getRowWeights@wirelessWaveformApp.nr5G_Full_Layout(obj, val);
        end

        function tiles = getNumTiles(obj, ~)
            configTab = 1;
            resourceGrid = false;
            for idx = 1:obj.maxNumBWPFigs
                resourceGrid = resourceGrid || obj.getVisualState(['Resource Grid (BWP#' num2str(idx) ')']);
            end
            advancedRightPane = obj.pxschExtraTile || obj.pxcchExtraTile || obj.xrsExtraTile;
            spectrum = obj.Parent.AppObj.pPlotSpectrum;
            channelBWView = obj.getVisualState('Channel Bandwidth View');

            visualOrInfo = 1;
            tiles = configTab + visualOrInfo + double(advancedRightPane) + double(~advancedRightPane && resourceGrid && (spectrum || channelBWView));
        end

        function n = numVisibleFigs(obj)
            n = numVisibleFigs@wirelessWaveformGenerator.nr5G_Dialog(obj);
            n = n+obj.pxschExtraTile;
        end

        function [tileCount, tileCoverage, tileOccupancy] = getTileLayout(obj, ~)

            tileCount = getNumTiles(obj);

            advancedRightPane = obj.pxschExtraTile || obj.pxcchExtraTile || obj.xrsExtraTile;
            if advancedRightPane
                tileCoverage = [1 2; 3 2];
            else
                if tileCount == 3
                    tileCoverage = [1 1; 2 3];
                else
                    tileCoverage = [1 1; 2 2];
                end
            end

            [tileCount1, ~, tileOccupancy1, tileID] = getTileLayout@wirelessWaveformApp.nr5G_Full_Layout(obj);
            tileOccupancy = repmat(tileOccupancy1(1), tileCount, 1);
            tileOccupancy(1:tileCount1) = tileOccupancy1;

            if ~isempty(obj.bwpTable)
                bwpIDs = cellstr(string(cellfun(@(x)(cat(1,[],x.BandwidthPartID)),obj.bwpWaveConfig))); % Get all the BWP IDs in the form {'1', '2', ...}
                bwpIDs(obj.maxNumBWPFigs+1:end) = []; % Remove the extra BWP IDs that go above the maximum number allowed
            else
                bwpIDs = {'1'};
            end
            someBWPenabled = false;
            for bwpNo = 1:length(bwpIDs)
                bwpID = str2double(bwpIDs{bwpNo});
                if bwpID <= obj.maxNumBWPFigs
                    if obj.getVisualState(['Resource Grid (BWP#' bwpIDs{bwpNo} ')'])
                        documentID = getTag(obj.Parent.AppObj) + "DocumentGroup_ResourceGridBWP" + bwpIDs{bwpNo};
                        str = struct('showOrder', bwpNo, 'id', documentID);
                        tileOccupancy(tileID).children = [tileOccupancy(tileID).children str];
                        tileOccupancy(tileID).showingChildId = documentID;
    
                        someBWPenabled = true;
                    end
                end
            end
            gridTileID = tileID;

            if ~advancedRightPane && someBWPenabled
                tileID = tileID + 1;
            end

            appObj = obj.Parent.AppObj;
            childOrder = 0;
            if appObj.pPlotSpectrum && ~isempty(appObj.pSpectrum1)
                childOrder = childOrder + 1;
                prefix = appObj.getWebScopePrefix(appObj.pSpectrum1);
                documentID = [prefix getClientId(appObj.pSpectrum1)];
                str = struct('showOrder', childOrder, 'id', documentID);
                tileOccupancy(tileID).children = [tileOccupancy(tileID).children str];
                if tileID ~= gridTileID
                    tileOccupancy(tileID).showingChildId = documentID;
                end
            end

            if appObj.pPlotCCDF
                childOrder = childOrder + 1;
                documentID = getTag(obj.Parent.AppObj) + "DocumentGroup_CCDF";
                str = struct('showOrder', childOrder, 'id', documentID);
                if isempty(tileOccupancy(tileID).children)
                    tileOccupancy(tileID).showingChildId = documentID;
                end
                tileOccupancy(tileID).children = [tileOccupancy(tileID).children str];
            end

            if appObj.pPlotTimeScope
                childOrder = childOrder + 1;
                prefix = appObj.getWebScopePrefix(appObj.pTimeScope);
                documentID = [prefix getClientId(appObj.pTimeScope)];
                str = struct('showOrder', childOrder, 'id', documentID);
                tileOccupancy(tileID).children = [tileOccupancy(tileID).children str];
            end

            for k = 1:length(obj.visualNames)
                if ~startsWith(obj.visualNames{k}, 'Resource Grid') && obj.getVisualState(obj.visualNames{k})
                    childOrder = childOrder + 1;
                    documentID = getTag(obj.Parent.AppObj) + "DocumentGroup_" + obj.getFigureTag(obj.visualNames{k});
                    str = struct('showOrder', childOrder, 'id', documentID);
                    if isempty(tileOccupancy(tileID).children)
                        tileOccupancy(tileID).showingChildId = documentID;
                    end
                    tileOccupancy(tileID).children = [tileOccupancy(tileID).children str];
                end
            end
        end

        %% Visualization
        function customVisualizations(obj, varargin)
            % Executes upon waveform generation to update all visuals

            % Resource Grid
            wgc = getConfiguration(obj);
            resetStatus = true;
            channelName = obj.lastFigInFocus;
            updateGrid(obj, wgc, resetStatus, channelName);

            % Channel bandwidth view
            updateChannelBandwidthView(obj);
        end

        function figureAdded(obj,figName)
            % This method initializes/updates the figure before it is added to
            % the current visuals.
            figureAdded@wirelessWaveformGenerator.nr5G_Dialog(obj,figName);

            % Update the RE mapping too.
            if contains(figName(isletter(figName)),'ResourceGrid') % Tag does not contain spaces
                % Initialize RE mapping
                updateREVisual(obj,obj.lastFigInFocus);
            end
        end

        function layoutPanels(obj)
            layoutPanels@wirelessWaveformGenerator.nr5G_Dialog(obj);
            layoutPanels@wirelessWaveformApp.nr5G_Full_Layout(obj);
            if obj.Initializing
                obj.Initializing = false; % also allow grid + channel bandwidth view updates. will do only 1 plot at extensionTypeChange->clearScopes->resetCustomVisuals
            end
        end

        function extraPanels = getExtraPanels(obj)
            extraPanels = getExtraPanels@wirelessWaveformApp.nr5G_Full_Layout(obj);
        end

        function b = spectrumEnabled(~)
            b = true;
        end

        function cellDialogs = getDialogsPerColumn(obj)
            % Stack Filtering panel below the 5G main configuration panel
            cellDialogs{1} = {obj; ...
                obj.Parent.FilteringDialog};
        end

        %% Listener handling
        function postSelectedTabChanged(obj)

            appObj = obj.Parent.AppObj;
            currTab = appObj.AppContainer.SelectedToolstripTab.tag;
            inRadioTab = strcmp(currTab, 'transmitterTab');
            if ~inRadioTab
                % Tranmitter->Generator => hide the left-side panel again
                for idx = 1:numel(appObj.AppContainer.getPanels)
                    appObj.AppContainer.getPanels{idx}.Opened = false;
                end
            end
        end

        function panel = getPanel(obj)
            panel = obj.Panel;
        end

    end

    % Methods specific to this class and that are currently used by
    % neighboring classes
    methods (Access = public)
        function frChanged(obj, ~)
            % Executed when frequency range changes (FR1<->FR2)
            frChangedBase(obj); % Actions that are common with 5G presets
            frChanged@wirelessWaveformApp.nr5G_Full_Layout(obj);
        end
        function frChangedGUI(obj, ~)
            % For the custom 5G Downlink/Uplink, this performs the same actions
            % as frChanged.
            frChangedGUI@wirelessWaveformApp.nr5G_Full_Layout(obj);
        end

        %% Visualization
        function updateGrid(obj, varargin)
            % Live grid update when a relevant property changes or waveform is
            % generated
            if obj.Initializing || ~obj.Parent.AppObj.pShowVisualizations
                return; % will do only 1 plot at extensionTypeChange->clearScopes->resetCustomVisuals
            end

            [wgc,gridset,waveResources] = updateGrid@wirelessWaveformGenerator.nr5G_Dialog(obj, varargin{:});

            try
                % If updateGrid in superclass succeeded
                if ~isempty(gridset)
                    % Detect conflicts among channels/signals
                    conflicts = nr5g.internal.wavegen.detectConflict(wgc,waveResources);

                    % Update conflict visualization and textbox callback
                    updateConflictResourceGrids(obj,wgc,conflicts);

                    % Highlight conflicts in configuration tables
                    markConflictInConfigTables(obj,conflicts);

                    % Check LBRM properties and update banner if necessary,
                    % when there are no other errors
                    pxschStr = obj.PXSCHfigureName;
                    chs = wgc.(pxschStr);
                    for idx = 1:numel(chs)
                        try
                            if chs{idx}.Coding && chs{idx}.LimitedBufferRateMatching
                                validateLBRMProperties(chs{idx},pxschStr,idx,true);
                            end
                        catch eLBRM
                            updateConfigDiagnostic(obj, eLBRM.message);
                            break; % Only throw warning for the first instance with problems
                        end
                    end

                end

            catch e
                updateConfigDiagnostic(obj, e.message);
            end

        end

        function updateREVisual(obj, channelName, varargin)
            % Live update of the RE mapping grid when a relevant property
            % changes, waveform is generated, or the tab in focus changes

            if obj.Initializing
                return
            end

            if nargin==3
                resetFlag = matches(varargin{1},'reset');
            else
                resetFlag = false;
            end

            % Construct the channel name and ID, if the tab is not Main or SS Burst
            if matches(channelName,["PDSCH","PUSCH","CORESET","SearchSpaces","PDCCH","PUCCH","CSIRS","CSI-RS","SRS"])
                % Channel is implemented as an nr5G_Table object
                channelName = replace(channelName,'CSIRS','CSI-RS'); % Ensure the CSI-RS name shows with the hyphen
                channelName = replace(channelName,["CORESET", "SearchSpaces"],'PDCCH'); % The RE Map is displayed for PDCCH
                tableObj = get5GTableObject(obj, channelName);
                selection = wirelessWaveformApp.internal.Utility.getSingleSelection(tableObj.Selection); % Get the channel instance
                id = tableObj.AllIDs(selection); % Get the channel instance ID
                channelName = [channelName ' ' num2str(id)];
            else
                % Tab is Main or SS Burst
                selection = [];
            end

            try
                % Update all activated BWP grids
                availableBWPIDs = cellfun(@(x)(cat(1,[],x.BandwidthPartID)),obj.bwpWaveConfig);
                for bwpIdx = availableBWPIDs(ismember(availableBWPIDs, 1:obj.maxNumBWPFigs))
                    name = ['Resource Grid (BWP#' num2str(bwpIdx) ')'];
                    if obj.getVisualState(name)
                        ax = getResourceGridAxes(obj,bwpIdx);
                        wirelessWaveformGenerator.internal.updateREMapping(obj,ax,bwpIdx,channelName,selection,resetFlag);
                    end
                end
            catch e
                updateConfigDiagnostic(obj, e.message);
            end
        end
    end

    % Protected methods
    methods (Access = protected)
        function popup = updateScopeOptions5GFull(obj, ~)
            % Executed when the "Visualize" button is clicked on the toolstrip
            % - do not offer post-OFDM time scope and constellation diagram
            % - also only offer already created BWP

            appObj = obj.Parent.AppObj;

            popup = matlab.ui.internal.toolstrip.PopupList();

            subItemTimeScope = matlab.ui.internal.toolstrip.ListItemWithCheckBox('Time Scope');
            subItemTimeScope.ShowDescription = false;
            subItemTimeScope.Value = appObj.pPlotTimeScope;
            subItemTimeScope.Tag = 'timeScope';
            subItemTimeScope.ValueChangedFcn = @(a, b) visualChanged(appObj, subItemTimeScope);
            popup.add(subItemTimeScope);

            sub_item1 = matlab.ui.internal.toolstrip.ListItemWithCheckBox('Spectrum Analyzer');
            sub_item1.ShowDescription = false;
            sub_item1.Value = appObj.pPlotSpectrum;
            sub_item1.Tag = 'spectrumAnalyzer';
            sub_item1.ValueChangedFcn = @(a, b) visualChanged(appObj, sub_item1);
            popup.add(sub_item1);

            bwps = cellfun(@(x)(cat(1,[],x.BandwidthPartID)),obj.bwpWaveConfig);

            propSet = obj.getPropertySet();
            if ~isempty(propSet.findProperty('Visualizations'))
                visuals = propSet.getPropValue('Visualizations');
                for idx = 1:length(visuals)
                    thisVis = visuals{idx};
                    if startsWith(thisVis, 'Resource Grid') && ~any(str2double(thisVis(end-1)) == bwps)
                        % do not offer Grid options for non-defined BWPs
                        continue;
                    end
                    sub_item4 = matlab.ui.internal.toolstrip.ListItemWithCheckBox(thisVis);
                    sub_item4.ShowDescription = false;
                    sub_item4.Tag = obj.getFigureTag(thisVis);
                    sub_item4.Value = obj.getVisualState(thisVis);
                    sub_item4.ValueChangedFcn = @(a, b) visualChanged(appObj, sub_item4);
                    popup.add(sub_item4);
                end

                % CCDF
                sub_itemCCDF = matlab.ui.internal.toolstrip.ListItemWithCheckBox('CCDF');
                sub_itemCCDF.ShowDescription = false;
                sub_itemCCDF.Value = obj.Parent.AppObj.pPlotCCDF;
                sub_itemCCDF.Tag = 'CCDF';
                sub_itemCCDF.ValueChangedFcn = @(a, b) visualChanged(obj.Parent.AppObj, sub_itemCCDF);
                popup.add(sub_itemCCDF);
            end
        end

        function bwChanged(obj, ~)
            bwChanged@wirelessWaveformApp.nr5G_Full_Layout(obj);
        end

        %% Visualization
        function updateConflictResourceGrids(obj,wgc,conflicts)

            % Update conflict grids in activated BWPs
            for bwpIdx = 1:length(wgc.BandwidthParts)
                ax = getResourceGridAxes(obj,wgc.BandwidthParts{bwpIdx}.BandwidthPartID);
                if ~isempty(ax)
                    wirelessWaveformGenerator.internal.plotResourceGridConflicts(ax,wgc,conflicts,bwpIdx,obj.GridConflictColor);

                    % Update resource grid text box callback with conflicts
                    fig = ax.Parent.Parent;
                    fig.WindowButtonMotionFcn{end+1} = conflicts;
                end
            end

        end

        function clearConflictResourceGrids(obj)
            % Get BWP config
            try
                wgc = getConfiguration(obj);
                bwp = wgc.BandwidthParts;
            catch e
                updateConfigDiagnostic(obj, e.message);
                % If the current configuration is invalid, get the stored
                % value of the BWP config object and use that as the
                % closest to the actual value.
                bwp = obj.bwpWaveConfig;
            end

            % Clear conflict grids in activated BWPs before switching
            for bwpIdx = 1:length(bwp)
                ax = getResourceGridAxes(obj,bwp{bwpIdx}.BandwidthPartID);
                if ~isempty(ax)
                    tag = 'wirelessWaveformGenerator.internal.plotResourceGridConflicts';
                    imgCfl = findobj(ax,'Type', 'Image','Tag',tag);
                    delete(imgCfl);
                end
            end

        end
        
        function customFilteringChangedGUI(obj)
            % Default layoutPanels creates a panel that is on top and hides SCS & BWP tables
            className = 'wirelessAppContainer.FilteringDialog';
            dlg = obj.Parent.DialogsMap(className);

            filteringChangedGUI(dlg); % the actually intended method
        end

        % Modify channel names in waveform resources for display in the PRB
        % resource grid text box. The channel instances displayed reflect the
        % row number in the corresponding configuration table instead of the
        % position in the nrXLCarrierConfig object.
        function waveResources = updateChannelNames(obj,waveResources)

            channelTypes = fieldnames(waveResources);

            for c = 1:numel(channelTypes)
                chType = channelTypes{c};
                channels = waveResources.(chType);

                idx = contains(obj.tableObjName,replace(chType,["PDSCH","PUSCH"],"PXSCH"),"IgnoreCase",true);
                if any(idx)
                    % Get the list of all IDs associated with this channel
                    nrTableObj = obj.(obj.tableObjName(idx));
                    ids = nrTableObj.AllIDs;
                    for ch = 1:length(channels)
                        channels(ch).Name = [chType, num2str(ids(ch))];
                    end
                end
                waveResources.(channelTypes{c}) = channels;
            end

        end

        %% Listener handling
        function propertyChanged(obj, tmp, data)

            if ~(strcmp(data.EventName, 'PropertyChanged') && strcmp(data.PropertyName, 'SelectedChild')) || ...
                    ~isvalid(obj.Parent) || ...
                    ~(isa(obj.Parent.CurrentDialog, 'wirelessWaveformGenerator.nr5G_DL_Dialog')  || ...
                    isa(obj.Parent.CurrentDialog, 'wirelessWaveformGenerator.nr5G_UL_Dialog'))

                return;
            end

            propertyChanged@wirelessWaveformApp.nr5G_Full_Layout(obj, tmp ,data)
        end

        function onClientAction(obj, tmp, ev)
            % Executes when, e.g., a click happens. Need to filter the clicks that
            % correspond to a tab change, so that we possibly change the PDSCH layout.

            if strcmpi(ev.EventData.EventType, 'ACTIVATED')
                if ~(isa(obj.Parent.CurrentDialog, 'wirelessWaveformGenerator.nr5G_DL_Dialog')  || ...
                        isa(obj.Parent.CurrentDialog, 'wirelessWaveformGenerator.nr5G_UL_Dialog')) || ...
                        ~isa(obj.Parent.CurrentDialog,class(obj)) || ...
                        isempty(ev.EventData.Client)
                    return;
                end

                onClientAction@wirelessWaveformApp.nr5G_Full_Layout(obj, tmp, ev);
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
                onTabChange@wirelessWaveformApp.nr5G_Full_Layout(obj, newTabName);
            elseif any(startsWith(newTabName, 'Resource'))
                obj.lastResourceGridInFocus = newTabName;
            end

            if isChannelOrSignal
                updateREVisual(obj, newTabName);
            end
        end
    end

end
