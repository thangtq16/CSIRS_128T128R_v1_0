classdef nr5G_PDCCH_Tab < handle
    % Dialog class that handles every graphical aspect related to PDCCH

    % Copyright 2023-2024 The MathWorks, Inc.

    properties
        % Object-specific properties
        pdcchFig % Figure containing PDCCH, CORESET, and Search Space
        pdcchGridLayout; % Grid containing PDCCH, CORESET, and Search Space
        coresetTable % CORESET table object
        searchSpacesTable % Search Spaces table object
        pdcchTable % PDCCH table object
    end

    properties (AbortSet)
        coresetWaveConfig % Cached CORESET config object
        searchSpacesWaveConfig % Cached Search Spaces config object
        pdcchWaveConfig % Cached PDCCH config object
    end

    properties (Constant)
        PXCCHfigureName    = 'PDCCH'; % Figure name
        pxcchExtraFigTag   = 'pxcchSingleChannelFig'; % Side panel figure tag
    end

    properties (Access = private)
        % Store default wavegen config objects
        DefaultConfigCORESET
        DefaultConfigSearchSpaces
        DefaultConfigPDCCH
    end

    properties (Access = private, Dependent)
        % Information about configuration error. If the wavegen
        % configuration for this channel represented by the app has an
        % error, ConfigError stores the MException thrown when trying to
        % update the wavegen configuration object with the app data. If
        % there is no error, ConfigError is empty.
        ConfigErrorCORESET
        ConfigErrorSearchSpaces
        ConfigErrorPDCCH
    end

    methods(Abstract)
        createTableGridLayout % Implemented in nr5G_Full_Layout
    end

    % Constructor and public methods
    methods (Access = public)
        function obj = nr5G_PDCCH_Tab(defaultWaveConfig, invisibleEntriesCORESET, invisibleEntriesSS, invisibleEntriesPDCCH)
            % Create grid layout that contains the downlink control tables
            createPDCCHGridLayout(obj);

            % Construct the CORESET table object
            defaultConfigCORESET = defaultWaveConfig.CORESET;
            obj.coresetTable = wirelessWaveformApp.nr5G_CORESET_Table(obj.pdcchGridLayout, defaultConfigCORESET, invisibleEntriesCORESET);
            % Place CORESET table in position (1,1) of the PDCCH grid layout
            setLayout(obj.coresetTable, Row=1, Column=1);
            % Initialize the cached configuration object
            obj.coresetWaveConfig = defaultConfigCORESET;
            obj.DefaultConfigCORESET = defaultConfigCORESET;

            % Construct the SearchSpaces table object
            defaultConfigSS = defaultWaveConfig.SearchSpaces;
            obj.searchSpacesTable = wirelessWaveformApp.nr5G_SearchSpaces_Table(obj.pdcchGridLayout, defaultConfigSS, invisibleEntriesSS);
            % Place SearchSpaces table in position (1,2) of the PDCCH grid layout
            setLayout(obj.searchSpacesTable, Row=1, Column=2);
            % Initialize the cached configuration object
            obj.searchSpacesWaveConfig = defaultConfigSS;
            obj.DefaultConfigSearchSpaces = defaultConfigSS;

            % Construct the PDCCH table object
            defaultConfigPDCCH = defaultWaveConfig.PDCCH;
            obj.pdcchTable = wirelessWaveformApp.nr5G_PDCCH_Table(obj.pdcchGridLayout, defaultConfigPDCCH, invisibleEntriesPDCCH);
            % Place PDCCH table in position (2,1:2) of the PDCCH grid layout
            setLayout(obj.pdcchTable, Row=2, Column=[1,2]);
            % Initialize the cached configuration object
            obj.pdcchWaveConfig = defaultConfigPDCCH;
            obj.DefaultConfigPDCCH = defaultConfigPDCCH;

            % Cross-table listeners
            obj.coresetTable.addlistener('IDChanged',@(src,event)obj.updateCORESEToptions(src,event));
            obj.searchSpacesTable.addlistener('IDChanged',@(src,event)obj.updateSSoptions(src,event));
            obj.searchSpacesTable.addlistener('NumCandidatesChanged',@(src,event)obj.updateNumCandidates(src,event));

        end

        %% Update Configurations
        function waveConfig = updateCachedConfigCORESET(obj, action, changedConfigIndex)
            % Update the nrCORESETConfig configuration object and the DL
            % cached configuration object from the table and the potential
            % side panel.

            if action=="ConfigChange"
                % If one of the current CORESET instances has changed,
                % update the cached configuration from the table
                waveConfig = obj.coresetWaveConfig;
                waveConfig = updateConfiguration(obj.coresetTable, waveConfig);
                waveConfig = mapSidePanel2CfgObj(obj, waveConfig);
            else
                % If a CORESET instance has been added, removed, or
                % duplicated, simply update the cached configuration object
                % as there is no need to read the whole table
                waveConfig = wirelessWaveformApp.internal.Utility.updateConfigDimension(obj.coresetWaveConfig, obj.DefaultConfigCORESET(1), action, changedConfigIndex, obj.coresetTable.AllIDs, 'CORESETID');
            end

            % Update the cache
            obj.coresetWaveConfig = waveConfig;
        end

        function waveConfig = updateCachedConfigSearchSpaces(obj, action, changedConfigIndex)
            % Update the nrSearchSpaceConfig configuration object and the DL
            % cached configuration object from the table and the potential
            % side panel.

            if action=="ConfigChange"
                % If one of the current Search Space instances has changed,
                % update the cached configuration from the table
                waveConfig = obj.searchSpacesWaveConfig;
                waveConfig = updateConfiguration(obj.searchSpacesTable, waveConfig);
                waveConfig = mapSidePanel2CfgObj(obj, waveConfig);
            else
                % If a Search Space instance has been added, removed, or
                % duplicated, simply update the cached configuration object
                % as there is no need to read the whole table
                waveConfig = wirelessWaveformApp.internal.Utility.updateConfigDimension(obj.searchSpacesWaveConfig, obj.DefaultConfigSearchSpaces, action, changedConfigIndex, obj.searchSpacesTable.AllIDs, 'SearchSpaceID');
            end

            % Update the cache
            obj.searchSpacesWaveConfig = waveConfig;
        end

        function waveConfig = updateCachedConfigPDCCH(obj, action, changedConfigIndex)
            % Update the nrWavegenXConfig configuration object and the DL
            % cached configuration object from the table and the potential
            % side panel.

            if action=="ConfigChange"
                % If one of the current PDCCH instances has changed,
                % update the cached configuration from the table
                waveConfig = obj.pdcchWaveConfig;
                waveConfig = updateConfiguration(obj.pdcchTable, waveConfig);
                waveConfig = mapSidePanel2CfgObj(obj, waveConfig);
            else
                % If a PDCCH instance has been added, removed, or
                % duplicated, simply update the cached configuration object
                % as there is no need to read the whole table
                waveConfig = wirelessWaveformApp.internal.Utility.updateConfigDimension(obj.pdcchWaveConfig, obj.DefaultConfigPDCCH, action, changedConfigIndex, obj.pdcchTable.AllIDs);
            end

            % Update the cache
            obj.pdcchWaveConfig = waveConfig;
        end

        %% Apply Configurations
        function applyConfigCORESET(obj, waveCfg)
            % Apply configuration when starting a new session, loading an
            % existing session, or using openInGenerator

            if isempty(obj.coresetTable)
                return; % App initialization
            end

            % Force the input configuration to be a row vector
            ch = waveCfg.CORESET(:)';

            % Use default if empty. This is possible only when using
            % openInGenerator.
            if isempty(ch)
                ch = obj.DefaultConfigCORESET;
            end

            % Update the cache
            obj.coresetWaveConfig = ch;

            % Map the channel configuration to the table
            applyConfiguration(obj.coresetTable, ch);

            % Update the side panel
            mapCache2SidePanelPDCCH(obj);
            updateControlsVisibilityPDCCH(obj);
        end

        function applyConfigSearchSpaces(obj, waveCfg, nvargs)
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
                % 1. numel(nvargs.IDs) < numel(waveCfg.SearchSpaces)
                % 2. isempty(nvargs.IDs) && (numel(obj.searchSpacesTable.AllIDs)<numel(waveCfg.SearchSpaces))
                nvargs.AllowUIChange (1,1) logical = true;

                % List of row IDs. If AllowUIChange is false, the value of
                % IDs is used to specify the desired IDs of the rows. If
                % IDs is empty, the existing list given by AllIDs is used.
                % In these cases AllowUIChange is considered true
                % irrespective of the user input:
                % 1. numel(nvargs.IDs) < numel(waveCfg.SearchSpaces)
                % 2. isempty(nvargs.IDs) && (numel(obj.searchSpacesTable.AllIDs)<numel(waveCfg.SearchSpaces))
                nvargs.IDs (1,:) uint8 = [];
            end

            if isempty(obj.searchSpacesTable)
                return; % App initialization
            end

            % Force the input configuration to be a row vector
            ch = waveCfg.SearchSpaces(:)';

            % Use default if empty. This is possible only when using
            % openInGenerator.
            if isempty(ch)
                ch = obj.DefaultConfigSearchSpaces;
            end

            % Update the cache
            obj.searchSpacesWaveConfig = ch;

            % Map the channel configuration to the table
            applyConfiguration(obj.searchSpacesTable, ch, AllowUIChange=nvargs.AllowUIChange, IDs=nvargs.IDs);

            % Update the side panel
            mapCache2SidePanelPDCCH(obj);
            updateControlsVisibilityPDCCH(obj);
        end

        function applyConfigPDCCH(obj, waveCfg, nvargs)
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
                % 1. numel(nvargs.IDs) < numel(waveCfg.PDCCH)
                % 2. isempty(nvargs.IDs) && (numel(obj.pdcchTable.AllIDs)<numel(waveCfg.PDCCH))
                nvargs.AllowUIChange (1,1) logical = true;

                % List of row IDs. If AllowUIChange is false, the value of
                % IDs is used to specify the desired IDs of the rows. If
                % IDs is empty, the existing list given by AllIDs is used.
                % In these cases AllowUIChange is considered true
                % irrespective of the user input:
                % 1. numel(nvargs.IDs) < numel(waveCfg.PDCCH)
                % 2. isempty(nvargs.IDs) && (numel(obj.pdcchTable.AllIDs)<numel(waveCfg.PDCCH))
                nvargs.IDs (1,:) uint8 = [];
            end

            if isempty(obj.pdcchTable)
                return; % App initialization
            end

            % Force the input configuration to be a row vector
            ch = waveCfg.PDCCH(:)';

            % Disabled default if empty. This is possible only when using
            % openInGenerator.
            if isempty(ch)
                ch = obj.DefaultConfigPDCCH;
                ch{1}.Enable = false;
            end

            % Update the cache
            obj.pdcchWaveConfig = ch;

            % Map the channel configuration to the table
            applyConfiguration(obj.pdcchTable, ch, AllowUIChange=nvargs.AllowUIChange, IDs=nvargs.IDs);

            % Update the side panel
            mapCache2SidePanelPDCCH(obj);
            updateControlsVisibilityPDCCH(obj);
        end

        %% Check Configuration
        function out = hasConfigErrorCORESET(obj)
            % Return the state of the configuration for this channel. The
            % output is true if the app represents an invalid wavegen
            % configuration for the specified channel, and false otherwise.
            out = hasConfigError(obj.coresetTable);
        end

        function e = getConfigErrorCORESET(obj)
            % eturn the wavegen configuration error corresponding to this
            % channel, if any. If the app represents a valid wavegen
            % configuration for this channel, the output is empty.
            e = obj.ConfigErrorCORESET;
        end

        function out = hasConfigErrorSearchSpaces(obj)
            % Return the state of the configuration for this channel. The
            % output is true if the app represents an invalid wavegen
            % configuration for the specified channel, and false otherwise.
            out = hasConfigError(obj.searchSpacesTable);
        end

        function e = getConfigErrorSearchSpaces(obj)
            % Return the wavegen configuration error corresponding to this
            % channel, if any. If the app represents a valid wavegen
            % configuration for this channel, the output is empty.
            e = obj.ConfigErrorSearchSpaces;
        end

        function out = hasConfigErrorPDCCH(obj)
            % Return the state of the configuration for this channel. The
            % output is true if the app represents an invalid wavegen
            % configuration for the specified channel, and false otherwise.
            out = hasConfigError(obj.pdcchTable);
        end

        function e = getConfigErrorPDCCH(obj)
            % Return the wavegen configuration error corresponding to this
            % channel, if any. If the app represents a valid wavegen
            % configuration for this channel, the output is empty.
            e = obj.ConfigErrorPDCCH;
        end

        %% Side panel
        function mapCache2SidePanelPDCCH(~)
            % This method is invoked when there is a need to update the right
            % side panel with contents corresponding to a newly selected row

            % No side panel in PDCCH, so this method is a no-op
        end

        function updateControlsVisibilityPDCCH(~)
            % Update the visibility of all dependent parameters in the side
            % panel.

            % No side panel in PDCCH, so this method is a no-op
        end
    end

    methods
        function e = get.ConfigErrorCORESET(obj)
            % Retrieve configuration error from the table
            e = getConfigError(obj.coresetTable);
        end

        function e = get.ConfigErrorSearchSpaces(obj)
            % Retrieve configuration error from the table
            e = getConfigError(obj.searchSpacesTable);
        end

        function e = get.ConfigErrorPDCCH(obj)
            % Retrieve configuration error from the table
            e = getConfigError(obj.pdcchTable);
        end
    end

    methods (Access = private)
        %% Layout
        function createPDCCHGridLayout(obj)
            % Create a grid layout to properly position the three control
            % tables in the PDCCH figure
            % For the PDCCH tab, all tables are shown in their entirety
            % and the scrollbar is placed in this grid layout.
            N = 2; % Number of rows and columns so that the pdcchGridLayout is an N-by-N grid
            obj.pdcchGridLayout = createTableGridLayout(obj,obj.pdcchFig,'pdcch',N);
            obj.pdcchGridLayout.ColumnWidth = repmat({'fit'},1,N);
            obj.pdcchGridLayout.RowHeight = repmat({'fit'},1,N);
            obj.pdcchGridLayout.Scrollable = true;
        end

        %% Cross-table interactions
        function updateCORESEToptions(obj, ~, ~)
            % Set the available CORESET as options in the dropdown of the
            % SearchSpaces table, i.e., make it difficult for a Search Space
            % to link to a non-present CORESET.

            csetIDList = cellstr(string(obj.coresetTable.AllIDs));
            updatePropertyValues(obj.searchSpacesTable, PropertyName="CORESETID", NewList=csetIDList);
        end

        function updateSSoptions(obj, ~, ~)
            % Set the available SearchSpaces as options in the dropdown of
            % the PDCCH table, i.e., make it difficult for a PDCCH to link
            % to a non-present Search Space.

            ssIDList = cellstr(string(obj.searchSpacesTable.AllIDs));
            updatePropertyValues(obj.pdcchTable, PropertyName="SearchSpaceID", NewList=ssIDList);

            % Make sure that also the cached values of NumCandidates are
            % updated
            updateNumCandidates(obj);
        end

        function updateNumCandidates(obj, ~, event)
            % Update the values of NumCandidates cached in the PDCCH table
            % and used for internal cross-parameter validation.

            ssIDList = cellfun(@(x)(cat(1,[],num2str(x.SearchSpaceID))),obj.searchSpacesWaveConfig,'UniformOutput',false);
            NumCandidatesList = cellfun(@(x)(cat(1,[],x.NumCandidates)),obj.searchSpacesWaveConfig,'UniformOutput',false);

            if nargin > 2
                thisSSID = num2str(event.ChangedSelection);
                NumCandidatesList(cellfun(@(x)isequal(thisSSID,x),ssIDList)) = event.NewValue;
            end
            updatePropertyValues(obj.pdcchTable, PropertyName="NumCandidates", NewList={ssIDList, NumCandidatesList});
        end

        %% Side panel
        function cfg = mapSidePanel2CfgObj(~, cfg, varargin)
            % This method is invoked when there is a need to store the edits at
            % the right side panel and store these internally in the cache.

            % No side panel in PDCCH, so this method is a no-op
        end
    end

    methods(Access = {?wirelessWaveformGenerator.nr5G_SSB_DataSource})
        function amendCachedConfigCORESET(obj, chWaveCfg, cfgIndex)
            % Update the cached CORESET wave config object with the input
            % CHWAVECFG object.

            arguments
                obj
                chWaveCfg (:,1) cell
                cfgIndex (:,1) uint8
            end

            % Ensure the CORESET cache is updated with the new wavegen
            % configuration object
            if numel(obj.coresetWaveConfig)~=numel(obj.coresetTable.AllIDs)
                % A new configuration object needs to be appended
                obj.coresetWaveConfig = cat(1, obj.coresetWaveConfig, chWaveCfg);
            else
                % Update the relevant CORESET config objects
                obj.coresetWaveConfig(cfgIndex) = chWaveCfg;
            end
        end
        function amendCachedConfigSearchSpaces(obj, chWaveCfg, cfgIndex)
            % Update the cached SearchSpaces wave config object with the input
            % CHWAVECFG object.

            arguments
                obj
                chWaveCfg (:,1) cell
                cfgIndex (:,1) uint8
            end

            % Ensure the SearchSpaces cache is updated with the new wavegen
            % configuration object
            if numel(obj.searchSpacesWaveConfig)~=numel(obj.searchSpacesTable.AllIDs)
                % A new configuration object needs to be appended
                obj.searchSpacesWaveConfig = cat(1, obj.searchSpacesWaveConfig, chWaveCfg);
            else
                % Update the relevant SearchSpaces config objects
                obj.searchSpacesWaveConfig(cfgIndex) = chWaveCfg;
            end
        end
        function amendCachedConfigPDCCH(obj, chWaveCfg, cfgIndex)
            % Update the cached PDCCH wave config object with the input
            % CHWAVECFG object.

            arguments
                obj
                chWaveCfg (:,1) cell
                cfgIndex (:,1) uint8
            end

            % Ensure the PDCCH cache is updated with the new wavegen
            % configuration object
            if numel(obj.pdcchWaveConfig)~=numel(obj.pdcchTable.AllIDs)
                % A new configuration object needs to be appended
                obj.pdcchWaveConfig = cat(1, obj.pdcchWaveConfig, chWaveCfg);
            else
                % Update the relevant PDCCH config objects
                obj.pdcchWaveConfig(cfgIndex) = chWaveCfg;
            end
        end
    end
end
