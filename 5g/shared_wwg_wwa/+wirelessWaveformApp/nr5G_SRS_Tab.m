classdef nr5G_SRS_Tab < handle
    % Dialog class that handles every graphical aspect related to SRS
    
    % Copyright 2023-2024 The MathWorks, Inc.
    
    properties
        % Object-specific properties
        srsFig % Figure containing the dialog
        xrsSingleChannelFig % Figure containing the side panel
        srsGridLayout; % Grid containing the table
        srsTable % Table object

        paramSRS % Object to host (parallel) layouts outside the main AppObj.pParameters object
    end
    
    properties (AbortSet)
        srsWaveConfig % Cached SRS config object
    end
    
    properties (Constant)
        XRSfigureName    = 'SRS'; % Figure name
        xrsExtraFigTag   = 'xrsSingleChannelFig'; % Side panel figure tag
        classNameSRSAdv  = 'wirelessWaveformApp.nr5G_SRSAdvanced_Dialog'; % Side panel class name
    end
    
    properties (Access = private)
        DefaultConfigSRS % Store default wavegen config object
    end

    properties (Access = private, Dependent)
        % Information about configuration error. If the wavegen
        % configuration for this channel represented by the app has an
        % error, ConfigError stores the MException thrown when trying to
        % update the wavegen configuration object with the app data. If
        % there is no error, ConfigError is empty.
        ConfigError
    end
    
    methods(Abstract)
        createTableGridLayout % Implemented in nr5G_Full_Layout
        getParent % Implemented in nr5G_UL_Dialog
    end
    
    % Constructor and public methods
    methods (Access=public)
        function obj = nr5G_SRS_Tab(defaultWaveConfig, invisibleEntries)
            % Create grid layout that contains the table
            obj.srsGridLayout = createTableGridLayout(obj,obj.srsFig,'srs',1);
            
            % Construct the table object
            defaultConfigSRS = defaultWaveConfig.SRS;
            obj.srsTable = wirelessWaveformApp.nr5G_SRS_Table(obj.srsGridLayout, defaultConfigSRS, invisibleEntries);
            
            % Initialize the cached configuration object
            obj.srsWaveConfig = defaultConfigSRS;
            obj.DefaultConfigSRS = defaultConfigSRS;
        end
        
        %% SRS Configuration
        function waveConfig = updateCachedConfigSRS(obj, action, changedConfigIndex)
            % Update the nrWavegenXConfig configuration object and the UL
            % cached configuration object from the table and the potential
            % side panel.
            
            if action=="ConfigChange"
                % If one of the current SRS instances has changed, update
                % the cached configuration from the table
                waveConfig = getConfiguration(obj);
            else
                % If a SRS instance has been added, removed, or duplicated,
                % simply update the cached configuration object as there is
                % no need to read the whole table
                waveConfig = wirelessWaveformApp.internal.Utility.updateConfigDimension(obj.srsWaveConfig, obj.DefaultConfigSRS, action, changedConfigIndex, obj.srsTable.AllIDs);
            end
            
            % Update the cache
            obj.srsWaveConfig = waveConfig;
        end
        
        function applyConfigSRS(obj, chWaveCfg)
            % Apply configuration when starting a new session, loading an
            % existing session, or using openInGenerator
            
            if isempty(obj.srsTable)
                return; % App initialization
            end
            
            % Force the input configuration to be a row vector
            ch = chWaveCfg.SRS(:)';
            
            % Disabled default if empty. This is possible only when using
            % openInGenerator.
            if isempty(ch)
                ch = obj.DefaultConfigSRS;
                ch{1}.Enable = false;
            end
            
            % Update the cache
            obj.srsWaveConfig = ch;
            
            % Map the channel configuration to the table
            applyConfiguration(obj.srsTable, ch);
            
            % Update the side panel
            mapCache2SidePanelSRS(obj);
            updateControlsVisibilitySRS(obj);
        end

        function out = hasConfigErrorSRS(obj)
            % Return the state of the configuration for this channel. The
            % output is true if the app represents an invalid wavegen
            % configuration for the specified channel, and false otherwise.
            out = hasConfigError(obj.srsTable);
        end

        function e = getConfigErrorSRS(obj)
            % eturn the wavegen configuration error corresponding to this
            % channel, if any. If the app represents a valid wavegen
            % configuration for this channel, the output is empty.
            e = obj.ConfigError;
        end
        
        %% Side panel
        function createAdvancedTabSRS(obj)
            % Create 2nd param object, so that there is no interference with
            % 1st one in layoutPanels()
            appObj = getParent(obj);
            obj.paramSRS = wirelessAppContainer.Parameters(appObj.WaveformGenerator);
            
            % Advanced properties
            appObj.DialogsMap(obj.classNameSRSAdv) = eval([obj.classNameSRSAdv '(obj.paramSRS, obj.xrsSingleChannelFig)']);
            dlg = appObj.DialogsMap(obj.classNameSRSAdv);
            obj.paramSRS.CurrentDialog = dlg; % needed for layoutPanels()
            
            % Arrange placements of all right-side SRS panels
            layoutUIControls(dlg);
            layoutPanels(dlg);
        end
        
        function mapCache2SidePanelSRS(obj)
            % This method is invoked when there is a need to update the right
            % side panel with contents corresponding to a newly selected row
            
            % Update side panel and get the configuration object of the
            % selected SRS instance from the cache
            updateSidePanelTitles(obj);
            cfgIdx = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.srsTable.Selection);
            srsCfg = obj.srsWaveConfig{cfgIdx};
            
            dlg = getSidePanelDialog(obj);
            dlg.Label  = srsCfg.Label;
            dlg.CyclicShift = srsCfg.CyclicShift;
            dlg.GroupSeqHopping = srsCfg.GroupSeqHopping;
            dlg.NSRSID = srsCfg.NSRSID;
            dlg.SRSPositioning = srsCfg.SRSPositioning;
            dlg.FrequencyScalingFactor = srsCfg.FrequencyScalingFactor;
            dlg.EnableStartRBHopping = srsCfg.EnableStartRBHopping;
            dlg.StartRBIndex = srsCfg.StartRBIndex;
        end
        
        % Public getters for the private properties that represents the
        % side panel class names
        function className = getClassNameSRSAdv(obj)
            className = obj.classNameSRSAdv;
        end
        
        %% Update visibility of dependent parameters in the side panel
        function updateControlsVisibilitySRS(~)
            % Update the visibility of all dependent parameters in the side
            % panel.
            
            % No dependent control visibility for SRS, so this method is a
            % no-op.
        end
    end

    methods
        function e = get.ConfigError(obj)
            % Retrieve configuration error from the table, as the side
            % panel does not hold any configuration that is invalid at set
            % time
            e = getConfigError(obj.srsTable);
        end
    end

    methods (Access = private)
        %% Configuration
        function waveCfg = getConfiguration(obj)
            % Get the nrWavegenXConfig configuration object from the table
            % and the potential side panel.
            
            % Update cache with latest edits
            waveCfg = obj.srsWaveConfig;
            waveCfg = updateConfiguration(obj.srsTable, waveCfg);
            waveCfg = mapSidePanel2CfgObj(obj, waveCfg);
        end
        
        %% Side panel interaction
        function dlg = getSidePanelDialog(obj)
            % Make sure the advanced panel exists
            createSidePanel(obj, 'SRS');

            % Retrieve the side panel
            appObj = getParent(obj);
            dlg = appObj.DialogsMap(obj.classNameSRSAdv);
        end

        function updateSidePanelTitles(obj)
            % Update titles of right-side panels based on displayed SRS
            
            % Make sure the advanced panel exists
            createSidePanel(obj, 'SRS');
            
            % Update titles of each panel
            appObj = getParent(obj);
            classes = {obj.classNameSRSAdv};
            cfgIdx = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.srsTable.Selection);
            srsID = num2str(obj.srsTable.AllIDs(cfgIdx));
            for idx = 1:length(classes)
                thisClass = classes{idx};
                dialog = appObj.WaveformGenerator.pParameters.DialogsMap(thisClass);
                
                currTitle = getTitle(dialog);
                
                currID = extract(string(currTitle),digitsPattern);
                currTitle = strrep(currTitle,currID,srsID);
                dialog.setTitle(currTitle);
                if idx == 1
                    obj.xrsSingleChannelFig.Name = erase(strrep(currTitle, '(', '- '), ')');
                end
            end
        end
        
        function cfg = mapSidePanel2CfgObj(obj, cfg)
            % This method is invoked when there is a need to store the edits at
            % the right side panel and store these internally in the cache.
            
            % Make sure the advanced panel exists
            createSidePanel(obj, 'SRS');
            
            % Get cached configuration
            cfgIdx = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.srsTable.Selection);
            srsCfg = cfg{cfgIdx};
            
            % Set properties
            dlg = getSidePanelDialog(obj);
            srsCfg.Label  = dlg.Label;
            srsCfg.CyclicShift = dlg.CyclicShift;
            srsCfg.GroupSeqHopping = dlg.GroupSeqHopping;
            srsCfg.NSRSID = dlg.NSRSID;
            srsCfg.SRSPositioning = dlg.SRSPositioning;
            srsCfg.FrequencyScalingFactor = dlg.FrequencyScalingFactor;
            srsCfg.EnableStartRBHopping = dlg.EnableStartRBHopping;
            srsCfg.StartRBIndex = dlg.StartRBIndex;
            
            % Update cached configuration
            cfg{cfgIdx} = srsCfg;
        end
    end
end
