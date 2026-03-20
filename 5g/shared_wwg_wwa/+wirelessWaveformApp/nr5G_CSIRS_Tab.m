classdef nr5G_CSIRS_Tab < handle
    % Dialog class that handles every graphical aspect related to CSI-RS
    
    % Copyright 2023-2024 The MathWorks, Inc.
    
    properties
        % Object-specific properties
        csirsFig % Figure containing the dialog
        csirsGridLayout; % Grid containing the table
        csirsTable % Table object
    end
    
    properties (AbortSet)
        csirsWaveConfig % Cached config object
    end
    
    properties (Constant)
        XRSfigureName    = 'CSIRS'; % Figure name
        xrsExtraFigTag   = 'xrsSingleChannelFig'; % Side panel figure tag
    end
    
    properties (Access = private)
        DefaultConfigCSIRS % Store default wavegen config object
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
    end
    
    % Constructor and public methods
    methods (Access = public)
        function obj = nr5G_CSIRS_Tab(defaultWaveConfig, invisibleEntries)
            % Create grid layout that contains the table
            obj.csirsGridLayout = createTableGridLayout(obj,obj.csirsFig,'csirs',1);
            
            % Construct the table object
            defaultConfigCSIRS = defaultWaveConfig.CSIRS;
            obj.csirsTable = wirelessWaveformApp.nr5G_CSIRS_Table(obj.csirsGridLayout, defaultConfigCSIRS, invisibleEntries);
            
            % Initialize the cached configuration object
            obj.csirsWaveConfig = defaultConfigCSIRS;
            obj.DefaultConfigCSIRS = defaultConfigCSIRS;
        end
        
        %% CSI-RS Configuration
        function waveConfig = updateCachedConfigCSIRS(obj, action, changedConfigIndex)
            % Update the nrWavegenXConfig configuration object and the DL
            % cached configuration object from the table and the potential
            % side panel.
            
            if action=="ConfigChange"
                % If one of the current CSI-RS instances has changed,
                % update the cached configuration from the table
                waveConfig = getConfiguration(obj);
            else
                % If a CSI-RS instance has been added, removed, or
                % duplicated, simply update the cached configuration object
                % as there is no need to read the whole table
                waveConfig = wirelessWaveformApp.internal.Utility.updateConfigDimension(obj.csirsWaveConfig, obj.DefaultConfigCSIRS, action, changedConfigIndex, obj.csirsTable.AllIDs);
            end
            
            % Update the cache
            obj.csirsWaveConfig = waveConfig;
        end
        
        function applyConfigCSIRS(obj, chWaveCfg)
            % Apply configuration when starting a new session, loading an
            % existing session, or using openInGenerator
            
            if isempty(obj.csirsTable)
                return; % App initialization
            end
            
            % Force the input configuration to be a row vector
            ch = chWaveCfg.CSIRS(:)';
            
            % Preprocess the input configuration
            if isempty(ch)
                % Disabled default if empty. This is possible only when
                % using openInGenerator.
                ch = obj.DefaultConfigCSIRS;
                ch{1}.Enable = false;
            else
                % Convert to multiple objects if cell arrays are used for
                % multiple CSI-RS. This is possible only when using
                % openInGenerator.
                ch = cell2Objects(obj, ch);
            end
            
            % Update the cache
            obj.csirsWaveConfig = ch;
            
            % Map the channel configuration to the table
            applyConfiguration(obj.csirsTable, ch);
            
            % Update the side panel
            mapCache2SidePanelCSIRS(obj);
            updateControlsVisibilityCSIRS(obj);
        end

        function out = hasConfigErrorCSIRS(obj)
            % Return the state of the configuration for this channel. The
            % output is true if the app represents an invalid wavegen
            % configuration for the specified channel, and false otherwise.
            out = hasConfigError(obj.csirsTable);
        end

        function e = getConfigErrorCSIRS(obj)
            % Return the wavegen configuration error corresponding to this
            % channel, if any. If the app represents a valid wavegen
            % configuration for this channel, the output is empty.
            e = obj.ConfigError;
        end
        
        %% Side panel
        function mapCache2SidePanelCSIRS(~)
            % This method is invoked when there is a need to update the right
            % side panel with contents corresponding to a newly selected row
            
            % No side panel in CSI-RS, so this method is a no-op
        end
        
        function updateControlsVisibilityCSIRS(~)
            % Update the visibility of all dependent parameters in the side
            % panel.
            
            % No side panel in CSI-RS, so this method is a no-op
        end
    end

    methods
        function e = get.ConfigError(obj)
            % Retrieve configuration error from the table
            e = getConfigError(obj.csirsTable);
        end
    end

    methods (Access = private)
        %% Configuration
        function waveCfg = getConfiguration(obj)
            % Get the nrWavegenXConfig configuration object from the table
            % and the potential side panel.
            
            % Update cache with latest edits
            waveCfg = obj.csirsWaveConfig;
            waveCfg = updateConfiguration(obj.csirsTable, waveCfg);
            waveCfg = mapSidePanel2CfgObj(obj, waveCfg);
        end
        
        function chWaveCfgOut = cell2Objects(obj, chWaveCfgIn)
            % Expand CSI-RS input configuration csirsIn, which contains
            % cell arrays to define multiple CSI-RS configurations, into
            % multiple independent CSI-RS objects.
            
            csirsDefault = obj.DefaultConfigCSIRS{1};
            props = getPublicSettableProperties(csirsDefault);
            
            chWaveCfgOut = [];
            for c = 1:numel(chWaveCfgIn) % Iterate over all input nrWavegenCSIRSConfig objects
                
                % Number of CSI-RS configs defined using cell in this object
                numCSIRSCell = numel(chWaveCfgIn{c}.RowNumber);
                
                % Create as many nrWavegenCSIRSConfig as number of cells in this object
                csirs = repmat(csirsDefault,1,numCSIRSCell);
                
                % For each property, copy its contents into all new
                % nrWavegenCSIRSConfig objects
                for pidx = 1:length(props)
                    val = chWaveCfgIn{c}.(props{pidx});
                    if strcmpi(props{pidx},'RowNumber')
                        val = num2cell(val);
                    elseif ~iscell(val)
                        val = {val};
                    end
                    [csirs.(props{pidx})] = deal(val{:});
                end
                chWaveCfgOut = [chWaveCfgOut,csirs]; %#ok<AGROW>
            end
            
            chWaveCfgOut = num2cell(chWaveCfgOut);
        end
        
        %% Side panel
        function cfg = mapSidePanel2CfgObj(~, cfg, varargin)
            % This method is invoked when there is a need to store the edits at
            % the right side panel and store these internally in the cache.
            
            % No side panel in CSI-RS, so this method is a no-op
        end
    end
end

% Get the public properties of nrWavegenCSIRSConfig class, as read-only
% properties are not settable
function publicProps = getPublicSettableProperties(config)

    mc = metaclass(config);
    props = mc.PropertyList;
    allProps = {props.Name};
    fcn = @(prop) strcmp(prop.GetAccess,'public') && strcmp(prop.SetAccess,'public') && ~prop.Hidden;
    publicInd = arrayfun(fcn,props);
    publicProps = allProps(publicInd);

end
