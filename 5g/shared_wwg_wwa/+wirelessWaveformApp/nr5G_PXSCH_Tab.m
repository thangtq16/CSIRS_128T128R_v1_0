classdef nr5G_PXSCH_Tab < handle
    % Dialog class that handles every graphical aspect related to PXSCH

    % Copyright 2024-2025 The MathWorks, Inc.

    properties (Abstract)
        % Properties defined in children classes
        pxschTable % Table object
    end

    properties
        % Object-specific properties
        pxschFig % Figure containing the dialog
        pxschSingleChannelFig % Figure containing the side panel
        pxschGridLayout; % Grid containing the table

        paramPXSCH % Object to host (parallel) layouts outside the main AppObj.pParameters object
    end

    properties (AbortSet)
        pxschWaveConfig % Cached PXSCH config object
    end

    properties (Constant, Abstract)
        PXSCHfigureName % Channel and figure name
        pxschExtraFigTag % Side panel figure tag
    end

    properties (SetAccess = protected, GetAccess = public, Abstract)
        % Side panel class names
        classNamePXSCHAdv
        classNamePXSCHDMRS
        classNamePXSCHPTRS
    end

    properties (Access = protected)
        DefaultConfigPXSCH % Store default wavegen config object
    end

    properties (Access = private)
        CodingDependentProperties = {'XOverhead','RVSequence', 'RVSequenceCW2', ...
            'EnableLBRM','MaxNumLayers','MCSTable'}; % Property names of the Advanced dialog that depend on Coding
    end

    properties (Access = private, Dependent)
        % Information about configuration error. If the wavegen
        % configuration for this channel represented by the app has an
        % error, ConfigError stores the MException thrown when trying to
        % update the wavegen configuration object with the app data. If
        % there is no error, ConfigError is empty.
        ConfigError
    end

    properties (Access = private, Constant)
        pCustomDataSource = [1; 0; 0; 1]; % Default custom data source
    end

    methods(Abstract)
        createTableGridLayout % Implemented in nr5G_Full_Layout
        getParent % Implemented in nr5G_XL_Dialog
    end

    methods (Access = protected, Abstract)
        % Cache --> Side panel
        dlg = mapCache2PXSCHAdv(obj, pxsch, dlg);  % Set link-specific advanced properties from the cache
        dlg = mapCache2PXSCHDMRS(obj, pxsch, dlg); % Set link-specific DM-RS properties from the cache
        dlg = mapCache2PXSCHPTRS(obj, pxsch, dlg); % Set link-specific PT-RS properties from the cache

        % Side panel --> Cache
        pxsch = mapPXSCHAdv2Cache(obj, pxsch, dlg);  % Set link-specific advanced properties from the side panel
        pxsch = mapPXSCHDMRS2Cache(obj, pxsch, dlg); % Set link-specific DM-RS properties from the side panel
        pxsch = mapPXSCHPTRS2Cache(obj, pxsch, dlg); % Set link-specific PT-RS properties from the side panel
    end

    % Constructor and public methods
    methods (Access = public)
        function obj = nr5G_PXSCH_Tab(codingDepProps)

            arguments
                % Link-specific properties whose visibility depend on
                % Coding, specified as a cell of char.
                codingDepProps (1,:) cell = {};
            end

            % Create grid layout that contains the PXSCH table
            obj.pxschGridLayout = createTableGridLayout(obj,obj.pxschFig,lower(obj.PXSCHfigureName),1);

            % Update property names of the Advanced dialog that depend on Coding
            obj.CodingDependentProperties = cat(2, obj.CodingDependentProperties, codingDepProps);
        end

        %% PXSCH Configuration
        function waveConfig = updateCachedConfigPXSCH(obj, action, changedConfigIndex)
            % Update the nrWavegenXConfig configuration object and the XL
            % cached configuration object from the table and the potential
            % side panel.

            if action=="ConfigChange"
                % If one of the current PXSCH instances has changed, update
                % the cached configuration from the table
                waveConfig = getConfiguration(obj);
            else
                % If a PXSCH instance has been added, removed, or duplicated,
                % simply update the cached configuration object as there is
                % no need to read the whole table
                waveConfig = wirelessWaveformApp.internal.Utility.updateConfigDimension(obj.pxschWaveConfig, obj.DefaultConfigPXSCH, action, changedConfigIndex, obj.pxschTable.AllIDs);
            end

            % Update the cache
            obj.pxschWaveConfig = waveConfig;
        end

        function applyConfigPXSCH(obj, waveCfg, nvargs)
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
                % 1. numel(nvargs.IDs) < numel(waveCfg.PXSCH)
                % 2. isempty(nvargs.IDs) && (numel(obj.pxschTable.AllIDs)<numel(waveCfg.PXSCH))
                nvargs.AllowUIChange (1,1) logical = true;

                % List of row IDs. If AllowUIChange is false, the value of
                % IDs is used to specify the desired IDs of the rows. If
                % IDs is empty, the existing list given by AllIDs is used.
                % In these cases AllowUIChange is considered true
                % irrespective of the user input:
                % 1. numel(nvargs.IDs) < numel(waveCfg.PXSCH)
                % 2. isempty(nvargs.IDs) && (numel(obj.pxschTable.AllIDs)<numel(waveCfg.PXSCH))
                nvargs.IDs (1,:) uint8 = [];
            end

            if isempty(obj.pxschTable)
                return; % App initialization
            end

            % Get the PXSCH configuration object from the nrXLCarrierConfig
            % object
            ch = getPXSCHCfg(obj, waveCfg);

            % Disabled default if empty. This is possible only when using
            % openInGenerator.
            if isempty(ch)
                ch = obj.DefaultConfigPXSCH;
                ch{1}.Enable = false;
            end

            % Update the cache
            obj.pxschWaveConfig = ch;

            % Map the channel configuration to the PXSCH table
            applyConfiguration(obj.pxschTable, ch, AllowUIChange=nvargs.AllowUIChange, IDs=nvargs.IDs);

            % Update the side panel
            mapCache2SidePanelPXSCH(obj);
            updateControlsVisibilityPXSCH(obj);
        end

        function out = hasConfigErrorPXSCH(obj)
            % Return the state of the configuration for this channel. The
            % output is true if the app represents an invalid wavegen
            % configuration for the specified channel, and false otherwise.
            out = hasConfigError(obj.pxschTable);
        end

        function e = getConfigErrorPXSCH(obj)
            % Return the wavegen configuration error corresponding to this
            % channel, if any. If the app represents a valid wavegen
            % configuration for this channel, the output is empty.
            e = obj.ConfigError;
        end

        %% Side panel
        function createAdvancedTabPXSCH(obj)
            % Create 2nd param object, so that there is no interference with
            % 1st one in layoutPanels()
            appObj = getParent(obj);
            obj.paramPXSCH = wirelessAppContainer.Parameters(appObj.WaveformGenerator);
            % Get the list of invisible properties, if any, passed as an
            % input to the nr5G_DL_Tabls/nr5G_UL_Tabs constructor. This is
            % saved in the properties InvisiblePDSCHEntries and
            % InvisiblePUSCHEntries, respectively.
            invisibleProps = obj.(['Invisible' obj.PXSCHfigureName 'Entries']); %#ok<NASGU>

            % Add DMRS & PTRS panels first, to ensure that the dialogs are
            % already present in the DialogsMap, to be used in Advanced Dialog
            % DMRS panel
            appObj.DialogsMap(obj.classNamePXSCHDMRS) = eval([obj.classNamePXSCHDMRS '(obj.paramPXSCH, obj.pxschSingleChannelFig, invisibleProps)']);
            dlg2 = appObj.DialogsMap(obj.classNamePXSCHDMRS);

            % PTRS panel
            appObj.DialogsMap(obj.classNamePXSCHPTRS) = eval([obj.classNamePXSCHPTRS '(obj.paramPXSCH, obj.pxschSingleChannelFig, invisibleProps)']);
            dlg3 = appObj.DialogsMap(obj.classNamePXSCHPTRS);

            % Advanced properties
            appObj.DialogsMap(obj.classNamePXSCHAdv) = eval([obj.classNamePXSCHAdv '(obj.paramPXSCH, obj.pxschSingleChannelFig, invisibleProps)']);
            dlg = appObj.DialogsMap(obj.classNamePXSCHAdv);
            obj.paramPXSCH.CurrentDialog = dlg; % needed for layoutPanels()

            % Arrange placements of all right-side PXSCH panels
            layoutUIControls(dlg);
            layoutUIControls(dlg2);
            layoutUIControls(dlg3);
            layoutPanels(dlg); % same for all
        end

        function mapCache2SidePanelPXSCH(obj)
            % This method is invoked when there is a need to update the right
            % side panel with contents corresponding to a newly selected row

            % Update side panel and get the configuration object of the
            % selected PXSCH instance from the cache
            updateSidePanelTitles(obj);
            cfgIdx = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pxschTable.Selection);
            pxschCfg = obj.pxschWaveConfig{cfgIdx};

            % Advanced properties
            dlgAdv = getSidePanelDialog(obj, "Advanced");
            dlgAdv.Label        = pxschCfg.Label;
            dlgAdv.XOverhead    = pxschCfg.XOverhead;
            dlgAdv.EnableLBRM   = pxschCfg.LimitedBufferRateMatching;
            dlgAdv.MaxNumLayers = pxschCfg.MaxNumLayers;
            dlgAdv.MCSTable     = pxschCfg.MCSTable;
            % Deal with RV sequence for each codeword
            if iscell(pxschCfg.RVSequence) % Single element cell array is not allowed by the wavegen config object
                dlgAdv.RVSequence    = pxschCfg.RVSequence{1};
                dlgAdv.RVSequenceCW2 = pxschCfg.RVSequence{2};
            else
                dlgAdv.RVSequence    = pxschCfg.RVSequence;
                dlgAdv.RVSequenceCW2 = pxschCfg.RVSequence;
            end

            % Data source is handled slightly different from programmatic API
            if ischar(pxschCfg.DataSource)
                dlgAdv.DataSource = pxschCfg.DataSource;
                dlgAdv.CustomDataSource = obj.pCustomDataSource;
            else
                dlgAdv.DataSource = 'User-defined';
                dlgAdv.CustomDataSource = pxschCfg.DataSource;
            end
            % Set PDSCH/PUSCH-specific advanced properties
            mapCache2PXSCHAdv(obj, pxschCfg, dlgAdv);

            % DM-RS panel
            dlgDMRS = getSidePanelDialog(obj, "DMRS");
            dlgDMRS.Power = pxschCfg.DMRSPower;
            dlgDMRS.DMRSConfigurationType    = pxschCfg.DMRS.DMRSConfigurationType;
            dlgDMRS.DMRSTypeAPosition        = pxschCfg.DMRS.DMRSTypeAPosition;
            dlgDMRS.DMRSAdditionalPosition   = pxschCfg.DMRS.DMRSAdditionalPosition;
            dlgDMRS.DMRSLength               = pxschCfg.DMRS.DMRSLength;
            dlgDMRS.CustomSymbolSet          = pxschCfg.DMRS.CustomSymbolSet;
            dlgDMRS.DMRSPortSet              = pxschCfg.DMRS.DMRSPortSet;
            dlgDMRS.NIDNSCID                 = pxschCfg.DMRS.NIDNSCID;
            dlgDMRS.NSCID                    = pxschCfg.DMRS.NSCID;
            dlgDMRS.NumCDMGroupsWithoutData  = pxschCfg.DMRS.NumCDMGroupsWithoutData;
            dlgDMRS.DMRSEnhancedR18          = pxschCfg.DMRS.DMRSEnhancedR18;
            % Set PDSCH/PUSCH-specific DM-RS properties
            mapCache2PXSCHDMRS(obj, pxschCfg, dlgDMRS);

            % PT-RS panel
            dlgPTRS = getSidePanelDialog(obj, "PTRS");
            dlgPTRS.Power            = pxschCfg.PTRSPower;
            dlgPTRS.EnablePTRS       = pxschCfg.EnablePTRS;
            dlgPTRS.TimeDensity      = pxschCfg.PTRS.TimeDensity;
            dlgPTRS.FrequencyDensity = pxschCfg.PTRS.FrequencyDensity;
            dlgPTRS.REOffset         = pxschCfg.PTRS.REOffset;
            dlgPTRS.PTRSPortSet      = pxschCfg.PTRS.PTRSPortSet;
            % Set PDSCH/PUSCH-specific PT-RS properties
            mapCache2PXSCHPTRS(obj, pxschCfg, dlgPTRS);

            % Disable editability of the side panel, if the corresponding
            % channel instance in the table is non-editable
            isEditable = obj.pxschTable.RowEditable(cfgIdx);
            set(dlgAdv.Layout.Children,  'Enable', uiservices.logicalToOnOff(isEditable));
            set(dlgDMRS.Layout.Children, 'Enable', uiservices.logicalToOnOff(isEditable));
            set(dlgPTRS.Layout.Children, 'Enable', uiservices.logicalToOnOff(isEditable));
        end

        function id = getAdvancedPXSCHID(obj)
            % Get the ID of the PXSCH that is currently displayed at the
            % side panel

            % Look at the number in the title:
            if isKey(obj.getParent.DialogsMap, obj.classNamePXSCHAdv) && isgraphics(obj.getParent.DialogsMap(obj.classNamePXSCHAdv).getPanels)
                dlg = getSidePanelDialog(obj, "Advanced");
                str = getTitle(dlg);
                id = double(extract(string(str),digitsPattern));
            else
                id = NaN;
            end
        end

        % Public getters for the private properties that represents the
        % side panel class names
        function className = getClassNamePXSCHAdv(obj)
            className = obj.classNamePXSCHAdv;
        end
        function className = getClassNamePXSCHDMRS(obj)
            className = obj.classNamePXSCHDMRS;
        end
        function className = getClassNamePXSCHPTRS(obj)
            className = obj.classNamePXSCHPTRS;
        end

        %% Update visibility of dependent parameters in the side panel
        function updateControlsVisibilityPXSCH(obj, needRepaint)
            % Update the visibility of all dependent parameters in the side
            % panel.
            if nargin<2
                needRepaint = false;
            end

            dlgAdv = getSidePanelDialog(obj, "Advanced");
            needRepaint = updateControlsVisibility(dlgAdv) || needRepaint;
            dlgDMRS = getSidePanelDialog(obj, "DMRS");
            needRepaint = updateControlsVisibility(dlgDMRS) || needRepaint;
            dlgPTRS = getSidePanelDialog(obj, "PTRS");
            needRepaint = updateControlsVisibility(dlgPTRS) || needRepaint;
            needRepaint = updateCodingDependentVisibility(obj) || needRepaint;
            needRepaint = updatePTRSEnable(obj) || needRepaint;

            % Update the visibility, if needed
            if needRepaint
                layoutUIControls(dlgAdv);
                layoutUIControls(dlgDMRS);
                layoutUIControls(dlgPTRS);
            end
        end
    end

    methods
        function e = get.ConfigError(obj)
            % Retrieve configuration error from the table, as the side
            % panel does not hold any configuration that is invalid at set
            % time
            e = getConfigError(obj.pxschTable);
        end
    end

    methods (Access = private)
        %% Configuration
        function waveCfg = getConfiguration(obj)
            % Get the nrWavegenXConfig configuration object from the table
            % and the potential side panel.

            % Update cache with latest edits
            waveCfg = obj.pxschWaveConfig;
            cfgIdx = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pxschTable.Selection);
            ptrsEnabledOld = waveCfg{cfgIdx}.EnablePTRS;
            waveCfg = updateConfiguration(obj.pxschTable, waveCfg);
            ptrsEnabledNew = waveCfg{cfgIdx}.EnablePTRS;
            if xor(ptrsEnabledOld, ptrsEnabledNew)
                % PT-RS was enabled/disabled in the table. Update the side
                % panel before mapping it to the cache
                updatePTRSEnable(obj, ptrsEnabledNew);
            end
            waveCfg = mapSidePanel2CfgObj(obj, waveCfg);
        end

        function ch = getPXSCHCfg(obj, cfg)
            % Get the PXSCH configuration object from the nrXLCarrierConfig
            % object and force it to be a row vector

            ch = cfg.(obj.PXSCHfigureName)(:)';
        end

        %% Side panel interaction
        function dlg = getSidePanelDialog(obj, dialogName)
            % Make sure the advanced panel exists
            createSidePanel(obj, obj.PXSCHfigureName);

            % Retrieve the side panel
            appObj = getParent(obj);
            switch dialogName
                case "Advanced"
                    className = obj.classNamePXSCHAdv;
                case "DMRS"
                    className = obj.classNamePXSCHDMRS;
                case "PTRS"
                    className = obj.classNamePXSCHPTRS;
            end
            dlg = appObj.DialogsMap(className);
        end

        function updateSidePanelTitles(obj)
            % Update titles of right-side panels based on displayed PXSCH

            % Make sure the advanced panel exists
            createSidePanel(obj, obj.PXSCHfigureName);

            % Update titles of each panel
            appObj = getParent(obj);
            classes = {obj.classNamePXSCHAdv, obj.classNamePXSCHDMRS, obj.classNamePXSCHPTRS};
            cfgIdx = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pxschTable.Selection);
            pxschID = num2str(obj.pxschTable.AllIDs(cfgIdx));
            for idx = 1:length(classes)
                thisClass = classes{idx};
                dialog = appObj.WaveformGenerator.pParameters.DialogsMap(thisClass);

                currTitle = getTitle(dialog);

                currID = extract(string(currTitle),digitsPattern);
                currTitle = strrep(currTitle,currID,pxschID);
                dialog.setTitle(currTitle);
                if idx == 1
                    obj.pxschSingleChannelFig.Name = erase(strrep(currTitle, '(', '- '), ')');
                end
            end
        end

        function cfg = mapSidePanel2CfgObj(obj, cfg)
            % This method is invoked when there is a need to store the edits at
            % the right side panel and store these internally in the cache.

            % Make sure the advanced panel exists
            createSidePanel(obj, obj.PXSCHfigureName);

            % Get cached configuration
            cfgIdx = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pxschTable.Selection);
            pxschCfg = cfg{cfgIdx};

            % Advanced panel
            dlgAdv = getSidePanelDialog(obj, "Advanced");
            pxschCfg.Label          = dlgAdv.Label;
            pxschCfg.XOverhead      = dlgAdv.XOverhead;
            pxschCfg.LimitedBufferRateMatching = dlgAdv.EnableLBRM;
            pxschCfg.MaxNumLayers              = dlgAdv.MaxNumLayers;
            pxschCfg.MCSTable                  = dlgAdv.MCSTable;
            if pxschCfg.NumCodewords == 2
                pxschCfg.RVSequence = {dlgAdv.RVSequence dlgAdv.RVSequenceCW2};
            else
                pxschCfg.RVSequence = dlgAdv.RVSequence;
            end
            % Data source is handled slightly different from programmatic API
            if strcmpi(dlgAdv.DataSource, 'User-defined')
                pxschCfg.DataSource = dlgAdv.CustomDataSource;
            else
                pxschCfg.DataSource = dlgAdv.DataSource;
            end
            % Set PDSCH/PUSCH-specific advanced properties
            pxschCfg = mapPXSCHAdv2Cache(obj, pxschCfg, dlgAdv);

            % DM-RS panel
            dlgDMRS = getSidePanelDialog(obj, "DMRS");
            pxschCfg.DMRSPower                     = dlgDMRS.Power;
            pxschCfg.DMRS.DMRSTypeAPosition        = dlgDMRS.DMRSTypeAPosition;
            pxschCfg.DMRS.DMRSAdditionalPosition   = dlgDMRS.DMRSAdditionalPosition;
            pxschCfg.DMRS.DMRSLength               = dlgDMRS.DMRSLength;
            pxschCfg.DMRS.CustomSymbolSet          = dlgDMRS.CustomSymbolSet;
            pxschCfg.DMRS.DMRSPortSet              = dlgDMRS.DMRSPortSet;
            pxschCfg.DMRS.NIDNSCID                 = dlgDMRS.NIDNSCID;
            pxschCfg.DMRS.NSCID                    = dlgDMRS.NSCID;
            pxschCfg.DMRS.DMRSEnhancedR18          = dlgDMRS.DMRSEnhancedR18;
            % Set PDSCH/PUSCH-specific DM-RS properties
            pxschCfg = mapPXSCHDMRS2Cache(obj, pxschCfg, dlgDMRS);

            % PT-RS panel
            dlgPTRS = getSidePanelDialog(obj, "PTRS");
            pxschCfg.PTRSPower             = dlgPTRS.Power;
            pxschCfg.EnablePTRS            = dlgPTRS.EnablePTRS;
            pxschCfg.PTRS.TimeDensity      = dlgPTRS.TimeDensity;
            pxschCfg.PTRS.FrequencyDensity = dlgPTRS.FrequencyDensity;
            pxschCfg.PTRS.REOffset         = dlgPTRS.REOffset;
            pxschCfg.PTRS.PTRSPortSet      = dlgPTRS.PTRSPortSet;
            % Set PDSCH/PUSCH-specific PT-RS properties
            pxschCfg = mapPXSCHPTRS2Cache(obj, pxschCfg, dlgPTRS);

            % Update cached configuration
            cfg{cfgIdx} = pxschCfg;
        end

        %% Dependent controls visibility
        function needRepaint = updateCodingDependentVisibility(obj)
            % TB Scaling, XOverhead and LBRM properties in advanced only
            % applicable when Coding = true.

            % Adjust visuals only if current row is displayed in
            % "advanced", as the user can toggle the Coding checkbox
            % without actually selecting the row
            currPXSCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pxschTable.Selection);
            if obj.pxschTable.AllIDs(currPXSCH) == getAdvancedPXSCHID(obj)

                % Get Coding value
                coding = obj.pxschWaveConfig{currPXSCH}.Coding;

                % Get Advanced property panel
                appObj = getParent(obj);
                dlg = appObj.DialogsMap(obj.classNamePXSCHAdv);
                oldVis = isVisible(dlg, 'XOverhead');

                needRepaint = xor(oldVis, coding);
                if needRepaint
                    % do re-layout only when it is needed, because it is expensive
                    setVisible(dlg, obj.CodingDependentProperties, coding);
                    if coding
                        % When Coding = true, visibility of MaxNumLayers
                        % and MCSTable is dependent on EnableLBRM
                        enableLBRMChanged(dlg);
                        % When Coding = true, visibility of RVSequenceCW2
                        % depends on number of layers
                        updateRVSequenceCW2Visibility(dlg);
                    end
                end
            else
                needRepaint = false;
            end
        end

        function needRepaint = updatePTRSEnable(obj, varargin)
            % PTRS Enable must be synchronized between table and advanced properties

            needRepaint = false;

            % Adjust visuals only if current row is displayed in
            % "advanced", as the user can toggle the Enable PT-RS checkbox
            % without actually selecting the row
            currPXSCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pxschTable.Selection);
            if obj.pxschTable.AllIDs(currPXSCH) == getAdvancedPXSCHID(obj)
                if nargin==2
                    enable = varargin{1};
                else
                    enable = obj.pxschWaveConfig{currPXSCH}.EnablePTRS;
                end

                dlgPTRS = getSidePanelDialog(obj, "PTRS");
                if xor(dlgPTRS.EnablePTRS, enable)
                    % do re-layout only when it is needed, because it is expensive
                    dlgPTRS.EnablePTRS = enable;
                    needRepaint = enabledChanged(dlgPTRS);
                end

                if needRepaint && (nargin==2 && enable)
                    % Update the PT-RS visibility
                    layoutUIControls(dlgPTRS);
                end
            end
        end
    end

    methods(Access = {?wirelessWaveformGenerator.nr5G_SSB_DataSource})
        function amendCachedConfigPXSCH(obj, chWaveCfg, cfgIndex)
            % Update the cached PXSCH wave config object with the input
            % CHWAVECFG object.

            arguments
                obj
                chWaveCfg (:,1) cell
                cfgIndex (:,1) uint8
            end

            % Ensure the PXSCH cache is updated with the new wavegen
            % configuration object
            if numel(obj.pxschWaveConfig)~=numel(obj.pxschTable.AllIDs)
                % A new configuration object needs to be appended
                obj.pxschWaveConfig = cat(1, obj.pxschWaveConfig, chWaveCfg);
            else
                % Update the relevant PXSCH config objects
                obj.pxschWaveConfig(cfgIndex) = chWaveCfg;
            end

            % Ensure the side panel is promptly updated as well
            mapCache2SidePanelPXSCH(obj);
        end
    end
end
