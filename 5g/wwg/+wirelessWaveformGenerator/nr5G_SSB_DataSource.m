classdef nr5G_SSB_DataSource < wirelessWaveformApp.nr5G_Dialog
    % Panel offering data-source configuration for SSB tab

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Access = private)
        % SIB1 channel row ID tracking
        SIB1PDSCHID
        SIB1PDCCHID
        SIB1COREID = 0 % CORESET0. Won't change.
        SIB1SSID
        SIB1BWPID

        % SIB1 local cached configuration
        SIB1Config

        SIB1Valid = false;
        SIB1Error
    end

    properties (SetAccess = private)
        % SIB1 channel row index tracking
        SIB1PDSCHIndex
        SIB1PDCCHIndex
        SIB1COREIndex = 1; % CORESET0 is always the first row
        SIB1SSIndex
        SIB1BWPIndex
    end

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
        Payload
    end

    properties (Dependent, SetObservable)
        SSBDMRSTypeAPosition
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Hidden)
        TitleString = getString(message('nr5g:waveformGeneratorApp:SSDataSourceTitle'))

        DataSourceType = 'charPopup'
        DataSourceDropDown = {'MIB', 'PN9-ITU', 'PN9', 'PN11', 'PN15', 'PN23', getUserDefinedSourceString()}
        DataSourceGUI
        DataSourceLabel

        SSBDMRSTypeAPositionType = 'numericPopup'
        SSBDMRSTypeAPositionDropDown = {'2', '3'}
        SSBDMRSTypeAPositionGUI
        SSBDMRSTypeAPositionLabel

        CellBarredType = 'checkbox'
        CellBarredGUI
        CellBarredLabel

        IntraFreqReselectionType = 'checkbox'
        IntraFreqReselectionGUI
        IntraFreqReselectionLabel

        PDCCHConfigSIB1Type = 'numericEdit'
        PDCCHConfigSIB1GUI
        PDCCHConfigSIB1Label

        PayloadType = 'charEdit'
        PayloadGUI
        PayloadLabel

        Sib1ConfigIndicesType = 'numericText'
        Sib1ConfigIndicesGUI
        Sib1ConfigIndicesLabel

        Sib1CheckType = 'checkbox'
        Sib1CheckGUI
        Sib1CheckLabel
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_SSB_DataSource(parent, fig)
            obj@wirelessWaveformApp.nr5G_Dialog(parent, fig, wirelessWaveformApp.nr5G_SSB_Dialog.DefaultCfg); % call base constructor

            % Callbacks needed either for validation of edit fields or for
            % changes to visibility of other controls
            obj.DataSourceGUI.(obj.Callback)           = @(src,evnt) dataSourceChangedGUI(obj, []);
            obj.PDCCHConfigSIB1GUI.(obj.Callback)      = @(src,evnt) configChangedGUI(obj,src,evnt);
            obj.PayloadGUI.(obj.Callback)              = @(src,evnt) updateProperty(obj,src,evnt,"DataSource",FieldNames="Payload");
            obj.Sib1CheckGUI.(obj.Callback)            = @(src,evnt) sib1ChangedGUI(obj);
            obj.SSBDMRSTypeAPositionGUI.(obj.Callback) = @(src,evnt) ssbDMRSTypeAPositionChangedGUI(obj);

            addlistener(obj,'SSBDMRSTypeAPosition','PostSet',@(src,event)obj.ssbDMRSTypeAPositionChangedObj(src,event));

            % This is not visible by default
            setVisible(obj, 'Payload', false);

            if ~(ispc || ismac) % Linux
                obj.ValueWidth = 140; % make sure all checkboxes fit
            end
        end

        function updateControlsVisibility(obj)
            dataSourceChangedGUI(obj);
        end

        function adjustSpec(obj)
            % This is needed so that the SSB panel(s) do not horizontally fill the entire App
            obj.panelFixedSize = true;
        end

        function adjustDialog(obj)
            % make sure all tags are unique; the same controls can be present in PDSCH
            obj.DataSourceGUI.Tag            = 'SSBDataSource';
            obj.SSBDMRSTypeAPositionGUI.Tag  = 'SSBDMRSTypeAPosition';
        end

        function props = displayOrder(~)
            props = {'DataSource'; 'SSBDMRSTypeAPosition'; 'CellBarred'; 'IntraFreqReselection'; ...
                'PDCCHConfigSIB1'; 'Sib1ConfigIndices';'Sib1Check'; 'Payload'};
        end

        function restoreDefaults(obj)
            c = wirelessWaveformApp.nr5G_SSB_Dialog.DefaultCfg;  % Get defaults from nrWavegenSSBurstConfig

            % Data source is handled slightly different from programmatic API
            if ischar(c.DataSource)
                obj.DataSource = c.DataSource;
                obj.Payload = zeros(1, 24);
            else
                obj.DataSource = getUserDefinedSourceString();
                obj.Payload = c.DataSource;
            end

            obj.SSBDMRSTypeAPosition    = c.DMRSTypeAPosition;
            obj.CellBarred              = c.CellBarred;
            obj.IntraFreqReselection    = c.IntraFreqReselection;
            obj.PDCCHConfigSIB1         = c.PDCCHConfigSIB1;
            obj.Sib1ConfigIndices       = [num2str(floor(obj.PDCCHConfigSIB1/16)),',',num2str(mod(obj.PDCCHConfigSIB1,16))];
            obj.Sib1Check               = 0;
            obj.SIB1PDSCHID             = [];
            obj.SIB1PDCCHID             = [];
            obj.SIB1SSID                = [];
            obj.SIB1BWPID               = [];
        end

        function str = getCatalogPrefix(~)
            str = 'nr5g:waveformGeneratorApp:';
        end
    end

    % Methods specific to this class and that are currently used by
    % neighboring classes
    methods (Access = public)
        function ssb = addSSBDataSourceConfig(obj, ssb)
            % Append the contents of this panel to an nrWavegenSSBurstConfig configuration

            % Data source is handled slightly different from programmatic API
            if ~strcmp(obj.DataSource, getUserDefinedSourceString())
                ssb.DataSource = obj.DataSource;
            else
                ssb.DataSource = obj.Payload;
            end

            ssb.DMRSTypeAPosition       = obj.SSBDMRSTypeAPosition;
            ssb.CellBarred              = obj.CellBarred;
            ssb.IntraFreqReselection    = obj.IntraFreqReselection;
            ssb.PDCCHConfigSIB1         = obj.PDCCHConfigSIB1;

            if  (~obj.SIB1Valid && obj.Sib1Check)
                rethrow(obj.SIB1Error);
            end
        end

        function dataSourceChangedGUI(obj, ~)
            % Determine the visibility of all items in this panel based on DataSource type

            mib     = strcmp(obj.DataSource, 'MIB');
            setVisible(obj, {'SSBDMRSTypeAPosition', 'CellBarred', 'IntraFreqReselection', 'PDCCHConfigSIB1','Sib1ConfigIndices'}, mib);

            if ~mib
                if obj.Sib1Check
                    % If Payload changed while SIB1 was enabled
                    % Disabled SIB1 and delete SIB1 Channels
                    obj.Sib1Check=0;
                    sib1Delete(obj);
                    updateGrid(obj.CurrentDialog);
                end
            end

            % Set visibility of SubcarrierSpacingCommon
            className = 'wirelessWaveformApp.nr5G_SSB_Dialog';
            dlg = obj.Parent.WaveformGenerator.pParameters.DialogsMap(className);
            updateControlsVisibility(dlg);

            % Set visibility of Payload
            payload = strcmp(obj.DataSource, getUserDefinedSourceString());
            if xor(payload,obj.PayloadGUI.Visible)
                % Do this only if the visibility of payload has changed
                setVisible(obj, 'Payload', payload);
                layoutUIControls(obj);
                % Check if there is any problem with the payload
                updateAppConfigDiagnostic(obj, []);
            end
        end

        function sib1ChangedGUI(obj)
            % This function is ran the first time SIB1 is generated
            dlg = obj.CurrentDialog;
            if obj.Sib1Check

                if (obj.PDCCHConfigSIB1 == 0)
                    obj.PDCCHConfigSIB1 = 4;                                                %Give PDCCHConfig-SIB1 a "nice" default value if it's being used "blindly" by user
                    obj.Sib1ConfigIndices = '0, 4';
                end

                updateSIB1Config(obj, UpdateGrid=false, Init=true);                                    %Function to create all the SIB1 channels (BWP/CORESET/SS/PDCCH/PDSCH)
                obj.Sib1Check = obj.SIB1Valid;

            else
                sib1Delete(obj);
            end

            % Update the grid visualization without clearing message window
            try
                wgc = getConfiguration(dlg);
                resetStatus = false;
                updateGrid(dlg, wgc, resetStatus);
            catch e
                % Update message
                updateAppConfigDiagnostic(dlg,e);
            end

            % Toggle visibility of Custom Frequency Offset based on whether
            % SIB1 is enabled or not.
            className = 'wirelessWaveformApp.nr5G_SSB_Dialog';
            dlgSSB = obj.Parent.WaveformGenerator.pParameters.DialogsMap(className);
            setVisible(dlgSSB, 'FrequencyOffset', ~obj.Sib1Check);
            updateControlsVisibility(dlgSSB)

            % If the non-SIB1 configuration was invalid, ensure that the
            % the potential table conflicts are properly updated
            tableConfigErrors = arrayfun(@(x)hasConfigError(dlg, lower(erase(x, "Table"))), dlg.tableObjName);
            if obj.Sib1Check && any(tableConfigErrors)
                % Use the cached nrDLCarrierConfig object to ensure a valid configuration is used
                wgc = dlg.cachedCfg;
                wgc.BandwidthParts(obj.SIB1BWPIndex) = obj.SIB1Config.BandwidthParts;
                wgc.CORESET(obj.SIB1COREIndex) = obj.SIB1Config.CORESET;
                wgc.SearchSpaces(obj.SIB1SSIndex) = obj.SIB1Config.SearchSpaces;
                wgc.PDCCH(obj.SIB1PDCCHIndex) = obj.SIB1Config.PDCCH;
                wgc.PDSCH(obj.SIB1PDSCHIndex) = obj.SIB1Config.PDSCH;

                % Call updateGrid() passing the wavegen config object as
                % the second optional input to skip validation on the
                % configuration
                resetStatus = false;
                updateGrid(dlg, wgc, resetStatus);
            end

            unfreezeApp(dlg.getParent.AppObj);
        end

        function sib1Delete(obj, figclearFlag)
            if nargin == 1
                figclearFlag = 1;
            end
            dlg = obj.CurrentDialog;

            %Identify the current indices of SIB1 tables
            obj.SIB1BWPIndex = find(dlg.bwpTable.AllIDs==obj.SIB1BWPID);

            % If SIB1BWPIndex is empty, then there is no SIB1 channels to delete!
            if ~isempty(obj.SIB1BWPIndex)
                for Sib1IDidx=1:numel(obj.SIB1SSID)
                    obj.SIB1SSIndex(Sib1IDidx) = find(dlg.searchSpacesTable.AllIDs==obj.SIB1SSID(Sib1IDidx));
                    obj.SIB1PDCCHIndex(Sib1IDidx) = find(dlg.pdcchTable.AllIDs==obj.SIB1PDCCHID(Sib1IDidx));
                    obj.SIB1PDSCHIndex(Sib1IDidx) = find(dlg.pxschTable.AllIDs==obj.SIB1PDSCHID(Sib1IDidx));
                end

                % Remove the figure corresponding to the deleted SIB1-BWP
                if figclearFlag
                    updateBWPFigure(dlg, "Remove", obj.SIB1BWPIndex);
                end

                % If the non-SIB1 configuration was invalid, ensure that the
                % the potential conflicts shown in the non-SIB1 BWP figures are
                % properly updated
                if any(arrayfun(@(x)hasConfigError(dlg, lower(erase(x, "Table"))), dlg.tableObjName))
                    % Use the cached nrDLCarrierConfig object to ensure a valid configuration is used
                    wgc = dlg.cachedCfg;
                    wgc.BandwidthParts(obj.SIB1BWPIndex) = [];
                    wgc.SearchSpaces(obj.SIB1SSIndex) = [];
                    wgc.PDCCH(obj.SIB1PDCCHIndex) = [];
                    wgc.PDSCH(obj.SIB1PDSCHIndex) = [];

                    % Call updateGrid() passing the wavegen config object as
                    % the second optional input to skip validation on the
                    % configuration
                    resetStatus = false;
                    updateGrid(dlg, wgc, resetStatus);
                end

                % Disable visualization to avoid unnecessary intermediate grid updates
                enableVizUpdates(obj, false);

                %Set rows editable ie not read-only
                updateRowEditabilitySIB1(obj,true);

                % Create a configuration with disabled PDCCH and PDSCH, will be
                % used for any single entry tables that are being removed.
                defaultConfig = wirelessWaveformApp.nr5G_DL_Tabs.DefaultCfg;
                defaultConfig.PDCCH{1}.Enable = 0;
                defaultConfig.PDSCH{1}.Enable = 0;

                % Delete in reverse order to avoid broken links
                % Delete PDSCH
                needCacheAmended = sib1TableUnload(obj,'pxschTable',defaultConfig.PDSCH,obj.SIB1PDSCHID);
                if needCacheAmended
                    amendCachedConfigPXSCH(dlg, defaultConfig.PDSCH, 1);
                end

                % Delete PDCCH
                needCacheAmended = sib1TableUnload(obj,'pdcchTable',defaultConfig.PDCCH,obj.SIB1PDCCHID);
                if needCacheAmended
                    amendCachedConfigPDCCH(dlg, defaultConfig.PDCCH, 1);
                end

                % Delete Searchspace
                needCacheAmended = sib1TableUnload(obj,'searchSpacesTable',defaultConfig.SearchSpaces,obj.SIB1SSID);
                if needCacheAmended
                    amendCachedConfigSearchSpaces(dlg, defaultConfig.SearchSpaces, 1);
                end

                %CORESET0 is not being deleted, but the default value is
                %restored
                if numel(dlg.coresetTable.AllIDs) > 1
                    defaultCORESET = defaultConfig.CORESET(1);
                    applyConfiguration(dlg.coresetTable, defaultCORESET, AllowUIChange=false, ConfigIDs=0);
                    updateCachedConfigCORESET(dlg, "ConfigChange", 1);
                else
                    defaultCORESET = defaultConfig.CORESET;
                    applyConfiguration(dlg.coresetTable, defaultCORESET);
                    % Update CORESET cached configuration
                    updateCachedConfigCORESET(dlg, "Add", 2);
                    amendCachedConfigCORESET(dlg, defaultCORESET, 1:numel(defaultCORESET));
                end
                %Remove BWP from config and apply
                % Do this first as it triggers the update in the visualization
                % that is probably one of the most expensive actions here
                if numel(dlg.bwpTable.AllIDs) > 1
                    removeConfiguration(dlg.bwpTable, obj.SIB1BWPID);
                else
                    % Only SIB1 exists if deleting. Re-instate the full-band BWP to link to
                    % the same SCS carrier as the current SIB1 one
                    cfg = dlg.cachedCfg;
                    scsIndex = cellfun(@(x)x.SubcarrierSpacing==cfg.BandwidthParts{1}.SubcarrierSpacing, cfg.SCSCarriers);
                    cfg.BandwidthParts{1}.NSizeBWP = cfg.SCSCarriers{scsIndex}.NSizeGrid;
                    cfg.BandwidthParts{1}.NStartBWP = cfg.SCSCarriers{scsIndex}.NStartGrid;
                    cfg.BandwidthParts{1}.BandwidthPartID = 1;
                    cfg.BandwidthParts{1}.Label = 'BWP1';
                    enableVizUpdates(obj, true); % Re-enable it to ensure the new figure for SIB1-BWP is displayed
                    appendConfiguration(dlg.bwpTable, cfg.BandwidthParts);
                    removeConfiguration(dlg.bwpTable, obj.SIB1BWPID);
                    enableVizUpdates(obj, false); % Re-disable the visualizations
                    % Update BWP cached configuration
                    amendCachedConfigBWP(dlg, cfg.BandwidthParts, 1);
                end

                % Reset the values of all SIB1 inidices to the default
                resetSIB1Indices(obj);

                sib1DlgName = 'wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog';
                sib1Dlg = obj.Parent.WaveformGenerator.pParameters.DialogsMap(sib1DlgName);
                collapseSIB1Panel(sib1Dlg,true);

                % Re-enable visualization
                enableVizUpdates(obj, true);

                % Clear valid properties
                obj.SIB1Valid = false;
                obj.SIB1Error = [];
            end
        end

        function updateSIB1Config(obj, nvargs)
            % This function gets called whenever a SIB1 driving parameter
            % is changed in the app. It takes care of applying configs to
            % various parts of the app and writing only to the correct table
            % row indices.

            arguments
                obj
                % UpdateGrid is a logical flag that controls whether
                % triggering an explicit call to updateGrid() or not. If
                % set to false, not only updateGrid() is not called but the
                % caller must also deal with the unfreezing of the app.
                nvargs.UpdateGrid (1,1) logical = true;
                % Init is a logical flag that signals if this is the first
                % time that the SIB1 function is being called. The error
                % handling is different based on this.
                nvargs.Init (1,1) logical = false;
            end

            if obj.Sib1Check                                                        % Required: the call from certain parts of the app is blind to whether it's enabled.
                dlg = obj.CurrentDialog;
                freezeApp(dlg.getParent.AppObj);                                    % Linux takes a very long time to load the SIB1 channels so freeze app
                enableVizUpdates(obj, false);                                       % Disable visualization to avoid unnecessary intermediate grid updates
                classNameSIB1 = 'wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog';
                sib1Param = obj.Parent.WaveformGenerator.pParameters.DialogsMap(classNameSIB1);
                classNameSSB = 'wirelessWaveformApp.nr5G_SSB_Dialog';
                dlgSSB = obj.Parent.WaveformGenerator.pParameters.DialogsMap(classNameSSB);
                e=[];
                scsForBWPCommon = 0;
                try
                    % Toggle valid high to obtain SSBurst config
                    obj.SIB1Valid = true;
                    % Get configuration for SSBurst and SCSCarriers
                    % to ensure SIB1 has valid carriers
                    ssbConfig = getSSBConfig(dlgSSB);
                    obj.SIB1Valid = false;

                    scsCarriersConfig = getWaveConfig(dlg, 'scscarriers');
                    scsCommon = ssbConfig.SubcarrierSpacingCommon;
                    allScs = [scsCarriersConfig{:}];
                    scsForBWPCommon = scsCommon == [allScs.SubcarrierSpacing];

                    if ~any(scsForBWPCommon)
                        % This occurs when SIB1 is attempted but a carrier
                        % in the definition is missing
                        coder.internal.error('nr5g:waveformGeneratorApp:Sib1CarrierDeleted');
                    end

                    % Internal function generating SIB1 channels
                    sib1cfg = wirelessWaveformGenerator.internal.SIB1ConfigGen.createSIB1WaveConfig(ssbConfig,dlg.NCellID,sib1Param,scsCarriersConfig);

                    % Re-enable visuals to ensure the new figure for SIB1-BWP is displayed
                    enableVizUpdates(obj, true);
                    % Apply BWP-SIB1
                    [obj.SIB1BWPIndex,obj.SIB1BWPID, sib1cfgBWPUpdate] = sib1TableLoad(obj,'bwpTable',sib1cfg.BandwidthParts,obj.SIB1BWPID);
                    sib1cfg.BandwidthParts = sib1cfgBWPUpdate;
                    amendCachedConfigBWP(dlg, sib1cfg.BandwidthParts, obj.SIB1BWPIndex);
                    enableVizUpdates(obj, false); % Re-disable the visualizations
                    % Adding BWP toggles app freeze mid function, therefore refreezing
                    freezeApp(dlg.getParent.AppObj);

                    % Apply Coreset0
                    obj.SIB1COREIndex = sib1TableLoad(obj,'coresetTable',sib1cfg.CORESET,obj.SIB1COREID);
                    amendCachedConfigCORESET(dlg, sib1cfg.CORESET, obj.SIB1COREIndex);

                    % Apply SearchSpace0
                    [obj.SIB1SSIndex, obj.SIB1SSID, sib1cfgSSUpdate] = sib1TableLoad(obj,'searchSpacesTable',sib1cfg.SearchSpaces,obj.SIB1SSID);
                    sib1cfg.SearchSpaces = sib1cfgSSUpdate;
                    amendCachedConfigSearchSpaces(dlg, sib1cfg.SearchSpaces, obj.SIB1SSIndex);

                    % Apply DCI
                    [obj.SIB1PDCCHIndex,obj.SIB1PDCCHID,sib1cfgPDCCHUpdate] = sib1TableLoad(obj,'pdcchTable',sib1cfg.PDCCH,obj.SIB1PDCCHID);
                    sib1cfg.PDCCH = sib1cfgPDCCHUpdate;
                    amendCachedConfigPDCCH(dlg, sib1cfg.PDCCH, obj.SIB1PDCCHIndex);

                    % Apply SIB1
                    [obj.SIB1PDSCHIndex,obj.SIB1PDSCHID,sib1cfgPDSCHUpdate] = sib1TableLoad(obj,'pxschTable',sib1cfg.PDSCH,obj.SIB1PDSCHID);
                    sib1cfg.PDSCH = sib1cfgPDSCHUpdate;
                    amendCachedConfigPXSCH(dlg, sib1cfg.PDSCH, obj.SIB1PDSCHIndex);


                    % Save current SIB1 config object in a local cache
                    obj.SIB1Config = sib1cfg;
                    collapseSIB1Panel(sib1Param,false);

                    % Set SIB1 table rows editable ie read-only now that
                    % SIB1 channels exist and are valid
                    updateRowEditabilitySIB1(obj,false);

                    % Re-enable visualization
                    enableVizUpdates(obj, true);

                    % SIB1 is now in a valid state (must be set before
                    % cache update)
                    obj.SIB1Valid = true;
                    obj.SIB1Error = [];

                    % Now that everything is updated, update the resource grid
                    % Ensure each channel's specific cache is updated
                    % correctly before updating the grid
                    updateCachedConfig(dlg, 'BWP');
                    updateCachedConfig(dlg, 'CORESET');
                    updateCachedConfig(dlg, 'SearchSpaces');
                    updateCachedConfig(dlg, 'PDCCH');
                    updateCachedConfig(dlg, 'PDSCH');
                    if nvargs.UpdateGrid
                        updateGrid(dlg);
                    end

                catch e
                    obj.SIB1Valid = false;
                    obj.SIB1Error = e;
                    enableVizUpdates(obj, true);
                    if ~any(scsForBWPCommon)
                        throwErrorPopup(obj, e);     % SIB1 is being deleted. Alert user!
                        sib1Delete(obj);
                        obj.Sib1Check = 0;
                    else
                        if nvargs.Init               % Error on initial SIB1 generation
                            throwErrorPopup(obj, e); % Throw a pop-up, don't fill banner
                        else                         % Else, fill banner as config needs fixed
                            updateAppConfigDiagnostic(obj.CurrentDialog, e);
                        end
                    end
                end

                % Ensure the app is not frozen anymore. This only happens
                % here when the NV argument UpdateGrid is set to true. When
                % UpdateGrid is false, the caller must deal with the
                % unfreezing of the app.
                if nvargs.UpdateGrid
                    unfreezeApp(dlg.getParent.AppObj);
                end
            end
        end

        function success = updateSIB1ChannelIDs(obj,loadedConfig)
            % Update SIB1 channel row ID for internal tracking. This is
            % used when loading a session with SIB1 enabled.
            success = true;
            sib1Config = loadedConfig.SIB1;
            cfg = loadedConfig.waveform;
            if sib1Config.SIB1Enabled
                if isfield(sib1Config,"SIB1PDSCHIndex")
                    % All the needed information available in the saved
                    % metadata
                    try
                        [obj.SIB1PDSCHID, obj.SIB1PDSCHIndex] = deal(sib1Config.SIB1PDSCHIndex);
                        [obj.SIB1PDCCHID, obj.SIB1PDCCHIndex] = deal(sib1Config.SIB1PDCCHIndex);
                        obj.SIB1COREIndex = sib1Config.SIB1COREIndex;
                        obj.SIB1COREID = cfg.CORESET{obj.SIB1COREIndex}.CORESETID;
                        obj.SIB1SSIndex = sib1Config.SIB1SSIndex;
                        obj.SIB1SSID = cfg.SearchSpaces{obj.SIB1SSIndex}.SearchSpaceID;
                        obj.SIB1BWPIndex = sib1Config.SIB1BWPIndex;
                        obj.SIB1BWPID = cfg.BandwidthParts{obj.SIB1BWPIndex}.BandwidthPartID;
                    catch ME
                        % Something went wrong. Likely, a corrupted saved
                        % file was loaded
                        success = false;
                    end
                else
                    % Retrieve the indices of the SIB1-related channel
                    % instances from the labels
                    % PDSCH
                    pdschLabels = getLabels(cfg.PDSCH);
                    pdschEnabled = isEnabled(cfg.PDSCH);
                    [obj.SIB1PDSCHID, obj.SIB1PDSCHIndex] = deal(find(pdschEnabled & strcmp(pdschLabels,"PDSCH1-SIB1"),1));
                    % PDCCH
                    pdcchLabels = getLabels(cfg.PDCCH);
                    pdcchEnabled = isEnabled(cfg.PDCCH);
                    [obj.SIB1PDCCHID, obj.SIB1PDCCHIndex] = deal(find(pdcchEnabled & strcmp(pdcchLabels,"PDCCH1-SIB1"),1));
                    % CORESET
                    coresetLabels = getLabels(cfg.CORESET);
                    obj.SIB1COREIndex = deal(find(strcmp(coresetLabels,"CORESET0-SIB1"),1));
                    obj.SIB1COREID = cfg.CORESET{obj.SIB1COREIndex}.CORESETID;
                    % Search spaces
                    ssLabels = getLabels(cfg.SearchSpaces);
                    obj.SIB1SSIndex = deal(find(strcmp(ssLabels,"SearchSpace0-SIB1"),1));
                    obj.SIB1SSID = cfg.SearchSpaces{obj.SIB1SSIndex}.SearchSpaceID;
                    % BWP
                    bwpLabels = getLabels(cfg.BandwidthParts);
                    obj.SIB1BWPIndex = deal(find(strcmp(bwpLabels,"BWP-SIB1"),1));
                    obj.SIB1BWPID = cfg.BandwidthParts{obj.SIB1BWPIndex}.BandwidthPartID;

                    success = ~any(isempty([obj.SIB1PDSCHID,obj.SIB1PDSCHIndex,obj.SIB1PDCCHID,obj.SIB1PDCCHIndex,obj.SIB1COREID,obj.SIB1COREIndex,obj.SIB1SSID,obj.SIB1SSIndex,obj.SIB1BWPID,obj.SIB1BWPIndex]));
                end

                if ~any(success)
                    % Reset all inidices to the default value and throw the error
                    resetSIB1Indices(obj);
                    throwErrorPopup(obj.CurrentDialog,getString(message('nr5g:waveformGeneratorApp:SIB1LoadFailed')));
                end
            end

            function labels = getLabels(chCfg)
                labels = cellfun(@(x)string(x.Label),chCfg,UniformOutput=true);
            end
            function enabled = isEnabled(chCfg)
                if ~isprop(chCfg{1},"Enable")
                    enabled = true(size(chCfg));
                else
                    enabled = cellfun(@(x)x.Enable,chCfg,UniformOutput=true);
                end
            end
        end

        function updateRowEditabilitySIB1(obj,editable)
            % Update editability of the channel table rows based on whether
            % SIB1 is enabled or not

            arguments
                obj
                editable (1,1) logical
            end

            dlg = obj.CurrentDialog;
            updateRowEditable(dlg.bwpTable, Row=obj.SIB1BWPIndex, Editable=editable);
            updateRowEditable(dlg.coresetTable, Row=obj.SIB1COREIndex, Editable=editable);
            updateRowEditable(dlg.searchSpacesTable, Row=obj.SIB1SSIndex, Editable=editable);
            updateRowEditable(dlg.pdcchTable, Row=obj.SIB1PDCCHIndex, Editable=editable);
            updateRowEditable(dlg.pxschTable, Row=obj.SIB1PDSCHIndex, Editable=editable);
        end

        function ssbDMRSTypeAPositionChangedGUI(obj)
            % Callback for user interaction with the SSBDMRSTypeAPosition
            % dropdown
            classNameSIB1 = 'wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog';
            if isKey(obj.Parent.AppObj.pParameters.DialogsMap, classNameSIB1)
                sib1Param = obj.Parent.AppObj.pParameters.DialogsMap(classNameSIB1);
                sib1Param.Sib1SymbolSetChange(obj.SSBDMRSTypeAPosition);
                updateSIB1Config(obj);
            end
        end

        function ssbDMRSTypeAPositionChangedObj(obj,~,~)
            % Callback for programmatic interaction with the
            % SSBDMRSTypeAPosition dropdown
            classNameSIB1 = 'wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog';
            if isKey(obj.Parent.AppObj.pParameters.DialogsMap, classNameSIB1)
                sib1Param = obj.Parent.AppObj.pParameters.DialogsMap(classNameSIB1);
                sib1Param.Sib1SymbolSetChange(obj.SSBDMRSTypeAPosition);
            end
        end

        function sib1NCellIDUpdate(obj)
            if obj.Sib1Check
                % Cell ID change only affects ShiftIndex of
                % CORESET0, no need for full update.
                dlg = obj.CurrentDialog;
                coresetConfig = dlg.coresetWaveConfig{obj.SIB1COREIndex};
                coresetConfig.ShiftIndex = dlg.NCellID;
                applyConfiguration(dlg.coresetTable, {coresetConfig}, AllowUIChange=false, ConfigIDs=coresetConfig.CORESETID);
                amendCachedConfigCORESET(dlg, {coresetConfig}, obj.SIB1COREIndex);
            end
        end
    end

    methods (Access = private)

        % Validators and visibility updates
        function configChangedGUI(obj, src, evnt)
            % First, check that the standalone value is correct
            ME = updateProperty(obj, src, evnt, "PDCCHConfigSIB1");

            % If there is no error, update SIB1 associated things
            if isempty(ME)
                % Decode CORESET0 and Searchspace0 from PDCCHConfigSIB1
                Coreset0Idx = floor(obj.PDCCHConfigSIB1/16);
                SearchSpace0Idx = mod(obj.PDCCHConfigSIB1,16);
                obj.Sib1ConfigIndices = [num2str(Coreset0Idx),',',num2str(SearchSpace0Idx)];
                try
                    classNameSIB1 = 'wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog';
                    sib1Param = obj.Parent.WaveformGenerator.pParameters.DialogsMap(classNameSIB1);
                    classNameSSB = 'wirelessWaveformApp.nr5G_SSB_Dialog';
                    dlgSSB = obj.Parent.WaveformGenerator.pParameters.DialogsMap(classNameSSB);


                    scsSSB = nr5g.internal.wavegen.blockPattern2SCS(dlgSSB.BlockPattern);
                    scsCommon = dlgSSB.SubcarrierSpacingCommon;
                    scsPair = [scsSSB scsCommon];

                    % Read CORESET0 configuration from TS 38.213 Tables 13-1 to 13-10
                    [~,csetDuration,~,csetPattern] = wirelessWaveformGenerator.internal.SIB1ConfigGen.hCORESET0Resources(Coreset0Idx,scsPair,sib1Param.Sib1MinBW,0);

                    % Only change SIB1 layout if a valid, supported coreset pattern returned
                    if csetPattern == 1
                        % Read SearchSpace0 configuration from TS 38.213 Tables 13-11 to 13-14
                        ssSlot = wirelessWaveformGenerator.internal.SIB1ConfigGen.hPDCCH0MonitoringOccasions(SearchSpace0Idx, 0:length(dlgSSB.TransmittedBlocks)-1,scsPair,csetPattern,csetDuration,dlgSSB.BlockPattern);
                        twoSSFlag = any(~diff(ssSlot));

                        setVisible(sib1Param, {'Sib1SymbolSet2'}, twoSSFlag);
                        layoutUIControls(sib1Param);

                        % Clear any existing SIB1 incase we're going from 1 to
                        % 2 configs. Don't clear BWP figure as we will
                        % still have 2 BWPs
                        clearBWPFig = 0;
                        sib1Delete(obj,clearBWPFig);
                    end
                    % Update tbs in case 2nd symbolset was added.
                    Sib1TBSUpdate(sib1Param);
                    updateSIB1Config(obj);
                catch e
                    throwErrorPopup(obj, e);
                end
            end
        end

        function enableVizUpdates(obj, tf)
            % Enable or disable visualization updates. If TF is false,
            % every call to updateGrid() results in a no-op until the
            % visualization is set back on.
            appObj = obj.Parent.AppObj;
            appObj.pShowVisualizations = tf;
        end

        % SIB1 methods
        function [sib1TableIndex,sib1TableID,sib1CfgUpdate] = sib1TableLoad(obj,downlinkTableName,sib1ChanCfg,sib1TableID)
            % This function handles aligning the config with the table and
            % vice versa. Works for all tables.

            % Get the table we're updating and checking for IDs
            tableObj = obj.CurrentDialog.(downlinkTableName);
            newEntry = false;

            % How many configs were passed in for this channel?
            numConfigs = numel(sib1ChanCfg);

            % Initialisation check ie "first time SIB runs"
            if isempty(sib1TableID)
                newEntry = true;

                % Generate as many IDs as required by size of the config
                sib1TableID = getNewIDs(tableObj.AllIDs, numConfigs);

                % If we're initialising BWP or SS, assign the index values
                % immediately, from inside this function
                if strcmp(downlinkTableName,'bwpTable')
                    obj.SIB1BWPID=sib1TableID;
                elseif strcmp(downlinkTableName,'searchSpacesTable')
                    obj.SIB1SSID=sib1TableID;
                end
            end

            % For every channel config with a "BandwidthPartID" or
            % "SearchSpaceID" field, assign the current IDs.

            if isprop(sib1ChanCfg{1},"BandwidthPartID")
                for cfgIdx = 1:numConfigs
                    sib1ChanCfg{cfgIdx}.BandwidthPartID =  obj.SIB1BWPID;
                end
                sib1CfgUpdate = sib1ChanCfg; % Updating Cache
            end

            if isprop(sib1ChanCfg{1},"SearchSpaceID")
                for cfgIdx = 1:numConfigs
                    sib1ChanCfg{cfgIdx}.SearchSpaceID =  obj.SIB1SSID(cfgIdx);
                end
                sib1CfgUpdate = sib1ChanCfg; % Updating Cache
            end

            % If this was a newly initialised channel
            if newEntry
                % New SIB1 must be added to table --> A new instance is needed
                appendConfiguration(tableObj, sib1ChanCfg);
            else
                % Existing SIB1 configuration has been updated
                applyConfiguration(tableObj, sib1ChanCfg, AllowUIChange=false, ConfigIDs=sib1TableID);
            end

            % Ensure that the returned indices for this channel are up to date
            sib1TableIndex = zeros(1,numConfigs);
            for tblIdx = 1: numConfigs
                sib1TableIndex(tblIdx) = find(tableObj.AllIDs==sib1TableID(tblIdx));
            end
        end

        function amendCachedConfigFlag = sib1TableUnload(obj,downlinkTableName,defaultChanCfg,sib1TableID)
            % Get the table we're updating and checking for IDs
            tableObj = obj.CurrentDialog.(downlinkTableName);
            if numel(tableObj.AllIDs) > 1
                removeConfiguration(tableObj, sib1TableID);
                amendCachedConfigFlag = false;
            else
                % Only the SIB1 configuration existed for this channel.
                % Append the default configuration (disabled, if
                % applicable) and then remove the SIB1 configuration
                appendConfiguration(tableObj, defaultChanCfg);
                removeConfiguration(tableObj, sib1TableID);
                % Update cached configuration
                amendCachedConfigFlag = true;
            end
        end

        function resetSIB1Indices(obj)
            % Reset SIB1 inidices to their default values
            [obj.SIB1PDSCHID, obj.SIB1PDSCHIndex] = deal([]);
            [obj.SIB1PDCCHID, obj.SIB1PDCCHIndex] = deal([]);
            [obj.SIB1SSID, obj.SIB1SSIndex] = deal([]);
            [obj.SIB1BWPID, obj.SIB1BWPIndex] = deal([]);
            obj.SIB1COREID = 0;
            obj.SIB1COREIndex = [];
        end
    end

    % Getters/setters
    methods
        function n = get.Payload(obj)
            n = getEditVal(obj, 'Payload');
        end
        function set.Payload(obj, val)
            if iscell(val)
                obj.PayloadGUI.(obj.EditValue) = sprintf('{''%s'',%d}',val{1},val{2});
            else
                setEditVal(obj, 'Payload', val);
            end
        end

        function val = get.SSBDMRSTypeAPosition(obj)
            val = getDropdownNumVal(obj, 'SSBDMRSTypeAPosition');
        end
        function set.SSBDMRSTypeAPosition(obj, val)
            setDropdownNumVal(obj, 'SSBDMRSTypeAPosition', val);
        end

        function out = get.ChannelName(~)
            out = 'SSB';
        end
    end
end

function s = getUserDefinedSourceString()

    s = getString(message('nr5g:waveformGeneratorApp:SSDataSourceUserDefined'));

end

function newIDs = getNewIDs(existingIDs, numNewInstances)
    possibleIDs = 1:(length(existingIDs)+numNewInstances);
    % Find which unique row numbers can now be used
    possibleIDs = setdiff(possibleIDs, existingIDs, 'stable');
    newIDs = possibleIDs(1:numNewInstances);
end

