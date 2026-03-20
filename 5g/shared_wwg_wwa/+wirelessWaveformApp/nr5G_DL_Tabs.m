classdef nr5G_DL_Tabs < wirelessWaveformApp.nr5G_MainGrid_Tab & ...
        wirelessWaveformApp.nr5G_PDCCH_Tab & ...
        wirelessWaveformApp.nr5G_PDSCH_Tab & ...
        wirelessWaveformApp.nr5G_CSIRS_Tab
    % Downlink-specific tabs of Full DL 5G configuration (used in WWG & WWA)

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Hidden)
        % List the name of all table objects contained in this waveform type
        tableObjName = ["scsCarriersTable", "bwpTable", "pxschTable", "coresetTable", "searchSpacesTable", "pdcchTable", "csirsTable"];
        % List the names of the figures containing the tabs
        tabFigNames = ["mainDLFig", "ssBurstFig", "pxschFig", "pdcchFig", "csirsFig"];
    end

    properties (Abstract, Hidden)
        SSBTabName
    end

    properties (Hidden, Constant)
        % Cache default nrDLCarrierConfig for UX speedup. Constructor takes
        % 0.5 sec because of numerous properties and validations
        % (validatestring). CTOR is exercised in updateGrid ->
        % getConfiguration -> nrDLCarrierConfig
        DefaultCfg = getDefaultConfig();
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_DL_Tabs(nvargs)

            % Parse any potential column to hide
            arguments
                % Name-Value arguments
                nvargs.InvisibleEntriesPDSCH   (1,:) string = string.empty
                nvargs.InvisibleEntriesCORESET (1,:) string = string.empty
                nvargs.InvisibleEntriesSS      (1,:) string = string.empty
                nvargs.InvisibleEntriesPDCCH   (1,:) string = string.empty
                nvargs.InvisibleEntriesCSIRS   (1,:) string = string.empty
            end

            % Constructor call
            defaultConfig = wirelessWaveformApp.nr5G_DL_Tabs.DefaultCfg;
            obj@wirelessWaveformApp.nr5G_MainGrid_Tab(defaultConfig, true); % call base constructor
            obj@wirelessWaveformApp.nr5G_PDSCH_Tab(defaultConfig, nvargs.InvisibleEntriesPDSCH); % call base constructor
            obj@wirelessWaveformApp.nr5G_PDCCH_Tab(defaultConfig, nvargs.InvisibleEntriesCORESET, ...
                nvargs.InvisibleEntriesSS, ...
                nvargs.InvisibleEntriesPDCCH); % call base constructor
            obj@wirelessWaveformApp.nr5G_CSIRS_Tab(defaultConfig, nvargs.InvisibleEntriesCSIRS); % call base constructor
        end

        function names = figureNames(obj)
            names = {'Main', obj.SSBTabName, 'PDSCH', 'PDCCH', 'CSI-RS'};
        end

        function tag = figTag(obj, name)
            switch name
                case 'Main'
                    tag = 'mainDLFig';
                case obj.SSBTabName
                    tag = 'ssBurstFig';
                case 'PDSCH'
                    tag = 'pxschFig';
                case 'PDCCH'
                    tag = 'pdcchFig';
                case 'CSI-RS'
                    tag = 'csirsFig';
            end
        end

        function createFigures(obj)
            % Create figures for all 5 tabs:
            obj.createFig(obj.SSBTabName,           'ssBurstFig');
            obj.createFig('PDSCH',                  'pxschFig');
            obj.createFig('PDCCH',                  'pdcchFig');
            obj.createFig('CSI-RS',                 'csirsFig');

            % PDSCH tab has an extra side fig (right-side panel) that is not
            % visible under the Main tab:
            appObj = obj.getParent.AppObj;
            obj.createFig('PDSCH 1 - Advanced Configuration', 'pxschSingleChannelFig', obj.pxschExtraFigTag);
            appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", obj.pxschExtraFigTag).Visible = false;
        end

        function setExtraConfigFigVisibility(obj, visibility)
            % This method turns on or off all tabs if the waveform type changes
            % (e.g., Full DL -> WLAN)

            appObj = obj.getParent.AppObj;
            appObj.pParameters.Layout.Parent = appObj.pParametersFig; % otherwise closing Main tab will kill Accordion panels

            names = figureNames(obj);
            for idx = 1:numel(names)
                doc = appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", figTag(obj,names{idx}));
                doc.Visible = visibility;
            end

            doc = appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", obj.pxschExtraFigTag);
            doc.Visible = visibility && any(strcmpi(obj.lastFigInFocus, obj.PXSCHfigureName));
        end

        function cleanupDlg(obj)
            % Dialog-specific cleanup when app is closing
            % Custom 5G DL has additional UI objects that need deletion when the
            % app is closing.
            delete([obj.pxschFig; obj.pxschSingleChannelFig; obj.ssBurstFig; obj.pdcchFig; obj.csirsFig])
        end

        function extraPanels = getExtraPanels(obj)
            extraPanels = {};
            extraPanels = cat(1,extraPanels,mat2cell(obj.pxschGridLayout.Children(:),ones(numel(obj.pxschGridLayout.Children(:)),1)));
            extraPanels = cat(1,extraPanels,mat2cell(obj.pdcchGridLayout.Children(:),ones(numel(obj.pdcchGridLayout.Children(:)),1)));
            extraPanels = cat(1,extraPanels,mat2cell(obj.csirsGridLayout.Children(:),ones(numel(obj.csirsGridLayout.Children(:)),1)));
        end

        function cfg = getConfiguration(obj)
            % Map all graphical content into an equivalent nrDLCarrierConfig object

            % Top level properties:
            cfg = obj.DefaultCfg; % default nrDLCarrierConfig object
            cfg.Label             = obj.Label;
            cfg.FrequencyRange    = obj.FrequencyRange;
            cfg.ChannelBandwidth  = obj.ChannelBandwidth;
            cfg.NCellID           = obj.NCellID;
            cfg.NumSubframes      = obj.NumSubframes;
            cfg.InitialNSubframe  = obj.InitialNSubframe;

            % Windowing controls are slightly different than programmatic API
            if strcmp(obj.WindowingSource, 'Auto')
                cfg.WindowingPercent = [];
            else
                cfg.WindowingPercent = obj.WindowingPercent;
            end

            if strcmp(obj.SampleRateSource, 'Auto')
                cfg.SampleRate = [];
            else
                cfg.SampleRate = obj.SampleRate;
            end

            if obj.PhaseCompensation
                cfg.CarrierFrequency   = obj.CarrierFrequency;
            else
                cfg.CarrierFrequency   = 0;
            end

            % SCS carriers & BWP:
            cfg.SCSCarriers = getWaveConfig(obj, 'scscarriers');
            cfg.BandwidthParts = getWaveConfig(obj, 'bwp');

            % SS Burst
            className = 'wirelessWaveformApp.nr5G_SSB_Dialog';
            if isKey(obj.getParent.DialogsMap, className)
                ssbDialog = obj.getParent.DialogsMap(className);
                cfg.SSBurst = getSSBConfig(ssbDialog);
            end

            % CORESET, Search spaces, PDCCH
            cfg.CORESET = getWaveConfig(obj, 'coreset');
            cfg.SearchSpaces = getWaveConfig(obj, 'searchspaces');
            cfg.PDCCH = getWaveConfig(obj, 'pdcch');

            % PDSCH
            cfg.PDSCH = getWaveConfig(obj, 'pdsch');

            % CSI-RS
            cfg.CSIRS = getWaveConfig(obj, 'csirs');
        end

        function markBrokenBWPLinks(obj)
            % paint red the rows that do not link to a valid BWP
            brokenLinks = struct('pdsch',[],'pdcch',[],'csirs',[]);
            chNames = fieldnames(brokenLinks);
            brokenLinks = getBrokenLinks(obj, 'bwp', chNames, 'BandwidthPartID', brokenLinks);
            markBrokenLinks(obj, brokenLinks);
        end

        function restoreDefaults(obj)
            % Disable visibility of side panels
            obj.pxschExtraTile = false;

            % Disable SIB1 before restoring the configuration
            classNameDS = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
            if isKey(obj.getParent.DialogsMap, classNameDS)
                dlgSSBDS = obj.getParent.DialogsMap(classNameDS);
                dlgSSBDS.Sib1Check = 0;
            end

            % Get the default wave configuration object and apply it to the
            % app
            cfg = obj.DefaultCfg;
            applyConfiguration(obj, cfg);

            % Restoring SSB Payload dialog for new sessions (Primarily for
            % non config object properties eg SIB1Check)
            if isKey(obj.getParent.DialogsMap, classNameDS)
                % Restoring SIB1 dialog for new sessions
                classNameSIB1 = 'wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog';
                if isKey(obj.getParent.DialogsMap, classNameSIB1)
                    dlgSIB1 = obj.getParent.DialogsMap(classNameSIB1);
                    restoreDefaults(dlgSIB1);
                end
            end
        end

        function applyConfiguration(obj, waveConfig)
            % Used on New, Open Session, openInGenerator, and in displaying
            % the configuration in WWA

            % Main tab
            applyConfigSCSCarriers(obj, waveConfig);
            applyConfigBWP(obj, waveConfig);

            % Make sure all SCS-related broken links are correctly reported
            brokenLinks.bwp = [];
            brokenLinks = getBrokenLinks(obj, 'scscarriers', 'bwp', 'SubcarrierSpacing', brokenLinks);
            markBrokenLinks(obj, brokenLinks);

            % Channels
            applyConfigSSBurst(obj, waveConfig);
            applyConfigCORESET(obj, waveConfig);
            applyConfigSearchSpaces(obj, waveConfig);
            applyConfigPDCCH(obj, waveConfig);
            applyConfigPXSCH(obj, waveConfig);
            applyConfigCSIRS(obj, waveConfig);

            % SIB1-specific actions
            className = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
            if isKey(obj.getParent.DialogsMap, className)
                dlgSSBDataSource = obj.getParent.DialogsMap(className);
                if dlgSSBDataSource.Sib1Check
                    % Expand SIB1 panel
                    classNameSIB1 = 'wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog';
                    sib1Param = obj.getParent.DialogsMap(classNameSIB1);
                    collapseSIB1Panel(sib1Param,false);
                    % Update editability of SIB1-specific rows in the
                    % channels tables
                    updateRowEditabilitySIB1(dlgSSBDataSource,false);
                end
            end

        end

        function applyConfigSSBurst(obj, cfg)
            % Map an nrDLCarrierConfig.SSBurst = nrWavegenSSBConfig object to all
            % UI elements under the SS Burst tab.
            ssb = cfg.SSBurst;

            % SS Burst main dialog
            className = 'wirelessWaveformApp.nr5G_SSB_Dialog';
            if ~isKey(obj.getParent.DialogsMap, className)
                return; % App initialization
            end

            % Mapping for properties that have 1-1 equivalence between GUI and programmatic API:
            dlgSSB = obj.getParent.DialogsMap(className);
            if ~ishghandle(dlgSSB.BlockPatternGUI)
                return; % App 2nd initialization
            end

            props = {'Power', 'BlockPattern', 'TransmittedBlocks', ...
                'KSSB', 'NCRBSSB', 'SubcarrierSpacingCommon'};
            for idx = 1:numel(props)
                dlgSSB.(props{idx}) = ssb.(props{idx});
            end

            dlgSSB.EnableSSB = ssb.Enable;
            dlgSSB.Period = ssb.Period(1);
            if ~isscalar(ssb.Period)
                dlgSSB.HalfFrameOffset = ssb.Period(2);
            else
                dlgSSB.HalfFrameOffset = 0;
            end

            % Custom frequency offsets are exposed differently in GUI than programmatic API
            if isempty(ssb.NCRBSSB)
                dlgSSB.FrequencyOffset = getString(message('nr5g:waveformApp:SSFrequencyOffsetCarrierCenter'));
            else
                dlgSSB.FrequencyOffset = getString(message('nr5g:waveformApp:SSFrequencyOffsetCustom'));
            end

            updateControlsVisibility(dlgSSB);

        end

        function frChanged(obj, ~)
            % Executes when Frequency Range changes (FR1<->FR2)

            %% SS Burst:
            % Need to take care of affected options in SS Burst tab:
            % - Block Pattern
            % - Transmitted Blocks
            % - Subcarrier Spacing Common

            className   = 'wirelessWaveformApp.nr5G_SSB_Dialog';
            if isKey(obj.getParent.DialogsMap, className)
                ssbDialog = obj.getParent.DialogsMap(className);
                if ~ishghandle(ssbDialog.BlockPatternGUI)
                    % Re-entering DL 5G, 2nd initialization not complete yet
                    return;
                end

                if strcmp(obj.FrequencyRange, 'FR1')
                    % Update dropdown:
                    ssbDialog.BlockPatternGUI.(ssbDialog.DropdownValues) = {'Case A (15 kHz)', 'Case B (30 kHz)', 'Case C (30 kHz)'};
                    ssbDialog.TransmittedBlocks = ones(1, 4);
                    ssbDialog.SubcarrierSpacingCommonGUI.(ssbDialog.DropdownValues) = {'15 kHz', '30 kHz'};
                else
                    % Update dropdown:
                    ssbDialog.BlockPatternGUI.(ssbDialog.DropdownValues) = {'Case D (120 kHz)', 'Case E (240 kHz)', 'Case F (480 kHz)', 'Case G (960 kHz)'};
                    ssbDialog.TransmittedBlocksGUI.(ssbDialog.EditValue) = '[1:64]';
                    ssbDialog.SubcarrierSpacingCommonGUI.(ssbDialog.DropdownValues) = {'60 kHz', '120 kHz'};
                end

                customFreqOffset = getString(message('nr5g:waveformApp:SSFrequencyOffsetCustom'));
                if strcmpi(ssbDialog.FrequencyOffset,customFreqOffset)
                    updateCustomFrequencyOffsetUnits(ssbDialog);
                end

                updateSubcarrierSpacingCommonVisibility(ssbDialog);

            end
        end

        function w = getColumnWeights(~, ~)
            % The right-side column (PDSCH Advanced config or Spectrum Analyzer)
            % is a bit smaller.
            c = 0.7324;
            w = [nan; c; 1-c];
        end

    end

    % Protect methods for derived classes
    methods (Access = protected)
        %% Visualization
        function markAllBrokenLinks(obj)
            % Update all broken links by highlighting red color in invalid rows
            brokenLinks = struct('pdsch',[],'pdcch',[],'csirs',[],'searchspaces',[]);
            brokenLinks = getBrokenLinks(obj, 'bwp', {'pdsch', 'pdcch', 'csirs'}, 'BandwidthPartID', brokenLinks); % BWP ID in PDSCH, PDCCH, CSI-RS table
            brokenLinks = getBrokenLinks(obj, 'coreset', 'searchspaces', 'CORESETID', brokenLinks); % CORESET ID in Search Spaces table
            brokenLinks = getBrokenLinks(obj, 'searchspaces', 'pdcch', 'SearchSpaceID', brokenLinks); % SearchSpaceID in PDCCH table
            markBrokenLinks(obj, brokenLinks);
        end


        % Link-specific actions when FR changes
        function frChangedForLink(obj)
            ssbDataSourceClassName = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
            if isKey(obj.getParent.DialogsMap,ssbDataSourceClassName)
                dataSourceDLG = obj.getParent.DialogsMap(ssbDataSourceClassName);
                % If SIB1 exists while changing FR
                if dataSourceDLG.Sib1Check
                    % Disable/Delete SIB1 channels that exist (they use invalid parameters)
                    sib1Delete(dataSourceDLG);
                    dataSourceDLG.Sib1Check = 0;
                end
            end
            classNameSIB1 = 'wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog';
            if isKey(obj.getParent.DialogsMap, classNameSIB1)
                dlgSIB1 = obj.getParent.DialogsMap(classNameSIB1);
                Sib1FR2View(dlgSIB1, strcmp(obj.FrequencyRange, 'FR1'));
            end            
        end

    end
end

function waveCfg = getDefaultConfig()
    % Get the default configuration object from nrDLCarrierConfig
    waveCfg = nrDLCarrierConfig;
    waveCfg.Label = 'Carrier1';

    % Take care of CORESET 0, which is not present in nrDLCarrierConfig
    waveCfg.CORESET{2} = waveCfg.CORESET{1}; % Row vector of CORESETs
    waveCfg.CORESET{1}.Label = 'CORESET0';
    waveCfg.CORESET{1}.CORESETID = 0;

    % One difference is that carriers are prepared for equal span in
    % different SCS (maximum occupancy).
    def15kHzNRB = 270;
    waveCfg.SCSCarriers{1}.NSizeGrid   = def15kHzNRB;
    waveCfg.SCSCarriers{1}.NStartGrid  = 3;
    waveCfg.BandwidthParts{1}.NSizeBWP = def15kHzNRB;
    waveCfg.BandwidthParts{1}.NStartBWP = 3;

    waveCfg.PDSCH{1}.PRBSet = 0:(def15kHzNRB-1);
    waveCfg.PDSCH{1}.EnablePTRS = logical(waveCfg.PDSCH{1}.EnablePTRS);
end
