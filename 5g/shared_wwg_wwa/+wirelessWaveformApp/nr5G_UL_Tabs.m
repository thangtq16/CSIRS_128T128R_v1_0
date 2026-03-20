classdef nr5G_UL_Tabs < wirelessWaveformApp.nr5G_MainGrid_Tab & ...
                        wirelessWaveformApp.nr5G_PUCCH_Tab & ...
                        wirelessWaveformApp.nr5G_PUSCH_Tab & ...
                        wirelessWaveformApp.nr5G_SRS_Tab & ...
                        wirelessWaveformApp.nr5G_ICGB_Dialog
    % Uplink-specific tabs of Full UL 5G configuration (used in WWG & WWA)

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Hidden)
        % List the name of all table objects contained in this waveform type
        tableObjName = ["scsCarriersTable", "bwpTable", "pxschTable", "pucchTable", "srsTable"];
        % List the names of the figures containing the tabs
        tabFigNames = ["mainULFig", "pxschFig", "pucchFig", "srsFig"];
    end

    properties (Hidden, Constant)
        % Cache default nrULCarrierConfig for UX speedup. Constructor takes
        % 0.5 sec because of numerous properties and validations
        % (validatestring). CTOR is exercised in updateGrid ->
        % getConfiguration -> nrULCarrierConfig
        DefaultCfg = getDefaultConfig();
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_UL_Tabs(nvargs)

            % Parse any potential column to hide
            arguments
                % Name-Value arguments
                nvargs.InvisibleEntriesPUSCH (1,:) string = string.empty
                nvargs.InvisibleEntriesPUCCH (1,:) string = string.empty
                nvargs.InvisibleEntriesSRS   (1,:) string = string.empty
            end

            % Constructor call
            defaultConfig = wirelessWaveformApp.nr5G_UL_Tabs.DefaultCfg;
            obj@wirelessWaveformApp.nr5G_MainGrid_Tab(defaultConfig, false); % call base constructor
            obj@wirelessWaveformApp.nr5G_ICGB_Dialog(); % call base constructor
            obj@wirelessWaveformApp.nr5G_PUSCH_Tab(defaultConfig, nvargs.InvisibleEntriesPUSCH); % call base constructor
            obj@wirelessWaveformApp.nr5G_PUCCH_Tab(defaultConfig, nvargs.InvisibleEntriesPUCCH); % call base constructor
            obj@wirelessWaveformApp.nr5G_SRS_Tab(defaultConfig, nvargs.InvisibleEntriesSRS); % call base constructor
        end

        function names = figureNames(~)
            names = {'Main', 'PUSCH', 'PUCCH', 'SRS'};
        end

        function tag = figTag(~, name)
            switch name
                case 'Main'
                    tag = 'mainULFig';
                case 'PUSCH'
                    tag = 'puschFig';
                case 'PUCCH'
                    tag = 'pucchFig';
                case 'SRS'
                    tag = 'srsFig';
            end
        end

        function createFigures(obj)
            % Create figures for all 3 tabs:
            obj.createFig('PUSCH',                              'pxschFig',               figTag(obj,'PUSCH'));
            obj.createFig('PUCCH',                              'pucchFig');
            obj.createFig('SRS',                                'srsFig');

            % Create the figures for the side panels
            appObj = obj.getParent.AppObj;
            obj.createFig('PUSCH 1 - Advanced Configuration',   'pxschSingleChannelFig',  obj.pxschExtraFigTag);
            appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", obj.pxschExtraFigTag).Visible = false;
            obj.createFig('PUCCH 1 - Advanced Configuration',    'pxcchSingleChannelFig');
            appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", obj.pxcchExtraFigTag).Visible = false;
            obj.createFig('SRS 1 - Advanced Configuration',    'xrsSingleChannelFig');
            appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", obj.xrsExtraFigTag).Visible = false;
        end

        function setExtraConfigFigVisibility(obj, visibility)
            % This method turns on or off all tabs if the waveform type changes
            % (e.g., Full DL -> WLAN)

            appObj = obj.getParent.AppObj;
            % exiting this waveform type, need to close documents
            appObj.pParameters.Layout.Parent = appObj.pParametersFig; % otherwise closing Main tab will kill Accordion panels

            names = figureNames(obj);
            for idx = 1:numel(names)
                doc = appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", figTag(obj,names{idx}));
                doc.Visible = visibility;
            end

            doc = appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", 'puschSingleChannelFig');
            doc.Visible =  visibility && any(strcmpi(obj.lastFigInFocus, obj.PXSCHfigureName));

            doc = appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", 'pxcchSingleChannelFig');
            doc.Visible =  visibility && any(strcmpi(obj.lastFigInFocus, 'PUCCH'));

            doc = appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", 'xrsSingleChannelFig');
            doc.Visible =  visibility && any(strcmpi(obj.lastFigInFocus, 'SRS'));
        end

        function cleanupDlg(obj)
            % Dialog-specific cleanup when app is closing
            % Custom 5G UL has additional UI objects that need deletion when the
            % app is closing.
            delete([obj.pxschFig; obj.pxschSingleChannelFig; obj.pucchFig; obj.pxcchSingleChannelFig; obj.srsFig])

            % Close the intra-cell guard band configuration pop-up window if the
            % app is closing
            delete(obj.icgbFig);
        end

        function extraPanels = getExtraPanels(obj)
            extraPanels = {};
            extraPanels = cat(1,extraPanels,mat2cell(obj.pxschGridLayout.Children(:),ones(numel(obj.pxschGridLayout.Children(:)),1)));
            extraPanels = cat(1,extraPanels,mat2cell(obj.pucchGridLayout.Children(:),ones(numel(obj.pucchGridLayout.Children(:)),1)));
            extraPanels = cat(1,extraPanels,mat2cell(obj.srsGridLayout.Children(:),ones(numel(obj.srsGridLayout.Children(:)),1)));
            if isKey(obj.getParent.DialogsMap, obj.getClassNamePUCCHAdv) && isgraphics(obj.getParent.DialogsMap(obj.getClassNamePUCCHAdv).getPanels)
                dlgAdv{1,1} = obj.getParent.DialogsMap(obj.getClassNamePUCCHAdv);
                dlgAdv{2,1} = obj.getParent.DialogsMap(obj.getClassNamePUCCHUCI);
                extraPanels = cat(1,extraPanels,dlgAdv);
            end
            if isKey(obj.getParent.DialogsMap, obj.getClassNameSRSAdv) && isgraphics(obj.getParent.DialogsMap(obj.getClassNameSRSAdv).getPanels)
                dlgAdv{1,1} = obj.getParent.DialogsMap(obj.getClassNameSRSAdv);
                extraPanels = cat(1,extraPanels,dlgAdv);
            end
        end

        function cfg = getConfiguration(obj)
            % Map all graphical content into an equivalent nrULCarrierConfig object

            % Top level:
            cfg = obj.DefaultCfg; % default nrULCarrierConfig object
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

            % Intra-cell Guard Bands
            if ~isempty(obj.icgbCache)
                cfg.IntraCellGuardBands = obj.icgbCache;
            else
                cfg.IntraCellGuardBands = {nrIntraCellGuardBandsConfig};
            end

            % PUSCH
            cfg.PUSCH = getWaveConfig(obj, 'pusch');

            % PUCCH
            cfg.PUCCH = getWaveConfig(obj, 'pucch');

            % SRS
            cfg.SRS = getWaveConfig(obj, 'srs');
        end

        function markBrokenBWPLinks(obj)
            % paint red the rows that do not link to a valid BWP
            brokenLinks = struct('pusch',[],'pucch',[],'srs',[]);
            chNames = fieldnames(brokenLinks);
            brokenLinks = getBrokenLinks(obj, 'bwp', chNames, 'BandwidthPartID', brokenLinks);
            markBrokenLinks(obj, brokenLinks);
        end

        function adjustDialog(obj)
            obj.getParent.AppObj.pParametersFig.Tag = 'WavegenFigUL'; % For testing, to avoid conflict with DL
        end

        function restoreDefaults(obj)
            % Disable visibility of side panels
            obj.pxschExtraTile = false;

            % Get the default wave configuration object and apply it to the
            % app
            cfg = obj.DefaultCfg;
            applyConfiguration(obj, cfg);
        end

        function applyConfiguration(obj, waveConfig)
            % Used on New, Open Session, openInGenerator, and in displaying
            % the configuration in WWA

            % Main tab
            applyConfigSCSCarriers(obj, waveConfig);
            applyConfigBWP(obj, waveConfig);
            applyICGBConfig(obj, waveConfig);

            % Make sure all SCS-related broken links are correctly reported
            brokenLinks.bwp = [];
            brokenLinks = getBrokenLinks(obj, 'scscarriers', 'bwp', 'SubcarrierSpacing', brokenLinks);
            markBrokenLinks(obj, brokenLinks);

            % Channels
            applyConfigPXSCH(obj, waveConfig);
            applyConfigPUCCH(obj, waveConfig);
            applyConfigSRS(obj, waveConfig);
        end

        function w = getColumnWeights(~, ~)
            % The right-side column (PXSCH/PUCCH Advanced config or Spectrum Analyzer)
            % is a bit smaller.
            c = 0.757;
            w = [nan; c; 1-c];
        end

    end

    % Protect methods for derived classes
    methods (Access = protected)

        %% Visualization
        function markAllBrokenLinks(obj)
            % Update all broken links by highlighting red color in invalid rows
            brokenLinks = struct('pusch',[],'pucch',[],'srs',[]);
            brokenLinks = getBrokenLinks(obj, 'bwp', {'pusch', 'pucch', 'srs'}, 'BandwidthPartID', brokenLinks); % BWP ID in PUSCH, PUCCH, SRS table
            markBrokenLinks(obj, brokenLinks);
        end

        % Link-specific actions when FR changes
        function frChangedForLink(obj)

            isFR2 = strcmpi(obj.FrequencyRange,'FR2');

            % Update ICGB configure button visibility
            if ~isempty(obj.icgbConfigButton)
                set([obj.icgbConfigButton],'Visible',uiservices.logicalToOnOff(~isFR2));
            end

            % Update PUSCH table and side panel
            if sidePanelExists(obj,'pusch')
                currPXSCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pxschTable.Selection);
                if isFR2 && obj.pxschWaveConfig{currPXSCH}.Interlacing
                    % Only update table and side panel if it's FR2 and the
                    % current PUSCH configuration is interlaced
                    obj.pxschWaveConfig{currPXSCH}.Interlacing = false;
                end

                % Map the channel configuration to the PUSCH table
                applyConfiguration(obj.pxschTable, obj.pxschWaveConfig);

                % Update the side panel
                mapCache2SidePanelPXSCH(obj);
                updateControlsVisibilityPXSCH(obj);
            end

            % Update PUCCH table and side panel
            if sidePanelExists(obj,'pucch')
                currPUCCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pucchTable.Selection);
                if isFR2 && nr5g.internal.interlacing.isInterlaced(obj.pucchWaveConfig{currPUCCH})
                    % Only update table and side panel if it's FR2 and the
                    % current PUSCH configuration is interlaced
                    obj.pucchWaveConfig{currPUCCH}.Interlacing = false;
                end

                % Map the channel configuration to the PUCCH table
                applyConfiguration(obj.pucchTable, obj.pucchWaveConfig);

                % Update the side panel
                mapCache2SidePanelPUCCH(obj);
                updateControlsVisibilityPUCCH(obj);
            end
        end
    end

end

function waveCfg = getDefaultConfig()
    % Get the default configuration object from nrULCarrierConfig
    waveCfg = nrULCarrierConfig;
    waveCfg.Label = 'Carrier1';

    % One difference is that carriers are prepared for equal span in
    % different SCS (maximum occupancy).
    def15kHzNRB = 270;
    waveCfg.SCSCarriers{1}.NSizeGrid   = def15kHzNRB;
    waveCfg.SCSCarriers{1}.NStartGrid  = 3;
    waveCfg.BandwidthParts{1}.NSizeBWP = def15kHzNRB;
    waveCfg.BandwidthParts{1}.NStartBWP = 3;

    waveCfg.PUSCH{1}.PRBSet = 0:(def15kHzNRB-1);
    waveCfg.PUSCH{1}.EnablePTRS = logical(waveCfg.PUSCH{1}.EnablePTRS);
    waveCfg.PUSCH{1}.TransformPrecoding = logical(waveCfg.PUSCH{1}.TransformPrecoding);

    waveCfg.PUCCH{1}.Label = 'PUCCH1';
end