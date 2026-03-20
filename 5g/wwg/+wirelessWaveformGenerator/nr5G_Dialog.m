classdef nr5G_Dialog < wirelessWaveformGenerator.WaveformConfigurationDialog & ...
        wirelessWaveformApp.nr5G_Main_Base_Dialog
    %
    
    %   Copyright 2018-2024 The MathWorks, Inc.
    properties
        ResourceGridLayout
    end
    
    methods (Static)
        function hPropDb = getPropertySet(~)
            hPropDb = extmgr.PropertySet(...
                'Visualizations',   'mxArray', {'Resource Grid (BWP#1)', 'Channel Bandwidth View', 'Resource Element Power (BWP#1)'});
        end
    end
    
    methods (Abstract)
        % Any 5G subclass must implement these methods:
        frChanged(obj,~);
        frChangedGUI(obj,~);
    end
    
    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_Dialog(parent)
            obj@wirelessWaveformGenerator.WaveformConfigurationDialog(parent); % call base constructor
            obj@wirelessWaveformApp.nr5G_Main_Base_Dialog(parent);
            obj.CCDFLegendChannelName = getString(message('nr5g:waveformGeneratorApp:CCDFLegendChannelName5G'));
        end
        
        function setupDialog(obj)
            setupDialog@wirelessWaveformGenerator.WaveformConfigurationDialog(obj);
            genDialog = getGenerationDialog(obj);
            if ~isempty(genDialog)
                genDialog.InputSource = 'PN9';  % repeatable setting for T&M
                genDialog.layoutUIControls();
            end
        end
        
        function helpCallback(~)
            helpview('5g', 'FiveGWaveformGenerator-app');
        end
        
        function b = exportsMLCode(~)
            b = true;
        end
        
        function adjustSpec(obj)
            % customization given the max label length
            obj.LabelWidth = 120;
        end
        
        function adjustDialog(obj)
            % Make sure the Label tag is unique
            adjustDialog@wirelessWaveformApp.nr5G_Main_Base_Dialog(obj);
        end
        
        function defaultVisualLayout(obj)
            obj.setVisualState('Resource Grid (BWP#1)', true);
            obj.setVisualState('Channel Bandwidth View', false);
            obj.setVisualState('Resource Element Power (BWP#1)', false);
        end
        
        function tag = getFigureTag(obj, visualName)
            if strcmp(visualName, 'Resource Element Power (BWP#1)')
                tag = 'ResourceElementPower';
            else
                % Figure Tag, remove spaces
                tag = getFigureTag@wirelessWaveformGenerator.WaveformConfigurationDialog(obj, visualName);
            end
            % remove '#', '(', ')' characters from field names of figures; can't
            % be saved in structure during Save Session
            tag = replace(tag, {'(',')','#'}, '');
        end
        
        function restoreDefaults(obj)
            % Label is taken care of in nr5G_DL_dialog and nr5G_UL_Dialog
            restoreDefaults@wirelessWaveformApp.nr5G_Main_Base_Dialog(obj);
        end
        
        function figureAdded(obj,figName)
            % This method initializes/updates the figure (for example the Resource
            % Grid and the Channel Bandwidth View, which do not require waveform generation)
            % before it is added to the current visuals
            if contains(figName(isletter(figName)),'ChannelBandwidthView') % Tags do not contain spaces
                % Initialize channel bandwidth view
                updateChannelBandwidthView(obj);
            elseif contains(figName(isletter(figName)),'ResourceGrid') % Tags do not contain spaces
                % Initialize resource grid
                updateGrid(obj);
            elseif contains(figName(isletter(figName)),'ResourceElementPower') % Tags do not contain spaces
                % Initialize Resource Element Power
                updateResourceElementPower(obj);
            end
        end
        
        function sr = getSampleRate(obj)
            sr = getSampleRate@wirelessWaveformApp.nr5G_Main_Base_Dialog(obj);
        end
        
        function str = getSampleRateStr(~)
            str = 'info.ResourceGrids(1).Info.SampleRate';
        end
        
        function [configline, configParam] = getConfigParam(obj)
            configline = '';
            configParam = obj.configGenVar;
        end
        
        function userDataText = getUserDataText(~)
            userDataText = 'configuration';
        end
        
        function AppDochyperlink = getAppLink(~)
            AppDochyperlink = '<a href="matlab:helpview(''5g'',''FiveGWaveformGenerator_app'')">5G Waveform Generator</a>';
        end
        
        function BlockDochyperlink = getBlockLink(~)
            BlockDochyperlink = 'helpview(''5g'',''FiveGWaveformGenerator_block'')';
        end
        
        function b = hasWindowing(~)
            % Remove Tukey windowing from Transmitter tab for all 5G waveforms
            b = true;
        end
        
        function str = getCustomFilterStr(obj)
            str = getBasicLPFilterStr(obj);
        end
        
        function str = getCatalogPrefix(~)
            str = 'nr5g:waveformApp:';
        end
        
        %% Visualization
        function customVisualizations(obj, varargin)
            % Executes upon waveform generation to update all visuals
            
            % Resource Grid, channel bandwidth view and Resource Element Power
            updateGrid(obj);
            updateChannelBandwidthView(obj);
            updateResourceElementPower(obj);
        end
        
        function resetCustomVisuals(obj)
            % also executed during New session, to initialize desired state of visuals
            if ~(isprop(obj, 'Initializing') && obj.Initializing)
                resetResourceGridAxes(obj);
                updateREVisual(obj, 'Main', 'reset');
                updateGrid(obj);
                updateChannelBandwidthView(obj);
            end
        end
        
        function outro(obj)
            
            resetCustomVisuals(obj);

            % Restore generic visualization options:
            visualizeBtn = find(obj.Parent.AppObj.pPrimaryTab, 'plots');
            visualizeBtn.DynamicPopupFcn =  @(a, b) updateScopeOptions(obj.Parent.AppObj, []);
        end
        
        function b = mayHaveEmptyTimePeriods(~)
            b = true;
            % TDD mode, and partial temporal PDSCH allocation (either in slots or symbols)
            % may cause a bad spectrum plot in the end
        end
        
        function o = offersCCDF(~)
            o = true;
        end
        
        function parent = getParent(obj)
            parent = obj.Parent;
        end
        
        %% MATLAB Code generation
        function addGenerationCode(obj, sw)
            % The waveform generation command
            addcr(sw, ['[waveform,info] = nrWaveformGenerator(' obj.configGenVar ');']);
        end
        
        function props = props2ExcludeFromConfigGeneration(~)
            % Interface differs between GUI and programmatic API
            props = {'WindowingSource', 'WindowingPercent', 'SampleRateSource', 'SampleRate', ...
                'PhaseCompensation', 'CarrierFrequency'};
        end
        
        function addConfigCode(obj,sw)
            % Get waveform configuration. This is is an nrDLCarrierConfig for
            % 5G DL or a nrULCarrierConfig for 5G UL
            carrierCfg = getConfiguration(obj);
            
            mCodeCfg = getWavegenMCodeGenerationStructure(carrierCfg,obj.configGenVar);
            wirelessWaveformGenerator.internal.add5GCarrierConfigMCode(sw,carrierCfg,mCodeCfg);
        end
    end
    
    % Methods specific to this class and that are currently used by
    % neighboring classes
    methods (Access = public)
        function [wgc,gridset,waveResources] = updateGrid(obj, varargin)
            % Live grid update when a relevant property changes or waveform is
            % generated
            gridset = [];
            waveResources = [];
            wgc = [];
            resetStatus = true;
            channelName = obj.ChannelName;
            if isempty(obj.visualStates)
                % App initialization
                return;
            end

            if nargin==1
                try
                    wgc = getConfiguration(obj);
                catch e
                    % Update error message
                    updateConfigDiagnostic(obj, e.message);
                    return;
                end
            else
                wgc = varargin{1};
                if nargin > 2
                    resetStatus = varargin{2};
                    if nargin > 3
                        channelName = varargin{3};
                    end
                end
            end
            try
                [gridset,msg,waveResources] = wirelessWaveformGenerator.internal.computeResourceGridPRB(wgc);

                % Update BWP grids and textbox callback
                updateBWPGrids(obj,wgc,gridset,waveResources);
                updateREVisual(obj, channelName);

                if ~isempty(msg.message)
                    updateConfigDiagnostic(obj, msg.message, MessageType=msg.type);
                elseif resetStatus
                    updateConfigDiagnostic(obj, ""); % Clear any configuration-related message
                end

            catch e
                updateConfigDiagnostic(obj, e.message);
            end
        end
        
        function updateChannelBandwidthView(obj)
            % Live channel bandwidth view update when a relevant property changes or waveform
            % is generated
            
            if isprop(obj, 'Initializing') && obj.Initializing
                return; % will do only 1 plot at extensionTypeChange->clearScopes->resetCustomVisuals
            end
            
            if isempty(obj.visualStates)
                % App initialization
                return;
            end
            
            try
                % Channel Bandwidth View:
                if obj.getVisualState('Channel Bandwidth View')
                    cfg = getConfiguration(obj);
                    ax = obj.getVisualAxes('Channel Bandwidth View');
                    wirelessWaveformGenerator.internal.plotCarriers(ax, cfg);
                end
            catch e
                updateConfigDiagnostic(obj, e.message);
            end
        end
    end
    
    % Private methods
    methods (Access = private)
        function updateBWPGrids(obj,wgc,gridset,waveResources)
            
            % Modify channel instance names if needed for text box display
            waveResources = updateChannelNames(obj,waveResources);
            
            % Update all activated BWP grids
            for bwpIdx = 1:length(wgc.BandwidthParts)
                ax = getResourceGridAxes(obj,wgc.BandwidthParts{bwpIdx}.BandwidthPartID);
                if ~isempty(ax)
                    wirelessWaveformGenerator.internal.plotResourceGrid(ax, wgc, gridset,obj.isDownlink, bwpIdx);
                    updateResourceGridTextBoxCallback(ax,wgc,waveResources);
                end
            end
        end
        
        function updateResourceElementPower(obj)
            
            % Resource Element Power
            if obj.getVisualState('Resource Element Power (BWP#1)')
                ax = obj.getVisualAxes('Resource Element Power (BWP#1)');
                % The RE power is only for 5G presets, so only one BWP exists
                wirelessWaveformGenerator.internal.plotResourceElementPower(ax, obj.gridSet)
            end
            
        end
    end
    
    % Protect methods for derived classes
    methods (Access=protected)
        
        % Set channel data source from the associated dialog
        function cfg = getConfigurationDataSource(obj,dschannel,cfg)
            chnumber = 1;
            genDialog = obj.getGenerationDialog;
            if ~isempty(genDialog)
                if any(strcmp(genDialog.InputSource, {'PN9', 'PN15'}))
                    cfg.(dschannel){chnumber}.DataSource = genDialog.InputSource; % PN9 or PN15 - geared for the FRC preset sources only
                else
                    cfg.(dschannel){chnumber}.DataSource = genDialog.InputValue; % User defined
                end
            end
        end
        
        function ax = getResourceGridAxes(obj,bwpID)
            
            ax = [];
            name = ['Resource Grid (BWP#' num2str(bwpID) ')'];
                numBWP = obj.maxNumBWPFigs;

            if bwpID<=numBWP && obj.getVisualState(name)
                ax = obj.getVisualAxes(name);
            end
            
            if ~isempty(ax) && isa(ax(1).Parent,'matlab.ui.Figure')
                obj.ResourceGridLayout = uigridlayout(ax.Parent,RowHeight={'fit','1x'},ColumnWidth={'1x',0,0});
                obj.ResourceGridLayout.ColumnSpacing = 0;
                ax.Padding = 'compact';
                ax.Parent = obj.ResourceGridLayout;
                ax.Layout.Row = [1 2];

                % Create an empty resource grid plot
                wirelessWaveformGenerator.internal.plotResourceGrid(ax,[],[],obj.isDownlink, []);
            end

            for axIdx = 1:numel(ax)
                if ~contains(ax(axIdx).Tag,'REAxes')
                    % We are interested in is the axes containing the
                    % resource grid and not that containing the RE
                    % mapping visualization.
                    break;
                end
            end
            ax = ax(axIdx);
            
        end
        
        % Delete all contents of the resource grid axes.
        function resetResourceGridAxes(obj)
            
                numBWP = obj.maxNumBWPFigs;
            
            % Loop over all BWP figures and delete their contents.
            for id = 1:numBWP
                ax = getResourceGridAxes(obj,id);
                if ~isempty(ax)
                    % Create an empty resource grid plot
                    wirelessWaveformGenerator.internal.plotResourceGrid(ax,[],[],obj.isDownlink,[]);
                end
            end
        end
        
        % Modify channel names in waveform resources for display in the PRB
        % resource grid text box. This method allows to customize the text
        % displayed in the text box for each channel. Subclasses may redefine
        % this method to adapt the text to their needs.
        function waveResources = updateChannelNames(~,waveResources)
        end
        
    end
    
end

function mcodeCfg = getWavegenMCodeGenerationStructure(waveCfgObj,configGenVar)
    % Create a default MATLAB code generation structure
    mcodeCfg = wirelessWaveformGenerator.internal.mCodeGenerationConfig(waveCfgObj);
    
    % Variable name of nrXLCarrierConfig object is configured by the
    % configGenVar property
    mcodeCfg.VarName = configGenVar;
    
    % Specify headers and variable names for each object property
    % Add multiple lines by concatenating strings if necessary
    mcodeCfg.SCSCarriers{1}.SectionHeader = "%% SCS specific carriers";
    mcodeCfg.SCSCarriers{1}.InstanceHeader = "% Carrier";
    mcodeCfg.SCSCarriers{1}.VarName = 'scscarrier';
    
    mcodeCfg.BandwidthParts{1}.SectionHeader =  "%% Bandwidth Parts";
    mcodeCfg.BandwidthParts{1}.InstanceHeader = "% BWP";
    mcodeCfg.BandwidthParts{1}.VarName = 'bwp';
    
    if isa(waveCfgObj,'nrDLCarrierConfig')
        mcodeCfg.SSBurst.SectionHeader = "%% Synchronization Signals Burst";
        mcodeCfg.SSBurst.VarName = 'ssburst';
        
        mcodeCfg.CORESET{1}.SectionHeader = "%% CORESET and Search Space Configuration";
        mcodeCfg.CORESET{1}.InstanceHeader = "% CORESET";
        mcodeCfg.CORESET{1}.VarName = 'coreset';
        
        mcodeCfg.SearchSpaces{1}.SectionHeader = "% Search Spaces";
        mcodeCfg.SearchSpaces{1}.InstanceHeader = "% Search Space";
        mcodeCfg.SearchSpaces{1}.VarName = 'searchspace';
        
        mcodeCfg.PDCCH{1}.SectionHeader = "%% PDCCH Instances Configuration";
        mcodeCfg.PDCCH{1}.InstanceHeader = "% PDCCH";
        mcodeCfg.PDCCH{1}.VarName = 'pdcch';
        
        mcodeCfg.PDSCH{1}.SectionHeader = "%% PDSCH Instances Configuration";
        mcodeCfg.PDSCH{1}.InstanceHeader = "% PDSCH";
        mcodeCfg.PDSCH{1}.VarName = 'pdsch';
        
        mcodeCfg.PDSCH{1}.DMRS.SectionHeader = "% PDSCH DM-RS";
        mcodeCfg.PDSCH{1}.DMRS.VarName = 'DMRS';
        mcodeCfg.PDSCH{1}.DMRS.IncludeParentName = true;
        
        mcodeCfg.PDSCH{1}.PTRS.SectionHeader = "% PDSCH PT-RS";
        mcodeCfg.PDSCH{1}.PTRS.VarName = 'PTRS';
        mcodeCfg.PDSCH{1}.PTRS.IncludeParentName = true;
        
        mcodeCfg.PDSCH{1}.ReservedPRB{1}.SectionHeader = "% PDSCH Reserved PRB";
        mcodeCfg.PDSCH{1}.ReservedPRB{1}.VarName = 'ReservedPRB';
        mcodeCfg.PDSCH{1}.ReservedPRB{1}.IncludeParentName = true;
        
        mcodeCfg.CSIRS{1}.SectionHeader = "%% CSI-RS Instances Configuration";
        mcodeCfg.CSIRS{1}.InstanceHeader = "% CSI-RS";
        mcodeCfg.CSIRS{1}.VarName = 'csirs';
        
    else % UL
        
        mcodeCfg.IntraCellGuardBands{1}.SectionHeader = "%% Intracell Guard Bands Configuration";
        mcodeCfg.IntraCellGuardBands{1}.InstanceHeader = "% IntraCellGuardBands";
        mcodeCfg.IntraCellGuardBands{1}.VarName = 'gb';
        
        
        mcodeCfg.PUSCH{1}.SectionHeader = "%% PUSCH Instances Configuration";
        mcodeCfg.PUSCH{1}.InstanceHeader = "% PUSCH";
        mcodeCfg.PUSCH{1}.VarName = 'pusch';
        
        mcodeCfg.PUSCH{1}.DMRS.SectionHeader = "% PUSCH DM-RS";
        mcodeCfg.PUSCH{1}.DMRS.VarName = 'DMRS';
        mcodeCfg.PUSCH{1}.DMRS.IncludeParentName = true;
        
        mcodeCfg.PUSCH{1}.PTRS.SectionHeader = "% PUSCH PT-RS";
        mcodeCfg.PUSCH{1}.PTRS.VarName = 'PTRS';
        mcodeCfg.PUSCH{1}.PTRS.IncludeParentName = true;
        
        mcodeCfg.PUCCH{1}.SectionHeader = "%% PUCCH Instances Configuration";
        mcodeCfg.PUCCH{1}.InstanceHeader = "% PUCCH";
        mcodeCfg.PUCCH{1}.VarName = 'pucch';
        
        mcodeCfg.SRS{1}.SectionHeader = "%% SRS Instances Configuration";
        mcodeCfg.SRS{1}.InstanceHeader = "% SRS";
        mcodeCfg.SRS{1}.VarName = 'srs';
        
    end
end

% Update resource grid text box callback with new waveform resources
function updateResourceGridTextBoxCallback(ax,wgc,waveResources)
    
    % Update resource grid text box callback with conflicts
    fig = ax.Parent.Parent;
    textBoxCallbackFun = @wirelessWaveformGenerator.internal.resourceGridTextboxCallback;
    fig.WindowButtonMotionFcn = {textBoxCallbackFun,wgc,waveResources};
    
end
