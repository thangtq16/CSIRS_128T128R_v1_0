classdef nr5G_UL_Dialog < wirelessWaveformGenerator.nr5G_Full_Base_Dialog & ...
        wirelessWaveformApp.nr5G_UL_Tabs
    % Fully-customizable 5G Uplink extension for Wireless Waveform Generator App
    
    %   Copyright 2020-2024 The MathWorks, Inc.
    
    properties (Hidden)
        TitleString = getString(message('nr5g:waveformApp:nrULTitle'))
        
        mainULFig
        ssBurstFig
        InvisiblePUSCHEntries = {}
        InvisiblePUCCHEntries = {}
    end
    
    methods (Static)
        function b = hasLeftFigurePanel(~)
            % Left-side panel can be avoided during launch
            b = false;
        end
        function tag = paramFigureTag(~)
            tag = 'mainULFig';
        end
    end
    
    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_UL_Dialog(parent)
            obj@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(parent, false); % call base constructor
            obj@wirelessWaveformApp.nr5G_UL_Tabs(); % call base constructor

            % Initialize value of cached configuration object
            obj.cachedCfg = obj.DefaultCfg;
            
            %% Listeners
            % Create listeners for knowing when any of the configurations stored
            % in table objects changes
            arrayfun(@(x)addlistener(obj.(x),'TableChanged',@(src,event)obj.tableChanged(src,event)),obj.tableObjName);
            arrayfun(@(x)addlistener(obj.(x),'Selection','PostSet',@(src,event)obj.tableChanged(src,event)),obj.tableObjName);
        end
        
        function cleanupDlg(obj)
            % Dialog-specific cleanup when app is closing
            % Custom 5G UL has additional UI objects that need deletion when the
            % app is closing.
            cleanupDlg@wirelessWaveformApp.nr5G_UL_Tabs(obj);
        end
        
        function extraPanels = getExtraPanels(obj)
            extraPanels = getExtraPanels@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(obj);
            extraPanelsUL = getExtraPanels@wirelessWaveformApp.nr5G_UL_Tabs(obj);
            extraPanels = cat(1,extraPanels,extraPanelsUL);
        end
        
        function setupDialog(obj)
            % Actions performed when entering this Full UL wavegen extension
            
            % Call baseclass method
            setupDialog@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(obj);
        end
        
        function cfg = getConfiguration(obj)
            % Map all graphical content into an equivalent nrULCarrierConfig object
            
            % Call base-class
            cfg = getConfiguration@wirelessWaveformApp.nr5G_UL_Tabs(obj);
        end
        
        function str = getIconDrawingCommand(obj)
            str = ['disp([''Freq. range: '' ' obj.configGenVar '.FrequencyRange newline ...' newline ...
                '''Bandwidth: '' num2str(' obj.configGenVar '.ChannelBandwidth) '' MHz'' newline ...' newline ...
                '''Subframes: '' num2str(' obj.configGenVar '.NumSubframes) ]);'];
        end
        
        function [blockName, maskTitleName, waveNameText] = getMaskTextWaveName(obj)
            blockName = ['5G NR ' obj.Parent.WaveformGenerator.pCurrentExtensionType];
            maskTitleName = ['5G NR ' obj.Parent.WaveformGenerator.pCurrentExtensionType ' Waveform Generator'];
            waveNameText = blockName;
        end
        
        function adjustSpec(obj)
            adjustSpec@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(obj);
            
            obj.panelFixedSize = true; % prevents panel containing top-level
            % properties (NumSubframes etc.) from
            % filling horizontal space and covering
            % SCS, BWP tables
            
            % For MATLAB Script generation:
            obj.configGenFcn  = @nrULCarrierConfig;
            obj.configGenVar  = 'cfgUL';
        end
        
        function adjustDialog(obj)
            adjustDialog@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(obj);
            adjustDialog@wirelessWaveformApp.nr5G_UL_Tabs(obj);
        end
        
        function restoreDefaults(obj)
            restoreDefaults@wirelessWaveformApp.nr5G_UL_Tabs(obj);
        end
        
        function applyConfiguration(dlg, waveConfig)
            % Used on New, Open Session, and openInGenerator.
            
            % Map an nrULCarrierConfig object to all tables and UI elements.
            % Adjust the elements of the configuration that are not supported.
            waveConfig = applyConfiguration@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(dlg, waveConfig);
            applyConfiguration@wirelessWaveformApp.nr5G_UL_Tabs(dlg, waveConfig);
            
        end
        
        function w = getColumnWeights(obj, numScopes)
            % The right-side column (PXSCH/PUCCH Advanced config or Spectrum Analyzer)
            % is a bit smaller.
            w = getColumnWeights@wirelessWaveformApp.nr5G_UL_Tabs(obj,numScopes);
        end
        
        function parent = getParent(obj)
            % needed for distributed classes to fetch protected property / method
            parent = obj.Parent;
        end

        function childDialogs = getDialogs2Reset(obj)
          childDialogs = getDialogs2Reset@wirelessWaveformGenerator.nr5G_Full_Base_Dialog(obj);
          dialogsMap = obj.getParent.DialogsMap;
          if ~isempty(obj.paramPUCCH)
            childDialogs = [childDialogs {dialogsMap(obj.classNamePUCCHAdv) dialogsMap(obj.classNamePUCCHUCI)}];
          end
          if ~isempty(obj.paramPXSCH)
            childDialogs = [childDialogs  {dialogsMap(obj.classNamePXSCHAdv) ...
              dialogsMap(obj.classNamePXSCHDMRS) dialogsMap(obj.classNamePXSCHPTRS)}];
          end
          if ~isempty(obj.paramSRS)
            childDialogs = [childDialogs {dialogsMap(obj.classNameSRSAdv)}];
          end
        end
    end
    
    % Protect methods for derived classes
    methods (Access = protected)
        
        %% Visualization
        function markAllBrokenLinks(obj)
            % Update all broken links by highlighting red color in invalid rows
            markAllBrokenLinks@wirelessWaveformApp.nr5G_UL_Tabs(obj);
        end
        
        % Link-specific actions when FR changes
        function frChangedForLink(obj)
            frChangedForLink@wirelessWaveformApp.nr5G_UL_Tabs(obj);
        end
    end
    
end
