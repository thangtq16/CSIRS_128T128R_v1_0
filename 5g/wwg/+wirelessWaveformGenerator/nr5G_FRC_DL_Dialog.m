classdef nr5G_FRC_DL_Dialog < wirelessWaveformGenerator.nr5G_FRC_Dialog
    %
    
    %   Copyright 2019-2025 The MathWorks, Inc.
    
    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
        FRC
        MCS
    end
    
    properties (Hidden, Dependent)
        Modulation
        TargetCodingRate
    end
    
    properties (Hidden)
        TitleString = getString(message('nr5g:waveformApp:nrDLFRCTitle'))
        
        infoClass = 'wirelessWaveformGenerator.nr5G_DL_InfoDialog'
        OCNGType = 'checkbox'
        OCNGGUI
        OCNGLabel

        MCSType = 'charPopup'
        MCSDropDown = ""
        MCSGUI
        MCSLabel
        
        % Properties used to open a copy of this preset definition in a
        % Downlink/Uplink waveform type
        ThisWaveformType = 'Downlink FRC';
        NewWaveformType = 'Downlink';

        ReferenceSubcarrierSpacingType = 'numericPopup'
        ReferenceSubcarrierSpacingDropDown = {'15','30','60','120'};
        ReferenceSubcarrierSpacingLabel
        ReferenceSubcarrierSpacingGUI
        TransmissionPeriodicityType = 'numericPopup'
        TransmissionPeriodicityDropDown = "";
        TransmissionPeriodicityLabel
        TransmissionPeriodicityGUI
        NumDownlinkSlotsType = 'numericEdit'
        NumDownlinkSlotsLabel
        NumDownlinkSlotsGUI
        NumUplinkSlotsType = 'numericEdit'
        NumUplinkSlotsLabel
        NumUplinkSlotsGUI
        EnableSpecialSlotsType = 'checkbox'
        EnableSpecialSlotsLabel
        EnableSpecialSlotsGUI
        NumDownlinkSymType = 'numericEdit'
        NumDownlinkSymLabel
        NumDownlinkSymGUI
        NumUplinkSymType = 'numericEdit'
        NumUplinkSymLabel
        NumUplinkSymGUI
        NumSpecialSlotsType = 'charText'
        NumSpecialSlotsLabel
        NumSpecialSlotsGUI
        TDDSlotAllocationType = 'charText'
        TDDSlotAllocationLabel
        TDDSlotAllocationGUI
    end

    properties (Access = private)
        pUpdateTDD % Indicate whether TDD GUI parameters should be overwritten by default FRC values
    end
    
    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_FRC_DL_Dialog(parent)
            obj@wirelessWaveformGenerator.nr5G_FRC_Dialog(parent, true); % call base constructor

            % Add info dialog to map
            if ~isKey(obj.Parent.DialogsMap, obj.infoClass)
                obj.Parent.DialogsMap(obj.infoClass) = eval([obj.infoClass '(obj.Parent)']);
            end
            
            % Add callbacks
            obj.MCSGUI.(obj.Callback)     = @(src,evnt) mcsChanged(obj, src);
            obj.NLayersGUI.(obj.Callback) = @(src,evnt) layersChanged(obj, src);  % Dialog specific callback
            obj.OCNGGUI.(obj.Callback)    = @(src,evnt) OCNGChanged(obj);

            obj.ReferenceSubcarrierSpacingGUI.(obj.Callback) = @(src,evnt) referenceSCSChanged(obj,src);
            obj.TransmissionPeriodicityGUI.(obj.Callback) = @(src,evnt) transmissionPeriodicityChanged(obj,src);
            obj.NumDownlinkSlotsGUI.(obj.Callback)        = @(src,evnt) numDownlinkSlotsChanged(obj,src,evnt);
            obj.NumDownlinkSymGUI.(obj.Callback)          = @(src,evnt) numDownlinkSymChanged(obj,src,evnt);
            obj.NumUplinkSlotsGUI.(obj.Callback)          = @(src,evnt) numUplinkSlotsChanged(obj,src,evnt);
            obj.NumUplinkSymGUI.(obj.Callback)            = @(src,evnt) numUplinkSymChanged(obj,src,evnt);
            obj.EnableSpecialSlotsGUI.(obj.Callback)      = @(src,evnt) enableSpecialSlotsChanged(obj,src);

            DuplexModeChanged(obj);

            % Update info
            obj.updateInfo();
        end

        function cellDialogs = getDialogsPerColumn(obj)
            cellDialogs{1} = {obj obj.Parent.GenerationDialog obj.Parent.FilteringDialog};
            cellDialogs{2} = {obj.Parent.DialogsMap(obj.infoClass)};
        end
        
        function adjustSpec(obj)
            obj.LabelWidth = 130;
            
            obj.MCSDropDown = wirelessWaveformApp.internal.getMCSOptionsDownlinkFRC('FR1');
            
            % For MATLAB script generation
            obj.configGenFcn  = @nrDLCarrierConfig;
            obj.configGenVar  = 'cfgDLFRC';
        end
        
        function props = displayOrder(~)
            props = {'FrequencyRange'; 'MCS'; 'SubcarrierSpacing'; 'ChannelBandwidth';...
                'NumSubframes'; 'NLayers'; 'NCellID'; 'RNTI'; 'OCNG';...
                'WindowingSource'; 'WindowingPercent'; 'SampleRateSource'; ...
                'SampleRate'; 'PhaseCompensation'; 'CarrierFrequency'; ...
                'DuplexMode'; 'EnableSpecialSlots'; 'ReferenceSubcarrierSpacing'; 'TransmissionPeriodicity'; ...
                'NumDownlinkSlots'; 'NumDownlinkSym'; 'NumUplinkSlots'; ...
                'NumUplinkSym'; 'NumSpecialSlots'; 'TDDSlotAllocation' ...
                };
        end
        
        function [blockName, maskTitleName, waveNameText] = getMaskTextWaveName(obj)
            blockName = ['5G NR ' obj.Parent.WaveformGenerator.pCurrentExtensionType];
            maskTitleName = ['5G NR ' obj.Parent.WaveformGenerator.pCurrentExtensionType ' Waveform Generator'];
            waveNameText = blockName;
        end
        
        function restoreDefaults(obj)
            obj.pUpdateTDD = false;
            obj.updateErrorCache([],[]);

            obj.FrequencyRange    = 'FR1';
            obj.MCS               = 'QPSK, R=1/3';
            obj.DuplexMode        = 'FDD';
            % Ensure that custom TDD visibility is correct
            obj.DuplexModeChanged(false);
            obj.SubcarrierSpacing = 15;
            obj.ChannelBandwidth  = 5;
            obj.NumSubframes      = 10;
            obj.NCellID           = 1;
            obj.RNTI              = 1;
            obj.OCNG              = false;  % Checkbox
            
            obj.EnableSpecialSlots = false;
            % Update available ReferenceSubcarrierSpacing options
            obj.ReferenceSubcarrierSpacingGUI.Items = wirelessWaveformGenerator.internal.getAvailableReferenceSCS(obj.FrequencyRange,obj.SubcarrierSpacing);
            obj.ReferenceSubcarrierSpacing = '15';
            % Update available TransmissionPeriodicity options
            obj.TransmissionPeriodicityGUI.Items = wirelessWaveformGenerator.internal.getAvailableTDDPeriods(obj.ReferenceSubcarrierSpacing);
            obj.TransmissionPeriodicity = '5';
            obj.NumDownlinkSlots = 3;
            obj.NumDownlinkSym = 10;
            obj.NumUplinkSlots = 1;
            obj.NumUplinkSym = 2;

            restoreMods(obj);
        end
        
        function str = getIconDrawingCommand(obj)
            str = ['disp([' obj.configGenVar '.Label newline ...' newline ...
                '''SCS: '' num2str(' obj.configGenVar '.BandwidthParts{1}.SubcarrierSpacing) '' kHz'' newline ...' newline ...
                '''Bandwidth: '' num2str(' obj.configGenVar '.ChannelBandwidth) '' MHz'' newline ...' newline ...
                '''Subframes: '' num2str(' obj.configGenVar '.NumSubframes) ]);'];
        end
        
        function cfg = getConfiguration(obj)
            isTDD = strcmpi(obj.DuplexMode,'TDD');
            customTDD = [];

            if isTDD
                customTDD = getTDDConfigGUI(obj);
            end

            refObj = wirelessWaveformApp.internal.hNRReferenceWaveformGenerator(obj.FRC, ...
                obj.ChannelBandwidth,obj.SubcarrierSpacing,obj.DuplexMode,double(obj.NCellID),[],[],obj.OCNG,customTDD);
            cfg = refObj.Config;

            % Tweak the preset parameters to reflect the additional dialog state
            cfg.NumSubframes              = double(obj.NumSubframes);
            cfg.PDSCH{1}.RNTI             = double(obj.RNTI);
            cfg.PDSCH{1}.NumLayers        = obj.NLayers;
            
            if obj.pUpdateTDD && isTDD % Update TDD GUI elements
                tddcfg = refObj.TDDConfig;
                updateTDDConfigGUI(obj,tddcfg);
            end


            % In the OCNG PDSCHs, set a different RNTI from the one used by
            % the reference PDSCH and set the same number of layers
            for i=2:length(cfg.PDSCH)
                cfg.PDSCH{i}.RNTI = mod(cfg.PDSCH{1}.RNTI + 1,65519) + 1;
                cfg.PDSCH{i}.NumLayers = cfg.PDSCH{1}.NumLayers;
            end
            
            % Set the PDSCH data source
            cfg = getConfigurationDataSource(obj,'PDSCH',cfg);
            % Set the modulation related parameters
            cfg = getConfigurationMods(obj,cfg);

        end

        function applyConfiguration(obj,cfg)
            % Used on New, Open Session, and openInGenerator
            obj.updateErrorCache([],[]);

            applyConfiguration@wirelessWaveformGenerator.nr5G_Presets_Dialog(obj,cfg);

            % Ensure that custom TDD visibility is correct
            obj.DuplexModeChanged(false);

        end

        % Properties to exclude when exporting
        function props = props2ExcludeFromConfig(~)
            % Read-only field do not get exported

            props = {'NumSpecialSlots','TDDSlotAllocation'};

        end

        function config = getConfigurationForSave(obj,~)

            if isempty(obj.pErrorCache)
                config = getConfigurationForSave@wirelessWaveformGenerator.nr5G_FRC_Dialog(obj);
            else
                % When config is invalid, do not allow saving session
                rethrow(obj.pErrorCache{end});
            end

        end

        function channelIndex = getChannelForREMapping(obj)
            try
                cfg = getConfiguration(obj);
            catch
                % Initialization; possibly inconsistent state
                channelIndex = 1;
                return;
            end
            ch = cfg.PDSCH;
             
            % Select PDSCH FRC channel (no OCNG)
            idx = cellfun(@(x) ~contains(x.Label,"OCNG"), ch);
            channelIndex = find(idx,1,'first');
        end

    end
    
    % Methods specific to this class and that are currently used by
    % neighboring classes
    methods (Access = public)
        function frChanged(obj, ~)
            % handle offered MCS:
            obj.MCSGUI.(obj.DropdownValues) = wirelessWaveformApp.internal.getMCSOptionsDownlinkFRC(obj.FrequencyRange);
            
            frChanged@wirelessWaveformGenerator.nr5G_Presets_Dialog(obj);

            % Update available ReferenceSubcarrierSpacing options
            obj.ReferenceSubcarrierSpacingGUI.Items = wirelessWaveformGenerator.internal.getAvailableReferenceSCS(obj.FrequencyRange,obj.SubcarrierSpacing);
            % Update available TransmissionPeriodicity options
            obj.TransmissionPeriodicityGUI.Items = wirelessWaveformGenerator.internal.getAvailableTDDPeriods(obj.ReferenceSubcarrierSpacing);

            obj.pUpdateTDD = true;
            updateInfo(obj);
            obj.pUpdateTDD = false;
        end
    end
    
    % Private methods
    methods (Access = private)
        function layersChanged(obj, ~)
            updateInfo(obj);
        end

        function mcsChanged(obj, ~)
            obj.pUpdateTDD = true;
            updateInfo(obj);
            obj.pUpdateTDD = false;
            updateREVisual(obj, obj.ChannelName);
        end
        
        function OCNGChanged(obj)
            updateInfo(obj);
            
            updateGrid(obj);
        end
    end
    
    % Protect methods for derived classes
    methods (Access = protected)
        function updateInfo(obj)
            try
                try
                    cfg = getConfiguration(obj);
                catch
                    % Initialization; possibly inconsistent state
                    return;
                end
                if ~obj.Parent.DialogsMap.isKey(obj.infoClass)
                    % dialog initialization
                    return;
                end
                ch = cfg.PDSCH{1};
                
                infoDialog = obj.Parent.DialogsMap(obj.infoClass);
                infoDialog.SubcarrierSpacing      = cfg.SCSCarriers{1}.SubcarrierSpacing;
                infoDialog.AllocatedRBs           = length(ch.PRBSet);
                infoDialog.Modulation             = ch.Modulation;
                mcs = obj.MCS;
                idx = strfind(mcs, '=');
                rate = mcs(idx(1)+1:end);
                infoDialog.TargetCodeRateGUI.(obj.TextValue) = rate;
                
                infoDialog.DMRSConfigurationType  = ch.DMRS.DMRSConfigurationType;
                infoDialog.DMRSAdditionalPosition = ch.DMRS.DMRSAdditionalPosition;
                infoDialog.FirstDMRSPosition      = ch.DMRS.DMRSTypeAPosition;
                
                infoDialog.AllocatedSymbolsGUI.(obj.TextValue) = sprintf('[%d %d]',ch.SymbolAllocation);
                infoDialog.Xoh_PDSCH              = ch.XOverhead;
                infoDialog.PDSCHMappingType       = ch.MappingType;
                
                % Calculate Payload (bits) / Transport Block Size (TBS)
                % Use the same methodology used by waveform generators
                carrier = nrCarrierConfig('NCellID', cfg.NCellID, ...
                    'SubcarrierSpacing', cfg.BandwidthParts{1}.SubcarrierSpacing, ...
                    'CyclicPrefix', cfg.BandwidthParts{1}.CyclicPrefix, ...
                    'NSizeGrid', cfg.BandwidthParts{1}.NSizeBWP, ...
                    'NStartGrid', cfg.BandwidthParts{1}.NStartBWP);
                ptrs = nrPDSCHPTRSConfig('TimeDensity', ch.PTRS.TimeDensity, ...
                    'FrequencyDensity', ch.PTRS.FrequencyDensity, ...
                    'REOffset', ch.PTRS.REOffset, ...
                    'PTRSPortSet', ch.PTRS.PTRSPortSet);
                dmrs = nrPDSCHDMRSConfig('DMRSConfigurationType', ch.DMRS.DMRSConfigurationType, ...
                    'DMRSTypeAPosition', ch.DMRS.DMRSTypeAPosition, ...
                    'DMRSAdditionalPosition', ch.DMRS.DMRSAdditionalPosition, ...
                    'DMRSLength', ch.DMRS.DMRSLength, ...
                    'NIDNSCID', ch.DMRS.NIDNSCID, ...
                    'NSCID', ch.DMRS.NSCID, ...
                    'NumCDMGroupsWithoutData', ch.DMRS.NumCDMGroupsWithoutData);
                pdsch = nrPDSCHConfig('NSizeBWP', cfg.BandwidthParts{1}.NSizeBWP, ...
                    'NStartBWP', cfg.BandwidthParts{1}.NStartBWP, ...
                    'Modulation', ch.Modulation, ...
                    'NumLayers', ch.NumLayers, ...
                    'MappingType', ch.MappingType, ...
                    'SymbolAllocation', ch.SymbolAllocation, ...
                    'PRBSet', ch.PRBSet, ...
                    'NID', ch.NID, ...
                    'RNTI', ch.RNTI, ...
                    'EnablePTRS', ch.EnablePTRS, ...
                    'PTRS', ptrs, ...
                    'DMRS', dmrs);
                
                [~, modinfo] = nrPDSCHIndices(carrier, pdsch);
                infoDialog.PayloadSize  = nrTBS(ch.Modulation, ch.NumLayers, length(ch.PRBSet), ...
                    modinfo.NREPerPRB, ch.TargetCodeRate, ch.XOverhead);
                
            catch exc
                % nrFRCDL may error
                obj.errorFromException(exc);
            end
        end

        function scsChanged(obj, ~)
            % Update available ReferenceSubcarrierSpacing options
            obj.ReferenceSubcarrierSpacingGUI.Items = wirelessWaveformGenerator.internal.getAvailableReferenceSCS(obj.FrequencyRange,obj.SubcarrierSpacing);
            % Update available TransmissionPeriodicity options (use
            % SubcarrierSpacing as the reference GUI is not updated yet)
            obj.TransmissionPeriodicityGUI.Items = wirelessWaveformGenerator.internal.getAvailableTDDPeriods(min(obj.SubcarrierSpacing,120));
            scsChanged@wirelessWaveformGenerator.nr5G_Presets_Dialog(obj);
        end

        function scsChangedGUI(obj, ~)
            obj.scsChanged();
            obj.pUpdateTDD = true;
            scsChangedGUI@wirelessWaveformGenerator.nr5G_Presets_Dialog(obj);
            obj.pUpdateTDD = false;
        end
        
        function DuplexModeChanged(obj,updateTDD)
            if nargin < 2
                updateTDD = true;
            end
            % Update available ReferenceSubcarrierSpacing options
            obj.ReferenceSubcarrierSpacingGUI.Items = wirelessWaveformGenerator.internal.getAvailableReferenceSCS(obj.FrequencyRange,obj.SubcarrierSpacing);
            % Update available TransmissionPeriodicity options (use
            % SubcarrierSpacing as the reference GUI is not updated yet)
            obj.TransmissionPeriodicityGUI.Items = wirelessWaveformGenerator.internal.getAvailableTDDPeriods(min(obj.SubcarrierSpacing,120));
            obj.pUpdateTDD = updateTDD;
            DuplexModeChanged@wirelessWaveformGenerator.nr5G_Presets_Dialog(obj);
            obj.pUpdateTDD = false;
            isTDD = strcmp(obj.DuplexMode,'TDD');
            setVisible(obj, 'ReferenceSubcarrierSpacing', isTDD);
            setVisible(obj, 'TransmissionPeriodicity', isTDD);
            setVisible(obj, 'NumDownlinkSlots', isTDD);
            setVisible(obj, 'NumUplinkSlots', isTDD);
            setVisible(obj, 'EnableSpecialSlots', isTDD);
            setVisible(obj, 'NumDownlinkSym', isTDD);
            setVisible(obj, 'NumUplinkSym', isTDD);
            setVisible(obj, 'NumSpecialSlots', isTDD);
            setVisible(obj, 'TDDSlotAllocation', isTDD);
            if isTDD && updateTDD
                applyTDDConfig(obj);
            end
            layoutUIControls(obj);
        end

        function applyTDDConfig(obj,~)
            msg = [];
            e = [];
            downlinkString = repmat('D ',1,obj.NumDownlinkSlots);
            uplinkString = repmat('U ',1,obj.NumUplinkSlots);
            scs = obj.ReferenceSubcarrierSpacing;
            numSlotsPerSubframe = scs/15; % slots per ms
            totalSlots = obj.TransmissionPeriodicity * numSlotsPerSubframe;
            numSpecialSlots = totalSlots - (obj.NumDownlinkSlots + obj.NumUplinkSlots);
            try
                if numSpecialSlots < 0
                    obj.NumSpecialSlots = 'N/A';
                    obj.TDDSlotAllocation = 'N/A';
                    coder.internal.error('nr5g:waveformApp:TDDSlotsExceedPeriod',obj.NumDownlinkSlots + obj.NumUplinkSlots,num2str(obj.TransmissionPeriodicity),scs);
                else
                    obj.NumSpecialSlots = numSpecialSlots;
                    specialString = repmat('S ',1,numSpecialSlots);
                    obj.TDDSlotAllocation = [downlinkString specialString uplinkString];
                    totalSpecialSymbols = numSpecialSlots * 14;
                    if (obj.NumDownlinkSym + obj.NumUplinkSym) > totalSpecialSymbols  && obj.EnableSpecialSlotsGUI.Value
                        coder.internal.error('nr5g:waveformApp:TDDSpecialSymbolsExceedSlots',obj.NumDownlinkSym + obj.NumUplinkSym,totalSpecialSymbols);
                    end
                end
            catch e
                msg = e.message;
            end
            obj.updateConfigDiagnostic(msg);
            obj.updateErrorCache(e,'TDD');
        end

        function customTDD = getTDDConfigGUI(obj)
            customTDD = struct();
            if ~obj.pUpdateTDD % Don't overwrite TDD parameters in GUI, use custom TDD config
                customTDD = struct();
                customTDD.referenceSubcarrierSpacing = obj.ReferenceSubcarrierSpacing;
                customTDD.dl_UL_TransmissionPeriodicity = obj.TransmissionPeriodicity;
                customTDD.nrofDownlinkSlots = obj.NumDownlinkSlots;
                customTDD.nrofUplinkSlots = obj.NumUplinkSlots;
                customTDD.nrofDownlinkSymbols = obj.NumDownlinkSym;
                customTDD.nrofUplinkSymbols = obj.NumUplinkSym;
                customTDD.useDefault = false;
                % Errors in DL FRC can only be caused by a custom TDD config.
                % Check if the custom config above is invalid and throw the error.
                % This will also clear the error if the config is valid.
                obj.applyTDDConfig();
                if ~isempty(obj.pErrorCache)
                    % Rethrow newest error
                    rethrow(obj.pErrorCache{end});
                end
            end
            customTDD.transmitSpecialSlot = obj.EnableSpecialSlotsGUI.Value;
        end

        function updateTDDConfigGUI(obj,tddcfg)
		    obj.EnableSpecialSlots = false;
            obj.ReferenceSubcarrierSpacing = tddcfg.referenceSubcarrierSpacing;
            obj.TransmissionPeriodicity = tddcfg.dl_UL_TransmissionPeriodicity;
            obj.NumDownlinkSlots = tddcfg.nrofDownlinkSlots;
            obj.NumUplinkSlots = tddcfg.nrofUplinkSlots;
            obj.NumDownlinkSym = tddcfg.nrofDownlinkSymbols;
            obj.NumUplinkSym = tddcfg.nrofUplinkSymbols;
            obj.applyTDDConfig();
        end

        function referenceSCSChanged(obj,~)
            % Update available TransmissionPeriodicity options
            obj.TransmissionPeriodicityGUI.Items = wirelessWaveformGenerator.internal.getAvailableTDDPeriods(obj.ReferenceSubcarrierSpacing);
            updateGrid(obj);
        end

        function transmissionPeriodicityChanged(obj,~)
            updateGrid(obj);
        end

        function enableSpecialSlotsChanged(obj,~)
            updateGrid(obj);
        end

        function numDownlinkSlotsChanged(obj,src,evnt)
            updateProperty(obj, src, evnt, "NumDownlinkSlots", ValidationFunction=@(x)validateTDDSlots(x,getString(message('nr5g:waveformApp:FRCSelectionDownlinkSlotsError'))));
        end

        function numUplinkSlotsChanged(obj,src,evnt)
            updateProperty(obj, src, evnt, "NumUplinkSlots", ValidationFunction=@(x)validateTDDSlots(x,getString(message('nr5g:waveformApp:FRCSelectionUplinkSlotsError'))));
        end

        function numDownlinkSymChanged(obj,src,evnt)
            updateProperty(obj, src, evnt, "NumDownlinkSym", ValidationFunction=@(x)validateTDDSymbols(x,getString(message('nr5g:waveformApp:FRCSelectionDownlinkSymbolsError'))));
        end

        function numUplinkSymChanged(obj,src,evnt)
            updateProperty(obj, src, evnt, "NumUplinkSym", ValidationFunction=@(x)validateTDDSymbols(x,getString(message('nr5g:waveformApp:FRCSelectionUplinkSymbolsError'))));
        end

        function out = getChannelName(~)
            out = 'PDSCH FRC';
        end
    end
    
    % Getters/setters
    methods
        function frc = get.FRC(obj)
            frc = ['DL-FRC-' obj.FrequencyRange '-' obj.Modulation];
        end

        function mcs = get.MCS(obj)
            mcs = getDropdownVal(obj, 'MCS');
        end
        function set.MCS(obj, val)
            setDropdownVal(obj, 'MCS', val);

            mcsChanged(obj);
        end

        function mod = get.Modulation(obj)
            mcs = obj.MCS;
            idx = strfind(mcs, ',');
            mod = mcs(1:idx(1)-1);
        end
        
        function mod = get.TargetCodingRate(obj)
            mcs = obj.MCS;
            idx = strfind(mcs, '=');
            rate = mcs(idx(1)+1:end);
            % do conversions that are necessary to match the Payload Size from the standard's tables
            if strcmp(rate, '1/3')
                rate = '308/1024'; % only used for QPSK
            elseif strcmp(rate, '1/2')
                rate = '517/1024'; % only used for 64-QAM
            elseif strcmp(rate, '3/4')
                rate = '772/1024'; % only used for 64-QAM
            elseif strcmp(rate, '4/5')
                rate = '822/1024'; % only used for 256-QAM
            elseif strcmp(rate, '0.78')
                rate = '805.5/1024'; % only used for 1024-QAM
            end
            mod = eval(rate);
        end
    end
    
end

function validateTDDSlots(val,str)
    validateattributes(val, {'numeric'}, {'integer', 'scalar', 'real', '>=', 0}, '', str);
end

function validateTDDSymbols(val,str)
    validateattributes(val, {'numeric'}, {'integer', 'scalar', 'real', '>=', 0, '<=', 14}, '', str);
end
