classdef nr5G_FRC_PUSCH_Dialog < wirelessWaveformGenerator.nr5G_Dialog
    % PUSCH configuration panel inside Uplink FRC extension for 5G Wireless
    % Waveform Generator App

    %   Copyright 2024 The MathWorks, Inc.

    % GUI properties with custom getters/setters
    properties (Dependent = true, Hidden = true)

        PUSCHSlotAllocation
        PUSCHTargetCodeRate

    end

    % GUI properties
    properties (Hidden = true)

        TitleString = getString(message('nr5g:waveformApp:FRCPUSCHTitle'))

        PUSCHNRBType = 'numericEdit'
        PUSCHNRBGUI
        PUSCHNRBLabel

        PUSCHLocationType = 'charPopup'
        PUSCHLocationDropDown = {'Upper bandwidth edge', 'Bandwidth center', 'Lower bandwidth edge', 'Custom'}
        PUSCHLocationGUI
        PUSCHLocationLabel

        RBOffsetType = 'numericEdit'
        RBOffsetGUI
        RBOffsetLabel

        AllocatedPRBSetType = 'charText'
        AllocatedPRBSetLabel
        AllocatedPRBSetGUI

        PUSCHSlotAllocationType = 'numericEdit'
        PUSCHSlotAllocationGUI
        PUSCHSlotAllocationLabel

        PUSCHPeriodType = 'numericEdit'
        PUSCHPeriodGUI
        PUSCHPeriodLabel

        PUSCHStartSymbolType = 'numericPopup'
        PUSCHStartSymbolDropDown = cellstr(string(0:13))
        PUSCHStartSymbolGUI
        PUSCHStartSymbolLabel

        PUSCHSymbolLengthType = 'numericPopup'
        PUSCHSymbolLengthDropDown = cellstr(string(0:14))
        PUSCHSymbolLengthGUI
        PUSCHSymbolLengthLabel

        PUSCHModulationType = 'charPopup'
        PUSCHModulationDropDown = {'QPSK','16QAM','64QAM','256QAM'}
        PUSCHModulationGUI
        PUSCHModulationLabel

        PUSCHTargetCodeRateType = 'charEdit'
        PUSCHTargetCodeRateGUI
        PUSCHTargetCodeRateLabel

        PayloadSizeType = 'numericText'
        PayloadSizeGUI
        PayloadSizeLabel

        PUSCHLayersType = 'numericPopup'
        PUSCHLayersDropDown = {'1','2','4'}
        PUSCHLayersGUI
        PUSCHLayersLabel

        % RNTI is defined in wirelessWaveformApp.nr5G_Main_Base.Dialog

        PUSCHTransformPrecodingType = 'checkbox'
        PUSCHTransformPrecodingGUI
        PUSCHTransformPrecodingLabel

        DMRSConfigurationTypeType = 'numericPopup'
        DMRSConfigurationTypeDropDown = {'1','2'}
        DMRSConfigurationTypeGUI
        DMRSConfigurationTypeLabel

        PUSCHMappingTypeType = 'charPopup'
        PUSCHMappingTypeDropDown = {'A','B'}
        PUSCHMappingTypeGUI
        PUSCHMappingTypeLabel

        DMRSTypeAPositionType = 'numericPopup'
        DMRSTypeAPositionDropDown = {'2','3'}
        DMRSTypeAPositionGUI
        DMRSTypeAPositionLabel

        DMRSAdditionalPositionType = 'numericPopup'
        DMRSAdditionalPositionDropDown = {'0','1','2','3'};
        DMRSAdditionalPositionGUI
        DMRSAdditionalPositionLabel

        PTRSType = 'checkbox'
        PTRSGUI
        PTRSLabel

        SelectPUSCHType = 'charPopup'
        SelectPUSCHDropDown = {'PUSCH 1','PUSCH 2'}
        SelectPUSCHGUI
        SelectPUSCHLabel

        EnablePUSCH2Type = 'checkbox'
        EnablePUSCH2GUI
        EnablePUSCH2Label

    end

    % Private properties for internal processing
    properties (Access = private)

        pConfigCache                                % cached nrULCarrierConfig object
        pMaxRBOffset                                % maximum RBOffset depending on BWP size and NRB
        pDefaultRBOffset                            % default RBOffset depending on BWP size and NRB
        pDefaultPUSCHLocation                       % default PUSCH location
        pNumLayersNonTP                             % last selected NumLayers value in non-TP scenario 
        pPUSCHNumber                                % selected PUSCH, relevant when TDD special slots are enabled
        pPUSCHLocation                              % cache of PUSCH locations, used in updatePUSCHConfig()

    end

    % Private constant properties
    properties (Access = private, Constant = true)

        pFRCClassName = 'wirelessWaveformGenerator.nr5G_FRC_UL_Dialog'  % FRC dialog name

    end

    %% Constructor, implementation of abstract methods defined in superclasses, and overriding superclass methods
    methods

        % Constructor
        function obj = nr5G_FRC_PUSCH_Dialog(parent)

            % Call base constructor
            obj@wirelessWaveformGenerator.nr5G_Dialog(parent);

            % Add GUI callbacks
            obj.PUSCHNRBGUI.(obj.Callback)                     = @(src,evnt)nrbChangedGUI(obj,src,evnt);
            obj.PUSCHLocationGUI.(obj.Callback)                = @(src,evnt)puschLocationChangedGUI(obj);
            obj.RBOffsetGUI.(obj.Callback)                     = @(src,evnt)rbOffsetChangedGUI(obj,src,evnt);
            obj.PUSCHSlotAllocationGUI.(obj.Callback)          = @(src,evnt)slotAllocationChangedGUI(obj,src,evnt);
            obj.PUSCHPeriodGUI.(obj.Callback)                  = @(src,evnt)periodChangedGUI(obj,src,evnt);
            obj.PUSCHStartSymbolGUI.(obj.Callback)             = @(src,evnt)symbolAllocationChangedGUI(obj);
            obj.PUSCHSymbolLengthGUI.(obj.Callback)            = @(src,evnt)symbolAllocationChangedGUI(obj);
            obj.PUSCHModulationGUI.(obj.Callback)              = @(src,evnt)modulationChangedGUI(obj);
            obj.PUSCHTargetCodeRateGUI.(obj.Callback)          = @(src,evnt)tcrChangedGUI(obj,src,evnt);
            obj.PUSCHLayersGUI.(obj.Callback)                  = @(src,evnt)numLayersChangedGUI(obj);
            obj.PUSCHTransformPrecodingGUI.(obj.Callback)      = @(src,evnt)tpChangedGUI(obj);
            obj.DMRSConfigurationTypeGUI.(obj.Callback)        = @(src,evnt)dmrsConfigurationChangedGUI(obj);
            obj.PUSCHMappingTypeGUI.(obj.Callback)             = @(src,evnt)dmrsConfigurationChangedGUI(obj);
            obj.DMRSTypeAPositionGUI.(obj.Callback)            = @(src,evnt)dmrsConfigurationChangedGUI(obj);
            obj.DMRSAdditionalPositionGUI.(obj.Callback)       = @(src,evnt)dmrsConfigurationChangedGUI(obj);
            obj.PTRSGUI.(obj.Callback)                         = @(src,evnt)ptrsChangedGUI(obj);
            obj.SelectPUSCHGUI.(obj.Callback)                  = @(src,evnt)PUSCHChangedGUI(obj);
            obj.EnablePUSCH2GUI.(obj.Callback)                 = @(src,evnt)PUSCH2ChangedGUI(obj);

            % Layout controls
            obj.setupDialog();

        end

        % Display order of GUI objects
        function props = displayOrder(~)

            props = {'SelectPUSCH','EnablePUSCH2','PUSCHModulation','PUSCHLayers','PUSCHMappingType', ...
                'PUSCHStartSymbol','PUSCHSymbolLength','PUSCHSlotAllocation','PUSCHPeriod', ...
                'PUSCHNRB','PUSCHLocation','RBOffset','AllocatedPRBSet', ...
                'RNTI','PUSCHTargetCodeRate','PayloadSize','PUSCHTransformPrecoding', ...
                'DMRSConfigurationType','DMRSTypeAPosition','DMRSAdditionalPosition','PTRS'};

        end

        function adjustSpec(obj)

            obj.LabelWidth = 160;

        end

        % Restore default values and layout
        function restoreDefaults(obj)

            % Update cached config to default (G-FR1-A1-1)
            refObj = wirelessWaveformApp.internal.hNRReferenceWaveformGenerator('G-FR1-A1-1',40,15,'FDD',1);
            obj.pConfigCache = refObj.Config;
            obj.pDefaultPUSCHLocation = 'Bandwidth center';
            obj.pPUSCHNumber = 1;
            pusch = obj.pConfigCache.PUSCH{obj.pPUSCHNumber};
            obj.pNumLayersNonTP = pusch.NumLayers;
            % Initialise private RBOffset properties to make sure they have two elements
            obj.pMaxRBOffset = [0,0];
            obj.pDefaultRBOffset = [0,0];

            % Populate GUI objects with default FRC (G-FR1-A1-1)
            obj.PUSCHNRB                        = numel(pusch.PRBSet);
            obj.updateRBOffsetRange();
            obj.pPUSCHLocation                  = {obj.pDefaultPUSCHLocation,obj.pDefaultPUSCHLocation};
            obj.PUSCHLocation                   = obj.pPUSCHLocation{obj.pPUSCHNumber};
            obj.RBOffset                        = obj.pDefaultRBOffset(obj.pPUSCHNumber);
            obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.PRBSet = obj.RBOffset+(0:obj.PUSCHNRB-1);
            obj.PUSCHSlotAllocation             = pusch.SlotAllocation;
            obj.PUSCHPeriod                     = pusch.Period;
            obj.PUSCHStartSymbol                = pusch.SymbolAllocation(1);
            obj.PUSCHSymbolLength               = pusch.SymbolAllocation(2);
            obj.PUSCHModulation                 = pusch.Modulation;
            obj.PUSCHTargetCodeRate             = pusch.TargetCodeRate;
            obj.PUSCHLayers                     = pusch.NumLayers;
            obj.RNTI                            = pusch.RNTI;
            obj.PUSCHTransformPrecoding         = pusch.TransformPrecoding;
            obj.DMRSConfigurationType           = pusch.DMRS.DMRSConfigurationType;
            obj.PUSCHMappingType                = pusch.MappingType;
            obj.DMRSTypeAPosition               = pusch.DMRS.DMRSTypeAPosition;
            obj.DMRSAdditionalPosition          = pusch.DMRS.DMRSAdditionalPosition;
            obj.PTRS                            = pusch.EnablePTRS;

            % Restore PUSCH controls to default (PUSCH 1 selected, PUSCH 2 disabled)
            obj.SelectPUSCH                     = ['PUSCH ' num2str(obj.pPUSCHNumber)];
            obj.EnablePUSCH2                    = false;

            % Adjust GUI visibility
            obj.updateDisabled();
            obj.updateConditionalVisibility();

        end

        % Get configuration for save for exporting
        function configStruct = getConfigurationForSave(obj,configStruct)
            % This dialog is exported along with nr5G_FRC_UL_Dialog in the
            % same structure, so need to append this dialog's fields to the
            % structure returned by nr5G_FRC_UL_Dialog.

            configStruct.PUSCH = obj.pConfigCache.PUSCH{1};
            if numel(obj.pConfigCache.PUSCH) > 1
                configStruct.PUSCH2 = obj.pConfigCache.PUSCH{2};
            end

            % Save PUSCHLocation
            configStruct.PUSCHLocation = obj.pPUSCHLocation{1};
            if numel(obj.pConfigCache.PUSCH) > 1
                configStruct.PUSCHLocation2 = obj.pPUSCHLocation{2};
            end

            % Save the internally cached pNumLayersNonTP
            configStruct.pNumLayersNonTP = obj.pNumLayersNonTP;

        end

        % Properties to exclude when exporting
        function props = props2ExcludeFromConfig(~)
            % Read-only field do not get exported

            props = {'SelectPUSCH','EnablePUSCH2','AllocatedPRBSet','PayloadSize'};

        end

        function frChangedGUI(~)
            % no-op for this dialog
        end

        function frChanged(~)
            % no-op for this dialog
        end

        function generateWaveform(~)
            % no-op for this dialog
        end

        % Update GUI controllability
        function updateDisabled(obj)
            % This is required when waveform generation is finished and the
            % whole panel is re-enabled, or when a configuration is applied

            obj.updateControllabilityRBOffset();

            obj.updateControllabilityPUSCHLayers();

            obj.PUSCH2ChangedGUI();
        end

    end

    %% Public interfaces
    methods (Access = public)

        % Apply configuration to the app
        function applyPUSCHConfig(obj,cfg,mappingFRC,PUSCH2)
            % Called when:
            % 1. an FRC is selected in the pop-up window
            % 2. loading saved sessions
            % 3. Selected PUSCH is changed

            if nargin<4
                PUSCH2 = false;
                if nargin<3
                    mappingFRC = false; % assume the config does not match an FRC by default
                end
            end

            % Extract PUSCH config
            obj.pPUSCHNumber = 1;
            if PUSCH2 && numel(cfg.PUSCH) > 1
                obj.pPUSCHNumber = 2;
            end
            pusch = cfg.PUSCH{obj.pPUSCHNumber};

            % Update fields
            obj.PUSCHNRB = numel(pusch.PRBSet);
            obj.RBOffset = pusch.PRBSet(1);
            obj.PUSCHSlotAllocation = pusch.SlotAllocation;
            obj.PUSCHPeriod = pusch.Period;
            obj.PUSCHStartSymbol = pusch.SymbolAllocation(1);
            obj.PUSCHSymbolLength = pusch.SymbolAllocation(2);
            obj.PUSCHModulation = pusch.Modulation;
            obj.PUSCHTargetCodeRate = pusch.TargetCodeRate;
            obj.PUSCHLayers = pusch.NumLayers;
            obj.RNTI = pusch.RNTI;
            obj.PUSCHTransformPrecoding = pusch.TransformPrecoding;
            obj.updateRBOffsetRange(cfg);
            obj.DMRSConfigurationType = pusch.DMRS.DMRSConfigurationType;
            obj.PUSCHMappingType = pusch.MappingType;
            obj.DMRSTypeAPosition = pusch.DMRS.DMRSTypeAPosition;
            obj.DMRSAdditionalPosition = pusch.DMRS.DMRSAdditionalPosition;
            obj.PTRS = pusch.EnablePTRS;
            obj.SelectPUSCH = ['PUSCH ' num2str(obj.pPUSCHNumber)];
            obj.AllocatedPRBSet = sprintf('[%d:%d]',pusch.PRBSet(1),pusch.PRBSet(end));

            % Update PUSCHLocation and RBOffset
            if mappingFRC
                % When mapping a selected FRC, set PUSCHLocation to default

                obj.PUSCHLocation = obj.pDefaultPUSCHLocation;
                obj.RBOffset = obj.pDefaultRBOffset(obj.pPUSCHNumber);
                cfg.PUSCH{1}.PRBSet = obj.RBOffset+(0:obj.PUSCHNRB-1);
                if numel(cfg.PUSCH) > 1
                    cfg.PUSCH{2}.PRBSet = cfg.PUSCH{1}.PRBSet;
                end

            else
                % When not mapping FRC, determine PUSCHLocation and
                % RBOffset from BWP size and PUSCH NRB

                % Treat full band allocation as bandwidth center
                if obj.RBOffset == 0 && obj.RBOffset~=obj.pDefaultRBOffset(obj.pPUSCHNumber)
                    obj.PUSCHLocation = 'Lower bandwidth edge';
                elseif obj.RBOffset == obj.pMaxRBOffset(obj.pPUSCHNumber) && obj.RBOffset~=obj.pDefaultRBOffset(obj.pPUSCHNumber)
                    obj.PUSCHLocation = 'Upper bandwidth edge';
                elseif obj.RBOffset == obj.pDefaultRBOffset(obj.pPUSCHNumber)
                    obj.PUSCHLocation = obj.pDefaultPUSCHLocation;
                else
                    obj.PUSCHLocation = 'Custom';
                end

            end

            % Update GUI controllability if necessary
            obj.updateDisabled();

            % Update cached config and grid and info
            obj.pConfigCache = cfg;

            % Flush the pNumLayersNonTP cache in non-TP cases
            if ~obj.pConfigCache.PUSCH{1}.TransformPrecoding
                obj.pNumLayersNonTP(1) = obj.pConfigCache.PUSCH{1}.NumLayers;
            end
            if numel(obj.pConfigCache.PUSCH) > 1
                if ~obj.pConfigCache.PUSCH{2}.TransformPrecoding
                    obj.pNumLayersNonTP(2) = obj.pConfigCache.PUSCH{2}.NumLayers;
                end
            end

            if numel(obj.pConfigCache.PUSCH) > 1
                obj.EnablePUSCH2 = obj.pConfigCache.PUSCH{2}.Enable;
            end

            obj.updateAppStatus(~mappingFRC);
            obj.updateErrorCache([],[]);
            obj.updateConditionalVisibility();
            
            layoutUIControls(obj);

        end

        % Update cached PUSCH config
        function updatePUSCHConfig(obj,cfg,updateCacheOnly,isCustom)
            % Called when carrier changes (by General Configuration)
            % require PUSCH resource allocation updates

            if nargin<4
                isCustom = true; % assume config extends beyond FRC by default
                if nargin<3
                    updateCacheOnly = false; % assume GUI, grid and info need updating as well by default
                end
            end

            if ~updateCacheOnly
                numPUSCH = numel(obj.pConfigCache.PUSCH);
                for puschIdx = 1:numPUSCH
                    maxNRB = cfg.SCSCarriers{1}.NSizeGrid;
                    puschNRB = numel(obj.pConfigCache.PUSCH{puschIdx}.PRBSet);
                    rbOffset = obj.pConfigCache.PUSCH{puschIdx}.PRBSet(1);

                    if puschNRB>maxNRB
                        % Force PUSCH to fit into carrier
                        puschNRB = maxNRB;
                    end

                    % Update RBOffset range
                    cfg.PUSCH{puschIdx}.PRBSet = rbOffset+(0:puschNRB-1);
                    obj.updateRBOffsetRange(cfg,puschIdx);

                    % Update RBOffset and PUSCHLocation if necessary
                    if rbOffset>obj.pMaxRBOffset(puschIdx)
                        % If PRBSet is no longer valid, update according to
                        % PUSCHLocation
                        if any(strcmpi(obj.pPUSCHLocation{puschIdx},{'Bandwidth center','Lower bandwidth edge','Custom'}))
                            % BWP size is smaller and PUSCH is now full-band
                            rbOffset = obj.pDefaultRBOffset(puschIdx);
                        else % 'Upper bandwidth edge'
                            % BWP size is smaller but PUSCH may not be
                            % full-band
                            rbOffset = obj.pMaxRBOffset(puschIdx);
                        end
                    else
                        % PRBSet is still valid, update RBOffset value
                        % according to PUSCHLocation for PUSCHLocation =
                        % 'Bandwidth center' or 'Upper bandwidth edge' (other
                        % situations don't need updating)
                        if strcmpi(obj.pPUSCHLocation{puschIdx},'Bandwidth center')
                            rbOffset = obj.pDefaultRBOffset(puschIdx);
                        elseif strcmpi(obj.pPUSCHLocation{puschIdx},'Upper bandwidth edge')
                            rbOffset = obj.pMaxRBOffset(puschIdx);
                        end
                    end

                    % Update RBOffset GUI controllability accordingly
                    obj.updateControllabilityRBOffset();

                    % Feed back into cache
                    cfg.PUSCH{puschIdx}.PRBSet = rbOffset+(0:puschNRB-1);

                    % Update GUI for selected PUSCH
                    if obj.pPUSCHNumber == puschIdx
                        obj.RBOffset = rbOffset;
                        obj.PUSCHNRB = puschNRB;
                    end
                    % Validate that the number of RBs fits the channel bandwidth,
                    % and clear any existing error if it does
                    obj.validatePUSCHNRBinBWP(cfg,puschIdx);
                end
            end

            % Update cache
            obj.updatePRBSet(cfg);
            % Update grid and info if necessary
            if ~updateCacheOnly
                obj.updateAppStatus(isCustom);
            end

        end

        % Load save session
        function loadSavedSession(obj,waveCfg,session,isCustom)
            % Called by nr5G_FRC_UL_Dialog/applyConfiguration, which is
            % called when loading saved sessions

            % Flush the pNumLayersNonTP cache, if any
            if isfield(session,'pNumLayersNonTP')
                obj.pNumLayersNonTP = session.pNumLayersNonTP;
            else
                % For pre-24b sessions, revert cache to default
                obj.pNumLayersNonTP = 1;
            end

            % Apply the PUSCH configuration
            obj.applyPUSCHConfig(waveCfg,~isCustom);

            % Update PUSCHLocation if necessary, as FRC may be saved with
            % different PUSCHLocation
            if isfield(session,'PUSCHLocation')
                obj.pPUSCHLocation{1} = session.PUSCHLocation;
                obj.PUSCHLocation = session.PUSCHLocation;
                if isfield(session,'PUSCHLocation2')
                    obj.pPUSCHLocation{2} = session.PUSCHLocation2;
                end
            end

        end

        function out = getPUSCHFRCChannelName(obj)
            if isscalar(obj.pConfigCache.PUSCH)
                out = 'PUSCH FRC';
            else
                out = ['PUSCH ' num2str(obj.pPUSCHNumber) ' FRC'];
            end
        end
    end

    %% Overriding protected methods
    methods (Access = protected)

        % Callback of RNTI
        function rntiChanged(obj,src,evnt)

            prop = 'RNTI';
            e = updateProperty(obj, src, evnt, prop, Config=obj.pConfigCache.PUSCH{obj.pPUSCHNumber});

            if isempty(e)
                % Update cached configuration
                obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.RNTI = obj.RNTI;
                % Update grid and info
                obj.updateAppStatus(false);
            end

        end

        function out = getChannelName(obj)
            out = getPUSCHFRCChannelName(obj);
        end

    end

    %% Callbacks of GUI controls defined in this class
    methods (Access = private)
    % General workflows:
    % 1. validation at setter level (cross validation is done when trying
    % to update Grid or ChannelView)
    % 2.a if valid, feed change into nr5G_FRC_UL_Dialog, which then updates
    % the cached config and update Grid, ChannelView and Info
    % 2.b if invalid, error

        % Callback for PUSCHNRB
        function nrbChangedGUI(obj,src,evnt)

            % First, check that the standalone value is correct
            function validationFcn(in)
                validateattributes(in,{'numeric'},{'integer','scalar','real','>',0,'<=',274},'','Number of allocated PUSCH RBs');
            end

            prop = 'PUSCHNRB';
            e = updateProperty(obj, src, evnt, prop, ValidationFunction=@validationFcn);

            % If there is no standalone error, update RBOffset and look for
            % potential cross-parameter issue
            if isempty(e)
                obj.updateRBOffsetRange();
                % Update RBOffset when necessary
                if strcmpi(obj.PUSCHLocation,'Upper bandwidth edge')
                    obj.RBOffset = obj.pMaxRBOffset(obj.pPUSCHNumber);
                elseif strcmpi(obj.PUSCHLocation,'Bandwidth center')
                    obj.RBOffset = obj.pDefaultRBOffset(obj.pPUSCHNumber);
                end
                % Feed into ConfigCache
                e = obj.updatePRBSet();
            end

            if isempty(e)
                % Validate that the number of RBs fits the channel bandwidth,
                % and clear any existing error if it does
                obj.validatePUSCHNRBinBWP();
                % Update grid and info
                obj.updateAppStatus(true);
            end

        end

        % Callback for PUSCHLocation
        function puschLocationChangedGUI(obj,~)

            location = obj.PUSCHLocation;
            obj.pPUSCHLocation{obj.pPUSCHNumber} = location;
            % Update RBOffset GUI controllability and its value if necessary
            obj.updateControllabilityRBOffset();
            if ~strcmpi(location, 'Custom')
                if strcmpi(location, 'Lower bandwidth edge')
                    obj.RBOffset = 0;
                elseif strcmpi(location, 'Upper bandwidth edge')
                    obj.RBOffset = obj.pMaxRBOffset(obj.pPUSCHNumber);
                else
                    obj.RBOffset = obj.pDefaultRBOffset(obj.pPUSCHNumber);
                end
            end

            % Feed new PRBSet into ConfigCache to validate
            e = obj.updatePRBSet();
            if isempty(e)
                % Update grid and info
                obj.updateAppStatus(false);
            end

        end

        % Callback for RBOffset
        function rbOffsetChangedGUI(obj,src,evnt)

            % First, check that the standalone value is correct
            function validationFcn(in)
                validateattributes(in,{'numeric'}, {'integer','scalar','real','nonnegative','<',274},'','resource block (RB) offset');
            end

            prop = 'RBOffset';
            e = updateProperty(obj, src, evnt, prop, ValidationFunction=@validationFcn);

            % If there is no standalone error, update RBOffset and look for
            % potential cross-parameter issue
            if isempty(e)
                % Feed new PRBSet into ConfigCache to validate
                e = obj.updatePRBSet();
            end

            if isempty(e)
                % Update grid and info
                obj.updateAppStatus(false);
            end

        end

        function e = validatePUSCHNRBinBWP(obj,cfg,puschNum)
            if nargin < 2
                % If config not provided, use cache
                cfg = obj.pConfigCache;
                puschNRB = obj.PUSCHNRB;
            else
                % If config is provided, puschNum must also be provided
                puschNRB = numel(cfg.PUSCH{puschNum}.PRBSet);
            end

            e = [];
            prop = 'InvalidPRBSetInBWP';
            try
                coder.internal.errorIf(cfg.BandwidthParts{1}.NSizeBWP < puschNRB,...
                    'nr5g:nrWaveformGenerator:InvalidPRBSetInBWP',puschNRB,'PUSCH',1,cfg.BandwidthParts{1}.NSizeBWP,1);
            catch e
                frcDlg = obj.getULFRCDialog();
                frcDlg.updateAppConfigDiagnostic(e);
            end
            obj.updateErrorCache(e,prop);
        end

        function e = updatePRBSet(obj,cfg)
            if nargin < 2
                cfg = [];
            end

            e = [];
            prop = 'PRBSet';
            try
                % Feed into cache
                if isempty(cfg)
                    obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.PRBSet = obj.RBOffset+(0:obj.PUSCHNRB-1);
                else
                    obj.pConfigCache = cfg;
                end
                obj.updateRBOffsetRange();
                val = obj.RBOffset;
                % Check if RBOffset is within acceptable values
                coder.internal.errorIf(val>obj.pMaxRBOffset(obj.pPUSCHNumber),'nr5g:waveformGeneratorApp:InvalidRBOffset',val,obj.pMaxRBOffset(obj.pPUSCHNumber));
                % Validate config
                obj.pConfigCache.validateConfig();
            catch e
                frcDlg = obj.getULFRCDialog();
                frcDlg.updateAppConfigDiagnostic(e);
            end
            obj.updateErrorCache(e,prop);
        end

        % Callback for PUSCHSlotAllocation
        function slotAllocationChangedGUI(obj,src,evnt)

            prop = 'PUSCHSlotAllocation';
            e = updateProperty(obj, src, evnt, 'SlotAllocation', Config=obj.pConfigCache.PUSCH{obj.pPUSCHNumber}, FieldNames=prop);

            if isempty(e)
                % Update cached configuration object
                obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.SlotAllocation = obj.PUSCHSlotAllocation;
                % Update grid and info
                obj.updateAppStatus(true);
            end

        end

        % Callback for Period
        function periodChangedGUI(obj,src,evnt)

            prop = 'PUSCHPeriod';
            e = updateProperty(obj, src, evnt, 'Period', Config=obj.pConfigCache.PUSCH{obj.pPUSCHNumber}, FieldNames=prop);

            if isempty(e)
                % Update cached configuration object
                obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.Period = obj.PUSCHPeriod;
                % Update grid and info
                obj.updateAppStatus(true);
            end

        end

        % Callback for PUSCHStartSymbol and PUSCHSymbolLength
        function symbolAllocationChangedGUI(obj,~)

            % Only cross-parameter validation, as these are all dropdowns
            % and no standalone invalid value can be entered
            e = [];
            prop = 'PUSCHSymbolAllocation';
            try
                pusch = obj.pConfigCache.PUSCH{obj.pPUSCHNumber};
                pusch.SymbolAllocation = [obj.PUSCHStartSymbol obj.PUSCHSymbolLength];
                obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.SymbolAllocation = [obj.PUSCHStartSymbol obj.PUSCHSymbolLength];
            catch e
                frcDlg = obj.getULFRCDialog();
                frcDlg.updateAppConfigDiagnostic(e);
            end
            obj.updateErrorCache(e,prop);

            if isempty(e)
                % Update grid and info
                obj.updateAppStatus(true);
            end

        end

        % Callback for PUSCHModulation
        function modulationChangedGUI(obj)

            obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.Modulation = obj.PUSCHModulation;
            % Update grid and info
            obj.updateAppStatus(true);

        end

        % Callback for PUSCHTargetCodeRate
        function tcrChangedGUI(obj,src,evnt)

            prop = 'PUSCHTargetCodeRate';
            e = updateProperty(obj, src, evnt, 'TargetCodeRate', Config=obj.pConfigCache.PUSCH{obj.pPUSCHNumber}, FieldNames=prop);

            if isempty(e)
                % Update cached configuration object
                obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.TargetCodeRate = obj.PUSCHTargetCodeRate;
                % Update grid and info
                obj.updateAppStatus(true);
            end

        end

        % Callback for PUSCHLayers
        function numLayersChangedGUI(obj)
            % NumLayers is only configurable when TransformPrecoding is false

            % Update NumLayers
            obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.NumLayers = obj.PUSCHLayers;
            obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.NumAntennaPorts = obj.PUSCHLayers;

            % Update cache of non-TP NumLayers
            obj.pNumLayersNonTP(obj.pPUSCHNumber) = obj.PUSCHLayers;

            % Update grid and info
            obj.updateAppStatus(true);

        end

        % Callback for PUSCHTransformPrecoding
        function tpChangedGUI(obj,~)

            obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.TransformPrecoding = obj.PUSCHTransformPrecoding;

            if obj.PUSCHTransformPrecoding
                % Change NumLayers to 1 if transform precoding is enabled
                obj.PUSCHLayers = 1;
            else
                % Return to the last selected NumLayers when transform
                % precoding is disabled
                obj.PUSCHLayers = obj.pNumLayersNonTP(obj.pPUSCHNumber);
            end

            % Update NumLayers and NumAntennaPorts in the cached config
            % accordingly
            obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.NumLayers = obj.PUSCHLayers;
            obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.NumAntennaPorts = obj.PUSCHLayers;

            % Update PUSCHLayers GUI controllability accordingly
            obj.updateControllabilityPUSCHLayers();

            % Update grid and info
            obj.updateAppStatus(true);

        end

        % Callback for DMRSConfigurationType, PUSCHMappingType,
        % DMRSTypeAPosition and DMRSAdditionalPositions
        function dmrsConfigurationChangedGUI(obj)

            % Only cross-parameter validation, as these are all dropdowns
            % and no standalone invalid value can be entered
            pusch = obj.pConfigCache.PUSCH{obj.pPUSCHNumber};
            e = [];
            prop = 'DMRSConfiguration';
            try
                pusch.MappingType = obj.PUSCHMappingType;
                pusch.DMRS.DMRSConfigurationType = obj.DMRSConfigurationType;
                pusch.DMRS.DMRSTypeAPosition = obj.DMRSTypeAPosition;
                pusch.DMRS.DMRSAdditionalPosition = obj.DMRSAdditionalPosition;
                obj.pConfigCache.PUSCH{obj.pPUSCHNumber} = pusch;
            catch e
                frcDlg = obj.getULFRCDialog();
                frcDlg.updateAppConfigDiagnostic(e);
            end
            obj.updateErrorCache(e,prop);

            if isempty(e)
                % Update grid and info
                obj.updateAppStatus(true);
                obj.updateConditionalVisibility();
            end

        end

        % Callback for PTRS
        function ptrsChangedGUI(obj,~)

            obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.EnablePTRS = obj.PTRS;
            % Update info
            obj.updateAppStatus(false);

        end

        function PUSCHChangedGUI(obj,~)
            PUSCH2 = strcmpi(obj.SelectPUSCH,'PUSCH 2');
            obj.applyPUSCHConfig(obj.pConfigCache,false,PUSCH2);
        end

        function PUSCH2ChangedGUI(obj,~)
            if obj.pPUSCHNumber == 1
                flag = true;  
            else
                flag = obj.EnablePUSCH2GUI.Value;
                obj.pConfigCache.PUSCH{2}.Enable = flag;
                obj.updateAppStatus(false);
            end

            setEnable(obj,'PUSCHNRB',flag);
            setEnable(obj,'PUSCHLocation',flag);
            if strcmpi(obj.PUSCHLocation,'Custom')
                setEnable(obj,'RBOffset',flag);
            end
            setEnable(obj,'PUSCHSlotAllocation',flag);
            setEnable(obj,'PUSCHPeriod',flag);
            setEnable(obj,'PUSCHStartSymbol',flag);
            setEnable(obj,'PUSCHSymbolLength',flag);
            setEnable(obj,'PUSCHModulation',flag);
            setEnable(obj,'PUSCHTargetCodeRate',flag);
            setEnable(obj,'PUSCHLayers',flag);
            setEnable(obj,'RNTI',flag);
            setEnable(obj,'PUSCHTransformPrecoding',flag);
            setEnable(obj,'DMRSConfigurationType',flag);
            setEnable(obj,'PUSCHMappingType',flag);
            setEnable(obj,'DMRSTypeAPosition',flag);
            setEnable(obj,'DMRSAdditionalPosition',flag);
            setEnable(obj,'PTRS',flag);
            setEnable(obj,'AllocatedPRBSet',flag);
            setEnable(obj,'PayloadSize',flag);
        end
    end

    %% Other functions
    methods (Access = private)

        % Update cached config in nr5G_FRC_UL_Dialog and tell it to update
        % grid, channel view and info if necessary
        function updateAppStatus(obj,isCustom)

            frcDlg = obj.getULFRCDialog();

            frcDlg.updateAppStatus(obj.pConfigCache,isCustom);

        end

        % Update the cached error to avoid generateWaveform
        function updateErrorCache(obj,e,prop)
            % This method MUST be called before calling updateAppStatus

            frcDlg = obj.getULFRCDialog();

            frcDlg.updateErrorCache(e,prop);

        end

        % Get nr5G_FRC_UL_Dialog object
        function dlg = getULFRCDialog(obj)

            className = obj.pFRCClassName;
            dlg = obj.Parent.DialogsMap(className);

        end

        % Update the default and max RBOffset according to PUSCH NRB and
        % channel bandwidth
        function updateRBOffsetRange(obj,cfg,puschNum)

            if nargin < 3
                % If puschNum not provided, use cached value to get selected PUSCH and use PUSCHNRB from GUI
                puschNum = obj.pPUSCHNumber;
                numRB = obj.PUSCHNRB;
                if nargin < 2
                    cfg = obj.pConfigCache; % if not provided, update according to config cache
                end
            else
                % If puschNum provided, use number of RBs from config
                numRB = numel(cfg.PUSCH{puschNum}.PRBSet);
            end

            maxRBOffset = cfg.BandwidthParts{1}.NSizeBWP - numRB;
            defaultRBOffset = floor(maxRBOffset/2);
            if maxRBOffset>=0
                obj.pMaxRBOffset(puschNum) = maxRBOffset;
                obj.pDefaultRBOffset(puschNum) = defaultRBOffset;
            end

        end

        % Update the conditional visibility of GUI objects
        function updateConditionalVisibility(obj)

            % Check DMRS type A position - dependent on MappingType
            oldVis = isVisible(obj,'DMRSTypeAPosition');
            newVis = strcmp(obj.pConfigCache.PUSCH{obj.pPUSCHNumber}.MappingType,'A');
            dmrsFlag = xor(oldVis,newVis);
            if dmrsFlag
                setVisible(obj,'DMRSTypeAPosition',newVis);
            end
            

            % Check visibility of PUSCH dropdown
            oldVis = isVisible(obj,'SelectPUSCH');
            newVis = numel(obj.pConfigCache.PUSCH) > 1;
            puschDropdownFlag = xor(oldVis,newVis);
            if puschDropdownFlag
                setVisible(obj,'SelectPUSCH',newVis);
            end

            % Check visibility of PUSCH 2 enable checkbox
            oldVis = isVisible(obj,'EnablePUSCH2');
            newVis = strcmp(obj.SelectPUSCH,'PUSCH 2');
            pusch2Flag = xor(oldVis,newVis);
            if pusch2Flag
                setVisible(obj,'EnablePUSCH2',newVis);
            end

            % Layout UI controls if visibilities have changed
            if dmrsFlag || puschDropdownFlag || pusch2Flag
                layoutUIControls(obj);
            end
        end

        % Update controllability of RBOffset depending on PUSCHLocation
        function updateControllabilityRBOffset(obj)
            % RBOffset: editfield

            % RBOffset: only active when PUSCHLocation is Custom
            flag = strcmpi(obj.PUSCHLocation,'Custom');
            setEnable(obj,'RBOffset',flag);
            setEditable(obj,'RBOffset',flag);

        end

        % Update controllability of PUSCHLayers depending on
        % PUSCHTransformPrecoding
        function updateControllabilityPUSCHLayers(obj)
            % PUSCHLayers: non-editable dropdown

            % PUSCHLayers: only active when PUSCHTransformPrecoding
            % is disabled
            flag = ~obj.PUSCHTransformPrecoding;
            setEnable(obj,'PUSCHLayers',flag);

        end

    end

    %% Custom getters and setters
    % Other properties have generic getters and setters defined by
    % wirelessAppContainer.Dialog
    methods

        function val = get.PUSCHSlotAllocation(obj)
            val = evalin('base',obj.PUSCHSlotAllocationGUI.Value);
        end
        function set.PUSCHSlotAllocation(obj,val)
            % Try to maintain [A:B] form if possible
            if isequal(val,min(val):max(val))
                str = ['[' num2str(min(val)) ':' num2str(max(val)) ']'];
            else
                str = mat2str(val);
            end
            setEditStr(obj,'PUSCHSlotAllocation',str);
        end

        function val = get.PUSCHTargetCodeRate(obj)
            val = evalin('base',obj.PUSCHTargetCodeRateGUI.Value);
        end
        function set.PUSCHTargetCodeRate(obj,val)
            % Try to maintain a/b form if possible
            if isnumeric(val)
                str = num2str(val);
            else
                str = val;
            end
            setEditStr(obj,'PUSCHTargetCodeRate',str);
        end
    end

end
