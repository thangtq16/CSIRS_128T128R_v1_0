classdef nr5G_DL_InfoDialog < wirelessWaveformGenerator.Dialog
    %

    %   Copyright 2019-2024 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
    end

    properties (Hidden)
        TitleString = getString(message('nr5g:waveformGeneratorApp:InfoTitle'))
        SubcarrierSpacingType = 'numericText'
        SubcarrierSpacingLabel
        SubcarrierSpacingGUI
        AllocatedRBsType = 'numericText'
        AllocatedRBsLabel
        AllocatedRBsGUI
        ModulationType = 'charText'
        ModulationLabel
        ModulationGUI
        TargetCodeRateType = 'charText'
        TargetCodeRateLabel
        TargetCodeRateGUI
        PayloadSizeType = 'numericText'
        PayloadSizeLabel
        PayloadSizeGUI
        DMRSConfigurationTypeType = 'numericText'
        DMRSConfigurationTypeLabel
        DMRSConfigurationTypeGUI
        PDSCHMappingTypeType = 'charText'
        PDSCHMappingTypeLabel
        PDSCHMappingTypeGUI
        AllocatedSymbolsType = 'numericText'
        AllocatedSymbolsLabel
        AllocatedSymbolsGUI
        DMRSAdditionalPositionType = 'numericText'
        DMRSAdditionalPositionLabel
        DMRSAdditionalPositionGUI
        FirstDMRSPositionType = 'numericText'
        FirstDMRSPositionLabel
        FirstDMRSPositionGUI
        Xoh_PDSCHType = 'numericText'
        Xoh_PDSCHLabel
        Xoh_PDSCHGUI
        configFcn = @struct
    end

    methods
        function obj = nr5G_DL_InfoDialog(parent)
            obj@wirelessWaveformGenerator.Dialog(parent); % call base constructor

            setupDialog(obj); % layout controls
        end

        function props = displayOrder(~)
            props = {'SubcarrierSpacing'; 'AllocatedRBs'; 'Modulation'; 'TargetCodeRate'; ...
                'PayloadSize'; 'PDSCHMappingType'; 'AllocatedSymbols'; ...
                'DMRSConfigurationType'; 'DMRSAdditionalPosition'; 'FirstDMRSPosition'; 'Xoh_PDSCH'; };
        end

        function adjustSpec(obj)
            obj.LabelWidth = 160;
        end

        function msg = getMsgString(~, id, varargin)
            msgID = ['nr5g:waveformGeneratorApp:' id];
            msg = getString(message(msgID, varargin{:}));
        end

        function restoreDefaults(obj)
            obj.SubcarrierSpacing = '';
            obj.AllocatedRBs = '';
            obj.Modulation = '';
            obj.TargetCodeRate = '';
            obj.PayloadSize = '';
            obj.DMRSConfigurationType = '';
            obj.AllocatedSymbols = '';
            obj.DMRSAdditionalPosition = '';
            obj.FirstDMRSPosition = '';
            obj.Xoh_PDSCH = '';
            obj.PDSCHMappingType = '';
        end
    end

end