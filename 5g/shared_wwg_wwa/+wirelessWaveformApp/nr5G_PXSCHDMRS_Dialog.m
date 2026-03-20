classdef nr5G_PXSCHDMRS_Dialog < wirelessWaveformApp.nr5G_Dialog
    % Interface to properties that are common for PXSCH DMRS

    %   Copyright 2020-2025 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
    end

    properties (Hidden)
        DMRSConfigurationTypeType = 'numericPopup'
        DMRSConfigurationTypeDropDown = {'1', '2'}
        DMRSConfigurationTypeLabel
        DMRSConfigurationTypeGUI

        DMRSTypeAPositionType = 'numericPopup'
        DMRSTypeAPositionDropDown = {'2', '3'}
        DMRSTypeAPositionLabel
        DMRSTypeAPositionGUI

        DMRSAdditionalPositionType = 'numericPopup'
        DMRSAdditionalPositionDropDown = {'0', '1', '2', '3'}
        DMRSAdditionalPositionLabel
        DMRSAdditionalPositionGUI

        DMRSLengthType = 'numericPopup'
        DMRSLengthDropDown = {'1', '2'}
        DMRSLengthLabel
        DMRSLengthGUI

        CustomSymbolSetType = 'numericEdit'
        CustomSymbolSetLabel
        CustomSymbolSetGUI

        DMRSPortSetType = 'numericEdit'
        DMRSPortSetLabel
        DMRSPortSetGUI

        NIDNSCIDType = 'numericEdit'
        NIDNSCIDLabel
        NIDNSCIDGUI

        NSCIDType = 'numericPopup'
        NSCIDDropDown = {'0', '1'}
        NSCIDLabel
        NSCIDGUI

        NumCDMGroupsWithoutDataType = 'numericPopup'
        NumCDMGroupsWithoutDataDropDown = {'1', '2', '3'}
        NumCDMGroupsWithoutDataLabel
        NumCDMGroupsWithoutDataGUI

        PowerType = 'numericEdit'
        PowerGUI
        PowerLabel

        DMRSEnhancedR18Type = 'checkbox'
        DMRSEnhancedR18GUI
        DMRSEnhancedR18Label
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_PXSCHDMRS_Dialog(parent, fig, invisibleProps, cfg)
            obj@wirelessWaveformApp.nr5G_Dialog(parent, fig, cfg); % call base constructor

            obj.PowerGUI.(obj.Callback)                   = @(src,evnt) updateProperty(obj,src,evnt,"DMRSPower",Fieldnames="Power");
            obj.DMRSConfigurationTypeGUI.(obj.Callback)   = @(src,evnt) DMRSConfigurationTypeChangedGUI(obj,src,evnt);
            obj.DMRSTypeAPositionGUI.(obj.Callback)       = @(src,evnt) updateProperty(obj,src,evnt,"DMRSTypeAPosition",Config=obj.Config.DMRS);
            obj.DMRSAdditionalPositionGUI.(obj.Callback)  = @(src,evnt) updateProperty(obj,src,evnt,"DMRSAdditionalPosition",Config=obj.Config.DMRS);
            obj.DMRSLengthGUI.(obj.Callback)              = @(src,evnt) updateProperty(obj,src,evnt,"DMRSLength",Config=obj.Config.DMRS);
            obj.CustomSymbolSetGUI.(obj.Callback)         = @(src,evnt) updateProperty(obj,src,evnt,"CustomSymbolSet",Config=obj.Config.DMRS);
            obj.DMRSPortSetGUI.(obj.Callback)             = @(src,evnt) updateProperty(obj,src,evnt,"DMRSPortSet",Config=obj.Config.DMRS);
            obj.NIDNSCIDGUI.(obj.Callback)                = @(src,evnt) updateProperty(obj,src,evnt,"NIDNSCID",Config=obj.Config.DMRS);
            obj.NSCIDGUI.(obj.Callback)                   = @(src,evnt) updateProperty(obj,src,evnt,"NSCID",Config=obj.Config.DMRS);
            obj.NumCDMGroupsWithoutDataGUI.(obj.Callback) = @(src,evnt) NumCDMGroupsWithoutDataChangedGUI(obj,src,evnt);
            obj.DMRSEnhancedR18GUI.(obj.Callback)         = @(src,evnt) updateProperty(obj,src,evnt,"DMRSEnhancedR18",Config=obj.Config.DMRS);

            if nargin > 2
                % Hide the properties that are not applicable at the time of
                % construction (this private property is used in displayOrder)
                invisibleProps = cellstr(invisibleProps);
                propsDlg = displayOrder(obj);  % Properties that are visible in this dialog
                propsInvisible = ismember(propsDlg, invisibleProps);
                setVisible(obj, propsDlg(propsInvisible), false);
            end
        end

        function cellDialogs = getDialogsPerColumn(obj)
            cellDialogs{1} = {obj};
        end
    end

    methods (Access = protected)
        function DMRSConfigurationTypeChangedGUI(obj,src,evnt)
            updateProperty(obj,src,evnt,"DMRSConfigurationType",Config=obj.Config.DMRS);
        end
        function NumCDMGroupsWithoutDataChangedGUI(obj,src,evnt)
            updateProperty(obj,src,evnt,"NumCDMGroupsWithoutData",Config=obj.Config.DMRS);
        end
    end
end
