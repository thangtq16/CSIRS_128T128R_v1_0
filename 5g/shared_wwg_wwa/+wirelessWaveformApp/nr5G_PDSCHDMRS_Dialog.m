classdef nr5G_PDSCHDMRS_Dialog < wirelessWaveformApp.nr5G_PXSCHDMRS_Dialog
    % Interface to properties that are unique for PDSCH DMRS

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Hidden)
        TitleString = 'PDSCH 1 DM-RS'

        DMRSReferencePointType = 'charPopup'
        DMRSReferencePointDropDown = {'CRB0', 'PRB0'}
        DMRSReferencePointLabel
        DMRSReferencePointGUI

        DMRSDownlinkR16Type = 'checkbox'
        DMRSDownlinkR16GUI
        DMRSDownlinkR16Label
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_PDSCHDMRS_Dialog(parent, fig, invisibleProps)
            obj@wirelessWaveformApp.nr5G_PXSCHDMRS_Dialog(parent, fig, invisibleProps, wirelessWaveformApp.nr5G_PDSCHAdvanced_Dialog.DefaultCfg); % call base constructor

            obj.DMRSReferencePointGUI.(obj.Callback) = @(src,evnt) updateProperty(obj,src,evnt,"DMRSReferencePoint",Config=obj.Config.DMRS);
            obj.DMRSDownlinkR16GUI.(obj.Callback)    = @(src,evnt) updateProperty(obj,src,evnt,"DMRSDownlinkR16",Config=obj.Config.DMRS);
        end

        function needRepaint = updateControlsVisibility(~,~)
            % No control visibility rules in this dialog
            needRepaint = false;
        end

        function adjustDialog(obj)
            % Make sure all tags are unique. Otherwise there is a conflict with SSB, PTRS
            obj.DMRSTypeAPositionGUI.Tag = 'PDSCHDMRSTypeAPosition';
            obj.PowerGUI.Tag = 'Power_DMRS';
        end

        function props = displayOrder(~)
            props = {'Power'; 'DMRSConfigurationType'; 'DMRSReferencePoint'; 'DMRSTypeAPosition'; ...
                     'DMRSAdditionalPosition'; 'DMRSLength'; 'CustomSymbolSet'; ...
                     'DMRSPortSet'; 'NIDNSCID'; 'NSCID'; 'NumCDMGroupsWithoutData'; ...
                     'DMRSDownlinkR16'; 'DMRSEnhancedR18'};
        end

        function restoreDefaults(obj)
            % This method is called in the baseclass constructor to set the
            % default values of the properties
            % Get the same defaults with programmatic objects
            c = nrWavegenPDSCHConfig;
            obj.Power                   = c.DMRSPower;

            c = nrPDSCHDMRSConfig;
            obj.DMRSConfigurationType   = c.DMRSConfigurationType;
            obj.DMRSReferencePoint      = c.DMRSReferencePoint;
            obj.DMRSTypeAPosition       = c.DMRSTypeAPosition;
            obj.DMRSAdditionalPosition  = c.DMRSAdditionalPosition;
            obj.DMRSLength              = c.DMRSLength;
            obj.CustomSymbolSet         = c.CustomSymbolSet;
            obj.DMRSPortSet             = c.DMRSPortSet;
            obj.NIDNSCID                = c.NIDNSCID;
            obj.NSCID                   = c.NSCID;
            obj.NumCDMGroupsWithoutData = c.NumCDMGroupsWithoutData;
            obj.DMRSDownlinkR16         = c.DMRSDownlinkR16;
            obj.DMRSEnhancedR18         = c.DMRSEnhancedR18;
        end
    end

    % Custom getters/setters
    methods
        function out = get.ChannelName(~)
            out = 'PDSCH';
        end
    end
end
