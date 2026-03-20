classdef nr5G_PUSCHDMRS_Dialog < wirelessWaveformApp.nr5G_PXSCHDMRS_Dialog
    % Interface to properties that are unique for PUSCH DMRS

    %   Copyright 2020-2025 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Hidden)
        TitleString = 'PUSCH 1 DM-RS'

        GroupHoppingType = 'checkbox'
        GroupHoppingLabel
        GroupHoppingGUI

        SequenceHoppingType = 'checkbox'
        SequenceHoppingLabel
        SequenceHoppingGUI

        NRSIDType = 'numericEdit'
        NRSIDLabel
        NRSIDGUI

        DMRSUplinkR16Type = 'checkbox'
        DMRSUplinkR16GUI
        DMRSUplinkR16Label

        DMRSUplinkTransformPrecodingR16Type = 'checkbox'
        DMRSUplinkTransformPrecodingR16GUI
        DMRSUplinkTransformPrecodingR16Label
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_PUSCHDMRS_Dialog(parent, fig, invisibleProps)
            obj@wirelessWaveformApp.nr5G_PXSCHDMRS_Dialog(parent, fig, invisibleProps, wirelessWaveformApp.nr5G_PUSCHAdvanced_Dialog.DefaultCfg); % call base constructor

            % Callback needed either for validation of edit field or updating the
            % cached configuration object following a user interaction
            obj.GroupHoppingGUI.(obj.Callback)                    = @(src,evnt) updateProperty(obj,src,evnt,"GroupHopping",Config=obj.Config.DMRS);
            obj.SequenceHoppingGUI.(obj.Callback)                 = @(src,evnt) updateProperty(obj,src,evnt,"SequenceHopping",Config=obj.Config.DMRS);
            obj.NRSIDGUI.(obj.Callback)                           = @(src,evnt) updateProperty(obj,src,evnt,"NRSID",Config=obj.Config.DMRS);
            obj.DMRSUplinkR16GUI.(obj.Callback)                   = @(src,evnt) updateProperty(obj,src,evnt,"DMRSUplinkR16",Config=obj.Config.DMRS);
            obj.DMRSUplinkTransformPrecodingR16GUI.(obj.Callback) = @(src,evnt) DMRSUplinkTransformPrecodingR16ChangedGUI(obj);
        end

        function needRepaint = updateControlsVisibility(obj)
            needRepaint = false;

            dlg = obj.Parent.AppObj.pParameters.CurrentDialog;
            currPXSCH = wirelessWaveformApp.internal.Utility.getSingleSelection(dlg.pxschTable.Selection);
            if dlg.pxschWaveConfig{currPXSCH}.TransformPrecoding
                needRepaint = DMRSUplinkTransformPrecodingR16Changed(obj);
            end
        end

        function adjustDialog(obj)
            % GroupHopping, SequenceHopping, and NRSID do not appear by default
            setVisible(obj, {'GroupHopping', 'SequenceHopping', 'NRSID', 'DMRSUplinkTransformPrecodingR16'}, false);

            % For test symmetry with PDSCH:
            obj.DMRSTypeAPositionGUI.Tag = 'PUSCHDMRSTypeAPosition';
            obj.PowerGUI.Tag = 'Power_DMRS';
        end

        function props = displayOrder(~)
            props = {'Power'; 'DMRSConfigurationType'; 'DMRSTypeAPosition'; ...
                     'DMRSAdditionalPosition'; 'DMRSLength'; 'CustomSymbolSet'; ...
                     'DMRSPortSet'; 'NIDNSCID'; 'NSCID'; 'GroupHopping'; ...
                     'SequenceHopping'; 'NRSID'; 'NumCDMGroupsWithoutData'; ...
                     'DMRSUplinkR16'; 'DMRSUplinkTransformPrecodingR16'; 'DMRSEnhancedR18'};
        end

        function restoreDefaults(obj)
            % This method is called in the baseclass constructor to set the
            % default values of the properties
            % Get the same defaults with programmatic objects
            c = nrWavegenPUSCHConfig;
            obj.Power                   = c.DMRSPower;

            c = nrPUSCHDMRSConfig;
            obj.DMRSConfigurationType   = c.DMRSConfigurationType;
            obj.DMRSTypeAPosition       = c.DMRSTypeAPosition;
            obj.DMRSAdditionalPosition  = c.DMRSAdditionalPosition;
            obj.DMRSLength              = c.DMRSLength;
            obj.CustomSymbolSet         = c.CustomSymbolSet;
            obj.DMRSPortSet             = c.DMRSPortSet;
            obj.NIDNSCID                = c.NIDNSCID;
            obj.NSCID                   = c.NSCID;
            obj.GroupHopping            = c.GroupHopping;
            obj.SequenceHopping         = c.SequenceHopping;
            obj.NRSID                   = c.NRSID;
            obj.NumCDMGroupsWithoutData = c.NumCDMGroupsWithoutData;
            obj.DMRSUplinkR16           = c.DMRSUplinkR16;
            obj.DMRSUplinkTransformPrecodingR16 = c.DMRSUplinkTransformPrecodingR16;
            obj.DMRSEnhancedR18         = c.DMRSEnhancedR18;
        end
    end

    % Methods specific to this class and that are currently used by
    % neighboring classes
    methods (Access = public)
        function DMRSUplinkTransformPrecodingR16ChangedGUI(obj,~)
            % Update the visibility of NIDNSCID and NSCID based on whether
            % DMRSUplinkTransformPrecodingR16 is checked or not
            needRepaint = DMRSUplinkTransformPrecodingR16Changed(obj);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                layoutUIControls(obj);
                updateCache(obj);
            end
        end

        function needRepaint = DMRSUplinkTransformPrecodingR16Changed(obj,~)
            % Update the visibility of NIDNSCID and NSCID based on whether
            % DMRSUplinkTransformPrecodingR16 is checked or not

            oldVis = isVisible(obj, 'NIDNSCID');
            newVis = isVisible(obj, 'DMRSUplinkTransformPrecodingR16') && obj.DMRSUplinkTransformPrecodingR16;
            needRepaint = xor(oldVis, newVis);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                setVisible(obj, {'NIDNSCID', 'NSCID'}, newVis);
            end
        end
    end

    methods (Access = protected)
        function DMRSConfigurationTypeChangedGUI(obj,src,evnt)
            function validationFcn(in,tp)
                coder.internal.errorIf(tp && in~=1,'nr5g:nrPUSCHDMRSConfig:InvalidDMRSConfigTypeWithTP',double(in));
            end
            tp = getTP(obj);
            updateProperty(obj,src,evnt,"DMRSConfigurationType",Config=obj.Config.DMRS,ValidationFunction=@(val)validationFcn(val,tp));
        end
        function NumCDMGroupsWithoutDataChangedGUI(obj,src,evnt)
            function validationFcn(in,tp)
                coder.internal.errorIf(tp && in~=2,'nr5g:nrPUSCHDMRSConfig:InvalidNumCDMGrpsWODataWithTP',double(in));
            end
            tp = getTP(obj);
            updateProperty(obj,src,evnt,"NumCDMGroupsWithoutData",Config=obj.Config.DMRS,ValidationFunction=@(val)validationFcn(val,tp));
        end
    end
    methods (Access = private)
        function tp = getTP(obj)
            currPXSCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.CurrentDialog.pxschTable.Selection);
            tp = obj.CurrentDialog.pxschWaveConfig{currPXSCH}.TransformPrecoding;
        end
    end

    % Custom getters/setters
    methods
        function out = get.ChannelName(~)
            out = 'PUSCH';
        end
    end
end
