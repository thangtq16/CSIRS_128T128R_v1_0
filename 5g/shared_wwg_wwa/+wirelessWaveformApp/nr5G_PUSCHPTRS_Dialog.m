classdef nr5G_PUSCHPTRS_Dialog < wirelessWaveformApp.nr5G_PXSCHPTRS_Dialog
    % Interface to PUSCH PTRS properties

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Hidden)
        TitleString = 'PUSCH 1 PT-RS'

        NumPTRSSamplesType = 'numericPopup'
        NumPTRSSamplesDropDown = {'2', '4'}
        NumPTRSSamplesLabel
        NumPTRSSamplesGUI

        NumPTRSGroupsType = 'numericPopup'
        NumPTRSGroupsDropDown = {'2', '4', '8'}
        NumPTRSGroupsLabel
        NumPTRSGroupsGUI

        NIDType = 'numericEdit'
        NIDLabel
        NIDGUI
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_PUSCHPTRS_Dialog(parent, fig, invisibleProps)
            obj@wirelessWaveformApp.nr5G_PXSCHPTRS_Dialog(parent, fig, invisibleProps, wirelessWaveformApp.nr5G_PUSCHAdvanced_Dialog.DefaultCfg); % call base constructor

            % Callbacks needed either for validation of edit fields, for
            % changes to visibility of other controls, or for grid live update
            obj.NumPTRSSamplesGUI.(obj.Callback) = @(src,evnt) updateProperty(obj,src,evnt,"NumPTRSSamples",Config=obj.Config.PTRS);
            obj.NumPTRSGroupsGUI.(obj.Callback)  = @(src,evnt) updateProperty(obj,src,evnt,"NumPTRSGroups",Config=obj.Config.PTRS);
            obj.NIDGUI.(obj.Callback)            = @(src,evnt) updateProperty(obj,src,evnt,"NID",Config=obj.Config.PTRS);
        end

        function adjustDialog(obj)
            adjustDialog@wirelessWaveformApp.nr5G_PXSCHPTRS_Dialog(obj);
            % PT-RS is turned off by default, so hide everything
            setVisible(obj, {'NumPTRSSamples', 'NumPTRSGroups', 'NID'}, false);

            % PUSCH PT-RS Port Set can also be 2-element vector (differently than PDSCH):
            set([obj.PTRSPortSetLabel obj.PTRSPortSetGUI], obj.Tooltip, getMsgString(obj, 'PUSCHPTRSPortSetTT'));
        end

        function props = displayOrder(~)
            props = {'EnablePTRS'; 'Power'; 'TimeDensity'; 'FrequencyDensity'; ...
                     'NumPTRSSamples'; 'NumPTRSGroups'; 'REOffset'; 'PTRSPortSet'; 'NID'};
        end

        function restoreDefaults(obj)
            % This method is called in the baseclass constructor to set the
            % default values of the properties
            % Get the same defaults with programmatic objects
            c = nrWavegenPUSCHConfig;
            obj.Power      = c.PTRSPower;
            obj.EnablePTRS = c.EnablePTRS;

            c = nrPUSCHPTRSConfig;
            obj.TimeDensity       = c.TimeDensity;
            obj.FrequencyDensity  = c.FrequencyDensity;
            obj.REOffset          = c.REOffset;
            obj.PTRSPortSet       = c.PTRSPortSet;
            obj.NumPTRSSamples    = c.NumPTRSSamples;
            obj.NumPTRSGroups     = c.NumPTRSGroups;
            obj.NID               = c.NID;
        end

        function needRepaint = enabledChanged(obj)
            % Get the current dialog
            currDlg = obj.Parent.AppObj.pParameters.CurrentDialog;
            pxschId = getAdvancedPXSCHID(currDlg);
            if isnan(pxschId)
                % Return early when the Advanced dialog is not yet loaded. This
                % happens at the time of mapping the PUSCH PT-RS dialog before
                % the mapping of PUSCH Advanced dialog.
                needRepaint = false;
                return
            end
            currPXSCH = wirelessWaveformApp.internal.Utility.getSingleSelection(currDlg.pxschTable.Selection);
            precod = currDlg.pxschWaveConfig{currPXSCH}.TransformPrecoding;

            % Turn on/off all other knobs, based on the EnablePTRS flag
            ptrsEnabled = obj.EnablePTRS;
            oldVal       = isVisible(obj, 'Power'); % = was EnablePTRS on?
            % Also get the value of Transform Precoding as some properties also
            % depend on that
            oldValPrecod = isVisible(obj, 'NID');

            % TimeDensity only visible when PT-RS is enabled
            setVisible(obj, 'TimeDensity', ptrsEnabled);

            % NumPTRSSamples, NumPTRSGroups, and NID only visible when
            % PT-RS is enabled and transform precoding is on
            setVisible(obj, {'NumPTRSSamples', 'NumPTRSGroups', 'NID'}, ptrsEnabled && precod);

            % Power, PTRSPortSet, FrequencyDensity, and REOffset only
            % visible when PT-RS is enabled and transform precoding is off
            setVisible(obj, {'Power', 'PTRSPortSet', 'FrequencyDensity', 'REOffset'}, ptrsEnabled && ~precod);

            needRepaint = xor(oldVal, ptrsEnabled) || (ptrsEnabled && xor(oldValPrecod, precod));
        end
    end

    % Custom getters/setters
    methods
        function out = get.ChannelName(~)
            out = 'PUSCH';
        end
    end
end
