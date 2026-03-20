classdef nr5G_PXSCHPTRS_Dialog < wirelessWaveformApp.nr5G_Dialog
    % Interface to properties that are common for PXSCH PTRS

    %   Copyright 2024 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
    end

    properties (Hidden)
        EnablePTRSType = 'checkbox'
        EnablePTRSGUI
        EnablePTRSLabel

        TimeDensityType = 'numericPopup'
        TimeDensityDropDown = {'1', '2', '4'}
        TimeDensityLabel
        TimeDensityGUI

        FrequencyDensityType = 'numericPopup'
        FrequencyDensityDropDown = {'2', '4'}
        FrequencyDensityLabel
        FrequencyDensityGUI

        REOffsetType = 'charPopup'
        REOffsetDropDown = {'00', '01', '10', '11'}
        REOffsetLabel
        REOffsetGUI

        PTRSPortSetType = 'numericEdit'
        PTRSPortSetLabel
        PTRSPortSetGUI

        PowerType = 'numericEdit'
        PowerGUI
        PowerLabel
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_PXSCHPTRS_Dialog(parent, fig, invisibleProps, cfg)
            obj@wirelessWaveformApp.nr5G_Dialog(parent, fig, cfg); % call base constructor

            % Callbacks needed either for validation of edit fields, for
            % changes to visibility of other controls, or for grid live update
            obj.EnablePTRSGUI.(obj.Callback)       = @(src,evnt)enabledChangedGUI(obj);
            obj.PowerGUI.(obj.Callback)            = @(src,evnt) updateProperty(obj,src,evnt,"PTRSPower",Fieldnames="Power");
            obj.TimeDensityGUI.(obj.Callback)      = @(src,evnt) updateProperty(obj,src,evnt,"TimeDensity",Config=obj.Config.PTRS);
            obj.FrequencyDensityGUI.(obj.Callback) = @(src,evnt) updateProperty(obj,src,evnt,"FrequencyDensity",Config=obj.Config.PTRS);
            obj.REOffsetGUI.(obj.Callback)         = @(src,evnt) updateProperty(obj,src,evnt,"REOffset",Config=obj.Config.PTRS);
            obj.PTRSPortSetGUI.(obj.Callback)      = @(src,evnt) updateProperty(obj,src,evnt,"PTRSPortSet",Config=obj.Config.PTRS);

            if nargin > 2
                % Hide the properties that are not applicable at the time of
                % construction (this private property is used in displayOrder)
                invisibleProps = cellstr(invisibleProps);
                propsDlg = displayOrder(obj);
                propsInvisible = ismember(propsDlg, invisibleProps);
                setVisible(obj, propsDlg(propsInvisible), false);
            end
        end

        function needRepaint = updateControlsVisibility(obj)
            needRepaint = enabledChanged(obj);
        end

        function adjustDialog(obj)
            % PT-RS is turned off by default, so hide everything
            setVisible(obj, {'Power', 'TimeDensity', 'FrequencyDensity', 'REOffset', 'PTRSPortSet'}, false);

            % avoid conflict with DMRS:
            obj.PowerGUI.Tag = 'Power_PTRS';
        end

        function needRepaint = enabledChanged(obj)
            % Turn on or off all other knobs, based on the Enable flag
            oldVal = isVisible(obj, 'Power');
            props = displayOrder(obj);
            setVisible(obj, setdiff(props, 'EnablePTRS'), obj.EnablePTRS);
            needRepaint = xor(oldVal, obj.EnablePTRS);
        end
    end

    % Validators and visibility updates
    methods (Access = private)
        function enabledChangedGUI(obj, ~)
            needRepaint = enabledChanged(obj);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                layoutUIControls(obj);
                updateCache(obj);

                % synchronize with Enable checkbox in the PXSCH table
                dlg = obj.CurrentDialog;
                currPXSCH = wirelessWaveformApp.internal.Utility.getSingleSelection(dlg.pxschTable.Selection);
                dlg.pxschWaveConfig{currPXSCH}.EnablePTRS = obj.EnablePTRS;
                applyConfiguration(dlg.pxschTable, dlg.pxschWaveConfig);

                % Update resource grid and RE mapping
                updateGrids(obj);
            end
        end
    end
end
