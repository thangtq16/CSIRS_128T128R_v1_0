classdef nr5G_SRSAdvanced_Dialog < wirelessWaveformApp.nr5G_Dialog
    % Interface to all SRS properties that are not present in the basic table

    %   Copyright 2022-2024 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Constant, Hidden, Access = private)
        DefaultCfg = nrWavegenSRSConfig;
    end

    properties (Hidden)
        TitleString = 'SRS 1 (Advanced Configuration)'

        LabelType = 'charEdit';
        LabelLabel
        LabelGUI

        CyclicShiftType = 'numericPopup';
        CyclicShiftDropDown = string(nrWavegenSRSConfig.CyclicShift_Options);
        CyclicShiftGUI
        CyclicShiftLabel

        GroupSeqHoppingType = 'charPopup';
        GroupSeqHoppingDropDown = nrWavegenSRSConfig.GroupSeqHopping_Values;
        GroupSeqHoppingGUI
        GroupSeqHoppingLabel

        NSRSIDType = 'numericEdit';
        NSRSIDLabel
        NSRSIDGUI

        SRSPositioningType = 'checkbox'
        SRSPositioningLabel
        SRSPositioningGUI

        FrequencyScalingFactorType = 'numericPopup';
        FrequencyScalingFactorDropDown = string(nrWavegenSRSConfig.FreqScaling_Options);
        FrequencyScalingFactorGUI
        FrequencyScalingFactorLabel

        EnableStartRBHoppingType = 'checkbox'
        EnableStartRBHoppingLabel
        EnableStartRBHoppingGUI

        StartRBIndexType = 'numericPopup';
        StartRBIndexDropDown = string(nrWavegenSRSConfig.StartRBIndex_Options);
        StartRBIndexGUI
        StartRBIndexLabel

    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_SRSAdvanced_Dialog(parent, fig)
            obj@wirelessWaveformApp.nr5G_Dialog(parent, fig, wirelessWaveformApp.nr5G_SRSAdvanced_Dialog.DefaultCfg); % call base constructor

            % Callbacks needed either for validation of edit fields or for
            % changes to visibility of other controls
            obj.LabelGUI.(obj.Callback)                  = @(src,evnt) updateCache(obj);
            obj.FrequencyScalingFactorGUI.(obj.Callback) = @(src,evnt) FrequencyScalingFactorChangedGUI(obj,src,evnt);
            obj.StartRBIndexGUI.(obj.Callback)           = @(src,evnt) StartRBIndexChangedGUI(obj,src,evnt);

        end

        function adjustDialog(obj)
            obj.LabelGUI.Tag = 'SRSLabel';
            obj.CyclicShiftGUI.Tag = 'CyclicShift';
            obj.GroupSeqHoppingGUI.Tag = 'GroupSeqHopping';
            obj.NSRSIDGUI.Tag = 'NSRSID';
            obj.SRSPositioningGUI.Tag = 'SRSPositioning';
            obj.FrequencyScalingFactorGUI.Tag = 'FrequencyScalingFactor';
            obj.EnableStartRBHoppingGUI.Tag = 'EnableStartRBHopping';
            obj.StartRBIndexGUI.Tag = 'StartRBIndex';
        end

        function props = displayOrder(~)
            props = {'Label'; 'CyclicShift'; 'GroupSeqHopping'; 'NSRSID'; 'SRSPositioning'; 'FrequencyScalingFactor'; 'StartRBIndex';'EnableStartRBHopping'};
        end

        function restoreDefaults(obj)
            % This method is called in the baseclass constructor to set the
            % default values of the properties
            % Get the same defaults with programmatic objects
            cfg = wirelessWaveformApp.nr5G_UL_Tabs.DefaultCfg;
            ch = cfg.SRS{1};

            obj.Label = ch.Label;
            obj.CyclicShift = ch.CyclicShift;
            obj.GroupSeqHopping = ch.GroupSeqHopping;
            obj.NSRSID = ch.NSRSID;
            obj.SRSPositioning = ch.SRSPositioning;

            obj.FrequencyScalingFactor = ch.FrequencyScalingFactor;
            obj.EnableStartRBHopping = ch.EnableStartRBHopping;
            obj.StartRBIndex = ch.StartRBIndex;

        end

    end

    % Validators and visibility updates
    methods (Access = private)
        function FrequencyScalingFactorChangedGUI(obj, src, evnt)
            % Adjust StartRBIndex dropdown if it exceeds
            % FrequencyScalingFactor-1
            adjustStartRBIndex(obj);

            % Update the property using the default callback
            updateProperty(obj,src,evnt,'FrequencyScalingFactor');
        end

        function StartRBIndexChangedGUI(obj, src, evnt)
            % Adjust StartRBIndex dropdown if it exceeds
            % FrequencyScalingFactor-1
            adjustStartRBIndex(obj);

            % Update the property using the default callback
            updateProperty(obj,src,evnt,'StartRBIndex');
        end

        function adjustStartRBIndex(obj)
            % Adjust StartRBIndex dropdown if it exceeds
            % FrequencyScalingFactor-1
            obj.StartRBIndex = min(obj.StartRBIndex,obj.FrequencyScalingFactor-1);
        end
    end

    % Custom getters/setters
    methods
        function out = get.ChannelName(~)
            out = 'SRS';
        end
    end
end
