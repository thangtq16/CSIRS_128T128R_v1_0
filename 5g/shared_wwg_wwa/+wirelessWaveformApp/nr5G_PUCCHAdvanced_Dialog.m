classdef nr5G_PUCCHAdvanced_Dialog < wirelessWaveformApp.nr5G_Dialog & ...
                                     wirelessWaveformApp.internal.interlaceBase
    % Interface to all PUCCH properties that are not present in the basic table

    %   Copyright 2021-2024 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Hidden)
        TitleString = 'PUCCH 1 (Advanced Configuration)'

        FormatType = 'numericText'
        FormatGUI
        FormatLabel

        LabelType = 'charEdit'
        LabelLabel
        LabelGUI

        Modulation_f0Type = 'charText'
        Modulation_f0GUI
        Modulation_f0Label

        Modulation_f1Type = 'charText'
        Modulation_f1GUI
        Modulation_f1Label

        Modulation_f2Type = 'charText'
        Modulation_f2GUI
        Modulation_f2Label

        Modulation_f34Type = 'charPopup'
        Modulation_f34DropDown = {'pi/2-BPSK', 'QPSK'}
        Modulation_f34Label
        Modulation_f34GUI

        FrequencyHoppingType = 'charPopup'
        FrequencyHoppingDropDown = {'neither', 'intraSlot', 'interSlot'}
        FrequencyHoppingLabel
        FrequencyHoppingGUI

        SecondHopStartPRBType = 'numericEdit'
        SecondHopStartPRBLabel
        SecondHopStartPRBGUI

        PUCCHGroupHoppingType = 'charPopup'
        PUCCHGroupHoppingDropDown = {'neither', 'enable', 'disable'}
        PUCCHGroupHoppingLabel
        PUCCHGroupHoppingGUI

        HoppingIDType = 'numericEdit'
        HoppingIDLabel
        HoppingIDGUI

        InitialCyclicShiftType = 'numericPopup'
        InitialCyclicShiftDropDown = {'0','1','2','3','4','5','6','7','8','9','10','11'}
        InitialCyclicShiftLabel
        InitialCyclicShiftGUI

        SpreadingFactorType = 'numericPopup'
        SpreadingFactorDropDown = {'2', '4'}
        SpreadingFactorLabel
        SpreadingFactorGUI

        OCCI_f1Type = 'numericPopup'
        OCCI_f1DropDown = {'0','1','2','3','4','5','6'}
        OCCI_f1Label
        OCCI_f1GUI

        OCCI_f4Type = 'numericPopup'
        OCCI_f4DropDown = {'0','1','2','3'}
        OCCI_f4Label
        OCCI_f4GUI

        NID0Type = 'numericEdit'
        NID0Label
        NID0GUI

        AdditionalDMRSType = 'checkbox'
        AdditionalDMRSLabel
        AdditionalDMRSGUI

        DMRSUplinkTransformPrecodingR16Type = 'checkbox'
        DMRSUplinkTransformPrecodingR16GUI
        DMRSUplinkTransformPrecodingR16Label

        DMRSPowerType = 'numericEdit'
        DMRSPowerLabel
        DMRSPowerGUI

        Interlacing_f0123Type = 'checkbox'
        Interlacing_f0123Label
        Interlacing_f0123GUI

        InterlaceIndex_f0123Type = 'numericEdit'
        InterlaceIndex_f0123Label
        InterlaceIndex_f0123GUI

        RBSetIndex_f0123Type = 'numericPopup'
        RBSetIndex_f0123DropDown = {'0','1','2','3','4'}
        RBSetIndex_f0123Label
        RBSetIndex_f0123GUI

        SpreadingFactor_f23Type = 'numericPopup'
        SpreadingFactor_f23DropDown = {'1','2','4'}
        SpreadingFactor_f23Label
        SpreadingFactor_f23GUI

        OCCI_f23Type = 'numericPopup'
        OCCI_f23DropDown = {'0','1','2','3'}
        OCCI_f23Label
        OCCI_f23GUI
    end

    properties (Dependent = true, Access = protected)
        % abstract dependent properties in interlaceBase
        CurrentChannelType;
        CurrentChannelIdx;
        CurrentCacheConfig;
        SupportInterlacing;
    end

    properties (Access = private)
        InvisibleProperties = {}       % Defines the properties that are invisible (at the time of dialog creation)
        DependentDialogs = {'wirelessWaveformApp.nr5G_PUCCHUCI_Dialog'}        % List of dependent dialog classes
        EnableDependentDialog = []     % Visibility flag of dependent dialog class (true -> enable)
        DefaultCfg = configureDictionary("double","cell"); % Stores the default configurations for each PUCCH format
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_PUCCHAdvanced_Dialog(parent, fig, invisibleProps)
            obj@wirelessWaveformApp.nr5G_Dialog(parent, fig, wirelessWaveformApp.nr5G_PUCCH_Tab.getDefaultConfigObject(0)); % call base constructor
            for format = 0:4
                obj.DefaultCfg(format) = {wirelessWaveformApp.nr5G_PUCCH_Tab.getDefaultConfigObject(format)};
            end

            % Callbacks needed either for validation of edit fields or for
            % changes to visibility of other controls
            obj.LabelGUI.(obj.Callback)                = @(src,evnt) labelChangedGUI(obj);
            obj.Modulation_f34GUI.(obj.Callback)       = @(src,evnt) modChangedGUI(obj);
            obj.FrequencyHoppingGUI.(obj.Callback)     = @(src,evnt) freqHopChangedGUI(obj);
            obj.SecondHopStartPRBGUI.(obj.Callback)    = @(src,evnt) updatePUCCHProperty(obj,src,evnt,"SecondHopStartPRB");
            obj.PUCCHGroupHoppingGUI.(obj.Callback)    = @(src,evnt) groupHoppingChangedGUI(obj);
            obj.HoppingIDGUI.(obj.Callback)            = @(src,evnt) updatePUCCHProperty(obj,src,evnt,"HoppingID");
            obj.InitialCyclicShiftGUI.(obj.Callback)   = @(src,evnt) updatePUCCHProperty(obj,src,evnt,"InitialCyclicShift");
            obj.SpreadingFactorGUI.(obj.Callback)      = @(src,evnt) spreadingFactorChangedGUI(obj,src,evnt);
            obj.OCCI_f1GUI.(obj.Callback)              = @(src,evnt) updatePUCCHProperty(obj,src,evnt,"OCCI",FieldNames="OCCI_f1");
            obj.OCCI_f4GUI.(obj.Callback)              = @(src,evnt) occif4ChangedGUI(obj,src,evnt);
            obj.NID0GUI.(obj.Callback)                 = @(src,evnt) updatePUCCHProperty(obj,src,evnt,"NID0");
            obj.AdditionalDMRSGUI.(obj.Callback)       = @(src,evnt) updatePUCCHProperty(obj,src,evnt,"AdditionalDMRS");
            obj.DMRSUplinkTransformPrecodingR16GUI.(obj.Callback) = @(src,evnt) dmrsTransformPrecodingChangedGUI(obj);
            obj.DMRSPowerGUI.(obj.Callback)            = @(src,evnt) updatePUCCHProperty(obj,src,evnt,"DMRSPower");
            obj.Interlacing_f0123GUI.(obj.Callback)    = @(src,evnt) interlacingf0123ChangedGUI(obj);
            obj.InterlaceIndex_f0123GUI.(obj.Callback) = @(src,evnt) interlaceindexf0123ChangedGUI(obj,src,evnt);
            obj.RBSetIndex_f0123GUI.(obj.Callback)     = @(src,evnt) updatePUCCHProperty(obj,src,evnt,"RBSetIndex",FieldNames="RBSetIndex_f0123");
            obj.SpreadingFactor_f23GUI.(obj.Callback)  = @(src,evnt) updatePUCCHProperty(obj,src,evnt,"SpreadingFactor",FieldNames="SpreadingFactor_f23");
            obj.OCCI_f23GUI.(obj.Callback)             = @(src,evnt) updatePUCCHProperty(obj,src,evnt,"OCCI",FieldNames="OCCI_f23");

            if nargin > 2
                % Hide the properties that are not applicable at the time of
                % construction
                invisibleProps = cellstr(invisibleProps);
                propsDlg = displayOrder(obj);  % Properties that are visible in this dialog
                propsInvisible = ismember(propsDlg, invisibleProps);
                obj.InvisibleProperties = propsDlg(propsInvisible);
            end

            if ~isempty(obj.DependentDialogs)
                % Check the dependent dialog classes and hide them as applicable
                numDependentClasses = numel(obj.DependentDialogs);
                enableDependentClasses = true(1,numDependentClasses);
                for i = 1:numDependentClasses
                    dlg = obj.Parent.AppObj.pParameters.DialogsMap(obj.DependentDialogs{i});
                    propsDlg = displayOrder(dlg);  % Properties that are visible in this dialog
                    propsInvisible = ismember(propsDlg, invisibleProps);
                    if all(propsInvisible)
                        % Disable that particular entry
                        enableDependentClasses(i) = false;
                    end
                end
                obj.EnableDependentDialog = enableDependentClasses;
            end
        end

        function needRepaint = updateControlsVisibility(obj)
            % Update visibility of all controls in this panel.

            % Make applicable properties visible
            props = getPropertiesForFormat(obj.Format);
            setVisible(obj, props, true);

            % Hide non-applicable properties
            allProps = displayOrder(obj);
            notProps = allProps(~ismember(allProps,props));
            setVisible(obj, notProps, false);

            % Adjust visibility for dependent properties common to all formats
            needRepaint = freqHopChanged(obj);

            % Adjust visibility of DMRSUplinkTransformPrecodingR16
            needRepaint = updateDMRSUplinkTransformPrecodingVisibility(obj) || needRepaint;

            % Adjust visibility of NID0
            needRepaint = dmrsTransformPrecodingChanged(obj) || needRepaint;

            % update interlacing related properties and table display
            needRepaint = updateInterlacingVis(obj) || needRepaint;

            % Ensure the always-invisible properties are indeed hidden
            setVisible(obj, obj.InvisibleProperties, false);
        end

        function adjustDialog(obj)
            % SecondHopStart, DMRSUplinkTransformPrecodingR16, InterlaceIndex_f0123, RBSetIndex_f0123, SpreadingFactor_f23 and OCCI_f23 don't appear by default:
            setVisible(obj, {'SecondHopStartPRB', 'DMRSUplinkTransformPrecodingR16', 'InterlaceIndex_f0123', ...
                             'RBSetIndex_f0123', 'SpreadingFactor_f23', 'OCCI_f23'}, false);
            % Make sure all tags are unique. Otherwise there is a conflict with
            % other channels
            obj.LabelGUI.Tag = 'PUCCHLabel';
            obj.FrequencyHoppingGUI.Tag = 'PUCCHFrequencyHopping';
            obj.SecondHopStartPRBGUI.Tag = 'PUCCHSecondHopStartPRB';
            % Decorate the tags for the remaining properties with the channel name
            obj.Modulation_f0GUI.Tag = 'PUCCHModulation_f0';
            obj.Modulation_f1GUI.Tag = 'PUCCHModulation_f1';
            obj.Modulation_f2GUI.Tag = 'PUCCHModulation_f2';
            obj.Modulation_f34GUI.Tag = 'PUCCHModulation_f34';
            obj.OCCI_f1GUI.Tag = 'PUCCHOCCI_f1';
            obj.OCCI_f4GUI.Tag = 'PUCCHOCCI_f4';
            obj.FormatGUI.Tag = 'PUCCHFormat';
            obj.HoppingIDGUI.Tag = 'PUCCHHoppingID';
            obj.InitialCyclicShiftGUI.Tag = 'PUCCHInitialCyclicShift';
            obj.SpreadingFactorGUI.Tag = 'PUCCHSpreadingFactor';
            obj.NID0GUI.Tag = 'PUCCHNID0';
            obj.AdditionalDMRSGUI.Tag = 'PUCCHAdditionalDMRS';
            obj.DMRSUplinkTransformPrecodingR16GUI.Tag = 'PUCCHDMRSUplinkTransformPrecodingR16';
            obj.DMRSPowerGUI.Tag = 'PUCCHDMRSPower';
            obj.Interlacing_f0123GUI.Tag = 'PUCCHInterlacing_f0123';
            obj.InterlaceIndex_f0123GUI.Tag = 'PUCCHInterlaceIndex_f0123';
            obj.RBSetIndex_f0123GUI.Tag = 'PUCCHRBSetIndex_f0123';
            obj.SpreadingFactor_f23GUI.Tag = 'PUCCHSpreadingFactor_f23';
            obj.OCCI_f23GUI.Tag = 'PUCCHOCCI_f23';
        end

        function props = displayOrder(~)
            props = {'Format'; 'Label'; 'Modulation_f0'; 'Modulation_f1'; 'Modulation_f2'; 'Modulation_f34'; ...
                     'Interlacing_f0123'; 'InterlaceIndex_f0123'; 'RBSetIndex_f0123'; ...
                     'SpreadingFactor_f23'; 'OCCI_f23'; ...
                     'FrequencyHopping'; 'SecondHopStartPRB'; ...
                     'PUCCHGroupHopping'; 'HoppingID'; ...
                     'InitialCyclicShift'; 'SpreadingFactor'; 'OCCI_f1'; 'OCCI_f4'; 'NID0'; ...
                     'DMRSPower'; 'AdditionalDMRS'; 'DMRSUplinkTransformPrecodingR16'};
        end

        function restoreDefaults(obj)
            % Get the same defaults with programmatic objects
            c0 = wirelessWaveformApp.nr5G_PUCCH_Tab.defaultPUCCH0Config;
            c2 = wirelessWaveformApp.nr5G_PUCCH_Tab.defaultPUCCH2Config;
            c4 = wirelessWaveformApp.nr5G_PUCCH_Tab.defaultPUCCH4Config;
            obj.Format                  = 0;
            obj.Label                   = c0.Label;
            obj.Modulation_f0           = 'Z-Chu';
            obj.Modulation_f1           = 'BPSK';
            obj.Modulation_f2           = 'QPSK';
            obj.Modulation_f34          = obj.Modulation_f34DropDown{2};
            obj.FrequencyHopping        = c0.FrequencyHopping;
            obj.SecondHopStartPRB       = c0.SecondHopStartPRB;
            obj.PUCCHGroupHopping       = c0.GroupHopping;
            obj.HoppingID               = c0.HoppingID;
            obj.InitialCyclicShift      = c0.InitialCyclicShift;
            obj.SpreadingFactor         = c4.SpreadingFactor;
            obj.OCCI_f1                 = c4.OCCI; % Default for format 1 and 4 are the same
            obj.OCCI_f4                 = c4.OCCI;
            obj.NID0                    = c2.NID0;
            obj.AdditionalDMRS          = c4.AdditionalDMRS;
            obj.DMRSUplinkTransformPrecodingR16 = c4.DMRSUplinkTransformPrecodingR16;
            obj.DMRSPower               = c4.DMRSPower;
            obj.Interlacing_f0123       = c0.Interlacing;
            obj.InterlaceIndex_f0123    = c0.InterlaceIndex;
            obj.RBSetIndex_f0123        = c0.RBSetIndex;
            obj.SpreadingFactor_f23     = c2.SpreadingFactor;
            obj.OCCI_f23                = c2.OCCI;

            % Set default visibility
            updateControlsVisibility(obj);

        end

        %% Stack Advanced/UCI vertically:
        function cellDialogs = getDialogsPerColumn(obj)
            cellDialogs{1} = {obj};
            for dialogIdx = 1:numel(obj.DependentDialogs)
                if obj.EnableDependentDialog(dialogIdx)
                    if isKey(obj.Parent.AppObj.pParameters.DialogsMap,obj.DependentDialogs{dialogIdx})
                        cellDialogs{1} = [cellDialogs{1}(:)' {obj.Parent.AppObj.pParameters.DialogsMap(obj.DependentDialogs{dialogIdx})}];
                    end
                end
            end
        end
    end

    % Methods specific to this class and that are currently used by
    % neighboring classes
    methods (Access = public)
        function modChanged(obj)
            switch obj.Format
                case 0
                    modulation = obj.Modulation_f0;
                case 1
                    modulation = obj.Modulation_f1;
                case 2
                    modulation = obj.Modulation_f2;
                otherwise
                    modulation = obj.Modulation_f34;

                    % Update the visibility of DMRSUplinkTransformPrecodingR16
                    needRepaint = updateDMRSUplinkTransformPrecodingVisibility(obj);
                    % Update the visibility of NID0
                    needRepaint = dmrsTransformPrecodingChanged(obj) || needRepaint;

                    if needRepaint
                        % do re-layout only when it is needed, because it is expensive
                        layoutUIControls(obj);
                    end
            end
            % Update the Modulation read-only cell in the main PUCCH table
            updatePropertyValues(obj.CurrentDialog.pucchTable, PropertyName="Modulation", NewLIst=modulation);
        end
    end

    % Validators and visibility updates
    methods (Access = private)
        function formatChanged(obj)
            props = getPropertiesForFormat(obj.Format);
            c = wirelessWaveformApp.nr5G_PUCCH_Tab.getDefaultConfigObject(obj.Format);
            % Assign the default values
            fnames = fieldnames(c);
            fnames = fnames(ismember(fnames,props));
            for n = 1:length(fnames)
                obj.(fnames{n}) = c.(fnames{n});
            end
            switch obj.Format
                case 0
                    obj.PUCCHGroupHopping = c.GroupHopping;
                    obj.Modulation_f0 = 'Z-Chu';
                    obj.Interlacing_f0123 = c.Interlacing;
                    obj.InterlaceIndex_f0123 = c.InterlaceIndex;
                    obj.RBSetIndex_f0123 = c.RBSetIndex;
                case 1
                    obj.PUCCHGroupHopping = c.GroupHopping;
                    obj.OCCI_f1 = c.OCCI;
                    obj.Interlacing_f0123 = c.Interlacing;
                    obj.InterlaceIndex_f0123 = c.InterlaceIndex;
                    obj.RBSetIndex_f0123 = c.RBSetIndex;

                    % Get the modulation scheme based on the number of
                    % UCI bits
                    className = 'wirelessWaveformApp.nr5G_PUCCHUCI_Dialog';
                    if ~isKey(obj.Parent.AppObj.pParameters.DialogsMap, className)
                        % Assume default value for NumUCIBits of 1
                        obj.Modulation_f1 = 'BPSK';
                    else
                        % Get the number of UCI bits from the UCI panel
                        dlg = obj.Parent.AppObj.pParameters.DialogsMap(className);
                        if dlg.NumUCIBits_f01==2
                            obj.Modulation_f1 = 'QPSK';
                        else
                            obj.Modulation_f1 = 'BPSK';
                        end
                    end
                case 2
                    obj.Modulation_f2 = 'QPSK';
                    obj.Interlacing_f0123 = c.Interlacing;
                    obj.InterlaceIndex_f0123 = c.InterlaceIndex;
                    obj.RBSetIndex_f0123 = c.RBSetIndex;
                    obj.SpreadingFactor_f23 = c.SpreadingFactor;
                    obj.OCCI_f23 = c.OCCI;
                case 3
                    obj.PUCCHGroupHopping = c.GroupHopping;
                    obj.Modulation_f34 = c.Modulation;
                    obj.Interlacing_f0123 = c.Interlacing;
                    obj.InterlaceIndex_f0123 = c.InterlaceIndex;
                    obj.RBSetIndex_f0123 = c.RBSetIndex;
                    obj.SpreadingFactor_f23 = c.SpreadingFactor;
                    obj.OCCI_f23 = c.OCCI;
                otherwise % format 4
                    obj.PUCCHGroupHopping = c.GroupHopping;
                    obj.Modulation_f34 = c.Modulation;
                    obj.OCCI_f4 = c.OCCI;
            end

            % Update visibility of dependent properties
            updateControlsVisibility(obj);

            % Update interlacing-related properties in the PUCCH table
            updateTableDisplayInterlacing(obj);
        end

        function ME = updatePUCCHProperty(obj,src,evnt,propName,nvargs)
            % Use error throwing behavior of nrWavegenPUCCHxConfig
            arguments
                obj
                src
                evnt
                propName (1,1) string
                nvargs.FieldNames (1,:) string = "";
            end

            cfg = obj.DefaultCfg{obj.Format};
            ME = updateProperty(obj, src, evnt, propName, FieldNames=nvargs.FieldNames, Config=cfg);
        end

        function labelChanged(obj)
            % Synchronize with the Label cell in the PUCCH table
            dlg = obj.CurrentDialog;
            currPUCCH = wirelessWaveformApp.internal.Utility.getSingleSelection(dlg.pucchTable.Selection);
            dlg.pucchWaveConfig{currPUCCH}.Label = obj.Label;
            applyConfiguration(dlg.pucchTable, dlg.pucchWaveConfig);
        end
        function labelChangedGUI(obj,~)
            labelChanged(obj);
            updateCache(obj);
        end

        function modChangedGUI(obj,~)
            modChanged(obj);
            updateCacheAndGrids(obj); % Do live grid update
        end

        function needRepaint = freqHopChanged(obj)
            oldVis = isVisible(obj, 'SecondHopStartPRB');
            newVis = ~strcmpi(obj.FrequencyHopping, 'neither');
            needRepaint = xor(oldVis, newVis);
            if needRepaint
                setVisible(obj, 'SecondHopStartPRB', newVis);
            end
        end

        function freqHopChangedGUI(obj, ~)
            needRepaint = freqHopChanged(obj);
            updateCacheAndGrids(obj); % Do live grid update
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                layoutUIControls(obj);
            end
        end

        function groupHoppingChangedGUI(obj, ~)
            % Make sure the banner is up to date with the potential group
            % hopping warning
            dlg = obj.CurrentDialog;

            % Check if sequence hopping is valid for this PUCCH format
            currPUCCH = wirelessWaveformApp.internal.Utility.getSingleSelection(dlg.pucchTable.Selection);
            pucch = dlg.pucchWaveConfig{currPUCCH};
            Mrb = numel(unique(pucch.PRBSet(:)));
            pucch.GroupHopping = obj.PUCCHGroupHopping;
            [~,isSeqHopValid] = nr5g.internal.wavegen.validatePUCCHSequenceHopping(pucch, obj.Format, Mrb);
            if ~isSeqHopValid
                msg = getString(message('nr5g:nrWaveformGenerator:InvalidGroupHopping',currPUCCH,obj.Format));
                updateConfigDiagnostic(dlg, msg, MessageType="warning");
            else
                % Update the cache
                updateCache(obj);
                % Clear the message, without clearing any potential other
                % issues
                updateAppConfigDiagnostic(dlg, []);
            end
        end

        function spreadingFactorChangedGUI(obj, src, evnt)
            % First, check that the standalone value is correct
            ME = updatePUCCHProperty(obj, src, evnt, "SpreadingFactor");
            
            % If there is no error, do the cross-parameter validation
            if isempty(ME)
                validateOCCIvsSF(obj);
            end
        end

        function occif4ChangedGUI(obj, src, evnt)
            % First, check that the standalone value is correct
            ME = updatePUCCHProperty(obj, src, evnt, "OCCI", FieldNames="OCCI_f4");
            
            % If there is no error, do the cross-parameter validation
            if isempty(ME)
                validateOCCIvsSF(obj);
            end
        end

        function validateOCCIvsSF(obj)
            % Cross-parameter validation of OCCI vs SpreadingFactor for
            % PUCCH format 4
            cfg = obj.DefaultCfg{obj.Format};
            cfg.SpreadingFactor = obj.SpreadingFactor;
            cfg.OCCI = obj.OCCI_f4;
            e = [];
            try
                validateConfig(cfg);
            catch e
            end

            % Update error message
            updateAppConfigDiagnostic(obj,e);
        end

        function needRepaint = dmrsTransformPrecodingChanged(obj)
            % Update the visibility of the properties depending on
            % DMRSUplinkTransformPrecodingR16.
            needRepaint = false;
            if any(obj.Format == [3, 4]) % This is a no-op for formats 0,1,2
                oldVis = isVisible(obj, 'NID0');
                newVis = obj.DMRSUplinkTransformPrecodingR16 && strcmpi(obj.Modulation_f34,'pi/2-BPSK');
                needRepaint = xor(oldVis, newVis);
                if needRepaint
                    setVisible(obj, 'NID0', newVis);
                end
            end
        end
        function dmrsTransformPrecodingChangedGUI(obj)
            % Update the visibility of the properties depending on
            % DMRSUplinkTransformPrecodingR16.
            needRepaint = dmrsTransformPrecodingChanged(obj);
            updateCache(obj);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                layoutUIControls(obj);
            end
        end

        function needRepaint = interlacingChanged(obj, ~)

            % Update cache
            updateCache(obj);

            % Update relevant visibility
            propList = {'InterlaceIndex_f0123' 'RBSetIndex_f0123' 'FrequencyHopping' 'SecondHopStartPRB' 'SpreadingFactor_f23' 'OCCI_f23'};
            needRepaint = updateInterlacingVis(obj, propList);
        end
        function interlacingf0123ChangedGUI(obj)
            % Interlacing properties follow the following work flow:
            % 1. Check input validity, and if valid, proceed
            % 2. Update app internal config cache
            % 3. Update related visibility

            % Update the visibility of the Interlacing-dependent properties
            needRepaint = interlacingChanged(obj);

            % Update relevant visibility
            if needRepaint
                % Update grids
                updateGrids(obj);

                % do re-layout only when it is needed, because it is expensive
                layoutUIControls(obj);

                % Update the PRBSet column in the table based on interlacing
                updateTableDisplayInterlacing(obj);
            end

        end

        function interlaceindexf0123ChangedGUI(obj, src, evnt)
            if obj.Format == 4
                return; % no-op for format 4
            end

            % First, check that the standalone value is correct
            ME = updatePUCCHProperty(obj, src, evnt, "InterlaceIndex", FieldNames="InterlaceIndex_f0123");

            % If there is no error, update GUI visibility
            if isempty(ME)
                propList = {'SpreadingFactor_f23' 'OCCI_f23'};
                needRepaint = updateInterlacingVis(obj,propList);
                if needRepaint
                    layoutUIControls(obj); % update GUI objects visibility
                end
            end
        end

        function needRepaint = updateDMRSUplinkTransformPrecodingVisibility(obj)
            % Update the visibility of DMRSUplinkTransformPrecodingR16
            needRepaint = false;
            if any(obj.Format == [3, 4]) % This is a no-op for formats 0,1,2
                oldVis = isVisible(obj, 'DMRSUplinkTransformPrecodingR16');
                newVis = strcmpi(obj.Modulation_f34,'pi/2-BPSK');
                needRepaint = xor(oldVis, newVis);
                if needRepaint
                    setVisible(obj, 'DMRSUplinkTransformPrecodingR16', newVis);
                end
            end
        end
    end

    % Getters/setters
    methods
        function val = get.CurrentChannelType(obj)
            % Get the type and format of the current selected channel instance
            if any(obj.Format == [0, 1]) % PUCCH format 0 1
                val = 'PUCCH01';
            elseif any(obj.Format == [2, 3]) % PUCCH format 2 3
                val = 'PUCCH23';
            else % PUCCH format 4 - no-op flag
                val = 'PUCCH4';
            end
        end

        function val = get.CurrentChannelIdx(obj)
            % Get the index in cache config for the current selected channel
            % instance
            if strcmpi(obj.CurrentChannelType,'PUCCH4')
                val = []; % empty for PUCCH format 4
            else % PUCCH format 0 1 2 3
                val = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.CurrentDialog.pucchTable.Selection);
            end
        end

        function val = get.CurrentCacheConfig(obj)
            % Get the config cell of the current channel type
            val = obj.CurrentDialog.pucchWaveConfig;
        end

        function val = get.SupportInterlacing(obj)
            % Get the flag for whether the current selected channel supports
            % interlacing
            % For PUCCH: channel supports interlacing if not format 4 and is
            % with 15 or 30 kHz SCS
            val = (~strcmpi(obj.CurrentChannelType,'PUCCH4')) && any(obj.CurrentSCS == [15, 30]);
        end

        function out = get.ChannelName(~)
            out = 'PUCCH';
        end
    end
end

function props = getPropertiesForFormat(format)
    switch format
        case 0
            % Property list of format 0
            props = {'Format', 'FrequencyHopping', 'SecondHopStartPRB', ...
                'Interlacing_f0123', 'InterlaceIndex_f0123', 'RBSetIndex_f0123', ...
                'PUCCHGroupHopping', 'HoppingID', 'InitialCyclicShift', 'Modulation_f0'};
        case 1
            % Property list of format 1
            props = {'Format', 'FrequencyHopping', 'SecondHopStartPRB', ...
                'Interlacing_f0123', 'InterlaceIndex_f0123', 'RBSetIndex_f0123', ...
                'PUCCHGroupHopping', 'HoppingID', 'InitialCyclicShift', 'OCCI_f1', ...
                'DMRSPower', 'Modulation_f1'};
        case 2
            % Property list of format 2
            props = {'Format', 'FrequencyHopping', 'SecondHopStartPRB', ...
                'Interlacing_f0123', 'InterlaceIndex_f0123', ...
                'RBSetIndex_f0123', 'SpreadingFactor_f23', 'OCCI_f23', ...
                'DMRSPower', 'NID0', 'Modulation_f2'};
        case 3
            % Property list of format 3
            props = {'Format', 'Modulation_f34', 'FrequencyHopping', 'SecondHopStartPRB', ...
                'Interlacing_f0123', 'InterlaceIndex_f0123', 'RBSetIndex_f0123', ...
                'SpreadingFactor_f23', 'OCCI_f23', 'DMRSPower', 'AdditionalDMRS', ...
                'DMRSUplinkTransformPrecodingR16', 'PUCCHGroupHopping', 'HoppingID', 'NID0'};
        otherwise % format 4
            % Property list of format 4
            props = {'Format', 'Modulation_f34', 'FrequencyHopping', 'SecondHopStartPRB', ...
                'DMRSPower', 'AdditionalDMRS', 'DMRSUplinkTransformPrecodingR16', ...
                'PUCCHGroupHopping', 'HoppingID', 'SpreadingFactor', 'OCCI_f4', 'NID0'};
    end
    props{end+1} = 'Label';
end