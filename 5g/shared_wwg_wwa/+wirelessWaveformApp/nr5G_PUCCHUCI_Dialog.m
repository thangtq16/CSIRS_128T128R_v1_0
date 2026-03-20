classdef nr5G_PUCCHUCI_Dialog < wirelessWaveformApp.nr5G_Dialog
    % Interface to all UCI properties

    %   Copyright 2021-2024 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
        CustomDataSourceUCI_f012 % Formats 0, 1, 2
        CustomDataSourceUCI_f34 % Formats 3, 4
        CustomDataSourceSR
        CustomDataSourceUCI2
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Hidden)
        TitleString = 'PUCCH 1 UCI'
        Format = 0;

        NumUCIBits_f01Type = 'numericPopup'
        NumUCIBits_f01DropDown = {'0', '1', '2'}
        NumUCIBits_f01Label
        NumUCIBits_f01GUI

        NumUCIBits_f2Type = 'numericEdit'
        NumUCIBits_f2Label
        NumUCIBits_f2GUI

        NumUCIBits_f34Type = 'numericEdit'
        NumUCIBits_f34Label
        NumUCIBits_f34GUI

        DataSourceUCI_f012Type = 'charPopup'
        DataSourceUCI_f012DropDown = {'PN9-ITU','PN9','PN11', 'PN15', 'PN23', 'User-defined'}
        DataSourceUCI_f012Label
        DataSourceUCI_f012GUI

        CustomDataSourceUCI_f012Type = 'numericEdit'
        CustomDataSourceUCI_f012Label
        CustomDataSourceUCI_f012GUI

        DataSourceUCI_f34Type = 'charPopup'
        DataSourceUCI_f34DropDown = {'PN9-ITU','PN9','PN11', 'PN15', 'PN23', 'User-defined'}
        DataSourceUCI_f34Label
        DataSourceUCI_f34GUI

        CustomDataSourceUCI_f34Type = 'numericEdit'
        CustomDataSourceUCI_f34Label
        CustomDataSourceUCI_f34GUI

        NumUCI2BitsType = 'numericEdit'
        NumUCI2BitsLabel
        NumUCI2BitsGUI

        DataSourceUCI2Type = 'charPopup'
        DataSourceUCI2DropDown = {'PN9-ITU','PN9','PN11', 'PN15', 'PN23', 'User-defined'}
        DataSourceUCI2Label
        DataSourceUCI2GUI

        CustomDataSourceUCI2Type = 'numericEdit'
        CustomDataSourceUCI2Label
        CustomDataSourceUCI2GUI

        DataSourceSRType = 'charPopup'
        DataSourceSRDropDown = {'PN9-ITU','PN9','PN11', 'PN15', 'PN23', 'User-defined'}
        DataSourceSRLabel
        DataSourceSRGUI

        CustomDataSourceSRType = 'numericEdit'
        CustomDataSourceSRLabel
        CustomDataSourceSRGUI

        TargetCodeRateType = 'numericEdit'
        TargetCodeRateLabel
        TargetCodeRateGUI
    end

    properties (Dependent = true, Access = private)
        Coding % Coding of this PUCCH instance
    end

    properties (Access = private)
        InvisibleProperties = {};
        DefaultCfg = configureDictionary("double","cell"); % Stores the default configurations for each PUCCH format
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_PUCCHUCI_Dialog(parent, fig, invisibleProps)
            obj@wirelessWaveformApp.nr5G_Dialog(parent, fig, wirelessWaveformApp.nr5G_PUCCH_Tab.getDefaultConfigObject(0)); % call base constructor
            for format = 0:4
                obj.DefaultCfg(format) = {wirelessWaveformApp.nr5G_PUCCH_Tab.getDefaultConfigObject(format)};
            end

            % Callbacks needed either for validation of edit fields or for
            % changes to visibility of other controls
            obj.NumUCIBits_f01GUI.(obj.Callback)           = @(src,evnt) numUCIBits_f01ChangedGUI(obj,src,evnt);
            obj.NumUCIBits_f2GUI.(obj.Callback)            = @(src,evnt) numUCIBits_f2ChangedGUI(obj,src,evnt);
            obj.NumUCIBits_f34GUI.(obj.Callback)           = @(src,evnt) numUCIBits_f34ChangedGUI(obj,src,evnt);
            obj.DataSourceUCI_f012GUI.(obj.Callback)       = @(src,evnt) sourceUCI_f012ChangedGUI(obj);
            obj.CustomDataSourceUCI_f012GUI.(obj.Callback) = @(src,evnt) updatePUCCHUCIProperty(obj,src,evnt,"DataSourceUCI",FieldNames="CustomDataSourceUCI_f012");
            obj.DataSourceUCI_f34GUI.(obj.Callback)        = @(src,evnt) sourceUCI_f34ChangedGUI(obj);
            obj.CustomDataSourceUCI_f34GUI.(obj.Callback)  = @(src,evnt) updatePUCCHUCIProperty(obj,src,evnt,"DataSourceUCI",FieldNames="CustomDataSourceUCI_f34");
            obj.NumUCI2BitsGUI.(obj.Callback)              = @(src,evnt) numUCI2BitsChangedGUI(obj,src,evnt);
            obj.DataSourceUCI2GUI.(obj.Callback)           = @(src,evnt) sourceUCI2ChangedGUI(obj);
            obj.CustomDataSourceUCI2GUI.(obj.Callback)     = @(src,evnt) updatePUCCHUCIProperty(obj,src,evnt,"DataSourceUCI2",FieldNames="CustomDataSourceUCI2");
            obj.DataSourceSRGUI.(obj.Callback)             = @(src,evnt) sourceSRChangedGUI(obj);
            obj.CustomDataSourceSRGUI.(obj.Callback)       = @(src,evnt) updatePUCCHUCIProperty(obj,src,evnt,"DataSourceSR",FieldNames="CustomDataSourceSR");
            obj.TargetCodeRateGUI.(obj.Callback)           = @(src,evnt) updatePUCCHUCIProperty(obj,src,evnt,"TargetCodeRate");

            if nargin > 2
                % Hide the properties that are not applicable at the time of
                % construction
                invisibleProps = cellstr(invisibleProps);
                propsDlg = displayOrder(obj);  % Properties that are visible in this dialog
                propsInvisible = ismember(propsDlg, invisibleProps);
                obj.InvisibleProperties = propsDlg(propsInvisible);
                setVisible(obj, obj.InvisibleProperties, false);
            end

            % Set default visibility
            formatChanged(obj);
            layoutUIControls(obj);
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

            % Adjust visibility for dependent properties
            switch obj.Format
                case {0, 1}
                    needRepaint = updateF01Vis(obj);
                case 2
                    needRepaint = updateF2Vis(obj);
                otherwise % Format 3 or 4
                    needRepaint = updateF2Vis(obj);
            end

            % Ensure the always-invisible properties are indeed hidden
            setVisible(obj, obj.InvisibleProperties, false);
        end

        function adjustDialog(obj)
            % Make sure all tags are unique. Otherwise there is a conflict
            % with other channels
            obj.TargetCodeRateGUI.Tag = 'PUCCHTargetCodeRate';
            % Decorate the tags for the remaining properties with the
            % channel name
            obj.NumUCIBits_f01GUI.Tag = 'PUCCHNumUCIBits_f01';
            obj.NumUCIBits_f2GUI.Tag = 'PUCCHNumUCIBits_f2';
            obj.NumUCIBits_f34GUI.Tag = 'PUCCHNumUCIBits_f34';
            obj.DataSourceUCI_f012GUI.Tag = 'PUCCHDataSourceUCI_f012';
            obj.DataSourceUCI_f34GUI.Tag = 'PUCCHDataSourceUCI_f34';
            obj.CustomDataSourceUCI_f012GUI.Tag = 'PUCCHCustomDataSourceUCI_f012';
            obj.CustomDataSourceUCI_f34GUI.Tag = 'PUCCHCustomDataSourceUCI_f34';
            obj.DataSourceSRGUI.Tag = 'PUCCHDataSourceSR';
            obj.CustomDataSourceSRGUI.Tag = 'PUCCHCustomDataSourceSR';
            obj.NumUCI2BitsGUI.Tag = 'PUCCHNumUCI2Bits';
            obj.DataSourceUCI2GUI.Tag = 'PUCCHDataSourceUCI2';
            obj.CustomDataSourceUCI2GUI.Tag = 'PUCCHCustomDataSourceUCI2';
        end

        function props = displayOrder(~)
            props = {'NumUCIBits_f01'; 'NumUCIBits_f2'; 'DataSourceUCI_f012'; 'CustomDataSourceUCI_f012'; ...
                     'NumUCIBits_f34'; 'DataSourceUCI_f34'; 'CustomDataSourceUCI_f34'; ...
                     'NumUCI2Bits'; 'DataSourceUCI2'; 'CustomDataSourceUCI2'; ...
                     'DataSourceSR'; 'CustomDataSourceSR'; 'TargetCodeRate'};
        end

        function restoreDefaults(obj)
            % Get the same defaults with programmatic objects
            c0 = wirelessWaveformApp.nr5G_PUCCH_Tab.defaultPUCCH0Config;
            c2 = wirelessWaveformApp.nr5G_PUCCH_Tab.defaultPUCCH2Config;
            c3 = wirelessWaveformApp.nr5G_PUCCH_Tab.defaultPUCCH3Config;
            obj.NumUCIBits_f01           = c0.NumUCIBits;
            obj.NumUCIBits_f2            = c2.NumUCIBits;
            obj.NumUCIBits_f34           = c3.NumUCIBits;
            obj.NumUCI2Bits              = c3.NumUCI2Bits;
            obj.DataSourceUCI_f012       = c0.DataSourceUCI;
            obj.CustomDataSourceUCI_f012 = ones(obj.NumUCIBits_f01, 1);
            obj.DataSourceSR             = 'User-defined';
            obj.CustomDataSourceSR       = c0.DataSourceSR;
            obj.CustomDataSourceUCI_f34  = ones(obj.NumUCIBits_f34, 1);
            obj.CustomDataSourceUCI2     = 1;
            obj.TargetCodeRate           = 0.15;

            % Set default visibility
            formatChanged(obj);
        end

        function str = getCatalogPrefix(~)
            str = 'nr5g:waveformApp:';
        end

        function cellDialogs = getDialogsPerColumn(obj)
            cellDialogs{1} = {obj};
        end
    end

    % Dependent visibility updates (also used by neighboring classes)
    methods (Access = public)
        function needRepaint = updateNumUCIBits_f34Vis(obj, coding)

            if nargin<2
                coding = obj.Coding;
            end

            % NumUCIBits_f34
            newVis1 = any(obj.Format == [3, 4]) && coding;
            tagList = {'NumUCIBits_f34'};
            visList = {newVis1};

            needRepaint = updateVisibility(obj,tagList,visList);

            % DataSourceUCI_f34
            needRepaint = updateSourceUCI_f34Vis(obj,coding) || needRepaint;

            % NumUCI2Bits
            needRepaint = updateNumUCI2BitsVis(obj,coding) || needRepaint;

        end
    end

    methods (Access = private)
        function updateAppConfigDiagnostic_private(obj, e)
            % Update the banner to display any warning related to the current
            % configuration or to clear any previous warning that no longer apply
            dlg = obj.CurrentDialog;

            % Only update the banner if there is an error or if the cached
            % PUCCH configuration has the right format. If the latter is
            % false, it means that the app has still work to do after this
            % call, so it is okay not to update the banner for now.
            currPUCCH = wirelessWaveformApp.internal.Utility.getSingleSelection(dlg.pucchTable.Selection);
            cachedFormat = double(extract(string(class(dlg.pucchWaveConfig{currPUCCH})),digitsPattern));
            if ~isempty(e) || isequal(obj.Format, cachedFormat)
                updateAppConfigDiagnostic(dlg, e);
            end
        end
    end

    % Validators and visibility updates
    methods (Access = private)
        function formatChanged(obj)
            props = getPropertiesForFormat(obj.Format);
            c = wirelessWaveformApp.nr5G_PUCCH_Tab.getDefaultConfigObject(obj.Format);

            % Assign the default values
            switch obj.Format
                case 0
                    obj.NumUCIBits_f01           = c.NumUCIBits;
                    obj.DataSourceUCI_f012       = c.DataSourceUCI;
                    obj.CustomDataSourceUCI_f012 = 1;
                    obj.DataSourceSR             = 'User-defined';
                    obj.CustomDataSourceSR       = c.DataSourceSR;

                case 1
                    obj.NumUCIBits_f01           = c.NumUCIBits;
                    obj.DataSourceUCI_f012       = c.DataSourceUCI;
                    obj.CustomDataSourceUCI_f012 = 1;

                case 2
                    obj.NumUCIBits_f2            = c.NumUCIBits;
                    obj.DataSourceUCI_f012       = c.DataSourceUCI;
                    obj.CustomDataSourceUCI_f012 = 1;

                otherwise % format 3 or 4
                    fnames = fieldnames(c);
                    fnames = fnames(ismember(fnames,props));
                    for n = 1:length(fnames)
                        obj.(fnames{n}) = c.(fnames{n});
                    end
            end

            % Update visibility of dependent properties
            updateControlsVisibility(obj);
        end

        function ME = updatePUCCHUCIProperty(obj,src,evnt,propName,nvargs)
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

        function needRepaint = updateF01Vis(obj)

            % DataSourceUCI_f012
            newVis1 = any(obj.Format == [0, 1]) && obj.NumUCIBits_f01>0;
            tagList1 = {'DataSourceUCI_f012'};

            % CustomDataSourceUCI_f012
            newVis2 = newVis1 && strcmpi(obj.DataSourceUCI_f012, 'User-defined');
            tagList2 = {'CustomDataSourceUCI_f012'};

            % DataSourceSR
            newVis3 = obj.Format == 0 && obj.NumUCIBits_f01>0;
            tagList3 = {'DataSourceSR'};

            % CustomDataSourceSR
            newVis4 = newVis3 && strcmpi(obj.DataSourceSR, 'User-defined');
            tagList4 = {'CustomDataSourceSR'};

            % Update visibilities
            tagList = [tagList1 tagList2 tagList3 tagList4];
            visList = {newVis1 newVis2 newVis3 newVis4};
            needRepaint = updateVisibility(obj,tagList,visList);

            % Update the modulation scheme, if needed
            updateModulationVis(obj);
        end

        function needRepaint = updateF2Vis(obj)

            % DataSourceUCI_f012
            newVis1 = (obj.Format == 2) && obj.NumUCIBits_f2>0;
            tagList1 = {'DataSourceUCI_f012'};

            % CustomDataSourceUCI_f012
            newVis2 = newVis1 && strcmpi(obj.DataSourceUCI_f012, 'User-defined');
            tagList2 = {'CustomDataSourceUCI_f012'};

            % Update visibilities
            tagList = [tagList1 tagList2];
            visList = {newVis1 newVis2};
            needRepaint = updateVisibility(obj,tagList,visList);
        end

        function needRepaint = updateF34Vis(obj)
            needRepaint = updateSourceUCI_f34Vis(obj);
            needRepaint = updateNumUCI2BitsVis(obj) || needRepaint;
        end

        function needRepaint = updateSourceUCI_f34Vis(obj, coding)

            if nargin<2
                coding = obj.Coding;
            end

            % DataSourceUCI_f34
            newVis1 = any(obj.Format == [3, 4]) && (~coding || (coding && obj.NumUCIBits_f34>0));
            tagList = {'DataSourceUCI_f34'};
            visList = {newVis1};

            % CustomDataSourceUCI_f34
            newVis2 = newVis1 && strcmpi(obj.DataSourceUCI_f34,'User-defined');
            tagList(end+1) = {'CustomDataSourceUCI_f34'};
            visList(end+1) = {newVis2};

            needRepaint = updateVisibility(obj,tagList,visList);

        end

        function needRepaint = updateNumUCI2BitsVis(obj, coding)

            if nargin<2
                coding = obj.Coding;
            end

            % NumUCI2Bits
            newVis1 = any(obj.Format == [3, 4]) && coding && obj.NumUCIBits_f34>0;
            tagList = {'NumUCI2Bits'};
            visList = {newVis1};

            needRepaint = updateVisibility(obj,tagList,visList);

            % DataSourceUCI2
            needRepaint = updateSourceUCI2Vis(obj, coding) || needRepaint;

        end

        function needRepaint = updateSourceUCI2Vis(obj, coding)

            if nargin<2
                coding = obj.Coding;
            end

            % DataSourceUCI2 and TargetCodeRate
            newVis1 = any(obj.Format == [3, 4]) && coding && obj.NumUCIBits_f34>0 && obj.NumUCI2Bits>0;
            tagList = {'DataSourceUCI2' 'TargetCodeRate'};
            visList = {newVis1 newVis1};

            % CustomDataSourceUCI2
            newVis2 = newVis1 && strcmpi(obj.DataSourceUCI2,'User-defined');
            tagList(end+1) = {'CustomDataSourceUCI2'};
            visList(end+1) = {newVis2};

            needRepaint = updateVisibility(obj,tagList,visList);

        end

        function sourceUCI_f34ChangedGUI(obj, ~)
            % Custom data source edit field appears only under user-defined data
            % source for UCI
            updateCache(obj);
            needRepaint = updateSourceUCI_f34Vis(obj);

            if needRepaint
                layoutUIControls(obj);
            end
        end

        function sourceUCI2ChangedGUI(obj, ~)
            % Custom data source edit field appears only under user-defined data
            % source for UCI part 2
            updateCache(obj);
            needRepaint = updateSourceUCI2Vis(obj);

            if needRepaint
                layoutUIControls(obj);
            end
        end

        function numUCI2BitsChangedGUI(obj, src, evnt)
            ME = updatePUCCHUCIProperty(obj, src, evnt, "NumUCI2Bits");

            % If there is no error, do the cross-parameter validation
            if isempty(ME)
                cfg = obj.DefaultCfg{obj.Format};
                cfg.NumUCIBits = obj.NumUCIBits_f34;
                cfg.NumUCI2Bits = obj.NumUCI2Bits;
                e = [];
                try
                    validateConfig(cfg);

                    % Update visibility
                    needRepaint = updateNumUCI2BitsVis(obj);

                    % do layout only if necessary
                    if needRepaint
                        layoutUIControls(obj);
                    end
                catch e
                end

                % Update error message
                updateAppConfigDiagnostic_private(obj,e);
            end
        end

        function numUCIBits_f01ChangedGUI(obj, src, evnt)
            % First, check that the standalone value is correct
            ME = updatePUCCHUCIProperty(obj, src, evnt, "NumUCIBits", FieldNames="NumUCIBits_f01");

            % If there is no error, update the visibility of this property
            % and those dependent on this property
            if isempty(ME)
                needRepaint = updateF01Vis(obj);
                if needRepaint
                    % do layout only if necessary
                    layoutUIControls(obj);
                end
            end
        end

        function numUCIBits_f2ChangedGUI(obj, src, evnt)
            % First, check that the standalone value is correct
            ME = updatePUCCHUCIProperty(obj, src, evnt, "NumUCIBits", FieldNames="NumUCIBits_f2");

            % If there is no error, update the visibility of this property
            % and those dependent on this property
            if isempty(ME)
                needRepaint = updateF2Vis(obj);
                if needRepaint
                    % do layout only if necessary
                    layoutUIControls(obj);
                end
            end
        end

        function numUCIBits_f34ChangedGUI(obj, src, evnt)
            % First, check that the standalone value is correct
            ME = updatePUCCHUCIProperty(obj, src, evnt, "NumUCIBits", FieldNames="NumUCIBits_f34");

            % If there is no error, do the cross-parameter validation
            if isempty(ME)
                cfg = obj.DefaultCfg{obj.Format};
                cfg.NumUCIBits = obj.NumUCIBits_f34;
                if obj.NumUCIBits_f34 > 0
                    cfg.NumUCI2Bits = obj.NumUCI2Bits;
                end
                e = [];
                try
                    validateConfig(cfg);

                    % Update visibility
                    needRepaint = updateNumUCIBits_f34Vis(obj);

                    % do layout only if necessary
                    if needRepaint
                        layoutUIControls(obj);
                    end
                catch e
                end

                % Update error message
                updateAppConfigDiagnostic_private(obj,e);
            end
        end

        function needRepaint = sourceSRChanged(obj)
            % Custom data source edit field appears only under user-defined data
            % source for SR
            oldVis = isVisible(obj, 'CustomDataSourceSR');
            newVis = strcmpi(obj.DataSourceSR, 'User-defined');

            needRepaint = xor(oldVis, newVis);
            if needRepaint
                setVisible(obj, 'CustomDataSourceSR', newVis);
            end
        end
        function sourceSRChangedGUI(obj, ~)
            % Custom data source edit field appears only under user-defined data
            % source for SR
            updateCache(obj);
            needRepaint = sourceSRChanged(obj);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                layoutUIControls(obj);
            end
        end

        function needRepaint = sourceUCI_f012Changed(obj)
            % Custom data source edit field appears only under user-defined
            % data source for UCI
            oldVis = isVisible(obj, 'CustomDataSourceUCI_f012');
            newVis = strcmpi(obj.DataSourceUCI_f012, 'User-defined');

            needRepaint = xor(oldVis, newVis);
            if needRepaint
                setVisible(obj, 'CustomDataSourceUCI_f012', newVis);
            end
        end
        function sourceUCI_f012ChangedGUI(obj, ~)
            % Custom data source edit field appears only under user-defined data
            % source for UCI
            updateCache(obj);
            needRepaint = sourceUCI_f012Changed(obj);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                layoutUIControls(obj);
            end
        end

        function setCustomDataSources(obj,prop,val)
            % Set the custom data sources
            % This requires special care for cell array input
            if iscell(val)
                obj.([prop 'GUI']).(obj.EditValue) = sprintf('{''%s'',%d}',val{1},val{2});
            else
                setEditVal(obj,prop,val);
            end
        end

        function updateModulationVis(obj)
            % Update the modulation in the advanced panel for format 1
            if obj.Format == 1
                className = 'wirelessWaveformApp.nr5G_PUCCHAdvanced_Dialog';
                dlg = obj.Parent.AppObj.pParameters.DialogsMap(className);
                if obj.NumUCIBits_f01==2
                    dlg.Modulation_f1 = 'QPSK';
                else
                    dlg.Modulation_f1 = 'BPSK';
                end
                modChanged(dlg);
            end
        end

        function needRepaint = updateVisibility(obj,tagList,visList)
            % Update the visibility of label and GUI indicated by tagList
            % according to visibility specified in visList

            needRepaint = false;
            for i = 1:numel(visList)
                tag = tagList{i};
                oldVis = isVisible(obj, tag);
                newVis = visList{i};
                if xor(oldVis,newVis)
                    setVisible(obj, tag, newVis);
                    needRepaint = true;
                end
            end

        end
    end

    % Getters/setters
    methods
        function n = get.CustomDataSourceUCI_f012(obj)
            n = getEditVal(obj, 'CustomDataSourceUCI_f012');
        end
        function set.CustomDataSourceUCI_f012(obj, val)
            setCustomDataSources(obj, 'CustomDataSourceUCI_f012', val);
        end

        function n = get.CustomDataSourceUCI_f34(obj)
            n = getEditVal(obj, 'CustomDataSourceUCI_f34');
        end
        function set.CustomDataSourceUCI_f34(obj, val)
            setCustomDataSources(obj, 'CustomDataSourceUCI_f34', val)
        end

        function n = get.CustomDataSourceUCI2(obj)
            n = getEditVal(obj, 'CustomDataSourceUCI2');
        end
        function set.CustomDataSourceUCI2(obj, val)
            setCustomDataSources(obj, 'CustomDataSourceUCI2', val)
        end

        function n = get.CustomDataSourceSR(obj)
            n = getEditVal(obj, 'CustomDataSourceSR');
        end
        function set.CustomDataSourceSR(obj, val)
            setCustomDataSources(obj, 'CustomDataSourceSR', val)
        end

        function val = get.Coding(obj)
            % Getter for dependent property Coding - coding of this PUCCH
            % instance

            if any(obj.Format == [0, 1])
                % No coding defined for formats 0 and 1
                val = false;
            else
                dlg = obj.CurrentDialog;
                currPUCCH = wirelessWaveformApp.internal.Utility.getSingleSelection(dlg.pucchTable.Selection);
                val = dlg.pucchWaveConfig{currPUCCH}.Coding;
            end

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
            props = {'DataSourceUCI_f012', 'CustomDataSourceUCI_f012', 'NumUCIBits_f01', 'DataSourceSR', 'CustomDataSourceSR'};
        case 1
            % Property list of format 1
            props = {'DataSourceUCI_f012', 'CustomDataSourceUCI_f012', 'NumUCIBits_f01'};
        case 2
            % Property list of format 2
            props = {'DataSourceUCI_f012', 'CustomDataSourceUCI_f012', 'NumUCIBits_f2'};
        otherwise
            % Property list of format 3 and 4
            props = {'NumUCIBits_f34', 'DataSourceUCI_f34', 'CustomDataSourceUCI_f34', ...
                     'NumUCI2Bits', 'DataSourceUCI2', 'CustomDataSourceUCI2', 'TargetCodeRate'};
    end
end