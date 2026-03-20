classdef nr5G_PXSCHAdvanced_Dialog < wirelessWaveformApp.nr5G_Dialog
    % Interface to common PXSCH properties that are not present in the basic table

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Abstract, Constant, Access = protected)
        DependentDialogs % List of dependent dialog classes
    end

    properties (Access = protected)
        EnableDependentDialog % Visibility flag of dependent dialog class (true -> enable)
    end

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
        CustomDataSource
    end

    properties (Hidden)
        LabelType = 'charEdit'
        LabelLabel
        LabelGUI

        XOverheadType = 'numericPopup'
        XOverheadDropDown = {'0', '6', '12', '18'}
        XOverheadLabel
        XOverheadGUI

        RVSequenceType = 'numericEdit'
        RVSequenceLabel
        RVSequenceGUI

        RVSequenceCW2Type = 'numericEdit'
        RVSequenceCW2Label
        RVSequenceCW2GUI

        DataSourceType = 'charPopup'
        DataSourceDropDown = {'PN9-ITU','PN9','PN11', 'PN15', 'PN23', 'User-defined'}
        DataSourceLabel
        DataSourceGUI

        CustomDataSourceType = 'numericEdit'
        CustomDataSourceLabel
        CustomDataSourceGUI

        EnableLBRMType = 'checkbox'
        EnableLBRMLabel
        EnableLBRMGUI

        MaxNumLayersType = 'numericPopup'
        MaxNumLayersDropDown = cellstr(string(1:8))
        MaxNumLayersLabel
        MaxNumLayersGUI

        MCSTableType = 'charPopup'
        MCSTableDropDown = {'qam64','qam256','qam1024'}
        MCSTableLabel
        MCSTableGUI

    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_PXSCHAdvanced_Dialog(parent, fig, invisibleProps, cfg)
            obj@wirelessWaveformApp.nr5G_Dialog(parent, fig, cfg); % call base constructor

            % Callbacks needed either for validation of edit fields or for
            % changes to visibility of other controls
            obj.LabelGUI.(obj.Callback)            =  @(src,evnt) updateCache(obj);
            obj.RVSequenceGUI.(obj.Callback)       =  @(src,evnt) rvSeqChangedGUI(obj,src,evnt);
            obj.RVSequenceCW2GUI.(obj.Callback)    =  @(src,evnt) rvSeqChangedGUI(obj,src,evnt);
            obj.DataSourceGUI.(obj.Callback)       =  @(src,evnt) sourceChangedGUI(obj);
            obj.CustomDataSourceGUI.(obj.Callback) =  @(src,evnt) customDataSourceChangedGUI(obj,src,evnt);
            obj.EnableLBRMGUI.(obj.Callback)       =  @(src,evnt) enableLBRMChangedGUI(obj);
            if nargin > 2
                % Hide the properties that are not applicable at the time of
                % construction (this private property is used in displayOrder)
                invisibleProps = cellstr(invisibleProps);
                propsDlg = displayOrder(obj);  % Properties that are visible in this dialog
                propsInvisible = ismember(propsDlg, invisibleProps);
                setVisible(obj, propsDlg(propsInvisible), false);

                if ~isempty(obj.DependentDialogs)
                    % Check the dependent dialog classes and hide them as applicable
                    numDependentClasses = numel(obj.DependentDialogs);
                    enableDependentClasses = true(1,numDependentClasses);
                    for i = 1:numDependentClasses
                        dlg = obj.Parent.AppObj.pParameters.DialogsMap(obj.DependentDialogs{i});
                        propsDlg = displayOrder(dlg);  % Properties that are visible in this dialog
                        propsInvisible = ismember(propsDlg,invisibleProps);
                        if all(propsInvisible)
                            % Disable that particular entry
                            enableDependentClasses(i) = false;
                        else
                            % Disable the specific properties in this
                            % dialog
                            setVisible(dlg, propsDlg(propsInvisible), false);
                        end
                    end
                    obj.EnableDependentDialog = enableDependentClasses;
                end
            end
        end

        function needRepaint = updateControlsVisibility(obj)
            needRepaint = sourceChanged(obj);
            needRepaint = enableLBRMChanged(obj) || needRepaint;
            needRepaint = updateRVSequenceCW2Visibility(obj) || needRepaint;
        end

        function needRepaint = enableLBRMChanged(obj)

            % Get Coding
            currPXSCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.CurrentDialog.pxschTable.Selection);
            coding = obj.CurrentDialog.pxschWaveConfig{currPXSCH}.Coding;

            oldVis = isVisible(obj, 'MaxNumLayers');
            newVis = coding && obj.EnableLBRM;
            needRepaint = xor(newVis,oldVis);
            if needRepaint
                setVisible(obj, {'MaxNumLayers','MCSTable'}, newVis);
            end

        end

        function needRepaint = updateRVSequenceCW2Visibility(obj)
            currPXSCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.CurrentDialog.pxschTable.Selection);

            % Get number of layers and coding
            twoCW = obj.CurrentDialog.pxschWaveConfig{currPXSCH}.NumCodewords == 2;
            coding = obj.CurrentDialog.pxschWaveConfig{currPXSCH}.Coding;

            oldVis = isVisible(obj, 'RVSequenceCW2');
            newVis = twoCW && coding;

            needRepaint = xor(oldVis, newVis);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                setVisible(obj, {'RVSequenceCW2'}, newVis);
            end
        end
    end

    % Validators and visibility updates
    methods (Access = private)
        function sourceChangedGUI(obj, ~)
            % Custom data source edit field appears only under user-defined data source
            needRepaint = sourceChanged(obj);
            updateCacheAndGrids(obj);

            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                layoutUIControls(obj);
            end
        end

        function needRepaint = sourceChanged(obj, ~)
            % Custom data source edit field appears only under user-defined data source
            oldVis = isVisible(obj, 'CustomDataSource');
            newVis = strcmpi(obj.DataSource, 'User-defined');
            needRepaint = xor(oldVis, newVis);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                setVisible(obj, 'CustomDataSource', newVis);
            end
        end

        function rvSeqChangedGUI(obj, src, evnt)
            updateProperty(obj, src, evnt, "RVSequence", FieldNames=["RVSequence" "RVSequenceCW2"], PropType="cell");
        end

        function customDataSourceChangedGUI(obj, src, evnt)
            updateProperty(obj, src, evnt, "DataSource", FieldNames="CustomDataSource");
        end

        function enableLBRMChangedGUI(obj,~)

            needRepaint = enableLBRMChanged(obj);

            if needRepaint
                updateCache(obj);
                layoutUIControls(obj);
            end

        end
    end

    % Getters/setters
    methods
        function n = get.CustomDataSource(obj)
            n = getEditVal(obj, 'CustomDataSource');
        end
        function set.CustomDataSource(obj, val)
            if iscell(val)
                obj.CustomDataSourceGUI.(obj.EditValue) = sprintf('{''%s'',%d}',val{1},val{2});
            else
                setEditVal(obj, 'CustomDataSource', val);
            end
        end
    end
end
