classdef nr5G_PDSCHAdvanced_Dialog < wirelessWaveformApp.nr5G_PXSCHAdvanced_Dialog
    % Interface to properties that are unique for PDSCH and are not present in the basic table

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Constant, Access = protected)
        DependentDialogs = {'wirelessWaveformApp.nr5G_PDSCHDMRS_Dialog' ...
                            'wirelessWaveformApp.nr5G_PDSCHPTRS_Dialog'} % List of dependent dialog classes
    end

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
    end

    properties (Constant, Hidden, Access = public)
        DefaultCfg = nrWavegenPDSCHConfig;
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Hidden)
        TitleString = 'PDSCH 1 (Advanced Configuration)'

        VRBToPRBInterleavingType = 'checkbox'
        VRBToPRBInterleavingLabel
        VRBToPRBInterleavingGUI

        VRBBundleSizeType = 'numericPopup'
        VRBBundleSizeDropDown = {'2', '4'}
        VRBBundleSizeLabel
        VRBBundleSizeGUI

        ReservedCORESETType = 'numericEdit'
        ReservedCORESETLabel
        ReservedCORESETGUI

        ReservedPRBType = 'numericEdit'
        ReservedPRBLabel
        ReservedPRBGUI

        ReservedSymbolsType = 'numericEdit'
        ReservedSymbolsLabel
        ReservedSymbolsGUI

        ReservedPeriodType = 'numericEdit'
        ReservedPeriodLabel
        ReservedPeriodGUI

        TBScalingType = 'numericPopup'
        TBScalingDropDown = {'0.25', '0.5', '1'}
        TBScalingLabel
        TBScalingGUI
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_PDSCHAdvanced_Dialog(parent, fig, invisibleProps)
            obj@wirelessWaveformApp.nr5G_PXSCHAdvanced_Dialog(parent, fig, invisibleProps, wirelessWaveformApp.nr5G_PDSCHAdvanced_Dialog.DefaultCfg); % call base constructor

            % Callbacks needed either for validation of edit fields or for
            % changes to visibility of other controls
            obj.VRBToPRBInterleavingGUI.(obj.Callback)  =  @(src,evnt) vrbChangedGUI(obj);
            obj.ReservedPRBGUI.(obj.Callback)           =  @(src,evnt) updateProperty(obj,src,evnt,"PRBSet",Fieldnames="ReservedPRB",Config=obj.Config.ReservedPRB{1});
            obj.ReservedSymbolsGUI.(obj.Callback)       =  @(src,evnt) updateProperty(obj,src,evnt,"SymbolSet",FieldNames="ReservedSymbols",Config=obj.Config.ReservedPRB{1});
            obj.ReservedPeriodGUI.(obj.Callback)        =  @(src,evnt) updateProperty(obj,src,evnt,"Period",FieldNames="ReservedPeriod",Config=obj.Config.ReservedPRB{1});

        end

        function needRepaint = updateControlsVisibility(obj)
            needRepaint = updateControlsVisibility@wirelessWaveformApp.nr5G_PXSCHAdvanced_Dialog(obj);
            needRepaint = vrbChanged(obj) || needRepaint;
        end

        function adjustDialog(obj)
            % Make sure all tags are unique. Otherwise there is a conflict with SSB
            obj.DataSourceGUI.Tag = 'PDSCHDataSource';
        end

        function props = displayOrder(~)
            props = {'Label'; 'VRBToPRBInterleaving'; 'VRBBundleSize'; 'ReservedCORESET'; ...
                     'ReservedPRB'; 'ReservedSymbols'; 'ReservedPeriod'; 'TBScaling'; ...
                     'XOverhead';  'EnableLBRM'; 'MaxNumLayers'; 'MCSTable'; ...
					 'RVSequence'; 'RVSequenceCW2'; 'DataSource'; 'CustomDataSource'};
        end

        function restoreDefaults(obj)
            % This method is called in the baseclass constructor to set the
            % default values of the properties
            % Get the same defaults with programmatic objects
            c = nrWavegenPDSCHConfig;
            obj.Label                 = c.Label;
            obj.RVSequence            = c.RVSequence;
            obj.RVSequenceCW2         = c.RVSequence; % nrWavegenPDSCHConfig has a single RV sequence by default, so duplicate it for the 2nd codeword
            obj.VRBToPRBInterleaving  = c.VRBToPRBInterleaving;
            obj.VRBBundleSize         = c.VRBBundleSize;
            obj.ReservedCORESET       = c.ReservedCORESET;
            obj.ReservedPRB           = c.ReservedPRB{1}.PRBSet;
            obj.ReservedSymbols       = c.ReservedPRB{1}.SymbolSet;
            obj.ReservedPeriod        = c.ReservedPRB{1}.Period;
            obj.TBScaling             = c.TBScaling;
            obj.XOverhead             = c.XOverhead;
            obj.DataSource            = c.DataSource;
            obj.CustomDataSource      = [1; 0; 0; 1];
            obj.EnableLBRM            = c.LimitedBufferRateMatching;
            obj.MaxNumLayers          = c.MaxNumLayers;
            obj.MCSTable              = c.MCSTable;

            % Set default visibility
            updateControlsVisibility(obj);
        end

        %% Stack Advanced/DMRS/PTRS vertically:
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

    % Visibility updates
    methods (Access = private)
        function needRepaint = vrbChanged(obj, ~)
            % Update visibility of VRB Bundle Size when VRB-to-PRB mapping changes
            % VRB Bundle Size is not visible when VRB interleaving is off
            oldVis = isVisible(obj, 'VRBBundleSize');
            newVis = obj.VRBToPRBInterleaving;
            needRepaint = xor(oldVis, newVis);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                setVisible(obj, 'VRBBundleSize', newVis);
            end
        end

        function vrbChangedGUI(obj, ~)
            % The user clicked on the VRB-to-PRB interleaving checkbox:
            % Update visibility of VRB Bundle Size and update the grid plots
            needRepaint = vrbChanged(obj);

            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                layoutUIControls(obj);
                updateCacheAndGrids(obj);
            end
        end
    end

    % Custom getters/setters
    methods
        function out = get.ChannelName(~)
            out = 'PDSCH';
        end
    end
end
