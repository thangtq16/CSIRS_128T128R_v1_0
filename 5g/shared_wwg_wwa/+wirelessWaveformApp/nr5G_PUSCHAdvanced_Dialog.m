classdef nr5G_PUSCHAdvanced_Dialog < wirelessWaveformApp.nr5G_PXSCHAdvanced_Dialog & ...
                                     wirelessWaveformApp.internal.interlaceBase
    % Interface to properties that are unique for PUSCH and are not present in the basic table

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Constant, Access = protected)
        DependentDialogs = {'wirelessWaveformApp.nr5G_PUSCHDMRS_Dialog' ...
                            'wirelessWaveformApp.nr5G_PUSCHPTRS_Dialog'} % List of dependent dialog classes
    end

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
    end

    properties (Constant, Hidden, Access = public)
        DefaultCfg = nrWavegenPUSCHConfig;
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Hidden)
        TitleString = 'PUSCH 1 (Advanced Configuration)'

        TransmissionSchemeType = 'charPopup'
        TransmissionSchemeDropDown = {'nonCodebook', 'codebook'}
        TransmissionSchemeLabel
        TransmissionSchemeGUI

        NumAntennaPortsType = 'numericPopup'
        NumAntennaPortsDropDown = {'1', '2', '4'}
        NumAntennaPortsLabel
        NumAntennaPortsGUI

        TPMIType = 'numericEdit'
        TPMILabel
        TPMIGUI

        FrequencyHoppingType = 'charPopup'
        FrequencyHoppingDropDown = {'neither', 'intraSlot', 'interSlot'}
        FrequencyHoppingLabel
        FrequencyHoppingGUI

        SecondHopStartPRBType = 'numericEdit'
        SecondHopStartPRBLabel
        SecondHopStartPRBGUI

        NRAPIDType = 'numericEdit'
        NRAPIDLabel
        NRAPIDGUI

        InterlacingType = 'checkbox'
        InterlacingLabel
        InterlacingGUI

        InterlaceIndexType = 'numericEdit'
        InterlaceIndexLabel
        InterlaceIndexGUI

        RBSetIndexType = 'numericEdit'
        RBSetIndexLabel
        RBSetIndexGUI
    end

    properties (Dependent = true, Access = protected)
        % abstract dependent properties in interlaceBase
        CurrentChannelType;
        CurrentChannelIdx;
        CurrentCacheConfig;
        SupportInterlacing;
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_PUSCHAdvanced_Dialog(parent, fig, invisibleProps)
            obj@wirelessWaveformApp.nr5G_PXSCHAdvanced_Dialog(parent, fig, invisibleProps, wirelessWaveformApp.nr5G_PUSCHAdvanced_Dialog.DefaultCfg); % call base constructor

            obj.MCSTableGUI.Items(3) = []; % remove the option not applicable for UL: qam1024

            % Callbacks needed either for validation of edit fields or for
            % changes to visibility of other controls
            obj.TransmissionSchemeGUI.(obj.Callback)  =  @(src,evnt) txSchemeChangedGUI(obj);
            obj.FrequencyHoppingGUI.(obj.Callback)    =  @(src,evnt) freqHopChangedGUI(obj);
            obj.InterlacingGUI.(obj.Callback)         =  @(src,evnt) interlacingChangedGUI(obj);
        end

        function needRepaint = updateControlsVisibility(obj)
            needRepaint = updateControlsVisibility@wirelessWaveformApp.nr5G_PXSCHAdvanced_Dialog(obj);
            needRepaint = txSchemeChanged(obj) || needRepaint;
            needRepaint = freqHopChanged(obj) || needRepaint;
            needRepaint = updateInterlacingVis(obj) || needRepaint;
        end

        function adjustDialog(obj)
            % TPMI, NumAntennaPorts, SecondHopStart do not appear by default:
            setVisible(obj, {'TPMI', 'NumAntennaPorts', 'SecondHopStartPRB', 'InterlaceIndex', 'RBSetIndex'}, false);

            % For test symmetry with PDSCH:
            obj.DataSourceGUI.Tag = 'PUSCHDataSource';
        end

        function props = displayOrder(~)
            props = {'Label'; 'TransmissionScheme'; ...
                'NumAntennaPorts'; 'TPMI'; ...
                'Interlacing'; 'InterlaceIndex';'RBSetIndex'; ...
                'FrequencyHopping'; 'SecondHopStartPRB'; ...
                'NRAPID'; 'XOverhead'; 'EnableLBRM'; ...
                'MaxNumLayers'; 'MCSTable'; 'RVSequence'; ...
                'RVSequenceCW2'; 'DataSource'; 'CustomDataSource'};
        end

        function restoreDefaults(obj)
            % This method is called in the baseclass constructor to set the
            % default values of the properties
            % Get the same defaults with programmatic objects
            c = nrWavegenPUSCHConfig;
            obj.Label               = c.Label;
            obj.RVSequence          = c.RVSequence;
            obj.RVSequenceCW2       = c.RVSequence;
            obj.TransmissionScheme  = c.TransmissionScheme;
            obj.NumAntennaPorts     = c.NumAntennaPorts;
            obj.TPMI                = c.TPMI;
            obj.FrequencyHopping    = c.FrequencyHopping;
            obj.SecondHopStartPRB   = c.SecondHopStartPRB;
            obj.NRAPID              = c.NRAPID;
            obj.Interlacing         = c.Interlacing;
            obj.InterlaceIndex      = c.InterlaceIndex;
            obj.RBSetIndex          = c.RBSetIndex;
            obj.XOverhead           = c.XOverhead;
            obj.DataSource          = c.DataSource;
            obj.CustomDataSource    = [1; 0; 0; 1];
            obj.EnableLBRM          = c.LimitedBufferRateMatching;
            obj.MaxNumLayers        = c.MaxNumLayers;
            obj.MCSTable            = c.MCSTable;

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

    % Validators and visibility updates
    methods (Access = private)
        function needRepaint = txSchemeChanged(obj, ~)
            oldVis = isVisible(obj, 'TPMI');
            newVis = strcmpi(obj.TransmissionScheme, 'codebook');
            needRepaint = xor(oldVis, newVis);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                setVisible(obj, {'TPMI', 'NumAntennaPorts'}, newVis);
            end
        end
        function txSchemeChangedGUI(obj, ~)
            needRepaint = txSchemeChanged(obj);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                layoutUIControls(obj);
                updateCache(obj);
            end
        end

        function needRepaint = freqHopChanged(obj, ~)
            oldVis = isVisible(obj, 'SecondHopStartPRB');
            newVis = ~strcmpi(obj.FrequencyHopping, 'neither');
            needRepaint = xor(oldVis, newVis);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                setVisible(obj, 'SecondHopStartPRB', newVis);
            end
        end
        function freqHopChangedGUI(obj, ~)
            needRepaint = freqHopChanged(obj);
            updateCacheAndGrids(obj);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive
                layoutUIControls(obj);
            end
        end

        function needRepaint = interlacingChanged(obj, ~)

            % Update cache
            updateCache(obj);

            % Update relevant visibility
            propList = {'InterlaceIndex' 'RBSetIndex' 'FrequencyHopping' 'SecondHopStartPRB'};
            needRepaint = updateInterlacingVis(obj, propList);
        end
        function interlacingChangedGUI(obj, ~)
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
    end

    % Getters/setters
    methods
        function val = get.CurrentChannelType(~)
            % Get the type and format of the current selected channel instance
            val = 'PUSCH';
        end

        function val = get.CurrentChannelIdx(obj)
            % Get the index in cache config for the current selected channel
            % instance
            val = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.CurrentDialog.pxschTable.Selection);
        end

        function val = get.CurrentCacheConfig(obj)
            % Get the config cell of the current channel type
            val = obj.CurrentDialog.pxschWaveConfig;
        end

        function val = get.SupportInterlacing(obj)
            % Get the flag for whether the current selected channel supports
            % interlacing
            % For PUSCH: channel supports interlacing with 15 or 30 kHz SCS
            val = any(obj.CurrentSCS == [15, 30]);
        end

        function out = get.ChannelName(~)
            out = 'PUSCH';
        end
    end
end
