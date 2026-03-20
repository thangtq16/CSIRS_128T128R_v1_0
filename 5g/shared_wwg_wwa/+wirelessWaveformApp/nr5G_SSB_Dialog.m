classdef nr5G_SSB_Dialog < wirelessWaveformApp.nr5G_Dialog
    % Panel offering main properties of SSB tab

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
        BlockPattern
        TransmittedBlocks
        SubcarrierSpacingCommon
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Hidden)
        TitleString = getString(message('nr5g:waveformApp:SSBurstTitle'))

        EnableSSBType = 'checkbox'
        EnableSSBGUI
        EnableSSBLabel

        BlockPatternType = 'charPopup'
        BlockPatternDropDown = {'Case A (15 kHz)', 'Case B (30 kHz)', 'Case C (30 kHz)'}
        BlockPatternGUI
        BlockPatternLabel

        TransmittedBlocksType = 'numericEdit'
        TransmittedBlocksGUI
        TransmittedBlocksLabel

        PeriodType = 'numericPopup'
        PeriodDropDown = {'5', '10', '20', '40', '80', '160'}
        PeriodGUI
        PeriodLabel

        HalfFrameOffsetType = 'numericEdit'
        HalfFrameOffsetGUI
        HalfFrameOffsetLabel

        FrequencyOffsetType = 'charPopup'
        FrequencyOffsetDropDown = {getCarrierCenterFrequencyOffsetString(), getCustomFrequencyOffsetString()}
        FrequencyOffsetGUI
        FrequencyOffsetLabel

        KSSBType = 'numericEdit'
        KSSBGUI
        KSSBLabel

        NCRBSSBType = 'numericEdit'
        NCRBSSBGUI
        NCRBSSBLabel

        PowerType = 'numericEdit'
        PowerGUI
        PowerLabel

        SubcarrierSpacingCommonType = 'charPopup'
        SubcarrierSpacingCommonDropDown = {'15 kHz', '30 kHz'}
        SubcarrierSpacingCommonGUI
        SubcarrierSpacingCommonLabel
    end

    properties (Access = private)
        InvisibleProperties = {}     % Defines the properties that are invisible (at the time of dialog creation)
        DependentDialogs = {}        % List of dependent dialog classes
        EnableDependentDialog = []   % Visibility flag of dependent dialog class (true -> enable)
    end

    properties (Constant, Hidden)
        DefaultCfg = nrWavegenSSBurstConfig;
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_SSB_Dialog(parent, fig, invisibleProps, dependentClasses)
            obj@wirelessWaveformApp.nr5G_Dialog(parent, fig, wirelessWaveformApp.nr5G_SSB_Dialog.DefaultCfg); % call base constructor

            % Callbacks needed either for validation of edit fields or for
            % changes to visibility of other controls
            obj.PowerGUI.(obj.Callback)             = @(src,evnt) powerChangedGUI(obj,src,evnt);
            obj.TransmittedBlocksGUI.(obj.Callback) = @(src,evnt) txBlocksChangedGUI(obj,src,evnt);
            obj.HalfFrameOffsetGUI.(obj.Callback)   = @(src,evnt) halfFrameOffsetChangedGUI(obj,src,evnt);
            obj.FrequencyOffsetGUI.(obj.Callback)   = @(src,evnt) frequencySourceChangedGUI(obj);
            obj.NCRBSSBGUI.(obj.Callback)           = @(src,evnt) numCRBChangedGUI(obj,src,evnt);
            obj.BlockPatternGUI.(obj.Callback)      = @(src,evnt) blockPatternChangedGUI(obj);
            obj.SubcarrierSpacingCommonGUI.(obj.Callback) = @(src,evnt) subcarrierSpacingCommonChangedGUI(obj);
            obj.PeriodGUI.(obj.Callback)            = @(src,evnt) PeriodChangedGUI(obj);
            obj.EnableSSBGUI.(obj.Callback)         = @(src,evnt) EnableChangedGUI(obj);

            % These are turned off by default, we are using an explicit setting to center the SSB
            setVisible(obj, {'NCRBSSB', 'KSSB'}, false);

            if nargin > 2
                % Hide the properties that are not applicable at the time of
                % construction (this private property is used in displayOrder)
                obj.InvisibleProperties = invisibleProps;
            end

            if nargin > 3
                % Set the dependent classes
                obj.DependentDialogs = dependentClasses;
            end

            if ~isempty(obj.DependentDialogs)
                % Check the dependent dialog classes and hide them as applicable
                numDependentClasses = numel(obj.DependentDialogs);
                enableDependentClasses = true(1,numDependentClasses);
                for i = 1:numDependentClasses
                    dlg = obj.Parent.AppObj.pParameters.DialogsMap(obj.DependentDialogs{i});
                    propsDlg = displayOrder(dlg);  % Properties that are visible in this dialog
                    propsInvisible = ismember(propsDlg,obj.InvisibleProperties);
                    if all(propsInvisible)
                        % Disable that particular entry
                        enableDependentClasses(i) = false;
                    end
                end
                obj.EnableDependentDialog = enableDependentClasses;
            end
        end

        function updateControlsVisibility(obj)
            % Toggle SIB1 visibility when required
            className = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
            if isKey(obj.Parent.AppObj.pParameters.DialogsMap, className)
                dlgSSBDS = obj.Parent.AppObj.pParameters.DialogsMap(className);
                setVisible(obj, {'FrequencyOffset'}, ~dlgSSBDS.Sib1Check);
                custom = strcmpi(obj.FrequencyOffset, getCustomFrequencyOffsetString());
                mib     = strcmp(dlgSSBDS.DataSource, 'MIB');
                setVisible(dlgSSBDS, 'Sib1Check', ~custom && obj.EnableSSB && mib);
                layoutUIControls(dlgSSBDS);
                layoutPanels(dlgSSBDS);
            end
            frequencySourceChanged(obj);
            updateSubcarrierSpacingCommonVisibility(obj);
            subcarrierSpacingCommonChanged(obj);
            layoutUIControls(obj);
            layoutPanels(obj);
        end

        function adjustSpec(obj)
            % This is needed so that the SSB panel(s) do not horizontally fill the entire App
            obj.panelFixedSize = true;
        end

        function adjustDialog(obj)
            % Make sure all tags are unique. Otherwise there is a conflict with
            % other channels/signals
            obj.EnableSSBGUI.Tag = 'SSBEnable';
        end

        function props = displayOrder(obj)
            props = {'EnableSSB'; 'Power'; 'BlockPattern'; 'TransmittedBlocks'; 'Period'; ...
                'HalfFrameOffset'; 'FrequencyOffset'; 'NCRBSSB'; 'KSSB'; 'SubcarrierSpacingCommon'};
            props = props(~ismember(props,obj.InvisibleProperties));
        end

        function restoreDefaults(obj)
            c = obj.DefaultCfg;  % Get defaults from nrWavegenSSBurstConfig
            obj.EnableSSB         = c.Enable;
            obj.BlockPattern      = c.BlockPattern;
            obj.TransmittedBlocks = c.TransmittedBlocks;
            obj.Period = c.Period;
            obj.HalfFrameOffset = 0;
            % The interface is slightly different from the programmatic API
            if isempty(c.NCRBSSB)
                obj.FrequencyOffset = getCarrierCenterFrequencyOffsetString();
            else
                obj.FrequencyOffset = getCustomFrequencyOffsetString();
            end
            obj.KSSB              = c.KSSB;
            obj.NCRBSSB           = c.NCRBSSB;
            obj.Power             = c.Power;
            obj.SubcarrierSpacingCommon = c.SubcarrierSpacingCommon;
            updateControlsVisibility(obj);
        end

        %% Stack the 2 SSB panels horizontally
        function cellDialogs = getDialogsPerColumn(obj)
            cellDialogs{1} = {obj};
            for dialogIdx = 1:numel(obj.DependentDialogs)
                if obj.EnableDependentDialog(dialogIdx)
                    if isKey(obj.Parent.AppObj.pParameters.DialogsMap,obj.DependentDialogs{dialogIdx})
                        if dialogIdx==1
                            cellDialogs{2} = {obj.Parent.AppObj.pParameters.DialogsMap(obj.DependentDialogs{dialogIdx})};
                        else
                            cellDialogs{2} = [cellDialogs{2}(:)' {obj.Parent.AppObj.pParameters.DialogsMap(obj.DependentDialogs{dialogIdx})}];
                        end
                    end
                end
            end
        end
    end

    % Methods specific to this class and that are currently used by
    % neighboring classes
    methods (Access = public)
        function ssb = getSSBConfig(obj)
            % Map GUI elements of SS Burst tab to an equivalent nrWavegenSSBurstConfig object:
            ssb = obj.DefaultCfg;

            ssb.Enable = obj.EnableSSB;
            ssb.BlockPattern = obj.BlockPattern;
            ssb.TransmittedBlocks = obj.TransmittedBlocks;
            ssb.Period = obj.Period;
            if obj.HalfFrameOffset~=0
                ssb.Period(2) = obj.HalfFrameOffset;
            end
            ssb.Power = obj.Power;

            % NCRBSSB/KSSB are handled slighlty differently from programmaric API
            if strcmp(obj.FrequencyOffset, getCarrierCenterFrequencyOffsetString())
                ssb.NCRBSSB = [];
            else
                ssb.NCRBSSB = obj.NCRBSSB;
            end
            ssb.KSSB = obj.KSSB;
            ssb.SubcarrierSpacingCommon = obj.SubcarrierSpacingCommon;

            % Get the SS Burst configuration from the Data Source, if the
            % dialog exists
            className = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
            if isKey(obj.Parent.AppObj.pParameters.DialogsMap, className)
                ssbDataSourceDialog = obj.Parent.AppObj.pParameters.DialogsMap(className);
                ssb = addSSBDataSourceConfig(ssbDataSourceDialog, ssb);
            end
        end

        function frequencySourceChanged(obj)
            % Update visibility of NCRBSSB and KSSB fields based on explicit
            % control on frequency centering

            custom = strcmpi(obj.FrequencyOffset, getCustomFrequencyOffsetString());
            setVisible(obj, {'NCRBSSB', 'KSSB'}, custom);
            if custom
                if isempty(obj.NCRBSSB)
                    try
                        scsCarriers = getWaveConfig(obj.CurrentDialog, 'scscarriers');
                        ssb = getSSBConfig(obj);
                        [centerNCRB,centerKSSB] = getSSBFrequencyPositionParameters(scsCarriers, ssb);
                        obj.NCRBSSB = centerNCRB;
                        obj.KSSB = centerKSSB;
                    catch e
                        updateAppConfigDiagnostic(obj, e);
                    end
                end
                updateCustomFrequencyOffsetUnits(obj);
            end
        end

        function frequencySourceChangedGUI(obj)
            % Frequency source changed following a direct GUI interaction
            updateControlsVisibility(obj);
            updateGrid(obj.CurrentDialog);
        end

        function subcarrierSpacingCommonChangedGUI(obj)
            % Change the units of NCRBSSB and KSSB when subcarrier spacing
            % common changes
            subcarrierSpacingCommonChanged(obj);
            updateSib1(obj);
        end


        function subcarrierSpacingCommonChanged(obj)
            updateCustomFrequencyOffsetUnits(obj);
        end

        function updateSubcarrierSpacingCommonVisibility(obj)
            % Update visibility of SubcarrierSpacingCommon
            ssbDataSourceFlag = strcmp(obj.DependentDialogs,'wirelessWaveformGenerator.nr5G_SSB_DataSource');
            if obj.EnableDependentDialog(ssbDataSourceFlag)
                className = obj.DependentDialogs{ssbDataSourceFlag};
                dataSourceDLG = obj.Parent.AppObj.pParameters.DialogsMap(className);
                scsSSB = nr5g.internal.wavegen.blockPattern2SCS(obj.BlockPattern,obj.SubcarrierSpacingCommon);
                custom = strcmpi(obj.FrequencyOffset, getCustomFrequencyOffsetString());
                scsCommonVisible = (scsSSB>=120 && custom && ~isempty(obj.NCRBSSB)) || ...
                    (strcmp(dataSourceDLG.DataSource, 'MIB'));
                setVisible(obj, 'SubcarrierSpacingCommon', scsCommonVisible);
            end
        end

        function parent = getParent(obj)
            % needed to fetch protected property
            parent = obj.Parent;
        end
    end

    % Validators and visibility updates
    methods (Access = private)

        function EnableChangedGUI(obj)
            ssbDataSourceFlag = strcmp(obj.DependentDialogs,'wirelessWaveformGenerator.nr5G_SSB_DataSource');
            if ~isempty(ssbDataSourceFlag)
                className = obj.DependentDialogs{ssbDataSourceFlag};
                dataSourceDLG = obj.Parent.AppObj.pParameters.DialogsMap(className);
                if obj.EnableSSB
                    updateSib1(obj)
                else
                    if dataSourceDLG.Sib1Check
                        % Disable/delete SIB1 if SSB just got disabled
                        dataSourceDLG.Sib1Check = 0;
                        sib1Delete(dataSourceDLG)
                    end
                end
                updateControlsVisibility(obj)
            end
            updateGrid(obj.CurrentDialog);
        end

        function PeriodChangedGUI(obj)
            updateSib1(obj)
            updateGrid(obj.CurrentDialog);
        end

        function powerChangedGUI(obj, src, evnt)
            % First, check that the standalone value is correct
            ME = updateProperty(obj, src, evnt, "Power");

            % If there is no error, do the cross-parameter validation
            if isempty(ME)
                e = []; % Initialize empty exception
                try
                    val = obj.Power;
                    %use error throwing behavior of nrWavegenSSBConfig
                    c = obj.DefaultCfg;
                    c.Power = val;
                    c.TransmittedBlocks = obj.TransmittedBlocks;
                    c.validateConfig;
                    updateSib1(obj);
                catch e
                end
                % Update message
                updateAppConfigDiagnostic(obj,e);
            end
        end

        function txBlocksChangedGUI(obj, src, evnt)
            % First, check that the standalone value is correct
            function validationFcn(in)
                % The App also supports vector of integer indices, e.g., [3 7 51]
                % So needs to have a custom validation
                if any(in>1) || isscalar(in)
                    validateattributes(in,{'double'},{'integer', 'row', 'positive', '<=' 64}, '', 'Transmitted Blocks');
                else
                    % binary vector; length must be 4, 8 or 64
                    validateattributes(in,{'double'},{'binary','row'}, '', 'Transmitted Blocks');
                    if ~any(length(in)==[4 8 64])
                        error(message('nr5g:nrWaveformGenerator:InvalidTxBlocks'));
                    end
                end
            end
            ME = updateProperty(obj, src, evnt, "TransmittedBlocks", ValidationFunction=@validationFcn);

            % If there is no error, do the cross-parameter validation
            if isempty(ME)
                e = []; % Initialize empty exception
                try
                    updateSib1(obj);
                    updateGrid(obj.CurrentDialog);
                catch e
                end
                % Update message
                updateAppConfigDiagnostic(obj,e);
            end
        end

        function halfFrameOffsetChangedGUI(obj, src, evnt)
            % First, check that the standalone value is correct
            function validationFcn(in)
                validateattributes(in,{'numeric'},{'nonempty','finite','nonnegative'});
                coder.internal.errorIf(mod(in,5),'nr5g:nrWaveformGenerator:InvalidSSBHalfFrameOffset');
            end
            ME = updateProperty(obj, src, evnt, "Period", FieldNames="HalfFrameOffset", ValidationFunction=@validationFcn);

            % If there is no error, do the cross-parameter validation
            if isempty(ME)
                e = []; % Initialize empty exception
                try
                    % Check the value against the Period
                    cfg = obj.DefaultCfg;
                    cfg.Period = [obj.Period obj.HalfFrameOffset];

                    % Update SIB1 and grid
                    updateSib1(obj)
                    updateGrid(obj.CurrentDialog);
                catch e
                end
                % Update message
                updateAppConfigDiagnostic(obj,e);
            end
        end

        function numCRBChangedGUI(obj, src, evnt)
            ME = updateProperty(obj, src, evnt, "NCRBSSB");

            % If there is no error, update the dependent visibility
            if isempty(ME)
                updateSubcarrierSpacingCommonVisibility(obj);
            end
        end

        function blockPatternChangedGUI(obj)
            updateGrid(obj.CurrentDialog);
            updateCustomFrequencyOffsetUnits(obj);
            updateSib1(obj)
        end
    end

    % Getters/setters
    methods
        function fr = get.BlockPattern(obj)
            fr = getDropdownVal(obj, 'BlockPattern');
            str = 'Case X';
            fr = fr(1:length(str));
        end
        function set.BlockPattern(obj, val)
            setDropdownStartingVal(obj, 'BlockPattern', val);
        end

        function n = get.TransmittedBlocks(obj)
            % Also support vector of integer indices, e.g., [3 7 51]
            n = getEditVal(obj, 'TransmittedBlocks');
            lengths = [4 8 64];
            if any(n>1) || (~any(length(n)==lengths) && all(n) && numel(n)==numel(unique(n)))
                % Instead of a binary c
                maxL = lengths(min(3, find(max(n) > [-1 lengths], 1, 'last')));
                temp = zeros(1, maxL);
                temp(n) = 1;
                n = temp;
            end
        end
        function set.TransmittedBlocks(obj, val)
            setEditVal(obj, 'TransmittedBlocks', val);
        end

        function scs = get.SubcarrierSpacingCommon(obj)
            scs = getDropdownVal(obj, 'SubcarrierSpacingCommon');
            scs = str2double(scs(1:end-4)); % drop ' kHz'
        end
        function set.SubcarrierSpacingCommon(obj, val)
            setDropdownStartingVal(obj, 'SubcarrierSpacingCommon', num2str(val));
            subcarrierSpacingCommonChanged(obj);
        end

        function out = get.ChannelName(~)
            out = 'SSB';
        end
    end

    methods

        function updateCustomFrequencyOffsetUnits(obj)
            % Customize labels with NCRBSSB and KSSB units

            custom = strcmpi(obj.FrequencyOffset, getCustomFrequencyOffsetString());
            if custom
                scsCommon = obj.SubcarrierSpacingCommon;
                [~,scsKSSB,scsNCRBSSB] = nr5g.internal.wavegen.blockPattern2SCS(obj.BlockPattern,scsCommon);
                obj.NCRBSSBLabel.Text = getNCRBSSBCustomLabel(scsNCRBSSB);
                obj.KSSBLabel.Text = getKSSBCustomLabel(scsKSSB);
            end
        end

        function layoutPanels(obj)
            layoutPanels@wirelessAppContainer.Dialog(obj);
            obj.Parent.Layout.ColumnWidth = {285, 357}; % do not let SSBurst panels span the whole tab
        end

    end
end

function s = getCarrierCenterFrequencyOffsetString()

    s = getString(message('nr5g:waveformApp:SSFrequencyOffsetCarrierCenter'));

end

function s = getCustomFrequencyOffsetString()

    s = getString(message('nr5g:waveformApp:SSFrequencyOffsetCustom'));

end

function s = getNCRBSSBCustomLabel(scs)

    s = getString(message('nr5g:waveformApp:NCRBSSBSCS',scs));

end

function s = getKSSBCustomLabel(scs)

    s = getString(message('nr5g:waveformApp:KSSBSCS',scs));

end

function [NCRB,KSSB] = getSSBFrequencyPositionParameters(scsCarriers, ssb)
    % Calculate the values of NCRBSSB and KSSB to center the SSB in its SCS
    % carrier

    [scsSSB,~,scsNCRBSSB] = nr5g.internal.wavegen.blockPattern2SCS(ssb.BlockPattern);
    scsCarrier = cellfun(@(x) x.SubcarrierSpacing, scsCarriers);
    carrierIdx = scsCarrier==scsSSB;
    if ~any(carrierIdx)
        % No SCS carrier exists for this SSB
        coder.internal.error('nr5g:nrWaveformGenerator:SSBNotInCarrier', scsSSB, ssb.BlockPattern);
    end
    burstCarrier = scsCarriers{carrierIdx};

    scsratio = scsSSB/scsNCRBSSB;
    minNCRB = burstCarrier.NStartGrid*scsratio;
    maxNCRB = (burstCarrier.NStartGrid+burstCarrier.NSizeGrid-20)*scsratio;
    NCRB = (minNCRB + maxNCRB)/2;
    KSSB = 0;

    % If the value of NCRBSSB is fractional, floor it and set KSSB to 6 to
    % center the SSB in its SCS carrier. Fractional values can only occur
    % for Case A since NCRBSSB is expressed in terms of 15 kHz and 60 kHz
    % subcarrier spacings in FR1 and FR2, respectively (TS 38.211 Section
    % 7.4.3.1). For Cases B to G, scsratio above is >= 2.
    if mod(NCRB,1)
        NCRB = floor(NCRB);
        KSSB = 6;
    end
end

function updateSib1(obj)
    ssbDataSourceFlag = strcmp(obj.DependentDialogs,'wirelessWaveformGenerator.nr5G_SSB_DataSource');
    if ~isempty(ssbDataSourceFlag)
        className = obj.DependentDialogs{ssbDataSourceFlag};
        dataSourceDLG = obj.Parent.AppObj.pParameters.DialogsMap(className);

        if dataSourceDLG.Sib1Check
            dlgSCSCheck = obj.Parent.AppObj.pParameters.CurrentDialog;
            scsCarriers = getWaveConfig(dlgSCSCheck, 'scscarriers');
            scsCommon = obj.SubcarrierSpacingCommon;
            allScs = [scsCarriers{:}];
            scsForBWPCommon = scsCommon == [allScs.SubcarrierSpacing];
            try
                coder.internal.errorIf(~any(scsForBWPCommon),'nr5g:waveformGeneratorApp:Sib1NoSCS',scsCommon);
                updateSIB1Config(dataSourceDLG);
            catch e
                throwErrorPopup(obj, e);
                % The only error this can catch is the Sib1NoSCS error
                % Set SubcarrierSpacingCommonGUI drop down to the only other value, that is known to exist as a carrier.
                obj.SubcarrierSpacingCommonGUI.Value = obj.SubcarrierSpacingCommonGUI.Items(~strcmp(obj.SubcarrierSpacingCommonGUI.Value, obj.SubcarrierSpacingCommonGUI.Items));
            end
        end
    end
end