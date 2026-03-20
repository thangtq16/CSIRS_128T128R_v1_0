classdef nr5G_Presets_Dialog < wirelessWaveformGenerator.nr5G_Dialog
    % Common functionality for 5G preset (TMs and FRCs) configurations

    %   Copyright 2022-2024 The MathWorks, Inc.

    properties
        maxNumBWPFigs = 1;      % An upper bound on BWP figs to be visualized
    end

    properties(Access=private,Hidden)
        pCopyPresetBtn = [];
        CopyPresetBtnTag = 'copyPresetBtn'; % Tag for the copy preset button
        CopyPresetColTag = 'copyPresetCol'; % Tag for the column containing the copy preset button
    end

    properties(Abstract,Hidden)
        % Properties used to open a copy of this preset definition in a
        % Downlink/Uplink waveform type
        ThisWaveformType
        NewWaveformType
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_Presets_Dialog(parent)
            obj@wirelessWaveformGenerator.nr5G_Dialog(parent); % call base constructor

            % Sync GUI controls with dialog state
            windowingChangedGUI(obj, []);
            srChangedGUI(obj, []);
            phaseCompChangedGUI(obj, []);
        end

        % Set up dialog when the waveform type changes to a preset
        function setupDialog(obj)
            import matlab.ui.internal.toolstrip.*

            setupDialog@wirelessWaveformGenerator.nr5G_Dialog(obj);

            % Clear any existing banner from previous waveform types, if
            % any, or restore the previous banner, if coming back to a
            % waveform type with configuration issues
            bannerContainer = findall(obj.Figure,Tag="BannerContainer");
            if ~isempty(bannerContainer) && ishghandle(bannerContainer)
                delete(bannerContainer);
            end
            % Set padding on the top to create more real estate so that the
            % banner doesn't cover anything
            setPaddingForBanner(obj);
            % Add callback to resize the banner when the figure resizes
            obj.Figure.AutoResizeChildren = 'off';
            obj.Figure.SizeChangedFcn = @(a, b) resizeBanner(obj);

            % Add a button in the toolstrip to copy this preset definition
            % to a full Downlink/Uplink waveform type
            % Get the toolstrip tab and its children
            tab = obj.Parent.WaveformGenerator.pPrimaryTab; % Toolstrip tab
            sec = tab.find('generalSection'); % Waveform Type section
            btn = tab.find(obj.CopyPresetBtnTag); % Copy presets button

            % Make sure the copy preset button exists and is visible
            if isempty(btn)
                if isempty(obj.pCopyPresetBtn)
                    % If the pCopyPresetBtn property is empty, it
                    % means that the button has not been created yet.
                    % Create the button.

                    % Add a new column to the Waveform Type section
                    col = sec.addColumn();
                    col.Tag = obj.CopyPresetColTag;

                    % Create the button and add it to the new column
                    btn = Button('','copyCurrentValues');
                    btn.Tag = obj.CopyPresetBtnTag;
                    col.add(btn);
                else
                    % If the copy preset button object is not part of
                    % the toolstrip, it means that it was previously
                    % removed when moving to a different waveform type.
                    % Add it again.
                    sec.add(obj.pCopyPresetBtn.Parent);
                    btn = tab.find(obj.CopyPresetBtnTag);
                end
            end

            % Update the copy preset button properties
            btn.Enabled = true;
            btn.ButtonPushedFcn = @(src, evt) copyPresetCallback(obj, []);
            btn.Text = getString(message('nr5g:waveformGeneratorApp:CopyPresetBtn',obj.NewWaveformType));
            btn.Description = getString(message('nr5g:waveformGeneratorApp:CopyPresetTT',obj.ThisWaveformType,obj.NewWaveformType));
            obj.pCopyPresetBtn = btn;
        end

        function updateREVisual(obj,channelName,varargin)
            % Live update of the RE mapping grid when a relevant property
            % changes or waveform is generated

            if nargin==3
                resetFlag = matches(varargin{1},'reset');
                selection = 1;
            else
                resetFlag = false;
                selection = getChannelForREMapping(obj);
            end

            try
                % Update RE mapping
                if obj.getVisualState('Resource Grid (BWP#1)')
                    bwpIdx = 1;
                    ax = getResourceGridAxes(obj,bwpIdx);
                    wirelessWaveformGenerator.internal.updateREMapping(obj,ax,bwpIdx,channelName,selection,resetFlag);
                end

            catch e
                 updateConfigDiagnostic(obj, e.message);
            end
        end

        % Executed when moving to a new waveform type (e.g., 5G TMs -> 5G DL)
        function outro(obj, newDialog)

            outro@wirelessWaveformGenerator.nr5G_Dialog(obj);

            % Clear any configuration-related message
            updateConfigDiagnostic(obj, "");

            % Revert extra padding added for the banner to the original
            % value
            revertPaddingForBanner(obj);

            % If the new waveform type is not a 5G preset, remove the copy
            % preset button from the toolstrip
            if ~isa(newDialog, 'wirelessWaveformGenerator.nr5G_TM_Dialog') && ...
                    ~isa(newDialog, 'wirelessWaveformGenerator.nr5G_FRC_DL_Dialog') && ...
                    ~isa(newDialog, 'wirelessWaveformGenerator.nr5G_FRC_UL_Dialog')

                % Get the toolstrip tab and its children
                tab = obj.Parent.WaveformGenerator.pPrimaryTab; % Toolstrip tab
                sec = tab.find('generalSection'); % Waveform Type section
                col = sec.find(obj.CopyPresetColTag); % Column containing the expand presets button

                % Remove the added column from the Waveform Type section
                sec.remove(col);
            end

            % If the copy preset button is pushed, apply the current
            % configuration to the target waveform type
            if ~obj.pCopyPresetBtn.Enabled
                % Get current waveform configuration
                cfg = getConfiguration(obj);

                % Apply configuration to the new waveform type
                newDialog.Parent.CurrentDialog = newDialog;
                applyConfiguration(newDialog, cfg);

                % Clear SIB1 check if it was enabled
                classNameDS = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
                if isKey(obj.getParent.DialogsMap, classNameDS)
                    dlgSSBDS = obj.getParent.DialogsMap(classNameDS);
                    dlgSSBDS.Sib1Check = 0;
                end

                % Restoring SIB1 dialog for new sessions
                classNameSIB1 = 'wirelessWaveformGenerator.nr5G_SSB_SIB1_Dialog';
                if isKey(obj.getParent.DialogsMap, classNameSIB1)
                    dlgSIB1 = obj.getParent.DialogsMap(classNameSIB1);
                    restoreDefaults(dlgSIB1);
                end

                % Set the flag to true to make sure the app doesn't override
                % the filtering configuration
                obj.Parent.FilteringDialog.keepFiltering = true;
            end

        end

        % Apply 'configuration' that may be presented from previously saved 'dialog state'
        function applyConfiguration(obj, cfg)
            % Direct name based field/property mapping from 'cfg' contents
            % to matching dialog properties
            applyConfiguration@wirelessWaveformGenerator.nr5G_Dialog(obj, cfg);

            % Make sure the info dialog is updated
            updateInfo(obj);

            % Update GUI controls that depend on dialog properties
            windowingChangedGUI(obj, []);
            srChangedGUI(obj, []);
            phaseCompChangedGUI(obj, []);
        end

        function frChanged(obj, ~)
            frChangedBase(obj);
            scsChanged(obj);
        end

        function frChangedGUI(obj, ~)
            % FR has changed via a direct GUI interaction: Update
            % everything related to it, including the info panel and the
            % grid and channel visualizations
            frChanged(obj);
            updateInfo(obj);
            updateGrid(obj);
            updateChannelBandwidthView(obj);
        end
    end

    % Private methods
    methods (Access = private)

        % Switch waveform type from preset (FRC or TM) to full DL/UL when
        % the button is pushed
        function copyPresetCallback(obj,~)

            % Disable the button as soon as it is pushed to avoid multiple
            % clicks
            obj.pCopyPresetBtn.Enabled = false;

            % Before switching waveform type, ensure that the current
            % configuration is valid to avoid the app hanging in between
            % two waveform types due to an invalid configuration
            try
                getConfiguration(obj);
            catch ME
                % The configuration was invalid: Re-enable the button
                % and throw an error
                obj.pCopyPresetBtn.Enabled = true;
                obj.errorFromException(ME);
                return;
            end

            % Pop out dialog box to explain that this will overwrite the
            % existing DL/UL configuration
            fig = obj.Parent.Layout.Parent; % Display the question dialog in the current app figure
            dlgMsg = getString(message('nr5g:waveformGeneratorApp:CopyPresetDlgMsg',obj.NewWaveformType));
            dlgTitle = getString(message('comm:waveformGenerator:DialogTitle'));
            dlgYes = getString(message('nr5g:waveformGeneratorApp:Yes'));
            dlgNo = getString(message('nr5g:waveformGeneratorApp:No'));
            answer = uiconfirm(fig,dlgMsg,dlgTitle,'Options',{dlgYes,dlgNo},'DefaultOption',1,'CancelOption',2);
            noOp = ~matches(answer,dlgYes);

            if ~noOp
                % Switch waveform to the full DL/UL waveform type
                extensionTypeChange(obj.Parent.WaveformGenerator, obj.NewWaveformType);
            else
                % Re-enable the button
                obj.pCopyPresetBtn.Enabled = true;
            end
        end
    end
end
