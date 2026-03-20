classdef nr5G_PUSCH_Tab < wirelessWaveformApp.nr5G_PXSCH_Tab
    % Dialog class that handles every graphical aspect related to PUSCH

    % Copyright 2024-2025 The MathWorks, Inc.

    properties
        % Object-specific properties
        pxschTable % PUSCH table object
    end

    properties (Constant)
        PXSCHfigureName    = 'PUSCH'; % Channel and figure name
        pxschExtraFigTag   = 'puschSingleChannelFig'; % Side panel figure tag
    end

    properties (SetAccess = protected, GetAccess = public)
        % Side panel class names
        classNamePXSCHAdv  = 'wirelessWaveformApp.nr5G_PUSCHAdvanced_Dialog';
        classNamePXSCHDMRS = 'wirelessWaveformApp.nr5G_PUSCHDMRS_Dialog';
        classNamePXSCHPTRS = 'wirelessWaveformApp.nr5G_PUSCHPTRS_Dialog';
    end

    % Constructor and public methods
    methods (Access = public)
        function obj = nr5G_PUSCH_Tab(defaultWaveConfig, invisibleEntries)
            obj@wirelessWaveformApp.nr5G_PXSCH_Tab(); % call base constructor

            % Construct the table object
            defaultConfigPXSCH = defaultWaveConfig.PUSCH;
            obj.pxschTable = wirelessWaveformApp.nr5G_PUSCH_Table(obj.pxschGridLayout, defaultConfigPXSCH, invisibleEntries);

            % Initialize the cached configuration object
            obj.pxschWaveConfig = defaultConfigPXSCH;
            obj.DefaultConfigPXSCH = defaultConfigPXSCH;
        end

        %% PXSCH Configuration
        function waveConfig = updateCachedConfigPXSCH(obj, action, changedConfigIndex)
            % Update the nrWavegenXConfig configuration object and the XL
            % cached configuration object from the table and the potential
            % side panel.

            % Call base class method
            waveConfig = updateCachedConfigPXSCH@wirelessWaveformApp.nr5G_PXSCH_Tab(obj, action, changedConfigIndex);

            % Turn off interlacing in case SCS and BWP tables have been modified
            % and interlacing is no longer supported for some instances
            waveConfig = turnOffUnsupportedInterlacing(obj, waveConfig);

            % Update the cache
            obj.pxschWaveConfig = waveConfig;
        end

        %% Update visibility of dependent parameters in the side panel
        function updateControlsVisibilityPXSCH(obj)
            % Update the visibility of all dependent parameters in the side
            % panel.

            % PUSCH-specific methods
            needRepaint = false;
            needRepaint = updateModDependentVisibility(obj) || needRepaint;
            needRepaint = updatePrecodingDependentVisibility(obj) || needRepaint;

            % Call baseclass method
            updateControlsVisibilityPXSCH@wirelessWaveformApp.nr5G_PXSCH_Tab(obj, needRepaint);
        end
    end

    methods (Access = protected)
        %% Side panel
        function dlg = mapCache2PXSCHAdv(~, pxsch, dlg)
            % Set PUSCH-specific advanced properties
            dlg.TransmissionScheme    = pxsch.TransmissionScheme;
            dlg.NumAntennaPorts       = pxsch.NumAntennaPorts;
            dlg.TPMI                  = pxsch.TPMI;
            dlg.FrequencyHopping      = pxsch.FrequencyHopping;
            dlg.SecondHopStartPRB     = pxsch.SecondHopStartPRB;
            dlg.NRAPID                = pxsch.NRAPID;
            dlg.Interlacing           = pxsch.Interlacing;
            dlg.InterlaceIndex        = pxsch.InterlaceIndex;
            dlg.RBSetIndex            = pxsch.RBSetIndex;
            % dlg.BetaOffsetACK         = pxsch.BetaOffsetACK;
            % dlg.BetaOffsetCSI1        = pxsch.BetaOffsetCSI1;
            % dlg.BetaOffsetCSI2        = pxsch.BetaOffsetCSI2;
            % dlg.UCIScaling            = pxsch.UCIScaling;
            % dlg.DisableULSCH          = pxsch.DisableULSCH;
        end

        function dlg = mapCache2PXSCHDMRS(~, pxsch, dlg)
            % Set PUSCH-specific DM-RS properties
            dlg.GroupHopping           = pxsch.DMRS.GroupHopping;
            dlg.SequenceHopping        = pxsch.DMRS.SequenceHopping;
            dlg.NRSID                  = pxsch.DMRS.NRSID;
            dlg.DMRSUplinkR16          = pxsch.DMRS.DMRSUplinkR16;
            dlg.DMRSUplinkTransformPrecodingR16 = pxsch.DMRS.DMRSUplinkTransformPrecodingR16;
        end

        function dlg = mapCache2PXSCHPTRS(~, pxsch, dlg)
            % Set PUSCH-specific PT-RS properties
            dlg.NumPTRSSamples = pxsch.PTRS.NumPTRSSamples;
            dlg.NumPTRSGroups  = pxsch.PTRS.NumPTRSGroups;
            dlg.NID            = pxsch.PTRS.NID;
        end

        function pxsch = mapPXSCHAdv2Cache(~, pxsch, dlg)
            % Set PUSCH-specific advanced properties
            pxsch.TransmissionScheme  = dlg.TransmissionScheme;
            pxsch.NumAntennaPorts     = dlg.NumAntennaPorts;
            pxsch.TPMI                = dlg.TPMI;
            pxsch.FrequencyHopping    = dlg.FrequencyHopping;
            pxsch.SecondHopStartPRB   = dlg.SecondHopStartPRB;
            pxsch.NRAPID              = dlg.NRAPID;
            pxsch.Interlacing         = dlg.Interlacing;
            pxsch.InterlaceIndex      = dlg.InterlaceIndex;
            pxsch.RBSetIndex          = dlg.RBSetIndex;
            % pxsch.BetaOffsetACK       = dlg.BetaOffsetACK;
            % pxsch.BetaOffsetCSI1      = dlg.BetaOffsetCSI1;
            % pxsch.BetaOffsetCSI2      = dlg.BetaOffsetCSI2;
            % pxsch.UCIScaling          = dlg.UCIScaling;
            % pxsch.DisableULSCH        = dlg.DisableULSCH;
        end

        function pxsch = mapPXSCHDMRS2Cache(~, pxsch, dlg)
            % Set PUSCH-specific DM-RS properties
            if pxsch.TransformPrecoding
                % Force the DMRSConfigurationType value to 1
                dlg.DMRSConfigurationType = 1;
                dlg.NumCDMGroupsWithoutData = 2;
            end
            pxsch.DMRS.DMRSConfigurationType  = dlg.DMRSConfigurationType;
            pxsch.DMRS.NumCDMGroupsWithoutData= dlg.NumCDMGroupsWithoutData;
            pxsch.DMRS.GroupHopping           = dlg.GroupHopping;
            pxsch.DMRS.SequenceHopping        = dlg.SequenceHopping;
            pxsch.DMRS.NRSID                  = dlg.NRSID;
            pxsch.DMRS.DMRSUplinkR16          = dlg.DMRSUplinkR16;
            pxsch.DMRS.DMRSUplinkTransformPrecodingR16 = dlg.DMRSUplinkTransformPrecodingR16;
        end

        function pxsch = mapPXSCHPTRS2Cache(~, pxsch, dlg)
            % Set PUSCH-specific PT-RS properties
            pxsch.PTRS.NumPTRSSamples = dlg.NumPTRSSamples;
            pxsch.PTRS.NumPTRSGroups  = dlg.NumPTRSGroups;
            pxsch.PTRS.NID            = dlg.NID;
        end
    end

    methods (Access = private)
        %% Dependent controls visibility
        function needRepaint = updatePrecodingDependentVisibility(obj)
            % Transform precoding affects visibility in DMRS and PTRS panels

            % Adjust visuals only if current row is displayed in
            % "advanced", as the user can toggle the Transform Precoding
            % checkbox without actually selecting the row
            currPXSCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pxschTable.Selection);
            if obj.pxschTable.AllIDs(currPXSCH) == getAdvancedPXSCHID(obj)

                % Get Transform Precoding and Modulation values:
                precoding  = obj.pxschWaveConfig{currPXSCH}.TransformPrecoding;
                modulation = obj.pxschWaveConfig{currPXSCH}.Modulation;

                % Get DMRS dialog object
                appObj = getParent(obj);
                dlgDMRS = appObj.DialogsMap(obj.classNamePXSCHDMRS);
                oldVis = isVisible(dlgDMRS, 'GroupHopping');

                needRepaint = xor(oldVis, precoding);
                if needRepaint
                    % do re-layout only when it is needed, because it is expensive

                    % Properties that should only appear when Transform Precoding is on:
                    setVisible(dlgDMRS, {'GroupHopping', 'SequenceHopping', 'NRSID'}, precoding);

                    % Properties that should only appear when Transform Precoding is on and Modulation is pi/2-BPSK:
                    if iscell(modulation)
                        modulation = modulation{1}; % only checking value of first modulation as two codewords not applicable for transform precoding
                    end
                    setVisible(dlgDMRS, 'DMRSUplinkTransformPrecodingR16', precoding && strcmpi(modulation,'pi/2-BPSK'));

                    % Properties that should not appear when Transform Precoding is on:
                    setVisible(dlgDMRS, {'NIDNSCID', 'NSCID', 'DMRSUplinkR16'}, ~precoding);

                    % PTRS changes from Transform Precoding changes are handled by
                    % updatePTRSEnable -> set.Enable
                end
            else
                needRepaint = false;
            end
        end

        function needRepaint = updateModDependentVisibility(obj)
            % Modulation affects visibility in PUSCH DMRS panel when transform precoding is on

            % Get Modulation value:
            currPXSCH = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.pxschTable.Selection);
            modulation = obj.pxschWaveConfig{currPXSCH}.Modulation;
            if iscell(modulation)
                modulation = modulation{1}; % only checking value of first modulation as two codewords not applicable for transform precoding
            end
            modPI2BPSK = strcmpi(modulation, 'pi/2-BPSK');

            % Get DMRS dialog object
            appObj = getParent(obj);
            dlgDMRS = appObj.DialogsMap(obj.classNamePXSCHDMRS);
            oldVis = isVisible(dlgDMRS, 'DMRSUplinkTransformPrecodingR16');

            needRepaint = xor(oldVis, modPI2BPSK);
            if needRepaint
                % do re-layout only when it is needed, because it is expensive

                % Properties that should only appear when Modulation is pi/2-BPSK (and transform precoding is on):
                setVisible(dlgDMRS, 'DMRSUplinkTransformPrecodingR16', modPI2BPSK);

                % Adjust the visibility of the properties affected by DMRSUplinkTransformPrecodingR16
                DMRSUplinkTransformPrecodingR16Changed(dlgDMRS);
            end
        end

        function waveConfig = turnOffUnsupportedInterlacing(obj, waveConfig)
            % Check whether interlacing is applicable throughout the config
            % and turn off those not supported

            for idx = 1:numel(waveConfig)
                % Only update the interlacing property is this instance has
                % interlacing flag on and does not support interlacing
                % anymore
                if waveConfig{idx}.Interlacing && ~supportInterlacing(obj, waveConfig{idx})
                    waveConfig{idx}.Interlacing = false; % turn off interlacing
                    if (((isempty(obj.pxschTable.Selection) && idx == 1) || ...
                            (~isempty(obj.pxschTable.Selection) && idx == obj.pxschTable.Selection(1))) && ...
                            isKey(obj.getParent.DialogsMap,obj.classNamePXSCHAdv))
                        dlg = obj.getParent.DialogsMap(obj.classNamePXSCHAdv);
                        dlg.Interlacing = false; % untick interlacing checkbox if currently selected
                    end
                end
            end
        end

        function flag = supportInterlacing(obj, puschWaveConfig)
            % Returns true or false whether the current PUSCH instance supports
            % interlacing or not. Only PUSCH instances linked to bandwidth parts
            % with 15 kHz or 30 kHz subcarrier spacing support interlacing.

            bwpID = puschWaveConfig.BandwidthPartID;
            availableBWPIDs = cellfun(@(x)(cat(1,[],x.BandwidthPartID)),obj.bwpWaveConfig);
            bwpRowNum = (availableBWPIDs == bwpID);
            if any(bwpRowNum)
                scs = obj.bwpWaveConfig{bwpRowNum}.SubcarrierSpacing;
                flag = (scs == 15 || scs == 30);
            else
                flag = false; % BWP no longer exists - treat as not supporting interlacing
            end
        end
    end
end
