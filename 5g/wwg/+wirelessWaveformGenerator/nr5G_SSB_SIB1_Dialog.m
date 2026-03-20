classdef nr5G_SSB_SIB1_Dialog <  wirelessWaveformApp.nr5G_Dialog
    % Dialog panel containing SIB1 Parameters

    %   Copyright 2024 The MathWorks, Inc.

    properties (Hidden = true, Access = public)
        TitleString = getString(message('nr5g:waveformGeneratorApp:Sib1Title'))

        % GUI
        Sib1MinBWType = 'numericPopup'
        Sib1MinBWDropDown = {'5','10','40'}
        Sib1MinBWGUI
        Sib1MinBWLabel

        Sib1FreqDomainType = 'numericEdit'
        Sib1FreqDomainGUI
        Sib1FreqDomainLabel

        Sib1SymbolSetType = 'charPopup'
        Sib1SymbolSetDropDown = {''}
        Sib1SymbolSetGUI
        Sib1SymbolSetLabel

        Sib1SymbolSet2Type = 'charPopup'
        Sib1SymbolSet2DropDown = {''}
        Sib1SymbolSet2GUI
        Sib1SymbolSet2Label

        Sib1MCSTCRType = 'charPopup'
        Sib1MCSTCRDropDown = {''};
        Sib1MCSTCRGUI
        Sib1MCSTCRLabel

        Sib1TBSType = 'numericText'
        Sib1TBSGUI
        Sib1TBSLabel

        Sib1RVType = 'numericEdit'
        Sib1RVGUI
        Sib1RVLabel

        Sib1InterleaveType = 'checkbox'
        Sib1InterleaveGUI
        Sib1InterleaveLabel

        Sib1PayloadType = 'charPopup'
        Sib1PayloadDropDown = {'PN9-ITU', 'PN9', 'PN11', 'PN15', 'PN23', 'User-defined'}
        Sib1PayloadGUI
        Sib1PayloadLabel

        Sib1PayloadCustomType = 'numericEdit'
        Sib1PayloadCustomGUI
        Sib1PayloadCustomLabel

    end %properties Hidden public

    properties
        Sib1TimeDomain
        Sib1TimeDomain2
        Sib1MCS
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    methods (Access = public)

        function obj = nr5G_SSB_SIB1_Dialog(parent, fig)
            obj@ wirelessWaveformApp.nr5G_Dialog(parent, fig, wirelessWaveformApp.nr5G_SSB_Dialog.DefaultCfg); % call base constructor
            obj.Sib1MinBWGUI.(obj.Callback)             = @(src,evnt) Sib1UpdateGUI(obj);
            obj.Sib1SymbolSetGUI.(obj.Callback)         = @(src,evnt) Sib1UpdateGUI(obj);
            obj.Sib1SymbolSet2GUI.(obj.Callback)        = @(src,evnt) Sib1UpdateGUI(obj);
            obj.Sib1FreqDomainGUI.(obj.Callback)        = @(src,evnt) Sib1FreqDomainChangedGUI(obj,src,evnt);
            obj.Sib1MCSTCRGUI.(obj.Callback)            = @(src,evnt) Sib1UpdateGUI(obj);
            obj.Sib1RVGUI.(obj.Callback)                = @(src,evnt) Sib1RVChangedGUI(obj,src,evnt);
            obj.Sib1InterleaveGUI.(obj.Callback)        = @(src,evnt) Sib1UpdateGUI(obj);
            obj.Sib1PayloadGUI.(obj.Callback)           = @(src,evnt) Sib1PayloadChangedGUI(obj);
            obj.Sib1PayloadCustomGUI.(obj.Callback)     = @(src,evnt) Sib1PayloadCustomChangedGUI(obj,src,evnt);
            restoreDefaults(obj);
        end %constructor

        function adjustSpec(obj)
            % This is needed so that the SSB panel(s) do not horizontally fill the entire App
            obj.panelFixedSize = true;
        end

        function props = displayOrder(~)
            props = {'Sib1MinBW';'Sib1FreqDomain'; 'Sib1SymbolSet'; 'Sib1SymbolSet2'; 'Sib1MCSTCR'; ...
                'Sib1TBS'; 'Sib1RV'; 'Sib1Interleave'; 'Sib1Payload';'Sib1PayloadCustom';};
        end

        function restoreDefaults(obj)
            % Set SIB1 params
            obj.Sib1MinBW = 5;
            obj.Sib1FreqDomain = [0 8];
            obj.Sib1RV = 0;
            obj.Sib1Interleave = 0;
            obj.Sib1Payload ='PN9-ITU';
            obj.Sib1PayloadCustom = [1;0;0;1];
            obj.Sib1TimeDomain = 1;
            obj.Sib1TimeDomain2 = 1;
            obj.Sib1MCS = 0;

            % Set dynamic GUI Values
            Sib1FR2View(obj, 1); % Start in FR1
            mcsTables=nrPDSCHMCSTables;
            % Set the MCS dropdown to only the QPSK values (values 0-9)
            qpskCodeRates = round(mcsTables.QAM64Table.TargetCodeRate(1:10),4);
            mscDropDown = cellstr([num2str((0:9)'), repmat(', Target code rate: ',10,1),num2str(qpskCodeRates)])';
            obj.Sib1MCSTCRGUI.(obj.DropdownValues) = mscDropDown;
            % Set the SymbolSet drop down based on default DMRS
            % Sib1SymbolSetChange calls Sib1TBSUpdate's default here.
            defaultSSB = nrWavegenSSBurstConfig;
            Sib1SymbolSetChange(obj,defaultSSB.DMRSTypeAPosition);
            % Start with some controls hidden
            setVisible(obj, {'Sib1SymbolSet2','Sib1PayloadCustom'}, 0);
            collapseSIB1Panel(obj,true);
            layoutUIControls(obj);
        end

        function str = getCatalogPrefix(~)
            str = 'nr5g:waveformGeneratorApp:';
        end

        function Sib1UpdateGUI(obj)
            % Get some numeric values from char drop-downs
            symbolSetStr = obj.Sib1SymbolSet;
            symbolSetStr2 = obj.Sib1SymbolSet2;
            mcsString = obj.Sib1MCSTCR;
            obj.Sib1TimeDomain = str2double(extract(extractBefore(string(symbolSetStr),"-"),digitsPattern));
            obj.Sib1TimeDomain2 = str2double(extract(extractBefore(string(symbolSetStr2),"-"),digitsPattern));
            obj.Sib1MCS = str2double(mcsString(1));
            Sib1TBSUpdate(obj)
            updateSib1(obj)
            updateGrid(obj.CurrentDialog);
        end

        function Sib1FreqDomainChangedGUI(obj, src, evnt)
            % First, check that the standalone value is correct
            function validationFcn(in)
                validateattributes(in,{'numeric'},{'nonnegative','numel', 2}, '', 'SIB1 frequency-domain resources');
                validateattributes(in(2),{'numeric'},{'positive','<',97}, '', 'SIB1 frequency-domain resources (RB Size)'); % Largest possible CORESET 0 size is 96
            end

            ME = updateProperty(obj, src, evnt, "Sib1FreqDomain", ValidationFunction=@validationFcn);

            % If there is no error, do the cross-parameter validation
            if isempty(ME)
                e = [];
                try
                    Sib1TBSUpdate(obj);
                    updateSib1(obj);
                    updateGrid(obj.CurrentDialog);
                catch e
                end

                % Update message
                updateAppConfigDiagnostic(obj,e);
            end
        end

        function Sib1RVChangedGUI(obj, src, evnt)
            % First, check that the standalone value is correct
            function validationFcn(in)
                validateattributes(in,{'numeric'},{'nonnegative', '<' 4}, '', 'redundancy version');
            end

            ME = updateProperty(obj, src, evnt, "Sib1RV", ValidationFunction=@validationFcn);

            % If there is no error, do the cross-parameter validation
            if isempty(ME)
                e = [];
                try
                    updateSib1(obj);
                    updateGrid(obj.CurrentDialog);
                catch e
                end

                % Update message
                updateAppConfigDiagnostic(obj,e);
            end
        end

        function Sib1PayloadChangedGUI(obj)
            customBitCheck     = strcmp(obj.Sib1Payload, 'User-defined');
            setVisible(obj, {'Sib1PayloadCustom'}, customBitCheck);
            updateSib1(obj);
            layoutUIControls(obj);
        end

        function Sib1PayloadCustomChangedGUI(obj,src,evnt)
            % First, check that the standalone value is correct
            function validationFcn(in)
                validateattributes(in,{'numeric'},{'vector', 'binary'}, '', 'custom data source');
            end

            ME = updateProperty(obj, src, evnt, "Sib1PayloadCustom", ValidationFunction=@validationFcn);

            % If there is no error, do the cross-parameter validation
            if isempty(ME)
                e = [];
                try
                    updateSib1(obj);
                catch e
                end

                % Update message
                updateAppConfigDiagnostic(obj,e);
            end
        end

        function Sib1TBSUpdate(obj)
            % Check app is loaded first.
            ssbDataSourceClassName = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
            if ~isKey(obj.Parent.WaveformGenerator.pParameters.DialogsMap,ssbDataSourceClassName)
                return;
            else
                dataSourceDLG = obj.Parent.WaveformGenerator.pParameters.DialogsMap(ssbDataSourceClassName);
                DMRSAPos = dataSourceDLG.SSBDMRSTypeAPosition;
            end

            % Get target coderate defined by MCS Index
            mcsString = obj.Sib1MCSTCR;
            tcr = str2double(extractAfter(string(mcsString),"rate: "));
            FreqResources = obj.Sib1FreqDomain;
            timeDomainArray = [obj.Sib1TimeDomain;obj.Sib1TimeDomain2];
            symbolSets = size(timeDomainArray,1);
            nBWPSize = []; % Don't need this input, we use FreqResources
            CoresetPattern = 1; % Only supported coreset pattern
            dci.VRBToPRBMapping = obj.Sib1Interleave; % Doesn't affect TBS but field needed

            tbs = zeros(1,symbolSets);
            % Calculate TBS for every symbolset required by SIB1
            for symIdx = 1:symbolSets
                dci.TimeDomainResources = timeDomainArray(symIdx)-1; % function is expecting DCI index so minus one from value.
                tbsPDSCH =  wirelessWaveformGenerator.internal.SIB1ConfigGen.hSIB1PDSCHConfiguration(dci,nBWPSize,DMRSAPos,CoresetPattern,FreqResources);
                tbs(symIdx) = tbsPDSCH.nrTBS(tcr);
            end

            obj.Sib1TBS = num2str(tbs(1));
            if (obj.Sib1SymbolSet2GUI.Visible && tbs(1)~=tbs(2))
                obj.Sib1TBS = [num2str(tbs(1)) ', ', num2str(tbs(2))];
            end
        end

        function Sib1FR2View(obj, FR1Flag)
            setVisible(obj, 'Sib1MinBW', FR1Flag);
            layoutUIControls(obj);
        end

        function Sib1SymbolSetChange(obj,DMRS)
            % Defined in TS 38.214 - Table 5.1.2.1.1-2
            TableDefaultA = {
                ['Row 1   - start  ',num2str(DMRS),',   length ', num2str(14-DMRS) ], ...
                ['Row 2   - start  ', num2str(DMRS),',   length ', num2str(12-DMRS)],...
                ['Row 3   - start  ', num2str(DMRS),',   length ', num2str(11-DMRS)],...
                ['Row 4   - start  ', num2str(DMRS),',   length ', num2str(9-DMRS)],...
                ['Row 5   - start  ', num2str(DMRS),',   length ', num2str(7-DMRS)],...
                ['Row 6   - start  ', num2str(DMRS+7),',   length 4'],...
                ['Row 7   - start  ', num2str(DMRS*2),',   length 4'],...
                'Row 8   - start  5,   length 7',...
                'Row 9   - start  5,   length 2' ,...
                'Row 10 - start  9,   length 2' ,...
                'Row 11 - start  12, length 2' ,...
                'Row 12 - start  1,   length 13' ,...
                'Row 13 - start  1,   length 6' ,...
                'Row 14 - start  2,   length 4', ...
                'Row 15 - start  4,   length 7', ...
                'Row 16 - start  8,   length 4' ...
                };

            obj.Sib1SymbolSetGUI.(obj.DropdownValues) = TableDefaultA;
            obj.Sib1SymbolSet2GUI.(obj.DropdownValues) = TableDefaultA;

            Sib1TBSUpdate(obj); % If this layout got changed, TBS likely needs recalculated.
        end

        function config = getConfigurationForSave(obj,sib1Enabled)
            config = getConfigurationForSave@wirelessAppContainer.Dialog(obj);
            % SIB1 state
            config.SIB1Enabled = sib1Enabled;
            % SIB1 channel row index tracking
            ssbDataSourceClassName = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
            if isKey(obj.Parent.WaveformGenerator.pParameters.DialogsMap,ssbDataSourceClassName)
                dataSourceDLG = obj.Parent.WaveformGenerator.pParameters.DialogsMap(ssbDataSourceClassName);
                % Ensure that the indices are up to date before saving them
                updateSIB1Config(dataSourceDLG, UpdateGrid=false);
                unfreezeApp(obj.CurrentDialog.getParent.AppObj);
                config.SIB1PDSCHIndex = dataSourceDLG.SIB1PDSCHIndex;
                config.SIB1PDCCHIndex = dataSourceDLG.SIB1PDCCHIndex;
                config.SIB1COREIndex = dataSourceDLG.SIB1COREIndex;
                config.SIB1SSIndex = dataSourceDLG.SIB1SSIndex;
                config.SIB1BWPIndex = dataSourceDLG.SIB1BWPIndex;
            end
        end

        function collapseSIB1Panel(obj,tf)
            arguments
                obj
                tf (1,1) logical
            end
            obj.Panel.Collapsed = tf;
        end
    end %methods public

    % Custom getters/setters
    methods
        function out = get.ChannelName(~)
            out = 'SSB';
        end
    end

end %classdef

%% Local functions
function updateSib1(obj)
    ssbDataSourceClassName = 'wirelessWaveformGenerator.nr5G_SSB_DataSource';
    if isKey(obj.Parent.WaveformGenerator.pParameters.DialogsMap,ssbDataSourceClassName)
        dataSourceDLG = obj.Parent.WaveformGenerator.pParameters.DialogsMap(ssbDataSourceClassName);
        updateSIB1Config(dataSourceDLG);
    end
end