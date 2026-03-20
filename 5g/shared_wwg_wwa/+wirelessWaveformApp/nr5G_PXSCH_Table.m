classdef nr5G_PXSCH_Table < wirelessWaveformApp.nr5G_Table
    % Common class for the creation and handling of the PDSCH/PUSCH table
    % in the PDSCH/PUSCH tab of Downlink/Uplink 5G

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Access = private)
        pTargetCodeRate (:,2) double % Target code rate for each row, saved as a two-column array
    end

    % Constructor and public interface methods
    methods (Access = public)
        function obj = nr5G_PXSCH_Table(parent, defaultConfig, columnsToHide, linkConfig)

            arguments
                % uiobject that contains the table
                parent (1,1)
                % Cell array containing the app default nrWavegenPDCCHConfig
                % configuration object(s)
                defaultConfig cell
                % List of the names of the table columns to remove during
                % construction, if any (optional)
                columnsToHide (1,:) string = string.empty;
                % Table name
                linkConfig.Name (1,1) string
                % List of modulation options to hide from the full list.
                % Since the list is a union of the available modulation
                % orders for both PDSCH and PUSCH, each channel needs to
                % use only a subset of them.
                linkConfig.ModulationOptionsToHide (1,:) string
            end

            % Call base-class constructor
            obj@wirelessWaveformApp.nr5G_Table(parent, defaultConfig, ...
                                               Name = linkConfig.Name, ...
                                               Tag = "PXSCH", ...
                                               PropNamesToLink = "BandwidthPartID", ...
                                               TableColumnMap = getTableColumnMap(), ...
                                               TableVisuals = getTableVisuals(linkConfig.ModulationOptionsToHide), ...
                                               HasTitle = true, ...
                                               ColumnsToHide = columnsToHide);

            % Initialize value of cached pTargetCodeRate property
            obj.pTargetCodeRate = repmat(defaultConfig{1}.TargetCodeRate, numel(defaultConfig), 2);
        end

        function applyConfiguration(obj, chWaveCfg, nvargs)
            % Map the channel configuration object to the table.
            % This happens when starting a new session, loading an existing
            % session, or using openInGenerator

            arguments
                % Mandatory inputs
                obj % This nr5G_Table object
                chWaveCfg % The wavegen channel to apply to the table

                % Name-Value arguments
                % AllowUIChange is used to update what the user sees on
                % the table programmatically. Set it to false if you don't
                % want to wipe out the visual features of the table, like
                % the selected row and the row IDs.
                % In these cases AllowUIChange is considered true
                % irrespective of the user input:
                % 1. numel(nvargs.IDs) < numel(chWaveCfg)
                % 2. isempty(nvargs.IDs) && (numel(obj.AllIDs)<numel(chWaveCfg))
                nvargs.AllowUIChange (1,1) logical = true;

                % List of row IDs. If AllowUIChange is false, the value of
                % IDs is used to specify the desired IDs of the rows. If
                % IDs is empty, the existing list given by AllIDs is used.
                % In these cases AllowUIChange is considered true
                % irrespective of the user input:
                % 1. numel(nvargs.IDs) < numel(chWaveCfg)
                % 2. isempty(nvargs.IDs) && (numel(obj.AllIDs)<numel(chWaveCfg))
                nvargs.IDs (1,:) uint8 = [];

                % ID of the configuration to apply. If non-empty, the table
                % only applies the given configuration to the specified ID.
                % If the ID is a vector, it must have the same number of
                % elements as the input configuration.
                nvargs.ConfigIDs (1,:) uint8 = [];
            end

            N = max(numel(chWaveCfg), numel(obj.AllIDs));
            if size(obj.pTargetCodeRate,1)~=N
                % Ensure the cached pTargetCodeRate has the right dimension.
                % Only update the cached value if the dimension needs changing
                obj.pTargetCodeRate = repmat(obj.pTargetCodeRate(1,:), N, 1);
            end

            % Call baseclass method
            applyConfiguration@wirelessWaveformApp.nr5G_Table(obj, chWaveCfg, AllowUIChange=nvargs.AllowUIChange, IDs=nvargs.IDs, ConfigIDs=nvargs.ConfigIDs);
        end

        %% Update internal properties
        function updatePropertyValues(obj, nvargs)
            % Update the available values of the PROPNAME property as
            % options in the dropdown of the table.
            % Supported properties are:
            % * BandwidthPartID

            arguments
                obj
                % Name of the internal property to update
                nvargs.PropertyName (1,1) string {mustBeMember(nvargs.PropertyName, "BandwidthPartID")}
                % New list of allowed values
                nvargs.NewList
            end

            % updateDropdownOptions automatically checks propName against
            % the list of properties to link to defined during construction
            updateDropdownOptions(obj, nvargs.PropertyName, nvargs.NewList);
        end
    end

    methods (Access = protected)
        function addTableEntry(obj, ~, ~)
            % Executes when the Add button is clicked

            % Call baseclass method
            addTableEntry@wirelessWaveformApp.nr5G_Table(obj);

            % Update the internal cached TargetCodeRate
            obj.pTargetCodeRate(end+1,:) = [obj.DefaultConfig{1}.TargetCodeRate obj.DefaultConfig{1}.TargetCodeRate];
        end

        function noOp = removeTableEntry(obj, cfgID, ~)
            % Executes when the Remove button is clicked. Remove row and
            % possibly shift contents up. Also handle selections.

            % Call baseclass method
            [noOp, selectedRows] = removeTableEntry@wirelessWaveformApp.nr5G_Table(obj, cfgID);

            if ~noOp
                % Remove deleted target code rate from internal cache
                obj.pTargetCodeRate(selectedRows,:) = [];
            end
        end

        function duplicateTableEntry(obj, ~, ~)
            % Executes when the Duplicate button is clicked

            selectedRows = obj.Selection(:);
            numNewEntries = numel(selectedRows);
            duplicatedRows = size(obj.Table.Data, 1) + (1:numNewEntries);

            % Call baseclass method
            duplicateTableEntry@wirelessWaveformApp.nr5G_Table(obj);

            % Copy the cached Target code rate to the internal cache of the
            % new instance(s)
            obj.pTargetCodeRate(duplicatedRows,:) = obj.pTargetCodeRate(selectedRows,:);
        end

        function updateEditability(obj, data)
            % Update the editability of the cells for this table

            % Call base class method that resets all cells to the default
            % state
            updateEditability@wirelessWaveformApp.nr5G_Table(obj, data);

            tableColName = getTableColumnName(obj);

            coding = cellfun(@(x)(x), data(:, strcmp(tableColName, 'Coding'))); % coding true | false
            twoCW = cellfun(@(x)(str2double(x) > 4), data(:, strcmp(tableColName, 'NumLayers'))); % numLayers > 4

            % TargetCodeRate is read-only when Coding is false
            TargetCodeRateColIdx = strcmp(tableColName, 'TargetCodeRate');
            editableRows = coding;
            obj.CellEditable( editableRows, TargetCodeRateColIdx) = true;
            obj.CellEditable(~editableRows, TargetCodeRateColIdx) = false;

            % ModulationCW2 is read-only when NumLayers <= 4
            ModulationCW2ColIdx = strcmp(tableColName, 'ModulationCW2');
            editableRows = twoCW;
            obj.CellEditable( editableRows, ModulationCW2ColIdx) = true;
            obj.CellEditable(~editableRows, ModulationCW2ColIdx) = false;

            % TargetCodeRateCW2 is read-only when Coding is false AND
            % NumLayers <= 4
            TargetCodeRateCW2ColIdx = strcmp(tableColName, 'TargetCodeRateCW2');
            editableRows = twoCW & coding;
            obj.CellEditable( editableRows, TargetCodeRateCW2ColIdx) = true;
            obj.CellEditable(~editableRows, TargetCodeRateCW2ColIdx) = false;
        end
    end

    %% Table <--> Object Mapping Functions
    methods (Access = private)
        % Modulation (CW 1)
        function [tableRow, chWaveCfg] = tab2ObjMapMod(obj, tableRow, colIdx, chWaveCfg)
            tableColName = getTableColumnName(obj);
            modulation = tableRow{colIdx};
            ModulationCW2ColIdx = strcmp(tableColName, 'ModulationCW2');
            modulationCW2 = tableRow{ModulationCW2ColIdx};
            if strcmp(modulationCW2,'N/A')
                chWaveCfg.Modulation = modulation;
            else
                chWaveCfg.Modulation = {modulation, modulationCW2};
            end
        end
        function tableVal = obj2TabMapMod(~, propVal, ~, ~)
            if iscell(propVal)
                tableVal = propVal{1};
            else
                tableVal = propVal;
            end
        end

        % Modulation (CW 2)
        function [tableRow, chWaveCfg] = tab2ObjMapModCW2(obj, tableRow, colIdx, chWaveCfg)
            tableColName = getTableColumnName(obj);
            NumLayersColIdx = strcmp(tableColName, 'NumLayers');
            oneCW = str2double(tableRow{NumLayersColIdx}) <= 4;
            if oneCW
                tableRow{colIdx} = 'N/A'; % 2nd CW modulation not applicable
            else
                ModulationColIdx = strcmp(tableColName, 'Modulation');
                modulationCW1 = tableRow{ModulationColIdx};
                modulationCW2 = tableRow{colIdx};
                if strcmp(modulationCW2, 'N/A') % 2 CW was just enabled
                    tableRow{colIdx} = modulationCW1;
                else
                    chWaveCfg.Modulation = {modulationCW1, modulationCW2};
                end
            end
        end
        function tableVal = obj2TabMapModCW2(~, propVal, chWaveCfg, ~)
            oneCW = chWaveCfg.NumLayers <= 4;
            if oneCW
                tableVal = 'N/A';
            else
                if iscell(propVal)
                    tableVal = propVal{2};
                else
                    tableVal = propVal;
                end
            end
        end

        % StartSymbol
        function [tableRow, chWaveCfg] = tab2ObjMapSymbolAllocation(obj, tableRow, ~, chWaveCfg)
            tableColName = getTableColumnName(obj);
            StartSymbolColIdx = strcmp(tableColName, 'StartSymbol');
            SymbolLengthColIdx = strcmp(tableColName, 'SymbolLength');
            startSymbol = str2double(tableRow{StartSymbolColIdx});
            symbolLength = str2double(tableRow(SymbolLengthColIdx));
            chWaveCfg = validateMultiColumnProp(obj, chWaveCfg, "SymbolAllocation", [startSymbol symbolLength]);
        end
        function tableVal = obj2TabMapStartSymbol(~, propVal, ~, ~)
            tableVal = num2str(propVal(1));
        end

        % SymbolLength
        function tableVal = obj2TabMapSymbolLength(~, propVal, ~, ~)
            tableVal = num2str(propVal(2));
        end

        % PRBSet
        function [tableRow, chWaveCfg] = tab2ObjMapPRBSet(obj, tableRow, colIdx, chWaveCfg)
            % Only parse PRBSet when not interlaced
            if ~strcmpi(tableRow{colIdx}, getString(message('nr5g:waveformApp:Interlaced')))
                [~, chWaveCfg] = evalFcn(obj, tableRow, colIdx, chWaveCfg);
            end
        end
        function tableVal = obj2TabMapPRBSet(obj, propVal, chWaveCfg, ~)
            if chWaveCfg.Interlacing
                tableVal = getString(message('nr5g:waveformApp:Interlaced'));
            else
                tableVal = mat2strFcn(obj, propVal);
            end
        end

        % Coding
        function [tableRow, chWaveCfg] = tab2ObjMapCoding(~, tableRow, colIdx, chWaveCfg)
            if ~isempty(tableRow{colIdx})
                chWaveCfg.Coding = tableRow{colIdx};
            else
                % In rare occasions, the Coding cell could be empty. This
                % code prevents any hard error and restores the correct app
                % status when the user clicks on the Coding checkbox.
                tableRow{colIdx} = chWaveCfg.Coding;
            end
        end

        % TargetCodeRate (CW 1)
        function [tableRow, chWaveCfg] = tab2ObjMapTCR(obj, tableRow, colIdx, chWaveCfg)
            tableColName = getTableColumnName(obj);
            if ~chWaveCfg.Coding
                tableRow{colIdx} = 'N/A'; % code rate not applicable
            else
                codeRateCW1 = tableRow{colIdx};
                CodeRateCW2ColIdx = strcmp(tableColName, 'TargetCodeRateCW2');
                codeRateCW2 = tableRow{CodeRateCW2ColIdx};
                idx = (tableRow{1} == obj.AllIDs);
                if strcmp(codeRateCW1, 'N/A') % Coding was just enabled
                    cache = obj.pTargetCodeRate(idx,1);
                    % Use previously cached value for target code rate
                    tableRow{colIdx} = cache;
                else
                    if strcmp(codeRateCW2,'N/A') || chWaveCfg.NumCodewords==1
                        chWaveCfg.TargetCodeRate = codeRateCW1;
                        obj.pTargetCodeRate(idx,:) = [codeRateCW1 codeRateCW1]; % Cache the current value of target code rate
                    else
                        chWaveCfg.TargetCodeRate = [codeRateCW1 codeRateCW2];
                        obj.pTargetCodeRate(idx,:) = [codeRateCW1 codeRateCW2]; % Cache the current value of target code rate
                    end
                end
            end
        end
        function tableVal = obj2TabMapTCR(~, propVal, chWaveCfg, ~)
            if ~chWaveCfg.Coding
                tableVal = 'N/A'; % code rate not applicable
            else
                tableVal = propVal(1); % first codeword value
            end
        end

        % TargetCodeRate (CW 2)
        function [tableRow, chWaveCfg] = tab2ObjMapTCRCW2(obj, tableRow, colIdx, chWaveCfg)
            tableColName = getTableColumnName(obj);
            NumLayersColIdx = strcmp(tableColName, 'NumLayers');
            oneCW = str2double(tableRow{NumLayersColIdx}) <= 4;
            if oneCW || ~chWaveCfg.Coding
                tableRow{colIdx} = 'N/A'; % 2nd CW target code rate not applicable
            else
                CodeRateColIdx = strcmp(tableColName, 'TargetCodeRate');
                codeRateCW1 = tableRow{CodeRateColIdx};
                codeRateCW2 = tableRow{colIdx};
                idx = (tableRow{1} == obj.AllIDs);
                if strcmp(codeRateCW2, 'N/A') % Coding was just enabled
                    cache = obj.pTargetCodeRate(idx,2);
                    tableRow{colIdx} = cache; % Use previously cached value for target code rate
                else
                    chWaveCfg.TargetCodeRate = [codeRateCW1 codeRateCW2];
                    % The new value of the target code rate is valid. Update
                    % the cache
                    obj.pTargetCodeRate(idx,:) = [codeRateCW1 codeRateCW2]; % Cache the current value of target code rate
                end
            end
        end
        function tableVal = obj2TabMapTCRCW2(~, propVal, chWaveCfg, ~)
            coding = chWaveCfg.Coding;
            oneCW = chWaveCfg.NumLayers <= 4;
            disabled = ~coding || oneCW;
            if disabled
                tableVal = 'N/A'; % 2nd CW code rate not applicable
            else
                if ~isscalar(propVal)
                    tableVal = propVal(2);
                else
                    tableVal = propVal;
                end
            end
        end

        % Transform precoding
        function [tableRow, chWaveCfg] = tab2ObjMapTP(obj, tableRow, colIdx, chWaveCfg)
            tableColName = getTableColumnName(obj);
            chWaveCfg.TransformPrecoding = tableRow{colIdx};
            ModulationCW2ColIdx = strcmp(tableColName, 'ModulationCW2');
            TargetCodeRateCW2ColIdx = strcmp(tableColName, 'TargetCodeRateCW2');
            if chWaveCfg.TransformPrecoding
                % Fix NumLayers to 1
                NumLayersColIdx = strcmp(tableColName, 'NumLayers');
                tableRow{NumLayersColIdx} = '1'; % char because it is a dropdown
                % CW 2 columsn are not applicable for 1 layer
                tableRow{ModulationCW2ColIdx} = 'N/A';
                tableRow{TargetCodeRateCW2ColIdx} = 'N/A';
            else
                % Can't allow 'pi/2-BPSK' when transform precoding is off
                ModulationColIdx = strcmp(tableColName, 'Modulation');
                if iscell(chWaveCfg.Modulation) % If there are two codewords 
                    if strcmpi(chWaveCfg.Modulation{1}, 'pi/2-BPSK')
                        tableRow{ModulationColIdx} = 'QPSK';
                    end
  
                    if strcmpi(chWaveCfg.Modulation{2}, 'pi/2-BPSK')
                        tableRow{ModulationCW2ColIdx} = 'QPSK';
                    end
                else
                    if strcmpi(chWaveCfg.Modulation, 'pi/2-BPSK')
                        tableRow{ModulationColIdx} = 'QPSK';
                    end
                end
            end
        end
    end

    methods (Static, Access = private)
        function tableData = getTableStaticData()
            % Provides a table containing all static data associated with
            % this class
            modOptions = {'pi/2-BPSK','QPSK','16QAM','64QAM','256QAM','1024QAM'};
            tableData = table(Size=[19, 7], VariableTypes={'string','string','cell','cell','logical','cell','cell'}, VariableNames={'TableColumnName','ObjectPropName','Table2ObjMap','Obj2TableMap','CellEditable','ColumnFormat','ColumnWidth'});
            %                  TableColumnName      ObjectPropName       Table2ObjMap                   Obj2TableMap               CellEditable  ColumnFormat             ColumnWidth 
            tableData(1,:)  = {"",                  "",                  {},                            {},                        false,         {[]},                    {'fit'}};  % ID column - Read only and no mapping
            tableData(2,:)  = {"Enable",            "Enable",            {@tab2ObjDirectMap},           {@obj2TabDirectMap},       true,          {'logical'},             {'fit'}};
            tableData(3,:)  = {"Power",             "Power",             {@evalFcn},                    {@mat2strFcn},             true,          {'char'},                {'1x'}};
            tableData(4,:)  = {"BWPID",             "BandwidthPartID",   {@str2doubleFcn},              {@num2strFcn},             true,          {{'1'}},                 {'1x'}};
            tableData(5,:)  = {"Modulation",        "Modulation",        {@tab2ObjMapMod},              {@obj2TabMapMod},          true,          {modOptions},            {'fit'}};
            tableData(6,:)  = {"ModulationCW2",     "Modulation",        {@tab2ObjMapModCW2},           {@obj2TabMapModCW2},       false,         {modOptions},            {'fit'}};
            tableData(7,:)  = {"NumLayers",         "NumLayers",         {@str2doubleFcn},              {@num2strFcn},             true,          {cellstr(string(1:8))},  {'fit'}};
            tableData(8,:)  = {"MappingType",       "MappingType",       {@tab2ObjDirectMap},           {@obj2TabDirectMap},       true,          {{'A', 'B'}},            {'fit'}};
            tableData(9,:)  = {"StartSymbol",       "SymbolAllocation",  {@tab2ObjMapSymbolAllocation}, {@obj2TabMapStartSymbol},  true,          {cellstr(string(0:13))}, {'fit'}};
            tableData(10,:) = {"SymbolLength",      "SymbolAllocation",  {@tab2ObjMapSymbolAllocation}, {@obj2TabMapSymbolLength}, true,          {cellstr(string(1:14))}, {'fit'}};
            tableData(11,:) = {"SlotAllocation",    "SlotAllocation",    {@evalFcn},                    {@mat2strFcn},             true,          {'char'},                {'1x'}};
            tableData(12,:) = {"Period",            "Period",            {@evalFcn},                    {@mat2strFcn},             true,          {'char'},                {'1x'}};
            tableData(13,:) = {"PRBSet",            "PRBSet",            {@tab2ObjMapPRBSet},           {@obj2TabMapPRBSet},       true,          {'char'},                {'1x'}};
            tableData(14,:) = {"VRBSet",            "PRBSet",            {@evalFcn},                    {@mat2strFcn},             true,          {'char'},                {'1x'}};
            tableData(15,:) = {"NID",               "NID",               {@evalFcn},                    {@mat2strFcn},             true,          {'char'},                {'1x'}};
            tableData(16,:) = {"RNTI",              "RNTI",              {@tab2ObjDirectMap},           {@obj2TabDirectMap},       true,          {[]},                    {'1x'}};
            tableData(17,:) = {"Coding",            "Coding",            {@tab2ObjMapCoding},           {@obj2TabDirectMap},       true,          {'logical'},             {'fit'}};
            tableData(18,:) = {"TargetCodeRate",    "TargetCodeRate",    {@tab2ObjMapTCR},              {@obj2TabMapTCR},          true,          {[]},                    {'1x'}};
            tableData(19,:) = {"TargetCodeRateCW2", "TargetCodeRate",    {@tab2ObjMapTCRCW2},           {@obj2TabMapTCRCW2},       false,         {[]},                    {'1x'}};
            tableData(20,:) = {"TransformPrecoding","TransformPrecoding",{@tab2ObjMapTP},               {@obj2TabDirectMap},       true,          {'logical'},             {'fit'}};
            tableData(21,:) = {"EnablePTRS",        "EnablePTRS",        {@tab2ObjDirectMap},           {@obj2TabDirectMap},       true,          {'logical'},             {'fit'}};
        end
    end
end

function tableMap = getTableColumnMap()
    % Provides the table column names, the equivalent properties of
    % configuration object, and the mapping rule between the two.
    tableMap = removevars(wirelessWaveformApp.nr5G_PXSCH_Table.getTableStaticData(), {'CellEditable','ColumnFormat','ColumnWidth'});
end

function tableViz = getTableVisuals(modOptionsToHide)
    % Provides the table column names, together with their column format,
    % column width, and whether the default column is editable or not.
    tableViz = removevars(wirelessWaveformApp.nr5G_PXSCH_Table.getTableStaticData(), {'ObjectPropName','Table2ObjMap','Obj2TableMap'});

    % Update the list of modulations provided
    tableColumnName = tableViz.TableColumnName;
    allMods = tableViz.ColumnFormat{tableColumnName=="Modulation"};
    allMods(cellfun(@(x)strcmp(modOptionsToHide,x),allMods)) = [];
    tableViz.ColumnFormat{tableColumnName=="Modulation"} = allMods;
    tableViz.ColumnFormat{tableColumnName=="ModulationCW2"} = allMods;
end