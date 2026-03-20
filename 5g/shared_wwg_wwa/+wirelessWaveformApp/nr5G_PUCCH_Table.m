classdef nr5G_PUCCH_Table < wirelessWaveformApp.nr5G_Table
    % Creation and handling of the PUCCH table in the PUCCH tab of Uplink 5G

    %   Copyright 2021-2024 The MathWorks, Inc.

    properties
        Format (:,1) double % PUCCH format of each row, saved as a column vector
    end

    properties (Access = private)
        defaultFormat (:,1) double % Default PUCCH format, based on the default configuration passed in at construction time
    end

    % Constructor and public interface methods
    methods (Access = public)
        function obj = nr5G_PUCCH_Table(parent, defaultConfig, columnsToHide)

            arguments
                % uiobject that contains the table
                parent (1,1)
                % Cell array containing the app default nrWavegenSRSConfig
                % configuration object(s)
                defaultConfig cell
                % List of the names of the table columns to remove during
                % construction, if any (optional)
                columnsToHide (1,:) string = string.empty;
            end

            % Call base-class constructor
            obj@wirelessWaveformApp.nr5G_Table(parent, defaultConfig, ...
                                               Name = getString(message("nr5g:waveformApp:PUCCHTableName")), ...
                                               Tag = "PUCCH", ...
                                               PropNamesToLink = "BandwidthPartID", ...
                                               TableColumnMap = getTableColumnMap(), ...
                                               TableVisuals = getTableVisuals(), ...
                                               HasTitle = true, ...
                                               ColumnsToHide = columnsToHide);

            % Initialize value of cached Format and DefaultFormat properties
            obj.Format = cellfun(@(x)double(extract(string(class(x)),digitsPattern)), defaultConfig);
            obj.defaultFormat = obj.Format;
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

            if nvargs.AllowUIChange
                % If the configuration to apply is meant to reset the
                % table, reset also the cached Format to the default
                obj.Format = obj.defaultFormat;
            end

            % Call base-class method
            applyConfiguration@wirelessWaveformApp.nr5G_Table(obj, chWaveCfg, AllowUIChange=nvargs.AllowUIChange, IDs=nvargs.IDs, ConfigIDs=nvargs.ConfigIDs);
        end

        %% Update internal properties
        function updatePropertyValues(obj, nvargs)
            % Update the available values of the given internal property as
            % options in the dropdown of the table.
            % Supported properties are:
            % * BandwidthPartID
            % * Modulation

            arguments
                obj
                % Name of the internal property to update
                nvargs.PropertyName (1,1) string {mustBeMember(nvargs.PropertyName, ...
                    ["BandwidthPartID", "Modulation"])}
                % New list of allowed values
                nvargs.NewList
            end

            propName = nvargs.PropertyName;
            newList = nvargs.NewList;
            switch propName
                case "Modulation"
                    % Update the Modulation read-only cell in the table
                    modColIdx = (getTableColumnName(obj) == propName);
                    selection = wirelessWaveformApp.internal.Utility.getSingleSelection(obj.Selection);
                    obj.Table.Data{selection, modColIdx} = newList;
                otherwise
                    % updateDropdownOptions automatically checks propName against
                    % the list of properties to link to defined during construction
                    updateDropdownOptions(obj, propName, newList);
            end
        end
    end

    methods (Access = protected)
        function addTableEntry(obj, ~, ~)
            % Executes when the Add button is clicked

            % Call baseclass method
            addTableEntry@wirelessWaveformApp.nr5G_Table(obj);

            % Update the internal cached Format
            obj.Format(end+1) = obj.defaultFormat(1); % By default, the add button adds a new PUCCH with the default format
        end

        function noOp = removeTableEntry(obj, cfgID, ~)
            % Executes when the Remove button is clicked. Remove row and
            % possibly shift contents up. Also handle selections.

            % Call baseclass method
            [noOp, selectedRows] = removeTableEntry@wirelessWaveformApp.nr5G_Table(obj, cfgID);

            if ~noOp
                % Remove deleted format from internal cache
                obj.Format(selectedRows) = [];
            end
        end

        function duplicateTableEntry(obj, ~, ~)
            % Executes when the Duplicate button is clicked

            selectedRows = obj.Selection(:);
            numNewEntries = numel(selectedRows);
            duplicatedRows = size(obj.Table.Data, 1) + (1:numNewEntries);

            % Call baseclass method
            duplicateTableEntry@wirelessWaveformApp.nr5G_Table(obj);

            % Copy the cached format to the internal cache of the new
            % instance(s)
            obj.Format(duplicatedRows) = obj.Format(selectedRows);
        end

        function updateEditability(obj, data)
            % Update the editability of the cells for this table

            % Call base class method that resets all cells to the default
            % state
            updateEditability@wirelessWaveformApp.nr5G_Table(obj, data);

            tableColName = getTableColumnName(obj);

            % PRBSet is read-only when Interlacing is true
            PRBSetColIdx = strcmp(tableColName, 'PRBSet');
            editableRows = ~strcmp(data(:, PRBSetColIdx), getString(message('nr5g:waveformApp:Interlaced')));
            obj.CellEditable( editableRows, PRBSetColIdx) = true;
            obj.CellEditable(~editableRows, PRBSetColIdx) = false;

            % NID, RNTI, and Coding are read-only for formats 0 and 1
            editableRows = ~any(obj.Format==[0 1], 2);
            colIdx = any(tableColName == ["NID", "RNTI", "Coding"], 2);
            obj.CellEditable( editableRows, colIdx) = true;
            obj.CellEditable(~editableRows, colIdx) = false;
        end

        function [data, chWaveCfg] = mapTable2ConfigSpecial(obj, data, chWaveCfg)
            % Modulation update from the table, which needs to be handled
            % separately.
            modColIdx    = (getTableColumnName(obj) == "Modulation");
            for idx = 1:size(data, 1)
                % Update Modulation
                switch obj.Format(idx)
                    case 0
                        data{idx, modColIdx} = 'Z-Chu';
                    case 1
                        if chWaveCfg{idx}.NumUCIBits==2
                            data{idx, modColIdx} = 'QPSK';
                        else
                            data{idx, modColIdx} = 'BPSK';
                        end
                    case 2
                        data{idx, modColIdx} = 'QPSK';
                    otherwise % case {3, 4}
                        data{idx, modColIdx} = chWaveCfg{idx}.Modulation;
                end
            end
        end

        function data = mapConfig2TableSpecial(obj, data, chWaveCfg)
            % Format, Modulation, NID, RNTI, and Coding updates from the
            % object, which need to be handled separately.
            tableColName = getTableColumnName(obj);
            formatColIdx = (tableColName == "Format");
            modColIdx    = (tableColName == "Modulation");
            NIDColIdx    = (tableColName == "NID");
            RNTIColIdx   = (tableColName == "RNTI");
            CodingColIdx = (tableColName == "Coding");
            for idx = 1:numel(chWaveCfg)
                % Update Format
                format = double(extract(string(class(chWaveCfg{idx})),digitsPattern));
                data{idx, formatColIdx} = ['Format ' num2str(format)]; % Update the displayed value in the dropdown
                obj.Format(idx) = format; % Update the cached value of Format for this row

                % Update Modulation
                switch format
                    case 0
                        data{idx, modColIdx} = 'Z-Chu';
                    case 1
                        if chWaveCfg{idx}.NumUCIBits==2
                            data{idx, modColIdx} = 'QPSK';
                        else
                            data{idx, modColIdx} = 'BPSK';
                        end
                    case 2
                        data{idx, modColIdx} = 'QPSK';
                    otherwise % case {3, 4}
                        data{idx, modColIdx} = chWaveCfg{idx}.Modulation;
                end

                % Update NID, RNTI, and Coding
                if any(format == [2, 3, 4])
                    data{idx, NIDColIdx} = mat2str(chWaveCfg{idx}.NID);
                    data{idx, RNTIColIdx} = chWaveCfg{idx}.RNTI;
                    data{idx, CodingColIdx} = chWaveCfg{idx}.Coding;
                else
                    data{idx, NIDColIdx} = 'N/A';
                    data{idx, RNTIColIdx} = 'N/A';
                    data{idx, CodingColIdx} = false;
                end
            end
        end
    end

    %% Table <--> Object Mapping Functions
    methods (Access = private)
        % Format
        function [tableRow, chWaveCfg] = tab2ObjMapFormat(obj, tableRow, colIdx, chWaveCfg)
            pucchIdx = (obj.AllIDs == tableRow{1}); % First column is always the channel ID
            % Update the cached format for this row
            format = double(extract(string(tableRow{colIdx}), digitsPattern));
            formatIn = double(extract(string(class(chWaveCfg)), digitsPattern));
            if format~=obj.Format(pucchIdx) || format~=formatIn % Format has changed
                % Update the wavegen config object to the new format,
                % keeping the enable state and label unmodified
                enable = chWaveCfg.Enable;
                label = chWaveCfg.Label;
                chWaveCfg = wirelessWaveformApp.nr5G_PUCCH_Tab.getDefaultConfigObject(format);
                chWaveCfg.Enable = enable;
                chWaveCfg.Label = label;

                % Update the internal cache
                obj.Format(pucchIdx) = format;
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
            if isprop(chWaveCfg, 'Interlacing') && chWaveCfg.Interlacing
                tableVal = getString(message('nr5g:waveformApp:Interlaced'));
            else
                tableVal = mat2strFcn(obj, propVal);
            end
        end

        % NID
        function [tableRow, chWaveCfg] = tab2ObjMapNID(obj, tableRow, colIdx, chWaveCfg)
            % NID only applies to PUCCH formats 2, 3, and 4
            thisRow = (obj.AllIDs == tableRow{1}); % First column is always the channel ID
            if any(obj.Format(thisRow) == [2, 3, 4])
                if strcmpi(tableRow{colIdx}, 'N/A')
                    % Format 2, 3, or 4 was just chosen
                    tableRow{colIdx} = mat2str(chWaveCfg.NID);
                else
                    [tableRow, chWaveCfg] = evalFcn(obj, tableRow, colIdx, chWaveCfg);
                end
            else
                tableRow{colIdx} = 'N/A'; % NID not applicable
            end
        end

        % RNTI
        function [tableRow, chWaveCfg] = tab2ObjMapRNTI(obj, tableRow, colIdx, chWaveCfg)
            % RNTI only applies to PUCCH formats 2, 3, and 4
            thisRow = (obj.AllIDs == tableRow{1}); % First column is always the channel ID
            if any(obj.Format(thisRow) == [2, 3, 4])
                if strcmpi(tableRow{colIdx}, 'N/A')
                    % Format 2, 3, or 4 was just chosen
                    tableRow{colIdx} = chWaveCfg.RNTI;
                else
                    [tableRow, chWaveCfg] = tab2ObjDirectMap(obj, tableRow, colIdx, chWaveCfg);
                end
            else
                tableRow{colIdx} = 'N/A'; % RNTI not applicable
            end
        end

        % Coding
        function [tableRow, chWaveCfg] = tab2ObjMapCoding(obj, tableRow, colIdx, chWaveCfg)
            % Coding only applies to PUCCH formats 2, 3, and 4
            thisRow = (obj.AllIDs == tableRow{1}); % First column is always the channel ID
            if any(obj.Format(thisRow) == [2, 3, 4])
                if ~obj.CellEditable(thisRow, colIdx)
                    % Format 2, 3, or 4 was just chosen
                    tableRow{colIdx} = chWaveCfg.Coding;
                else
                    [tableRow, chWaveCfg] = tab2ObjDirectMap(obj, tableRow, colIdx, chWaveCfg);
                end
            else
                tableRow{colIdx} = false; % Coding not applicable
            end
        end
    end

    methods (Static, Access = private)
        function tableData = getTableStaticData()
            % Provides a table containing all static data associated with
            % this class
            formatOptions = {'Format 0', 'Format 1', 'Format 2', 'Format 3', 'Format 4'}; % Format dropdown options
            tableData = table(Size=[14, 7], VariableTypes={'string','string','cell','cell','logical','cell','cell'}, VariableNames={'TableColumnName','ObjectPropName','Table2ObjMap','Obj2TableMap','CellEditable','ColumnFormat','ColumnWidth'});
            %                  TableColumnName   ObjectPropName     Table2ObjMap                   Obj2TableMap               CellEditable  ColumnFormat             ColumnWidth 
            tableData(1,:)  = {"",               "",                {},                            {},                        false,        {[]},                    {"fit"}};  % ID column - Read only and no mapping
            tableData(2,:)  = {"Label",          "Label",           {@tab2ObjDirectMap},           {@obj2TabDirectMap},       true,         {'char'},                {"1x"}};
            tableData(3,:)  = {"Enable",         "Enable",          {@tab2ObjDirectMap},           {@obj2TabDirectMap},       true,         {'logical'},             {"1x"}};
            tableData(4,:)  = {"Format",         "",                {@tab2ObjMapFormat},           {"special"},               true,         {formatOptions},         {"1x"}};
            tableData(5,:)  = {"Power",          "Power",           {@tab2ObjDirectMap},           {@obj2TabDirectMap},       true,         {[]},                    {"1x"}};
            tableData(6,:)  = {"BWPID",          "BandwidthPartID", {@str2doubleFcn},              {@num2strFcn},             true,         {{'1'}},                 {"1x"}};
            tableData(7,:)  = {"Modulation",     "Modulation",      {},                            {"special"},               false,        {'char'},                {"1x"}};
            tableData(8,:)  = {"StartSymbol",    "SymbolAllocation",{@tab2ObjMapSymbolAllocation}, {@obj2TabMapStartSymbol},  true,         {cellstr(string(0:13))}, {'1x'}};
            tableData(9,:)  = {"SymbolLength",   "SymbolAllocation",{@tab2ObjMapSymbolAllocation}, {@obj2TabMapSymbolLength}, true,         {cellstr(string(1:14))}, {'1x'}};
            tableData(10,:) = {"SlotAllocation", "SlotAllocation",  {@evalFcn},                    {@mat2strFcn},             true,         {'char'},                {"1x"}};
            tableData(11,:) = {"Period",         "Period",          {@evalFcn},                    {@mat2strFcn},             true,         {'char'},                {"1x"}};
            tableData(12,:) = {"PRBSet",         "PRBSet",          {@tab2ObjMapPRBSet},           {@obj2TabMapPRBSet},       true,         {'char'},                {'1x'}};
            tableData(13,:) = {"NID",            "NID",             {@tab2ObjMapNID},              {"special"},               false,        {'char'},                {'1x'}};
            tableData(14,:) = {"RNTI",           "RNTI",            {@tab2ObjMapRNTI},             {"special"},               false,        {[]},                    {'1x'}};
            tableData(15,:) = {"Coding",         "Coding",          {@tab2ObjMapCoding},           {"special"},               false,        {'logical'},             {'1x'}};
        end
    end
end

function tableMap = getTableColumnMap()
    % Provides the table column names, the equivalent properties of
    % configuration object, and the mapping rule between the two.
    tableMap = removevars(wirelessWaveformApp.nr5G_PUCCH_Table.getTableStaticData(), {'CellEditable','ColumnFormat','ColumnWidth'});
end

function tableViz = getTableVisuals()
    % Provides the table column names, together with their column format,
    % column width, and whether the default column is editable or not.
    tableViz = removevars(wirelessWaveformApp.nr5G_PUCCH_Table.getTableStaticData(), {'ObjectPropName','Table2ObjMap','Obj2TableMap'});
end