classdef nr5G_PDCCH_Table < wirelessWaveformApp.nr5G_Table
    % Creation and handling of the PDCCH table in the PDCCH tab of Downlink 5G

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Constant, Access = private)
        defaultCustomDataSource = '[1; 0; 0; 1]';
        defaultCandidate = '1';
    end

    properties (Access = private)
        pCustomDataSource (:,1) cell % Custom data source for each row, saved as a column vector
        pCandidate (:,1) cell % Candidate for each row, saved as a column vector
        cachedNumCandidates = dictionary('1', {[8 8 4 2 1]}); % Internal cache of the NumCandidates property of SearchSpaces to carry out validation
    end

    % Constructor and public interface methods
    methods (Access = public)
        function obj = nr5G_PDCCH_Table(parent, defaultConfig, columnsToHide)

            arguments
                % uiobject that contains the table
                parent (1,1)
                % Cell array containing the app default nrWavegenPDCCHConfig
                % configuration object(s)
                defaultConfig cell
                % List of the names of the table columns to remove during
                % construction, if any (optional)
                columnsToHide (1,:) string = string.empty;
            end

            % Call base-class constructor
            obj@wirelessWaveformApp.nr5G_Table(parent, defaultConfig, ...
                                               Name = getString(message("nr5g:waveformApp:PDCCHTableName")), ...
                                               Tag = "PDCCH", ...
                                               PropNamesToLink = ["BandwidthPartID", "SearchSpaceID"], ...
                                               TableColumnMap = getTableColumnMap(), ...
                                               TableVisuals = getTableVisuals(), ...
                                               HasTitle = true, ...
                                               ColumnsToHide = columnsToHide);

            % Initialize value of cached properties
            obj.pCustomDataSource = repmat({obj.defaultCustomDataSource}, numel(defaultConfig), 1);
            obj.pCandidate = repmat({obj.defaultCandidate}, numel(defaultConfig), 1);

            % Update PDCCH table grid to fit the content
            obj.TableGrid.ColumnWidth{1} = 'fit';
            obj.TableGrid.RowHeight{end} = 'fit';
        end

        %% Update internal properties
        function updatePropertyValues(obj, nvargs)
            % Update the available values of the given internal property.
            % Supported properties are:
            % * BandwidthPartID
            % * SearchSpaceID
            % * NumCandidates

            arguments
                obj
                % Name of the internal property to update
                nvargs.PropertyName (1,1) string {mustBeMember(nvargs.PropertyName, ...
                    ["BandwidthPartID", "SearchSpaceID", "NumCandidates"])}
                % New list of allowed values
                nvargs.NewList
            end

            propName = nvargs.PropertyName;
            newList = nvargs.NewList;
            switch propName
                case 'NumCandidates'
                    ssID = newList{1};
                    NumCandidates = newList{2};
                    updateNumCandidates(obj, ssID, NumCandidates);
                otherwise
                    % updateDropdownOptions automatically checks propName
                    % against the list of properties to link to defined
                    % during construction
                    updateDropdownOptions(obj, propName, newList);
            end
        end
    end

    % Protected methods
    methods (Access = protected)
        function addTableEntry(obj, ~, ~)
            % Executes when the Add button is clicked

            % Call baseclass method
            addTableEntry@wirelessWaveformApp.nr5G_Table(obj);

            % Update the internal cached properties
            obj.pCustomDataSource{end+1} = obj.defaultCustomDataSource;
            obj.pCandidate{end+1} = obj.defaultCandidate;
        end

        function noOp = removeTableEntry(obj, cfgID, ~)
            % Executes when the Remove button is clicked. Remove row and
            % possibly shift contents up. Also handle selections.

            % Call baseclass method
            [noOp, selectedRows] = removeTableEntry@wirelessWaveformApp.nr5G_Table(obj, cfgID);

            if ~noOp
                % Remove deleted rows from internal cache
                obj.pCustomDataSource(selectedRows) = [];
                obj.pCandidate(selectedRows) = [];
            end
        end

        function duplicateTableEntry(obj, ~, ~)
            % Executes when the Duplicate button is clicked

            selectedRows = obj.Selection(:);
            numNewEntries = numel(selectedRows);
            duplicatedRows = size(obj.Table.Data, 1) + (1:numNewEntries);

            % Call baseclass method
            duplicateTableEntry@wirelessWaveformApp.nr5G_Table(obj);

            % Copy the cached properties to the internal cache of the
            % new instance(s)
            obj.pCustomDataSource(duplicatedRows) = obj.pCustomDataSource(selectedRows);
            obj.pCandidate(duplicatedRows) = obj.pCandidate(selectedRows);
        end

        function updateEditability(obj, data)
            % Update the editability of the cells for this table

            % Call base class method that resets all cells to the default
            % state
            updateEditability@wirelessWaveformApp.nr5G_Table(obj, data);

            tableColName = getTableColumnName(obj);

            % Candidate is read-only when CCE Offset is non-empty
            CandidateColIdx = strcmp(tableColName, 'Candidate');
            editableRows = strcmp(data(:, strcmp(tableColName, 'CCEOffset')), '[]');
            obj.CellEditable( editableRows, CandidateColIdx) = true;
            obj.CellEditable(~editableRows, CandidateColIdx) = false;

            % PayloadSize is read-only when Coding is false
            PayloadSizeColIdx = strcmp(tableColName, 'PayloadSize');
            editableRows = cellfun(@(x)(x), data(:, strcmp(tableColName, 'Coding')));
            obj.CellEditable( editableRows, PayloadSizeColIdx) = true;
            obj.CellEditable(~editableRows, PayloadSizeColIdx) = false;

            % CustomDataSource is read-only when DataSource is present and
            % not set to user-defined
            DataSourceColIdx = strcmp(tableColName, 'DataSource');
            if any(DataSourceColIdx)
                CustomDataSourceColIdx = strcmp(tableColName, 'CustomDataSource');
                editableRows = (strcmp(data(:, DataSourceColIdx), 'User-defined'));
                obj.CellEditable( editableRows, CustomDataSourceColIdx) = true;
                obj.CellEditable(~editableRows, CustomDataSourceColIdx) = false;
            end
        end
    end

    methods (Access = private)
        function updateNumCandidates(obj, ssID, NumCandidates)
            % Update the cached values of NumCandidates for each Search
            % Space ID to be used in validation of the PDCCH candidate

            % Update the cached property with the new ones
            obj.cachedNumCandidates = insert(obj.cachedNumCandidates,ssID,NumCandidates,Overwrite=true);

            % Ensure that old ssID that no longer exist are properly
            % deleted from the cache
            k = keys(obj.cachedNumCandidates);
            idx = ~ismember(k,string(ssID));
            obj.cachedNumCandidates = remove(obj.cachedNumCandidates, k(idx));
        end
    end

    %% Table <--> Object Mapping Functions
    methods (Access = private)
        % Candidate
        function [tableRow, chWaveCfg] = tab2ObjMapCandidate(obj, tableRow, colIdx, chWaveCfg)
            % Only process the candidate value if it is not set to N/A
            % (i.e., if CCEOffset is empty) or if CCEOffset is empty, which
            % means that the user just set it to empty.
            idx = (tableRow{1} == obj.AllIDs);
            if isempty(chWaveCfg.CCEOffset)
                tableColName = getTableColumnName(obj);
                CCEOffsetIdx = strcmp(tableColName, 'CCEOffset');
                if strcmp(tableRow{CCEOffsetIdx}, '[]')
                    if strcmp(tableRow{colIdx}, 'N/A') % CCEOffset was just set to empty
                        candidate = obj.pCandidate{idx}; % Use previously cached value
                        tableRow{colIdx} = candidate;
                    else
                        thisSS = num2str(chWaveCfg.SearchSpaceID);
                        aggLevel = chWaveCfg.AggregationLevel;
                        candidate = str2double(tableRow{colIdx});
                        % Get NumCandidates for this aggregation level
                        if isKey(obj.cachedNumCandidates, thisSS)
                            % Only check the candidate if the PDCCH is linked to a
                            % valid Search Space
                            numCandidates = obj.cachedNumCandidates{thisSS};
                            aggLevelIdx = 1+log2(aggLevel);
                            maxCandidate = numCandidates(aggLevelIdx);
                            tableRow{colIdx} = num2str(max(1, min(maxCandidate, candidate)));
                            try
                                validateattributes(candidate, {'double'}, ...
                                    {'scalar', 'integer', 'nonnegative', '<=', maxCandidate}, '', ...
                                    getString(message('nr5g:waveformApp:SSTableValidateCandidate', num2str(aggLevel), num2str(thisSS))));
                            catch e
                                % This is a cross-parameter issue. Modify the error to use
                                % an identifier so that the app infrastructure knows this
                                % is a cross-parameter issue and doesn't revert the value.
                                e2 = MException(obj.CrossParameterErrorID, e.message);
                                throwAsCaller(e2);
                            end
                        end
                        chWaveCfg.AllocatedCandidate = candidate;

                        % The new value of candidate is valid. Update the cache
                        obj.pCandidate{idx} = tableRow{colIdx};
                    end
                else
                    % If CCEOffset is non-empty, set Candidate to N/A
                    tableRow{colIdx} = 'N/A';
                end
            else
                % If CCEOffset is non-empty, set Candidate to N/A
                tableRow{colIdx} = 'N/A';
            end
        end
        function tableVal = obj2TabMapCandidate(obj, propVal, chWaveCfg, ~)
            if isempty(chWaveCfg.CCEOffset)
                tableVal = num2strFcn(obj, propVal);
            else
                % If CCEOffset is non-empty, set Candidate to N/A
                tableVal = 'N/A';
            end
        end

        % PayloadSize
        function [tableRow, chWaveCfg] = tab2ObjMapPayloadSize(obj, tableRow, colIdx, chWaveCfg)
            % Update DataBlockSize with the value in PayloadSize only if
            % this is not 'N/A'
            tableColName = getTableColumnName(obj);
            codingIdx = strcmp(tableColName, 'Coding');
            coding = tableRow{codingIdx};
            if any(codingIdx) && ~coding
                tableRow{colIdx} = 'N/A'; % data blocksize not applicable
            else
                if strcmp(tableRow{colIdx}, 'N/A') % Coding was just enabled
                    tableRow{colIdx} = obj.DefaultConfig{1}.DataBlockSize; % restart DataBlockSize from default
                end
                [tableRow, chWaveCfg] = tab2ObjDirectMap(obj, tableRow, colIdx, chWaveCfg);
            end
        end
        function tableVal = obj2TabMapPayloadSize(~, propVal, chWaveCfg, ~)
            if chWaveCfg.Coding
                tableVal = propVal;
            else
                tableVal = 'N/A';
            end
        end

        % DataSource
        function [tableRow, chWaveCfg] = tab2ObjMapDataSource(obj, tableRow, colIdx, chWaveCfg)
            % Update data source whith DataSource when is not user-defined
            if ~strcmpi(tableRow{colIdx}, 'User-defined')
                [~, chWaveCfg] = tab2ObjDirectMap(obj, tableRow, colIdx, chWaveCfg);
            else
                % If DataSource is set to user-defined, set it to a valid
                % value here and the next pass will update it correctly
                % with the values from CustomDataSource
                chWaveCfg.DataSource = 0;
            end
        end
        function tableVal = obj2TabMapDataSource(~, propVal, ~, ~)
            if ischar(propVal)
                % PN sequence
                tableVal = propVal;
            else
                % Custom data source
                tableVal = 'User-defined';
            end
        end

        % CustomDataSource
        function [tableRow, chWaveCfg] = tab2ObjMapCustomDataSource(obj, tableRow, colIdx, chWaveCfg)
            % Update data source with CustomDataSource in these cases:
            % 1. When DataSource entry is present in the table and
            %    is set to user-defined
            % 2. When DataSource entry is not present in the table and
            %    only CustomDataSource is present
            tableColName = getTableColumnName(obj);
            dataSourceIdx = strcmp(tableColName, 'DataSource');
            if ~any(dataSourceIdx) || strcmpi(tableRow{dataSourceIdx}, 'User-defined')
                if isnumeric(tableRow{colIdx})
                    % Ensure that all complex inputs are automatically modified
                    % to be real
                    tableRow{colIdx} = real(tableRow{colIdx});
                end
                customDataSource = tableRow{colIdx};
                idx = (tableRow{1} == obj.AllIDs);
                if strcmp(customDataSource, 'N/A') % Custom data source was just enabled
                    customDataSource = obj.pCustomDataSource{idx}; % Use previously cached value for custom data source
                    tableRow{colIdx} = customDataSource;
                end
                [~, chWaveCfg] = evalFcn(obj, tableRow, colIdx, chWaveCfg);
                % The new value of the data source is
                % valid. Update the cache
                obj.pCustomDataSource{idx} = customDataSource; % Cache the current value of custom data source
            else
                % DataSource is a PN sequence. Set CustomDataSource to
                % N/A
                tableRow{colIdx} = 'N/A';
            end
        end
        function tableVal = obj2TabMapCustomDataSource(obj, propVal, ~, idx)
            % When CustomDataSource is visible, assign the value
            % following this:
            % If DataSource is not user-defined, set the CustomDataSource
            % value by parsing. If the value is user-defined, set the
            % CustomDataSource value to N/A. Set the read-only status as
            % applicable.
            if ischar(propVal)
                tableVal = 'N/A';
                % Update the cached custom data source for this row
                obj.pCustomDataSource{idx} = obj.defaultCustomDataSource;
            else
                if iscell(propVal)
                    dataSourceString = sprintf('{''%s'',%d}', propVal{1}, propVal{2});
                else
                    dataSourceString = mat2str(propVal);
                end
                tableVal = dataSourceString;
                % Update the cached custom data source for this row
                obj.pCustomDataSource{idx} = dataSourceString;
            end
        end
    end

    methods (Static, Access = private)
        function tableData = getTableStaticData()
            % Provides a table containing all static data associated with
            % this class
            tableData = table(Size=[17, 7], VariableTypes={'string','string','cell','cell','logical','cell','cell'}, VariableNames={'TableColumnName','ObjectPropName','Table2ObjMap','Obj2TableMap','CellEditable','ColumnFormat','ColumnWidth'});
            %                  TableColumnName     ObjectPropName        Table2ObjMap                   Obj2TableMap                   CellEditable  ColumnFormat                                                 ColumnWidth 
            tableData(1,:)  = {"",                 "",                   {},                            {},                            false,        {[]},                                                        {'fit'}};  % ID column - Read only and no mapping
            tableData(2,:)  = {"Enable",           "Enable",             {@tab2ObjDirectMap},           {@obj2TabDirectMap},           true,         {'logical'},                                                 {53}};
            tableData(3,:)  = {"Power",            "Power",              {@evalFcn},                    {@mat2strFcn},                 true,         {'char'},                                                    {53}};
            tableData(4,:)  = {"BWPID",            "BandwidthPartID",    {@str2doubleFcn},              {@num2strFcn},                 true,         {{'1'}},                                                     {43}};
            tableData(5,:)  = {"SearchSpaceID",    "SearchSpaceID",      {@str2doubleFcn},              {@num2strFcn},                 true,         {{'1'}},                                                     {65}};
            tableData(6,:)  = {"AggregationLevel", "AggregationLevel",   {@str2doubleFcn},              {@num2strFcn},                 true,         {{'1', '2', '4', '8', '16'}},                                {87}};
            tableData(7,:)  = {"Candidate",        "AllocatedCandidate", {@tab2ObjMapCandidate},        {@obj2TabMapCandidate},        true,         {cellstr(string(1:8))},                                      {73}};
            tableData(8,:)  = {"CCEOffset",        "CCEOffset",          {@evalFcn},                    {@mat2strFcn},                 true,         {[]},                                                        {56}};
            tableData(9,:)  = {"AllocatedSlots",   "SlotAllocation",     {@evalFcn},                    {@mat2strFcn},                 true,         {'char'},                                                    {69}};
            tableData(10,:) = {"Period",           "Period",             {@evalFcn},                    {@mat2strFcn},                 true,         {'char'},                                                    {53}};
            tableData(11,:) = {"Coding",           "Coding",             {@tab2ObjDirectMap},           {@obj2TabDirectMap},           true,         {'logical'},                                                 {56}};
            tableData(12,:) = {"PayloadSize",      "DataBlockSize",      {@tab2ObjMapPayloadSize},      {@obj2TabMapPayloadSize},      true,         {[]},                                                        {61}};
            tableData(13,:) = {"DataSource",       "DataSource",         {@tab2ObjMapDataSource},       {@obj2TabMapDataSource},       true,         {{'PN9-ITU', 'PN9', 'PN11', 'PN15', 'PN23', 'User-defined'}},{80}};
            tableData(14,:) = {"CustomDataSource", "DataSource",         {@tab2ObjMapCustomDataSource}, {@obj2TabMapCustomDataSource}, false,        {'char'},                                                    {90}};
            tableData(15,:) = {"RNTI",             "RNTI",               {@tab2ObjDirectMap},           {@obj2TabDirectMap},           true,         {[]},                                                        {'fit'}};
            tableData(16,:) = {"DMRSScramblingID", "DMRSScramblingID",   {@evalFcn},                    {@mat2strFcn},                 true,         {'char'},                                                    {95}};
            tableData(17,:) = {"DMRSPower",        "DMRSPower",          {@tab2ObjDirectMap},           {@obj2TabDirectMap},           true,         {[]},                                                        {78}};
            tableData(18,:) = {"Label",            "Label",              {@tab2ObjDirectMap},           {@obj2TabDirectMap},           true,         {'char'},                                                    {'auto'}};
        end
    end
end

function tableMap = getTableColumnMap()
    % Provides the table column names, the equivalent properties of
    % configuration object, and the mapping rule between the two.
    tableMap = removevars(wirelessWaveformApp.nr5G_PDCCH_Table.getTableStaticData(), {'CellEditable','ColumnFormat','ColumnWidth'});
end

function tableViz = getTableVisuals()
    % Provides the table column names, together with their column format,
    % column width, and whether the default column is editable or not.
    tableViz = removevars(wirelessWaveformApp.nr5G_PDCCH_Table.getTableStaticData(), {'ObjectPropName','Table2ObjMap','Obj2TableMap'});
end