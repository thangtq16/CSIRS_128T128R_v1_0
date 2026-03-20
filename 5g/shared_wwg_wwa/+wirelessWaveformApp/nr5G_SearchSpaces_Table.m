classdef nr5G_SearchSpaces_Table < wirelessWaveformApp.nr5G_Table
    % Creation and handling of the SearchSpaces table in the PDCCH tab of Downlink 5G

    %   Copyright 2024 The MathWorks, Inc.

    events
        IDChanged % Explicitly notify the app that SearchSpaceID has changed
        NumCandidatesChanged % Explicitly notify the app that NumCandidates has changed
    end

    % Constructor and public interface methods
    methods (Access = public)
        function obj = nr5G_SearchSpaces_Table(parent, defaultConfig, columnsToHide)

            arguments
                % uiobject that contains the table
                parent (1,1)
                % Cell array containing the app default nrSearchSpaceConfig
                % configuration object(s)
                defaultConfig cell
                % List of the names of the table columns to remove during
                % construction, if any (optional)
                columnsToHide (1,:) string = string.empty;
            end

            % Call base-class constructor
            obj@wirelessWaveformApp.nr5G_Table(parent, defaultConfig, ...
                                               Name = getString(message("nr5g:waveformApp:SSTableName")), ...
                                               Tag = "SS", ...
                                               PropNamesToLink = "CORESETID", ...
                                               TableColumnMap = getTableColumnMap(), ...
                                               TableVisuals = getTableVisuals(), ...
                                               HasTitle = true, ...
                                               ColumnsToHide = columnsToHide);

            % Update SearchSpaces table grid to fit the content
            obj.TableGrid.ColumnWidth{1} = 'fit';
            obj.TableGrid.RowHeight{end} = 'fit';
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

            % Get list of IDs in the table
            existingIDs = cellfun(@(x)x,obj.Table.Data(:,1)');

            % Call base-class method
            applyConfiguration@wirelessWaveformApp.nr5G_Table(obj, chWaveCfg, AllowUIChange=nvargs.AllowUIChange, IDs=nvargs.IDs, ConfigIDs=nvargs.ConfigIDs);

            % If the IDs have changed, notify the app of the ID update
            newIDs = cellfun(@(x)x.SearchSpaceID,chWaveCfg);
            if isempty(nvargs.ConfigIDs) && ~isequal(existingIDs,newIDs)
                notify(obj, 'IDChanged');
            end
        end

        %% Update internal properties
        function updatePropertyValues(obj, nvargs)
            % Update the available values of the PROPNAME property as
            % options in the dropdown of the table.
            % Supported properties are:
            % * CORESETID

            arguments
                obj
                % Name of the internal property to update
                nvargs.PropertyName (1,1) string {mustBeMember(nvargs.PropertyName, "CORESETID")}
                % New list of allowed values
                nvargs.NewList
            end

            % updateDropdownOptions automatically checks propName against
            % the list of properties to link to defined during construction
            updateDropdownOptions(obj, nvargs.PropertyName, nvargs.NewList);
        end
    end

    % Protected methods
    methods (Access = protected)
        function addTableEntry(obj, ~, ~)
            % Executes when the Add button is clicked

            % Call base-class method
            addTableEntry@wirelessWaveformApp.nr5G_Table(obj);

            % Notify the app of the ID update
            notify(obj, 'IDChanged');
        end

        function noOp = removeTableEntry(obj, cfgID, ~)
            % Executes when the Remove button is clicked

            % Call base-class method
            noOp = removeTableEntry@wirelessWaveformApp.nr5G_Table(obj, cfgID);

            if ~noOp
                % Notify the app of the ID update
                notify(obj, 'IDChanged');
            end
        end

        function duplicateTableEntry(obj, ~, ~)
            % SearchSpaces-specific callback for when a row is duplicated

            % Call base-class method
            duplicateTableEntry@wirelessWaveformApp.nr5G_Table(obj);

            % Notify the app of the ID update
            notify(obj, 'IDChanged');
        end

        % Method to handle special notification of the change of NumCandidates
        function notifySpecial(obj, changedCellIndex)
            % Handle special case of candidates
            if ~isempty(changedCellIndex)
                tableColName = getTableColumnName(obj);
                candidateColIdx = find(tableColName=="CandidatesPerAgLevel");
                NumCandidatesChanged = (changedCellIndex(2)==candidateColIdx);
                if NumCandidatesChanged
                    % Get the updated value of NumCandidates
                    candidates = evalin('base', obj.Table.Data{changedCellIndex(1), candidateColIdx});

                    % Notify the app of the NumCandidates update
                    notify(obj, 'NumCandidatesChanged', wirelessWaveformApp.internal.PropChangedEventData('NumCandidates', changedCellIndex(1), {candidates}));
                end
            end
        end
    end

    % Table <--> Object Mapping Functions
    methods (Access = private)
        % SlotPeriodAndOffset
        function [tableRow, chWaveCfg] = tab2ObjMapSlotPeriodAndOffset(obj, tableRow, colIdx, chWaveCfg)
            tableColName = getTableColumnName(obj);
            SlotPeriodColIdx = strcmp(tableColName, 'SlotPeriod');
            SlotOffsetColIdx = strcmp(tableColName, 'SlotOffset');
            slotPeriod = tableRow{SlotPeriodColIdx};
            if imag(slotPeriod)==0
                tableRow{SlotPeriodColIdx} = real(slotPeriod); % Ensure that no imaginary part is displayed if the input is real
            end
            slotOffset = tableRow{SlotOffsetColIdx};
            if imag(slotOffset)==0
                tableRow{SlotOffsetColIdx} = real(slotOffset); % Ensure that no imaginary part is displayed if the input is real
            end

            % Static set-time validation
            validateattributes(tableRow{colIdx},{'numeric'}, {'integer','row'}, ...
                '',replace(obj.Table.ColumnName{colIdx},"|"," "));
            chWaveCfg = validateMultiColumnProp(obj, chWaveCfg, "SlotPeriodAndOffset", [slotPeriod slotOffset]);
        end
        function tableVal = obj2TabMapSlotPeriod(~, propVal, ~, ~)
            % Period needs the first element of the SlotPeriodAndOffset
            % object property
            tableVal = propVal(1);
        end
        function tableVal = obj2TabMapSlotOffset(~, propVal, ~, ~)
            % Offset needs the second element of the SlotPeriodAndOffset
            % object property
            tableVal = propVal(2);
        end
    end

    methods (Static, Access = private)
        function tableData = getTableStaticData()
            % Provides a table containing all static data associated with
            % this class
            tableData = table(Size=[9, 7], VariableTypes={'string','string','cell','cell','logical','cell','cell'}, VariableNames={'TableColumnName','ObjectPropName','Table2ObjMap','Obj2TableMap','CellEditable','ColumnFormat','ColumnWidth'});
            %                 TableColumnName         ObjectPropName           Table2ObjMap                      Obj2TableMap             CellEditable  ColumnFormat             ColumnWidth
            tableData(1,:) = {"SearchSpaceID",        "SearchSpaceID",         {@tab2ObjDirectMap},              {@obj2TabDirectMap},     false,        {[]},                    {'fit'}};
            tableData(2,:) = {"CORESETID",            "CORESETID",             {@str2doubleFcn},                 {@num2strFcn},           true,         {{'0', '1'}},            {72}};
            tableData(3,:) = {"Type",                 "SearchSpaceType",       {@tab2ObjDirectMap},              {@obj2TabDirectMap},     true,         {{'ue', 'common'}},      {48}};
            tableData(4,:) = {"StartSymbol",          "StartSymbolWithinSlot", {@str2doubleFcn},                 {@num2strFcn},           true,         {cellstr(string(0:13))}, {58}};
            tableData(5,:) = {"SlotPeriod",           "SlotPeriodAndOffset",   {@tab2ObjMapSlotPeriodAndOffset}, {@obj2TabMapSlotPeriod}, true,         {[]},                    {52}};
            tableData(6,:) = {"SlotOffset",           "SlotPeriodAndOffset",   {@tab2ObjMapSlotPeriodAndOffset}, {@obj2TabMapSlotOffset}, true,         {[]},                    {50}};
            tableData(7,:) = {"Duration",             "Duration",              {@tab2ObjDirectMap},              {@obj2TabDirectMap},     true,         {[]},                    {64}};
            tableData(8,:) = {"CandidatesPerAgLevel", "NumCandidates",         {@evalFcn},                       {@mat2strFcn},           true,         {'char'},                {79}};
            tableData(9,:) = {"Label",                "Label",                 {@tab2ObjDirectMap},              {@obj2TabDirectMap},     true,         {'char'},                {'auto'}};
        end
    end
end

function tableMap = getTableColumnMap()
    % Provides the table column names, the equivalent properties of
    % configuration object, and the mapping rule between the two.
    tableMap = removevars(wirelessWaveformApp.nr5G_SearchSpaces_Table.getTableStaticData(), {'CellEditable','ColumnFormat','ColumnWidth'});
end

function tableViz = getTableVisuals()
    % Provides the table column names, together with their column format,
    % column width, and whether the default column is editable or not.
    tableViz = removevars(wirelessWaveformApp.nr5G_SearchSpaces_Table.getTableStaticData(), {'ObjectPropName','Table2ObjMap','Obj2TableMap'});
end