classdef nr5G_CORESET_Table < wirelessWaveformApp.nr5G_Table
    % Creation and handling of the CORESET table in the PDCCH tab of Downlink 5G

    %   Copyright 2024 The MathWorks, Inc.

    events
        IDChanged % Explicitly notify the app that CORESETID has changed
    end

    % Constructor and public interface methods
    methods (Access = public)
        function obj = nr5G_CORESET_Table(parent, defaultConfig, columnsToHide)

            arguments
                % uiobject that contains the table
                parent (1,1)
                % Cell array containing the app default nrCORESETConfig
                % configuration object(s)
                defaultConfig cell
                % List of the names of the table columns to remove during
                % construction, if any (optional)
                columnsToHide (1,:) string = string.empty;
            end

            % Call base-class constructor
            obj@wirelessWaveformApp.nr5G_Table(parent, defaultConfig, ...
                                               Name = getString(message("nr5g:waveformApp:CORESETTableName")), ...
                                               Tag = "CORESET", ...
                                               TableColumnMap = getTableColumnMap(), ...
                                               TableVisuals = getTableVisuals(), ...
                                               HasTitle = true, ...
                                               ColumnsToHide = columnsToHide);

            % Update CORESET table grid to fit the content
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
            newIDs = cellfun(@(x)x.CORESETID,chWaveCfg);
            if isempty(nvargs.ConfigIDs) && ~isequal(existingIDs,newIDs)
                notify(obj, 'IDChanged');
            end
        end
    end

    % Protected methods
    methods (Access = protected)
        function data = getNewRowData(obj)
            % Call baseclass method
            data = getNewRowData@wirelessWaveformApp.nr5G_Table(obj);

            if isempty(obj.Table.Data)
                % Initialization. This must be CORESET0
                nextID = 0;
                % Assign the new ID to the first column, which is always
                % the ID column
                data{1,1} = nextID;
                % Construct the new Label with the new ID
                labelColIdx = (getTableColumnName(obj) == "Label");
                data{1,labelColIdx} = replace(data{1,labelColIdx},digitsPattern,string(nextID));
            end
        end

        function addTableEntry(obj, ~, ~)
            % Executes when the Add button is clicked

            % Call base-class method
            addTableEntry@wirelessWaveformApp.nr5G_Table(obj);

            % Notify the app of the ID update
            notify(obj, 'IDChanged');
        end

        function noOp = removeTableEntry(obj, cfgID, ~)
            % Executes when the Remove button is clicked

            % If the selected row contains CORESET0, do nothing
            if isCORESET0(obj)
                return;
            end

            % Call base-class method
            noOp = removeTableEntry@wirelessWaveformApp.nr5G_Table(obj, cfgID);

            if ~noOp
                % Notify the app of the ID update
                notify(obj, 'IDChanged');
            end
        end

        function duplicateTableEntry(obj, ~, ~)
            % CORESET-specific callback for when a row is duplicated when CORESET0 row is selected

            % Call base-class method
            duplicateTableEntry@wirelessWaveformApp.nr5G_Table(obj);

            % Notify the app of the ID update
            notify(obj, 'IDChanged');
        end

        function updateEditability(obj, data)
            % Update the editability of the cells for this table

            % Call base class method that resets all cells to the default
            % state
            updateEditability@wirelessWaveformApp.nr5G_Table(obj, data);

            tableColName = getTableColumnName(obj);

            % BundleSize, InterleaverSize, and ShiftIndex are read-only
            % when CCEREGMapping is noninterlaved
            CCEREGMappingColIdx = strcmp(tableColName, 'CCEREGMapping');
            editableRows = strcmp(data(:, CCEREGMappingColIdx), 'interleaved');
            colIdx = any(tableColName == ["REGBundleSize", "InterleaverSize", "ShiftIndex"], 2);
            obj.CellEditable( editableRows, colIdx) = true;
            obj.CellEditable(~editableRows, colIdx) = false;
        end

        function buttonEnabled = enableRemoveButton(obj)
            % On top of the base rules for when the Remove button is
            % enabled or not, the Remove button should also be disabled for
            % CORESET0.

            buttonEnabled = ~isCORESET0(obj) && enableRemoveButton@wirelessWaveformApp.nr5G_Table(obj);
        end
    end

    methods (Access = private)
        function flag = isCORESET0(obj)
            % Returns true if the current selection contains CORESET0
            flag = any(obj.AllIDs(obj.Selection)==0);
        end

        %% Table <--> Object Mapping Functions
        % REGBundleSize
        function [tableRow, chWaveCfg] = tab2ObjMapREGBundleSize(~, tableRow, colIdx, chWaveCfg)
            if strcmp(chWaveCfg.CCEREGMapping, 'noninterleaved')
                tableRow{colIdx} = 'N/A';
            else
                % If BundleSize is 'N/A' then CCEREGMapping just changed.
                % Set it to the default value.
                if strcmp(tableRow{colIdx}, 'N/A')
                    tableRow{colIdx} = num2str(chWaveCfg.REGBundleSize);
                else
                    chWaveCfg.REGBundleSize = str2double(tableRow{colIdx});
                end
            end
        end
        function tableVal = obj2TabMapREGBundleSize(~, propVal, chWaveCfg, ~)
            if strcmp(chWaveCfg.CCEREGMapping, 'interleaved')
                tableVal = num2str(propVal);
            else
                tableVal = 'N/A';
            end
        end

        % InterleaverSize
        function [tableRow, chWaveCfg] = tab2ObjMapInterleaverSize(~, tableRow, colIdx, chWaveCfg)
            if strcmp(chWaveCfg.CCEREGMapping, 'noninterleaved')
                tableRow{colIdx} = 'N/A';
            else
                % If InterleaverSize is 'N/A' then CCEREGMapping just changed.
                % Set it to the default value.
                if strcmp(tableRow{colIdx}, 'N/A')
                    tableRow{colIdx} = num2str(chWaveCfg.InterleaverSize);
                else
                    chWaveCfg.InterleaverSize = str2double(tableRow{colIdx});
                end
            end
        end
        function tableVal = obj2TabMapInterleaverSize(~, propVal, chWaveCfg, ~)
            if strcmp(chWaveCfg.CCEREGMapping, 'interleaved')
                tableVal = num2str(propVal);
            else
                tableVal = 'N/A';
            end
        end

        % ShiftIndex
        function [tableRow, chWaveCfg] = tab2ObjMapShiftIndex(obj, tableRow, colIdx, chWaveCfg)
            if strcmp(chWaveCfg.CCEREGMapping, 'noninterleaved')
                tableRow{colIdx} = 'N/A';
            else
                % If ShiftIndex is 'N/A' then CCEREGMapping just changed.
                % Set it to the default value.
                if strcmp(tableRow{colIdx}, 'N/A')
                    tableRow{colIdx} = chWaveCfg.ShiftIndex;
                else
                    [tableRow, chWaveCfg] = tab2ObjDirectMap(obj, tableRow, colIdx, chWaveCfg);
                end
            end
        end
        function tableVal = obj2TabMapShiftIndex(~, propVal, chWaveCfg, ~)
            if strcmp(chWaveCfg.CCEREGMapping, 'interleaved')
                tableVal = propVal;
            else
                tableVal = 'N/A';
            end
        end
    end

    methods (Static, Access = private)
        function tableData = getTableStaticData()
            % Provides a table containing all static data associated with
            % this class
            tableData = table(Size=[10, 7], VariableTypes={'string','string','cell','cell','logical','cell','cell'}, VariableNames={'TableColumnName','ObjectPropName','Table2ObjMap','Obj2TableMap','CellEditable','ColumnFormat','ColumnWidth'});
            %                 TableColumnName         ObjectPropName         Table2ObjMap                  Obj2TableMap                  CellEditable  ColumnFormat                                   ColumnWidth
            tableData(1,:)  = {"CORESETID",           "CORESETID",           {@tab2ObjDirectMap},          {@obj2TabDirectMap},          false,        {[]},                                         {"fit"}};
            tableData(2,:)  = {"FrequencyResources",  "FrequencyResources",  {@evalFcn},                   {@mat2strFcn},                true,         {'char'},                                     {80}};
            tableData(3,:)  = {"Duration",            "Duration",            {@str2doubleFcn},             {@num2strFcn},                true,         {{'1', '2', '3'}},                            {64}};
            tableData(4,:)  = {"CCEREGMapping",       "CCEREGMapping",       {@tab2ObjDirectMap},          {@obj2TabDirectMap},          true,         {nrCORESETConfig.CCEREGMapping_Values},       {75}};
            tableData(5,:)  = {"REGBundleSize",       "REGBundleSize",       {@tab2ObjMapREGBundleSize},   {@obj2TabMapREGBundleSize},   true,         {{'2', '3', '6'}},                            {85}};
            tableData(6,:)  = {"InterleaverSize",     "InterleaverSize",     {@tab2ObjMapInterleaverSize}, {@obj2TabMapInterleaverSize}, true,         {{'2', '3', '6'}},                            {75}};
            tableData(7,:)  = {"ShiftIndex",          "ShiftIndex",          {@tab2ObjMapShiftIndex},      {@obj2TabMapShiftIndex},      true,         {[]},                                         {46}};
            tableData(8,:)  = {"PrecoderGranularity", "PrecoderGranularity", {@tab2ObjDirectMap},          {@obj2TabDirectMap},          true,         {nrCORESETConfig.PrecoderGranularity_Values}, {85}};
            tableData(9,:)  = {"RBOffset",            "RBOffset",            {@evalFcn},                   {@mat2strFcn},                true,         {{'[]','0','1','2','3','4','5'}},             {52}};
            tableData(10,:) = {"Label",               "Label",               {@tab2ObjDirectMap},          {@obj2TabDirectMap},          true,         {'char'},                                     {"auto"}};
        end
    end
end

function tableMap = getTableColumnMap()
    % Provides the table column names, the equivalent properties of
    % configuration object, and the mapping rule between the two.
    tableMap = removevars(wirelessWaveformApp.nr5G_CORESET_Table.getTableStaticData(), {'CellEditable','ColumnFormat','ColumnWidth'});
end

function tableViz = getTableVisuals()
    % Provides the table column names, together with their column format,
    % column width, and whether the default column is editable or not.
    tableViz = removevars(wirelessWaveformApp.nr5G_CORESET_Table.getTableStaticData(), {'ObjectPropName','Table2ObjMap','Obj2TableMap'});
end