classdef nr5G_CSIRS_Table < wirelessWaveformApp.nr5G_Table
    % Creation and handling of the CSI-RS table in the CSI-RS tab of Downlink 5G

    %   Copyright 2020-2024 The MathWorks, Inc.

    % Constructor and public interface methods
    methods (Access = public)
        function obj = nr5G_CSIRS_Table(parent, defaultConfig, columnsToHide)

            arguments
                % uiobject that contains the table
                parent (1,1)
                % Cell array containing the app default nrWavegenCSIRSConfig
                % configuration object(s)
                defaultConfig cell
                % List of the names of the table columns to remove during
                % construction, if any (optional)
                columnsToHide (1,:) string = string.empty;
            end

            % Call base-class constructor
            obj@wirelessWaveformApp.nr5G_Table(parent, defaultConfig, ...
                                               Name = getString(message("nr5g:waveformApp:CSIRSTableName")), ...
                                               Tag = "CSIRS", ...
                                               PropNamesToLink = "BandwidthPartID", ...
                                               TableColumnMap = getTableColumnMap(), ...
                                               TableVisuals = getTableVisuals(), ...
                                               HasTitle = true, ...
                                               ColumnsToHide = columnsToHide);

            % Make the table grid to fit the table column-wise and ensure
            % it is scrollable. This avoids the table to overly stretch
            % when the app is extremely large.
            obj.TableGrid.ColumnWidth{1} = 'fit';
            obj.TableGrid.Scrollable = true;
        end

        %% Update internal properties
        function updatePropertyValues(obj, nvargs)
            % Update the available values of the given property as
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

    % Protected methods
    methods (Access = protected)
        function updateEditability(obj, data)
            % Update the editability of the cells for this table

            % Call base class method that resets all cells to the default
            % state
            updateEditability@wirelessWaveformApp.nr5G_Table(obj, data);

            tableColName = getTableColumnName(obj);

            % Power is read-only when CSIRSType is 'zp'
            CSIRSTypeColIdx = strcmp(tableColName, 'CSIRSType');
            if any(CSIRSTypeColIdx)
                PowerColIdx = strcmp(tableColName, 'Power');
                editableRows = strcmp('nzp', data(:, CSIRSTypeColIdx));
                obj.CellEditable( editableRows, PowerColIdx) = true;
                obj.CellEditable(~editableRows, PowerColIdx) = false;
            end

            % Period and SlotOffset are read-only when Periodic is
            % unchecked
            PeriodicColIdx = strcmp(tableColName, 'Periodic');
            if any(PeriodicColIdx)
                editableRows = cellfun(@(x)(x), data(:, PeriodicColIdx));
                PeriodColIdx = strcmp(tableColName, 'Period');
                obj.CellEditable( editableRows, PeriodColIdx) = true;
                obj.CellEditable(~editableRows, PeriodColIdx) = false;
                SlotOffsetColIdx = strcmp(tableColName, 'SlotOffset');
                obj.CellEditable( editableRows, SlotOffsetColIdx) = true;
                obj.CellEditable(~editableRows, SlotOffsetColIdx) = false;
            end

            % SymbolLocation2 is read-only when RowNumber~=[13,14,16,17]
            RowNumberColIdx = strcmp(tableColName, 'RowNumber');
            if any(RowNumberColIdx)
                SymbolLocation2ColIdx = strcmp(tableColName, 'SymbolLocation2');
                editableRows = contains(data(:, RowNumberColIdx), string([13,14,16,17]));
                obj.CellEditable( editableRows, SymbolLocation2ColIdx) = true;
                obj.CellEditable(~editableRows, SymbolLocation2ColIdx) = false;
            end
        end
    end

    % Table <--> Object Mapping Functions
    methods (Access = private)
        % CSIRSType
        function tableVal = obj2TabMapCSIRSType(obj, propVal, ~, ~)
            if iscell(propVal)
                propVal = propVal{1};
            end
            tableVal = obj2TabDirectMap(obj, propVal);
        end

        % Periodic
        function tableVal = obj2TabMapPeriodic(~, propVal, ~, ~)
            tableVal = ~strcmp(propVal, 'off');
        end

        % Period
        function [tableRow, chWaveCfg] = tab2ObjMapPeriod(obj, tableRow, colIdx, chWaveCfg)
            tableColName = getTableColumnName(obj);
            PeriodicColIdx = strcmp(tableColName, 'Periodic');
            periodic = tableRow{PeriodicColIdx};
            if ~periodic
                % If periodic is turned off, then Period does not apply
                tableRow{colIdx} = 'N/A'; % revert
                chWaveCfg.CSIRSPeriod = 'off';
            else
                period = tableRow{colIdx};
                if strcmp(period, 'N/A')  % Periodic checkbox was just enabled
                    period = '1';         % Start from a default
                end
                tableRow{colIdx} = period;
                if str2double(period)==1
                    chWaveCfg.CSIRSPeriod = 'on';
                else
                    offsetTmp = 0; % This value will be updated in the next loop
                    chWaveCfg.CSIRSPeriod = [str2double(period) offsetTmp];
                end
            end
        end
        function tableVal = obj2TabMapPeriod(~, propVal, ~, ~)
            if strcmp('off', propVal)
                tableVal = 'N/A';
            else
                if strcmp('on', propVal)
                    tableVal = '1';
                else
                    tableVal = num2str(propVal(1));
                end
            end
        end

        % SlotOffset
        function [tableRow, chWaveCfg] = tab2ObjMapSlotOffset(obj, tableRow, colIdx, chWaveCfg)
            tableColName = getTableColumnName(obj);
            PeriodicColIdx = strcmp(tableColName, 'Periodic');
            periodic = tableRow{PeriodicColIdx};
            if ~periodic
                % If periodic is turned off, then Period does not apply
                tableRow{colIdx} = 'N/A'; % revert
                chWaveCfg.CSIRSPeriod = 'off';
            else
                PeriodColIdx = strcmp(tableColName, 'Period');
                period = str2double(tableRow{PeriodColIdx});
                offset = tableRow{colIdx};
                if strcmp(offset, 'N/A')  % Periodic checkbox was just enabled
                    offset = 0;           % Start from a default
                elseif imag(offset)==0
                    % Ensure that no imaginary part is displayed if the input is real
                    offset = real(offset);
                end
                tableRow{colIdx} =  offset;
                % Static set-time validation of offset
                validateattributes(offset,{'numeric'},...
                    {'scalar','integer','nonnegative'},'',replace(obj.Table.ColumnName{colIdx},"|"," "));
                if period~=1
                    chWaveCfg = validateMultiColumnProp(obj, chWaveCfg, "CSIRSPeriod", [period offset]);
                end
            end
        end
        function tableVal = obj2TabMapSlotOffset(~, propVal, ~, ~)
            if strcmp('off', propVal)
                tableVal = 'N/A';
            else
                if strcmp('on', propVal)
                    tableVal = 0;
                else
                    tableVal = propVal(2);
                end
            end
        end

        % Density
        function [tableRow, chWaveCfg] = tab2ObjMapDensity(~, tableRow, colIdx, chWaveCfg)
            if contains(tableRow{colIdx},'odd')
                density = 'dot5odd';
            elseif contains(tableRow{colIdx},'even')
                density = 'dot5even';
            else
                density = tableRow{colIdx};
            end
            chWaveCfg.Density = density;
        end
        function tableVal = obj2TabMapDensity(~, propVal, ~, ~)
            if strcmp(propVal, 'dot5even')
                density = '0.5 (even)';
            elseif strcmp(propVal, 'dot5odd')
                density = '0.5 (odd)';
            else
                density = propVal;
            end
            tableVal = density;
        end

        % SymbolLocation1
        function tableVal = obj2TabMapSymbolLocation1(~, propVal, ~, ~)
            if iscell(propVal)
                propVal = propVal{1};
            end
            tableVal = num2str(propVal(1));
        end

        % SymbolLocation2
        function [tableRow, chWaveCfg] = tab2ObjMapSymbolLocation2(~, tableRow, colIdx, chWaveCfg)
            if ~any(chWaveCfg.RowNumber==[13,14,16,17])
                tableRow{colIdx} = 'N/A';
            end
            if ~strcmpi(tableRow{colIdx},'N/A')
                chWaveCfg.SymbolLocations(2) = str2double(tableRow{colIdx});
            end
        end
        function tableVal = obj2TabMapSymbolLocation2(~, propVal, ~, ~)
            if iscell(propVal)
                propVal = propVal{1};
            end
            if numel(propVal)>1
                tableVal = num2str(propVal(2));
            else
                tableVal = 'N/A';
            end
        end

        % SubcarrierLocations
        function tableVal = obj2TabMapSubcarrierLocations(obj, propVal, ~, ~)
            if iscell(propVal)
                propVal = propVal{1};
            end
            tableVal = mat2strFcn(obj, propVal);
        end
    end

    methods (Static, Access = private)
        function tableData = getTableStaticData()
            % Provides a table containing all static data associated with
            % this class
            tableData = table(Size=[17, 7], VariableTypes={'string','string','cell','cell','logical','cell','cell'}, VariableNames={'TableColumnName','ObjectPropName','Table2ObjMap','Obj2TableMap','CellEditable','ColumnFormat','ColumnWidth'});
            %                  TableColumnName        ObjectPropName         Table2ObjMap                  Obj2TableMap                     CellEditable  ColumnFormat                                                                          ColumnWidth
            tableData(1,:)  = {"",                    "",                    {},                           {},                              false,        {[]},                                                                                 {'fit'}};  % ID column - Read only and no mapping
            tableData(2,:)  = {"Enable",              "Enable",              {@tab2ObjDirectMap},          {@obj2TabDirectMap},             true,         {'logical'},                                                                          {53}};
            tableData(3,:)  = {"Power",               "Power",               {@tab2ObjDirectMap},          {@obj2TabDirectMap},             true,         {[]},                                                                                 {53}};
            tableData(4,:)  = {"BWPID",               "BandwidthPartID",     {@str2doubleFcn},             {@num2strFcn},                   true,         {{'1'}},                                                                              {43}};
            tableData(5,:)  = {"CSIRSType",           "CSIRSType",           {@tab2ObjDirectMap},          {@obj2TabMapCSIRSType},          true,         {nrWavegenCSIRSConfig.CSIRSType_Values},                                              {55}};
            tableData(6,:)  = {"Periodic",            "CSIRSPeriod",         {},                           {@obj2TabMapPeriodic},           true,         {'logical'},                                                                          {62}};
            tableData(7,:)  = {"Period",              "CSIRSPeriod",         {@tab2ObjMapPeriod},          {@obj2TabMapPeriod},             true,         {horzcat( 'N/A', '1', cellstr(string(nrWavegenCSIRSConfig.SlotPeriodicity_Options)))},{53}};
            tableData(8,:)  = {"SlotOffset",          "CSIRSPeriod",         {@tab2ObjMapSlotOffset},      {@obj2TabMapSlotOffset},         true,         {[]},                                                                                 {50}};
            tableData(9,:)  = {"RowNumber",           "RowNumber",           {@str2doubleFcn},             {@num2strFcn},                   true,         {cellstr(string(1:18))},                                                              {62}};
            tableData(10,:) = {"Density",             "Density",             {@tab2ObjMapDensity},         {@obj2TabMapDensity},            true,         {{'one', 'three', '0.5 (odd)', '0.5 (even)'}},                                        {58}};
            tableData(11,:) = {"SymbolLocation1",     "SymbolLocations",     {@str2doubleFcn},             {@obj2TabMapSymbolLocation1},    true,         {cellstr(string(0:13))},                                                              {78}};
            tableData(12,:) = {"SymbolLocation2",     "SymbolLocations",     {@tab2ObjMapSymbolLocation2}, {@obj2TabMapSymbolLocation2},    false,        {horzcat( 'N/A', cellstr(string(2:12)))},                                             {78}};
            tableData(13,:) = {"SubcarrierLocations", "SubcarrierLocations", {@evalFcn},                   {@obj2TabMapSubcarrierLocations},true,         {[]},                                                                                 {75}};
            tableData(14,:) = {"NumRB",               "NumRB",               {@tab2ObjDirectMap},          {@obj2TabDirectMap},             true,         {[]},                                                                                 {52}};
            tableData(15,:) = {"RBOffset",            "RBOffset",            {@tab2ObjDirectMap},          {@obj2TabDirectMap},             true,         {[]},                                                                                 {50}};
            tableData(16,:) = {"NID",                 "NID",                 {@tab2ObjDirectMap},          {@obj2TabDirectMap},             true,         {[]},                                                                                 {'fit'}};
            tableData(17,:) = {"Label",               "Label",               {@tab2ObjDirectMap},          {@obj2TabDirectMap},             true,         {'char'},                                                                             {100}};
        end
    end
end

function tableMap = getTableColumnMap()
    % Provides the table column names, the equivalent properties of
    % configuration object, and the mapping rule between the two.
    tableMap = removevars(wirelessWaveformApp.nr5G_CSIRS_Table.getTableStaticData(), {'CellEditable','ColumnFormat','ColumnWidth'});
end

function tableViz = getTableVisuals()
    % Provides the table column names, together with their column format,
    % column width, and whether the default column is editable or not.
    tableViz = removevars(wirelessWaveformApp.nr5G_CSIRS_Table.getTableStaticData(), {'ObjectPropName','Table2ObjMap','Obj2TableMap'});
end