classdef nr5G_SRS_Table < wirelessWaveformApp.nr5G_Table
    % Creation and handling of the SRS table in the SRS tab of Uplink 5G

    %   Copyright 2020-2024 The MathWorks, Inc.

    % Constructor and public interface methods
    methods (Access = public)
        function obj = nr5G_SRS_Table(parent, defaultConfig, columnsToHide)

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
                                               Name = getString(message("nr5g:waveformApp:SRSTableName")), ...
                                               Tag = "SRS", ...
                                               PropNamesToLink = "BandwidthPartID", ...
                                               TableColumnMap = getTableColumnMap(), ...
                                               TableVisuals = getTableVisuals(), ...
                                               HasTitle = true, ...
                                               ColumnsToHide = columnsToHide);
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

    %% Table <--> Object Mapping Functions
    methods (Access = private)
        % KBarTC
        function tableVal = obj2TabMapKBarTC(~, propVal, chWaveCfg, ~)
            tableVal = num2str(min(propVal, chWaveCfg.KTC-1));
        end
    end

    methods (Static, Access = private)
        function tableData = getTableStaticData()
            % Provides a table containing all static data associated with
            % this class
            tableData = table(Size=[17, 7], VariableTypes={'string','string','cell','cell','logical','cell','cell'}, VariableNames={'TableColumnName','ObjectPropName','Table2ObjMap','Obj2TableMap','CellEditable','ColumnFormat','ColumnWidth'});
            %                  TableColumnName   ObjectPropName     Table2ObjMap         Obj2TableMap         CellEditable  ColumnFormat                                                ColumnWidth 
            tableData(1,:)  = {"",               "",                {},                  {},                  false,        {[]},                                                       {"fit"}};  % ID column - Read only and no mapping
            tableData(2,:)  = {"Enable",         "Enable",          {@tab2ObjDirectMap}, {@obj2TabDirectMap}, true,         {'logical'},                                                {"1x"}};
            tableData(3,:)  = {"Power",          "Power",           {@tab2ObjDirectMap}, {@obj2TabDirectMap}, true,         {[]},                                                       {"1x"}};
            tableData(4,:)  = {"BWPID",          "BandwidthPartID", {@str2doubleFcn},    {@num2strFcn},       true,         {{'1'}},                                                    {"1x"}};
            tableData(5,:)  = {"SRSPorts",       "NumSRSPorts",     {@str2doubleFcn},    {@num2strFcn},       true,         {cellstr(string(nrWavegenSRSConfig.NumSRSPorts_Options))},  {"1x"}};
            tableData(6,:)  = {"SymbolStart",    "SymbolStart",     {@str2doubleFcn},    {@num2strFcn},       true,         {cellstr(string(nrWavegenSRSConfig.SymbolStart_Options))},  {"1x"}};
            tableData(7,:)  = {"SRSSymbols",     "NumSRSSymbols",   {@str2doubleFcn},    {@num2strFcn},       true,         {cellstr(string(nrWavegenSRSConfig.NumSRSSymbols_Options))},{"1x"}};
            tableData(8,:)  = {"SlotAllocation", "SlotAllocation",  {@evalFcn},          {@mat2strFcn},       true,         {'char'},                                                   {"1x"}};
            tableData(9,:)  = {"Period",         "Period",          {@evalFcn},          {@mat2strFcn},       true,         {'char'},                                                   {"1x"}};
            tableData(10,:) = {"FrequencyStart", "FrequencyStart",  {@tab2ObjDirectMap}, {@obj2TabDirectMap}, true,         {[]},                                                       {"1x"}};
            tableData(11,:) = {"NRRC",           "NRRC",            {@tab2ObjDirectMap}, {@obj2TabDirectMap}, true,         {[]},                                                       {"1x"}};
            tableData(12,:) = {"CSRS",           "CSRS",            {@tab2ObjDirectMap}, {@obj2TabDirectMap}, true,         {[]},                                                       {"1x"}};
            tableData(13,:) = {"BSRS",           "BSRS",            {@str2doubleFcn},    {@num2strFcn},       true,         {cellstr(string(nrWavegenSRSConfig.BSRS_Options))},         {"1x"}};
            tableData(14,:) = {"BHop",           "BHop",            {@str2doubleFcn},    {@num2strFcn},       true,         {cellstr(string(nrWavegenSRSConfig.BHop_Options))},         {"1x"}};
            tableData(15,:) = {"Repetition",     "Repetition",      {@str2doubleFcn},    {@num2strFcn},       true,         {cellstr(string(nrWavegenSRSConfig.Repetition_Options))},   {"1x"}};
            tableData(16,:) = {"KTC",            "KTC",             {@str2doubleFcn},    {@num2strFcn},       true,         {cellstr(string(nrWavegenSRSConfig.KTC_Options))},          {"1x"}};
            tableData(17,:) = {"KBarTC",         "KBarTC",          {@str2doubleFcn},    {@obj2TabMapKBarTC}, true,         {cellstr(string(nrWavegenSRSConfig.KBarTC_Options))},       {"1x"}};
        end
    end
end

function tableMap = getTableColumnMap()
    % Provides the table column names, the equivalent properties of
    % configuration object, and the mapping rule between the two.
    tableMap = removevars(wirelessWaveformApp.nr5G_SRS_Table.getTableStaticData(), {'CellEditable','ColumnFormat','ColumnWidth'});
end

function tableViz = getTableVisuals()
    % Provides the table column names, together with their column format,
    % column width, and whether the default column is read only or not.
    tableViz = removevars(wirelessWaveformApp.nr5G_SRS_Table.getTableStaticData(), {'ObjectPropName','Table2ObjMap','Obj2TableMap'});
end