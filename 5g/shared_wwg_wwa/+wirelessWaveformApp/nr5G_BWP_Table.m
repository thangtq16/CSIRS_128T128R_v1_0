classdef nr5G_BWP_Table < wirelessWaveformApp.nr5G_Table
    % Creation and handling of the BWP table in the Main tab

    %   Copyright 2024 The MathWorks, Inc.

    properties (Access = private)
        isDownlink
        pFrequencyRange (1,1) string = "FR1";
        SCSGridSize  = wirelessWaveformApp.nr5G_SCSCarriers_Table.DefaultSCSGridSize;
        SCSGridStart = wirelessWaveformApp.nr5G_SCSCarriers_Table.DefaultSCSGridStart;
    end

    events
        SCSGridValuesChanged % Explicitly notify the app that the SCS grid has changed
    end

    % Constructor and public interface methods
    methods (Access = public)
        function obj = nr5G_BWP_Table(parent, defaultConfig, isDownlink, columnsToHide)

            arguments
                % uiobject that contains the table
                parent (1,1)
                % Cell array containing the app default nrWavegenBWPConfig
                % configuration object(s)
                defaultConfig cell
                % Flag that specifies whether this table refers to Downlink
                % or Uplink
                isDownlink (1,1) logical
                % List of the names of the table columns to remove during
                % construction, if any (optional)
                columnsToHide (1,:) string = string.empty;
            end

            % Call base-class constructor
            obj@wirelessWaveformApp.nr5G_Table(parent, defaultConfig, ...
                                               Name = getString(message("nr5g:waveformApp:BWPTableName")), ...
                                               Tag = "BWP", ...
                                               PropNamesToLink = "SubcarrierSpacing", ...
                                               TableColumnMap = getTableColumnMap(), ...
                                               TableVisuals = getTableVisuals(), ...
                                               HasTitle = true, ...
                                               ColumnsToHide = columnsToHide);

            % Update the table grid to fit the content
            obj.TableGrid.ColumnWidth{1} = 'fit';
            obj.TableGrid.RowHeight{end} = 'fit';

            % Set the link state of this table
            obj.isDownlink = isDownlink;
        end

        %% Update internal properties
        function updatePropertyValues(obj, nvargs)
            % Update the available values of the given internal property.
            % Supported properties are:
            % * FrequencyRange
            % * SubcarrierSpacing
            % * SCSCarrier - This updates the internal cache of start grid
            %                and grid size for the given SCS carrier

            arguments
                obj
                % Name of the internal property to update
                nvargs.PropertyName (1,1) string {mustBeMember(nvargs.PropertyName, ...
                    ["FrequencyRange", "SubcarrierSpacing", "SCSCarriers"])}
                % New list of allowed values
                nvargs.NewList
            end

            propName = nvargs.PropertyName;
            newList = nvargs.NewList;
            switch propName
                case "FrequencyRange"
                    updateFrequencyRange(obj, newList);
                case "SubcarrierSpacing"
                    updateSCSOptions(obj, newList);
                case "SCSCarriers"
                    scs = newList{1};
                    scsSize = newList{2};
                    scsStart = newList{3};
                    updateSCSGridValues(obj, scs, scsSize, scsStart);
            end
        end
    end

    % Protected methods
    methods (Access = protected)
        function data = getNewRowData(obj)
            % Returns the data content of the new row, adjusted with the
            % smallest available ID

            % Call base-class method
            data = getNewRowData@wirelessWaveformApp.nr5G_Table(obj);

            % Start with NSizeBWP/NStartBWP equal to NSizeGrid/NSizeBWP for
            % the same SCS
            tableColName = getTableColumnName(obj);
            scsCol = strcmp(tableColName, 'SubcarrierSpacing');
            scs = data{1, scsCol};
            [bwpSize, bwpStart] = getMaxBWPStartSize(obj, scs);
            bwpCol = contains(tableColName, ["BWPStart","BWPSize"]);
            data(1, bwpCol) = {bwpSize, bwpStart};
        end

        function updateEditability(obj, data)
            % Update the editability of the cells for this table

            % Call base class method that resets all cells to the default
            % state
            updateEditability@wirelessWaveformApp.nr5G_Table(obj, data);

            tableColName = getTableColumnName(obj);

            % Cyclic prefix is read-only when SCS is anything but 60 kHz
            CyclicPrefixColIdx = strcmp(tableColName, 'CyclicPrefix');
            editableRows = cellfun(@(x)(startsWith(x,'60')), data(:, strcmp(tableColName, 'SubcarrierSpacing')));
            obj.CellEditable( editableRows, CyclicPrefixColIdx) = true;
            obj.CellEditable(~editableRows, CyclicPrefixColIdx) = false;
        end
    end

    methods (Access = private)
        function updateFrequencyRange(obj, val)
            obj.pFrequencyRange = val;

            % Update SCS allowed values and clear the existing table
            obj.Table.Data = {};

            % Add a new legitimate entry
            data = getNewRowData(obj);
            obj.Table.Data = data;

            % Update SCS dropdown in the BWP table
            if obj.pFrequencyRange == "FR1"
                newSCSList = wirelessWaveformApp.nr5G_SCSCarriers_Table.DefaultSCSListFR1(1);
            else % FR2
                if obj.isDownlink
                    newSCSList = wirelessWaveformApp.nr5G_SCSCarriers_Table.DefaultSCSListFR2_DL(1:2);
                else %UL
                    newSCSList = wirelessWaveformApp.nr5G_SCSCarriers_Table.DefaultSCSListFR2_UL(1);
                end
            end
            updateSCSOptions(obj, newSCSList);

            % Update cells editability
            updateEditability(obj, data);
        end

        function updateSCSOptions(obj, scsList)
            % Set the available SCS as options in the dropdown of the BWP table,
            % i.e., make it difficult for a BWP to link to a non-present carrier.

            if ~isempty(scsList)
                % Empty is a corner case for FR2, and single-carrier 240 kHz SCS. BWP
                % doesn't support that and error will be thrown in generation.
                updateDropdownOptions(obj, 'SubcarrierSpacing', scsList, true);
            end
        end

        function updateSCSGridValues(obj, scs, scsSize, scsStart)
            % Update the cached values of SCS grid size and start for each
            % input subcarrier spacing
            scsKey = string(scs);
            % Grid size and start for 60 kHz are slightly different
            % depending on the frequency range
            scsKey(matches(scsKey,"60")) = scsKey(matches(scsKey,"60")) + "_" + extract(obj.pFrequencyRange, digitsPattern);
            % Update the cache
            obj.SCSGridSize(scsKey) = scsSize;
            obj.SCSGridStart(scsKey) = scsStart;

            notify (obj, 'SCSGridValuesChanged');
        end

        function [bwpSize, bwpStart] = getMaxBWPStartSize(obj, scs)
            % Get the maximum value of NSizeBWP and NStartBWP so that the
            % BWP spans by default the whole grid defined by the SCS
            % carrier for this subcarrier spacing.

            scsKey = char(extract(scs, digitsPattern)); % Remove the trailing kHz
            if matches(scsKey, "60")
                % Grid size and start for 60 kHz are slightly different
                % depending on the frequency range
                scsKey = scsKey + "_" + extract(obj.pFrequencyRange, digitsPattern);
            end
            bwpSize = obj.SCSGridSize(scsKey);
            bwpStart = obj.SCSGridStart(scsKey);
        end
    end

    %% Table <--> Object Mapping Functions
    methods (Access = private)
        % SubcarrierSpacing
        function [tableRow, chWaveCfg] = tab2ObjMapSCS(~, tableRow, colIdx, chWaveCfg)
            % To avoid potential undesired cross-parameter issues, set the
            % value of cyclic prefizx to normal here. The right value will
            % be set later on when the tab2ObjMapCP() method is called.
            chWaveCfg.CyclicPrefix = 'normal';
            chWaveCfg.SubcarrierSpacing = str2double(extract(string(tableRow{colIdx}),digitsPattern));
        end
        function tableVal = obj2TabMapSCS(~, propVal, ~, ~)
            tableVal = [num2str(propVal) ' kHz'];
        end

        % CyclicPrefix
        function [tableRow, chWaveCfg] = tab2ObjMapCP(~, tableRow, colIdx, chWaveCfg)
            if chWaveCfg.SubcarrierSpacing ~= 60
                % Ensure 'normal' cyclic prefix is set
                tableRow{colIdx} = 'normal';
            end
            chWaveCfg.CyclicPrefix = tableRow{colIdx};
        end

        % NSizeBWP
        function [tableRow, chWaveCfg] = tab2ObjMapNSizeBWP(obj, tableRow, colIdx, chWaveCfg)
            % Range-limit for NSizeBWP: Real integer from 1 to bwpSizeMax
            % for this carrier
            scs = string(chWaveCfg.SubcarrierSpacing);
            [bwpSizeMax, ~] = getMaxBWPStartSize(obj, scs);
            bwpSize = max(1, min(bwpSizeMax, round(real(tableRow{colIdx}))));
            tableRow{colIdx} = bwpSize;
            chWaveCfg.NSizeBWP = bwpSize;
        end

        % NStartBWP
        function [tableRow, chWaveCfg] = tab2ObjMapNStartBWP(obj, tableRow, colIdx, chWaveCfg)
            % Range-limit for NStartBWP: Real integer from 0 to 2473
            scs = string(chWaveCfg.SubcarrierSpacing);
            [bwpSizeMax, bwpStartMax] = getMaxBWPStartSize(obj, scs);
            bwpStart = max(bwpStartMax, max(0, min(2473, round(real(tableRow{colIdx})))));
            bwpSize = chWaveCfg.NSizeBWP;
            % Don't allow NStartBWP>bwpStartMax
            if bwpStart+bwpSize>bwpSizeMax+bwpStartMax
                bwpStart = bwpStartMax; % Don't allow BWP outside of carrier. Shift to carrier start
            end
            tableRow{colIdx} = bwpStart;
            chWaveCfg.NStartBWP = bwpStart;
        end
    end

    methods (Static, Access = private)
        function tableData = getTableStaticData()
            % Provides a table containing all static data associated with
            % this class
            tableData = table(Size=[6, 7], VariableTypes={'string','string','cell','cell','logical','cell','cell'}, VariableNames={'TableColumnName','ObjectPropName','Table2ObjMap','Obj2TableMap','CellEditable','ColumnFormat','ColumnWidth'});
            %                 TableColumnName      ObjectPropName       Table2ObjMap            Obj2TableMap         CellEditable  ColumnFormat              ColumnWidth
            tableData(1,:) = {"BWPID",             "BandwidthPartID",   {@tab2ObjDirectMap},    {@obj2TabDirectMap}, false,        {{'1'}},                  {"fit"}};
            tableData(2,:) = {"SubcarrierSpacing", "SubcarrierSpacing", {@tab2ObjMapSCS},       {@obj2TabMapSCS},    true,         {{'15 kHz'}},             {"fit"}};
            tableData(3,:) = {"CyclicPrefix",      "CyclicPrefix",      {@tab2ObjMapCP},        {@obj2TabDirectMap}, false,        {{'normal', 'extended'}}, {"fit"}};
            tableData(4,:) = {"BWPSize",           "NSizeBWP",          {@tab2ObjMapNSizeBWP},  {@obj2TabDirectMap}, true,         {[]},                     {"fit"}};
            tableData(5,:) = {"BWPStart",          "NStartBWP",         {@tab2ObjMapNStartBWP}, {@obj2TabDirectMap}, true,         {[]},                     {"fit"}};
            tableData(6,:) = {"Label",             "Label",             {@tab2ObjDirectMap},    {@obj2TabDirectMap}, true,         {'char'},                 {"auto"}};
        end
    end
end

function tableMap = getTableColumnMap()
    % Provides the table column names, the equivalent properties of
    % configuration object, and the mapping rule between the two.
    tableMap = removevars(wirelessWaveformApp.nr5G_BWP_Table.getTableStaticData(), {'CellEditable','ColumnFormat','ColumnWidth'});
end

function tableViz = getTableVisuals()
    % Provides the table column names, together with their column format,
    % column width, and whether the default column is editable or not.
    tableViz = removevars(wirelessWaveformApp.nr5G_BWP_Table.getTableStaticData(), {'ObjectPropName','Table2ObjMap','Obj2TableMap'});
end