classdef nr5G_SCSCarriers_Table < wirelessWaveformApp.nr5G_Table
% Creation and handling of the SCS Carriers table in the Main tab of full 5G 

%   Copyright 2020-2024 The MathWorks, Inc.

    properties (Constant)
        DefaultSCSListFR1    = {'15 kHz', '30 kHz',  '60 kHz'};
        DefaultSCSListFR2_DL = {'60 kHz', '120 kHz', '240 kHz', '480 kHz', '960 kHz'};
        DefaultSCSListFR2_UL = {'60 kHz', '120 kHz', '480 kHz', '960 kHz'};
        DefaultSCSGridSize   = getDefaultSCSGridValue('size');
        DefaultSCSGridStart  = getDefaultSCSGridValue('start');
    end

    properties (Access = private)
        isDownlink
        FrequencyRange (1,1) string = "FR1";
        CachedSCS
    end

    % Constructor and public interface methods
    methods (Access = public)
        function obj = nr5G_SCSCarriers_Table(parent, defaultConfig, isDownlink, columnsToHide)

            arguments
                % uiobject that contains the table
                parent (1,1)
                % Cell array containing the app default nrSCSCarrierConfig
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
                                               Name = getString(message("nr5g:waveformApp:SCSTableName")), ...
                                               Tag = "SCS", ...
                                               PropNamesToLink = "SubcarrierSpacing", ...
                                               TableColumnMap = getTableColumnMap(), ...
                                               TableVisuals = getTableVisuals(), ...
                                               HasTitle = true, ...
                                               HasDuplicateButton = false, ...
                                               ColumnsToHide = columnsToHide);

            % Update SCS Carriers table grid to fit the content
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

            arguments
                obj
                % Name of the internal property to update
                nvargs.PropertyName (1,1) string {mustBeMember(nvargs.PropertyName, "FrequencyRange")}
                % New list of allowed values
                nvargs.NewList
            end

            propName = nvargs.PropertyName;
            newList = nvargs.NewList;
            switch propName
                case "FrequencyRange"
                    updateFrequencyRange(obj, newList);
            end
        end

        function [gridSize,gridStart] = getSCSDefaultGridData(obj,scs)

            arguments
                obj
                scs string
            end

            % Find maximum allowed NumRB for the new SCS
            scsKey = char(extract(scs,digitsPattern)); % Remove the trailing kHz
            if matches(scsKey,"60")
                % Grid size and start for 60 kHz are slightly different
                % depending on the frequency range
                scsKey = scsKey + "_" + extract(obj.FrequencyRange, digitsPattern);
            end
            gridSize = obj.DefaultSCSGridSize(scsKey);
            gridStart = obj.DefaultSCSGridStart(scsKey);
        end
    end

    methods (Access = protected)
        function data = getNewRowData(obj)
            % Returns the data content of the new row, adjusted with the
            % smallest available ID
            % Override of the base class method

            % Get possible new SCS (only those that haven't been used yet for this FR)
            candidateSCS = getCandidateSCS(obj);
            scs = candidateSCS{1};  % get the 1st allowable SCS
            [gridSize,gridStart] = getSCSDefaultGridData(obj,scs);

            % Get the smallest available ID
            nextID = appendNextAvailableRowName(obj);

            % Assign the data
            data = {nextID, scs, gridSize, gridStart};

            obj.RowEditable(end+1) = true;
        end

        function addTableEntry(obj, ~, ~)
            % Executes when the Add button is clicked

            tableColName = getTableColumnName(obj);
            scsColIdx = find(tableColName=="SubcarrierSpacing");
            maxNumRows = numel(obj.Table.ColumnFormat{scsColIdx}); % Maximum number of rows for SCS table
            if size(obj.Table.Data, 1) == maxNumRows
                % There is 1 carrier for every possible SCS. No more allowed. If
                % by chance the user clicked on the "Add" button too many times
                % too fast for the app to disable the button in time, make sure
                % this function is a no-op, instead of trying to add another row
                % that would otherwise result in an error down the pipeline.
                return;
            end

            % Call base-class method
            addTableEntry@wirelessWaveformApp.nr5G_Table(obj);

            % Cache to support unique SCS per carrier
            obj.CachedSCS = obj.Table.Data(:, scsColIdx);
        end

        function buttonEnabled = enableAddButton(obj)
            % Control interactions with Add button
            % There is 1 carrier for every possible SCS. No more allowed

            tableColName = getTableColumnName(obj);
            scsColIdx = (tableColName=="SubcarrierSpacing");
            maxNumRows = numel(obj.Table.ColumnFormat{scsColIdx}); % Maximum number of rows for SCS table
            buttonEnabled = (size(obj.Table.Data, 1) < maxNumRows) && enableAddButton@wirelessWaveformApp.nr5G_Table(obj);
        end
    end

    methods (Access = private)
        function updateFrequencyRange(obj, val)
            obj.FrequencyRange = val;

            % Update SCS allowed values and clear the existing table
            % SCS dropdown in SCS table
            if obj.FrequencyRange == "FR1"
                newSCSList = obj.DefaultSCSListFR1;
            else % FR2
                if obj.isDownlink
                    newSCSList = obj.DefaultSCSListFR2_DL;
                else % UL
                    newSCSList = obj.DefaultSCSListFR2_UL;
                end
            end
            updateDropdownOptions(obj, "SubcarrierSpacing", newSCSList);

            % Add a new legitimate entry
            obj.Table.Data = {}; % Wipe clean the table first
            obj.Table.Data = getNewRowData(obj);
            if obj.FrequencyRange == "FR2" && obj.isDownlink
                % Add the 120 carrier, for the SS Burst not to error (no 60 kHz SS Burst)
                obj.Table.Data(2, :) = getNewRowData(obj);
            end
            updateEditability(obj, obj.Table.Data);

            % Disabled (full SCS) state no longer applies, we start
            % clean with 1 SCS entry
            updateButtonInteraction(obj);
        end

        function candidateSCS = getCandidateSCS(obj)
            % Get possible new SCS (only those that haven't been used yet for this FR)
            if ~isempty(obj.Table.Data)
                % Table has already been created
                tableColName = getTableColumnName(obj);
                scsColIdx = find(tableColName=="SubcarrierSpacing");
                existingSCS = obj.Table.Data(:, scsColIdx);                 % currently used
                possibleSCS = obj.Table.ColumnFormat{scsColIdx};            % all possible
                candidateSCS = setdiff(possibleSCS, existingSCS, 'stable'); % remaining
            else
                % 1st time launch
                if obj.FrequencyRange == "FR1"
                    candidateSCS = obj.DefaultSCSListFR1(1);
                else % FR2
                    if obj.isDownlink
                        candidateSCS = obj.DefaultSCSListFR2_DL(1);
                    else % UL
                        candidateSCS = obj.DefaultSCSListFR2_UL(1);
                    end
                end
            end
        end
    end

    %% Table <--> Object Mapping Functions
    methods (Access = private)
        % SubcarrierSpacing
        function [tableRow, chWaveCfg] = tab2ObjMapSCS(~, tableRow, colIdx, chWaveCfg)
            newSCS = str2double(extract(string(tableRow{colIdx}),digitsPattern));
            chWaveCfg.SubcarrierSpacing = newSCS;
        end
        function tableVal = obj2TabMapSCS(~, propVal, ~, ~)
            tableVal = [num2str(propVal) ' kHz'];
        end
    end

    methods (Static, Access = private)
        function tableData = getTableStaticData()
            % Provides a table containing all static data associated with
            % this class
            tableData = table(Size=[4, 7], VariableTypes={'string','string','cell','cell','logical','cell','cell'}, VariableNames={'TableColumnName','ObjectPropName','Table2ObjMap','Obj2TableMap','CellEditable','ColumnFormat','ColumnWidth'});
            %                 TableColumnName      ObjectPropName       Table2ObjMap         Obj2TableMap         CellEditable  ColumnFormat                      ColumnWidth
            tableData(1,:) = {"",                  "",                  {},                  {},                  false,        {[]},                             {"fit"}};  % ID column - Read only and no mapping
            tableData(2,:) = {"SubcarrierSpacing", "SubcarrierSpacing", {@tab2ObjMapSCS},    {@obj2TabMapSCS},    true,         {{'15 kHz', '30 kHz', '60 kHz'}}, {"fit"}};
            tableData(3,:) = {"GridSize",          "NSizeGrid",         {@tab2ObjDirectMap}, {@obj2TabDirectMap}, true,         {[]},                             {"fit"}};
            tableData(4,:) = {"GridStart",         "NStartGrid",        {@tab2ObjDirectMap}, {@obj2TabDirectMap}, true,         {[]},                             {"fit"}};
        end
    end
end

function tableMap = getTableColumnMap()
    % Provides the table column names, the equivalent properties of
    % configuration object, and the mapping rule between the two.
    tableMap = removevars(wirelessWaveformApp.nr5G_SCSCarriers_Table.getTableStaticData(), {'CellEditable','ColumnFormat','ColumnWidth'});
end

function tableViz = getTableVisuals()
    % Provides the table column names, together with their column format,
    % column width, and whether the default column is editable or not.
    tableViz = removevars(wirelessWaveformApp.nr5G_SCSCarriers_Table.getTableStaticData(), {'ObjectPropName','Table2ObjMap','Obj2TableMap'});
end

function out = getDefaultSCSGridValue(in)
% Get default SCS carrier size and start grid for all subcarrier spacing
% values. The output is a dictionary linking the subcarrier spacing to the
% grid size/start.

    % Initialize the dictionary for all SCS values
    out  = dictionary(["15","30","60_1","60_2","120","240","480","960"],nan);

    % Return the requested output
    if contains(in,"size",IgnoreCase=true)
        % Try to make all SCS span the same frequency range
        out("15")   = nr5g.internal.wavegen.getNumRB('FR1', 15, 50); % 50 MHz bandwidth
        out("30")   = nr5g.internal.wavegen.getNumRB('FR1', 30, 50); % 50 MHz bandwidth
        out("60_1") = nr5g.internal.wavegen.getNumRB('FR1', 60, 50); % 50 MHz bandwidth
        out("60_2") = nr5g.internal.wavegen.getNumRB('FR2', 60, 50); % 50 MHz bandwidth
        out("120")  = nr5g.internal.wavegen.getNumRB('FR2', 120, 50); % 50 MHz bandwidth
        out("240")  = 16;
        out("480")  = nr5g.internal.wavegen.getNumRB('FR2', 480, 400); % 400 MHz bandwidth
        out("960")  = nr5g.internal.wavegen.getNumRB('FR2', 960, 400); % 400 MHz bandwidth
    else % start
        out("15")   = 3;
        out("30")   = 3;
        out("60_1") = 2;
        out("60_2") = 3;
        out("120")  = 2;
        out("240")  = 1;
        out("480")  = 3;
        out("960")  = 3;
    end

end