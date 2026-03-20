classdef nr5G_Table < handle & ...
        wirelessWaveformApp.internal.ErrorPopup % Mixin to buy in the custom error popup
    % Common table configuration for 5G Wireless Waveform apps, where a table is defined as an
    % uitable with augmented functionality, such as add/remove/duplicate
    % buttons, update editability of table cells based on cross-parameter
    % rules, and map the content of the uitable to the wavegen
    % configuration object and viceversa.

    %   Copyright 2023-2024 The MathWorks, Inc.

    %% Properties
    properties (Dependent, Access = protected)
        % Properties needed by ErrorPopup
        ErrorFigure % Figure containing the object in error
        ErrorTitle % Error popup title
    end

    properties (GetAccess = public, SetAccess = private, SetObservable)
        % Index of the selected instance(s), specified as the
        % currently-selected row(s) in the table.
        Selection
    end

    properties (GetAccess = public, SetAccess = private, Dependent)
        % IDs of all channel instances, specified as the table row headings
        AllIDs
    end

    properties (SetAccess = protected, GetAccess = public)
        % Ability to edit rows in the Table property, defined as a logical
        % array of M elements, where M is the number of channel instances
        % (rows)
        RowEditable (1,:) logical = true(1,0)
    end

    properties (Access = protected)
        % Ability to edit cells in the Table property, defined as an M-by-N
        % logical array, where M is the number of channel instances (rows)
        % and N is the number of columns displayed in the table
        CellEditable
    end

    properties (GetAccess = protected, SetAccess = private)
        Table % uitable object
        TableGrid % Grid layout of table and buttons
        DefaultConfig % Store default wavegen config object
    end

    properties (Access = private)
        % Table identifiers
        Title
        Name
        Tag

        % Table functionality
        AddButton
        RemoveButton
        DuplicateButton
        NewRowDefaultData % Store the default data content of each new row

        % Property storing the visible table column names, their respective
        % config object property names, and the mapping rule between them
        TableColumnMap

        % Property storing the table column names, together with their
        % column format, column width, and whether the default column is
        % editable or not.
        TableVisuals

        % List of configuration object property names used in
        % setNewRowDefaultData() to ensure no link is broken for the
        % default case
        PropNamesToLink = string.empty;

        % Flags to define if the derived table has title and duplicate
        % button
        HasTitle           = false;
        HasDuplicateButton = true;

        % Cache information about configuration error. If the wavegen
        % configuration represented by the table has an error, ConfigError
        % stores the MException thrown when trying to update the wavegen
        % configuration object with the table data. If there is no error,
        % ConfigError is empty.
        ConfigError = [];

        % Cache of highlighted rows
        HighlightedRowsBrokenLink = []; % highlighted because fo broken link
        HighlightedRowsConflict = []; % highlighted because of conflict
    end

    properties (Constant, Access = private)
        % Tables color/style/behavior
        ErrorRowStyle        = matlab.ui.style.internal.SemanticStyle('BackgroundColor', '--mw-graphics-backgroundColor-secondary-error');
        ReadOnlyCellStyle    = matlab.ui.style.internal.SemanticStyle('BackgroundColor', '--mw-backgroundColor-input-readonly', 'FontColor', '--mw-color-readOnly');
        ReadOnlyCellBehavior = matlab.ui.style.Behavior('Editable', 'off');
        IDColumnStyle        = matlab.ui.style.internal.SemanticStyle('BackgroundColor', '--mw-backgroundColor-primary');

        % Additional table-related properties
        ButtonWidth = 21 % Dimension(s) of add, remove and duplicate buttons
    end

    properties (Constant, Access = protected)
        % Error ID for cross-parameter issues to avoid reverting the last
        % value that caused cross-parameter issues
        CrossParameterErrorID = "nr5g:waveformApp:TableCrossParameterIssue";
    end

    %% Events
    events
        TableChanged
    end

    %% Public methods
    methods (Access = public)
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

            % If AllowUIChange is false, ensure that all inputs are valid.
            % If not, consider AllowUIChange to be true.
            allowUIChange = nvargs.AllowUIChange;
            if ~allowUIChange &&...
                    (~isempty(nvargs.IDs) && length(nvargs.IDs)<length(chWaveCfg) || ...
                    ( isempty(nvargs.IDs) && length(obj.AllIDs)<length(chWaveCfg)))
                allowUIChange = true;
            end

            if allowUIChange
                % Restore default value to avoid wrong mapping between
                % cache and selected rows
                obj.Table.Selection = [];

                % Update the read-only property RowEditable to the default
                % value for all the rows
                obj.RowEditable = true(1, length(chWaveCfg));
            end

            % Call channel-specific method
            data = mapConfig2Table(obj, chWaveCfg, allowUIChange, nvargs.IDs, nvargs.ConfigIDs);

            % Update table
            if isempty(nvargs.ConfigIDs)
                obj.Table.Data = data;
            else
                idx = find(ismember(obj.AllIDs, nvargs.ConfigIDs));
                obj.Table.Data(idx, :) = data(idx, :);
            end
            updateEditability(obj, obj.Table.Data);

            % Update ability for user interaction with table buttons
            updateButtonInteraction(obj);

            % Update the internal error cached information, now that a new
            % configuration has been pushed in
            data = obj.Table.Data;
            try
                chWaveCfgTmp = repmat(obj.DefaultConfig(1),size(data,1),1);
                mapTable2Config(obj, data, chWaveCfgTmp);
                obj.ConfigError = []; % No error, clear the internal error cached information
            catch e
                % The wavegen configuration represented by the table has an
                % error. Update the internal error cached information.
                obj.ConfigError = e;
            end
        end

        function chWaveCfg = updateConfiguration(obj, chWaveCfg)
            % Map the table to the channel configuration object

            [~, chWaveCfg] = mapTable2Config(obj, obj.Table.Data, chWaveCfg);
        end

        function out = hasConfigError(obj)
            % Return the state of the configuration for this channel. The
            % output is true if the table represents an invalid wavegen
            % configuration for the specified channel, and false otherwise.
            out = ~isempty(obj.ConfigError);
        end

        function e = getConfigError(obj)
            % Return the wavegen configuration error corresponding to this
            % table, if any. If the table represents a valid wavegen
            % configuration, the output is empty.
            e = obj.ConfigError;
        end

        %% Table visualization
        function setLayout(obj, layout)
            % Set the layout of the 5G table to define its placement within
            % its container

            arguments
                % This object
                obj (1,1)

                % Name-Value Arguments

                % Index of the row(s) that contains the 5G table
                layout.Row (1,:) uint8 = 1;
                % Index of the column(s) that contains the 5G table
                layout.Column (1,:) uint8 = 1;
            end

            obj.TableGrid.Layout.Row = layout.Row;
            obj.TableGrid.Layout.Column = layout.Column;
        end

        function highlightRows(obj, nvargs)
            % Update the visualization of highlighting in the table.
            % If ROWINDEX is empty, existing highlighting is removed.
            % highlightRows may be called to mark broken link or conflict.
            % When called for one reason, rows already highlighted for the
            % other reason should remain highlighted.

            arguments
                % This nr5G_Table object
                obj

                % Highlighting reason: broken link or conflict
                nvargs.Reason (1,1) string {mustBeMember(nvargs.Reason,["BrokenLink","Conflict"])};

                % Highlighted row index: empty means highlighting should be
                % removed for all rows if they were highlighted for the
                % same reason as nvargs.Reason
                nvargs.Rows (1,:) uint8 = [];
            end

            resetTableStyle(obj,'row'); % remove all highlighting
            % Get the row indices and cache name to be updated
            switch nvargs.Reason
                % When highlighting for one reason, keep rows highlighted
                % for the other reason still highlighted
                case 'BrokenLink'
                    cacheToUpdate = 'HighlightedRowsBrokenLink';
                    rowIndex = unique([obj.HighlightedRowsConflict nvargs.Rows]);
                case 'Conflict'
                    cacheToUpdate = 'HighlightedRowsConflict';
                    rowIndex = unique([obj.HighlightedRowsBrokenLink nvargs.Rows]);
            end
            setTableErrorStyle(obj, rowIndex);
            obj.(cacheToUpdate) = nvargs.Rows;

        end

        function updateDisabled(obj)
            % Reset the Enable property of the table buttons after they
            % have been disabled

            % Reset the enable state of table and table buttons.
            set(obj.TableGrid.Children, 'Enable', uiservices.logicalToOnOff(true));

            % Update ability for user interaction with table buttons
            updateButtonInteraction(obj);
        end

        function updateRowEditable(obj, nvargs)
            % Update the editability of entire rows in the table. When a
            % row is non-editable, it cannot be deleted by user
            % interaction.

            arguments
                % Mandatory inputs
                obj

                % Name-Value Arguments
                % Row index to which apply the updated editability. Empty
                % means that the Editable value applies to all rows.
                nvargs.Row uint8 = [];
                % Scalar flag to defined whether the set row is editable or
                % not. If nvargs.Row specifies multiple rows, the same
                % value of Editable applies to all specified rows.
                nvargs.Editable (1,1) logical = true;
            end

            % If NV argument Row is empty, update the entire table
            if isempty(nvargs.Row)
                rows = 1:height(obj.Table.Data);
            else
                rows = nvargs.Row;
            end

            % Update the read-only property RowEditable
            obj.RowEditable(rows) = nvargs.Editable;
        end
    end

    %% Getters/Setters
    methods
        function selection = get.Selection(obj)
            % Selection is a numeric vector indicating the selected row
            selection = obj.Table.Selection;
        end

        function IDs = get.AllIDs(obj)
            % AllIDs is a numeric array indicating the row headings of this
            % table, where the first column represents the ID, which has
            % the same functionality and follows the same rules as the row
            % heading
            IDs = [obj.Table.Data{:,1}];
        end

        function set.CellEditable(obj, val)
            % When updating the value of CellEditable, automatically take
            % care of the editability style in the table
            obj.CellEditable = val;
            updateNonEditableCellStyle(obj);
        end

        function set.RowEditable(obj, val)
            % When updating the value of RowEditable, automatically take
            % care of the editability style in the table
            obj.RowEditable = val;
            updateNonEditableCellStyle(obj);
        end

        function fig = get.ErrorFigure(obj)
            % Get the figure containing this table
            fig = ancestor(obj.TableGrid, "matlab.ui.Figure");
        end

        function t = get.ErrorTitle(obj)
            % Define the title for the error popup
            t = obj.Name;
        end
    end

    %% Protected methods that can be used but not redefined by derived classes
    methods(Sealed, Access = protected)
        %% Constructor
        function obj = nr5G_Table(parent, defaultConfig, tableConfig)
            % Create the 5G table

            arguments
                % uiobject that contains the 5G table
                parent (1,1)
                % Cell array containing the app default nrWavegenXConfig
                % configuration object(s)
                defaultConfig cell

                % Name-Value arguments

                % Table name
                tableConfig.Name (1,1) string

                % Table tag
                tableConfig.Tag (1,1) string

                % Property storing the table column names, their respective
                % config object property names, and the mapping rule between them.
                % This is the internal private property containing all the
                % possible columns. Upon construction, the user can decide
                % whether to hide some of the table columns.
                tableConfig.TableColumnMap (:,4) table

                % Property storing the table column names, together with
                % their column format, column width, and if the column is
                % editable by default.
                % This is the internal private property containing all the
                % possible columns. Upon construction, the user can decide
                % whether to hide some of the table columns.
                tableConfig.TableVisuals (:,4) table

                % List of configuration object property names used to
                % ensure no link is broken when adding and extra row with
                % the default configuration (optional)
                tableConfig.PropNamesToLink (1,:) string = string.empty

                % Flag to define if the table has a title (optional)
                tableConfig.HasTitle (1,1) logical = false

                % Flag to define if the table has a duplicate button (optional)
                tableConfig.HasDuplicateButton (1,1) logical = true

                % List of the names of the table columns to remove during
                % construction, if any (optional)
                tableConfig.ColumnsToHide (1,:) string = string.empty
            end

            import matlab.graphics.internal.themes.specifyThemePropertyMappings

            % Remove table columns from the spec, if any was provided
            tableColumnMap = tableConfig.TableColumnMap;
            tableVisuals = tableConfig.TableVisuals;
            columnsToHide = tableConfig.ColumnsToHide;
            invisibleColumnIndices = any((tableColumnMap.TableColumnName~="" & tableColumnMap.TableColumnName==columnsToHide), 2);
            tableVisuals(invisibleColumnIndices, :) = [];
            tableColumnMap(invisibleColumnIndices, :) = [];

            % Store the default configuration object
            obj.DefaultConfig = defaultConfig;

            % Assign properties from the input Name-Value arguments
            obj.Name = tableConfig.Name;
            obj.Tag = tableConfig.Tag;
            obj.PropNamesToLink = tableConfig.PropNamesToLink;
            obj.HasTitle = tableConfig.HasTitle;
            obj.HasDuplicateButton = tableConfig.HasDuplicateButton;

            % Set up the grid layout to contain table and buttons
            obj.TableGrid = setTableGrid(obj, parent);

            % Create the table
            NCols = height(tableVisuals);
            tableTag = obj.Tag + "Table";
            emptyColumnName = tableColumnMap.TableColumnName=="";
            columnName = getTableString(tableTag, tableColumnMap.TableColumnName(~emptyColumnName));
            if any(emptyColumnName)
                columnName = cat(2, {''}, columnName);
            end
            obj.Table = uitable('Parent',                 obj.TableGrid, ...
                                'ColumnName',             columnName, ...
                                'ColumnFormat',           tableVisuals.ColumnFormat', ...
                                'ColumnWidth',            tableVisuals.ColumnWidth', ...
                                'ColumnEditable',         [false, true(1,NCols-1)],... % The first column acts as the row name for all tables so it's non-editable
                                'RowName',                {}, ... % The first column acts as the row name for all tables
                                'CellEditCallback',       @obj.tableCallback, ...
                                'SelectionChangedFcn',    @obj.selectTableEntry, ... % This is the default; each table might override it
                                'Tag',                    tableTag, ...
                                'RearrangeableColumns',   false, ...
                                'SelectionType',          'row', ...
                                'RowStriping',            false);
            obj.Table.addStyle(uistyle('HorizontalAlignment', 'left'));

            % Change the style of the first column to be always read-only
            % to act as the row heading
            setIDColumnStyle(obj);

            % Adjust layout
            obj.Table.Layout.Row = [2 5]; % Table spans all rows but the first
            obj.Table.Layout.Column = 1; % Table spans the first column

            % Set the table column map and the table visuals properties
            obj.TableColumnMap = tableColumnMap;
            obj.TableVisuals = tableVisuals;

            % Create the data content of each new row
            setNewRowDefaultData(obj);

            % Populate the first row with the default values
            obj.Table.Data = {};
            for tableRow = 1:numel(defaultConfig)
                % In case the default wave configuration object has
                % multiple instances, ensure that all of them are correctly
                % instantiated.
                obj.Table.Data(end+1, :) = getNewRowData(obj);
            end

            % Set the table background color to the default (white for light
            % mode and black for dark mode)
            specifyThemePropertyMappings(obj.Table,'BackgroundColor','--mw-backgroundColor-input');
            % Make sure to properly set initial read-only cells, if any
            updateEditability(obj, obj.Table.Data);

            % Add, remove, duplicate buttons
            addTableButtons(obj);

            % Add title, if any
            if ~obj.HasTitle
                obj.TableGrid.RowHeight{1} = 0; % No title
            else
                obj.Title = uilabel('Parent', obj.TableGrid, ...
                                    'Text', obj.Name, 'FontWeight', 'Bold');
                obj.Title.Layout.Row = 1;
                obj.Title.Layout.Column = 1;
            end
        end

        %% Table Buttons
        function updateButtonInteraction(obj)
            % Update user interaction with table buttons
            setButtonEnable(obj, "add", enableAddButton(obj));
            setButtonEnable(obj, "remove", enableRemoveButton(obj));
            if obj.HasDuplicateButton
                setButtonEnable(obj, "duplicate", enableDuplicateButton(obj));
            end
        end

        %% Table data handling
        function updateDropdownOptions(obj, propName, newList, updateFirstElement)
            % Update the dropdown options of the table column corresponding
            % to PROPNAME with the values given in NEWLIST.

            arguments
                % Mandatory inputs
                obj
                propName
                newList

                % Optionl inputs
                updateFirstElement (1,1) logical = false;
            end

            if isInitialized(obj)
                % Ensure only the properties defined in PropNameToLink are
                % considered here
                propName(~contains(propName, obj.PropNamesToLink)) = [];
                colIdx = find(strcmp(propName, obj.TableColumnMap.ObjectPropName));
                if ~isempty(colIdx)
                    firstElementChanged = ~isequal(obj.Table.ColumnFormat{colIdx}{1}, newList{1});
                    obj.Table.ColumnFormat{colIdx} = newList(:)';

                    % If the first element of a property that is linked has
                    % changed, the default data content of new rows needs
                    % updating
                    if firstElementChanged
                        setNewRowDefaultData(obj);
                    end

                    % If the currently-shown element is no longer
                    % available, and the optional input updateFirstElement
                    % is true, change it to the first item of the list
                    rowsToUpdate = ~ismember(obj.Table.Data(:,colIdx), newList(:));
                    if updateFirstElement && any(rowsToUpdate)
                        % Update the affected rows and their editability
                        obj.Table.Data(rowsToUpdate,colIdx) = deal(newList(1));
                        updateEditability(obj, obj.Table.Data);
                    end
                end
            end
        end

        function chWaveCfg = validateMultiColumnProp(obj, chWaveCfg, propName, value)
            % For those cases where a single property PROPNAME of the
            % configuration object is split into multiple table columns,
            % validate that the values of all related columns do not clash
            % with each other causing a cross-parameter issue. If there is
            % no issue, assign the input VALUE to the configuration object
            % property. If there is an issue, throw an error.
            try
                chWaveCfg.(propName) = value;
            catch e
                % This is a cross-parameter issue. Modify the error to use
                % an identifier so that the app infrastructure knows this
                % is a cross-parameter issue and doesn't revert the value.
                e2 = MException(obj.CrossParameterErrorID, e.message);
                throwAsCaller(e2);
            end
        end

        %% General tools
        function setIDColumnStyle(obj)
            % Set the first column of the table as the ID column so that it
            % looks and feel like a row column.
            addStyle(obj.Table,obj.IDColumnStyle,'column',1); % Change background color
            addStyle(obj.Table,uistyle('FontWeight','bold'),'column',1); % Change font weight to bold
        end

        function nextID = appendNextAvailableRowName(obj)
            % This methods finds the smallest available row ID number. For
            % example, if 3 rows are added and the 2nd is deleted, then
            % adding new rows will fill the gap, i.e., [1; 3; 2; 4].

            if isInitialized(obj)
                % find the smallest available ID:
                curRowsNum = obj.AllIDs;
                possibleRows = 1:(length(curRowsNum)+1);
                % Find which row numbers can now be used
                nextID = setdiff(possibleRows, curRowsNum, 'stable');
                nextID = nextID(1); % Get first ID available
            else
                nextID = 1;
            end
        end

        function tableColumnName = getTableColumnName(obj)
            tableColumnName = obj.TableColumnMap.TableColumnName;
        end

        function out = getCompressedVectorString(~,in)
            % Try to establish a [0:N] pattern if possible
            if isempty(in)
                out = '[]'; % For display purposes, force the displayed string to be []
            elseif ~isscalar(in) && all(diff(in)==1)
                out = sprintf('[%d:%d]',in(1),in(end));
            else
                out = mat2str(in);
            end
        end
    end

    %% Protected methods
    methods (Access = protected)
        %% Table Buttons
        % Default rules for enabling/disabling table buttons. Derived
        % tables can implement their own special rules for any of the table
        % buttons.

        function buttonEnabled = enableAddButton(~)
            % Add button is always enabled by default

            buttonEnabled = true;
        end

        function buttonEnabled = enableRemoveButton(obj)
            % Remove button should be disabled when at least one of these
            % cases is true:
            % 1. No row is selected
            % 2. All rows are selected
            % 3. One of the selected rows is non editable

            selectedRows = obj.Selection(:);
            N = size(obj.Table.Data, 1);
            buttonEnabled = ~isempty(selectedRows) && N>size(selectedRows, 1) && all(obj.RowEditable(selectedRows));
        end

        function buttonEnabled = enableDuplicateButton(obj)
            % Duplicate button should be disabled when no row is selected

            buttonEnabled = ~isempty(obj.Selection);
        end

        %% Table data handling
        function data = getNewRowData(obj)
            % Returns the data content of the new row, adjusted with the
            % smallest available ID

            % Get the default data
            data = obj.NewRowDefaultData;

            % Get the smallest available ID
            nextID = appendNextAvailableRowName(obj);
            % Assign the new ID to the first column
            data{1,1} = nextID;
            % Construct the new Label with the new ID
            labelColIdx = (obj.TableColumnMap.ObjectPropName == "Label");
            if any(labelColIdx)
                data{1,labelColIdx} = replace(data{1,labelColIdx},digitsPattern,string(nextID));
            end
            obj.RowEditable(end+1) = true;
        end

        function selectTableEntry(obj, ~, ~)
            % Executes upon selection of table row(s)

            % Update Selection property
            obj.Selection = obj.Table.Selection;

            % Update ability for user interaction with table buttons
            updateButtonInteraction(obj);
        end

        function addTableEntry(obj, ~, ~)
            % Executes when the Add button is clicked. Expects a newRow implementation.

            % Get the new data
            data = [obj.Table.Data; getNewRowData(obj)];

            % Update the table data
            obj.Table.Data = data;

            % Update editability
            newRowIdx = height(data);
            updateEditability(obj, data);

            % Update ability for user interaction with table buttons
            updateButtonInteraction(obj);

            % Notify the app of the table update
            notify(obj, 'TableChanged', wirelessWaveformApp.internal.TableChangedEventData('Add',newRowIdx));
        end

        function [noOp, selectedRows] = removeTableEntry(obj, cfgID, ~)
            % Executes when the Remove button is clicked. Remove row and
            % possibly shift contents up. Also handle selections.

            noOp = false;

            % Do not remove anything if no rows or all rows are selected,
            % or when the table has only one row.
            if nargin > 1 && isnumeric(cfgID)
                selectedRows = find(ismember(obj.AllIDs, cfgID));
            else
                selectedRows = obj.Selection(:);
            end
            numRows = size(obj.Table.Data, 1);
            numSelectedRows = length(selectedRows);
            if (numRows == 1) || any(numSelectedRows == [0 numRows])
                noOp = true;
                return;
            end

            % Delete rows from table
            obj.Table.Data(selectedRows, :) = [];
            obj.CellEditable(selectedRows,:) = [];
            obj.RowEditable(selectedRows) = [];

            % Clear selections
            obj.Table.Selection = [];

            % Update ability for user interaction with table buttons
            updateButtonInteraction(obj);

            % Notify the app of the table update
            notify(obj, 'TableChanged', wirelessWaveformApp.internal.TableChangedEventData('Remove',selectedRows));
        end

        function duplicateTableEntry(obj, ~, ~)
            % Executes when the duplicate button is clicked. Add row and
            % duplicate content. Also handle selections.

            selectedRows = obj.Selection(:);
            sourceData = obj.Table.Data(selectedRows, :);
            numEntries = size(obj.Table.Data, 1);

            % By default, all columns of the table, but the first, are
            % duplicated. This is because the first column of the table is
            % the row heading that must be unique.
            firstCol = 2;

            % Create new table entries and copy selected rows content into
            % new ones
            numNewEntries = size(sourceData, 1);
            for r = 1:numNewEntries
                obj.Table.Data = [obj.Table.Data; getNewRowData(obj)];
                obj.Table.Data(end,firstCol:end) = sourceData(r,firstCol:end);
            end

            % Make sure that the read-only style is correctly assigned
            duplicatedRows = numEntries+(1:numNewEntries);
            obj.CellEditable(duplicatedRows,:) = obj.CellEditable(selectedRows,:);

            % Update ability for user interaction with table buttons
            updateButtonInteraction(obj);

            % Notify the app of the table update
            notify(obj, 'TableChanged', wirelessWaveformApp.internal.TableChangedEventData('Duplicate',selectedRows));
        end

        %% Table visualization
        function updateEditability(obj, data)
            % Update the editability of the cells for this table.
            % By default, the values stored in TableVisuals are used.
            % Derived classes can override this default behaviour.

            obj.CellEditable = repmat(obj.TableVisuals.CellEditable(:)', size(data,1), 1);
        end
    end

    %% Private methods
    methods (Access = private)
        %% Table grid
        function tableGrid = setTableGrid(obj, parent)
            % Create the grid layout to place each table into. The layout
            % consists of: title, table, 3 buttons on the right
            % By default, the table grid is not scrollable, as the
            % scrollbar should be in the uitable objects, to be as close as
            % possible to the unit the user is currently working on. Each
            % tab can implement their own exception to this default
            % behaviour.

            size4button = obj.ButtonWidth + 2;
            tableGrid = uigridlayout(parent);
            tableGrid.ColumnWidth = {'1x',size4button}; % 2 columns: title/table, buttons
            tableGrid.RowHeight = {17,size4button,size4button,size4button,'1x'}; % 5 rows: title, 3 buttons, table
            tableGrid.ColumnSpacing = 2;
            tableGrid.RowSpacing = 2;
            tableGrid.Padding = [1 1 1 1]; % Reduce the outside padding to better use the available real estate
            tableGrid.Scrollable = false;
            tableGrid.Tag = obj.Tag + "TableGrid";
        end

        %% Table data handling
        function setNewRowDefaultData(obj)
            % Defines the default data content of a new row

            % Get the default config object
            chWaveCfg = obj.DefaultConfig(1);

            % Link to the 1st defined values of obj.PropNameToLink
            for propId = 1:length(obj.PropNamesToLink)
                propCol = (obj.PropNamesToLink(propId) == obj.TableColumnMap.ObjectPropName);
                propVal = obj.Table.ColumnFormat{propCol};
                if strcmp(propVal{1}, '0') && ~isscalar(propVal)
                    % Avoid ID 0 as default
                    propVal = propVal(2); % try the next one
                end
                tableRow = cell(1, numel(propCol));
                tableRow{propCol} = propVal{1};
                [~, chWaveCfg{1}] = obj.TableColumnMap.Table2ObjMap{propCol}(obj, tableRow, propCol, chWaveCfg{1});
            end

            labelColIdx = (obj.TableColumnMap.ObjectPropName == "Label");
            if any(labelColIdx)
                % Construct the new Label (smallest available ID)
                nextID = appendNextAvailableRowName(obj);
                chWaveCfg{1}.Label = replace(chWaveCfg{1}.Label,digitsPattern,string(nextID));
            end

            obj.NewRowDefaultData = mapConfig2Table(obj, chWaveCfg, true, []);
        end

        function data = validateData(obj, data)
            % Validate the data input and, if valid, assign it back to
            % the data output
            chWaveCfg = repmat(obj.DefaultConfig(1),size(data,1),1);
            data = mapTable2Config(obj, data, chWaveCfg);
            obj.ConfigError = []; % No error, clear the internal error cached information
        end

        function tableCallback(obj, ~, event)
            % Called when a table value has been edited

            % Create event data to broadcast
            tableEvent = wirelessWaveformApp.internal.TableChangedEventData('ConfigChange',event.Indices(1));

            % Table input validation, by reusing the config object error messages
            data = obj.Table.Data;
            try
                data = validateData(obj, data);
            catch e
                % The wavegen configuration represented by the table has an
                % error.
                if matches(e.identifier,obj.CrossParameterErrorID)
                    % The error is caused by a cross-parameter issue.
                    % Update the internal error cached information and
                    % notify the listeners of the error.
                    obj.ConfigError = e;
                    tableEvent.Error = e;
                    notify(obj, 'TableChanged', tableEvent);
                    return
                else
                    % Throw an error popup and revert the invalid value
                    % to the previous one after the user closes the popup
                    throwErrorPopup(obj, e, Event=event);

                    if inputFromWorkspace(obj, event)
                        % The invalid value has been reverted to the
                        % previous one. No need to continue the execution
                        % of this method.
                        return
                    else
                        % The previous value might still be an invalid one,
                        % in the case the value was a variable that does
                        % not exist anymore, so run the check again.

                        data = obj.Table.Data;
                        try
                            data = validateData(obj, data);
                        catch e2
                            % Also the previous value was invalid. Update
                            % the internal error cached information and
                            % notify the listeners of the error.
                            obj.ConfigError = e2;
                            tableEvent.Error = e2;
                            notify(obj, 'TableChanged', tableEvent);
                            return
                        end
                    end
                end
            end
            obj.Table.Data = data; % Accept good values
            updateEditability(obj, data); % Update the cache of read-only properties for this table

            % Notify the app of the table update
            notify(obj, 'TableChanged', tableEvent);
            notifySpecial(obj, event.Indices); % Handles special notifications, if any
        end

        %% Table <--> Object mapping
        function data = mapConfig2Table(obj, chWaveCfg, allowUIChange, ids, cfgID)
            % Apply the channel/signal-specific configuration to the table data

            % Define local variables
            tableColName = obj.TableColumnMap.TableColumnName;
            chWavePropName = obj.TableColumnMap.ObjectPropName;
            obj2TableMapFcn = obj.TableColumnMap.Obj2TableMap;

            % Update table
            data = cell(numel(chWaveCfg), numel(tableColName));
            if nargin==4 || isempty(cfgID)
                cfgIndices = 1:numel(chWaveCfg);
            else
                cfgIndices = find(ismember(obj.AllIDs, cfgID));
            end
            for r = 1:numel(cfgIndices)
                rowIdx = cfgIndices(r);
                cfgIdx = rowIdx-cfgIndices(1)+1;
                for colIdx = 1:numel(tableColName)
                    % Set the table entries based on the object properties
                    if isempty(obj2TableMapFcn{colIdx})
                        % This is used to set the first ID column when
                        % there is no channel ID
                        if allowUIChange
                            % Assign the ID values in sorted order
                            data{rowIdx, colIdx} = rowIdx;
                        elseif ~isempty(ids)
                            % User given IDs
                            data{rowIdx, colIdx} = ids(rowIdx);
                        else
                            % Keep the current values of the row IDs
                            data{rowIdx, colIdx} = obj.AllIDs(rowIdx);
                        end
                    elseif strcmp(obj2TableMapFcn{colIdx}, "special")
                        % Special cases, if any, are handled by mapConfig2TableSpecial
                    else
                        data{rowIdx, colIdx} = obj2TableMapFcn{colIdx}(obj, chWaveCfg{cfgIdx}.(chWavePropName(colIdx)), chWaveCfg{cfgIdx}, rowIdx);
                    end
                end
            end

            % Handling of special cases, if any, is left to the derived
            % table
            data = mapConfig2TableSpecial(obj, data, chWaveCfg);
        end

        function [data, chWaveCfg] = mapTable2Config(obj, data, chWaveCfg)
            % Called when a table value has been edited or when the app
            % requires to map the table to the wavegen object

            % Define local variables
            tableColName = obj.TableColumnMap.TableColumnName;
            table2ObjMapFcn = obj.TableColumnMap.Table2ObjMap;

            % Table validation
            for cfgIdx = 1:size(data, 1)
                for colIdx = 1:length(tableColName)
                    % Set the properties of the configuration object based
                    % on the table entries
                    if isempty(table2ObjMapFcn{colIdx})
                        % Do nothing when there is no mapping function.
                        % For instance, this is used to set the first ID
                        % column when there is no channel ID and does not
                        % need to be validated
                    elseif strcmp(table2ObjMapFcn{colIdx}, "special")
                        % Special cases, if any, are handled by mapTable2ConfigSpecial
                    else
                        [data(cfgIdx,:), chWaveCfg{cfgIdx}] = table2ObjMapFcn{colIdx}(obj, data(cfgIdx, :), colIdx, chWaveCfg{cfgIdx});
                    end
                end
            end

            % Handling of special cases, if any, is left to the derived
            % table
            [data, chWaveCfg] = mapTable2ConfigSpecial(obj, data, chWaveCfg);
        end

        %% Buttons
        function addTableButtons(obj)
            % Create 1 add, 1 remove and 1 duplicate button for this table
            % (tag) except for SCS

            %% Add button
            obj.AddButton = uibutton('push', 'Parent', obj.TableGrid, 'Tag', obj.Tag + "AddButton", 'Tooltip', getMsgString(obj, 'addButtonTT', obj.Name), ...
                'Text', '', 'IconAlignment', 'center', 'ButtonPushedFcn', @obj.addTableEntry);
            matlab.ui.control.internal.specifyIconID(obj.AddButton, 'add', 16, 16);
            % Add button is in position (2,2)
            obj.AddButton.Layout.Row = 2;
            obj.AddButton.Layout.Column = 2;

            % Remove button
            obj.RemoveButton = uibutton('push', 'Parent', obj.TableGrid, 'Tag', obj.Tag + "RemoveButton", 'Tooltip', getMsgString(obj, 'removeButtonTT', obj.Name),...
                'Text', '', 'IconAlignment', 'center', 'Enable', 'off', 'ButtonPushedFcn', @obj.removeTableEntry);
            matlab.ui.control.internal.specifyIconID(obj.RemoveButton, 'delete', 16, 16);
            % Remove button is in position (3,2)
            obj.RemoveButton.Layout.Row = 3;
            obj.RemoveButton.Layout.Column = 2;

            % Duplicate button
            if obj.HasDuplicateButton
                obj.DuplicateButton = uibutton('push', 'Parent', obj.TableGrid, 'Tag', obj.Tag + "DuplicateButton", 'Tooltip', getMsgString(obj, 'duplicateButtonTT', obj.Name),...
                    'Text', '', 'IconAlignment', 'center', 'Enable', 'off', 'ButtonPushedFcn', @obj.duplicateTableEntry);
                matlab.ui.control.internal.specifyIconID(obj.DuplicateButton, 'copy', 16, 16);
                % Duplicate button is in position (4,2)
                obj.DuplicateButton.Layout.Row = 4;
                obj.DuplicateButton.Layout.Column = 2;
            end
        end

        function setButtonEnable(obj, buttonType, bool)
            % Enables/disables the button.
            % This is the protected interface to avoid sharing the whole
            % table buttons with the children
            switch buttonType
                case "add"
                    b = obj.AddButton;
                case "remove"
                    b = obj.RemoveButton;
                case "duplicate"
                    b = obj.DuplicateButton;
            end
            b.Enable = uiservices.logicalToOnOff(bool);
        end

        %% Table visualization
        function setTableErrorStyle(obj, rowIdx)
            % Set the style for error/conflict to the given row number, if
            % non-empty
            if ~isempty(rowIdx)
                addStyle(obj.Table,obj.ErrorRowStyle,'row',rowIdx);
            end
        end

        function resetTableStyle(obj, target)
            % Clear all table styles that apply to TARGET ('row', 'column', 'cell')
            % If no target is provided, clear all styles applied to the table

            styleCfg = obj.Table.StyleConfigurations;
            if ~isempty(styleCfg)
                if nargin < 2
                    % Clear all styles applied to this table
                    removeStyle(obj.Table);
                else
                    styleTargetCellIndex = find(styleCfg.Target==target);
                    if ~isempty(styleTargetCellIndex)
                        removeStyle(obj.Table,styleTargetCellIndex);
                    end
                end
            end
        end

        function updateNonEditableCellStyle(obj)
            % Update the style of non-editable (read-only) cells by
            % following these steps:
            % 1. Clear out the existing styles that might have been previous
            %    applied to the cells of the specified table(s)
            % 2. Apply the style to the now clean table(s)

            % Update the table style
            resetTableStyle(obj, 'cell');
            setNonEditableCellStyle(obj);
        end

        function setNonEditableCellStyle(obj, rowIdx)
            % Set the style and behavior of non-editable (read-only) cells.
            % If ROWIDX is nonempty, the read-only style and behaviour is
            % applied only to the row(s) specified by ROWIDX.

            if nargin < 2
                % By default, apply the style to the entire table
                rowIdx = [];
            end

            % Ensure the editability of the entire row respects the user's
            % setting of RowEditable
            cellEditable_local = obj.CellEditable;
            cellEditable_local(~obj.RowEditable, :) = false;

            % Find read-only cells and set the styles
            [r,c] = find(~cellEditable_local);
            if ~isempty(rowIdx)
                % Only the specified row(s) need updating
                idx = ~ismember(r,rowIdx);
                r(idx) = [];
                c(idx) = [];
            end
            if ~isempty(r)
                addStyle(obj.Table,obj.ReadOnlyCellStyle,   'cell',[r(:),c(:)]); % Gray out read-only cells
                addStyle(obj.Table,obj.ReadOnlyCellBehavior,'cell',[r(:),c(:)]); % Make read-only cells non-editable
            end
        end

        %% Miscellaneous
        function flag = isInitialized(obj)
            % Checks if the 5G table has been properly initialized
            flag = ~isempty(obj.Table) && isvalid(obj.Table) && ~isempty(obj.Table.Data);
        end

        function msg = getMsgString(~, id, varargin)
            msgID = ['nr5g:waveformApp:' id];
            msg = getString(message(msgID, varargin{:}));
        end
    end

    %% Table <--> Object Mapping Functions
    methods (Access = protected)
        % All direct mappings
        function [tableRow, chWaveCfg] = tab2ObjDirectMap(obj, tableRow, colIdx, chWaveCfg)
            if isnumeric(tableRow{colIdx}) && imag(tableRow{colIdx})==0
                % Ensure that no imaginary part is displayed if the input is real
                tableRow{colIdx} = real(tableRow{colIdx});
            end
            chWavePropName = obj.TableColumnMap.ObjectPropName;
            chWaveCfg.(chWavePropName{colIdx}) = tableRow{colIdx};
        end
        function tableVal = obj2TabDirectMap(~, propVal, ~, ~)
            tableVal = propVal;
        end

        % str2double
        function [tableRow, chWaveCfg] = str2doubleFcn(obj, tableRow, colIdx, chWaveCfg)
            chWavePropName = obj.TableColumnMap.ObjectPropName;
            chWaveCfg.(chWavePropName{colIdx}) = str2double(tableRow{colIdx});
        end

        % num2str
        function tableVal = num2strFcn(~, propVal, ~, ~)
            tableVal = num2str(propVal);
        end

        % mat2str
        function tableVal = mat2strFcn(obj, propVal, ~, ~)
            % Try to establish a [0:N] pattern if possible
            tableVal = getCompressedVectorString(obj, propVal);
        end

        % evalin
        function [tableRow, chWaveCfg] = evalFcn(obj, tableRow, colIdx, chWaveCfg)
            chWavePropName = obj.TableColumnMap.ObjectPropName;
            chWaveCfg.(chWavePropName{colIdx}) = evalin('base', tableRow{colIdx});
        end

        % Method to handle special cases of table --> object mapping
        function [data, chWaveCfg] = mapTable2ConfigSpecial(~, data, chWaveCfg)
            % No-op by default. Each derived table can specify their own
            % special-case handling, if any.
        end

        % Method to handle special cases of objet --> table mapping
        function data = mapConfig2TableSpecial(~, data, ~)
            % No-op by default. Each derived table can specify their own
            % special-case handling, if any.
        end

        % Method to handle special notifications, if any
        function notifySpecial(obj, changedCellIndex) %#ok<INUSD>
            % No-op by default. Each derived table can specify their own
            % special-case handling, if any.
        end
    end

    methods (Access = {?wirelessWaveformGenerator.nr5G_SSB_DataSource})
        function appendConfiguration(obj, chWaveCfg)
            numCfg = numel(chWaveCfg);
            for c = 1:numCfg
                % For each input configuration, add a new table row
                addTableEntry(obj);
            end

            % Once all the new rows are correctly added, apply the new
            % configurations to those rows alone
            ids = obj.AllIDs((end-numCfg+1):end);
            applyConfiguration(obj, chWaveCfg, AllowUIChange=false, ConfigIDs=ids);
        end

        function removeConfiguration(obj, cfgID)
            removeTableEntry(obj, cfgID);
        end
    end
end

function s = getTableString(tableName, def)
    % Get the strings associated to the input def of the given
    % table from the message catalog
    s = cell(1, numel(def));
    for i = 1:numel(def)
        s{i} = getString(message("nr5g:waveformApp:" + tableName + def(i)));
    end
end