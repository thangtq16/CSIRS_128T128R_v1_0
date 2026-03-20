classdef nr5G_ICGB_Dialog < handle
    % Intracell Guard Band Configuration extension for 5G Wireless Waveform Generator/Analyzer App
    
    %   Copyright 2023-2024 The MathWorks, Inc.
    
    properties (Hidden = true, Access = public)
        
        % GUI
        icgbConfigButton
        icgbFig
        
        % config
        icgbCache %{nrIntraCellGuardBandsConfig}
        
    end %properties Hidden public
    
    properties (Access = private)
        
        % GUI
        icgbConfigButtonGrid
        icgbFigGridLayout
        icgb15kHzTable
        icgb15kHzTitle
        icgb30kHzTable
        icgb30kHzTitle
        icgbTableGrid
        icgbButtonGrid
        icgbApplyButton
        icgbCloseButton
        
        % Cache & others
        icgb15kHzCache = cell(5,2);
        icgb30kHzCache = cell(5,2);
        icgbErrorFlag = false;
        icgbStrings = getICGBStrings();
        icgbTableNumRows = 5; % number of rows in config tables
        
    end %properties private
    
    methods (Access = public)
        
        function obj = nr5G_ICGB_Dialog()
            % Create the button and initialize cache
            
            % Creation of "Intra-cell Guard Bands" button and the button
            % grid layout to handle its placement in the main tab
            obj.icgbConfigButtonGrid = uigridlayout('Parent',obj.carrierGridLayout);
            obj.icgbConfigButtonGrid.ColumnWidth = {'fit'};
            obj.icgbConfigButtonGrid.RowHeight = {'fit'};
            obj.icgbConfigButtonGrid.Layout.Row = 2; % button is below SCS table
            obj.icgbConfigButtonGrid.Layout.Column = 1; % button is below SCS table
            obj.icgbConfigButtonGrid.Padding = [0 0 0 0];
            obj.icgbConfigButtonGrid.Tag = 'ICGBConfigButtonGrid';
            
            obj.icgbConfigButton = uibutton('Parent',     obj.icgbConfigButtonGrid, ...
                'Text',       obj.icgbStrings.ICGBButton, ...
                'Tooltip',    obj.icgbStrings.ICGBButtonTT, ...
                'Tag',        'ICGBConfigButton', ...
                'Position',   [10 10 200 22]); %ok<*MCNPN>
            obj.icgbConfigButton.ButtonPushedFcn = @(a,b,c) icgbConfigButtonCallback(obj);
            
        end %createICGB
        
        function applyICGBConfig(obj,cfg)
            % Apply configuration to cache
            % Called when opening saved session or openInGenerator
            % Called by applyConfiguration@nr5G_Full_Base_Dialog
            
            if isempty(obj.icgb15kHzCache)
                return; % use on app opening or new session or DL
            end
            
            icgbCfg = cfg.IntraCellGuardBands;
            obj.updateNumRows(icgbCfg);
            for i = 1:numel(icgbCfg)
                % Loop over all carriers configured with intracell
                % guardbands
                mat = icgbCfg{i}.GuardBandSize;
                if isempty(mat)
                    continue; % no-op for empty GuardBandSize
                end
                tmp = cell(obj.icgbTableNumRows,2);
                for row = 1:min(obj.icgbTableNumRows,size(mat,1))
                    % In matrix:  start | size
                    % In table:   size  | start
                    % Extract size and start from the matrix
                    tmp{row,1} = mat(row,2);
                    tmp{row,2} = mat(row,1);
                end
                prop = ['icgb' num2str(icgbCfg{i}.SubcarrierSpacing) 'kHzCache'];
                obj.(prop) = tmp;
            end
            obj.icgbCache = icgbCfg;
            
        end %applyICGBConfig
        
    end %methods public
    
    methods (Access = private)
        
        function icgbConfigButtonCallback(obj)
            % Callback for ICGB Configuration button
            % Create modal pop-up window for configuring ICGB
            
            obj.icgbConfigButton.Enable = false; % disable button
            
            % Bring Channel Bandwidth View to front, when pShowVisualizations is true
            if obj.getParent.AppObj.pShowVisualizations
                obj.bringChannelBandwidthViewToFrontICGB();
            end
            
            % Create window
            obj.icgbFig = uifigure('WindowStyle','modal');
            obj.icgbFig.Position(3) = 570;
            obj.icgbFig.Position(4) = 270;
            obj.icgbFigGridLayout = uigridlayout(obj.icgbFig,'Tag','icgbFigGridLayout');
            obj.icgbFigGridLayout.Scrollable = true;
            obj.icgbFigGridLayout.ColumnWidth = {'1x','fit','1x'}; % to put things in center
            obj.icgbFigGridLayout.RowHeight = {'1x','fit','fit','1x'}; % to put things in center
            obj.icgbFigGridLayout.RowSpacing = 10;
            obj.icgbFig.Name = obj.icgbStrings.ICGBButton;
            obj.icgbFig.Tag = 'ICGBConfigFig';
            obj.icgbFig.CloseRequestFcn = @(a,b,c) closeButtonCallback(obj); % window close callback
            
            % Create tables and titles
            obj.createICGBTables();
            
            % Create buttons
            obj.createICGBButtons();
            
            matlab.graphics.internal.themes.figureUseDesktopTheme(obj.icgbFig);
            
        end %icgbConfigButtonCallback
        
        function createICGBTables(obj)
            % Create ICGB config tables in the pop-up window
            
            import matlab.graphics.internal.themes.specifyThemePropertyMappings
            
            spec = {obj.icgbStrings.ICGBGuardBandSize,   'numeric', 'fit';
                obj.icgbStrings.ICGBGuardBandStart,  'numeric', 'fit';};
            
            % 2X2 grid containing 2 tables and 2 titles
            obj.icgbTableGrid = uigridlayout(obj.icgbFigGridLayout);
            obj.icgbTableGrid.ColumnWidth = {'fit','fit'};
            obj.icgbTableGrid.RowHeight = {17,145.3}; % title 5-row table
            obj.icgbTableGrid.ColumnSpacing = 20;
            obj.icgbTableGrid.RowSpacing = 2;
            obj.icgbTableGrid.Padding = [1 1 1 1];
            obj.icgbTableGrid.Scrollable = false;
            obj.icgbTableGrid.Tag = 'ICGBConfigTablesGrid';
            obj.icgbTableGrid.Layout.Row = 2;
            obj.icgbTableGrid.Layout.Column = 2;
            
            for i = 1:2
                scs = i*15;
                % create table
                tableName = ['icgb' num2str(scs) 'kHzTable'];
                obj.(tableName) = uitable('Parent',               obj.icgbTableGrid, ...
                    'ColumnName',           spec(1:end,1), ...
                    'ColumnFormat',         spec(1:end,2)', ...
                    'ColumnWidth',          spec(1:end,3)', ...
                    'ColumnEditable',       true, ...
                    'CellEditCallback',     @(a,b) icgbConfigTableCallback(obj,scs), ...
                    'RowName',              'numbered', ...
                    'Tag',                  ['icgb' num2str(scs) 'kHzTable'], ...
                    'RearrangeableColumns', false, ...
                    'RowStriping',          false, ...
                    'Interruptible',        'off'); %#ok<*MCNPN>
                obj.(tableName).Layout.Row = 2;
                obj.(tableName).Layout.Column = i;
                obj.mapCache2TableICGB(scs);
                specifyThemePropertyMappings(obj.(tableName),'BackgroundColor','--mw-backgroundColor-input');
                % add title
                titleName = ['icgb' num2str(scs) 'kHzTitle'];
                titleText = obj.icgbStrings.(titleName);
                obj.(titleName) = uilabel('Parent',     obj.icgbTableGrid, ...
                    'Text',       titleText, ...
                    'FontWeight', 'Bold');
                obj.(titleName).Layout.Row = 1;
                obj.(titleName).Layout.Column = i;
                obj.(titleName).HorizontalAlignment = 'center';
                
            end
            
        end %createICGBTable
        
        function createICGBButtons(obj)
            % Create Apply and Close button in the pop-up window
            
            % 1X3 grid containing empty space and 2 buttons
            obj.icgbButtonGrid = uigridlayout(obj.icgbFigGridLayout);
            obj.icgbButtonGrid.ColumnWidth = {'1x','fit','fit'};
            obj.icgbButtonGrid.RowHeight = {'fit'};
            obj.icgbButtonGrid.ColumnSpacing = 10;
            obj.icgbButtonGrid.RowSpacing = 2;
            obj.icgbButtonGrid.Padding = [1 1 1 1];
            obj.icgbButtonGrid.Scrollable = false;
            obj.icgbButtonGrid.Tag = 'ICGBConfigButtonsGrid';
            obj.icgbButtonGrid.Layout.Row = 3;
            obj.icgbButtonGrid.Layout.Column = 2;
            
            % Apply button
            applyText = obj.icgbStrings.ApplyText;
            obj.icgbApplyButton = uibutton('Parent',   obj.icgbButtonGrid, ...
                'Text',     applyText, ...
                'Tag',      'icgbConfigApplyBtn');
            obj.icgbApplyButton.Layout.Row = 1;
            obj.icgbApplyButton.Layout.Column = 2;
            obj.icgbApplyButton.ButtonPushedFcn = @(a,b,c) applyButtonCallback(obj);
            
            % Close button
            closeText = obj.icgbStrings.CloseText;
            obj.icgbCloseButton = uibutton('Parent',   obj.icgbButtonGrid, ...
                'Text',     closeText, ...
                'Tag',      'icgbConfigCloseBtn');
            obj.icgbCloseButton.Layout.Row = 1;
            obj.icgbCloseButton.Layout.Column = 3;
            obj.icgbCloseButton.ButtonPushedFcn = @(a,b,c) closeButtonCallback(obj);
            
        end %createICGBButtons
        
        function icgbConfigTableCallback(obj,scs)
            % ICGB configuration table callback - flush empty, nan and
            % non-numeric input to empty
            % Shared by 15kHz and 30kHz
            
            tableName = ['icgb' num2str(scs) 'kHzTable'];
            
            for i = 1:obj.icgbTableNumRows
                
                for j = 1:2
                    
                    if any(isnan(obj.(tableName).Data{i,j})) || ...   % NaN input or []
                            ~isnumeric(obj.(tableName).Data{i,j})          % non-numeric input
                        
                        obj.(tableName).Data{i,j} = [];
                        
                    else % numeric input
                        
                        if ~imag(obj.(tableName).Data{i,j})
                            % To prevent real values from being automatically
                            % converted to complex when a complex value was
                            % previously entered, also for better display
                            obj.(tableName).Data{i,j} = real(obj.(tableName).Data{i,j});
                        end
                        
                    end
                end
                
            end
            
        end %icgbConfigTableCallback
        
        function applyButtonCallback(obj)
            % 'Apply' button in pop-up window button pushed callback
            
            % validate configuration by trying to apply
            obj.icgbErrorFlag = false; % reset error flag
            [mat15,msg15] = obj.mapTable2MatICGB(15);
            [mat30,msg30] = obj.mapTable2MatICGB(30);
            msg = combineICGBErrorMessage(msg15,msg30);
            if ~isempty(msg) % invalid config
                throwErrorPopup(obj, msg, Figure=obj.icgbFig); % error out from window
                obj.icgbErrorFlag = true;
                return; % do not update cache
            else % valid config
                % update cache
                obj.mapICGBConfig2Cache(mat15,mat30);
                
                % update grid & channel bandwidth view
                updateGrid(obj);
                updateChannelBandwidthView(obj);
                
                % map configuration table content to table cache
                obj.mapTable2CacheICGB();
                
            end
            
        end %applyButtonCallback
        
        function closeButtonCallback(obj)
            % Pop-up window close request callback
            
            if ~obj.hasUnappliedICGBConfig() % no unapplied changes - close directly
                
                obj.closeAndEmptyICGB();
                
            else % has unapplied changes - pop-up uiconfirm
                
                confirmText = obj.icgbStrings.ConfirmText;
                confirmTitle = obj.icgbStrings.ConfirmTitle;
                options = {obj.icgbStrings.Yes, obj.icgbStrings.No, obj.icgbStrings.OpCancel};
                selection = uiconfirm(obj.icgbFig, confirmText, confirmTitle, 'Options', options);
                switch selection
                    case obj.icgbStrings.Yes
                        % apply configurations
                        obj.applyButtonCallback();
                        if ~obj.icgbErrorFlag
                            obj.closeAndEmptyICGB(); % do not close window if there is error
                        end
                    case obj.icgbStrings.No
                        obj.closeAndEmptyICGB();
                    case obj.icgbStrings.OpCancel
                        return;
                end
                
            end
            
        end %closeButtonCallback
        
        function mapICGBConfig2Cache(obj,mat15,mat30)
            % Construct nrIntraCellGuardBandsConfig object and map to cache
            % assuming mat15 and mat30 are valid GuardBandSize matrices
            
            c15 = nrIntraCellGuardBandsConfig('SubcarrierSpacing',15,'GuardBandSize',mat15);
            
            c30 = nrIntraCellGuardBandsConfig('SubcarrierSpacing',30,'GuardBandSize',mat30);
            
            % construct configuration using only non-empty sizes if possible
            if isempty(mat15) && ~isempty(mat30)
                icgb = {c30};
            elseif isempty(mat30) % this guarantees same default when generating script in App
                icgb = {c15};
            else
                icgb = {c15,c30};
            end
            
            % map to cache
            obj.icgbCache = icgb;
            
        end %mapICGBConfig2Cache
        
        function [mat,msg] = mapTable2MatICGB(obj,scs)
            % Construct the GuardBandSize matrix for the scs from config table
            % For table-specific error handling
            
            mat = [];
            materr = []; % For handling error messages
            msg = [];
            for idx = 1:obj.icgbTableNumRows
                vec = obj.getRowFromTableICGB(scs, idx);
                mat = [mat; vec]; %#ok<AGROW>
                vecerr = obj.getRowFromTableICGB(scs,idx,true);
                materr = [materr; vecerr]; %#ok<AGROW>
            end
            
            % create nrIntraCellGuardBandsConfig objects
            % reuse nrIntraCellGuardBandsConfig error messages
            c = nrIntraCellGuardBandsConfig('SubcarrierSpacing',scs);
            try
                % materr is equivalent to mat here, but use materr to have
                % correct row index in error messages
                c.GuardBandSize = materr;
            catch e
                strName = ['SCS' num2str(scs) 'kHz'];
                msg = [obj.icgbStrings.(strName) ' ' e.message];
                mat = []; % return empty matrix for invalid input
            end
            
        end %mapTable2MatICGB
        
        function mapCache2TableICGB(obj,scs)
            % Get the cached ICGB data and display in config table(s)
            
            for scsVal = scs
                cacheName = ['icgb' num2str(scsVal) 'kHzCache']; % cache name
                tableName = ['icgb' num2str(scsVal) 'kHzTable']; % table name
                if isempty(obj.(cacheName))
                    obj.(cacheName) = cell(5,2);
                end
                obj.(tableName).Data = obj.(cacheName);
            end
            
        end %mapCache2TableICGB
        
        function mapTable2CacheICGB(obj)
            % Map contents of ICGB configuration tables to table cache
            
            for i = 1:2
                
                scs = i*15;
                
                cacheName = ['icgb' num2str(scs) 'kHzCache']; % cache name
                tableName = ['icgb' num2str(scs) 'kHzTable']; % table name
                for j = 1:obj.icgbTableNumRows
                    if ~isempty(obj.(tableName).Data{j,1}) && ~isempty(obj.(tableName).Data{j,2})
                        obj.(cacheName){j,1} = obj.(tableName).Data{j,1};
                        obj.(cacheName){j,2} = obj.(tableName).Data{j,2};
                    else
                        % has empty cell - treat guard band as empty
                        obj.(cacheName){j,1} = [];
                        obj.(cacheName){j,2} = [];
                    end
                end
                
            end
            
        end %mapTable2CacheICGB
        
        function closeAndEmptyICGB(obj)
            % Close pop-up window and empty all related ui objects
            
            delete(obj.icgbFig);
            obj.icgbFig = [];
            obj.icgbFigGridLayout = [];
            obj.icgb15kHzTable = [];
            obj.icgb15kHzTitle = [];
            obj.icgb30kHzTable = [];
            obj.icgb30kHzTitle = [];
            obj.icgbTableGrid = [];
            obj.icgbButtonGrid = [];
            obj.icgbApplyButton = [];
            obj.icgbCloseButton = [];
            % Enable button
            obj.icgbConfigButton.Enable = true;
            
        end %closeAndEmptyICGB
        
        function bringChannelBandwidthViewToFrontICGB(obj)
            % Bring Channel Bandwidth View to front
            % Called when opening the ICGB window
            
            obj.setVisualState('Channel Bandwidth View',true);
            appObj = obj.getParent.AppObj;
            document = appObj.AppContainer.getDocument(getTag(appObj) + "DocumentGroup", 'Channel Bandwidth View');
            document.Selected = true;
            
        end %bringChannelBandwidthViewToFrontICGB
        
        function flag = hasUnappliedICGBConfig(obj)
            % Flag for unapplied changes in the config tables
            
            if ~isempty(obj.icgb15kHzTable) && ~isempty(obj.icgb30kHzTable)
                flag = hasDifferentICGB(obj.icgb15kHzCache,obj.icgb15kHzTable.Data) || ...
                    hasDifferentICGB(obj.icgb30kHzCache,obj.icgb30kHzTable.Data);
            else % table not created
                flag = false;
            end
            
        end %hasUnappliedICGBConfig
        
        function row = getRowFromTableICGB(obj,scs,idx,errorMsg)
            % Get one row for GuardBandSize matrix from config table
            
            if nargin<4
                errorMsg = false;
            end
            
            tableName = ['icgb' num2str(scs) 'kHzTable'];
            
            % size and start are swapped between matrix and table
            if isempty(obj.(tableName).Data{idx,1}) && ~isempty(obj.(tableName).Data{idx,2}) % treat empty cell as NaN
                row = [obj.(tableName).Data{idx,2} NaN];
            elseif ~isempty(obj.(tableName).Data{idx,1}) && isempty(obj.(tableName).Data{idx,2}) % treat empty cell as NaN
                row = [NaN obj.(tableName).Data{idx,1}];
            elseif isempty(obj.(tableName).Data{idx,1}) && isempty(obj.(tableName).Data{idx,2}) % both empty: guard band doesn't exist
                if errorMsg % Trying to construct error messages
                    row = [0 0]; % use 0 size to make sure row index is correct in error messages
                else
                    row = []; % use [] to avoid unnecessary change in the table
                end
            else
                row = [obj.(tableName).Data{idx,2} obj.(tableName).Data{idx,1}];
            end
            
        end %getRowFromTableICGB
        
        function updateNumRows(obj,icgbCfg)
            % Update the number of rows in the config tables according to cfg
            
            nRow = 5;
            for i = 1:numel(icgbCfg)
                % By default, have 5 rows in the table
                % If the loaded session / cfg that calls openInGenerator
                % has more than 5 guard bands, load all of them
                nRow = max(nRow,size(icgbCfg{i}.GuardBandSize,1));
            end
            obj.icgbTableNumRows = nRow;
            obj.icgb15kHzCache = cell(obj.icgbTableNumRows,2);
            obj.icgb30kHzCache = cell(obj.icgbTableNumRows,2);
            
        end %updateNumRow
        
    end %methods private
    
end %classdef

%% Local functions

function s = getICGBStrings()
    % Get ICGB strings from catalogue
    
    def = {'icgb15kHzTitle' 'icgb30kHzTitle' 'ICGBGuardBandStart' 'ICGBGuardBandSize' ...
        'ICGBButton' 'ICGBButtonTT' 'ApplyText' 'CloseText' 'ConfirmText' ...
        'ConfirmTitle' 'Yes' 'No' 'OpCancel' 'SCS15kHz' 'SCS30kHz'};
    
    for i = 1:numel(def)
        s.(def{i}) = getString(message(['nr5g:waveformApp:' def{i}]));
    end
    
end

function msg = combineICGBErrorMessage(msg15,msg30)
    % Combine error message of configuring ICGB
    
    msg = [];
    if ~isempty(msg15) && ~isempty(msg30) % both error
        msg = sprintf("%s\n%s",msg15,msg30);
    elseif isempty(msg15) && ~isempty(msg30) % only 30kHz error
        msg = msg30;
    elseif ~isempty(msg15) && isempty(msg30) % only 15kHz error
        msg = msg15;
    end
    
end

function flag = hasDifferentICGB(c1,c2)
    % Compare ICGB data, treating [] and [0 0] as equal as in
    % getRowFromTableICGB
    
    c1(cellfun(@isempty,c1)) = {0};
    c2(cellfun(@isempty,c2)) = {0};
    
    flag = ~isequal(c1,c2);
    
end
