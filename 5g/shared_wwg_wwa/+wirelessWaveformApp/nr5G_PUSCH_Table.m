classdef nr5G_PUSCH_Table < wirelessWaveformApp.nr5G_PXSCH_Table
    % Creation and handling of the PUSCH table in the PUSCH tab of Uplink 5G

    %   Copyright 2024 The MathWorks, Inc.

    % Constructor and public interface methods
    methods (Access = public)
        function obj = nr5G_PUSCH_Table(parent, defaultConfig, columnsToHide)

            arguments
                % uiobject that contains the table
                parent (1,1)
                % Cell array containing the app default nrWavegenPUSCHConfig
                % configuration object(s)
                defaultConfig cell
                % List of the names of the table columns to remove during
                % construction, if any (optional)
                columnsToHide (1,:) string = string.empty;
            end

            % Call base-class constructor
            columnsToHide = [columnsToHide, "VRBSet"];
            obj@wirelessWaveformApp.nr5G_PXSCH_Table(parent, defaultConfig, columnsToHide, ...
                                                     Name = getString(message("nr5g:waveformApp:PUSCHTableName")), ...
                                                     ModulationOptionsToHide = '1024QAM');
        end
    end

    methods (Access = protected)
        function updateEditability(obj, data)
            % PUSCH-specific addition to the updateEditability method in
            % the base class

            % Call base class method
            updateEditability@wirelessWaveformApp.nr5G_PXSCH_Table(obj, data);

            % PRBSet is read-only when Interlacing is true
            tableColName = getTableColumnName(obj);
            PRBSetColIdx = strcmp(tableColName, 'PRBSet');
            editableRows = ~strcmp(data(:, PRBSetColIdx), getString(message('nr5g:waveformApp:Interlaced')));
            obj.CellEditable( editableRows, PRBSetColIdx) = true;
            obj.CellEditable(~editableRows, PRBSetColIdx) = false;
        end
    end
end