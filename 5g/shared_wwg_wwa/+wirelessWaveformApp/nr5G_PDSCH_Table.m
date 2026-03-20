classdef nr5G_PDSCH_Table < wirelessWaveformApp.nr5G_PXSCH_Table
    % Creation and handling of the PDSCH table in the PDSCH tab of Downlink 5G

    %   Copyright 2024 The MathWorks, Inc.

    % Constructor and public interface methods
    methods (Access = public)
        function obj = nr5G_PDSCH_Table(parent, defaultConfig, columnsToHide)

            arguments
                % uiobject that contains the table
                parent (1,1)
                % Cell array containing the app default nrWavegenPDSCHConfig
                % configuration object(s)
                defaultConfig cell
                % List of the names of the table columns to remove during
                % construction, if any (optional)
                columnsToHide (1,:) string = string.empty;
            end

            % Call base-class constructor
            columnsToHide = [columnsToHide, "PRBSet", "TransformPrecoding"];
            obj@wirelessWaveformApp.nr5G_PXSCH_Table(parent, defaultConfig, columnsToHide, ...
                                                     Name = getString(message("nr5g:waveformApp:PDSCHTableName")), ...
                                                     ModulationOptionsToHide = 'pi/2-BPSK');
        end
    end
end