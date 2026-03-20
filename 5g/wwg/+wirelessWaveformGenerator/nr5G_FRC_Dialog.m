classdef nr5G_FRC_Dialog < wirelessWaveformGenerator.nr5G_Presets_Dialog
    %

    %   Copyright 2019-2024 The MathWorks, Inc.

    properties (SetAccess = private, GetAccess = protected)
        pErrorCache
        pErrorProp
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_FRC_Dialog(parent, isDownlink)
            obj@wirelessWaveformGenerator.nr5G_Presets_Dialog(parent); % call base constructor

            obj.isDownlink = isDownlink;
        end

        function config = getConfigurationForSave(obj)
            config.waveform   = getConfigurationForSave@wirelessWaveformGenerator.nr5G_Presets_Dialog(obj);
            config.generation = getConfigurationForSave(obj.Parent.GenerationDialog);
            config.filtering  = getConfigurationForSave(obj.Parent.FilteringDialog);
        end

        function waveform = generateWaveform(obj)
            waveconfig = getConfiguration(obj);
            [waveform, obj.gridSet] = nrWaveformGenerator(waveconfig);
            updateInfo(obj);
        end

        function c = getSourceClass(~)
            c = 'wirelessAppContainer.sources.BitSourceDialog';
        end

        % Update the cached error
        function updateErrorCache(obj,e,prop)
            % For UL: This method MUST be called before updating the PUSCH dialog cache
            
            % When prop is empty, clear all errors
            if isempty(prop)
                % Clear all errors
                obj.pErrorCache = {};
                obj.pErrorProp = {};
            else
                idx = find(strcmpi(obj.pErrorProp,prop));
                if isempty(idx)
                    if ~isempty(e)
                        % New error found - add to cache
                        obj.pErrorCache{end+1} = e;
                        obj.pErrorProp{end+1} = prop;
                    end
                else
                    % This property has previous error - either clear or
                    % overwrite with new error by stacking to the end
                    obj.pErrorCache(idx) = [];
                    obj.pErrorProp(idx) = [];
                    if isempty(e)
                        % Error with this property is fixed - clear from
                        % cache
                        % Update banner to show previous error, if any
                        if ~isempty(obj.pErrorCache)
                            obj.updateConfigDiagnostic(obj.pErrorCache{end}.message);
                        else
                            obj.updateConfigDiagnostic("");
                        end

                    else
                        % Error with this property still exists - stack to
                        % the end of the cache
                        % No need to update banner as it has already
                        % been updated by property callback
                        obj.pErrorCache{end+1} = e;
                        obj.pErrorProp{end+1} = prop;
                    end
                end
            end

        end

    end

end
