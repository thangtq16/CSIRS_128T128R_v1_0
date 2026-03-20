classdef nr5G_Dialog < wirelessAppContainer.Dialog & ...
        wirelessWaveformApp.internal.UpdateDialogProperty & ...
        wirelessWaveformApp.internal.UpdateAppStatus
    % Base class for the 5G dialogs shared between WWG and WWA, providing
    % the bridge between wirelessAppContainer.Dialog and the 5G-specific
    % usage.

    %   Copyright 2024 The MathWorks, Inc.

    properties (Dependent, Access = public)
        CurrentDialog
    end

    properties (Hidden)
        configFcn = @struct
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_Dialog(parent, fig, cfg)

            arguments
                parent (1,1)
                fig (1,1) {mustBeValid(fig)}
                cfg (1,1)
            end

            % Call base constructors
            obj@wirelessWaveformApp.internal.UpdateDialogProperty(parent, cfg);
            obj@wirelessAppContainer.Dialog(parent, fig);
        end

        function createUIControls(obj)
            % Extend the base class behaviour by adding default callbacks

            % Call base class method
            createUIControls@wirelessAppContainer.Dialog(obj);

            % Add default callback
            props = displayOrder(obj);
            for idx = 1:length(props)
                propName = props{idx};
                controlGUI = obj.([propName 'GUI']);
                if ~isa(controlGUI,'matlab.ui.control.Label') && ~isa(controlGUI,'matlab.ui.control.Button')
                    controlGUI.(obj.Callback) = @(src,evnt) updateProperty(obj,src,evnt,propName);
                end
            end
        end

        function adjustSpec(obj)
            % customization given the max label length of the side panels
            % controls
            obj.LabelWidth = 135;
        end

        function str = getCatalogPrefix(~)
            str = 'nr5g:waveformApp:';
        end
    end

    % Getters/setters
    methods
        function dlg = get.CurrentDialog(obj)
            dlg = obj.Parent.AppObj.pParameters.CurrentDialog;
        end
    end

end

function mustBeValid(in)
    % Check that the input is a valid handle object
    if ~isvalid(in)
        error("Input must be a valid handle object.");
    end
end