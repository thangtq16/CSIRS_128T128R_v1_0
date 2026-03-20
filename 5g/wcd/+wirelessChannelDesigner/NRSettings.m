classdef NRSettings <  wirelessAppContainer.Dialog

  %   Copyright 2024 The MathWorks, Inc.

	properties (Hidden)
    TitleString = getString(message('nr5g:channelDesignerApp:NRSettingsTitle'))
    configFcn = @struct
    configGenFcn = @struct
    configGenVar = 'plotSettings';
    
    DurationType = 'numericEdit'
    DurationLabel
    DurationGUI
  end

  methods % constructor

    function obj = NRSettings(parent)
        obj@wirelessAppContainer.Dialog(parent); % call base constructor
        
        obj.DurationGUI.(obj.Callback)      = @(a,b) durationChanged(obj, []);
    end

    function restoreDefaults(obj)
      obj.Duration = 1;
    end

    function durationChanged(obj, ~)
      try
        val = obj.Duration;
        validateattributes(val, {'numeric'}, {'real', 'positive', 'integer', 'scalar', 'finite'}, '', 'duration (in Analysis Settings)');
      catch e
        obj.Parent.AppObj.pThrewValidationError = true;
        if strcmp(obj.Parent.AppObj.pPlotSettingsFig.Visible, 'on')
          obj.errorFromException(e, '', obj.Parent.AppObj.pPlotSettingsFig);
        else
          obj.errorFromException(e);
        end
      end
    end

    function props = displayOrder(~)
      props = {'Duration'};
    end

    function str = getCatalogPrefix(~)
      str = 'nr5g:channelDesignerApp:';
    end
    
  end
end