classdef TDLSignalParameters <  wirelessAppContainer.Dialog

  %   Copyright 2024 The MathWorks, Inc.	

	properties (Hidden)
    TitleString = getString(message('nr5g:channelDesignerApp:TDLSignalParametersTitle'))
    configFcn = @struct
    configGenFcn = @struct
    configGenVar = 'signalParams';

    NRBType = 'numericEdit'
    NRBLabel
    NRBGUI

    SCSType = 'numericPopup'
    SCSDropDown = cellfun(@num2str, num2cell(15*2.^(0:6)), 'UniformOutput', false)'
    SCSLabel
    SCSGUI

    ECPType = 'charPopup'
    ECPDropDown = {'normal', 'extended'}
    ECPLabel
    ECPGUI

    SampleRateInfoType = 'numericText'
    SampleRateInfoLabel
    SampleRateInfoGUI
  end

  methods % constructor

    function obj = TDLSignalParameters(parent)
      obj@wirelessAppContainer.Dialog(parent); % call base constructor

      obj.SCSGUI.(obj.Callback)               = @(a,b) scsChanged(obj, []);
      obj.NRBGUI.(obj.Callback)               = @(a,b) nrbChanged(obj, []);
      obj.ECPGUI.(obj.Callback)               = @(a,b) updateSampleRate(obj, []);

      scsChanged(obj);
    end

    function props = displayOrder(~)
      props = {'NRB'; 'SCS'; 'ECP'; 'SampleRateInfo'};
    end

    function scsChanged(obj, ~)
      setVisible(obj, 'ECP', obj.SCS == 60);

      updateSampleRate(obj);
      
      layoutUIControls(obj);
    end
    function nrbChanged(obj, ~)
      try
        val = obj.NRB;
        validateattributes(val, {'numeric'}, {'real', 'scalar', 'integer', 'positive', '<=', 275}, '', 'number of resource blocks');

        updateSampleRate(obj);
      catch e
        obj.errorFromException(e);
      end
    end

    function sr = getSampleRate(obj)
      sr = obj.SampleRateInfo;
    end

    function restoreDefaults(obj)
      c = nrCarrierConfig; % same defaults
      obj.NRB = c.NSizeGrid;
      obj.SCS = c.SubcarrierSpacing;
      updateSampleRate(obj);

      obj.ECP = c.CyclicPrefix;
    end

    function updateSampleRate(obj, ~)
      info = nrOFDMInfo(obj.NRB, obj.SCS, "CyclicPrefix", obj.ECP);
      obj.SampleRateInfo = info.SampleRate;

      if ~isempty(obj.Parent.AppObj.pParameters.CurrentDialog)
        autoAnalyze(obj.Parent.AppObj.pParameters.CurrentDialog);
      % else initializing
      end
    end

    function addConfigCode(obj, sw)
      if obj.SCS ~= 60
        addcr(sw, ['s = nrOFDMInfo(' num2str(obj.NRB) ',  ' num2str(obj.SCS) ');']);
      else
        addcr(sw, ['s = nrOFDMInfo(' num2str(obj.NRB) ',  ' num2str(obj.SCS) ', CyclicPrefix=''' obj.ECP ''');']);
      end
      addcr(sw, 'Fs = s.SampleRate;');
      addcr(sw);
    end

    function str = getCatalogPrefix(~)
      str = 'nr5g:channelDesignerApp:';
    end
    
  end
end