classdef TDLMIMODialog < wirelessChannelDesigner.MIMODialog
  %

  %   Copyright 2024-2025 The MathWorks, Inc.

	properties (Hidden)
    MIMOCorrelationType = 'charPopup'
    MIMOCorrelationDropDown = {'Low', 'Medium', 'Medium-A', 'UplinkMedium', 'High', 'Custom'}
    MIMOCorrelationLabel
    MIMOCorrelationGUI

    PolarizationType = 'charPopup'
    PolarizationDropDown = {'Co-Polar', 'Cross-Polar', 'Custom'}
    PolarizationLabel
    PolarizationGUI

    TransmitPolarizationAnglesType = 'numericEdit'
    TransmitPolarizationAnglesLabel
    TransmitPolarizationAnglesGUI

    ReceivePolarizationAnglesType = 'numericEdit'
    ReceivePolarizationAnglesLabel
    ReceivePolarizationAnglesGUI

    XPRType = 'numericEdit'
    XPRLabel
    XPRGUI

    TransmissionDirectionType = 'charPopup'
    TransmissionDirectionDropDown = {'Downlink', 'Uplink'}
    TransmissionDirectionLabel
    TransmissionDirectionGUI
  end

  methods % constructor

    function obj = TDLMIMODialog(parent)
        obj@wirelessChannelDesigner.MIMODialog(parent); % call base constructor

        % dropdown callbacks for conditional visibility:
        obj.MIMOCorrelationGUI.(obj.Callback)             = @(a,b) mimoCorrChanged(obj, []);
        obj.PolarizationGUI.(obj.Callback)                = @(a,b) polarizationChanged(obj, []);

        obj.TransmitPolarizationAnglesGUI.(obj.Callback)  = @(a,b) txAnglesChanged(obj, []);
        obj.ReceivePolarizationAnglesGUI.(obj.Callback)   = @(a,b) rxAnglesChanged(obj, []);

        obj.XPRGUI.(obj.Callback)                         = @(a,b) xprChanged(obj, []);
        obj.TransmissionDirectionGUI.(obj.Callback)       = @(a,b) directionChanged(obj, []);
    end

    function adjustSpec(obj)
      obj.TitleString = getString(message('nr5g:channelDesignerApp:TDLMIMOTitle'));
      obj.configFcn = @nrTDLChannel;
      obj.configGenFcn = @nrTDLChannel;
      obj.configGenVar = 'tdlChan';
    end

    function restoreDefaults(obj)
      chan = feval(obj.configFcn);
      
      props = displayOrder(obj);
      commMIMOProp = {'SpatialCorrelationSpecification'};
      for idx = 1:numel(props)
        prop = props{idx};
        if ~strcmp(prop, commMIMOProp)
          obj.(prop) = chan.(prop);
        end
      end
    end

    function setupDialog(obj)
      mimoCorrChanged(obj);
      polarizationChanged(obj);
    end
    
    function props = displayOrder(~)
      props = {'MIMOCorrelation'; 'Polarization'; 'TransmissionDirection'; ...
        'NumTransmitAntennas'; 'NumReceiveAntennas'; 'TransmitCorrelationMatrix'; 'ReceiveCorrelationMatrix';
        'TransmitPolarizationAngles'; 'ReceivePolarizationAngles'; 'XPR'; 'SpatialCorrelationMatrix'};
    end

    function props = props2ExcludeFromConfig(obj)
      isCustomCorr = strcmp(obj.MIMOCorrelation, 'Custom');
      isCustomPolar = strcmp(obj.Polarization, 'Custom');
      isCrossPolar = strcmp(obj.Polarization, 'Cross-Polar');

      props = {'SpatialCorrelationSpecification'};
      if isCustomCorr
        props = [props, 'TransmissionDirection', 'NumReceiveAntennas'];
      end
      if isCustomCorr && ~isCustomPolar
        props = [props, 'NumTransmitAntennas'];
      end
      if ~isCustomCorr || isCustomPolar
        props = [props, 'TransmitCorrelationMatrix', 'ReceiveCorrelationMatrix'];
      end
      if ~isCustomCorr || ~isCrossPolar
        props = [props 'TransmitPolarizationAngles', 'ReceivePolarizationAngles', 'XPR'];
      end
      if ~isCustomCorr || ~isCustomPolar
        props = [props, 'SpatialCorrelationMatrix'];
      end
    end

    function cfg = getConfiguration(obj)
      % simply get properties, don't do custom logic from immediate superclass
      cfg = getConfiguration@wirelessChannelDesigner.ChannelConfigurationDialog(obj);
    end
    function applyConfiguration(obj, cfg)
      % don't do custom actions of MIMODialog superclass, which touches 2ndary dialogs      
      applyConfiguration@wirelessChannelDesigner.ChannelConfigurationDialog(obj, cfg);

      updateVisibilities(obj);
      layoutUIControls(obj);
    end

    function mimoCorrChanged(obj, ~)
      updateVisibilities(obj);
      
      updateNumTxRx(obj.Parent.CurrentDialog);

      layoutUIControls(obj);

      autoAnalyze(obj.Parent.AppObj.pParameters.CurrentDialog);
    end

    function polarizationChanged(obj, ~)
      updateVisibilities(obj);

      updateNumTxRx(obj.Parent.CurrentDialog);

      layoutUIControls(obj);

      autoAnalyze(obj.Parent.CurrentDialog);
    end
    function updateVisibilities(obj)
      
      noShowProps = props2ExcludeFromConfig(obj);
      props = {'TransmissionDirection', 'NumReceiveAntennas', 'NumTransmitAntennas', ...
        'TransmitCorrelationMatrix', 'ReceiveCorrelationMatrix',  'TransmitPolarizationAngles', ...
        'ReceivePolarizationAngles', 'XPR', 'SpatialCorrelationMatrix'};
      for idx = 1:numel(props)
        setVisible(obj, props{idx}, ~contains(props{idx}, noShowProps));
      end
    end

    function spatialChanged(~, ~)
      % Don't do any gets to SpatialCorrelationSpecification, it
      % doesn't show in this dialog, it adds no value as it is only internally meaningful
    end

    function txAnglesChanged(obj, ~)
      try
        val = obj.TransmitPolarizationAngles;
        validateattributes(val, {'numeric'}, {'real', 'row'}, '', 'transmit polarization angles');
      catch e
        obj.errorFromException(e);
      end

      autoAnalyze(obj.Parent.AppObj.pParameters.CurrentDialog);
    end
    function rxAnglesChanged(obj, ~)
      try
        val = obj.ReceivePolarizationAngles;
        validateattributes(val, {'numeric'}, {'real', 'row'}, '', 'receive polarization angles');
      catch e
        obj.errorFromException(e);
      end

      autoAnalyze(obj.Parent.AppObj.pParameters.CurrentDialog);
    end
    function xprChanged(obj, ~)
      try
        val = obj.XPR;
        validateattributes(val, {'numeric'}, {'real', 'row'}, '', 'XPR');
      catch e
        obj.errorFromException(e);
      end

      autoAnalyze(obj.Parent.AppObj.pParameters.CurrentDialog);
    end
    function directionChanged(obj, ~)
      autoAnalyze(obj.Parent.AppObj.pParameters.CurrentDialog);
    end

    function addConfigCode(obj, sw)
    % MIMO properties

      addcr(sw, ['% ' getString(message('comm:channelDesigner:MIMOTitle'))]);
      addcr(sw, [obj.configGenVar '.MIMOCorrelation = ''' obj.MIMOCorrelation ''';']);
      addcr(sw, [obj.configGenVar '.Polarization = ''' obj.Polarization ''';']);
      
      isCustomCorr = strcmp(obj.MIMOCorrelation, 'Custom');
      isCustomPolar = strcmp(obj.Polarization, 'Custom');
      isCrossPolar = strcmp(obj.Polarization, 'Cross-Polar');

      if ~isCustomCorr
        addcr(sw, [obj.configGenVar '.TransmissionDirection = ''' obj.TransmissionDirection ''';']);
        addcr(sw, [obj.configGenVar '.NumReceiveAntennas = '  num2str(obj.NumReceiveAntennas) ';']);
      end
      if ~isCustomCorr || (isCustomCorr && isCustomPolar)
        addcr(sw, [obj.configGenVar '.NumTransmitAntennas = '  num2str(obj.NumTransmitAntennas) ';']);
      end
      if isCustomCorr && ~isCustomPolar
        addcr(sw, [obj.configGenVar '.TransmitCorrelationMatrix = ' mat2str(obj.TransmitCorrelationMatrix) ';']);
        addcr(sw, [obj.configGenVar '.ReceiveCorrelationMatrix = '  mat2str(obj.ReceiveCorrelationMatrix) ';']);
      end
      if isCustomCorr && isCrossPolar
        addcr(sw, [obj.configGenVar '.TransmitPolarizationAngles = ' mat2str(obj.TransmitPolarizationAngles) ';']);
        addcr(sw, [obj.configGenVar '.ReceivePolarizationAngles = '  mat2str(obj.ReceivePolarizationAngles) ';']);
        addcr(sw, [obj.configGenVar '.XPR = '                        num2str(obj.XPR) ';']);
      end
     
      if isCustomCorr && isCustomPolar
        addcr(sw, [obj.configGenVar '.SpatialCorrelationMatrix = ' mat2str(obj.SpatialCorrelationMatrix) ';']);
      end

      addcr(sw);
    end

    function str = getCatalogPrefix(~)
      str = 'nr5g:channelDesignerApp:';
    end
  end
end