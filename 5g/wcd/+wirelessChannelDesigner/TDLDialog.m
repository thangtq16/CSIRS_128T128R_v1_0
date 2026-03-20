classdef TDLDialog < wirelessChannelDesigner.MIMOFadingDialog & wirelessChannelDesigner.MIMOChannelConfigurationDialog
  %

  %   Copyright 2024-2025 The MathWorks, Inc. 	

	properties (Hidden)
    DelayProfileType = 'charPopup'
    DelayProfileDropDown
    DelayProfileLabel
    DelayProfileGUI

    DelaySpreadType = 'numericEdit'
    DelaySpreadLabel
    DelaySpreadGUI

    KFactorScalingType = 'checkbox'
    KFactorScalingLabel
    KFactorScalingGUI

    KFactorFirstTapType = 'numericEdit'
    KFactorFirstTapLabel
    KFactorFirstTapGUI

    PathGainSampleRateDropDown = {'signal', 'auto'}

    % patches for Waterfall plot
    pWaterfallTimingPatch
    pWaterfallCyclicPrefixPatch

    % Cached path delays/power, needed for preset profiles
    ActualPathDelays
    ActualAveragePowers
  end

  methods (Static)
    function hPropDb = getPropertySet(~)
      hPropDb = extmgr.PropertySet(...
         'Visualizations',   'mxArray', {'Power Delay Profile', 'Impulse Response', 'OFDM Response'});
    end
  end

  methods % constructor

    function [visualNames, visualTags] = getVisualNamesAndTags(obj)
      % custom visuals (not webscopes):
      propSet = obj.getPropertySet();
      if ~isempty(propSet.findProperty('Visualizations'))
        visualNames = propSet.getPropValue('Visualizations');
        visualTags = {'powerDelayProfile', 'waterfall', 'channelResponse'};
      else
        visualNames = {};
      end
    end

    function obj = TDLDialog(parent)
        obj@wirelessChannelDesigner.MIMOChannelConfigurationDialog(parent); % call base constructor
        obj@wirelessChannelDesigner.MIMOFadingDialog(parent); % call base constructor

        className = 'wirelessChannelDesigner.TDLSignalParameters';
        if ~isKey(obj.Parent.DialogsMap, className)
          obj.Parent.DialogsMap(className) = eval([className '(obj.Parent)']); %#ok<*EVLDOT> 
        end
        className = 'wirelessChannelDesigner.TDLMIMODialog';
        if ~isKey(obj.Parent.DialogsMap, className)
          obj.Parent.DialogsMap(className) = eval([className '(obj.Parent)']); %#ok<*EVLDOT> 
        end

        % dropdown callbacks for conditional visibility:
        obj.DelayProfileGUI.(obj.Callback)              = @(a,b) profileChanged(obj, []);
        obj.KFactorScalingGUI.(obj.Callback)            = @(a,b) kScalingChanged(obj, []);
        obj.DelaySpreadGUI.(obj.Callback)               = @(a,b) delaySpreadChanged(obj, []);
        obj.KFactorFirstTapGUI.(obj.Callback)           = @(a,b) kFactorFirstTapChanged(obj, []);

        obj.PathGainSampleRateGUI.(obj.Callback)        = @(a,b) pathGainRateChanged(obj, []);
        obj.NormalizePathGainsGUI.(obj.Callback)        = @(a,b) autoAnalyze(obj);
    end

    function adjustSpec(obj)
      obj.TitleString = getMsgString(obj, 'TDLTitle');
      obj.configFcn = @nrTDLChannel;
      obj.configGenFcn = @nrTDLChannel;
      obj.configGenVar = 'tdlChan';

      % fetch supported profiles from programmatic object
      c = nrTDLChannel; 
      profileSet = c.DelayProfileSet.getAllowedValues;
      obj.DelayProfileDropDown = profileSet(~contains(profileSet,'NTN'));
      
      % different type than comm MIMO:
      obj.PathGainSampleRateType = 'charPopup';
      obj.FadingTechniqueType = 'charText';
    end
    
    function helpCallback(~)
      helpview('5g', 'channelDesigner-app');
    end

    function pathGainRateChanged(obj, ~)
      autoAnalyze(obj);
    end

    function setupDialog(obj)
      setupDialog@wirelessChannelDesigner.MIMOChannelConfigurationDialog(obj);
      obj.FadingTechnique = 'Sum of sinusoids';
      setEnable(obj, 'FadingTechnique', false);

      % init visibility:
      profileChanged(obj);
      kScalingChanged(obj);
      fadingTechniqueChanged(obj);

      className = 'wirelessChannelDesigner.TDLMIMODialog';
      mimoDialog = obj.Parent.DialogsMap(className);
      setupDialog(mimoDialog); % SpatialCorrelationSpecification, if it stays
    end

    function postLayoutActions(obj)
      updateVisuals(obj); 
    end
    
    function updateDisabled(obj)
      % ensures GUI disabled after path-gain generation is complete
      setEnable(obj, 'FadingTechnique', false);
    end
     
    function outro(obj, ~)
      setEnable(obj, 'FadingTechnique', true);
    end

    function props = displayOrder(~)
      props = {'DelayProfile'; 'DelaySpread'; 'PathDelays'; 'AveragePathGains'; 'FadingDistribution'; ...
        'KFactorFirstTap'; 'KFactorScaling'; 'KFactor'; 'MaximumDopplerShift'; 'PathGainSampleRate'; 
        'FadingTechnique'; 'NumSinusoids'; 'InitialTime'; 'NormalizePathGains'; 'RandomStream'; 'Seed'; };
    end

    function restoreDefaults(obj)
      chan = feval(obj.configFcn);
      
      props = displayOrder(obj);
      props2exclude = {'DirectPathDopplerShift', 'DirectPathInitialPhase', 'FadingTechnique', 'PathGainSampleRate'};
      for idx = 1:numel(props)
        prop = props{idx};
        if ~any(strcmp(prop, props2exclude))
          obj.(prop) = chan.(prop);
        end
      end

      obj.PathGainSampleRate = 'auto'; % different than programmatic obj
    end

    function sr = getSignalSampleRate(obj)
      sr = nan;
      className = 'wirelessChannelDesigner.TDLSignalParameters';
      if isKey(obj.Parent.DialogsMap, className)
        dialog = obj.Parent.DialogsMap(className);
        sr = getSampleRate(dialog);
      end
    end

    function cfgFading = getConfiguration(obj)
      % need to combine configurations from multiple panels into a single
      % config object

      release(obj.SysObj); % to allow customizations

      % 1. First, get properties corresponding to this panel (TDL Fading configuration):
      cfgFading = getConfiguration@wirelessChannelDesigner.ChannelConfigurationDialog(obj);
      
      % 2. Get Sample Rate from carrier parameters:
      className = 'wirelessChannelDesigner.TDLSignalParameters';
      if ~isKey(obj.Parent.DialogsMap, className)
        return; % App must be initializing
      end
      cfgFading.SampleRate = getSampleRate(obj.Parent.DialogsMap(className));

      % 3. Get MIMO properties from the MIMO configuration dialog (in a separate object):
      className = 'wirelessChannelDesigner.TDLMIMODialog';
      tdlMIMO = obj.Parent.DialogsMap(className);
      cfgMIMO = getConfiguration(tdlMIMO);

      % then combine the 2 objects:
      props = displayOrder(tdlMIMO);
      for idx = 1:numel(props)
        prop = props{idx};
        if ~any(strcmp(prop, props2ExcludeFromConfig(tdlMIMO) ))
          cfgFading.(prop) = cfgMIMO.(prop);
        end
      end

      % No filtering in the App
      cfgFading.ChannelFiltering = false;

      % NumSamples based on NumSlots (in Analysis Settings) and Signal Sample Rate:
      cfgFading.NumTimeSamples = getNumSamples(obj);
    end

    function cfg = getConfigurationForSaveSession(obj)
      cfg.Channel = getConfiguration(obj);
      cfg.Carrier = getCarrier(obj);
    end
    function carrier = getCarrier(obj)
      className = 'wirelessChannelDesigner.TDLSignalParameters';
      dlg = obj.Parent.DialogsMap(className);
      if dlg.SCS == 60
        carrier = nrCarrierConfig('NSizeGrid', dlg.NRB, 'SubcarrierSpacing', dlg.SCS, 'CyclicPrefix', dlg.ECP);
      else
        carrier = nrCarrierConfig('NSizeGrid', dlg.NRB, 'SubcarrierSpacing', dlg.SCS);
      end
    end

    function nsamp = getNumSamples(obj)
      stopTime = getImpulseStopTime(obj); % time in sec based on NumSlots
      nsamp = max(1, ceil(stopTime*getSignalSampleRate(obj)));
    end
    function r = getPathGainRate(obj)
      if strcmp(obj.PathGainSampleRate, 'signal')
        className = 'wirelessChannelDesigner.TDLSignalParameters';
        dlg = obj.Parent.DialogsMap(className);
        r = dlg.SampleRateInfo;
      else % 'auto'
        % r = getPathGainRate@wirelessChannelDesigner.MIMOFadingDialog(obj);
        r = 2*64*getMaximumDopplerShift(obj);
      end
    end
    function applyConfiguration(obj, cfg)
      % take a programmatic configuration, as an nrTDLChannel, and spread
      % it to the GUI accordingly
      
      % 1) fading properties
      applyConfiguration@wirelessChannelDesigner.ChannelConfigurationDialog(obj, cfg);
      profileChanged(obj);

      % 2) mimo properties
      mimoDialog = 'wirelessChannelDesigner.TDLMIMODialog';
      if ~isKey(obj.Parent.DialogsMap, mimoDialog)
        return;
      end
      mimoDialog = obj.Parent.DialogsMap(mimoDialog);
      applyConfiguration(mimoDialog, cfg);

      % 3) if sample rate doesn't match current carrier config, then warn
      className = 'wirelessChannelDesigner.TDLSignalParameters';
      dlg = obj.Parent.DialogsMap(className);
      if cfg.SampleRate ~= dlg.SampleRateInfo
        obj.warnInStatusBar( getMsgString(obj, 'SampleRateMismatch', num2str(cfg.SampleRate/1e6), num2str(dlg.SampleRateInfo/1e6)) )
      end
    end
    function applyConfigurationFromSave(obj, cfg)
      className = 'wirelessChannelDesigner.TDLSignalParameters';
      dlg = obj.Parent.DialogsMap(className);
      dlg.NRB = cfg.Carrier.NSizeGrid;
      dlg.SCS = cfg.Carrier.SubcarrierSpacing;
      dlg.ECP = cfg.Carrier.CyclicPrefix;

      applyConfiguration(obj, cfg.Channel);
    end

    function [pg, pp] = getPathDelayAndPowers(obj)
      % return values cached last, after a setup()/info()
      pg = obj.ActualPathDelays;
      pp = obj.ActualAveragePowers;
    end

    function [Nt, Nr] = getNumTxAndRx(obj)
      % get numTx / numRx antennas from underlying comm.MIMOChannel object
      cfg = getConfiguration(obj);
      setup(cfg);
      mimoChan = getMIMOChannel(cfg);
      [Nt, Nr] = getNumTxAndRx(mimoChan);
    end

    function customVisualizations(obj, ~)
      % called when an analysis selection toggles in the gallery
      updateVisuals(obj);
    end

    function setupCustomVisualizations(obj)
      % called right before analsis / path-gain generation

      visual = 'Power Delay Profile';
      if obj.getVisualState(visual)
        setupPowerDelayProfile(obj);
      end

      visual = 'Impulse Response';
      if obj.getVisualState(visual)
        setupIRWaterfall(obj);
      end

      visual = 'OFDM Response';
      if obj.getVisualState(visual)
        setupChannelResponse(obj);
      end

      % Get & cache underlying pathDelays/averageGains
      chanObj = getConfiguration(obj); % this is an nrTDLChannel
      s = info(chanObj);
      obj.ActualPathDelays = s.PathDelays;
      obj.ActualAveragePowers = s.AveragePathGains;
    end
    function stepPathGainVisualizations(obj, pathGains, sampleTimes)
      % called by nrTDLChannel.step()

      updateNumTxRx(obj); % in case old values are no longer supported
      txAnt = getTxAntenna(obj);
      rxAnt = getRcvAntenna(obj);

      visual = 'Impulse Response';
      if obj.getVisualState(visual)
        stepIRWaterfall(obj, pathGains(:, :, txAnt, rxAnt), sampleTimes);
      end

      visual = 'OFDM Response';
      if obj.getVisualState(visual)
        stepChannelResponse(obj, pathGains(:, :, txAnt, rxAnt), sampleTimes);
      end
    end

    function [timingEstimate, cpDelayLimit] = getOverlayValues(obj, newGains)
      % given a set of path gains, compute the Perfect Timing Estimate, and
      % CP Delay Limit, in seconds

      className = 'wirelessChannelDesigner.TDLSignalParameters';
      dialog = obj.Parent.DialogsMap(className);
      nrb = dialog.NRB;
      scs = dialog.SCS;
      ecp = dialog.ECP;
      
      timingEstimate = (nrPerfectTimingEstimate(newGains, obj.pCoefficients) - obj.pChannelDelay)/obj.getSignalSampleRate();

      ofdmInfo = nrOFDMInfo(nrb, scs, "CyclicPrefix", ecp);
      cpDelayLimit = timingEstimate + ofdmInfo.CyclicPrefixLengths(2)/obj.getSignalSampleRate();
    end
    function stepIRWaterfall(obj, newGains, sampleTimes)
      % base method plots the 3D surf of path gains
      ax = stepIRWaterfall@wirelessChannelDesigner.MIMOChannelConfigurationDialog(obj, abs(newGains), sampleTimes);
        
      % 5G-side plots patches for Perfect Timing Estimate, CP Delay Limit
      [offset, cpLimit] = getOverlayValues(obj, newGains);
                
      opac = obj.Opacity;
      
      if size(obj.pCoefficients, 1) > 1 % 2D plot without delay axes
        if isequal(ax.View, [0 90]) % 2D plot
          plot(ax, offset*ones(1, 2),  ax.YLim, 'Color', obj.Red, 'Tag', 'timingEstimate');
          plot(ax, cpLimit*ones(1, 2), ax.YLim, 'Color', obj.Green, 'Tag', 'cpDelayLimit');
        else 
          % 3D plot
          maxTime = max(sampleTimes);
          if isempty(obj.pWaterfallTimingPatch) || ~ishghandle(obj.pWaterfallTimingPatch)
            % create patches for 1st time
            obj.pWaterfallTimingPatch = fill3(ax, offset*ones(1, 4), ...
              [maxTime 0 0 maxTime], ...
              [ax.ZLim(1)*ones(1, 2) ax.ZLim(end)*ones(1, 2)], ...
              obj.Red, 'EdgeColor', obj.Red, 'FaceAlpha', opac, ...
              'EdgeAlpha', opac, 'LineWidth', 2, 'Tag', 'timingEstimate');
  
            obj.pWaterfallCyclicPrefixPatch = fill3(ax, cpLimit*ones(1, 4), ...
              [maxTime 0 0 maxTime], ...
              [ax.ZLim(1)*ones(1, 2) ax.ZLim(end)*ones(1, 2)], ...
              obj.Green, 'EdgeColor', obj.Green, 'FaceAlpha', opac, ...
              'EdgeAlpha', opac, 'LineWidth', 2, 'Tag', 'cpDelayLimit');
          else
            % update position of previously created patches, while more data
            % have been plotted and axes have grown
            set([obj.pWaterfallTimingPatch obj.pWaterfallCyclicPrefixPatch], ...
              'YData', [maxTime 0 0 maxTime], ...
              'ZData', [ax.ZLim(1)*ones(1, 2) ax.ZLim(end)*ones(1, 2)]);
          end
        end
        signalNames = {getString(message('comm:channelDesigner:ChannelImpulseResponse')), ...
          getMsgString(obj, 'PerfectTimingEstimate'), getMsgString(obj, 'CyclicPrefixDelayLimit')};
      else
        signalNames = {getString(message('comm:channelDesigner:ChannelImpulseResponse'))};
      end

      legend(ax, signalNames, ...
          'Color', obj.Black, 'EdgeColor', obj.White, 'TextColor', obj.White, 'Location', 'northeast');
    end

    function stepChannelResponse(obj, newGains, sampleTimes)
      visual = 'OFDM Response';
      fig = obj.getVisualFig(visual);
      ax = fig.CurrentAxes;

      % same as in nrTDLChannel.step():
      toffset = channelDelay(newGains, obj.pCoefficients.');
      ofdminfo = nr5g.internal.OFDMInfo(getCarrier(obj),[]);
      nslot = getNumSlots(obj);
      ofdmSymbolCenter = nr5g.internal.OFDMSampleTimesIndices(ofdminfo,nslot,sampleTimes,getNumSamples(obj),toffset);
      newGains = newGains(ofdmSymbolCenter,:,:,:);
      H = nr5g.internal.OFDMChannelResponse(ofdminfo,newGains,obj.pCoefficients,toffset);
      
      surf(ax, (1:size(H, 2)), 1:size(H, 1), mag2db(abs(H)));
      shading(ax, 'flat');

      xlabel(ax, getMsgString(obj, 'OFDMSymbolsLabel'));
      ylabel(ax, getMsgString(obj, 'SubcarriersLabel'));
      zlabel(ax, getString(message('comm:channelDesigner:MagnitudeDBLabel')));

      legend(ax, 'off');
    end

    function gains = generatePathGains(obj)
      obj.Parent.AppObj.pThrewValidationError = false;

      chanObj = getConfiguration(obj); % this is an nrTDLChannel

      if obj.Parent.AppObj.pThrewValidationError
        gains = [];
        % thrown from validations within getConfiguration
        return
      end

      % cache needed info post-setup:
      obj.pCoefficients = getPathFilters(chanObj);
      s = info(chanObj);
      obj.pChannelDelay = s.ChannelFilterDelay;

      % generate path gains
      [gains, sampleTimes] = chanObj(); % step 
      appObj = obj.Parent.AppObj;
      appObj.updateProgressBar(100); % one-shot for now
      appObj.pExport2File.Enabled = true;
      appObj.pExport2WS.Enabled = true;

      % TDL-specific visualizations (post path gain generation)
      stepPathGainVisualizations(obj, gains, sampleTimes);
    end

    function props = props2ExcludeFromConfig(obj)
      props = {'DirectPathDopplerShift', 'DirectPathInitialPhase', 'FadingTechnique'};
      isCustom = strcmp(obj.DelayProfile, 'Custom');
      if ~isCustom
        props = [props, 'PathDelays', 'AveragePathGains', 'FadingDistribution'];
      end
      if ~isCustom || ~strcmp(obj.FadingDistribution, 'Rician')
        props = [props 'KFactorFirstTap'];
      end
      delaySpreadShows = any(strcmp(obj.DelayProfile, { 'TDL-A', 'TDL-B', 'TDL-C', 'TDL-D', 'TDL-E'}));
      if ~delaySpreadShows
        props = [props, 'DelaySpread'];
      end
      if ~any(strcmp(obj.DelayProfile, {'TDL-D', 'TDL-E'}))
        props = [props, 'KFactorScaling'];
      end
      if ~any(strcmp(obj.DelayProfile, {'TDL-D', 'TDL-E'})) || ~obj.KFactorScaling
        props = [props, 'KFactor'];
      end

      if strcmp(obj.RandomStream, 'Global stream')
        props = [props, 'Seed'];
      end
    end
  
    function profileChanged(obj, ~)
      isCustom = strcmp(obj.DelayProfile, 'Custom');
      setVisible(obj, {'PathDelays', 'AveragePathGains', 'FadingDistribution'}, isCustom);

      isRician = strcmp(obj.FadingDistribution, 'Rician');
      setVisible(obj, 'KFactorFirstTap', isCustom && isRician);

      delaySpreadShows = any(strcmp(obj.DelayProfile, { 'TDL-A', 'TDL-B', 'TDL-C', 'TDL-D', 'TDL-E'}));
      setVisible(obj, 'DelaySpread', delaySpreadShows);

      isDorE = any(strcmp(obj.DelayProfile, {'TDL-D', 'TDL-E'}));
      setVisible(obj, 'KFactorScaling', isDorE);
      setVisible(obj, 'KFactor', isDorE && obj.KFactorScaling);

      layoutUIControls(obj);

      delaysChanged(obj); % possible update multipath components in Analysis Settings
      % autoAnalyze(obj); % called by delays changed
    end

    function delaysChanged(obj, ~)
      delaysChanged@wirelessChannelDesigner.MIMOFadingDialog(obj, []); % validation

      autoAnalyze(obj);
    end
    function gainsChanged(obj, ~)
      gainsChanged@wirelessChannelDesigner.MIMOFadingDialog(obj, []); % validation

      autoAnalyze(obj);
    end

    function fadingDistChanged(obj, ~)
      isRician = strcmp(obj.FadingDistribution, 'Rician');
      isCustom = strcmp(obj.DelayProfile, 'Custom');
      setVisible(obj, 'KFactorFirstTap', isCustom && isRician);

      layoutUIControls(obj);

      autoAnalyze(obj);
    end

    function kScalingChanged(obj, ~)
      isDorE = any(strcmp(obj.DelayProfile, {'TDL-D', 'TDL-E'}));
      setVisible(obj, 'KFactor', isDorE && obj.KFactorScaling);

      layoutUIControls(obj);

      autoAnalyze(obj);
    end

    function delaySpreadChanged(obj, ~)
      try
        val = obj.DelaySpread;
        validateattributes(val, {'numeric'}, {'nonnegative', 'real', 'scalar'}, '', 'delay spread');
      catch e
        obj.errorFromException(e);
      end
      autoAnalyze(obj);
    end
    function kFactorFirstTapChanged(obj, ~)
      try
        val = obj.KFactorFirstTap;
        validateattributes(val, {'numeric'}, {'real', 'scalar'}, '', 'K factor of 1st tap');
      catch e
        obj.errorFromException(e);
      end
      autoAnalyze(obj);
    end
    function kChanged(obj, ~)
      % KFactor in NR TDL is in dB, so we must allow negative values, thus
      % override this validation:
      try
        val = obj.KFactor;
        validateattributes(val, {'numeric'}, {'real', 'scalar'}, '', 'K factor');
      catch e
        obj.errorFromException(e);
      end
    end

    function cellDialogs = getDialogsPerColumn(obj)
      % Panels that show in left-side Channel Design figure panel
      cellDialogs{1} = {obj ...
        obj.Parent.DialogsMap('wirelessChannelDesigner.TDLMIMODialog') ...
        obj.Parent.DialogsMap('wirelessChannelDesigner.TDLSignalParameters')};
    end
    function settingDialogs = getAnalysisSettingsDialogs(~)
      % Panels that show when Analysis Settings is clicked
      settingDialogs = {'wirelessChannelDesigner.CommonSettings', 'wirelessChannelDesigner.NRSettings'};
    end    

    function b = offersVisual(obj, tag)
      % determines presence of entry in analysis gallery
      b = contains(tag, obj.visualNames);
    end
    
    function b = visualOnByDefault(~, tag)
      % called by base setupDialog() upon initialization, to establish
      % pressed button in analysis gallery
      onTags = {'powerDelayProfile', 'channelResponse'};
      b = any(strcmp(tag, onTags));
    end

    function defaultVisualLayout(obj)
      % Called by Default Layout button
      obj.setVisualState('Power Delay Profile', true);
      obj.setVisualState('Impulse Response', false);
      obj.setVisualState('OFDM Response', true);
    end

    function b = spectrumEnabled(~)
      % don't buy Spectrum Analyzer (1D Frequency Response) here
      b = false;
    end

    function staticVisualizations(obj)
      % Visualizations that can realize before analysis / path gain generation
      if obj.getVisualState('Power Delay Profile')
        stepPowerDelayProfile(obj);
      end
    end
    
    function numSlots = getNumSlots(obj)
      % Get Number of slots from Analysis Settings
      className = 'wirelessChannelDesigner.NRSettings';
      if isKey(obj.Parent.DialogsMap, className)
        dlg = obj.Parent.DialogsMap(className);
        numSlots = dlg.Duration; % in slots
      else % Analysis Settings button was never clicked.
        % default numSlots is 1
        numSlots = 1;
      end
    end

    function t = getImpulseStopTime(obj)
      numSlots = getNumSlots(obj);

      % find numerology
      className = 'wirelessChannelDesigner.TDLSignalParameters';
      dlg = obj.Parent.DialogsMap(className);
      scs = dlg.SCS;

      % 1 ms per slot for SCS = 15 kHz, half for 30 kHz etc..
      t = numSlots * 1e-3 * 15/scs;
    end

    function b = canAutoAnalyze(~)
      b = true;
    end
  
    function autoAnalyze(obj)

      appObj = obj.Parent.AppObj;
      if appObj.AppContainer.Busy || ~appObj.pAutoAnalyze.Enabled || ~appObj.pAutoAnalyze.Value
        return;
      end

      resetAllAxes(obj);
      
      obj.Parent.AppObj.setStatus(getString(message('comm:channelDesigner:AnalyzingChannel', obj.Parent.AppObj.pCurrentExtensionType)));

      try
        cfg = getConfiguration(obj);
        setup(cfg);
        cfgMIMO = getMIMOChannel(cfg);
        validateProperties(cfgMIMO);

      catch exc
        warnInTopVisual(obj, exc);
        warnInStatusBar(obj, exc);
        return;
      end
     
      autoAnalyze@wirelessChannelDesigner.MIMOFadingDialog(obj);
    end

    function addConfigCode(obj, sw)
      % Export to ML Script

      className = 'wirelessChannelDesigner.TDLSignalParameters';
      signalParameters = obj.Parent.DialogsMap(className);
      addConfigCode(signalParameters, sw);

      addConfigCode@wirelessChannelDesigner.ChannelConfigurationDialog(obj, sw);
      addcr(sw, [obj.configGenVar '.ChannelFiltering = false;']);
      addcr(sw, [obj.configGenVar '.SampleRate = Fs;']);
      nsamp = getNumSamples(obj);
      numSlots = round( getNumSamples(obj) / (getSignalSampleRate(obj) * 1e-3 * 15/signalParameters.SCS) );
      
      addcr(sw, [obj.configGenVar '.NumTimeSamples = ' num2str(round(nsamp/numSlots)) ';']);
      
      addcr(sw);

      className = 'wirelessChannelDesigner.TDLMIMODialog';
      tdlMIMO = obj.Parent.DialogsMap(className);
      addConfigCode(tdlMIMO, sw);
    end
    function [sampPerSlot, numSlots] = splitDuration2Steps(obj)

      nsamp = getNumSamples(obj);
      numSlots = getNumSlots(obj);
      sampPerSlot = round(nsamp/numSlots);
    end

    function str = getCatalogPrefix(~)
      str = 'nr5g:channelDesignerApp:';
    end
  end
end