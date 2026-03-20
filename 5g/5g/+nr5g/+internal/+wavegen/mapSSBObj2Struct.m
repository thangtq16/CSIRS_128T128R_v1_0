function ssbStruct = mapSSBObj2Struct(ssbObj, carriers)
% This is an internal, undocumented function that can change anytime. It is
% currently used to support the SS-based utilities (hSSBurstInfo, hSSBurst, ssBurstResources). 

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

  ssbStruct.Enable                    = ssbObj.Enable;
  ssbStruct.Power                     = ssbObj.Power;
  ssbStruct.BlockPattern              = ssbObj.BlockPattern;
  ssbStruct.TransmittedBlocks         = ssbObj.TransmittedBlocks;
  ssbStruct.Period                    = ssbObj.Period;
  ssbStruct.DataSource                = ssbObj.DataSource;
  ssbStruct.DMRSTypeAPosition         = ssbObj.DMRSTypeAPosition;
  ssbStruct.CellBarred                = ssbObj.CellBarred;
  ssbStruct.IntraFreqReselection      = ssbObj.IntraFreqReselection;
  ssbStruct.PDCCHConfigSIB1           = ssbObj.PDCCHConfigSIB1;
  ssbStruct.SubcarrierSpacingCommon   = ssbObj.SubcarrierSpacingCommon;

  % Set Frequency Point A using carrier with highest SCS
  carrierscs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers,'SubcarrierSpacing', 'double');
  [maxscs, maxIdx] = max(carrierscs);
  ssbStruct.FrequencyPointA = -(carriers{maxIdx}.NSizeGrid/2 + carriers{maxIdx}.NStartGrid)*12*maxscs*1e3;
  
  % define these to help codegen
  ssbStruct.SubcarrierSpacing = NaN;  
  ssbStruct.SampleRate = NaN;         
  ssbStruct.NCellID = 0;
  ssbStruct.NHalfFrame = 0;
  ssbStruct.NFrame = 0;

  % Determine frequency placement of SS Burst
  
  % First determine frequency limits of carriers (in Hz), relative to PointA
  minFreq = inf;
  maxFreq = 0;
  ssbMin = inf;
  ssbMax = 0;
  
  [ssbSCS,kssbUnit,ncrbUnit] = nr5g.internal.wavegen.blockPattern2SCS(ssbObj.BlockPattern,ssbObj.SubcarrierSpacingCommon);

  % The waveform center is the same as the center of the highest SCS carrier:
  carrierscs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers, 'SubcarrierSpacing');
  maxSCS = max(carrierscs);
 
  for idx = 1:length(carriers)
      thisMin = 1e3 * 12 * carriers{idx}.SubcarrierSpacing * carriers{idx}.NStartGrid;
      thisMax = 1e3 * 12 * carriers{idx}.SubcarrierSpacing * (carriers{idx}.NStartGrid + carriers{idx}.NSizeGrid);

      if carriers{idx}.SubcarrierSpacing == maxSCS
          minFreq = thisMin;
          maxFreq = thisMax;
      end
      if carriers{idx}.SubcarrierSpacing == ssbSCS
          ssbMin = thisMin;
          ssbMax = thisMax;
      end
  end
  % calculate waveformCenter, because FrequencySSB specification is
  % relative to that
  waveformCenter = minFreq + (maxFreq-minFreq)/2;

  if isempty(ssbObj.NCRBSSB)
      % automatic placement at the center
      ssbStruct.FrequencySSB = ssbMin(1)+(ssbMax(1)-ssbMin(1))/2 - waveformCenter(1);
  else
      % custom placement from Point A
      ssbWidth = 20; % RB
      ssbStruct.FrequencySSB = 1e3 * 12 * ncrbUnit  * ssbObj.NCRBSSB(1) + ...
                               1e3 * 12 * ssbSCS(1) * ssbWidth/2 + ...
                               1e3 * kssbUnit * ssbObj.KSSB(1) - waveformCenter(1);
  end
end
