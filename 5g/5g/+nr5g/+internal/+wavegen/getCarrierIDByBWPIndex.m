function carrierID = getCarrierIDByBWPIndex(carriers, bwp, bwpIndex)
% This is an internal function that can change any time. This utility is
% shared between nrWaveformGenerator() and the Wireless Waveform Generator App.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

  carrierID = NaN; % init for codegen. nrDLCarrierConfig.validateConfig() ensured one exists

  scs = bwp{bwpIndex}.SubcarrierSpacing;
  for idx = 1:length(carriers)
    if carriers{idx}.SubcarrierSpacing == scs
      carrierID = idx;
    end
  end
end