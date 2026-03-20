function bwp = linkSCS2BWP(waveconfig)
%

%   Copyright 2019-2022 The MathWorks, Inc.

% The waveform generator performs this function internally therein.
% Nevertheless, it is also needed by the resource-grid plotters, outside of
% the waveform-generation call, thus the need for a standalone utility.

bwp = waveconfig.BWP;
carrierscs = [waveconfig.Carriers.SubcarrierSpacing];
for bp=1:length(waveconfig.BWP)
  % Map it into a SCS specific carrier level RE grid
  cidx = find(bwp(bp).SubcarrierSpacing == carrierscs,1);
  % assert(~isempty(cidx),'A SCS specific carrier configuration for SCS = %d kHz has not been defined. This carrier definition is required for BWP %d.',bwp(bp).SubcarrierSpacing,bp);
  % Record the carrier index associated with the BWP
  bwp(bp).CarrierIdx = cidx;
end
