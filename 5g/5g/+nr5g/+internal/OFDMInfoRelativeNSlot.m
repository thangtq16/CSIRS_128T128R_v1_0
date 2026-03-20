function [cpLengths,symbolPhases] = OFDMInfoRelativeNSlot(info,nSlot,N)
%OFDMInfoRelativeNSlot OFDM information w.r.t. relative NSlot
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

    if ~isempty(nSlot)
        nSlot = double(nSlot);
        
        slotsPerFrame = info.SlotsPerSubframe * 10;
        nFrame = 0; % frame number does not affect OFDM information
        nSlot = nr5g.internal.getRelativeNSlotAndSFN(nSlot,nFrame,slotsPerFrame);
        
        nSubframes = ceil(N / length(info.CyclicPrefixLengths));
        nSlot = mod(nSlot,info.SlotsPerSubframe);
        
        cpLengths = repmat(info.CyclicPrefixLengths,1,nSubframes);
        cpLengths = circshift(cpLengths,-nSlot*info.SymbolsPerSlot);
        
        if ~isempty(info.SymbolPhases)
            symbolPhases = repmat(info.SymbolPhases,1,nSubframes);
            symbolPhases = circshift(symbolPhases,-nSlot*info.SymbolsPerSlot);
        else
            symbolPhases = zeros(1,N);
        end
    else
        cpLengths = info.CyclicPrefixLengths;
        symbolPhases = info.SymbolPhases;
    end

end
