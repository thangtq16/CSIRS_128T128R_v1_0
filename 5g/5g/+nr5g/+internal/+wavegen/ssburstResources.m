function [rsvBwp,rsvCarrier,ssburstOut] = ssburstResources(ssburstIn,carriers,bwps)
% Get the set of RB level resources in each SCS carrier that overlap with
% the SS burst

%#codegen

%   Copyright 2019-2024 The MathWorks, Inc.
   
    % Initialize output
    numBwps = length(bwps);
    rsvBwp = cell(1, numBwps);
    for b = 1:numBwps
      rsvBwp{b} = nrPDSCHReservedConfig;
    end
    rsvCarrier = rsvBwp;

    % If the burst is not active then return early
    if ~ssburstIn.Enable
      return;
    end
    
    % Burst symbols in active half frame
    ssbStartSymbols = nr5g.internal.wavegen.hSSBurstStartSymbols(ssburstIn);
    ssbOccupiedSymbols = (0:3)' + ssbStartSymbols(logical(ssburstIn.TransmittedBlocks));
    ssbSymbolsInHalfFrame = reshape(ssbOccupiedSymbols,1,[]);

    % SSB frequence bands (Hz)
    ssbFreqBands = ssbFrequencyBands(carriers,ssburstIn);
    ssburstOut.SubcarrierSpacing = nr5g.internal.wavegen.blockPattern2SCS(ssburstIn.BlockPattern);

    halfFrameOffset = 0;
    if ~isscalar(ssburstIn.Period)
        halfFrameOffset = ssburstIn.Period(2);
    end

    % Calculate BWP oriented RB indices 
    for b = 1:numBwps
        bp = bwps{b};

        % Project the SS burst symbols into the BWP
        rsvBwp{b}.SymbolSet = ssbReservedSymbols(ssbSymbolsInHalfFrame,halfFrameOffset,ssburstOut.SubcarrierSpacing,bp);

        % Set SSB reservation periodicity in slots
        rsvBwp{b}.Period = ssburstIn.Period(1)*bp.SubcarrierSpacing/15;

        % Calculate BWP oriented PRBs reserved for SSB
        rsvBwp{b}.PRBSet = ssbReservedPRB(ssbFreqBands,bp.NStartBWP,bp.NSizeBWP,bp.SubcarrierSpacing);

        % Copy symbol set and period from BWP reservation
        rsvCarrier{b}.SymbolSet = rsvBwp{b}.SymbolSet;
        rsvCarrier{b}.Period = rsvBwp{b}.Period;

        % Determine the carrier associated to this BWP
        cidx = nr5g.internal.wavegen.getCarrierIDByBWPID(carriers,bwps,bp.BandwidthPartID);
        carrier = carriers{cidx};

        % Calculate the carrier oriented PRBs reserved for SSB
        rsvCarrier{b}.PRBSet = ssbReservedPRB(ssbFreqBands,carrier.NStartGrid,carrier.NSizeGrid,carrier.SubcarrierSpacing);        
        
    end
   
end

function f = ssbFrequencyBands(carriers,ssburstIn)
    
    % SSB SCS
    ssbSCS = nr5g.internal.wavegen.blockPattern2SCS(ssburstIn.BlockPattern);

    % Establish the maximum carrier SCS configured
    % The center of the overall waveform will be on k0 of this SCS carrier 
    carrierscs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers, 'SubcarrierSpacing');
    [~,maxidx] = max(carrierscs);
    
    % Relative center frequencies of first and last carriers of SS block
    bnds = [-120 120-1]*ssbSCS*1e3 + ssburstIn.FrequencySSB;
    % Center frequency of DC subcarrier of combined SCS waveform, relative to 'point A' 
    f0 = ((carriers{maxidx}.NStartGrid + fix(carriers{maxidx}.NSizeGrid/2))*12 + 6*mod(carriers{maxidx}.NSizeGrid,2))*carriers{maxidx}.SubcarrierSpacing*1e3;
    % Relative center frequencies of SS block edge carriers in waveform (max SCS carrier)
    f = f0 + bnds;  

end

function mappedSymbols = ssbReservedSymbols(ssbSymbols,halfFrameOffset,ssbSCS,bwp)
    
    ft = bwp.SubcarrierSpacing;           % 'Target' SCS
    symPerSlot = nr5g.internal.wavegen.symbolsPerSlot(bwp);
    fs = ssbSCS;  % 'Source' SCS
    scaling = (fs*14)/(ft*symPerSlot); % SSB is always defined in SCS associated to normal CP
    sym = [];
    for i = 1:length(ssbSymbols)
        s = ssbSymbols(i);
        sym = [sym floor(s/scaling):floor((s+1-eps(s))/scaling)]; %#ok<AGROW>
    end
    mappedSymbols = unique(sym) + halfFrameOffset*scaling*symPerSlot;
    
end

function prb = ssbReservedPRB(f,gridStart,gridSize,scs)

    rbLim = fix(f/(12*scs*1e3)) - gridStart;
    rb = rbLim(1):rbLim(2);
    prb = rb( (rb >=0) & (rb<gridSize) );

end