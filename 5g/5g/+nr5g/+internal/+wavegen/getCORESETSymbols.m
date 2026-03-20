% getCORESETSymbols Get symbols associated with CORESET and search space in a BWP
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

function rmallocatedsymbols = getCORESETSymbols(numSubframes,bwp,cs,searchSpace,initnsf)

    if nargin < 5
        initnsf = 0;
    end

    % Expand the allocated slots across the repetition period
    rmallocatedSlots = nr5g.internal.wavegen.expandbyperiod(searchSpace.SlotPeriodAndOffset(2):(searchSpace.SlotPeriodAndOffset(2)+searchSpace.Duration-1),searchSpace.SlotPeriodAndOffset(1),numSubframes,bwp.SubcarrierSpacing,initnsf);
    
    % Expand to identify all symbols included in this CORESET sequence
    symbperslot = nr5g.internal.wavegen.symbolsPerSlot(bwp);
    slotsymbs = searchSpace.StartSymbolWithinSlot(searchSpace.StartSymbolWithinSlot+cs.Duration <= symbperslot);
    csetsymbols = expander(slotsymbs,cs.Duration);
    rmallocatedsymbols = nr5g.internal.wavegen.addRowAndColumn(csetsymbols,symbperslot*rmallocatedSlots);
    
end

% Expand 'd' by amount 'e', with optional non-unity strides and exclusion 
function expanded = expander(d,e,s,o,excl)
    if nargin < 5
        excl = 0;
    end
    if nargin < 4
        o = 0;
    end
    if nargin < 3
        s = 1;
    end

    if ~excl
        eseq = (o:s:e-1)';
    else
        tmp = (o:s:e-1)';
        eseq = setdiff((0:e-1)',tmp);
    end

%     expanded = reshape(reshape(d,1,[]) + eseq,1,[]);  % Use column expansion
    % Codegen-friendly version:
    tmp = zeros(length(eseq), length(d));
    for idx1 = 1:length(eseq)
        for idx2 = 1:length(d)
            tmp(idx1, idx2) = d(idx2) + eseq(idx1);
        end
    end
    expanded = tmp(:)';
end
