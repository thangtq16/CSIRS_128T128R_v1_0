function [out,flag] = extractActiveResourceBlocks(reserved,prbset,symbolset,nslot,nslotsymb)
%extractActiveResourceBlocks Extract active resource blocks
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [OUT,FLAG] = extractActiveResourceBlocks(RESERVED,PRBSET,SYMBOLSET,NSLOT,NSLOTSYMB)
%   returns the active resource blocks OUT, given the input cell array of
%   reserved configuration objects RESERVED, vector of physical resource
%   blocks PRBSET, the OFDM symbol set SYMBOLSET, the slot number NSLOT and
%   the number of OFDM symbols per slot NSLOTSYMB. It also provides the
%   output FLAG indicating if there is any exclusion of resource blocks or
%   not. FLAG value of 1 indicates that OUT contains the PRBSET after
%   removing the reserved resource blocks.
%
%   OUT is a cell array of length equal to the number of OFDM symbols per
%   slot.
%
%   Example:
%   % Remove the resource block numbers 0,1,2 in all the symbols from the
%   % configuration with PRB set 1 to 10, symbols occupying complete slot,
%   % slot number set to 0 and symbols per slot set to 14.
%
%   nslot = 0;
%   nslotsymb = 14;
%   prbset = 1:10;
%   symbolset = 0:nslotsymb-1;
%   reserved{1}.PRBSet = [0 1 2];
%   reserved{1}.SymbolSet = symbolset;
%   reserved{1}.Period = [];
%   [out,flag] = nr5g.internal.pdsch.extractActiveResourceBlocks(reserved,prbset,symbolset,nslot,nslotsymb);
%   flag

%   Copyright 2019 The MathWorks, Inc.

%#codegen

    % Assign output cell array with PRB
    out = repmat({prbset},1,nslotsymb);
    flag = 0;
    for ri=1:length(reserved)

        % Reference the current reserved symbol/PRB indices pair
        if ~isempty(reserved{ri}.SymbolSet)
            reservedsymbols = double(reserved{ri}.SymbolSet(:));
        else
            reservedsymbols = zeros(0,1);
        end
        if ~isempty(reserved{ri}.PRBSet)
            reservedprb = double(reserved{ri}.PRBSet(:));
        else
            reservedprb = zeros(0,1);
        end

        % Find any of the allocated symbols which overlap with reserved
        % symbols
        %
        % If the reserved period is empty then get number of complete
        % slots in the reserved period and cyclically extend pattern to
        % cover current slot
        if isempty(reserved{ri}.Period)
            reservedperiod = 0;
        else
            reservedperiod = double(reserved{ri}.Period(1));
        end
        offset = mod(double(nslot),reservedperiod)*nslotsymb; % Symbol offset (whole number of slots) into pattern period
        if numel(symbolset) && ~isempty(reservedsymbols)
            % Included this check for codegen with empty allocated
            % symbols, to make sure no run-time errors were thrown
            temp = zeros(1,nslotsymb);
            temp(symbolset+1) = 1;
            % Make sure the reserved symbols contain only the symbols
            % within the reserved slot
            resSymbolsWithinSlot = reservedsymbols-offset;
            reservedSymbolSet = resSymbolsWithinSlot((resSymbolsWithinSlot<nslotsymb) & (resSymbolsWithinSlot>=0));
            temp(reservedSymbolSet+1) = temp(reservedSymbolSet+1)+1;
            inter = find(temp==2)-1;
        else
            inter = zeros(0,1);
        end

        % Reference the PRB associated with the overlapping symbols
        for i = 1:numel(inter)
            prbCellInter = out{inter(i)+1};
            intersectMatrix = repmat(out{inter(i)+1},numel(reservedprb),1) == repmat(reservedprb,1,numel(out{inter(i)+1}));
            if nnz(~sum(intersectMatrix,1)) < numel(prbCellInter)
                out{inter(i)+1} = reshape(prbCellInter(~sum(intersectMatrix,1)),1,[]);
                flag = 1;
            end
        end
    end

end