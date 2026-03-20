function ptrssymbolset = ptrsSymbolIndicesCPOFDM(symbolset,dmrssymbolset,lptrs)
%ptrsSymbolIndicesCPOFDM PT-RS OFDM symbol indices
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   PTRSSYMBOLSET = ptrsSymbolIndicesCPOFDM(SYMBOLSET,DMRSSYMBOLSET,LPTRS)
%   returns the 0-based PT-RS OFDM symbol locations PTRSSYMBOLSET, given
%   the inputs, set of OFDM symbols allocated for shared channel SYMBOLSET,
%   DM-RS symbol locations DMRSSYMBOLSET and PT-RS time density LPTRS.
%
%   Example:
%   % Get the PT-RS symbol locations in the physical shared channel symbol
%   % allocation of 0 to 13 with DM-RS symbols locations at 2 and time
%   % density of PT-RS set to 2.
%
%   symbolset = 0:13;
%   dmrssymbolset = 2;
%   lptrs = 2;
%   ptrssymbolset = nr5g.internal.pxsch.ptrsSymbolIndicesCPOFDM(symbolset,dmrssymbolset,lptrs)

%   Copyright 2019 The MathWorks, Inc.

%#codegen

    % PT-RS OFDM symbol indices
    if ~isempty(dmrssymbolset) && ~isempty(symbolset)
        % Bounds of allocated symbols
        [lb,ub] = bounds(symbolset);

        % If DM-RS symbols is not empty
        ptrssymbolset = lb:lptrs:dmrssymbolset(1)-1;
        for i = 1:numel(dmrssymbolset)
            ptrssymbolset(ptrssymbolset >= dmrssymbolset(i)) = [];
            ptrssymbolset = [ptrssymbolset (dmrssymbolset(i)+lptrs):lptrs:ub]; %#ok<AGROW>
        end

        % For non-standard set-ups, only return the PT-RS symbol indices
        % that overlap the actual allocation indices
        temp = zeros(1,ub+1);
        temp(symbolset+1) = 1;
        temp(ptrssymbolset+1) = temp(ptrssymbolset+1) + 1;
        ptrssymbolset = find(temp==2)-1;
    else
        ptrssymbolset = zeros(1,0);
    end

end
