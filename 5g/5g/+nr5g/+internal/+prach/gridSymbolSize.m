function L = gridSymbolSize(prach)
%gridSymbolSize Number of OFDM symbols in a PRACH slot resource grid
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2022 The MathWorks, Inc.

%#codegen

    switch (prach.Format)
        case {'0','1','2','3'}
            % For long sequences (LRA=839) the number of OFDM symbols in
            % the PRACH slot grid is the number of OFDM symbols in one
            % PRACH preamble, because there is only one PRACH time occasion
            % per PRACH slot
            L = prach.PRACHDuration;
        otherwise 
            % For short sequences (LRA=139,571,1151) the number of OFDM
            % symbols in the PRACH slot grid is 14
            L = 14;
    end

    if (strcmpi(prach.Format,'C0'))
        % In the case of format C0, each preamble has one active sequence
        % period (see TS 38.211 Table 6.3.3.1-2) but including the guard
        % and the cyclic prefix, the preamble spans two OFDM symbols. For
        % this reason, the slot grid related to format C0 has 7 OFDM
        % symbols, rather than 14, and each value related to it that is
        % derived directly from TS 38.211 is halved.
        L = L / 2;
    end

end
