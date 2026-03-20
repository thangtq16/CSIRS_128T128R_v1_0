function minE = getMinUCIBitCapacity(A)
%getMinUCIBitCapacity Get minimum bit capacity for UCI encoding
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   MINE = nr5g.internal.getMinUCIBitCapacity(A) gets the minimum bit
%   capacity MINE for UCI encoding, given the UCI bit length A.

%   Copyright 2021-2022 The MathWorks, Inc.

%#codegen

    if A<=11              % Small block lengths
        if A==1
            minE = 0;     % Allows no coding for qm=1
        else
            minE = A;
        end
    elseif A<=19 % Parity check (PC) polar encoding
        minE = A+6+3;     % A+crcLen+nPC
    else                  % CRC-aided (CA) polar encoding
        crcLen = 11;
        if (A<1013)
            % One code block segment, A + crcLen
            minE = A + crcLen;
        else
            % Two code block segments, A + padding bit if 'A' is odd +
            % crcLen x 2
            minE = A + mod(A,2) + (crcLen * 2);
        end
    end

end