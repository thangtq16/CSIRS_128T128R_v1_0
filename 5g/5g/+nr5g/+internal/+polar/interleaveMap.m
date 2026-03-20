function pi = interleaveMap(K)
%interleaveMap Interleaver mapping pattern for Polar coding
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   pi = nr5g.internal.polar.interleaveMap(K) returns the interleaving
%   pattern for a length K.
% 
%   See also nrPolarEncode, nrPolarDecode.

%   Copyright 2018-2023 The MathWorks, Inc.

%#codegen

%   Reference:
%   [1] 3GPP TS 38.212, "3rd Generation Partnership Project; Technical 
%   Specification Group Radio Access Network; NR; Multiplexing and channel 
%   coding (Release 15). Section 5.3.1.1.

    Kilmax = 164;
    pat = nr5g.internal.polar.getInterleavePattern();
    pi = zeros(K,1);
    k = 0;
    for m = 0:Kilmax-1
        if pat(m+1) >= Kilmax-K
            pi(k+1) = pat(m+1)-(Kilmax-K);
            k = k+1;
        end
    end

end
